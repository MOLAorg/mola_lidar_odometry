# mola_lidar_odometry repository — AI Agent Context Guide

## Project Overview

**MOLA** (Modular Optimization framework for Localization and mApping) is a modern C++ and ROS 2 framework for robot **localization and SLAM** (Simultaneous Localization and Mapping).

This repository provides a LiDAR-Inertial Odometry (LIO) frontend for the MOLA framework. 

Official Docs: https://docs.mola-slam.org/latest/

## `ros2-lidar-odometry.launch.py`: `initial_pose` argument

Added because it was missing: `InitLocalization::FixedPose` has always
supported a known starting pose via the pipeline YAML's `MOLA_INITIAL_X/Y/Z/
YAW/PITCH/ROLL` env vars, but the ROS 2 launch file had no argument wired to
them (only `mola_footprint_to_base_link_tf`, a different thing — a static
footprint-to-base_link offset, not the localization starting pose). Use
`initial_pose:="[x, y, z, yaw_deg, pitch_deg, roll_deg]"` (same format as
`mola_footprint_to_base_link_tf`); empty (default) leaves the pipeline
YAML's own fallback (origin) in place.

## State estimator integration

Per LiDAR scan, `LidarOdometry` queries the configured `mola::NavStateFilter`
for a motion prior via `state_.navstate_fuse->estimated_navstate(scan_ref_time,
publish_reference_frame)` (`module/src/LidarOdometry_ProcessScan.cpp`), then feeds
the registered pose back with `fuse_pose()`.

The smoother's shipped `state-estimation-smoother.yaml` (loaded by this repo's
ros2 launch) defaults to its real-time path: `async_backend: true` (the per-scan
`estimated_navstate()` returns a fast prediction from the smoother's lightweight
predictor re-anchored on the last completed backend solve, while the iSAM2 window
solve runs concurrently in the smoother's own thread - bounding per-scan latency),
plus high-rate keyframe decimation (`odometry_min_sample_period` /
`imu_min_sample_period` = 0.1). Override per launch via the `MOLA_*` env vars.
For OFFLINE / reproducible batch runs set `MOLA_ASYNC_BACKEND=false` (async
serving is non-deterministic). Full parameter table: `mola_state_estimation`'s
AGENTS.md ("Real-time setup").

### REP-105 `map -> odom` published directly (jitter-free)

By default, under `publish_localization_following_rep105: true` the bridge
composes `map -> odom = (map -> base_link)(t) * (odom -> base_link)^-1` by TF
lookup. When the localizer's stamp leads the odom TF (the normal case for an
estimate extrapolated to "now"), the exact lookup misses and the bridge composes
against a stale odom transform, injecting motion-correlated `map -> odom` jitter.

To avoid the composition, the smoother can publish `map -> odom` straight from
its own `T_map_to_odom` graph variable (`publish_map_to_odom_tf: true`, method
suffix `/map_odom`), and the bridge's TF source is routed to that method via the
`localization_publish_tf_source` launch argument
(`localization_publish_tf_source:=state_estimation/map_odom`). The bridge then
forwards it verbatim (its `child_frame != base_link` path).

Minimal launch-side setup (single wheel-odom source, ROS odom frame `odom`):
```
MOLA_PUBLISH_MAP_TO_ODOM_TF=true
MOLA_MAP_TO_ODOM_FRAME=odom_wheels    # the source's sensor label
MOLA_MAP_TO_ODOM_CHILD_FRAME=odom     # the REP-105 odom /tf frame
```
plus `localization_publish_tf_source:=state_estimation/map_odom`. The primary
`map -> base_link` update still drives the pose topic
(`MOLA_LOCALIZATION_PUBLISH_ODOM_MSGS_SOURCE` stays at the estimator's method).

Frame-name gotcha: the smoother keys each odometry source by its sensor label
(the bridge subscription's `output_sensor_label`; the `nav_msgs/Odometry` topic
path defaults to `MOLA_ODOM_SENSOR_LABEL|odom_wheels`). The `map -> odom` `/tf`
child frame must equal the REP-105 odom frame the external driver publishes
`odom -> base_link` for (usually `odom`). So either set the smoother's
`map_to_odom_child_frame` to that frame, or set `MOLA_ODOM_SENSOR_LABEL` to it;
otherwise the tree does not connect (tf2 "two unconnected trees"). LO's own poses
are fed under `MOLA_LO_PUBLISH_REF_FRAME` (default `map`), so they do not create
a separate odom source.

## SharedKeyframeMap sink (central-map keyframe push, e.g. mola_mapper_3d)

Separately from the dense `navstate_fuse` querying above, `LidarOdometry`
optionally detects a `mola::SharedKeyframeMap` sink at init
(`findService<mola::SharedKeyframeMap>()` in `LidarOdometry_Initialize.cpp`,
guarded by `#if defined(MOLA_HAS_SHARED_KEYFRAME_MAP_SINK)` /
`__has_include(<mola_kernel/interfaces/SharedKeyframeMap.h>)` so this still
builds against an older `mola_kernel`). Unlike `navstate_fuse`, this is
OPTIONAL: most systems still write their own local `.simplemap` only and
never have a sink.

When present, `pushKeyframeToSharedKeyframeMap()`
(`LidarOdometry_ProcessScan.cpp`) pushes a SPARSE keyframe at the exact same
`distance_enough_sm` criterion as the self-written simplemap (see the shared
keyframe policy below), but
INDEPENDENTLY of whether `params_.simplemap.generate` is set (the underlying
distance-checker's `insert()` is now driven by `pushToSharedKeyframeMapNow`
too, not just `updateSimpleMap` -- this was a real bug, see below). It uses
`source_frame_id = params_.publish_reference_frame + "_kf"`, a DEDICATED name
distinct from `publish_reference_frame` (the frame the dense, every-scan
`fuse_pose()` calls above use): reusing the same name lets the sink's
anchor-once tie collide with the dense path's tie on the same
(relocalization-seeded) keyframe, which threw
`gtsam::IndeterminantLinearSystemException` at startup in real testing
against KITTI through `mola_mapper_3d`'s
`mola-cli-launchs/lidar_odometry_mapper3d_from_kitti.yaml`. See that
package's agents.md ("Real end-to-end validation lessons") for the full list
of bugs this integration surfaced, several of which were in `mola_mapper_3d`,
not here -- but two structural changes live in this file:
`pushKeyframeToSharedKeyframeMap()`'s dedicated frame name, and the
distance-checker `insert()` gating fix mentioned above.

## Scan enqueue & overload handling (drop stale, keep freshest)

Incoming LiDAR scans reach a single worker thread (`worker_lidar_`) via
`onNewObservation` -> `sendLidarScanToProcessQueue`
(`LidarOdometry_SensorCallbacks.cpp`). Two routes:

- **LO (no IMU de-skew):** the scan is ready immediately and goes straight to
  `submitReadyLidarScanToWorker()`.
- **LIO (IMU de-skew):** the scan is parked on `worker_lidar_wait_for_imu_list_`
  until IMU data covering its whole time span has arrived; `onIMUImpl` then
  submits the now-ready scans via the same `submitReadyLidarScanToWorker()`.

`submitReadyLidarScanToWorker()` implements a **"drop stale, keep freshest"**
policy against a single pending slot (`worker_lidar_pending_fresh_scan_`): if the
worker is idle the scan is dispatched at once; if it is busy the scan replaces
(drops) any older scan waiting in the slot. `onLidar` processes its scan and then
drains the slot in a loop, so under overload the worker always advances to the
*newest* available scan instead of grinding through a deep FIFO backlog. This
keeps end-to-end latency near a single processing period (so the state-estimator
prediction is queried only a little into the future) for both LO and LIO.
`params_.max_lidar_queue_before_drop` now only bounds the IMU wait list.

## Adaptive-threshold sustained-failure recovery is ON by default

`recover_on_sustained_failure` (all `pipelines/*.yaml`, `adaptive_threshold`
block) defaults to **true**. Root cause: the adaptive-threshold sigma is an
EMA-smoothed P-controller driven toward `icp_quality_controller_setpoint`
(default 0.85); a run of easy/near-static scans with goodness consistently
above the setpoint (routine with a clean/synthetic LiDAR, e.g. MVSIM) drives
sigma monotonically down to its `min_motion` floor. Once there, a single
larger inter-scan motion (a turn) is enough to push ICP into failure, and
since sigma only shrinks on GOOD ICPs and previously only grew when this flag
was on, the correspondence search window stayed frozen shut forever — a
permanent stall (`estimated_trajectory` stops growing, "Not able to use
velocity motion model" repeats indefinitely). Reproduced with
`mola_lidar_odometry` + MVSIM (Jackal, 3D LiDAR) driving a short, obstacle-free
path: sigma reached the floor by ~pathStep 90-140 and then stalled
permanently every time with the flag off; with it on, the pipeline
self-recovered within a handful of scans and ran cleanly to completion. Set
`MOLA_ADAPT_THRESHOLD_RECOVER=false` to restore the old (deadlock-prone)
behavior.

`recover_after_n_bad`/`recover_growth_factor` default to a fast reaction (2
bad frames, x2.0 growth) instead of the initially-added slow one (5 bad
frames, x1.5). Rationale, also found via MVSIM end-to-end testing: every
frame stuck is a frame of real, untracked vehicle motion; the slower
defaults let that gap grow large enough that once the search window finally
reopened, ICP locked onto a self-consistent but WRONG registration (a
sudden ~30-40 deg yaw error that then persisted for the rest of the run,
instead of a brief quality dip that recovers to the true pose).


## Keyframe-creation policy (shared by local map and simplemap)

`mola::KeyframeDecider` + `mola::KeyframeDecisionOptions`
(`KeyframeDecider.{h,cpp}`) hold the single "is this pose far enough from the
existing keyframes to warrant a new one?" policy. There are TWO instances,
tuned independently: `state_.kf_decider_local_map` (local metric map) and
`state_.kf_decider_simplemap` (the simplemap, which is also what feeds a
SharedKeyframeMap sink). `Parameters::MapUpdateOptions` and
`Parameters::SimpleMapOptions` both DERIVE from `KeyframeDecisionOptions`, so
its fields stay plain, non-nested YAML keys of their sections; both are loaded
by `Parameters::load_keyframe_policy()`, which lives on `Parameters` because
the two distance thresholds may be formulas that must register into its
dynamic-parameter pool. The two distance thresholds are passed to `check()` per
call, never held, so those formulas are re-evaluated every scan. The two policy
SELECTORS (`measure_from_last_kf_only`, `nearby_keyframe_time_window`) are
instead read once, by the constructor, which takes the whole options struct:
they pick which of the two mutually exclusive storages the decider maintains
(the `SearchablePoseList` KD-tree, or the in-window deque), so they cannot be
per-scan formulas. `check()` asserts the window did not change afterwards.

`nearby_keyframe_time_window` [s] (`MOLA_SIMPLEMAP_KF_TIME_WINDOW`, default 0 =
disabled) bounds how far BACK IN TIME the redundancy test reaches. With the
purely spatial default, revisiting a mapped area creates NO keyframes, since
the previous pass' ones are the nearest neighbors. That is right for the local
map (a revisit adds no coverage) but starves loop closure, which can only
detect a loop whose BOTH endpoints exist as keyframes -- so the revisit that
should close the loop produces nothing to close it with. With a positive
window, only keyframes newer than that take part in the test (plus the most
recent one ALWAYS, so a parked vehicle does not emit one keyframe per window),
and a revisit spawns fresh keyframes. Enable it on the simplemap, not the local
map. `test/test_keyframe_decider.cpp` covers both regimes.

Under the window, the in-window deque is the ONLY store (the KD-tree is not
even fed), it is scanned newest-first and left as soon as
`min_nearby_poses_occupied` matches, and `insert()` clamps non-monotonic
timestamps so the time-ordered pruning stays valid.

## Non-repetitive (solid-state) LiDARs (e.g. Livox AVIA)

Spinning LiDARs (Velodyne, Ouster, ...) cover their full FOV every rotation, so
a single scan gives complete coverage and the default keyframe insertion
criterion (insert once per distance threshold, `min_nearby_poses_occupied=1`) is
correct.

Solid-state / non-repetitive LiDARs such as the **Livox AVIA** do NOT complete a
full-FOV sweep in one frame: the scan pattern is pseudo-random and accumulates
over multiple frames. Two consequences for pipeline tuning:

1. **`MOLA_MIN_NEARBY_POSES_OCCUPIED=2`** (and `MOLA_SIMPLEMAP_MIN_NEARBY_POSES=2`):
   raise this from the default 1 so that at least 2 scans are accumulated before
   the robot moves on, giving the local map and simplemap denser, more uniform
   coverage per location. Set in `local_map_updates.min_nearby_poses_occupied`
   and `simplemap.min_nearby_poses_occupied` in the pipeline YAML (both env-var
   gated). See `LidarOdometry.h:225` for the docstring.

2. **`MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU`**: always set this for
   LIO with any LiDAR that has per-point timestamps (Livox via `offset_time`,
   VLP-16 natively). The default `Linear` only uses the state-estimator twist;
   `IMU` uses raw gyroscope readings for finer-grained deskewing.

3. **`MOLA_IGNORE_NO_POINT_STAMPS=false`**: fail loudly if per-point timestamps
   are absent, rather than silently skipping deskewing (the default `true`).
   Use this when the sensor is known to always provide per-point timestamps.

See `mola-cli-launchs/lidar_odometry_from_botanicgarden_livox.yaml` for a
complete example with all three env vars set.

## Selectable local-map class in `pipelines/lidar3d-gicp.yaml`

`${MOLA_LOCALMAP_CLASS|mola::KeyframePointCloudMap}` picks the class used for
both the `localmap` layer and the `observation` (scan) layer -- `Matcher_Cov2Cov`
pairs the two, so they must always be the same `mp2p_icp::NearestPointWithCovCapable`
class. Both classes' option keys live side by side in one `creationOpts` block:
each map class silently ignores the keys it does not define, which is what makes
a single YAML enough. When adding keys, keep KFM's *required* ones
(`max_search_keyframes`, `k_correspondences_for_cov`) present.

- `mola::KeyframePointCloudMap` (default): keyframe-based, points kept in per-KF
  local frames, so it survives loop-closure re-mapping. Tuned by the
  `MOLA_LOCALMAP_*` vars.
- `mola::IncrementalPointCloud`: single global frame, one incremental
  self-balancing k-d tree, no per-scan tree rebuild. **Odometry only** (a global
  SE(3) re-map would force a full rebuild). Tuned by `MOLA_INCREMENTAL_MAP_*`:
  `MAX_SIZE` (eviction cube **half-side** [m] -- a much tighter budget than KFM's
  `remove_frames_farther_than`, which is a radius over keyframe centres),
  `ASYNC_REBUILD` (default `true`; moves the k-d tree rebuilds off the mapping
  thread and is what keeps insertion latency flat), `ALPHA_BALANCE`,
  `ALPHA_DELETED`, `RESERVE_POINTS`.

`mola::IncrementalPointCloud` needs `mola_metric_maps` built against
nanoflann >= 1.10.0. On distributions with an older one the class still exists
and is registered, but instantiating it throws an explanatory error, so
selecting it there fails with a clear message rather than silently falling back.
## IMU gravity correction

`params.imu_gravity_correction` constrains the ICP solution's tilt from the
accelerometer. Two knobs decide *how*, both defaulting to `true`:

- `use_rank2_prior`: deliver it as mp2p_icp's yaw-free, rank-2 `gravityPrior`,
  which touches only the two tilt DOFs. The legacy path (`false`) folds
  pitch/roll into the SE(3) pose prior, whose diagonal only isolates roll/pitch
  near yaw=0 and which injects translation. Compiled only when the mp2p_icp in
  use provides `mp2p_icp::GravityPrior` (`__has_include` guard in
  `LidarOdometry.h`); older versions still build and fall back with a warning.
- `adaptive_sigma`: widen `sigma_deg` by the measured dispersion of the buffered
  accelerometer directions. Required in practice: the quasi-static gate accepts
  `|norm(a)-g| <= 2 m/s^2`, i.e. up to ~11.8 deg of aliased tilt, so without it
  the constraint asserts tilt the reading does not contain and net-degrades
  odometry on a moving vehicle.

Note that the two are not independent in effect: the rank-2 form alone is a
correctness fix but does not improve odometry, because the tilt-to-Z coupling
lives in the geometry Hessian and is parameterization-invariant. The gain comes
from `adaptive_sigma`.

## Reproducible odometry evaluation

Trajectory-to-trajectory comparisons are only meaningful under all of:

- the **batch CLI** (`mola-lidar-odometry-cli`), never the real-time GUI: under
  real-time pacing scans are dropped and the motion prior is extrapolated with
  queueing delay, which alone moved APE on one sequence by several times.
- **`taskset -c N`**: `tbb::parallel_reduce` is FP non-associative, so thread
  scheduling changes the result. Pinning makes runs bit-identical; check with
  `md5sum` on the output `.tum` before comparing anything.
- **`MOLA_ASYNC_BACKEND=false`** when using the smoother state estimator (its
  async serving path is non-deterministic).

The smoother also needs `-l <libmola_state_estimation_smoother.so>`; the CLI
does not load that plugin by default and the class factory otherwise fails with
"unknown class name".

## Environment Variables (Debug/Tracing Flags)

Debug/tracing flags in C++ code use `mrpt::get_env<T>(name, default)` (from
`<mrpt/core/get_env.h>`), never plain `::getenv`/`std::getenv`. (The
`MOLA_MIN_NEARBY_POSES_OCCUPIED`-style vars above are a different mechanism:
they are resolved by `mola_yaml`'s `${VAR|default}` expansion inside the
pipeline YAML, not read directly in C++.)

| Variable | Type | Default | Location | Purpose |
|----------|------|---------|----------|---------|
| `MOLA_DEBUG_DUMP_ICP_LOG_FROM_TIMESTAMP` | double | 0 | `module/src/LidarOdometry_ProcessScan.cpp` | Start of a timestamp range for forcing ICP debug-log dumps (paired with `..._TO_TIMESTAMP`) |
| `MOLA_DEBUG_DUMP_ICP_LOG_TO_TIMESTAMP` | double | 0 | `module/src/LidarOdometry_ProcessScan.cpp` | End of the timestamp range above |
| `MOLA_LO_DEBUG_ICP_QUALITY` | bool | false | `module/src/LidarOdometry_ProcessScan.cpp` | Trace ICP quality metrics per scan |
| `LO_PIPELINE_YAML` | string | (unset) | `test/test_lidar_odometry_rawlog.cpp`, `test/test_lidar_odometry_rosbag2.cpp` | Path to the LO pipeline YAML used by the test |
| `LO_STATE_ESTIM_YAML` | string | (unset) | same tests | Path to the state-estimator YAML used by the test |
| `LO_TEST_RAWLOG` | string | (unset) | `test/test_lidar_odometry_rawlog.cpp` | Path to the input rawlog dataset |
| `LO_TEST_ROSBAG2` | string | (unset) | `test/test_lidar_odometry_rosbag2.cpp` | Path to the input rosbag2 dataset |
| `LO_TEST_LIDAR_TOPIC` | string | (unset) | `test/test_lidar_odometry_rosbag2.cpp` | LiDAR topic name to read from the rosbag2 |
| `LO_TEST_GT_TUM` | string | (unset) | both tests above | Path to the ground-truth trajectory (TUM format) |

`functor_should_generate_debug_file` (the callback backing the two
`MOLA_DEBUG_DUMP_ICP_LOG_*` vars above) is only installed on `icp_params`
when one of them (or `write_debug_icp_log_if_quality_under`) is actually
configured: `mp2p_icp::ICP::align()` gives an installed functor unconditional
priority over its own `generateDebugFiles` / `MP2P_ICP_GENERATE_DEBUG_FILES`
(and that path's `decimationDebugFiles` support), so an always-installed
functor would silently shadow the global flag even when none of its own
triggers fire. To dump every ICP call unconditionally, use
`MP2P_ICP_GENERATE_DEBUG_FILES=1` with neither `MOLA_DEBUG_DUMP_ICP_LOG_*` var
set. Each dumped `.icplog` embeds the full local-map snapshot for that call
(tens of MB), so a wide from/to range or an unbounded `MP2P_ICP_GENERATE_DEBUG_FILES`
run over a long dataset can reach tens of GB quickly; keep the range narrow
(a few seconds) or rely on `decimationDebugFiles`.

Plain `getenv()` calls remain only in `module/src/libcfgpath/cfgpath.h`
(vendored third-party code resolving standard XDG base directories:
`XDG_CONFIG_HOME`, `XDG_DATA_HOME`, `XDG_CACHE_HOME`, `HOME`) — these are OS
path-resolution lookups, not app debug flags, and are left untouched.

## Building

We use colcon, the ROS2 build tool, and mola_common with utility cmake helpers.

## Testing

- **Framework**: CMake + GTest
- Each package has `tests/` with its own `CMakeLists.txt`
- CI/CD: `.github/workflows/` — builds on ROS 2 Humble, Jazzy, Kilted, Rolling
- Style: enforced with `.clang-format` and `.clang-tidy`

## Code style

- Use clang-format-14
- Don't use one line statements: "if (foo) bar;" ==> "if (foo) {\n bar;\n}" ; enforced by clang-tidy rules
- Don't declare more than one variable in the same line: "int a,b;" => "int a;\n int b;"
- Don't use en, em dash "—", use any alternative notation.
- Use American spelling.
- Use anonymous namespaces instead of static.

