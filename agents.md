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

When paired with `mola_state_estimation_smoother` configured with
`async_backend: true`, that per-scan `estimated_navstate()` call does NOT trigger
the smoother's iSAM2 window solve on the LiDAR thread; it returns a fast prediction
from the smoother's lightweight predictor (re-anchored on the last completed
backend solve), while the window solve runs concurrently in the smoother's own
thread. This bounds per-scan latency for real-time use. With the default
`async_backend: false`, `estimated_navstate()` runs the window solve synchronously
(deterministic, but heavier per call) - see `mola_state_estimation`'s docs.

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
`distance_enough_sm` criterion as the self-written simplemap, but
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

