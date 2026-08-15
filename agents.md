# mola_lidar_odometry repository — AI Agent Context Guide

## Project Overview

**MOLA** (Modular Optimization framework for Localization and mApping) is a modern C++ and ROS 2 framework for robot **localization and SLAM** (Simultaneous Localization and Mapping).

This repository provides a LiDAR-Inertial Odometry (LIO) frontend for the MOLA framework. 

Official Docs: https://docs.mola-slam.org/latest/

## Dataset wrappers: `scripts/mola-lo-{gui,cli}-*` + `scripts/lib/`

Each supported dataset has exactly one description of itself, in
`scripts/lib/profiles/<name>.sh`: its on-disk layout, topic names, frames,
extrinsics and pipeline tweaks, exported as `MOLA_*` environment variables.
A profile invokes nothing.

Three kinds of consumer share it, so none of them restates a dataset:

- `mola-lo-gui-<name>` — online replay via `mola-cli` + a launch YAML.
- `mola-lo-cli-<name>` — offline batch via `mola-lidar-odometry-cli`.
- Out-of-tree harnesses (`eval/cli_*.sh`, the CI regression job) source
  `lib/dataset-profile.sh` and call `mola_lo_load_profile` directly.

Both binaries read the same variables: the launch YAMLs always did, and the
offline CLI's `--lidar-sensor-label` / `--imu-sensor-label` /
`--base-link-frame-id` / `--tf-topic` / `--tf-static-topic` carry matching
`envname()` fallbacks (`MOLA_LIDAR_TOPIC`, `MOLA_IMU_TOPIC`,
`MOLA_TF_BASE_LINK`, ...). An explicit flag still wins; an option that took
its value from the environment is reported at startup, since it changes the
run without appearing in the command line.

Every value a profile sets uses `: "${VAR:=default}"`, so **anything the
caller exports first wins**. That is how a harness overrides one field
without forking the profile. Two variables the caller sets:

- `MOLA_LO_MODE` = `gui` | `cli`. Profiles branch on it where a dataset
  genuinely differs — a camera bag is worth replaying for a preview and is
  pure decode cost in a batch run.
- `MOLA_LO_SKIP_STATE_ESTIMATOR=1` — for callers running a method with its
  own internal estimator (the DLIO / Fast-LIO2 wrappers).

Adding a dataset means adding one profile plus two one-line wrappers, and
listing it in `MOLA_LO_DATASET_WRAPPERS` in `CMakeLists.txt`.

### Per-dataset notes

- **conslam** autodetects ROS 1 (`.bag`) vs ROS 2 (`.mcap`) from the
  extension. `CONSLAM_BASE_FRAME` picks which frame the trajectory is
  reported in: `imu` (default, the dataset's own reference frame) or
  `lidar`, needed when scoring against a ground truth sampled in the LiDAR
  frame. The two differ by a pure 180 deg yaw.
- **grandtour** publishes one bag per topic, so the profile takes a *mission
  directory* (or its `*_hesai_undist.bag`) and resolves the siblings by the
  `<mission>_<topic>.bag` naming. Body frame is `base`, not `base_link`.
  Extrinsics come from the mission's own `/tf_static`, so no fixed poses are
  set. The LiDAR stream is already undistorted, so deskewing defaults to
  `None`. The /tf tree view is on by default (this is a legged robot with a
  full joint tree, which is the point), skipping the four frames not
  physically on the body: `odom`, `enu_origin`, `dlio_odom`, `dlio_map`.
- **tiers** records FIVE lidars at once — that is what the dataset is for —
  so it gets one wrapper per sensor (`-ouster-os0`, `-ouster-os1`,
  `-velodyne`, `-livox-horizon`, `-livox-avia`) rather than one that silently
  picks a winner. Extrinsics are a co-located placeholder: these bags carry
  no `/tf` and the dataset publishes no calibration.
- **oxford-spires** stitches multipart sequences, ordering the `raw/ros2bag/`
  parts by their trailing `_<n>` numerically.
- **ouster** is gui-only: a live source, which the offline CLI cannot read.

## `lidar_odometry_from_rosbag1.yaml`: up to 4 bags replayed jointly

`rosbag_filename` is a sequence fed from `MOLA_INPUT_ROSBAG1` plus three
optional slots, `MOLA_INPUT_ROSBAG1_2` / `_3` / `_4` (empty entries are
dropped by `Rosbag1Dataset`). Per-topic datasets that ship `/tf`, the IMU, a
camera or the odometry in separate bag files therefore need no launch file of
their own. `mola_lo_bag_slots` in `lib/dataset-profile.sh` fills these and the
comma-joined spelling the offline CLI takes, from one list.

## Robot /tf tree visualization (opt-in)

`visualization.show_tf_tree` draws the subtree of coordinate frames below
`tf_tree_root_frame` (empty = ask the data source for its `base_link` frame),
which on a legged robot is its joint tree. Off by default; every knob is also
live in the GUI's "View" tab.

The frames come from `mola::TransformTreeSource`, detected at init via
`findService<>()` and guarded by `__has_include`
(`MOLA_HAS_TRANSFORM_TREE_SOURCE`), so this still builds against an older
`mola_kernel`. The source resolves the poses against the root and does the
subtree filtering itself (see `mola`'s agents.md); LO only draws.

Snapshots are pulled once per visualization update, at the current scan's
timestamp, so the joints match the rendered cloud rather than the wall clock.
The result is drawn at the vehicle pose, since the poses are body-relative.

`tf_tree_exclude_frames` (comma-separated) drops a frame together with its
subtree. It is needed in practice because datasets publish *inverted* edges:
GrandTour has `base -> odom` and `hesai_lidar -> dlio_odom`, so without
excluding them a ~130 m translation drags an unrelated subtree into the
robot's own tree.

Links (`tf_tree_show_links`) render as thin `CCylinder`s, not GL lines: MRPT
cannot draw lines with a configurable thickness, so they are barely visible.
Radius is `tf_tree_link_radius` (`MOLA_LO_TF_TREE_LINK_RADIUS`, default 0.02 m),
also live in the GUI. Orientation is derived from the two endpoints (pitch =
`acos(nz)`, yaw = `atan2(ny, nx)` of the unit direction), the same approach
used for `mola_mapper`'s graph-edge cylinders.

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

- **No IMU received (yet):** the scan is ready immediately and goes straight to
  `submitReadyLidarScanToWorker()`.
- **With IMU:** the scan is parked on `worker_lidar_wait_for_imu_list_` until IMU
  data covering its whole time span has arrived; `onIMUImpl` then submits the
  now-ready scans via the same `submitReadyLidarScanToWorker()`. See
  "Timestamp-driven LiDAR/IMU synchronization" below.

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


## Local-map locking: `state_mtx_` vs `local_map_content_mtx_`

Rendering the local map is O(map size) and must not run on the LiDAR worker
thread, nor under `state_mtx_`: under `mola::IncrementalPointCloud` it measured
~55 ms mean / ~120 ms max on a GrandTour mission, which turned roughly one
`onLidar` in 18 into a >100 ms spike and backed the scan queue up to 18 entries.
So `updateVisualizationLocalMap()` only makes the decimation decision and
snapshots what the render needs (map `shared_ptr`, `render_params_t`, viz frame),
then enqueues the render on `worker_viz_local_map_`. There it takes only the
finer-grained `local_map_content_mtx_`, which every mutation of
`state_.local_map` contents (insert, `clear()`, `load_from_file()`) must also
hold. Effect on the same run: the inline cost drops to ~5 us and `onLidar`'s max
from 155 ms to 66 ms, with the same number of renders.

`worker_viz_local_map_` is deliberately a **separate** 1-thread
`POLICY_DROP_OLD` pool from `worker_viz_`: with one thread that policy caps the
queue at a single pending task, so sharing the pool would let the per-scan
current-observation frames drop the much rarer local-map render before it ever
ran. The "hide the local map" clear is routed through the same pool so it cannot
be overtaken by a render already in flight.

`visualization.map_update_decimation` (`MOLA_GUI_MAP_UPDATE_DECIMATION`,
default 10 in most pipelines) bounds how often that render is even requested.
The render itself no longer holds `local_map_content_mtx_` for its whole
duration: `cheapLayerSnapshot()` deep-copies the layers under the mutex and the
O(map size) recolorize/OpenGL build then runs on that private copy, unlocked.
The copy is ~6x cheaper than the render it replaces in the critical section
(27 ms vs 182 ms mean on Oxford Spires), which takes `onLidar`'s max from
216 ms to 107 ms and makes `onLidar.4.update_local_map` equal to the insertion
it wraps, i.e. zero lock wait.

Point layers are copied into a plain `CGenericPointsMap` rather than cloned
through their own type: `insertAnotherMap()` calls `registerPointFieldsFrom()`
(so every per-point field survives) and skips non-finite points (so the slots
`IncrementalPointCloud` blanks on eviction are dropped), while the target has no
spatial index to build. Cloning an `IncrementalPointCloud` through its own copy
constructor would instead `resetIndex()`, an O(N log N) k-d tree bulk build the
renderer never queries. The copy does pick up the storage slots that are
tombstoned but not reclaimed yet; that measured under 2% of storage
(`MOLA_INCREMENTAL_MAP_DEBUG_STATS`: live/storage 0.999 mean, 0.982 worst), and
it is the same trade-off `doPublishUpdatedLocalMap()` already makes.

The render worker pays ~20 ms more per render for the extra copy step, which is
fine: it is off the critical path by construction.

## Timestamp-driven LiDAR/IMU synchronization

IMU readings are **not** consumed when they arrive. `onIMUImpl()` only appends
them to `pending_imu_` (`PendingImuBuffer`, `ImuScanSync.h`) and updates
`latest_imu_time_`, inline on the sensor-input thread. A scan is held on
`worker_lidar_wait_for_imu_list_` (`ScanImuWaitList`) until `latest_imu_time_`
reaches the scan's own coverage end (its timestamp plus the estimated scan
period, frozen when the scan is parked). `processLidarScan()` then calls
`consumePendingImu()`, which feeds every buffered sample up to that same time,
in timestamp order, into the de-skew velocity buffer, the pitch/roll
calibrator and the gravity estimators.

The IMU samples a scan sees are therefore a function of the timestamps alone,
never of how the sensor callbacks interleaved, which is what makes two identical
offline runs produce identical trajectories. `mola_state_estimation_simple`
buffers IMU readings the same way for the same reason.

Limits: the gate only engages after the first IMU reading, so scans preceding it
are processed straight away. Reproducibility also assumes no scan is dropped for
overload, which is the offline case.

**The wait is bounded** (`params_.max_time_to_wait_for_imu`, default 0.5 s). An
IMU that stops mid-run freezes `latest_imu_time_`, so without a bound every later
scan waits for data that never comes and the odometry stalls permanently and
silently (`max_lidar_queue_before_drop` then merely recycles the wait list).
`imu_scan_release_time()` (`ImuScanSync.h`) therefore also releases scans whose
coverage end is older than `latest_obs_time_ - max_time_to_wait_for_imu`: the
odometry degrades to LiDAR-only and picks the IMU back up by itself when it
returns. Set the parameter to 0 to wait indefinitely.

The bound is in **sensor time** (`latest_obs_time_`, the newest timestamp on any
input), never the wall clock. That is what keeps it reproducible: a wall-clock
timeout would make the released set depend on machine load, reintroducing the
nondeterminism this design exists to remove, and it would buy nothing, since a
run whose inputs have all gone silent has nothing to process anyway.

Releasing a scan without its IMU means the pipeline moves past instants whose
readings may still arrive (also possible when an input interleaves the two
streams out of order). `PendingImuBuffer` keeps a watermark of the newest
consumed instant and rejects anything at or below it, so IMU data is never
applied out of chronological order; `clear()` resets it, a reset being a new
session.

**End of input.** The last scans of a run are parked waiting for IMU that the
dataset no longer contains, so they must be flushed explicitly or they are lost:
`flushPendingLidarScans()` releases them regardless of coverage and blocks until
the worker is idle. `shutdownCleanup()` calls it, which covers everything going
through `onQuit()`; `mola-lidar-odometry-cli` calls it directly after the replay
loop, because it reads `estimatedTrajectory()` before that point. Note that
`isBusy()` deliberately does **not** count the wait list: callers poll it between
observations, and a parked scan is only released by a later observation, so
counting it there would deadlock.

## `imu_state_mtx_`: the sensor input no longer waits on the LiDAR worker

`processLidarScan()` holds `state_mtx_` for its whole body, so an `onIMU()` that
also took `state_mtx_` blocked for the duration of every scan. At 200-400 Hz IMU
vs 10 Hz LiDAR that meant the IMU worker's total time tracked `onLidar`'s almost
exactly (33.1 s vs 32.7 s on Oxford Spires), and since
`releaseReadyLidarScansToWorker()` is the last thing `onIMU()` does, the wait fed
straight back into scan-submission latency.

`imu_state_mtx_` (recursive) now guards exactly the state the two threads share:
`pending_imu_`, `imu_initializer`, `gravity_estimator`, `map_gravity`,
`recent_imu_stamps`, `parameter_source` (variable map, `realize()` flags, and the
`localVelocityBuffer` that IMU samples write and the deskew stage reads), and
any *invocation* of `obs_generators`. `onIMU()` takes only this mutex; the LiDAR
thread takes it in short windows on top of `state_mtx_`. Result on the same
sequence: `onIMU` mean 957 -> 349 us, max 138.6 -> 9.7 ms, total 33.1 -> 12.4 s,
with `onLidar` unchanged.

The residual 12.4 s is accounted for exactly by the two stages that genuinely
share those objects, `onLidar.0.apply_generators` (3.0 ms/scan) and
`onLidar.1.deskew_early` (3.8 ms/scan), plus the ~176 us of real per-sample work.
Removing it means making `mola::imu::LocalVelocityBuffer` internally thread-safe
(it has no mutex today) so `FilterDeskew`'s single
`collect_samples_around_reference_time()` snapshot can run concurrently with IMU
appends; that is a `mola_imu_preintegration` change, not one for this repo.

Full lock order: `state_mtx_` -> `local_map_content_mtx_` -> `imu_state_mtx_`.
Never the reverse. The sensor input takes only `imu_state_mtx_` and the local-map
render worker only `local_map_content_mtx_`, so neither can invert it. Anything
that replaces `state_` wholesale (`reset()`) or rebuilds the pipelines
(`initialize()`) must hold `state_mtx_` **and** `imu_state_mtx_`.


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

## `pipelines/lidar3d-gicp-single-filter.yaml` (temporary test variant)

Same as `lidar3d-gicp.yaml`, except that the two chained `FilterDecimateAdaptive`
stages are replaced by ONE filter emitting both `decimated_for_map` and
`decimated_for_icp` from a single voxelization pass, via its `outputs`
parameter. Voxelizing is the dominant cost, so the second stage becomes
essentially free (~2 ms/scan on a 100k-point cloud).

One parameter does NOT survive the fold, and the fold-back has to reconcile it:
the chained "icp" stage has its own `voxel_size`
(`${MOLA_CLOUD_DECIMATION_VOXEL_SIZE_ICP|0.10}`), while a single filter has only
one grid (`${MOLA_CLOUD_DECIMATION_VOXEL_SIZE_MAP|0.15}`). So the ICP cloud is
sampled from the 0.15 m grid over the full scan rather than from a 0.10 m grid
over the already-decimated map cloud.

The two stages are separately settable since 2026-08-15; before that, one
`MOLA_CLOUD_DECIMATION_VOXEL_SIZE` drove both and only the defaults differed.
That name is retired and now has no effect -- set both of the above to
reproduce a run recorded against it. The stages were split because they want
different cell sizes: measured on KITTI, a coarse map voxel wins from 400 m of
travel onward while a finer one wins at 100-300 m, so a single value cannot
express both drift accumulation and short-range precision.

It lives in a separate file
only because `outputs` needs an mp2p_icp newer than the current release; fold it
back into `lidar3d-gicp.yaml` and delete it once mp2p_icp is re-released. It is
deliberately NOT wired into `test/CMakeLists.txt`, which must keep building
against the released mp2p_icp.

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

The accelerometer supplies `up_body` (a per-scan measurement); the map's own
vertical `up_map` is a separate question. By default it is FROZEN from one
accelerometer average at the first keyframe, so whatever error that capture had
biases verticality for the whole run.
`imu_gravity_correction.map_gravity.enabled` replaces it with
`mola::imu::MapGravityEstimator`, which solves for gravity in the map frame from
preintegrated IMU plus this odometry's own relative attitudes and velocities;
its earned pitch/roll sigma is added in quadrature to the prior's. There is no
quality threshold anywhere in that path, by design: the library reports every
usable estimate with its sigma and the weighting decides (see
`mola_imu_preintegration/agents.md`). Do NOT read those sigmas as a confidence
gate, though: measured on a handheld dataset the error/sigma ratio is 8.4
median and 19.0 worst, so "a weak estimate silences itself" is not established.

`map_gravity.log_only` computes and logs the estimate without letting it reach
the verticality reference, so the trajectory is identical to a disabled run.
That is the mode to validate the estimator on a new dataset: with the feedback
loop closed, the map frame being estimated is partly the estimator's own doing,
and scoring it against ground truth would be self-referential.

## Re-leveling the map frame (`map_gravity.relevel_map_frame`)

All of the above corrects the per-scan *prior*; the map frame itself stays
where it started, which is the initial body frame, tilt included. On a handheld
dataset that is a median 8.7 deg lean (max 20.7) baked into every map product
for the whole run. `relevel_map_frame` (default **false**) rotates the map
frame once, about the map origin, by the estimator's `Result::correction`.

It is a **gauge change**, not a state update, and must stay one:
`MapFrameRelevel.h` applies `p -> b + p` to the local map, the simplemap, the
trajectory; `KeyframeDecider`/`SearchablePoseList::transform_left_multiply()`
move the keyframe-density bookkeeping with them; and
`NavStateFilter::transform_frame()` does the same inside the state estimator.
Anything expressed in the *vehicle* frame (twists, sensor extrinsics) is
invariant and must not be touched. The decision is taken in
`evaluateMapFrameRelevel()` and applied by `applyMapFrameRelevel()`, which runs
outside `imu_state_mtx_` because the lock order forbids taking the local-map and
simplemap mutexes under it. It fires before the scan reaches either map, and
refuses outright once a map has been loaded or geo-referenced.

Two things about the trigger that are easy to get wrong:

- **It is not `min_intervals_for_convergence`.** That gates the per-scan prior,
  where the later, settled estimate is the useful one. For leveling the map once
  the estimate is at its *best* at the first solve (0.72 deg at ~5.6 s) and
  degrades from there, so `relevel_min_intervals` is deliberately tiny (5 = the
  first solve).
- **The magnitude gate is load-bearing.** The correction's residual is the
  estimator's own error (0.63 deg median, 1.43 p90, 1.75 worst at the firing
  point), so on an already-level start it makes things worse. The gate tests the
  estimate while the quantity that must be large is the truth, and they differ
  by that error, hence `relevel_min_tilt_deg` ~ 2x the p90, not 1x.

The applied rotation is logged once at INFO and written to the local map's
`metadata` and to every later keyframe's metadata observation, since a run with
a re-leveled map frame is not pose-comparable with one without it.

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

Always characterize the run-to-run noise floor (the same config twice) before
believing a difference between two configs, and confirm the estimate covers the
full ground-truth timespan.

Ready-made rig for Oxford Spires (`StateEstimationSimple`, deterministic):
`~/lo-gravity-eval/{oxford_env.sh,run_oxford.sh,eval_tum.py,analyze_map_gravity.py}`.
The bags carry no `/tf`, so the sensor extrinsics must be passed as the fixed
poses the env script sets, and the sensor labels are the topic names — or use
`mola-lo-cli-oxford-spires`, which sets exactly those from the profile.

`eval/cli_*.sh` are batch sweeps, not launchers: they choose sequences,
parallelism and metrics, and hand the launching to `mola-lo-cli-<dataset>`.
Anything a sweep pins deliberately (KITTI's `MOLA_INITIAL_VX`, which differs
from the interactive default) is set there explicitly and commented, since a
sweep's numbers are only comparable to its own history.

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
- CI/CD: `.github/workflows/build-ros.yml` has two matrices. GitHub-hosted
  (x86_64) runs Humble/Jazzy/Rolling/Lyrical stable plus Jazzy testing+coverage;
  Humble and Rolling testing entries exist commented-out, Lyrical has none.
  There, `setup ROS environment` only runs for testing entries, so stable ones
  need a prebuilt `ros:<distro>` image (or the manual `ubuntu:resolute` setup
  used for pre-buildfarm distros like Lyrical). Self-hosted (arm64) only covers
  Humble/Jazzy, each stable + testing, and runs `setup ROS environment`
  unconditionally for both.
- Style: enforced with `.clang-format` and `.clang-tidy`

## Code style

- Use clang-format-14
- Don't use one line statements: "if (foo) bar;" ==> "if (foo) {\n bar;\n}" ; enforced by clang-tidy rules
- Don't declare more than one variable in the same line: "int a,b;" => "int a;\n int b;"
- Don't use en, em dash "—", use any alternative notation.
- Use American spelling.
- Use anonymous namespaces instead of static.

