^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_lidar_odometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#101 <https://github.com/MOLAorg/mola_lidar_odometry/issues/101>`_ from MOLAorg/fix/relocalize-tf-source-and-sigma-recovery
  fix: two bugs blocking GNSS+IMU relocalization (FromStateEstimator)
* fix: MOLA_LOCALIZATION_PUBLISH_TF_SOURCE used the wrong module-name string
* fix: maximum_sigma default equal to initial_sigma made sustained-failure recovery a no-op
* Remove dead mrpt/gui included
* fix: enable adaptive-threshold recovery by default; add initial_pose launch arg
* fix: show sensible errors if in localization only but there's no local map
* feat: map freeze after relocalization (`#100 <https://github.com/MOLAorg/mola_lidar_odometry/issues/100>`_)
* fix: drop stale LiDAR scans and keep the freshest under overload (`#99 <https://github.com/MOLAorg/mola_lidar_odometry/issues/99>`_)
* feat: export as plot data the complete onLidar CPU time
* fix: don't process/insert a scan into the map before initial localization converges
* fix: don't discard a preexisting map on IMU releveling or bad-first-ICP restart
* Merge pull request `#98 <https://github.com/MOLAorg/mola_lidar_odometry/issues/98>`_ from MOLAorg/feature/imu-leveling-preserve-xyzyaw
  fix: preserve x/y/z/yaw prior in InitLocalization::PitchAndRollFromIMU
* Merge pull request `#97 <https://github.com/MOLAorg/mola_lidar_odometry/issues/97>`_ from MOLAorg/feature/imu-bag-recv-timestamp-env-var
  Wire MOLA_IMU_USE_BAG_RECV_TIME env var for IMU bag-recv-time timestamp override
* fix: preserve x/y/z/yaw prior in InitLocalization::PitchAndRollFromIMU
* lidar_odometry_from_rosbag2.yaml: wire MOLA_IMU_USE_BAG_RECV_TIME env var
* debug: add env-gated ICP quality/adaptive-threshold trace
* Merge pull request `#95 <https://github.com/MOLAorg/mola_lidar_odometry/issues/95>`_ from MOLAorg/feat/shared-keyframe-velocity-metadata
  Carry per-keyframe velocity metadata in shared-keyframe push
* refactor: remove dead pushKeyframeToSharedKeyframeMap helper
* feat: carry per-keyframe velocity metadata in shared-keyframe push
* Merge pull request `#94 <https://github.com/MOLAorg/mola_lidar_odometry/issues/94>`_ from MOLAorg/feat/icp-metric-plots
* feat: stream ICP time/goodness to mola_viz_imgui plot windows
* Merge pull request `#93 <https://github.com/MOLAorg/mola_lidar_odometry/issues/93>`_ from MOLAorg/feat/expose-approximate-cov-env-var
* feat: expose KeyframePointCloudMap's approximate_cov as MOLA_LOCALMAP_APPROXIMATE_COV
* docs: show ImGui as the new default
* feat: expose new simple estimator GNSS fuse params
* feat: pass georef to state estimator, if compatible
* feat: expose color changing as public API
* launch: default viz module to MolaVizImGui with per-app imgui_app_name
  Sets a unique imgui_app_name in each launch file so Dear ImGui's
  layout persistence stores a separate UI configuration per app.
* fix: uncheck show trajectory never cleared old viz
* Merge pull request `#92 <https://github.com/MOLAorg/mola_lidar_odometry/issues/92>`_ from MOLAorg/feat/viz-decay-lookat-frame-aware
  feat: frame-aware decay clouds and camera look-at for mapper_3d
* feat: frame-aware decay clouds and camera look-at for mapper_3d
* Merge pull request `#91 <https://github.com/MOLAorg/mola_lidar_odometry/issues/91>`_ from MOLAorg/feat/use-viz-with-movable-frames
* feat: viz with movable frames
* Merge pull request `#90 <https://github.com/MOLAorg/mola_lidar_odometry/issues/90>`_ from MOLAorg/feat/push-to-central-map
  feat: push KFs to central map server
* Merge pull request `#89 <https://github.com/MOLAorg/mola_lidar_odometry/issues/89>`_ from MOLAorg/fix/imu-grav-align
  fix: bugs in gravity-alignment from IMU accelerometer
* Merge pull request `#88 <https://github.com/MOLAorg/mola_lidar_odometry/issues/88>`_ from MOLAorg/feat/delayed-map-load
  feat: add delayed map load option
* Merge pull request `#87 <https://github.com/MOLAorg/mola_lidar_odometry/issues/87>`_ from MOLAorg/fix/state-mtx-non-recursive
  refactor: convert state_mtx\_ from recursive_mutex to plain mutex
* Merge pull request `#86 <https://github.com/MOLAorg/mola_lidar_odometry/issues/86>`_ from MOLAorg/fix/lidar-odometry-shutdown-order-segfault
  fix: avoid use-after-free during shutdown via LidarOdometry::onQuit()
* Merge pull request `#85 <https://github.com/MOLAorg/mola_lidar_odometry/issues/85>`_ from MOLAorg/feat/ros2-launch-tf-no-ns-option
* feat: add new ros2 launch argument to optionally disable /tf NS remappings
* Merge pull request `#84 <https://github.com/MOLAorg/mola_lidar_odometry/issues/84>`_ from MOLAorg/feat/simpler-adaptive-sigma
  feat: simplify adaptive sigma algorithm
* chore: put all yaml files in sync re adaptive parameters
* feat: simplify adaptive sigma algorithm
* docs: explain how to select the viz module
* Merge pull request `#83 <https://github.com/MOLAorg/mola_lidar_odometry/issues/83>`_ from MOLAorg/feat/ros1-input
* feat: Add ros1 bag input helper scripts
* Contributors: Jose Luis Blanco-Claraco

2.2.1 (2026-06-04)
------------------
* simple state estimator: MOLA_NAVSTATE_VELOCITY_FILTER is now true by default
* ci: fix Jazzy Jalisco EOL date in CI workflow comment (May 2024 - May 2029)
* ci: scope arm64 apt-cacher-ng proxy to apt only (fix xmllint test) (`#82 <https://github.com/MOLAorg/mola_erathos_slam/issues/82>`_)
* chore: add profiler to unit tests
* ci: add tests run in self-hosted
* fix self-runner label
* CI: add self hosted runner jobs too
* feat: selective disabling each of the imgui tabs
* gicp yaml: expose more viz params and the new velocity filter param
* docs: sync pipeline variables from actual yaml
* Merge pull request `#81 <https://github.com/MOLAorg/mola_erathos_slam/issues/81>`_ from MOLAorg/clean-gui-code
  GUI: clear dead code and reorganize ImGui tabs
* fix: multithread issues
* gui: split in 3 tabs when in imgui mode
* chore: remove dead code for mola<2.6.0 compatibility
* Merge pull request `#80 <https://github.com/MOLAorg/mola_erathos_slam/issues/80>`_ from MOLAorg/more-robust-multi-sensor
  More robust multi sensor
* fix: gracefully handle missing scans in multi-lidar settings
* chore: add warning if dropped lidar scans in multi-sensor mode
* chore: add debug-level traces for multi-lidar settings
* Contributors: Jose Luis Blanco-Claraco

2.2.0 (2026-05-11)
------------------
* fix: don't exit upon state estimator lack of convergence
* Merge pull request `#79 <https://github.com/MOLAorg/mola_lidar_odometry/issues/79>`_ from MOLAorg/simplify-ci
  CI: simplify clang-format helpers and use ros: docker image for jazzy stable
* CI: simplify clang-format helpers and use ros: docker image for jazzy stable
  - Replace the old formatter.sh with a new version supporting --check mode
  - Simplify check-clang-format.yml to just apt-install clang-format-14 and run the script
  - Use ros:jazzy pre-built image for jazzy stable CI build (faster, no setup-ros needed)
* Update error threshold in lidar odometry test
* fix: do not wipe out a loaded map if first scan is bad
* fix: viz must clear current obs when unchecked live
* fix: safer thread viz with just one thread
* chore: set default for adaptive threshold 'alpha' low-pass filter 0.99 to 0.90 for adapting faster to changes
* feat: show sensor pose corners in MolaViz
* chore: fix comment formatting (seems to trigger a libfyaml/mola_yaml parser bug)
* fix: initial pose from yaml files expected yaw/pitch/roll in degrees
* Expose initial sigma as env var too
* chore: expose viz module as env var (prepare for testing imgui)
* CI: sensible job names
* Merge pull request `#78 <https://github.com/MOLAorg/mola_lidar_odometry/issues/78>`_ from MOLAorg/bump-cmake-version
  bump min req cmake version to 3.22
* bump min req cmake version to 3.22
* FIX: regression in last adaptive sigma PR
* chore: add minimal agents.md
* Merge pull request `#76 <https://github.com/MOLAorg/mola_lidar_odometry/issues/76>`_ from Zeal-Robotics/feat/sustained-failure-recovery
  feat: optional adaptive-threshold recovery on sustained ICP failure
* feat: optional adaptive-threshold recovery on sustained ICP failure
  The KISS-ICP adaptive threshold only updates sigma on a good ICP, which
  is correct for isolated bad scans but creates a deadlock under sustained
  failure: once sigma is frozen at a small value, the matcher window
  (2*sigma) is too tight to find correspondences, ICP stays bad, sigma
  stays frozen, and the system cannot recover without an external
  relocalize.
  Add three opt-in fields to AdaptiveThreshold:
  recover_on_sustained_failure (default false)
  recover_after_n_bad          (default 5)
  recover_growth_factor        (default 1.5)
  After N consecutive bad ICPs, sigma is grown multiplicatively (capped at
  maximum_sigma) so the next attempt has a wider correspondence search
  radius. The counter resets on any good ICP, at which point the standard
  KISS-ICP rule resumes and re-tightens sigma. With the flag off, behavior
  is identical to before.
  The four bundled pipeline YAMLs (lidar3d-gicp, -gicp-optimize-twist,
  -icp, -ndt) gain matching ${MOLA_ADAPT_THRESHOLD_RECOVER*|default} hooks
  so the feature can be toggled via env vars without forking the YAML.
* Fix CI escaping
* fix: CI code coverage flags
* feat: Add new mola-lo-gui-ouster script
* Update build-ros.yml to disable known regression in current stable Humble
  Comment out the configuration for non-testing ROS build.
* Merge pull request `#75 <https://github.com/MOLAorg/mola_lidar_odometry/issues/75>`_ from Zeal-Robotics/fix/cli-warn-no-state-estimator-yaml
  fix(cli): warn when no --state-estimator-param-file is provided
* fix(cli): warn when no --state-estimator-param-file is provided
  When `mola-lidar-odometry-cli` is invoked without
  `--state-estimator-param-file`, the state estimator is constructed but
  `initialize()` is never called, so it silently runs with the built-in
  C++ defaults. These differ from the bundled
  `state-estimator-params/*.yaml` shipped with this package and used by
  the corresponding `mola-cli-launchs/*.yaml` files, which makes the CLI
  and the GUI launcher behave noticeably differently for the same
  estimator.
  Add an `else` branch that prints a clear warning naming the active
  estimator class and pointing at the bundled YAMLs, so users know to
  pass `--state-estimator-param-file` (or accept the C++ defaults
  deliberately). No behaviour change otherwise.
* fix: GUI update staled in some conditions
* fix: don't reduce adaptive threshold on bad ICPs
* fix: UI text labels never updated if ICP was bad
* fix: clearing gravity vector
* IMU buffer for gravity alignment: increase max circular buffer size
* Merge pull request `#62 <https://github.com/MOLAorg/mola_lidar_odometry/issues/62>`_ from MOLAorg/wip/draw-gravity-align-as-arrow
  Draw arrow from IMU gravity alignment
* CI: Fix for new ROS rolling
* feat: Optional visualization of IMU gravity alignment vector
* tests: show error levels even if test pass
* gicp: lower covariance floors
* fix: decaying clouds were not cleared if unchecked UI box
* icp pipeline yaml: add new covariance selection method params
* feat: ros2 launch file new "use-sim-time:=true" argument
* feat: Localmap is now also rendered using 'intensity' or whatever color channel
* docs: clear README
* Merge pull request `#72 <https://github.com/MOLAorg/mola_lidar_odometry/issues/72>`_ from manankharwar/patch-1
  docs: add FusionCore to Related projects
* Update README.md
* Contributors: Jose Luis Blanco-Claraco, Robin Van Cauwenbergh

2.1.0 (2026-04-29)
------------------
* FIX: show_localmap was not loaded from YAML file; expose more pipeline env vars
* Merge pull request `#70 <https://github.com/MOLAorg/mola_erathos_slam/issues/70>`_ from MOLAorg/feat/multiple-scans-per-volume
  simple estimator params file: expose all hardcoded values as env vars
* simple estimator params file: expose all hardcoded values as env vars
* Merge pull request `#69 <https://github.com/MOLAorg/mola_erathos_slam/issues/69>`_ from MOLAorg/feat/multiple-scans-per-volume
  feat: new option to include multiple scans per space volume
* feat: new option to include multiple scans per space volume
* feat: Add IMU & LiDAR configurable QoS (requires latest mola_bridge_ros2)
* Merge pull request `#67 <https://github.com/MOLAorg/mola_erathos_slam/issues/67>`_ from Zeal-Robotics/fix/pose-timestamp-after-deskew
  fix: stamp published pose with deskew reference time, not raw obs stamp
* fix: stamp published pose with deskew reference time, not raw obs stamp
  The ICP-derived pose corresponds to the vehicle at t=0 of the deskewed
  cloud. With the default `FilterAdjustTimestamps: MiddleIsZero`, t=0 is
  the *middle* of the scan, while `obs->timestamp` (taken from the LiDAR
  driver header) is the *start* of the scan. The published
  `LocalizationUpdate.timestamp` and the timestamp fed back into
  `navstate_fuse->fuse_pose()` were both using `obs->timestamp`, so
  downstream consumers (and the internal fuser) saw a pose tagged with
  a time roughly half a scan period (~50 ms at 10 Hz) before the moment
  the pose actually holds.
  The absolute reference time is already tracked inside
  `ParameterSource::localVelocityBuffer.get_reference_zero_time()`:
  - `Generator` seeds it with `obs->timestamp`
  - `FilterAdjustTimestamps` shifts it by the same offset it applies to
  per-point timestamps
  - `FilterAbsoluteTimestamp` already uses it as the absolute reference
  for per-point timestamps
  This commit reads it back after the observation pipeline runs and
  threads it through every pose-time use site:
  - ICP initial-guess query (`estimated_navstate`)
  - ICP result fusion (`fuse_pose`)
  - `estimated_trajectory` insertion
  - `past_simplemaps_observations` keyframe key
  - Published `LocalizationUpdate`, map, deskewed-scan, debug-trace
  timestamps
  Sensor-cadence uses (rate stats, drop-too-close logic, debug log
  context, `last_obs_timestamp`, per-label staleness tracking) keep the
  raw `obs->timestamp` since they are about *when the observation
  arrived*, not *when the pose holds*.
  If `FilterAdjustTimestamps` is not configured the buffer's reference
  equals `obs->timestamp` and behavior is unchanged. There is a fallback
  to `obs->timestamp` for safety if the buffer was never seeded.
* Merge pull request `#66 <https://github.com/MOLAorg/mola_erathos_slam/issues/66>`_ from MOLAorg/better-grav-aligner-tests
  review: address gravity-aligner / rebaker review comments
* review: address gravity-aligner / rebaker review comments
  - Fix stale Doxygen on onNewKeyframe (window param) and computePublishResidual
  (drop tail_kf_id reference, document unconditional residual contract).
  - TrajectoryRebaker::rebake: slide anchor forward to lower_bound when no KF
  >= anchor_id exists, so corrected_poses and reported anchor_id stay
  consistent.
  - Remove redundant empty-input guard in two-arg rebake overload.
  - Replace composePoint+subtraction delta computation with rotateVector in
  TrajectoryRebaker and the test helper.
  - Extract kDefaultAxisEps single source of truth used by Params and
  rotationFromGravity.
  - Fix sign in test bodyGravity comment (a_body.z = +g*cos), clarify
  accelerometer convention.
  - Wire maxGravityErrorDeg into KnownPitchTilt_converges as a sanity check.
  - Clarify LinearDrift_recoversHorizontalPath comment that R_grav per-KF
  exactly cancels T_odom.R (scenario justifying the 5cm Z tolerance).
* Merge pull request `#65 <https://github.com/MOLAorg/mola_erathos_slam/issues/65>`_ from MOLAorg/feat/better-gravity-align
  feat: add GravityMapAligner with per-KF IMU pool and robust gravity estimation
* fix minimum accelerometer noise
* feat: add TrajectoryRebaker for per-KF gravity-aware pose chain re-integration
* feat: add GravityMapAligner with per-KF IMU pool and robust gravity estimation
  New class GravityMapAligner stores one time-averaged body-frame
  accelerometer sample per keyframe and provides:
  - estimateGravityVector / estimatePerKFCorrection: IRLS/Huber-robust
  weighted mean of R_i·a_body_i to estimate gravity direction in the
  odom frame, with per-window support for the per-KF rebake path.
  - rotationFromGravity: pitch/roll-only correction (zero yaw by
  construction) that sends the estimated gravity vector to +Z.
  - onNewKeyframe / onKeyframeDropped for pool lifecycle management.
  Unit tests cover: nullopt below threshold, zero-tilt identity correction,
  2° pitch convergence (<0.1°), Huber robustness with 20% outlier KFs,
  pool eviction, yaw-free output, and per-KF windowed estimation.
  Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
* Merge pull request `#64 <https://github.com/MOLAorg/mola_erathos_slam/issues/64>`_ from MOLAorg/fix/more-rational-use-mutexes
  FIX: More rational use of mutexes and shared opengl objects
* FIX: More rational use of mutexes and shared opengl objects
* Merge pull request `#63 <https://github.com/MOLAorg/mola_erathos_slam/issues/63>`_ from MOLAorg/fix/deadlock-load-map
  FIX: potential dead locks/long waits requesting relocalization
* FIX: potential dead locks/long waits requesting relocalization
* Merge pull request `#60 <https://github.com/MOLAorg/mola_erathos_slam/issues/60>`_ from Zeal-Robotics/fix/sensor-frame-max-range-and-filter-anchors
  fix(mola_lidar_odometry): anchor ESTIMATED_SENSOR_MAX_RANGE and FilterByRange at the sensor
* Rename pipeline *_sensor\_* names to *_observation_radius\_*, document base_link-anchored deadzone
  The pipeline variables and YAML param keys historically called
  *_SENSOR_MAX_RANGE / *_sensor_range\_* are actually the bounding-radius
  of the latest observation cloud measured from base_link, not anything
  sensor-anchored. This caused real-world confusion: users reading the
  name expected sensor-frame semantics and were puzzled when the
  canonical FilterByRange deadzone removed points around base_link
  instead of around the sensor.
  Rename to make the semantics self-documenting:
  - Dynamic vars (parameter source):
  ESTIMATED_SENSOR_MAX_RANGE     -> ESTIMATED_OBSERVATION_RADIUS
  INSTANTANEOUS_SENSOR_MAX_RANGE -> INSTANTANEOUS_OBSERVATION_RADIUS
  - YAML param keys:
  max_sensor_range_filter_coefficient -> observation_radius_filter_coefficient
  absolute_minimum_sensor_range       -> absolute_minimum_observation_radius
  - Mirroring C++ symbol renames in state\_, params\_, and methods
  (doInitializeEstimatedObservationRadius etc.). GUI label tweaked.
  All legacy names remain working as deprecated aliases (dynamic vars are
  double-published; YAML keys fall back via cfg.getOrDefault). A one-shot
  warning is emitted at init when the loaded YAML still references any
  legacy name; aliases can be removed in a future release.
  The user-facing env var MOLA_ABS_MIN_SENSOR_RANGE is intentionally kept
  unchanged: the YAML \${VAR|default} substitution is single-pass, so the
  elegant chained-default pattern doesn't aliasing it cleanly, and a
  setenv shim has caveats for callers that load pipelines outside of the
  canonical entry points. The misnomer survives at the env-var name only;
  docs already point to the renamed concept.
  Also clarify in lidar3d-gicp.yaml that the L\u221E cube deadzone is
  intentionally vehicle-anchored (centered at base_link, not at the
  sensor) so it covers the vehicle body and a person standing next to the
  robot regardless of where the sensor is mounted. Users who additionally
  want a sensor-anchored cut can layer one on via observations_prefilter_file.
  No behavioral change.
* Merge branch 'Zeal-Robotics-perf/prewarm-icp-search-on-startup' into develop
* perf(initialize): pre-warm ICP search structures for preloaded local maps
  When the local map is loaded from disk via `load_existing_local_map` (the
  multi-session SLAM / localization-only path), each layer that implements
  `mp2p_icp::IcpPrepareCapable` defers building its ICP search structures
  (per-keyframe global-frame cloud materialization, merged submap
  construction, KD-tree build, ...) until the first call to
  `mp2p_icp::ICP::align()`. For a non-trivial preloaded map this is
  seconds of work that blocks the lidar worker on the very first scan,
  backs the bag-feed up, and freezes the GUI.
  Move that cost to startup by walking `state\_.local_map->layers` right
  after `load_from_file` and calling `icp_get_prepared_as_global()` on
  every IcpPrepareCapable layer, using the configured initial pose as the
  reference point. The total amount of work is unchanged; it just happens
  alongside `load_from_file` (where the user already expects a delay)
  instead of during real-time playback.
  If the actual first ICP estimate ends up far from the initial pose, the
  selected sub-keyframe set may be rebuilt then, but the per-keyframe
  point-cloud and cache data is already materialized so that rebuild is
  cheap.
* Merge pull request `#59 <https://github.com/MOLAorg/mola_erathos_slam/issues/59>`_ from Zeal-Robotics/feat/expose-transform-tolerance-env-vars
  feat(launch): expose transform_tolerance and transform_publish_period
* feat(launch): expose transform_tolerance and transform_publish_period
  Plumbs the two new BridgeROS2 params through the standard
  MOLA_ROS2\_* env-var convention so they can be overridden from
  ros2 launch without editing YAML:
  MOLA_ROS2_TRANSFORM_TOLERANCE       default 0.1
  MOLA_ROS2_TRANSFORM_PUBLISH_PERIOD  default 0.05 (20 Hz; 0 disables)
  Defaults match BridgeROS2 and the AMCL / slam_toolbox / RTAB-Map
  convention, so consumers can lookupTransform(map, base_link, now())
  without tf2 ExtrapolationException.
* Add MOLA_DESKEW_IGNORE_ACCELEROMETER env var
* consistent ros2 topic name var MOLA_ODOMETRY_TOPIC for both offline/online
* Merge pull request `#58 <https://github.com/MOLAorg/mola_erathos_slam/issues/58>`_ from MOLAorg/feat/ros2-diagnostics
  Implement new mola_kernel diagnostics API
* clang format
* diagnostics: use wall-clock reception time for input-data staleness check
  last_obs_timestamp stores the sensor hardware timestamp which may not be
  synchronized to system time (GPS-disciplined lidars, unsync'd clocks, etc.).
  Introduce last_obs_reception_time (set via mrpt::Clock::now() when each scan
  arrives) and use it for the Input Data stale/error age computation so the check
  reflects actual data flow rather than sensor-vs-host clock drift.
  Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
* docs/launch: alphabetical ordering for use\_* args; drop spurious tf remap from aggregator
  - mola_lo_ros_node.rst: move use_diagnostic_aggregator before use_imu_for_lio
  - launch.py: remove remappings=tf_remaps from diagnostic_aggregator node (it
  only needs diagnostics topics, not tf)
  Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
* diagnostics: fix ICP quality startup false-ERROR, timing period source, and overall scope
  - ICP Quality: treat as STALE (not ERROR) until state\_.last_icp_timestamp is set
  - Timing: derive sensorPeriod from observed scan interval (last_observed_scan_period_sec)
  rather than params\_.min_time_between_scans throttle threshold
  - Overall Status: iterate only over entries added by this provider (startIndex..end)
  so unrelated providers' diagnostics do not influence the overall level
  Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
* diagnostics: validate threshold ordering in Parameters::Diagnostics::initialize
  Assert that warn < error for icp_quality, input_stale_sec, and dropped_ratio,
  and that all values are in their valid ranges, so misconfiguration cannot
  silently invert severities at runtime.
  Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
* Add optional diagnostic_aggregator launch + sample config
  Off by default (use_diagnostic_aggregator:=True to enable). Intended for
  isolated bring-up/demos; in a larger stack a central aggregator should
  group LidarOdometry statuses instead.
  Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
* docs: add REP-107 diagnostics page for MOLA-LO
  Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
* Implement new mola_kernel diagnostics API (exported to ROS2 via BridgeROS2)
* ros2 launch: improved configuration validator
* ros2 launch: Add missing mola_bridge_odometry_frame argument; fix broken behavior
* Merge pull request `#57 <https://github.com/MOLAorg/mola_erathos_slam/issues/57>`_ from MOLAorg/feat/clarify-usage-modes
  docs: clarify usage modes; cli: add /tf selection args
* ros2 launch: safe guard against importing duplicated odometry
* ros2 launch: add wheels odometry arguments; fix opaque function must come last
* docs: clarify usage modes; cli: add /tf selection args
* Auto-transition to active mode after state estimator convergence
* GUI: Add message when lidar stream starts
* Merge pull request `#55 <https://github.com/MOLAorg/mola_erathos_slam/issues/55>`_ from MOLAorg/feat/add-odom-frame-env-var
  ros2 launch: export MOLA_TF_ESTIMATED_ODOMETRY
* ros2 launch: export MOLA_TF_ESTIMATED_ODOMETRY from mola_lo_reference_frame for consistency
* Merge pull request `#54 <https://github.com/MOLAorg/mola_erathos_slam/issues/54>`_ from MOLAorg/fix-missing-ros2-launch-base-link
  BUGFIX: Changing base_link in ros2 launch didn't propagate to LO
* BUGFIX: Changing base_link in ros2 launch didn't propagate to LO
* Merge pull request `#45 <https://github.com/MOLAorg/mola_erathos_slam/issues/45>`_ from MOLAorg/fix/init-from-gps-imu
  Support initialization from state estimator
* Support initialization from state estimator
* Contributors: Jose Luis Blanco-Claraco, Robin Van Cauwenbergh

* Rename the family of *_sensor_* pipeline names that actually meant
  observation-from-``base_link`` to *_observation_radius_* /
  ``*_OBSERVATION_RADIUS`` to clarify semantics. The legacy names remain as
  deprecated aliases so existing pipelines keep working unchanged; a
  one-shot warning is emitted at init when the loaded YAML still references
  any legacy name. The aliases will be removed in a future release.

  - Dynamic variables: ``ESTIMATED_SENSOR_MAX_RANGE`` →
    ``ESTIMATED_OBSERVATION_RADIUS``, ``INSTANTANEOUS_SENSOR_MAX_RANGE`` →
    ``INSTANTANEOUS_OBSERVATION_RADIUS``.
  - YAML param keys: ``max_sensor_range_filter_coefficient`` →
    ``observation_radius_filter_coefficient``, ``absolute_minimum_sensor_range``
    → ``absolute_minimum_observation_radius``.

2.0.0 (2026-04-02)
------------------
* Merge pull request `#53 <https://github.com/MOLAorg/mola_lidar_odometry/issues/53>`_ from MOLAorg/feat/use-imu-grav-align
  Use IMU readings to estimate gravity up vector as ICP prior constraint
* Add formal CLA
* mola-cli-launch files: apply yaml format
* FIX: mola-lidar-odometry-cli now can handle the two types of GPS messages too
* gui stats: more robust sensor rate estimation (include gnss now too)
* docs: add layer name to pipeline table
* gicp pipeline: expose new KF-metric map parameters as optional env vars
* ros2 launch: add new argument 'gpsfix_topic_name' for GPSFix messages
* Merge pull request `#50 <https://github.com/MOLAorg/mola_lidar_odometry/issues/50>`_ from MOLAorg/feat/refactor-gui
  Port to the new backend agnostic GUI API
* Port to the new backend agnostic GUI API
* ROS2 node and ros2bags: support gps_msgs/GpsFix messages too
* On bad ICP, do neither publish or show deskewed scans
* gicp pipeline: expose more env vars for KF map params
* Merge pull request `#48 <https://github.com/MOLAorg/mola_lidar_odometry/issues/48>`_ from MOLAorg/feat/selective-icp-log
  New env var to debug bad ICP cases: MOLA_WRITE_DEBUG_ICP_LOG_IF_QUALITY_UNDER
* New env var to debug bad ICP cases: MOLA_WRITE_DEBUG_ICP_LOG_IF_QUALITY_UNDER
* CSV stats: include icp_quality too
* Merge pull request `#47 <https://github.com/MOLAorg/mola_lidar_odometry/issues/47>`_ from MOLAorg/better-deskew-performance
  More efficient deskew and visualization
* refactor deskew and viz updates
* Ensure no detached threads (`#46 <https://github.com/MOLAorg/mola_lidar_odometry/issues/46>`_)
  * Fix: potential miss error report saving mm files
  * Ensure no detached threads for disk io
  * minor fixes
* fix obsolete usage of ament_target_dependencies()
* Remove no longer needed param period_publish_new_localization (Removed in BridgeROS2)
* Fix: provide a mechanism to read 2D lidar from ROS (`#43 <https://github.com/MOLAorg/mola_lidar_odometry/issues/43>`_)
  * Fix: provide a mechanism to read 2D lidar from ROS
  * docs: add new ros launch argument `lidar_topic_type`
  * doc: better roslaunch arg description
* Fix building against older versions of mrpt & mp2p_icp
* Saving simplemap with lazy-load is done in its own detaled thread
* Merge pull request `#40 <https://github.com/MOLAorg/mola_lidar_odometry/issues/40>`_ from MOLAorg/feat/more-clang-tidy-checks
  Add many more clang-tidy checks
* Remove thread_local aux variable for viz camera rotation
* Increase level of clang-tidy warnings, and fix them
* Copyright year bump
* Merge pull request `#39 <https://github.com/MOLAorg/mola_lidar_odometry/issues/39>`_ from MOLAorg/feat/optional-prefilter-pipeline
  Add optional prefilter pipeline for all ICP methods
* Add many more clang-tidy checks
* Add optional prefilter pipeline for all ICP methods
* Contributors: Jose Luis Blanco-Claraco

1.3.1 (2025-12-29)
------------------
* Merge pull request #36 from MOLAorg/feat/use-generic-map-fields
  Use CGenericPointsMap to propagate all sensor per-point fields thru mapping pipelines
* Support to visualize clouds in MOLA Viz recolorized by any cloud point field
* Use CGenericPointsMap to propagate all sensor per-point fields thru mapping pipelines
* Contributors: Jose Luis Blanco-Claraco

1.3.0 (2025-12-15)
------------------
* Add CI and documentation badges to README
  Added badges for CI build, clang-format, documentation, and code coverage.
* Add option 'use_imu_orientation' to disable using IMU orientation for initialization
* docs: add table clarifying localmap types
* Make it compatible with observations w/o cov
* Discard GPS readings with invalid cov matrix
* Add warning if sensor stamps go backwards in time
* Update usage instructions for mola_lo_apps.rst
  Added usage instructions for LIO with Ouster in mola_lo_apps.rst.
* Contributors: Jose Luis Blanco-Claraco

1.2.2 (2025-11-08)
------------------
* Add quick instructions to launch mola-lo-gui on an Ouster dataset
* Add option to store persistent settings and check version
* Fix: missing publication of LocalMap if BridgeROS2 loads after this module.
* Contributors: Jose Luis Blanco-Claraco

1.2.1 (2025-10-28)
------------------
* Fix build against upcoming mrpt v2.15.0
* Reduce the limit of published points to avoid FoxGlove WS overflow
* docs: show first usage with rosbags & rawlogs
* Contributors: Jose Luis Blanco-Claraco

1.2.0 (2025-10-21)
------------------
* Tune ROS2 publication rates for reduced viz load
* New option 'publish_deskewed_scans'
* Fix unit tests
* ros2 launch: sort arguments
* Contributors: Jose Luis Blanco-Claraco

1.1.0 (2025-10-20)
------------------
* Docs: describe the new GICP and LIO pipelines
* Update rviz settings
* Prefer to publish deskewed clouds in 'map' frame
* FIX: ROS2 interface must use correct cloud and pose timestamps
* Update and fix LIO ROS2 launch demo and docs
* ROS: support rendering deskewed clouds
* Replace deprecated ament_target_dependencies() with pure cmake
* Publish deskewed scans for ROS visualization
* Make use of ConstPtr for processing incoming observations
* Code clean up: remove macros for building against very old mola_kernel versions
* ros2 launch: add argument 'mola_tf_base_link'
* Contributors: Jose Luis Blanco-Claraco

1.0.0 (2025-10-13)
------------------
* Merge pull request #26 from MOLAorg/feature/better-lio
  Better LIO & new GICP pipeline
* CI: Run on ROS testing only
* Add custom 'name' to pipeline stages for profiler
* Update docs: Show example of use for MOLA_TF_BASE_LINK=os_sensor
* Feature: add option to save deskewed clouds
* refactor part of processScan() for code clarity
* fix clang-tidy warnings
* New config flag MOLA_SAVE_MM to save final metric map at session end
* add option to re-colorize clouds by intensity (local map)
* Add a clear message at initialization showing the name of the used pipeline
* Fix macro to detect newer mp2p_icp version
* cli app: show LO pose
* Refactor the way Lidar scans are enqueued depending on LO/LIO usage
* ICP pipelines: renamed old 'default' as 'icp', and add new 'default' symlink pointing to 'gicp'
* cli: use MOLA YAML parser for state estimation files
* Fix kitti eval scripts
* cli: fix expected contents of state estimation param files
* Debug traces: more covariance data
* GUI: Show keyframe stats
* configurable icp quality setpoint
* Fix lidar rate for multiple lidars
* New param to change the color of trajectory in the GUI
* reset local viz clouds when re-localizing
* gicp pipeline: use 2 resolutions (icp / map)
* Auto-scale intensity for visualization
* Less aggressive P controller for adaptive sigma
* Use adaptive sampler
* Update to latest mp2p_icp library API
* Progress optimizing new gicp pipeline
* New GICP pipeline file
* Fix for latest mola imu API changes
* Send velocity and orientations to the local velocity buffer
* Better visualization of current / past clouds, with configurable colormaps from the yaml file
* Move to the new deskew_method flag in mp2p_icp
* README.md: update bibtex reference
* Move IMU initialization to package mola_imu_preintegration
* IMU initializer moved out to the mola_imu_preintegration package for better reusability
* PitchRoll init: Add to-do note on IMU bias
* GUI: show lidar & imu rates
* remove obsolete pipeline
* Fix typos in YAML comments
* Configurable GUI background color
* Implement display dense local map (decaying deskewed clouds)
* Implement visualization of past clouds as transparent, decaying clouds
* Add missing header for latest mola_kernel
* option to show mulran dataset clouds with their real intensity channel
* Visualization: show the deskewed current observation instead of raw
* Contributors: Jose Luis Blanco-Claraco

0.9.0 (2025-08-26)
------------------
* FIX: bug in formula for pitch-roll initialization from IMU
* Store local IMU velocity buffer in key-frame simplemaps
* mola-lidar-odometry-cli: New CLI arguments to support datasets with IMUs
* Implement precise IMU-based deskew (requires latest mp2p_icp library)
* fix clang-format
* Modernize copyright notices
* rosbag2 mola-cli launch file: add `MOLA_ROS2BAG_EXPORT_TO_RAWLOG_FILE` optional env var
* Contributors: Jose Luis Blanco-Claraco

0.8.0 (2025-06-06)
------------------
* Publish mp2p_icp metric map metadata, if existing in loaded maps.
* state estimation config yaml file: expose IMU sensor name env var
* Update mola_lo_pipelines.rst: explicitly show an example of using the NDT pipeline
* ros2 launch: add new argument to control the scan validity filter based on minimum point count (now, enabled by default)
* Update broken link to ROS Index
* mola-lidar-odometry-cli: now also forward raw sensor data to state estimator
* Fix build against mola <1.8.0
* Docs: better explain existing variables to override sensor poses
* gui option: implement show as orthographic camera
* Contributors: Jose Luis Blanco-Claraco

0.7.3 (2025-05-25)
------------------
* feature: new threshold to discard state estimation as invalid if uncertainty is too high
* Fixed unit tests in CI
* Prepare GUI for ortho camera option
* progress implementing init pitch/roll from IMU
* pipelines YAML files reformated with RedHat YAML formatter
* Update env var name to explicitly mention LO: MOLA_LO_INITIAL_LOCALIZATION_METHOD
* docs: on initial localization methods
* ROS2 launch: Add new `mola_state_estimator_reference_frame` argument.
  It should be used together with `mola_lo_reference_frame` to use an alternative reference map TF frame than the default `map`.
* Fix wrong namespace in class name (it worked anyway because of a fall-back mechanism using unqualified names)
* Expose env vars to change the reference frame_id for smoother (MOLA_TF_MAP)
* fix: potential missing publication of updated poses if there is no map subscriber
* lidar 3d pipeline: add rendering options for local map
* Contributors: Jose Luis Blanco-Claraco

0.7.2 (2025-04-23)
------------------
* better integration of clang-tidy, colcon_defaults, and clangd with vscode
* Expose two more env vars: MOLA_MAP_CLOUD_DECIMATION, MOLA_ICP_CLOUD_DECIMATION
* FIX: also initial pose for localmap
* BUGFIX: Initial twist was wrong for custom initial poses
* Contributors: Jose Luis Blanco-Claraco

0.7.1 (2025-03-15)
------------------
* FIX: Handle correctly the case of input scans with non-normal numbers
* docs: format of ros2 launch argument
* FIX: reset map to start again might lead to divergence; Add new 'reset_state' command via MOLA dynamic variables
* Force requiring valid poses for IMU and GNSS inputs
* Refactor implementation source into several smaller files
* FIX: mola-lo didn't exit due to waiting ICP queue if fed faster than ICP processing
* FIX: mola-lo-gui apps may show duplicated UI controls in particular circumstances
* Drop frames warning message now tells the exact drop ratio
* Initial localization method is now loadable from yaml or ros2 launch file
* MOLA-LO no longer subscribes to wheels odometry. That is now delegated directly to state estimation modules.
* Add new ROS2 launch argument: `forward_ros_tf_odom_to_mola`
* Contributors: Jose Luis Blanco-Claraco

0.7.0 (2025-02-22)
------------------
* Implement new mola_kernel diagnostics API
* Ensure map is published after ROS2 bridge is already listening (FIXES: potential loss of map publication if MM map is given via env var)
* FIX: Proper configurable dropped frames mechanism and stats
* FIX: Update GUI, publish maps, correctly independently of whether MolaGUI is enabled
* launch: fix localization source name
* FIX: Do not ever reset the map when in localization mode
* Fix: refresh GUI with initial map
* Allow dropping LiDAR frames in too slow for real-time, but not any other observation type
* FIX: ensure georef metadata is published when map_load service is called
* rename kitti ros2 demo file to unclutter ros2 launch autocompletion
* Add ros launch argument 'use_state_estimator'
* FIX: publish georeferencing metadata at start up
* Add ROS2 launch arguments to select an state_estimator method
* update citation
* Add more params to smoother state estimation default YAML file
* Add env variable MOLA_STATE_ESTIMATOR_PUBLISH_RATE to control filtered pose update rate
* Add new env var MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION and ros2 launch argument for it
* Add new ros launch argument mola_footprint_to_base_link_tf
* Fix expected pose format in yaml
* ROS2 launch: shutdown if mvsim crashes
* Fix parse error with default .mm and .simplemap launch arguments
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

0.6.2 (2025-02-13)
------------------
* ros2 launch: add .mm and .simplemap optional initial map arguments
* All exhaustive docs on ros2-related mola launch YAML files with the meaning of all BridgeROS2 parameter
* Delegate publishing georeference info to BridgeROS2
* Contributors: Jose Luis Blanco-Claraco

0.6.1 (2025-01-26)
------------------
* Do not re-publish the map if it does not change, e.g. in localization-only mode
* ros2 launch file: two new arguments 'mola_lo_pipeline' and 'generate_simplemap'
* Default 3D-LO pipeline: Add new env var 'MOLA_LOCALMAP_LAYER_NAME', useful when localizing with prebuilt maps
* Merge pull request #12 from r-aguilera/develop
  fix launch file params
* fix launch file params
* Contributors: Jose Luis Blanco-Claraco, Raúl Aguilera

0.6.0 (2025-01-21)
------------------
* Fix: publish map on first iteration
* Publish georeferencing frames (utm, enu) when loading a metric map with georef. info
* ros2 lidar odometry launch: add ros argument for /tf reference_frame
* ROS2 kitti Lidar-Odometry demo: fixed to publish correct /tf's
* Add new frame parameters to pipeline YAML files
* Two new parameters (publish_reference_frame, publish_vehicle_frame), to have explicit control on frame names published to both, ROS, and the MOLA state_estimator
* ROS2 service call for load_map(): more concise error messages
* Contributors: Jose Luis Blanco-Claraco

0.5.4 (2025-01-16)
------------------
* Add a debug helper env var MOLA_BRIDGE_ROS2_EXPORT_TO_RAWLOG_FILE
* Do not reset the state estimator on a bad ICP, allowing merging from other sensors or extrapolating.
* Docs: add missing ros2 launch args
* More ROS2 launch arguments
* Contributors: Jose Luis Blanco-Claraco

0.5.3 (2025-01-15)
------------------
* FIX: mola_state_estimator_simple must be available as a build dep too for easier usage of mola-lo-cli
* Contributors: Jose Luis Blanco-Claraco

0.5.2 (2025-01-11)
------------------
* Merge pull request #11 from MOLAorg/10-bad-first-icp-re-starting-from-scratch-with-a-new-local-map
  Fix NaN pointcloud radius in doInitializeEstimatedMaxSensorRange()
* Unit tests: add test run against MulRan dataset fragment (Lidar+IMU)
* cli: fix name of example pipeline file when --help invoked
* unit tests: fix wrong usage of state estimator yaml file
* mola-lo-gui-mulran: show IMU & GPS data in GUI
* Define a sensible value for maxRange
* Fix cmake warning when built w/o mola_state_estimation_simple sourced in the env
* Contributors: Jose Luis Blanco-Claraco

0.5.1 (2025-01-07)
------------------
* mola-lidar-odometry-cli: add flags to select the state estimation method
* Contributors: Jose Luis Blanco-Claraco

0.5.0 (2024-12-29)
------------------
* cmake test logic: add find_package() for state_estimation_simple
* Merge pull request #7 from MOLAorg/wip/new-state-estimators
  New state estimators (Merge after MOLA 1.5.0 is installable via apt)
* Split state estimation params so each implementation has its own yaml file
* CI: build against both, ROS testing and stable
* Add new state estimator module in all MOLA-CLI yaml files
* Update to new state estimation packages
* Reorganization such as state estimator is now an independent external module
* docs: add new ros-arg publish_localization_following_rep105
* FIX: publish local map even when not active
* Contributors: Jose Luis Blanco-Claraco

0.4.1 (2024-12-20)
------------------
* ROS2 launch: add ros argument for new option publish_localization_following_rep105
* rviz2 demo file: better orbit view
* ROS2 config file: define env vars for all tf frames (odom, map, base_link)
* Contributors: Jose Luis Blanco-Claraco

0.4.0 (2024-12-18)
------------------
* demo rviz file: fix lidar topic name
* Include /tf remaps too in ros2 launch
* mola launch for ROS 2: Add placeholder for ros args parsing
* mola launch for ROS 2: add env variables to quickly control verbosity of each module.
  Env. vars. are:  MOLA_VERBOSITY_MOLA_VIZ, MOLA_VERBOSITY_MOLA_LO,MOLA_VERBOSITY_BRIDGE_ROS2 (Default: INFO)
* Support for ROS2 namespaces in launch file
* docs; and fix launch var typo
* ROS 2 launch: add more ros args
* move MOLA-LO ROS2 docs to the main MOLA repo
* Expose one more runtime param: generate_simplemap
* clarify docs on sensor input topic names
* runtime parameters: update in GUI too
* publish ICP quality as part of localization updates
* mola module name changed: 'icp_odom' -> 'lidar_odom'
* Do not publish localization if ICP is not good
* Expose runtime parameters using MOLA v1.4.0 configurable parameters: active, mapping_enabled
* docs clarifications
* map_load service: allow not having a .simplemap file and don't report it as an error
* FIX: motion model handling during re-localization
* Implement map_save
* reset adaptive sigma upon relocalization
* Implement map_load; Implement relocalize around pose
* Forward IMU readings to the navstate fusion module
* CI and readme: remove ROS2 iron
* Merge branch 'wip/map_load_save' into develop
* docs: add ref to yaml extensions
* Add docs on 3D-NDT pipeline and demo usage with Mulran
* parameterize maximum_sigma
* CLI: add flag to retrieve all twists in a file; avoid use of "static" variables
* LO: Add a getter for the latest pose and twist
* doc: explain "no tf" error message
* tune 3D-NDT defaults
* Kitti and Mulran evaluation scripts: extend so they can be run with other pipelines
* ros2 launch: Add 'use_rviz' argument
* NDT pipeline: expose max sigma as parameter too
* Avoid anoying warning message when not really needed
* Extend options for GNSS initialization
* Add docs on mola-lo-gui-rawlog
* Default pipeline: reduce density of keyframes in simplemap
* Docs: mola_lo_apps.rst fix PIPELINE_YAML var name
* Update mola_lo_pipelines.rst: fix format
* recover passing var args to mola-lo-gui-rosbag2 script
* UI: show instantaneous max. sensor range too
* FIX: formula for the estimated max. sensor range fixed for asymmetric cases
* add new visualization param ground_grid_spacing
* viz: grow ground grid as the local map grows
* FIX: disabling visualization of raw observations left last raw observation rendered
* fix: separate GPS topic and sensorLabel variables
* Consistent GPS topic name
* Add another env variable: MOLA_LOCAL_VOXELMAP_RESOLUTION
* Expose new param for local map max size
* enable the relocalize API
* Expose fixed sensor pose coords as optional env variables
* Readme: add ROS badges for arm64 badges
* GitHub actions: use ROS2-testing packages
* Contributors: Jose Luis Blanco-Claraco

0.3.3 (2024-09-01)
------------------
* default 3D pipeline: Expose a couple more parameters as env variables
* Depend on new mrpt_lib packages (deprecate mrpt2)
* Contributors: Jose Luis Blanco-Claraco

0.3.2 (2024-08-26)
------------------
* Support input dataset directories for split bags
* Contributors: Jose Luis Blanco-Claraco

0.3.1 (2024-08-22)
------------------
* add missing exec dependencies to package.xml for mola-lo-* commands.
* Contributors: Jose Luis Blanco-Claraco

0.3.0 (2024-08-14)
------------------
* First public release
* Contributors: Jose Luis Blanco-Claraco
