# mola_lidar_odometry repository — AI Agent Context Guide

## Project Overview

**MOLA** (Modular Optimization framework for Localization and mApping) is a modern C++ and ROS 2 framework for robot **localization and SLAM** (Simultaneous Localization and Mapping).

This repository provides a LiDAR-Inertial Odometry (LIO) frontend for the MOLA framework. 

Official Docs: https://docs.mola-slam.org/latest/

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

