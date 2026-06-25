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

