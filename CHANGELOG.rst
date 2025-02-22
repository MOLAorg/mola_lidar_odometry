^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_lidar_odometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
