^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_lidar_odometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
