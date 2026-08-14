# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2026, Jose Luis Blanco-Claraco
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.22...3.31)  # Ubuntu 22.04+

if("$ENV{ROS_VERSION}" STREQUAL "2")
    set(DETECTED_ROS2 TRUE)
endif()

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_lidar_odometry LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find dependencies:
find_package(mrpt-maps REQUIRED)
find_package(CLI11 REQUIRED)
find_package(mola_kernel REQUIRED)
find_package(mola_yaml REQUIRED)
find_package(mp2p_icp REQUIRED)
find_package(mp2p_icp_filters REQUIRED)
find_package(mola_pose_list REQUIRED)
find_package(mola_imu_preintegration REQUIRED)

# -----------------------
# define lib:
set(LIB_SRCS
  module/src/GravityMapAligner.cpp
  module/src/KeyframeDecider.cpp
  module/src/LidarOdometry_Main.cpp
  module/src/MapFrameRelevel.cpp
  module/src/LidarOdometry_Relocalization.cpp
  module/src/LidarOdometry_Parameters.cpp
  module/src/LidarOdometry_Initialize.cpp
  module/src/LidarOdometry_ProcessScan.cpp
  module/src/LidarOdometry_AdaptiveVariables.cpp
  module/src/LidarOdometry_MapServer.cpp
  module/src/LidarOdometry_GUI.cpp
  module/src/LidarOdometry_SensorCallbacks.cpp
  module/src/LidarOdometry_Publish.cpp
  module/src/LidarOdometry_InitialLocalization.cpp
  # Last:
  module/src/register.cpp
)

set(LIB_PUBLIC_HDRS
  module/include/mola_lidar_odometry/LidarOdometry.h
  module/include/mola_lidar_odometry/GravityMapAligner.h
  module/include/mola_lidar_odometry/MapFrameRelevel.h
)

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::maps
    mola::mola_kernel
    mola::mola_yaml
    mola::mp2p_icp
    mola::mp2p_icp_filters
    mola::mola_pose_list
    mola::mola_imu_preintegration
# PRIVATE_LINK_LIBRARIES
#  mrpt::maps
  CMAKE_DEPENDENCIES
    mola_kernel
    mola_yaml
    mp2p_icp
    mp2p_icp_filters
    mrpt-maps
)

# mola_add_library() expects headers under "ROOT/include", so do this manually here:
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src
)
install(
  DIRECTORY module/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

mola_version_to_hexadecimal(MP2P_ICP_VERSION_HEX "${mp2p_icp_VERSION}")
target_compile_definitions(${PROJECT_NAME} PRIVATE MP2P_ICP_VERSION=${MP2P_ICP_VERSION_HEX})

get_target_property(MY_VERSION ${PROJECT_NAME} VERSION)
target_compile_definitions(${PROJECT_NAME} PRIVATE MOLA_LO_VERSION=\"${MY_VERSION}\")

option(ENABLE_ONLINE_VERSION_CHECK "Enable checking for new versions" ON)
if(ENABLE_ONLINE_VERSION_CHECK)
    target_compile_definitions(${PROJECT_NAME}  PRIVATE MOLA_LO_HAS_ONLINE_VERSION_CHECK)
endif()

# CLI apps
# ---------------------------------
find_package(mola_input_kitti_dataset) # optional
find_package(mola_input_kitti360_dataset) # optional
find_package(mola_input_mulran_dataset) # optional
find_package(mola_input_rawlog) # optional
find_package(mola_input_rosbag2) # optional
find_package(mola_input_rosbag1) # optional
find_package(mola_input_paris_luco_dataset) # optional
find_package(mola_state_estimation_simple QUIET) # optional
find_package(mola_state_estimation_smoother QUIET) # optional

mola_add_executable(
  TARGET  mola-lidar-odometry-cli
  SOURCES apps/mola-lidar-odometry-cli.cpp
  LINK_LIBRARIES
  ${PROJECT_NAME}
  mrpt::maps
  CLI11::CLI11
)
target_compile_definitions(mola-lidar-odometry-cli PRIVATE MOLA_LO_VERSION=\"${MY_VERSION}\")

if(mola_state_estimation_simple_FOUND)
  message(STATUS "mola-lidar-odometry-cli: With mola_state_estimation_simple = YES")
  target_link_libraries(mola-lidar-odometry-cli mola::mola_state_estimation_simple)
  target_compile_definitions(mola-lidar-odometry-cli PRIVATE HAVE_MOLA_SE_SIMPLE)
else()
  message(STATUS "mola-lidar-odometry-cli: With mola_state_estimation_simple = NO")
endif()

# Not selected as a default/fallback estimator like mola_state_estimation_simple
# above (no HAVE_MOLA_SE_SMOOTHER define needed for that): linking it in is
# what's required so its class is registered with MRPT's RTTI and therefore
# reachable via the CLI's own --state-estimator <classname> flag
# (mrpt::rtti::classFactory() finds any class whose translation unit is
# actually linked into the binary, generic mechanism, no compile-time
# branching needed on the caller's side). Without this, --state-estimator
# mola::state_estimation_smoother::StateEstimationSmoother fails at runtime
# with "Apparently unknown class name ... (missing plugin .so file?)" even
# though the flag itself is accepted -- found running the first-ever
# offline+smoother (cfg-02) sweep through this pipeline, 2026-08-11.
if(mola_state_estimation_smoother_FOUND)
  message(STATUS "mola-lidar-odometry-cli: With mola_state_estimation_smoother = YES")
  # mola_state_estimation_smoother's exported target only records its GTSAM
  # link dependency by bare name (CMake's legacy
  # IMPORTED_LINK_DEPENDENT_LIBRARIES mechanism, not a namespaced imported
  # target), so the gtsam/gtsam_unstable targets it references must already
  # exist in THIS scope too, or the final link fails with "cannot find
  # -lgtsam" despite the .so already carrying its own GTSAM dependency
  # privately. find_package(GTSAM) here is what defines them.
  find_package(GTSAM QUIET)
  find_package(GTSAM_UNSTABLE QUIET)
  # --no-as-needed around this one: nothing in this CLI's own source
  # references a symbol from mola_state_estimation_smoother directly (its
  # class is only ever reached indirectly, by name, through
  # mrpt::rtti::classFactory() at runtime -- unlike
  # mola_state_estimation_simple above, which IS called directly as a
  # compile-time fallback and so survives linking as-is). Without this, the
  # default --as-needed linker behavior sees no referenced symbol and drops
  # the .so from the binary's DT_NEEDED entries entirely, silently
  # reproducing the exact "missing plugin .so file?" runtime error this
  # whole block exists to fix -- confirmed via ldd, which showed the
  # smoother absent even after the .so linked and built without error.
  target_link_libraries(mola-lidar-odometry-cli
    "-Wl,--no-as-needed"
    mola::mola_state_estimation_smoother
    "-Wl,--as-needed"
  )
else()
  message(STATUS "mola-lidar-odometry-cli: With mola_state_estimation_smoother = NO")
endif()

if(mola_input_kitti_dataset_FOUND)
  message(STATUS "mola-lidar-odometry-cli: With Kitti = YES")
  target_link_libraries(mola-lidar-odometry-cli mola::mola_input_kitti_dataset)
  target_compile_definitions(mola-lidar-odometry-cli PRIVATE HAVE_MOLA_INPUT_KITTI)
else()
  message(STATUS "mola-lidar-odometry-cli: With Kitti = NO")
endif()

if(mola_input_kitti360_dataset_FOUND)
  message(STATUS "mola-lidar-odometry-cli: With Kitti360 = YES")
  target_link_libraries(mola-lidar-odometry-cli mola::mola_input_kitti360_dataset)
  target_compile_definitions(mola-lidar-odometry-cli PRIVATE HAVE_MOLA_INPUT_KITTI360)
else()
  message(STATUS "mola-lidar-odometry-cli: With Kitti360 = NO")
endif()

if(mola_input_mulran_dataset_FOUND)
  message(STATUS "mola-lidar-odometry-cli: With Mulran = YES")
  target_link_libraries(mola-lidar-odometry-cli mola::mola_input_mulran_dataset)
  target_compile_definitions(mola-lidar-odometry-cli PRIVATE HAVE_MOLA_INPUT_MULRAN)
else()
  message(STATUS "mola-lidar-odometry-cli: With Mulran = NO")
endif()

if(mola_input_rawlog_FOUND)
  message(STATUS "mola-lidar-odometry-cli: With Rawlog = YES")
  target_link_libraries(mola-lidar-odometry-cli mola::mola_input_rawlog)
  target_compile_definitions(mola-lidar-odometry-cli PRIVATE HAVE_MOLA_INPUT_RAWLOG)
else()
  message(STATUS "mola-lidar-odometry-cli: With Rawlog = NO")
endif()

if(mola_input_rosbag2_FOUND)
  message(STATUS "mola-lidar-odometry-cli: With rosbag2 = YES")
  target_link_libraries(mola-lidar-odometry-cli mola::mola_input_rosbag2)
  target_compile_definitions(mola-lidar-odometry-cli PRIVATE HAVE_MOLA_INPUT_ROSBAG2)
else()
  message(STATUS "mola-lidar-odometry-cli: With rosbag2 = NO")
endif()

if(mola_input_rosbag1_FOUND)
  message(STATUS "mola-lidar-odometry-cli: With rosbag1 = YES")
  target_link_libraries(mola-lidar-odometry-cli mola::mola_input_rosbag1)
  target_compile_definitions(mola-lidar-odometry-cli PRIVATE HAVE_MOLA_INPUT_ROSBAG1)
else()
  message(STATUS "mola-lidar-odometry-cli: With rosbag1 = NO")
endif()

if(mola_input_paris_luco_dataset_FOUND)
  message(STATUS "mola-lidar-odometry-cli: With paris_luco = YES")
  target_link_libraries(mola-lidar-odometry-cli mola::mola_input_paris_luco_dataset)
  target_compile_definitions(mola-lidar-odometry-cli PRIVATE HAVE_MOLA_INPUT_PARIS_LUCO)
else()
  message(STATUS "mola-lidar-odometry-cli: With paris_luco = NO")
endif()


# Install files:
install(
  DIRECTORY
    config
    mola-cli-launchs
    pipelines
    ros2-launchs
    rviz2
    state-estimator-params
  DESTINATION
    share/${PROJECT_NAME}
)

# The per-dataset knowledge behind all the wrappers below. They locate it
# relative to themselves, so it has to land in share/ under this exact name.
install(
  DIRECTORY
    scripts/lib/
  DESTINATION
    share/${PROJECT_NAME}/scripts-lib
)

# Install the dataset wrappers too. Every dataset gets a "gui" one (online
# replay through mola-cli) and a "cli" one (offline batch through
# mola-lidar-odometry-cli); both are thin front-ends over the same profile in
# scripts-lib/profiles/. Ouster is gui-only: it is a live source, which the
# offline CLI cannot read.
set(MOLA_LO_DATASET_WRAPPERS
    botanicgarden
    citrusfarm
    conslam
    grandtour
    kitti
    kitti360
    mulran
    oxford-spires
    paris-luco
    rawlog
    rosbag1
    rosbag2
    # TIERS records five lidars at once, so it gets one wrapper per sensor
    # rather than one that silently picks a winner:
    tiers-ouster-os0
    tiers-ouster-os1
    tiers-velodyne
    tiers-livox-horizon
    tiers-livox-avia
)

set(MOLA_LO_WRAPPER_PATHS scripts/mola-lo-gui-ouster)
foreach(ds ${MOLA_LO_DATASET_WRAPPERS})
  list(APPEND MOLA_LO_WRAPPER_PATHS
       scripts/mola-lo-gui-${ds}
       scripts/mola-lo-cli-${ds})
endforeach()

install(
  PROGRAMS
    ${MOLA_LO_WRAPPER_PATHS}
  DESTINATION
    bin
)

# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if(DETECTED_ROS2)
  # find dependencies
  find_package(ament_cmake REQUIRED)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    ament_lint_auto_find_test_dependencies()

    add_subdirectory(test)
  endif()

  ament_package()
endif()
