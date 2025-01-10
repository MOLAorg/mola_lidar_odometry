# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2024, Jose Luis Blanco-Claraco
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.5)

if("$ENV{ROS_VERSION}" STREQUAL "2")
    set(DETECTED_ROS2 TRUE)
endif()

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_lidar_odometry LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find dependencies:
find_package(mrpt-maps REQUIRED)
find_package(mrpt-tclap REQUIRED)
find_mola_package(mola_kernel)
find_mola_package(mp2p_icp)
find_mola_package(mp2p_icp_filters)
find_mola_package(mola_pose_list)

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS module/src/*.cpp module/src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS module/include/*.h)

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::maps
    mola::mola_kernel
    mola::mp2p_icp
    mola::mp2p_icp_filters
    mola::mola_pose_list
# PRIVATE_LINK_LIBRARIES
#  mrpt::maps
  CMAKE_DEPENDENCIES
    mola_kernel
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

# CLI apps
# ---------------------------------
find_package(mola_input_kitti_dataset) # optional
find_package(mola_input_kitti360_dataset) # optional
find_package(mola_input_mulran_dataset) # optional
find_package(mola_input_rawlog) # optional
find_package(mola_input_rosbag2) # optional
find_package(mola_input_paris_luco_dataset) # optional
find_package(mola_state_estimation_simple QUIET) # optional

mola_add_executable(
  TARGET  mola-lidar-odometry-cli
  SOURCES apps/mola-lidar-odometry-cli.cpp
  LINK_LIBRARIES
  ${PROJECT_NAME}
  mrpt::maps
  mrpt::tclap
)
if(mola_state_estimation_simple_FOUND)
  message(STATUS "mola-lidar-odometry-cli: With mola_state_estimation_simple = YES")
  target_link_libraries(mola-lidar-odometry-cli mola::mola_state_estimation_simple)
  target_compile_definitions(mola-lidar-odometry-cli PRIVATE HAVE_MOLA_SE_SIMPLE)
else()
  message(STATUS "mola-lidar-odometry-cli: With mola_state_estimation_simple = NO")
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
    mola-cli-launchs
    pipelines
    ros2-launchs
    rviz2
    state-estimator-params
  DESTINATION
    share/${PROJECT_NAME}
)

# Install gui-* demo "executables" too:
install(
  PROGRAMS
    scripts/mola-lo-gui-kitti
    scripts/mola-lo-gui-kitti360
    scripts/mola-lo-gui-mulran
    scripts/mola-lo-gui-rawlog
    scripts/mola-lo-gui-rosbag2
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
