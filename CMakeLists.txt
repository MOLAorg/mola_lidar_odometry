# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2023, Jose Luis Blanco-Claraco
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.4)

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
find_mola_package(mola_imu_preintegration)

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

mola_add_library(
	TARGET ${PROJECT_NAME}
	SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
	PUBLIC_LINK_LIBRARIES
		mrpt::maps
		mola_kernel
		mp2p_icp
		mp2p_icp_filters
		mola_imu_preintegration
#	PRIVATE_LINK_LIBRARIES
#		mrpt::maps
	CMAKE_DEPENDENCIES
		mola_kernel
		mp2p_icp
		mp2p_icp_filters
		mola_imu_preintegration
		mrpt-maps
)

# CLI apps
# ---------------------------------
mola_add_executable(
	TARGET  mola-lidar-odometry-cli
	SOURCES apps/mola-lidar-odometry-cli.cpp
	LINK_LIBRARIES
		${PROJECT_NAME}
		mrpt::core
		mrpt::tclap
)

# -----------------------
# define tests:
enable_testing()
add_subdirectory(tests)


# Install files:
install(DIRECTORY
  launch
  params
  DESTINATION
    share/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if (DETECTED_ROS2)
	# find dependencies
	find_package(ament_cmake REQUIRED)

	#ament_target_dependencies(MY_TARGET
	#  rclcpp...
	#)

	ament_package()
endif()
