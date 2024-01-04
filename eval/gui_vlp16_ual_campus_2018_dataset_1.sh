#!/usr/bin/env bash

MOLA_INPUT_RAWLOG=/mnt/storage/ual-datasets/ecarm_2018_02_23_velodyne_rtk_campus/2018-02-23_merged_lidar_only.rawlog \
MOLA_ODOMETRY_PIPELINE_YAML=$HOME/ros2_ws/src/mola_lidar_odometry/params/lidar-inertial-pipeline-simple.yaml \
MOLA_TIME_WARP=2.0 \
MOLA_LIDAR_NAME=Velodyne1_SCAN \
  mola-cli \
    -c $HOME/ros2_ws/src/mola_lidar_odometry/mola-cli-launchs/lidar_odometry_from_rawlog.yaml $@
