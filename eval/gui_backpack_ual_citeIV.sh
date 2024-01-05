#!/usr/bin/env bash

MOLA_INPUT_RAWLOG=/mnt/storage/ual-datasets/2023-11-23_Mochila3D_CiteIV_small_loop/2023-11-23_CiteIV_small_loop.rawlog \
MOLA_ODOMETRY_PIPELINE_YAML=$HOME/ros2_ws/src/mola_lidar_odometry/params/lidar-inertial-pipeline-simple.yaml \
  mola-cli \
    $HOME/ros2_ws/src/mola_lidar_odometry/mola-cli-launchs/lidar_odometry_from_rawlog.yaml
