#!/usr/bin/env bash

MOLA_INPUT_RAWLOG=/mnt/storage/ual-ouster-backpack/2023-11-23_citeIV_small_loop/2023-11-23_citeIV_small_loop.rawlog \
  mola-cli \
    $HOME/ros2_ws/src/mola_lidar_odometry/mola-cli-launchs/lidar_odometry_from_rawlog.yaml
