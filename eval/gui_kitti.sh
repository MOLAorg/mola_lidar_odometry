#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Error: Exactly 1 argument is required."
    echo "Usage: $0 <KITTI_SEQ>"
    echo "With <KITTI_SEQ> 00 or 01 or 02..."
    exit 1
fi

KITTI_SEQ=$1 \
MOLA_ODOMETRY_PIPELINE_YAML=$HOME/ros2_ws/src/mola_lidar_odometry/params/lidar-inertial-pipeline-simple.yaml \
  mola-cli \
    -c $HOME/ros2_ws/src/mola_lidar_odometry/mola-cli-launchs/lidar_odometry_from_kitti.yaml
