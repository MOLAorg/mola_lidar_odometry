#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Error: Exactly 1 argument is required."
    echo "Usage: $0 <MULRAN_SEQ>"
    echo "With <MULRAN_SEQ> KAIST01 or DCC01 or..."
    exit 1
fi

PIPELINE_YAML="${PIPELINE_YAML:-$HOME/ros2_ws/src/mola_lidar_odometry/params/lidar-inertial-pipeline-simple.yaml}"

MULRAN_SEQ=$1 \
MOLA_ODOMETRY_PIPELINE_YAML=$PIPELINE_YAML \
  mola-cli \
    -c $HOME/ros2_ws/src/mola_lidar_odometry/mola-cli-launchs/lidar_odometry_from_mulran.yaml
