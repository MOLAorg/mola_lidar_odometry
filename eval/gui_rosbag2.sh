#!/usr/bin/env bash

if [ "$#" -eq 0 ]; then
    echo "Error: At least 1 argument is required."
    echo "Usage: $0 /path/to/dataset.mcap [additional flags for mola-cli]"
    echo "You can set env var MOLA_LIDAR_TOPIC to define the lidar topic name (default: '/ouster/points')"
    exit 1
fi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

FILE=$1
shift 1

MOLA_INPUT_ROSBAG2=$FILE \
MOLA_ODOMETRY_PIPELINE_YAML=${MOLA_ODOMETRY_PIPELINE_YAML:-$SCRIPT_DIR/../params/lidar-inertial-pipeline-simple.yaml} \
  mola-cli \
    $SCRIPT_DIR/../mola-cli-launchs/lidar_odometry_from_rosbag2.yaml \
    $@
