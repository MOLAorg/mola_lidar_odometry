#!/usr/bin/env bash

if [ "$#" -eq 0 ]; then
    echo "Error: At least 1 argument is required."
    echo "Usage: $0 /path/to/dataset.mcap [additional flags for mola-cli]"
    echo "You can set env var MOLA_LIDAR_TOPIC to define the lidar topic name (default: '/ouster/points')"
    exit 1
fi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

MOLA_LAUNCHS_DIR=$SCRIPT_DIR/../mola-cli-launchs/
if [ -d $SCRIPT_DIR/../share/mola_lidar_odometry/mola-cli-launchs ]; then
  MOLA_LAUNCHS_DIR=$SCRIPT_DIR/../share/mola_lidar_odometry/mola-cli-launchs
fi

FILE=$1
shift 1

MOLA_ODOMETRY_PIPELINE_YAML="${PIPELINE_YAML:-$MOLA_LAUNCHS_DIR/../params/lidar3d-default.yaml}" \
MOLA_INPUT_ROSBAG2=$FILE \
  mola-cli \
    $MOLA_LAUNCHS_DIR/lidar_odometry_from_rosbag2.yaml \
    $@
