#!/usr/bin/env bash

if [ "$#" -eq 0 ]; then
    echo "Error: At least 1 argument is required."
    echo "Usage: $0 <KITTI_SEQ> [additional flags for mola-cli]"
    echo "With <KITTI_SEQ> 00 or 01 or 02..."
    exit 1
fi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

MOLA_LAUNCHS_DIR=$SCRIPT_DIR/../mola-cli-launchs/
if [ -d $SCRIPT_DIR/../share/mola_lidar_odometry/mola-cli-launchs ]; then
  MOLA_LAUNCHS_DIR=$SCRIPT_DIR/../share/mola_lidar_odometry/mola-cli-launchs
fi

SEQ=$1
shift 1

KITTI_SEQ=$SEQ \
MOLA_ODOMETRY_PIPELINE_YAML="${PIPELINE_YAML:-$MOLA_LAUNCHS_DIR/../params/lidar3d-default.yaml}" \
MOLA_INITIAL_VX=20.0 \
  mola-cli \
    $MOLA_LAUNCHS_DIR/lidar_odometry_from_kitti.yaml \
    $@
