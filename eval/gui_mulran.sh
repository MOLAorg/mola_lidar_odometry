#!/usr/bin/env bash

if [ "$#" -eq 0 ]; then
    echo "Error: At least 1 argument is required."
    echo "Usage: $0 <MULRAN_SEQ> [additional flags for mola-cli]"
    echo "With <KITTI_SEQ> 00 or 01 or 02..."
    exit 1
fi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

PIPELINE_YAML="${PIPELINE_YAML:-$SCRIPT_DIR/../params/lidar-inertial-pipeline-simple.yaml}"

SEQ=$1
shift 1

MULRAN_SEQ=$SEQ \
MOLA_ODOMETRY_PIPELINE_YAML=$PIPELINE_YAML \
  mola-cli \
    $SCRIPT_DIR/../mola-cli-launchs/lidar_odometry_from_mulran.yaml \
    $@
