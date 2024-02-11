#!/usr/bin/env bash

if [ "$#" -eq 0 ]; then
    echo "Error: At least 1 argument is required."
    echo "Usage: $0 <MULRAN_SEQ> [additional flags for mola-cli]"
    echo "With <MULRAN_SEQ> KAIST01, DCC01, ..."
    exit 1
fi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

PIPELINE_YAML="${PIPELINE_YAML:-$SCRIPT_DIR/../params/lidar3d-default.yaml}"

SEQ=$1
shift 1

MULRAN_SEQ=$SEQ \
  mola-cli \
    $SCRIPT_DIR/../mola-cli-launchs/lidar_odometry_from_mulran.yaml \
    $@
