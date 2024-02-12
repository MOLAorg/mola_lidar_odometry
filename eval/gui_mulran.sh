#!/usr/bin/env bash

if [ "$#" -eq 0 ]; then
    echo "Error: At least 1 argument is required."
    echo "Usage: $0 <MULRAN_SEQ> [additional flags for mola-cli]"
    echo "With <MULRAN_SEQ> KAIST01, DCC01, ..."
    exit 1
fi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

MOLA_LAUNCHS_DIR=$SCRIPT_DIR/../mola-cli-launchs/
if [ -d $SCRIPT_DIR/../share/mola_lidar_odometry/mola-cli-launchs ]; then
  MOLA_LAUNCHS_DIR=$SCRIPT_DIR/../share/mola_lidar_odometry/mola-cli-launchs
fi

SEQ=$1
shift 1

MOLA_ODOMETRY_PIPELINE_YAML="${PIPELINE_YAML:-$MOLA_LAUNCHS_DIR/../params/lidar3d-default.yaml}" \
MULRAN_SEQ=$SEQ \
  mola-cli \
    $MOLA_LAUNCHS_DIR/lidar_odometry_from_mulran.yaml \
    $@
