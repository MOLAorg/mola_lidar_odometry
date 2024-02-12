#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

MOLA_LAUNCHS_DIR=$SCRIPT_DIR/../mola-cli-launchs/
if [ -d $SCRIPT_DIR/../share/mola_lidar_odometry/mola-cli-launchs ]; then
  MOLA_LAUNCHS_DIR=$SCRIPT_DIR/../share/mola_lidar_odometry/mola-cli-launchs
fi

MOLA_ODOMETRY_PIPELINE_YAML="${PIPELINE_YAML:-$MOLA_LAUNCHS_DIR/../params/lidar3d-default.yaml}" \
mola-cli \
  $MOLA_LAUNCHS_DIR/lidar_odometry_from_paris_luco.yaml \
  $@
