#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

PIPELINE_YAML="${PIPELINE_YAML:-$SCRIPT_DIR/../params/lidar3d-default.yaml}"

mola-cli \
  $SCRIPT_DIR/../mola-cli-launchs/lidar_odometry_from_paris_luco.yaml \
  $@
