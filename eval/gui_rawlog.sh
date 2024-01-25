#!/usr/bin/env bash

if [ "$#" -eq 0 ]; then
    echo "Error: At least 1 argument is required."
    echo "Usage: $0 /path/to/dataset.rawlog [additional flags for mola-cli]"
    echo "You can set env var MOLA_LIDAR_NAME to define the lidar topic sensor label (default: 'lidar')"
    exit 1
fi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

FILE=$1
shift 1

MOLA_INPUT_RAWLOG=$FILE \
  mola-cli \
    $SCRIPT_DIR/../mola-cli-launchs/lidar_odometry_from_rawlog.yaml \
    $@
