#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Default pipeline YAML file:
PIPELINE_YAML="${PIPELINE_YAML:-$SCRIPT_DIR/../pipelines/lidar3d-default.yaml}"
DEFAULT_SEQS_TO_RUN="00 02 03 04 05 06 07 08 09 10 18"
SEQS_TO_RUN="${SEQS_TO_RUN:-${DEFAULT_SEQS_TO_RUN}}"
NUM_THREADS=3

if [ ! -f $PIPELINE_YAML ]; then
    echo "Error: Expected local file: '$PIPELINE_YAML'"
    echo "Usage: Invoke this script from your ~/ros_ws directory."
    exit 1
fi

parallel -j${NUM_THREADS} --lb --halt now,fail=1 \
  SEQ={} \
  MOLA_GENERATE_SIMPLEMAP=true \
  MOLA_SIMPLEMAP_OUTPUT=results/kitti360_{}.simplemap \
  MOLA_SIMPLEMAP_GENERATE_LAZY_LOAD=true \
  mola-lidar-odometry-cli \
    -c $PIPELINE_YAML\
    --input-kitti360-seq {} \
    --output-tum-path results/kitti360_{}_mola.tum $@ \
::: $SEQS_TO_RUN

# Eval metrics for each sequence:
for d in $SEQS_TO_RUN; do

done

