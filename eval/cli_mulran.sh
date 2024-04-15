#!/usr/bin/env bash

# Default pipeline YAML file:
PIPELINE_YAML="${PIPELINE_YAML:-src/mola_lidar_odometry/pipelines/lidar3d-default.yaml}"

DEFAULT_SEQUENCES="KAIST01 KAIST02 KAIST03 DCC01 DCC02 DCC03 Riverside01 Riverside02 Riverside03 Sejong01 Sejong02 Sejong03"
SEQUENCES="${SEQUENCES:-${DEFAULT_SEQUENCES}}"

echo "Sequences to run: $SEQUENCES"
NUM_THREADS=3

if [ ! -f $PIPELINE_YAML ]; then
    echo "Error: Expected local file: '$PIPELINE_YAML'"
    echo "Usage: Invoke this script from your ~/ros_ws directory."
    exit 1
fi

parallel -j${NUM_THREADS} --lb \
  SEQ={} \
  MOLA_GENERATE_SIMPLEMAP=true \
  MOLA_SIMPLEMAP_OUTPUT=results/mulran_{}.simplemap \
  MOLA_SIMPLEMAP_GENERATE_LAZY_LOAD=true \
  mola-lidar-odometry-cli \
    -c $PIPELINE_YAML\
    --input-mulran-seq {} \
    --output-tum-path results/mulran_{}_mola.tum \
::: $SEQUENCES

# Eval kitti metrics for each sequence:
# (the Mulran MOLA module generates the "*_gt.txt" files used below)
for d in $SEQUENCES; do
  kitti-metrics-eval -r results/mulran_${d}_mola.tum --gt-tum-path mulran_${d}_mola_gt.tum --no-figures
done

# TODO: Eval ATE/RTE with evo?
for d in $SEQUENCES; do
  evo_ape tum results/mulran_${d}_mola.tum results/mulran_${d}_mola_gt.tum -a
done
