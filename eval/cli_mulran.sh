#!/usr/bin/env bash

# Default pipeline YAML file:
PIPELINE_YAML="${PIPELINE_YAML:-src/mola_lidar_odometry/params/lidar3d-default.yaml}"

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
  SEQ={} mola-lidar-odometry-cli \
    -c $PIPELINE_YAML\
    --input-mulran-seq {} \
    --output-tum-path results/estim_{}.txt \
::: $SEQUENCES

# Eval kitti metrics for each sequence:
# (the Mulran MOLA module generates the "*_gt.txt" files used below)
for d in $SEQUENCES; do
  kitti-metrics-eval -r results/estim_${d}.txt --gt-tum-path results/estim_${d}_gt.txt --no-figures
done

# TODO: Eval ATE/RTE with evo?
for d in $SEQUENCES; do
  evo_ape tum results/estim_${d}.txt results/estim_${d}_gt.txt -a
done
