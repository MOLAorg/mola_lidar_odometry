#!/usr/bin/env bash

# Default pipeline YAML file:
PIPELINE_YAML="${PIPELINE_YAML:-src/mola_lidar_odometry/params/lidar-inertial-pipeline-simple.yaml}"
PLUGIN_MAPS=install/mola_metric_maps/lib/libmola_metric_maps.so

#SEQUENCES=DCC01
SEQUENCES="KAIST01 KAIST02 KAIST03"

if [ ! -f $PIPELINE_YAML ]; then
    echo "Error: Expected local file: '$PIPELINE_YAML'"
    echo "Usage: Invoke this script from your ~/ros_ws directory."
    exit 1
fi
if [ ! -f $PLUGIN_MAPS ]; then
    echo "Error: Expected local file: 'src/mola_lidar_odometry/params/config-lidar-inertial-odometry.yaml'"
    echo "Usage: Invoke this script from your ~/ros_ws directory."
    exit 1
fi

parallel -j2 --lb \
  SEQ={} mola-lidar-odometry-cli \
    -c $PIPELINE_YAML\
    -l $PLUGIN_MAPS\
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
  evo_ape tum results/estim_DCC01.txt results/estim_DCC01_gt.txt -a
done