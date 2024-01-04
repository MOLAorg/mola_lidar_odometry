#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Default pipeline YAML file:
PIPELINE_YAML="${PIPELINE_YAML:-$SCRIPT_DIR/../params/lidar-inertial-pipeline-simple.yaml}"
PLUGIN_MAPS=install/mola_metric_maps/lib/libmola_metric_maps.so
SEQS_TO_RUN="00 01 02 03 04 05 06 07 08 09 10"

if [ ! -f $PIPELINE_YAML ]; then
    echo "Error: Expected local file: '$PIPELINE_YAML'"
    echo "Usage: Invoke this script from your ~/ros_ws directory."
    exit 1
fi
if [ ! -f $PLUGIN_MAPS ]; then
    echo "Error: Expected local file: '$PLUGIN_MAPS'"
    echo "Usage: Invoke this script from your ~/ros_ws directory."
    exit 1
fi

parallel -j2 --lb --halt now,fail=1 \
  SEQ={} mola-lidar-odometry-cli \
    -c $PIPELINE_YAML\
    -l $PLUGIN_MAPS\
    --input-kitti-seq {} \
    --output-tum-path results/estim_{}.txt \
::: $SEQS_TO_RUN

#    --kitti-correction-angle-deg 0.21 \

# Eval kitti metrics for each sequence alone:
for d in $SEQS_TO_RUN; do
  kitti-metrics-eval -r results/estim_${d}.txt -s ${d} --no-figures
done

# Eval overall kitti metrics:
kitti-metrics-eval -r results/estim_%02i.txt -s 00 -s 01 -s 02 -s 03 -s 04 -s 05 -s 06 -s 07 -s 08 -s 09 -s 10 --no-figures
