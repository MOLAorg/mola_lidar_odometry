#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Default pipeline YAML file:
PIPELINE_YAML="${PIPELINE_YAML:-$SCRIPT_DIR/../pipelines/lidar3d-default.yaml}"
DEFAULT_SEQS_TO_RUN="00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21"
SEQS_TO_RUN="${SEQS_TO_RUN:-${DEFAULT_SEQS_TO_RUN}}"
NUM_THREADS="${NUM_THREADS:-3}"

if [ ! -f $PIPELINE_YAML ]; then
    echo "Error: Expected local file: '$PIPELINE_YAML'"
    echo "Usage: Invoke this script from your ~/ros_ws directory."
    exit 1
fi

# This var: MOLA_SIMPLEMAP_ALSO_NON_KEYFRAMES
# is defined to save all .simplemap with *all* frames as keyframes,
# for usage with the loop-closure postprocessing tool and afterwards
# still be able to recover the full trajectory for metric error evaluation.


parallel -j${NUM_THREADS} --lb --halt now,fail=1 \
  SEQ={} \
  MOLA_INITIAL_VX=18.0 \
  MOLA_SIMPLEMAP_ALSO_NON_KEYFRAMES=true \
  MOLA_SIMPLEMAP_MIN_XYZ=10.0 \
  MOLA_SIMPLEMAP_MIN_ROT=20.0 \
  MOLA_GENERATE_SIMPLEMAP=true \
  MOLA_SIMPLEMAP_OUTPUT=results/kitti_{}.simplemap \
  mola-lidar-odometry-cli \
    -c $PIPELINE_YAML\
    --input-kitti-seq {} \
    --output-tum-path results/kitti_{}_mola.tum \
    $@ \
::: $SEQS_TO_RUN

#    --kitti-correction-angle-deg 0.21 \

# Eval kitti metrics for each sequence alone:
for d in $SEQS_TO_RUN; do
  if [ -f results/kitti_${d}_mola_gt.tum ]; then
    kitti-metrics-eval -r results/kitti_${d}_mola.tum -s ${d} --no-figures
  fi
done

# Eval overall kitti metrics:
if [ "$DEFAULT_SEQS_TO_RUN" = "$SEQS_TO_RUN" ]; then
  kitti-metrics-eval -r results/kitti_%02i_mola.tum -s 00 -s 01 -s 02 -s 03 -s 04 -s 05 -s 06 -s 07 -s 08 -s 09 -s 10 --no-figures
fi
