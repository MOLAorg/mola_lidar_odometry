#!/usr/bin/env bash
# Batch KITTI evaluation with the GICP pipeline: run every sequence through
# MOLA-LO, then score the results with kitti-metrics-eval.
#
# Launching is NOT this script's job -- that is mola-lo-cli-kitti, which
# shares its dataset knowledge with the GUI wrapper and with any other
# consumer (see scripts/lib/profiles/kitti.sh). This file only decides which
# sequences to sweep, how many at a time, and what to measure.

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Passed through to mola-lo-cli-kitti, which honors them:
PIPELINE_YAML="${MOLA_PIPELINE_YAML:-$SCRIPT_DIR/../pipelines/lidar3d-gicp.yaml}"
export MOLA_ODOMETRY_PIPELINE_YAML="$PIPELINE_YAML"
export MOLA_STATE_ESTIMATOR_YAML="${MOLA_SE_YAML:-$SCRIPT_DIR/../state-estimator-params/state-estimation-simple.yaml}"

DEFAULT_SEQS_TO_RUN="00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21"
SEQS_TO_RUN="${SEQS_TO_RUN:-${DEFAULT_SEQS_TO_RUN}}"
NUM_THREADS="${MOLA_NUM_THREADS:-1}"

if [ ! -f "$PIPELINE_YAML" ]; then
    echo "Error: Expected local file: '$PIPELINE_YAML'"
    echo "Usage: Invoke this script from your ~/ros_ws directory."
    exit 1
fi

# This var: MOLA_SIMPLEMAP_ALSO_NON_KEYFRAMES
# is defined to save all .simplemap with *all* frames as keyframes,
# for usage with the loop-closure postprocessing tool and afterwards
# still be able to recover the full trajectory for metric error evaluation.

# This sweep has always run without an initial-velocity prior, unlike
# cli_kitti.sh; MOLA_INITIAL_VX is pinned to zero rather than left to the
# profile's default so that stays true.
parallel -j${NUM_THREADS} --lb --halt now,fail=1 \
  SEQ={} \
  MOLA_INITIAL_VX=0.0 \
  MOLA_SIMPLEMAP_ALSO_NON_KEYFRAMES=true \
  MOLA_SIMPLEMAP_MIN_XYZ=10.0 \
  MOLA_SIMPLEMAP_MIN_ROT=20.0 \
  MOLA_GENERATE_SIMPLEMAP=true \
  MOLA_SIMPLEMAP_OUTPUT=results/kitti_{}.simplemap \
  mola-lo-cli-kitti {} \
    --output-tum-path results/kitti_{}_mola_gicp.tum \
    $@ \
::: $SEQS_TO_RUN

# Eval kitti metrics for each sequence alone:
for d in $SEQS_TO_RUN; do
  if [ -f results/kitti_${d}_mola_gt.tum ]; then
    kitti-metrics-eval -r results/kitti_${d}_mola_gicp.tum -s ${d} --no-figures
  fi
done

# Eval overall kitti metrics:
if [ "$DEFAULT_SEQS_TO_RUN" = "$SEQS_TO_RUN" ]; then
  kitti-metrics-eval -r results/kitti_%02i_mola_gicp.tum -s 00 -s 01 -s 02 -s 03 -s 04 -s 05 -s 06 -s 07 -s 08 -s 09 -s 10 --no-figures
fi
