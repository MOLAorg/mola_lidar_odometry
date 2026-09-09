#!/usr/bin/env bash
# Batch KITTI evaluation: run every sequence through MOLA-LO, then score the
# results with kitti-metrics-eval.
#
# Launching is NOT this script's job -- that is mola-lo-cli-kitti, which
# shares its dataset knowledge with the GUI wrapper and with any other
# consumer (see scripts/lib/profiles/kitti.sh). This file only decides which
# sequences to sweep, how many at a time, and what to measure.
#
# Alternative NDT-3D pipeline, launch with:
# PIPELINE_PREFIX=_ndt MOLA_PIPELINE_YAML=src/mola_lidar_odometry/pipelines/lidar3d-ndt.yaml SEQS_TO_RUN="00 01" src/mola_lidar_odometry/eval/cli_kitti.sh

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Passed through to mola-lo-cli-kitti, which honors them:
PIPELINE_YAML="${MOLA_PIPELINE_YAML:-$SCRIPT_DIR/../pipelines/lidar3d-default.yaml}"
export MOLA_ODOMETRY_PIPELINE_YAML="$PIPELINE_YAML"
export MOLA_STATE_ESTIMATOR_YAML="${MOLA_SE_YAML:-$SCRIPT_DIR/../state-estimator-params/state-estimation-simple.yaml}"

DEFAULT_SEQS_TO_RUN="00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21"
SEQS_TO_RUN="${SEQS_TO_RUN:-${DEFAULT_SEQS_TO_RUN}}"
NUM_THREADS="${NUM_THREADS:-3}"

if [ ! -f "$PIPELINE_YAML" ]; then
    echo "Error: Expected local file: '$PIPELINE_YAML'"
    echo "Usage: Invoke this script from your ~/ros_ws directory."
    exit 1
fi

# This var: MOLA_SIMPLEMAP_ALSO_NON_KEYFRAMES
# is defined to save all .simplemap with *all* frames as keyframes,
# for usage with the loop-closure postprocessing tool and afterwards
# still be able to recover the full trajectory for metric error evaluation.

# MOLA_INITIAL_VX is set here, not left to the profile's default: the
# published numbers from this sweep were produced with 18.0, and the profile
# (i.e. what an interactive run gets) uses 20.0. Kept explicit so the
# benchmark stays comparable to its own history rather than silently shifting.
#
# The value is a genuine trade-off, not a tuning leftover: a large prior is
# needed on seq 12, which starts already at speed, and introduces artifacts
# on the rest of the corpus. A sweep that reports one aggregate number over
# all sequences is therefore reporting a compromise; keep that in mind before
# reading a small delta here as a pipeline improvement.
#
# CAVEAT ADDED 2026-08-21, and it is about the pipeline this script runs.
# "A large prior is needed on seq 12" was established on `lidar3d-icp.yaml`.
# This script exports MOLA_ODOMETRY_PIPELINE_YAML itself, defaulting to
# `lidar3d-default.yaml` (GICP) -- overriding the ICP default that
# scripts/lib/profiles/kitti.sh would otherwise apply -- and on GICP that claim
# does not hold. Measured on the sparse 2D reference for seqs 11-15, ratio of
# estimated to reference path length and RMSE after planar alignment:
#
#            seq 12 unseeded          VX=18 vs unseeded, seqs 11-15
#   ICP      0.268 / 403 m            (the seed is what rescues it)
#   GICP     0.995 / 4.82 m           worse on 4 of 5; its only gain is seq 12
#                                     itself, 4.16 vs 4.82, inside the ~3 m
#                                     noise floor of that reference
#
# So the seed is coupled to the PIPELINE, not to the dataset, and this script
# currently pairs it with the pipeline that does not need it. The value is left
# at 18.0 deliberately: the published numbers from this sweep were produced with
# it, and changing it silently would break comparability with its own history --
# which is the same reason the line above exists. Anyone re-baselining should
# drop it, or switch this script to the ICP pipeline, rather than keep both.
#
# Full measurements: ~/plans/lio/detail/216_smoother_state_estimator_parity.md
# section 11.
parallel -j${NUM_THREADS} --lb --halt now,fail=1 \
  SEQ={} \
  MOLA_INITIAL_VX=18.0 \
  MOLA_SIMPLEMAP_ALSO_NON_KEYFRAMES=true \
  MOLA_SIMPLEMAP_MIN_XYZ=10.0 \
  MOLA_SIMPLEMAP_MIN_ROT=20.0 \
  MOLA_GENERATE_SIMPLEMAP=true \
  MOLA_SIMPLEMAP_OUTPUT=results/kitti_{}.simplemap \
  mola-lo-cli-kitti {} \
    --output-tum-path results/kitti_{}_mola${PIPELINE_PREFIX}.tum \
    $@ \
::: $SEQS_TO_RUN

# Eval kitti metrics for each sequence alone:
for d in $SEQS_TO_RUN; do
  if [ -f results/kitti_${d}_mola${PIPELINE_PREFIX}_gt.tum ]; then
    kitti-metrics-eval -r results/kitti_${d}_mola${PIPELINE_PREFIX}.tum -s ${d} --no-figures
  fi
done

# Eval overall kitti metrics:
if [ "$DEFAULT_SEQS_TO_RUN" = "$SEQS_TO_RUN" ]; then
  kitti-metrics-eval -r results/kitti_%02i_mola${PIPELINE_PREFIX}.tum -s 00 -s 01 -s 02 -s 03 -s 04 -s 05 -s 06 -s 07 -s 08 -s 09 -s 10 --no-figures
fi
