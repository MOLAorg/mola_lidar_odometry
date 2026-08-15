#!/usr/bin/env bash
# Batch KITTI-360 sweep.
#
# Launching is NOT this script's job -- that is mola-lo-cli-kitti360, which
# shares its dataset knowledge with the GUI wrapper (see
# scripts/lib/profiles/kitti360.sh). This file only decides which sequences
# to sweep and how many at a time.

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

PIPELINE_YAML="${PIPELINE_YAML:-$SCRIPT_DIR/../pipelines/lidar3d-default.yaml}"
export MOLA_ODOMETRY_PIPELINE_YAML="$PIPELINE_YAML"
export MOLA_STATE_ESTIMATOR_YAML="${MOLA_SE_YAML:-$SCRIPT_DIR/../state-estimator-params/state-estimation-simple.yaml}"

DEFAULT_SEQS_TO_RUN="test_0 test_1 test_2 test_3 00 03 04 05 06 07 09 10"
# Removed 02: See: https://github.com/autonomousvision/kitti360Scripts/issues/92
# Removed 08: no timestamps.txt file?
# Removed 18: no timestamps.txt file?

SEQS_TO_RUN="${SEQS_TO_RUN:-${DEFAULT_SEQS_TO_RUN}}"
NUM_THREADS="${NUM_THREADS:-3}"

if [ ! -f "$PIPELINE_YAML" ]; then
    echo "Error: Expected local file: '$PIPELINE_YAML'"
    echo "Usage: Invoke this script from your ~/ros_ws directory."
    exit 1
fi

parallel -j${NUM_THREADS} --lb --halt now,fail=1 \
  SEQ={} \
  MOLA_SIMPLEMAP_ALSO_NON_KEYFRAMES=true \
  MOLA_SIMPLEMAP_MIN_XYZ=10.0 \
  MOLA_SIMPLEMAP_MIN_ROT=20.0 \
  MOLA_GENERATE_SIMPLEMAP=true \
  MOLA_SIMPLEMAP_OUTPUT=results/kitti360_{}.simplemap \
  MOLA_SIMPLEMAP_GENERATE_LAZY_LOAD=true \
  mola-lo-cli-kitti360 {} \
    --output-tum-path results/kitti360_{}_mola.tum $@ \
::: $SEQS_TO_RUN

# No metrics step here yet: KITTI-360 ships no ready-to-score GT in the form
# kitti-metrics-eval takes.
