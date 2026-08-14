#!/usr/bin/env bash
# Batch MulRan evaluation: run every sequence through MOLA-LO, then score both
# KITTI relative metrics and evo's APE.
#
# Launching is NOT this script's job -- that is mola-lo-cli-mulran, which
# shares its dataset knowledge with the GUI wrapper (see
# scripts/lib/profiles/mulran.sh). This file only decides which sequences to
# sweep, how many at a time, and what to measure.
#
# Alternative NDT-3D pipeline, launch with:
# PIPELINE_PREFIX=_ndt PIPELINE_YAML=src/mola_lidar_odometry/pipelines/lidar3d-ndt.yaml src/mola_lidar_odometry/eval/cli_mulran.sh

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

PIPELINE_YAML="${PIPELINE_YAML:-$SCRIPT_DIR/../pipelines/lidar3d-default.yaml}"
export MOLA_ODOMETRY_PIPELINE_YAML="$PIPELINE_YAML"
export MOLA_STATE_ESTIMATOR_YAML="${MOLA_SE_YAML:-$SCRIPT_DIR/../state-estimator-params/state-estimation-simple.yaml}"

# What sequences to test:
DEFAULT_SEQUENCES="KAIST01 KAIST02 KAIST03 DCC01 DCC02 DCC03 Riverside01 Riverside02 Riverside03 Sejong01 Sejong02 Sejong03"
SEQUENCES="${SEQUENCES:-${DEFAULT_SEQUENCES}}"

echo "Sequences to run: $SEQUENCES"
NUM_THREADS="${NUM_THREADS:-3}"

if [ ! -f "$PIPELINE_YAML" ]; then
    echo "Error: Expected local file: '$PIPELINE_YAML'"
    echo "Usage: Invoke this script from your ~/ros_ws directory."
    exit 1
fi

# Add this one to also create the maps (It requires several GBs!)
#GENERATE_SIMPLEMAPS=true
GENERATE_SIMPLEMAPS=false

parallel -j${NUM_THREADS} --lb --halt now,fail=1 \
  SEQ={} \
  MOLA_GENERATE_SIMPLEMAP=${GENERATE_SIMPLEMAPS} \
  MOLA_SIMPLEMAP_ALSO_NON_KEYFRAMES=true \
  MOLA_SIMPLEMAP_MIN_XYZ=10.0 \
  MOLA_SIMPLEMAP_MIN_ROT=20.0 \
  MOLA_SIMPLEMAP_OUTPUT=results/mulran_{}.simplemap \
  MOLA_SIMPLEMAP_GENERATE_LAZY_LOAD=true \
  mola-lo-cli-mulran {} \
    --output-tum-path results/mulran_{}_mola${PIPELINE_PREFIX}.tum \
::: $SEQUENCES

# Eval kitti metrics for each sequence:
# (the Mulran MOLA module generates the "*_gt.txt" files used below)
out=results/mulran_metrics_mola${PIPELINE_PREFIX}.txt
rm $out || true
echo "KITTI_METRIC:" >> $out
for d in $SEQUENCES; do
  kitti-metrics-eval -r results/mulran_${d}_mola${PIPELINE_PREFIX}.tum --gt-tum-path results/mulran_${d}_mola${PIPELINE_PREFIX}_gt.tum --no-figures >> $out
done

# Eval APE with evo:
echo "EVO APE:" >> $out
for d in $SEQUENCES; do
  echo "$d" >> $out
  evo_ape tum results/mulran_${d}_mola${PIPELINE_PREFIX}.tum results/mulran_${d}_mola${PIPELINE_PREFIX}_gt.tum -a  >> $out
done
