#!/usr/bin/env bash
# Checks how the dataset-profile library picks the state estimator class from
# MOLA_STATE_ESTIMATOR / MOLA_STATE_ESTIMATOR_YAML, and the smoother defaults
# that follow from it. Needs no dataset: the rawlog profile only routes the
# file name.
#
# Usage: test_dataset_profile_estimator.sh <path to dataset-profile.sh>
set -uo pipefail

LIB=$1
SMOOTHER=mola::state_estimation_smoother::StateEstimationSmoother
SIMPLE=mola::state_estimation_simple::StateEstimationSimple

TMP=$(mktemp -d)
trap 'rm -rf -- "$TMP"' EXIT
printf 'params:\n  sliding_window_length: 6.0\n' >"$TMP/smoother.yaml"
printf 'params:\n  max_time_to_use_velocity_model: 0.75\n' >"$TMP/simple.yaml"

FAILS=0
check() {
  local label=$1 expected=$2 got=$3
  if [ "$got" = "$expected" ]; then
    echo "ok   $label"
  else
    echo "FAIL $label: expected '$expected', got '$got'"
    FAILS=$((FAILS + 1))
  fi
}

# Runs one sequence of loads in a clean shell and prints the final value of
# the variable named by SHOW (default: the class). Each argument is either
# "load" or a VAR=value / unset:VAR assignment.
run() {
  env -i PATH="$PATH" HOME="$HOME" SHOW="${SHOW:-MOLA_STATE_ESTIMATOR}" bash -c '
    set -euo pipefail
    source "$1"
    shift
    MOLA_LO_MODE=cli
    for step in "$@"; do
      case "$step" in
        load) mola_lo_load_profile rawlog /dev/null/dataset.rawlog >/dev/null ;;
        unset:*) unset "${step#unset:}" ;;
        *) export "${step?}" ;;
      esac
    done
    echo "${!SHOW:-unset}"
  ' _ "$LIB" "$@"
}

check "smoother YAML" "$SMOOTHER" "$(run MOLA_STATE_ESTIMATOR_YAML="$TMP/smoother.yaml" load)"
check "simple YAML" "$SIMPLE" "$(run MOLA_STATE_ESTIMATOR_YAML="$TMP/simple.yaml" load)"
check "no YAML" unset "$(run load)"
check "explicit class wins" "$SIMPLE" \
  "$(run MOLA_STATE_ESTIMATOR="$SIMPLE" MOLA_STATE_ESTIMATOR_YAML="$TMP/smoother.yaml" load)"
check "skip flag" unset \
  "$(run MOLA_LO_SKIP_STATE_ESTIMATOR=1 MOLA_STATE_ESTIMATOR_YAML="$TMP/smoother.yaml" load)"
check "re-inferred when the YAML changes" "$SMOOTHER" \
  "$(run MOLA_STATE_ESTIMATOR_YAML="$TMP/simple.yaml" load MOLA_STATE_ESTIMATOR_YAML="$TMP/smoother.yaml" load)"
check "inferred class dropped with the YAML" unset \
  "$(run MOLA_STATE_ESTIMATOR_YAML="$TMP/simple.yaml" load unset:MOLA_STATE_ESTIMATOR_YAML load)"
# A class set to the very value inferred earlier cannot be told apart from it,
# so the override uses another class.
check "explicit class set after an inference survives a YAML change" my::CustomEstimator \
  "$(run MOLA_STATE_ESTIMATOR_YAML="$TMP/simple.yaml" load MOLA_STATE_ESTIMATOR=my::CustomEstimator \
    MOLA_STATE_ESTIMATOR_YAML="$TMP/smoother.yaml" load)"

# LiDAR-grade increment sigma with the smoother only, unless the caller set one:
SHOW=MOLA_REL_POSE_INCR_SIGMA_ANG
check "smoother: LiDAR increment sigma" 1e-5 "$(run MOLA_STATE_ESTIMATOR_YAML="$TMP/smoother.yaml" load)"
check "simple: no increment sigma" unset "$(run MOLA_STATE_ESTIMATOR_YAML="$TMP/simple.yaml" load)"
check "smoother: caller's increment sigma wins" 0.003 \
  "$(run MOLA_REL_POSE_INCR_SIGMA_ANG=0.003 MOLA_STATE_ESTIMATOR_YAML="$TMP/smoother.yaml" load)"

exit "$FAILS"
