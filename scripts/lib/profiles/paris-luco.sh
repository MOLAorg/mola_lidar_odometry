# Paris-LuCo (the ParisLuco128 sequence used by CT-ICP and others).
#
# Read by mola_input_paris_luco_dataset from PARIS_LUCO_BASE_DIR: a single
# sequence, so there is nothing to select.

MOLA_LO_PROFILE_NO_ARGS=1

mola_lo_profile_usage() {
  echo "Usage: $0 [additional flags]"
  echo ""
  echo "Paris-LuCo publishes a single sequence, so no sequence argument is"
  echo "needed. The dataset location is taken from \$PARIS_LUCO_BASE_DIR."
}

mola_lo_profile_resolve() {
  MOLA_LO_EXTRA_ARGS=("$@")

  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_paris_luco.yaml
  MOLA_LO_CLI_INPUT=(--input-paris-luco)
}
