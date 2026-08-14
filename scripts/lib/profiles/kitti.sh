# KITTI odometry benchmark (https://www.cvlibs.net/datasets/kitti/).
#
# The sequence is read by mola_input_kitti_dataset from KITTI_BASE_DIR, so
# there are no topics or bags involved: only a two-digit sequence number.
#
# Note that the published ground truth is expressed in the CAMERA frame, not
# the Velodyne one. Scoring an estimate against it without correcting for that
# is mostly absorbed by APE's alignment and catastrophic in RPE.

mola_lo_profile_usage() {
  echo "Error: A KITTI sequence number is required."
  echo "Usage: $0 <KITTI_SEQ> [additional flags]"
  echo "With <KITTI_SEQ> 00 or 01 or 02..."
  echo ""
  echo "The dataset location is taken from \$KITTI_BASE_DIR."
}

mola_lo_profile_resolve() {
  local seq=$1
  shift
  MOLA_LO_EXTRA_ARGS=("$@")

  KITTI_SEQ="$seq"
  export KITTI_SEQ

  # A car on a road: the very first scans are matched much better with a
  # sensible forward-velocity prior than from a standstill assumption.
  : "${MOLA_INITIAL_VX:=20.0}"
  export MOLA_INITIAL_VX

  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_kitti.yaml
  MOLA_LO_CLI_INPUT=(--input-kitti-seq "$seq")
}
