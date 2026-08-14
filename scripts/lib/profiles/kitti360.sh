# KITTI-360 (https://www.cvlibs.net/datasets/kitti-360/).
#
# Read by mola_input_kitti360_dataset from KITTI360_BASE_DIR: a sequence name,
# no topics or bags.

mola_lo_profile_usage() {
  echo "Error: A KITTI-360 sequence name is required."
  echo "Usage: $0 <KITTI360_SEQ> [additional flags]"
  echo "With <KITTI360_SEQ> 00, 03, 04, ..., or test_0, test_1, ..."
  echo ""
  echo "The dataset location is taken from \$KITTI360_BASE_DIR."
}

mola_lo_profile_resolve() {
  local seq=$1
  shift
  MOLA_LO_EXTRA_ARGS=("$@")

  KITTI_SEQ="$seq"
  export KITTI_SEQ

  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_kitti360.yaml
  MOLA_LO_CLI_INPUT=(--input-kitti360-seq "$seq")
}
