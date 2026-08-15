# MulRan (https://sites.google.com/view/mulran-pr/dataset): a car carrying an
# Ouster OS1-64 around Daejeon and Sejong.
#
# Read by mola_input_mulran_dataset from MULRAN_BASE_DIR: a sequence name, no
# topics or bags. That module also writes the "<seq>_gt.tum" files the batch
# evaluation in eval/ scores against.

mola_lo_profile_usage() {
  echo "Error: A MulRan sequence name is required."
  echo "Usage: $0 <MULRAN_SEQ> [additional flags]"
  echo "With <MULRAN_SEQ> KAIST01, DCC01, Riverside01, Sejong01, ..."
  echo ""
  echo "The dataset location is taken from \$MULRAN_BASE_DIR."
}

mola_lo_profile_resolve() {
  local seq=$1
  shift
  MOLA_LO_EXTRA_ARGS=("$@")

  MULRAN_SEQ="$seq"
  export MULRAN_SEQ

  # The OS1-64 spins while the car drives at road speed, and this dataset's
  # IMU is good enough to deskew from:
  : "${MOLA_DESKEW_METHOD:=MotionCompensationMethod::IMU}"
  export MOLA_DESKEW_METHOD

  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_mulran.yaml
  MOLA_LO_CLI_INPUT=(--input-mulran-seq "$seq")
}
