# ConSLAM (a hand-held scanner walked through construction sites).
#
# Accepts either a ROS 1 bag (.bag) or a ROS 2 bag (.mcap), autodetected from
# the file extension.
#
# Topics in these bags have NO leading slash ('imu/data', not '/imu/data'),
# and there is no /tf or /tf_static at all, so every sensor pose must be
# fixed. See mola_mapper's lidar_odometry_mapper_from_conslam.yaml for the
# validated reference configuration this mirrors.
#
# Which frame is base_link is a deliberate choice, not a constant, hence
# CONSLAM_BASE_FRAME below: the IMU is the dataset's own reference frame and
# the natural default, but anything scoring against a ground truth sampled at
# LiDAR scan times and expressed in the LiDAR frame has to report in that
# frame instead, or the comparison silently measures the mounting offset.
# The two frames differ by a pure 180 deg yaw (both share the "up" axis, per
# the paper's Sec. 3.3 / Fig. 1(a) axis conventions). The translation between
# the two mounting points ships only in the dataset's separate
# data_calib.zip, so it is left at zero: a few centimetres on a hand-held
# frame, not a real calibration.

mola_lo_profile_usage() {
  echo "Error: A ConSLAM ROS 1 (.bag) or ROS 2 (.mcap) bag file is required."
  echo "Usage: $0 /path/to/sequenceN.bag [additional flags]"
  echo "       $0 /path/to/sequenceN.mcap [additional flags]"
  echo ""
  echo "Optional environment variables:"
  echo "  CONSLAM_BASE_FRAME   'imu' (default) or 'lidar': which frame the"
  echo "                       estimated trajectory is reported in."
}

mola_lo_profile_resolve() {
  local file=$1
  shift
  MOLA_LO_EXTRA_ARGS=("$@")

  : "${MOLA_IMU_TOPIC:=imu/data}"
  : "${MOLA_LIDAR_TOPIC:=pp_points/synced2rgb}"
  export MOLA_IMU_TOPIC MOLA_LIDAR_TOPIC

  : "${MOLA_USE_FIXED_IMU_POSE:=1}"
  : "${MOLA_USE_FIXED_LIDAR_POSE:=1}"
  export MOLA_USE_FIXED_IMU_POSE MOLA_USE_FIXED_LIDAR_POSE

  # The camera is a preview only -- nothing in the odometry consumes it -- so
  # a batch run would install a handler and decode images for nothing.
  if [ "$MOLA_LO_MODE" = "gui" ]; then
    : "${MOLA_CAMERA_TOPIC:=pp_rgb/synced2points}"
    : "${MOLA_USE_FIXED_CAMERA_POSE:=1}"
    export MOLA_CAMERA_TOPIC MOLA_USE_FIXED_CAMERA_POSE
  fi

  : "${CONSLAM_BASE_FRAME:=imu}"
  case "$CONSLAM_BASE_FRAME" in
    imu)
      : "${MOLA_TF_BASE_LINK:=imu}"
      : "${LIDAR_POSE_YAW:=180}"
      : "${IMU_POSE_YAW:=0}"
      ;;
    lidar)
      : "${MOLA_TF_BASE_LINK:=lidar}"
      : "${LIDAR_POSE_YAW:=0}"
      : "${IMU_POSE_YAW:=180}"
      ;;
    *)
      echo "Error: CONSLAM_BASE_FRAME must be 'imu' or 'lidar', got '$CONSLAM_BASE_FRAME'." >&2
      return 1
      ;;
  esac
  : "${LIDAR_POSE_X:=0}" ; : "${LIDAR_POSE_Y:=0}" ; : "${LIDAR_POSE_Z:=0}"
  : "${LIDAR_POSE_PITCH:=0}" ; : "${LIDAR_POSE_ROLL:=0}"
  : "${IMU_POSE_X:=0}" ; : "${IMU_POSE_Y:=0}" ; : "${IMU_POSE_Z:=0}"
  : "${IMU_POSE_PITCH:=0}" ; : "${IMU_POSE_ROLL:=0}"
  export MOLA_TF_BASE_LINK
  export LIDAR_POSE_X LIDAR_POSE_Y LIDAR_POSE_Z
  export LIDAR_POSE_YAW LIDAR_POSE_PITCH LIDAR_POSE_ROLL
  export IMU_POSE_X IMU_POSE_Y IMU_POSE_Z
  export IMU_POSE_YAW IMU_POSE_PITCH IMU_POSE_ROLL

  # Hand-held scanner (not a vehicle): larger/faster rotations than a car, so
  # lean on the IMU for deskewing and for the initial pitch/roll estimate:
  : "${MOLA_LO_INITIAL_LOCALIZATION_METHOD:=InitLocalization::PitchAndRollFromIMU}"
  : "${MOLA_DESKEW_METHOD:=MotionCompensationMethod::IMU}"
  export MOLA_LO_INITIAL_LOCALIZATION_METHOD MOLA_DESKEW_METHOD

  # No GNSS in this dataset: pin the first pose to the map origin, or the
  # smoother's GTSAM graph is left with a rank-deficient null-space
  # (IndeterminantLinearSystemException):
  : "${MOLA_LINK_FIRST_POSE_SIGMA:=1e-6}"
  export MOLA_LINK_FIRST_POSE_SIGMA

  mola_lo_use_smoother || return 1

  if [[ "$file" == *.mcap ]]; then
    MOLA_INPUT_ROSBAG2="$file"
    export MOLA_INPUT_ROSBAG2
    MOLA_LO_LAUNCH_FILE=lidar_odometry_from_rosbag2.yaml
    MOLA_LO_CLI_INPUT=(--input-rosbag2 "$file")
  else
    mola_lo_bag_slots "$file" || return 1
    MOLA_LO_LAUNCH_FILE=lidar_odometry_from_rosbag1.yaml
    MOLA_LO_CLI_INPUT=(--input-rosbag1 "$MOLA_LO_BAGS_JOINED")
  fi
}
