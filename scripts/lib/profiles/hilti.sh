# Hilti SLAM Challenge 2022 / Hilti-Oxford (a hand-held survey pole walked
# through construction sites and Oxford's Sheldonian Theatre).
#
# One monolithic ROS 1 bag per sequence: a Hesai PandarXT-32 lidar, a
# Sevensense Alphasense IMU, and five cameras. There is no /tf or /tf_static
# in the bags at all, so every sensor pose has to be fixed here.
#
# base_link is the IMU, and that is not an arbitrary choice: the dataset's
# own calibration/calibration_files/lidar_calibration.yaml declares the IMU
# as base_link at identity, and every ground-truth file is expressed in the
# IMU frame. Reporting in the IMU frame therefore lands the estimate in the
# reference's own frame, with no offset to compose afterwards. The lidar sits
# at T_imu_pandar below, taken from that same file (translation
# [-0.001, -0.00855, 0.055], quaternion x,y,z,w [0.7071068, -0.7071068, 0, 0],
# i.e. a 5.6 cm lever arm).
#
# Ground truth is worth reading before picking a sequence: only
# exp14_basement_2, exp16_attic_to_upper_gallery_2 and
# exp18_corridor_lower_gallery_2 ship a dense 6-DoF trajectory (~10 Hz). The
# other thirteen sequences ship surveyed control points -- a handful of
# positions with a placeholder identity quaternion, not poses.

mola_lo_profile_usage() {
  echo "Error: A Hilti 2022 ROS 1 '.bag' file is required."
  echo "Usage: $0 /path/to/expNN_name.bag [additional flags]"
  echo ""
  echo "Optional environment variables:"
  echo "  HILTI_BASE_FRAME   'imu' (default) or 'lidar': which frame the"
  echo "                     estimated trajectory is reported in. The ground"
  echo "                     truth is in the IMU frame, so 'imu' is what"
  echo "                     scores directly against it."
}

mola_lo_profile_resolve() {
  local file=$1
  shift
  MOLA_LO_EXTRA_ARGS=("$@")

  if [ -z "$file" ]; then
    mola_lo_profile_usage
    return 1
  fi

  : "${MOLA_LIDAR_TOPIC:=/hesai/pandar}"
  : "${MOLA_IMU_TOPIC:=/alphasense/imu}"
  export MOLA_LIDAR_TOPIC MOLA_IMU_TOPIC

  # The five cameras are a preview only; decoding them in a batch run costs
  # time and buys nothing. Same convention as conslam.sh.
  if [ "$MOLA_LO_MODE" = "gui" ]; then
    : "${MOLA_CAMERA_TOPIC:=/alphasense/cam0/image_raw}"
    : "${MOLA_USE_FIXED_CAMERA_POSE:=1}"
    export MOLA_CAMERA_TOPIC MOLA_USE_FIXED_CAMERA_POSE
  fi

  : "${MOLA_USE_FIXED_IMU_POSE:=1}"
  : "${MOLA_USE_FIXED_LIDAR_POSE:=1}"
  export MOLA_USE_FIXED_IMU_POSE MOLA_USE_FIXED_LIDAR_POSE

  : "${HILTI_BASE_FRAME:=imu}"
  case "$HILTI_BASE_FRAME" in
    imu)
      # T_imu_pandar, from the dataset's lidar_calibration.yaml.
      : "${MOLA_TF_BASE_LINK:=imu}"
      : "${LIDAR_POSE_X:=-0.001}"
      : "${LIDAR_POSE_Y:=-0.00855}"
      : "${LIDAR_POSE_Z:=0.055}"
      : "${LIDAR_POSE_YAW:=-90}"
      : "${LIDAR_POSE_PITCH:=0}"
      : "${LIDAR_POSE_ROLL:=180}"
      : "${IMU_POSE_X:=0}" ; : "${IMU_POSE_Y:=0}" ; : "${IMU_POSE_Z:=0}"
      : "${IMU_POSE_YAW:=0}" ; : "${IMU_POSE_PITCH:=0}" ; : "${IMU_POSE_ROLL:=0}"
      ;;
    lidar)
      # T_pandar_imu, the inverse of the above.
      : "${MOLA_TF_BASE_LINK:=lidar}"
      : "${LIDAR_POSE_X:=0}" ; : "${LIDAR_POSE_Y:=0}" ; : "${LIDAR_POSE_Z:=0}"
      : "${LIDAR_POSE_YAW:=0}" ; : "${LIDAR_POSE_PITCH:=0}" ; : "${LIDAR_POSE_ROLL:=0}"
      : "${IMU_POSE_X:=-0.00855}"
      : "${IMU_POSE_Y:=-0.001}"
      : "${IMU_POSE_Z:=0.055}"
      # This rotation is its own inverse (R^T == R), hence the same angles
      # as the imu branch above; only the translation changes.
      : "${IMU_POSE_YAW:=-90}"
      : "${IMU_POSE_PITCH:=0}"
      : "${IMU_POSE_ROLL:=180}"
      ;;
    *)
      echo "Error: HILTI_BASE_FRAME must be 'imu' or 'lidar', got '$HILTI_BASE_FRAME'." >&2
      return 1
      ;;
  esac
  export MOLA_TF_BASE_LINK
  export LIDAR_POSE_X LIDAR_POSE_Y LIDAR_POSE_Z
  export LIDAR_POSE_YAW LIDAR_POSE_PITCH LIDAR_POSE_ROLL
  export IMU_POSE_X IMU_POSE_Y IMU_POSE_Z
  export IMU_POSE_YAW IMU_POSE_PITCH IMU_POSE_ROLL

  # Hand-held rig, like conslam: larger and faster rotations than a vehicle,
  # so lean on the IMU both for deskewing and for the initial pitch/roll. The
  # clouds do carry genuine per-point timestamps (an f64 'timestamp' field
  # plus 'ring'), so deskewing has real data to work with here.
  : "${MOLA_LO_INITIAL_LOCALIZATION_METHOD:=InitLocalization::PitchAndRollFromIMU}"
  : "${MOLA_DESKEW_METHOD:=MotionCompensationMethod::IMU}"
  export MOLA_LO_INITIAL_LOCALIZATION_METHOD MOLA_DESKEW_METHOD

  # No GNSS in this dataset: pin the first pose to the map origin, or the
  # smoother's GTSAM graph is left with a rank-deficient null-space
  # (IndeterminantLinearSystemException). Same as conslam and botanicgarden.
  : "${MOLA_LINK_FIRST_POSE_SIGMA:=1e-6}"
  export MOLA_LINK_FIRST_POSE_SIGMA

  mola_lo_use_smoother || return 1

  mola_lo_bag_slots "$file" || return 1
  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_rosbag1.yaml
  MOLA_LO_CLI_INPUT=(--input-rosbag1 "$MOLA_LO_BAGS_JOINED")
}
