# TIERS multi-modal LiDAR dataset
# (https://github.com/TIERS/tiers-lidars-dataset): a trolley-mounted rig
# carrying FIVE lidars recording simultaneously into one ROS 1 bag, which is
# the whole point of the dataset -- it exists to compare them.
#
# There is therefore no single "the" lidar here, and picking one silently
# would be presenting one sensor's result as the dataset's. TIERS_SENSOR
# selects which one, and each choice has its own mola-lo-{gui,cli}-tiers-*
# wrapper so the sensor under test is visible in the command that ran it:
#
#   ouster-os0     /os_cloud_node/points   OS0-128, 128 beams, 90 deg FOV
#   ouster-os1     /os_cloud_nodee/points  OS1-64, 64 beams, 45 deg FOV
#                                          (the doubled "e" is upstream's
#                                          own topic name, not a typo here)
#   velodyne       /velodyne_points        VLP-16, no IMU of its own
#   livox-horizon  /livox/lidar            Livox Horizon, non-repetitive
#   livox-avia     /avia/livox/lidar       Livox AVIA, non-repetitive
#
# EXTRINSICS ARE NOT KNOWN HERE. These bags carry no /tf or /tf_static at all,
# and the dataset publishes no calibration file alongside them, so each lidar
# is treated as CO-LOCATED with its IMU (identity offset). That is a
# placeholder, not a calibration: it is centimetre-scale on this rig, invisible
# in APE after alignment, and real in RPE. Get the true extrinsics before
# treating any relative metric from this dataset as a reference number.
#
# Ground truth is a VRPN mocap topic embedded in the same bag, reporting a
# marker cluster on the trolley ~0.73 m from the lidars -- correct for that
# separately when scoring, it does not belong to the launch configuration.

mola_lo_profile_usage() {
  echo "Error: A TIERS ROS 1 bag file is required."
  echo "Usage: $0 /path/to/<sequence>.bag [additional flags]"
  echo ""
  echo "Sensor selected by this wrapper: ${TIERS_SENSOR:-(none)}"
}

mola_lo_profile_resolve() {
  local file=$1
  shift
  MOLA_LO_EXTRA_ARGS=("$@")

  : "${TIERS_SENSOR:=ouster-os0}"

  # The Livox units publish livox_ros_driver/CustomMsg, which Rosbag1Dataset
  # converts to a point cloud like any other; they are non-repetitive, so a
  # single scan does not cover the full FOV and the pipeline wants to
  # accumulate more per location.
  local non_repetitive=0
  case "$TIERS_SENSOR" in
    ouster-os0)
      : "${MOLA_LIDAR_TOPIC:=/os_cloud_node/points}"
      : "${MOLA_IMU_TOPIC:=/os_cloud_node/imu}"
      ;;
    ouster-os1)
      : "${MOLA_LIDAR_TOPIC:=/os_cloud_nodee/points}"
      : "${MOLA_IMU_TOPIC:=/os_cloud_nodee/imu}"
      ;;
    velodyne)
      : "${MOLA_LIDAR_TOPIC:=/velodyne_points}"
      # The VLP-16 has no IMU of its own on this rig; borrow the OS0's, which
      # is the only choice that does not also change the lidar under test.
      : "${MOLA_IMU_TOPIC:=/os_cloud_node/imu}"
      ;;
    livox-horizon)
      : "${MOLA_LIDAR_TOPIC:=/livox/lidar}"
      : "${MOLA_IMU_TOPIC:=/livox/imu}"
      non_repetitive=1
      ;;
    livox-avia)
      : "${MOLA_LIDAR_TOPIC:=/avia/livox/lidar}"
      : "${MOLA_IMU_TOPIC:=/avia/livox/imu}"
      non_repetitive=1
      ;;
    *)
      echo "Error: unknown TIERS_SENSOR '$TIERS_SENSOR'." >&2
      echo "       Expected one of: ouster-os0 ouster-os1 velodyne livox-horizon livox-avia" >&2
      return 1
      ;;
  esac
  export MOLA_LIDAR_TOPIC MOLA_IMU_TOPIC

  echo "TIERS sequence '$(basename "$file")', sensor '$TIERS_SENSOR':"
  echo "  LiDAR topic: $MOLA_LIDAR_TOPIC"
  echo "  IMU topic  : $MOLA_IMU_TOPIC"

  if [ "$non_repetitive" -eq 1 ]; then
    : "${MOLA_MIN_NEARBY_POSES_OCCUPIED:=2}"
    : "${MOLA_SIMPLEMAP_MIN_NEARBY_POSES:=2}"
    export MOLA_MIN_NEARBY_POSES_OCCUPIED MOLA_SIMPLEMAP_MIN_NEARBY_POSES
  fi

  # Co-located placeholder, see this file's header.
  : "${MOLA_TF_BASE_LINK:=base_link}"
  : "${LIDAR_POSE_X:=0}" ; : "${LIDAR_POSE_Y:=0}" ; : "${LIDAR_POSE_Z:=0}"
  : "${LIDAR_POSE_YAW:=0}" ; : "${LIDAR_POSE_PITCH:=0}" ; : "${LIDAR_POSE_ROLL:=0}"
  : "${MOLA_USE_FIXED_LIDAR_POSE:=true}"
  : "${IMU_POSE_X:=0}" ; : "${IMU_POSE_Y:=0}" ; : "${IMU_POSE_Z:=0}"
  : "${IMU_POSE_YAW:=0}" ; : "${IMU_POSE_PITCH:=0}" ; : "${IMU_POSE_ROLL:=0}"
  : "${MOLA_USE_FIXED_IMU_POSE:=true}"
  export MOLA_TF_BASE_LINK
  export LIDAR_POSE_X LIDAR_POSE_Y LIDAR_POSE_Z
  export LIDAR_POSE_YAW LIDAR_POSE_PITCH LIDAR_POSE_ROLL MOLA_USE_FIXED_LIDAR_POSE
  export IMU_POSE_X IMU_POSE_Y IMU_POSE_Z
  export IMU_POSE_YAW IMU_POSE_PITCH IMU_POSE_ROLL MOLA_USE_FIXED_IMU_POSE

  mola_lo_bag_slots "$file" || return 1

  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_rosbag1.yaml
  MOLA_LO_CLI_INPUT=(--input-rosbag1 "$MOLA_LO_BAGS_JOINED")
}
