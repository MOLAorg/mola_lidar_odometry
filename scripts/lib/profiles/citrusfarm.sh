# CitrusFarm (https://ucr-robotics.github.io/Citrus-Farm-Dataset/): a Clearpath
# Jackal driven through citrus orchard rows.
#
# Each sequence ships a "base_*.bag" (LiDAR+IMU+GPS) -- most sequences split
# across several parts, which must all be replayed, in recording order -- and
# a matching "odom_*.bag" (wheel odometry).
#
# base_link is the Microstrain GX5 IMU, at identity; the extrinsics below come
# from the dataset's own CAD + Kalibr calibration chain, not from a placeholder.
# lidar_odometry_from_citrusfarm.yaml carries the same values as its defaults.
#
# Wheel odometry is NOT fused by default, and that is deliberate rather than an
# oversight: mrpt::obs::CObservationOdometry cannot carry a sensor pose at all,
# so the ~180 deg yaw between the Jackal chassis frame and the "imu" frame the
# rest of the pipeline uses is silently discarded, and the increment gets fused
# pointing the wrong way. Set MOLA_ODOMETRY_TOPIC explicitly if you want it
# anyway. Revisit once CObservationOdometry rotation exists upstream.

mola_lo_profile_usage() {
  echo "Error: A CitrusFarm ROS 1 'base_*.bag' file is required."
  echo "Usage: $0 /path/to/base_*.bag [more base_*.bag parts] [/path/to/odom_*.bag] [additional flags]"
}

mola_lo_profile_resolve() {
  # Split the base bag parts (the first argument, plus any later one whose
  # basename starts with "base_") and, if given, the odom bag (basename
  # starting with "odom_") from any remaining flags to forward on.
  local -a base_bags=()
  local odom_bag=""
  MOLA_LO_EXTRA_ARGS=()
  local arg arg_basename
  for arg in "$@"; do
    arg_basename=$(basename -- "$arg")
    # The odometry bag is recognized wherever it appears, including first, so
    # it never consumes a base slot and shifts the parts by one.
    if [ -z "$odom_bag" ] && [[ "$arg_basename" == odom_* ]]; then
      odom_bag=$arg
    # The first non-odometry argument is the first base part whatever it is
    # called, so a renamed or re-exported bag keeps working; later parts have
    # to be recognizable as such to tell them from flags.
    elif [ "${#base_bags[@]}" -eq 0 ] || [[ "$arg_basename" == base_* ]]; then
      base_bags+=("$arg")
    else
      MOLA_LO_EXTRA_ARGS+=("$arg")
    fi
  done

  if [ "${#base_bags[@]}" -eq 0 ]; then
    mola_lo_profile_usage
    return 1
  fi

  : "${MOLA_LIDAR_TOPIC:=/velodyne_points}"
  : "${MOLA_IMU_TOPIC:=/microstrain/imu/data}"
  : "${MOLA_TF_BASE_LINK:=imu}"
  export MOLA_LIDAR_TOPIC MOLA_IMU_TOPIC MOLA_TF_BASE_LINK

  # T_imu_lidar, derived from CAD + the Kalibr calibration chain. There is no
  # /tf or /tf_static in these bags, so fixed poses are mandatory.
  : "${LIDAR_POSE_X:=0.0285}"
  : "${LIDAR_POSE_Y:=0.0091}"
  : "${LIDAR_POSE_Z:=0.0776}"
  : "${LIDAR_POSE_YAW:=179.8419}"
  : "${LIDAR_POSE_PITCH:=-0.3290}"
  : "${LIDAR_POSE_ROLL:=0.1130}"
  : "${MOLA_USE_FIXED_LIDAR_POSE:=true}"
  : "${IMU_POSE_X:=0}" ; : "${IMU_POSE_Y:=0}" ; : "${IMU_POSE_Z:=0}"
  : "${IMU_POSE_YAW:=0}" ; : "${IMU_POSE_PITCH:=0}" ; : "${IMU_POSE_ROLL:=0}"
  : "${MOLA_USE_FIXED_IMU_POSE:=true}"
  export LIDAR_POSE_X LIDAR_POSE_Y LIDAR_POSE_Z
  export LIDAR_POSE_YAW LIDAR_POSE_PITCH LIDAR_POSE_ROLL MOLA_USE_FIXED_LIDAR_POSE
  export IMU_POSE_X IMU_POSE_Y IMU_POSE_Z
  export IMU_POSE_YAW IMU_POSE_PITCH IMU_POSE_ROLL MOLA_USE_FIXED_IMU_POSE

  # The launch file takes the odom bag in a slot of its own, so it never
  # displaces a base part; the offline CLI takes one comma-joined list.
  MOLA_INPUT_ROSBAG1_ODOM="$odom_bag"
  export MOLA_INPUT_ROSBAG1_ODOM

  mola_lo_bag_slots "${base_bags[@]}" || return 1

  local cli_bags=$MOLA_LO_BAGS_JOINED
  # Only join the odometry bag in when its topic is actually enabled: an
  # unused bag in the list is just replay cost, and joining it silently would
  # also diverge from the online path, where the slot is separate.
  if [ -n "$odom_bag" ] && [ -n "${MOLA_ODOMETRY_TOPIC:-}" ]; then
    cli_bags="$cli_bags,$odom_bag"
  fi

  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_citrusfarm.yaml
  MOLA_LO_CLI_INPUT=(--input-rosbag1 "$cli_bags")
}
