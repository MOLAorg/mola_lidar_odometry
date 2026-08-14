# Generic ROS 1 bag input: not a particular dataset, so this profile only
# routes the bag(s) and leaves every topic at the launch file's default. Pass
# several bags to replay them jointly as one merged stream, which is how
# per-topic datasets (lidar, /tf and the IMU in separate files) are handled
# without a launch file of their own.

mola_lo_profile_usage() {
  echo "Error: A ROS 1 bag file (.bag) is required."
  echo "Usage: $0 /path/to/dataset.bag [more .bag files] [additional flags]"
  echo ""
  echo "Common environment variables:"
  echo "  MOLA_LIDAR_TOPIC       LiDAR topic name (default: '/ouster/points')"
  echo "  MOLA_IMU_TOPIC         IMU topic name (default: '/imu')"
  echo "  MOLA_GNSS_TOPIC        GNSS NavSatFix topic name (default: '/gps')"
  echo "  MOLA_ODOMETRY_TOPIC    Wheels 2D odometry (optional)"
  echo "  MOLA_TF_BASE_LINK      Robot base /tf frame id (default: 'base_link')"
}

mola_lo_profile_resolve() {
  local -a bags=()
  MOLA_LO_EXTRA_ARGS=()
  local arg
  for arg in "$@"; do
    if [ "${#bags[@]}" -eq 0 ] || [[ "$arg" == *.bag ]]; then
      bags+=("$arg")
    else
      MOLA_LO_EXTRA_ARGS+=("$arg")
    fi
  done

  mola_lo_bag_slots "${bags[@]}" || return 1

  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_rosbag1.yaml
  MOLA_LO_CLI_INPUT=(--input-rosbag1 "$MOLA_LO_BAGS_JOINED")
}
