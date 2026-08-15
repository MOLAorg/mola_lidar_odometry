# Generic ROS 2 bag input: not a particular dataset, so this profile only
# routes the bag and leaves every topic at the launch file's default. Split
# bags are given as the directory containing their metadata.yaml.

mola_lo_profile_usage() {
  echo "Error: A ROS 2 bag is required."
  echo "Usage: $0 /path/to/dataset.mcap [additional flags]"
  echo "       $0 /path/to/dataset_directory    (must contain a metadata.yaml)"
  echo ""
  echo "Common environment variables:"
  echo "  MOLA_LIDAR_TOPIC       LiDAR topic name (default: '/ouster/points')"
  echo "  MOLA_IMU_TOPIC         IMU topic name (default: '/imu')"
  echo "  MOLA_GNSS_TOPIC        GNSS NavSatFix topic name (default: '/gps')"
  echo "  MOLA_ODOMETRY_TOPIC    Wheels 2D odometry (optional)"
  echo "  MOLA_TF_BASE_LINK      Robot base /tf frame id (default: 'base_link')"
  echo "  MOLA_TF_TOPIC          /tf topic in the bag (default: '/tf')"
  echo "  MOLA_TF_STATIC_TOPIC   /tf_static topic in the bag (default: '/tf_static')"
  echo "                         Override both for namespaced bags (e.g. /robot1/tf)."
}

mola_lo_profile_resolve() {
  local file=$1
  shift
  MOLA_LO_EXTRA_ARGS=("$@")

  MOLA_INPUT_ROSBAG2="$file"
  export MOLA_INPUT_ROSBAG2

  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_rosbag2.yaml
  MOLA_LO_CLI_INPUT=(--input-rosbag2 "$file")
}
