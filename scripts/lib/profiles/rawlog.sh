# Generic MRPT rawlog input: not a particular dataset, so this profile only
# routes the file and leaves every sensor label at the launch file's default.

mola_lo_profile_usage() {
  echo "Error: A rawlog file is required."
  echo "Usage: $0 /path/to/dataset.rawlog [additional flags]"
  echo ""
  echo "Common environment variables:"
  echo "  MOLA_LIDAR_NAME   LiDAR sensor label in the rawlog (default: 'lidar')"
}

mola_lo_profile_resolve() {
  local file=$1
  shift
  MOLA_LO_EXTRA_ARGS=("$@")

  MOLA_INPUT_RAWLOG="$file"
  export MOLA_INPUT_RAWLOG

  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_rawlog.yaml
  MOLA_LO_CLI_INPUT=(--input-rawlog "$file")
}
