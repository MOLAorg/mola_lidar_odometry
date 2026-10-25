# FoMo (https://fomo.norlab.ulaval.ca): a year of repeated drives of one
# off-road vehicle in a boreal forest, with six routes and twelve deployments
# that span bare ground, over a meter of snow, summer vegetation and night.
#
# The dataset ships no ROS bags, only one file per scan and CSV IMUs. Convert
# a trajectory to rosbag2 first, with these topics and frames: PointCloud2
# /robosense/points (frame robosense, float64 per-point time "t" in seconds
# relative to the scan start), Imu /vectornav/imu and /xsens/imu, and the
# robosense -> IMU transforms in /tf_static. This profile takes the
# resulting rosbag2 directory.
#
# The bag carries the 128-channel RoboSense lidar (/robosense/points, true
# per-point timestamps, not motion compensated) and the VectorNav IMU
# (/vectornav/imu, 200 Hz). The Xsens IMU is also recorded, as /xsens/imu.
# The static transforms from the lidar to both IMUs are in /tf_static.
#
# base_link is the lidar. The reference trajectory is a GNSS position only,
# fused from three receivers, so scoring needs the xyz-only metrics.

mola_lo_profile_usage() {
  echo "Error: A FoMo rosbag2 directory (containing metadata.yaml) is required."
  echo "Usage: $0 /path/to/<trajectory>/rosbag2 [additional flags]"
  echo ""
  echo "Optional environment variables:"
  echo "  MOLA_IMU_TOPIC   IMU topic (default: '/vectornav/imu'; '/xsens/imu' is the other)"
}

mola_lo_profile_resolve() {
  local file=$1
  shift
  MOLA_LO_EXTRA_ARGS=("$@")

  : "${MOLA_LIDAR_TOPIC:=/robosense/points}"
  : "${MOLA_IMU_TOPIC:=/vectornav/imu}"
  : "${MOLA_TF_BASE_LINK:=robosense}"
  export MOLA_LIDAR_TOPIC MOLA_IMU_TOPIC MOLA_TF_BASE_LINK

  MOLA_INPUT_ROSBAG2="$file"
  export MOLA_INPUT_ROSBAG2

  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_rosbag2.yaml
  MOLA_LO_CLI_INPUT=(--input-rosbag2 "$file")
}
