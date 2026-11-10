# NTU-VIRAL (https://ntu-aris.github.io/ntu_viral_dataset/): a drone with two
# Ouster OS1-16 (one horizontal, one vertical), a VectorNav VN100 IMU, cameras
# and UWB, one ROS 1 bag per sequence (eee_01..spms_03).
#
# There is no /tf in the bags, so every sensor pose is fixed here, from the
# dataset's own calibration (lidar_horz.yaml, lidar_vert.yaml, imu_v100.yaml).
# Their "Body" frame is the IMU's, and it points down (z-down), so base_link
# here is that body frame turned upside down, a 180 deg pitch:
#   horizontal LiDAR  T_Body_Lidar = [I | (-0.050, 0, 0.055)]
#                     => in base_link: 0.050 0 -0.055, pitch 180
#   IMU               T_Body_Imu = I  => in base_link: 0 0 0, pitch 180
#
# Only the horizontal LiDAR is used. The vertical one would sit at
# 0.550 0.030 -0.050, roll -90 in base_link, but the offline CLI gives every
# LiDAR the same fixed pose.
#
# The ground truth is the position of a Leica prism, not a pose: to score
# against it, move the estimate to the prism, which is at
# (0.293656, -0.012288, 0.273095) in this base_link (T_Body_Prism in
# leica_prism.yaml, turned the same way).

mola_lo_profile_usage() {
  echo "Error: An NTU-VIRAL ROS 1 '.bag' file is required."
  echo "Usage: $0 /path/to/eee_01.bag [additional flags]"
}

mola_lo_profile_resolve() {
  local file=$1
  shift
  MOLA_LO_EXTRA_ARGS=("$@")

  if [ -z "$file" ]; then
    mola_lo_profile_usage
    return 1
  fi

  : "${MOLA_LIDAR_TOPIC:=/os1_cloud_node1/points}"
  # The IMU helps a little (eee_03 / sbs_01 prism ATE: 119 / 90 mm without it,
  # 112 / 84 mm with it); IMU deskew on top did not improve further.
  : "${MOLA_IMU_TOPIC:=/imu/imu}"
  : "${MOLA_TF_BASE_LINK:=body}"
  export MOLA_LIDAR_TOPIC MOLA_IMU_TOPIC MOLA_TF_BASE_LINK

  # The Ouster driver stamps each cloud when it is published (end of the
  # sweep), while the per-point times run 0..0.1 s from the sweep start. Read
  # as-is, every pose lands ~0.11 s late against the IMU and the ground truth:
  # a constant shift of the LO poses minimizes the prism ATE at -0.08..-0.17 s
  # on 15 sequences (median -0.11 s, about half the ATE gone). This moves the
  # LiDAR stamps back by that median. It applies to the .bag files only:
  # rawlogs whose point times were already centered lag ~0.06 s instead.
  : "${MOLA_LIDAR_TIME_OFFSET:=-0.110}"
  export MOLA_LIDAR_TIME_OFFSET

  if [ "$MOLA_LO_MODE" = "gui" ]; then
    : "${MOLA_CAMERA_TOPIC:=/left/image_raw}"
    : "${MOLA_USE_FIXED_CAMERA_POSE:=1}"
    export MOLA_CAMERA_TOPIC MOLA_USE_FIXED_CAMERA_POSE
  fi

  : "${MOLA_USE_FIXED_IMU_POSE:=1}"
  : "${MOLA_USE_FIXED_LIDAR_POSE:=1}"
  export MOLA_USE_FIXED_IMU_POSE MOLA_USE_FIXED_LIDAR_POSE

  : "${LIDAR_POSE_X:=0.050}"
  : "${LIDAR_POSE_Y:=0.000}"
  : "${LIDAR_POSE_Z:=-0.055}"
  : "${LIDAR_POSE_YAW:=0}"
  : "${LIDAR_POSE_PITCH:=180}"
  : "${LIDAR_POSE_ROLL:=0}"
  : "${IMU_POSE_X:=0}" ; : "${IMU_POSE_Y:=0}" ; : "${IMU_POSE_Z:=0}"
  : "${IMU_POSE_YAW:=0}" ; : "${IMU_POSE_PITCH:=180}" ; : "${IMU_POSE_ROLL:=0}"
  export LIDAR_POSE_X LIDAR_POSE_Y LIDAR_POSE_Z
  export LIDAR_POSE_YAW LIDAR_POSE_PITCH LIDAR_POSE_ROLL
  export IMU_POSE_X IMU_POSE_Y IMU_POSE_Z
  export IMU_POSE_YAW IMU_POSE_PITCH IMU_POSE_ROLL

  # A small drone close to walls: a spherical minimum-range filter, as in the
  # hand-held profiles.
  : "${MOLA_MINIMUM_RANGE_FILTER_L_INFINITY:=false}"
  export MOLA_MINIMUM_RANGE_FILTER_L_INFINITY

  mola_lo_bag_slots "$file" || return 1
  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_rosbag1.yaml
  MOLA_LO_CLI_INPUT=(--input-rosbag1 "$MOLA_LO_BAGS_JOINED")
}
