# BotanicGarden (https://github.com/robot-pesg/BotanicGarden): a ground robot
# driven through a botanical garden, recorded as one ROS 1 "LIO" bag per
# sequence.
#
# That bag records two lidars at once, so two pipelines are available:
#   (default) the spinning Velodyne VLP-16 -> lidar_odometry_from_botanicgarden.yaml
#   --livox   the solid-state Livox AVIA   -> lidar_odometry_from_botanicgarden_livox.yaml
# The Livox is non-repetitive (a single scan does not cover the full FOV), so
# its pipeline needs different tuning; see this repo's agents.md,
# "Non-repetitive (solid-state) LiDARs".
#
# base_link is the Xsens IMU (identity pose); the Velodyne sits at the
# T_xsens_vlp16 offset below. That is the launch file's own convention, not
# the dataset's top-level README, which names a different base_link.
#
# Wheel odometry (/odom) lives in the SAME bag and is fused when
# MOLA_ODOMETRY_TOPIC is set. This dataset's odometry frame is within ~1.5 deg
# of vehicle-aligned, so unlike CitrusFarm there is no silently-discarded
# rotation to worry about.

mola_lo_profile_usage() {
  echo "Error: A BotanicGarden ROS 1 '..._LIO.bag' file is required."
  echo "Usage: $0 /path/to/dataset_LIO.bag [--livox] [additional flags]"
  echo ""
  echo "Optional environment variables:"
  echo "  BOTANICGARDEN_GT_BAG   also replay /gt_poses from the matching gt bag"
  echo "                         (reference only; not consumed by the odometry)"
  echo "  MOLA_ODOMETRY_TOPIC    set to '/odom' to fuse wheel odometry"
}

mola_lo_profile_resolve() {
  # Split the bag path (first non-flag argument) from the --livox selector and
  # any remaining flags to forward on:
  local bag=""
  local use_livox=0
  MOLA_LO_EXTRA_ARGS=()
  local arg
  for arg in "$@"; do
    if [ "$arg" = "--livox" ]; then
      use_livox=1
    elif [ -z "$bag" ]; then
      bag=$arg
    else
      MOLA_LO_EXTRA_ARGS+=("$arg")
    fi
  done

  if [ -z "$bag" ]; then
    mola_lo_profile_usage
    return 1
  fi

  BOTANICGARDEN_LIO_BAG="$bag"
  export BOTANICGARDEN_LIO_BAG

  : "${MOLA_DESKEW_METHOD:=MotionCompensationMethod::IMU}"
  : "${MOLA_IGNORE_NO_POINT_STAMPS:=false}"
  # Accelerometer suppressed on top of the IMU deskew already selected above:
  # cuts the band 2.3x at unchanged path length. Deskew-only tuning -- unlike
  # Oxford Spires/KITTI, the GICP decimation voxel/stride tuning is NOT
  # applied here: it was never validated on this dataset, and a full-corpus
  # A/B run 2026-08-18 found it regresses BotanicGarden badly (+506% mean
  # APE) when it leaked in via a global pipeline default. See
  # lio/03_accuracy_pipeline.md ranked action 3c and §0.3.
  : "${MOLA_DESKEW_IGNORE_ACCELEROMETER:=true}"
  export MOLA_DESKEW_METHOD MOLA_IGNORE_NO_POINT_STAMPS MOLA_DESKEW_IGNORE_ACCELEROMETER

  if [ "$use_livox" -eq 1 ]; then
    MOLA_LO_LAUNCH_FILE=lidar_odometry_from_botanicgarden_livox.yaml
    : "${MOLA_LIDAR_TOPIC:=/livox/lidar}"
    # Non-repetitive scan pattern: accumulate 2+ scans per location before
    # moving on, for denser local map / simplemap coverage:
    : "${MOLA_MIN_NEARBY_POSES_OCCUPIED:=2}"
    : "${MOLA_SIMPLEMAP_MIN_NEARBY_POSES:=2}"
    export MOLA_MIN_NEARBY_POSES_OCCUPIED MOLA_SIMPLEMAP_MIN_NEARBY_POSES
  else
    MOLA_LO_LAUNCH_FILE=lidar_odometry_from_botanicgarden.yaml
    : "${MOLA_LIDAR_TOPIC:=/velodyne_points}"
  fi
  : "${MOLA_IMU_TOPIC:=/imu/data}"
  export MOLA_LIDAR_TOPIC MOLA_IMU_TOPIC

  # base_link is the Xsens IMU, at identity; the Velodyne at T_xsens_vlp16.
  # Only consumed by the offline path and the generic launch files: the two
  # dataset-specific launch files above carry the same values themselves.
  : "${MOLA_TF_BASE_LINK:=xsens}"
  : "${LIDAR_POSE_X:=0.0584868}"
  : "${LIDAR_POSE_Y:=0.0084042}"
  : "${LIDAR_POSE_Z:=0.1689155}"
  : "${LIDAR_POSE_YAW:=-1.448155}"
  : "${LIDAR_POSE_PITCH:=0.097462}"
  : "${LIDAR_POSE_ROLL:=0.444791}"
  : "${MOLA_USE_FIXED_LIDAR_POSE:=true}"
  : "${IMU_POSE_X:=0}" ; : "${IMU_POSE_Y:=0}" ; : "${IMU_POSE_Z:=0}"
  : "${IMU_POSE_YAW:=0}" ; : "${IMU_POSE_PITCH:=0}" ; : "${IMU_POSE_ROLL:=0}"
  : "${MOLA_USE_FIXED_IMU_POSE:=true}"
  export MOLA_TF_BASE_LINK
  export LIDAR_POSE_X LIDAR_POSE_Y LIDAR_POSE_Z
  export LIDAR_POSE_YAW LIDAR_POSE_PITCH LIDAR_POSE_ROLL MOLA_USE_FIXED_LIDAR_POSE
  export IMU_POSE_X IMU_POSE_Y IMU_POSE_Z
  export IMU_POSE_YAW IMU_POSE_PITCH IMU_POSE_ROLL MOLA_USE_FIXED_IMU_POSE

  # No GNSS in this dataset: pin the first pose to the map origin, or the
  # smoother's GTSAM graph is left with a rank-deficient null-space
  # (IndeterminantLinearSystemException):
  : "${MOLA_LINK_FIRST_POSE_SIGMA:=1e-6}"
  export MOLA_LINK_FIRST_POSE_SIGMA

  mola_lo_use_smoother || return 1

  local -a bags=("$bag")
  [ -n "${BOTANICGARDEN_GT_BAG:-}" ] && bags+=("$BOTANICGARDEN_GT_BAG")
  mola_lo_bag_slots "${bags[@]}" || return 1

  MOLA_LO_CLI_INPUT=(--input-rosbag1 "$MOLA_LO_BAGS_JOINED")
}
