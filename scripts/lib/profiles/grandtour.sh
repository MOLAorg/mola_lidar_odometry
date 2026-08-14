# GrandTour (https://grand-tour.leggedrobotics.com): an ANYbotics ANYmal-D
# quadruped carrying the open-source "Boxi" sensor payload.
#
# Each mission is published as a set of per-topic ROS 1 bags, so the LiDAR,
# the IMU, /tf and the front camera live in SEPARATE files. They are replayed
# jointly through the generic rosbag1 launch file / a comma-joined
# --input-rosbag1, so no dataset-specific launch file is needed.
#
# Bags used (the rest of each mission's bags are ignored):
#   <mission>_hesai_undist.bag  /boxi/hesai/points_undistorted   10 Hz   (required)
#   <mission>_tf_minimal.bag    /tf, /tf_static                          (optional)
#   <mission>_adis.bag          /boxi/adis/imu                   200 Hz  (optional)
#   <mission>_hdr_front.bag     /boxi/hdr/front/image_raw/...    ~11 Hz  (optional, GUI only)
#
# Sensor extrinsics come from the dataset's own /tf_static (base -> box_base ->
# hesai_lidar / adis16475_imu / hdr_base -> hdr_front), so no fixed sensor
# poses are needed as long as the tf bag is present. Without it, the LiDAR
# falls back to a fixed pose at the origin (the payload extrinsics are then
# simply unknown).
#
# Scoring against this dataset's ground truth needs a body-frame correction:
# every GT source is anchored to a physical sensor mount (cpt7_imu, 0.293 m
# from base; or the total-station prism, 0.395 m), not to base, which is what
# the odometry reports. See the dataset's own COMFORT benchmark rules.

mola_lo_profile_usage() {
  echo "Error: A GrandTour mission directory (or its '*_hesai_undist.bag') is required."
  echo "Usage: $0 /path/to/<mission-dir>/ [additional flags]"
  echo "       $0 /path/to/<mission>_hesai_undist.bag [additional flags]"
  echo ""
  echo "Example:"
  echo "  $0 ~/datasets/grand-tour/2024-10-01-11-29-55/"
}

mola_lo_profile_resolve() {
  local arg=$1
  shift
  MOLA_LO_EXTRA_ARGS=("$@")

  # Resolve the "<dir>/<mission>" prefix shared by all of a mission's bags,
  # from either a mission directory or any one of its bag files:
  local lidar_bag
  if [ -d "$arg" ]; then
    lidar_bag=$(ls "${arg%/}"/*_hesai_undist.bag 2>/dev/null | head -n 1)
    if [ -z "$lidar_bag" ]; then
      echo "Error: no '*_hesai_undist.bag' found in directory '$arg'." >&2
      return 1
    fi
  elif [ -f "$arg" ]; then
    lidar_bag=$arg
  else
    echo "Error: '$arg' is neither a directory nor a file." >&2
    return 1
  fi

  local prefix=${lidar_bag%_hesai_undist.bag}
  if [ "$prefix" = "$lidar_bag" ]; then
    echo "Error: '$lidar_bag' does not follow the expected '<mission>_hesai_undist.bag' naming." >&2
    return 1
  fi

  local tf_bag=${prefix}_tf_minimal.bag
  local imu_bag=${prefix}_adis.bag
  local camera_bag=${prefix}_hdr_front.bag

  echo "GrandTour mission '$(basename "$prefix")':"
  echo "  LiDAR bag: $lidar_bag"

  # GrandTour topic names and /tf frames:
  : "${MOLA_LIDAR_TOPIC:=/boxi/hesai/points_undistorted}"
  : "${MOLA_TF_BASE_LINK:=base}"  # the ANYmal body frame; NOT named 'base_link'
  export MOLA_LIDAR_TOPIC MOLA_TF_BASE_LINK

  local -a bags=("$lidar_bag")

  if [ -f "$tf_bag" ]; then
    echo "  TF bag   : $tf_bag"
    bags+=("$tf_bag")
  else
    echo "  TF bag   : (not found: '$tf_bag'; using a fixed LiDAR pose at the origin)"
    : "${MOLA_USE_FIXED_LIDAR_POSE:=1}"
    export MOLA_USE_FIXED_LIDAR_POSE
  fi

  if [ -f "$imu_bag" ]; then
    echo "  IMU bag  : $imu_bag"
    bags+=("$imu_bag")
    : "${MOLA_IMU_TOPIC:=/boxi/adis/imu}"
    # A legged robot pitches and rolls constantly from the very first scan,
    # so level the initial pose from the IMU instead of assuming a flat start:
    : "${MOLA_LO_INITIAL_LOCALIZATION_METHOD:=InitLocalization::PitchAndRollFromIMU}"
    export MOLA_LO_INITIAL_LOCALIZATION_METHOD
  else
    echo "  IMU bag  : (not found: '$imu_bag'; running LiDAR-only odometry)"
    : "${MOLA_IMU_TOPIC:=}"
  fi
  export MOLA_IMU_TOPIC

  # The camera is a GUI preview only: nothing in the odometry consumes it, so
  # a batch run would pay for decoding several GB of JPEG for nothing.
  if [ "$MOLA_LO_MODE" = "gui" ] && [ -f "$camera_bag" ]; then
    echo "  Camera bag: $camera_bag"
    bags+=("$camera_bag")
    : "${MOLA_CAMERA_TOPIC:=/boxi/hdr/front/image_raw/compressed}"
    export MOLA_CAMERA_TOPIC
  fi

  # The LiDAR stream used here is the dataset's already-undistorted one, so
  # deskewing it a second time would over-compensate the motion:
  : "${MOLA_DESKEW_METHOD:=MotionCompensationMethod::None}"
  # Use a shorter minimum range since the robot body is small in this dataset:
  : "${MOLA_MINIMUM_RANGE_FILTER:=1.5}"
  export MOLA_DESKEW_METHOD MOLA_MINIMUM_RANGE_FILTER

  if [ "$MOLA_LO_MODE" = "gui" ]; then
    # This is a legged robot with a full joint tree in /tf, which is the whole
    # point of showing it here, so turn the tf tree view on by default.
    # The four frames skipped below are the ones that are NOT physically part
    # of the robot, and would otherwise be drawn as if they were:
    #   odom                  the world-fixed odometry frame (published
    #                         inverted, as a child of 'base')
    #   enu_origin            the geodetic reference frame (child of 'cpt7_imu')
    #   dlio_odom, dlio_map   the onboard SLAM's own frames (children of
    #                         'hesai_lidar')
    # Everything else in the tree is real hardware on the body: the four legs,
    # the IMUs, the LiDARs, the cameras, and the total-station 'prism'.
    : "${MOLA_LO_SHOW_TF_TREE:=true}"
    : "${MOLA_LO_TF_TREE_EXCLUDE:=odom,enu_origin,dlio_odom,dlio_map}"
    export MOLA_LO_SHOW_TF_TREE MOLA_LO_TF_TREE_EXCLUDE
  fi

  mola_lo_bag_slots "${bags[@]}" || return 1

  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_rosbag1.yaml
  MOLA_LO_CLI_INPUT=(--input-rosbag1 "$MOLA_LO_BAGS_JOINED")
}
