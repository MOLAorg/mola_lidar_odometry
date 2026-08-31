# GrandTour (https://grand-tour.leggedrobotics.com): an ANYbotics ANYmal-D
# quadruped carrying the open-source "Boxi" sensor payload.
#
# Each mission is published as a set of per-topic ROS 1 bags, so the LiDAR,
# the IMU, /tf and the front camera live in SEPARATE files. They are replayed
# jointly through the generic rosbag1 launch file / a comma-joined
# --input-rosbag1, so no dataset-specific launch file is needed.
#
# Bags used (the rest of each mission's bags are ignored):
#   <mission>_hesai_undist.bag           /boxi/hesai/points_undistorted     10 Hz  (default LiDAR)
#   <mission>_livox_undist.bag           /boxi/livox/points_undistorted     10 Hz  (opt-in alternative LiDAR)
#   <mission>_anymal_velodyne_undist.bag /anymal/velodyne/points_undistorted 10 Hz (opt-in alternative LiDAR)
#   <mission>_tf_minimal.bag    /tf, /tf_static                          (optional)
#   <mission>_tf_model.bag      /tf, /tf_static                          (only for the STIM320, see below)
#   <mission>_adis.bag          /boxi/adis/imu                   200 Hz  (optional, default IMU)
#   <mission>_stim320_imu.bag   /boxi/stim320/imu                500 Hz  (opt-in alternative IMU)
#   <mission>_hdr_<tag>.bag     /boxi/hdr/<tag>/image_raw/...    10 Hz   (optional, GUI only)
#   <mission>_alphasense.bag    /boxi/alphasense/<tag>/image_...  10 Hz   (optional, GUI only)
#   <mission>_anymal_state.bag  /anymal/state_estimator/odometry         (opt-in, see below)
#
# The Boxi payload carries three LiDARs, not one: Hesai and Livox share the
# payload's own rigid group (box_base -> hesai_lidar / livox_lidar), while
# the Velodyne is mounted directly on the ANYmal body (base -> velodyne_lidar,
# hence its bag's "anymal_" prefix rather than a Boxi one). All three
# "_undist" streams are already-undistorted sensor_msgs/PointCloud2 at the
# same ~10 Hz, so they drop into the pipeline identically -- no message-type
# handling was added for this, unlike the Livox AVIA CustomMsg in the
# BotanicGarden profile. Select with MOLA_GRANDTOUR_LIDAR (below).
#
# Sensor extrinsics come from the dataset's own /tf_static (base -> box_base ->
# hesai_lidar / adis16475_imu / hdr_base -> hdr_front; base -> velodyne_lidar),
# so no fixed sensor poses are needed as long as the tf bag is present. Without
# it, the LiDAR falls back to a fixed pose at the origin (the extrinsics are
# then simply unknown), same for all three LiDARs.
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
  echo "Optional environment variables:"
  echo "  MOLA_GRANDTOUR_LIDAR   which LiDAR to use: 'hesai' (default, Boxi payload),"
  echo "                         'livox' (Boxi payload) or 'velodyne' (mounted on the"
  echo "                         ANYmal body itself, not the payload)"
  echo "  MOLA_ODOMETRY_TOPIC    the robot's legged kinematic-inertial odometry, fused"
  echo "                         by default (this adds <mission>_anymal_state.bag to"
  echo "                         the inputs). Set it EMPTY to run LiDAR-inertial only"
  echo "  MOLA_ODOMETRY_OBS_CLASS  how to read that topic: 'CObservationRobotPose'"
  echo "                         (default, full SE(3) + covariance) or the planar"
  echo "                         'CObservationOdometry'"
  echo "  MOLA_GRANDTOUR_IMU     which IMU to use: 'adis' (default, 200 Hz) or"
  echo "                         'stim320' (500 Hz, tactical grade). The STIM320 also"
  echo "                         requires <mission>_tf_model.bag, since its frame is"
  echo "                         absent from the smaller tf_minimal.bag"
  echo "  MOLA_GRANDTOUR_CAMERA  which camera to preview in the GUI (ignored in CLI"
  echo "                         mode): 'hdr_front' (default), 'hdr_left', 'hdr_right',"
  echo "                         or one of the five Alphasense cameras"
  echo "                         'alphasense_front_center', 'alphasense_front_left',"
  echo "                         'alphasense_front_right', 'alphasense_left',"
  echo "                         'alphasense_right'"
  echo ""
  echo "Example:"
  echo "  $0 ~/datasets/grand-tour/2024-10-01-11-29-55/"
}

# Map a LiDAR tag to the bag suffix and topic it is carried on. Shared by the
# primary sensor and by the optional second one fused with it, so the two
# cannot drift apart.
_grandtour_lidar_spec() {
  case "$1" in
    hesai)
      _gt_lidar_suffix=_hesai_undist.bag
      _gt_lidar_topic=/boxi/hesai/points_undistorted
      ;;
    livox)
      _gt_lidar_suffix=_livox_undist.bag
      _gt_lidar_topic=/boxi/livox/points_undistorted
      ;;
    velodyne)
      _gt_lidar_suffix=_anymal_velodyne_undist.bag
      _gt_lidar_topic=/anymal/velodyne/points_undistorted
      ;;
    *)
      echo "Error: $2 must be 'hesai', 'livox' or 'velodyne', got '$1'." >&2
      return 1
      ;;
  esac
}

mola_lo_profile_resolve() {
  local arg=$1
  shift
  MOLA_LO_EXTRA_ARGS=("$@")

  # Which LiDAR: the Hesai (Boxi payload) by default, or one of the other two
  # the robot also carries -- see the file header for how they're mounted.
  : "${MOLA_GRANDTOUR_LIDAR:=hesai}"
  local lidar_suffix
  local lidar_topic_default
  _grandtour_lidar_spec "$MOLA_GRANDTOUR_LIDAR" MOLA_GRANDTOUR_LIDAR || return 1
  lidar_suffix=$_gt_lidar_suffix
  lidar_topic_default=$_gt_lidar_topic

  # Resolve the "<dir>/<mission>" prefix shared by all of a mission's bags,
  # from either a mission directory or any one of its bag files:
  local lidar_bag
  if [ -d "$arg" ]; then
    lidar_bag=$(ls "${arg%/}"/*"$lidar_suffix" 2>/dev/null | head -n 1)
    if [ -z "$lidar_bag" ]; then
      echo "Error: no '*$lidar_suffix' found in directory '$arg' (MOLA_GRANDTOUR_LIDAR=$MOLA_GRANDTOUR_LIDAR)." >&2
      return 1
    fi
  elif [ -f "$arg" ]; then
    lidar_bag=$arg
  else
    echo "Error: '$arg' is neither a directory nor a file." >&2
    return 1
  fi

  local prefix=${lidar_bag%"$lidar_suffix"}
  if [ "$prefix" = "$lidar_bag" ]; then
    echo "Error: '$lidar_bag' does not follow the expected '<mission>$lidar_suffix' naming." >&2
    return 1
  fi

  # Which IMU: the ADIS16475 by default, or the higher-grade STIM320.
  #
  # The two are not interchangeable as far as /tf goes. `tf_minimal.bag`
  # publishes the ADIS's frame but NOT the STIM320's, so selecting the
  # STIM320 also selects the full `tf_model.bag`, which carries every
  # sensor frame (verified: it has base, hesai_lidar, prism, adis16475_imu
  # and stim320_imu, and the same /tf and /tf_static message counts).
  : "${MOLA_GRANDTOUR_IMU:=adis}"
  # MOLA_GRANDTOUR_TF_BAG lets a caller supply a TF bag with refined
  # extrinsics in place of the recorded one; empty keeps the mission's own.
  local tf_bag=${MOLA_GRANDTOUR_TF_BAG:-${prefix}_tf_minimal.bag}
  local imu_bag=${prefix}_adis.bag
  local imu_topic=/boxi/adis/imu

  case "$MOLA_GRANDTOUR_IMU" in
    adis) ;;
    stim320)
      tf_bag=${prefix}_tf_model.bag
      imu_bag=${prefix}_stim320_imu.bag
      imu_topic=/boxi/stim320/imu
      ;;
    *)
      echo "Error: MOLA_GRANDTOUR_IMU must be 'adis' or 'stim320', got '$MOLA_GRANDTOUR_IMU'." >&2
      return 1
      ;;
  esac

  # Which camera to preview. The three HDR cameras each ship in their own bag,
  # while the five Alphasense cameras share a single one, so the bag and the
  # topic have to be resolved together rather than derived from the tag alone.
  : "${MOLA_GRANDTOUR_CAMERA:=hdr_front}"
  local camera_bag
  local camera_topic

  case "$MOLA_GRANDTOUR_CAMERA" in
    hdr_front | hdr_left | hdr_right)
      camera_bag=${prefix}_${MOLA_GRANDTOUR_CAMERA}.bag
      camera_topic=/boxi/hdr/${MOLA_GRANDTOUR_CAMERA#hdr_}/image_raw/compressed
      ;;
    alphasense_front_center | alphasense_front_left | alphasense_front_right | \
      alphasense_left | alphasense_right)
      camera_bag=${prefix}_alphasense.bag
      camera_topic=/boxi/alphasense/${MOLA_GRANDTOUR_CAMERA#alphasense_}/image_raw/compressed
      ;;
    *)
      echo "Error: MOLA_GRANDTOUR_CAMERA must be one of hdr_{front,left,right} or" >&2
      echo "       alphasense_{front_center,front_left,front_right,left,right}," >&2
      echo "       got '$MOLA_GRANDTOUR_CAMERA'." >&2
      return 1
      ;;
  esac

  local odom_bag=${prefix}_anymal_state.bag

  echo "GrandTour mission '$(basename "$prefix")':"
  echo "  LiDAR bag: $lidar_bag  ($MOLA_GRANDTOUR_LIDAR)"

  # Optional second LiDAR, fused with the first as one scan group. The rig
  # carries three; the odometry groups them via multiple_lidars.lidar_count,
  # which the offline CLI derives from the number of comma-separated labels it
  # is given. Resolved here, before the topic default is frozen below.
  : "${MOLA_GRANDTOUR_LIDAR2:=}"
  local lidar2_bag=
  if [ -n "$MOLA_GRANDTOUR_LIDAR2" ]; then
    if [ "$MOLA_GRANDTOUR_LIDAR2" = "$MOLA_GRANDTOUR_LIDAR" ]; then
      echo "Error: MOLA_GRANDTOUR_LIDAR2 must differ from MOLA_GRANDTOUR_LIDAR." >&2
      return 1
    fi
    _grandtour_lidar_spec "$MOLA_GRANDTOUR_LIDAR2" MOLA_GRANDTOUR_LIDAR2 || return 1
    lidar2_bag=${prefix}${_gt_lidar_suffix}
    if [ ! -f "$lidar2_bag" ]; then
      echo "Error: MOLA_GRANDTOUR_LIDAR2=$MOLA_GRANDTOUR_LIDAR2 needs '$lidar2_bag', not found." >&2
      return 1
    fi
    echo "  LiDAR bag 2: $lidar2_bag  ($MOLA_GRANDTOUR_LIDAR2)"
    lidar_topic_default="${lidar_topic_default},${_gt_lidar_topic}"
    # One fixed pose cannot describe two sensors: the extrinsics must come
    # from /tf, which the TF bag below provides.
    MOLA_USE_FIXED_LIDAR_POSE=false
    export MOLA_USE_FIXED_LIDAR_POSE
  fi

  # GrandTour topic names and /tf frames:
  : "${MOLA_LIDAR_TOPIC:=$lidar_topic_default}"
  : "${MOLA_TF_BASE_LINK:=base}"  # the ANYmal body frame; NOT named 'base_link'
  export MOLA_LIDAR_TOPIC MOLA_TF_BASE_LINK

  local -a bags=("$lidar_bag")
  if [ -n "$lidar2_bag" ]; then
    bags+=("$lidar2_bag")
  fi

  if [ -f "$tf_bag" ]; then
    echo "  TF bag   : $tf_bag"
    bags+=("$tf_bag")
  elif [ "$MOLA_GRANDTOUR_IMU" = "stim320" ]; then
    # Unlike the ADIS case below, this cannot fall back to a fixed pose: the
    # whole reason this bag is required is that it is the only source of the
    # STIM320's extrinsics.
    echo "Error: MOLA_GRANDTOUR_IMU=stim320 needs '$tf_bag', which was not found." >&2
    echo "       Fetch it with: klein download -p GrandTourDataset -m release_<mission> \\" >&2
    echo "                        --dest <dir> --create-dirs -y <mission>_tf_model.bag" >&2
    return 1
  else
    echo "  TF bag   : (not found: '$tf_bag'; using a fixed LiDAR pose at the origin)"
    : "${MOLA_USE_FIXED_LIDAR_POSE:=1}"
    export MOLA_USE_FIXED_LIDAR_POSE
  fi

  if [ -f "$imu_bag" ]; then
    echo "  IMU bag  : $imu_bag  ($MOLA_GRANDTOUR_IMU)"
    bags+=("$imu_bag")
    : "${MOLA_IMU_TOPIC:=$imu_topic}"
    # A legged robot pitches and rolls constantly from the very first scan,
    # so level the initial pose from the IMU instead of assuming a flat start:
    : "${MOLA_LO_INITIAL_LOCALIZATION_METHOD:=InitLocalization::PitchAndRollFromIMU}"
    export MOLA_LO_INITIAL_LOCALIZATION_METHOD
  else
    echo "  IMU bag  : (not found: '$imu_bag'; running LiDAR-only odometry)"
    : "${MOLA_IMU_TOPIC:=}"
  fi
  export MOLA_IMU_TOPIC

  # Legged kinematic-inertial odometry, ON by default for this dataset. Every
  # other profile leaves odometry fusion opt-in, because a dataset is not opted
  # in merely by carrying a pose topic; here it is enabled because it was
  # measured to help, and the frame is known to be right.
  #
  # `/anymal/state_estimator/odometry` is a nav_msgs/Odometry reported as
  # odom -> base, i.e. for the very frame this profile already uses as
  # MOLA_TF_BASE_LINK. Checked by reading the messages' child_frame_id, not
  # assumed from the topic name.
  #
  # Read as CObservationRobotPose, not the default CObservationOdometry: the
  # latter is planar, so it would drop z, roll, pitch and the covariance this
  # source publishes -- exactly the components that matter on stairs.
  #
  # The velocity sigmas are deliberately loose. This source's pose channel is
  # good and its velocity channel is not worth trusting against ICP: measured
  # on the missions with a reference, tightening them degrades the result
  # monotonically (0.3146 -> 0.3952 -> 0.6496 -> 0.7947 m ATE on arc-3 as the
  # linear sigma goes 1.0 -> 0.3 -> 0.1 -> 0.03).
  #
  # Measured across the seven missions that have a reference: -13.3 % mean ATE,
  # driven almost entirely by the one mission that goes indoors (arc-3,
  # 0.3829 -> 0.3146). Set MOLA_ODOMETRY_TOPIC= (empty) to turn it off.
  # Note "=" and not ":=": an explicitly empty MOLA_ODOMETRY_TOPIC is how a
  # caller turns fusion off, and ":=" would treat that as unset and re-enable it.
  : "${MOLA_ODOMETRY_TOPIC=/anymal/state_estimator/odometry}"
  : "${MOLA_ODOMETRY_OBS_CLASS:=CObservationRobotPose}"
  : "${MOLA_NAVSTATE_SIGMA_WHEEL_ODOM_LINVEL:=1.0}"
  : "${MOLA_NAVSTATE_SIGMA_WHEEL_ODOM_ANGVEL:=0.5}"
  export MOLA_ODOMETRY_TOPIC MOLA_ODOMETRY_OBS_CLASS
  export MOLA_NAVSTATE_SIGMA_WHEEL_ODOM_LINVEL MOLA_NAVSTATE_SIGMA_WHEEL_ODOM_ANGVEL

  if [ -n "${MOLA_ODOMETRY_TOPIC:-}" ]; then
    if [ -f "$odom_bag" ]; then
      echo "  Odometry bag: $odom_bag"
      bags+=("$odom_bag")
    else
      echo "  Odometry bag: (not found: '$odom_bag'; MOLA_ODOMETRY_TOPIC matches nothing)"
    fi
  fi

  # The camera is a GUI preview only: nothing in the odometry consumes it, so
  # a batch run would pay for decoding several GB of JPEG for nothing.
  if [ "$MOLA_LO_MODE" = "gui" ] && [ -f "$camera_bag" ]; then
    echo "  Camera bag: $camera_bag  ($MOLA_GRANDTOUR_CAMERA)"
    bags+=("$camera_bag")
    : "${MOLA_CAMERA_TOPIC:=$camera_topic}"
    export MOLA_CAMERA_TOPIC
  elif [ "$MOLA_LO_MODE" = "gui" ]; then
    echo "  Camera bag: (not found: '$camera_bag'; no camera preview)"
  fi

  # The LiDAR stream used here is the dataset's already-undistorted one, so
  # deskewing it a second time would over-compensate the motion:
  : "${MOLA_DESKEW_METHOD:=MotionCompensationMethod::None}"
  # Use a shorter minimum range since the robot body is small in this dataset:
  #
  # This one is on a cliff edge, so re-tune it only with measurements in hand:
  # dropping to 1.0 m costs little, but raising it to 2.0 m degrades odometry by
  # more than an order of magnitude, and 3.0 m is *better* than 2.0 m. A legged
  # platform depends on the near-field ground returns this filter removes.
  : "${MOLA_MINIMUM_RANGE_FILTER:=1.5}"

  # A single incremental k-d tree in one global frame, rather than the default
  # keyframe-based local map. Measured across every mission of this dataset that
  # has a reference trajectory, it lowers absolute trajectory error on all of
  # them, by ~46% on average, at ~44% more CPU (still comfortably faster than
  # real time). A constantly pitching platform benefits from a stable, denser
  # local map instead of one rebuilt from a small rotating keyframe set.
  #
  # ODOMETRY ONLY: this map cannot be re-mapped by a global SE(3) correction, so
  # override it when running loop-closure SLAM on this dataset:
  #   MOLA_LOCALMAP_CLASS=mola::KeyframePointCloudMap
  : "${MOLA_LOCALMAP_CLASS:=mola::IncrementalPointCloud}"

  export MOLA_DESKEW_METHOD MOLA_MINIMUM_RANGE_FILTER MOLA_LOCALMAP_CLASS

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
