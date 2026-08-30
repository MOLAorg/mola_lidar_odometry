# Newer College Dataset (https://ori-drs.github.io/newer-college-dataset/):
# a handheld rig walked around New College, Oxford. The most-cited handheld
# LiDAR benchmark, and one of the few that ships a survey-grade prior map, so
# it serves trajectory and map-quality scoring from the same recording.
#
# Two collections, two different rigs. This profile is written for the 2020
# one and says so:
#
#   2020  Ouster OS1-64 + RealSense D435i   /os1_cloud_node/points, /os1_cloud_node/imu
#   2021  Ouster OS0-128 + Alphasense       different topics entirely
#
# NEWER_COLLEGE_YEAR selects between them. Only 2020 has been exercised
# against real bags here; the 2021 branch is written from its published
# topic naming and must be checked against a bag before it is trusted.
#
# THE ONE THING THAT WILL BITE YOU: these 2020 bags carry ROS 1-era frame
# ids WITH A LEADING SLASH -- "/os1_lidar", "/os1_imu" -- while tf2
# canonicalizes names on insertion, so the tf tree holds them unslashed and
# no lookup matched. Every observation was dropped, and until
# mola_input_rosbag1 08a5754 the warning about that dropped observation
# segfaulted the process. With that fix in, tf resolves normally and this
# profile needs no workaround; without it, set the fixed poses below.
#
# The sequences are split across MANY bags (10 for 01_short, 16 for
# 02_long). mola_lo_bag_slots() caps at 5 because the GUI launch files expose
# five slots, so the gui wrapper cannot replay a full sequence; the offline
# CLI takes a comma-separated list of any length and is what the eval uses.

mola_lo_profile_usage() {
  echo "Error: One or more Newer College ROS 1 '.bag' files are required."
  echo "Usage: $0 /path/to/rooster_*.bag [more bags ...] [additional flags]"
  echo ""
  echo "Optional environment variables:"
  echo "  NEWER_COLLEGE_YEAR   '2020' (default, OS1-64 + RealSense) or"
  echo "                       '2021' (OS0-128 + Alphasense). The two"
  echo "                       collections share no topic names."
}

mola_lo_profile_resolve() {
  local -a bags=()
  MOLA_LO_EXTRA_ARGS=()
  local arg
  for arg in "$@"; do
    case "$arg" in
      -*) MOLA_LO_EXTRA_ARGS+=("$arg") ;;
      *.bag) bags+=("$arg") ;;
      *) MOLA_LO_EXTRA_ARGS+=("$arg") ;;
    esac
  done

  if [ "${#bags[@]}" -eq 0 ]; then
    mola_lo_profile_usage
    return 1
  fi

  : "${NEWER_COLLEGE_YEAR:=2020}"
  case "$NEWER_COLLEGE_YEAR" in
    2020)
      : "${MOLA_LIDAR_TOPIC:=/os1_cloud_node/points}"
      : "${MOLA_IMU_TOPIC:=/os1_cloud_node/imu}"
      # The RealSense also publishes /camera/imu, /camera/accel/sample and
      # /camera/gyro/sample. The Ouster's own IMU is the one the published
      # calibration refers to, so it is the default here.
      if [ "$MOLA_LO_MODE" = "gui" ]; then
        : "${MOLA_CAMERA_TOPIC:=/camera/infra1/image_rect_raw}"
        : "${MOLA_USE_FIXED_CAMERA_POSE:=1}"
        export MOLA_CAMERA_TOPIC MOLA_USE_FIXED_CAMERA_POSE
      fi
      ;;
    2021)
      # NOT yet verified against a bag on this machine -- only the 2021
      # prior maps were downloaded, not its sequences. Check with
      # `--list`-style inspection before trusting these.
      : "${MOLA_LIDAR_TOPIC:=/os_cloud_node/points}"
      : "${MOLA_IMU_TOPIC:=/os_cloud_node/imu}"
      ;;
    *)
      echo "Error: NEWER_COLLEGE_YEAR must be 2020 or 2021, got '$NEWER_COLLEGE_YEAR'." >&2
      return 1
      ;;
  esac
  export NEWER_COLLEGE_YEAR MOLA_LIDAR_TOPIC MOLA_IMU_TOPIC

  # base_link is the lidar, which is also the frame the published ground
  # truth is expressed in, so the estimate lands in the reference's own frame
  # and needs no offset composed afterwards.
  #
  # The extrinsics are left to /tf_static rather than pinned here: these bags
  # DO carry it (os1_sensor -> os1_lidar, os1_sensor -> os1_imu), and with
  # the slash-canonicalization fix it resolves. Setting MOLA_USE_FIXED_*_POSE
  # is the escape hatch for an older mola_input_rosbag1; the values, composed
  # from that same /tf_static, are:
  #   T_lidar_imu = (-0.006253, 0.011775, -0.028535), yaw 180 deg
  # The 180 deg matters -- a translation-only version of this is wrong twice
  # over, the same trap grand-tour had.
  : "${MOLA_TF_BASE_LINK:=os1_lidar}"
  export MOLA_TF_BASE_LINK

  # Handheld, so the motion is faster and rotates harder than a vehicle's:
  # lean on the IMU both for deskewing and for the initial pitch/roll, as
  # conslam and hilti2022 do.
  : "${MOLA_LO_INITIAL_LOCALIZATION_METHOD:=InitLocalization::PitchAndRollFromIMU}"
  : "${MOLA_DESKEW_METHOD:=MotionCompensationMethod::IMU}"
  export MOLA_LO_INITIAL_LOCALIZATION_METHOD MOLA_DESKEW_METHOD

  # No GNSS: pin the first pose or the smoother's graph is rank-deficient
  # and it gives up after two scans with "underconstrained variables".
  : "${MOLA_LINK_FIRST_POSE_SIGMA:=1e-6}"
  export MOLA_LINK_FIRST_POSE_SIGMA

  mola_lo_use_smoother || return 1

  # The offline CLI splits a comma-separated list into a YAML sequence, so a
  # 10- or 16-bag sequence replays in one run. mola_lo_bag_slots() is used
  # only for the gui path's five slots, and refuses more than that.
  if [ "$MOLA_LO_MODE" = "gui" ]; then
    mola_lo_bag_slots "${bags[@]}" || return 1
  else
    local IFS=,
    MOLA_LO_BAGS_JOINED="${bags[*]}"
  fi
  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_rosbag1.yaml
  MOLA_LO_CLI_INPUT=(--input-rosbag1 "$MOLA_LO_BAGS_JOINED")
}
