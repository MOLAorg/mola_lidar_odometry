# VBR, Vision Benchmark in Rome (https://rvp-group.net/slam-benchmark.html):
# one sensor head, carried by hand around four sites in Rome (colosseo,
# pincio, spagna, diag; Ouster OS0-128, 10 Hz) or mounted on a car (campus,
# ciampino; Ouster OS1-64, 20 Hz). Eight train sequences with public ground
# truth, eight test ones scored on the authors' public leaderboard.
#
# Upstream ships each sequence as ROS 1 bags split in ~10 GiB chunks
# (<seq>_00.bag, <seq>_01.bag, ...) plus a "vbr_calib.yaml". The bags carry
# no /tf at all, so every sensor pose below is fixed from that file, which is
# identical for both platforms. A LiDAR+IMU-only copy merged into a single
# <seq>_lio.bag works the same way.
#
# base_link is the LiDAR ("os_sensor"): it is the calibration's body frame
# (T_b = identity) and also the frame the ground truth is expressed in, so
# the estimate lands in the reference's own frame with no offset to compose.
#
# Two IMUs are recorded, both reading gravity along +Z:
#   sbg     /imu/data    SBG Ellipse-E, 0.61 m below the LiDAR (default; the
#                        one the dataset's camera-IMU calibration refers to)
#   ouster  /ouster/imu  the LiDAR's internal IMU, ~10x noisier
# VBR_IMU selects between them.

mola_lo_profile_usage() {
  echo "Error: A VBR sequence directory or its ROS 1 '.bag' file(s) are required."
  echo "Usage: $0 /path/to/<seq>/ [additional flags]"
  echo "       $0 /path/to/<seq>_lio.bag [additional flags]"
  echo "       $0 /path/to/<seq>_00.bag /path/to/<seq>_01.bag ... [additional flags]"
  echo ""
  echo "Optional environment variables:"
  echo "  VBR_IMU   'sbg' (default, /imu/data) or 'ouster' (/ouster/imu)."
  echo "  MOLA_GROUND_TRUTH_TUM_FILE  GT drawn by the GUI; defaults to"
  echo "            '<seq>_gt.txt' next to the bags (train sequences only)."
}

mola_lo_profile_resolve() {
  local -a bags=()
  MOLA_LO_EXTRA_ARGS=()
  local arg
  for arg in "$@"; do
    if [ -d "$arg" ]; then
      # A sequence directory: prefer the merged LiDAR+IMU copy, otherwise
      # every upstream chunk, in recording order.
      local -a found=()
      mapfile -t found < <(find "$arg" -maxdepth 1 -name '*_lio.bag' | sort)
      if [ "${#found[@]}" -eq 0 ]; then
        mapfile -t found < <(find "$arg" -maxdepth 1 -name '*_[0-9][0-9].bag' | sort)
      fi
      if [ "${#found[@]}" -eq 0 ]; then
        echo "Error: no '*_lio.bag' or '*_NN.bag' files in '$arg'." >&2
        return 1
      fi
      bags+=("${found[@]}")
    elif [[ "$arg" == *.bag ]]; then
      bags+=("$arg")
    else
      MOLA_LO_EXTRA_ARGS+=("$arg")
    fi
  done

  if [ "${#bags[@]}" -eq 0 ]; then
    mola_lo_profile_usage
    return 1
  fi

  # Train sequences ship their ground truth next to the bags, as
  # <seq>_gt.txt, with <seq> taken from "<seq>_lio.bag" or "<seq>_NN.bag":
  if [ -z "${MOLA_GROUND_TRUTH_TUM_FILE:-}" ]; then
    local first_bag=${bags[0]}
    local seq
    seq=$(basename -- "$first_bag")
    seq=${seq%.bag}
    if [[ "$seq" == *_lio ]]; then
      seq=${seq%_lio}
    elif [[ "$seq" =~ ^(.*)_[0-9][0-9]$ ]]; then
      seq=${BASH_REMATCH[1]}
    fi
    local gt
    gt="$(dirname -- "$first_bag")/${seq}_gt.txt"
    if [ -f "$gt" ]; then
      MOLA_GROUND_TRUTH_TUM_FILE=$gt
      export MOLA_GROUND_TRUTH_TUM_FILE
    fi
  fi

  : "${MOLA_LIDAR_TOPIC:=/ouster/points}"
  : "${MOLA_TF_BASE_LINK:=os_sensor}"
  export MOLA_LIDAR_TOPIC MOLA_TF_BASE_LINK

  : "${LIDAR_POSE_X:=0}" ; : "${LIDAR_POSE_Y:=0}" ; : "${LIDAR_POSE_Z:=0}"
  : "${LIDAR_POSE_YAW:=0}" ; : "${LIDAR_POSE_PITCH:=0}" ; : "${LIDAR_POSE_ROLL:=0}"
  : "${MOLA_USE_FIXED_LIDAR_POSE:=true}"
  export LIDAR_POSE_X LIDAR_POSE_Y LIDAR_POSE_Z
  export LIDAR_POSE_YAW LIDAR_POSE_PITCH LIDAR_POSE_ROLL MOLA_USE_FIXED_LIDAR_POSE

  # IMU poses are vbr_calib.yaml's T_b (sensor pose in the LiDAR frame),
  # rotations converted to yaw/pitch/roll in degrees.
  : "${VBR_IMU:=sbg}"
  case "$VBR_IMU" in
    sbg)
      : "${MOLA_IMU_TOPIC:=/imu/data}"
      : "${IMU_POSE_X:=-0.045630}"
      : "${IMU_POSE_Y:=-0.007849}"
      : "${IMU_POSE_Z:=-0.608415}"
      : "${IMU_POSE_YAW:=-1.8340}"
      : "${IMU_POSE_PITCH:=-0.3841}"
      : "${IMU_POSE_ROLL:=0.5167}"
      ;;
    ouster)
      : "${MOLA_IMU_TOPIC:=/ouster/imu}"
      : "${IMU_POSE_X:=0.006253}"
      : "${IMU_POSE_Y:=-0.011775}"
      : "${IMU_POSE_Z:=0.007645}"
      : "${IMU_POSE_YAW:=0}" ; : "${IMU_POSE_PITCH:=0}" ; : "${IMU_POSE_ROLL:=0}"
      ;;
    *)
      echo "Error: VBR_IMU must be 'sbg' or 'ouster', got '$VBR_IMU'." >&2
      return 1
      ;;
  esac
  : "${MOLA_USE_FIXED_IMU_POSE:=true}"
  export VBR_IMU MOLA_IMU_TOPIC
  export IMU_POSE_X IMU_POSE_Y IMU_POSE_Z
  export IMU_POSE_YAW IMU_POSE_PITCH IMU_POSE_ROLL MOLA_USE_FIXED_IMU_POSE

  # GUI preview camera: only present in the upstream chunks, not in a
  # LiDAR+IMU-only copy (the camera source is optional in the launch file).
  if [ "$MOLA_LO_MODE" = "gui" ]; then
    : "${MOLA_CAMERA_TOPIC:=/camera_left/image_raw}"
    : "${CAMERA_POSE_X:=0.070739}"
    : "${CAMERA_POSE_Y:=0.234351}"
    : "${CAMERA_POSE_Z:=-0.666049}"
    : "${CAMERA_POSE_YAW:=-89.6782}"
    : "${CAMERA_POSE_PITCH:=-0.0757}"
    : "${CAMERA_POSE_ROLL:=-90.0742}"
    : "${MOLA_USE_FIXED_CAMERA_POSE:=true}"
    export MOLA_CAMERA_TOPIC CAMERA_POSE_X CAMERA_POSE_Y CAMERA_POSE_Z
    export CAMERA_POSE_YAW CAMERA_POSE_PITCH CAMERA_POSE_ROLL MOLA_USE_FIXED_CAMERA_POSE
  fi

  # The clouds carry per-point time ("t"), and half the corpus is hand-held:
  # IMU deskewing and IMU-derived initial pitch/roll, as for other hand-held
  # rigs.
  : "${MOLA_LO_INITIAL_LOCALIZATION_METHOD:=InitLocalization::PitchAndRollFromIMU}"
  : "${MOLA_DESKEW_METHOD:=MotionCompensationMethod::IMU}"
  export MOLA_LO_INITIAL_LOCALIZATION_METHOD MOLA_DESKEW_METHOD

  # No GNSS: pin the first pose (set inside mola_lo_use_smoother).
  mola_lo_use_smoother || return 1

  if [ "$MOLA_LO_MODE" = "gui" ]; then
    mola_lo_bag_slots "${bags[@]}" || return 1
  else
    local IFS=,
    MOLA_LO_BAGS_JOINED="${bags[*]}"
  fi
  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_rosbag1.yaml
  MOLA_LO_CLI_INPUT=(--input-rosbag1 "$MOLA_LO_BAGS_JOINED")
}
