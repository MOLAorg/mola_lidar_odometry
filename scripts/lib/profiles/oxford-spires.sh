# Oxford Spires Dataset (https://dynamic.robots.ox.ac.uk/datasets/oxford-spires):
# a hand-held/backpack VILENS payload walked around Oxford colleges.
#
# A sequence is a directory containing raw/ros2bag/, under which the recording
# may be split across several ros2bag part directories. All parts must be
# replayed, or the run silently covers only a fraction of the ground truth.
# Part directories end in "_<n>"; they are ordered by that number, not
# lexicographically, so a sequence with ten or more parts still replays in
# order.
#
# There is no /tf or /tf_static in these bags, so fixed sensor poses are
# mandatory. lidar_odometry_from_oxford_spires.yaml carries the same values as
# its defaults.
#
# Those poses are in the dataset's BASE frame, NOT the raw LiDAR frame, and
# the difference is a 180 deg yaw. Getting that wrong -- placing the LiDAR at
# identity, as if base and lidar were the same frame -- leaves every reported
# pose a body yaw away from the dataset's own ground truth. APE hides it (a
# body yaw moves position only by the 0.124 m lever arm, and evo's --align
# absorbs the rest); it negates the horizontal part of every RELATIVE motion,
# so vertical-drift and tilt metrics blow up. Symptom if it ever comes back:
# relative vertical drift in the tens-to-hundreds of mm/m on a run whose APE
# looks perfectly good.
#
# T_base_lidar is sensor.yaml's own value. T_base_imu is the LiDAR-frame IMU
# pose composed through it (T_base_lidar . T_lidar_imu), which reproduces
# sensor.yaml's independently recorded T_base_imu to ~1 mm and 0.6 deg, so
# the two calibrations agree.

# Number of OXFORD_SPIRES_BAG_N slots the launch YAML defines:
OXFORD_SPIRES_MAX_BAG_PARTS=6

mola_lo_profile_usage() {
  echo "Error: An Oxford Spires sequence directory is required."
  echo "Usage: $0 /path/to/data/sequences/<sequence-name>/ [--headless] [additional flags]"
  echo ""
  echo "Example:"
  echo "  $0 /mnt/storage/oxford-spires/data/sequences/2024-03-12-keble-college-01/"
  echo ""
  echo "--headless (alias --no-gui): run without the 3D GUI (batch/CI)."
}

mola_lo_profile_resolve() {
  local seq_dir=$1
  shift

  # --headless / --no-gui is consumed here, not forwarded: mola-cli would not
  # understand it. It just sets the same MOLA_WITH_GUI the launch YAML reads.
  MOLA_LO_EXTRA_ARGS=()
  local arg
  for arg in "$@"; do
    case "$arg" in
      --headless | --no-gui)
        MOLA_WITH_GUI=false
        export MOLA_WITH_GUI
        ;;
      *) MOLA_LO_EXTRA_ARGS+=("$arg") ;;
    esac
  done

  if [ ! -d "$seq_dir" ]; then
    echo "Error: '$seq_dir' is not a directory." >&2
    return 1
  fi

  local bag_root="${seq_dir%/}/raw/ros2bag"
  if [ ! -d "$bag_root" ]; then
    echo "Error: '$bag_root' does not exist. Expected a sequence directory such as" >&2
    echo "       '.../data/sequences/2024-03-12-keble-college-01/', containing a" >&2
    echo "       'raw/ros2bag/' subdirectory." >&2
    return 1
  fi

  # Sort on the trailing "_<n>" numerically. `sort -t_ -k` would break on the
  # timestamps earlier in the names, so key on the suffix explicitly.
  local -a parts
  mapfile -t parts < <(
    find "$bag_root" -mindepth 1 -maxdepth 1 -type d -exec test -f '{}/metadata.yaml' \; -print |
      while read -r d; do
        n=${d##*_}
        [[ "$n" =~ ^[0-9]+$ ]] || n=0
        printf '%s\t%s\n' "$n" "$d"
      done | sort -n -k1,1 | cut -f2-
  )

  if [ "${#parts[@]}" -eq 0 ]; then
    echo "Error: no ros2bag directories (with a 'metadata.yaml') found under '$bag_root'." >&2
    return 1
  fi
  if [ "${#parts[@]}" -gt "$OXFORD_SPIRES_MAX_BAG_PARTS" ]; then
    echo "Error: sequence has ${#parts[@]} ros2bag parts, but the launch file exposes" >&2
    echo "       $OXFORD_SPIRES_MAX_BAG_PARTS OXFORD_SPIRES_BAG_N slots." >&2
    return 1
  fi

  echo "Sequence '$(basename "${seq_dir%/}")': found ${#parts[@]} ros2bag part(s):"
  local p
  for p in "${parts[@]}"; do
    echo "  - $p"
  done

  # Set every slot, not just the ones we have parts for: otherwise a
  # OXFORD_SPIRES_BAG_N the user left exported from a previous run would leak
  # through and get appended as a stale extra part.
  local i
  for ((i = 0; i < OXFORD_SPIRES_MAX_BAG_PARTS; i++)); do
    printf -v "OXFORD_SPIRES_BAG_$i" '%s' "${parts[$i]:-}"
    export "OXFORD_SPIRES_BAG_$i"
  done

  : "${MOLA_LIDAR_TOPIC:=/hesai/pandar}"
  : "${MOLA_IMU_TOPIC:=/alphasense_driver_ros/imu}"
  : "${MOLA_CAMERA_TOPIC:=/alphasense_driver_ros/cam0/debayered/image/compressed}"
  : "${MOLA_TF_BASE_LINK:=base_link}"
  export MOLA_LIDAR_TOPIC MOLA_IMU_TOPIC MOLA_CAMERA_TOPIC MOLA_TF_BASE_LINK

  : "${LIDAR_POSE_X:=0}" ; : "${LIDAR_POSE_Y:=0}" ; : "${LIDAR_POSE_Z:=0.124}"
  : "${LIDAR_POSE_YAW:=180}" ; : "${LIDAR_POSE_PITCH:=0}" ; : "${LIDAR_POSE_ROLL:=0}"
  : "${MOLA_USE_FIXED_LIDAR_POSE:=true}"
  : "${IMU_POSE_X:=-0.018771}"
  : "${IMU_POSE_Y:=0.008218}"
  : "${IMU_POSE_Z:=0.053526}"
  : "${IMU_POSE_YAW:=89.3737}"
  : "${IMU_POSE_PITCH:=-0.1665}"
  : "${IMU_POSE_ROLL:=-0.1287}"
  : "${MOLA_USE_FIXED_IMU_POSE:=true}"
  export LIDAR_POSE_X LIDAR_POSE_Y LIDAR_POSE_Z
  export LIDAR_POSE_YAW LIDAR_POSE_PITCH LIDAR_POSE_ROLL MOLA_USE_FIXED_LIDAR_POSE
  export IMU_POSE_X IMU_POSE_Y IMU_POSE_Z
  export IMU_POSE_YAW IMU_POSE_PITCH IMU_POSE_ROLL MOLA_USE_FIXED_IMU_POSE

  # Handheld/backpack platform: lean on the IMU for the initial pitch/roll
  # estimate and for deskewing, same as the ConSLAM wrapper. The Alphasense
  # IMU does not publish an onboard-fused orientation (its Imu messages carry
  # an all-zero quaternion), so the pitch/roll initializer automatically falls
  # back to leveling from raw accelerometer readings.
  : "${MOLA_LO_INITIAL_LOCALIZATION_METHOD:=InitLocalization::PitchAndRollFromIMU}"
  : "${MOLA_DESKEW_METHOD:=MotionCompensationMethod::IMU}"
  # Keep the accelerometer in the deskew. This used to be suppressed, on
  # evidence that gyro-only deskew cut path-length error and improved APE.
  # Re-measured on all 13 sequences against today's pipeline, both halves of
  # that are gone: est/gt is identical to three decimals either way, and
  # suppressing the accelerometer now COSTS about 11% of APE (median 0.886x
  # in its favour, better on 9 of 13). The defect it compensated for has since
  # been fixed elsewhere, so the override no longer earns its keep. Left
  # explicit rather than deleted so the measurement is not lost.
  : "${MOLA_DESKEW_IGNORE_ACCELEROMETER:=false}"
  export MOLA_LO_INITIAL_LOCALIZATION_METHOD MOLA_DESKEW_METHOD MOLA_DESKEW_IGNORE_ACCELEROMETER

  # GICP-pipeline decimation tuning -- per-dataset, not a shipped pipeline
  # default. See the KITTI profile for the shared rationale/evidence; this
  # dataset independently confirmed the same configuration (56% cut in
  # pooled vertical drift at a third of the cost, 12-13/13 sequences).
  : "${MOLA_CLOUD_DECIMATION_VOXEL_SIZE_MAP:=0.45}"
  : "${MOLA_CLOUD_DECIMATION_VOXEL_SIZE_ICP:=0.45}"
  : "${MOLA_VOXEL_STRIDE_MAP:=1}"
  : "${MOLA_VOXEL_STRIDE_ICP:=2}"
  : "${MOLA_LOCALMAP_K_CORRESPONDENCES_FOR_COV:=10}"
  export MOLA_CLOUD_DECIMATION_VOXEL_SIZE_MAP MOLA_CLOUD_DECIMATION_VOXEL_SIZE_ICP
  export MOLA_VOXEL_STRIDE_MAP MOLA_VOXEL_STRIDE_ICP MOLA_LOCALMAP_K_CORRESPONDENCES_FOR_COV

  # No GNSS in this dataset: pin the first pose to the map origin, or the
  # smoother's GTSAM graph is left with a rank-deficient null-space
  # (IndeterminantLinearSystemException):
  : "${MOLA_LINK_FIRST_POSE_SIGMA:=1e-6}"
  export MOLA_LINK_FIRST_POSE_SIGMA

  mola_lo_use_smoother || return 1

  local IFS=,
  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_oxford_spires.yaml
  MOLA_LO_CLI_INPUT=(--input-rosbag2 "${parts[*]}")
}
