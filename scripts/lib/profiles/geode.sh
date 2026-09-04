# GEODE (https://github.com/PengYu-Team/GEODE_dataset): a heterogeneous-LiDAR
# dataset built specifically around DEGENERATE geometry -- flat surfaces,
# stairwells, metro tunnels (shield and tunneling), off-road, inland
# waterways, urban tunnels and bridges.
#
# Three acquisition devices, and which one a bag came from decides its topics:
#
#   alpha   Velodyne VLP-16 (spinning)      /velodyne_points     PointCloud2
#   beta    Ouster OS1-64 (spinning)        /ouster/points       PointCloud2
#   gamma   Livox AVIA (non-repetitive)     /livox/lidar         CustomMsg
#
# Topic names here were read out of the bags themselves, not from upstream's
# cali/*.yaml. Those config files cannot be trusted for this: all three name
# the same "/os1_cloud_node1/points" and "/imu/imu", all three declare
# VERT_RES 16 (beta is a 64-beam Ouster), and image_topic is also "/imu/imu".
# They are visibly copy-pasted, and not one of those three lidar topics is
# what the bags actually contain -- beta's is "/ouster/points".
#
# The external IMU is on /imu/data at ~100 Hz on all three rigs. The beta and
# gamma rigs also record their lidar's built-in IMU (/ouster/imu, /livox/imu);
# neither is used by default, matching botanicgarden.sh.
#
# Sequence naming: most sequences carry the device as a filename suffix
# (Offroad1_beta, Tunneling_tunnel2_alpha, ...). The single-device scenarios
# do not, so they are listed explicitly below -- bridge* and Urban_Tunnel*
# are alpha rigs, flat_surfaces_* is a gamma rig. Both were confirmed by
# reading the bags. GEODE_DEVICE overrides the detection.
#
# KNOWN, as of 2026-08-23: with the stock lidar3d-default pipeline this
# profile gets 1.4% drift through a metro tunnel (Tunneling_tunnel2_alpha)
# and 4.6% on the flat-surfaces AVIA sequences, but it DIVERGES on
# bridge01 -- the estimate travels 41% of the 3.97 km reference path and
# drifts 552 m vertically. Wrong topics, a flipped IMU, missing per-point
# timestamps, IMU deskewing and a rotating lever arm have all been ruled out
# by experiment, so this looks like pipeline tuning rather than a mistake in
# the description below. Upstream's own usage notes say to "adapt your SLAM
# algorithm using the provided dataset parameters".
#
# GROUND TRUTH, which is where this dataset will bite you: several scenarios
# were recorded with all three devices bolted to one rack and a SINGLE ground
# truth device, and which device the reference is expressed in changes per
# scenario. GNSS/INS scenarios (off-road, inland waterways) are in the BETA
# frame; the Leica-prism metro tunnels are in the ALPHA frame. So the device
# that needs no correction differs by scenario, and the rest need a half-metre
# body offset that no global trajectory alignment can absorb. That correction
# belongs to whoever scores the run, not to this profile -- see
# upstream's README, "Localization Evaluation".

mola_lo_profile_usage() {
  echo "Error: A GEODE ROS 1 '.bag' file is required."
  echo "Usage: $0 /path/to/<sequence>.bag [additional flags]"
  echo ""
  echo "Optional environment variables:"
  echo "  GEODE_DEVICE   'alpha' | 'beta' | 'gamma'. Normally detected from"
  echo "                 the file name; set this for a renamed bag."
}

mola_lo_profile_resolve() {
  local file=$1
  shift
  MOLA_LO_EXTRA_ARGS=("$@")

  if [ -z "$file" ]; then
    mola_lo_profile_usage
    return 1
  fi

  local base
  base="$(basename "$file")"
  if [ -z "${GEODE_DEVICE:-}" ]; then
    case "$base" in
      *_alpha*|bridge*|Urban_Tunnel*|urban_tunnel*) GEODE_DEVICE=alpha ;;
      *_beta*|*_Beta*)                              GEODE_DEVICE=beta ;;
      *_gamma*|*_Gamma*|flat_surfaces_*)            GEODE_DEVICE=gamma ;;
      *)
        echo "Error: cannot tell which GEODE device '$base' came from." >&2
        echo "       Set GEODE_DEVICE to alpha, beta or gamma." >&2
        return 1
        ;;
    esac
  fi

  # Lidar topic and the sensor's pose in the IMU frame, per device. The poses
  # are T_IMU_LiDAR from upstream's cali/<device>_config.yaml, decomposed to
  # this repo's degrees convention. Unlike the topic names in those same
  # files, the extrinsic matrices are per-device and self-consistent.
  case "$GEODE_DEVICE" in
    alpha)
      : "${MOLA_LIDAR_TOPIC:=/velodyne_points}"
      : "${LIDAR_POSE_X:=0.09610}"
      : "${LIDAR_POSE_Y:=-0.13380}"
      : "${LIDAR_POSE_Z:=0.30320}"
      : "${LIDAR_POSE_YAW:=-0.2865}"
      : "${LIDAR_POSE_PITCH:=0.8767}"
      : "${LIDAR_POSE_ROLL:=-0.1261}"
      ;;
    beta)
      : "${MOLA_LIDAR_TOPIC:=/ouster/points}"
      : "${LIDAR_POSE_X:=-0.02717}"
      : "${LIDAR_POSE_Y:=-0.03487}"
      : "${LIDAR_POSE_Z:=0.06264}"
      : "${LIDAR_POSE_YAW:=-2.9771}"
      : "${LIDAR_POSE_PITCH:=-0.2861}"
      : "${LIDAR_POSE_ROLL:=-0.7826}"
      ;;
    gamma)
      : "${MOLA_LIDAR_TOPIC:=/livox/lidar}"
      : "${LIDAR_POSE_X:=0.04926}"
      : "${LIDAR_POSE_Y:=-0.01250}"
      : "${LIDAR_POSE_Z:=0.02695}"
      : "${LIDAR_POSE_YAW:=-1.5768}"
      : "${LIDAR_POSE_PITCH:=0.1000}"
      : "${LIDAR_POSE_ROLL:=-1.4587}"
      # Non-repetitive scan pattern: accumulate more than one scan per
      # location before moving on, so the local map and the simplemap are
      # dense enough to register against. Same treatment as the Livox arm of
      # botanicgarden.sh.
      : "${MOLA_MIN_NEARBY_POSES_OCCUPIED:=2}"
      : "${MOLA_SIMPLEMAP_MIN_NEARBY_POSES:=2}"
      export MOLA_MIN_NEARBY_POSES_OCCUPIED MOLA_SIMPLEMAP_MIN_NEARBY_POSES
      ;;
    *)
      echo "Error: GEODE_DEVICE must be alpha, beta or gamma, got '$GEODE_DEVICE'." >&2
      return 1
      ;;
  esac
  export GEODE_DEVICE MOLA_LIDAR_TOPIC

  : "${MOLA_IMU_TOPIC:=/imu/data}"
  export MOLA_IMU_TOPIC

  # base_link is the IMU, so the lidar carries the offset above. There is no
  # /tf or /tf_static in these bags, so every pose has to be fixed here.
  : "${MOLA_TF_BASE_LINK:=imu}"
  : "${MOLA_USE_FIXED_LIDAR_POSE:=1}"
  : "${MOLA_USE_FIXED_IMU_POSE:=1}"
  : "${IMU_POSE_X:=0}" ; : "${IMU_POSE_Y:=0}" ; : "${IMU_POSE_Z:=0}"
  : "${IMU_POSE_YAW:=0}" ; : "${IMU_POSE_PITCH:=0}" ; : "${IMU_POSE_ROLL:=0}"
  export MOLA_TF_BASE_LINK MOLA_USE_FIXED_LIDAR_POSE MOLA_USE_FIXED_IMU_POSE
  export LIDAR_POSE_X LIDAR_POSE_Y LIDAR_POSE_Z
  export LIDAR_POSE_YAW LIDAR_POSE_PITCH LIDAR_POSE_ROLL
  export IMU_POSE_X IMU_POSE_Y IMU_POSE_Z
  export IMU_POSE_YAW IMU_POSE_PITCH IMU_POSE_ROLL

  # The stereo pair is a preview only; decoding it in a batch run costs time
  # and buys nothing. Note the topic name differs between rigs.
  if [ "$MOLA_LO_MODE" = "gui" ]; then
    # The topic name genuinely differs between rigs: alpha publishes
    # /left_camera/compressed, beta and gamma /left_camera/image/compressed.
    case "$GEODE_DEVICE" in
      alpha) : "${MOLA_CAMERA_TOPIC:=/left_camera/compressed}" ;;
      *)     : "${MOLA_CAMERA_TOPIC:=/left_camera/image/compressed}" ;;
    esac
    : "${MOLA_USE_FIXED_CAMERA_POSE:=1}"
    export MOLA_CAMERA_TOPIC MOLA_USE_FIXED_CAMERA_POSE
  fi

  # Degenerate geometry is the whole point of this dataset, so lean on the
  # IMU: in a tunnel or on a flat surface the point-to-plane terms stop
  # constraining along-track motion, and the motion model is what carries the
  # estimate through.
  : "${MOLA_LO_INITIAL_LOCALIZATION_METHOD:=InitLocalization::PitchAndRollFromIMU}"
  : "${MOLA_DESKEW_METHOD:=MotionCompensationMethod::IMU}"
  export MOLA_LO_INITIAL_LOCALIZATION_METHOD MOLA_DESKEW_METHOD

  # No GNSS is fed to the pipeline (it is the reference, not an input), so pin
  # the first pose or the smoother's graph is rank-deficient. Same as
  # conslam, hilti2022 and botanicgarden.
  : "${MOLA_LINK_FIRST_POSE_SIGMA:=1e-6}"
  export MOLA_LINK_FIRST_POSE_SIGMA

  mola_lo_use_smoother || return 1

  mola_lo_bag_slots "$file" || return 1
  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_rosbag1.yaml
  MOLA_LO_CLI_INPUT=(--input-rosbag1 "$MOLA_LO_BAGS_JOINED")
}
