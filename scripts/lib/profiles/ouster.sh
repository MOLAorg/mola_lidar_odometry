# Ouster sensors via mola_input_ouster (no ROS required): a live sensor, or a
# PCAP/OSF recording. Not a published dataset, so there is nothing to resolve
# on disk; the input is selected by environment variable, or an .osf recording
# is given as the first argument. There is no offline-CLI counterpart
# (mola_input_ouster is not an OfflineDatasetSource).
#
# OUSTER_VARIANT=rev8 (what mola-lo-gui-ouster-rev8 sets) adds the LIO defaults
# for Rev8 high-resolution sensors, see mola_lo_ouster_rev8_defaults().

MOLA_LO_PROFILE_NO_ARGS=1

mola_lo_profile_usage() {
  echo "Error: no input source specified."
  echo ""
  echo "Pass an .osf recording as the first argument, or set one of:"
  echo "  OUSTER_HOSTNAME=<hostname>              for a live sensor"
  echo "  OUSTER_PCAP=<file> OUSTER_META=<json>   for PCAP replay"
  echo "  OUSTER_OSF=<file>                       for OSF replay"
  echo ""
  echo "Common optional variables:"
  echo "  MOLA_LIDAR_NAME    Sensor label (default: lidar)"
  echo "  MOLA_IMU_NAME      IMU label (default: imu)"
  echo "  MOLA_TIME_WARP     Replay speed multiplier (default: 1.0) [PCAP/OSF]"
  echo "  OUSTER_LIDAR_MODE  Resolution/rate (default: MODE_1024x10) [live]"
  echo "  OUSTER_DECIMATE_COLUMNS / OUSTER_DECIMATE_ROWS"
  echo "                     Keep every N-th column / row of each scan (default: 1)"
  echo "  SENSOR_POSE_{X,Y,Z,YAW,PITCH,ROLL}  Mounting pose on vehicle (default: 0)"
}

# Rev8 sensors deliver far more than LiDAR odometry needs: 4096x256 at 5 Hz
# (~290k valid points per scan) plus a 2560 Hz IMU. Measured on such a sensor,
# replayed in real time with the GUI on a 16-core desktop:
#   - OUSTER_DECIMATE_COLUMNS=4 (4096 -> 1024 columns): full scans keep the GUI
#     at ~35 fps with 56 ms stalls, and LO at 165 ms/scan (250 ms max) for a
#     200 ms period, dropping ~3% of scans. Decimated, the GUI holds 60 fps,
#     LO takes ~60 ms/scan (100 ms max) and drops nothing, while the trajectory
#     stays within run-to-run noise of a lossless full-resolution run. For a
#     2048-column mode, use 2 to get the same 1024 columns.
#   - IMU de-skew and initial pitch/roll from the IMU: this is a LIO launcher.
#   - mola::IncrementalPointCloud local map: on fast, long-range motion (a
#     drone flight) it rejected ~32% of scans versus ~87% for the keyframe map.
#   - GUI colors from the per-point RGB of "-RGB" models, for the live clouds,
#     the local map and the sensor preview. Set OUSTER_GUI_COLOR_BY_RGB=false
#     for sensors without RGB, to get the intensity colormaps back.
mola_lo_ouster_rev8_defaults() {
  : "${OUSTER_DECIMATE_COLUMNS:=4}"
  : "${MOLA_DESKEW_METHOD:=MotionCompensationMethod::IMU}"
  : "${MOLA_LO_INITIAL_LOCALIZATION_METHOD:=InitLocalization::PitchAndRollFromIMU}"
  : "${MOLA_LOCALMAP_CLASS:=mola::IncrementalPointCloud}"
  export OUSTER_DECIMATE_COLUMNS MOLA_DESKEW_METHOD MOLA_LO_INITIAL_LOCALIZATION_METHOD \
    MOLA_LOCALMAP_CLASS

  if [ "${OUSTER_GUI_COLOR_BY_RGB:-true}" = true ]; then
    : "${MOLA_GUI_LAST_CLOUDS_COLOR_FIELD:=rgb}"
    : "${MOLA_GUI_CURRENT_CLOUD_COLOR_FIELD:=rgb}"
    : "${MOLA_GUI_LOCAL_MAP_COLOR_BY_COORDINATE:=rgb}"
    : "${MOLA_GUI_PREVIEW_COLOR_FROM_Z:=false}"
    export MOLA_GUI_LAST_CLOUDS_COLOR_FIELD MOLA_GUI_CURRENT_CLOUD_COLOR_FIELD \
      MOLA_GUI_LOCAL_MAP_COLOR_BY_COORDINATE MOLA_GUI_PREVIEW_COLOR_FROM_Z
  fi
}

mola_lo_profile_resolve() {
  if [ "$#" -gt 0 ] && [[ "$1" == *.osf ]]; then
    if [ ! -f "$1" ]; then
      echo "Error: OSF file not found: '$1'" >&2
      return 1
    fi
    export OUSTER_OSF="$1"
    shift
  fi
  MOLA_LO_EXTRA_ARGS=("$@")

  if [ -z "${OUSTER_HOSTNAME:-}" ] && [ -z "${OUSTER_PCAP:-}" ] && [ -z "${OUSTER_OSF:-}" ]; then
    mola_lo_profile_usage
    return 1
  fi

  case "${OUSTER_VARIANT:-}" in
    "") ;;
    rev8) mola_lo_ouster_rev8_defaults ;;
    *)
      echo "Error: unknown OUSTER_VARIANT='${OUSTER_VARIANT}' (known: rev8)" >&2
      return 1
      ;;
  esac

  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_ouster.yaml
}
