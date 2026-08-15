# Ouster sensors via mola_input_ouster (no ROS required): a live sensor, or a
# PCAP/OSF recording. Not a published dataset, so there is nothing to resolve
# on disk; the input is selected by environment variable, and there is no
# offline-CLI counterpart (mola_input_ouster is not an OfflineDatasetSource).

MOLA_LO_PROFILE_NO_ARGS=1

mola_lo_profile_usage() {
  echo "Error: no input source specified."
  echo ""
  echo "Set one of:"
  echo "  OUSTER_HOSTNAME=<hostname>              for a live sensor"
  echo "  OUSTER_PCAP=<file> OUSTER_META=<json>   for PCAP replay"
  echo "  OUSTER_OSF=<file>                       for OSF replay"
  echo ""
  echo "Common optional variables:"
  echo "  MOLA_LIDAR_NAME    Sensor label (default: lidar)"
  echo "  MOLA_IMU_NAME      IMU label (default: imu)"
  echo "  MOLA_TIME_WARP     Replay speed multiplier (default: 1.0) [PCAP/OSF]"
  echo "  OUSTER_LIDAR_MODE  Resolution/rate (default: MODE_1024x10) [live]"
  echo "  SENSOR_POSE_{X,Y,Z,YAW,PITCH,ROLL}  Mounting pose on vehicle (default: 0)"
}

mola_lo_profile_resolve() {
  MOLA_LO_EXTRA_ARGS=("$@")

  if [ -z "${OUSTER_HOSTNAME:-}" ] && [ -z "${OUSTER_PCAP:-}" ] && [ -z "${OUSTER_OSF:-}" ]; then
    mola_lo_profile_usage
    return 1
  fi

  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_ouster.yaml
}
