#!/usr/bin/env bash
# Personal shortcut: one hard-coded local recording, replayed through the
# generic rawlog wrapper. Nothing dataset-specific lives here.
MOLA_TIME_WARP=4.0 \
MOLA_LIDAR_NAME=Velodyne1_SCAN \
exec mola-lo-gui-rawlog \
  /mnt/storage/ual-datasets/ecarm_2018_02_26_velodyne_rtk_campus/dataset_2018-02-26_merged.rawlog \
  "$@"
