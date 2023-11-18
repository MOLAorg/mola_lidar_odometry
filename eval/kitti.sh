#!/bin/bash

parallel -j3 --lb \
  KITTI_SEQ={} mola-lidar-odometry-cli \
    -c src/mola_lidar_odometry/params/config-lidar-inertial-odometry.yaml\
    -l install/mola_metric_maps/lib/libmola_metric_maps.so\
    --input-kitti-seq {} \
    --output-tum-path results/estim_{}.txt \
::: 00 01 02 03 04 05 06 07 08 09 10

