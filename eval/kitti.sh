#!/bin/bash

parallel -j3 --lb \
  SEQ={} mola-lidar-odometry-cli \
    -c src/mola_lidar_odometry/params/config-lidar-inertial-odometry.yaml\
    -l install/mola_metric_maps/lib/libmola_metric_maps.so\
    --input-kitti-seq {} \
    --output-tum-path results/estim_{}.txt \
::: 00 01 02 03 04 05 06 07 08 09 10

# Eval kitti metrics for each sequence alone:
for d in 00 01 02 03 04 05 06 07 08 09 10 11; do
  kitti-metrics-eval -r estim_${d}.txt -s ${d} --no-figures
done

# Eval overall kitti metrics:
kitti-metrics-eval -r results/estim_%02i.txt -s 00 -s 01 -s 02 -s 03 -s 04 -s 05 -s 06 -s 07 -s 08 -s 09 -s 10 --no-figures
