# `mola_lidar_odometry`
LIDAR odometry components, based on the MOLA and MRPT frameworks,
compatible with ROS 2.

## Contents
This repository provides a C++ library `mola_lidar_odometry` implementing LIDAR
odometry. Sensor input is provided via MOLA components, and ROS 2 example launch files are
provided in [launch](launch/).

A CLI interface `mola-lidar-odometry-cli` is also provided for running on
offline datasets.

## Build and install
Refer to: https://docs.mola-slam.org/latest/#installing

![mola-slam-kitti-demo](https://github.com/user-attachments/assets/45255aba-6ea2-44eb-b5e4-4cc52e8e7615)


## Documentation and tutorials
See: https://docs.mola-slam.org/

## License
Copyright (C) 2018-2024 Jose Luis Blanco <jlblanco@ual.es>, University of Almeria

This package is released under the GNU GPL v3 license as open source, with the main 
intention of being useful for research and evaluation purposes.
Commercial licenses [available upon request](https://docs.mola-slam.org/latest/solutions.html).
