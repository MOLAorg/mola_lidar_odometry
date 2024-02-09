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
Refer to the [root MOLA repository](https://github.com/MOLAorg/mola).

## Docs and examples
See this package page [in the documentation](https://docs.mola-slam.org/latest/modules.html).

## License
Copyright (C) 2018-2023 Jose Luis Blanco <jlblanco@ual.es>, University of Almeria

This package is released under the GNU GPL v3 license as open source for research
and evaluation purposes only. Commercial licenses available upon request, for this
odometry package alone or in combination with the complete SLAM system.
