# mola_lidar_odometry
LIDAR odometry component based on the MOLA and MRPT frameworks,
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

## ROS build farm status

| Distro | Develop branch | Releases | Stable release |
| ---    | ---            | ---      |  ---      |
| ROS2 Humble  (u22.04) |  [![Build Status](https://build.ros2.org/job/Hdev__mola_lidar_odometry__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mola_lidar_odometry__ubuntu_jammy_amd64/) | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_lidar_odometry__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_lidar_odometry__ubuntu_jammy_amd64__binary/)  | [![Version](https://img.shields.io/ros/v/humble/mola_lidar_odometry)](https://index.ros.org/search/?term=mola_lidar_odometry) |
| ROS2 Iron  (u22.04)   |  [![Build Status](https://build.ros2.org/job/Idev__mola_lidar_odometry__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Idev__mola_lidar_odometry__ubuntu_jammy_amd64/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mola_lidar_odometry__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mola_lidar_odometry__ubuntu_jammy_amd64__binary/)  | [![Version](https://img.shields.io/ros/v/iron/mola_lidar_odometry)](https://index.ros.org/search/?term=mola_lidar_odometry) |
| ROS 2 Jazzy @ u24.04 | [![Build Status](https://build.ros2.org/job/Jdev__mola_lidar_odometry__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mola_lidar_odometry__ubuntu_noble_amd64/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mola_lidar_odometry__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mola_lidar_odometry__ubuntu_noble_amd64__binary/) | [![Version](https://img.shields.io/ros/v/jazzy/mola_lidar_odometry)](https://index.ros.org/search/?term=mola_lidar_odometry) | 
| ROS 2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__mola_lidar_odometry__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mola_lidar_odometry__ubuntu_noble_amd64/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mola_lidar_odometry__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mola_lidar_odometry__ubuntu_noble_amd64__binary/) | [![Version](https://img.shields.io/ros/v/rolling/mola_lidar_odometry)](https://index.ros.org/search/?term=mola_lidar_odometry) |


## License
Copyright (C) 2018-2024 Jose Luis Blanco <jlblanco@ual.es>, University of Almeria

This package is released under the GNU GPL v3 license as open source, with the main 
intention of being useful for research and evaluation purposes.
Commercial licenses [available upon request](https://docs.mola-slam.org/latest/solutions.html).

