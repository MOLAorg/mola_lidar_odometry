#!/usr/bin/env python3
# Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria, and individual contributors.
# SPDX-License-Identifier: GPL-3.0
"""
The ROS 2 launch file picks the odometry source for the vehicle motion during
an initial global search: its external odometry, if any, unless the user set
MOLA_LO_INIT_GLOBAL_SEARCH_MOTION_ODOMETRY. Never the state estimator.
Usage: test_launch_global_search_motion.py <ros2-lidar-odometry.launch.py>
"""
import importlib.util
import os
import sys

from launch import LaunchContext
from launch.utilities import perform_substitutions

ENV = 'MOLA_LO_INIT_GLOBAL_SEARCH_MOTION_ODOMETRY'

spec = importlib.util.spec_from_file_location('lo_launch', sys.argv[1])
lo_launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lo_launch)


def resolve(odom_topic='', label='odom_wheels', tf_path='False'):
    ctx = LaunchContext()
    ctx.launch_configurations.update({
        'odom_topic_name': odom_topic,
        'odom_sensor_label': label,
        'forward_ros_tf_odom_to_mola': tf_path,
    })
    actions = lo_launch.resolve_global_search_motion_odometry(ctx)
    values = {perform_substitutions(ctx, a.name): perform_substitutions(ctx, a.value)
              for a in actions}
    return values.get(ENV)


def check(got, expected, what):
    if got != expected:
        print(f'FAIL: {what}: got {got!r}, expected {expected!r}')
        sys.exit(1)
    print(f'OK: {what}: {got!r}')


# Registered in the launch description, after the arguments it reads. Only
# its entities are inspected, so the package need not be installed (e.g. tests
# of a Debian package build):
lo_launch.get_package_share_directory = lambda _pkg: ''
ld = lo_launch.generate_launch_description()
names = []
for e in ld.entities:
    if type(e).__name__ == 'DeclareLaunchArgument':
        names.append(e.name)
    elif getattr(e, '_OpaqueFunction__function', None) is \
            lo_launch.resolve_global_search_motion_odometry:
        names.append('<resolve>')
    elif getattr(getattr(e, '_OpaqueFunction__function', None), '__name__', '') == \
            'make_node_group':
        names.append('<nodes>')
if '<resolve>' not in names:
    print('FAIL: resolve_global_search_motion_odometry is not in the launch description')
    sys.exit(1)
for arg in ('odom_topic_name', 'odom_sensor_label', 'forward_ros_tf_odom_to_mola'):
    if arg not in names or names.index(arg) > names.index('<resolve>'):
        print(f'FAIL: argument {arg} is not declared before it is resolved')
        sys.exit(1)
if '<nodes>' not in names or names.index('<nodes>') < names.index('<resolve>'):
    print('FAIL: the choice must be made before the nodes are launched')
    sys.exit(1)
print('OK: registered after its arguments and before the nodes')

os.environ.pop(ENV, None)
check(resolve(), None, 'no external odometry: vehicle assumed still')
check(resolve(odom_topic='/odom'), 'odom_wheels', 'odom_topic_name, default label')
check(resolve(odom_topic='/odom', label='wheels2'), 'wheels2', 'odom_topic_name, own label')
check(resolve(tf_path='True'), 'odom', 'forward_ros_tf_odom_to_mola')
os.environ[ENV] = 'my_odom'
check(resolve(odom_topic='/odom'), None, 'an explicit setting is kept')
