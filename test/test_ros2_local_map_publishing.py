# -----------------------------------------------------------------------------
# Integration test: the ROS 2 launch of MOLA-LO publishes the local map as a
# point cloud when `publish_local_map` is enabled, and does not when disabled.
# A rosbag2 with LiDAR scans is played into the regular launch file (headless).
# -----------------------------------------------------------------------------

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import launch
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_testing
import launch_testing.actions
import launch_testing.util
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2

# The ROS 2 bridge appends "_points" to point cloud topics:
LOCAL_MAP_TOPIC = '/lidar_odometry/localmap_points'
POSE_TOPIC = '/lidar_odometry/pose'
LIDAR_TOPIC = '/rslidar_points'


def _skip_reason():
    """The test launches the installed package, which a Debian package build does not have
    yet when it runs the tests: skipped there, run under `colcon test`."""
    try:
        get_package_share_directory('mola_lidar_odometry')
    except PackageNotFoundError:
        return 'mola_lidar_odometry is not installed yet (e.g. a Debian package build)'
    return None


SKIP_REASON = _skip_reason()

# Generous, for slow CI machines: startup, plus a few bag replays.
MAX_WAIT_SECONDS = 90.0
# With publishing disabled, how long to keep listening once LO is running:
QUIET_PERIOD_SECONDS = 10.0


@pytest.mark.launch_test
@launch_testing.parametrize('publish_local_map', ['True', 'False'])
def generate_test_description(publish_local_map):
    if SKIP_REASON:
        # nothing to launch: keep the launch alive until the (skipped) tests have run
        return launch.LaunchDescription([
            DeclareLaunchArgument('rosbag', default_value=''),
            launch_testing.util.KeepAliveProc(),
            launch_testing.actions.ReadyToTest(),
        ])
    lo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mola_lidar_odometry'),
            'ros2-launchs', 'ros2-lidar-odometry.launch.py')),
        launch_arguments={
            'lidar_topic_name': LIDAR_TOPIC,
            'ignore_lidar_pose_from_tf': 'True',
            'use_rviz': 'False',
            'use_mola_gui': 'False',
            'use_diagnostic_aggregator': 'False',
            'publish_local_map': publish_local_map,
        }.items())

    # Replays the short bag in a loop, so data keeps flowing for the whole test:
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '--loop', LaunchConfiguration('rosbag')],
        output='screen')

    return launch.LaunchDescription([
        DeclareLaunchArgument('rosbag', description='rosbag2 file with LiDAR scans'),
        # Publish on every chance, not only every N scans, to keep the test short:
        SetEnvironmentVariable('MOLA_PUBLISH_LOCAL_MAP_UPDATES_EVERY_N', '1'),
        lo_launch,
        # Give MOLA time to start and subscribe before the data arrives:
        TimerAction(period=5.0, actions=[bag_play]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestLocalMapPublishing(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_local_map_publishing')
        self.local_map_points = []
        self.poses_received = 0
        self.node.create_subscription(
            PointCloud2, LOCAL_MAP_TOPIC,
            lambda msg: self.local_map_points.append(msg.width * msg.height),
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(
            Odometry, POSE_TOPIC, self._on_pose, qos_profile_sensor_data)

    def tearDown(self):
        self.node.destroy_node()

    def _on_pose(self, _msg):
        self.poses_received += 1

    def _spin_until(self, condition, timeout):
        end = time.time() + timeout
        while time.time() < end and not condition():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return condition()

    def test_local_map_topic(self, publish_local_map):
        if SKIP_REASON:
            self.skipTest(SKIP_REASON)
        # LO must be running and estimating poses in both cases:
        self.assertTrue(
            self._spin_until(lambda: self.poses_received > 0, MAX_WAIT_SECONDS),
            f'No odometry received on {POSE_TOPIC}')

        if publish_local_map == 'True':
            self.assertTrue(
                self._spin_until(
                    lambda: any(n > 0 for n in self.local_map_points), MAX_WAIT_SECONDS),
                f'No non-empty point cloud received on {LOCAL_MAP_TOPIC}')
        else:
            self._spin_until(lambda: False, QUIET_PERIOD_SECONDS)
            self.assertGreater(self.poses_received, 1)
            self.assertEqual(
                self.local_map_points, [],
                f'Local map published on {LOCAL_MAP_TOPIC} despite publish_local_map=False')

