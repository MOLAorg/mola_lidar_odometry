# -----------------------------------------------------------------------------
# Integration test: initial localization of the ROS 2 launch of MOLA-LO, with
# the smoother as state estimator, on a real dataset fragment with a prebuilt
# map and ground truth (mola_test_datasets, ual/husky_benches). Each test case
# is one test_*.yaml of the dataset directory: the initial localization method
# (a manual pose, or the state estimator pose from GNSS and an attitude IMU,
# with or without the global search), and the expected outcome: localized
# (near the ground truth) or not (far from it).
# -----------------------------------------------------------------------------

import bisect
import glob
import math
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import launch
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.util
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
import yaml

# With the state estimator, the vehicle pose is its output:
POSE_TOPIC = '/state_estimation/pose'
DATASET_DIR = os.environ.get('LO_TEST_INITIAL_LOCALIZATION_DIR', '')
# Slower than real time, so slow CI machines keep up. A test case may set its
# own `bag_rate`, e.g. to keep the vehicle still during a slow global search:
BAG_RATE = 0.5
# Besides the whole fragment, at its playback rate:
STARTUP_SECONDS = 60.0


def _skip_reason():
    """The test launches the installed package, which a Debian package build does not have
    yet when it runs the tests: skipped there, run under `colcon test`."""
    try:
        get_package_share_directory('mola_lidar_odometry')
    except PackageNotFoundError:
        return 'mola_lidar_odometry is not installed yet (e.g. a Debian package build)'
    if not os.path.isfile(os.path.join(DATASET_DIR, 'dataset.yaml')):
        return f'No dataset.yaml in LO_TEST_INITIAL_LOCALIZATION_DIR="{DATASET_DIR}"'
    return None


SKIP_REASON = _skip_reason()
# SetEnvironmentVariable changes the environment of this process, so each test
# case starts from this one, not from what the previous case left:
INITIAL_ENVIRONMENT = dict(os.environ)
# All test_*.yaml of the dataset, or those in LO_TEST_INITIAL_LOCALIZATION_CASES (comma separated):
TEST_CASES = [c for c in sorted(
    os.path.basename(f) for f in glob.glob(os.path.join(DATASET_DIR, 'test_*.yaml')))
    if c in os.environ.get('LO_TEST_INITIAL_LOCALIZATION_CASES', c).split(',')] or ['none']


def _load_yaml(name):
    with open(os.path.join(DATASET_DIR, name)) as f:
        return yaml.safe_load(f)


def _load_case(case):
    """The test case and its dataset description."""
    c = _load_yaml(case)
    return c, _load_yaml(c['dataset'])


def _launch_arguments(case, dataset):
    """Launch arguments of ros2-lidar-odometry.launch.py and environment variables."""
    topics = dataset['topics']
    lo = dataset['lidar_odometry']
    il = case['initial_localization']
    args = {
        'lidar_topic_name': topics['lidar'],
        'imu_topic_name': topics['lidar_imu'],
        'odom_topic_name': topics['wheel_odometry'],
        'mola_lo_pipeline': os.path.join(
            get_package_share_directory('mola_lidar_odometry'), 'pipelines', lo['pipeline']),
        'use_imu_for_lio': str(lo['use_imu_for_lio']),
        'use_state_estimator': str(lo['use_state_estimator']),
        'start_mapping_enabled': str(lo['start_mapping_enabled']),
        'mola_initial_map_mm_file': os.path.join(DATASET_DIR, dataset['map']),
        'mola_tf_base_link': dataset['frames']['base_link'],
        'initial_localization_method': il['method'],
        'use_sim_time': 'true',
        'use_rviz': 'False',
        'use_mola_gui': 'False',
        'use_diagnostic_aggregator': 'False',
        'publish_local_map': 'False',
        # The fragment has no odom -> base_link: publish map -> base_link
        'publish_localization_following_rep105': 'False',
    }
    if 'pose' in il:
        args['initial_pose'] = str(il['pose'])
    if 'gnss_mode' in il:
        args['gnss_mode'] = il['gnss_mode']
        args['gnss_topic_name'] = il['gnss_topic']
        args['attitude_imu_topic_name'] = il.get('attitude_imu_topic', '')
    env = dict(lo.get('env', {}))
    env.update(il.get('env', {}))
    return args, {k: str(v).lower() if isinstance(v, bool) else str(v) for k, v in env.items()}


@pytest.mark.launch_test
@launch_testing.parametrize('case', TEST_CASES)
def generate_test_description(case):
    if SKIP_REASON:
        # nothing to launch: keep the launch alive until the (skipped) tests have run
        return launch.LaunchDescription([
            launch_testing.util.KeepAliveProc(),
            launch_testing.actions.ReadyToTest(),
        ])
    os.environ.clear()
    os.environ.update(INITIAL_ENVIRONMENT)
    tc, dataset = _load_case(case)
    args, env = _launch_arguments(tc, dataset)
    lo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mola_lidar_odometry'),
            'ros2-launchs', 'ros2-lidar-odometry.launch.py')),
        launch_arguments=args.items())

    # The fragment topics keep the robot namespace, given in full to LO; only
    # its static TF must reach the global /tf_static:
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '--clock', '--rate', str(tc.get('bag_rate', BAG_RATE)),
             os.path.join(DATASET_DIR, dataset['fragment']),
             '--remap', f"{dataset['topics']['tf_static']}:=/tf_static"],
        output='screen')

    return launch.LaunchDescription(
        [SetEnvironmentVariable(k, v) for k, v in env.items()] + [
            lo_launch,
            # Give MOLA time to start, load the map and subscribe before the data arrives:
            TimerAction(period=10.0, actions=[bag_play]),
            launch_testing.actions.ReadyToTest(),
        ])


def _load_tum(path):
    """List of (t, x, y, z, yaw) from a TUM file."""
    out = []
    with open(path) as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            t, x, y, z, qx, qy, qz, qw = (float(v) for v in line.split())
            out.append((t, x, y, z, _yaw(qx, qy, qz, qw)))
    return out


def _yaw(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def _wrap_deg(a):
    return abs((math.degrees(a) + 180.0) % 360.0 - 180.0)


class TestInitialLocalization(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_initial_localization')
        self.poses = []
        self.node.create_subscription(
            Odometry, POSE_TOPIC, self._on_pose, qos_profile_sensor_data)

    def tearDown(self):
        self.node.destroy_node()

    def _on_pose(self, msg):
        p = msg.pose.pose
        o = p.orientation
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.poses.append((t, p.position.x, p.position.y, p.position.z,
                           _yaw(o.x, o.y, o.z, o.w)))

    def _spin_until(self, condition, timeout):
        end = time.time() + timeout
        while time.time() < end and not condition():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return condition()

    def test_initial_localization(self, case, proc_output):
        if SKIP_REASON:
            self.skipTest(SKIP_REASON)
        tc, dataset = _load_case(case)
        expected = tc['expected']
        gt = _load_tum(os.path.join(DATASET_DIR, dataset['ground_truth']))
        gt_stamps = [g[0] for g in gt]
        t_end = dataset['start_stamp'] + dataset['duration']
        t_from = t_end - expected['evaluate_last_seconds']

        # Until the end of the fragment:
        last_scan = gt_stamps[-1]
        self._spin_until(
            lambda: self.poses and self.poses[-1][0] >= last_scan - 0.3,
            STARTUP_SECONDS + dataset['duration'] / tc.get('bag_rate', BAG_RATE))

        # Errors of the poses of the evaluated period, against the nearest ground
        # truth pose (one per LiDAR scan; the robot is still in that period):
        errors = []
        for t, x, y, z, yaw in self.poses:
            if t < t_from:
                continue
            i = bisect.bisect_left(gt_stamps, t)
            j = min((k for k in (i - 1, i) if 0 <= k < len(gt)), key=lambda k: abs(gt[k][0] - t))
            if abs(gt[j][0] - t) > 0.3:
                continue
            g = gt[j]
            errors.append((math.hypot(x - g[1], y - g[2]), _wrap_deg(yaw - g[4])))
        if errors:
            print(f'[{case}] {len(self.poses)} poses received, {len(errors)} evaluated, '
                  f'position error {min(e for e, _ in errors):.2f} to '
                  f'{max(e for e, _ in errors):.2f} m, yaw error up to '
                  f'{max(a for _, a in errors):.1f} deg')

        if expected['localized']:
            self.assertTrue(errors, f'No pose on {POSE_TOPIC} over the last '
                            f"{expected['evaluate_last_seconds']} s of the fragment")
            for e, a in errors:
                self.assertLess(e, expected['max_position_error'])
                self.assertLess(a, expected['max_yaw_error_deg'])
            if 'log_contains' in expected:
                # MOLA logs INFO messages to stdout:
                proc_output.assertWaitFor(
                    expected['log_contains'], timeout=1.0, stream='stdout')
        else:
            # Not localized: a pose, but far from the truth.
            self.assertTrue(errors, f'No pose on {POSE_TOPIC} over the last '
                            f"{expected['evaluate_last_seconds']} s of the fragment")
            for e, _ in errors:
                self.assertGreater(e, expected['min_position_error'])
