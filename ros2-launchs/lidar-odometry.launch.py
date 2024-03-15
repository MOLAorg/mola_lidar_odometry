
# ROS 2 launch file

from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    myDir = get_package_share_directory("mola_lidar_odometry")

    # args that can be set from the command line or a default will be used
    # Mandatory
    lidar_topic_name_arg = DeclareLaunchArgument(
        "lidar_topic_name", description="Topic name to listen for PointCloud2 input from the LiDAR (for example '/ouster/points')")

    topic_env_var = SetEnvironmentVariable(
        name='MOLA_LIDAR_NAME', value=LaunchConfiguration('lidar_topic_name'))

    ignore_lidar_pose_from_tf_arg = DeclareLaunchArgument(
        "ignore_lidar_pose_from_tf", default_value="false", description="If true, the LiDAR pose will be assumed to be at the origin (base_link). Set to false (default) if you want to read the actual sensor pose from /tf")

    fixed_sensorpose_env_var = SetEnvironmentVariable(
        name='MOLA_USE_FIXED_LIDAR_POSE', value=LaunchConfiguration('ignore_lidar_pose_from_tf'))

    mola_cli_node = Node(
        package='mola_launcher',
        executable='mola-cli',
        output='screen',
        arguments=[
                os.path.join(myDir, 'mola-cli-launchs', 'lidar_odometry_ros2.yaml')]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
                '-d', [os.path.join(myDir, 'rviz2', 'lidar-odometry.rviz')]]
    )

    return LaunchDescription([
        lidar_topic_name_arg,
        topic_env_var,
        ignore_lidar_pose_from_tf_arg,
        fixed_sensorpose_env_var,
        mola_cli_node,
        rviz2_node
    ])
