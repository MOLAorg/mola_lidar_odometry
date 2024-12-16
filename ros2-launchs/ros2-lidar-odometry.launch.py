
# ROS 2 launch file

from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    myDir = get_package_share_directory("mola_lidar_odometry")

    # -------------------
    #     Arguments
    # -------------------
    # Mandatory
    lidar_topic_name_arg = DeclareLaunchArgument(
        "lidar_topic_name", description="Topic name to listen for PointCloud2 input from the LiDAR (for example '/ouster/points')")
    topic_env_var = SetEnvironmentVariable(
        name='MOLA_LIDAR_TOPIC', value=LaunchConfiguration('lidar_topic_name'))
    # ~~~~~~~~~~~~
    ignore_lidar_pose_from_tf_arg = DeclareLaunchArgument(
        "ignore_lidar_pose_from_tf", default_value="false", description="If true, the LiDAR pose will be assumed to be at the origin (base_link). Set to false (default) if you want to read the actual sensor pose from /tf")
    fixed_sensorpose_env_var = SetEnvironmentVariable(
        name='MOLA_USE_FIXED_LIDAR_POSE', value=LaunchConfiguration('ignore_lidar_pose_from_tf'))
    # ~~~~~~~~~~~~
    gnss_topic_name_arg = DeclareLaunchArgument(
        "gnss_topic_name", default_value="/gps", description="Topic name to listen for NavSatFix input from a GNSS (for example '/gps')")
    gps_topic_env_var = SetEnvironmentVariable(
        name='MOLA_GNSS_TOPIC', value=LaunchConfiguration('gnss_topic_name'))
    # ~~~~~~~~~~~~
    imu_topic_name_arg = DeclareLaunchArgument(
        "imu_topic_name", default_value="/imu", description="Topic name to listen for Imu input (for example '/imu')")
    imu_topic_env_var = SetEnvironmentVariable(
        name='MOLA_IMU_TOPIC', value=LaunchConfiguration('imu_topic_name'))
    # ~~~~~~~~~~~~
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to launch RViz2 with default lidar-odometry.rviz configuration")
    # ~~~~~~~~~~~~
    use_mola_gui_arg = DeclareLaunchArgument(
        "use_mola_gui", default_value="True", description="Whether to open MolaViz GUI interface for watching live mapping and control UI")
    use_mola_gui_env_var = SetEnvironmentVariable(
        name='MOLA_IMU_TOPIC', value=LaunchConfiguration('use_mola_gui'))

    # MOLA subsystem configuration YAML file
    # ------------------------------------------
    mola_system_yaml_file = os.path.join(
        myDir, 'mola-cli-launchs', 'lidar_odometry_ros2.yaml')

    # -------------------
    #        Node
    # -------------------
    mola_cli_node = Node(
        package='mola_launcher',
        executable='mola-cli',
        output='screen',
        arguments=[mola_system_yaml_file]
    )

    rviz2_node = Node(
        condition=IfCondition(use_rviz),
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
        gnss_topic_name_arg,
        gps_topic_env_var,
        imu_topic_name_arg,
        imu_topic_env_var,
        use_mola_gui_arg,
        use_mola_gui_env_var,
        mola_cli_node,
        use_rviz_arg,
        rviz2_node
    ])
