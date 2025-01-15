
# ROS 2 launch file

from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import GroupAction
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
        "gnss_topic_name", default_value="gps", description="Topic name to listen for NavSatFix input from a GNSS (for example '/gps')")
    gps_topic_env_var = SetEnvironmentVariable(
        name='MOLA_GNSS_TOPIC', value=LaunchConfiguration('gnss_topic_name'))
    # ~~~~~~~~~~~~
    imu_topic_name_arg = DeclareLaunchArgument(
        "imu_topic_name", default_value="imu", description="Topic name to listen for Imu input (for example '/imu')")
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
        name='MOLA_WITH_GUI', value=LaunchConfiguration('use_mola_gui'))
    # ~~~~~~~~~~~~
    publish_localization_following_rep105_arg = DeclareLaunchArgument(
        "publish_localization_following_rep105", default_value="True", description="Whether to publish localization TFs in between map->odom (true) or directly map->base_link (false)")
    publish_localization_following_rep105_env_var = SetEnvironmentVariable(
        name='MOLA_LOCALIZ_USE_REP105', value=LaunchConfiguration('publish_localization_following_rep105'))
    # ~~~~~~~~~~~~
    start_mapping_enabled_arg = DeclareLaunchArgument(
        "start_mapping_enabled", default_value="True", description="Whether MOLA-LO should start with map update enabled (true), or in localization-only mode (false)")
    start_mapping_enabled_env_var = SetEnvironmentVariable(
        name='MOLA_MAPPING_ENABLE', value=LaunchConfiguration('start_mapping_enabled'))
    # ~~~~~~~~~~~~
    start_active_arg = DeclareLaunchArgument(
        "start_active", default_value="True", description="Whether MOLA-LO should start active, that is, processing incoming sensor data (true), or ignoring them (false)")
    start_active_env_var = SetEnvironmentVariable(
        name='MOLA_START_ACTIVE', value=LaunchConfiguration('start_mapping_enabled'))
    # ~~~~~~~~~~~~

    # Namespace (Based on Nav2's bring-up launch file!)
    # ---------------------------------------------------
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    #
    # (JLBC further explanation) The problem is the "tf2" library. It's hardcoded to subscribe
    # to "/tf". This remapping allows "/robot/tf" to be seen as "/tf" so tf2_ros (and RViz) can see it.
    #
    tf_remaps = [('/tf', 'tf'),
                 ('/tf_static', 'tf_static')]

    # MOLA subsystem configuration YAML file
    # ------------------------------------------
    mola_system_yaml_file = os.path.join(
        myDir, 'mola-cli-launchs', 'lidar_odometry_ros2.yaml')

    # -------------------
    #        Node
    # -------------------
    node_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            package='mola_launcher',
            executable='mola-cli',
            output='screen',
            remappings=tf_remaps,
            arguments=[mola_system_yaml_file]
        ),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            remappings=tf_remaps,
            arguments=[
                '-d', [os.path.join(myDir, 'rviz2', 'lidar-odometry.rviz')]]
        )
    ])

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_namespace_cmd,
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
        publish_localization_following_rep105_arg,
        publish_localization_following_rep105_env_var,
        start_mapping_enabled_arg,
        start_mapping_enabled_env_var,
        start_active_arg,
        start_active_env_var,
        use_rviz_arg,
        node_group
    ])
