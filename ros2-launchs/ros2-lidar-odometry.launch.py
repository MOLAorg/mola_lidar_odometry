# ROS 2 launch file

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable,
                            GroupAction, Shutdown, OpaqueFunction)
from ament_index_python import get_package_share_directory
import os


def resolve_gnss_mode(context, *args, **kwargs):
    """
    Translate the user-facing `gnss_mode` into the low-level env vars that
    control georef estimation and initial localization.

    Values:
      - none         : do not use GNSS for estimation or relocalization (default).
      - log_only     : subscribe to GNSS and include readings in the simplemap,
                       but do not affect state estimation (useful for offline
                       post-processing georeferencing).
      - live_georef  : estimate the enu->map transform online. Requires smoother.
      - relocalize   : use GNSS to auto-initialize localization in a loaded
                       geo-referenced map. Requires smoother + initial map.

    Explicit `estimate_geo_reference` / `initial_localization_method` args
    still win when set by the user: we only apply the gnss_mode-implied
    override when that arg was left empty by the user.
    """
    mode = LaunchConfiguration('gnss_mode').perform(context).strip().lower()
    use_smoother = LaunchConfiguration(
        'use_state_estimator').perform(context).lower() == 'true'

    if mode in ('', 'none'):
        return []

    actions = []

    if mode == 'log_only':
        actions.append(SetEnvironmentVariable(
            name='MOLA_GENERATE_SIMPLEMAP', value='True'))
    elif mode == 'live_georef':
        if not use_smoother:
            raise RuntimeError(
                "\n\n[ERROR] gnss_mode:=live_georef requires use_state_estimator:=True "
                "(the smoother).\n"
            )
        user_geo_ref = LaunchConfiguration(
            'estimate_geo_reference').perform(context).strip()
        if not user_geo_ref:
            actions.append(SetEnvironmentVariable(
                name='MOLA_ESTIMATE_GEO_REF', value='True'))
    elif mode == 'relocalize':
        if not use_smoother:
            raise RuntimeError(
                "\n\n[ERROR] gnss_mode:=relocalize requires use_state_estimator:=True "
                "(the smoother).\n"
            )
        user_loc_method = LaunchConfiguration(
            'initial_localization_method').perform(context).strip()
        if not user_loc_method:
            actions.append(SetEnvironmentVariable(
                name='MOLA_LO_INITIAL_LOCALIZATION_METHOD',
                value='InitLocalization::FromStateEstimator'))
    else:
        raise RuntimeError(
            f"\n\n[ERROR] Unknown gnss_mode '{mode}'. "
            "Valid values: none, log_only, live_georef, relocalize.\n"
        )

    return actions


def validate_odometry_sources(context, *args, **kwargs):
    """
    Enforce the BridgeROS2 invariants on external odometry:

    1. Exactly one pathway at a time. The TF-based path
       (`forward_ros_tf_odom_to_mola:=True`) is a single-source legacy
       mechanism that hardcodes sensorLabel='odom'; combining it with a
       direct `nav_msgs/Odometry` subscription (`odom_topic_name:=...`)
       would feed duplicate observations of the same physical source to
       the state estimator.

    2. The TF-based path is incompatible with `use_state_estimator:=True`.
       The smoother publishes `map -> base_link` directly (not REP-105),
       so if an external wheel driver is also broadcasting
       `odom -> base_link` to /tf (which is exactly what Variant A needs
       to be there in order to be read), `base_link` ends up with two
       parents and tf2 rejects the tree. Use `odom_topic_name:=...`
       instead — nav_msgs/Odometry is consumed as an observation and does
       not collide with the bridge's /tf broadcast.
    """
    tf_path = LaunchConfiguration(
        'forward_ros_tf_odom_to_mola').perform(context).lower() == 'true'
    topic = LaunchConfiguration('odom_topic_name').perform(context).strip()
    use_smoother = LaunchConfiguration(
        'use_state_estimator').perform(context).lower() == 'true'
    if tf_path and topic:
        raise RuntimeError(
            "\n\n[ERROR] Two odometry sources are enabled simultaneously:\n"
            "  forward_ros_tf_odom_to_mola:=True  (via /tf)\n"
            f"  odom_topic_name:={topic!r}  (via nav_msgs/Odometry topic)\n"
            "These are mutually exclusive. Disable one of them.\n"
        )
    if tf_path and use_smoother:
        raise RuntimeError(
            "\n\n[ERROR] forward_ros_tf_odom_to_mola:=True is incompatible "
            "with use_state_estimator:=True (smoother).\n"
            "The smoother publishes `map -> base_link` directly; if an "
            "external driver is also publishing `odom -> base_link` to "
            "/tf, the tf2 tree becomes invalid (two parents for "
            "base_link).\n"
            "Use `odom_topic_name:=<your /odom topic>` instead — "
            "nav_msgs/Odometry observations do not participate in /tf.\n"
        )
    return []


def resolve_state_estimator_config(context, *args, **kwargs):
    """
    Runtime logic to resolve the YAML path. This prevents the launch file 
    from crashing if the optional smoother package is missing.
    """
    use_smoother = LaunchConfiguration(
        'use_state_estimator').perform(context).lower() == 'true'
    user_provided_path = LaunchConfiguration(
        'state_estimator_config_yaml').perform(context)

    # If the user manually provided a path via CLI, use that.
    if user_provided_path.strip() != "":
        return [SetEnvironmentVariable(name='MOLA_STATE_ESTIMATOR_YAML', value=user_provided_path)]

    # Otherwise, determine the default based on the estimator type
    if use_smoother:
        try:
            smoother_dir = get_package_share_directory(
                "mola_state_estimation_smoother")
            yaml_path = os.path.join(
                smoother_dir, "params", "state-estimation-smoother.yaml")
        except Exception:
            # Package missing: We only throw an error if the user explicitly requested the smoother
            raise RuntimeError(
                "\n\n[ERROR] 'use_state_estimator' is True, but the package 'mola_state_estimation_smoother' "
                "was not found. Please install it or set 'use_state_estimator:=False'.\n"
            )
    else:
        # Default path for the simple estimator
        yaml_path = '../state-estimator-params/state-estimation-simple.yaml'

    return [SetEnvironmentVariable(name='MOLA_STATE_ESTIMATOR_YAML', value=yaml_path)]


def generate_launch_description():
    myDir = get_package_share_directory("mola_lidar_odometry")

    # -------------------
    #     Arguments
    # -------------------
    # Mandatory
    lidar_topic_name_arg = DeclareLaunchArgument(
        "lidar_topic_name", description="Topic name to listen for LiDAR input, for example '/ouster/points' for PointCloud2 or '/scan' for LaserScan; see lidar_topic_type")
    lidar_topic_env_var = SetEnvironmentVariable(
        name='MOLA_LIDAR_TOPIC', value=LaunchConfiguration('lidar_topic_name'))

    # ~~~~~~~~~~~~
    # Smoother Specific Arguments (Only applied if use_state_estimator=True)
    # ~~~~~~~~~~~~
    navstate_kinematic_model_arg = DeclareLaunchArgument(
        "navstate_kinematic_model",
        default_value="KinematicModel::ConstantVelocity",
        description="[Smoother only] Kinematic model for internal motion model factors. Options: KinematicModel::ConstantVelocity, KinematicModel::Tricycle.")

    navstate_sliding_window_sec_arg = DeclareLaunchArgument(
        "navstate_sliding_window_sec",
        default_value="2.5",
        description="[Smoother only] Time window to keep past observations in the filter [seconds].")

    navstate_sigma_random_walk_linacc_arg = DeclareLaunchArgument(
        "navstate_sigma_random_walk_linacc",
        default_value="1.0",
        description="[Smoother only] Random walk model for linear acceleration uncertainty [m/s²].")

    navstate_sigma_random_walk_angacc_arg = DeclareLaunchArgument(
        "navstate_sigma_random_walk_angacc",
        default_value="10.0",
        description="[Smoother only] Random walk angular acceleration uncertainty [rad/s²].")

    estimate_geo_reference_arg = DeclareLaunchArgument(
        "estimate_geo_reference",
        default_value="",
        description="[Smoother only] Whether to estimate the best geo-referencing for {enu} -> {map} from incoming GNSS readings. "
                    "If empty (default), the pipeline YAML fallback is used (false) and `gnss_mode:=live_georef` may flip it to true automatically.")

    # ~~~~~~~~~~~~
    # Standard Arguments
    # ~~~~~~~~~~~~
    lidar_topic_type_arg = DeclareLaunchArgument(
        "lidar_topic_type", default_value="PointCloud2", description="The type of LiDAR topic to subscribe to. Options: 'PointCloud2' (default) or 'LaserScan'")
    lidar_topic_type_env_var = SetEnvironmentVariable(
        name='MOLA_LIDAR_TOPIC_TYPE', value=LaunchConfiguration('lidar_topic_type'))
    # ~~~~~~~~~~~~
    ignore_lidar_pose_from_tf_arg = DeclareLaunchArgument(
        "ignore_lidar_pose_from_tf", default_value="false", description="If true, the LiDAR pose will be assumed to be at the origin (base_link). Set to false (default) if you want to read the actual sensor pose from /tf")
    ignore_lidar_pose_from_tf_env_var = SetEnvironmentVariable(
        name='MOLA_USE_FIXED_LIDAR_POSE', value=LaunchConfiguration('ignore_lidar_pose_from_tf'))

    ignore_imu_pose_from_tf_arg = DeclareLaunchArgument(
        "ignore_imu_pose_from_tf", default_value="false", description="If true, the IMU pose will be assumed to be at the origin (base_link). Set to false (default) if you want to read the actual sensor pose from /tf")
    ignore_imu_pose_from_tf_env_var = SetEnvironmentVariable(
        name='MOLA_USE_FIXED_IMU_POSE', value=LaunchConfiguration('ignore_imu_pose_from_tf'))

    gnss_topic_name_arg = DeclareLaunchArgument(
        "gnss_topic_name", default_value="gps", description="Topic name to listen for NavSatFix input from a GNSS (for example '/gps')")
    gps_topic_env_var = SetEnvironmentVariable(
        name='MOLA_GNSS_TOPIC', value=LaunchConfiguration('gnss_topic_name'))
    # ~~~~~~~~~~~~
    gpsmsg_topic_name_arg = DeclareLaunchArgument(
        "gpsfix_topic_name", default_value="gpsfix", description="Topic name to listen for gps_msgs/GPSFix input from a GNSS (for example '/gpsfix')")
    gpsmsg_topic_env_var = SetEnvironmentVariable(
        name='MOLA_GPS_FIX_TOPIC', value=LaunchConfiguration('gpsfix_topic_name'))
    # ~~~~~~~~~~~~
    imu_topic_name_arg = DeclareLaunchArgument(
        "imu_topic_name", default_value="imu", description="Topic name to listen for Imu input (for example '/imu')")
    imu_topic_name_env_var = SetEnvironmentVariable(
        name='MOLA_IMU_TOPIC', value=LaunchConfiguration('imu_topic_name'))

    # ~~~~~~~~~~~~
    # Subscription QoS overrides (per REP-2003 the defaults are best-effort
    # with a depth of 50; raise depth and/or switch to reliable when the
    # publisher is reliable and high-rate, e.g. a 640 Hz IMU consumed
    # together with a heavy SLAM pipeline).
    # ~~~~~~~~~~~~
    imu_qos_reliability_arg = DeclareLaunchArgument(
        "imu_qos_reliability", default_value="best_effort",
        description="QoS reliability for the IMU subscription. Options: 'best_effort' (default) or 'reliable'. Set to 'reliable' to match a reliable high-rate publisher and avoid silent drops.")
    imu_qos_reliability_env_var = SetEnvironmentVariable(
        name='MOLA_IMU_QOS_RELIABILITY', value=LaunchConfiguration('imu_qos_reliability'))

    imu_qos_depth_arg = DeclareLaunchArgument(
        "imu_qos_depth", default_value="50",
        description="QoS history depth for the IMU subscription. Default 50; raise (e.g. 200-1000) for high-rate IMUs under SLAM load.")
    imu_qos_depth_env_var = SetEnvironmentVariable(
        name='MOLA_IMU_QOS_DEPTH', value=LaunchConfiguration('imu_qos_depth'))

    lidar_qos_reliability_arg = DeclareLaunchArgument(
        "lidar_qos_reliability", default_value="best_effort",
        description="QoS reliability for the LiDAR subscription. Options: 'best_effort' (default) or 'reliable'.")
    lidar_qos_reliability_env_var = SetEnvironmentVariable(
        name='MOLA_LIDAR_QOS_RELIABILITY', value=LaunchConfiguration('lidar_qos_reliability'))

    lidar_qos_depth_arg = DeclareLaunchArgument(
        "lidar_qos_depth", default_value="50",
        description="QoS history depth for the LiDAR subscription. Default 50.")
    lidar_qos_depth_env_var = SetEnvironmentVariable(
        name='MOLA_LIDAR_QOS_DEPTH', value=LaunchConfiguration('lidar_qos_depth'))

    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to launch RViz2 with default lidar-odometry.rviz configuration")

    # diagnostic_aggregator is OFF by default.
    #
    # Rationale: on a real robot there is typically a single, central
    # diagnostic_aggregator launched by the system integrator that groups
    # diagnostics from ALL nodes (drivers, controllers, navigation, MOLA-LO,
    # ...) into one /diagnostics_agg tree. Launching our own aggregator here
    # would duplicate / conflict with that system-wide one.
    #
    # Enable this flag only when:
    #   - You are running MOLA-LO in isolation (bring-up, demos, debugging).
    #   - You just want to quickly visualize MOLA-LO's own health in
    #     rqt_robot_monitor without setting up a central aggregator.
    #
    # Leave it False when:
    #   - MOLA-LO is part of a larger robot stack that already launches
    #     diagnostic_aggregator (the common production case). In that case
    #     just include "LidarOdometry" (or the relevant startswith/contains
    #     pattern) in your central aggregator YAML.
    use_diagnostic_aggregator = LaunchConfiguration('use_diagnostic_aggregator')
    use_diagnostic_aggregator_arg = DeclareLaunchArgument(
        "use_diagnostic_aggregator", default_value="False",
        description=(
            "Whether to launch a standalone diagnostic_aggregator with the "
            "MOLA-LO sample config (publishes /diagnostics_agg for "
            "rqt_robot_monitor). Enable for isolated bring-up/demos; leave "
            "disabled when a central aggregator is launched elsewhere in "
            "the robot stack."))

    use_mola_gui_arg = DeclareLaunchArgument(
        "use_mola_gui", default_value="True", description="Whether to open MolaViz GUI interface for watching live mapping and control UI")
    use_mola_gui_env_var = SetEnvironmentVariable(
        name='MOLA_WITH_GUI', value=LaunchConfiguration('use_mola_gui'))

    publish_localization_following_rep105_arg = DeclareLaunchArgument(
        "publish_localization_following_rep105", default_value="True", description="Whether to publish localization TFs in between map->odom (true) or directly map->base_link (false)")
    publish_localization_following_rep105_env_var = SetEnvironmentVariable(
        name='MOLA_LOCALIZ_USE_REP105', value=LaunchConfiguration('publish_localization_following_rep105'))

    start_mapping_enabled_arg = DeclareLaunchArgument(
        "start_mapping_enabled", default_value="True", description="Whether MOLA-LO should start with map update enabled (true), or in localization-only mode (false)")
    start_mapping_enabled_env_var = SetEnvironmentVariable(
        name='MOLA_MAPPING_ENABLED', value=LaunchConfiguration('start_mapping_enabled'))

    min_nearby_poses_occupied_arg = DeclareLaunchArgument(
        "min_nearby_poses_occupied", default_value="1",
        description="Minimum number of scans from a pose region before that region is considered 'occupied' in the local map. "
                    "Increase to 2+ for non-repetitive-scan lidars (e.g. Livox) to collect more data per location.")
    min_nearby_poses_occupied_env_var = SetEnvironmentVariable(
        name='MOLA_MIN_NEARBY_POSES_OCCUPIED', value=LaunchConfiguration('min_nearby_poses_occupied'))

    simplemap_min_nearby_poses_arg = DeclareLaunchArgument(
        "simplemap_min_nearby_poses", default_value="1",
        description="Same as min_nearby_poses_occupied but for the simplemap keyframe insertion.")
    simplemap_min_nearby_poses_env_var = SetEnvironmentVariable(
        name='MOLA_SIMPLEMAP_MIN_NEARBY_POSES', value=LaunchConfiguration('simplemap_min_nearby_poses'))

    start_active_arg = DeclareLaunchArgument(
        "start_active", default_value="True", description="Whether MOLA-LO should start active, that is, processing incoming sensor data (true), or ignoring them (false)")
    start_active_env_var = SetEnvironmentVariable(
        name='MOLA_START_ACTIVE', value=LaunchConfiguration('start_active'))

    mola_lo_reference_frame_arg = DeclareLaunchArgument(
        "mola_lo_reference_frame", default_value="map",
        description="Parent /tf frame of the localization update emitted by MOLA-LO (the `reference_frame` of its LocalizationUpdate; see ROS 2 API docs on published /tf).")
    mola_lo_reference_frame_env_var = SetEnvironmentVariable(
        name='MOLA_LO_PUBLISH_REF_FRAME', value=LaunchConfiguration('mola_lo_reference_frame'))

    # BridgeROS2's `odom_frame`. Under REP-105, the bridge publishes
    # `reference_frame -> odom_frame` (e.g. `map -> odom`) and reads the
    # external `odom_frame -> base_link` from /tf. This is independent from
    # LO's publish reference frame (`mola_lo_reference_frame`, the *parent*
    # of the localization TF).
    mola_bridge_odometry_frame_arg = DeclareLaunchArgument(
        "mola_bridge_odometry_frame", default_value="odom",
        description="BridgeROS2's odom /tf frame name (the REP-105 'odom' child "
                    "or the parent of an externally-published odometry TF).")
    mola_tf_estimated_odom_env_var = SetEnvironmentVariable(
        name='MOLA_TF_ESTIMATED_ODOMETRY', value=LaunchConfiguration('mola_bridge_odometry_frame'))

    mola_se_reference_frame_arg = DeclareLaunchArgument(
        "mola_state_estimator_reference_frame", default_value="map",
        description="Parent /tf frame of the pose updates emitted by the MOLA State Estimator, and BridgeROS2's `reference_frame` param.")
    mola_tf_map_env_var = SetEnvironmentVariable(
        name='MOLA_TF_MAP', value=LaunchConfiguration('mola_state_estimator_reference_frame'))

    mola_lo_pipeline_arg = DeclareLaunchArgument(
        "mola_lo_pipeline", default_value="../pipelines/lidar3d-default.yaml", description="The LiDAR-Odometry pipeline configuration YAML file defining the LO system.")
    mola_lo_pipeline_env_var = SetEnvironmentVariable(
        name='MOLA_ODOMETRY_PIPELINE_YAML', value=LaunchConfiguration('mola_lo_pipeline'))

    generate_simplemap_arg = DeclareLaunchArgument(
        "generate_simplemap", default_value="False", description="Whether to create a '.simplemap'")
    generate_simplemap_env_var = SetEnvironmentVariable(
        name='MOLA_GENERATE_SIMPLEMAP', value=LaunchConfiguration('generate_simplemap'))

    mola_initial_map_mm_file_arg = DeclareLaunchArgument(
        "mola_initial_map_mm_file", default_value="\"\"", description="Can be used to provide a metric map '.mm' file to be loaded as initial map. Refer to online tutorials.")
    mola_initial_map_mm_file_env_var = SetEnvironmentVariable(
        name='MOLA_LOAD_MM', value=LaunchConfiguration('mola_initial_map_mm_file'))

    mola_initial_map_sm_file_arg = DeclareLaunchArgument(
        "mola_initial_map_sm_file", default_value="\"\"", description="Initial keyframes map '.simplemap' file.")
    mola_initial_map_sm_file_env_var = SetEnvironmentVariable(
        name='MOLA_LOAD_SM', value=LaunchConfiguration('mola_initial_map_sm_file'))

    mola_footprint_to_base_link_tf_arg = DeclareLaunchArgument(
        "mola_footprint_to_base_link_tf", default_value="[0, 0, 0, 0, 0, 0]", description="Can be used to define a custom transformation between base_footprint and base_link. The coordinates are [x, y, z, yaw_deg, pitch_deg, roll_deg].")
    mola_footprint_to_base_link_tf_env_var = SetEnvironmentVariable(
        name='MOLA_TF_FOOTPRINT_TO_BASE_LINK', value=LaunchConfiguration('mola_footprint_to_base_link_tf'))

    enforce_planar_motion_arg = DeclareLaunchArgument(
        "enforce_planar_motion", default_value="False", description="Whether to enforce z, pitch, and roll to be zero.")
    enforce_planar_motion_env_var = SetEnvironmentVariable(
        name='MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION', value=LaunchConfiguration('enforce_planar_motion'))

    forward_ros_tf_odom_to_mola_arg = DeclareLaunchArgument(
        "forward_ros_tf_odom_to_mola", default_value="False", description="Whether to import an existing /tf 'odom'->'base_link' odometry (2D CObservationOdometry). Mutually exclusive with `odom_topic_name`.")
    forward_ros_tf_odom_to_mola_env_var = SetEnvironmentVariable(
        name='MOLA_FORWARD_ROS_TF_ODOM_TO_MOLA', value=LaunchConfiguration('forward_ros_tf_odom_to_mola'))

    odom_topic_name_arg = DeclareLaunchArgument(
        "odom_topic_name", default_value="",
        description="If non-empty, BridgeROS2 subscribes directly to this nav_msgs/Odometry topic and forwards each message as a 3D CObservationRobotPose (6x6 covariance) — preferred for smoother fusion. Mutually exclusive with `forward_ros_tf_odom_to_mola`.")
    odom_topic_name_env_var = SetEnvironmentVariable(
        name='MOLA_ODOM_TOPIC', value=LaunchConfiguration('odom_topic_name'))

    odom_sensor_label_arg = DeclareLaunchArgument(
        "odom_sensor_label", default_value="odom_wheels",
        description="sensorLabel attached to observations from `odom_topic_name`. Use distinct labels per source when fusing multiple external odometries.")
    odom_sensor_label_env_var = SetEnvironmentVariable(
        name='MOLA_ODOM_SENSOR_LABEL', value=LaunchConfiguration('odom_sensor_label'))

    initial_localization_method_arg = DeclareLaunchArgument(
        "initial_localization_method", default_value="",
        description="Initial-localization method. Options: InitLocalization::FixedPose (start at identity or given pose), "
                    "InitLocalization::FromStateEstimator (wait for smoother convergence, e.g. from GNSS), "
                    "InitLocalization::PitchAndRollFromIMU (use IMU to estimate pitch/roll at startup, assumes sensor stationary). "
                    "If empty (default), the pipeline YAML fallback is used (FixedPose) and `gnss_mode:=relocalize` may switch it to FromStateEstimator.")

    def _apply_initial_localization_method(context, *args, **kwargs):
        v = LaunchConfiguration('initial_localization_method').perform(context).strip()
        return [SetEnvironmentVariable(name='MOLA_LO_INITIAL_LOCALIZATION_METHOD', value=v)] if v else []
    initial_localization_method_env_var = OpaqueFunction(
        function=_apply_initial_localization_method)

    use_state_estimator_arg = DeclareLaunchArgument(
        "use_state_estimator", default_value="False",
        description="If true, uses StateEstimationSmoother (requires optional package).")

    # Convenience high-level GNSS mode selector (see resolve_gnss_mode()).
    gnss_mode_arg = DeclareLaunchArgument(
        "gnss_mode", default_value="none",
        description="High-level GNSS usage: none | log_only | live_georef | relocalize. "
                    "'live_georef' and 'relocalize' require use_state_estimator:=True.")

    # MOLA_ESTIMATE_GEO_REF is set only when the user explicitly provides
    # `estimate_geo_reference:=...`; otherwise the pipeline YAML fallback
    # (false) is used, and `gnss_mode:=live_georef` in resolve_gnss_mode()
    # may still flip it to true.
    def _apply_estimate_geo_ref(context, *args, **kwargs):
        if LaunchConfiguration('use_state_estimator').perform(context).lower() != 'true':
            return []
        v = LaunchConfiguration('estimate_geo_reference').perform(context).strip()
        return [SetEnvironmentVariable(name='MOLA_ESTIMATE_GEO_REF', value=v)] if v else []

    # Environment variables that only apply if the smoother is active
    smoother_env_vars = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_state_estimator')),
        actions=[
            SetEnvironmentVariable('MOLA_NAVSTATE_KINEMATIC_MODEL', LaunchConfiguration(
                'navstate_kinematic_model')),
            SetEnvironmentVariable('MOLA_NAVSTATE_SLIDING_WINDOW_SEC', LaunchConfiguration(
                'navstate_sliding_window_sec')),
            SetEnvironmentVariable('MOLA_NAVSTATE_SIGMA_RANDOM_WALK_LINACC', LaunchConfiguration(
                'navstate_sigma_random_walk_linacc')),
            SetEnvironmentVariable('MOLA_NAVSTATE_SIGMA_RANDOM_WALK_ANGACC', LaunchConfiguration(
                'navstate_sigma_random_walk_angacc')),
            OpaqueFunction(function=_apply_estimate_geo_ref),
        ]
    )

    # Class selection env var
    use_state_estimator_env_var = SetEnvironmentVariable(
        name='MOLA_STATE_ESTIMATOR', value=PythonExpression([
            "'mola::state_estimation_smoother::StateEstimationSmoother' if ",
            LaunchConfiguration('use_state_estimator'),
            " else 'mola::state_estimation_simple::StateEstimationSimple'"
        ]))

    localization_publish_tf_source_env_var = SetEnvironmentVariable(
        name='MOLA_LOCALIZATION_PUBLISH_TF_SOURCE',
        value=PythonExpression([
            "'state_estimator' if ", LaunchConfiguration(
                'use_state_estimator'), " else 'lidar_odometry'"
        ])
    )
    localization_publish_odom_source_env_var = SetEnvironmentVariable(
        name='MOLA_LOCALIZATION_PUBLISH_ODOM_MSGS_SOURCE',
        value=PythonExpression([
            "'state_estimator' if ", LaunchConfiguration(
                'use_state_estimator'), " else 'lidar_odometry'"
        ])
    )
    # Config YAML Argument (Default is empty to trigger OpaqueFunction auto-detection)
    state_estimator_config_yaml_arg = DeclareLaunchArgument(
        "state_estimator_config_yaml", default_value="",
        description="Path to estimator YAML. If empty, it is auto-resolved based on use_state_estimator.")

    lidar_scan_validity_minimum_point_count_arg = DeclareLaunchArgument(
        "lidar_scan_validity_minimum_point_count", default_value="100",
        description="Minimum number of points required in an incoming LiDAR scan for it to be processed; scans below this threshold are discarded.")
    lidar_scan_validity_minimum_point_env_var = SetEnvironmentVariable(
        name='MOLA_OBS_VALIDITY_MIN_POINTS', value=LaunchConfiguration('lidar_scan_validity_minimum_point_count'))
    lidar_scan_validity_enable_env_var = SetEnvironmentVariable(
        name='MOLA_ENABLE_OBS_VALIDITY_FILTER', value='True')

    mola_deskew_method_arg = DeclareLaunchArgument(
        "mola_deskew_method", default_value="MotionCompensationMethod::Linear",
        description="Motion-compensation (deskew) method for LiDAR scans. Options: MotionCompensationMethod::None, "
                    "MotionCompensationMethod::Linear (default, constant-velocity), MotionCompensationMethod::IMU "
                    "(requires an IMU topic; use the higher-level `use_imu_for_lio:=True` to enable LIO mode).")

    # Convenience flag: LiDAR-Inertial Odometry. When true, overrides
    # `mola_deskew_method` to `MotionCompensationMethod::IMU`. Requires `imu_topic_name`
    # to be a valid IMU topic.
    use_imu_for_lio_arg = DeclareLaunchArgument(
        "use_imu_for_lio", default_value="False",
        description="If true, enable LIO mode (MotionCompensationMethod::IMU for deskew). Requires a working imu_topic_name.")
    mola_deskew_method_env_var = SetEnvironmentVariable(
        name='MOLA_DESKEW_METHOD',
        value=PythonExpression([
            "'MotionCompensationMethod::IMU' if ",
            LaunchConfiguration('use_imu_for_lio'),
            " else '",
            LaunchConfiguration('mola_deskew_method'),
            "'"
        ]))
    # ~~~~~~~~~~~~
    imu_gravity_correction_arg = DeclareLaunchArgument(
        "imu_gravity_correction", default_value="true", description="Whether to use IMU accelerometer readings to constrain ICP pitch/roll (prevents vertical drift; safe to leave enabled even without an IMU)")
    imu_gravity_correction_env_var = SetEnvironmentVariable(
        name='MOLA_IMU_GRAVITY_CORRECTION', value=LaunchConfiguration('imu_gravity_correction'))
    # ~~~~~~~~~~~~
    imu_gravity_sigma_deg_arg = DeclareLaunchArgument(
        "imu_gravity_sigma_deg", default_value="2.0", description="Sigma [degrees] for the gravity-derived pitch/roll prior. Lower values = more trust in IMU.")
    imu_gravity_sigma_deg_env_var = SetEnvironmentVariable(
        name='MOLA_IMU_GRAVITY_SIGMA_DEG', value=LaunchConfiguration('imu_gravity_sigma_deg'))
    # ~~~~~~~~~~~~
    imu_gravity_avg_samples_arg = DeclareLaunchArgument(
        "imu_gravity_avg_samples", default_value="20", description="Number of IMU samples to average when estimating the gravity direction for pitch/roll correction.")
    imu_gravity_avg_samples_env_var = SetEnvironmentVariable(
        name='MOLA_IMU_GRAVITY_AVG_SAMPLES', value=LaunchConfiguration('imu_gravity_avg_samples'))
    # ~~~~~~~~~~~~
    imu_gravity_max_age_arg = DeclareLaunchArgument(
        "imu_gravity_max_age", default_value="2.0", description="Maximum age [seconds] of IMU samples used for gravity alignment. Samples older than this are discarded.")
    imu_gravity_max_age_env_var = SetEnvironmentVariable(
        name='MOLA_IMU_GRAVITY_MAX_AGE', value=LaunchConfiguration('imu_gravity_max_age'))
    # ~~~~~~~~~~~~
    mola_tf_base_link_arg = DeclareLaunchArgument(
        "mola_tf_base_link", default_value="base_link", description="The /tf frame name for the robot base link.")
    mola_tf_base_link_env_var = SetEnvironmentVariable(
        name='MOLA_TF_BASE_LINK', value=LaunchConfiguration('mola_tf_base_link'))
    # Ensure LO's published frame matches the bridge/SE base_link frame
    # (used in LocalizationUpdate.child_frame for TF and odometry output):
    mola_lo_publish_vehicle_frame_env_var = SetEnvironmentVariable(
        name='MOLA_LO_PUBLISH_VEHICLE_FRAME', value=LaunchConfiguration('mola_tf_base_link'))

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

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (bag) clock if true')

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
            arguments=[mola_system_yaml_file],
            parameters=[{'use_sim_time': use_sim_time}],
            on_exit=Shutdown()
        ),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            remappings=tf_remaps,
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-d', [os.path.join(myDir, 'rviz2', 'lidar-odometry.rviz')]]
        ),

        # Optional standalone diagnostic_aggregator (see flag docs above).
        Node(
            condition=IfCondition(use_diagnostic_aggregator),
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name='diagnostic_aggregator',
            parameters=[
                os.path.join(
                    get_package_share_directory('mola_lidar_odometry'),
                    'config', 'diagnostics_aggregator.yaml'),
                {'use_sim_time': use_sim_time},
            ],
        )
    ])

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_use_sim_time_cmd,
        enforce_planar_motion_arg,
        enforce_planar_motion_env_var,
        forward_ros_tf_odom_to_mola_arg,
        forward_ros_tf_odom_to_mola_env_var,
        odom_topic_name_arg,
        odom_topic_name_env_var,
        odom_sensor_label_arg,
        odom_sensor_label_env_var,
        generate_simplemap_arg,
        generate_simplemap_env_var,
        gnss_topic_name_arg,
        gps_topic_env_var,
        gpsmsg_topic_name_arg,
        gpsmsg_topic_env_var,
        ignore_imu_pose_from_tf_arg,
        ignore_imu_pose_from_tf_env_var,
        ignore_lidar_pose_from_tf_arg,
        ignore_lidar_pose_from_tf_env_var,
        imu_topic_name_arg,
        imu_topic_name_env_var,
        imu_qos_reliability_arg,
        imu_qos_reliability_env_var,
        imu_qos_depth_arg,
        imu_qos_depth_env_var,
        lidar_qos_reliability_arg,
        lidar_qos_reliability_env_var,
        lidar_qos_depth_arg,
        lidar_qos_depth_env_var,
        initial_localization_method_arg,
        initial_localization_method_env_var,
        lidar_scan_validity_enable_env_var,
        lidar_scan_validity_minimum_point_count_arg,
        lidar_scan_validity_minimum_point_env_var,
        lidar_topic_env_var,
        lidar_topic_name_arg,
        lidar_topic_type_arg,
        lidar_topic_type_env_var,
        imu_gravity_correction_arg,
        imu_gravity_correction_env_var,
        imu_gravity_sigma_deg_arg,
        imu_gravity_sigma_deg_env_var,
        imu_gravity_avg_samples_arg,
        imu_gravity_avg_samples_env_var,
        imu_gravity_max_age_arg,
        imu_gravity_max_age_env_var,
        use_imu_for_lio_arg,
        mola_deskew_method_arg,
        mola_deskew_method_env_var,
        mola_footprint_to_base_link_tf_arg,
        mola_footprint_to_base_link_tf_env_var,
        mola_initial_map_mm_file_arg,
        mola_initial_map_mm_file_env_var,
        mola_initial_map_sm_file_arg,
        mola_initial_map_sm_file_env_var,
        mola_lo_pipeline_arg,
        mola_lo_pipeline_env_var,
        mola_lo_reference_frame_arg,
        mola_lo_reference_frame_env_var,
        mola_bridge_odometry_frame_arg,
        mola_tf_estimated_odom_env_var,
        mola_se_reference_frame_arg,
        mola_tf_base_link_arg,
        mola_tf_base_link_env_var,
        mola_lo_publish_vehicle_frame_env_var,
        mola_tf_map_env_var,
        publish_localization_following_rep105_arg,
        publish_localization_following_rep105_env_var,
        start_active_arg,
        start_active_env_var,
        start_mapping_enabled_arg,
        start_mapping_enabled_env_var,
        min_nearby_poses_occupied_arg,
        min_nearby_poses_occupied_env_var,
        simplemap_min_nearby_poses_arg,
        simplemap_min_nearby_poses_env_var,
        use_mola_gui_arg,
        use_mola_gui_env_var,
        use_rviz_arg,
        use_diagnostic_aggregator_arg,
        use_state_estimator_arg,
        use_state_estimator_env_var,
        # Reject: (a) enabling the /tf pathway and a direct /odom subscription
        # at once, and (b) /tf pathway + smoother (would break tf2 tree).
        # Must run after use_state_estimator_arg is declared.
        OpaqueFunction(function=validate_odometry_sources),
        gnss_mode_arg,

        # Smoother Specific
        navstate_kinematic_model_arg,
        navstate_sliding_window_sec_arg,
        navstate_sigma_random_walk_linacc_arg,
        navstate_sigma_random_walk_angacc_arg,
        estimate_geo_reference_arg,
        smoother_env_vars,

        # Must run AFTER generate_simplemap_env_var, initial_localization_method_env_var,
        # and smoother_env_vars so the high-level gnss_mode overrides their low-level args.
        OpaqueFunction(function=resolve_gnss_mode),

        # Config YAML must come later
        state_estimator_config_yaml_arg,
        OpaqueFunction(function=resolve_state_estimator_config),

        localization_publish_tf_source_env_var,
        localization_publish_odom_source_env_var,
        # group
        node_group
    ])