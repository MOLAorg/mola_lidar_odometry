.. This file becomes embedded into root MOLA / ROS2API docs page, within the MOLA-LO node docs:

.. note::

   It is recommended to start with the tutorial on how to :ref:`build a map <building-maps>`.

This launch file (`view sources <https://github.com/MOLAorg/mola_lidar_odometry/blob/develop/ros2-launchs/ros2-lidar-odometry.launch.py>`_)
runs **MOLA-LO** live on point clouds received from a ROS 2 topic, **demonstrating a few features**:

.. tip::

   See the :ref:`MOLA in ROS 2 cookbook <mola_ros2_cookbook>` for a
   configuration-by-configuration walkthrough (Simple vs Smoother,
   namespaced, GNSS modes, etc.) with live-editable commands.

* Launching and visualizing LO in both, ``mola_viz`` and ``RViz2`` (or use FoxGlove if preferred).
* How MOLA ``mola_lidar_odometry`` publishes the local map,
  the estimated trajectory, and `/tf` for the estimated odometry.

.. image:: https://mrpt.github.io/imgs/mola-lo-ros2-launch-demo-live-forest.png

.. tab-set::

   .. tab-item:: Basic LO usage

      .. code-block:: bash

         # Minimal LO use case (requires correct LiDAR sensor /tf):
         ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
            lidar_topic_name:=ouster/points

         # LO usage without sensor /tf:
         ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
            lidar_topic_name:=ouster/points \
            ignore_lidar_pose_from_tf:=True \
            publish_localization_following_rep105:=False

   .. tab-item:: LIO usage (Ouster with /tf)
      :selected:

      This is how to use LiDAR-Inertial Odometry (LIO) by using LiDAR clouds plus an IMU:

      .. code-block:: bash

         # Example LIO usage for Ouster LiDAR + integrated IMU:
         ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
           mola_deskew_method:=MotionCompensationMethod::IMU \
           lidar_topic_name:=/ouster/points \
           imu_topic_name:=/ouster/imu \
           mola_tf_base_link:=os_sensor

   .. tab-item:: LIO usage (No /tf)

      This is how to use LiDAR-Inertial Odometry (LIO) by using LiDAR clouds plus an IMU,
      when no /tf is available for the sensor poses so you must manually specify them:

      .. code-block:: bash

         # Example LIO usage for LiDAR + IMU for Oxford Spires Dataset:
         IMU_POSE_YAW=90 \
         LIDAR_POSE_YAW=180 \
         ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
           mola_deskew_method:=MotionCompensationMethod::IMU \
           lidar_topic_name:=/hesai/pandar \
           imu_topic_name:=/alphasense_driver_ros/imu \
           ignore_lidar_pose_from_tf:=True \
           ignore_imu_pose_from_tf:=True \
           publish_localization_following_rep105:=False


   .. tab-item:: Robot with NS

      If your robot uses a ROS 2 namespace ``ROBOT_NS`` for all its sensor and tf topics, use:

      .. code-block:: bash

         # Minimal use case:
         ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
            lidar_topic_name:=ouster/points \
            use_namespace:=True \
            namespace:=ROBOT_NS

   .. tab-item:: 2D LiDAR

      To use with a 2D LiDAR, define the argument `lidar_topic_type:=LaserScan`, e.g.:

      .. code-block:: bash

         # Minimal use case:
         ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
            lidar_topic_name:=/scan \
            lidar_topic_type:=LaserScan \
            mola_lo_pipeline:=../pipelines/lidar2d.yaml \
            ignore_lidar_pose_from_tf:=False \
            publish_localization_following_rep105:=True

|

.. dropdown:: How to invoke for a rosbag (``.mcap``, ``.db3``)
    :icon: list-unordered

    You can also directly run MOLA-LO on a dataset instead of live ROS messages, which is normally more efficient.
    See MOLA-LO GUI apps for all the details.

    .. code-block:: bash

       # Example: run MOLA-LIO on a Ouster dataset.
       MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU  \
       MOLA_IMU_TOPIC=/ouster/imu \
       MOLA_LIDAR_TOPIC=/ouster/points \
       MOLA_TF_BASE_LINK=os_sensor \
       mola-lo-gui-rosbag2  /path/to/your/dataset.mcap

|


.. _mola_lo_ros_launch_arguments:

.. dropdown:: All launch arguments
    :open:
    :icon: list-unordered

    .. code-block:: bash

       $ ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py --show-args
       Arguments (pass arguments as '<name>:=<value>'):

         'namespace':
            Top-level namespace
            (default: '')

         'use_namespace':
            Whether to apply a namespace to the navigation stack
            (default: 'false')

         'enforce_planar_motion':
            Whether to enforce z, pitch, and roll to be zero.
            (default: 'False')

         'forward_ros_tf_odom_to_mola':
            Whether to import an existing /tf 'odom'->'base_link' odometry.
            (default: 'False')

         'generate_simplemap':
            Whether to create a '.simplemap'
            (default: 'False')

         'gnss_topic_name':
            Topic name to listen for NavSatFix input from a GNSS (for example '/gps')
            (default: 'gps')

          'gpsfix_topic_name':
             Topic name to listen for gps_msgs/GPSFix input from a GNSS (for example '/gpsfix')
             (default: 'gpsfix')

          'ignore_imu_pose_from_tf':
             If true, the IMU pose will be assumed to be at the origin (base_link). Set to false (default) if you want to read the actual sensor pose from /tf
             (default: 'false')

         'ignore_lidar_pose_from_tf':
            If true, the LiDAR pose will be assumed to be at the origin (base_link). Set to false (default) if you want to read the actual sensor pose from /tf
            (default: 'false')

         'imu_topic_name':
            Topic name to listen for Imu input (for example '/imu')
            (default: 'imu')

         'initial_localization_method':
            Method for initialization.
            (default: 'InitLocalization::FixedPose')

         'lidar_scan_validity_minimum_point_count':
            Minimum number of points in each LiDAR raw scan for it to be considered valid; otherwise, it is ignored.
            (default: '100')

         'lidar_topic_name':
            Topic name to listen for PointCloud2 input from the LiDAR (for example '/ouster/points')

          'lidar_topic_type':
             The type of LiDAR topic to subscribe to. Options: 'PointCloud2' (default) or 'LaserScan'
             (default: 'PointCloud2')

          'mola_deskew_method':
             Which motion-compensation method to use to align LiDAR scans more precisely
             (default: 'MotionCompensationMethod::Linear')

         'mola_footprint_to_base_link_tf':
            Can be used to define a custom transformation between base_footprint and base_link. The coordinates are [x, y, z, yaw_deg, pitch_deg, roll_deg].
            (default: '[0, 0, 0, 0, 0, 0]')

         'mola_initial_map_mm_file':
            Can be used to provide a metric map '.mm' file to be loaded as initial map. Refer to online tutorials.
            (default: '""')

         'mola_initial_map_sm_file':
            Initial keyframes map '.simplemap' file.
            (default: '""')

         'mola_lo_pipeline':
            The LiDAR-Odometry pipeline configuration YAML file defining the LO system.
            (default: '../pipelines/lidar3d-default.yaml')

         'mola_lo_reference_frame':
            The /tf frame name to be used for MOLA-LO localization updates
            (default: 'map')

         'mola_state_estimator_reference_frame':
            The /tf frame name to be used as reference for MOLA State Estimators to publish pose updates
            (default: 'map')

         'mola_tf_base_link':
            The /tf frame name for the robot base link.
            (default: 'base_link')

         'publish_localization_following_rep105':
            Whether to publish localization TFs in between map->odom (true) or directly map->base_link (false)
            (default: 'True')

         'start_active':
            Whether MOLA-LO should start active, that is, processing incoming sensor data (true), or ignoring them (false)
            (default: 'True')

         'start_mapping_enabled':
            Whether MOLA-LO should start with map update enabled (true), or in localization-only mode (false)
            (default: 'True')

         'use_mola_gui':
            Whether to open MolaViz GUI interface for watching live mapping and control UI
            (default: 'True')

         'use_rviz':
            Whether to launch RViz2 with default lidar-odometry.rviz configuration
            (default: 'True')

         'use_state_estimator':
            If true, uses StateEstimationSmoother (requires optional package).
            (default: 'False')

         'navstate_kinematic_model':
            [Smoother only] Kinematic model for internal motion model factors. Options: KinematicModel::ConstantVelocity, KinematicModel::Tricycle.
            (default: 'KinematicModel::ConstantVelocity')

         'navstate_sliding_window_sec':
            [Smoother only] Time window to keep past observations in the filter [seconds].
            (default: '2.5')

         'navstate_sigma_random_walk_linacc':
            [Smoother only] Random walk model for linear acceleration uncertainty [m/s²].
            (default: '1.0')

         'navstate_sigma_random_walk_angacc':
            [Smoother only] Random walk angular acceleration uncertainty [rad/s²].
            (default: '10.0')

         'estimate_geo_reference':
            [Smoother only] Whether to estimate the best geo-referencing for {enu} -> {map} from incoming GNSS readings.
            (default: 'False')

         'state_estimator_config_yaml':
            Path to estimator YAML. If empty, it is auto-resolved based on use_state_estimator.
            (default: '')

.. _mola_lo_ros_mola-cli-env-vars:

.. dropdown:: Configure sensor inputs for ROS 2 node and rosbag2 input
    :icon: list-unordered

    The following environment variables can be set to change the behavior of how ``BridgeROS2``
    handles input ROS 2 messages on sensor inputs.
    Please, refer to the actual mola-cli launch files where these variables are defined:

    - `mola-cli-launchs/lidar_odometry_from_rosbag2.yaml <https://github.com/MOLAorg/mola_lidar_odometry/blob/develop/mola-cli-launchs/lidar_odometry_from_rosbag2.yaml>`_
    - `mola-cli-launchs/lidar_odometry_ros2.yaml <https://github.com/MOLAorg/mola_lidar_odometry/blob/develop/mola-cli-launchs/lidar_odometry_ros2.yaml>`_

    Environment variables:

    - ``MOLA_TF_BASE_LINK`` (Default: ``"base_link"``): The robot reference frame id in ``/tf``. Used to get sensor poses with respect to the vehicle.

    - ``MOLA_TF_FOOTPRINT_LINK`` (Default: ``base_footprint``): If not empty, the node will broadcast a static /tf from base_footprint to base_link with the TF base_footprint_to_base_link_tf at start up.

    - ``MOLA_TF_FOOTPRINT_TO_BASE_LINK`` (Default: ``'[0, 0, 0, 0, 0, 0]'``): [x, y, z, yaw_deg, pitch_deg, roll_deg].

    - ``MOLA_LIDAR_TOPIC`` (Default: ``'/ouster/points'``): The ``sensor_msgs/PointCloud2`` topic with raw LiDAR data (mandatory).

    - ``MOLA_USE_FIXED_LIDAR_POSE`` (Default: ``false``): If false, sensor pose will be retrieved from ``/tf``. You can also set it to true and then the sensor pose will be given by these env. variables:

        - ``LIDAR_POSE_X``, ``LIDAR_POSE_Y``, ``LIDAR_POSE_Z`` (in meters).
        - ``LIDAR_POSE_YAW``, ``LIDAR_POSE_PITCH``, ``LIDAR_POSE_ROLL`` (in degrees).

    - ``MOLA_GNSS_TOPIC`` (Default: ``'/gps'``): The ``sensor_msgs/NavSatFix`` topic with GNSS data (optional).
    
    - ``MOLA_USE_FIXED_GNSS_POSE`` (Default: ``true``): If false, sensor pose will be retrieved from ``/tf``. You can also set it to true and then the sensor pose will be given by these env. variables:

        - ``GNSS_POSE_X``, ``GNSS_POSE_Y``, ``GNSS_POSE_Z`` (in meters).
        - ``GNSS_POSE_YAW``, ``GNSS_POSE_PITCH``, ``GNSS_POSE_ROLL`` (in degrees).

    - ``MOLA_IMU_TOPIC`` (Default: ``'/imu'``): The ``sensor_msgs/Imu`` topic with IMU data (optional).
    
    - ``MOLA_USE_FIXED_IMU_POSE`` (Default: ``true``): If false, sensor pose will be retrieved from ``/tf``. You can also set it to true and then the sensor pose will be given by these env. variables:

        - ``IMU_POSE_X``, ``IMU_POSE_Y``, ``IMU_POSE_Z`` (in meters).
        - ``IMU_POSE_YAW``, ``IMU_POSE_PITCH``, ``IMU_POSE_ROLL`` (in degrees).


.. dropdown:: More LO parameters
    :icon: list-unordered

    If using the default :ref:`pipeline <mola_lo_pipelines>`, the ``lidar3d-gicp.yaml`` pipeline file defines plenty
    of :ref:`additional parameters and options <mola_3d_gicp_pipeline>` that you can explore.

    See also the docs for the :ref:`ROS 2 API <mola_ros2api>` and :ref:`this tutorial <tutorial-mola-lo-map-and-localize>` on how to save and load a map using ROS 2 MOLA-LO nodes.

