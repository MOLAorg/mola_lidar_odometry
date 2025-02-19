.. This file becomes embedded into root MOLA / ROS2API docs page, within the MOLA-LO node docs:

.. note::

   It is recommended to start with the tutorial on how to :ref:`build a map <building-maps>`.

This launch file (`view sources <https://github.com/MOLAorg/mola_lidar_odometry/blob/develop/ros2-launchs/ros2-lidar-odometry.launch.py>`_)
runs **MOLA-LO** live on point clouds received from a ROS 2 topic, **demonstrating a few features**:

* Launching and visualizing LO in both, ``mola_viz`` and ``RViz2`` (or use FoxGlove if preferred).
* How MOLA ``mola_lidar_odometry`` publishes the local map,
  the estimated trajectory, and `/tf` for the estimated odometry.

.. image:: https://mrpt.github.io/imgs/mola-lo-ros2-launch-demo-live-forest.png

.. tab-set::

   .. tab-item:: Basic usage
      :selected:

      .. code-block:: bash

         # Minimal use case (requires correct LiDAR sensor /tf):
         ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
            lidar_topic_name:=ouster/points

         # Usage without sensor /tf:
         ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
            lidar_topic_name:=ouster/points \
            ignore_lidar_pose_from_tf:=True \
            publish_localization_following_rep105:=False

   .. tab-item:: Robot with NS
      :selected:

      If your robot uses a ROS 2 namespace ``ROBOT_NS`` for all its sensor and tf topics, use:

      .. code-block:: bash

         # Minimal use case:
         ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
            lidar_topic_name:=ouster/points \
            use_namespace:=True \
            namespace:=ROBOT_NS

   .. tab-item:: All launch arguments

      .. code-block:: bash

            $ ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py --show-args
            Arguments (pass arguments as '<name>:=<value>'):

               'namespace':
                  Top-level namespace
                  (default: '')

               'use_namespace':
                  Whether to apply a namespace to the navigation stack
                  (default: 'false')

               'lidar_topic_name':
                  Topic name to listen for PointCloud2 input from the LiDAR (for example '/ouster/points')

               'ignore_lidar_pose_from_tf':
                  If true, the LiDAR pose will be assumed to be at the origin (base_link). Set to false (default) if you want to read the actual sensor pose from /tf
                  (default: 'false')

               'gnss_topic_name':
                  Topic name to listen for NavSatFix input from a GNSS (for example '/gps')
                  (default: 'gps')

               'imu_topic_name':
                  Topic name to listen for Imu input (for example '/imu')
                  (default: 'imu')

               'use_mola_gui':
                  Whether to open MolaViz GUI interface for watching live mapping and control UI
                  (default: 'True')

               'publish_localization_following_rep105':
                  Whether to publish localization TFs in between map->odom (true) or directly map->base_link (false)
                  (default: 'True')

               'start_mapping_enabled':
                  Whether MOLA-LO should start with map update enabled (true), or in localization-only mode (false)
                  (default: 'True')

               'start_active':
                  Whether MOLA-LO should start active, that is, processing incoming sensor data (true), or ignoring them (false)
                  (default: 'True')

               'mola_lo_reference_frame':
                  The /tf frame name to be used for MOLA-LO localization updates
                  (default: 'map')

               'mola_lo_pipeline':
                  The LiDAR-Odometry pipeline configuration YAML file defining the LO system. Absolute path, or relative to 'mola-cli-launchs/lidar_odometry_ros2.yaml'. Default is the 'lidar3d-default.yaml' system described in the IJRR 2025 paper.
                  (default: '../pipelines/lidar3d-default.yaml')

               'generate_simplemap':
                  Whether to create a '.simplemap', useful for map post-processing. Refer to online tutorials.
                  (default: 'False')

               'mola_initial_map_mm_file':
                  Can be used to provide a metric map '.mm' file to be loaded as initial map. Refer to online tutorials.
                  (default: '""')

               'mola_initial_map_sm_file':
                  Can be used to provide a keyframes map '.simplemap' file to be loaded as initial map. Refer to online tutorials.
                  (default: '""')

               'mola_footprint_to_base_link_tf':
                  Can be used to define a custom transformation between base_footprint and base_link. The coordinates are [x, y, z, yaw_deg, pitch_deg, roll_deg].
                  (default: '[0, 0, 0, 0, 0, 0]')

               'enforce_planar_motion':
                  Whether to enforce z, pitch, and roll to be zero.
                  (default: 'False')

               'use_state_estimator':
                  If false, the basic state estimator 'mola::state_estimation_simple::StateEstimationSimple' will be used. If true, 'mola::state_estimation_smoother::StateEstimationSmoother' is used instead.
                  (default: 'False')

               'state_estimator_config_yaml':
                  A YAML file with settings for the state estimator. Absolute path or relative to 'mola-cli-launchs/lidar_odometry_ros2.yaml'
                  (default: PythonExpr(''../state-estimator-params/state-estimation-smoother.yaml' if ' + LaunchConfig('use_state_estimator') + ' else '../state-estimator-params/state-estimation-simple.yaml''))

               'use_rviz':
                  Whether to launch RViz2 with default lidar-odometry.rviz configuration
                  (default: 'True')


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

    The ``lidar3d-default.yaml`` pipeline file defines plenty of :ref:`additional parameters and options <mola_3d_default_pipeline>` that you can explore.
    See also the docs for the :ref:`ROS 2 API <mola_ros2api>` and :ref:`this tutorial <tutorial-mola-lo-map-and-localize>` on how to save and load a map using ROS 2 MOLA-LO nodes.

