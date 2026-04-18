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

    This listing is kept in sync with
    `ros2-lidar-odometry.launch.py <https://github.com/MOLAorg/mola_lidar_odometry/blob/develop/ros2-launchs/ros2-lidar-odometry.launch.py>`_.
    You can always regenerate it locally with:

    .. code-block:: bash

       ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py --show-args

    Arguments (pass as ``<name>:=<value>``):

    * ``enforce_planar_motion`` (default ``False``):
      Whether to enforce z, pitch, and roll to be zero.

    * ``estimate_geo_reference`` (default ``""``) *[Smoother only]*:
      Whether to estimate the best geo-referencing for ``{enu} -> {map}`` from incoming
      GNSS readings. If empty (default), the pipeline YAML fallback is used (``false``)
      and ``gnss_mode:=live_georef`` may flip it to ``true`` automatically.

    * ``forward_ros_tf_odom_to_mola`` (default ``False``):
      Whether to import an existing ``/tf`` ``odom → base_link`` odometry (2D
      ``CObservationOdometry``). Mutually exclusive with ``odom_topic_name``.

    * ``generate_simplemap`` (default ``False``):
      Whether to create a ``.simplemap``.

    * ``gnss_mode`` (default ``none``):
      High-level GNSS usage: ``none | log_only | live_georef | relocalize``.
      ``live_georef`` and ``relocalize`` require ``use_state_estimator:=True``.

    * ``gnss_topic_name`` (default ``gps``):
      Topic name to listen for ``NavSatFix`` input from a GNSS (e.g. ``/gps``).

    * ``gpsfix_topic_name`` (default ``gpsfix``):
      Topic name to listen for ``gps_msgs/GPSFix`` input from a GNSS (e.g. ``/gpsfix``).

    * ``ignore_imu_pose_from_tf`` (default ``false``):
      If ``true``, the IMU pose is assumed to be at the origin (``base_link``). Leave
      ``false`` if you want to read the actual sensor pose from ``/tf``.

    * ``ignore_lidar_pose_from_tf`` (default ``false``):
      If ``true``, the LiDAR pose is assumed to be at the origin (``base_link``).
      Leave ``false`` if you want to read the actual sensor pose from ``/tf``.

    * ``imu_gravity_avg_samples`` (default ``20``):
      Number of IMU samples to average when estimating the gravity direction for
      pitch/roll correction.

    * ``imu_gravity_correction`` (default ``true``):
      Whether to use IMU accelerometer readings to constrain ICP pitch/roll (prevents
      vertical drift; safe to leave enabled even without an IMU).

    * ``imu_gravity_max_age`` (default ``2.0``):
      Maximum age [seconds] of IMU samples used for gravity alignment. Older samples
      are discarded.

    * ``imu_gravity_sigma_deg`` (default ``2.0``):
      Sigma [degrees] for the gravity-derived pitch/roll prior. Lower = more trust in IMU.

    * ``imu_topic_name`` (default ``imu``):
      Topic name to listen for ``Imu`` input (e.g. ``/imu``).

    * ``initial_localization_method`` (default ``""``):
      Initial-localization method. Options: ``InitLocalization::FixedPose`` (start at
      identity or given pose), ``InitLocalization::FromStateEstimator`` (wait for
      smoother convergence, e.g. from GNSS), ``InitLocalization::PitchAndRollFromIMU``
      (use IMU to estimate pitch/roll at startup, assumes sensor stationary). If empty
      (default), the pipeline YAML fallback is used (``FixedPose``) and
      ``gnss_mode:=relocalize`` may switch it to ``FromStateEstimator``.

    * ``lidar_scan_validity_minimum_point_count`` (default ``100``):
      Minimum number of points required in an incoming LiDAR scan for it to be processed;
      scans below this threshold are discarded.

    * ``lidar_topic_name`` (**required**):
      Topic name to listen for LiDAR input, e.g. ``/ouster/points`` for ``PointCloud2``
      or ``/scan`` for ``LaserScan``. See ``lidar_topic_type``.

    * ``lidar_topic_type`` (default ``PointCloud2``):
      The type of LiDAR topic to subscribe to. Options: ``PointCloud2`` or ``LaserScan``.

    * ``mola_bridge_odometry_frame`` (default ``odom``):
      BridgeROS2's odom ``/tf`` frame name (the REP-105 "odom" child or the parent of
      an externally-published odometry TF).

    * ``mola_deskew_method`` (default ``MotionCompensationMethod::Linear``):
      Motion-compensation (deskew) method for LiDAR scans. Options:
      ``MotionCompensationMethod::None``, ``MotionCompensationMethod::Linear``
      (constant-velocity), ``MotionCompensationMethod::IMU`` (requires an IMU topic;
      use ``use_imu_for_lio:=True`` as the higher-level shortcut).

    * ``mola_footprint_to_base_link_tf`` (default ``[0, 0, 0, 0, 0, 0]``):
      Custom transformation between ``base_footprint`` and ``base_link``, as
      ``[x, y, z, yaw_deg, pitch_deg, roll_deg]``.

    * ``mola_initial_map_mm_file`` (default ``""``):
      Optional path to a metric map ``.mm`` file to load as the initial map.

    * ``mola_initial_map_sm_file`` (default ``""``):
      Optional path to a keyframes ``.simplemap`` file to load.

    * ``mola_lo_pipeline`` (default ``../pipelines/lidar3d-default.yaml``):
      The LO pipeline configuration YAML file.

    * ``mola_lo_reference_frame`` (default ``map``):
      Parent ``/tf`` frame of the localization update emitted by MOLA-LO (the
      ``reference_frame`` of its ``LocalizationUpdate``; see ROS 2 API docs on
      published ``/tf``).

    * ``mola_state_estimator_reference_frame`` (default ``map``):
      Parent ``/tf`` frame of the pose updates emitted by the MOLA State Estimator,
      and BridgeROS2's ``reference_frame`` parameter.

    * ``mola_tf_base_link`` (default ``base_link``):
      The ``/tf`` frame name for the robot base link.

    * ``namespace`` (default ``""``):
      Top-level ROS 2 namespace to push the MOLA stack into (together with
      ``use_namespace:=True``).

    * ``odom_sensor_label`` (default ``odom_wheels``):
      ``sensorLabel`` attached to observations from ``odom_topic_name``. Use distinct
      labels per source when fusing multiple external odometries.

    * ``odom_topic_name`` (default ``""``):
      If non-empty, BridgeROS2 subscribes directly to this ``nav_msgs/Odometry`` topic
      and forwards each message as a 3D ``CObservationRobotPose`` (6×6 covariance) —
      preferred for smoother fusion. Mutually exclusive with ``forward_ros_tf_odom_to_mola``.

    * ``publish_localization_following_rep105`` (default ``True``):
      Whether the bridge publishes localization TFs as ``map → odom`` (REP-105, true)
      or directly ``map → base_link`` (false). REP-105 is incompatible with the smoother.

    * ``start_active`` (default ``True``):
      Whether MOLA-LO starts active (processing incoming sensor data) or idle.

    * ``start_mapping_enabled`` (default ``True``):
      Whether MOLA-LO starts with map update enabled, or in localization-only mode.

    * ``state_estimator_config_yaml`` (default ``""``):
      Path to estimator YAML. If empty, it is auto-resolved based on ``use_state_estimator``.

    * ``use_imu_for_lio`` (default ``False``):
      If ``true``, enables LIO mode (forces ``MotionCompensationMethod::IMU`` for deskew).
      Requires a working ``imu_topic_name``.

    * ``use_mola_gui`` (default ``True``):
      Whether to open the MolaViz GUI for live mapping visualization and control.

    * ``use_namespace`` (default ``false``):
      Whether to apply ``namespace`` to the MOLA stack (remaps ``/tf`` and ``/tf_static``).

    * ``use_rviz`` (default ``True``):
      Whether to launch RViz2 with the default ``lidar-odometry.rviz`` configuration.

    * ``use_state_estimator`` (default ``False``):
      If ``true``, uses ``StateEstimationSmoother`` (requires the optional
      ``mola_state_estimation_smoother`` package).

    **Smoother-only arguments** (applied only when ``use_state_estimator:=True``):

    * ``navstate_kinematic_model`` (default ``KinematicModel::ConstantVelocity``):
      Kinematic model for internal motion-model factors. Options:
      ``KinematicModel::ConstantVelocity``, ``KinematicModel::Tricycle``.

    * ``navstate_sigma_random_walk_angacc`` (default ``10.0``):
      Random-walk angular acceleration uncertainty [rad/s²].

    * ``navstate_sigma_random_walk_linacc`` (default ``1.0``):
      Random-walk linear acceleration uncertainty [m/s²].

    * ``navstate_sliding_window_sec`` (default ``2.5``):
      Time window [seconds] to keep past observations in the filter.

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

