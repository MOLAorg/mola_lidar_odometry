.. _mola_lo_pipelines:

============================
LO/LIO pipelines
============================

____________________________________________

.. contents::
   :depth: 1
   :local:
   :backlinks: none

____________________________________________


Most :ref:`parts <mola-internal-arch>` of the MOLA-LO system are **configured dynamically** from a YAML file.
Basically, the whole design about how many **local map layers** exist, the pointcloud **processing pipelines**,
**ICP matchers and optimizers**, etc. can be changed from this YAML file, without the need to touch the code or recompile.
Users can design new systems by learning how to modify the provided pipeline files.

The best way to understand the different parts of this file is to browse the YAML file of :ref:`the default GICP pipeline <mola_3d_gicp_pipeline>`
provided for 3D LiDARs. Most of the times, comments in the YAML are self-explanatory.
In case of doubts, do not hesitate in `opening an issue <https://github.com/MOLAorg/mola/issues>`_ to ask.

.. note::

   This page also enumerates all **environment variables** that can be defined to modify the behavior of the pipelines.

.. dropdown:: MOLA-specific YAML extensions
    :icon: light-bulb

    MOLA-LO uses the C++ library ``mola_yaml`` to parse YAML files, hence all YAML language extensions defined there
    applies to input YAML files used anywhere in a MOLA-LO system, e.g. ``${VAR|default}`` means "replace by environment
    variable ``VAR`` or, if it does not exist, by ``default``". Read all about :ref:`MOLA YAML extensions <yaml_extensions>`.

.. dropdown:: Specifying the pipeline file in MOLA-LO apps
   :icon: checklist

   All MOLA-LO :ref:`GUI applications <mola_lo_apps>` defaults to using the symlink `lidar3d-default.yaml` which at presents
   points to the :ref:`3D LiDAR GICP pipeline <mola_3d_gicp_pipeline>`
   defined below. To use the alternative 2D pipeline or any other custom pipeline, please set the corresponding environment
   variable before invoking the :ref:`GUI application <mola_lo_apps>` (or derive your own script by copying and modifying the provided ones).
   For example:

   .. code-block:: bash

      # Example using the 3D-NDT alternative pipeline:
      PIPELINE_YAML=$(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/pipelines/lidar3d-ndt.yaml \
      MOLA_LOCAL_VOXELMAP_RESOLUTION=5.0 \
      mola-lo-gui-rosbag  # [...]

   If you use the `CLI interface <mola_lidar_odometry_cli>`_ instead, the pipeline file to use needs to be always explicitly
   specified, there is none by default.


|

____________________________________________

|

.. _mola_icp_pipelines_summary:
Summary of ICP pipelines
~~~~~~~~~~~~~~~~~~~~~~~~

The table below summarizes the optimization algorithm and local-map types used by the ICP pipelines in MOLA-LO.
When saving a map to a ``*.mm`` file, the corresponding C++ class will match the “Local map type” column.
Remember that the accompanying ``.simplemap`` file will always contain a keyframe-map representation
independent of the local map type used for ICP.

To use a prepared ``.mm`` map for localization, ensure it contains a layer named ``localmap`` with the **exact C++ class** listed below.

+-----------------------+---------------------------------------------------------+---------------------------------------------+
| Pipeline config file  | Local map type and layer name                           | ICP algorithm                               |
+=======================+=========================================================+=============================================+
| ``lidar3d-gicp.yaml`` | Keyframe-based 3D point clouds (layer: ``localmap``)    | Generalized ICP (“cov-to-cov”)              |
|    (Default)          | (:ref:`doxid-classmola_1_1_keyframe_point_cloud_map`)   |                                             |
+-----------------------+---------------------------------------------------------+---------------------------------------------+
| ``lidar3d-icp.yaml``  | Voxel-based 3D point clouds (layer: ``localmap``)       | Standard ICP (point-to-point)               |
|                       | (:ref:`doxid-classmola_1_1_hashed_voxel_point_cloud`)   |                                             |
+-----------------------+---------------------------------------------------------+---------------------------------------------+
| ``lidar3d-ndt.yaml``  | Voxel-based 3D NDT map (layer: ``localmap``)            | Standard ICP (point-to-plane, point-to-     |
|                       | (:ref:`doxid-classmola_1_1_n_d_t`)                      | point)                                      |
+-----------------------+---------------------------------------------------------+---------------------------------------------+
| ``lidar2d.yaml``      | Voxel-based 2D occupancy map (layer: ``localmap``)      | Standard ICP (point-to-occupied-voxel)      |
|                       | (``mrpt::maps::CVoxelMap``)                             |                                             |
+-----------------------+---------------------------------------------------------+---------------------------------------------+

____________________________________________

|

.. _mola_3d_gicp_pipeline:

1. Generalized ICP (GICP) pipeline for 3D LiDAR (``lidar3d-gicp.yaml``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
As of Oct 2025, this is the **default and recommended configuration** for most common sensor setups and environments.

A peer-reviewed paper describing this pipeline is in preparation, but very briefly, this pipeline defines a
local map based on key-frames of point clouds, and exploits the Generalized ICP algorithm :cite:`segal2009gicp`.

.. image:: https://mrpt.github.io/imgs/MOLA_LIO_Oxford_Spires_stairs.gif


.. note::

   This pipeline can be LO (default) or LIO. To **actually employ IMU data**, you must enable the ROS 2 launch file argument:

   .. code-block:: bash

      ros2 launch [...] \
        mola_deskew_method:=MotionCompensationMethod::IMU \
        [...]

   or, if using MOLA from the CLI: 

   .. code-block:: bash

      MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
      mola-lo-gui-rosbag2 [...]


.. note::

   See: :ref:`pipelines_env_vars`

.. dropdown:: YAML listing
    :icon: code-review

    File: `mola_lidar_odometry/pipelines/lidar3d-gicp.yaml <https://github.com/MOLAorg/mola_lidar_odometry/blob/develop/pipelines/lidar3d-gicp.yaml>`_

    .. literalinclude:: ../../../mola_lidar_odometry/pipelines/lidar3d-gicp.yaml
       :language: yaml

|

____________________________________________

|

.. _mola_3d_icp_pipeline:

2. ICP pipeline for 3D LiDAR (``lidar3d-icp.yaml``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
This was the **reference configuration** used for most examples in the MOLA-LO paper :cite:`blanco2025mola_lo`,
and should work great out of the box for most common situations; although as of Oct 2025, the newer pipeline :ref:`lidar3d-gicp.yaml <mola_3d_gicp_pipeline>`
is now the default and recommended in general.

As described in the paper :cite:`blanco2025mola_lo`, this pipeline defines a **voxel-based 3D point-cloud local map**,
and filtering pipelines to **downsample** incoming raw LiDAR data.

.. image:: https://mrpt.github.io/imgs/mola-slam-kitti-demo.gif

.. note::

   See: :ref:`pipelines_env_vars`

.. dropdown:: YAML listing
    :icon: code-review

    File: `mola_lidar_odometry/pipelines/lidar3d-default.yaml <https://github.com/MOLAorg/mola_lidar_odometry/blob/develop/pipelines/lidar3d-default.yaml>`_

    .. literalinclude:: ../../../mola_lidar_odometry/pipelines/lidar3d-default.yaml
       :language: yaml


|

____________________________________________

|


.. _mola_3d_ndt_pipeline:

3. 3D mapping pipeline using 3D-NDT (``lidar3d-ndt.yaml``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
This is an **alternative configuration** for 3D mapping used in the MOLA-LO paper, and should also work great
out of the box for most common situations where, *at least*, part of the environment has flat surfaces.

As described in the paper :cite:`blanco2025mola_lo`, this pipeline uses an NDT-like :cite:`magnusson2007scan` local map, based on **3D voxels**
whose contents switch between bare points and Gaussians depending on how planar and how many points are.
This pipeline exploits the **point-to-plane pairings**.

.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls autoplay loop muted style="width: 100%;">
       <source src="https://mrpt.github.io/videos/mola-slam-mulran-demo-ndt.mp4" type="video/mp4">
     </video>
   </div>

.. note::

   See: :ref:`pipelines_env_vars`

.. dropdown:: YAML listing
    :icon: code-review

    File: `mola_lidar_odometry/pipelines/lidar3d-ndt.yaml <https://github.com/MOLAorg/mola_lidar_odometry/blob/develop/pipelines/lidar3d-ndt.yaml>`_

    .. literalinclude:: ../../../mola_lidar_odometry/pipelines/lidar3d-ndt.yaml
       :language: yaml


|
____________________________________________

|


4. Pipeline for 2D LiDAR (``lidar2d.yaml``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
This alternative configuration uses an **occupancy voxel map** instead of point clouds
as local map, and performs **ray-tracing** to accumulate evidence about the freeness
or occupancy of voxels from 2D LiDAR scans.
If it recommended to use wheels-based odometry to help the mapping process.

.. image:: https://mrpt.github.io/imgs/lidar2d-radish-demo.gif

.. note::

   See: :ref:`pipelines_env_vars`

.. dropdown:: YAML listing
    :icon: code-review

    File: `mola_lidar_odometry/pipelines/lidar2d.yaml <https://github.com/MOLAorg/mola_lidar_odometry/blob/develop/pipelines/lidar2d.yaml>`_

    .. literalinclude:: ../../../mola_lidar_odometry/pipelines/lidar2d.yaml
       :language: yaml


|

____________________________________________

|

.. _pipelines_env_vars:

Configuring pipelines via environment variables
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

All the following environment variables can be set with ``export VAR=VALUE`` before
invoking any of the MOLA-LO programs (cli, gui, or ROS node), or directly as prefixes
to the invocation line, e.g. ``VAR1=VALUE1 VAR2=VALUE2 mola-xxx``.

Unless said otherwise, all variables are valid for all the pipelines described above.

.. note::

   If using MOLA-LO via mola-cli (which includes the GUI applications or the ROS 2 interface),
   there are additional environment variables to tune each particular
   `mola-cli launch file <https://github.com/MOLAorg/mola_lidar_odometry/tree/develop/mola-cli-launchs>`_.
   Those variables are documented :ref:`here <mola-gui-apps-common-env-vars>`.

.. _mola_lo_pipeline_sensor_inputs:

Sensor inputs: LiDAR
^^^^^^^^^^^^^^^^^^^^^

.. dropdown:: Overriding the LiDAR sensor pose
   :icon: checklist

   To manually override the sensor pose on the vehicle/robot, see also :ref:`these environment variables <mola_lo_ros_mola-cli-env-vars>`,
   or the corresponding :ref:`ROS2 launch arguments <mola_lo_ros_launch_arguments>`.


- ``MOLA_LIDAR_NAME`` (Default: ``['lidar', '/ouster/points']``): A **sensor label** (maybe including a regular expression) of what
  observations are to be treated as input LiDAR point clouds. For most dataset sources, the default ``lidar`` is enough.
  For ROS bags or live ROS 2 as sources, the default behavior is assigning **sensor labels** exactly the same than
  incoming **ROS topic names**, so **set this to your ROS 2 topic name for the LiDAR**, but in principle both are different things.
  Read carefully the contents of the `mola-cli launch files <https://github.com/MOLAorg/mola_lidar_odometry/tree/develop/mola-cli-launchs>`_
  and the comments therein to understand the differences.

- ``MOLA_LIDAR_COUNT`` (Default: ``1``): Useful only if using several lidar_sensor_labels or regex's. Can be used to
  work with vehicles with two or more LiDARs.

- ``MOLA_LIDAR_MAX_TIME_OFFSET`` (Default: ``0.1`` [s]): Maximum delay between different LiDAR observations to handle them together.
  Note that deskewing takes into account the exact delays between clouds from different LiDARs.

- ``MOLA_ABS_MIN_SENSOR_RANGE`` (Default: ``5.0`` [m]): Absolute minimum for the otherwise
  automatically detected observation radius (see ``ESTIMATED_OBSERVATION_RADIUS``).

- ``MOLA_MINIMUM_RANGE_FILTER`` (Default: 3% of the estimated observation radius): Minimum range
  (L-infinity cube around ``base_link``) for 3D points used by ICP; intended to cut out points coming from
  the robot/vehicle body itself or a person standing right next to it.

Sensor inputs: IMU (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. dropdown:: Overriding the IMU sensor pose
   :icon: checklist

   To manually override the sensor pose on the vehicle/robot, see also :ref:`these environment variables <mola_lo_ros_mola-cli-env-vars>`,
   or the corresponding :ref:`ROS2 launch arguments <mola_lo_ros_launch_arguments>`.

- ``MOLA_IMU_NAME`` (Default: ``imu``): **Sensor label** (or regex) of the observations with IMU data, if it exists.
  This is used to estimate the vehicle's pose and velocity, and to deskew point clouds.
  For most dataset sources, the default ``imu`` is enough.
  For ROS bags or live ROS 2 as sources, the default behavior is assigning **sensor labels** exactly the same than
  incoming **ROS topic names**, so **set this to your ROS 2 topic name for the IMU**, but in principle both are different things.
  Read carefully the contents of the `mola-cli launch files <https://github.com/MOLAorg/mola_lidar_odometry/tree/develop/mola-cli-launchs>`_
  and the comments therein to understand the differences.

- ``MOLA_DESKEW_IGNORE_ACCELEROMETER`` (Default: ``false``): Enable if a noisy IMU sensor causes shaky motion in the estimation,
  but you still want to use gyroscope for precise deskewing in LIO.


IMU gravity correction (pitch/roll)
"""""""""""""""""""""""""""""""""""""

When an IMU is available, MOLA-LO can use the accelerometer readings to
continuously estimate the gravity direction and inject it as a **pitch/roll
constraint** into the ICP prior. This prevents vertical drift in the LiDAR
odometry output without requiring a full factor-graph smoother.

The feature averages recent accelerometer samples in a circular buffer,
rotates the result to the vehicle frame using the IMU ``sensorPose``
extrinsics, and derives pitch and roll from the gravity direction.

.. note::

   This feature acts on the ICP prior independently of the
   ``StateEstimationSmoother`` gravity factor. Both can be active simultaneously.

- ``MOLA_IMU_GRAVITY_CORRECTION`` (Default: ``true``): Set to ``false`` to disable
  accelerometer-based pitch/roll correction of the ICP prior. Safe to leave
  enabled even without an IMU: when no IMU data is present the correction is
  silently skipped.

- ``MOLA_IMU_GRAVITY_SIGMA_DEG`` (Default: ``2.0`` [deg]): Standard deviation of the
  gravity-derived pitch/roll prior. Lower values give more trust to the IMU.
  Typical range: 1-5 deg.

- ``MOLA_IMU_GRAVITY_AVG_SAMPLES`` (Default: ``20``): Number of recent accelerometer
  samples to average for the gravity estimate. The correction activates as soon as
  3 samples are available, even if the full window has not filled yet.

- ``MOLA_IMU_GRAVITY_MAX_AGE`` (Default: ``2.0`` [s]): Maximum age in seconds for
  accelerometer samples used in the gravity average. Samples older than this are
  discarded, ensuring the estimate reflects current orientation rather than stale
  data. Set to ``0`` to disable age filtering.


Sensor inputs: Wheels odometry (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- ``MOLA_ODOMETRY_NAME`` (Default: ``wheel_odom``): **Sensor label** (or regex) of the observations
  with wheels odometry, if it exists.

Sensor inputs: GPS (GNSS) (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. dropdown:: Overriding the GNSS/GPS sensor pose
   :icon: checklist

   To manually override the sensor pose on the vehicle/robot, see also :ref:`these environment variables <mola_lo_ros_mola-cli-env-vars>`,
   or the corresponding :ref:`ROS2 launch arguments <mola_lo_ros_launch_arguments>`.


- ``MOLA_GPS_NAME`` (Default: ``gps``): **Sensor label** (or regex) of GNSS observations inside the pipeline.
  Used only for storage in simple-maps for post-processing (geo-referencing, etc.).

  .. note::

     For ROS 2 live or rosbag sources, the ROS topic name that the bridge subscribes to is controlled separately
     by ``MOLA_GNSS_TOPIC`` (see :ref:`mola_lo_ros_mola-cli-env-vars`). These are two different variables:
     ``MOLA_GPS_NAME`` is the internal sensor label; ``MOLA_GNSS_TOPIC`` is the ROS topic.


Scan de-skew options
^^^^^^^^^^^^^^^^^^^^^^

- ``MOLA_DESKEW_METHOD`` (Default: ``MotionCompensationMethod::Linear``): Selects the scan de-skew (motion compensation) method.

  .. note::

     **IMPORTANT**: If you do not change this from its default, IMU data will not be used for deskewing.
     To fully achieve the best accuracy when an IMU is available, set this to ``MotionCompensationMethod::IMU``.

- ``MOLA_IGNORE_NO_POINT_STAMPS`` (Default: ``true``): If enabled (default), input point clouds without per-point timestamps
  will be processed without doing any de-skew. If set to ``false``, an exception is triggered in that event,
  which can be used as a fail-safe check against missing stamps, important in high-velocity scenarios.

- ``MOLA_SCAN_POINT_STAMPS_ADJUST_METHOD`` (Default: ``TimestampAdjustMethod::MiddleIsZero``): Method for adjusting
  per-point timestamps so that the scan mid-point is at time zero. Affects how twist is applied during deskewing.


General options
^^^^^^^^^^^^^^^^^^^^^^

- ``MOLA_OPTIMIZE_TWIST`` (Default: ``true`` for most pipelines, hardcoded ``false`` in the GICP pipeline):
  Whether to optimize vehicle twist (linear and angular velocity vectors) within the ICP loop.
  Useful for high-dynamics scenarios. Requires incoming point clouds with per-point timestamps.
  In the GICP pipeline the early-deskew pass takes priority; use ``lidar3d-gicp-optimize-twist.yaml``
  to re-enable it.

- ``MOLA_MAPPING_ENABLED`` (Default: ``true``): Whether to update the local map. Can be temporarily disabled,
  or permanently disabled when using MOLA-LO for localization from a prebuilt map.

- ``MOLA_LOAD_MM`` (Default: none): Path to a metric map (``*.mm``) file with a prebuilt metric map to load at startup.
  Useful for multisession mapping or localization-only mode.

- ``MOLA_SAVE_MM`` (Default: none): If set to a non-empty path, the final local metric map is saved to a ``*.mm`` file
  at the end of the session.

- ``MOLA_MINIMUM_ICP_QUALITY`` (Default: ``0.50``): Minimum quality (from ``mp2p_icp`` quality evaluators), in the
  range [0, 1], to consider an ICP optimization valid.

- ``MOLA_WRITE_DEBUG_ICP_LOG_IF_QUALITY_UNDER`` (Default: none): If set to a value in [0, 1], ``.icplog`` debug files
  are saved whenever ICP quality drops below that threshold, independently of ``MP2P_ICP_GENERATE_DEBUG_FILES``.
  Useful for targeted debugging of bad frames without enabling full logging.

- ``MOLA_START_ACTIVE`` (Default: ``true``): If set to ``false``, the odometry pipeline will ignore incoming observations
  until active is set to ``true`` (e.g. via the GUI).

- ``MOLA_PROFILER`` (Default: ``true``): Enable pipeline and ICP step-level timing profiler. Disable to reduce overhead
  in production deployments.

- ``MOLA_LO_PUBLISH_REF_FRAME`` (Default: ``odom``): Reference frame name used when publishing pose updates.

- ``MOLA_LO_PUBLISH_VEHICLE_FRAME`` (Default: ``base_link``): Vehicle frame name used when publishing pose updates.

- ``MOLA_LO_PUBLISH_DESKEWED_SCANS`` (Default: ``false``): If enabled, deskewed scans are published as ROS 2 messages,
  mostly for visualization. May slow down the system.


Adaptive threshold
^^^^^^^^^^^^^^^^^^^^^^

The adaptive threshold controls the ICP matching window (sigma), following the approach of KISS-ICP.

- ``MOLA_SIGMA_INITIAL`` (Default: ``0.50`` [m]): Initial sigma value at startup.

- ``MOLA_SIGMA_MIN_MOTION`` (Default: ``0.04`` [m]): Absolute minimum for sigma.

- ``MOLA_SIGMA_MAX_MOTION`` (Default: ``0.50`` [m]; GICP and ICP pipelines): Upper cap for sigma.
  Named ``MOLA_SIGMA_MAX`` in the NDT pipeline with a default of ``0.5`` [m].

- ``MOLA_ADAPT_THRESHOLD_ALPHA`` (Default: ``0.90``): Alpha parameter of the IIR low-pass filter for the adaptive
  threshold proportional controller (refer to the paper).

- ``MOLA_ADAPT_THRESHOLD_RECOVER`` (Default: ``false``): If enabled, sigma is grown multiplicatively after a streak
  of bad ICP results, up to ``MOLA_SIGMA_MAX_MOTION``, allowing the matcher window to re-open and ICP to recover
  from sustained failure.

- ``MOLA_ADAPT_THRESHOLD_RECOVER_AFTER_N_BAD`` (Default: ``5``): Number of consecutive bad ICP results before
  recovery growth kicks in, when ``MOLA_ADAPT_THRESHOLD_RECOVER`` is enabled.

- ``MOLA_ADAPT_THRESHOLD_RECOVER_GROWTH_FACTOR`` (Default: ``1.5``): Multiplicative growth factor applied to sigma
  per bad frame during recovery.


Local map update
^^^^^^^^^^^^^^^^^^^^^^

- ``MOLA_MIN_XYZ_BETWEEN_MAP_UPDATES`` (Default: heuristic formula, see YAML file): Minimum distance in meters between
  updates to the local map.

- ``MOLA_MIN_ROT_BETWEEN_MAP_UPDATES`` (In degrees. Default: heuristic formula, see YAML file): Minimum angle in degrees
  between updates to the local map.

- ``MOLA_LOCAL_MAP_MAX_SIZE`` (In meters; default: heuristic formula, see YAML file): Parts of the local metric map farther
  away than this distance, measured from the current robot pose, will be removed. This saves memory and avoids
  inconsistencies before loop-closure (which is handled outside of the LO module).

- ``MOLA_LOCAL_VOXELMAP_RESOLUTION`` (In meters; default: heuristic formula, see YAML file): Size of voxels for the local map.
  **Not used in the GICP pipeline**, which uses a keyframe-based map instead.

- ``MOLA_MIN_NEARBY_POSES_OCCUPIED`` (Default: ``1``): Minimum number of nearby local-map poses that must be occupied
  before a new keyframe is accepted into the map.

- ``MOLA_PUBLISH_LOCAL_MAP_UPDATES_EVERY_N`` (Default: ``40`` in GICP, ``5`` in NDT): Publish local map visualization
  updates every N ICP iterations.


Observation filter pipeline
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- ``MOLA_LO_OBS_PREFILTER_PIPELINE_FILE`` (Default: none): Path to an optional user-defined ``mp2p_icp`` pipeline YAML
  file applied to raw observations before any built-in filtering. Use this to inject custom filters (e.g. a ring-based
  ground filter or a sector masking filter) without modifying the main pipeline file.

- ``MOLA_CLOUD_DECIMATION_VOXEL_SIZE`` (Default: ``0.15`` [m] for map, ``0.10`` [m] for ICP; GICP pipeline only):
  Minimum voxel size for adaptive decimation of point clouds. The GICP pipeline uses two separate decimation stages;
  this sets the floor voxel size for both. See also ``MOLA_DECIMATED_POINTS_MAP`` and ``MOLA_DECIMATED_POINTS_ICP``.

- ``MOLA_DECIMATED_POINTS_MAP`` (Default: ``10000``; GICP pipeline only): Target point count for the cloud inserted
  into the local map after adaptive decimation.

- ``MOLA_DECIMATED_POINTS_ICP`` (Default: ``3000``; GICP pipeline only): Target point count for the cloud used in
  ICP matching after adaptive decimation.


ICP settings
^^^^^^^^^^^^^^^^^^^^^^

- ``MOLA_MAX_ICP_ITERATIONS`` (Default: ``25``): Maximum number of ICP iterations per scan.

- ``MOLA_LOCALMAP_LAYER_NAME`` (Default: ``localmap``; GICP and ICP pipelines): Name of the metric map layer used
  as the local map for ICP matching and map insertion.

- ``MOLA_LO_ROBUST_KERNEL`` (Default: ``RobustKernel::GemanMcClure``): Robust kernel type used in the ICP
  Gauss-Newton solver.

- ``MOLA_LO_ROBUST_KERNEL_PARAM`` (Default: ``6.0``): Parameter for the robust kernel (scale; in normalized
  covariance units for the GICP pipeline).

- ``MOLA_LO_ROBUST_KERNEL_PRIOR_REF_BLEND`` (Default: ``0.0``): Blend factor in [0, 1] for the residual
  reference used by the robust kernel in the Gauss-Newton solver. ``0.0`` (default) keeps the classic
  behavior, where each factor is judged only by how much it diverges from the current linearization point.
  With values ``>0`` the kernel residual is blended toward the value predicted at the prior mean pose
  (e.g. the motion model / IMU prior), so correspondences inconsistent with the prior are down-weighted even
  when the current iterate is already corrupted. Has no effect when no prior is supplied to ICP.

- ``MOLA_ICP_COVARIANCE_METHOD`` (Default: ``Censi3D``; GICP pipeline only): Post-optimization SE(3) covariance
  estimation method. ``Censi3D`` is the sandwich estimator suited for cov-to-cov pipelines.

- ``MOLA_ICP_COV_DEFAULT_POINT_SIGMA`` (Default: ``0.01`` [m]; GICP pipeline only): Per-point sigma used in
  covariance estimation.

- ``MOLA_ICP_COV_FLOOR_XYZ`` (Default: ``0.001`` [m]; GICP pipeline only): Floor on the XYZ diagonal of the output
  covariance, to keep downstream filters numerically stable.

- ``MOLA_ICP_COV_FLOOR_ANGLES_DEG`` (Default: ``0.1`` [deg]; GICP pipeline only): Floor on the angular diagonal
  of the output covariance.


GICP local map parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following variables tune the ``mola::KeyframePointCloudMap`` used as local map in the GICP pipeline.
They have no effect on the ICP or NDT pipelines.

- ``MOLA_LOCALMAP_MAX_SEARCH_KEYFRAMES`` (Default: ``3``): Maximum number of keyframes searched for correspondences
  per ICP step.

- ``MOLA_LOCALMAP_K_CORRESPONDENCES_FOR_COV`` (Default: ``20``): Number of nearest neighbors used to estimate
  per-point covariance in the local map.

- ``MOLA_LOCALMAP_USE_VIEW_DIRECTION_FILTER`` (Default: ``true``): Enable filtering of candidate keyframes by
  view direction, to avoid matching from very different angles.

- ``MOLA_LOCALMAP_VIEW_DIRECTION_FILTER_ANGLE_DEG`` (Default: ``120`` [deg]): Maximum angular difference allowed
  between the current view direction and a candidate keyframe to be considered for matching.

- ``MOLA_LOCALMAP_DIVERSE_KEYFRAMES`` (Default: ``1``): Number of diverse keyframes to force into the candidate set
  regardless of proximity. Must be less than ``MOLA_LOCALMAP_MAX_SEARCH_KEYFRAMES``.

- ``MOLA_LOCALMAP_VIZ_MAX_POINTS_PER_KF`` (Default: ``10000``): Maximum points rendered per keyframe in the GUI.

- ``MOLA_LOCALMAP_VIZ_MAX_POINTS_OVERALL`` (Default: ``500000``): Maximum total points rendered for the local map
  in the GUI (e.g. to avoid FoxGlove WebSocket overflow).


Simple-map generation
^^^^^^^^^^^^^^^^^^^^^^^^

- ``MOLA_GENERATE_SIMPLEMAP`` (Default: ``false``): If enabled, a simple-map will be saved at the end of the mapping
  session. This can then be used as input to any of the ``mp2p_icp`` applications.

- ``MOLA_LOAD_SM`` (Default: none): If set, loads an existing simple-map file at startup and continues appending
  keyframes to it.

- ``MOLA_SIMPLEMAP_OUTPUT`` (Default: ``final_map.simplemap``): Output file name for simple-maps.

- ``MOLA_SIMPLEMAP_MIN_XYZ`` (in meters), ``MOLA_SIMPLEMAP_MIN_ROT`` (in degrees): Minimum distance between
  simple-map keyframes. Useful to control the density of generated simple-maps. Defaults are heuristic formulas.

- ``MOLA_SIMPLEMAP_MIN_NEARBY_POSES`` (Default: ``1``): Minimum number of nearby poses occupied before a new
  keyframe is accepted into the simple-map.

- ``MOLA_SIMPLEMAP_GENERATE_LAZY_LOAD`` (Default: ``false``): If enabled, generated simple-map files will be much
  smaller since all heavy observations will be stored in external files, making those maps faster to process later.

- ``MOLA_SIMPLEMAP_ALSO_NON_KEYFRAMES`` (Default: ``false``): If enabled, all LiDAR observations will generate a
  KeyFrame in the simple-map, but without raw sensory data if the keyframe does not fulfill the minimum distance
  criteria. Useful to generate, in post-processing, the full vehicle trajectory without missing any timestep.

- ``MOLA_SAVE_DESKEWED_SCANS`` (Default: ``false``): If enabled, deskewed (motion-compensated) scans are stored
  into simple-map keyframes instead of raw scans. Useful when post-processing requires already-compensated clouds.


Trajectory files generation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- ``MOLA_SAVE_TRAJECTORY`` (Default: ``false``): If enabled, a TUM file will be saved at the end with the full
  vehicle trajectory.

- ``MOLA_TUM_TRAJECTORY_OUTPUT`` (Default: ``estimated_trajectory.tum``): Output file name for the TUM trajectory.


Observation validity filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- ``MOLA_ENABLE_OBS_VALIDITY_FILTER`` (Default: ``false``): Enables a pre-filter that discards incoming scans that
  appear incomplete, e.g. due to faulty network connections or missing UDP packets.

- ``MOLA_OBS_VALIDITY_MIN_POINTS`` (Default: ``1000``): Minimum number of points a scan must contain to be accepted
  when ``MOLA_ENABLE_OBS_VALIDITY_FILTER`` is enabled.


Initial localization
^^^^^^^^^^^^^^^^^^^^^^

- ``MOLA_LO_INITIAL_LOCALIZATION_METHOD`` (Default: ``InitLocalization::FixedPose``): Strategy used to determine the
  initial pose at startup. Options:

  - ``InitLocalization::FixedPose``: Use a fixed pose defined by ``MOLA_INITIAL_*`` below.
  - ``InitLocalization::IMUCalibration``: Collect IMU samples to determine initial orientation.
  - ``InitLocalization::FromStateEstimator``: Wait for an external state estimator to converge.

- ``MOLA_INITIAL_X``, ``MOLA_INITIAL_Y``, ``MOLA_INITIAL_Z`` (Default: ``0.0`` [m]): Initial position when using
  ``InitLocalization::FixedPose``.

- ``MOLA_INITIAL_YAW``, ``MOLA_INITIAL_PITCH``, ``MOLA_INITIAL_ROLL`` (Default: ``0.0`` [deg]): Initial orientation
  when using ``InitLocalization::FixedPose``.

- ``MOLA_LO_INITIAL_IMU_SAMPLES`` (Default: ``400``): Number of IMU samples to collect for initial orientation
  calibration when using ``InitLocalization::IMUCalibration``.

- ``MOLA_LO_INITIAL_IMU_USE_ORIENTATION`` (Default: ``true``): Whether to use the IMU orientation quaternion (if
  provided by the driver) directly, instead of computing orientation from accelerometer readings.

- ``MOLA_LO_INIT_SE_MAX_POS_SIGMA`` (Default: ``0.5`` [m]): Maximum position uncertainty accepted from the state
  estimator for ``InitLocalization::FromStateEstimator`` to be considered converged.

- ``MOLA_LO_INIT_SE_MAX_ORI_SIGMA`` (Default: ``3.0`` [deg]): Maximum orientation uncertainty accepted from the
  state estimator for ``InitLocalization::FromStateEstimator``.

- ``MOLA_LO_INIT_SE_TIMEOUT`` (Default: ``60.0`` [s]): Timeout for ``InitLocalization::FromStateEstimator``.
  If the state estimator does not converge within this time, initialization falls back to ``FixedPose``.


Visualization
^^^^^^^^^^^^^^^^^^^

.. note::

   These settings only have effect if launched via :ref:`MOLA-LO GUI applications <mola_lo_apps>`.

- ``MOLA_VEHICLE_MODEL_FILE`` (Default: none): Path to a 3D model file loadable via Assimp (e.g. Collada ``*.dae``)
  with a representation of the vehicle/robot to show in the GUI.

- ``MOLA_VEHICLE_MODEL_X``, ``MOLA_VEHICLE_MODEL_Y``, ``MOLA_VEHICLE_MODEL_Z``, ``MOLA_VEHICLE_MODEL_YAW``,
  ``MOLA_VEHICLE_MODEL_PITCH``, ``MOLA_VEHICLE_MODEL_ROLL`` (Default: ``0``): Transformation applied to the 3D asset
  defined in ``MOLA_VEHICLE_MODEL_FILE``. Translations are in meters, rotations in degrees.

.. dropdown:: Additional GUI visualization variables
   :icon: eye

   - ``MOLA_GUI_SHOW_CURRENT_OBS`` (Default: ``false``): Show live deskewed LiDAR points for the current frame.
   - ``MOLA_GUI_SHOW_DESKEWED_DECAY`` (Default: ``true``): Show deskewed LiDAR points from recent frames, fading over time.
   - ``MOLA_GUI_LAST_CLOUDS_POINT_SIZE`` (Default: ``1.0``): Point size for the recent deskewed cloud visualization.
   - ``MOLA_GUI_LAST_CLOUDS_COLORMAP`` (Default: ``cmJET``): Colormap for recent deskewed clouds (``mrpt::img::TColormap`` name).
   - ``MOLA_GUI_LAST_CLOUDS_COLOR_FIELD`` (Default: ``intensity``): Point field used for colormap (e.g. ``x``, ``y``, ``z``, ``ring``, ``intensity``).
   - ``MOLA_GUI_CLOUDS_DECAY_SECS`` (Default: ``10.0`` [s]): Fade-out duration for the deskewed cloud decay visualization.
   - ``MOLA_GUI_CURRENT_CLOUD_POINT_SIZE`` (Default: ``2.0``): Point size for the current-frame cloud.
   - ``MOLA_GUI_CURRENT_CLOUD_COLORMAP`` (Default: ``cmHOT``): Colormap for the current-frame cloud.
   - ``MOLA_GUI_CURRENT_CLOUD_COLOR_FIELD`` (Default: ``intensity``): Point field used to colorize the current-frame cloud.
   - ``MOLA_GUI_SHOW_ESTIMATED_GRAVITY_VECTOR`` (Default: ``false``): Overlay the estimated gravity direction vector in the 3D view.
   - ``MOLA_GUI_BACKGROUND_GRAY_LEVEL`` (Default: ``0.3``): Background brightness for the 3D view (0=black, 1=white).
   - ``MOLA_GUI_SHOW_LOCAL_MAP`` (Default: ``true``): Whether to render the local map in the GUI.
   - ``MOLA_GUI_SHOW_GROUND_GRID`` (Default: ``true``): Whether to show the ground reference grid in the GUI.
   - ``MOLA_GUI_LOCAL_MAP_COLOR_BY_COORDINATE`` (Default: ``intensity``; GICP pipeline only): Point field used to
     colorize the local map in the GUI (e.g. ``x``, ``y``, ``z``, ``ring``, ``intensity``).


Motion model
^^^^^^^^^^^^^^^^^^^^^^
A constant velocity motion model is used by default, provided by the ``mola_state_estimation_simple`` module.

- ``MOLA_MAX_TIME_TO_USE_VELOCITY_MODEL`` (Default: ``0.75`` [s]): Maximum time between LiDAR frames to use the
  velocity model. Larger delays will cause the latest vehicle pose to be used as the initial guess instead.
- ``MOLA_NAVSTATE_SIGMA_RANDOM_WALK_LINACC`` (Default: ``1.0`` [m/s^2]): Linear acceleration standard deviation.
- ``MOLA_NAVSTATE_SIGMA_RANDOM_WALK_ANGACC`` (Default: ``10.0`` [rad/s^2]): Angular acceleration standard deviation.


.. _pipeline_icp_log_files:

ICP log files
^^^^^^^^^^^^^^^^^^^^^^

- ``MP2P_ICP_GENERATE_DEBUG_FILES`` (Default: ``false``): If enabled, ``mp2p_icp::ICP`` log files will be saved
  into a subdirectory ``icp-logs`` under the current directory. Those logs can be analyzed
  with the GUI tool: :ref:`icp-log-viewer <app_icp-log-viewer>`.

.. note::

   Enabling ICP log files is the most powerful tool to **debug mapping or localization** issues or to understand what
   is going on under the hood. However, **it introduces a significant cost** in both CPU running time and disk space.


If ``MP2P_ICP_GENERATE_DEBUG_FILES`` is not enabled, the rest of parameters that follow have no effect:

- ``MP2P_ICP_LOG_FILES_DECIMATION`` (Default: ``10``): How many ICP runs to drop before saving one to disk.
- ``MP2P_ICP_LOG_FILES_SAVE_DETAILS`` (Default: ``false``): If enabled, results and pairings of **intermediate**
  optimization steps are also stored in the ICP logs. Great to learn how ICP actually works, but increases log file sizes.
- ``MP2P_ICP_LOG_FILES_SAVE_DETAILS_DECIMATION`` (Default: ``3``): If ``MP2P_ICP_LOG_FILES_SAVE_DETAILS`` is enabled,
  how many ICP internal iterations to drop for each saved one.


Trace debug files
^^^^^^^^^^^^^^^^^^^^^^
"Trace" files are optional CSV files with low-level debugging information, sampled once per time step.

- ``MOLA_SAVE_DEBUG_TRACES`` (Default: ``false``): Whether to generate and save this debug information to a file.
- ``MOLA_DEBUG_TRACES_FILE`` (Default: ``mola-lo-traces.csv``): The name of the file to store trace information, if enabled.