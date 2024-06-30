.. _mola_lidar_odometry_cli:

============================
LiDAR odometry CLI
============================

``mola-lidar-odometry-cli`` is a standalone command line program to run
MOLA-LO on a dataset in an offline fashion.
The dataset is processed as fast as possible using all available CPU cores.
Its outputs include the vehicle trajectory (as a file in `TUM format <https://github.com/MichaelGrupp/evo/wiki/Formats#tum---tum-rgb-d-dataset-trajectory-format>`_)
and the :ref:`simple-map <mola-lo-role>`, which can be analyzed with :ref:`sm-cli <app_sm-cli>`
and used to generate metric maps using :ref:`sm2mm <app_sm2mm>`.

.. hint::

    We recommend using `evo <https://github.com/MichaelGrupp/evo>`_ to visualize
    and compare the output TUM trajectories. You can also use
    `mrpt::poses::CPose3DInterpolator <https://docs.mrpt.org/reference/latest/class_mrpt_poses_CPose3DInterpolator.html>`_
    to load and parse TUM files in C++, or its Python wrapped version within ``pymrpt``.


1. Usage examples
-----------------

Process a ROS 2 bag
~~~~~~~~~~~~~~~~~~~~~

    .. code-block:: bash

        mola-lidar-odometry-cli \
          -c $(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/pipelines/lidar3d-default.yaml \
          --input-rosbag2 /PATH/TO/YOUR/rosbag.mcap \
          --lidar-sensor-label /ouster/points \
          --output-tum-path trajectory.tum

.. note::
    Remember changing ``--lidar-sensor-label /ouster/points`` to your actual raw (unfiltered) LiDAR topic (``sensor_msgs/PointCloud2``).

.. dropdown:: Does your bag lack ``/tf``?
    :icon: alert

    By default, ``mola-lidar-odometry-cl`` will try to use ``tf2`` messages in the rosbag to find out the relative pose
    of the LiDAR sensor with respect to the vehicle frame (default: ``base_link``). If your system **does not** have ``tf`` data
    (for example, if you only launched the LiDAR driver node) you must then set the environment variable ``MOLA_USE_FIXED_LIDAR_POSE=true``
    to use the default (identity) sensor pose on the vehicle. So, launch it like: 

    .. code-block:: bash

        MOLA_USE_FIXED_LIDAR_POSE=true \
        mola-lidar-odometry-cli \
          [...]  # the rest does not change.

.. dropdown:: Want to visualize the output in real-time?
    :icon: light-bulb

    ``mola-lidar-odometry-cli`` is explicitly designed to be as fast as possible by not interacting with any GUI or messaging system. 
    If you prefer to visualize the results as they are being processed, there are two options:

    * Use the built-in GUI in the provided apps: :ref:`mola-lo-gui-rosbag2 <mola_lo_apps>`.
    * Replay the bag with `ros2 bag play` and launch the :ref:`ROS 2 launch file <mola_lo_ros>` so you can use RViz2 or FoxGlove for visualization.aunch
.. dropdown:: More parameters
    :icon: list-unordered

    The ``lidar3d-default.yaml`` pipeline file defines plenty of :ref:`additional parameters and options <mola_3d_default_pipeline>` that you can explore.

|


Process a KITTI dataset sequence
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

First, make sure of downloading and extracting the dataset files following the layout
expected by mola::KittiDataset.
Then, set the ``KITTI_BASE_DIR`` environment variable and launch the desired sequence (e.g. ``00``) with:

    .. code-block:: bash

        export KITTI_BASE_DIR=/path/to/kitti_root

        mola-lidar-odometry-cli \
          -c $(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/pipelines/lidar3d-default.yaml \
          --input-kitti-seq 00 \
          --output-tum-path kitti-00.tum

.. dropdown:: More parameters
    :icon: list-unordered

    The ``lidar3d-default.yaml`` pipeline file defines plenty of :ref:`additional parameters and options <mola_3d_default_pipeline>` that you can explore.

|

2. Complete list of arguments
-------------------------------

    .. code-block:: bash

        USAGE:

        mola-lidar-odometry-cli  [--input-paris-luco] [--input-mulran-seq
                                    <KAIST01>] [--input-kitti360-seq <00>]
                                    [--kitti-correction-angle-deg <0.205
                                    [degrees]>] [--input-kitti-seq <00>]
                                    [--input-rosbag2 <dataset.mcap>]
                                    [--input-rawlog <dataset.rawlog>]
                                    [--lidar-sensor-label <lidar1>] [--skip-first-n
                                    <Number of dataset entries to skip>]
                                    [--only-first-n <Number of dataset entries to
                                    run>] [--output-simplemap
                                    <output-map.simplemap>] [--output-tum-path
                                    <output-trajectory.txt>] [-l <foobar.so>] [-v
                                    <INFO>] -c <demo.yml> [--] [--version] [-h]


        Where: 

        --input-paris-luco
            INPUT DATASET: Use Paris Luco dataset (unique sequence=00)

        --input-mulran-seq <KAIST01>
            INPUT DATASET: Use Mulran dataset sequence KAIST01|KAIST01|...

        --input-kitti360-seq <00>
            INPUT DATASET: Use KITTI360 dataset sequence number 00|01|...|test_00
            |...

        --kitti-correction-angle-deg <0.205 [degrees]>
            Correction vertical angle offset (see Deschaud,2018)

        --input-kitti-seq <00>
            INPUT DATASET: Use KITTI dataset sequence number 00|01|...

        --input-rosbag2 <dataset.mcap>
            INPUT DATASET: rosbag2. Input dataset in rosbag2 format (*.mcap)

        --input-rawlog <dataset.rawlog>
            INPUT DATASET: rawlog. Input dataset in rawlog format (*.rawlog)

        --lidar-sensor-label <lidar1>
            If provided, this supersedes the values in the 'lidar_sensor_labels'
            entry of the odometry pipeline, defining the sensorLabel/topic name to
            read LIDAR data from. It can be a regular expression (std::regex)

        --skip-first-n <Number of dataset entries to skip>
            Skip the first N dataset entries (0=default, not used)

        --only-first-n <Number of dataset entries to run>
            Run for the first N steps only (0=default, not used)

        --output-simplemap <output-map.simplemap>
            Enables building and saving the simplemap for the mapping session

        --output-tum-path <output-trajectory.txt>
            Save the estimated path as a TXT file using the TUM file format (see
            evo docs)

        -l <foobar.so>,  --load-plugins <foobar.so>
            One or more (comma separated) *.so files to load as plugins

        -v <INFO>,  --verbosity <INFO>
            Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)

        -c <demo.yml>,  --config <demo.yml>
            (required)  Input YAML config file (required) (*.yml)

        --,  --ignore_rest
            Ignores the rest of the labeled arguments following this flag.

        --version
            Displays version information and exits.

        -h,  --help
            Displays usage information and exits.




