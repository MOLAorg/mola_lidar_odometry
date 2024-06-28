.. _mola_lidar_odometry_cli:

============================
LiDAR odometry CLI
============================

``mola-lidar-odometry-cli`` is a standalone command line program to run
MOLA-LO on a dataset in an offline fashion.
The dataset is processed as fast as possible using all available CPU cores.
Available outputs include the trajectory (as a file in TUM format) and
the :ref:`simple-map <mola-lo-role>`, which can be used to generate
metric maps.

.. contents:: :local:


Usage examples
-----------------

Process a ROS 2 bag
~~~~~~~~~~~~~~~~~~~~~

    .. code-block:: bash

        mola-lidar-odometry-cli \
          -c $(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/pipelines/lidar3d-default.yaml \
          --input-rosbag2 /PATH/TO/YOUR/rosbag.mcap \
          --lidar-sensor-label /ouster/points \
          --output-tum-path trajectory.tum


Complete list of arguments
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




