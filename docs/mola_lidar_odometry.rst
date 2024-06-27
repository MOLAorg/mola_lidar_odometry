.. _mola_lidar_odometry:

============================
LiDAR odometry
============================

`MOLA LiDAR odometry (MOLA-LO) <https://github.com/MOLAorg/mola_lidar_odometry/>`_ is
the main MOLA component for 3D and 2D LiDAR odometry and localization.
It is designed to provide accurate and robust motion tracking, with a 
default configuration provided to **work out of the box** for you without 
parameter tuning: it works for 16 to 128 ring LiDARs, 
in outdoor or indoor environments, with motion profiles ranging from
hand-held, slow wheeled or quadruped robots, to fast vehicles on highways.

This page explains the role of this component in the MOLA ecosystem,
how to deal with generated maps, and gives a glimpse into its internal theoretical design.

After getting familiar with the core ideas, you can jump into 
:ref:`running some demos <mola_lo_demos>`,
learning about :ref:`provided CLI entry points <mola_lo_apps>`,
and how to :ref:`build a map from your own ROS 2 bag dataset <mola_lo_ros>`.

.. contents:: Provided LO pipelines
   :depth: 1
   :hidden:
   :local:
   :backlinks: none

.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls autoplay loop muted style="width: 512px;">
       <source src="mola_main_page_video.mp4" type="video/mp4">
     </video>
   </div>

|

Role within the MOLA ecosystem
----------------------------------

At a conceptual level, an odometry module takes **raw sensory data** as input
and outputs an **estimated trajectory** of the sensor or vehicle.

By picking a subset of the **input raw data** and pairing them with the **corresponding
pose** at which it was sensed, we can build a sparse graph of tuples "(Observation, pose)".
Such a graph is called a **view-based map** or **simple-map** in our framework.

.. image:: imgs/odometry_inputs_outputs.png
  :width: 400

Simple-maps can then be used as input to :ref:`mp2p_icp applications <mp2p_icp_applications>` for analysis, filtering,
or creation of arbitrarily-complex metric maps of different kinds.
Refer to :ref:`mp2p_icp_basics`.

MOLA-LO is provided as the C++ class `mola::LidarOdometry <class_mola_LidarOdometry.html>`_, which 
implements the `mola::ExecutableBase` interface so it is able to communicate
with other input and output MOLA modules:

.. image:: mola_system_scheme.png
  :width: 690

As shown in the figure above, once encapsulated within a MOLA application container,
the LO module can take **input sensory data** from other MOLA input modules, 
and the live LO output can optionally be either **visualized** in the ``mola_viz`` GUI,
and/or **published** to an external ROS 2 system.

Therefore, the most flexible way to use MOLA LO is by means of **different combinations
of input and output modules**, depending on what are the desired input data sources,
and that is defined by means of **mola-cli launch files**.

:ref:`mola-cli <mola-launcher>` is a standalone command line interface (CLI) program
provided by the `mola_launcher <https://github.com/MOLAorg/mola/tree/develop/mola_launcher>`_ package.
**Predefined launch files** `are provided <https://github.com/MOLAorg/mola_lidar_odometry/tree/develop/mola-cli-launchs>`_
for common tasks like running MOLA-LO on well-known public datasets 
or from rosbags.
However, to make thinks simpler, a set of executable scripts are provided
to make launching them easier: :ref:`mola_lo_apps`.

Apart of this way to run MOLA-LO, two additional ways are provided for convenience: 

* :ref:`mola-lidar-odometry-cli <mola_lidar_odometry_cli>`: this standalone program
  is provided as a way to **process a given dataset as fast as possible**, without
  any interaction with GUIs, message subscription or reception, etc.
  It is also great for scripting and automating SLAM pipelines from raw datasets or rosbags.
* :ref:`ROS 2 integration <mola_lo_ros>`: ROS 2 launch files are also provided for easier integration
  for real-time odometry and mapping.

|

Internal architecture
-------------------------

Internally, MOLA LO is based on mp2p_icp filtering and ICP pipelines:

.. image:: mola_lidar_odometry_architecture.png
  :width: 690

Refer to the MOLA LO paper for further details.


How to cite it
-------------------------

The ``mola_lidar_odometry`` system was presented in :cite:`blanco2024mola_lo`:

  J.L. Blanco,
  `A flexible framework for accurate LiDAR odometry, map manipulation, and localization`_, in
  ArXiV, 2024.
