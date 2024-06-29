.. _mola_lo_pipelines:

============================
LiDAR odometry pipelines
============================

.. contents:: Provided LO pipelines
   :depth: 1
   :local:
   :backlinks: none

Most :ref:`parts <mola-internal-arch>` of the MOLA-LO system are **configured dynamically** from a YAML file.
Basically, the whole design about how many **local map layers** exist, the pointcloud **processing pipelines**,
**ICP matchers and optimizers**, etc. can be changed from this YAML file, without the need to touch the code or recompile.

The best way to understand the different parts of this file is to :ref:`browse the default pipeline <mola_3d_default_pipeline>`
provided for 3D LiDARs, and the rest of provided alternative pipelines. Most of the times, comments in the YAML
are self-explanatory. In case of doubts, do not hesitate in opening an issue to ask.

.. note::

   This page also enumerates all **environment variables** that can be defined to modify the behavior of the pipelines.

.. dropdown:: MOLA-specific YAML extensions
    :icon: light-bulb

    MOLA-LO uses the C++ library ``mola_yaml`` to parse YAML files, hence all YAML language extensions defined there
    applies to input YAML files used anywhere in a MOLA-LO system, e.g. ``${VAR|default}`` means "replace by environment
    variable ``VAR`` or, if it does not exist, by ``default``".


|

.. _mola_3d_default_pipeline:

Default pipeline for 3D LiDAR
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
This is the reference configuration used for most examples in the MOLA-LO paper, and should work great
out of the box for most common situations.

.. dropdown:: YAML listing
    :icon: code-review

    File: `mola_lidar_odometry/pipelines/lidar3d-default.yaml <https://github.com/MOLAorg/mola_lidar_odometry/blob/develop/pipelines/lidar3d-default.yaml>`_

    .. literalinclude:: ../../../mola_lidar_odometry/pipelines/lidar3d-default.yaml
       :language: yaml

.. dropdown:: Environment variables
    :icon: checklist
    :open:

    * ``XXXX``: Write me...

|

Pipeline for 2D LiDAR
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Write me...

