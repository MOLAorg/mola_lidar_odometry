.. _mola_lo_diagnostics:

===========================
ROS 2 Diagnostics (REP-107)
===========================

MOLA-LO implements the ``mola::DiagnosticsProvider`` interface so that its
health status is published, when running in a ROS 2 stack, as
``diagnostic_msgs/DiagnosticArray`` messages on the ``/diagnostics`` topic,
following `REP-107 <https://www.ros.org/reps/rep-0107.html>`_.

Publication is handled by :ref:`mola_bridge_ros2 <mola_bridge_ros2>`, which
discovers all loaded modules implementing ``DiagnosticsProvider`` via
``findService<>()`` and aggregates their statuses.

.. contents::
   :depth: 1
   :local:
   :backlinks: none

Published statuses
---------------------

MOLA-LO publishes the following ``DiagnosticStatus`` entries on each tick:

* **LidarOdometry: Input Data** — staleness of incoming LiDAR observations
  and dropped-frame ratio.
* **LidarOdometry: ICP Quality** — current ICP quality metric from the last
  scan matching.
* **LidarOdometry: Timing** — processing time utilization (fraction of the
  expected frame period used by the pipeline).
* **LidarOdometry: Local Map** — local map size and update state
  (mapping enabled/disabled).
* **LidarOdometry: Overall Status** — worst-of aggregate of the above.

Each status carries a REP-107 severity (``OK`` / ``WARN`` / ``ERROR`` /
``STALE``) plus a set of ``key=value`` pairs with the numeric values used
for the decision.

Configuration
----------------

Thresholds are exposed as YAML parameters under the ``diagnostics`` block of
the LO pipeline (see ``pipelines/lidar3d-default.yaml``):

.. code-block:: yaml

   diagnostics:
     icp_quality_warn:       0.30    # ICP quality below this → WARN
     icp_quality_error:      0.10    # ICP quality below this → ERROR
     input_stale_sec:        3.0     # Age of last input beyond this → STALE
     input_error_sec:        5.0     # Age of last input beyond this → ERROR
     dropped_ratio_warn:     0.20    # Dropped-frame ratio beyond this → WARN
     dropped_ratio_error:    0.50    # Dropped-frame ratio beyond this → ERROR
     timing_utilization_warn: 0.80   # Frame utilization beyond this → WARN

All fields are optional; omitted entries fall back to the defaults shown
above.

Inspecting diagnostics at runtime
-----------------------------------

.. code-block:: bash

   # Raw array:
   ros2 topic echo /diagnostics

   # Human-readable live view (flat list, no aggregator needed):
   ros2 run rqt_runtime_monitor rqt_runtime_monitor

   # Aggregated / grouped view (requires a running diagnostic_aggregator,
   # reads /diagnostics_agg):
   ros2 run rqt_robot_monitor rqt_robot_monitor

Foxglove Studio also ships a native *Diagnostics — Detail* and
*Diagnostics — Summary* panel that consumes ``/diagnostics`` directly; no
aggregator required.

Testing with the bundled aggregator
-----------------------------------

For quick bring-up / demos, the launch file can start a standalone
``diagnostic_aggregator`` preconfigured for MOLA-LO:

.. code-block:: bash

   ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
       lidar_topic_name:=/ouster/points \
       use_diagnostic_aggregator:=True

   # In another terminal:
   ros2 run rqt_robot_monitor rqt_robot_monitor

The sample aggregator YAML is installed at
``$(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/config/diagnostics_aggregator.yaml``
and groups MOLA-LO statuses under the ``MOLA LO`` category.

.. note::

   Leave ``use_diagnostic_aggregator:=False`` (the default) when MOLA-LO is
   part of a larger robot stack that already launches its own central
   aggregator — two aggregators subscribing to the same ``/diagnostics``
   will produce duplicated/conflicting ``/diagnostics_agg`` trees. In that
   case, just add a ``startswith: ['LidarOdometry']`` analyzer to your
   existing aggregator YAML.

