/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

/**
 * @file   mola-lidar-odometry-cli.cpp
 * @brief  main() for the cli app running lidar-inertial odometry for
 *         offline datasets.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 22, 2023
 */

#include <mola_kernel/MinimalModuleContainer.h>
#include <mola_kernel/interfaces/OfflineDatasetSource.h>
#include <mola_kernel/pretty_print_exception.h>
#include <mola_lidar_odometry/LidarOdometry.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/progress.h>
#include <mrpt/system/string_utils.h>

#include <memory>

#if defined(HAVE_MOLA_SE_SIMPLE)
#include <mola_state_estimation_simple/StateEstimationSimple.h>
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
#include <mola_input_kitti_dataset/KittiOdometryDataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_KITTI360)
#include <mola_input_kitti360_dataset/Kitti360Dataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_MULRAN)
#include <mola_input_mulran_dataset/MulranDataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_RAWLOG)
#include <mola_input_rawlog/RawlogDataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
#include <mola_input_rosbag2/Rosbag2Dataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG1)
#include <mola_input_rosbag1/Rosbag1Dataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_PARIS_LUCO)
#include <mola_input_paris_luco_dataset/ParisLucoDataset.h>
#endif

#include <CLI/CLI.hpp>
#include <csignal>  // sigaction
#include <cstdlib>
#include <iostream>
#include <string>

namespace
{

// Thin wrapper mimicking TCLAP::ValueArg's isSet()/getValue() so the rest of
// this file (option consumption below) stays a straightforward read.
template <typename T>
struct Opt
{
  T value{};
  CLI::Option * opt = nullptr;

  bool isSet() const { return opt && opt->count() > 0; }
  const T & getValue() const { return value; }
};

struct Cli
{
  // Declare supported cli switches ===========
  CLI::App cmd{"mola-lidar-odometry-cli"};

  Opt<std::string> argYAML;
  Opt<std::string> arg_verbosity_level;
  Opt<std::string> arg_plugins;
  Opt<std::string> arg_stateEstimatorClass;
  Opt<std::string> arg_stateEstimatorParams;
  Opt<std::string> arg_outPath;
  Opt<std::string> arg_outTwist;
  Opt<std::string> arg_outSimpleMap;
  Opt<int> arg_firstN;
  Opt<int> arg_skipFirstN;
  Opt<std::string> arg_lidarLabel;
  Opt<std::string> arg_imuLabel;
  Opt<std::string> arg_baseLinkName;
  Opt<std::string> arg_tfTopic;
  Opt<std::string> arg_tfStaticTopic;
  Opt<double> arg_progressBarPeriod;

// Input dataset can come from one of these:
// --------------------------------------------
#if defined(HAVE_MOLA_INPUT_RAWLOG)
  Opt<std::string> argRawlog;
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
  Opt<std::string> argRosbag2;
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG1)
  Opt<std::string> argRosbag1;
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
  Opt<std::string> argKittiSeq;
  Opt<double> argKittiAngleDeg;
#endif

#if defined(HAVE_MOLA_INPUT_KITTI360)
  Opt<std::string> argKitti360Seq;
#endif

#if defined(HAVE_MOLA_INPUT_MULRAN)
  Opt<std::string> argMulranSeq;
#endif

#if defined(HAVE_MOLA_INPUT_PARIS_LUCO)
  Opt<bool> argParisLucoSeq;
#endif

  Cli()
  {
    cmd.set_version_flag("--version", std::string(MOLA_LO_VERSION));

    argYAML.opt =
      cmd
        .add_option(
          "-c,--config", argYAML.value, "Input pipeline YAML config file (required) (*.yml)")
        ->required();

    arg_verbosity_level.opt = cmd.add_option(
      "-v,--verbosity", arg_verbosity_level.value,
      "Verbosity level: ERROR|WARN|INFO|DEBUG {Default: INFO}");

    arg_plugins.opt = cmd
                        .add_option(
                          "-l,--load-plugins", arg_plugins.value,
                          "One or more {comma separated} *.so files to load as plugins")
                        ->option_text("foobar.so");

    arg_stateEstimatorClass.opt =
      cmd
        .add_option(
          "--state-estimator", arg_stateEstimatorClass.value,
          "The C++ class name of the state estimator to use")
        ->option_text("(StateEstimationSimple|StateEstimationSmoother)");

    // No option_text() here: CLI11 only shows the REQUIRED marker in --help
    // when the type name is auto-generated (as --config does), not when
    // option_text() overrides it.
    arg_stateEstimatorParams.opt =
      cmd
        .add_option(
          "--state-estimator-param-file", arg_stateEstimatorParams.value,
          "Path to YAML parameters file to configure the state estimator.")
        ->required();

    arg_outPath.opt = cmd
                        .add_option(
                          "--output-tum-path", arg_outPath.value,
                          "Save the estimated path as a TXT file using the TUM file format {see "
                          "evo docs}")
                        ->option_text("output-trajectory.txt");

    arg_outTwist.opt =
      cmd
        .add_option("--output-twist", arg_outTwist.value, "Save the estimated twist as a TXT file")
        ->option_text("output-twist.txt");

    arg_outSimpleMap.opt = cmd
                             .add_option(
                               "--output-simplemap", arg_outSimpleMap.value,
                               "Enables building and saving the simplemap for the mapping session")
                             ->option_text("output-map.simplemap");

    arg_firstN.opt = cmd
                       .add_option(
                         "--only-first-n", arg_firstN.value,
                         "Run for the first N steps only {0=default, not used}")
                       ->check(CLI::NonNegativeNumber);

    arg_skipFirstN.opt = cmd
                           .add_option(
                             "--skip-first-n", arg_skipFirstN.value,
                             "Skip the first N dataset entries {0=default, not used}")
                           ->check(CLI::NonNegativeNumber);

    // The four options below take an environment-variable fallback, spelled
    // exactly as the mola-cli-launchs/*.yaml files already spell it. A single
    // dataset profile (scripts/lib/profiles/*.sh) can then drive this offline
    // CLI and the online launch files from one set of exported variables,
    // instead of each caller keeping its own flags-vs-env translation table.
    // An explicit command-line flag still wins over the environment.
    arg_lidarLabel.value = "lidar1";
    arg_lidarLabel.opt = cmd
                           .add_option(
                             "--lidar-sensor-label", arg_lidarLabel.value,
                             "If provided, this supersedes the values in the 'lidar_sensor_labels' "
                             "entry of the odometry pipeline, defining the sensorLabel/topic name "
                             "to read LIDAR data from. It can be a regular expression {std::regex}")
                           ->envname("MOLA_LIDAR_TOPIC");

    arg_imuLabel.value = "imu";
    arg_imuLabel.opt = cmd
                         .add_option(
                           "--imu-sensor-label", arg_imuLabel.value,
                           "If provided, this supersedes the values in the 'imu_sensor_label' "
                           "entry of the odometry pipeline, defining the sensorLabel/topic name to "
                           "read IMU data from. It can be a regular expression {std::regex}")
                         ->envname("MOLA_IMU_TOPIC")
                         ->capture_default_str();

    arg_baseLinkName.value = "base_link";
    arg_baseLinkName.opt =
      cmd
        .add_option(
          "--base-link-frame-id", arg_baseLinkName.value,
          "Only for rosbag input sources. This defines the /tf frame_id used as "
          "reference frame for the vehicle or robot. It is used to get sensors poses with "
          "respect to the vehicle from /tf data.")
        ->envname("MOLA_TF_BASE_LINK")
        ->capture_default_str();

    arg_tfTopic.value = "/tf";
    arg_tfTopic.opt =
      cmd
        .add_option(
          "--tf-topic", arg_tfTopic.value,
          "Only for rosbag2 input: /tf topic name in the bag. Override for namespaced bags "
          "(e.g. '/robot1/tf').")
        ->envname("MOLA_TF_TOPIC")
        ->capture_default_str();

    arg_tfStaticTopic.value = "/tf_static";
    arg_tfStaticTopic.opt =
      cmd
        .add_option(
          "--tf-static-topic", arg_tfStaticTopic.value,
          "Only for rosbag2 input: /tf_static topic name in the bag. Override for namespaced "
          "bags (e.g. '/robot1/tf_static').")
        ->envname("MOLA_TF_STATIC_TOPIC")
        ->capture_default_str();

    arg_progressBarPeriod.value = -1.0;
    arg_progressBarPeriod.opt =
      cmd
        .add_option(
          "--progress-bar-period", arg_progressBarPeriod.value,
          "Minimum percentage step, in [0,100], between progress bar printouts. Use 0 to "
          "disable the progress bar entirely. Handy to cut down log verbosity in batch/CI runs "
          "(e.g. Jenkins). {Default: print on (almost) every processed entry}")
        ->check(CLI::Range(0.0, 100.0));

#if defined(HAVE_MOLA_INPUT_RAWLOG)
    argRawlog.opt = cmd
                      .add_option(
                        "--input-rawlog", argRawlog.value,
                        "INPUT DATASET: rawlog. Input dataset in rawlog format {*.rawlog}")
                      ->option_text("dataset.rawlog");
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
    argRosbag2.opt = cmd
                       .add_option(
                         "--input-rosbag2", argRosbag2.value,
                         "INPUT DATASET: rosbag2. Input dataset in rosbag2 format {*.mcap}")
                       ->option_text("dataset.mcap");
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG1)
    argRosbag1.opt = cmd
                       .add_option(
                         "--input-rosbag1", argRosbag1.value,
                         "INPUT DATASET: rosbag1. Input dataset in ROS 1 bag format {*.bag}")
                       ->option_text("dataset.bag");
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
    argKittiSeq.opt = cmd
                        .add_option(
                          "--input-kitti-seq", argKittiSeq.value,
                          "INPUT DATASET: Use KITTI dataset sequence number 00|01|...")
                        ->option_text("00");

    argKittiAngleDeg.opt = cmd
                             .add_option(
                               "--kitti-correction-angle-deg", argKittiAngleDeg.value,
                               "Correction vertical angle offset {see Deschaud,2018}")
                             ->option_text("0.205 [degrees]");
#endif

#if defined(HAVE_MOLA_INPUT_KITTI360)
    argKitti360Seq.opt =
      cmd
        .add_option(
          "--input-kitti360-seq", argKitti360Seq.value,
          "INPUT DATASET: Use KITTI360 dataset sequence number 00|01|...|test_00|...")
        ->option_text("00");
#endif

#if defined(HAVE_MOLA_INPUT_MULRAN)
    argMulranSeq.opt = cmd
                         .add_option(
                           "--input-mulran-seq", argMulranSeq.value,
                           "INPUT DATASET: Use Mulran dataset sequence KAIST01|KAIST02|...")
                         ->option_text("KAIST01");
#endif

#if defined(HAVE_MOLA_INPUT_PARIS_LUCO)
    argParisLucoSeq.opt = cmd.add_flag(
      "--input-paris-luco", argParisLucoSeq.value,
      "INPUT DATASET: Use Paris Luco dataset (unique sequence=00)");
#endif
  }
};  // end struct "Cli"

#if defined(HAVE_MOLA_INPUT_RAWLOG)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_rawlog(
  const std::string & rawlogFile, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::RawlogDataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      rawlog_filename: '%s'
      read_all_first: true
)"""",
    rawlogFile.c_str())));

  o->initialize(cfg);

  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_MULRAN)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_mulran(
  const std::string & mulranSequence, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::MulranDataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      base_dir: ${MULRAN_BASE_DIR}
      sequence: '%s'
      time_warp_scale: 1.0
      publish_lidar: true
      publish_ground_truth: true
)"""",
    mulranSequence.c_str())));

  o->initialize(cfg);

  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_rosbag2(
  Cli & cli, const std::string & rosbag2file, const mrpt::system::VerbosityLevel logLevel)
{
  ASSERTMSG_(
    cli.arg_lidarLabel.isSet(),
    "Using a rosbag2 as input requires telling what is the lidar topic "
    "with --lidar-sensor-label <TOPIC_NAME>");

  auto o = std::make_shared<mola::Rosbag2Dataset>();

  // A comma-separated value becomes a YAML sequence, so a recording split
  // across several bag directories (e.g. Oxford Spires keble-college-04, two
  // halves of one continuous recording sharing a base timestamp) is replayed as
  // the single sequence it is. Rosbag2Dataset accepts a scalar or a sequence.
  std::string bagsYaml;
  {
    std::vector<std::string> parts;
    mrpt::system::tokenize(rosbag2file, ",", parts);
    ASSERT_(!parts.empty());
    if (parts.size() == 1) {
      bagsYaml = "'" + mrpt::system::trim(parts[0]) + "'";
    } else {
      for (const auto & bp : parts) {
        bagsYaml += "\n        - '" + mrpt::system::trim(bp) + "'";
      }
    }
  }
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      rosbag_filename: %s
      base_link_frame_id: '%s'
      tf_topic: '%s'
      tf_static_topic: '%s'
      sensors:
        - topic: '%s'
          type: CObservationPointCloud
          # If present, this will override whatever /tf tells about the sensor pose:
          fixed_sensor_pose: "${LIDAR_POSE_X|0} ${LIDAR_POSE_Y|0} ${LIDAR_POSE_Z|0} ${LIDAR_POSE_YAW|0} ${LIDAR_POSE_PITCH|0} ${LIDAR_POSE_ROLL|0}"  # 'x y z yaw_deg pitch_deg roll_deg'
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_LIDAR_POSE|false}
        - topic: ${MOLA_GNSS_TOPIC|'/gps'}
          sensorLabel: 'gps'
          #type: CObservationGPS  # This will be determined automatically by Rosbag2Dataset
          is_optional: true
          fixed_sensor_pose: "${GNSS_POSE_X|0} ${GNSS_POSE_Y|0} ${GNSS_POSE_Z|0} ${GNSS_POSE_YAW|0} ${GNSS_POSE_PITCH|0} ${GNSS_POSE_ROLL|0}"  # 'x y z yaw_deg pitch_deg roll_deg'
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_GNSS_POSE|false}
        - topic: '%s'
          type: CObservationIMU
          # If present, this will override whatever /tf tells about the sensor pose:
          fixed_sensor_pose: "${IMU_POSE_X|0} ${IMU_POSE_Y|0} ${IMU_POSE_Z|0} ${IMU_POSE_YAW|0} ${IMU_POSE_PITCH|0} ${IMU_POSE_ROLL|0}" # 'x y z yaw_deg pitch_deg roll_deg''
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_IMU_POSE|false}
)"""",
    bagsYaml.c_str(), cli.arg_baseLinkName.getValue().c_str(), cli.arg_tfTopic.getValue().c_str(),
    cli.arg_tfStaticTopic.getValue().c_str(), cli.arg_lidarLabel.getValue().c_str(),
    cli.arg_imuLabel.getValue().c_str())));

  o->initialize(cfg);

  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG1)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_rosbag1(
  Cli & cli, const std::string & rosbag1file, const mrpt::system::VerbosityLevel logLevel)
{
  ASSERTMSG_(
    cli.arg_lidarLabel.isSet(),
    "Using a rosbag1 as input requires telling what is the lidar topic "
    "with --lidar-sensor-label <TOPIC_NAME>");

  auto o = std::make_shared<mola::Rosbag1Dataset>();

  // A comma-separated value becomes a YAML sequence, so a recording split
  // across several bag files is replayed as the single sequence it is.
  // Rosbag1Dataset accepts a scalar or a sequence.
  std::string bagsYaml;
  {
    std::vector<std::string> parts;
    mrpt::system::tokenize(rosbag1file, ",", parts);
    ASSERT_(!parts.empty());
    if (parts.size() == 1) {
      bagsYaml = "'" + mrpt::system::trim(parts[0]) + "'";
    } else {
      for (const auto & bp : parts) {
        bagsYaml += "\n        - '" + mrpt::system::trim(bp) + "'";
      }
    }
  }
  o->setMinLoggingLevel(logLevel);

  // Note: /tf and /tf_static topic names are fixed in Rosbag1Dataset.
  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      rosbag_filename: %s
      base_link_frame_id: '%s'
      sensors:
        - topic: '%s'
          type: CObservationPointCloud
          # If present, this will override whatever /tf tells about the sensor pose:
          fixed_sensor_pose: "${LIDAR_POSE_X|0} ${LIDAR_POSE_Y|0} ${LIDAR_POSE_Z|0} ${LIDAR_POSE_YAW|0} ${LIDAR_POSE_PITCH|0} ${LIDAR_POSE_ROLL|0}"  # 'x y z yaw_deg pitch_deg roll_deg'
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_LIDAR_POSE|false}
        - topic: ${MOLA_GNSS_TOPIC|'/gps'}
          sensorLabel: 'gps'
          is_optional: true
          fixed_sensor_pose: "${GNSS_POSE_X|0} ${GNSS_POSE_Y|0} ${GNSS_POSE_Z|0} ${GNSS_POSE_YAW|0} ${GNSS_POSE_PITCH|0} ${GNSS_POSE_ROLL|0}"  # 'x y z yaw_deg pitch_deg roll_deg'
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_GNSS_POSE|false}
        - topic: '%s'
          type: CObservationIMU
          # If present, this will override whatever /tf tells about the sensor pose:
          fixed_sensor_pose: "${IMU_POSE_X|0} ${IMU_POSE_Y|0} ${IMU_POSE_Z|0} ${IMU_POSE_YAW|0} ${IMU_POSE_PITCH|0} ${IMU_POSE_ROLL|0}" # 'x y z yaw_deg pitch_deg roll_deg''
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_IMU_POSE|false}
        # Wheel odometry (disabled by default -- an empty topic name means
        # no handler is installed, matching lidar_odometry_from_rosbag1.yaml/
        # rosbag2.yaml's MOLA_ODOMETRY_TOPIC convention exactly, name and
        # empty-by-default alike, rather than opting every dataset whose bag
        # happens to carry a topic literally named "/odom" -- a very common
        # name across unrelated robots/conventions -- into wheel-odom fusion
        # silently). Set MOLA_ODOMETRY_TOPIC explicitly to enable, e.g. for a
        # separate odom_*.bag comma-joined into rosbag_filename above
        # (Rosbag1Dataset merges all listed bags into one topic space).
        # is_optional is set for readability/consistency with the gps entry
        # above, but Rosbag1Dataset doesn't actually implement that key (grep
        # confirms it) -- harmless here regardless, since an empty or absent
        # topic name is simply never seen in the bag, so no handler ever
        # fires; nothing about "optional" behavior is being relied on.
        #
        # No fixed_sensor_pose here, and this is NOT interchangeable with the
        # lidar/imu/gps entries above: mrpt::obs::CObservationOdometry cannot
        # carry a sensor pose at all (getSensorPose()/setSensorPose() are
        # both no-ops in that class), and Rosbag1Dataset::toOdometry() makes
        # no attempt to apply one from config either way. If the wheel-
        # odometry frame has ANY nontrivial rotation relative to base_link,
        # fusing it here injects motion in the wrong frame -- verified this
        # is a real, non-hypothetical trap: lidar_odometry_from_citrusfarm.yaml's
        # odom_wheels entry carries a ~180deg-yaw fixed_sensor_pose (a real
        # calibration value, matching its lidar entry's rotation) that gets
        # silently discarded exactly this way, which is why that dataset's
        # wheel odometry is deliberately NOT enabled anywhere it's actually
        # invoked (plans-mola-server's run-single-test.sh) as of 2026-08-11.
        # Fine for a dataset whose wheel-odometry frame is already
        # (approximately) base_link-aligned, e.g. BotanicGarden's Xsens IMU.
        - topic: ${MOLA_ODOMETRY_TOPIC|''}
          sensorLabel: ${MOLA_ODOM_SENSOR_LABEL|odom_wheels}
          type: CObservationOdometry
          is_optional: true
)"""",
    bagsYaml.c_str(), cli.arg_baseLinkName.getValue().c_str(),
    cli.arg_lidarLabel.getValue().c_str(), cli.arg_imuLabel.getValue().c_str())));

  o->initialize(cfg);

  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_kitti(
  Cli & cli, const std::string & kittiSeqNumber, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::KittiOdometryDataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      base_dir: ${KITTI_BASE_DIR}
      sequence: '%s'
      time_warp_scale: 1.0
      clouds_as_organized_points: false
      publish_lidar: true
      publish_image_0: false
      publish_image_1: false
      publish_ground_truth: true
)"""",
    kittiSeqNumber.c_str())));

  o->initialize(cfg);

  if (cli.argKittiAngleDeg.isSet()) {
    o->VERTICAL_ANGLE_OFFSET = mrpt::DEG2RAD(cli.argKittiAngleDeg.getValue());
  }

  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_KITTI360)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_kitti360(
  const std::string & kittiSeqNumber, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::Kitti360Dataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      base_dir: ${KITTI360_DATASET}
      sequence: '%s'
      time_warp_scale: 1.0
      publish_lidar: true
      publish_image_0: false
      publish_image_1: false
      publish_image_2: false
      publish_image_3: false
      publish_ground_truth: true
)"""",
    kittiSeqNumber.c_str())));

  o->initialize(cfg);

  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_PARIS_LUCO)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_paris_luco(
  const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::ParisLucoDataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(
    R""""(
    params:
      base_dir: ${PARIS_LUCO_BASE_DIR}
      sequence: '00'  # There is only one sequence in this dataset
      time_warp_scale: 1.0
)""""));

  o->initialize(cfg);

  return o;
}
#endif

void mola_signal_handler(int s);
void mola_install_signal_handler();

void mola_signal_handler(int s)
{
  std::cerr << "Caught signal " << s << ". Shutting down..."
            << "\n";
  exit(0);  // NOLINT
}

void mola_install_signal_handler()
{
  struct sigaction sigIntHandler
  {
  };

  sigIntHandler.sa_handler = &mola_signal_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, nullptr);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
int main_odometry(Cli & cli)
{
  // Declare main LO module:
  // ------------------------------------------
  auto liodom = mola::LidarOdometry::Create();

  // Declare state estimator module:
  // ------------------------------------------
  mola::NavStateFilter::Ptr stateEstimator;
  if (cli.arg_stateEstimatorClass.isSet()) {
    const auto sClass = cli.arg_stateEstimatorClass.getValue();
    auto o = mrpt::rtti::classFactory(sClass);
    ASSERTMSG_(
      o, mrpt::format(
           "Apparently unknown class name: '%s' (missing plugin .so file?)", sClass.c_str()));
    stateEstimator = std::dynamic_pointer_cast<mola::NavStateFilter>(o);
    ASSERTMSG_(
      stateEstimator,
      mrpt::format(
        "Class '%s' does not implemented the expected interface mola::NavStateFilter",
        sClass.c_str()));
  }

#if defined(HAVE_MOLA_SE_SIMPLE)
  // Default?
  if (!stateEstimator) {
    stateEstimator = mola::state_estimation_simple::StateEstimationSimple::Create();
  }
#endif

  ASSERTMSG_(
    stateEstimator,
    "Either provide an explicit --state-estimator flag or build against "
    "mola::state_estimation_simple");

  // Cast to the interface that accepts raw sensor data:
  auto stateEstimatorAsRawConsumer =
    std::dynamic_pointer_cast<mola::RawDataConsumer>(stateEstimator);
  if (!stateEstimatorAsRawConsumer) {
    std::cerr << "[Warning] The state estimator '" << stateEstimator->GetRuntimeClass()->className
              << "' does not implement the mola::RawDataConsumer interface, so it will not receive "
                 "raw sensor data.\n";
  }

  // This is an OFFLINE batch tool: it consumes the dataset as fast as the
  // pipeline allows, and reproducibility is the whole point of running it.
  // The smoother's shipped YAML defaults `async_backend` to true, which is the
  // right setting for a real-time deployment (queries are served from a
  // lock-free predictor re-anchored on the backend's last completed solve) and
  // the wrong one here twice over: that serving path is non-deterministic, and
  // with no clock pacing the front end outruns the backend by construction, so
  // how stale the anchor is becomes a function of host load rather than of the
  // data.
  //
  // Default it off for this binary, without overwriting an explicit setting
  // (third argument 0), so anyone who really wants the real-time path can still
  // ask for it. Datasets whose YAML does not read this variable are unaffected.
  setenv("MOLA_ASYNC_BACKEND", "false", 0 /* do not overwrite */);

  // Make mandatory to specify state estimation config file, so defaults and initialize() are not skipped
  {
    const auto seParamsFile = cli.arg_stateEstimatorParams.getValue();
    auto seParams = mrpt::containers::yaml::FromFile(seParamsFile);
    stateEstimator->initialize(mola::parse_yaml(seParams));
  }

  // Make both modules discoverables to each other:
  // -------------------------------------------------
  const mola::MinimalModuleContainer moduleContainer = {{liodom, stateEstimator}};

  // Logging level:
  mrpt::system::VerbosityLevel logLevel = liodom->getMinLoggingLevel();
  if (cli.arg_verbosity_level.isSet()) {
    using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
    logLevel = vl::name2value(cli.arg_verbosity_level.getValue());
    liodom->setVerbosityLevel(logLevel);
    stateEstimator->setVerbosityLevel(logLevel);
  }

  // Add a logger hook to detect visible messages to the terminal
  // and avoid overwriting them with the CLI progress bar:
  bool liodom_emitted_log = false;
  std::mutex liodom_emitted_log_mtx;
  const auto mark_emitted_log = [&]() {
    auto lck = mrpt::lockHelper(liodom_emitted_log_mtx);
    liodom_emitted_log = true;
  };
  const auto has_emitted_log = [&]() -> bool {
    auto lck = mrpt::lockHelper(liodom_emitted_log_mtx);
    return liodom_emitted_log;
  };
  const auto unmark_emitted_log = [&]() {
    auto lck = mrpt::lockHelper(liodom_emitted_log_mtx);
    liodom_emitted_log = false;
  };
  liodom->mrpt::system::COutputLogger::logRegisterCallback(
    [&](
      [[maybe_unused]] std::string_view msg, const mrpt::system::VerbosityLevel level,
      [[maybe_unused]] std::string_view loggerName,
      [[maybe_unused]] const mrpt::Clock::time_point timestamp) {
      if (level < liodom->getMinLoggingLevel()) {
        return;
      }
      mark_emitted_log();
    });

  // Initialize LiDAR Odometry:
  const auto file_yml = cli.argYAML.getValue();
  const auto cfg = mola::load_yaml_file(file_yml);

  // Enable time profiling: // can be enabled via YAML options
  // liodom->profiler_.enable();

  // liodom->initialize_common(cfg); // can be skipped for a non-MOLA system
  liodom->initialize(cfg);

  if (cli.arg_outSimpleMap.isSet()) {
    liodom->params_.simplemap.generate = true;
    // don't save within the LidarOdometry object, we will do it here in
    // this cli app:
    liodom->params_.simplemap.save_final_map_to_file.clear();
  }

  if (cli.arg_lidarLabel.isSet()) {
    liodom->params_.lidar_sensor_labels.assign(1, std::regex(cli.arg_lidarLabel.getValue()));
  }

  if (cli.arg_imuLabel.isSet()) {
    liodom->params_.imu_sensor_label = std::regex(cli.arg_imuLabel.getValue());
  }

  // Select dataset input:
  std::shared_ptr<mola::OfflineDatasetSource> dataset;

#if defined(HAVE_MOLA_INPUT_RAWLOG)
  if (cli.argRawlog.isSet()) {
    dataset = dataset_from_rawlog(cli.argRawlog.getValue(), logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_KITTI)
    if (cli.argKittiSeq.isSet()) {
    dataset = dataset_from_kitti(cli, cli.argKittiSeq.getValue(), logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_KITTI360)
    if (cli.argKitti360Seq.isSet()) {
    dataset = dataset_from_kitti360(cli.argKitti360Seq.getValue(), logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_MULRAN)
    if (cli.argMulranSeq.isSet()) {
    dataset = dataset_from_mulran(cli.argMulranSeq.getValue(), logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG2)
    if (cli.argRosbag2.isSet()) {
    dataset = dataset_from_rosbag2(cli, cli.argRosbag2.getValue(), logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG1)
    if (cli.argRosbag1.isSet()) {
    dataset = dataset_from_rosbag1(cli, cli.argRosbag1.getValue(), logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_PARIS_LUCO)
    if (cli.argParisLucoSeq.isSet()) {
    dataset = dataset_from_paris_luco(logLevel);
  } else
#endif
  {
    THROW_EXCEPTION(
      "At least one of the dataset input CLI flags must be defined. "
      "Use --help.");
  }
  ASSERT_(dataset);

  // Optional output twist:
  std::optional<mrpt::poses::CPose3DInterpolator> outTwist;
  if (cli.arg_outTwist.isSet()) {
    outTwist.emplace();
  }
  /// Timestamp of the last observation fed, to stamp the twist of the state the
  /// end-of-input flush may still produce:
  std::optional<mrpt::Clock::time_point> lastObsTimestamp;

  // Save GT, if available:
  if (cli.arg_outPath.isSet() && dataset->hasGroundTruthTrajectory()) {
    using namespace std::string_literals;

    const auto gtPath = dataset->getGroundTruthTrajectory();

    const auto gtOutFile = mrpt::system::fileNameChangeExtension(cli.arg_outPath.getValue(), "") +
                           "_gt."s + mrpt::system::extractFileExtension(cli.arg_outPath.getValue());

    std::cout << "Ground truth available. Saving it to: " << gtOutFile << "\n";

    gtPath.saveToTextFile_TUM(gtOutFile);
  }

  const double tStart = mrpt::Clock::nowDouble();

  size_t lastDatasetEntry = dataset->datasetSize();
  size_t firstDatasetEntry = 0;

  if (cli.arg_skipFirstN.isSet()) {
    firstDatasetEntry = cli.arg_skipFirstN.getValue();
  }

  if (cli.arg_firstN.isSet()) {
    lastDatasetEntry = firstDatasetEntry + cli.arg_firstN.getValue();
  }

  mrpt::keep_min(lastDatasetEntry, dataset->datasetSize());

  // Progress bar verbosity: unset => print on (almost) every entry (legacy
  // behavior); 0 => fully disabled; >0 => only every that many percent
  // (useful to cut down log spam in batch/CI runs, e.g. Jenkins).
  const bool progressBarDisabled =
    cli.arg_progressBarPeriod.isSet() && cli.arg_progressBarPeriod.getValue() <= 0;
  // Throttled mode targets batch/CI logs (the point of --progress-bar-period), where
  // in-place VT100 cursor updates don't render sensibly, so each printout gets its own line.
  const bool progressBarThrottled = !progressBarDisabled && cli.arg_progressBarPeriod.isSet();
  const double progressBarPeriodFraction =
    cli.arg_progressBarPeriod.isSet() ? cli.arg_progressBarPeriod.getValue() / 100.0 : 0;
  double lastProgressBarPrintedFraction = -1.0;

  if (!progressBarDisabled && !progressBarThrottled) {
    std::cout << "\n";  // Needed for the VT100 codes below.
  }

  // Run:
  for (size_t i = firstDatasetEntry; i < lastDatasetEntry; i++) {
    // Get observations from the dataset:
    using mrpt::obs::CObservation2DRangeScan;
    using mrpt::obs::CObservation3DRangeScan;
    using mrpt::obs::CObservationGPS;
    using mrpt::obs::CObservationIMU;
    using mrpt::obs::CObservationOdometry;
    using mrpt::obs::CObservationPointCloud;
    using mrpt::obs::CObservationRotatingScan;
    using mrpt::obs::CObservationVelodyneScan;

    const auto sf = dataset->datasetGetObservations(i);
    ASSERT_(sf);

    mrpt::obs::CObservation::Ptr obs;
    obs = sf->getObservationByClass<CObservationRotatingScan>();
    if (!obs) {
      obs = sf->getObservationByClass<CObservationPointCloud>();
    }
    if (!obs) {
      obs = sf->getObservationByClass<CObservation3DRangeScan>();
    }
    if (!obs) {
      obs = sf->getObservationByClass<CObservation2DRangeScan>();
    }
    if (!obs) {
      obs = sf->getObservationByClass<CObservationVelodyneScan>();
    }
    if (!obs) {
      obs = sf->getObservationByClass<CObservationGPS>();
    }
    if (!obs) {
      obs = sf->getObservationByClass<CObservationOdometry>();
    }
    if (!obs) {
      obs = sf->getObservationByClass<CObservationIMU>();
    }
    if (!obs) {
      continue;
    }

    // Send it to the odometry pipeline & the state estimator:
    if (stateEstimatorAsRawConsumer) {
      stateEstimatorAsRawConsumer->onNewObservation(obs);
    }

    liodom->onNewObservation(obs);

    // Show stats:
    const size_t N = (dataset->datasetSize() - 1);
    const double pc = static_cast<double>(i) / static_cast<double>(N);

    const bool isLastEntry = (i + 1 == lastDatasetEntry);
    bool doPrintProgress = false;
    if (progressBarDisabled) {
      doPrintProgress = false;
    } else if (progressBarThrottled) {
      // Reduced-verbosity mode: print only every `progressBarPeriodFraction`.
      if (
        lastProgressBarPrintedFraction < 0 ||
        pc - lastProgressBarPrintedFraction >= progressBarPeriodFraction || isLastEntry) {
        doPrintProgress = true;
        lastProgressBarPrintedFraction = pc;
      }
    } else {
      // Legacy default: print on (almost) every processed entry.
      doPrintProgress = true;
    }

    if (doPrintProgress) {
      const double tNow = mrpt::Clock::nowDouble();
      const double ETA = pc > 0 ? (tNow - tStart) * (1.0 / pc - 1) : .0;
      const double totalTime = ETA + (tNow - tStart);

      if (!progressBarThrottled) {
        // VT100 codes: cursor up and clear line, for an in-place update.
        if (!has_emitted_log()) {
          std::cout << "\033[A\33[2KT\r";
        }
        unmark_emitted_log();
      }

      std::optional<mrpt::poses::CPose3D> lastPose;
      if (const auto optPoseAndTwist = liodom->lastEstimatedState(); optPoseAndTwist) {
        const auto [pose, twist] = *optPoseAndTwist;
        lastPose = pose.mean;
      }

      std::cout << mrpt::system::progress(pc, 30)
                << mrpt::format(
                     " %6zu/%6zu (%.02f%%) ETA=%s/T=%s | Pose=%s | q=%.01f%%\n", i, N, 100 * pc,
                     mrpt::system::formatTimeInterval(ETA).c_str(),
                     mrpt::system::formatTimeInterval(totalTime).c_str(),
                     lastPose.has_value() ? lastPose->asString().c_str() : "(None)",
                     100.0 * liodom->lastIcpQuality());
      std::cout.flush();
    }

    while (liodom->isBusy()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Keep track of vehicle velocities?
    if (outTwist) {
      if (const auto optPoseAndTwist = liodom->lastEstimatedState(); optPoseAndTwist) {
        const auto & [pose, tw] = optPoseAndTwist.value();
        outTwist->insert(
          obs->timestamp, mrpt::math::TPose3D(tw.vx, tw.vy, tw.vz, tw.wz, tw.wy, tw.wx));
      }
      lastObsTimestamp = obs->timestamp;
    }
  }

  // The dataset is over, so scans still waiting for IMU data covering their
  // time span will never get it. Process them now, before reading the results
  // below, or the tail of the trajectory is lost:
  liodom->flushPendingLidarScans();

  // The flush may have produced one more state, after the loop wrote its last
  // twist entry:
  if (outTwist && lastObsTimestamp) {
    if (const auto optPoseAndTwist = liodom->lastEstimatedState(); optPoseAndTwist) {
      const auto & [pose, tw] = optPoseAndTwist.value();
      outTwist->insert(
        *lastObsTimestamp, mrpt::math::TPose3D(tw.vx, tw.vy, tw.vz, tw.wz, tw.wy, tw.wx));
    }
  }

  if (cli.arg_outPath.isSet()) {
    const auto fil = cli.arg_outPath.getValue();
    std::cout << "\nSaving estimated path in TUM format to: " << fil
              << std::endl;  // NOLINT(performance-avoid-endl)

    const mrpt::poses::CPose3DInterpolator lastEstimatedTrajectory = liodom->estimatedTrajectory();

    lastEstimatedTrajectory.saveToTextFile_TUM(fil);
  }

  if (cli.arg_outSimpleMap.isSet()) {
    const auto fil = cli.arg_outSimpleMap.getValue();

    auto sm = liodom->reconstructedMap();

    std::cout << "\nSaving reconstructed map with " << sm.size() << " keyframes to: " << fil
              << std::endl;  // NOLINT(performance-avoid-endl)

    sm.saveToFile(fil);
  }

  if (outTwist) {
    const auto fil = cli.arg_outTwist.getValue();
    std::cout << "\nSaving estimated twist to: " << fil
              << std::endl;  // NOLINT(performance-avoid-endl)
    outTwist->saveToTextFile(fil);
  }

  return 0;
}

// Prints the options whose value came from their envname() fallback rather
// than from the command line. CLI11 does not record the provenance, so this
// re-derives it: an option that ended up set, has an env fallback, that
// variable is present, and none of its flag spellings appears in argv.
void report_options_taken_from_env(const CLI::App & cmd, int argc, char ** argv)
{
  const std::vector<std::string> args(argv + 1, argv + argc);

  for (const auto * opt : cmd.get_options()) {
    const auto & envName = opt->get_envname();
    if (envName.empty() || opt->count() == 0) {
      continue;
    }
    if (::getenv(envName.c_str()) == nullptr) {
      continue;
    }
    bool onCommandLine = false;
    for (const auto & name : opt->get_lnames()) {
      const std::string flag = "--" + name;
      for (const auto & a : args) {
        if (a == flag || a.rfind(flag + "=", 0) == 0) {
          onCommandLine = true;
        }
      }
    }
    if (!onCommandLine) {
      std::cout << "Note: " << opt->get_name() << " taken from the environment ($" << envName
                << "): '" << opt->as<std::string>() << "'\n";
    }
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    Cli cli;

    // Parse arguments:
    try {
      cli.cmd.parse(argc, argv);
    } catch (const CLI::ParseError & e) {
      return cli.cmd.exit(e);
    }

    // Options that took their value from the environment change what this
    // run does without appearing anywhere in the command line, so say so.
    // Relevant because --lidar-sensor-label / --imu-sensor-label supersede
    // the pipeline YAML for every input source, rosbag or not.
    report_options_taken_from_env(cli.cmd, argc, argv);

    // Load plugins:
    if (cli.arg_plugins.isSet()) {
      std::string errMsg;
      const auto plugins = cli.arg_plugins.getValue();
      std::cout << "Loading plugin(s): " << plugins << "\n";
      if (!mrpt::system::loadPluginModules(plugins, errMsg)) {
        std::cerr << errMsg << std::endl;  // NOLINT(performance-avoid-endl)
        return 1;
      }
    }

    mola_install_signal_handler();

    main_odometry(cli);

    return 0;
  } catch (std::exception & e) {
    mola::pretty_print_exception(e, "Exit due to exception:");
    return 1;
  }
}
