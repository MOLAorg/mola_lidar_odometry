// -----------------------------------------------------------------------------
//   A Modular Optimization framework for Localization and mApping  (MOLA)
//
// Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
// Licensed under the GNU GPL v3.
//
// This file is part of MOLA.
// MOLA is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// MOLA. If not, see <https://www.gnu.org/licenses/>.
//
// Closed-source licenses available upon request, for this odometry package
// alone or in combination with the complete SLAM system.
// -----------------------------------------------------------------------------

/**
 * @file   mola-lidar-odometry-cli.cpp
 * @brief  main() for the cli app running lidar-inertial odometry for
 *         offline datasets.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 22, 2023
 */

#include <mola_kernel/interfaces/OfflineDatasetSource.h>
#include <mola_kernel/pretty_print_exception.h>
#include <mola_lidar_odometry/LidarOdometry.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationGPS.h>
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

#if defined(HAVE_MOLA_INPUT_PARIS_LUCO)
#include <mola_input_paris_luco_dataset/ParisLucoDataset.h>
#endif

#include <csignal>  // sigaction
#include <cstdlib>
#include <iostream>
#include <string>

struct Cli
{
  // Declare supported cli switches ===========
  TCLAP::CmdLine cmd{"mola-lidar-odometry-cli"};

  TCLAP::ValueArg<std::string> argYAML{
    "c", "config", "Input YAML config file (required) (*.yml)", true, "", "demo.yml", cmd};

  TCLAP::ValueArg<std::string> arg_verbosity_level{
    "v",    "verbosity", "Verbosity level: ERROR|WARN|INFO|DEBUG {Default: INFO}", false, "",
    "INFO", cmd};

  TCLAP::ValueArg<std::string> arg_plugins{
    "l",   "load-plugins", "One or more {comma separated} *.so files to load as plugins",
    false, "foobar.so",    "foobar.so",
    cmd};

  TCLAP::ValueArg<std::string> arg_outPath{
    "",
    "output-tum-path",
    "Save the estimated path as a TXT file using the TUM file format {see evo "
    "docs}",
    false,
    "output-trajectory.txt",
    "output-trajectory.txt",
    cmd};

  TCLAP::ValueArg<std::string> arg_outTwist{
    "",    "output-twist",     "Save the estimated twist as a TXT file",
    false, "output-twist.txt", "output-twist.txt",
    cmd};

  TCLAP::ValueArg<std::string> arg_outSimpleMap{
    "",
    "output-simplemap",
    "Enables building and saving the simplemap for the mapping session",
    false,
    "output-map.simplemap",
    "output-map.simplemap",
    cmd};

  TCLAP::ValueArg<int> arg_firstN{
    "",
    "only-first-n",
    "Run for the first N steps only {0=default, not used}",
    false,
    0,
    "Number of dataset entries to run",
    cmd};

  TCLAP::ValueArg<int> arg_skipFirstN{
    "",
    "skip-first-n",
    "Skip the first N dataset entries {0=default, not used}",
    false,
    0,
    "Number of dataset entries to skip",
    cmd};

  TCLAP::ValueArg<std::string> arg_lidarLabel{
    "",
    "lidar-sensor-label",
    "If provided, this supersedes the values in the 'lidar_sensor_labels' "
    "entry of the odometry pipeline, defining the sensorLabel/topic name to "
    "read LIDAR data from. It can be a regular expression {std::regex}",
    false,
    "lidar1",
    "lidar1",
    cmd};

// Input dataset can come from one of these:
// --------------------------------------------
#if defined(HAVE_MOLA_INPUT_RAWLOG)
  TCLAP::ValueArg<std::string> argRawlog{
    "",    "input-rawlog",   "INPUT DATASET: rawlog. Input dataset in rawlog format {*.rawlog}",
    false, "dataset.rawlog", "dataset.rawlog",
    cmd};
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
  TCLAP::ValueArg<std::string> argRosbag2{
    "",    "input-rosbag2", "INPUT DATASET: rosbag2. Input dataset in rosbag2 format {*.mcap}",
    false, "dataset.mcap",  "dataset.mcap",
    cmd};
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
  TCLAP::ValueArg<std::string> argKittiSeq{
    "",
    "input-kitti-seq",
    "INPUT DATASET: Use KITTI dataset sequence number 00|01|...",
    false,
    "00",
    "00",
    cmd};
  TCLAP::ValueArg<double> argKittiAngleDeg{
    "",
    "kitti-correction-angle-deg",
    "Correction vertical angle offset {see Deschaud,2018}",
    false,
    0.205,
    "0.205 [degrees]",
    cmd};
#endif

#if defined(HAVE_MOLA_INPUT_KITTI360)
  TCLAP::ValueArg<std::string> argKitti360Seq{
    "",
    "input-kitti360-seq",
    "INPUT DATASET: Use KITTI360 dataset sequence number 00|01|...|test_00|...",
    false,
    "00",
    "00",
    cmd};
#endif

#if defined(HAVE_MOLA_INPUT_MULRAN)
  TCLAP::ValueArg<std::string> argMulranSeq{
    "",    "input-mulran-seq", "INPUT DATASET: Use Mulran dataset sequence KAIST01|KAIST01|...",
    false, "KAIST01",          "KAIST01",
    cmd};
#endif

#if defined(HAVE_MOLA_INPUT_PARIS_LUCO)
  TCLAP::SwitchArg argParisLucoSeq{
    "", "input-paris-luco", "INPUT DATASET: Use Paris Luco dataset (unique sequence=00)", cmd};
#endif

};  // end struct "Cli"

namespace
{
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
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      rosbag_filename: '%s'
      base_link_frame_id: 'base_link'
      sensors:
        - topic: '%s'
          type: CObservationPointCloud
          # If present, this will override whatever /tf tells about the sensor pose:
          fixed_sensor_pose: "${LIDAR_POSE_X|0} ${LIDAR_POSE_Y|0} ${LIDAR_POSE_Z|0} ${LIDAR_POSE_YAW|0} ${LIDAR_POSE_PITCH|0} ${LIDAR_POSE_ROLL|0}"  # 'x y z yaw_deg pitch_deg roll_deg'
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_LIDAR_POSE|false}
        - topic: ${MOLA_GNSS_TOPIC|'/gps'}
          sensorLabel: 'gps'
          type: CObservationGPS
          fixed_sensor_pose: "${GPS_POSE_X|0} ${GPS_POSE_Y|0} ${GPS_POSE_Z|0} ${GPS_POSE_YAW|0} ${GPS_POSE_PITCH|0} ${GPS_POSE_ROLL|0}"  # 'x y z yaw_deg pitch_deg roll_deg'
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_GNSS_POSE|false}
)"""",
    rosbag2file.c_str(), cli.arg_lidarLabel.getValue().c_str())));

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

  if (cli.argKittiAngleDeg.isSet())
    o->VERTICAL_ANGLE_OFFSET = mrpt::DEG2RAD(cli.argKittiAngleDeg.getValue());

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
  std::cerr << "Caught signal " << s << ". Shutting down..." << std::endl;
  exit(0);
}

void mola_install_signal_handler()
{
  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = &mola_signal_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, nullptr);
}

int main_odometry(Cli & cli)
{
  mola::LidarOdometry liodom;

  mrpt::system::VerbosityLevel logLevel = liodom.getMinLoggingLevel();
  if (cli.arg_verbosity_level.isSet()) {
    using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
    logLevel = vl::name2value(cli.arg_verbosity_level.getValue());
    liodom.setVerbosityLevel(logLevel);
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
  liodom.mrpt::system::COutputLogger::logRegisterCallback(
    [&](
      [[maybe_unused]] std::string_view msg, const mrpt::system::VerbosityLevel level,
      [[maybe_unused]] std::string_view loggerName,
      [[maybe_unused]] const mrpt::Clock::time_point timestamp) {
      if (level < liodom.getMinLoggingLevel()) return;
      mark_emitted_log();
    });

  // Initialize LiDAR Odometry:
  const auto file_yml = cli.argYAML.getValue();
  const auto cfg = mola::load_yaml_file(file_yml);

  // Enable time profiling: // can be enabled via YAML options
  // liodom.profiler_.enable();

  // liodom.initialize_common(cfg); // can be skipped for a non-MOLA
  // system
  liodom.initialize(cfg);

  if (cli.arg_outSimpleMap.isSet()) {
    liodom.params_.simplemap.generate = true;
    // don't save within the LidarOdometry object, we will do it here in
    // this cli app:
    liodom.params_.simplemap.save_final_map_to_file.clear();
  }

  if (cli.arg_lidarLabel.isSet())
    liodom.params_.lidar_sensor_labels.assign(1, std::regex(cli.arg_lidarLabel.getValue()));

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
  if (cli.arg_outTwist.isSet()) outTwist.emplace();

  // Save GT, if available:
  if (cli.arg_outPath.isSet() && dataset->hasGroundTruthTrajectory()) {
    using namespace std::string_literals;

    const auto gtPath = dataset->getGroundTruthTrajectory();

    const auto gtOutFile = mrpt::system::fileNameChangeExtension(cli.arg_outPath.getValue(), "") +
                           "_gt."s + mrpt::system::extractFileExtension(cli.arg_outPath.getValue());

    std::cout << "Ground truth available. Saving it to: " << gtOutFile << std::endl;

    gtPath.saveToTextFile_TUM(gtOutFile);
  }

  const double tStart = mrpt::Clock::nowDouble();

  size_t lastDatasetEntry = dataset->datasetSize();
  size_t firstDatasetEntry = 0;

  if (cli.arg_skipFirstN.isSet()) firstDatasetEntry = cli.arg_skipFirstN.getValue();

  if (cli.arg_firstN.isSet()) lastDatasetEntry = firstDatasetEntry + cli.arg_firstN.getValue();

  mrpt::keep_min(lastDatasetEntry, dataset->datasetSize());

  std::cout << "\n";  // Needed for the VT100 codes below.

  // Run:
  for (size_t i = firstDatasetEntry; i < lastDatasetEntry; i++) {
    // Get observations from the dataset:
    using mrpt::obs::CObservation2DRangeScan;
    using mrpt::obs::CObservation3DRangeScan;
    using mrpt::obs::CObservationGPS;
    using mrpt::obs::CObservationOdometry;
    using mrpt::obs::CObservationPointCloud;
    using mrpt::obs::CObservationRotatingScan;
    using mrpt::obs::CObservationVelodyneScan;

    const auto sf = dataset->datasetGetObservations(i);
    ASSERT_(sf);

    mrpt::obs::CObservation::Ptr obs;
    obs = sf->getObservationByClass<CObservationRotatingScan>();
    if (!obs) obs = sf->getObservationByClass<CObservationPointCloud>();
    if (!obs) obs = sf->getObservationByClass<CObservation3DRangeScan>();
    if (!obs) obs = sf->getObservationByClass<CObservation2DRangeScan>();
    if (!obs) obs = sf->getObservationByClass<CObservationVelodyneScan>();
    if (!obs) obs = sf->getObservationByClass<CObservationGPS>();
    if (!obs) obs = sf->getObservationByClass<CObservationOdometry>();

    if (!obs) continue;

    // Send it to the odometry pipeline:
    liodom.onNewObservation(obs);

    // Show stats:
    static int cnt = 0;
    if (cnt++ % 100 == 0) {
      cnt = 0;
      const size_t N = (dataset->datasetSize() - 1);
      const double pc = (1.0 * i) / N;

      const double tNow = mrpt::Clock::nowDouble();
      const double ETA = pc > 0 ? (tNow - tStart) * (1.0 / pc - 1) : .0;
      const double totalTime = ETA + (tNow - tStart);

      // VT100 codes: cursor up and clear line
      if (!has_emitted_log()) std::cout << "\033[A\33[2KT\r";
      unmark_emitted_log();

      std::cout << mrpt::system::progress(pc, 30)
                << mrpt::format(
                     " %6zu/%6zu (%.02f%%) ETA=%s / T=%s\n", i, N, 100 * pc,
                     mrpt::system::formatTimeInterval(ETA).c_str(),
                     mrpt::system::formatTimeInterval(totalTime).c_str());
      std::cout.flush();
    }

    while (liodom.isBusy()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Keep track of vehicle velocities?
    if (outTwist) {
      if (const auto optPoseAndTwist = liodom.lastEstimatedState(); optPoseAndTwist) {
        const auto & [pose, tw] = optPoseAndTwist.value();
        outTwist->insert(
          obs->timestamp, mrpt::math::TPose3D(tw.vx, tw.vy, tw.vz, tw.wz, tw.wy, tw.wx));
      }
    }
  }

  if (cli.arg_outPath.isSet()) {
    const auto fil = cli.arg_outPath.getValue();
    std::cout << "\nSaving estimated path in TUM format to: " << fil << std::endl;

    mrpt::poses::CPose3DInterpolator lastEstimatedTrajectory = liodom.estimatedTrajectory();

    lastEstimatedTrajectory.saveToTextFile_TUM(fil);
  }

  if (cli.arg_outSimpleMap.isSet()) {
    const auto fil = cli.arg_outSimpleMap.getValue();

    auto sm = liodom.reconstructedMap();

    std::cout << "\nSaving reconstructed map with " << sm.size() << " keyframes to: " << fil
              << std::endl;

    sm.saveToFile(fil);
  }

  if (outTwist) {
    const auto fil = cli.arg_outTwist.getValue();
    std::cout << "\nSaving estimated twist to: " << fil << std::endl;
    outTwist->saveToTextFile(fil);
  }

  return 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    Cli cli;

    // Parse arguments:
    if (!cli.cmd.parse(argc, argv)) return 1;  // should exit.

    // Load plugins:
    if (cli.arg_plugins.isSet()) {
      std::string errMsg;
      const auto plugins = cli.arg_plugins.getValue();
      std::cout << "Loading plugin(s): " << plugins << std::endl;
      if (!mrpt::system::loadPluginModules(plugins, errMsg)) {
        std::cerr << errMsg << std::endl;
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
