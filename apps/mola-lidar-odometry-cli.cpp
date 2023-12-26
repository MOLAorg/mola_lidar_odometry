/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */

/**
 * @file   mola-lidar-odometry-cli.cpp
 * @brief  main() for the cli app running lidar-inertial odometry for
 *         offline datasets.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 22, 2023
 */

#include <mola_kernel/interfaces/OfflineDatasetSource.h>
#include <mola_kernel/pretty_print_exception.h>
#include <mola_lidar_odometry/LidarInertialOdometry.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
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

#if defined(HAVE_MOLA_INPUT_MULRAN)
#include <mola_input_mulran_dataset/MulranDataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_RAWLOG)
#include <mola_input_rawlog/RawlogDataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
#include <mola_input_rosbag2/Rosbag2Dataset.h>
#endif

#include <csignal>  // sigaction
#include <cstdlib>
#include <iostream>
#include <string>

// Declare supported cli switches ===========
static TCLAP::CmdLine cmd("mola-lidar-odometry-cli");

static TCLAP::ValueArg<std::string> argYAML(
    "c", "config", "Input YAML config file (required) (*.yml)", true, "",
    "demo.yml", cmd);

static TCLAP::ValueArg<std::string> arg_verbosity_level(
    "v", "verbosity", "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)",
    false, "", "INFO", cmd);

static TCLAP::ValueArg<std::string> arg_plugins(
    "l", "load-plugins",
    "One or more (comma separated) *.so files to load as plugins", false,
    "foobar.so", "foobar.so", cmd);

static TCLAP::ValueArg<std::string> arg_outPath(
    "", "output-tum-path",
    "Save the estimated path as a TXT file using the TUM file format (see evo "
    "docs)",
    false, "output-trajectory.txt", "output-trajectory.txt", cmd);

static TCLAP::ValueArg<std::string> arg_outSimpleMap(
    "", "output-simplemap",
    "Enables building and saving the simplemap for the mapping session", false,
    "output-map.simplemap", "output-map.simplemap", cmd);

static TCLAP::ValueArg<int> arg_firstN(
    "", "only-first-n", "Run for the first N steps only (0=default, not used)",
    false, 0, "Number of dataset entries to run", cmd);

static TCLAP::ValueArg<std::string> arg_lidarLabel(
    "", "lidar-sensor-label",
    "If provided, this supersedes the values in the 'lidar_sensor_labels' "
    "entry of the odometry pipeline, defining the sensorLabel/topic name to "
    "read LIDAR data from. It can be a regular expression (std::regex)",
    false, "lidar1", "lidar1", cmd);

// Input dataset can come from one of these:
// --------------------------------------------
#if defined(HAVE_MOLA_INPUT_RAWLOG)
static TCLAP::ValueArg<std::string> argRawlog(
    "", "input-rawlog",
    "INPUT DATASET: rawlog. Input dataset in rawlog format (*.rawlog)", false,
    "dataset.rawlog", "dataset.rawlog", cmd);
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
static TCLAP::ValueArg<std::string> argRosbag2(
    "", "input-rosbag2",
    "INPUT DATASET: rosbag2. Input dataset in rosbag2 format (*.mcap)", false,
    "dataset.mcap", "dataset.mcap", cmd);
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
static TCLAP::ValueArg<std::string> argKittiSeq(
    "", "input-kitti-seq",
    "INPUT DATASET: Use KITTI dataset sequence number 00|01|...", false, "00",
    "00", cmd);
static TCLAP::ValueArg<double> argKittiAngleDeg(
    "", "kitti-correction-angle-deg",
    "Correction vertical angle offset (see Deschaud,2018)", false, 0.205,
    "0.205 [degrees]", cmd);
#endif

#if defined(HAVE_MOLA_INPUT_MULRAN)
static TCLAP::ValueArg<std::string> argMulranSeq(
    "", "input-mulran-seq",
    "INPUT DATASET: Use Mulran dataset sequence KAIST01|KAIST01|...", false,
    "KAIST01", "KAIST01", cmd);
#endif

#if defined(HAVE_MOLA_INPUT_RAWLOG)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_rawlog(
    const std::string& rawlogFile, const mrpt::system::VerbosityLevel logLevel)
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
    const std::string&                 mulranSequence,
    const mrpt::system::VerbosityLevel logLevel)
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
    const std::string& rosbag2file, const mrpt::system::VerbosityLevel logLevel)
{
    ASSERTMSG_(
        arg_lidarLabel.isSet(),
        "Using a rosbag2 as input requires telling what is the lidar topic "
        "with --lidar-sensor-label <TOPIC_NAME>");

    auto o = std::make_shared<mola::Rosbag2Dataset>();
    o->setMinLoggingLevel(logLevel);

    const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
        R""""(
    params:
      rosbag_filename: '%s'
      base_link_frame_id: 'base_footprint'
      sensors:
        - topic: '%s'
          type: CObservationPointCloud
          # If present, this will override whatever /tf tells about the sensor pose:
          fixed_sensor_pose: "0 0 0 0 0 0"  # 'x y z yaw_deg pitch_deg roll_deg'
)"""",
        rosbag2file.c_str(), arg_lidarLabel.getValue().c_str())));

    o->initialize(cfg);

    return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_kitti(
    const std::string&                 kittiSeqNumber,
    const mrpt::system::VerbosityLevel logLevel)
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

    if (argKittiAngleDeg.isSet())
        o->VERTICAL_ANGLE_OFFSET = mrpt::DEG2RAD(argKittiAngleDeg.getValue());

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

static int main_odometry()
{
    mola::LidarInertialOdometry liodom;

    mrpt::system::VerbosityLevel logLevel = liodom.getMinLoggingLevel();
    if (arg_verbosity_level.isSet())
    {
        using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
        logLevel = vl::name2value(arg_verbosity_level.getValue());
        liodom.setVerbosityLevel(logLevel);
    }

    // Initialize:
    const auto file_yml = argYAML.getValue();
    const auto cfg      = mola::load_yaml_file(file_yml);

    // Enable time profiling:
    liodom.profiler_.enable();

    // liodom.initialize_common(cfg); // can be skipped for a non-MOLA
    // system
    liodom.initialize(cfg);

    if (arg_outSimpleMap.isSet()) liodom.params_.simplemap.generate = true;

    if (arg_lidarLabel.isSet())
        liodom.params_.lidar_sensor_labels.assign(
            1, std::regex(arg_lidarLabel.getValue()));

    // Select dataset input:
    std::shared_ptr<mola::OfflineDatasetSource> dataset;

#if defined(HAVE_MOLA_INPUT_RAWLOG)
    if (argRawlog.isSet())
    {
        dataset = dataset_from_rawlog(argRawlog.getValue(), logLevel);
    }
    else
#endif
#if defined(HAVE_MOLA_INPUT_KITTI)
        if (argKittiSeq.isSet())
    {
        dataset = dataset_from_kitti(argKittiSeq.getValue(), logLevel);
    }
    else
#endif
#if defined(HAVE_MOLA_INPUT_MULRAN)
        if (argMulranSeq.isSet())
    {
        dataset = dataset_from_mulran(argMulranSeq.getValue(), logLevel);
    }
    else
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG2)
        if (argRosbag2.isSet())
    {
        dataset = dataset_from_rosbag2(argRosbag2.getValue(), logLevel);
    }
    else
#endif
    {
        THROW_EXCEPTION(
            "At least one of the dataset input CLI flags must be defined. "
            "Use --help.");
    }
    ASSERT_(dataset);

    // Save GT, if available:
    if (arg_outPath.isSet() && dataset->hasGroundTruthTrajectory())
    {
        using namespace std::string_literals;

        const auto gtPath = dataset->getGroundTruthTrajectory();

        const auto gtOutFile =
            mrpt::system::fileNameChangeExtension(arg_outPath.getValue(), "") +
            "_gt.txt"s;

        std::cout << "Ground truth available. Saving it to: " << gtOutFile
                  << std::endl;

        gtPath.saveToTextFile_TUM(gtOutFile);
    }

    const double tStart = mrpt::Clock::nowDouble();

    size_t nDatasetEntriesToRun = dataset->datasetSize();
    if (arg_firstN.isSet()) nDatasetEntriesToRun = arg_firstN.getValue();

    std::cout << "\n";  // Needed for the VT100 codes below.

    // Run:
    for (size_t i = 0; i < nDatasetEntriesToRun; i++)
    {
        // Get observations from the dataset:
        using namespace mrpt::obs;

        const auto sf = dataset->datasetGetObservations(i);
        ASSERT_(sf);

        CObservation::Ptr obs;
        obs = sf->getObservationByClass<CObservationRotatingScan>();
        if (!obs) obs = sf->getObservationByClass<CObservationPointCloud>();
        if (!obs) obs = sf->getObservationByClass<CObservation3DRangeScan>();
        if (!obs) obs = sf->getObservationByClass<CObservation2DRangeScan>();
        if (!obs) obs = sf->getObservationByClass<CObservationVelodyneScan>();

        if (!obs) continue;

        // Send it to the odometry pipeline:
        liodom.onNewObservation(obs);

        // Show stats:
        static int cnt = 0;
        if (cnt++ % 20 == 0)
        {
            cnt             = 0;
            const size_t N  = (dataset->datasetSize() - 1);
            const double pc = (1.0 * i) / N;

            const double tNow = mrpt::Clock::nowDouble();
            const double ETA  = pc > 0 ? (tNow - tStart) * (1.0 / pc - 1) : .0;
            const double totalTime = ETA + (tNow - tStart);

            std::cout
                << "\033[A\33[2KT\r"  // VT100 codes: cursor up and clear line
                << mrpt::system::progress(pc, 30)
                << mrpt::format(
                       " %6zu/%6zu (%.02f%%) ETA=%s / T=%s\n", i, N, 100 * pc,
                       mrpt::system::formatTimeInterval(ETA).c_str(),
                       mrpt::system::formatTimeInterval(totalTime).c_str());
            std::cout.flush();
        }

        while (liodom.isBusy())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    if (arg_outPath.isSet())
    {
        const auto fil = arg_outPath.getValue();
        std::cout << "\nSaving estimated path in TUM format to: " << fil
                  << std::endl;

        mrpt::poses::CPose3DInterpolator lastEstimatedTrajectory =
            liodom.estimatedTrajectory();

        lastEstimatedTrajectory.saveToTextFile_TUM(fil);
    }

    if (arg_outSimpleMap.isSet())
    {
        const auto fil = arg_outSimpleMap.getValue();

        auto sm = liodom.reconstructedMap();

        std::cout << "\nSaving reconstructed map with " << sm.size()
                  << " keyframes to: " << fil << std::endl;

        sm.saveToFile(fil);
    }

    return 0;
}

int main(int argc, char** argv)
{
    try
    {
        // Parse arguments:
        if (!cmd.parse(argc, argv)) return 1;  // should exit.

        // Load plugins:
        if (arg_plugins.isSet())
        {
            std::string errMsg;
            const auto  plugins = arg_plugins.getValue();
            std::cout << "Loading plugin(s): " << plugins << std::endl;
            if (!mrpt::system::loadPluginModules(plugins, errMsg))
            {
                std::cerr << errMsg << std::endl;
                return 1;
            }
        }

        mola_install_signal_handler();

        main_odometry();

        return 0;
    }
    catch (std::exception& e)
    {
        mola::pretty_print_exception(e, "Exit due to exception:");
        return 1;
    }
}
