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

#include <mola_kernel/pretty_print_exception.h>
#include <mola_lidar_odometry/LidarInertialOdometry.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/os.h>
#include <mrpt/system/progress.h>

#if defined(HAVE_MOLA_INPUT_KITTI)
#include <mola_input_kitti_dataset/KittiOdometryDataset.h>
#endif

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

// Input dataset can come from one of these:
static TCLAP::ValueArg<std::string> argRawlog(
    "", "input-rawlog",
    "INPUT DATASET: rawlog. Input dataset in rawlog format (*.rawlog)", false,
    "dataset.rawlog", "dataset.rawlog", cmd);

#if defined(HAVE_MOLA_INPUT_KITTI)
static TCLAP::ValueArg<std::string> argKittiSeq(
    "", "input-kitti-seq",
    "INPUT DATASET: Use KITTI dataset sequence number 00|01|...", false, "00",
    "00", cmd);
#endif

class OfflineDatasetSource
{
   public:
    OfflineDatasetSource()          = default;
    virtual ~OfflineDatasetSource() = default;

    virtual size_t size() const = 0;

    virtual mrpt::obs::CObservation::Ptr getObservation(size_t index) const = 0;
};

class RawlogSource : public OfflineDatasetSource
{
   public:
    RawlogSource()          = default;
    virtual ~RawlogSource() = default;

    void init(const std::string& rawlogFile)
    {
        // Load dataset:
        std::cout << "Loading dataset: " << rawlogFile << std::endl;
        dataset_.loadFromRawLogFile(rawlogFile);
        std::cout << "Dataset loaded (" << dataset_.size() << " entries)."
                  << std::endl;
    }

    size_t size() const override { return dataset_.size(); }

    mrpt::obs::CObservation::Ptr getObservation(size_t index) const override
    {
        const mrpt::serialization::CSerializable::Ptr obj =
            dataset_.getAsGeneric(index);
        const auto obs =
            std::dynamic_pointer_cast<mrpt::obs::CObservation>(obj);
        return obs;
    }

   private:
    mrpt::obs::CRawlog dataset_;
};

#if defined(HAVE_MOLA_INPUT_KITTI)
class KittiSource : public OfflineDatasetSource
{
   public:
    KittiSource()          = default;
    virtual ~KittiSource() = default;

    void init(const std::string& kittiSeqNumber)
    {
        const auto kittiCfg =
            mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
                R""""(
    params:
      base_dir: ${KITTI_BASE_DIR}
      sequence: '%s'
      time_warp_scale: 1.0
      publish_lidar: true
      publish_image_0: true
      publish_image_1: true
      publish_ground_truth: true
)"""",
                kittiSeqNumber.c_str())));

        kittiDataset_.initialize(kittiCfg);
    }

    size_t size() const override { return kittiDataset_.getTimestepCount(); }

    mrpt::obs::CObservation::Ptr getObservation(size_t index) const override
    {
        return kittiDataset_.getPointCloud(index);
    }

   private:
    mutable mola::KittiOdometryDataset kittiDataset_;
};

#endif

static int main_odometry()
{
    mola::LidarInertialOdometry liodom;

    // Define the verbosity level here so it affects all possible
    // commands of mola-cli:
    if (arg_verbosity_level.isSet())
    {
        using vl     = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
        const auto v = vl::name2value(arg_verbosity_level.getValue());
        liodom.setVerbosityLevel(v);
    }

    // Initialize:
    const auto file_yml = argYAML.getValue();
    const auto cfg      = mola::load_yaml_file(file_yml);

    // Enable time profiling:
    liodom.profiler_.enable();

    // liodom.initialize_common(cfg); // can be skipped for a non-MOLA
    // system
    liodom.initialize(cfg);

    // Select dataset input:
    std::shared_ptr<OfflineDatasetSource> dataset;

    if (argRawlog.isSet())
    {
        auto ds = std::make_shared<RawlogSource>();
        ds->init(argRawlog.getValue());

        dataset = ds;
    }
#if defined(HAVE_MOLA_INPUT_KITTI)
    else if (argKittiSeq.isSet())
    {
        auto ds = std::make_shared<KittiSource>();
        ds->init(argKittiSeq.getValue());

        dataset = ds;
    }
#endif
    else
    {
        THROW_EXCEPTION(
            "At least one of the dataset input CLI flags must be defined. "
            "Use --help.");
    }

    // Run:
    for (size_t i = 0; i < dataset->size(); i++)
    {
        const auto obs = dataset->getObservation(i);
        if (!obs) continue;

        liodom.onNewObservation(obs);

        static int cnt = 0;
        if (cnt++ % 10 == 0)
        {
            cnt = 0;
            std::cout << mrpt::system::progress(
                             1.0 * i / (dataset->size() - 1), 30)
                      << "\r";
        }

        while (liodom.isBusy())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
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

        main_odometry();

        return 0;
    }
    catch (std::exception& e)
    {
        mola::pretty_print_exception(e, "Exit due to exception:");
        return 1;
    }
}
