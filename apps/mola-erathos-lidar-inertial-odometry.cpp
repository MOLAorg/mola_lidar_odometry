/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   mola-erathos-lidar-inertial-odometry.cpp
 * @brief  main() for the cli app running lidar-inertial odom on offline
 * datasets
 * @author Jose Luis Blanco Claraco
 * @date   Sep 22, 2023
 */

#include <mola_erathos_slam/LidarInertialOdometry.h>
#include <mola_kernel/pretty_print_exception.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/progress.h>

#include <cstdlib>
#include <iostream>
#include <string>

// Declare supported cli switches ===========
static TCLAP::CmdLine cmd("mola-erathos-lidar-inertial-odometry");

static TCLAP::ValueArg<std::string> argYAML(
    "c", "config", "Input YAML config file (required) (*.yml)", true, "",
    "demo.yml", cmd);

static TCLAP::ValueArg<std::string> argRawlog(
    "r", "rawlog", "Input dataset in rawlog format (required) (*.rawlog)", true,
    "dataset.rawlog", "dataset.rawlog", cmd);

static TCLAP::ValueArg<std::string> arg_verbosity_level(
    "v", "verbosity", "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)",
    false, "", "INFO", cmd);

static int main_odometry()
{
    mola::erathos::LidarInertialOdometry liodom;

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

    // liodom.initialize_common(cfg); // can be skipped for a non-MOLA system
    liodom.initialize(cfg);

    // Load dataset:
    std::cout << "Loading dataset: " << argRawlog.getValue() << std::endl;

    mrpt::obs::CRawlog dataset;
    dataset.loadFromRawLogFile(argRawlog.getValue());

    std::cout << "Dataset loaded (" << dataset.size() << " entries)."
              << std::endl;

    // Run:
    for (size_t i = 0; i < dataset.size(); i++)
    {
        const mrpt::serialization::CSerializable::Ptr obj =
            dataset.getAsGeneric(i);
        const auto obs =
            std::dynamic_pointer_cast<mrpt::obs::CObservation>(obj);
        if (!obs) continue;

        liodom.onNewObservation(obs);

        static int cnt = 0;
        if (cnt++ % 10 == 0)
        {
            cnt = 0;
            std::cout << mrpt::system::progress(
                             1.0 * i / (dataset.size() - 1), 30)
                      << "\r";
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

        main_odometry();

        return 0;
    }
    catch (std::exception& e)
    {
        mola::pretty_print_exception(e, "Exit due to exception:");
        return 1;
    }
}
