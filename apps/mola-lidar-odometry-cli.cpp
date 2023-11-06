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
#include <mrpt/obs/CRawlog.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/os.h>
#include <mrpt/system/progress.h>

#include <cstdlib>
#include <iostream>
#include <string>

// Declare supported cli switches ===========
static TCLAP::CmdLine cmd("mola-lidar-odometry-cli");

static TCLAP::ValueArg<std::string> argYAML(
    "c", "config", "Input YAML config file (required) (*.yml)", true, "",
    "demo.yml", cmd);

static TCLAP::ValueArg<std::string> argRawlog(
    "r", "rawlog", "Input dataset in rawlog format (required) (*.rawlog)", true,
    "dataset.rawlog", "dataset.rawlog", cmd);

static TCLAP::ValueArg<std::string> arg_verbosity_level(
    "v", "verbosity", "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)",
    false, "", "INFO", cmd);

static TCLAP::ValueArg<std::string> arg_plugins(
    "l", "load-plugins",
    "One or more (comma separated) *.so files to load as plugins", false,
    "foobar.so", "foobar.so", cmd);

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
