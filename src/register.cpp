/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   register.cpp
 * @brief  Register MOLA modules in the factory
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2018
 */

/** \defgroup mola_mola_slam_grp mola_lidar_odometry
 * MOLA module: LIDAR-based odometry systems.
 *
 */

#include <mola_lidar_odometry/LidarInertialOdometry.h>
#include <mola_lidar_odometry/OccGrid.h>
#include <mrpt/core/initializer.h>
#include <mrpt/rtti/CObject.h>

using namespace mola;

MRPT_INITIALIZER(do_register_mola_lidar_odometry)
{
    using mrpt::rtti::registerClass;

    // Register modules:
    MOLA_REGISTER_MODULE(LidarInertialOdometry);

    // and register RTTI info:
    registerClass(CLASS_ID(mola::OccGrid));
}
