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

/** \defgroup mola_mola_erathos_slam_grp mola_erathos_slam
 * MOLA module: The Erathos-SLAM system.
 *
 */

#include <mola_erathos_slam/LidarInertialOdometry.h>
#include <mola_erathos_slam/OccGrid.h>
#include <mrpt/core/initializer.h>
#include <mrpt/rtti/CObject.h>

using namespace mola::erathos;

MRPT_INITIALIZER(do_register_mola_erathos_slam)
{
    using mrpt::rtti::registerClass;

    // Register modules:
    MOLA_REGISTER_MODULE(LidarInertialOdometry);

    // and register RTTI info:
    registerClass(CLASS_ID(mola::erathos::OccGrid));
}