/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

/**
 * @file   LidarOdometry.cpp
 * @brief  Main C++ class exposing LIDAR odometry
 * @author Jose Luis Blanco Claraco
 * @date   Sep 16, 2023
 */

// This module:
#include <mola_lidar_odometry/LidarOdometry.h>

// Eigen:
#include <Eigen/Core>

namespace mola
{
void LidarOdometry::relocalize_near_pose_pdf(const mrpt::poses::CPose3DPDFGaussian & p)
{
  auto lckState = mrpt::lockHelper(state_mtx_);

  auto & il = params_.initial_localization;

  // Enforce re-localizatio on the next iteration:
  state_.initial_localization_done = false;
  // In this pose:
  il.fixed_initial_pose = p.mean.asTPose();
  il.initial_pose_cov = p.cov;

  // Reset adaptative sigma to initial value:
  state_.adapt_thres_sigma = std::sqrt(p.cov.asEigen().block<2, 2>(0, 0).trace());

  // Handle uncertainty in next steps:
  state_.step_counter_post_relocalization =
    il.additional_uncertainty_after_reloc_how_many_timesteps;

  MRPT_LOG_INFO_STREAM(
    "relocalize_near_pose_pdf(): Using thres_sigma="  //
    << state_.adapt_thres_sigma << " to relocalize near: " << p.mean);
}

void LidarOdometry::relocalize_from_gnss()
{
  MRPT_LOG_INFO("relocalize_from_gnss() called");

  auto lckState = mrpt::lockHelper(state_mtx_);

  auto & il = params_.initial_localization;

  // Enforce re-localization on the next iteration:
  state_.initial_localization_done = false;
  il.method = InitLocalization::FromStateEstimator;

  // Reset adaptative sigma to initial value:
  state_.adapt_thres_sigma = params_.adaptive_threshold.initial_sigma;

  // Handle uncertainty in next steps:
  state_.step_counter_post_relocalization =
    il.additional_uncertainty_after_reloc_how_many_timesteps;
}

}  // namespace mola
