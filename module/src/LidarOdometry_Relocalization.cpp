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
// Public entry points: non-blocking wrappers that dispatch the work to the
// internal lidar worker thread via enqueue_request(). The caller (typically
// a ROS callback thread) does not take state_mtx_, which decouples it from
// whatever the main processing loop is currently doing.

void LidarOdometry::relocalize_near_pose_pdf(const mrpt::poses::CPose3DPDFGaussian & p)
{
  this->enqueue_request([this, p]() { this->relocalize_near_pose_pdf_impl(p); });
}

void LidarOdometry::relocalize_from_gnss()
{
  this->enqueue_request([this]() { this->relocalize_from_gnss_impl(); });
}

// Synchronous bodies: run on the lidar worker thread (or spin thread) via
// processPendingUserRequests(). They take state_mtx_ themselves.

void LidarOdometry::relocalize_near_pose_pdf_impl(const mrpt::poses::CPose3DPDFGaussian & p)
{
  auto lckState = mrpt::lockHelper(state_mtx_);

  auto & il = params_.initial_localization;

  // Enforce re-localization on the next iteration:
  state_.initial_localization_done = false;
  // In this pose:
  il.fixed_initial_pose = p.mean.asTPose();
  il.initial_pose_cov = p.cov;
  // An explicit pose request must also select the method that consumes it:
  // otherwise, a system configured to initialize from the state estimator
  // (or from the IMU) would ignore the pose and keep waiting for its own
  // source to converge, while the `initial_localization_done = false` above
  // already stopped scans from being processed. That is precisely the case
  // where a manual re-localization is needed, e.g. poor GNSS coverage.
  il.method = InitLocalization::FixedPose;

  // Reset adaptive sigma to initial value:
  state_.adapt_thres_sigma = std::sqrt(p.cov.asEigen().block<2, 2>(0, 0).trace());

  // Handle uncertainty in next steps:
  state_.step_counter_post_relocalization = std::max(
    il.additional_uncertainty_after_reloc_how_many_timesteps,
    il.additional_map_freeze_after_reloc_how_many_timesteps);

  MRPT_LOG_INFO_STREAM(
    "relocalize_near_pose_pdf(): Using thres_sigma=" << state_.adapt_thres_sigma
                                                     << " to relocalize near: " << p.mean);
}

void LidarOdometry::relocalize_from_gnss_impl()
{
  MRPT_LOG_INFO("relocalize_from_gnss() called");

  auto lckState = mrpt::lockHelper(state_mtx_);

  auto & il = params_.initial_localization;

  // Enforce re-localization on the next iteration:
  state_.initial_localization_done = false;
  il.method = InitLocalization::FromStateEstimator;

  // Reset adaptive sigma to initial value:
  state_.adapt_thres_sigma = params_.adaptive_threshold.initial_sigma;

  // Handle uncertainty in next steps:
  state_.step_counter_post_relocalization = std::max(
    il.additional_uncertainty_after_reloc_how_many_timesteps,
    il.additional_map_freeze_after_reloc_how_many_timesteps);
}

}  // namespace mola
