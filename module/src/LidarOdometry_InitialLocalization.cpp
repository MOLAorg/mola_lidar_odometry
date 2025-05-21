// -----------------------------------------------------------------------------
//   A Modular Optimization framework for Localization and mApping  (MOLA)
//
// Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
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
 * @file   LidarOdometry.cpp
 * @brief  Main C++ class exposing LIDAR odometry
 * @author Jose Luis Blanco Claraco
 * @date   Sep 16, 2023
 */

// This module:
#include <mola_lidar_odometry/LidarOdometry.h>

namespace mola
{

void LidarOdometry::handleInitialLocalization()
{
  auto & il = params_.initial_localization;

  switch (params_.initial_localization.method) {
      // FIXED POSE INITIALIZATION
      // ------------------------------
    case mola::InitLocalization::FixedPose: {
      mrpt::poses::CPose3DPDFGaussian initPose;
      initPose.mean = mrpt::poses::CPose3D(il.fixed_initial_pose);
      if (!il.initial_pose_cov) {
        initPose.cov.setDiagonal(1e-12);
      } else {
        initPose.cov = *il.initial_pose_cov;
      }

      ASSERT_(state_.navstate_fuse);
      state_.navstate_fuse->reset();  // needed after a re-localization to forget the past

      // Fake an evolution to be able to have an initial velocity estimation:
      // Use a tiny time step to let the filter remain with a large uncertainty about twist:
      ASSERT_(state_.last_obs_timestamp.has_value());
      const auto t1 =
        mrpt::Clock::fromDouble(mrpt::Clock::toDouble(*state_.last_obs_timestamp) - 2e-3);
      const auto t2 =
        mrpt::Clock::fromDouble(mrpt::Clock::toDouble(*state_.last_obs_timestamp) - 1e-3);
      state_.navstate_fuse->fuse_pose(t1, initPose, params_.publish_reference_frame);
      state_.navstate_fuse->fuse_pose(t2, initPose, params_.publish_reference_frame);

#if 0
      // And now, fake a twist estimation with a large covariance to make sure the filter does not become overconfident on it starting with zero velocity:
      auto twistCov = mrpt::math::CMatrixDouble66::Identity();
      twistCov *= 1e3;
      state_.navstate_fuse->fuse_twist(t2, mrpt::math::TTwist3D(), twistCov);
#endif

      // also, keep it as the last pose for subsequent ICP runs:
      state_.last_lidar_pose = initPose;

      MRPT_LOG_INFO_STREAM("Initial re-localization done with pose: " << initPose.mean);

      state_.initial_localization_done = true;
    } break;

      // INITIALIZATION FROM STATE ESTIMATOR (GNSS, ETC.)
      // ---------------------------------------------------
    case mola::InitLocalization::FromStateEstimator: {
      THROW_EXCEPTION("Write me!");
    } break;

      // INITIALIZATION FROM IMU FOR PITCH & ROLL ONLY
      // ---------------------------------------------------
    case mola::InitLocalization::PitchAndRollFromIMU: {
      THROW_EXCEPTION("Write me!");
    } break;

    default:
      THROW_EXCEPTION("Unknown value for initial_localization.method");
  };
}

}  // namespace mola
