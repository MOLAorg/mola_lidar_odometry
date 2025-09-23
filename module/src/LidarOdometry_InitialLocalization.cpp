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
#include <mrpt/obs/CObservationIMU.h>

namespace mola
{

void LidarOdometry::handleInitialLocalization()
{
  auto & il = params_.initial_localization;

  auto lambdaInitFromPose = [this](const mrpt::poses::CPose3DPDFGaussian & initPose) {
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
    // also, keep it as the last pose for subsequent ICP runs:
    state_.last_lidar_pose = initPose;
  };

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

      lambdaInitFromPose(initPose);
      doRemoveCloudsWithDecay();

#if 0
      // And now, fake a twist estimation with a large covariance to make sure the filter does not become overconfident on it starting with zero velocity:
      auto twistCov = mrpt::math::CMatrixDouble66::Identity();
      twistCov *= 1e3;
      state_.navstate_fuse->fuse_twist(t2, mrpt::math::TTwist3D(), twistCov);
#endif

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
      // Construct the IMU averager object:
      if (!state_.imu_initializer) {
        state_.imu_initializer = mola::imu::ImuInitialCalibrator();
        state_.imu_initializer->parameters.max_samples_age = il.imu_initial_calibration_max_age;
        state_.imu_initializer->parameters.required_samples =
          il.imu_initial_calibration_sample_count;
      }
      // Already collected enough samples?
      if (state_.imu_initializer->isReady()) {
        // Get the average pitch & roll:
        const auto & imu_calib_opt = state_.imu_initializer->getCalibration();
        ASSERT_(imu_calib_opt);

        MRPT_LOG_INFO_STREAM("IMU automatic calibration done: " << imu_calib_opt->asString());

        // Set as the initial pose:
        il.fixed_initial_pose =
          mrpt::math::TPose3D(0, 0, 0, 0, imu_calib_opt->pitch, imu_calib_opt->roll);

        MRPT_LOG_INFO_STREAM(
          "Initial re-localization done from IMU pitch/roll with pose: "
          << il.fixed_initial_pose.asString());

        mrpt::poses::CPose3DPDFGaussian initPose;
        initPose.mean = mrpt::poses::CPose3D(il.fixed_initial_pose);
        initPose.cov.setDiagonal(1e-12);
        lambdaInitFromPose(initPose);
        doRemoveCloudsWithDecay();

        state_.local_map->clear();
        ASSERT_(state_.local_map->empty());
        state_.reconstructed_simplemap.clear();

        state_.initial_localization_done = true;
        state_.imu_initializer.reset();  // no longer needed

      } else {
        MRPT_LOG_THROTTLE_INFO(
          5.0, "Waiting for enough IMU samples to initialize pitch & roll while stationary...");
      }
    } break;

    default:
      THROW_EXCEPTION("Unknown value for initial_localization.method");
  };
}

}  // namespace mola
