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
      if (!state_.imu_averager) {
        state_.imu_averager =
          ImuAverager(il.pitch_and_roll_from_imu_sample_count, il.pitch_and_roll_from_imu_max_age);
      }
      // Already collected enough samples?
      if (state_.imu_averager->isReady()) {
        // Get the average pitch & roll:
        const auto [pitch, roll] = state_.imu_averager->getPitchRoll();

        // Set as the initial pose:
        il.fixed_initial_pose = mrpt::math::TPose3D(0, 0, 0, 0, pitch, roll);

        MRPT_LOG_INFO_STREAM(
          "Initial re-localization done from IMU pitch/roll with pose: "
          << il.fixed_initial_pose.asString());

        mrpt::poses::CPose3DPDFGaussian initPose;
        initPose.mean = mrpt::poses::CPose3D(il.fixed_initial_pose);
        initPose.cov.setDiagonal(1e-12);
        lambdaInitFromPose(initPose);

        state_.local_map->clear();
        ASSERT_(state_.local_map->empty());
        state_.reconstructed_simplemap.clear();

        state_.initial_localization_done = true;
        state_.imu_averager.reset();  // no longer needed

      } else {
        MRPT_LOG_THROTTLE_INFO(
          5.0, "Waiting for enough IMU samples to initialize pitch & roll while stationary...");
      }
    } break;

    default:
      THROW_EXCEPTION("Unknown value for initial_localization.method");
  };
}

void LidarOdometry::ImuAverager::add(const std::shared_ptr<const mrpt::obs::CObservationIMU> & obs)
{
  // Add:
  ASSERT_(obs);
  samples_[mrpt::Clock::toDouble(obs->timestamp)] = obs;

  // Remove old samples:
  while (!samples_.empty() &&
         samples_.begin()->first < samples_.rbegin()->first - max_samples_age_) {
    samples_.erase(samples_.begin());
  }
}

bool LidarOdometry::ImuAverager::isReady() const
{
  // samples enough?
  return (samples_.size() >= required_samples_);
}

std::tuple<double, double> LidarOdometry::ImuAverager::getPitchRoll() const
{
  mrpt::math::TVector3D avr_accel(0, 0, 0);
  std::size_t count = 0;

  for (const auto & [_, imu] : samples_) {
    ASSERT_(imu);
    // TODO: Check for direct IMU-provided pitch/roll values?

    const auto accel_sensor = mrpt::math::TTwist3D(  //
      imu->get(mrpt::obs::IMU_X_ACC),                //
      imu->get(mrpt::obs::IMU_Y_ACC),                //
      imu->get(mrpt::obs::IMU_Z_ACC),                //
      0, 0, 0);

    // TODO: Minimum sanity check for the acceleration vector?

    const auto accel_base_link = accel_sensor.rotated(imu->sensorPose.asTPose());

    // Accumulate:
    avr_accel += mrpt::math::TVector3D(accel_base_link.vx, accel_base_link.vy, accel_base_link.vz);
    count++;
  }

  // Average:
  if (count > 0) {
    avr_accel *= 1.0 / static_cast<double>(count);
  }

  // Compute pitch & roll from the XYZ acceleration vector:
  const auto up_vector = avr_accel.unitarize();

  std::cout << "[getPitchRoll] down_vector: " << up_vector << std::endl;
  const double pitch = -std::asin(up_vector.x);
  const double roll = std::asin(up_vector.x);
  return {pitch, roll};
}

}  // namespace mola
