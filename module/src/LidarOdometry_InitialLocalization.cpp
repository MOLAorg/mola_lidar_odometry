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
#include <mrpt/obs/CObservationIMU.h>

namespace mola
{

void LidarOdometry::handleInitialLocalizationDoInitFromPose(
  const mrpt::poses::CPose3DPDFGaussian & initPose, bool resetStateEstimator)
{
  if (resetStateEstimator) {
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
  }

  // also, keep it as the last pose for subsequent ICP runs:
  state_.last_lidar_pose = initPose;
}

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

      handleInitialLocalizationDoInitFromPose(initPose, true);
      doRemoveCloudsWithDecay();

      MRPT_LOG_INFO_STREAM("Initial re-localization done with pose: " << initPose.mean);

      state_.initial_localization_done = true;
    } break;

      // INITIALIZATION FROM STATE ESTIMATOR (GNSS, ETC.)
      // ---------------------------------------------------
    case mola::InitLocalization::FromStateEstimator: {
      handleInitialLocalizationStateEstimation();
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
#if defined(MOLA_IMU_PREINT_HAS_USE_IMU_ORIENT_PARAM)  // Remove in a few months from Dec 2025
        state_.imu_initializer->parameters.use_imu_orientation = il.use_imu_orientation;
#endif
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
        handleInitialLocalizationDoInitFromPose(initPose, true);
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

void LidarOdometry::onExternalMapUpdate(const MapSourceBase::MapUpdate & mu)
{
  // We're interested in geo-referencing updates
  if (!mu.georeferencing.has_value()) {
    return;
  }

  auto lck = mrpt::lockHelper(state_mtx_);

  // Store the external geo-reference
  state_.external_georef = mu.georeferencing;

  MRPT_LOG_INFO_STREAM(
    "Received external geo-referencing from state estimator: "
    << "lat=" << mu.georeferencing->geo_coord.lat.getAsString()
    << ", lon=" << mu.georeferencing->geo_coord.lon.getAsString()
    << ", T_enu_to_map=" << mu.georeferencing->T_enu_to_map.mean.asString());

  // If we don't have geo-ref in local map yet, set it:
  if (!state_.local_map->georeferencing.has_value()) {
    state_.local_map->georeferencing.emplace();
    state_.local_map->georeferencing->geo_coord = mu.georeferencing->geo_coord;
    state_.local_map->georeferencing->T_enu_to_map = mu.georeferencing->T_enu_to_map;

    state_.mark_local_map_georef_as_updated();
    MRPT_LOG_INFO("Applied external geo-referencing to local map");
  }
}

void LidarOdometry::onExternalLocalizationUpdate(  // NOLINT
  const LocalizationSourceBase::LocalizationUpdate & lu)
{
  // This callback could be used for real-time monitoring of state estimator
  // convergence, but the main check happens in handleInitialLocalization()
  (void)lu;
}

void LidarOdometry::handleInitialLocalizationStateEstimation()
{
  auto & il = params_.initial_localization;

  ASSERT_(state_.navstate_fuse);

  // Track when we started waiting
  if (!state_.waiting_for_state_estimator_since) {
    state_.waiting_for_state_estimator_since = mrpt::Clock::now();
    MRPT_LOG_INFO("Waiting for state estimator to converge (GNSS+IMU fusion)...");
  }

  // Check timeout
  const double elapsed =
    mrpt::system::timeDifference(*state_.waiting_for_state_estimator_since, mrpt::Clock::now());

  if (il.from_state_estimator_timeout > 0 && elapsed > il.from_state_estimator_timeout) {
    MRPT_LOG_ERROR_FMT(
      "Timeout (%.1f s) waiting for state estimator to converge. "
      "Check GNSS/IMU sensors.",
      il.from_state_estimator_timeout);
    // Could fall back to another method or throw an error
    THROW_EXCEPTION("State estimator convergence timeout");
  }

  // Check if state estimator has converged
  mrpt::poses::CPose3DPDFGaussian estimatedPose;

  // Snapshot georef state under lock to avoid data race with
  // onExternalMapUpdate which writes state_.external_georef concurrently.
  {
    auto lckState = mrpt::lockHelper(state_mtx_);

    const bool hasGeoRef =
      state_.local_map->georeferencing.has_value() || state_.external_georef.has_value();

    if (!hasGeoRef) {
      MRPT_LOG_THROTTLE_INFO(
        5.0, "Waiting for geo-referencing (from loaded map or state estimator)...");
      return;  // Not ready yet
    }

    // Apply external georef to local map if needed
    if (!state_.local_map->georeferencing.has_value() && state_.external_georef.has_value()) {
      state_.local_map->georeferencing.emplace();
      state_.local_map->georeferencing->geo_coord = state_.external_georef->geo_coord;
      state_.local_map->georeferencing->T_enu_to_map = state_.external_georef->T_enu_to_map;
      state_.mark_local_map_georef_as_updated();
    }
  }

  // Now check if state estimator has converged
#if MOLA_VERSION_CHECK(2, 5, 0)
  bool converged = state_.navstate_fuse->has_converged_localization(estimatedPose);
#else
  bool converged = false;
  MRPT_LOG_THROTTLE_WARN(5.0, "Initialization from state estimation requires mola_kernel >= 2.5.0");
#endif

  // Additional covariance check with our own thresholds
  if (converged) {
    const double pos_sigma_max = std::sqrt(
      std::max({estimatedPose.cov(0, 0), estimatedPose.cov(1, 1), estimatedPose.cov(2, 2)}));
    const double ori_sigma_max_deg = mrpt::RAD2DEG(std::sqrt(
      std::max({estimatedPose.cov(3, 3), estimatedPose.cov(4, 4), estimatedPose.cov(5, 5)})));

    if (
      pos_sigma_max > il.from_state_estimator_max_position_sigma ||
      ori_sigma_max_deg > il.from_state_estimator_max_orientation_sigma_deg) {
      converged = false;
      MRPT_LOG_THROTTLE_DEBUG_FMT(
        5.0,
        "State estimator converged but uncertainty too high: "
        "pos_sigma=%.3f m (max=%.3f), ori_sigma=%.2f deg (max=%.2f)",
        pos_sigma_max, il.from_state_estimator_max_position_sigma, ori_sigma_max_deg,
        il.from_state_estimator_max_orientation_sigma_deg);
    }
  }

  if (!converged) {
    MRPT_LOG_THROTTLE_INFO_FMT(
      5.0, "Waiting for state estimator to converge (elapsed: %.1f s)...", elapsed);
    return;  // Not ready yet
  }

  // State estimator has converged - initialize!
  MRPT_LOG_INFO_STREAM(
    "State estimator converged after "
    << elapsed
    << " s. "
       "Initializing LiDAR odometry with pose: "
    << estimatedPose.mean << " (sigmas: " << std::sqrt(estimatedPose.cov(0, 0)) << ", "
    << std::sqrt(estimatedPose.cov(1, 1)) << ", " << std::sqrt(estimatedPose.cov(2, 2)) << " m)");

  handleInitialLocalizationDoInitFromPose(estimatedPose, false);
  doRemoveCloudsWithDecay();

  MRPT_TODO("Auto-transition into active after convergence");

  state_.initial_localization_done = true;
  state_.waiting_for_state_estimator_since.reset();
}

}  // namespace mola
