/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   LidarOdometry_GravityRebake.cpp
 * @brief  Online gravity tilt correction for the active local map.
 *
 * Implements LidarOdometry::maybeApplyGravityTiltCorrection().
 *
 * Design notes:
 *  - Reuses GravityMapAligner::rotationFromGravity() as a pure static helper
 *    to build the pitch/roll-only rotation.
 *  - GravityMapAligner and TrajectoryRebaker are NOT used for this online
 *    path (they were designed for an offline batch strategy).
 *  - The mola::KeyframeMapCapable interface (from mola_kernel) is guarded
 *    with __has_include so this TU builds cleanly without mola_metric_maps.
 */

#include <mola_lidar_odometry/GravityMapAligner.h>
#include <mola_lidar_odometry/LidarOdometry.h>

#if defined(__has_include)
#if __has_include(<mola_kernel/interfaces/KeyframeMapCapable.h>)
#include <mola_kernel/interfaces/KeyframeMapCapable.h>
#endif
#endif

#include <mrpt/math/utils.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/system/datetime.h>

#include <cmath>

using namespace mola;

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
bool LidarOdometry::maybeApplyGravityTiltCorrection()
{
#if defined(__has_include) && __has_include(<mola_kernel/interfaces/KeyframeMapCapable.h>)

  const auto & p = params_.imu_gravity_correction.tilt_rebake;

  // --- Throttling ---
  const auto now = mrpt::Clock::now();
  if (state_.last_gravity_rebake_tim.has_value()) {
    const double elapsed = mrpt::system::timeDifference(*state_.last_gravity_rebake_tim, now);
    if (elapsed < p.min_interval_sec) {
      return false;
    }
  }

  // --- Get gravity pitch/roll from IMU ---
  const auto gravityPR = state_.gravity_estimator.estimatedPitchRoll(
    std::min(params_.imu_gravity_correction.averaging_samples, 10u),
    params_.imu_gravity_correction.max_age_seconds);
  if (!gravityPR.has_value()) {
    return false;
  }
  const auto [imu_pitch, imu_roll] = *gravityPR;

  // --- Build gravity unit vector in vehicle frame ---
  // Convention (from estimatedPitchRoll): vehicle level => g_veh = +Z.
  const double g_veh_x = -std::sin(imu_pitch);
  const double g_veh_y = std::sin(imu_roll) * std::cos(imu_pitch);
  const double g_veh_z = std::cos(imu_roll) * std::cos(imu_pitch);
  const mrpt::math::TVector3D g_veh(g_veh_x, g_veh_y, g_veh_z);

  // --- Express gravity in world (local-map) frame ---
  const mrpt::math::CMatrixDouble33 R_world = state_.last_lidar_pose.mean.getRotationMatrix();
  const mrpt::math::TVector3D g_world{
    R_world(0, 0) * g_veh_x + R_world(0, 1) * g_veh_y + R_world(0, 2) * g_veh_z,
    R_world(1, 0) * g_veh_x + R_world(1, 1) * g_veh_y + R_world(1, 2) * g_veh_z,
    R_world(2, 0) * g_veh_x + R_world(2, 1) * g_veh_y + R_world(2, 2) * g_veh_z};

  // --- Compute tilt angle of world z-axis relative to gravity ---
  const double g_world_z_clamped = std::max(-1.0, std::min(1.0, g_world.z));
  const double tilt_rad = std::acos(g_world_z_clamped);
  MRPT_LOG_DEBUG_FMT("Tilt rebake, estimated tilt angle=%.02lf deg", mrpt::RAD2DEG(tilt_rad));
  if (mrpt::RAD2DEG(tilt_rad) < p.trigger_deg) {
    return false;
  }

  // --- Find a KeyframeMapCapable layer in the local map ---
  mola::KeyframeMapCapable * kfCap = nullptr;
  for (auto & [layerName, layerMap] : state_.local_map->layers) {
    auto * candidate = dynamic_cast<mola::KeyframeMapCapable *>(layerMap.get());
    if (candidate != nullptr) {
      kfCap = candidate;
      break;
    }
  }
  if (kfCap == nullptr) {
    if (!state_.gravity_rebake_unsupported_warned) {
      MRPT_LOG_WARN(
        "imu_gravity_correction.tilt_rebake is enabled, but the "
        "configured local map layer does not implement "
        "mola::KeyframeMapCapable; feature DISABLED for this run.");
      state_.gravity_rebake_unsupported_warned = true;
    }
    return false;
  }

  // --- Check minimum active keyframe count ---
  const auto kfPoses = kfCap->keyframePoses();
  if (kfPoses.size() < p.min_active_keyframes) {
    return false;
  }

  // --- Build the world-frame pitch/roll-only correction rotation ---
  // rotationFromGravity(g_world) returns a CPose3D with t=0 and a pure
  // pitch/roll rotation sending g_world -> +Z (yaw=0).
  const mrpt::poses::CPose3D R_correct = mola::GravityMapAligner::rotationFromGravity(g_world);

  // --- Pivot: oldest active KF ---
  const auto pivotId = kfPoses.begin()->first;
  const auto & T_pivot = kfPoses.begin()->second;

  // T_correct_world: rotation about pivot's position in world frame.
  // T_correct_world(p) = t_pivot + R_correct * (p - t_pivot)
  //                    = R_correct + (t_pivot - R_correct*t_pivot)
  // For applyPivotTransform: delta_at_pivot = T_pivot^{-1} * R_correct * T_pivot
  const mrpt::poses::CPose3D delta_at_pivot = (-T_pivot) + R_correct + T_pivot;

  // (i) Rotate active local-map KFs:
  kfCap->applyPivotTransform(pivotId, delta_at_pivot);

  // T_correct_world applies R_correct about the pivot position in world frame.
  // We need this explicit form to rotate all other cached state.
  // t_new = R_correct.R * (t - t_pivot) + t_pivot
  // As a CPose3D left-multiply: T_correct_world = T_pivot + R_correct + (-T_pivot)
  const mrpt::poses::CPose3D T_correct_world = T_pivot + R_correct + (-T_pivot);

  // (ii) In-progress simplemap KFs:
  {
    auto smLck = mrpt::lockHelper(state_simplemap_mtx_);
    for (size_t i = 0; i < state_.reconstructed_simplemap.size(); i++) {
      auto & kf = state_.reconstructed_simplemap.get(i);
      if (!kf.pose) {
        continue;
      }
      auto pdf = std::dynamic_pointer_cast<mrpt::poses::CPose3DPDFGaussian>(kf.pose);
      if (!pdf) {
        continue;
      }
      pdf->changeCoordinatesReference(T_correct_world);
    }
  }

  // (iii) Cached world-frame state:
  state_.last_lidar_pose.changeCoordinatesReference(T_correct_world);

  if (state_.last_motion_model_output.has_value()) {
    // Twist is body-local, do NOT rotate — only the pose.
    state_.last_motion_model_output->pose.changeCoordinatesReference(T_correct_world);
  }

  // Rotate every entry in estimated_trajectory:
  {
    auto lckTraj = mrpt::lockHelper(state_trajectory_mtx_);
    mrpt::poses::CPose3DInterpolator updated;
    for (const auto & [t, p_tpose] : state_.estimated_trajectory) {
      const mrpt::poses::CPose3D p_old(p_tpose);
      const mrpt::poses::CPose3D p_new = T_correct_world + p_old;
      updated.insert(t, p_new.asTPose());
    }
    state_.estimated_trajectory = std::move(updated);
  }

  // Rebuild distance checkers from the corrected estimated_trajectory:
  if (state_.distance_checker_local_map.has_value()) {
    state_.distance_checker_local_map.emplace(params_.local_map_updates.measure_from_last_kf_only);
    for (const auto & [t, p_tpose] : state_.estimated_trajectory) {
      state_.distance_checker_local_map->insert(mrpt::poses::CPose3D(p_tpose));
    }
  }
  if (state_.distance_checker_simplemap.has_value()) {
    state_.distance_checker_simplemap.emplace(params_.simplemap.measure_from_last_kf_only);
    auto smLck = mrpt::lockHelper(state_simplemap_mtx_);
    for (size_t i = 0; i < state_.reconstructed_simplemap.size(); i++) {
      const auto & kf = state_.reconstructed_simplemap.get(i);
      if (kf.pose) {
        state_.distance_checker_simplemap->insert(kf.pose->getMeanVal());
      }
    }
  }

  // Clear visualization path so it is rebuilt from corrected trajectory:
  if (state_.glEstimatedPath) {
    state_.glEstimatedPath->clear();
  }

  state_.mark_local_map_as_updated(/*force_republish=*/true);
  state_.last_gravity_rebake_tim = now;

  MRPT_LOG_INFO_FMT(
    "Tilt rebake applied: tilt was %.2f deg, pivot KF=%lu, "
    "correction pitch/roll=(%.3f, %.3f) deg",
    mrpt::RAD2DEG(tilt_rad), static_cast<unsigned long>(pivotId), mrpt::RAD2DEG(imu_pitch),
    mrpt::RAD2DEG(imu_roll));

  return true;

#else
  // Feature not available: mola_metric_maps or mola_kernel not present.
  (void)this;
  return false;
#endif
}
