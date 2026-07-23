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
 * @file   LidarOdometry_GravityEstimation.cpp
 * @brief  Gravity-direction estimation feeding the ICP prior's pitch/roll.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_imu_preintegration/ImuPreintegrator.h>
#include <mola_kernel/interfaces/KeyframeMapCapable.h>
#include <mola_lidar_odometry/LidarOdometry.h>
#include <mrpt/poses/CPose3D.h>

#include <Eigen/Dense>
#include <algorithm>

// The map-level rebake needs the mola_kernel KeyframeMapCapable interface
// (mutable per-KF poses). mola_kernel is a hard dependency, but guard on the
// header so an older kernel without the interface degrades gracefully.
#if defined(__has_include) && __has_include(<mola_kernel/interfaces/KeyframeMapCapable.h>)
#define MOLA_LO_HAS_KFM_RELEVELING 1
#endif

namespace mola
{
namespace
{
/// Largest gap [s] between consecutive IMU samples still integrated: a longer
/// one means a sensor dropout, and integrating across it would fabricate data.
constexpr double MAX_IMU_SAMPLE_GAP = 1.0;

mrpt::math::TVector3D rotate(const mrpt::poses::CPose3D & pose, const mrpt::math::TVector3D & v)
{
  return pose.rotateVector(v);
}
}  // namespace

void LidarOdometry::updateMapGravityEstimator(
  const double timestamp, const mrpt::poses::CPose3D & vehiclePose,
  const std::optional<mrpt::math::TTwist3D> & twist,
  const std::optional<mrpt::math::CMatrixDouble66> & twistInvCov)
{
  if (
    !params_.imu_gravity_correction.enabled ||
    params_.imu_gravity_correction.method !=
      Parameters::IMUGravityCorrection::Method::Preintegration) {
    return;
  }

  // Without a velocity estimate there is nothing to anchor an interval on:
  // the whole method rests on comparing the odometry's velocity change against
  // the preintegrated one.
  if (!twist.has_value()) {
    return;
  }

#if !defined(MOLA_LO_HAS_MAP_GRAVITY_ESTIMATOR)
  // Built against an older mola_imu_preintegration: the method is rejected at
  // parameter-load time, so this path is unreachable.
  return;
#else
  auto & est = state_.map_gravity_estimator;
  if (!est.has_value()) {
    est.emplace();
    est->parameters = params_.imu_gravity_correction.estimator;

    // Seed the bias priors from the at-rest calibration, when available: a
    // better prior center directly improves the weakly-observable accel bias.
    if (state_.imu_initializer.has_value()) {
      if (const auto calib = state_.imu_initializer->getCalibration(); calib.has_value()) {
        est->set_bias_prior(calib->bias_acc_b, calib->bias_gyro);
      }
    }
  }

  // Map-frame velocity and its covariance (NavState twist is expressed in the
  // LOCAL VEHICLE frame, so it has to be rotated):
  const mrpt::math::TVector3D v_body{twist->vx, twist->vy, twist->vz};
  const mrpt::math::TVector3D v_map = rotate(vehiclePose, v_body);

  mrpt::math::CMatrixDouble33 cov_v_map =
    mola::imu::MapGravityEstimator::Interval::defaultVelocityCov();
  if (twistInvCov.has_value()) {
    // Invert the linear-velocity block of the information matrix, then rotate
    // it into the map frame. Fall back to the loose default if it is singular
    // (a common case when the estimator has not converged yet).
    const auto infoLin = twistInvCov->blockCopy<3, 3>(0, 0);
    if (const double d = infoLin.det(); std::isfinite(d) && d > 1e-12) {
      const auto R = vehiclePose.getRotationMatrix().asEigen();
      cov_v_map.asEigen() = R * infoLin.asEigen().inverse() * R.transpose();
    }
  }

  // ---- First call: just open the interval ------------------------------
  if (!state_.gravity_interval_anchor.has_value()) {
    auto & a = state_.gravity_interval_anchor.emplace();
    a.timestamp = timestamp;
    a.pose = vehiclePose;
    a.v_map = v_map;
    a.cov_v_map = cov_v_map;
    return;
  }

  const auto & anchor = *state_.gravity_interval_anchor;

  const double elapsed = timestamp - anchor.timestamp;
  if (elapsed < params_.imu_gravity_correction.interval_duration) {
    return;  // keep accumulating
  }
  if (elapsed <= 0) {
    // Non-monotonic stamp: restart the interval rather than feed garbage.
    state_.gravity_interval_anchor.reset();
    return;
  }

  // ---- Close the interval ------------------------------------------------
  // Drain the IMU samples of this span from the DEDICATED long-retention
  // buffer (fed in onIMUImpl through our own ImuTransformer, so the samples
  // are already in the vehicle frame with the lever arm applied).
  const auto window = state_.gravity_imu_buffer.window_since(anchor.timestamp, timestamp);

  mola::imu::ImuPreintegrator pim(state_.imu_preint_params);
  pim.reset_integration();

  double tPrev = anchor.timestamp;
  for (const auto & [ta, a_b] : window.a_b) {
    const double dt = ta - tPrev;
    tPrev = ta;
    if (dt <= 0 || dt > MAX_IMU_SAMPLE_GAP) {
      continue;
    }
    mrpt::math::TVector3D w_b{0, 0, 0};
    if (const auto itw = window.w_b.find(ta); itw != window.w_b.end()) {
      w_b = itw->second;
    }
    pim.integrate_measurement(a_b, w_b, dt);
  }

  mola::imu::MapGravityEstimator::Interval itv;
  itv.t_from = anchor.timestamp;
  itv.t_to = timestamp;
  itv.delta = pim.current_state();
  itv.R_from = anchor.pose.getRotationMatrix();
  itv.R_to = vehiclePose.getRotationMatrix();
  itv.v_from = anchor.v_map;
  itv.v_to = v_map;
  itv.cov_v_from = anchor.cov_v_map;
  itv.cov_v_to = cov_v_map;

  if (est->add_interval(itv)) {
    est->solve();

    if (const auto & r = est->latest_result(); r.has_value()) {
      MRPT_LOG_DEBUG_FMT(
        "Map-gravity estimate: tilt=%.3f deg (pitch=%.3f roll=%.3f, sigma=%.3f/%.3f deg), "
        "|g|=%.4f, b_a=%s, b_g=%s, %zu intervals, %s",
        mrpt::RAD2DEG(r->tilt), mrpt::RAD2DEG(r->pitch_correction),
        mrpt::RAD2DEG(r->roll_correction), mrpt::RAD2DEG(r->pitch_sigma),
        mrpt::RAD2DEG(r->roll_sigma), r->gravity_in_map.norm(), r->bias_acc.asString().c_str(),
        r->bias_gyro.asString().c_str(), r->num_intervals,
        r->converged ? "CONVERGED" : "not converged");
    }
  } else {
    // Rejections are expected and benign (a dropout, a stamp glitch); the
    // unknowns are global, so a skipped interval leaves nothing unconstrained.
    MRPT_LOG_THROTTLE_WARN_FMT(
      5.0, "Map-gravity interval rejected: %s", est->last_rejection_reason().c_str());
  }

  // Re-anchor for the next interval. Chaining exactly (the new `from` is this
  // `to`) means every IMU sample belongs to exactly one interval.
  auto & a = *state_.gravity_interval_anchor;
  a.timestamp = timestamp;
  a.pose = vehiclePose;
  a.v_map = v_map;
  a.cov_v_map = cov_v_map;
#endif
}

std::optional<LidarOdometry::GravityPitchRoll> LidarOdometry::estimateGravityPitchRoll(
  const mrpt::poses::CPose3D & vehiclePose) const
{
  using Method = Parameters::IMUGravityCorrection::Method;

  if (!params_.imu_gravity_correction.enabled) {
    return {};
  }

  const double sigma_deg_default = params_.imu_gravity_correction.sigma_deg;

  // ---- Preintegration method -------------------------------------------
#if defined(MOLA_LO_HAS_MAP_GRAVITY_ESTIMATOR)
  if (params_.imu_gravity_correction.method == Method::Preintegration) {
    if (state_.map_gravity_estimator.has_value()) {
      const auto & r = state_.map_gravity_estimator->latest_result();
      if (r.has_value() && r->converged) {
        const double s = std::max(r->pitch_sigma, r->roll_sigma);
        if (s <= mrpt::DEG2RAD(params_.imu_gravity_correction.max_accepted_sigma_deg)) {
          // The estimator yields gravity in the MAP frame. Transport it into
          // the current VEHICLE frame with the odometry attitude, so what this
          // function returns has exactly the same meaning as the accelerometer
          // average below: the vehicle's tilt with respect to true vertical.
          //
          // Do NOT instead left-multiply the pose by the estimator's leveling
          // `correction`: that expresses the pose in a re-leveled frame while
          // the local map stays in the old one, so the ICP prior ends up
          // fighting the map's own geometry. Measured on Oxford Spires, that
          // diverged the solution after ~57 s.
          //
          // In the single-interval limit this reduces exactly to the raw
          // accelerometer reading; the gain comes from the window averaging,
          // the body-acceleration cancellation and the bias estimation.
          if (const auto pr = vehiclePitchRollFromMapGravity(r->gravity_in_map, vehiclePose);
              pr.has_value()) {
            // Floor the earned sigma: the information matrix can grow very
            // confident over a long window, and a prior far tighter than the
            // ICP's own attitude accuracy destabilizes registration.
            const double sigma_rad =
              std::max(s, mrpt::DEG2RAD(params_.imu_gravity_correction.min_sigma_deg));
            return applyGravityCalibration(pr->first, pr->second, sigma_rad);
          }
        }
      }
    }
    // Not converged yet: fall through to the accelerometer average, which at
    // least works while the vehicle is still quasi-static at start-up.
  }
#endif

  // ---- Accelerometer-average method (also the not-converged fallback) ----
  const auto gravityPR = state_.gravity_estimator.estimatedPitchRoll(
    params_.imu_gravity_correction.averaging_samples,
    params_.imu_gravity_correction.max_age_seconds);

  if (!gravityPR.has_value()) {
    return {};
  }
  // The estimator already reduced its samples to (pitch, roll).
  return applyGravityCalibration(
    gravityPR->first, gravityPR->second, mrpt::DEG2RAD(sigma_deg_default));
}

std::optional<std::pair<double, double>> LidarOdometry::vehiclePitchRollFromMapGravity(
  const mrpt::math::TVector3D & gravity_in_map, const mrpt::poses::CPose3D & vehiclePose)
{
  const double gn = gravity_in_map.norm();
  if (gn < 1e-6) {
    return {};
  }

  // "Up" in the map frame, then rotated into the vehicle frame by the inverse
  // of the vehicle attitude. This yields the vehicle's tilt with respect to
  // true vertical, NOT the map's tilt.
  const mrpt::math::TVector3D up_map = gravity_in_map * (-1.0 / gn);
  const mrpt::math::TVector3D u = vehiclePose.inverseRotateVector(up_map);

  const double n = u.norm();
  if (n < 1e-6) {
    return {};
  }
  const double nx = u.x / n;
  const double ny = u.y / n;
  const double nz = u.z / n;

  // Same convention as GravityEstimator::estimatedPitchRoll(): with the vehicle
  // level, the measured "up" direction is [0, 0, +1] (z-up).
  return std::make_pair(std::asin(-nx), std::atan2(ny, nz));
}

std::optional<LidarOdometry::GravityPitchRoll> LidarOdometry::applyGravityCalibration(
  const double imu_pitch, const double imu_roll, const double sigma_rad) const
{
  // The gravity estimate is an *absolute* tilt w.r.t. true vertical, but the
  // map/global frame may not itself be exactly level (e.g. a nonzero
  // `fixed_initial_pose` orientation, or the vehicle starting on a slope).
  // Re-express it relative to the map frame using the tilt recorded at the map
  // origin as a calibration offset. Both readings share the same (gauge) yaw=0
  // convention, which is valid since pitch/roll extracted from gravity alone
  // are independent of the (unobservable) yaw used to express them.
  GravityPitchRoll out;
  out.pitch = imu_pitch;
  out.roll = imu_roll;
  // Cap the applied prior sigma (strengthen the pitch/roll constraint). This is
  // independent of the acceptance gate: the estimate is already accepted here;
  // this only bounds how confident the resulting prior is.
  out.sigma_rad =
    std::min(sigma_rad, mrpt::DEG2RAD(params_.imu_gravity_correction.max_applied_sigma_deg));

  if (state_.gravity_calib_pitch_roll.has_value()) {
    const auto [pitch0, roll0] = *state_.gravity_calib_pitch_roll;

    const mrpt::poses::CPose3D R_map_veh0(
      mrpt::poses::CPose3D(params_.initial_localization.fixed_initial_pose));
    const mrpt::poses::CPose3D R_grav_veh0(0, 0, 0, 0, pitch0, roll0);
    const mrpt::poses::CPose3D R_grav_veh_t(0, 0, 0, 0, imu_pitch, imu_roll);

    const mrpt::poses::CPose3D R_map_veh_t = R_map_veh0 + (-R_grav_veh0) + R_grav_veh_t;

    out.pitch = R_map_veh_t.pitch();
    out.roll = R_map_veh_t.roll();
  }

  return out;
}

void LidarOdometry::updateMapReleveling(const mrpt::Clock::time_point & scan_tim)
{
#if defined(MOLA_LO_HAS_KFM_RELEVELING) && defined(MOLA_LO_HAS_MAP_GRAVITY_ESTIMATOR)
  const auto & rebakeParams = params_.imu_gravity_correction.tilt_rebake;
  if (!rebakeParams.enabled) {
    return;
  }

  // 1) Locate the KeyframeMapCapable layer in the local map (by interface, so we
  //    do not depend on the configured layer name).
  mola::KeyframeMapCapable * kfCap = nullptr;
  if (state_.local_map) {
    for (auto & [name, layer] : state_.local_map->layers) {
      if (auto * c = dynamic_cast<mola::KeyframeMapCapable *>(layer.get()); c != nullptr) {
        kfCap = c;
        break;
      }
    }
  }
  if (kfCap == nullptr) {
    if (!state_.gravity_rebake_unsupported_warned) {
      state_.gravity_rebake_unsupported_warned = true;
      MRPT_LOG_WARN(
        "imu_gravity_correction.tilt_rebake is enabled, but the configured "
        "local-map layer is not KeyframeMapCapable; map re-leveling disabled.");
    }
    return;
  }

  // 2) Gravity direction in the map frame from the PREINTEGRATION estimator.
  //    The raw per-KF accelerometer average (GravityMapAligner) is body-accel
  //    contaminated and unusable on a dynamic platform; the preintegration
  //    estimator cancels body acceleration via velocity differencing.
  if (!state_.map_gravity_estimator.has_value()) {
    return;
  }
  const auto & gres = state_.map_gravity_estimator->latest_result();
  if (!gres.has_value() || !gres->converged) {
    return;
  }
  const double gn = gres->gravity_in_map.norm();
  if (gn < 1e-3) {
    return;
  }
  // "Up" in the map frame (gravity points down):
  const mrpt::math::TVector3D up_map = gres->gravity_in_map * (-1.0 / gn);

  // 3) Trigger on the map's accumulated tilt vs true vertical.
  const double tiltRad = std::acos(std::clamp(up_map.z, -1.0, 1.0));

  MRPT_LOG_THROTTLE_DEBUG_FMT(
    2.0, "rebake check: mapTilt=%.2f deg (trigger=%.1f)", mrpt::RAD2DEG(tiltRad),
    rebakeParams.trigger_deg);

  // 4) Throttle + minimum active keyframes + trigger threshold.
  if (state_.last_gravity_rebake_tim.has_value()) {
    const double dt = mrpt::system::timeDifference(*state_.last_gravity_rebake_tim, scan_tim);
    if (dt < rebakeParams.min_interval_sec) {
      return;
    }
  }
  const auto kfPoses = kfCap->keyframePoses();
  if (kfPoses.size() < rebakeParams.min_active_keyframes) {
    return;
  }
  if (mrpt::RAD2DEG(tiltRad) < rebakeParams.trigger_deg) {
    return;
  }

  // 5) Rigid pitch/roll leveling rotation (sends map-frame "up" to +Z),
  //    applied about the CURRENT pose as pivot. Rotating about the current
  //    position freezes where we are and only re-levels orientation, so neither
  //    the live pose nor the recorded trajectory jumps in position. (Rotating
  //    about a distant pivot, e.g. the oldest KF, swings far-away poses by the
  //    lever arm and compounds into large drift.)
  const mrpt::poses::CPose3D R_world = mola::GravityMapAligner::rotationFromGravity(up_map);
  const mrpt::poses::CPose3D Tp =
    mrpt::poses::CPose3D::FromTranslation(state_.last_lidar_pose.mean.translation());
  const mrpt::poses::CPose3D T_net = Tp + R_world + (-Tp);

  // 6) Apply to the local map (all active KFs). Points are stored per-KF in
  //    their local frame, so this re-levels the map geometry ICP registers
  //    against, with no re-insertion of points. KFs near the current pose barely
  //    move; distant ones age out of the window before mattering.
  for (const auto & [id, pose] : kfPoses) {
    kfCap->setKeyframePose(id, T_net + pose);
  }

  // (c) Live cached pose + motion-model prior (twist stays body-local). Position
  //     is frozen (pivot = current pose); only the orientation re-levels.
  state_.last_lidar_pose.changeCoordinatesReference(T_net);
  if (state_.last_motion_model_output) {
    state_.last_motion_model_output->pose.changeCoordinatesReference(T_net);
  }

  // NOTE: the historical estimated_trajectory and simplemap are deliberately NOT
  // rotated. They were committed in their own (progressively re-leveled) frames;
  // rotating the whole history about the current pose would swing distant past
  // poses by the lever arm. Forward poses build on the (frozen-position,
  // re-leveled) current pose, so the trajectory stays position-continuous.

  // (g) ATOMIC STATE MIGRATION. The map frame just rotated, so state expressed
  //     in it must move too. The preint estimator's interval window and the
  //     interval anchor are still in the OLD frame; feeding the next interval
  //     across that discontinuity corrupts g_M and runs the tilt away. Rather
  //     than rotate each stored interval, RESET the estimator (re-seeding the
  //     bias prior so it re-converges quickly) and clear the interval anchor, so
  //     it re-converges cleanly in the NEW frame. Its "converged" gate then
  //     blocks the next rebake until it has, breaking the feedback loop.
  const auto biasAcc = gres->bias_acc;
  const auto biasGyro = gres->bias_gyro;
  state_.map_gravity_estimator->reset();
  state_.map_gravity_estimator->set_bias_prior(biasAcc, biasGyro);
  state_.gravity_interval_anchor.reset();

  state_.last_gravity_rebake_tim = scan_tim;

  MRPT_LOG_INFO_FMT(
    "Map gravity rebake: %zu KFs re-leveled, map tilt was %.2f deg, correction "
    "pitch=%.2f roll=%.2f deg",
    kfPoses.size(), mrpt::RAD2DEG(tiltRad), mrpt::RAD2DEG(R_world.pitch()),
    mrpt::RAD2DEG(R_world.roll()));
#else
  (void)scan_tim;
#endif
}

}  // namespace mola
