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
#include <mola_lidar_odometry/MapFrameRelevel.h>

// mp2p_icp:
#include <mp2p_icp_filters/FilterDeskew.h>

// MRPT:
#include <mrpt/core/get_env.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/Lie/SO.h>

#include <algorithm>
#include <cmath>
#include <mutex>

namespace mola
{

bool LidarOdometry::isPipelineUsingIMU() const
{
  auto lckState = mrpt::lockHelper(state_mtx_);
  return isPipelineUsingIMU_locked();
}

bool LidarOdometry::isPipelineUsingIMU_locked() const
{
  if (state_.isPipelinesUsingIMU.has_value()) {
    return *state_.isPipelinesUsingIMU;
  }

  const auto hasDeskewStage = [](const mp2p_icp_filters::FilterPipeline & pipeline) {
    for (const auto & stage : pipeline) {
      auto deskew = std::dynamic_pointer_cast<mp2p_icp_filters::FilterDeskew>(stage);
      if (!deskew) {
        continue;
      }
      // It's deskew: we need IMU if using IMU-based methods:
      return deskew->method != mp2p_icp_filters::MotionCompensationMethod::None &&
             deskew->method != mp2p_icp_filters::MotionCompensationMethod::Linear;
    }
    return false;
  };

  state_.isPipelinesUsingIMU =
    hasDeskewStage(state_.pc_filter1) || hasDeskewStage(state_.pc_filter2) ||
    hasDeskewStage(state_.pc_filter3) || hasDeskewStage(state_.pc_deskew);

  return *state_.isPipelinesUsingIMU;
}

#if defined(MOLA_LO_HAS_MAP_GRAVITY_ESTIMATOR)
void LidarOdometry::closeMapGravityInterval(
  double timestamp, const mrpt::poses::CPose3D & pose, const mrpt::math::TTwist3D & twistLocal)
{
  // Caller holds state_mtx_; map_gravity is fed by the IMU thread too:
  auto lckImu = mrpt::lockHelper(imu_state_mtx_);

  auto & mg = state_.map_gravity;

  // Velocity in the MAP frame, which is the frame the estimator works in.
  const auto R = pose.getRotationMatrix();
  const mrpt::math::TVector3D v_map = {
    R(0, 0) * twistLocal.vx + R(0, 1) * twistLocal.vy + R(0, 2) * twistLocal.vz,
    R(1, 0) * twistLocal.vx + R(1, 1) * twistLocal.vy + R(1, 2) * twistLocal.vz,
    R(2, 0) * twistLocal.vx + R(2, 1) * twistLocal.vy + R(2, 2) * twistLocal.vz};

  // Gravity enters as (v_to - v_from - R_from*dV) / dt, so a velocity error eps
  // becomes a gravity error eps/dt: at the scan rate (dt ~ 0.1 s) a 0.1 m/s
  // velocity error is ~1 m/s^2, i.e. ~6 deg of apparent tilt. Accumulate over a
  // longer span before closing an interval so that sensitivity drops as 1/dt.
  const double openSpan = mg.open_t_from.has_value() ? (timestamp - *mg.open_t_from) : 0.0;
  const bool spanLongEnough =
    openSpan >= params_.imu_gravity_correction.map_gravity.min_interval_seconds;

  if (mg.open_t_from.has_value() && spanLongEnough) {
    mola::imu::MapGravityEstimator::Interval iv;
    iv.t_from = *mg.open_t_from;
    iv.t_to = timestamp;
    iv.delta = mg.preintegrator.current_state();
    iv.R_from = mg.open_R_from.getRotationMatrix();
    iv.R_to = R;
    iv.v_from = mg.open_v_from;
    iv.v_to = v_map;

    if (mg.estimator.add_interval(iv)) {
      if (++mg.intervals_since_solve >= params_.imu_gravity_correction.map_gravity.solve_every_n) {
        mg.intervals_since_solve = 0;
        if (mg.estimator.solve()) {
          const auto & r = *mg.estimator.latest_result();

          // The scan timestamp is logged so the estimate can be joined against
          // the trajectory and scored offline (see map_gravity.log_only).
          MRPT_LOG_DEBUG_FMT(
            "MapGravity: t=%.6f g_map=[%.4f %.4f %.4f] tilt=%.3f deg (pitch %.3f+-%.3f, roll "
            "%.3f+-%.3f deg) ba=[%.4f %.4f %.4f] bg=[%.5f %.5f %.5f] intervals=%zu",
            r.timestamp, r.gravity_in_map.x, r.gravity_in_map.y, r.gravity_in_map.z,
            mrpt::RAD2DEG(r.tilt), mrpt::RAD2DEG(r.pitch_correction), mrpt::RAD2DEG(r.pitch_sigma),
            mrpt::RAD2DEG(r.roll_correction), mrpt::RAD2DEG(r.roll_sigma), r.bias_acc.x,
            r.bias_acc.y, r.bias_acc.z, r.bias_gyro.x, r.bias_gyro.y, r.bias_gyro.z,
            r.num_intervals);

          evaluateMapFrameRelevel(r);
        }
      }
    } else {
      MRPT_LOG_DEBUG_STREAM(
        "MapGravity: interval rejected: " << mg.estimator.last_rejection_reason());
    }
  }

  // Only re-open (and drop the accumulated preintegration) once an interval was
  // actually closed; otherwise keep integrating into the still-open one.
  if (!mg.open_t_from.has_value() || spanLongEnough) {
    mg.open_t_from = timestamp;
    mg.open_R_from = pose;
    mg.open_v_from = v_map;
    mg.preintegrator.reset_integration();
  }
}

#if defined(MOLA_LO_CAN_RELEVEL_MAP_FRAME)
void LidarOdometry::evaluateMapFrameRelevel(const mola::imu::MapGravityEstimator::Result & r)
{
  // Caller holds state_mtx_ and imu_state_mtx_.
  const auto & p = params_.imu_gravity_correction.map_gravity;
  auto & mg = state_.map_gravity;

  if (!p.relevel_map_frame || p.log_only || mg.relevel_decided) {
    return;
  }

  // Readiness is a data-quantity question, deliberately expressed as an
  // interval count and not as the reported sigmas: those are measured to be
  // roughly an order of magnitude more confident than the estimate is
  // accurate, so gating on them would not mean what it says.
  if (r.num_intervals < p.relevel_min_intervals) {
    return;
  }

  // The map frame is the frame everything else is anchored to, so once
  // anything external has been tied to it, moving it is no longer a private
  // gauge choice:
  if (
    state_.map_has_been_loaded || state_.external_georef.has_value() ||
    (state_.local_map && state_.local_map->georeferencing.has_value())) {
    mg.relevel_decided = true;
    MRPT_LOG_INFO(
      "Map-frame re-leveling stood down: the map frame is already tied to a "
      "loaded or geo-referenced map.");
    return;
  }

  const double tiltDeg = mrpt::RAD2DEG(r.tilt);

  // A tilt this large is not a leaning map, it is a broken estimate; refusing
  // is cheaper than rotating the whole session by it.
  constexpr double MAX_PLAUSIBLE_TILT_DEG = 45.0;
  if (!std::isfinite(tiltDeg) || tiltDeg > MAX_PLAUSIBLE_TILT_DEG) {
    mg.relevel_decided = true;
    MRPT_LOG_WARN_FMT(
      "Map-frame re-leveling stood down: implausible estimated tilt of %.2f deg.", tiltDeg);
    return;
  }

  // The magnitude gate. The estimate carries an error of its own, so applying
  // it to an already-level map frame replaces a small error with a larger one.
  // Only fire when the tilt removed clearly exceeds the error introduced.
  if (tiltDeg < p.relevel_min_tilt_deg) {
    mg.relevel_decided = true;
    MRPT_LOG_INFO_FMT(
      "Map-frame re-leveling stood down: estimated tilt %.2f deg is below "
      "relevel_min_tilt_deg=%.2f deg, so correcting it would not be a net gain "
      "(%zu intervals, t=%.3f).",
      tiltDeg, p.relevel_min_tilt_deg, r.num_intervals, r.timestamp);
    return;
  }

  mg.relevel_decided = true;
  mg.pending_relevel =
    mrpt::poses::CPose3D::FromRotationAndTranslation(r.correction, mrpt::math::TVector3D(0, 0, 0));
}

void LidarOdometry::applyMapFrameRelevel()
{
  // Caller holds state_mtx_ only: the rotation below needs the local-map and
  // simplemap mutexes, and the lock order forbids taking those while holding
  // imu_state_mtx_ (which the solve loop that requested this does hold).
  std::optional<mrpt::poses::CPose3D> pending;
  {
    auto lckImu = mrpt::lockHelper(imu_state_mtx_);
    std::swap(pending, state_.map_gravity.pending_relevel);
  }
  if (!pending.has_value()) {
    return;
  }

  const auto & b = *pending;
  const ProfilerEntry tle(profiler_, "onLidar.map_frame_relevel");

  // 1/6: the local map contents.
  {
    auto lckMapContents = mrpt::lockHelper(local_map_content_mtx_);
    if (state_.local_map) {
      transform_to_new_map_frame(*state_.local_map, b);
      state_.local_map->metadata["map_frame_relevel_rotation"] = "'" + b.asString() + "'";
    }
  }
  state_.mark_local_map_as_updated(true /*force_republish*/);

  // 2/6: the keyframes already written to the simplemap.
  {
    auto lckSM = mrpt::lockHelper(state_simplemap_mtx_);
    transform_to_new_map_frame(state_.reconstructed_simplemap, b);
  }

  // 3/6: the published trajectory.
  {
    auto lckTraj = mrpt::lockHelper(state_trajectory_mtx_);
    transform_to_new_map_frame(state_.estimated_trajectory, b);
  }

  // 4/6: cached poses and the keyframe-density bookkeeping. The deciders hold
  // map-frame poses, so leaving them behind would make the next distance check
  // compare frames rather than positions.
  state_.last_lidar_pose.changeCoordinatesReference(b);
  if (state_.last_motion_model_output) {
    state_.last_motion_model_output->pose.changeCoordinatesReference(b);
    // `twist` is in the vehicle frame, hence invariant.
  }
  if (state_.kf_decider_local_map) {
    state_.kf_decider_local_map->transform_left_multiply(b);
  }
  if (state_.kf_decider_simplemap) {
    state_.kf_decider_simplemap->transform_left_multiply(b);
  }
  state_.last_deskewed_scan_for_publishing.reset();

  // 5/6: the state estimator, whose stored poses are map-frame too.
  if (state_.navstate_fuse) {
#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_TRANSFORM_FRAME)
    const bool ok = state_.navstate_fuse->transform_frame(b);
#else
    const bool ok = false;
#endif
    if (!ok) {
      // Without frame support the only consistent option is to drop the
      // estimator state: keeping poses in the old frame would feed the next
      // ICP an initial guess off by the correction.
      MRPT_LOG_WARN(
        "The state estimator cannot re-express its state in a new frame; "
        "resetting it instead. Expect one scan without a motion model.");
      state_.navstate_fuse->reset();
      state_.last_motion_model_output.reset();
    }
  }

  // 6/6: the verticality reference is now level by construction, and the
  // map-gravity estimator's window was expressed in the frame just rotated.
  {
    auto lckImu = mrpt::lockHelper(imu_state_mtx_);
    state_.gravity_calib_pitch_roll = std::make_pair(0.0, 0.0);
    state_.gravity_calib_pose = mrpt::poses::CPose3D::Identity();

    auto & mg = state_.map_gravity;
    mg.estimator.reset();
    mg.preintegrator.reset_integration();
    mg.open_t_from.reset();
    mg.intervals_since_solve = 0;
    mg.applied_relevel = b;
  }

  MRPT_LOG_INFO_FMT(
    "Map frame re-leveled once, about the map origin, by [%s] (tilt %.3f deg). "
    "All map products from here on are gravity-aligned.",
    b.asString().c_str(),
    mrpt::RAD2DEG(mrpt::poses::Lie::SO<3>::log(b.getRotationMatrix()).norm()));
}
#else
// Built against a mola/mola_pose_list/mola_kernel without the pieces the gauge
// change needs. The feature stands down rather than rotating part of the state.
void LidarOdometry::evaluateMapFrameRelevel(const mola::imu::MapGravityEstimator::Result &)
{
  auto & mg = state_.map_gravity;
  if (!params_.imu_gravity_correction.map_gravity.relevel_map_frame || mg.relevel_decided) {
    return;
  }
  mg.relevel_decided = true;
  MRPT_LOG_WARN(
    "map_gravity.relevel_map_frame is enabled but this build cannot apply it: it needs "
    "SearchablePoseList::transform_left_multiply() and NavStateFilter::transform_frame(), "
    "which this mola_pose_list / mola_kernel does not provide. Standing down; upgrade "
    "those packages to use it.");
}

void LidarOdometry::applyMapFrameRelevel() {}
#endif
#endif  // MOLA_LO_HAS_MAP_GRAVITY_ESTIMATOR

// Deliberately NOT inside the map-gravity block above: the rank-2 prior is a
// mp2p_icp feature and stands on its own, using the map-frame gravity estimate
// only when there is one (see the inner guard below). Its declaration and its
// call site are gated on MOLA_LO_HAS_MP2P_GRAVITY_PRIOR alone, so gating the
// definition on both leaves the symbol undefined whenever a build has the
// newer mp2p_icp but the older mola_imu_preintegration.
#if defined(MOLA_LO_HAS_MP2P_GRAVITY_PRIOR)
std::optional<mp2p_icp::GravityPrior> LidarOdometry::buildGravityPrior() const
{
  // Caller holds state_mtx_; gravity_estimator and map_gravity are fed by the
  // IMU thread. Held across the whole body so the reading and the map-frame
  // reference it is combined with come from one consistent snapshot:
  auto lckImu = mrpt::lockHelper(imu_state_mtx_);

  // "Up" unit vector from (pitch, roll), matching the convention of
  // GravityEstimator::estimatedPitchRoll(): pitch = asin(-u.x),
  // roll = atan2(u.y, u.z). At rest the accelerometer's specific force points
  // up, so this IS the measured gravity-up direction in the vehicle frame.
  const auto upFrom = [](double p, double r) {
    return mrpt::math::TVector3D(
      -std::sin(p), std::cos(p) * std::sin(r), std::cos(p) * std::cos(r));
  };

  mp2p_icp::GravityPrior g;

  // Where this scan's reading comes from: the odometry attitude source when
  // one is configured and current, otherwise the accelerometer average. The
  // odometry reading carries its own fixed sigma, since the adaptive widening
  // below measures accelerometer dispersion and has nothing to say about an
  // externally estimated attitude.
  std::optional<double> readingSigmaRad;

  const auto upOdom = state_.gravity_calib_from_odometry ? odometryUpBody() : std::nullopt;

  if (upOdom.has_value()) {
    g.up_body = *upOdom;
    readingSigmaRad = mrpt::DEG2RAD(params_.imu_gravity_correction.odometry_attitude.sigma_deg);
  } else if (state_.gravity_calib_from_odometry) {
    // The map-origin reference was captured from the odometry source, so an
    // accelerometer reading here would be measured against a different
    // vertical: the constant offset between the two would stop being a gauge
    // and become a tilt of the map frame, which is what capturing both from
    // one source exists to prevent. Emitting no prior for this scan is the
    // only consistent option while the source is silent.
    return std::nullopt;
  } else {
    const auto pr = state_.gravity_estimator.estimatedPitchRoll(
      params_.imu_gravity_correction.averaging_samples,
      params_.imu_gravity_correction.max_age_seconds);
    if (!pr.has_value()) {
      return std::nullopt;
    }
    g.up_body = upFrom(pr->first, pr->second);
  }

  // Gravity direction expressed in the MAP frame. The map origin is not
  // necessarily level (nonzero fixed_initial_pose, or starting on a slope), so
  // rotate the tilt captured there into map coordinates. No Euler algebra is
  // involved: this is a plain vector rotation, valid at any yaw.
  double extraSigmaRad = 0;
  bool upMapFromEstimator = false;

#if defined(MOLA_LO_HAS_MAP_GRAVITY_ESTIMATOR)
  // Preferred source: the online map-frame gravity estimate. It replaces the
  // one-shot capture below, whose error is whatever the accelerometer average
  // happened to be at the first keyframe and which never improves afterwards.
  if (
    params_.imu_gravity_correction.map_gravity.enabled &&
    !params_.imu_gravity_correction.map_gravity.log_only && !state_.gravity_calib_from_odometry) {
    if (const auto & r = state_.map_gravity.estimator.latest_result();
        r.has_value() && r->converged) {
      const auto & gm = r->gravity_in_map;
      const double n = std::sqrt(gm.x * gm.x + gm.y * gm.y + gm.z * gm.z);
      if (n > 1e-3) {
        // gravity_in_map points DOWN, "up" is its negation.
        g.up_map = {-gm.x / n, -gm.y / n, -gm.z / n};
        upMapFromEstimator = true;
        // The reference direction is itself uncertain; add its earned sigma to
        // the per-scan reading's, instead of pretending the reference is exact.
        extraSigmaRad = std::max(r->pitch_sigma, r->roll_sigma);
      }
    }
  }
#endif

  if (!upMapFromEstimator) {
    if (state_.gravity_calib_pitch_roll.has_value()) {
      const auto [pitch0, roll0] = *state_.gravity_calib_pitch_roll;
      // Transport the reading through the pose it was taken at. That pose is
      // the map origin whenever the capture succeeded on the first scan.
      g.up_map = state_.gravity_calib_pose->rotateVector(upFrom(pitch0, roll0));
    } else {
      // No reference yet: assuming the map is level is a guess, not a fact,
      // and it is wrong by the map's own tilt for as long as it lasts.
      g.up_map = {0, 0, 1};
    }
  }

  const double s = readingSigmaRad.value_or(effectiveGravitySigmaRad());
  g.sigma_rad = std::sqrt(s * s + extraSigmaRad * extraSigmaRad);
  return g;
}

#endif  // MOLA_LO_HAS_MP2P_GRAVITY_PRIOR

std::optional<mrpt::math::TVector3D> LidarOdometry::odometryUpBody() const
{
  // Caller holds imu_state_mtx_, under which odom_attitude is written.
  const auto & oa = params_.imu_gravity_correction.odometry_attitude;

  if (!oa.enabled || !state_.odom_attitude.valid) {
    return std::nullopt;
  }

  // Report nothing rather than a stale attitude: a source that stops
  // publishing must not freeze the vertical at its last value for the rest of
  // the run. What that degrades to is the caller's decision, since it depends
  // on which source the map-origin reference was captured from.
  if (oa.max_age_seconds > 0) {
    const double age = latest_obs_time_.load() - state_.odom_attitude.timestamp;
    if (age > oa.max_age_seconds) {
      return std::nullopt;
    }
  }

  return state_.odom_attitude.up_body;
}

void LidarOdometry::captureMapOriginVerticality(const mrpt::poses::CPose3D & poseAtCapture)
{
  // Caller holds state_mtx_; gravity_estimator is fed by the IMU thread:
  auto lckImu = mrpt::lockHelper(imu_state_mtx_);

  if (!params_.imu_gravity_correction.enabled || state_.gravity_calib_pitch_roll.has_value()) {
    return;
  }

  // A reading from the odometry attitude source has to be referenced against a
  // map origin captured from that same source. Mixing the two turns the
  // constant offset between them into a permanent tilt of the map frame, which
  // is exactly what this reference exists to avoid. No dispersion gate applies
  // here: the source publishes an already-filtered attitude rather than a raw
  // specific force, so there is nothing to wait for.
  //
  // It does have to be waited FOR, though: the capture runs on the first scan,
  // which routinely precedes the first odometry message. Without this the
  // reference would come from the accelerometer and the readings from the
  // odometry, i.e. the very mix described above.
  const auto oaUp = odometryUpBody();

  if (params_.imu_gravity_correction.odometry_attitude.enabled && !oaUp.has_value()) {
    const double now = state_.last_obs_timestamp.has_value()
                         ? mrpt::Clock::toDouble(*state_.last_obs_timestamp)
                         : 0.0;

    if (!state_.odom_attitude_wait_since.has_value()) {
      state_.odom_attitude_wait_since = now;
    }
    const double waited = now - *state_.odom_attitude_wait_since;
    const double timeout = params_.imu_gravity_correction.map_origin_capture_timeout;

    if (timeout <= 0 || waited < timeout) {
      MRPT_LOG_THROTTLE_INFO_FMT(
        2.0,
        "Deferring the map-origin verticality capture: waiting for odometry source '%s' (%.1f s)",
        params_.imu_gravity_correction.odometry_attitude.sensor_label.c_str(), waited);
      return;
    }

    MRPT_LOG_WARN_FMT(
      "Odometry attitude source '%s' produced nothing in %.1f s: falling back to the "
      "accelerometer for BOTH the map-origin reference and the per-scan reading, so the two "
      "stay consistent.",
      params_.imu_gravity_correction.odometry_attitude.sensor_label.c_str(), waited);
  }

  if (oaUp.has_value()) {
    const auto & u = *oaUp;
    state_.gravity_calib_pitch_roll =
      std::make_pair(std::asin(std::clamp(-u.x, -1.0, 1.0)), std::atan2(u.y, u.z));
    state_.gravity_calib_pose = poseAtCapture;
    state_.gravity_calib_from_odometry = true;

    const auto [pOdom, rOdom] = *state_.gravity_calib_pitch_roll;
    MRPT_LOG_INFO_FMT(
      "Map-origin verticality reference captured from the odometry attitude source '%s': "
      "pitch=%.3f roll=%.3f deg (tilt %.3f deg), at pose [%s]",
      params_.imu_gravity_correction.odometry_attitude.sensor_label.c_str(), mrpt::RAD2DEG(pOdom),
      mrpt::RAD2DEG(rOdom),
      mrpt::RAD2DEG(std::acos(std::clamp(std::cos(pOdom) * std::cos(rOdom), -1.0, 1.0))),
      poseAtCapture.asString().c_str());
    return;
  }

  const auto pr = state_.gravity_estimator.estimatedPitchRoll(
    params_.imu_gravity_correction.averaging_samples,
    params_.imu_gravity_correction.max_age_seconds);
  if (!pr.has_value()) {
    // Expected at the very first scan, where the IMU buffer holds only what
    // arrived in the last few milliseconds. Retried on the next scan.
    return;
  }

  // The data is there, but is it gravity? This one reading defines the map's vertical for the
  // whole run, so defer it while the buffered directions disagree too much, the same way the
  // capture is already retried when there is no data at all. Deferring forever is not an
  // option either: a platform that is never still still needs a reference, so after a timeout
  // the reading is taken as-is.
  const double maxDisp =
    mrpt::DEG2RAD(params_.imu_gravity_correction.map_origin_max_dispersion_deg);
  bool takenOnTimeout = false;

  if (maxDisp > 0) {
    const auto disp = state_.gravity_estimator.directionDispersionSigma(
      params_.imu_gravity_correction.max_age_seconds);

    if (disp.has_value() && *disp > maxDisp) {
      const double now = state_.last_obs_timestamp.has_value()
                           ? mrpt::Clock::toDouble(*state_.last_obs_timestamp)
                           : 0.0;

      if (!state_.gravity_calib_first_available_time.has_value()) {
        state_.gravity_calib_first_available_time = now;
      }
      const double waited = now - *state_.gravity_calib_first_available_time;
      const double timeout = params_.imu_gravity_correction.map_origin_capture_timeout;

      if (timeout <= 0 || waited < timeout) {
        MRPT_LOG_THROTTLE_INFO_FMT(
          2.0,
          "Deferring the map-origin verticality capture: accelerometer dispersion %.2f deg > "
          "%.2f deg (waited %.1f s)",
          mrpt::RAD2DEG(*disp), mrpt::RAD2DEG(maxDisp), waited);
        return;
      }
      takenOnTimeout = true;
      MRPT_LOG_WARN_FMT(
        "Taking the map-origin verticality reference after %.1f s with an accelerometer "
        "dispersion of %.2f deg (> %.2f deg): the platform never became still, so the map's "
        "vertical carries that motion.",
        waited, mrpt::RAD2DEG(*disp), mrpt::RAD2DEG(maxDisp));
    }
  }

  state_.gravity_calib_pitch_roll = pr;
  state_.gravity_calib_pose = poseAtCapture;

  // This reference decides the map's vertical for the whole run unless
  // map_gravity takes over, so log what was captured and when: a run whose
  // tilt behavior is questioned later starts here.
  const auto [p0, r0] = *pr;
  MRPT_LOG_INFO_FMT(
    "Map-origin verticality reference captured from the accelerometer%s: "
    "pitch=%.3f roll=%.3f deg (tilt %.3f deg), at pose [%s]",
    takenOnTimeout ? " (on dispersion-gate timeout)" : "", mrpt::RAD2DEG(p0), mrpt::RAD2DEG(r0),
    mrpt::RAD2DEG(std::acos(std::clamp(std::cos(p0) * std::cos(r0), -1.0, 1.0))),
    poseAtCapture.asString().c_str());
}

double LidarOdometry::effectiveGravitySigmaRad() const
{
  // Caller holds state_mtx_; gravity_estimator is fed by the IMU thread:
  auto lckImu = mrpt::lockHelper(imu_state_mtx_);

  const double sigma0 = mrpt::DEG2RAD(params_.imu_gravity_correction.sigma_deg);
  if (!params_.imu_gravity_correction.adaptive_sigma) {
    return sigma0;
  }

  // Add the MEASURED dispersion of the buffered gravity directions in
  // quadrature. Quasi-static -> dispersion ~0 -> sigma ~= sigma0 (unchanged).
  // Under real vehicle dynamics the accepted samples disagree, sigma grows, and
  // the constraint self-silences instead of asserting an aliased tilt that the
  // reading does not actually contain.
  const auto disp = state_.gravity_estimator.directionDispersionSigma(
    params_.imu_gravity_correction.max_age_seconds);
  if (!disp.has_value()) {
    return sigma0;
  }
  return std::sqrt(sigma0 * sigma0 + (*disp) * (*disp));
}

// here happens the main stuff:
void LidarOdometry::processLidarScan(  // NOLINT
  const CObservation::ConstPtr & obs, const std::optional<double> imuCoverageEndTime)
{
  using namespace std::string_literals;

#ifdef MOLA_KERNEL_VIZ_HAS_METRICS
  // Reports the total per-scan "onLidar" cost (all nested profiler sections
  // included) as a live metric. Declared before tle_global so it destructs
  // right after it, once profiler_ has just recorded the finished "onLidar"
  // entry for this scan.
  struct OnLidarTimeMetricReporter
  {
    // Deleting the copy/move members below makes this a non-aggregate, so it
    // needs an explicit constructor for the brace initialization used at the
    // end of the struct.
    explicit OnLidarTimeMetricReporter(LidarOdometry * s) : self(s) {}

    OnLidarTimeMetricReporter(const OnLidarTimeMetricReporter &) = delete;
    OnLidarTimeMetricReporter & operator=(const OnLidarTimeMetricReporter &) = delete;
    OnLidarTimeMetricReporter(OnLidarTimeMetricReporter &&) = delete;
    OnLidarTimeMetricReporter & operator=(OnLidarTimeMetricReporter &&) = delete;

    ~OnLidarTimeMetricReporter()
    {
      if (!self->visualizer_) {
        return;
      }
      if (!self->metric_onlidar_time_ms_) {
        self->metric_onlidar_time_ms_ =
          self->visualizer_->register_metric("lidar_odom/onLidar_time_ms", "ms");
      }
      self->metric_onlidar_time_ms_->push(1000.0 * self->profiler_.getLastTime("onLidar"));
    }
    LidarOdometry * self;
  } onLidarTimeMetricReporter{this};
#endif

  const ProfilerEntry tle_global(profiler_, "onLidar");

  // Check if we need to process any pending async request:
  {
    const ProfilerEntry tle_ur(profiler_, "onLidar.userRequests");
    processPendingUserRequests();
  }

  // make sure data is loaded, if using an offline lazy-load dataset.
  ASSERT_(obs);
  obs->load();
  const auto this_obs_tim = obs->timestamp;

  std::unique_lock<std::mutex> lckState(state_mtx_);

  // Feed all the IMU data belonging to this scan's time span into the
  // IMU-derived state, in timestamp order, before anything reads it. The scan
  // was held back until that data existed (see sendLidarScanToProcessQueue),
  // so the set of samples consumed here is fixed by the timestamps alone:
  if (imuCoverageEndTime) {
    const ProfilerEntry tleImu(profiler_, "onLidar.0.consume_imu");
    consumePendingImu(*imuCoverageEndTime);
  }

  // for rate stats:
  state_.append_lidar_stamp(obs->sensorLabel, obs->timestamp, *this);

  // Only process pointclouds that are sufficiently apart in time:

  // Keep timestamps for logging purposes:
  state_.last_obs_timestamp = this_obs_tim;
  state_.last_obs_reception_time = mrpt::Clock::now();
  if (!state_.first_ever_timestamp) {
    state_.first_ever_timestamp = this_obs_tim;
  }

  // Handle initial localization options:
  if (!state_.initial_localization_done) {
    handleInitialLocalization();

    // Methods that need to converge first (e.g. waiting for IMU samples to
    // calibrate pitch/roll, or for the state estimator to converge) leave
    // initial_localization_done as false until then. Do not process this
    // scan (in particular, do not run ICP nor insert it into the map) until
    // we actually have a valid initial pose.
    if (!state_.initial_localization_done) {
      return;
    }
  }

  if (state_.last_obs_tim_by_label.count(obs->sensorLabel) != 0) {
    const double lidar_delta_time =
      mrpt::system::timeDifference(state_.last_obs_tim_by_label[obs->sensorLabel], this_obs_tim);

    if (lidar_delta_time < params_.min_time_between_scans) {
      // Drop observation.
      MRPT_LOG_DEBUG_FMT(
        "onLidarImpl: dropping observation, for %f< "
        "`min_time_between_scans`=%f.",
        lidar_delta_time, params_.min_time_between_scans);
      return;
    }
  }

  state_.last_obs_tim_by_label[obs->sensorLabel] = this_obs_tim;

  // Use the observation to update the estimated sensor range:
  if (!state_.estimated_observation_radius.has_value()) {
    doInitializeEstimatedObservationRadius(*obs);
  }

  // Handle multiple simultaneous LIDARs:
  const mrpt::obs::CSensoryFrame sf = collectRawObservations(obs);
  if (sf.empty()) {
    return;  // not all required LiDARs yet.
  }

  // Cache sensor poses (in vehicle frame) for GUI visualization:
  for (const auto & o : sf) {
    const mrpt::poses::CPose3D sp = o->getSensorPose();
    state_.last_lidar_sensor_poses[o->sensorLabel] = sp;
  }

  // Refresh dyn. variables used in the mp2p_icp pipelines:
  updatePipelineDynamicVariables(this_obs_tim);

  if (isLoggingLevelVisible(mrpt::system::LVL_DEBUG)) {
    auto lckImu = mrpt::lockHelper(imu_state_mtx_);
    MRPT_LOG_DEBUG_STREAM("Dynamic variables: " << state_.parameter_source.printVariableValues());
  }

  // Extract points from observation:
  auto observation = observationFromRawSensor(sf);

  // Keep a copy of "raw" for visualization in the GUI:
  mp2p_icp::metric_map_t observationRawForViz;
  if (observation->layers.count("raw") != 0) {
    observationRawForViz.layers["raw"] = observation->layers.at("raw");
  }

  // Filter/segment the point cloud (optional, but normally will be present):

  if (!state_.pc_prefilter.empty()) {  // optional pre-filter stage
    ProfilerEntry tle1(profiler_, "onLidar.1.filter_pre");
    mp2p_icp_filters::apply_filter_pipeline(state_.pc_prefilter, *observation, profiler_);
  }

  // Use early deskew?
  const bool use_early_deskew =
    !state_.pc_deskew.empty() && (!params_.optimize_twist || isPipelineUsingIMU_locked());

  if (use_early_deskew) {
    ProfilerEntry tle1(profiler_, "onLidar.1.deskew_early");
    {
      // FilterDeskew snapshots the localVelocityBuffer that the IMU thread
      // appends to (one collect_samples_around_reference_time() call; the
      // per-point work afterwards runs on that private copy). The lock spans
      // the pipeline because the snapshot happens inside the filter:
      auto lckImu = mrpt::lockHelper(imu_state_mtx_);
      mp2p_icp_filters::apply_filter_pipeline(state_.pc_deskew, *observation, profiler_);
    }
    // Now observation has a "deskewed" layer with the full cloud deskewed.
  } else {
    // Fallback:
    observation->layers["deskewed"] = observation->layers["raw"];
  }

  // Keep reference to the deskewed or raw cloud for viz/publish:
  // (shallow copy, just the shared_ptr)
  mrpt::maps::CPointsMap::Ptr fullCloudForVizAndPublish;
  if (observation->layers.count("deskewed") != 0) {
    auto pts = observation->point_layer("deskewed");
    if (pts) {
      fullCloudForVizAndPublish = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(pts);
    }
  }

  {
    ProfilerEntry tle1(profiler_, "onLidar.1.filter_1st");
    mp2p_icp_filters::apply_filter_pipeline(state_.pc_filter1, *observation, profiler_);
  }

  {
    ProfilerEntry tle1(profiler_, "onLidar.1.filter_2nd");
    mp2p_icp_filters::apply_filter_pipeline(state_.pc_filter2, *observation, profiler_);
  }

  // Update sensor max range from the obs map layers:
  doUpdateEstimatedObservationRadius(*observation);

  profiler_.enter("onLidar.2.copy_vars");

  // check observation validity:
  if (const bool obsValid = doCheckIsValidObservation(*observation); !obsValid) {
    MRPT_LOG_WARN_FMT(
      "Observation discarded as non-valid for pathStep=%zu, timestamp=%s UTC",
      state_.estimated_trajectory.size(), mrpt::system::dateTimeToString(this_obs_tim).c_str());

    return;
  }

  // Store for next step:
  std::optional<mrpt::Clock::time_point> last_obs_tim;
  if (auto it = state_.last_obs_tim_by_label.find(obs->sensorLabel);
      it != state_.last_obs_tim_by_label.end()) {
    last_obs_tim = it->second;
  }

  profiler_.leave("onLidar.2.copy_vars");

  if (observation->empty()) {
    MRPT_LOG_WARN_STREAM(
      "Observation of type `" << obs->GetRuntimeClass()->className
                              << "` could not be converted into a "
                                 "pointcloud. Doing nothing.");
    return;
  }

  // The ICP-derived pose corresponds to the vehicle at t=0 of the deskewed
  // cloud, i.e. the local velocity buffer's reference_zero_time (set by
  // Generator to obs.timestamp, then shifted by FilterAdjustTimestamps when
  // configured, e.g. MiddleIsZero -> mid-scan). Use it consistently for any
  // pose-time semantics (state fusion, trajectory, published stamps).
  // With no per-point time adjustment configured this equals obs->timestamp.
  const double scan_ref_time_s = [this]() {
    auto lckImu = mrpt::lockHelper(imu_state_mtx_);
    return state_.parameter_source.localVelocityBuffer.get_reference_zero_time();
  }();
  const auto scan_ref_time =
    scan_ref_time_s > 0 ? mrpt::Clock::fromDouble(scan_ref_time_s) : this_obs_tim;

  // local map: used for LIDAR odometry:
  bool updateLocalMap = false;

  // Simplemap: an optional map to be saved to disk at the end of the mapping
  // session:
  bool updateSimpleMap = false;
  bool distance_enough_sm = false;

#if defined(MOLA_HAS_SHARED_KEYFRAME_MAP_SINK)
  // Whether this scan's keyframe will be pushed to a central-map backend
  // (e.g. mola_mapper_3d), at the same sparsity as the self-written simplemap
  // but independently of whether params_.simplemap.generate is set:
  bool pushToSharedKeyframeMapNow = false;
  // Built inside the lock, invoked after releasing state_mtx_ (see below):
  std::optional<mola::SharedKeyframeMap::KeyframeInsertRequest> pendingKfReq;
  std::shared_ptr<mola::SharedKeyframeMap> pendingKfSink;
#endif

  // First time we cannot do ICP since we need at least two pointclouds:
  ASSERT_(state_.local_map);

  // Request the current pose/twist estimation:
  ProfilerEntry tleMotion(profiler_, "onLidar.2b.estimated_navstate");

  state_.last_motion_model_output =
    state_.navstate_fuse->estimated_navstate(scan_ref_time, params_.publish_reference_frame);

  bool hasMotionModel = state_.last_motion_model_output.has_value();

  // don't count as a valid motion model if its uncertainty is too large:
  if (hasMotionModel) {
    const auto & cov_inv = state_.last_motion_model_output->pose.cov_inv;

    hasMotionModel = cov_inv(0, 0) >= params_.min_motion_model_xyz_cov_inv &&
                     cov_inv(1, 1) >= params_.min_motion_model_xyz_cov_inv &&
                     cov_inv(2, 2) >= params_.min_motion_model_xyz_cov_inv;

    if (!hasMotionModel) {
      MRPT_LOG_DEBUG_STREAM(
        "Discarding motion model at due to large uncertainty: pose_inv_cov=\n"
        << state_.last_motion_model_output->pose.cov_inv.asString());
    }
  }

  tleMotion.stop();

  if (state_.local_map->empty() && params_.local_map_updates.enabled) {
    // Skip ICP.
    MRPT_LOG_DEBUG("First pointcloud: skipping ICP and directly adding to local map.");

    // Create a first KF (at origin)
    updateLocalMap = true;
    updateSimpleMap = true;     // update SimpleMap too
    distance_enough_sm = true;  // and treat this one as a KeyFrame with SF
#if defined(MOLA_HAS_SHARED_KEYFRAME_MAP_SINK)
    pushToSharedKeyframeMapNow = static_cast<bool>(state_.shared_keyframe_map_sink);
#endif

    // Update trajectory too:
    {
      auto lck = mrpt::lockHelper(state_trajectory_mtx_);
      state_.estimated_trajectory.insert(
        scan_ref_time, params_.initial_localization.fixed_initial_pose);
    }

    // Record, once, the absolute IMU-derived tilt at the map origin (if
    // available yet). This is the calibration offset needed to later
    // re-express absolute IMU tilt readings relative to this (possibly
    // non-level) map frame instead of true vertical:
    captureMapOriginVerticality(
      mrpt::poses::CPose3D(params_.initial_localization.fixed_initial_pose));

    // Define the current robot pose at the origin with minimal uncertainty
    // (cannot be zero).
    mrpt::poses::CPose3DPDFGaussian initPose;
    initPose.mean = mrpt::poses::CPose3D(params_.initial_localization.fixed_initial_pose);
    initPose.cov.setDiagonal(1e-12);

    state_.navstate_fuse->fuse_pose(scan_ref_time, initPose, params_.publish_reference_frame);

    // Seed last_lidar_pose and the simplemap keyframe decider with the origin
    // so the next scan's distance check starts from here rather than from
    // "empty" (an empty decider asks for a keyframe at every pose, which would
    // spuriously fire a second keyframe on the very next scan):
    state_.last_lidar_pose = initPose;
    if (!state_.kf_decider_simplemap) {
      state_.kf_decider_simplemap.emplace(params_.simplemap);
    }
    {
      const mrpt::poses::CPose3D sensorPoseInVehicle = obs->getSensorPose();
      state_.kf_decider_simplemap->insert(
        state_.last_lidar_pose.mean + sensorPoseInVehicle, mrpt::Clock::toDouble(obs->timestamp));
    }
  } else {
    // Register point clouds using ICP:
    // ------------------------------------
    profiler_.enter("onLidar.2c.prepare_icp_in");

    // Use velocity model for the initial guess:
    const double dt = last_obs_tim ? mrpt::system::timeDifference(*last_obs_tim, this_obs_tim) : .0;

    ICP_Output out;
    ICP_Input in;

    in.init_guess_local_wrt_global = mrpt::math::TPose3D::Identity();

    if (state_.last_motion_model_output) {
      // ICP initial pose:
      in.init_guess_local_wrt_global = state_.last_motion_model_output->pose.mean.asTPose();

      // ICP prior term: any information!=0?
      if (state_.last_motion_model_output->pose.cov_inv != mrpt::math::CMatrixDouble66::Zero()) {
        // Send it to the ICP solver:
        in.prior.emplace(state_.last_motion_model_output->pose);

        // Special case: 2D lidars mean we are working on SE(2):
        if (std::dynamic_pointer_cast<const mrpt::obs::CObservation2DRangeScan>(obs)) {
          // fix: z, pitch (rot_y), roll (rot_x):
          const double large_certainty = 1e6;

          auto & m = in.prior->mean;

          m.z(0);
          m.setYawPitchRoll(m.yaw(), .0, .0);

          // The ICP solver applies this information matrix to the residual
          // log(P_prior^-1 * P_cur), i.e. in the SE(3) *Lie* tangent, whose
          // rotation entries are the rotation-vector components
          // [3]=w_x (~roll), [4]=w_y (~pitch), [5]=w_z (~yaw). That is NOT
          // MRPT's (x,y,z,yaw,pitch,roll) Euler ordering: the two swap
          // indices 3 and 5. SE(2) fixes z/pitch/roll, leaving x, y and yaw:
          in.prior->cov_inv(2, 2) = large_certainty;  // dz
          in.prior->cov_inv(3, 3) = large_certainty;  // roll  (w_x)
          in.prior->cov_inv(4, 4) = large_certainty;  // pitch (w_y)
        }

        MRPT_LOG_DEBUG_STREAM("ICP prior=" << *in.prior);
      }

      MRPT_LOG_DEBUG_STREAM(
        "Est.twist=" << (hasMotionModel ? state_.last_motion_model_output->twist.asString()
                                        : "(none)"s)
                     << " dt=" << dt << " s. "
                     << " Est. pose: " << state_.last_motion_model_output->pose.mean
                     << "\nEst. pose cov_inv:\n"
                     << state_.last_motion_model_output->pose.cov_inv.asString());
    } else {
      // Use the last pose without velocity motion model:
      in.init_guess_local_wrt_global = state_.last_lidar_pose.mean.asTPose();

      // (Skip the warning message if we are in the first timestep, since that's totally normal and expected, and the warning becomes confusing):
      if (state_.estimated_trajectory.size() > 1) {
        MRPT_LOG_THROTTLE_WARN_FMT(
          2.0,
          "Not able to use velocity motion model for this timestep "
          "(pathStep=%zu, timestamp=%s UTC)",
          state_.estimated_trajectory.size(), mrpt::system::dateTimeToString(this_obs_tim).c_str());
      }
    }

    // Apply the IMU verticality constraint as a yaw-free, rank-2 observation
    // handled by the solver itself: it constrains only the two tilt DOFs and
    // never touches yaw or translation.
#if defined(MOLA_LO_HAS_MP2P_GRAVITY_PRIOR)
    if (params_.imu_gravity_correction.enabled && params_.imu_gravity_correction.use_rank2_prior) {
      in.gravityPrior = buildGravityPrior();
      if (in.gravityPrior.has_value()) {
        MRPT_LOG_DEBUG_FMT(
          "IMU gravity (rank-2): up_body=[%.4f %.4f %.4f] up_map=[%.4f %.4f %.4f] sigma=%.2f deg",
          in.gravityPrior->up_body.x, in.gravityPrior->up_body.y, in.gravityPrior->up_body.z,
          in.gravityPrior->up_map.x, in.gravityPrior->up_map.y, in.gravityPrior->up_map.z,
          mrpt::RAD2DEG(in.gravityPrior->sigma_rad));
      }
    }
#endif

    // Legacy path: fold the gravity-derived pitch/roll into the SE(3) prior.
    if (params_.imu_gravity_correction.enabled && !params_.imu_gravity_correction.use_rank2_prior) {
      // gravity_estimator is fed by the IMU thread; one consistent snapshot for
      // the reading, its dispersion and the buffer size logged below:
      auto lckImu = mrpt::lockHelper(imu_state_mtx_);

      const auto gravityPR = state_.gravity_estimator.estimatedPitchRoll(
        params_.imu_gravity_correction.averaging_samples,
        params_.imu_gravity_correction.max_age_seconds);

      if (gravityPR.has_value()) {
        const auto [imu_pitch, imu_roll] = *gravityPR;

        // The gravity estimator reports *absolute* tilt w.r.t. true vertical,
        // but the map/global frame may not itself be exactly level (e.g. a
        // nonzero `fixed_initial_pose` orientation, or the vehicle starting
        // on a slope). Re-express the absolute IMU tilt relative to the map
        // frame using the tilt recorded at the map origin as a calibration
        // offset. Both readings share the same (gauge) yaw=0 convention, which
        // is valid since pitch/roll extracted from gravity alone are
        // independent of the (unobservable) yaw used to express them.
        double map_pitch = imu_pitch;
        double map_roll = imu_roll;
        if (state_.gravity_calib_pitch_roll.has_value()) {
          const auto [pitch0, roll0] = *state_.gravity_calib_pitch_roll;

          const mrpt::poses::CPose3D & R_map_veh0 = *state_.gravity_calib_pose;
          const mrpt::poses::CPose3D R_grav_veh0(0, 0, 0, 0, pitch0, roll0);
          const mrpt::poses::CPose3D R_grav_veh_t(0, 0, 0, 0, imu_pitch, imu_roll);

          const mrpt::poses::CPose3D R_map_veh_t = R_map_veh0 + (-R_grav_veh0) + R_grav_veh_t;

          map_pitch = R_map_veh_t.pitch();
          map_roll = R_map_veh_t.roll();
        }

        if (!in.prior.has_value()) {
          // Create a prior from current initial guess:
          in.prior.emplace();
          in.prior->mean = mrpt::poses::CPose3D(in.init_guess_local_wrt_global);
          in.prior->cov_inv = mrpt::math::CMatrixDouble66::Zero();
        }

        // Override pitch & roll in the prior mean:
        const double cur_yaw = in.prior->mean.yaw();
        in.prior->mean.setYawPitchRoll(cur_yaw, map_pitch, map_roll);

        const double sigma_rad = effectiveGravitySigmaRad();
        const double inv_var = 1.0 / (sigma_rad * sigma_rad);

        // Gravity observes the two TILT axes only, and must leave yaw free.
        //
        // The ICP solver applies this information matrix to the residual
        // log(P_prior^-1 * P_cur), i.e. in the SE(3) *Lie* tangent, whose
        // rotation entries are the rotation-vector components
        // [3]=w_x (~roll), [4]=w_y (~pitch), [5]=w_z (~yaw). That is NOT
        // MRPT's (x,y,z,yaw,pitch,roll) Euler ordering used by
        // CPose3DPDFGaussian elsewhere: the two swap indices 3 and 5. Writing
        // "roll" at index 5 therefore pinned YAW to the motion model's own
        // yaw and left roll completely unconstrained.
        mrpt::keep_max(in.prior->cov_inv(3, 3), inv_var);  // roll  (w_x)
        mrpt::keep_max(in.prior->cov_inv(4, 4), inv_var);  // pitch (w_y)

        MRPT_LOG_DEBUG_FMT(
          "IMU gravity correction: pitch=%.2f deg, roll=%.2f deg "
          "(sigma=%.1f deg, %zu samples)",
          mrpt::RAD2DEG(imu_pitch), mrpt::RAD2DEG(imu_roll),
          params_.imu_gravity_correction.sigma_deg, state_.gravity_estimator.acc_buffer.size());
      }
    }

    // If we don't have a valid twist estimation, use a larger ICP
    // correspondence threshold:
    in.align_kind = hasMotionModel ? AlignKind::RegularOdometry : AlignKind::NoMotionModel;

    // Run totals (reported at shutdown). Counted here rather than where the
    // warning is emitted, because that one is throttled.
    state_.registrations_attempted++;
    if (!hasMotionModel) {
      state_.registration_no_motion_model++;
    }

    in.icp_params = params_.icp[in.align_kind].icp_parameters;
    in.last_keyframe_pose = state_.last_lidar_pose.mean;

    if (state_.last_icp_timestamp) {
      in.time_since_last_keyframe =
        mrpt::system::timeDifference(*state_.last_icp_timestamp, this_obs_tim);
      state_.last_observed_scan_period_sec = in.time_since_last_keyframe;
    }
    state_.last_icp_timestamp = this_obs_tim;

    profiler_.leave("onLidar.2c.prepare_icp_in");

    // -----------------------------------------------------
    // Run ICP
    // -----------------------------------------------------
    ProfilerEntry tle_icp(profiler_, "onLidar.3.run_icp");
#ifdef MOLA_KERNEL_VIZ_HAS_METRICS
    const double icp_t0 = mrpt::Clock::nowDouble();
#endif

    mrpt::math::TPose3D current_solution = in.init_guess_local_wrt_global;
    size_t twistCorrectionCount = 0;

    auto & icpCase = params_.icp.at(in.align_kind);

    icpCase.icp->setIterationHook([&](const mp2p_icp::ICP::IterationHook_Input & ih) {
      mp2p_icp::ICP::IterationHook_Output ho;

      if (!params_.optimize_twist) {
        return ho;  // not enabled
      }

      if (twistCorrectionCount >= params_.optimize_twist_max_corrections) {
        return ho;
      }

      const auto solutionDelta =
        ih.currentSolution->optimalPose - mrpt::poses::CPose3D(current_solution);

      // check minimum deltas:
      const double deltaTrans = solutionDelta.translation().norm();
      const double deltaRot =
        mrpt::poses::Lie::SO<3>::log(solutionDelta.getRotationMatrix()).norm();

      if (
        deltaTrans > params_.optimize_twist_rerun_min_trans ||
        deltaRot > mrpt::DEG2RAD(params_.optimize_twist_rerun_min_rot_deg)) {
        params_.optimize_twist_max_corrections++;

        MRPT_LOG_DEBUG_STREAM(
          "ICP hook: " << ih.currentIteration << " solutionDelta: trans=" << deltaTrans
                       << " deltaRot=" << mrpt::RAD2DEG(deltaRot));

        // request a restart, saving the new check point:
        ho.request_stop = true;
        current_solution = ih.currentSolution->optimalPose.asTPose();
      }
      return ho;
    });

    mp2p_icp::Results icp_result;
    auto icp_params = in.icp_params;
    size_t remainingIcpIters = icp_params.maxIterations;

    // Debug helper: dump ICP logs to file if quality is under a threshold, or if the timestamp is
    // within a range. Only install the predicate when one of those two triggers is actually
    // configured: mp2p_icp::ICP::align() gives an installed functor unconditional priority over its
    // own `generateDebugFiles`/`MP2P_ICP_GENERATE_DEBUG_FILES` (and that path's `decimationDebugFiles`
    // support), so always installing it here silently shadowed that global switch (see
    // functor_should_generate_debug_file's use in mp2p_icp/src/ICP.cpp).
    thread_local auto MOLA_DEBUG_DUMP_ICP_LOG_FROM_TIMESTAMP =
      mrpt::get_env<double>("MOLA_DEBUG_DUMP_ICP_LOG_FROM_TIMESTAMP", 0);
    thread_local auto MOLA_DEBUG_DUMP_ICP_LOG_TO_TIMESTAMP =
      mrpt::get_env<double>("MOLA_DEBUG_DUMP_ICP_LOG_TO_TIMESTAMP", 0);

    // A half-open or reversed window (TO unset/zero, or earlier than FROM) can never match, so
    // it must not install the functor either: doing so would shadow the native switch without
    // ever dumping anything.
    const bool hasTimestampWindow =
      MOLA_DEBUG_DUMP_ICP_LOG_FROM_TIMESTAMP > 0 &&
      MOLA_DEBUG_DUMP_ICP_LOG_TO_TIMESTAMP >= MOLA_DEBUG_DUMP_ICP_LOG_FROM_TIMESTAMP;

    if (params_.write_debug_icp_log_if_quality_under.has_value() || hasTimestampWindow) {
      icp_params.functor_should_generate_debug_file =
        [this, hasTimestampWindow](const mp2p_icp::LogRecord & log) -> bool {
        const bool cond_1 =
          params_.write_debug_icp_log_if_quality_under.has_value() &&
          log.icpResult.quality < params_.write_debug_icp_log_if_quality_under.value();

        const bool cond_2 =
          (hasTimestampWindow && state_.last_icp_timestamp.has_value() &&
           mrpt::Clock::toDouble(*state_.last_icp_timestamp) >=
             MOLA_DEBUG_DUMP_ICP_LOG_FROM_TIMESTAMP &&
           mrpt::Clock::toDouble(*state_.last_icp_timestamp) <=
             MOLA_DEBUG_DUMP_ICP_LOG_TO_TIMESTAMP);

        return cond_1 || cond_2;
      };
    }

    do {
      icp_params.maxIterations = remainingIcpIters;

      // Skip ICP if we started without map and with mapping disabled:
      if (state_.local_map->empty()) {
        ASSERT_(!params_.local_map_updates.enabled);
        break;
      }

      // Run ICP:
#if defined(MOLA_LO_HAS_MP2P_GRAVITY_PRIOR)
      icpCase.icp->align(
        *observation, *state_.local_map, current_solution, icp_params, icp_result, in.prior,
        std::nullopt /*outputDebugInfo*/, in.gravityPrior);
#else
      icpCase.icp->align(
        *observation, *state_.local_map, current_solution, icp_params, icp_result, in.prior);
#endif

      if (icp_result.nIterations <= remainingIcpIters) {
        remainingIcpIters -= icp_result.nIterations;
      } else {  // who knows?...
        remainingIcpIters = 0;
      }

      if (
        icp_result.terminationReason == mp2p_icp::IterTermReason::HookRequest &&
        in.time_since_last_keyframe > 0) {
        // Re-estimate twist:
        const auto incrPose = icp_result.optimal_tf.mean - in.last_keyframe_pose;

        const double At = in.time_since_last_keyframe;

        mrpt::math::TTwist3D tw;
        tw.vx = incrPose.x() / At;
        tw.vy = incrPose.y() / At;
        tw.vz = incrPose.z() / At;
        const auto logRot = mrpt::poses::Lie::SO<3>::log(incrPose.getRotationMatrix());
        tw.wx = logRot[0] / At;
        tw.wy = logRot[1] / At;
        tw.wz = logRot[2] / At;

        MRPT_LOG_DEBUG_STREAM(
          "ICP hook dt=" << At << ":\nnew estimated twist:" << tw.asString() << "\n"
                         << "old estimated twist:"
                         << state_.last_motion_model_output->twist.asString() << "\n");

        // Update twist dynamic variables, then re-run pipelines.
        // One lock for the update AND the realize(): the accessor takes
        // imu_state_mtx_ itself (recursively), but releasing it in between
        // would let the IMU thread run apply_generators() on variables that
        // are updated but not realized yet.
        {
          auto lckImu = mrpt::lockHelper(imu_state_mtx_);
          updatePipelineTwistVariables(tw);
          // Make all changes effective and evaluate the variables now:
          state_.parameter_source.realize();
        }

        // and re-apply 2nd pass:
        ProfilerEntry tle1c(profiler_, "onLidar.1.filter_2nd");

        mp2p_icp_filters::apply_filter_pipeline(state_.pc_filter2, *observation, profiler_);

        tle1c.stop();

        // for stats:
        profiler_.registerUserMeasure("onLidar.twist_corrections", 1.0);
      }

    } while (icp_result.terminationReason == mp2p_icp::IterTermReason::HookRequest);

    out.found_pose_to_wrt_from = icp_result.optimal_tf;
    out.goodness = icp_result.quality;
    out.icp_iterations = icp_result.nIterations;

#ifdef MOLA_KERNEL_VIZ_HAS_METRICS
    // Stream ICP metrics to the visualizer's live plot windows, if any
    // (mola_viz_imgui "Plots" menu; no-op on the nanogui backend / when
    // nobody is plotting). Registered lazily since visualizer_ may not be
    // available on the very first calls. Guarded by the feature macro so
    // this module still builds against an older mola_kernel.
    if (visualizer_) {
      if (!metric_icp_time_ms_) {
        metric_icp_time_ms_ = visualizer_->register_metric("lidar_odom/icp_time_ms", "ms");
        metric_icp_goodness_ = visualizer_->register_metric("lidar_odom/icp_goodness", "%");
      }
      metric_icp_time_ms_->push(1000.0 * (mrpt::Clock::nowDouble() - icp_t0));
      metric_icp_goodness_->push(100.0 * out.goodness);
    }
#endif

    MRPT_LOG_DEBUG_FMT(
      "ICP (kind=%u): goodness=%.02f%% iters=%u pose=%s "
      "termReason=%s pose_cov diagonal sigmas:{%e %e %e [m] %e %e %e [deg]}",
      static_cast<unsigned int>(in.align_kind), 100.0 * out.goodness,
      static_cast<unsigned int>(icp_result.nIterations),
      out.found_pose_to_wrt_from.getMeanVal().asString().c_str(),
      mrpt::typemeta::enum2str(icp_result.terminationReason).c_str(),
      std::sqrt(out.found_pose_to_wrt_from.cov(0, 0)),
      std::sqrt(out.found_pose_to_wrt_from.cov(1, 1)),
      std::sqrt(out.found_pose_to_wrt_from.cov(2, 2)),
      mrpt::RAD2DEG(std::sqrt(out.found_pose_to_wrt_from.cov(3, 3))),
      mrpt::RAD2DEG(std::sqrt(out.found_pose_to_wrt_from.cov(4, 4))),
      mrpt::RAD2DEG(std::sqrt(out.found_pose_to_wrt_from.cov(5, 5))));

    tle_icp.stop();
    // ------------------------------------------------------
    // (end, run ICP)
    // ------------------------------------------------------

    const bool icpIsGood = (out.goodness >= params_.min_icp_goodness);

    state_.last_icp_was_good = icpIsGood;
    if (!icpIsGood) {
      state_.registration_icp_rejected++;
    }
    state_.last_icp_quality = out.goodness;
    state_.last_icp_iterations = out.icp_iterations;

    if (icpIsGood) {
      state_.last_lidar_pose = out.found_pose_to_wrt_from;
    }

    // Update velocity model:
    // Snapshot before it gets decremented below, so the map-freeze check
    // further down still sees this step as part of the recovery window.
    const uint32_t stepCounterPostRelocalization = state_.step_counter_post_relocalization;

    if (icpIsGood) {
      // Good ICP, update state estimation filter with new data from ICP:

      if (state_.step_counter_post_relocalization == 0) {
        // Do integrate info:
        state_.navstate_fuse->fuse_pose(
          scan_ref_time, out.found_pose_to_wrt_from, params_.publish_reference_frame);
      } else {
        // Skip during post-relocalization:
        state_.step_counter_post_relocalization--;
      }

    } else {
      // Bad ICP:
      // Was: state_.navstate_fuse->reset();
      // Do not reset state estimation in order to allow it to fuse other sensor sources.
    }

    // Update trajectory too:
    if (icpIsGood) {
      auto lck = mrpt::lockHelper(state_trajectory_mtx_);
      state_.estimated_trajectory.insert(scan_ref_time, state_.last_lidar_pose.mean);
    }

    // Retry the map-origin verticality capture if the first keyframe was too
    // early for the accelerometer buffer (no-op once captured).
    if (icpIsGood) {
      captureMapOriginVerticality(state_.last_lidar_pose.mean);
    }

#if defined(MOLA_LO_HAS_MAP_GRAVITY_ESTIMATOR)
    // Close the preintegration interval at this newly-committed pose. Only on a
    // good ICP: the estimator trusts the odometry's RELATIVE attitude, so a
    // failed registration must not enter the window.
    if (
      icpIsGood && params_.imu_gravity_correction.enabled &&
      params_.imu_gravity_correction.map_gravity.enabled && hasMotionModel) {
      closeMapGravityInterval(
        mrpt::Clock::toDouble(scan_ref_time), state_.last_lidar_pose.mean,
        state_.last_motion_model_output->twist);
    }

    // If that solve asked for the map frame to be leveled, do it here: before
    // this scan reaches the local map or the simplemap, so nothing is ever
    // written in the frame that is about to be replaced.
    applyMapFrameRelevel();
#endif

    // Update for stats in CSV format:
    {
      auto lckImu = mrpt::lockHelper(imu_state_mtx_);
      state_.parameter_source.updateVariable("icp_iterations", out.icp_iterations);
      state_.parameter_source.updateVariable(
        "twistCorrectionCount", static_cast<double>(twistCorrectionCount));
      state_.parameter_source.updateVariable("icp_quality", state_.last_icp_quality);
    }

    // Adaptive threshold method:
    // Only update on good ICP: a bad ICP may have converged to a local minimum
    // near the initial guess, yielding an artificially small motion model error
    // that would incorrectly shrink the search threshold.
    if (params_.adaptive_threshold.enabled && icpIsGood) {
      doUpdateAdaptiveThreshold();

      MRPT_LOG_DEBUG_STREAM("Adaptive threshold: sigma=" << state_.adapt_thres_sigma);

    }  // end adaptive threshold

    {
      thread_local auto MOLA_LO_DEBUG_ICP_QUALITY =
        mrpt::get_env<bool>("MOLA_LO_DEBUG_ICP_QUALITY", false);
      if (MOLA_LO_DEBUG_ICP_QUALITY) {
        printf(
          "[LidarOdometry] pathStep=%zu timestamp=%s goodness=%.3f minRequired=%.3f "
          "iters=%u termReason=%s isGood=%d adapt_thres_sigma=%.4f consecutive_bad=%d\n",
          state_.estimated_trajectory.size(), mrpt::system::dateTimeToString(this_obs_tim).c_str(),
          out.goodness, params_.min_icp_goodness, out.icp_iterations,
          mrpt::typemeta::enum2str(icp_result.terminationReason).c_str(), icpIsGood ? 1 : 0,
          state_.adapt_thres_sigma, state_.consecutive_bad_icps);
      }
    }

    // Sustained-failure recovery for the adaptive threshold (opt-in).
    // The rule above never updates sigma on a bad ICP, which is correct
    // for isolated failures but creates a deadlock under sustained failure: with
    // sigma frozen small, the matcher window stays tight and ICP cannot find
    // enough correspondences to recover. When enabled, after a streak of bad
    // ICPs we grow sigma multiplicatively (capped at maximum_sigma) to enlarge
    // the correspondence search radius for the next attempt.
    if (icpIsGood) {
      state_.consecutive_bad_icps = 0;
    } else {
      state_.consecutive_bad_icps++;

      if (
        params_.adaptive_threshold.enabled &&
        params_.adaptive_threshold.recover_on_sustained_failure &&
        state_.consecutive_bad_icps >= params_.adaptive_threshold.recover_after_n_bad) {
        if (state_.adapt_thres_sigma == 0) {
          state_.adapt_thres_sigma = params_.adaptive_threshold.initial_sigma;
        }
        const double new_sigma = std::min(
          state_.adapt_thres_sigma * params_.adaptive_threshold.recover_growth_factor,
          params_.adaptive_threshold.maximum_sigma);
        if (new_sigma > state_.adapt_thres_sigma) {
          MRPT_LOG_THROTTLE_WARN_FMT(
            2.0,
            "Sustained ICP failure (n=%d, q=%.2f): widening adaptive threshold "
            "sigma %.3f -> %.3f m to recover correspondences",
            state_.consecutive_bad_icps, state_.last_icp_quality, state_.adapt_thres_sigma,
            new_sigma);
          state_.adapt_thres_sigma = new_sigma;
          // Reset counter so the KISS-ICP EMA has N bad frames of breathing
          // room before the next growth event; without this, every subsequent
          // bad ICP would immediately re-trigger recovery and sigma would lock
          // at maximum_sigma indefinitely.
          state_.consecutive_bad_icps = 0;
        }
      }
    }

    // Create keyframe deciders on first usage:
    if (!state_.kf_decider_local_map) {
      state_.kf_decider_local_map.emplace(params_.local_map_updates);
    }

    if (!state_.kf_decider_simplemap) {
      state_.kf_decider_simplemap.emplace(params_.simplemap);
    }

    // Use the lidar sensor pose (in world frame) as the distance-checker key.
    // This correctly handles moving lidars and non-repetitive scan patterns,
    // since the sensor itself -- not the base_link -- determines coverage.
    const mrpt::poses::CPose3D sensorPoseInVehicle = obs->getSensorPose();
    const mrpt::poses::CPose3D lidarPoseInWorld = state_.last_lidar_pose.mean + sensorPoseInVehicle;

    const double obsTimestamp = mrpt::Clock::toDouble(obs->timestamp);

    // Create a new KF if we are far enough from the existing ones:
    const auto decisionLocalMap =
      state_.kf_decider_local_map->check(params_.local_map_updates, lidarPoseInWorld, obsTimestamp);

    const bool distFarEnoughLocal = decisionLocalMap.create;
    const double euclidean_dist_since_last = decisionLocalMap.translation;
    const double rot_since_last = decisionLocalMap.rotation;

    // Reuse the post-relocalization recovery window/counter (see
    // additional_map_freeze_after_reloc_how_many_timesteps): only actually
    // freezes anything when that parameter is >0, so leaving it at its
    // default of 0 is a no-op regardless of
    // additional_uncertainty_after_reloc_how_many_timesteps's own value.
    // Use the pre-decrement snapshot so the last step of the recovery window
    // (where pose fusion was still suppressed above) also freezes the map.
    const bool mapFrozenPostReloc =
      params_.initial_localization.additional_map_freeze_after_reloc_how_many_timesteps > 0 &&
      stepCounterPostRelocalization > 0;

    // clang-format off
    updateLocalMap =
      (icpIsGood &&
       // Only if we are in mapping mode:
       params_.local_map_updates.enabled &&
       // skip map update for the special ICP alignment without motion model
       hasMotionModel &&
       distFarEnoughLocal &&
       !mapFrozenPostReloc
       );
    // clang-format on

    if (updateLocalMap) {
      state_.kf_decider_local_map->insert(lidarPoseInWorld, obsTimestamp);

      if (
        params_.local_map_updates.max_distance_to_keep_keyframes > 0 &&
        (state_.localmap_check_removal_counter++ >=
         params_.local_map_updates.check_for_removal_every_n)) {
        const ProfilerEntry tleCleanup(profiler_, "onLidar.distant_kfs_cleanup");

        state_.localmap_check_removal_counter = 0;

        const auto nInit = state_.kf_decider_local_map->size();

        state_.kf_decider_local_map->removeAllFartherThan(
          lidarPoseInWorld, params_.local_map_updates.max_distance_to_keep_keyframes);

        const auto nFinal = state_.kf_decider_local_map->size();
        MRPT_LOG_DEBUG_STREAM("removeAllFartherThan: " << nInit << " => " << nFinal << " KFs");
      }
    }

    // Same shared policy as the local map above, but with the simplemap's own
    // (independently tuned) thresholds and time window:
    const auto decisionSimpleMap =
      state_.kf_decider_simplemap->check(params_.simplemap, lidarPoseInWorld, obsTimestamp);

    distance_enough_sm = decisionSimpleMap.create;

    // clang-format off
    updateSimpleMap =
      params_.simplemap.generate &&
      (icpIsGood &&
       (distance_enough_sm || params_.simplemap.add_non_keyframes_too) &&
       !mapFrozenPostReloc
       );
    // clang-format on

#if defined(MOLA_HAS_SHARED_KEYFRAME_MAP_SINK)
    pushToSharedKeyframeMapNow =
      static_cast<bool>(state_.shared_keyframe_map_sink) && icpIsGood && distance_enough_sm;
#endif

    if (
      (updateSimpleMap
#if defined(MOLA_HAS_SHARED_KEYFRAME_MAP_SINK)
       || pushToSharedKeyframeMapNow
#endif
       ) &&
      distance_enough_sm) {
      // Advance the keyframe-sparsity reference point even when
      // simplemap.generate is disabled, as long as something (the sink)
      // still needs the distance_enough_sm criterion to actually behave as
      // "sparse" instead of always firing (an empty checker reports every
      // pose as "far enough"):
      state_.kf_decider_simplemap->insert(lidarPoseInWorld, obsTimestamp);
    }

    MRPT_LOG_DEBUG_FMT(
      "Since last KF: dist=%5.03f m rotation=%.01f deg updateLocalMap=%s "
      "updateSimpleMap=%s",
      euclidean_dist_since_last, mrpt::RAD2DEG(rot_since_last), updateLocalMap ? "YES" : "NO",
      updateSimpleMap ? "YES" : "NO");

  }  // end: yes, we can do ICP

  // If this was a bad ICP, and we just started with an empty map, re-start again.
  // Do NOT restart if a starting map was loaded (from start-up config, or via
  // a runtime map_load() service call): that would wipe the loaded map.
  if (
    !state_.last_icp_was_good && state_.estimated_trajectory.size() == 1 &&
    params_.local_map_updates.enabled && !state_.map_has_been_loaded) {
    // Re-start the local map:
    {
      auto lckMapContents = mrpt::lockHelper(local_map_content_mtx_);
      state_.local_map->clear();
    }
    state_.estimated_trajectory.clear();
    state_.gravity_calib_pitch_roll.reset();  // new map origin: recapture at next first KF
    state_.gravity_calib_pose.reset();
    state_.gravity_calib_first_available_time.reset();
    updateLocalMap = false;
    state_.last_icp_was_good = true;

    MRPT_LOG_WARN("Bad first ICP, re-starting from scratch with a new local map");
  }

  // Should we create a new KF?
  if (updateLocalMap) {
    ProfilerEntry tle2(profiler_, "onLidar.4.update_local_map");

    // The whole block below mutates the local map layers, so it must exclude
    // a concurrent visualization render (see local_map_content_mtx_ docs):
    auto lckMapContents = mrpt::lockHelper(local_map_content_mtx_);

    // If the local map is empty, create it from this first observation:
    if (state_.local_map->empty()) {
      const ProfilerEntry tle3(profiler_, "onLidar.4.update_local_map.create");
      MRPT_LOG_DEBUG("Creating local map since it was empty");

      for (const auto & o : sf) {
        mp2p_icp_filters::apply_generators(state_.local_map_generators, *o, *state_.local_map);
      }
    }

    ProfilerEntry tle3(profiler_, "onLidar.4.update_local_map.insert");

    // Merge "observation_layers_to_merge_local_map" in local map:
    // Input  metric_map_t: observation
    // Output metric_map_t: state_.local_map

    // 1/4: temporarily make a (shallow) copy of the observation layers into
    // the local map:
    for (const auto & [lyName, lyMap] : observation->layers) {
      ASSERTMSG_(
        state_.local_map->layers.count(lyName) == 0,
        mrpt::format(
          "Error: local map layer name '%s' collides with one of the "
          "observation layers, please use different layer names.",
          lyName.c_str()));

      state_.local_map->layers[lyName] = lyMap;  // shallow copy
    }

    // 2/4: Make sure dynamic variables are up-to-date,
    // in particular, [robot_x, ..., robot_roll]:
    // One lock for the update AND the realize(), see the twist update above:
    {
      auto lckImu = mrpt::lockHelper(imu_state_mtx_);
      updatePipelineDynamicVariablesRobotPoseOnly();
      // Make all changes effective and evaluate the variables now:
      state_.parameter_source.realize();
    }

    // 3/4: Apply pipeline
    mp2p_icp_filters::apply_filter_pipeline(state_.obs2map_merge, *state_.local_map, profiler_);

    // 4/4: remove temporary layers:
    for (const auto & [lyName, lyMap] : observation->layers) {
      state_.local_map->layers.erase(lyName);
    }

    // No more changes to the map contents below:
    lckMapContents.unlock();

    tle3.stop();

    state_.mark_local_map_as_updated();

    tle2.stop();

#ifdef MOLA_KERNEL_VIZ_HAS_METRICS
    // Stream the local-map update cost to the visualizer's live plot
    // windows, mirroring the ICP time/goodness metrics above.
    if (visualizer_) {
      if (!metric_update_local_map_time_ms_) {
        metric_update_local_map_time_ms_ =
          visualizer_->register_metric("lidar_odom/update_local_map_time_ms", "ms");
      }
      metric_update_local_map_time_ms_->push(
        1000.0 * profiler_.getLastTime("onLidar.4.update_local_map"));
    }
#endif

  }  // end done add a new KF to local map

  // Optional build simplemap:
  if (updateSimpleMap) {
    doUpdateSimpleMap(
      sf, distance_enough_sm, observation, scan_ref_time, fullCloudForVizAndPublish);
  }

#if defined(MOLA_HAS_SHARED_KEYFRAME_MAP_SINK)
  // Build the keyframe request while holding state_mtx_; the actual sink call
  // is deferred to after all state accesses so state_mtx_ can be released first:
  if (pushToSharedKeyframeMapNow) {
    mola::SharedKeyframeMap::KeyframeInsertRequest req;
    req.timestamp = scan_ref_time;
    // A DEDICATED source_frame_id, distinct from params_.publish_reference_frame:
    // that one is shared by the dense, every-good-ICP-scan fuse_pose() calls (for
    // short-term prediction), which tie every such scan absolutely to
    // F(publish_reference_frame). Reusing the same frame here would let the sink's
    // anchor-once tie collide with that dense, already-present tie on the very
    // same (relocalization-seeded) first keyframe, which produced a
    // gtsam::IndeterminantLinearSystemException at startup in practice.
    req.source_frame_id = params_.publish_reference_frame + "_kf";
    // state_.last_lidar_pose is this instance's own odometry estimate (in its
    // own, possibly drift-prone frame), exactly as written to the self-built
    // simplemap: the sink chains it via *relative* motion, not absolute.
    req.pose_in_source = state_.last_lidar_pose;
    // Enrich the shared keyframe with the same "metadata" comment (bbox + the
    // local velocity buffer) the self-built simplemap carries, so downstream
    // consumers can deskew the raw scan with the per-keyframe velocity window:
    req.observations = sf;
    appendKeyframeMetadataObs(req.observations, scan_ref_time, *observation);
    req.quality = std::clamp(state_.last_icp_quality, 0.0, 1.0);
    pendingKfReq = std::move(req);
    pendingKfSink = state_.shared_keyframe_map_sink;
  }
#endif

  // In any case, publish the vehicle pose, no matter if it's a keyframe or not,
  // if ICP quality was good enough:
  if (state_.last_icp_was_good) {
    doPublishUpdatedLocalization(scan_ref_time);
  }

  // Prepare deskewed scan for publishing:
  if (params_.publish_deskewed_scans && fullCloudForVizAndPublish && state_.last_icp_was_good) {
    ProfilerEntry tleDs(profiler_, "onLidar.5.prepare_deskewed_publish");

    // If we used the early deskew, fullCloudForVizAndPublish is
    // already deskewed. If in fallback path, we need one final deskew:
    if (!use_early_deskew && observation->layers.count("raw") != 0) {
      mp2p_icp::metric_map_t mm;
      mm.layers["raw"] = observation->layers.at("raw");
      mp2p_icp_filters::apply_filter_pipeline(state_.obsDeskewForViz, mm);
      fullCloudForVizAndPublish =
        std::const_pointer_cast<mrpt::maps::CPointsMap>(mm.point_layer("viz"));
    }

    // Transform to global (map) frame:
    // Publish a transformed cloud to avoid imperfect positioning in RViz / FoxGlove due to latency between /tf and scans:
    auto tfCloud = mrpt::maps::CGenericPointsMap::Create();
    tfCloud->insertAnotherMap(fullCloudForVizAndPublish.get(), state_.last_lidar_pose.mean);
    state_.last_deskewed_scan_for_publishing = tfCloud;
  }

  // Publish new local map & deskewed scan for visualization on external systems (ROS):
  doPublishUpdatedLocalMap(scan_ref_time);

  if (state_.last_icp_was_good) {
    doPublishDeskewedScan(scan_ref_time);
  }

  // Optional debug traces to CSV file:
  doWriteDebugTracesFile(scan_ref_time);

  // Optional real-time GUI via MOLA VizInterface:
  if (visualizer_ && state_.local_map) {
    const ProfilerEntry tle(profiler_, "onLidar.6.updateVisualization");

    if (state_.last_icp_was_good) {
      updateVisualization(observationRawForViz, fullCloudForVizAndPublish, lckState);
    } else {
      // On bad ICP: still update the local map display, GUI panel, and text
      // labels. Skipping the full updateVisualization() avoids 3D pose/path
      // updates that would be based on an unreliable ICP result.
      updateVisualizationAlways(lckState);
    }
  }

#if defined(MOLA_HAS_SHARED_KEYFRAME_MAP_SINK)
  // All state_ accesses are done; release state_mtx_ before invoking the sink
  // so it no longer blocks other state-mutex users during IPC/slow operations:
  if (pendingKfReq) {
    lckState.unlock();
    try {
      pendingKfSink->requestInsertKeyframe(*pendingKfReq);
    } catch (const std::exception & e) {
      MRPT_LOG_WARN_STREAM(
        "shared keyframe map push failed at t="
        << mrpt::system::dateTimeToString(pendingKfReq->timestamp) << ": " << e.what());
    } catch (...) {
      MRPT_LOG_WARN_STREAM(
        "shared keyframe map push failed at t="
        << mrpt::system::dateTimeToString(pendingKfReq->timestamp) << " (unknown exception)");
    }
  }
#endif
}

mp2p_icp::metric_map_t::Ptr LidarOdometry::observationFromRawSensor(
  const mrpt::obs::CSensoryFrame & sf)
{
  auto observation = mp2p_icp::metric_map_t::Create();

  ProfilerEntry tle0(profiler_, "onLidar.0.apply_generators");

  ASSERT_(!sf.empty());
  const auto timeOfFirstSFObs = sf.getObservationByIndex(0)->timestamp;

  for (const auto & o : sf) {
    mp2p_icp::metric_map_t thisObs;
    mp2p_icp::metric_map_t * obsTrg = sf.size() == 1 ? observation.get() : &thisObs;

    {
      // Shares obs_generators (and, through them, the localVelocityBuffer whose
      // reference-zero time they set) with the IMU thread's own
      // apply_generators() call, plus the realize() flags that thread reads:
      auto lckImu = mrpt::lockHelper(imu_state_mtx_);

      mp2p_icp_filters::apply_generators(state_.obs_generators, *o, *obsTrg);

      // Update relative timestamps for multiple lidars:
      const double dt = mrpt::system::timeDifference(timeOfFirstSFObs, o->timestamp);

      state_.parameter_source.updateVariable("SENSOR_TIME_OFFSET", dt);
      // Make all changes effective and evaluate the variables now:
      state_.parameter_source.realize();
    }

    mp2p_icp_filters::apply_filter_pipeline(state_.pc_filterAdjustTimes, *obsTrg, profiler_);

    // for multiple LiDAR setups:
    if (obsTrg != observation.get()) {
      observation->merge_with(*obsTrg);
    }
  }
  tle0.stop();
  return observation;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
mrpt::obs::CSensoryFrame LidarOdometry::collectRawObservations(
  const mrpt::obs::CObservation::ConstPtr & obs)
{
  // Helper: build a sensory frame from sync_obs filtered by max_time_offset
  // around ref_tim, emit debug lines and a partial-group warning, then clear
  // sync_obs. Returns the frame and the count of lidar observations added.
  const auto flushSyncObs =
    [this](const mrpt::Clock::time_point & ref_tim) -> std::pair<mrpt::obs::CSensoryFrame, size_t> {
    mrpt::obs::CSensoryFrame out_sf;
    size_t lidar_obs_in_sf = 0;

    for (const auto & [label, o] : state_.sync_obs) {
      const auto dt = std::abs(mrpt::system::timeDifference(o->timestamp, ref_tim));
      if (dt > params_.multiple_lidars.max_time_offset) {
        MRPT_LOG_DEBUG_FMT(
          "[MULTI_LIDAR_SYNC] Sensor '%s': dt=%.04f s > max_time_offset=%.04f s -> DROPPED",
          label.c_str(), dt, params_.multiple_lidars.max_time_offset);
        continue;
      }
      MRPT_LOG_DEBUG_FMT(
        "[MULTI_LIDAR_SYNC] Sensor '%s': dt=%.04f s -> included in sensory frame", label.c_str(),
        dt);
      out_sf += std::const_pointer_cast<mrpt::obs::CObservation>(o);
      lidar_obs_in_sf++;
    }

    state_.sync_obs.clear();

    if (lidar_obs_in_sf > 0 && lidar_obs_in_sf < params_.multiple_lidars.lidar_count) {
      MRPT_LOG_THROTTLE_WARN_FMT(
        2.0,
        "[MULTI_LIDAR_SYNC] Only %zu/%u LiDAR sensors in sensory frame "
        "(max_time_offset=%.04f s). Processing with partial data.",
        lidar_obs_in_sf, params_.multiple_lidars.lidar_count,
        params_.multiple_lidars.max_time_offset);
    }

    if (isLoggingLevelVisible(mrpt::system::LVL_DEBUG)) {
      std::string grouped_labels;
      for (size_t i = 0; i < out_sf.size(); i++) {
        if (!grouped_labels.empty()) {
          grouped_labels += ", ";
        }
        grouped_labels += out_sf.getObservationByIndex(i)->sensorLabel;
      }
      if (lidar_obs_in_sf < params_.multiple_lidars.lidar_count) {
        MRPT_LOG_DEBUG_FMT(
          "[MULTI_LIDAR_SYNC] Partial group: %zu/%u LiDAR sensors included: [%s]", lidar_obs_in_sf,
          params_.multiple_lidars.lidar_count, grouped_labels.c_str());
      } else {
        MRPT_LOG_DEBUG_FMT(
          "[MULTI_LIDAR_SYNC] All %zu/%u LiDAR sensors grouped successfully. Included: [%s]",
          lidar_obs_in_sf, params_.multiple_lidars.lidar_count, grouped_labels.c_str());
      }
    }

    return {out_sf, lidar_obs_in_sf};
  };

  mrpt::obs::CSensoryFrame sf;
  if (params_.multiple_lidars.lidar_count > 1) {
    // Detect a stale incomplete group: if this sensor label is already in
    // sync_obs, a full scan period elapsed without the other sensor(s) arriving.
    // Flush the old group now to get an estimate from partial data rather than
    // discarding it entirely, then start a fresh group with the current obs.
    if (!state_.sync_obs.empty() && state_.sync_obs.count(obs->sensorLabel) != 0) {
      MRPT_LOG_THROTTLE_WARN_FMT(
        2.0,
        "[MULTI_LIDAR_SYNC] Sensor '%s' arrived again before previous group was complete "
        "(%zu/%u sensors). Flushing incomplete group.",
        obs->sensorLabel.c_str(), state_.sync_obs.size(), params_.multiple_lidars.lidar_count);

      // Use the newest timestamp in the stale group as the time-window reference:
      const auto max_it = std::max_element(
        state_.sync_obs.begin(), state_.sync_obs.end(),
        [](const auto & a, const auto & b) { return a.second->timestamp < b.second->timestamp; });
      const mrpt::Clock::time_point ref_tim = max_it->second->timestamp;

      auto [old_sf, old_count] = flushSyncObs(ref_tim);
      // sync_obs is now clear.
      if (old_count > 0) {
        // Start a new group with the current obs before returning the flushed frame:
        state_.sync_obs[obs->sensorLabel] = obs;
        return old_sf;
      }
      // All stale entries were outside the time window (very pathological):
      // fall through and keep waiting with the new group.
    }

    // Add current observation to the accumulation buffer:
    state_.sync_obs[obs->sensorLabel] = obs;

    // [MULTI_LIDAR_SYNC] Log arrival of each sensor and current sync buffer state:
    if (isLoggingLevelVisible(mrpt::system::LVL_DEBUG)) {
      std::string present_sensors;
      for (const auto & [lbl, o] : state_.sync_obs) {
        if (!present_sensors.empty()) {
          present_sensors += ", ";
        }
        present_sensors += lbl;
        present_sensors += mrpt::format("(t=%.04f)", mrpt::Clock::toDouble(o->timestamp));
      }
      MRPT_LOG_DEBUG_FMT(
        "[MULTI_LIDAR_SYNC] Trigger sensor='%s' t=%.06f. Buffer %zu/%u: [%s]",
        obs->sensorLabel.c_str(), mrpt::Clock::toDouble(obs->timestamp), state_.sync_obs.size(),
        params_.multiple_lidars.lidar_count, present_sensors.c_str());
    }

    if (state_.sync_obs.size() < params_.multiple_lidars.lidar_count) {
      MRPT_LOG_DEBUG_FMT(
        "[MULTI_LIDAR_SYNC] Waiting: have %zu/%u sensors. Still missing %zu sensor(s).",
        state_.sync_obs.size(), params_.multiple_lidars.lidar_count,
        params_.multiple_lidars.lidar_count - state_.sync_obs.size());
      return {};
    }

    // All expected sensors arrived: filter by time window and build the sensory frame:
    auto [full_sf, n] = flushSyncObs(obs->timestamp);
    ASSERT_(n > 0);
    sf = std::move(full_sf);
  } else {
    // Single LIDAR:
    sf.insert(std::const_pointer_cast<mrpt::obs::CObservation>(obs));
  }

  return sf;
}

void LidarOdometry::appendKeyframeMetadataObs(
  mrpt::obs::CSensoryFrame & keyframe_obs, const mrpt::Clock::time_point & scan_ref_time,
  const mp2p_icp::metric_map_t & observation)
{
  using namespace std::string_literals;

  auto metadataObs = mrpt::obs::CObservationComment::Create();
  metadataObs->timestamp = scan_ref_time;
  metadataObs->sensorLabel = "metadata";

  mrpt::containers::yaml kf_metadata = mrpt::containers::yaml::Map();
  std::optional<mrpt::math::TBoundingBoxf> bbox;
  for (const auto & [layerName, layerMap] : observation.layers) {
    if (bbox) {
      bbox = bbox->unionWith(layerMap->boundingBox());
    } else {
      bbox = layerMap->boundingBox();
    }
  }
  if (bbox) {
    kf_metadata["frame_bbox_min"] = "'"s + bbox->min.asString() + "'"s;
    kf_metadata["frame_bbox_max"] = "'"s + bbox->max.asString() + "'"s;
  }

  // Store local velocity buffer in the KF metadata so it is possible to deskew
  // the scan later on with precision.
  kf_metadata["local_velocity_buffer"] = [this]() {
    auto lckImu = mrpt::lockHelper(imu_state_mtx_);
    return state_.parameter_source.localVelocityBuffer.toYAML();
  }();

#if defined(MOLA_LO_HAS_MAP_GRAVITY_ESTIMATOR)
  // A map-frame gauge change makes the keyframe poses of this run
  // non-comparable with those of a run without it, so record it where the
  // poses themselves are stored.
  {
    auto lckImu = mrpt::lockHelper(imu_state_mtx_);
    if (state_.map_gravity.applied_relevel.has_value()) {
      kf_metadata["map_frame_relevel_rotation"] =
        "'"s + state_.map_gravity.applied_relevel->asString() + "'"s;
    }
  }
#endif

  // convert yaml to string:
  std::stringstream ss;
  ss << kf_metadata;
  metadataObs->text = ss.str();

  keyframe_obs.insert(metadataObs);
}

void LidarOdometry::doUpdateSimpleMap(
  const mrpt::obs::CSensoryFrame & sf, const bool distance_enough_sm,
  const mp2p_icp::metric_map_t::Ptr & observation, const mrpt::Clock::time_point & scan_ref_time,
  const mrpt::maps::CPointsMap::Ptr & deskewedCloud)
{
  using namespace std::string_literals;

  auto lck = mrpt::lockHelper(state_simplemap_mtx_);

  auto keyframe_obs = mrpt::obs::CSensoryFrame::Create();
  // Add observations only if this is a real keyframe
  // (the alternative is this is a regular frame, but the option
  //  add_non_keyframes_too is set):
  if (distance_enough_sm) {
    *keyframe_obs += sf;

    if (params_.simplemap.save_deskewed_scans && deskewedCloud) {
      auto od = mrpt::obs::CObservationPointCloud::Create();
      od->timestamp = scan_ref_time;
      auto spc = mrpt::maps::CGenericPointsMap::Create();
      spc->insertAnotherMap(deskewedCloud.get(), {});
      od->pointcloud = spc;
      od->sensorLabel = "deskewed";
      keyframe_obs->insert(od);
    }

    const auto curLidarStamp = scan_ref_time;

    // insert GNSS too? Search for a close-enough observation:
    std::optional<double> closestTimeAbsDiff;
    mrpt::obs::CObservationGPS::ConstPtr closestGPS;

    for (const auto & [gpsStamp, gpsObs] : state_.last_gnss_) {
      const double timeDiff = std::abs(mrpt::system::timeDifference(gpsStamp, curLidarStamp));

      if (timeDiff > params_.simplemap.save_gnss_max_age) {
        continue;
      }

      if (!closestTimeAbsDiff || timeDiff < *closestTimeAbsDiff) {
        closestTimeAbsDiff = timeDiff;
        closestGPS = gpsObs;
      }
    }
    if (closestGPS) {
      *keyframe_obs += std::const_pointer_cast<mrpt::obs::CObservationGPS>(closestGPS);
    }
  } else {
    // Otherwise (we are in here because add_non_keyframes_too).
    // Since we are adding anyway a "comment" observation with the
    // valid timestamp of this frame, it is enough for postprocessing
    // tools.
    ASSERT_(params_.simplemap.add_non_keyframes_too);
  }

  // Add metadata ("comment") observation with bbox + velocity buffer:
  appendKeyframeMetadataObs(*keyframe_obs, scan_ref_time, *observation);

  // Add keyframe to simple map:
  MRPT_LOG_DEBUG_STREAM("New SimpleMap KeyFrame. SF=" << keyframe_obs->size() << " observations.");

  std::optional<mrpt::math::TTwist3D> curTwist;
  if (state_.last_motion_model_output) {
    curTwist = state_.last_motion_model_output->twist;
  }

  state_.reconstructed_simplemap.insert(
    // Pose: mean + covariance
    mrpt::poses::CPose3DPDFGaussian::Create(state_.last_lidar_pose),
    // SensoryFrame: set of observations from this KeyFrame:
    keyframe_obs,
    // twist
    curTwist);

  // Mechanism to free old SFs:
  // We cannot unload them right now, for the case when they are being
  // used in a GUI, etc.
  // (1/2) Add to the list:
  state_.past_simplemaps_observations[scan_ref_time] = keyframe_obs;

  const ProfilerEntry tleUnloadSM(profiler_, "onLidar.5.unload_past_sm_obs");

  // (2/2) Unload old lazy-load observations to save RAM, if applicable:
  constexpr size_t MAX_SIZE_UNLOAD_QUEUE = 100;
  unloadPastSimplemapObservations(MAX_SIZE_UNLOAD_QUEUE);
}

}  // namespace mola
