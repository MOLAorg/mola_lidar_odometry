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

// MRPT:
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/math/slerp.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/datetime.h>

// mp2p_icp
#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/Generator.h>

// Std:
#include <algorithm>
#include <cmath>
#include <iterator>
#include <memory>

namespace
{
// Not std::isnormal(), which also rejects exact zeros (e.g. the z extent of
// a planar 2D scan):
[[nodiscard]] bool isFinitePoint(const mrpt::math::TPoint3Df & pt)
{
  return std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z);
}

// Attitude `R` at time `t_from`, carried to `t_to` (earlier or later) with the
// angular velocity readings (body frame) in between, each one held until the
// next.
[[nodiscard]] mrpt::math::CMatrixDouble33 carryAttitude(
  const mrpt::math::CMatrixDouble33 & R, const std::map<double, mrpt::math::TVector3D> & w,
  const double t_from, const double t_to)
{
  if (w.empty() || t_to == t_from) {
    return R;
  }
  if (t_to < t_from) {
    // Undo the rotation from t_to to t_from (e.g. interleaved LiDAR streams):
    const mrpt::poses::CPose3D fwd(
      carryAttitude(mrpt::math::CMatrixDouble33::Identity(), w, t_to, t_from),
      mrpt::math::CVectorFixedDouble<3>::Zero());
    return R * (-fwd).getRotationMatrix();
  }
  const auto step = [](const mrpt::math::TVector3D & w_b, const double dt) {
    return mrpt::poses::Lie::SO<3>::exp((w_b * dt).asVector<mrpt::math::CVectorFixedDouble<3>>());
  };

  auto it = w.upper_bound(t_from);
  // The reading in effect at t_from: the last one up to it, else the next one.
  mrpt::math::TVector3D w_now = it != w.begin() ? std::prev(it)->second : it->second;
  double t = t_from;
  mrpt::math::CMatrixDouble33 R_t = R;
  for (; it != w.end() && it->first < t_to; ++it) {
    R_t = R_t * step(w_now, it->first - t);
    t = it->first;
    w_now = it->second;
  }
  return R_t * step(w_now, t_to - t);
}
}  // namespace

namespace mola
{

void LidarOdometry::updatePipelineDynamicVariablesRobotPoseOnly()
{
  auto lckImu = mrpt::lockHelper(imu_state_mtx_);

  const auto & p = state_.last_lidar_pose.mean;
  state_.parameter_source.updateVariable("robot_x", p.x());
  state_.parameter_source.updateVariable("robot_y", p.y());
  state_.parameter_source.updateVariable("robot_z", p.z());
  state_.parameter_source.updateVariable("robot_yaw", p.yaw());
  state_.parameter_source.updateVariable("robot_pitch", p.pitch());
  state_.parameter_source.updateVariable("robot_roll", p.roll());
}

void LidarOdometry::updatePipelineDynamicVariables(const mrpt::Clock::time_point & stamp)
{
  // The twist the state estimator holds right after fusing the last scan's
  // pose, and the instant it refers to. last_motion_model_output was queried
  // before that fusion, so it describes the motion of one scan earlier.
  // Only that of the immediately preceding scan is used (an older one may fall
  // outside the de-skew sample window), so it is consumed here. Only the IMU
  // de-skew methods use it, and the query may wait for the estimator.
  std::optional<mrpt::math::TTwist3D> freshTwist;
  double freshTwistStamp = 0;
  if (state_.navstate_fuse && state_.last_fused_scan_ref_time && isPipelineUsingIMU_locked()) {
    const auto ns = state_.navstate_fuse->estimated_navstate(
      *state_.last_fused_scan_ref_time, params_.publish_reference_frame);
    if (ns) {
      freshTwist = ns->twist;
      freshTwistStamp = mrpt::Clock::toDouble(*state_.last_fused_scan_ref_time);
    }
  }
  state_.last_fused_scan_ref_time.reset();

  // Writes the localVelocityBuffer the IMU thread also appends to, and ends
  // with realize(), which flips the "evaluated" flags that thread reads:
  auto lckImu = mrpt::lockHelper(imu_state_mtx_);

  const auto stamp_s = mrpt::Clock::toDouble(stamp);

  // Set dynamic variables for twist usage within ICP pipelines
  // (e.g. de-skew methods)
  {
    // The twist variables (used by the Linear de-skew and other decisions):
    mrpt::math::TTwist3D twistForIcpVars = {0, 0, 0, 0, 0, 0};
    if (state_.last_motion_model_output) {
      twistForIcpVars = state_.last_motion_model_output->twist;
    }

    // The velocity sample of the de-skew buffer, and its stamp: the IMU
    // de-skew methods integrate the accelerometer from it to the sweep.
    mrpt::math::TVector3D bufferVelocity(
      twistForIcpVars.vx, twistForIcpVars.vy, twistForIcpVars.vz);
    double bufferStamp = stamp_s;
    if (freshTwist) {
      bufferVelocity = {freshTwist->vx, freshTwist->vy, freshTwist->vz};
      bufferStamp = freshTwistStamp;
    }

    // Optionally, take the linear velocity from an independent odometry source
    // instead, averaged over this sweep, whose duration is taken as the time
    // since the previous scan:
    state_.deskew_twist_from_odometry = false;
    if (params_.deskew_odometry_sensor_label) {
      constexpr double DEFAULT_SWEEP = 0.1;  // [s]
      double sweep = DEFAULT_SWEEP;
      if (state_.deskew_prev_scan_stamp) {
        const double dt = stamp_s - *state_.deskew_prev_scan_stamp;
        if (dt > 0.01 && dt < 0.5) {
          sweep = dt;
        }
      }
      state_.deskew_prev_scan_stamp = stamp_s;

      if (const auto od = deskewTwistFromOdometry(stamp_s, sweep); od) {
        // Linear part only: the IMU de-skew methods take rotation from the
        // gyroscope, and the angular variables also drive other decisions
        // (e.g. keyframe thresholds), which an odometry's raw, gait-rate
        // angular velocity was measured to destabilize.
        twistForIcpVars.vx = od->twist.vx;
        twistForIcpVars.vy = od->twist.vy;
        twistForIcpVars.vz = od->twist.vz;
        // The mean velocity over the covered interval, i.e. that at its middle:
        bufferVelocity = od->velocity_at_middle;
        bufferStamp = od->middle_stamp;
        state_.deskew_twist_from_odometry = true;
      } else {
        MRPT_LOG_THROTTLE_WARN_FMT(
          5.0,
          "De-skew odometry '%s' does not cover the sweep at t=%.3f; using the state "
          "estimator's velocity for this scan.",
          params_.deskew_odometry_sensor_label->c_str(), stamp_s);
      }
    }

    this->updatePipelineTwistVariables(twistForIcpVars);

    state_.parameter_source.localVelocityBuffer.add_linear_velocity(bufferStamp, bufferVelocity);
    state_.parameter_source.localVelocityBuffer.add_angular_velocity(
      stamp_s, {twistForIcpVars.wx, twistForIcpVars.wy, twistForIcpVars.wz});
  }

  // robot pose:
  updatePipelineDynamicVariablesRobotPoseOnly();
  const auto & p = state_.last_lidar_pose.mean;

  // The attitude sample, at this observation's stamp: that of the last pose,
  // carried from the instant the pose refers to with the angular velocities in
  // between. The IMU de-skew methods remove gravity with it, so the attitude of
  // an earlier instant would leak gravity into the integrated velocity. Kept
  // at this stamp, the sample is always within the de-skew sample window.
  auto & buffer = state_.parameter_source.localVelocityBuffer;
  auto R = p.getRotationMatrix();
  if (state_.last_lidar_pose_stamp) {
    R = carryAttitude(R, buffer.get_angular_velocities(), *state_.last_lidar_pose_stamp, stamp_s);
  }
  buffer.add_orientation(stamp_s, R);

  state_.parameter_source.updateVariable(
    "ADAPTIVE_THRESHOLD_SIGMA", state_.adapt_thres_sigma != 0
                                  ? state_.adapt_thres_sigma
                                  : params_.adaptive_threshold.initial_sigma);

  state_.parameter_source.updateVariable("ICP_ITERATION", 0);

  const auto ensureVarIsDefined = [&](const std::string & varName) {
#if defined(MP2P_ICP_HAS_PARAMETER_SOURCE_HAS_VARIABLE)
    if (!state_.parameter_source.hasVariable(varName)) {
#else
    if (!state_.parameter_source.getVariableValues().count(varName)) {
#endif
      state_.parameter_source.updateVariable(varName, 0);
    }
  };

  // For stats generation in CSV
  ensureVarIsDefined("icp_iterations");
  ensureVarIsDefined("SENSOR_TIME_OFFSET");
  ensureVarIsDefined("twistCorrectionCount");
  ensureVarIsDefined("icp_quality");

  if (state_.estimated_observation_radius) {
    state_.parameter_source.updateVariable(
      "ESTIMATED_OBSERVATION_RADIUS", *state_.estimated_observation_radius);
    // Deprecated alias kept for one release cycle so user pipelines that
    // still reference the old name keep working unchanged. A one-shot
    // warning is emitted at init (see initialize()) when the loaded YAML
    // still uses the legacy name.
    state_.parameter_source.updateVariable(
      "ESTIMATED_SENSOR_MAX_RANGE", *state_.estimated_observation_radius);
  }

  const double instRadius = state_.instantaneous_observation_radius
                              ? *state_.instantaneous_observation_radius
                              : 20.0 /*default, only used once*/;
  state_.parameter_source.updateVariable("INSTANTANEOUS_OBSERVATION_RADIUS", instRadius);
  // Deprecated alias, see comment above.
  state_.parameter_source.updateVariable("INSTANTANEOUS_SENSOR_MAX_RANGE", instRadius);

  if (state_.last_obs_timestamp && state_.first_ever_timestamp) {
    state_.parameter_source.updateVariable(
      "current_relative_timestamp",
      mrpt::system::timeDifference(*state_.first_ever_timestamp, *state_.last_obs_timestamp));
  }

  // Make all changes effective and evaluate the variables now:
  state_.parameter_source.realize();
}

void LidarOdometry::doUpdateAdaptiveThreshold()
{
  if (!state_.estimated_observation_radius.has_value()) {
    return;
  }

  // Initialization guard FIRST
  if (state_.adapt_thres_sigma == 0) {
    state_.adapt_thres_sigma = params_.adaptive_threshold.initial_sigma;
  }

  const double ALPHA = params_.adaptive_threshold.alpha;
  const double KP = params_.adaptive_threshold.kp;
  ASSERT_(KP > 0.0);  // any positive gain is valid

  // Additive P-controller: positive error → increase sigma
  const double error =
    params_.adaptive_threshold.icp_quality_controller_setpoint - state_.last_icp_quality;
  const double delta = std::clamp(
    KP * error, -params_.adaptive_threshold.max_sigma_step,
    params_.adaptive_threshold.max_sigma_step);
  const double new_sigma = state_.adapt_thres_sigma + delta;

  // EMA smoothing
  state_.adapt_thres_sigma = ALPHA * state_.adapt_thres_sigma + (1.0 - ALPHA) * new_sigma;

  // Hard bounds last
  mrpt::saturate(
    state_.adapt_thres_sigma, params_.adaptive_threshold.min_motion,
    params_.adaptive_threshold.maximum_sigma);

  MRPT_LOG_DEBUG_FMT(
    "delta: %+.4f ICP q=%.3f sigma=%.4f", delta, state_.last_icp_quality, state_.adapt_thres_sigma);
}

namespace
{
/** Radius of the observation, as either the bounding-box max-norm (the
 * historical behavior, `quantile` = 1) or a quantile of the per-point norms.
 *
 * The max-norm is set by a single point, so a stray far return decides the
 * scale of the whole scene; six shipped parameters are derived from this
 * number. The quantile path samples with a stride instead of reading every
 * point: it needs the shape of the distribution, not all of it.
 */
/** The quantile path only. The default (`quantile` = 1) is deliberately NOT
 * routed through here: see the call sites.
 */
double observationRadiusQuantile(
  const mrpt::maps::CPointsMap & pts, double quantile, uint32_t maxSamples)
{
  const auto bb = pts.boundingBox();
  const double maxNorm = std::max(bb.max.norm(), bb.min.norm());

  const auto & xs = pts.getPointsBufferRef_x();
  const auto & ys = pts.getPointsBufferRef_y();
  const auto & zs = pts.getPointsBufferRef_z();
  const size_t n = xs.size();
  if (n < 2) {
    return maxNorm;
  }

  const size_t stride = std::max<size_t>(1, n / std::max<uint32_t>(1, maxSamples));
  std::vector<double> norms;
  norms.reserve(n / stride + 1);
  for (size_t i = 0; i < n; i += stride) {
    const double r2 = static_cast<double>(xs[i]) * xs[i] + static_cast<double>(ys[i]) * ys[i] +
                      static_cast<double>(zs[i]) * zs[i];
    if (std::isfinite(r2)) {
      norms.push_back(std::sqrt(r2));
    }
  }
  if (norms.empty()) {
    return maxNorm;
  }

  const size_t k =
    std::min(norms.size() - 1, static_cast<size_t>(quantile * (norms.size() - 1) + 0.5));
  std::nth_element(norms.begin(), norms.begin() + k, norms.end());
  return norms[k];
}
}  // namespace

void LidarOdometry::doInitializeEstimatedObservationRadius(const mrpt::obs::CObservation & o)
{
  auto & maxRange = state_.estimated_observation_radius;
  ASSERT_(!maxRange.has_value());  // this method is for 1st call only

  mp2p_icp_filters::Generator gen;
  gen.params.target_layer = "raw";
  gen.initialize({});

  mp2p_icp::metric_map_t map;
  gen.process(o, map);

  auto pts = map.point_layer("raw");

  if (pts->empty()) {
    return;
  }

  // The default keeps the original expression, in its original place: moving
  // it into a function changes its inlining context, and a last-ULP
  // difference in R is enough to flip one point across a range threshold on
  // a pipeline this sensitive to its input. Measured: routing the default
  // through the helper changed the trajectory on 2 of 14 sequences.
  const auto bb = pts->boundingBox();

  double radius = params_.observation_radius_quantile >= 1.0
                    ? std::max(bb.max.norm(), bb.min.norm())
                    : observationRadiusQuantile(
                        *pts, params_.observation_radius_quantile,
                        params_.observation_radius_quantile_max_samples);

  // check for NaN, Infinities, etc.: See: https://github.com/MOLAorg/mola_lidar_odometry/issues/10
  if (!std::isnormal(radius)) {
    MRPT_LOG_WARN_STREAM(
      "NaN bounding box for observation point cloud. Using default observation radius.");
    radius = params_.absolute_minimum_observation_radius;
  } else {
    mrpt::keep_max(radius, params_.absolute_minimum_observation_radius);
  }

  maxRange = radius;

  MRPT_LOG_DEBUG_STREAM(
    "Estimated observation radius=" << *state_.estimated_observation_radius
                                    << " (instantaneous=" << radius << ")");
}

bool LidarOdometry::doUpdateEstimatedObservationRadius(const mp2p_icp::metric_map_t & m)
{
  const double ALPHA = params_.observation_radius_filter_coefficient;

  auto & maxRange = state_.estimated_observation_radius;
  if (!maxRange.has_value()) {
    return false;  // doInitializeEstimatedObservationRadius() has not succeeded yet
  }

  for (const auto & [layerName, layer] : m.layers) {
    auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(layer);
    if (!pts || pts->empty()) {
      continue;
    }

    const auto bb = pts->boundingBox();
    if (!isFinitePoint(bb.min) || !isFinitePoint(bb.max)) {
      continue;  // skip NaN, INF, etc.
    }

    // The default keeps the original expression, in its original place: moving
    // it into a function changes its inlining context, and a last-ULP
    // difference in R is enough to flip one point across a range threshold on
    // a pipeline this sensitive to its input. Measured: routing the default
    // through the helper changed the trajectory on 2 of 14 sequences.
    double radius = params_.observation_radius_quantile >= 1.0
                      ? std::max(bb.max.norm(), bb.min.norm())
                      : observationRadiusQuantile(
                          *pts, params_.observation_radius_quantile,
                          params_.observation_radius_quantile_max_samples);

    mrpt::keep_max(radius, params_.absolute_minimum_observation_radius);

    state_.instantaneous_observation_radius = radius;

    // low-pass filter update:
    maxRange = (maxRange.value() * ALPHA) + (radius * (1.0 - ALPHA));

    MRPT_LOG_DEBUG_STREAM(
      "Estimated observation radius=" << *state_.estimated_observation_radius
                                      << " (instantaneous=" << radius << ")");

    // one layer is enough:
    return true;
  }
  MRPT_LOG_DEBUG(
    "Estimated observation radius could NOT be updated, no points layer "
    "found in observation metric_map_t");
  return false;
}

std::optional<LidarOdometry::DeskewOdometryTwist> LidarOdometry::deskewTwistFromOdometry(
  const double t0, const double span) const
{
  // Caller holds imu_state_mtx_.
  const auto & poses = state_.deskew_odometry_poses;
  if (poses.size() < 2 || span <= 0) {
    return {};
  }

  // Pose at time t, interpolated between the two bracketing samples:
  const auto poseAt = [&](const double t) -> std::optional<mrpt::poses::CPose3D> {
    auto it1 = poses.lower_bound(t);
    if (it1 == poses.end()) {
      return {};
    }
    if (it1->first == t) {
      return it1->second;
    }
    if (it1 == poses.begin()) {
      return {};
    }
    const auto it0 = std::prev(it1);
    const double frac = (t - it0->first) / (it1->first - it0->first);
    mrpt::math::TPose3D p;
    mrpt::math::slerp(it0->second.asTPose(), it1->second.asTPose(), frac, p);
    return mrpt::poses::CPose3D(p);
  };

  // The sweep may not be fully covered: its end, e.g. online, when the scan
  // arrives before the odometry of its last instants; its start, when the
  // odometry begins mid-sweep. Use the covered part if it is at least half.
  const double ta = std::max(t0, poses.begin()->first);
  const double tb = std::min(t0 + span, poses.rbegin()->first);
  if (tb - ta < 0.5 * span) {
    return {};
  }

  const auto pa = poseAt(ta);
  const auto pb = poseAt(tb);
  if (!pa || !pb) {
    return {};
  }

  const double dt = tb - ta;
  const auto incr = *pb - *pa;  // in the vehicle frame at ta
  const auto logRot = mrpt::poses::Lie::SO<3>::log(incr.getRotationMatrix());

  DeskewOdometryTwist out;
  auto & tw = out.twist;
  tw.vx = incr.x() / dt;
  tw.vy = incr.y() / dt;
  tw.vz = incr.z() / dt;
  tw.wx = logRot[0] / dt;
  tw.wy = logRot[1] / dt;
  tw.wz = logRot[2] / dt;

  // The same velocity in the vehicle frame at the middle of the interval,
  // half of the rotation later (constant angular velocity):
  auto halfRot = logRot;
  halfRot *= 0.5;
  const mrpt::poses::CPose3D midFrame(
    mrpt::poses::Lie::SO<3>::exp(halfRot), mrpt::math::CVectorFixedDouble<3>::Zero());
  out.velocity_at_middle = midFrame.inverseRotateVector({tw.vx, tw.vy, tw.vz});
  out.middle_stamp = 0.5 * (ta + tb);
  return out;
}

void LidarOdometry::updatePipelineTwistVariables(const mrpt::math::TTwist3D & tw)
{
  auto lckImu = mrpt::lockHelper(imu_state_mtx_);

  state_.parameter_source.updateVariable("vx", tw.vx);
  state_.parameter_source.updateVariable("vy", tw.vy);
  state_.parameter_source.updateVariable("vz", tw.vz);
  state_.parameter_source.updateVariable("wx", tw.wx);
  state_.parameter_source.updateVariable("wy", tw.wy);
  state_.parameter_source.updateVariable("wz", tw.wz);
}

}  // namespace mola
