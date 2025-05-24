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

// MRPT:
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/TTwist3D.h>
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
#include <memory>

namespace
{
[[nodiscard]] bool isNormalPoint(const mrpt::math::TPoint3Df & pt)
{
  return std::isnormal(pt.x) && std::isnormal(pt.y) && std::isnormal(pt.z);
}
}  // namespace

namespace mola
{

void LidarOdometry::updatePipelineDynamicVariables()
{
  // Set dynamic variables for twist usage within ICP pipelines
  // (e.g. de-skew methods)
  {
    mrpt::math::TTwist3D twistForIcpVars = {0, 0, 0, 0, 0, 0};
    if (state_.last_motion_model_output) {
      twistForIcpVars = state_.last_motion_model_output->twist;
    }

    this->updatePipelineTwistVariables(twistForIcpVars);
  }

  // robot pose:
  const auto & p = state_.last_lidar_pose.mean;
  state_.parameter_source.updateVariable("robot_x", p.x());
  state_.parameter_source.updateVariable("robot_y", p.y());
  state_.parameter_source.updateVariable("robot_z", p.z());
  state_.parameter_source.updateVariable("robot_yaw", p.yaw());
  state_.parameter_source.updateVariable("robot_pitch", p.pitch());
  state_.parameter_source.updateVariable("robot_roll", p.roll());

  state_.parameter_source.updateVariable(
    "ADAPTIVE_THRESHOLD_SIGMA", state_.adapt_thres_sigma != 0
                                  ? state_.adapt_thres_sigma
                                  : params_.adaptive_threshold.initial_sigma);

  state_.parameter_source.updateVariable("ICP_ITERATION", 0);

  const auto ensureVarIsDefined = [&](const std::string & varName) {
    if (!state_.parameter_source.getVariableValues().count(varName)) {
      state_.parameter_source.updateVariable(varName, 0);
    }
  };

  ensureVarIsDefined("icp_iterations");
  ensureVarIsDefined("SENSOR_TIME_OFFSET");
  ensureVarIsDefined("twistCorrectionCount");

  if (state_.estimated_sensor_max_range) {
    state_.parameter_source.updateVariable(
      "ESTIMATED_SENSOR_MAX_RANGE", *state_.estimated_sensor_max_range);
  }

  state_.parameter_source.updateVariable(
    "INSTANTANEOUS_SENSOR_MAX_RANGE", state_.instantaneous_sensor_max_range
                                        ? *state_.instantaneous_sensor_max_range
                                        : 20.0 /*default, only used once*/);

  if (state_.last_obs_timestamp && state_.first_ever_timestamp) {
    state_.parameter_source.updateVariable(
      "current_relative_timestamp",
      mrpt::system::timeDifference(*state_.first_ever_timestamp, *state_.last_obs_timestamp));
  }

  // Make all changes effective and evaluate the variables now:
  state_.parameter_source.realize();
}

// KISS-ICP adaptive threshold method (MIT License; code adapted to MRPT)
namespace
{
double computeModelError(const mrpt::poses::CPose3D & model_deviation, double max_range)
{
  const double theta = mrpt::poses::Lie::SO<3>::log(model_deviation.getRotationMatrix()).norm();
  const double delta_rot = 2.0 * max_range * std::sin(theta / 2.0);
  const double delta_trans = model_deviation.translation().norm();
  return delta_trans + delta_rot;
}
}  // namespace

void LidarOdometry::doUpdateAdaptiveThreshold(const mrpt::poses::CPose3D & lastMotionModelError)
{
  if (!state_.estimated_sensor_max_range.has_value()) return;

  const double max_range = state_.estimated_sensor_max_range.value();

  const double ALPHA = params_.adaptive_threshold.alpha;

  const double model_error = computeModelError(lastMotionModelError, max_range);
  double rot_error = 0;
  if (state_.last_motion_model_output) {
    const auto & tw = state_.last_motion_model_output->twist;
    rot_error = 0.1 * mrpt::math::TVector3D(tw.wx, tw.wy, tw.wz).norm() * max_range;
  }

  computeModelError(lastMotionModelError, max_range);

  // proportional controller constant
  const double KP = params_.adaptive_threshold.kp;
  ASSERT_(KP > 1.0);

  const double new_sigma =
    (model_error + rot_error) * mrpt::saturate_val(KP * (1.0 - state_.last_icp_quality), 0.1, KP);

  if (state_.adapt_thres_sigma == 0)  // initial
    state_.adapt_thres_sigma = params_.adaptive_threshold.initial_sigma;

  state_.adapt_thres_sigma = ALPHA * state_.adapt_thres_sigma + (1.0 - ALPHA) * new_sigma;

  mrpt::saturate(
    state_.adapt_thres_sigma, params_.adaptive_threshold.min_motion,
    params_.adaptive_threshold.maximum_sigma);

  MRPT_LOG_DEBUG_FMT(
    "model_error: %f  new_sigma: %f ICP q=%f sigma=%f", model_error, new_sigma,
    state_.last_icp_quality, state_.adapt_thres_sigma);
}

void LidarOdometry::doInitializeEstimatedMaxSensorRange(const mrpt::obs::CObservation & o)
{
  auto & maxRange = state_.estimated_sensor_max_range;
  ASSERT_(!maxRange.has_value());  // this method is for 1st call only

  mp2p_icp_filters::Generator gen;
  gen.params_.target_layer = "raw";
  gen.initialize({});

  mp2p_icp::metric_map_t map;
  gen.process(o, map);

  auto pts = map.point_layer("raw");

  if (pts->empty()) return;

  const auto bb = pts->boundingBox();

  double radius = std::max(bb.max.norm(), bb.min.norm());

  // check for NaN, Infinities, etc.: See: https://github.com/MOLAorg/mola_lidar_odometry/issues/10
  if (!std::isnormal(radius)) {
    MRPT_LOG_WARN_STREAM("NaN bounding box for sensor point cloud. Using default sensor range.");
    radius = params_.absolute_minimum_sensor_range;
  } else {
    mrpt::keep_max(radius, params_.absolute_minimum_sensor_range);
  }

  maxRange = radius;

  MRPT_LOG_DEBUG_STREAM(
    "Estimated sensor max range=" << *state_.estimated_sensor_max_range
                                  << " (instantaneous=" << radius << ")");
}

void LidarOdometry::doUpdateEstimatedMaxSensorRange(const mp2p_icp::metric_map_t & m)
{
  const double ALPHA = params_.max_sensor_range_filter_coefficient;

  auto & maxRange = state_.estimated_sensor_max_range;
  ASSERT_(maxRange.has_value());  // this method is for subsequent calls only

  for (const auto & [layerName, layer] : m.layers) {
    auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(layer);
    if (!pts || pts->empty()) continue;

    const auto bb = pts->boundingBox();
    if (!isNormalPoint(bb.min) || !isNormalPoint(bb.max)) continue;  // skip NaN, INF, etc.

    double radius = std::max(bb.max.norm(), bb.min.norm());

    mrpt::keep_max(radius, params_.absolute_minimum_sensor_range);

    state_.instantaneous_sensor_max_range = radius;

    // low-pass filter update:
    maxRange = maxRange.value() * ALPHA + radius * (1.0 - ALPHA);

    MRPT_LOG_DEBUG_STREAM(
      "Estimated sensor max range=" << *state_.estimated_sensor_max_range
                                    << " (instantaneous=" << radius << ")");

    // one layer is enough:
    return;
  }
  MRPT_LOG_DEBUG(
    "Estimated sensor max range could NOT be updated, no points layer "
    "found in observation metric_map_t");
}

void LidarOdometry::updatePipelineTwistVariables(const mrpt::math::TTwist3D & tw)
{
  state_.parameter_source.updateVariable("vx", tw.vx);
  state_.parameter_source.updateVariable("vy", tw.vy);
  state_.parameter_source.updateVariable("vz", tw.vz);
  state_.parameter_source.updateVariable("wx", tw.wx);
  state_.parameter_source.updateVariable("wy", tw.wy);
  state_.parameter_source.updateVariable("wz", tw.wz);
}

}  // namespace mola
