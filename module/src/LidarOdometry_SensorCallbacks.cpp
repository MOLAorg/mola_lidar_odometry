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

// Others:
#include <mola_kernel/interfaces/ExecutableBase.h>  // mola::ProfilerEntry
#include <mrpt/core/exceptions.h>                   // MRPT_TRY_START
#include <mrpt/core/lock_helper.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>

// Std:
#include <regex>

namespace mola
{

void LidarOdometry::onNewObservation(const CObservation::Ptr & o)
{
  MRPT_TRY_START
  const ProfilerEntry tleg(profiler_, "onNewObservation");

  ASSERT_(o);

  {
    auto lckStateFlags = mrpt::lockHelper(state_flags_mtx_);

    if (!state_.initialized) {
      MRPT_LOG_THROTTLE_ERROR(
        2.0,
        "Discarding incoming observations: the system initialize() method has not been called "
        "yet!");
      return;
    }
    if (state_.fatal_error) {
      MRPT_LOG_THROTTLE_ERROR(
        2.0, "Discarding incoming observations: a fatal error ocurred above.");

      this->requestShutdown();  // request end of mola-cli app, if applicable
      return;
    }

    // SLAM enabled?
    if (!state_.active) {
      // and do not process the observation:
      return;
    }
  }

  // Is it an IMU obs?
  if (
    params_.imu_sensor_label &&
    std::regex_match(o->sensorLabel, params_.imu_sensor_label.value())) {
    {
      auto lck = mrpt::lockHelper(is_busy_mtx_);
      state_.worker_tasks_others++;
    }

    // Yes, it's an IMU obs:
    auto fut = worker_.enqueue(&LidarOdometry::onIMU, this, o);
    (void)fut;
  }

  // Is it GNSS?
  if (
    params_.gnss_sensor_label &&
    std::regex_match(o->sensorLabel, params_.gnss_sensor_label.value())) {
    {
      auto lck = mrpt::lockHelper(is_busy_mtx_);
      state_.worker_tasks_others++;
    }
    auto fut = worker_.enqueue(&LidarOdometry::onGPS, this, o);
    (void)fut;
  }

  // Is it a LIDAR obs?
  for (const auto & re : params_.lidar_sensor_labels) {
    if (!std::regex_match(o->sensorLabel, re)) continue;

    // Yes, it's a LIDAR obs:
    const int queued = [this]() {
      auto lck = mrpt::lockHelper(is_busy_mtx_);
      return state_.worker_tasks_lidar;
    }();

    profiler_.registerUserMeasure("onNewObservation.lidar_queue_length", queued);
    if (queued > params_.max_lidar_queue_before_drop) {
      MRPT_LOG_THROTTLE_WARN_FMT(
        1.0, "Dropping observation due to LiDAR worker thread too busy (dropped frames: %.02f%%)",
        getDropStats() * 100.0);
      profiler_.registerUserMeasure("onNewObservation.drop_observation", 1);
      addDropStats(true);
      return;
    }
    addDropStats(false);
    profiler_.enter("delay_onNewObs_to_process");

    {
      auto lck = mrpt::lockHelper(is_busy_mtx_);
      state_.worker_tasks_lidar++;
    }

    // Enqueue task:
    auto fut = worker_.enqueue(&LidarOdometry::onLidar, this, o);

    (void)fut;

    break;  // do not keep processing the list
  }

  MRPT_TRY_END
}

void LidarOdometry::onLidar(const CObservation::Ptr & o)
{
  const bool abort_running = [this]() {
    auto lck = mrpt::lockHelper(is_busy_mtx_);
    return destructor_called_;
  }();

  // All methods that are enqueued into a thread pool should have its own
  // top-level try-catch:
  if (!abort_running) {
    try {
      processLidarScan(o);
    } catch (const std::exception & e) {
      MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
      auto lckStateFlags = mrpt::lockHelper(state_flags_mtx_);
      state_.fatal_error = true;
    }
  }

  {
    auto lck = mrpt::lockHelper(is_busy_mtx_);
    state_.worker_tasks_lidar--;
  }
}

void LidarOdometry::onIMU(const CObservation::Ptr & o)
{
  // All methods that are enqueued into a thread pool should have its own
  // top-level try-catch:
  try {
    onIMUImpl(o);
  } catch (const std::exception & e) {
    MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
    auto lckStateFlags = mrpt::lockHelper(state_flags_mtx_);
    state_.fatal_error = true;
  }

  {
    auto lck = mrpt::lockHelper(is_busy_mtx_);
    state_.worker_tasks_others--;
  }
}

void LidarOdometry::onIMUImpl(const CObservation::Ptr & o)
{
  ASSERT_(o);

  const ProfilerEntry tleg(profiler_, "onIMU");

  auto imu = std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(o);
  ASSERTMSG_(
    imu, mrpt::format(
           "IMU observation with label '%s' does not have the expected "
           "type 'mrpt::obs::CObservationIMU', it is '%s' instead",
           o->sensorLabel.c_str(), o->GetRuntimeClass()->className));

  MRPT_LOG_DEBUG_STREAM(
    "onIMU called for timestamp=" << mrpt::system::dateTimeLocalToString(imu->timestamp));

  // Was: state_.navstate_fuse->fuse_imu(*imu);
  // But since March-2025, state estimators actively subscribe to sensor inputs and it is not
  // our responsibility to forward IMU to them.

  // Uses of IMU in MOLA-LO (this class):
  // 1) During special initialization to compensate for pitch/roll;
  // 2) (TODO!) Improved scan de-skewing.

  // 1) Initial pitch/roll estimation:
  {
    auto lckState = mrpt::lockHelper(state_mtx_);
    if (state_.imu_averager.has_value()) {
      state_.imu_averager->add(imu);
    }
  }

  if (
    !imu->has(mrpt::obs::IMU_X_ACC) || !imu->has(mrpt::obs::IMU_Y_ACC) ||
    !imu->has(mrpt::obs::IMU_Z_ACC)) {
    // No acceleration data:
    return;
  }

  const auto accel_sensor = mrpt::math::TTwist3D(  //
    imu->get(mrpt::obs::IMU_X_ACC),                //
    imu->get(mrpt::obs::IMU_Y_ACC),                //
    imu->get(mrpt::obs::IMU_Z_ACC),                //
    0, 0, 0);

  const auto accel_base_link = accel_sensor.rotated(imu->sensorPose.asTPose());

  // TODO(jlbc): Continue with scan de-skewing using IMU data
  (void)accel_base_link;
}

void LidarOdometry::onGPS(const CObservation::Ptr & o)
{
  // All methods that are enqueued into a thread pool should have its own
  // top-level try-catch:
  try {
    onGPSImpl(o);
  } catch (const std::exception & e) {
    MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
    auto lckStateFlags = mrpt::lockHelper(state_flags_mtx_);
    state_.fatal_error = true;
  }

  {
    auto lck = mrpt::lockHelper(is_busy_mtx_);
    state_.worker_tasks_others--;
  }
}

void LidarOdometry::onGPSImpl(const CObservation::Ptr & o)
{
  ASSERT_(o);

  const ProfilerEntry tleg(profiler_, "onGPS");

  auto gps = std::dynamic_pointer_cast<mrpt::obs::CObservationGPS>(o);
  ASSERTMSG_(
    gps, mrpt::format(
           "GPS observation with label '%s' does not have the expected "
           "type 'mrpt::obs::CObservationGPS', it is '%s' instead",
           o->sensorLabel.c_str(), o->GetRuntimeClass()->className));

  MRPT_LOG_DEBUG_FMT("GNSS observation received, t=%.03f", mrpt::Clock::toDouble(gps->timestamp));

  // Keep the latest GPS observations for simplemap insertion:
  state_.last_gnss_.emplace(gps->timestamp, gps);

  // remove old ones:
  while (state_.last_gnss_.size() > params_.gnss_queue_max_size) {
    state_.last_gnss_.erase(state_.last_gnss_.begin());
  }
}

bool LidarOdometry::doCheckIsValidObservation(const mp2p_icp::metric_map_t & m)
{
  if (!params_.observation_validity_checks.enabled) {
    return true;  // it's valid
  }

  auto it = m.layers.find(params_.observation_validity_checks.check_layer_name);
  ASSERTMSG_(
    it != m.layers.end(),
    mrpt::format(
      "Observation validity check expected observation layer '%s' but did not exist",
      params_.observation_validity_checks.check_layer_name.c_str()));

  auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(it->second);
  ASSERTMSG_(
    pts, mrpt::format(
           "Observation validity check expected observation layer '%s' of type CPointsMap",
           params_.observation_validity_checks.check_layer_name.c_str()));

  const bool valid = pts->size() > params_.observation_validity_checks.minimum_point_count;

  MRPT_LOG_DEBUG_STREAM("Observation validity check: layer size=" << pts->size());
  return valid;
}

}  // namespace mola
