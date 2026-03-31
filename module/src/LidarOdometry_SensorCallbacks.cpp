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

#if MOLA_VERSION_CHECK(2, 1, 0)
void LidarOdometry::onNewObservation(const CObservation::ConstPtr & o)
#else
void LidarOdometry::onNewObservation(const CObservation::Ptr & o)
#endif
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
    auto fut = worker_others_.enqueue(&LidarOdometry::onIMU, this, o);
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
    auto fut = worker_others_.enqueue(&LidarOdometry::onGPS, this, o);
    (void)fut;
  }

  // Is it a LIDAR obs?
  for (const auto & re : params_.lidar_sensor_labels) {
    if (!std::regex_match(o->sensorLabel, re)) {
      continue;
    }

    // Yes, it's a LIDAR obs:
    sendLidarScanToProcessQueue(o);

    break;  // do not keep processing the list
  }

  MRPT_TRY_END
}

void LidarOdometry::sendLidarScanToProcessQueue(const CObservation::ConstPtr & o)
{
  // If we don't rely on IMU, directly enqueue the task. Otherwise, put it on the wait list until
  // IMU data for the required timestamps has arrived so we can do de-skew:

  auto queued = [this]() {
    auto lck = mrpt::lockHelper(is_busy_mtx_);
    return state_.worker_tasks_lidar + worker_lidar_.pendingTasks();
  }() + [this]() {
    auto lck = mrpt::lockHelper(worker_lidar_wait_for_imu_list_mtx_);
    return static_cast<int>(worker_lidar_wait_for_imu_list_.size());
  }();

  profiler_.registerUserMeasure("onNewObservation.lidar_queue_length", static_cast<double>(queued));
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

  if (!isPipelineUsingIMU()) {
    {
      auto lck = mrpt::lockHelper(is_busy_mtx_);
      state_.worker_tasks_lidar++;
    }

    auto fut = worker_lidar_.enqueue(&LidarOdometry::onLidar, this, o);
    (void)fut;
  } else {
    // IMU usage:
    auto lck = mrpt::lockHelper(worker_lidar_wait_for_imu_list_mtx_);
    worker_lidar_wait_for_imu_list_.emplace(mrpt::Clock::toDouble(o->getTimeStamp()), o);
  }
}

void LidarOdometry::onLidar(const CObservation::ConstPtr & o)
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

void LidarOdometry::onIMU(const CObservation::ConstPtr & o)
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

void LidarOdometry::onIMUImpl(const CObservation::ConstPtr & o)
{
  ASSERT_(o);

  const ProfilerEntry tleg(profiler_, "onIMU");

  auto imu = std::dynamic_pointer_cast<const mrpt::obs::CObservationIMU>(o);
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
  // 2) Improved scan de-skewing.

  // 1) Initial pitch/roll estimation:
  {
    auto lckState = mrpt::lockHelper(state_mtx_);
    if (state_.imu_initializer.has_value()) {
      state_.imu_initializer->add(imu);
    }

    // and for rate stats:
    state_.append_imu_stamp(imu->timestamp, *this);
  }

  // 2) Precise scan de-skewing is done via Generator, which in turns passes the IMU data to the
  //    LocalVelocityBuffer inside the ParameterSource.
  //    The LocalVelocityBuffer also needs velocity and orientation estimations, which are sent out
  //    in updatePipelineDynamicVariables()
  {
    auto lckState = mrpt::lockHelper(state_mtx_);
    mp2p_icp::metric_map_t dummy_map;
    mp2p_icp_filters::apply_generators(state_.obs_generators, *imu, dummy_map);
  }

  // 3) Gravity estimation for ICP verticality correction:
  if (params_.imu_gravity_correction.enabled) {
    auto lckState2 = mrpt::lockHelper(state_mtx_);
    state_.gravity_estimator.add(*imu, params_.imu_gravity_correction.averaging_samples);
  }

  // Finally, schedule to run those LiDAR scans that were waiting for IMU data:
  if (isPipelineUsingIMU()) {
    const auto imuTim = mrpt::Clock::toDouble(imu->timestamp);

    const auto rate_lidar_hz = [&] {
      auto lckState = mrpt::lockHelper(state_mtx_);
      auto [rate_lidar, rate_imu_, rate_gnss_] = state_.get_sensor_rates();
      return rate_lidar;
    }();

    const double lidar_scan_period = rate_lidar_hz > 0 ? 1.0 / rate_lidar_hz : 0.1;

    auto lck = mrpt::lockHelper(worker_lidar_wait_for_imu_list_mtx_);

    for (auto it = worker_lidar_wait_for_imu_list_.begin();
         it != worker_lidar_wait_for_imu_list_.end();) {
      const auto [lidarTim, lidarObs] = *it;

      const double lidar2imu_dt = imuTim - lidarTim;
      if (lidar2imu_dt > lidar_scan_period) {
        // Enqueue this one:
        {
          auto lck2 = mrpt::lockHelper(is_busy_mtx_);
          state_.worker_tasks_lidar++;
        }
        auto fut = worker_lidar_.enqueue(&LidarOdometry::onLidar, this, lidarObs);
        (void)fut;

        // and remove from the list:
        it = worker_lidar_wait_for_imu_list_.erase(it);
      } else {
        ++it;
      }
    }
  }
}

void LidarOdometry::onGPS(const CObservation::ConstPtr & o)
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

void LidarOdometry::onGPSImpl(const CObservation::ConstPtr & o)
{
  ASSERT_(o);

  const ProfilerEntry tleg(profiler_, "onGPS");

  auto gps = std::dynamic_pointer_cast<const mrpt::obs::CObservationGPS>(o);
  ASSERTMSG_(
    gps, mrpt::format(
           "GPS observation with label '%s' does not have the expected "
           "type 'mrpt::obs::CObservationGPS', it is '%s' instead",
           o->sensorLabel.c_str(), o->GetRuntimeClass()->className));

  MRPT_LOG_DEBUG_FMT("GNSS observation received, t=%.03f", mrpt::Clock::toDouble(gps->timestamp));

  // ensure covariance is valid:
  if (gps->covariance_enu) {
    const auto minCov = gps->covariance_enu->minimumDiagonal();
    if (minCov < 0 || std::isnan(minCov) || std::isinf(minCov)) {
      MRPT_LOG_THROTTLE_WARN_STREAM(
        5.0, "Discarding GPS observation with invalid covariance matrix");
      return;
    }
  }

  // for rate stats:
  state_.append_gnss_stamp(gps->timestamp, *this);

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

namespace
{
/// Computes rate as (N-1)/(t_last - t_first).
/// Returns 0.0 if the buffer has fewer than 2 stamps, or if the most recent
/// stamp is older than `staleness_timeout` seconds relative to `ref_time`.
double rate_from_stamps_buffer(
  const mrpt::containers::circular_buffer<double> & stamps, double ref_time = 0,
  double staleness_timeout = 5.0)
{
  if (stamps.size() < 2) return .0;

  const double t_first = stamps.peek(0);
  const double t_last = stamps.peek(stamps.size() - 1);

  // Staleness check: if ref_time is provided and the newest stamp
  // is too old, report 0 (sensor stopped):
  if (ref_time > 0 && (ref_time - t_last) > staleness_timeout) return .0;

  const double dt = t_last - t_first;
  if (dt <= 0) return .0;

  return static_cast<double>(stamps.size() - 1) / dt;
}

}  // namespace

std::tuple<double, double, double> LidarOdometry::MethodState::get_sensor_rates()
{
  // Use the latest observation timestamp as reference for staleness detection.
  // This works for both live and offline (rawlog/rosbag) data sources.
  const double ref_time = last_obs_timestamp ? mrpt::Clock::toDouble(*last_obs_timestamp) : 0.0;

  // Aggregate across all lidar sensors:
  double lidar_rate = 0;
  for (auto & [label, stamps] : recent_lidar_stamps)
    lidar_rate += rate_from_stamps_buffer(stamps, ref_time);

  const double imu_rate = rate_from_stamps_buffer(recent_imu_stamps, ref_time);
  const double gnss_rate = rate_from_stamps_buffer(recent_gnss_stamps, ref_time);

  return {lidar_rate, imu_rate, gnss_rate};
}

namespace
{
/// Appends a timestamp to a circular buffer, with backwards-time rejection.
/// Returns true if the stamp was accepted, false if rejected.
bool append_stamp_to_buffer(
  mrpt::containers::circular_buffer<double> & stamps, double t, const char * sensor_name,
  const mrpt::system::COutputLogger & logger)
{
  // Reject backwards timestamps instead of just warning:
  if (stamps.size() > 0) {
    const double t_prev = stamps.peek(stamps.size() - 1);
    if (t < t_prev) {
      logger.logFmt(
        mrpt::system::LVL_WARN,
        "%s timestamps went backwards in time: t[k-1]=%f, t[k]=%f (discarding for rate calc)",
        sensor_name, t_prev, t);
      return false;
    }
  }
  if (stamps.available() == 0) stamps.pop();
  stamps.push(t);
  return true;
}
}  // namespace

void LidarOdometry::MethodState::append_lidar_stamp(
  const std::string & sensorLabel, const mrpt::Clock::time_point & stamp,
  const mrpt::system::COutputLogger & logger)
{
  constexpr std::size_t LIDAR_STAMPS_QUEUE_LENGTH = 50;

  auto [it_stamps, is_new] =
    recent_lidar_stamps.try_emplace(sensorLabel, LIDAR_STAMPS_QUEUE_LENGTH);

  append_stamp_to_buffer(it_stamps->second, mrpt::Clock::toDouble(stamp), "LiDAR", logger);
}

void LidarOdometry::MethodState::append_imu_stamp(
  const mrpt::Clock::time_point & stamp, const mrpt::system::COutputLogger & logger)
{
  append_stamp_to_buffer(recent_imu_stamps, mrpt::Clock::toDouble(stamp), "IMU", logger);
}

void LidarOdometry::MethodState::append_gnss_stamp(
  const mrpt::Clock::time_point & stamp, const mrpt::system::COutputLogger & logger)
{
  append_stamp_to_buffer(recent_gnss_stamps, mrpt::Clock::toDouble(stamp), "GNSS", logger);
}

void LidarOdometry::MethodState::GravityEstimator::add(
  const mrpt::obs::CObservationIMU & imu, uint32_t max_samples)
{
  if (
    !imu.has(mrpt::obs::IMU_X_ACC) || !imu.has(mrpt::obs::IMU_Y_ACC) ||
    !imu.has(mrpt::obs::IMU_Z_ACC)) {
    return;
  }

  const std::array<double, 3> acc = {
    imu.get(mrpt::obs::IMU_X_ACC), imu.get(mrpt::obs::IMU_Y_ACC), imu.get(mrpt::obs::IMU_Z_ACC)};

  // Basic sanity: reject readings that are clearly not gravity-like
  const double norm = std::sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  if (std::abs(norm - 9.8) > 3.0 && std::abs(norm - 1.0) > 0.5) {
    return;  // not a plausible gravity reading
  }

  // Trim buffer to the desired averaging window:
  while (acc_buffer.size() >= max_samples) {
    acc_buffer.pop();
  }
  acc_buffer.push(acc);

  // Remember the sensor pose for vehicle-frame transform:
  imu_sensor_pose = imu.sensorPose;
}

std::optional<std::pair<double, double>>
LidarOdometry::MethodState::GravityEstimator::estimatedPitchRoll(uint32_t required_samples) const
{
  if (acc_buffer.size() < required_samples) {
    return std::nullopt;
  }

  // Average the accelerometer readings (in sensor frame):
  double sx = 0, sy = 0, sz = 0;
  const auto n = acc_buffer.size();
  for (std::size_t i = 0; i < n; i++) {
    const auto & a = acc_buffer.peek(i);
    sx += a[0];
    sy += a[1];
    sz += a[2];
  }
  const double inv_n = 1.0 / static_cast<double>(n);
  const double ax = sx * inv_n;
  const double ay = sy * inv_n;
  const double az = sz * inv_n;

  // Transform averaged gravity from IMU sensor frame to vehicle frame:
  const auto rotMat = imu_sensor_pose.getRotationMatrix();

  const double gx_veh = rotMat(0, 0) * ax + rotMat(0, 1) * ay + rotMat(0, 2) * az;
  const double gy_veh = rotMat(1, 0) * ax + rotMat(1, 1) * ay + rotMat(1, 2) * az;
  const double gz_veh = rotMat(2, 0) * ax + rotMat(2, 1) * ay + rotMat(2, 2) * az;

  // From gravity direction in vehicle frame, estimate pitch and roll.
  // Convention: if vehicle is level, gravity = [0, 0, +g] (z-up).
  //   pitch = rotation about Y that tilts gravity into X
  //   roll  = rotation about X that tilts gravity into Y
  const double g_norm = std::sqrt(gx_veh * gx_veh + gy_veh * gy_veh + gz_veh * gz_veh);
  if (g_norm < 1e-3) {
    return std::nullopt;
  }

  const double nx = gx_veh / g_norm;
  const double ny = gy_veh / g_norm;
  const double nz = gz_veh / g_norm;

  const double pitch = std::asin(-nx);
  const double roll = std::atan2(ny, nz);

  return std::make_pair(pitch, roll);
}

}  // namespace mola
