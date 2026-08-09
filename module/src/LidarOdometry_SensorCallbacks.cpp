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
#include <utility>
#include <vector>

namespace mola
{

namespace
{
/// Age [s], relative to the newest buffered reading, beyond which an unconsumed
/// IMU observation is dropped. Only reached when no LiDAR scan is being
/// processed, since each scan consumes everything up to its own end of sweep.
constexpr double kPendingImuMaxAge = 2.0;
}  // namespace

void LidarOdometry::onNewObservation(const CObservation::ConstPtr & o)
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
    // Yes, it's an IMU obs. Handled inline, not on a worker: onIMU() only
    // buffers the reading, and doing so in the caller's thread is what keeps
    // the buffer contents (and hence which scans are ready to run) a function
    // of the input sequence alone. The costly part of using an IMU reading
    // happens later, on the LiDAR worker thread, in consumePendingImu().
    onIMU(o);
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

int LidarOdometry::pendingLidarScanCount() const
{
  const bool executing = [this]() {
    auto lck = mrpt::lockHelper(is_busy_mtx_);
    return state_.worker_tasks_lidar != 0;
  }();
  return (executing ? 1 : 0) + static_cast<int>(worker_lidar_.pendingTasks());
}

void LidarOdometry::sendLidarScanToProcessQueue(const CObservation::ConstPtr & o)
{
  // Two routes to the single worker thread, depending on whether an IMU is in
  // use at all:
  //  - No IMU: the scan is ready right away -> hand it to the worker.
  //  - IMU:    park it on the wait list until IMU data covering the whole scan
  //            time span has been received (see onIMUImpl), then submit.
  // The gate is what makes the IMU-derived state a function of the observation
  // timestamps alone: the scan consumes exactly the IMU samples within its own
  // span (see consumePendingImu), and it only runs once all of them exist, so
  // the result no longer depends on how the sensor callbacks interleave.
  // In both cases the worker pool (POLICY_DROP_OLD) applies a "drop stale, keep
  // freshest" policy, so under overload old scans are skipped and only the
  // newest ready one is processed, keeping latency low.

  // The wait list only makes sense once IMU data is actually flowing in: with
  // no IMU there is nothing to wait for, and holding scans back would stall the
  // odometry altogether. IMU readings are buffered inline by onNewObservation(),
  // so this test is decided by the input sequence, not by thread scheduling.
  if (latest_imu_time_.load() <= 0) {
    profiler_.registerUserMeasure(
      "onNewObservation.lidar_queue_length", static_cast<double>(pendingLidarScanCount()));

    submitReadyLidarScanToWorker(o, std::nullopt);
    return;
  }

  // IMU usage: park on the wait list.
  const double thisStamp = mrpt::Clock::toDouble(o->getTimeStamp());
  size_t waitListSize = 0;
  {
    auto lck = mrpt::lockHelper(worker_lidar_wait_for_imu_list_mtx_);

    // Estimate the scan period from consecutive *arrival* timestamps (EMA), so
    // it reflects the true sensor rate even while scans are being dropped:
    if (last_lidar_arrival_stamp_ > 0) {
      const double dt = thisStamp - last_lidar_arrival_stamp_;
      if (dt > 0 && dt < 1.0) {
        const double prev = lidar_scan_period_.load();
        lidar_scan_period_.store(0.9 * prev + 0.1 * dt);
      }
    }
    last_lidar_arrival_stamp_ = thisStamp;

    // The scan's IMU window ends at its estimated end of sweep:
    worker_lidar_wait_for_imu_list_.add(thisStamp, {o, thisStamp + lidar_scan_period_.load()});

    // Safety cap: if IMU data lags badly, keep the wait list from growing without
    // bound by dropping the oldest still-waiting scans (they would be the most
    // stale ones anyway):
    const auto nDropped =
      worker_lidar_wait_for_imu_list_.trim_to(params_.max_lidar_queue_before_drop);
    for (std::size_t i = 0; i < nDropped; i++) {
      addDropStats(true);
      profiler_.registerUserMeasure("onNewObservation.drop_observation", 1);
    }
    waitListSize = worker_lidar_wait_for_imu_list_.size();
  }

  // The IMU covering this or an earlier scan may already have been received,
  // so try to release now instead of waiting for the next IMU callback:
  releaseReadyLidarScansToWorker();

  const int queued = pendingLidarScanCount() + static_cast<int>(waitListSize);
  profiler_.registerUserMeasure("onNewObservation.lidar_queue_length", static_cast<double>(queued));
}

void LidarOdometry::releaseReadyLidarScansToWorker()
{
  const double imuTime = latest_imu_time_.load();
  if (imuTime <= 0) {
    return;  // no IMU received yet: nothing can be released
  }

  releaseLidarScansToWorker(imuTime);
}

std::future<void> LidarOdometry::releaseLidarScansToWorker(const double upToImuTime)
{
  // Collect every waiting scan whose whole span is already covered by received
  // IMU data, in chronological order, then submit outside the wait-list lock:
  const std::vector<ScanImuWaitList::Entry> readyScans = [&]() {
    auto lck = mrpt::lockHelper(worker_lidar_wait_for_imu_list_mtx_);
    return worker_lidar_wait_for_imu_list_.take_ready(upToImuTime);
  }();

  // On an IMU catch-up burst several scans can qualify at once, but only the
  // newest matters ("keep freshest"): drop-account the older ones directly
  // instead of submitting each only to have it superseded by the next.
  if (readyScans.empty()) {
    return {};  // an invalid future: nothing was submitted
  }

  for (std::size_t i = 0; i + 1 < readyScans.size(); i++) {
    addDropStats(true);
    profiler_.registerUserMeasure("onNewObservation.drop_observation", 1);
  }
  return submitReadyLidarScanToWorker(
    readyScans.back().obs, readyScans.back().imu_coverage_end_time);
}

std::future<void> LidarOdometry::submitReadyLidarScanToWorker(
  const CObservation::ConstPtr & o, std::optional<double> imuCoverageEndTime)
{
  // worker_lidar_ uses POLICY_DROP_OLD: with a single worker thread, enqueue()
  // itself discards any older, not-yet-started scan once one is already queued
  // behind the one currently running. The pool gives no callback for that
  // eviction, so detect it here (before enqueueing) purely for the drop-stats
  // accounting; running scans are never aborted, only queued ones can be
  // dropped this way.
  if (worker_lidar_.pendingTasks() > 0) {
    addDropStats(true);
    profiler_.registerUserMeasure("onNewObservation.drop_observation", 1);
    MRPT_LOG_THROTTLE_WARN_FMT(
      1.0, "Dropping LiDAR scan (worker busy: keeping only the freshest); dropped frames: %.02f%%",
      getDropStats() * 100.0);
  }

  return worker_lidar_.enqueue(
    &LidarOdometry::onLidar, this, o, mrpt::Clock::nowDouble(), imuCoverageEndTime);
}

void LidarOdometry::onLidar(
  const CObservation::ConstPtr & o, const double readyTimestamp,
  const std::optional<double> imuCoverageEndTime)
{
  const bool abort_running = [this]() {
    auto lck = mrpt::lockHelper(is_busy_mtx_);
    return destructor_called_;
  }();

  if (abort_running) {
    return;
  }

  {
    auto lck = mrpt::lockHelper(is_busy_mtx_);
    state_.worker_tasks_lidar = 1;
  }

  profiler_.registerUserMeasure(
    "delay_onNewObs_to_process", mrpt::Clock::nowDouble() - readyTimestamp);

  // All methods that are enqueued into a thread pool should have its own
  // top-level try-catch:
  try {
    addDropStats(false);
    processLidarScan(o, imuCoverageEndTime);
  } catch (const std::exception & e) {
    MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
    auto lckStateFlags = mrpt::lockHelper(state_flags_mtx_);
    state_.fatal_error = true;
  }

  {
    auto lck = mrpt::lockHelper(is_busy_mtx_);
    state_.worker_tasks_lidar = 0;
  }
}

void LidarOdometry::onIMU(const CObservation::ConstPtr & o)
{
  // Keep a top-level try-catch, as for the methods running on a thread pool:
  try {
    onIMUImpl(o);
  } catch (const std::exception & e) {
    MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
    auto lckStateFlags = mrpt::lockHelper(state_flags_mtx_);
    state_.fatal_error = true;
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

  // This callback only buffers: the reading is consumed later, by the LiDAR
  // worker thread, in consumePendingImu(). Consuming it here instead would make
  // the IMU-derived state seen by a scan depend on how many IMU callbacks
  // happened to finish first, which is wall-clock dependent.

  const double imuTim = mrpt::Clock::toDouble(imu->timestamp);
  {
    auto lckImu = mrpt::lockHelper(imu_state_mtx_);
    pending_imu_.add(imuTim, imu, kPendingImuMaxAge);

    // Rate stats are pure instrumentation, so they are updated on arrival: they
    // must keep reporting a live IMU even while no scan is being processed.
    state_.append_imu_stamp(imu->timestamp, *this);
  }

  // Mark how far IMU data has now been received, so a waiting scan is only
  // released once the IMU covering its whole span is available.
  // Monotonic update, to be robust against out-of-order IMU timestamps:
  {
    double prev = latest_imu_time_.load();
    while (imuTim > prev && !latest_imu_time_.compare_exchange_weak(prev, imuTim)) {
    }
  }

  // Finally, release any waiting LiDAR scan now fully covered by received IMU data:
  releaseReadyLidarScansToWorker();
}

void LidarOdometry::consumePendingImu(const double upToTime)
{
  auto lckImu = mrpt::lockHelper(imu_state_mtx_);

  for (const auto & imuPtr : pending_imu_.take_up_to(upToTime)) {
    const auto & imu = *imuPtr;

    // 1) Initial pitch/roll estimation:
    if (state_.imu_initializer.has_value()) {
      state_.imu_initializer->add(imuPtr);
    }

    // 2) Precise scan de-skewing is done via Generator, which in turns passes the IMU data to the
    //    LocalVelocityBuffer inside the ParameterSource.
    //    The LocalVelocityBuffer also needs velocity and orientation estimations, which are sent
    //    out in updatePipelineDynamicVariables()
    {
      mp2p_icp::metric_map_t dummy_map;
      mp2p_icp_filters::apply_generators(state_.obs_generators, imu, dummy_map);
    }

    // 3) Gravity estimation for ICP verticality correction:
    if (params_.imu_gravity_correction.enabled) {
      state_.gravity_estimator.add(imu, params_.imu_gravity_correction.averaging_samples);

#if defined(MOLA_LO_HAS_MAP_GRAVITY_ESTIMATOR)
      // 3b) Preintegrate for the map-frame gravity estimator. Unlike the
      //     accelerometer average above, this consumes EVERY sample and does not
      //     require the platform to be quasi-static.
      if (params_.imu_gravity_correction.map_gravity.enabled) {
        accumulateImuForMapGravity(imu);
      }
#endif
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

  // Reject readings whose specific-force deviates from gravity by >2 m/s²
  // (tighter than original threshold to avoid bias during dynamic motion):
  const double norm = std::sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  if (std::abs(norm - 9.81) > 2.0) {
    return;
  }

  // Also reject readings taken while the vehicle is rotating: a quasi-static
  // |acc|≈g does not guarantee the *direction* of the specific-force vector
  // is unbiased (e.g. during turns or fast attitude changes), so gate on
  // angular rate too, if available:
  if (imu.has(mrpt::obs::IMU_WX) && imu.has(mrpt::obs::IMU_WY) && imu.has(mrpt::obs::IMU_WZ)) {
    const double wx = imu.get(mrpt::obs::IMU_WX);
    const double wy = imu.get(mrpt::obs::IMU_WY);
    const double wz = imu.get(mrpt::obs::IMU_WZ);
    const double w_norm = std::sqrt(wx * wx + wy * wy + wz * wz);
    constexpr double max_angular_rate_for_quasi_static = mrpt::DEG2RAD(5.0);
    if (w_norm > max_angular_rate_for_quasi_static) {
      return;
    }
  }

  const double stamp = mrpt::Clock::toDouble(imu.timestamp);

  // Trim buffer to the desired averaging window:
  while (acc_buffer.size() >= max_samples) {
    acc_buffer.pop();
  }
  acc_buffer.push({stamp, acc});

  // Remember the sensor pose for vehicle-frame transform:
  imu_sensor_pose = imu.sensorPose;
  imu_sensor_pose_timestamp = stamp;
}

#if defined(MOLA_LO_HAS_MAP_GRAVITY_ESTIMATOR)
void LidarOdometry::accumulateImuForMapGravity(const mrpt::obs::CObservationIMU & imu)
{
  // Caller holds imu_state_mtx_ (state_.map_gravity).
  using namespace mrpt::obs;

  if (
    !imu.has(IMU_X_ACC) || !imu.has(IMU_Y_ACC) || !imu.has(IMU_Z_ACC) || !imu.has(IMU_WX) ||
    !imu.has(IMU_WY) || !imu.has(IMU_WZ)) {
    return;  // preintegration needs both accelerometer and gyroscope
  }

  const double stamp = mrpt::Clock::toDouble(imu.timestamp);
  auto & mg = state_.map_gravity;

  // dt from the previous sample. The first one only seeds the clock.
  if (!mg.last_imu_time.has_value()) {
    mg.last_imu_time = stamp;
    return;
  }
  const double dt = stamp - *mg.last_imu_time;
  mg.last_imu_time = stamp;
  if (dt <= 0 || dt > 1.0) {
    return;  // out-of-order or a gap: skip rather than corrupt the integral
  }

  // Rotate into the VEHICLE frame. The lever-arm contribution to the
  // accelerometer is not compensated: it is a centripetal/tangential term that
  // averages out over a window and is small for the offsets seen here.
  const auto R = imu.sensorPose.getRotationMatrix();
  const mrpt::math::TVector3D a_s = {imu.get(IMU_X_ACC), imu.get(IMU_Y_ACC), imu.get(IMU_Z_ACC)};
  const mrpt::math::TVector3D w_s = {imu.get(IMU_WX), imu.get(IMU_WY), imu.get(IMU_WZ)};

  const auto rot = [&R](const mrpt::math::TVector3D & v) {
    return mrpt::math::TVector3D(
      R(0, 0) * v.x + R(0, 1) * v.y + R(0, 2) * v.z, R(1, 0) * v.x + R(1, 1) * v.y + R(1, 2) * v.z,
      R(2, 0) * v.x + R(2, 1) * v.y + R(2, 2) * v.z);
  };

  mg.preintegrator.integrate_measurement(rot(a_s), rot(w_s), dt);
}
#endif

std::optional<std::pair<double, double>>
LidarOdometry::MethodState::GravityEstimator::estimatedPitchRoll(
  uint32_t required_samples, double max_age_seconds) const
{
  if (acc_buffer.size() < required_samples) {
    return std::nullopt;
  }

  // Determine the newest timestamp in the buffer for age filtering:
  const double newest_stamp = acc_buffer.peek(acc_buffer.size() - 1).timestamp;
  const bool use_age_filter = max_age_seconds > 0;

  // Average the accelerometer readings (in sensor frame),
  // considering only samples within the age limit:
  double sx = 0, sy = 0, sz = 0;
  std::size_t count = 0;
  const auto n = acc_buffer.size();
  for (std::size_t i = 0; i < n; i++) {
    const auto & entry = acc_buffer.peek(i);
    if (use_age_filter && (newest_stamp - entry.timestamp) > max_age_seconds) {
      continue;  // too old
    }
    sx += entry.acc[0];
    sy += entry.acc[1];
    sz += entry.acc[2];
    ++count;
  }

  if (count < required_samples) {
    return std::nullopt;
  }

  const double inv_n = 1.0 / static_cast<double>(count);
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

std::optional<double> LidarOdometry::MethodState::GravityEstimator::directionDispersionSigma(
  double max_age_seconds) const
{
  const auto n = acc_buffer.size();
  if (n < 2) {
    return std::nullopt;
  }

  const double newest_stamp = acc_buffer.peek(n - 1).timestamp;
  const bool use_age_filter = max_age_seconds > 0;

  const auto isUsable = [&](std::size_t i, double & nrm) {
    const auto & e = acc_buffer.peek(i);
    if (use_age_filter && (newest_stamp - e.timestamp) > max_age_seconds) {
      return false;
    }
    nrm = std::sqrt(e.acc[0] * e.acc[0] + e.acc[1] * e.acc[1] + e.acc[2] * e.acc[2]);
    return nrm >= 1e-6;
  };

  // Mean direction. The sensor frame suffices: a fixed rotation to the vehicle
  // frame does not change the angles between directions.
  double mx = 0;
  double my = 0;
  double mz = 0;
  std::size_t count = 0;
  for (std::size_t i = 0; i < n; i++) {
    double nrm = 0;
    if (!isUsable(i, nrm)) {
      continue;
    }
    const auto & e = acc_buffer.peek(i);
    mx += e.acc[0] / nrm;
    my += e.acc[1] / nrm;
    mz += e.acc[2] / nrm;
    ++count;
  }
  if (count < 2) {
    return std::nullopt;
  }

  const double mn = std::sqrt(mx * mx + my * my + mz * mz);
  if (mn < 1e-6) {
    return std::nullopt;
  }
  mx /= mn;
  my /= mn;
  mz /= mn;

  // RMS angle of each sample's direction about the mean direction:
  double sumSqAng = 0;
  for (std::size_t i = 0; i < n; i++) {
    double nrm = 0;
    if (!isUsable(i, nrm)) {
      continue;
    }
    const auto & e = acc_buffer.peek(i);
    const double dot = (e.acc[0] * mx + e.acc[1] * my + e.acc[2] * mz) / nrm;
    const double ang = std::acos(std::min(1.0, std::max(-1.0, dot)));
    sumSqAng += ang * ang;
  }

  return std::sqrt(sumSqAng / static_cast<double>(count));
}

}  // namespace mola
