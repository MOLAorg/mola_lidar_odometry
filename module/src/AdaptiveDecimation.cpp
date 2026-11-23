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
 * @file   AdaptiveDecimation.cpp
 * @brief  Motion-driven LiDAR scan decimation
 * @author Jose Luis Blanco Claraco
 */

#include <mola_lidar_odometry/AdaptiveDecimation.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/poses/Lie/SO.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace mola
{
namespace
{
// A longer gap between gyroscope readings is not integrated: it cannot be
// trusted.
constexpr double MAX_GYRO_READING_GAP = 0.1;  // [s]

// Registrations the quality median is taken over, and the fewest to use it:
constexpr std::size_t QUALITY_WINDOW = 50;
constexpr std::size_t QUALITY_MIN_SAMPLES = 10;
}  // namespace

void AdaptiveDecimation::addGyro(double t, const mrpt::math::TVector3D & w)
{
  if (lastGyroStamp_ && t <= *lastGyroStamp_) {
    return;
  }
  if (lastGyroStamp_ && t - *lastGyroStamp_ > MAX_GYRO_READING_GAP) {
    sweepGap_ = true;
    intervalCovered_ = false;
  }
  if (lastGyroStamp_ && t - *lastGyroStamp_ <= MAX_GYRO_READING_GAP) {
    const double dt = t - *lastGyroStamp_;
    mrpt::math::CVectorFixedDouble<3> v;
    v[0] = w.x * dt;
    v[1] = w.y * dt;
    v[2] = w.z * dt;
    const auto dR = mrpt::poses::Lie::SO<3>::exp(v);
    R_ = R_ * dR;
    R_sweep_ = R_sweep_ * dR;
    gyroSeen_ = true;
    sweepCovered_ = true;
  }
  lastGyroStamp_ = t;
}

void AdaptiveDecimation::addRegistrationQuality(double t, double quality, const Budgets & b)
{
  if (b.min_quality_ratio <= 0) {
    return;
  }
  if (recentQualities_.size() >= QUALITY_MIN_SAMPLES) {
    std::vector<double> v(recentQualities_.begin(), recentQualities_.end());
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    if (quality < b.min_quality_ratio * v[v.size() / 2]) {
      lowQualityUntil_ = t + b.quality_hold_time;
    }
  }
  recentQualities_.push_back(quality);
  if (recentQualities_.size() > QUALITY_WINDOW) {
    recentQualities_.pop_front();
  }
}

double AdaptiveDecimation::gyroRotation() const { return mrpt::poses::Lie::SO<3>::log(R_).norm(); }

const char * AdaptiveDecimation::decide(const Scan & scan, const Budgets & b)
{
  const double sweepRotation = mrpt::poses::Lie::SO<3>::log(R_sweep_).norm();
  // The gyroscope covered the latest sweep, without a gap, up to its end, and
  // the whole interval since the last processed scan:
  const bool sweepOk =
    sweepCovered_ && !sweepGap_ &&
    (scan.sweep_end <= 0 || scan.sweep_end - *lastGyroStamp_ <= MAX_GYRO_READING_GAP);
  const bool gyroOk = sweepOk && intervalCovered_;
  R_sweep_ = mrpt::math::CMatrixDouble33::Identity();
  sweepCovered_ = false;
  sweepGap_ = false;

  const char * reason = [&]() -> const char * {
    if (scan.gap <= 0 || scan.period <= 0) {
      return "start";
    }
    if (!scan.last_registration_good) {
      return "bad_registration";
    }
    if (lowQualityUntil_ && scan.stamp <= *lowQualityUntil_) {
      return "quality";
    }
    if (skipped_ >= b.max_skipped_scans) {
      return "max_skipped_scans";
    }
    // Budgets are checked where the next processed scan would be:
    const double nextGap = scan.gap + scan.period;
    if (nextGap > b.max_time_gap) {
      return "time_gap";
    }
    if (b.require_gyro && !gyroOk) {
      return "no_gyro";
    }
    if (!scan.twist) {
      return "no_motion_model";
    }
    const auto & tw = *scan.twist;
    const double speed = std::sqrt(tw.vx * tw.vx + tw.vy * tw.vy + tw.vz * tw.vz);
    if (speed * nextGap > b.max_distance) {
      return "distance";
    }
    // The gyroscope rotation so far plus that of the latest sweep (so the
    // start of a turn counts at once), or the estimator angular velocity:
    const double nextRotation =
      gyroOk ? gyroRotation() + sweepRotation
             : std::sqrt(tw.wx * tw.wx + tw.wy * tw.wy + tw.wz * tw.wz) * nextGap;
    if (nextRotation > mrpt::DEG2RAD(b.max_rotation_deg)) {
      return "rotation";
    }
    return nullptr;
  }();

  if (!reason) {
    skipped_++;
    return nullptr;
  }
  skipped_ = 0;
  R_ = mrpt::math::CMatrixDouble33::Identity();
  gyroSeen_ = false;
  intervalCovered_ = true;
  return reason;
}

}  // namespace mola
