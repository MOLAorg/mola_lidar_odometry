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
 * @file   AdaptiveDecimation.h
 * @brief  Motion-driven LiDAR scan decimation
 * @author Jose Luis Blanco Claraco
 */
#pragma once

#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TTwist3D.h>

#include <cstdint>
#include <deque>
#include <optional>

namespace mola
{
/** Decides whether a LiDAR scan can be skipped, from the motion since the last
 *  processed scan. A scan is skipped only if, by the next scan, every budget
 *  would still hold: rotation (integrated gyroscope, or the estimator angular
 *  velocity if allowed without gyroscope readings), distance (estimator linear
 *  speed times the gap), time gap and number of consecutive skipped scans.
 *  Every scan is also processed after a bad registration, and for a while
 *  after one whose quality fell below its recent median: in scenes where
 *  registration gets marginal, the larger steps of skipped scans are what
 *  makes it fail.
 *
 *  Uses sensor time only, so its decisions are deterministic. Not thread safe.
 */
class AdaptiveDecimation
{
public:
  struct Budgets
  {
    /// Largest number of consecutive skipped scans (3: at most 1 of 4).
    uint32_t max_skipped_scans = 3;
    /// [deg] Rotation budget between processed scans.
    double max_rotation_deg = 8.0;
    /// [m] Distance budget between processed scans.
    double max_distance = 1.5;
    /// [s] Time budget between processed scans.
    double max_time_gap = 0.45;
    /// Process every scan whose sweep the gyroscope did not cover without a
    /// gap. Otherwise, the rotation then comes from the estimator twist, which
    /// is one processed scan old and misses the start of a fast turn.
    bool require_gyro = true;
    /// Process every scan while a registration quality is below this ratio
    /// of the recent median quality, and quality_hold_time after it (0: off).
    double min_quality_ratio = 0.97;
    /// [s] See min_quality_ratio.
    double quality_hold_time = 2.0;
  };

  struct Scan
  {
    /// [s] Sensor time of this scan.
    double stamp = 0;
    /// [s] Since the last processed scan of this sensor (<= 0: none yet).
    double gap = 0;
    /// [s] Sensor period, e.g. from the last two scans (<= 0: unknown).
    double period = 0;
    /// [s] End of the time span the gyroscope readings given so far should
    /// cover for this scan (<= 0: unknown, not checked).
    double sweep_end = 0;
    /// False if the last registration was rejected or was a prediction.
    bool last_registration_good = true;
    /// The state estimator twist, if any.
    std::optional<mrpt::math::TTwist3D> twist;
  };

  /// Integrates a gyroscope reading [rad/s] at sensor time `t` [s]. The
  /// rotation angle does not depend on the frame the readings are in.
  void addGyro(double t, const mrpt::math::TVector3D & w);

  /** Returns nullptr if the scan can be skipped, or the reason why it has to
   *  be processed ("start", "bad_registration", "max_skipped_scans",
   *  "quality", "time_gap", "no_gyro", "no_motion_model", "distance" or
   *  "rotation"). In
   *  the latter case, the accumulated motion restarts. */
  const char * decide(const Scan & scan, const Budgets & budgets);

  /// Records the quality of a registration done at sensor time `t` [s].
  void addRegistrationQuality(double t, double quality, const Budgets & budgets);

  /// [rad] Rotation since the last processed scan, from the gyroscope.
  double gyroRotation() const;

private:
  /// Gyroscope rotation since the last processed scan, and since the last
  /// decision (the latest sweep):
  mrpt::math::CMatrixDouble33 R_ = mrpt::math::CMatrixDouble33::Identity();
  mrpt::math::CMatrixDouble33 R_sweep_ = mrpt::math::CMatrixDouble33::Identity();
  std::optional<double> lastGyroStamp_;
  bool gyroSeen_ = false;
  bool sweepCovered_ = false;  ///< readings integrated in the latest sweep
  bool sweepGap_ = false;      ///< a reading gap in the latest sweep
  /// R_ covers the whole interval since the last processed scan (no gap):
  bool intervalCovered_ = true;
  uint32_t skipped_ = 0;
  std::deque<double> recentQualities_;
  std::optional<double> lowQualityUntil_;  ///< [s]
};

}  // namespace mola
