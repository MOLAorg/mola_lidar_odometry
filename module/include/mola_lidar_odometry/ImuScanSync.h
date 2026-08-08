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
 * @file   ImuScanSync.h
 * @brief  Timestamp-based synchronization between the LiDAR and IMU inputs
 * @author Jose Luis Blanco Claraco
 */
#pragma once

#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservationIMU.h>

#include <map>
#include <vector>

namespace mola
{
/** Holds IMU observations until the LiDAR scan that needs them is processed.
 *
 *  Which samples a scan gets is decided by their timestamps alone, so it does
 *  not matter on which thread, or in which order, the samples were delivered.
 *
 * \ingroup mola_lidar_odometry_grp
 */
class PendingImuBuffer
{
public:
  PendingImuBuffer() = default;

  /** Stores one observation, then drops everything older than `maxAge` seconds
   *  relative to the newest sample held.
   *  A multimap, so that two readings sharing a timestamp are both kept: which
   *  of them is "the" reading for that instant is undecidable, and dropping one
   *  would silently discard data the previous code did use. */
  void add(double timestamp, const mrpt::obs::CObservationIMU::ConstPtr & imu, double maxAge)
  {
    samples_.emplace(timestamp, imu);

    const double oldestToKeep = samples_.rbegin()->first - maxAge;
    samples_.erase(samples_.begin(), samples_.lower_bound(oldestToKeep));
  }

  /** Removes and returns every sample with a timestamp not newer than
   *  `upToTime`, in timestamp order. */
  std::vector<mrpt::obs::CObservationIMU::ConstPtr> take_up_to(double upToTime)
  {
    const auto itEnd = samples_.upper_bound(upToTime);

    std::vector<mrpt::obs::CObservationIMU::ConstPtr> ret;
    ret.reserve(static_cast<std::size_t>(std::distance(samples_.begin(), itEnd)));
    for (auto it = samples_.begin(); it != itEnd; ++it) {
      ret.push_back(it->second);
    }
    samples_.erase(samples_.begin(), itEnd);
    return ret;
  }

  void clear() { samples_.clear(); }

  std::size_t size() const { return samples_.size(); }

private:
  std::multimap<double /*timestamp*/, mrpt::obs::CObservationIMU::ConstPtr> samples_;
};

/** Holds LiDAR scans back until the IMU covering their whole time span has been
 *  received, so a scan always runs against a fixed set of IMU samples.
 *
 *  Pure timestamp bookkeeping: no threads, no locks, no clocks.
 *
 * \ingroup mola_lidar_odometry_grp
 */
class ScanImuWaitList
{
public:
  ScanImuWaitList() = default;

  struct Entry
  {
    mrpt::obs::CObservation::ConstPtr obs;
    /// Sensor time [s] up to which this scan needs IMU data, i.e. its estimated
    /// end of sweep. Frozen when the scan enters the list, so release and
    /// consumption always agree on the same window.
    double imu_coverage_end_time = 0;
  };

  void add(double scanTimestamp, const Entry & e) { list_.emplace(scanTimestamp, e); }

  /** Removes and returns, in timestamp order, every scan whose IMU window is
   *  already covered by data received up to `latestImuTime`. */
  std::vector<Entry> take_ready(double latestImuTime)
  {
    std::vector<Entry> ret;
    for (auto it = list_.begin(); it != list_.end();) {
      if (latestImuTime < it->second.imu_coverage_end_time) {
        // The list is sorted by timestamp: no later scan can qualify either.
        break;
      }
      ret.push_back(it->second);
      it = list_.erase(it);
    }
    return ret;
  }

  /** Drops the oldest scans until at most `maxSize` remain.
   *  \return How many were dropped. */
  std::size_t trim_to(std::size_t maxSize)
  {
    std::size_t dropped = 0;
    while (list_.size() > maxSize) {
      list_.erase(list_.begin());
      dropped++;
    }
    return dropped;
  }

  std::size_t size() const { return list_.size(); }

private:
  std::multimap<double /*scan timestamp*/, Entry> list_;
};

}  // namespace mola
