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
 * @file   PointTimeOutlierFilter.h
 * @brief  Drops LiDAR points whose per-point time is far outside the usual range
 * @author Jose Luis Blanco Claraco
 */
#pragma once

#include <mrpt/maps/CPointsMap.h>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

namespace mola
{
/** Drops the points of a LiDAR scan whose per-point time (the "t" field) is far
 *  outside the range that the previous scans of the same sensor had.
 *
 *  Per-point times come from the sensor clock. A clock that jumps (e.g. a
 *  timing input with spurious edges) gives some columns of a scan times that
 *  are seconds away from the rest, and the scan reference time (e.g. the
 *  middle of its time range) then moves by that much.
 *
 *  The usual range is the median of the minimum and of the maximum time of the
 *  last `window_length` scans, as given (before this filter), so a minority of
 *  bad scans in the window does not move it. It is widened by a margin: the
 *  largest of `margin_ratio` of its width, `min_margin`, and the sensor period
 *  (median interval between consecutive scan stamps), so that sensors that only
 *  report the directions with returns (a narrow sector one moment, the whole
 *  sweep the next) never lose points of a normal sweep. A scan within that
 *  band is accepted as is; otherwise, the points out of it are removed, with all
 *  their fields. Nothing is removed until the window holds `min_window_length`
 *  scans.
 *
 *  Cost: one pass over the time field per scan; a second one, compacting all
 *  fields in place, only for scans with outliers.
 *
 *  One instance per sensor: different sensors have different time ranges.
 *
 * \ingroup mola_lidar_odometry_grp
 */
class PointTimeOutlierFilter
{
public:
  struct Parameters
  {
    bool enabled = true;

    /** Number of recent scans whose time ranges define the usual one. */
    uint32_t window_length = 50;

    /** Nothing is removed until the window holds this many scans. */
    uint32_t min_window_length = 10;

    /** Tolerance beyond the usual [min, max], as a fraction of its width. */
    double margin_ratio = 0.5;

    /** Lower bound of that tolerance [s] (the sensor period is another one). */
    double min_margin = 0.01;

    /** Throws if the values are inconsistent (e.g. a window shorter than its
     *  warm-up, which would disable the filter, or negative margins). */
    void validate() const;
  };

  struct Result
  {
    /** Points with a time field, before the filter. */
    std::size_t points = 0;

    /** Points removed. */
    std::size_t rejected = 0;

    /** Time range of the scan before the filter [s]. */
    double scan_min = 0;
    double scan_max = 0;

    /** Accepted range [lo, hi], if the scan was checked (warmed-up window). */
    std::optional<std::pair<double, double>> accepted;
  };

  PointTimeOutlierFilter() = default;
  explicit PointTimeOutlierFilter(const Parameters & p);

  /** Checks one scan, given as the point clouds (layers) of one observation
   *  taken at `stamp` [s]. Clouds without a "t" field are ignored. The scan
   *  time range and stamp are added to the window afterwards. */
  Result process(const std::vector<mrpt::maps::CPointsMap *> & clouds, double stamp);

  /** As process(), but against the current window only, which is left
   *  unchanged (e.g. to re-process old scans with what was learned live). */
  [[nodiscard]] Result check(const std::vector<mrpt::maps::CPointsMap *> & clouds) const;

  /** The usual time range [min, max] of the window, once it is warmed up. */
  [[nodiscard]] std::optional<std::pair<double, double>> usual_range() const;

  /** The range a scan must be within, once the window is warmed up. */
  [[nodiscard]] std::optional<std::pair<double, double>> accepted_range() const;

  [[nodiscard]] const Parameters & parameters() const { return params_; }

  void reset()
  {
    window_.clear();
    periods_.clear();
    last_stamp_.reset();
  }

  /** Removes, in place and keeping the order, the points of `pc` whose
   *  `keep` entry is zero, from all its fields. \return removed points. */
  static std::size_t remove_points(mrpt::maps::CPointsMap & pc, const std::vector<uint8_t> & keep);

private:
  Parameters params_;

  /** (min, max) time of the most recent scans, oldest first. */
  std::deque<std::pair<float, float>> window_;

  /** Intervals between consecutive scan stamps [s], oldest first. */
  std::deque<double> periods_;
  std::optional<double> last_stamp_;

  [[nodiscard]] static Result apply(
    const std::vector<mrpt::maps::CPointsMap *> & clouds,
    const std::optional<std::pair<double, double>> & accepted);

  /** Removes the points out of [lo, hi] from every cloud. \return removed
   *  points. */
  static std::size_t remove_outside(
    const std::vector<mrpt::maps::CPointsMap *> & clouds, double lo, double hi);
};

}  // namespace mola
