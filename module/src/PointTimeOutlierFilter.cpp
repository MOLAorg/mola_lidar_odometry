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
 * @file   PointTimeOutlierFilter.cpp
 * @brief  Drops LiDAR points whose per-point time is far outside the usual range
 * @author Jose Luis Blanco Claraco
 */

#include <mola_lidar_odometry/PointTimeOutlierFilter.h>
#include <mrpt/core/exceptions.h>

#include <algorithm>
#include <limits>
#include <string>

namespace mola
{
namespace
{
template <typename T>
T median_of(std::vector<T> & v)
{
  const auto mid = v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2);
  std::nth_element(v.begin(), mid, v.end());
  return *mid;
}

template <typename BUF>
void compact(BUF * buf, const std::vector<uint8_t> & keep)
{
  if (buf == nullptr || buf->size() != keep.size()) {
    return;
  }
  std::size_t j = 0;
  for (std::size_t i = 0; i < keep.size(); i++) {
    if (keep[i] != 0) {
      (*buf)[j++] = (*buf)[i];
    }
  }
}

// Intervals between stamps longer than this are gaps, not the sensor period:
constexpr double MAX_PERIOD = 2.0;  // [s]
}  // namespace

void PointTimeOutlierFilter::Parameters::validate() const
{
  ASSERTMSG_(window_length >= 1, "point_time_outliers: window_length must be >= 1");
  ASSERTMSG_(
    min_window_length <= window_length,
    "point_time_outliers: min_window_length must not exceed window_length (the filter would "
    "never start)");
  ASSERTMSG_(margin_ratio >= 0, "point_time_outliers: margin_ratio must be >= 0");
  ASSERTMSG_(min_margin >= 0, "point_time_outliers: min_margin must be >= 0");
}

PointTimeOutlierFilter::PointTimeOutlierFilter(const Parameters & p) : params_(p)
{
  params_.validate();
}

std::optional<std::pair<double, double>> PointTimeOutlierFilter::usual_range() const
{
  if (window_.empty() || window_.size() < std::max<std::size_t>(1, params_.min_window_length)) {
    return {};
  }
  std::vector<float> mins;
  std::vector<float> maxs;
  mins.reserve(window_.size());
  maxs.reserve(window_.size());
  for (const auto & [lo, hi] : window_) {
    mins.push_back(lo);
    maxs.push_back(hi);
  }
  return std::make_pair(median_of(mins), median_of(maxs));
}

std::optional<std::pair<double, double>> PointTimeOutlierFilter::accepted_range() const
{
  const auto usual = usual_range();
  if (!usual) {
    return {};
  }
  const auto [uMin, uMax] = *usual;
  double margin = std::max(params_.min_margin, params_.margin_ratio * (uMax - uMin));
  if (!periods_.empty()) {
    std::vector<double> p(periods_.begin(), periods_.end());
    margin = std::max(margin, median_of(p));
  }
  return std::make_pair(uMin - margin, uMax + margin);
}

PointTimeOutlierFilter::Result PointTimeOutlierFilter::apply(
  const std::vector<mrpt::maps::CPointsMap *> & clouds,
  const std::optional<std::pair<double, double>> & accepted)
{
  Result r;

  // 1st pass: time range of the scan.
  float tMin = std::numeric_limits<float>::max();
  float tMax = std::numeric_limits<float>::lowest();
  for (const auto * pc : clouds) {
    if (pc == nullptr) {
      continue;
    }
    const auto * ts = pc->getPointsBufferRef_float_field("t");
    if (ts == nullptr) {
      continue;
    }
    for (const float t : *ts) {
      // Comparisons with NaN are false: NaN times do not count here.
      tMin = std::min(tMin, t);
      tMax = std::max(tMax, t);
    }
    r.points += ts->size();
  }
  if (r.points == 0 || tMin > tMax) {
    r.points = 0;
    return r;  // no (finite) per-point times
  }
  r.scan_min = tMin;
  r.scan_max = tMax;

  if (!accepted) {
    return r;
  }
  r.accepted = accepted;
  const auto [lo, hi] = *accepted;
  if (tMin < lo || tMax > hi) {
    r.rejected = remove_outside(clouds, lo, hi);  // 2nd pass, only for scans with outliers
  }
  return r;
}

std::size_t PointTimeOutlierFilter::remove_outside(
  const std::vector<mrpt::maps::CPointsMap *> & clouds, const double lo, const double hi)
{
  std::size_t removed = 0;
  std::vector<uint8_t> keep;
  for (auto * pc : clouds) {
    if (pc == nullptr) {
      continue;
    }
    const auto * ts = pc->getPointsBufferRef_float_field("t");
    if (ts == nullptr || ts->empty()) {
      continue;
    }
    keep.resize(ts->size());
    bool any = false;
    for (std::size_t i = 0; i < ts->size(); i++) {
      const float t = (*ts)[i];
      // NaN times stay: they are not evidence of a clock jump.
      const bool out = t < lo || t > hi;
      keep[i] = out ? 0 : 1;
      any = any || out;
    }
    if (any) {
      removed += remove_points(*pc, keep);
    }
  }
  return removed;
}

PointTimeOutlierFilter::Result PointTimeOutlierFilter::check(
  const std::vector<mrpt::maps::CPointsMap *> & clouds) const
{
  if (!params_.enabled) {
    return {};
  }
  return apply(clouds, accepted_range());
}

PointTimeOutlierFilter::Result PointTimeOutlierFilter::process(
  const std::vector<mrpt::maps::CPointsMap *> & clouds, const double stamp)
{
  if (!params_.enabled) {
    return {};
  }
  const auto r = apply(clouds, accepted_range());
  if (r.points == 0) {
    return r;
  }

  // The window takes the range as given, so the medians stay robust:
  const auto maxLength = std::max<std::size_t>(1, params_.window_length);
  window_.emplace_back(r.scan_min, r.scan_max);
  while (window_.size() > maxLength) {
    window_.pop_front();
  }
  if (last_stamp_) {
    const double dt = stamp - *last_stamp_;
    if (dt > 0 && dt < MAX_PERIOD) {
      periods_.push_back(dt);
      while (periods_.size() > maxLength) {
        periods_.pop_front();
      }
    }
  }
  last_stamp_ = stamp;
  return r;
}

std::size_t PointTimeOutlierFilter::remove_points(
  mrpt::maps::CPointsMap & pc, const std::vector<uint8_t> & keep)
{
  const std::size_t n = pc.size();
  if (keep.size() != n) {
    return 0;
  }
  const auto nKeep =
    static_cast<std::size_t>(std::count_if(keep.begin(), keep.end(), [](uint8_t k) { return k; }));
  if (nKeep == n) {
    return 0;
  }

  // x, y, z are among the float fields:
  for (const auto & name : pc.getPointFieldNames_float()) {
    compact(pc.getPointsBufferRef_float_field(name), keep);
  }
  for (const auto & name : pc.getPointFieldNames_double()) {
    compact(pc.getPointsBufferRef_double_field(name), keep);
  }
  for (const auto & name : pc.getPointFieldNames_uint16()) {
    compact(pc.getPointsBufferRef_uint16_field(name), keep);
  }
  for (const auto & name : pc.getPointFieldNames_uint8()) {
    compact(pc.getPointsBufferRef_uint8_field(name), keep);
  }
  for (const auto & name : pc.getPointFieldNames_uint32()) {
    compact(pc.getPointsBufferRef_uint32_field(name), keep);
  }
  pc.resize(nKeep);
  pc.mark_as_modified();
  return n - nKeep;
}

}  // namespace mola
