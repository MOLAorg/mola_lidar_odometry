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
 * @file   PoseHistory.h
 * @brief  Queries on a timestamped history of vehicle poses (e.g. an odometry source)
 * @author Jose Luis Blanco Claraco
 */
#pragma once

#include <mrpt/math/TPose3D.h>
#include <mrpt/math/slerp.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

#include <iterator>
#include <map>
#include <optional>

namespace mola
{
/** Pose at time t [s] from a history of poses keyed by timestamp [s]: the
 *  stored one at an exact sample time, else the SE(3) interpolation (SLERP)
 *  between the samples around t. A query at most maxEndGap past either end
 *  takes that end's pose (e.g. a newest stamp slightly after the last
 *  reading). nullopt if the history is empty or does not cover t.
 *
 * \ingroup mola_lidar_odometry_grp
 */
inline std::optional<mrpt::poses::CPose3D> pose_history_at(
  const std::map<double, mrpt::poses::CPose3D> & poses, double t, double maxEndGap)
{
  if (poses.empty()) {
    return std::nullopt;
  }
  const auto after = poses.lower_bound(t);
  if (after != poses.end() && after->first == t) {
    return after->second;
  }
  if (after == poses.begin()) {
    return (after->first - t) <= maxEndGap ? std::optional(after->second) : std::nullopt;
  }
  if (after == poses.end()) {
    const auto & last = *poses.rbegin();
    return (t - last.first) <= maxEndGap ? std::optional(last.second) : std::nullopt;
  }
  const auto before = std::prev(after);
  const double f = (t - before->first) / (after->first - before->first);
  mrpt::math::TPose3D p;
  mrpt::math::slerp(before->second.asTPose(), after->second.asTPose(), f, p);
  return mrpt::poses::CPose3D(p);
}

/** Whether the history brackets time t [s], i.e. a pose at t is exact or
 *  interpolated, not an end pose taken for a t just outside it (which later
 *  readings, even out of order, may still improve).
 *
 * \ingroup mola_lidar_odometry_grp
 */
inline bool pose_history_brackets(const std::map<double, mrpt::poses::CPose3D> & poses, double t)
{
  return !poses.empty() && poses.begin()->first <= t && poses.rbegin()->first >= t;
}

/** Planar motion from `from` to `to`: that between their projections on the
 *  horizontal plane (x, y, yaw), in the frame of the first one. Unlike the
 *  projection of their 3D relative pose, a tilted vehicle does not stretch it.
 *
 * \ingroup mola_lidar_odometry_grp
 */
inline mrpt::poses::CPose2D planar_motion(
  const mrpt::poses::CPose3D & from, const mrpt::poses::CPose3D & to)
{
  return mrpt::poses::CPose2D(to) - mrpt::poses::CPose2D(from);
}

}  // namespace mola
