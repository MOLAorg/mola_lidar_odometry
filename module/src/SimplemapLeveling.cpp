/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/
/**
 * @file   SimplemapLeveling.cpp
 * @brief  Moves simplemap keyframes with the correction of a leveled path
 * @author Jose Luis Blanco Claraco
 */

#include <mola_lidar_odometry/SimplemapLeveling.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/system/datetime.h>

#include <algorithm>

std::optional<mrpt::Clock::time_point> mola::keyframe_stamp(const mrpt::obs::CSensoryFrame & sf)
{
  std::optional<mrpt::Clock::time_point> first;
  for (const auto & o : sf) {
    if (!o || o->timestamp == mrpt::system::InvalidTimeStamp()) {
      continue;
    }
    if (o->sensorLabel == "metadata") {
      return o->timestamp;
    }
    if (!first) {
      first = o->timestamp;
    }
  }
  return first;
}

std::size_t mola::level_simplemap_keyframes(
  mrpt::maps::CSimpleMap & sm, const mrpt::poses::CPose3DInterpolator & before,
  const mrpt::poses::CPose3DInterpolator & after)
{
  if (before.empty() || after.empty()) {
    return 0;
  }
  std::size_t n = 0;
  for (std::size_t i = 0; i < sm.size(); i++) {
    auto & kf = sm.get(i);
    if (!kf.pose || !kf.sf) {
      continue;
    }
    const auto stamp = keyframe_stamp(*kf.sf);
    if (!stamp) {
      continue;
    }
    // Each path held at its own ends (they need not span the same times):
    const auto t_before = std::clamp(*stamp, before.begin()->first, before.rbegin()->first);
    const auto t_after = std::clamp(*stamp, after.begin()->first, after.rbegin()->first);
    mrpt::poses::CPose3D p_before;
    mrpt::poses::CPose3D p_after;
    bool ok_before = false;
    bool ok_after = false;
    before.interpolate(t_before, p_before, ok_before);
    after.interpolate(t_after, p_after, ok_after);
    if (!ok_before || !ok_after) {
      continue;
    }
    // A copy: keyframes may share their pose objects with another map.
    auto moved =
      std::dynamic_pointer_cast<mrpt::poses::CPose3DPDF>(kf.pose->duplicateGetSmartPtr());
    ASSERT_(moved);
    moved->changeCoordinatesReference(p_after + (-p_before));
    kf.pose = moved;
    n++;
  }
  return n;
}
