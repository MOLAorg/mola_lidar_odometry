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
 * @file   KeyframeDecider.cpp
 * @brief  Shared "should a new keyframe be created here?" policy
 * @author Jose Luis Blanco Claraco
 */

#include <mola_lidar_odometry/KeyframeDecider.h>
#include <mrpt/poses/Lie/SO.h>

#include <algorithm>
#include <optional>

namespace mola
{

namespace
{
/** Translational and rotational distance of `p` with respect to `ref`. */
std::pair<double, double> pose_distance(
  const mrpt::poses::CPose3D & p, const mrpt::poses::CPose3D & ref)
{
  const mrpt::poses::CPose3D delta = p - ref;
  return {delta.norm(), mrpt::poses::Lie::SO<3>::log(delta.getRotationMatrix()).norm()};
}
}  // namespace

void KeyframeDecider::prune_recent(double now, double window) const
{
  // Always keep the newest entry, whatever its age: otherwise a stationary
  // vehicle would emit one keyframe per window once its last one ages out.
  while (recent_.size() > 1 && (now - recent_.front().first) > window) {
    recent_.pop_front();
  }
}

KeyframeDecider::Decision KeyframeDecider::check(
  const KeyframeDecisionOptions & opts, const mrpt::poses::CPose3D & pose, double timestamp) const
{
  Decision d;

  const double maxTranslation = opts.min_translation_between_keyframes;
  const double maxRotation = mrpt::DEG2RAD(opts.min_rotation_between_keyframes);

  // Temporal policy: only recent keyframes take part in the test, so
  // revisiting a mapped area does create new keyframes (needed by loop
  // closure to have both endpoints of a loop).
  if (opts.nearby_keyframe_time_window > 0) {
    prune_recent(timestamp, opts.nearby_keyframe_time_window);

    if (recent_.empty()) {
      d.create = true;
      return d;
    }

    // The window holds a handful of entries, so a linear scan is cheaper
    // than any spatial index. It doubles as the "occupied volume" count, so
    // min_nearby_poses_occupied keeps working under the temporal policy:
    uint32_t nearbyCount = 0;
    std::optional<double> closestTranslation;

    for (const auto & [t, p] : recent_) {
      const auto [dist, rot] = pose_distance(pose, p);

      if (!closestTranslation.has_value() || dist < *closestTranslation) {
        closestTranslation = dist;
        d.translation = dist;
        d.rotation = rot;
      }

      if (dist <= maxTranslation && rot <= maxRotation) {
        nearbyCount++;
      }
    }

    // With the default min_nearby_poses_occupied=1 this is just "no recent
    // keyframe is nearby".
    d.create = nearbyCount < std::max<uint32_t>(1, opts.min_nearby_poses_occupied);
    return d;
  }

  // Classic, purely spatial policy: distance to the closest keyframe ever
  // created (or just to the last one, if measure_from_last_kf_only).
  const auto [isFirstPose, distanceToClosest] = poses_.check(pose);

  d.translation = distanceToClosest.norm();
  d.rotation = mrpt::poses::Lie::SO<3>::log(distanceToClosest.getRotationMatrix()).norm();

  d.create = isFirstPose || d.translation > maxTranslation || d.rotation > maxRotation;

#if defined(MOLA_POSE_LIST_HAS_KFM_POSE_PLUMBING)
  // A volume is only "occupied" once enough keyframes were taken from it.
  // Used by non-repetitive-scan lidars, which need several scans per spot.
  if (!d.create && opts.min_nearby_poses_occupied > 1) {
    const uint32_t nearbyCount = poses_.countNearby(pose, maxTranslation, maxRotation);
    if (nearbyCount < opts.min_nearby_poses_occupied) {
      d.create = true;
    }
  }
#else
  // Older mola_pose_list: countNearby() unavailable, min_nearby_poses_occupied
  // has no effect.
#endif

  return d;
}

void KeyframeDecider::insert(
  const KeyframeDecisionOptions & opts, const mrpt::poses::CPose3D & pose, double timestamp)
{
  poses_.insert(pose);

  // Only maintained while the temporal policy is in use: nothing ever prunes
  // this deque otherwise, and it would grow for the whole run to no purpose.
  if (opts.nearby_keyframe_time_window > 0) {
    recent_.emplace_back(timestamp, pose);
  }
}

void KeyframeDecider::removeAllFartherThan(const mrpt::poses::CPose3D & pose, double maxTranslation)
{
  poses_.removeAllFartherThan(pose, maxTranslation);
}

}  // namespace mola
