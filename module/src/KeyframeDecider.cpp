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

namespace mola
{

namespace
{
/** Rotation magnitude of a pose increment [rad]. Markedly costlier than the
 *  translational norm, so callers should reach it only when needed. */
double rotation_distance(const mrpt::poses::CPose3D & delta)
{
  return mrpt::poses::Lie::SO<3>::log(delta.getRotationMatrix()).norm();
}
}  // namespace

void KeyframeDecider::prune_recent(double now, double window)
{
  // Always keep the newest entry, whatever its age: otherwise a stationary
  // vehicle would emit one keyframe per window once its last one ages out.
  while (recent_.size() > 1 && (now - recent_.front().first) > window) {
    recent_.pop_front();
  }
}

KeyframeDecider::Decision KeyframeDecider::check(
  const KeyframeDecisionOptions & opts, const mrpt::poses::CPose3D & pose, double timestamp)
{
  ASSERTMSG_(
    temporal_ == (opts.nearby_keyframe_time_window > 0),
    "nearby_keyframe_time_window must not change after the decider was built");

  Decision d;

  const double maxTranslation = opts.min_translation_between_keyframes;
  const double maxRotation = mrpt::DEG2RAD(opts.min_rotation_between_keyframes);

  // Temporal policy: only recent keyframes take part in the test, so
  // revisiting a mapped area does create new keyframes (needed by loop
  // closure to have both endpoints of a loop).
  if (temporal_) {
    prune_recent(timestamp, opts.nearby_keyframe_time_window);

    if (recent_.empty()) {
      d.create = true;
      return d;
    }

    // Diagnostics are reported against the newest keyframe, which is what the
    // caller logs as "since last KF" (and is O(1), unlike the closest one):
    {
      const mrpt::poses::CPose3D delta = pose - recent_.back().second;
      d.translation = delta.norm();
      d.rotation = rotation_distance(delta);
    }

    // Linear scan over the window, newest first: the vehicle is normally
    // closest to the most recent keyframes, so the count is reached, and the
    // loop left, after very few iterations. The count doubles as the
    // "occupied volume" test, so min_nearby_poses_occupied keeps working
    // under the temporal policy.
    const uint32_t occupiedCount = std::max<uint32_t>(1, opts.min_nearby_poses_occupied);
    uint32_t nearbyCount = 0;

    for (auto it = recent_.rbegin(); it != recent_.rend() && nearbyCount < occupiedCount; ++it) {
      const mrpt::poses::CPose3D delta = pose - it->second;

      // Cheap test first: only candidates that already passed it are worth
      // the rotation logarithm.
      if (delta.norm() > maxTranslation) {
        continue;
      }

      if (rotation_distance(delta) <= maxRotation) {
        nearbyCount++;
      }
    }

    // With the default min_nearby_poses_occupied=1 this is just "no recent
    // keyframe is nearby".
    d.create = nearbyCount < occupiedCount;
    return d;
  }

  // Classic, purely spatial policy: distance to the closest keyframe ever
  // created (or just to the last one, if measure_from_last_kf_only).
  const auto [isFirstPose, distanceToClosest] = poses_.check(pose);

  d.translation = distanceToClosest.norm();
  d.rotation = rotation_distance(distanceToClosest);

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

void KeyframeDecider::insert(const mrpt::poses::CPose3D & pose, double timestamp)
{
  if (!temporal_) {
    poses_.insert(pose);
    return;
  }

  // prune_recent() pops from the front, so the deque must stay sorted in time.
  // Guard against non-monotonic input (sensor jitter across a multi-lidar
  // group, replayed or missing timestamps):
  if (!recent_.empty()) {
    timestamp = std::max(timestamp, recent_.back().first);
  }

  // Same semantics as SearchablePoseList's own "last only" mode:
  if (from_last_only_) {
    recent_.clear();
  }

  recent_.emplace_back(timestamp, pose);
  created_++;
}

void KeyframeDecider::removeAllFartherThan(const mrpt::poses::CPose3D & pose, double maxTranslation)
{
  if (!temporal_) {
    poses_.removeAllFartherThan(pose, maxTranslation);
    return;
  }

  const double maxSqrDist = mrpt::square(maxTranslation);
  const auto center = pose.translation();

  std::deque<Entry> kept;
  for (const auto & e : recent_) {
    if ((e.second.translation() - center).sqrNorm() <= maxSqrDist) {
      kept.push_back(e);
    }
  }

  // Keep the newest entry regardless of distance, as prune_recent() does:
  if (kept.empty() && !recent_.empty()) {
    kept.push_back(recent_.back());
  }

  // Keyframes dropped here are gone for good, so they stop being counted:
  created_ -= std::min(created_, recent_.size() - kept.size());

  recent_.swap(kept);
}

}  // namespace mola
