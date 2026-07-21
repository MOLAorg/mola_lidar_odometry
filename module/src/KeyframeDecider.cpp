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

namespace mola
{

KeyframeDecider::Decision KeyframeDecider::check(
  const KeyframeDecisionOptions & opts, const mrpt::poses::CPose3D & pose) const
{
  Decision d;

  const double maxTranslation = opts.min_translation_between_keyframes;
  const double maxRotation = mrpt::DEG2RAD(opts.min_rotation_between_keyframes);

  // Distance to the closest keyframe ever created (or just to the last one,
  // if measure_from_last_kf_only).
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

void KeyframeDecider::insert(const mrpt::poses::CPose3D & pose) { poses_.insert(pose); }

void KeyframeDecider::removeAllFartherThan(const mrpt::poses::CPose3D & pose, double maxTranslation)
{
  poses_.removeAllFartherThan(pose, maxTranslation);
}

}  // namespace mola
