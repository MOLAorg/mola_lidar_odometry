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
 * @file   LidarOdometry_GravityRebake.cpp
 * @brief  Online gravity rebake hooks: KF-pool maintenance and (later)
 *         per-KF trajectory rebake.
 */

#include <mola_lidar_odometry/LidarOdometry.h>
#include <mrpt/system/datetime.h>

namespace mola
{
mola::KeyframePointCloudMap * LidarOdometry::resolveKfmLayerOnce()
{
  if (state_.kfm_layer_resolved) {
    if (state_.kfm_layer_name.empty()) {
      return nullptr;
    }
    auto it = state_.local_map->layers.find(state_.kfm_layer_name);
    if (it == state_.local_map->layers.end()) {
      return nullptr;
    }
    return dynamic_cast<mola::KeyframePointCloudMap *>(it->second.get());
  }

  // First-time resolution: scan the layer set for a matching type.
  mola::KeyframePointCloudMap * found = nullptr;
  for (const auto & [layer_name, layer_map] : state_.local_map->layers) {
    auto * kfm = dynamic_cast<mola::KeyframePointCloudMap *>(layer_map.get());
    if (kfm != nullptr) {
      state_.kfm_layer_name = layer_name;
      found = kfm;
      break;
    }
  }

  state_.kfm_layer_resolved = true;

  if (found == nullptr && params_.imu_gravity_correction.enabled_map_realignment) {
    MRPT_LOG_WARN(
      "imu_gravity_correction.enabled_map_realignment=true, but the active local map has no "
      "KeyframePointCloudMap layer. Online gravity rebake disabled for this run.");
  }
  return found;
}

void LidarOdometry::onLocalMapKFInsertedForGravity(const mrpt::Clock::time_point & this_obs_tim)
{
  if (!params_.imu_gravity_correction.enabled_map_realignment) {
    return;
  }

#if defined(MOLA_METRIC_MAPS_HAS_KFM_POSE_PLUMBING) && \
  defined(MOLA_IMU_PREINTEGRATION_HAS_WINDOW_SINCE)

  auto * kfm = resolveKfmLayerOnce();
  if (kfm == nullptr) {
    return;
  }

  const auto kf_id_opt = kfm->lastInsertedKeyFrameID();
  if (!kf_id_opt) {
    return;
  }
  const auto kf_id = *kf_id_opt;

  const double now_s = mrpt::Clock::toDouble(this_obs_tim);

  // Slice the LocalVelocityBuffer over the interval covered by this KF.
  // First KF of the session has no prior timestamp: take everything
  // accumulated up to `now_s`.
  const double from_s = state_.last_kf_ts_s.value_or(0.0);
  const auto window = state_.parameter_source.localVelocityBuffer.window_since(from_s, now_s);

  // Feed the new KF into the per-KF accelerometer pool. If the window
  // happens to have no acc samples (e.g. IMU not yet active), the
  // aligner silently skips the insertion.
  state_.gravity_map_aligner.onNewKeyframe(kf_id, window);

  state_.last_kf_ts_s = now_s;

  // Drop any KFs that the local map evicted on this insertion (they
  // are the ones in the just-drained list, since the KF map only
  // appends on insert and accumulates evictions).
  const auto evicted = kfm->drainEvictedKeyFrameIDs();
  for (const auto evicted_id : evicted) {
    state_.gravity_map_aligner.onKeyframeDropped(evicted_id);
  }

#else
  (void)this_obs_tim;
  // Feature dependencies not available at build time: silently no-op.
#endif
}

}  // namespace mola
