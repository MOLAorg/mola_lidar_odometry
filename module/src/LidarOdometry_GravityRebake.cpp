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
#include <mola_lidar_odometry/TrajectoryRebaker.h>
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

  // Incremental rebake of the trailing window of the local map.
  runIncrementalRebake(*kfm, kf_id, this_obs_tim);

#else
  (void)this_obs_tim;
  // Feature dependencies not available at build time: silently no-op.
#endif
}

void LidarOdometry::runIncrementalRebake(
  mola::KeyframePointCloudMap & kfm, mola::KeyframePointCloudMap::KeyFrameID new_kf_id,
  const mrpt::Clock::time_point & this_obs_tim)
{
#if defined(MOLA_METRIC_MAPS_HAS_KFM_POSE_PLUMBING)
  const auto & p = params_.imu_gravity_correction;

  if (state_.gravity_map_aligner.size() < p.min_keyframes) {
    return;
  }

  const ProfilerEntry tle(profiler_, "onLidar.7.gravity_rebake");

  // Snapshot of the (already gravity-aligned by previous rebakes)
  // local-map KF poses. The new KF's entry is whatever ICP just wrote
  // (in the previous rebaked frame), and is the tail of the chain.
  auto kf_poses = kfm.cloneKFPoses();
  if (kf_poses.empty() || kf_poses.find(new_kf_id) == kf_poses.end()) {
    return;
  }

  // Determine the rebake anchor: the start of the trailing window of
  // up to `rebake_window_kfs` KFs ending at `new_kf_id`. Older KFs
  // remain frozen at whatever a previous rebake wrote for them.
  const auto W = p.rebake_window_kfs;
  auto window_first_it = kf_poses.find(new_kf_id);
  for (uint32_t i = 0; i < W && window_first_it != kf_poses.begin(); ++i) {
    --window_first_it;
  }
  const auto anchor_id = window_first_it->first;

  // Per-KF pitch/roll corrections over the full pool (only those
  // landing in [anchor_id, new_kf_id] are consumed by the rebake).
  const auto r_grav = state_.gravity_map_aligner.estimatePerKFCorrection(
    kf_poses, p.window_half_width_kfs, p.min_pool_per_kf);
  if (r_grav.empty()) {
    return;
  }

  // Pre-rebake tail pose, captured BEFORE setKeyframePose mutates the map.
  const auto tail_before = kf_poses.at(new_kf_id);

  // Chain-integrate from the anchor forward.
  const auto result = TrajectoryRebaker::rebake(kf_poses, r_grav, anchor_id);

  if (result.corrected_poses.find(new_kf_id) == result.corrected_poses.end()) {
    return;  // rebaker could not reach the tail (degenerate window).
  }

  // Commit corrected poses back to the local-map KF layer.
  for (const auto & [id, pose] : result.corrected_poses) {
    kfm.setKeyframePose(id, pose);
  }
  state_.mark_local_map_as_updated();

  const auto & tail_after = result.corrected_poses.at(new_kf_id);

  // Live state updates so the next scan's ICP starts in the rebaked frame.
  state_.last_lidar_pose.mean = tail_after;

  // Re-fuse the corrected pose into navstate_fuse so its filter state
  // reflects the rebaked frame from now on.
  if (state_.navstate_fuse) {
    mrpt::poses::CPose3DPDFGaussian corrected_pdf;
    corrected_pdf.mean = tail_after;
    corrected_pdf.cov = state_.last_lidar_pose.cov;
    state_.navstate_fuse->fuse_pose(this_obs_tim, corrected_pdf, params_.publish_reference_frame);
  }

  // Overwrite the most-recent trajectory entry so it agrees with the
  // rebaked stored pose at this timestamp. (Older entries remain in
  // the previous frame; they are downstream-only output, not used by
  // ICP, so their staleness is benign for the live pipeline.)
  {
    auto lck = mrpt::lockHelper(state_trajectory_mtx_);
    state_.estimated_trajectory.insert(this_obs_tim, tail_after);
  }

  // Update the publish residual so the live publish stream is
  // continuous across this rebake event:
  //   pose_pub_after_rebake = T_grav_publish_new + tail_after
  //                         = T_grav_publish_old + tail_before
  // Hence T_grav_publish_new = T_grav_publish_old + (tail_before "minus" tail_after).
  state_.T_grav_publish =
    state_.T_grav_publish + TrajectoryRebaker::computePublishResidual(tail_before, tail_after);

  MRPT_LOG_DEBUG_FMT(
    "gravity_rebake: anchor_id=%llu new_kf_id=%llu window=%u pool=%zu rgrav=%zu",
    static_cast<unsigned long long>(anchor_id), static_cast<unsigned long long>(new_kf_id),
    static_cast<unsigned>(W), state_.gravity_map_aligner.size(), r_grav.size());
#else
  (void)kfm;
  (void)new_kf_id;
  (void)this_obs_tim;
#endif
}

}  // namespace mola
