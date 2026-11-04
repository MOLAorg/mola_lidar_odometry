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
 * @file   LidarOdometry_LocalMapRebuild.cpp
 * @brief  Rebuilds the local map from the corrected keyframes of a global mapper
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_lidar_odometry/LidarOdometry.h>

#if defined(MOLA_LO_HAS_LOCAL_MAP_REBUILD)

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp/pointcloud_field_utils.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/datetime.h>
#include <mrpt/version.h>

#include <algorithm>
#include <chrono>
#include <regex>
#include <utility>

namespace mola
{
namespace
{
/** The keyframe metadata written by LidarOdometry::appendKeyframeMetadataObs(),
 *  if any. */
std::optional<mrpt::containers::yaml> keyframe_metadata(const mrpt::obs::CSensoryFrame & sf)
{
  for (std::size_t i = 0; i < sf.size(); i++) {
    const auto c =
      std::dynamic_pointer_cast<const mrpt::obs::CObservationComment>(sf.getObservationByIndex(i));
    if (!c) {
      continue;
    }
    try {
      auto y = mrpt::containers::yaml::FromText(c->text);
      if (y.isMap() && y.has("local_velocity_buffer")) {
        return y;
      }
    } catch (const std::exception &) {
      // Not the keyframe metadata.
    }
  }
  return std::nullopt;
}

/** The newest sample at or before t, or the oldest one if all are later. */
template <typename MAP>
std::optional<typename MAP::mapped_type> sample_at(const MAP & m, double t)
{
  if (m.empty()) {
    return std::nullopt;
  }
  auto it = m.upper_bound(t);
  if (it != m.begin()) {
    --it;
  }
  return it->second;
}

/** Re-expresses all point layers in another frame (point coordinates, and the
 *  view and normal direction fields). Other layers are dropped. */
void transform_point_layers(mp2p_icp::metric_map_t & m, const mrpt::poses::CPose3D & b)
{
  for (auto it = m.layers.begin(); it != m.layers.end();) {
    const auto * pts = it->second ? mp2p_icp::MapToPointsMap(*it->second) : nullptr;
    if (pts == nullptr) {
      it = m.layers.erase(it);
      continue;
    }
    auto out = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
      mrpt::rtti::classFactory(pts->GetRuntimeClass()->className));
    ASSERT_(out);
    out->insertAnotherMap(pts, b);
#if MRPT_VERSION < 0x030600
    // Older versions copy the direction fields verbatim:
    mp2p_icp::rotateViewDirectionFields(*out, b);
#endif
    it->second = out;
    ++it;
  }
}

std::vector<mrpt::obs::CObservation::ConstPtr> lidar_observations(
  const mrpt::obs::CSensoryFrame & sf, const std::vector<std::regex> & labels)
{
  std::vector<mrpt::obs::CObservation::ConstPtr> out;
  for (std::size_t i = 0; i < sf.size(); i++) {
    const mrpt::obs::CObservation::ConstPtr o = sf.getObservationByIndex(i);
    for (const auto & re : labels) {
      if (std::regex_match(o->sensorLabel, re)) {
        out.push_back(o);
        break;
      }
    }
  }
  return out;
}

void set_robot_pose_variables(mp2p_icp::ParameterSource & ps, const mrpt::poses::CPose3D & p)
{
  ps.updateVariable("robot_x", p.x());
  ps.updateVariable("robot_y", p.y());
  ps.updateVariable("robot_z", p.z());
  ps.updateVariable("robot_yaw", p.yaw());
  ps.updateVariable("robot_pitch", p.pitch());
  ps.updateVariable("robot_roll", p.roll());
}

/** As LidarOdometry::insertScanIntoLocalMap(): the observation layers are
 *  merged into the map by the obs2map_merge pipeline, with the robot pose. */
void merge_into_map(
  mp2p_icp::ParameterSource & ps, const mp2p_icp_filters::FilterPipeline & obs2map,
  const mp2p_icp::metric_map_t & observation, const mrpt::poses::CPose3D & robotPose,
  mp2p_icp::metric_map_t & map)
{
  for (const auto & [name, layer] : observation.layers) {
    ASSERTMSG_(
      map.layers.count(name) == 0,
      "Local map layer name '" + name + "' collides with an observation layer.");
    map.layers[name] = layer;
  }
  set_robot_pose_variables(ps, robotPose);
  ps.realize();
  mp2p_icp_filters::apply_filter_pipeline(obs2map, map);
  for (const auto & [name, layer] : observation.layers) {
    map.layers.erase(name);
  }
}

}  // namespace

void LidarOdometry::startLocalMapRebuild()
{
  if (local_map_correction_source_) {
    return;  // already subscribed (initialize() again after a reset())
  }
  if (
    !params_.local_map_updates.rebuild_on_corrections || !params_.local_map_updates.enabled ||
    !state_.shared_keyframe_map_sink) {
    return;
  }
  // The keyframe ids are those of the sink, so only the sink itself can be asked:
  local_map_correction_source_ =
    std::dynamic_pointer_cast<mola::LocalMapCorrectionSource>(state_.shared_keyframe_map_sink);
  if (!local_map_correction_source_) {
    return;
  }
  if (params_.simplemap.generate && params_.simplemap.generate_lazy_load_scan_files) {
    // Those keyframe scans are moved to disk and unloaded by another thread,
    // while a rebuild would read them.
    local_map_correction_source_.reset();
    MRPT_LOG_WARN(
      "Local map rebuilds disabled: not compatible with "
      "simplemap.generate_lazy_load_scan_files.");
    return;
  }
  rebuild_config_ = lastInitConfig_;

  local_map_rebuild_thread_ = std::thread([this]() { localMapRebuildThreadLoop(); });
  local_map_correction_subscription_ =
    local_map_correction_source_->subscribe_to_local_map_corrections(
      [this](const mola::LocalMapCorrection &) {
        {
          auto lck = mrpt::lockHelper(local_map_rebuild_mtx_);
          local_map_rebuild_requested_ = true;
        }
        local_map_rebuild_cv_.notify_one();
      });
  MRPT_LOG_INFO(
    "Local map will be rebuilt from the corrected keyframes of the central map after each of "
    "its corrections.");
}

void LidarOdometry::stopLocalMapRebuild()
{
  if (!local_map_correction_source_) {
    return;
  }
  local_map_correction_source_->unsubscribe_from_local_map_corrections(
    local_map_correction_subscription_);
  {
    auto lck = mrpt::lockHelper(local_map_rebuild_mtx_);
    local_map_rebuild_exit_ = true;
  }
  local_map_rebuild_cancel_ = true;  // a rebuild in progress stops early
  local_map_rebuild_cv_.notify_one();
  if (local_map_rebuild_thread_.joinable()) {
    local_map_rebuild_thread_.join();
  }
}

void LidarOdometry::localMapRebuildThreadLoop()
{
  for (;;) {
    {
      std::unique_lock<std::mutex> lck(local_map_rebuild_mtx_);
      local_map_rebuild_cv_.wait(
        lck, [this]() { return local_map_rebuild_requested_ || local_map_rebuild_exit_; });
      if (local_map_rebuild_exit_) {
        return;
      }
      // Corrections arriving while rebuilding are merged into the next one:
      local_map_rebuild_requested_ = false;
    }
    bool done = false;
    try {
      done = rebuildLocalMap();
    } catch (const std::exception & e) {
      MRPT_LOG_ERROR_STREAM("Local map rebuild failed: " << e.what());
      auto lck = mrpt::lockHelper(state_mtx_);
      state_.local_map_rebuild_reference.reset();
    }
    // Not done (e.g. no reference yet, or the map changed meanwhile): retried
    // after the next pushed keyframe, since the source will not repeat it.
    local_map_rebuild_pending_ = !done;
  }
}

void LidarOdometry::invalidateLocalMapRebuild()
{
  local_map_generation_++;
  state_.last_pushed_keyframe.reset();
  state_.recent_local_map_insertions.clear();
}

void LidarOdometry::onKeyframePushed(
  mola::SharedKeyframeMap::KeyFrameID id, const mrpt::Clock::time_point & stamp,
  const mrpt::poses::CPose3D & pose, uint64_t generation)
{
  if (!local_map_correction_source_ || generation != local_map_generation_.load()) {
    return;
  }
  state_.last_pushed_keyframe = MethodState::PushedKeyframe{id, stamp, pose};
  if (local_map_rebuild_pending_.exchange(false)) {
    {
      auto lck = mrpt::lockHelper(local_map_rebuild_mtx_);
      local_map_rebuild_requested_ = true;
    }
    local_map_rebuild_cv_.notify_one();
  }

  // Only the insertions after the reference of the next rebuild (or of the one
  // in progress) are needed:
  auto oldestNeeded = stamp;
  if (state_.local_map_rebuild_reference && *state_.local_map_rebuild_reference < oldestNeeded) {
    oldestNeeded = *state_.local_map_rebuild_reference;
  }
  auto & q = state_.recent_local_map_insertions;
  while (!q.empty() && q.front().stamp <= oldestNeeded) {
    q.pop_front();
  }
}

std::optional<mp2p_icp::metric_map_t> LidarOdometry::preprocessKeyframeForRebuild(
  const mrpt::obs::CSensoryFrame & sf, const mrpt::Clock::time_point & stamp,
  const std::vector<std::regex> & lidarLabels, bool earlyDeskew)
{
  auto & p = *rebuild_pipelines_;

  const auto lidarObs = lidar_observations(sf, lidarLabels);
  if (lidarObs.empty()) {
    return std::nullopt;
  }

  // De-skew with the velocities the scan had: the twist used then (constant
  // velocity methods), and its velocity buffer (IMU methods):
  mrpt::math::TTwist3D twist(0, 0, 0, 0, 0, 0);
  p.parameter_source.localVelocityBuffer.clear();
  if (const auto md = keyframe_metadata(sf); md) {
    const auto & y = *md;
    if (y["local_velocity_buffer"].isMap()) {
      p.parameter_source.localVelocityBuffer.fromYAML(y["local_velocity_buffer"]);
    }
    if (
      y.has("deskew_twist") && y["deskew_twist"].isSequence() &&
      y["deskew_twist"].asSequence().size() == 6) {
      const auto & tw = y["deskew_twist"].asSequence();
      for (int i = 0; i < 6; i++) {
        twist[i] = tw.at(i).as<double>();
      }
    } else {
      // Older keyframes: the buffer velocities at the scan time.
      const double t = mrpt::Clock::toDouble(stamp);
      const auto & vb = p.parameter_source.localVelocityBuffer;
      if (const auto v = sample_at(vb.get_linear_velocities(), t); v) {
        twist.vx = v->x;
        twist.vy = v->y;
        twist.vz = v->z;
      }
      if (const auto w = sample_at(vb.get_angular_velocities(), t); w) {
        twist.wx = w->x;
        twist.wy = w->y;
        twist.wz = w->z;
      }
    }
  }
  p.parameter_source.updateVariable("vx", twist.vx);
  p.parameter_source.updateVariable("vy", twist.vy);
  p.parameter_source.updateVariable("vz", twist.vz);
  p.parameter_source.updateVariable("wx", twist.wx);
  p.parameter_source.updateVariable("wy", twist.wy);
  p.parameter_source.updateVariable("wz", twist.wz);
  p.parameter_source.realize();

  // As LidarOdometry::observationFromRawSensor() and preprocessScan():
  mp2p_icp::metric_map_t out;
  const auto timeOfFirstObs = lidarObs.front()->timestamp;
  for (const auto & o : lidarObs) {
    mp2p_icp::metric_map_t thisObs;
    mp2p_icp_filters::apply_generators(p.obs_generators, *o, thisObs);
    p.parameter_source.updateVariable(
      "SENSOR_TIME_OFFSET", mrpt::system::timeDifference(timeOfFirstObs, o->timestamp));
    p.parameter_source.realize();
    mp2p_icp_filters::apply_filter_pipeline(p.adjust_timestamps, thisObs);
    out.merge_with(thisObs);
  }
  mp2p_icp_filters::apply_filter_pipeline(p.prefilter, out);
  if (earlyDeskew) {
    mp2p_icp_filters::apply_filter_pipeline(p.deskew, out);
  } else if (const auto it = out.layers.find("raw"); it != out.layers.end()) {
    out.layers["deskewed"] = it->second;
  }
  mp2p_icp_filters::apply_filter_pipeline(p.filter1, out);
  mp2p_icp_filters::apply_filter_pipeline(p.filter2, out);
  // What the scan keeps after its ICP (e.g. without the full clouds):
  mp2p_icp_filters::apply_filter_pipeline(p.final_pass, out);
  if (out.empty()) {
    return std::nullopt;
  }
  return out;
}

bool LidarOdometry::rebuildLocalMap()
{
  using mrpt::poses::CPose3D;
  const auto t0 = std::chrono::steady_clock::now();

  // 1/5: what to rebuild, from the scan thread state:
  MethodState::PushedKeyframe ref;
  uint64_t generation = 0;
  double radius = 0;
  double spacing = 0;
  KeyframeDecisionOptions deciderOptions;
  std::vector<std::regex> lidarLabels;
  std::string ownSource;
  bool reexpressInReference = true;
  bool earlyDeskew = false;
  std::map<std::string, double> variables;
  {
    auto lck = mrpt::lockHelper(state_mtx_);
    if (state_.map_has_been_loaded) {
      // Its contents are not in the central map: a rebuild would drop them.
      MRPT_LOG_ONCE_INFO("Local map not rebuilt after corrections: it was loaded from a file.");
      return true;
    }
    if (
      !state_.last_pushed_keyframe || !state_.local_map || !params_.local_map_updates.enabled ||
      state_.local_map_rebuild_reference) {
      return false;
    }
    ref = *state_.last_pushed_keyframe;
    generation = local_map_generation_.load();
    radius = params_.local_map_updates.max_distance_to_keep_keyframes;
    spacing = params_.local_map_updates.rebuild_keyframe_spacing;
    deciderOptions = params_.local_map_updates;
    lidarLabels = params_.lidar_sensor_labels;
    ownSource = params_.publish_reference_frame + "_kf";
    // Point clouds only use the insertion pose to transform the points (and to
    // evict distant ones): other classes keep it (ray origins, keyframe poses).
    for (const auto & [name, layer] : state_.local_map->layers) {
      if (!std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(layer)) {
        reexpressInReference = false;
      }
    }
    state_.local_map_rebuild_reference = ref.stamp;
    auto lckImu = mrpt::lockHelper(imu_state_mtx_);
    // As preprocessScan():
    earlyDeskew =
      !state_.pc_deskew.empty() && (!params_.optimize_twist || isPipelineUsingIMU_locked());
    variables = state_.parameter_source.getVariableValues();
  }
  const auto finish = [this]() {
    auto lck = mrpt::lockHelper(state_mtx_);
    state_.local_map_rebuild_reference.reset();
  };
  const auto aborted = [&]() {
    return local_map_rebuild_cancel_.load() || generation != local_map_generation_.load();
  };
  if (radius <= 0) {
    radius = 100.0;  // a local map without distant keyframe removal
  }

  // 2/5: the corrected keyframes around the reference, newest first, one per
  // rebuild_keyframe_spacing (denser ones mostly repeat points). Own keyframes
  // pushed after the reference are among the recent scans already.
  const auto keyframes = local_map_correction_source_->local_map_keyframes(ref.id, radius);
  if (!keyframes) {
    finish();
    return false;
  }
  std::vector<const LocalMapKeyframe *> selected;
  for (auto it = keyframes->rbegin(); it != keyframes->rend(); ++it) {
    const auto & kf = *it;
    if (kf.source_frame_id == ownSource && kf.timestamp > ref.stamp) {
      continue;
    }
    const auto pos = kf.pose_from_reference.translation();
    const bool tooClose = std::any_of(selected.begin(), selected.end(), [&](const auto * u) {
      return (u->pose_from_reference.translation() - pos).norm() < spacing;
    });
    if (!tooClose) {
      selected.push_back(&kf);
    }
  }
  if (!reexpressInReference) {
    // Inserted from their own poses, these classes evict around each one: the
    // farthest first, and the reference last.
    std::stable_sort(selected.begin(), selected.end(), [](const auto * a, const auto * b) {
      return a->pose_from_reference.translation().norm() >
             b->pose_from_reference.translation().norm();
    });
  }

  // 3/5: the pipelines, once:
  if (!rebuild_pipelines_) {
    auto p = std::make_unique<RebuildPipelines>();
    const auto & c = rebuild_config_;
    const auto v = this->getMinLoggingLevel();
    if (c.has("observations_generator") && !c["observations_generator"].isNullNode()) {
      p->obs_generators = mp2p_icp_filters::generators_from_yaml(c["observations_generator"], v);
    } else {
      auto g = mp2p_icp_filters::Generator::Create();
      g->initialize({});
      p->obs_generators.push_back(g);
    }
    const auto pipeline = [&](const char * key, mp2p_icp_filters::FilterPipeline & out) {
      if (c.has(key)) {
        out = mp2p_icp_filters::filter_pipeline_from_yaml(c[key], v);
      }
    };
    pipeline("observations_filter_adjust_timestamps", p->adjust_timestamps);
    if (c.has("observations_prefilter_file")) {
      if (const auto f = c["observations_prefilter_file"].as<std::string>(); !f.empty()) {
        p->prefilter =
          mp2p_icp_filters::filter_pipeline_from_yaml(mrpt::containers::yaml::FromFile(f), v);
      }
    }
    pipeline("observations_deskew_pass", p->deskew);
    pipeline("observations_filter_1st_pass", p->filter1);
    pipeline("observations_filter_2nd_pass", p->filter2);
    pipeline("observations_filter_final_pass", p->final_pass);
    if (c.has("localmap_generator") && !c["localmap_generator"].isNullNode()) {
      p->local_map_generators = mp2p_icp_filters::generators_from_yaml(c["localmap_generator"], v);
    } else {
      auto g = mp2p_icp_filters::Generator::Create();
      g->initialize({});
      p->local_map_generators.push_back(g);
    }
    pipeline("insert_observation_into_local_map", p->obs2map_merge);
    ASSERT_(!p->obs2map_merge.empty());

    for (auto * fp :
         {&p->adjust_timestamps, &p->prefilter, &p->deskew, &p->filter1, &p->filter2,
          &p->final_pass, &p->obs2map_merge}) {
      mp2p_icp::AttachToParameterSource(*fp, p->parameter_source);
    }
    mp2p_icp::AttachToParameterSource(p->obs_generators, p->parameter_source);
    mp2p_icp::AttachToParameterSource(p->local_map_generators, p->parameter_source);
    rebuild_pipelines_ = std::move(p);
  }
  auto & p = *rebuild_pipelines_;
  for (const auto & [name, value] : variables) {
    p.parameter_source.updateVariable(name, value);
  }
  p.parameter_source.realize();

  // 4/5: the new local map, anchored at the reference pose of this odometry, so
  // the vehicle pose does not jump. For point clouds, each keyframe goes in
  // expressed in the reference frame, with the reference as robot pose, so the
  // map keeps the points within its radius of the reference, wherever each one
  // was taken from. Other map classes get each keyframe from its own pose.
  auto newMap = mp2p_icp::metric_map_t::Create();
  for (const auto * kf : selected) {
    const auto lidarObs = lidar_observations(kf->observations, lidarLabels);
    if (!lidarObs.empty()) {
      for (const auto & o : lidarObs) {
        mp2p_icp_filters::apply_generators(p.local_map_generators, *o, *newMap);
      }
      break;
    }
  }
  if (newMap->layers.empty()) {
    finish();
    return true;  // nothing to rebuild from
  }

  // The recent scans: merged first into point clouds, so their points win over
  // older ones nearby; last into the others, so eviction centers on them.
  std::size_t numRecent = 0;
  auto mergedUpTo = ref.stamp;
  std::vector<std::pair<double, CPose3D>> recentDeciderEntries;
  const auto mergeRecent = [&](const std::vector<MethodState::LocalMapInsertion> & todo) {
    for (const auto & ins : todo) {
      merge_into_map(p.parameter_source, p.obs2map_merge, ins.observation, ins.robot_pose, *newMap);
      recentDeciderEntries.emplace_back(ins.obs_timestamp, ins.robot_pose + ins.sensor_pose);
      mergedUpTo = ins.stamp;
      numRecent++;
    }
  };
  const auto recentSince = [&](const mrpt::Clock::time_point & t) {
    std::vector<MethodState::LocalMapInsertion> todo;
    for (const auto & ins : state_.recent_local_map_insertions) {
      if (ins.stamp > t) {
        todo.push_back(ins);
      }
    }
    return todo;
  };
  if (reexpressInReference) {
    std::vector<MethodState::LocalMapInsertion> todo;
    {
      auto lck = mrpt::lockHelper(state_mtx_);
      todo = recentSince(mergedUpTo);
    }
    mergeRecent(todo);
  }

  // Sensor poses for the new keyframe decider, inserted in time order below:
  std::vector<std::pair<double, CPose3D>> deciderEntries;
  double oldestAge = 0;
  std::size_t numFailed = 0;
  for (const auto * kfp : selected) {
    if (aborted()) {
      finish();
      return false;
    }
    const auto & kf = *kfp;
    try {
      auto obs =
        preprocessKeyframeForRebuild(kf.observations, kf.timestamp, lidarLabels, earlyDeskew);
      if (!obs) {
        continue;
      }
      const auto lidarObs = lidar_observations(kf.observations, lidarLabels);
      const CPose3D kfPose = ref.pose + kf.pose_from_reference;
      if (reexpressInReference) {
        transform_point_layers(*obs, kf.pose_from_reference);
        merge_into_map(p.parameter_source, p.obs2map_merge, *obs, ref.pose, *newMap);
      } else {
        merge_into_map(p.parameter_source, p.obs2map_merge, *obs, kfPose, *newMap);
      }
      // The keyframe decider holds sensor poses:
      deciderEntries.emplace_back(
        mrpt::Clock::toDouble(lidarObs.front()->timestamp),
        kfPose + lidarObs.front()->getSensorPose());
      oldestAge = std::max(oldestAge, mrpt::system::timeDifference(kf.timestamp, ref.stamp));
    } catch (const std::exception & e) {
      // E.g. a keyframe of another front end: the others still count.
      numFailed++;
      MRPT_LOG_DEBUG_STREAM("Local map rebuild: keyframe " << kf.id << " skipped: " << e.what());
    }
  }
  if (deciderEntries.empty()) {
    finish();
    return true;
  }
  if (numFailed > 0) {
    MRPT_LOG_WARN_FMT("Local map rebuild: %zu keyframes could not be used.", numFailed);
  }

  // 5/5: the scans inserted since, and the swap. Most of them are merged without
  // state_mtx_, so the scan thread is not blocked; only the last few, under it,
  // right before the swap.
  mp2p_icp::metric_map_t::Ptr oldMap;
  for (int pass = 0;; pass++) {
    std::unique_lock<std::mutex> lck(state_mtx_);
    if (aborted() || !state_.local_map || !params_.local_map_updates.enabled) {
      state_.local_map_rebuild_reference.reset();
      MRPT_LOG_INFO("Local map rebuild discarded: the local map changed meanwhile.");
      return false;
    }
    const auto todo = recentSince(mergedUpTo);
    constexpr std::size_t MAX_UNDER_LOCK = 5;
    constexpr int MAX_PASSES = 4;
    const bool last = todo.size() <= MAX_UNDER_LOCK || pass + 1 >= MAX_PASSES;
    if (!last) {
      lck.unlock();
    }
    mergeRecent(todo);
    if (!last) {
      continue;
    }

    KeyframeDecider newDecider(deciderOptions);
    std::sort(deciderEntries.begin(), deciderEntries.end(), [](const auto & a, const auto & b) {
      return a.first < b.first;
    });
    std::sort(
      recentDeciderEntries.begin(), recentDeciderEntries.end(),
      [](const auto & a, const auto & b) { return a.first < b.first; });
    for (const auto * entries : {&deciderEntries, &recentDeciderEntries}) {
      for (const auto & [stamp, pose] : *entries) {
        newDecider.insert(pose, stamp);
      }
    }

    state_.local_map_rebuild_reference.reset();
    newMap->metadata = state_.local_map->metadata;
    newMap->georeferencing = state_.local_map->georeferencing;
    {
      auto lckMapContents = mrpt::lockHelper(local_map_content_mtx_);
      // Destroyed below, without the locks (a large map takes a while):
      oldMap = std::exchange(state_.local_map, newMap);
    }
    state_.kf_decider_local_map.emplace(std::move(newDecider));
    state_.mark_local_map_as_updated(true /*force_republish*/);
    break;
  }
  oldMap.reset();
  local_map_rebuilds_++;

  MRPT_LOG_INFO_FMT(
    "Local map rebuilt from %zu of %zu corrected keyframes (the oldest one %.0f s before "
    "the reference) and %zu recent scans, in %.0f ms.",
    deciderEntries.size(), keyframes->size(), oldestAge, numRecent,
    1e3 * std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
  return true;
}

}  // namespace mola

#endif
