/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along
 * with MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */
/**
 * @file   LidarOdometry_SlidingWindow.cpp
 * @brief  Sliding-window map insertion: refine a keyframe against the ones
 *         that came after it, and only then merge it into the local map.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_lidar_odometry/LidarOdometry.h>
#include <mola_yaml/yaml_helpers.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/system/filesystem.h>

#if defined(MOLA_LO_HAS_GTSAM)
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <mrpt/poses/gtsam_wrappers.h>
#endif

#include <algorithm>
#include <fstream>

using namespace mola;

namespace
{
/// Translation and rotation magnitude of a pose increment, for the logs.
std::pair<double, double> poseDelta(const mrpt::poses::CPose3D & a, const mrpt::poses::CPose3D & b)
{
  const auto d = b - a;
  const double dxyz = d.translation().norm();
  const double drot = mrpt::RAD2DEG(mrpt::poses::Lie::SO<3>::log(d.getRotationMatrix()).norm());
  return {dxyz, drot};
}

/// Appends one TUM line. The probe writes two of these files per run and the
/// whole point is to score them against each other.
void appendTum(
  std::ofstream & f, const mrpt::Clock::time_point & stamp, const mrpt::poses::CPose3D & p)
{
  if (!f.is_open()) {
    return;
  }
  const auto q = p.asTPose();
  mrpt::math::CQuaternionDouble quat;
  p.getAsQuaternion(quat);
  f << mrpt::format(
         "%.06f %.06f %.06f %.06f %.06f %.06f %.06f %.06f", mrpt::Clock::toDouble(stamp), q.x, q.y,
         q.z, quat.x(), quat.y(), quat.z(), quat.r())
    << "\n";
}
}  // namespace

void LidarOdometry::Parameters::SlidingWindowOptions::initialize(
  const Yaml & cfg, [[maybe_unused]] Parameters & parent)
{
  YAML_LOAD_OPT(enabled, bool);
  YAML_LOAD_OPT(window_size, uint32_t);
  YAML_LOAD_OPT(links_per_keyframe, uint32_t);
  YAML_LOAD_OPT(max_lag_seconds, double);
  YAML_LOAD_OPT(sigma_map_xyz, double);
  YAML_LOAD_OPT(sigma_map_rot_deg, double);
  YAML_LOAD_OPT(sigma_s2s_xyz, double);
  YAML_LOAD_OPT(sigma_s2s_rot_deg, double);
  YAML_LOAD_OPT(correct_translation, bool);
  YAML_LOAD_OPT(correct_yaw, bool);
  YAML_LOAD_OPT(solve_graph, bool);
  YAML_LOAD_OPT(min_s2s_quality, double);
  YAML_LOAD_OPT(max_link_disagreement_xyz, double);
  YAML_LOAD_OPT(max_link_disagreement_rot_deg, double);
  YAML_LOAD_OPT(s2s_huber_k, double);
  YAML_LOAD_OPT(max_correction_xyz, double);
  YAML_LOAD_OPT(max_correction_rot_deg, double);
  YAML_LOAD_OPT(window_layer, bool);
  YAML_LOAD_OPT(window_layer_name, std::string);
  YAML_LOAD_OPT(map_layer_name, std::string);
  YAML_LOAD_OPT(obs_layer_name, std::string);
  YAML_LOAD_OPT(update_trajectory, bool);
  YAML_LOAD_OPT(probe_dir, std::string);
}

// ---------------------------------------------------------------------------
//  Merging one window entry into the consolidated map
// ---------------------------------------------------------------------------
void LidarOdometry::slidingWindowMergeIntoMap(MethodState::SlidingWindowKF & kf)
{
  ASSERT_(kf.obs);
  ASSERT_(state_.local_map);

  const auto & sw = params_.sliding_window;

  ProfilerEntry tle(profiler_, "onLidar.4.update_local_map");

  auto lckMapContents = mrpt::lockHelper(local_map_content_mtx_);

  // Same four-step dance as the immediate path, except the pose the merge
  // pipeline is told about is the refined one instead of the live one.
  for (const auto & [lyName, lyMap] : kf.obs->layers) {
    if (state_.local_map->layers.count(lyName) != 0) {
      continue;  // e.g. the window layer, which lives in the local map itself
    }
    state_.local_map->layers[lyName] = lyMap;
  }

  {
    auto lckImu = mrpt::lockHelper(imu_state_mtx_);
    updatePipelineDynamicVariablesRobotPoseOnly(kf.pose);
    state_.parameter_source.realize();
  }

  mp2p_icp_filters::apply_filter_pipeline(state_.obs2map_merge, *state_.local_map, profiler_);

  for (const auto & [lyName, lyMap] : kf.obs->layers) {
    auto it = state_.local_map->layers.find(lyName);
    if (it != state_.local_map->layers.end() && it->second == lyMap) {
      state_.local_map->layers.erase(it);
    }
  }

  lckMapContents.unlock();

  state_.mark_local_map_as_updated();

  // Bookkeeping for the end-of-run summary and the probe files.
  const auto [dxyz, drot] = poseDelta(kf.live_pose, kf.pose);
  state_.swin_merged++;
  state_.swin_sum_corr_xyz += dxyz;
  state_.swin_sum_corr_rot_deg += drot;
  state_.swin_max_corr_xyz = std::max(state_.swin_max_corr_xyz, dxyz);
  state_.swin_max_corr_rot_deg = std::max(state_.swin_max_corr_rot_deg, drot);

  if (!sw.probe_dir.empty()) {
    appendTum(swin_probe_live_, kf.stamp, kf.live_pose);
    appendTum(swin_probe_refined_, kf.stamp, kf.pose);
  }

  MRPT_LOG_DEBUG_FMT(
    "[swin] merged kf #%lu: correction %.4f m / %.4f deg", static_cast<unsigned long>(kf.id), dxyz,
    drot);
}

// ---------------------------------------------------------------------------
//  The window map layer
// ---------------------------------------------------------------------------
void LidarOdometry::slidingWindowRebuildWindowLayer()
{
  const auto & sw = params_.sliding_window;
  if (!sw.window_layer) {
    return;
  }
  ASSERT_(state_.local_map);

  ProfilerEntry tle(profiler_, "onLidar.4.swin_window_layer");

  auto lckMapContents = mrpt::lockHelper(local_map_content_mtx_);

  auto itWin = state_.local_map->layers.find(sw.window_layer_name);
  ASSERTMSG_(
    itWin != state_.local_map->layers.end(),
    "sliding_window.window_layer is enabled but the pipeline's local map has "
    "no layer named '" +
      sw.window_layer_name + "'");

  auto itMap = state_.local_map->layers.find(sw.map_layer_name);
  ASSERT_(itMap != state_.local_map->layers.end());

  itWin->second->clear();

  // Reuse the shipped merge pipeline by temporarily letting the window layer
  // answer to the consolidated layer's name: the pipeline hardcodes the
  // target, and a second copy of it in every YAML is not worth the drift.
  auto realMapLayer = itMap->second;
  auto winLayer = itWin->second;
  itMap->second = winLayer;

  for (auto & kf : state_.swin_kfs) {
    if (kf.committed) {
      continue;
    }
    for (const auto & [lyName, lyMap] : kf.obs->layers) {
      if (state_.local_map->layers.count(lyName) != 0) {
        continue;
      }
      state_.local_map->layers[lyName] = lyMap;
    }
    {
      auto lckImu = mrpt::lockHelper(imu_state_mtx_);
      updatePipelineDynamicVariablesRobotPoseOnly(kf.pose);
      state_.parameter_source.realize();
    }
    mp2p_icp_filters::apply_filter_pipeline(state_.obs2map_merge, *state_.local_map, profiler_);
    for (const auto & [lyName, lyMap] : kf.obs->layers) {
      auto it = state_.local_map->layers.find(lyName);
      if (it != state_.local_map->layers.end() && it->second == lyMap) {
        state_.local_map->layers.erase(it);
      }
    }
  }

  state_.local_map->layers[sw.map_layer_name] = realMapLayer;
  state_.local_map->layers[sw.window_layer_name] = winLayer;
}

// ---------------------------------------------------------------------------
//  The graph
// ---------------------------------------------------------------------------
void LidarOdometry::slidingWindowSolve()
{
  const auto & sw = params_.sliding_window;
  if (!sw.solve_graph || state_.swin_kfs.size() < 2) {
    return;
  }

#if !defined(MOLA_LO_HAS_GTSAM)
  THROW_EXCEPTION("sliding_window.solve_graph requires mola_lidar_odometry to be built with GTSAM");
#else
  ProfilerEntry tle(profiler_, "onLidar.4.swin_solve");

  using gtsam::symbol_shorthand::X;
  using mrpt::gtsam_wrappers::toPose3;
  using mrpt::gtsam_wrappers::toTPose3D;

  // GTSAM's Pose3 tangent order is [rot(3), trans(3)].
  const auto sigmasMap =
    (gtsam::Vector(6) << mrpt::DEG2RAD(sw.sigma_map_rot_deg), mrpt::DEG2RAD(sw.sigma_map_rot_deg),
     mrpt::DEG2RAD(sw.sigma_map_rot_deg), sw.sigma_map_xyz, sw.sigma_map_xyz, sw.sigma_map_xyz)
      .finished();
  const auto sigmasS2S =
    (gtsam::Vector(6) << mrpt::DEG2RAD(sw.sigma_s2s_rot_deg), mrpt::DEG2RAD(sw.sigma_s2s_rot_deg),
     mrpt::DEG2RAD(sw.sigma_s2s_rot_deg), sw.sigma_s2s_xyz, sw.sigma_s2s_xyz, sw.sigma_s2s_xyz)
      .finished();
  const auto sigmasFixed = (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished();

  const auto noiseMap = gtsam::noiseModel::Diagonal::Sigmas(sigmasMap);
  gtsam::SharedNoiseModel noiseS2S = gtsam::noiseModel::Diagonal::Sigmas(sigmasS2S);
  if (sw.s2s_huber_k > 0) {
    noiseS2S = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(sw.s2s_huber_k), noiseS2S);
  }
  const auto noiseFixed = gtsam::noiseModel::Diagonal::Sigmas(sigmasFixed);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;

  std::map<uint64_t, const MethodState::SlidingWindowKF *> byId;
  for (const auto & kf : state_.swin_kfs) {
    byId[kf.id] = &kf;
    initial.insert(X(kf.id), toPose3(kf.pose));
    // An already-merged keyframe is map geometry now: it is the gauge, not a
    // variable. The still-free ones are pulled toward what scan-to-map said.
    graph.addPrior(
      X(kf.id), toPose3(kf.committed ? kf.pose : kf.live_pose),
      kf.committed ? noiseFixed : noiseMap);
  }

  size_t nLinks = 0;
  for (const auto & lk : state_.swin_links) {
    if (byId.count(lk.from_id) == 0 || byId.count(lk.to_id) == 0) {
      continue;
    }
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      X(lk.from_id), X(lk.to_id), toPose3(lk.rel), noiseS2S);
    nLinks++;
  }
  if (nLinks == 0) {
    return;
  }

  gtsam::LevenbergMarquardtParams lmp;
  lmp.setMaxIterations(20);
  lmp.setRelativeErrorTol(1e-8);
  lmp.setAbsoluteErrorTol(1e-10);

  gtsam::Values result;
  try {
    result = gtsam::LevenbergMarquardtOptimizer(graph, initial, lmp).optimize();
  } catch (const std::exception & e) {
    MRPT_LOG_WARN_STREAM("[swin] graph solve failed, keeping live poses: " << e.what());
    return;
  }

  for (auto & kf : state_.swin_kfs) {
    if (kf.committed) {
      continue;
    }
    auto p = toTPose3D(result.at<gtsam::Pose3>(X(kf.id)));
    const auto live = kf.live_pose.asTPose();
    if (!sw.correct_translation) {
      p.x = live.x;
      p.y = live.y;
      p.z = live.z;
    }
    if (!sw.correct_yaw) {
      p.yaw = live.yaw;
    }
    const auto cand = mrpt::poses::CPose3D(p);
    const auto [cxyz, crot] = poseDelta(kf.live_pose, cand);
    if (
      (sw.max_correction_xyz > 0 && cxyz > sw.max_correction_xyz) ||
      (sw.max_correction_rot_deg > 0 && crot > sw.max_correction_rot_deg)) {
      MRPT_LOG_WARN_FMT(
        "[swin] rejecting an implausible correction on kf #%lu: %.4f m / %.4f deg",
        static_cast<unsigned long>(kf.id), cxyz, crot);
      kf.pose = kf.live_pose;
      continue;
    }
    kf.pose = cand;
  }
#endif
}

// ---------------------------------------------------------------------------
//  The per-keyframe entry point
// ---------------------------------------------------------------------------
void LidarOdometry::slidingWindowOnKeyframe(
  const mp2p_icp::metric_map_t::Ptr & observation, const mrpt::poses::CPose3D & livePose,
  const mrpt::Clock::time_point & stamp)
{
  const auto & sw = params_.sliding_window;
  if (!sw.enabled) {
    return;
  }
  ASSERT_(observation);

  ProfilerEntry tleAll(profiler_, "onLidar.4.swin");

  // Open the probe files on first use.
  if (!sw.probe_dir.empty() && !swin_probe_live_.is_open()) {
    mrpt::system::createDirectory(sw.probe_dir);
    ASSERTMSG_(
      mrpt::system::directoryExists(sw.probe_dir),
      "sliding_window.probe_dir does not exist and could not be created: " + sw.probe_dir);
    swin_probe_live_.open(sw.probe_dir + "/swin_live.tum");
    swin_probe_refined_.open(sw.probe_dir + "/swin_refined.tum");
  }

  // 1) Push the new keyframe.
  MethodState::SlidingWindowKF kf;
  kf.id = state_.swin_next_id++;
  kf.stamp = stamp;
  kf.live_pose = livePose;
  kf.pose = livePose;
  // Keep only the layers this feature needs, in the scan's own frame: the raw
  // and deskewed clouds are the bulk of the memory and nothing here reads
  // them.
  kf.obs = mp2p_icp::metric_map_t::Create();
  for (const auto & [lyName, lyMap] : observation->layers) {
    if (lyName == "raw" || lyName == "deskewed" || lyName == "raw_filtered" || lyName == "viz") {
      continue;
    }
    kf.obs->layers[lyName] = lyMap;
  }
  state_.swin_kfs.push_back(kf);

  // 2) Link it to its predecessors by direct scan-to-scan registration. This
  //    is the only information in the whole scheme that scan-to-map does not
  //    already have.
  if (sw.solve_graph && sw.links_per_keyframe > 0 && state_.swin_kfs.size() >= 2) {
    ProfilerEntry tle(profiler_, "onLidar.4.swin_s2s");

    auto & icpCase = params_.icp.at(AlignKind::RegularOdometry);
    // The odometry loop leaves an iteration hook behind that captures its own
    // stack; never call align() with it installed from anywhere else.
    icpCase.icp->setIterationHook({});

    auto icpParams = icpCase.icp_parameters;
    icpParams.functor_should_generate_debug_file = {};

    const size_t nNew = state_.swin_kfs.size() - 1;
    const auto & cur = state_.swin_kfs[nNew];

    // An entry whose points are already in the map no longer carries them, so
    // it cannot be registered against: walk back over the ones that still can.
    size_t nDone = 0;
    for (size_t k = 1; k <= nNew && nDone < sw.links_per_keyframe; k++) {
      const auto & prev = state_.swin_kfs[nNew - k];
      if (!prev.obs || !cur.obs) {
        continue;
      }

      auto itPrevObs = prev.obs->layers.find(sw.obs_layer_name);
      auto itCurObs = cur.obs->layers.find(sw.obs_layer_name);
      if (itPrevObs == prev.obs->layers.end() || itCurObs == cur.obs->layers.end()) {
        continue;
      }
      nDone++;

      // Register the new scan against the older one, both in their own local
      // frames, so the result is a pure relative constraint.
      mp2p_icp::metric_map_t global;
      global.layers[sw.map_layer_name] = itPrevObs->second;
      if (sw.window_layer) {
        // The pipeline's matcher lists the window layer too, and it throws on
        // a layer it cannot find. Pointing both names at the same cloud pairs
        // every point twice, which scales the whole normal-equation system and
        // the paired ratio by two and so leaves both the solution and the
        // quality exactly where they were.
        global.layers[sw.window_layer_name] = itPrevObs->second;
      }
      mp2p_icp::metric_map_t local;
      local.layers[sw.obs_layer_name] = itCurObs->second;

      const mrpt::poses::CPose3D guess = cur.pose - prev.pose;  // cur seen from prev

      mp2p_icp::Results res;
      try {
        icpCase.icp->align(local, global, guess.asTPose(), icpParams, res);
      } catch (const std::exception & e) {
        MRPT_LOG_THROTTLE_WARN_STREAM(5.0, "[swin] scan-to-scan ICP failed: " << e.what());
        continue;
      }

      if (res.quality < sw.min_s2s_quality) {
        state_.swin_links_dropped++;
        MRPT_LOG_DEBUG_FMT(
          "[swin] dropping s2s link %lu->%lu, quality %.3f", static_cast<unsigned long>(prev.id),
          static_cast<unsigned long>(cur.id), res.quality);
        continue;
      }
      const auto [ddxyz, ddrot] = poseDelta(guess, mrpt::poses::CPose3D(res.optimal_tf.mean));
      if (
        (sw.max_link_disagreement_xyz > 0 && ddxyz > sw.max_link_disagreement_xyz) ||
        (sw.max_link_disagreement_rot_deg > 0 && ddrot > sw.max_link_disagreement_rot_deg)) {
        state_.swin_links_dropped++;
        MRPT_LOG_DEBUG_FMT(
          "[swin] dropping diverged s2s link %lu->%lu: %.4f m / %.4f deg from the live poses",
          static_cast<unsigned long>(prev.id), static_cast<unsigned long>(cur.id), ddxyz, ddrot);
        continue;
      }
      state_.swin_links_kept++;
      state_.swin_sum_disagree_xyz += ddxyz;
      state_.swin_sum_disagree_rot_deg += ddrot;

      MethodState::SlidingWindowLink lk;
      lk.from_id = prev.id;
      lk.to_id = cur.id;
      lk.rel = mrpt::poses::CPose3D(res.optimal_tf.mean);
      state_.swin_links.push_back(lk);
    }
  }

  // 3) Re-solve the window.
  slidingWindowSolve();

  // 4) Consolidate whatever has aged out. `window_size` counts the entries
  //    that are still free to move; one already-merged entry is kept behind
  //    them as the gauge.
  const double now = mrpt::Clock::toDouble(stamp);
  for (;;) {
    size_t nFree = 0;
    for (const auto & e : state_.swin_kfs) {
      if (!e.committed) {
        nFree++;
      }
    }
    if (nFree == 0) {
      break;
    }
    // The oldest free entry:
    auto it = std::find_if(
      state_.swin_kfs.begin(), state_.swin_kfs.end(), [](const auto & e) { return !e.committed; });
    const double age = now - mrpt::Clock::toDouble(it->stamp);
    const bool tooMany = nFree > sw.window_size;
    const bool tooOld = sw.max_lag_seconds > 0 && age > sw.max_lag_seconds;
    if (!tooMany && !tooOld) {
      break;
    }
    if (sw.update_trajectory) {
      // Everything up to the next keyframe still in the window belongs to this
      // one; the newest keyframe is the upper bound when there is no other.
      auto itNext = std::next(it);
      const auto until =
        itNext != state_.swin_kfs.end() ? itNext->stamp : state_.swin_kfs.back().stamp;
      slidingWindowCorrectTrajectory(*it, until);
    }
    slidingWindowMergeIntoMap(*it);
    it->committed = true;
    // Its points stay around while it is the window's gauge, so the next
    // keyframe can still be registered directly against it; they go away with
    // the entry itself at step 5.
  }

  // 5) Drop everything older than the single gauge entry, and any link that
  //    referenced it.
  while (state_.swin_kfs.size() >= 2 && state_.swin_kfs[0].committed &&
         state_.swin_kfs[1].committed) {
    state_.swin_kfs.pop_front();
  }
  if (!state_.swin_kfs.empty()) {
    const uint64_t oldest = state_.swin_kfs.front().id;
    state_.swin_links.erase(
      std::remove_if(
        state_.swin_links.begin(), state_.swin_links.end(),
        [oldest](const auto & lk) { return lk.from_id < oldest || lk.to_id < oldest; }),
      state_.swin_links.end());
  }

  // 6) Refresh the optional window map layer at the new poses.
  slidingWindowRebuildWindowLayer();
}

void LidarOdometry::slidingWindowCorrectTrajectory(
  const MethodState::SlidingWindowKF & kf, const mrpt::Clock::time_point & until)
{
  // The keyframe's correction is a rigid motion in the world frame, and every
  // scan registered between this keyframe and the next one inherited its
  // error, so the same correction applies to all of them. Piecewise constant
  // rather than blended: the corrections are millimetres and change slowly, so
  // the step at a keyframe boundary is far below what it removes.
  const auto dT = kf.pose + (-kf.live_pose);

  auto lck = mrpt::lockHelper(state_trajectory_mtx_);

  const double t0 = mrpt::Clock::toDouble(kf.stamp);
  const double t1 = mrpt::Clock::toDouble(until);

  std::vector<std::pair<mrpt::Clock::time_point, mrpt::math::TPose3D>> fixed;
  for (const auto & [t, p] : state_.estimated_trajectory) {
    const double ts = mrpt::Clock::toDouble(t);
    if (ts < t0 || ts >= t1) {
      continue;
    }
    fixed.emplace_back(t, (dT + mrpt::poses::CPose3D(p)).asTPose());
  }
  for (const auto & [t, p] : fixed) {
    state_.estimated_trajectory.insert(t, p);
  }
}

void LidarOdometry::slidingWindowFlush()
{
  if (!params_.sliding_window.enabled) {
    return;
  }
  for (auto & kf : state_.swin_kfs) {
    if (kf.committed || !kf.obs) {
      continue;
    }
    slidingWindowMergeIntoMap(kf);
    kf.committed = true;
    kf.obs.reset();
  }
  if (state_.swin_merged > 0) {
    MRPT_LOG_INFO_FMT(
      "[swin] merged %lu keyframes; mean correction %.5f m / %.5f deg, max %.5f m / %.5f deg",
      static_cast<unsigned long>(state_.swin_merged), state_.swin_sum_corr_xyz / state_.swin_merged,
      state_.swin_sum_corr_rot_deg / state_.swin_merged, state_.swin_max_corr_xyz,
      state_.swin_max_corr_rot_deg);
  }
  if (state_.swin_links_kept + state_.swin_links_dropped > 0) {
    MRPT_LOG_INFO_FMT(
      "[swin] s2s links kept=%lu dropped=%lu; mean disagreement with the live "
      "poses %.5f m / %.5f deg",
      static_cast<unsigned long>(state_.swin_links_kept),
      static_cast<unsigned long>(state_.swin_links_dropped),
      state_.swin_links_kept ? state_.swin_sum_disagree_xyz / state_.swin_links_kept : 0.0,
      state_.swin_links_kept ? state_.swin_sum_disagree_rot_deg / state_.swin_links_kept : 0.0);
  }
  swin_probe_live_.close();
  swin_probe_refined_.close();
}
