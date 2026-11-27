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
 * @file   WallUpReader.cpp
 * @brief  Up direction of a trajectory frame from the plumb walls around it
 * @author Jose Luis Blanco Claraco
 */

#include <mola_lidar_odometry/WallUpReader.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>

#if __has_include(<mp2p_icp_filters/estimate_up_from_normals.h>)
#include <mp2p_icp_filters/estimate_up_from_normals.h>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <unordered_set>

using namespace mola;

namespace
{
// One key per voxel; coordinates beyond +-2^20 voxels share keys, which only
// thins the decimation there.
std::uint64_t voxel_key(float x, float y, float z, double voxel)
{
  constexpr std::int64_t OFFSET = 1 << 20;
  constexpr std::uint64_t MASK = (1U << 21) - 1;
  const auto ix =
    static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(x / voxel)) + OFFSET);
  const auto iy =
    static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(y / voxel)) + OFFSET);
  const auto iz =
    static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(z / voxel)) + OFFSET);
  return (ix & MASK) | ((iy & MASK) << 21) | ((iz & MASK) << 42);
}
}  // namespace

bool WallUpReader::checks_walls_against_floors()
{
#if defined(MP2P_ICP_HAS_UP_REFERENCES_CHECK)
  return true;
#else
  return false;
#endif
}

WallUpReader::WallUpReader(const Parameters & p) : params_(p)
{
#if !defined(MP2P_ICP_HAS_ESTIMATE_UP_COVARIANCE)
  THROW_EXCEPTION(
    "WallUpReader needs an mp2p_icp with the up covariance API "
    "(MP2P_ICP_HAS_ESTIMATE_UP_COVARIANCE)");
#endif
  ASSERT_GT_(params_.window, 0.0);
  ASSERT_GT_(params_.period, 0.0);
  ASSERT_GT_(params_.voxel, 0.0);
  ASSERT_GT_(params_.max_range, 0.0);
  ASSERT_GT_(params_.normal_radius, 0.0);
  ASSERT_GE_(params_.flats_weight, 0.0);
  ASSERT_GE_(params_.max_walls_floors_angle, 0.0);
  // Without cells the covariance falls back to per-normal independence,
  // which is not reliable: every reading would be dropped.
  ASSERTMSG_(
    params_.cluster_size > 0,
    "WallUpReader: cluster_size must be > 0 (0 gives no reliable covariance, so no reading)");
}

void WallUpReader::add_cloud(
  double t, const mrpt::poses::CPose3D & vehicle_pose,
  const mrpt::maps::CPointsMap::ConstPtr & cloud)
{
  if (!cloud) {
    return;
  }
  stats_.clouds++;
  Batch b;
  b.t = t;
  b.pose = vehicle_pose;
  b.cloud = cloud;
  batches_.push_back(std::move(b));
  new_since_read_ = true;
  while (!batches_.empty() && batches_.front().t < t - params_.window) {
    batches_.pop_front();
  }
}

void WallUpReader::decimate(Batch & b) const
{
  const auto & xs = b.cloud->getPointsBufferRef_x();
  const auto & ys = b.cloud->getPointsBufferRef_y();
  const auto & zs = b.cloud->getPointsBufferRef_z();
  const double r2max = params_.max_range * params_.max_range;
  std::unordered_set<std::uint64_t> seen;
  for (std::size_t i = 0; i < xs.size(); i++) {
    const double r2 = static_cast<double>(xs[i]) * xs[i] + static_cast<double>(ys[i]) * ys[i] +
                      static_cast<double>(zs[i]) * zs[i];
    // Written so that NaN points are dropped too:
    if (!(r2 <= r2max)) {
      continue;
    }
    const auto p = b.pose.composePoint(mrpt::math::TPoint3D(xs[i], ys[i], zs[i]));
    const mrpt::math::TPoint3Df pf(
      static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
    if (seen.insert(voxel_key(pf.x, pf.y, pf.z, params_.voxel)).second) {
      b.pts.push_back(pf);
    }
  }
  b.cloud.reset();
}

std::optional<WallUpReader::Reading> WallUpReader::read(double t)
{
  if (last_read_ && t - *last_read_ < params_.period) {
    return {};
  }
  last_read_ = t;
  if (!new_since_read_) {
    // The same scans as the previous reading: nothing new to say. The period
    // is spent all the same, so this counts once per period.
    stats_.no_new_scans++;
    return {};
  }
  new_since_read_ = false;

  while (!batches_.empty() && batches_.front().t < t - params_.window) {
    batches_.pop_front();
  }

  const auto t0 = std::chrono::steady_clock::now();
  for (auto & b : batches_) {
    if (b.cloud) {
      decimate(b);
    }
  }
  const auto account = [&]() {
    const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    stats_.cost_total += dt;
    stats_.cost_max = std::max(stats_.cost_max, dt);
    return dt;
  };

  // The window's scans, decimated again together (they overlap):
  mrpt::maps::CSimplePointsMap pc;
  std::unordered_set<std::uint64_t> seen;
  for (const auto & b : batches_) {
    for (const auto & p : b.pts) {
      if (seen.insert(voxel_key(p.x, p.y, p.z, params_.voxel)).second) {
        pc.insertPointFast(p.x, p.y, p.z);
      }
    }
  }
  if (pc.size() < params_.min_points) {
    stats_.too_few_points++;
    account();
    return {};
  }

#if defined(MP2P_ICP_HAS_ESTIMATE_UP_COVARIANCE)
  mp2p_icp_filters::PlanarNormalsParams np;
  np.search_radius = params_.normal_radius;
  std::vector<mrpt::math::TPoint3Df> pts;
  const auto normals = mp2p_icp_filters::estimate_planar_normals(pc, np, &pts);

  mp2p_icp_filters::EstimateUpParams up;
  up.flats_weight = params_.flats_weight;
  up.cluster_size = params_.cluster_size;
#if defined(MP2P_ICP_HAS_UP_REFERENCES_CHECK)
  up.max_walls_flats_angle_deg = params_.max_walls_floors_angle;
#endif
  mp2p_icp_filters::EstimateUpResult res;
  try {
    res = mp2p_icp_filters::estimate_up_from_normals(normals, up, &pts);
  } catch (const std::exception &) {
    // Not observable from this window (or too few normals): no reading.
    stats_.unobservable++;
    account();
    return {};
  }

#if defined(MP2P_ICP_HAS_UP_REFERENCES_CHECK)
  if (res.references_disagree) {
    // Floors on a slope or walls that lean (contradicted), or walls that
    // cannot tell (unconfirmed): neither is a vertical reference here.
    if (res.walls_contradict_flats) {
      stats_.floors_contradicted++;
    } else {
      stats_.floors_unconfirmed++;
    }
    account();
    return {};
  }
#endif

#if defined(MP2P_ICP_HAS_ESTIMATE_UP_COVARIANCE)
  if (!res.cov_reliable) {
    // Too few independent groups of normals: the covariance is not to be
    // trusted, and neither is a filter weighting by it.
    stats_.unreliable_cov++;
    account();
    return {};
  }
#endif

  Reading r;
  r.t = t;
  r.up = res.up;
  // Consecutive windows share most of their scans: count them once.
  const double overlap = std::max(1.0, params_.window / params_.period);
  r.up_cov = res.up_cov;
  r.up_cov *= overlap;
  r.points = pc.size();
  r.walls = res.wall_count;
  r.flats = res.flat_count;
  r.cost = account();
  stats_.readings++;
  return r;
#else
  account();
  return {};
#endif
}
