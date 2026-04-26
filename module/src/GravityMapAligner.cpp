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

#include <mola_lidar_odometry/GravityMapAligner.h>

#include <algorithm>
#include <cmath>

namespace
{
mrpt::math::TVector3D robust_weighted_mean(
  const std::vector<mrpt::math::TVector3D> & xs, const std::vector<double> & seed_w,
  double huber_thr, std::size_t iters)
{
  if (xs.empty()) {
    return {0, 0, 0};
  }
  std::vector<double> w = seed_w;
  mrpt::math::TVector3D mean{0, 0, 0};

  auto weighted_mean = [&]() {
    mrpt::math::TVector3D acc{0, 0, 0};
    double wsum = 0;
    for (std::size_t i = 0; i < xs.size(); ++i) {
      acc += w[i] * xs[i];
      wsum += w[i];
    }
    if (wsum <= 0) {
      return mrpt::math::TVector3D{0, 0, 0};
    }
    return acc / wsum;
  };

  mean = weighted_mean();
  for (std::size_t it = 0; it < iters; ++it) {
    for (std::size_t i = 0; i < xs.size(); ++i) {
      const mrpt::math::TVector3D d = xs[i] - mean;
      const double r = d.norm();
      const double hw = (r <= huber_thr) ? 1.0 : (huber_thr / r);
      w[i] = seed_w[i] * hw;
    }
    mean = weighted_mean();
  }
  return mean;
}

}  // namespace

namespace mola
{
void GravityMapAligner::onNewKeyframe(
  KeyFrameID id, const mola::imu::LocalVelocityBuffer::SamplesByTime & window)
{
  const auto & a_map = window.a_b;
  if (a_map.empty()) {
    return;
  }

  mrpt::math::TVector3D mean{0, 0, 0};
  for (const auto & [t, a] : a_map) {
    mean += a;
  }
  const auto n = static_cast<double>(a_map.size());
  mean *= 1.0 / n;

  // Intra-KF std of |a|:
  double sum_var = 0;
  for (const auto & [t, a] : a_map) {
    const double d = a.norm() - mean.norm();
    sum_var += d * d;
  }
  constexpr double std_floor = 0.01;  // ~1e-4 variance floor (m/s²)
  const double stddev = std::max(std_floor, (a_map.size() >= 2) ? std::sqrt(sum_var / n) : 0.0);

  KFAccSample s;
  s.a_body = mean;
  s.a_body_std = stddev;
  s.n_samples = a_map.size();
  samples_[id] = s;
}

void GravityMapAligner::onNewKeyframeRaw(KeyFrameID id, const KFAccSample & s) { samples_[id] = s; }

void GravityMapAligner::onKeyframeDropped(KeyFrameID id) { samples_.erase(id); }

void GravityMapAligner::clear() { samples_.clear(); }

mrpt::poses::CPose3D GravityMapAligner::rotationFromGravity(
  const mrpt::math::TVector3D & g_hat, double axis_eps)
{
  const double gn = g_hat.norm();
  if (gn < axis_eps) {
    return mrpt::poses::CPose3D::Identity();
  }

  // Normalized gravity direction in the current (odom) frame.
  const double gx = g_hat.x / gn;
  const double gy = g_hat.y / gn;
  const double gz = g_hat.z / gn;

  // Already aligned with +Z :  nothing to do.
  if (std::abs(gz - 1.0) < axis_eps) {
    return mrpt::poses::CPose3D::Identity();
  }

  // Decompose R = Ry(pitch) * Rx(roll), yaw = 0, such that R * g_hat = +Z.
  //   Rx(roll)  zeroes the Y component:   roll  = atan2(gy, gz)
  //   Ry(pitch) zeroes the X component:   pitch = atan2(-gx, sqrt(gy²+gz²))
  // By placing yaw=0 explicitly in the CPose3D constructor (ZYX convention),
  // getYawPitchRoll() returns exactly zero yaw :  no floating-point accumulation
  // from quaternion conversion.
  const double roll = std::atan2(gy, gz);
  const double pitch = std::atan2(-gx, std::sqrt(gy * gy + gz * gz));

  return {0.0, 0.0, 0.0, 0.0 /*yaw*/, pitch, roll};
}

std::optional<mrpt::math::TVector3D> GravityMapAligner::estimateGravityVector(
  const std::map<KeyFrameID, mrpt::poses::CPose3D> & kf_poses) const
{
  std::vector<KeyFrameID> all_ids;
  all_ids.reserve(samples_.size());
  for (const auto & [id, s] : samples_) {
    all_ids.push_back(id);
  }
  return estimateGravityVectorOverWindow(kf_poses, all_ids, params_.min_keyframes);
}

std::optional<mrpt::math::TVector3D> GravityMapAligner::estimateGravityVectorOverWindow(
  const std::map<KeyFrameID, mrpt::poses::CPose3D> & kf_poses,
  const std::vector<KeyFrameID> & id_window, std::size_t min_pool) const
{
  std::vector<mrpt::math::TVector3D> gs;
  std::vector<double> ws;
  gs.reserve(id_window.size());
  ws.reserve(id_window.size());

  for (KeyFrameID id : id_window) {
    auto its = samples_.find(id);
    if (its == samples_.end()) {
      continue;
    }
    auto itp = kf_poses.find(id);
    if (itp == kf_poses.end()) {
      continue;
    }

    // Rotate body-frame accel into {odom}: g_odom = R_i_odom · a_body_i
    const auto & R = itp->second;  // CPose3D; we use its rotation
    const mrpt::math::TVector3D g_odom = R.rotateVector(its->second.a_body);
    gs.push_back(g_odom);

    // Seed weight: 1 / (a_body_std² + small). Down-weight very dynamic KFs:
    const double std_i = its->second.a_body_std;
    double w = 1.0 / std::max(1e-6, std_i * std_i);
    if (std_i > params_.max_kf_acc_std_mps2) {
      w *= 1e-3;  // heavy down-weight
    }
    ws.push_back(w);
  }

  if (gs.size() < std::max<std::size_t>(2, min_pool)) {
    return std::nullopt;
  }

  return robust_weighted_mean(gs, ws, params_.huber_threshold_mps2, params_.irls_iterations);
}

std::optional<mrpt::poses::CPose3D> GravityMapAligner::estimateGlobalCorrection(
  const std::map<KeyFrameID, mrpt::poses::CPose3D> & kf_poses) const
{
  auto g = estimateGravityVector(kf_poses);
  if (!g) {
    return std::nullopt;
  }
  return rotationFromGravity(*g, params_.axis_eps);
}

std::map<GravityMapAligner::KeyFrameID, mrpt::poses::CPose3D>
GravityMapAligner::estimatePerKFCorrection(
  const std::map<KeyFrameID, mrpt::poses::CPose3D> & kf_poses, std::size_t window_half_width,
  std::size_t min_pool_per_kf) const
{
  std::map<KeyFrameID, mrpt::poses::CPose3D> out;
  if (samples_.empty() || kf_poses.empty()) {
    return out;
  }

  // Ordered id list from kf_poses (std::map is already sorted):
  std::vector<KeyFrameID> pose_ids;
  pose_ids.reserve(kf_poses.size());
  for (const auto & [id, p] : kf_poses) {
    pose_ids.push_back(id);
  }

  for (std::size_t k = 0; k < pose_ids.size(); ++k) {
    const KeyFrameID id_k = pose_ids[k];
    if (samples_.count(id_k) == 0) {
      continue;
    }

    const std::size_t lo = (k > window_half_width) ? (k - window_half_width) : 0;
    const std::size_t hi = std::min(pose_ids.size(), k + window_half_width + 1);
    std::vector<KeyFrameID> win(pose_ids.begin() + lo, pose_ids.begin() + hi);

    auto g = estimateGravityVectorOverWindow(kf_poses, win, min_pool_per_kf);
    if (!g) {
      continue;
    }
    out.emplace(id_k, rotationFromGravity(*g, params_.axis_eps));
  }
  return out;
}

}  // namespace mola
