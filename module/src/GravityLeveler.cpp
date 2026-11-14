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
 * @file   GravityLeveler.cpp
 * @brief  Causal gravity leveling of an output trajectory
 * @author Jose Luis Blanco Claraco
 */

#include <mola_lidar_odometry/GravityLeveler.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/poses/Lie/SO.h>

#include <cmath>

using namespace mola;

namespace
{
// 99 % quantile of a chi-squared distribution with 2 degrees of freedom
constexpr double CHI2_2DOF_99 = 9.21;

// Pure rotation whose effect on a horizontal displacement (dx, dy, 0) is a
// height change tau . (dx, dy): rotation vector (tau_y, -tau_x, 0).
mrpt::poses::CPose3D leveling_rotation(const mrpt::math::TVector2D & tau)
{
  mrpt::math::CVectorFixedDouble<3> w;
  w[0] = tau.y;
  w[1] = -tau.x;
  w[2] = 0;
  return mrpt::poses::CPose3D(
    mrpt::poses::Lie::SO<3>::exp(w), mrpt::math::CVectorFixedDouble<3>::Zero());
}

mrpt::math::CVectorFixedDouble<3> as_vector(const mrpt::math::TVector3D & v)
{
  mrpt::math::CVectorFixedDouble<3> r;
  r[0] = v.x;
  r[1] = v.y;
  r[2] = v.z;
  return r;
}
}  // namespace

void GravityLeveler::add_imu(double t, const mrpt::math::TVector3D & specific_force)
{
  pending_imu_.push_back({t, specific_force});
}

std::optional<mrpt::math::TVector2D> GravityLeveler::tilt() const
{
  if (!filter_started_) {
    return {};
  }
  return mrpt::math::TVector2D(x_[0], x_[1]);
}

mrpt::poses::CPose3D GravityLeveler::add_pose(double t, const mrpt::poses::CPose3D & pose)
{
  if (!t0_) {
    t0_ = t;
  }

  // IMU readings up to this pose, into the trajectory frame:
  while (!pending_imu_.empty() && pending_imu_.front().t <= t) {
    const auto & s = pending_imu_.front();
    window_imu_.push_back({s.t, pose.rotateVector(s.f)});
    pending_imu_.pop_front();
  }
  while (!window_imu_.empty() && window_imu_.front().t < t - params_.window) {
    window_imu_.pop_front();
  }

  // The trajectory's own velocity, from the pose about velocity_span ago:
  PoseSample ps;
  ps.t = t;
  ps.p = pose.translation();
  if (!poses_.empty()) {
    const PoseSample * ref = &poses_.front();
    for (const auto & q : poses_) {
      if (q.t <= t - params_.velocity_span) {
        ref = &q;
      } else {
        break;
      }
    }
    const double dt = t - ref->t;
    if (dt > 0) {
      ps.v = (ps.p - ref->p) * (1.0 / dt);
    }
  }
  poses_.push_back(ps);
  const double keep = params_.window + params_.velocity_span + 1.0;
  while (poses_.size() > 2 && poses_.front().t < t - keep) {
    poses_.pop_front();
  }

  if (!next_epoch_) {
    next_epoch_ = *t0_ + params_.window;
  }
  if (t >= *next_epoch_) {
    next_epoch_ = t + params_.epoch_period;
    update_filter(t, pose);
  }

  // The part of the tilt beyond the deadband:
  mrpt::math::TVector2D tau(0, 0);
  if (filter_started_) {
    const double mag = std::hypot(x_[0], x_[1]);
    const double d = mrpt::DEG2RAD(params_.deadband);
    if (mag > d) {
      const double s = (mag - d) / mag;
      tau = mrpt::math::TVector2D(x_[0] * s, x_[1] * s);
    }
  }

  // Each increment is rotated by the correction held at its start, and the
  // attitude by the current one:
  const auto R_prev = leveling_rotation(applied_tilt_);
  const auto R_now = leveling_rotation(tau);

  const auto R_att = R_now.getRotationMatrix() * pose.getRotationMatrix();
  mrpt::math::TVector3D p = pose.translation();
  if (last_in_ && last_out_) {
    p =
      last_out_->translation() + R_prev.rotateVector(pose.translation() - last_in_->translation());
  }
  const mrpt::poses::CPose3D out(R_att, as_vector(p));

  applied_tilt_ = tau;
  last_in_ = pose;
  last_out_ = out;
  return out;
}

void GravityLeveler::update_filter(double t, const mrpt::poses::CPose3D & pose)
{
  if (window_imu_.size() < params_.min_readings_per_window || poses_.size() < 2) {
    return;
  }

  mrpt::math::TVector3D f_mean(0, 0, 0);
  for (const auto & s : window_imu_) {
    f_mean += s.f;
  }
  f_mean *= 1.0 / static_cast<double>(window_imu_.size());

  // The trajectory's own mean acceleration over the window:
  const PoseSample * first = &poses_.back();
  for (const auto & q : poses_) {
    if (q.t >= t - params_.window) {
      first = &q;
      break;
    }
  }
  mrpt::math::TVector3D a_mean(0, 0, 0);
  const double dt_win = t - first->t;
  if (dt_win > 0) {
    a_mean = (poses_.back().v - first->v) * (1.0 / dt_win);
  }

  const auto g = f_mean - a_mean;
  if (std::abs(g.z) < 0.5 * params_.gravity) {
    return;
  }
  Eigen::Vector2d z;
  z << g.x / g.z, g.y / g.z;

  const double q = mrpt::DEG2RAD(params_.tilt_random_walk);
  const double r = mrpt::DEG2RAD(params_.reading_sigma);

  if (!filter_started_) {
    P_.setZero();
    const double s0 = mrpt::DEG2RAD(params_.initial_tilt_sigma);
    const double b0 = params_.initial_acc_error_sigma;
    P_(0, 0) = s0 * s0;
    P_(1, 1) = s0 * s0;
    for (int i = 2; i < 5; i++) {
      P_(i, i) = b0 * b0;
    }
    x_.setZero();
    x_[0] = z[0];
    x_[1] = z[1];
  } else if (last_epoch_) {
    const double dt = t - *last_epoch_;
    P_(0, 0) += q * q * dt;
    P_(1, 1) += q * q * dt;
  }
  last_epoch_ = t;

  // tau_reading = tau + (R b)_xy / g
  Eigen::Matrix<double, 2, 5> H = Eigen::Matrix<double, 2, 5>::Zero();
  H(0, 0) = 1;
  H(1, 1) = 1;
  const auto R = pose.getRotationMatrix();
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 3; j++) {
      H(i, 2 + j) = R(i, j) / params_.gravity;
    }
  }

  const Eigen::Vector2d y = z - H * x_;
  const Eigen::Matrix2d S = H * P_ * H.transpose() + r * r * Eigen::Matrix2d::Identity();

  // A window that does not measure gravity must not enter the correction:
  if (filter_started_ && y.dot(S.ldlt().solve(y)) > CHI2_2DOF_99) {
    return;
  }
  filter_started_ = true;

  const Eigen::Matrix<double, 5, 2> K = P_ * H.transpose() * S.inverse();
  x_ += K * y;
  P_ = (Eigen::Matrix<double, 5, 5>::Identity() - K * H) * P_;
}
