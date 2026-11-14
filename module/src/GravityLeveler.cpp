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
#include <mrpt/core/exceptions.h>
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

// Attitude at fraction `s` in [0,1] between two attitudes (geodesic).
mrpt::math::CMatrixDouble33 interpolate_rotation(
  const mrpt::math::CMatrixDouble33 & R0, const mrpt::math::CMatrixDouble33 & R1, double s)
{
  const mrpt::math::CMatrixDouble33 dR(R0.asEigen().transpose() * R1.asEigen());
  auto w = mrpt::poses::Lie::SO<3>::log(dR);
  for (int i = 0; i < 3; i++) {
    w[i] *= s;
  }
  return mrpt::math::CMatrixDouble33(R0.asEigen() * mrpt::poses::Lie::SO<3>::exp(w).asEigen());
}
}  // namespace

GravityLeveler::GravityLeveler(const Parameters & p) : params_(p)
{
  ASSERT_GT_(params_.window, 0.0);
  ASSERT_GT_(params_.epoch_period, 0.0);
  ASSERT_GT_(params_.velocity_span, 0.0);
  ASSERT_GE_(params_.deadband, 0.0);
  ASSERT_GE_(params_.tilt_random_walk, 0.0);
  ASSERT_GT_(params_.reading_sigma, 0.0);
  ASSERT_GT_(params_.gravity, 0.0);
}

void GravityLeveler::add_imu(double t, const mrpt::math::TVector3D & specific_force)
{
  stats_.imu_readings++;
  pending_imu_.push_back({t, specific_force, {}});
  // Bounded while no pose arrives:
  while (!pending_imu_.empty() && pending_imu_.front().t < t - params_.window) {
    pending_imu_.pop_front();
  }
}

std::optional<mrpt::math::TVector2D> GravityLeveler::tilt() const
{
  if (!filter_started_) {
    return {};
  }
  return mrpt::math::TVector2D(x_[0], x_[1]);
}

void GravityLeveler::reset_filter()
{
  filter_started_ = false;
  consecutive_rejections_ = 0;
  last_epoch_.reset();
}

mrpt::poses::CPose3D GravityLeveler::add_pose(double t, const mrpt::poses::CPose3D & pose)
{
  const bool is_new = !last_t_ || t > *last_t_;

  if (is_new) {
    if (!t0_) {
      t0_ = t;
    }

    // IMU readings up to this pose, into the trajectory frame, each with the
    // attitude interpolated at its own time:
    const auto R1 = pose.getRotationMatrix();
    const auto R0 = last_in_ ? last_in_->getRotationMatrix() : R1;
    while (!pending_imu_.empty() && pending_imu_.front().t <= t) {
      auto s = pending_imu_.front();
      pending_imu_.pop_front();
      if (last_t_ && s.t <= *last_t_) {
        s.R = R0;
      } else if (last_t_) {
        s.R = interpolate_rotation(R0, R1, (s.t - *last_t_) / (t - *last_t_));
      } else {
        s.R = R1;
      }
      const Eigen::Vector3d fw = s.R.asEigen() * Eigen::Vector3d(s.f.x, s.f.y, s.f.z);
      s.f = mrpt::math::TVector3D(fw[0], fw[1], fw[2]);
      window_imu_.push_back(s);
    }
    const double keep = params_.window + params_.velocity_span + 1.0;
    while (!window_imu_.empty() && window_imu_.front().t < t - keep) {
      window_imu_.pop_front();
    }

    // The trajectory's own velocity over the last velocity_span seconds. It
    // describes the motion about span/2 ago, and is only valid once a whole
    // span of poses exists:
    PoseSample ps;
    ps.t = t;
    ps.p = pose.translation();
    const PoseSample * ref = nullptr;
    for (const auto & q : poses_) {
      if (q.t <= t - params_.velocity_span) {
        ref = &q;
      } else {
        break;
      }
    }
    if (ref && t > ref->t) {
      ps.v = (ps.p - ref->p) * (1.0 / (t - ref->t));
      ps.v_t = 0.5 * (t + ref->t);
    }
    poses_.push_back(ps);
    while (poses_.size() > 2 && poses_.front().t < t - keep) {
      poses_.pop_front();
    }

    if (!next_epoch_) {
      next_epoch_ = *t0_ + params_.window;
    }
    if (t >= *next_epoch_) {
      next_epoch_ = t + params_.epoch_period;
      update_filter(t);
    }
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

  const mrpt::math::CMatrixDouble33 R_att = R_now.getRotationMatrix() * pose.getRotationMatrix();
  mrpt::math::TVector3D p = pose.translation();
  if (last_in_ && last_out_) {
    p =
      last_out_->translation() + R_prev.rotateVector(pose.translation() - last_in_->translation());
  }
  const mrpt::poses::CPose3D out(R_att, as_vector(p));

  // A pose older than the last one is leveled, but does not move the state
  // the next increments and interpolations start from:
  if (is_new) {
    applied_tilt_ = tau;
    last_in_ = pose;
    last_out_ = out;
    last_t_ = t;
  }
  return out;
}

void GravityLeveler::update_filter(double t)
{
  // Mean acceleration of the trajectory between two valid velocities that
  // span (most of) the window:
  const PoseSample & now = poses_.back();
  if (!now.v) {
    return;
  }
  const PoseSample * first = nullptr;
  for (const auto & q : poses_) {
    if (q.t >= t - params_.window && q.v) {
      first = &q;
      break;
    }
  }
  if (!first || now.v_t - first->v_t < 0.5 * params_.window) {
    return;
  }
  const mrpt::math::TVector3D a_mean = (*now.v - *first->v) * (1.0 / (now.v_t - first->v_t));

  // Specific force over the same interval, between the midpoints of the two
  // velocities' supports:
  const double ta = first->v_t;
  const double tb = now.v_t;
  // Sparse poses may reach back beyond the IMU history kept: skip, since the
  // two means would cover different intervals.
  if (window_imu_.empty() || window_imu_.front().t > ta) {
    return;
  }
  mrpt::math::TVector3D f_mean(0, 0, 0);
  mrpt::math::CMatrixDouble33 R_mean = mrpt::math::CMatrixDouble33::Zero();
  std::size_t n = 0;
  for (const auto & s : window_imu_) {
    if (s.t <= ta || s.t > tb) {
      continue;
    }
    f_mean += s.f;
    R_mean += s.R;
    n++;
  }
  if (n < params_.min_readings_per_window) {
    return;
  }
  f_mean *= 1.0 / static_cast<double>(n);
  R_mean *= 1.0 / static_cast<double>(n);

  const auto g = f_mean - a_mean;
  if (std::abs(g.z) < 0.5 * params_.gravity) {
    return;
  }
  Eigen::Vector2d z;
  z << g.x / g.z, g.y / g.z;
  stats_.tilt_readings++;

  const double q = mrpt::DEG2RAD(params_.tilt_random_walk);
  const double r = mrpt::DEG2RAD(params_.reading_sigma);

  const bool starting = !filter_started_;
  if (starting) {
    P_.setZero();
    const double s0 = mrpt::DEG2RAD(params_.initial_tilt_sigma);
    const double b0 = params_.initial_acc_error_sigma;
    P_(0, 0) = s0 * s0;
    P_(1, 1) = s0 * s0;
    for (int i = 2; i < 5; i++) {
      P_(i, i) = b0 * b0;
    }
    x_.setZero();
  } else if (last_epoch_) {
    const double dt = t - *last_epoch_;
    P_(0, 0) += q * q * dt;
    P_(1, 1) += q * q * dt;
  }
  last_epoch_ = t;

  // tau_reading = tau + (mean(R) b)_xy / g
  Eigen::Matrix<double, 2, 5> H = Eigen::Matrix<double, 2, 5>::Zero();
  H(0, 0) = 1;
  H(1, 1) = 1;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 3; j++) {
      H(i, 2 + j) = R_mean(i, j) / params_.gravity;
    }
  }

  const Eigen::Vector2d y = z - H * x_;
  const Eigen::Matrix2d S = H * P_ * H.transpose() + r * r * Eigen::Matrix2d::Identity();

  // A window that does not measure gravity must not enter the correction,
  // but a run of them means the state itself is wrong: restart from here.
  if (!starting && y.dot(S.ldlt().solve(y)) > CHI2_2DOF_99) {
    stats_.gated_readings++;
    consecutive_rejections_++;
    if (consecutive_rejections_ < params_.max_consecutive_rejections) {
      return;
    }
    stats_.restarts++;
    reset_filter();
    update_filter(t);
    return;
  }
  consecutive_rejections_ = 0;
  filter_started_ = true;

  const Eigen::Matrix<double, 5, 2> K = P_ * H.transpose() * S.inverse();
  x_ += K * y;
  P_ = (Eigen::Matrix<double, 5, 5>::Identity() - K * H) * P_;
  P_ = (0.5 * (P_ + P_.transpose())).eval();
}
