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

#include <gtest/gtest.h>
#include <mola_lidar_odometry/GravityLeveler.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/poses/Lie/SO.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <functional>
#include <iostream>
#include <limits>
#include <optional>
#include <random>
#include <vector>

namespace
{
constexpr double G = 9.81;
constexpr double RADIUS = 20.0;
constexpr double SPEED = 2.0;
constexpr double IMU_RATE = 100.0;
constexpr double POSE_RATE = 10.0;

// True motion: a level platform driving a horizontal circle at constant speed.
mrpt::poses::CPose3D true_pose(double t)
{
  const double w = SPEED / RADIUS;
  const double a = w * t;
  return mrpt::poses::CPose3D::FromXYZYawPitchRoll(
    RADIUS * std::sin(a), RADIUS * (1.0 - std::cos(a)), 0.0, a, 0.0, 0.0);
}

// Specific force in the vehicle frame: centripetal acceleration plus gravity.
mrpt::math::TVector3D true_specific_force(double t)
{
  const double w = SPEED / RADIUS;
  const double a = w * t;
  const mrpt::math::TVector3D acc_world(
    -RADIUS * w * w * std::sin(a), RADIUS * w * w * std::cos(a), 0.0);
  return true_pose(t).inverseRotateVector(acc_world + mrpt::math::TVector3D(0, 0, G));
}

struct RunResult
{
  std::vector<mrpt::poses::CPose3D> in;
  std::vector<mrpt::poses::CPose3D> out;
};

// Feeds the leveler with a trajectory expressed in a frame tilted by
// `frame_tilt`, and IMU readings rotated by `imu_misalignment`.
RunResult run(
  double duration, const mrpt::poses::CPose3D & frame_tilt,
  const mrpt::poses::CPose3D & imu_misalignment, bool with_imu,
  const mola::GravityLeveler::Parameters & params = {}, mola::GravityLeveler * out = nullptr)
{
  mola::GravityLeveler local(params);
  mola::GravityLeveler & leveler = out ? *out : local;
  RunResult r;
  // Exact timestamps (k / rate), as simulators and fixed-rate logs give them:
  std::size_t k_imu = 0;
  const auto n_poses = static_cast<std::size_t>(duration * POSE_RATE);
  for (std::size_t k = 0; k <= n_poses; k++) {
    const double t = static_cast<double>(k) / POSE_RATE;
    while (with_imu && static_cast<double>(k_imu) / IMU_RATE <= t) {
      const double ti = static_cast<double>(k_imu) / IMU_RATE;
      leveler.add_imu(ti, imu_misalignment.rotateVector(true_specific_force(ti)));
      k_imu++;
    }
    const auto p = frame_tilt + true_pose(t);
    r.in.push_back(p);
    r.out.push_back(leveler.add_pose(t, p));
  }
  return r;
}

// Angle between a pose's z axis and the vertical [deg]
double tilt_of(const mrpt::poses::CPose3D & p)
{
  const auto z = p.rotateVector({0, 0, 1});
  return mrpt::RAD2DEG(std::acos(std::min(1.0, z.z / z.norm())));
}

double height_range(const std::vector<mrpt::poses::CPose3D> & v, size_t from)
{
  double lo = v.at(from).z();
  double hi = lo;
  for (size_t i = from; i < v.size(); i++) {
    lo = std::min(lo, v[i].z());
    hi = std::max(hi, v[i].z());
  }
  return hi - lo;
}
}  // namespace

TEST(GravityLeveler, RemovesFrameTilt)
{
  const auto tilt =
    mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(4.0), mrpt::DEG2RAD(-2.0));
  const auto r = run(300.0, tilt, mrpt::poses::CPose3D(), true);

  // After convergence (one window plus margin), compare one lap of the
  // input and the output: the tilted frame makes the circle rise and fall.
  const size_t from = r.in.size() - static_cast<size_t>(2 * M_PI * RADIUS / SPEED * POSE_RATE);
  const double zin = height_range(r.in, from);
  const double zout = height_range(r.out, from);
  EXPECT_GT(zin, 3.0);
  // Only the deadband (2 deg) is left: about 2 * R * sin(2 deg) = 1.4 m.
  EXPECT_LT(zout, 1.6);
}

TEST(GravityLeveler, LevelsAttitudeAndSeparatesImuError)
{
  // Tilted frame and a misaligned IMU, no deadband: the filter must attribute
  // the frame tilt to the frame and the misalignment to the IMU (the platform
  // turns), so the output attitude ends level.
  const auto tilt =
    mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(3.0), mrpt::DEG2RAD(-2.0));
  const auto mis =
    mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(0.6), mrpt::DEG2RAD(0.4));
  mola::GravityLeveler::Parameters p;
  p.deadband = 0;
  mola::GravityLeveler lev(p);
  const auto r = run(600.0, tilt, mis, true, p, &lev);

  EXPECT_GT(tilt_of(r.in.back()), 3.5);
  EXPECT_LT(tilt_of(r.out.back()), 0.3);
  ASSERT_TRUE(lev.tilt().has_value());
  EXPECT_NEAR(
    mrpt::RAD2DEG(std::hypot(lev.tilt()->x, lev.tilt()->y)),
    mrpt::RAD2DEG(std::acos(tilt.rotateVector({0, 0, 1}).z)), 0.3);
  EXPECT_EQ(lev.stats().restarts, 0U);
}

TEST(GravityLeveler, MovingStartWithExactTimestamps)
{
  // A level frame, moving from the first pose on: no fake initial velocity
  // may show up as acceleration, i.e. a tilt.
  mola::GravityLeveler::Parameters p;
  p.deadband = 0;
  mola::GravityLeveler lev(p);
  const auto r = run(300.0, mrpt::poses::CPose3D(), mrpt::poses::CPose3D(), true, p, &lev);
  ASSERT_TRUE(lev.tilt().has_value());
  EXPECT_LT(mrpt::RAD2DEG(std::hypot(lev.tilt()->x, lev.tilt()->y)), 0.2);
  EXPECT_LT(height_range(r.out, 0), 0.5);
  EXPECT_EQ(lev.stats().restarts, 0U);
}

TEST(GravityLeveler, LeavesLevelFrameAlone)
{
  // A level trajectory, read by an IMU misaligned by 0.5 deg: within the
  // deadband, so the output must be the input.
  const auto mis =
    mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(0.5), mrpt::DEG2RAD(0.3));
  const auto r = run(200.0, mrpt::poses::CPose3D(), mis, true);
  for (size_t i = 0; i < r.in.size(); i++) {
    EXPECT_NEAR(r.out[i].x(), r.in[i].x(), 1e-9);
    EXPECT_NEAR(r.out[i].y(), r.in[i].y(), 1e-9);
    EXPECT_NEAR(r.out[i].z(), r.in[i].z(), 1e-9);
  }
}

TEST(GravityLeveler, NoImuIsPassThrough)
{
  const auto tilt = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(5.0), 0.0);
  const auto r = run(60.0, tilt, mrpt::poses::CPose3D(), false);
  for (size_t i = 0; i < r.in.size(); i++) {
    EXPECT_NEAR((r.out[i].asVectorVal() - r.in[i].asVectorVal()).norm(), 0.0, 1e-9);
  }
}

TEST(GravityLeveler, StalePoseDoesNotMoveState)
{
  // A pose not newer than the previous one is leveled, but the outputs that
  // follow must be those of a run without it.
  const auto tilt = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(4.0), 0.0);
  mola::GravityLeveler a;
  mola::GravityLeveler b;
  std::size_t k_imu = 0;
  for (std::size_t k = 0; k <= 600; k++) {
    const double t = static_cast<double>(k) / POSE_RATE;
    while (static_cast<double>(k_imu) / IMU_RATE <= t) {
      const double ti = static_cast<double>(k_imu) / IMU_RATE;
      a.add_imu(ti, true_specific_force(ti));
      b.add_imu(ti, true_specific_force(ti));
      k_imu++;
    }
    const auto p = tilt + true_pose(t);
    const auto pa = a.add_pose(t, p);
    const auto pb = b.add_pose(t, p);
    EXPECT_NEAR((pa.asVectorVal() - pb.asVectorVal()).norm(), 0.0, 1e-9);
    if (k == 300) {
      // Stale pose, only to b:
      (void)b.add_pose(t - 1.0, tilt + true_pose(t - 1.0));
    }
  }
}

TEST(GravityLeveler, SparsePosesDoNotFakeTilt)
{
  // A level platform accelerating along x, with poses 9 s apart: the
  // velocities then reach back beyond the IMU history, and a reading over
  // mismatched intervals would show up as a tilt.
  mola::GravityLeveler::Parameters p;
  p.deadband = 0;
  mola::GravityLeveler lev(p);
  const auto x = [](double t) { return t * t * t / 60.0; };
  std::size_t k_imu = 0;
  for (std::size_t k = 0; k <= 6; k++) {
    const double t = 9.0 * static_cast<double>(k);
    while (static_cast<double>(k_imu) / IMU_RATE <= t) {
      const double ti = static_cast<double>(k_imu) / IMU_RATE;
      lev.add_imu(ti, mrpt::math::TVector3D(ti / 10.0, 0, G));
      k_imu++;
    }
    (void)lev.add_pose(t, mrpt::poses::CPose3D(x(t), 0, 0, 0, 0, 0));
  }
  if (lev.tilt()) {
    EXPECT_LT(mrpt::RAD2DEG(std::hypot(lev.tilt()->x, lev.tilt()->y)), 0.1);
  }
}

TEST(GravityLeveler, ImuDropoutsDoNotFakeTilt)
{
  // A level platform accelerating along x, with poses at 10 Hz and the IMU
  // missing from 5 s on (until 8 s, or for good): windows with no IMU
  // readings over part of their interval must not be read.
  const auto x = [](double t) { return t * t * t / 60.0; };
  for (const double resume : {8.0, 1e9}) {
    mola::GravityLeveler::Parameters p;
    p.deadband = 0;
    mola::GravityLeveler lev(p);
    std::size_t k_imu = 0;
    for (std::size_t k = 0; k <= 300; k++) {
      const double t = static_cast<double>(k) / POSE_RATE;
      while (static_cast<double>(k_imu) / IMU_RATE <= t) {
        const double ti = static_cast<double>(k_imu) / IMU_RATE;
        if (ti < 5.0 || ti >= resume) {
          lev.add_imu(ti, mrpt::math::TVector3D(ti / 10.0, 0, G));
        }
        k_imu++;
      }
      (void)lev.add_pose(t, mrpt::poses::CPose3D(x(t), 0, 0, 0, 0, 0));
    }
    if (lev.tilt()) {
      EXPECT_LT(mrpt::RAD2DEG(std::hypot(lev.tilt()->x, lev.tilt()->y)), 0.1)
        << "IMU resumes at " << resume;
    }
  }
}

TEST(GravityLeveler, UnevenImuRateDoesNotFakeTilt)
{
  // A level platform accelerating along x, with the IMU at 200 Hz for 12 s
  // and at 5 Hz afterwards: windows across the change must weight readings
  // by time, not by count.
  mola::GravityLeveler::Parameters p;
  p.deadband = 0;
  mola::GravityLeveler lev(p);
  const auto x = [](double t) { return t * t * t / 60.0; };
  double ti = 0;
  double max_tilt = 0;
  for (std::size_t k = 0; k <= 300; k++) {
    const double t = static_cast<double>(k) / POSE_RATE;
    while (ti <= t) {
      lev.add_imu(ti, mrpt::math::TVector3D(ti / 10.0, 0, G));
      ti += ti < 12.0 ? 1.0 / 200.0 : 1.0 / 5.0;
    }
    (void)lev.add_pose(t, mrpt::poses::CPose3D(x(t), 0, 0, 0, 0, 0));
    if (lev.tilt()) {
      max_tilt = std::max(max_tilt, std::hypot(lev.tilt()->x, lev.tilt()->y));
    }
  }
  ASSERT_TRUE(lev.tilt().has_value());
  EXPECT_LT(mrpt::RAD2DEG(max_tilt), 0.1);
}

TEST(GravityLeveler, ImplausibleTiltIsIgnored)
{
  // Readings far beyond any frame tilt (here, a trajectory attitude 20 deg
  // off gravity) must leave the output alone, and must not restart the
  // filter into them.
  const auto mis = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(20.0), 0.0);
  mola::GravityLeveler lev;
  const auto r = run(200.0, mrpt::poses::CPose3D(), mis, true, {}, &lev);
  for (size_t i = 0; i < r.in.size(); i++) {
    EXPECT_NEAR((r.out[i].asVectorVal() - r.in[i].asVectorVal()).norm(), 0.0, 1e-9);
  }
  EXPECT_FALSE(lev.tilt().has_value());
  EXPECT_GT(lev.stats().implausible_readings, 0U);
  EXPECT_EQ(lev.stats().restarts, 0U);
}

TEST(GravityLeveler, InvertedGravityIsIgnored)
{
  // An attitude almost upside down gives a small tilt ratio with gravity
  // pointing down: not a reading either.
  const auto mis = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, 0.0, mrpt::DEG2RAD(179.0));
  mola::GravityLeveler lev;
  const auto r = run(200.0, mrpt::poses::CPose3D(), mis, true, {}, &lev);
  for (size_t i = 0; i < r.in.size(); i++) {
    EXPECT_NEAR((r.out[i].asVectorVal() - r.in[i].asVectorVal()).norm(), 0.0, 1e-9);
  }
  EXPECT_FALSE(lev.tilt().has_value());
  EXPECT_GT(lev.stats().implausible_readings, 0U);
}

TEST(GravityLeveler, ImplausibleReadingsAfterStartKeepTheEstimate)
{
  // A running filter (3 deg frame tilt) meets 100 s of a trajectory attitude
  // 20 deg off gravity, then good readings again.
  const auto tilt = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(3.0), 0.0);
  const auto mis = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(20.0), 0.0);
  mola::GravityLeveler lev;
  std::size_t k_imu = 0;
  std::optional<mrpt::math::TVector2D> tilt_before;
  mola::GravityLeveler::Stats stats_before;
  for (std::size_t k = 0; k <= static_cast<std::size_t>(300.0 * POSE_RATE); k++) {
    const double t = static_cast<double>(k) / POSE_RATE;
    while (static_cast<double>(k_imu) / IMU_RATE <= t) {
      const double ti = static_cast<double>(k_imu) / IMU_RATE;
      const bool bad = ti >= 100.0 && ti < 200.0;
      const auto f = true_specific_force(ti);
      lev.add_imu(ti, bad ? mis.rotateVector(f) : f);
      k_imu++;
    }
    lev.add_pose(t, tilt + true_pose(t));
    // From one window after the bad span starts, every reading is implausible:
    if (std::abs(t - 115.0) < 1e-9) {
      ASSERT_TRUE(lev.tilt().has_value());
      tilt_before = lev.tilt();
      stats_before = lev.stats();
    }
    if (std::abs(t - 200.0) < 1e-9) {
      EXPECT_EQ(lev.tilt()->x, tilt_before->x);
      EXPECT_EQ(lev.tilt()->y, tilt_before->y);
      EXPECT_EQ(lev.stats().gated_readings, stats_before.gated_readings);
      EXPECT_EQ(lev.stats().restarts, stats_before.restarts);
      EXPECT_GT(lev.stats().implausible_readings, stats_before.implausible_readings + 50);
    }
  }
  EXPECT_EQ(lev.stats().restarts, 0U);
  ASSERT_TRUE(lev.tilt().has_value());
  EXPECT_NEAR(mrpt::RAD2DEG(lev.tilt()->norm()), 3.0, 0.5);
}

namespace
{
// The trajectory frame's tilt at time t, wandering slowly by a few degrees
mrpt::poses::CPose3D wander(double t)
{
  return mrpt::poses::CPose3D::FromYawPitchRoll(
    0.0, mrpt::DEG2RAD(2.5 * std::sin(t / 40.0)), mrpt::DEG2RAD(1.5 * std::cos(t / 55.0) - 1.0));
}

// Up of the true world, seen in a frame tilted by `tilt`, with noise
mrpt::math::TVector3D up_seen(const mrpt::poses::CPose3D & tilt, double nx, double ny)
{
  auto u = tilt.rotateVector({0, 0, 1});
  u.x += nx;
  u.y += ny;
  return u * (1.0 / u.norm());
}

mrpt::math::CMatrixDouble22 iso_cov(double sigma_deg)
{
  const double s = mrpt::DEG2RAD(sigma_deg);
  mrpt::math::CMatrixDouble22 c;
  c.setZero();
  c(0, 0) = s * s;
  c(1, 1) = s * s;
  return c;
}

struct WanderResult
{
  RunResult r;
  double max_tilt_error_deg = 0;  // after the first minute
};

// A level circle driven while the odometry's frame wanders: each increment
// and attitude rotated by the tilt of its time. Up readings every
// `period` seconds (0: none), IMU readings optional.
WanderResult run_wander(
  double duration, double period, double sigma_deg, bool with_imu, mola::GravityLeveler & lev)
{
  WanderResult w;
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(0.0, mrpt::DEG2RAD(sigma_deg));
  std::size_t k_imu = 0;
  double next_up = period;
  mrpt::poses::CPose3D prev_true = true_pose(0);
  mrpt::math::TPoint3D p_in = prev_true.translation();
  const auto n_poses = static_cast<std::size_t>(duration * POSE_RATE);
  for (std::size_t k = 0; k <= n_poses; k++) {
    const double t = static_cast<double>(k) / POSE_RATE;
    const auto tilt = wander(t);
    while (with_imu && static_cast<double>(k_imu) / IMU_RATE <= t) {
      const double ti = static_cast<double>(k_imu) / IMU_RATE;
      lev.add_imu(ti, true_specific_force(ti));
      k_imu++;
    }
    if (period > 0 && t >= next_up) {
      next_up += period;
      lev.add_up_reading(t, up_seen(tilt, noise(rng), noise(rng)), iso_cov(sigma_deg));
    }
    const auto pt = true_pose(t);
    if (k > 0) {
      p_in = p_in + tilt.rotateVector(pt.translation() - prev_true.translation());
    }
    prev_true = pt;
    mrpt::math::CVectorFixedDouble<3> xyz;
    xyz[0] = p_in.x;
    xyz[1] = p_in.y;
    xyz[2] = p_in.z;
    const mrpt::poses::CPose3D in(
      mrpt::math::CMatrixDouble33(
        tilt.getRotationMatrix().asEigen() * pt.getRotationMatrix().asEigen()),
      xyz);
    w.r.in.push_back(in);
    w.r.out.push_back(lev.add_pose(t, in));
    if (t > 60.0 && lev.tilt()) {
      const auto u = tilt.rotateVector({0, 0, 1});
      const double ex = lev.tilt()->x - u.x / u.z;
      const double ey = lev.tilt()->y - u.y / u.z;
      w.max_tilt_error_deg = std::max(w.max_tilt_error_deg, mrpt::RAD2DEG(std::hypot(ex, ey)));
    }
  }
  return w;
}
}  // namespace

TEST(GravityLeveler, UpReadingsFollowAWanderingFrame)
{
  // Wall readings alone (no IMU), 0.1 deg noise every 5 s: the filter tracks
  // the wander (up to 0.06 deg/s here, with the lag of the default random
  // walk) and the output height stays flat where the input climbs.
  mola::GravityLeveler lev;
  const auto w = run_wander(400.0, 5.0, 0.1, false, lev);
  EXPECT_GT(height_range(w.r.in, 0), 2.0);
  EXPECT_LT(height_range(w.r.out, 0), 0.25 * height_range(w.r.in, 0));
  EXPECT_LT(w.max_tilt_error_deg, 0.8);
  EXPECT_GT(lev.stats().up_readings, 70U);
  EXPECT_EQ(lev.stats().gated_up_readings, 0U);
  EXPECT_EQ(lev.stats().up_restarts, 0U);
}

TEST(GravityLeveler, UpReadingsWithImuSeparateTheImuError)
{
  // A constant frame tilt, a misaligned IMU and wall readings: the tilt goes
  // to the frame (the walls see it), the misalignment to the body-frame term,
  // so the output attitude ends level with no deadband.
  const auto tilt =
    mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(3.0), mrpt::DEG2RAD(-2.0));
  const auto mis =
    mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(0.6), mrpt::DEG2RAD(0.4));
  mola::GravityLeveler::Parameters p;
  p.deadband = 0;
  p.up_reading_deadband = 0;
  mola::GravityLeveler lev(p);
  std::size_t k_imu = 0;
  RunResult r;
  for (std::size_t k = 0; k <= static_cast<std::size_t>(300.0 * POSE_RATE); k++) {
    const double t = static_cast<double>(k) / POSE_RATE;
    while (static_cast<double>(k_imu) / IMU_RATE <= t) {
      const double ti = static_cast<double>(k_imu) / IMU_RATE;
      lev.add_imu(ti, mis.rotateVector(true_specific_force(ti)));
      k_imu++;
    }
    if (k % 50 == 0 && k > 0) {
      lev.add_up_reading(t, up_seen(tilt, 0, 0), iso_cov(0.1));
    }
    const auto p_in = tilt + true_pose(t);
    r.in.push_back(p_in);
    r.out.push_back(lev.add_pose(t, p_in));
  }
  ASSERT_TRUE(lev.tilt().has_value());
  const auto u = tilt.rotateVector({0, 0, 1});
  EXPECT_NEAR(lev.tilt()->x, u.x / u.z, mrpt::DEG2RAD(0.1));
  EXPECT_NEAR(lev.tilt()->y, u.y / u.z, mrpt::DEG2RAD(0.1));
  EXPECT_LT(tilt_of(r.out.back()), 0.1);
  EXPECT_EQ(lev.stats().gated_up_readings, 0U);
}

TEST(GravityLeveler, UpReadingsLeaveALevelFrameAlone)
{
  // A level frame, noisy readings (0.1 deg every 5 s): with no deadband the
  // estimate follows that noise and moves the output by up to ~0.15 m over
  // the 600 m path; a deadband above the noise keeps it within centimeters.
  for (const double deadband : {0.0, 0.25}) {
    mola::GravityLeveler::Parameters p;
    p.up_reading_deadband = deadband;
    mola::GravityLeveler lev(p);
    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0.0, mrpt::DEG2RAD(0.1));
    double max_dev = 0;
    for (std::size_t k = 0; k <= static_cast<std::size_t>(300.0 * POSE_RATE); k++) {
      const double t = static_cast<double>(k) / POSE_RATE;
      if (k % 50 == 0 && k > 0) {
        lev.add_up_reading(
          t, up_seen(mrpt::poses::CPose3D(), noise(rng), noise(rng)), iso_cov(0.1));
      }
      const auto in = true_pose(t);
      const auto out = lev.add_pose(t, in);
      max_dev = std::max(max_dev, (out.asVectorVal() - in.asVectorVal()).norm());
    }
    ASSERT_TRUE(lev.tilt().has_value());
    EXPECT_LT(mrpt::RAD2DEG(lev.tilt()->norm()), 0.15);
    EXPECT_LT(max_dev, deadband > 0 ? 0.03 : 0.2);
  }
}

TEST(GravityLeveler, UpReadingOutliersAreGatedAndImplausibleOnesIgnored)
{
  const auto tilt = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(1.0), 0.0);
  mola::GravityLeveler lev;
  for (int k = 1; k <= 20; k++) {
    lev.add_up_reading(5.0 * k, up_seen(tilt, 0, 0), iso_cov(0.1));
  }
  const auto before = *lev.tilt();
  // One wall reading 3 deg away: beyond the gate, not used.
  const auto off = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(4.0), 0.0);
  lev.add_up_reading(105.0, up_seen(off, 0, 0), iso_cov(0.1));
  EXPECT_EQ(lev.stats().gated_up_readings, 1U);
  EXPECT_NEAR(lev.tilt()->x, before.x, 1e-6);
  EXPECT_NEAR(lev.tilt()->y, before.y, 1e-6);
  // Up pointing down, or far beyond a frame tilt: ignored.
  lev.add_up_reading(110.0, {0, 0, -1}, iso_cov(0.1));
  lev.add_up_reading(
    115.0, up_seen(mrpt::poses::CPose3D::FromYawPitchRoll(0, 0.5, 0), 0, 0), iso_cov(0.1));
  EXPECT_EQ(lev.stats().implausible_up_readings, 2U);
  EXPECT_EQ(lev.stats().up_restarts, 0U);
}

TEST(GravityLeveler, UpReadingsHaveTheirOwnTiltLimit)
{
  // A frame tilted 9 deg, beyond the accelerometer's 5 deg limit, and no
  // accelerometer: wall readings still see it and level the output, attitude
  // and height, if they need no accelerometer to agree. With their limit at
  // 5 deg, or by default (no accelerometer reading beyond 5 deg), they are
  // all ignored.
  const auto tilt = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(9.0), 0.0);
  struct Case
  {
    double limit;
    bool needs_acc;
  };
  const double wide = mola::GravityLeveler::Parameters{}.max_up_tilt;
  for (const auto c : {Case{wide, false}, Case{5.0, false}, Case{wide, true}}) {
    mola::GravityLeveler::Parameters p;
    p.max_up_tilt = c.limit;
    p.max_up_tilt_needs_accelerometer = c.needs_acc;
    mola::GravityLeveler lev(p);
    const double limit = c.needs_acc ? p.max_tilt : c.limit;
    RunResult r;
    for (std::size_t k = 0; k <= static_cast<std::size_t>(120.0 * POSE_RATE); k++) {
      const double t = static_cast<double>(k) / POSE_RATE;
      if (k % 10 == 0 && k > 0) {
        lev.add_up_reading(t, up_seen(tilt, 0, 0), iso_cov(0.1));
      }
      r.in.push_back(tilt + true_pose(t));
      r.out.push_back(lev.add_pose(t, r.in.back()));
    }
    if (limit > 9.0) {
      EXPECT_EQ(lev.stats().implausible_up_readings, 0U);
      ASSERT_TRUE(lev.tilt().has_value());
      EXPECT_NEAR(std::atan(lev.tilt()->norm()), mrpt::DEG2RAD(9.0), mrpt::DEG2RAD(0.01));
      // Leveled to within a hundredth of a degree: the correction is the
      // angle of the tilt, not its tangent.
      EXPECT_LT(tilt_of(r.out.back() + (-true_pose(120.0))), 0.01);
      EXPECT_GT(height_range(r.in, 0), 5.0);
      EXPECT_LT(height_range(r.out, 10), 0.05);
    } else {
      EXPECT_EQ(lev.stats().implausible_up_readings, lev.stats().up_readings);
      EXPECT_FALSE(lev.tilt().has_value());
      EXPECT_NEAR(height_range(r.out, 0), height_range(r.in, 0), 1e-9);
    }
  }
}

TEST(GravityLeveler, UpReadingsBeyondTheAccelerometerLimitNeedItsAgreement)
{
  // A frame tilted 9 deg with the accelerometer: its own readings are beyond
  // max_tilt, so the up readings (9 deg) are taken. A level frame whose
  // accelerometer sees it level: up readings of 9 deg come from structure
  // that is not plumb, and are ignored.
  const auto tilt9 = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(9.0), 0.0);
  for (const bool tilted : {true, false}) {
    mola::GravityLeveler lev;
    const auto frame = tilted ? tilt9 : mrpt::poses::CPose3D();
    std::size_t k_imu = 0;
    for (std::size_t k = 0; k <= static_cast<std::size_t>(120.0 * POSE_RATE); k++) {
      const double t = static_cast<double>(k) / POSE_RATE;
      while (static_cast<double>(k_imu) / IMU_RATE <= t) {
        const double ti = static_cast<double>(k_imu) / IMU_RATE;
        lev.add_imu(ti, true_specific_force(ti));
        k_imu++;
      }
      if (k % 10 == 0 && t > 20.0) {
        lev.add_up_reading(t, up_seen(tilt9, 0, 0), iso_cov(0.1));
      }
      (void)lev.add_pose(t, frame + true_pose(t));
    }
    if (tilted) {
      EXPECT_GT(lev.stats().implausible_readings, 0U);
      EXPECT_EQ(lev.stats().implausible_up_readings, 0U);
      ASSERT_TRUE(lev.tilt().has_value());
      EXPECT_NEAR(std::atan(lev.tilt()->norm()), mrpt::DEG2RAD(9.0), mrpt::DEG2RAD(0.05));
    } else {
      EXPECT_EQ(lev.stats().implausible_readings, 0U);
      EXPECT_EQ(lev.stats().implausible_up_readings, lev.stats().up_readings);
      ASSERT_TRUE(lev.tilt().has_value());
      EXPECT_LT(mrpt::RAD2DEG(lev.tilt()->norm()), 0.5);
    }
  }
}

TEST(GravityLeveler, WeakGravityIsNoAgreementForWiderUpReadings)
{
  // A level frame whose accelerometer reads a weak gravity (a failing
  // attitude, or a sensor fault): that is no sign of a frame tilted beyond
  // max_tilt, so 9 deg up readings stay ignored.
  mola::GravityLeveler lev;
  std::size_t k_imu = 0;
  for (std::size_t k = 0; k <= static_cast<std::size_t>(60.0 * POSE_RATE); k++) {
    const double t = static_cast<double>(k) / POSE_RATE;
    while (static_cast<double>(k_imu) / IMU_RATE <= t) {
      lev.add_imu(static_cast<double>(k_imu) / IMU_RATE, {0, 0, 4.0});
      k_imu++;
    }
    if (k % 10 == 0 && t > 20.0) {
      const double d = mrpt::DEG2RAD(9.0);
      lev.add_up_reading(t, {std::sin(d), 0.0, std::cos(d)}, iso_cov(0.1));
    }
    (void)lev.add_pose(t, mrpt::poses::CPose3D(t, 0, 0, 0, 0, 0));
  }
  EXPECT_GT(lev.stats().implausible_readings, 0U);
  EXPECT_GT(lev.stats().up_readings, 0U);
  EXPECT_EQ(lev.stats().implausible_up_readings, lev.stats().up_readings);
  EXPECT_FALSE(lev.tilt().has_value());
}

namespace
{
struct LevelFrameResult
{
  double max_abs_z = 0;
  double max_pitch_deg = 0;
};

// A level frame driven straight along x at 1 m/s for 120 s, optionally with
// the accelerometer (level, at rest). Up readings every second from t = 5 s:
// the first `n_bad` read a pitch of `bad_deg` (e.g. rock faces), the rest
// read level.
LevelFrameResult run_level_frame(
  const mola::GravityLeveler::Parameters & p, int n_bad, double bad_deg, bool with_imu)
{
  mola::GravityLeveler lev(p);
  LevelFrameResult r;
  std::size_t k_imu = 0;
  int n_up = 0;
  for (std::size_t k = 0; k <= static_cast<std::size_t>(120.0 * POSE_RATE); k++) {
    const double t = static_cast<double>(k) / POSE_RATE;
    while (with_imu && static_cast<double>(k_imu) / IMU_RATE <= t) {
      lev.add_imu(static_cast<double>(k_imu) / IMU_RATE, {0, 0, G});
      k_imu++;
    }
    if (k % 10 == 0 && t >= 5.0) {
      const double d = mrpt::DEG2RAD(n_up < n_bad ? bad_deg : 0.0);
      lev.add_up_reading(t, {std::sin(d), 0.0, std::cos(d)}, iso_cov(0.3));
      n_up++;
    }
    const auto out = lev.add_pose(t, mrpt::poses::CPose3D(t, 0, 0, 0, 0, 0));
    r.max_abs_z = std::max(r.max_abs_z, std::abs(out.z()));
    r.max_pitch_deg = std::max(r.max_pitch_deg, mrpt::RAD2DEG(std::abs(out.pitch())));
  }
  return r;
}
}  // namespace

TEST(GravityLeveler, UpReadingsBeforeTheFirstAccelerometerReadingAreHeldToItsLimit)
{
  // The first wall readings come before the first accelerometer reading
  // (one window, 10 s): with the accelerometer in use, a 9 deg reading of a
  // level frame (a rock face) must not start the filter at 9 deg.
  for (const int n_bad : {1, 3}) {
    const auto r = run_level_frame({}, n_bad, 9.0, true);
    EXPECT_LT(r.max_abs_z, 0.01) << n_bad;
    EXPECT_LT(r.max_pitch_deg, 0.1) << n_bad;
  }
}

TEST(GravityLeveler, UpReadingsWithoutAccelerometerAreHeldToItsLimit)
{
  // Walls only (no accelerometer, the walls' deadband for both): ten 8 deg
  // readings of a level frame are ignored by default, and taken only if the
  // wider limit needs no accelerometer.
  mola::GravityLeveler::Parameters p;
  p.deadband = p.up_reading_deadband;
  const auto held = run_level_frame(p, 10, 8.0, false);
  EXPECT_LT(held.max_abs_z, 0.01);
  EXPECT_LT(held.max_pitch_deg, 0.1);

  p.max_up_tilt_needs_accelerometer = false;
  const auto taken = run_level_frame(p, 10, 8.0, false);
  EXPECT_GT(taken.max_abs_z, 0.5);
}

TEST(GravityLeveler, UpReadingTiltLimitMustBeValid)
{
  for (const double limit : {0.0, -1.0, 90.0}) {
    mola::GravityLeveler::Parameters p;
    p.max_up_tilt = limit;
    EXPECT_ANY_THROW(mola::GravityLeveler{p});
  }
}

namespace
{
// A frame whose tilt depends on where the vehicle is in it, as in a map that
// is slightly domed: a tilt growing with the distance from the origin.
mrpt::poses::CPose3D tilt_at(const mrpt::math::TPoint3D & p)
{
  constexpr double K = 0.002;  // [rad/m]
  return mrpt::poses::CPose3D::FromYawPitchRoll(0.0, K * p.x, -K * p.y);
}

// Laps of a 10 m circle at 1 m/s. Every second, an up reading of the place
// where the vehicle was 2.5 s before (a trailing window), with that place or
// without it. Returns the output height each time a lap ends, where the true
// path is level.
std::vector<double> laps_over_a_dome(mola::GravityLeveler & lev, bool give_place, int laps)
{
  constexpr double R = 10.0;
  constexpr double DT = 0.1;
  const auto truth = [](double t) {
    return mrpt::math::TPoint3D(R * std::cos(t / R), R * std::sin(t / R), 0);
  };
  const double lap = 2 * M_PI * R;
  std::vector<double> heights;
  std::deque<std::pair<mrpt::math::TPoint3D, mrpt::math::TPoint3D>> history;  // true, input
  mrpt::math::TPoint3D p_in = truth(0);
  int next_lap = 1;
  for (int k = 1; heights.size() < static_cast<std::size_t>(laps); k++) {
    const double t = k * DT;
    const auto a = truth(t - DT);
    const auto b = truth(t);
    const auto mid = mrpt::math::TPoint3D(0.5 * (a.x + b.x), 0.5 * (a.y + b.y), 0);
    p_in = p_in + tilt_at(mid).rotateVector(b - a);
    history.emplace_back(b, p_in);
    if (history.size() > 26) {
      history.pop_front();
    }
    if (k % 10 == 0 && history.size() == 26) {
      const auto & [p_true, p_frame] = history.front();
      const auto u = tilt_at(p_true).rotateVector({0, 0, 1});
      const auto place = give_place ? std::optional<mrpt::math::TPoint3D>(p_frame) : std::nullopt;
      lev.add_up_reading(t, u, iso_cov(0.05), place);
    }
    mrpt::math::CVectorFixedDouble<3> xyz;
    xyz[0] = p_in.x;
    xyz[1] = p_in.y;
    xyz[2] = p_in.z;
    const auto out = lev.add_pose(t, mrpt::poses::CPose3D(tilt_at(b).getRotationMatrix(), xyz));
    if (t >= next_lap * lap - 1e-9) {
      heights.push_back(out.z());
      next_lap++;
    }
  }
  return heights;
}
}  // namespace

TEST(GravityLeveler, UpReadingsByPlaceDoNotLagBehind)
{
  // By time, each pose gets the tilt of where the vehicle was 2.5 s before,
  // and on a domed frame that error points along the track: the output climbs
  // lap after lap. By place, the readings are applied where they were taken.
  mola::GravityLeveler::Parameters p;
  p.up_reading_place_radius = 5.0;
  mola::GravityLeveler by_place(p);
  p.up_reading_place_radius = 0.0;
  mola::GravityLeveler by_time(p);
  const auto h_place = laps_over_a_dome(by_place, true, 5);
  const auto h_time = laps_over_a_dome(by_time, true, 5);
  ASSERT_EQ(h_place.size(), 5U);
  ASSERT_EQ(h_time.size(), 5U);
  const double climb_time = (h_time[4] - h_time[1]) / 3.0;
  const double climb_place = (h_place[4] - h_place[1]) / 3.0;
  std::cout << "height change per lap: by time " << climb_time << " m, by place " << climb_place
            << " m\n";
  EXPECT_GT(std::abs(climb_time), 0.1);
  EXPECT_LT(std::abs(climb_place), 0.2 * std::abs(climb_time));
  EXPECT_GT(by_place.stats().place_leveled_poses, 2500U);
  EXPECT_EQ(by_time.stats().place_leveled_poses, 0U);
}

namespace
{
// Out and back along x at speed `v`: a level frame, the accelerometer (with
// the smooth 10 s reversal at t = 100 s it measures), and up readings every
// second on the way out only (a revisit adds no new local-map scans), each of
// the place 2.5 s behind. They read level, except `n_bad` of them from
// t = 40 s, which read a pitch of 3 deg. Returns the largest height change
// on the way back, where the true path is level.
double out_and_back(const mola::GravityLeveler::Parameters & p, double v, int n_bad)
{
  constexpr double T_TURN = 100.0;
  constexpr double T_END = 195.0;
  const double t1 = T_TURN - 5.0;
  const auto x_at = [&](double t) {
    if (t <= t1) {
      return v * t;
    }
    const double tt = std::min(t, t1 + 10.0) - t1;
    double x = v * t1 + v * 10.0 / M_PI * std::sin(M_PI * tt / 10.0);
    if (t > t1 + 10.0) {
      x -= v * (t - t1 - 10.0);
    }
    return x;
  };
  const auto acc_at = [&](double t) {
    if (t <= t1 || t >= t1 + 10.0) {
      return 0.0;
    }
    return -v * M_PI / 10.0 * std::sin(M_PI * (t - t1) / 10.0);
  };
  mola::GravityLeveler lev(p);
  std::size_t k_imu = 0;
  double z_turn = 0;
  double max_dz_back = 0;
  for (std::size_t k = 0; k <= static_cast<std::size_t>(T_END * POSE_RATE); k++) {
    const double t = static_cast<double>(k) / POSE_RATE;
    while (static_cast<double>(k_imu) / IMU_RATE <= t) {
      const double ti = static_cast<double>(k_imu) / IMU_RATE;
      lev.add_imu(ti, {acc_at(ti), 0, G});
      k_imu++;
    }
    if (k % 10 == 0 && t >= 5.0 && t < t1) {
      const auto i = std::lround(t);
      const double d = mrpt::DEG2RAD((i >= 40 && i < 40 + n_bad) ? 3.0 : 0.0);
      lev.add_up_reading(
        t, {std::sin(d), 0.0, std::cos(d)}, iso_cov(0.3),
        mrpt::math::TPoint3D(x_at(t - 2.5), 0, 0));
    }
    const auto out = lev.add_pose(t, mrpt::poses::CPose3D(x_at(t), 0, 0, 0, 0, 0));
    if (t <= T_TURN) {
      z_turn = out.z();
    } else {
      max_dz_back = std::max(max_dz_back, std::abs(out.z() - z_turn));
    }
  }
  EXPECT_GE(lev.stats().gated_up_readings, static_cast<std::size_t>(n_bad));
  return max_dz_back;
}
}  // namespace

TEST(GravityLeveler, RejectedUpReadingsDoNotLevelTheirPlace)
{
  // A reading the filter rejects must not level the poses of its place when
  // they are revisited: at vehicle speed one or two readings are all a place
  // has, and an outlier among them would bend the way back.
  for (const double v : {1.0, 5.0}) {
    for (const int n_bad : {1, 2}) {
      EXPECT_LT(out_and_back({}, v, n_bad), 0.02) << "v=" << v << " n_bad=" << n_bad;
    }
  }
}

TEST(GravityLeveler, UpReadingsByPlaceFadeOutTowardTheirEdge)
{
  // A level reading at the origin, then 1 deg readings far away bring the
  // filter's state to 1 deg. Driving past the origin, the correction falls
  // from the state's to the place's and back with the distance to that
  // reading, without a jump at the radius. The place is left 13 s after the
  // last reading, beyond the 8 s hold, but the walls' deadband (0) stays in
  // force while poses are leveled by place alone: right after leaving the
  // place the state's 1 deg is still applied, not cut by the accelerometer's
  // deadband (2 deg).
  mola::GravityLeveler::Parameters p;
  p.up_reading_hold = 8.0;
  p.up_reading_ramp = 0.0;
  mola::GravityLeveler lev(p);
  const double d1 = mrpt::DEG2RAD(1.0);
  lev.add_up_reading(1.0, {0, 0, 1}, iso_cov(0.3), mrpt::math::TPoint3D(0, 0, 0));
  double t = 1.0;
  for (int i = 0; i < 20; i++) {
    t += 1.0;
    lev.add_up_reading(
      t, {std::sin(d1), 0.0, std::cos(d1)}, iso_cov(0.3), mrpt::math::TPoint3D(100, 0, 0));
    (void)lev.add_pose(t, mrpt::poses::CPose3D(100, 0, 0, 0, 0, 0));
  }
  ASSERT_TRUE(lev.tilt().has_value());
  ASSERT_NEAR(mrpt::RAD2DEG(std::atan(lev.tilt()->norm())), 1.0, 0.1);
  std::optional<double> prev;
  double max_step = 0;
  double at_origin = 1.0;
  double after = 0.0;
  for (int k = -80; k <= 80; k++) {
    const double x = 0.1 * k;
    t += 0.1;
    (void)lev.add_pose(t, mrpt::poses::CPose3D(x, 0, 0, 0, 0, 0));
    const double a = mrpt::RAD2DEG(std::atan(lev.applied_tilt().norm()));
    if (prev) {
      max_step = std::max(max_step, std::abs(a - *prev));
    }
    prev = a;
    if (k == 0) {
      at_origin = a;
    }
    if (k > 50) {
      after = std::max(after, a);
    }
  }
  std::cout << "by place at the origin " << at_origin << " deg, after it " << after
            << " deg, largest step " << max_step << " deg\n";
  EXPECT_LT(at_origin, 0.1);
  EXPECT_GT(after, 0.9);
  EXPECT_LT(max_step, 0.1);
  EXPECT_GT(lev.stats().place_leveled_poses, 80U);

  // Standing outside the place until the hold expires: the accelerometer's
  // 2 deg deadband covers the state's 1 deg, so no correction. Driving back
  // past the place, its deadband fades in with its readings too: no jump
  // where its readings start.
  for (int i = 0; i < 100; i++) {
    t += 0.1;
    (void)lev.add_pose(t, mrpt::poses::CPose3D(8.0, 0, 0, 0, 0, 0));
  }
  EXPECT_LT(mrpt::RAD2DEG(std::atan(lev.applied_tilt().norm())), 0.01);
  prev.reset();
  max_step = 0;
  for (int k = 80; k >= -80; k--) {
    t += 0.1;
    (void)lev.add_pose(t, mrpt::poses::CPose3D(0.1 * k, 0, 0, 0, 0, 0));
    const double a = mrpt::RAD2DEG(std::atan(lev.applied_tilt().norm()));
    if (prev) {
      max_step = std::max(max_step, std::abs(a - *prev));
    }
    prev = a;
  }
  std::cout << "back past the place after the hold: largest step " << max_step << " deg\n";
  EXPECT_LT(max_step, 0.1);
}

namespace
{
// The correction [deg] applied at `query` when the only reading kept there is
// one of 1 deg pitch at the origin, and the filter's state was brought back to
// level by readings far away (so the hold is fresh: the walls' deadband, 0).
double correction_near_one_reading(const mrpt::math::TPoint3D & query)
{
  mola::GravityLeveler lev;
  const double d1 = mrpt::DEG2RAD(1.0);
  lev.add_up_reading(
    1.0, {std::sin(d1), 0.0, std::cos(d1)}, iso_cov(0.3), mrpt::math::TPoint3D(0, 0, 0));
  double t = 1.0;
  for (int i = 0; i < 20; i++) {
    t += 1.0;
    lev.add_up_reading(t, {0, 0, 1}, iso_cov(0.3), mrpt::math::TPoint3D(100, 0, 0));
    (void)lev.add_pose(t, mrpt::poses::CPose3D(100, 0, 0, 0, 0, 0));
  }
  EXPECT_LT(mrpt::RAD2DEG(std::atan(lev.tilt()->norm())), 0.05);
  (void)lev.add_pose(t + 0.1, mrpt::poses::CPose3D(query.x, query.y, query.z, 0, 0, 0));
  return mrpt::RAD2DEG(std::atan(lev.applied_tilt().norm()));
}
}  // namespace

TEST(GravityLeveler, UpReadingsByPlaceStayOnTheirFloor)
{
  // Another floor of the map, 3.2 m above or below, carries its own tilt
  // error: readings of one floor must not level the floor above, at the same
  // spot or next to it, while they still level their own floor on a revisit.
  EXPECT_GT(correction_near_one_reading({0, 0, 0}), 0.95);
  EXPECT_GT(correction_near_one_reading({1, 0, 0}), 0.95);
  EXPECT_LT(correction_near_one_reading({0, 0, 3.2}), 0.01);
  EXPECT_LT(correction_near_one_reading({1, 0, 3.2}), 0.01);
  EXPECT_LT(correction_near_one_reading({0, 0, -3.2}), 0.01);
  // The vertical cut, 1.5 m by default:
  EXPECT_GT(correction_near_one_reading({0, 0, 1.4}), 0.95);
  EXPECT_LT(correction_near_one_reading({0, 0, 1.6}), 0.01);
}

TEST(GravityLeveler, UpReadingsByPlaceFollowARamp)
{
  // A 10 % ramp: 2 m and 4 m along it rise 0.2 m and 0.4 m, still the same
  // place, weighted by the horizontal distance alone.
  const double at2 = correction_near_one_reading({2, 0, 0.2});
  const double at4 = correction_near_one_reading({4, 0, 0.4});
  EXPECT_NEAR(at2, correction_near_one_reading({2, 0, 0}), 1e-9);
  EXPECT_NEAR(at4, correction_near_one_reading({4, 0, 0}), 1e-9);
  EXPECT_GT(at2, 0.95);
  EXPECT_GT(at4, 0.2);
  EXPECT_LT(correction_near_one_reading({5.5, 0, 0.55}), 0.01);
}

TEST(GravityLeveler, UpReadingsByPlaceCanBeTurnedOff)
{
  // Radius 0: the places are ignored, the output is the time-based one.
  mola::GravityLeveler::Parameters p;
  p.up_reading_place_radius = 0.0;
  mola::GravityLeveler with_places(p);
  mola::GravityLeveler without(p);
  const auto a = laps_over_a_dome(with_places, true, 2);
  const auto b = laps_over_a_dome(without, false, 2);
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); i++) {
    EXPECT_EQ(a[i], b[i]);
  }
  EXPECT_EQ(with_places.stats().place_leveled_poses, 0U);

  p.up_reading_place_radius = -1.0;
  EXPECT_ANY_THROW(mola::GravityLeveler{p});
  p.up_reading_place_radius = 5.0;
  p.up_reading_place_max_dz = 0.0;
  EXPECT_ANY_THROW(mola::GravityLeveler{p});
}

TEST(GravityLeveler, UpReadingWarmupIgnoresTheFirstReadings)
{
  // A level frame whose first 30 s of readings lean 1 deg (the structure of
  // the first places, not the frame), then correct ones, all with their
  // place. Without a warmup the output follows them and leaves the level
  // circle by decimeters. With it they are not kept by place either, so the
  // second lap over those places stays level too.
  const auto lean = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(1.0), 0.0);
  for (const double warmup : {0.0, 30.0}) {
    mola::GravityLeveler::Parameters p;
    p.up_reading_warmup = warmup;
    mola::GravityLeveler lev(p);
    double max_dz = 0;
    for (std::size_t k = 0; k <= static_cast<std::size_t>(120.0 * POSE_RATE); k++) {
      const double t = static_cast<double>(k) / POSE_RATE;
      if (k % 10 == 0 && k > 0) {
        const auto u = (t < 30.0 ? lean : mrpt::poses::CPose3D()).rotateVector({0, 0, 1});
        lev.add_up_reading(t, u, iso_cov(0.1), true_pose(t).translation());
      }
      max_dz = std::max(max_dz, std::abs(lev.add_pose(t, true_pose(t)).z()));
    }
    if (warmup > 0) {
      EXPECT_EQ(lev.stats().warmup_up_readings, 29U);
      EXPECT_LT(max_dz, 0.02);
    } else {
      EXPECT_EQ(lev.stats().warmup_up_readings, 0U);
      EXPECT_GT(max_dz, 0.2);
    }
  }
  mola::GravityLeveler::Parameters p;
  p.up_reading_warmup = -1.0;
  EXPECT_ANY_THROW(mola::GravityLeveler{p});
}

TEST(GravityLeveler, RepeatedlyRejectedUpReadingsRestartTheFilter)
{
  // The frame tilt steps by 3 deg (e.g. the state locked onto a wrong value):
  // after max_consecutive_rejections readings in a row the filter restarts
  // from the walls.
  const auto a = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(0.5), 0.0);
  const auto b = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(3.5), 0.0);
  // A slow random walk, so the step stays beyond the gate reading after reading:
  mola::GravityLeveler::Parameters p;
  p.up_reading_random_walk = 0.05;
  mola::GravityLeveler lev(p);
  for (int k = 1; k <= 20; k++) {
    lev.add_up_reading(5.0 * k, up_seen(a, 0, 0), iso_cov(0.05));
  }
  for (int k = 21; k <= 30; k++) {
    lev.add_up_reading(5.0 * k, up_seen(b, 0, 0), iso_cov(0.05));
  }
  EXPECT_EQ(lev.stats().up_restarts, 1U);
  const auto u = b.rotateVector({0, 0, 1});
  EXPECT_NEAR(lev.tilt()->x, u.x / u.z, mrpt::DEG2RAD(0.1));
}

TEST(GravityLeveler, DeadbandRampsBackWhenUpReadingsStop)
{
  // A 1 deg frame tilt read by walls for one minute: corrected (no deadband
  // for up readings). After the 30 s hold the deadband grows to 2 deg over
  // 30 s, so the correction fades without a step and is gone at the end.
  const auto tilt = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(1.0), 0.0);
  mola::GravityLeveler::Parameters p;
  p.up_reading_hold = 30.0;
  p.up_reading_ramp = 30.0;
  mola::GravityLeveler lev(p);
  double applied_at_60 = -1;
  double applied_at_97 = -1;
  double applied_at_130 = -1;
  double max_step = 0;
  double prev = 0;
  for (std::size_t k = 0; k <= static_cast<std::size_t>(150.0 * POSE_RATE); k++) {
    const double t = static_cast<double>(k) / POSE_RATE;
    if (k % 50 == 0 && k > 0 && t <= 60.0) {
      lev.add_up_reading(t, up_seen(tilt, 0, 0), iso_cov(0.1));
    }
    (void)lev.add_pose(t, tilt + true_pose(t));
    const double applied = mrpt::RAD2DEG(lev.applied_tilt().norm());
    if (t > 61.0) {
      max_step = std::max(max_step, std::abs(applied - prev));
    }
    prev = applied;
    if (std::abs(t - 60.0) < 1e-9) {
      applied_at_60 = applied;
    }
    if (std::abs(t - 97.5) < 1e-9) {
      applied_at_97 = applied;
    }
    if (std::abs(t - 130.0) < 1e-9) {
      applied_at_130 = applied;
    }
  }
  EXPECT_NEAR(applied_at_60, 1.0, 0.1);
  // At 97.5 s the deadband is 0.5 deg:
  EXPECT_NEAR(applied_at_97, 0.5, 0.1);
  EXPECT_EQ(applied_at_130, 0.0);
  // 2 deg over 30 s at 10 Hz: under 0.01 deg per pose.
  EXPECT_LT(max_step, 0.01);
}

TEST(GravityLeveler, NonFiniteUpReadingsAreIgnored)
{
  const auto tilt = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(1.0), 0.0);
  mola::GravityLeveler lev;
  for (int k = 1; k <= 10; k++) {
    lev.add_up_reading(5.0 * k, up_seen(tilt, 0, 0), iso_cov(0.1));
  }
  const auto before = *lev.tilt();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  auto cov_nan = iso_cov(0.1);
  cov_nan(0, 0) = nan;
  auto cov_neg = iso_cov(0.1);
  cov_neg(1, 1) = -1e-6;
  auto cov_not_psd = iso_cov(0.1);
  cov_not_psd(0, 1) = 1.0;
  cov_not_psd(1, 0) = 1.0;
  lev.add_up_reading(55.0, up_seen(tilt, 0, 0), cov_nan);
  lev.add_up_reading(56.0, up_seen(tilt, 0, 0), cov_neg);
  lev.add_up_reading(57.0, up_seen(tilt, 0, 0), cov_not_psd);
  lev.add_up_reading(58.0, {nan, 0, 1}, iso_cov(0.1));
  EXPECT_EQ(lev.stats().implausible_up_readings, 4U);
  ASSERT_TRUE(lev.tilt().has_value());
  EXPECT_EQ(lev.tilt()->x, before.x);
  EXPECT_EQ(lev.tilt()->y, before.y);
  // And the filter keeps working:
  lev.add_up_reading(60.0, up_seen(tilt, 0, 0), iso_cov(0.1));
  EXPECT_TRUE(std::isfinite(lev.tilt()->x) && std::isfinite(lev.tilt()->y));
  const auto out = lev.add_pose(60.0, tilt + true_pose(60.0));
  EXPECT_TRUE(std::isfinite(out.z()));
}

TEST(GravityLeveler, RandomWalkSwitchesWhereTheHoldExpires)
{
  // An up reading accepted at 100 s, hold 30 s: a prediction to 131 s uses
  // the up readings' random walk over 100-130 s and the accelerometer's over
  // 130-131 s. A reading with a huge covariance leaves the predicted
  // covariance as it is, so the prediction can be read back.
  mola::GravityLeveler::Parameters p;
  p.up_reading_hold = 30.0;
  p.up_reading_random_walk = 0.3;
  p.tilt_random_walk = 0.1;
  const auto tilt = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(1.0), 0.0);
  mrpt::math::CMatrixDouble22 huge;
  huge.setZero();
  huge(0, 0) = 1e6;
  huge(1, 1) = 1e6;
  const double qw = mrpt::DEG2RAD(0.3);
  const double qa = mrpt::DEG2RAD(0.1);
  for (const double t_next : {125.0, 131.0, 160.0}) {
    mola::GravityLeveler lev(p);
    lev.add_up_reading(100.0, up_seen(tilt, 0, 0), iso_cov(0.1));
    const double p0 = (*lev.tilt_cov())(0, 0);
    lev.add_up_reading(t_next, up_seen(tilt, 0, 0), huge);
    const double grown = (*lev.tilt_cov())(0, 0) - p0;
    const double walls = std::min(t_next, 130.0) - 100.0;
    const double imu = std::max(0.0, t_next - 130.0);
    EXPECT_NEAR(grown, qw * qw * walls + qa * qa * imu, 1e-3 * qw * qw * walls) << t_next;
  }
}

TEST(GravityLeveler, SigmasMustBePositive)
{
  // A zero initial sigma and sigma floor would let a reading with a zero
  // covariance give a singular innovation covariance:
  mola::GravityLeveler::Parameters p;
  p.up_reading_sigma_floor = 0.0;
  EXPECT_ANY_THROW(mola::GravityLeveler{p});
  p = {};
  p.initial_tilt_sigma = 0.0;
  EXPECT_ANY_THROW(mola::GravityLeveler{p});
  // With the defaults, a reading of zero covariance is a valid one:
  mola::GravityLeveler lev;
  mrpt::math::CMatrixDouble22 zero;
  zero.setZero();
  lev.add_up_reading(1.0, {0, 0, 1}, zero);
  lev.add_up_reading(2.0, {0, 0, 1}, zero);
  EXPECT_EQ(lev.stats().singular_readings, 0U);
  ASSERT_TRUE(lev.tilt().has_value());
  EXPECT_TRUE(std::isfinite(lev.tilt()->x) && std::isfinite(lev.tilt()->y));
}

TEST(GravityLeveler, OlderPoseGetsTheCurrentCorrection)
{
  // A 1 deg frame tilt read once at 60 s; by 130 s the hold (30 s) and the
  // ramp (30 s) are over and the correction is gone. A pose at 70 s, older
  // than the last one, must be leveled with that current (null) correction,
  // not with the one its own time would have had.
  const auto tilt = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(1.0), 0.0);
  mola::GravityLeveler lev;
  lev.add_up_reading(60.0, up_seen(tilt, 0, 0), iso_cov(0.1));
  for (int k = 600; k <= 1300; k++) {
    const double t = k / POSE_RATE;
    (void)lev.add_pose(t, tilt + true_pose(t));
  }
  ASSERT_EQ(lev.applied_tilt().norm(), 0.0);
  const auto in = tilt + true_pose(70.0);
  const auto out = lev.add_pose(70.0, in);
  const auto d = mrpt::poses::Lie::SO<3>::log(mrpt::math::CMatrixDouble33(
    out.getRotationMatrix().asEigen() * in.getRotationMatrix().asEigen().transpose()));
  EXPECT_LT(mrpt::RAD2DEG(d.norm()), 1e-6);
  EXPECT_EQ(lev.applied_tilt().norm(), 0.0);
}

TEST(GravityLeveler, OlderUpReadingDoesNotAgeTheHold)
{
  // Up readings accepted at 100 s and then at 50 s (out of order, before any
  // pose): the hold runs from the newest one, so at 110 s the walls are
  // 10 s old and their (null) deadband applies to the 1 deg tilt.
  const auto tilt = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(1.0), 0.0);
  mola::GravityLeveler lev;
  lev.add_up_reading(100.0, up_seen(tilt, 0, 0), iso_cov(0.1));
  lev.add_up_reading(50.0, up_seen(tilt, 0, 0), iso_cov(0.1));
  EXPECT_EQ(lev.stats().gated_up_readings, 0U);
  (void)lev.add_pose(110.0, tilt + true_pose(110.0));
  EXPECT_NEAR(mrpt::RAD2DEG(lev.applied_tilt().norm()), 1.0, 0.1);
}

namespace
{
struct RevisitResult
{
  double split = 0;        // output height difference at x = 10 m, back minus out
  double max_step = 0;     // largest per-step change of the output offset [m]
  double pitch_split = 0;  // output pitch difference there [deg]
};

// Out to x = 50 m and back at 1 m/s on level ground, no accelerometer. Up
// readings by time only, of their place: level until t = 20 s, then 1 deg of
// pitch. The way back gets 1 deg where the way out had 0, so the path
// integral of the correction splits the two visits of x = 10 m.
RevisitResult out_and_back_by_time(double revisit_radius)
{
  mola::GravityLeveler::Parameters p;
  p.up_reading_place_radius = 0;
  p.deadband = p.up_reading_deadband;
  p.revisit_radius = revisit_radius;
  mola::GravityLeveler lev(p);
  RevisitResult r;
  double z_out = 0;
  double pitch_out = 0;
  std::optional<double> prev_offset;
  for (std::size_t k = 0; k <= static_cast<std::size_t>(100.0 * POSE_RATE); k++) {
    const double t = static_cast<double>(k) / POSE_RATE;
    const double x = t <= 50.0 ? t : 100.0 - t;
    if (k % 10 == 0) {
      const double d = mrpt::DEG2RAD(t < 20.0 ? 0.0 : 1.0);
      lev.add_up_reading(
        t, {std::sin(d), 0.0, std::cos(d)}, iso_cov(0.1), mrpt::math::TPoint3D(x, 0, 0));
    }
    const auto out = lev.add_pose(t, mrpt::poses::CPose3D(x, 0, 0, 0, 0, 0));
    if (prev_offset) {
      r.max_step = std::max(r.max_step, std::abs(out.z() - *prev_offset));
    }
    prev_offset = out.z();
    if (k == 100) {
      z_out = out.z();
      pitch_out = out.pitch();
    }
    if (k == 900) {
      r.split = out.z() - z_out;
      r.pitch_split = mrpt::RAD2DEG(out.pitch() - pitch_out);
    }
  }
  return r;
}
}  // namespace

TEST(GravityLeveler, RevisitsKeepTheOffsetOfTheirPlace)
{
  // Integrated along the path, the two visits of a place are split by what
  // the correction did in between: the floor would be drawn twice. With
  // revisit_radius, the way back takes the offset of the way out there,
  // faded in over revisit_fade so the output does not jump.
  const auto integrated = out_and_back_by_time(0.0);
  const auto by_place = out_and_back_by_time(1.0);
  std::cout << "split at x = 10 m: integrated " << integrated.split << " m, by place "
            << by_place.split << " m; largest step " << by_place.max_step << " m\n";
  EXPECT_GT(std::abs(integrated.split), 0.1);
  EXPECT_LT(std::abs(by_place.split), 0.005);
  EXPECT_LT(by_place.max_step, 0.02);
  // The attitude too: both visits get the same correction
  EXPECT_GT(std::abs(integrated.pitch_split), 0.5);
  EXPECT_LT(std::abs(by_place.pitch_split), 0.01);

  mola::GravityLeveler::Parameters p;
  p.revisit_radius = 1.0;
  p.revisit_fade = 1.0;
  EXPECT_ANY_THROW(mola::GravityLeveler{p});
}

namespace
{
struct PathRun
{
  std::vector<double> t;
  std::vector<mrpt::poses::CPose3D> in;
  std::vector<mrpt::poses::CPose3D> out;
  mola::GravityLeveler::Stats stats;
};

// Poses at 10 Hz along `at(t)` (input heights as given) for `duration` s,
// without an accelerometer, and up readings every second of the pitch
// `pitch_deg(t)`, by time only (no place), so that the correction changes
// along the path.
PathRun run_path(
  double revisit_radius, const std::function<mrpt::math::TPoint3D(double)> & at, double duration,
  const std::function<double(double)> & pitch_deg)
{
  mola::GravityLeveler::Parameters p;
  p.up_reading_place_radius = 0;
  p.deadband = p.up_reading_deadband;
  p.revisit_radius = revisit_radius;
  p.revisit_fade = 5.0 * std::max(1.0, revisit_radius);
  mola::GravityLeveler lev(p);
  PathRun r;
  for (std::size_t k = 0; k <= static_cast<std::size_t>(duration * POSE_RATE); k++) {
    const double t = static_cast<double>(k) / POSE_RATE;
    if (k % 10 == 0) {
      const double d = mrpt::DEG2RAD(pitch_deg(t));
      lev.add_up_reading(t, {std::sin(d), 0.0, std::cos(d)}, iso_cov(0.1));
    }
    const auto x = at(t);
    r.t.push_back(t);
    r.in.emplace_back(x.x, x.y, x.z, 0, 0, 0);
    r.out.push_back(lev.add_pose(t, r.in.back()));
  }
  r.stats = lev.stats();
  return r;
}

// Largest per-step change of the output offset (output minus input) [m]
double max_offset_step(const PathRun & r)
{
  double m = 0;
  for (std::size_t i = 1; i < r.t.size(); i++) {
    const auto a = r.out[i].translation() - r.in[i].translation();
    const auto b = r.out[i - 1].translation() - r.in[i - 1].translation();
    m = std::max(m, (a - b).norm());
  }
  return m;
}

// Change of the height difference of poses i and j from leveling [m]
double split(const PathRun & r, std::size_t i, std::size_t j)
{
  return (r.out[j].z() - r.out[i].z()) - (r.in[j].z() - r.in[i].z());
}

mrpt::math::TPoint3D on_circle(double t)
{
  constexpr double R = 5.0;  // a lap of 31.4 m at 1 m/s
  return {R * std::cos(t / R), R * std::sin(t / R), 0.0};
}
}  // namespace

TEST(GravityLeveler, RevisitLapsAllTakeTheFirstVisit)
{
  // Twelve laps of a circle while the correction wanders: every lap takes the
  // first lap's correction (it does not chain from lap to lap), and nothing
  // jumps when a lap passes a place visited 31 s, 63 s... before.
  const auto wander = [](double t) { return 1.0 + 0.8 * std::sin(t / 37.0); };
  const auto r = run_path(1.0, on_circle, 12 * 31.4, wander);
  const auto off = run_path(0.0, on_circle, 12 * 31.4, wander);
  const std::size_t lap = 314;
  double worst = 0;
  double worst_off = 0;
  for (std::size_t k = 2; k < 12; k++) {
    worst = std::max(worst, std::abs(split(r, lap + 50, k * lap + 50)));
    worst_off = std::max(worst_off, std::abs(split(off, lap + 50, k * lap + 50)));
  }
  std::cout << "lap-to-lap split: integrated " << worst_off << " m, by first visit " << worst
            << " m; largest step " << max_offset_step(r) << " m\n";
  EXPECT_GT(worst_off, 0.05);
  EXPECT_LT(worst, 0.005);
  EXPECT_LT(max_offset_step(r), 0.01);
  EXPECT_GT(r.stats.revisit_poses, 3000U);
  EXPECT_EQ(off.stats.revisit_poses, 0U);
}

TEST(GravityLeveler, RevisitKeepsTheFirstVisitThroughLaterOnes)
{
  // A place visited at t = 0, 31 and 61 s (two loops of 31 m and 30 m), then
  // a stop there, while the readings go from 0 to 1 deg (t = 15 s) and to
  // 2 deg (t = 45 s). Every visit and the whole stop keep the first visit's
  // correction, and nothing steps at the third visit (t = 61 to 61.1 s).
  const auto at = [](double t) -> mrpt::math::TPoint3D {
    const double r1 = 31.0 / (2 * M_PI);
    const double r2 = 30.0 / (2 * M_PI);
    if (t < 31.0) {
      return {r1 - r1 * std::cos(t / r1), r1 * std::sin(t / r1), 0.0};
    }
    if (t < 61.0) {
      const double u = t - 31.0;
      return {r2 * std::cos(u / r2) - r2, r2 * std::sin(u / r2), 0.0};
    }
    return {0.0, 0.0, 0.0};
  };
  const auto pitch = [](double t) { return t < 15 ? 0.0 : t < 45 ? 1.0 : 2.0; };
  const auto r = run_path(1.0, at, 90, pitch);
  const auto off = run_path(0.0, at, 90, pitch);
  double worst = 0;
  double worst_off = 0;
  for (const std::size_t k : {310U, 610U, 611U, 900U}) {
    worst = std::max(worst, std::abs(split(r, 0, k)));
    worst_off = std::max(worst_off, std::abs(split(off, 0, k)));
  }
  const auto offset = [&r](std::size_t k) {
    return (r.out[k].translation() - r.in[k].translation()).norm();
  };
  std::cout << "split from the first visit: integrated " << worst_off << " m, by first visit "
            << worst << " m; step at the third visit " << std::abs(offset(611) - offset(610))
            << " m; pitch at the end " << mrpt::RAD2DEG(r.out[900].pitch()) << " deg (integrated "
            << mrpt::RAD2DEG(off.out[900].pitch()) << " deg)\n";
  EXPECT_GT(worst_off, 0.05);
  EXPECT_LT(worst, 0.005);
  EXPECT_LT(std::abs(offset(611) - offset(610)), 0.001);
  EXPECT_LT(max_offset_step(r), 0.01);
  EXPECT_NEAR(r.out[900].pitch(), r.out[0].pitch(), mrpt::DEG2RAD(0.05));
  EXPECT_GT(std::abs(off.out[900].pitch() - r.out[0].pitch()), mrpt::DEG2RAD(1.0));
}

TEST(GravityLeveler, RevisitIsTakenUpOverTheApproach)
{
  // Out along y = 0, then back along y = 8 m, closing in on the way out
  // between x = 20 and 10 m: the first visit's offset is taken up gradually
  // as the distance falls from the fade (5 m) to the radius (1 m), and in
  // full where the paths meet. Parallel at 3 m it is half taken (the weight
  // there), at 8 m not at all.
  const auto at = [](double t) -> mrpt::math::TPoint3D {
    if (t <= 50) {
      return {t, 0, 0};
    }
    const double x = 100 - t;
    const double y = x >= 20 ? 8.0 : x <= 10 ? 0.0 : 8.0 * (x - 10) / 10;
    return {x, y, 0};
  };
  const auto pitch = [](double t) { return t < 30 ? 0.0 : 1.0; };
  const auto r = run_path(1.0, at, 100, pitch);
  const auto off = run_path(0.0, at, 100, pitch);
  // x = 5 m: out at t = 5 s, back at t = 95 s
  const double s_on = split(r, 50, 950);
  const double s_off = split(off, 50, 950);
  std::cout << "split at x = 5 m: integrated " << s_off << " m, by first visit " << s_on
            << " m; largest step " << max_offset_step(r) << " m\n";
  EXPECT_GT(std::abs(s_off), 0.1);
  EXPECT_LT(std::abs(s_on), 0.005);
  EXPECT_LT(max_offset_step(r), 0.01);
}

TEST(GravityLeveler, RevisitOfAParallelPathIsPartly)
{
  // A way back parallel to the way out, 3 m apart: within the fade, so part
  // of the first visit's offset is taken (the effective reach is the fade,
  // 5 m); 8 m apart, none.
  const auto pitch = [](double t) { return t < 30 ? 0.0 : 1.0; };
  for (const double gap : {3.0, 8.0}) {
    const auto at = [gap](double t) -> mrpt::math::TPoint3D {
      return t <= 50 ? mrpt::math::TPoint3D(t, 0, 0) : mrpt::math::TPoint3D(100 - t, gap, 0);
    };
    const auto r = run_path(1.0, at, 100, pitch);
    const auto off = run_path(0.0, at, 100, pitch);
    const double kept = split(r, 50, 950) / split(off, 50, 950);
    std::cout << "parallel at " << gap << " m keeps " << kept << " of its split\n";
    if (gap < 5) {
      EXPECT_GT(kept, 0.2);
      EXPECT_LT(kept, 0.8);
    } else {
      EXPECT_NEAR(kept, 1.0, 1e-9);
    }
  }
}

TEST(GravityLeveler, RevisitIsNotASlowPassOrAStop)
{
  // A straight 30 m line, level, seen in a frame tilted 1 deg (the input
  // climbs), at 0.05 m/s: the pose's own trail is close by for minutes, but
  // it is not a revisit, so the climb is leveled as without revisits. Then
  // a 300 s stop while the readings move to 2 deg: the correction follows.
  const double tan1 = std::tan(mrpt::DEG2RAD(1.0));
  const auto at = [tan1](double t) -> mrpt::math::TPoint3D {
    const double x = 0.05 * std::min(t, 600.0);
    return {x, 0, x * tan1};
  };
  const auto pitch = [](double t) { return t < 600 ? -1.0 : -2.0; };
  const auto r = run_path(1.0, at, 900, pitch);
  const auto off = run_path(0.0, at, 900, pitch);
  EXPECT_EQ(r.stats.revisit_poses, 0U);
  const std::size_t end_line = 6000;
  EXPECT_NEAR(r.out[end_line].z(), off.out[end_line].z(), 1e-9);
  EXPECT_LT(std::abs(r.out[end_line].z()), 0.05 * r.in[end_line].z());
  // After the stop, both apply 2 deg:
  EXPECT_NEAR(mrpt::RAD2DEG(r.out.back().pitch()), mrpt::RAD2DEG(off.out.back().pitch()), 1e-6);
}

TEST(GravityLeveler, RevisitStaysOnItsFloor)
{
  // Way out on the floor at z = 0, way back 3.2 m above (the floor above):
  // another place. 1 m above (within the floor cut) it is partly the same.
  const auto pitch = [](double t) { return t < 30 ? 0.0 : 1.0; };
  for (const double dz : {3.2, 1.0}) {
    const auto at = [dz](double t) -> mrpt::math::TPoint3D {
      return t <= 50 ? mrpt::math::TPoint3D(t, 0, 0) : mrpt::math::TPoint3D(100 - t, 0, dz);
    };
    const auto r = run_path(1.0, at, 100, pitch);
    if (dz > 1.5) {
      EXPECT_EQ(r.stats.revisit_poses, 0U);
    } else {
      EXPECT_GT(r.stats.revisit_poses, 0U);
    }
  }
}

TEST(GravityLeveler, RevisitIgnoresOlderPosesAndScalesWithTheRadius)
{
  // An older pose is leveled with the current correction, and neither moves
  // the path nor adds a place; a radius of 6 m with a 30 m fade works.
  const auto pitch = [](double t) { return t < 30 ? 0.0 : 1.0; };
  const auto at = [](double t) -> mrpt::math::TPoint3D { return {t <= 50 ? t : 100 - t, 0, 0}; };
  mola::GravityLeveler::Parameters p;
  p.up_reading_place_radius = 0;
  p.deadband = p.up_reading_deadband;
  p.revisit_radius = 1.0;
  mola::GravityLeveler a(p);
  mola::GravityLeveler b(p);
  double max_diff = 0;
  for (std::size_t k = 0; k <= 1000; k++) {
    const double t = static_cast<double>(k) / POSE_RATE;
    for (auto * lev : {&a, &b}) {
      if (k % 10 == 0) {
        const double d = mrpt::DEG2RAD(pitch(t));
        lev->add_up_reading(t, {std::sin(d), 0.0, std::cos(d)}, iso_cov(0.1));
      }
    }
    const auto x = at(t);
    const auto oa = a.add_pose(t, mrpt::poses::CPose3D(x.x, x.y, x.z, 0, 0, 0));
    if (k == 600) {
      // An old pose, far away, out of order:
      (void)b.add_pose(t - 30, mrpt::poses::CPose3D(500, 500, 0, 0, 0, 0));
    }
    const auto ob = b.add_pose(t, mrpt::poses::CPose3D(x.x, x.y, x.z, 0, 0, 0));
    max_diff = std::max(max_diff, (oa.translation() - ob.translation()).norm());
  }
  EXPECT_LT(max_diff, 1e-9);
  EXPECT_EQ(a.stats().revisit_poses, b.stats().revisit_poses);

  p.revisit_radius = 6.0;
  p.revisit_fade = 30.0;
  EXPECT_NO_THROW(mola::GravityLeveler{p});
  p.revisit_fade = 6.0;
  EXPECT_ANY_THROW(mola::GravityLeveler{p});
}
