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

#include <algorithm>
#include <array>
#include <cmath>
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
