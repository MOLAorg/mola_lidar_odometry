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
#include <cmath>
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
