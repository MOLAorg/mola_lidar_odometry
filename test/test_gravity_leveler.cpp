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
  const mrpt::poses::CPose3D & imu_misalignment, bool with_imu)
{
  mola::GravityLeveler leveler;
  RunResult r;
  double t_imu = 0;
  for (double t = 0; t <= duration; t += 1.0 / POSE_RATE) {
    while (with_imu && t_imu <= t) {
      leveler.add_imu(t_imu, imu_misalignment.rotateVector(true_specific_force(t_imu)));
      t_imu += 1.0 / IMU_RATE;
    }
    const auto p = frame_tilt + true_pose(t);
    r.in.push_back(p);
    r.out.push_back(leveler.add_pose(t, p));
  }
  return r;
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
    mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(5.0), mrpt::DEG2RAD(-3.0));
  const auto r = run(300.0, tilt, mrpt::poses::CPose3D(), true);

  // After convergence (one window plus margin), compare one lap of the
  // input and the output: the tilted frame makes the circle rise and fall.
  const size_t from = r.in.size() - static_cast<size_t>(2 * M_PI * RADIUS / SPEED * POSE_RATE);
  const double zin = height_range(r.in, from);
  const double zout = height_range(r.out, from);
  EXPECT_GT(zin, 3.0);
  // Only the deadband (2 deg) is left: about 2 * R * tan(2 deg) = 1.4 m.
  EXPECT_LT(zout, 0.45 * zin);
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
