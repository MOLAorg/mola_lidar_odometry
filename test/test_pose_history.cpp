/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

/**
 * @file   test_pose_history.cpp
 * @brief  Unit tests for pose_history_at() and planar_motion()
 * @author Jose Luis Blanco Claraco
 */

#include <gtest/gtest.h>
#include <mola_lidar_odometry/PoseHistory.h>
#include <mrpt/core/bits_math.h>

#include <cmath>
#include <map>

namespace
{
using mrpt::poses::CPose3D;

CPose3D tilted(double x, double y, double z)
{
  // 30 deg pitch, e.g. a vehicle on a slope:
  return CPose3D::FromXYZYawPitchRoll(x, y, z, 0, mrpt::DEG2RAD(30.0), 0);
}

void expect_near(const CPose3D & a, const CPose3D & b)
{
  EXPECT_NEAR(a.x(), b.x(), 1e-9);
  EXPECT_NEAR(a.y(), b.y(), 1e-9);
  EXPECT_NEAR(a.z(), b.z(), 1e-9);
  EXPECT_NEAR(a.yaw(), b.yaw(), 1e-9);
  EXPECT_NEAR(a.pitch(), b.pitch(), 1e-9);
  EXPECT_NEAR(a.roll(), b.roll(), 1e-9);
}
}  // namespace

TEST(PoseHistory, EmptyHistory) { EXPECT_FALSE(mola::pose_history_at({}, 1.0, 0.5).has_value()); }

// Stored samples come back as they are, also for tilted poses:
TEST(PoseHistory, ExactSamplesOfTiltedPoses)
{
  const std::map<double, CPose3D> h = {{10.0, tilted(0, 0, 0)}, {11.0, tilted(0, 0, 1)}};
  expect_near(*mola::pose_history_at(h, 10.0, 0.5), tilted(0, 0, 0));
  expect_near(*mola::pose_history_at(h, 11.0, 0.5), tilted(0, 0, 1));
}

// In between, the 3D position is interpolated linearly, attitude included:
TEST(PoseHistory, InterpolatesIn3D)
{
  const std::map<double, CPose3D> h = {{10.0, tilted(0, 0, 0)}, {11.0, tilted(2, 1, 1)}};
  expect_near(*mola::pose_history_at(h, 10.25, 0.5), tilted(0.5, 0.25, 0.25));

  const std::map<double, CPose3D> turn = {
    {0.0, CPose3D::FromXYZYawPitchRoll(0, 0, 0, 0, 0, 0)},
    {1.0, CPose3D::FromXYZYawPitchRoll(1, 0, 0, mrpt::DEG2RAD(40.0), 0, 0)}};
  const auto mid = *mola::pose_history_at(turn, 0.5, 0.5);
  EXPECT_NEAR(mid.x(), 0.5, 1e-9);
  EXPECT_NEAR(mrpt::RAD2DEG(mid.yaw()), 20.0, 1e-6);
}

// Queries a little past either end take that end; farther ones are not covered:
TEST(PoseHistory, Ends)
{
  const std::map<double, CPose3D> h = {{10.0, tilted(0, 0, 0)}, {11.0, tilted(1, 0, 0)}};
  expect_near(*mola::pose_history_at(h, 9.7, 0.5), tilted(0, 0, 0));
  expect_near(*mola::pose_history_at(h, 11.4, 0.5), tilted(1, 0, 0));
  EXPECT_FALSE(mola::pose_history_at(h, 9.4, 0.5).has_value());
  EXPECT_FALSE(mola::pose_history_at(h, 11.6, 0.5).has_value());
}

// 3 m forward along a 30 deg slope are 2.598 m on the horizontal plane:
TEST(PoseHistory, PlanarMotionOfTiltedVehicle)
{
  const auto from = tilted(0, 0, 0);
  const auto to = from + CPose3D::FromTranslation(3.0, 0, 0);
  const auto m = mola::planar_motion(from, to);
  EXPECT_NEAR(m.x(), 3.0 * std::cos(mrpt::DEG2RAD(30.0)), 1e-9);
  EXPECT_NEAR(m.y(), 0.0, 1e-9);
  EXPECT_NEAR(m.phi(), 0.0, 1e-9);
}

// The motion is given in the frame of the first pose:
TEST(PoseHistory, PlanarMotionInFirstFrame)
{
  const auto from = CPose3D::FromXYZYawPitchRoll(5, 5, 0, mrpt::DEG2RAD(90.0), 0, 0);
  const auto to = CPose3D::FromXYZYawPitchRoll(5, 7, 0, mrpt::DEG2RAD(90.0), 0, 0);
  const auto m = mola::planar_motion(from, to);
  EXPECT_NEAR(m.x(), 2.0, 1e-9);
  EXPECT_NEAR(m.y(), 0.0, 1e-9);
}

// A pose at t is final only once the history brackets t:
TEST(PoseHistory, Brackets)
{
  const std::map<double, CPose3D> h = {{10.0, tilted(0, 0, 0)}, {11.0, tilted(1, 0, 0)}};
  EXPECT_TRUE(mola::pose_history_brackets(h, 10.5));
  EXPECT_TRUE(mola::pose_history_brackets(h, 10.0));
  EXPECT_TRUE(mola::pose_history_brackets(h, 11.0));
  // Within the end gaps, pose_history_at() gives an end pose, but later
  // readings (also out of order, older ones) may still change it:
  EXPECT_FALSE(mola::pose_history_brackets(h, 11.3));
  EXPECT_FALSE(mola::pose_history_brackets(h, 9.7));
  EXPECT_FALSE(mola::pose_history_brackets({}, 1.0));
}
