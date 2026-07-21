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
 * @file   test_keyframe_decider.cpp
 * @brief  Unit tests for the shared keyframe-creation policy
 * @author Jose Luis Blanco Claraco
 */

#include <gtest/gtest.h>
#include <mola_lidar_odometry/KeyframeDecider.h>
#include <mrpt/core/exceptions.h>

#include <cmath>

namespace
{
mrpt::poses::CPose3D at(double x, double y = 0)
{
  return mrpt::poses::CPose3D::FromXYZYawPitchRoll(x, y, 0, 0, 0, 0);
}

mola::KeyframeDecisionOptions defaultOptions()
{
  mola::KeyframeDecisionOptions o;
  o.min_translation_between_keyframes = 1.0;
  o.min_rotation_between_keyframes = 30.0;
  return o;
}

/** Drives the decider along a straight line, inserting whenever it asks for a
 *  keyframe. Returns how many keyframes were created. */
size_t run(
  mola::KeyframeDecider & d, const mola::KeyframeDecisionOptions & o, double fromX, double toX,
  double step)
{
  ASSERT_(step != 0);
  const int nSteps = static_cast<int>(std::floor((toX - fromX) / step + 1e-9)) + 1;
  ASSERT_(nSteps > 0);

  size_t created = 0;
  for (int i = 0; i < nSteps; i++) {
    const double x = fromX + i * step;
    if (d.check(o, at(x)).create) {
      d.insert(at(x));
      created++;
    }
  }
  return created;
}
}  // namespace

// The first ever pose always creates a keyframe.
TEST(KeyframeDecider, FirstPoseAlwaysCreates)
{
  const auto o = defaultOptions();
  mola::KeyframeDecider d;

  EXPECT_TRUE(d.check(o, at(0)).create);
  EXPECT_TRUE(d.empty());
}

// Classic spatial policy: one keyframe per min_translation_between_keyframes.
TEST(KeyframeDecider, SpatialSpacing)
{
  const auto o = defaultOptions();
  mola::KeyframeDecider d;

  const size_t n = run(d, o, 0.0, 10.0, 0.25);

  // The threshold is strict, so keyframes land at 0, 1.25, 2.5, ... 10 m:
  EXPECT_EQ(n, 9u);
  EXPECT_EQ(d.size(), 9u);
}

// Coming back over an already-mapped area creates NO keyframes, since the
// previous pass' ones are the nearest neighbors.
TEST(KeyframeDecider, RevisitCreatesNothing)
{
  const auto o = defaultOptions();
  mola::KeyframeDecider d;

  run(d, o, 0.0, 10.0, 0.25);
  const size_t nAfterFirstPass = d.size();

  // Drive back over the very same places:
  const size_t createdOnRevisit = run(d, o, 10.0, 0.0, -0.25);

  EXPECT_EQ(createdOnRevisit, 0u);
  EXPECT_EQ(d.size(), nAfterFirstPass);
}

// Rotating in place beyond the angular threshold also creates keyframes.
TEST(KeyframeDecider, RotationThreshold)
{
  const auto o = defaultOptions();
  mola::KeyframeDecider d;

  d.insert(mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 0, 0, 0, 0));

  const auto smallTurn =
    mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 0, mrpt::DEG2RAD(10), 0, 0);
  EXPECT_FALSE(d.check(o, smallTurn).create);

  const auto bigTurn = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 0, mrpt::DEG2RAD(45), 0, 0);
  EXPECT_TRUE(d.check(o, bigTurn).create);
}

// min_nearby_poses_occupied > 1 asks for several keyframes per spot (used by
// non-repetitive-scan lidars).
TEST(KeyframeDecider, MinNearbyPosesOccupied)
{
  auto o = defaultOptions();
  o.min_nearby_poses_occupied = 3;

  mola::KeyframeDecider d;

  // Three keyframes at the very same place, then no more:
  for (int i = 0; i < 3; i++) {
    ASSERT_TRUE(d.check(o, at(0)).create) << "i=" << i;
    d.insert(at(0));
  }
  EXPECT_FALSE(d.check(o, at(0)).create);
  EXPECT_EQ(d.size(), 3u);
}

// measure_from_last_kf_only ignores all but the last keyframe, so a revisit
// creates keyframes even without the time window.
TEST(KeyframeDecider, MeasureFromLastOnly)
{
  const auto o = defaultOptions();
  mola::KeyframeDecider d(true /*measure_from_last_kf_only*/);

  run(d, o, 0.0, 10.0, 0.25);

  // Back to the origin: far from the LAST keyframe (at 10 m), so it creates.
  EXPECT_TRUE(d.check(o, at(0)).create);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
