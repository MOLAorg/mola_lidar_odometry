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
  double step, double & t, double dt)
{
  ASSERT_(step != 0);
  const int nSteps = static_cast<int>(std::floor((toX - fromX) / step + 1e-9)) + 1;
  ASSERT_(nSteps > 0);

  size_t created = 0;
  for (int i = 0; i < nSteps; i++, t += dt) {
    const double x = fromX + i * step;
    if (d.check(o, at(x), t).create) {
      d.insert(at(x), t);
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
  mola::KeyframeDecider d(o);

  EXPECT_TRUE(d.check(o, at(0), 0.0).create);
  EXPECT_TRUE(d.empty());
}

// Classic spatial policy: one keyframe per min_translation_between_keyframes.
TEST(KeyframeDecider, SpatialSpacing)
{
  const auto o = defaultOptions();
  mola::KeyframeDecider d(o);

  double t = 0;
  const size_t n = run(d, o, 0.0, 10.0, 0.25, t, 0.1);

  // The threshold is strict, so keyframes land at 0, 1.25, 2.5, ... 10 m:
  EXPECT_EQ(n, 9u);
  EXPECT_EQ(d.size(), 9u);
}

// The bug this policy exists to fix: with the purely spatial policy, coming
// back over an already-mapped area creates NO keyframes, so loop closure never
// gets the second endpoint of the loop.
TEST(KeyframeDecider, RevisitCreatesNothingWithoutTimeWindow)
{
  const auto o = defaultOptions();
  mola::KeyframeDecider d(o);

  double t = 0;
  run(d, o, 0.0, 10.0, 0.25, t, 0.1);
  const size_t nAfterFirstPass = d.size();

  // Drive back over the very same places, much later:
  t += 1000.0;
  const size_t createdOnRevisit = run(d, o, 10.0, 0.0, -0.25, t, 0.1);

  EXPECT_EQ(createdOnRevisit, 0u);
  EXPECT_EQ(d.size(), nAfterFirstPass);
}

// With the temporal window enabled, the same revisit does create keyframes,
// since the first pass' ones are no longer visible to the policy.
TEST(KeyframeDecider, RevisitCreatesKeyframesWithTimeWindow)
{
  auto o = defaultOptions();
  o.nearby_keyframe_time_window = 20.0;

  mola::KeyframeDecider d(o);

  double t = 0;
  run(d, o, 0.0, 10.0, 0.25, t, 0.1);
  const size_t nAfterFirstPass = d.size();
  EXPECT_EQ(nAfterFirstPass, 9u);

  // Same places, long after the window:
  t += 1000.0;
  const size_t createdOnRevisit = run(d, o, 10.0, 0.0, -0.25, t, 0.1);

  // The revisit is sampled as densely as the first pass, minus the keyframe
  // the first pass created out of an empty map: the second pass starts right
  // on top of the last keyframe of the first one.
  EXPECT_EQ(createdOnRevisit, nAfterFirstPass - 1);
  EXPECT_EQ(d.size(), nAfterFirstPass + createdOnRevisit);
}

// A stationary vehicle must NOT emit one keyframe per time window: the newest
// keyframe is kept in the window whatever its age.
TEST(KeyframeDecider, StationaryVehicleDoesNotAccumulate)
{
  auto o = defaultOptions();
  o.nearby_keyframe_time_window = 5.0;

  mola::KeyframeDecider d(o);

  double t = 0;
  ASSERT_TRUE(d.check(o, at(0), t).create);
  d.insert(at(0), t);

  // Parked for 10 minutes:
  for (int i = 0; i < 6000; i++) {
    t += 0.1;
    EXPECT_FALSE(d.check(o, at(0), t).create);
  }

  EXPECT_EQ(d.size(), 1u);
}

// The time window must not disturb normal forward driving: same spacing as the
// spatial policy.
TEST(KeyframeDecider, TimeWindowKeepsForwardSpacing)
{
  auto o = defaultOptions();
  o.nearby_keyframe_time_window = 20.0;

  mola::KeyframeDecider d(o);

  double t = 0;
  const size_t n = run(d, o, 0.0, 10.0, 0.25, t, 0.1);

  EXPECT_EQ(n, 9u);  // same as the purely spatial policy
}

// Rotating in place beyond the angular threshold also creates keyframes.
TEST(KeyframeDecider, RotationThreshold)
{
  const auto o = defaultOptions();
  mola::KeyframeDecider d(o);

  d.insert(mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 0, 0, 0, 0), 0.0);

  const auto smallTurn =
    mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 0, mrpt::DEG2RAD(10), 0, 0);
  EXPECT_FALSE(d.check(o, smallTurn, 1.0).create);

  const auto bigTurn = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 0, mrpt::DEG2RAD(45), 0, 0);
  EXPECT_TRUE(d.check(o, bigTurn, 1.0).create);
}

// min_nearby_poses_occupied > 1 asks for several keyframes per spot (used by
// non-repetitive-scan lidars).
// Under the spatial policy the count is delegated to
// SearchablePoseList::countNearby(), so the option is a documented no-op when
// building against an older mola_pose_list without it:
#if defined(MOLA_POSE_LIST_HAS_KFM_POSE_PLUMBING)
TEST(KeyframeDecider, MinNearbyPosesOccupied)
{
  auto o = defaultOptions();
  o.min_nearby_poses_occupied = 3;

  mola::KeyframeDecider d(o);

  // Three keyframes at the very same place, then no more:
  for (int i = 0; i < 3; i++) {
    ASSERT_TRUE(d.check(o, at(0), i).create) << "i=" << i;
    d.insert(at(0), i);
  }
  EXPECT_FALSE(d.check(o, at(0), 4.0).create);
  EXPECT_EQ(d.size(), 3u);
}
#endif

// The temporal policy must NOT bypass the occupancy count: with both options
// set, a spot still needs min_nearby_poses_occupied keyframes, and no more.
TEST(KeyframeDecider, MinNearbyPosesOccupiedWithTimeWindow)
{
  auto o = defaultOptions();
  o.min_nearby_poses_occupied = 3;
  o.nearby_keyframe_time_window = 5.0;

  mola::KeyframeDecider d(o);

  double t = 0;

  // Same place, well inside the window: 3 keyframes, then no more.
  for (int i = 0; i < 3; i++, t += 0.5) {
    ASSERT_TRUE(d.check(o, at(0), t).create) << "i=" << i;
    d.insert(at(0), t);
  }
  EXPECT_FALSE(d.check(o, at(0), t).create);
  EXPECT_EQ(d.size(), 3u);

  // Past the window, all but the newest keyframe age out, so the occupancy
  // count drops below the threshold and the spot admits keyframes again:
  t += 100.0;
  EXPECT_TRUE(d.check(o, at(0), t).create);
}

// measure_from_last_kf_only ignores all but the last keyframe, so a revisit
// creates keyframes even without the time window.
TEST(KeyframeDecider, MeasureFromLastOnly)
{
  auto o = defaultOptions();
  o.measure_from_last_kf_only = true;

  mola::KeyframeDecider d(o);

  double t = 0;
  run(d, o, 0.0, 10.0, 0.25, t, 0.1);

  // Back to the origin: far from the LAST keyframe (at 10 m), so it creates.
  EXPECT_TRUE(d.check(o, at(0), t).create);
}

// measure_from_last_kf_only must keep meaning "only the last one" when the
// temporal window is also enabled, instead of being silently ignored.
TEST(KeyframeDecider, MeasureFromLastOnlyWithTimeWindow)
{
  auto o = defaultOptions();
  o.measure_from_last_kf_only = true;
  o.nearby_keyframe_time_window = 20.0;

  mola::KeyframeDecider d(o);

  double t = 0;
  run(d, o, 0.0, 10.0, 0.25, t, 0.1);

  // Only the last keyframe is ever retained as a decision reference:
  EXPECT_EQ(d.activeSize(), 1u);

  // Back to the origin: far from that last keyframe (at 10 m), so it creates.
  EXPECT_TRUE(d.check(o, at(0), t).create);
}

// Keyframes explicitly dropped by the local-map cleanup must stop taking part
// in the test under the temporal policy too.
TEST(KeyframeDecider, RemoveAllFartherThanUnderTimeWindow)
{
  auto o = defaultOptions();
  o.nearby_keyframe_time_window = 1000.0;  // long enough to retain everything

  mola::KeyframeDecider d(o);

  double t = 0;
  run(d, o, 0.0, 10.0, 0.25, t, 0.1);
  EXPECT_EQ(d.size(), 9u);

  // Keep only what is within 2 m of the far end of the run:
  d.removeAllFartherThan(at(10.0), 2.0);
  EXPECT_LT(d.size(), 9u);

  // The origin's keyframe is gone, so that spot admits a keyframe again:
  EXPECT_TRUE(d.check(o, at(0), t).create);
}

// Out-of-order timestamps must not defeat the window pruning, which pops from
// the front and therefore needs the entries to stay sorted in time.
TEST(KeyframeDecider, NonMonotonicTimestamps)
{
  auto o = defaultOptions();
  o.nearby_keyframe_time_window = 5.0;

  mola::KeyframeDecider d(o);

  d.insert(at(0), 100.0);
  d.insert(at(2), 99.0);  // jitter: older than the previous one
  d.insert(at(4), 101.0);

  // Well past the window: all but the newest entry must have aged out, so the
  // origin (whose keyframe is gone) asks for a new one.
  EXPECT_TRUE(d.check(o, at(0), 200.0).create);
  EXPECT_EQ(d.activeSize(), 1u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
