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
#include <mola_lidar_odometry/SimplemapLeveling.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <cmath>
#include <utility>
#include <vector>

namespace
{
// A level circle, 20 m radius at 2 m/s
mrpt::poses::CPose3D true_pose(double t)
{
  const double a = 0.1 * t;
  return mrpt::poses::CPose3D::FromXYZYawPitchRoll(
    20.0 * std::sin(a), 20.0 * (1.0 - std::cos(a)), 0.0, a, 0.0, 0.0);
}

// A keyframe whose sensory frame carries a "metadata" observation stamped `t`
mrpt::maps::CSimpleMap::Keyframe keyframe(double t, const mrpt::poses::CPose3D & p, bool meta)
{
  auto sf = mrpt::obs::CSensoryFrame::Create();
  auto first = mrpt::obs::CObservationComment::Create();
  first->timestamp = mrpt::Clock::fromDouble(t - 0.05);
  first->sensorLabel = "lidar";
  sf->insert(first);
  if (meta) {
    auto m = mrpt::obs::CObservationComment::Create();
    m->timestamp = mrpt::Clock::fromDouble(t);
    m->sensorLabel = "metadata";
    sf->insert(m);
  }
  return {mrpt::poses::CPose3DPDFGaussian::Create(p), sf};
}
}  // namespace

TEST(SimplemapLeveling, KeyframeStampPrefersTheMetadata)
{
  const auto with = keyframe(10.0, {}, true);
  const auto without = keyframe(10.0, {}, false);
  EXPECT_NEAR(mrpt::Clock::toDouble(*mola::keyframe_stamp(*with.sf)), 10.0, 1e-6);
  EXPECT_NEAR(mrpt::Clock::toDouble(*mola::keyframe_stamp(*without.sf)), 9.95, 1e-6);
}

TEST(SimplemapLeveling, KeyframesMoveWithTheLeveledPath)
{
  // A path and its leveled version, which differ by a rotation of the frame
  // about the origin that grows with time; keyframes sit off the path (the
  // odometry's own poses, not the fused ones): each moves by the path's
  // correction at its stamp, as a left composition in the common frame.
  mrpt::poses::CPose3DInterpolator before;
  mrpt::poses::CPose3DInterpolator after;
  const auto corr = [](double t) {
    return mrpt::poses::CPose3D::FromYawPitchRoll(
      0.0, mrpt::DEG2RAD(0.02 * t), mrpt::DEG2RAD(-0.01 * t));
  };
  for (int k = 0; k <= 100; k++) {
    const double t = k;
    const auto p = true_pose(t);
    before.insert(mrpt::Clock::fromDouble(t), p.asTPose());
    after.insert(mrpt::Clock::fromDouble(t), (corr(t) + p).asTPose());
  }
  const mrpt::poses::CPose3D offset(0.3, -0.2, 0.1, 0.05, 0.01, -0.02);
  mrpt::maps::CSimpleMap sm;
  for (const double t : {10.0, 50.0, 120.0}) {
    auto kf = keyframe(t, true_pose(t) + offset, true);
    sm.insert(kf.pose, kf.sf);
  }
  sm.insert(mrpt::poses::CPose3DPDFGaussian::Create(offset), mrpt::obs::CSensoryFrame::Create());
  const auto shared = sm.get(0).pose;

  EXPECT_EQ(mola::level_simplemap_keyframes(sm, before, after), 3U);
  for (const auto & [i, t] :
       std::vector<std::pair<int, double>>{{0, 10.0}, {1, 50.0}, {2, 100.0}}) {
    const double tk = i == 2 ? 120.0 : t;  // beyond the path: its last correction
    const auto expected = corr(t) + true_pose(tk) + offset;
    const auto got = sm.get(static_cast<std::size_t>(i)).pose->getMeanVal();
    EXPECT_NEAR((got.asVectorVal() - expected.asVectorVal()).norm(), 0.0, 1e-6) << "kf " << i;
  }
  // The keyframe without a stamp stays; the original pose object is untouched.
  EXPECT_NEAR(
    (sm.get(3).pose->getMeanVal().asVectorVal() - offset.asVectorVal()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
    (shared->getMeanVal().asVectorVal() - (true_pose(10.0) + offset).asVectorVal()).norm(), 0.0,
    1e-12);
}

TEST(SimplemapLeveling, PathsOverDisjointTimesHoldTheirOwnEnds)
{
  // before spans [0, 10] s and after [20, 30] s: a keyframe at 15 s takes
  // before(10) and after(20), each held at its own end.
  mrpt::poses::CPose3DInterpolator before;
  mrpt::poses::CPose3DInterpolator after;
  const auto corr = mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(1.0), 0.0);
  for (int k = 0; k <= 10; k++) {
    before.insert(mrpt::Clock::fromDouble(k), true_pose(k).asTPose());
    after.insert(mrpt::Clock::fromDouble(k + 20.0), (corr + true_pose(k + 20.0)).asTPose());
  }
  mrpt::maps::CSimpleMap sm;
  const auto kf = keyframe(15.0, true_pose(15.0), true);
  sm.insert(kf.pose, kf.sf);
  EXPECT_EQ(mola::level_simplemap_keyframes(sm, before, after), 1U);
  const auto expected = (corr + true_pose(20.0)) + (-true_pose(10.0)) + true_pose(15.0);
  const auto got = sm.get(0).pose->getMeanVal();
  EXPECT_NEAR((got.asVectorVal() - expected.asVectorVal()).norm(), 0.0, 1e-6);
}
