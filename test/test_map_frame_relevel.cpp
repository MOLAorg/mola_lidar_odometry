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
 * @file   test_map_frame_relevel.cpp
 * @brief  The map-frame change must be a pure gauge change
 * @author Jose Luis Blanco Claraco
 */

#include <gtest/gtest.h>
#include <mola_lidar_odometry/KeyframeDecider.h>
#include <mola_lidar_odometry/MapFrameRelevel.h>
// Same optional-dependency probe LidarOdometry.h uses, so this test still
// builds against a mola_imu_preintegration without the map-gravity estimator.
#if __has_include(<mola_imu_preintegration/MapGravityEstimator.h>)
#include <mola_imu_preintegration/MapGravityEstimator.h>
#define TEST_HAS_MAP_GRAVITY_ESTIMATOR 1
#endif
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/Lie/SO.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{
/** A moderately curved walk with changing attitude, so that a bug that only
 *  cancels on straight, level motion cannot hide. */
std::vector<mrpt::poses::CPose3D> makeTrajectory()
{
  std::vector<mrpt::poses::CPose3D> out;
  for (int i = 0; i < 25; i++) {
    const double s = 0.1 * i;
    out.push_back(mrpt::poses::CPose3D::FromXYZYawPitchRoll(
      3.0 * s, 1.5 * s * s, 0.4 * std::sin(s), 0.3 * s, 0.05 * std::cos(s), -0.07 * s));
  }
  return out;
}

/** A tilt of ~9 deg with a roll component, i.e. the size of the map lean this
 *  feature exists to remove. */
mrpt::poses::CPose3D theCorrection()
{
  return mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(7.0), mrpt::DEG2RAD(-5.5));
}

double rotAngle(const mrpt::poses::CPose3D & p)
{
  return mrpt::poses::Lie::SO<3>::log(p.getRotationMatrix()).norm();
}

void expectSamePose(const mrpt::poses::CPose3D & a, const mrpt::poses::CPose3D & b, double tol)
{
  const auto d = a - b;
  EXPECT_NEAR(d.translation().norm(), 0.0, tol);
  EXPECT_NEAR(rotAngle(d), 0.0, tol);
}
}  // namespace

TEST(MapFrameRelevel, TrajectoryRelativePosesArePreserved)
{
  const auto poses = makeTrajectory();
  const auto b = theCorrection();

  mrpt::poses::CPose3DInterpolator traj;
  for (size_t i = 0; i < poses.size(); i++) {
    traj.insert(mrpt::Clock::fromDouble(1000.0 + 0.1 * i), poses[i].asTPose());
  }

  mola::transform_to_new_map_frame(traj, b);

  ASSERT_EQ(traj.size(), poses.size());

  size_t i = 0;
  std::vector<mrpt::poses::CPose3D> newPoses;
  for (const auto & [t, p] : traj) {
    const mrpt::poses::CPose3D pNew(p);
    // Global poses moved exactly by the gauge rotation:
    expectSamePose(pNew, b + poses[i], 1e-9);
    newPoses.push_back(pNew);
    i++;
  }

  // ... and every relative pose is untouched, which is what "gauge" means:
  for (size_t k = 1; k < poses.size(); k++) {
    expectSamePose(newPoses[k] - newPoses[k - 1], poses[k] - poses[k - 1], 1e-9);
  }
}

TEST(MapFrameRelevel, SimplemapKeyframePosesAndCovariances)
{
  const auto poses = makeTrajectory();
  const auto b = theCorrection();

  mrpt::maps::CSimpleMap sm;
  for (const auto & p : poses) {
    auto pdf = mrpt::poses::CPose3DPDFGaussian::Create();
    pdf->mean = p;
    pdf->cov.setDiagonal(1e-4);
    // A twist in the VEHICLE frame, which a map-frame change must not touch:
    sm.insert(pdf, mrpt::obs::CSensoryFrame::Create(), mrpt::math::TTwist3D(1, 0, 0, 0, 0, 0.2));
  }

  mola::transform_to_new_map_frame(sm, b);

  for (size_t i = 0; i < poses.size(); i++) {
    const auto & [pose, sf, twist] = sm.get(i);
    expectSamePose(pose->getMeanVal(), b + poses[i], 1e-9);
    ASSERT_TRUE(twist.has_value());
    EXPECT_NEAR(twist->vx, 1.0, 1e-12);
    EXPECT_NEAR(twist->wz, 0.2, 1e-12);
  }

  // Relative poses preserved:
  for (size_t k = 1; k < poses.size(); k++) {
    const auto & [pk, sfk, twk] = sm.get(k);
    const auto & [pj, sfj, twj] = sm.get(k - 1);
    expectSamePose(pk->getMeanVal() - pj->getMeanVal(), poses[k] - poses[k - 1], 1e-9);
  }
}

TEST(MapFrameRelevel, MetricMapPointsRotateAndKeepTheirShape)
{
  const auto b = theCorrection();

  mp2p_icp::metric_map_t mm;
  auto pts = mrpt::maps::CSimplePointsMap::Create();
  std::vector<mrpt::math::TPoint3D> original;
  for (int i = 0; i < 50; i++) {
    const mrpt::math::TPoint3D p(0.7 * i, -0.3 * i + 2.0, 0.2 * std::sin(0.3 * i));
    original.push_back(p);
    pts->insertPoint(p);
  }
  mm.layers["localmap"] = pts;

  mola::transform_to_new_map_frame(mm, b);

  auto out = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(mm.layers.at("localmap"));
  ASSERT_TRUE(out);
  ASSERT_EQ(out->size(), original.size());

  for (size_t i = 0; i < original.size(); i++) {
    mrpt::math::TPoint3D q;
    out->getPoint(i, q.x, q.y, q.z);
    const auto expected = b.composePoint(original[i]);
    // Point maps store single-precision coordinates:
    EXPECT_NEAR((q - expected).norm(), 0.0, 1e-4);
  }

  // Inter-point distances (the map's actual content) are untouched:
  for (size_t i = 1; i < original.size(); i++) {
    mrpt::math::TPoint3D q0;
    mrpt::math::TPoint3D q1;
    out->getPoint(i - 1, q0.x, q0.y, q0.z);
    out->getPoint(i, q1.x, q1.y, q1.z);
    EXPECT_NEAR((q1 - q0).norm(), (original[i] - original[i - 1]).norm(), 1e-4);
  }
}

TEST(MapFrameRelevel, MetricMapRefusesGeoreferencedMaps)
{
  mp2p_icp::metric_map_t mm;
  mm.layers["localmap"] = mrpt::maps::CSimplePointsMap::Create();
  mm.georeferencing.emplace();

  EXPECT_THROW(mola::transform_to_new_map_frame(mm, theCorrection()), std::exception);
}

// Exercises KeyframeDecider::transform_left_multiply(), which is itself only
// compiled against a mola_pose_list that provides the underlying call.
#if defined(MOLA_POSE_LIST_HAS_TRANSFORM_LEFT_MULTIPLY)
TEST(MapFrameRelevel, KeyframeDeciderDecisionsAreUnchanged)
{
  const auto poses = makeTrajectory();
  const auto b = theCorrection();

  mola::KeyframeDecisionOptions opts;
  opts.min_translation_between_keyframes = 1.0;
  opts.min_rotation_between_keyframes = 15.0;

  mola::KeyframeDecider before(opts);
  mola::KeyframeDecider after(opts);

  // Feed both with the same (old-frame) history, then rotate one of them:
  for (size_t i = 0; i < 10; i++) {
    before.insert(poses[i], 100.0 + i);
    after.insert(poses[i], 100.0 + i);
  }
  after.transform_left_multiply(b);

  // Every query, asked in its own frame, must yield the same decision:
  for (size_t i = 10; i < poses.size(); i++) {
    const auto dOld = before.check(opts, poses[i], 100.0 + i);
    const auto dNew = after.check(opts, b + poses[i], 100.0 + i);

    EXPECT_EQ(dOld.create, dNew.create) << " at i=" << i;
    EXPECT_NEAR(dOld.translation, dNew.translation, 1e-9);
    EXPECT_NEAR(dOld.rotation, dNew.rotation, 1e-9);
  }
}
#endif

TEST(MapFrameRelevel, MapFrameVelocitiesRotateWithTheFrame)
{
  // The odometry keeps its twist in the VEHICLE frame; the map-frame velocity
  // is derived as R*v. After the gauge change the attitude becomes b*R, so the
  // map-frame velocity must come out rotated by exactly b, with no change in
  // magnitude. This is the invariant the map-gravity estimator relies on.
  const auto b = theCorrection();
  const mrpt::poses::CPose3D R = mrpt::poses::CPose3D::FromYawPitchRoll(0.4, -0.2, 0.15);
  const mrpt::math::TVector3D vLocal(2.0, -0.5, 0.3);

  const auto vMapOld = R.rotateVector(vLocal);
  const auto vMapNew = (b + R).rotateVector(vLocal);

  EXPECT_NEAR((vMapNew - b.rotateVector(vMapOld)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(vMapNew.norm(), vMapOld.norm(), 1e-12);
}

#if defined(TEST_HAS_MAP_GRAVITY_ESTIMATOR)
TEST(MapFrameRelevel, TheCorrectionLevelsTheMap)
{
  // End-to-end on the actual quantity consumed: the estimator's `correction`
  // must take the measured map-frame gravity onto -Z, by the smallest possible
  // rotation.
  const mrpt::math::TVector3D gMap(1.21, -0.83, -9.70);

  const auto R = mola::imu::MapGravityEstimator::correction_from_gravity(gMap);
  const mrpt::poses::CPose3D b =
    mrpt::poses::CPose3D::FromRotationAndTranslation(R, mrpt::math::TVector3D(0, 0, 0));

  const auto gLevel = b.rotateVector(gMap);
  EXPECT_NEAR(gLevel.x, 0.0, 1e-9);
  EXPECT_NEAR(gLevel.y, 0.0, 1e-9);
  EXPECT_NEAR(gLevel.z, -gMap.norm(), 1e-9);

  // It is the geodesic (minimal-angle) rotation between the two directions, so
  // its magnitude is exactly the tilt it removes and nothing more. Note that
  // its Euler yaw is NOT identically zero: for a rotation about an axis in the
  // XY plane, the ZYX yaw is a second-order term (here ~0.3 deg for a ~9.5 deg
  // tilt). Yaw is unobservable anyway, so this is another gauge freedom.
  const double tiltBetween = std::acos(std::clamp(-gMap.z / gMap.norm(), -1.0, 1.0));
  EXPECT_NEAR(rotAngle(b), tiltBetween, 1e-9);
}
#endif
