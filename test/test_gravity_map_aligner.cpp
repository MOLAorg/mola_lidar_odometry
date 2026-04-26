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
 * Unit tests for GravityMapAligner.
 * Tests §5.1 and §5.2 of TODOs/online-gravity-alignment.md.
 *
 * No external datasets needed — all synthetic.
 */

#include <gtest/gtest.h>
#include <mola_lidar_odometry/GravityMapAligner.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/poses/CPose3D.h>

#include <cmath>
#include <random>

using namespace mola;
using mrpt::math::TVector3D;
using mrpt::poses::CPose3D;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace
{
// Pitch rotation about Y by angle_rad (positive = nose up).
CPose3D makePitch(double angle_rad)
{
  mrpt::math::CQuaternionDouble q;
  q.fromRodriguesVector(mrpt::math::TVector3D(0.0, angle_rad, 0.0));
  return {q, 0, 0, 0};
}

// Body-frame proper acceleration (= -gravity, the value an accelerometer
// measures) when the robot is pitched by angle_rad about +Y (nose up).
// Accelerometer convention: with the robot upright, gravity is (0,0,-g) in
// world, the accelerometer reads (0,0,+g) in body (pointing "up" through the
// floor). Pitched by angle_rad:
//   a_body.x = g * sin(pitch_rad)
//   a_body.y = 0
//   a_body.z = +g * cos(pitch_rad)
TVector3D bodyGravity(double pitch_rad, double g = 9.81)
{
  return {g * std::sin(pitch_rad), 0.0, g * std::cos(pitch_rad)};
}

// Angle between vector v and +Z, in degrees.
double angleToPlusZ_deg(const TVector3D & v)
{
  const double n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  if (n < 1e-12) {
    return 0.0;
  }
  const double cz = std::max(-1.0, std::min(1.0, v.z / n));
  return std::acos(cz) * 180.0 / M_PI;
}

// After applying R_corr to kf_poses, check that every mapped gravity vector
// is within tol_deg of +Z.
double maxGravityErrorDeg(
  const std::map<GravityMapAligner::KeyFrameID, CPose3D> & kf_poses,
  const GravityMapAligner & aligner, const CPose3D & R_corr)
{
  double max_err = 0;
  for (const auto & [id, s] : aligner.samples()) {
    auto itp = kf_poses.find(id);
    if (itp == kf_poses.end()) {
      continue;
    }
    const CPose3D corrected = R_corr + itp->second;  // R_corr left-multiplied
    const TVector3D g_world = corrected.rotateVector(s.a_body);
    max_err = std::max(max_err, angleToPlusZ_deg(g_world));
  }
  return max_err;
}
}  // namespace

// ---------------------------------------------------------------------------
// Test 1: below-threshold → nullopt
// ---------------------------------------------------------------------------
TEST(GravityMapAligner, BelowMinKeyframes_returnsNullopt)
{
  GravityMapAligner::Params p;
  p.min_keyframes = 10;
  GravityMapAligner aligner(p);

  std::map<GravityMapAligner::KeyFrameID, CPose3D> poses;
  for (uint64_t i = 0; i < 9; ++i) {
    aligner.onNewKeyframeRaw(i, {{0, 0, 9.81}, 0.01, 10});
    poses[i] = CPose3D::Identity();
  }

  EXPECT_FALSE(aligner.estimateGravityVector(poses).has_value());
}

// ---------------------------------------------------------------------------
// Test 2: identity world (no tilt) → correction is near-identity
// ---------------------------------------------------------------------------
TEST(GravityMapAligner, ZeroTilt_identityCorrection)
{
  GravityMapAligner::Params p;
  p.min_keyframes = 5;
  GravityMapAligner aligner(p);

  // All KFs perfectly upright: a_body = (0,0,g) in body frame, identity pose.
  std::map<GravityMapAligner::KeyFrameID, CPose3D> poses;
  for (uint64_t i = 0; i < 20; ++i) {
    aligner.onNewKeyframeRaw(i, {{0.0, 0.0, 9.81}, 0.02, 10});
    poses[i] = CPose3D::Identity();
  }

  auto corr = aligner.estimateGlobalCorrection(poses);
  ASSERT_TRUE(corr.has_value());

  // The correction should be nearly identity: all angles < 0.01°.
  double yaw = 0, pitch = 0, roll = 0;
  corr->getYawPitchRoll(yaw, pitch, roll);

  EXPECT_NEAR(roll * 180.0 / M_PI, 0.0, 0.05);
  EXPECT_NEAR(pitch * 180.0 / M_PI, 0.0, 0.05);
  EXPECT_NEAR(yaw * 180.0 / M_PI, 0.0, 0.05);
}

// ---------------------------------------------------------------------------
// Test 3: known 2° pitch tilt → estimator converges to within 0.1°
//
// Scenario: all KF poses are identity (odom == world), body-frame gravity
// is (sin(tilt)*g, 0, cos(tilt)*g) + noise. This models a 2° tilt of the
// odom frame relative to world gravity. The estimator should recover a
// correction with pitch ≈ -tilt_deg, roll ≈ 0°, yaw = 0° exactly.
// We test the ESTIMATOR accuracy (pool mean), not per-sample noise residuals.
// ---------------------------------------------------------------------------
TEST(GravityMapAligner, KnownPitchTilt_converges)
{
  const double tilt_deg = 2.0;
  const double tilt_rad = tilt_deg * M_PI / 180.0;

  GravityMapAligner::Params p;
  p.min_keyframes = 10;
  p.huber_threshold_mps2 = 0.5;
  p.irls_iterations = 3;
  GravityMapAligner aligner(p);

  std::mt19937 rng(42);
  std::normal_distribution<double> noise(0.0, 0.04);  // ~0.04 m/s²

  std::map<GravityMapAligner::KeyFrameID, CPose3D> poses;
  for (uint64_t i = 0; i < 30; ++i) {
    // g_odom = I * a_body ≈ (sin(tilt)*g, 0, cos(tilt)*g). Pool mean converges
    // to within 0.04/sqrt(30) ≈ 0.007° of the true direction.
    const TVector3D a_clean = bodyGravity(tilt_rad);
    GravityMapAligner::KFAccSample s;
    s.a_body = {a_clean.x + noise(rng), a_clean.y + noise(rng), a_clean.z + noise(rng)};
    s.a_body_std = 0.04;
    s.n_samples = 50;
    aligner.onNewKeyframeRaw(i, s);
    poses[i] = CPose3D::Identity();
  }

  auto corr = aligner.estimateGlobalCorrection(poses);
  ASSERT_TRUE(corr.has_value());

  // R_corr = rotationFromGravity(g_odom_mean) sends g_odom_mean to +Z.
  // For g_odom_mean ≈ (sin(tilt),0,cos(tilt))*g, the extracted pitch is ≈ -tilt_deg.
  double yaw = 0, pitch = 0, roll = 0;
  corr->getYawPitchRoll(yaw, pitch, roll);
  EXPECT_NEAR(pitch * 180.0 / M_PI, -tilt_deg, 0.1)
    << "Pitch correction should cancel the known tilt";
  EXPECT_NEAR(roll * 180.0 / M_PI, 0.0, 0.1);
  EXPECT_NEAR(yaw * 180.0 / M_PI, 0.0, 1e-8);

  // Sanity: applying R_corr to each pooled accel should put it near +Z.
  // This is a per-KF MAX (not a pool mean), so per-sample accel noise of
  // ~0.04 m/s² translates to ~0.04/9.81 rad ≈ 0.23° per axis, and the max
  // over 30 KFs can reasonably reach ~1°.
  EXPECT_LT(maxGravityErrorDeg(poses, aligner, *corr), 1.0);
}

// ---------------------------------------------------------------------------
// Test 4: 20 % outlier KFs (high linear accel) → Huber keeps estimate stable
// ---------------------------------------------------------------------------
TEST(GravityMapAligner, HuberRobustness_outliersIgnored)
{
  const double tilt_deg = 2.0;
  const double tilt_rad = tilt_deg * M_PI / 180.0;

  GravityMapAligner::Params p;
  p.min_keyframes = 10;
  p.huber_threshold_mps2 = 0.5;
  p.max_kf_acc_std_mps2 = 1.5;
  p.irls_iterations = 3;
  GravityMapAligner aligner(p);

  std::mt19937 rng(7);
  std::normal_distribution<double> small_noise(0.0, 0.04);
  std::normal_distribution<double> big_noise(0.0, 3.0);  // outlier

  std::map<GravityMapAligner::KeyFrameID, CPose3D> poses;
  for (uint64_t i = 0; i < 50; ++i) {
    const TVector3D a_clean = bodyGravity(tilt_rad);
    const bool outlier = (i % 5 == 0);  // exactly 20 %
    const double std_used = outlier ? 2.5 : 0.04;
    auto & nd = outlier ? big_noise : small_noise;
    GravityMapAligner::KFAccSample s;
    s.a_body = {a_clean.x + nd(rng), a_clean.y + nd(rng), a_clean.z + nd(rng)};
    s.a_body_std = std_used;
    s.n_samples = 50;
    aligner.onNewKeyframeRaw(i, s);
    poses[i] = CPose3D::Identity();
  }

  auto corr = aligner.estimateGlobalCorrection(poses);
  ASSERT_TRUE(corr.has_value());

  // Despite 20 % outlier KFs the estimator should still recover pitch ≈ -tilt_deg.
  // The per-std down-weighting (1e-3 for std_i > max_kf_acc_std_mps2) combined
  // with Huber IRLS suppresses the large-noise KFs.
  double yaw = 0, pitch = 0, roll = 0;
  corr->getYawPitchRoll(yaw, pitch, roll);
  EXPECT_NEAR(pitch * 180.0 / M_PI, -tilt_deg, 0.2)
    << "Huber test: pitch estimate should survive 20% outlier KFs";
  EXPECT_NEAR(yaw * 180.0 / M_PI, 0.0, 1e-8);
}

// ---------------------------------------------------------------------------
// Test 5: onKeyframeDropped removes the KF; dropping enough falls below min
// ---------------------------------------------------------------------------
TEST(GravityMapAligner, DropKeyframe_poolShrinks)
{
  GravityMapAligner::Params p;
  p.min_keyframes = 10;
  GravityMapAligner aligner(p);

  std::map<GravityMapAligner::KeyFrameID, CPose3D> poses;
  for (uint64_t i = 0; i < 12; ++i) {
    aligner.onNewKeyframeRaw(i, {{0, 0, 9.81}, 0.01, 10});
    poses[i] = CPose3D::Identity();
  }
  EXPECT_TRUE(aligner.estimateGravityVector(poses).has_value());

  // Drop until we're below threshold.
  aligner.onKeyframeDropped(0);
  aligner.onKeyframeDropped(1);
  aligner.onKeyframeDropped(2);  // now 9 remain
  EXPECT_EQ(aligner.size(), 9u);
  EXPECT_FALSE(aligner.estimateGravityVector(poses).has_value());
}

// ---------------------------------------------------------------------------
// Test 6: rotationFromGravity is pitch/roll-only (yaw == 0)
// ---------------------------------------------------------------------------
TEST(GravityMapAligner, rotationFromGravity_noYaw)
{
  // Arbitrary tilted vector.
  const TVector3D g{0.3, 0.1, 9.8};
  const CPose3D R = GravityMapAligner::rotationFromGravity(g);

  double yaw = 0, pitch = 0, roll = 0;
  R.getYawPitchRoll(yaw, pitch, roll);
  EXPECT_NEAR(yaw * 180.0 / M_PI, 0.0, 1e-8) << "rotationFromGravity must not produce yaw";

  // After applying R, the vector should point to +Z.
  mrpt::math::TPoint3D p_local(g.x, g.y, g.z);
  mrpt::math::TPoint3D p_world, orig_world;
  R.composePoint(p_local, p_world);
  R.composePoint(mrpt::math::TPoint3D(0, 0, 0), orig_world);
  const TVector3D r{p_world.x - orig_world.x, p_world.y - orig_world.y, p_world.z - orig_world.z};
  EXPECT_LT(angleToPlusZ_deg(r), 0.001) << "rotationFromGravity must send g to +Z";
}

// ---------------------------------------------------------------------------
// Test 7: per-KF window correction
// ---------------------------------------------------------------------------
TEST(GravityMapAligner, PerKFCorrection_windowedResult)
{
  // 20 KFs all with the same 2° pitch — every KF should get a correction.
  const double tilt_rad = 2.0 * M_PI / 180.0;

  GravityMapAligner::Params p;
  p.min_keyframes = 3;
  GravityMapAligner aligner(p);

  std::map<GravityMapAligner::KeyFrameID, CPose3D> poses;
  for (uint64_t i = 0; i < 20; ++i) {
    aligner.onNewKeyframeRaw(i, {bodyGravity(tilt_rad), 0.02, 20});
    poses[i] = makePitch(tilt_rad);
  }

  const auto per_kf = aligner.estimatePerKFCorrection(poses, /*P=*/5, /*min_pool=*/3);
  EXPECT_EQ(per_kf.size(), 20u) << "All 20 KFs should have a per-KF correction";

  // Each per-KF correction must have zero yaw.
  for (const auto & [id, R] : per_kf) {
    double yaw = 0, pitch = 0, roll = 0;
    R.getYawPitchRoll(yaw, pitch, roll);
    EXPECT_NEAR(yaw * 180.0 / M_PI, 0.0, 1e-7) << "Per-KF correction must have zero yaw, KF " << id;
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
