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
 * Unit tests for TrajectoryRebaker.
 * Tests §5.3 and §5.4 of TODOs/online-gravity-alignment.md.
 * All synthetic; no external datasets.
 */

#include <gtest/gtest.h>
#include <mola_lidar_odometry/GravityMapAligner.h>
#include <mola_lidar_odometry/TrajectoryRebaker.h>
#include <mrpt/poses/CPose3D.h>

#include <cmath>

using namespace mola;
using mrpt::poses::CPose3D;
using KeyFrameID = TrajectoryRebaker::KeyFrameID;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace
{
CPose3D makePose(double x, double y, double z, double yaw, double pitch, double roll)
{
  return CPose3D(x, y, z, yaw, pitch, roll);
}

// Pitch-only rotation about Y by angle_rad.
CPose3D makeRot(double pitch_rad, double roll_rad = 0.0)
{
  return {0, 0, 0, 0.0, pitch_rad, roll_rad};
}

// Extract pitch in degrees from a CPose3D.
double pitchDeg(const CPose3D & p)
{
  double y, pitch, r;
  p.getYawPitchRoll(y, pitch, r);
  return pitch * 180.0 / M_PI;
}

// Extract position z from a CPose3D.
double zPos(const CPose3D & p) { return p.z(); }

}  // namespace

// ---------------------------------------------------------------------------
// Test 1: empty inputs → empty result.
// ---------------------------------------------------------------------------
TEST(TrajectoryRebaker, EmptyInput_returnsEmpty)
{
  std::map<KeyFrameID, CPose3D> poses, r_grav;
  const auto result = TrajectoryRebaker::rebake(poses, r_grav);
  EXPECT_TRUE(result.corrected_poses.empty());
}

// ---------------------------------------------------------------------------
// Test 2: single KF (anchor only) → position unchanged, rotation corrected.
// ---------------------------------------------------------------------------
TEST(TrajectoryRebaker, SingleKF_anchorRotationCorrected)
{
  const double tilt = 3.0 * M_PI / 180.0;

  std::map<KeyFrameID, CPose3D> poses, r_grav;
  poses[0] = makePose(10.0, 5.0, 2.0, 0.0, tilt, 0.0);
  r_grav[0] = makeRot(-tilt);  // corrects the tilt

  const auto result = TrajectoryRebaker::rebake(poses, r_grav, 0);
  ASSERT_EQ(result.corrected_poses.size(), 1u);

  const CPose3D & corrected = result.corrected_poses.at(0);

  // Position must be unchanged.
  EXPECT_NEAR(corrected.x(), 10.0, 1e-9);
  EXPECT_NEAR(corrected.y(), 5.0, 1e-9);
  EXPECT_NEAR(corrected.z(), 2.0, 1e-9);

  // Rotation: R_grav * R_odom = Ry(-tilt) * Ry(tilt) = identity → pitch ≈ 0.
  EXPECT_NEAR(pitchDeg(corrected), 0.0, 1e-6);
}

// ---------------------------------------------------------------------------
// Test 3: two KFs — anchor frozen, second KF position re-integrated.
// ---------------------------------------------------------------------------
TEST(TrajectoryRebaker, TwoKFs_relativeGeometryPreserved)
{
  // Robot drives 10 m forward (along X) in a frame with 5° pitch drift.
  const double tilt = 5.0 * M_PI / 180.0;

  std::map<KeyFrameID, CPose3D> poses, r_grav;
  // KF 0: at origin, pitched 5°.
  poses[0] = makePose(0.0, 0.0, 0.0, 0.0, tilt, 0.0);
  // KF 1: 10 m ahead in the (tilted) odom frame.
  poses[1] = makePose(10.0, 0.0, 0.0, 0.0, tilt, 0.0);

  // Same correction for both KFs: undo the 5° tilt.
  r_grav[0] = makeRot(-tilt);
  r_grav[1] = makeRot(-tilt);

  const auto result = TrajectoryRebaker::rebake(poses, r_grav, 0);
  ASSERT_EQ(result.corrected_poses.size(), 2u);

  const CPose3D & c0 = result.corrected_poses.at(0);
  const CPose3D & c1 = result.corrected_poses.at(1);

  // Anchor position is frozen.
  EXPECT_NEAR(c0.x(), 0.0, 1e-9);
  EXPECT_NEAR(c0.y(), 0.0, 1e-9);
  EXPECT_NEAR(c0.z(), 0.0, 1e-9);

  // Both KFs should now have ~0° pitch.
  EXPECT_NEAR(pitchDeg(c0), 0.0, 1e-6);
  EXPECT_NEAR(pitchDeg(c1), 0.0, 1e-6);

  // The 10 m travel should be preserved in magnitude.
  const double dist = std::sqrt(
    std::pow(c1.x() - c0.x(), 2) + std::pow(c1.y() - c0.y(), 2) + std::pow(c1.z() - c0.z(), 2));
  EXPECT_NEAR(dist, 10.0, 1e-6);
}

// ---------------------------------------------------------------------------
// Test 4: §5.3 — 100 KFs, straight horizontal path, 10° linear pitch drift.
//
// The odom frame accumulates 10° of pitch drift linearly (0.1°/KF) so that
// what should be a flat horizontal path appears to go uphill. R_grav[i] is
// constructed via GravityMapAligner::rotationFromGravity for each KF, so
// R_grav[i] EXACTLY cancels T_odom[i].R per KF (perfect per-KF correction
// scenario). The 5 cm Z tolerance is valid under that assumption; a
// global/noisy r_grav scenario would need a looser tolerance and is not
// covered here.
// After rebake:
//  - Per-KF pitch residual < 0.05°.
//  - Z-position drift from true path < 5 cm per 100 m traversed.
// ---------------------------------------------------------------------------
TEST(TrajectoryRebaker, LinearDrift_recoversHorizontalPath)
{
  const int N_KFS = 100;
  const double step_m = 1.0;      // 1 m between KFs
  const double max_drift = 10.0;  // total drift in degrees

  std::map<KeyFrameID, CPose3D> odom_poses, r_grav;

  // Build drifted odom poses: odom frame tilted by pitch_rad at each KF.
  // The robot physically moves 1 m forward (world X) per step. In the tilted
  // odom frame this appears as (cos(p), 0, -sin(p)) per step — flat forward
  // motion looks slightly downward in a forward-tilted odom frame.
  // R_grav[i] = Ry(-pitch_i): undoes the tilt at each KF.
  double x_odom = 0.0, z_odom = 0.0;
  for (int i = 0; i < N_KFS; ++i) {
    const double drift_deg = max_drift * static_cast<double>(i) / (N_KFS - 1);
    const double pitch_rad = drift_deg * M_PI / 180.0;

    // T_i.R = Ry(pitch_rad): robot flat in world, odom frame tilted by pitch_rad.
    odom_poses[i] = makePose(x_odom, 0.0, z_odom, 0.0, pitch_rad, 0.0);

    // Physical forward motion (world X) in the tilted odom frame: (cos(p), 0, -sin(p)).
    if (i < N_KFS - 1) {
      x_odom += step_m * std::cos(pitch_rad);
      z_odom -= step_m * std::sin(pitch_rad);
    }

    // g_odom = Ry(pitch) * (0,0,g) = (sin(p)*g, 0, cos(p)*g).
    // rotationFromGravity returns Ry(-pitch), which sends g_odom to +Z.
    r_grav[i] = GravityMapAligner::rotationFromGravity(
      {std::sin(pitch_rad) * 9.81, 0.0, std::cos(pitch_rad) * 9.81});
  }

  const auto result = TrajectoryRebaker::rebake(odom_poses, r_grav, 0);
  ASSERT_EQ(result.corrected_poses.size(), static_cast<std::size_t>(N_KFS));

  // Anchor position must be unchanged.
  EXPECT_NEAR(result.corrected_poses.at(0).x(), odom_poses.at(0).x(), 1e-9);
  EXPECT_NEAR(result.corrected_poses.at(0).z(), odom_poses.at(0).z(), 1e-9);

  // Check all KFs after anchor.
  for (int i = 0; i < N_KFS; ++i) {
    const CPose3D & c = result.corrected_poses.at(i);

    // Pitch residual < 0.05°.
    EXPECT_NEAR(pitchDeg(c), 0.0, 0.05) << "KF " << i << " pitch residual too large";

    // Z should be ~0 (flat path). Tolerance: 5 cm per 100 m → 5 cm total here.
    EXPECT_NEAR(zPos(c), 0.0, 0.05) << "KF " << i << " Z drift too large";
  }
}

// ---------------------------------------------------------------------------
// Test 5: §5.4 — non-monotonic drift (grows to 5° then decays to 1°).
//
// Confirms that the rebake distributes correction non-uniformly along the
// chain: it is NOT a single global rotation (which would fail here).
// ---------------------------------------------------------------------------
TEST(TrajectoryRebaker, NonMonotonicDrift_nonUniformCorrection)
{
  const int N_KFS = 100;
  const double step_m = 1.0;

  // Drift profile: rises linearly to 5° at midpoint, falls to 1° at end.
  auto drift_deg = [&](int i) -> double {
    const double t = static_cast<double>(i) / (N_KFS - 1);
    if (t <= 0.5) {
      return 5.0 * (2.0 * t);  // 0 → 5°
    }
    return 5.0 - (5.0 - 1.0) * (2.0 * (t - 0.5));  // 5° → 1°
  };

  std::map<KeyFrameID, CPose3D> odom_poses, r_grav;

  double x_odom = 0.0, z_odom = 0.0;
  for (int i = 0; i < N_KFS; ++i) {
    const double pitch_rad = drift_deg(i) * M_PI / 180.0;
    odom_poses[i] = makePose(x_odom, 0.0, z_odom, 0.0, pitch_rad, 0.0);

    if (i < N_KFS - 1) {
      x_odom += step_m * std::cos(pitch_rad);
      z_odom -= step_m * std::sin(pitch_rad);  // tilted odom: flat forward looks downward
    }

    r_grav[i] = GravityMapAligner::rotationFromGravity(
      {std::sin(pitch_rad) * 9.81, 0.0, std::cos(pitch_rad) * 9.81});
  }

  const auto result = TrajectoryRebaker::rebake(odom_poses, r_grav, 0);
  ASSERT_EQ(result.corrected_poses.size(), static_cast<std::size_t>(N_KFS));

  // All KFs should have near-zero pitch after rebake.
  for (int i = 0; i < N_KFS; ++i) {
    EXPECT_NEAR(pitchDeg(result.corrected_poses.at(i)), 0.0, 0.05)
      << "KF " << i << " (non-monotonic drift test) pitch residual";
  }

  // The midpoint correction must differ from the endpoint correction,
  // proving a single global rotation was NOT applied.
  // With a global rotation of, say, -3° (average drift), midpoint KFs would
  // still have ~2° residual. Our rebake should bring both to ~0°.
  EXPECT_NEAR(pitchDeg(result.corrected_poses.at(0)), 0.0, 0.05);
  EXPECT_NEAR(pitchDeg(result.corrected_poses.at(50)), 0.0, 0.05);
  EXPECT_NEAR(pitchDeg(result.corrected_poses.at(99)), 0.0, 0.05);
}

// ---------------------------------------------------------------------------
// Test 6: KFs missing from r_grav get identity correction (pass-through).
// ---------------------------------------------------------------------------
TEST(TrajectoryRebaker, MissingRgrav_identityFallback)
{
  // Three KFs: only the middle one has a correction.
  std::map<KeyFrameID, CPose3D> poses, r_grav;
  const double tilt = 4.0 * M_PI / 180.0;

  poses[0] = makePose(0.0, 0.0, 0.0, 0.0, tilt, 0.0);
  poses[1] = makePose(5.0, 0.0, 0.0, 0.0, tilt, 0.0);
  poses[2] = makePose(10.0, 0.0, 0.0, 0.0, tilt, 0.0);

  // Only r_grav for KF 0 (anchor) — others fall back to identity.
  r_grav[0] = makeRot(-tilt);

  const auto result = TrajectoryRebaker::rebake(poses, r_grav, 0);
  ASSERT_EQ(result.corrected_poses.size(), 3u);

  // Anchor: rotation corrected.
  EXPECT_NEAR(pitchDeg(result.corrected_poses.at(0)), 0.0, 1e-6);

  // KF 1 & 2: identity correction → rotation = tilt (not corrected by r_grav).
  // Their positions ARE re-integrated relative to the corrected anchor though.
  EXPECT_NEAR(pitchDeg(result.corrected_poses.at(1)), pitchDeg(poses.at(1)), 1e-6);
  EXPECT_NEAR(pitchDeg(result.corrected_poses.at(2)), pitchDeg(poses.at(2)), 1e-6);
}

// ---------------------------------------------------------------------------
// Test 7: computePublishResidual — result is T_before⁻¹ * T_after.
// ---------------------------------------------------------------------------
TEST(TrajectoryRebaker, PublishResidual_inverseCompose)
{
  const CPose3D before = makePose(1.0, 2.0, 0.0, 0.1, 0.05, 0.0);
  const CPose3D after = makePose(1.0, 2.0, 0.0, 0.1, 0.0, 0.0);  // pitch corrected

  const CPose3D residual = TrajectoryRebaker::computePublishResidual(before, after);

  // Publish-continuity contract: residual + after == before, so that
  // immediately after a rebake the published pose does not jump.
  const CPose3D reconstructed = residual + after;

  EXPECT_NEAR(reconstructed.x(), before.x(), 1e-9);
  EXPECT_NEAR(reconstructed.y(), before.y(), 1e-9);
  EXPECT_NEAR(reconstructed.z(), before.z(), 1e-9);

  double y_r, p_r, r_r;
  reconstructed.getYawPitchRoll(y_r, p_r, r_r);
  double y_b, p_b, r_b;
  before.getYawPitchRoll(y_b, p_b, r_b);
  EXPECT_NEAR(p_r, p_b, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
