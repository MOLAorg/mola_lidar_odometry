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
 * Unit tests for LidarOdometry::vehiclePitchRollFromMapGravity().
 *
 * This is the seam where a map-frame gravity estimate is turned into the ICP
 * prior's pitch/roll. The invariant under test is the one an earlier bug got
 * wrong: the result must describe the VEHICLE's tilt with respect to true
 * vertical, INDEPENDENTLY of how tilted the map frame itself is. Returning the
 * map tilt instead made the prior fight the (unchanged) local map and diverged
 * real runs, but left every math-level unit test green -- hence this test at
 * the integration seam. No datasets needed; all synthetic.
 */

#include <gtest/gtest.h>
#include <mola_lidar_odometry/LidarOdometry.h>
#include <mrpt/poses/CPose3D.h>

#include <cmath>

using mrpt::math::TVector3D;
using mrpt::poses::CPose3D;

namespace
{
constexpr double G = 9.81;

// Gravity vector expressed in a frame obtained by rotating the world by `R`.
TVector3D gravityInFrame(const CPose3D & R_frame_from_world)
{
  const TVector3D g_world{0, 0, -G};
  return R_frame_from_world.rotateVector(g_world);
}

// Build a rotation-only pose from yaw/pitch/roll in DEGREES.
CPose3D ypr(double yaw_deg, double pitch_deg, double roll_deg)
{
  return CPose3D(
    0, 0, 0, mrpt::DEG2RAD(yaw_deg), mrpt::DEG2RAD(pitch_deg), mrpt::DEG2RAD(roll_deg));
}
}  // namespace

// A level vehicle in a TILTED map must report zero tilt: the map's own tilt
// must NOT leak into the vehicle prior. This is the exact regression.
TEST(VehiclePitchRollFromMapGravity, LevelVehicleInTiltedMap_reportsLevel)
{
  for (const auto & mapTilt : {ypr(30, 4, -3), ypr(-120, -6, 2), ypr(0, 8, 8)}) {
    // Vehicle level w.r.t. the world => its attitude in the map frame IS the
    // map tilt, and gravity in the map is the tilted gravity.
    const CPose3D vehicleInMap = mapTilt;
    const TVector3D g_map = gravityInFrame(mapTilt);

    const auto pr =
      mola::LidarOdometry::vehiclePitchRollFromMapGravity(g_map, vehicleInMap);
    ASSERT_TRUE(pr.has_value());

    EXPECT_NEAR(mrpt::RAD2DEG(pr->first), 0.0, 1e-9) << "map tilt " << mapTilt.asString();
    EXPECT_NEAR(mrpt::RAD2DEG(pr->second), 0.0, 1e-9) << "map tilt " << mapTilt.asString();
  }
}

// A vehicle tilted by a known amount in a LEVEL map must recover that tilt.
TEST(VehiclePitchRollFromMapGravity, TiltedVehicleInLevelMap_recoversTilt)
{
  const double truePitch = 5.0;
  const double trueRoll = -7.0;

  const CPose3D vehicleInMap = ypr(40, truePitch, trueRoll);
  const TVector3D g_map = gravityInFrame(ypr(0, 0, 0));  // level map

  const auto pr = mola::LidarOdometry::vehiclePitchRollFromMapGravity(g_map, vehicleInMap);
  ASSERT_TRUE(pr.has_value());

  EXPECT_NEAR(mrpt::RAD2DEG(pr->first), truePitch, 1e-6);
  EXPECT_NEAR(mrpt::RAD2DEG(pr->second), trueRoll, 1e-6);
}

// The core invariant: the recovered tilt equals the vehicle's WORLD tilt,
// regardless of the map tilt. Compose a known world tilt with several map
// tilts; the answer must not move.
TEST(VehiclePitchRollFromMapGravity, RecoversWorldTilt_independentOfMapTilt)
{
  const double truePitch = 6.0;
  const double trueRoll = 3.0;
  const CPose3D vehicleInWorld = ypr(0, truePitch, trueRoll);

  for (const auto & mapTilt : {ypr(0, 0, 0), ypr(70, 5, -4), ypr(-30, -8, 6)}) {
    // Vehicle attitude in the map frame = map<-world composed with world<-vehicle.
    const CPose3D vehicleInMap = mapTilt + vehicleInWorld;
    const TVector3D g_map = gravityInFrame(mapTilt);

    const auto pr =
      mola::LidarOdometry::vehiclePitchRollFromMapGravity(g_map, vehicleInMap);
    ASSERT_TRUE(pr.has_value());

    EXPECT_NEAR(mrpt::RAD2DEG(pr->first), truePitch, 1e-6) << "map tilt " << mapTilt.asString();
    EXPECT_NEAR(mrpt::RAD2DEG(pr->second), trueRoll, 1e-6) << "map tilt " << mapTilt.asString();
  }
}

// Yaw must not affect pitch/roll: a pure heading change leaves the tilt alone.
TEST(VehiclePitchRollFromMapGravity, InvariantToYaw)
{
  const TVector3D g_map = gravityInFrame(ypr(0, 0, 0));

  for (const double yaw : {0.0, 45.0, 123.0, -160.0}) {
    const auto pr =
      mola::LidarOdometry::vehiclePitchRollFromMapGravity(g_map, ypr(yaw, 4.0, -2.0));
    ASSERT_TRUE(pr.has_value());
    EXPECT_NEAR(mrpt::RAD2DEG(pr->first), 4.0, 1e-6) << "yaw " << yaw;
    EXPECT_NEAR(mrpt::RAD2DEG(pr->second), -2.0, 1e-6) << "yaw " << yaw;
  }
}

// A degenerate (near-zero) gravity vector yields nullopt rather than a NaN.
TEST(VehiclePitchRollFromMapGravity, DegenerateGravity_returnsNullopt)
{
  EXPECT_FALSE(
    mola::LidarOdometry::vehiclePitchRollFromMapGravity({0, 0, 0}, ypr(0, 0, 0)).has_value());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
