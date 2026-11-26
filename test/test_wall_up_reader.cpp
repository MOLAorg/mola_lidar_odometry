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
#include <mola_lidar_odometry/WallUpReader.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/maps/CSimplePointsMap.h>

#if __has_include(<mp2p_icp_filters/estimate_up_from_normals.h>)
#include <mp2p_icp_filters/estimate_up_from_normals.h>
#endif

#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <vector>

#if defined(MP2P_ICP_HAS_ESTIMATE_UP_COVARIANCE)

namespace
{
// A 12 x 8 x 3 m room in the world frame: four walls, floor and ceiling,
// sampled every 5 cm with 5 mm noise. `walls_only_x`: only the two walls
// normal to X (a corridor), and no floor or ceiling.
std::vector<mrpt::math::TPoint3D> room(bool walls_only_x = false)
{
  std::mt19937 rng(1);
  std::normal_distribution<double> n(0.0, 0.005);
  std::vector<mrpt::math::TPoint3D> pts;
  const double step = 0.05;
  for (double a = 0; a <= 8.0; a += step) {
    for (double z = 0; z <= 3.0; z += step) {
      pts.emplace_back(0.0 + n(rng), a, z);
      pts.emplace_back(12.0 + n(rng), a, z);
    }
  }
  if (walls_only_x) {
    return pts;
  }
  for (double a = 0; a <= 12.0; a += step) {
    for (double z = 0; z <= 3.0; z += step) {
      pts.emplace_back(a, 0.0 + n(rng), z);
      pts.emplace_back(a, 8.0 + n(rng), z);
    }
    for (double b = 0; b <= 8.0; b += step) {
      pts.emplace_back(a, b, 0.0 + n(rng));
      pts.emplace_back(a, b, 3.0 + n(rng));
    }
  }
  return pts;
}

// The room as a scan from `vehicle` (true pose in the world): points in the
// vehicle frame.
mrpt::maps::CPointsMap::ConstPtr scan(
  const std::vector<mrpt::math::TPoint3D> & world, const mrpt::poses::CPose3D & vehicle)
{
  auto pc = std::make_shared<mrpt::maps::CSimplePointsMap>();
  for (const auto & p : world) {
    const auto l = vehicle.inverseComposePoint(p);
    pc->insertPointFast(static_cast<float>(l.x), static_cast<float>(l.y), static_cast<float>(l.z));
  }
  return pc;
}
}  // namespace

TEST(WallUpReader, ReadsTheTiltOfTheTrajectoryFrame)
{
  // The trajectory frame leans 2 deg / -1.5 deg against the world: the walls
  // it registers lean with it, and the reading is that frame's view of up.
  const auto tilt =
    mrpt::poses::CPose3D::FromYawPitchRoll(0.0, mrpt::DEG2RAD(2.0), mrpt::DEG2RAD(-1.5));
  const auto world = room();
  mola::WallUpReader::Parameters p;
  p.window = 20.0;
  p.period = 5.0;
  p.min_points = 500;
  mola::WallUpReader reader(p);
  for (int k = 0; k < 6; k++) {
    const auto vehicle = mrpt::poses::CPose3D(3.0 + k, 4.0, 1.0, 0.3 * k, 0, 0);
    reader.add_cloud(k, tilt + vehicle, scan(world, vehicle));
  }
  const auto r = reader.read(5.0);
  ASSERT_TRUE(r.has_value());
  const auto u = tilt.rotateVector({0, 0, 1});
  EXPECT_NEAR(r->up.x, u.x, mrpt::DEG2RAD(0.05));
  EXPECT_NEAR(r->up.y, u.y, mrpt::DEG2RAD(0.05));
  EXPECT_GT(r->walls, 100U);
  EXPECT_GT(r->flats, 100U);
  EXPECT_GT(r->up_cov(0, 0), 0.0);
  EXPECT_LT(std::sqrt(r->up_cov(0, 0)), mrpt::DEG2RAD(0.2));

  // Not again before the period:
  EXPECT_FALSE(reader.read(7.0).has_value());
  EXPECT_EQ(reader.stats().readings, 1U);
  // Nor after it, with no new scan (e.g. standing still), counted once per
  // period however often read() is called:
  for (const double t : {12.0, 12.5, 13.0, 16.9, 17.0}) {
    EXPECT_FALSE(reader.read(t).has_value());
  }
  EXPECT_EQ(reader.stats().no_new_scans, 2U);
  reader.add_cloud(
    17.5, tilt + mrpt::poses::CPose3D(4, 4, 1, 0, 0, 0),
    scan(world, mrpt::poses::CPose3D(4, 4, 1, 0, 0, 0)));
  EXPECT_FALSE(reader.read(18.0).has_value());  // the period spent at 17
  EXPECT_TRUE(reader.read(22.0).has_value());
}

TEST(WallUpReader, DefaultConstructedReads)
{
  // The default constructor is the parameterized one with the defaults.
  mola::WallUpReader reader;
  EXPECT_EQ(reader.parameters().window, mola::WallUpReader::Parameters{}.window);
  const auto world = room();
  const auto vehicle = mrpt::poses::CPose3D(4, 4, 1, 0, 0, 0);
  reader.add_cloud(0.0, vehicle, scan(world, vehicle));
  EXPECT_TRUE(reader.read(1.0).has_value());
}

TEST(WallUpReader, RequiresAPositivePeriod)
{
  mola::WallUpReader::Parameters p;
  p.period = 0.0;
  EXPECT_ANY_THROW(mola::WallUpReader{p});
}

TEST(WallUpReader, RequiresPositiveCovarianceCells)
{
  mola::WallUpReader::Parameters p;
  p.cluster_size = 0.0;
  EXPECT_ANY_THROW(mola::WallUpReader{p});
}

TEST(WallUpReader, OverlappingWindowsInflateTheCovariance)
{
  // The same scans read with period = window (no overlap) and with a period
  // four times shorter: the covariance grows by that factor.
  const auto world = room();
  const auto vehicle = mrpt::poses::CPose3D(4, 4, 1, 0.2, 0, 0);
  double var[2] = {0, 0};
  for (int k = 0; k < 2; k++) {
    mola::WallUpReader::Parameters p;
    p.window = 8.0;
    p.period = k == 0 ? 8.0 : 2.0;
    p.min_points = 500;
    mola::WallUpReader reader(p);
    reader.add_cloud(0.0, vehicle, scan(world, vehicle));
    const auto r = reader.read(1.0);
    ASSERT_TRUE(r.has_value());
    var[k] = r->up_cov(0, 0);
  }
  EXPECT_NEAR(var[1] / var[0], 4.0, 1e-9);
}

TEST(WallUpReader, NonFinitePointsAreDropped)
{
  const auto world = room();
  const auto vehicle = mrpt::poses::CPose3D(4, 4, 1, 0, 0, 0);
  auto pc = std::make_shared<mrpt::maps::CSimplePointsMap>(
    *std::dynamic_pointer_cast<const mrpt::maps::CSimplePointsMap>(scan(world, vehicle)));
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (int i = 0; i < 100; i++) {
    pc->insertPointFast(nan, nan, nan);
    pc->insertPointFast(1.0f, nan, 0.5f);
  }
  mola::WallUpReader::Parameters p;
  p.min_points = 500;
  mola::WallUpReader reader(p);
  reader.add_cloud(0.0, mrpt::poses::CPose3D(), pc);
  const auto r = reader.read(1.0);
  ASSERT_TRUE(r.has_value());
  EXPECT_TRUE(std::isfinite(r->up.x) && std::isfinite(r->up.y) && std::isfinite(r->up.z));
  EXPECT_NEAR(r->up.z, 1.0, 1e-4);
}

TEST(WallUpReader, OldScansLeaveTheWindow)
{
  mola::WallUpReader::Parameters p;
  p.window = 10.0;
  p.min_points = 500;
  mola::WallUpReader reader(p);
  const auto world = room();
  reader.add_cloud(
    0.0, mrpt::poses::CPose3D(), scan(world, mrpt::poses::CPose3D(4, 4, 1, 0, 0, 0)));
  // 30 s later the only scan is out of the window: too few points.
  EXPECT_FALSE(reader.read(30.0).has_value());
  EXPECT_EQ(reader.stats().too_few_points, 1U);
}

TEST(WallUpReader, ACorridorWithoutFloorsIsNotObservableWallsOnly)
{
  // Walls of a single direction and no floor: up is free to turn about their
  // normal, so walls-only gives no reading.
  mola::WallUpReader::Parameters p;
  p.flats_weight = 0.0;
  p.min_points = 500;
  mola::WallUpReader reader(p);
  const auto world = room(true);
  reader.add_cloud(
    0.0, mrpt::poses::CPose3D(), scan(world, mrpt::poses::CPose3D(6, 4, 1, 0, 0, 0)));
  EXPECT_FALSE(reader.read(1.0).has_value());
  EXPECT_EQ(reader.stats().unobservable, 1U);
}

#else
TEST(WallUpReader, NeedsTheUpCovarianceApi)
{
  EXPECT_ANY_THROW(mola::WallUpReader(mola::WallUpReader::Parameters{}));
  EXPECT_ANY_THROW(mola::WallUpReader{});
}
#endif
