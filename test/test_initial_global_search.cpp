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
 * @file   test_initial_global_search.cpp
 * @brief  Initial localization from a state estimator (the real smoother) fed
 *         with a poor GNSS, with and without a heading from an IMU, on
 *         synthetic worlds and sensor data generated on the fly.
 */

#include <gtest/gtest.h>
#include <mola_kernel/MinimalModuleContainer.h>
#include <mola_lidar_odometry/LidarOdometry.h>
#include <mola_state_estimation_simple/StateEstimationSimple.h>
#include <mola_yaml/yaml_helpers.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/core/get_env.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/topography/conversions.h>

#include <chrono>
#include <cstdlib>
#include <future>
#include <thread>

#include "synthetic_world.h"

// The global search and these scenarios need recent enough dependencies:
#if __has_include(<mola_relocalization/relocalization.h>)
#include <mola_relocalization/relocalization.h>
#endif
#if __has_include(<mola_state_estimation_smoother/Parameters.h>)
#include <mola_state_estimation_smoother/Parameters.h>
#endif

namespace
{
using namespace lo_test;

void set_env(const std::string & name, const std::string & value)
{
  ::setenv(name.c_str(), value.c_str(), 1);
}

mrpt::containers::yaml lo_config()
{
  return mola::load_yaml_file(mrpt::get_env<std::string>("LO_PIPELINE_YAML"));
}

// ---------------------------------------------------------------------------
// Map building: LO in mapping mode along a known path, odometry = truth
// ---------------------------------------------------------------------------
std::string build_map(const World & world, const std::vector<mrpt::poses::CPose3D> & path)
{
  set_env("MOLA_MAPPING_ENABLED", "true");
  set_env("MOLA_LO_PUBLISH_REF_FRAME", "odom");
  set_env("MOLA_LOAD_MM", "");
  set_env("MOLA_LO_INITIAL_LOCALIZATION_METHOD", "InitLocalization::FixedPose");
  const auto & p0 = path.front();
  set_env("MOLA_INITIAL_X", std::to_string(p0.x()));
  set_env("MOLA_INITIAL_Y", std::to_string(p0.y()));
  set_env("MOLA_INITIAL_YAW", std::to_string(mrpt::RAD2DEG(p0.yaw())));

  auto liodom = mola::LidarOdometry::Create();
  auto stateEstimator = mola::state_estimation_simple::StateEstimationSimple::Create();
  mola::MinimalModuleContainer moduleContainer = {{liodom, stateEstimator}};

  liodom->setMinLoggingLevel(mrpt::system::LVL_WARN);
  liodom->initialize(lo_config());
  liodom->params_.simplemap.generate = false;
  liodom->params_.estimated_trajectory.output_file.clear();
  liodom->params_.lidar_sensor_labels.assign(1, std::regex("lidar"));

  stateEstimator->setMinLoggingLevel(mrpt::system::LVL_WARN);
  stateEstimator->initialize(
    mola::load_yaml_file(mrpt::get_env<std::string>("LO_STATE_ESTIM_SIMPLE_YAML")));

  double t = 0;
  for (const auto & p : path) {
    liodom->onNewObservation(odometry(mrpt::poses::CPose2D(p), t));
    liodom->onNewObservation(scan(world, p, t));
    wait_idle(*liodom);
    t += 0.1;
  }

  // map_save() waits for the request to run in spinOnce():
  const auto base = mrpt::system::getTempFileName();
  auto saving = std::async(std::launch::async, [&]() { return liodom->map_save(base); });
  while (saving.wait_for(std::chrono::milliseconds(5)) != std::future_status::ready) {
    liodom->spinOnce();
  }
  const auto ret = saving.get();
  EXPECT_TRUE(ret.success) << ret.error_message;

  // Geo-reference it, as a map for GNSS-aided localization:
  mp2p_icp::metric_map_t mm;
  EXPECT_TRUE(mm.load_from_file(base + ".mm"));
  std::cout << "Map built: " << mm.contents_summary() << "\n";
  const auto g = make_georef();
  mm.georeferencing.emplace();
  mm.georeferencing->geo_coord = g.geo_coord;
  mm.georeferencing->T_enu_to_map = g.T_enu_to_map;
  EXPECT_TRUE(mm.save_to_file(base + ".mm"));
  return base;
}

// A path from (x0,y0) to (x1,y1) and back, every 0.2 m:
std::vector<mrpt::poses::CPose3D> straight_path(double x0, double y0, double x1, double y1)
{
  std::vector<mrpt::poses::CPose3D> path;
  const double len = mrpt::math::TPoint2D(x1 - x0, y1 - y0).norm();
  const double yaw = std::atan2(y1 - y0, x1 - x0);
  const auto n = static_cast<int>(len / 0.2);
  for (int i = 0; i <= n; i++) {
    const double f = static_cast<double>(i) / n;
    path.push_back(mrpt::poses::CPose3D::FromXYZYawPitchRoll(
      x0 + f * (x1 - x0), y0 + f * (y1 - y0), 0, yaw, 0, 0));
  }
  return path;
}

// ---------------------------------------------------------------------------
// Localization: the real smoother with GNSS (and maybe IMU), LO on the map
// ---------------------------------------------------------------------------
struct Scenario
{
  World world;
  std::string mapBase;         //!< empty: no prebuilt map, mapping from scratch
  mrpt::poses::CPose3D truth;  //!< vehicle pose at the start
  double speed = 0;            //!< [m/s] straight ahead from `truth`
  double lidarStart = 0;       //!< [s] LiDAR scans start arriving this late
  mrpt::math::TPoint2D gnssBias;
  double gnssSigma = 0.5;
  bool withImu = true;
  bool globalSearch = true;
  double minHalfWidthXY = 3.0;
  double duration = 20.0;  // [s]
};

struct Result
{
  /// LO's first and last poses, if localized
  std::optional<mrpt::poses::CPose3D> first, last;
};

Result localize(const Scenario & sc)
{
  const bool fromScratch = sc.mapBase.empty();
  // Without a map, the geo-reference can only come from the estimator:
  set_env("MOLA_ESTIMATE_GEO_REF", fromScratch ? "true" : "false");
  set_env("MOLA_MAPPING_ENABLED", fromScratch ? "true" : "false");
  set_env("MOLA_LOAD_MM", fromScratch ? "" : sc.mapBase + ".mm");
  set_env("MOLA_LO_INITIAL_LOCALIZATION_METHOD", "InitLocalization::FromStateEstimator");
  set_env("MOLA_LO_INIT_GLOBAL_SEARCH", sc.globalSearch ? "true" : "false");
  set_env("MOLA_LO_INIT_GLOBAL_SEARCH_MIN_XY", std::to_string(sc.minHalfWidthXY));
  // Poses in the geo-referenced map frame, as for GNSS-aided localization:
  set_env("MOLA_LO_PUBLISH_REF_FRAME", "map");
  set_env("MOLA_LO_INIT_SE_MAX_POS_SIGMA", "0.5");

  // The smoother: relocalization in a geo-referenced map, synchronous for
  // repeatability. Without a heading source, it converges on position alone.
  set_env("MOLA_ASYNC_BACKEND", "false");
  set_env("MOLA_IMU_ATTITUDE_AZIMUTH_OFFSET", "-90.0");  // ENU, like simulators
  set_env("MOLA_SE_CONVERGENCE_ORI_SIGMA", sc.withImu ? "5.0" : "360.0");

  auto liodom = mola::LidarOdometry::Create();
  // By class name, through its interface (its own headers clash with LO's):
  auto smoother = std::dynamic_pointer_cast<mola::NavStateFilter>(
    mrpt::rtti::classFactory("mola::state_estimation_smoother::StateEstimationSmoother"));
  EXPECT_TRUE(smoother);
  mola::MinimalModuleContainer moduleContainer = {{liodom, smoother}};

  smoother->setMinLoggingLevel(
    mrpt::get_env<bool>("VERBOSE", false) ? mrpt::system::LVL_DEBUG : mrpt::system::LVL_WARN);
  smoother->initialize(
    mola::load_yaml_file(mrpt::get_env<std::string>("LO_STATE_ESTIM_SMOOTHER_YAML")));
  if (!fromScratch) {
    smoother->set_geo_reference(make_georef());
  }

  liodom->setMinLoggingLevel(
    mrpt::get_env<bool>("VERBOSE", false) ? mrpt::system::LVL_DEBUG : mrpt::system::LVL_INFO);
  liodom->initialize(lo_config());
  liodom->params_.simplemap.generate = false;
  liodom->params_.estimated_trajectory.output_file.clear();
  liodom->params_.lidar_sensor_labels.assign(1, std::regex("lidar"));

  const auto georef = make_georef();
  // 10 Hz lidar, wheel odometry and IMU; 1 Hz GNSS:
  const auto steps = static_cast<int>(sc.duration * 10);
  for (int i = 0; i < steps; i++) {
    const double dt = 0.1 * i;
    const double t = 1000.0 + dt;
    const auto vehicle = sc.truth + mrpt::poses::CPose3D::FromTranslation(sc.speed * dt, 0, 0);
    smoother->fuse_odometry(*odometry(mrpt::poses::CPose2D(sc.speed * dt, 0, 0), t), "odom_wheels");
    if (sc.withImu) {
      smoother->fuse_imu(*imu(vehicle, t));
    }
    if (i % 10 == 0) {
      smoother->fuse_gnss(*gnss(georef, vehicle, sc.gnssBias, sc.gnssSigma, t));
    }
    smoother->spinOnce();
    if (dt < sc.lidarStart) {
      continue;
    }
    liodom->onNewObservation(scan(sc.world, vehicle, t));
    wait_idle(*liodom);
    liodom->spinOnce();
  }

  Result r;
  const auto traj = liodom->estimatedTrajectory();
  if (mrpt::get_env<bool>("PRINT_TRAJ", false)) {
    size_t k = 0;
    for (const auto & [t, p] : traj) {
      if (k++ % 10 == 0) {
        std::cout << "traj t=" << mrpt::Clock::toDouble(t) << " " << p.asString() << "\n";
      }
    }
  }
  if (!traj.empty()) {
    r.first = mrpt::poses::CPose3D(traj.begin()->second);
    r.last = mrpt::poses::CPose3D(traj.rbegin()->second);
  }
  return r;
}

double xy_error(const mrpt::poses::CPose3D & a, const mrpt::poses::CPose3D & b)
{
  return mrpt::math::TPoint2D(a.x() - b.x(), a.y() - b.y()).norm();
}

double yaw_error_deg(const mrpt::poses::CPose3D & a, const mrpt::poses::CPose3D & b)
{
  return mrpt::RAD2DEG(std::abs(mrpt::math::wrapToPi(a.yaw() - b.yaw())));
}

// One map of the hall for all the tests that use it:
const std::string & hall_map()
{
  // Along a line clear of walls and pillars:
  static const std::string base = build_map(unique_hall(), straight_path(3, 14.5, 37, 14.5));
  return base;
}

Scenario hall_scenario()
{
  Scenario sc;
  sc.world = unique_hall();
  sc.mapBase = hall_map();
  sc.truth = mrpt::poses::CPose3D::FromXYZYawPitchRoll(18.0, 14.0, 0, mrpt::DEG2RAD(30.0), 0, 0);
  sc.gnssBias = {1.8, -1.2};
  return sc;
}
}  // namespace

class InitialGlobalSearch : public ::testing::Test
{
protected:
  void SetUp() override
  {
#if !defined(MOLA_RELOCALIZATION_HAS_CANDIDATES) || \
  !defined(MOLA_STATE_ESTIMATION_SMOOTHER_HAS_ORIENTATION_GAUGE)
    GTEST_SKIP() << "mola_relocalization or mola_state_estimation_smoother too old";
#endif
  }
};

// The GNSS bias puts the state estimator ~2 m off: without a search, LO
// starts from there.
TEST_F(InitialGlobalSearch, BiasedGnssWithoutSearchStartsOff)
{
  auto sc = hall_scenario();
  sc.globalSearch = false;
  sc.duration = 8.0;
  const auto r = localize(sc);
  ASSERT_TRUE(r.first.has_value());
  std::cout << "Without search, first pose: " << r.first->asString()
            << " truth: " << sc.truth.asString() << "\n";
  EXPECT_GT(xy_error(*r.first, sc.truth), 1.0);
}

void expect_localized(const Result & r, const mrpt::poses::CPose3D & truth)
{
  ASSERT_TRUE(r.first.has_value());
  ASSERT_TRUE(r.last.has_value());
  std::cout << "First pose: " << r.first->asString() << " last: " << r.last->asString()
            << " truth: " << truth.asString() << "\n";
  for (const auto & p : {*r.first, *r.last}) {
    EXPECT_LT(xy_error(p, truth), 0.1);
    EXPECT_LT(yaw_error_deg(p, truth), 1.0);
  }
}

TEST_F(InitialGlobalSearch, BiasedGnssWithImu)
{
  expect_localized(localize(hall_scenario()), hall_scenario().truth);
}

// No heading source at all: yaw is searched too.
TEST_F(InitialGlobalSearch, BiasedGnssWithoutImu)
{
  auto sc = hall_scenario();
  sc.withImu = false;
  sc.truth = mrpt::poses::CPose3D::FromXYZYawPitchRoll(26.0, 12.0, 0, mrpt::DEG2RAD(-120.0), 0, 0);
  expect_localized(localize(sc), sc.truth);
}

// Identical rows, GNSS off by more than one row: the match is ambiguous, and
// LO must not report a pose.
TEST_F(InitialGlobalSearch, AmbiguousRowsAreNotLocalized)
{
  const double pitch = 1.8;
  Scenario sc;
  sc.world = endless_rows(pitch);
  // Mapped beyond the sensor range around the test pose:
  sc.mapBase = build_map(sc.world, straight_path(-20, 0.9, 20, 0.9));
  sc.truth = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.0, 0.9, 0, 0, 0, 0);
  sc.gnssBias = {0.5, 2.0};
  sc.duration = 12.0;
  const auto r = localize(sc);
  if (r.last) {
    std::cout << "Rows: " << r.last->asString() << "\n";
  }
  EXPECT_FALSE(r.last.has_value());
}

// Without a prebuilt map there is nothing to search in: the estimator
// geo-references a map frame anchored where the vehicle starts, and LO must
// start its map at the estimator pose, even if the vehicle has already moved
// by the time the first scan arrives.
TEST_F(InitialGlobalSearch, EmptyMapStartsAtEstimatorPose)
{
  auto sc = hall_scenario();
  sc.mapBase.clear();
  sc.gnssBias = {0, 0};
  sc.gnssSigma = 0.1;
  sc.speed = 0.5;
  sc.lidarStart = 3.0;
  sc.duration = 8.0;
  const auto r = localize(sc);
  ASSERT_TRUE(r.first.has_value());
  ASSERT_TRUE(r.last.has_value());

  // Expected poses, in the map frame anchored at the start pose:
  const auto expectedFirst = mrpt::poses::CPose3D::FromTranslation(sc.speed * sc.lidarStart, 0, 0);
  const auto expectedLast =
    mrpt::poses::CPose3D::FromTranslation(sc.speed * (sc.duration - 0.1), 0, 0);
  std::cout << "First pose: " << r.first->asString() << " last: " << r.last->asString() << "\n";
  EXPECT_LT(xy_error(*r.first, expectedFirst), 0.3);
  EXPECT_LT(xy_error(*r.last, expectedLast), 0.3);
  EXPECT_LT(yaw_error_deg(*r.last, expectedLast), 2.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  mrpt::random::getRandomGenerator().randomize(42);
  return RUN_ALL_TESTS();
}
