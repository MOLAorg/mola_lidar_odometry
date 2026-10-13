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

// The global search and these scenarios need recent enough dependencies:
#if __has_include(<mola_relocalization/relocalization.h>)
#include <mola_relocalization/relocalization.h>
#endif
#if __has_include(<mola_state_estimation_smoother/Parameters.h>)
#include <mola_state_estimation_smoother/Parameters.h>
#endif

namespace
{
// ---------------------------------------------------------------------------
// Synthetic world: vertical walls on a flat floor
// ---------------------------------------------------------------------------
struct Wall
{
  mrpt::math::TPoint2D a, b;
  double height = 3.0;
};
using World = std::vector<Wall>;

void add_box(World & w, double x, double y, double half)
{
  w.push_back({{x - half, y - half}, {x + half, y - half}});
  w.push_back({{x + half, y - half}, {x + half, y + half}});
  w.push_back({{x + half, y + half}, {x - half, y + half}});
  w.push_back({{x - half, y + half}, {x - half, y - half}});
}

// A hall with inner walls and pillars: no symmetry.
World unique_hall()
{
  World w = {{{0, 0}, {40, 0}},  {{40, 0}, {40, 25}}, {{40, 25}, {0, 25}},  {{0, 25}, {0, 0}},
             {{10, 0}, {10, 7}}, {{10, 7}, {17, 7}},  {{25, 18}, {33, 18}}, {{30, 0}, {30, 5}}};
  for (const auto & [x, y] :
       std::vector<mrpt::math::TPoint2D>{{6, 17}, {20, 12}, {28, 9}, {35, 21}, {15, 20}}) {
    add_box(w, x, y, 0.3);
  }
  return w;
}

// Identical long rows every `pitch` meters, much longer than the sensor range,
// with structural posts on a regular grid (as in a greenhouse):
World endless_rows(double pitch)
{
  World w;
  for (int i = -5; i <= 5; i++) {
    w.push_back({{-200, i * pitch}, {200, i * pitch}, 1.5});
    for (int k = -20; k <= 20; k++) {
      const auto first = w.size();
      add_box(w, 3.0 * k, i * pitch + 0.3, 0.05);
      for (auto j = first; j < w.size(); j++) {
        w[j].height = 2.5;
      }
    }
  }
  return w;
}

// A 3D lidar on the vehicle, raycasting the world (walls and floor at z=0).
constexpr double kSensorHeight = 1.2;  // [m]
constexpr double kMaxRange = 30.0;     // [m]

mrpt::obs::CObservationPointCloud::Ptr scan(
  const World & world, const mrpt::poses::CPose3D & vehicle, double t)
{
  auto & rng = mrpt::random::getRandomGenerator();
  auto pts = mrpt::maps::CSimplePointsMap::Create();
  const auto sensorOnVehicle =
    mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, kSensorHeight, 0, 0, 0);
  const auto sensor = vehicle + sensorOnVehicle;

  // Only walls within range:
  World near;
  for (const auto & wall : world) {
    const auto o = mrpt::math::TPoint2D(sensor.x(), sensor.y());
    const auto e = wall.b - wall.a;
    const double t =
      std::clamp(((o - wall.a).x * e.x + (o - wall.a).y * e.y) / e.sqrNorm(), 0.0, 1.0);
    if ((wall.a + e * t - o).norm() < kMaxRange) {
      near.push_back(wall);
    }
  }

  for (int iEl = 0; iEl < 16; iEl++) {
    const double el = mrpt::DEG2RAD(-15.0 + 2.0 * iEl);
    for (int iAz = 0; iAz < 360; iAz++) {
      const double az = mrpt::DEG2RAD(iAz);
      // Ray in the sensor frame, and in the world:
      const mrpt::math::TPoint3D dirL(
        std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
      const auto dirW = sensor.rotateVector(dirL);
      const auto o = sensor.translation();

      double best = kMaxRange;
      if (dirW.z < -1e-6) {
        best = std::min(best, -o.z / dirW.z);  // floor
      }
      const double dxy = mrpt::math::TPoint2D(dirW.x, dirW.y).norm();
      if (dxy > 1e-6) {
        for (const auto & wall : near) {
          // Intersect the ray's XY projection with the wall segment:
          const auto e = wall.b - wall.a;
          const double den = dirW.x * e.y - dirW.y * e.x;
          if (std::abs(den) < 1e-9) {
            continue;
          }
          const double ax = wall.a.x - o.x;
          const double ay = wall.a.y - o.y;
          const double r = (ax * e.y - ay * e.x) / den;        // along the 3D ray
          const double s = (ax * dirW.y - ay * dirW.x) / den;  // along the wall
          if (r <= 0 || s < 0 || s > 1 || r >= best) {
            continue;
          }
          const double z = o.z + r * dirW.z;
          if (z < 0 || z > wall.height) {
            continue;
          }
          best = r;
        }
      }
      if (best >= kMaxRange) {
        continue;
      }
      const double range = best + rng.drawGaussian1D(0, 0.01);
      pts->insertPoint(
        static_cast<float>(range * dirL.x), static_cast<float>(range * dirL.y),
        static_cast<float>(range * dirL.z));
    }
  }

  auto obs = mrpt::obs::CObservationPointCloud::Create();
  obs->sensorLabel = "lidar";
  obs->timestamp = mrpt::Clock::fromDouble(t);
  obs->sensorPose = sensorOnVehicle;
  obs->pointcloud = pts;
  return obs;
}

// ---------------------------------------------------------------------------
// GNSS and IMU
// ---------------------------------------------------------------------------
mola::Georeferencing make_georef()
{
  mola::Georeferencing g;
  g.geo_coord.lat = 36.84;
  g.geo_coord.lon = -2.40;
  g.geo_coord.height = 100.0;
  g.T_enu_to_map.mean = mrpt::poses::CPose3D::Identity();
  g.T_enu_to_map.cov.setZero();
  return g;
}

// A fix at the vehicle position (map = ENU here) plus `bias` and noise:
mrpt::obs::CObservationGPS::Ptr gnss(
  const mola::Georeferencing & georef, const mrpt::poses::CPose3D & vehicle,
  const mrpt::math::TPoint2D & bias, double sigma, double t)
{
  auto & rng = mrpt::random::getRandomGenerator();
  const mrpt::math::TPoint3D enu(
    vehicle.x() + bias.x + rng.drawGaussian1D(0, sigma),
    vehicle.y() + bias.y + rng.drawGaussian1D(0, sigma), vehicle.z());

  mrpt::math::TPoint3D geocentric;
  mrpt::topography::ENUToGeocentric(
    enu, georef.geo_coord, geocentric, mrpt::topography::TEllipsoid::Ellipsoid_WGS84());
  mrpt::topography::TGeodeticCoords geo;
  mrpt::topography::geocentricToGeodetic(geocentric, geo);

  mrpt::obs::gnss::Message_NMEA_GGA gga;
  gga.fields.latitude_degrees = geo.lat.decimal_value;
  gga.fields.longitude_degrees = geo.lon.decimal_value;
  gga.fields.altitude_meters = geo.height;
  gga.fields.fix_quality = 1;
  gga.fields.satellitesUsed = 9;

  auto obs = mrpt::obs::CObservationGPS::Create();
  obs->sensorLabel = "gps";
  obs->timestamp = mrpt::Clock::fromDouble(t);
  obs->setMsg(gga);
  mrpt::math::CMatrixDouble33 cov;
  cov.setDiagonal(std::vector<double>({sigma * sigma, sigma * sigma, 4 * sigma * sigma}));
  obs->covariance_enu = cov;
  return obs;
}

// Absolute attitude, as given in ENU by a simulator:
mrpt::obs::CObservationIMU::Ptr imu(const mrpt::poses::CPose3D & vehicle, double t)
{
  auto obs = mrpt::obs::CObservationIMU::Create();
  obs->sensorLabel = "imu";
  obs->timestamp = mrpt::Clock::fromDouble(t);
  mrpt::math::CQuaternionDouble q;
  vehicle.getAsQuaternion(q);
  obs->set(mrpt::obs::IMU_ORI_QUAT_W, q.r());
  obs->set(mrpt::obs::IMU_ORI_QUAT_X, q.x());
  obs->set(mrpt::obs::IMU_ORI_QUAT_Y, q.y());
  obs->set(mrpt::obs::IMU_ORI_QUAT_Z, q.z());
  return obs;
}

mrpt::obs::CObservationOdometry::Ptr odometry(const mrpt::poses::CPose2D & p, double t)
{
  auto obs = mrpt::obs::CObservationOdometry::Create();
  obs->sensorLabel = "odom";
  obs->timestamp = mrpt::Clock::fromDouble(t);
  obs->odometry = p;
  obs->hasVelocities = true;
  return obs;
}

void wait_idle(const mola::LidarOdometry & lo)
{
  while (lo.isBusy()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

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
  std::string mapBase;
  mrpt::poses::CPose3D truth;  //!< static vehicle
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
  set_env("MOLA_MAPPING_ENABLED", "false");
  set_env("MOLA_LOAD_MM", sc.mapBase + ".mm");
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
  smoother->set_geo_reference(make_georef());

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
    const double t = 1000.0 + 0.1 * i;
    smoother->fuse_odometry(*odometry(mrpt::poses::CPose2D(), t), "odom_wheels");
    if (sc.withImu) {
      smoother->fuse_imu(*imu(sc.truth, t));
    }
    if (i % 10 == 0) {
      smoother->fuse_gnss(*gnss(georef, sc.truth, sc.gnssBias, sc.gnssSigma, t));
    }
    smoother->spinOnce();
    liodom->onNewObservation(scan(sc.world, sc.truth, t));
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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  mrpt::random::getRandomGenerator().randomize(42);
  return RUN_ALL_TESTS();
}
