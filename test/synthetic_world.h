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
 * @file   synthetic_world.h
 * @brief  Synthetic worlds and sensor data (3D lidar, GNSS, IMU, odometry)
 *         generated on the fly, shared by the unit tests.
 */

#pragma once

#include <mola_lidar_odometry/LidarOdometry.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/topography/conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

namespace lo_test
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

inline void add_box(World & w, double x, double y, double half)
{
  w.push_back({{x - half, y - half}, {x + half, y - half}});
  w.push_back({{x + half, y - half}, {x + half, y + half}});
  w.push_back({{x + half, y + half}, {x - half, y + half}});
  w.push_back({{x - half, y + half}, {x - half, y - half}});
}

// A hall with inner walls and pillars: no symmetry.
inline World unique_hall()
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
inline World endless_rows(double pitch)
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

inline mrpt::obs::CObservationPointCloud::Ptr scan(
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
inline mola::Georeferencing make_georef()
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
inline mrpt::obs::CObservationGPS::Ptr gnss(
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
inline mrpt::obs::CObservationIMU::Ptr imu(const mrpt::poses::CPose3D & vehicle, double t)
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

inline mrpt::obs::CObservationOdometry::Ptr odometry(const mrpt::poses::CPose2D & p, double t)
{
  auto obs = mrpt::obs::CObservationOdometry::Create();
  obs->sensorLabel = "odom";
  obs->timestamp = mrpt::Clock::fromDouble(t);
  obs->odometry = p;
  obs->hasVelocities = true;
  return obs;
}

inline void wait_idle(const mola::LidarOdometry & lo)
{
  while (lo.isBusy()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

}  // namespace lo_test
