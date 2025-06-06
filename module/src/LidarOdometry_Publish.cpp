// -----------------------------------------------------------------------------
//   A Modular Optimization framework for Localization and mApping  (MOLA)
//
// Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
// Licensed under the GNU GPL v3.
//
// This file is part of MOLA.
// MOLA is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// MOLA. If not, see <https://www.gnu.org/licenses/>.
//
// Closed-source licenses available upon request, for this odometry package
// alone or in combination with the complete SLAM system.
// -----------------------------------------------------------------------------

/**
 * @file   LidarOdometry.cpp
 * @brief  Main C++ class exposing LIDAR odometry
 * @author Jose Luis Blanco Claraco
 * @date   Sep 16, 2023
 */

// This module:
#include <mola_lidar_odometry/LidarOdometry.h>

// MRPT:
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/serialization/CArchive.h>

// SFINAE to detect for mp2p_icp map metadata:
namespace
{
// clang-format off
template <typename T, typename = void> struct has_metadata_field : std::false_type {};
template <typename T> struct has_metadata_field<T, std::void_t<decltype(T::metadata)>> : std::true_type {};

template <typename T, typename = void> struct has_map_metadata_field : std::false_type {};
template <typename T> struct has_map_metadata_field<T, std::void_t<decltype(T::map_metadata)>> : std::true_type {};
// clang-format on
}  // namespace

namespace mola
{

void LidarOdometry::doPublishUpdatedLocalization(const mrpt::Clock::time_point & this_obs_tim)
{
  const ProfilerEntry tle(profiler_, "advertiseUpdatedLocalization");

  LocalizationUpdate lu;
  lu.method = "lidar_odometry";
  lu.reference_frame = params_.publish_reference_frame;
#if MOLA_VERSION_CHECK(1, 6, 0)
  lu.child_frame = params_.publish_vehicle_frame;
#endif
  lu.timestamp = this_obs_tim;
  lu.pose = state_.last_lidar_pose.mean.asTPose();
  lu.cov = state_.last_lidar_pose.cov;
#if MOLA_VERSION_CHECK(1, 4, 0)
  lu.quality = state_.last_icp_quality;
#endif

  advertiseUpdatedLocalization(lu);
}

void LidarOdometry::doPublishUpdatedMap(const mrpt::Clock::time_point & this_obs_tim)
{
  // Publish geo-referenced data for the map, if applicable.
  publishMetricMapGeoreferencingData();

  if (!state_.local_map_needs_publish) {
    return;
  }

  // Don't publish if nobody is listening, OR, if it is still
  // pending to subscribe to us:
  if (!anyUpdateMapSubscriber()) {
    return;
  }

  if (
    state_.localmap_advertise_updates_counter++ <
    params_.local_map_updates.publish_map_updates_every_n) {
    return;
  }

  state_.local_map_needs_publish = false;

  if (!anyUpdateMapSubscriber()) {
    MRPT_LOG_DEBUG("doPublishUpdatedMap: Skipping, since we have no subscriber.");
    return;
  }

  const ProfilerEntry tleCleanup(profiler_, "advertiseMap");
  state_.localmap_advertise_updates_counter = 0;

  MapUpdate mu;
  mu.method = "lidar_odometry";
  mu.reference_frame = params_.publish_reference_frame;
  mu.timestamp = this_obs_tim;

  // publish all local map layers:
  // make map *copies* to make this multithread safe.
  // This is costly for large maps (!). That's why we decimate sending
  // map notifications and check for anyUpdateMapSubscriber() above.
  for (const auto & [layerName, layerMap] : state_.local_map->layers) {
    mu.map_name = layerName;

    // Make a copy of the maps:
    if (auto mapPts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(layerMap);
        mapPts) {  // point cloud maps:
      auto mapCopy = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
        mrpt::rtti::classFactory(layerMap->GetRuntimeClass()->className));
      ASSERT_(mapCopy);
      mapCopy->insertAnotherMap(mapPts.get(), mrpt::poses::CPose3D::Identity());

      mu.map = mapCopy;
    } else if (auto * auxPts = layerMap->getAsSimplePointsMap();
               auxPts) {  // classes implementing getAsSimplePointsMap()
      auto mapCopy = mrpt::maps::CSimplePointsMap::Create();
      mapCopy->insertAnotherMap(auxPts, mrpt::poses::CPose3D::Identity());

      mu.map = mapCopy;
    } else {
      // Any other class: make a deep copy.
      mrpt::io::CMemoryStream buf;
      auto ar = mrpt::serialization::archiveFrom(buf);
      ar << *layerMap;
      buf.Seek(0);
      auto out = ar.ReadObject();

      mu.map = std::dynamic_pointer_cast<mrpt::maps::CMetricMap>(out);
      ASSERT_(mu.map);
    }

    // send it out:
    advertiseUpdatedMap(mu);

    MRPT_LOG_DEBUG_STREAM("Published map layer: '" << layerName << "'");
  }

  // And publish the map metadata, if existing:
  // (We need mp2p_icp>=1.7.0 for this field to exist)
  if constexpr (
    has_metadata_field<mp2p_icp::metric_map_t>::value && has_map_metadata_field<MapUpdate>::value) {
    std::stringstream ss;
    state_.local_map->metadata.printAsYAML(
      ss, mrpt::containers::YamlEmitOptions{.emitHeader = false});
    mu.map_name = "metadata";
    mu.map_metadata = ss.str();
    mu.map.reset();  // no map, just metadata
    advertiseUpdatedMap(mu);
  } else {
    MRPT_LOG_DEBUG(
      "Not publishing map metadata, since mp2p_icp::metric_map_t::metadata or "
      "MapUpdate::map_metadata are not available at build time.");
  }
}

void LidarOdometry::publishMetricMapGeoreferencingData()
{
  if (!state_.local_map) {
    return;
  }

  if (!state_.local_map->georeferencing.has_value()) {
    // no geo-ref data
    return;
  }
  const auto & g = state_.local_map->georeferencing.value();

  if (!state_.local_map_georef_needs_publish) {
    return;
  }

  // Don't publish if nobody is listening, OR, if it is still
  // pending to subscribe to us:
  if (!anyUpdateMapSubscriber()) {
    return;
  }

  state_.local_map_georef_needs_publish = false;

#if MOLA_VERSION_CHECK(1, 6, 1)  // we need mola::Georeference struct
  // This will publish geo-ref data via mola_kernel API as mrpt_nav_interfaces::msg::GeoreferencingMetadata

  MRPT_LOG_DEBUG_STREAM(
    "Publishing map georeferencing metadata: T_enu_to_map="
    << g.T_enu_to_map.asString()                           //
    << " geo_coord.lat=" << g.geo_coord.lat.getAsString()  //
    << " geo_coord.lon=" << g.geo_coord.lon.getAsString()  //
    << " geo_coord.height=" << g.geo_coord.height          //
  );

  MapUpdate mu;
  mu.method = "lidar_odometry";
  mu.reference_frame = params_.publish_reference_frame;
  mu.timestamp = mrpt::Clock::now();
  mu.map_name = "georef";

  auto & georef = mu.georeferencing.emplace();
  georef.T_enu_to_map = g.T_enu_to_map;
  georef.geo_coord = g.geo_coord;

  // send it out:
  advertiseUpdatedMap(mu);
#else
  MRPT_LOG_WARN(
    "Not able to publish georeferencing map metadata due to too old mola_kernel version (!)");
#endif
}

void LidarOdometry::onPublishDiagnostics()
{
#if MOLA_VERSION_CHECK(1, 6, 2)
  auto lckState = mrpt::lockHelper(state_mtx_);

  const auto curStamp = state_.last_obs_timestamp ? *state_.last_obs_timestamp : mrpt::Clock::now();

  mrpt::containers::yaml diagValues = mrpt::containers::yaml::Map();

  const double dtAvr = profiler_.getMeanTime("onLidar");

  diagValues["icp_quality"] = state_.last_icp_quality;
  diagValues["average_process_time"] = dtAvr;
  diagValues["dropped_frames_ratio"] = getDropStats();
  diagValues["parameters"] = getModuleParameters();

  DiagnosticsOutput diag;
  diag.timestamp = curStamp;
  diag.label = "status";
  diag.value = diagValues;

  module_publish_diagnostics(diag);
#endif
}

}  // namespace mola
