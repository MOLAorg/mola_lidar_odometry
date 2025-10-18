/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

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

// MOLA:
#include <mola_kernel/version.h>

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
  lu.child_frame = params_.publish_vehicle_frame;
  lu.timestamp = this_obs_tim;
  lu.pose = state_.last_lidar_pose.mean.asTPose();
  lu.cov = state_.last_lidar_pose.cov;
  lu.quality = state_.last_icp_quality;

  advertiseUpdatedLocalization(lu);
}

void LidarOdometry::doPublishUpdatedLocalMap(const mrpt::Clock::time_point & this_obs_tim)
{
  // Publish geo-referenced data for the map, if applicable.
  publishMetricMapGeoreferencingData();

  if (!state_.local_map_needs_publish) {
    return;
  }

  if (
    state_.localmap_advertise_updates_counter++ <
    params_.local_map_updates.publish_map_updates_every_n) {
    return;
  }

  // Don't publish if nobody is listening, OR, if it is still
  // pending to subscribe to us:
  if (!anyUpdateMapSubscriber()) {
    MRPT_LOG_DEBUG("doPublishUpdatedLocalMap: Skipping, since we have no subscriber.");
    return;
  }

  state_.local_map_needs_publish = false;

  const ProfilerEntry tleCleanup(profiler_, "doPublishUpdatedLocalMap");
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
    }
    // classes implementing getAsSimplePointsMap()
    else if (auto * auxPts = layerMap->getAsSimplePointsMap(); auxPts) {
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

  // And publish the map metadata (added in mp2p_icp>=1.7.0)
  {
    std::stringstream ss;
    state_.local_map->metadata.printAsYAML(
      ss, mrpt::containers::YamlEmitOptions{.emitHeader = false});
    mu.map_name = "metadata";
    mu.map_metadata = ss.str();
    mu.map.reset();  // no map, just metadata
    advertiseUpdatedMap(mu);
  }
}

void LidarOdometry::doPublishDeskewedScan(const mrpt::Clock::time_point & this_obs_tim)
{
  if (!state_.last_deskewed_scan_for_publishing) {
    return;
  }

  const auto deskewedScan = state_.last_deskewed_scan_for_publishing;
  state_.last_deskewed_scan_for_publishing.reset();

  // Don't publish if nobody is listening, OR, if it is still
  // pending to subscribe to us:
  if (!anyUpdateMapSubscriber()) {
    MRPT_LOG_DEBUG("doPublishDeskewedScan: Skipping, since we have no subscriber.");
    return;
  }

  const ProfilerEntry tleCleanup(profiler_, "doPublishDeskewedScan");

  MapUpdate mu;
  mu.method = "lidar_odometry";
  mu.reference_frame = params_.publish_reference_frame;
  mu.timestamp = this_obs_tim;
  mu.map_name = "deskewed_scan";
  mu.map = deskewedScan;
#if MOLA_VERSION_CHECK(2, 1, 0)
  mu.keep_last_one_only = false;  // Aggregate all scans
#endif

  // send it out:
  advertiseUpdatedMap(mu);

  MRPT_LOG_DEBUG("Published deskewed map");
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
}

void LidarOdometry::onPublishDiagnostics()
{
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
}

}  // namespace mola
