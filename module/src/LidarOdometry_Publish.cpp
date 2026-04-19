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
#include <mrpt/system/datetime.h>

#include <iomanip>
#include <limits>
#include <sstream>

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

  // === This was ===
  // Don't publish if nobody is listening, OR, if it is still
  // pending to subscribe to us:
  //if (!anyUpdateMapSubscriber()) { return; }
  // ================
  // However, DON'T do this, since if publishing to ROS, latched messages
  // from a once-at-start-up publication would be expected.

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
  mu.keep_last_one_only = false;  // Aggregate all scans

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

  // === This was ===
  // Don't publish if nobody is listening, OR, if it is still
  // pending to subscribe to us:
  //if (!anyUpdateMapSubscriber()) { return; }
  // ================
  // However, DON'T do this, since if publishing to ROS, latched messages
  // from a once-at-start-up publication would be expected.

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

#if MOLA_HAS_DIAGNOSTICS_PROVIDER
namespace
{
mola::DiagnosticKeyValue kv_d(const std::string & key, double value, int precision = 3)
{
  std::ostringstream ss;
  ss.precision(precision);
  ss << std::fixed << value;
  return {key, ss.str()};
}
mola::DiagnosticKeyValue kv_u(const std::string & key, std::size_t value)
{
  return {key, std::to_string(value)};
}
mola::DiagnosticLevel worst(mola::DiagnosticLevel a, mola::DiagnosticLevel b)
{
  using L = mola::DiagnosticLevel;
  auto rank = [](L x) {
    switch (x) {
      case L::OK:
        return 0;
      case L::WARN:
        return 1;
      case L::STALE:
        return 2;
      case L::ERROR:
        return 3;
    }
    return 0;
  };
  return rank(a) >= rank(b) ? a : b;
}
}  // namespace

void LidarOdometry::getDiagnostics(std::vector<mola::DiagnosticStatusMsg> & status)
{
  using L = mola::DiagnosticLevel;

  auto lckState = mrpt::lockHelper(state_mtx_);

  const auto & th = params_.diagnostics;
  const auto now = mrpt::Clock::now();
  // Use wall-clock reception time to avoid false positives when sensor
  // hardware clocks are not synchronized to system time.
  const double lastObsAgeSec = state_.last_obs_reception_time
                                 ? mrpt::system::timeDifference(*state_.last_obs_reception_time, now)
                                 : std::numeric_limits<double>::infinity();

  const double icpQ = state_.last_icp_quality;
  const double dtAvr = profiler_.getMeanTime("onLidar");
  const double droppedRatio = getDropStats();
  const std::size_t startIndex = status.size();
  const double sensorPeriod = state_.last_observed_scan_period_sec;

  // 1) Input Data
  {
    mola::DiagnosticStatusMsg s;
    s.name = "LidarOdometry: Input Data";
    if (!state_.last_obs_reception_time) {
      s.level = L::STALE;
      s.message = "No observations received yet";
    } else if (lastObsAgeSec > th.input_error_sec) {
      s.level = L::ERROR;
      s.message = "No observations for >" + std::to_string(th.input_error_sec) + "s";
    } else if (lastObsAgeSec > th.input_stale_sec) {
      s.level = L::STALE;
      s.message = "Input appears stale";
    } else if (droppedRatio > th.dropped_ratio_error) {
      s.level = L::ERROR;
      s.message = "Very high frame drop rate";
    } else if (droppedRatio > th.dropped_ratio_warn) {
      s.level = L::WARN;
      s.message = "High frame drop rate";
    } else {
      s.level = L::OK;
      s.message = "Nominal";
    }
    s.values.push_back(kv_d("last_obs_age_sec", lastObsAgeSec));
    s.values.push_back(kv_d("dropped_ratio", droppedRatio));
    status.push_back(std::move(s));
  }

  // 2) ICP Quality
  {
    mola::DiagnosticStatusMsg s;
    s.name = "LidarOdometry: ICP Quality";
    if (!state_.last_icp_timestamp) {
      s.level = L::STALE;
      s.message = "No ICP result yet";
    } else if (icpQ < th.icp_quality_error) {
      s.level = L::ERROR;
      s.message = "ICP quality critically low";
    } else if (icpQ < th.icp_quality_warn) {
      s.level = L::WARN;
      s.message = "ICP quality degraded";
    } else {
      s.level = L::OK;
      s.message = "Nominal";
    }
    s.values.push_back(kv_d("quality", icpQ));
    s.values.push_back(kv_u("last_icp_iterations", state_.last_icp_iterations));
    s.values.push_back({"last_icp_was_good", state_.last_icp_was_good ? "true" : "false"});
    status.push_back(std::move(s));
  }

  // 3) Timing
  {
    mola::DiagnosticStatusMsg s;
    s.name = "LidarOdometry: Timing";
    const double util = (sensorPeriod > 0) ? (dtAvr / sensorPeriod) : 0.0;
    if (sensorPeriod > 0 && util > th.timing_utilization_warn) {
      s.level = L::WARN;
      s.message = "Process time close to sensor period";
    } else {
      s.level = L::OK;
      s.message = "Nominal";
    }
    s.values.push_back(kv_d("avg_ms", dtAvr * 1000.0));
    s.values.push_back(kv_d("sensor_period_ms", sensorPeriod * 1000.0));
    s.values.push_back(kv_d("utilization_pct", util * 100.0));
    status.push_back(std::move(s));
  }

  // 4) Local Map (informational)
  {
    mola::DiagnosticStatusMsg s;
    s.name = "LidarOdometry: Local Map";
    s.level = L::OK;
    s.message = "Informational";
    std::size_t numLayers = 0;
    std::size_t totalPts = 0;
    if (state_.local_map) {
      numLayers = state_.local_map->layers.size();
      for (const auto & [layerName, layerMap] : state_.local_map->layers) {
        if (!layerMap) continue;
        if (auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(layerMap); pts)
          totalPts += pts->size();
      }
    }
    s.values.push_back(kv_u("num_layers", numLayers));
    s.values.push_back(kv_u("total_points", totalPts));
    status.push_back(std::move(s));
  }

  // 5) Overall status: worst-of the entries added by this provider
  {
    mola::DiagnosticStatusMsg s;
    s.name = "LidarOdometry: Overall Status";
    s.level = L::OK;
    for (std::size_t i = startIndex; i < status.size(); ++i) {
      s.level = worst(s.level, status[i].level);
    }
    switch (s.level) {
      case L::OK:
        s.message = "Nominal";
        break;
      case L::WARN:
        s.message = "Degraded";
        break;
      case L::STALE:
        s.message = "Stale input";
        break;
      case L::ERROR:
        s.message = "Non-functional";
        break;
    }
    s.values.push_back(kv_d("icp_quality", icpQ));
    s.values.push_back(kv_d("process_time_avg_ms", dtAvr * 1000.0));
    s.values.push_back(kv_d("dropped_frames_ratio", droppedRatio));
    status.push_back(std::move(s));
  }
}
#endif  // MOLA_HAS_DIAGNOSTICS_PROVIDER

}  // namespace mola
