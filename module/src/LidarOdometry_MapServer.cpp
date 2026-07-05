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
#include <mrpt/system/filesystem.h>

// STD:
#include <future>
#include <memory>

namespace mola
{
namespace
{
// Helper: enqueue a request that computes a value, and block the caller on
// its future. The caller holds no internal mutex while waiting, so it cannot
// contribute to deadlock with the lidar worker thread.
template <typename Fn>
auto runOnWorkerAndWait(LidarOdometry & self, Fn && fn) -> decltype(fn())
{
  using R = decltype(fn());
  auto promise = std::make_shared<std::promise<R>>();
  auto fut = promise->get_future();

  self.enqueue_request([promise, fn = std::forward<Fn>(fn)]() mutable {
    try {
      promise->set_value(fn());
    } catch (...) {
      try {
        promise->set_exception(std::current_exception());
      } catch (...) {
      }
    }
  });

  return fut.get();
}
}  // namespace

// Public entry points: non-blocking dispatch + wait-on-future.

MapServer::ReturnStatus LidarOdometry::map_load(const std::string & path)
{
  return runOnWorkerAndWait(*this, [this, path]() { return this->map_load_impl(path); });
}

MapServer::ReturnStatus LidarOdometry::map_save(const std::string & path)
{
  return runOnWorkerAndWait(*this, [this, path]() { return this->map_save_impl(path); });
}

// Synchronous bodies: run on the lidar worker thread (or spin thread) via
// processPendingUserRequests().

MapServer::ReturnStatus LidarOdometry::map_load_impl(const std::string & path)
{
  using namespace std::string_literals;

  const auto mmFile = path + ".mm"s;
  const auto smFile = path + ".simplemap"s;

  MRPT_LOG_INFO_STREAM("[map_load] Trying to load mm: " << mmFile << " and sm: " << smFile);

  MapServer::ReturnStatus ret;

  // Load the local metric map under state_mtx_ (it is shared with the
  // main lidar processing loop).
  bool mmLoadOk = false;
  {
    auto lckState = mrpt::lockHelper(state_mtx_);

    ASSERT_(state_.local_map);
    state_.local_map->clear();
    state_.gravity_calib_pitch_roll.reset();  // new map origin: recapture at next first KF

    // Mark for GUI / publishing even if we fail to load (so the map is shown
    // as empty rather than stale):
    state_.mark_local_map_as_updated(true);
    state_.mark_local_map_georef_as_updated();

    try {
      if (!mrpt::system::fileExists(mmFile)) {
        throw std::runtime_error("");
      }
      mmLoadOk = state_.local_map->load_from_file(mmFile);
    } catch (const std::exception &) {
      // It's ok if we cannot load the map.
    }

    state_.map_has_been_loaded = mmLoadOk;
  }

  // The reconstructed simplemap is protected by state_simplemap_mtx_, not
  // state_mtx_ (see reconstructedMap() / saveReconstructedMapToFile()).
  // The previous implementation mutated it under state_mtx_, which was a
  // data race with those readers.
  bool smLoadOk = false;
  {
    auto lckSM = mrpt::lockHelper(state_simplemap_mtx_);

    state_.reconstructed_simplemap.clear();

    try {
      if (!mrpt::system::fileExists(smFile)) {
        throw std::runtime_error("");
      }
      smLoadOk = state_.reconstructed_simplemap.loadFromFile(smFile);
    } catch (const std::exception &) {
      // It's ok if we cannot load the simplemap.
    }
  }

  ret.success = mmLoadOk;  // smLoadOk is not mandatory

  if (!mmLoadOk) {
    ret.error_message += "Error loading metric local map from: "s + mmFile + ". ";
  }
  if (!smLoadOk) {
    ret.error_message += "Warning: no simplemap (keyframes) file found (expected: '"s + smFile +
                         "'). Required for multisession mapping. ";
  }

  if (ret.success) {
    MRPT_LOG_INFO_STREAM("[map_load] Successful.");
  } else {
    MRPT_LOG_ERROR_STREAM("[map_load] Error loading from map prefix: " << path);
  }

  return ret;
}

MapServer::ReturnStatus LidarOdometry::map_save_impl(const std::string & path)
{
  using namespace std::string_literals;

  const auto mmFile = path + ".mm"s;
  const auto smFile = path + ".simplemap"s;

  MRPT_LOG_INFO_STREAM("[map_save] Trying to save mm: " << mmFile << " and sm: " << smFile);

  MapServer::ReturnStatus ret;

  // NOTE: map_save_impl runs on the lidar worker thread via
  // processPendingUserRequests() (see map_save() dispatch). The lidar worker
  // is the only writer of state_.local_map and state_.reconstructed_simplemap,
  // and it is currently executing this function rather than a scan, so no
  // concurrent writer can mutate those objects here. Other threads only ever
  // read them after taking a deep copy under the corresponding mutex
  // (see doPublishUpdatedLocalMap(), reconstructedMap(), ...), so it is safe
  // and necessary to release state_mtx_ / state_simplemap_mtx_ during the
  // actual serialization.
  const bool mmSaveOk = state_.local_map->save_to_file(mmFile);
  const bool smSaveOk = state_.reconstructed_simplemap.saveToFile(smFile);

  ret.success = mmSaveOk && smSaveOk;

  if (!mmSaveOk) {
    ret.error_message = "Error saving metric local map from: "s + mmFile + ". ";
  }
  if (!smSaveOk) {
    ret.error_message = "Error saving simplemap (keyframes) from: "s + smFile + ". ";
  }

  if (ret.success) {
    MRPT_LOG_INFO_STREAM("[map_save] Successful.");
  } else {
    MRPT_LOG_ERROR_STREAM("[map_save] Error saving map to map prefix: " << path);
  }

  return ret;
}

}  // namespace mola
