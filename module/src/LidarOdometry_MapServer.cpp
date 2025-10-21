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
#include <mrpt/system/filesystem.h>

namespace mola
{

MapServer::ReturnStatus LidarOdometry::map_load(const std::string & path)
{
  using namespace std::string_literals;

  auto lckState = mrpt::lockHelper(state_mtx_);

  MapServer::ReturnStatus ret;

  const auto mmFile = path + ".mm"s;
  const auto smFile = path + ".simplemap"s;

  MRPT_LOG_INFO_STREAM("[map_load] Trying to load mm: " << mmFile << " and sm: " << smFile);

  // Clear existing map:
  ASSERT_(state_.local_map);
  state_.local_map->clear();
  state_.reconstructed_simplemap.clear();

  // Update the GUI and publish the map, even if we are not being fed with incoming obs:
  // (and even if we fail to load the maps, so they are shown as empty).
  state_.mark_local_map_as_updated(true);
  state_.mark_local_map_georef_as_updated();

  bool mmLoadOk = false;
  try {
    // do not show the default error msg for this simple error, which is long and may intimidate newest users:
    if (!mrpt::system::fileExists(mmFile)) {
      throw std::runtime_error("");
    }

    mmLoadOk = state_.local_map->load_from_file(mmFile);
  } catch (const std::exception &) {
  }

  bool smLoadOk = false;
  try {
    // do not show the default error msg for this simple error, which is long and may intimidate newest users:
    if (!mrpt::system::fileExists(smFile)) {
      throw std::runtime_error("");
    }

    smLoadOk = state_.reconstructed_simplemap.loadFromFile(smFile);
  } catch (const std::exception &) {
  }

  ret.success = mmLoadOk;  // smLoadOk: not mandatory

  if (!mmLoadOk) {
    ret.error_message = "Error loading metric local map from: "s + mmFile + ". ";
  }
  if (!smLoadOk) {
    ret.error_message = "Warning: no simplemap (keyframes) file found (expected: '"s + smFile +
                        "'). Required for multisession mapping. ";
  }

  if (ret.success) {
    MRPT_LOG_INFO_STREAM("[map_load] Successful.");
  } else {
    MRPT_LOG_ERROR_STREAM("[map_load] Error loading from map prefix: " << path);
  }

  return ret;
}

MapServer::ReturnStatus LidarOdometry::map_save(const std::string & path)
{
  using namespace std::string_literals;

  auto lckState = mrpt::lockHelper(state_mtx_);

  MapServer::ReturnStatus ret;

  const auto mmFile = path + ".mm"s;
  const auto smFile = path + ".simplemap"s;

  MRPT_LOG_INFO_STREAM("[map_save] Trying to save mm: " << mmFile << " and sm: " << smFile);

  bool mmSaveOk = state_.local_map->save_to_file(mmFile);
  bool smSaveOk = state_.reconstructed_simplemap.saveToFile(smFile);

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
