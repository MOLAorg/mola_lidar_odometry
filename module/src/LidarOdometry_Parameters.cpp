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

// MOLA:
#include <mola_yaml/yaml_helpers.h>

// MRPT:
#include <mrpt/gui/CDisplayWindowGUI.h>  // for nanogui controls

namespace mola
{

void LidarOdometry::Parameters::AdaptiveThreshold::initialize(const Yaml & cfg)
{
  YAML_LOAD_REQ(enabled, bool);
  YAML_LOAD_REQ(initial_sigma, double);
  YAML_LOAD_REQ(min_motion, double);
  YAML_LOAD_REQ(kp, double);
  YAML_LOAD_REQ(alpha, double);
  YAML_LOAD_OPT(maximum_sigma, double);
}

void LidarOdometry::Parameters::Visualization::initialize(const Yaml & cfg)
{
  YAML_LOAD_OPT(map_update_decimation, int);
  YAML_LOAD_OPT(show_trajectory, bool);
  YAML_LOAD_OPT(show_current_observation, bool);
  YAML_LOAD_OPT(show_ground_grid, bool);
  YAML_LOAD_OPT(ground_grid_spacing, float);
  YAML_LOAD_OPT(show_console_messages, bool);
  YAML_LOAD_OPT(current_pose_corner_size, double);
  YAML_LOAD_OPT(local_map_point_size, float);
  YAML_LOAD_OPT(local_map_render_voxelmap_free_space, bool);

  if (cfg.has("model")) {
    ASSERT_(cfg["model"].isSequence());
    const auto models = cfg["model"].asSequenceRange();
    for (const auto & e : models) {
      ASSERT_(e.isMap());
      auto c = e.asMap();
      auto & m = model.emplace_back();
      ASSERT_(c.count("file") != 0);
      m.file = c["file"].as<std::string>();

      if (m.file.empty()) {
        model.erase(--model.end());
        continue;
      }

      if (c.count("tf.x")) m.tf.x = c["tf.x"].as<float>();
      if (c.count("tf.y")) m.tf.y = c["tf.y"].as<float>();
      if (c.count("tf.z")) m.tf.z = c["tf.z"].as<float>();

      if (c.count("tf.yaw")) m.tf.yaw = mrpt::DEG2RAD(c["tf.yaw"].as<float>());

      if (c.count("tf.pitch")) m.tf.pitch = mrpt::DEG2RAD(c["tf.pitch"].as<float>());

      if (c.count("tf.roll")) m.tf.roll = mrpt::DEG2RAD(c["tf.roll"].as<float>());

      if (c.count("scale")) m.scale = c["scale"].as<float>();
    }
  }

  YAML_LOAD_OPT(gui_subwindow_starts_hidden, bool);
  YAML_LOAD_OPT(camera_follows_vehicle, bool);
  YAML_LOAD_OPT(camera_rotates_with_vehicle, bool);
  YAML_LOAD_OPT(camera_orthographic, bool);
}

void LidarOdometry::Parameters::SimpleMapOptions::initialize(const Yaml & cfg, Parameters & parent)
{
  YAML_LOAD_OPT(generate, bool);
  DECLARE_PARAMETER_IN_OPT(cfg, min_translation_between_keyframes, parent);
  DECLARE_PARAMETER_IN_OPT(cfg, min_rotation_between_keyframes, parent);
  YAML_LOAD_OPT(save_final_map_to_file, std::string);
  YAML_LOAD_OPT(add_non_keyframes_too, bool);
  YAML_LOAD_OPT(measure_from_last_kf_only, bool);
  YAML_LOAD_OPT(generate_lazy_load_scan_files, bool);
  YAML_LOAD_OPT(save_gnss_max_age, double);
}

void LidarOdometry::Parameters::MultipleLidarOptions::initialize(
  const Yaml & cfg, Parameters & parent)
{
  DECLARE_PARAMETER_IN_REQ(cfg, max_time_offset, parent);
  YAML_LOAD_REQ(lidar_count, uint32_t);
}

void LidarOdometry::Parameters::MapUpdateOptions::initialize(const Yaml & cfg, Parameters & parent)
{
  YAML_LOAD_OPT(enabled, bool);
  DECLARE_PARAMETER_IN_REQ(cfg, min_translation_between_keyframes, parent);
  DECLARE_PARAMETER_IN_REQ(cfg, min_rotation_between_keyframes, parent);
  DECLARE_PARAMETER_IN_OPT(cfg, max_distance_to_keep_keyframes, parent);
  DECLARE_PARAMETER_IN_OPT(cfg, check_for_removal_every_n, parent);
  DECLARE_PARAMETER_IN_OPT(cfg, publish_map_updates_every_n, parent);
  YAML_LOAD_OPT(measure_from_last_kf_only, bool);
  YAML_LOAD_OPT(load_existing_local_map, std::string);
}

void LidarOdometry::Parameters::TrajectoryOutputOptions::initialize(const Yaml & cfg)
{
  YAML_LOAD_OPT(save_to_file, bool);
  YAML_LOAD_OPT(output_file, std::string);
}

void LidarOdometry::Parameters::TraceOutputOptions::initialize(const Yaml & cfg)
{
  YAML_LOAD_OPT(save_to_file, bool);
  YAML_LOAD_OPT(output_file, std::string);
}

void LidarOdometry::Parameters::InitialLocalizationOptions::initialize(const Yaml & cfg)
{
  MCP_LOAD_OPT(cfg, method);

  YAML_LOAD_OPT(additional_uncertainty_after_reloc_how_many_timesteps, uint32_t);
  YAML_LOAD_OPT(pitch_and_roll_from_imu_sample_count, uint32_t);
  YAML_LOAD_OPT(pitch_and_roll_from_imu_max_age, double);

  if (cfg.has("fixed_initial_pose")) {
    ASSERT_(
      cfg["fixed_initial_pose"].isSequence() && cfg["fixed_initial_pose"].asSequence().size() == 6);

    auto & p = fixed_initial_pose;
    const auto seq = cfg["fixed_initial_pose"].asSequenceRange();
    for (size_t i = 0; i < 6; i++) p[i] = seq.at(i).as<double>();
  }
}

void LidarOdometry::Parameters::ObservationValidityChecks::initialize(const Yaml & cfg)
{
  YAML_LOAD_OPT(enabled, bool);
  YAML_LOAD_OPT(check_layer_name, std::string);
  YAML_LOAD_OPT(minimum_point_count, uint32_t);
}

#if MOLA_VERSION_CHECK(1, 4, 0)
void LidarOdometry::onParameterUpdate(const mrpt::containers::yaml & names_values)
{
  if (names_values.isNullNode() || names_values.empty()) return;

  ASSERT_(names_values.isMap());

  auto lckState = mrpt::lockHelper(state_mtx_);

  // Load parameters:
  setActive(names_values.getOrDefault("active", isActive()));

  params_.local_map_updates.enabled =
    names_values.getOrDefault("mapping_enabled", params_.local_map_updates.enabled);
  params_.simplemap.generate =
    names_values.getOrDefault("generate_simplemap", params_.simplemap.generate);

  // Special triggering reset "variabe":
  if (names_values.getOrDefault("reset_state", false)) {
    this->enqueue_request([this]() {
      MRPT_LOG_INFO("Received a reset() command via parameters update.");
      reset();
    });
  }

  // and reflect changes in the GUI, if used.
  this->enqueue_request([this]() {
    auto lckGuiMtx = mrpt::lockHelper(state_gui_mtx_);
    if (gui_.cbActive) {
      gui_.cbActive->setChecked(isActive());
      gui_.cbMapping->setChecked(params_.local_map_updates.enabled);
      gui_.cbSaveSimplemap->setChecked(params_.simplemap.generate);
    }
  });
}
#endif

void LidarOdometry::onExposeParameters()
{
#if MOLA_VERSION_CHECK(1, 4, 0)
  mrpt::containers::yaml nv = mrpt::containers::yaml::Map();
  nv["active"] = isActive();
  nv["mapping_enabled"] = params_.local_map_updates.enabled;
  nv["generate_simplemap"] = params_.simplemap.generate;
  nv["reset_state"] = false;

  this->exposeParameters(nv);
#endif
}

}  // namespace mola
