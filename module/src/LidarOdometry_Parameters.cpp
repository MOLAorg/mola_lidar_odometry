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

// MOLA:
#include <mola_yaml/yaml_helpers.h>

// MRPT:
#include <mrpt/gui/CDisplayWindowGUI.h>  // for nanogui controls

namespace mola
{

void LidarOdometry::Parameters::Diagnostics::initialize(const Yaml & cfg)
{
  YAML_LOAD_OPT(icp_quality_warn, double);
  YAML_LOAD_OPT(icp_quality_error, double);
  YAML_LOAD_OPT(input_stale_sec, double);
  YAML_LOAD_OPT(input_error_sec, double);
  YAML_LOAD_OPT(dropped_ratio_warn, double);
  YAML_LOAD_OPT(dropped_ratio_error, double);
  YAML_LOAD_OPT(timing_utilization_warn, double);

  // Validate thresholds so misconfiguration cannot silently invert severities.
  ASSERTMSG_(
    icp_quality_warn >= 0 && icp_quality_error >= 0,
    mrpt::format(
      "diagnostics: icp_quality_warn (%.3f) and icp_quality_error (%.3f) must be >= 0",
      icp_quality_warn, icp_quality_error));
  ASSERTMSG_(
    icp_quality_error < icp_quality_warn,
    mrpt::format(
      "diagnostics: icp_quality_error (%.3f) must be < icp_quality_warn (%.3f)", icp_quality_error,
      icp_quality_warn));

  ASSERTMSG_(
    input_stale_sec >= 0 && input_error_sec >= 0,
    mrpt::format(
      "diagnostics: input_stale_sec (%.3f) and input_error_sec (%.3f) must be >= 0",
      input_stale_sec, input_error_sec));
  ASSERTMSG_(
    input_stale_sec < input_error_sec,
    mrpt::format(
      "diagnostics: input_stale_sec (%.3f) must be < input_error_sec (%.3f)", input_stale_sec,
      input_error_sec));

  ASSERTMSG_(
    dropped_ratio_warn >= 0 && dropped_ratio_warn <= 1 && dropped_ratio_error >= 0 &&
      dropped_ratio_error <= 1,
    mrpt::format(
      "diagnostics: dropped_ratio_warn (%.3f) and dropped_ratio_error (%.3f) must be in [0,1]",
      dropped_ratio_warn, dropped_ratio_error));
  ASSERTMSG_(
    dropped_ratio_warn < dropped_ratio_error,
    mrpt::format(
      "diagnostics: dropped_ratio_warn (%.3f) must be < dropped_ratio_error (%.3f)",
      dropped_ratio_warn, dropped_ratio_error));

  ASSERTMSG_(
    timing_utilization_warn >= 0 && timing_utilization_warn <= 1,
    mrpt::format(
      "diagnostics: timing_utilization_warn (%.3f) must be in [0,1]", timing_utilization_warn));
}

void LidarOdometry::Parameters::AdaptiveThreshold::initialize(const Yaml & cfg)
{
  YAML_LOAD_REQ(enabled, bool);
  YAML_LOAD_REQ(initial_sigma, double);
  YAML_LOAD_REQ(min_motion, double);
  YAML_LOAD_REQ(kp, double);
  YAML_LOAD_REQ(alpha, double);
  YAML_LOAD_OPT(maximum_sigma, double);
  YAML_LOAD_OPT(icp_quality_controller_setpoint, double);
}

void LidarOdometry::Parameters::Visualization::initialize(const Yaml & cfg)
{
  YAML_LOAD_OPT(map_update_decimation, int);
  YAML_LOAD_OPT(background_color_gray_level, float);
  YAML_LOAD_OPT(show_trajectory, bool);

  if (cfg.has("trajectory_rgba")) {
    ASSERT_(cfg["trajectory_rgba"].isSequence() && cfg["trajectory_rgba"].asSequence().size() == 4);
    trajectory_rgba = cfg["trajectory_rgba"].toStdVector<float>();
  }

  YAML_LOAD_OPT(show_current_observation, bool);
  YAML_LOAD_OPT(show_last_deskewed_observations_decay, bool);
  YAML_LOAD_OPT(show_localmap, bool);
  YAML_LOAD_OPT(observations_decay_seconds, double);
  YAML_LOAD_OPT(observations_initial_alpha, float);
  YAML_LOAD_OPT(current_observation_alpha, float);
  YAML_LOAD_OPT(show_ground_grid, bool);
  YAML_LOAD_OPT(ground_grid_spacing, float);
  YAML_LOAD_OPT(show_console_messages, bool);
  YAML_LOAD_OPT(current_pose_corner_size, float);
  YAML_LOAD_OPT(local_map_point_size, float);
  YAML_LOAD_OPT(current_observation_point_size, float);
  YAML_LOAD_OPT(last_deskewed_observations_point_size, float);
  YAML_LOAD_OPT(local_map_render_voxelmap_free_space, bool);

  MCP_LOAD_OPT(cfg, current_observation_colormap);
  MCP_LOAD_OPT(cfg, current_observation_color_by_field);

  MCP_LOAD_OPT(cfg, last_deskewed_observations_colormap);
  MCP_LOAD_OPT(cfg, last_deskewed_observations_color_by_field);

  MCP_LOAD_OPT(cfg, local_map_colormap);
  MCP_LOAD_OPT(cfg, local_map_colormap_color_by_field);

  YAML_LOAD_OPT(gui_subwindow_starts_hidden, bool);
  YAML_LOAD_OPT(camera_follows_vehicle, bool);
  YAML_LOAD_OPT(camera_rotates_with_vehicle, bool);
  YAML_LOAD_OPT(camera_orthographic, bool);
  YAML_LOAD_OPT(show_gravity_align_vector, bool);

  initializeModelPart(cfg);
}

void LidarOdometry::Parameters::Visualization::initializeModelPart(const Yaml & cfg)
{
  if (!cfg.has("model")) {
    return;
  }

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

    if (c.count("tf.x") != 0) {
      m.tf.x = c["tf.x"].as<float>();
    }
    if (c.count("tf.y") != 0) {
      m.tf.y = c["tf.y"].as<float>();
    }
    if (c.count("tf.z") != 0) {
      m.tf.z = c["tf.z"].as<float>();
    }
    if (c.count("tf.yaw") != 0) {
      m.tf.yaw = mrpt::DEG2RAD(c["tf.yaw"].as<float>());
    }
    if (c.count("tf.pitch") != 0) {
      m.tf.pitch = mrpt::DEG2RAD(c["tf.pitch"].as<float>());
    }
    if (c.count("tf.roll") != 0) {
      m.tf.roll = mrpt::DEG2RAD(c["tf.roll"].as<float>());
    }
    if (c.count("scale") != 0) {
      m.scale = c["scale"].as<float>();
    }
  }
}

void LidarOdometry::Parameters::SimpleMapOptions::initialize(const Yaml & cfg, Parameters & parent)
{
  YAML_LOAD_OPT(generate, bool);
  DECLARE_PARAMETER_IN_OPT(cfg, min_translation_between_keyframes, parent);
  DECLARE_PARAMETER_IN_OPT(cfg, min_rotation_between_keyframes, parent);
  YAML_LOAD_OPT(save_final_map_to_file, std::string);
  YAML_LOAD_OPT(add_non_keyframes_too, bool);
  YAML_LOAD_OPT(measure_from_last_kf_only, bool);
  YAML_LOAD_OPT(min_nearby_poses_occupied, uint32_t);
  ASSERTMSG_(
    min_nearby_poses_occupied >= 1, mrpt::format(
                                      "simplemap.min_nearby_poses_occupied=%u must be >= 1",
                                      static_cast<unsigned>(min_nearby_poses_occupied)));
  YAML_LOAD_OPT(generate_lazy_load_scan_files, bool);
  YAML_LOAD_OPT(save_gnss_max_age, double);
  YAML_LOAD_OPT(save_deskewed_scans, bool);
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
  YAML_LOAD_OPT(min_nearby_poses_occupied, uint32_t);
  ASSERTMSG_(
    min_nearby_poses_occupied >= 1, mrpt::format(
                                      "simplemap.min_nearby_poses_occupied=%u must be >= 1",
                                      static_cast<unsigned>(min_nearby_poses_occupied)));
  YAML_LOAD_OPT(load_existing_local_map, std::string);
  YAML_LOAD_OPT(save_final_local_map, std::string);
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
  YAML_LOAD_OPT(imu_initial_calibration_sample_count, uint32_t);
  YAML_LOAD_OPT(imu_initial_calibration_max_age, double);
  YAML_LOAD_OPT(use_imu_orientation, bool);
  YAML_LOAD_OPT(from_state_estimator_max_position_sigma, double);
  YAML_LOAD_OPT(from_state_estimator_max_orientation_sigma_deg, double);
  YAML_LOAD_OPT(from_state_estimator_timeout, double);

  if (cfg.has("fixed_initial_pose")) {
    ASSERT_(
      cfg["fixed_initial_pose"].isSequence() && cfg["fixed_initial_pose"].asSequence().size() == 6);

    auto & p = fixed_initial_pose;
    const auto seq = cfg["fixed_initial_pose"].asSequenceRange();
    for (size_t i = 0; i < 6; i++) {
      p[i] = seq.at(i).as<double>();
    }
  }
}

void LidarOdometry::Parameters::ObservationValidityChecks::initialize(const Yaml & cfg)
{
  YAML_LOAD_OPT(enabled, bool);
  YAML_LOAD_OPT(check_layer_name, std::string);
  YAML_LOAD_OPT(minimum_point_count, uint32_t);
}

void LidarOdometry::Parameters::IMUGravityCorrection::initialize(const Yaml & cfg)
{
  YAML_LOAD_OPT(enabled, bool);
  YAML_LOAD_OPT(sigma_deg, double);
  YAML_LOAD_OPT(averaging_samples, uint32_t);
  YAML_LOAD_OPT(max_age_seconds, double);

  if (enabled) {
    ASSERTMSG_(
      averaging_samples >= 1 && averaging_samples <= 200,
      mrpt::format(
        "imu_gravity_correction.averaging_samples=%u is out of valid range [1, 200]",
        static_cast<unsigned>(averaging_samples)));

    ASSERTMSG_(
      sigma_deg > 0, mrpt::format("imu_gravity_correction.sigma_deg=%.4f must be > 0", sigma_deg));
  }
}

void LidarOdometry::onParameterUpdate(const mrpt::containers::yaml & names_values)
{
  if (names_values.isNullNode() || names_values.empty()) {
    return;
  }

  ASSERT_(names_values.isMap());

  auto lckState = mrpt::lockHelper(state_mtx_);

  // Load parameters:
  setActive(names_values.getOrDefault("active", isActive()));

  params_.local_map_updates.enabled =
    names_values.getOrDefault("mapping_enabled", params_.local_map_updates.enabled);
  params_.simplemap.generate =
    names_values.getOrDefault("generate_simplemap", params_.simplemap.generate);

  // Special triggering reset "variable":
  if (names_values.getOrDefault("reset_state", false)) {
    this->enqueue_request([this]() {
      MRPT_LOG_INFO("Received a reset() command via parameters update.");
      reset();
    });
  }

  // and reflect changes in the GUI, if used.
#if !MOLA_VERSION_CHECK(2, 6, 0)
  this->enqueue_request([this]() {
    auto lckGuiMtx = mrpt::lockHelper(state_gui_mtx_);
    if (gui_.cbActive) {
      gui_.cbActive->setChecked(isActive());
      gui_.cbMapping->setChecked(params_.local_map_updates.enabled);
      gui_.cbSaveSimplemap->setChecked(params_.simplemap.generate);
    }
  });
#endif
}

void LidarOdometry::onExposeParameters()
{
  mrpt::containers::yaml nv = mrpt::containers::yaml::Map();
  nv["active"] = isActive();
  nv["mapping_enabled"] = params_.local_map_updates.enabled;
  nv["generate_simplemap"] = params_.simplemap.generate;
  nv["reset_state"] = false;

  this->exposeParameters(nv);
}

}  // namespace mola
