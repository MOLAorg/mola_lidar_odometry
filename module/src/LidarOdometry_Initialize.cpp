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

// MP2P_ICP:
#include <mp2p_icp/icp_pipeline_from_yaml.h>

namespace mola
{

namespace
{
void load_icp_set_of_params(
  LidarOdometry::Parameters::ICP_case & out, const mrpt::containers::yaml & cfg)
{
  const auto [icp, params] = mp2p_icp::icp_pipeline_from_yaml(cfg);

  out.icp = icp;
  out.icp_parameters = params;
}
}  // namespace

void LidarOdometry::initialize_frontend(const Yaml & c)
{
  MRPT_TRY_START

  auto lckState = mrpt::lockHelper(state_mtx_);

  this->setLoggerName("LidarOdometry");

  // make a copy of the initialization, for use in reset()
  lastInitConfig_ = c;

  // Load params:
  const auto cfg = c["params"];
  MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << cfg);

  ENSURE_YAML_ENTRY_EXISTS(cfg, "lidar_sensor_labels");
  if (cfg["lidar_sensor_labels"].isSequence()) {
    const auto lsl = cfg["lidar_sensor_labels"].asSequenceRange();
    for (const auto & sl : lsl) {
      const auto s = sl.as<std::string>();
      MRPT_LOG_DEBUG_STREAM("Adding as input lidar sensor label: " << s);
      params_.lidar_sensor_labels.emplace_back(s);
    }
  } else {
    ASSERT_(cfg["lidar_sensor_labels"].isScalar());
    const auto s = cfg["lidar_sensor_labels"].as<std::string>();
    MRPT_LOG_DEBUG_STREAM("Adding as input lidar sensor label: " << s);
    params_.lidar_sensor_labels.emplace_back(s);
  }
  ASSERT_(!params_.lidar_sensor_labels.empty());

  // Obs2map merge pipeline:
  ASSERT_(c["insert_observation_into_local_map"].isSequence());
  // Create, and copy my own verbosity level:
  state_.obs2map_merge = mp2p_icp_filters::filter_pipeline_from_yaml(
    c["insert_observation_into_local_map"], this->getMinLoggingLevel());

  // Attach to the parameter source for dynamic parameters:
  mp2p_icp::AttachToParameterSource(state_.obs2map_merge, state_.parameter_source);

  ASSERT_(!state_.obs2map_merge.empty());

  if (cfg.has("imu_sensor_label"))
    params_.imu_sensor_label = cfg["imu_sensor_label"].as<std::string>();

  if (cfg.has("gnss_sensor_label"))
    params_.gnss_sensor_label = cfg["gnss_sensor_label"].as<std::string>();

  ASSERT_(cfg.has("local_map_updates"));
  params_.local_map_updates.initialize(cfg["local_map_updates"], params_);

  if (cfg.has("multiple_lidars"))
    params_.multiple_lidars.initialize(cfg["multiple_lidars"], params_);

  YAML_LOAD_OPT(params_, min_time_between_scans, double);
  YAML_LOAD_REQ(params_, min_icp_goodness, double);
  YAML_LOAD_OPT(params_, max_sensor_range_filter_coefficient, double);
  YAML_LOAD_OPT(params_, absolute_minimum_sensor_range, double);
  YAML_LOAD_OPT(params_, start_active, bool);

  YAML_LOAD_OPT(params_, max_lidar_queue_before_drop, int32_t);
  YAML_LOAD_OPT(params_, gnss_queue_max_size, uint32_t);
  YAML_LOAD_OPT(params_, min_motion_model_xyz_cov_inv, double);

  YAML_LOAD_OPT(params_, optimize_twist, bool);
  YAML_LOAD_OPT(params_, optimize_twist_rerun_min_trans, double);
  YAML_LOAD_OPT(params_, optimize_twist_rerun_min_rot_deg, double);
  YAML_LOAD_OPT(params_, optimize_twist_max_corrections, size_t);

  YAML_LOAD_OPT(params_, publish_reference_frame, std::string);
  YAML_LOAD_OPT(params_, publish_vehicle_frame, std::string);

  if (cfg.has("adaptive_threshold"))
    params_.adaptive_threshold.initialize(cfg["adaptive_threshold"]);

  if (cfg.has("visualization")) params_.visualization.initialize(cfg["visualization"]);

  YAML_LOAD_OPT(params_, pipeline_profiler_enabled, bool);
  YAML_LOAD_OPT(params_, icp_profiler_enabled, bool);
  YAML_LOAD_OPT(params_, icp_profiler_full_history, bool);

  if (cfg.has("simplemap")) params_.simplemap.initialize(cfg["simplemap"], params_);

  if (cfg.has("estimated_trajectory"))
    params_.estimated_trajectory.initialize(cfg["estimated_trajectory"]);

  if (cfg.has("debug_traces")) params_.debug_traces.initialize(cfg["debug_traces"]);

  if (cfg.has("observation_validity_checks"))
    params_.observation_validity_checks.initialize(cfg["observation_validity_checks"]);

  if (c.has("initial_localization")) {
    params_.initial_localization.initialize(c["initial_localization"]);
  }

  // Watch for legacy (mola_lidar_odometry version <0.5.0) organization:
  if (c.has("navstate_fuse_params")) {
    THROW_EXCEPTION(
      "It seems you are using a legacy mola_lo pipeline config file. Please, refer to release "
      "notes for mola_lidar_odometry 0.5.0");
  }

  ENSURE_YAML_ENTRY_EXISTS(c, "icp_settings_with_vel");
  load_icp_set_of_params(params_.icp[AlignKind::RegularOdometry], c["icp_settings_with_vel"]);

  if (c.has("icp_settings_without_vel")) {
    load_icp_set_of_params(params_.icp[AlignKind::NoMotionModel], c["icp_settings_without_vel"]);
  } else {
    // Default: use the regular ICP settings:
    params_.icp[AlignKind::NoMotionModel] = params_.icp[AlignKind::RegularOdometry];
  }

  for (auto & [kind, icpCase] : params_.icp) {
    icpCase.icp->profiler().enable(params_.icp_profiler_enabled);
    icpCase.icp->profiler().enableKeepWholeHistory(params_.icp_profiler_full_history);

    // Attach all ICP instances to the parameter source for dynamic
    // parameters:
    icpCase.icp->attachToParameterSource(state_.parameter_source);

    // Attach final filter pipeline:
    // (mostly to save space & CPU when loggint to disk)
    icpCase.icp_parameters.functor_before_logging_local = [this](mp2p_icp::metric_map_t & m) {
      const ProfilerEntry tle(profiler_, "icp_functor_before_logging");

      mp2p_icp_filters::apply_filter_pipeline(state_.pc_filter3, m, profiler_);
    };
  }
  // system-wide profiler:
  profiler_.enable(params_.pipeline_profiler_enabled);

  // Create lidar segmentation algorithm:
  {
    const ProfilerEntry tle(profiler_, "filterPointCloud_initialize");

    // Observation -> map generator:
    if (c.has("observations_generator") && !c["observations_generator"].isNullNode()) {
      // Create, and copy my own verbosity level:
      state_.obs_generators = mp2p_icp_filters::generators_from_yaml(
        c["observations_generator"], this->getMinLoggingLevel());
    } else {
      MRPT_LOG_WARN(
        "Using default mp2p_icp_filters::Generator for "
        "observations since no YAML 'observations_generator' entry "
        "was given");

      auto defaultGen = mp2p_icp_filters::Generator::Create();
      defaultGen->initialize({});
      state_.obs_generators.push_back(defaultGen);
    }

    // Attach to the parameter source for dynamic parameters:
    mp2p_icp::AttachToParameterSource(state_.obs_generators, state_.parameter_source);

    if (c.has("observations_filter_adjust_timestamps")) {
      // Create, and copy my own verbosity level:
      state_.pc_filterAdjustTimes = mp2p_icp_filters::filter_pipeline_from_yaml(
        c["observations_filter_adjust_timestamps"], this->getMinLoggingLevel());

      // Attach to the parameter source for dynamic parameters:
      mp2p_icp::AttachToParameterSource(state_.pc_filterAdjustTimes, state_.parameter_source);
    } else {
      MRPT_LOG_WARN(
        "No YAML entry 'observations_filter_adjust_timestamps', this "
        "filter stage will have no effect.");
    }

    if (c.has("observations_filter_1st_pass")) {
      // Create, and copy my own verbosity level:
      state_.pc_filter1 = mp2p_icp_filters::filter_pipeline_from_yaml(
        c["observations_filter_1st_pass"], this->getMinLoggingLevel());

      // Attach to the parameter source for dynamic parameters:
      mp2p_icp::AttachToParameterSource(state_.pc_filter1, state_.parameter_source);
    } else {
      MRPT_LOG_WARN(
        "No YAML entry 'observations_filter_1st_pass', this "
        "filter stage will have no effect.");
    }

    if (c.has("observations_filter_2nd_pass")) {
      // Create, and copy my own verbosity level:
      state_.pc_filter2 = mp2p_icp_filters::filter_pipeline_from_yaml(
        c["observations_filter_2nd_pass"], this->getMinLoggingLevel());

      // Attach to the parameter source for dynamic parameters:
      mp2p_icp::AttachToParameterSource(state_.pc_filter2, state_.parameter_source);
    } else {
      MRPT_LOG_WARN(
        "No YAML entry 'observations_filter_2nd_pass', this "
        "filter stage will have no effect.");
    }

    if (c.has("observations_filter_final_pass")) {
      // Create, and copy my own verbosity level:
      state_.pc_filter3 = mp2p_icp_filters::filter_pipeline_from_yaml(
        c["observations_filter_final_pass"], this->getMinLoggingLevel());

      // Attach to the parameter source for dynamic parameters:
      mp2p_icp::AttachToParameterSource(state_.pc_filter3, state_.parameter_source);
    } else {
      MRPT_LOG_WARN(
        "No YAML entry 'observations_filter_final_pass', this "
        "filter stage will have no effect.");
    }

    // Local map generator:
    if (c.has("localmap_generator") && !c["localmap_generator"].isNullNode()) {
      // Create, and copy my own verbosity level:
      state_.local_map_generators =
        mp2p_icp_filters::generators_from_yaml(c["localmap_generator"], this->getMinLoggingLevel());
    } else {
      std::cout << "[warning] Using default mp2p_icp_filters::Generator "
                   "for the local map since no YAML 'localmap_generator' "
                   "entry was given\n";

      auto defaultGen = mp2p_icp_filters::Generator::Create();
      defaultGen->initialize({});
      state_.local_map_generators.push_back(defaultGen);
    }
    // Attach to the parameter source for dynamic parameters:
    mp2p_icp::AttachToParameterSource(state_.local_map_generators, state_.parameter_source);
  }

  // Parameterizable values in params_:
  params_.attachToParameterSource(state_.parameter_source);

  // Preload maps (multisession SLAM or localization-only):
  if (!params_.local_map_updates.load_existing_local_map.empty()) {
    const bool loadOk =
      state_.local_map->load_from_file(params_.local_map_updates.load_existing_local_map);
    ASSERT_(loadOk);

    state_.mark_local_map_as_updated();
    state_.mark_local_map_georef_as_updated();
  }

  if (!params_.simplemap.load_existing_simple_map.empty()) {
    const bool loadOk =
      state_.reconstructed_simplemap.loadFromFile(params_.simplemap.load_existing_simple_map);
    ASSERT_(loadOk);
  }

  // Attach to the state estimation module, which since MOLA-LO v0.5.0,
  // must run as a separate MOLA module:
  {
    auto mods = findService<mola::NavStateFilter>();
    ASSERTMSG_(
      mods.size() == 1,
      "No state estimation MOLA module (mola::NavStateFilter) was found. Please, check your MOLA "
      "system .yaml file");
    state_.navstate_fuse = std::dynamic_pointer_cast<NavStateFilter>(mods[0]);
    ASSERT_(state_.navstate_fuse);
    MRPT_LOG_DEBUG("Attached to the state estimation module");
  }

  // end of initialization:
  {
    auto lckStateFlags = mrpt::lockHelper(state_flags_mtx_);

    state_.initialized = true;
    state_.active = params_.start_active;
  }

  // Make runtime params exposed:
  onExposeParameters();

  MRPT_TRY_END
}

}  // namespace mola
