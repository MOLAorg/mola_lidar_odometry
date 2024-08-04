// -----------------------------------------------------------------------------
//   A Modular Optimization framework for Localization and mApping  (MOLA)
//
// Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
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
#include <mp2p_icp/Solver_GaussNewton.h>
#include <mp2p_icp/icp_pipeline_from_yaml.h>

// MRPT:
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/CAssimpModel.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>

// STD:
#include <chrono>
#include <thread>

// fix for older mrpt versions:
#include <mrpt/config.h>
#if MRPT_VERSION <= 0x020d00  // < v2.13.0
// YAML API:
#define asSequenceRange asSequence
#endif

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(LidarOdometry, FrontEndBase, mola)

LidarOdometry::LidarOdometry() = default;

LidarOdometry::~LidarOdometry()
{
  using namespace std::chrono_literals;

  try  // a dtor should never throw
  {
    {
      auto lck = mrpt::lockHelper(is_busy_mtx_);
      destructor_called_ = true;
    }

    worker_.clear();
    while (isBusy()) {
      MRPT_LOG_THROTTLE_WARN(
        2.0,
        "Destructor: waiting for remaining tasks on the worker "
        "threads...");
      std::this_thread::sleep_for(100ms);
    }

    if (params_.simplemap.generate)  //
      saveReconstructedMapToFile();

    if (params_.estimated_trajectory.save_to_file) saveEstimatedTrajectoryToFile();
  } catch (const std::exception & e) {
    std::cerr << "[~LidarOdometry] Exception: " << e.what();
  }
  catch (const std::exception & e) { std::cerr << "[~LidarOdometry] Exception: " << e.what(); }
}

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

void LidarOdometry::Parameters::AdaptiveThreshold::initialize(const Yaml & cfg)
{
  YAML_LOAD_REQ(enabled, bool);
  YAML_LOAD_REQ(initial_sigma, double);
  YAML_LOAD_REQ(min_motion, double);
  YAML_LOAD_REQ(kp, double);
  YAML_LOAD_REQ(alpha, double);
}

void LidarOdometry::Parameters::Visualization::initialize(const Yaml & cfg)
{
  YAML_LOAD_OPT(map_update_decimation, int);
  YAML_LOAD_OPT(show_trajectory, bool);
  YAML_LOAD_OPT(show_current_observation, bool);
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
  YAML_LOAD_OPT(enabled, bool);
  // TODO(jlbc): define enum "method"

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

  if (cfg.has("wheel_odometry_sensor_label"))
    params_.wheel_odometry_sensor_label = cfg["wheel_odometry_sensor_label"].as<std::string>();

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

  YAML_LOAD_OPT(params_, optimize_twist, bool);
  YAML_LOAD_OPT(params_, optimize_twist_rerun_min_trans, double);
  YAML_LOAD_OPT(params_, optimize_twist_rerun_min_rot_deg, double);
  YAML_LOAD_OPT(params_, optimize_twist_max_corrections, size_t);

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

  if (c.has("initial_localization"))
    params_.initial_localization.initialize(c["initial_localization"]);

  ENSURE_YAML_ENTRY_EXISTS(c, "navstate_fuse_params");
  state_.navstate_fuse.setMinLoggingLevel(this->getMinLoggingLevel());
  state_.navstate_fuse.initialize(c["navstate_fuse_params"]);

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
      ProfilerEntry tle(profiler_, "icp_functor_before_logging");

      mp2p_icp_filters::apply_filter_pipeline(state_.pc_filter3, m, profiler_);
    };
  }
  // system-wide profiler:
  profiler_.enable(params_.pipeline_profiler_enabled);

  // Create lidar segmentation algorithm:
  {
    ProfilerEntry tle(profiler_, "filterPointCloud_initialize");

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
    bool loadOk =
      state_.local_map->load_from_file(params_.local_map_updates.load_existing_local_map);
    ASSERT_(loadOk);
  }

  if (!params_.simplemap.load_existing_simple_map.empty()) {
    bool loadOk =
      state_.reconstructed_simplemap.loadFromFile(params_.simplemap.load_existing_simple_map);
    ASSERT_(loadOk);
  }

  // end of initialization:
  state_.initialized = true;
  state_.active = params_.start_active;

  MRPT_TRY_END
}
void LidarOdometry::spinOnce()
{
  MRPT_TRY_START

  ProfilerEntry tleg(profiler_, "spinOnce");

  processPendingUserRequests();

  MRPT_TRY_END
}

void LidarOdometry::reset()
{
  ASSERTMSG_(!lastInitConfig_.empty(), "initialize() must be called first.");

  state_ = MethodState();
  initialize(lastInitConfig_);
}

void LidarOdometry::onNewObservation(const CObservation::Ptr & o)
{
  MRPT_TRY_START
  ProfilerEntry tleg(profiler_, "onNewObservation");

  ASSERT_(o);

  auto lckState = mrpt::lockHelper(state_mtx_);

  if (!state_.initialized) {
    MRPT_LOG_THROTTLE_ERROR(
      2.0,
      "Discarding incoming observations: the system initialize() method "
      "has not be called yet!");
    return;
  }
  if (state_.fatal_error) {
    MRPT_LOG_THROTTLE_ERROR(2.0, "Discarding incoming observations: a fatal error ocurred above.");

    this->requestShutdown();  // request end of mola-cli app, if applicable
    return;
  }

  // Force a refresh of the GUI?
  // Executed here since
  // otherwise the GUI would never show up if inactive, or if the LIDAR
  // observations are misconfigured and are not been fed in.
  if ((visualizer_ && state_.local_map) && (state_.local_map->empty() || !state_.active)) {
    if (mrpt::Clock::nowDouble() - gui_.timestampLastUpdateUI > 1.0) updateVisualization({});
  }

  // SLAM enabled?
  if (!state_.active) return;

  // Is it an IMU obs?
  if (
    params_.imu_sensor_label &&
    std::regex_match(o->sensorLabel, params_.imu_sensor_label.value())) {
    {
      auto lck = mrpt::lockHelper(is_busy_mtx_);
      state_.worker_tasks++;
    }

    // Yes, it's an IMU obs:
    auto fut = worker_.enqueue(&LidarOdometry::onIMU, this, o);
    (void)fut;
  }

  // Is it odometry?
  if (
    params_.wheel_odometry_sensor_label &&
    std::regex_match(o->sensorLabel, params_.wheel_odometry_sensor_label.value())) {
    {
      auto lck = mrpt::lockHelper(is_busy_mtx_);
      state_.worker_tasks++;
    }
    auto fut = worker_.enqueue(&LidarOdometry::onWheelOdometry, this, o);
    (void)fut;
  }

  // Is it GNSS?
  if (
    params_.gnss_sensor_label &&
    std::regex_match(o->sensorLabel, params_.gnss_sensor_label.value())) {
    {
      auto lck = mrpt::lockHelper(is_busy_mtx_);
      state_.worker_tasks++;
    }
    auto fut = worker_.enqueue(&LidarOdometry::onGPS, this, o);
    (void)fut;
  }

  // Is it a LIDAR obs?
  for (const auto & re : params_.lidar_sensor_labels) {
    if (!std::regex_match(o->sensorLabel, re)) continue;

    // Yes, it's a LIDAR obs:
    const auto queued = worker_.pendingTasks();
    profiler_.registerUserMeasure("onNewObservation.queue_length", queued);
    if (queued > params_.max_worker_thread_queue_before_drop) {
      MRPT_LOG_THROTTLE_ERROR(1.0, "Dropping observation due to worker threads too busy.");
      profiler_.registerUserMeasure("onNewObservation.drop_observation", 1);
      return;
    }
    profiler_.enter("delay_onNewObs_to_process");

    {
      auto lck = mrpt::lockHelper(is_busy_mtx_);
      state_.worker_tasks++;
    }

    // Enqueue task:
    auto fut = worker_.enqueue(&LidarOdometry::onLidar, this, o);

    (void)fut;

    break;  // do not keep processing the list
  }

  MRPT_TRY_END
}

void LidarOdometry::onLidar(const CObservation::Ptr & o)
{
  {
    auto lck = mrpt::lockHelper(is_busy_mtx_);
    if (destructor_called_) return;  // abort pending tasks
  }

  // All methods that are enqueued into a thread pool should have its own
  // top-level try-catch:
  try {
    onLidarImpl(o);
  } catch (const std::exception & e) {
    MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
    state_.fatal_error = true;
  }
  {
    auto lck = mrpt::lockHelper(is_busy_mtx_);
    state_.worker_tasks--;
  }
}

// here happens the main stuff:
void LidarOdometry::onLidarImpl(const CObservation::Ptr & obs)
{
  using namespace std::string_literals;

  // Check if we need to process any pending async request:
  processPendingUserRequests();

  auto lckState = mrpt::lockHelper(state_mtx_);

  ASSERT_(obs);

  profiler_.leave("delay_onNewObs_to_process");

  // make sure data is loaded, if using an offline lazy-load dataset.
  obs->load();

  // Only process pointclouds that are sufficiently apart in time:
  const auto this_obs_tim = obs->timestamp;
  double lidar_delta_time = 0;
  if (
    state_.last_obs_tim_by_label.count(obs->sensorLabel) &&
    (lidar_delta_time =
       mrpt::system::timeDifference(state_.last_obs_tim_by_label[obs->sensorLabel], this_obs_tim)) <
      params_.min_time_between_scans) {
    // Drop observation.
    MRPT_LOG_DEBUG_FMT(
      "onLidarImpl: dropping observation, for %f< "
      "`min_time_between_scans`=%f.",
      lidar_delta_time, params_.min_time_between_scans);
    return;
  }

  ProfilerEntry tleg(profiler_, "onLidar");

  // Use the observation to update the estimated sensor range:
  if (!state_.estimated_sensor_max_range.has_value()) doInitializeEstimatedMaxSensorRange(*obs);

  // Handle multiple simultaneous LIDARs:
  mrpt::obs::CSensoryFrame sf;
  if (params_.multiple_lidars.lidar_count > 1) {
    // Synchronize 2+ lidars:
    state_.sync_obs[obs->sensorLabel] = obs;
    if (state_.sync_obs.size() < params_.multiple_lidars.lidar_count) {
      MRPT_LOG_THROTTLE_DEBUG(5.0, "Skipping ICP since still waiting for all of multiple LIDARs");
      return;
    }
    // now, keep all of them within the time window:
    for (const auto & [label, o] : state_.sync_obs) {
      const auto dt = std::abs(mrpt::system::timeDifference(o->timestamp, obs->timestamp));
      if (dt > params_.multiple_lidars.max_time_offset) continue;

      sf += o;  // include this observation
    }
    // and clear for the next iter:
    state_.sync_obs.clear();

    ASSERT_(!sf.empty());
    MRPT_LOG_DEBUG_STREAM(
      "multiple_lidars: " << sf.size() << " valid observations have been synchronized.");
  } else {
    // Single LIDAR:
    sf.insert(obs);
  }

  // Refresh dyn. variables used in the mp2p_icp pipelines:
  updatePipelineDynamicVariables();

  MRPT_LOG_DEBUG_STREAM("Dynamic variables: " << state_.parameter_source.printVariableValues());

  // Extract points from observation:
  auto observation = mp2p_icp::metric_map_t::Create();

  ProfilerEntry tle0(profiler_, "onLidar.0.apply_generators");

  ASSERT_(!sf.empty());
  const auto timeOfFirstSFObs = sf.getObservationByIndex(0)->timestamp;

  for (const auto & o : sf) {
    mp2p_icp::metric_map_t thisObs;
    mp2p_icp::metric_map_t * obsTrg = sf.size() == 1 ? observation.get() : &thisObs;

    mp2p_icp_filters::apply_generators(state_.obs_generators, *o, *obsTrg);

    // Update relative timestamps for multiple lidars:
    const double dt = mrpt::system::timeDifference(timeOfFirstSFObs, o->timestamp);

    state_.parameter_source.updateVariable("SENSOR_TIME_OFFSET", dt);
    // Make all changes effective and evaluate the variables now:
    state_.parameter_source.realize();

    mp2p_icp_filters::apply_filter_pipeline(state_.pc_filterAdjustTimes, *obsTrg, profiler_);

    // for multiple LiDAR setups:
    if (obsTrg != observation.get()) observation->merge_with(*obsTrg);
  }

  // Keep a copy of "raw" for visualization in the GUI:
  mp2p_icp::metric_map_t observationRawForViz;
  if (observation->layers.count("raw"))
    observationRawForViz.layers["raw"] = observation->layers.at("raw");

  tle0.stop();

  // Filter/segment the point cloud (optional, but normally will be
  // present):
  ProfilerEntry tle1(profiler_, "onLidar.1.filter_1st");

  mp2p_icp_filters::apply_filter_pipeline(state_.pc_filter1, *observation, profiler_);
  tle1.stop();

  ProfilerEntry tle1b(profiler_, "onLidar.1.filter_2nd");

  mp2p_icp_filters::apply_filter_pipeline(state_.pc_filter2, *observation, profiler_);

  tle1b.stop();

  profiler_.enter("onLidar.2.copy_vars");

  // Update sensor max range from the obs map layers:
  doUpdateEstimatedMaxSensorRange(*observation);

  // check observation validity:
  if (bool obsValid = doCheckIsValidObservation(*observation); !obsValid) {
    MRPT_LOG_WARN_FMT(
      "Observation discarded as non-valid for pathStep=%zu, timestamp=%s UTC",
      state_.estimated_trajectory.size(), mrpt::system::dateTimeToString(this_obs_tim).c_str());

    return;
  }

  // Store for next step:
  std::optional<mrpt::Clock::time_point> last_obs_tim;
  if (auto it = state_.last_obs_tim_by_label.find(obs->sensorLabel);
      it != state_.last_obs_tim_by_label.end())
    last_obs_tim = it->second;

  state_.last_obs_tim_by_label[obs->sensorLabel] = this_obs_tim;

  // Keep timestamps for logging purposes:
  state_.last_obs_timestamp = this_obs_tim;
  if (!state_.first_ever_timestamp) state_.first_ever_timestamp = this_obs_tim;

  profiler_.leave("onLidar.2.copy_vars");

  if (observation->empty()) {
    MRPT_LOG_WARN_STREAM(
      "Observation of type `" << obs->GetRuntimeClass()->className
                              << "` could not be converted into a "
                                 "pointcloud. Doing nothing.");
    return;
  }

  // Handle initial localization options:
  if (params_.initial_localization.enabled && !state_.initial_localization_done) {
    mrpt::poses::CPose3DPDFGaussian initPose;
    initPose.mean = mrpt::poses::CPose3D(params_.initial_localization.fixed_initial_pose);
    initPose.cov.setDiagonal(1e-12);

    // Fake an evolution to be able to have an initial velocity estimation:
    const auto t1 = mrpt::Clock::fromDouble(mrpt::Clock::toDouble(this_obs_tim) - 0.2);
    const auto t2 = mrpt::Clock::fromDouble(mrpt::Clock::toDouble(this_obs_tim) - 0.1);
    state_.navstate_fuse.fuse_pose(t1, initPose, NAVSTATE_LIODOM_FRAME);
    state_.navstate_fuse.fuse_pose(t2, initPose, NAVSTATE_LIODOM_FRAME);

    MRPT_LOG_INFO_STREAM("Initial re-localization done with pose: " << initPose.mean);

    state_.initial_localization_done = true;
  }

  // local map: used for LIDAR odometry:
  bool updateLocalMap = false;

  // Simplemap: an optional map to be saved to disk at the end of the mapping
  // session:
  bool updateSimpleMap = false;
  bool distance_enough_sm = false;

  // First time we cannot do ICP since we need at least two pointclouds:
  ASSERT_(state_.local_map);

  // Request the current pose/twist estimation:
  ProfilerEntry tleMotion(profiler_, "onLidar.2b.estimated_navstate");

  state_.last_motion_model_output =
    state_.navstate_fuse.estimated_navstate(this_obs_tim, NAVSTATE_LIODOM_FRAME);

  const bool hasMotionModel = state_.last_motion_model_output.has_value();

  tleMotion.stop();

  if (state_.local_map->empty()) {
    // Skip ICP.
    MRPT_LOG_DEBUG("First pointcloud: skipping ICP and directly adding to local map.");

    // Create a first KF (at origin)
    updateLocalMap = true;
    updateSimpleMap = true;     // update SimpleMap too
    distance_enough_sm = true;  // and treat this one as a KeyFrame with SF

    // Update trajectory too:
    {
      auto lck = mrpt::lockHelper(state_trajectory_mtx_);
      state_.estimated_trajectory.insert(this_obs_tim, state_.last_lidar_pose.mean);
    }

    // Define the current robot pose at the origin with minimal uncertainty
    // (cannot be zero).
    mrpt::poses::CPose3DPDFGaussian initPose;
    initPose.mean = mrpt::poses::CPose3D::Identity();
    initPose.cov.setDiagonal(1e-12);

    state_.navstate_fuse.fuse_pose(this_obs_tim, initPose, NAVSTATE_LIODOM_FRAME);
  } else {
    // Register point clouds using ICP:
    // ------------------------------------
    profiler_.enter("onLidar.2c.prepare_icp_in");

    mrpt::poses::CPose3DPDFGaussian initial_guess;

    // Use velocity model for the initial guess:
    const double dt = last_obs_tim ? mrpt::system::timeDifference(*last_obs_tim, this_obs_tim) : .0;

    ICP_Output out;
    ICP_Input in;

    in.init_guess_local_wrt_global = mrpt::math::TPose3D::Identity();

    if (state_.last_motion_model_output) {
      // ICP initial pose:
      in.init_guess_local_wrt_global = state_.last_motion_model_output->pose.mean.asTPose();

      // ICP prior term: any information!=0?
      if (state_.last_motion_model_output->pose.cov_inv != mrpt::math::CMatrixDouble66::Zero()) {
        // Send it to the ICP solver:
        in.prior.emplace(state_.last_motion_model_output->pose);

        // Special case: 2D lidars mean we are working on SE(2):
        if (std::dynamic_pointer_cast<mrpt::obs::CObservation2DRangeScan>(obs)) {
          // fix: z, pitch (rot_y), roll (rot_x):
          const double large_certainty = 1e6;

          auto & m = in.prior->mean;

          m.z(0);
          m.setYawPitchRoll(m.yaw(), .0, .0);

          in.prior->cov_inv(2, 2) = large_certainty;  // dz
          in.prior->cov_inv(3, 3) = large_certainty;  // rx
          in.prior->cov_inv(4, 4) = large_certainty;  // ry
        }
      }

      MRPT_LOG_DEBUG_STREAM(
        "Est.twist=" << (hasMotionModel ? state_.last_motion_model_output->twist.asString()
                                        : "(none)"s)
                     << " dt=" << dt << " s. "
                     << " Est. pose cov_inv:\n"
                     << state_.last_motion_model_output->pose.cov_inv.asString());
    } else {
      // Use the last pose without velocity motion model:
      MRPT_LOG_THROTTLE_WARN_FMT(
        2.0,
        "Not able to use velocity motion model for this timestep "
        "(pathStep=%zu, timestamp=%s UTC)",
        state_.estimated_trajectory.size(), mrpt::system::dateTimeToString(this_obs_tim).c_str());

      in.init_guess_local_wrt_global = state_.last_lidar_pose.mean.asTPose();
    }

    // If we don't have a valid twist estimation, use a larger ICP
    // correspondence threshold:
    in.align_kind = hasMotionModel ? AlignKind::RegularOdometry : AlignKind::NoMotionModel;

    in.icp_params = params_.icp[in.align_kind].icp_parameters;
    in.last_keyframe_pose = state_.last_lidar_pose.mean;

    if (state_.last_icp_timestamp)
      in.time_since_last_keyframe =
        mrpt::system::timeDifference(*state_.last_icp_timestamp, this_obs_tim);
    state_.last_icp_timestamp = this_obs_tim;

    profiler_.leave("onLidar.2c.prepare_icp_in");

    // ------------------------------------------------------
    // Run ICP
    // -----------------------------------------------------
    ProfilerEntry tle_icp(profiler_, "onLidar.3.run_icp");

    mrpt::math::TPose3D current_solution = in.init_guess_local_wrt_global;
    size_t twistCorrectionCount = 0;

    auto & icpCase = params_.icp.at(in.align_kind);

    icpCase.icp->setIterationHook([&](const mp2p_icp::ICP::IterationHook_Input & ih) {
      mp2p_icp::ICP::IterationHook_Output ho;

      if (!params_.optimize_twist) return ho;  // not enabled

      if (twistCorrectionCount >= params_.optimize_twist_max_corrections) return ho;

      const auto solutionDelta =
        ih.currentSolution->optimalPose - mrpt::poses::CPose3D(current_solution);

      // check minimum deltas:
      const double deltaTrans = solutionDelta.translation().norm();
      const double deltaRot =
        mrpt::poses::Lie::SO<3>::log(solutionDelta.getRotationMatrix()).norm();

      if (
        deltaTrans > params_.optimize_twist_rerun_min_trans ||
        deltaRot > mrpt::DEG2RAD(params_.optimize_twist_rerun_min_rot_deg)) {
        params_.optimize_twist_max_corrections++;

        MRPT_LOG_DEBUG_STREAM(
          "ICP hook: " << ih.currentIteration << " solutionDelta: trans=" << deltaTrans
                       << " deltaRot=" << mrpt::RAD2DEG(deltaRot));

        // request a restart, saving the new check point:
        ho.request_stop = true;
        current_solution = ih.currentSolution->optimalPose.asTPose();
      }
      return ho;
    });

    mp2p_icp::Results icp_result;
    auto icp_params = in.icp_params;
    size_t remainingIcpIters = icp_params.maxIterations;

    do {
      icp_params.maxIterations = remainingIcpIters;

      icpCase.icp->align(
        *observation, *state_.local_map, current_solution, icp_params, icp_result, in.prior);

      if (icp_result.nIterations <= remainingIcpIters)
        remainingIcpIters -= icp_result.nIterations;
      else  // who knows?...
        remainingIcpIters = 0;

      if (
        icp_result.terminationReason == mp2p_icp::IterTermReason::HookRequest &&
        in.time_since_last_keyframe > 0) {
        // Re-estimate twist:
        const auto incrPose = icp_result.optimal_tf.mean - in.last_keyframe_pose;

        const double At = in.time_since_last_keyframe;

        mrpt::math::TTwist3D tw;
        tw.vx = incrPose.x() / At;
        tw.vy = incrPose.y() / At;
        tw.vz = incrPose.z() / At;
        const auto logRot = mrpt::poses::Lie::SO<3>::log(incrPose.getRotationMatrix());
        tw.wx = logRot[0] / At;
        tw.wy = logRot[1] / At;
        tw.wz = logRot[2] / At;

        MRPT_LOG_DEBUG_STREAM(
          "ICP hook dt=" << At << ":\nnew estimated twist:" << tw.asString() << "\n"
                         << "old estimated twist:"
                         << state_.last_motion_model_output->twist.asString() << "\n");

        // Update twist dynamic variables, then re-run pipelines:
        updatePipelineTwistVariables(tw);
        // Make all changes effective and evaluate the variables now:
        state_.parameter_source.realize();

        // and re-apply 2nd pass:
        ProfilerEntry tle1c(profiler_, "onLidar.1.filter_2nd");

        mp2p_icp_filters::apply_filter_pipeline(state_.pc_filter2, *observation, profiler_);

        tle1c.stop();

        // for stats:
        profiler_.registerUserMeasure("onLidar.twist_corrections", 1.0);
      }

    } while (icp_result.terminationReason == mp2p_icp::IterTermReason::HookRequest);

    out.found_pose_to_wrt_from = icp_result.optimal_tf;
    out.goodness = icp_result.quality;
    out.icp_iterations = icp_result.nIterations;

    MRPT_LOG_DEBUG_FMT(
      "ICP (kind=%u): goodness=%.02f%% iters=%u pose=%s "
      "termReason=%s",
      static_cast<unsigned int>(in.align_kind), 100.0 * out.goodness,
      static_cast<unsigned int>(icp_result.nIterations),
      out.found_pose_to_wrt_from.getMeanVal().asString().c_str(),
      mrpt::typemeta::enum2str(icp_result.terminationReason).c_str());

    tle_icp.stop();
    // ------------------------------------------------------
    // (end, run ICP)
    // ------------------------------------------------------

    const bool icpIsGood = (out.goodness >= params_.min_icp_goodness);

    state_.last_icp_was_good = icpIsGood;
    state_.last_icp_quality = out.goodness;

    if (icpIsGood) state_.last_lidar_pose = out.found_pose_to_wrt_from;

    // Update velocity model:
    if (icpIsGood) {
      state_.navstate_fuse.fuse_pose(
        this_obs_tim, out.found_pose_to_wrt_from, NAVSTATE_LIODOM_FRAME);
    } else {
      state_.navstate_fuse.reset();
    }

    // Update trajectory too:
    if (icpIsGood) {
      auto lck = mrpt::lockHelper(state_trajectory_mtx_);
      state_.estimated_trajectory.insert(this_obs_tim, state_.last_lidar_pose.mean);
    }

    // Update for stats:
    state_.parameter_source.updateVariable("icp_iterations", out.icp_iterations);
    state_.parameter_source.updateVariable("twistCorrectionCount", twistCorrectionCount);

    // KISS-ICP adaptive threshold method:
    if (params_.adaptive_threshold.enabled) {
      // Run this even if "!icpIsGood":

      const mrpt::poses::CPose3D motionModelError =
        out.found_pose_to_wrt_from.mean - mrpt::poses::CPose3D(in.init_guess_local_wrt_global);

      doUpdateAdaptiveThreshold(motionModelError);

      MRPT_LOG_DEBUG_STREAM(
        "Adaptive threshold: sigma=" << state_.adapt_thres_sigma
                                     << " motionModelError=" << motionModelError.asString());
    }  // end adaptive threshold

    // Create distance checker on first usage:
    if (!state_.distance_checker_local_map)
      state_.distance_checker_local_map.emplace(
        params_.local_map_updates.measure_from_last_kf_only);

    if (!state_.distance_checker_simplemap)
      state_.distance_checker_simplemap.emplace(params_.simplemap.measure_from_last_kf_only);

    // Create a new KF if the distance since the last one is large
    // enough:
    const auto [isFirstPoseInChecker, distanceToClosest] =
      state_.distance_checker_local_map->check(state_.last_lidar_pose.mean);

    const double dist_eucl_since_last = distanceToClosest.norm();
    const double rot_since_last =
      mrpt::poses::Lie::SO<3>::log(distanceToClosest.getRotationMatrix()).norm();

    // clang-format off
        updateLocalMap =
            (icpIsGood &&
            // Only if we are in mapping mode:
            params_.local_map_updates.enabled &&
            // skip map update for the special ICP alignment without motion model
             hasMotionModel &&
             (isFirstPoseInChecker ||
              dist_eucl_since_last > params_.local_map_updates.min_translation_between_keyframes ||
              rot_since_last >
              mrpt::DEG2RAD(params_.local_map_updates.min_rotation_between_keyframes))
            );
    // clang-format on

    if (updateLocalMap) {
      state_.distance_checker_local_map->insert(state_.last_lidar_pose.mean);

      if (
        params_.local_map_updates.max_distance_to_keep_keyframes > 0 &&
        (state_.localmap_check_removal_counter++ >=
         params_.local_map_updates.check_for_removal_every_n)) {
        ProfilerEntry tleCleanup(profiler_, "onLidar.distant_kfs_cleanup");

        state_.localmap_check_removal_counter = 0;

        const auto nInit = state_.distance_checker_local_map->size();

        state_.distance_checker_local_map->removeAllFartherThan(
          state_.last_lidar_pose.mean, params_.local_map_updates.max_distance_to_keep_keyframes);

        const auto nFinal = state_.distance_checker_local_map->size();
        MRPT_LOG_DEBUG_STREAM("removeAllFartherThan: " << nInit << " => " << nFinal << " KFs");
      }
    }

    const auto [isFirstPoseInSMChecker, distanceToClosestSM] =
      state_.distance_checker_simplemap->check(state_.last_lidar_pose.mean);

    const double dist_eucl_since_last_sm = distanceToClosestSM.norm();
    const double rot_since_last_sm =
      mrpt::poses::Lie::SO<3>::log(distanceToClosestSM.getRotationMatrix()).norm();

    distance_enough_sm =
      isFirstPoseInSMChecker ||
      dist_eucl_since_last_sm > params_.simplemap.min_translation_between_keyframes ||
      rot_since_last_sm > mrpt::DEG2RAD(params_.simplemap.min_rotation_between_keyframes);

    // clang-format off
        updateSimpleMap =
            params_.simplemap.generate &&
            (icpIsGood &&
             (distance_enough_sm || params_.simplemap.add_non_keyframes_too)
            );
    // clang-format on

    if (updateSimpleMap && distance_enough_sm)
      state_.distance_checker_simplemap->insert(state_.last_lidar_pose.mean);

    MRPT_LOG_DEBUG_FMT(
      "Since last KF: dist=%5.03f m rotation=%.01f deg updateLocalMap=%s "
      "updateSimpleMap=%s",
      dist_eucl_since_last, mrpt::RAD2DEG(rot_since_last), updateLocalMap ? "YES" : "NO",
      updateSimpleMap ? "YES" : "NO");

  }  // end: yes, we can do ICP

  // If this was a bad ICP, and we just started with an empty map, re-start
  // again:
  if (!state_.last_icp_was_good && state_.estimated_trajectory.size() == 1) {
    // Re-start the local map:
    state_.local_map->clear();
    state_.estimated_trajectory.clear();
    updateLocalMap = false;
    state_.last_icp_was_good = true;

    MRPT_LOG_WARN("Bad first ICP, re-starting from scratch with a new local map");
  }

  // Should we create a new KF?
  if (updateLocalMap) {
    ProfilerEntry tle2(profiler_, "onLidar.4.update_local_map");

    // If the local map is empty, create it from this first observation:
    if (state_.local_map->empty()) {
      ProfilerEntry tle3(profiler_, "onLidar.4.update_local_map.create");
      MRPT_LOG_DEBUG("Creating local map since it was empty");

      for (const auto & o : sf)
        mp2p_icp_filters::apply_generators(state_.local_map_generators, *o, *state_.local_map);
    }

    ProfilerEntry tle3(profiler_, "onLidar.4.update_local_map.insert");

    // Merge "observation_layers_to_merge_local_map" in local map:
    // Input  metric_map_t: observation
    // Output metric_map_t: state_.local_map

    // 1/4: temporarily make a (shallow) copy of the observation layers into
    // the local map:
    for (const auto & [lyName, lyMap] : observation->layers) {
      ASSERTMSG_(
        state_.local_map->layers.count(lyName) == 0,
        mrpt::format(
          "Error: local map layer name '%s' collides with one of the "
          "observation layers, please use different layer names.",
          lyName.c_str()));

      state_.local_map->layers[lyName] = lyMap;  // shallow copy
    }

    // 2/4: Make sure dynamic variables are up-to-date,
    // in particular, [robot_x, ..., robot_roll]
    updatePipelineDynamicVariables();

    // 3/4: Apply pipeline
    mp2p_icp_filters::apply_filter_pipeline(state_.obs2map_merge, *state_.local_map, profiler_);

    // 4/4: remove temporary layers:
    for (const auto & [lyName, lyMap] : observation->layers) state_.local_map->layers.erase(lyName);

    tle3.stop();

    state_.local_map_needs_viz_update = true;

  }  // end done add a new KF to local map

  // Optional build simplemap:
  if (updateSimpleMap) {
    auto lck = mrpt::lockHelper(state_simplemap_mtx_);

    auto obsSF = mrpt::obs::CSensoryFrame::Create();
    // Add observations only if this is a real keyframe
    // (the alternative is this is a regular frame, but the option
    //  add_non_keyframes_too is set):
    if (distance_enough_sm) {
      *obsSF += sf;

      const auto curLidarStamp = obs->getTimeStamp();

      // insert GNSS too? Search for a close-enough observation:
      std::optional<double> closestTimeAbsDiff;
      mrpt::obs::CObservationGPS::Ptr closestGPS;

      for (const auto & [gpsStamp, gpsObs] : state_.last_gnss_) {
        const double timeDiff = std::abs(mrpt::system::timeDifference(gpsStamp, curLidarStamp));

        if (timeDiff > params_.simplemap.save_gnss_max_age) continue;

        if (!closestTimeAbsDiff || timeDiff < *closestTimeAbsDiff) {
          closestTimeAbsDiff = timeDiff;
          closestGPS = gpsObs;
        }
      }
      if (closestGPS) *obsSF += closestGPS;
    } else {
      // Otherwise (we are in here because add_non_keyframes_too).
      // Since we are adding anyway a "comment" observation with the
      // valid timestamp of this frame, it is enough for postprocessing
      // tools.
      ASSERT_(params_.simplemap.add_non_keyframes_too);
    }

    // Add metadata ("comment") observation:
    auto metadataObs = mrpt::obs::CObservationComment::Create();
    metadataObs->timestamp = this_obs_tim;
    metadataObs->sensorLabel = "metadata";

    mrpt::containers::yaml kf_metadata = mrpt::containers::yaml::Map();
    std::optional<mrpt::math::TBoundingBoxf> bbox;
    for (const auto & [layerName, layerMap] : observation->layers) {
      if (bbox)
        bbox = bbox->unionWith(layerMap->boundingBox());
      else
        bbox = layerMap->boundingBox();
    }
    if (bbox) {
      kf_metadata["frame_bbox_min"] = "'"s + bbox->min.asString() + "'"s;
      kf_metadata["frame_bbox_max"] = "'"s + bbox->max.asString() + "'"s;
    }

    // convert yaml to string:
    std::stringstream ss;
    ss << kf_metadata;
    metadataObs->text = ss.str();

    // insert it:
    *obsSF += metadataObs;

    // Add keyframe to simple map:
    MRPT_LOG_DEBUG_STREAM("New SimpleMap KeyFrame. SF=" << obsSF->size() << " observations.");

    std::optional<mrpt::math::TTwist3D> curTwist;
    if (hasMotionModel) curTwist = state_.last_motion_model_output->twist;

    state_.reconstructed_simplemap.insert(
      // Pose: mean + covariance
      mrpt::poses::CPose3DPDFGaussian::Create(state_.last_lidar_pose),
      // SensoryFrame: set of observations from this KeyFrame:
      obsSF,
      // twist
      curTwist);

    // Mechanism to free old SFs:
    // We cannot unload them right now, for the case when they are being
    // used in a GUI, etc.
    // (1/2) Add to the list:
    state_.past_simplemaps_observations[this_obs_tim] = obsSF;

    ProfilerEntry tleUnloadSM(profiler_, "onLidar.5.unload_past_sm_obs");

    // (2/2) Unload old lazy-load observations to save RAM, if applicable:
    constexpr size_t MAX_SIZE_UNLOAD_QUEUE = 100;
    unloadPastSimplemapObservations(MAX_SIZE_UNLOAD_QUEUE);

  }  // end update simple map

  // In any case, publish the vehicle pose, no matter if it's a keyframe or
  // not:
  doPublishUpdatedLocalization(this_obs_tim);

  // Publish new local map:
  doPublishUpdatedMap(this_obs_tim);

  // Optional debug traces to CSV file:
  doWriteDebugTracesFile(this_obs_tim);

  // Optional real-time GUI via MOLA VizInterface:
  if (visualizer_ && state_.local_map) {
    ProfilerEntry tle(profiler_, "onLidar.6.updateVisualization");

    updateVisualization(observationRawForViz);
  }
}

void LidarOdometry::onIMU(const CObservation::Ptr & o)
{
  // All methods that are enqueued into a thread pool should have its own
  // top-level try-catch:
  try {
    onIMUImpl(o);
  } catch (const std::exception & e) {
    MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
    state_.fatal_error = true;
  }

  {
    auto lck = mrpt::lockHelper(is_busy_mtx_);
    state_.worker_tasks--;
  }
}

void LidarOdometry::onIMUImpl(const CObservation::Ptr & o)
{
  ASSERT_(o);

  ProfilerEntry tleg(profiler_, "onIMU");
}

void LidarOdometry::onWheelOdometry(const CObservation::Ptr & o)
{
  // All methods that are enqueued into a thread pool should have its own
  // top-level try-catch:
  try {
    onWheelOdometryImpl(o);
  } catch (const std::exception & e) {
    MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
    state_.fatal_error = true;
  }

  {
    auto lck = mrpt::lockHelper(is_busy_mtx_);
    state_.worker_tasks--;
  }
}

void LidarOdometry::onWheelOdometryImpl(const CObservation::Ptr & o)
{
  ASSERT_(o);

  ProfilerEntry tleg(profiler_, "onWheelOdometry");

  auto odo = std::dynamic_pointer_cast<mrpt::obs::CObservationOdometry>(o);
  ASSERTMSG_(
    odo, mrpt::format(
           "Odometry observation with label '%s' does not have the expected "
           "type 'mrpt::obs::CObservationOdometry', it is '%s' instead",
           o->sensorLabel.c_str(), o->GetRuntimeClass()->className));

  MRPT_LOG_DEBUG_STREAM("onWheelOdometry: odom=" << odo->odometry);

  state_.navstate_fuse.fuse_odometry(*odo);
}

void LidarOdometry::onGPS(const CObservation::Ptr & o)
{
  // All methods that are enqueued into a thread pool should have its own
  // top-level try-catch:
  try {
    onGPSImpl(o);
  } catch (const std::exception & e) {
    MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
    state_.fatal_error = true;
  }

  {
    auto lck = mrpt::lockHelper(is_busy_mtx_);
    state_.worker_tasks--;
  }
}

void LidarOdometry::onGPSImpl(const CObservation::Ptr & o)
{
  ASSERT_(o);

  ProfilerEntry tleg(profiler_, "onGPS");

  auto gps = std::dynamic_pointer_cast<mrpt::obs::CObservationGPS>(o);
  ASSERTMSG_(
    gps, mrpt::format(
           "GPS observation with label '%s' does not have the expected "
           "type 'mrpt::obs::CObservationGPS', it is '%s' instead",
           o->sensorLabel.c_str(), o->GetRuntimeClass()->className));

  MRPT_LOG_DEBUG_FMT("GNSS observation received, t=%.03f", mrpt::Clock::toDouble(gps->timestamp));

  // Keep the latest GPS observations for simplemap insertion:
  state_.last_gnss_.emplace(gps->timestamp, gps);

  // remove old ones:
  while (state_.last_gnss_.size() > params_.gnss_queue_max_size) {
    state_.last_gnss_.erase(state_.last_gnss_.begin());
  }
}

bool LidarOdometry::isBusy() const
{
  bool b;
  is_busy_mtx_.lock();
  b = state_.worker_tasks != 0;
  is_busy_mtx_.unlock();
  return b || worker_.pendingTasks();
}

mrpt::poses::CPose3DInterpolator LidarOdometry::estimatedTrajectory() const
{
  auto lck = mrpt::lockHelper(state_trajectory_mtx_);
  return state_.estimated_trajectory;
}

mrpt::maps::CSimpleMap LidarOdometry::reconstructedMap() const
{
  auto lck = mrpt::lockHelper(state_simplemap_mtx_);
  return state_.reconstructed_simplemap;
}

// KISS-ICP adaptive threshold method (MIT License; code adapted to MRPT)
namespace
{
double computeModelError(const mrpt::poses::CPose3D & model_deviation, double max_range)
{
  const double theta = mrpt::poses::Lie::SO<3>::log(model_deviation.getRotationMatrix()).norm();
  const double delta_rot = 2.0 * max_range * std::sin(theta / 2.0);
  const double delta_trans = model_deviation.translation().norm();
  return delta_trans + delta_rot;
}
}  // namespace

void LidarOdometry::doUpdateAdaptiveThreshold(const mrpt::poses::CPose3D & lastMotionModelError)
{
  if (!state_.estimated_sensor_max_range.has_value()) return;

  const double max_range = state_.estimated_sensor_max_range.value();

  const double ALPHA = params_.adaptive_threshold.alpha;

  double model_error = computeModelError(lastMotionModelError, max_range);
  double rot_error = 0;
  if (state_.last_motion_model_output) {
    const auto & tw = state_.last_motion_model_output->twist;
    rot_error = 0.1 * mrpt::math::TVector3D(tw.wx, tw.wy, tw.wz).norm() * max_range;
  }

  computeModelError(lastMotionModelError, max_range);

  // proportional controller constant
  const double KP = params_.adaptive_threshold.kp;
  ASSERT_(KP > 1.0);

  const double new_sigma =
    (model_error + rot_error) * mrpt::saturate_val(KP * (1.0 - state_.last_icp_quality), 0.1, KP);

  if (state_.adapt_thres_sigma == 0)  // initial
    state_.adapt_thres_sigma = params_.adaptive_threshold.initial_sigma;

  state_.adapt_thres_sigma = ALPHA * state_.adapt_thres_sigma + (1.0 - ALPHA) * new_sigma;

  mrpt::keep_max(state_.adapt_thres_sigma, params_.adaptive_threshold.min_motion);

  MRPT_LOG_DEBUG_FMT(
    "model_error: %f  new_sigma: %f ICP q=%f sigma=%f", model_error, new_sigma,
    state_.last_icp_quality, state_.adapt_thres_sigma);
}

void LidarOdometry::doInitializeEstimatedMaxSensorRange(const mrpt::obs::CObservation & o)
{
  auto & maxRange = state_.estimated_sensor_max_range;
  ASSERT_(!maxRange.has_value());  // this method is for 1st call only

  mp2p_icp_filters::Generator gen;
  gen.params_.target_layer = "raw";
  gen.initialize({});

  mp2p_icp::metric_map_t map;
  gen.process(o, map);

  auto pts = map.point_layer("raw");

  if (pts->empty()) return;

  const auto bb = pts->boundingBox();
  double radius = 0.5 * (bb.max - bb.min).norm();

  mrpt::keep_max(radius, params_.absolute_minimum_sensor_range);

  maxRange = radius;

  MRPT_LOG_DEBUG_STREAM(
    "Estimated sensor max range=" << *state_.estimated_sensor_max_range
                                  << " (instantaneous=" << radius << ")");
}

void LidarOdometry::doUpdateEstimatedMaxSensorRange(const mp2p_icp::metric_map_t & m)
{
  const double ALPHA = params_.max_sensor_range_filter_coefficient;

  auto & maxRange = state_.estimated_sensor_max_range;
  ASSERT_(maxRange.has_value());  // this method is for subsequent calls only

  for (const auto & [layerName, layer] : m.layers) {
    auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(layer);
    if (!pts) continue;

    const auto bb = pts->boundingBox();
    double radius = 0.5 * (bb.max - bb.min).norm();

    mrpt::keep_max(radius, params_.absolute_minimum_sensor_range);

    state_.instantaneous_sensor_max_range = radius;

    // low-pass filter update:
    maxRange = maxRange.value() * ALPHA + radius * (1.0 - ALPHA);

    MRPT_LOG_DEBUG_STREAM(
      "Estimated sensor max range=" << *state_.estimated_sensor_max_range
                                    << " (instantaneous=" << radius << ")");

    // one layer is enough:
    return;
  }
  MRPT_LOG_DEBUG(
    "Estimated sensor max range could NOT be updated, no points layer "
    "found in observation metric_map_t");
}

bool LidarOdometry::doCheckIsValidObservation(const mp2p_icp::metric_map_t & m)
{
  if (!params_.observation_validity_checks.enabled) return true;  // it's valid

  auto it = m.layers.find(params_.observation_validity_checks.check_layer_name);
  ASSERTMSG_(
    it != m.layers.end(),
    mrpt::format(
      "Observation validity check expected observation layer '%s' but did not exist",
      params_.observation_validity_checks.check_layer_name.c_str()));

  auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(it->second);
  ASSERTMSG_(
    pts, mrpt::format(
           "Observation validity check expected observation layer '%s' of type CPointsMap",
           params_.observation_validity_checks.check_layer_name.c_str()));

  bool valid = pts->size() > params_.observation_validity_checks.minimum_point_count;

  MRPT_LOG_DEBUG_STREAM("Observation validity check: layer size=" << pts->size());
  return valid;
}

void LidarOdometry::updatePipelineTwistVariables(const mrpt::math::TTwist3D & tw)
{
  state_.parameter_source.updateVariable("vx", tw.vx);
  state_.parameter_source.updateVariable("vy", tw.vy);
  state_.parameter_source.updateVariable("vz", tw.vz);
  state_.parameter_source.updateVariable("wx", tw.wx);
  state_.parameter_source.updateVariable("wy", tw.wy);
  state_.parameter_source.updateVariable("wz", tw.wz);
}

void LidarOdometry::updatePipelineDynamicVariables()
{
  // Set dynamic variables for twist usage within ICP pipelines
  // (e.g. de-skew methods)
  {
    mrpt::math::TTwist3D twistForIcpVars = {0, 0, 0, 0, 0, 0};
    if (state_.last_motion_model_output) twistForIcpVars = state_.last_motion_model_output->twist;

    this->updatePipelineTwistVariables(twistForIcpVars);
  }

  // robot pose:
  const auto & p = state_.last_lidar_pose.mean;
  state_.parameter_source.updateVariable("robot_x", p.x());
  state_.parameter_source.updateVariable("robot_y", p.y());
  state_.parameter_source.updateVariable("robot_z", p.z());
  state_.parameter_source.updateVariable("robot_yaw", p.yaw());
  state_.parameter_source.updateVariable("robot_pitch", p.pitch());
  state_.parameter_source.updateVariable("robot_roll", p.roll());

  state_.parameter_source.updateVariable(
    "ADAPTIVE_THRESHOLD_SIGMA", state_.adapt_thres_sigma != 0
                                  ? state_.adapt_thres_sigma
                                  : params_.adaptive_threshold.initial_sigma);

  state_.parameter_source.updateVariable("ICP_ITERATION", 0);

  const auto ensureVarIsDefined = [&](const std::string & varName) {
    if (!state_.parameter_source.getVariableValues().count(varName))
      state_.parameter_source.updateVariable(varName, 0);
  };

  ensureVarIsDefined("icp_iterations");
  ensureVarIsDefined("SENSOR_TIME_OFFSET");
  ensureVarIsDefined("twistCorrectionCount");

  if (state_.estimated_sensor_max_range) {
    state_.parameter_source.updateVariable(
      "ESTIMATED_SENSOR_MAX_RANGE", *state_.estimated_sensor_max_range);
  }

  state_.parameter_source.updateVariable(
    "INSTANTANEOUS_SENSOR_MAX_RANGE", state_.instantaneous_sensor_max_range
                                        ? *state_.instantaneous_sensor_max_range
                                        : 20.0 /*default, only used once*/);

  if (state_.last_obs_timestamp && state_.first_ever_timestamp) {
    state_.parameter_source.updateVariable(
      "current_relative_timestamp",
      mrpt::system::timeDifference(*state_.first_ever_timestamp, *state_.last_obs_timestamp));
  }

  // Make all changes effective and evaluate the variables now:
  state_.parameter_source.realize();
}

void LidarOdometry::updateVisualization(const mp2p_icp::metric_map_t & currentObservation)
{
  ProfilerEntry tle(profiler_, "updateVisualization");

  gui_.timestampLastUpdateUI = mrpt::Clock::nowDouble();

  // In this point, we are called by the LIDAR worker thread, so it's safe
  // to read the state without mutexes.
  ASSERT_(visualizer_);

  // So they can be called at once at the end to minimize "flicker":
  std::vector<std::function<void()>> updateTasks;

  // Vehicle pose:
  // ---------------------------
  if (!state_.glVehicleFrame) {
    state_.glVehicleFrame = mrpt::opengl::CSetOfObjects::Create();

    if (const auto l = params_.visualization.current_pose_corner_size; l > 0) {
      auto glCorner = mrpt::opengl::stock_objects::CornerXYZ(l);
      state_.glVehicleFrame->insert(glCorner);
    }

    // 3D model:
    if (!params_.visualization.model.empty()) {
      const auto & _ = params_.visualization;

      for (const auto & model : _.model) {
        const auto localFileName = model.file;

        auto m = mrpt::opengl::CAssimpModel::Create();

        ASSERT_FILE_EXISTS_(localFileName);

        int loadFlags = mrpt::opengl::CAssimpModel::LoadFlags::RealTimeMaxQuality |
                        mrpt::opengl::CAssimpModel::LoadFlags::FlipUVs;

        m->loadScene(localFileName, loadFlags);

        m->setScale(model.scale);
        m->setPose(model.tf);

        state_.glVehicleFrame->insert(m);
      }
    }
  }

  // Update vehicle pose
  // -------------------------
  state_.glVehicleFrame->setPose(state_.last_lidar_pose.mean);
  updateTasks.emplace_back(
    [this]() { visualizer_->update_3d_object("liodom/vehicle", state_.glVehicleFrame); });

  // Update current observation
  // ----------------------------
  if (currentObservation.layers.count("raw") && params_.visualization.show_current_observation) {
    ProfilerEntry tle1(profiler_, "updateVisualization.update_cur_obs");

    // Visualize the raw data only, not the filtered layers:
    mp2p_icp::metric_map_t mm;
    mm.layers["raw"] = currentObservation.layers.at("raw");

    mp2p_icp::render_params_t rp;
    rp.points.allLayers.pointSize = 1.0f;
    auto & cm = rp.points.allLayers.colorMode.emplace();
    cm.colorMap = mrpt::img::cmJET;
    cm.keep_original_cloud_color = true;
    // cm.recolorizeByCoordinate     = mp2p_icp::Coordinate::Z;

    // get visualization
    auto glCurrentObservation = mm.get_visualization(rp);
    // move to current pose
    glCurrentObservation->setPose(state_.last_lidar_pose.mean);
    // and enqueue for updating in the opengl thread:
    updateTasks.emplace_back(
      [=]() { visualizer_->update_3d_object("liodom/cur_obs", glCurrentObservation); });
  }

  // Estimated path:
  // ------------------------
  if (params_.visualization.show_trajectory) {
    ProfilerEntry tle2(profiler_, "updateVisualization.update_traj");

    if (!state_.glEstimatedPath) {
      state_.glEstimatedPath = mrpt::opengl::CSetOfLines::Create();
      state_.glEstimatedPath->setColor_u8(0x30, 0x30, 0x30);

      state_.glPathGrp = mrpt::opengl::CSetOfObjects::Create();
    }
    // Update path viz:
    for (size_t i = state_.glEstimatedPath->size(); i < state_.estimated_trajectory.size(); i++) {
      auto it = state_.estimated_trajectory.begin();
      std::advance(it, i);

      const auto t = it->second.translation();

      if (state_.glEstimatedPath->empty())
        state_.glEstimatedPath->appendLine(t, t);
      else
        state_.glEstimatedPath->appendLineStrip(t);
    }
    state_.glPathGrp->clear();
    state_.glPathGrp->insert(mrpt::opengl::CSetOfLines::Create(*state_.glEstimatedPath));

    updateTasks.emplace_back(
      [this]() { visualizer_->update_3d_object("liodom/path", state_.glPathGrp); });
  }

  // GUI follow vehicle:
  // ---------------------------
  if (params_.visualization.camera_follows_vehicle) {
    updateTasks.emplace_back([this]() {
      visualizer_->update_viewport_look_at(state_.last_lidar_pose.mean.translation());
    });
  }

  if (params_.visualization.camera_rotates_with_vehicle) {
    updateTasks.emplace_back([this]() {
      thread_local std::optional<double> last_yaw;

      const double yaw = state_.last_lidar_pose.mean.yaw();
      double yawIncr = 0;
      if (last_yaw) yawIncr = mrpt::math::wrapToPi(yaw - *last_yaw);
      last_yaw = yaw;

      visualizer_->update_viewport_camera_azimuth(yawIncr, false /*incremental*/);
    });
  }

  // Local map:
  // -----------------------------
  if (
    (state_.mapUpdateCnt++ > params_.visualization.map_update_decimation) &&
    state_.local_map_needs_viz_update) {
    ProfilerEntry tle2(profiler_, "updateVisualization.update_local_map");

    state_.mapUpdateCnt = 0;
    state_.local_map_needs_viz_update = false;

    mp2p_icp::render_params_t rp;
    rp.points.allLayers.pointSize = params_.visualization.local_map_point_size;

    rp.points.allLayers.render_voxelmaps_free_space =
      params_.visualization.local_map_render_voxelmap_free_space;

    auto glMap = state_.local_map->get_visualization(rp);

    updateTasks.emplace_back([=]() { visualizer_->update_3d_object("liodom/localmap", glMap); });
  }

  // now, update all visual elements at once:
  for (const auto & ut : updateTasks) ut();

  // Show a warning if no lidar input is being received:
  if (state_.local_map->empty()) {
    const auto s = mrpt::format(
      "t=%.03f *WARNING* No input LiDAR observations received yet!", mrpt::Clock::nowDouble());
    visualizer_->output_console_message(s);
    return;
  }

  // Sub-window with custom UI
  // -------------------------------------
  if (!gui_.ui) {
    auto fut = visualizer_->create_subwindow("mola_lidar_odometry");
    gui_.ui = fut.get();

    // wait until this code is executed in the UI thread:
    auto fut2 = visualizer_->enqueue_custom_nanogui_code([this]() { internalBuildGUI(); });

    fut2.get();
  }

  ProfilerEntry tle3(profiler_, "updateVisualization.update_gui");

  gui_.lbIcpQuality->setCaption(
    mrpt::format("ICP quality: %.01f%%", 100.0 * state_.last_icp_quality));
  gui_.lbSigma->setCaption(mrpt::format("Threshold sigma: %.02f", state_.adapt_thres_sigma));
  if (state_.estimated_sensor_max_range)
    gui_.lbSensorRange->setCaption(
      mrpt::format("Est. max range: %.02f m", *state_.estimated_sensor_max_range));
  {
    // const double dt    = profiler_.getLastTime("onLidar");
    const double dtAvr = profiler_.getMeanTime("onLidar");
    gui_.lbTime->setCaption(mrpt::format(
      "Process time: %6.02f ms (%6.02f Hz)", 1e3 * dtAvr, dtAvr > 0 ? 1.0 / dtAvr : .0));
  }

  if (state_.last_motion_model_output) {
    const auto & tw = state_.last_motion_model_output->twist;
    const double speed = mrpt::math::TVector3D(tw.vx, tw.vy, tw.vz).norm();

    gui_.lbSpeed->setCaption(mrpt::format(
      "Speed: %.02f m/s | %.02f km/h | %.02f mph", speed, speed * 3600.0 / 1000.0,
      speed / 0.44704));
  }
}

void LidarOdometry::saveEstimatedTrajectoryToFile() const
{
  if (params_.estimated_trajectory.output_file.empty()) return;

  auto lck = mrpt::lockHelper(state_trajectory_mtx_);

  const auto fil = params_.estimated_trajectory.output_file;

  MRPT_LOG_INFO_STREAM(
    "Saving estimated trajectory with " << state_.estimated_trajectory.size()
                                        << " keyframes to file '" << fil << "' in TUM format...");

  state_.estimated_trajectory.saveToTextFile_TUM(fil);

  MRPT_LOG_INFO("Final trajectory saved.");
}

void LidarOdometry::saveReconstructedMapToFile() const
{
  if (params_.simplemap.save_final_map_to_file.empty()) return;

  // make sure the unload queue is empty first,
  // so if we have this feature enabled, all SF entries have been
  // "externalized" to make reading them much faster and less RAM
  // intensive:
  unloadPastSimplemapObservations(0 /* unload until queue is empty */);

  auto lck = mrpt::lockHelper(state_simplemap_mtx_);

  const auto fil = params_.simplemap.save_final_map_to_file;

  MRPT_LOG_INFO_STREAM(
    "Saving final simplemap with " << state_.reconstructed_simplemap.size()
                                   << " keyframes to file '" << fil << "'...");
  std::cout.flush();

  state_.reconstructed_simplemap.saveToFile(fil);

  MRPT_LOG_INFO("Final simplemap saved.");
}

void LidarOdometry::internalBuildGUI()
{
  ASSERT_(gui_.ui);

  gui_.ui->requestFocus();
  gui_.ui->setVisible(!params_.visualization.gui_subwindow_starts_hidden);
  gui_.ui->setPosition({5, 700});

  gui_.ui->setLayout(
    new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 5, 2));
  gui_.ui->setFixedWidth(340);

  auto tabWidget = gui_.ui->add<nanogui::TabWidget>();

  auto * tab1 = tabWidget->createTab("Status");
  tab1->setLayout(new nanogui::GroupLayout());

  auto * tab2 = tabWidget->createTab("Control");
  tab2->setLayout(new nanogui::GroupLayout());

  auto * tab3 = tabWidget->createTab("View");
  tab3->setLayout(new nanogui::GroupLayout());

  tabWidget->setActiveTab(0);

  // tab 1: status
  gui_.lbIcpQuality = tab1->add<nanogui::Label>(" ");
  gui_.lbSigma = tab1->add<nanogui::Label>(" ");
  gui_.lbSensorRange = tab1->add<nanogui::Label>(" ");
  gui_.lbSpeed = tab1->add<nanogui::Label>(" ");
  gui_.lbTime = tab1->add<nanogui::Label>(" ");

  // tab 2: control
  auto cbActive = tab2->add<nanogui::CheckBox>("Active");
  cbActive->setChecked(state_.active);
  cbActive->setCallback(
    [&](bool checked) { this->enqueue_request([this, checked]() { state_.active = checked; }); });

  auto cbMapping = tab2->add<nanogui::CheckBox>("Mapping enabled");
  cbMapping->setChecked(params_.local_map_updates.enabled);
  cbMapping->setCallback([&](bool checked) {
    this->enqueue_request([this, checked]() { params_.local_map_updates.enabled = checked; });
  });

  {
    auto * lbMsg = tab2->add<nanogui::Label>("Traj./map are saved at exit or when button clicked");
    lbMsg->setFontSize(14);
  }

  {
    auto * panel = tab2->add<nanogui::Widget>();
    panel->setLayout(
      new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Maximum, 1, 1));

    auto * cbSaveTrajectory = panel->add<nanogui::CheckBox>("Save trajectory");
    cbSaveTrajectory->setChecked(params_.estimated_trajectory.save_to_file);
    cbSaveTrajectory->setCallback([this](bool checked) {
      this->enqueue_request(
        [this, checked]() { params_.estimated_trajectory.save_to_file = checked; });
    });

    auto * edTrajOutFile = panel->add<nanogui::TextBox>();
    edTrajOutFile->setFontSize(13);
    edTrajOutFile->setEditable(true);
    edTrajOutFile->setAlignment(nanogui::TextBox::Alignment::Left);
    edTrajOutFile->setValue(params_.estimated_trajectory.output_file);
    edTrajOutFile->setCallback([this](const std::string & f) {
      this->enqueue_request([this, f]() { params_.estimated_trajectory.output_file = f; });
      return true;
    });
  }

  {
    auto * panel = tab2->add<nanogui::Widget>();
    panel->setLayout(
      new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Maximum, 1, 1));

    auto * cbSaveSimplemap = panel->add<nanogui::CheckBox>("Generate simplemap");
    cbSaveSimplemap->setChecked(params_.simplemap.generate);
    cbSaveSimplemap->setCallback([this](bool checked) {
      this->enqueue_request([this, checked]() { params_.simplemap.generate = checked; });
    });

    auto * edMapOutFile = panel->add<nanogui::TextBox>();
    edMapOutFile->setFontSize(13);
    edMapOutFile->setEditable(true);
    edMapOutFile->setAlignment(nanogui::TextBox::Alignment::Left);
    edMapOutFile->setValue(params_.simplemap.save_final_map_to_file);
    edMapOutFile->setCallback([this](const std::string & f) {
      this->enqueue_request([this, f]() { params_.simplemap.save_final_map_to_file = f; });
      return true;
    });
  }

  {
    auto * panel = tab2->add<nanogui::Widget>();
    panel->setLayout(
      new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Maximum, 1, 1));

    auto * btnSaveTraj = panel->add<nanogui::Button>("Save traj. now", ENTYPO_ICON_SAVE);
    btnSaveTraj->setFontSize(14);

    btnSaveTraj->setCallback([this]() { this->saveEstimatedTrajectoryToFile(); });

    auto * btnSaveMap = panel->add<nanogui::Button>("Save map now", ENTYPO_ICON_SAVE);
    btnSaveMap->setFontSize(14);

    btnSaveMap->setCallback([this]() { this->saveReconstructedMapToFile(); });
  }

  {
    auto * panel = tab2->add<nanogui::Widget>();
    panel->setLayout(
      new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Maximum, 1, 1));

    auto btnReset = panel->add<nanogui::Button>("Reset", ENTYPO_ICON_CCW);
    btnReset->setCallback([&]() { this->enqueue_request([this]() { this->reset(); }); });

    auto btnQuit = panel->add<nanogui::Button>("Quit", ENTYPO_ICON_ARROW_LEFT);
    btnQuit->setCallback([&]() { this->requestShutdown(); });
  }

  // tab 3: view
  auto * cbShowTraj = tab3->add<nanogui::CheckBox>("Show trajectory");
  cbShowTraj->setChecked(params_.visualization.show_trajectory);
  cbShowTraj->setCallback([&](bool checked) {
    this->enqueue_request([this, checked]() { params_.visualization.show_trajectory = checked; });
  });

  auto * cbShowObs = tab3->add<nanogui::CheckBox>("Show raw observation");
  cbShowObs->setChecked(params_.visualization.show_current_observation);
  cbShowObs->setCallback([&](bool checked) {
    this->enqueue_request(
      [this, checked]() { params_.visualization.show_current_observation = checked; });
  });

  auto * cbFollowVeh = tab3->add<nanogui::CheckBox>("Camera follows vehicle");
  cbFollowVeh->setChecked(params_.visualization.camera_follows_vehicle);
  cbFollowVeh->setCallback([&](bool checked) {
    this->enqueue_request(
      [this, checked]() { params_.visualization.camera_follows_vehicle = checked; });
  });

  auto * cbRotateVeh = tab3->add<nanogui::CheckBox>("Camera rotates with vehicle");
  cbRotateVeh->setChecked(params_.visualization.camera_rotates_with_vehicle);
  cbRotateVeh->setCallback([&](bool checked) {
    this->enqueue_request(
      [this, checked]() { params_.visualization.camera_rotates_with_vehicle = checked; });
  });

  auto * cbShowMsgs = tab3->add<nanogui::CheckBox>("Show log messages");
  cbShowMsgs->setChecked(params_.visualization.show_console_messages);
  cbShowMsgs->setCallback([&](bool checked) {
    this->enqueue_request(
      [this, checked]() { params_.visualization.show_console_messages = checked; });
  });

  this->mrpt::system::COutputLogger::logRegisterCallback(
    [&](
      std::string_view msg, const mrpt::system::VerbosityLevel level, std::string_view loggerName,
      const mrpt::Clock::time_point timestamp) {
      using namespace std::string_literals;

      if (!params_.visualization.show_console_messages) return;

      if (level < this->getMinLoggingLevel()) return;

      visualizer_->output_console_message(
        "["s + mrpt::system::timeLocalToString(timestamp) + "|"s + mrpt::typemeta::enum2str(level) +
        " |"s + std::string(loggerName) + "]"s + std::string(msg));
    });
}

void LidarOdometry::doPublishUpdatedLocalization(const mrpt::Clock::time_point & this_obs_tim)
{
  if (!anyUpdateMapSubscriber()) return;

  ProfilerEntry tle(profiler_, "advertiseUpdatedLocalization");

  LocalizationUpdate lu;
  lu.method = "lidar_odometry";
  lu.reference_frame = "odom";
  lu.timestamp = this_obs_tim;
  lu.pose = state_.last_lidar_pose.mean.asTPose();
  lu.cov = state_.last_lidar_pose.cov;

  advertiseUpdatedLocalization(lu);
}

void LidarOdometry::doPublishUpdatedMap(const mrpt::Clock::time_point & this_obs_tim)
{
  if (
    state_.localmap_advertise_updates_counter++ <
    params_.local_map_updates.publish_map_updates_every_n)
    return;

  if (!anyUpdateMapSubscriber()) return;

  ProfilerEntry tleCleanup(profiler_, "advertiseMap");
  state_.localmap_advertise_updates_counter = 0;

  MapUpdate mu;
  mu.method = "lidar_odometry";
  mu.reference_frame = "map";
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
    } else if (auto auxPts = layerMap->getAsSimplePointsMap();
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
  }
}

void LidarOdometry::unloadPastSimplemapObservations(const size_t maxSizeUnloadQueue) const
{
  auto lck = mrpt::lockHelper(state_simplemap_mtx_);

  auto & pso = state_.past_simplemaps_observations;

  while (pso.size() > maxSizeUnloadQueue) {
    for (auto & o : *pso.begin()->second) handleUnloadSinglePastObservation(o);

    pso.erase(pso.begin());
  }
}

void LidarOdometry::handleUnloadSinglePastObservation(mrpt::obs::CObservation::Ptr & o) const
{
  using mrpt::obs::CObservationPointCloud;

  // Generic method first: it will work with datasets providing input
  // observations *already* in lazy-load format:
  o->unload();

  // special case: point cloud
  auto oPts = std::dynamic_pointer_cast<CObservationPointCloud>(o);
  if (!oPts) return;

  if (oPts->isExternallyStored()) return;  // already external, do nothing.

  if (params_.simplemap.save_final_map_to_file.empty())
    return;  // no generation of simplemap requested by the user

  if (!params_.simplemap.generate_lazy_load_scan_files) return;  // feature is disabled

  ASSERT_(oPts->pointcloud);

  const std::string filename = mrpt::format(
    "%s_%.09f.bin", mrpt::system::fileNameStripInvalidChars(oPts->sensorLabel).c_str(),
    mrpt::Clock::toDouble(oPts->timestamp));

  // Create the default "/Images" directory.
  const auto & smFile = params_.simplemap.save_final_map_to_file;

  const std::string out_basedir = mrpt::system::pathJoin(
    {mrpt::system::extractFileDirectory(smFile),
     mrpt::system::extractFileName(smFile) + std::string("_Images")});

  if (!mrpt::system::directoryExists(out_basedir)) {
    bool dirCreatedOk = mrpt::system::createDirectory(out_basedir);
    ASSERTMSG_(
      dirCreatedOk, mrpt::format(
                      "Error creating lazy-load directory for "
                      "output simplemap: '%s'",
                      out_basedir.c_str()));

    MRPT_LOG_INFO_STREAM("Creating lazy-load directory for output .simplemap: " << out_basedir);
  }

  // Establish as reference external path base:
  mrpt::io::setLazyLoadPathBase(out_basedir);

  oPts->setAsExternalStorage(
    filename, CObservationPointCloud::ExternalStorageFormat::MRPT_Serialization);

  oPts->unload();  // this actually saves the data to disk
}

void LidarOdometry::enqueue_request(const std::function<void()> & userRequest)
{
  auto lck = mrpt::lockHelper(requests_mtx_);
  requests_.push_back(userRequest);
}

#ifdef HAVE_MOLA_KERNEL_RELOCALIZE_API
void LidarOdometry::relocalize_near_pose_pdf(const mrpt::poses::CPose3DPDFGaussian & p)
{
  //TODO!
}

void LidarOdometry::relocalize_from_gnss()
{
  //TODO!
}
#endif

void LidarOdometry::processPendingUserRequests()
{
  auto lckState = mrpt::lockHelper(state_mtx_);
  auto lck = mrpt::lockHelper(requests_mtx_);

  for (const auto & r : requests_) {
    try {
      r();
    } catch (const std::exception & e) {
      MRPT_LOG_ERROR_STREAM("Error processing asynchronous enqueue_request(): " << e.what());
    }
  }
  requests_.clear();
}

void LidarOdometry::doWriteDebugTracesFile(const mrpt::Clock::time_point & this_obs_tim)
{
  if (!params_.debug_traces.save_to_file) return;  // disabled

  if (debug_traces_of_ && !debug_traces_of_->is_open())
    return;  // apparently, an error creating the file

  bool firstLine = false;
  if (!debug_traces_of_) {
    debug_traces_of_.emplace();
    debug_traces_of_->open(params_.debug_traces.output_file);
    if (debug_traces_of_->is_open()) {
      MRPT_LOG_INFO_STREAM("Writing debug traces to: " << params_.debug_traces.output_file);
      firstLine = true;
    } else {
      MRPT_LOG_ERROR_STREAM(
        "Could not create debug traces file: " << params_.debug_traces.output_file);
      return;
    }
  }

  auto & of = debug_traces_of_.value();

  auto vars = state_.parameter_source.getVariableValues();
  vars["timestamp"] = mrpt::Clock::toDouble(this_obs_tim);
  vars["time_onLidar"] = profiler_.getLastTime("onLidar");

  if (firstLine) {
    for (const auto & [name, value] : vars)  //
      of << "\"" << name << "\",";
    of << "\n";
  }
  for (const auto & [name, value] : vars)  //
    of << mrpt::format("%f,", value);
  of << "\n";
}
