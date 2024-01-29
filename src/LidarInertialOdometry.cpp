/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */
/**
 * @file   LidarInertialOdometry.cpp
 * @brief  Header for main C++ class exposing LIDAR-inertial odometry
 * @author Jose Luis Blanco Claraco
 * @date   Sep 16, 2023
 */

#include <mola_lidar_odometry/LidarInertialOdometry.h>
#include <mola_yaml/yaml_helpers.h>
#include <mp2p_icp/Solver_GaussNewton.h>
#include <mp2p_icp/icp_pipeline_from_yaml.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/maps/CColouredPointsMap.h>
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

#include <chrono>
#include <thread>

using namespace mola;

// static const std::string ANNOTATION_NAME_PC_LAYERS =
// "lidar-pointcloud-layers";

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(LidarInertialOdometry, FrontEndBase, mola)

LidarInertialOdometry::LidarInertialOdometry() = default;

LidarInertialOdometry::~LidarInertialOdometry()
{
    using namespace std::chrono_literals;

    try  // a dtor should never throw
    {
        worker_.clear();
        while (isBusy())
        {
            MRPT_LOG_THROTTLE_WARN(
                2.0,
                "Destructor: waiting for remaining tasks on the worker "
                "threads...");
            std::this_thread::sleep_for(100ms);
        }

        if (params_.simplemap.generate &&
            !params_.simplemap.save_final_map_to_file.empty())
        {
            const auto fil = params_.simplemap.save_final_map_to_file;

            MRPT_LOG_INFO_STREAM(
                "Saving final simplemap with " << state_.reconstructedMap.size()
                                               << " keyframes to file '" << fil
                                               << "'...");

            state_.reconstructedMap.saveToFile(fil);

            MRPT_LOG_INFO("Final simplemap saved.");
        }

        if (params_.estimated_trajectory.save_to_file &&
            !params_.estimated_trajectory.output_file.empty())
        {
            const auto fil = params_.estimated_trajectory.output_file;

            MRPT_LOG_INFO_STREAM(
                "Saving final trajectory with "
                << state_.estimatedTrajectory.size() << " keyframes to file '"
                << fil << "' in TUM format...");

            state_.estimatedTrajectory.saveToTextFile_TUM(fil);

            MRPT_LOG_INFO("Final trajectory saved.");
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "[~LidarInertialOdometry] Exception: " << e.what();
    }
}

namespace
{
void load_icp_set_of_params(
    LidarInertialOdometry::Parameters::ICP_case& out,
    const mrpt::containers::yaml&                cfg)
{
    const auto [icp, params] = mp2p_icp::icp_pipeline_from_yaml(cfg);

    out.icp           = icp;
    out.icpParameters = params;
}
}  // namespace

void LidarInertialOdometry::Parameters::AdaptiveThreshold::initialize(
    const Yaml& cfg)
{
    YAML_LOAD_REQ(enabled, bool);
    YAML_LOAD_REQ(initial_sigma, double);
    YAML_LOAD_REQ(min_motion, double);
}

void LidarInertialOdometry::Parameters::Visualization::initialize(
    const Yaml& cfg)
{
    YAML_LOAD_OPT(map_update_decimation, int);
    YAML_LOAD_OPT(show_trajectory, bool);
    YAML_LOAD_OPT(current_pose_corner_size, double);

    YAML_LOAD_OPT(model_file, std::string);
    YAML_LOAD_OPT(model_tf.x, double);
    YAML_LOAD_OPT(model_tf.y, double);
    YAML_LOAD_OPT(model_tf.z, double);
    YAML_LOAD_OPT_DEG(model_tf.yaw, double);
    YAML_LOAD_OPT_DEG(model_tf.pitch, double);
    YAML_LOAD_OPT_DEG(model_tf.roll, double);

    YAML_LOAD_OPT(model_scale, double);

    YAML_LOAD_OPT(gui_subwindow_starts_hidden, bool);
}

void LidarInertialOdometry::Parameters::SimpleMapOptions::initialize(
    const Yaml& cfg, Parameters& parent)
{
    YAML_LOAD_OPT(generate, bool);
    DECLARE_PARAMETER_IN_OPT(cfg, min_translation_between_keyframes, parent);
    DECLARE_PARAMETER_IN_OPT(cfg, min_rotation_between_keyframes, parent);
    YAML_LOAD_OPT(save_final_map_to_file, std::string);
    YAML_LOAD_OPT(measure_from_last_kf_only, bool);
}

void LidarInertialOdometry::Parameters::MapUpdateOptions::initialize(
    const Yaml& cfg, Parameters& parent)
{
    DECLARE_PARAMETER_IN_REQ(cfg, min_translation_between_keyframes, parent);
    DECLARE_PARAMETER_IN_REQ(cfg, min_rotation_between_keyframes, parent);
    YAML_LOAD_OPT(measure_from_last_kf_only, bool);
}

void LidarInertialOdometry::Parameters::TrajectoryOutputOptions::initialize(
    const Yaml& cfg)
{
    YAML_LOAD_OPT(save_to_file, bool);
    YAML_LOAD_OPT(output_file, std::string);
}

void LidarInertialOdometry::initialize(const Yaml& c)
{
    MRPT_TRY_START

    // Load params:
    const auto cfg = c["params"];
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << cfg);

    ENSURE_YAML_ENTRY_EXISTS(cfg, "lidar_sensor_labels");
    if (cfg["lidar_sensor_labels"].isSequence())
    {
        for (const auto& sl : cfg["lidar_sensor_labels"].asSequence())
        {
            const auto s = sl.as<std::string>();
            MRPT_LOG_DEBUG_STREAM("Adding as input lidar sensor label: " << s);
            params_.lidar_sensor_labels.emplace_back(s);
        }
    }
    else
    {
        ASSERT_(cfg["lidar_sensor_labels"].isScalar());
        const auto s = cfg["lidar_sensor_labels"].as<std::string>();
        MRPT_LOG_DEBUG_STREAM("Adding as input lidar sensor label: " << s);
        params_.lidar_sensor_labels.emplace_back(s);
    }
    ASSERT_(!params_.lidar_sensor_labels.empty());

    ASSERT_(cfg["observation_layers_to_merge_local_map"].isSequence());

    for (const auto& sl :
         cfg["observation_layers_to_merge_local_map"].asSequence())
    {
        ASSERTMSG_(
            sl.isMap(),
            "Each entry in 'observation_layers_to_merge_local_map' must be a "
            "dictionary/map with values 'from_obs' and 'from_obs' with layer "
            "names");

        const auto sFrom = sl.asMap().at("from_obs").as<std::string>();
        const auto sInto = sl.asMap().at("into_map").as<std::string>();

        MRPT_LOG_DEBUG_STREAM(
            "Adding as observation_layers_to_merge_local_map: from_obs="
            << sFrom << " => into_map=" << sInto);

        params_.observation_layers_to_merge_local_map.emplace_back(
            sFrom, sInto);
    }
    ASSERT_(!params_.observation_layers_to_merge_local_map.empty());

    if (cfg.has("imu_sensor_label"))
    {
        params_.imu_sensor_label.emplace(
            cfg["imu_sensor_label"].as<std::string>());
    }

    if (cfg.has("wheel_odometry_sensor_label"))
    {
        params_.wheel_odometry_sensor_label.emplace(
            cfg["wheel_odometry_sensor_label"].as<std::string>());
    }

    ASSERT_(cfg.has("local_map_updates"));
    params_.local_map_updates.initialize(cfg["local_map_updates"], params_);

    YAML_LOAD_OPT(params_, min_time_between_scans, double);
    YAML_LOAD_OPT(params_, min_icp_goodness, double);
    YAML_LOAD_OPT(params_, max_sensor_range_filter_coefficient, double);

    if (cfg.has("adaptive_threshold"))
        params_.adaptive_threshold.initialize(cfg["adaptive_threshold"]);

    if (cfg.has("visualization"))
        params_.visualization.initialize(cfg["visualization"]);

    YAML_LOAD_OPT(params_, pipeline_profiler_enabled, bool);
    YAML_LOAD_OPT(params_, icp_profiler_enabled, bool);
    YAML_LOAD_OPT(params_, icp_profiler_full_history, bool);

    if (cfg.has("simplemap"))
        params_.simplemap.initialize(cfg["simplemap"], params_);

    if (cfg.has("estimated_trajectory"))
        params_.estimated_trajectory.initialize(cfg["estimated_trajectory"]);

    if (cfg.has("initial_twist"))
    {
        ASSERT_(
            cfg["initial_twist"].isSequence() &&
            cfg["initial_twist"].asSequence().size() == 6);

        auto&       tw  = params_.initial_twist.emplace();
        const auto& seq = cfg["initial_twist"].asSequence();
        for (size_t i = 0; i < 6; i++) tw[i] = seq.at(i).as<double>();
    }

    ENSURE_YAML_ENTRY_EXISTS(c, "navstate_fuse_params");
    params_.navstate_fuse_params.loadFrom(c["navstate_fuse_params"]);

    ENSURE_YAML_ENTRY_EXISTS(c, "icp_settings_with_vel");
    load_icp_set_of_params(
        params_.icp[AlignKind::RegularOdometry], c["icp_settings_with_vel"]);

    if (c.has("icp_settings_without_vel"))
    {
        load_icp_set_of_params(
            params_.icp[AlignKind::NoMotionModel],
            c["icp_settings_without_vel"]);
    }
    else
    {
        // Default: use the regular ICP settings:
        params_.icp[AlignKind::NoMotionModel] =
            params_.icp[AlignKind::RegularOdometry];
    }

    for (auto& [kind, icpCase] : params_.icp)
    {
        icpCase.icp->profiler().enable(params_.icp_profiler_enabled);
        icpCase.icp->profiler().enableKeepWholeHistory(
            params_.icp_profiler_full_history);

        // Attach all ICP instances to the parameter source for dynamic
        // parameters:
        icpCase.icp->attachToParameterSource(state_.icpParameterSource);
    }
    // system-wide profiler:
    profiler_.enable(params_.pipeline_profiler_enabled);

    // Create lidar segmentation algorithm:
    {
        ProfilerEntry tle(profiler_, "filterPointCloud_initialize");

        // Observation -> map generator:
        if (c.has("observations_generator") &&
            !c["observations_generator"].isNullNode())
        {
            // Create, and copy my own verbosity level:
            state_.obs_generators = mp2p_icp_filters::generators_from_yaml(
                c["observations_generator"], this->getMinLoggingLevel());
        }
        else
        {
            std::cout
                << "[warning] Using default mp2p_icp_filters::Generator for "
                   "observations since no YAML 'observations_generator' entry "
                   "was given\n";

            auto defaultGen = mp2p_icp_filters::Generator::Create();
            defaultGen->initialize({});
            state_.obs_generators.push_back(defaultGen);
        }

        // Attach to the parameter source for dynamic parameters:
        mp2p_icp::AttachToParameterSource(
            state_.obs_generators, state_.icpParameterSource);

        if (c.has("observations_filter"))
        {
            // Create, and copy my own verbosity level:
            state_.pc_filter = mp2p_icp_filters::filter_pipeline_from_yaml(
                c["observations_filter"], this->getMinLoggingLevel());

            // Attach to the parameter source for dynamic parameters:
            mp2p_icp::AttachToParameterSource(
                state_.pc_filter, state_.icpParameterSource);
        }

        // Local map generator:
        if (c.has("localmap_generator") &&
            !c["localmap_generator"].isNullNode())
        {
            // Create, and copy my own verbosity level:
            state_.local_map_generators =
                mp2p_icp_filters::generators_from_yaml(
                    c["localmap_generator"], this->getMinLoggingLevel());
        }
        else
        {
            std::cout << "[warning] Using default mp2p_icp_filters::Generator "
                         "for the local map since no YAML 'localmap_generator' "
                         "entry was given\n";

            auto defaultGen = mp2p_icp_filters::Generator::Create();
            defaultGen->initialize({});
            state_.local_map_generators.push_back(defaultGen);
        }
        // Attach to the parameter source for dynamic parameters:
        mp2p_icp::AttachToParameterSource(
            state_.local_map_generators, state_.icpParameterSource);
    }

    // Parameterizable values in params_:
    params_.attachToParameterSource(state_.icpParameterSource);

    state_.initialized = true;

    MRPT_TRY_END
}
void LidarInertialOdometry::spinOnce()
{
    MRPT_TRY_START

    ProfilerEntry tleg(profiler_, "spinOnce");

    //
    MRPT_TRY_END
}

void LidarInertialOdometry::reset()
{
    // TODO: there is more to be done! Check state_ fields changed in
    // initialize()
    state_ = MethodState();
}

void LidarInertialOdometry::onNewObservation(const CObservation::Ptr& o)
{
    MRPT_TRY_START
    ProfilerEntry tleg(profiler_, "onNewObservation");

    ASSERT_(o);

    if (!state_.initialized)
    {
        MRPT_LOG_THROTTLE_ERROR(
            2.0,
            "Discarding incoming observations: the system initialize() method "
            "has not be called yet!");
        return;
    }
    if (state_.fatal_error)
    {
        MRPT_LOG_THROTTLE_ERROR(
            2.0,
            "Discarding incoming observations: a fatal error ocurred above.");
        return;
    }

    // Is it an IMU obs?
    if (params_.imu_sensor_label &&
        std::regex_match(o->sensorLabel, params_.imu_sensor_label.value()))
    {
        {
            auto lck = mrpt::lockHelper(is_busy_mtx_);
            state_.worker_tasks++;
        }

        // Yes, it's an IMU obs:
        auto fut = worker_.enqueue(&LidarInertialOdometry::onIMU, this, o);
        (void)fut;
    }

    // Is it odometry?
    if (params_.wheel_odometry_sensor_label &&
        std::regex_match(
            o->sensorLabel, params_.wheel_odometry_sensor_label.value()))
    {
        {
            auto lck = mrpt::lockHelper(is_busy_mtx_);
            state_.worker_tasks++;
        }
        auto fut =
            worker_.enqueue(&LidarInertialOdometry::onWheelOdometry, this, o);
        (void)fut;
    }

    // Is it a LIDAR obs?
    for (const auto& re : params_.lidar_sensor_labels)
    {
        if (!std::regex_match(o->sensorLabel, re)) continue;

        // Yes, it's a LIDAR obs:
        const auto queued = worker_.pendingTasks();
        profiler_.registerUserMeasure("onNewObservation.queue_length", queued);
        if (queued > params_.max_worker_thread_queue_before_drop)
        {
            MRPT_LOG_THROTTLE_ERROR(
                1.0, "Dropping observation due to worker threads too busy.");
            profiler_.registerUserMeasure(
                "onNewObservation.drop_observation", 1);
            return;
        }
        profiler_.enter("delay_onNewObs_to_process");

        {
            auto lck = mrpt::lockHelper(is_busy_mtx_);
            state_.worker_tasks++;
        }

        // Enqueue task:
        auto fut = worker_.enqueue(&LidarInertialOdometry::onLidar, this, o);

        (void)fut;

        break;  // do not keep processing the list
    }

    MRPT_TRY_END
}

void LidarInertialOdometry::onLidar(const CObservation::Ptr& o)
{
    // All methods that are enqueued into a thread pool should have its own
    // top-level try-catch:
    try
    {
        onLidarImpl(o);
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
        state_.fatal_error = true;
    }
    {
        auto lck = mrpt::lockHelper(is_busy_mtx_);
        state_.worker_tasks--;
    }
}

// here happens the main stuff:
void LidarInertialOdometry::onLidarImpl(const CObservation::Ptr& o)
{
    using namespace std::string_literals;

    ASSERT_(o);

    ProfilerEntry tleg(profiler_, "onLidar");
    profiler_.leave("delay_onNewObs_to_process");

    // make sure data is loaded, if using an offline lazy-load dataset.
    o->load();

    // Only process pointclouds that are sufficiently apart in time:
    const auto this_obs_tim     = o->timestamp;
    double     lidar_delta_time = 0;
    if (state_.last_obs_tim && (lidar_delta_time = mrpt::system::timeDifference(
                                    *state_.last_obs_tim, this_obs_tim)) <
                                   params_.min_time_between_scans)
    {
        // Drop observation.
        MRPT_LOG_DEBUG_FMT(
            "onLidarImpl: dropping observation, for %f< "
            "`min_time_between_scans`=%f.",
            lidar_delta_time, params_.min_time_between_scans);
        return;
    }

    // Use the observation to update the estimated sensor range:
    if (!state_.estimated_sensor_max_range.has_value())
        doInitializeEstimatedMaxSensorRange(*o);

    // Refresh dyn. variables used in the mp2p_icp pipelines:
    updatePipelineDynamicVariables();

    MRPT_LOG_DEBUG_STREAM(
        "Dynamic variables: "
        << state_.icpParameterSource.printVariableValues());

    // Extract points from observation:
    auto observation = mp2p_icp::metric_map_t::Create();

    ProfilerEntry tle0(profiler_, "onLidar.0.apply_generators");

    mp2p_icp_filters::apply_generators(state_.obs_generators, *o, *observation);

    tle0.stop();

    // Filter/segment the point cloud (optional, but normally will be
    // present):
    ProfilerEntry tle1(profiler_, "onLidar.1.filter_pointclouds");

    mp2p_icp_filters::apply_filter_pipeline(state_.pc_filter, *observation);

    tle1.stop();

    // Update sensor max range from the obs map layers:
    doUpdateEstimatedMaxSensorRange(*observation);

    profiler_.enter("onLidar.2.copy_vars");

    // Store for next step:
    const auto last_obs_tim = state_.last_obs_tim;
    state_.last_obs_tim     = this_obs_tim;

    profiler_.leave("onLidar.2.copy_vars");

    if (observation->empty())
    {
        MRPT_LOG_WARN_STREAM(
            "Observation of type `" << o->GetRuntimeClass()->className
                                    << "` could not be converted into a "
                                       "pointcloud. Doing nothing.");
        return;
    }

    // local map: used for LIDAR odometry:
    bool updateLocalMap = false;

    // Simplemap: an optional map to be saved to disk at the end of the mapping
    // session:
    bool updateSimpleMap = false;

    // First time we cannot do ICP since we need at least two pointclouds:
    ASSERT_(state_.local_map);

    // Request the current pose/twist estimation:
    std::optional<NavState> motionModelOutput =
        state_.navstate_fuse.estimated_navstate(this_obs_tim);

    const bool hasMotionModel = motionModelOutput.has_value();

    if (state_.local_map->empty())
    {
        // Skip ICP.
        MRPT_LOG_DEBUG(
            "First pointcloud: skipping ICP and directly adding to local map.");

        // Create a first KF (at origin)
        updateLocalMap = true;

        // Update trajectory too:
        {
            auto lck = mrpt::lockHelper(stateTrajectory_mtx_);
            state_.estimatedTrajectory.insert(
                this_obs_tim, state_.last_lidar_pose.mean);
        }

        // Define the current robot pose at the origin with minimal uncertainty
        // (cannot be zero).
        mrpt::poses::CPose3DPDFGaussian initPose;
        initPose.mean = mrpt::poses::CPose3D::Identity();
        initPose.cov.setDiagonal(1e-12);

        state_.navstate_fuse.fuse_pose(this_obs_tim, initPose);

        // set optional initial twist:
        if (params_.initial_twist)
        {
            state_.navstate_fuse.force_last_twist(*params_.initial_twist);

            // and reset since we only want to enforce the a-priori twist once:
            params_.initial_twist.reset();
        }
    }
    else
    {
        // Register point clouds using ICP:
        // ------------------------------------
        profiler_.enter("onLidar.2c.prepare_icp_in");

        mrpt::poses::CPose3DPDFGaussian initial_guess;

        // Use velocity model for the initial guess:
        const double dt = last_obs_tim ? mrpt::system::timeDifference(
                                             *last_obs_tim, this_obs_tim)
                                       : .0;

        ICP_Output icp_out;
        ICP_Input  icp_in;

        icp_in.init_guess_local_wrt_global = mrpt::math::TPose3D::Identity();

        if (motionModelOutput)
        {
            // ICP initial pose:
            icp_in.init_guess_local_wrt_global =
                motionModelOutput->pose.mean.asTPose();

            // ICP prior term: any information!=0?
            if (motionModelOutput->pose.cov_inv !=
                mrpt::math::CMatrixDouble66::Zero())
            {
                icp_in.prior.emplace(motionModelOutput->pose);
            }
        }

        // Send out to icp:
        icp_in.local_pc  = observation;
        icp_in.global_pc = state_.local_map;
        icp_in.debug_str = "lidar_odom";

        // If we don't have a valid twist estimation, use a larger ICP
        // correspondence threshold:
        icp_in.align_kind = hasMotionModel ? AlignKind::RegularOdometry
                                           : AlignKind::NoMotionModel;

        icp_in.icp_params = params_.icp[icp_in.align_kind].icpParameters;

        profiler_.leave("onLidar.2c.prepare_icp_in");

        // Run ICP:
        {
            ProfilerEntry tle(profiler_, "onLidar.3.icp_latest");
            run_one_icp(icp_in, icp_out);
        }
        const bool icpIsGood     = icp_out.goodness >= params_.min_icp_goodness;
        state_.last_icp_was_good = icpIsGood;
        state_.last_icp_quality  = icp_out.goodness;

        if (icpIsGood) state_.last_lidar_pose = icp_out.found_pose_to_wrt_from;

        // Update velocity model:
        if (icpIsGood)
        {
            state_.navstate_fuse.fuse_pose(
                this_obs_tim, icp_out.found_pose_to_wrt_from);
        }
        else
        {
            state_.navstate_fuse.reset();
        }

        // Update trajectory too:
        if (icpIsGood)
        {
            auto lck = mrpt::lockHelper(stateTrajectory_mtx_);
            state_.estimatedTrajectory.insert(
                this_obs_tim, state_.last_lidar_pose.mean);
        }

        MRPT_LOG_DEBUG_STREAM(
            "Est.twist=" << (hasMotionModel
                                 ? motionModelOutput->twist.asString()
                                 : "(none)"s)
                         << " dt=" << dt << " s. "
                         << " Est. pose cov_inv:\n"
                         << motionModelOutput->pose.cov_inv.asString());
        MRPT_LOG_DEBUG_STREAM(
            "Time since last scan=" << mrpt::system::formatTimeInterval(dt));

        // KISS-ICP adaptive threshold method:
        if (params_.adaptive_threshold.enabled && icpIsGood)
        {
            const mrpt::poses::CPose3D motionModelError =
                icp_out.found_pose_to_wrt_from.mean -
                mrpt::poses::CPose3D(icp_in.init_guess_local_wrt_global);

            doUpdateAdaptiveThreshold(motionModelError);

            MRPT_LOG_DEBUG_STREAM(
                "Adaptive threshold: sigma=" << state_.adapt_thres_sigma
                                             << " motionModelError="
                                             << motionModelError.asString());
        }  // end adaptive threshold

        // Create distance checker on first usage:
        if (!state_.distanceCheckerLocalMap)
            state_.distanceCheckerLocalMap.emplace(
                params_.local_map_updates.measure_from_last_kf_only);

        if (!state_.distanceCheckerSimplemap)
            state_.distanceCheckerSimplemap.emplace(
                params_.simplemap.measure_from_last_kf_only);

        // Create a new KF if the distance since the last one is large
        // enough:
        const auto [isFirstPoseInChecker, distanceToClosest] =
            state_.distanceCheckerLocalMap->check(state_.last_lidar_pose.mean);

        const double dist_eucl_since_last = distanceToClosest.norm();
        const double rot_since_last =
            mrpt::poses::Lie::SO<3>::log(distanceToClosest.getRotationMatrix())
                .norm();

        // clang-format off
        updateLocalMap =
            (icpIsGood &&
            // skip map update for the special ICP alignment without motion model
             hasMotionModel &&
             (isFirstPoseInChecker ||
              dist_eucl_since_last > params_.local_map_updates.min_translation_between_keyframes ||
              rot_since_last > mrpt::DEG2RAD(params_.local_map_updates.min_rotation_between_keyframes)));
        // clang-format on

        if (updateLocalMap)
            state_.distanceCheckerLocalMap->insert(state_.last_lidar_pose.mean);

        const auto [isFirstPoseInSMChecker, distanceToClosestSM] =
            state_.distanceCheckerSimplemap->check(state_.last_lidar_pose.mean);

        const double dist_eucl_since_last_sm = distanceToClosestSM.norm();
        const double rot_since_last_sm =
            mrpt::poses::Lie::SO<3>::log(
                distanceToClosestSM.getRotationMatrix())
                .norm();

        // clang-format off
        updateSimpleMap =
            (params_.simplemap.generate) &&
            (icpIsGood &&
             (isFirstPoseInSMChecker ||
              dist_eucl_since_last_sm > params_.simplemap.min_translation_between_keyframes ||
              rot_since_last_sm > mrpt::DEG2RAD(params_.simplemap.min_rotation_between_keyframes)));
        // clang-format on

        if (updateSimpleMap)
            state_.distanceCheckerSimplemap->insert(
                state_.last_lidar_pose.mean);

        MRPT_LOG_DEBUG_FMT(
            "Since last KF: dist=%5.03f m rotation=%.01f deg updateLocalMap=%s "
            "updateSimpleMap=%s",
            dist_eucl_since_last, mrpt::RAD2DEG(rot_since_last),
            updateLocalMap ? "YES" : "NO", updateSimpleMap ? "YES" : "NO");

    }  // end: yes, we can do ICP

    // If this was a bad ICP, and we just started with an empty map, re-start
    // again:
    if (!state_.last_icp_was_good && state_.estimatedTrajectory.size() == 1)
    {
        // Re-start the local map:
        state_.local_map->clear();
        state_.estimatedTrajectory.clear();
        updateLocalMap           = false;
        state_.last_icp_was_good = true;

        MRPT_LOG_WARN(
            "Bad first ICP, re-starting from scratch with a new local map");
    }

    // Should we create a new KF?
    if (updateLocalMap)
    {
        ProfilerEntry tle2(profiler_, "onLidar.4.update_local_map");

        // If the local map is empty, create it from this first observation:
        if (state_.local_map->empty())
        {
            ProfilerEntry tle3(profiler_, "onLidar.4.update_local_map.create");
            MRPT_LOG_DEBUG("Creating local map since it was empty");

            mp2p_icp_filters::apply_generators(
                state_.local_map_generators, *o, *state_.local_map);
        }

        ProfilerEntry tle3(profiler_, "onLidar.4.update_local_map.insert");

        // Merge "observation_layers_to_merge_local_map" in local map:
        for (const auto& [lyObs, lyMap] :
             params_.observation_layers_to_merge_local_map)
        {
            auto itObs = observation->layers.find(lyObs);
            ASSERTMSG_(
                itObs != observation->layers.end(),
                mrpt::format(
                    "Error inserting LIDAR observation into local map: "
                    "expected a metric_map_t layer named '%s' in the "
                    "observation, but it was not found. Actual contents: %s",
                    lyObs.c_str(), observation->contents_summary().c_str()));

            auto itLocalMap = state_.local_map->layers.find(lyMap);
            ASSERTMSG_(
                itLocalMap != state_.local_map->layers.end(),
                mrpt::format(
                    "Error inserting LIDAR observation into local map: "
                    "expected a metric_map_t layer named '%s' in the local "
                    "map, but it was "
                    "not found. Actual contents: %s",
                    lyMap.c_str(),
                    state_.local_map->contents_summary().c_str()));

            mrpt::obs::CObservationPointCloud obsPc;
            obsPc.timestamp = o->timestamp;
            obsPc.pointcloud =
                std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
                    itObs->second);
            ASSERTMSG_(
                obsPc.pointcloud,
                "Only observation layers of classes derived from "
                "mrpt::maps::CPointsMap can be used to be inserted into the "
                "local map");

            MRPT_LOG_DEBUG_FMT(
                "UpdateLocalMap: Inserting observation layer '%s' into "
                "local map layer '%s'",
                lyObs.c_str(), lyMap.c_str());

            itLocalMap->second->insertObservation(
                obsPc, state_.last_lidar_pose.mean);
        }
        tle3.stop();

    }  // end done add a new KF to local map

    // Optional build simplemap:
    if (updateSimpleMap)
    {
        auto lck = mrpt::lockHelper(stateSimpleMap_mtx_);

        mrpt::obs::CSensoryFrame obsSF;
        obsSF.insert(o);

        std::optional<mrpt::math::TTwist3D> curTwist;
        if (hasMotionModel) curTwist = motionModelOutput->twist;

        state_.reconstructedMap.insert(
            // Pose: mean + covariance
            mrpt::poses::CPose3DPDFGaussian::Create(state_.last_lidar_pose),
            // SensoryFrame: set of observations (only one)
            mrpt::obs::CSensoryFrame::Create(obsSF),
            // twist
            curTwist);
    }

    // In any case, publish to the SLAM BackEnd what's our **current**
    // vehicle pose, no matter if it's a keyframe or not:
    if (slam_backend_)
    {
        ProfilerEntry tle(profiler_, "onLidar.5.advertiseUpdatedLocalization");

        BackEndBase::AdvertiseUpdatedLocalization_Input new_loc;
        new_loc.timestamp = this_obs_tim;
        // new_loc.reference_kf = state_.last_kf;
        new_loc.pose = state_.last_lidar_pose.mean.asTPose();

        std::future<void> adv_pose_fut =
            slam_backend_->advertiseUpdatedLocalization(new_loc);
    }

    // Optional real-time GUI via MOLA VizInterface:
    if (visualizer_ && state_.local_map)
    {
        ProfilerEntry tle(profiler_, "onLidar.6.updateVisualization");

        updateVisualization();
    }
}

void LidarInertialOdometry::run_one_icp(const ICP_Input& in, ICP_Output& out)
{
    using namespace std::string_literals;

    MRPT_START

    {
        ProfilerEntry tle(profiler_, "run_one_icp");

        ASSERT_(in.local_pc);
        ASSERT_(in.global_pc);
        const auto& pcs_local  = *in.local_pc;
        const auto& pcs_global = *in.global_pc;

        mrpt::math::TPose3D current_solution = in.init_guess_local_wrt_global;

        mp2p_icp::Results icp_result;

        params_.icp.at(in.align_kind)
            .icp->align(
                pcs_local, pcs_global, current_solution, in.icp_params,
                icp_result, in.prior);

        if (icp_result.quality > 0)
        {
            // Accept it:
            current_solution = icp_result.optimal_tf.mean.asTPose();
        }

        out.found_pose_to_wrt_from = icp_result.optimal_tf;
        out.goodness               = icp_result.quality;

        MRPT_LOG_DEBUG_FMT(
            "ICP (kind=%u): goodness=%.02f%% iters=%u pose=%s "
            "termReason=%s",
            static_cast<unsigned int>(in.align_kind), 100.0 * out.goodness,
            static_cast<unsigned int>(icp_result.nIterations),
            out.found_pose_to_wrt_from.getMeanVal().asString().c_str(),
            mrpt::typemeta::enum2str(icp_result.terminationReason).c_str());
    }

    MRPT_END
}

void LidarInertialOdometry::onIMU(const CObservation::Ptr& o)
{
    // All methods that are enqueued into a thread pool should have its own
    // top-level try-catch:
    try
    {
        onIMUImpl(o);
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
        state_.fatal_error = true;
    }

    {
        auto lck = mrpt::lockHelper(is_busy_mtx_);
        state_.worker_tasks--;
    }
}

void LidarInertialOdometry::onIMUImpl(const CObservation::Ptr& o)
{
    ASSERT_(o);

    ProfilerEntry tleg(profiler_, "onIMU");
}

void LidarInertialOdometry::onWheelOdometry(const CObservation::Ptr& o)
{
    // All methods that are enqueued into a thread pool should have its own
    // top-level try-catch:
    try
    {
        onWheelOdometryImpl(o);
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
        state_.fatal_error = true;
    }

    {
        auto lck = mrpt::lockHelper(is_busy_mtx_);
        state_.worker_tasks--;
    }
}

void LidarInertialOdometry::onWheelOdometryImpl(const CObservation::Ptr& o)
{
    ASSERT_(o);

    ProfilerEntry tleg(profiler_, "onWheelOdometry");

    auto odo = std::dynamic_pointer_cast<mrpt::obs::CObservationOdometry>(o);
    ASSERTMSG_(
        odo,
        mrpt::format(
            "Odometry observation with label '%s' does not have the expected "
            "type 'mrpt::obs::CObservationOdometry', it is '%s' instead",
            o->sensorLabel.c_str(), o->GetRuntimeClass()->className));

    // odo->timestamp;
}

bool LidarInertialOdometry::isBusy() const
{
    bool b;
    is_busy_mtx_.lock();
    b = state_.worker_tasks != 0;
    is_busy_mtx_.unlock();
    return b || worker_.pendingTasks();
}

mrpt::poses::CPose3DInterpolator LidarInertialOdometry::estimatedTrajectory()
    const
{
    auto lck = mrpt::lockHelper(stateTrajectory_mtx_);
    return state_.estimatedTrajectory;
}

mrpt::maps::CSimpleMap LidarInertialOdometry::reconstructedMap() const
{
    auto lck = mrpt::lockHelper(stateSimpleMap_mtx_);
    return state_.reconstructedMap;
}

// KISS-ICP adaptive threshold method (MIT License; code adapted to MRPT)
namespace
{
double computeModelError(
    const mrpt::poses::CPose3D& model_deviation, double max_range)
{
    const double theta =
        mrpt::poses::Lie::SO<3>::log(model_deviation.getRotationMatrix())
            .norm();
    const double delta_rot   = 2.0 * max_range * std::sin(theta / 2.0);
    const double delta_trans = model_deviation.translation().norm();
    return delta_trans + delta_rot;
}
}  // namespace

void LidarInertialOdometry::doUpdateAdaptiveThreshold(
    const mrpt::poses::CPose3D& lastMotionModelError)
{
    if (!state_.estimated_sensor_max_range.has_value()) return;

    const double max_range = state_.estimated_sensor_max_range.value();

    const double ALPHA = 0.99;

    double model_error = computeModelError(lastMotionModelError, max_range);

    if (state_.adapt_thres_sigma == 0)  // initial
        state_.adapt_thres_sigma = params_.adaptive_threshold.initial_sigma;

    state_.adapt_thres_sigma =
        ALPHA * state_.adapt_thres_sigma + (1.0 - ALPHA) * model_error;

    mrpt::keep_max(
        state_.adapt_thres_sigma, params_.adaptive_threshold.min_motion);
}

void LidarInertialOdometry::doInitializeEstimatedMaxSensorRange(
    const mrpt::obs::CObservation& o)
{
    auto& maxRange = state_.estimated_sensor_max_range;
    ASSERT_(!maxRange.has_value());  // this method is for 1st call only

    mrpt::maps::CSimplePointsMap pts;
    pts.insertObservation(o);

    if (pts.empty()) return;

    const auto   bb     = pts.boundingBox();
    const double radius = 0.5 * (bb.max - bb.min).norm();

    maxRange = radius;

    MRPT_LOG_DEBUG_STREAM(
        "Estimated sensor max range=" << *state_.estimated_sensor_max_range
                                      << " (instantaneous=" << radius << ")");
}

void LidarInertialOdometry::doUpdateEstimatedMaxSensorRange(
    const mp2p_icp::metric_map_t& m)
{
    const double ALPHA = params_.max_sensor_range_filter_coefficient;

    auto& maxRange = state_.estimated_sensor_max_range;
    ASSERT_(maxRange.has_value());  // this method is for subsequent calls only

    for (const auto& [layerName, layer] : m.layers)
    {
        auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(layer);
        if (!pts) continue;

        const auto bb     = pts->boundingBox();
        double     radius = 0.5 * (bb.max - bb.min).norm();

        mrpt::keep_max(radius, params_.absolute_minimum_sensor_range);

        // low-pass filter update:
        maxRange = maxRange.value() * ALPHA + radius * (1.0 - ALPHA);

        MRPT_LOG_DEBUG_STREAM(
            "Estimated sensor max range=" << *state_.estimated_sensor_max_range
                                          << " (instantaneous=" << radius
                                          << ")");

        // one layer is enough:
        return;
    }
    MRPT_LOG_DEBUG(
        "Estimated sensor max range could NOT be updated, no points layer "
        "found in observation metric_map_t");
}

void LidarInertialOdometry::updatePipelineDynamicVariables()
{
    // Set dynamic variables for twist usage within ICP pipelines
    // (e.g. de-skew methods)
    {
        mrpt::math::TTwist3D twistForIcpVars = {0, 0, 0, 0, 0, 0};
        if (state_.navstate_fuse.get_last_twist())
            twistForIcpVars = state_.navstate_fuse.get_last_twist().value();

        state_.icpParameterSource.updateVariable("VX", twistForIcpVars.vx);
        state_.icpParameterSource.updateVariable("VY", twistForIcpVars.vy);
        state_.icpParameterSource.updateVariable("VZ", twistForIcpVars.vz);
        state_.icpParameterSource.updateVariable("WX", twistForIcpVars.wx);
        state_.icpParameterSource.updateVariable("WY", twistForIcpVars.wy);
        state_.icpParameterSource.updateVariable("WZ", twistForIcpVars.wz);
    }

    state_.icpParameterSource.updateVariable(
        "ADAPTIVE_THRESHOLD_SIGMA",
        state_.adapt_thres_sigma != 0
            ? state_.adapt_thres_sigma
            : params_.adaptive_threshold.initial_sigma);

    if (state_.estimated_sensor_max_range)
    {
        state_.icpParameterSource.updateVariable(
            "ESTIMATED_SENSOR_MAX_RANGE", *state_.estimated_sensor_max_range);
    }

    // Make all changes effective and evaluate the variables now:
    state_.icpParameterSource.realize();
}

void LidarInertialOdometry::updateVisualization()
{
    // In this point, we are called by the LIDAR worker thread, so it's safe
    // to read the state without mutexes.
    ASSERT_(visualizer_);

    // Vehicle pose:
    // ---------------------------
    if (!state_.glVehicleFrame)
    {
        state_.glVehicleFrame = mrpt::opengl::CSetOfObjects::Create();
        if (const auto l = params_.visualization.current_pose_corner_size;
            l > 0)
        {
            auto glCorner = mrpt::opengl::stock_objects::CornerXYZ(l);
            state_.glVehicleFrame->insert(glCorner);
        }

        // 3D model:
        if (!params_.visualization.model_file.empty())
        {
            const auto& _ = params_.visualization;

            const auto localFileName = _.model_file;

            auto m = mrpt::opengl::CAssimpModel::Create();

            ASSERT_FILE_EXISTS_(localFileName);

            int loadFlags =
                mrpt::opengl::CAssimpModel::LoadFlags::RealTimeMaxQuality |
                mrpt::opengl::CAssimpModel::LoadFlags::FlipUVs;

            m->loadScene(localFileName, loadFlags);

            m->setScale(_.model_scale);
            m->setPose(_.model_tf);

            state_.glVehicleFrame->insert(m);
        }
    }
    state_.glVehicleFrame->setPose(state_.last_lidar_pose.mean);
    visualizer_->update_3d_object("liodom/vehicle", state_.glVehicleFrame);

    // Estimated path:
    // ------------------------
    if (params_.visualization.show_trajectory)
    {
        if (!state_.glEstimatedPath)
        {
            state_.glEstimatedPath = mrpt::opengl::CSetOfLines::Create();
            state_.glEstimatedPath->setColor_u8(0x30, 0x30, 0x30);

            state_.glPathGrp = mrpt::opengl::CSetOfObjects::Create();
        }
        // Update path viz:
        for (size_t i = state_.glEstimatedPath->size();
             i < state_.estimatedTrajectory.size(); i++)
        {
            auto it = state_.estimatedTrajectory.begin();
            std::advance(it, i);

            const auto t = it->second.translation();

            if (state_.glEstimatedPath->empty())
                state_.glEstimatedPath->appendLine(t, t);
            else
                state_.glEstimatedPath->appendLineStrip(t);
        }
        state_.glPathGrp->clear();
        state_.glPathGrp->insert(
            mrpt::opengl::CSetOfLines::Create(*state_.glEstimatedPath));

        visualizer_->update_3d_object("liodom/path", state_.glPathGrp);
    }

    // GUI follow vehicle:
    // ---------------------------
    visualizer_->update_viewport_look_at(
        state_.last_lidar_pose.mean.translation());

    // Local map:
    // -----------------------------
    if (!state_.glLocalMap)  //
        state_.glLocalMap = mrpt::opengl::CSetOfObjects::Create();

    if (state_.mapUpdateCnt++ > params_.visualization.map_update_decimation)
    {
        state_.mapUpdateCnt = 0;
        state_.glLocalMap->clear();
        auto glMap = state_.local_map->get_visualization();
        for (auto& o : *glMap) state_.glLocalMap->insert(o);

        visualizer_->update_3d_object("liodom/localmap", state_.glLocalMap);
    }

    // Console messages:
    // -------------------------
    {
        std::stringstream ss;
        if (state_.last_obs_tim)
            ss << mrpt::format(
                "t=%.03f ", mrpt::Clock::toDouble(state_.last_obs_tim.value()));

        ss << mrpt::format(
            "pose:%65s ", state_.last_lidar_pose.mean.asString().c_str());

        if (state_.navstate_fuse.get_last_twist())
            ss << mrpt::format(
                "twist:%65s ", state_.navstate_fuse.get_last_twist()
                                   .value()
                                   .asString()
                                   .c_str());

        ss << mrpt::format(
            "path KFs: %4zu ", state_.estimatedTrajectory.size());

        ss << mrpt::format(
            "simplemap KFs: %4zu ", state_.reconstructedMap.size());

        visualizer_->output_console_message(ss.str());
    }

    // Sub-window with custom UI
    // -------------------------------------
    if (!state_.ui)
    {
        auto fut  = visualizer_->create_subwindow("mola_lidar_odometry");
        state_.ui = fut.get();
        state_.ui->requestFocus();
        state_.ui->setVisible(
            !params_.visualization.gui_subwindow_starts_hidden);
        state_.ui->setPosition({5, 700});

        state_.ui->setLayout(new nanogui::BoxLayout(
            nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 5, 2));
        state_.ui->setFixedWidth(300);

        state_.lbIcpQuality  = state_.ui->add<nanogui::Label>(" ");
        state_.lbSigma       = state_.ui->add<nanogui::Label>(" ");
        state_.lbSensorRange = state_.ui->add<nanogui::Label>(" ");
        state_.lbTime        = state_.ui->add<nanogui::Label>(" ");
        state_.lbPeriod      = state_.ui->add<nanogui::Label>(" ");
    }
    state_.lbIcpQuality->setCaption(
        mrpt::format("ICP quality: %.01f%%", 100.0 * state_.last_icp_quality));
    state_.lbSigma->setCaption(
        mrpt::format("Threshold sigma: %.02f", state_.adapt_thres_sigma));
    if (state_.estimated_sensor_max_range)
        state_.lbSensorRange->setCaption(mrpt::format(
            "Est. max range: %.02f m", *state_.estimated_sensor_max_range));
    {
        const double dt    = profiler_.getLastTime("onLidar");
        const double dtAvr = profiler_.getMeanTime("onLidar");
        state_.lbTime->setCaption(mrpt::format(
            "Process time: %6.02f ms (avr: %6.02f ms)", 1e3 * dt, 1e3 * dtAvr));
        if (dt > 0 && dtAvr > 0)
        {
            state_.lbPeriod->setCaption(mrpt::format(
                "Process rate: %6.02f Hz (avr: %6.02f Hz)", 1.0 / dt,
                1.0 / dtAvr));
        }
    }
}

std::tuple<bool /*isFirst*/, mrpt::poses::CPose3D /*distanceToClosest*/>
    LidarInertialOdometry::SearchablePoseList::check(
        const mrpt::poses::CPose3D& p) const
{
    const bool           isFirst = empty();
    mrpt::poses::CPose3D distanceToClosest;
    if (isFirst) return {isFirst, distanceToClosest};

    if (from_last_only_)
    {  //
        distanceToClosest = p - last_kf_;
    }
    else
    {
        std::vector<mrpt::math::TPoint3Df> closest;
        std::vector<float>                 closestSqrDist;
        std::vector<uint64_t>              closestID;

        kf_points_.nn_multiple_search(
            p.translation().cast<float>(), 20, closest, closestSqrDist,
            closestID);
        ASSERT_(!closest.empty());  // empty()==false from check above

        // Check for both, rotation and translation.
        // Use a heuristic SE(3) metric to merge both parts:
        constexpr double ROTATION_WEIGHT = 1.0;

        std::optional<size_t> bestIdx;

        for (size_t i = 0; i < closest.size(); i++)
        {
            const auto&  candidate = kf_poses_.at(closestID.at(i));
            const double rot       = mrpt::poses::Lie::SO<3>::log(
                                   (p - candidate).getRotationMatrix())
                                   .norm();

            closestSqrDist[i] += ROTATION_WEIGHT * mrpt::square(rot);

            if (!bestIdx || closestSqrDist[i] < closestSqrDist[*bestIdx])
                bestIdx = i;
        }

        const auto& closestPose = kf_poses_.at(closestID.at(*bestIdx));

        distanceToClosest = p - closestPose;
    }

    return {isFirst, distanceToClosest};
}
