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
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>

using namespace mola;

// static const std::string ANNOTATION_NAME_PC_LAYERS =
// "lidar-pointcloud-layers";

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(LidarInertialOdometry, FrontEndBase, mola)

LidarInertialOdometry::LidarInertialOdometry() = default;

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

    YAML_LOAD_REQ(params_, min_dist_xyz_between_keyframes, double);
    YAML_LOAD_OPT_DEG(params_, min_rotation_between_keyframes, double);

    YAML_LOAD_OPT(params_, min_time_between_scans, double);
    YAML_LOAD_OPT(params_, max_time_to_use_velocity_model, double);
    YAML_LOAD_OPT(params_, min_icp_goodness, double);

    params_.adaptive_threshold.initialize(cfg["adaptive_threshold"]);

    YAML_LOAD_OPT(params_, icp_profiler_enabled, bool);
    YAML_LOAD_OPT(params_, icp_profiler_full_history, bool);

    YAML_LOAD_OPT(params_, simplemap.generate, bool);
    YAML_LOAD_OPT(params_, simplemap.min_dist_xyz_between_keyframes, double);
    YAML_LOAD_OPT_DEG(
        params_, simplemap.min_rotation_between_keyframes, double);

    ENSURE_YAML_ENTRY_EXISTS(cfg, "icp_settings_with_vel");
    load_icp_set_of_params(
        params_.icp[AlignKind::RegularOdometry], cfg["icp_settings_with_vel"]);

    ENSURE_YAML_ENTRY_EXISTS(cfg, "icp_settings_without_vel");
    load_icp_set_of_params(
        params_.icp[AlignKind::NoMotionModel], cfg["icp_settings_without_vel"]);

    for (auto& [kind, icpCase] : params_.icp)
    {
        icpCase.icp->profiler().enable(params_.icp_profiler_enabled);
        icpCase.icp->profiler().enableKeepWholeHistory(
            params_.icp_profiler_full_history);

        // Attach all ICP instances to the parameter source for dynamic
        // parameters:
        icpCase.icp->attachToParameterSource(state_.icpParameterSource);
    }

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

void LidarInertialOdometry::reset() { state_ = MethodState(); }

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

#if 0
    MRPT_LOG_DEBUG_STREAM(
        "onNewObservation: class=" << o->GetRuntimeClass()->className
                                   << " sensorLabel=" << o->sensorLabel);
#endif

    // Is it an IMU obs?
    if (params_.imu_sensor_label &&
        std::regex_match(o->sensorLabel, params_.imu_sensor_label.value()))
    {
        {
            auto lck        = mrpt::lockHelper(is_busy_mtx_);
            state_.busy_imu = true;
        }

        // Yes, it's an IMU obs:
        auto fut = worker_imu_.enqueue(&LidarInertialOdometry::onIMU, this, o);
        (void)fut;
    }

    // Is it a LIDAR obs?
    for (const auto& re : params_.lidar_sensor_labels)
    {
        if (!std::regex_match(o->sensorLabel, re)) continue;

        // Yes, it's a LIDAR obs:
        const auto queued = worker_lidar_.pendingTasks();
        profiler_.registerUserMeasure("onNewObservation.queue_length", queued);
        if (queued > 10)
        {
            MRPT_LOG_THROTTLE_ERROR(
                1.0, "Dropping observation due to worker threads too busy.");
            profiler_.registerUserMeasure(
                "onNewObservation.drop_observation", 1);
            return;
        }
        profiler_.enter("delay_onNewObs_to_process");

        {
            auto lck          = mrpt::lockHelper(is_busy_mtx_);
            state_.busy_lidar = true;
        }

        // Enqueue task:
        auto fut =
            worker_lidar_.enqueue(&LidarInertialOdometry::onLidar, this, o);

        (void)fut;
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
        auto lck          = mrpt::lockHelper(is_busy_mtx_);
        state_.busy_lidar = false;
    }
}

// here happens the main stuff:
void LidarInertialOdometry::onLidarImpl(const CObservation::Ptr& o)
{
    using namespace std::string_literals;

    ASSERT_(o);

    ProfilerEntry tleg(profiler_, "onLidar");
    profiler_.leave("delay_onNewObs_to_process");

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

    updatePipelineDynamicVariables();

    MRPT_LOG_DEBUG_STREAM(
        "Dynamic variables: "
        << state_.icpParameterSource.printVariableValues());

    // Extract points from observation:
    auto observation = mp2p_icp::metric_map_t::Create();

    ProfilerEntry tle0(profiler_, "onLidar.0.apply_generators");

    mp2p_icp_filters::apply_generators(state_.obs_generators, *o, *observation);

    tle0.stop();

    // Use the observation to update the estimated sensor range:
    doUpdateEstimatedMaxSensorRange(*observation);

    // Filter/segment the point cloud (optional, but normally will be
    // present):
    ProfilerEntry tle1(profiler_, "onLidar.1.filter_pointclouds");

    mp2p_icp_filters::apply_filter_pipeline(state_.pc_filter, *observation);

    tle1.stop();

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
                this_obs_tim, state_.current_pose.mean);
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
        bool hasMotionModel                = false;

        // Use velocity model, if we have one:
        if (state_.last_iter_twist &&
            dt < params_.max_time_to_use_velocity_model)
        {
            const auto& tw = state_.last_iter_twist.value();

            // For the velocity model, we don't have any known "bias":
            const mola::RotationIntegrationParams rotParams = {};

            const auto rot33 = mola::incremental_rotation(
                {tw.wx, tw.wy, tw.wz}, rotParams, dt);

            icp_in.init_guess_local_wrt_global =
                (state_.current_pose.mean +
                 mrpt::poses::CPose3D::FromRotationAndTranslation(
                     rot33, mrpt::math::TVector3D(tw.vx, tw.vy, tw.vz) * dt))
                    .asTPose();

            hasMotionModel = true;
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
        state_.current_pose = icp_out.found_pose_to_wrt_from;

        // Update velocity model:
        mrpt::poses::CPose3D incrPose;
        if (dt < params_.max_time_to_use_velocity_model && state_.last_pose)
        {
            ASSERT_GT_(dt, .0);

            state_.last_iter_twist.emplace();
            auto& tw = *state_.last_iter_twist;

            incrPose = state_.current_pose.mean - *state_.last_pose;

            tw.vx = incrPose.x() / dt;
            tw.vy = incrPose.y() / dt;
            tw.vz = incrPose.z() / dt;

            const auto logRot =
                mrpt::poses::Lie::SO<3>::log(incrPose.getRotationMatrix());

            tw.wx = logRot[0] / dt;
            tw.wy = logRot[1] / dt;
            tw.wz = logRot[2] / dt;
        }
        else
        {
            state_.last_iter_twist.reset();
        }

        // Update trajectory too:
        {
            auto lck = mrpt::lockHelper(stateTrajectory_mtx_);
            state_.estimatedTrajectory.insert(
                this_obs_tim, state_.current_pose.mean);
        }

        MRPT_LOG_DEBUG_STREAM(
            "Est.twist=" << (state_.last_iter_twist
                                 ? state_.last_iter_twist->asString()
                                 : "(none)"s)
                         << " dt=" << dt << " s.");
        MRPT_LOG_DEBUG_STREAM(
            "Time since last scan=" << mrpt::system::formatTimeInterval(dt));

        // KISS-ICP adaptive threshold method:
        if (params_.adaptive_threshold.enabled)
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

        // Create a new KF if the distance since the last one is large
        // enough:
        state_.accum_since_last_kf += incrPose;
        state_.accum_since_last_simplemap_kf += incrPose;

        const double dist_eucl_since_last = state_.accum_since_last_kf.norm();
        const double rot_since_last =
            mrpt::poses::Lie::SO<3>::log(
                state_.accum_since_last_kf.getRotationMatrix())
                .norm();

        updateLocalMap =
            (icp_out.goodness > params_.min_icp_goodness &&
             hasMotionModel &&  // skip map update for the special ICP alignment
                                // without motion model
             (dist_eucl_since_last > params_.min_dist_xyz_between_keyframes ||
              rot_since_last > params_.min_rotation_between_keyframes));

        if (updateLocalMap)
            state_.accum_since_last_kf = mrpt::poses::CPose3D::Identity();

        const double dist_eucl_since_last_sm =
            state_.accum_since_last_simplemap_kf.norm();
        const double rot_since_last_sm =
            mrpt::poses::Lie::SO<3>::log(
                state_.accum_since_last_simplemap_kf.getRotationMatrix())
                .norm();

        updateSimpleMap =
            (icp_out.goodness > params_.min_icp_goodness &&
             (dist_eucl_since_last_sm >
                  params_.simplemap.min_dist_xyz_between_keyframes ||
              rot_since_last_sm >
                  params_.simplemap.min_rotation_between_keyframes));

        if (updateSimpleMap)
            state_.accum_since_last_simplemap_kf =
                mrpt::poses::CPose3D::Identity();

        MRPT_LOG_DEBUG_FMT(
            "Since last KF: dist=%5.03f m rotation=%.01f deg updateLocalMap=%s "
            "updateSimpleMap=%s",
            dist_eucl_since_last, mrpt::RAD2DEG(rot_since_last),
            updateLocalMap ? "YES" : "NO", updateSimpleMap ? "YES" : "NO");

    }  // end: yes, we can do ICP

    // save for next iter:
    state_.last_pose = state_.current_pose.mean;

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
                obsPc, state_.current_pose.mean);
        }
        tle3.stop();

    }  // end done add a new KF to local map

    // Optional build simplemap:
    if (updateSimpleMap)
    {
        auto lck = mrpt::lockHelper(stateSimpleMap_mtx_);

        mrpt::obs::CSensoryFrame obsSF;
        obsSF.insert(o);

        state_.reconstructedMap.insert(
            // Pose: mean + covariance
            mrpt::poses::CPose3DPDFGaussian::Create(state_.current_pose),
            // SensoryFrame: set of observations (only one)
            mrpt::obs::CSensoryFrame::Create(obsSF),
            // twist
            state_.last_iter_twist);
    }

    // In any case, publish to the SLAM BackEnd what's our **current**
    // vehicle pose, no matter if it's a keyframe or not:
    if (slam_backend_)
    {
        ProfilerEntry tle(profiler_, "onLidar.5.advertiseUpdatedLocalization");

        BackEndBase::AdvertiseUpdatedLocalization_Input new_loc;
        new_loc.timestamp = this_obs_tim;
        // new_loc.reference_kf = state_.last_kf;
        new_loc.pose = state_.current_pose.mean.asTPose();

        std::future<void> adv_pose_fut =
            slam_backend_->advertiseUpdatedLocalization(new_loc);
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
                icp_result);

        if (icp_result.quality > 0)
        {
            // Keep as init value for next stage:
            current_solution = icp_result.optimal_tf.mean.asTPose();
        }

        out.found_pose_to_wrt_from = icp_result.optimal_tf;
        out.goodness               = icp_result.quality;

        MRPT_LOG_DEBUG_FMT(
            "ICP (kind=%u): goodness=%.03f iters=%u rel_pose=%s "
            "termReason=%u",
            static_cast<unsigned int>(in.align_kind), out.goodness,
            static_cast<unsigned int>(icp_result.nIterations),
            out.found_pose_to_wrt_from.getMeanVal().asString().c_str(),
            static_cast<unsigned int>(icp_result.terminationReason));
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
        auto lck        = mrpt::lockHelper(is_busy_mtx_);
        state_.busy_imu = false;
    }
}

void LidarInertialOdometry::onIMUImpl(const CObservation::Ptr& o)
{
    ASSERT_(o);

    ProfilerEntry tleg(profiler_, "onIMU");
}

bool LidarInertialOdometry::isBusy() const
{
    bool b1, b2;
    is_busy_mtx_.lock();
    b1 = state_.busy_imu;
    b2 = state_.busy_lidar;
    is_busy_mtx_.unlock();
    return b1 || b2 || worker_lidar_.pendingTasks() ||
           worker_imu_.pendingTasks();
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
    const double max_range = state_.estimated_sensor_max_range;

    double model_error = computeModelError(lastMotionModelError, max_range);

    if (model_error > params_.adaptive_threshold.min_motion)
    {
        state_.adapt_thres_sse2 += mrpt::square(model_error);
        state_.adapt_thres_num_samples++;
    }

    if (state_.adapt_thres_num_samples < 1)
    {
        state_.adapt_thres_sigma = params_.adaptive_threshold.initial_sigma;
    }
    else
    {
        state_.adapt_thres_sigma =
            std::sqrt(state_.adapt_thres_sse2 / state_.adapt_thres_num_samples);
    }
}

void LidarInertialOdometry::doUpdateEstimatedMaxSensorRange(
    const mp2p_icp::metric_map_t& localFrame)
{
    constexpr double ALPHA = 0.9;

    // Use the first non-empty point cloud layer to estimate maximum sensor
    // range:
    for (const auto& [name, map] : localFrame.layers)
    {
        const auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(map);
        if (!pts) continue;
        if (pts->empty()) continue;

        const auto   bb     = pts->boundingBox();
        const double radius = 0.5 * (bb.max - bb.min).norm();

        state_.estimated_sensor_max_range =
            state_.estimated_sensor_max_range * ALPHA + radius * (1.0 - ALPHA);

        MRPT_LOG_DEBUG_STREAM(
            "Estimated sensor max range=" << state_.estimated_sensor_max_range
                                          << " (instantaneous=" << radius
                                          << ")");
        // With one layer is enough:
        return;
    }
    // No way to automatically determine sensor range, do not touch it
}

void LidarInertialOdometry::updatePipelineDynamicVariables()
{
    // Set dynamic variables for twist usage within ICP pipelines
    // (e.g. de-skew methods)
    {
        mrpt::math::TTwist3D twistForIcpVars = {0, 0, 0, 0, 0, 0};
        if (state_.last_iter_twist)
            twistForIcpVars = state_.last_iter_twist.value();

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

    state_.icpParameterSource.updateVariable(
        "ESTIMATED_SENSOR_MAX_RANGE", state_.estimated_sensor_max_range);

    // Make all changes effective and evaluate the variables now:
    state_.icpParameterSource.realize();
}
