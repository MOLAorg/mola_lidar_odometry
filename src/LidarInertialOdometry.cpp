/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   LidarInertialOdometry.cpp
 * @brief  Header for main C++ class exposing LIDAR-inertial odometry
 * @author Jose Luis Blanco Claraco
 * @date   Sep 16, 2023
 */

#include <mola_lidar_slam/LidarInertialOdometry.h>
#include <mola_yaml/yaml_helpers.h>
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

static void load_icp_set_of_params(
    LidarInertialOdometry::Parameters::ICP_case& out,
    const mrpt::containers::yaml&                cfg)
{
    const auto [icp, params] = mp2p_icp::icp_pipeline_from_yaml(cfg);

    out.icp           = icp;
    out.icpParameters = params;
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

    if (cfg["observation_layers_to_merge_local_map"].isSequence())
    {
        for (const auto& sl :
             cfg["observation_layers_to_merge_local_map"].asSequence())
        {
            const auto s = sl.as<std::string>();
            MRPT_LOG_DEBUG_STREAM(
                "Adding as observation_layers_to_merge_local_map: " << s);
            params_.observation_layers_to_merge_local_map.emplace_back(s);
        }
    }
    else
    {
        ASSERT_(cfg["observation_layers_to_merge_local_map"].isScalar());
        const auto s =
            cfg["observation_layers_to_merge_local_map"].as<std::string>();
        MRPT_LOG_DEBUG_STREAM(
            "Adding as observation_layers_to_merge_local_map: " << s);
        params_.observation_layers_to_merge_local_map.emplace_back(s);
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
    YAML_LOAD_OPT(params_, min_icp_goodness_lc, double);

    YAML_LOAD_OPT(params_, min_dist_to_matching, double);
    YAML_LOAD_OPT(params_, max_dist_to_matching, double);
    YAML_LOAD_OPT(params_, max_dist_to_loop_closure, double);
    YAML_LOAD_OPT(params_, max_nearby_align_checks, unsigned int);
    YAML_LOAD_OPT(params_, min_topo_dist_to_consider_loopclosure, unsigned int);
    YAML_LOAD_OPT(params_, loop_closure_montecarlo_samples, unsigned int);

    YAML_LOAD_OPT(params_, viz_decor_decimation, int);
    YAML_LOAD_OPT(params_, viz_decor_pointsize, float);

    ENSURE_YAML_ENTRY_EXISTS(cfg, "icp_settings_with_vel");
    load_icp_set_of_params(
        params_.icp[AlignKind::LidarOdometry], cfg["icp_settings_with_vel"]);

    ENSURE_YAML_ENTRY_EXISTS(cfg, "icp_settings_without_vel");
    load_icp_set_of_params(
        params_.icp[AlignKind::NearbyAlign], cfg["icp_settings_without_vel"]);

    // Create lidar segmentation algorithm:
    {
        ProfilerEntry tle(profiler_, "filterPointCloud_initialize");

        if (cfg.has("pointcloud_generator"))
        {
            // Create, and copy my own verbosity level:
            state_.pc_generators = mp2p_icp_filters::generators_from_yaml(
                cfg["pointcloud_generator"], this->getMinLoggingLevel());
        }
        else
        {
            std::cout
                << "[warning] Using default mp2p_icp_filters::Generator "
                   "since no YAML 'pointcloud_generator' entry was given\n";

            auto defaultGen = mp2p_icp_filters::Generator::Create();
            defaultGen->initialize({});
            state_.pc_generators.push_back(defaultGen);
        }

        if (cfg.has("pointcloud_filter"))
        {
            // Create, and copy my own verbosity level:
            state_.pc_filter = mp2p_icp_filters::filter_pipeline_from_yaml(
                cfg["pointcloud_filter"], this->getMinLoggingLevel());
        }
    }

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

    MRPT_LOG_DEBUG_STREAM(
        "onNewObservation: class=" << o->GetRuntimeClass()->className
                                   << " sensorLabel=" << o->sensorLabel);

    // Is it an IMU obs?
    if (params_.imu_sensor_label &&
        std::regex_match(o->sensorLabel, params_.imu_sensor_label.value()))
    {
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
    {
        auto lck          = mrpt::lockHelper(is_busy_mtx_);
        state_.busy_lidar = true;
    }
    try
    {
        onLidarImpl(o);
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
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

    // Extract points from observation:
    auto this_obs_points = mp2p_icp::metric_map_t::Create();
    mp2p_icp_filters::apply_generators(
        state_.pc_generators, *o, *this_obs_points);

    // Filter/segment the point cloud (optional, but normally will be present):
    ProfilerEntry tle1(profiler_, "onLidar.1.filter_pointclouds");

    mp2p_icp_filters::apply_filter_pipeline(state_.pc_filter, *this_obs_points);

    tle1.stop();

    profiler_.enter("onLidar.2.copy_vars");

    // Store for next step:
    const auto last_obs_tim = state_.last_obs_tim;
    state_.last_obs_tim     = this_obs_tim;

    profiler_.leave("onLidar.2.copy_vars");

    if (this_obs_points->empty())
    {
        MRPT_LOG_WARN_STREAM(
            "Observation of type `" << o->GetRuntimeClass()->className
                                    << "` could not be converted into a "
                                       "pointcloud. Doing nothing.");
        return;
    }

    bool updateLocalMap = false;

    // First time we cannot do ICP since we need at least two pointclouds:
    ASSERT_(state_.local_map);

    if (state_.local_map->empty())
    {
        // Skip ICP.
        MRPT_LOG_DEBUG(
            "First pointcloud: skipping ICP and directly adding to local map.");

        // Create a first KF (at origin)
        updateLocalMap = true;
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

        icp_in.init_guess_to_wrt_from = mrpt::math::TPose3D::Identity();
        bool hasMotionModel           = false;

        // Use velocity model, if we have one:
        if (state_.last_iter_twist &&
            dt < params_.max_time_to_use_velocity_model)
        {
            const auto& tw = state_.last_iter_twist.value();

            // For the velocity model, we don't any known "bias":
            const mola::RotationIntegrationParams rotParams = {};

            const auto rot33 = mola::incremental_rotation(
                {tw.wx, tw.wy, tw.wz}, rotParams, dt);

            icp_in.init_guess_to_wrt_from =
                (state_.current_pose +
                 mrpt::poses::CPose3D::FromRotationAndTranslation(
                     rot33, mrpt::math::TVector3D(tw.vx, tw.vy, tw.vz)))
                    .asTPose();

            hasMotionModel = true;
        }

        icp_in.to_pc     = this_obs_points;
        icp_in.from_pc   = state_.local_map;
        icp_in.debug_str = "lidar_odom";

        // If we don't have a valid twist estimation, use a larger ICP
        // correspondence threshold:
        icp_in.icp_params =
            hasMotionModel ? params_.icp[AlignKind::LidarOdometry].icpParameters
                           : params_.icp[AlignKind::NearbyAlign].icpParameters;

        profiler_.leave("onLidar.2c.prepare_icp_in");

        // Run ICP:
        {
            ProfilerEntry tle(profiler_, "onLidar.3.icp_latest");
            run_one_icp(icp_in, icp_out);
        }
        state_.current_pose = icp_out.found_pose_to_wrt_from.getMeanVal();

        // Update velocity model:
        if (dt < params_.max_time_to_use_velocity_model)
        {
            ASSERT_GT_(dt, .0);

            state_.last_iter_twist.emplace();
            auto& tw = *state_.last_iter_twist;

            tw.vx = state_.current_pose.x() / dt;
            tw.vy = state_.current_pose.y() / dt;
            tw.vz = state_.current_pose.z() / dt;

            const auto logRot = mrpt::poses::Lie::SO<3>::log(
                state_.current_pose.getRotationMatrix());

            tw.wx = logRot[0] / dt;
            tw.wy = logRot[1] / dt;
            tw.wz = logRot[2] / dt;
        }
        else
        {
            state_.last_iter_twist.reset();
        }

        MRPT_LOG_DEBUG_STREAM(
            "Est.twist="
            << (state_.last_iter_twist ? state_.last_iter_twist->asString()
                                       : "(none)"s));
        MRPT_LOG_DEBUG_STREAM(
            "Time since last scan=" << mrpt::system::formatTimeInterval(dt));

        // Create a new KF if the distance since the last one is large
        // enough:
        MRPT_TODO("continue here!");
        state_.accum_since_last_kf =
            state_.accum_since_last_kf + state_.current_pose;

        const double dist_eucl_since_last = state_.accum_since_last_kf.norm();
        const double rot_since_last =
            mrpt::poses::Lie::SO<3>::log(
                state_.accum_since_last_kf.getRotationMatrix())
                .norm();

        MRPT_LOG_DEBUG_FMT(
            "Since last KF: dist=%5.03f m rotation=%.01f deg",
            dist_eucl_since_last, mrpt::RAD2DEG(rot_since_last));

        updateLocalMap =
            (icp_out.goodness > params_.min_icp_goodness &&
             (dist_eucl_since_last > params_.min_dist_xyz_between_keyframes ||
              rot_since_last > params_.min_rotation_between_keyframes));

    }  // end: yes, we can do ICP

    // Should we create a new KF?
    if (updateLocalMap)
    {
        std::cout << "Observation: " << this_obs_points->contents_summary()
                  << std::endl;

        // Merge "observation_layers_to_merge_local_map" in local map:
        mp2p_icp::metric_map_t tmpMap;
        for (const auto& layer : params_.observation_layers_to_merge_local_map)
        {
            ASSERT_(this_obs_points->layers.count(layer) != 0);

            tmpMap.layers[layer] = this_obs_points->layers.at(layer);
        }

        std::cout << "Local map before: "
                  << state_.local_map->contents_summary() << std::endl;

        state_.local_map->merge_with(tmpMap, state_.current_pose.asTPose());

        std::cout << "Local map after : "
                  << state_.local_map->contents_summary() << std::endl;

#if 0
        // Yes: create new KF
        // 1) New KeyFrame
        BackEndBase::ProposeKF_Input kf;

        kf.timestamp = this_obs_tim;
        // Build simplemap
        MRPT_TODO("re-enable this");
        if (0)
        {
            mrpt::obs::CSensoryFrame& sf = kf.observations.emplace();
            sf.push_back(o);
        }

        profiler_.enter("onLidar.3a.addKeyFrame");

        std::future<BackEndBase::ProposeKF_Output> kf_out_fut;
        kf_out_fut = slam_backend_->addKeyFrame(kf);

        // Wait until it's executed:
        auto kf_out = kf_out_fut.get();

        ASSERT_(kf_out.success);
        ASSERT_(kf_out.new_kf_id);

        const mola::id_t new_kf_id = kf_out.new_kf_id.value();
        ASSERT_(new_kf_id != mola::INVALID_ID);

        profiler_.leave("onLidar.3a.addKeyFrame");

        // Add point cloud to the KF annotations in the map:
        // Also, add Rendering decorations for the map visualizer:
        ASSERT_(worldmodel_);
        {
            profiler_.enter("onLidar.wait.ent.writelock");
            worldmodel_->entities_lock_for_write();
            profiler_.leave("onLidar.wait.ent.writelock");

            ProfilerEntry tle(profiler_, "onLidar.4.writePCsToWorldModel");

            worldmodel_->entity_annotations_by_id(new_kf_id).emplace(
                std::piecewise_construct,
                std::forward_as_tuple(ANNOTATION_NAME_PC_LAYERS),
                std::forward_as_tuple(
                    this_obs_points, ANNOTATION_NAME_PC_LAYERS));

            MRPT_TODO("move this render stuff to a low-prio worker thread?");
            if (state_.kf_decor_decim_cnt < 0 ||
                ++state_.kf_decor_decim_cnt > params_.viz_decor_decimation)
            {
                auto obs_render = mrpt::opengl::CSetOfObjects::Create();

                if (auto obs_pc =
                        dynamic_cast<const mrpt::obs::CObservationPointCloud*>(
                            o.get());
                    obs_pc != nullptr && obs_pc->pointcloud)
                {
                    state_.kf_decor_decim_cnt = 0;

                    obs_pc->pointcloud->renderOptions.point_size =
                        params_.viz_decor_pointsize;
                    obs_pc->pointcloud->getVisualizationInto(*obs_render);
                }
                else
                {
                    state_.kf_decor_decim_cnt = 0;

                    mrpt::maps::CColouredPointsMap pm;
                    pm.renderOptions.point_size = params_.viz_decor_pointsize;

                    if (pm.insertObservationPtr(o))
                        pm.getVisualizationInto(*obs_render);
                }

                if (obs_render)
                    worldmodel_->entity_annotations_by_id(new_kf_id).emplace(
                        std::piecewise_construct,
                        std::forward_as_tuple("render_decoration"),
                        std::forward_as_tuple(obs_render, "render_decoration"));
            }

            worldmodel_->entities_unlock_for_write();
        }
        MRPT_LOG_INFO_STREAM("New KF: ID=" << new_kf_id);

        // Reset accumulators:
        state_.accum_since_last_kf = mrpt::poses::CPose3D::Identity();

#endif
    }  // end done add a new KF

    // In any case, publish to the SLAM BackEnd what's our **current**
    // vehicle pose, no matter if it's a keyframe or not:
    if (slam_backend_)
    {
        ProfilerEntry tle(profiler_, "onLidar.5.advertiseUpdatedLocalization");

        BackEndBase::AdvertiseUpdatedLocalization_Input new_loc;
        new_loc.timestamp = this_obs_tim;
        // new_loc.reference_kf = state_.last_kf;
        new_loc.pose = state_.accum_since_last_kf.asTPose();

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

        ASSERT_(in.from_pc);
        ASSERT_(in.to_pc);
        const auto& pcs_from = *in.from_pc;
        const auto& pcs_to   = *in.to_pc;

        mrpt::math::TPose3D current_solution = in.init_guess_to_wrt_from;

        mp2p_icp::Results icp_result;

        params_.icp.at(in.align_kind)
            .icp->align(
                pcs_from, pcs_to, current_solution, in.icp_params, icp_result);

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

        // Check quality of match:
        MRPT_TODO("Impl. finite differences based Hessian check");
    }

    MRPT_END
}

void LidarInertialOdometry::onIMU(const CObservation::Ptr& o)
{
    // All methods that are enqueued into a thread pool should have its own
    // top-level try-catch:
    {
        auto lck        = mrpt::lockHelper(is_busy_mtx_);
        state_.busy_imu = true;
    }
    try
    {
        onIMUImpl(o);
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
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
