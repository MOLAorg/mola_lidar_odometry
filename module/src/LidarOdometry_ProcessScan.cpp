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

// mp2p_icp:
#include <mp2p_icp_filters/FilterDeskew.h>

// MRPT:
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/Lie/SO.h>

namespace mola
{

bool LidarOdometry::isPipelineUsingIMU() const
{
  auto lckState = mrpt::lockHelper(state_mtx_);

  if (state_.isPipelinesUsingIMU.has_value()) {
    return *state_.isPipelinesUsingIMU;
  }

  const auto hasDeskewStage = [](const mp2p_icp_filters::FilterPipeline & pipeline) {
    for (const auto & stage : pipeline) {
      auto deskew = std::dynamic_pointer_cast<mp2p_icp_filters::FilterDeskew>(stage);
      if (!deskew) {
        continue;
      }
      // It's deskew: we need IMU if using IMU-based methods:
      return deskew->method != mp2p_icp_filters::MotionCompensationMethod::None &&
             deskew->method != mp2p_icp_filters::MotionCompensationMethod::Linear;
    }
    return false;
  };

  state_.isPipelinesUsingIMU = hasDeskewStage(state_.pc_filter1) ||
                               hasDeskewStage(state_.pc_filter2) ||
                               hasDeskewStage(state_.pc_filter3);

  return *state_.isPipelinesUsingIMU;
}

// here happens the main stuff:
void LidarOdometry::processLidarScan(const CObservation::ConstPtr & obs)
{
  using namespace std::string_literals;

  const ProfilerEntry tle_global(profiler_, "onLidar");

  // Check if we need to process any pending async request:
  {
    const ProfilerEntry tle_ur(profiler_, "onLidar.userRequests");
    processPendingUserRequests();
  }

  // make sure data is loaded, if using an offline lazy-load dataset.
  ASSERT_(obs);
  obs->load();
  const auto this_obs_tim = obs->timestamp;

  auto lckState = mrpt::lockHelper(state_mtx_);

  profiler_.leave("delay_onNewObs_to_process");

  // for rate stats:
  state_.append_lidar_stamp(obs->sensorLabel, obs->timestamp);

  // Only process pointclouds that are sufficiently apart in time:

  // Keep timestamps for logging purposes:
  state_.last_obs_timestamp = this_obs_tim;
  if (!state_.first_ever_timestamp) {
    state_.first_ever_timestamp = this_obs_tim;
  }

  // Handle initial localization options:
  if (!state_.initial_localization_done) {
    handleInitialLocalization();
  }

  if (state_.last_obs_tim_by_label.count(obs->sensorLabel)) {
    const double lidar_delta_time =
      mrpt::system::timeDifference(state_.last_obs_tim_by_label[obs->sensorLabel], this_obs_tim);

    if (lidar_delta_time < params_.min_time_between_scans) {
      // Drop observation.
      MRPT_LOG_DEBUG_FMT(
        "onLidarImpl: dropping observation, for %f< "
        "`min_time_between_scans`=%f.",
        lidar_delta_time, params_.min_time_between_scans);
      return;
    }
  }

  state_.last_obs_tim_by_label[obs->sensorLabel] = this_obs_tim;

  // Use the observation to update the estimated sensor range:
  if (!state_.estimated_sensor_max_range.has_value()) {
    doInitializeEstimatedMaxSensorRange(*obs);
  }

  // Handle multiple simultaneous LIDARs:
  const mrpt::obs::CSensoryFrame sf = collectRawObservations(obs);
  if (sf.empty()) {
    return;  // not all required LiDARs yet.
  }

  // Refresh dyn. variables used in the mp2p_icp pipelines:
  updatePipelineDynamicVariables(this_obs_tim);

  MRPT_LOG_DEBUG_STREAM("Dynamic variables: " << state_.parameter_source.printVariableValues());

  // Extract points from observation:
  auto observation = observationFromRawSensor(sf);

  // Keep a copy of "raw" for visualization in the GUI:
  mp2p_icp::metric_map_t observationRawForViz;
  if (observation->layers.count("raw")) {
    observationRawForViz.layers["raw"] = observation->layers.at("raw");
  }

  // Filter/segment the point cloud (optional, but normally will be
  // present):
  ProfilerEntry tle1(profiler_, "onLidar.1.filter_1st");

  mp2p_icp_filters::apply_filter_pipeline(state_.pc_filter1, *observation, profiler_);
  tle1.stop();

  ProfilerEntry tle1b(profiler_, "onLidar.1.filter_2nd");

  mp2p_icp_filters::apply_filter_pipeline(state_.pc_filter2, *observation, profiler_);

  tle1b.stop();

  // Update sensor max range from the obs map layers:
  doUpdateEstimatedMaxSensorRange(*observation);

  profiler_.enter("onLidar.2.copy_vars");

  // check observation validity:
  if (const bool obsValid = doCheckIsValidObservation(*observation); !obsValid) {
    MRPT_LOG_WARN_FMT(
      "Observation discarded as non-valid for pathStep=%zu, timestamp=%s UTC",
      state_.estimated_trajectory.size(), mrpt::system::dateTimeToString(this_obs_tim).c_str());

    return;
  }

  // Store for next step:
  std::optional<mrpt::Clock::time_point> last_obs_tim;
  if (auto it = state_.last_obs_tim_by_label.find(obs->sensorLabel);
      it != state_.last_obs_tim_by_label.end()) {
    last_obs_tim = it->second;
  }

  profiler_.leave("onLidar.2.copy_vars");

  if (observation->empty()) {
    MRPT_LOG_WARN_STREAM(
      "Observation of type `" << obs->GetRuntimeClass()->className
                              << "` could not be converted into a "
                                 "pointcloud. Doing nothing.");
    return;
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
    state_.navstate_fuse->estimated_navstate(this_obs_tim, params_.publish_reference_frame);

  bool hasMotionModel = state_.last_motion_model_output.has_value();

  // don't count as a valid motion model if its uncertainty is too large:
  if (hasMotionModel) {
    const auto & cov_inv = state_.last_motion_model_output->pose.cov_inv;

    hasMotionModel = cov_inv(0, 0) >= params_.min_motion_model_xyz_cov_inv &&
                     cov_inv(1, 1) >= params_.min_motion_model_xyz_cov_inv &&
                     cov_inv(2, 2) >= params_.min_motion_model_xyz_cov_inv;

    if (!hasMotionModel) {
      MRPT_LOG_DEBUG_STREAM(
        "Discarding motion model at due to large uncertainty: pose_inv_cov=\n"
        << state_.last_motion_model_output->pose.cov_inv.asString());
    }
  }

  tleMotion.stop();

  if (state_.local_map->empty() && params_.local_map_updates.enabled) {
    // Skip ICP.
    MRPT_LOG_DEBUG("First pointcloud: skipping ICP and directly adding to local map.");

    // Create a first KF (at origin)
    updateLocalMap = true;
    updateSimpleMap = true;     // update SimpleMap too
    distance_enough_sm = true;  // and treat this one as a KeyFrame with SF

    // Update trajectory too:
    {
      auto lck = mrpt::lockHelper(state_trajectory_mtx_);
      state_.estimated_trajectory.insert(
        this_obs_tim, params_.initial_localization.fixed_initial_pose);
    }

    // Define the current robot pose at the origin with minimal uncertainty
    // (cannot be zero).
    mrpt::poses::CPose3DPDFGaussian initPose;
    initPose.mean = mrpt::poses::CPose3D(params_.initial_localization.fixed_initial_pose);
    initPose.cov.setDiagonal(1e-12);

    state_.navstate_fuse->fuse_pose(this_obs_tim, initPose, params_.publish_reference_frame);
  } else {
    // Register point clouds using ICP:
    // ------------------------------------
    profiler_.enter("onLidar.2c.prepare_icp_in");

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
        if (std::dynamic_pointer_cast<const mrpt::obs::CObservation2DRangeScan>(obs)) {
          // fix: z, pitch (rot_y), roll (rot_x):
          const double large_certainty = 1e6;

          auto & m = in.prior->mean;

          m.z(0);
          m.setYawPitchRoll(m.yaw(), .0, .0);

          in.prior->cov_inv(2, 2) = large_certainty;  // dz
          in.prior->cov_inv(3, 3) = large_certainty;  // rx
          in.prior->cov_inv(4, 4) = large_certainty;  // ry
        }

        MRPT_LOG_DEBUG_STREAM("ICP prior=" << *in.prior);
      }

      MRPT_LOG_DEBUG_STREAM(
        "Est.twist=" << (hasMotionModel ? state_.last_motion_model_output->twist.asString()
                                        : "(none)"s)
                     << " dt=" << dt << " s. "
                     << " Est. pose: " << state_.last_motion_model_output->pose.mean
                     << "\nEst. pose cov_inv:\n"
                     << state_.last_motion_model_output->pose.cov_inv.asString());
    } else {
      // Use the last pose without velocity motion model:
      in.init_guess_local_wrt_global = state_.last_lidar_pose.mean.asTPose();

      // (Skip the warning message if we are in the first timestep, since that's totally normal and expected, and the warning becomes confusing):
      if (state_.estimated_trajectory.size() > 1) {
        MRPT_LOG_THROTTLE_WARN_FMT(
          2.0,
          "Not able to use velocity motion model for this timestep "
          "(pathStep=%zu, timestamp=%s UTC)",
          state_.estimated_trajectory.size(), mrpt::system::dateTimeToString(this_obs_tim).c_str());
      }
    }

    // If we don't have a valid twist estimation, use a larger ICP
    // correspondence threshold:
    in.align_kind = hasMotionModel ? AlignKind::RegularOdometry : AlignKind::NoMotionModel;

    in.icp_params = params_.icp[in.align_kind].icp_parameters;
    in.last_keyframe_pose = state_.last_lidar_pose.mean;

    if (state_.last_icp_timestamp) {
      in.time_since_last_keyframe =
        mrpt::system::timeDifference(*state_.last_icp_timestamp, this_obs_tim);
    }
    state_.last_icp_timestamp = this_obs_tim;

    profiler_.leave("onLidar.2c.prepare_icp_in");

    // -----------------------------------------------------
    // Run ICP
    // -----------------------------------------------------
    ProfilerEntry tle_icp(profiler_, "onLidar.3.run_icp");

    mrpt::math::TPose3D current_solution = in.init_guess_local_wrt_global;
    size_t twistCorrectionCount = 0;

    auto & icpCase = params_.icp.at(in.align_kind);

    icpCase.icp->setIterationHook([&](const mp2p_icp::ICP::IterationHook_Input & ih) {
      mp2p_icp::ICP::IterationHook_Output ho;

      if (!params_.optimize_twist) {
        return ho;  // not enabled
      }

      if (twistCorrectionCount >= params_.optimize_twist_max_corrections) {
        return ho;
      }

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

      // Skip ICP if we started without map and with mapping disabled:
      if (state_.local_map->empty()) {
        ASSERT_(!params_.local_map_updates.enabled);
        break;
      }

      // Run ICP:
      icpCase.icp->align(
        *observation, *state_.local_map, current_solution, icp_params, icp_result, in.prior);

      if (icp_result.nIterations <= remainingIcpIters) {
        remainingIcpIters -= icp_result.nIterations;
      } else {  // who knows?...
        remainingIcpIters = 0;
      }

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
      "termReason=%s pose_cov diagonal sigmas:{%e %e %e [m] %e %e %e [deg]}",
      static_cast<unsigned int>(in.align_kind), 100.0 * out.goodness,
      static_cast<unsigned int>(icp_result.nIterations),
      out.found_pose_to_wrt_from.getMeanVal().asString().c_str(),
      mrpt::typemeta::enum2str(icp_result.terminationReason).c_str(),
      std::sqrt(out.found_pose_to_wrt_from.cov(0, 0)),
      std::sqrt(out.found_pose_to_wrt_from.cov(1, 1)),
      std::sqrt(out.found_pose_to_wrt_from.cov(2, 2)),
      mrpt::RAD2DEG(std::sqrt(out.found_pose_to_wrt_from.cov(3, 3))),
      mrpt::RAD2DEG(std::sqrt(out.found_pose_to_wrt_from.cov(4, 4))),
      mrpt::RAD2DEG(std::sqrt(out.found_pose_to_wrt_from.cov(5, 5))));

    tle_icp.stop();
    // ------------------------------------------------------
    // (end, run ICP)
    // ------------------------------------------------------

    const bool icpIsGood = (out.goodness >= params_.min_icp_goodness);

    state_.last_icp_was_good = icpIsGood;
    state_.last_icp_quality = out.goodness;
    state_.last_icp_iterations = out.icp_iterations;

    if (icpIsGood) {
      state_.last_lidar_pose = out.found_pose_to_wrt_from;
    }

    // Update velocity model:
    if (icpIsGood) {
      // Good ICP, update state estimation filter with new data from ICP:

      if (state_.step_counter_post_relocalization == 0) {
        // Do integrate info:
        state_.navstate_fuse->fuse_pose(
          this_obs_tim, out.found_pose_to_wrt_from, params_.publish_reference_frame);
      } else {
        // Skip during post-relocalization:
        state_.step_counter_post_relocalization--;
      }

    } else {
      // Bad ICP:
      // Was: state_.navstate_fuse->reset();
      // Do not reset state estimation in order to allow it to fuse other sensor sources.
    }

    // Update trajectory too:
    if (icpIsGood) {
      auto lck = mrpt::lockHelper(state_trajectory_mtx_);
      state_.estimated_trajectory.insert(this_obs_tim, state_.last_lidar_pose.mean);
    }

    // Update for stats:
    state_.parameter_source.updateVariable("icp_iterations", out.icp_iterations);
    state_.parameter_source.updateVariable(
      "twistCorrectionCount", static_cast<double>(twistCorrectionCount));

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
    if (!state_.distance_checker_local_map) {
      state_.distance_checker_local_map.emplace(
        params_.local_map_updates.measure_from_last_kf_only);
    }

    if (!state_.distance_checker_simplemap) {
      state_.distance_checker_simplemap.emplace(params_.simplemap.measure_from_last_kf_only);
    }

    // Create a new KF if the distance since the last one is large enough:
    const auto [isFirstPoseInChecker, distanceToClosest] =
      state_.distance_checker_local_map->check(state_.last_lidar_pose.mean);

    const double euclidean_dist_since_last = distanceToClosest.norm();
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
        euclidean_dist_since_last > params_.local_map_updates.min_translation_between_keyframes ||
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
        const ProfilerEntry tleCleanup(profiler_, "onLidar.distant_kfs_cleanup");

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

    const double euclidean_dist_since_last_sm = distanceToClosestSM.norm();
    const double rot_since_last_sm =
      mrpt::poses::Lie::SO<3>::log(distanceToClosestSM.getRotationMatrix()).norm();

    distance_enough_sm =
      isFirstPoseInSMChecker ||
      euclidean_dist_since_last_sm > params_.simplemap.min_translation_between_keyframes ||
      rot_since_last_sm > mrpt::DEG2RAD(params_.simplemap.min_rotation_between_keyframes);

    // clang-format off
    updateSimpleMap =
      params_.simplemap.generate &&
      (icpIsGood &&
       (distance_enough_sm || params_.simplemap.add_non_keyframes_too)
       );
    // clang-format on

    if (updateSimpleMap && distance_enough_sm) {
      state_.distance_checker_simplemap->insert(state_.last_lidar_pose.mean);
    }

    MRPT_LOG_DEBUG_FMT(
      "Since last KF: dist=%5.03f m rotation=%.01f deg updateLocalMap=%s "
      "updateSimpleMap=%s",
      euclidean_dist_since_last, mrpt::RAD2DEG(rot_since_last), updateLocalMap ? "YES" : "NO",
      updateSimpleMap ? "YES" : "NO");

  }  // end: yes, we can do ICP

  // If this was a bad ICP, and we just started with an empty map, re-start again:
  if (
    !state_.last_icp_was_good && state_.estimated_trajectory.size() == 1 &&
    params_.local_map_updates.enabled) {
    // Re-start the local map:
    state_.local_map->clear();
    state_.estimated_trajectory.clear();
    updateLocalMap = false;
    state_.last_icp_was_good = true;

    MRPT_LOG_WARN("Bad first ICP, re-starting from scratch with a new local map");
  }

  // Should we create a new KF?
  if (updateLocalMap) {
    const ProfilerEntry tle2(profiler_, "onLidar.4.update_local_map");

    // If the local map is empty, create it from this first observation:
    if (state_.local_map->empty()) {
      const ProfilerEntry tle3(profiler_, "onLidar.4.update_local_map.create");
      MRPT_LOG_DEBUG("Creating local map since it was empty");

      for (const auto & o : sf) {
        mp2p_icp_filters::apply_generators(state_.local_map_generators, *o, *state_.local_map);
      }
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
    // in particular, [robot_x, ..., robot_roll]:
    updatePipelineDynamicVariablesRobotPoseOnly();
    // Make all changes effective and evaluate the variables now:
    state_.parameter_source.realize();

    // 3/4: Apply pipeline
    mp2p_icp_filters::apply_filter_pipeline(state_.obs2map_merge, *state_.local_map, profiler_);

    // 4/4: remove temporary layers:
    for (const auto & [lyName, lyMap] : observation->layers) {
      state_.local_map->layers.erase(lyName);
    }

    tle3.stop();

    state_.mark_local_map_as_updated();

  }  // end done add a new KF to local map

  // Optional build simplemap:
  if (updateSimpleMap) {
    doUpdateSimpleMap(sf, distance_enough_sm, observation, this_obs_tim);
  }

  // In any case, publish the vehicle pose, no matter if it's a keyframe or not,
  // if ICP quality was good enough:
  if (state_.last_icp_was_good) {
    doPublishUpdatedLocalization(this_obs_tim);
  }

  // Publish new local map & deskewed scan for visualization on external systems (ROS):
  doPublishUpdatedLocalMap(this_obs_tim);
  doPublishDeskewedScan(this_obs_tim);

  // Optional debug traces to CSV file:
  doWriteDebugTracesFile(this_obs_tim);

  // Optional real-time GUI via MOLA VizInterface:
  if (visualizer_ && state_.local_map) {
    const ProfilerEntry tle(profiler_, "onLidar.6.updateVisualization");

    updateVisualization(observationRawForViz);
  }
}

mp2p_icp::metric_map_t::Ptr LidarOdometry::observationFromRawSensor(
  const mrpt::obs::CSensoryFrame & sf)
{
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
    if (obsTrg != observation.get()) {
      observation->merge_with(*obsTrg);
    }
  }
  tle0.stop();
  return observation;
}

mrpt::obs::CSensoryFrame LidarOdometry::collectRawObservations(
  const mrpt::obs::CObservation::ConstPtr & obs)
{
  mrpt::obs::CSensoryFrame sf;
  if (params_.multiple_lidars.lidar_count > 1) {
    // Synchronize 2+ lidars:
    state_.sync_obs[obs->sensorLabel] = obs;
    if (state_.sync_obs.size() < params_.multiple_lidars.lidar_count) {
      MRPT_LOG_THROTTLE_DEBUG(5.0, "Skipping ICP since still waiting for all of multiple LIDARs");
      return {};
    }
    // now, keep all of them within the time window:
    for (const auto & [label, o] : state_.sync_obs) {
      const auto dt = std::abs(mrpt::system::timeDifference(o->timestamp, obs->timestamp));
      if (dt > params_.multiple_lidars.max_time_offset) {
        continue;
      }

      sf += std::const_pointer_cast<mrpt::obs::CObservation>(o);  // include this observation
    }
    // and clear for the next iter:
    state_.sync_obs.clear();

    ASSERT_(!sf.empty());
    MRPT_LOG_DEBUG_STREAM(
      "multiple_lidars: " << sf.size() << " valid observations have been synchronized.");
  } else {
    // Single LIDAR:
    sf.insert(std::const_pointer_cast<mrpt::obs::CObservation>(obs));
  }

  return sf;
}

void LidarOdometry::doUpdateSimpleMap(
  const mrpt::obs::CSensoryFrame & sf, const bool distance_enough_sm,
  const mp2p_icp::metric_map_t::Ptr & observation, const mrpt::Clock::time_point & this_obs_tim)
{
  using namespace std::string_literals;

  auto lck = mrpt::lockHelper(state_simplemap_mtx_);

  auto keyframe_obs = mrpt::obs::CSensoryFrame::Create();
  // Add observations only if this is a real keyframe
  // (the alternative is this is a regular frame, but the option
  //  add_non_keyframes_too is set):
  if (distance_enough_sm) {
    *keyframe_obs += sf;

    if (params_.simplemap.save_deskewed_scans) {
      mp2p_icp::metric_map_t mm;
      mm.layers["raw"] = observation->layers.at("raw");
      // This will remove the "raw" layer and replace it with a "deskewed" one:
      mp2p_icp_filters::apply_filter_pipeline(state_.obsDeskewForViz, mm);
      const auto deskewed_cloud = mm.point_layer(mm.layers.begin()->first);

      auto od = mrpt::obs::CObservationPointCloud::Create();
      od->timestamp = this_obs_tim;
      auto spc = mrpt::maps::CPointsMapXYZI::Create();
      spc->insertAnotherMap(deskewed_cloud.get(), {});
      od->pointcloud = spc;
      od->sensorLabel = "deskewed";
      keyframe_obs->insert(od);
    }

    const auto curLidarStamp = this_obs_tim;

    // insert GNSS too? Search for a close-enough observation:
    std::optional<double> closestTimeAbsDiff;
    mrpt::obs::CObservationGPS::ConstPtr closestGPS;

    for (const auto & [gpsStamp, gpsObs] : state_.last_gnss_) {
      const double timeDiff = std::abs(mrpt::system::timeDifference(gpsStamp, curLidarStamp));

      if (timeDiff > params_.simplemap.save_gnss_max_age) {
        continue;
      }

      if (!closestTimeAbsDiff || timeDiff < *closestTimeAbsDiff) {
        closestTimeAbsDiff = timeDiff;
        closestGPS = gpsObs;
      }
    }
    if (closestGPS) {
      *keyframe_obs += std::const_pointer_cast<mrpt::obs::CObservationGPS>(closestGPS);
    }
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
    if (bbox) {
      bbox = bbox->unionWith(layerMap->boundingBox());
    } else {
      bbox = layerMap->boundingBox();
    }
  }
  if (bbox) {
    kf_metadata["frame_bbox_min"] = "'"s + bbox->min.asString() + "'"s;
    kf_metadata["frame_bbox_max"] = "'"s + bbox->max.asString() + "'"s;
  }

  // Store local velocity buffer in the KF metadata so it is possible to deskew the scan later on with precision
#if MP2P_ICP_VERSION >= 0x010801  // 1.8.1
  kf_metadata["local_velocity_buffer"] = state_.parameter_source.localVelocityBuffer.toYAML();
#endif

  // convert yaml to string:
  std::stringstream ss;
  ss << kf_metadata;
  metadataObs->text = ss.str();

  // insert it:
  *keyframe_obs += metadataObs;

  // Add keyframe to simple map:
  MRPT_LOG_DEBUG_STREAM("New SimpleMap KeyFrame. SF=" << keyframe_obs->size() << " observations.");

  std::optional<mrpt::math::TTwist3D> curTwist;
  if (state_.last_motion_model_output) {
    curTwist = state_.last_motion_model_output->twist;
  }

  state_.reconstructed_simplemap.insert(
    // Pose: mean + covariance
    mrpt::poses::CPose3DPDFGaussian::Create(state_.last_lidar_pose),
    // SensoryFrame: set of observations from this KeyFrame:
    keyframe_obs,
    // twist
    curTwist);

  // Mechanism to free old SFs:
  // We cannot unload them right now, for the case when they are being
  // used in a GUI, etc.
  // (1/2) Add to the list:
  state_.past_simplemaps_observations[this_obs_tim] = keyframe_obs;

  const ProfilerEntry tleUnloadSM(profiler_, "onLidar.5.unload_past_sm_obs");

  // (2/2) Unload old lazy-load observations to save RAM, if applicable:
  constexpr size_t MAX_SIZE_UNLOAD_QUEUE = 100;
  unloadPastSimplemapObservations(MAX_SIZE_UNLOAD_QUEUE);
}

}  // namespace mola
