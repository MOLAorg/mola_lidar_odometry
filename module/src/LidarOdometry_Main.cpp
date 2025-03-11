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

// MP2P_ICP:
#include <mp2p_icp/Solver_GaussNewton.h>

// MRPT:
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/CAssimpModel.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/random.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/topography/conversions.h>

// GUI:
#include <mrpt/gui/CDisplayWindowGUI.h>

// STD:
#include <chrono>
#include <thread>

namespace mola
{

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

    while (isBusy()) {
      MRPT_LOG_THROTTLE_WARN(
        2.0, "Destructor: waiting for remaining tasks on the worker threads...");
      std::this_thread::sleep_for(100ms);
    }
    worker_.clear();

    if (params_.simplemap.generate)  //
      saveReconstructedMapToFile();

    if (params_.estimated_trajectory.save_to_file) saveEstimatedTrajectoryToFile();
  } catch (const std::exception & e) {
    std::cerr << "[~LidarOdometry] Exception: " << e.what();
  }
}

void LidarOdometry::spinOnce()
{
  MRPT_TRY_START

  const ProfilerEntry tleg(profiler_, "spinOnce");

  processPendingUserRequests();

  // Force a refresh of the GUI?
  // Executed here since
  // otherwise the GUI would never show up if inactive, or if the LIDAR
  // observations are misconfigured and are not been fed in.
  if (visualizer_ && ((state_.local_map && state_.local_map->empty()) || !isActive())) {
    if (mrpt::Clock::nowDouble() - gui_.timestampLastUpdateUI > 1.0) {
      updateVisualization({});
    }
  }

  // If SLAM/Localization is disabled, refresh the current map
  // here if needed, since it won't be published until observations arrive.
  {
    auto lckState = mrpt::lockHelper(state_mtx_);

    const auto mapStamp =
      state_.last_obs_timestamp ? *state_.last_obs_timestamp : mrpt::Clock::now();

    doPublishUpdatedMap(mapStamp);
  }

  // Publish optional regular diagnostics:
#if MOLA_VERSION_CHECK(1, 6, 2)
  if (module_is_time_to_publish_diagnostics()) {
    onPublishDiagnostics();
  }
#endif

  MRPT_TRY_END
}

void LidarOdometry::reset()
{
  ASSERTMSG_(!lastInitConfig_.empty(), "initialize() must be called first.");

  auto lck = mrpt::lockHelper(state_mtx_);

  state_ = MethodState();
  initialize(lastInitConfig_);
}

bool LidarOdometry::isBusy() const
{
  bool b;
  is_busy_mtx_.lock();
  b = (state_.worker_tasks_lidar != 0) || (state_.worker_tasks_others != 0);
  is_busy_mtx_.unlock();
  return b || worker_.pendingTasks();
}

bool LidarOdometry::isActive() const
{
  auto lckStateFlags = mrpt::lockHelper(state_flags_mtx_);
  return state_.active;
}

void LidarOdometry::setActive(const bool active)
{
  auto lckStateFlags = mrpt::lockHelper(state_flags_mtx_);
  state_.active = active;
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

std::optional<std::tuple<mrpt::poses::CPose3DPDFGaussian, mrpt::math::TTwist3D>>
LidarOdometry::lastEstimatedState() const
{
  {
    auto lckStateFlags = mrpt::lockHelper(state_flags_mtx_);
    if (!state_.initialized || state_.fatal_error) return {};
  }

  auto lck = mrpt::lockHelper(state_mtx_);

  if (!state_.last_motion_model_output) return {};
  const auto & tw = state_.last_motion_model_output->twist;
  const auto & pose = state_.last_lidar_pose;
  return {{pose, tw}};
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
    const bool dirCreatedOk = mrpt::system::createDirectory(out_basedir);
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

void LidarOdometry::addDropStats(bool frame_is_dropped)
{
  state_.drop_frames_stats_good[state_.drop_frames_stats_next_index] = !frame_is_dropped;
  state_.drop_frames_stats_dropped[state_.drop_frames_stats_next_index] = frame_is_dropped;
  if (++state_.drop_frames_stats_next_index >= MethodState::DROP_STATS_WINDOW_LENGHT)
    state_.drop_frames_stats_next_index = 0;
}

double LidarOdometry::getDropStats() const
{
  auto lckStateFlags = mrpt::lockHelper(state_flags_mtx_);
  const auto good =
    std::count(state_.drop_frames_stats_good.begin(), state_.drop_frames_stats_good.end(), true);
  const auto bad = std::count(
    state_.drop_frames_stats_dropped.begin(), state_.drop_frames_stats_dropped.end(), true);
  const auto total = static_cast<double>(good + bad);
  return total != 0 ? static_cast<double>(bad) / total : .0;
}

}  // namespace mola
