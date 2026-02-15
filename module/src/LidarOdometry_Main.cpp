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

// MP2P_ICP:
#include <mp2p_icp/Solver_GaussNewton.h>
#include <mrpt/config/CConfigFile.h>
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

// Portable config path
#include "libcfgpath/cfgpath.h"

// Portable online version check:
#if defined(MOLA_LO_HAS_ONLINE_VERSION_CHECK)
#include <arpa/inet.h>  // for sockaddr_in
#include <netdb.h>      // for gethostbyname(), hostent
#include <unistd.h>     // for close()

#include <cstring>  // for memset
#include <iostream>
#include <sstream>
#include <string>
#endif

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
    worker_lidar_.clear();
    worker_others_.clear();
    worker_viz_.clear();

    if (params_.simplemap.generate) {
      saveReconstructedMapToFile();
    }

    if (params_.estimated_trajectory.save_to_file) {
      saveEstimatedTrajectoryToFile();
    }
    if (!params_.local_map_updates.save_final_local_map.empty()) {
      saveLocalMapToFile();
    }

    // This must come after save map calls above:
    while (worker_disk_io_.pendingTasks() > 0) {
      MRPT_LOG_THROTTLE_WARN(
        2.0, "Destructor: waiting for pending tasks on the lazy-load write thread...");
      std::this_thread::sleep_for(100ms);
    }
    worker_disk_io_.clear();

  } catch (const std::exception & e) {
    std::cerr << "[~LidarOdometry] Exception: " << e.what();
  }
}

void LidarOdometry::spinOnce()
{
  MRPT_TRY_START

  const ProfilerEntry tle(profiler_, "spinOnce");

  processPendingUserRequests();

  // Force a refresh of the GUI?
  // Executed here since
  // otherwise the GUI would never show up if inactive, or if the LIDAR
  // observations are misconfigured and are not been fed in.
  if (visualizer_ && ((state_.local_map && state_.local_map->empty()) || !isActive())) {
    if (mrpt::Clock::nowDouble() - gui_.timestampLastUpdateUI > 1.0) {
      updateVisualization({}, {});
    }
  }

  // If SLAM/Localization is disabled, refresh the current map
  // here if needed, since it won't be published until observations arrive.
  {
    auto lckState = mrpt::lockHelper(state_mtx_);

    const auto mapStamp =
      state_.last_obs_timestamp ? *state_.last_obs_timestamp : mrpt::Clock::now();

    doPublishUpdatedLocalMap(mapStamp);
  }

  // Publish optional regular diagnostics:
  if (module_is_time_to_publish_diagnostics()) {
    onPublishDiagnostics();
  }

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
  bool b = false;
  is_busy_mtx_.lock();
  b = (state_.worker_tasks_lidar != 0) || (state_.worker_tasks_others != 0);
  is_busy_mtx_.unlock();
  return b || worker_lidar_.pendingTasks() != 0 || worker_others_.pendingTasks() != 0;
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
    if (!state_.initialized || state_.fatal_error) {
      return {};
    }
  }

  auto lck = mrpt::lockHelper(state_mtx_);

  if (!state_.last_motion_model_output) {
    return {};
  }
  const auto & tw = state_.last_motion_model_output->twist;
  const auto & pose = state_.last_lidar_pose;
  return {{pose, tw}};
}

void LidarOdometry::saveEstimatedTrajectoryToFile() const
{
  if (params_.estimated_trajectory.output_file.empty()) {
    return;
  }

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
  if (params_.simplemap.save_final_map_to_file.empty()) {
    return;
  }

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

void LidarOdometry::saveLocalMapToFile() const
{
  if (params_.local_map_updates.save_final_local_map.empty()) {
    return;
  }

  auto lck = mrpt::lockHelper(state_mtx_);

  const auto fil = params_.local_map_updates.save_final_local_map;

  MRPT_LOG_INFO_STREAM("Saving final metric map to file '" << fil << "'...");
  std::cout.flush();

  const bool saved_ok = state_.local_map->save_to_file(fil);
  if (!saved_ok) {
    MRPT_LOG_ERROR_STREAM("Error saving map to: " << fil);
  } else {
    MRPT_LOG_INFO("Final local metric map saved.");
  }
}

void LidarOdometry::unloadPastSimplemapObservations(const size_t maxSizeUnloadQueue) const
{
  auto lck = mrpt::lockHelper(state_simplemap_mtx_);

  auto & pso = state_.past_simplemaps_observations;

  while (pso.size() > maxSizeUnloadQueue) {
    for (auto & o : *pso.begin()->second) {
      handleUnloadSinglePastObservation(o);
    }

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
  if (!oPts) {
    return;
  }

  if (oPts->isExternallyStored()) {
    return;  // already external, do nothing.
  }

  if (params_.simplemap.save_final_map_to_file.empty()) {
    return;  // no generation of simplemap requested by the user
  }

  if (!params_.simplemap.generate_lazy_load_scan_files) {
    return;  // feature is disabled
  }

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
      dirCreatedOk,
      mrpt::format(
        "Error creating lazy-load directory for output simplemap: '%s'", out_basedir.c_str()));

    MRPT_LOG_INFO_STREAM("Creating lazy-load directory for output .simplemap: " << out_basedir);
  }

  // Establish as reference external path base:
  mrpt::io::setLazyLoadPathBase(out_basedir);

  oPts->setAsExternalStorage(
    filename, CObservationPointCloud::ExternalStorageFormat::MRPT_Serialization);

  // Since unload() does the actual saving to disk and it might take some time, let's run it in its own thread.
  const auto fut = this->worker_disk_io_.enqueue([oPts]() {
    try {
      oPts->unload();
    } catch (const std::exception & e) {
      std::cerr << "[LidarOdometry] handleUnloadSinglePastObservation(): Error saving "
                   "observation to disk: "
                << e.what() << "\n";
    }
  });
  (void)fut;
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

#if defined(MOLA_LO_HAS_ONLINE_VERSION_CHECK)
namespace
{
std::string http_get(const std::string & host, const std::string & path = "/")
{
  const int port = 80;

  // Resolve hostname
  hostent * server = gethostbyname(host.c_str());  // NOLINT
  if (server == nullptr) {
    throw std::runtime_error("Error: no such host");
  }

  // Create socket
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    throw std::runtime_error("Error: opening socket");
  }

  // Build server address struct
  sockaddr_in serv_addr{};
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(port);
  std::memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);

  // Connect
  if (connect(sock, reinterpret_cast<sockaddr *>(&serv_addr), sizeof(serv_addr)) < 0) {  // NOLINT
    close(sock);
    throw std::runtime_error("Error: connecting");
  }

  // Send HTTP request
  std::ostringstream request;
  request << "GET " << path << " HTTP/1.0\r\n"
          << "Host: " << host << "\r\n"
          << "Connection: close\r\n\r\n";

  std::string req_str = request.str();
  send(sock, req_str.c_str(), req_str.size(), 0);

  // Read response
  std::string response;
  std::array<char, 1024> buffer = {};
  ssize_t bytes = 0;
  while ((bytes = recv(sock, buffer.data(), buffer.size() - 1, 0)) > 0) {
    buffer[bytes] = '\0';
    response += std::string(buffer.data());
  }

  close(sock);
  return response;
}
bool parse_http_response(const std::string & response, std::string & body_out)
{
  std::istringstream ss(response);
  std::string line;

  // Parse status line
  if (!std::getline(ss, line)) {
    return false;
  }

  if (line.find("200") == std::string::npos) {
    return false;  // not 200 OK
  }

  // Skip headers
  while (std::getline(ss, line)) {
    if (line == "\r" || line.empty()) {
      break;
    }
  }

  // Read remaining lines as body
  std::ostringstream body;
  while (std::getline(ss, line)) {
    body << line;
    if (!ss.eof()) {
      body << '\n';
    }
  }

  // Trim whitespace
  std::string raw = body.str();

  // Remove leading whitespace
  size_t start = raw.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    body_out.clear();
    return true;
  }

  // Remove trailing whitespace
  size_t end = raw.find_last_not_of(" \t\r\n");

  body_out = raw.substr(start, end - start + 1);
  return true;
}

bool version_greater(const std::string & a, const std::string & b)  // NOLINT
{
  std::istringstream sa(a);
  std::istringstream sb(b);
  int va = 0;
  int vb = 0;
  char dot = 0;

  while (true) {
    if (!(sa >> va)) {
      va = 0;  // default 0 if missing
    }
    if (!(sb >> vb)) {
      vb = 0;
    }

    if (va > vb) {
      return true;
    }
    if (va < vb) {
      return false;
    }

    // consume '.' if present
    if (!(sa >> dot) && !(sb >> dot)) {
      break;  // both ended
    }
  }
  return false;  // equal or smaller
}

void check_new_version(mrpt::config::CConfigFile & cfg, mrpt::system::COutputLogger * logger)
{
  // Do we need to check today?
  const auto lastTim = cfg.read_double("config", "last_version_check", 0);
  const auto now = mrpt::Clock::nowDouble();
  if (now < lastTim + 24 * 60 * 60) {
    return;
  }
  // Check in a detached thread:
  std::thread([logger]() {
    try {
      auto response = http_get("docs.mola-slam.org", "/latest/mola_lidar_odometry.version");
      std::string body;
      if (parse_http_response(response, body)) {
        if (version_greater(MOLA_LO_VERSION, body)) {
          logger->logFmt(
            mrpt::system::LVL_DEBUG,
            "[mola-lidar-odometry module] There is a newer version of mola_lidar_odometry "
            "available online: '%s' vs current '%s'",
            body.c_str(), MOLA_LO_VERSION);
        }

      } else {
        std::cout << "[FAIL] Invalid response\n";
      }
    } catch (const std::exception & e) {
      logger->logFmt(mrpt::system::LVL_DEBUG, "[check_new_version] Error: %s", e.what());
    }
  }).detach();

  cfg.write("config", "last_version_check", mrpt::format("%f", now));
}
}  // namespace
#endif

void LidarOdometry::onInitializePersistentState()
{
  try {
    // Get user app config file
    char appCfgFile[2048];
    ::get_user_config_file(appCfgFile, sizeof(appCfgFile), "mola-lidar-odometry");
    mrpt::config::CConfigFile appCfg(appCfgFile);

    // Handle persistent values:
    appCfg.write("config", "last_run", mrpt::format("%f", mrpt::Clock::nowDouble()));

    // Handle printing a notice of newer versions available:
#if defined(MOLA_LO_HAS_ONLINE_VERSION_CHECK)
    check_new_version(appCfg, this);
#endif

  } catch (const std::exception & e) {
    MRPT_LOG_WARN_STREAM("Error initializing persistent config file:\n" << e.what());
  }
}

void LidarOdometry::doWriteDebugTracesFile(const mrpt::Clock::time_point & this_obs_tim)
{
  if (!params_.debug_traces.save_to_file) {
    return;  // disabled
  }

  if (debug_traces_of_ && !debug_traces_of_->is_open()) {
    return;  // apparently, an error creating the file
  }

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
    for (const auto & [name, value] : vars) {
      of << "\"" << name << "\",";
    }
    of << "\n";
  }
  for (const auto & [name, value] : vars) {
    of << mrpt::format("%f,", value);
  }
  of << "\n";
}

void LidarOdometry::addDropStats(bool frame_is_dropped)
{
  state_.drop_frames_stats_good[state_.drop_frames_stats_next_index] = !frame_is_dropped;
  state_.drop_frames_stats_dropped[state_.drop_frames_stats_next_index] = frame_is_dropped;
  if (++state_.drop_frames_stats_next_index >= MethodState::DROP_STATS_WINDOW_LENGTH) {
    state_.drop_frames_stats_next_index = 0;
  }
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
