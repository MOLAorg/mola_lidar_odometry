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
#include <mola_kernel/interfaces/VizInterface.h>
#include <mola_kernel/version.h>

// MRPT:
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/obs/customizable_obs_viz.h>
#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/CAssimpModel.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>

namespace mola
{

mola::gui::Tab LidarOdometry::buildTabStatus()
{
  using namespace mola::gui;

  Tab tab;
  tab.title = "Status";
  tab.widgets.emplace_back(Label{gui_.lbIcpQuality});
  tab.widgets.emplace_back(Label{gui_.lbSensorRates});
  tab.widgets.emplace_back(Label{gui_.lbSensorRange});
  tab.widgets.emplace_back(Label{gui_.lbSpeed});
  tab.widgets.emplace_back(Label{gui_.lbTime});
  tab.widgets.emplace_back(Label{gui_.lbLidarQueue});
  tab.widgets.emplace_back(Label{gui_.lbMapStats});
  return tab;
}

mola::gui::Tab LidarOdometry::buildTabControl()
{
  using namespace mola::gui;

  Tab tab;
  tab.title = "Control";

  tab.widgets.emplace_back(CheckBox{"Active", isActive(), [this](bool checked) {
                                      this->enqueue_request([this, checked]() {
                                        auto lckStateFlags = mrpt::lockHelper(state_flags_mtx_);
                                        state_.active = checked;
                                      });
                                    }});

  tab.widgets.emplace_back(CheckBox{
    "Mapping enabled", params_.local_map_updates.enabled, [this](bool checked) {
      this->enqueue_request([this, checked]() { params_.local_map_updates.enabled = checked; });
    }});

  {
    Label lbl;
    lbl.text =
      std::make_shared<LiveString>("Traject./map are saved at exit or when button clicked");
    lbl.font_size = 14;
    tab.widgets.emplace_back(std::move(lbl));
  }

  {
    Row row;
    row.widgets.emplace_back(
      CheckBox{"Save trajectory", params_.estimated_trajectory.save_to_file, [this](bool checked) {
                 this->enqueue_request(
                   [this, checked]() { params_.estimated_trajectory.save_to_file = checked; });
               }});
    row.widgets.emplace_back(TextBox{
      "", params_.estimated_trajectory.output_file, 13, [this](std::string f) {  // NOLINT
        this->enqueue_request([this, f]() { params_.estimated_trajectory.output_file = f; });
        return true;
      }});
    tab.widgets.emplace_back(std::move(row));
  }

  {
    Row row;
    row.widgets.emplace_back(
      CheckBox{"Generate simplemap", params_.simplemap.generate, [this](bool checked) {
                 this->enqueue_request([this, checked]() { params_.simplemap.generate = checked; });
               }});
    row.widgets.emplace_back(TextBox{
      "", params_.simplemap.save_final_map_to_file, 13, [this](std::string f) {  // NOLINT
        this->enqueue_request([this, f]() { params_.simplemap.save_final_map_to_file = f; });
        return true;
      }});
    tab.widgets.emplace_back(std::move(row));
  }

  {
    Row row;
    row.widgets.emplace_back(Button{
      "Save trajectory now", 0, "", 14, [this]() { this->saveEstimatedTrajectoryToFile(); }});
    row.widgets.emplace_back(
      Button{"Save map now", 0, "", 14, [this]() { this->saveReconstructedMapToFile(); }});
    tab.widgets.emplace_back(std::move(row));
  }

  {
    Row row;
    row.widgets.emplace_back(
      Button{"Reset", 0, "", 0, [this]() { this->enqueue_request([this]() { this->reset(); }); }});
    row.widgets.emplace_back(Button{"Quit", 0, "", 0, [this]() { this->requestShutdown(); }});
    tab.widgets.emplace_back(std::move(row));
  }

  return tab;
}

mola::gui::Tab LidarOdometry::buildTabView()
{
  using namespace mola::gui;

  Tab tab;
  tab.title = "View";

  tab.widgets.emplace_back(CheckBox{
    "Orthographic camera", params_.visualization.camera_orthographic, [this](bool checked) {
      this->enqueue_request([this, checked]() {
        params_.visualization.camera_orthographic = checked;
        visualizer_->update_viewport_camera_orthographic(checked);
      });
    }});

  tab.widgets.emplace_back(CheckBox{
    "Show trajectory", params_.visualization.show_trajectory, [this](bool checked) {
      this->enqueue_request([this, checked]() { params_.visualization.show_trajectory = checked; });
    }});

  tab.widgets.emplace_back(CheckBox{
    "Show raw observation", params_.visualization.show_current_observation, [this](bool checked) {
      this->enqueue_request(
        [this, checked]() { params_.visualization.show_current_observation = checked; });
    }});

  tab.widgets.emplace_back(CheckBox{
    "Show dense local map (decaying)", params_.visualization.show_last_deskewed_observations_decay,
    [this](bool checked) {
      this->enqueue_request([this, checked]() {
        params_.visualization.show_last_deskewed_observations_decay = checked;
      });
    }});

  tab.widgets.emplace_back(CheckBox{
    "Show local map", params_.visualization.show_localmap, [this](bool checked) {
      this->enqueue_request([this, checked]() { params_.visualization.show_localmap = checked; });
    }});

  tab.widgets.emplace_back(CheckBox{
    "Camera follows vehicle", params_.visualization.camera_follows_vehicle, [this](bool checked) {
      this->enqueue_request(
        [this, checked]() { params_.visualization.camera_follows_vehicle = checked; });
    }});

  tab.widgets.emplace_back(CheckBox{
    "Camera rotates with vehicle", params_.visualization.camera_rotates_with_vehicle,
    [this](bool checked) {
      this->enqueue_request(
        [this, checked]() { params_.visualization.camera_rotates_with_vehicle = checked; });
    }});

  tab.widgets.emplace_back(CheckBox{
    "Show gravity-alignment vector", params_.visualization.show_gravity_align_vector,
    [this](bool checked) {
      this->enqueue_request(
        [this, checked]() { params_.visualization.show_gravity_align_vector = checked; });
    }});

  const float sensorPosesSize = params_.visualization.sensor_poses_corner_size > 0.0f
                                  ? params_.visualization.sensor_poses_corner_size
                                  : 0.5f;
  tab.widgets.emplace_back(
    CheckBox{"Show sensor poses", sensorPosesSize > 0.0f, [this, sensorPosesSize](bool checked) {
               this->enqueue_request([this, checked, sensorPosesSize]() {
                 params_.visualization.sensor_poses_corner_size = checked ? sensorPosesSize : 0.0f;
               });
             }});

  return tab;
}

void LidarOdometry::internalBuildGUI()
{
  using namespace mola::gui;

  // Create the LiveStrings shared between the module (writer) and the GUI (reader):
  gui_.lbIcpQuality = std::make_shared<LiveString>(" ");
  gui_.lbSensorRates = std::make_shared<LiveString>(" ");
  gui_.lbSensorRange = std::make_shared<LiveString>(" ");
  gui_.lbTime = std::make_shared<LiveString>(" ");
  gui_.lbSpeed = std::make_shared<LiveString>(" ");
  gui_.lbLidarQueue = std::make_shared<LiveString>(" ");
  gui_.lbMapStats = std::make_shared<LiveString>(" ");

  // Background 3D scene: change background color (backend-agnostic)
  visualizer_->execute_custom_code_on_background_scene([this](mrpt::opengl::Scene & scene) {
    const auto f = params_.visualization.background_color_gray_level;
    scene.getViewport()->setCustomBackgroundColor({f, f, f});
  });

  // Register log messages forwarding to the on-screen console (backend-agnostic)
  this->mrpt::system::COutputLogger::logRegisterCallback(
    [this](
      std::string_view msg, const mrpt::system::VerbosityLevel level, std::string_view loggerName,
      const mrpt::Clock::time_point timestamp) {
      using namespace std::string_literals;

      if (level < this->getMinLoggingLevel()) {
        return;
      }

      visualizer_->output_console_message(
        "["s + mrpt::system::timeLocalToString(timestamp) + "|"s + mrpt::typemeta::enum2str(level) +
        " |"s + std::string(loggerName) + "]"s + std::string(msg));
    });

  const bool hidden = params_.visualization.gui_subwindow_starts_hidden;

  if (visualizer_->gui_backend() == VizInterface::BACKEND_IMGUI) {
    // ImGui: 3 separate windows stacked vertically so all are visible at once.
    // The docking system persists layout across runs via imgui.ini.
    struct WinDef
    {
      std::string title;
      Tab (LidarOdometry::*build)();
      std::array<int, 2> pos;
    };
    const std::array<WinDef, 3> defs = {{
      {"LiDAR Odom: Status", &LidarOdometry::buildTabStatus, {5, 5}},
      {"LiDAR Odom: Control", &LidarOdometry::buildTabControl, {5, 200}},
      {"LiDAR Odom: View", &LidarOdometry::buildTabView, {5, 430}},
    }};
    const std::array<bool, 3> tabEnabled = {{
      params_.visualization.show_tab_status,
      params_.visualization.show_tab_control,
      params_.visualization.show_tab_view,
    }};
    for (size_t i = 0; i < defs.size(); i++) {
      if (!tabEnabled[i]) {
        continue;
      }
      const auto & d = defs[i];
      WindowDescription desc;
      desc.title = d.title;
      desc.position = d.pos;
      desc.size = {340, 0};
      desc.starts_hidden = hidden;
      desc.tabs.emplace_back((this->*d.build)());
      visualizer_->create_subwindow_from_description(desc).get();
    }
  } else {
    // nanogui (and any future backend): single window with 3 tabs.
    WindowDescription desc;
    desc.title = "mola_lidar_odometry";
    desc.position = {5, 700};
    desc.size = {340, 0};
    desc.starts_hidden = hidden;
    if (params_.visualization.show_tab_status) {
      desc.tabs.emplace_back(buildTabStatus());
    }
    if (params_.visualization.show_tab_control) {
      desc.tabs.emplace_back(buildTabControl());
    }
    if (params_.visualization.show_tab_view) {
      desc.tabs.emplace_back(buildTabView());
    }
    visualizer_->create_subwindow_from_description(desc).get();
  }
}

void LidarOdometry::doRemoveCloudsWithDecay()
{
  // Remove possible old 3D objects if the user disabled visualization on the fly:
  if (visualizer_) {
    visualizer_->clear_all_point_clouds_with_decay();
  }
}

namespace
{
void doRecolorize(
  const mrpt::img::TColormap & colormap, const std::string & colorByField,
  const mrpt::maps::CPointsMap * org_cloud, const mrpt::opengl::CPointCloudColoured::Ptr & cloud)
{
  if (colormap == mrpt::img::TColormap::cmNONE) {
    return;
  }

  // Colorize by intensity with custom color map:
  mrpt::obs::PointCloudRecoloringParameters rp;
  rp.colorMap = colormap;
  rp.colorizeByField = colorByField;

  mrpt::obs::recolorize3Dpc(cloud, org_cloud, rp);
};

// Upserts a 3D object, optionally as a child of a movable scene frame node
// (parentFrame). Falls back to a root insert when built against an older
// mola_kernel that lacks the movable-frame API.
void vizUpsert3D(
  const mola::VizInterface::Ptr & viz, const std::string & name,
  const mrpt::opengl::CSetOfObjects::Ptr & obj, const std::string & parentFrame)
{
#if defined(MOLA_KERNEL_VIZ_HAS_MOVABLE_FRAMES)
  viz->update_3d_object(name, obj, "main", "main", parentFrame);
#else
  (void)parentFrame;
  viz->update_3d_object(name, obj);
#endif
}

}  // namespace

void LidarOdometry::setDeskewedColoring(
  mrpt::img::TColormap colormap, const std::string & colorByField)
{
  enqueue_request([this, colormap, colorByField]() {
    params_.visualization.current_observation_colormap = colormap;
    params_.visualization.current_observation_color_by_field = colorByField;
    params_.visualization.last_deskewed_observations_colormap = colormap;
    params_.visualization.last_deskewed_observations_color_by_field = colorByField;
  });
}

void LidarOdometry::setLocalMapColoring(
  mrpt::img::TColormap colormap, const std::string & colorByField)
{
  enqueue_request([this, colormap, colorByField]() {
    params_.visualization.local_map_colormap = colormap;
    params_.visualization.local_map_colormap_color_by_field = colorByField;
  });
}

void LidarOdometry::setTrajectoryVisualization(bool show, const std::vector<float> & rgba)
{
  enqueue_request([this, show, rgba]() {
    params_.visualization.show_trajectory = show;
    if (rgba.size() == 4) {
      params_.visualization.trajectory_rgba = rgba;
    }
  });
}

std::string LidarOdometry::vizParentFrame() const
{
#if defined(MOLA_KERNEL_VIZ_HAS_MOVABLE_FRAMES)
  if (params_.visualization.render_in_movable_frame) {
    return params_.publish_reference_frame;
  }
#endif
  return {};
}

void LidarOdometry::updateVisualization(
  const mp2p_icp::metric_map_t & currentObservation,
  const mrpt::maps::CPointsMap::Ptr & deskewedCloud)
{
  const ProfilerEntry tle(profiler_, "updateVisualization");

  gui_.timestampLastUpdateUI = mrpt::Clock::nowDouble();

  // In this point, we are called by the LIDAR worker thread, so it's safe
  // to read the state without mutexes.
  ASSERT_(visualizer_);

  // Movable scene-frame to draw under (empty = viewport root):
  const std::string vizFrame = vizParentFrame();

  // So they can be called at once at the end to minimize "flicker":
  std::vector<std::function<void()>> updateTasks;

  // Vehicle pose:
  // ---------------------------
  if (!state_.glVehicleModelsLoaded) {
    updateVisualizationInitVehFrame();
  }

  // Update vehicle pose
  // -------------------------
  // Build a *fresh* CSetOfObjects each update. The vehicle model children
  // are read-only after load, so sharing their Ptrs between the cached
  // slot and this fresh parent is safe. We never hand out the cached
  // list itself to the GUI thread, so there is no writer/reader race on
  // the parent (MolaViz::update_3d_object deep-reads it on the GUI
  // thread while the lidar worker may keep producing new frames).
  auto glVehicle = mrpt::opengl::CSetOfObjects::Create();
  if (const auto l = params_.visualization.current_pose_corner_size; l > 0) {
    glVehicle->insert(mrpt::opengl::stock_objects::CornerXYZ(l));
  }
  if (const auto l = params_.visualization.sensor_poses_corner_size; l > 0) {
    for (const auto & [label, sp] : state_.last_lidar_sensor_poses) {
      auto sensorCorner = mrpt::opengl::stock_objects::CornerXYZSimple(l);
      sensorCorner->setPose(sp);
      glVehicle->insert(sensorCorner);
    }
  }
  for (const auto & m : state_.glVehicleModels) {
    glVehicle->insert(m);
  }
  glVehicle->setPose(state_.last_lidar_pose.mean);
  updateTasks.emplace_back([visualizer = visualizer_, glVehicle, vizFrame]() {
    vizUpsert3D(visualizer, "liodom/vehicle", glVehicle, vizFrame);
  });

  // Update current observation
  // ----------------------------
  updateVisualizationCurrentObservation(currentObservation, deskewedCloud);

  // Estimated path:
  // ------------------------
  updateVisualizationPath(updateTasks);

  // Estimated gravity vector:
  // ---------------------------------
  updateVisualizationGravityVector(updateTasks);

  // GUI follow vehicle:
  // ---------------------------
  if (params_.visualization.camera_follows_vehicle) {
    const auto t = state_.last_lidar_pose.mean.translation();
#if defined(MOLA_KERNEL_VIZ_HAS_MOVABLE_FRAMES)
    const std::string lookAtFrame = vizParentFrame();
    updateTasks.emplace_back([visualizer = visualizer_, t, lookAtFrame]() {
      visualizer->update_viewport_look_at(t, "main", "main", lookAtFrame);
    });
#else
    updateTasks.emplace_back(
      [visualizer = visualizer_, t]() { visualizer->update_viewport_look_at(t); });
#endif
  }

  if (params_.visualization.camera_rotates_with_vehicle) {
    const double yaw = state_.last_lidar_pose.mean.yaw();
    double yawIncr = 0;
    if (state_.last_yaw_for_viz_camera) {
      yawIncr = mrpt::math::wrapToPi(yaw - *state_.last_yaw_for_viz_camera);
    }
    state_.last_yaw_for_viz_camera = yaw;

    updateTasks.emplace_back([visualizer = visualizer_, yawIncr]() {
      visualizer->update_viewport_camera_azimuth(yawIncr, false /*incremental*/);
    });
  }

  // ground grid:
  {
    auto glGroundGrid = mrpt::opengl::CSetOfObjects::Create();
    if (params_.visualization.show_ground_grid) {
      auto glGrid = mrpt::opengl::CGridPlaneXY::Create();

      mrpt::math::TBoundingBoxf bbox;
      bbox.min = {-10.0f, -10.0f, -1.0f};
      bbox.max = {+10.0f, +10.0f, +1.0f};

      for (const auto & [lyName, lyMap] : state_.local_map->layers) {
        bbox = bbox.unionWith(lyMap->boundingBox());
      }

      glGrid->setGridFrequency(params_.visualization.ground_grid_spacing);
      glGrid->setColor_u8(0xff, 0xff, 0xff, 0x80);
      glGrid->setPlaneLimits(bbox.min.x, bbox.max.x, bbox.min.y, bbox.max.y);

      glGroundGrid->insert(glGrid);
    }
    updateTasks.emplace_back([visualizer = visualizer_, glGroundGrid, vizFrame]() {
      vizUpsert3D(visualizer, "liodom/groundgrid", glGroundGrid, vizFrame);
    });
  }

  // now, update all visual elements at once:
  for (const auto & ut : updateTasks) {
    ut();
  }

  // Show a warning if no lidar input is being received yet.
  // Note: in localization mode, state_.local_map is already non-empty at
  // startup (loaded from a prebuilt map file), so it cannot be used to tell
  // whether real lidar scans have been processed. Use recent_lidar_stamps_
  // instead, which is only populated once a lidar observation is processed.
  if (state_.recent_lidar_stamps.empty()) {
    const auto s =
      state_.local_map->empty()
        ? mrpt::format(
            "t=%.03f *WARNING* No input LiDAR observations received yet!", mrpt::Clock::nowDouble())
        : mrpt::format(
            "t=%.03f Prebuilt map loaded. Waiting for input LiDAR observations...",
            mrpt::Clock::nowDouble());
    visualizer_->output_console_message(s);
    gui_.was_waiting_for_lidar_data = true;
  } else if (gui_.was_waiting_for_lidar_data) {
    gui_.was_waiting_for_lidar_data = false;
    const auto s =
      mrpt::format("t=%.03f LiDAR data started to be received.", mrpt::Clock::nowDouble());
    visualizer_->output_console_message(s);
    MRPT_LOG_INFO(s);
  }

  updateVisualizationAlways();
}

void LidarOdometry::updateVisualizationAlways()
{
  // Local map: update whenever map content changed, independent of ICP quality.
  {
    std::vector<std::function<void()>> updateTasks;
    updateVisualizationLocalMap(updateTasks);
    for (const auto & ut : updateTasks) {
      ut();
    }
  }

  // Sub-window with custom UI
  // -------------------------------------
  auto lckGuiMtx = mrpt::lockHelper(state_gui_mtx_);
  if (!gui_.gui_created) {
    internalBuildGUI();
    gui_.gui_created = true;
  }

  // Update indicators:
  updateVisualizationTextLabels();
}

void LidarOdometry::updateVisualizationInitVehFrame()
{
  // Load the vehicle 3D model(s) exactly once and cache them as read-only
  // children. The corner is cheap and rebuilt on every update so its color
  // / size can track runtime parameter changes if ever needed.
  state_.glVehicleModels.clear();

  if (!params_.visualization.model.empty()) {
    const auto & _ = params_.visualization;

    for (const auto & model : _.model) {
      const auto localFileName = model.file;

      auto m = mrpt::opengl::CAssimpModel::Create();

      ASSERT_FILE_EXISTS_(localFileName);

      const int loadFlags = mrpt::opengl::CAssimpModel::LoadFlags::RealTimeMaxQuality |
                            mrpt::opengl::CAssimpModel::LoadFlags::FlipUVs;

      m->loadScene(localFileName, loadFlags);

      m->setScale(static_cast<float>(model.scale));
      m->setPose(model.tf);

      state_.glVehicleModels.push_back(m);
    }
  }

  state_.glVehicleModelsLoaded = true;
}

void LidarOdometry::updateVisualizationCurrentObservation(
  const mp2p_icp::metric_map_t & currentObservation,
  const mrpt::maps::CPointsMap::Ptr & deskewedCloud)
{
  const bool hasRaw = (currentObservation.layers.count("raw") != 0);
  const bool showCurrent = params_.visualization.show_current_observation;
  const bool showDecay = params_.visualization.show_last_deskewed_observations_decay;

  const std::string vizFrame = vizParentFrame();

  if (!hasRaw || (!showCurrent && !showDecay)) {
    // Route the clear through the worker thread so it is ordered after any
    // previously enqueued frame lambdas and cannot be overwritten by them.
    auto viz = visualizer_;
    (void)worker_viz_.enqueue([=]() {
      // Always clear cur_obs: we reach here precisely when showCurrent is
      // false or there is no raw layer, so any previously displayed cloud
      // must be removed.
      if (viz) {
        auto empty = mrpt::opengl::CSetOfObjects::Create();
        vizUpsert3D(viz, "liodom/cur_obs", empty, vizFrame);
        viz->clear_all_point_clouds_with_decay();
      }
    });
    return;
  }

  const ProfilerEntry tle1(profiler_, "updateVisualization.update_cur_obs");

  // Snapshot all values the background thread will need.
  // Avoid any access to state_ or params_ from the lambda.
  const mrpt::poses::CPose3D currentPose = state_.last_lidar_pose.mean;

  // Shallow-copy the raw layer for the worker:
  auto rawLayer = currentObservation.layers.at("raw");

  // Copy viz parameters into local value types:
  const auto curObsColormap = params_.visualization.current_observation_colormap;
  const auto curObsColorField = params_.visualization.current_observation_color_by_field;
  const float curObsPointSize = params_.visualization.current_observation_point_size;
  const float curObsAlpha = params_.visualization.current_observation_alpha;

  const auto decayColormap = params_.visualization.last_deskewed_observations_colormap;
  const auto decayColorField = params_.visualization.last_deskewed_observations_color_by_field;
  const float decayPointSize = params_.visualization.last_deskewed_observations_point_size;
  const float decayAlpha = params_.visualization.observations_initial_alpha;
  const double decaySeconds = params_.visualization.observations_decay_seconds;

  // Shared pointer to the deskewed cloud (may be null):
  // Make a copy intentionally so we capture by value in the lambda
  auto deskewed = deskewedCloud;  // NOLINT(performance-unnecessary-copy-initialization)

  // Capture visualizer as a shared_ptr value (not via `this`):
  auto viz = visualizer_;
  const auto * profilerPtr = &profiler_;

  // Enqueue the heavy work into the viz worker thread:
  const auto fut = worker_viz_.enqueue([=]() {
    const ProfilerEntry tle2(*profilerPtr, "updateVisualization.update_cur_obs_thread");

    // --- Current observation cloud ---
    if (showCurrent) {
      mp2p_icp::metric_map_t mm;
      mm.layers["raw"] = rawLayer;

      mp2p_icp::render_params_t rp;
      rp.points.allLayers.force_alpha_channel = true;
      rp.points.allLayers.pointSize = curObsPointSize;
      rp.points.allLayers.color.A = mrpt::f2u8(curObsAlpha);

      auto & cm = rp.points.allLayers.colorMode.emplace();
      cm.colorMap = curObsColormap;
      cm.recolorizeByField = curObsColorField;

      auto glCurrentObs = mm.get_visualization(rp);
      glCurrentObs->setPose(currentPose);

      if (auto cloud = glCurrentObs->getByClass<mrpt::opengl::CPointCloudColoured>(0); cloud) {
        const auto orgCloud = mm.point_layer("raw");
        doRecolorize(curObsColormap, curObsColorField, orgCloud.get(), cloud);
      }

      vizUpsert3D(viz, "liodom/cur_obs", glCurrentObs, vizFrame);
    } else {
      auto empty = mrpt::opengl::CSetOfObjects::Create();
      vizUpsert3D(viz, "liodom/cur_obs", empty, vizFrame);
    }

    // --- Decaying deskewed clouds ---
    if (showDecay && deskewed) {
      mp2p_icp::metric_map_t mm;
      mm.layers["raw"] = deskewed;

      mp2p_icp::render_params_t rp;
      auto & cm = rp.points.allLayers.colorMode.emplace();
      rp.points.allLayers.force_alpha_channel = true;
      cm.colorMap = decayColormap;
      cm.recolorizeByField = decayColorField;
      rp.points.allLayers.pointSize = decayPointSize;
      rp.points.allLayers.color.A = mrpt::f2u8(decayAlpha);

      auto glDecayObs = mm.get_visualization(rp);

      if (auto cloud = glDecayObs->getByClass<mrpt::opengl::CPointCloudColoured>(0); cloud) {
        cloud->setPose(currentPose);
        doRecolorize(decayColormap, decayColorField, deskewed.get(), cloud);

#if defined(MOLA_KERNEL_VIZ_HAS_MOVABLE_FRAMES)
        viz->insert_point_cloud_with_decay(cloud, decaySeconds, "main", "main", vizFrame);
#else
        viz->insert_point_cloud_with_decay(cloud, decaySeconds);
#endif
      }
    } else if (!showDecay) {
      viz->clear_all_point_clouds_with_decay();
    }
  });

  (void)fut;
}

void LidarOdometry::updateVisualizationLocalMap(std::vector<std::function<void()>> & updateTasks)
{
  const std::string vizFrame = vizParentFrame();
  if (
    params_.visualization.show_localmap && state_.local_map &&
    ((state_.mapUpdateCnt++ > params_.visualization.map_update_decimation) &&
     state_.local_map_needs_viz_update)) {
    const ProfilerEntry tle2(profiler_, "updateVisualization.update_local_map");

    state_.mapUpdateCnt = 0;
    state_.local_map_needs_viz_update = false;

    mp2p_icp::render_params_t rp;
    rp.points.allLayers.pointSize = params_.visualization.local_map_point_size;

    auto & cm = rp.points.allLayers.colorMode.emplace();
    cm.colorMap = params_.visualization.local_map_colormap;
    cm.recolorizeByField = params_.visualization.local_map_colormap_color_by_field;

    rp.points.allLayers.render_voxelmaps_free_space =
      params_.visualization.local_map_render_voxelmap_free_space;

    // local map:
    auto glMap = state_.local_map->get_visualization(rp);

    updateTasks.emplace_back([visualizer = visualizer_, glMap, vizFrame]() {
      vizUpsert3D(visualizer, "liodom/localmap", glMap, vizFrame);
    });
  }

  // Clear the local map if the user clicks on "hide it" at runtime:
  if (!params_.visualization.show_localmap) {
    auto glMap = mrpt::opengl::CSetOfObjects::Create();
    updateTasks.emplace_back([visualizer = visualizer_, glMap, vizFrame]() {
      vizUpsert3D(visualizer, "liodom/localmap", glMap, vizFrame);
    });

    // Force an immediate redraw the next time the local map is shown again,
    // since it may not change on its own (e.g. localization-only mode):
    state_.local_map_needs_viz_update = true;
    state_.mapUpdateCnt = std::numeric_limits<int>::max();
  }
}

void LidarOdometry::updateVisualizationPath(std::vector<std::function<void()>> & updateTasks)
{
  if (!params_.visualization.show_trajectory) {
    auto empty = mrpt::opengl::CSetOfObjects::Create();
    updateTasks.emplace_back([visualizer = visualizer_, empty, vizFrame = vizParentFrame()]() {
      vizUpsert3D(visualizer, "liodom/path", empty, vizFrame);
    });
    return;
  }

  const ProfilerEntry tle2(profiler_, "updateVisualization.update_traject");

  if (!state_.glEstimatedPath) {
    state_.glEstimatedPath = mrpt::opengl::CSetOfLines::Create();
    const auto & rgba = params_.visualization.trajectory_rgba;
    state_.glEstimatedPath->setColor(rgba.at(0), rgba.at(1), rgba.at(2), rgba.at(3));
  }
  // Update path viz:
  for (size_t i = state_.glEstimatedPath->size(); i < state_.estimated_trajectory.size(); i++) {
    auto it = state_.estimated_trajectory.begin();
    std::advance(it, i);

    const auto t = it->second.translation();

    if (state_.glEstimatedPath->empty()) {
      state_.glEstimatedPath->appendLine(t, t);
    } else {
      state_.glEstimatedPath->appendLineStrip(t);
    }
  }
  // Hand a *fresh* wrapper containing a deep clone of the lines to the
  // GUI thread, so the worker-private glEstimatedPath buffer can keep
  // growing on subsequent ticks without racing with MolaViz's deep
  // read on the GUI thread.
  auto pathGrp = mrpt::opengl::CSetOfObjects::Create();
  pathGrp->insert(mrpt::opengl::CSetOfLines::Create(*state_.glEstimatedPath));

  updateTasks.emplace_back([visualizer = visualizer_, pathGrp, vizFrame = vizParentFrame()]() {
    vizUpsert3D(visualizer, "liodom/path", pathGrp, vizFrame);
  });
}

void LidarOdometry::updateVisualizationGravityVector(
  std::vector<std::function<void()>> & updateTasks)
{
  const std::string vizFrame = vizParentFrame();
  if (!params_.visualization.show_gravity_align_vector) {
    auto grp = mrpt::opengl::CSetOfObjects::Create();
    updateTasks.emplace_back([visualizer = visualizer_, grp, vizFrame]() {
      vizUpsert3D(visualizer, "liodom/gravity_vector", grp, vizFrame);
    });
    return;
  }

  const ProfilerEntry tle2(profiler_, "updateVisualization.update_gravity");

  const auto gravityPR = state_.gravity_estimator.estimatedPitchRoll(
    params_.imu_gravity_correction.averaging_samples,
    params_.imu_gravity_correction.max_age_seconds);

  if (!gravityPR.has_value()) {
    return;
  }

  const auto [imu_pitch, imu_roll] = *gravityPR;
  const auto & veh = state_.last_lidar_pose.mean;
  const auto arrowPose = mrpt::math::TPose3D(veh.x(), veh.y(), veh.z(), 0.0, imu_pitch, imu_roll);

  auto glArrow = mrpt::opengl::CArrow::Create();
  glArrow->setArrowEnds(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f);
  glArrow->setColor_u8(0xff, 0xa5, 0x00, 0xdc);  // orange

  auto grp = mrpt::opengl::CSetOfObjects::Create();
  grp->setPose(arrowPose);
  grp->insert(glArrow);

  updateTasks.emplace_back([visualizer = visualizer_, grp, vizFrame]() {
    vizUpsert3D(visualizer, "liodom/gravity_vector", grp, vizFrame);
  });
}

void LidarOdometry::updateVisualizationTextLabels()
{
  const ProfilerEntry tle3(profiler_, "updateVisualization.update_gui");

  if (!gui_.lbIcpQuality) {
    return;  // GUI not yet created
  }

  gui_.lbIcpQuality->set(mrpt::format(
    "ICP quality: %.01f%% | Thresh: %.02f | Iters: %zu", 100.0 * state_.last_icp_quality,
    state_.adapt_thres_sigma, state_.last_icp_iterations));

  {
    const auto [rate_lidar, rate_imu, rate_gnss] = state_.get_sensor_rates();
    gui_.lbSensorRates->set(mrpt::format(
      "LiDAR=%6.02f Hz | IMU=%6.02f Hz | GNSS=%5.02f Hz", rate_lidar, rate_imu, rate_gnss));
  }

  if (state_.estimated_observation_radius) {
    gui_.lbSensorRange->set(mrpt::format(
      "Est. obs. radius: %.02f m (inst: %.02f m)", *state_.estimated_observation_radius,
      state_.instantaneous_observation_radius ? *state_.instantaneous_observation_radius : .0));
  } else {
    gui_.lbSensorRange->set("Est. obs. radius: (Not available)");
  }

  {
    const double dtAvr = profiler_.getMeanTime("onLidar");
    gui_.lbTime->set(mrpt::format(
      "Process time: %6.02f ms (%6.02f Hz)", 1e3 * dtAvr, dtAvr > 0 ? 1.0 / dtAvr : .0));
  }

  {
    const double averageLidarQueue = profiler_.getMeanTime("onNewObservation.lidar_queue_length");
    gui_.lbLidarQueue->set(mrpt::format(
      "Dropped frames: %5.02f%% (avr queue=%4.02f)", getDropStats() * 100.0, averageLidarQueue));
  }

  gui_.lbMapStats->set(mrpt::format(
    "Keyframes: Localmap=%zu, simplemap=%zu", state_.distance_checker_local_map->size(),
    state_.distance_checker_simplemap->size()));

  if (state_.last_motion_model_output) {
    const auto & tw = state_.last_motion_model_output->twist;
    const double speed = mrpt::math::TVector3D(tw.vx, tw.vy, tw.vz).norm();
    gui_.lbSpeed->set(mrpt::format(
      "Speed: %.02f m/s | %.02f km/h | %.02f mph", speed, speed * 3600.0 / 1000.0,
      speed / 0.44704));
  } else {
    gui_.lbSpeed->set("Speed: (Not available)");
  }
}

}  // namespace mola
