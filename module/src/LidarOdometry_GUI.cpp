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
#include <mola_kernel/version.h>

// MRPT:
#if !MOLA_VERSION_CHECK(2, 6, 0)
#include <mrpt/gui/CDisplayWindowGUI.h>
#endif
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/obs/customizable_obs_viz.h>
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

#if MOLA_VERSION_CHECK(2, 6, 0)

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

  WindowDescription desc;
  desc.title = "mola_lidar_odometry";
  desc.position = {5, 700};
  desc.size = {340, 0};
  desc.starts_hidden = params_.visualization.gui_subwindow_starts_hidden;

  // ------- Tab 1: Status -------
  {
    Tab tab;
    tab.title = "Status";
    tab.widgets.emplace_back(Label{gui_.lbIcpQuality});
    tab.widgets.emplace_back(Label{gui_.lbSensorRates});
    tab.widgets.emplace_back(Label{gui_.lbSensorRange});
    tab.widgets.emplace_back(Label{gui_.lbSpeed});
    tab.widgets.emplace_back(Label{gui_.lbTime});
    tab.widgets.emplace_back(Label{gui_.lbLidarQueue});
    tab.widgets.emplace_back(Label{gui_.lbMapStats});
    desc.tabs.emplace_back(std::move(tab));
  }

  // ------- Tab 2: Control -------
  {
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
      row.widgets.emplace_back(CheckBox{
        "Save trajectory", params_.estimated_trajectory.save_to_file, [this](bool checked) {
          this->enqueue_request(
            [this, checked]() { params_.estimated_trajectory.save_to_file = checked; });
        }});
      row.widgets.emplace_back(TextBox{
        "", params_.estimated_trajectory.output_file, 13, [this](std::string f) {
          this->enqueue_request([this, f]() { params_.estimated_trajectory.output_file = f; });
          return true;
        }});
      tab.widgets.emplace_back(std::move(row));
    }

    {
      Row row;
      row.widgets.emplace_back(CheckBox{
        "Generate simplemap", params_.simplemap.generate, [this](bool checked) {
          this->enqueue_request([this, checked]() { params_.simplemap.generate = checked; });
        }});
      row.widgets.emplace_back(TextBox{
        "", params_.simplemap.save_final_map_to_file, 13, [this](std::string f) {
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
      row.widgets.emplace_back(Button{
        "Reset", 0, "", 0, [this]() { this->enqueue_request([this]() { this->reset(); }); }});
      row.widgets.emplace_back(Button{"Quit", 0, "", 0, [this]() { this->requestShutdown(); }});
      tab.widgets.emplace_back(std::move(row));
    }

    desc.tabs.emplace_back(std::move(tab));
  }

  // ------- Tab 3: View -------
  {
    Tab tab;
    tab.title = "View";

    tab.widgets.emplace_back(CheckBox{
      "Orthographic camera", params_.visualization.camera_orthographic, [this](bool checked) {
        this->enqueue_request([this, checked]() {
          params_.visualization.camera_orthographic = checked;
          visualizer_->update_viewport_camera_orthographic(checked);
        });
      }});

    tab.widgets.emplace_back(
      CheckBox{"Show trajectory", params_.visualization.show_trajectory, [this](bool checked) {
                 this->enqueue_request(
                   [this, checked]() { params_.visualization.show_trajectory = checked; });
               }});

    tab.widgets.emplace_back(CheckBox{
      "Show raw observation", params_.visualization.show_current_observation, [this](bool checked) {
        this->enqueue_request(
          [this, checked]() { params_.visualization.show_current_observation = checked; });
      }});

    tab.widgets.emplace_back(CheckBox{
      "Show dense local map (decaying)",
      params_.visualization.show_last_deskewed_observations_decay, [this](bool checked) {
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
      "Show log messages", params_.visualization.show_console_messages, [this](bool checked) {
        this->enqueue_request(
          [this, checked]() { params_.visualization.show_console_messages = checked; });
      }});

    desc.tabs.emplace_back(std::move(tab));
  }

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

      if (!params_.visualization.show_console_messages) return;
      if (level < this->getMinLoggingLevel()) return;

      visualizer_->output_console_message(
        "["s + mrpt::system::timeLocalToString(timestamp) + "|"s + mrpt::typemeta::enum2str(level) +
        " |"s + std::string(loggerName) + "]"s + std::string(msg));
    });

  // Create the sub-window and block until the GUI thread has processed it:
  visualizer_->create_subwindow_from_description(desc).get();
}

#else  // !MOLA_VERSION_CHECK(2, 6, 0): legacy nanogui path

void LidarOdometry::internalBuildGUI_Legacy()
{
  ASSERT_(gui_.ui);

  gui_.ui->requestFocus();
  gui_.ui->setVisible(!params_.visualization.gui_subwindow_starts_hidden);
  gui_.ui->setPosition({5, 700});

  gui_.ui->setLayout(
    new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 5, 2));
  gui_.ui->setFixedWidth(340);

  auto * tabWidget = gui_.ui->add<nanogui::TabWidget>();

  auto * tab1 = tabWidget->createTab("Status");
  tab1->setLayout(new nanogui::GroupLayout());

  auto * tab2 = tabWidget->createTab("Control");
  tab2->setLayout(new nanogui::GroupLayout());

  auto * tab3 = tabWidget->createTab("View");
  tab3->setLayout(new nanogui::GroupLayout());

  tabWidget->setActiveTab(0);

  // tab 1: status
  gui_.lbIcpQuality = tab1->add<nanogui::Label>(" ");
  gui_.lbSensorRates = tab1->add<nanogui::Label>(" ");
  gui_.lbSensorRange = tab1->add<nanogui::Label>(" ");
  gui_.lbSpeed = tab1->add<nanogui::Label>(" ");
  gui_.lbTime = tab1->add<nanogui::Label>(" ");
  gui_.lbLidarQueue = tab1->add<nanogui::Label>(" ");
  gui_.lbMapStats = tab1->add<nanogui::Label>(" ");

  // tab 2: control
  gui_.cbActive = tab2->add<nanogui::CheckBox>("Active");
  gui_.cbActive->setChecked(isActive());
  gui_.cbActive->setCallback([&](bool checked) {
    this->enqueue_request([this, checked]() {
      auto lckStateFlags = mrpt::lockHelper(state_flags_mtx_);
      state_.active = checked;
    });
  });

  gui_.cbMapping = tab2->add<nanogui::CheckBox>("Mapping enabled");
  gui_.cbMapping->setChecked(params_.local_map_updates.enabled);
  gui_.cbMapping->setCallback([&](bool checked) {
    this->enqueue_request([this, checked]() { params_.local_map_updates.enabled = checked; });
  });

  {
    auto * lbMsg =
      tab2->add<nanogui::Label>("Traject./map are saved at exit or when button clicked");
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

    gui_.cbSaveSimplemap = panel->add<nanogui::CheckBox>("Generate simplemap");
    gui_.cbSaveSimplemap->setChecked(params_.simplemap.generate);
    gui_.cbSaveSimplemap->setCallback([this](bool checked) {
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

    auto * btnSaveTrajectory = panel->add<nanogui::Button>("Save trajectory now", ENTYPO_ICON_SAVE);
    btnSaveTrajectory->setFontSize(14);

    btnSaveTrajectory->setCallback([this]() { this->saveEstimatedTrajectoryToFile(); });

    auto * btnSaveMap = panel->add<nanogui::Button>("Save map now", ENTYPO_ICON_SAVE);
    btnSaveMap->setFontSize(14);

    btnSaveMap->setCallback([this]() { this->saveReconstructedMapToFile(); });
  }

  {
    auto * panel = tab2->add<nanogui::Widget>();
    panel->setLayout(
      new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Maximum, 1, 1));

    auto * btnReset = panel->add<nanogui::Button>("Reset", ENTYPO_ICON_CCW);
    btnReset->setCallback([&]() { this->enqueue_request([this]() { this->reset(); }); });

    auto * btnQuit = panel->add<nanogui::Button>("Quit", ENTYPO_ICON_ARROW_LEFT);
    btnQuit->setCallback([&]() { this->requestShutdown(); });
  }

  // tab 3: view
  auto * cbOrthoCam = tab3->add<nanogui::CheckBox>("Orthographic camera");
  cbOrthoCam->setChecked(params_.visualization.camera_orthographic);
  cbOrthoCam->setCallback([&](bool checked) {
    this->enqueue_request([this, checked]() {
      params_.visualization.camera_orthographic = checked;
      visualizer_->update_viewport_camera_orthographic(checked);
    });
  });

  auto * cbShowTrajectory = tab3->add<nanogui::CheckBox>("Show trajectory");
  cbShowTrajectory->setChecked(params_.visualization.show_trajectory);
  cbShowTrajectory->setCallback([&](bool checked) {
    this->enqueue_request([this, checked]() { params_.visualization.show_trajectory = checked; });
  });

  auto * cbShowObs = tab3->add<nanogui::CheckBox>("Show raw observation");
  cbShowObs->setChecked(params_.visualization.show_current_observation);
  cbShowObs->setCallback([&](bool checked) {
    this->enqueue_request(
      [this, checked]() { params_.visualization.show_current_observation = checked; });
  });

  auto * cbShowDLM = tab3->add<nanogui::CheckBox>("Show dense local map (decaying)");
  cbShowDLM->setChecked(params_.visualization.show_last_deskewed_observations_decay);
  cbShowDLM->setCallback([&](bool checked) {
    this->enqueue_request(
      [this, checked]() { params_.visualization.show_last_deskewed_observations_decay = checked; });
  });

  auto * cbShowSLM = tab3->add<nanogui::CheckBox>("Show local map");
  cbShowSLM->setChecked(params_.visualization.show_localmap);
  cbShowSLM->setCallback([&](bool checked) {
    this->enqueue_request([this, checked]() { params_.visualization.show_localmap = checked; });
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

  // Background 3D scene: change background color
  visualizer_->execute_custom_code_on_background_scene([this](mrpt::opengl::Scene & scene) {
    const auto f = params_.visualization.background_color_gray_level;
    scene.getViewport()->setCustomBackgroundColor({f, f, f});
  });

  this->mrpt::system::COutputLogger::logRegisterCallback(
    [&](
      std::string_view msg, const mrpt::system::VerbosityLevel level, std::string_view loggerName,
      const mrpt::Clock::time_point timestamp) {
      using namespace std::string_literals;

      if (!params_.visualization.show_console_messages) {
        return;
      }

      if (level < this->getMinLoggingLevel()) {
        return;
      }

      visualizer_->output_console_message(
        "["s + mrpt::system::timeLocalToString(timestamp) + "|"s + mrpt::typemeta::enum2str(level) +
        " |"s + std::string(loggerName) + "]"s + std::string(msg));
    });
}

#endif  // MOLA_VERSION_CHECK(2, 6, 0)

void LidarOdometry::doRemoveCloudsWithDecay()
{
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

}  // namespace

void LidarOdometry::updateVisualization(
  const mp2p_icp::metric_map_t & currentObservation,
  const mrpt::maps::CPointsMap::Ptr & deskewedCloud)
{
  const ProfilerEntry tle(profiler_, "updateVisualization");

  gui_.timestampLastUpdateUI = mrpt::Clock::nowDouble();

  // In this point, we are called by the LIDAR worker thread, so it's safe
  // to read the state without mutexes.
  ASSERT_(visualizer_);

  // So they can be called at once at the end to minimize "flicker":
  std::vector<std::function<void()>> updateTasks;

  // Vehicle pose:
  // ---------------------------
  if (!state_.glVehicleFrame) {
    updateVisualizationInitVehFrame();
  }

  // Update vehicle pose
  // -------------------------
  state_.glVehicleFrame->setPose(state_.last_lidar_pose.mean);
  updateTasks.emplace_back(
    [this]() { visualizer_->update_3d_object("liodom/vehicle", state_.glVehicleFrame); });

  // Update current observation
  // ----------------------------
  updateVisualizationCurrentObservation(currentObservation, deskewedCloud);

  // Estimated path:
  // ------------------------
  updateVisualizationPath(updateTasks);

  // GUI follow vehicle:
  // ---------------------------
  if (params_.visualization.camera_follows_vehicle) {
    updateTasks.emplace_back([this]() {
      visualizer_->update_viewport_look_at(state_.last_lidar_pose.mean.translation());
    });
  }

  if (params_.visualization.camera_rotates_with_vehicle) {
    updateTasks.emplace_back([this]() {
      const double yaw = state_.last_lidar_pose.mean.yaw();
      double yawIncr = 0;
      if (state_.last_yaw_for_viz_camera) {
        yawIncr = mrpt::math::wrapToPi(yaw - *state_.last_yaw_for_viz_camera);
      }
      state_.last_yaw_for_viz_camera = yaw;

      visualizer_->update_viewport_camera_azimuth(yawIncr, false /*incremental*/);
    });
  }

  // Local map:
  // -----------------------------
  updateVisualizationLocalMap(updateTasks);

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
    updateTasks.emplace_back(
      [this, glGroundGrid]() { visualizer_->update_3d_object("liodom/groundgrid", glGroundGrid); });
  }

  // now, update all visual elements at once:
  for (const auto & ut : updateTasks) {
    ut();
  }

  // Show a warning if no lidar input is being received:
  if (state_.local_map->empty()) {
    const auto s = mrpt::format(
      "t=%.03f *WARNING* No input LiDAR observations received yet!", mrpt::Clock::nowDouble());
    visualizer_->output_console_message(s);
  }

  // Sub-window with custom UI
  // -------------------------------------
  auto lckGuiMtx = mrpt::lockHelper(state_gui_mtx_);
#if MOLA_VERSION_CHECK(2, 6, 0)
  if (!gui_.gui_created) {
    internalBuildGUI();
    gui_.gui_created = true;
  }
#else
  if (gui_.ui == nullptr) {
    auto fut = visualizer_->create_subwindow("mola_lidar_odometry");
    gui_.ui = fut.get();

    // wait until this code is executed in the UI thread:
    auto fut2 = visualizer_->enqueue_custom_nanogui_code([this]() { internalBuildGUI_Legacy(); });

    fut2.get();
  }
#endif

  // Update indicators:
  updateVisualizationTextLabels();
}

void LidarOdometry::updateVisualizationInitVehFrame()
{
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

      const int loadFlags = mrpt::opengl::CAssimpModel::LoadFlags::RealTimeMaxQuality |
                            mrpt::opengl::CAssimpModel::LoadFlags::FlipUVs;

      m->loadScene(localFileName, loadFlags);

      m->setScale(static_cast<float>(model.scale));
      m->setPose(model.tf);

      state_.glVehicleFrame->insert(m);
    }
  }
}

void LidarOdometry::updateVisualizationCurrentObservation(
  const mp2p_icp::metric_map_t & currentObservation,
  const mrpt::maps::CPointsMap::Ptr & deskewedCloud)
{
  const bool hasRaw = (currentObservation.layers.count("raw") != 0);
  const bool showCurrent = params_.visualization.show_current_observation;
  const bool showDecay = params_.visualization.show_last_deskewed_observations_decay;

  if (!hasRaw || (!showCurrent && !showDecay)) {
    // None enabled, clean up directly:
    if (showCurrent) {
      auto empty = mrpt::opengl::CSetOfObjects::Create();
      visualizer_->update_3d_object("liodom/cur_obs", empty);
    }
    doRemoveCloudsWithDecay();
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
      auto & cm = rp.points.allLayers.colorMode.emplace();
      rp.points.allLayers.force_alpha_channel = true;
      cm.colorMap = curObsColormap;
      cm.recolorizeByField = curObsColorField;
      rp.points.allLayers.pointSize = curObsPointSize;
      rp.points.allLayers.color.A = mrpt::f2u8(curObsAlpha);

      auto glCurrentObs = mm.get_visualization(rp);
      glCurrentObs->setPose(currentPose);

      if (auto cloud = glCurrentObs->getByClass<mrpt::opengl::CPointCloudColoured>(0); cloud) {
        const auto orgCloud = mm.point_layer("raw");
        doRecolorize(curObsColormap, curObsColorField, orgCloud.get(), cloud);
      }

      viz->update_3d_object("liodom/cur_obs", glCurrentObs);
    } else {
      auto empty = mrpt::opengl::CSetOfObjects::Create();
      viz->update_3d_object("liodom/cur_obs", empty);
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

        viz->insert_point_cloud_with_decay(cloud, decaySeconds);
      }
    }
  });

  (void)fut;
}

void LidarOdometry::updateVisualizationLocalMap(std::vector<std::function<void()>> & updateTasks)
{
  if (
    params_.visualization.show_localmap && state_.local_map &&
    ((state_.mapUpdateCnt++ > params_.visualization.map_update_decimation) &&
     state_.local_map_needs_viz_update)) {
    const ProfilerEntry tle2(profiler_, "updateVisualization.update_local_map");

    state_.mapUpdateCnt = 0;
    state_.local_map_needs_viz_update = false;

    mp2p_icp::render_params_t rp;
    rp.points.allLayers.pointSize = params_.visualization.local_map_point_size;

    rp.points.allLayers.render_voxelmaps_free_space =
      params_.visualization.local_map_render_voxelmap_free_space;

    // local map:
    auto glMap = state_.local_map->get_visualization(rp);

    updateTasks.emplace_back(
      [this, glMap]() { visualizer_->update_3d_object("liodom/localmap", glMap); });
  }

  // Clear the local map if the user clicks on "hide it" at runtime:
  if (!params_.visualization.show_localmap) {
    auto glMap = mrpt::opengl::CSetOfObjects::Create();
    updateTasks.emplace_back(
      [this, glMap]() { visualizer_->update_3d_object("liodom/localmap", glMap); });
  }
}

void LidarOdometry::updateVisualizationPath(std::vector<std::function<void()>> & updateTasks)
{
  if (params_.visualization.show_trajectory) {
    const ProfilerEntry tle2(profiler_, "updateVisualization.update_traject");

    if (!state_.glEstimatedPath) {
      state_.glEstimatedPath = mrpt::opengl::CSetOfLines::Create();
      const auto & rgba = params_.visualization.trajectory_rgba;
      state_.glEstimatedPath->setColor(rgba.at(0), rgba.at(1), rgba.at(2), rgba.at(3));
      state_.glPathGrp = mrpt::opengl::CSetOfObjects::Create();
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
    state_.glPathGrp->clear();
    state_.glPathGrp->insert(mrpt::opengl::CSetOfLines::Create(*state_.glEstimatedPath));

    updateTasks.emplace_back(
      [this]() { visualizer_->update_3d_object("liodom/path", state_.glPathGrp); });
  }
}

void LidarOdometry::updateVisualizationTextLabels()
{
  const ProfilerEntry tle3(profiler_, "updateVisualization.update_gui");

#if MOLA_VERSION_CHECK(2, 6, 0)
  if (!gui_.lbIcpQuality) return;  // GUI not yet created

  gui_.lbIcpQuality->set(mrpt::format(
    "ICP quality: %.01f%% | Thresh: %.02f | Iters: %zu", 100.0 * state_.last_icp_quality,
    state_.adapt_thres_sigma, state_.last_icp_iterations));

  {
    const auto [rate_lidar, rate_imu] = state_.get_lidar_imu_sensor_rates();
    gui_.lbSensorRates->set(mrpt::format("LiDAR=%6.02f Hz | IMU=%6.02f Hz", rate_lidar, rate_imu));
  }

  if (state_.estimated_sensor_max_range) {
    gui_.lbSensorRange->set(mrpt::format(
      "Est. max range: %.02f m (inst: %.02f m)", *state_.estimated_sensor_max_range,
      state_.instantaneous_sensor_max_range ? *state_.instantaneous_sensor_max_range : .0));
  } else {
    gui_.lbSensorRange->set("Est. max range: (Not available)");
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

#else  // legacy nanogui path

  gui_.lbIcpQuality->setCaption(mrpt::format(
    "ICP quality: %.01f%% | Thresh: %.02f | Iters: %zu", 100.0 * state_.last_icp_quality,
    state_.adapt_thres_sigma, state_.last_icp_iterations));

  {
    const auto [rate_lidar, rate_imu] = state_.get_lidar_imu_sensor_rates();
    gui_.lbSensorRates->setCaption(
      mrpt::format("LiDAR=%6.02f Hz | IMU=%6.02f Hz", rate_lidar, rate_imu));
  }

  if (state_.estimated_sensor_max_range) {
    gui_.lbSensorRange->setCaption(mrpt::format(
      "Est. max range: %.02f m (inst: %.02f m)", *state_.estimated_sensor_max_range,
      state_.instantaneous_sensor_max_range ? *state_.instantaneous_sensor_max_range : .0));
  } else {
    gui_.lbSensorRange->setCaption("Est. max range: (Not available)");
  }

  {
    const double dtAvr = profiler_.getMeanTime("onLidar");
    gui_.lbTime->setCaption(mrpt::format(
      "Process time: %6.02f ms (%6.02f Hz)", 1e3 * dtAvr, dtAvr > 0 ? 1.0 / dtAvr : .0));
  }

  {
    const double averageLidarQueue = profiler_.getMeanTime("onNewObservation.lidar_queue_length");

    gui_.lbLidarQueue->setCaption(mrpt::format(
      "Dropped frames: %5.02f%% (avr queue=%4.02f)", getDropStats() * 100.0, averageLidarQueue));
  }
  gui_.lbMapStats->setCaption(mrpt::format(
    "Keyframes: Localmap=%zu, simplemap=%zu", state_.distance_checker_local_map->size(),
    state_.distance_checker_simplemap->size()));

  if (state_.last_motion_model_output) {
    const auto & tw = state_.last_motion_model_output->twist;
    const double speed = mrpt::math::TVector3D(tw.vx, tw.vy, tw.vz).norm();

    gui_.lbSpeed->setCaption(mrpt::format(
      "Speed: %.02f m/s | %.02f km/h | %.02f mph", speed, speed * 3600.0 / 1000.0,
      speed / 0.44704));
  } else {
    gui_.lbSpeed->setCaption("Speed: (Not available)");
  }

#endif  // MOLA_VERSION_CHECK(2, 6, 0)
}

}  // namespace mola
