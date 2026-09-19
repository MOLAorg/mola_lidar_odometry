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
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/version.h>
#include <mrpt/viz/CArrow.h>
#include <mrpt/viz/CAssimpModel.h>
#include <mrpt/viz/CCylinder.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CPointCloudColoured.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CText.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/stock_objects.h>

// STD:
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <sstream>

namespace mola
{

namespace
{
#if defined(MOLA_HAS_TRANSFORM_TREE_SOURCE)
/** Splits "a, b ,c" into {"a","b","c"}, ignoring empty items. */
std::set<std::string> splitCommaSeparated(const std::string & s)
{
  std::set<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item = mrpt::system::trim(item);
    if (!item.empty()) {
      out.insert(item);
    }
  }
  return out;
}

// tf links render as CCylinder (thin tubes), visible even at a distance,
// unlike GL lines, which MRPT cannot draw with a configurable thickness.
// Few radial slices and no end caps keep the per-link triangle count low.
constexpr uint32_t kTfLinkCylinderSlices = 6;

// A CCylinder's local Z axis runs from its base (at the object's origin) to
// its top (at height `len`). Orient it so that axis points from `a` to `b`:
// pitch = acos(nz), yaw = atan2(ny, nx) for the unit direction (nx,ny,nz),
// derived from MRPT's R = Rz(yaw)*Ry(pitch)*Rx(roll) convention applied to
// local Z (roll is a free rotation about the cylinder's own axis, so 0 is
// fine). Returns nullptr for a degenerate (near-zero-length) link.
mrpt::viz::CCylinder::Ptr makeTfLinkCylinder(
  const mrpt::math::TPoint3D & a, const mrpt::math::TPoint3D & b, float radius)
{
  const mrpt::math::TPoint3D d = b - a;
  const double len = d.norm();
  if (len < 1e-6) {
    return {};
  }
  const double pitch = std::acos(std::clamp(d.z / len, -1.0, 1.0));
  const double yaw = std::atan2(d.y, d.x);

  auto glCyl =
    mrpt::viz::CCylinder::Create(radius, radius, static_cast<float>(len), kTfLinkCylinderSlices);
  glCyl->setHasBases(false, false);
  glCyl->setColor_u8(0x80, 0x80, 0x80, 0xff);
  glCyl->setPose(mrpt::poses::CPose3D(a.x, a.y, a.z, yaw, pitch, 0.0));
  return glCyl;
}

/** Fills `out` with one XYZ corner per frame of `tree` (plus, optionally, the
 *  parent->child links and the frame names), keeping the poses exactly as
 *  given, i.e. relative to the tree's own root.
 *
 *  `subtreeRoot` (empty = the whole tree) selects which subtree to draw;
 *  `excluded` frames are dropped together with their own subtrees.
 *
 *  \return how many frames were actually drawn.
 */
size_t renderTfTree(
  const mola::TransformTree & tree, const std::string & subtreeRoot,
  const std::set<std::string> & excluded, float cornerSize, bool showLinks, float linkRadius,
  bool showNames, mrpt::viz::CSetOfObjects & out)
{
  // Nodes come ordered parents-before-children, so both the "keep only this
  // subtree" and the "drop this frame" tests propagate down in a single pass:
  const bool wholeTree = subtreeRoot.empty() || subtreeRoot == tree.root;
  std::set<std::string> kept;
  std::set<std::string> dropped;
  std::map<std::string, mrpt::poses::CPose3D> posesByFrame;
  size_t nDrawn = 0;

  for (const auto & node : tree.nodes) {
    if (!wholeTree && node.frame != subtreeRoot && kept.count(node.parent) == 0) {
      continue;
    }
    if (excluded.count(node.frame) != 0 || dropped.count(node.parent) != 0) {
      dropped.insert(node.frame);
      continue;
    }
    kept.insert(node.frame);
    nDrawn++;
    posesByFrame[node.frame] = node.pose_in_root;

    if (cornerSize > 0) {
      auto corner = mrpt::viz::stock_objects::CornerXYZSimple(cornerSize);
      corner->setPose(node.pose_in_root);
      out.insert(corner);
    }

    if (showNames) {
      auto label = mrpt::viz::CText::Create(node.frame);
      label->setLocation(node.pose_in_root.translation());
      out.insert(label);
    }

    if (showLinks && !node.parent.empty()) {
      if (const auto it = posesByFrame.find(node.parent); it != posesByFrame.end()) {
        if (auto glCyl = makeTfLinkCylinder(
              it->second.translation(), node.pose_in_root.translation(), linkRadius);
            glCyl) {
          out.insert(glCyl);
        }
      }
    }
  }

  return nDrawn;
}
#endif

}  // namespace

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

  {
    Row row;
    row.widgets.emplace_back(CheckBox{
      "Show /tf tree", params_.visualization.show_tf_tree, [this](bool checked) {
        this->enqueue_request([this, checked]() { params_.visualization.show_tf_tree = checked; });
      }});
    row.widgets.emplace_back(TextBox{
      "size", std::to_string(params_.visualization.tf_tree_corner_size), 5,
      [this](std::string s) {  // NOLINT
        float v = 0;
        try {
          v = std::stof(s);
        } catch (const std::exception &) {
          return false;
        }
        if (v < 0) {
          return false;
        }
        this->enqueue_request([this, v]() { params_.visualization.tf_tree_corner_size = v; });
        return true;
      }});
    tab.widgets.emplace_back(std::move(row));
  }

  {
    Row row;
    row.widgets.emplace_back(
      CheckBox{"tf links", params_.visualization.tf_tree_show_links, [this](bool checked) {
                 this->enqueue_request(
                   [this, checked]() { params_.visualization.tf_tree_show_links = checked; });
               }});
    row.widgets.emplace_back(
      CheckBox{"tf names", params_.visualization.tf_tree_show_names, [this](bool checked) {
                 this->enqueue_request(
                   [this, checked]() { params_.visualization.tf_tree_show_names = checked; });
               }});
    row.widgets.emplace_back(TextBox{
      "link radius", std::to_string(params_.visualization.tf_tree_link_radius), 5,
      [this](std::string s) {  // NOLINT
        float v = 0;
        try {
          v = std::stof(s);
        } catch (const std::exception &) {
          return false;
        }
        if (v < 0) {
          return false;
        }
        this->enqueue_request([this, v]() { params_.visualization.tf_tree_link_radius = v; });
        return true;
      }});
    tab.widgets.emplace_back(std::move(row));
  }

  {
    Row row;
    row.widgets.emplace_back(TextBox{
      "tf root", params_.visualization.tf_tree_root_frame, 13, [this](std::string f) {  // NOLINT
        this->enqueue_request([this, f]() { params_.visualization.tf_tree_root_frame = f; });
        return true;
      }});
    row.widgets.emplace_back(TextBox{
      "exclude", params_.visualization.tf_tree_exclude_frames, 13,
      [this](std::string f) {  // NOLINT
        this->enqueue_request([this, f]() { params_.visualization.tf_tree_exclude_frames = f; });
        return true;
      }});
    tab.widgets.emplace_back(std::move(row));
  }

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
  visualizer_->execute_custom_code_on_background_scene([this](mrpt::viz::Scene & scene) {
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
  const mrpt::maps::CPointsMap * org_cloud, const mrpt::viz::CPointCloudColoured::Ptr & cloud)
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

// A deep copy of `m`, cheap enough to make while holding
// local_map_content_mtx_. It exists to keep that mutex OUT of the render:
// get_visualization() costs ~6x this copy (167.9 ms vs 28.1 ms mean on an
// Oxford Spires local map) and touches nothing but the copy, so holding the
// map mutex across it stalled the LiDAR worker's own map insertion for
// ~110 ms at a time.
//
// Point layers are copied into a plain CGenericPointsMap rather than cloned
// through their own type: insertAnotherMap() carries every per-point field
// over and skips non-finite slots, while the target has no spatial index to
// build. Cloning e.g. mola::IncrementalPointCloud via its copy constructor
// would instead bulk-build a k-d tree that the renderer never queries.
// The copy sees, in addition to the live points, the storage slots that are
// tombstoned but not reclaimed yet; that population measured <2% of storage
// on Oxford Spires (live/storage 0.999 mean, 0.982 worst), i.e. visually
// irrelevant, and it is the same trade-off doPublishUpdatedLocalMap() already
// makes when copying layers out for ROS.
mp2p_icp::metric_map_t cheapLayerSnapshot(const mp2p_icp::metric_map_t & m)
{
  // Shallow copy first, so any metadata this struct grows (id, label, lines,
  // planes, georeferencing, ...) is carried over without enumerating it here:
  mp2p_icp::metric_map_t out = m;

  for (auto & [name, layer] : out.layers) {
    if (!layer) {
      continue;
    }

    if (const auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(layer); pts) {
      auto copy = mrpt::maps::CGenericPointsMap::Create();
      copy->insertAnotherMap(pts.get(), mrpt::poses::CPose3D::Identity());
      // The renderer reads both of these off the layer:
      copy->renderOptions = pts->renderOptions;
      copy->genericMapParams = pts->genericMapParams;
      layer = copy;
    } else {
      // Voxel maps and friends: the RTTI copy is what they support.
      layer = std::dynamic_pointer_cast<mrpt::maps::CMetricMap>(layer->duplicateGetSmartPtr());
    }
  }

  return out;
}

// Upserts a 3D object, optionally as a child of a movable scene frame node
// (parentFrame). Falls back to a root insert when built against an older
// mola_kernel that lacks the movable-frame API.
void vizUpsert3D(
  const mola::VizInterface::Ptr & viz, const std::string & name,
  const mrpt::viz::CSetOfObjects::Ptr & obj, const std::string & parentFrame)
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

void LidarOdometry::setCurrentPoseCornerVisualization(bool show)
{
  enqueue_request([this, show]() { params_.visualization.show_current_pose_corner = show; });
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
  const mrpt::maps::CPointsMap::Ptr & deskewedCloud, std::unique_lock<std::mutex> & lckState)
{
  const ProfilerEntry tle(profiler_, "updateVisualization");

  gui_.timestampLastUpdateUI = mrpt::Clock::nowDouble();

  // We may be called either from the LIDAR worker thread or from the executor
  // thread (spinOnce()), so state_ is read under state_mtx_, already held by
  // the caller (see lckState).
  ASSERT_(lckState.owns_lock());
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
  auto glVehicle = mrpt::viz::CSetOfObjects::Create();
  if (const auto l = params_.visualization.current_pose_corner_size;
      params_.visualization.show_current_pose_corner && l > 0) {
    glVehicle->insert(mrpt::viz::stock_objects::CornerXYZ(l));
  }
  if (const auto l = params_.visualization.sensor_poses_corner_size; l > 0) {
    for (const auto & [label, sp] : state_.last_lidar_sensor_poses) {
      auto sensorCorner = mrpt::viz::stock_objects::CornerXYZSimple(l);
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

  // Robot /tf tree (opt-in):
  // ---------------------------
  updateVisualizationTfTree(updateTasks, vizFrame);

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
    auto glGroundGrid = mrpt::viz::CSetOfObjects::Create();
    if (params_.visualization.show_ground_grid) {
      auto glGrid = mrpt::viz::CGridPlaneXY::Create();

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

  updateVisualizationAlways(lckState);
}

void LidarOdometry::updateVisualizationAlways(std::unique_lock<std::mutex> & lckState)
{
  ASSERT_(lckState.owns_lock());

  // Local map: update whenever map content changed, independent of ICP quality.
  updateVisualizationLocalMap();

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

      auto m = mrpt::viz::CAssimpModel::Create();

      ASSERT_FILE_EXISTS_(localFileName);

      const int loadFlags = mrpt::viz::CAssimpModel::LoadFlags::RealTimeMaxQuality |
                            mrpt::viz::CAssimpModel::LoadFlags::FlipUVs;

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
        auto empty = mrpt::viz::CSetOfObjects::Create();
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

      if (auto cloud = glCurrentObs->getByClass<mrpt::viz::CPointCloudColoured>(0); cloud) {
        const auto orgCloud = mm.point_layer("raw");
        doRecolorize(curObsColormap, curObsColorField, orgCloud.get(), cloud);
      }

      vizUpsert3D(viz, "liodom/cur_obs", glCurrentObs, vizFrame);
    } else {
      auto empty = mrpt::viz::CSetOfObjects::Create();
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

      if (auto cloud = glDecayObs->getByClass<mrpt::viz::CPointCloudColoured>(0); cloud) {
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

void LidarOdometry::updateVisualizationLocalMap()
{
  const std::string vizFrame = vizParentFrame();

  bool decimationReached = false;
  if (params_.visualization.show_localmap && state_.local_map) {
    const auto decimation =
      static_cast<unsigned int>(std::max(0, params_.visualization.map_update_decimation));
    decimationReached = state_.mapUpdateCnt > decimation;

    // Saturating increment: the counter is *set* to its maximum value elsewhere
    // to request an immediate refresh, so it must not wrap around.
    if (state_.mapUpdateCnt < std::numeric_limits<unsigned int>::max()) {
      state_.mapUpdateCnt++;
    }
  }

  if (decimationReached && state_.local_map_needs_viz_update) {
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

    // local map: this recolors/renders every point currently in the map, an
    // O(map size) cost that keeps growing with it (e.g. under
    // mola::IncrementalPointCloud, whose local map never shrinks back below
    // its eviction cube), and it used to run right here, on the LiDAR worker
    // thread: measured at ~55 ms mean / ~120 ms max on a GrandTour mission,
    // which is what turned one onLidar in ~18 into a >100 ms spike and backed
    // up the scan queue. So it is handed to a worker instead, where it takes
    // only the finer-grained local_map_content_mtx_ (never state_mtx_, hence
    // no inversion of the lock order documented in the class declaration).
    //
    // Snapshot all values the worker will need. Avoid any access to state_ or
    // params_ from the lambda; keeping a shared_ptr to the map also means a
    // concurrent reset() cannot drop the last reference mid-render.
    const auto localMap = state_.local_map;
    auto viz = visualizer_;
    const auto * profilerPtr = &profiler_;
    auto * mapContentsMtx = &local_map_content_mtx_;

    (void)worker_viz_local_map_.enqueue([=]() {
      const ProfilerEntry tle3(*profilerPtr, "updateVisualization.update_local_map_thread");

      // Copy under the map mutex, render outside it: see cheapLayerSnapshot().
      mp2p_icp::metric_map_t snapshot;
      {
        const ProfilerEntry tleCopy(*profilerPtr, "updateVisualization.update_local_map_copy");
        auto lckMapContents = mrpt::lockHelper(*mapContentsMtx);
        snapshot = cheapLayerSnapshot(*localMap);
      }

      const auto glMap = snapshot.get_visualization(rp);
      vizUpsert3D(viz, "liodom/localmap", glMap, vizFrame);
    });
  }

  // Clear the local map if the user clicks on "hide it" at runtime:
  if (!params_.visualization.show_localmap) {
    // Routed through the same worker so it is ordered after any render already
    // enqueued above and cannot be overwritten by it.
    auto viz = visualizer_;
    (void)worker_viz_local_map_.enqueue([=]() {
      auto glMap = mrpt::viz::CSetOfObjects::Create();
      vizUpsert3D(viz, "liodom/localmap", glMap, vizFrame);
    });

    // Force an immediate redraw the next time the local map is shown again,
    // since it may not change on its own (e.g. localization-only mode):
    state_.local_map_needs_viz_update = true;
    state_.mapUpdateCnt = std::numeric_limits<unsigned int>::max();
  }
}

void LidarOdometry::updateVisualizationTfTree(
  std::vector<std::function<void()>> & updateTasks, const std::string & vizFrame)
{
#if defined(MOLA_HAS_TRANSFORM_TREE_SOURCE)
  const auto & vp = params_.visualization;

  auto glTree = mrpt::viz::CSetOfObjects::Create();

  if (vp.show_tf_tree && state_.transform_tree_source) {
    // ALWAYS query from the robot body frame, whatever subtree is to be shown:
    // the returned poses are relative to the queried root, and the result is
    // drawn at the vehicle pose below, so querying from any other frame would
    // offset the whole tree by the (unknown here) body -> root transform.
    // tf_tree_root_frame then only selects which subtree of it to draw.
    std::string queryRoot = state_.transform_tree_source->transform_tree_default_root();
    if (queryRoot.empty()) {
      queryRoot = vp.tf_tree_root_frame;
    }

    // Query at the scan time, so the joints match the rendered cloud rather
    // than the wall clock (the source falls back to its latest data itself).
    if (const auto tree =
          state_.transform_tree_source->transform_tree(queryRoot, state_.last_obs_timestamp);
        tree) {
      const size_t n = renderTfTree(
        *tree, vp.tf_tree_root_frame, splitCommaSeparated(vp.tf_tree_exclude_frames),
        vp.tf_tree_corner_size, vp.tf_tree_show_links, vp.tf_tree_link_radius,
        vp.tf_tree_show_names, *glTree);

      MRPT_LOG_THROTTLE_DEBUG_FMT(
        5.0, "[tf tree] Rendering %zu of %zu frame(s) under '%s'.", n, tree->nodes.size(),
        queryRoot.c_str());
    } else {
      MRPT_LOG_THROTTLE_WARN_FMT(
        5.0, "[tf tree] Root frame '%s' is not known to the data source.", queryRoot.c_str());
    }
  }

  // The tree poses are relative to the robot body, so draw it at the same
  // pose as the vehicle model:
  glTree->setPose(state_.last_lidar_pose.mean);
  updateTasks.emplace_back([visualizer = visualizer_, glTree, vizFrame]() {
    vizUpsert3D(visualizer, "liodom/tf_tree", glTree, vizFrame);
  });
#else
  (void)updateTasks;
  (void)vizFrame;
#endif
}

void LidarOdometry::updateVisualizationPath(std::vector<std::function<void()>> & updateTasks)
{
  if (!params_.visualization.show_trajectory) {
    auto empty = mrpt::viz::CSetOfObjects::Create();
    updateTasks.emplace_back([visualizer = visualizer_, empty, vizFrame = vizParentFrame()]() {
      vizUpsert3D(visualizer, "liodom/path", empty, vizFrame);
    });
    return;
  }

  const ProfilerEntry tle2(profiler_, "updateVisualization.update_traject");

  if (!state_.glEstimatedPath) {
    state_.glEstimatedPath = mrpt::viz::CSetOfLines::Create();
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
  auto pathGrp = mrpt::viz::CSetOfObjects::Create();
  pathGrp->insert(mrpt::viz::CSetOfLines::Create(*state_.glEstimatedPath));

  updateTasks.emplace_back([visualizer = visualizer_, pathGrp, vizFrame = vizParentFrame()]() {
    vizUpsert3D(visualizer, "liodom/path", pathGrp, vizFrame);
  });
}

void LidarOdometry::updateVisualizationGravityVector(
  std::vector<std::function<void()>> & updateTasks)
{
  const std::string vizFrame = vizParentFrame();
  if (!params_.visualization.show_gravity_align_vector) {
    auto grp = mrpt::viz::CSetOfObjects::Create();
    updateTasks.emplace_back([visualizer = visualizer_, grp, vizFrame]() {
      vizUpsert3D(visualizer, "liodom/gravity_vector", grp, vizFrame);
    });
    return;
  }

  const ProfilerEntry tle2(profiler_, "updateVisualization.update_gravity");

  const auto gravityPR = [this]() {
    auto lckImu = mrpt::lockHelper(imu_state_mtx_);
    return state_.gravity_estimator.estimatedPitchRoll(
      params_.imu_gravity_correction.averaging_samples,
      params_.imu_gravity_correction.max_age_seconds);
  }();

  if (!gravityPR.has_value()) {
    return;
  }

  const auto [imu_pitch, imu_roll] = *gravityPR;
  const auto & veh = state_.last_lidar_pose.mean;
  const auto arrowPose = mrpt::math::TPose3D(veh.x(), veh.y(), veh.z(), 0.0, imu_pitch, imu_roll);

  auto glArrow = mrpt::viz::CArrow::Create();
  glArrow->setArrowEnds(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f);
  glArrow->setColor_u8(0xff, 0xa5, 0x00, 0xdc);  // orange

  auto grp = mrpt::viz::CSetOfObjects::Create();
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
    // recent_imu_stamps is appended by the sensor-input thread:
    auto lckImu = mrpt::lockHelper(imu_state_mtx_);
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

  // The deciders are created lazily on the first processed scan, while the GUI
  // may refresh before that (or while initial localization is still pending):
  gui_.lbMapStats->set(mrpt::format(
    "Keyframes: Localmap=%zu, simplemap=%zu",
    state_.kf_decider_local_map ? state_.kf_decider_local_map->size() : 0u,
    state_.kf_decider_simplemap ? state_.kf_decider_simplemap->size() : 0u));

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
