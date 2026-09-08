/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */
/**
 * @file   LidarOdometry_VisualPatches.cpp
 * @brief  Camera intake and map-anchored photometric patches.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 8, 2026
 */

#include <mola_lidar_odometry/LidarOdometry.h>

#if defined(MOLA_LO_HAS_MP2P_VISUAL_PATCHES)

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/CObservationImage.h>

#include <limits>
#include <utility>

using namespace mola;

void LidarOdometry::onImage(const CObservation::ConstPtr & o)
{
  MRPT_TRY_START
  const ProfilerEntry tle(profiler_, "onImage");

  const auto img = std::dynamic_pointer_cast<const mrpt::obs::CObservationImage>(o);
  if (!img) {
    MRPT_LOG_THROTTLE_WARN_FMT(
      5.0,
      "Observation '%s' matches the visual_patches camera label but is not a "
      "CObservationImage; ignoring it.",
      o->sensorLabel.c_str());
    return;
  }

  // Intrinsics: the observation's own, or the configured override for readers
  // that deliver images without their camera_info.
  mrpt::img::TCamera cam = img->cameraParams;
  if (const auto ovr = params_.visual_patches.cameraOverride(); ovr.has_value()) {
    cam = *ovr;
  }
  if (cam.fx() <= 0 || cam.fy() <= 0) {
    MRPT_LOG_THROTTLE_WARN(
      5.0,
      "Camera images carry no intrinsics and none were configured in "
      "visual_patches.camera_fx/fy/cx/cy: the photometric term stays inactive.");
    return;
  }

  // Grayscale here, on the input thread: it is per-image work that would
  // otherwise land on the LiDAR worker's critical path.
  mrpt::img::CImage gray;
  if (img->image.isColor()) {
    gray = img->image.grayscale();
  } else {
    gray = img->image;
  }
  // Force the conversion now, so nothing lazy happens under the lock later.
  gray.forceLoad();

  mrpt::poses::CPose3D poseOnVehicle = img->cameraPose;
  if (!params_.visual_patches.camera_pose_override.empty()) {
    poseOnVehicle = mrpt::poses::CPose3D::FromString(params_.visual_patches.camera_pose_override);
  }

  CameraFrame frame;
  frame.gray = gray;
  frame.camera = cam;
  frame.pose_on_vehicle = poseOnVehicle;
  frame.timestamp = mrpt::Clock::toDouble(img->timestamp);

  auto lck = mrpt::lockHelper(visual_image_mtx_);
  image_buffer_.push_back(std::move(frame));
  while (image_buffer_.size() > IMAGE_BUFFER_MAX) {
    image_buffer_.pop_front();
  }

  MRPT_TRY_END
}

std::optional<LidarOdometry::CameraFrame> LidarOdometry::selectImageForScan(double scanTime) const
{
  MRPT_TRY_START

  auto lck = mrpt::lockHelper(visual_image_mtx_);
  if (image_buffer_.empty()) {
    return {};
  }

  // Nearest in time, not newest: which frame is "newest" when a scan reaches
  // the worker depends on how far the input thread has run ahead.
  const CameraFrame * best = nullptr;
  double bestAge = std::numeric_limits<double>::max();
  for (const auto & f : image_buffer_) {
    const double age = std::abs(f.timestamp - scanTime);
    if (age < bestAge) {
      bestAge = age;
      best = &f;
    }
  }

  if (bestAge > params_.visual_patches.max_image_age) {
    // Logged, and not at debug level: a silent rejection here looks exactly
    // like a camera that is not wired up at all.
    MRPT_LOG_THROTTLE_WARN_FMT(
      10.0,
      "Nearest camera frame is %.1f ms from this scan, over the %.1f ms limit: "
      "no photometric term. Check the rig's LiDAR/camera phase and "
      "visual_patches.max_image_age.",
      1e3 * bestAge, 1e3 * params_.visual_patches.max_image_age);
    return {};
  }

  return *best;

  MRPT_TRY_END
  return {};
}

std::shared_ptr<const mp2p_icp::VisualPatchTerm> LidarOdometry::buildVisualPatchTerm(
  const CameraFrame & frame, const mrpt::poses::CPose3D & predictedVehiclePose) const
{
  MRPT_TRY_START

  const auto predictedCameraPose = predictedVehiclePose + frame.pose_on_vehicle;

  return state_.visual_patch_map.makeTerm(
    frame.gray, frame.camera, frame.pose_on_vehicle, predictedCameraPose);

  MRPT_TRY_END
  return {};
}

void LidarOdometry::captureVisualPatches(
  const CameraFrame & frame, const mrpt::poses::CPose3D & vehiclePose,
  const mp2p_icp::metric_map_t & observation)
{
  MRPT_TRY_START
  const ProfilerEntry tle(profiler_, "onLidar.visual_patches_capture");

  // Which layer of the scan may donate anchors. Anything with enough points
  // works: the z-buffer inside decides which of them actually become patches.
  const mrpt::maps::CPointsMap * pts = nullptr;
  const auto & wanted = params_.visual_patches.points_layer;
  for (const auto & [name, layer] : observation.layers) {
    if (!wanted.empty() && name != wanted) {
      continue;
    }
    const auto * asPts = dynamic_cast<const mrpt::maps::CPointsMap *>(layer.get());
    if (asPts && !asPts->empty()) {
      pts = asPts;
      break;
    }
  }
  if (!pts) {
    MRPT_LOG_THROTTLE_WARN_FMT(
      5.0, "No usable point layer%s in the scan to anchor visual patches on.",
      wanted.empty() ? "" : (" named '" + wanted + "'").c_str());
    return;
  }

  state_.visual_patch_map.captureFrom(
    frame.gray, frame.camera, frame.pose_on_vehicle, vehiclePose, *pts);

  MRPT_TRY_END
}

#endif  // MOLA_LO_HAS_MP2P_VISUAL_PATCHES
