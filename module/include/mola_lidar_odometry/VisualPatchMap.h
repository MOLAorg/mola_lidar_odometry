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
 * @file   VisualPatchMap.h
 * @brief  Photometric patches anchored to LiDAR map points.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 8, 2026
 */
#pragma once

#include <mp2p_icp/VisualPatches.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/poses/CPose3D.h>

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

namespace mola
{
/** A store of photometric patches anchored to points of the LiDAR map.
 *
 * This is the map-anchored counterpart of a visual landmark map: the 3D
 * position of each patch belongs to the LiDAR map and is never estimated from
 * images, so the camera contributes an observation OF THE MAP instead of a
 * second trajectory to be fused with the first one.
 *
 * One patch is kept per voxel, captured the first time the anchor is seen and
 * recaptured when the viewpoint has moved far enough that the stored
 * appearance no longer predicts the new one.
 */
class VisualPatchMap
{
public:
  VisualPatchMap() = default;

  struct Parameters
  {
    /// Master switch. Everything below is inert while this is false.
    bool enabled = false;

    /// Regex matching the sensor label of the camera to use.
    std::string camera_sensor_label = "camera.*";

    /// Overrides the camera pose on the vehicle carried by the observation.
    /// Format: "[x y z yaw_deg pitch_deg roll_deg]". Empty: use the
    /// observation's own `cameraPose`. The camera frame is the optical one:
    /// +Z along the optical axis, +X right, +Y down.
    std::string camera_pose_override;

    /// Half-size of the square patches, in pixels. Each patch is
    /// (2*half_size+1)^2 samples.
    uint32_t half_size = 3;

    /// Edge [m] of the voxel that holds at most one patch. This, and not a
    /// feature detector, is what spreads the patches over the scene.
    double voxel_size = 0.75;

    /// Maximum number of patches kept in the store.
    uint32_t max_patches = 4000;

    /// Maximum number of patches handed to one ICP solve.
    uint32_t max_patches_per_frame = 250;

    /// Maximum |t_image - t_scan| [s] for an image to be used at all. A stale
    /// image would be scored against the wrong pose.
    double max_image_age = 0.04;

    /// Minimum mean absolute gradient [gray levels/px] for a patch to be
    /// captured. Texture-free patches cost time and carry no information.
    double min_gradient = 8.0;

    /// Edge [px] of the cell used as a coarse z-buffer when capturing, so that
    /// only the nearest candidate of each image region becomes a patch. This
    /// is the occlusion filter, and it also decimates.
    double occlusion_cell_px = 12.0;

    /// A stored patch is recaptured once the direction from which its anchor
    /// is seen has changed by more than this [deg].
    double recapture_angle_deg = 20.0;

    /// Patches seen from more than this [deg] away from their reference view
    /// are not used: the warp no longer predicts their appearance.
    double max_view_angle_deg = 40.0;

    /// Depth range [m] in the camera frame for capture and use.
    double min_depth = 0.8;
    double max_depth = 40.0;

    /// Patches farther than this [m] from the current camera are dropped.
    double prune_radius = 80.0;

    /// Photometric noise model and robustness, see mp2p_icp::VisualPatchTerm.
    double sigma_intensity = 12.0;
    double huber_delta = 2.0;
    double max_rms_sigmas = 6.0;
    double weight = 1.0;

    /// Set the term's scale from the data rather than from `weight`, by
    /// matching its chi-square per DOF to the LiDAR block's. Makes the result
    /// independent of `sigma_intensity`. See mp2p_icp::VisualPatchTerm.
    bool auto_balance = true;
    /// Independent residuals per patch; NOT the pixel count. Measured ~8 of 49
    /// for a 7x7 patch. Measure with MP2P_ICP_VISUAL_RESIDUAL_FILE.
    double effective_pixels_per_patch = 8.0;
    /// Safety rail on the automatic scale, not a tuning knob.
    /// Rail on auto_balance, as the largest information share the camera may
    /// take. A rail, not a target: measured shares are 0.005 to 0.18.
    double max_information_share = 0.5;

    /// Estimate a real surface normal for each anchor from its LiDAR
    /// neighborhood, instead of assuming the patch fronto-parallel to the
    /// reference camera. The warp is only as good as this normal.
    /// DEFAULT OFF on measurement: it is what the leader does, and it does
    /// reject grazing surfaces correctly, but on GrandTour it tripled the
    /// patch rejection rate without moving the photometric chi-square, and
    /// cost accuracy on both missions.
    bool estimate_normals = false;
    /// Neighbors used for that plane fit, and the planarity it must reach
    /// (smallest eigenvalue over the middle one).
    uint32_t normal_knn = 10;
    double normal_max_planarity_ratio = 0.15;

    /// Estimate one photometric gain per frame; see mp2p_icp::VisualPatchTerm.
    bool estimate_gain = false;
    double max_gain = 3.0;

    /// Pyramid levels for the coarse-to-fine photometric solve. One level is
    /// the behavior of no pyramid at all.
    uint32_t pyramid_levels = 3;

    /// Samples of the per-scan calibration ratio kept for the running estimate
    /// that actually sets the scale, and how many are needed before it is
    /// trusted. Reacting to each frame's own residuals was measured to be
    /// harmful; the ratio is a property of the sensor pair, not of a frame.
    uint32_t scale_window = 200;
    uint32_t scale_min_samples = 30;

    /// Intrinsics to use when the image observation carries none, which is
    /// the case for any rosbag reader that does not pair images with their
    /// camera_info. Left at zero, the observation's own values are used.
    double camera_fx = 0;
    double camera_fy = 0;
    double camera_cx = 0;
    double camera_cy = 0;
    uint32_t camera_ncols = 0;
    uint32_t camera_nrows = 0;
    /// One of: none, plumb_bob, kannala_brandt.
    std::string camera_distortion = "none";
    /// Distortion coefficients, space separated, in the order the model above
    /// expects: [k1 k2 p1 p2 k3] for plumb_bob, [k1 k2 k3 k4] for fisheye.
    std::string camera_distortion_coeffs;

    /// Name of the observation layer whose points may become anchors. Empty
    /// means the first point layer found, in layer-name order.
    std::string points_layer;

    void initialize(const mrpt::containers::yaml & c);

    /// The configured intrinsics, or nullopt when none were given.
    [[nodiscard]] std::optional<mrpt::img::TCamera> cameraOverride() const;
  };

  Parameters params;

  /** Builds the term for one ICP solve: the stored patches predicted visible
   *  from `predictedCameraPose`, in the global frame.
   *
   * \return An empty pointer when the store has nothing usable to offer.
   */
  std::shared_ptr<const mp2p_icp::VisualPatchTerm> makeTerm(
    const mrpt::img::CImage & grayImage, const mrpt::img::TCamera & camera,
    const mrpt::poses::CPose3D & cameraPoseOnVehicle,
    const mrpt::poses::CPose3D & predictedCameraPose) const;

  /** Captures new patches, and refreshes stale ones, from a registered scan.
   *
   * \param points Scan points in the VEHICLE frame.
   * \param vehiclePose The registered pose of that frame in the global map.
   */
  void captureFrom(
    const mrpt::img::CImage & grayImage, const mrpt::img::TCamera & camera,
    const mrpt::poses::CPose3D & cameraPoseOnVehicle, const mrpt::poses::CPose3D & vehiclePose,
    const mrpt::maps::CPointsMap & points);

  size_t size() const { return patches_.size(); }
  void clear() { patches_.clear(); }

  /** Feeds the running estimate of the calibration ratio with one solve's own
   *  value (mp2p_icp::Results::visual_auto_scale_instant). */
  void pushScaleSample(double instant);

  /** The running estimate handed to the solver, or <=0 while too few samples
   *  have accumulated. The median, not the mean: a single frame whose
   *  photometric fit collapses must not move it. */
  [[nodiscard]] double scaleHint() const;

  /// Counters for the run summary, since the term itself only reports what one
  /// solve saw.
  struct Stats
  {
    uint64_t frames_captured = 0;
    uint64_t patches_created = 0;
    uint64_t patches_refreshed = 0;
    uint64_t patches_pruned = 0;
    uint64_t terms_built = 0;
    uint64_t patches_offered = 0;
  };
  mutable Stats stats;

private:
  /// Voxel index of an anchor, packed into one integer key.
  using voxel_key_t = int64_t;

  voxel_key_t keyOf(const mrpt::math::TPoint3D & p) const;

  struct StoredPatch
  {
    mp2p_icp::visual_patch_t patch;
    /// Unit direction from the anchor towards the reference camera, cached to
    /// keep the view-angle tests off the pose arithmetic.
    mrpt::math::TVector3D ref_view_dir;
  };

  std::unordered_map<voxel_key_t, StoredPatch> patches_;

  std::deque<double> scale_samples_;

  /// The LiDAR surface normal at `p` (VEHICLE frame), or nullopt when its
  /// neighborhood is not planar enough for one to mean anything.
  [[nodiscard]] std::optional<mrpt::math::TVector3D> estimateNormal(
    const mrpt::maps::CPointsMap & points, const mrpt::math::TPoint3D & p) const;

  void prune(const mrpt::math::TPoint3D & cameraOrigin);
};

}  // namespace mola
