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
 * @file   VisualPatchMap.cpp
 * @brief  Photometric patches anchored to LiDAR map points.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 8, 2026
 */

#include <mola_lidar_odometry/VisualPatchMap.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/string_utils.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

using namespace mola;

namespace
{
/// Bilinear sample of an 8-bit grayscale image. Bounds must be checked first.
double bilinear(const mrpt::img::CImage & im, double u, double v)
{
  const int x0 = static_cast<int>(std::floor(u));
  const int y0 = static_cast<int>(std::floor(v));
  const double ax = u - x0;
  const double ay = v - y0;

  const auto * r0 = im.ptrLine<uint8_t>(static_cast<unsigned int>(y0));
  const auto * r1 = im.ptrLine<uint8_t>(static_cast<unsigned int>(y0 + 1));

  return (1 - ay) * ((1 - ax) * r0[x0] + ax * r0[x0 + 1]) +
         ay * ((1 - ax) * r1[x0] + ax * r1[x0 + 1]);
}

double dotProduct(const mrpt::math::TVector3D & a, const mrpt::math::TVector3D & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
}  // namespace

void VisualPatchMap::Parameters::initialize(const mrpt::containers::yaml & c)
{
  MCP_LOAD_OPT(c, enabled);
  MCP_LOAD_OPT(c, camera_sensor_label);
  MCP_LOAD_OPT(c, camera_pose_override);
  MCP_LOAD_OPT(c, half_size);
  MCP_LOAD_OPT(c, voxel_size);
  MCP_LOAD_OPT(c, max_patches);
  MCP_LOAD_OPT(c, max_patches_per_frame);
  MCP_LOAD_OPT(c, max_image_age);
  MCP_LOAD_OPT(c, min_gradient);
  MCP_LOAD_OPT(c, occlusion_cell_px);
  MCP_LOAD_OPT(c, recapture_angle_deg);
  MCP_LOAD_OPT(c, max_view_angle_deg);
  MCP_LOAD_OPT(c, min_depth);
  MCP_LOAD_OPT(c, max_depth);
  MCP_LOAD_OPT(c, prune_radius);
  MCP_LOAD_OPT(c, sigma_intensity);
  MCP_LOAD_OPT(c, huber_delta);
  MCP_LOAD_OPT(c, max_rms_sigmas);
  MCP_LOAD_OPT(c, weight);
  MCP_LOAD_OPT(c, auto_balance);
  MCP_LOAD_OPT(c, effective_pixels_per_patch);
  MCP_LOAD_OPT(c, max_information_share);
  MCP_LOAD_OPT(c, estimate_normals);
  MCP_LOAD_OPT(c, normal_knn);
  MCP_LOAD_OPT(c, normal_max_planarity_ratio);
  MCP_LOAD_OPT(c, estimate_gain);
  MCP_LOAD_OPT(c, max_gain);
  MCP_LOAD_OPT(c, pyramid_levels);
  MCP_LOAD_OPT(c, scale_window);
  MCP_LOAD_OPT(c, scale_min_samples);
  MCP_LOAD_OPT(c, points_layer);
  MCP_LOAD_OPT(c, camera_fx);
  MCP_LOAD_OPT(c, camera_fy);
  MCP_LOAD_OPT(c, camera_cx);
  MCP_LOAD_OPT(c, camera_cy);
  MCP_LOAD_OPT(c, camera_ncols);
  MCP_LOAD_OPT(c, camera_nrows);
  MCP_LOAD_OPT(c, camera_distortion);
  MCP_LOAD_OPT(c, camera_distortion_coeffs);

  ASSERT_GT_(voxel_size, 0.0);
  ASSERT_GT_(half_size, 0U);
  ASSERT_GT_(sigma_intensity, 0.0);
}

std::optional<mrpt::img::TCamera> VisualPatchMap::Parameters::cameraOverride() const
{
  if (camera_fx <= 0 || camera_fy <= 0) {
    return {};
  }
  mrpt::img::TCamera cam;
  cam.ncols = camera_ncols;
  cam.nrows = camera_nrows;
  cam.intrinsicParams.setZero();
  cam.intrinsicParams(0, 0) = camera_fx;
  cam.intrinsicParams(1, 1) = camera_fy;
  cam.intrinsicParams(0, 2) = camera_cx;
  cam.intrinsicParams(1, 2) = camera_cy;
  cam.intrinsicParams(2, 2) = 1.0;

  if (camera_distortion == "none" || camera_distortion.empty()) {
    cam.distortion = mrpt::img::DistortionModel::none;
  } else if (camera_distortion == "plumb_bob") {
    cam.distortion = mrpt::img::DistortionModel::plumb_bob;
  } else if (camera_distortion == "kannala_brandt") {
    cam.distortion = mrpt::img::DistortionModel::kannala_brandt;
  } else {
    THROW_EXCEPTION_FMT(
      "visual_patches.camera_distortion must be one of none|plumb_bob|kannala_brandt, got '%s'",
      camera_distortion.c_str());
  }

  cam.dist.fill(0);
  std::vector<std::string> tokens;
  mrpt::system::tokenize(camera_distortion_coeffs, " \t,", tokens);
  if (cam.distortion == mrpt::img::DistortionModel::kannala_brandt) {
    // MRPT packs the four fisheye coefficients as [k1 k2 * * k3 k4].
    ASSERTMSG_(
      tokens.size() == 4,
      "kannala_brandt needs exactly 4 coefficients in camera_distortion_coeffs");
    cam.dist[0] = std::stod(tokens[0]);
    cam.dist[1] = std::stod(tokens[1]);
    cam.dist[4] = std::stod(tokens[2]);
    cam.dist[5] = std::stod(tokens[3]);
  } else {
    ASSERTMSG_(tokens.size() <= 8, "at most 8 coefficients in camera_distortion_coeffs");
    for (size_t i = 0; i < tokens.size(); i++) {
      cam.dist[i] = std::stod(tokens[i]);
    }
  }
  return cam;
}

void VisualPatchMap::pushScaleSample(double instant)
{
  if (!(instant > 0) || !std::isfinite(instant)) {
    return;
  }
  scale_samples_.push_back(instant);
  while (scale_samples_.size() > params.scale_window) {
    scale_samples_.pop_front();
  }
}

double VisualPatchMap::scaleHint() const
{
  if (scale_samples_.size() < params.scale_min_samples) {
    return -1.0;
  }
  std::vector<double> v(scale_samples_.begin(), scale_samples_.end());
  const size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  return v[mid];
}

VisualPatchMap::voxel_key_t VisualPatchMap::keyOf(const mrpt::math::TPoint3D & p) const
{
  const double inv = 1.0 / params.voxel_size;
  // 21 bits per axis: +/- 2^20 voxels, i.e. hundreds of km at any sane size.
  const auto ix = static_cast<int64_t>(std::floor(p.x * inv)) & 0x1FFFFF;
  const auto iy = static_cast<int64_t>(std::floor(p.y * inv)) & 0x1FFFFF;
  const auto iz = static_cast<int64_t>(std::floor(p.z * inv)) & 0x1FFFFF;
  return ix | (iy << 21) | (iz << 42);
}

std::shared_ptr<const mp2p_icp::VisualPatchTerm> VisualPatchMap::makeTerm(
  const mrpt::img::CImage & grayImage, const mrpt::img::TCamera & camera,
  const mrpt::poses::CPose3D & cameraPoseOnVehicle,
  const mrpt::poses::CPose3D & predictedCameraPose) const
{
  MRPT_START

  if (patches_.empty()) {
    return {};
  }

  const int W = static_cast<int>(grayImage.getWidth());
  const int H = static_cast<int>(grayImage.getHeight());
  const int half = static_cast<int>(params.half_size);
  const double margin = half + 6;

  const double cosMaxView = std::cos(mrpt::DEG2RAD(params.max_view_angle_deg));

  // Keep the best candidate of each image cell, so the patches handed to the
  // solver are spread over the frame instead of piling up on one textured
  // object. A poorly spread set constrains rotation far worse for the same
  // number of residuals.
  const double cellPx = std::max(8.0, params.occlusion_cell_px);
  const int nCx = static_cast<int>(std::ceil(W / cellPx));
  const int nCy = static_cast<int>(std::ceil(H / cellPx));

  struct Cand
  {
    const StoredPatch * sp = nullptr;
    double score = -2.0;  // cos of the view-angle change: larger is better
  };
  std::vector<Cand> best(static_cast<size_t>(nCx) * nCy);

  const auto camOrigin = predictedCameraPose.translation();

  for (const auto & [key, sp] : patches_) {
    const auto p_c = predictedCameraPose.inverseComposePoint(sp.patch.pt_global);
    if (p_c.z < params.min_depth || p_c.z > params.max_depth) {
      continue;
    }
    const auto px = mp2p_icp::projectToPixel(camera, p_c, params.min_depth);
    if (!px) {
      continue;
    }
    if (px->x < margin || px->y < margin || px->x > W - 1 - margin || px->y > H - 1 - margin) {
      continue;
    }

    mrpt::math::TVector3D viewDir = camOrigin - sp.patch.pt_global;
    const double n = viewDir.norm();
    if (n < 1e-6) {
      continue;
    }
    viewDir *= 1.0 / n;

    const double cosAng = dotProduct(viewDir, sp.ref_view_dir);
    if (cosAng < cosMaxView) {
      continue;
    }

    const int cx = std::min(nCx - 1, static_cast<int>(px->x / cellPx));
    const int cy = std::min(nCy - 1, static_cast<int>(px->y / cellPx));
    auto & slot = best[static_cast<size_t>(cy) * nCx + cx];
    if (cosAng > slot.score) {
      slot.score = cosAng;
      slot.sp = &sp;
    }
  }

  std::vector<const StoredPatch *> chosen;
  chosen.reserve(best.size());
  for (const auto & b : best) {
    if (b.sp) {
      chosen.push_back(b.sp);
    }
  }
  if (chosen.empty()) {
    return {};
  }

  // If the frame is richer than the budget, keep the best-viewed ones.
  if (chosen.size() > params.max_patches_per_frame) {
    std::partial_sort(
      chosen.begin(), chosen.begin() + params.max_patches_per_frame, chosen.end(),
      [&](const StoredPatch * a, const StoredPatch * b) {
        const auto da = camOrigin - a->patch.pt_global;
        const auto db = camOrigin - b->patch.pt_global;
        return dotProduct(da, a->ref_view_dir) / std::max(1e-9, da.norm()) >
               dotProduct(db, b->ref_view_dir) / std::max(1e-9, db.norm());
      });
    chosen.resize(params.max_patches_per_frame);
  }

  stats.terms_built++;
  stats.patches_offered += chosen.size();

  auto term = std::make_shared<mp2p_icp::VisualPatchTerm>();
  term->image = grayImage;
  term->camera = camera;
  term->camera_pose_on_local = cameraPoseOnVehicle;
  term->half_size = params.half_size;
  term->sigma_intensity = params.sigma_intensity;
  term->huber_delta = params.huber_delta;
  term->max_rms_sigmas = params.max_rms_sigmas;
  term->min_depth = params.min_depth;
  term->weight = params.weight;
  term->auto_balance = params.auto_balance;
  term->effective_pixels_per_patch = params.effective_pixels_per_patch;
  term->max_information_share = params.max_information_share;
  term->estimate_gain = params.estimate_gain;
  term->max_gain = params.max_gain;
  term->pyramid_levels = params.pyramid_levels;
  term->buildPyramid(params.pyramid_levels);
  term->scale_hint = scaleHint();
  term->patches.reserve(chosen.size());
  for (const auto * sp : chosen) {
    term->patches.push_back(sp->patch);
  }

  return term;

  MRPT_END
}

void VisualPatchMap::captureFrom(
  const mrpt::img::CImage & grayImage, const mrpt::img::TCamera & camera,
  const mrpt::poses::CPose3D & cameraPoseOnVehicle, const mrpt::poses::CPose3D & vehiclePose,
  const mrpt::maps::CPointsMap & points)
{
  MRPT_START

  const size_t nPts = points.size();
  if (nPts == 0) {
    return;
  }

  const mrpt::poses::CPose3D camPose = vehiclePose + cameraPoseOnVehicle;
  const auto camOrigin = camPose.translation();

  const int W = static_cast<int>(grayImage.getWidth());
  const int H = static_cast<int>(grayImage.getHeight());
  const int half = static_cast<int>(params.half_size);
  const double margin = half + 6;

  const double cellPx = std::max(4.0, params.occlusion_cell_px);
  const int nCx = static_cast<int>(std::ceil(W / cellPx));
  const int nCy = static_cast<int>(std::ceil(H / cellPx));

  // Coarse z-buffer: only the nearest candidate of each cell survives, which
  // both removes points seen through a nearer surface and decimates the scan
  // down to a manageable number of anchors.
  struct Cell
  {
    double depth = std::numeric_limits<double>::max();
    size_t idx = 0;
    bool used = false;
    float px = 0, py = 0;
  };
  std::vector<Cell> cells(static_cast<size_t>(nCx) * nCy);

  const auto & xs = points.getPointsBufferRef_x();
  const auto & ys = points.getPointsBufferRef_y();
  const auto & zs = points.getPointsBufferRef_z();

  for (size_t i = 0; i < nPts; i++) {
    // Scan points arrive in the vehicle frame; the camera frame is one more
    // composition away, and the global anchor one after that.
    const mrpt::math::TPoint3D pVeh(xs[i], ys[i], zs[i]);
    const auto p_c = cameraPoseOnVehicle.inverseComposePoint(pVeh);
    if (p_c.z < params.min_depth || p_c.z > params.max_depth) {
      continue;
    }
    const auto px = mp2p_icp::projectToPixel(camera, p_c, params.min_depth);
    if (!px) {
      continue;
    }
    if (px->x < margin || px->y < margin || px->x > W - 1 - margin || px->y > H - 1 - margin) {
      continue;
    }
    const int cx = std::min(nCx - 1, static_cast<int>(px->x / cellPx));
    const int cy = std::min(nCy - 1, static_cast<int>(px->y / cellPx));
    auto & cell = cells[static_cast<size_t>(cy) * nCx + cx];
    if (p_c.z < cell.depth) {
      cell.depth = p_c.z;
      cell.idx = i;
      cell.used = true;
      cell.px = px->x;
      cell.py = px->y;
    }
  }

  // The reference pyramid is built once per frame and shared by every patch
  // captured from it.
  std::vector<mrpt::img::CImage> refPyramid;
  refPyramid.push_back(grayImage);
  for (uint32_t lv = 1; lv < params.pyramid_levels; lv++) {
    const auto & prev = refPyramid.back();
    if (prev.getWidth() < 32 || prev.getHeight() < 32) {
      break;
    }
    refPyramid.push_back(prev.scaleHalf(mrpt::img::IMG_INTERP_LINEAR));
  }

  const double cosRecapture = std::cos(mrpt::DEG2RAD(params.recapture_angle_deg));
  const size_t nPix = static_cast<size_t>(2 * half + 1) * static_cast<size_t>(2 * half + 1);

  std::vector<float> patchBuf(nPix);

  for (const auto & cell : cells) {
    if (!cell.used) {
      continue;
    }
    const mrpt::math::TPoint3D pVeh(xs[cell.idx], ys[cell.idx], zs[cell.idx]);
    const mrpt::math::TPoint3D pGlobal = vehiclePose.composePoint(pVeh);

    mrpt::math::TVector3D viewDir = camOrigin - pGlobal;
    const double vn = viewDir.norm();
    if (vn < 1e-6) {
      continue;
    }
    viewDir *= 1.0 / vn;

    const auto key = keyOf(pGlobal);
    auto it = patches_.find(key);
    const bool exists = it != patches_.end();
    if (exists && dotProduct(viewDir, it->second.ref_view_dir) > cosRecapture) {
      // The stored appearance still predicts this view: leave it alone, so a
      // patch keeps the long baseline that makes it worth more than a track.
      continue;
    }

    // Sample the patch at full resolution and measure its texture there.
    double gradSum = 0;
    size_t k = 0;
    for (int dy = -half; dy <= half; dy++) {
      for (int dx = -half; dx <= half; dx++, k++) {
        const double u = cell.px + dx;
        const double v = cell.py + dy;
        patchBuf[k] = static_cast<float>(bilinear(grayImage, u, v));
        gradSum += std::abs(bilinear(grayImage, u + 1, v) - bilinear(grayImage, u - 1, v)) +
                   std::abs(bilinear(grayImage, u, v + 1) - bilinear(grayImage, u, v - 1));
      }
    }
    if (0.25 * gradSum / static_cast<double>(nPix) < params.min_gradient) {
      continue;
    }

    StoredPatch sp;
    sp.patch.pt_global = pGlobal;
    sp.patch.ref_camera_pose = camPose;
    sp.ref_view_dir = viewDir;

    // A real surface normal from the LiDAR neighborhood, which is what the
    // affine warp is built on. Without it the patch is taken fronto-parallel
    // to the reference camera, and the warp is then wrong by the surface's
    // actual slant: a systematic residual that no photometric noise covers.
    if (params.estimate_normals) {
      const auto n = estimateNormal(points, pVeh);
      if (n) {
        // Rotate into the map frame; the sign is fixed later against the view.
        const auto R = vehiclePose.getRotationMatrix();
        sp.patch.normal_global = mrpt::math::TVector3D(
          R(0, 0) * n->x + R(0, 1) * n->y + R(0, 2) * n->z,
          R(1, 0) * n->x + R(1, 1) * n->y + R(1, 2) * n->z,
          R(2, 0) * n->x + R(2, 1) * n->y + R(2, 2) * n->z);
      }
    }
    sp.patch.ref_patches.emplace_back(patchBuf.begin(), patchBuf.end());

    // One more reference patch per coarser level, cut from the reduced
    // reference image rather than from a decimation of the fine patch: the
    // current image will be sampled the same way, and the two have to match.
    bool levelsOk = true;
    for (size_t lv = 1; lv < refPyramid.size(); lv++) {
      const auto & im = refPyramid[lv];
      const double sc = 1.0 / static_cast<double>(1u << lv);
      const double cu = cell.px * sc;
      const double cv = cell.py * sc;
      if (
        cu < half + 2 || cv < half + 2 || cu > static_cast<double>(im.getWidth()) - half - 3 ||
        cv > static_cast<double>(im.getHeight()) - half - 3) {
        levelsOk = false;
        break;
      }
      std::vector<float> lvPatch(nPix);
      size_t kk = 0;
      for (int dy = -half; dy <= half; dy++) {
        for (int dx = -half; dx <= half; dx++, kk++) {
          lvPatch[kk] = static_cast<float>(bilinear(im, cu + dx, cv + dy));
        }
      }
      sp.patch.ref_patches.push_back(std::move(lvPatch));
    }
    if (!levelsOk) {
      continue;
    }

    if (exists) {
      it->second = std::move(sp);
      stats.patches_refreshed++;
    } else {
      patches_.emplace(key, std::move(sp));
      stats.patches_created++;
    }
  }

  stats.frames_captured++;
  prune(camOrigin);

  MRPT_END
}

std::optional<mrpt::math::TVector3D> VisualPatchMap::estimateNormal(
  const mrpt::maps::CPointsMap & points, const mrpt::math::TPoint3D & p) const
{
  std::vector<float> nx, ny, nz, dist2;
  std::vector<size_t> idx;
  points.kdTreeNClosestPoint3DWithIdx(
    static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z), params.normal_knn,
    nx, ny, nz, idx, dist2);

  if (nx.size() < 4) {
    return {};
  }

  // Plane through the neighborhood, as the eigenvector of the smallest
  // eigenvalue of its scatter matrix.
  double cx = 0, cy = 0, cz = 0;
  for (size_t i = 0; i < nx.size(); i++) {
    cx += nx[i];
    cy += ny[i];
    cz += nz[i];
  }
  const double inv = 1.0 / static_cast<double>(nx.size());
  cx *= inv;
  cy *= inv;
  cz *= inv;

  Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < nx.size(); i++) {
    const Eigen::Vector3d d(nx[i] - cx, ny[i] - cy, nz[i] - cz);
    C.noalias() += d * d.transpose();
  }
  C *= inv;

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
  const auto & ev = es.eigenvalues();  // ascending
  if (!(ev[1] > 1e-12) || ev[0] / ev[1] > params.normal_max_planarity_ratio) {
    // Not planar enough for a normal to mean anything: a corner, an edge, or
    // foliage. Better no normal than a confidently wrong one.
    return {};
  }
  const auto v = es.eigenvectors().col(0);
  return mrpt::math::TVector3D(v.x(), v.y(), v.z());
}

void VisualPatchMap::prune(const mrpt::math::TPoint3D & cameraOrigin)
{
  // Drop what the camera has left behind first: those can never be scored
  // again, and they are what makes the store grow without bound.
  const double r2 = params.prune_radius * params.prune_radius;
  for (auto it = patches_.begin(); it != patches_.end();) {
    const auto d = it->second.patch.pt_global - cameraOrigin;
    if (d.sqrNorm() > r2) {
      it = patches_.erase(it);
      stats.patches_pruned++;
    } else {
      ++it;
    }
  }

  if (patches_.size() <= params.max_patches) {
    return;
  }

  // Still over the cap: drop the farthest, in one pass, down to 80% of it so
  // this does not run on every frame.
  std::vector<std::pair<double, voxel_key_t>> byDist;
  byDist.reserve(patches_.size());
  for (const auto & [key, sp] : patches_) {
    byDist.emplace_back((sp.patch.pt_global - cameraOrigin).sqrNorm(), key);
  }
  const size_t keep = static_cast<size_t>(0.8 * params.max_patches);
  std::nth_element(
    byDist.begin(), byDist.begin() + keep, byDist.end(),
    [](const auto & a, const auto & b) { return a.first < b.first; });
  for (size_t i = keep; i < byDist.size(); i++) {
    patches_.erase(byDist[i].second);
    stats.patches_pruned++;
  }
}
