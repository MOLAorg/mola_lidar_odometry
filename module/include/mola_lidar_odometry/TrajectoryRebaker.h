/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   TrajectoryRebaker.h
 * @brief  Chain-integration trajectory rebake using per-KF gravity corrections
 *
 * Design reference: TODOs/online-gravity-alignment.md §2.4.2.
 *
 * Takes a map of original ("odom") KF poses and a map of per-KF
 * pitch/roll corrections (from GravityMapAligner) and produces a new
 * map of corrected poses where:
 *
 *  - The anchor KF's position is frozen (unchanged); only its rotation
 *    is updated to R_grav_anchor * R_anchor_odom.
 *  - Each subsequent KF's position is the previous corrected position
 *    plus the odom relative displacement re-expressed in the corrected
 *    frame. Short-range odometry (relative motion between adjacent KFs)
 *    is therefore preserved.
 *  - Long-range accumulated pitch drift is redistributed along the chain.
 *
 * The class is stateless: all methods are static. Caller is responsible
 * for thread safety and for choosing the anchor KF.
 */
#pragma once

#include <mrpt/poses/CPose3D.h>

#include <cstdint>
#include <map>

namespace mola
{
/**
 * Trajectory rebaker: corrects a chain of KF poses using per-KF
 * pitch/roll rotations so that the resulting trajectory is
 * gravity-aligned.
 *
 * \ingroup mola_lidar_odometry_grp
 */
class TrajectoryRebaker
{
public:
  using KeyFrameID = uint64_t;

  /// Output of a rebake operation.
  struct Result
  {
    /// Corrected poses for every KF that could be processed (i.e. present in
    /// both odom_poses and r_grav, from anchor onward).
    std::map<KeyFrameID, mrpt::poses::CPose3D> corrected_poses;

    /// The anchor KF id used (first KF of the processed window).
    KeyFrameID anchor_id = 0;
  };

  /**
   * @brief Rebake a contiguous window of KF poses.
   *
   * Processes KFs in ascending id order within the intersection of
   * `odom_poses` and `r_grav`, starting at `anchor_id`.
   *
   * The anchor KF's POSITION is copied verbatim from `odom_poses`;
   * only its rotation is updated:
   *   T_anchor_new.R   = R_grav[anchor] * T_anchor_odom.R
   *   T_anchor_new.pos = T_anchor_odom.pos   (frozen)
   *
   * For every subsequent KF `i` (in ascending id order after anchor):
   *   Δpos_odom  = T_i_odom.pos - T_{i-1}_odom.pos
   *   Δpos_new   = (R_{i-1}_new * R_{i-1}_odom⁻¹) * Δpos_odom
   *   T_i_new.pos = T_{i-1}_new.pos + Δpos_new
   *   T_i_new.R   = R_grav[i] * T_i_odom.R
   *
   * KFs before `anchor_id` in `odom_poses` are skipped (their poses are
   * left unchanged by the caller). KFs in `odom_poses` that have no entry
   * in `r_grav` are passed through unchanged (identity correction applied).
   *
   * @param odom_poses  Original LIO poses keyed by KF id (sorted map).
   * @param r_grav      Per-KF pitch/roll corrections from GravityMapAligner.
   *                    KFs missing from this map get identity correction.
   * @param anchor_id   First KF id to process; its position is frozen.
   * @returns           Result with corrected_poses for anchor..end of window.
   */
  static Result rebake(
    const std::map<KeyFrameID, mrpt::poses::CPose3D> & odom_poses,
    const std::map<KeyFrameID, mrpt::poses::CPose3D> & r_grav, KeyFrameID anchor_id);

  /**
   * @brief Convenience overload: anchor = first KF present in both maps.
   */
  static Result rebake(
    const std::map<KeyFrameID, mrpt::poses::CPose3D> & odom_poses,
    const std::map<KeyFrameID, mrpt::poses::CPose3D> & r_grav);

  /**
   * @brief Compute the publish residual after a rebake.
   *
   * When a rebake changes the pose of the most-recent KF (id `tail_kf_id`),
   * the live per-scan pose needs a compensating transform so that the
   * published pose shows no jump:
   *
   *   T_grav_publish_new = T_tail_old * T_tail_new.inverse()
   *
   * Contract: applying the residual to the post-rebake tail pose reproduces
   * the pre-rebake tail pose, i.e. `residual + tail_after == tail_before`.
   * The caller LEFT-composes this onto the live (post-rebake) pose between
   * rebakes (`pose_pub = residual + pose_stored`) so the publish stream is
   * continuous across the rebake event.
   *
   * Returns identity if `tail_kf_id` is absent from either map.
   */
  static mrpt::poses::CPose3D computePublishResidual(
    const mrpt::poses::CPose3D & tail_pose_before, const mrpt::poses::CPose3D & tail_pose_after);
};

}  // namespace mola
