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

#include <mola_lidar_odometry/TrajectoryRebaker.h>

namespace mola
{
TrajectoryRebaker::Result TrajectoryRebaker::rebake(
  const std::map<KeyFrameID, mrpt::poses::CPose3D> & odom_poses,
  const std::map<KeyFrameID, mrpt::poses::CPose3D> & r_grav, KeyFrameID anchor_id)
{
  Result result;
  result.anchor_id = anchor_id;

  if (odom_poses.empty()) {
    return result;
  }

  // Make sure there's at least one KF >= anchor_id so the loop will enter the
  // anchor branch; otherwise corrected_poses would silently be empty while
  // result.anchor_id still claimed the requested id. Slide the anchor forward
  // to the first available id in that case, and report it back to the caller.
  auto anchor_it = odom_poses.lower_bound(anchor_id);
  if (anchor_it == odom_poses.end()) {
    return result;
  }
  anchor_id = anchor_it->first;
  result.anchor_id = anchor_id;

  // Identity correction for KFs absent from r_grav.
  const mrpt::poses::CPose3D identity = mrpt::poses::CPose3D::Identity();

  // Helper: look up r_grav or return identity.
  auto get_r_grav = [&](KeyFrameID id) -> const mrpt::poses::CPose3D & {
    auto it = r_grav.find(id);
    return (it != r_grav.end()) ? it->second : identity;
  };

  // Iterate odom_poses in ascending id order, starting from anchor_id.
  bool in_window = false;
  mrpt::poses::CPose3D prev_odom, prev_new;

  for (const auto & [id, T_odom] : odom_poses) {
    if (id < anchor_id) {
      continue;
    }

    const mrpt::poses::CPose3D & Rg = get_r_grav(id);

    if (!in_window) {
      // ---- Anchor KF: freeze position, correct rotation. ----
      // T_new.R   = R_grav * R_odom
      // T_new.pos = T_odom.pos  (unchanged)
      mrpt::poses::CPose3D T_new = Rg + T_odom;  // left-multiply: applies Rg rotation
      // Restore original position (Rg + T_odom shifts position; undo that):
      T_new.x(T_odom.x());
      T_new.y(T_odom.y());
      T_new.z(T_odom.z());

      result.corrected_poses[id] = T_new;

      prev_odom = T_odom;
      prev_new = T_new;
      in_window = true;
    } else {
      // ---- Subsequent KF: re-integrate relative displacement. ----
      //
      // Δpos_odom = T_i.pos - T_{i-1}.pos  (in odom frame)
      // The displacement in odom frame, re-expressed in corrected frame:
      //   Δpos_new = (R_{i-1}_new * R_{i-1}_odom⁻¹) * Δpos_odom
      //
      // R_{i-1}_odom⁻¹  maps odom vectors → body frame of KF i-1.
      // R_{i-1}_new      maps that body frame → corrected world frame.
      // Together they convert an odom displacement to corrected-world displacement.

      const mrpt::math::TVector3D delta_odom{
        T_odom.x() - prev_odom.x(), T_odom.y() - prev_odom.y(), T_odom.z() - prev_odom.z()};

      // Build C = R_{i-1}_new * R_{i-1}_odom⁻¹  as a pure-rotation CPose3D.
      // CPose3D stores rotation + translation; we only need the rotation part.
      // prev_new.R * prev_odom.R^T = prev_new * prev_odom.inverse() (rotation only).
      const mrpt::poses::CPose3D C = prev_new + (-prev_odom);  // R_new * R_odom^-1

      // Rotate the odom displacement into the corrected frame:
      const mrpt::math::TVector3D delta_new = C.rotateVector(delta_odom);

      // New corrected rotation: R_grav_i * R_i_odom.
      mrpt::poses::CPose3D T_new = Rg + T_odom;

      // Override position with the re-integrated one:
      T_new.x(prev_new.x() + delta_new.x);
      T_new.y(prev_new.y() + delta_new.y);
      T_new.z(prev_new.z() + delta_new.z);

      result.corrected_poses[id] = T_new;

      prev_odom = T_odom;
      prev_new = T_new;
    }
  }

  return result;
}

TrajectoryRebaker::Result TrajectoryRebaker::rebake(
  const std::map<KeyFrameID, mrpt::poses::CPose3D> & odom_poses,
  const std::map<KeyFrameID, mrpt::poses::CPose3D> & r_grav)
{
  // Three-arg overload already handles the empty-input case.
  const KeyFrameID anchor = odom_poses.empty() ? KeyFrameID{0} : odom_poses.begin()->first;
  return rebake(odom_poses, r_grav, anchor);
}

mrpt::poses::CPose3D TrajectoryRebaker::computePublishResidual(
  const mrpt::poses::CPose3D & tail_pose_before, const mrpt::poses::CPose3D & tail_pose_after)
{
  // T_grav_publish = T_tail_before.inverse() * T_tail_after
  // This is the transform the caller should left-compose onto the live pose
  // so that T_live_new + T_grav_publish_new == T_live_old + T_grav_publish_old.
  // Caller applies residual as: pose_pub = residual + pose_stored_new.
  // For continuity: residual ∘ T_new = T_old, so residual = T_old ∘ T_new⁻¹.
  return tail_pose_before + (-tail_pose_after);
}

}  // namespace mola
