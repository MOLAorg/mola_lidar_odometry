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
 * @file   MapFrameRelevel.h
 * @brief  Re-expresses map-frame data in a new reference frame
 * @author Jose Luis Blanco Claraco
 */
#pragma once

#include <mp2p_icp/metricmap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DInterpolator.h>

namespace mola
{
/** \defgroup mola_lo_map_frame_relevel Map-frame gauge change
 *
 *  A change of the map frame is a GAUGE change, not a state update: every
 *  quantity expressed in the map frame is left-composed with the same rigid
 *  transform `b`, so all relative poses, distances and angles between mapped
 *  entities are preserved exactly. Quantities expressed in the vehicle's own
 *  frame (body-frame twists, sensor extrinsics) are invariant and must NOT be
 *  touched.
 *
 *  The overloads below apply `p -> b + p` to each of the containers the
 *  odometry keeps in the map frame. They are free functions so that the
 *  invariant above is testable without a running odometry instance.
 *  @{ */

/** Applies `p -> b + p` to every pose of a trajectory. */
void transform_to_new_map_frame(
  mrpt::poses::CPose3DInterpolator & trajectory, const mrpt::poses::CPose3D & b);

/** Applies `p -> b + p` to every keyframe pose (mean and covariance) of a
 *  simplemap. The per-keyframe twists are in the vehicle frame and are left
 *  untouched.
 */
void transform_to_new_map_frame(mrpt::maps::CSimpleMap & sm, const mrpt::poses::CPose3D & b);

/** Applies `p -> b + p` to every layer of a metric map.
 *  \throw std::exception if any layer is of a type that cannot be transformed,
 *         or if the map carries geo-referencing (which the transform would
 *         silently invalidate).
 */
void transform_to_new_map_frame(mp2p_icp::metric_map_t & m, const mrpt::poses::CPose3D & b);

/** @} */

}  // namespace mola

/** Feature macro: mola_lidar_odometry provides the map-frame gauge-change
 *  helpers used by `imu_gravity_correction.map_gravity.relevel_map_frame`.
 */
#define MOLA_LO_HAS_MAP_FRAME_RELEVEL 1
