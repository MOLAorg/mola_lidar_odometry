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
 * @file   MapFrameRelevel.cpp
 * @brief  Re-expresses map-frame data in a new reference frame
 * @author Jose Luis Blanco Claraco
 */

#include <mola_lidar_odometry/MapFrameRelevel.h>
#include <mp2p_icp/MetricMapMergeCapable.h>
#include <mrpt/maps/CPointsMap.h>

namespace mola
{

void transform_to_new_map_frame(
  mrpt::poses::CPose3DInterpolator & trajectory, const mrpt::poses::CPose3D & b)
{
  for (auto & [t, p] : trajectory) {
    p = (b + mrpt::poses::CPose3D(p)).asTPose();
  }
}

void transform_to_new_map_frame(mrpt::maps::CSimpleMap & sm, const mrpt::poses::CPose3D & b)
{
  for (auto & [pose, sf, twist] : sm) {
    // Transforms both the mean and the covariance:
    pose->changeCoordinatesReference(b);
    // `twist` is in the vehicle frame, hence invariant: left as is.
  }
}

void transform_to_new_map_frame(mp2p_icp::metric_map_t & m, const mrpt::poses::CPose3D & b)
{
  ASSERTMSG_(
    !m.georeferencing.has_value(),
    "Cannot change the map frame of a geo-referenced map: the georeferencing "
    "transform would no longer describe it.");

  // Only generic layers are used by the odometry; refuse silently dropping
  // anything else:
  ASSERTMSG_(
    m.lines.empty() && m.planes.empty(),
    "Cannot change the map frame of a map holding lines or planes.");

  for (auto & [name, layer] : m.layers) {
    if (auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(layer); pts) {
      pts->changeCoordinatesReference(b);
      continue;
    }
    if (auto mrg = std::dynamic_pointer_cast<mp2p_icp::MetricMapMergeCapable>(layer); mrg) {
      mrg->transform_map_left_multiply(b);
      continue;
    }
    THROW_EXCEPTION_FMT(
      "Map layer '%s' (class '%s') does not support changing its reference frame.", name.c_str(),
      layer->GetRuntimeClass()->className);
  }
}

}  // namespace mola
