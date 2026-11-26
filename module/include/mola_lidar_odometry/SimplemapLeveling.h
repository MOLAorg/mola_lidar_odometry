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
 * @file   SimplemapLeveling.h
 * @brief  Moves simplemap keyframes with the correction of a leveled path
 * @author Jose Luis Blanco Claraco
 */
#pragma once

#include <mrpt/core/Clock.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3DInterpolator.h>

#include <cstddef>
#include <optional>

namespace mola
{
/** Stamp of a simplemap keyframe: its "metadata" observation, which carries
 *  the scan reference time the keyframe pose refers to (LidarOdometry),
 *  else its first observation with a valid stamp. sm-cli export-keyframes
 *  always takes the first observation (the scan's own stamp), which may
 *  differ by up to one scan period.
 */
std::optional<mrpt::Clock::time_point> keyframe_stamp(const mrpt::obs::CSensoryFrame & sf);

/** Moves every keyframe pose of `sm` by the correction a leveled path had
 *  at the keyframe's stamp: after(t) * before(t)^-1, both interpolated, each
 *  held at its own ends. The keyframes and both paths must be in the
 *  same frame: the correction is a rotation about that frame's axes. Keyframes
 *  without a stamp are left as they are. Pose covariances are rotated with
 *  the poses. Returns how many keyframes moved.
 */
std::size_t level_simplemap_keyframes(
  mrpt::maps::CSimpleMap & sm, const mrpt::poses::CPose3DInterpolator & before,
  const mrpt::poses::CPose3DInterpolator & after);

}  // namespace mola
