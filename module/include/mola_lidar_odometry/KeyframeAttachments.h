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
 * @file   KeyframeAttachments.h
 * @brief  Observations of other sensors attached to simplemap keyframes
 * @author Jose Luis Blanco Claraco
 */
#pragma once

#include <mrpt/core/Clock.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservationImage.h>

#include <cstddef>
#include <map>
#include <string>

namespace mola
{
/** Recent observations of one sensor, from which the one closest in time to
 *  each new keyframe is taken. Not thread-safe.
 *
 * \ingroup mola_lidar_odometry_grp
 */
class KeyframeAttachmentBuffer
{
public:
  struct Options
  {
    /// Maximum time offset [s] between an observation and the keyframe.
    double max_age = 0.5;
    /// If true, each observation is attached to one keyframe at most.
    bool attach_once = false;
    /// Older observations are dropped beyond this many.
    std::size_t max_size = 10;
  };

  KeyframeAttachmentBuffer() = default;
  explicit KeyframeAttachmentBuffer(const Options & opts) : opts_(opts) {}

  [[nodiscard]] const Options & options() const { return opts_; }

  void add(const mrpt::obs::CObservation::ConstPtr & obs);

  /** The observation closest in time to `t` within `max_age` (with
   *  `attach_once`, among those not taken before, and then marked as taken),
   *  or nullptr if there is none. */
  [[nodiscard]] mrpt::obs::CObservation::ConstPtr take_closest(const mrpt::Clock::time_point & t);

  [[nodiscard]] std::size_t size() const { return buf_.size(); }

private:
  struct Entry
  {
    mrpt::obs::CObservation::ConstPtr obs;
    bool taken = false;
  };
  Options opts_;
  std::map<mrpt::Clock::time_point, Entry> buf_;
};

/** Storage format of keyframe images: "jpg" or "png" files in the lazy-load
 *  directory of the simplemap, or "inline" (pixels inside the .simplemap). */
[[nodiscard]] bool is_valid_keyframe_image_format(const std::string & format);

/** File name for an externally stored observation: "<label>_<stamp>.<ext>". */
[[nodiscard]] std::string external_observation_file_name(
  const mrpt::obs::CObservation & obs, const std::string & ext);

/** A copy of `obs` whose image refers to `fileName` (lazy-load, relative to
 *  mrpt::io::getLazyLoadPathBase()) instead of holding the pixels. The file is
 *  not written here. `obs` is left untouched: CImage copies are shallow, so
 *  converting the image of an observation shared with other modules in place
 *  would change theirs too. */
[[nodiscard]] mrpt::obs::CObservationImage::Ptr external_image_copy(
  const mrpt::obs::CObservationImage & obs, const std::string & fileName);

}  // namespace mola
