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
 * @file   KeyframeAttachments.cpp
 * @brief  Observations of other sensors attached to simplemap keyframes
 * @author Jose Luis Blanco Claraco
 */

#include <mola_lidar_odometry/KeyframeAttachments.h>
#include <mrpt/core/format.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>

#include <cmath>
#include <optional>

namespace mola
{
void KeyframeAttachmentBuffer::add(const mrpt::obs::CObservation::ConstPtr & obs)
{
  if (!obs) {
    return;
  }
  buf_.emplace(obs->timestamp, Entry{obs, false});
  while (buf_.size() > opts_.max_size) {
    buf_.erase(buf_.begin());
  }
}

mrpt::obs::CObservation::ConstPtr KeyframeAttachmentBuffer::take_closest(
  const mrpt::Clock::time_point & t)
{
  std::optional<double> bestDt;
  Entry * best = nullptr;
  for (auto & [stamp, e] : buf_) {
    if (opts_.attach_once && e.taken) {
      continue;
    }
    const double dt = std::abs(mrpt::system::timeDifference(stamp, t));
    if (dt > opts_.max_age) {
      continue;
    }
    if (!bestDt || dt < *bestDt) {
      bestDt = dt;
      best = &e;
    }
  }
  if (best == nullptr) {
    return {};
  }
  best->taken = true;
  return best->obs;
}

bool is_valid_keyframe_image_format(const std::string & format)
{
  return format == "jpg" || format == "png" || format == "inline";
}

std::string external_observation_file_name(
  const mrpt::obs::CObservation & obs, const std::string & ext)
{
  return mrpt::format(
    "%s_%.09f.%s", mrpt::system::fileNameStripInvalidChars(obs.sensorLabel).c_str(),
    mrpt::Clock::toDouble(obs.timestamp), ext.c_str());
}

mrpt::obs::CObservationImage::Ptr external_image_copy(
  const mrpt::obs::CObservationImage & obs, const std::string & fileName)
{
  auto copy = std::make_shared<mrpt::obs::CObservationImage>(obs);
  // A new image, so the shared pixel buffer of `obs` is not touched:
  copy->image = mrpt::img::CImage();
  copy->image.setExternalStorage(fileName);
  return copy;
}

}  // namespace mola
