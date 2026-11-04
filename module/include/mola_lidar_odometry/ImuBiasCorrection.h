/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

/**
 * @file   ImuBiasCorrection.h
 * @brief  Removal of the gyroscope bias of IMU readings, from mola::ImuBiasSource,
 *         and its description for the GUI
 * @author Jose Luis Blanco Claraco
 */
#pragma once

#if __has_include(<mola_kernel/interfaces/ImuBiasSource.h>)

#include <mola_kernel/interfaces/ImuBiasSource.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/format.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/system/datetime.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace mola
{
/** Returns the reading with its gyroscope bias removed, using the first source
 *  with an estimate (past its prior) for its sensor label, and that estimate; or
 *  the same reading and nothing if none has one, or it has no angular velocity.
 */
inline std::pair<mrpt::obs::CObservationIMU::ConstPtr, std::optional<ImuBias>> remove_imu_gyro_bias(
  const mrpt::obs::CObservationIMU::ConstPtr & imu,
  const std::vector<std::shared_ptr<ImuBiasSource>> & sources)
{
  using namespace mrpt::obs;
  if (!imu || !imu->has(IMU_WX) || !imu->has(IMU_WY) || !imu->has(IMU_WZ)) {
    return {imu, std::nullopt};
  }
  for (const auto & src : sources) {
    if (!src) {
      continue;
    }
    auto b = src->imu_bias(imu->sensorLabel);
    if (!b || !b->last_update) {
      continue;  // unknown IMU, or still the prior
    }
    auto out = CObservationIMU::Create(*imu);
    out->set(IMU_WX, imu->get(IMU_WX) - b->gyro.x);
    out->set(IMU_WY, imu->get(IMU_WY) - b->gyro.y);
    out->set(IMU_WZ, imu->get(IMU_WZ) - b->gyro.z);
    return {out, std::move(b)};
  }
  return {imu, std::nullopt};
}

/** Three short text lines describing the bias estimate of the IMU `label` for
 *  the GUI: its age at time `now` (e.g. the newest IMU reading), the gyroscope
 *  bias [deg/s] and the accelerometer bias [m/s^2], each with the largest
 *  standard deviation of its axes. Also tells apart no source, no IMU yet, an IMU
 *  unknown to the sources, and a prior not yet updated by any measurement (whose
 *  bias is not removed from the readings, see remove_imu_gyro_bias()). */
inline std::array<std::string, 3> imu_bias_status_lines(
  const std::vector<std::shared_ptr<ImuBiasSource>> & sources, const std::string & label,
  const std::optional<mrpt::Clock::time_point> & now)
{
  if (sources.empty()) {
    return {"IMU bias: no source", " ", " "};
  }
  if (label.empty()) {
    return {"IMU bias: no IMU readings yet", " ", " "};
  }
  // As remove_imu_gyro_bias(): the first source with a measured estimate, else a prior:
  std::optional<ImuBias> best;
  for (const auto & src : sources) {
    if (!src) {
      continue;
    }
    auto b = src->imu_bias(label);
    if (b && b->last_update) {
      best = std::move(b);
      break;
    }
    if (b && !best) {
      best = std::move(b);
    }
  }
  if (!best) {
    return {mrpt::format("IMU bias ('%s'): not estimated", label.c_str()), " ", " "};
  }
  if (!best->last_update) {
    return {mrpt::format("IMU bias ('%s'): prior, no measurement yet", label.c_str()), " ", " "};
  }
  const auto maxSigma = [](const mrpt::math::CMatrixDouble33 & c) {
    return std::sqrt(std::max({c(0, 0), c(1, 1), c(2, 2), 0.0}));
  };
  const auto & g = best->gyro;
  const auto & a = best->acc;
  std::string header = mrpt::format("IMU bias ('%s'):", label.c_str());
  if (now) {
    header += mrpt::format(
      " updated %.0f s ago", std::max(0.0, mrpt::system::timeDifference(*best->last_update, *now)));
  }
  return {
    header,
    mrpt::format(
      " Gyro: %+.3f %+.3f %+.3f deg/s (sigma %.3f)", mrpt::RAD2DEG(g.x), mrpt::RAD2DEG(g.y),
      mrpt::RAD2DEG(g.z), mrpt::RAD2DEG(maxSigma(best->gyro_cov))),
    mrpt::format(
      " Acc: %+.2f %+.2f %+.2f m/s^2 (sigma %.2f)", a.x, a.y, a.z, maxSigma(best->acc_cov))};
}

}  // namespace mola

#endif
