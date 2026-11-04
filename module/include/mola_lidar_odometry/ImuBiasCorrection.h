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
 * @brief  Removal of the gyroscope bias of IMU readings, from mola::ImuBiasSource
 * @author Jose Luis Blanco Claraco
 */
#pragma once

#if __has_include(<mola_kernel/interfaces/ImuBiasSource.h>)

#include <mola_kernel/interfaces/ImuBiasSource.h>
#include <mrpt/obs/CObservationIMU.h>

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

}  // namespace mola

#endif
