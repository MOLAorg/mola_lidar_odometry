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
 * @file   test_imu_bias_correction.cpp
 * @brief  Unit tests for remove_imu_gyro_bias() (IMU biases from mola::ImuBiasSource)
 * @author Jose Luis Blanco Claraco
 */

#include <gtest/gtest.h>
#include <mola_lidar_odometry/ImuBiasCorrection.h>

#if __has_include(<mola_kernel/interfaces/ImuBiasSource.h>)

#include <mrpt/core/bits_math.h>
#include <mrpt/random/RandomGenerators.h>

#include <map>
#include <mutex>

namespace
{
// Biases like those of a MEMS IMU integrated in a LiDAR [rad/s]:
const mrpt::math::TVector3D LIDAR_IMU_BIAS{0.0004, -0.0051, -0.0035};
constexpr double RATE = 100.0;  // [Hz]

/// A bias provider with fixed estimates per label.
class FixedBiasSource : public mola::ImuBiasSource
{
public:
  void set(const std::string & label, const mrpt::math::TVector3D & gyro, bool measured)
  {
    std::lock_guard<std::mutex> lck(mtx_);
    mola::ImuBias b;
    b.sensor_label = label;
    b.gyro = gyro;
    b.gyro_cov.setDiagonal(mrpt::square(1e-4));
    b.acc_cov.setDiagonal(mrpt::square(0.3));
    b.timestamp = mrpt::Clock::fromDouble(1.8e9);
    if (measured) {
      b.last_update = b.timestamp;
    }
    biases_[label] = b;
  }

  std::vector<std::string> imu_bias_sensor_labels() const override
  {
    std::lock_guard<std::mutex> lck(mtx_);
    std::vector<std::string> out;
    for (const auto & [label, b] : biases_) {
      out.push_back(label);
    }
    return out;
  }

private:
  std::optional<mola::ImuBias> imu_bias_impl(
    const std::string & label, const std::optional<mrpt::Clock::time_point> &) const override
  {
    std::lock_guard<std::mutex> lck(mtx_);
    const auto it = biases_.find(label);
    if (it == biases_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  std::map<std::string, mola::ImuBias> biases_;
  mutable std::mutex mtx_;
};

mrpt::obs::CObservationIMU::Ptr reading(const std::string & label, double t, double wz)
{
  auto & rng = mrpt::random::getRandomGenerator();
  auto o = mrpt::obs::CObservationIMU::Create();
  o->sensorLabel = label;
  o->timestamp = mrpt::Clock::fromDouble(1.8e9 + t);
  o->set(mrpt::obs::IMU_WX, LIDAR_IMU_BIAS.x + rng.drawGaussian1D(0, 0.003));
  o->set(mrpt::obs::IMU_WY, LIDAR_IMU_BIAS.y + rng.drawGaussian1D(0, 0.003));
  o->set(mrpt::obs::IMU_WZ, wz + LIDAR_IMU_BIAS.z + rng.drawGaussian1D(0, 0.003));
  o->set(mrpt::obs::IMU_Z_ACC, 9.81);
  return o;
}
}  // namespace

// A 90 deg turn followed by 60 s of straight driving: the integrated heading of
// the corrected readings stays right, the raw one drifts by the bias.
TEST(ImuBiasCorrection, CorrectedHeadingDoesNotDrift)
{
  mrpt::random::getRandomGenerator().randomize(5);
  auto src = std::make_shared<FixedBiasSource>();
  src->set("lidar_imu", LIDAR_IMU_BIAS, true);
  const std::vector<std::shared_ptr<mola::ImuBiasSource>> sources = {src};

  double yawRaw = 0;
  double yawCorrected = 0;
  double yawTrue = 0;
  for (int i = 0; i < static_cast<int>(63.14 * RATE); i++) {
    const double t = i / RATE;
    const double wz = t < 3.14 ? 0.5 : 0.0;
    yawTrue += wz / RATE;
    const auto raw = reading("lidar_imu", t, wz);
    const auto [corr, bias] = mola::remove_imu_gyro_bias(raw, sources);
    ASSERT_TRUE(bias.has_value());
    ASSERT_NE(corr.get(), raw.get());
    // Only the angular velocity changes:
    EXPECT_EQ(corr->get(mrpt::obs::IMU_Z_ACC), raw->get(mrpt::obs::IMU_Z_ACC));
    EXPECT_EQ(corr->timestamp, raw->timestamp);
    yawRaw += raw->get(mrpt::obs::IMU_WZ) / RATE;
    yawCorrected += corr->get(mrpt::obs::IMU_WZ) / RATE;
  }
  // 63 s at -0.2 deg/s: ~13 deg of drift, vs the noise integral (0.14 deg, 1 sigma):
  EXPECT_GT(std::abs(yawRaw - yawTrue), mrpt::DEG2RAD(12.0));
  EXPECT_LT(std::abs(yawCorrected - yawTrue), mrpt::DEG2RAD(0.6));
}

// Readings without a usable estimate are passed through unchanged; the first
// source with one is used.
TEST(ImuBiasCorrection, PassThroughAndSourceOrder)
{
  auto priorOnly = std::make_shared<FixedBiasSource>();
  priorOnly->set("lidar_imu", {0.1, 0.1, 0.1}, false);  // still the prior: skipped
  auto good = std::make_shared<FixedBiasSource>();
  good->set("lidar_imu", LIDAR_IMU_BIAS, true);
  const std::vector<std::shared_ptr<mola::ImuBiasSource>> sources = {priorOnly, nullptr, good};

  const auto raw = reading("lidar_imu", 0, 0);
  const auto [corr, bias] = mola::remove_imu_gyro_bias(raw, sources);
  ASSERT_TRUE(bias.has_value());
  EXPECT_NEAR(corr->get(mrpt::obs::IMU_WY), raw->get(mrpt::obs::IMU_WY) - LIDAR_IMU_BIAS.y, 1e-15);

  // Unknown label:
  const auto other = reading("attitude_imu", 0, 0);
  const auto [o2, b2] = mola::remove_imu_gyro_bias(other, sources);
  EXPECT_FALSE(b2.has_value());
  EXPECT_EQ(o2.get(), other.get());

  // No angular velocity:
  auto accOnly = mrpt::obs::CObservationIMU::Create();
  accOnly->sensorLabel = "lidar_imu";
  accOnly->set(mrpt::obs::IMU_Z_ACC, 9.81);
  const auto [o3, b3] = mola::remove_imu_gyro_bias(accOnly, sources);
  EXPECT_FALSE(b3.has_value());
  EXPECT_EQ(o3.get(), accOnly.get());

  // No sources:
  const auto [o4, b4] = mola::remove_imu_gyro_bias(raw, {});
  EXPECT_FALSE(b4.has_value());
  EXPECT_EQ(o4.get(), raw.get());
}

#else
TEST(ImuBiasCorrection, Skipped) { GTEST_SKIP() << "mola_kernel lacks mola::ImuBiasSource"; }
#endif

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
