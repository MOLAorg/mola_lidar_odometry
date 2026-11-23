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

#include <gtest/gtest.h>
#include <mola_lidar_odometry/AdaptiveDecimation.h>
#include <mrpt/core/bits_math.h>

#include <string>
#include <vector>

namespace
{
constexpr double PERIOD = 0.1;  // 10 Hz LiDAR
constexpr double IMU_DT = 0.005;

mrpt::math::TTwist3D twist(double v, double w)
{
  mrpt::math::TTwist3D tw;
  tw.vx = v;
  tw.wz = w;
  return tw;
}

/// Runs `n` scans with a constant twist and, if `w_gyro` is set, gyroscope
/// readings at that yaw rate. Returns the decision for each scan ("" = skip).
std::vector<std::string> run(
  mola::AdaptiveDecimation & ad, const mola::AdaptiveDecimation::Budgets & b, int n, double v,
  double w, bool with_gyro)
{
  std::vector<std::string> out;
  double t = 0;
  double tLastProcessed = -1;
  for (int i = 0; i < n; i++) {
    if (with_gyro) {
      for (double ti = t - PERIOD + IMU_DT; ti <= t + 1e-9; ti += IMU_DT) {
        ad.addGyro(ti, {0, 0, w});
      }
    }
    mola::AdaptiveDecimation::Scan s;
    s.gap = tLastProcessed < 0 ? 0 : t - tLastProcessed;
    s.period = PERIOD;
    s.twist = twist(v, w);
    const char * r = ad.decide(s, b);
    out.push_back(r ? r : "");
    if (r) {
      tLastProcessed = t;
    }
    t += PERIOD;
  }
  return out;
}

/// The budgets these tests were written for.
mola::AdaptiveDecimation::Budgets budgets5()
{
  mola::AdaptiveDecimation::Budgets b;
  b.max_rotation_deg = 5.0;
  b.max_distance = 1.0;
  return b;
}

int countProcessed(const std::vector<std::string> & d)
{
  int n = 0;
  for (const auto & s : d) {
    n += s.empty() ? 0 : 1;
  }
  return n;
}
}  // namespace

TEST(AdaptiveDecimation, StandingStillKeepsOneOfMaxSkippedPlusOne)
{
  mola::AdaptiveDecimation ad;
  mola::AdaptiveDecimation::Budgets b = budgets5();
  const auto d = run(ad, b, 41, 0, 0, true);
  EXPECT_EQ(d[0], "start");
  EXPECT_EQ(countProcessed(d), 11);  // 1 of 4
  EXPECT_EQ(d[4], "max_skipped_scans");
}

TEST(AdaptiveDecimation, FastRotationProcessesEveryScan)
{
  mola::AdaptiveDecimation ad;
  mola::AdaptiveDecimation::Budgets b = budgets5();
  // 40 deg/s: 4 deg per scan, two scans would exceed the 5 deg budget.
  const auto d = run(ad, b, 20, 0, mrpt::DEG2RAD(40.0), true);
  EXPECT_EQ(countProcessed(d), 20);
  EXPECT_EQ(d[5], "rotation");
}

TEST(AdaptiveDecimation, ModerateRotationKeepsOneOfTwo)
{
  mola::AdaptiveDecimation ad;
  mola::AdaptiveDecimation::Budgets b = budgets5();
  // 20 deg/s: 2 deg per scan, so 2 scans fit in the budget but 3 do not.
  const auto d = run(ad, b, 21, 0, mrpt::DEG2RAD(20.0), true);
  EXPECT_EQ(countProcessed(d), 11);
}

TEST(AdaptiveDecimation, WithoutGyroProcessesEveryScanByDefault)
{
  mola::AdaptiveDecimation ad;
  mola::AdaptiveDecimation::Budgets b = budgets5();
  const auto d = run(ad, b, 20, 0, 0, false);
  EXPECT_EQ(countProcessed(d), 20);
  EXPECT_EQ(d[1], "no_gyro");
}

TEST(AdaptiveDecimation, WithoutGyroCanUseTheEstimatorTwist)
{
  mola::AdaptiveDecimation::Budgets b = budgets5();
  b.require_gyro = false;
  {
    mola::AdaptiveDecimation ad;
    const auto d = run(ad, b, 20, 0, mrpt::DEG2RAD(40.0), false);
    EXPECT_EQ(countProcessed(d), 20);
  }
  {
    mola::AdaptiveDecimation ad;
    const auto d = run(ad, b, 41, 0, 0, false);
    EXPECT_EQ(countProcessed(d), 11);
  }
}

TEST(AdaptiveDecimation, StartOfATurnCountsAtOnce)
{
  mola::AdaptiveDecimation ad;
  mola::AdaptiveDecimation::Budgets b = budgets5();
  mola::AdaptiveDecimation::Scan s;
  s.period = PERIOD;
  s.twist = twist(0, 0);
  double t = 0;
  const auto sweep = [&](double w) {
    for (int i = 0; i < 20; i++) {
      t += IMU_DT;
      ad.addGyro(t, {0, 0, w});
    }
  };
  sweep(0);
  s.gap = 0;
  EXPECT_STREQ(ad.decide(s, b), "start");
  // Still, then a 30 deg/s turn in the second sweep: 3 deg so far, and 3 more
  // by the next scan, over the 5 deg budget.
  sweep(0);
  s.gap = PERIOD;
  EXPECT_EQ(ad.decide(s, b), nullptr);
  sweep(mrpt::DEG2RAD(30.0));
  s.gap = 2 * PERIOD;
  EXPECT_STREQ(ad.decide(s, b), "rotation");
}

TEST(AdaptiveDecimation, SpeedLimitsTheGap)
{
  mola::AdaptiveDecimation ad;
  mola::AdaptiveDecimation::Budgets b = budgets5();
  // 4 m/s: 0.4 m per scan, so at most 2 scans per 1 m budget.
  const auto d = run(ad, b, 21, 4.0, 0, true);
  EXPECT_EQ(countProcessed(d), 11);
  EXPECT_EQ(d[2], "distance");
}

TEST(AdaptiveDecimation, TimeGapBudget)
{
  mola::AdaptiveDecimation ad;
  mola::AdaptiveDecimation::Budgets b = budgets5();
  b.max_skipped_scans = 100;
  b.max_time_gap = 0.25;
  const auto d = run(ad, b, 21, 0, 0, true);
  EXPECT_EQ(countProcessed(d), 11);  // gaps of 0.2 s
  EXPECT_EQ(d[2], "time_gap");
}

TEST(AdaptiveDecimation, BadRegistrationAndMissingModelAreProcessed)
{
  mola::AdaptiveDecimation ad;
  mola::AdaptiveDecimation::Budgets b = budgets5();
  b.require_gyro = false;
  mola::AdaptiveDecimation::Scan s;
  s.gap = PERIOD;
  s.period = PERIOD;
  s.twist = twist(0, 0);
  s.last_registration_good = false;
  EXPECT_STREQ(ad.decide(s, b), "bad_registration");

  s.last_registration_good = true;
  s.twist.reset();
  EXPECT_STREQ(ad.decide(s, b), "no_motion_model");

  s.twist = twist(0, 0);
  EXPECT_EQ(ad.decide(s, b), nullptr);
}

TEST(AdaptiveDecimation, GyroRotationIsTheNetAngle)
{
  mola::AdaptiveDecimation ad;
  // +0.5 rad/s then -0.5 rad/s, 0.1 s each: back to the start.
  double t = 0;
  for (int i = 0; i < 20; i++, t += IMU_DT) {
    ad.addGyro(t, {0, 0, 0.5});
  }
  for (int i = 0; i < 20; i++, t += IMU_DT) {
    ad.addGyro(t, {0, 0, -0.5});
  }
  EXPECT_NEAR(ad.gyroRotation(), 0.0, 0.01);
}

TEST(AdaptiveDecimation, LowQualityHoldsFullRate)
{
  mola::AdaptiveDecimation ad;
  mola::AdaptiveDecimation::Budgets b = budgets5();
  b.require_gyro = false;
  for (int i = 0; i < 20; i++) {
    ad.addRegistrationQuality(0.1 * i, 0.99, b);
  }
  mola::AdaptiveDecimation::Scan s;
  s.gap = PERIOD;
  s.period = PERIOD;
  s.twist = twist(0, 0);
  s.stamp = 2.0;
  EXPECT_EQ(ad.decide(s, b), nullptr);

  // 0.95 < 0.97 * 0.99: every scan for the next 2 s.
  ad.addRegistrationQuality(2.0, 0.95, b);
  s.stamp = 3.9;
  EXPECT_STREQ(ad.decide(s, b), "quality");
  s.stamp = 4.1;
  EXPECT_EQ(ad.decide(s, b), nullptr);

  // A small drop is no trigger:
  ad.addRegistrationQuality(4.1, 0.97, b);
  s.stamp = 4.2;
  EXPECT_EQ(ad.decide(s, b), nullptr);
}

TEST(AdaptiveDecimation, SweepWithoutGyroAfterASkipIsProcessed)
{
  mola::AdaptiveDecimation ad;
  const auto b = budgets5();
  mola::AdaptiveDecimation::Scan s;
  s.period = PERIOD;
  s.twist = twist(0, 0);
  double t = 0;
  const auto sweep = [&](bool gyro) {
    for (int i = 0; i < 20; i++) {
      t += IMU_DT;
      if (gyro) {
        ad.addGyro(t, {0, 0, 0});
      }
    }
  };
  sweep(true);
  s.gap = 0;
  EXPECT_STREQ(ad.decide(s, b), "start");
  sweep(true);
  s.gap = PERIOD;
  EXPECT_EQ(ad.decide(s, b), nullptr);
  // The gyroscope stops: the stale motion so far must not allow a skip.
  sweep(false);
  s.gap = 2 * PERIOD;
  EXPECT_STREQ(ad.decide(s, b), "no_gyro");
  // It comes back after a gap: that sweep is not covered either.
  sweep(true);
  s.gap = PERIOD;
  EXPECT_STREQ(ad.decide(s, b), "no_gyro");
  sweep(true);
  EXPECT_EQ(ad.decide(s, b), nullptr);
}

TEST(AdaptiveDecimation, GyroStoppingInsideTheSweepIsNotCoverage)
{
  mola::AdaptiveDecimation ad;
  auto b = budgets5();
  b.max_time_gap = 1.0;
  constexpr double SWEEP = 0.2;  // 5 Hz LiDAR
  mola::AdaptiveDecimation::Scan s;
  s.period = SWEEP;
  s.twist = twist(0, 0);
  double t = 0;
  // Gyroscope readings over the first `covered` seconds of a sweep:
  const auto sweep = [&](double covered) {
    const double start = t;
    for (double ti = start + IMU_DT; ti <= start + covered + 1e-9; ti += IMU_DT) {
      ad.addGyro(ti, {0, 0, 0});
    }
    t = start + SWEEP;
    s.sweep_end = t;
  };
  sweep(SWEEP);
  s.gap = 0;
  EXPECT_STREQ(ad.decide(s, b), "start");
  sweep(SWEEP);
  s.gap = SWEEP;
  EXPECT_EQ(ad.decide(s, b), nullptr);
  // The readings stop 0.15 s before the end of the sweep:
  sweep(0.05);
  s.gap = 2 * SWEEP;
  EXPECT_STREQ(ad.decide(s, b), "no_gyro");
}

TEST(AdaptiveDecimation, RotationAfterAGapDoesNotUseAPartialGyroInterval)
{
  // Without require_gyro, a gyroscope gap since the last processed scan
  // leaves the integrated rotation incomplete, so the estimator twist is used
  // until the next processed scan.
  mola::AdaptiveDecimation ad;
  auto b = budgets5();
  b.require_gyro = false;
  b.max_rotation_deg = 3.5;
  mola::AdaptiveDecimation::Scan s;
  s.period = PERIOD;
  const double w = mrpt::DEG2RAD(10.0);
  s.twist = twist(0, w);
  double t = 0;
  const auto sweep = [&](bool gyro) {
    for (int i = 0; i < 20; i++) {
      t += IMU_DT;
      if (gyro) {
        ad.addGyro(t, {0, 0, w});
      }
    }
    s.sweep_end = t;
  };
  sweep(true);
  s.gap = 0;
  EXPECT_STREQ(ad.decide(s, b), "start");
  sweep(false);  // twist: 10 deg/s x 0.2 s = 2 deg
  s.gap = PERIOD;
  EXPECT_EQ(ad.decide(s, b), nullptr);
  sweep(true);  // the gap ends in this sweep; twist: 3 deg
  s.gap = 2 * PERIOD;
  EXPECT_EQ(ad.decide(s, b), nullptr);
  // A covered sweep, but the gyroscope missed part of the interval: it would
  // give ~3 deg; the twist gives 4 deg, over the budget.
  sweep(true);
  s.gap = 3 * PERIOD;
  EXPECT_STREQ(ad.decide(s, b), "rotation");
}
