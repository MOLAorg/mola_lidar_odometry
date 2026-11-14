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
 * @file   test_point_time_outlier_filter.cpp
 * @brief  Unit tests for PointTimeOutlierFilter
 * @author Jose Luis Blanco Claraco
 */

#include <gtest/gtest.h>
#include <mola_lidar_odometry/PointTimeOutlierFilter.h>
#include <mrpt/maps/CGenericPointsMap.h>

#include <limits>
#include <vector>

namespace
{
constexpr std::size_t N = 1000;
constexpr double SCAN_PERIOD = 0.1;  // [s]

double timeOf(std::size_t i, double t0 = 0, double span = SCAN_PERIOD)
{
  return t0 + span * static_cast<double>(i) / static_cast<double>(N);
}

// A scan of N points with times evenly spread over [t0, t0 + span), and other
// fields of every type that identify each point: x = index, "intensity" =
// index (float), "range" = index (double), "ring" = index % 32 (uint16),
// "flag" = index % 7 (uint8), "id" = index (uint32).
mrpt::maps::CGenericPointsMap::Ptr makeScan(double t0 = 0, double span = SCAN_PERIOD)
{
  auto pc = mrpt::maps::CGenericPointsMap::Create();
  pc->registerField_float("t");
  pc->registerField_float("intensity");
  pc->registerField_double("range");
  pc->registerField_uint16("ring");
  pc->registerField_uint8("flag");
  pc->registerField_uint32("id");
  for (std::size_t i = 0; i < N; i++) {
    pc->insertPointFast(static_cast<float>(i), 0, 0);
    pc->insertPointField_float("t", static_cast<float>(timeOf(i, t0, span)));
    pc->insertPointField_float("intensity", static_cast<float>(i));
    pc->insertPointField_double("range", static_cast<double>(i));
    pc->insertPointField_uint16("ring", static_cast<uint16_t>(i % 32));
    pc->insertPointField_uint8("flag", static_cast<uint8_t>(i % 7));
    pc->insertPointField_uint32("id", static_cast<uint32_t>(i));
  }
  return pc;
}

// Feeds scans to a filter at the sensor period.
class Feeder
{
public:
  explicit Feeder(mola::PointTimeOutlierFilter & f) : f_(&f) {}

  mola::PointTimeOutlierFilter::Result feed(mrpt::maps::CPointsMap & pc)
  {
    stamp_ += SCAN_PERIOD;
    return f_->process({&pc}, stamp_);
  }

  void warmUp(std::size_t scans, double t0 = 0, double span = SCAN_PERIOD)
  {
    for (std::size_t i = 0; i < scans; i++) {
      auto pc = makeScan(t0, span);
      EXPECT_EQ(feed(*pc).rejected, 0U);
    }
  }

private:
  mola::PointTimeOutlierFilter * f_;
  double stamp_ = 1000.0;
};
}  // namespace

// Normal scans are never touched, before and after the window is warmed up;
// the accepted band is the usual range widened by (at least) the sensor period.
TEST(PointTimeOutlierFilter, NormalScansPassUntouched)
{
  mola::PointTimeOutlierFilter f;
  EXPECT_FALSE(f.usual_range().has_value());
  Feeder(f).warmUp(100);

  const auto usual = f.usual_range();
  ASSERT_TRUE(usual.has_value());
  EXPECT_NEAR(usual->first, 0.0, 1e-6);
  EXPECT_NEAR(usual->second, timeOf(N - 1), 1e-6);

  const auto band = f.accepted_range();
  ASSERT_TRUE(band.has_value());
  EXPECT_NEAR(band->first, -SCAN_PERIOD, 1e-6);
  EXPECT_NEAR(band->second, timeOf(N - 1) + SCAN_PERIOD, 1e-6);
}

// Columns stamped seconds later (a jump of the sensor clock) are removed, with
// all their fields of every type, the rest keeping their order and values, and
// the cached bounding box follows.
TEST(PointTimeOutlierFilter, RemovesClockJumpColumnsWithAllFields)
{
  mola::PointTimeOutlierFilter f;
  Feeder feeder(f);
  feeder.warmUp(20);

  auto pc = makeScan();
  EXPECT_NEAR(pc->boundingBox().max.x, N - 1, 1e-3);
  auto & t = *pc->getPointsBufferRef_float_field("t");
  // The last 30 % of the points, 3 s later:
  const std::size_t firstBad = N * 7 / 10;
  for (std::size_t i = firstBad; i < N; i++) {
    t[i] += 3.0f;
  }

  const auto r = feeder.feed(*pc);
  EXPECT_EQ(r.points, N);
  EXPECT_EQ(r.rejected, N - firstBad);
  ASSERT_TRUE(r.accepted.has_value());
  EXPECT_NEAR(r.scan_max, 3.0 + timeOf(N - 1), 1e-5);

  ASSERT_EQ(pc->size(), firstBad);
  const auto & xs = pc->getPointsBufferRef_x();
  const auto & ts = *pc->getPointsBufferRef_float_field("t");
  const auto & is = *pc->getPointsBufferRef_float_field("intensity");
  const auto & ds = *pc->getPointsBufferRef_double_field("range");
  const auto & rs = *pc->getPointsBufferRef_uint16_field("ring");
  const auto & fs = *pc->getPointsBufferRef_uint8_field("flag");
  const auto & ids = *pc->getPointsBufferRef_uint32_field("id");
  ASSERT_EQ(ts.size(), firstBad);
  ASSERT_EQ(is.size(), firstBad);
  ASSERT_EQ(ds.size(), firstBad);
  ASSERT_EQ(rs.size(), firstBad);
  ASSERT_EQ(fs.size(), firstBad);
  ASSERT_EQ(ids.size(), firstBad);
  for (std::size_t i = 0; i < firstBad; i++) {
    EXPECT_EQ(xs[i], static_cast<float>(i));
    EXPECT_EQ(is[i], static_cast<float>(i));
    EXPECT_EQ(ds[i], static_cast<double>(i));
    EXPECT_EQ(rs[i], static_cast<uint16_t>(i % 32));
    EXPECT_EQ(fs[i], static_cast<uint8_t>(i % 7));
    EXPECT_EQ(ids[i], static_cast<uint32_t>(i));
    EXPECT_LT(ts[i], SCAN_PERIOD);
  }
  EXPECT_NEAR(pc->boundingBox().max.x, firstBad - 1, 1e-3);
}

// A backward jump (times seconds before the scan start) is caught too.
TEST(PointTimeOutlierFilter, RemovesBackwardJumps)
{
  mola::PointTimeOutlierFilter f;
  Feeder feeder(f);
  feeder.warmUp(20);

  auto pc = makeScan();
  auto & t = *pc->getPointsBufferRef_float_field("t");
  for (std::size_t i = 0; i < 100; i++) {
    t[i] -= 2.0f;
  }
  EXPECT_EQ(feeder.feed(*pc).rejected, 100U);
  EXPECT_EQ(pc->size(), N - 100);
}

// Times slightly beyond the usual range (within the margin) are accepted.
TEST(PointTimeOutlierFilter, SmallDeviationsAreAccepted)
{
  mola::PointTimeOutlierFilter f;
  Feeder feeder(f);
  feeder.warmUp(20);

  auto pc = makeScan();
  auto & t = *pc->getPointsBufferRef_float_field("t");
  t.front() = -0.05f;
  t.back() = 0.17f;
  EXPECT_EQ(feeder.feed(*pc).rejected, 0U);
  EXPECT_EQ(pc->size(), N);
}

// A sensor that only reports directions with returns: scans covering a narrow
// sector, then a whole sweep. The margin never goes below the sensor period, so
// the full sweep loses nothing.
TEST(PointTimeOutlierFilter, NarrowSectorsDoNotClipAFullSweep)
{
  mola::PointTimeOutlierFilter f;
  Feeder feeder(f);
  feeder.warmUp(50, 0.02, 0.02);  // returns only from 20 to 40 ms into the sweep

  auto full = makeScan();
  EXPECT_EQ(feeder.feed(*full).rejected, 0U);
  EXPECT_EQ(full->size(), N);
}

// Nothing is removed while the window is not warmed up yet.
TEST(PointTimeOutlierFilter, NothingRemovedBeforeWarmUp)
{
  mola::PointTimeOutlierFilter::Parameters p;
  p.min_window_length = 10;
  mola::PointTimeOutlierFilter f(p);
  Feeder feeder(f);
  feeder.warmUp(9);

  auto pc = makeScan();
  (*pc->getPointsBufferRef_float_field("t"))[5] = 4.0f;
  const auto r = feeder.feed(*pc);
  EXPECT_EQ(r.rejected, 0U);
  EXPECT_FALSE(r.accepted.has_value());
  EXPECT_EQ(pc->size(), N);
}

// A burst of bad scans, a minority in the window, does not move the usual
// range: the next normal scans still pass and the bad ones are still caught.
TEST(PointTimeOutlierFilter, BurstOfBadScansDoesNotMoveTheUsualRange)
{
  mola::PointTimeOutlierFilter f;
  Feeder feeder(f);
  feeder.warmUp(50);

  for (int k = 0; k < 10; k++) {
    auto pc = makeScan(1.0 + k);
    EXPECT_EQ(feeder.feed(*pc).rejected, N);
  }
  const auto usual = f.usual_range();
  ASSERT_TRUE(usual.has_value());
  EXPECT_NEAR(usual->second, timeOf(N - 1), 1e-6);

  auto pc = makeScan();
  EXPECT_EQ(feeder.feed(*pc).rejected, 0U);
}

// A lasting change of the time range (e.g. another LiDAR mode) is adopted
// once it is the majority of the window.
TEST(PointTimeOutlierFilter, AdaptsToALastingChange)
{
  mola::PointTimeOutlierFilter::Parameters p;
  p.window_length = 20;
  mola::PointTimeOutlierFilter f(p);
  Feeder feeder(f);
  feeder.warmUp(20);

  std::size_t lastRejected = 0;
  for (int k = 0; k < 15; k++) {
    auto pc = makeScan(0, 4 * SCAN_PERIOD);  // a 0.4 s scan
    lastRejected = feeder.feed(*pc).rejected;
  }
  EXPECT_EQ(lastRejected, 0U);
}

// check() filters against the current window without learning from the scan.
TEST(PointTimeOutlierFilter, CheckDoesNotLearn)
{
  mola::PointTimeOutlierFilter f;
  {
    auto pc = makeScan(3.0);
    EXPECT_EQ(f.check({pc.get()}).rejected, 0U);  // not warmed up: nothing to compare with
  }
  Feeder(f).warmUp(20);
  const auto before = f.usual_range();

  for (int k = 0; k < 30; k++) {
    auto pc = makeScan(3.0);
    EXPECT_EQ(f.check({pc.get()}).rejected, N);
  }
  const auto after = f.usual_range();
  ASSERT_TRUE(before && after);
  EXPECT_EQ(before->first, after->first);
  EXPECT_EQ(before->second, after->second);
}

// Clouds without per-point times, disabled filter, NaN times: no effect.
TEST(PointTimeOutlierFilter, NoTimesDisabledAndNaN)
{
  {
    mola::PointTimeOutlierFilter f;
    auto pc = mrpt::maps::CGenericPointsMap::Create();
    pc->insertPoint(1, 2, 3);
    for (int i = 0; i < 20; i++) {
      EXPECT_EQ(f.process({pc.get()}, i * SCAN_PERIOD).points, 0U);
    }
    EXPECT_FALSE(f.usual_range().has_value());
  }
  {
    mola::PointTimeOutlierFilter::Parameters p;
    p.enabled = false;
    mola::PointTimeOutlierFilter f(p);
    Feeder feeder(f);
    feeder.warmUp(20);
    auto pc = makeScan();
    (*pc->getPointsBufferRef_float_field("t"))[0] = 10.0f;
    EXPECT_EQ(feeder.feed(*pc).rejected, 0U);
    EXPECT_EQ(pc->size(), N);
  }
  {
    mola::PointTimeOutlierFilter f;
    Feeder feeder(f);
    feeder.warmUp(20);
    auto pc = makeScan();
    (*pc->getPointsBufferRef_float_field("t"))[3] = std::numeric_limits<float>::quiet_NaN();
    EXPECT_EQ(feeder.feed(*pc).rejected, 0U);
    EXPECT_EQ(pc->size(), N);
  }
}

// Several layers of one observation share the decision and each loses its own
// outliers.
TEST(PointTimeOutlierFilter, SeveralLayers)
{
  mola::PointTimeOutlierFilter f;
  Feeder(f).warmUp(20);

  auto a = makeScan();
  auto b = makeScan();
  (*b->getPointsBufferRef_float_field("t"))[10] = 2.0f;
  (*b->getPointsBufferRef_float_field("t"))[20] = 2.5f;
  const auto r = f.process({a.get(), b.get()}, 2000.0);
  EXPECT_EQ(r.points, 2 * N);
  EXPECT_EQ(r.rejected, 2U);
  EXPECT_EQ(a->size(), N);
  EXPECT_EQ(b->size(), N - 2);
}

// Parameters that would disable the filter or remove everything are rejected.
TEST(PointTimeOutlierFilter, InvalidParametersThrow)
{
  mola::PointTimeOutlierFilter::Parameters p;
  p.window_length = 5;
  p.min_window_length = 10;
  EXPECT_ANY_THROW(mola::PointTimeOutlierFilter{p});

  p = {};
  p.margin_ratio = -0.1;
  EXPECT_ANY_THROW(mola::PointTimeOutlierFilter{p});

  p = {};
  p.min_margin = -1;
  EXPECT_ANY_THROW(p.validate());

  EXPECT_NO_THROW(mola::PointTimeOutlierFilter{mola::PointTimeOutlierFilter::Parameters{}});
}
