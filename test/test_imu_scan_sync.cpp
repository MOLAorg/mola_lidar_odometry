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
 * @file   test_imu_scan_sync.cpp
 * @brief  Unit tests for PendingImuBuffer and ScanImuWaitList
 * @author Jose Luis Blanco Claraco
 */

#include <gtest/gtest.h>
#include <mola_lidar_odometry/ImuScanSync.h>
#include <mrpt/obs/CObservationPointCloud.h>

#include <algorithm>
#include <limits>
#include <vector>

namespace
{
mrpt::obs::CObservationIMU::Ptr makeImu(double t)
{
  auto o = mrpt::obs::CObservationIMU::Create();
  o->timestamp = mrpt::Clock::fromDouble(t);
  o->set(mrpt::obs::IMU_WZ, t);  // a value that identifies the sample
  return o;
}

mrpt::obs::CObservation::Ptr makeScan(double t)
{
  auto o = mrpt::obs::CObservationPointCloud::Create();
  o->timestamp = mrpt::Clock::fromDouble(t);
  return o;
}

std::vector<double> valuesOf(const std::vector<mrpt::obs::CObservationIMU::ConstPtr> & v)
{
  std::vector<double> ret;
  for (const auto & o : v) {
    ret.push_back(o->get(mrpt::obs::IMU_WZ));
  }
  return ret;
}
}  // namespace

// The samples a given time window yields must not depend on the order in which
// they were handed to the buffer.
TEST(PendingImuBuffer, WindowIsIndependentOfInsertionOrder)
{
  const std::vector<double> times = {1.0, 1.1, 1.2, 1.3, 1.4};

  const auto fill = [&](const std::vector<double> & order) {
    mola::PendingImuBuffer buf;
    for (const double t : order) {
      buf.add(t, makeImu(t), 10.0);
    }
    return buf;
  };

  auto reversed = times;
  std::reverse(reversed.begin(), reversed.end());

  const std::vector<double> expected = {1.0, 1.1, 1.2};

  auto inOrder = fill(times);
  auto outOfOrder = fill(reversed);

  EXPECT_EQ(valuesOf(inOrder.take_up_to(1.25)), expected);
  EXPECT_EQ(valuesOf(outOfOrder.take_up_to(1.25)), expected);

  // ...and the samples past the window are still there, for the next scan:
  EXPECT_EQ(inOrder.size(), 2U);
  EXPECT_EQ(outOfOrder.size(), 2U);
}

// A sample is consumed exactly once, and the boundary is inclusive.
TEST(PendingImuBuffer, ConsumesEachSampleOnce)
{
  mola::PendingImuBuffer buf;
  for (const double t : {1.0, 1.1, 1.2, 1.3}) {
    buf.add(t, makeImu(t), 10.0);
  }

  EXPECT_EQ(valuesOf(buf.take_up_to(1.1)), (std::vector<double>{1.0, 1.1}));
  EXPECT_EQ(valuesOf(buf.take_up_to(1.3)), (std::vector<double>{1.2, 1.3}));
  EXPECT_TRUE(buf.take_up_to(1.3).empty());
  EXPECT_EQ(buf.size(), 0U);
}

// Two readings sharing a timestamp are both kept: dropping one would silently
// discard data that used to be fused.
TEST(PendingImuBuffer, KeepsReadingsSharingATimestamp)
{
  mola::PendingImuBuffer buf;
  buf.add(1.0, makeImu(1.0), 10.0);
  buf.add(1.0, makeImu(1.0), 10.0);
  buf.add(1.1, makeImu(1.1), 10.0);

  EXPECT_EQ(buf.size(), 3U);
  EXPECT_EQ(valuesOf(buf.take_up_to(1.0)), (std::vector<double>{1.0, 1.0}));
  EXPECT_EQ(buf.size(), 1U);
}

TEST(PendingImuBuffer, DropsSamplesOlderThanMaxAge)
{
  mola::PendingImuBuffer buf;
  for (const double t : {1.0, 1.5, 2.0, 2.5, 3.0}) {
    buf.add(t, makeImu(t), 1.0);
  }

  // Only those within 1 s of the newest one survive:
  EXPECT_EQ(valuesOf(buf.take_up_to(10.0)), (std::vector<double>{2.0, 2.5, 3.0}));
}

// A scan is released only once IMU data reaches its own coverage end time, and
// that decision never depends on anything but timestamps.
TEST(ScanImuWaitList, ReleasesOnlyFullyCoveredScans)
{
  mola::ScanImuWaitList list;
  list.add(1.0, {makeScan(1.0), 1.1});
  list.add(1.1, {makeScan(1.1), 1.2});
  list.add(1.2, {makeScan(1.2), 1.3});

  EXPECT_TRUE(list.take_ready(1.05).empty());
  EXPECT_EQ(list.size(), 3U);

  // Exact coverage is enough:
  const auto first = list.take_ready(1.1);
  ASSERT_EQ(first.size(), 1U);
  EXPECT_EQ(first[0].imu_coverage_end_time, 1.1);
  EXPECT_EQ(list.size(), 2U);

  // A catch-up burst releases several at once, oldest first:
  const auto burst = list.take_ready(5.0);
  ASSERT_EQ(burst.size(), 2U);
  EXPECT_EQ(burst[0].imu_coverage_end_time, 1.2);
  EXPECT_EQ(burst[1].imu_coverage_end_time, 1.3);
  EXPECT_EQ(list.size(), 0U);
}

// A scan whose coverage end is beyond the newest IMU data blocks only itself;
// the ones before it are still released (they are older, hence covered first).
TEST(ScanImuWaitList, TrimDropsTheOldest)
{
  mola::ScanImuWaitList list;
  for (int i = 0; i < 5; i++) {
    const double t = 1.0 + 0.1 * i;
    list.add(t, {makeScan(t), t + 0.1});
  }

  EXPECT_EQ(list.trim_to(2), 3U);
  EXPECT_EQ(list.size(), 2U);

  const auto ready = list.take_ready(10.0);
  ASSERT_EQ(ready.size(), 2U);
  // The two newest ones are the survivors:
  EXPECT_NEAR(ready[0].imu_coverage_end_time, 1.4, 1e-9);
  EXPECT_NEAR(ready[1].imu_coverage_end_time, 1.5, 1e-9);

  EXPECT_EQ(list.trim_to(2), 0U);
}

// What flushPendingLidarScans() relies on at the end of the input: an infinite
// coverage time releases every waiting scan, whatever IMU data did arrive.
TEST(ScanImuWaitList, InfiniteCoverageReleasesEverything)
{
  mola::ScanImuWaitList list;
  for (int i = 0; i < 4; i++) {
    const double t = 1.0 + 0.1 * i;
    list.add(t, {makeScan(t), t + 0.1});
  }

  // None of them is covered by the IMU data actually received:
  EXPECT_TRUE(list.take_ready(0.5).empty());
  EXPECT_EQ(list.size(), 4U);

  const auto flushed = list.take_ready(std::numeric_limits<double>::infinity());
  EXPECT_EQ(flushed.size(), 4U);
  EXPECT_EQ(list.size(), 0U);
}

// Once a scan has consumed up to some instant, a reading for an earlier one is
// of no use: applying it would feed IMU data out of chronological order. This
// is what a scan released without its IMU (a stalled IMU that later resumes)
// would otherwise cause.
TEST(PendingImuBuffer, RejectsReadingsAlreadyConsumed)
{
  mola::PendingImuBuffer buf;

  EXPECT_TRUE(buf.add(1.0, makeImu(1.0), 10.0));
  EXPECT_TRUE(buf.add(1.1, makeImu(1.1), 10.0));

  EXPECT_EQ(valuesOf(buf.take_up_to(1.2)), (std::vector<double>{1.0, 1.1}));

  // Late arrivals for instants already passed:
  EXPECT_FALSE(buf.add(1.05, makeImu(1.05), 10.0));
  EXPECT_FALSE(buf.add(1.2, makeImu(1.2), 10.0));
  EXPECT_EQ(buf.size(), 0U);

  // Anything newer is still accepted as usual:
  EXPECT_TRUE(buf.add(1.3, makeImu(1.3), 10.0));
  EXPECT_EQ(valuesOf(buf.take_up_to(2.0)), (std::vector<double>{1.3}));

  // clear() starts a fresh session, so the watermark must go with it:
  buf.clear();
  EXPECT_TRUE(buf.add(1.0, makeImu(1.0), 10.0));
}

// A healthy IMU decides the release time on its own; a stalled one hands over
// to the bound, so scans keep flowing instead of the odometry stalling for good.
TEST(ImuScanReleaseTime, BoundsTheWaitOnlyWhenTheImuFallsBehind)
{
  constexpr double maxWait = 0.5;

  // IMU keeping up with the rest of the inputs: it alone decides.
  EXPECT_EQ(mola::imu_scan_release_time(10.0, 10.02, maxWait), 10.0);
  // Still within the tolerated lag:
  EXPECT_EQ(mola::imu_scan_release_time(10.0, 10.4, maxWait), 10.0);
  // Stopped: sensor time has moved on, so scans up to (now - maxWait) run
  // without it. The released window keeps advancing as sensor time does.
  EXPECT_EQ(mola::imu_scan_release_time(10.0, 11.0, maxWait), 10.5);
  EXPECT_EQ(mola::imu_scan_release_time(10.0, 20.0, maxWait), 19.5);
  // ...and the IMU coming back takes over again:
  EXPECT_EQ(mola::imu_scan_release_time(21.0, 20.0, maxWait), 21.0);

  // Disabled: wait indefinitely, whatever sensor time says.
  EXPECT_EQ(mola::imu_scan_release_time(10.0, 99.0, 0.0), 10.0);
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
