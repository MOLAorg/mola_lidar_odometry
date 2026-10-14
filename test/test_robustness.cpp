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
 * @file   test_robustness.cpp
 * @brief  Error and concurrency paths of LidarOdometry on synthetic data:
 *         empty scans, reset(), optimize_twist, GNSS input and map saving
 *         while scans are being processed. Best run under ASan/TSan too.
 */

#include <gtest/gtest.h>
#include <mola_kernel/MinimalModuleContainer.h>
#include <mola_lidar_odometry/LidarOdometry.h>
#include <mola_state_estimation_simple/StateEstimationSimple.h>
#include <mola_yaml/yaml_helpers.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/core/get_env.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <fstream>
#include <future>
#include <string>
#include <thread>

#include "synthetic_world.h"

namespace
{
using namespace lo_test;

void set_env(const std::string & name, const std::string & value)
{
  ::setenv(name.c_str(), value.c_str(), 1);
}

// A short straight path inside the hall, every 0.2 m, one scan each 0.1 s:
const auto kStartPose = mrpt::poses::CPose3D::FromXYZYawPitchRoll(3.0, 14.0, 0, 0, 0, 0);
constexpr int kNumScans = 25;

mrpt::poses::CPose3D pathPose(int i)
{
  return kStartPose + mrpt::poses::CPose3D::FromTranslation(0.2 * i, 0, 0);
}

double scanTime(int i) { return 1000.0 + 0.1 * i; }

struct Fixture
{
  mola::LidarOdometry::Ptr liodom = mola::LidarOdometry::Create();
  mola::state_estimation_simple::StateEstimationSimple::Ptr stateEstimator =
    mola::state_estimation_simple::StateEstimationSimple::Create();

  // Keeps both modules visible to each other, so LO can find the state estimator:
  mola::MinimalModuleContainer moduleContainer = {{liodom, stateEstimator}};
  World world = unique_hall();
  std::string tmpBase = mrpt::system::getTempFileName();

  explicit Fixture(bool generateSimplemap = false, const std::string & tracesFile = {})
  {
    set_env("MOLA_MAPPING_ENABLED", "true");
    set_env("MOLA_LOAD_MM", "");
    set_env("MOLA_LO_INITIAL_LOCALIZATION_METHOD", "InitLocalization::FixedPose");
    set_env("MOLA_INITIAL_X", std::to_string(kStartPose.x()));
    set_env("MOLA_INITIAL_Y", std::to_string(kStartPose.y()));
    set_env("MOLA_INITIAL_YAW", "0");
    set_env("MOLA_SAVE_TRAJECTORY", "false");
    set_env("MOLA_GENERATE_SIMPLEMAP", generateSimplemap ? "true" : "false");
    set_env("MOLA_SIMPLEMAP_OUTPUT", tmpBase + "_final.simplemap");
    set_env("MOLA_SAVE_DEBUG_TRACES", tracesFile.empty() ? "false" : "true");
    set_env("MOLA_DEBUG_TRACES_FILE", tracesFile);

    liodom->setMinLoggingLevel(mrpt::system::LVL_ERROR);
    liodom->initialize(mola::load_yaml_file(mrpt::get_env<std::string>("LO_PIPELINE_YAML")));

    stateEstimator->setMinLoggingLevel(mrpt::system::LVL_ERROR);
    stateEstimator->initialize(
      mola::load_yaml_file(mrpt::get_env<std::string>("LO_STATE_ESTIM_YAML")));
  }

  void feedScan(int i)
  {
    liodom->onNewObservation(scan(world, pathPose(i), scanTime(i)));
    wait_idle(*liodom);
  }

  void feedScans(int first, int count)
  {
    for (int i = first; i < first + count; i++) {
      feedScan(i);
    }
  }

  void expectTrackedPath(size_t minPoses) const
  {
    const auto traj = liodom->estimatedTrajectory();
    ASSERT_GE(traj.size(), minPoses);
    const auto last = liodom->lastEstimatedState();
    ASSERT_TRUE(last.has_value());
  }
};

mrpt::obs::CObservationPointCloud::Ptr emptyScan(double t)
{
  auto obs = mrpt::obs::CObservationPointCloud::Create();
  obs->sensorLabel = "lidar";
  obs->timestamp = mrpt::Clock::fromDouble(t);
  obs->pointcloud = mrpt::maps::CSimplePointsMap::Create();
  return obs;
}
}  // namespace

// A first scan without points (e.g. from a driver spinning up) must be
// skipped, not be a fatal error that discards everything that follows.
TEST(Robustness, EmptyFirstScanIsSkipped)
{
  Fixture f;
  f.liodom->onNewObservation(emptyScan(scanTime(0) - 0.1));
  wait_idle(*f.liodom);

  f.feedScans(0, kNumScans);
  f.expectTrackedPath(kNumScans / 2);
}

// reset() rebuilds everything from the original configuration: it must not
// accumulate configuration entries, nor break scan processing.
TEST(Robustness, ResetTwiceThenProcess)
{
  Fixture f;
  f.feedScans(0, 5);

  const auto numLabels = f.liodom->params_.lidar_sensor_labels.size();
  f.liodom->reset();
  f.liodom->reset();
  EXPECT_EQ(f.liodom->params_.lidar_sensor_labels.size(), numLabels);

  f.feedScans(5, kNumScans);
  f.expectTrackedPath(kNumScans / 2);
}

// With twist optimization requesting a restart on every ICP iteration, the
// number of restarts per scan must respect its configured maximum.
TEST(Robustness, OptimizeTwistRespectsMaxCorrections)
{
  const auto traces = mrpt::system::getTempFileName() + "_traces.csv";
  constexpr size_t maxCorrections = 3;
  {
    Fixture f(false, traces);
    f.liodom->params_.optimize_twist = true;
    f.liodom->params_.optimize_twist_rerun_min_trans = 0;
    f.liodom->params_.optimize_twist_rerun_min_rot_deg = 0;
    f.liodom->params_.optimize_twist_max_corrections = maxCorrections;

    f.feedScans(0, kNumScans);
    f.expectTrackedPath(kNumScans / 2);
  }

  std::ifstream csv(traces);
  ASSERT_TRUE(csv.is_open());
  std::string line;
  ASSERT_TRUE(std::getline(csv, line));
  std::vector<std::string> columns;
  mrpt::system::tokenize(line, ",", columns);
  const auto it = std::find(columns.begin(), columns.end(), "\"twistCorrectionCount\"");
  ASSERT_NE(it, columns.end());
  const auto col = static_cast<size_t>(std::distance(columns.begin(), it));

  // Rows with variables that did not exist yet when the header was written
  // have their columns shifted: skip them.
  double maxSeen = 0;
  size_t rowsChecked = 0;
  while (std::getline(csv, line)) {
    std::vector<std::string> cells;
    mrpt::system::tokenize(line, ",", cells);
    if (cells.size() != columns.size()) {
      continue;
    }
    rowsChecked++;
    const double n = std::stod(cells[col]);
    EXPECT_LE(n, static_cast<double>(maxCorrections));
    maxSeen = std::max(maxSeen, n);
  }
  EXPECT_GT(rowsChecked, 0U);
  EXPECT_GT(maxSeen, 0.0);
}

// GNSS arriving from another input thread while scans are processed and
// keyframes are added to the simplemap.
TEST(Robustness, GnssWhileBuildingSimplemap)
{
  Fixture f(true /*simplemap*/);
  const auto georef = make_georef();

  // GNSS stamps follow the scan being processed, strictly increasing:
  std::atomic<double> currentScanTime = scanTime(0);
  std::atomic_bool done = false;
  std::thread gnssThread([&]() {
    double lastT = 0;
    while (!done) {
      const double t = std::max(lastT + 1e-4, currentScanTime.load());
      lastT = t;
      f.liodom->onNewObservation(gnss(georef, kStartPose, {0, 0}, 0.1, t));
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
  });

  for (int i = 0; i < kNumScans; i++) {
    currentScanTime = scanTime(i);
    f.feedScan(i);
  }
  done = true;
  gnssThread.join();
  f.expectTrackedPath(kNumScans / 2);

  const auto sm = f.liodom->reconstructedMap();
  ASSERT_GT(sm.size(), 0U);
  size_t kfsWithGnss = 0;
  for (const auto & kf : sm) {
    if (kf.sf && kf.sf->getObservationByClass<mrpt::obs::CObservationGPS>()) {
      kfsWithGnss++;
    }
  }
  EXPECT_GT(kfsWithGnss, 0U);
}

// map_save() requests are served from spinOnce() too: saving must not race
// with the map updates of scans being processed meanwhile.
TEST(Robustness, MapSaveWhileProcessing)
{
  Fixture f(true /*simplemap*/);
  f.feedScans(0, 5);

  std::atomic_bool done = false;
  std::thread spinThread([&]() {
    while (!done) {
      f.liodom->spinOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  std::vector<std::future<mola::MapServer::ReturnStatus>> saves;
  for (int k = 0; k < 3; k++) {
    saves.push_back(std::async(std::launch::async, [&f, k]() {
      return f.liodom->map_save(f.tmpBase + "_save" + std::to_string(k));
    }));
  }

  f.feedScans(5, kNumScans);

  for (auto & s : saves) {
    const auto ret = s.get();
    EXPECT_TRUE(ret.success) << ret.error_message;
  }
  done = true;
  spinThread.join();

  for (int k = 0; k < 3; k++) {
    mp2p_icp::metric_map_t mm;
    EXPECT_TRUE(mm.load_from_file(f.tmpBase + "_save" + std::to_string(k) + ".mm"));
    EXPECT_FALSE(mm.empty());
  }
}
