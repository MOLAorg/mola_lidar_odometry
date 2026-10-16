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
 *         empty scans, invalid input, reset(), optimize_twist, GNSS input,
 *         map saving and settings changes while scans are being processed.
 *         Best run under ASan/TSan too.
 */

#include <gtest/gtest.h>
#include <mola_kernel/MinimalModuleContainer.h>
#include <mola_lidar_odometry/LidarOdometry.h>
#include <mola_state_estimation_simple/StateEstimationSimple.h>
#include <mola_yaml/yaml_helpers.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/get_env.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <fstream>
#include <future>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

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

// With simplemap generation disabled, not even the first scan may add a
// keyframe: it would be written out by map_save and returned by
// reconstructedMap().
TEST(Robustness, NoSimplemapKeyframesWhenDisabled)
{
  Fixture f(false /*simplemap*/);
  f.feedScans(0, 5);
  f.expectTrackedPath(3);
  EXPECT_EQ(f.liodom->reconstructedMap().size(), 0U);
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

// Settings changed from other threads (as a GUI does) apply while the LiDAR
// worker and the executor thread run, and their readers see consistent values.
TEST(Robustness, SettingsChangedWhileProcessing)
{
  Fixture f(true /*simplemap*/);
  f.feedScans(0, 3);

  std::atomic_bool done = false;
  std::thread spinThread([&]() {
    while (!done) {
      f.liodom->spinOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
  std::thread settingsThread([&]() {
    for (int k = 0; !done; k++) {
      const std::string smFile = f.tmpBase + "_sm" + std::to_string(k % 2) + ".simplemap";
      f.liodom->enqueue_locked_request([&f, smFile]() {
        f.liodom->params_.simplemap.save_final_map_to_file = smFile;
        f.liodom->params_.estimated_trajectory.output_file = smFile + ".tum";
      });
      f.liodom->setTrajectoryVisualization(k % 2 == 0, {1.0f, 0.0f, 0.0f, 1.0f});
      f.liodom->setLocalMapColoring(mrpt::img::cmJET, k % 2 == 0 ? "z" : "intensity");
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  });
  std::thread saveThread([&]() {
    while (!done) {
      f.liodom->saveEstimatedTrajectoryToFile();
      f.liodom->saveReconstructedMapToFile();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });

  f.feedScans(3, kNumScans);

  done = true;
  spinThread.join();
  settingsThread.join();
  saveThread.join();

  f.expectTrackedPath(kNumScans / 2);
}

// reset() replaces the whole state while other threads read the trajectory,
// the simplemap and the flags through the public, thread-safe accessors.
TEST(Robustness, ResetWhileReading)
{
  Fixture f(true /*simplemap*/);

  std::atomic_bool done = false;
  std::thread readerThread([&]() {
    while (!done) {
      (void)f.liodom->estimatedTrajectory();
      (void)f.liodom->reconstructedMap();
      (void)f.liodom->isActive();
      (void)f.liodom->isBusy();
    }
  });

  for (int r = 0; r < 3; r++) {
    f.feedScans(0, 5);
    f.liodom->reset();
  }
  f.feedScans(0, kNumScans);

  done = true;
  readerThread.join();

  f.expectTrackedPath(kNumScans / 2);
}

namespace
{
mrpt::obs::CObservationOdometry::Ptr odometryLabeled(const std::string & label, double t)
{
  auto obs = odometry(mrpt::poses::CPose2D(), t);
  obs->sensorLabel = label;
  return obs;
}

// A scan whose lazy-load file does not exist: loading it throws.
mrpt::obs::CObservationPointCloud::Ptr unreadableScan(double t)
{
  auto obs = mrpt::obs::CObservationPointCloud::Create();
  obs->sensorLabel = "lidar";
  obs->timestamp = mrpt::Clock::fromDouble(t);
  obs->setAsExternalStorage(
    "/nonexistent/scan.bin",
    mrpt::obs::CObservationPointCloud::ExternalStorageFormat::KittiBinFile);
  return obs;
}

mrpt::obs::CObservationIMU::Ptr nonFiniteImu(double t)
{
  auto obs = imu(kStartPose, t);
  obs->set(mrpt::obs::IMU_WZ, std::numeric_limits<double>::quiet_NaN());
  return obs;
}

mrpt::obs::CObservationGPS::Ptr nonFiniteCovarianceGnss(double t)
{
  auto obs = gnss(make_georef(), kStartPose, {0, 0}, 0.1, t);
  (*obs->covariance_enu)(0, 1) = std::numeric_limits<double>::quiet_NaN();
  return obs;
}
}  // namespace

// Malformed input (an observation of the wrong class for its sensor label, an
// unreadable scan, non-finite readings) must be dropped and counted, not stop
// the odometry.
TEST(Robustness, InvalidInputIsDropped)
{
  Fixture f;
  f.feedScans(0, 5);

  f.liodom->onNewObservation(odometryLabeled("imu", scanTime(5) - 0.05));
  f.liodom->onNewObservation(odometryLabeled("gps", scanTime(5) - 0.04));
  f.liodom->onNewObservation(nonFiniteImu(scanTime(5) - 0.03));
  f.liodom->onNewObservation(nonFiniteCovarianceGnss(scanTime(5) - 0.025));
  f.liodom->onNewObservation(unreadableScan(scanTime(5) - 0.02));
  wait_idle(*f.liodom);  // else the scan worker may drop it, unprocessed, for the next one
  f.liodom->onNewObservation(odometryLabeled("lidar", scanTime(5) - 0.01));
  wait_idle(*f.liodom);
  EXPECT_EQ(f.liodom->invalidInputCount(), 6U);

  f.feedScans(5, kNumScans - 5);
  f.expectTrackedPath(kNumScans / 2);
  ASSERT_GE(f.liodom->estimatedTrajectory().size(), static_cast<size_t>(kNumScans - 5));
}

// The same for the very first scan, before anything has been initialized.
TEST(Robustness, UnreadableFirstScanIsDropped)
{
  Fixture f;
  f.liodom->onNewObservation(unreadableScan(scanTime(0) - 0.1));
  wait_idle(*f.liodom);
  EXPECT_EQ(f.liodom->invalidInputCount(), 1U);

  f.feedScans(0, kNumScans);
  ASSERT_GE(f.liodom->estimatedTrajectory().size(), static_cast<size_t>(kNumScans - 5));
}

// The same checks apply to a dedicated attitude IMU.
TEST(Robustness, InvalidAttitudeImuIsDropped)
{
  set_env("MOLA_ATTITUDE_IMU_NAME", "attitude_imu");
  Fixture f;
  ::unsetenv("MOLA_ATTITUDE_IMU_NAME");

  auto bad = nonFiniteImu(scanTime(0) - 0.1);
  bad->sensorLabel = "attitude_imu";
  f.liodom->onNewObservation(bad);
  f.liodom->onNewObservation(odometryLabeled("attitude_imu", scanTime(0) - 0.05));
  EXPECT_EQ(f.liodom->invalidInputCount(), 2U);

  f.feedScans(0, kNumScans);
  f.expectTrackedPath(kNumScans / 2);
}

// Getters, diagnostics and parameter changes are called from the ROS executor,
// which also serves the sensor subscriptions: they must not wait while a slow
// scan (or a global search, or a map load) holds the state.
TEST(Robustness, StatusDoesNotWaitForSlowScans)
{
  using namespace std::chrono_literals;

  Fixture f;
  f.feedScans(0, 5);
  const auto before = f.liodom->lastEstimatedState();
  ASSERT_TRUE(before.has_value());

  // Holds the state lock for a while, as a slow scan does:
  std::promise<void> started;
  auto startedFut = started.get_future();
  f.liodom->enqueue_locked_request([&started]() {
    started.set_value();
    std::this_thread::sleep_for(2s);
  });
  std::thread spinner([&]() { f.liodom->spinOnce(); });
  startedFut.wait();

  const auto t0 = std::chrono::steady_clock::now();
  const auto st = f.liodom->lastEstimatedState();
  (void)f.liodom->lastIcpQuality();
  std::vector<mola::DiagnosticStatusMsg> diag;
  static_cast<mola::DiagnosticsProvider &>(*f.liodom).getDiagnostics(diag);
  f.liodom->changeParameters(mrpt::containers::yaml::FromText("mapping_enabled: false"));
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  spinner.join();
  EXPECT_LT(std::chrono::duration<double>(elapsed).count(), 0.5);

  ASSERT_TRUE(st.has_value());
  EXPECT_NEAR((std::get<0>(*st).mean - std::get<0>(*before).mean).norm(), 0.0, 1e-9);
  EXPECT_FALSE(diag.empty());

  // The parameter change is applied by the next request drain:
  f.liodom->spinOnce();
  EXPECT_FALSE(f.liodom->params_.local_map_updates.enabled);
}

// The local map is copied out on a worker thread, and only once there is a
// subscriber: until then the update stays pending, not lost.
TEST(Robustness, LocalMapIsPublishedOnceSubscribed)
{
  using namespace std::chrono_literals;

  for (const char * mapClass : {"mola::KeyframePointCloudMap", "mola::IncrementalPointCloud"}) {
    // Declared before the fixture, so they outlive its publishing worker:
    std::mutex mtx;
    std::map<std::string, mrpt::maps::CMetricMap::ConstPtr> received;

    set_env("MOLA_PUBLISH_LOCAL_MAP_UPDATES_EVERY_N", "1");
    set_env("MOLA_LOCALMAP_CLASS", mapClass);
    Fixture f;
    ::unsetenv("MOLA_PUBLISH_LOCAL_MAP_UPDATES_EVERY_N");
    ::unsetenv("MOLA_LOCALMAP_CLASS");

    f.feedScans(0, 10);

    f.liodom->subscribeToMapUpdates([&](const mola::MapSourceBase::MapUpdate & mu) {
      std::lock_guard<std::mutex> lck(mtx);
      received[mu.map_name] = mu.map;
    });
    {
      // Nothing was published without a subscriber, so nothing is replayed:
      std::lock_guard<std::mutex> lck(mtx);
      EXPECT_EQ(received.count("localmap"), 0U) << mapClass;
    }

    f.feedScan(10);

    bool gotMap = false;
    for (int i = 0; i < 500 && !gotMap; i++) {
      {
        std::lock_guard<std::mutex> lck(mtx);
        gotMap = received.count("localmap") != 0;
      }
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(gotMap) << mapClass;
    std::lock_guard<std::mutex> lck(mtx);
    const auto & map = received["localmap"];
    ASSERT_TRUE(map) << mapClass;
    EXPECT_FALSE(map->isEmpty()) << mapClass;
    // Point maps are sent as a plain copy:
    if (const auto pts = std::dynamic_pointer_cast<const mrpt::maps::CPointsMap>(map); pts) {
      EXPECT_GT(pts->size(), 1000U) << mapClass;
    }
  }
}

// The default local map is a keyframe map, not a points map: its size has no
// cheap generic query, so diagnostics must say so rather than report 0 points.
TEST(Robustness, LocalMapDiagnosticsPoints)
{
  Fixture f;
  f.feedScans(0, 5);

  std::vector<mola::DiagnosticStatusMsg> diag;
  static_cast<mola::DiagnosticsProvider &>(*f.liodom).getDiagnostics(diag);

  const auto it = std::find_if(
    diag.begin(), diag.end(), [](const auto & s) { return s.name == "LidarOdometry: Local Map"; });
  ASSERT_NE(it, diag.end());

  std::map<std::string, std::string> values;
  for (const auto & kv : it->values) {
    values[kv.key] = kv.value;
  }
  EXPECT_EQ(values["num_layers"], "1");
  EXPECT_EQ(values["points.localmap"], "n/a");
  EXPECT_EQ(values["total_points"], "n/a");
}
