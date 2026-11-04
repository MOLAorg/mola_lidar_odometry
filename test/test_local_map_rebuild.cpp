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
 * @file   test_local_map_rebuild.cpp
 * @brief  The local map rebuilt from the keyframes of a central map, after one
 *         of its corrections: on a synthetic hall, with a central map that also
 *         has keyframes of an earlier visit to a place this run has not seen,
 *         at poses other than where their scans were taken (as corrected).
 */

#include <gtest/gtest.h>
#include <mola_kernel/MinimalModuleContainer.h>
#include <mola_lidar_odometry/LidarOdometry.h>
#include <mola_state_estimation_simple/StateEstimationSimple.h>
#include <mola_yaml/yaml_helpers.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/core/get_env.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/system/filesystem.h>

#include <chrono>
#include <future>
#include <mutex>
#include <thread>

#include "synthetic_world.h"

#if defined(MOLA_LO_HAS_LOCAL_MAP_REBUILD)

namespace mola
{
/** A central map like a global mapper: it keeps the keyframes pushed by LO,
 *  plus some of an earlier visit, and gives them back with the poses it has. */
class MockCentralMap : public ExecutableBase,
                       public SharedKeyframeMap,
                       public LocalMapCorrectionSource
{
  DEFINE_MRPT_OBJECT(MockCentralMap, mola)

public:
  MockCentralMap() = default;
  ~MockCentralMap() override = default;

  void initialize(const mrpt::containers::yaml &) override {}
  void spinOnce() override {}

  std::optional<KeyFrameID> requestInsertKeyframe(const KeyframeInsertRequest & req) override
  {
    auto lck = mrpt::lockHelper(mtx_);
    kfs_.push_back({req.timestamp, req.pose_in_source.mean, req.observations});
    return kfs_.size() - 1;
  }

  /// A keyframe of an earlier visit, in the same frame as LO's.
  void add_earlier_keyframe(const mrpt::poses::CPose3D & pose, mrpt::obs::CSensoryFrame sf)
  {
    auto lck = mrpt::lockHelper(mtx_);
    earlier_.push_back({mrpt::Clock::fromDouble(1.0), pose, std::move(sf)});
  }

  [[nodiscard]] std::size_t pushed_count() const
  {
    auto lck = mrpt::lockHelper(mtx_);
    return kfs_.size();
  }

  void correct()
  {
    LocalMapCorrection c;
    c.new_loop_closures = 1;
    advertise_local_map_correction(c);
  }

private:
  struct Kf
  {
    mrpt::Clock::time_point stamp;
    mrpt::poses::CPose3D pose;
    mrpt::obs::CSensoryFrame sf;
  };

  [[nodiscard]] std::optional<std::vector<LocalMapKeyframe>> local_map_keyframes_impl(
    KeyFrameID reference, double radius) const override
  {
    auto lck = mrpt::lockHelper(mtx_);
    if (reference >= kfs_.size()) {
      return std::nullopt;
    }
    const auto & ref = kfs_.at(reference).pose;
    std::vector<LocalMapKeyframe> out;
    const auto add = [&](const Kf & kf, KeyFrameID id) {
      if ((kf.pose.translation() - ref.translation()).norm() > radius) {
        return;
      }
      LocalMapKeyframe k;
      k.id = id;
      k.timestamp = kf.stamp;
      k.source_frame_id = "test";
      k.pose_from_reference = kf.pose - ref;
      k.observations = kf.sf;
      out.push_back(std::move(k));
    };
    for (std::size_t i = 0; i < earlier_.size(); i++) {
      add(earlier_[i], kfs_.size() + i);
    }
    for (std::size_t i = 0; i < kfs_.size(); i++) {
      add(kfs_[i], i);
    }
    return out;
  }

  mutable std::mutex mtx_;
  std::vector<Kf> kfs_;
  std::vector<Kf> earlier_;
};

IMPLEMENTS_MRPT_OBJECT(MockCentralMap, ExecutableBase, mola)
}  // namespace mola

namespace
{
using namespace lo_test;
using mrpt::poses::CPose3D;

void set_env(const std::string & name, const std::string & value)
{
  ::setenv(name.c_str(), value.c_str(), 1);
}

mp2p_icp::metric_map_t save_local_map(mola::LidarOdometry & lo)
{
  // map_save() waits for the request to run in spinOnce():
  const auto base = mrpt::system::getTempFileName();
  auto saving = std::async(std::launch::async, [&]() { return lo.map_save(base); });
  while (saving.wait_for(std::chrono::milliseconds(5)) != std::future_status::ready) {
    lo.spinOnce();
  }
  const auto ret = saving.get();
  EXPECT_TRUE(ret.success) << ret.error_message;
  mp2p_icp::metric_map_t mm;
  EXPECT_TRUE(mm.load_from_file(base + ".mm"));
  mrpt::system::deleteFile(base + ".mm");
  mrpt::system::deleteFile(base + ".simplemap");
  return mm;
}

/// Points of the map within an axis-aligned box.
std::size_t points_in_box(
  const mp2p_icp::metric_map_t & mm, const mrpt::math::TPoint3D & lo,
  const mrpt::math::TPoint3D & hi)
{
  std::size_t n = 0;
  for (const auto & [name, layer] : mm.layers) {
    const auto * pts = mp2p_icp::MapToPointsMap(*layer);
    if (pts == nullptr) {
      continue;
    }
    for (std::size_t i = 0; i < pts->size(); i++) {
      mrpt::math::TPoint3D p;
      pts->getPoint(i, p);
      if (p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y && p.z >= lo.z && p.z <= hi.z) {
        n++;
      }
    }
  }
  return n;
}

}  // namespace

TEST(LocalMapRebuild, ReusesEarlierVisitWithoutPoseJump)
{
  const auto world = unique_hall();
  // A diagonal run, so the reference keyframe is rotated:
  constexpr double YAW_DEG = 20.0;
  const auto p0 = CPose3D::FromXYZYawPitchRoll(3.0, 10.5, 0, mrpt::DEG2RAD(YAW_DEG), 0, 0);

  set_env("MOLA_MAPPING_ENABLED", "true");
  set_env("MOLA_LO_PUBLISH_REF_FRAME", "odom");
  set_env("MOLA_LOAD_MM", "");
  set_env("MOLA_LO_INITIAL_LOCALIZATION_METHOD", "InitLocalization::FixedPose");
  set_env("MOLA_INITIAL_X", std::to_string(p0.x()));
  set_env("MOLA_INITIAL_Y", std::to_string(p0.y()));
  set_env("MOLA_INITIAL_YAW", std::to_string(YAW_DEG));

  auto liodom = mola::LidarOdometry::Create();
  auto stateEstimator = mola::state_estimation_simple::StateEstimationSimple::Create();
  auto central = std::make_shared<mola::MockCentralMap>();
  mola::MinimalModuleContainer moduleContainer = {{liodom, stateEstimator, central}};

  // A pillar side only visible from the north (the run goes south of it). The
  // central map has the earlier keyframes 0.3 m north of where their scans were
  // taken, so that side must appear in the rebuilt map 0.3 m north too:
  constexpr double PILLAR_X = 35.0;
  constexpr double PILLAR_NORTH_Y = 21.3;
  constexpr double SHIFT_Y = 0.3;
  const mrpt::math::TPoint3D trueFaceLo(PILLAR_X - 0.2, PILLAR_NORTH_Y - 0.1, 0.3);
  const mrpt::math::TPoint3D trueFaceHi(PILLAR_X + 0.2, PILLAR_NORTH_Y + 0.1, 2.5);
  const mrpt::math::TPoint3D shiftedFaceLo(PILLAR_X - 0.2, PILLAR_NORTH_Y + SHIFT_Y - 0.1, 0.3);
  const mrpt::math::TPoint3D shiftedFaceHi(PILLAR_X + 0.2, PILLAR_NORTH_Y + SHIFT_Y + 0.1, 2.5);
  for (const auto & pose :
       {CPose3D::FromXYZYawPitchRoll(37.0, 23.0, 0, M_PI, 0, 0),
        CPose3D::FromXYZYawPitchRoll(33.0, 23.0, 0, M_PI, 0, 0)}) {
    mrpt::obs::CSensoryFrame sf;
    sf.insert(scan(world, pose, 1.0));
    central->add_earlier_keyframe(CPose3D::FromTranslation(0, SHIFT_Y, 0) + pose, sf);
  }

  stateEstimator->setMinLoggingLevel(mrpt::system::LVL_WARN);
  stateEstimator->initialize(
    mola::load_yaml_file(mrpt::get_env<std::string>("LO_STATE_ESTIM_YAML")));
  liodom->setMinLoggingLevel(mrpt::system::LVL_WARN);
  liodom->initialize(mola::load_yaml_file(mrpt::get_env<std::string>("LO_PIPELINE_YAML")));
  liodom->params_.simplemap.generate = false;
  liodom->params_.estimated_trajectory.output_file.clear();
  liodom->params_.lidar_sensor_labels.assign(1, std::regex("lidar"));

  // Straight ahead at 2 m/s, the wheel odometry being the truth:
  double t = 0;
  auto drive = [&](int steps) {
    for (int i = 0; i < steps; i++) {
      const auto p = p0 + CPose3D::FromTranslation(0.2 * (t * 10), 0, 0);
      liodom->onNewObservation(odometry(mrpt::poses::CPose2D(p), t));
      liodom->onNewObservation(scan(world, p, t));
      wait_idle(*liodom);
      liodom->spinOnce();
      t += 0.1;
    }
  };
  drive(40);
  ASSERT_GT(central->pushed_count(), 2U);

  const auto before = save_local_map(*liodom);
  const auto nTrueBefore = points_in_box(before, trueFaceLo, trueFaceHi);
  const auto nShiftedBefore = points_in_box(before, shiftedFaceLo, shiftedFaceHi);

  // The vehicle goes on while the map is rebuilt in the background:
  central->correct();
  int steps = 40;
  while (liodom->localMapRebuildCount() == 0 && steps < 120) {
    drive(1);
    steps++;
  }
  ASSERT_EQ(liodom->localMapRebuildCount(), 1U);
  std::cout << "Rebuilt after " << (steps - 40) << " more scans\n";

  const auto after = save_local_map(*liodom);
  const auto nTrueAfter = points_in_box(after, trueFaceLo, trueFaceHi);
  const auto nShiftedAfter = points_in_box(after, shiftedFaceLo, shiftedFaceHi);
  std::cout << "Local map before the rebuild: " << before.contents_summary()
            << ", hidden pillar side points " << nTrueBefore << " (shifted: " << nShiftedBefore
            << "); after: " << after.contents_summary() << ", " << nTrueAfter << " ("
            << nShiftedAfter << ")\n";
  EXPECT_EQ(nTrueBefore + nShiftedBefore, 0U);
  EXPECT_EQ(nTrueAfter, 0U);
  EXPECT_GT(nShiftedAfter, 10U);

  // On the rebuilt map, the odometry goes on without a jump:
  drive(30);
  steps += 30;
  const auto traj = liodom->estimatedTrajectory();
  ASSERT_GT(traj.size(), 60U);
  double maxStep = 0;
  std::optional<CPose3D> prev;
  for (const auto & [stamp, pose] : traj) {
    const CPose3D p(pose);
    if (prev) {
      maxStep = std::max(maxStep, (p.translation() - prev->translation()).norm());
    }
    prev = p;
  }
  const auto truthEnd = p0 + CPose3D::FromTranslation(0.2 * (steps - 1), 0, 0);
  const double endErr = (prev->translation() - truthEnd.translation()).norm();
  std::cout << "Max step between scans " << maxStep << " m, end position error " << endErr
            << " m\n";
  EXPECT_LT(maxStep, 0.3);
  EXPECT_LT(endErr, 0.10);
  EXPECT_EQ(liodom->localMapRebuildCount(), 1U);
}

#else
TEST(LocalMapRebuild, Skipped) { GTEST_SKIP() << "mola_kernel without LocalMapCorrectionSource"; }
#endif

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
