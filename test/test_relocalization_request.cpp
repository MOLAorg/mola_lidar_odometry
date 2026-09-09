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
 * @file   test_relocalization_request.cpp
 * @brief  A relocalization request must select the method that consumes it
 * @author Jose Luis Blanco Claraco
 */

#include <gtest/gtest.h>
#include <mola_kernel/MinimalModuleContainer.h>
#include <mola_lidar_odometry/LidarOdometry.h>
#include <mola_state_estimation_simple/StateEstimationSimple.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/get_env.h>

#include <memory>

namespace
{
struct Fixture
{
  mola::LidarOdometry::Ptr liodom = mola::LidarOdometry::Create();
  mola::state_estimation_simple::StateEstimationSimple::Ptr stateEstimator =
    mola::state_estimation_simple::StateEstimationSimple::Create();

  // Keeps both modules visible to each other, so LO can find the state estimator:
  mola::MinimalModuleContainer moduleContainer = {{liodom, stateEstimator}};

  Fixture()
  {
    liodom->setMinLoggingLevel(mrpt::system::LVL_WARN);
    liodom->initialize(mola::load_yaml_file(mrpt::get_env<std::string>("LO_PIPELINE_YAML")));

    stateEstimator->setMinLoggingLevel(mrpt::system::LVL_WARN);
    stateEstimator->initialize(
      mola::load_yaml_file(mrpt::get_env<std::string>("LO_STATE_ESTIM_YAML")));

    liodom->params_.simplemap.generate = false;
    liodom->params_.estimated_trajectory.output_file.clear();
  }

  /// Relocalization entry points only enqueue the request; spinOnce() runs it.
  void runPendingRequests() { liodom->spinOnce(); }
};

mrpt::poses::CPose3DPDFGaussian someRequestedPose()
{
  mrpt::poses::CPose3DPDFGaussian p;
  p.mean =
    mrpt::poses::CPose3D::FromXYZYawPitchRoll(10.0, -4.0, 0.5, mrpt::DEG2RAD(30.0), 0.0, 0.0);
  p.cov.setDiagonal(std::vector<double>({0.25, 0.25, 0.01, 0.01, 0.01, 0.05}));
  return p;
}
}  // namespace

// A manual request carries the pose in initial_localization.fixed_initial_pose,
// which only the FixedPose method reads. Requesting it from any other
// configured method must switch to FixedPose: otherwise the pose is stored and
// never used, while the same request has already stopped scan processing.
TEST(Relocalization, NearPoseSelectsFixedPoseMethod)
{
  for (const auto configuredMethod :
       {mola::InitLocalization::FromStateEstimator, mola::InitLocalization::PitchAndRollFromIMU,
        mola::InitLocalization::FixedPose}) {
    Fixture f;
    f.liodom->params_.initial_localization.method = configuredMethod;

    const auto p = someRequestedPose();
    f.liodom->relocalize_near_pose_pdf(p);
    f.runPendingRequests();

    const auto & il = f.liodom->params_.initial_localization;

    EXPECT_EQ(il.method, mola::InitLocalization::FixedPose)
      << "configured method index: " << static_cast<int>(configuredMethod);

    EXPECT_NEAR((mrpt::poses::CPose3D(il.fixed_initial_pose) - p.mean).norm(), 0.0, 1e-9);
    ASSERT_TRUE(il.initial_pose_cov.has_value());
    EXPECT_NEAR((*il.initial_pose_cov - p.cov).sum_abs(), 0.0, 1e-9);
  }
}

// The two entry points must remain usable in any order at runtime.
TEST(Relocalization, FromGnssRestoresStateEstimatorMethod)
{
  Fixture f;

  f.liodom->relocalize_near_pose_pdf(someRequestedPose());
  f.runPendingRequests();
  EXPECT_EQ(f.liodom->params_.initial_localization.method, mola::InitLocalization::FixedPose);

  f.liodom->relocalize_from_gnss();
  f.runPendingRequests();
  EXPECT_EQ(
    f.liodom->params_.initial_localization.method, mola::InitLocalization::FromStateEstimator);
}
