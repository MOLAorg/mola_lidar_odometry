/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

#include <gtest/gtest.h>
#include <mola_kernel/MinimalModuleContainer.h>
#include <mola_kernel/interfaces/OfflineDatasetSource.h>
#include <mola_kernel/pretty_print_exception.h>
#include <mola_lidar_odometry/LidarOdometry.h>
#include <mola_state_estimation_simple/StateEstimationSimple.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/rtti/CObject.h>

namespace
{
struct OdometryTestParams
{
  std::string yamlConfigFile;
  std::string stateEstimConfigFile;
  std::string rawlogFile;
  std::string gtTrajectory;
};

int main_odometry(const OdometryTestParams & p)
{
  const auto & yamlConfigFile = p.yamlConfigFile;
  const auto & stateEstimConfigFile = p.stateEstimConfigFile;
  const auto & rawlogFile = p.rawlogFile;
  const auto & gtTrajectory = p.gtTrajectory;
  auto liodom = mola::LidarOdometry::Create();
  auto stateEstimator = mola::state_estimation_simple::StateEstimationSimple::Create();

  // Put both modules together so LO can find the state estimator:
  mola::MinimalModuleContainer moduleContainer = {{liodom, stateEstimator}};

  // Initialize LiDAR Odometry:
  const auto cfg = mola::load_yaml_file(yamlConfigFile);

  // quiet for the unit tests:
  liodom->setMinLoggingLevel(mrpt::system::LVL_DEBUG);

  liodom->initialize(cfg);

  liodom->params_.simplemap.generate = false;
  liodom->params_.estimated_trajectory.output_file.clear();

  liodom->params_.lidar_sensor_labels.assign(1, std::regex("lidar"));

  // Initialize estimator (default settings):
  stateEstimator->setMinLoggingLevel(mrpt::system::LVL_DEBUG);
  const auto cfgEstimator = mola::load_yaml_file(stateEstimConfigFile);
  stateEstimator->initialize(cfgEstimator);

  // dataset input:
  mrpt::obs::CRawlog dataset;
  bool datasetReadOk = dataset.loadFromRawLogFile(rawlogFile);
  ASSERT_(datasetReadOk);
  ASSERT_GT_(dataset.size(), 2);

  // Run:
  for (size_t i = 0; i < dataset.size(); i++) {
    // Get observations from the dataset:
    using namespace mrpt::obs;

    mrpt::obs::CSensoryFrame sf;
    sf.insert(dataset.getAsObservation(i));

    CObservation::Ptr obs;
    if (!obs) {
      obs = sf.getObservationByClass<CObservationPointCloud>();
    }
    if (!obs) {
      obs = sf.getObservationByClass<CObservationOdometry>();
    }
    if (!obs) {
      obs = sf.getObservationByClass<CObservationIMU>();
    }
    if (!obs) {
      continue;
    }

    // Send it to the odometry pipeline:
    liodom->onNewObservation(obs);

    while (liodom->isBusy()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  const mrpt::poses::CPose3DInterpolator trajectory = liodom->estimatedTrajectory();

  mrpt::poses::CPose3DInterpolator gt;
  const bool gtLoadOk = gt.loadFromTextFile_TUM(gtTrajectory);
  ASSERT_(gtLoadOk);

  ASSERT_GE_(gt.size(), trajectory.size());
  ASSERT_GE_(trajectory.size(), gt.size() - 1);

  auto itP = trajectory.cbegin();
  auto itGT = gt.cbegin();

  for (; itP != trajectory.cend(); ++itP, ++itGT) {
    const auto pose = itP->second;
    const auto gt = itGT->second;

    const double err = mrpt::poses::Lie::SE<3>::log(mrpt::poses::CPose3D(gt - pose)).norm();

    EXPECT_LT(err, 0.40) << "Estimated trajectory pose mismatch:\n"
                         << " LO pose: " << pose << "\n"
                         << " GT pose: " << gt << "\n";
  }

  return 0;
}

}  // namespace

TEST(RunDataset, FromRawlog)
{
  OdometryTestParams params;
  params.yamlConfigFile = mrpt::get_env<std::string>("LO_PIPELINE_YAML");
  params.stateEstimConfigFile = mrpt::get_env<std::string>("LO_STATE_ESTIM_YAML");
  params.rawlogFile = mrpt::get_env<std::string>("LO_TEST_RAWLOG");
  params.gtTrajectory = mrpt::get_env<std::string>("LO_TEST_GT_TUM");

  main_odometry(params);
}

// The main function running all the tests
int main([[maybe_unused]] int argc, [[maybe_unused]] char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
