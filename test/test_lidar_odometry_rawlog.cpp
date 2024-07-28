// -----------------------------------------------------------------------------
//   A Modular Optimization framework for Localization and mApping  (MOLA)
//
// Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
// Licensed under the GNU GPL v3.
//
// This file is part of MOLA.
// MOLA is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// MOLA. If not, see <https://www.gnu.org/licenses/>.
//
// Closed-source licenses available upon request, for this odometry package
// alone or in combination with the complete SLAM system.
// -----------------------------------------------------------------------------

#include <gtest/gtest.h>
#include <mola_kernel/interfaces/OfflineDatasetSource.h>
#include <mola_kernel/pretty_print_exception.h>
#include <mola_lidar_odometry/LidarOdometry.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/rtti/CObject.h>

namespace
{

static int main_odometry(
  const std::string & yamlConfigFile, const std::string & rawlogFile,
  const std::string & gtTrajectory)
{
  mola::LidarOdometry liodom;

  // Initialize LiDAR Odometry:
  const auto cfg = mola::load_yaml_file(yamlConfigFile);

  // quiet for the unit tests:
  liodom.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  liodom.initialize(cfg);

  liodom.params_.simplemap.generate = false;
  liodom.params_.estimated_trajectory.output_file.clear();

  liodom.params_.lidar_sensor_labels.assign(1, std::regex("lidar"));

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
    if (!obs) obs = sf.getObservationByClass<CObservationPointCloud>();
    if (!obs) obs = sf.getObservationByClass<CObservationOdometry>();
    if (!obs) continue;

    // Send it to the odometry pipeline:
    liodom.onNewObservation(obs);

    while (liodom.isBusy()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  const mrpt::poses::CPose3DInterpolator trajectory = liodom.estimatedTrajectory();

  mrpt::poses::CPose3DInterpolator gt;
  const bool gtLoadOk = gt.loadFromTextFile_TUM(gtTrajectory);
  ASSERT_(gtLoadOk);

  ASSERT_EQUAL_(trajectory.size(), 3U);
  ASSERT_EQUAL_(gt.size(), trajectory.size());

  int failed_tests = 0;

  auto itP = trajectory.cbegin();
  auto itGT = gt.cbegin();

  for (; itP != trajectory.cend(); ++itP, ++itGT) {
    const auto pose = itP->second;
    const auto gt = itGT->second;

    const double err = mrpt::poses::Lie::SE<3>::log(mrpt::poses::CPose3D(gt - pose)).norm();

    EXPECT_LT(err, 0.1) << "Estimated trajectory pose mismatch:\n"
                        << " LO pose: " << pose << "\n"
                        << " GT pose: " << gt << "\n"
                        << ++failed_tests;
  }

  if (failed_tests)
    std::cout << "Test failed\n";
  else
    std::cout << "Test passed\n";

  return 0;
}

}  // namespace

TEST(RunDataset, FromRawlog)
{
  const std::string yamlConfigFile = mrpt::get_env<std::string>("LO_PIPELINE_YAML");
  const std::string rawlogFile = mrpt::get_env<std::string>("LO_TEST_RAWLOG");
  const std::string gtTrajectory = mrpt::get_env<std::string>("LO_TEST_GT_TUM");

  main_odometry(yamlConfigFile, rawlogFile, gtTrajectory);
}

// The main function running all the tests
int main([[maybe_unused]] int argc, [[maybe_unused]] char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
