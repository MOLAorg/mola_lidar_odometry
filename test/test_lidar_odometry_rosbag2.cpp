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
#include <mola_input_rosbag2/Rosbag2Dataset.h>
#include <mola_kernel/MinimalModuleContainer.h>
#include <mola_kernel/interfaces/OfflineDatasetSource.h>
#include <mola_kernel/pretty_print_exception.h>
#include <mola_lidar_odometry/LidarOdometry.h>
#include <mola_state_estimation_simple/StateEstimationSimple.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/rtti/CObject.h>

namespace
{
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_rosbag2(
  const std::string & rosbag2file, const std::string & lidarTopic,
  const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::Rosbag2Dataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      rosbag_filename: '%s'
      base_link_frame_id: 'base_link'
      sensors:
        - topic: '%s'
          type: CObservationPointCloud
          # If present, this will override whatever /tf tells about the sensor pose:
          fixed_sensor_pose: "0 0 0 0 0 0"  # 'x y z yaw_deg pitch_deg roll_deg'
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_LIDAR_POSE|false}
        - topic: ${MOLA_GNSS_TOPIC|'/gps'}
          sensorLabel: 'gps'
          type: CObservationGPS
          fixed_sensor_pose: "0 0 0 0 0 0"  # 'x y z yaw_deg pitch_deg roll_deg'
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_GNSS_POSE|false}
)"""",
    rosbag2file.c_str(), lidarTopic.c_str())));

  o->initialize(cfg);

  return o;
}

int main_odometry(
  const std::string & yamlConfigFile, const std::string & stateEstimConfigFile,
  const std::string & rosbag2File, const std::string & lidarTopic, const std::string & gtTrajectory)
{
  auto liodom = mola::LidarOdometry::Create();
  auto stateEstimator = mola::state_estimation_simple::StateEstimationSimple::Create();

  // Put both modules together so LO can find the state estimator:
  mola::MinimalModuleContainer moduleContainer = {{liodom, stateEstimator}};

  // Initialize LiDAR Odometry:
  const auto cfg = mola::load_yaml_file(yamlConfigFile);

  // quiet for the unit tests:
  liodom->setMinLoggingLevel(mrpt::system::LVL_ERROR);

  liodom->initialize(cfg);

  liodom->params_.simplemap.generate = false;
  liodom->params_.estimated_trajectory.output_file.clear();

  liodom->params_.lidar_sensor_labels.assign(1, std::regex(lidarTopic));

  // Initialize estimator (default settings):
  stateEstimator->setMinLoggingLevel(mrpt::system::LVL_ERROR);
  const auto cfgEstimator = mola::load_yaml_file(stateEstimConfigFile);
  stateEstimator->initialize(cfgEstimator);

  // dataset input:
  std::shared_ptr<mola::OfflineDatasetSource> dataset;
  dataset = dataset_from_rosbag2(rosbag2File, lidarTopic, mrpt::system::LVL_ERROR);

  // Run:
  for (size_t i = 0; i < dataset->datasetSize(); i++) {
    // Get observations from the dataset:
    using mrpt::obs::CObservation2DRangeScan;
    using mrpt::obs::CObservation3DRangeScan;
    using mrpt::obs::CObservationGPS;
    using mrpt::obs::CObservationOdometry;
    using mrpt::obs::CObservationPointCloud;
    using mrpt::obs::CObservationRotatingScan;
    using mrpt::obs::CObservationVelodyneScan;

    const auto sf = dataset->datasetGetObservations(i);
    ASSERT_(sf);

    mrpt::obs::CObservation::Ptr obs;
    obs = sf->getObservationByClass<CObservationPointCloud>();
    if (!obs) obs = sf->getObservationByClass<CObservationOdometry>();

    if (!obs) continue;

    // Send it to the odometry pipeline:
    liodom->onNewObservation(obs);

    while (liodom->isBusy()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  const mrpt::poses::CPose3DInterpolator trajectory = liodom->estimatedTrajectory();

  trajectory.saveToTextFile_TUM("/tmp/gt.tum");

  mrpt::poses::CPose3DInterpolator gt;
  const bool gtLoadOk = gt.loadFromTextFile_TUM(gtTrajectory);
  ASSERT_(gtLoadOk);

  ASSERT_EQUAL_(gt.size(), trajectory.size());

  auto itP = trajectory.cbegin();
  auto itGT = gt.cbegin();

  for (; itP != trajectory.cend(); ++itP, ++itGT) {
    const auto pose = itP->second;
    const auto gt = itGT->second;

    const double err = mrpt::poses::Lie::SE<3>::log(mrpt::poses::CPose3D(gt - pose)).norm();

    EXPECT_LT(err, 0.1) << "Estimated trajectory pose mismatch:\n"
                        << " LO pose: " << pose << "\n"
                        << " GT pose: " << gt << "\n";
  }

  return 0;
}

}  // namespace

TEST(RunDataset, FromRosbag2)
{
  const std::string yamlConfigFile = mrpt::get_env<std::string>("LO_PIPELINE_YAML");
  const std::string yamlEstimatorFile = mrpt::get_env<std::string>("LO_STATE_ESTIM_YAML");
  const std::string rosbag2File = mrpt::get_env<std::string>("LO_TEST_ROSBAG2");
  const std::string lidarTopic = mrpt::get_env<std::string>("LO_TEST_LIDAR_TOPIC");
  const std::string gtTrajectory = mrpt::get_env<std::string>("LO_TEST_GT_TUM");

  main_odometry(yamlConfigFile, yamlEstimatorFile, rosbag2File, lidarTopic, gtTrajectory);
}

// The main function running all the tests
int main([[maybe_unused]] int argc, [[maybe_unused]] char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
