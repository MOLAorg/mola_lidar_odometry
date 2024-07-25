# Unit tests:

find_package(mola_test_datasets QUIET)

if(NOT mola_test_datasets_FOUND)
  message(STATUS "WARNING: Skipping unit tests since mola_test_datasets was not found")
  return()
endif()

message(STATUS "mola_test_datasets: ${mola_test_datasets_DIR}")

set(DEFAULT_PIPELINE_YAML ${mola_lidar_odometry_SOURCE_DIR}/pipelines/lidar3d-default.yaml)

set(KITTI00_DATASET_FILE "${mola_test_datasets_DIR}/../datasets/kitti/kitti_00_extract.rawlog")
set(KITTI00_GT_TUM ${mola_lidar_odometry_SOURCE_DIR}/test/kitti_00_fragment_gt.tum)


# Test dynamic parameters
ament_add_gtest(test_lidar_odometry_kitti
  test_lidar_odometry_kitti.cpp
)
ament_target_dependencies(test_lidar_odometry_kitti
  mrpt-obs
)
target_link_libraries(test_lidar_odometry_kitti
  mola::mola_lidar_odometry
)

set_property(
  TEST test_lidar_odometry_kitti
  PROPERTY ENVIRONMENT
  "LO_PIPELINE_YAML=${DEFAULT_PIPELINE_YAML};LO_TEST_RAWLOG=${KITTI00_DATASET_FILE};LO_TEST_GT_TUM=${KITTI00_GT_TUM}"
)


