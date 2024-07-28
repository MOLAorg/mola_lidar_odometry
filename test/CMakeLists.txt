# Unit tests:

find_package(mola_test_datasets QUIET)
if(NOT mola_test_datasets_FOUND)
  message(STATUS "WARNING: Skipping unit tests since mola_test_datasets was not found")
  return()
endif()

find_package(mola_metric_maps QUIET)
if(NOT mola_metric_maps_FOUND)
  message(STATUS "WARNING: Skipping unit tests since mola_metric_maps was not found")
  return()
endif()

message(STATUS "mola_test_datasets : ${mola_test_datasets_DIR}")
message(STATUS "mola_metric_maps   : ${mola_metric_maps_DIR}")

set(DEFAULT_PIPELINE_YAML ${mola_lidar_odometry_SOURCE_DIR}/pipelines/lidar3d-default.yaml)

# Test: from rawlog file, using KITTI fragment
# ---------------------------------------------------
set(KITTI00_DATASET_FILE "${mola_test_datasets_DIR}/../datasets/kitti/kitti_00_extract.rawlog")
set(KITTI00_GT_TUM ${mola_lidar_odometry_SOURCE_DIR}/test/kitti_00_fragment_gt.tum)

ament_add_gtest(test_lidar_odometry_kitti test_lidar_odometry_rawlog.cpp
  ENV LO_PIPELINE_YAML=${DEFAULT_PIPELINE_YAML}
  ENV LO_TEST_RAWLOG=${KITTI00_DATASET_FILE}
  ENV LO_TEST_GT_TUM=${KITTI00_GT_TUM}
  ENV MOLA_PROFILER=false
)
ament_target_dependencies(test_lidar_odometry_kitti  mrpt-obs)
target_link_libraries(test_lidar_odometry_kitti mola::mola_lidar_odometry)


# Test: from rosbag2 file, ouster data with XYZIRT channels
# ----------------------------------------------------------
# ...
