# Unit tests:

find_package(mola_test_datasets QUIET)
find_package(mola_metric_maps QUIET)
find_package(mola_input_rosbag2 QUIET)

if((NOT mola_test_datasets_FOUND) OR
   (NOT mola_metric_maps_FOUND) OR
   (NOT mola_input_rosbag2_FOUND)
  )
  message(STATUS "**********************************************************************")
  message(STATUS " WARNING: Skipping unit tests since test dependencies were not found  ")
  message(STATUS "**********************************************************************")
  return()
endif()

#message(STATUS "mola_test_datasets : ${mola_test_datasets_DIR}")
#message(STATUS "mola_metric_maps   : ${mola_metric_maps_DIR}")

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


# Test: from rosbag2 file, RSLIDAR data with XYZIRT channels
# -----------------------------------------------------------
set(RSLIDAR_DATASET_FILE "${mola_test_datasets_DIR}/../datasets/rslidar_warehouse/rslidar_fragment.mcap")
set(RSLIDAR_GT_TUM ${mola_lidar_odometry_SOURCE_DIR}/test/rslidar_fragment_gt.tum)
set(RSLIDAR_TOPIC "/rslidar_points")

if(${mrpt-ros2bridge_VERSION} VERSION_GREATER_EQUAL "2.13.5")
  set(IGNORE_POINT_STAMPS "false")  # this was a bug fixed in 2.13.5, don't make this pkg to fail
else()
  set(IGNORE_POINT_STAMPS "true")
endif()

if(EXISTS "${RSLIDAR_DATASET_FILE}")
  ament_add_gtest(test_lidar_odometry_rslidar test_lidar_odometry_rosbag2.cpp
    ENV LO_PIPELINE_YAML=${DEFAULT_PIPELINE_YAML}
    ENV LO_TEST_ROSBAG2=${RSLIDAR_DATASET_FILE}
    ENV LO_TEST_LIDAR_TOPIC=${RSLIDAR_TOPIC}
    ENV LO_TEST_GT_TUM=${RSLIDAR_GT_TUM}
    ENV MOLA_PROFILER=false
    MOLA_USE_FIXED_LIDAR_POSE=true  # so we don't need /tf topic in test dataset
    MOLA_IGNORE_NO_POINT_STAMPS=${IGNORE_POINT_STAMPS} # make it fail if no timestamps are found
  )
  ament_target_dependencies(test_lidar_odometry_rslidar  mrpt-obs)
  target_link_libraries(test_lidar_odometry_rslidar
    mola::mola_lidar_odometry
    mola::mola_input_rosbag2
  )
else()
  message(STATUS "Skipping test for missing dataset: ${RSLIDAR_DATASET_FILE}")
endif()
