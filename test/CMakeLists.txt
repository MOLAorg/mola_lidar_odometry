# Unit tests:

find_package(mola_test_datasets)
find_package(mola_metric_maps)
find_package(mola_input_rosbag2)
find_package(mola_state_estimation_simple)

if((NOT mola_test_datasets_FOUND) OR
   (NOT mola_metric_maps_FOUND) OR
   (NOT mola_input_rosbag2_FOUND) OR
   (NOT mola_state_estimation_simple_FOUND)
  )
  message(STATUS "**********************************************************************")
  message(STATUS " WARNING: Skipping unit tests since test dependencies were not found  ")
  message(STATUS "**********************************************************************")

  # If we are in a ROS build farm, unit tests are MANDATORY, not like in user local builds:
  if (("${BUILD_TESTING}" STREQUAL "1") AND ("$ENV{HOME}" STREQUAL "/home/buildfarm"))
    message(SEND_ERROR "** Requirements for unit tests not found (see messages above). Stopping generation since unit tests are required in build farms **")
  endif()

  return()
endif()

#message(STATUS "mola_test_datasets : ${mola_test_datasets_DIR}")
#message(STATUS "mola_metric_maps   : ${mola_metric_maps_DIR}")

set(DEFAULT_PIPELINE_YAML ${mola_lidar_odometry_SOURCE_DIR}/pipelines/lidar3d-default.yaml)
set(DEFAULT_STATE_ESTIMATOR_YAML ${mola_lidar_odometry_SOURCE_DIR}/state-estimator-params/state-estimation-simple.yaml)

# Test: from rawlog file, using KITTI fragment
# ---------------------------------------------------
set(KITTI00_DATASET_FILE "${mola_test_datasets_DIR}/../datasets/kitti/kitti_00_extract.rawlog")
set(KITTI00_GT_TUM ${mola_lidar_odometry_SOURCE_DIR}/test/kitti_00_fragment_gt.tum)

ament_add_gtest(test_lidar_odometry_kitti test_lidar_odometry_rawlog.cpp
  ENV LO_PIPELINE_YAML=${DEFAULT_PIPELINE_YAML}
  ENV LO_STATE_ESTIM_YAML=${DEFAULT_STATE_ESTIMATOR_YAML}
  ENV LO_TEST_RAWLOG=${KITTI00_DATASET_FILE}
  ENV LO_TEST_GT_TUM=${KITTI00_GT_TUM}
  ENV MOLA_PROFILER=false
)
ament_target_dependencies(test_lidar_odometry_kitti
  mrpt-obs
  mola_state_estimation_simple
)
target_link_libraries(test_lidar_odometry_kitti
    mola::mola_lidar_odometry
    mola::mola_state_estimation_simple
)


# Test: from rawlog file, using MULRAN fragment (incl IMU,GPS)
# ---------------------------------------------------------------
set(MULRAN_KAIST01_DATASET_FILE "${mola_test_datasets_DIR}/../datasets/mulran/mulran_KAIST01_extract.rawlog")
set(MULRAN_KAIST01_GT_TUM ${mola_lidar_odometry_SOURCE_DIR}/test/mulran_KAIST01_fragment_gt.tum)

if (NOT EXISTS ${MULRAN_KAIST01_DATASET_FILE})
  message(STATUS "**Skipping test**: Mulran dataset not available (temporary situation until mola_test_dataset package is updated)")
else()
ament_add_gtest(test_lidar_odometry_mulran test_lidar_odometry_rawlog.cpp
  ENV LO_PIPELINE_YAML=${DEFAULT_PIPELINE_YAML}
  ENV LO_STATE_ESTIM_YAML=${DEFAULT_STATE_ESTIMATOR_YAML}
  ENV LO_TEST_RAWLOG=${MULRAN_KAIST01_DATASET_FILE}
  ENV LO_TEST_GT_TUM=${MULRAN_KAIST01_GT_TUM}
  ENV MOLA_PROFILER=false
)
ament_target_dependencies(test_lidar_odometry_mulran
  mrpt-obs
  mola_state_estimation_simple
)
target_link_libraries(test_lidar_odometry_mulran
    mola::mola_lidar_odometry
    mola::mola_state_estimation_simple
)
endif()

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
    ENV LO_STATE_ESTIM_YAML=${DEFAULT_STATE_ESTIMATOR_YAML}
    ENV LO_TEST_ROSBAG2=${RSLIDAR_DATASET_FILE}
    ENV LO_TEST_LIDAR_TOPIC=${RSLIDAR_TOPIC}
    ENV LO_TEST_GT_TUM=${RSLIDAR_GT_TUM}
    ENV MOLA_PROFILER=false
    MOLA_USE_FIXED_LIDAR_POSE=true  # so we don't need /tf topic in test dataset
    MOLA_IGNORE_NO_POINT_STAMPS=${IGNORE_POINT_STAMPS} # make it fail if no timestamps are found
  )
  ament_target_dependencies(test_lidar_odometry_rslidar 
    mrpt-obs
    mola_state_estimation_simple
  )
  target_link_libraries(test_lidar_odometry_rslidar
    mola::mola_lidar_odometry
    mola::mola_input_rosbag2
    mola::mola_state_estimation_simple
  )
else()
  message(STATUS "Skipping test for missing dataset: ${RSLIDAR_DATASET_FILE}")
endif()
