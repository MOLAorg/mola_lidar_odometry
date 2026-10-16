# Unit tests:

ament_add_gtest(test_keyframe_decider test_keyframe_decider.cpp)
target_link_libraries(test_keyframe_decider mola::mola_lidar_odometry)

ament_add_gtest(test_map_frame_relevel test_map_frame_relevel.cpp)
target_link_libraries(test_map_frame_relevel mola::mola_lidar_odometry mrpt::mrpt_obs)

ament_add_gtest(test_imu_scan_sync test_imu_scan_sync.cpp)
target_link_libraries(test_imu_scan_sync mola::mola_lidar_odometry mrpt::mrpt_obs)

# Every shipped pipeline, with its blocks, must resolve to a complete config:
ament_add_gtest(test_pipelines_load test_pipelines_load.cpp
  ENV LO_PIPELINES_DIR=${mola_lidar_odometry_SOURCE_DIR}/pipelines
)
target_link_libraries(test_pipelines_load mola::mola_lidar_odometry mola::mola_yaml)

# -----------------------------------------------------------------------
# Data-dependent integration tests (require optional packages):
# -----------------------------------------------------------------------
find_package(mola_test_datasets)
find_package(mola_metric_maps)
find_package(mola_input_rosbag2)
find_package(mola_state_estimation_simple)

# Test: relocalization requests. Reads no dataset, so it is gated only on what
# it does need: the metric maps the LO pipeline instantiates, and a state
# estimator for LO to attach to.
# ------------------------------------------------------------------------------
if(mola_metric_maps_FOUND AND mola_state_estimation_simple_FOUND)
  ament_add_gtest(test_relocalization_request test_relocalization_request.cpp
    ENV LO_PIPELINE_YAML=${mola_lidar_odometry_SOURCE_DIR}/pipelines/lidar3d-default.yaml
    ENV LO_STATE_ESTIM_YAML=${mola_lidar_odometry_SOURCE_DIR}/state-estimator-params/state-estimation-simple.yaml
    ENV MOLA_PROFILER=false
  )
  target_link_libraries(test_relocalization_request
    mola::mola_state_estimation_simple
    mola::mola_lidar_odometry
  )
endif()

# Test: error and concurrency paths (empty scans, reset, optimize_twist, GNSS,
# map saving) on synthetic data generated on the fly: no dataset needed.
# ------------------------------------------------------------------------------
if(mola_metric_maps_FOUND AND mola_state_estimation_simple_FOUND)
  ament_add_gtest(test_robustness test_robustness.cpp
    TIMEOUT 600
    ENV LO_PIPELINE_YAML=${mola_lidar_odometry_SOURCE_DIR}/pipelines/lidar3d-default.yaml
    ENV LO_STATE_ESTIM_YAML=${mola_lidar_odometry_SOURCE_DIR}/state-estimator-params/state-estimation-simple.yaml
    ENV MOLA_PROFILER=false
  )
  target_link_libraries(test_robustness
    mrpt::mrpt_obs
    mola::mola_state_estimation_simple
    mola::mola_lidar_odometry
  )
endif()

# Test: initial localization from a poor GNSS (with or without IMU), with the
# real smoother and a global search. Synthetic world and sensor data are
# generated on the fly: no dataset needed.
# ------------------------------------------------------------------------------
find_package(mola_state_estimation_smoother QUIET)
if(mola_metric_maps_FOUND AND mola_state_estimation_simple_FOUND AND
   mola_state_estimation_smoother_FOUND)
  # The smoother's exported target names GTSAM by bare name:
  find_package(GTSAM QUIET)
  find_package(GTSAM_UNSTABLE QUIET)
  # Its config may be under lib/<arch>/, so locate its share/ via the ament index:
  ament_index_has_resource(SMOOTHER_PREFIX packages mola_state_estimation_smoother)
  find_file(SMOOTHER_PARAMS_YAML state-estimation-smoother.yaml
    PATHS ${SMOOTHER_PREFIX}/share/mola_state_estimation_smoother/params
          ${mola_state_estimation_smoother_DIR}/../params
    NO_DEFAULT_PATH)
  ament_add_gtest(test_initial_global_search test_initial_global_search.cpp
    TIMEOUT 900
    ENV LO_PIPELINE_YAML=${mola_lidar_odometry_SOURCE_DIR}/pipelines/lidar3d-default.yaml
    ENV LO_STATE_ESTIM_SIMPLE_YAML=${mola_lidar_odometry_SOURCE_DIR}/state-estimator-params/state-estimation-simple.yaml
    ENV LO_STATE_ESTIM_SMOOTHER_YAML=${SMOOTHER_PARAMS_YAML}
    ENV MOLA_PROFILER=false
  )
  # The smoother is only created by class name: keep it linked.
  target_link_libraries(test_initial_global_search
    mrpt::mrpt_obs
    mola::mola_state_estimation_simple
    mola::mola_lidar_odometry
    mola::mola_relocalization
    "-Wl,--no-as-needed"
    mola::mola_state_estimation_smoother
    "-Wl,--as-needed"
  )
endif()

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
target_link_libraries(test_lidar_odometry_kitti
  mrpt::mrpt_obs
  mola::mola_state_estimation_simple
  mola::mola_lidar_odometry
  mola::mola_state_estimation_simple
)


# LIO Test: from rawlog file, using MULRAN fragment (incl IMU,GPS)
# ------------------------------------------------------------------
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
  ENV MOLA_DESKEW_METHOD="MotionCompensationMethod::IMU"
  ENV MOLA_PROFILER=false
)
target_link_libraries(test_lidar_odometry_mulran
  mrpt::mrpt_obs
  mola::mola_state_estimation_simple
  mola::mola_lidar_odometry
  mola::mola_state_estimation_simple
)

endif()

# Test: from rosbag2 file, RSLIDAR data with XYZIRT channels
# -----------------------------------------------------------
set(RSLIDAR_DATASET_FILE "${mola_test_datasets_DIR}/../datasets/rslidar_warehouse/rslidar_fragment.mcap")
set(RSLIDAR_GT_TUM ${mola_lidar_odometry_SOURCE_DIR}/test/rslidar_fragment_gt.tum)
set(RSLIDAR_TOPIC "/rslidar_points")

if(${mrpt_libros_bridge_VERSION} VERSION_GREATER_EQUAL "2.13.5")
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
  target_link_libraries(test_lidar_odometry_rslidar
    mrpt::mrpt_obs
    mola::mola_lidar_odometry
    mola::mola_input_rosbag2
    mola::mola_state_estimation_simple
  )
else()
  message(STATUS "Skipping test for missing dataset: ${RSLIDAR_DATASET_FILE}")
endif()

# -----------------------------------------------------------------------------
# Integration tests: complete, headless mola-cli runs of the shipped launch
# files on the datasets above, as a user would launch them. Unlike the tests
# above, these also cover module loading, YAML launch files and their variable
# expansion, the dataset source modules, threading and shutdown.
# -----------------------------------------------------------------------------
find_package(Python3 REQUIRED COMPONENTS Interpreter)
ament_index_has_resource(MOLA_LAUNCHER_PREFIX packages mola_launcher)
find_program(MOLA_CLI_EXECUTABLE mola-cli
  PATHS ${MOLA_LAUNCHER_PREFIX}/bin
  NO_DEFAULT_PATH)

# mola_lo_add_cli_test(<name> <launch yaml> <reference tum> [ENV VAR=value ...])
function(mola_lo_add_cli_test NAME LAUNCH REFERENCE)
  cmake_parse_arguments(ARG "" "" "ENV" ${ARGN})
  ament_add_test(${NAME}
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/run_mola_cli_dataset.py
      --mola-cli ${MOLA_CLI_EXECUTABLE}
      --launch ${mola_lidar_odometry_SOURCE_DIR}/mola-cli-launchs/${LAUNCH}
      --reference ${REFERENCE}
    TIMEOUT 300
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    # The module under test, found before any other installed copy of it:
    ENV MOLA_MODULES_LIB_PATH=$<TARGET_FILE_DIR:mola_lidar_odometry>
      ${ARG_ENV}
  )
endfunction()

if(NOT MOLA_CLI_EXECUTABLE)
  message(STATUS "Skipping mola-cli integration tests: mola-cli not found")
else()
  mola_lo_add_cli_test(test_mola_cli_kitti lidar_odometry_from_rawlog.yaml ${KITTI00_GT_TUM}
    ENV MOLA_INPUT_RAWLOG=${KITTI00_DATASET_FILE}
  )
  if(EXISTS ${MULRAN_KAIST01_DATASET_FILE})
    mola_lo_add_cli_test(test_mola_cli_mulran lidar_odometry_from_rawlog.yaml
      ${MULRAN_KAIST01_GT_TUM}
      ENV MOLA_INPUT_RAWLOG=${MULRAN_KAIST01_DATASET_FILE}
          MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU
    )
  endif()
  if(EXISTS ${RSLIDAR_DATASET_FILE})
    mola_lo_add_cli_test(test_mola_cli_rslidar_mcap lidar_odometry_from_rosbag2.yaml
      ${RSLIDAR_GT_TUM}
      ENV MOLA_INPUT_ROSBAG2=${RSLIDAR_DATASET_FILE}
          MOLA_LIDAR_TOPIC=${RSLIDAR_TOPIC}
          MOLA_USE_FIXED_LIDAR_POSE=true
          MOLA_IGNORE_NO_POINT_STAMPS=${IGNORE_POINT_STAMPS}
    )
  endif()
endif()

# Test: real ROS 2 launch with a rosbag2, local map published (or not) as a point cloud
# -----------------------------------------------------------------------------------
# With MRPT 3, the default local map class (a keyframe map, not a points map)
# only reaches ROS through CMetricMap::asPointsMapView(), overridden in
# mola_metric_maps. Check both in the headers we build against:
set(LOCALMAP_REACHES_ROS FALSE)
get_target_property(_mrpt_obs_incs mrpt::mrpt_obs INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(_mola_maps_incs mola::mola_metric_maps INTERFACE_INCLUDE_DIRECTORIES)
find_file(_CMETRICMAP_H mrpt/maps/CMetricMap.h PATHS ${_mrpt_obs_incs} NO_DEFAULT_PATH)
find_file(_KFM_H mola_metric_maps/KeyframePointCloudMap.h PATHS ${_mola_maps_incs} NO_DEFAULT_PATH)
if(_CMETRICMAP_H AND _KFM_H)
  file(STRINGS ${_CMETRICMAP_H} _mrpt_has_view REGEX "define MRPT_HAS_METRIC_MAP_POINTS_VIEW")
  file(STRINGS ${_KFM_H} _kfm_has_view REGEX "asPointsMapView")
  if(_mrpt_has_view AND _kfm_has_view)
    set(LOCALMAP_REACHES_ROS TRUE)
  endif()
endif()

find_package(launch_testing_ament_cmake QUIET)
if(NOT LOCALMAP_REACHES_ROS)
  message(STATUS "Skipping test_ros2_local_map_publishing: MRPT or mola_metric_maps lack CMetricMap::asPointsMapView()")
elseif(launch_testing_ament_cmake_FOUND AND EXISTS ${RSLIDAR_DATASET_FILE})
  add_launch_test(test_ros2_local_map_publishing.py
    TARGET test_ros2_local_map_publishing
    TIMEOUT 400
    ARGS "rosbag:=${RSLIDAR_DATASET_FILE}"
  )
endif()
