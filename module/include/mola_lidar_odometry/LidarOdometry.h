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

/**
 * @file   LidarOdometry.h
 * @brief  Header for main C++ class exposing LIDAR odometry
 * @author Jose Luis Blanco Claraco
 * @date   Sep 16, 2023
 */
#pragma once

// MOLA interfaces:
#include <mola_kernel/interfaces/FrontEndBase.h>
#include <mola_kernel/interfaces/LocalizationSourceBase.h>
#include <mola_kernel/interfaces/MapServer.h>
#include <mola_kernel/interfaces/MapSourceBase.h>
#include <mola_kernel/interfaces/NavStateFilter.h>
#include <mola_kernel/interfaces/Relocalization.h>
#include <mola_kernel/version.h>

// Other packages:
#include <mola_pose_list/SearchablePoseList.h>

// MP2P_ICP
#include <mp2p_icp/ICP.h>
#include <mp2p_icp/Parameterizable.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/Generator.h>

// MRPT
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/serialization/CSerializable.h>

// STD:
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <regex>
#include <string>
#include <vector>

// Fwrd decls:
namespace nanogui
{
class Label;
class CheckBox;
}  // namespace nanogui

namespace mola
{
/** LIDAR-inertial odometry based on ICP against a local metric map model.
 */
class LidarOdometry : public mola::FrontEndBase,
                      public mola::LocalizationSourceBase,
                      public mola::MapSourceBase,
                      public mola::MapServer,
                      public mola::Relocalization
{
  DEFINE_MRPT_OBJECT(LidarOdometry, mola)

public:
  LidarOdometry();
  ~LidarOdometry();

  /** @name Main API
     * @{ */

  // See docs in base class
  void initialize_frontend(const Yaml & cfg) override;
  void spinOnce() override;
  void onNewObservation(const CObservation::Ptr & o) override;

  /** Re-initializes the odometry system. It effectively calls initialize()
     *  once again with the same parameters that were used the first time.
     */
  void reset();

  enum class AlignKind : uint8_t
  {
    RegularOdometry = 0,
    NoMotionModel
  };

  enum class InitLocalization : uint8_t
  {
    FixedPose = 0,
    FromGNSS_Static,
    FromGNSS_Motion,
  };

  struct Parameters : public mp2p_icp::Parameterizable
  {
    /** List of sensor labels or regex's to be matched to input observations
         *  to be used as raw lidar observations.
         */
    std::vector<std::regex> lidar_sensor_labels;

    /** Sensor labels or regex to be matched to input observations
         *  to be used as raw IMU observations.
         */
    std::optional<std::regex> imu_sensor_label;

    /** Sensor labels or regex to be matched to input observations
         *  to be used as wheel odometry observations.
         */
    std::optional<std::regex> wheel_odometry_sensor_label;

    /** Sensor labels or regex to be matched to input observations
         *  to be used as GNSS (GPS) observations.
         */
    std::optional<std::regex> gnss_sensor_label;

    /** Minimum time (seconds) between scans for being attempted to be
         * aligned. Scans faster than this rate will be just silently ignored.
         */
    double min_time_between_scans = 0.05;

    double max_sensor_range_filter_coefficient = 0.999;
    double absolute_minimum_sensor_range = 5.0;  // [m]

    /** If enabled (slower), vehicle twist will be optimized during ICP
         *  enabling better and more robust odometry in high dynamics motion.
         */
    bool optimize_twist = false;
    double optimize_twist_rerun_min_trans = 0.1;    // [m]
    double optimize_twist_rerun_min_rot_deg = 0.5;  // [deg]
    size_t optimize_twist_max_corrections = 8;

    struct MultipleLidarOptions
    {
      /** If N>1, the system LO system will try to group "N" observations
             * before attempting to use them for localization and map update.
             */
      uint32_t lidar_count = 1;

      /** If using multiple LIDARs, the maximum delay between the first
             * and last one in order to be treated as a "group". In seconds. */
      double max_time_offset = 25e-3;

      void initialize(const Yaml & c, Parameters & parent);
    };

    MultipleLidarOptions multiple_lidars;

    struct MapUpdateOptions
    {
      /** If set to false, the odometry system can be used as
             * localization-only.
             */
      bool enabled = true;

      /** Minimum Euclidean distance (x,y,z) between keyframes inserted
             * into the local map [meters]. */
      double min_translation_between_keyframes = 1.0;

      /** Minimum rotation (in 3D space, yaw, pitch,roll, altogether)
             * between keyframes inserted into
             * the local map [in degrees]. */
      double min_rotation_between_keyframes = 30.0;

      /** If true, distance from the last map update are only considered.
             * Use if mostly mapping without "closed loops".
             *
             *  If false (default), a KD-tree will be used to check the distance
             * to *all* past map insert poses.
             */
      bool measure_from_last_kf_only = false;

      /** Should match the "remove farther than" option of the local
             * metric map. 0 means deletion of distant keyframes is disabled.
             * In meters.
             */
      double max_distance_to_keep_keyframes = 0;

      /** If  `max_distance_to_keep_keyframes` is not zero, this indicates
             * how often to do the distant keyframes clean up.
             */
      uint32_t check_for_removal_every_n = 100;

      /** Publish updated map via mola::MapSourceBase once every N frames
             */
      uint32_t publish_map_updates_every_n = 5;

      /** If non-empty, the local map will be loaded from the given `*.mm`
             * file instead of generating it from scratch.
             * This can be used for multi-session SLAM, or for
             * localization-only.
             */
      std::string load_existing_local_map;

      void initialize(const Yaml & c, Parameters & parent);
    };

    MapUpdateOptions local_map_updates;

    /** Minimum ICP "goodness" (in the range [0,1]) for a new KeyFrame to be
         * accepted during regular lidar odometry & mapping */
    double min_icp_goodness = 0.4;

    bool pipeline_profiler_enabled = false;
    bool icp_profiler_enabled = false;
    bool icp_profiler_full_history = false;

    struct Visualization
    {
      int map_update_decimation = 10;
      bool show_trajectory = true;
      bool show_ground_grid = true;
      float ground_grid_spacing = 5.0f;
      bool show_current_observation = true;
      double current_pose_corner_size = 1.5;  //! [m]
      float local_map_point_size = 3.0f;
      bool local_map_render_voxelmap_free_space = false;
      bool gui_subwindow_starts_hidden = false;
      bool show_console_messages = true;
      bool camera_follows_vehicle = true;
      bool camera_rotates_with_vehicle = false;

      /** If not empty, an optional 3D model (.DAE, etc) to load for
             * visualizing the robot/vehicle pose */
      struct ModelPart
      {
        std::string file;
        mrpt::math::TPose3D tf;  /// Optional 3D model offset/rotation
        double scale = 1.0;
      };
      std::vector<ModelPart> model;

      void initialize(const Yaml & c);
    };
    Visualization visualization;

    // KISS-ICP adaptive threshold method:
    struct AdaptiveThreshold
    {
      bool enabled = true;
      double initial_sigma = 0.5;
      double maximum_sigma = 3.0;
      double min_motion = 0.10;
      double kp = 5.0;
      double alpha = 0.99;

      void initialize(const Yaml & c);
    };
    AdaptiveThreshold adaptive_threshold;

    /** ICP parameters for the case of having, or not, a good velocity
         * model that works a good prior. Each entry in the vector is an
         * "ICP stage", to be run as a sequence of coarser to finer detail
         */
    struct ICP_case
    {
      mp2p_icp::ICP::Ptr icp;
      mp2p_icp::Parameters icp_parameters;
    };

    std::map<AlignKind, ICP_case> icp;

    // === SIMPLEMAP GENERATION ====
    struct SimpleMapOptions
    {
      bool generate = false;

      /** Minimum Euclidean distance (x,y,z) between keyframes inserted
             * into the simplemap [meters]. */
      double min_translation_between_keyframes = 1.0;

      /** Minimum rotation (in 3D space, yaw, pitch,roll, altogether)
             * between keyframes inserted into
             * the map [in degrees]. */
      double min_rotation_between_keyframes = 30.0;

      /** If true, distance from the last map update are only considered.
             * Use if mostly mapping without "closed loops".
             *
             *  If false (default), a KD-tree will be used to check the distance
             * to *all* past map insert poses.
             */
      bool measure_from_last_kf_only = false;

      /** If not empty, the final simple map will be dumped to a file at
             * destruction time */
      std::string save_final_map_to_file;

      /** If enabled, all frames are stored in the simplemap, but
             * non-keyframes will be without associated observations. */
      bool add_non_keyframes_too = false;

      /** If !=0, storing the latest GNSS observation together with the
             * Lidar observation in the simplemap CSensoryFrame (SF)
             * ("keyframe") will be enabled. This parameter sets the maximum age
             * in seconds for a GNSS (GPS) observation to be considered valid to
             * be stored in the SF.
             */
      double save_gnss_max_age = 1.0;  // [s]

      /** If enabled, a directory will be create alongside the .simplemap
             *  and pointclouds will be externally serialized there, for much
             * faster loading and processing of simplemaps.
             */
      bool generate_lazy_load_scan_files = false;

      /** If non-empty, the simple map will be loaded from the given
             * `*.simplemap` file instead of generating it from scratch. This
             * can be used for multi-session SLAM, or for localization-only.
             */
      std::string load_existing_simple_map;

      void initialize(const Yaml & c, Parameters & parent);
    };

    SimpleMapOptions simplemap;

    // === OUTPUT TRAJECTORY ====
    struct TrajectoryOutputOptions
    {
      bool save_to_file = false;

      /** If save_to_file==true, the final estimated trajectory will be
             * dumped to a file at destruction time */
      std::string output_file = "output.txt";

      void initialize(const Yaml & c);
    };

    TrajectoryOutputOptions estimated_trajectory;

    // === TRACE LOG GENERATION ====
    struct TraceOutputOptions
    {
      bool save_to_file = false;

      /** If save_to_file==true, all internal variables will be saved to a
             * csv file */
      std::string output_file = "mola-lo-traces.csv";

      void initialize(const Yaml & c);
    };

    TraceOutputOptions debug_traces;

    struct InitialLocalizationOptions
    {
      InitialLocalizationOptions() = default;

      bool enabled = false;
      InitLocalization method = InitLocalization::FixedPose;

      mrpt::math::TPose3D fixed_initial_pose;
      std::optional<mrpt::math::CMatrixDouble66> initial_pose_cov;

      // Right after a re-localization, do not update the pose state estimator for a few iterations
      uint32_t additional_uncertainty_after_reloc_how_many_timesteps = 5;

      void initialize(const Yaml & c);
    };

    InitialLocalizationOptions initial_localization;

    struct ObservationValidityChecks
    {
      ObservationValidityChecks() = default;

      bool enabled = false;
      std::string check_layer_name = "raw";
      uint32_t minimum_point_count = 1000;

      void initialize(const Yaml & c);
    };

    ObservationValidityChecks observation_validity_checks;

    bool start_active = true;

    uint32_t max_worker_thread_queue_before_drop = 500;

    uint32_t gnss_queue_max_size = 100;

    /** When publishing pose updates, the reference frame for both, estimated robot poses, and the local map.*/
    std::string publish_reference_frame = "odom";

    /** When publishing pose updates, the vehicle frame name.*/
    std::string publish_vehicle_frame = "base_link";
  };

  /** Algorithm parameters */
  Parameters params_;

  bool isBusy() const;

  /** Returns a copy of the estimated trajectory, with timestamps for each
     * lidar observation.
     * Multi-thread safe to call.
     */
  mrpt::poses::CPose3DInterpolator estimatedTrajectory() const;

  /** Returns the last estimated kinematic state: pose of the vehicle in the LiDAR odometry
   *  frame, and its estimated local (body-frame) twist vector.
   *  This method will block if LO is running in another thread, until it is safe to get the data.
   *  It will return std::nullopt if pose information is not available yet, e.g. still initializing.
   */
  std::optional<std::tuple<mrpt::poses::CPose3DPDFGaussian, mrpt::math::TTwist3D>>
  lastEstimatedState() const;

  /** Returns a copy of the estimated simplemap.
     * Multi-thread safe to call.
     */
  mrpt::maps::CSimpleMap reconstructedMap() const;

  void saveEstimatedTrajectoryToFile() const;
  void saveReconstructedMapToFile() const;

  /** Enqueue a custom user request to be executed on the main LidarOdometry
     *  thread on the next iteration.
     *
     *  So, this method is safe to be called from any other thread.
     *
     */
  void enqueue_request(const std::function<void()> & userRequest);

  /** @} */

  /** @name Virtual interface of Relocalization
     *{ */

  /** Re-localize near this pose, including uncetainty.
     *  \param[in] pose The pose, in the local map frame.
     *  There is no return value from this method.
     */
  void relocalize_near_pose_pdf(const mrpt::poses::CPose3DPDFGaussian & p) override;

  /** Re-localize with the next incoming GNSS message.
     *  There is no return value from this method.
     */
  void relocalize_from_gnss() override;

  /** @} */

  /** @name Virtual interface of MapServer
     *{ */

  /** Loads a map from file(s) and sets it as active current map.
     * Different implementations may use one or more files to store map as
     * files.
     *
     *  \param[in] path File name(s) prefix for the map to load. Do not add file
     * extension.
     */
  MapServer::ReturnStatus map_load(const std::string & path) override;

  /** Saves a map from file(s) and sets it as active current map.
     * Different implementations may use one or more files to store map as
     * files.
     *
     *  \param[in] path File name(s) prefix for the map to save. Do not add file
     * extension.
     */
  MapServer::ReturnStatus map_save(const std::string & path) override;

  /** @} */

protected:
#if MOLA_VERSION_CHECK(1, 4, 0)  // Parameters added in MOLA v1.4.0
  // See docs in base class.
  void onParameterUpdate(const mrpt::containers::yaml & names_values) override;
#endif
  void onExposeParameters();  // called after initialization

  void publishMetricMapGeoreferencingData();

private:
  struct ICP_Input
  {
    using Ptr = std::shared_ptr<ICP_Input>;

    mrpt::poses::CPose3D last_keyframe_pose;
    std::optional<mrpt::poses::CPose3DPDFGaussianInf> prior;
    id_t global_id = mola::INVALID_ID;
    id_t local_id = mola::INVALID_ID;
    double time_since_last_keyframe = 0;
    mp2p_icp::metric_map_t::Ptr global_pc, local_pc;
    mrpt::math::TPose3D init_guess_local_wrt_global;
    mp2p_icp::Parameters icp_params;
    AlignKind align_kind = AlignKind::RegularOdometry;
  };
  struct ICP_Output
  {
    ICP_Output() = default;

    mrpt::poses::CPose3DPDFGaussian found_pose_to_wrt_from;
    double goodness = .0;
    uint32_t icp_iterations = 0;
  };

  /** All variables that hold the algorithm state */
  struct MethodState
  {
    MethodState() = default;

    bool initialized = false;
    bool fatal_error = false;

    // will be true after the first incoming LiDAR frame and re-localization is enabled and run
    bool initial_localization_done = false;

    /// if false, input observations will be just ignored.
    /// Useful for real-time execution on robots.
    bool active = true;

    int worker_tasks = 0;

    mrpt::poses::CPose3DPDFGaussian last_lidar_pose;  //!< in local map

    std::map<std::string, mrpt::Clock::time_point> last_obs_tim_by_label;
    bool last_icp_was_good = true;
    double last_icp_quality = .0;

    std::optional<mrpt::Clock::time_point> first_ever_timestamp;
    std::optional<mrpt::Clock::time_point> last_obs_timestamp;
    std::optional<mrpt::Clock::time_point> last_icp_timestamp;

    /// Cache for multiple LIDAR synchronization:
    std::map<std::string /*label*/, mrpt::obs::CObservation::Ptr> sync_obs;

    // navstate_fuse to merge pose estimates, IMU, odom, estimate twist.
    std::shared_ptr<mola::NavStateFilter> navstate_fuse;

    std::optional<NavState> last_motion_model_output;

    /// The source of "dynamic variables" in ICP pipelines:
    mp2p_icp::ParameterSource parameter_source;

    // KISS-ICP-like adaptive threshold method:
    double adapt_thres_sigma = 0;  // 0: initial

    // Automatic estimation of max range:
    std::optional<double> estimated_sensor_max_range;
    std::optional<double> instantaneous_sensor_max_range;

    mp2p_icp_filters::GeneratorSet obs_generators;
    mp2p_icp_filters::FilterPipeline pc_filterAdjustTimes;
    mp2p_icp_filters::FilterPipeline pc_filter1, pc_filter2, pc_filter3;
    mp2p_icp_filters::GeneratorSet local_map_generators;
    mp2p_icp::metric_map_t::Ptr local_map = mp2p_icp::metric_map_t::Create();
    mp2p_icp_filters::FilterPipeline obs2map_merge;

    mrpt::poses::CPose3DInterpolator estimated_trajectory;
    mrpt::maps::CSimpleMap reconstructed_simplemap;

    // to check for map updates. Defined as optional<> so we enforce
    // setting their type in the ctor:
    std::optional<SearchablePoseList> distance_checker_local_map;
    std::optional<SearchablePoseList> distance_checker_simplemap;

    /// See check_for_removal_every_n
    uint32_t localmap_check_removal_counter = 0;
    uint32_t localmap_advertise_updates_counter = std::numeric_limits<uint32_t>::max();

    /// To update the map in the viz only if really needed
    bool local_map_needs_viz_update = true;
    bool local_map_needs_publish = true;

    void mark_local_map_as_updated()
    {
      local_map_needs_viz_update = true;
      local_map_needs_publish = true;
    }

    /// To handle post-re-localization. >0 means we are "recovering" from a request to re-localize:
    uint32_t step_counter_post_relocalization = 0;

    // GNSS: keep a list of recent observations to later on search the one
    // closest to each LIDAR observation:
    std::map<mrpt::Clock::time_point, std::shared_ptr<mrpt::obs::CObservationGPS>> last_gnss_;

    // Visualization:
    mrpt::opengl::CSetOfObjects::Ptr glVehicleFrame, glPathGrp;
    mrpt::opengl::CSetOfLines::Ptr glEstimatedPath;
    int mapUpdateCnt = std::numeric_limits<int>::max();

    // List of old observations to be unload()'ed, to save RAM if:
    // 1) building a simplemap, and
    // 2) Using a dataset source that supports lazy-load:
    mutable std::map<mrpt::Clock::time_point, mrpt::obs::CSensoryFrame::Ptr>
      past_simplemaps_observations;
  };

  /** The worker thread pool with 1 thread for processing incomming
     * IMU or LIDAR observations*/
  mrpt::WorkerThreadsPool worker_{
    1 /*num threads*/, mrpt::WorkerThreadsPool::POLICY_FIFO, "worker_lidar_odom"};

  MethodState state_;
  const MethodState & state() const { return state_; }
  MethodState stateCopy() const { return state_; }

  struct StateUI
  {
    StateUI() = default;

    double timestampLastUpdateUI = 0;

    nanogui::Window * ui = nullptr;
    nanogui::Label * lbIcpQuality = nullptr;
    nanogui::Label * lbSigma = nullptr;
    nanogui::Label * lbSensorRange = nullptr;
    nanogui::Label * lbTime = nullptr;
    nanogui::Label * lbSpeed = nullptr;
    nanogui::CheckBox * cbActive = nullptr;
    nanogui::CheckBox * cbMapping = nullptr;
    nanogui::CheckBox * cbSaveSimplemap = nullptr;
  };

  StateUI gui_;

  /// The configuration used in the last call to initialize()
  Yaml lastInitConfig_;

  bool destructor_called_ = false;
  mutable std::mutex is_busy_mtx_;
  mutable std::recursive_mutex state_mtx_;
  mutable std::mutex state_trajectory_mtx_;
  mutable std::recursive_mutex state_simplemap_mtx_;

  /// The list of pending tasks from enqueue_request():
  std::vector<std::function<void()>> requests_;
  std::mutex requests_mtx_;

  // Process requests_(), at the spinOnce() rate.
  void processPendingUserRequests();

  void onLidar(const CObservation::Ptr & o);
  void onLidarImpl(const CObservation::Ptr & obs);

  void onIMU(const CObservation::Ptr & o);
  void onIMUImpl(const CObservation::Ptr & o);

  void onWheelOdometry(const CObservation::Ptr & o);
  void onWheelOdometryImpl(const CObservation::Ptr & o);

  void onGPS(const CObservation::Ptr & o);
  void onGPSImpl(const CObservation::Ptr & o);

  // KISS-ICP adaptive threshold method:
  void doUpdateAdaptiveThreshold(const mrpt::poses::CPose3D & lastMotionModelError);

  void doInitializeEstimatedMaxSensorRange(const mrpt::obs::CObservation & o);
  void doUpdateEstimatedMaxSensorRange(const mp2p_icp::metric_map_t & m);

  /// Returns false if the scan/observation is not valid:
  bool doCheckIsValidObservation(const mp2p_icp::metric_map_t & m);

  void updatePipelineDynamicVariables();
  void updatePipelineTwistVariables(const mrpt::math::TTwist3D & tw);

  void updateVisualization(const mp2p_icp::metric_map_t & currentObservation);

  void internalBuildGUI();

  void doPublishUpdatedLocalization(const mrpt::Clock::time_point & this_obs_tim);

  void doPublishUpdatedMap(const mrpt::Clock::time_point & this_obs_tim);

  void doWriteDebugTracesFile(const mrpt::Clock::time_point & this_obs_tim);
  std::optional<std::ofstream> debug_traces_of_;

  void unloadPastSimplemapObservations(const size_t maxSizeUnloadQueue) const;

  void handleUnloadSinglePastObservation(CObservation::Ptr & o) const;
};

}  // namespace mola
