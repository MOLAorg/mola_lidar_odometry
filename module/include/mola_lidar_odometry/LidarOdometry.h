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
 * @file   LidarOdometry.h
 * @brief  Header for main C++ class exposing LIDAR odometry
 * @author Jose Luis Blanco Claraco
 * @date   Sep 16, 2023
 */
#pragma once

// MOLA interfaces:
#include <mola_kernel/id.h>  // INVALID_ID
#if __has_include(<mola_kernel/interfaces/DiagnosticsProvider.h>)
#include <mola_kernel/interfaces/DiagnosticsProvider.h>
#define MOLA_HAS_DIAGNOSTICS_PROVIDER 1
#endif
#include <mola_kernel/interfaces/FrontEndBase.h>
#include <mola_kernel/interfaces/LocalizationSourceBase.h>
#include <mola_kernel/interfaces/MapServer.h>
#include <mola_kernel/interfaces/MapSourceBase.h>
#include <mola_kernel/interfaces/NavStateFilter.h>
#include <mola_kernel/interfaces/Relocalization.h>
#if __has_include(<mola_kernel/interfaces/SharedKeyframeMap.h>)
#include <mola_kernel/interfaces/SharedKeyframeMap.h>
#define MOLA_HAS_SHARED_KEYFRAME_MAP_SINK 1
#endif
#include <mola_kernel/version.h>

// Other packages:
#include <mola_imu_preintegration/ImuInitialCalibrator.h>
#include <mola_lidar_odometry/KeyframeDecider.h>

// MP2P_ICP
#include <mp2p_icp/ICP.h>
#include <mp2p_icp/Parameterizable.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/Generator.h>

// The yaw-free, rank-2 gravity prior is only available in recent mp2p_icp
// versions. Keep building against older ones, falling back at run time to the
// legacy path that folds tilt into the SE(3) pose prior.
#if defined(__has_include)
#if __has_include(<mp2p_icp/GravityPrior.h>)
#include <mp2p_icp/GravityPrior.h>
#define MOLA_LO_HAS_MP2P_GRAVITY_PRIOR 1
#endif
#endif

// MRPT
#include <mrpt/containers/circular_buffer.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/typemeta/TEnumType.h>

// STD:
#include <array>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <regex>
#include <string>
#include <vector>

// Forward declarations:
#include <mola_kernel/GuiWidgetDescription.h>

namespace mola
{
template <std::size_t N, typename T>
constexpr std::array<T, N> create_array(const T & value);

enum class InitLocalization : uint8_t
{
  /// Initialize around a given SE(3) pose with covariance:
  FixedPose = 0,
  /// Initialize from the external state estimator, with an optional maximum uncertainty threshold
  FromStateEstimator,
  /// Initialize pitch & roll from a short IMU sequence, assuming sensor is roughly stationary at startup
  PitchAndRollFromIMU,
};

/** LIDAR-inertial odometry based on ICP against a local metric map model.
 */
class LidarOdometry : public mola::FrontEndBase,
                      public mola::LocalizationSourceBase,
                      public mola::MapSourceBase,
                      public mola::MapServer,
                      public mola::Relocalization
#if MOLA_HAS_DIAGNOSTICS_PROVIDER
,
                      public mola::DiagnosticsProvider
#endif
{
  DEFINE_MRPT_OBJECT(LidarOdometry, mola)

private:
  constexpr static std::size_t IMU_BUFFER_SIZE = 4096;

public:
  LidarOdometry();
  ~LidarOdometry() override;

  // Prevent copying and moving
  LidarOdometry(const LidarOdometry &) = delete;
  LidarOdometry & operator=(const LidarOdometry &) = delete;
  LidarOdometry(LidarOdometry &&) = delete;
  LidarOdometry & operator=(LidarOdometry &&) = delete;

  /** @name Main API
     * @{ */

  // See docs in base class
  void initialize_frontend(const Yaml & cfg) override;
  void spinOnce() override;
  void onNewObservation(const CObservation::ConstPtr & o) override;
  void onQuit() override;

  /** Re-initializes the odometry system. It effectively calls initialize()
     *  once again with the same parameters that were used the first time.
     */
  void reset();

  enum class AlignKind : uint8_t
  {
    RegularOdometry = 0,
    NoMotionModel
  };

  struct Parameters : public mp2p_icp::Parameterizable
  {
    /** Loads the keyframe-creation policy shared by the local-map and
         *  simplemap sections. Lives here, and not in KeyframeDecisionOptions
         *  itself, because the two distance thresholds may be formulas and
         *  must register into THIS object's dynamic-parameter pool.
         *  \param section_name Only used in error messages.
         *  \param distances_required Whether the two distance thresholds must
         *         be present in the YAML. Kept per-section for backwards
         *         compatibility: the local-map section demands them, the
         *         simplemap one defaults them.
         */
    void load_keyframe_policy(
      mola::KeyframeDecisionOptions & o, const Yaml & cfg, const char * section_name,
      bool distances_required);

    /** List of sensor labels or regex's to be matched to input observations
         *  to be used as raw lidar observations.
         */
    std::vector<std::regex> lidar_sensor_labels;

    /** Sensor labels or regex to be matched to input observations
         *  to be used as raw IMU observations.
         */
    std::optional<std::regex> imu_sensor_label;

    /** Sensor labels or regex to be matched to input observations
         *  to be used as GNSS (GPS) observations.
         */
    std::optional<std::regex> gnss_sensor_label;

    /** Minimum time (seconds) between scans for being attempted to be
         * aligned. Scans faster than this rate will be just silently ignored.
         */
    double min_time_between_scans = 0.05;

    // Low-pass filter alpha for the EMA of ESTIMATED_OBSERVATION_RADIUS, and
    // the absolute floor for that estimate. Both used to be named *_sensor_*
    // before the observation-radius rename; the old YAML keys still parse.
    double observation_radius_filter_coefficient = 0.999;
    double absolute_minimum_observation_radius = 5.0;  // [m]

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

    /** Keyframe-creation policy for the LOCAL METRIC MAP. The distance
         * thresholds, the occupancy count and the temporal window all come
         * from mola::KeyframeDecisionOptions, shared with SimpleMapOptions;
         * they stay plain (non-nested) YAML keys of the `local_map_updates`
         * section.
         */
    struct MapUpdateOptions : public mola::KeyframeDecisionOptions
    {
      /** If set to false, the odometry system can be used as
             * localization-only.
             */
      bool enabled = true;

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

      /** If not empty, saves the final local metric map to a ".mm" file */
      std::string save_final_local_map;

      /** If true, map loading from file is deferred until after the GUI is
             * first rendered, so the GUI appears before the (potentially long)
             * file I/O. Can also be enabled via the env var
             * MOLA_LO_LOAD_MAP_AFTER_GUI=1.
             */
      bool load_map_after_gui_init = false;

      void initialize(const Yaml & c, Parameters & parent);
    };

    MapUpdateOptions local_map_updates;

    /** Minimum ICP "goodness" (in the range [0,1]) for a new KeyFrame to be
         * accepted during regular lidar odometry & mapping */
    double min_icp_goodness = 0.4;

    /** If defined, .icplog files will be saved if ICP quality drops below the given threshold */
    std::optional<double> write_debug_icp_log_if_quality_under;

    bool pipeline_profiler_enabled = false;
    bool icp_profiler_enabled = false;
    bool icp_profiler_full_history = false;

    struct Visualization
    {
      float current_observation_alpha = 0.20f;

      float background_color_gray_level = 0.3f;

      float current_pose_corner_size = 1.5f;  //! [m]
      float sensor_poses_corner_size = 0.5f;  //! [m], 0 to disable

      /** Whether to render the current-pose XYZ corner at all (independently
             * of current_pose_corner_size). Useful to turn off from a
             * first-person camera, where the corner ends up filling the whole
             * view. Can also be changed at runtime via
             * setCurrentPoseCornerVisualization().
             */
      bool show_current_pose_corner = true;

      // --- Ground grid ---
      bool show_ground_grid = true;
      float ground_grid_spacing = 5.0f;

      // --- Trajectory ---
      bool show_trajectory = true;
      std::vector<float> trajectory_rgba = {0.1f, 0.1f, 0.1f, 1.0f};

      // --- Current scan observation ---
      bool show_current_observation = false;  // Show the incoming raw lidar scan observation
      float current_observation_point_size = 3.0f;
      mrpt::img::TColormap current_observation_colormap = mrpt::img::TColormap::cmJET;
      /// Can be any pointcloud field name. Will change to "z" for simple XYZ clouds.
      std::string current_observation_color_by_field = "intensity";

      // --- Deskewed decaying scans ---
      /// Show a sliding window of decaying past observations to visualize a "dense local map"
      bool show_last_deskewed_observations_decay = true;
      double observations_decay_seconds = 5.0;
      float observations_initial_alpha = 0.10f;
      float last_deskewed_observations_point_size = 1.0f;
      mrpt::img::TColormap last_deskewed_observations_colormap = mrpt::img::TColormap::cmJET;
      /// Can be any pointcloud field name. Will change to "z" for simple XYZ clouds.
      std::string last_deskewed_observations_color_by_field = "intensity";

      // --- Local map ---
      /// Show the (decimated) underlying local map used to register observations
      /// to (less dense than `show_last_deskewed_observations_decay`).
      bool show_localmap = false;
      float local_map_point_size = 3.0f;
      bool local_map_render_voxelmap_free_space = false;

      mrpt::img::TColormap local_map_colormap = mrpt::img::TColormap::cmJET;
      /// Can be any pointcloud field name. Will change to "z" for simple XYZ clouds.
      std::string local_map_colormap_color_by_field = "intensity";

      /// If show_localmap==true, how many frames to wait to update the visualization of the
      /// map, which is a costly operation.
      int map_update_decimation = 10;

      bool gui_subwindow_starts_hidden = false;

      // --- Tab visibility ---
      bool show_tab_status = true;
      bool show_tab_control = true;
      bool show_tab_view = true;

      // --- camera control ---
      bool camera_follows_vehicle = true;
      bool camera_rotates_with_vehicle = false;
      bool camera_orthographic = false;
      bool show_gravity_align_vector = false;

      /// If true (and a movable-frame-capable visualizer is present), all 3D
      /// objects are drawn as children of a movable scene frame node named
      /// `publish_reference_frame` instead of the viewport root. A central
      /// backend such as mola_mapper_3d then repositions that frame as it
      /// estimates `T_map_to_{odom}`, so this odometry's dense clouds / local
      /// map stay correctly placed in {map} without being re-rendered. When no
      /// such backend is present the frame stays at the identity pose, so the
      /// behavior is identical to drawing at the root (standalone runs).
      bool render_in_movable_frame = true;

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

    private:
      void initializeModelPart(const Yaml & c);
    };
    Visualization visualization;

    // Adaptive threshold method based purely on ICP quality
    struct AdaptiveThreshold
    {
      bool enabled = true;
      double initial_sigma = 0.5;    // Units: [m]
      double maximum_sigma = 3.0;    // Units: [m]
      double min_motion = 0.10;      // Units: [m]
      double max_sigma_step = 0.05;  // Units: [m]
      double kp = 5.0;
      double alpha = 0.99;
      double icp_quality_controller_setpoint = 0.85;  // Range: [0,1]

      // Sustained-failure recovery (opt-in).
      // Updates sigma on good ICP, so a streak of bad ICPs
      // freezes sigma at its last (typically small) value. With a small
      // matcher window the system cannot recover from a perturbation that
      // exceeds 2*sigma. When enabled, sigma is multiplicatively grown
      // toward maximum_sigma after recover_after_n_bad consecutive failures.
      bool recover_on_sustained_failure = false;
      int recover_after_n_bad = 5;
      double recover_growth_factor = 1.5;

      void initialize(const Yaml & c);
    };
    AdaptiveThreshold adaptive_threshold;

    /** Thresholds for REP-107 diagnostics levels. */
    struct Diagnostics
    {
      double icp_quality_warn = 0.30;
      double icp_quality_error = 0.10;

      /// Input considered stale if no observation received for this many seconds
      double input_stale_sec = 3.0;

      /// Fatal-level staleness (no data for this many seconds)
      double input_error_sec = 5.0;

      /// Warn when dropped-frames ratio exceeds this value
      double dropped_ratio_warn = 0.20;
      double dropped_ratio_error = 0.50;

      /// Warn when average process time exceeds this fraction of sensor period
      double timing_utilization_warn = 0.80;

      void initialize(const Yaml & c);
    };
    Diagnostics diagnostics;

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
    /** Keyframe-creation policy for the SIMPLEMAP, which is also the one
         * pushed to a mola::SharedKeyframeMap sink (the central mapper's
         * keyframe backbone). Same shared policy as MapUpdateOptions, but
         * tuned independently: unlike the local map, this one feeds loop
         * closure, so it is the one that usually wants
         * `nearby_keyframe_time_window` enabled.
         */
    struct SimpleMapOptions : public mola::KeyframeDecisionOptions
    {
      bool generate = false;

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

      /** If enabled, saved keyframes will contain an additional 'deskewed' observation with the motion-compensated cloud. */
      bool save_deskewed_scans = false;

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

      InitLocalization method = InitLocalization::FixedPose;

      mrpt::math::TPose3D fixed_initial_pose;
      std::optional<mrpt::math::CMatrixDouble66> initial_pose_cov;

      // Right after a re-localization, do not update the pose state estimator for a few iterations
      uint32_t additional_uncertainty_after_reloc_how_many_timesteps = 5;

      // Right after a re-localization (or initial localization), also do not
      // update the local map nor the simplemap for this many timesteps.
      // 0 (default) disables this and preserves pre-existing behavior:
      // map updates are independent of the re-localization recovery window.
      // Shares the same recovery-window counter as
      // additional_uncertainty_after_reloc_how_many_timesteps (the window
      // becomes the max of the two when both are set).
      uint32_t additional_map_freeze_after_reloc_how_many_timesteps = 0;

      /// Number of IMU (accelerometer) samples to accumulate while stationary to estimate Pitch & Roll:
      uint32_t imu_initial_calibration_sample_count = 50;

      /// Maximum time span (in seconds) for the "imu_initial_calibration_sample_count" IMU samples:
      double imu_initial_calibration_max_age = 0.75;

      /// If provided by the IMU, prefer gravity-aligned orientation from the sensor instead of accelerometer data.
      bool use_imu_orientation = true;

      /// Maximum position sigma (m) to accept state estimator as converged
      /// Used when method == FromStateEstimator
      double from_state_estimator_max_position_sigma = 0.5;  // [m]

      /// Maximum orientation sigma (deg) to accept state estimator as converged
      /// Used when method == FromStateEstimator
      double from_state_estimator_max_orientation_sigma_deg = 3.0;  // [deg]

      /// Timeout (seconds) waiting for state estimator to converge
      /// If <=0, wait indefinitely
      double from_state_estimator_timeout = 60.0;  // [s]

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

    struct IMUGravityCorrection
    {
      /// Enable accelerometer-based pitch/roll correction of the ICP solution.
      bool enabled = true;

      /// Apply the verticality constraint as mp2p_icp's yaw-free, rank-2
      /// `gravityPrior` (recommended) instead of folding the gravity-derived
      /// pitch/roll into the SE(3) pose prior.
      ///
      /// The legacy path (false) encodes tilt in a 6x6 prior information
      /// matrix, whose diagonal only isolates roll/pitch near yaw=0 and which
      /// injects translation directly. Kept selectable for reproducibility of
      /// older results.
      ///
      /// Requires an mp2p_icp version providing mp2p_icp::GravityPrior; when
      /// built against an older one this silently falls back to the legacy
      /// path (with a warning), it does not fail.
      bool use_rank2_prior = true;

      /// Widen `sigma_deg` in quadrature by the MEASURED angular dispersion of
      /// the buffered accelerometer directions, so the constraint stands down
      /// automatically when the accelerometer is not actually measuring
      /// gravity.
      ///
      /// Needed because the quasi-static acceptance gate alone is far too
      /// permissive: accepting |norm(a) - g| <= 2 m/s^2 admits up to
      /// asin(2/9.81) ~= 11.8 deg of aliased tilt. Without this, on a real
      /// vehicle the gravity constraint asserts tilt information the reading
      /// does not contain, and odometry gets worse rather than better.
      bool adaptive_sigma = true;

      /// Sigma [degrees] for the gravity-derived verticality constraint.
      /// Lower values = more trust in IMU. Typical: 1 to 5 deg.
      double sigma_deg = 2.0;

      /// Number of recent accelerometer samples to average for gravity estimation.
      uint32_t averaging_samples = 20;

      /// Maximum age [seconds] for accelerometer samples used in averaging.
      /// Samples older than this are discarded. 0 = no age limit.
      double max_age_seconds = 2.0;

      void initialize(const Yaml & c);
    };

    IMUGravityCorrection imu_gravity_correction;

    bool start_active = true;

    /** Under overload, incoming scans are processed with a "drop stale, keep
     *  freshest" policy: the single worker thread always advances to the newest
     *  ready scan, dropping older ones, so latency stays near one processing
     *  period regardless of this value. This parameter only bounds the auxiliary
     *  wait list used while a scan waits for its IMU data (IMU de-skew pipelines):
     *  if IMU delivery lags, no more than this many scans are kept waiting before
     *  the oldest are dropped. */
    uint32_t max_lidar_queue_before_drop = 15;

    uint32_t gnss_queue_max_size = 100;

    ///  Minimum inverse covariance in (X,Y,Z) for a valid motion model
    double min_motion_model_xyz_cov_inv = 1.0;

    /** When publishing pose updates, the reference frame for both, estimated robot poses, and the local map.*/
    std::string publish_reference_frame = "odom";

    /** When publishing pose updates, the vehicle frame name.*/
    std::string publish_vehicle_frame = "base_link";

    /* If enabled, deskewed scans will be published (so, they will be available as ROS2 messages), mostly for visualization.
   * This may slow-down the system, so it is disabled by default. */
    bool publish_deskewed_scans = false;
  };

  /** Algorithm parameters */
  Parameters params_;

  bool isBusy() const;

  /** Drains the worker thread pools and saves the simplemap/trajectory/local
   *  map to disk, if so configured. Idempotent: safe to call from onQuit()
   *  and then again from the destructor.
   *
   *  Must run to completion (and thus stop calling back into other modules,
   *  e.g. via VizInterface or BridgeROS2's TF broadcaster) before any other
   *  module of the running MOLA system is destroyed. onQuit() is invoked by
   *  MolaLauncherApp for that exact purpose, before any module destructor
   *  runs.
   */
  void shutdownCleanup();
  std::atomic_bool shutdown_cleanup_done_{false};

  bool isActive() const;
  void setActive(bool active);

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

  /** Returns the ICP quality (range: [0,1]) of the last registered scan. Only valid if lastEstimatedState() returns non-empty. */
  double lastIcpQuality() const;

  /** Returns a copy of the estimated simplemap.
     * Multi-thread safe to call.
     */
  mrpt::maps::CSimpleMap reconstructedMap() const;

  void saveEstimatedTrajectoryToFile() const;
  void saveReconstructedMapToFile() const;
  void saveLocalMapToFile() const;

  /** Enqueue a custom user request to be executed on the main LidarOdometry
     *  thread on the next iteration.
     *
     *  So, this method is safe to be called from any other thread.
     *
     */
  void enqueue_request(const std::function<void()> & userRequest);

  /** @} */

  /** @name Runtime visualization coloring overrides
     *  All setters are thread-safe: the change is applied on the internal
     *  LidarOdometry worker thread via enqueue_request(), so they may be
     *  called from any thread (e.g. a host GUI) at any time.
     *{ */

  /** Sets the colormap and color-by-field used to render the live deskewed
     *  scan streams (the incoming current observation and the decaying
     *  "dense local map" of past observations). Pass cmNONE as \a colormap to
     *  keep the point cloud's own RGB colors (no recoloring). */
  void setDeskewedColoring(mrpt::img::TColormap colormap, const std::string & colorByField);

  /** Sets the colormap and color-by-field used to render the local map cloud
     *  (e.g. cmGRAYSCALE for a grayscale global map). */
  void setLocalMapColoring(mrpt::img::TColormap colormap, const std::string & colorByField);

  /** Sets estimated-trajectory line visibility and, if \a rgba has size 4,
     *  its RGBA color (each component in [0,1]). Lets a host own the
     *  trajectory appearance instead of toggling the scene object externally. */
  void setTrajectoryVisualization(bool show, const std::vector<float> & rgba);

  /** Shows or hides the current-pose XYZ corner opengl object. Useful for a
     *  host using a first-person camera placed at the vehicle pose, where the
     *  corner would otherwise fill the whole view. */
  void setCurrentPoseCornerVisualization(bool show);

  /** @} */

  /** @name Virtual interface of Relocalization
     *{ */

  /** Re-localize near this pose, including uncertainty.
     *  \param[in] pose The pose, in the local map frame.
     *  There is no return value from this method.
     *
     *  Thread-safe: may be called from any thread. The actual work is
     *  dispatched asynchronously to the internal lidar worker thread via
     *  enqueue_request(), so this call does not block on state_mtx_ and
     *  cannot be involved in any deadlock with the main processing loop.
     */
  void relocalize_near_pose_pdf(const mrpt::poses::CPose3DPDFGaussian & p) override;

  /** Re-localize with the next incoming GNSS message.
     *  There is no return value from this method.
     *
     *  Thread-safe and non-blocking; see relocalize_near_pose_pdf() for details.
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
     *
     *  Thread-safe: the load work is dispatched to the internal lidar worker
     *  thread via enqueue_request() and this call blocks on an std::future
     *  until the work completes. The caller holds no internal mutex while
     *  waiting, so it cannot contribute to deadlock with the processing loop.
     */
  MapServer::ReturnStatus map_load(const std::string & path) override;

  /** Saves a map from file(s) and sets it as active current map.
     * Different implementations may use one or more files to store map as
     * files.
     *
     *  \param[in] path File name(s) prefix for the map to save. Do not add file
     * extension.
     *
     *  Thread-safe; see map_load() for details on the async dispatch.
     */
  MapServer::ReturnStatus map_save(const std::string & path) override;

  /** @} */

protected:
  // See docs in base class.
  void onParameterUpdate(const mrpt::containers::yaml & names_values) override;
  void onExposeParameters();  // called after initialization

  void publishMetricMapGeoreferencingData();

private:
  struct ICP_Input
  {
    using Ptr = std::shared_ptr<ICP_Input>;

    mrpt::poses::CPose3D last_keyframe_pose;
    std::optional<mrpt::poses::CPose3DPDFGaussianInf> prior;
#if defined(MOLA_LO_HAS_MP2P_GRAVITY_PRIOR)
    /// Yaw-free, rank-2 gravity observation (alternative to folding tilt into
    /// `prior`; see buildGravityPrior()). Only set when the rank-2 path is on.
    std::optional<mp2p_icp::GravityPrior> gravityPrior;
#endif
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

    // ------ these flags are protected by state_flags_mtx_  ---------
    bool initialized = false;
    bool fatal_error = false;
    bool active = true;  //!< whether to process or ignore incoming sensors
    // ------ ^^^ end of these flags are protected ^^^^      ---------

    // ------ these vars are protected by is_busy_mtx_  ---------
    // worker_tasks_lidar is 1 while onLidar() is actively executing a scan, 0
    // otherwise (queued-but-not-yet-running scans are tracked separately, by
    // worker_lidar_.pendingTasks()).
    int worker_tasks_lidar = 0;
    int worker_tasks_others = 0;
    // ------ ^^^ end of these flags are protected ^^^^      ---------

    // ------ these vars are protected by drop_stats_mtx_  ---------
    static constexpr std::size_t DROP_STATS_WINDOW_LENGTH = 128;
    std::array<bool, DROP_STATS_WINDOW_LENGTH> drop_frames_stats_good =
      create_array<DROP_STATS_WINDOW_LENGTH>(true);
    std::array<bool, DROP_STATS_WINDOW_LENGTH> drop_frames_stats_dropped =
      create_array<DROP_STATS_WINDOW_LENGTH>(false);
    std::size_t drop_frames_stats_next_index = 0;
    // ------ ^^^ end of these flags are protected ^^^^      ---------

    // All other fields are protected by state_mtx_

    // will be true after the first incoming LiDAR frame and re-localization is enabled and run
    bool initial_localization_done = false;

    /// Used for pitch & roll initialization
    std::optional<mola::imu::ImuInitialCalibrator> imu_initializer;

    /// Accumulates recent accelerometer readings and provides
    /// a smoothed pitch/roll estimate from the gravity direction.
    struct GravityEstimator
    {
      struct TimestampedAcc
      {
        double timestamp = 0;
        std::array<double, 3> acc = {0, 0, 0};
      };

      mrpt::containers::circular_buffer<TimestampedAcc> acc_buffer{IMU_BUFFER_SIZE};
      mrpt::poses::CPose3D imu_sensor_pose;  ///< last known IMU extrinsics
      double imu_sensor_pose_timestamp = 0;  ///< timestamp of last sensor pose update

      void add(const mrpt::obs::CObservationIMU & imu, uint32_t max_samples);

      /// Returns estimated (pitch, roll) in radians from averaged
      /// accelerometer data in the vehicle frame, or nullopt if not enough data.
      /// max_age_seconds <= 0 means no age filtering.
      std::optional<std::pair<double, double>> estimatedPitchRoll(
        uint32_t required_samples, double max_age_seconds) const;

      /// Empirical 1-sigma [rad] of the gravity DIRECTION over the samples
      /// currently in the buffer: the RMS angle between each buffered sample's
      /// direction and their mean direction.
      ///
      /// This is the honest uncertainty of the reading, measured rather than
      /// assumed. While quasi-static it is small; under real vehicle dynamics
      /// (braking, cornering, vibration) the accepted samples disagree and it
      /// grows, so a caller that adds it in quadrature to its configured sigma
      /// gets a verticality constraint that self-silences exactly when the
      /// quasi-static assumption behind it stops holding.
      ///
      /// nullopt if fewer than 2 usable samples.
      std::optional<double> directionDispersionSigma(double max_age_seconds) const;
    };

    GravityEstimator gravity_estimator;

    /// Gravity-derived (pitch, roll), in radians, captured at the time the first
    /// keyframe (map origin) was created. The IMU gravity estimator reports
    /// *absolute* tilt with respect to true vertical, while the map/global frame
    /// may itself not be exactly level (e.g. `fixed_initial_pose` has nonzero
    /// pitch/roll, or the vehicle was on a slope at start-up). This calibration
    /// offset is required to correctly re-express later absolute IMU tilt
    /// readings relative to the (possibly non-level) map frame.
    std::optional<std::pair<double, double>> gravity_calib_pitch_roll;

    mrpt::poses::CPose3DPDFGaussian last_lidar_pose;  //!< in local map

    std::map<std::string, mrpt::Clock::time_point> last_obs_tim_by_label;
    std::map<std::string, mrpt::poses::CPose3D> last_lidar_sensor_poses;  //!< sensor pose per label
    bool last_icp_was_good = true;
    double last_icp_quality = .0;
    std::size_t last_icp_iterations = 0;

    std::optional<mrpt::Clock::time_point> first_ever_timestamp;
    std::optional<mrpt::Clock::time_point> last_obs_timestamp;
    std::optional<mrpt::Clock::time_point> last_icp_timestamp;
    /// Wall-clock time when the last observation was received (used for
    /// staleness checks, avoids relying on sensor hardware clocks).
    std::optional<mrpt::Clock::time_point> last_obs_reception_time;
    double last_observed_scan_period_sec = 0.0;  //!< seconds between last two processed scans

    /// Timestamp when we started waiting for state estimator convergence
    std::optional<mrpt::Clock::time_point> waiting_for_state_estimator_since;

    /// Received georeferencing from external state estimator
    std::optional<mola::Georeferencing> external_georef;

    /// Cache for multiple LIDAR synchronization:
    std::map<std::string /*label*/, mrpt::obs::CObservation::ConstPtr> sync_obs;

    // navstate_fuse to merge pose estimates, IMU, odom, estimate twist.
    std::shared_ptr<mola::NavStateFilter> navstate_fuse;

#if defined(MOLA_HAS_SHARED_KEYFRAME_MAP_SINK)
    // Central-map backend (e.g. mola_mapper_3d) accepting keyframe-insertion
    // requests, if any is present in the running MOLA system. Detected the
    // same way as navstate_fuse, but optional: nullptr if none is found.
    std::shared_ptr<mola::SharedKeyframeMap> shared_keyframe_map_sink;
#endif

    std::optional<NavState> last_motion_model_output;

    /// The source of "dynamic variables" in ICP pipelines:
    mp2p_icp::ParameterSource parameter_source;

    // KISS-ICP-like adaptive threshold method:
    double adapt_thres_sigma = 0;  // 0: initial

    // Counter of consecutive bad ICPs; drives the optional sustained-failure
    // recovery in AdaptiveThreshold.
    int consecutive_bad_icps = 0;

    // Automatic estimation of the observation bounding-radius (measured from
    // base_link, not from the sensor — see ESTIMATED_OBSERVATION_RADIUS docs):
    std::optional<double> estimated_observation_radius;
    std::optional<double> instantaneous_observation_radius;

    mp2p_icp_filters::GeneratorSet obs_generators;
    mp2p_icp_filters::FilterPipeline pc_filterAdjustTimes;
    mp2p_icp_filters::FilterPipeline pc_prefilter;
    mp2p_icp_filters::FilterPipeline pc_deskew;
    mp2p_icp_filters::FilterPipeline pc_filter1, pc_filter2, pc_filter3;
    mp2p_icp_filters::GeneratorSet local_map_generators;
    mp2p_icp::metric_map_t::Ptr local_map = mp2p_icp::metric_map_t::Create();
    mp2p_icp_filters::FilterPipeline obs2map_merge;

    /// Set to true whenever a preexisting map is loaded into local_map /
    /// reconstructed_simplemap, either at start-up (doPreloadLocalMap()) or
    /// via a runtime map_load() service call. Used to avoid discarding an
    /// inherited map (multisession/multi-robot mapping) on later events that
    /// would otherwise assume a brand new, empty map (e.g. IMU-based initial
    /// re-localization).
    bool map_has_been_loaded = false;

    // fallback only for when not using IMU and optimize_twist is enabled:
    mp2p_icp_filters::FilterPipeline obsDeskewForViz;

    mutable std::optional<bool> isPipelinesUsingIMU;  //!< See isPipelineUsingIMU()

    mrpt::poses::CPose3DInterpolator estimated_trajectory;
    mrpt::maps::CSimpleMap reconstructed_simplemap;

    // to check for map updates. Defined as optional<> so we enforce
    // setting their type in the ctor:
    std::optional<KeyframeDecider> kf_decider_local_map;
    std::optional<KeyframeDecider> kf_decider_simplemap;

    /// See check_for_removal_every_n
    uint32_t localmap_check_removal_counter = 0;
    uint32_t localmap_advertise_updates_counter = std::numeric_limits<uint32_t>::max();

    /// To update the map in the viz only if really needed
    bool local_map_needs_viz_update = true;
    bool local_map_needs_publish = true;
    bool local_map_georef_needs_publish = true;

    std::optional<double> last_yaw_for_viz_camera;

    void mark_local_map_as_updated(bool force_republish = false)
    {
      local_map_needs_viz_update = true;
      local_map_needs_publish = true;
      if (force_republish) {
        localmap_advertise_updates_counter = std::numeric_limits<uint32_t>::max();
      }
    }

    void mark_local_map_georef_as_updated() { local_map_georef_needs_publish = true; }

    /// For visualization on ROS: a decaying "cloud" of the last deskewed scans.
    mrpt::maps::CPointsMap::ConstPtr last_deskewed_scan_for_publishing;

    /// To handle post-re-localization. >0 means we are "recovering" from a request to re-localize:
    uint32_t step_counter_post_relocalization = 0;

    // GNSS: keep a list of recent observations to later on search the one
    // closest to each LIDAR observation:
    std::map<mrpt::Clock::time_point, std::shared_ptr<const mrpt::obs::CObservationGPS>> last_gnss_;

    // Visualization:
    // Cache only the *expensive*, read-only-after-load vehicle model
    // children (typically CAssimpModel). Each update builds a fresh
    // CSetOfObjects around them and hands it to MolaViz, so we never
    // share a long-lived, mutable object pointer with the GUI thread.
    // glVehicleCachedBuilt tracks whether loading has already happened.
    std::vector<mrpt::opengl::CRenderizable::Ptr> glVehicleModels;
    bool glVehicleModelsLoaded = false;
    // Worker-private growing buffer for the estimated path. Never handed
    // to the GUI thread directly: each update clones it into a fresh
    // CSetOfObjects wrapper before dispatch.
    mrpt::opengl::CSetOfLines::Ptr glEstimatedPath;
    int mapUpdateCnt = std::numeric_limits<int>::max();

    // List of old observations to be unload()'ed, to save RAM if:
    // 1) building a simplemap, and
    // 2) Using a dataset source that supports lazy-load:
    mutable std::map<mrpt::Clock::time_point, mrpt::obs::CSensoryFrame::Ptr>
      past_simplemaps_observations;

    /// Used to estimate sensor rate, mapped by sensorLabel
    std::map<std::string, mrpt::containers::circular_buffer<double>> recent_lidar_stamps;

    /// Used to estimate sensor rate
    mrpt::containers::circular_buffer<double> recent_imu_stamps{1500};

    /// Used to estimate GNSS sensor rate
    mrpt::containers::circular_buffer<double> recent_gnss_stamps{100};

    /// Returns the rates (Hz) of incoming LiDAR, IMU, and GNSS sensors
    /// for the past few seconds. A rate of 0.0 means no data or stale data.
    std::tuple<double, double, double> get_sensor_rates();

    void append_lidar_stamp(
      const std::string & sensorLabel, const mrpt::Clock::time_point & stamp,
      const mrpt::system::COutputLogger & logger);
    void append_imu_stamp(
      const mrpt::Clock::time_point & stamp, const mrpt::system::COutputLogger & logger);
    void append_gnss_stamp(
      const mrpt::Clock::time_point & stamp, const mrpt::system::COutputLogger & logger);

  };  // end of MethodState

  /** The worker thread pool with 1 thread for processing incoming observations.
   *  Uses POLICY_DROP_OLD: when a new scan is enqueued while one is already
   *  waiting (the worker thread being busy with a previous one), the pool
   *  itself discards the older, not-yet-started scan, so the worker always
   *  resumes on the freshest available one instead of grinding through a deep
   *  backlog ("drop stale, keep freshest"). Running tasks are never aborted. */
  mrpt::WorkerThreadsPool worker_lidar_{
    1 /*num threads*/, mrpt::WorkerThreadsPool::POLICY_DROP_OLD, "worker_lidar"};

  std::multimap<double /*timestamp*/, CObservation::ConstPtr> worker_lidar_wait_for_imu_list_;
  std::mutex worker_lidar_wait_for_imu_list_mtx_;

  /// Timestamp (seconds, sensor clock) up to which IMU data has actually been
  /// *fed* into the de-skew LocalVelocityBuffer (updated at the end of the IMU
  /// feeding in onIMUImpl). A waiting scan may only be released to the worker
  /// once this passes its own timestamp by one scan period, i.e. once the IMU
  /// covering the scan's whole span is available for de-skew. Kept as an atomic
  /// so the wait-list can be drained (see releaseReadyLidarScansToWorker) from
  /// the sensor-input thread too, without waiting for the (FIFO, possibly
  /// backed-up) IMU worker to run its own drain -- decoupling scan release from
  /// IMU-processing latency.
  std::atomic<double> latest_fed_imu_time_{0};

  /// Cached estimate of the LiDAR scan period [s], read lock-free by
  /// releaseReadyLidarScansToWorker(). Estimated from consecutive scan *arrival*
  /// (sensor) timestamps rather than the processed-scan rate: under heavy
  /// dropping the processed rate collapses, which would otherwise inflate this
  /// period and make scans wait for far more IMU than they actually need,
  /// needlessly deepening the wait list.
  std::atomic<double> lidar_scan_period_{0.1};
  /// Sensor timestamp [s] of the previous LiDAR scan seen at input, for the
  /// arrival-based period estimate above. Guarded by the wait-list mutex.
  double last_lidar_arrival_stamp_ = 0;

#if defined(MOLA_LO_HAS_MP2P_GRAVITY_PRIOR)
  /// Builds the yaw-free rank-2 gravity observation for the current scan from
  /// the accelerometer gravity estimate, expressed against the map-frame
  /// gravity direction captured at the map origin. nullopt if no reading yet.
  /// Caller must hold state_mtx_.
  [[nodiscard]] std::optional<mp2p_icp::GravityPrior> buildGravityPrior() const;
#endif

  /// The gravity sigma [rad] actually applied this scan: the configured
  /// `imu_gravity_correction.sigma_deg`, optionally widened by the measured
  /// direction dispersion (see `adaptive_sigma`). Caller must hold state_mtx_.
  [[nodiscard]] double effectiveGravitySigmaRad() const;

  /** The worker thread pool with 1 thread for processing incoming observations*/
  mrpt::WorkerThreadsPool worker_others_{
    1 /*num threads*/, mrpt::WorkerThreadsPool::POLICY_FIFO, "worker_imu"};

  /** The worker thread pool with 1 thread for processing saving lazy-load observations to disk */
  mutable mrpt::WorkerThreadsPool worker_disk_io_{
    1 /*num threads*/, mrpt::WorkerThreadsPool::POLICY_FIFO, "worker_disk"};

  /** Runs colorization tasks for clouds to be shown in the gui */
  // Single thread: POLICY_DROP_OLD skips stale frames while 1 thread ensures
  // serial ordering so a newer clear cannot be overwritten by an older frame's lambda.
  mrpt::WorkerThreadsPool worker_viz_{1, mrpt::WorkerThreadsPool::POLICY_DROP_OLD, "worker_viz"};

  MethodState state_;
  const MethodState & state() const { return state_; }
  MethodState stateCopy() const { return state_; }

#ifdef MOLA_KERNEL_VIZ_HAS_METRICS
  /** Metric plot channels (mola_viz_imgui "Plots" menu); lazily registered
   *  from the first onLidarImpl() call, once visualizer_ is available.
   *  Guarded by the feature macro so this module still builds against an
   *  older mola_kernel that predates register_metric()/push_metric(). */
  MetricChannel::Ptr metric_icp_time_ms_;
  MetricChannel::Ptr metric_icp_goodness_;
  MetricChannel::Ptr metric_onlidar_time_ms_;
  MetricChannel::Ptr metric_update_local_map_time_ms_;
#endif

  // Accessing this struct in gui_ requires acquiring state_gui_mtx_
  struct StateUI
  {
    StateUI() = default;

    double timestampLastUpdateUI = 0;
    bool was_waiting_for_lidar_data = true;

    bool gui_created = false;
    mola::gui::LiveString::Ptr lbIcpQuality;
    mola::gui::LiveString::Ptr lbSensorRates;
    mola::gui::LiveString::Ptr lbSensorRange;
    mola::gui::LiveString::Ptr lbTime;
    mola::gui::LiveString::Ptr lbSpeed;
    mola::gui::LiveString::Ptr lbLidarQueue;
    mola::gui::LiveString::Ptr lbMapStats;
  };

  // Accessing this struct in gui_ requires acquiring state_gui_mtx_
  StateUI gui_;

  /// The configuration used in the last call to initialize()
  Yaml lastInitConfig_;

  bool destructor_called_ = false;
  mutable std::mutex is_busy_mtx_;
  /// Guards MethodState::drop_frames_stats_* (see addDropStats()/getDropStats()).
  mutable std::mutex drop_stats_mtx_;
  mutable std::mutex state_flags_mtx_;
  mutable std::mutex state_mtx_;
  mutable std::mutex state_trajectory_mtx_;
  mutable std::recursive_mutex state_simplemap_mtx_;
  mutable std::mutex state_gui_mtx_;

  /// The list of pending tasks from enqueue_request().
  /// Protected exclusively by requests_mtx_. Must never be accessed while
  /// holding state_mtx_ to avoid lock-order inversions.
  std::vector<std::function<void()>> requests_;
  std::mutex requests_mtx_;

  // --- Synchronous bodies for the public async API entry points. ---
  // These run on the internal lidar worker thread (or spin thread) after
  // being enqueued. They take whatever state_/simplemap_/... locks they need
  // themselves, so their *callers* (ROS service threads, GUI threads)
  // never touch these mutexes directly.
  void relocalize_near_pose_pdf_impl(const mrpt::poses::CPose3DPDFGaussian & p);
  void relocalize_from_gnss_impl();
  MapServer::ReturnStatus map_load_impl(const std::string & path);
  MapServer::ReturnStatus map_save_impl(const std::string & path);

  /// Locks drop_stats_mtx_ internally to update the drop-stats window.
  void addDropStats(bool frame_is_dropped);

  /// Returns the ratio [0,1] of lidar frames dropped due to slow processing in the last few seconds.
  double getDropStats() const;

  // Process requests_(), at the spinOnce() rate.
  void processPendingUserRequests();

  /// `readyTimestamp` is when the scan became ready for processing (used to
  /// report the "delay_onNewObs_to_process" queueing-delay metric); it cannot
  /// be measured via profiler_.enter()/leave() here since worker_lidar_'s
  /// POLICY_DROP_OLD may discard a queued scan before it ever runs onLidar().
  void onLidar(const CObservation::ConstPtr & o, double readyTimestamp);
  void processLidarScan(const CObservation::ConstPtr & obs);

  void onIMU(const CObservation::ConstPtr & o);
  void onIMUImpl(const CObservation::ConstPtr & o);

  void onGPS(const CObservation::ConstPtr & o);
  void onGPSImpl(const CObservation::ConstPtr & o);

  // Adaptive threshold method:
  void doUpdateAdaptiveThreshold();

  void doInitializeEstimatedObservationRadius(const mrpt::obs::CObservation & o);
  void doUpdateEstimatedObservationRadius(const mp2p_icp::metric_map_t & m);

  /// Returns false if the scan/observation is not valid:
  bool doCheckIsValidObservation(const mp2p_icp::metric_map_t & m);

  void updatePipelineDynamicVariables(const mrpt::Clock::time_point & stamp);
  void updatePipelineTwistVariables(const mrpt::math::TTwist3D & tw);
  void updatePipelineDynamicVariablesRobotPoseOnly();

  void updateVisualization(
    const mp2p_icp::metric_map_t & currentObservation,
    const mrpt::maps::CPointsMap::Ptr & deskewedCloud);

  void updateVisualizationInitVehFrame();
  void updateVisualizationCurrentObservation(
    const mp2p_icp::metric_map_t & currentObservation,
    const mrpt::maps::CPointsMap::Ptr & deskewedCloud);
  void updateVisualizationLocalMap(std::vector<std::function<void()>> & updateTasks);
  void updateVisualizationPath(std::vector<std::function<void()>> & updateTasks);
  void updateVisualizationGravityVector(std::vector<std::function<void()>> & updateTasks);
  void updateVisualizationTextLabels();
  void updateVisualizationAlways();

  void internalBuildGUI();
  mola::gui::Tab buildTabStatus();
  mola::gui::Tab buildTabControl();
  mola::gui::Tab buildTabView();

  void doRemoveCloudsWithDecay();

  /// Movable scene-frame name to draw 3D objects under (see
  /// Visualization::render_in_movable_frame). Empty => draw at the viewport
  /// root (standalone / older mola_kernel).
  [[nodiscard]] std::string vizParentFrame() const;

  void onExternalMapUpdate(const MapSourceBase::MapUpdate & mu);
  void onExternalLocalizationUpdate(const LocalizationSourceBase::LocalizationUpdate & lu);

  void doPublishUpdatedLocalization(const mrpt::Clock::time_point & scan_ref_time);

  void doPublishUpdatedLocalMap(const mrpt::Clock::time_point & scan_ref_time);

  void doPublishDeskewedScan(const mrpt::Clock::time_point & scan_ref_time);

  void doWriteDebugTracesFile(const mrpt::Clock::time_point & scan_ref_time);
  std::optional<std::ofstream> debug_traces_of_;

  void unloadPastSimplemapObservations(size_t maxSizeUnloadQueue) const;

  void handleUnloadSinglePastObservation(CObservation::Ptr & o) const;

  void onPublishDiagnostics();

#if MOLA_HAS_DIAGNOSTICS_PROVIDER
  /// REP-107 structured diagnostics, collected by BridgeROS2 at ~1 Hz.
  void getDiagnostics(std::vector<mola::DiagnosticStatusMsg> & status) override;
#endif
  void handleInitialLocalization();
  void handleInitialLocalizationStateEstimation();
  void handleInitialLocalizationDoInitFromPose(
    const mrpt::poses::CPose3DPDFGaussian & initPose, bool resetStateEstimator);

  bool isPipelineUsingIMU() const;
  /// Same as isPipelineUsingIMU(), but assumes state_mtx_ is already held by the caller.
  bool isPipelineUsingIMU_locked() const;
  void sendLidarScanToProcessQueue(const CObservation::ConstPtr & o);

  /** Hands a LiDAR scan that is ready for processing (i.e. already has the IMU
   *  data it needs for de-skew, when applicable) to the single worker thread.
   *  worker_lidar_'s POLICY_DROP_OLD implements the "drop stale, keep freshest"
   *  policy itself: if the worker is idle the scan starts right away; if it is
   *  busy, this call replaces any older not-yet-started scan still queued
   *  behind it. This method only adds the drop-stats bookkeeping the pool
   *  itself doesn't provide. Safe to call from any thread. */
  void submitReadyLidarScanToWorker(const CObservation::ConstPtr & o);
  /// Number of LiDAR scans currently running or queued on worker_lidar_ (0, 1, or 2).
  int pendingLidarScanCount() const;

  /** Releases to the worker every LiDAR scan on worker_lidar_wait_for_imu_list_
   *  whose whole time span is already covered by IMU data fed into the de-skew
   *  buffer (i.e. latest_fed_imu_time_ is more than one scan period past the
   *  scan timestamp). On an IMU catch-up burst several scans can qualify at
   *  once; only the freshest is submitted (the pool would drop the rest anyway).
   *  Takes no heavy locks (only the wait-list mutex + atomics), so it can run on
   *  the sensor-input thread while onLidar holds state_mtx_; this decouples scan
   *  release from IMU-worker processing latency. No-op for LO (empty wait list). */
  void releaseReadyLidarScansToWorker();
  mp2p_icp::metric_map_t::Ptr observationFromRawSensor(const mrpt::obs::CSensoryFrame & sf);
  mrpt::obs::CSensoryFrame collectRawObservations(const mrpt::obs::CObservation::ConstPtr & obs);

  void onInitializePersistentState();

  /** Loads the local map (and optional simplemap) from the paths stored in
   *  params_.local_map_updates.load_existing_local_map and
   *  params_.simplemap.load_existing_simple_map.
   *  Safe to call from initialize_frontend() (startup) or from spinOnce()
   *  (deferred). Acquires its own locks internally.
   */
  void doPreloadLocalMap();

  /// True when map loading has been deferred to spinOnce() via load_map_after_gui_init.
  bool pending_preload_map_ = false;

  void doUpdateSimpleMap(
    const mrpt::obs::CSensoryFrame & sf, bool distance_enough_sm,
    const mp2p_icp::metric_map_t::Ptr & observation, const mrpt::Clock::time_point & scan_ref_time,
    const mrpt::maps::CPointsMap::Ptr & deskewedCloud);

  /** Appends a "metadata" CObservationComment (frame bbox + the local velocity
   *  buffer needed for precise later deskew) to a keyframe's sensory frame.
   *  Shared by the self-built simplemap and the shared-keyframe-map push so both
   *  carry the same per-keyframe velocity window. */
  void appendKeyframeMetadataObs(
    mrpt::obs::CSensoryFrame & keyframe_obs, const mrpt::Clock::time_point & scan_ref_time,
    const mp2p_icp::metric_map_t & observation);
};

namespace detail
{
template <typename T, std::size_t... Is>
constexpr std::array<T, sizeof...(Is)> create_array(
  T value, [[maybe_unused]] std::index_sequence<Is...> seq)
{
  // cast Is to void to remove the warning: unused value
  return {{(static_cast<void>(Is), value)...}};
}
}  // namespace detail

template <std::size_t N, typename T>
constexpr std::array<T, N> create_array(const T & value)
{
  return detail::create_array(value, std::make_index_sequence<N>());
}
}  // namespace mola

MRPT_ENUM_TYPE_BEGIN_NAMESPACE(mola, mola::InitLocalization)
MRPT_FILL_ENUM(InitLocalization::FixedPose);
MRPT_FILL_ENUM(InitLocalization::FromStateEstimator);
MRPT_FILL_ENUM(InitLocalization::PitchAndRollFromIMU);
MRPT_ENUM_TYPE_END()
