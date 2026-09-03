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
#if __has_include(<mola_kernel/interfaces/TransformTreeSource.h>)
#include <mola_kernel/interfaces/TransformTreeSource.h>
/** Feature macro: mola_kernel provides mola::TransformTreeSource, enabling
 *  the optional /tf tree visualization. */
#define MOLA_HAS_TRANSFORM_TREE_SOURCE 1
#endif
#include <mola_kernel/version.h>

// Other packages:
#include <mola_imu_preintegration/ImuInitialCalibrator.h>
#if __has_include(<mola_imu_preintegration/MapGravityEstimator.h>)
#include <mola_imu_preintegration/MapGravityEstimator.h>
/** Feature macro: mola_imu_preintegration provides mola::imu::MapGravityEstimator,
 *  used here to estimate the gravity direction IN THE MAP FRAME instead of
 *  freezing it from a single accelerometer average at the first keyframe. */
#define MOLA_LO_HAS_MAP_GRAVITY_ESTIMATOR 1
#endif
#include <mola_lidar_odometry/ImuScanSync.h>
#include <mola_lidar_odometry/KeyframeDecider.h>
#include <mola_lidar_odometry/MapFrameRelevel.h>

/** Feature macro: the one-off map-frame gauge change is available.
 *
 *  It needs three things at once, from three different packages, so it is
 *  gated on all of them rather than on this package alone: the local helpers
 *  here, SearchablePoseList::transform_left_multiply() from mola_pose_list
 *  (to carry the keyframe bookkeeping across the change) and
 *  NavStateFilter::transform_frame() from mola_kernel (to carry the estimator
 *  state). Against an older set the whole feature compiles out and enabling it
 *  at runtime stands down with a warning, which is deliberate: applying the
 *  rotation to only some of the state would leave the session inconsistent,
 *  which is worse than not applying it at all.
 */
#if defined(MOLA_LO_HAS_MAP_FRAME_RELEVEL) &&            \
  defined(MOLA_POSE_LIST_HAS_TRANSFORM_LEFT_MULTIPLY) && \
  defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_TRANSFORM_FRAME)
#define MOLA_LO_CAN_RELEVEL_MAP_FRAME 1
#endif

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
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/typemeta/TEnumType.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSetOfObjects.h>

// STD:
#include <array>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
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

  /** Processes the LiDAR scans still held back waiting for IMU data, using
     *  whatever IMU samples did arrive, and blocks until the worker is idle.
     *
     *  Call it once the input is known to be over (end of a dataset, end of a
     *  rosbag replay): those scans wait for IMU that will never arrive, and
     *  without this they are silently discarded, losing the tail of the
     *  estimated trajectory. shutdownCleanup() calls it as well, so pipelines
     *  going through onQuit() need no explicit call.
     *  As during normal operation, only the freshest pending scan is actually
     *  processed; the older ones are accounted as dropped.
     */
  void flushPendingLidarScans();

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

    /** Quantile of the per-point norms used as ESTIMATED_OBSERVATION_RADIUS,
     * in (0, 1]. **1.0 means the bounding-box max-norm**, which is what this
     * estimate has always been and remains the default.
     *
     * The max-norm is an outlier statistic, not a scene scale: one far return
     * sets it. Measured on three lidars recorded simultaneously in one room,
     * it reads 13.8, 18.0 and 101.8 m -- a 7.4x disagreement about a fixed
     * scene -- while across every outdoor sequence measured it spans only
     * 69.8-91.2 m. Six shipped parameters are derived from it (`range_min`,
     * `range_max`, the keyframe distances, the local-map extent), so an
     * estimate with that failure mode has almost no leverage where tuning is
     * needed and a great deal of spurious leverage where it is not.
     *
     * A quantile below 1.0 (0.98 is a reasonable choice) reads the same on
     * scenes whose returns really do reach that far, and stops one stray
     * return from setting the scale of a room.
     */
    double observation_radius_quantile = 1.0;

    /** Cap on how many points are sampled to evaluate
     * `observation_radius_quantile`. The quantile needs a distribution, not
     * every point; a strided sample of a few thousand from a 60k-point scan
     * settles it to well under a metre, and keeps this O(sample) rather than
     * O(scan) on every frame. Ignored when the quantile is 1.0.
     */
    uint32_t observation_radius_quantile_max_samples = 8000;

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

      // --- Robot /tf tree ---
      /** Draws the subtree of coordinate frames below tf_tree_root_frame
       * (e.g. a legged robot's joints) as it moves. Opt-in: it requires the
       * data source to implement mola::TransformTreeSource, and costs one
       * subtree snapshot per visualization update. */
      bool show_tf_tree = false;

      /** Subtree root. Empty (default) means "ask the data source" (its
       * base_link_frame_id, via TransformTreeSource). */
      std::string tf_tree_root_frame;

      float tf_tree_corner_size = 0.1f;  //! [m]

      /** Draws a link from each frame to its parent. Rendered as thin
       * cylinders rather than GL lines, which MRPT cannot draw with a
       * configurable thickness and are barely visible. */
      bool tf_tree_show_links = true;

      /** Radius of the tf_tree_show_links cylinders. */
      float tf_tree_link_radius = 0.02f;  //! [m]

      /** Draws each frame's name next to it. */
      bool tf_tree_show_names = false;

      /** Comma-separated frame names to leave out, together with their own
       * subtrees. Datasets do publish inverted edges (e.g. "base -> odom"
       * instead of "odom -> base"), which would otherwise drag a whole
       * unrelated, far-away subtree into the robot's own tree. */
      std::string tf_tree_exclude_frames;

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

      /// Seconds of accelerometer data to average while stationary to estimate Pitch & Roll.
      /// This is the knob that actually determines accuracy: the error is dominated by platform
      /// motion during the window, not by sensor noise, so what matters is the DURATION and not
      /// how many samples the sensor happens to deliver in it.
      double imu_initial_calibration_window_seconds = 1.0;

      /// Minimum number of samples inside the window above. A sanity floor to reject a
      /// degenerate handful of samples; it is not what sets the averaging time.
      uint32_t imu_initial_calibration_min_samples = 20;

      /// Maximum RMS angular dispersion [deg] of the accelerometer directions in the window for
      /// it to be accepted as measuring gravity. While it is exceeded, initialization is
      /// deferred and retried with a fresher window, instead of freezing an attitude taken while
      /// the platform was being jostled. 0 disables the gate.
      double imu_initial_calibration_max_dispersion_deg = 1.5;

      /// How long [s] the dispersion gate above may defer initialization before accepting the
      /// most recent window anyway, so a permanently dynamic start still initializes.
      /// 0 waits indefinitely for a quiet window.
      double imu_initial_calibration_dispersion_timeout = 5.0;

      /// DEPRECATED, use imu_initial_calibration_window_seconds instead.
      /// Number of IMU (accelerometer) samples to accumulate to estimate Pitch & Roll. Couples
      /// the averaging time to the sensor rate, and cannot be satisfied at all by a low-rate IMU
      /// if the samples do not fit within imu_initial_calibration_max_age. Only honored when it
      /// is set in the YAML and imu_initial_calibration_window_seconds is not.
      uint32_t imu_initial_calibration_sample_count = 50;

      /// Maximum time span (in seconds) for the "imu_initial_calibration_sample_count" IMU
      /// samples. Only used by that deprecated path: in time-window mode the buffer horizon is
      /// the window itself.
      double imu_initial_calibration_max_age = 0.75;

      /// Set by initialize() when only the deprecated sample-count parameter is present in the
      /// YAML, in which case the legacy readiness rule is preserved as-is.
      bool imu_initial_calibration_legacy_mode = false;

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

      /// Maximum RMS angular dispersion [deg] of the buffered accelerometer directions for the
      /// one-shot map-origin verticality capture to be accepted. The capture decides the map's
      /// vertical reference for the whole run, so while this is exceeded it is deferred and
      /// retried at the next scan rather than freezing a reading taken during a footfall.
      /// 0 disables the gate.
      double map_origin_max_dispersion_deg = 1.0;

      /// How long [s] the capture may be deferred by the gate above before it is taken anyway.
      /// Measured from the first scan at which accelerometer data was available.
      /// 0 defers indefinitely until a quiet reading shows up.
      double map_origin_capture_timeout = 3.0;

      /// Take the verticality reading from an odometry source's ABSOLUTE
      /// attitude instead of from the accelerometer.
      ///
      /// Only the "up" axis is taken, never the position or the heading. The
      /// source's reference frame differs from the map frame by an unknown yaw
      /// and translation, and neither of those touches the direction of
      /// gravity, so the vertical transfers between the two frames exactly
      /// while nothing else does.
      ///
      /// The motivation is that an accelerometer only measures gravity while
      /// the platform is quasi-static, so on a legged or otherwise
      /// continuously-accelerating platform `adaptive_sigma` correctly stands
      /// the constraint down almost all of the time, leaving the vertical
      /// unconstrained for the whole run. A kinematic-inertial state estimator
      /// on the platform itself does not have that limitation and publishes an
      /// attitude that stays gravity-referenced while walking.
      ///
      /// The map-origin reference is then captured from the same source (see
      /// captureMapOriginVerticality), because mixing a reading from one
      /// source with a reference from another injects the constant offset
      /// between them as a permanent map tilt.
      ///
      /// Only honored by the rank-2 prior path; `use_rank2_prior: false`
      /// ignores it, with a warning at initialization.
      struct OdometryAttitude
      {
        bool enabled = false;

        /// Sensor label of the odometry observation to read. It must carry a
        /// full 3D attitude, i.e. arrive as mrpt::obs::CObservationRobotPose;
        /// planar CObservationOdometry has no pitch or roll to offer and is
        /// ignored.
        std::string sensor_label = "odom_wheels";

        /// Sigma [degrees] of the verticality constraint when the reading
        /// comes from this source. `adaptive_sigma` does not apply here: it
        /// widens by accelerometer dispersion, which says nothing about an
        /// external attitude estimate.
        double sigma_deg = 1.0;

        /// Maximum age [s] of the reading, measured against the newest
        /// observation timestamp seen by the system. Older readings are
        /// ignored and the accelerometer is used instead, so a source that
        /// stops publishing degrades to the previous behavior rather than
        /// freezing the vertical. 0 = no age limit.
        double max_age_seconds = 0.5;

        void initialize(const Yaml & c);
      };

      OdometryAttitude odometry_attitude;

      /// Estimate the map-frame gravity direction online, instead of freezing
      /// it from one accelerometer average at the first keyframe.
      ///
      /// The frozen capture is only as good as `averaging_samples` worth of
      /// accelerometer while the platform is not actually static: measured on
      /// a handheld sequence, independent 50 ms averages disagree by ~4 deg
      /// RMS, and whatever value happens to be captured then biases the
      /// verticality reference for the whole run.
      ///
      /// mola::imu::MapGravityEstimator instead solves for gravity in the map
      /// frame (plus IMU biases) from preintegrated IMU and this odometry's
      /// own relative attitudes and velocities, so platform acceleration
      /// cancels and no quasi-static window is needed. Its `up_map` and its
      /// earned sigma then replace the frozen ones.
      struct MapGravity
      {
        bool enabled = false;

        /// Run a solve() every N closed intervals (a solve is a small
        /// Gauss-Newton over the window, but not free).
        uint32_t solve_every_n = 5;

        /// Minimum wall-clock span [s] of one interval. Gravity is recovered as
        /// (v_to - v_from - R_from*dV)/dt, so a velocity error eps shows up as
        /// a gravity error eps/dt: closing an interval every scan (dt ~ 0.1 s)
        /// turns a 0.1 m/s velocity error into ~6 deg of apparent tilt.
        double min_interval_seconds = 1.0;

        /// Run the estimator and log its result, but do NOT let it influence
        /// the verticality constraint. The trajectory then comes out identical
        /// to a run with `enabled: false`, which is what makes the estimate
        /// scoreable against ground truth: with the feedback loop closed, the
        /// map frame it is estimating is partly its own doing.
        ///
        /// Use this to validate the estimator on a new dataset before trusting
        /// it, and keep it OFF in production.
        bool log_only = false;

        /// Rotate the MAP FRAME itself, once, so that it becomes
        /// gravity-aligned, instead of only feeding the per-scan verticality
        /// prior. The map frame is the initial body frame, so on a platform
        /// that starts tilted the whole map leans by that tilt for the rest of
        /// the run, and no later mechanism removes it.
        ///
        /// This is a GAUGE change, not a state update: the local map, the
        /// simplemap, the trajectory, the state estimator and the published
        /// `odom` frame are all rotated together about the map origin, so
        /// every relative quantity is preserved exactly. It happens at most
        /// once per session, and never after a map has been loaded or
        /// geo-referenced.
        ///
        /// Off by default: it changes the frame every product of the run is
        /// expressed in.
        bool relevel_map_frame = false;

        /// Number of intervals the estimator must hold before its estimate is
        /// used to re-level the map frame. Deliberately NOT
        /// `min_intervals_for_convergence` (which gates the per-scan prior, a
        /// different consumer with different needs): for leveling the map once,
        /// the estimate is at its best at the FIRST solve and slowly degrades
        /// afterwards, so waiting is harmful. The default corresponds to that
        /// first solve under the shipped `solve_every_n`.
        uint32_t relevel_min_intervals = 5;

        /// Minimum estimated tilt [deg] for the re-level to be worth doing.
        /// Mandatory, and not a formality: the estimate carries an error of its
        /// own, so correcting a map frame that is already level replaces a
        /// small error with a larger one.
        ///
        /// The gate is applied to the ESTIMATE, but the quantity that has to be
        /// large is the TRUE tilt, and the two differ by that same error. So
        /// the threshold is set at about twice the measured p90 error of the
        /// estimate at its firing point, not at one times it. Below the
        /// threshold the correction is permanently stood down (and logged),
        /// rather than retried later.
        double relevel_min_tilt_deg = 3.0;

        /// Options forwarded verbatim to mola::imu::MapGravityEstimator, so its
        /// parameters do not have to be mirrored here. Note that its own
        /// defaults are tuned for a different use: `window_size` in particular
        /// wants to be much larger here (100+ rather than 20), since the whole
        /// point is that verticality information accumulates.
        mrpt::containers::yaml estimator_params;

        void initialize(const Yaml & c);
      };

      MapGravity map_gravity;

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

    /** How long [s] a LiDAR scan may be held waiting for the IMU data covering
     *  its time span before it is processed without it. Measured in *sensor*
     *  time: the newest timestamp received on any input, not the wall clock,
     *  so a given input always yields the same trajectory whatever the machine
     *  load, offline or online.
     *
     *  Without this bound, an IMU that stops mid-run stalls the odometry
     *  permanently, since the scans behind it never become ready. With it, the
     *  odometry degrades to LiDAR-only after the timeout and recovers by itself
     *  when IMU data comes back. Set to 0 to disable and wait indefinitely. */
    double max_time_to_wait_for_imu = 0.5;

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

    // All other fields are protected by state_mtx_, EXCEPT the ones marked
    // below as protected by imu_state_mtx_ (the IMU-derived state; see that
    // mutex's docs for the full list and the lock order).

    // will be true after the first incoming LiDAR frame and re-localization is enabled and run
    bool initial_localization_done = false;

    /// Used for pitch & roll initialization.
    /// Protected by imu_state_mtx_.
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

    /// Protected by imu_state_mtx_.
    GravityEstimator gravity_estimator;

#if defined(MOLA_LO_HAS_MAP_GRAVITY_ESTIMATOR)
    /// Online estimate of gravity in the MAP frame (plus IMU biases), used to
    /// replace the one-shot `gravity_calib_pitch_roll` capture when
    /// `imu_gravity_correction.map_gravity.enabled`.
    struct MapGravityState
    {
      mola::imu::MapGravityEstimator estimator;
      mola::imu::ImuPreintegrator preintegrator;

      /// Wall-clock stamp of the last IMU sample fed to the preintegrator, to
      /// derive dt for the next one.
      std::optional<double> last_imu_time;

      /// Open interval start: stamp, attitude and map-frame velocity captured
      /// when the previous scan was committed.
      std::optional<double> open_t_from;
      mrpt::poses::CPose3D open_R_from;
      mrpt::math::TVector3D open_v_from{0, 0, 0};

      uint32_t intervals_since_solve = 0;

      /// Correction awaiting application to the map frame, set by the solve
      /// loop and consumed (once) by applyMapFrameRelevel(). The solve runs
      /// under imu_state_mtx_, while rotating the map needs the local-map and
      /// simplemap mutexes, which the lock order forbids taking from there.
      std::optional<mrpt::poses::CPose3D> pending_relevel;

      /// Set once the re-level decision has been taken, whichever way it went:
      /// the map frame is a gauge, and changing it more than once per session
      /// would make the run's own output non-comparable with itself.
      bool relevel_decided = false;

      /// The rotation actually applied to the map frame, if any. Recorded in
      /// the map and keyframe metadata so a run stays auditable.
      std::optional<mrpt::poses::CPose3D> applied_relevel;
    };

    /// Protected by imu_state_mtx_.
    MapGravityState map_gravity;
#endif

    /// Gravity-derived (pitch, roll), in radians, captured at the time the first
    /// keyframe (map origin) was created. The IMU gravity estimator reports
    /// *absolute* tilt with respect to true vertical, while the map/global frame
    /// may itself not be exactly level (e.g. `fixed_initial_pose` has nonzero
    /// pitch/roll, or the vehicle was on a slope at start-up). This calibration
    /// offset is required to correctly re-express later absolute IMU tilt
    /// readings relative to the (possibly non-level) map frame.
    std::optional<std::pair<double, double>> gravity_calib_pitch_roll;

    /// Sensor timestamp of the first scan at which an accelerometer average was available for
    /// the map-origin verticality capture, so the dispersion gate's timeout can be measured.
    std::optional<double> gravity_calib_first_available_time;

    /// Newest verticality reading taken from an odometry source's absolute
    /// attitude, when `imu_gravity_correction.odometry_attitude` is enabled.
    /// Protected by imu_state_mtx_.
    struct OdometryAttitudeState
    {
      /// "Up" direction in the vehicle frame.
      mrpt::math::TVector3D up_body{0, 0, 1};

      /// Observation timestamp, on the same sensor-time scale as
      /// `latest_obs_time_`, so the two can be compared directly.
      double timestamp = 0;

      bool valid = false;
    };

    /// Protected by imu_state_mtx_.
    OdometryAttitudeState odom_attitude;

    /// True when the map-origin verticality reference was captured from the
    /// odometry attitude source. The per-scan reading is then taken from that
    /// same source and from nowhere else: a reading referenced against a
    /// vertical defined by a different sensor carries the constant offset
    /// between the two as a permanent map tilt, which is the error this
    /// reference exists to prevent.
    /// Protected by imu_state_mtx_.
    bool gravity_calib_from_odometry = false;

    /// When the map-origin capture first had to wait for the odometry attitude
    /// source, on the sensor-time scale. Bounds that wait.
    /// Protected by imu_state_mtx_.
    std::optional<double> odom_attitude_wait_since;

    /// Vehicle pose at the instant `gravity_calib_pitch_roll` was captured.
    /// The capture is attempted at the first keyframe, but the accelerometer
    /// average is often not available yet there: at the very first scan the
    /// IMU buffer holds only the few milliseconds that arrived before it, and
    /// the quasi-static gate rejects most of those on a moving platform. It is
    /// therefore retried on later scans, and the reading has to be transported
    /// through the pose it was taken at to still refer to the map frame.
    /// Equals `fixed_initial_pose` when the first attempt succeeds, which is
    /// what makes the retry a strict extension of the original behavior.
    std::optional<mrpt::poses::CPose3D> gravity_calib_pose;

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

#if defined(MOLA_HAS_TRANSFORM_TREE_SOURCE)
    // Data source exposing a /tf tree (dataset reader or live ROS bridge), if
    // any is present in the running MOLA system. Optional: nullptr if none is
    // found, in which case the /tf tree visualization stays empty.
    std::shared_ptr<mola::TransformTreeSource> transform_tree_source;
#endif

    std::optional<NavState> last_motion_model_output;

    /// The source of "dynamic variables" in ICP pipelines.
    /// Protected by imu_state_mtx_: besides the variable map and the realize()
    /// flags, it owns the localVelocityBuffer that IMU samples write and the
    /// LiDAR deskew stage reads.
    mp2p_icp::ParameterSource parameter_source;

    // KISS-ICP-like adaptive threshold method:
    double adapt_thres_sigma = 0;  // 0: initial

    // Counter of consecutive bad ICPs; drives the optional sustained-failure
    // recovery in AdaptiveThreshold.
    int consecutive_bad_icps = 0;

    /// Run totals, reported once at shutdown. `registration_no_motion_model` is
    /// the one that had no aggregate before: the front end already logs a
    /// throttled warning when the state estimator returns nothing (or returns a
    /// prediction too uncertain to use) and falls back to a zero-motion initial
    /// guess, but a throttled line cannot be counted, so an estimator that was
    /// silently failing on 1 scan in 20 looked identical to one that never
    /// failed. Both counters exclude the very first scan, which has no motion
    /// model by definition.
    size_t registrations_attempted = 0;
    size_t registration_no_motion_model = 0;
    size_t registration_icp_rejected = 0;

    // Automatic estimation of the observation bounding-radius (measured from
    // base_link, not from the sensor — see ESTIMATED_OBSERVATION_RADIUS docs):
    std::optional<double> estimated_observation_radius;
    std::optional<double> instantaneous_observation_radius;

    /// Invocations are protected by imu_state_mtx_: apply_generators() on an
    /// IMU observation appends to parameter_source.localVelocityBuffer.
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
    std::vector<mrpt::viz::CVisualObject::Ptr> glVehicleModels;
    bool glVehicleModelsLoaded = false;
    // Worker-private growing buffer for the estimated path. Never handed
    // to the GUI thread directly: each update clones it into a fresh
    // CSetOfObjects wrapper before dispatch.
    mrpt::viz::CSetOfLines::Ptr glEstimatedPath;
    /// Decimation counter for the local map visualization. Saturating (never
    /// wraps around), so its maximum value means "refresh at the next chance".
    unsigned int mapUpdateCnt = std::numeric_limits<unsigned int>::max();

    // List of old observations to be unload()'ed, to save RAM if:
    // 1) building a simplemap, and
    // 2) Using a dataset source that supports lazy-load:
    mutable std::map<mrpt::Clock::time_point, mrpt::obs::CSensoryFrame::Ptr>
      past_simplemaps_observations;

    /// Used to estimate sensor rate, mapped by sensorLabel
    std::map<std::string, mrpt::containers::circular_buffer<double>> recent_lidar_stamps;

    /// Used to estimate sensor rate
    /// Protected by imu_state_mtx_.
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

  ScanImuWaitList worker_lidar_wait_for_imu_list_;
  std::mutex worker_lidar_wait_for_imu_list_mtx_;

  /// Newest timestamp (seconds, sensor clock) present in pending_imu_. A
  /// waiting scan may only be released to the worker once this reaches the
  /// scan's own IMU coverage end time, i.e. once every IMU sample the scan will
  /// consume has actually been received. Kept as an atomic so the wait list can
  /// be drained (see releaseReadyLidarScansToWorker) from the sensor-input
  /// thread too, without waiting for the LiDAR worker to become free.
  std::atomic<double> latest_imu_time_{0};

  /// Newest timestamp (seconds, sensor clock) seen on *any* input: the system's
  /// notion of "now" in sensor time. It is what bounds how long a scan may wait
  /// for IMU data (params_.max_time_to_wait_for_imu). Taking it from the
  /// observation stream instead of the wall clock is what keeps the outcome
  /// reproducible: the same input always leads to the same decisions.
  std::atomic<double> latest_obs_time_{0};

  /// IMU observations received but not consumed yet. They are consumed by
  /// consumePendingImu(), from the LiDAR worker thread, right before the scan
  /// that needs them is processed, so which samples enter the IMU-derived state
  /// depends on timestamps only and not on how the sensor callbacks interleave.
  /// Protected by imu_state_mtx_.
  PendingImuBuffer pending_imu_;

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

  /// The newest verticality reading from the odometry attitude source, as an
  /// "up" direction in the vehicle frame, or nullopt when that source is
  /// disabled, has produced nothing yet, or its newest reading is older than
  /// `odometry_attitude.max_age_seconds`. Caller must hold imu_state_mtx_.
  [[nodiscard]] std::optional<mrpt::math::TVector3D> odometryUpBody() const;

  /// Captures the map-origin verticality reference from the accelerometer, if
  /// it has not been captured yet and an average is available. Safe (and
  /// intended) to call on every scan: it is a no-op once captured.
  /// `poseAtCapture` is the vehicle pose the reading belongs to, needed to
  /// express it in the map frame. Caller must hold state_mtx_.
  void captureMapOriginVerticality(const mrpt::poses::CPose3D & poseAtCapture);

#if defined(MOLA_LO_HAS_MAP_GRAVITY_ESTIMATOR)
  /// Feeds one IMU observation into the map-gravity preintegrator, in the
  /// vehicle frame. Caller must hold state_mtx_.
  void accumulateImuForMapGravity(const mrpt::obs::CObservationIMU & imu);

  /// Closes the currently open preintegration interval at the just-committed
  /// scan pose, appends it to the map-gravity estimator, and periodically
  /// re-solves. Caller must hold state_mtx_.
  void closeMapGravityInterval(
    double timestamp, const mrpt::poses::CPose3D & pose, const mrpt::math::TTwist3D & twistLocal);

  /// Decides, at most once per session, whether the latest map-gravity
  /// estimate should re-level the map frame, and if so leaves the rotation in
  /// `map_gravity.pending_relevel`. Caller must hold state_mtx_ and
  /// imu_state_mtx_.
  void evaluateMapFrameRelevel(const mola::imu::MapGravityEstimator::Result & r);

  /// Applies a pending map-frame re-level: rotates the local map, the
  /// simplemap, the trajectory, the keyframe deciders, the state estimator and
  /// the cached poses about the map origin, then resets the verticality
  /// reference and the map-gravity estimator. No-op if nothing is pending.
  /// Caller must hold state_mtx_ and NO other state mutex.
  void applyMapFrameRelevel();

#endif

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

  /** Renders the local map for the gui. Same 1-thread POLICY_DROP_OLD rationale
   *  as worker_viz_, but a pool of its own: with one thread, POLICY_DROP_OLD
   *  caps the queue at a single pending task, so sharing worker_viz_ would let
   *  the per-scan current-observation frames drop the (much rarer) local map
   *  render before it ever runs, and would serialize the two renders.
   *
   *  Its task holds raw pointers into `*this` (the profiler and
   *  local_map_content_mtx_, both declared *after* this pool and therefore
   *  destroyed *before* it). That is safe only because shutdownCleanup() calls
   *  clear() on this pool, which joins its thread, and shutdownCleanup() runs
   *  from the destructor body, i.e. before any member is destroyed. Keep it in
   *  that list if the shutdown path is ever reworked.
   *  A still-*pending* render is dropped there rather than waited for: it would
   *  delay shutdown by a full render to draw a frame nobody will see. */
  mrpt::WorkerThreadsPool worker_viz_local_map_{
    1, mrpt::WorkerThreadsPool::POLICY_DROP_OLD, "worker_viz_map"};

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

  /// Guards the IMU-derived state, which the sensor-input thread appends to
  /// (pending_imu_, recent_imu_stamps) while the LiDAR worker thread consumes
  /// it, so the former no longer has to wait on state_mtx_, which
  /// processLidarScan() holds for its whole body (filters, ICP, map update,
  /// visualization). At 200 Hz IMU / 10 Hz LiDAR that made the input thread
  /// block for essentially the entire duration of every scan, and since
  /// releaseReadyLidarScansToWorker() runs at the end of onIMU(), that wait fed
  /// straight back into scan submission latency.
  ///
  /// Covers pending_imu_ and, in MethodState: imu_initializer,
  /// gravity_estimator, map_gravity, recent_imu_stamps, parameter_source (its
  /// variable map, the realize() "evaluated" flags, and localVelocityBuffer),
  /// and any *invocation* of obs_generators (which writes the velocity buffer
  /// and reads those flags).
  ///
  /// Lock order: state_mtx_ -> local_map_content_mtx_ -> imu_state_mtx_; never
  /// the reverse. The sensor-input thread takes only this one (and never the
  /// local map), so it cannot invert the order.
  ///
  /// Recursive because the accessors that take it compose (e.g.
  /// updatePipelineDynamicVariables() -> updatePipelineTwistVariables(),
  /// buildGravityPrior() -> effectiveGravitySigmaRad()); locking at accessor
  /// granularity is what keeps the ownership rule above reviewable, rather than
  /// threading a lock object through a dozen signatures.
  mutable std::recursive_mutex imu_state_mtx_;

  /// Guards the *contents* (layers) of MethodState::local_map.
  /// Rendering the map is O(map size) and would stall every other user of
  /// state_mtx_ (dataset reader, sensor input, executor thread) if done under it,
  /// so the render runs on worker_viz_local_map_ and takes only this one.
  /// Lock order: a thread that needs both must take state_mtx_ first; it must
  /// never be held while acquiring state_mtx_.
  mutable std::mutex local_map_content_mtx_;

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
  void onLidar(
    const CObservation::ConstPtr & o, double readyTimestamp,
    std::optional<double> imuCoverageEndTime);
  void processLidarScan(
    const CObservation::ConstPtr & obs, std::optional<double> imuCoverageEndTime);

  void onIMU(const CObservation::ConstPtr & o);
  void onIMUImpl(const CObservation::ConstPtr & o);

  /** Stores the "up" direction implied by an odometry observation's absolute
   *  attitude, for use as the verticality reading. Like onIMU(), it only
   *  buffers, so it runs inline on the caller's thread and the stored value
   *  stays a function of the input sequence alone. */
  void onOdometryAttitude(const CObservation::ConstPtr & o);

  /** Feeds every pending IMU observation with a timestamp not newer than
   *  `upToTime` into the IMU-derived state (de-skew velocity buffer, initial
   *  pitch/roll calibrator, gravity estimators), in timestamp order, and drops
   *  them from pending_imu_.
   *  Caller must hold state_mtx_. */
  void consumePendingImu(double upToTime);

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

  /// All these methods read state_, so the caller must own state_mtx_ and pass
  /// its lock object down, which they assert on entry.
  void updateVisualization(
    const mp2p_icp::metric_map_t & currentObservation,
    const mrpt::maps::CPointsMap::Ptr & deskewedCloud, std::unique_lock<std::mutex> & lckState);

  void updateVisualizationInitVehFrame();
  void updateVisualizationCurrentObservation(
    const mp2p_icp::metric_map_t & currentObservation,
    const mrpt::maps::CPointsMap::Ptr & deskewedCloud);
  /// Only decides *whether* to refresh the local map view and snapshots what
  /// that takes; the O(map size) render itself is enqueued on
  /// worker_viz_local_map_ (see local_map_content_mtx_).
  void updateVisualizationLocalMap();
  void updateVisualizationPath(std::vector<std::function<void()>> & updateTasks);

  /// Renders the /tf subtree below the configured root frame, as a child of
  /// the vehicle frame (its poses are relative to the robot body). A no-op
  /// unless enabled AND a mola::TransformTreeSource was found at init.
  void updateVisualizationTfTree(
    std::vector<std::function<void()>> & updateTasks, const std::string & vizFrame);
  void updateVisualizationGravityVector(std::vector<std::function<void()>> & updateTasks);
  void updateVisualizationTextLabels();
  void updateVisualizationAlways(std::unique_lock<std::mutex> & lckState);

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
   *  itself doesn't provide. Safe to call from any thread.
   *  \return The enqueued task's future, so a caller that must not race the
   *          worker (flushPendingLidarScans) can wait on this very scan rather
   *          than on sampled busy counters. */
  std::future<void> submitReadyLidarScanToWorker(
    const CObservation::ConstPtr & o, std::optional<double> imuCoverageEndTime);
  /// Number of LiDAR scans currently running or queued on worker_lidar_ (0, 1, or 2).
  int pendingLidarScanCount() const;

  /** Releases to the worker every LiDAR scan on worker_lidar_wait_for_imu_list_
   *  whose whole time span is already covered by received IMU data (i.e.
   *  latest_imu_time_ has reached the scan's own IMU coverage end time).
   *  On an IMU catch-up burst several scans can qualify at
   *  once; only the freshest is submitted (the pool would drop the rest anyway).
   *  Takes no heavy locks (only the wait-list mutex + atomics), so it can run on
   *  the sensor-input thread while onLidar holds state_mtx_; this decouples scan
   *  release from IMU-worker processing latency. No-op for LO (empty wait list). */
  void releaseReadyLidarScansToWorker();

  /** Common implementation of releaseReadyLidarScansToWorker() and
   *  flushPendingLidarScans(): releases every waiting scan whose IMU coverage
   *  end time is not beyond \a upToImuTime. Passing infinity releases them all,
   *  which is what "no more input is coming" means.
   *  \return The submitted scan's future, or an invalid future if none was. */
  std::future<void> releaseLidarScansToWorker(double upToImuTime);
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
