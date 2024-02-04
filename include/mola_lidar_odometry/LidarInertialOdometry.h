/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */
/**
 * @file   LidarOdometry.h
 * @brief  Header for main C++ class exposing LIDAR-inertial odometry
 * @author Jose Luis Blanco Claraco
 * @date   Sep 16, 2023
 */
#pragma once

#include <mola_kernel/interfaces/FrontEndBase.h>
#include <mola_navstate_fuse/NavStateFuse.h>
#include <mola_navstate_fuse/NavStateFuseParams.h>
#include <mp2p_icp/ICP.h>
#include <mp2p_icp/Parameterizable.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/Generator.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/serialization/CSerializable.h>

#include <regex>

namespace mola
{
/** LIDAR-inertial odometry based on ICP against a local metric map model.
 */
class LidarInertialOdometry : public FrontEndBase
{
    DEFINE_MRPT_OBJECT(LidarInertialOdometry, mola)

   public:
    LidarInertialOdometry();
    ~LidarInertialOdometry();

    /** @name Main API
     * @{ */

    // See docs in base class
    void initialize(const Yaml& cfg) override;
    void spinOnce() override;
    void onNewObservation(const CObservation::Ptr& o) override;

    /** Re-initializes the front-end. initialize() needs to be called again. */
    void reset();

    enum class AlignKind : uint8_t
    {
        RegularOdometry = 0,
        NoMotionModel
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

        /** Minimum time (seconds) between scans for being attempted to be
         * aligned. Scans faster than this rate will be just silently ignored.
         */
        double min_time_between_scans = 0.05;

        double max_sensor_range_filter_coefficient = 0.9;
        double absolute_minimum_sensor_range       = 5.0;  // [m]

        struct MapUpdateOptions
        {
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

            void initialize(const Yaml& c, Parameters& parent);
        };

        MapUpdateOptions local_map_updates;

        /** Minimum ICP "goodness" (in the range [0,1]) for a new KeyFrame to be
         * accepted during regular lidar odometry & mapping */
        double min_icp_goodness = 0.4;

        bool pipeline_profiler_enabled = false;
        bool icp_profiler_enabled      = false;
        bool icp_profiler_full_history = false;

        struct Visualization
        {
            int    map_update_decimation    = 10;
            bool   show_trajectory          = true;
            double current_pose_corner_size = 1.5;  //! [m]

            bool gui_subwindow_starts_hidden = false;

            /** If not empty, an optional 3D model (.DAE, etc) to load for
             * visualizing the robot/vehicle pose */
            struct ModelPart
            {
                std::string         file;
                mrpt::math::TPose3D tf;  /// Optional 3D model offset/rotation
                double              scale = 1.0;
            };
            std::vector<ModelPart> model;

            void initialize(const Yaml& c);
        };
        Visualization visualization;

        // KISS-ICP adaptive threshold method:
        struct AdaptiveThreshold
        {
            bool   enabled       = true;
            double initial_sigma = 0.5;
            double min_motion    = 0.10;

            void initialize(const Yaml& c);
        };
        AdaptiveThreshold adaptive_threshold;

        /** ICP parameters for the case of having, or not, a good velocity
         * model that works a good prior. Each entry in the vector is an
         * "ICP stage", to be run as a sequence of coarser to finer detail
         */
        struct ICP_case
        {
            mp2p_icp::ICP::Ptr   icp;
            mp2p_icp::Parameters icpParameters;
        };

        std::map<AlignKind, ICP_case> icp;

        mola::NavStateFuseParams navstate_fuse_params;

        std::vector<std::pair<mp2p_icp::layer_name_t, mp2p_icp::layer_name_t>>
            observation_layers_to_merge_local_map;

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

            void initialize(const Yaml& c, Parameters& parent);
        };

        SimpleMapOptions simplemap;

        // === OUTPUT TRAJECTORY ====
        struct TrajectoryOutputOptions
        {
            bool save_to_file = false;

            /** If save_to_file==true, the final estimated trajectory will be
             * dumped to a file at destruction time */
            std::string output_file = "output.txt";

            void initialize(const Yaml& c);
        };

        TrajectoryOutputOptions estimated_trajectory;

        std::optional<mrpt::math::TTwist3D> initial_twist;

        uint32_t max_worker_thread_queue_before_drop = 500;
    };

    /** Algorithm parameters */
    Parameters params_;

    bool isBusy() const;

    /** Returns a copy of the estimated trajectory, with timestamps for each
     * lidar observation.
     * Multi-thread safe to call.
     */
    mrpt::poses::CPose3DInterpolator estimatedTrajectory() const;

    /** Returns a copy of the estimated simplemap.
     * Multi-thread safe to call.
     */
    mrpt::maps::CSimpleMap reconstructedMap() const;

    /** @} */

   private:
    struct ICP_Input
    {
        using Ptr = std::shared_ptr<ICP_Input>;

        AlignKind                   align_kind{AlignKind::RegularOdometry};
        id_t                        global_id{mola::INVALID_ID};
        id_t                        local_id{mola::INVALID_ID};
        mp2p_icp::metric_map_t::Ptr global_pc, local_pc;
        mrpt::math::TPose3D         init_guess_local_wrt_global;
        mp2p_icp::Parameters        icp_params;

        std::optional<mrpt::poses::CPose3DPDFGaussianInf> prior;

        /** used to identity where does this request come from */
        std::string debug_str;
    };
    struct ICP_Output
    {
        double                          goodness{.0};
        mrpt::poses::CPose3DPDFGaussian found_pose_to_wrt_from;
    };
    void run_one_icp(const ICP_Input& in, ICP_Output& out);

    class SearchablePoseList
    {
       public:
        SearchablePoseList() = default;

        SearchablePoseList(bool measure_from_last_kf_only)
            : from_last_only_(measure_from_last_kf_only)
        {
        }

        bool empty() const
        {
            if (from_last_only_)  //
                return last_kf_ == mrpt::poses::CPose3D::Identity();
            else
                return kf_poses_.empty();
        }

        size_t size() const { return from_last_only_ ? 1 : kf_poses_.size(); }

        void insert(const mrpt::poses::CPose3D& p)
        {
            if (from_last_only_)
            {  //
                last_kf_ = p;
            }
            else
            {
                kf_points_.insertPoint(p.translation());
                kf_poses_.push_back(p);
            }
        }

        [[nodiscard]] std::tuple<
            bool /*isFirst*/, mrpt::poses::CPose3D /*distanceToClosest*/>
            check(const mrpt::poses::CPose3D& p) const;

        void removeAllFartherThan(
            const mrpt::poses::CPose3D& p, const double maxTranslation);

       private:
        // if from_last_only_==true
        mrpt::poses::CPose3D last_kf_ = mrpt::poses::CPose3D::Identity();

        // if from_last_only_==false
        std::deque<mrpt::poses::CPose3D> kf_poses_;
        mrpt::maps::CSimplePointsMap     kf_points_;

        bool from_last_only_ = false;
    };

    /** All variables that hold the algorithm state */
    struct MethodState
    {
        bool initialized = false;
        bool fatal_error = false;

        int worker_tasks = 0;

        std::optional<mrpt::Clock::time_point> last_obs_tim;
        bool                                   last_icp_was_good = true;
        double                                 last_icp_quality  = .0;
        mrpt::poses::CPose3DPDFGaussian last_lidar_pose;  //!< in local map

        // navstate_fuse to merge pose estimates, IMU, odom, estimate twist.
        mola::NavStateFuse navstate_fuse;

        /// The source of "dynamic variables" in ICP pipelines:
        mp2p_icp::ParameterSource icpParameterSource;

        // KISS-ICP-like adaptive threshold method:
        double adapt_thres_sigma = 0;  // 0: initial

        // Automatic estimation of max range:
        std::optional<double> estimated_sensor_max_range;

        mp2p_icp_filters::GeneratorSet   obs_generators;
        mp2p_icp_filters::FilterPipeline pc_filter;
        mrpt::poses::CPose3DInterpolator estimatedTrajectory;
        mrpt::maps::CSimpleMap           reconstructedMap;
        mp2p_icp_filters::GeneratorSet   local_map_generators;
        mp2p_icp::metric_map_t::Ptr      local_map =
            mp2p_icp::metric_map_t::Create();

        // to check for map updates. Defined as optional<> so we enforce
        // setting their type in the ctor:
        std::optional<SearchablePoseList> distanceCheckerLocalMap;
        std::optional<SearchablePoseList> distanceCheckerSimplemap;

        /// See check_for_removal_every_n
        uint32_t localMapCheckForRemovalCounter = 0;

        // Visualization:
        mrpt::opengl::CSetOfObjects::Ptr glVehicleFrame, glLocalMap, glPathGrp;
        mrpt::opengl::CSetOfLines::Ptr   glEstimatedPath;
        int              mapUpdateCnt  = std::numeric_limits<int>::max();
        nanogui::Window* ui            = nullptr;
        nanogui::Label*  lbIcpQuality  = nullptr;
        nanogui::Label*  lbSigma       = nullptr;
        nanogui::Label*  lbSensorRange = nullptr;
        nanogui::Label*  lbTime        = nullptr;
        nanogui::Label*  lbPeriod      = nullptr;
    };

    /** The worker thread pool with 1 thread for processing incomming
     * IMU or LIDAR observations*/
    mrpt::WorkerThreadsPool worker_{
        1 /*num threads*/, mrpt::WorkerThreadsPool::POLICY_FIFO,
        "worker_lidar_odom"};

    MethodState        state_;
    const MethodState& state() const { return state_; }
    MethodState        stateCopy() const { return state_; }

    mutable std::mutex is_busy_mtx_;
    mutable std::mutex stateTrajectory_mtx_;
    mutable std::mutex stateSimpleMap_mtx_;

    void onLidar(const CObservation::Ptr& o);
    void onLidarImpl(const CObservation::Ptr& o);

    void onIMU(const CObservation::Ptr& o);
    void onIMUImpl(const CObservation::Ptr& o);

    void onWheelOdometry(const CObservation::Ptr& o);
    void onWheelOdometryImpl(const CObservation::Ptr& o);

    // KISS-ICP adaptive threshold method:
    void doUpdateAdaptiveThreshold(
        const mrpt::poses::CPose3D& lastMotionModelError);

    void doInitializeEstimatedMaxSensorRange(const mrpt::obs::CObservation& o);
    void doUpdateEstimatedMaxSensorRange(const mp2p_icp::metric_map_t& m);

    void updatePipelineDynamicVariables();

    void updateVisualization();
};

}  // namespace mola
