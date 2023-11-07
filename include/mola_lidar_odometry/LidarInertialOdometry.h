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

#include <mola_imu_preintegration/RotationIntegrator.h>
#include <mola_kernel/interfaces/FrontEndBase.h>
#include <mp2p_icp/ICP.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/Generator.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/serialization/CSerializable.h>

#include <regex>

namespace mola
{
/**
 *
 *
 */
class LidarInertialOdometry : public FrontEndBase
{
    DEFINE_MRPT_OBJECT(LidarInertialOdometry, mola)

   public:
    LidarInertialOdometry();
    ~LidarInertialOdometry() = default;

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
        LidarOdometry = 0,
        NearbyAlign
    };

    struct Parameters
    {
        /** List of sensor labels or regex's to be matched to input observations
         *  to be used as raw lidar observations.
         */
        std::vector<std::regex> lidar_sensor_labels;

        /** Sensor labels or regex to be matched to input observations
         *  to be used as raw IMU observations.
         */
        std::optional<std::regex> imu_sensor_label;

        /** Minimum time (seconds) between scans for being attempted to be
         * aligned. Scans faster than this rate will be just silently ignored.
         */
        double min_time_between_scans{0.2};

        double max_time_to_use_velocity_model = 1.0;

        /** Minimum Euclidean distance (x,y,z) between keyframes inserted into
         * the map [meters]. */
        double min_dist_xyz_between_keyframes{1.0};

        /** Minimum rotation (in 3D space, yaw, pitch,roll, altogether) between
         * keyframes inserted into
         * the map [rad here, degrees in the yaml file]. */
        double min_rotation_between_keyframes{mrpt::DEG2RAD(30.0)};

        /** Minimum ICP "goodness" (in the range [0,1]) for a new KeyFrame to be
         * accepted during regular lidar odometry & mapping */
        double min_icp_goodness{0.4};

        /** Minimum ICP quality for a loop closure to be accepted */
        double min_icp_goodness_lc{0.6};

        bool icp_profiler_enabled      = false;
        bool icp_profiler_full_history = false;

        /** Size of the voxel filter [meters] */
        unsigned int full_pointcloud_decimation{20};
        double       voxel_filter_resolution{.5};
        unsigned int voxel_filter_decimation{1};
        float        voxel_filter_max_e2_e0{30.f}, voxel_filter_max_e1_e0{30.f};
        float voxel_filter_min_e2_e0{100.f}, voxel_filter_min_e1_e0{100.f};

        /** Distance range to check for additional SE(3) edges */
        double       min_dist_to_matching{6.0};
        double       max_dist_to_matching{12.0};
        double       max_dist_to_loop_closure{30.0};
        unsigned int loop_closure_montecarlo_samples{10};
        unsigned int max_nearby_align_checks{2};
        unsigned int min_topo_dist_to_consider_loopclosure{20};

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

        mola::RotationIntegrationParams imu_params;

        std::vector<std::string> observation_layers_to_merge_local_map;

        /** Generate render visualization decoration for every N keyframes */
        int   viz_decor_decimation{5};
        float viz_decor_pointsize{2.0f};
    };

    /** Algorithm parameters */
    Parameters params_;

    bool isBusy() const;

    /** Returns a copy of the estimated trajectory, with timestamps for each
     * lidar observation.
     * Multi-thread safe to call.
     */
    mrpt::poses::CPose3DInterpolator estimatedTrajectory() const;

    /** @} */

   private:
    struct ICP_Input
    {
        using Ptr = std::shared_ptr<ICP_Input>;

        AlignKind                   align_kind{AlignKind::LidarOdometry};
        id_t                        global_id{mola::INVALID_ID};
        id_t                        local_id{mola::INVALID_ID};
        mp2p_icp::metric_map_t::Ptr global_pc, local_pc;
        mrpt::math::TPose3D         init_guess_local_wrt_global;
        mp2p_icp::Parameters        icp_params;

        /** used to identity where does this request come from */
        std::string debug_str;
    };
    struct ICP_Output
    {
        double                          goodness{.0};
        mrpt::poses::CPose3DPDFGaussian found_pose_to_wrt_from;
    };
    void run_one_icp(const ICP_Input& in, ICP_Output& out);

    /** All variables that hold the algorithm state */
    struct MethodState
    {
        bool initialized = false;
        bool fatal_error = false;

        bool busy_imu = false, busy_lidar = false;

        std::optional<mrpt::Clock::time_point> last_obs_tim;
        std::optional<mrpt::math::TTwist3D>    last_iter_twist;
        std::optional<mrpt::poses::CPose3D>    last_pose;  //!< in local map
        mrpt::poses::CPose3D                   current_pose;  //!< in local map
        mrpt::poses::CPose3D                   accum_since_last_kf;

        mp2p_icp_filters::GeneratorSet   obs_generators;
        mp2p_icp_filters::FilterPipeline pc_filter;

        mp2p_icp_filters::GeneratorSet local_map_generators;

        mp2p_icp::metric_map_t::Ptr local_map =
            mp2p_icp::metric_map_t::Create();

        mrpt::poses::CPose3DInterpolator estimatedTrajectory;
    };

    /** The worker thread pool with 1 thread for processing incomming scans */
    mrpt::WorkerThreadsPool worker_lidar_{
        1, mrpt::WorkerThreadsPool::POLICY_FIFO, "worker_lidar"};

    /** The worker thread pool with 1 thread for processing incomming IMU */
    mrpt::WorkerThreadsPool worker_imu_{
        1, mrpt::WorkerThreadsPool::POLICY_FIFO, "worker_imu"};

    MethodState        state_;
    const MethodState& state() const { return state_; }
    MethodState        stateCopy() const { return state_; }

    mutable std::mutex is_busy_mtx_;
    mutable std::mutex stateTrajectory_mtx_;

    void onLidar(const CObservation::Ptr& o);
    void onLidarImpl(const CObservation::Ptr& o);

    void onIMU(const CObservation::Ptr& o);
    void onIMUImpl(const CObservation::Ptr& o);
};

}  // namespace mola
