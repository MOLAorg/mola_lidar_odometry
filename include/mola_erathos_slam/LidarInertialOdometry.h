/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
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
#include <mrpt/serialization/CSerializable.h>

#include <regex>

namespace mola::erathos
{
/**
 *
 *
 */
class LidarInertialOdometry : public FrontEndBase
{
    DEFINE_MRPT_OBJECT(LidarInertialOdometry, mola::erathos)

   public:
    LidarInertialOdometry();
    ~LidarInertialOdometry() = default;

    /** @name Main API
     * @{ */

    // See docs in base class
    void initialize(const Yaml& cfg) override;
    void spinOnce() override;
    void onNewObservation(const CObservation::Ptr& o) override;

    /** Re-initializes the front-end */
    void reset();

    enum class AlignKind : uint8_t
    {
        LidarOdometry,
        NearbyAlign
    };

    struct Parameters
    {
        /** List of sensor labels or regex's to be matched to input observations
         *  to be used as raw lidar observations.
         */
        std::vector<std::regex> lidar_sensor_labels_;

        /** Sensor labels or regex to be matched to input observations
         *  to be used as raw IMU observations.
         */
        std::optional<std::regex> imu_sensor_label_;

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

        /** Generate render visualization decoration for every N keyframes */
        int   viz_decor_decimation{5};
        float viz_decor_pointsize{2.0f};
    };

    /** Algorithm parameters */
    Parameters params_;

    /** @} */

   private:
    struct ICP_Input
    {
        using Ptr = std::shared_ptr<ICP_Input>;

        AlignKind                   align_kind{AlignKind::LidarOdometry};
        id_t                        to_id{mola::INVALID_ID};
        id_t                        from_id{mola::INVALID_ID};
        mp2p_icp::metric_map_t::Ptr to_pc, from_pc;
        mrpt::math::TPose3D         init_guess_to_wrt_from;
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
        std::optional<mrpt::Clock::time_point> last_obs_tim;
        std::optional<mrpt::math::TTwist3D>    last_iter_twist;
        mrpt::poses::CPose3D                   accum_since_last_kf{};

        mp2p_icp_filters::GeneratorSet   pc_generators;
        mp2p_icp_filters::FilterPipeline pc_filter;

        mp2p_icp::metric_map_t::Ptr local_map =
            mp2p_icp::metric_map_t::Create();
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

    void onLidar(const CObservation::Ptr& o);
    void onLidarImpl(const CObservation::Ptr& o);

    void onIMU(const CObservation::Ptr& o);
    void onIMUImpl(const CObservation::Ptr& o);
};

}  // namespace mola::erathos