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
 * @file   mola-lidar-odometry-graph-cli.cpp
 * @brief
 * @author Jose Luis Blanco Claraco
 * @date   Nov 21, 2023
 */

/*
KITTI_SEQ=04 mola-lidar-odometry-graph-cli \
    --input-kitti-seq 04 \
    --input-tum-path results/estim_04.txt  \
    --icp-config src/mola_lidar_odometry/params/icp_odom_params.yaml \
    --filter src/mola_lidar_odometry/params/graph_filter_pipeline.yaml
*/

#include <mola_input_kitti_dataset/KittiOdometryDataset.h>
#include <mola_kernel/pretty_print_exception.h>
#include <mola_lidar_odometry/LidarInertialOdometry.h>
#include <mola_yaml/yaml_helpers.h>
#include <mp2p_icp/icp_pipeline_from_yaml.h>
#include <mp2p_icp_filters/Generator.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/progress.h>

#include <csignal>  // sigaction
#include <cstdlib>
#include <iostream>
#include <string>

// Gtsam:
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <mrpt/poses/gtsam_wrappers.h>

// Declare supported cli switches ===========
static TCLAP::CmdLine cmd("mola-lidar-odometry-graph-cli");

static TCLAP::ValueArg<std::string> argYAML_ICP(
    "", "icp-config", "Input YAML config file (required) (*.yml)", true, "",
    "demo.yml", cmd);

static TCLAP::ValueArg<std::string> argYAML_Filter(
    "", "filter", "Input YAML config file (required) (*.yml)", true, "",
    "demo.yml", cmd);

static TCLAP::ValueArg<std::string> argInPath(
    "", "input-tum-path", "", true, "trajectory.txt", "trajectory.txt", cmd);

static TCLAP::ValueArg<std::string> argOutPath(
    "", "output-tum-path", "", true, "out.txt", "out.txt", cmd);

static TCLAP::ValueArg<int> arg_firstN(
    "", "only-first-n", "Run for the first N steps only (0=default, not used)",
    false, 0, "Number of dataset entries to run", cmd);

static TCLAP::ValueArg<std::string> argKittiSeq(
    "", "input-kitti-seq",
    "INPUT DATASET: Use KITTI dataset sequence number 00|01|...", true, "00",
    "00", cmd);
static TCLAP::ValueArg<double> argKittiAngleDeg(
    "", "kitti-correction-angle-deg",
    "Correction vertical angle offset (see Deschaud,2018)", false, 0.205,
    "0.205 [degrees]", cmd);

static TCLAP::ValueArg<double> argSlidingLen(
    "", "sw-length", "Sliding window length (in meters)", false, 10.0,
    "10.0 [meters]", cmd);

class OfflineDatasetSource
{
   public:
    OfflineDatasetSource()          = default;
    virtual ~OfflineDatasetSource() = default;

    virtual size_t size() const = 0;

    virtual mrpt::obs::CObservation::Ptr getObservation(size_t index) const = 0;
};

class KittiSource : public OfflineDatasetSource
{
   public:
    KittiSource()          = default;
    virtual ~KittiSource() = default;

    void init(const std::string& kittiSeqNumber)
    {
        const auto kittiCfg =
            mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
                R""""(
    params:
      base_dir: ${KITTI_BASE_DIR}
      sequence: '%s'
      time_warp_scale: 1.0
      publish_lidar: true
      publish_image_0: true
      publish_image_1: true
      publish_ground_truth: true
)"""",
                kittiSeqNumber.c_str())));

        kittiDataset_.initialize(kittiCfg);

        if (argKittiAngleDeg.isSet())
            kittiDataset_.VERTICAL_ANGLE_OFFSET =
                mrpt::DEG2RAD(argKittiAngleDeg.getValue());
    }

    size_t size() const override { return kittiDataset_.getTimestepCount(); }

    mrpt::obs::CObservation::Ptr getObservation(size_t index) const override
    {
        return kittiDataset_.getPointCloud(index);
    }

    auto groundTruthPoses() const
    {
        return kittiDataset_.getGroundTruthTrajectory();
    }

   private:
    mutable mola::KittiOdometryDataset kittiDataset_;
};

struct ICP_case
{
    mp2p_icp::ICP::Ptr   icp;
    mp2p_icp::Parameters icpParameters;
};

static void load_icp_set_of_params(
    ICP_case& out, const mrpt::containers::yaml& cfg)
{
    const auto [icp, params] = mp2p_icp::icp_pipeline_from_yaml(cfg);

    out.icp           = icp;
    out.icpParameters = params;
}

static void addIcpEdge(
    const mrpt::poses::CPose3DPDFGaussian& relPose, size_t kfRef, size_t kfCur,
    gtsam::NonlinearFactorGraph& fg)
{
    using gtsam::symbol_shorthand::X;

    // auto priorNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);

    gtsam::Pose3   relPose3;
    gtsam::Matrix6 relPoseCov;
    mrpt::gtsam_wrappers::to_gtsam_se3_cov6(relPose, relPose3, relPoseCov);

    auto icpNoise = gtsam::noiseModel::Gaussian::Covariance(relPoseCov);
    gtsam::noiseModel::Base::shared_ptr icpRobNoise;

    double icp_edge_robust_param = 0.1;

    if (icp_edge_robust_param > 0)
        icpRobNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Fair::Create(icp_edge_robust_param),
            icpNoise);
    else
        icpRobNoise = icpNoise;

    fg.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        X(kfRef), X(kfCur), relPose3, icpRobNoise);

    // m_state.kf_connectivity_add(curKfId, *newOtherKfId);
}

struct ICP_Input
{
    using Ptr = std::shared_ptr<ICP_Input>;

    mola::id_t                  global_id{mola::INVALID_ID};
    mola::id_t                  local_id{mola::INVALID_ID};
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

static void run_one_icp(const ICP_Input& in, ICP_Output& out, ICP_case& icpCase)
{
    using namespace std::string_literals;

    MRPT_START

    ASSERT_(in.local_pc);
    ASSERT_(in.global_pc);
    const auto& pcs_local  = *in.local_pc;
    const auto& pcs_global = *in.global_pc;

    mrpt::math::TPose3D current_solution = in.init_guess_local_wrt_global;

    mp2p_icp::Results icp_result;

    icpCase.icp->align(
        pcs_local, pcs_global, current_solution, in.icp_params, icp_result);

    if (icp_result.quality > 0)
    {
        // Keep as init value for next stage:
        current_solution = icp_result.optimal_tf.mean.asTPose();
    }

    out.found_pose_to_wrt_from = icp_result.optimal_tf;
    out.goodness               = icp_result.quality;

#if 0
    MRPT_LOG_DEBUG_FMT(
        "ICP (kind=%u): goodness=%.03f iters=%u rel_pose=%s "
        "termReason=%u",
        static_cast<unsigned int>(in.align_kind), out.goodness,
        static_cast<unsigned int>(icp_result.nIterations),
        out.found_pose_to_wrt_from.getMeanVal().asString().c_str(),
        static_cast<unsigned int>(icp_result.terminationReason));
#endif

    MRPT_END
}

static int main_odometry()
{
    // Initialize:
    const auto file_yml = argYAML_ICP.getValue();
    const auto cfg      = mola::load_yaml_file(file_yml);

    // Select dataset input:
    auto dataset = std::make_shared<KittiSource>();
    dataset->init(argKittiSeq.getValue());

    const double tStart = mrpt::Clock::nowDouble();

    size_t nDatasetEntriesToRun = dataset->size();
    if (arg_firstN.isSet()) nDatasetEntriesToRun = arg_firstN.getValue();

    // load input path:
    mrpt::poses::CPose3DInterpolator inputPath;
    inputPath.loadFromTextFile_TUM(argInPath.getValue());

    std::cout << "Loaded input path: " << inputPath.size() << " from "
              << argInPath.getValue() << std::endl;

    // -------------------------------
    ICP_case icp_;
    load_icp_set_of_params(
        icp_, mrpt::containers::yaml::FromFile(argYAML_ICP.getValue()));

    auto pcFilter = mp2p_icp_filters::filter_pipeline_from_yaml_file(
        argYAML_Filter.getValue());

    mp2p_icp_filters::GeneratorSet obs_generators;
    {
        auto defaultGen = mp2p_icp_filters::Generator::Create();
        defaultGen->initialize({});
        obs_generators.push_back(defaultGen);
    }

    std::map<size_t, mp2p_icp::metric_map_t::Ptr> localMaps;
    // -------------------------------
    using gtsam::symbol_shorthand::X;

    gtsam::NonlinearFactorGraph fg;
    gtsam::Values               fgInitValues;
    // -------------------------------
    std::vector<mrpt::poses::CPose3D> pathPoses;
    std::vector<double>               pathTimes;
    pathPoses.reserve(nDatasetEntriesToRun);
    pathTimes.reserve(nDatasetEntriesToRun);
    for (size_t i = 0; i < nDatasetEntriesToRun; i++)
    {
        auto itPose = inputPath.begin();
        std::advance(itPose, i);
        pathPoses.emplace_back(itPose->second);
        pathTimes.emplace_back(mrpt::Clock::toDouble(itPose->first));
    }

    // ---------------------------------

    std::cout << "\n";  // Needed for the VT100 codes below.

    // Run:
    for (size_t i = 0; i < nDatasetEntriesToRun; i++)
    {
        const auto obs = dataset->getObservation(i);
        if (!obs) continue;

        // ------------------------------
        auto& lm = localMaps[i];

        lm = mp2p_icp::metric_map_t::Create();
        mp2p_icp_filters::apply_generators(obs_generators, *obs, *lm);
        mp2p_icp_filters::apply_filter_pipeline(pcFilter, *lm);

        // std::cout << lm->contents_summary() << "\n";

        fgInitValues.insert(
            X(i), mrpt::gtsam_wrappers::toPose3(pathPoses.at(i)));

        // 0) prior:
        if (i == 0)
        {
            auto priorNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1.0);
            fg.addPrior(
                X(i), mrpt::gtsam_wrappers::toPose3(pathPoses.at(i)),
                priorNoise);
        }

        // 1) Add current relative pose from model-based odom:
        if (i > 1)
        {
            const auto& pose_i    = pathPoses.at(i);
            const auto& pose_im1  = pathPoses.at(i - 1);
            const auto  deltaPose = pose_i - pose_im1;

            auto cov = mrpt::math::CMatrixDouble66::Identity();
            cov.setDiagonal(0.05);

            const auto p = mrpt::poses::CPose3DPDFGaussian(deltaPose, cov);

            addIcpEdge(p, i - 1, i, fg);
        }

        // 2) ICP arcs:

        const double SLIDING_WINDOW_MAX_DIST = argSlidingLen.getValue();

        if (i > 0)
        {
            for (int j = i - 1; j > 0; j -= 3)
            {
                //
                const auto& poseCur   = pathPoses.at(i);
                const auto& poseRef   = pathPoses.at(j);
                const auto  deltaPose = poseCur - poseRef;

                if (deltaPose.translation().norm() > SLIDING_WINDOW_MAX_DIST)
                    break;

                ICP_Output icp_out;
                ICP_Input  icp_in;

                icp_in.init_guess_local_wrt_global = deltaPose.asTPose();

                icp_in.local_pc = localMaps[i];
                icp_in.local_id = i;

                icp_in.global_pc = localMaps[j];
                icp_in.global_id = j;

                icp_in.debug_str = "sliding_window";

                icp_in.icp_params = icp_.icpParameters;

                // Run ICP:
                run_one_icp(icp_in, icp_out, icp_);

                if (icp_out.goodness > 0.50)
                {
                    addIcpEdge(icp_out.found_pose_to_wrt_from, j, i, fg);
                }
            }
        }

        // ------------------------------

        static int cnt = 0;
        if (cnt++ % 20 == 0)
        {
            cnt             = 0;
            const size_t N  = (dataset->size() - 1);
            const double pc = (1.0 * i) / N;

            const double tNow = mrpt::Clock::nowDouble();
            const double ETA  = pc > 0 ? (tNow - tStart) * (1.0 / pc - 1) : .0;
            const double totalTime = ETA + (tNow - tStart);

            std::cout
                << "\033[A\33[2KT\r"  // VT100 codes: cursor up and clear line
                << mrpt::system::progress(pc, 30)
                << mrpt::format(
                       " %6zu/%6zu (%.02f%%) ETA=%s / T=%s\n", i, N, 100 * pc,
                       mrpt::system::formatTimeInterval(ETA).c_str(),
                       mrpt::system::formatTimeInterval(totalTime).c_str());
            std::cout.flush();
        }
    }

#if 0
    fg.print();
    fgInitValues.print();
#endif

    if (0)
    {
        std::ofstream fDot("fg.dot");
        fDot << fg.dot();
    }

    // Optimize:
    auto lmParams          = gtsam::LevenbergMarquardtParams::LegacyDefaults();
    lmParams.maxIterations = 20;

    const auto nFactors = fg.size();

    lmParams.iterationHook = [nFactors](
                                 size_t iter, double errInit, double errFinal) {
        std::cout << "[LM] iter: " << iter
                  << " rmse: " << std::sqrt(errInit / nFactors) << " => "
                  << std::sqrt(errFinal / nFactors) << std::endl;
    };

    auto optimizer =
        gtsam::LevenbergMarquardtOptimizer(fg, fgInitValues, lmParams);

    const auto& optimValues = optimizer.optimize();

    // save solution:
    mrpt::poses::CPose3DInterpolator out;
    for (size_t i = 0; i < nDatasetEntriesToRun; i++)
    {
        using gtsam::symbol_shorthand::X;

        const double t = pathTimes.at(i);
        if (!optimValues.exists(X(i))) continue;

        const auto p = optimValues.at<gtsam::Pose3>(X(i));
        out.insert(
            mrpt::Clock::fromDouble(t), mrpt::gtsam_wrappers::toTPose3D(p));
    }

    std::cout << "Saving path to: " << argOutPath.getValue() << std::endl;
    out.saveToTextFile_TUM(argOutPath.getValue());

    return 0;
}

int main(int argc, char** argv)
{
    try
    {
        // Parse arguments:
        if (!cmd.parse(argc, argv)) return 1;  // should exit.

        main_odometry();

        return 0;
    }
    catch (std::exception& e)
    {
        mola::pretty_print_exception(e, "Exit due to exception:");
        return 1;
    }
}
