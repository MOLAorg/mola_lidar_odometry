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
 * @file   LidarOdometry_LearnedCovariance.cpp
 * @brief  Optional learned covariance of each registration
 */

#include <mola_lidar_odometry/LidarOdometry.h>

#if defined(MOLA_LO_HAS_MP2P_LEARNED_COVARIANCE)

#include <mp2p_icp/covariance.h>

#include <Eigen/LU>

using namespace mola;

namespace
{
mrpt::maps::CPointsMap::Ptr scan_layer_of(
  const mp2p_icp::metric_map_t & observation, const std::string & layer)
{
  if (observation.layers.count(layer) == 0) {
    return {};
  }
  auto pts = observation.point_layer(layer);
  if (!pts || pts->empty()) {
    return {};
  }
  return pts;
}
}  // namespace

std::optional<std::shared_future<mrpt::math::CMatrixDouble66>>
LidarOdometry::submitLearnedCovariance(
  const mp2p_icp::metric_map_t & observation, const ICP_Input & in, const mp2p_icp::Results & res,
  const mrpt::poses::CPose3DPDFGaussian & pose)
{
  const auto scan = scan_layer_of(observation, params_.learned_covariance.scan_layer);
  if (!scan) {
    MRPT_LOG_THROTTLE_WARN_FMT(
      5.0, "learned_covariance: the observation has no layer '%s'; keeping the solver's covariance",
      params_.learned_covariance.scan_layer.c_str());
    return {};
  }

  mp2p_icp::LearnedCovarianceEstimator::Request req;
  req.scan = scan;
  req.solution = pose.mean;
  req.context.initialGuess = mrpt::poses::CPose3D(in.init_guess_local_wrt_global);
  if (in.prior) {
    // The prior may carry no information along some axes; keep it invertible.
    const Eigen::Matrix<double, 6, 6> info =
      in.prior->cov_inv.asEigen() + 1e-6 * Eigen::Matrix<double, 6, 6>::Identity();
    req.context.priorCovariance = info.inverse();
    req.context.priorInSolver = true;
  }
  req.context.iterations = static_cast<double>(res.nIterations);
  req.context.converged = res.terminationReason == mp2p_icp::IterTermReason::Stalled;
  req.context.quality = res.quality;

  const auto learned = state_.learned_cov->submit(std::move(req));

  // Run by whoever waits for the covariance, in its thread: the estimator's
  // tangent-space result in MRPT's [x y z yaw pitch roll] convention, or the
  // solver's own covariance if there is no learned one.
  const mrpt::poses::CPose3D est = pose.mean;
  const mrpt::math::CMatrixDouble66 solverCov = pose.cov;
  return std::async(
           std::launch::deferred,
           [learned, est, solverCov]() {
             const auto r = learned.get();
             if (!r.ok) {
               return solverCov;
             }
             return mp2p_icp::covariance_tangent_to_ypr(mrpt::math::CMatrixDouble66(r.Sigma), est);
           })
    .share();
}

void LidarOdometry::pushLearnedCovarianceWindow(
  const mp2p_icp::metric_map_t & observation, const mrpt::poses::CPose3D & pose)
{
  if (auto scan = scan_layer_of(observation, params_.learned_covariance.scan_layer); scan) {
    state_.learned_cov->pushToWindow(std::move(scan), pose);
  }
}

#endif
