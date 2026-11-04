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
  ASSERTMSG_(
    observation.layers.count(layer) != 0, "learned_covariance: the observation has no layer '" +
                                            layer + "'; the observation pipeline must keep it");
  return observation.point_layer(layer);
}
}  // namespace

void LidarOdometry::applyLearnedCovariance(
  const mp2p_icp::metric_map_t & observation, const ICP_Input & in, const mp2p_icp::Results & res,
  mrpt::poses::CPose3DPDFGaussian & pose)
{
  const auto & lc = params_.learned_covariance;
  if (state_.learned_cov_window.empty()) {
    return;  // no reference yet: keep the solver's covariance
  }
  const auto scan = scan_layer_of(observation, lc.scan_layer);
  if (!scan || scan->empty()) {
    return;
  }

  auto & ev = *state_.learned_cov_evaluator;
  if (state_.learned_cov_reference_dirty) {
    ev.finalizeReference();
    state_.learned_cov_reference_dirty = false;
  }
  ev.setScan(*scan);

  mp2p_icp::CovFeatureContext ctx;
  ctx.initialGuess = mrpt::poses::CPose3D(in.init_guess_local_wrt_global);
  if (in.prior) {
    // The prior may carry no information along some axes; keep it invertible.
    const Eigen::Matrix<double, 6, 6> info =
      in.prior->cov_inv.asEigen() + 1e-6 * Eigen::Matrix<double, 6, 6>::Identity();
    ctx.priorCovariance = info.inverse();
    ctx.priorInSolver = true;
  }
  ctx.iterations = static_cast<double>(res.nIterations);
  ctx.converged = res.terminationReason == mp2p_icp::IterTermReason::Stalled;
  ctx.quality = res.quality;

  const mrpt::poses::CPose3D est = pose.mean;
  const auto r = ev.evaluate(est, ctx);
  if (!r.ok) {
    MRPT_LOG_THROTTLE_WARN(
      5.0, "learned_covariance: features undefined for this registration, keeping the solver's");
    return;
  }
  const auto pred = learned_cov_model_.predict(r, lc.context);
  state_.last_learned_cov_p_fail = pred.pFail;
  pose.cov = mp2p_icp::covariance_tangent_to_ypr(mrpt::math::CMatrixDouble66(pred.Sigma), est);

  MRPT_LOG_DEBUG_FMT(
    "learned_covariance: sigmas {%e %e %e [m] %e %e %e [deg]} p_fail=%.3f",
    std::sqrt(pred.Sigma(0, 0)), std::sqrt(pred.Sigma(1, 1)), std::sqrt(pred.Sigma(2, 2)),
    mrpt::RAD2DEG(std::sqrt(pred.Sigma(3, 3))), mrpt::RAD2DEG(std::sqrt(pred.Sigma(4, 4))),
    mrpt::RAD2DEG(std::sqrt(pred.Sigma(5, 5))), pred.pFail);
}

void LidarOdometry::pushLearnedCovarianceWindow(
  const mp2p_icp::metric_map_t & observation, const mrpt::poses::CPose3D & pose)
{
  const auto & lc = params_.learned_covariance;
  const auto scan = scan_layer_of(observation, lc.scan_layer);
  if (!scan || scan->empty()) {
    return;
  }
  if (!state_.learned_cov_evaluator) {
    state_.learned_cov_evaluator = std::make_shared<mp2p_icp::CovFeatureEvaluator>();
  }
  // The window slides one scan per step: update the evaluator's reference
  // incrementally instead of rebuilding it from every scan in the window.
  auto & ev = *state_.learned_cov_evaluator;
  state_.learned_cov_window.emplace_back(scan, pose);
  ev.addReferenceCloud(*scan, pose);
  while (state_.learned_cov_window.size() > lc.window_scans) {
    const auto & [oldScan, oldPose] = state_.learned_cov_window.front();
    ev.removeReferenceCloud(*oldScan, oldPose);
    state_.learned_cov_window.pop_front();
  }
  state_.learned_cov_reference_dirty = true;
}

#endif
