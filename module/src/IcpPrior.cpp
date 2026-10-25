/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   IcpPrior.cpp
 * @brief  The pose prior handed to the ICP solver, in the solver's convention
 * @author Jose Luis Blanco Claraco
 */

#include <mola_lidar_odometry/IcpPrior.h>
#include <mp2p_icp/covariance.h>

#include <Eigen/Dense>

namespace mola
{
mrpt::poses::CPose3DPDFGaussianInf icp_prior_from_motion_model(
  const mrpt::poses::CPose3DPDFGaussianInf & motionModelPose)
{
  auto prior = motionModelPose;
#if defined(MP2P_ICP_HAS_COVARIANCE_YPR_CONVERSIONS)
  prior.cov_inv = mp2p_icp::information_ypr_to_tangent(prior.cov_inv, prior.mean);
#endif
  return prior;
}

void set_icp_prior_mean(
  mrpt::poses::CPose3DPDFGaussianInf & prior, const mrpt::poses::CPose3D & newMean)
{
#if defined(MP2P_ICP_HAS_COVARIANCE_YPR_CONVERSIONS)
  // Back to [x y z yaw pitch roll] through the old mean (info = J^T * infoYpr * J),
  // then into the tangent of the new one:
  const Eigen::Matrix<double, 6, 6> JoldInv =
    mp2p_icp::jacobian_ypr_wrt_tangent(prior.mean).asEigen().inverse();
  mrpt::math::CMatrixDouble66 infoYpr;
  infoYpr.asEigen() = JoldInv.transpose() * prior.cov_inv.asEigen() * JoldInv;
  prior.cov_inv = mp2p_icp::information_ypr_to_tangent(infoYpr, newMean);
#endif
  prior.mean = newMean;
}

}  // namespace mola
