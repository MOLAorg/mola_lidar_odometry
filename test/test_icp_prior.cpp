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

#include <gtest/gtest.h>
#include <mola_lidar_odometry/IcpPrior.h>
#include <mp2p_icp/covariance.h>

#include <Eigen/Dense>

namespace
{
mrpt::poses::CPose3DPDFGaussianInf tiltedMotionModelPose()
{
  mrpt::poses::CPose3DPDFGaussianInf p;
  p.mean = mrpt::poses::CPose3D::FromXYZYawPitchRoll(3.0, -1.0, 0.4, 2.1, 0.12, -0.2);
  // Anisotropic and correlated, as a motion model gives it:
  Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
  const double sigmas[6] = {0.05, 0.3, 0.02, 0.01, 0.002, 0.004};
  for (int i = 0; i < 6; i++) {
    A(i, i) = sigmas[i];
  }
  A(0, 1) = 0.02;
  A(3, 5) = 0.001;
  const Eigen::Matrix<double, 6, 6> cov = A * A.transpose();
  p.cov_inv.asEigen() = cov.inverse();
  return p;
}

double relativeError(const Eigen::Matrix<double, 6, 6> & a, const Eigen::Matrix<double, 6, 6> & b)
{
  return (a - b).norm() / b.norm();
}
}  // namespace

TEST(IcpPrior, ConvertedAtItsMean)
{
  const auto mm = tiltedMotionModelPose();
  const auto prior = mola::icp_prior_from_motion_model(mm);

  EXPECT_EQ(prior.mean, mm.mean);
#if defined(MP2P_ICP_HAS_COVARIANCE_YPR_CONVERSIONS)
  const auto expected = mp2p_icp::information_ypr_to_tangent(mm.cov_inv, mm.mean);
  EXPECT_LT(relativeError(prior.cov_inv.asEigen(), expected.asEigen()), 1e-12);
#else
  EXPECT_EQ(prior.cov_inv, mm.cov_inv);
#endif
}

// Moving the mean, as the SE(2) and gravity paths do, gives the motion model
// information converted at the NEW mean, not at the one it was built at.
TEST(IcpPrior, MovingTheMeanReconvertsTheInformation)
{
  const auto mm = tiltedMotionModelPose();
  auto prior = mola::icp_prior_from_motion_model(mm);

  auto newMean = mm.mean;
  newMean.setYawPitchRoll(newMean.yaw(), 0.03, 0.05);
  mola::set_icp_prior_mean(prior, newMean);

  EXPECT_EQ(prior.mean, newMean);
#if defined(MP2P_ICP_HAS_COVARIANCE_YPR_CONVERSIONS)
  const auto expected = mp2p_icp::information_ypr_to_tangent(mm.cov_inv, newMean);
  EXPECT_LT(relativeError(prior.cov_inv.asEigen(), expected.asEigen()), 1e-9);

  // ...which is measurably different from keeping the old one:
  const auto stale = mp2p_icp::information_ypr_to_tangent(mm.cov_inv, mm.mean);
  EXPECT_GT(relativeError(stale.asEigen(), expected.asEigen()), 1e-3);
#else
  EXPECT_EQ(prior.cov_inv, mm.cov_inv);
#endif
}
