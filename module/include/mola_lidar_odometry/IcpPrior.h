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
 * @file   IcpPrior.h
 * @brief  The pose prior handed to the ICP solver, in the solver's convention
 * @author Jose Luis Blanco Claraco
 */

#pragma once

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>

namespace mola
{
/** \defgroup mola_lo_icp_prior ICP pose prior
 *
 *  The ICP solver applies the information matrix of its pose prior to
 *  `log(P_prior^-1 * P_cur)`, i.e. in the SE(3) tangent of the prior mean:
 *  translation in the frame of the mean, then rotation-vector components. A
 *  motion model gives its pose in the `[x y z yaw pitch roll]` layout of
 *  CPose3DPDFGaussianInf instead, and converting between both depends on the
 *  mean (mp2p_icp::information_ypr_to_tangent()). These keep the information
 *  in the tangent of the mean it is applied at.
 *
 *  Against an mp2p_icp without `MP2P_ICP_HAS_COVARIANCE_YPR_CONVERSIONS`, whose
 *  Censi3D covariances reached the motion model already in that tangent, they
 *  leave the information unchanged, as before.
 *  @{ */

/** The ICP prior for a motion model pose given in the `[x y z yaw pitch roll]`
 *  layout: same mean, information converted to the tangent of the mean. */
mrpt::poses::CPose3DPDFGaussianInf icp_prior_from_motion_model(
  const mrpt::poses::CPose3DPDFGaussianInf & motionModelPose);

/** Moves the mean of an ICP prior, re-expressing its information in the
 *  tangent of the new mean. For a prior built with
 *  icp_prior_from_motion_model(), the result is the motion model information
 *  converted at the new mean. */
void set_icp_prior_mean(
  mrpt::poses::CPose3DPDFGaussianInf & prior, const mrpt::poses::CPose3D & newMean);

/** @} */

}  // namespace mola
