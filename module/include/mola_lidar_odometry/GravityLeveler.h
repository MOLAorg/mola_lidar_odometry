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
 * @file   GravityLeveler.h
 * @brief  Causal gravity leveling of an output trajectory
 * @author Jose Luis Blanco Claraco
 */
#pragma once

#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/poses/CPose3D.h>

#include <Eigen/Dense>
#include <deque>
#include <optional>

namespace mola
{
/** Causal gravity leveling of a trajectory produced by an odometry or a state
 *  estimator, without touching the estimator itself.
 *
 *  The trajectory's frame may slowly tilt with respect to gravity (e.g. where
 *  the scans barely constrain pitch and roll), which turns horizontal motion
 *  into a vertical drift. This class estimates that tilt and rotates the
 *  trajectory increments to undo it:
 *
 *  - Measurement, every `epoch_period` seconds: the mean specific force over
 *    the trailing `window` seconds, rotated into the trajectory frame with the
 *    trajectory's own attitudes, minus the trajectory's own mean acceleration
 *    over the same window. Normalized, its horizontal part is the frame tilt
 *    `tau` (2 DoF, small angles), plus the effect of any body-fixed
 *    accelerometer error.
 *  - A Kalman filter with state `[tau (2), b (3)]`: `tau` a random walk with
 *    `tilt_random_walk` deg/sqrt(s), `b` a constant body-frame accelerometer
 *    error (bias plus IMU misalignment). A body-fixed error turns with the
 *    platform and a frame tilt does not, which separates both while the
 *    platform turns. Readings that disagree with the prediction beyond a
 *    chi-squared gate are ignored.
 *  - Output: the part of the estimated tilt beyond `deadband` degrees is
 *    removed by rotating every increment of the trajectory, and its attitude,
 *    by the small rotation that levels the frame. The deadband leaves alone a
 *    frame that is already level within the reading's own error.
 *
 *  Only past data are used. Positions are corrected incrementally, so the
 *  output is continuous even when the correction changes.
 */
class GravityLeveler
{
public:
  struct Parameters
  {
    /** Trailing window of the gravity reading [s] */
    double window = 10.0;
    /** Period between filter updates [s] */
    double epoch_period = 1.0;
    /** Random walk of the frame tilt [deg/sqrt(s)] */
    double tilt_random_walk = 0.1;
    /** Noise of each tilt reading [deg] */
    double reading_sigma = 0.15;
    /** Tilt left uncorrected [deg] */
    double deadband = 2.0;
    /** Prior sigma of the initial tilt [deg] */
    double initial_tilt_sigma = 3.0;
    /** Prior sigma of the body-frame accelerometer error [m/s^2] */
    double initial_acc_error_sigma = 0.3;
    /** Time span for the trajectory's own velocity [s] */
    double velocity_span = 1.0;
    /** Minimum number of IMU readings in a window to take a reading */
    size_t min_readings_per_window = 10;
    /** Standard gravity [m/s^2] */
    double gravity = 9.81;
  };

  GravityLeveler() = default;
  explicit GravityLeveler(const Parameters & p) : params_(p) {}

  const Parameters & parameters() const { return params_; }

  /** Adds one accelerometer reading (specific force, in the vehicle frame) */
  void add_imu(double t, const mrpt::math::TVector3D & specific_force);

  /** Adds the next pose of the trajectory (timestamps must increase) and
   *  returns it leveled. IMU readings up to `t` must have been added before.
   */
  mrpt::poses::CPose3D add_pose(double t, const mrpt::poses::CPose3D & pose);

  /** The current tilt estimate (x, y components, in radians), before the
   *  deadband. Empty until the first reading has been taken. */
  std::optional<mrpt::math::TVector2D> tilt() const;

  /** The tilt being removed from the output (after the deadband) [rad] */
  mrpt::math::TVector2D applied_tilt() const { return applied_tilt_; }

private:
  struct PoseSample
  {
    double t = 0;
    mrpt::math::TPoint3D p;
    mrpt::math::TVector3D v;
  };

  struct ImuSample
  {
    double t = 0;
    mrpt::math::TVector3D f;
  };

  Parameters params_;

  std::deque<ImuSample> pending_imu_;  // vehicle frame, after the last pose
  std::deque<ImuSample> window_imu_;   // trajectory frame
  std::deque<PoseSample> poses_;

  std::optional<double> t0_;
  std::optional<double> next_epoch_;
  std::optional<double> last_epoch_;
  bool filter_started_ = false;

  Eigen::Matrix<double, 5, 1> x_ = Eigen::Matrix<double, 5, 1>::Zero();
  Eigen::Matrix<double, 5, 5> P_ = Eigen::Matrix<double, 5, 5>::Zero();

  mrpt::math::TVector2D applied_tilt_{0, 0};

  std::optional<mrpt::poses::CPose3D> last_in_;
  std::optional<mrpt::poses::CPose3D> last_out_;

  void update_filter(double t, const mrpt::poses::CPose3D & pose);
};

}  // namespace mola

/** Feature macro: mola_lidar_odometry provides mola::GravityLeveler */
#define MOLA_LO_HAS_GRAVITY_LEVELER 1
