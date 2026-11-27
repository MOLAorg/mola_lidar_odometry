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

#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/poses/CPose3D.h>

#include <Eigen/Dense>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <unordered_map>
#include <vector>

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
 *  - Measurement, every `epoch_period` seconds: the mean specific force over a
 *    trailing window, each reading rotated into the trajectory frame with the
 *    trajectory's attitude at its own time, minus the trajectory's own mean
 *    acceleration over the same interval. Normalized, its horizontal part is
 *    the frame tilt `tau` (2 DoF, small angles), plus the effect of any
 *    body-fixed accelerometer error.
 *  - Readings beyond `max_tilt`, or with gravity not pointing up, are
 *    ignored: a frame tilt is small, and a large reading means the
 *    trajectory's attitude itself is failing.
 *  - A Kalman filter with state `[tau (2), b (3)]`: `tau` a random walk with
 *    `tilt_random_walk` deg/sqrt(s), `b` a constant body-frame accelerometer
 *    error (bias plus IMU misalignment), observed through the mean attitude
 *    over the window. A body-fixed error turns with the platform and a frame
 *    tilt does not, which separates both while the platform turns. Readings
 *    beyond a chi-squared gate are ignored; after
 *    `max_consecutive_rejections` of them in a row the filter restarts from
 *    the current reading, so a wrong state cannot lock itself in.
 *  - Optionally, readings of the up direction measured from plumb structure
 *    (walls, columns, floors) seen in the trajectory frame, add_up_reading().
 *    They observe the tilt directly, with no body-frame term, and with the
 *    covariance of their own estimate (plus `up_reading_sigma_floor` for what
 *    it cannot see, e.g. walls a little out of plumb), behind their own
 *    chi-squared gate. Their plausibility limit is `max_tilt`, and up to
 *    `max_up_tilt` where the accelerometer also sees the frame beyond
 *    `max_tilt`.
 *  - Output: the part of the estimated tilt beyond a deadband is removed by
 *    rotating every increment of the trajectory, and its attitude, by the
 *    small rotation that levels the frame. The deadband leaves alone a frame
 *    that is already level within the readings' own error: `deadband`
 *    degrees for the accelerometer alone, `up_reading_deadband` while an up
 *    reading has been accepted within the last `up_reading_hold` seconds,
 *    since those readings are an order of magnitude more precise and a
 *    2-degree deadband would hide the wander they measure. For the same
 *    reason the tilt's random walk is then `up_reading_random_walk`. After
 *    the hold, the deadband grows to `deadband` over `up_reading_ramp`
 *    seconds, so the correction fades instead of dropping at once.
 *  - Up readings that come with their place, and that the filter accepted,
 *    are also kept by place: a pose within `up_reading_place_radius`
 *    (horizontally, and `up_reading_place_max_dz` vertically) of some
 *    of them is leveled with those readings instead of with the filter's
 *    state, fading into the state toward their edge. The frame's tilt
 *    depends on where the odometry is in its map, so a revisited place gets
 *    its own readings back, not the last one taken elsewhere, and a trailing
 *    window is applied where it was measured instead of a few meters later.
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
    /** Tilt left uncorrected while up readings are recent [deg]. 0: their
     *  own covariance already weighs them, a deadband only biases. */
    double up_reading_deadband = 0.0;
    /** Random walk of the frame tilt while up readings are recent: precise
     *  readings let the filter follow a faster wander [deg/sqrt(s)] */
    double up_reading_random_walk = 0.3;
    /** An up reading keeps `up_reading_deadband` in force this long [s] */
    double up_reading_hold = 30.0;
    /** After the hold, time for the deadband to grow back to `deadband` [s] */
    double up_reading_ramp = 30.0;
    /** Added in quadrature to the sigmas of each up reading [deg], > 0 */
    double up_reading_sigma_floor = 0.1;
    /** A pose within this horizontal distance [m] of accepted up readings
     *  that came with their place, and on the same floor (see
     *  `up_reading_place_max_dz`), is leveled with them, 0: by time only.
     *  Their mean is weighted by their covariance and by a Gaussian of the
     *  horizontal distance (sigma = radius/2, lowered to zero at the radius),
     *  with `up_reading_deadband`.
     *  Where those Gaussians sum to less than 0.5 (1 is a reading right at
     *  the pose), the mean and that deadband are blended in proportion with
     *  the filter's state and its deadband, so the correction fades into the
     *  state toward the edge of the readings instead of jumping at the
     *  radius. While poses are leveled by place alone, `up_reading_deadband`
     *  stays in force as if a reading had just been accepted (see
     *  `up_reading_hold`). */
    double up_reading_place_radius = 5.0;
    /** Readings more than this above or below a pose [m] are another place:
     *  another floor of the map carries its own tilt error. A ramp of 10 %
     *  rises 0.5 m within the radius. > 0 */
    double up_reading_place_max_dz = 1.5;
    /** Accelerometer readings beyond this tilt are ignored: they mean a
     *  failing attitude of the trajectory, not a tilt of its frame [deg] */
    double max_tilt = 5.0;
    /** Up readings beyond this tilt are ignored [deg], in (0, 90). They are
     *  measured on the structure, so they can read a frame tilted beyond
     *  `max_tilt`, but see `max_up_tilt_needs_accelerometer`. */
    double max_up_tilt = 15.0;
    /** If true, up readings beyond `max_tilt` are taken (up to `max_up_tilt`)
     *  only while the latest accelerometer reading also sees the frame beyond
     *  `max_tilt` (with gravity of a plausible magnitude, pointing up). Before
     *  that, and without an accelerometer, they are held to
     *  `max_tilt`: a frame the accelerometer sees nearly level is not tilted
     *  by 10 deg, and such a reading comes from structure that is not plumb
     *  or level (rock faces, slopes). Set it to false to take them up to
     *  `max_up_tilt` with no accelerometer. */
    bool max_up_tilt_needs_accelerometer = true;
    /** Prior sigma of the initial tilt [deg], > 0 */
    double initial_tilt_sigma = 3.0;
    /** Prior sigma of the body-frame accelerometer error [m/s^2] */
    double initial_acc_error_sigma = 0.3;
    /** Time span for the trajectory's own velocity [s] */
    double velocity_span = 1.0;
    /** Minimum number of IMU readings in a window to take a reading */
    std::size_t min_readings_per_window = 10;
    /** Longest time without IMU readings within a window to take a reading [s] */
    double max_imu_gap = 0.5;
    /** Consecutive gated readings after which the filter restarts */
    std::size_t max_consecutive_rejections = 5;
    /** Standard gravity [m/s^2] */
    double gravity = 9.81;
  };

  /** Counters, for diagnostics */
  struct Stats
  {
    std::size_t imu_readings = 0;
    std::size_t tilt_readings = 0;
    std::size_t gated_readings = 0;
    std::size_t implausible_readings = 0;
    std::size_t restarts = 0;
    std::size_t up_readings = 0;
    std::size_t gated_up_readings = 0;
    std::size_t implausible_up_readings = 0;
    std::size_t up_restarts = 0;
    /** Readings with an innovation covariance that cannot be inverted */
    std::size_t singular_readings = 0;
    /** Poses leveled with the up readings of their place */
    std::size_t place_leveled_poses = 0;
  };

  GravityLeveler() = default;
  /** \throw std::exception on invalid parameters */
  explicit GravityLeveler(const Parameters & p);

  const Parameters & parameters() const { return params_; }
  const Stats & stats() const { return stats_; }

  /** Adds one accelerometer reading (specific force, in the vehicle frame) */
  void add_imu(double t, const mrpt::math::TVector3D & specific_force);

  /** Adds a reading of the up direction of the trajectory frame at time `t`,
   *  e.g. from the walls and floors seen around the trajectory, with the
   *  covariance of its x and y components [rad^2]. It updates the filter at
   *  once, so it should come before the poses newer than `t`. `place`, if
   *  given, is where in the trajectory frame the reading was measured (e.g.
   *  the mean position of the scans behind it), see `up_reading_place_radius`.
   */
  void add_up_reading(
    double t, const mrpt::math::TVector3D & up, const mrpt::math::CMatrixDouble22 & up_cov,
    const std::optional<mrpt::math::TPoint3D> & place = std::nullopt);

  /** Adds the next pose of the trajectory and returns it leveled. IMU
   *  readings up to `t` should have been added before. A pose not newer than
   *  the previous one is leveled with the current correction but does not
   *  update the filter.
   */
  mrpt::poses::CPose3D add_pose(double t, const mrpt::poses::CPose3D & pose);

  /** The current tilt estimate, before the deadband: the x/z and y/z
   *  components of the frame's up direction (tangents; the tilt angle is
   *  atan(|tilt|)). Empty until the first reading has been taken. */
  std::optional<mrpt::math::TVector2D> tilt() const;

  /** Covariance of tilt() (tangents, approximately [rad^2] for small tilts).
   *  Empty until the first reading. */
  std::optional<mrpt::math::CMatrixDouble22> tilt_cov() const;

  /** The tilt being removed from the output (after the deadband), as tilt()
   *  (tangents) */
  mrpt::math::TVector2D applied_tilt() const { return applied_tilt_; }

private:
  struct PoseSample
  {
    double t = 0;
    mrpt::math::TPoint3D p;
    std::optional<mrpt::math::TVector3D> v;
    double v_t = 0;  // midpoint of the interval the velocity spans
  };

  struct ImuSample
  {
    double t = 0;
    mrpt::math::TVector3D f;  // vehicle frame, then trajectory frame
    mrpt::math::CMatrixDouble33 R;
  };

  Parameters params_;
  Stats stats_;

  std::deque<ImuSample> pending_imu_;  // vehicle frame, after the last pose
  std::deque<ImuSample> window_imu_;   // trajectory frame
  std::deque<PoseSample> poses_;

  std::optional<double> t0_;
  std::optional<double> next_epoch_;
  std::optional<double> last_epoch_;
  bool filter_started_ = false;
  std::size_t consecutive_rejections_ = 0;
  std::size_t consecutive_up_rejections_ = 0;
  std::optional<double> last_up_accepted_t_;
  /** Whether the latest accelerometer reading saw the frame beyond max_tilt */
  bool acc_beyond_max_tilt_ = false;

  Eigen::Matrix<double, 5, 1> x_ = Eigen::Matrix<double, 5, 1>::Zero();
  Eigen::Matrix<double, 5, 5> P_ = Eigen::Matrix<double, 5, 5>::Zero();

  mrpt::math::TVector2D applied_tilt_{0, 0};

  /** Up readings kept by place, in a grid of cells of the place radius */
  struct PlacedReading
  {
    mrpt::math::TPoint3D p;
    Eigen::Vector2d tau = Eigen::Vector2d::Zero();
    double weight = 0;
  };
  std::vector<PlacedReading> placed_;
  std::unordered_map<std::uint64_t, std::vector<std::size_t>> place_grid_;

  /** The latest pose leveled by place: it holds the deadband too */
  std::optional<double> last_place_leveled_t_;

  struct PlaceTilt
  {
    Eigen::Vector2d tau = Eigen::Vector2d::Zero();
    /** Sum of the Gaussian weights of the readings, by distance only */
    double kernel = 0;
  };
  /** The tilt read near `p`, if there are readings within the radius */
  std::optional<PlaceTilt> tilt_at_place(const mrpt::math::TPoint3D & p) const;

  /** Keeps an accepted up reading (tilt `tau`, covariance `R`) by place */
  void keep_by_place(
    const std::optional<mrpt::math::TPoint3D> & place, const Eigen::Vector2d & tau,
    const Eigen::Matrix2d & R);

  std::optional<double> last_t_;
  std::optional<mrpt::poses::CPose3D> last_in_;
  std::optional<mrpt::poses::CPose3D> last_out_;

  void update_filter(double t);
  void reset_filter();

  /** The deadband in force at time `t` [rad] */
  double deadband_at(double t) const;

  /** Starts the filter if needed, propagates it to `t` and returns whether
   *  this is its first reading */
  bool predict_to(double t);

  /** Kalman update with a tilt reading `z = H x + noise(R)`; false if gated */
  bool correct(
    const Eigen::Vector2d & z, const Eigen::Matrix<double, 2, 5> & H, const Eigen::Matrix2d & R,
    bool starting, std::size_t & consecutive_rejections, std::size_t & gated);
};

}  // namespace mola

/** Feature macro: mola_lidar_odometry provides mola::GravityLeveler */
#define MOLA_LO_HAS_GRAVITY_LEVELER 1
