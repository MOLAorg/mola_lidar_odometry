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
 * @file   GravityMapAligner.h
 * @brief  Per-KF accelerometer pool + robust gravity estimation (pitch/roll only)
 *
 * Design reference: TODOs/online-gravity-alignment.md §2.4 and §3.1.
 * This class is pure: it owns a per-KF pool of averaged body-frame
 * accelerations, and exposes robust primitives to estimate per-KF
 * pitch/roll corrections sending the local gravity estimate to +Z.
 * It does NOT know about the local-map class or any ROS/TF state.
 */
#pragma once

#include <mola_imu_preintegration/LocalVelocityBuffer.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/poses/CPose3D.h>

#include <cstdint>
#include <map>
#include <optional>

namespace mola
{
/** Per-keyframe accelerometer pool + robust gravity-direction estimator.
 *
 * Samples are keyed by an external KeyFrameID (opaque `uint64_t`): the
 * KeyframePointCloudMap id when used from LIO, or any integer in unit tests.
 *
 * Thread-safety: none. Callers serialize access externally.
 *
 * \ingroup mola_lidar_odometry_grp
 */
class GravityMapAligner
{
public:
  using KeyFrameID = uint64_t;

  /// Default numerical guard for axis-angle degeneracies in
  /// rotationFromGravity / Params::axis_eps.
  static constexpr double kDefaultAxisEps = 1e-9;

  struct Params
  {
    /// Don't produce any estimate below this pool size.
    std::size_t min_keyframes = 10;

    /// Huber threshold on |R_i·a_i − ĝ|, in m/s².
    double huber_threshold_mps2 = 0.5;

    /// KFs with intra-interval accel std above this are heavily down-weighted
    /// (reject highly dynamic motion). In m/s².
    double max_kf_acc_std_mps2 = 1.5;

    /// Number of IRLS iterations.
    std::size_t irls_iterations = 3;

    /// Expected gravity magnitude in m/s² (used only for initial weighting).
    double expected_g_mps2 = 9.81;

    /// Numerical guard for axis-angle near-zero/180°.
    double axis_eps = kDefaultAxisEps;
  };

  /// One averaged sample per KF.
  struct KFAccSample
  {
    mrpt::math::TVector3D a_body{0, 0, 0};  //!< Time-averaged body-frame accel (m/s²).
    double a_body_std = 0;                  //!< Intra-KF std of |a|, m/s².
    std::size_t n_samples = 0;              //!< Number of raw samples averaged.
  };

  GravityMapAligner() = default;
  explicit GravityMapAligner(Params p) : params_(std::move(p)) {}

  const Params & params() const { return params_; }
  void setParams(Params p) { params_ = std::move(p); }

  /// Register a KF and the accelerometer samples taken during its interval.
  /// Computes and stores, from the samples in `window`, the time-averaged
  /// body-frame acceleration `a_body` and the intra-KF std of |a|.
  /// If `window` contains no accelerometer samples (e.g., empty), the KF is
  /// NOT added to the pool.
  void onNewKeyframe(KeyFrameID id, const mola::imu::LocalVelocityBuffer::SamplesByTime & window);

  /// Insert a pre-computed sample directly (useful for tests / rehydration).
  void onNewKeyframeRaw(KeyFrameID id, const KFAccSample & s);

  /// Drop per-KF data when a KF is evicted from the local map.
  void onKeyframeDropped(KeyFrameID id);

  /// Reset all state.
  void clear();

  std::size_t size() const { return samples_.size(); }
  bool has(KeyFrameID id) const { return samples_.count(id) > 0; }

  const std::map<KeyFrameID, KFAccSample> & samples() const { return samples_; }

  /** Robust weighted mean of the gravity direction in the "map" frame, pooled
   *  over all currently-stored KFs whose ids are also present in `kf_poses`.
   *
   *  @returns  The estimated gravity vector (not normalized) in the `kf_poses`
   *            frame, or nullopt if pool size is below params_.min_keyframes.
   */
  std::optional<mrpt::math::TVector3D> estimateGravityVector(
    const std::map<KeyFrameID, mrpt::poses::CPose3D> & kf_poses) const;

  /** Robust weighted mean of gravity, pooled over the subset of ids in
   *  `id_window` that are both present in this aligner and in `kf_poses`.
   *  Returns nullopt if fewer than `min_pool` KFs in the intersection (or
   *  if <2 samples, which would make the IRLS ill-conditioned).
   */
  std::optional<mrpt::math::TVector3D> estimateGravityVectorOverWindow(
    const std::map<KeyFrameID, mrpt::poses::CPose3D> & kf_poses,
    const std::vector<KeyFrameID> & id_window, std::size_t min_pool) const;

  /** Builds the pitch+roll-only rotation that sends the unit vector `g_hat`
   *  to +Z. Yaw is left at zero (unobservable with gravity only).
   *  Returns identity if `g_hat` is already aligned with +Z within axis_eps.
   */
  static mrpt::poses::CPose3D rotationFromGravity(
    const mrpt::math::TVector3D & g_hat, double axis_eps = kDefaultAxisEps);

  /** One-shot pool-wide correction: pitch+roll rotation sending the robust
   *  pool mean of `R_i · a_i` (in the `kf_poses` frame) to +Z.
   *  nullopt if pool is below trigger size.
   */
  std::optional<mrpt::poses::CPose3D> estimateGlobalCorrection(
    const std::map<KeyFrameID, mrpt::poses::CPose3D> & kf_poses) const;

  /** For each KF id in `kf_poses` that also has pool samples, compute a
   *  per-KF pitch+roll rotation `R_grav_i` (sending the windowed robust mean
   *  of `R_j · a_j` for j in `Window(i)` to +Z). `window_half_width` is `P`
   *  in `Window(i) = [i-P, i+P]` (truncated at ends).
   *  @param min_pool_per_kf   Minimum number of pool members in Window(i)
   *                           required to emit R_grav_i (else KF is skipped).
   *  @returns map from KF id to R_grav_i.
   */
  std::map<KeyFrameID, mrpt::poses::CPose3D> estimatePerKFCorrection(
    const std::map<KeyFrameID, mrpt::poses::CPose3D> & kf_poses, std::size_t window_half_width,
    std::size_t min_pool_per_kf) const;

private:
  Params params_;
  std::map<KeyFrameID, KFAccSample> samples_;
};

}  // namespace mola
