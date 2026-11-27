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
 * @file   WallUpReader.h
 * @brief  Up direction of a trajectory frame from the plumb walls around it
 * @author Jose Luis Blanco Claraco
 */
#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/poses/CPose3D.h>

#include <cstddef>
#include <deque>
#include <optional>
#include <vector>

namespace mola
{
/** Reads the up direction of a trajectory frame from the structure seen
 *  around the trajectory: walls are plumb and floors and ceilings level, so
 *  the direction that makes them so, in the trajectory frame, is that frame's
 *  view of the true vertical. Meant as the input of
 *  GravityLeveler::add_up_reading().
 *
 *  Feed it the scans as they are registered (add_cloud(): the de-skewed
 *  points in the vehicle frame and the vehicle pose in the trajectory frame).
 *  add_cloud() only keeps the pointer, so it is cheap where the scans come
 *  from (e.g. the odometry's worker); the scans are transformed, cut to
 *  `max_range` and voxel-decimated in read(). Every `period` seconds, read()
 *  estimates the up direction from the scans of the last `window` seconds
 *  with mp2p_icp_filters::estimate_planar_normals() and
 *  estimate_up_from_normals(), with the covariance of that estimate.
 *
 *  No reading comes from a window from which up is not observable (too few
 *  planar points, a single wall direction and no floor), whose covariance
 *  rests on too few groups of normals to be trusted, whose walls do not
 *  confirm its floors (`max_walls_floors_angle`: sloped ground, leaning walls,
 *  or floors along a single facade), or that holds no scan newer than the
 *  previous reading (e.g. standing still). Consecutive windows
 *  overlap by about window/period scans, so each reading's covariance is
 *  inflated by that factor: a filter fed with all of them does not count
 *  the shared scans several times.
 *
 *  Memory is bounded by the window: scans older than it are dropped. It
 *  never changes the scans it is given.
 *
 *  Not thread-safe: add_cloud() and read() must not run concurrently.
 */
class WallUpReader
{
public:
  struct Parameters
  {
    /** Scans older than this are not used [s]. Short: a trailing window
     *  makes the reading lag the frame's wander by about half of it. */
    double window = 5.0;
    /** Time between readings [s], > 0 */
    double period = 1.0;
    /** Voxel size of the decimation of each scan and of the window [m] */
    double voxel = 0.10;
    /** Points farther than this from the vehicle are not used [m] */
    double max_range = 20.0;
    /** Neighborhood radius of each local plane fit [m] */
    double normal_radius = 0.4;
    /** Weight of floors and ceilings against walls (0: walls only) */
    double flats_weight = 1.0;
    /** Cells grouping the normals for the covariance [m], > 0 */
    double cluster_size = 2.0;
    /** A window with fewer points than this gives no reading */
    std::size_t min_points = 2000;
    /** No reading from a window whose walls do not confirm its floors [deg]:
     *  - the walls alone and the floors alone are more than this apart on up,
     *    so they contradict each other (floors on a slope, walls that lean);
     *  - or the walls alone do not give up (one wall direction, e.g. along a
     *    single facade, or open ground with few walls), so its floors cannot
     *    be told from a slope.
     *  0: no check. Ignored, with a warning from the CLI, where
     *  checks_walls_against_floors() is false. */
    double max_walls_floors_angle = 2.0;
  };

  struct Reading
  {
    double t = 0;
    /** Unit up vector, in the trajectory frame */
    mrpt::math::TVector3D up{0, 0, 1};
    /** Covariance of up.x and up.y [rad^2] */
    mrpt::math::CMatrixDouble22 up_cov;
    std::size_t points = 0;
    std::size_t walls = 0;
    std::size_t flats = 0;
    /** Wall-clock time spent in this reading [s] */
    double cost = 0;
  };

  struct Stats
  {
    std::size_t clouds = 0;
    std::size_t readings = 0;
    std::size_t unobservable = 0;
    std::size_t too_few_points = 0;
    std::size_t unreliable_cov = 0;
    /** Windows whose walls contradict their floors (max_walls_floors_angle) */
    std::size_t floors_contradicted = 0;
    /** Windows with floors whose walls alone do not give up */
    std::size_t floors_unconfirmed = 0;
    /** Periods without a scan newer than the previous reading */
    std::size_t no_new_scans = 0;
    /** Wall-clock time spent in read() [s] */
    double cost_total = 0;
    double cost_max = 0;
  };

  /** With the default parameters, checked like any others (and like them,
   *  \throw std::exception without the up covariance API of mp2p_icp) */
  WallUpReader() : WallUpReader(Parameters{}) {}
  /** \throw std::exception on invalid parameters */
  explicit WallUpReader(const Parameters & p);

  const Parameters & parameters() const { return params_; }
  const Stats & stats() const { return stats_; }

  /** Whether this build checks walls against floors (max_walls_floors_angle):
   *  false if built against an mp2p_icp without that API
   *  (MP2P_ICP_HAS_UP_REFERENCES_CHECK), where no window is dropped for it. */
  static bool checks_walls_against_floors();

  /** Adds a scan at time `t`: its points in the vehicle frame, and the
   *  vehicle pose in the trajectory frame. Only the pointer is kept: the
   *  cloud must not change afterwards. */
  void add_cloud(
    double t, const mrpt::poses::CPose3D & vehicle_pose,
    const mrpt::maps::CPointsMap::ConstPtr & cloud);

  /** A reading at time `t`, if `period` has passed since the previous one and
   *  the window allows one */
  std::optional<Reading> read(double t);

private:
  struct Batch
  {
    double t = 0;
    mrpt::poses::CPose3D pose;
    mrpt::maps::CPointsMap::ConstPtr cloud;  // until decimated
    std::vector<mrpt::math::TPoint3Df> pts;  // trajectory frame, decimated
  };

  void decimate(Batch & b) const;

  Parameters params_;
  Stats stats_;
  std::deque<Batch> batches_;
  std::optional<double> last_read_;
  bool new_since_read_ = false;
};

}  // namespace mola
