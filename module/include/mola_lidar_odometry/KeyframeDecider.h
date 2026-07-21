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
 * @file   KeyframeDecider.h
 * @brief  Shared "should a new keyframe be created here?" policy
 * @author Jose Luis Blanco Claraco
 */
#pragma once

#include <mola_pose_list/SearchablePoseList.h>
#include <mrpt/poses/CPose3D.h>

#include <cstdint>
#include <deque>
#include <utility>

namespace mola
{
/** Options of the keyframe-creation policy shared by the local metric map and
 *  the simplemap / central-mapper keyframe deciders.
 *
 *  Both `Parameters::MapUpdateOptions` and `Parameters::SimpleMapOptions`
 *  derive from this struct, so all fields keep being plain, non-nested YAML
 *  keys of their respective sections.
 *
 * \ingroup mola_lidar_odometry_grp
 */
struct KeyframeDecisionOptions
{
  /** Minimum Euclidean distance (x,y,z) between keyframes [meters]. */
  double min_translation_between_keyframes = 1.0;

  /** Minimum rotation (in 3D space, yaw, pitch, roll altogether) between
   *  keyframes [degrees]. */
  double min_rotation_between_keyframes = 30.0;

  /** If true, only the distance to the LAST keyframe is considered.
   *  Use if mostly mapping without "closed loops".
   *
   *  If false (default), a KD-tree is used to check the distance to *all*
   *  past keyframe poses. See also nearby_keyframe_time_window, which bounds
   *  how far back in time that search reaches.
   */
  bool measure_from_last_kf_only = false;

  /** Minimum number of stored poses within the threshold distance before a
   *  volume is considered "occupied" and no new keyframe is inserted there.
   *  Default=1 gives the classic behavior. Increase to 2+ for
   *  non-repetitive-scan lidars (e.g. Livox) so multiple scans are taken from
   *  each location.
   */
  uint32_t min_nearby_poses_occupied = 1;

  /** Time window [seconds] for the "is there already a keyframe here?" test.
   *  0 (default) disables it: the test then reaches ALL past keyframes.
   *
   *  With the default, purely spatial policy, revisiting a mapped area
   *  creates NO new keyframes, since the previous pass' keyframes are the
   *  nearest neighbors. That is the desired behavior for a local metric map
   *  (a revisit adds no coverage), but it starves loop closure: a loop can
   *  only be detected if BOTH of its endpoints exist as keyframes, so the
   *  revisit that should close the loop produces nothing to close it with.
   *
   *  When set to a positive value, only keyframes newer than this many
   *  seconds take part in the test (plus the most recent one, always, so a
   *  stationary vehicle does not emit one keyframe per window). Older
   *  keyframes become invisible to it, so a revisit spawns fresh keyframes.
   */
  double nearby_keyframe_time_window = 0;

  // Loaded by LidarOdometry::Parameters::load_keyframe_policy(), which owns
  // the dynamic-expression parameter pool the distance thresholds register
  // into. All fields live as plain (non-nested) keys of their YAML section.
};

/** Decides whether a new keyframe should be created at a given pose, and
 *  stores the poses of those already created.
 *
 *  One instance exists per "map" whose keyframe density is governed
 *  independently (the local metric map, and the simplemap / central-mapper
 *  keyframe backbone), each with its own KeyframeDecisionOptions.
 *
 *  The options are passed in at every check() call rather than held: they may
 *  be dynamic expressions (e.g. a distance that shrinks with angular
 *  velocity), re-evaluated by mp2p_icp::Parameterizable before each scan.
 *
 * \ingroup mola_lidar_odometry_grp
 */
class KeyframeDecider
{
public:
  KeyframeDecider() = default;

  explicit KeyframeDecider(bool measure_from_last_kf_only) : poses_(measure_from_last_kf_only) {}

  struct Decision
  {
    /** Whether a new keyframe should be created at the queried pose. */
    bool create = false;

    /** Translational distance to the closest existing keyframe [m]
     *  (0 if there is none yet). For logging/diagnostics. */
    double translation = 0;

    /** Rotational distance to the closest existing keyframe [rad]
     *  (0 if there is none yet). For logging/diagnostics. */
    double rotation = 0;
  };

  /** Evaluates the policy for a candidate keyframe pose. Does NOT modify the
   *  stored poses; call insert() if the keyframe is actually created.
   *  \param timestamp Observation timestamp [s], only used if
   *         `opts.nearby_keyframe_time_window` is enabled.
   */
  [[nodiscard]] Decision check(
    const KeyframeDecisionOptions & opts, const mrpt::poses::CPose3D & pose,
    double timestamp) const;

  /** Stores an actually-created keyframe pose. */
  void insert(const mrpt::poses::CPose3D & pose, double timestamp);

  /** Drops stored poses farther than the given distance (local map cleanup) */
  void removeAllFartherThan(const mrpt::poses::CPose3D & pose, double maxTranslation);

  [[nodiscard]] size_t size() const { return poses_.size(); }

  [[nodiscard]] bool empty() const { return poses_.empty(); }

private:
  /** All stored keyframe poses (KD-tree searchable). */
  SearchablePoseList poses_;

  /** Keyframes within the time window, oldest first. Only maintained (and
   *  only non-empty) while `nearby_keyframe_time_window` is enabled. The
   *  newest entry is never dropped, whatever its age. */
  mutable std::deque<std::pair<double, mrpt::poses::CPose3D>> recent_;

  /** Drops entries older than the window, always keeping the newest one. */
  void prune_recent(double now, double window) const;
};

}  // namespace mola

/** Feature macro: the keyframe-creation policy is shared between the local map
 *  and simplemap deciders (mola::KeyframeDecider) and supports a temporal
 *  window (`nearby_keyframe_time_window`).
 */
#define MOLA_LO_HAS_KEYFRAME_DECIDER 1
