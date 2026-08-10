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
   *  how far back in time that search reaches. Combining both is allowed but
   *  redundant: the last keyframe is always inside the window, so the window
   *  has no additional effect.
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
   *
   *  Read once, at KeyframeDecider construction time: unlike the two distance
   *  thresholds, this one cannot be a per-scan dynamic expression, since it
   *  selects which of the two policies (and which storage) the decider uses.
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
 *  The two distance thresholds are passed in at every check() call rather than
 *  held: they may be dynamic expressions (e.g. a distance that shrinks with
 *  angular velocity), re-evaluated by mp2p_icp::Parameterizable before each
 *  scan. The two policy *selectors* (`measure_from_last_kf_only` and
 *  `nearby_keyframe_time_window`) are instead read once, by the constructor,
 *  and pick which of the two mutually exclusive storages is maintained.
 *
 * \ingroup mola_lidar_odometry_grp
 */
class KeyframeDecider
{
public:
  /** There is no default constructor on purpose: the options select which
   *  policy, and which storage, the instance uses for its whole lifetime. */
  explicit KeyframeDecider(const KeyframeDecisionOptions & opts)
  : poses_(opts.measure_from_last_kf_only),
    from_last_only_(opts.measure_from_last_kf_only),
    temporal_(opts.nearby_keyframe_time_window > 0)
  {
  }

  struct Decision
  {
    /** Whether a new keyframe should be created at the queried pose. */
    bool create = false;

    /** Translational distance to the reference keyframe [m]: the closest one
     *  under the spatial policy, the newest one under the temporal policy
     *  (0 if there is none yet). For logging/diagnostics. */
    double translation = 0;

    /** Rotational distance to that same reference keyframe [rad]
     *  (0 if there is none yet). For logging/diagnostics. */
    double rotation = 0;
  };

  /** Evaluates the policy for a candidate keyframe pose. Does NOT store the
   *  queried pose; call insert() if the keyframe is actually created (it does
   *  drop stored poses that fell out of the time window, though).
   *  \param timestamp Observation timestamp [s], only used under the temporal
   *         policy.
   */
  [[nodiscard]] Decision check(
    const KeyframeDecisionOptions & opts, const mrpt::poses::CPose3D & pose, double timestamp);

  /** Stores an actually-created keyframe pose. */
  void insert(const mrpt::poses::CPose3D & pose, double timestamp);

  /** Drops stored poses farther than the given distance (local map cleanup) */
  void removeAllFartherThan(const mrpt::poses::CPose3D & pose, double maxTranslation);

  /** Re-expresses every stored keyframe pose in a new reference frame, i.e.
   *  each stored `p` becomes `b + p`. Relative distances are preserved, so no
   *  past decision changes; only the frame the poses live in moves.
   *
   *  Needs SearchablePoseList::transform_left_multiply(), so it is compiled
   *  out against an older mola_pose_list. Callers must guard on
   *  MOLA_POSE_LIST_HAS_TRANSFORM_LEFT_MULTIPLY. */
#if defined(MOLA_POSE_LIST_HAS_TRANSFORM_LEFT_MULTIPLY)
  void transform_left_multiply(const mrpt::poses::CPose3D & b);
#endif

  /** Number of keyframes created so far (minus those dropped by
   *  removeAllFartherThan). This is what the GUI reports: under the temporal
   *  policy, keyframes that aged out of the window still count, since they
   *  remain in the map and merely stopped taking part in the decision. */
  [[nodiscard]] size_t size() const { return temporal_ ? created_ : poses_.size(); }

  [[nodiscard]] bool empty() const { return size() == 0; }

  /** Number of keyframes currently taking part in the decision, i.e. those
   *  inside the time window under the temporal policy. Equals size() under the
   *  spatial one. */
  [[nodiscard]] size_t activeSize() const { return temporal_ ? recent_.size() : poses_.size(); }

private:
  using Entry = std::pair<double /*timestamp*/, mrpt::poses::CPose3D>;

  /** Keyframe poses under the spatial policy (KD-tree searchable).
   *  Left empty, and never queried, while `temporal_` is set. */
  SearchablePoseList poses_;

  /** Keyframe poses under the temporal policy, oldest first. Left empty, and
   *  never queried, unless `temporal_` is set. The newest entry is never
   *  dropped, whatever its age. */
  std::deque<Entry> recent_;

  bool from_last_only_ = false;

  /** Keyframes created, tracked only under the temporal policy (under the
   *  spatial one `poses_` already holds them all). See size(). */
  size_t created_ = 0;

  /** Whether the temporal policy (and hence `recent_`) is the active one.
   *  Fixed at construction: see nearby_keyframe_time_window. */
  bool temporal_ = false;

  /** Drops entries older than the window, always keeping the newest one. */
  void prune_recent(double now, double window);
};

}  // namespace mola

/** Feature macro: the keyframe-creation policy is shared between the local map
 *  and simplemap deciders (mola::KeyframeDecider) and supports a temporal
 *  window (`nearby_keyframe_time_window`).
 */
#define MOLA_LO_HAS_KEYFRAME_DECIDER 1
