# KITTI odometry benchmark (https://www.cvlibs.net/datasets/kitti/).
#
# The sequence is read by mola_input_kitti_dataset from KITTI_BASE_DIR, so
# there are no topics or bags involved: only a two-digit sequence number.
#
# Note that the published ground truth is expressed in the CAMERA frame, not
# the Velodyne one. Scoring an estimate against it without correcting for that
# is mostly absorbed by APE's alignment and catastrophic in RPE.

mola_lo_profile_usage() {
  echo "Error: A KITTI sequence number is required."
  echo "Usage: $0 <KITTI_SEQ> [additional flags]"
  echo "With <KITTI_SEQ> 00 or 01 or 02..."
  echo ""
  echo "The dataset location is taken from \$KITTI_BASE_DIR."
}

mola_lo_profile_resolve() {
  local seq=$1
  shift
  MOLA_LO_EXTRA_ARGS=("$@")

  KITTI_SEQ="$seq"
  export KITTI_SEQ

  # A car on a road: the very first scans are matched better with a forward-
  # velocity prior than from a standstill assumption.
  #
  # This default is NOT free, and is not the right choice everywhere. Earlier
  # work found a large value necessary on seq 12 (which starts already moving
  # fast) but artifact-inducing on the rest of the corpus. It is kept as the
  # interactive default because that is the case it was tuned for; batch and
  # regression runs deliberately override it -- eval/cli_kitti.sh pins 18.0
  # to stay comparable with its own published history, and the server-side
  # regression corpus pins 0.0. Do not "unify" these to one number: they are
  # different trade-offs, not an oversight.
  : "${MOLA_INITIAL_VX:=20.0}"
  export MOLA_INITIAL_VX

  # No pipeline override: KITTI runs the shipped default, like every other
  # dataset here.
  #
  # This profile used to select the point-to-point pipeline
  # (lidar3d-icp.yaml), on a measurement over 00-10 that it wins the KITTI
  # devkit's relative metric on 8 of 11 sequences. That result still
  # reproduces -- relative translation error is 3.8 % lower on average, better
  # on 8 of 11 -- but it prices only local registration. Scored on absolute
  # trajectory error over the same 11 sequences, the default cov-to-cov
  # pipeline wins 9 of 11, with a mean of 1.65 m against 2.72 m and a median
  # of 1.07 m against 1.93 m. Three sequences are not close: 06 is 0.45 m
  # against 4.78 m, 01 is 2.02 against 4.09, and 09 is 1.07 against 1.93.
  #
  # Good local registration that drifts is what that pattern describes, and
  # ATE is the metric this dataset's users compare. Note the two are not in
  # conflict about anything subtle: the relative gain is a few percent and the
  # absolute cost is 65 %.
  #
  # The decimation tuning below was always measured against this pipeline, so
  # the two now agree instead of being tuned for different targets.

  # Decimation tuning for that pipeline. A coarser voxel visited completely
  # via the stride bound beats the shipped fine voxel sampled at a stride of
  # 3-5, on both translation and rotation, at a third of the cost, on the
  # full 00-10 corpus. Per-dataset, not a shipped pipeline default: it was
  # only ever validated on KITTI and Oxford Spires -- see the Oxford Spires
  # profile for its own copy of these, and lio/03_accuracy_pipeline.md for
  # why BotanicGarden/citrus-farm/ConSLAM must not inherit it untested.
  : "${MOLA_CLOUD_DECIMATION_VOXEL_SIZE_MAP:=0.45}"
  : "${MOLA_CLOUD_DECIMATION_VOXEL_SIZE_ICP:=0.45}"
  : "${MOLA_VOXEL_STRIDE_MAP:=1}"
  : "${MOLA_VOXEL_STRIDE_ICP:=2}"
  : "${MOLA_LOCALMAP_K_CORRESPONDENCES_FOR_COV:=10}"
  export MOLA_CLOUD_DECIMATION_VOXEL_SIZE_MAP MOLA_CLOUD_DECIMATION_VOXEL_SIZE_ICP
  export MOLA_VOXEL_STRIDE_MAP MOLA_VOXEL_STRIDE_ICP MOLA_LOCALMAP_K_CORRESPONDENCES_FOR_COV

  # Local map for the GICP pipeline: the radius-bounded incremental cloud
  # instead of the default 3-keyframe map, plus the map-side planarity gate.
  #
  # Measured over 00-10 with the KITTI devkit metric (the leaderboard one),
  # against a baseline reproducing the published regression corpus exactly:
  #
  #                       translation %   rotation deg/100 m
  #   shipped keyframe map      0.608           0.176
  #   + planarity gate 0.20     0.577           0.175
  #   incremental map + gate    0.571           0.1215
  #   KISS-ICP, for reference   0.502           0.148
  #
  # The rotation number is the point: 0.1215 is 18% BETTER than KISS-ICP,
  # from 19% worse. The gain comes from map extent -- the keyframe map only
  # ever queried its 3 nearest keyframes, so the retention radius could not
  # reach a pairing.
  #
  # PER-DATASET ON PURPOSE, and it must not be promoted to a pipeline default:
  #  - On Oxford Spires the same map class is a consistent HORIZONTAL win
  #    (0.67-0.79x on 4 of 4 sites) but an unpredictable VERTICAL cost, up to
  #    2.9x APE on bodleian-library-02. Nothing yet controls that.
  #  - IncrementalPointCloud is ODOMETRY ONLY: a global SE(3) re-map forces a
  #    full rebuild, so do not combine it with loop closure or the SLAM
  #    map-building entry points. Override MOLA_LOCALMAP_CLASS back to
  #    mola::KeyframePointCloudMap for those.
  #
  # The gate needs mola_metric_maps with max_plane_deviation_for_cov
  # (MOLAorg/mola#200); on an older core the key is ignored and this reverts to
  # the previous behavior rather than failing.
  : "${MOLA_LOCALMAP_CLASS:=mola::IncrementalPointCloud}"
  : "${MOLA_LOCALMAP_MAX_PLANE_DEV_FOR_COV:=0.20}"
  export MOLA_LOCALMAP_CLASS MOLA_LOCALMAP_MAX_PLANE_DEV_FOR_COV

  # Smoother-only relative-pose prior floor (no-op for `simple`), raised from
  # the class default of 0. On a well-conditioned road corpus the unfloored
  # smoother's prior collapses to near-zero velocity between scans and the
  # trajectory comes out 5x short; the floor alone fixes it, back to parity
  # with `simple` (1.01x) on all 11 sequences. Per-dataset, not a shipped
  # default: the families where the smoother earns its keep by pinning ICP
  # (uneven or degenerate geometry) lose this same floor back. Oxford Spires
  # carries its own copy of these values for the same reason.
  : "${MOLA_SMOOTHER_SIGMA_REL_POSE_LIN:=0.1}"
  : "${MOLA_SMOOTHER_SIGMA_REL_POSE_ANG:=0.02}"
  export MOLA_SMOOTHER_SIGMA_REL_POSE_LIN MOLA_SMOOTHER_SIGMA_REL_POSE_ANG

  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_kitti.yaml
  MOLA_LO_CLI_INPUT=(--input-kitti-seq "$seq")
}
