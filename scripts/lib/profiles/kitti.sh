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

  # KITTI does better on the point-to-point pipeline than on the default
  # cov-to-cov one, and by enough to be worth selecting here: measured over
  # sequences 00-10 it is better on 8 of 11 in translation and 7 of 11 in
  # rotation, and costs about half the time per scan.
  #
  # This is a per-dataset choice, not a claim about the pipelines in general.
  # A forward ablation between the two found that the difference does not come
  # from any single stage: intermediate configurations are several times worse
  # than either pipeline, so the two are separate optima rather than points on
  # a path. Swapping just the matcher, in particular, is far worse than either.
  #
  # Batch and regression runs override this the same way they override the
  # velocity prior above, so their published numbers stay comparable.
  : "${MOLA_ODOMETRY_PIPELINE_YAML:=$MOLA_LO_PIPELINES_DIR/lidar3d-icp.yaml}"
  export MOLA_ODOMETRY_PIPELINE_YAML

  MOLA_LO_LAUNCH_FILE=lidar_odometry_from_kitti.yaml
  MOLA_LO_CLI_INPUT=(--input-kitti-seq "$seq")
}
