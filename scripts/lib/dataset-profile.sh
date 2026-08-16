# Shared plumbing for the mola-lo-gui-* / mola-lo-cli-* dataset wrappers.
# Sourced, never executed.
#
# A "dataset profile" (lib/profiles/<name>.sh) describes ONE public dataset:
# how it is laid out on disk, what its topics and frames are called, and which
# pipeline knobs it needs. It exports that description as environment
# variables and does not invoke anything.
#
# The point is that a single description then feeds every consumer:
#   - mola-lo-gui-<name>   online replay through mola-cli + a launch YAML,
#   - mola-lo-cli-<name>   offline batch through mola-lidar-odometry-cli,
#   - out-of-tree harnesses (batch sweeps in eval/, the CI regression job)
#     which source the profile directly instead of restating the dataset.
#
# Both binaries read the same MOLA_* variables: the launch YAMLs always did,
# and mola-lidar-odometry-cli's topic/frame options carry matching envname()
# fallbacks, so nothing has to translate env vars into flags.
#
# ---------------------------------------------------------------------------
# Profile contract
# ---------------------------------------------------------------------------
# A profile file defines two functions:
#
#   mola_lo_profile_usage       Prints the dataset-specific usage message.
#
#   mola_lo_profile_resolve     Receives the wrapper's arguments. Consumes the
#                               ones naming the dataset, exports the MOLA_*
#                               description, and sets:
#                                 MOLA_LO_LAUNCH_FILE  launch YAML basename
#                                 MOLA_LO_CLI_INPUT    array of offline input
#                                                      flags, e.g.
#                                                      (--input-rosbag1 a,b)
#                                 MOLA_LO_EXTRA_ARGS   array of leftover args
#                                                      to forward verbatim
#
# Two variables are set by the caller before the profile runs, and profiles
# may branch on them:
#
#   MOLA_LO_MODE                "gui" or "cli". A few datasets legitimately
#                               differ: a camera bag is worth replaying for a
#                               GUI preview and is dead weight in a batch run.
#
# Every value a profile publishes uses the ': "${VAR:=default}"' idiom, so a
# caller that exported the variable first always wins. That is what lets the
# CI keep its per-dataset overrides without forking the profile.
#
# A profile may also pick the odometry pipeline itself, by publishing
# MOLA_ODOMETRY_PIPELINE_YAML the same way. Do that only where a dataset has a
# measured reason to differ from the default, and state the evidence in the
# profile: see profiles/kitti.sh.

# Absolute path of this library's directory, resolved through symlinks.
MOLA_LO_LIB_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)

# Where mola-cli-launchs/, pipelines/ and state-estimator-params/ live. This
# library sits in scripts/lib/ in a source checkout and in
# share/mola_lidar_odometry/scripts-lib/ once installed, so those data
# directories are one level up in the installed case and two in the source
# one. Every wrapper used to re-derive this; it lives here now.
mola_lo_share_dir() {
  if [ -d "$MOLA_LO_LIB_DIR/../mola-cli-launchs" ]; then
    (cd -- "$MOLA_LO_LIB_DIR/.." && pwd)
  else
    (cd -- "$MOLA_LO_LIB_DIR/../.." && pwd)
  fi
}

MOLA_LO_SHARE_DIR=$(mola_lo_share_dir)
MOLA_LO_LAUNCHS_DIR="$MOLA_LO_SHARE_DIR/mola-cli-launchs"
MOLA_LO_PIPELINES_DIR="$MOLA_LO_SHARE_DIR/pipelines"

# Fills the four MOLA_INPUT_ROSBAG1* slots the rosbag1 launch files expose
# from the bag list given as arguments, and sets MOLA_LO_BAGS_JOINED to the
# same list comma-joined, which is the spelling the offline CLI's
# --input-rosbag1 takes. Empty arguments are dropped; unused slots are set to
# the empty string rather than left alone, so a stale value exported by an
# earlier run cannot leak in as an extra bag.
#
# The joined list is returned through a variable, not echoed: a caller
# capturing it with $(...) would run this in a subshell and lose the exports.
mola_lo_bag_slots() {
  local -a bags=()
  local b
  for b in "$@"; do
    [ -n "$b" ] && bags+=("$b")
  done

  local max_slots=4
  if [ "${#bags[@]}" -gt "$max_slots" ]; then
    echo "Error: ${#bags[@]} bags given, the rosbag1 launch files expose $max_slots slots." >&2
    return 1
  fi

  local i
  for ((i = 0; i < max_slots; i++)); do
    local var="MOLA_INPUT_ROSBAG1"
    [ "$i" -gt 0 ] && var="MOLA_INPUT_ROSBAG1_$((i + 1))"
    printf -v "$var" '%s' "${bags[$i]:-}"
    export "${var?}"
  done

  local IFS=,
  MOLA_LO_BAGS_JOINED="${bags[*]}"
}

# Selects the mola_state_estimation_smoother instead of the default simple
# estimator, resolving its params YAML through `ros2 pkg prefix` since it
# lives in another package's install prefix. Several datasets want this.
#
# A caller that runs a method with its own internal estimator (the DLIO and
# Fast-LIO2 wrappers), or that has already chosen an estimator of its own,
# sets MOLA_LO_SKIP_STATE_ESTIMATOR=1 and gets the rest of the profile
# without this.
mola_lo_use_smoother() {
  if [ "${MOLA_LO_SKIP_STATE_ESTIMATOR:-0}" = "1" ]; then
    return 0
  fi

  if [ -n "${MOLA_STATE_ESTIMATOR_YAML:-}" ]; then
    : "${MOLA_STATE_ESTIMATOR:=mola::state_estimation_smoother::StateEstimationSmoother}"
    export MOLA_STATE_ESTIMATOR
    return 0
  fi

  local prefix
  prefix=$(ros2 pkg prefix mola_state_estimation_smoother 2>/dev/null)
  local yaml="$prefix/share/mola_state_estimation_smoother/params/state-estimation-smoother.yaml"
  if [ -z "$prefix" ] || [ ! -f "$yaml" ]; then
    echo "Error: package 'mola_state_estimation_smoother' was not found. Please install it," >&2
    echo "       or export MOLA_STATE_ESTIMATOR_YAML to use a different estimator." >&2
    return 1
  fi

  : "${MOLA_STATE_ESTIMATOR:=mola::state_estimation_smoother::StateEstimationSmoother}"
  MOLA_STATE_ESTIMATOR_YAML="$yaml"
  export MOLA_STATE_ESTIMATOR MOLA_STATE_ESTIMATOR_YAML
}

# Sources a profile and runs it. $1 is the profile name, the rest are the
# wrapper's own arguments. Returns non-zero on any failure rather than
# exiting: callers other than the wrappers (a CI harness, say) need to report
# the failure their own way before the process ends.
mola_lo_load_profile() {
  local name=$1
  shift

  local profile="$MOLA_LO_LIB_DIR/profiles/$name.sh"
  if [ ! -f "$profile" ]; then
    echo "Error: no dataset profile '$name' (looked in '$MOLA_LO_LIB_DIR/profiles/')." >&2
    return 1
  fi

  MOLA_LO_LAUNCH_FILE=""
  MOLA_LO_CLI_INPUT=()
  MOLA_LO_EXTRA_ARGS=()
  # Profiles for datasets that publish a single sequence set this: there is
  # nothing to name on the command line, so an empty invocation is valid.
  MOLA_LO_PROFILE_NO_ARGS=0

  # shellcheck source=/dev/null
  source "$profile"

  if [ "$#" -eq 0 ] && [ "$MOLA_LO_PROFILE_NO_ARGS" -eq 0 ]; then
    mola_lo_profile_usage
    return 1
  fi

  mola_lo_profile_resolve "$@"
}

# Both exec helpers below default the pipeline YAML the same way every
# wrapper used to, honoring PIPELINE_YAML for backwards compatibility.
mola_lo_pipeline_yaml() {
  echo "${MOLA_ODOMETRY_PIPELINE_YAML:-${PIPELINE_YAML:-$MOLA_LO_PIPELINES_DIR/lidar3d-default.yaml}}"
}

# Online: mola-cli + the profile's launch YAML.
mola_lo_exec_gui() {
  local launch="$MOLA_LO_LAUNCHS_DIR/$MOLA_LO_LAUNCH_FILE"
  if [ ! -f "$launch" ]; then
    echo "Error: launch file not found: '$launch'" >&2
    exit 1
  fi

  MOLA_ODOMETRY_PIPELINE_YAML=$(mola_lo_pipeline_yaml)
  export MOLA_ODOMETRY_PIPELINE_YAML

  exec mola-cli "$launch" "${MOLA_LO_EXTRA_ARGS[@]}" "$@"
}

# Offline: mola-lidar-odometry-cli + the profile's input flags. Topics and
# frames are NOT passed as flags: the binary picks them up from the same
# MOLA_* variables the profile already exported.
mola_lo_exec_cli() {
  local -a args=(-c "$(mola_lo_pipeline_yaml)")

  # --state-estimator-param-file is required by the binary. Profiles that
  # call mola_lo_use_smoother have already pointed it at the smoother.
  local se_yaml=${MOLA_STATE_ESTIMATOR_YAML:-}
  if [ -z "$se_yaml" ]; then
    se_yaml="$MOLA_LO_SHARE_DIR/state-estimator-params/state-estimation-simple.yaml"
  fi
  args+=(--state-estimator-param-file "$se_yaml")
  if [ -n "${MOLA_STATE_ESTIMATOR:-}" ]; then
    args+=(--state-estimator "$MOLA_STATE_ESTIMATOR")
  fi

  args+=("${MOLA_LO_CLI_INPUT[@]}")

  exec mola-lidar-odometry-cli "${args[@]}" "${MOLA_LO_EXTRA_ARGS[@]}" "$@"
}
