#!/usr/bin/env bash
# Reproducibly stage the read-only Hector checkout in a writable Catkin overlay.
# shellcheck disable=SC1090
set -euo pipefail

canonical_source="${ASTRA_HECTOR_SOURCE:-/home/yanzu/rl_reference/controllers/hector-quadrotor-noetic}"
overlay="${ASTRA_HECTOR_OVERLAY:-/tmp/astra_hector_training_overlay}"
check_only=false

while (($#)); do
  case "$1" in
    --source)
      shift
      canonical_source="${1:-}"
      ;;
    --overlay)
      shift
      overlay="${1:-}"
      ;;
    --check-only)
      check_only=true
      ;;
    --help)
      echo "prepare_hector_training_overlay.sh [--source PATH] [--overlay PATH] [--check-only]"
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
  shift
done

generated_xacro_rel="hector_quadrotor/hector_quadrotor_gazebo/urdf/quadrotor_plugins.gazebo.xacro"
template_xacro_rel="${generated_xacro_rel}.in"
required_source_paths=(
  hector_gazebo
  hector_localization
  hector_models
  hector_quadrotor
  "$template_xacro_rel"
  hector_quadrotor/hector_quadrotor_gazebo/package.xml
  hector_quadrotor/hector_quadrotor_description/urdf/quadrotor.gazebo.xacro
)

validate_canonical_source() {
  local missing=0
  local relative_path
  for relative_path in "${required_source_paths[@]}"; do
    if [[ ! -e "$canonical_source/$relative_path" ]]; then
      echo "canonical Hector source is missing: $canonical_source/$relative_path" >&2
      missing=1
    fi
  done
  return "$missing"
}

validate_overlay() {
  local candidate="$1"
  local missing=0
  local source_entry
  for source_entry in hector_gazebo hector_localization hector_models hector_quadrotor; do
    if [[ ! -d "$candidate/src/$source_entry" ]]; then
      echo "Hector overlay source is missing: $candidate/src/$source_entry" >&2
      missing=1
    elif [[ -L "$candidate/src/$source_entry" ]]; then
      echo "Hector overlay source must be a writable staged copy, not a symlink: $candidate/src/$source_entry" >&2
      missing=1
    fi
  done
  if [[ ! -f "$candidate/src/$generated_xacro_rel" ]]; then
    echo "generated Hector xacro is missing: $candidate/src/$generated_xacro_rel" >&2
    missing=1
  fi
  if [[ ! -f "$candidate/devel/setup.bash" ]]; then
    echo "Hector overlay setup is missing: $candidate/devel/setup.bash" >&2
    missing=1
  fi
  if [[ ! -f "$candidate/devel/.catkin" ]]; then
    echo "Hector overlay Catkin marker is missing: $candidate/devel/.catkin" >&2
    missing=1
  elif [[ "$(cat "$candidate/devel/.catkin")" != "$candidate/src" ]]; then
    echo "Hector overlay setup contains a stale source prefix: $candidate/devel/.catkin" >&2
    missing=1
  fi
  if [[ ! -f "$candidate/build/Makefile" ]]; then
    echo "Hector overlay configure/build is incomplete: $candidate/build/Makefile" >&2
    missing=1
  fi
  return "$missing"
}

validate_canonical_source

if validate_overlay "$overlay"; then
  echo "Hector training overlay preflight PASS: $overlay"
  exit 0
fi

if "$check_only"; then
  echo "Hector training overlay preflight FAIL: $overlay" >&2
  exit 2
fi

overlay_parent="$(dirname "$overlay")"
mkdir -p "$overlay_parent"
invalid_backup=""
restore_previous_overlay() {
  local status=$?
  trap - EXIT INT TERM
  if [[ -n "$invalid_backup" && -e "$invalid_backup" ]]; then
    if [[ -e "$overlay" ]]; then
      failed_overlay="${overlay}.failed_$(date +%Y%m%d_%H%M%S)_$$"
      mv -- "$overlay" "$failed_overlay"
      echo "preserved failed Hector overlay build at: $failed_overlay" >&2
    fi
    mv -- "$invalid_backup" "$overlay"
    echo "restored previous Hector overlay after staging failure: $overlay" >&2
  fi
  exit "$status"
}

if [[ -e "$overlay" || -L "$overlay" ]]; then
  invalid_backup="${overlay}.invalid_$(date +%Y%m%d_%H%M%S)_$$"
  mv -- "$overlay" "$invalid_backup"
  echo "preserved invalid Hector overlay at: $invalid_backup"
fi
trap restore_previous_overlay EXIT INT TERM

mkdir -p "$overlay/src"
cp -a "$canonical_source/." "$overlay/src/"

source /opt/ros/noetic/setup.bash
(
  cd "$overlay"
  catkin_make -j2
)

validate_overlay "$overlay"
trap - EXIT INT TERM

echo "Hector training overlay generated from canonical source: $canonical_source"
echo "Hector training overlay preflight PASS: $overlay"
