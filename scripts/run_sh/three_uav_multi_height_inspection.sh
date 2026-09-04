#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
multi_layer_enabled=false
forwarded=()
for argument in "$@"; do
  case "$argument" in
    --multi-layer) multi_layer_enabled=true ;;
    --single-layer) multi_layer_enabled=false ;;
    --learning-speed)
      echo "the multi-height mission profile forbids Learning Speed/dynamic speed" >&2
      exit 2
      ;;
    --help)
      echo "three_uav_multi_height_inspection.sh [--single-layer|--multi-layer] [three_uav_inspection options]"
      echo "Default: distinct top heights with one layer; --multi-layer enables 34->30, 28->24 and 22->18 m."
      forwarded+=("$argument")
      ;;
    *) forwarded+=("$argument") ;;
  esac
done

export ASTRA_THREE_UAV_LAUNCH=triple_tower_multi_height_inspection.launch
export ASTRA_MULTI_LAYER_ENABLED="$multi_layer_enabled"
exec "$script_dir/three_uav_inspection.sh" "${forwarded[@]}"
