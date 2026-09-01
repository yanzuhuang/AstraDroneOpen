#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash

forest_v1_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
forest_v1_build_dir="${TMPDIR:-/tmp}/astra_forest_randomization_v1_build"
forest_v1_ros_dir="${TMPDIR:-/tmp}/astra_forest_randomization_v1_ros"
forest_v1_output_dir="${FOREST_OUTPUT_DIR:-${forest_v1_dir}}"
forest_v1_base_world="${forest_v1_dir}/learning_speed_forest_v1_base.world"
forest_v1_config="${forest_v1_dir}/forest_pool_v1.json"
forest_v1_overwrite="${FOREST_OVERWRITE:-0}"
forest_v1_ros_master_uri="${FOREST_ROS_MASTER_URI:-http://127.0.0.1:11321}"
forest_v1_gazebo_master_uri="${FOREST_GAZEBO_MASTER_URI:-http://127.0.0.1:11355}"

mkdir -p "${forest_v1_build_dir}" "${forest_v1_ros_dir}/log" "${forest_v1_output_dir}"
export ROS_HOME="${forest_v1_ros_dir}"
export ROS_LOG_DIR="${forest_v1_ros_dir}/log"
export ROS_MASTER_URI="${forest_v1_ros_master_uri}"
export GAZEBO_MASTER_URI="${forest_v1_gazebo_master_uri}"

mapfile -t forest_v1_seed_pairs < <(
  python3 -c 'import json,sys; data=json.load(open(sys.argv[1], encoding="utf-8")); [print("{} {}".format(item["logical_seed"], item["raw_seed"])) for item in data["seed_pool"]]' "${forest_v1_config}"
)

for forest_v1_pair in "${forest_v1_seed_pairs[@]}"; do
  read -r forest_v1_logical_seed forest_v1_raw_seed <<<"${forest_v1_pair}"
  forest_v1_target="${forest_v1_output_dir}/learning_speed_forest_v1_seed${forest_v1_logical_seed}.world"
  if [[ -e "${forest_v1_target}" && "${forest_v1_overwrite}" != "1" ]]; then
    echo "拒绝覆盖已有文件：${forest_v1_target}" >&2
    echo "确认需要重建 v1 输出时，显式设置 FOREST_OVERWRITE=1。" >&2
    exit 2
  fi
done

cmake -S "${forest_v1_dir}" -B "${forest_v1_build_dir}"
cmake --build "${forest_v1_build_dir}" --parallel 2

roslaunch gazebo_ros empty_world.launch \
  world_name:="${forest_v1_base_world}" \
  gui:=false paused:=false use_sim_time:=true &
forest_v1_roslaunch_pid=$!

cleanup_forest_v1_batch() {
  kill "${forest_v1_roslaunch_pid}" 2>/dev/null || true
  wait "${forest_v1_roslaunch_pid}" 2>/dev/null || true
}
trap cleanup_forest_v1_batch EXIT INT TERM

for _ in $(seq 1 200); do
  if rosservice info /gazebo/spawn_sdf_model >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done
if ! rosservice info /gazebo/spawn_sdf_model >/dev/null 2>&1; then
  echo "Gazebo spawn service 未就绪。" >&2
  exit 3
fi

forest_v1_delete_layout() {
  local forest_v1_model
  for forest_v1_model in sparse_0 sparse_1 sparse_2 \
    medium_3 medium_4 medium_5 medium_6 medium_7 medium_8 \
    dense_9 dense_10 dense_11 dense_12 dense_13 dense_14 dense_15 dense_16 dense_17; do
    rosservice call /gazebo/delete_model "model_name: '${forest_v1_model}'" >/dev/null
  done
}

forest_v1_save_world() {
  local forest_v1_target="$1"
  gz topic -p /gazebo/server/control -m \
    "save_world_name: 'forest_randomization_v1' save_filename: '${forest_v1_target}'"
  for _ in $(seq 1 100); do
    if [[ -s "${forest_v1_target}" ]] && xmllint --noout "${forest_v1_target}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.1
  done
  echo "保存 world 失败：${forest_v1_target}" >&2
  return 1
}

for forest_v1_pair in "${forest_v1_seed_pairs[@]}"; do
  read -r forest_v1_logical_seed forest_v1_raw_seed <<<"${forest_v1_pair}"
  forest_v1_target="${forest_v1_output_dir}/learning_speed_forest_v1_seed${forest_v1_logical_seed}.world"
  if [[ "${forest_v1_overwrite}" == "1" ]]; then
    rm -f -- "${forest_v1_target}"
  fi
  "${forest_v1_build_dir}/create_random_forest" _seed:="${forest_v1_raw_seed}"
  forest_v1_save_world "${forest_v1_target}"
  echo "已生成 logical_seed=${forest_v1_logical_seed} raw_seed=${forest_v1_raw_seed}: ${forest_v1_target}"
  forest_v1_delete_layout
done

python3 "${forest_v1_dir}/verify_forest_randomization_v1.py" \
  --config "${forest_v1_config}" \
  --world-dir "${forest_v1_output_dir}"
