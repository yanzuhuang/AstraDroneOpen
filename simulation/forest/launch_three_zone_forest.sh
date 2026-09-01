#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash

forest_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
forest_build_dir="${TMPDIR:-/tmp}/astra_three_zone_forest_build"
forest_ros_dir="${TMPDIR:-/tmp}/astra_three_zone_forest_ros"
# 默认随机种子。这里的 0 会通过私有 ROS 参数覆盖 create_random_forest.cpp
# 中 seed 的备用默认值；要永久更换启动脚本的默认 seed，应修改这里。
# 临时换 seed 可执行：FOREST_SEED=2 ./launch_three_zone_forest.sh
forest_seed="${FOREST_SEED:-0}"

mkdir -p "${forest_build_dir}" "${forest_ros_dir}/log"
export ROS_HOME="${forest_ros_dir}"
export ROS_LOG_DIR="${forest_ros_dir}/log"

cmake -S "${forest_dir}" -B "${forest_build_dir}"
cmake --build "${forest_build_dir}" --parallel 2

# Forest Randomization v1 的空白 base world；不覆盖已经验证的 seed0 基准 world。
roslaunch gazebo_ros empty_world.launch \
  world_name:="${forest_dir}/learning_speed_forest_v1_base.world" \
  gui:=true paused:=false use_sim_time:=true &
forest_roslaunch_pid=$!

cleanup_forest_visualization() {
  kill "${forest_roslaunch_pid}" 2>/dev/null || true
}
trap cleanup_forest_visualization EXIT INT TERM

for _ in $(seq 1 100); do
  if rosservice info /gazebo/spawn_sdf_model >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done

if ! rosservice info /gazebo/spawn_sdf_model >/dev/null 2>&1; then
  echo "Gazebo spawn service did not become ready." >&2
  exit 1
fi

"${forest_build_dir}/create_random_forest" _seed:="${forest_seed}"
wait "${forest_roslaunch_pid}"
