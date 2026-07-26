#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../../../.." && pwd)"
px4_root="/home/yanzu/PX4-Autopilot"

enable_control=false
gui=true
fusion=false
rviz=false
stop=false
for argument in "$@"; do
  case "$argument" in
    --control) enable_control=true ;;
    --headless) gui=false ;;
    --cloud-fusion) fusion=true ;;
    --rviz) rviz=true ;;
    --stop) stop=true ;;
    --help)
      echo "run_dual_tower.sh [--control] [--headless] [--cloud-fusion] [--rviz]"
      echo "run_dual_tower.sh --stop"
      echo "默认 dry-run；只有 --control 会解锁并执行双机自动飞行。"
      exit 0
      ;;
    *)
      echo "未知参数: $argument" >&2
      exit 2
      ;;
  esac
done

source /opt/ros/noetic/setup.bash
source "$repo_root/simulation/sim_workspace/devel/setup.bash"
source "$repo_root/AstraDrone_ros1_ws/devel/setup.bash"
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:-}:$repo_root/simulation/sim_workspace/src:$px4_root:$px4_root/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
export GAZEBO_MODEL_PATH="$repo_root/simulation/astra_gazebo_models:$repo_root/simulation/px4_sim_files/px4_iris_sdf:${GAZEBO_MODEL_PATH:-}"
# The upper catkin workspace was built without the simulation workspace as an
# underlay, so sourcing it replaces these paths. Restore the lower-workspace
# Livox/Gazebo plugins explicitly for the numbered MID360 models.
export LD_LIBRARY_PATH="$repo_root/simulation/sim_workspace/devel/lib:${LD_LIBRARY_PATH:-}"
export GAZEBO_PLUGIN_PATH="$repo_root/simulation/sim_workspace/devel/lib:${GAZEBO_PLUGIN_PATH:-}"

if "$stop"; then
  launch_pattern='[r]oslaunch astra_swarm_bringup dual_tower_inspection\.launch'
  mapfile -t launch_pids < <(pgrep -f "$launch_pattern" || true)
  if ((${#launch_pids[@]} == 0)); then
    echo "双机仿真未运行；没有发送停止信号。"
    exit 0
  fi

  for launch_pid in "${launch_pids[@]}"; do
    kill -INT "$launch_pid"
  done
  echo "已向双机 roslaunch 发送 SIGINT，正在等待 PX4、Gazebo 和 ROS 节点退出……"

  for _ in {1..200}; do
    running=false
    for launch_pid in "${launch_pids[@]}"; do
      if kill -0 "$launch_pid" 2>/dev/null; then
        running=true
        break
      fi
    done
    if ! "$running"; then
      echo "双机仿真已安全退出。"
      exit 0
    fi
    sleep 0.1
  done

  echo "双机 roslaunch 在20秒内未完全退出；未强制杀进程，请检查原启动终端。" >&2
  exit 1
fi

exec roslaunch astra_swarm_bringup dual_tower_inspection.launch \
  enable_control:="$enable_control" gui:="$gui" \
  enable_cloud_fusion:="$fusion" rviz:="$rviz"
