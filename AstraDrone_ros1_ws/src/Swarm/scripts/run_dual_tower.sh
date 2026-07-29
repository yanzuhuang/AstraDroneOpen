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
profile=high
results_dir=""
while (($#)); do
  case "$1" in
    --control) enable_control=true ;;
    --headless) gui=false ;;
    --cloud-fusion) fusion=true ;;
    --rviz) rviz=true ;;
    --stop) stop=true ;;
    --profile)
      shift
      if (($# == 0)); then
        echo "--profile 需要 high 或 low" >&2
        exit 2
      fi
      profile="$1"
      ;;
    --results-dir)
      shift
      if (($# == 0)); then
        echo "--results-dir 需要目录" >&2
        exit 2
      fi
      results_dir="$1"
      ;;
    --help)
      echo "run_dual_tower.sh [--profile high|low] [--control] [--headless]"
      echo "                  [--cloud-fusion] [--rviz] [--results-dir DIR]"
      echo "run_dual_tower.sh --stop"
      echo "默认 high + dry-run；只有 --control 会解锁并执行双机自动飞行。"
      exit 0
      ;;
    *)
      echo "未知参数: $1" >&2
      exit 2
      ;;
  esac
  shift
done

if [[ "$profile" != "high" && "$profile" != "low" ]]; then
  echo "无效 profile: $profile（仅支持 high 或 low）" >&2
  exit 2
fi

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

if [[ -z "$results_dir" ]]; then
  results_dir="/tmp/astra_swarm_evidence/${profile}_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$results_dir"

task_config="$repo_root/AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/config/stage3_ego.yaml"
low_altitude=false
uav1_top=34.0
uav1_final=30.0
uav2_top=28.0
uav2_final=24.0
uav1_takeoff=4.0
uav2_takeoff=4.0
map_size_z=46.0
ground_height=-0.50
virtual_ceil=45.0
mission_maximum_height=45.0
max_vel=0.30
max_acc=0.50
planning_horizon=7.5
if [[ "$profile" == "low" ]]; then
  task_config="$repo_root/AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/config/stage3_low_altitude.yaml"
  low_altitude=true
  uav1_top=13.0
  uav1_final=9.0
  uav2_top=7.0
  uav2_final=3.0
  uav1_takeoff=13.0
  uav2_takeoff=7.0
  map_size_z=20.0
  ground_height=2.0
  virtual_ceil=18.0
  mission_maximum_height=18.0
  max_vel=0.50
fi

echo "profile=$profile control=$enable_control results=$results_dir"
exec roslaunch astra_swarm_bringup dual_tower_inspection.launch \
  enable_control:="$enable_control" gui:="$gui" \
  enable_cloud_fusion:="$fusion" rviz:="$rviz" \
  task_config:="$task_config" low_altitude_enabled:="$low_altitude" \
  uav1_top_height:="$uav1_top" uav1_final_height:="$uav1_final" \
  uav2_top_height:="$uav2_top" uav2_final_height:="$uav2_final" \
  uav1_takeoff_height:="$uav1_takeoff" \
  uav2_takeoff_height:="$uav2_takeoff" \
  map_size_z:="$map_size_z" ground_height:="$ground_height" \
  virtual_ceil_height:="$virtual_ceil" \
  mission_maximum_height:="$mission_maximum_height" \
  max_vel:="$max_vel" max_acc:="$max_acc" \
  planning_horizon:="$planning_horizon" obstacles_inflation:=0.40 \
  uav1_report_file:="$results_dir/uav1.csv" \
  uav2_report_file:="$results_dir/uav2.csv"
