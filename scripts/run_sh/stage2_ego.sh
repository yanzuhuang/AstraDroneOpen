#!/usr/bin/env bash

set -Eeuo pipefail

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
script_dir="$(dirname "$script_path")"
repo_root="$(readlink -f "$script_dir/../..")"
session_name="stage2_ego"
default_world="$repo_root/simulation/astra_gazebo_worlds/worksite.world"

usage() {
    cat <<'EOF'
用法：
  stage2_ego.sh [--scenario dry-run] [--attach]
  stage2_ego.sh --control --scenario single|dual|tower
                [--waypoints N] [--gui] [--rviz] [--world FILE]
                [--report FILE] [--attach]
  stage2_ego.sh --stop

默认 dry-run：启动 PX4/Gazebo、FAST-LIO、EGO、traj_server 和任务管理器，
但 bridge 不注册 MAVROS setpoint publisher，不解锁。

飞行验证必须显式同时给出 --control 和 single/dual/tower；顺序应为
dry-run -> 故障注入 -> single -> dual -> tower。
默认世界为 worksite.world。
EOF
}

setup_ros() {
    source /opt/ros/noetic/setup.bash
    source "$repo_root/simulation/sim_workspace/devel/setup.bash"
    source "$repo_root/AstraDrone_ros1_ws/devel/setup.bash" --extend
}

setup_px4() {
    setup_ros
    local px4_root="${PX4_AUTOPILOT_ROOT:-${HOME:?HOME is not set}/PX4-Autopilot}"
    source "$px4_root/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
        "$px4_root" "$px4_root/build/px4_sitl_default" >/dev/null
    export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:-}:$px4_root:$px4_root/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
}

wait_topic() {
    local topic="$1"
    local label="$2"
    local limit="$3"
    local elapsed
    for ((elapsed = 0; elapsed < limit; elapsed += 2)); do
        if timeout 2s rostopic echo -n 1 "$topic" >/dev/null 2>&1; then
            return 0
        fi
        sleep 2
    done
    echo "等待 $label 超时：$topic" >&2
    return 1
}

run_component() {
    local component="$1"
    shift
    case "$component" in
        px4)
            setup_px4
            exec roslaunch \
                "$repo_root/simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch" \
                world:="$1" gui:="$2" interactive:=false
            ;;
        fast_lio)
            setup_ros
            wait_topic /livox/imu/header "MID360 IMU" 180
            exec roslaunch fast_lio mapping_mid360.launch rviz:=false
            ;;
        integration)
            setup_ros
            wait_topic /mavros/state "MAVROS/PX4" 180
            wait_topic /Odometry/header "FAST-LIO odometry" 180
            wait_topic /cloud_registered/header "FAST-LIO cloud" 60
            exec roslaunch astra_tower_mission stage2_ego.launch \
                enable_control:="$1" scenario:="$2" waypoint_count:="$3" \
                rviz:="$4" report_file:="$5"
            ;;
        *)
            echo "未知内部组件：$component" >&2
            exit 2
            ;;
    esac
}

if [[ "${1:-}" == "--component" ]]; then
    shift
    component="${1:?缺少组件名}"
    shift
    run_component "$component" "$@"
fi

scenario="dry_run"
enable_control=false
world_file="$default_world"
waypoint_count=8
gazebo_gui=false
rviz=false
attach=false
stop=false
report_file=""

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --scenario)
            [[ "$#" -ge 2 ]] || { echo "--scenario 缺少值" >&2; exit 2; }
            case "$2" in
                dry-run|dry_run) scenario="dry_run" ;;
                single|dual|tower) scenario="$2" ;;
                *) echo "scenario 必须是 dry-run/single/dual/tower" >&2; exit 2 ;;
            esac
            shift 2
            ;;
        --control) enable_control=true; shift ;;
        --world)
            [[ "$#" -ge 2 ]] || { echo "--world 缺少值" >&2; exit 2; }
            world_file="$2"; shift 2
            ;;
        --waypoints)
            [[ "$#" -ge 2 ]] || { echo "--waypoints 缺少值" >&2; exit 2; }
            waypoint_count="$2"; shift 2
            ;;
        --gui) gazebo_gui=true; shift ;;
        --rviz) rviz=true; shift ;;
        --report)
            [[ "$#" -ge 2 ]] || { echo "--report 缺少值" >&2; exit 2; }
            report_file="$2"; shift 2
            ;;
        --attach) attach=true; shift ;;
        --stop) stop=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ "$stop" == true ]]; then
    if ! tmux has-session -t "$session_name" 2>/dev/null; then
        echo "阶段二会话未运行。"
        exit 0
    fi
    while IFS= read -r window; do
        tmux send-keys -t "$session_name:$window" C-c 2>/dev/null || true
    done < <(tmux list-windows -t "$session_name" -F '#{window_name}')
    for _ in {1..35}; do
        if ! pgrep -x -f 'px4|gzserver|mavros|fastlio_mapping|ego_planner_node|traj_server|ego_mavros_bridge|stage2_ego_mission_node' >/dev/null 2>&1; then
            break
        fi
        sleep 1
    done
    tmux kill-session -t "$session_name" 2>/dev/null || true
    echo "阶段二会话已停止。"
    exit 0
fi

if [[ "$scenario" == "dry_run" && "$enable_control" == true ]]; then
    echo "dry-run 禁止 --control" >&2
    exit 2
fi
if [[ "$scenario" != "dry_run" && "$enable_control" != true ]]; then
    echo "飞行场景必须显式添加 --control" >&2
    exit 2
fi
if [[ "$scenario" == "tower" ]] &&
   ! [[ "$waypoint_count" =~ ^[0-9]+$ && "$waypoint_count" -ge 1 && "$waypoint_count" -le 8 ]]; then
    echo "tower 的 --waypoints 必须为 1 到 8" >&2
    exit 2
fi
[[ -f "$world_file" ]] || { echo "world 不存在：$world_file" >&2; exit 1; }
world_file="$(readlink -f "$world_file")"

if tmux has-session -t "$session_name" 2>/dev/null; then
    echo "阶段二会话已运行；请先执行 $script_path --stop" >&2
    exit 1
fi
for process_name in rosmaster gzserver px4 mavros fastlio_mapping \
    ego_planner_node traj_server ego_mavros_bridge tower_mission; do
    if pgrep -x "$process_name" >/dev/null 2>&1; then
        echo "检测到冲突进程：$process_name" >&2
        exit 1
    fi
done

if [[ -z "$report_file" ]]; then
    evidence_dir="/tmp/astra_stage2_evidence"
    mkdir -p "$evidence_dir"
    report_file="$evidence_dir/${scenario}_$(date +%Y%m%d_%H%M%S).csv"
fi

printf -v px4_command '%q --component px4 %q %q' \
    "$script_path" "$world_file" "$gazebo_gui"
printf -v fast_lio_command '%q --component fast_lio' "$script_path"
printf -v integration_command '%q --component integration %q %q %q %q %q' \
    "$script_path" "$enable_control" "$scenario" "$waypoint_count" \
    "$rviz" "$report_file"

tmux new-session -d -s "$session_name" -n px4_gazebo "$px4_command"
tmux new-window -d -t "$session_name:" -n fast_lio "$fast_lio_command"
tmux new-window -d -t "$session_name:" -n integration "$integration_command"
tmux select-window -t "$session_name:integration"

echo "阶段二已启动：scenario=$scenario control=$enable_control"
echo "证据 CSV：$report_file"
echo "查看日志：tmux attach -t $session_name"
echo "安全停止：$script_path --stop"
if [[ "$attach" == true ]]; then
    exec tmux attach-session -t "$session_name"
fi
