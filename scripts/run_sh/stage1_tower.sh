#!/usr/bin/env bash

set -Eeuo pipefail

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
script_dir="$(dirname "$script_path")"
repo_root="$(readlink -f "$script_dir/../..")"
session_name="stage1_tower"
default_world="$repo_root/simulation/astra_gazebo_worlds/forest.world"

usage() {
    cat <<'EOF'
用法：
  stage1_tower.sh [--preview] [--waypoints N] [--attach]
  stage1_tower.sh --control [--mode hover|mission] [--waypoints N]
                  [--headless] [--no-rviz] [--world FILE] [--attach]
                  [--report CSV_FILE] [--restart]
  stage1_tower.sh --stop

默认是只启动 RViz 路线预览：不启动 PX4/Gazebo、不注册 MAVROS 控制 Topic、
不解锁。只有显式 --control 才启动自动 OFFBOARD 任务。

选项：
  --preview       明确选择只读路线预览（默认）
  --control       启用 PX4/Gazebo 与阶段1自动控制
  --mode MODE     hover 仅起飞/悬停/返航/降落；mission 执行圆周（默认）
  --waypoints N   本轮均匀检查点数；正式任务8，逐级测试可用1、4
  --headless      Gazebo 无 GUI，同时不启动 RViz
  --no-rviz       不启动 RViz
  --world FILE    指定 world；默认 forest.world
  --report FILE   CSV 证据文件；默认写 /tmp/astra_stage1_evidence/
  --restart       只重启本脚本自己的 stage1_tower tmux 会话
  --attach        启动后进入 tmux 查看日志
  --stop          安全停止本脚本创建的整套阶段1进程
EOF
}

setup_ros_environment() {
    local ros_setup="/opt/ros/noetic/setup.bash"
    local sim_setup="$repo_root/simulation/sim_workspace/devel/setup.bash"
    local astra_setup="$repo_root/AstraDrone_ros1_ws/devel/setup.bash"
    for required_file in "$ros_setup" "$sim_setup" "$astra_setup"; do
        if [[ ! -f "$required_file" ]]; then
            echo "缺少环境文件：$required_file" >&2
            exit 1
        fi
    done
    # shellcheck disable=SC1090
    source "$ros_setup"
    # shellcheck disable=SC1090
    source "$sim_setup"
    # shellcheck disable=SC1090
    source "$astra_setup" --extend
}

setup_px4_environment() {
    setup_ros_environment
    local user_home="${HOME:?HOME is not set}"
    local px4_root="${PX4_AUTOPILOT_ROOT:-$user_home/PX4-Autopilot}"
    local gazebo_setup="$px4_root/Tools/simulation/gazebo-classic/setup_gazebo.bash"
    local px4_build="$px4_root/build/px4_sitl_default"
    local gazebo_pkg="$px4_root/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
    if [[ ! -f "$gazebo_setup" ]]; then
        echo "缺少 PX4 Gazebo 环境：$gazebo_setup" >&2
        exit 1
    fi
    # shellcheck disable=SC1090
    source "$gazebo_setup" "$px4_root" "$px4_build" >/dev/null
    export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:-}:$px4_root:$gazebo_pkg"
}

wait_for_topic() {
    local topic="$1"
    local description="$2"
    local timeout_seconds="$3"
    local elapsed
    echo "等待 $description（$topic，最多 ${timeout_seconds}s）……"
    for ((elapsed = 0; elapsed < timeout_seconds; elapsed += 2)); do
        if timeout 2s rostopic echo -n 1 "$topic" >/dev/null 2>&1; then
            return 0
        fi
        sleep 2
    done
    echo "等待 $description 超时：$topic" >&2
    return 1
}

run_component() {
    local component="$1"
    shift
    case "$component" in
        preview)
            setup_ros_environment
            exec roslaunch astra_tower_mission stage1_tower.launch \
                enable_control:=false waypoint_count:="$1" rviz:="$2"
            ;;
        px4_gazebo)
            setup_px4_environment
            unset LIBGL_ALWAYS_SOFTWARE MESA_LOADER_DRIVER_OVERRIDE \
                __GLX_VENDOR_LIBRARY_NAME DRI_PRIME \
                __NV_PRIME_RENDER_OFFLOAD __VK_LAYER_NV_optimus
            exec roslaunch \
                "$repo_root/simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch" \
                world:="$1" gui:="$2" interactive:=false
            ;;
        fast_lio)
            setup_ros_environment
            wait_for_topic /livox/imu/header "Gazebo MID360 IMU" 180
            exec roslaunch fast_lio mapping_mid360.launch rviz:=false
            ;;
        mission)
            setup_ros_environment
            wait_for_topic /mavros/state "MAVROS/PX4 state" 180
            wait_for_topic /Odometry/header "FAST-LIO odometry" 180
            wait_for_topic /mavros/local_position/pose "MAVROS local pose" 60
            exec roslaunch astra_tower_mission stage1_tower.launch \
                enable_control:=true run_mode:="$1" waypoint_count:="$2" \
                rviz:="$3" report_file:="$4"
            ;;
        *)
            echo "未知内部组件：$component" >&2
            exit 2
            ;;
    esac
}

stage1_processes_running() {
    local process_name
    for process_name in rosmaster rosout gzserver gzclient px4 mavros \
        fastlio_mapping tower_mission_node stage1_rviz; do
        if pgrep -x "$process_name" >/dev/null 2>&1; then
            return 0
        fi
    done
    return 1
}

list_stage1_processes() {
    local process_name
    for process_name in rosmaster rosout gzserver gzclient px4 mavros \
        fastlio_mapping tower_mission_node stage1_rviz; do
        pgrep -a -x "$process_name" 2>/dev/null || true
    done
}

stop_owned_session() {
    if ! tmux has-session -t "$session_name" 2>/dev/null; then
        echo "阶段1 tmux 会话未运行；未终止任何其他进程。"
        return 0
    fi
    while IFS= read -r window_name; do
        tmux send-keys -t "$session_name:$window_name" C-c 2>/dev/null || true
    done < <(tmux list-windows -t "$session_name" -F '#{window_name}')
    echo "已向本脚本的各窗口发送 Ctrl-C，等待 roslaunch 清理子进程……"
    local elapsed
    for ((elapsed = 0; elapsed < 35; ++elapsed)); do
        if ! stage1_processes_running; then
            break
        fi
        sleep 1
    done
    if tmux has-session -t "$session_name" 2>/dev/null; then
        tmux kill-session -t "$session_name"
    fi
    sleep 2
    if stage1_processes_running; then
        echo "仍检测到相关名称的进程；它们可能不属于本脚本，未强制终止：" >&2
        list_stage1_processes >&2
        return 1
    fi
    echo "阶段1会话已安全停止。"
}

if [[ "${1:-}" == "--component" ]]; then
    shift
    component_name="${1:?缺少内部组件名称}"
    shift
    run_component "$component_name" "$@"
fi

enable_control=false
run_mode="mission"
waypoint_count=8
world_file="$default_world"
gazebo_gui=true
rviz=true
attach=false
restart=false
stop=false
report_file=""

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --preview)
            enable_control=false
            shift
            ;;
        --control)
            enable_control=true
            shift
            ;;
        --mode)
            [[ "$#" -ge 2 ]] || { echo "--mode 缺少值" >&2; exit 2; }
            case "$2" in
                hover) run_mode="hover_only" ;;
                mission) run_mode="mission" ;;
                *) echo "--mode 只能是 hover 或 mission" >&2; exit 2 ;;
            esac
            shift 2
            ;;
        --waypoints)
            [[ "$#" -ge 2 ]] || { echo "--waypoints 缺少值" >&2; exit 2; }
            waypoint_count="$2"
            shift 2
            ;;
        --world)
            [[ "$#" -ge 2 ]] || { echo "--world 缺少文件" >&2; exit 2; }
            world_file="$2"
            shift 2
            ;;
        --headless)
            gazebo_gui=false
            rviz=false
            shift
            ;;
        --no-rviz)
            rviz=false
            shift
            ;;
        --report)
            [[ "$#" -ge 2 ]] || { echo "--report 缺少文件" >&2; exit 2; }
            report_file="$2"
            shift 2
            ;;
        --attach)
            attach=true
            shift
            ;;
        --restart)
            restart=true
            shift
            ;;
        --stop)
            stop=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "未知参数：$1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

for required_command in tmux timeout pgrep; do
    command -v "$required_command" >/dev/null 2>&1 || {
        echo "缺少命令：$required_command" >&2
        exit 1
    }
done

if [[ "$stop" == true ]]; then
    stop_owned_session
    exit $?
fi

if ! [[ "$waypoint_count" =~ ^[0-9]+$ ]] ||
   ((waypoint_count < 1 || waypoint_count > 360)); then
    echo "--waypoints 必须是 1 到 360 的整数" >&2
    exit 2
fi

if [[ "$restart" == true ]]; then
    stop_owned_session
elif tmux has-session -t "$session_name" 2>/dev/null; then
    echo "阶段1已在 tmux 会话 $session_name 中运行；用 --restart 只重启该会话。" >&2
    exit 1
fi

if stage1_processes_running; then
    echo "检测到可能冲突的 ROS/PX4/Gazebo 进程；为避免误杀和双控制源，拒绝启动：" >&2
    list_stage1_processes >&2
    echo "请确认归属并使用其原启动方式安全停止。" >&2
    exit 1
fi

if [[ "$enable_control" == false ]]; then
    printf -v preview_command '%q --component preview %q %q' \
        "$script_path" "$waypoint_count" "$rviz"
    tmux new-session -d -s "$session_name" -n preview "$preview_command"
    echo "阶段1路线预览已启动：严格圆周 + ${waypoint_count} 个检查点。"
    echo "未启动 PX4/Gazebo，未注册 MAVROS setpoint publisher，不会解锁。"
else
    [[ -f "$world_file" ]] || { echo "world 不存在：$world_file" >&2; exit 1; }
    world_file="$(readlink -f "$world_file")"
    if [[ -z "$report_file" ]]; then
        evidence_dir="/tmp/astra_stage1_evidence"
        mkdir -p "$evidence_dir"
        report_file="$evidence_dir/stage1_${run_mode}_${waypoint_count}_$(date +%Y%m%d_%H%M%S).csv"
    fi
    printf -v px4_command '%q --component px4_gazebo %q %q' \
        "$script_path" "$world_file" "$gazebo_gui"
    printf -v fast_lio_command '%q --component fast_lio' "$script_path"
    printf -v mission_command '%q --component mission %q %q %q %q' \
        "$script_path" "$run_mode" "$waypoint_count" "$rviz" "$report_file"
    tmux new-session -d -s "$session_name" -n px4_gazebo "$px4_command"
    tmux new-window -d -t "$session_name:" -n fast_lio "$fast_lio_command"
    tmux new-window -d -t "$session_name:" -n tower_mission "$mission_command"
    tmux select-window -t "$session_name:tower_mission"
    echo "阶段1控制任务已启动：PX4/Gazebo + FAST-LIO，world=$world_file"
    echo "模式=$run_mode，检查点=${waypoint_count}，CSV=$report_file"
    echo "控制节点将在完整 preflight、setpoint 预发送后才请求 OFFBOARD 和解锁。"
fi

echo "查看日志：tmux attach -t $session_name"
echo "安全停止：$script_path --stop"
if [[ "$attach" == true ]]; then
    exec tmux attach-session -t "$session_name"
fi
