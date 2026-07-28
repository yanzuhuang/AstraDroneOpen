#!/usr/bin/env bash

set -Eeuo pipefail

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
script_dir="$(dirname "$script_path")"
repo_root="$(readlink -f "$script_dir/../..")"
session_name="stage3_low_altitude"
default_world="$repo_root/simulation/astra_gazebo_worlds/worksite.world"

usage() {
    cat <<'EOF'
用法：
  stage3_low_altitude.sh [--gui] [--rviz] [--world FILE]
                         [--report FILE] [--bag FILE]
  stage3_low_altitude.sh --control [--gui] [--rviz] [--world FILE]
                         [--report FILE] [--bag FILE]
  stage3_low_altitude.sh --stop

默认是无控制验证。只有显式 --control 才会解锁、垂直起飞至 3 m，
并在项目原有 worksite.world 中按固定正式目标执行 ENTRY_GATE、
原顺序 3 m 绕塔一圈、EXIT_GATE、HOME_HOVER 和返航降落。
CSV 与 rosbag 默认写入 /tmp/astra_stage3_low_evidence/。
EOF
}

setup_ros() {
    source /opt/ros/noetic/setup.bash
    source "$repo_root/simulation/sim_workspace/devel/setup.bash"
    source "$repo_root/AstraDrone_ros1_ws/devel/setup.bash" --extend
}

setup_px4() {
    setup_ros
    local px4_root="${PX4_AUTOPILOT_ROOT:-/home/yanzu/PX4-Autopilot}"
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
        recorder)
            setup_ros
            wait_topic /mavros/state "MAVROS/PX4" 180
            exec bash "$script_dir/stage3_low_record_bag.sh" "$1"
            ;;
        integration)
            setup_ros
            wait_topic /mavros/state "MAVROS/PX4" 180
            wait_topic /Odometry "FAST-LIO odometry" 180
            wait_topic /cloud_registered "FAST-LIO cloud" 60
            for _ in {1..60}; do
                if rosnode ping -c 1 /stage3_low_evidence_recorder \
                    >/dev/null 2>&1; then
                    break
                fi
                sleep 1
            done
            rosnode ping -c 1 /stage3_low_evidence_recorder \
                >/dev/null 2>&1 || {
                echo "等待低空证据 recorder 注册超时" >&2
                exit 1
            }
            exec roslaunch astra_tower_mission stage3_low_altitude.launch \
                enable_control:="$1" rviz:="$2" report_file:="$3"
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

enable_control=false
world_file="$default_world"
gazebo_gui=false
rviz=false
stop=false
report_file=""
bag_file=""

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --control) enable_control=true; shift ;;
        --world)
            [[ "$#" -ge 2 ]] || { echo "--world 缺少值" >&2; exit 2; }
            world_file="$2"; shift 2
            ;;
        --gui) gazebo_gui=true; shift ;;
        --rviz) rviz=true; shift ;;
        --report)
            [[ "$#" -ge 2 ]] || { echo "--report 缺少值" >&2; exit 2; }
            report_file="$(readlink -m "$2")"; shift 2
            ;;
        --bag)
            [[ "$#" -ge 2 ]] || { echo "--bag 缺少值" >&2; exit 2; }
            bag_file="$(readlink -m "$2")"; shift 2
            ;;
        --stop) stop=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ "$stop" == true ]]; then
    if ! tmux has-session -t "$session_name" 2>/dev/null; then
        echo "低空避障会话未运行。"
        exit 0
    fi
    evidence_report="$(
        tmux show-options -t "$session_name" -v @report_file \
            2>/dev/null || true
    )"
    evidence_bag="$(
        tmux show-options -t "$session_name" -v @bag_file \
            2>/dev/null || true
    )"
    evidence_world="$(
        tmux show-options -t "$session_name" -v @world_file \
            2>/dev/null || true
    )"
    if tmux list-windows -t "$session_name" -F '#{window_name}' |
        grep -qx recorder; then
        tmux send-keys -t "$session_name:recorder" C-c 2>/dev/null || true
        for _ in {1..60}; do
            if ! tmux list-windows -t "$session_name" -F '#{window_name}' |
                grep -qx recorder; then
                break
            fi
            sleep 1
        done
    fi
    while IFS= read -r window; do
        tmux send-keys -t "$session_name:$window" C-c 2>/dev/null || true
    done < <(tmux list-windows -t "$session_name" -F '#{window_name}')
    for _ in {1..30}; do
        if ! tmux has-session -t "$session_name" 2>/dev/null; then
            break
        fi
        sleep 1
    done
    tmux kill-session -t "$session_name" 2>/dev/null || true
    if [[ -n "$evidence_bag" && ! -e "$evidence_bag" &&
          -e "${evidence_bag}.active" ]]; then
        echo "rosbag 未写入索引尾部，正在恢复索引：${evidence_bag}.active"
        rosbag reindex "${evidence_bag}.active"
        mv "${evidence_bag}.active" "$evidence_bag"
    fi
    echo "低空避障会话已停止。"
    if [[ -n "$evidence_report" && -s "$evidence_report" ]]; then
        setup_ros
        python3 "$repo_root/scripts/tool/plot_low_altitude_trajectory.py" \
            --csv "$evidence_report" \
            --bag "$evidence_bag" \
            --world "$evidence_world" \
            --output-dir "$repo_root/trc_picture"
    fi
    exit 0
fi

[[ -f "$world_file" ]] || { echo "world 不存在：$world_file" >&2; exit 1; }
world_file="$(readlink -f "$world_file")"
default_world="$(readlink -f "$default_world")"
if [[ "$world_file" != "$default_world" ]]; then
    echo "低空模式只允许使用项目原有 worksite.world：$default_world" >&2
    exit 2
fi
[[ -f "$repo_root/simulation/sim_workspace/devel/setup.bash" ]] || {
    echo "下层仿真工作空间尚未构建。" >&2
    exit 1
}
[[ -f "$repo_root/AstraDrone_ros1_ws/devel/setup.bash" ]] || {
    echo "主 ROS 工作空间尚未构建。" >&2
    exit 1
}

if tmux has-session -t "$session_name" 2>/dev/null; then
    echo "低空避障会话已运行；请先执行 $script_path --stop" >&2
    exit 1
fi
for process_spec in \
    'gzserver|Gazebo' \
    '/build/px4_sitl_default/bin/px4|PX4 SITL' \
    'mavros_node|MAVROS' \
    'laserMapping|FAST-LIO' \
    'ego_planner_node|EGO planner' \
    'traj_server|traj_server' \
    'ego_mavros_bridge|EGO bridge' \
    'stage3_ego_mission_node|stage3 task'; do
    process_pattern="${process_spec%%|*}"
    process_label="${process_spec#*|}"
    if pgrep -f "$process_pattern" >/dev/null 2>&1; then
        echo "检测到冲突进程：$process_label" >&2
        exit 1
    fi
done

evidence_dir="/tmp/astra_stage3_low_evidence"
mkdir -p "$evidence_dir"
mode="dry_run"
[[ "$enable_control" == true ]] && mode="control"
timestamp="$(date +%Y%m%d_%H%M%S)"
if [[ -z "$report_file" ]]; then
    report_file="$evidence_dir/${mode}_${timestamp}.csv"
fi
if [[ -z "$bag_file" ]]; then
    bag_file="$evidence_dir/${mode}_${timestamp}.bag"
fi
mkdir -p "$(dirname "$report_file")" "$(dirname "$bag_file")"
if [[ -e "$report_file" || -e "$bag_file" ||
      -e "${bag_file}.active" ]]; then
    echo "拒绝覆盖已有证据文件。" >&2
    exit 2
fi

printf -v px4_command '%q --component px4 %q %q' \
    "$script_path" "$world_file" "$gazebo_gui"
printf -v fast_lio_command '%q --component fast_lio' "$script_path"
printf -v recorder_command '%q --component recorder %q' \
    "$script_path" "$bag_file"
printf -v integration_command '%q --component integration %q %q %q' \
    "$script_path" "$enable_control" "$rviz" "$report_file"

tmux new-session -d -s "$session_name" -n px4_gazebo "$px4_command"
tmux set-option -t "$session_name" @report_file "$report_file"
tmux set-option -t "$session_name" @bag_file "$bag_file"
tmux set-option -t "$session_name" @world_file "$world_file"
tmux new-window -d -t "$session_name:" -n fast_lio "$fast_lio_command"
tmux new-window -d -t "$session_name:" -n recorder "$recorder_command"
tmux new-window -d -t "$session_name:" -n integration "$integration_command"
tmux select-window -t "$session_name:integration"

echo "低空避障已启动：control=$enable_control"
echo "实际加载 world：$world_file"
echo "证据 CSV：$report_file"
echo "证据 bag：$bag_file"
echo "查看日志：tmux attach -t $session_name"
echo "安全停止：$script_path --stop"
