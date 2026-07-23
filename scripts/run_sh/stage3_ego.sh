#!/usr/bin/env bash

set -Eeuo pipefail

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
script_dir="$(dirname "$script_path")"
repo_root="$(readlink -f "$script_dir/../..")"
session_name="stage3_ego"
default_world="$repo_root/simulation/astra_gazebo_worlds/worksite.world"

usage() {
    cat <<'EOF'
用法：
  stage3_ego.sh [--gui] [--rviz] [--world FILE] [--report FILE]
                [--sector-limit 1..8] [--bag FILE] [--attach]
  stage3_ego.sh --control [--gui] [--rviz] [--world FILE]
                [--report FILE] [--sector-limit 1..8] [--bag FILE] [--attach]
  stage3_ego.sh --stop

查看完整动态仿真（会自动解锁、起飞和执行任务）：
  stage3_ego.sh --control --sector-limit 1 --gui --rviz --attach

默认 dry-run：按阶段二已经验证的顺序启动 PX4/Gazebo、FAST-LIO、EGO、
traj_server、bridge 和阶段三任务管理器；PlannerStatus 由 EGO FSM 直接发布，但 bridge
不注册 MAVROS setpoint publisher，不解锁。

阶段三控制必须显式添加 --control。只有 dry-run、点云/TF/占据图对齐、
preflight 和控制权唯一性证据全部通过并由项目负责人批准后才能使用。
默认世界为 worksite.world。生产配置是 8 扇区；首次带控制验证必须显式
使用 --sector-limit 1，禁止直接跑完整 8 扇区。
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
            exec bash "$script_dir/stage3_record_bag.sh" "$1"
            ;;
        integration)
            setup_ros
            wait_topic /mavros/state "MAVROS/PX4" 180
            wait_topic /Odometry "FAST-LIO odometry" 180
            wait_topic /cloud_registered "FAST-LIO cloud" 60
            if [[ "$5" == true ]]; then
                for _ in {1..60}; do
                    if rosnode ping -c 1 /stage3_evidence_recorder \
                        >/dev/null 2>&1; then
                        break
                    fi
                    sleep 1
                done
                rosnode ping -c 1 /stage3_evidence_recorder >/dev/null 2>&1 || {
                    echo "等待 stage3 recorder 注册超时" >&2
                    exit 1
                }
            fi
            exec roslaunch astra_tower_mission stage3_ego.launch \
                enable_control:="$1" rviz:="$2" report_file:="$3" \
                sector_limit:="$4"
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
attach=false
stop=false
report_file=""
sector_limit=8
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
            report_file="$2"; shift 2
            ;;
        --bag)
            [[ "$#" -ge 2 ]] || { echo "--bag 缺少值" >&2; exit 2; }
            bag_file="$(readlink -m "$2")"; shift 2
            ;;
        --sector-limit)
            [[ "$#" -ge 2 ]] || { echo "--sector-limit 缺少值" >&2; exit 2; }
            [[ "$2" =~ ^[1-8]$ ]] || {
                echo "--sector-limit 必须是 1..8 的整数" >&2
                exit 2
            }
            sector_limit="$2"; shift 2
            ;;
        --attach) attach=true; shift ;;
        --stop) stop=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ "$stop" == true ]]; then
    if ! tmux has-session -t "$session_name" 2>/dev/null; then
        echo "阶段三会话未运行。"
        exit 0
    fi

    # A large compressed bag can need tens of seconds to flush its final chunk
    # and index after SIGINT. Stop the recorder first and do not destroy the
    # tmux session while that evidence is still being finalized.
    recorder_running=false
    while IFS= read -r window; do
        if [[ "$window" == "recorder" ]]; then
            recorder_running=true
            break
        fi
    done < <(tmux list-windows -t "$session_name" -F '#{window_name}')
    if [[ "$recorder_running" == true ]]; then
        echo "正在优雅停止 stage3 recorder 并等待 bag 索引写入……"
        tmux send-keys -t "$session_name:recorder" C-c 2>/dev/null || true
        for _ in {1..120}; do
            recorder_running=false
            while IFS= read -r window; do
                if [[ "$window" == "recorder" ]]; then
                    recorder_running=true
                    break
                fi
            done < <(tmux list-windows -t "$session_name" -F '#{window_name}' \
                2>/dev/null || true)
            [[ "$recorder_running" == false ]] && break
            sleep 1
        done
        if [[ "$recorder_running" == true ]]; then
            echo "stage3 recorder 在 120 秒内未完成；为保护 bag，未强制结束会话。" >&2
            exit 1
        fi
        echo "stage3 recorder 已完成，bag 索引已写入。"
    fi

    while IFS= read -r window; do
        tmux send-keys -t "$session_name:$window" C-c 2>/dev/null || true
    done < <(tmux list-windows -t "$session_name" -F '#{window_name}')
    for _ in {1..35}; do
        if ! tmux has-session -t "$session_name" 2>/dev/null; then
            break
        fi
        sleep 1
    done
    tmux kill-session -t "$session_name" 2>/dev/null || true
    echo "阶段三会话已停止。"
    exit 0
fi

[[ -f "$world_file" ]] || { echo "world 不存在：$world_file" >&2; exit 1; }
world_file="$(readlink -f "$world_file")"
[[ -f "$repo_root/simulation/sim_workspace/devel/setup.bash" ]] || {
    echo "下层仿真工作空间尚未构建。" >&2
    exit 1
}
[[ -f "$repo_root/AstraDrone_ros1_ws/devel/setup.bash" ]] || {
    echo "主 ROS 工作空间尚未构建。" >&2
    exit 1
}

if tmux has-session -t "$session_name" 2>/dev/null; then
    echo "阶段三会话已运行；请先执行 $script_path --stop" >&2
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
    'stage3_ego_mission_node|stage3 task' \
    'stage2_ego_mission_node|stage2 task' \
    'occupancy_stamp_adapter_node|occupancy adapter'; do
    process_pattern="${process_spec%%|*}"
    process_label="${process_spec#*|}"
    if pgrep -f "$process_pattern" >/dev/null 2>&1; then
        echo "检测到冲突进程：$process_label" >&2
        exit 1
    fi
done

if [[ -z "$report_file" ]]; then
    evidence_dir="/tmp/astra_stage3_evidence"
    mkdir -p "$evidence_dir"
    mode="dry_run"
    [[ "$enable_control" == true ]] && mode="control"
    report_file="$evidence_dir/${mode}_$(date +%Y%m%d_%H%M%S).csv"
fi

if [[ -n "$bag_file" ]]; then
    mkdir -p "$(dirname "$bag_file")"
    if [[ -e "$bag_file" || -e "${bag_file}.active" ]]; then
        echo "拒绝覆盖已有 bag：$bag_file" >&2
        exit 2
    fi
fi

printf -v px4_command '%q --component px4 %q %q' \
    "$script_path" "$world_file" "$gazebo_gui"
printf -v fast_lio_command '%q --component fast_lio' "$script_path"
record_bag=false
if [[ -n "$bag_file" ]]; then
    record_bag=true
    printf -v recorder_command '%q --component recorder %q' \
        "$script_path" "$bag_file"
fi
printf -v integration_command '%q --component integration %q %q %q %q %q' \
    "$script_path" "$enable_control" "$rviz" "$report_file" "$sector_limit" \
    "$record_bag"

tmux new-session -d -s "$session_name" -n px4_gazebo "$px4_command"
tmux new-window -d -t "$session_name:" -n fast_lio "$fast_lio_command"
if [[ "$record_bag" == true ]]; then
    tmux new-window -d -t "$session_name:" -n recorder "$recorder_command"
fi
tmux new-window -d -t "$session_name:" -n integration "$integration_command"
tmux select-window -t "$session_name:integration"

echo "阶段三已启动：control=$enable_control sector_limit=$sector_limit"
echo "证据 CSV：$report_file"
[[ "$record_bag" == true ]] && echo "完整 rosbag：$bag_file"
echo "查看日志：tmux attach -t $session_name"
echo "安全停止：$script_path --stop"
if [[ "$attach" == true ]]; then
    exec tmux attach-session -t "$session_name"
fi
