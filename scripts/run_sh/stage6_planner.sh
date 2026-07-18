#!/usr/bin/env bash

set -Eeuo pipefail

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
script_dir="$(dirname "$script_path")"
repo_root="$(readlink -f "$script_dir/../..")"
session_name="stage6_planner"
default_world="$repo_root/simulation/astra_gazebo_worlds/dynamic_avoidance.world"

usage() {
    cat <<'EOF'
用法：
  stage6_planner.sh [--headless] [--world WORLD_FILE] [--control] [--attach]
  stage6_planner.sh --stop

默认启动：PX4 + NVIDIA 硬件渲染 Gazebo GUI + FAST-LIO +
          EGO-Planner + NVIDIA 硬件渲染 RViz（DRY_RUN，不解锁无人机）。

选项：
  --headless     不打开 Gazebo GUI；物理仿真仍在 gzserver 中运行
  --world FILE   指定 Gazebo world，默认使用 dynamic_avoidance.world
  --control      启用 PX4 控制：自动 OFFBOARD、解锁、起飞；不加则为 DRY_RUN
  --attach       启动后进入 tmux 查看各组件日志
  --stop         停止本脚本创建的整套仿真
EOF
}

setup_environment() {
    local ros_setup="/opt/ros/noetic/setup.bash"
    local sim_setup="$repo_root/simulation/sim_workspace/devel/setup.bash"
    local astra_setup="$repo_root/AstraDrone_ros1_ws/devel/setup.bash"
    local user_home="${HOME:?HOME is not set}"
    local px4_root="${PX4_AUTOPILOT_ROOT:-$user_home/PX4-Autopilot}"
    local px4_gazebo_setup="$px4_root/Tools/simulation/gazebo-classic/setup_gazebo.bash"
    local px4_build="$px4_root/build/px4_sitl_default"
    local px4_gazebo_pkg="$px4_root/Tools/simulation/gazebo-classic/sitl_gazebo-classic"

    for required_file in "$ros_setup" "$sim_setup" "$astra_setup" "$px4_gazebo_setup"; do
        if [[ ! -f "$required_file" ]]; then
            echo "缺少运行环境文件：$required_file" >&2
            exit 1
        fi
    done

    # shellcheck disable=SC1090
    source "$ros_setup"
    # shellcheck disable=SC1090
    source "$sim_setup"
    # shellcheck disable=SC1090
    source "$astra_setup"
    # shellcheck disable=SC1090
    source "$px4_gazebo_setup" "$px4_root" "$px4_build" >/dev/null
    export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:-}:$px4_root:$px4_gazebo_pkg"
}

wait_for_topic() {
    local topic="$1"
    local description="$2"

    echo "等待 $description（$topic）……"
    until timeout 3s rostopic echo -n 1 "$topic" >/dev/null 2>&1; do
        sleep 2
    done
}

run_component() {
    local component="$1"
    shift
    setup_environment

    case "$component" in
        px4_gazebo)
            local world_file="$1"
            local gazebo_gui="$2"
            unset LIBGL_ALWAYS_SOFTWARE MESA_LOADER_DRIVER_OVERRIDE \
                __GLX_VENDOR_LIBRARY_NAME DRI_PRIME \
                __NV_PRIME_RENDER_OFFLOAD __VK_LAYER_NV_optimus
            echo "启动 PX4、MAVROS 与 Gazebo（gui=$gazebo_gui）：$world_file"
            exec roslaunch \
                "$repo_root/simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch" \
                world:="$world_file" gui:="$gazebo_gui"
            ;;
        fast_lio)
            wait_for_topic "/livox/imu/header" "Gazebo MID360 IMU"
            echo "启动 FAST-LIO……"
            exec roslaunch fast_lio mapping_mid360.launch rviz:=false
            ;;
        planner_rviz)
            local enable_control_value="${1:-false}"
            wait_for_topic "/Odometry/header" "FAST-LIO 里程计"
            wait_for_topic "/cloud_registered/header" "FAST-LIO 注册点云"
            unset LIBGL_ALWAYS_SOFTWARE MESA_LOADER_DRIVER_OVERRIDE \
                __GLX_VENDOR_LIBRARY_NAME DRI_PRIME \
                __NV_PRIME_RENDER_OFFLOAD __VK_LAYER_NV_optimus
            if [[ "$enable_control_value" == true ]]; then
                echo "启动 EGO-Planner 与 NVIDIA 硬件渲染 RViz（控制已启用）……"
            else
                echo "启动 EGO-Planner 与 NVIDIA 硬件渲染 RViz（DRY_RUN）……"
            fi
            exec roslaunch ego_gazebo_bridge stage6_gazebo.launch \
                enable_control:="$enable_control_value" rviz:=true
            ;;
        *)
            echo "未知内部组件：$component" >&2
            exit 2
            ;;
    esac
}

stage6_runtime_running() {
    local process_name
    for process_name in rosmaster gzserver gzclient rviz px4 \
        fastlio_mapping ego_planner_node traj_server waypoint_generator \
        ego_mavros_bridge; do
        if pgrep -x "$process_name" >/dev/null 2>&1; then
            return 0
        fi
    done
    return 1
}

wait_for_runtime_exit() {
    local max_seconds="$1"
    local elapsed

    for ((elapsed = 0; elapsed < max_seconds; elapsed++)); do
        if ! stage6_runtime_running; then
            return 0
        fi
        sleep 1
    done
    return 1
}

if [[ "${1:-}" == "--component" ]]; then
    shift
    if [[ "$#" -lt 1 ]]; then
        echo "缺少内部组件名称" >&2
        exit 2
    fi
    component_name="$1"
    shift
    run_component "$component_name" "$@"
fi

world_file="$default_world"
open_gazebo_gui=true
enable_control=false
attach=false
stop=false

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --headless)
            open_gazebo_gui=false
            shift
            ;;
        --world)
            if [[ "$#" -lt 2 ]]; then
                echo "--world 后必须提供文件路径" >&2
                exit 2
            fi
            world_file="$2"
            shift 2
            ;;
        --control)
            enable_control=true
            shift
            ;;
        --attach)
            attach=true
            shift
            ;;
        --detach)
            # Backward-compatible alias: detached is now the default.
            attach=false
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

if [[ "$stop" == true ]]; then
    if tmux has-session -t "$session_name" 2>/dev/null; then
        while IFS= read -r window_name; do
            tmux send-keys -t "$session_name:$window_name" C-c 2>/dev/null || true
        done < <(tmux list-windows -t "$session_name" -F '#{window_name}')
        echo "正在等待 ROS、PX4 和 Gazebo 完整退出……"
        wait_for_runtime_exit 30 || true
        if tmux has-session -t "$session_name" 2>/dev/null; then
            tmux kill-session -t "$session_name"
        fi
        if wait_for_runtime_exit 10; then
            echo "已停止 Stage6 仿真会话：$session_name"
        else
            echo "Stage6 的部分进程仍未退出；请稍候后再次执行 --stop。" >&2
            exit 1
        fi
    else
        if stage6_runtime_running; then
            echo "tmux 会话已结束，正在等待残留进程自行清理……"
            if ! wait_for_runtime_exit 20; then
                echo "仍检测到 ROS/Gazebo 进程；它们可能不属于本脚本，未强制终止。" >&2
                exit 1
            fi
        fi
        echo "Stage6 仿真会话未运行。"
    fi
    exit 0
fi

for required_command in tmux nvidia-smi timeout; do
    if ! command -v "$required_command" >/dev/null 2>&1; then
        echo "缺少命令：$required_command" >&2
        exit 1
    fi
done

if [[ ! -f "$world_file" ]]; then
    echo "Gazebo world 不存在：$world_file" >&2
    exit 1
fi
world_file="$(readlink -f "$world_file")"

if ! nvidia-smi >/dev/null 2>&1; then
    echo "NVIDIA 驱动当前不可用；请先完整重启电脑，再启动 Stage6。" >&2
    exit 1
fi

driver_version="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -n 1)"
if [[ "$driver_version" == 570.* ]]; then
    echo "检测到已知会触发 gzclient Xid 31 的 NVIDIA 570 驱动；请改用已验证的 535 驱动。" >&2
    exit 1
fi

if journalctl -b -k --no-pager 2>/dev/null | grep -q "GPU Reset Required"; then
    echo "本次开机已出现 GPU Reset Required；请完整重启后再启动 Stage6。" >&2
    exit 1
fi

if tmux has-session -t "$session_name" 2>/dev/null; then
    echo "Stage6 已在 tmux 会话 $session_name 中运行。"
    if [[ "$attach" == true ]]; then
        exec tmux attach-session -t "$session_name"
    fi
    exit 0
fi

if stage6_runtime_running; then
    echo "检测到 ROS/Gazebo 进程，等待可能仍在进行的退出清理……"
    if ! wait_for_runtime_exit 20; then
        echo "检测到其他 ROS/Gazebo 仿真仍在运行。请先停止它，避免端口和控制源冲突。" >&2
        exit 1
    fi
fi

gazebo_gui_value=false
if [[ "$open_gazebo_gui" == true ]]; then
    gazebo_gui_value=true
fi

printf -v base_command '%q --component px4_gazebo %q %q' \
    "$script_path" "$world_file" "$gazebo_gui_value"
printf -v fast_lio_command '%q --component fast_lio' "$script_path"
printf -v planner_command '%q --component planner_rviz %q' \
    "$script_path" "$enable_control"

tmux new-session -d -s "$session_name" -n px4_gazebo "$base_command"
tmux new-window -d -t "$session_name:" -n fast_lio "$fast_lio_command"
tmux new-window -d -t "$session_name:" -n planner_rviz "$planner_command"

tmux select-window -t "$session_name:planner_rviz"

echo "Stage6 已启动："
echo "  PX4/Gazebo、FAST-LIO、EGO-Planner 和 RViz 均在 tmux 会话 $session_name 中。"
if [[ "$open_gazebo_gui" == true ]]; then
    echo "  Gazebo GUI 和 RViz 均使用 NVIDIA GPU（当前驱动：$driver_version）。"
else
    echo "  Gazebo Server 以 headless 模式运行，RViz 使用 NVIDIA GPU。"
fi
if [[ "$enable_control" == true ]]; then
    echo "  PX4 控制已启用：输入健康后将自动进入 OFFBOARD、解锁并起飞至 1 m。"
    echo "  到达 HOVER_READY 后再发送 RViz 目标并启用轨迹跟踪。"
else
    echo "  当前为 DRY_RUN：无人机不会解锁，也不会发送 MAVROS setpoint。"
    echo "  完成数据与坐标检查后，用 --control 重新启动才能实际起飞。"
fi
echo "  GUI 通常在 20～30 秒内出现；可用 Alt+Tab 切换 Gazebo/RViz。"
echo "  查看日志：$script_path --attach"

if [[ "$attach" == true ]]; then
    exec tmux attach-session -t "$session_name"
fi
