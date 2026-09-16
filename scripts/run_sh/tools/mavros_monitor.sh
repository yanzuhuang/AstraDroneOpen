#!/bin/bash
# Read-only, foreground-owned four-pane MAVROS monitor.
set -euo pipefail

usage() {
    cat <<'EOF'
Usage: mavros_monitor.sh [--namespace /|/uav1|/uav2|/uav3] [--help]

Monitor four existing topics with rostopic echo in a private tmux session:
  /mavros/local_position/pose
  /mavros/state
  /mavros/setpoint_position/local
  /mavros/setpoint_raw/local
Default namespace: / (root). --namespace prepends /uav1, /uav2 or /uav3.
Requires tmux, a sourced ROS environment and an already running ROS master.
Does not start ROS/Gazebo/PX4/control, publish topics or create result directories.
Run in a separate terminal, outside tmux. Ctrl+b d exits and stops this monitor;
Ctrl+C inside a pane stops only that pane. Closing the launcher also cleans up.
Each invocation owns a unique session; existing sessions are never replaced.
There is no --stop option and no persistent detached mode.
EOF
}

namespace=''
while (($#)); do
    case "$1" in
        --help|-h) usage; exit 0 ;;
        --namespace)
            if (($# < 2)); then
                echo 'Missing --namespace value' >&2; exit 2
            fi
            case "$2" in
                /) namespace='' ;;
                /uav1|/uav2|/uav3) namespace="$2" ;;
                *) echo 'Namespace must be /, /uav1, /uav2 or /uav3' >&2; exit 2 ;;
            esac
            shift 2 ;;
        *) echo "Unknown argument: $1 (see --help)" >&2; exit 2 ;;
    esac
done

if [[ -n ${TMUX:-} ]]; then
    echo 'Run this monitor in a separate terminal outside tmux.' >&2; exit 2
fi
for dependency in tmux rostopic; do
    command -v "$dependency" >/dev/null || {
        echo "Missing dependency: $dependency" >&2; exit 127;
    }
done

topics=(
    /mavros/local_position/pose
    /mavros/state
    /mavros/setpoint_position/local
    /mavros/setpoint_raw/local
)
session="mavros_monitor_${UID}_$$_${RANDOM}"
created=false
cleanup() {
    if [[ $created == true ]]; then
        tmux -L "$session" kill-session -t "=$session" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

# Only whitelisted namespace values and literal topic names enter shell commands.
tmux -L "$session" -f /dev/null new-session -d -s "$session" -n monitor \
    "exec rostopic echo ${namespace}${topics[0]}"
created=true
for topic in "${topics[@]:1}"; do
    tmux -L "$session" split-window -t "=$session:monitor" \
        "exec rostopic echo ${namespace}${topic}"
    tmux -L "$session" select-layout -t "=$session:monitor" tiled >/dev/null
done
printf 'MAVROS monitor session: %s\nExit and clean up: Ctrl+b d\n' "$session"
tmux -L "$session" -2 attach-session -t "=$session"
