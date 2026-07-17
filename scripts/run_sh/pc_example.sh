#!/bin/bash

# 默认保持原有行为；阶段 4 实验使用 --base-only，避免自动启动旧控制节点。
start_default_control=true
if [ "${1:-}" = "--base-only" ]; then
    start_default_control=false
elif [ "$#" -gt 0 ]; then
    echo "用法: $0 [--base-only]"
    exit 2
fi

if tmux has-session -t pc_example 2>/dev/null; then
    tmux kill-session -t pc_example
fi
tmux new-session -d -s pc_example

# split
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v

tmux select-pane -t 0
tmux send-keys "roscore &" C-m 

tmux select-pane -t 1
tmux send-keys "sleep 3s" C-m 
tmux send-keys "roslaunch $HOME/AstraDroneOpen/simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch world:=$HOME/AstraDroneOpen/simulation/astra_gazebo_worlds/forest.world" C-m

tmux select-pane -t 2
tmux send-keys "sleep 6s" C-m 
tmux send-keys "astra" C-m
tmux send-keys "roslaunch fast_lio mapping_mid360.launch rviz:=false" C-m

tmux select-pane -t 3
if [ "$start_default_control" = true ]; then
    tmux send-keys "sleep 10s" C-m
    tmux send-keys "astra" C-m
    tmux send-keys "roslaunch offboard autoarming_control.launch" C-m
else
    tmux send-keys "echo 'Stage 4 base-only: 未启动控制节点，请在新终端启动 stage4_trajectory.launch'" C-m
fi

tmux select-pane -t 4
tmux send-keys "qgc" C-m 

tmux -2 attach-session -t pc_example
