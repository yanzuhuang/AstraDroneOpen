# AstraDroneOpen 强化学习正式 10k 训练配置整理结论

> 历史快照（已废弃）：本文件保留 10k transition-stop 整理当时的证据，不再描述当前
> 正式配置。当前唯一正常停止条件已改为 100 completed Episodes，checkpoint 为
> Episode 20/40/60/80/100；以 `AGENTS.md`、`studynote.md`、
> `AstraDroneOpen_项目技术演进与LearningSpeed阶段汇总.md` 和当前源码/config 为准。

> 整理日期：2026-08-23  
> 项目：`/home/yanzu/AstraDroneOpen`  
> 结论：`SUPERSEDED HISTORICAL SNAPSHOT`
> Runtime 边界：正式 10k training 与 deterministic evaluation 均未启动

## 1. 最终结论

第一轮正式 SAC training 的代码、配置、reset、checkpoint、evaluation、日志和文档
入口已经整理完成：

- 目标为 `10000` 个 valid active-Episode transitions；
- `valid_transition_count == 10000` 是唯一正常完成条件；training 不受固定 Episode 数限制；
- Replay Buffer capacity 为 `100000`；
- 从空 Replay Buffer、新 output ID 开始，不允许加载旧 10k pilot；
- 仅在 step `5000` 和 `10000` 保存 checkpoint；
- training 过程中不执行 evaluation；
- evaluation 独立加载指定 checkpoint，使用 deterministic Actor，不更新网络，也不
  创建或写入 training Replay；
- training reset 在 worksite nominal Hover 周围使用可复现的安全随机 XY 起点；
- evaluation reset 固定使用 nominal Hover；
- 正式 training、Gazebo runtime、training transitions、checkpoint 和 evaluation
  均未在本轮启动或产生。

本结论只表示代码、配置和静态测试已经准备好，不表示正式 10k 已 PASS、策略已收敛
或长期训练安全已经得到证明。

## 2. 冻结的第一轮正式训练参数

| 参数 | 当前值 | 配置字段 |
|---|---:|---|
| total valid transitions | `10000` | `training.target_valid_transitions` |
| Replay Buffer | `100000` | `replay.capacity` |
| learning starts | `1000` | `training.learning_starts` |
| batch size | `64` | `sac.batch_size` |
| critic-only startup | `100 updates` | `sac.critic_warmup_updates` |
| Actor LR | `1e-5` | `sac.policy_learning_rate` |
| Critic LR | `1e-3` | `sac.critic_learning_rate` |
| Alpha LR | `1e-3` | `sac.alpha_learning_rate` |
| gamma | `0.99` | `sac.gamma` |
| tau | `0.005` | `sac.tau` |
| target entropy | `-1` | `sac.target_entropy` |
| log-std | `[-3,-1]` | `sac.log_std_min/max` |
| learner update rate | `5 Hz wall time` | `sac.updates_per_second` |
| policy update frequency | `2` | `sac.policy_frequency` |
| normalized action | `[-1,1]` | 1-D tanh Actor / `ActionMapping` |
| v_max | `[0.05,0.40] m/s` | `action.expected_v_max_min/max` |
| SAC seed | `1` | `sac.seed` |
| checkpoint steps | `[5000,10000]` | `training.checkpoint_steps` |
| evaluation during training | 不存在 | 无配置字段/调用路径；独立 evaluation mode |
| max Episode | `500 steps` | `episode.max_steps` |
| coordinator max Episode time | `55 s` | launch `max_episode_time` |

主要 SAC 配置文件：

```text
AstraDrone_ros1_ws/src/learning_speed_rl/config/sac_training_v1.yaml
```

## 3. Random Hover reset 结论

### 3.1 最终范围

```text
nominal center = (0.0, 0.0)
x offset       = [-1.0, +1.0] m
y offset       = [-1.0, +1.0] m
z              = 3.0 m fixed
yaw            = 0 rad fixed
reset seed     = 1001
max attempts   = 32
ENTRY_GATE     = (-4.3148485145, 5.8522070123, 3.0)
```

Episode 1 使用 nominal spawn `(0,0,3)`。此后每次 training reset 使用 seed `1001`
的可复现序列重新采样 XY。Evaluation 关闭随机化并固定回 nominal Hover。

### 3.2 安全依据

审计使用当前 `worksite.world`、Hector 碰撞几何和 training EGO 参数：

- Hector UAV XY collision radius：约 `0.395567 m`；
- EGO `obstacles_inflation`：`0.30 m`；
- reset static audit additional clearance：`0.50 m`；
- z=3 m 高度范围内，附近 pine collision 最大审计半径：约 `0.859 m`；
- 最不利点是随机矩形西南角与 southwest pine 的组合；
- 计入上述全部包络后，最不利角点仍保留约 `1.3303 m` 的额外净余量；
- 此区域地面由 worksite 的 `z=0` collision plane 承载，固定 z=3 m 不受局部
  heightmap 抬升影响。

因此第一版采用完整 `2 m × 2 m` 矩形，而没有扩大到未经证明的范围。

配置文件：

```text
AstraDrone_ros1_ws/src/MissionControl/hector_ego_training_backend/
  config/worksite_training_reset.yaml
```

### 3.3 Reset 顺序与记录

正式顺序保持为：

```text
terminal + SAC closure
-> sample candidate
-> bounds/static-clearance validation
-> adapter reset-Hover target acknowledgement
-> cancel EGO + adapter hold
-> clear Observation C / Observation v2 / truth-cloud temporal history
-> stop Pose/Twist controllers
-> pause Gazebo
-> teleport candidate + zero linear/angular twist
-> unpause Gazebo
-> restart controllers + fresh Hover command + engage
-> generation barrier + truth/Mid360 five-frame warm-up
-> current-generation valid Observation C
-> fresh ENTRY B-spline and PositionCommand
-> next Episode
```

Unsafe candidate 会被拒绝并重新采样。32 次耗尽后 fail closed，不会回退到一个
已知不安全的坐标。

`episode_results.json` 的每个 Episode 都记录 `start_reset`；`reset_results.json`
记录：

- `nominal_hover`；
- `sampled_reset_x/y/z`；
- `sampled_yaw`；
- `reset_random_seed`；
- `sample_index`、`attempt_count`；
- `candidate_validation_result` 和全部 sampling attempts；
- reset generation、barrier、warm-up 与最终 physical readiness。

## 4. 主要修改文件

| 范围 | 文件 |
|---|---|
| 正式 SAC 参数 | `learning_speed_rl/config/sac_training_v1.yaml` |
| training/evaluation/checkpoint owner | `learning_speed_rl/scripts/sac_training_runner.py` |
| 纯训练 schedule 合同 | `learning_speed_rl/training/formal_training_contract.py` |
| worksite reset 参数 | `hector_ego_training_backend/config/worksite_training_reset.yaml` |
| reset sample/validation/identity | `hector_ego_training_backend/episode_reset_contract.py` |
| reset runtime owner | `training_episode_reset_coordinator.py` |
| Hector reset target同步 | `position_command_to_hector.py` |
| 正式 launch | `hector_worksite_sac_training.launch` 及其 reset includes |
| 测试 | `test_episode_reset_contract.py`、`test_formal_training_contract.py` |
| 项目交接 | `AGENTS.md`、`studynote.md`、统一RL交接文档、package README |

路径前缀分别为：

```text
/home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws/src/learning_speed_rl/
/home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws/src/MissionControl/hector_ego_training_backend/
```

## 5. 正式 10k training 启动命令

如果 `/tmp/astra_hector_training_overlay` 已被清空，先按 `studynote.md` 的步骤 1 重建
只读 Hector 临时 overlay。之后正式 training 只执行操作者脚本：

```bash
cd /home/yanzu/AstraDroneOpen
scripts/run_sh/learning_speed_sac_training.sh
```

GUI training 使用：

```bash
scripts/run_sh/learning_speed_sac_training.sh --gui
```

脚本自动生成唯一 `RUN_ID/SAC_OUTPUT`、拒绝残留 ROS/Gazebo/training stack、调用同一个
正式 launch，并自动在同一终端显示 SAC/Episode/reset/checkpoint/failure 关键日志。
Headless 与 GUI 二选一，不能复用已经启动过的 training output directory。

## 6. 数据、checkpoint 与日志

所有正式数据均写入：

```text
/home/yanzu/AstraDroneOpen/runtime_artifacts/rl_training/<RUN_ID>/
```

| 内容 | 实际路径（相对 `$SAC_OUTPUT`） |
|---|---|
| transition/action/v_max/reward | `sac_transition_audit.jsonl` |
| Episode 汇总 | `sac_episode_summaries.json`、`episode_results.json` |
| reset 数据 | `reset_results.json`、`qualification_events.jsonl` |
| Replay 完成态 | `sac_replay_snapshot.npz` |
| learner loss/Q/alpha/action | `sac_learner_metrics.jsonl` |
| step 5000 checkpoint | `sac_checkpoint_step_05000.pt` |
| step 10000 checkpoint | `sac_checkpoint_step_10000.pt` |
| checkpoint manifest | `sac_checkpoint_manifest.json` |
| runner/coordinator summary | `sac_runtime_summary.json`、`qualification_summary.json` |
| 完整 roslaunch/Gazebo/EGO console | `logs/training_console.log` |
| SAC transition/Episode/learner 关键日志 | `logs/ros/<ROS_SESSION>/uav1-sac_training_runner-*.log` |
| Episode/reset/barrier 关键日志 | `logs/ros/<ROS_SESSION>/uav1-training_episode_reset_coordinator-*.log` |
| ROS/roslaunch/node logs | `logs/ros/` |
| ROS home | `ros_home/` |

正式操作者脚本自动在训练终端筛选显示每 100 个 valid transition 的
transition/10000、Episode、reward、Replay size、update count、critic/actor loss、alpha、
action 和 `v_max`，并显示 Episode/reset/checkpoint/failure；完整 EGO/roslaunch 噪声只
落盘到 console log。

重新打开历史 run 的关键日志：

```bash
RUNNER_LOG="$(find "$SAC_OUTPUT/logs/ros" -type f \
  -name 'uav1-sac_training_runner-*.log' -print -quit)"
COORDINATOR_LOG="$(find "$SAC_OUTPUT/logs/ros" -type f \
  -name 'uav1-training_episode_reset_coordinator-*.log' -print -quit)"
tail -F "$RUNNER_LOG" "$COORDINATOR_LOG"
```

训练结束后：

```bash
python3 -m json.tool "$SAC_OUTPUT/sac_runtime_summary.json" | less
python3 -m json.tool "$SAC_OUTPUT/sac_episode_summaries.json" | less
python3 -m json.tool "$SAC_OUTPUT/reset_results.json" | less
less "$SAC_OUTPUT/sac_learner_metrics.jsonl"
find "$SAC_OUTPUT/logs/ros" -maxdepth 3 -type f -print
```

## 7. 停止与 resume 边界

正常完成是在累计精确 10000 条 valid transition 后：写入第 10000 条 Replay、保存
final checkpoint、记录 `training_target_reached`，required runner 退出并触发整个
roslaunch teardown。中途人工停止才在 roslaunch 终端按一次 `Ctrl-C`，等待节点
teardown。

- 已到达的 checkpoint step 会保留；
- transition audit、console 和 ROS 日志实时落盘；
- Ctrl-C 不承诺保存一个额外临时 checkpoint；
- 中断时不保证产生完整 Replay snapshot；
- 当前不支持从中断 checkpoint/replay resume 正式 training；
- 重新训练必须使用新 run ID，并从空 Replay Buffer 开始。

## 8. 独立 deterministic evaluation

Training 完成后设置：

```bash
export TRAINING_RUN_ID="<已完成的training RUN_ID>"
export TRAINING_OUTPUT="$ASTRA_ROOT/runtime_artifacts/rl_training/$TRAINING_RUN_ID"
```

定义 evaluation helper：

```bash
evaluate_checkpoint() {
  STEP="$1"
  PAD="$(printf '%05d' "$STEP")"
  EVAL_ID="sac_eval_${PAD}_$(date +%Y%m%d_%H%M%S)"
  EVAL_OUTPUT="$ASTRA_ROOT/runtime_artifacts/rl_evaluation/$EVAL_ID"

  mkdir -p "$ASTRA_ROOT/runtime_artifacts/rl_evaluation"
  mkdir "$EVAL_OUTPUT" || return 1
  mkdir -p "$EVAL_OUTPUT/logs/ros" "$EVAL_OUTPUT/ros_home"
  export ROS_HOME="$EVAL_OUTPUT/ros_home"
  export ROS_LOG_DIR="$EVAL_OUTPUT/logs/ros"

  set -o pipefail
  roslaunch hector_ego_training_backend hector_worksite_sac_training.launch \
    output_dir:="$EVAL_OUTPUT" \
    gui:=false \
    runner_mode:=evaluation \
    random_start_enabled:=false \
    episode_count:=3 \
    run_id:="$EVAL_ID" \
    evaluation_checkpoint_path:="$TRAINING_OUTPUT/sac_checkpoint_step_${PAD}.pt" \
    evaluation_checkpoint_step:="$STEP" \
    sac_config:="$ASTRA_ROOT/AstraDrone_ros1_ws/src/learning_speed_rl/config/sac_training_v1.yaml" \
    2>&1 | tee "$EVAL_OUTPUT/logs/evaluation_console.log"
}
```

分别执行：

```bash
evaluate_checkpoint 5000
evaluate_checkpoint 10000
```

每次 evaluation 使用独立 output directory。Evaluation 强制：

- fixed nominal Hover；
- deterministic Actor；
- `network_updates=false`；
- `write_training_replay=false`；
- checkpoint step 与 checkpoint 内部 counter 精确一致。

## 9. 静态验证结果

| 检查 | 结果 |
|---|---|
| Hector package nosetests | `25/25 PASS` |
| learning_speed_rl nosetests | `105 total, 0 error/failure, 3 optional-torch skips`；显式加载当前 PyTorch 后 SAC/Replay/formal schedule `16/16 PASS` |
| Python compile | PASS |
| YAML parse | PASS |
| launch XML parse | PASS |
| Hector 临时 overlay 窄构建 | PASS |
| Astra 两个 training packages 窄构建 | PASS |
| training/evaluation `roslaunch --nodes` | PASS |
| training/evaluation `roslaunch --dump-params` | PASS |
| training forbidden PX4/MAVROS/FAST-LIO/bridge nodes | `NONE` |
| training random reset | `true` |
| evaluation random reset | `false` |
| checkpoint steps | `[5000,10000]` |
| evaluation during training | `false` |
| evaluation Replay/update | `false/false` |
| `git diff --check` | PASS |

本轮只执行了编译、YAML/XML、纯逻辑/unit test、窄构建和 launch 静态展开。

## 10. 当前明确未完成

- 正式 10k training 尚未启动；
- step 5000/10000 正式 checkpoint 尚未生成；
- 新配置的 runtime random-reset qualification 尚未执行；
- deterministic evaluation 尚未执行；
- 没有已收敛 SAC 模型；
- 没有 resume、PER、Stage 2、full-stack transfer 或真机验证结论。

# FINAL STATUS: CODE READY; FORMAL 10K TRAINING AND EVALUATION NOT STARTED
