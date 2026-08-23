# SAC training parameter and v_max qualification report

> 后续状态：本文件记录 pre-fix 首轮 NO-GO。`trajectory_terminal_timing_diagnosis_report.md`
> 已确认并最小修复 fixed terminal-convergence integration bug；1.00–1.75 顺序重测 8/8
> PASS，当前 fixed-v_max 六档 `[0.30,1.75]` 最终 PASS。历史失败表不回写。

> 日期：2026-08-24
>
> 范围：代码/config/launch/test/文档与 lightweight fixed-v_max 短程 qualification
>
> 正式长期 training 是否启动：**NO**
>
> 正式 evaluation 是否启动：**NO**

## 1. 参数修改结果

当前唯一正式配置为
`AstraDrone_ros1_ws/src/learning_speed_rl/config/sac_training_v1.yaml`：

| 参数 | 正式值 |
|---|---:|
| total training Episodes | 10000 completed Episodes |
| max steps / Episode | 500 |
| coordinator max Episode time | 55 s（保持原正式值） |
| checkpoint | 每 500 completed Episodes；500…10000，共 20 个 |
| final checkpoint | Episode 10000 |
| independent evaluation | 100 Episodes |
| Replay capacity | 100000 transitions |
| batch size | 64 |
| learning starts | 1000 transitions |
| critic-only startup | 100 learner updates |

Episode 9999 不满足正常 stop；Episode 10000 在 terminal transition、Replay closure、
checkpoint 全部完成后正常 stop；Episode 10001 被 schedule/coordinator 双层拒绝。
Training 分支没有 evaluation 调用路径。Evaluation 分支为 deterministic Actor、fixed
nominal Hover、无 learner/network update、无 training Replay。

## 2. 新 v_max 映射

正式 normalized Actor action 保持 `[-1,1]`，映射为：

```text
v_max = 1.025 + 0.725 * action
action=-1 -> 0.30 m/s
action= 0 -> 1.025 m/s
action=+1 -> 1.75 m/s
```

实现仍使用通用线性 `ActionMapping`，没有添加 action shaping。

## 3. 旧隐藏速度限制审计

正式 roslaunch 静态展开结果：

| active 参数 | 展开值 |
|---|---:|
| EGO manager/optimization/bspline static max | 1.75 m/s |
| EGO dynamic minimum / maximum | 0.30 / 1.75 m/s |
| SpeedSafetyFilter minimum / maximum | 0.30 / 1.75 m/s |
| SAC runner expected minimum / maximum | 0.30 / 1.75 m/s |

结论：正式 action 链未发现 active 的旧 0.05/0.40 clamp。源码中仍存在 generic backend
fallback、EGO generic default、0.05 s timing、0.05 feasibility tolerance、0.40 m collision
高度门和 `minimum_active_speed_mps=0.05` 等 literal；它们不是正式 action 上下限，正式
launch 已显式覆盖 action bounds。未加入 slew、low-pass、hysteresis、cooldown 或 action
shaping；Learning Speed force-replan delta 仍为当前 `[-0.3,+0.5] m/s` 外触发一次。

## 4. fixed-v_max 各档测试结果

复用现有
`hector_worksite_training_episode_reset.launch` 与 worksite Episode/reset coordinator；每档
2 Episodes：Episode 1 nominal Hover，Episode 2 seed 1001 random Hover。每档 EGO static
ceiling 为正式 1.75 m/s；没有修改 EGO 其他参数、Hector PID、Reward、Observation C、
random Hover 或 safety gates。

| fixed v_max | verdict | Episodes | Obs C valid | tracking p95/max m | actual p95/max m/s | replans | reset |
|---:|---|---:|---:|---:|---:|---:|---:|
| 0.30 | PASS | 2 success | 1.0000 | 0.0178 / 0.0675 | 0.2099 / 0.3023 | 76 | 2/2 |
| 0.75 | PASS | 2 success | 1.0000 | 0.0568 / 0.1051 | 0.5315 / 0.5538 | 30 | 2/2 |
| 1.00 | NO-GO | 2 failure | 0.9828 | 0.0896 / 0.1304 | 0.7372 / 0.7891 | 21 | 2/2 |
| 1.25 | NO-GO | 2 failure | 0.9826 | 0.1146 / 0.1482 | 0.9690 / 1.0212 | 17 | 2/2 |
| 1.50 | NO-GO | 2 failure | 0.9810 | 0.1442 / 0.1816 | 1.2168 / 1.3622 | 15 | 2/2 |
| 1.75 | NO-GO | 2 failure | 0.9789 | 0.1699 / 0.2041 | 1.5630 / 1.6972 | 13 | 2/2 |

六档 planner failure、`NO_FEASIBLE_TRAJECTORY`、collision、controller failure 均为 0；
12/12 reset 成功。1.00–1.75 的每个 Episode 都以
`invalid_observation:continuous` fail-closed；console 将它对齐到 EGO 轨迹结束/terminal
convergence 时的 `Observation C invalid: trajectory_unavailable`。这是有效真实失败，不
补跑、不降门限。

0.30 PASS 后 roslaunch teardown 阶段出现一次 adapter timer 向已关闭 topic publish 的
traceback；它发生在 coordinator 已写出 PASS、2/2 Episode 与 2/2 reset 之后，未计作
飞行期 controller/reset failure，但作为 cleanup warning 保留在原 console/ROS 日志中。

0.75 的 q01 在 Episode 开始前因 gzserver exit 139 和 physical readiness timeout 失效，
原目录保留；只按新 ID q02 做了一次基础设施重试，得到表中的有效 2/2 PASS。
权威非 Markdown 工件位于：

```text
runtime_artifacts/fixed_vmax_qualification/20260824_q01/
runtime_artifacts/fixed_vmax_qualification/20260824_q02_v075_infrastructure_retry/
```

## 5. 最高稳定速度

当前 lightweight `EGO + traj_server + Hector` backend 的最高稳定通过 fixed v_max 为：

```text
0.75 m/s
```

## 6. [0.30,1.75] 是否 PASS

**NO-GO / FAIL。** 只有 0.30、0.75 通过；1.00–1.75 均触发冻结的 Observation C
continuous-invalid failure。因此不能把静态参数链正确写成 backend runtime qualification
PASS。

## 7. Reward 当前标定范围是否覆盖

现有 A/B fixed-speed calibration 数据覆盖 0.30–1.75 m/s；Stage 1 Reward speed anchors 为
0.75/1.25/1.75 m/s，公式接受正速度输入。本轮未发现明显速度域缺口，后续可单独评估
是否保持 Reward 不变。该结论不等于已经批准 Reward，也不覆盖 lightweight backend
NO-GO。本轮 `stage1_reward.yaml` 和 Reward 实现均未修改。

## 8. 当前是否允许进入正式 10000-Episode training

**不允许。** 阻塞条件是 `[0.30,1.75]` fixed-v_max lightweight qualification NO-GO，
不是参数文件或 checkpoint schedule 未完成。下一轮应单独诊断 terminal convergence 中
EGO trajectory lifetime、PositionCommand/Hector tracking 与 Observation C
`trajectory_unavailable` 的时序关系；不得为了 PASS 调 EGO、PID、Reward、Observation
C、random Hover 或安全门。

## 9. 正式 training / evaluation 最新命令

参数 YAML：

```text
/home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws/src/learning_speed_rl/config/sac_training_v1.yaml
```

正式 training 操作者入口（当前禁止执行）：

```bash
/home/yanzu/AstraDroneOpen/scripts/run_sh/learning_speed_sac_training.sh
```

它使用
`runtime_artifacts/rl_training/sac_training_10000ep_YYYYMMDD_HHMMSS/`，checkpoint 命名为
`sac_checkpoint_episode_0500.pt`、`..._1000.pt`……
`sac_checkpoint_episode_10000.pt`。

独立 evaluation 方式（必须等合格 training checkpoint 产生后才执行）：

```bash
roslaunch hector_ego_training_backend hector_worksite_sac_training.launch \
  output_dir:="$EVAL_OUTPUT" \
  gui:=false \
  runner_mode:=evaluation \
  random_start_enabled:=false \
  evaluation_episodes:=100 \
  run_id:="$EVAL_ID" \
  evaluation_checkpoint_path:="$TRAINING_OUTPUT/sac_checkpoint_episode_${CHECKPOINT_TAG}.pt" \
  evaluation_checkpoint_episode:="$CHECKPOINT_EPISODE" \
  sac_config:=/home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws/src/learning_speed_rl/config/sac_training_v1.yaml
```

完整环境、唯一目录创建和变量设置见 `studynote.md`。

## 10. 是否启动过正式长期 training

**NO。** 本轮只运行 fixed-v_max qualification；没有启动 SAC training，没有创建正式
training RUN_ID，没有更新网络或写 training Replay，也没有运行正式 evaluation。

## 验证摘要

- Python source compile：PASS
- YAML/XML parse：PASS
- unit tests：PASS（learning_speed_rl 105 + hector backend 27 = 132 tests）
- roslaunch training/evaluation static expansion：PASS
- formal active action bounds：PASS（0.30/1.75）
- git diff --check：PASS
