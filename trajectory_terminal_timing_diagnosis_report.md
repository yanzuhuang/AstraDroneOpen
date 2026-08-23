# Trajectory terminal timing diagnosis report

> 日期：2026-08-24
>
> 范围：lightweight `EGO + traj_server + Hector` fixed-v_max qualification 的
> `invalid_observation:continuous / trajectory_unavailable`
>
> 正式 10000-Episode SAC training：**未启动**
>
> 正式 evaluation：**未启动**

## 1. 结论

确实存在 bug，但不是最初假设的“UAV 已完整满足 success，coordinator 却先检查下一帧
Observation C”的简单 ordering bug。

- A 的答案：**NO**。1.00 m/s pre-fix 诊断中，B-spline 结束和第一帧
  `trajectory_unavailable` 出现时，位置已进入 0.25 m 容差，但实际速度仍高于 0.20 m/s；
  terminal failure 时 success 包络只保持约 0.121 s，未达到冻结的 0.30 s sustain。
- 真正根因：EGO B-spline 正常到达其规划终点后，traj_server 按设计继续发布同一
  trajectory ID 的终点零速 Hover `PositionCommand`，让 Hector 消除 tracking lag；
  Observation C 也按设计拒绝没有未来 B-spline 段的样本。fixed Episode coordinator
  缺少“正式轨迹结束后的终端收敛期”语义，把该预期阶段当成正常飞行轨迹丢失，并在
  UAV 完成原 success sustain 前锁存 `invalid_observation:continuous`。
- 因此这是 fixed qualification terminal integration bug，不是 EGO 提前结束、traj_server
  停止输出或 Observation C valid 规则错误。

## 2. 证据与 0.75 PASS / 1.00 NO-GO 对比

用原 qualification 入口运行独立 nominal Episode，并通过外部 rosbag 只读记录：

```text
/clock
/uav1/Odometry
/uav1/planning/bspline
/uav1/planning/pos_cmd
/uav1/learning_speed/observation_c
/uav1/training/episode_identity
/uav1/position_command_to_hector/backend_state
/uav1/planner/status
```

两个有效 bag 各生成了最后 100 个 Observation step 的完整 JSON 时间线，每行包含 UAV
position、ENTRY distance、actual speed、B-spline start/end/ID、PositionCommand
flag/ID/setpoint/speed、Observation valid/reason、success-envelope sustain 和 terminal state：

```text
runtime_artifacts/fixed_vmax_terminal_timing_diagnosis/20260824_d02_v075_infrastructure_retry/timeline_analysis.json
runtime_artifacts/fixed_vmax_terminal_timing_diagnosis/20260824_d01/v100/timeline_analysis.json
```

0.75 的 d01 在 Episode 前因 gzserver exit 139/physical readiness timeout 失效，原工件
保留；只以新 ID d02 重试一次。1.00 d01 为有效、可复现的真实 pre-fix failure。

### 2.1 关键时间线

| 事件 | 0.75 m/s PASS | 1.00 m/s pre-fix NO-GO |
|---|---:|---:|
| Episode active | 11939.025 | 11938.824 |
| 最后 active B-spline | ID 15 | ID 11 |
| B-spline end | 11957.649 | 11952.795 |
| 结束时 UAV distance / speed | 尚有 future trajectory | 0.0845 m / 0.2460 m/s |
| 第一帧 trajectory_unavailable | success 后 reset barrier 才 invalid | source 11952.825 |
| 第一帧 invalid 时 distance / speed | N/A | 0.0771 m / 0.2447 m/s |
| PositionCommand | ID 15, READY | ID 11, READY，终点 hold |
| hold setpoint distance / speed | success 前仍有轨迹 | 0.0787 m / 0.0000 m/s |
| 首次进入完整 success 包络 | 11956.665 | 11952.985 |
| terminal latch | 11957.004 SUCCESS | 11953.106 FAILURE |
| latch 时 distance / speed | 0.0961 m / 0.1774 m/s | 0.0220 m / 0.1053 m/s |
| latch 前 success sustain | 约 0.339 s | 约 0.121 s |
| 最新 B-spline 在 latch 后余量 | 约 0.645 s | 已结束约 0.311 s |

0.75 在 Observation C 仍有未来 B-spline 时完成 0.30 s sustain，因此先锁存 success。
1.00 的最后 B-spline 结束时，Hector 仍在跟踪同一终点；traj_server 没有停止，100 Hz
PositionCommand 继续保持 `trajectory_flag=READY`、相同 trajectory ID、终点零速度。
Observation C 从下一帧开始正确报告：

```text
trajectory_lookup_result=no_active_trajectory_at_stamp
diagnostics=[trajectory_unavailable]
```

随后 UAV 才进入完整 success 包络。pre-fix coordinator 在 sustain 达到 0.30 s 前按 wall
time invalid grace 锁存 failure。

### 2.2 closure 顺序

fixed qualification 使用 `action_owner=fixed`，不启动 AstraDroneEnv/SAC runner，因此没有
transition recorder、Replay experience 或 SAC `terminal_transition_closed`；此处
transition closure 明确为 **N/A**，不能伪造为已验证。

pre-fix 1.00 的实际顺序：

```text
B-spline end
-> traj_server final PositionCommand hold
-> Observation C trajectory_unavailable
-> actual speed enters success envelope
-> success sustain only 0.121 s
-> coordinator TERMINAL_LATCHED FAILURE
-> Episode failure event
-> reset/next qualification closure
```

修复后顺序：

```text
B-spline end
-> strict FIXED_TERMINAL_HOLD recognized
-> Observation C remains invalid (not rewritten)
-> actual position/speed satisfy original gates for original 0.30 s
-> coordinator TERMINAL_LATCHED SUCCESS
-> Episode success event
-> normal reset/qualification closure
```

## 3. 最小修复

只修改 fixed Episode coordinator/纯合同与测试：

- 新增纯函数 `fixed_terminal_hold_matches()`；
- coordinator 仅在 `action_owner == fixed` 时允许 terminal hold；
- 必须同时满足：
  - fresh Observation C，且唯一 reason 为 `trajectory_unavailable`；
  - lookup 为 `no_active_trajectory_at_stamp`；
  - Observation source stamp 确实晚于最新正式 B-spline end；
  - Observation/latest B-spline/PositionCommand trajectory ID 完全一致；
  - PositionCommand fresh、`READY`、目标在原 0.25 m ENTRY 容差内、命令速度≤原0.20 m/s；
  - UAV 实际位置已在原 0.25 m ENTRY 容差内；
  - planner/collision/controller checks 已先通过。
- guard 只暂缓 continuous-invalid terminal；Observation C 消息仍为 invalid，valid ratio
  不被美化；实际 success 仍必须满足原位置、速度和 0.30 s sustain。
- 新增 `FIXED_TERMINAL_HOLD` event 与每 Episode `fixed_terminal_hold_used` 审计字段。

没有修改 Reward、EGO 参数/core、traj_server、Hector PID、`v_max=[0.30,1.75]`、
Observation C valid 规则/invalid grace、random Hover、SAC 参数或 Learning Speed replan
语义。SAC action-owner 路径不启用这个 fixed-only guard。

## 4. 顺序重测结果

每档 2 Episodes：Episode 1 nominal Hover，Episode 2 seed 1001 random Hover；EGO static
ceiling 1.75 m/s，其他参数保持冻结。

| fixed v_max | 结果 | terminal hold 使用 | Obs C valid | tracking max m | actual max m/s | planner/collision/controller | reset |
|---:|---|---:|---:|---:|---:|---:|---:|
| 1.00 | PASS 2/2 | 2/2 | 0.9697 | 0.1293 | 0.7927 | 0 / 0 / 0 | 2/2 |
| 1.25 | PASS 2/2 | 2/2 | 0.9544 | 0.1487 | 1.0234 | 0 / 0 / 0 | 2/2 |
| 1.50 | PASS 2/2 | 2/2 | 0.8831 | 0.1788 | 1.3640 | 0 / 0 / 0 | 2/2 |
| 1.75 | PASS 2/2 | 1/2 | 0.9144 | 0.2056 | 1.6952 | 0 / 0 / 0 | 2/2 |

1.75 nominal Episode 未使用 guard、在 B-spline 结束前正常 success；random Episode 使用
guard 后 success，证明 guard 不是全局或无条件放行。四档合计 8/8 Episode success、
8/8 reset success，没有真实 planner、`NO_FEASIBLE_TRAJECTORY`、collision 或 controller
failure。

权威工件：

```text
runtime_artifacts/fixed_vmax_terminal_timing_requalification/20260824_r01/
```

## 5. 验证

- hector backend unit tests：28 PASS；
- learning_speed_rl unit tests：105 PASS；
- Python compile：87 files PASS；
- XML parse：19 files PASS；
- YAML parse：13 files PASS；
- roslaunch static contract：PASS；
- `invalid_observation_grace=0.5`、success 0.25 m/0.20 m/s/0.30 s、EGO max_acc 0.8、
  v_max 0.30/1.75、random Hover 均保持；
- `git diff --check`：PASS。

## 6. 最终边界

结合此前 0.30/0.75 各 2/2 PASS 与本轮 1.00–1.75 顺序重测 8/8 PASS，当前
lightweight fixed-v_max qualification 对 `[0.30,1.75] m/s` 的六档覆盖最终为
**PASS**。

该 PASS 只属于 fixed-v_max lightweight backend qualification。fixed path 没有 SAC
transition closure，正式 10000-Episode SAC training 与正式 evaluation 本轮均未启动；
不得把本结果写成 SAC 收敛、正式 training runtime PASS、full-stack 或真机 PASS。
