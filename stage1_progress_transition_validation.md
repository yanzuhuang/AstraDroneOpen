# Stage 1 Progress Transition Validation

验证日期：2026-08-21  
代码基线：`scene01-3uav-circuit-mission` @ `b96bacdd4f23d21da878f6ec22131796545c6705`  
验证 run：`progress_ctx_A_v125_r01` / `mission_1`  
工件目录：`runtime_artifacts/learning_speed/progress_transition_validation_20260821/progress_ctx_A_v125_r01`

## 范围

本次只修改 Learning Speed training data contract 与只读 recorder：

- transition schema 升级为 `learning_speed_sac_transition_v1.1`；
- 新增独立的 `learning_speed_progress_reward_context_v1.0`；
- `state_t` 和 `state_t_plus_1` 各自记录 mission state、mission-state receipt ROS time、`tower_mission/progress`、progress receipt ROS time及绑定的 Observation C receive time；
- transition 记录 `run_id`、`episode_id`、`P_t`、`P_t_plus_1`、`Delta_P` 和 monotonic audit flag；
- progress/state 缺失时不生成 transition，保持 fail closed；
- validation bag 增加 `/uav1/tower_mission/progress`。

Observation C 的五项 policy input 保持不变：

```text
lidar_surrogate
future_positions_body
actual_velocity_body
tracking_error_body
previous_applied_v_max
```

progress、mission state 和 run/episode provenance 只在 reward context/diagnostics 中，未进入 policy input。`reward=null`、`reward_defined=false`、`training_ready=false`；未实现 reward，未开始 SAC。planner、bridge、PX4、route 和 Observation C 语义均未修改。

## 代码与单测

- `learning_speed_rl` package tests：62 tests，0 errors，0 failures；
- transition contract 单测覆盖五项 policy input 冻结、run/episode provenance、`P_t/P_t_plus_1/Delta_P`、future receipt 拒绝、Observation 绑定和负 delta 原样保留审计；
- launch XML、runner shell、`roslaunch --nodes` 与 `--dump-params` 静态检查通过；
- 参数展开确认 recorder 的 `mission_progress`、`mission_state`、`run_id`、`episode_id` 均存在。

第一次受限测试中的 ROS integration failure 仅由沙箱禁止枚举本地网络接口造成；按项目验证规范在允许本地 ROS master/网络接口的环境中重跑后，62/62 全部通过。

## 唯一一次数据链验证飞行

配置：

- Environment A：`outdoor_village.world`；
- fixed `v_max=1.25 m/s`；
- 当前 4.0 m/s、3.0 m/s²、7.5 m parameter generation；
- `requested=filtered=applied=1.25 m/s`；
- 8 sectors、1 lap、3.0 m、counter-clockwise；
- route fingerprint：`adf4ae97c9416a443609e0463e1f84afc577aebe0f84df7a1023cce402418aed`。

该 run 是 instrumentation/data-contract validation，使用独立目录，`qualification_only=true`，没有加入或修改原 16-cell Stage 1 calibration matrix。原 matrix manifest 与 cells CSV 的 SHA-256 仍分别为：

```text
0604f41402f6f5500bac606da53e621f666af6aad53bc9dded3f15a3196f1f5d
0d75a2ed9cd88808a1a58e6c2e92e5efe69609e37dc7c8ed4c2bb70872f955c7
```

## Progress 结果

bag 中 `/uav1/tower_mission/progress` 共 3,663 条：

```text
0.000 -> 0.125 -> 0.250 -> 0.375 -> 0.500
      -> 0.625 -> 0.750 -> 0.875 -> 1.000
```

- 最小值 0，最大值 1；
- 原始 topic 相邻下降次数：0；
- recorder callback 下降次数：0；
- calibration samples 中下降次数：0；
- waypoint 在 sim time 136.524 s 合法发生 `8 -> 1` 闭环；对应 progress 已从 0.875 增至 1.0，没有 reset 或负跳变；
- `orbit_complete=true` 出现在 149.176 s，故 progress=1 仍未被误写成 mission terminal。

## Transition 因果结果

共生成 452 条 transition candidate，逐条检查：

- schema/provenance/算术/时间绑定问题：0；
- `observations_missing_progress_context=0`；
- `dropped_incomplete=0`；
- `ignored_unmatched_applied_ack=0`；
- 非预期 `Delta_P < 0`：0；
- `P_t/P_t_plus_1` 与两个嵌套 progress snapshot 不一致：0；
- `Delta_P != P_t_plus_1 - P_t`：0；
- progress/state receipt 晚于对应 Observation receive：0；
- reward context 与 `state_t/state_t_plus_1` Observation receive 不绑定：0；
- state/action/next-state 原有因果顺序违规：0；
- policy state 仍恰好五项输入：452/452；
- reward/training boundary 违规：0。

progress receipt 相对 Observation receive 的最大年龄为 0.060 s；所有 state-to-action 与 applied-to-next margins 均非负。

`Delta_P` 分布：

```text
0.000: 447 transitions
0.125:   5 transitions
negative: 0 transitions
```

原始 progress 的 8 次正跳变中，5 次落在被当前 Observation C/action 合同接受的 transition 窗口内；其余 3 次发生在没有可接受因果 transition 的间隙，因此没有被错误归因。结论只保证每条已生成 transition 的 reward context 可可靠计算，不把稀疏 candidate 集扩写为连续训练数据，也不改变 `training_ready=false`。

## EXIT / return / landing

calibration samples 中：

| Phase | Rows | Progress range | Decrease |
|---|---:|---:|---:|
| `GO_TO_EXIT_GATE` | 24 | 1.0–1.0 | 0 |
| `NORMAL_RETURN` | 100 | 1.0–1.0 | 0 |
| `RETURN_HOME` | 215 | 1.0–1.0 | 0 |
| `DONE` | 10 | 1.0–1.0 | 0 |

共有 14 条 accepted transition 涉及 `NORMAL_RETURN` 或 `RETURN_HOME`，其 `P_t=P_t_plus_1=1.0`、`Delta_P=0`。没有把返回 HOME 或 landing 误算为负巡检进度。

## 任务与安全终态

- mission：success、done、terminated，非 truncated；
- orbit complete：true；
- planner failure episodes：0；
- collision/emergency/tracking/dangerous terminal：全部 false；
- MAVROS 最终 `armed=false`、mode `AUTO.LAND`；
- PX4 extended landed state：`ON_GROUND`。

关键工件 SHA-256：

```text
control_chain.bag          94dff8dc7c7ecf5e5a58e071cad2b3f827c93a57f3c260ca06cdd0a48bd17fbd
transition_candidates.jsonl 19db36b81bd8abfbb4af1df8e7ce483e7a11ffed0c6838cb6d86895aeb0d5201
run_summary.json           6edfcbe918a2256cc6a812347eb784bfb67814e1014c0793e367a08403021ef8
mission.csv                bbce6f317788fcef5d12bbc35d4985f7f883e24e8b431b143766fb35cea38fcf
```

## 最终判定

`/uav1/tower_mission/progress` 已能以独立 reward context 因果绑定到每条新 transition，并可靠得到 `P_t`、`P_t_plus_1` 和 `Delta_P`。mission state 与 run/episode provenance 完整；五项 policy input、reward-null 边界和全部飞行/规划语义保持不变。

**PROGRESS REWARD CONTEXT READY: YES**

