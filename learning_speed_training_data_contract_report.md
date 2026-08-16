# Learning Speed SAC Training Data Contract 与人工标定采集报告

日期：2026-08-16  
分支：`scene01-3uav-circuit-mission`  
实现起点 HEAD：`214de226220448a767f68d1d2a7d91535009d714`

## 1. 边界

本次只冻结数据合同并增加只读人工标定采集器。没有计算 reward，没有设置
`phi1/phi2` 或任何 reward 权重，没有 SAC/PER、模型推理或训练，也没有修改 EGO
核心、Observation C 特征、clearance 语义、三机任务、安全状态机和 bridge 的
`1.0 m / 1.0 s` tracking gate。Learning Speed 的工程默认仍为关闭。

## 2. 最终 transition

合同版本为 `learning_speed_sac_transition_v1.0`：

```text
state_t
  -> requested_v_max
  -> SpeedSafetyFilter filtered_v_max
  -> EGO applied_v_max acknowledgement
  -> state_t+1
  -> reward
  -> terminated / truncated
```

第一版 policy input 恰好是五项：

| 字段 | source | timestamp | frame | unit | valid condition |
|---|---|---|---|---|---|
| `lidar_surrogate[3200]` | valid atomic Observation C / Observation v2 stamped mirror | `ObservationC.header.stamp=t` | 当前 `body`，ROS FLU | 障碍 bin 为 m；free/unknown 保留既有 surrogate 编码 | 3200、finite、Observation C valid |
| `future_positions_body[20][3]` | EGO 正式 `planning/bspline` 的 De Boor 求值 | 同一个 `t` | 当前 `body`，ROS FLU | m | 20x3、finite、轨迹在 `t` 有效；不是 visualization/PositionCommand 近似 |
| `actual_velocity_body[3]` | timestamped FAST-LIO position causal difference + alpha=0.30 filter | 同一个 `t` | 当前 `body`，ROS FLU | m/s | pose exact/有界插值且 velocity valid；不使用未填充的 odom twist |
| `tracking_error_body[3]` | official B-spline desired position minus timestamp-matched FAST-LIO actual position | 同一个 `t` | 当前 `body`，ROS FLU | m | desired/actual 同 frame、同 timestamp、finite |
| `previous_applied_v_max` | EGO `learning_speed/applied_v_max` 回执历史 | `<=t` 的最近回执，zero-order hold | scalar，无 frame | m/s | positive finite，禁止未来值泄漏 |

`mission_state`、`planner_state`、clearance/clutter、三个 lidar mask/semantic、
diagnostics 和 terminal provenance 都只记录，不进入 `policy_input()`。

Adapter 通过新增的只读审计镜像 `learning_speed/action_stamped` 原子记录同一 policy
cycle 的 requested/filtered 值与 ROS action stamp；控制链原有两个 `Float64` topic 不变。
EGO applied 回执仍无 Header，合同使用采集节点的 ROS callback receipt time，并强制
`requested_stamp <= filtered_stamp <= applied_stamp`。`state_t+1` 是 applied 回执后收到的
第一条更新且 valid 的 Observation C。

在产生 action candidate 前，采集器比较 `state_t` 中的 trajectory
`(start_time, traj_id, frame)` 与该时刻已收到的最新正式 `planning/bspline`；不相等就
fail closed 丢弃 action。由此保证 `state_t` 使用的是 `action_t` 之前最新可用的官方
B-spline，而不是旧轨迹或未来轨迹。

当前 reward 固定为 `null`、`reward_defined=false`、`training_ready=false`。
`terminated` 表示 mission done 或 collision proxy / EGO emergency /
tracking-safety terminal；`truncated` 保留给未正常 terminal 的超时、人工停止或基础设施
中断。物理 Gazebo contact 尚未仪器化，collision 明确仍是 EGO inflated occupancy proxy。

## 3. 人工标定采集字段

`calibration_samples.csv` 逐 Observation C 记录：

- nearest known-obstacle surrogate distance；
- known-obstacle angular-bin fraction（density proxy）、known-obstacle bin count
  （clutter proxy）、free/unknown bin count；
- body actual velocity/speed 与 body tracking-error xyz/norm；
- requested / filtered / applied `v_max` 及各自 callback receipt timestamp；
- collision proxy、emergency、现有 tracking gate 的只读镜像 terminal；
- planner state/failure/reason/consecutive count；
- mission state/result 和 Observation C valid/version/frame/invalid reasons。

`observation_diagnostics.jsonl` 另存完整 valid/unknown mask、semantic、diagnostics，
以及沿用 baseline 定义的 raw-filtered 最近点距离和 EGO inflated occupied-center
距离；二者保留各自 source stamp 和 representation，不混写成一种 clearance。

`planner_failure_episodes.csv` 记录每段 failure 的开始/结束、原因集合、最大连续失败数和
是否恢复。`transition_candidates.jsonl` 保存完整五项 state、三段 action 和 next state，
但没有 reward，不能直接用于训练。`run_summary.json` 汇总 mission、terminal、planner
episode、Observation C valid ratio 和因因果检查被丢弃的 action。

障碍 density/count 只用于后续人工标定，不是新 Observation C 特征，不是物体实例数或
物理体积，也不替换 `rule.md` 的 raw/inflated clearance 语义。

## 4. 启动一次采集

先显式启动现有仿真栈和 Learning Speed；是否加 `--control` 仍由操作者决定。采集器本身
不启用控制：

```bash
# 已运行的 stack 必须显式带 --learning-speed
scripts/run_sh/learning_speed_calibration.sh --namespace uav1 --run-id manual_001
```

若该 namespace 已有 Observation C，追加 `--observation-c-running`。结果只写入：

```text
runtime_artifacts/learning_speed/calibration/manual_001/
```

采集器在 mission-done 锁存后 1 秒自动 finalize；人工中止时由 ROS shutdown 写入
`truncated=true` 的 episode summary。

## 5. 尚未做

尚未选择 `phi1/phi2` 定义或权重、reward 各项权重、decision horizon、训练/验证划分、
SAC/PER 超参数、网络或模型；也未启动任何受控飞行进行本接口的在线标定验收。下一步应先
收集多 clutter/速度档重复数据并审查代理指标与 terminal 的可辨识性，再单独评审 reward。

## 6. 修改文件分组

- 合同与纯逻辑：`training/data_contract.py`、`training/calibration.py`；
- ROS 审计接口：`SpeedActionStamped.msg`、`speed_adapter_node.py`、
  `calibration_data_recorder.py`；
- 启动/配置：`calibration_data_collection.launch`、
  `calibration_data_collection.yaml`、`learning_speed_calibration.sh`；
- 测试：`test_training_data_contract.py`、`speed_action_integration.*`；
- 文档/构建：本报告、package README、CMake；
- 三机入口仅把原值 `max_vel=0.20` 的声明移到首次引用之前，修复 roslaunch 参数展开
  顺序，不改变数值或行为。
