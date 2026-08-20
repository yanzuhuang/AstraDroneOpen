# AstraDroneOpen 项目技术演进与 Learning Speed 阶段汇总

> 整理日期：2026-08-20  
> 整理范围：根目录 17 份专题报告，以及 `runtime_artifacts/` 内与当前技术演进、Learning Speed、Observation、clearance、轨迹跟踪、高速 qualification、planner failure、数据合同直接相关的 Markdown 证据。  
> 证据原则：本文件只重新组织已有结论；较早报告的失败、条件性结论和参数代际被保留，但均以其后的直接运行/审计证据更新，不把历史中间态误写为当前事实。

## 目录与结论速览

1. [证据范围、版本与冲突处理](#1-证据范围版本与冲突处理)
2. [系统架构与三机 EGO-Swarm 集成](#2-系统架构与三机-ego-swarm-集成)
3. [Mid360 / 感知链问题与修复](#3-mid360--感知链问题与修复)
4. [clearance / inflation 历史问题与语义清理](#4-clearance--inflation-历史问题与语义清理)
5. [Learning Speed Adapter 与 Observation C](#5-learning-speed-adapter-与-observation-c)
6. [Training Data Contract](#6-training-data-contract)
7. [固定速度 baseline 与数据代际](#7-固定速度-baseline-与数据代际)
8. [高速 tracking 根因与 bridge 修复](#8-高速-tracking-根因与-bridge-修复)
9. [CURRENT_POSITION_IN_OCCUPANCY 与 random fallback 排查](#9-current_position_in_occupancy-与-random-fallback-排查)
10. [高速参数链：4.0 m/s / 3.0 m/s2](#10-高速参数链40-ms--30-ms2)
11. [1.75 / 2.0 / 2.5 qualification 最终结果](#11-175--20--25-qualification-最终结果)
12. [当前最终冻结结论](#12-当前最终冻结结论)
13. [当前项目状态与下一步：Stage 1 Reward → SAC](#13-当前项目状态与下一步stage-1-reward--sac)

## 1. 证据范围、版本与冲突处理

### 1.1 本次纳入的正式报告

17 份指定报告均已阅读并纳入。其中，重复的背景、相同代码链和被后续实验替代的中间结论不再逐段复制；每项最终判断均保留来源链。

| 主题 | 正式报告 | 在本汇总中的作用 |
|---|---|---|
| 三机主链 | `ego_swarm_three_uav_integration_report.md` | 以末尾最终 SITL 闭环 PASS 为准，保留早期 HOLD/地图边界失败作为修复历史。 |
| Mid360 启动 | `worksite_mid360_startup_root_cause_report.md` | worksite heightfield × ODE MultiRay 根因及 mesh collision 修复。 |
| clearance | `AstraDroneOpen_障碍物膨胀与真实距离历史问题.md`、`clearance_semantics_cleanup_report.md`、`pre_entry_clearance_root_cause_report.md`、`inflation_clearance_semantics_regression_audit_report.md` | 历史语义混用、入口重复净空、回归审计和 EGO stacked margin。 |
| Adapter / Observation | `learning_speed_adapter_integration_report.md`、`learning_speed_observation_v2_report.md`、`scheme_c_trajectory_fusion_report.md` | 动态 `v_max` 边界、v2 surrogate、Observation C 融合合同。 |
| 数据与 baseline | `learning_speed_training_data_contract_report.md`、`fixed_speed_baseline_report.md` | transition 合同、终态记账与初始人工标定语义。 |
| 高速演进 | `ego_high_speed_prequalification_audit_report.md`、`high_speed_trajectory_tracking_audit_report.md`、`high_speed_current_position_in_occupancy_audit_report.md`、`high_speed_parameter_chain_update_report.md`、`high_speed_progressive_qualification_report.md`、`final_velocity_execution_chain_audit_report.md` | 从旧 bridge 限幅、异常下潜风险、参数链更新到最终运行结果与速度链关闭审计。 |

### 1.2 纳入的 `runtime_artifacts/` 证据

以下工件提供了正式报告之外的更新数据或原始运行证据，已合并其增量结论：

| 工件 | 采用的增量证据 |
|---|---|
| `learning_speed/calibration/manual_calibration_report.md` | 首轮 A/B 12 次、0.30–1.50 m/s 正式 baseline 的逐档指标。 |
| `learning_speed/calibration/observation_c_invalid_audit_report.md` | 首轮低 valid ratio 的量化基线；其问题已由最终数据质量报告关闭。 |
| `learning_speed/calibration/observation_c_final_data_quality_report.md` | 1 ns timestamp 根因、因果 replay 和 Observation C data-quality PASS。 |
| `learning_speed/calibration/observation_c_environment_a_postfix_validation.md` | 环境 A post-fix 实飞 PASS 与 0.05 s 合同未放宽的证据。 |
| `learning_speed/calibration/stage1_reward_calibration_postfix_report.md` | 14 条 post-fix A/B 正式 run、40,564 条合并样本及代际隔离。文件名含 “Stage 1 Reward”，但其中没有 reward 定义或训练。 |
| `learning_speed/high_speed_progressive_qualification_20260820/manual_calibration_report.md` | 1.75/2.0/2.5 的原始 qualification 汇总；以根目录 progressive 与 final execution-chain 报告的解释为准。 |
| `high_speed_exploration/high_speed_exploration_report.md` | 旧 2.0 m/s tracking FAIL 和 1.5 m/s 条件 PASS，是 bridge 根因审计的历史起点。 |
| `learning_speed/evaluation/20260813_gazebo_dynamic_speed_test.md`、`20260813_mock_interface_test.md` | Mock/动态 Adapter 的可逆响应；轻量 world 不替代 worksite 任务验收。 |
| `observation_v2/evaluation/evidence_manifest.md`、`observation_v2_worksite_tower_validation_20260815/OBSERVATION_V2_WORKSITE_TOWER_VALIDATION_REPORT.md` | v2 专项塔旁语义、unknown、yaw/history 对齐和只读边界。 |

`postfix_stage1_20260818/manual_calibration_report.md` 与 progressive 目录下的同名 `manual_calibration_report.md` 是模板式缺失行汇总（0/12），不代表正式实验失败；其实际有效数据已由同目录的独立汇总报告、每 run 工件和根目录报告覆盖，故不重复抄录。没有把自动日志摘要、纯运行记录或无法形成技术判断的工件纳入正文。

### 1.3 已解决的结论冲突

| 旧结论 | 后续证据 | 当前最终结论 |
|---|---|---|
| 三机低空绕塔曾因输入、时效、地图边界或授权中断而 FAILED。 | 最终 dry-run 与控制闭环完成三机 360°、EXIT、逆序返航、落地；安全门未降低。 | 三机低空绕塔 SITL 最终验收 **PASS**。 |
| Observation v2 总报告为 PARTIAL。 | 后续 worksite UAV1-only 塔旁专项验证完成 353.32° 绕塔、方向/unknown/history 专项均 PASS。 | v2 的塔旁专项语义验证 **PASS**；这不把它扩大为完整三机 v2 飞行验收或控制模块。 |
| 首轮 B 的 Observation C 有大规模 kinematic gap 与低 valid ratio。 | 直接诊断定位为 sec/nsec 经 float 往返丢 1 ns；保持 0.05 s，因果 replay 及 A 实飞验证通过。 | Observation C 数据质量门 **PASS**；旧首轮比例保留为修复前代际证据。 |
| 旧 2.0 m/s 探索连续 tracking FAIL，曾把可通过边界置于 1.5–2.0。 | 发现 EGO 高速度指令被 bridge `max_velocity=0.5`、`max_acceleration=1.0` 截断；后续参数链和 qualification 使用不同 generation。 | 旧失败不能代表修复后高速资格边界；在 4.0/3.0 generation 中 2.0 PASS。 |
| progressive 报告在 2.5 FAIL 后暂不建议进入 Reward。 | 2026-08-20 最终 request→PX4→actual 审计排除隐藏速度 ceiling，确认 2.5 是安全边界而非速度链未闭环。 | 可进入 **Stage 1 Reward 的独立设计/审计**；仍不允许实现 reward 或开始 SAC。 |

## 2. 系统架构与三机 EGO-Swarm 集成

### 2.1 当前执行主链

```text
Gazebo iris_mid360_d435
  -> /uavN/livox/lidar + /uavN/livox/imu
  -> FAST-LIO raw odom / registered cloud
  -> frame adapter + teammate/self/ground filter
  -> /uavN/Odometry + /uavN/stage3/cloud_registered_filtered
  -> EGO-Swarm /planning/bspline
  -> traj_server /planning/pos_cmd
  -> ego_gazebo_bridge /mavros/setpoint_raw/local
  -> MAVROS -> PX4 OFFBOARD -> Gazebo
```

任务层决定八扇区、候选、ENTRY/EXIT、返航与终态；EGO-Swarm 负责局部安全轨迹、重规划和共享 B-spline 避碰；bridge 是唯一 MAVROS raw-local 控制出口。Learning Speed 只改变 EGO 的 `v_max` 约束，不选择 waypoint、不生成轨迹、不绕过 EGO/PX4 安全门。

### 2.2 三机低空绕塔最终验收

最终控制闭环满足：UAV3 leader、UAV2 middle、UAV1 trailing，逆时针；UAV3 先放行，其后按前机领先约 65–70° 放行，目标相邻相位 67.5°。三机均完成独立 360° / 8 扇区、EXIT_GATE、按实际进场路线逆序返航、HOME、受控降落与解除武装；最小机间距离始终高于 3.0 m 门限，最终 `armed=false`、`ON_GROUND`、bridge/task/flight=`DONE`。

早期失败没有被删除：包括候选/地图边界、时效误判和外部授权中断。其后修复保持原 safety threshold，并以新的闭环证据取代“第五阶段 FAILED”的阶段状态。

## 3. Mid360 / 感知链问题与修复

### 3.1 worksite 启动问题

`worksite.world` 初始卡在约 `11934.010 s` 的首因不是 `<state>`、PX4/MAVROS、FAST-LIO、Observation 或三机 namespace，而是 `vrc_driving_terrain` 的 ODE heightfield collision 与 Mid360 自定义 20,000-ray ODE MultiRay 求交组合的病态性能。Gazebo physics 被首次重型射线求交阻塞，PX4 lockstep poll timeout 是下游症状。

最小修复：同一 129×129 高度图逐像素生成 16,641 顶点、32,768 三角形的 collision mesh，替换 collision 表示；visual、全部高度样本、其他模型、`samples=20000`、`lidar_downsample=1` 均保持。

| 修复后验证 | 结果 |
|---|---|
| 1 UAV 完整 Mid360 → FAST-LIO → filtered cloud → 无控制 EGO | PASS，filtered cloud 约 9.6–10.8 Hz，RTF 约 0.49–0.50。 |
| 2 UAV 完整链 | PASS，RTF 约 0.29。 |
| 3 UAV 完整链 | PASS，三机 filtered cloud 各约 8.16–8.32 Hz，RTF 约 0.23。 |

三机 RTF 仍是算力风险，不能从启动通过推出任意任务负载下性能充裕。

### 3.2 Observation v2：只读 Mid360 surrogate

v2 使用正式 `/uavN/stage3/cloud_registered_filtered` 与同 frame 的 FAST-LIO odometry：5 帧历史在各自采样时刻的 body frame 保存，再投影到当前 body；0.05 m voxel 融合；4.5° 全空间 partition（80 azimuth × 40 elevation = 3200 bins）；obstacle/free/unknown 三态和独立 mask。缺 pose、超出有界插值、frame/stamp 不一致、时间倒退均 fail closed。

| 已验证项 | 当前结论 |
|---|---|
| body-frame 历史对齐、平移/yaw ghost | 塔旁专项 76 个样本的 0.20 m overlap 为 1.0，未见双影。 |
| inward / CW / CCW 语义 | PASS；塔体在 inward bins，切向方向按真实结构呈 free/obstacle。 |
| tower-behind unknown | PASS；unknown 随位置/yaw 变化，未把无回波全写为 free。 |
| 控制影响 | 无。v2 只发布诊断/可视化，不发布 MAVROS 或 EGO 控制。 |

v2 的 `historical_fov_radial_sampling_v1_approximation` 是可审计 surrogate，不应声称与论文实现 bit-for-bit 一致；它也不是完整稠密 3D occupancy。

## 4. clearance / inflation 历史问题与语义清理

### 4.1 已关闭的历史问题

历史问题是把 EGO 已膨胀 occupancy 边界当作“真实障碍物距离”，或在已膨胀表示上再次套用 raw/coarse 的净空门限。膨胀本身是规划安全机制，不是错误；错误是未区分表示语义。

当前冻结规则：

| 表示 / 用途 | 有效 clearance 语义 |
|---|---|
| raw、filtered cloud、粗几何 | `minimum_clearance=1.0 m`，只应用一次。 |
| EGO 实际 inflated occupancy | `map_additional_clearance=0.5 m`。 |
| EGO 碰撞、A*、B-spline | 使用 inflated occupancy；任务层不把该距离写成 raw/真实距离。 |
| Observation / nearest / density / clutter | 由 filtered point returns / surrogate 计算；raw、filtered、inflated center distance 作为不同 diagnostics 保存。 |

`cloud_inflation` 的任务层重复加成已删除；`planningMapPointsAreInflated()` 按实际非空 representation 判断，避免 occupancy 为空时 fallback 到 raw/filtered 却沿用 inflated 语义。clean-up 回归为 302 tests、0 errors、0 failures。

### 4.2 PRE_ENTRY 0.967 m 事件：旧结论 → 修复 → 当前状态

| 环节 | 已有证据 |
|---|---|
| 旧拒绝 | ENTRY dispatch 对已膨胀 voxel 中心错误使用 1.0 m；目标到 voxel center 为 0.967340 m。 |
| 真实来源 | 最近 filtered raw point 约 1.62 m，来自真实 `Pine_Tree_6_clone` 回波；不是 terrain、self cloud、FAST-LIO 重影或历史残影。 |
| inflation 机制 | `resolution=0.25`、`obstacles_inflation=0.4` 产生 `ceil(0.4/0.25)=2` cells：XY ±0.50 m、Z ±0.25 m 的离散方盒。 |
| 修复 | dispatch 改为 representation-aware `mappedTaskClearance()`；不是降低 raw 1.0 m 阈值。 |
| 验证 | 此类 inflated map 上的 0.967 m 情况按 0.5 m 合法通过；后续 tower-side v2 运行实际进入塔旁。 |

### 4.3 不是 same-buffer double inflation，但存在 stacked margin

回归审计确认 EGO 点云模式仅有一次 occupancy voxel dilation；没有同一 buffer 的第二次 inflation。与此同时，`getInflateOccupancy()` 作为 hard collision boundary 后，optimizer `dist0=1.2` 仍提供方向性 soft clearance；任务层还在 inflated map 外使用 0.5 m operational margin。故：

- “inflated-as-real-distance”：**NO，已修复且未回归**；
- “同 buffer double inflation”：**NO**；
- “hard inflated boundary + directional `dist0` stacked margin”：**YES**；
- “任务层 0.5 m 运营边距”：**YES，但在 EGO optimizer 外**。

不得把这些直接相加为半径 `0.4 + 1.2 = 1.6 m`：inflation 是各向异性离散 voxel 方盒，`dist0` 又取决于碰撞 control point 的 direction。当前仍有配置可读性风险：任务 YAML 的 `dist0=0.8` 不控制 EGO；实际 EGO private namespace 在该栈中为 `dist0=1.2`。

## 5. Learning Speed Adapter 与 Observation C

### 5.1 动态 `v_max` Adapter 边界

```text
fixed/mock request
  -> SpeedSafetyFilter (clamp, hysteresis, slew, low-pass)
  -> /uavN/learning_speed/v_max
  -> EGO dynamic gate
  -> PlannerManager + BsplineOptimizer
  -> /uavN/learning_speed/applied_v_max acknowledgement
  -> 必要时连续重规划
```

Adapter 默认关闭，合法消息才更新 manager 和 optimizer；已发布旧轨迹不直接重采样，超过 `replan_delta` 且满足 cooldown 后从当前 p/v/a 状态重规划。它不是执行端紧急制动器，实际速度也不等于标量 `v_max`。

早期 Mock 高→低→高试验和轻量世界三机试验确认 request/filter/applied 与 EGO 重规划可逆响应；0.08 长保持会触发现有 ENTRY no-progress watchdog，故该试验只证明接口，不能当作完整 worksite 任务验收。

### 5.2 Observation C 冻结输入与因果要求

Observation C 将 v2、官方 EGO `/planning/bspline`、时间对齐 FAST-LIO 状态和已应用 `v_max` 融合为一个原子样本。唯一 policy input 为：

| 输入 | 来源与合同 |
|---|---|
| `lidar_surrogate[3200]` | v2 的 body-frame 三态 surrogate。 |
| `future_positions_body[20][3]` | 官方 B-spline De Boor 求值；默认距离采样、0.25 m 间隔、最大 5.0 m。 |
| `actual_velocity_body[3]` | timestamped FAST-LIO position 的因果差分与 `alpha=0.30` 低通。 |
| `tracking_error_body[3]` | 同时刻 official desired position − FAST-LIO actual position。 |
| `previous_applied_v_max` | observation stamp 时或之前的最近 applied acknowledgement。 |

mission/planner state、raw/inflated clearance、density、clutter、lidar masks 和 diagnostics 均是审计上下文，不进入冻结 policy input。任何缺少有效 pose/trajectory、future state、frame/stamp 不一致、非有限值或超出历史边界均输出 invalid；不以“最新数据”代替历史数据。

### 5.3 Observation C 数据质量闭环

首轮 12 run 的 38,337 条 Observation C 中，raw valid ratio 为 0.693，A/B aggregate 为 0.774/0.605。低 B 比例的主因不是 0.05 s 过严、FAST-LIO 缺失或 buffer 太短，而是大 sim time 下 `Header.stamp -> float seconds -> ROS Time` 往返稳定丢失 1 ns：本应 exact 的 FAST-LIO state 被误作 after state，约 0.10 s bracket 因而正确地 fail closed。

修复直接保留 ROS `sec/nsec`，维持 0.05 s 完整 bracket 上限，并让 receipt 使用 callback/producer/source stamp 的因果下界。环境 B 同源 causal replay 的 raw/training-active valid ratio 从 0.567677/0.659158 提升到 0.863591/0.995863；kinematic gap 与 trajectory timestamp mismatch 均为 0，无 unexplained invalid。环境 A post-fix 实飞为 0.867908/0.979209，亦无 kinematic gap/timestamp mismatch。结论：**Observation C data-quality gate PASS**，但 reward 仍为 null。

## 6. Training Data Contract

冻结 transition：

```text
state_t -> requested_v_max -> filtered_v_max -> applied_v_max
        -> state_t+1 -> reward -> terminated/truncated
```

采集器只在 Observation C 有效、正式 B-spline 可用且 action acknowledgement 符合因果顺序时关联候选 transition。真实 mission/planner/safety failure 必须保留；只有基础设施失败允许新 retry ID，且不得覆盖原尝试。`qualification_only=true` 数据不能混入正式标定统计。

当前边界没有改变：

```text
reward = null
reward_defined = false
training_ready = false
```

没有 SAC actor/critic、replay/PER、训练循环、已选模型，也没有定义 `φ1/φ2` 或权重。`inference/model_runner.py` 仅是未来 reviewed model 的 fail-closed 边界。

## 7. 固定速度 baseline 与数据代际

### 7.1 首轮正式 A/B baseline（1.50 m/s ceiling）

环境 A 是 `outdoor_village.world` 的开放路线，环境 B 是 `worksite.world` 的密集塔区；各环境内部路线指纹固定。0.30–1.50 m/s 共 12 条正式 run 全部 terminal、mission PASS、指标与 requested/filtered/applied 链完整。恢复后的 `planner:CANCELLED` 保留为真实结果，不覆盖 mission outcome。

| 环境 | 速度范围 | 任务结果 | 关键观察 |
|---|---:|---|---|
| A | 0.30–1.50 | 6/6 PASS | actual p95 0.232→1.035 m/s；tracking p95 0.233→0.694 m；高档出现恢复后的 planner CANCELLED。 |
| B | 0.30–1.50 | 6/6 PASS | actual p95 0.242→1.052 m/s；tracking p95 0.150→0.670 m；1.50 有 planner CANCELLED。 |

这批数据的旧 Observation C valid ratio（单次 0.545–0.834）是修复前数据质量事实，不应在后续分析中改写为 post-fix 结果。

### 7.2 post-fix 2.00 m/s ceiling 标定代际

post-fix 代际在 A/B 各固定路线执行 0.30–1.75 m/s，共 14 条 terminal formal run、40,564 条合并样本；与首轮 1.50 m/s ceiling 数据严格隔离。

| 环境 | 0.30–1.50 | 1.75 | Observation C（training-active / raw） |
|---|---|---|---|
| A | 全部 mission success | success | 0.9728–0.9856 / 0.8079–0.9318；无 unexplained invalid。 |
| B | 全部 mission success | **failure** | 0.9942–0.9994 / 0.7837–0.9625；无 kinematic gap、timestamp mismatch、causality 或 unexplained-invalid 回归。 |

`postfix_B_v175_r01` 是真实 mission failure，必须保留；它不被随后高速 qualification 的 1.75 PASS 覆盖，因为二者 ceiling、acceleration、bridge generation 不同。

## 8. 高速 tracking 根因与 bridge 修复

### 8.1 旧失败的根因

早期高速探索中，EGO 可产生约 1.5 m/s 级参考，而 `ego_gazebo_bridge` 仍以三维范数把 velocity 限制在 `0.5 m/s`、acceleration 限制在 `1.0 m/s²`。代表性 v200 数据中 planner 约 1.54 m/s、PX4 raw 约 0.50 m/s；velocity clamp 占比约 84.1%。PX4 `MPC_XY_VEL_MAX=12`、`MPC_ACC_HOR_MAX=5` 和姿态证据并不支持“PX4 是首要速度瓶颈”。

因此旧 `2.0 m/s` 两次 ENTRY tracking safety FAIL（tracking 超 1.0 m）应解释为 planner/bridge 不一致，而不是修复后平台的绝对速度上限。EGO 的逐轴约束、bridge 的三维范数约束、PX4 XY/Z 分离上限也不能只以相同数字判断一致。

### 8.2 修复原则与保留边界

高速专项改为令 bridge 使用覆盖 EGO 逐轴盒的 norm envelope，并在 preflight 显式核对 request、filter、dynamic gate、manager、optimizer、B-spline、bridge 与 live PX4 参数。`/planning/bspline` 发布约 1 Hz 不是低频控制：traj_server 连续求值并在约 100 Hz 输出 `pos_cmd`，bridge 约 50 Hz 发布 raw setpoint。

本次不通过降低 tracking gate、clearance、inflation、`dist0`、mission failure 或 PX4 安全门来制造 PASS。`bspline/limit_vel` 与 `bspline/limit_acc` 是展示型但未被 C++ physical-feasibility 读取的 rosparam，不再当作独立保护层。

## 9. CURRENT_POSITION_IN_OCCUPANCY 与 random fallback 排查

### 9.1 失败事实与证据边界

在旧 bridge-aligned 1.5 m/s B run 中，`CURRENT_POSITION_IN_OCCUPANCY` 不是空地图或 self-filter/长期残影问题：失败区有真实松树回波；0.25 m voxel、0.4 m inflation 使最近 inflated voxel 约 0.34 m；失败轨迹 9/10 的控制点最低约 1.150/1.168 m，而三个成功对照约为 3.0 m。任务目标仍为 3.0 m，但当时 EGO 的可规划高度仅受 1.0 m ground / 4.5 m ceiling 约束，没有整条轨迹的任务高度硬包络。

同参数的后续 qualification run 没有复现 mission-active occupancy 因果链并成功收尾，因此不能反推出“上一轮命中了哪一个具体 source point”。当前可确认的是机制范围，而不是精确单点复现。

### 9.2 random fallback 与垂直下潜的分类

最强可复核顺序是：两个确定性初始化 refine collision 失败 → 一次 random polynomial fallback 成功并发布 trajectory 9 → trajectory 9 深下潜 → trajectory 10 继承 → 靠近真实树/inflated voxel → current-position collision。

| 分类 | 结论 |
|---|---|
| 主因 | random initialization / fallback。random midpoint 可含对称的垂直扰动；本次近等高目标下其理论幅值足以引入大幅负 z。 |
| 次因 | inflated boundary + `dist0` collision cost；其存在已证实，但失败当次每个 base point/direction/gradient 未记录。 |
| 第三层 | optimizer 的 smoothness/terminal/feasibility 组合没有中段绝对高度 reference，可允许条件性 z drift。 |
| 未证实为直接主因 | A* guide 的精确路径；不能声称 A* 单独生成 1.15 m 轨迹。 |

这不是“全局负 z 常数漂移”或“random 必然向下”的结论。random 系数对零对称，但当 base/direction 已有 z 分量时，现有 cost 结构没有把中段拉回任务高度。该风险保持为 planner/高度契约风险；没有以修改 planner、world 或安全参数换取 qualification PASS。

## 10. 高速参数链：4.0 m/s / 3.0 m/s²

### 10.1 qualification 专用代际

为验证更高水平速度，UAV1 专用 high-speed generation 使用以下静态设置；常规三机默认配置未被改写。

| 层 | 值 / 语义 |
|---|---|
| Fixed requests | 1.75 / 2.0 / 2.5 / 3.0 / 3.5 m/s allowlist。 |
| filter、dynamic gate、manager、optimizer ceiling | 4.0 m/s。 |
| manager / optimizer acceleration | 3.0 m/s²，逐轴。 |
| feasibility tolerance | 0.0（qualification-only）。 |
| planning horizon | 7.5 m；只完成结构/运动学审计，非高速全域实证。 |
| bridge velocity norm | `sqrt(3) * (4.0 + 1e-4) = 6.928376... m/s`。 |
| bridge acceleration norm | `sqrt(3) * (3.0 + 1e-4) = 5.196326... m/s²`。 |

bridge envelope 高于 4.0 的意义是让它覆盖 EGO 的 XYZ box，而不是允许 EGO 以 6.93 m/s 规划。PX4 历史 ULog 的 `MPC_XY_VEL_MAX=12`、`MPC_ACC_HOR_MAX=5` 覆盖水平包络；`MPC_Z_VEL_MAX_UP/DN=3.0/1.5` 不覆盖完整 ±4.0 m/s XYZ box，故该 generation 只能解释为低空等高任务的水平 qualification，不能宣称完整 3D 4.0 能力。

### 10.2 静态结论如何被运行证据更新

参数链报告的静态 preflight、单测和 ROS 集成测试证明 request 不会在 adapter/EGO/bridge 的旧 0.5/1.0 限制处被隐式截断。后续实飞发现 bridge acceleration saturation：2.0 全程 22 次（0.516%）、2.5 全程 37 次（0.865%）；因此“静态无隐式截断”不能扩展成“运行时所有 acceleration 完全无截断”。该现象保留为瞬态 provenance，不改变其 velocity-chain 已闭环的结论。

## 11. 1.75 / 2.0 / 2.5 qualification 最终结果

环境为 B / `worksite` dense route；每档一次，真实 failure 即停，均为 `qualification_only=true`。3.0、3.5 因 2.5 的真实安全失败未运行。

| configured `v_max` | qualification | actual mean / p95 / max (m/s) | tracking mean / p95 / max (m) | raw / training-active valid | 关键终态 |
|---:|---|---|---|---|---|
| 1.75 | **PASS** | 1.128 / 1.567 / 2.239 | 0.163 / 0.340 / 0.664 | 0.692 / 0.998 | mission 完成，ON_GROUND。 |
| 2.00 | **PASS** | 1.254 / 1.834 / 2.330 | 0.182 / 0.338 / 0.744 | 0.656 / 0.972 | mission 完成，ON_GROUND。 |
| 2.50 | **FAIL** | 1.452 / 2.041 / 2.527 | 0.268 / 0.535 / 3.150 | 0.571 / 0.836 | `CURRENT_POSITION_IN_OCCUPANCY` 锁存 collision/dangerous proxy；虽最终 mission success、disarm，qualification 仍 FAIL。 |
| 3.00 / 3.50 | NOT_RUN | — | — | — | 按 progressive-stop 规则不运行。 |

三档的飞行期 requested=filtered=applied=configured；unexplained invalid、trajectory timestamp mismatch、kinematic gap 均为 0。`trajectory_unavailable` 及少量 startup/pose-arrival lidar invalid 是明确 fail-closed 原因。

### 11.1 最终速度执行链审计

最新审计以 `/planning/pos_cmd → bridge raw → PX4 trajectory_setpoint → PX4 internal setpoint → actual velocity` 的原生时间戳对齐为准，2.5 仅分析第一次 collision 前的正常 `TRACK_EGO` 区间。

| configured | pos_cmd mean / p95 | bridge raw mean / p95 | PX4 actual mean / p95 | raw→PX4 输入最大差 |
|---:|---|---|---|---:|
| 1.75 | 1.113 / 1.790 | 1.113 / 1.790 | 1.105 / 1.653 | `5.9e-8 m/s` |
| 2.00 | 1.190 / 2.070 | 1.190 / 2.070 | 1.207 / 1.973 | `9.8e-8 m/s` |
| 2.50（collision 前） | 1.538 / 2.351 | 1.538 / 2.351 | 1.521 / 2.199 | `1.2e-7 m/s` |

结论为 **HIDDEN VELOCITY LIMIT: NO**：三档 velocity saturation 均为 0，bridge raw 与 PX4 input 对齐误差近似浮点误差；actual p95 随速度档上升，不存在 0.5/1.5/2.0 共同平台。多数任务样本未持续达到 90% configured（约 10.6% / 8.6% / 6.5% 的 pos_cmd 样本达到），原因是目标接近、转弯、加减速、重规划和制动；`v_max` 是逐轴可行性上界，不是恒定巡航目标。

2.0 的 22 次 acceleration saturation 对去除邻域后的 actual mean/p95 影响仅 +0.126%/+0.051%；2.5 的 failure 前为 0。因此 acceleration saturation 不是 sustained actual speed 低于 configured 的主因，但仍要保留为控制瞬态审计字段。

## 12. 当前最终冻结结论

| 项目 | 冻结结论 | 不应扩大的解释 |
|---|---|---|
| 三机 EGO-Swarm 低空巡塔 | SITL 全闭环 PASS。 | 不等于 outdoor 三机巡塔、真机或视觉业务闭环通过。 |
| worksite Mid360 启动 | heightfield collision 修复后 1/2/3 机感知链 PASS。 | RTF 约 0.23 的三机算力余量仍需关注。 |
| clearance 语义 | inflated-as-real 与 same-buffer double inflation 均已关闭。 | hard inflation + directional `dist0` stacked margin 仍存在且应明确表达。 |
| Observation v2 / C | v2 塔旁专项 PASS；C data-quality gate PASS，0.05 s fail-closed 保持。 | v2 不是控制输入；C PASS 不等于 reward/SAC 已就绪。 |
| Learning Speed | fixed/mock Adapter 到 EGO dynamic `v_max` 已验证，默认关闭。 | 它不是轨迹、避障或 PX4 控制器。 |
| baseline 数据 | 首轮 12 run 与 post-fix 14 run 均保留并分代。 | 不得混合 1.50-ceiling 与 2.00-ceiling 数据。 |
| 高速执行链 | 旧 0.5 bridge 瓶颈已排除；最新 velocity chain 无隐藏 ceiling。 | PX4 Z 速度限制仍不构成完整 3D 4.0 m/s 合同。 |
| qualification 上限 | 当前最高稳定 qualification：**2.0 m/s**。 | 2.5 mission 最终完成不等于 qualification PASS；3.0/3.5 未运行。 |
| planner 风险 | random fallback / 高度契约与 collision-proxy 机制已收窄。 | 失败当次 A* 具体路径和每项 gradient 仍未直接记录，不能伪称完全单点根因。 |

## 13. 当前项目状态与下一步：Stage 1 Reward → SAC

### 13.1 已关闭的问题

- worksite 的 Mid360/ODE heightfield 启动阻塞；
- PRE_ENTRY 在已膨胀 map 上重复使用 raw 1.0 m clearance；
- raw/真实距离与 inflated planner 距离的历史语义混用；
- Observation C 的 1 ns timestamp 假 gap、trajectory timestamp mismatch 与 unexplained invalid；
- 旧 high-speed planner/bridge 速度不一致与“PX4 隐藏速度 ceiling”假设。

### 13.2 保留风险与真实失败

- 2.5 m/s 的 `CURRENT_POSITION_IN_OCCUPANCY` / collision-proxy **FAIL** 是有效安全边界；不得以最终 `mission_success` 重标为 PASS。
- 高速 random fallback 的中段高度契约和诊断可观测性仍不足；A*、base point、direction、cost/gradient 分解未在失败 bag 完整记录。
- EGO 逐轴限制、bridge norm envelope、PX4 XY/Z 限制不是同一语义；尤其 PX4 vertical 上/下限不覆盖完整 4.0 m/s XYZ 盒。
- 2.0/2.5 全程出现少量 bridge acceleration saturation，虽非持续速度瓶颈，仍须保留在 reward 前的 failure/quality provenance 中。
- 三机 worksite 的 RTF 约 0.23；Observation v2 和三机 Outdoor 任务闭环、真机均未完成。

### 13.3 为什么现在可以进入 Stage 1 Reward 的设计/审计

进入条件来自已有证据，而不是删除失败：

1. `state_t → action → applied → state_t+1 → terminal` 的因果数据合同已经冻结；Observation C 对 source stamp、trajectory 与 previous applied action 均 fail closed，未使用未来数据。
2. 固定速度数据保存 actual speed、tracking、terminal、planner/collision provenance、障碍上下文与 action chain；真实失败没有被清洗为成功。
3. 高速速度执行链已审计到 PX4 原生时间戳层，确认 configured 与 sustained actual 的差异主要来自轨迹相位和跟踪动力学，不是隐藏的 bridge/PX4 speed clamp。
4. 2.5 FAIL、post-fix B/1.75 failure、planner recovery 和 acceleration saturation 都是可被 Stage 1 Reward 设计审计的真实边界输入，而非未解释的基础设施噪声。

这只授权**独立的 Reward 语义设计与审计**。在项目负责人进一步明确目标前，仍然禁止：

- 实现 reward；
- 定义 `φ1`、`φ2` 或任何 reward 权重；
- 启动 SAC 或训练/回放循环；
- 让未审查模型接入 `v_max` 控制链；
- 调低 clearance、tracking、collision、mission 或 qualification 门限来扩大通过速度。

## 来源索引

本汇总的正式来源是第 1.1 节列出的 17 份报告；运行工件来源是第 1.2 节列出的 10 组证据。引用优先级为：最新直接控制链/运行工件与正式报告末尾“最终结论”高于同一报告中的历史中间失败段落；旧结果仅用于说明演进、根因和代际，不覆盖当前冻结结论。
