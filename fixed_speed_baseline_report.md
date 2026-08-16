# Mission completion 最小修复与 Fixed-Speed Baseline 报告

## 审计边界

- 分支：`scene01-3uav-circuit-mission`
- 审计/实现起点 HEAD：`e7e17986042ffd7000523d4cde1fcda3afbae594`
- 工作区起点已有 Observation v2/C、clearance、world/model 与
  `FAST_LIO/Log/mat_pre.txt` 等未提交修改；本任务保留这些用户资产。
- 未修改 EGO、FAST-LIO、PX4、waypoint、ENTRY_GATE、EXIT_GATE 或安全阈值；
  未训练 SAC，未定义 reward 或权重。

## Mission bookkeeping 修复

根因是 `updateOrbitProgress()` 受 `orbit_released_latched_` 门控，而该锁存过去
主要由 `WAIT_ORBIT_PERMISSION` 的授权路径置位。单机
`orbit_permission_enabled=false` 会正确绕过协调等待，却也意外绕过独立进度记账。

最小修复把两个事件明确分开：

- permission ON：仍只在既有 individual ORBIT_RELEASE 被接受后启动记账；
- permission OFF：到达同一个首个正式巡检点时启动记账，仅更新锁存与进度，
  不增加状态、goal、HOLD、轨迹或 waypoint。

新增四个 latched `std_msgs/Bool`：`mission_success`、`mission_failure`、
`mission_done`、`orbit_complete`。`DONE` 只有在无失败锁存且 orbit complete 时才是
success；安全返航/落地不会抹掉先前任务失败。

测试覆盖 permission ON/OFF 释放条件、成功、失败与不完整 terminal 组合。真实
permission-OFF run `v020_r02` 得到 `accumulated=365.387deg`、
`sector_mask=0xff`、`start_error=3.54313deg`、`reverse_detected=false`，并完成原
EXIT_GATE、返航、落地、disarm/ON_GROUND，四信号为 true/false/true/true。

## 速度范围与档位依据

`uav_tower_stack.launch` 的通用缺省仍是 0.30 m/s，但当前正式
`triple_tower_inspection`/单机 baseline 明确覆盖为 0.20 m/s；实际 run 参数展开确认
EGO manager 与 optimizer 均为 0.20 m/s，dynamic limit 为 0.05--0.20 m/s。
当前任务 entry nominal/stable ceiling 是 0.20 m/s，SpeedSafetyFilter 也被正式 launch
约束为 0.05--0.20 m/s。既有 Mock 飞行覆盖 0.08、0.12、0.20 m/s；0.08 长保持
曾与三机任务的 no-progress 时标不兼容，0.12/0.20 有链路证据。因此选取：

- 0.08 m/s：低区诊断档；
- 0.12 m/s：低中档及既有 Mock 证据点；
- 0.16 m/s：中高插值档；
- 0.20 m/s：当前静态 EGO 与工程安全上限。

不测试低于 0.05 或高于 0.20 m/s，也不为某档成功而修改安全条件。

## Baseline 实现与场景控制

`FixedSpeedPolicy` 是不可变 source；adapter 的 `fixed` 与 `mock` 模式互斥，且未
提供 RL backend。正式链路为：

```text
fixed v_max -> SpeedSafetyFilter -> /uav1/learning_speed/v_max
            -> EGO dynamic limit -> /uav1/learning_speed/applied_v_max
```

每次重启同一 UAV1 worksite/PX4/FAST-LIO/EGO/mission/Observation C 栈，静态
EGO ceiling 固定 0.20 m/s，只改变 fixed source。随机性来自 Gazebo physics/
scheduler、sensor timing 和插件噪声；当前没有统一 per-run seed 接口。

## 指标语义

- raw clearance：UAV 中心到 `/stage3/cloud_registered_filtered` 最近点；无膨胀，
  未减机体半径，不是表面净空。
- inflated clearance：UAV 中心到 `/stage3/occupancy_inflate` 最近已膨胀 voxel
  中心；不是 raw clearance，也不是物理表面净空。
- collision：`PlannerStatus.current_position_in_collision` 相对 EGO inflated map
  的占据代理；goal-in-occupancy 单列。Gazebo 物理 contact 当前未仪器化。
- planning latency：mission `current_target` 回调到该目标后首条新 Bspline 回调；
  EGO optimizer 内部耗时未暴露，不能声称为 optimizer latency。
- emergency trajectory：当前可观测代理是
  `PlannerStatus.emergency_stop_active`（EGO `EMERGENCY_STOP` 状态）；
  `EMERGENCY_STOP_TIMEOUT` 另列为持续超时，接口没有独立 trajectory-class 字段。
- actual velocity：Observation C 的 timestamp-aligned pose finite difference；
  FAST-LIO `Odometry.twist` 当前为零，不可作为实际速度或 reward 信号。

## 重复实验结果

机器可读权威结果位于 `runtime_artifacts/fixed_speed_baseline/`。本节由完成后的
`baseline_runs.csv`、`baseline_summary.csv/json` 填充。非 terminal 的基础设施
中断 run 会保留在 run ledger，但从速度汇总排除；真实 terminal failure 仍计入。

<!-- BASELINE_RESULTS_START -->

正式 run ID 为 `v008_r01..r03`、`v012_r01..r03`、`v016_r01..r03`
和 `v020_r02..r04`，每档 3 次，共 12 次 terminal run。`v020_r01` 因基础
设施中断未得到 terminal outcome，保留在 run ledger 中但不进入任何速度
汇总。每次都记录独立 run ID；随机性主要来自 Gazebo physics/scheduler、
sensor timing 和插件噪声。

### Mission 与 safety

| fixed `v_max` (m/s) | runs | success | orbit complete | terminal duration mean / p95 (s) | successful completion mean / p95 (s) | min raw (m) | min inflated-center (m) | collision proxy / non-return | emergency |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.08 | 3 | 0/3 (0%) | 0/3 (0%) | 1839.566 / 1959.100 | -- / -- | 0.832 | 0.310 | 3/3 / 3/3 | 0/3 |
| 0.12 | 3 | 2/3 (66.7%) | 3/3 (100%) | 1536.867 / 1565.650 | 1520.376 / 1527.104 | 0.801 | 0.175 | 3/3 / 1/3 | 0/3 |
| 0.16 | 3 | 2/3 (66.7%) | 2/3 (66.7%) | 1094.598 / 1144.974 | 1141.472 / 1145.412 | 0.426 | 0.108 | 3/3 / 1/3 | 0/3 |
| 0.20 | 3 | 2/3 (66.7%) | 3/3 (100%) | 994.649 / 1042.779 | 966.925 / 975.902 | 0.826 | 0.102 | 3/3 / 1/3 | 0/3 |

`collision proxy` 不是 Gazebo contact：它表示 EGO inflated map 在当前位置报
occupied，包括 RETURN/降落阶段。因此同时给出排除 RETURN/降落后的 run
计数。当前没有可靠的 Gazebo 物理 contact 字段，不将该代理量误报为
机体碰撞。

### Tracking 与 velocity

下表的 mean/median/p95 是先对每个 run 统计，再在同档 3 次上取均值；
max 是该档所有 run 的最大值。

| `v_max` | tracking norm mean / median / p95 / max (m) | applied mean (m/s) | actual mean / median / p95 / max (m/s) | actual mean / applied | actual p95 / applied |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0.08 | 0.0915 / 0.0766 / 0.1703 / 2.8103 | 0.0800 | 0.0558 / 0.0532 / 0.0938 / 0.6266 | 69.8% | 117.3% |
| 0.12 | 0.0856 / 0.0743 / 0.1588 / 2.6755 | 0.1200 | 0.0737 / 0.0718 / 0.1213 / 0.5992 | 61.4% | 101.1% |
| 0.16 | 0.0949 / 0.0746 / 0.1536 / 2.8466 | 0.1600 | 0.0972 / 0.0970 / 0.1486 / 1.2014 | 60.7% | 92.9% |
| 0.20 | 0.0846 / 0.0752 / 0.1522 / 2.8337 | 0.2000 | 0.1156 / 0.1204 / 0.1734 / 0.6115 | 57.8% | 86.7% |

requested 与 applied 的档内均值差均在浮点误差范围（绝对值不超过
`5.6e-17 m/s`），说明 fixed source、filter 和 EGO applied 回执一致。actual
均值仅为 applied 的 57.8%--69.8%，因为整 episode 包含起飞、HOLD、进场、
返航和降落；转场/终止附近的有限差分尖峰也使 max 明显超过 applied，
所以 max 不可直接作 reward。

body xyz 为带符号的 body-frame tracking error，同样按“3 次 run 统计的
均值 / 中位数 / p95，以及全档最大值”表示：

| `v_max` | body x mean / median / p95 / max (m) | body y mean / median / p95 / max (m) | body z mean / median / p95 / max (m) |
| ---: | ---: | ---: | ---: |
| 0.08 | -0.0027 / -0.0032 / 0.0465 / 0.4724 | -0.0039 / -0.0037 / 0.0461 / 0.1454 | 0.0036 / -0.0128 / 0.1534 / 2.7950 |
| 0.12 | -0.0019 / -0.0042 / 0.0483 / 0.9338 | -0.0029 / -0.0040 / 0.0488 / 0.5124 | -0.0124 / -0.0211 / 0.1203 / 2.5545 |
| 0.16 | -0.0050 / -0.0049 / 0.0462 / 0.5872 | -0.0059 / -0.0054 / 0.0471 / 0.6053 | -0.0100 / -0.0275 / 0.1079 / 2.5630 |
| 0.20 | -0.0034 / -0.0042 / 0.0507 / 0.4196 | -0.0049 / -0.0053 / 0.0446 / 0.5682 | -0.0171 / -0.0230 / 0.1067 / 2.8294 |

### Planner 与 Observation C

| `v_max` | replans / replacements mean | goal-to-first-Bspline latency mean / p95 (s) | planner failure | Observation C valid | rate (Hz) | latency p95 (s) |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.08 | 1609.3 / 1609.3 | 0.0176 / 0.0275 | 3/3 | 67.71% | 10.074 | 0.1084 |
| 0.12 | 1300.7 / 1300.7 | 0.0056 / 0.0107 | 1/3 | 67.87% | 10.065 | 0.1084 |
| 0.16 | 939.3 / 939.3 | 0.0058 / 0.0120 | 1/3 | 67.16% | 10.065 | 0.1079 |
| 0.20 | 855.0 / 855.0 | 0.0065 / 0.0110 | 1/3 | 67.54% | 10.063 | 0.1085 |

replanning count 与 trajectory replacement count 都是新 Bspline ID 的代理计数，不是
optimizer 内部调用数。各档的主要 Observation C invalid reason 都是
`kinematic_interpolation_gap_too_large`（0.08/0.12/0.16/0.20 分别为
14172/11794/8443/7565 次），其次是 trajectory timestamp mismatch
（2950/2473/1906/1765 次）。valid ratio 没有随速度升高而先行崩溃，
但约 67% 的有效率不足以无过滤进入训练。

### 退化与稳定工作区

明确的性能退化首先出现在低速 `0.08 m/s`：3/3 mission failure、0/3 orbit
complete，与现有 mission/no-progress 时标耦合。`0.12--0.20 m/s` 每档都是
2/3 success 且无 emergency，是当前“相对可用区间”，但没有任何档达到
3/3 success，因此不把任何速度声称为已验收的稳定工作点。

在当前安全上限内没有观测到“高速开始显著退化”的统计拐点；
`0.20 m/s` 的 success rate 与 0.12/0.16 相同，但最小 inflated-center
distance 只有 0.102 m，且仍有 1 次 non-return collision/planner-failure
proxy，不应外推到更高速度。

<!-- BASELINE_RESULTS_END -->

## Training data contract 可获得性

可在 `ObservationC.valid=true` 时稳定关联的候选字段是：ROS timestamp、带 mask/
semantic 的 lidar surrogate、EGO future trajectory、body-frame actual velocity、
body-frame tracking error、previous applied v_max，以及独立的 mission/planner state。
terminal 后可把 success/failure/done/orbit_complete 关联为 episode outcome。

必须重新定义或过滤：Observation C invalid 样本；零值 FAST-LIO twist；raw 与
inflated clearance 的不同 representation；未仪器化的物理 contact；未暴露的 EGO
内部 planning latency；以及尚未定义 horizon/termination 的 causal next outcome。
这些字段本阶段只做可获得性审计，不构成 reward。

## 结论

<!-- FINAL_VERDICT_START -->

permission ON/OFF 都通过自动化测试；permission OFF 又由 12 次真实
UAV1 run 验证了记账会启动。permission ON 没有为本单机 baseline 额外执行
三机飞行，其原有 ORBIT_RELEASE 路径未改。修复只改变 bookkeeping 锁存
和 terminal 语义，不改变现有绕塔轨迹。

12 次正式运行、失败记录、速度链路回执、指标汇总和训练字段
审计都已产出。Baseline PASS 表示实验和数据合同预审计完成，不表示
所有速度的 mission 都成功。

```text
MISSION COMPLETION BOOKKEEPING: PASS
FIXED SPEED BASELINE: PASS
```

> 下一阶段可以正式建立 SAC training data contract，并基于 baseline 数据设计 reward；当前仍未开始 SAC 训练。

<!-- FINAL_VERDICT_END -->
