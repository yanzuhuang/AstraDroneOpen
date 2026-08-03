# Stage 5 三机正式绕塔八边形轨迹任务层优化报告

日期：2026-08-03  
分支：`ego-swarm`  
基线 HEAD：`f68f07137db57698c15f39d093f812ff9851da15`  
优化前证据：`test_evidence/stage5_first_point_tier_fix_validation_20260803_012118/`  
最终成功证据：`test_evidence/stage5_octagon_optimization_validation_20260803_040000/`

## 1. 结论

本轮已经完成整圈任务层硬 Tier 和逆时针单调目标约束：

1. 后续七个巡塔扇区与第一巡塔点一样，严格按 `12.5 m -> 14.5 m -> 16.5 m` 逐层穷尽；半径不再参与跨 Tier 软评分竞争。
2. 每个 Tier 内先保留安全的已锁定目标，再优先名义角，最后才使用该 Tier 内既有评分；外层高净空候选不能跳过内层安全候选。
3. 同扇区重选、HOLD 恢复、bounded retry 和恢复路径均以实际累计逆时针进度为下界；相反方向恢复被明确禁止。
4. 八个主扇区和闭圈判定未改变；未增加中间圆弧点，未修改 world、角色顺序、ENTRY_GATE、机间阈值、`reverse_detected` 门限或 EGO/EGO-Swarm 核心。
5. 原 `worksite.world` 最终控制验证中三机均完成进场、八扇区、闭圈、EXIT_GATE、返航和落地；三机 `sector_mask=0xFF`、`reverse_detected=false`、最终 `DONE`、`armed=false`、`landed_state=1`。

任务层策略与安全/完成性验收通过；端到端“实际轨迹呈清晰规则八边形”的视觉验收仍未完全通过。UAV2 的八个任务主航点全部为对应的 12.5 m 点，但实际轨迹左侧仍存在明显内切和局部绕行。证据表明这不是 UAV2 候选 Tier 或航点顺序差异，而与相位调速、HOLD 后 EGO 恢复以及同一目标下的高频 trajectory 更新相关。按本轮边界没有修改 EGO/EGO-Swarm 核心，建议下一轮做专项审计。

## 2. 修改文件

- `AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/include/astra_tower_mission/stage3_planner.h`
  - 声明整圈硬 Tier 选择器和有向累计角辅助函数。
- `AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/src/stage3_planner.cpp`
  - 实现逐半径 Tier 穷尽；实现跨 0° 的 CW/CCW 有向目标进度和闭圈锚点判定。
- `AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/src/stage3_ego_mission_node.cpp`
  - 记录当前和历史最远有向进度；过滤后方候选；低空正式绕塔使用硬 Tier；输出低 Tier 完整拒绝构成；恢复只允许配置方向且恢复序列必须单调前进。
- `AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/test/stage3_planner_test.cpp`
  - 增加内层胜过高分外层、逐层回退、软走廊偏好不得越 Tier、跨 0° 单调进度四项回归。
- `scripts/tool/analyze_stage5_octagon.py`
  - 用统一正式窗口和公共 world 坐标计算优化前后指标，生成同尺度对比图、三机合并诊断图和三张单机诊断图。

没有修改 `worksite.world`、配置中的安全阈值、EGO/EGO-Swarm vendor 源码或外部 PX4。

## 3. 整圈硬 Tier 实现

对每个后续巡塔扇区，候选半径由扇区名义半径 12.5 m 和 2.0 m 步长映射为：

| Tier | 半径 | 进入条件 |
|---:|---:|---|
| 0 | 12.5 m | 始终先完整检查；包含名义角和扇区内调角 |
| 1 | 14.5 m | Tier 0 没有任何安全候选 |
| 2 | 16.5 m | Tier 0、1 均没有任何安全候选 |

选择器只在当前第一个“存在安全解”的 Tier 内比较候选。当前锁定目标仍安全时优先保持，避免 HOLD/map refresh 后无故换点；没有安全锁时优先精确名义角，再使用原有级内评分和稳定 ID 裁决。`prefer_clear_straight_corridor` 也只在当前 Tier 内生效。

运行日志为每次外扩输出 `[STAGE5_ORBIT_TIER]`，包括选中 Tier、世界角、半径、候选 ID、实际最远进度，以及每个低 Tier 的候选总数和逐原因拒绝计数。

## 4. 逆时针单调前进

任务节点从首巡塔点记录 `orbit_start_angle`，每个定位样本按配置方向展开到有向累计角；同时保存历史最远有向角。对新主目标和重选候选执行以下硬检查：

- 普通扇区目标的有向角不得小于历史最远实际进度；
- 闭圈首点被映射到 `2π` 而不是 0，避免把合法闭圈误判为回头；
- 同扇区 12.5/14.5/16.5 m 调角使用同一进度下界；
- HOLD 恢复和 fresh goal 维持原目标 ID；仅在原目标经过 bounded retry 被确认不可达后，才在同一扇区重新评价；
- 任务层 R1/R2/re-entry 恢复只评价配置的逆时针方向，三个恢复点逐点检查不后退；相反方向即使评分更高也不能采用。

八扇区访问序列、`sector_mask=0xFF` 和回到记录首点的完整一圈判定保持不变。最终运行的闭圈结果为 UAV1/UAV2/UAV3 = 370.925°/375.401°/365.340°，三机均 `reverse_detected=false`。

## 5. 测试与短时检查

- 白名单构建 `astra_tower_mission;astra_swarm_manager;astra_swarm_bringup`：通过。
- `stage3_planner_test`：63/63，通过；本报告完成前再次直接运行确认。
- `ego_task_utils_test` + `tower_route_test`：10/10 + 8/8，通过。
- `astra_swarm_manager`：43/43，通过。
- `stage3_no_control_integration.test`：1/1，通过。
- `bash -n scripts/run_sh/stage5_three_uav.sh`：通过。
- 分析器 `py_compile`、本轮文件 `git diff --check`：通过。

第一次在只读 `~/.ros` 和受限本地 ROS 网络环境调用短时集成测试时产生了一个保留的 `MISSING-rostest-test_stage3_no_control_integration.xml`；随后在 `/tmp` 可写 ROS 环境运行同一测试为 1/1。该标记按“不清理证据”要求保留，不是测试逻辑失败。

## 6. 完整控制验证和保留证据

### 6.1 运行记录

1. `test_evidence/stage5_octagon_optimization_validation_20260803_024645/`
   - 沙箱内首次启动在 ROS 网络接口权限处失败，未解锁、未飞行；证据保留。
2. `test_evidence/stage5_octagon_optimization_validation_20260803_024800/`
   - 完成进场并进入后半圈；UAV2 第 8 扇区已先生成并执行 12.5 m 新轨迹，随后 EGO 报告 `CURRENT_POSITION_IN_OCCUPANCY`，实际轨迹明显回撤。
   - 任务层没有选择后方点，依次穷尽前方 12.5/14.5/16.5 m；最终 21 个候选构成为 `NON_MONOTONIC_ORBIT_TARGET:6 | OCCUPANCY_OR_CLEARANCE:4 | PLANNER_UNREACHABLE:11`，安全失败降落。
   - UAV2 已落地后有序停止该次无效验收运行；4.45 GB bag、CSV/JSON/PNG/日志均保留。
3. `test_evidence/stage5_octagon_optimization_validation_20260803_040000/`
   - 使用完全相同的 world、代码、参数和安全阈值重新运行；三机全部完成。
   - bag 已封存为 `stage5.bag`，大小约 12.39 GB；不存在 `.active`。

失败样本说明 `CURRENT_POSITION_IN_OCCUPANCY` 会使同一目标后续 EGO 生成持续失败；本轮没有通过后方目标、降低净空或修改核心规划器绕过该安全状态。

### 6.2 八个最终主航点

角度和半径均在公共 `worksite.world` ENU 坐标中相对 `radio_tower=(-10.0551,19.7104)` 计算。表中 S1～S8 为访问顺序；候选 ID 保留节点内部 0 基扇区编号。

#### UAV1

| 顺序 | 扇区 | 角度 | 半径 / m | 候选 ID |
|---:|---:|---:|---:|---|
| 1 | S1 | 280.5° | 12.5 | `l0_s0_c15`（联合 ID `U1_G1_A2_P0_E3_Ol0_s0_c15`） |
| 2 | S2 | 337.5° | 12.5 | `l0_s1_c0` |
| 3 | S3 | 22.5° | 12.5 | `l0_s2_c0` |
| 4 | S4 | 67.5° | 12.5 | `l0_s3_c0` |
| 5 | S5 | 112.5° | 12.5 | `l0_s4_c0` |
| 6 | S6 | 167.5° | 14.5 | `l0_s5_c13` |
| 7 | S7 | 202.5° | 12.5 | `l0_s6_c0` |
| 8 | S8 | 237.5° | 12.5 | `l0_s7_c9` |

计数：12.5 m = 7，14.5 m = 1，16.5 m = 0。

#### UAV2

| 顺序 | 扇区 | 角度 | 半径 / m | 候选 ID |
|---:|---:|---:|---:|---|
| 1 | S1 | 310.0° | 12.5 | `l0_s0_c3`（联合 ID `U2_G1_A0_P0_E4_Ol0_s0_c3`） |
| 2 | S2 | 355.0° | 12.5 | `l0_s1_c3` |
| 3 | S3 | 45.0° | 12.5 | `l0_s2_c0` |
| 4 | S4 | 90.0° | 12.5 | `l0_s3_c0` |
| 5 | S5 | 135.0° | 12.5 | `l0_s4_c0` |
| 6 | S6 | 175.0° | 12.5 | `l0_s5_c3` |
| 7 | S7 | 225.0° | 12.5 | `l0_s6_c0` |
| 8 | S8 | 270.0° | 12.5 | `l0_s7_c0` |

计数：12.5 m = 8，14.5 m = 0，16.5 m = 0。

#### UAV3

| 顺序 | 扇区 | 角度 | 半径 / m | 候选 ID |
|---:|---:|---:|---:|---|
| 1 | S1 | 337.5° | 12.5 | `l0_s0_c0`（联合 ID `U3_G1_A3_P4_E4_Ol0_s0_c0`） |
| 2 | S2 | 22.5° | 12.5 | `l0_s1_c0` |
| 3 | S3 | 67.5° | 12.5 | `l0_s2_c0` |
| 4 | S4 | 112.5° | 12.5 | `l0_s3_c0` |
| 5 | S5 | 162.5° | 14.5 | `l0_s4_c7` |
| 6 | S6 | 202.5° | 12.5 | `l0_s5_c0` |
| 7 | S7 | 237.5° | 12.5 | `l0_s6_c9` |
| 8 | S8 | 282.5° | 12.5 | `l0_s7_c9` |

计数：12.5 m = 7，14.5 m = 1，16.5 m = 0。

最终运行后又将每次选择日志与当时的 `candidate_targets` 快照逐项关联。所有后续调角点的同 Tier 名义候选都确有硬拒绝：UAV2 S2/S6 为 `KNOWN_OBSTACLE_CLEARANCE`，UAV1 S8 和 UAV3 S7/S8 为 `OCCUPANCY_OR_CLEARANCE`；其余名义候选一旦安全均选择 `c0`。因此本轮没有把安全的同半径名义点交给普通软评分淘汰。

### 6.3 每次外扩的确切原因

| UAV / 主点 | 最终回退 | 低 Tier 完整穷尽结果 | 同 Tier 后续 |
|---|---|---|---|
| UAV3 / S5 | 14.5 m `l0_s4_c7` | 12.5 m 共 7 个：`PLANNER_UNREACHABLE:1`、`KNOWN_OBSTACLE_CLEARANCE:6` | 12.5 m `l0_s4_c18` 发生 `NO_FEASIBLE_TRAJECTORY` 和一次同目标 bounded retry/CANCELLED，之后才进入 14.5 m |
| UAV1 / S6 | 14.5 m `l0_s5_c13` | 12.5 m 共 7 个：`PLANNER_UNREACHABLE:1`、`KNOWN_OBSTACLE_CLEARANCE:6` | 14.5 m `l0_s5_c7` 发生 `NO_FEASIBLE_TRAJECTORY` 后，在同一 Tier、同一扇区前向重选为 `l0_s5_c13` |

没有 16.5 m 外扩。所有外扩均有安全原因；没有因普通评分选择外层点。

## 7. 优化前后统一统计

正式窗口统一为每架飞机自身首次 `ORBIT_RELEASE` 到首次退出状态之前；位置统一转换到 world ENU。优化前基线也由同一分析器重新计算。半径样本为 10 Hz；`12.5±0.5 m` 即 12.0～13.0 m。

### 7.1 实际正式绕塔半径

| UAV | 版本 | min / m | max / m | mean / m | σ / m | 12.5±0.5 m 占比 |
|---|---|---:|---:|---:|---:|---:|
| UAV1 | 优化前 | 9.848 | 16.374 | 12.739 | 1.709 | 24.20% |
| UAV1 | 本轮 | 9.288 | 16.180 | 12.203 | 1.297 | 35.35% |
| UAV2 | 优化前 | 8.202 | 13.977 | 11.479 | 1.031 | 27.79% |
| UAV2 | 本轮 | 7.964 | 13.841 | 11.480 | 1.057 | 29.06% |
| UAV3 | 优化前 | 9.925 | 16.939 | 12.862 | 1.783 | 28.89% |
| UAV3 | 本轮 | 8.974 | 15.285 | 12.029 | 1.179 | 37.34% |

UAV1/UAV3 的均值、标准差和目标半径占比明显改善，且最终主点不再使用 16.5 m；UAV2 的实际均值几乎不变，说明其内切不是半径 Tier 选择造成的。

### 7.2 小回头

统一口径：10 Hz world XY，7 点居中平滑后展开逆时针累计角；从局部峰值回撤至少 0.1°计一次事件。最大反向距离为该回撤峰值至谷值期间实际 10 Hz XY 路程，优化前后完全使用同一脚本。

| UAV | 版本 | 次数 | 最大回撤角 | 最大反向距离 / m |
|---|---|---:|---:|---:|
| UAV1 | 优化前 | 7 | 0.590° | 0.434 |
| UAV1 | 本轮 | 8 | 0.662° | 0.391 |
| UAV2 | 优化前 | 6 | 0.756° | 1.208 |
| UAV2 | 本轮 | 13 | 0.731° | 0.900 |
| UAV3 | 优化前 | 5 | 0.469° | 0.654 |
| UAV3 | 本轮 | 4 | 1.231° | 1.093 |

所有事件仍未触发既有单步 -5° `reverse_detected` 门限。任务目标没有后退，但实际小回头次数没有对三机一致改善；尤其 UAV2 次数增加、UAV3 最大单次回撤增加。这也是整体视觉验收不能判定完全通过的原因之一。

### 7.3 HOLD、bounded retry 和无解事件

| UAV | 正式 HOLD 区间 / s | 触发原因 |
|---|---|---|
| UAV1 | 12456.5～12460.7 | 相位下界链式 HOLD；恢复重发同一 `l0_s2_c0` |
| UAV1 | 12555.8～12578.6 | 相位下界链式 HOLD；恢复重发同一 `l0_s3_c0` |
| UAV1 | 12714.5～12716.4；12716.6～12718.5 | `l0_s5_c18` 的 `NO_FEASIBLE_TRAJECTORY`，随后同目标 bounded retry/CANCELLED |
| UAV1 | 12731.4～12733.4；12733.6～12735.6 | 14.5 m `l0_s5_c7` 的 `NO_FEASIBLE_TRAJECTORY`，随后同目标 bounded retry/CANCELLED |
| UAV1 | 12777.0～12778.9 | 最终 `l0_s5_c13` 暂时无进度；同目标 bounded retry 后完成 |
| UAV2 | 12456.5～12460.7 | UAV3 规划失败引起的相位链式 HOLD；目标未改变 |
| UAV2 | 12555.8～12578.5 | 相位下界 HOLD；目标未改变 |
| UAV3 | 12456.4～12458.3；12458.5～12460.4 | 12.5 m `l0_s4_c18` 的 `NO_FEASIBLE_TRAJECTORY`，同目标 bounded retry/CANCELLED 后进入下一 Tier |

HOLD 没有被删除或缩短安全门；fresh goal 均保持原目标。最终运行没有出现返回上一扇区、HOLD 后目标回退或同扇区前后跳转。

### 7.4 trajectory_id、相位调速和机间距离

| UAV | 正式窗口 / s | trajectory_id 首→末 | ID 切换 | speed scale min/mean/max |
|---|---|---|---:|---|
| UAV1 | 12303.5～13076.7 | 96→769 | 665 | 0.000 / 0.926 / 1.000 |
| UAV2 | 12244.0～13011.0 | 103→772 | 666 | 0.000 / 0.939 / 1.000 |
| UAV3 | 12179.4～12903.8 | 167→831 | 652 | 1.000 / 1.000 / 1.000 |

UAV2 正式窗口 speed scale 分布：全速 86.18%，部分相位调速 10.38%，0 速/HOLD 3.44%。按其八个最终目标归属的同目标 trajectory_id 更新数依次为 77、70、81、76、76、96、96、93，共 665 次；另有一次目标边界切换。因此 UAV2 的明显实际轨迹差异发生在主目标基本对应的前提下，并伴随高频局部重规划。

三机共同正式绕塔窗口的最小 3D 距离：

| 机对 | 最小 3D 距离 / m |
|---|---:|
| UAV1–UAV2 | 9.333 |
| UAV1–UAV3 | 18.314 |
| UAV2–UAV3 | 10.081 |

总体最小值 9.333 m，未降低或触发机间安全阈值。

### 7.5 完成性

| UAV | 八扇区 | 完整一圈 | EXIT_GATE | 返航 | 落地终态 |
|---|---|---|---|---|---|
| UAV1 | `0xFF` | 370.925° | 完成 | 完成 | `DONE`, disarmed, ON_GROUND |
| UAV2 | `0xFF` | 375.401° | 完成 | 完成 | `DONE`, disarmed, ON_GROUND |
| UAV3 | `0xFF` | 365.340° | 完成 | 完成 | `DONE`, disarmed, ON_GROUND |

## 8. 图表

- 优化前后同尺度 XY 对比：`test_evidence/stage5_octagon_optimization_validation_20260803_040000/octagon_analysis/before_after_xy_same_scale.png`
- 三机合并诊断图：`test_evidence/stage5_octagon_optimization_validation_20260803_040000/octagon_analysis/three_uav_octagon_diagnostics.png`
- 单机诊断图：
  - `uav1_octagon_diagnostic.png`
  - `uav2_octagon_diagnostic.png`
  - `uav3_octagon_diagnostic.png`
- 完整机器可读统计：`octagon_analysis.json`

所有图使用相同塔心、相同 world ENU 坐标、相同 X/Y 范围和 1:1 比例。单机诊断图在实际 XY 上叠加真正下发的八个最终主航点，并标注访问序号、扇区、角度、半径、候选 ID、HOLD、bounded retry 和 `NO_FEASIBLE_TRAJECTORY` 位置。

## 9. UAV2 专项判断

UAV2 八个主航点全部为 12.5 m，角度按其角色相位依次为 310°、355°、45°、90°、135°、175°、225°、270°。与 UAV1/UAV3 的角色相位差和安全调角相比，UAV2 没有异常外层候选、后方候选或扇区乱序；反而 UAV1/UAV3 各有一个有明确安全原因的 14.5 m 点。

UAV2 实际轨迹仍明显不同：其平均半径 11.480 m，最低 7.964 m，12.5±0.5 m 占比仅 29.06%；13 次小回头主要分布在持续相位调速和每个主目标 70～96 次 EGO trajectory 更新期间。两次正式 HOLD 都是相位链触发，恢复后目标未改变。第一轮失败样本还显示同一第 8 目标先成功生成轨迹，随后 `CURRENT_POSITION_IN_OCCUPANCY` 导致后续规划全部失败。

因此，UAV2 的剩余问题不属于本轮整圈硬 Tier 或单向航点规则可继续修复的任务层候选差异。下一轮应专项审计：

1. UAV2 各目标内 trajectory_id 高频更新与实际曲率/半径偏差的时序关系；
2. 相位 speed scale 变化是否使 EGO 在同一局部目标上反复生成不同拓扑轨迹；
3. HOLD cancel/resume 后 traj_server/EGO 状态、旧 B-spline 清理和新 goal generation 对齐；
4. `CURRENT_POSITION_IN_OCCUPANCY` 在 UAV2 第 8 扇区的点云、膨胀地图和自身位置时序。

该专项审计可以先只读分析 bag/状态/轨迹，不应在没有证据前修改 EGO/EGO-Swarm 核心。

## 10. 最终验收判断和遗留问题

- **整圈硬 Tier：通过。** 已证明 12.5 m 完整穷尽后才外扩；无软权重越级。
- **逆时针八扇区顺序：通过。** 三机目标和恢复均未后退，闭圈顺序及 `0xFF` 不变。
- **安全与完成性：通过。** 三机完成全流程并落地，机间最小 9.333 m。
- **任务层引起的小回头：通过限定范围检查。** 未发现候选池重建选后方、fresh goal 不一致、累计角丢失或相位恢复目标回退。
- **实际轨迹规则八边形视觉终验：未完全通过。** 主航点八边形清晰，但 UAV2 实际轨迹仍有明显内切，三机小回头指标也未一致改善。

必须继续的问题是 UAV2 的 EGO/相位/HOLD 同目标轨迹更新专项审计；不建议在本轮任务层规则中继续增加航点、改圆弧、放宽安全标准或修改核心规划器。

本轮未清理任何 bag、JSON、CSV、PNG 或失败样本。`AstraDrone_ros1_ws/src/SLAM/FAST_LIO/Log/mat_pre.txt` 是任务开始前即存在并在仿真中继续变化的受保护运行副产物，不属于本轮源码修改，不应提交。
