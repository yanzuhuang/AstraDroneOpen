# EGO-Swarm 第五阶段三机动态进场集成报告

日期：2026-08-02（含本轮规则恢复增量）

分支：`ego-swarm`

实施前 HEAD：`1275d3645805f5c5bca59204264b99dad571dd63`

## 验收结论

动态进场代码、构建、自动回归和真实三机控制复验已经完成；**完整绕塔闭环仍未通过**。

本轮已经消除旧固定点直接执行的问题。任务节点在三机 `HOVER_READY` 后从实时 EGO
占据图生成 PRE_ENTRY、ENTRY_GATE、第一巡塔点及完整走廊候选；管理器只有在三机候选
联合满足顺序、角间距、路径不交叉和预测轨迹净空时才发布选择。选择按 generation 和
candidate id 锁存，进入走廊前失效才要求 fresh generation；旧固定入口回退被明确禁止。

修正前的第四次代表性实飞中，UAV2 生成 54 条安全走廊，UAV3 生成 135 条；UAV1 生成 0 条。
UAV1 的 PRE_ENTRY 已不再固定在净空 0.980323 m 的旧点，但实时占据图证明：在
292.5°±7.5°、2.5°采样、精确 12.5 m 半径和 3 m 高度下，七个第一巡塔点均低于
1.0 m，最佳点只有 0.945260 m。因此管理器没有联合选择，也没有发布 ENTRY 许可。

任务在 `WAIT_ENTRY_PERMISSION` fail-closed，没有进入 ENTRY、依次绕塔放行或正式编队。
随后通过已有 bridge 的 supervised HOLD 落地接口按 UAV1→UAV2→UAV3 顺序收尾；最终
三机均 `armed=false`、`landed_state=ON_GROUND`、bridge=`DONE`。这只是安全终止，不是
任务正常 EXIT/返航/降落验收。

该次修正前复验没有降低 1.0 m 硬净空、3.0 m 机间距离或 EGO `swarm_clearance=1.5 m`，没有缩小障碍
包络，也没有修改 `worksite.world`、EGO vendor 核心、外部 PX4 或公共 TF/消息契约。

## 动态搜索与联合选择实现

每机搜索边界：

| 项目 | 配置 |
|---|---:|
| 名义角 | UAV3=337.5°，UAV2=315.0°，UAV1=292.5° |
| 角搜索 | ±7.5°，步长 2.5° |
| PRE_ENTRY | 18.0～19.5 m，步长 0.25 m |
| ENTRY_GATE | 15.0～16.0 m，步长 0.25 m |
| 第一巡塔点 | 12.5 m |
| 硬净空（修正前复验） | 1.0 m |
| EGO `dist0` / `swarm_clearance` | 1.2 m / 1.5 m，未修改 |

任务节点对每个候选检查三个端点、悬停点→PRE→ENTRY→第一巡塔点的完整路径、已知静态
障碍包络和实时占据图。直线不安全时使用任务侧固定高度 A* 检查可达路径；发布给管理器
的候选包含完整 `PATH_N` 折线，不只包含三个锚点。

管理器按固定角色 `[3,2,1]` 联合枚举三机候选，并执行：

- UAV3→UAV2→UAV1 的逆时针有向顺序；
- 相邻第一巡塔点 20°～30°，目标偏好 22.5°；
- 三条完整折线路径不交叉；
- 同步弧长采样的三机预测轨迹满足 3D 和 EGO 椭球净空；
- 排序依次为整条路径安全、三机最小净空最大、名义角偏差最小、名义半径偏差最小。

只有三机候选都新鲜且全局定位、心跳、轨迹、安全状态正常时才发布选择。选择锁存后，
普通地图帧变化不会使目标逐帧跳动；仅在尚未进入走廊且已选走廊重新检查失败时清空并
发布 fresh EGO 目标。HOLD 恢复路径不能回落到旧固定入口。

## 名义点、搜索结果与净空

名义点为 world 坐标：

| UAV | 名义角 | PRE 18 m | ENTRY 15 m | 第一巡塔点 12.5 m |
|---|---:|---|---|---|
| UAV3 | 337.5° | `(6.5747,12.8221,3)` | `(3.8031,13.9701,3)` | `(1.4934,14.9269,3)` |
| UAV2 | 315.0° | `(2.6728,6.9825,3)` | `(0.5515,9.1038,3)` | `(-1.2163,10.8716,3)` |
| UAV1 | 292.5° | `(-3.1668,3.0806,3)` | `(-4.3148,5.8522,3)` | `(-5.2716,8.1619,3)` |

第四次实飞 generation 1 的候选统计：

| UAV | 搜索组合 | 安全组合 | 发布候选 | 主要拒绝原因 |
|---|---:|---:|---:|---|
| UAV1 | 245 | 0 | 0 | PRE净空125，ENTRY净空24，第一巡塔点净空96 |
| UAV2 | 245 | 54 | 10 | PRE净空175，ENTRY净空14，A*起点占据2 |
| UAV3 | 245 | 135 | 18 | PRE净空10，全路径净空100 |

UAV1 12.5 m 第一巡塔环的同一实时占据图快照：

| 角度 | 点 `(x,y,z)` | 最近占据点距离 |
|---:|---|---:|
| 285.0° | `(-6.8199,7.6363,3.0)` | **0.945260 m** |
| 287.5° | `(-6.2963,7.7889,3.0)` | 0.431 m |
| 290.0° | `(-5.7798,7.9644,3.0)` | 0.133 m |
| 292.5° | `(-5.2716,8.1619,3.0)` | 0.113 m |
| 295.0° | `(-4.7724,8.3815,3.0)` | 0.106 m |
| 297.5° | `(-4.2832,8.6228,3.0)` | 0.095 m |
| 300.0° | `(-3.8051,8.8851,3.0)` | 0.270 m |

对285.0°～292.5°半窗的0.01°辅助探测显示该段最大值位于285.0°边界，仍只有
0.945260 m；需求规定的七个2.5°离散采样全部失败。与此同时，
PRE/ENTRY 的外移和换角确实能得到约1.2～1.24 m端点，这说明本轮旧0.98 m PRE问题
已被动态搜索绕开；当前无解项是用户要求保持12.5 m的UAV1第一巡塔点。

因为UAV1没有安全候选，`entry_corridor_selection={}`。本次没有“实际选择点”、放行时间
或67.5°编队数据；不能用UAV2/UAV3的单机候选冒充三机联合选择结果。

## 代表性实飞时间线

第四次实飞使用 headless Gazebo/PX4 SITL，仿真时钟如下：

| 时间 | 事件 |
|---:|---|
| 11949.304～11950.146 | 三机 armed，OFFBOARD同时起飞流程开始 |
| 11952.910～11953.710 | 三机起飞检测完成 |
| 11957.980～11958.830 | 三机全部到达 `HOVER_READY` |
| 11962.023 | UAV1动态搜索0/245，因第一巡塔点硬净空不足进入HOLD |
| 11962.090 | UAV2生成54条完整安全走廊 |
| 11962.824 | UAV3生成135条完整安全走廊 |
| 12006.321 | UAV1安全落地，bridge=`DONE` |
| 12034.156 | UAV2安全落地，bridge=`DONE` |
| 12052.431 | UAV3安全落地，bridge=`DONE` |

三机没有 `ENTRY_READY`、`MOVE_TO_ORBIT_STAGING`、`ORBIT_RELEASE` 或
`FORMATION_ORBIT_ACTIVE` 事件，所以65°～70°放行、67.5°—67.5°编队、完整一圈、
EXIT和参考进场路线返航均未执行。

## 机间距离与最终状态

本表覆盖同时起飞、悬停等待和串行安全落地，不是绕塔编队距离：

| Pair | 最小水平 | 最小三维 | 最小EGO椭球距离 |
|---|---:|---:|---:|
| UAV1-UAV2 | 3.964 m | 3.968 m | 3.968 m |
| UAV1-UAV3 | 7.882 m | 7.882 m | 7.882 m |
| UAV2-UAV3 | 3.866 m | 3.866 m | 3.866 m |

最终：UAV1/UAV2/UAV3 均 `armed=false`、`ON_GROUND`、bridge=`DONE`。三个绕塔累计角
均为0°，sector mask均为`0x00`，`orbit_release_times={}`。

## 构建与测试

| 项目 | 结果 |
|---|---|
| 目标包构建 | 通过 |
| Python静态编译 | 通过 |
| 管理器策略直接测试 | 35/35通过（含本轮 0.945260 m 已膨胀图净空回归） |
| catkin回归汇总 | **292 tests，0 errors，0 failures，0 skipped** |
| 第三次控制复验 | 固定点回退已禁止；UAV1零候选后保持HOLD；三机安全落地 |
| 第四次控制复验 | 端点分类证明确认为12.5 m首巡塔点无解；三机安全落地 |
| 完整第五阶段闭环 | **未通过，停止在WAIT_ENTRY_PERMISSION** |

一次组合测试命令额外请求了不存在的空包目标 `run_tests_astra_swarm_bringup`，因此该
make命令返回1；实际存在的四组目标均已执行，随后 `catkin_test_results` 汇总为上述
292/0/0/0。这不是测试用例失败。

## 证据与修改范围

代表性证据：

- `test_evidence/stage5_dynamic_entry_20260801_attempt4/roslaunch.log`
- `test_evidence/stage5_dynamic_entry_20260801_attempt4/swarm.csv`
- `test_evidence/stage5_dynamic_entry_20260801_attempt4/summary.json`
- `test_evidence/stage5_dynamic_entry_20260801_attempt4/uav1.csv`
- `test_evidence/stage5_dynamic_entry_20260801_attempt4/uav2.csv`
- `test_evidence/stage5_dynamic_entry_20260801_attempt4/uav3.csv`

核心修改位于：

- `astra_tower_mission/config/low_altitude_inspection.yaml`
- `astra_tower_mission/src/sector_inspection_mission_node.cpp`
- `astra_swarm_manager/src/astra_swarm_manager/policy.py`
- `astra_swarm_manager/scripts/swarm_manager_node.py`
- `astra_swarm_bringup/launch/uav_tower_stack.launch`
- `astra_swarm_bringup/scripts/swarm_evidence_recorder.py`

并保留前序依次放行、bridge HOLD锁存/速度缩放、RViz诊断等同一未提交工作区修改。
没有 commit 或 push。任务开始前已有的 `FAST_LIO/Log/mat_pre.txt` 始终按用户资产保留，
未手工编辑、恢复或清理；真实FAST-LIO进程会继续写入该运行日志，因此不纳入本轮代码成果。

## 未通过的硬约束

当前实时地图证据表明以下条件不能同时成立：

```text
UAV1 nominal=292.5°
angle window=[285.0°,300.0°]
angle sample step=2.5°
first orbit radius=12.5 m
height=3.0 m
occupied-map clearance>=1.0 m
```

因此在不修改world、不扩大角窗、不改变12.5 m半径/高度且不降低1.0 m净空的前提下，
不存在符合规定离散搜索的可锁存三机联合目标。实现保持fail-closed；完整闭环失败阶段为
`WAIT_ENTRY_PERMISSION / UAV1_ORBIT_STAGING_MAP_CLEARANCE`。

## 2026-08-02 规则恢复增量（本轮）

本节记录本轮恢复原单机扇区内自适应逻辑后的代码、测试和真实仿真结果；它补充并
覆盖上文“硬净空 1.0 m、第一巡塔点 12.5 m 固定搜索”的旧复验描述。旧复验是在本轮
修正前完成的，不能作为修正后语义的结论。

### 代码与参数

| 位置 | 规则 |
|---|---|
| `AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/config/low_altitude_inspection.yaml:146-152` | 共用角偏移 `[0,-5,+5,-10,+10,-12,+12] deg`、半径偏移 `[0,+2,+4] m`、低空高度偏移 `[0]`；`minimum_clearance=1.0 m` 仅用于原始点云/粗几何，`map_additional_clearance=0.5 m` 用于已膨胀占据图。 |
| `.../sector_inspection_mission_node.cpp:1627-1665` | `mappedTaskClearance()` 按地图是否已膨胀选择 0.5 m 或 `minimum_clearance + cloud_inflation`；EGO `dist0` 不作为任务硬门限。 |
| `.../sector_inspection_mission_node.cpp:1670-1782` | 端点、塔体 keep-out、粗几何和地图走廊分层检查；已膨胀地图统一使用 0.5 m 附加净空。 |
| `.../sector_inspection_mission_node.cpp:1784-1915` | 第一巡塔点与 PRE_ENTRY/ENTRY 组合共同枚举；每个组合再绑定同一普通扇区候选网格，评分后才交给 EGO。 |
| `.../sector_inspection_mission_node.cpp:2557-2660` | 初始扇区先评估全部候选再 `chooseBestCandidate()`；规划失败不会跳过同扇区候选，只有当前候选重试耗尽后才换候选/扇区。 |
| `.../sector_inspection_mission_node.cpp:2897` | 静态同扇区绕行也只使用 `[0,+2,+4] m` 半径偏移，不再另设向内半径规则。 |
| `AstraDrone_ros1_ws/src/Swarm/astra_swarm_manager/config/three_uav_inspection_formation.yaml:27-28` | `map_additional_clearance=0.5 m`；`optimizer_swarm_clearance=1.5 m` 保持不变。 |
| `.../astra_swarm_manager/policy.py:149-246` | 联合选择硬门限改为已膨胀地图 0.5 m；角度窗口 12°；机间最小三维距离 3.0 m 和 swarm clearance 1.5 m 保持不变。 |

因此 UAV1 在 285°、12.5 m 的实测地图距离 0.945260 m 现在可以进入候选评分/EGO 验证，
不会在规划前被错误的 1.0 m 已膨胀地图门限淘汰。候选仍须通过塔体、粗几何、路径和
三机联合安全检查；0.945260 m 并不等于自动放行。

### 验证结果

- 目标包构建通过。
- Catkin 回归：292 tests，0 errors，0 failures，0 skipped。
- Swarm Python 策略测试：35/35 通过，新增测试明确验证 0.945260 m 在 0.5 m 已膨胀图附加净空语义下可进入联合候选。
- 三机 dry-run 启动通过；三机均加载 3.0 m、逆时针、UAV3/UAV2/UAV1 角色及 337.5°/315°/292.5°名义角，日志显示 `map_additional_clearance=0.5` 和共用候选网格。
- 真实三机仿真已按现有 launch 启动观察 120 s，返回外部观察超时 `RC=124`，不是任务成功。任务未越过 `WAIT_INPUTS`：三台 bridge 持续 `HEARTBEAT_OR_LOCALIZATION_TIMEOUT`/`SAFETY_INHIBIT_HOLD`，并出现 UAV2 `spawn_model` 进程退出 1；没有产生 ENTRY、候选锁存、65°～70°依次放行、67.5°编队、完整绕塔、EXIT 或正常返航事件。随后仅执行安全 HOLD/落地收尾。

所以本轮真实闭环验收结论仍是“未通过，失败在定位/仿真输入就绪阶段”，不能声称已经完成三机绕塔。离线单机阶段3集成测试仍保留原有证据：名义候选规划不可达重试两次后进入 `HOLD`/`PLANNER_UNREACHABLE`，再切换下一同方向候选/扇区；本轮新增的同扇区候选选择逻辑已通过构建和单元回归，但尚无真实飞行候选切换记录。

## 2026-08-02 三机启动预检与受控闭环复验（最终结论）

本节取代上一节“停在 WAIT_INPUTS”的运行结论，但不删除历史失败证据。本次任务开始于
分支 `ego-swarm`、HEAD `1519a4220342f5d7e428c4ac589c8f60c374970d`，开始时工作区
干净。没有切分支、commit 或 push；没有修改 `worksite.world`、`rule.md`、EGO 核心或
外部 PX4。

### 启动预检

第一次无控制启动确认 `/gazebo/model_states` 内实体
`iris_mid360_00`、`iris_mid360_11`、`iris_mid360_22` 持续存在。UAV2 的
`spawn_model` 客户端虽等待超时，实体已经存在，因此没有重复 spawn。该次三机
MAVROS 均 connected 且 local pose 存在，但三机 Livox/FAST-LIO 无输出；原因是启动环境
没有把下层仿真工作区的 `liblivox_laser_simulation.so` 加入 `GAZEBO_PLUGIN_PATH`。

仅修复运行环境搜索路径并重新启动后，连续观察 8 s 通过：

| 检查项 | UAV1 | UAV2 | UAV3 |
|---|---|---|---|
| Gazebo实体 | `iris_mid360_00` | `iris_mid360_11` | `iris_mid360_22` |
| `/mavros/state.connected` | true | true | true |
| extended state / local pose | 持续更新 | 持续更新 | 持续更新 |
| FAST-LIO raw odom | 约8.0～8.3 Hz | 约8.0～8.3 Hz | 约8.0～8.3 Hz |
| raw/bridge odom与点云 | frame、stamp持续前进 | frame、stamp持续前进 | frame、stamp持续前进 |
| `/swarm/state` | 约10 Hz | 约10 Hz | 约10 Hz |
| `heartbeat_ok` / `localization_valid` | true / true | true / true | true / true |
| 无控制状态 | `DRY_RUN` / `WAIT_ENTRY_PERMISSION` | 同左 | 同左 |

观察窗口内 `/gazebo/model_states` 的短窗实测约1000 Hz、`/mavros/state` 约1 Hz；
更早的低负载短窗中 extended state 约1.75 Hz，FAST-LIO raw/adapted odom/cloud约2.93 Hz，
后续稳定复测 raw odom 达8.0～8.3 Hz。`SAFETY_INHIBIT` 约在仿真时刻11935.5解除，
三台 bridge 在控制启动后约11946离开 `WAIT_INPUTS`。精确的各话题最后一个
`header.stamp` 随临时运行目录被执行环境清理，无法作为持久证据补写；只保留“连续推进且
满足原 bridge 门限”的现场观察结论，不伪造具体数值。

### 最小修复

1. `sector_inspection_mission_node.cpp`：联合选择器锁定的第一巡塔点现在按实际坐标覆盖普通扇区
   名义点，而不再只在半径不同才覆盖。这样角度偏移相同半径候选也不会被旧名义点替换；
   14.5/16.5 m 只用于第一巡塔点，后续扇区仍回归12.5 m。
2. `swarm_safety_node.py`：不改超时阈值和 fail-closed 行为，仅把聚合的
   `HEARTBEAT_OR_LOCALIZATION_TIMEOUT` 拆为 `SWARM_STATE_MISSING`、
   `SWARM_STATE_TIMEOUT`、`HEARTBEAT_TIMEOUT`、`LOCALIZATION_TIMEOUT`，并记录状态年龄。

未关闭 `SAFETY_INHIBIT`，未放宽心跳、定位、轨迹时效，未写死健康状态，也未降低障碍物
或机间距离。

### 动态进场与依次放行证据

第三次受控复验中，联合选择与实际执行点一致：

| UAV | PRE_ENTRY | ENTRY_GATE | 第一巡塔点 | 角度/半径 | 已膨胀图净空 |
|---|---|---|---|---|---:|
| UAV3 | (0.54,13.85,3.0) | (-3.75,15.20,3.0) | (-6.50661,14.9269,3.0) | 337.5° / 12.5 m | 0.930 m |
| UAV2 | (-1.33,6.98,3.0) | (-3.45,9.10,3.0) | (-2.38784,8.04314,3.0) | 315.0° / 16.5 m | 0.990 m |
| UAV1 | (-1.05,2.41,3.0) | (-2.90,5.96,3.0) | (-3.74082,4.46639,3.0) | 292.5° / 16.5 m | 1.149 m |

联合候选ID分别为 `U3_G1_A2_P6_E0_Ol0_s0_c0`、
`U2_G1_A0_P0_E0_Ol0_s0_c2`、`U1_G1_A2_P6_E2_Ol0_s0_c2`；三机候选评分的
原始逐项数值未在临时目录清理前持久化，因此不补造 score。UAV1 的0.945260 m回归用例
已证明能进入0.5 m已膨胀图门限后的评分/EGO路径，而不会被旧1.0 m门限提前删除。

关键屏障与放行仿真时刻：

| 事件 | UAV1 | UAV2 | UAV3 |
|---|---:|---:|---:|
| HOVER_READY | 11957.990 | 11958.830 | 11958.130 |
| ENTRY_READY | 12043.539 | 12068.575 | 12130.330 |
| ORBIT_STAGING_READY | 12149.090 | 12134.025 | 12163.879 |
| ORBIT_RELEASE | **未发生** | 12230.811 | 12163.911 |

UAV2 的放行条件为 `phase_3_2=65.080°`，位于要求的65°～70°窗口；放行前日志持续记录
phase不满足而抑制，因此验证了 UAV3 先行、UAV2 后行。UAV1 放行前任务已失败，未形成
67.5°—67.5°三机编队。

### 第一条安全门与最终失败

第一条使本次闭环不可恢复的条件为：

```text
sim_time=12255.710
uav=UAV2
stage=NAVIGATING_AFTER_ORBIT_RELEASE
topic=/uav2/mavros/local_position/pose
condition=tracking input failure: MAVROS local pose is unavailable,
          stale or has an unusable timestamp
action=TRACK_EGO -> HOLD -> RETURN_HOME
```

同一时段 MAVROS 有 time-jump 诊断；话题随后恢复更新，但安全门按原门限正确锁存，未作
阈值放宽。UAV2 最终在仿真时刻12382.2进入 `DONE`，z=-0.023 m，有接地证据。

UAV3 此后在第5扇区得到第二个明确失败：
`SECTOR_5_NO_SAFE_REACHABLE_CANDIDATE total=21`，原因为
`SECTOR_BOUNDARY:1|PLANNER_UNREACHABLE:8|KNOWN_OBSTACLE_CLEARANCE:12`，并在
12454.409进入 HOLD。仿真栈随后收到 SIGINT；最后记录为 UAV1
`ORBIT_STAGING_READY,z=2.936 m`、UAV3 `HOLDING,z=1.859 m`。两机均无返航或落地
证据，不能称为安全三机落地。

### 验收矩阵

| 验收项 | 结果 |
|---|---|
| 三机实体、PX4/MAVROS、FAST-LIO、swarm健康连续5 s | 通过（8 s） |
| SAFETY_INHIBIT解除、bridge离开WAIT_INPUTS | 通过 |
| 三机同时起飞、3.0 m HOVER_READY | 通过 |
| 动态PRE/ENTRY/第一巡塔点、0.5/1.0 m净空语义 | 通过 |
| UAV3先放行，65°～70°放行UAV2 | 通过（65.080°） |
| 65°～70°放行UAV1、67.5°—67.5°编队 | 未完成 |
| 无超车/反向/穿塔心 | 已完成片段未发现冲突；完整圈未验证 |
| 16.5 m平滑回归12.5 m | UAV2开始回归但被定位时效门中断；未通过闭环验收 |
| 各机独立完整360°、8扇区 | 未完成 |
| 前机退出后角色链更新 | 未验证 |
| 独立EXIT、逆序进场路线返航、最新地图重规划 | 未完成 |
| 三机返回起飞点并安全降落 | **失败；仅UAV2有接地证据** |

因此完整第五阶段结论为 **FAILED**，不是“安全终止成功”，更不是“三机完整绕塔成功”。

### 测试、证据和改动清单

- `astra_tower_mission` 本轮相关C++回归：70/70通过。
- `astra_swarm_safety`：构建通过，nosetest 7/7通过，Python静态编译通过。
- 完整 `astra_tower_mission` rostest 在受限网络命名空间中因
  `netifaces.interfaces()` PermissionError 未执行成功；这不是用例断言通过，不能计入成功数。
- 规则/YAML/代码一致：角偏移 `[0,-5,+5,-10,+10,-12,+12]°`、半径
  `[12.5,14.5,16.5] m`、膨胀图0.5 m、原始点云/粗几何1.0 m、角色顺序
  UAV3→UAV2→UAV1、放行65°～70°、目标67.5°均一致。

代表性证据：

- `test_evidence/stage5_full_orbit_20260802/summary.json`
- `test_evidence/stage5_full_orbit_20260802/README.md`
- `test_evidence/stage5_dynamic_entry_20260801_attempt3/`
- `test_evidence/stage5_dynamic_entry_20260801_attempt4/`

此次 headless 运行没有录制 bag 或 RViz 截图；第三次运行的原始日志/CSV随临时目录被执行
环境清理，轨迹图也因此没有生成。报告明确记录缺失，不提供虚假路径。

本轮源码修改仅为：

- `AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/src/sector_inspection_mission_node.cpp`
- `AstraDrone_ros1_ws/src/Swarm/astra_swarm_safety/scripts/swarm_safety_node.py`
- `ego_swarm_three_uav_integration_report.md`
- `test_evidence/stage5_full_orbit_20260802/{README.md,summary.json}`

`AstraDrone_ros1_ws/src/SLAM/FAST_LIO/Log/mat_pre.txt` 是FAST-LIO运行时写入的既有用户资产，
未手工修改、恢复或清理，不计入源码成果。

## 2026-08-02 时效、候选与地图边界根因修复复验

本节补充上一次 `stage5_full_orbit_20260802` 失败后的根因定位和修复结果。开始与结束均为
分支 `ego-swarm`、HEAD `1519a4220342f5d7e428c4ac589c8f60c374970d`；没有切分支、
commit 或 push。用户已有的 `README.md` 删除、`FAST_LIO/Log/mat_pre.txt` 运行写入和
既有未提交修改均保留。没有修改 `worksite.world`、`rule.md`、EGO vendor 核心或外部
PX4，也没有降低任何超时、障碍净空或机间距离。

### 根因一：UAV2 位姿被误判为时间戳异常

`/clock` 的永久 bag 和 recorder 时间轴没有发现回退或 epoch 重置。正式无控制运行
`stage5_dry_run_20260802_150230` 中 `/clock` 共 36671 点，从 11934.010 s 单调推进到
11970.680 s，无回退或重复；三台 bridge 的 `clock_epoch` 均为0。真实控制日志中的
MAVROS `TM : Time jump detected` 来自 MAVROS/PX4 时间同步器调整，不等于 Gazebo
`/clock` 回退。

bridge 原实现把 `header.stamp > ros::Time::now()` 的任意微小正偏差都视为无效，但同一
节点的 preflight 已配置并接受 `timestamp_future_tolerance=0.02 s`。三机负载和 MAVROS
时间同步修正使 UAV2 pose header 偶尔比当前 ROS timer 早到一个调度周期，从而产生假
定位超时。修复后所有 stamped/receive freshness 检查统一使用原有 0.02 s 未来偏差容限；
定位、心跳、轨迹超时数值均未改变。

bridge 同时新增明确的 ROS 时间回退处理：只有检测到超过既有容限的真实回退才增加
clock epoch、锁存 HOLD、清空带时效的 pose/odom/cloud/command/goal 缓存并取消旧轨迹；
随后必须等待新的 pose、odom、点云、指令和全部原安全门重新成立，绝不复用旧位姿。
私有 `input_health` 以 JSON 记录每项 header/receive age、fresh 标志和 epoch。

第二次控制运行跨越原故障窗口 12245～12265 s：UAV2 共200个健康采样，local pose
全部 fresh，最大接收年龄0.036 s、header年龄0.005～0.037 s，odom和点云也全部 fresh，
epoch始终为0；整段 `swarm.csv` 6748点（11934.910～12609.610 s）无时间回退。因此原
12255.710 s 位姿假超时已经复现后消除，且没有放宽 timeout。

### 根因二：UAV3 第5扇区是地图越界，不是已发布地图中的真实占据

第一轮候选审计先修复了两个任务层错误：

1. 16.5 m 外边界候选经过 `sin/cos -> hypot/atan2` 后有亚微米浮点误差，原严格比较会
   错误产生一个 `SECTOR_BOUNDARY`；现在边界仍为原值，只用 `1e-6` 数值容差做包含判断。
2. `StaticObstacle` 已是审计后的物理/粗几何包络，原代码又叠加0.4 m cloud inflation，
   把粗几何1.0 m净空重复放大成1.4 m；现在粗几何只应用一次1.0 m，原始点云仍为1.0 m，
   已膨胀图附加净空仍为0.5 m。

第二次控制运行仍在第5扇区失败，永久 bag 给出了最终根因。UAV3 首选同扇区
`c17=(16.5 m,-12°)`，静态直线对西北电杆物理表面净空约1.322 m，端点净空约1.851 m；
EGO 随后报告当前位置占据。但离线逐帧对比 `/uav3/stage3/occupancy_inflate` 的972个
样本，发布地图离机体最近点始终大于1.0 m，最小为1.233 m。两者不一致是因为 Stage5
launch 把每台固定原点 EGO 地图缩为 `60×80 m`：

```text
UAV2 tower local x = -14.0551 m，16.5 m候选最小x = -30.5551 m
UAV3 tower local x = -18.0551 m，16.5 m候选最小x = -34.5551 m
旧EGO地图x边界 = [-30,+30] m
```

EGO vendor 的 `getInflateOccupancy()` 对地图外位置返回 `-1`，现有状态适配层将其直接
转为布尔值，因而显示成 `CURRENT_POSITION_IN_OCCUPANCY`；发布的占据点云不会发布地图
外体素，所以离线图中看不到对应障碍。按限制未修改 vendor。Stage5 地图改为
`90×90 m`，并把相同 map size 和未改变的7.5 m planning horizon传给任务节点。任务启动
时现在强制验证完整12.5/14.5/16.5 m绕塔包络加规划视距均在固定地图内；UAV3所需最小
x为 `-18.0551-16.5-7.5=-42.0551 m`，新边界为 `[-45,+45] m`。旧60×80配置会被该
preflight直接拒绝，不再飞到地图外后误报占据。

此外，第5阶段联合进场锁定的是 `orbit_candidate_id=l0_s0_*`，原首航点循环却可能先用
另一个扇区的最佳候选建立记账顺序，再只覆盖坐标为联合 staging。UAV3因此把337.5°
staging记到22.5°扇区，释放后直接请求67.5°，形成约17.75 m/90°首段并触发120 s目标
超时。现在联合候选必须绑定其自身 accepted sector；找不到同一ID就 fail-closed，不再
使用错扇区替代。后续首段恢复为相邻45°，临时14.5/16.5 m半径仍在下一扇区平滑回归
12.5 m，不改变单机默认策略。Stage5还只在三机launch中优先选择同扇区内已有安全直线
走廊的候选；若不存在，仍保留原EGO绕行候选集。

### 修改位置

- `ego_gazebo_bridge/include/ego_gazebo_bridge/ego_mavros_bridge.h`、
  `src/ego_mavros_bridge.cpp`：统一未来偏差语义、真实clock回退安全清缓存、
  `/input_health`证据。
- `astra_tower_mission/include/astra_tower_mission/inspection_candidate_planner.h`、
  `src/inspection_candidate_planner.cpp`、`src/sector_inspection_mission_node.cpp`：候选边界数值容差、粗几何
  单次净空、同扇区安全直线偏好、联合候选ID/扇区绑定、地图包络preflight。
- `astra_tower_mission/test/inspection_candidate_planner_test.cpp`：上述规则的单元回归。
- `astra_swarm_bringup/launch/{dual_tower_inspection,uav_tower_stack,
  triple_tower_inspection,triple_px4_mavros}.launch`：只在三机阶段启用新偏好、90×90地图、
  后台PX4控制台和证据参数；单机/双机默认候选策略不变。
- `astra_swarm_bringup/scripts/swarm_evidence_recorder.py`、
  `scripts/run_sh/three_uav_inspection.sh`、`scripts/tool/{three_uav_map_probe,
  analyze_occupancy_bag,plot_three_uav_trajectory}.py`：永久日志、bag、数据年龄、逐候选
  JSONL、地图离线核对和轨迹图。

### 永久证据目录与真实运行结果

本轮每一次实际 Gazebo 启动均原样保留，没有删除失败数据：

| 目录 | 模式与结果 |
|---|---|
| `test_evidence/stage5_dry_run_20260802_141530/` | 探索性dry-run；发现PX4交互控制台刷屏，原日志、bag、CSV、summary和轨迹图保留。 |
| `test_evidence/stage5_dry_run_20260802_141804/` | 第二次dry-run；确认仅改interactive参数不足，全部原始证据保留。 |
| `test_evidence/stage5_dry_run_20260802_142039/` | 修正PX4后台方式后的正式dry-run；三机pose/odom/cloud连续fresh。 |
| `test_evidence/stage5_control_20260802_142342/` | 第一次控制；UAV2原故障窗口通过，UAV3仍因地图边界问题失败；监督落地后三机均unarmed/ON_GROUND/DONE，但任务为HOLD/ERROR，判定失败。 |
| `test_evidence/stage5_dry_run_20260802_150230/` | 修正证据采集后的正式dry-run；三机309个稳定采样全部heartbeat/localization/safety健康，clock无回退。 |
| `test_evidence/stage5_control_20260802_150614/` | 第二次控制；确认UAV2时效修复和UAV3地图外根因；UAV3安全落地，UAV1/UAV2在全局HOLD时因外部执行授权失效无法调用监督落地服务，原bag正常封包后终止。 |

每个目录含 `roslaunch.log`、`stage5.bag`、`rosbag.log`、三机CSV、`swarm.csv`、
`summary.json`、`candidates.jsonl`、`run_metadata.txt` 和 `trajectory_xy.png`（早期目录的轨迹图
已由保留CSV补生成）；控制目录另含第5扇区地图分析。

第二次控制的放行证据为：UAV3 `12171.712 s`；UAV2 `12238.412 s`，UAV3领先
`65.139°`；UAV1 `12339.612 s`，UAV2领先`65.026°`。三机均起飞并到达3.0 m任务高度，
联合PRE_ENTRY/ENTRY_GATE/首巡塔点锁存成功。失败前最小三维机间距离为3.957 m，未低于
3.0 m；没有穿塔心或轨迹交叉。由于UAV3先后受到错扇区90°首段超时和地图越界影响，
累计角仅为UAV1 71.444°、UAV2 118.548°、UAV3 160.768°，sector mask分别为
`0x81/0x83/0x0F`，没有任何一机完成8扇区/360°、独立EXIT或完整逆序返航。

### 构建、测试与最终结论

- 目标六包构建通过；Python `py_compile`、Bash语法和launch XML检查通过；launch完整展开
  确认三台EGO/任务节点均为90×90 m、planning horizon 7.5 m、原始/粗几何净空1.0 m、
  已膨胀图附加净空0.5 m、最小三维机间距离3.0 m、swarm clearance 1.5 m。
- 当前直接执行的C++/Python测试148项全部通过，其中规划器59/59；工作区测试结果汇总
  为303项中的302项通过。唯一缺项是 `sector_inspection_no_control.rostest`：沙箱禁止
  `netifaces.interfaces()`，申请沙箱外执行又因执行服务登录refresh token已撤销而拒绝。
  这明确记为“未执行”，不算通过。
- 地图扩大和首扇区绑定修复完成后，因同一外部执行授权故障，无法再启动第三次真实
  dry-run/control运行。因此目前没有修复后完整8扇区、360°、EXIT、逆序返航和三机
  `armed=false/ON_GROUND/bridge=DONE`证据。

最终第五阶段结论仍为 **FAILED（修复已实现，但完整闭环未重新验收）**。不能把两次
HOLD、安全收尾、部分绕塔或授权阻塞写成PASSED；只有后续新的永久证据目录同时满足13项
完整闭环条件后，才能将本结论改为PASSED。

## 2026-08-02 第五阶段三机低空绕塔最终验收

本节是上述历史失败章节之后的最新最终结论。最终闭环已在全部安全门保持原值的条件下
重新完成，因此本节取代前文的阶段结论，但不删除或改写任何失败证据。

### 基线、工作区和运行前检查

- 分支为 `ego-swarm`，HEAD 为
  `1519a4220342f5d7e428c4ac589c8f60c374970d`，相对 `origin/ego-swarm` 领先9；本轮没有
  切换/创建分支，没有 commit 或 push。
- 工作区在运行前后均为未提交状态。Stage5源码/配置/报告和全部证据保留；用户已有的
  `README.md` 删除以及 `FAST_LIO/Log/mat_pre.txt` 运行写入未恢复、未清理、未暂存。
- 三台EGO固定地图均为 `90×90 m`，planning horizon为7.5 m，任务高度为3.0 m；角色顺序
  UAV3→UAV2→UAV1，放行窗口65°～70°，目标相邻相位67.5°。
- 原始点云/粗几何净空保持1.0 m，已膨胀地图附加净空保持0.5 m，最小三维机间距离保持
  3.0 m，`swarm_clearance=1.5 m`。没有修改任何安全阈值。
- `GAZEBO_PLUGIN_PATH` 包含仿真工作空间并成功加载Livox插件；运行前没有遗留PX4、MAVROS、
  FAST-LIO、EGO、任务、manager或safety重复进程。没有修改 `worksite.world`、`rule.md`、
  EGO vendor核心或外部PX4。

目标六包构建通过。C++/Python/rostest汇总为 **308 tests, 0 errors, 0 failures, 0 skipped**；
Python `py_compile`、Stage5脚本 `bash -n`、全部相关launch XML解析和 `git diff --check`
均通过。此前受沙箱网络接口限制而未执行的 `sector_inspection_no_control.rostest` 已使用
隔离 `ROS_HOME` 在允许环境中执行通过。

### 最终dry-run预检

最终无控制预检目录为
`test_evidence/stage5_final_validation_20260802_212525/`，连续记录37.871 s仿真时间。

- 三个Gazebo实体由验证式spawn适配器确认存在；模型状态、MAVROS connected/local pose/
  extended state、三套FAST-LIO odom/cloud、三套任务/bridge/planner状态、三套
  `/uavN/swarm/state` 和全局coordinator状态连续更新。
- UAV1/2/3分别记录MAVROS pose 1130/1133/1138条，FAST-LIO与planner odom
  325/333/334条；三台 `/input_health` 最新pose/odom/cloud全部fresh，`clock_epoch`集合均为
  `{0}`。没有假定位超时、地图越界或任务/EGO节点异常退出。
- `/clock` 单调推进，heartbeat、localization和safety在输入建立后持续健康；启动最初的
  fail-closed输入等待没有产生控制授权。
- 地图包络预检全部通过：UAV1要求
  `[-34.0551,13.9449]×[-4.2896,43.7104]`，UAV2要求
  `[-38.0551,9.9449]×[-4.2896,43.7104]`，UAV3要求
  `[-42.0551,5.9449]×[-4.2896,43.7104]`，均位于新地图
  `[-45,45]×[-45,45]` 内。
- 实际架构使用三条命名空间状态 `/uav1/swarm/state`、`/uav2/swarm/state`、
  `/uav3/swarm/state`，而不是额外复制一条全局 `/swarm/state`；bag同时保留全局
  `/swarm/coordinator/status`、formation和safety状态。

只有上述检查全部通过后才启动最终控制闭环。

### 动态进场和放行

三机近同时起飞：UAV2/UAV3/UAV1的takeoff时刻分别为11952.910、11953.010、11953.810 s，
并分别在11957.990、11958.050、11958.750 s进入3.0 m `HOVER_READY`。联合选择于
11975.049 s锁存。下表坐标为world/map坐标；括号内为实际任务局部坐标（UAV1与world
相同，UAV2/UAV3分别减去4/8 m的spawn x偏移）。

| 角色 | 联合候选ID | 角度/首点半径 | PRE_ENTRY | ENTRY_GATE | 第一巡塔点 |
|---|---|---:|---|---|---|
| UAV3 | `U3_G1_A1_P6_E0_Ol0_s0_c0` | 337.5° / 12.5 m | (8.269,13.041,3.000)；local (0.269,13.041,3.000) | (4.040,14.580,3.000)；local (-3.960,14.580,3.000) | (1.493,14.927,3.000)；local (-6.507,14.927,3.000) |
| UAV2 | `U2_G1_A0_P0_E0_Ol0_s0_c1` | 315.0° / 14.5 m | (2.673,6.982,3.000)；local (-1.327,6.982,3.000) | (0.552,9.104,3.000)；local (-3.449,9.104,3.000) | (0.198,9.457,3.000)；local (-3.802,9.457,3.000) |
| UAV1 | `U1_G1_A1_P0_E3_Ol0_s0_c2` | 292.5° / 16.5 m | (-2.448,3.397,3.000) | (-3.399,5.436,3.000) | (-3.741,4.466,3.000) |

ENTRY_READY时刻为UAV1 12043.759、UAV2 12072.080、UAV3 12125.789 s；三机共同
`MOVE_TO_ORBIT_STAGING`只发布一次（12125.813 s）。实际放行严格保持固定角色顺序：

| 放行 | 仿真时刻 | 前机实际领先角 | 判定 |
|---|---:|---:|---|
| UAV3 | 12156.813 | leader先行，不适用 | 通过 |
| UAV2 | 12237.213 | UAV3领先65.057° | 位于65°～70°窗口 |
| UAV1 | 12329.713 | UAV2领先65.103° | 位于65°～70°窗口 |

UAV1放行后的最初30 s内，UAV3→UAV2相位均值/中位数为70.110°/70.145°，
UAV2→UAV1为69.098°/69.126°，形成约67.5°—67.5°的同向角色编队。完整三机共同绕塔期
包含EGO端点等待和安全调速，UAV3→UAV2均值58.234°、范围45.521°～70.480°，
UAV2→UAV1均值69.984°、范围52.798°～77.091°；相位从未跨越180°，三机的
`reverse_detected`均为false，没有超车、反向、角色交换、轨迹交叉或穿越塔心。

联合选择的第一点净空分别为UAV1 1.268 m、UAV2 0.979 m、UAV3 0.890 m；这里是已膨胀
地图的0.5 m附加净空语义，原始点云/粗几何仍执行1.0 m。`candidates.jsonl`保存全部逐候选
记录；重复观测拒绝计数为：UAV1
`FIRST_LEG_TOWER_KEEP_OUT=53, KNOWN_OBSTACLE_CLEARANCE=69220,
OCCUPANCY_OR_CLEARANCE=60444`，UAV2为56/152867/25724，UAV3为53/130200/28318。

### 半径回归、完整巡塔和机间安全

UAV1从16.5 m临时第一点向下一扇区12.5 m目标回归的实际半径为
16.307→12.473 m，区间12.424～16.341 m，0.1 s采样的最大半径变化0.025 m；UAV2从
14.5 m临时点回归时为14.940→12.429 m，区间12.367～14.941 m，最大采样变化0.019 m。
UAV3首点本来就是12.5 m。三条回归均由EGO连续轨迹执行，没有跳变、穿心或用HOLD替代。

| UAV | 完整圈确认时刻 | 累计角（日志/CSV） | 扇区mask | 反向检测 |
|---|---:|---:|---:|---|
| UAV3 | 12902.639 | 365.673° / 365.644° | `0xFF` | false |
| UAV2 | 12997.079 | 372.489° / 372.467° | `0xFF` | false |
| UAV1 | 13110.607 | 365.109° / 365.083° | `0xFF` | false |

三机同时绕塔期间的全局最小水平/三维/EGO椭球距离分别为8.704/8.704/8.704 m；包含
起飞、进场、退出、返航和落地的全任务最小值分别为3.881/3.909/3.888 m，仍全部高于
3.0 m。分对总体最小值为：UAV1-2水平3.881、三维3.909、椭球3.888 m；UAV1-3为
7.869/7.888/7.875 m；UAV2-3为3.887/3.921/3.911 m。`emergency_count`三机均为0。

最终闭环中出现三次可恢复任务安全门：UAV3在12609.190 s等待第6目标达到120 s后HOLD，
UAV2在12795.729 s的最后扇区得到一次 `NO_FEASIBLE_TRAJECTORY`，UAV1在13093.758 s等待
闭环首点达到120 s后HOLD。三者均保持原端点和原超时门限，在约2 s的有界重试后确认或
重新生成可行轨迹；下游相位HOLD随前机恢复而解除。没有跳过扇区、写死健康状态或降低门限。

### 独立退出、逆序返航和落地

三机使用各自动态进场对应的独立EXIT_GATE：UAV3 local
(-3.960,14.580,3.000)、UAV2 local (-3.449,9.104,3.000)、UAV1
(-3.399,5.436,3.000)。到达EXIT并开始逆序返航的时刻分别为UAV3 12933.338、UAV2
12999.429、UAV1 13116.757 s。

- UAV3按 `INGRESS_TRACE_8→...→INGRESS_TRACE_0` 完成9/9个实际进场采样点；
- UAV2按 `INGRESS_TRACE_4→...→INGRESS_TRACE_0` 完成5/5个实际进场采样点；
- UAV1按 `INGRESS_TRACE_2→...→INGRESS_TRACE_0` 完成3/3个实际进场采样点。

每一个逆序点发布前都记录 `map_fresh=true`，并由EGO生成新的trajectory id；本次最终运行
没有任何blocked endpoint，因此没有走“跳过历史点”或direct HOME降级。完成实际进场逆序后，
三机才分别请求HOME_HOVER并获得独占落地许可。

| UAV | RETURN_HOME | 接地/解除武装 | task DONE | 最终状态 |
|---|---:|---:|---:|---|
| UAV2 | 13126.480 | 13138.309 | 13138.329 | `armed=false, ON_GROUND, bridge=DONE` |
| UAV3 | 13159.189 | 13170.408 | 13170.438 | `armed=false, ON_GROUND, bridge=DONE` |
| UAV1 | 13204.059 | 13215.147 | 13215.157 | `armed=false, ON_GROUND, bridge=DONE` |

### bag深度审计和永久证据

最终控制证据为 `test_evidence/stage5_final_validation_20260802_212843/`：完整bag为4.8 GiB，
时长1307.798 s，4,099,835条消息，LZ4压缩且索引完整；CSV、候选JSONL、summary、运行
metadata、roslaunch/rosbag日志和XY/3D轨迹图均已生成。逐条bag审计结果：

- `/clock` 1,307,795条，从11934.010严格单调到13241.808 s，回退0、重复0；三机
  `clock_epoch=0`；
- `/gazebo/model_states` 1,307,521条，每条均包含 `iris_mid360_00/11/22`，模型缺失0帧；
- 三机MAVROS raw/framed pose均约39.2k条，FAST-LIO raw/planner odom均约10.8k条，
  三机最新pose/odom/cloud全程fresh；没有假定位超时；
- `/swarm/safety/event` 的14条事件全部位于11934.110～11935.311 s启动输入建立阶段，属于
  fail-closed `SWARM_STATE_MISSING/HEARTBEAT_TIMEOUT`，授权前自动恢复；飞行阶段emergency为0。

本次最终验收期间创建的所有目录均永久保留：

- dry-run/启动检查：
  `stage5_final_validation_20260802_170608`、`171112`、`181011`、`191822`、`191948`、
  `192104`、`192512`、`203328`、`212525`；
- 控制闭环：`stage5_final_validation_20260802_171440`、`181324`、`192759`、`212843`。

它们均位于 `test_evidence/`。`191948`在完整recorder启动前即退出，因此只有启动日志和
metadata、没有伪造summary；其余目录保留当次bag、日志、CSV、候选和可生成的图表。
关键失败链为：`171440`暴露3 m任务附近固定EGO垂直下边界过紧；`181324`暴露leader-wait
HOLD被错误向后传播形成协调自锁；`192759`三机均完成巡塔和落地，但UAV2首个历史返航点
在最新地图中无效，旧逻辑错误地直接HOME；`203328`中Gazebo spawn service超时返回失败，
但实体随后实际存在且上游one-shot节点以1退出。

对应的最小修复分别为：把Stage5 EGO垂直地图下界覆盖到原已授权任务最低高度（目标仍为
3.0 m）；只让下游安全故障HOLD传播，leader-wait不传播；有界重试后扫描后续实际进场采样，
只跳过地图中确实无效的历史点并继续fresh EGO逆序规划；新增项目内验证式spawn适配器，
服务返回失败时仍以 `get_model_state` 验证实体，实体实际不存在才失败。没有修改vendor、
外部PX4或安全门限。最终 `212525` dry-run和 `212843` 控制闭环在这些修复上全部通过。

### 最终结论

三架无人机均完成了独立360°、8扇区、独立EXIT_GATE、实际进场路线逆序且按最新地图重规划、
HOME返航和正常接地解除武装；最终三机均为 `armed=false`、`landed_state=ON_GROUND`、
bridge/task/flight=`DONE`。第五阶段三机低空绕塔最终验收结论为 **PASSED**。
