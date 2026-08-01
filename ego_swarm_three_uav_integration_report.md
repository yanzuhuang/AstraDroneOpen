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

- `astra_tower_mission/config/stage3_low_altitude.yaml`
- `astra_tower_mission/src/stage3_ego_mission_node.cpp`
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
| `AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/config/stage3_low_altitude.yaml:146-152` | 共用角偏移 `[0,-5,+5,-10,+10,-12,+12] deg`、半径偏移 `[0,+2,+4] m`、低空高度偏移 `[0]`；`minimum_clearance=1.0 m` 仅用于原始点云/粗几何，`map_additional_clearance=0.5 m` 用于已膨胀占据图。 |
| `.../stage3_ego_mission_node.cpp:1627-1665` | `mappedTaskClearance()` 按地图是否已膨胀选择 0.5 m 或 `minimum_clearance + cloud_inflation`；EGO `dist0` 不作为任务硬门限。 |
| `.../stage3_ego_mission_node.cpp:1670-1782` | 端点、塔体 keep-out、粗几何和地图走廊分层检查；已膨胀地图统一使用 0.5 m 附加净空。 |
| `.../stage3_ego_mission_node.cpp:1784-1915` | 第一巡塔点与 PRE_ENTRY/ENTRY 组合共同枚举；每个组合再绑定同一普通扇区候选网格，评分后才交给 EGO。 |
| `.../stage3_ego_mission_node.cpp:2557-2660` | 初始扇区先评估全部候选再 `chooseBestCandidate()`；规划失败不会跳过同扇区候选，只有当前候选重试耗尽后才换候选/扇区。 |
| `.../stage3_ego_mission_node.cpp:2897` | 静态同扇区绕行也只使用 `[0,+2,+4] m` 半径偏移，不再另设向内半径规则。 |
| `AstraDrone_ros1_ws/src/Swarm/astra_swarm_manager/config/stage5_three_uav_formation.yaml:27-28` | `map_additional_clearance=0.5 m`；`optimizer_swarm_clearance=1.5 m` 保持不变。 |
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
