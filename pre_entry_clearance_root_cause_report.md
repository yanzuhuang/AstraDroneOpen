# PRE_ENTRY clearance 根因审计报告

日期：2026-08-15  
分支：`scene01-3uav-circuit-mission`  
审计 HEAD：`e7e17986042ffd7000523d4cde1fcda3afbae594`

## 1. 结论

根因分类：

- **主因：`OTHER` — `DUPLICATE_CLEARANCE_ON_ALREADY_INFLATED_MAP`**。
- 次因：`REAL_OBSTACLE`（`Pine_Tree_6_clone` 的真实 LiDAR 回波）加
  `VOXEL_THRESHOLD_EDGE_CASE`（0.25 m 量化和方盒膨胀产生 0.967340 m 体素）。
- 明确排除：`TERRAIN_COLLISION_CHANGE`、`GROUND_FILTER_REGRESSION`、
  `SELF_CLOUD_REGRESSION`、`FAST_LIO_ALIGNMENT`、`MAP_INFLATION_CHANGE`、
  `PRE_ENTRY_CONFIGURATION_CHANGE`。

当前被拒绝的点没有新增到障碍物内部：其最近 **filtered 实点**仍有约
1.62 m 净空。最终被任务查询的是 EGO 已膨胀 occupancy 的体素中心，距离为
0.967340 m。错误在于 `ENTRY_GATE_TRANSIT` dispatch 对这份已膨胀地图再次直接应用
raw/coarse 专用的 1.0 m，而候选生成、走廊检查、配置和项目规则早已统一规定：

- raw/uninflated cloud 与粗几何：`minimum_clearance=1.0 m`；
- 已包含 EGO vehicle envelope 的 inflated map：
  `map_additional_clearance=0.5 m`。

因此这不是降低 1.0 m，而是修复一个绕过既有 representation-aware selector 的调用点。
唯一产品代码修改为：

```cpp
// before
const double endpoint_clearance = filter_config_.minimum_clearance;

// after
const double endpoint_clearance = mappedTaskClearance();
```

PRE_ENTRY、ENTRY_GATE、EXIT_GATE、候选规则、EGO、world 和所有安全配置均未修改。

## 2. 0.967342 m 的完整计算链

### 2.1 阈值与调用者

1. `config/low_altitude_inspection.yaml:90-99,149-152` 定义：
   `minimum_clearance=1.0`、`map_points_are_inflated=true`、
   `map_additional_clearance=0.5`。
2. `sector_inspection_mission_node.cpp:375-377` 订阅 occupancy topic；共享栈将它接到
   `/uav1/stage3/occupancy_inflate`，上游是 EGO
   `/grid_map/occupancy_inflate`。stamp adapter 只补合法时间戳，不改点坐标。
3. `occupancyCallback():1462-1478` 在 frame 精确匹配且 stamp 非零后，逐点复制有限
   `x/y/z` 到 `occupancy_points_`。
4. `planningMapPoints():2496-2498` 优先返回非空 `occupancy_points_`；仅在它为空时
   fallback 到 filtered cloud。
5. `mappedEndpointBlockage():1633-1661` 对每个点计算普通 3-D 欧氏距离，选最近点，
   并以 `distance < clearance` 判占据。它不是 ESDF，也不是近似 distance query。
6. 修复前 `ENTRY_GATE_TRANSIT:4727` 硬写
   `filter_config_.minimum_clearance`；这就是 `required_clearance=1` 的来源。
7. 同一类其他 map check 使用 `mappedTaskClearance():1673-1678`，在 inflated map 上
   正确返回 0.5 m。修复后 dispatch 也调用该函数。

坐标差和距离为：

```text
target  = (-3.166800, 3.080570, 3.000000)
voxel   = (-4.125000, 3.125000, 2.875000)
delta   = ( 0.958200,-0.044430, 0.125000)
distance = sqrt(0.9582^2 + (-0.04443)^2 + 0.125^2)
         = 0.967339788 m
```

运行日志用内部未舍入目标 `(-3.166798,3.080568,3)` 打印为 0.967342 m；两者差异只是
输出精度。

`(-4.125,3.125,2.875)` 是 **inflated voxel center**：
`GridMap::indexToPos()` 在 `grid_map.h:371-372` 用
`(index+0.5)*resolution+origin` 输出中心。它不是原始 LiDAR 点。

距离公式本身没有再加 UAV 半径；但被测 cell 已由 EGO `obstacles_inflation=0.40 m`
扩张生成，所以已经间接包含 planner 的 vehicle-envelope inflation。修复前的 1.0 m
又作为点到该 envelope 的附加球形门限，造成重复收取安全包络。

## 3. occupied 点逐层来源

局部窗口取 PRE_ENTRY 周围 3 m，使用失败判定前最后一帧。完整数据见
`runtime_artifacts/learning_speed/pre_entry_clearance_audit/layer_summary.csv` 和
`local_points.csv`。

| 层 | run 1 最近点/距离 | run 2 最近点/距离 | 结论 |
|---|---|---|---|
| FAST-LIO registered raw | `(-4.69661,2.66988,2.65851)` / 1.620371 m | `(-4.70293,2.67580,2.68188)` / 1.620107 m | 真正的传感器点 |
| frame adapted | 同上 | 同上 | 只改 frame 名，不改坐标 |
| teammate filtered | 同上 | 同上 | UAV1-only 无 live peer，pass-through |
| self-filter before/after | 同上 | 同上 | 此点未被自体 mask 删除 |
| ground filtered | 同上 | 同上 | z≈2.66 m，不满足 `z<0.2` 删除条件 |
| raw occupancy topic | 0 点 | 0 点 | EGO point-cloud shortcut 不填 raw occupancy buffer |
| inflated occupancy | `(-4.125,3.125,2.875)` / 0.967340 m | 完全相同 | endpoint 实际查询层 |
| stamped inflated | 与上游逐点相同 | 与上游逐点相同 | adapter 没有几何变化 |

EGO 当前使用 point-cloud shortcut。`grid_map.cpp:781-832`：

```text
resolution = 0.25 m
configured obstacles_inflation = 0.40 m
inf_step = ceil(0.40 / 0.25) = 2 cells
inf_step_z = 1 cell
```

真实回波所在 base cell 中心为 `(-4.625,2.625,2.625)`。向 PRE 方向取
`(+2,+2,+1)` cell，即偏移 `(+0.50,+0.50,+0.25)` m，得到
`(-4.125,3.125,2.875)`。该实现是离散方盒膨胀，不是半径严格等于 0.40 m 的球。
逐运行重建见 `inflation_reconstruction.csv`。

### 3.1 实物归属

`worksite.world` 的生效 `<state>` 在 `:169-177` 把
`Pine_Tree_6_clone` 放在 `(-5.47665,2.30635,0)`；该模型使用真实
`pine_tree.dae` collision mesh（`:976-998`）。PRE 西侧局部点簇、最近 raw return、
树中心和 inflation 扩张方向完全一致。

最终分类：

- 最早 raw source：**C. 塔/杆/建筑结构中的真实树木结构**，同时属于
  **A. 真实环境障碍**；
- endpoint 返回的具体 cell：**G. occupancy inflation 产生**；
- 不是 B 地面、D 自体、E teammate、F FAST-LIO 重影、H 历史残留。

`resetBuffer()` 每次 cloud callback 都更新局部 buffer，而且同一时刻的 raw cloud 中存在
源点，因此不是历史残留。

## 4. 可视化

诊断图：

![PRE_ENTRY local diagnostic](runtime_artifacts/learning_speed/pre_entry_clearance_audit/pre_entry_local_diagnostic.png)

图中可直接看到：真实松树回波在 1.0 m 圆外；量化和 EGO inflation 后的 cell 进入
1.0 m 圆，但仍在已膨胀地图所要求的 0.5 m additional clearance 圆外。

## 5. “以前成功”与当前失败

必须先修正一个证据前提：最新有效成功绕塔并没有执行这个精确 nominal PRE_ENTRY。

- 2026-08-05 完整成功证据：
  `runtime_artifacts/stage5_control_20260805_153438/`，最终三机完成、落地、解除武装。
- 该次 UAV1 动态选择 PRE 为
  `(-2.447971,3.396860,3)`，不是 `(-3.1668,3.08057,3)`；
  `roslaunch.log:3258,3265` 可核对。
- 历史集成报告明确记录 nominal 旧点的 inflated-map 净空约 **0.980323 m**，并在
  2026-08-02 规则恢复增量中规定 inflated map 使用 0.5 m、raw/coarse 使用 1.0 m。
- 当前三次同构运行的 inflated 距离都是 **0.967339788 m**。从历史 0.980323 到当前
  0.967340 仅约 1.3 cm，符合 raw return 在相邻位置但仍量化到同一边缘 cell 的差别；
  两者其实都小于 1.0 m、都大于 0.5 m。

所以事实不是“同一地图下它以前稳定大于 1.0、现在掉到 0.967”。最新成功运行选择了
另一个点；对这个 nominal 点，历史证据也已经显示约 0.98 m。它之所以在规则恢复后可
进入候选/EGO 路径，是因为 map representation 的合法门限是 0.5 m。当前 single legacy
路径在 dispatch 时又错误用了 1.0 m，才把相同安全语义重新变成拒绝。

## 6. 逐项版本变化审计

| 项目 | 历史成功/规则恢复版 vs 当前 | 判断 |
|---|---|---|
| PRE_ENTRY | 同一名义公式：塔心 `(-10.0551,19.7104)`、18 m、292.5°、z=3 | 未变化 |
| 1.0 m | 配置始终存在，且定义为 raw/uninflated/coarse | 未变化；dispatch 用错 representation |
| GridMap resolution | `1519a42`、`c8cfadf`、当前均 0.25 m | 未变化 |
| obstacle inflation | 上述版本均 0.40 m | 未变化 |
| ground filter | `minimum_z=0.20`；后续只增加 diagnostics | 几何语义未变化，且源点 z≈2.66 |
| self filter | 当前源点 before/after 完全相同 | 无泄漏回归证据 |
| teammate filter | single 无 live peer 时源码直接 pass-through；bag 逐点一致 | 无影响 |
| FAST-LIO | `mid360.yaml` 在规则恢复/成功基线到当前无源码 diff | 无配置、extrinsic、voxel/frame 变化 |
| Mid360 | 正式 launch 仍 `lidar_downsample=1`；规划不使用 D435 | sample/downsample/range/frame 无相关变化 |
| Learning Speed | 复现 launch 未启用，默认 `learning_speed_enabled=false` | 无影响 |
| Observation v2 | 两次 pre-fix 和 post-fix 都没有启动 observer | 无影响 |

当前工作区的 `FAST_LIO/Log/mat_pre.txt` 是既有受保护运行日志修改，不是 FAST-LIO 配置；
本次未编辑或恢复它。

## 7. terrain collision 专项验证

离线比较覆盖 PRE、nearest inflated xy、raw tree return xy、树 origin 和两点周围
±0.5/±1.0 m。结果见 `terrain_height_comparison.csv`。

- 129×129 mesh 顶点对原 PNG 高度样本最大误差：`4.90e-10 m`，样本转换精确。
- PRE xy：原高度场 bilinear `-3.431372549020 m`，mesh
  `-3.431372549000 m`，差 `2e-11 m`。
- nearest inflated xy：插值方法差约 `0.02073 m`。
- raw tree return xy：差约 `0.06399 m`；树 origin 处最大约 `0.10984 m`。
- 但模型同时保留 `grass_plane` collision `z=0`；上述 terrain surface 全部约在
  `z=-3.32..-3.43 m`，而源回波为 `z=2.66..2.72 m`。

因此即使保守采用局部最大 11 cm 插值差，它仍在 grass collision 下方约 3.3 m、在
目标回波下方约 6 m，不足以产生当前 occupied voxel。terrain mesh 改动解决 Mid360
启动性能问题，但不是本次净空拒绝根因。

## 8. 重复实验与修复验证

所有运行均使用当前 `worksite.world`、正式三机共享栈的 UAV1-only 模式，
`d435_enabled=true`、`lidar_downsample=1`；没有启动 Observation 或 Learning Speed。
完整表见 `repeated_runs.csv`。

| 运行 | 代码 | nearest inflated | 查询门限 | 结果 |
|---|---|---:|---:|---|
| 既有 Observation v2 worksite run | pre-fix | 0.967342 m | 错误的 1.0 m | FAIL |
| `pre_fix_run_1` | pre-fix，无 observer | 0.967339788 m | 错误的 1.0 m | FAIL |
| `pre_fix_run_2` | pre-fix，无 observer | 0.967339788 m | 错误的 1.0 m | FAIL |
| `post_fix_run_1` | one-line fix，无 observer | 0.967339788 m | 正确的 inflated-map 0.5 m | PASS |

前两次新复现均在仿真时刻约 11951.3 得到完全相同 cell 和距离，随后由既有错误路径安全
返航/落地。它不是在 0.95–1.05 m 随机摆动。

修复后：

- 11951.261：正式发布 `PRE_ENTRY_UAV1_LEGACY=(-3.166798,3.080568,3)`；
- 11993.011：正式发布 `ENTRY_GATE_UAV1_A292.500000`，证明 PRE 已实际到达并完成切换；
- 按任务范围停止继续绕塔：取消当前 EGO trajectory 形成 mission-supervised HOLD，
  再调用既有 land；
- 最终 `connected=true`、`armed=false`、mode=`AUTO.LAND`、
  `landed_state=1 (MAV_LANDED_STATE_ON_GROUND)`。

产品代码构建通过；`astra_tower_mission` 回归为 **292 tests / 0 errors / 0 failures**。
第一次沙箱内 rostest 因无权查询本地网卡产生 1 个环境失败；设置可写 ROS log 目录并在
允许本地接口的相同环境重跑后 292/292，不计作代码失败。

## 9. 修改和安全边界

产品代码只修改：

- `AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/src/sector_inspection_mission_node.cpp`
  一行：dispatch 使用已有 `mappedTaskClearance()`。

诊断数据与可复现工具全部位于：

- `runtime_artifacts/learning_speed/pre_entry_clearance_audit/`

保持不变：

- `minimum_clearance=1.0 m`；
- `map_additional_clearance=0.5 m`；
- PRE/ENTRY/EXIT 坐标；
- 候选和航点选择规则；
- EGO 避障和 GridMap vendor；
- FAST-LIO、Mid360、PX4；
- `worksite.world` 几何；
- Observation v2 和 Learning Speed。

## 10. 最终明确回答

> 这个问题是不是最近 Learning Speed / Observation v2 新增限制造成的？

**NO。**

证据：

1. 两次 pre-fix 复现没有启动 Observation v2 或 Learning Speed，仍完全相同失败；
2. 出错的 dispatch 行来自 2026-07-29 的 `488ddfc`，早于当前 Learning Speed 增量；
3. Observation v2 没有 mission/EGO/MAVROS 控制输出；Learning Speed 在本次 launch 中
   未启用；
4. 仅修复旧任务节点的一处 map-clearance selector 后，在同一 0.967340 m occupancy 下
   PRE_ENTRY 已发布并实际到达。

本次在根因定位、最低风险修复和 PRE_ENTRY 专项复验后停止；没有继续 Observation v2
PASS 验收、trajectory fusion、SAC、RL training 或 dynamic `v_max`。
