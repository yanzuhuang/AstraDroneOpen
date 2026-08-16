# Clearance Semantics Cleanup Report

审计基线：`scene01-3uav-circuit-mission`，HEAD
`e7e17986042ffd7000523d4cde1fcda3afbae594`。本报告按“一个独立的布尔判定或
安全函数调用算一个检查点；循环内的点数不重复计数”统计。

## 1. 为什么以前已经修过，这次还会再次出现？

以前的修复是增量式修复。`488ddfc` 引入低空路径和地图 clearance 逻辑，
`b79d00e` / `1519a42` 又加入了 `mappedTaskClearance()` 和完整入口走廊检查，
但表示语义仍散落在大型状态机、候选规划器和 recovery helper 中。候选生成路径已经
使用统一入口，`ENTRY_GATE_TRANSIT` 的发布前检查却保留了旧的
`filter_config_.minimum_clearance`。

另一个根因是 `planningMapPoints()` 在 occupancy 为空时会退回 filtered/raw cloud，
旧代码却只看 YAML 中名义上的 `map_points_are_inflated=true`，没有按当前实际返回的
数据选择阈值。废弃的任务层 `cloud_inflation` 又形成了第三套隐含语义，把部分 raw / 
coarse 检查悄悄变成 1.4 m。此前没有覆盖 selector 边界值和各状态 dispatch/recovery
路径的回归用例，所以同类残余没有被测试拦住。

## 2. 之前修复漏掉了哪些 legacy 路径？

共漏掉 7 类路径、12 个具体判定位置：

1. `ENTRY_GATE_TRANSIT` 发布前仍用 1.0 m 检查 inflated occupancy；
2. `planningMapPoints()` 从 inflated occupancy 退回 raw cloud 时仍沿用 0.5 m；
3. sector candidate 的 raw endpoint 和 raw corridor 叠加了 0.4 m；
4. ENTRY_GATE candidate 的 coarse endpoint、raw endpoint、coarse corridor、raw
   corridor 叠加了 0.4 m；
5. same-sector fallback detour 的 coarse geometry 使用 1.4 m；
6. layer-transition 的整列和逐段检查把 inflated map 与 coarse geometry 合并后统一用
   1.4 m；
7. recovery endpoint/corridor 把 inflated map 与 coarse geometry 合并后统一用
   1.0 m。

另外发现两个容易继续漂移但不属于同一个阈值冲突的问题：normal return 自己重复实现
selector；EXIT_GATE 复用旧 gate 时没有使用最新地图做 dispatch 前复核。本次一并清理。

## 3. 本次一共发现多少 clearance 检查点？

共发现并逐项分类 **49 个可执行检查点**：

| 类别 | 数量 | 数据与用途 |
|---|---:|---|
| 任务候选、PRE_ENTRY/ENTRY_GATE、sector、fallback、layer、EXIT、normal return、landing | 39 | raw/filtered、coarse geometry 或 inflated occupancy，按实际表示选择阈值 |
| 低空 A* | 2 | inflated occupancy 的可通行网格、独立 tower keep-out |
| planner status adapter | 2 | inflated occupancy 上的当前位置/目标执行期碰撞看门狗 |
| swarm manager / safety | 2 | 联合入口候选余量、机间分离 |
| EGO 内部 | 1 | `occupancy_buffer_inflate_` 的规划碰撞判定与发布 |
| 独立 route / emergency / recovery 保护 | 3 | tower radial keep-out、紧急返航粗几何、当前位置碰撞释放保护 |

cloud ground filter、self filter、teammate filter 只改变输入点集或做诊断，不实施任务
clearance 判定，因此没有把它们虚计为 clearance 检查点。

## 4. 哪些存在冲突？

上述 49 个检查点中，存在冲突的是第 2 节列出的 **7 类路径、12 个具体判定位置**。
冲突形式有三种：inflated map 错用 1.0/1.4 m、raw/coarse 错用 1.4 m、以及同一次
调用把 inflated 与 coarse 数据混在一个阈值下检查。

以下检查不是冲突并予以保留：planner adapter 的 0.55 m 执行期看门狗、landing
区域柱体检查、recovery 的当前位置占据释放门、tower keep-out、紧急返航粗几何以及
1.5 m 机间分离。它们保护不同对象或不同时间点，不是候选 clearance 的重复实现。

## 5. 删除或修改了哪些残余代码？

- 新增纯函数 `mappedTaskClearance()` overload 和可直接单测的
  `mappedEndpointClear()`，候选、ENTRY、dispatch、layer、recovery、EXIT 和 return
  全部经同一语义入口；
- selector 改为按 `planningMapPoints()` 当前实际返回的数据判断。occupancy 非空才按
  inflated 处理；退回 filtered/raw cloud 时自动使用 minimum；
- sector/ENTRY candidate 不再减去或叠加任务层 0.4 m；
- recovery 和 layer-transition 将 map 与 coarse geometry 拆成两次独立检查；
- ENTRY_GATE dispatch 改用统一 selector；EXIT_GATE 增加最新地图和 coarse endpoint
  复核；normal return 删除自有 selector；
- 删除已经无消费者、容易再次造成双重膨胀的任务层 `cloud_inflation` 结构字段、ROS
  参数和 YAML 项。EGO 的 `grid_map/obstacles_inflation` 完全未改；
- 把 `map_additional_clearance` 的 C++ fail-safe 默认从 0.0 对齐到 0.5，并把低空 A*
  的旧 0.6 fallback 对齐到运行 YAML 中既有的 0.5；
- 更新 `rule.md`，明确 raw/filtered/coarse 的 minimum 只应用一次。

没有修改 PRE_ENTRY / ENTRY_GATE / EXIT_GATE 坐标、航点选择规则或任何用户现有场景
资产。

## 6. 现在 raw / filtered / inflated 分别使用什么 clearance？

| 数据表示 | 任务层阈值 |
|---|---:|
| raw cloud | `minimum_clearance = 1.0 m` |
| filtered cloud | `minimum_clearance = 1.0 m` |
| coarse geometry | `minimum_clearance = 1.0 m` |
| EGO inflated occupancy | `map_additional_clearance = 0.5 m` |

EGO 在生成 inflated occupancy 时仍使用原有
`grid_map/obstacles_inflation = 0.4 m`。filtered cloud 没有被任务层再次膨胀；ground、
self、teammate filter 只筛点。

## 7. 是否还存在已知的重复/冲突路径？

**没有已知冲突路径。** 全局反查没有发现 inflated occupancy 直接使用
`minimum_clearance`，也没有发现 `minimum_clearance + 0.4` 或另一套表示选择 ternary。

仍保留两类合理二次检查：候选生成后在 permission/dispatch 前使用最新地图复核，以及
planner adapter 在轨迹执行期间持续检查当前位置和目标。它们用于抵抗地图随时间变化，
且任务层复核使用同一个 selector，不再形成第二套 clearance 语义。

## 8. 单机和三机语义是否完全一致？

**是。** 单机 `low_altitude_inspection.launch` 与三机
`triple_tower_inspection.launch` 都加载同一个 `low_altitude_inspection.yaml`。实际
`--dump-params` 结果为：单机和 UAV1/UAV2/UAV3 的 candidate/entry minimum 均为
1.0，map additional 和低空 A* additional 均为 0.5，EGO obstacles inflation 均为
0.4；三机 manager 的 map additional 也是 0.5。没有 launch override 覆盖错误。

通用高空 `sector_inspection.yaml` 仍保留原有更严格的 2.0 m minimum；C++ 通用 fallback
也保留 2.0 m 以便 YAML 缺失时 fail closed。它不是当前单机/三机低空 worksite 配置，
也不会改变“raw 用 minimum、inflated 用 map additional”的表示语义。

## 9. regression tests 是否全部通过？

**是。** `astra_tower_mission` 构建通过，包内结果为 **302 tests, 0 errors,
0 failures, 0 skipped**。新增 `ClearanceSemantics` 5/5 通过：

- raw obstacle 0.8 m：FAIL；
- raw obstacle 1.2 m：PASS；
- inflated voxel 0.4 m：FAIL；
- inflated voxel 0.8 m：PASS；
- 当前 PRE_ENTRY 到 inflated voxel 的 3-D 距离约 0.96734 m：按 0.5 m PASS，且测试
  证明旧 1.0 m 会错误 FAIL。

既有 fallback、recovery、layer、entry/sector 相关回归全部通过。无控制 rostest 1/1
通过，完整走过 PRE_ENTRY、ENTRY_GATE、sector、layer transition、EXIT_GATE 和
RETURN_HOME，并确认没有 MAVROS 控制 publisher。单机和三机 launch 均完成纯解析；
没有进行三机飞行或任何解锁/起飞。

## 10. 是否修改过任何安全阈值？

**没有修改批准的运行安全阈值，也没有降低任何配置阈值。**
`minimum_clearance=1.0 m`、`map_additional_clearance=0.5 m`、EGO
`obstacles_inflation=0.4 m` 均保持不变。删除的是未经批准的重复 0.4 m 任务层叠加；
它是导致 raw 1.2 m 被错误拒绝的 legacy 双重计费，不是独立安全阈值。C++ fallback
只对齐到既有 YAML 的 0.5 m，运行配置没有改变。

## CLEARANCE SEMANTICS CLEANUP

**PASS**

