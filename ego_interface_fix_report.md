# EGO-Planner 第一轮接口一致性修复报告

日期：2026-07-30

依据：`/home/yanzu/AstraDroneOpen/ego_planner_diff_audit.md`

## 1. 基线、分支与范围

- 修复前分支：`ego-swarm`
- 修复前 commit：`b2b7fa0e52c3b579e0b6bdda5091dc868c039108`
- 本轮工作分支：`ego-interface-fix-round1-20260730`
- 本轮没有 commit 或 push。
- 修复前已有且受保护的工作区修改：
  `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/Log/mat_pre.txt`
- 修复前已有未跟踪报告：`ego_planner_diff_audit.md`
- 上述两个用户资产均未修改。

本轮只修改 Topic、frame、取消时间戳契约、B 样条索引保护的可测试性及相关
launch/test。未修改：

- `planner_manager.cpp`
- `EGOReplanFSM::getLocalTarget()`
- global trajectory 生成、保存和跟踪公式
- `planning_horizon` 默认值（仍为 `7.5`）
- A*、`plan_env`、地图占据与障碍物膨胀
- B 样条代价函数、梯度、权重和动力学可行性公式
- 任务层 ENTRY_GATE、EXIT_GATE、8 扇区、高度分层与返航逻辑
- EGO-Swarm

未启动 Gazebo、PX4、MAVROS、完整单机或双机飞行。

## 2. 修改文件

### 2.1 修改

| 文件 | 修改内容 |
| --- | --- |
| `planner/plan_manage/src/ego_replan_fsm.cpp` | planner-owned Topic 改为可配置相对默认值；用公共 NodeHandle 在车辆 namespace 下解析；status frame 强制与 planning frame 一致 |
| `planner/plan_manage/include/plan_manage/ego_replan_fsm.h` | 保存 odom、waypoint、cancel、bspline、data、status Topic 参数 |
| `planner/plan_manage/src/traj_server.cpp` | goal/cancel/bspline/PositionCommand Topic 参数化；接入可测试的取消时间戳门禁 |
| `planner/plan_manage/launch/advanced_param.xml` | 传递 FSM Topic 参数；`status_frame_id` 显式取 `planning_frame` |
| `planner/plan_manage/launch/run_in_sim.launch` | 统一 goal/cancel/status/bspline/waypoint/pos_cmd 相对 Topic 参数 |
| `planner/plan_manage/launch/simple_run.launch` | 同上 |
| `planner/plan_manage/CMakeLists.txt` | 注册取消时间戳门禁 gtest |
| `planner/bspline_opt/src/bspline_optimizer.cpp` | 保留现有 `in_id/out_id` 初始化保护，将原判断提取为可测试 helper |
| `planner/bspline_opt/CMakeLists.txt` | 注册边界保护 gtest |
| `planner/bspline_opt/package.xml` | 增加 gtest 测试依赖 |
| `uav_simulator/Utils/waypoint_generator/src/waypoint_generator.cpp` | manual goal 生成 Path 时保留源 planning goal 时间戳，不再覆盖为回调时刻 |
| `MissionControl/ego_gazebo_bridge/launch/ego_gazebo_bridge.launch` | 规划内部 Topic 使用相对默认值并向 FSM、traj_server、waypoint_generator、bridge 一致传递 |
| `Swarm/astra_swarm_bringup/launch/uav_tower_stack.launch` | 删除 planner-owned 绝对名补偿 remap；依赖 group namespace 隔离 |

### 2.2 新增

| 文件 | 用途 |
| --- | --- |
| `planner/plan_manage/include/plan_manage/trajectory_cancellation_gate.h` | 可独立测试的 cancel→goal→Bspline 时间戳/轨迹 ID 门禁 |
| `planner/plan_manage/test/trajectory_cancellation_gate_test.cpp` | 7 个取消门禁定向测试 |
| `planner/plan_manage/test/interface_namespace_smoke.launch` | 两套无仿真、无控制的 UAV0/UAV1 ROS 接口 smoke stack |
| `planner/bspline_opt/include/bspline_opt/occupied_segment_guard.h` | 对现有 `in_id >= 0 && out_id > in_id` 判断的可测试封装 |
| `planner/bspline_opt/test/occupied_segment_guard_test.cpp` | 5 个 B 样条占据区间索引边界测试 |

## 3. Topic 修复

### 3.1 修改前

FSM 与 traj_server 的命名规则不一致：

| 接口 | 修改前实现 |
| --- | --- |
| FSM cancel | 参数默认 `/planning/cancel`，绝对名 |
| FSM status | 参数默认 `/planner/status`，绝对名 |
| FSM Bspline | C++ 写死 `/planning/bspline` |
| FSM waypoint | C++ 写死 `/waypoint_generator/waypoints` |
| FSM data display | C++ 写死 `/planning/data_display` |
| traj_server Bspline/cancel/goal | 相对 `planning/...` |
| traj_server PositionCommand | C++ 写死 `/position_cmd` |

实际双机 launch 只能在 group 外围用多个绝对 remap 补偿。任一新 launch 漏掉一项，
就可能回到全局共享 Topic。

### 3.2 修改后

以下 planner-owned 默认值全部为相对名，同时仍可由参数覆盖：

| 参数 | 默认值 |
| --- | --- |
| `fsm/waypoint_topic` | `waypoint_generator/waypoints` |
| `fsm/cancel_topic` | `planning/cancel` |
| `fsm/bspline_topic` | `planning/bspline` |
| `fsm/data_display_topic` | `planning/data_display` |
| `fsm/status_topic` | `planner/status` |
| `traj_server/goal_topic` | `planning/goal` |
| `traj_server/cancel_topic` | `planning/cancel` |
| `traj_server/bspline_topic` | `planning/bspline` |
| `traj_server/position_command_topic` | `planning/pos_cmd` |

FSM 在 `ego_replan_fsm.cpp:28-57、113-122` 读取/校验参数，并用公共
`ros::NodeHandle` 创建接口。traj_server 在 `traj_server.cpp:281-328` 读取、
校验并创建接口。C++ 中没有写死 `/uav0`、`/uav1`、`/uav2`。

### 3.3 UAV0/UAV1 实际解析结果

`interface_namespace_smoke.launch` 在真实 ROS master 上启动了：

- `/uav0/ego_planner_node`
- `/uav0/traj_server`
- `/uav0/waypoint_generator`
- `/uav1/ego_planner_node`
- `/uav1/traj_server`
- `/uav1/waypoint_generator`

实际 Topic 为：

| 接口 | UAV0 | UAV1 |
| --- | --- | --- |
| cancel | `/uav0/planning/cancel` | `/uav1/planning/cancel` |
| status | `/uav0/planner/status` | `/uav1/planner/status` |
| planning goal | `/uav0/planning/goal` | `/uav1/planning/goal` |
| Bspline | `/uav0/planning/bspline` | `/uav1/planning/bspline` |
| waypoint Path | `/uav0/waypoint_generator/waypoints` | `/uav1/waypoint_generator/waypoints` |
| PositionCommand | `/uav0/planning/pos_cmd` | `/uav1/planning/pos_cmd` |

`rostopic info` 结果：

- `/uav0/planning/cancel` 只有 `/uav0/ego_planner_node` 和
  `/uav0/traj_server` 两个订阅者。
- `/uav1/planning/cancel` 只有 `/uav1/ego_planner_node` 和
  `/uav1/traj_server` 两个订阅者。
- UAV0/UAV1 status 各自只有本机 planner publisher。
- UAV0/UAV1 Bspline 各自只连接本机 planner 与本机 traj_server。
- UAV0/UAV1 goal 各自只连接本机 waypoint_generator 与本机 traj_server。
- UAV0/UAV1 waypoint 各自只连接本机 waypoint_generator 与本机 planner。
- UAV0/UAV1 PositionCommand 各自只有本机 traj_server publisher。

向 `/uav0/planning/cancel` 发布一次 `std_msgs/Empty` 后，UAV0 FSM 从
`INIT` 进入 `WAIT_TARGET`，UAV1 FSM 保持 `INIT`；UAV0 traj_server 同时清空
轨迹并停止 PositionCommand。没有观察到跨 namespace 回调。

### 3.4 当前项目实际双机 launch

当前项目实际实例名是 `uav1/uav2`，入口为：

- `Swarm/astra_swarm_bringup/launch/dual_tower_inspection.launch`
- `Swarm/astra_swarm_bringup/launch/uav_tower_stack.launch`

在 `start_sim:=false enable_control:=false` 下完成完整参数展开。结果为：

| 接口 | UAV1 | UAV2 |
| --- | --- | --- |
| cancel | `/uav1/planning/cancel` | `/uav2/planning/cancel` |
| status | `/uav1/planner/status` | `/uav2/planner/status` |
| planning goal | `/uav1/planning/goal` | `/uav2/planning/goal` |
| Bspline | `/uav1/planning/bspline` | `/uav2/planning/bspline` |
| PositionCommand | `/uav1/planning/pos_cmd` | `/uav2/planning/pos_cmd` |

项目 task、sensor、MAVROS 和仍使用 vendor 绝对名的 GridMap 接口继续由 launch
显式配置；本轮没有改变它们，也没有接入或修改 EGO-Swarm 规划算法。

## 4. Frame 参数链路

`advanced_param.xml:70-83` 现在显式令：

```text
planning_frame
  -> ego_planner_node/planning/frame_id
  -> ego_planner_node/fsm/status_frame_id
  -> PlanningVisualization marker frame
  -> waypoint_generator/frame_id
  -> traj_server/traj_server/frame_id
```

FSM 启动时要求 `planning/frame_id` 与 `fsm/status_frame_id` 均非空且完全相同；
不一致时 `ROS_FATAL` 并退出，不再静默出现 `world` 与 `camera_init` 混用。

参数展开证据：

| 实例 | planning | status | PositionCommand | waypoint | RViz marker |
| --- | --- | --- | --- | --- | --- |
| smoke UAV0 | `uav0/camera_init` | `uav0/camera_init` | `uav0/camera_init` | `uav0/camera_init` | `uav0/camera_init` |
| smoke UAV1 | `uav1/camera_init` | `uav1/camera_init` | `uav1/camera_init` | `uav1/camera_init` | `uav1/camera_init` |
| 项目 UAV1 | `uav1/camera_init` | `uav1/camera_init` | `uav1/camera_init` | `uav1/camera_init` | `uav1/camera_init` |
| 项目 UAV2 | `uav2/camera_init` | `uav2/camera_init` | `uav2/camera_init` | `uav2/camera_init` | `uav2/camera_init` |

RViz marker frame 由已有
`traj_utils/src/planning_visualization.cpp:8-15、35-36、71-72、107-108、149-150`
读取同一 `planning/frame_id`。本轮没有增加 TF 转换。

## 5. cancel 后新目标时间戳契约

### 5.1 契约

1. cancel callback 以 `ros::Time::now()` 记录 cancel 时刻。
2. cancel 后仅接受非零且严格晚于 cancel 的 planning goal。
3. 同一 cancel 周期内后续 goal 时间戳必须严格递增。
4. 在有效新 goal 出现前，所有 Bspline 均拒绝。
5. 新 Bspline 的 `start_time` 必须非零且不早于有效新 goal。
6. 被 cancel 的 trajectory ID 持续拒绝，即使已经接受过一条新轨迹。
7. cancel 仍立即令 traj_server `receive_traj_=false` 并清空轨迹，
   PositionCommand timer 不再发布旧轨迹。

实现位置：

- `trajectory_cancellation_gate.h:11-68`
- `traj_server.cpp:27-66`
- FSM 原有 waypoint 时间门禁：
  `ego_replan_fsm.cpp:186-193、311-321`

`waypoint_generator.cpp:87-102、187-192` 对
`manual-lonely-waypoint` 保留 planning goal 的原始时间戳，使 FSM 接收的 Path
和 traj_server 直接接收的 PoseStamped 使用同一时刻。零时间戳不会被中间节点
替换成 `now` 来绕过 cancellation gate。

### 5.2 任务层检查

实际目标发布点已经满足 `goal.header.stamp = ros::Time::now()` 或等价的本轮
`now`：

| 发布链路 | 代码位置 | 结果 |
| --- | --- | --- |
| bridge 转发普通 planning goal | `ego_mavros_bridge.cpp:850-854` | 发布前显式设为 `now` |
| bridge 发布返航 planning goal | `ego_mavros_bridge.cpp:1935-1953` | 发布前显式设为 `now` |
| Stage 2 目标 | `ego_waypoint_mission_node.cpp:558-563` | 每次发布用调用时的 `now` |
| Stage 3 目标 | `sector_inspection_mission_node.cpp:1171-1177、2945-2954` | `makeGoal()` 使用 `ros::Time::now()` |

因此任务层源码无需为本轮额外修改。bridge cancel 在
`ego_mavros_bridge.cpp:896-919` 先使旧目标/控制失效，再发布 planning cancel；
之后到达的新任务目标会重新取当前时间。

### 5.3 单元与 callback 级结果

`trajectory_cancellation_gate_test`：

| 场景 | 期望 | 结果 |
| --- | --- | --- |
| 正常目标与正常轨迹 | 接受 | PASS |
| cancel 后收到 cancel 前旧目标 | 拒绝 | PASS |
| cancel 后零时间戳目标 | 拒绝且门禁保持关闭 | PASS |
| cancel 后等于或早于 cancel 的目标 | 拒绝 | PASS |
| cancel 后有效新时间戳目标 | 接受 | PASS |
| 有效新目标后的新 Bspline | 接受 | PASS |
| 新轨迹成功后再次到达的旧/已取消 Bspline | 拒绝 | PASS |

共 7/7 PASS。

最小 ROS callback 检查还确认：

- 发布零时间戳 planning goal 后，waypoint Path 的时间戳仍为零；
- traj_server 输出明确的 zero/stale/non-increasing 拒绝告警；
- 发布 `header: auto` 的新 goal 后，waypoint Path 保留同一非零时间戳。

## 6. B 样条索引保护

保留了审计报告确认的现有保护：

```cpp
int in_id = -1, out_id = -1;
...
in_id >= 0 && out_id > in_id
```

`bspline_optimizer.cpp:50、96-105` 的行为没有恢复成参考版的未初始化索引。
本轮仅将合法性表达式提取到
`occupied_segment_guard.h:7-10`，生产逻辑仍使用同一条件。

`occupied_segment_guard_test`：

| 边界 | 输入模型 | 当前结果 |
| --- | --- | --- |
| 正常进入和离开 | `in=3, out=7` | 合法，PASS |
| 只有进入、没有离开 | `in=3, out=-1` | 非法，不加入区间，PASS |
| 起始检查段已占据且随后离开 | `in=2, out=4` | 合法，PASS |
| 终止检查段仍占据 | `in=4, out=-1` | 非法，不加入区间，PASS |
| 极短占据区间 | `in=4, out=5` | 合法，PASS |

共 5/5 PASS。

这些是索引保护的定向单元测试，不是带真实 GridMap、控制点采样和优化器求解的
集成测试。当前源码对未形成合法 entry/exit pair 的结果是不加入
`segment_ids`；本轮没有把它改为“规划立即失败”，也没有引入新的忽略策略。
建议下一轮用可控假 GridMap 做 `initControlPoints()` 集成测试，再决定是否需要
把非法区间升级为显式规划失败。

## 7. 构建、测试与静态检查

受影响包构建命令：

```bash
catkin_make \
  -DCATKIN_WHITELIST_PACKAGES='waypoint_generator;bspline_opt;ego_planner;ego_gazebo_bridge;astra_swarm_bringup' \
  -j2
```

结果：PASS。

定向测试：

```bash
catkin_make \
  -DCATKIN_WHITELIST_PACKAGES='bspline_opt;ego_planner' \
  run_tests_bspline_opt run_tests_ego_planner -j2
```

结果：

- 新增取消门禁：7/7 PASS
- 新增 B 样条边界：5/5 PASS
- `catkin_test_results build/test_results/bspline_opt`：
  10 tests，0 error，0 failure，0 skipped
- `catkin_test_results build/test_results/ego_planner`：
  24 tests，0 error，0 failure，0 skipped

所有修改的 XML/launch 均通过 `xmllint --noout`。本轮文件的
`git diff --check` 通过。整个仓库直接执行该检查仍会报告修复前已存在的
`FAST_LIO/Log/mat_pre.txt` 空白问题；该受保护文件未触碰。

构建期间出现环境中已有的 VTK/PCL 可选目标缺失警告，但未导致配置、编译或测试
失败。ROS smoke 启动提示 `~/.ros/log` 超过 1 GB；本轮未清理用户日志。

## 8. 尚未解决的问题

1. B 样条五个边界只完成索引 helper 测试，尚无假 GridMap 驱动的
   `initControlPoints()` 端到端结果。
2. 未在真实 FAST-LIO 点云、odom 与动态 TF 下验证 frame 契约。
3. 未运行实际 `uav1/uav2` 双机节点图；只做了完整 launch 参数展开和独立
   UAV0/UAV1 最小 ROS 图验证。
4. `/use_sim_time` 若在运行中倒退或重置，新 goal 会因不晚于 cancel 而被安全
   拒绝；正式仿真应确保任务期间 `/clock` 单调，或在未来单独设计 clock-reset
   恢复策略。
5. GridMap 的 vendor 绝对 Topic 仍由双机 launch 显式 remap。本轮只统一
   planner-owned goal/cancel/status/waypoint/Bspline/PositionCommand。
6. 纯 `run_in_sim.launch` 现在把内部目标入口统一为相对 `planning/goal`。
   直接使用 RViz goal 的独立演示应显式把 `goal_topic` 覆盖为相对
   `move_base_simple/goal`（在根 namespace 的单机演示中解析为
   `/move_base_simple/goal`）；项目主链仍由 bridge 接收 RViz/任务目标并转发到
   `planning/goal`。

## 9. 下一步最小单机 SITL 回归方案

1. 静态展开单机 launch，确认所有 planning frame 与 Topic 参数，无控制启动。
2. 启动单机 Gazebo/PX4 SITL、MAVROS、FAST-LIO 和 EGO，但 bridge 保持
   dry-run；检查 odom/cloud/goal/status/Bspline frame、时间戳与唯一 publisher。
3. 发布一个带当前时间戳、同 planning frame 的低速近距离目标，确认只产生一条
   新 trajectory ID，status 与 Bspline 均有效，bridge 不控制飞行。
4. 在已有控制权/preflight 检查全部通过后，显式启用单机控制，只执行一个低速、
   固定高度、无遮挡目标，核对 PositionCommand 连续性与跟踪误差。
5. 在一条有效轨迹中途调用 cancel：确认旧 PositionCommand 立即停止、bridge
   HOLD、安全状态与 planner `WAIT_TARGET` 一致。
6. 依次注入零时间戳、旧时间戳和有效新时间戳目标；前两者不得解除门禁，最后一项
   应生成新 trajectory ID，旧 Bspline 重放仍应被拒绝。
7. 最后再做一次单机静态障碍目标，记录最小净空和 B 样条边界行为。通过后才能考虑
   双机 dry-run；本报告不建议也未执行 EGO-Swarm 接入。
