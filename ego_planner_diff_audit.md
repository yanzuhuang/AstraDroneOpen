# 本地 EGO-Planner 与干净参考版完整差异审计

审计日期：2026-07-30  
审计方式：只读源码比较；未修改、覆盖、删除、格式化或移动两个被比较目录中的任何文件，未编译，未启动 ROS/PX4/Gazebo，也未运行飞行仿真。

## 1. 审计基线

### 1.1 比较目录

- 本地修改版：`/home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws/src/Planner/ego-planner`
- 干净参考版：`/home/yanzu/AstraDroneOpen_reference/AstraDrone_ros1_ws/src/Planner/ego-planner`

### 1.2 Git 基线

| 项目 | 分支 | commit SHA | 工作区状态 |
| --- | --- | --- | --- |
| 本地仓库 | `ego-swarm` | `b2b7fa0e52c3b579e0b6bdda5091dc868c039108` | 相对 `origin/ego-swarm` 领先 1；`ego-planner` 目录内无未提交/暂存修改 |
| 干净参考仓库 | `main` | `000b7254d0148f7022f26b92e9fbdac1c0600d70` | 干净，跟踪 `origin/main` |

本地仓库唯一既有工作区修改是比较范围外的
`AstraDrone_ros1_ws/src/SLAM/FAST_LIO/Log/mat_pre.txt`，本轮未触碰。

注意：项目导航文档记录的旧阶段基线与当前实际 HEAD 不同。本报告只描述上述两个当前目录快照之间的差异，不把旧文档中的 `498c7c6` 当作本次参考目录 SHA。

### 1.3 方法和统计口径

使用目录级 `git diff --no-index`，同时启用：

- `--ignore-all-space`
- `--ignore-blank-lines`

并人工检查全部内容差异。`.git`、构建产物、日志、bag、CSV、图片、缓存和临时产物不计入；两个目录中没有出现仅由空格或空行造成的额外差异文件。

统计结果：

| 类型 | 路径数 |
| --- | ---: |
| 本地新增 | 20 |
| 本地删除 | 15 |
| 内容修改 | 12 |
| 合计 | 47 |

差异共计 566 行增加、42 行删除。15 个“删除”和其中 15 个“新增”实际是同一组构建标记从 `CATKIN_IGNORE` 政名为 `CATKIN_IGNORE.disabled`；因此它们是 30 个路径级差异、15 项逻辑构建适配，不能误判成 15 个功能源码删除加 15 个新功能文件。

按区域统计：

| 区域 | 路径级差异数 | 结论 |
| --- | ---: | --- |
| `patches` | 1 | 本地 vendor patch 说明 |
| `planner/plan_manage` | 14 | FSM、取消/状态接口、目标高度、traj_server、launch 和测试 |
| `planner/bspline_opt` | 3 | 一组构建标记替换；一处控制点障碍段边界保护 |
| `planner/plan_env` | 2 | 仅构建标记替换，算法源码无差异 |
| `planner/path_searching` | 2 | 仅构建标记替换，A* 源码无差异 |
| `planner/traj_utils` | 4 | 构建标记替换；RViz frame、namespace、颜色参数化 |
| `uav_simulator` | 21 | 10 组构建标记替换；waypoint frame 适配 |

## 2. 完整文件清单

### 2.1 本地新增文件（20）

功能/测试文件：

1. `patches/0001-astra-goal-height.patch`
2. `planner/plan_manage/include/plan_manage/goal_height.h`
3. `planner/plan_manage/include/plan_manage/planning_status_tracker.h`
4. `planner/plan_manage/test/goal_height_test.cpp`
5. `planner/plan_manage/test/planning_status_tracker_test.cpp`

构建标记文件：

6. `planner/bspline_opt/CATKIN_IGNORE.disabled`
7. `planner/path_searching/CATKIN_IGNORE.disabled`
8. `planner/plan_env/CATKIN_IGNORE.disabled`
9. `planner/plan_manage/CATKIN_IGNORE.disabled`
10. `planner/traj_utils/CATKIN_IGNORE.disabled`
11. `uav_simulator/Utils/cmake_utils/CATKIN_IGNORE.disabled`
12. `uav_simulator/Utils/odom_visualization/CATKIN_IGNORE.disabled`
13. `uav_simulator/Utils/pose_utils/CATKIN_IGNORE.disabled`
14. `uav_simulator/Utils/quadrotor_msgs/CATKIN_IGNORE.disabled`
15. `uav_simulator/Utils/uav_utils/CATKIN_IGNORE.disabled`
16. `uav_simulator/Utils/waypoint_generator/CATKIN_IGNORE.disabled`
17. `uav_simulator/local_sensing/CATKIN_IGNORE.disabled`
18. `uav_simulator/mockamap/CATKIN_IGNORE.disabled`
19. `uav_simulator/so3_control/CATKIN_IGNORE.disabled`
20. `uav_simulator/so3_quadrotor_simulator/CATKIN_IGNORE.disabled`

所有 `CATKIN_IGNORE.disabled` 都只有一行注释
`# Stage 6: package intentionally enabled.`。该文件名不会被 catkin 当作忽略标记。

### 2.2 本地删除文件（15）

本地没有删除规划算法源码。以下均为参考版中的空 `CATKIN_IGNORE`，本地以同目录的 `CATKIN_IGNORE.disabled` 取代：

1. `planner/bspline_opt/CATKIN_IGNORE`
2. `planner/path_searching/CATKIN_IGNORE`
3. `planner/plan_env/CATKIN_IGNORE`
4. `planner/plan_manage/CATKIN_IGNORE`
5. `planner/traj_utils/CATKIN_IGNORE`
6. `uav_simulator/Utils/cmake_utils/CATKIN_IGNORE`
7. `uav_simulator/Utils/odom_visualization/CATKIN_IGNORE`
8. `uav_simulator/Utils/pose_utils/CATKIN_IGNORE`
9. `uav_simulator/Utils/quadrotor_msgs/CATKIN_IGNORE`
10. `uav_simulator/Utils/uav_utils/CATKIN_IGNORE`
11. `uav_simulator/Utils/waypoint_generator/CATKIN_IGNORE`
12. `uav_simulator/local_sensing/CATKIN_IGNORE`
13. `uav_simulator/mockamap/CATKIN_IGNORE`
14. `uav_simulator/so3_control/CATKIN_IGNORE`
15. `uav_simulator/so3_quadrotor_simulator/CATKIN_IGNORE`

分类：全部 `KEEP_LOCAL`。这是当前 AstraDroneOpen catkin 构建启用方式，不是规划行为修改。风险为低到中：全工作空间无白名单构建时会扩大参与构建的包集合，因此后续仍应使用项目既定白名单和下层/上层工作空间顺序。

### 2.3 内容修改文件（12）

1. `planner/bspline_opt/src/bspline_optimizer.cpp`
2. `planner/plan_manage/CMakeLists.txt`
3. `planner/plan_manage/include/plan_manage/ego_replan_fsm.h`
4. `planner/plan_manage/launch/advanced_param.xml`
5. `planner/plan_manage/launch/run_in_sim.launch`
6. `planner/plan_manage/launch/simple_run.launch`
7. `planner/plan_manage/package.xml`
8. `planner/plan_manage/src/ego_replan_fsm.cpp`
9. `planner/plan_manage/src/traj_server.cpp`
10. `planner/traj_utils/include/traj_utils/planning_visualization.h`
11. `planner/traj_utils/src/planning_visualization.cpp`
12. `uav_simulator/Utils/waypoint_generator/src/waypoint_generator.cpp`

## 3. 逐项重要差异

下表的行号以当前本地文件为主；涉及未修改的参考函数时同时给出参考位置。

| 文件 | 函数/行号 | 参考版行为 | 本地行为 | 修改影响 | 分类 | 建议操作 | 风险 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `planner/bspline_opt/src/bspline_optimizer.cpp` | `BsplineOptimizer::initControlPoints()`，本地 50、97–104；参考约 50、93–100 | `in_id/out_id` 未初始化；检测到一对进入/离开标志后无条件加入区间 | 初始化为 `-1`，仅在 `in_id >= 0 && out_id > in_id` 时加入，否则报错并忽略 | 消除未初始化值进入 `segment_ids` 的未定义行为，但确实改动了 B 样条障碍段初始化的底层路径 | `REVIEW` | 不要整文件恢复。为“起点占据、终点占据、短占据段、只有进入没有离开”补定向测试后，将此小补丁独立保存；确认忽略非法区间不会掩盖上游地图/采样错误 | 中 |
| `planner/plan_manage/include/plan_manage/goal_height.h` | `resolveManualGoalHeight()`，9–24 | 不存在；手动目标高度在 FSM 中固定为 1.0 m | 在 `use_goal_height=true` 时使用目标 z，否则使用配置高度；选中值必须有限且大于 0 | 目标接口适配，不改优化器、地图或局部目标算法 | `KEEP_LOCAL` | 保留为独立 vendor patch；运行 launch 必须显式决定是否启用目标 z | 中 |
| `planner/plan_manage/src/ego_replan_fsm.cpp` | `init()`，8–103 | 只加载原始 FSM 参数，部分布尔量依赖默认构造/后续赋值；不提供状态和取消接口 | 显式初始化状态；验证 flight type、waypoint 数量和有限值；加载高度、状态、取消、状态 frame；创建 20 Hz 状态 timer、cancel subscriber 和 latched status publisher | 增加输入防御、任务层状态/取消接口；不改变正常成功路径的规划公式 | `MERGE_MANUALLY` | 保留验证和接口代码，但在双机回归前处理绝对 Topic 默认值及 status frame 与 planning frame 的绑定，见第 7 节 | 高 |
| 同上 | `planGlobalTrajbyGivenWps()`，105–154 | 循环内反复执行 `end_pt_ = wps.back()`；最终一轮才得到已初始化的最后点 | 循环结束后只赋值一次；禁止 preset 模式 0 waypoint；记录规划结果 | 对合法非空 waypoint 的最终 global end point 等价；消除读取尚未填充 `wps.back()` 的不良写法 | `KEEP_LOCAL` | 保留；可加 1 点和 50 点边界单测 | 低 |
| 同上 | `waypointCallback()`，156–230 | 接受首个 pose，目标 z 无条件改成 `1.0`；无空 Path、odom、有限值、取消时间检查 | 拒绝空 Path、无 odom、旧/零时间戳目标和 NaN/Inf；目标 z 由 `resolveManualGoalHeight()` 决定；生成 target ID 并记录结果 | 改变的是 global goal 输入和安全门禁；global trajectory 算法本身未改。默认仍是配置固定高度 1.0 m | `MERGE_MANUALLY` | 保留校验与可配置高度；核对所有任务目标的 `header.stamp`，否则取消后的零时间戳目标会被永久拒绝 | 高 |
| 同上 | `odometryCallback()`，233–268 | 直接复制位置、速度、姿态 | NaN/Inf 时拒绝该帧，否则复制 | FAST-LIO/odom 输入防御，不改坐标转换；本文件仍假设输入已经在正确规划 frame | `KEEP_LOCAL` | 保留；结合 odom 断流和异常值测试 | 低 |
| 同上 | `recordPlanningResult()`、`cancelCallback()`、`statusCallback()`，270–326 | 不存在 | 仅在实际规划调用后累计失败；取消后转 `WAIT_TARGET`；发布 FSM、trajectory ID、碰撞和失败原因 | 提供任务层需要的规划结果和取消接口；状态查询调用占据判断但不改变地图 | `MERGE_MANUALLY` | 保留功能；统一 namespace、Topic 和 frame 参数；验证取消和新目标竞态 | 高 |
| 同上 | `changeFSMExecState()`，328–343；`checkCollisionCallback()`，559–565；`callReboundReplan()`，578–629 | 原状态转换、碰撞重规划和轨迹发布 | 额外记录 emergency 起始时间、失败原因和每次实际规划结果 | 观测状态增强；没有改变成功/失败分支、重规划触发阈值或 B 样条发布内容 | `KEEP_LOCAL` | 保留；状态发布不得被任务层误当作额外重规划次数 | 低 |
| `planner/plan_manage/include/plan_manage/planning_status_tracker.h` | `PlanningStatusTracker`，11–43 | 不存在 | 成功清零连续失败，失败递增，不设置固定上限；只读状态不会增加次数 | 纯状态统计，不是重试控制器 | `KEEP_LOCAL` | 保留 | 低 |
| `planner/plan_manage/src/traj_server.cpp` | `cancelCallback()`、`goalCallback()`、`bsplineCallback()`，25–68 | 接受任意新 Bspline；持续执行当前轨迹 | cancel 清空轨迹并停止 PositionCommand；仅在看到时间戳晚于取消的新 goal 后接受不同/更新的轨迹 | 增加执行层取消门禁，不改 B 样条求值公式；零时间戳 goal 会使门禁无法打开 | `MERGE_MANUALLY` | 保留“取消立即停发”能力；对零时间戳和 namespace 做明确契约或修正后再验收，不能直接整文件恢复 | 高 |
| 同上 | `calculate_yaw()`，113–199 | 直接用 `(time_now-time_last)` 计算 yaw rate，首帧 dt=0 时可能产生 NaN | dt 非有限或 `<=1e-6` 时返回上次 yaw 和 0 yaw rate | 数值安全修复，不改变正常 dt 下前视航向算法 | `KEEP_LOCAL` | 保留并独立成小补丁 | 低 |
| 同上 | `cmdCallback()` 257；`main()` 288–315 | PositionCommand frame 固定 `world`；`time_forward` 不校验 | frame 参数化；校验非空 frame 和非负有限 `time_forward`；订阅 cancel/goal | 必要坐标系和执行接口适配 | `MERGE_MANUALLY` | 保留 frame/参数校验；取消/目标 Topic 应与 FSM 使用同一套相对或显式 namespaced 参数 | 高 |
| `planner/plan_manage/launch/advanced_param.xml` | 37–48、62–76、93–130 | map resolution=0.1、inflation=0.099、ground=-0.01、ceil=2.5、frame=`world` 写死；可视化无 namespace/color 参数 | 把上述值参数化，默认数值基本不变；增加目标高度、planning frame、marker namespace 和颜色 | 工程配置适配。没有修改 `planning_horizon` 的传值规则；默认地图占据参数未变 | `KEEP_LOCAL` | 保留。注意 `visualization_truncate_height` 默认从参考的 2.4 变为与 ceil 相同的 2.5，仅影响显示截断 | 低 |
| `planner/plan_manage/launch/run_in_sim.launch` | 8、44–45、80、88 | planning、traj_server、waypoint frame 依赖原固定值 | 统一传递 `planning_frame`，手动高度默认 1.0 | frame/目标接口适配 | `KEEP_LOCAL` | 保留 | 低 |
| `planner/plan_manage/launch/simple_run.launch` | 8、44–45、80、88 | 同上 | 同上 | frame/目标接口适配 | `KEEP_LOCAL` | 保留 | 低 |
| `planner/traj_utils/include/traj_utils/planning_visualization.h` | `PlanningVisualization` 成员，23–25 | 无可配置 frame、marker namespace、轨迹颜色 | 新增三类配置成员 | RViz/轨迹记录显示接口，不影响规划 | `KEEP_LOCAL` | 保留 | 低 |
| `planner/traj_utils/src/planning_visualization.cpp` | 构造函数 8–21；marker 生成 35–40、71–76、107–111、149–153；`displayOptimalList()` 195–210 | marker frame 混用硬编码 `world`/`map`，无 namespace，最优轨迹固定红色 | 统一使用 `planning/frame_id`，设置 marker namespace，最优轨迹颜色参数化 | 解决 frame 和双机 RViz marker ID/namespace 隔离；不影响规划轨迹 | `KEEP_LOCAL` | 保留；双机应给 UAV0/UAV1 不同 marker namespace | 低 |
| `uav_simulator/Utils/waypoint_generator/src/waypoint_generator.cpp` | `publish_waypoints()` 87–97；`publish_waypoints_vis()` 99–117；`goal_callback()` 147–214；`main()` 251–259 | waypoint/可视化 frame 固定 `world`；不检查目标 frame；每次 callback 重新读取 waypoint type | frame 参数化；拒绝非空且不匹配的 goal frame；waypoint type 启动时读取一次 | 必要 frame 适配；它只做一致性检查，不执行 TF 转换 | `KEEP_LOCAL` | 保留；上游必须先转换 frame。若运行时需要动态修改 `waypoint_type`，本地行为已不再支持，应单独确认 | 中 |
| `planner/plan_manage/CMakeLists.txt` | 13、44、65–81 | 无 Astra 状态消息依赖和新增测试 | 加入 `astra_custom_msgs`、catkin 导出依赖、两个 gtest | 当前项目构建和消息接口适配 | `KEEP_LOCAL` | 保留 | 低 |
| `planner/plan_manage/package.xml` | 54、64、68、76 | 无 Astra 状态消息和 gtest 依赖 | 声明 `astra_custom_msgs` 的 build/export/exec 依赖和 gtest | 当前项目构建适配 | `KEEP_LOCAL` | 保留 | 低 |
| `patches/0001-astra-goal-height.patch` | 1–21 | 不存在 | 记录目标高度 vendor patch 的意图和文件范围；不是可直接 `git apply` 的完整 unified diff | 审计/维护资料 | `KEEP_LOCAL` | 保留，但后续应生成真正可重放且带基线 SHA 的 patch；当前文件只是说明 | 低 |
| 两个新增测试文件 | `goal_height_test.cpp` 12–35；`planning_status_tracker_test.cpp` 8–33 | 不存在 | 验证高度选择和失败计数不受状态发布循环影响 | 回归保护，不影响运行时 | `KEEP_LOCAL` | 保留并扩充取消/时间戳、namespace、B 样条非法区间测试 | 低 |

## 4. 规划核心专项审计

### 4.1 Global trajectory 生成、保存和跟踪

结论：没有发现本地修改 global trajectory 的生成公式、保存容器或正常跟踪算法。

证据：

- `planner/plan_manage/src/planner_manager.cpp` 与参考版逐字节一致。
  - `EGOPlannerManager::planGlobalTrajWaypoints()`：本地 286–364。
  - `EGOPlannerManager::planGlobalTraj()`：本地 366–428。
  - 两者仍通过 `global_data_.setGlobalTraj(gl_traj, time_now)` 保存 global trajectory。
- `planner/plan_manage/include/plan_manage/plan_container.hpp` 与参考版逐字节一致。
  - `GlobalTrajData::setGlobalTraj()`：36–45。
  - global position/velocity/acceleration 的时间修正和读取逻辑未改。
- `planner/plan_manage/src/traj_server.cpp` 的位置、速度、加速度 B 样条解析与 `cmdCallback()` 求值逻辑未改；本地只增加取消门禁、frame 参数和 yaw 首帧 dt 防护。

唯一与 global goal 有关的行为差异是输入端：

- preset waypoint 的 `end_pt_` 赋值从循环内移到循环后，对合法非空列表的最终结果等价；
- manual goal 的 z 从参考版固定 `1.0` 改为“默认配置高度，显式启用时保留 incoming goal z”。

因此不能把当前差异描述成“重写 global trajectory”。它是 global goal 输入契约和安全接口适配。

### 4.2 Local target 和 planning horizon

结论：没有修改 local target 截取规则，也没有把它改成只取前方极短一段。

证据：

- `EGOReplanFSM::getLocalTarget()` 位于
  `planner/plan_manage/src/ego_replan_fsm.cpp:668–717`，与参考版内容一致。
- 它仍以
  `t_step = planning_horizen_ / 20 / max_vel`
  沿 global trajectory 采样，并选择相对 `start_pt_` 距离第一次达到
  `planning_horizen_` 的点；到 global trajectory 末尾时才使用 `end_pt_`。
- `run_in_sim.launch:40` 和 `simple_run.launch:40` 的默认
  `planning_horizon=7.5` 与参考版一致。
- `advanced_param.xml:62` 仍把 launch 的 `planning_horizon` 原样传给 FSM；
  `advanced_param.xml:138` 仍把同一值传给 planner manager。

若运行中观察到 local target 很短，应优先检查实际 launch 展开值、最大速度、global trajectory 几何和 `last_progress_time_`，而不是归因于本地源码差异。该运行现象在没有 rosparam dump/bag 的情况下属于 `REVIEW`。

### 4.3 FSM 状态转换和重规划触发

正常状态转换和触发条件未改：

- `EXEC_TRAJ` 仍在轨迹结束时回到 `WAIT_TARGET`；
- 距终点小于 `thresh_no_replan` 时不重规划；
- 距当前局部轨迹起点小于 `thresh_replan` 时不重规划；
- 其他情况进入 `REPLAN_TRAJ`；
- `planFromCurrentTraj()` 仍依次尝试 current trajectory 初始化、polynomial 初始化、random polynomial 初始化；
- 碰撞检查仍只检查当前局部轨迹前 2/3，并按 `emergency_time_` 决定 emergency stop 或 replan。

本地新增的状态行为只有：

- 显式 cancel 进入 `WAIT_TARGET`；
- 记录 emergency 持续时间和规划结果；
- 拒绝取消前或无有效时间戳的新 waypoint。

这属于工程安全接口，但与同一 FSM 文件混合，故该文件整体分类为 `MERGE_MANUALLY`，不能整文件覆盖参考版。

### 4.4 B 样条初始化、优化和动力学可行性

发现一处底层差异：

- `BsplineOptimizer::initControlPoints()` 对占据区间索引做初始化和合法性检查，详见第 3 节。

未发现以下修改：

- smoothness/collision/feasibility/fitness 代价函数；
- L-BFGS early exit 或梯度公式；
- velocity/acceleration feasibility cost；
- B 样条时间重分配；
- refine optimization；
- 动力学可行性检查阈值。

本地文件中仍可看到固定 `MAX_RESART_NUMS_SET=3` 和 rebound 次数 `<=20`
（`bspline_optimizer.cpp:889、975–976`），但这些代码在参考版中完全相同，不是本地引入。

### 4.5 A* 搜索

`planner/path_searching/src/dyn_a_star.cpp` 与参考版逐字节一致。没有修改：

- A* 邻接扩展；
- heuristic；
- g/f score；
- 搜索终止条件；
- 起终点占据处理；
- 路径回溯。

### 4.6 地图、点云和占据判断

`planner/plan_env/src/grid_map.cpp` 与参考版逐字节一致，`plan_env` 其余源码也无内容差异。没有修改：

- 障碍物膨胀实现；
- 占据概率更新；
- 点云清理或 raycast；
- map resolution 的读取/使用；
- occupancy 判断；
- 局部地图更新范围。

`advanced_param.xml` 只是把原来写死在 launch 中的 resolution、inflation、ground、ceil 和 frame 变成参数，默认占据数值不变。Mid360/FAST-LIO 的具体点云过滤或 remap 不在本次两个目录的差异中；本地 EGO 内只看到通用 odometry/frame 接口防御。

### 4.7 场景硬编码和临时验证逻辑

在本地差异源码和 launch 中没有发现以下字符串或等价新增逻辑：

- `radio_tower` / tower / 铁塔；
- 8 扇区或 sector；
- `ENTRY_GATE`；
- `EXIT_GATE`；
- `worksite`；
- 固定塔坐标；
- 固定障碍物坐标；
- 为单次绕塔验证写入 EGO optimizer、A* 或 map 的分支。

新增目标高度测试使用 4、8、16 m 作为测试样例，但运行代码没有写死 8 m 塔任务；默认固定高度来自参数，incoming goal z 是显式 opt-in。

`uav_simulator/mockamap` 和 `map_generator` 原本就包含示例/随机障碍生成代码，且其源码与参考版相同，不能算本地新增的场景硬编码。

## 5. 固定 12 次重规划专项结论

结论：没有发现本地加入“固定 12 次”“最多 12 次”“循环 12 次”或第 12 次后改变规划行为的代码。

具体证据：

- 新增 `PlanningStatusTracker::recordAttempt()` 只按实际调用递增
  `consecutive_failures_`，没有阈值、停止条件或 12 的比较。
- `statusCallback()` 读取计数，不会触发规划或增加次数。
- `GEN_NEW_TRAJ` 和 `REPLAN_TRAJ` 的原始重复调用行为与参考版一致。
- `planFromCurrentTraj()` 每次最多按三种初始化方式尝试，这是参考版已有行为。
- optimizer 中 3 次 restart、20 次 rebound 上限也是参考版已有行为。
- 源码中与数字 12 有关的多项命中来自多项式系数矩阵列偏移、消息序列化长度或参数 `p_min=0.12`，均与重规划次数无关。

如果历史运行日志恰好显示 12 次失败，应把它视为运行时结果而不是当前 EGO 源码的固定上限；需要结合当时日志时间轴、状态 Topic 和实际 launch 参数进一步确认。

## 6. 工程接口适配摘要

建议保留的本地工程适配：

1. 15 组 `CATKIN_IGNORE` 改名，启用当前 catkin 包。
2. `astra_custom_msgs::PlannerStatus` 依赖、状态统计和测试。
3. manual goal z 的显式 opt-in 适配及单测。
4. odometry NaN/Inf 防御。
5. planning/traj/waypoint/RViz frame 参数化。
6. RViz marker namespace 和颜色参数化，适合 UAV0/UAV1 显示隔离。
7. waypoint goal frame 一致性检查。
8. traj_server 首帧 yaw dt=0 防护。
9. cancel 后停止 PositionCommand 的安全能力。
10. map 参数从 launch 常量改为可配置参数，默认规划数值不变。

没有在该目录差异中看到直接的 Mid360 点云回调、FAST-LIO 点云过滤、PX4/MAVROS 发布器或 UAV0/UAV1 专用 Topic 名。它们若存在，位于本次明确禁止扩展比较的 AstraDroneOpen 其他目录；本报告不对目录外实现作差异结论。

## 7. 风险最高的修改

### 7.1 取消门禁依赖非零且单调的新目标时间戳

位置：

- `planner/plan_manage/src/ego_replan_fsm.cpp:158–163`
- `planner/plan_manage/src/traj_server.cpp:34–68`

FSM 明确拒绝 cancel 后的零时间戳 waypoint。traj_server 在 cancel 后如果收到的
`planning/goal.header.stamp` 为零，则 `post_cancel_goal_time_` 仍为零，后续 Bspline
会一直满足 `predates_goal`，从而持续被拒绝。该策略是 fail-closed，但要求任务层严格发布非零 ROS 时间戳。

分类：`MERGE_MANUALLY`，风险高。应先做 callback 级测试，再做无控制 ROS 图测试。

### 7.2 双机 namespace 默认值不一致

位置：

- FSM 默认 `fsm/status_topic="/planner/status"`、
  `fsm/cancel_topic="/planning/cancel"`：
  `ego_replan_fsm.cpp:27–29`
- traj_server 使用相对 Topic：
  `traj_server.cpp:288–289`

绝对 Topic 默认值会绕过 node namespace，而 traj_server 的相对 Topic 会进入 namespace。
本目录的 `advanced_param.xml` 没有显式设置 FSM 的 status/cancel Topic。若直接在
`/uav0`、`/uav1` 下启动，可能出现两个 FSM 共享绝对 cancel/status，而各 traj_server
监听各自相对 cancel 的不一致。

分类：`MERGE_MANUALLY`，风险高。不能因为“加入了 namespace 支持”就默认已经完成双机隔离，必须以 launch 展开和 ROS master publisher/subscriber 图验证。

### 7.3 状态 frame 默认值未与 planning frame 绑定

FSM 的 `status_frame_id_` 默认 `camera_init`，而 `advanced_param.xml` 的
`planning_frame` 默认 `world`，且没有把后者传给 `fsm/status_frame_id`。状态消息可能
声称碰撞布尔值属于 `camera_init`，实际 `end_pt_`、odom 和 map 使用的是另一规划 frame。

分类：`MERGE_MANUALLY`，风险中到高。应使 status frame 明确继承 planning frame，或由上层 launch 显式传入同一值。

### 7.4 B 样条非法占据区间被忽略

本地修复避免了未初始化索引，但“记录错误后继续”是否总比“本次规划失败”更安全，仅靠静态代码无法完全确定。若非法区间意味着地图/控制点状态已经不一致，忽略它可能让后续优化少处理一个障碍段。

分类：`REVIEW`，风险中。需要定向单测决定是“忽略该区间”还是“立即返回失败”。

### 7.5 所有 simulator 包被启用

10 个 `uav_simulator` 包的 `CATKIN_IGNORE` 均被禁用。它符合当前项目构建适配要求，
但会扩大无白名单构建时的依赖面。分类仍为 `KEEP_LOCAL`；风险通过既定白名单构建流程控制。

## 8. 分类总览

| 分类 | 结论 |
| --- | --- |
| `KEEP_LOCAL` | 构建标记、目标高度选择器和测试、状态统计器、odom 有限值校验、frame/marker 参数化、yaw dt 防护、CMake/package 适配 |
| `MERGE_MANUALLY` | `ego_replan_fsm.h/.cpp` 和 `traj_server.cpp`：同文件混有必要接口适配、取消门禁和需要复核的 Topic/frame 契约，禁止整文件覆盖 |
| `REVIEW` | `bspline_optimizer.cpp::initControlPoints()` 的非法占据区间处理；运行中若出现短 local target，也需结合参数/bag 复核 |
| `RESTORE_REFERENCE` | 本次没有发现可安全建议直接恢复参考实现的独立差异段 |
| `IGNORE` | 纯格式、空行、日志、bag、CSV、图片、缓存、临时产物；本次 47 个计入差异的路径中没有仅格式差异项 |

“没有 `RESTORE_REFERENCE`”不表示所有本地代码已经完成运行验收；它只表示当前发现的核心差异不是铁塔/扇区/固定 12 次等应立即移除的场景污染。高风险接口仍必须人工合并和测试。

## 9. 建议后续恢复/整理顺序

本轮禁止改源码。若负责人审阅后决定整理 vendor 差异，建议顺序如下：

1. 固定本报告的两个 SHA，保存完整可重放 diff；不要直接覆盖整个 `ego-planner`。
2. 先保留所有 `CATKIN_IGNORE.disabled`、CMake/package、frame、marker 和 yaw dt 防护。
3. 将目标高度改动整理成真正的 unified vendor patch，并记录基线 SHA；保留默认兼容行为，但项目 launch 应显式设置 `use_goal_height`。
4. 对 FSM 的状态/取消接口逐段人工合并：先统一相对/绝对 Topic 和 UAV namespace，再统一 status/planning frame，最后保留输入校验和状态统计。
5. 对 traj_server 逐段人工合并：保留 frame 和 dt 防护；在时间戳契约明确后再保留或调整 cancellation gate。
6. 单独审查 `initControlPoints()` 小补丁；根据定向测试决定非法区间应忽略还是使本次规划失败。
7. `planner_manager.cpp`、`plan_container.hpp`、`plan_env`、`path_searching` 当前已与参考版一致，无需“恢复”。
8. 最后再审查 `uav_simulator` 构建启用范围；不要把 simulator 的构建选择与 EGO 规划核心混为一项回滚。

## 10. 建议回归测试顺序

后续经负责人批准后，建议按风险从低到高执行：

1. 静态差异复核：确认仅保留批准的 vendor patch，检查所有 Topic、frame、namespace 的 launch 展开值。
2. 纯单元测试：
   - `goal_height_test`
   - `planning_status_tracker_test`
   - 新增 cancel/零时间戳/旧时间戳/新时间戳测试
   - 新增 `initControlPoints()` 非法占据区间测试
3. 目标包白名单构建和测试，不启动 Gazebo/PX4。
4. ROS 无控制接口测试：
   - UAV0/UAV1 cancel/status/goal/bspline Topic 隔离
   - `world/map/camera_init` frame 一致性
   - cancel 后 PositionCommand 立即停止
   - 新目标是否可靠解除 cancellation gate
5. 确定性 planner 测试：
   - 单 global goal、多个 global waypoint
   - `planning_horizon` 边界和 global 末端 local target
   - 不可达目标和重复 replan，确认没有固定 12 次行为
6. 静态地图测试：
   - 起点/终点占据
   - 短障碍段
   - 膨胀半径、分辨率、地面和虚拟天花板参数边界
   - B 样条可行性和碰撞检查
7. 只读/无控制集成验证：FAST-LIO odom/点云 frame、时戳、有限值和 EGO 输出健康。
8. 最后才在明确控制授权、安全门禁和唯一 MAVROS 控制出口成立后，按单目标、双目标、完整任务逐级进行 SITL 飞行回归。

## 11. 最终结论

1. 本地相对参考版共有 47 个忽略空白后的路径级差异：20 新增、15 删除、12 修改。
2. 15 个删除全部是空 `CATKIN_IGNORE`；没有删除 EGO 规划源码。
3. A*、地图/点云占据算法、global trajectory manager/container、local target 截取规则均未修改。
4. 没有发现固定 12 次重规划、铁塔、8 扇区、ENTRY_GATE、EXIT_GATE、固定塔坐标或一次性仿真障碍硬编码进入 EGO 底层。
5. B 样条核心只有一处非法占据区间索引防护，建议定向测试后作为独立补丁保留或调整。
6. 大部分差异属于构建、目标、状态/取消、frame、namespace/RViz 和执行安全适配，应保留。
7. 最高风险不在搜索/优化算法，而在取消后的时间戳门禁、绝对/相对 Topic 混用以及 status frame 与 planning frame 默认不一致。
8. `ego_replan_fsm.*` 和 `traj_server.cpp` 不能整文件恢复参考版，必须逐段人工合并。
