# AstraDroneOpen 代码审计报告

## 0. 报告元数据

- 项目路径：/home/yanzu/AstraDroneOpen
- 审计日期：2026-07-20
- 审计基线分支：ego-project
- 审计基线提交：498c7c6
- 对比范围：main...HEAD
- 审计方式：只读代码检查、配置解析、语法检查及已有测试程序验证
- 审计期间未修改、新增、删除或重命名任何文件

注意：首次将本报告写入仓库前复核时，仓库处于 detached HEAD 898614d（ego-project~1），且工作区干净；当前分支名称已更新为 ego-project。因此，本报告中的差异统计、逐文件列表和代码结论仍以实际审计时的 498c7c6 为准。

## 1. 执行摘要

当前代码可以作为仿真验证原型和后续重构起点，但不适合作为正式工程落地的直接基线。

正面因素：

- 对真机误运行有 /use_sim_time 硬保护。
- Stage 6 默认 enable_control=false，不主动向 MAVROS 输出控制指令。
- 新增了输入时效、有限值、落地状态、TF 和部分控制权检查。
- EGO-Planner 到 MAVROS 的基本链路已经形成。
- 新增数学辅助代码具有基础单元测试。
- 审计基线源码通过只读语法检查，launch、XML、YAML 和脚本基础检查正常。

阻止正式落地的主要因素：

1. 默认软降落参数超出当前 PX4 声明的支持范围。
2. 坐标对齐检查发生在解锁和起飞之后，检查时机过晚。
3. dry-run 没有真正验证规划/MAVROS 对齐和任务边界。
4. EGO 动态轨迹指令被降级成纯位置跟随，速度、加速度和 yaw_dot 被丢弃。
5. 两个控制节点均超过 1,100 行，状态机、参数、ROS I/O 和安全逻辑高度耦合。
6. 控制权冲突检测只覆盖一个 setpoint Topic。
7. EGO 原始核心代码被直接修改，且混入了与当前集成非必需的改动。
8. 外部 PX4 环境不是可复现的干净版本。
9. 文档中存在强制上锁、降落模式和 Stage 6 架构等严重过期或互相矛盾的描述。
10. 缺少当前 checkout 的完整干净编译、状态机测试、故障注入测试和长时间 SITL 证据。

## 2. 分支、工作区和差异状态

审计执行时：

- 当前分支：ego-project，符合预期。
- 工作区：干净，无暂存、未暂存或未跟踪文件。
- main 与当前分支的 merge-base 均为 9a8a662。
- HEAD：498c7c6。
- 相对 main 新增 10 个提交。
- main...HEAD：76 个改动文件，14,202 行新增，1,807 行删除。
- git diff --check 仅发现 FAST_LIO/Log/mat_pre.txt 中的尾随空白；排除该运行日志后没有空白错误。

外部运行环境：

- PX4 路径：/home/yanzu/PX4-Autopilot
- PX4 提交：99c40407ffd7ac184e2d7b4b293f36f10fe561ef
- 描述：v1.15.4-dirty
- 状态：detached HEAD，Gazebo Classic 子目录为脏状态，并存在未跟踪启动目录

外部 PX4 不属于当前仓库的 76 个改动，但会影响 Stage 6 的可复现性。

## 3. 高优先级问题

### 3.1 PX4 软降落参数超出声明范围

autoarming_control.launch 和 stage4_trajectory.launch 默认开启 configure_px4_soft_landing=true，并请求：

    MPC_LAND_SPEED = 0.30

当前 PX4 v1.15.4 对 MPC_LAND_SPEED 声明的最小值是 0.6 m/s，默认值是 0.7 m/s。当前请求值处于声明支持范围之外。

若 PX4 或 MAVROS 拒绝、修正该值，控制器将拒绝解锁；即使底层允许写入，也属于未受支持、未验证的配置。正式工程不能依赖本机恰好接受该值的行为。

涉及位置：

- AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src/autoarming_control.cpp:386
- AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/launch/autoarming_control.launch:26
- AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/launch/stage4_trajectory.launch:48

### 3.2 坐标对齐检查发生在起飞之后

bridge 在 WAIT_INPUTS、PRESTREAM、ARM_OFFBOARD 和 TAKEOFF 阶段只检查输入新鲜度。直到 HOVER_READY 准备进入 EGO 跟踪时，才调用 plannerAlignmentValid。

因此错误的 map 与 camera_init 关系不会阻止解锁和起飞，只会阻止起飞后的规划跟踪。

stage6_gazebo.launch 默认发布单位变换：

    map -> camera_init = identity

该假设在特定仿真启动顺序下可能成立，但没有自动校准或解锁前证据。正式工程应在解锁前完成 TF、位置、航向、home 和 child-frame 外参检查。

### 3.3 dry-run 检查范围不足

kDryRun 只调用 baseInputsFresh，不会：

- 捕获 home；
- 检查规划里程计与 MAVROS 位姿对齐；
- 检查目标包络；
- 检查规划指令能否正确变换；
- 检查控制权冲突。

因此日志中的 Inputs healthy 只能证明基础消息存在且新鲜，不能证明系统已经满足启用控制的条件。

### 3.4 EGO PositionCommand 语义被削弱

quadrotor_msgs/PositionCommand 包含 position、velocity、acceleration、yaw、yaw_dot 和控制增益。

bridge 只保留位置和 yaw，再以默认 0.5 m/s 做位置步进限制，最后发布 PoseStamped。这使系统成为限速的位置追随器，而不是忠实执行 EGO 动态轨迹。

可能后果：

- 实际轨迹长期滞后于规划轨迹；
- 规划器预测位置与飞行器实际位置偏离；
- 障碍物附近的安全裕量失真；
- 急转弯和重规划时无法复现规划动力学。

正式方案需要明确选择：

- 使用 MAVROS raw local setpoint 传递位置、速度和加速度；或
- 明确将 EGO 仅作为路径生成器，并单独设计路径跟随器和安全裕量。

### 3.5 控制权冲突检测不完整

bridge 只检查 /mavros/setpoint_position/local 的其他发布者，没有检查：

- /mavros/setpoint_raw/local
- /mavros/setpoint_velocity/*
- /mavros/setpoint_raw/attitude
- 姿态和推力控制接口

旧 autoarming_control 同时注册 position 和 raw-local 发布器。两个节点同时运行时，现有检查不足以证明控制权唯一。

pc_example.sh 默认仍可能启动旧控制器，而 Stage 6 的进程冲突检查没有覆盖 autoarming_control、position_control、pix_tracker 等可能的控制节点。

### 3.6 状态机缺少完整终止策略

autoarming_control.cpp：

- 共 1,299 行，main 函数从约第 294 行延伸到文件末尾；
- FCU、pose、extended state 的初始等待没有超时；
- /mavros/state 没有独立时间戳看门狗；
- 非降落阶段 pose 超时后可能无限保持；
- AUTO.LAND 和服务失败可以持续重试；
- 修改 PX4 全局参数后不恢复；
- 多个浮点参数没有统一 isfinite 检查。

ego_mavros_bridge.cpp：

- 共 1,158 行；
- controlTimerCallback 承担整个状态机；
- 起飞没有总超时；
- 降落和模式服务失败没有有限的失败终点；
- 控制冲突发生后仍可能继续与另一个发布者竞争；
- stepToward 使用配置周期，而不是实际 timer dt。

### 3.7 外部依赖不可复现

Stage 6 脚本默认依赖：

- /opt/ros/noetic
- 用户目录下的 PX4-Autopilot
- 特定 NVIDIA 驱动策略

脚本显式拒绝 NVIDIA 570，并偏向已验证的 535；即使在 headless 场景也将 RViz 纳入主要启动路径，Topic 等待循环没有超时。

正式工程需要锁定：

- PX4 commit 或 tag；
- Gazebo 子模块版本；
- ROS 依赖；
- airframe 和参数；
- GPU 与无 GPU 两套启动路径；
- 容器或明确的环境安装清单。

## 4. EGO、PX4、Gazebo、MAVROS 接口审计

### 4.1 启动关系

当前 Stage 6 启动顺序：

1. 启动 ROS master、PX4 SITL、Gazebo、MAVROS 和 iris_mid360。
2. 等待 /livox/imu/header。
3. 启动 FAST-LIO。
4. 等待 /Odometry/header 和 /cloud_registered/header。
5. 启动 EGO-Planner、traj_server、waypoint_generator、bridge 和 RViz。
6. 默认处于 dry-run；显式启用后才进入 OFFBOARD 控制。

该顺序总体合理，但 Topic 等待没有超时，也没有在解锁前建立完整的坐标系、控制权和 PX4 参数契约。

### 4.2 Topic、消息和坐标系

| 链路 | Topic/消息 | 坐标系 | 结论 |
|---|---|---|---|
| Gazebo MID360 到 FAST-LIO | /livox/lidar、/livox/imu | 传感器/body | 配置匹配 |
| FAST-LIO 到 EGO | /Odometry，nav_msgs/Odometry | camera_init 到 body | EGO odom remap 正确 |
| FAST-LIO 到 EGO map | /cloud_registered，PointCloud2 | camera_init | Topic 正确，但 EGO 自身不验证或变换 frame |
| FAST-LIO TF 到 MAVROS vision | camera_init 到 body | 外部视觉 | MAVROS vision_pose.tf.listen=true，链路成立 |
| MAVROS 到 bridge | /mavros/local_position/pose | map 到 base_link | 与 FAST-LIO child frame 不完全相同，需要外参契约 |
| RViz goal 到 bridge | /move_base_simple/goal | 任意可 TF frame | 变换后发布 /planning/goal |
| waypoint generator 到 EGO FSM | /waypoint_generator/waypoints，Path | camera_init | 已增加 frame 检查 |
| EGO 到 traj_server | /planning/bspline | 规划坐标系 | 正常 |
| traj_server 到 bridge | /planning/pos_cmd，PositionCommand | camera_init | frame 已参数化，但动态字段被 bridge 丢弃 |
| bridge 到 MAVROS | /mavros/setpoint_position/local，PoseStamped | ROS ENU map | MAVROS 再转换为 PX4 NED |
| offboard 降落到 MAVROS | /mavros/setpoint_raw/local，PositionTarget | ROS ENU 数据、LOCAL_NED 枚举 | z 下降使用负速度的处理基本正确 |

关键约束：

- EGO GridMap::cloudCallback 直接把点云数值当作地图坐标，没有读取或验证 header.frame_id，也没有 TF 转换。因此 /cloud_registered 必须严格处于 EGO 地图坐标系。
- bridge 虽然检查点云 frame，但只是控制门禁；EGO 自身仍会处理错误 frame 的点云。
- camera_init 到 body 与 map 到 base_link 的 child frame 不同。即使世界原点一致，也应明确 IMU/body 到机体基准点的外参。
- planning_horizon=7.5 m，地图局部更新范围为 5.5 m，最大射线长度为 4.5 m。需明确未知空间策略。
- 手动目标的 z 被 EGO FSM 替换为 manual_target_height，RViz 输入高度没有保留。

## 5. 代码质量审计

### 5.1 重复代码

- Stage 3 和 Stage 4 launch 重复大量降落、超时和安全参数。
- offboard 与 bridge 分别实现解锁、OFFBOARD、起飞、保持、返航、降落、服务重试和状态检查。
- stage 系列文档、studymap.md、studynote.md、order.md 大量重复 ROS、PX4、启动和安全说明。
- 15 个 CATKIN_IGNORE.disabled 内容相同，只是人为标记。

### 5.2 硬编码

- offboard 多处固定 frame 为 map。
- pc_example.sh 固定项目位于用户 HOME 下的 AstraDroneOpen。
- Stage 6 固定 ROS Noetic、PX4 默认目录和 NVIDIA 驱动策略。
- bridge 默认 identity map 到 camera_init。
- EGO 默认目标高度强制为 1 m。
- analyzer 使用固定绝对 Topic。
- PX4 参数由任务节点直接修改且不恢复。

### 5.3 模块耦合

- 控制状态机直接耦合 ROS subscriber、publisher、service、参数解析和任务轨迹。
- bridge 同时承担 goal 适配、TF、控制权、飞行状态机、降落和 PositionCommand 跟随。
- 启动脚本同时承担环境检测、进程管理、GPU策略、Topic 健康检查和 UI 启动。
- EGO vendor 代码与项目定制修改没有隔离层。

### 5.4 异常处理

已有优点：

- 多数 ROS 服务返回值会检查。
- TF 异常会捕获。
- pose、odom、cloud 和 command 有部分 freshness 检查。
- 落地后正常 disarm 依赖 fresh ON_GROUND。
- ROS master 查询失败按控制权冲突处理，倾向安全。

不足：

- 初始等待和多个飞行阶段缺少总超时。
- 部分服务请求可无限重试。
- FCU state 没有严格 freshness。
- AUTO.LAND 失败缺少确定的最终状态。
- 参数缺少统一的有限值、范围和跨参数关系校验。

### 5.5 注释和文档准确性

- autoarming_control.cpp 文件头称 pose/OFFBOARD 异常回退 AUTO.LAND；实际非降落阶段只冻结任务。
- stage3.md 将当前降落描述为全面改成 AUTO.LAND；实际正常路径仍由 OFFBOARD 软降落并正常 disarm，AUTO.LAND 只是故障回退。
- stage2.md 仍大量指导使用 MAVLink 400/21196 强制上锁，与当前源码和安全策略不一致。
- studymap.md 称 Stage 6 只使用 EGO 原生 mock/SO3，不连接 FAST-LIO/PX4，与当前实现相反。
- studymap.md 仍称 world 路径缺少斜杠，但该 launch 已修复。
- stage3_waypoints.yaml 把同时改变高度的动作描述为原地转向。
- 降落接触辅助减少推力的注释，与实际增加向下速度指令不完全一致。

## 6. EGO-Planner 原始核心修改评估

| 修改 | 判断 |
|---|---|
| bspline_optimizer.cpp 初始化索引并拒绝非法段 | 通用健壮性修复，但不是 Stage 6 集成必需；应作为独立补丁或上游贡献 |
| ego_replan_fsm 初始化标志、检查有限值 | 有价值，但应与集成逻辑分离 |
| FSM 仅允许 flight type 1 和 2 | 可能破坏原始 REFERENCE_PATH=3 能力，属于兼容性回退 |
| FSM 固定 manual_target_height | 项目需要，但直接改变原始 goal z 语义 |
| traj_server.cpp 处理零或非法 dt | 控制安全相关，修改合理 |
| traj_server.cpp 参数化 command frame | camera_init 集成所必需，修改合理 |
| visualization frame 参数化 | 合理，但只影响显示 |
| waypoint generator frame 参数化并拒绝错误 frame | 合理，但应维护为小型 vendor patch |
| advanced_param.xml 增加 frame、高度和膨胀参数 | 合理的集成配置 |
| run_in_sim.launch、simple_run.launch 修改 | 只有保留 EGO 原生仿真时需要，不是最终 PX4 链路必需 |

建议：

- 将 EGO-Planner 固定在可识别的上游版本。
- 维护少量可重放的补丁。
- 集成逻辑优先放在 bridge、launch 和配置层。
- 每个核心补丁记录上游版本、修改原因、测试和撤销方法。

另外，未由本分支引入但在语法检查中发现：EGO 的 polynomial_traj.h 中 getMeanVel 存在无返回值路径。Catkin 当前通过 system include 隐藏相关告警，不应视为无风险。

## 7. 全部 76 个改动文件及分类

路径缩写：

- OB：AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard
- BR：AstraDrone_ros1_ws/src/MissionControl/ego_gazebo_bridge
- EP：AstraDrone_ros1_ws/src/Planner/ego-planner/planner
- ES：AstraDrone_ros1_ws/src/Planner/ego-planner/uav_simulator

对已删除的 CATKIN_IGNORE，可以保留表示保留其删除状态；暂时无法确定表示需决定是否恢复。

### 7.1 Offboard：10 个

| 文件 | 作用 | 分类 |
|---|---|---|
| OB/CMakeLists.txt | 构建控制器、测试和分析脚本 | 需要重构：缺少完整 install-space 规则 |
| OB/config/stage3_waypoints.yaml | Stage 3 航点任务 | 需要重构：注释与动作不一致 |
| OB/include/offboard/landing_profile.h | S 曲线软降落速度模型 | 可以保留 |
| OB/include/offboard/trajectory_reference.h | 圆、方形、8 字轨迹参考生成 | 可以保留；正式使用前评估方形拐角 |
| OB/launch/autoarming_control.launch | 航点任务启动和安全参数 | 需要重构：与 Stage 4 大量重复 |
| OB/launch/stage4_trajectory.launch | Stage 4 轨迹任务启动 | 需要重构 |
| OB/package.xml | ROS 包依赖元数据 | 需要重构：maintainer 和 license 为占位内容 |
| OB/scripts/stage4_analyze_bag.py | rosbag 轨迹误差分析 | 需要重构：Topic 固定且未做时间同步 |
| OB/src/autoarming_control.cpp | 起飞、任务、返航和降落状态机 | 需要重构：1,299 行、职责过多 |
| OB/test/trajectory_reference_test.cpp | 轨迹和降落数学测试 | 可以保留 |

### 7.2 EGO/MAVROS bridge：11 个

| 文件 | 作用 | 分类 |
|---|---|---|
| BR/CMakeLists.txt | bridge 构建、安装和测试 | 可以保留 |
| BR/config/stage6_gazebo.yaml | Stage 6 超时、速度、frame 和 Topic 参数 | 需要重构：补齐坐标和感知契约 |
| BR/include/ego_gazebo_bridge/command_utils.h | 命令校验、角度和步进工具接口 | 可以保留 |
| BR/include/ego_gazebo_bridge/ego_mavros_bridge.h | bridge 状态和 ROS 成员定义 | 需要重构：类职责过大 |
| BR/launch/stage6_gazebo.launch | EGO、traj_server、waypoint、bridge、RViz 总启动 | 需要重构：默认 identity TF 风险 |
| BR/package.xml | bridge ROS 包依赖 | 需要重构：maintainer 为占位内容 |
| BR/rviz/stage6_gazebo.rviz | Stage 6 可视化配置 | 可以保留 |
| BR/src/command_utils.cpp | 命令有限值、距离和限速工具 | 可以保留；后续补四元数规范化 |
| BR/src/ego_mavros_bridge.cpp | 坐标变换、goal 适配和飞行状态机 | 需要重构：1,158 行及控制语义问题 |
| BR/src/ego_mavros_bridge_node.cpp | bridge 节点入口 | 可以保留 |
| BR/test/command_utils_test.cpp | bridge 工具单元测试 | 可以保留，需扩展状态机测试 |

### 7.3 EGO planner：19 个

| 文件 | 作用 | 分类 |
|---|---|---|
| EP/bspline_opt/CATKIN_IGNORE（删除） | 启用 B-spline 优化包 | 可以保留 |
| EP/bspline_opt/CATKIN_IGNORE.disabled | 说明包已启用 | 建议删除 |
| EP/bspline_opt/src/bspline_optimizer.cpp | B-spline 优化核心 | 需要重构：隔离为 vendor patch |
| EP/path_searching/CATKIN_IGNORE（删除） | 启用路径搜索包 | 可以保留 |
| EP/path_searching/CATKIN_IGNORE.disabled | 启用标记 | 建议删除 |
| EP/plan_env/CATKIN_IGNORE（删除） | 启用地图环境包 | 可以保留 |
| EP/plan_env/CATKIN_IGNORE.disabled | 启用标记 | 建议删除 |
| EP/plan_manage/CATKIN_IGNORE（删除） | 启用规划管理包 | 可以保留 |
| EP/plan_manage/CATKIN_IGNORE.disabled | 启用标记 | 建议删除 |
| EP/plan_manage/include/plan_manage/ego_replan_fsm.h | FSM 新增状态和高度参数 | 需要重构：vendor 核心修改 |
| EP/plan_manage/launch/advanced_param.xml | EGO 核心参数和 Topic remap | 需要重构：参数契约需集中 |
| EP/plan_manage/launch/run_in_sim.launch | EGO 原生仿真启动 | 需要重构或与正式链路分离 |
| EP/plan_manage/launch/simple_run.launch | 简化 EGO 仿真启动 | 需要重构或与正式链路分离 |
| EP/plan_manage/src/ego_replan_fsm.cpp | 目标输入、状态机和全局轨迹 | 需要重构：含语义改变和兼容性回退 |
| EP/plan_manage/src/traj_server.cpp | B-spline 转 PositionCommand | 需要重构：保留必要补丁并隔离 |
| EP/traj_utils/CATKIN_IGNORE（删除） | 启用轨迹工具包 | 可以保留 |
| EP/traj_utils/CATKIN_IGNORE.disabled | 启用标记 | 建议删除 |
| EP/traj_utils/include/traj_utils/planning_visualization.h | 可视化 frame 参数接口 | 需要重构为独立小补丁 |
| EP/traj_utils/src/planning_visualization.cpp | 可视化 marker frame 输出 | 需要重构为独立小补丁 |

### 7.4 EGO 原生模拟器：21 个

| 文件 | 作用 | 分类 |
|---|---|---|
| ES/Utils/cmake_utils/CATKIN_IGNORE（删除） | 启用原生模拟器 CMake 工具 | 暂时无法确定 |
| ES/Utils/cmake_utils/CATKIN_IGNORE.disabled | 启用标记 | 建议删除 |
| ES/Utils/odom_visualization/CATKIN_IGNORE（删除） | 启用 odom 可视化 | 暂时无法确定 |
| ES/Utils/odom_visualization/CATKIN_IGNORE.disabled | 启用标记 | 建议删除 |
| ES/Utils/pose_utils/CATKIN_IGNORE（删除） | 启用 pose 工具 | 暂时无法确定 |
| ES/Utils/pose_utils/CATKIN_IGNORE.disabled | 启用标记 | 建议删除 |
| ES/Utils/quadrotor_msgs/CATKIN_IGNORE（删除） | 启用 EGO 消息定义 | 可以保留，Stage 6 必需 |
| ES/Utils/quadrotor_msgs/CATKIN_IGNORE.disabled | 启用标记 | 建议删除 |
| ES/Utils/uav_utils/CATKIN_IGNORE（删除） | 启用 UAV 工具 | 暂时无法确定 |
| ES/Utils/uav_utils/CATKIN_IGNORE.disabled | 启用标记 | 建议删除 |
| ES/Utils/waypoint_generator/CATKIN_IGNORE（删除） | 启用目标生成器 | 可以保留，Stage 6 使用 |
| ES/Utils/waypoint_generator/CATKIN_IGNORE.disabled | 启用标记 | 建议删除 |
| ES/Utils/waypoint_generator/src/waypoint_generator.cpp | RViz goal 转 EGO waypoint | 需要重构为隔离补丁或适配节点 |
| ES/local_sensing/CATKIN_IGNORE（删除） | 启用 EGO 原生局部感知 | 暂时无法确定 |
| ES/local_sensing/CATKIN_IGNORE.disabled | 启用标记 | 建议删除 |
| ES/mockamap/CATKIN_IGNORE（删除） | 启用随机地图 | 暂时无法确定 |
| ES/mockamap/CATKIN_IGNORE.disabled | 启用标记 | 建议删除 |
| ES/so3_control/CATKIN_IGNORE（删除） | 启用原生 SO3 控制器 | 暂时无法确定 |
| ES/so3_control/CATKIN_IGNORE.disabled | 启用标记 | 建议删除 |
| ES/so3_quadrotor_simulator/CATKIN_IGNORE（删除） | 启用原生动力学模拟器 | 暂时无法确定 |
| ES/so3_quadrotor_simulator/CATKIN_IGNORE.disabled | 启用标记 | 建议删除 |

上述 8 个暂时无法确定项，取决于项目是否正式支持 EGO 原生 mock/SO3 仿真。如果正式范围只保留 PX4、Gazebo、FAST-LIO 链路，应恢复这些包的 CATKIN_IGNORE。

### 7.5 日志、脚本、仿真和文档：15 个

| 文件 | 作用 | 分类 |
|---|---|---|
| AstraDrone_ros1_ws/src/SLAM/FAST_LIO/Log/mat_pre.txt | FAST-LIO 运行矩阵日志 | 建议删除：撤销改动并禁止运行日志入库 |
| ego-planner学习.md | 原始学习笔记 | 建议删除或移出工程仓库 |
| order.md | 常用启动和 rosbag 命令 | 需要重构：硬编码路径且内容过长 |
| ros.md | ROS 入门教程 | 需要重构并迁移至教程目录 |
| scripts/run_sh/pc_example.sh | 原有 PX4、FAST-LIO、offboard tmux 启动 | 需要重构：默认控制冲突和硬编码 |
| scripts/run_sh/stage6_planner.sh | Stage 6 环境检查和 tmux 编排 | 需要重构：平台绑定、无超时、进程识别不全 |
| simulation/sim_workspace/src/dynamic_obstacle_controller/launch/astra_dynamic_avoidance_static.launch | 修复动态障碍 world 路径 | 可以保留 |
| stage0.md | ROS/PX4 基础学习阶段 | 需要重构或迁移归档 |
| stage1.md | Offboard 基础学习阶段 | 需要重构：含旧控制逻辑 |
| stage2.md | 旧自动降落学习阶段 | 需要重构，优先级最高：含危险的 21196 指令 |
| stage3.md | 航点任务说明 | 需要重构：降落描述与源码不符 |
| stage4.md | 轨迹飞行和分析说明 | 需要重构 |
| stage6.md | EGO/PX4/FAST-LIO 集成说明 | 需要重构并成为唯一权威架构文档 |
| studymap.md | 总学习路线 | 需要重构：与当前 Stage 6 架构直接矛盾 |
| studynote.md | 综合学习笔记 | 需要重构或合并归档 |

分类汇总：

- 可以保留：17 个
- 需要重构：34 个
- 建议删除：17 个
- 暂时无法确定：8 个

## 8. 本次验证结果

通过的只读检查：

- offboard、bridge 和本分支修改的 EGO C++ 源码通过 g++ 语法检查。
- Python analyzer 通过 AST 解析。
- 9 个 XML/launch 文件解析通过。
- 2 个 YAML 配置文件解析通过。
- Bash 脚本通过 bash -n。
- 相关脚本通过 ShellCheck。
- roslaunch --files、--nodes、--dump-params 解析成功。
- Stage 6 解析出的主要节点：
  - /verified_map_to_planning_frame
  - /ego_planner_node
  - /traj_server
  - /waypoint_generator
  - /ego_mavros_bridge
- 现有 bridge 测试二进制：6/6 通过。
- 现有 offboard 测试二进制：7/7 通过。

验证限制：

- 现有测试二进制时间戳早于审计源码 checkout，只能作为辅助证据，不能证明当前源码完整重编译通过。
- 为遵守不写入文件的审计要求，没有执行会生成 build、devel、日志或缓存的完整 catkin_make。
- 没有进行本轮实时 Gazebo/PX4 飞行测试。
- 没有状态机级单测、rostest、服务失败测试、TF 错误测试或传感器断流测试。

## 9. 分批整理和验证顺序

### 第一批：安全和可复现性阻断项

修改内容：

- 确定受支持的 PX4 软降落参数。
- 禁止任务节点静默修改不受支持的 PX4 参数。
- 锁定 PX4、Gazebo 和 airframe 版本。
- 将坐标对齐、TF、home 和控制权检查移到解锁之前。
- 让 dry-run 执行完整 preflight，但不发布 setpoint、不调用解锁服务。
- 检查全部 MAVROS 控制 Topic。
- 增加 FCU、起飞、降落和服务重试的总超时及终止状态。

验证：

- 编译 offboard 和 ego_gazebo_bridge。
- 使用 fake MAVROS 或 rostest 验证错误 TF、过期输入、参数拒绝和其他控制发布者均阻止解锁。
- 检查 dry-run 下 bridge 在所有 MAVROS setpoint Topic 上均无输出。
- PX4 SITL 执行起飞、悬停、正常降落和服务拒绝测试。

### 第二批：拆分两个大型状态机

建议拆分为：

- 参数加载和统一校验；
- ROS 输入快照；
- freshness/watchdog；
- 飞行状态机；
- MAVROS 服务客户端；
- setpoint 生成；
- 降落策略；
- 控制权监视；
- 任务轨迹或路径跟随。

每次修改后的编译测试：

    cd /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws
    catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_ENABLE_TESTING=ON
    catkin_make run_tests_offboard run_tests_ego_gazebo_bridge
    catkin_test_results

补充状态转换表驱动测试，覆盖每个状态、超时和服务失败分支。

### 第三批：收敛 EGO vendor 修改和包范围

修改内容：

- 固定 EGO 上游版本。
- 只保留 frame 参数化和非法 dt 等必要补丁。
- 恢复 REFERENCE_PATH 兼容性，或明确删除并提供迁移说明。
- 删除全部 CATKIN_IGNORE.disabled。
- 明确是否正式支持原生 mock/SO3 仿真。
- 最终 PX4 链路只启用必需包。

验证：

- 独立构建 EGO 必需包。
- 若保留原生仿真，单独构建并运行原生仿真 smoke test。
- 对比补丁应用前后的标准 EGO 示例。
- 运行 goal、重规划、轨迹超时和 frame 错误测试。

### 第四批：重新确认控制和坐标语义

修改内容：

- 明确 PositionCommand 的执行策略。
- 建立 camera_init、map、body、base_link 的正式 TF 契约。
- 验证传感器安装外参。
- 调整 planning horizon、局部更新范围和未知空间策略。
- 对 goal z、返航高度和降落地面高度给出统一语义。

运行验证：

- 空场：起飞、两个目标、返航、降落。
- 静态障碍：重规划、最小净空和轨迹误差。
- 故障注入：odom、cloud、PositionCommand、FCU、TF 和 OFFBOARD 分别断流。
- 同时启动冲突控制节点，确认解锁前阻断。
- rosbag 统计指令/实际轨迹误差、速度、加速度、最小障碍距离和故障响应时间。

### 第五批：脚本、日志和文档

修改内容：

- 移除运行日志和 disabled 标记。
- 增加 FAST-LIO 日志忽略规则。
- 合并重复教程。
- 删除所有 21196 当前操作指导。
- 以 stage6.md 或正式架构文档作为唯一集成事实来源。
- 为脚本增加 headless、超时、版本检测和明确的控制模式。
- 禁止 pc_example.sh 默认启动控制器。

验证：

- ShellCheck 和 bash -n。
- 所有文档命令在干净环境逐条执行。
- roslaunch --dump-params 与文档参数表比对。
- 检查仓库运行一次后没有产生新的 git 改动。

### 第六批：正式基线验收

验收条件：

- 从干净 clone 可完成构建。
- 外部依赖版本全部锁定。
- install-space 启动可用。
- CI 包含编译、单测、launch/XML/YAML 和静态检查。
- 完成多轮确定性 SITL 测试及长时间稳定性测试。
- 所有失败场景都有明确、有限时间内的安全终态。
- 工作区运行后保持干净。
- 文档与实际 Topic、参数、frame 和启动关系一致。

## 10. 最终判断

当前 ego-project 审计基线已经完成有价值的原型集成，但在安全门禁、控制语义、坐标契约、vendor 管理、可复现性和文档一致性方面仍存在正式工程阻断项。

建议严格按照本报告的分批顺序整理，不应直接将审计基线 HEAD 作为生产或真机基线。
