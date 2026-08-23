# 1. AstraDrone 绕塔项目的三套 RViz 配置

本项目中与当前绕塔任务直接对应的 RViz 配置有三套。它们分别服务于：单机固定航线展示、单机 EGO 低空避障展示，以及三机绕塔的全局态势展示。三者不能混用，因为订阅的 Topic、命名空间和固定坐标系不同。

## 1. 单机固定绕塔：`fixed_orbit_inspection.rviz`

配置文件：

```text
AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/rviz/fixed_orbit_inspection.rviz
```

启动入口：`astra_tower_mission/launch/fixed_orbit_inspection.launch`。该 launch 的 `rviz` 参数默认是 `true`，因此运行固定绕塔检查时会默认启动此 RViz。

它用于阶段 1 的单机、固定高度、预设闭合航线绕塔任务。这里的航线由任务节点直接生成和执行，并不通过 EGO-Planner 做局部避障或在线重规划。

固定坐标系是 `map`，主要显示：

| 显示内容 | Topic | 用途 |
|---|---|---|
| 地图网格 | 无 Topic | 观察任务的平面位置关系。 |
| 闭合航线预览 | `/tower_mission/route_preview` | 绿色路线，表示任务计划的绕塔闭环。 |
| 航点与朝向 | `/tower_mission/waypoint_poses` | 橙色箭头，表示每个航点位置和面向塔的 yaw。 |
| 塔与巡检安全包络 | `/tower_mission/route_markers` | 显示铁塔碰撞包络及巡检环。 |
| 实际飞行轨迹 | `/tower_mission/actual_path` | 蓝色轨迹，用于与预设航线比较。 |
| 当前目标 | `/tower_mission/current_target` | 当前正在跟踪的目标位姿。 |
| 飞机实际位姿 | `/mavros/local_position/pose` | MAVROS 提供的 PX4/Gazebo 实际位姿。 |

判断任务时，要同时看绿色的计划闭环和蓝色的实际轨迹；计划路线正确不代表飞机一定已经正确跟踪。

## 2. 单机低空避障：`ego_gazebo_bridge.rviz`

配置文件：

```text
AstraDrone_ros1_ws/src/MissionControl/ego_gazebo_bridge/rviz/ego_gazebo_bridge.rviz
```

启动入口是 `ego_gazebo_bridge/launch/ego_gazebo_bridge.launch`。该 launch 的 `rviz` 参数默认是 `false`；单机低空避障的 `low_altitude_inspection.launch` 将该参数透传给 bridge，因此只有显式传入 `rviz:=true` 时才会启动。

这套配置用于单机的 FAST-LIO + EGO-Planner 局部规划链，重点是检查感知、占据地图与规划轨迹是否一致。固定坐标系是 `camera_init`，即当前 EGO 规划坐标系。

| 显示内容 | Topic | 用途 |
|---|---|---|
| 地图网格 | 无 Topic | 提供局部三维观察参考。 |
| FAST-LIO 注册点云 | `/cloud_registered` | 观察传感器建图输入和环境障碍物。 |
| FAST-LIO 里程计 | `/Odometry` | 显示飞机的定位结果和短历史。 |
| 膨胀占据地图 | `/grid_map/occupancy_inflate` | 以体素盒显示规划器判定不可通行的安全障碍区。 |
| EGO 目标点 | `/ego_planner_node/goal_point` | 当前局部规划目标。 |
| 全局参考 | `/ego_planner_node/global_list` | 到达目标的全局参考路线。 |
| 优化局部轨迹 | `/ego_planner_node/optimal_list` | EGO 实际优化输出、用于执行的局部绕障轨迹。 |
| TF 坐标树 | `/tf`、`/tf_static` | 检查 `camera_init`、机体和其他坐标系的连通性。 |

配置中还保留了 `/ego_planner_node/a_star_list` 的 A* 搜索路径显示，但默认关闭。低空避障排查时应优先核对点云、膨胀地图、目标和优化轨迹：若它们不在同一空间关系中，先检查 frame/TF，而不是直接判断规划失败。

## 3. 三机绕塔：`triple_tower.rviz`

配置文件：

```text
AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/config/triple_tower.rviz
```

启动入口：`astra_swarm_bringup/launch/triple_tower_inspection.launch`。其中 `start_rviz` 默认是 `true`，启动的节点名为 `astra_swarm_three_uav_rviz`，命令指定该配置并以 `world` 为固定坐标系。

这套配置是三机项目的全局态势面板。它不会启动每架机内部的 bridge RViz；每架机的 `uav_tower_stack.launch` 明确传入 `rviz:=false`，避免同一项目弹出三套重复的单机规划窗口。

固定坐标系是 `world`，主要显示：

| 显示内容 | Topic 或来源 | 用途 |
|---|---|---|
| 世界网格 | 无 Topic | 以全局坐标观察三机与铁塔的相对位置。 |
| 三架飞机模型 | `uav1`、`uav2`、`uav3` 的 robot description/TF | 显示三架机当前姿态。 |
| 三份独立点云 | `/uav1/cloud_registered_peer_filtered`、`/uav2/cloud_registered_peer_filtered`、`/uav3/cloud_registered_peer_filtered` | 比较每架机的局部感知结果。 |
| 三条 EGO 优化轨迹 | `/uav1/drone_0_ego_planner_node/optimal_list`、`/uav2/drone_1_ego_planner_node/optimal_list`、`/uav3/drone_2_ego_planner_node/optimal_list` | 观察每架机当前的局部规划。 |
| 三条实际航迹 | `/uav1/swarm/actual_path`、`/uav2/swarm/actual_path`、`/uav3/swarm/actual_path` | 对比三机实际执行情况。 |
| 三个当前任务目标 | `/uav1/tower_mission/current_target`、`/uav2/tower_mission/current_target`、`/uav3/tower_mission/current_target` | 显示各机此刻要去的位置与朝向。 |
| 任务航点 | `/swarm/rviz/mission_markers` | 显示绕塔任务的 WP1--WP8 等任务标记。 |
| 集群状态诊断 | `/swarm/rviz/status_markers` | 显示集群任务状态与协调诊断标记。 |
| TF 坐标树 | `/tf`、`/tf_static` | 检查 world 与三架无人机的坐标树是否连通。 |

## 4. 单机低空避障与三机绕塔 RViz 的直接区别

两套配置的区别主要是观察目的不同，不表示底层规划或感知链路不同。单机配置面向一架机的避障调试，三机配置面向公共世界坐标中的集群任务态势。

| 对比项 | 单机低空避障 `ego_gazebo_bridge.rviz` | 三机绕塔 `triple_tower.rviz` |
|---|---|---|
| Fixed Frame | `camera_init`，以单机规划局部坐标为中心。 | `world`，统一显示三架机和铁塔。 |
| 默认观察距离 | 约 `18 m`，适合近看障碍、地图和局部轨迹。 | 约 `58 m`，视点对准塔心，适合看三机全局关系。 |
| 无人机显示 | 不加载完整 RobotModel，主要通过里程计坐标轴观察本机。 | 加载 UAV1、UAV2、UAV3 三套 RobotModel。 |
| 环境点云 | 一路 `/cloud_registered`，按高度着色。 | 三路 `cloud_registered_peer_filtered`，分别使用橙、蓝、绿显示。 |
| 膨胀占据地图 | 默认显示 `/grid_map/occupancy_inflate`，能直接观察 EGO 判定的不可通行体素。 | 当前没有显示三架机的 `/uavN/stage3/occupancy_inflate`。 |
| EGO 规划 | 显示单机目标、全局参考、优化局部轨迹；A* 搜索显示存在但默认关闭。 | 显示三架机各自的目标点和优化局部轨迹，不显示每机全局参考或 A* 搜索。 |
| 实际飞行路径 | 没有三机任务专用的实际轨迹组。 | 显示三架机不同颜色的 `/uavN/swarm/actual_path`。 |
| 任务信息 | 重点是单机感知、地图和规划结果。 | 额外显示三机当前任务目标、WP1--WP8 和集群状态诊断。 |
| 交互工具 | 有 `2D Nav Goal` 和 `Publish Point`，可手动发送调试输入。 | 不提供手动目标工具，正式目标由三机任务管理器发布。 |
| TF 显示 | 默认显示 frame 名称，方便单机排查。 | 默认隐藏 frame 名称，避免三棵 TF 树遮挡画面。 |

三机 RViz 当前显示的是队友过滤后的点云 `/uavN/cloud_registered_peer_filtered`；EGO 实际使用的输入还会继续经过自身机体过滤，成为 `/uavN/cloud_registered_self_filtered`。因此三机窗口中的彩色点云不是规划器最终输入的完全等价可视化。单机窗口显示的 `/cloud_registered` 同样位于低空地面过滤之前，但它同时显示规划器生成的膨胀占据地图，所以更适合解释“EGO 为什么把某处判断为障碍”。

### RobotModel 的内存和渲染开销

当前每架 RViz RobotModel 加载三个视觉 mesh：Iris 机体、MID360 和 D435。源码资产的复杂度约为：

| Mesh | 文件大小 | 三角面数量 |
|---|---:|---:|
| Iris | `3.9 MB` | `79,996` |
| MID360 | `31 MB` | `376,005` |
| D435 | `12 MB` | `231,186` |
| 每架合计 | 约 `47 MB` 源文件 | 约 `687,187` |

三架机需要绘制约 `206 万`个三角面实例，因此在集成显卡、软件渲染或 Gazebo GUI 与 RViz 共用同一块负载较高的 GPU 时可能影响帧率。不过 RobotModel 是静态网格：mesh 加载后主要更新 TF 和绘制，不会像点云那样持续接收大消息并反复重建几何；相同 URI 的 mesh 资源通常还能由渲染资源管理器缓存。因此不能仅根据源文件大小推断成三倍常驻内存，也不能在没有实测前把 RobotModel 判定为当前主要内存瓶颈。

当前建议保留三机模型，因为完整模型能清楚表示机头方向、姿态和传感器安装关系。若实测发现关闭 `Three UAV Models` 后 RViz 帧率明显改善，再改为轻量 Marker：每架机使用不同颜色的球体或立方体表示位置，并增加一个箭头表示 yaw。只用球体无法看出航向；只用对称正方体也不容易辨认机头方向。

### 三路膨胀占据地图的性能影响

膨胀占据地图比 RobotModel 更可能造成持续卡顿。现有三机飞行 bag 的序列化载荷统计中，三路 `/uavN/stage3/occupancy_inflate` 合计约 `8.3 MB/s`，约占全部未压缩 bag 载荷的 `55.4%`。每路约 `9 Hz` 更新，而且 RViz 若采用单机配置中的 `Boxes` 样式，需要持续把大量体素转换并绘制为小立方体。

因此不建议把三路膨胀地图全部设为默认开启。推荐做法是：

1. 在三机 RViz 中预先加入 UAV1、UAV2、UAV3 三个独立的膨胀地图 Display，但默认全部关闭。关闭的 Display 不订阅 Topic，基本不会增加持续数据和渲染负担。
2. 排查某架机为什么绕障或规划失败时，只打开该架机的膨胀地图，同时可临时关闭另外两架机的点云。
3. 只有检查三机地图一致性时才短时间同时打开三路，并观察 RViz 帧率、`rviz` 进程内存和 GPU 使用率。
4. 若需要长期同时观察，优先降低可视化发布频率或显示稀疏诊断快照，不改变 EGO 内部地图分辨率和安全膨胀参数。

性能排查顺序应是：先关闭三路膨胀地图，再关闭不需要观察的点云，最后再测试关闭 RobotModel。动态点云和体素通常比三个静态机体模型更值得优先优化。

## 使用关系速记

```text
单机固定绕塔、无 EGO 避障
  -> fixed_orbit_inspection.rviz
  -> 固定坐标系：map

单机低空避障、FAST-LIO + EGO
  -> ego_gazebo_bridge.rviz
  -> 固定坐标系：camera_init

三机低空绕塔、集群协调
  -> triple_tower.rviz
  -> 固定坐标系：world
```

不要将三机 Topic 直接填入单机 EGO RViz，也不要把单机 `/cloud_registered` 误认为三机的每机点云。三机任务使用带 `/uav1`、`/uav2`、`/uav3` 前缀的独立命名空间；单机任务使用无前缀的全局 Topic。

# 2. ROS 数据流与 Topic：三机如何真正飞起来

本节学习的是：一个“绕塔目标”怎样经过 ROS 的消息通道，最终成为 PX4 的飞行指令。这里的 Topic 可以理解为持续传送某一类数据的“管道”：发布者写入，订阅者读取；它不是函数调用，也不会因为某个节点发布一次就自动让所有后续动作完成。

三机项目中，每架机有自己的命名空间，例如 UAV1 的绝大多数 Topic 都以 `/uav1/` 开头。下文用 `N` 表示 `1`、`2` 或 `3`，例如 `/uavN/Odometry` 指 `/uav1/Odometry`、`/uav2/Odometry` 或 `/uav3/Odometry`。

## 一条完整的数据闭环

```text
Gazebo 传感器
  ├─ /uavN/livox/lidar、/uavN/livox/imu
  │      ↓
  ├─ FAST-LIO + frame_adapter
  │      ├─ /uavN/Odometry                 本机位姿
  │      ├─ /uavN/cloud_registered         原始注册点云
  │      └─ /tf、/tf_static                坐标关系
  │      ↓
  ├─ 队友点云过滤 + 自身机体过滤 + 地面过滤
  │      └─ /uavN/stage3/cloud_registered_filtered
  │      ↓
  ├─ EGO-Swarm：点云建图、避障、协同重规划
  │      └─ /uavN/planning/bspline         带时间的 B 样条轨迹
  │      ↓
  ├─ traj_server
  │      └─ /uavN/planning/pos_cmd         PositionCommand（位置、速度、加速度、yaw）
  │      ↓
  ├─ /uavN/ego_mavros_bridge
  │      └─ /uavN/mavros/setpoint_raw/local  PositionTarget
  │      ↓
  ├─ MAVROS → MAVLink → PX4 OFFBOARD → Gazebo 飞机运动
  │      ├─ 运动后的传感器数据重新进入 FAST-LIO，产生 /uavN/Odometry
  │      └─ /uavN/mavros/local_position/pose 反馈给 bridge 执行检查
  └─ 两类反馈分别重新进入任务、规划和 bridge，构成闭环
```

上图中箭头不表示只有一条串行流水线：任务状态机、集群协调器和安全监督也会持续读取位姿、状态和轨迹，并在合适的时刻更新目标或许可。

## 目标从哪里来

`/uavN/tower_mission` 是本机任务状态机。它根据当前任务阶段和集群许可选择下一个巡检点，并发布：

| Topic | 消息类型 | 含义 |
|---|---|---|
| `/uavN/planning/goal` | `geometry_msgs/PoseStamped` | 交给规划器的当前目标位姿。它回答“要到哪里”。 |
| `/uavN/tower_mission/current_target` | `geometry_msgs/PoseStamped` | 当前任务目标的可视化/状态输出，供 RViz、集群状态和记录器观察。 |
| `/uavN/tower_mission/state` | `std_msgs/String` | 本机任务阶段，例如等待许可、进场、绕塔或退出。 |
| `/uavN/tower_mission/selected_tower_center` | `geometry_msgs/PointStamped` | 塔心位置，bridge 用它在绕塔时计算“朝向塔”的 yaw。 |

`astra_swarm_manager` 不替代这个目标 Topic 去逐点指挥飞机。它通过 `/uavN/swarm/...permission` 一类 Topic 发放阶段许可；`tower_mission` 在获准后才发布或推进下一目标。因此要分清：**许可决定“现在可不可以做”，目标决定“接下来去哪”。**

## 感知、规划与轨迹的三种不同数据

| 数据 | 典型 Topic | 谁产生 | 谁使用 | 不要混淆为 |
|---|---|---|---|---|
| 位姿/里程计 | `/uavN/Odometry` | FAST-LIO 经 frame adapter 整理 | 任务、EGO-Swarm、bridge | 规划轨迹。它只是“飞机现在在哪、朝哪”。 |
| 点云 | `/uavN/cloud_registered`、`/uavN/stage3/cloud_registered_filtered` | FAST-LIO 和过滤节点 | EGO-Swarm 占据地图 | 飞机的飞行路径。它描述观测到的环境点。 |
| 膨胀占据地图 | `/uavN/drone_*_ego_planner_node/grid_map/occupancy_inflate` | EGO-Swarm | 任务节点和 RViz | 原始点云。它是为安全距离膨胀后的不可通行空间。 |
| B 样条 | `/uavN/planning/bspline` | EGO-Swarm | `traj_server` | 单个目标点。它是带时间参数的连续规划轨迹。 |
| 连续控制参考 | `/uavN/planning/pos_cmd` | `traj_server` | `ego_mavros_bridge` | PX4 原生指令。它仍是 ROS 侧的 `PositionCommand`。 |
| MAVROS setpoint | `/uavN/mavros/setpoint_raw/local` | `ego_mavros_bridge` | MAVROS/PX4 | 实际位姿反馈。它是唯一的持续控制出口。 |

## `traj_server` 与 bridge 为什么要分开

EGO-Swarm 输出的是整段 B 样条，`traj_server` 按时间把它展开成高频的 `PositionCommand`。该消息同时带位置、速度、加速度、yaw 和 yaw rate，因而比“下一个航点”更接近可执行轨迹。

`ego_mavros_bridge` 仍不能被省掉：它检查输入是否新鲜、frame 是否一致、是否发生多个控制节点抢占 Topic，以及集群起飞许可是否满足；只有控制启用且检查通过时，才把命令转换成 `mavros_msgs/PositionTarget` 发布到 `/uavN/mavros/setpoint_raw/local`。在 dry-run 中，它可以观察整条链路，但不会创建或发布该控制 Topic。

## 看 Topic 时的正确顺序

排障时按“先有输入，再有规划，最后有控制”检查，而不是先盯着 RViz：

```text
1. /uavN/Odometry 是否持续且 frame_id 正确？
2. /uavN/stage3/cloud_registered_filtered 是否持续、是否在 planning frame？
3. /uavN/planning/goal 是否随任务阶段更新？
4. /uavN/planning/bspline 和 /uavN/planning/pos_cmd 是否出现且时戳新鲜？
5. bridge state 是否允许跟踪，且 /uavN/mavros/setpoint_raw/local 是否只有 bridge 一个发布者？
6. /uavN/mavros/state 是否已连接、OFFBOARD、已解锁？
```

常用的只读观察命令如下；将 `uav1` 替换为要检查的无人机即可：

```bash
rostopic info /uav1/planning/goal
rostopic echo -n 1 /uav1/Odometry
rostopic echo -n 1 /uav1/planning/pos_cmd
rostopic echo -n 1 /uav1/ego_mavros_bridge/state
rostopic info /uav1/mavros/setpoint_raw/local
```

`rostopic info` 特别适合查“谁在发布、谁在订阅”；正常控制时，`/uavN/mavros/setpoint_raw/local` 的发布者应只有本机的 `ego_mavros_bridge`。`rostopic echo -n 1` 只读取一条消息，适合先确认 Topic 是否存在、消息是否新鲜、坐标 frame 是否符合预期。

# 3. 单机任务状态机：一架机怎样完成自己的绕塔任务

当前三机项目不是只有一个总状态机。每架机各运行一个 `sector_inspection_mission_node`，节点名都叫 `tower_mission`，但命名空间不同：`/uav1/tower_mission`、`/uav2/tower_mission`、`/uav3/tower_mission`。它们运行同一套状态机逻辑，却各自维护本机目标、完成进度、失败次数和恢复过程。

这个节点的职责只有一句话：**在正确的时刻选出本机下一个正式目标，并决定等待、重试、恢复、返航或结束。** 它不自己计算绕障曲线，不直接发布 MAVROS 控制 setpoint，也不替集群协调器决定三机的先后顺序。

## 状态机与规划器、bridge 的边界

```text
tower_mission
  - 决定：下一正式目标是什么；何时等待许可；该重试、恢复还是结束
  - 发布：/uavN/planning/goal
                 ↓
EGO-Swarm + traj_server
  - 决定：在点云障碍和队友轨迹下，怎样生成连续可行轨迹
  - 发布：/uavN/planning/pos_cmd
                 ↓
ego_mavros_bridge
  - 决定：轨迹能否被安全交给飞控；进入 HOLD、返航或降落的执行动作
  - 发布：/uavN/mavros/setpoint_raw/local
```

所以状态机中的“到达目标”不是它自己控制飞机到达，而是它根据里程计、规划器状态、bridge 状态和到达阈值判断：上一次发出的正式目标是否已被下层成功执行。

## 主流程：从等待输入到完成

源码中有二十多个精细状态；学习时先将它们按任务意图归为六段。箭头表示正常流程，方括号表示需要集群许可。

```text
1. 准备
WAIT_INPUTS
  -> STAGING_POINT / SEGMENTED_CLIMB

2. 进场
  -> [WAIT_ENTRY_PERMISSION]
  -> ENTRY_GATE_TRANSIT
  -> [ENTRY_READY：等待 MOVE_TO_ORBIT_STAGING]
  -> [ORBIT_STAGING_READY：等待 ORBIT_RELEASE]

3. 绕塔巡检
  -> EVALUATING -> TARGET_LOCKED -> NAVIGATING
  -> （下一扇区）EVALUATING -> ...

4. 可选换层
  -> [WAIT_TRANSITION_PERMISSION] -> LAYER_TRANSITION -> EVALUATING

5. 退出与返航
  -> [WAIT_EXIT_PERMISSION] -> GO_TO_EXIT_GATE
  -> NORMAL_RETURN 或 RETURN_EGRESS -> RETURN_HOME -> DONE

6. 任何阶段的异常分支
  -> HOLDING -> 重试 / 重新选点 / RECOVERING / 返航 / FAILURE_LANDING / ERROR
```

当前 `triple_tower_inspection.launch` 默认是低空模式（`low_altitude_enabled=true`）、单层（每机 `layer_offsets=[0.0]`）、巡检高度 `3.0 m`。因此“换层”状态是通用能力，当前默认任务通常不会实际进入；而 `STAGING_POINT`/通用分段爬升也会被 bridge 已完成的低空稳定悬停路径部分绕过。它们仍保留在状态机中，以支持非低空或多层配置。

## 每一段具体在做什么

| 任务段 | 关键状态 | 进入条件与动作 | 正常离开条件 |
|---|---|---|---|
| 输入准备 | `WAIT_INPUTS` | 等待新鲜里程计、点云/地图、bridge 状态；控制模式还要等待 bridge `HOVER_READY` 与任务开始许可。随后记录 home，并构造本层扇区与进场候选。 | 输入健康且允许开始。 |
| 起飞后集结/爬升 | `STAGING_POINT`、`SEGMENTED_CLIMB` | 高度较高的通用模式先到安全集结点，再按分段目标爬升并等待点云覆盖稳定。低空模式中，bridge 已稳定在入口高度，任务节点直接使用当前悬停位姿建立进场。 | 入口走廊候选可用，且地图新鲜。 |
| 等待进场 | `WAIT_ENTRY_PERMISSION` | 发布可选进场走廊，等待协调器从三机候选中选择一个并发放独占进入许可。 | 已锁定走廊且许可新鲜为真。 |
| 入口通行 | `ENTRY_GATE_TRANSIT` | 向 EGO-Swarm 发布锁定的入口门目标；规划器负责从当前位置绕障到门。 | 到达入口门，且本机规划/bridge/地图仍健康。 |
| 等待绕塔放行 | `ENTRY_READY`、`ORBIT_STAGING_READY` | 到入口后先等待进入绕塔预备位置的许可，再等待正式开始绕塔的许可。等待时请求停止轨迹跟踪/保持，避免未经协调继续推进。 | 收到对应许可，并能锁定第一个方向正确的扇区目标。 |
| 选择与锁定扇区 | `EVALUATING`、`TARGET_LOCKED` | 在当前扇区的正式航点和同扇区备选点中，依据新鲜占据地图评估安全性；一旦锁定，正式目标坐标不再被任务层悄悄改写。 | 已得到一个安全、可规划的正式扇区目标。 |
| 执行绕塔 | `NAVIGATING` | 发布 `/uavN/planning/goal`，等待 EGO-Swarm 和 bridge 执行。 | 到达并满足到达保持条件，扇区标为已覆盖；然后评估下一个扇区。 |
| 换层 | `WAIT_TRANSITION_PERMISSION`、`LAYER_TRANSITION` | 一层完成后等待集群许可，再沿受检查的固定垂直转换目标到下一层。 | 新层激活后重新开始扇区评估。 |
| 退出与返航 | `WAIT_EXIT_PERMISSION`、`GO_TO_EXIT_GATE`、`NORMAL_RETURN`、`RETURN_EGRESS`、`RETURN_HOME` | 先等待退出走廊许可；正常情况下从出口门沿实际入场路径反向返回，再由 bridge 完成 home hover 和降落。若正常返程不可用，才使用保守的安全返航走廊。 | bridge 报告完成，且任务节点验证已落地、已上锁并在 home 附近，才进入 `DONE`。 |

## 绕塔时一个目标怎样被处理

以“前往当前扇区的一个标准航点”为例：

```text
EVALUATING
  读取最新占据地图，比较正式航点和同扇区备选点
      ↓
TARGET_LOCKED
  锁定一个正式目标；不把局部 A* 中间点误当作新的任务航点
      ↓
NAVIGATING
  发布 /uavN/planning/goal
      ↓
EGO-Swarm 自己生成/重规划 B 样条
      ↓
任务节点读取 odom、planner/status、bridge/state
  ├─ 到达并满足保持条件：该扇区 COVERED，评估下一扇区
  ├─ 规划器暂时不可达、无进展或目标超时：进入 HOLDING
  └─ 地图/协调许可失效或 bridge HOLD：立即进入 HOLDING
```

这里最容易误解的一点是：**EGO-Swarm 可以为了避障在局部轨迹上绕开障碍，但任务状态机仍然记住“我正在完成哪个正式扇区目标”。** 轨迹绕路不等于任务航点被改成了绕路中的某个中间点。

## HOLD 不是结束，而是受控决策点

`HOLDING` 是异常处理的中心状态，而不是简单报错。进入 HOLD 后，状态机会先停止继续推进，并根据失败类型选择有限、可解释的后续动作：

| 失败类型 | HOLD 后的典型处理 |
|---|---|
| 集群许可失效、阶段安全禁止 | 保持等待；许可恢复后要求 EGO 重新生成新轨迹，不恢复旧 B 样条。 |
| 同一安全目标出现短暂规划失败、无进展或超时 | 对同一锁定目标进行有限次数重试。 |
| 重试后仍确认不可达 | `RELOCATING`，只在当前任务允许的候选范围内重选目标。 |
| 需要姿态/高度恢复 | `RECOVERING`，执行受限的 R1/R2/重新进场步骤，然后回到重选目标。 |
| 进场、退出、返程恢复耗尽 | 请求受控返航；安全返程也失败时才进入 `FAILURE_LANDING` 或 `ERROR`。 |

这解释了为什么状态机没有采用“规划失败就随便换下一个航点”的策略：它会先保持、有限重试、记录原因，再在明确范围内恢复或返航，避免把一次短暂感知/规划抖动误判成任务完成。

## 状态机依赖什么、发布什么

| 类别 | 主要输入/输出 | 目的 |
|---|---|---|
| 位姿与感知输入 | `/uavN/Odometry`、过滤后的点云、膨胀占据地图 | 判断当前位置、目标/走廊是否安全、地图是否新鲜。 |
| 规划执行反馈 | `/uavN/planning/pos_cmd`、`/uavN/planner/status`、`/uavN/ego_mavros_bridge/state` | 判断规划是否生成、是否失败、bridge 是否 HOLD 或已返航。 |
| 飞控反馈 | `/uavN/mavros/state`、`/uavN/mavros/extended_state` | 确认连接、解锁与最终接地。 |
| 集群许可输入 | `/uavN/swarm/...permission` | 控制开始、进场、绕塔、换层、退出与降落的先后。 |
| 核心输出 | `/uavN/planning/goal`、`/uavN/tower_mission/state`、`/uavN/tower_mission/current_target` | 分别是给规划器的目标、给协调/记录器的状态、给显示/记录的当前任务目标。 |

学习和排查时，先看 `/uavN/tower_mission/state` 知道任务“正在等什么”，再看 `/uavN/tower_mission/current_target` 知道“要去哪”，最后看 `/uavN/planning/pos_cmd` 与 bridge 状态确认“下层是否真的在执行”。不要只看到 RViz 中有轨迹，就认为任务状态机已经允许进入下一扇区。

# 4. 航点选择与评分：当前扇区该去哪个点

本节讲的是任务层怎样选择“正式航点”。它与 EGO-Swarm 的局部绕障不同：任务层决定当前扇区的正式目标坐标，EGO-Swarm 再决定绕开障碍和队友后怎样到达该目标。

## 先记住结论

当前三机低空绕塔不是把所有候选点混在一起，谁总分高就去谁；它是“**先按半径分层，再在当前层内选择**”。

```text
当前应执行的扇区
  → 生成该扇区候选点
  → 淘汰不安全/不可用点
  → 先看 12.5 m 半径层
  → 12.5 m 全部淘汰，才看 14.5 m
  → 14.5 m 也全部淘汰，才看 16.5 m
  → 在第一个仍有安全候选的半径层内作最终选择
```

最终选择也有固定优先顺序：

```text
仍安全的旧锁定点
  → 当前半径层的中心角名义点
    → 其余安全候选的评分最高者
```

因此，`12.5 m + 扇区中心角 + 3 m` 的名义点只要满足硬约束，就直接锁定；不会因为外圈候选评分更高而改去外圈。

## 候选点从哪里来

三机低空任务固定 8 个扇区，逆时针执行。每个扇区围绕它自己的中心角，生成以下候选网格：

```text
半径：12.5 m、14.5 m、16.5 m
角度偏移：0°、-5°、+5°、-10°、+10°、-12°、+12°
高度：3.0 m（低空任务固定，不允许候选降到 2 m、1 m 或 0 m）
```

候选的角度始终不能离开当前扇区。若当前要完成第 3 扇区，任务只能在第 3 扇区的上述网格中找替代点；不会跳到第 4 或第 2 扇区来换取更高分数。

## 先淘汰：哪些点根本不能评分

以下是硬约束。任一项失败，候选会被标记为拒绝，不参与评分：

- 地图不新鲜，或目标高度/扇区边界不合法；
- 违反塔体 keep-out；
- 候选端点落在 EGO 膨胀占据地图中，或端点净空不够；
- 与已知粗略静态障碍的端点净空不够；
- 已经被有限次数 EGO 尝试证实为 `PLANNER_UNREACHABLE`；
- 会使当前逆时针绕塔进度倒退。

### 当前净空到底是多少

“净空”是候选点到障碍的三维距离，并不是一个统一的单一半径。当前三机低空配置中：

| 对象 | 任务层要求 | 如何理解 |
|---|---:|---|
| EGO 已膨胀占据地图 | 至少 `0.5 m` | 候选点到任何膨胀占据点的距离必须不少于 `0.5 m`。 |
| EGO 地图膨胀本身 | `0.4 m` | 已包含在占据地图中；粗略理解，真实障碍外的总操作余量约为 `0.4 + 0.5 = 0.9 m`，但体素化使它不是精确几何保证。 |
| 塔体 | 离塔心至少 `8.41 m` | 塔体碰撞半径 `6.41 m` 加任务 keep-out `2.0 m`。 |
| 已知粗略静态障碍 | 至少 `1.0 m` | 候选端点必须离其粗略包络至少 `1.0 m`。 |

因此，“被障碍物占据”指候选端点距离某个膨胀地图占据点小于 `0.5 m`。这种候选直接得到 `OCCUPANCY_OR_CLEARANCE`，不会被发给 EGO-Swarm。

## 什么时候真正打分

只有出现下面的情况才打分：当前最小可用半径层中，没有仍安全的旧锁定目标，也没有安全的中心角名义点。此时在该半径层的剩余安全候选间计算：

```text
分数 =
  + 0.2 × 净空
  - 6.0 × 半径偏差
  - 4.0 × 12.5 × 角度偏差（弧度）
  - 14.0 × 高度偏差
  - 0.1 × 到当前位置的三维距离
  - 0.2 × 到上一正式巡检目标的三维距离
  - 1.0 × 未知区域比例
  - 6.0 × 直线走廊被阻挡的风险
```

分数越大越好。当前低空任务高度固定，且评分只发生在同一半径层内，因此“高度偏差”和“半径偏差”通常均为零；实际最常比较的是角度偏离、净空、当前位置距离、连续性和走廊风险。

| 评分要素 | 它实际表示什么 | 对选择的影响 |
|---|---|---|
| 净空 | 候选点到最近塔体、膨胀占据点或硬静态障碍的距离。 | 越大越加分；但低于阈值时直接淘汰，而不是仅扣分。 |
| 半径偏差 | `|候选半径 - 12.5 m|`。 | 越靠近名义巡检圆越好；不过半径层本身先后已是硬顺序。 |
| 角度偏差 | 候选点偏离当前扇区中心角的绝对角度。 | 越接近标准扇区中心线越好；在 `12.5 m` 半径上偏 `5°` 约相当于沿圆周偏 `1.09 m`。 |
| 高度偏差 | 候选高度偏离当前巡检层高度的距离。 | 当前低空固定 `3 m`，通常为零；高空多层任务才会明显影响结果。 |
| 当前位置距离 | 当前 odom 位置到候选点的三维直线距离。 | 更近会少扣分，避免不必要的大跨度目标跳转。 |
| 连续性 | 候选点到**上一正式巡检目标**的三维直线距离。 | 更近会少扣分，避免正式航点突然跳得很远；它只是距离近似，不是曲率或真正轨迹平滑度。 |
| 未知区域比例 | 候选附近有多少区域未被当前传感器充分观测。 | 当前低空配置中只作为风险扣分，不会单独淘汰端点。 |
| 走廊风险 | 从当前位置到候选点画直线，并按 `0.4 m` 间隔检查中间是否穿过膨胀地图或静态障碍。 | 端点安全但直线路径不安全时，候选会被标记 `STRAIGHT_CORRIDOR_BLOCKED`。 |

## 走廊风险为何不总是直接淘汰

“候选端点安全”与“直线飞过去安全”是两个不同问题：

```text
当前位置 ────（直线中间有障碍）──── 候选端点
                                  ↑
                              端点本身仍安全
```

当前三机 launch 启用了 `prefer_clear_straight_corridor=true`：如果同一半径层存在端点安全且直线走廊也安全的候选，所有直线走廊被挡住的候选都会被排除；如果该层所有安全端点的直线走廊都被挡住，它们才会保留并各扣 `6` 分，由 EGO-Swarm 为锁定的正式目标生成局部绕行轨迹。已知静态障碍走廊若被配置为硬约束，则会直接淘汰；当前低空配置将其作为风险，而非一律硬淘汰。

## 已锁定点后来失败怎么办

评分只负责选出目标，不能保证 EGO 一定能到达。若锁定点端点安全，但 EGO 报告不可达、无进展或目标超时，状态机会：

```text
HOLD 2 秒
  → 对同一锁定目标最多重试 2 次
  → 仍不可达：标记该 candidate ID 为 PLANNER_UNREACHABLE
  → 回到当前扇区，按上述规则选下一个候选
  → 当前扇区全部候选耗尽：不跳扇区、不标记完成，转入受控恢复或返航/降落
```

这就是“任务层选正式点，EGO-Swarm 选绕行轨迹”的边界：任务层不会因为 EGO 的局部绕障而忘记当前正在完成哪个扇区，也不会把局部 A* 中间点偷换成新的正式巡检航点。

# 5. EGO-Swarm 规划逻辑：怎样同时避开障碍和队友

当前三机项目使用的是 EGO-Swarm 规划核心，而仓库仍沿用 `ego_planner` 包名、`ego_planner_node` 可执行文件和 `EGOPlannerManager` 类名。不要被名称误导：每架机的规划器除读取本机目标、里程计和点云外，还会交换带时间戳的队友 B 样条轨迹，并把机间距离纳入规划与在线检查。

它不是一个集中式“大脑”替三架机一次性求解整条任务。实际结构是 **三套本地规划器并行运行、每套各自重规划、通过共享预测轨迹互相约束**：

```text
/uav1/tower_mission ──目标──> UAV1 EGO-Swarm ──本机轨迹──┐
/uav2/tower_mission ──目标──> UAV2 EGO-Swarm ──本机轨迹──┼─> /swarm/trajectories
/uav3/tower_mission ──目标──> UAV3 EGO-Swarm ──本机轨迹──┘       （共享）
                                      ↑                                    │
                                      └──── 每套规划器读取三机预测轨迹 ──────┘
```

## 它接收什么，输出什么

以第 N 架机为例：

| 类型 | Topic / 参数 | 作用 |
|---|---|---|
| 当前位姿 | `/uavN/Odometry` | FAST-LIO 给出的当前位置、速度和姿态；规划从这里开始。 |
| 环境障碍 | `/uavN/stage3/cloud_registered_filtered` | 经过队友点云、自身机体和地面处理后的本机点云；用于构建本机局部占据地图。 |
| 正式目标 | `/uavN/planning/goal` | 单机任务状态机给出的下一个目标。EGO-Swarm 只负责怎样到达，不决定任务顺序。 |
| 取消请求 | `/uavN/planning/cancel` | 使当前轨迹失效，避免旧轨迹继续由 `traj_server` 执行。 |
| 队友预测 | `/swarm/trajectories`、`/swarm/broadcast_bspline` | 三机共享的、带开始时刻和 `drone_id` 的 B 样条；用于时空避让。 |
| 本机局部轨迹 | `/uavN/planning/bspline` | 在 `uavN/camera_init` 中给 `traj_server` 执行的三次 B 样条。 |
| 规划状态 | `/uavN/planner/status` | 让任务状态机获知规划成功、不可达、地图陈旧等情况。 |

注意：队友不是主要通过“把对方当成点云里的移动障碍物”来避让，而是通过对方未来一段时间会经过哪里来避让。点云过滤首先避免本机局部地图把队友机体残影误当成静态障碍；EGO-Swarm 的机间避让依据是共享的时间化轨迹。

## 从一个目标到一条局部 B 样条

```text
收到 /uavN/planning/goal
  ↓
1. 以当前 odom 和目标构造全局参考
  ↓
2. 在本机膨胀占据地图中选取规划视野内的局部目标
  ↓
3. 生成初始路径，参数化为三次 B 样条控制点
  ↓
4. 优化控制点：平滑、静态障碍距离、动力学可行性、贴近参考、队友轨迹间距
  ↓
5. 检查速度/加速度约束；成功后生成轨迹编号、开始时间和 knots
  ↓
本地 /uavN/planning/bspline ──> traj_server
共享 world B 样条 ─────────────> 队友规划器
```

“B 样条”可以先理解为一条由少数控制点描述的光滑、带时间曲线，而不是许多离散航点的折线。`traj_server` 可以对它求导，得到同一时刻的位置、速度和加速度；这就是下游能连续控制飞机的原因。

规划器并非只在收到新目标时工作。它的执行 FSM 以约 `100 Hz` 检查执行状态，并会在轨迹执行一段时间后重规划；安全检查约 `20 Hz` 检查本机未来可执行轨迹是否进入膨胀障碍或接近队友。若可以重新规划，便发布新 B 样条；若危险已经很近且无法及时重规划，规划器会进入紧急停止轨迹。bridge 和任务状态机还会在其外层继续执行 HOLD、返航和降落策略。

## 两套坐标：本地规划，公共比较

每架机的 EGO-Swarm 在自己的 `uavN/camera_init` 中建图和生成本地执行轨迹；因此 UAV2 和 UAV3 不会把同一个本地 `(x,y,z)` 数字误认为与 UAV1 处于同一物理位置。

但是机间距离必须在同一坐标系中比较。当前约定是：

```text
本机 local B 样条（uavN/camera_init）
       │  使用 swarm_origin 转换
       ▼
公共 B 样条（world，/swarm/trajectories）
       │  队友收到后，按自己的 swarm_origin 反变换
       ▼
队友本地规划坐标中的“预测队友轨迹”
```

默认 `swarm_origin` 分别是 UAV1 `(0,0,0)`、UAV2 `(4,0,0)`、UAV3 `(8,0,0)`，与 `world -> uavN/map` 的初始平移一致。共享消息的 `frame_id` 必须是 `world`；接收者会拒绝 frame 不正确、轨迹格式错误、开始时刻异常或过期的队友轨迹。它也会忽略远到超出本机规划视野的队友预测，避免把无关远处飞机加入本地优化。

## 它怎样避免相撞

EGO-Swarm 的机间检查不是只看“此刻两机是否相撞”，而是比较**同一绝对时间**的预测位置：

```text
对未来时刻 t：
  p_self(t)  = 本机 B 样条在 t 的位置
  p_peer(t)  = 队友 B 样条在 t 的位置

若 ||p_self(t) - p_peer(t)|| < swarm_clearance
  => 当前轨迹冲突，触发重新规划或安全处理
```

当前三机 launch 的 `swarm_clearance` 是 `1.50 m`。规划器在收到新的队友 B 样条时会检查与本机正在执行轨迹的时空冲突；它还会持续检查当前轨迹的前段。这里的距离是对预测轨迹的规划约束，不等同于最终验收所需的真实最小机间距证据。

首次启动时，规划器还保留了按 `drone_id` 的顺序启动机制：`drone_id=0` 可以先生成第一条轨迹，后续编号的规划器等待前序轨迹后再生成首轨迹。之后每次成功重规划都会刷新共享轨迹链，不能把它理解为“UAV1 永远集中指挥 UAV2/UAV3”。

## 当前低空绕塔参数如何影响它

当前默认三机低空配置的关键值是：

| 参数 | 当前值 | 对规划的直接含义 |
|---|---:|---|
| `max_vel` | `0.20 m/s` | 轨迹速度上限。 |
| `max_acc` | `0.50 m/s²` | 轨迹加速度上限。 |
| `planning_horizon` | `7.5 m` | 每次局部规划优先处理的前方范围；不是整项任务的总半径。 |
| `map_resolution` | `0.25 m` | 当前三机 launch 传给 EGO 的占据地图体素边长。 |
| `obstacles_inflation` | `0.40 m` | 点云障碍在地图中额外膨胀的安全边界。 |
| `swarm_clearance` | `1.50 m` | 预测队友轨迹间的最小规划间距。 |
| 本地更新窗口 | `12.5 × 12.5 × 4.5 m` | 低空配置覆盖当前飞机附近、供重规划使用的点云范围。 |

这些是当前 launch/配置的仿真参数，不是通用真机安全保证。特别是点云漏检、`map -> camera_init` 的单位对齐假设失效、队友轨迹超时或控制执行偏差，都可能使真实净空与规划预期不同；所以集群协调、安全监督和 bridge 的检查仍不能省略。

## EGO-Swarm 与其他层的分工

```text
astra_swarm_manager：三机谁可以进场、绕塔、退出（任务时序）
sector_inspection_mission_node：本机下一正式目标是什么（任务语义）
EGO-Swarm：此刻怎样绕开环境与预测队友到该目标（局部时空轨迹）
ego_mavros_bridge：这条轨迹能否安全交给 MAVROS/PX4（执行安全）
```

因此，如果三机间距风险发生，不能只问“EGO-Swarm 为什么没让它们分开”：可能是任务层过早放行、共享轨迹/时间戳/坐标不正确、点云或定位异常、局部规划不可行，或实际飞控执行偏离。排查顺序应是先确认三机 `planner/status` 和共享轨迹是否新鲜，再确认任务许可和 bridge 状态，最后才判断优化器参数是否需要调整。

# 6. 当前三机低空绕塔（阶段 3）的分层架构

先区分两种“分层”的口径，二者不矛盾。

从整个项目的**功能**看，可以粗分为三块：

```text
任务与控制主链：决定并执行三机绕塔
显示层：RViz 展示坐标、点云、轨迹与三机态势
记录层：CSV / JSONL 保存任务状态和验收证据
```

显示层和记录层不参与控制决策：RViz 只订阅并显示数据，记录器只订阅并写入文件；两者停止或异常不应替代任务状态机、规划器或飞控来指挥飞机。因此，这三块不是从上到下的三级控制链，而是“一条主链 + 两个旁路支撑能力”。

如果只沿着真正让无人机飞行的**任务与控制主链**看，“阶段 3”也只是项目阶段编号，不表示软件只有三层。当前三机低空绕塔共有五层主链；其中只有前两层属于“任务管理”。

```text
第 1 层：集群协调
  -> 第 2 层：单机任务
      -> 第 3 层：EGO-Swarm 局部协同规划
          -> 第 4 层：单机执行与控制权
              -> 第 5 层：飞控执行
```

三块功能与五层主链的关系如下：

```text
显示层（RViz）     <──订阅──  五层任务与控制主链  ──发布──> 飞行器
记录层（CSV/JSONL）<──订阅──┘
```

因此，回答“项目有多少层”时：若问功能模块，可说三块（任务与控制、显示、记录）；若问实际控制链，答案是五层主链，外加横向安全监督。

## 第 1 层：集群协调层

核心节点是 `astra_swarm_manager`，整个三机任务只有一个。它根据三架机的状态、预测轨迹和安全许可，决定每架机是否可以进入下一个任务阶段。

它向每架机发放的不是具体坐标，而是许可：起飞、任务开始、进入巡检区、准备绕塔、开始绕塔、换层、退出和降落。也就是说，它回答的是“哪一架机现在可以做什么”。

本层包含：

1. 一个 `astra_swarm_manager`：集群协调与许可发放。
2. 三个 `swarm_state_publisher`：每架机一个，汇总本机任务、桥接、飞控和预测状态后发布给协调器。

## 第 2 层：单机任务层

核心节点是 `sector_inspection_mission_node`，节点名为 `tower_mission`。三架机各有一个实例：`/uav1/tower_mission`、`/uav2/tower_mission` 和 `/uav3/tower_mission`。

这一层把巡检任务拆成可执行的阶段，并在需要时等待第 1 层发放许可。它决定“本机下一站要去哪里”，但不负责计算绕障曲线。

每个单机任务状态机包含的主要小阶段是：

```text
等待输入
  -> 集结点 / 分段爬升
  -> 等待进场许可
  -> 进入巡检区
  -> 等待绕塔准备许可 / 绕塔许可
  -> 评估并锁定当前巡检目标
  -> 导航、绕塔、必要时保持或恢复
  -> 等待换层许可并换层
  -> 等待退出许可
  -> 退出、返航、降落
  -> 完成或报错
```

因此，第 2 层不是“一个总任务状态机”，而是三份独立的单机状态机；它们接受同一个第 1 层协调器的约束。

## 第 3 层：EGO-Swarm 局部协同规划层

每架机运行一套 EGO-Swarm 规划核心和一个 `traj_server`，共三套。EGO-Swarm 是在 EGO-Planner 基础上扩展出的多机规划版本，因此仓库仍保留历史包名 `ego_planner`、节点名 `ego_planner_node` 和类名 `EGOPlannerManager`；名称保留不表示当前三机任务只运行单机 EGO-Planner。

每套的两个核心节点是：

1. `ego_planner_node`：读取本机里程计、点云、膨胀占据地图和第 2 层给出的当前目标，生成或重规划 B 样条轨迹；同时带本机 `drone_id` 发布本机轨迹、接收另外两架机的轨迹，并按 `swarm_clearance` 检查机间冲突。
2. `traj_server`：把协同规划得到的整条 B 样条按时间展开为连续的 `PositionCommand`，包含位置、速度、加速度和航向参考。

这一层回答“到当前目标，怎样绕开环境障碍物和其他无人机飞过去”。它不是任务管理器，也不决定三架机的任务先后次序；起飞、进场、绕塔、换层和返航的次序仍由第 1 层的 `astra_swarm_manager` 管理。

## 第 4 层：单机执行与控制权层

每架机一个 `ego_mavros_bridge`，共三个。它是 EGO 轨迹到 MAVROS/PX4 之间唯一允许持续发布控制指令的出口。

它的主要小阶段包括：连接与输入检查、OFFBOARD 准备、起飞/悬停就绪、跟踪 EGO、HOLD、返航悬停、降落和完成/错误。它回答“这条规划轨迹现在能否安全执行；不能执行时应保持、返航还是降落”。

## 第 5 层：飞控执行层

每架机都有一组 MAVROS 和 PX4 SITL，共三组：

1. MAVROS：将 ROS 的控制消息转换为 MAVLink/PX4 接口，并处理坐标转换。
2. PX4：在 OFFBOARD 模式下执行位置、速度、姿态、推力等底层闭环控制。

这一层不理解“巡检扇区”或“集群许可”，只负责让飞机按照上层给定的连续指令飞行。

## 横向安全层与五层主链的关系

横向安全层不是“第 6 层”，因为它不位于第 1 层到第 5 层的单向命令链上，也不负责把一个任务目标逐层翻译成电机控制。它像覆盖在主链旁边的一条独立安全监督线。

```text
五层主链：集群协调 -> 单机任务 -> EGO-Swarm 规划 -> bridge 执行 -> PX4
                         ↑                                    ↓
横向安全层：读取三机状态、预测轨迹、间距和规划状态；发现风险后阻止继续放行
```

本项目的横向安全层主要是一个 `astra_swarm_safety` 节点。它读取每架机的 `SwarmState`、预测轨迹、当前巡检扇区和规划状态，检查心跳是否新鲜、预测最小间距是否满足要求、是否存在碰撞风险。

它发布 `/swarm/safety/clear`。`astra_swarm_manager` 只有在这个安全结果为允许、且每架机状态正常时，才会继续发放下一阶段许可。因此它与第 1 层的关系是“监督和否决”：安全层不为飞机挑选航点，但可以阻止协调器批准进入、绕塔、换层或降落。

同时，第 4 层的每个 `ego_mavros_bridge` 也有本机独立的输入时效、控制权冲突和轨迹执行安全检查；发现问题会进入 HOLD。两者的分工是：

| 安全机制 | 覆盖范围 | 主要动作 |
|---|---|---|
| `astra_swarm_safety` | 三机之间 | 阻止集群协调器继续放行，避免三机冲突。 |
| `astra_swarm_manager` | 三机任务次序 | 根据安全结果和协调规则，发或不发许可。 |
| `ego_mavros_bridge` | 单架机自身 | 指令异常、控制冲突或执行异常时 HOLD、返航或降落。 |
| EGO-Swarm 核心（节点仍名为 `ego_planner_node`） | 单架机局部环境与机间轨迹 | 避开点云/占据地图中的环境障碍，并以共享 B 样条轨迹避免机间冲突。 |

最简记忆：第 1 层管“三架机能否同时做某事”，第 2 层管“本机接下来去哪里”，第 3 层由 EGO-Swarm 管“本机怎样同时避开环境和队友到那里”，第 4 层管“是否安全地把轨迹交给飞控”，第 5 层管“怎样真的飞出来”；横向安全层持续检查三机是否仍然允许继续执行。

# 7. 三机绕塔项目的 TF 坐标系

本节说明当前 `triple_tower_inspection.launch` 的三机仿真坐标契约。TF（Transform）可以理解为一张“坐标系之间如何平移和旋转”的关系图：每个 frame 都有自己的原点和 XYZ 轴；有了两帧间的 TF，才可以把“在 A 坐标系下的位置”换算成“在 B 坐标系下的位置”。

当前项目不是让三架机共用一个没有前缀的 `map` 或 `body`。它采用一个公共世界坐标系 `world`，下面挂接三棵彼此隔离、带无人机前缀的本机坐标树。这样同名的 FAST-LIO frame 不会互相冲突。

## 总体结构

```text
world                                      三机共享的绝对坐标系
|
|-- uav1/map                               UAV1 的本地地图 / MAVROS 参考系
|    `-- uav1/camera_init                  UAV1 的 FAST-LIO 与 EGO-Swarm 规划系
|         `-- uav1/body                    UAV1 的 FAST-LIO 动态机体系
|              `-- uav1/base_link          UAV1 机体模型基准
|                   |-- uav1/mid360_link   MID360 的安装位置
|                   `-- uav1/d435_link     D435i 的安装位置
|                        |-- uav1/d435_color_optical_frame
|                        |-- uav1/d435_depth_optical_frame
|                        |-- uav1/d435_infra1_optical_frame
|                        `-- uav1/d435_infra2_optical_frame
|
|-- uav2/map
|    `-- uav2/camera_init
|         `-- uav2/body
|              `-- uav2/base_link
|                   |-- uav2/mid360_link
|                   `-- uav2/d435_link -> 各 D435i optical frame
|
`-- uav3/map
     `-- uav3/camera_init
          `-- uav3/body
               `-- uav3/base_link
                    |-- uav3/mid360_link
                    `-- uav3/d435_link -> 各 D435i optical frame
```

箭头 `父坐标系 -> 子坐标系` 的含义是：子坐标系的位置和姿态用父坐标系描述。例如 `world -> uav2/map` 表示 UAV2 的本地原点放在公共世界中的什么位置。

## 三个层次：全局、本机规划、传感器/机体

| 层次 | 坐标系 | 谁使用 | 作用 |
|---|---|---|---|
| 公共全局层 | `world` | 三机 RViz、EGO-Swarm 共享轨迹、集群协调与安全 | 所有无人机、铁塔和环境障碍共有的绝对参考。 |
| 每机本地参考层 | `uavN/map` | MAVROS 位姿、任务层、bridge | 第 N 架机自己的局部 ENU 地图参考；同一座塔在三架机的 local 坐标中数值不同。 |
| 每机规划层 | `uavN/camera_init` | FAST-LIO、点云、EGO-Swarm、`traj_server` | 第 N 架机的规划 frame；当前仿真中与本机 `map` 通过单位静态 TF 对齐。 |
| 动态机体层 | `uavN/body` | FAST-LIO 输出、机体姿态 | 随无人机飞行实时运动；FAST-LIO 发布 `camera_init -> body`。 |
| 物理模型层 | `uavN/base_link` | RViz 机体模型、传感器安装关系 | 机体模型的基准 frame；当前通过单位静态 TF 与 `body` 重合。 |
| 传感器层 | `uavN/mid360_link`、`uavN/d435_link` 及 optical frames | RViz 和传感器几何 | 表示激光雷达、D435i 及其光学坐标系相对机体的固定安装位置。 |

这里 `N` 只能是 `1`、`2` 或 `3`。例如，UAV2 的规划点云应是 `uav2/camera_init`，绝不能写成无前缀的 `camera_init`，否则会与其他飞机的数据混淆。

## 各 TF 的来源与是否变化

| TF | 类型 | 当前来源 | 含义 |
|---|---|---|---|
| `world -> uavN/map` | 静态 | 三机 launch 的 `static_transform_publisher` | 把每架机的本地原点放到公共世界中。 |
| `uavN/map -> uavN/camera_init` | 静态、单位变换 | `ego_gazebo_bridge.launch` | 当前 Gazebo/MID360/FAST-LIO 与 MAVROS ENU 启动时对齐，因此平移为 0、旋转为单位四元数。 |
| `uavN/camera_init -> uavN/body` | 动态 | FAST-LIO，经 `frame_adapter_node.py` 改写为带前缀的 frame 名 | 表示 FAST-LIO 估计出的无人机实时位置和姿态；飞行时持续变化。 |
| `uavN/body -> uavN/base_link` | 静态、单位变换 | `uav_sensor_frames.launch` | 让 FAST-LIO 的机体系与 RViz 机体模型连接；当前两者重合。 |
| `uavN/base_link -> uavN/mid360_link` | 静态 | `uav_sensor_frames.launch` | MID360 位于机体基准上方 `0.08 m`。 |
| `uavN/base_link -> uavN/d435_link` | 静态 | `uav_sensor_frames.launch` | D435i 位于机体前方 `0.12 m`、上方 `0.03 m`，并有固定安装旋转。 |
| `uavN/d435_link -> ...optical_frame` | 静态 | `uav_sensor_frames.launch` | D435i 彩色、深度、红外光学坐标系的固定转换。 |

“静态”表示启动后不会随飞行改变；“动态”表示无人机飞行时持续更新。当前树中真正描述飞行运动的关键边是 `uavN/camera_init -> uavN/body`。

## 套回当前三机项目：静态/动态与单位/非单位

“静态/动态”和“单位/非单位”是两组彼此独立的判断。把它们直接套到当前三机的 TF 树中：

```text
world ──静态、非单位──> uavN/map
      └─ UAV1: (0, 0, 0)；UAV2: (4, 0, 0)；UAV3: (8, 0, 0)

uavN/map ──静态、单位──> uavN/camera_init
uavN/camera_init ──动态、通常非单位──> uavN/body
uavN/body ──静态、单位──> uavN/base_link
uavN/base_link ──静态、非单位──> uavN/mid360_link
uavN/base_link ──静态、非单位──> uavN/d435_link
```

- `world -> uavN/map`：静态，因为三架机的本地原点出生后不移动；非单位，因为 UAV2、UAV3 的本地原点相对共享世界原点分别平移 `+4 m`、`+8 m`。
- `uavN/map -> uavN/camera_init`：当前仿真中的静态单位对齐，表示 MAVROS 本地坐标和 FAST-LIO 规划原点暂时按同一坐标处理；这不是实测的真实传感器标定结果。
- `uavN/camera_init -> uavN/body`：动态 TF。FAST-LIO 持续估计无人机当前位姿；飞行时平移和旋转都会改变，因此通常不是单位变换。
- `uavN/body -> uavN/base_link`：静态单位对齐。当前工程把 FAST-LIO 的机体系与 RViz 机体模型基准视为同一位置、同一朝向。
- `base_link -> mid360_link`、`base_link -> d435_link`：静态但非单位。传感器相对机体不动，但有固定安装偏移；D435 还带有固定安装旋转。

所以判断任意一条 TF 时，分别问两个问题：它会不会随时间改变？它是不是“零平移 + 单位旋转”？前者决定静态或动态，后者决定单位或非单位。

## 默认出生位置下，三棵本地树如何落在同一个世界中

三机 launch 的默认出生位置是 UAV1 `(0, 0, 0)`、UAV2 `(4, 0, 0)`、UAV3 `(8, 0, 0)`，三个 `world -> uavN/map` 都没有初始旋转。因此默认关系为：

```text
world -> uav1/map : 平移 (0, 0, 0)
world -> uav2/map : 平移 (4, 0, 0)
world -> uav3/map : 平移 (8, 0, 0)
```

设一点在第 N 架机本地 `map` 中的坐标为 `(x_local, y_local, z_local)`，则在默认无旋转条件下：

```text
UAV1：p_world = (x_local,     y_local, z_local)
UAV2：p_world = (x_local + 4, y_local, z_local)
UAV3：p_world = (x_local + 8, y_local, z_local)
```

所以铁塔在 `world` 中只有一个绝对位置，但在每架机的 `map/camera_init` 中会有不同的 X 坐标。例如默认塔心世界坐标为 `(-10.0551, 19.7104)` 时：

```text
uav1 本地：(-10.0551, 19.7104)
uav2 本地：(-14.0551, 19.7104)
uav3 本地：(-18.0551, 19.7104)
```

这正是三机任务 launch 为每架机分别传入本地塔心、并让 EGO-Swarm 以 `world` 作为 `swarm_common_frame` 的原因：本地规划各自独立，但队友轨迹比较必须回到同一个公共坐标系。

## 数据怎样走过这棵 TF 树

```text
MID360 激光数据
  -> FAST-LIO 原始输出（原始 frame 名未带 uav 前缀）
  -> frame_adapter_node.py 重写 frame 名
  -> /uavN/Odometry：header.frame_id = uavN/camera_init
                        child_frame_id  = uavN/body
  -> /uavN/cloud_registered：frame_id = uavN/camera_init
  -> EGO-Swarm 在 uavN/camera_init 中建图并规划
  -> 将本机 B 样条按本机 swarm_origin 转成 world 中的共享轨迹
  -> 读取两架队友在 world 中的共享轨迹，进行机间避碰
  -> traj_server 输出本机规划指令
  -> ego_mavros_bridge 通过 uavN/map <-> uavN/camera_init 的 TF
     校验/转换后发布给 /uavN/mavros/setpoint_raw/local
  -> MAVROS/PX4 执行
```

`frame_adapter_node.py` 是多机 TF 的关键适配层：FAST-LIO 本身有硬编码的 `camera_init`、`body` 等 frame 名；适配器把每架机的里程计、点云和 TF 改写为 `uav1/...`、`uav2/...`、`uav3/...`，从而防止三套 FAST-LIO 把同名 frame 发布到同一棵 ROS TF 树中。

## 容易混淆的名称

| 名称 | 是否是当前公共三机 TF frame | 说明 |
|---|---:|---|
| `world` | 是 | 三机共同的绝对 frame。 |
| `uavN/map` | 是 | 第 N 架机局部参考，挂在 `world` 下。 |
| `uavN/camera_init` | 是 | FAST-LIO/EGO-Swarm 的本机规划 frame。 |
| `uavN/body` | 是 | FAST-LIO 动态机体系。 |
| `uavN/base_link` | 是 | RViz/传感器模型 frame。 |
| `odom`、`base_link`（无 `uavN/` 前缀） | 否，属于 MAVROS 外部里程计消息约定 | frame adapter 为 MAVROS 的 `odometry/out` 复制一份使用这些名字的消息，以便 MAVROS 完成 ENU/FLU 到 PX4 NED/FRD 的转换；它们不应被当成三机公共 TF 树中的 frame。 |
| `iris_without_GPS_N::base_link`、`mid360_N::lidar_link` | 否，属于 Gazebo/SDF 模型命名 | 它们描述 Gazebo 模型链接；RViz/ROS 的项目级 TF 使用的是 `uavN/base_link` 和 `uavN/mid360_link`。 |

## 当前约定的边界与排查

`uavN/map -> uavN/camera_init` 的单位变换是当前 Gazebo 仿真的对齐假设，bridge 会在运行中检查位置和 yaw 是否一致；它不是已经完成真机外参标定的结论。真机或非对齐定位系统接入时，不能继续默认单位变换，必须提供经过测量验证的 TF。

启动三机仿真后，可用以下只读命令检查实际树和某一条转换：

```bash
rosrun tf2_tools view_frames.py
rosrun tf tf_echo world uav3/body
rosrun tf tf_echo uav2/camera_init uav2/mid360_link
rostopic echo -n 1 /uav1/Odometry
```

排查顺序：先确认 `world` 到三架机的 `map` 都存在；再确认每架机都有 `map -> camera_init -> body`；最后检查机体和传感器静态分支。若 RViz 中三机重叠、点云漂移或 EGO-Swarm 误判队友位置，优先核对消息的 `header.frame_id`、`world -> uavN/map` 初始平移，以及 `map -> camera_init` 是否仍满足对齐条件。

# 强化学习训练启动与参数修改

本节只说明当前正式的 `worksite.world + Hector + SAC` training-only 路径。
它不会启动 PX4、MAVROS、FAST-LIO 或 `ego_mavros_bridge`。当前 10k 旧 pilot
已经 fail-closed，禁止从旧 checkpoint/replay 继续；下面的命令用于以后创建一个
新的、独立 output directory 并从空 Replay Buffer 开始。

## 1. 环境准备

当前 Hector checkout 是只读外部依赖，源码位于：

```text
/home/yanzu/rl_reference/controllers/hector-quadrotor-noetic
```

Hector 的 Gazebo xacro 有构建期生成文件，因此不要直接在老师 checkout 中构建。
首次使用或 `/tmp` 被清空后，用临时 overlay 构建最小依赖：

```bash
export ASTRA_ROOT=/home/yanzu/AstraDroneOpen
export HECTOR_OVERLAY=/tmp/astra_hector_training_overlay

mkdir -p "$HECTOR_OVERLAY/src"
if [ ! -f "$HECTOR_OVERLAY/devel/setup.bash" ]; then
  cp -a /home/yanzu/rl_reference/controllers/hector-quadrotor-noetic/. \
    "$HECTOR_OVERLAY/src/"
  cd "$HECTOR_OVERLAY"
  source /opt/ros/noetic/setup.bash
  catkin_make -j2 \
    -DCATKIN_WHITELIST_PACKAGES='hector_uav_msgs;hector_gazebo_plugins;hector_quadrotor_model;hector_quadrotor_controller;hector_quadrotor_controller_gazebo;hector_quadrotor_gazebo_plugins;hector_quadrotor_description;hector_quadrotor_gazebo;message_to_tf'
fi
```

构建 Astra 的两个 training package：

```bash
cd "$ASTRA_ROOT"
source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/simulation/sim_workspace/devel/setup.bash"
source "$HECTOR_OVERLAY/devel/setup.bash" --extend

cd "$ASTRA_ROOT/AstraDrone_ros1_ws"
catkin_make -j2 \
  -DCATKIN_WHITELIST_PACKAGES='hector_ego_training_backend;learning_speed_rl'
```

每个新训练终端都执行以下环境设置：

```bash
export ASTRA_ROOT=/home/yanzu/AstraDroneOpen
export HECTOR_OVERLAY=/tmp/astra_hector_training_overlay

source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/simulation/sim_workspace/devel/setup.bash"
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
source "$HECTOR_OVERLAY/devel/setup.bash" --extend

export PYTHONPATH="$ASTRA_ROOT/runtime_artifacts/sac_python_packages${PYTHONPATH:+:$PYTHONPATH}"
export ROS_HOME=/tmp/astra_sac_user
export ROS_LOG_DIR=/tmp/astra_sac_user/logs
mkdir -p "$ROS_LOG_DIR"

cd "$ASTRA_ROOT"
```

当前 PyTorch training dependency 已位于
`runtime_artifacts/sac_python_packages`；固定/Mock 和 full-stack 不依赖它。启动前
先用只读命令确认没有另一套训练 stack 占用默认 ROS/Gazebo master：

```bash
ps -eo pid,stat,cmd | rg 'rosmaster|roslaunch|gzserver|gzclient|sac_training_runner'
```

若已有用户运行，不要直接杀进程；先停止自己的旧运行或在另一个终端显式配置一组
未占用的 `ROS_MASTER_URI`/`GAZEBO_MASTER_URI`。

## 2. 启动强化训练

正式 launch 文件真实存在于：

```text
AstraDrone_ros1_ws/src/MissionControl/hector_ego_training_backend/
  launch/hector_worksite_sac_training.launch
```

它现在默认加载 `learning_speed_rl/config/sac_training_v1.yaml`。Headless 新训练：

```bash
export RUN_ID="sac_controlled_$(date +%Y%m%d_%H%M%S)"
export SAC_OUTPUT="$ASTRA_ROOT/runtime_artifacts/$RUN_ID"
mkdir -p "$SAC_OUTPUT/logs/ros"
export ROS_LOG_DIR="$SAC_OUTPUT/logs/ros"

roslaunch hector_ego_training_backend hector_worksite_sac_training.launch \
  output_dir:="$SAC_OUTPUT" \
  gui:=false \
  runner_mode:=pilot \
  episode_count:=20 \
  max_episode_time:=55.0 \
  run_id:="$RUN_ID" \
  sac_config:="$ASTRA_ROOT/AstraDrone_ros1_ws/src/learning_speed_rl/config/sac_training_v1.yaml"
```

需要 Gazebo GUI 时只改 `gui:=true`：

```bash
roslaunch hector_ego_training_backend hector_worksite_sac_training.launch \
  output_dir:="$SAC_OUTPUT" \
  gui:=true \
  runner_mode:=pilot \
  episode_count:=20 \
  max_episode_time:=55.0 \
  run_id:="$RUN_ID" \
  sac_config:="$ASTRA_ROOT/AstraDrone_ros1_ws/src/learning_speed_rl/config/sac_training_v1.yaml"
```

当前 launch **没有** `target_valid_transitions:=...` 参数。训练 target 从
`sac_training_v1.yaml` 的 `training.target_valid_transitions` 读取；当前是 10000。
若开启一个新的实验并修改 target，还要同步检查：

- `episode.max_steps=500`；无真实 early terminal 时，`episode_count` 至少应覆盖
  `target / 500`；
- `training.checkpoint_steps` 不能超过 target，且应包含最终 target；
- 每次必须使用全新 `RUN_ID`/`SAC_OUTPUT`，不能覆盖或续接旧 pilot。

当前建议先运行 qualification 或小规模受控实验并检查 checkpoint/evaluation，再
决定是否做新的 10k；不要把上述命令扩成 100k/1M 默认流程。

## 3. 如何停止训练

正常停止方式是在启动 `roslaunch` 的终端按一次 `Ctrl-C`，等待 Gazebo、controller
和 ROS nodes 完成 teardown。不要把 `kill -9` 当正常停止方法。

pilot 只在 `training.checkpoint_steps` 指定的 environment step 保存 checkpoint；
任意时刻 `Ctrl-C` **不会承诺额外保存一个最新 checkpoint**。因此中止后应保留：

- 已经落盘的 `sac_checkpoint_step_*.pt`；
- `sac_checkpoint_manifest.json`；
- `sac_transition_audit.jsonl`、`sac_learner_metrics.jsonl`；
- Episode/reset/qualification JSON 与 `logs/`。

所有输出都在本次 `$SAC_OUTPUT` 下；不得移动到 `test_evidence/`，也不得覆盖历史
runtime directory。

## 4. Checkpoint

pilot checkpoint 直接保存在本次 output root：

```text
$SAC_OUTPUT/sac_checkpoint_step_00000.pt
$SAC_OUTPUT/sac_checkpoint_step_02000.pt
...
$SAC_OUTPUT/sac_checkpoint_manifest.json
```

qualification 模式使用名称 `sac_smoke_checkpoint.pt`。checkpoint 包含 Actor、
Q1/Q2、target Q1/Q2、actor/critic/alpha optimizer、log-alpha、environment/update
counters 和完整 SAC config。

**当前 training resume 未实现。** `runner_mode:=pilot` 总是从新 agent 与空 Replay
Buffer 开始，不能把 `evaluation_checkpoint_path` 冒充训练 resume。

当前支持只读 deterministic evaluation，不写 training replay、不更新网络。示例：

```bash
export EVAL_ID="sac_eval_$(date +%Y%m%d_%H%M%S)"
export EVAL_OUTPUT="$ASTRA_ROOT/runtime_artifacts/$EVAL_ID"

roslaunch hector_ego_training_backend hector_worksite_sac_training.launch \
  output_dir:="$EVAL_OUTPUT" \
  gui:=false \
  runner_mode:=evaluation \
  episode_count:=3 \
  run_id:="$EVAL_ID" \
  evaluation_checkpoint_path:="$SAC_OUTPUT/sac_checkpoint_step_02000.pt" \
  evaluation_checkpoint_step:=2000 \
  sac_config:="$ASTRA_ROOT/AstraDrone_ros1_ws/src/learning_speed_rl/config/sac_training_v1.yaml"
```

checkpoint 的 config 必须与 evaluation 加载的 config 完全一致。

## 5. 参数在哪里修改

除非开启一个新的实验，优先只改 `sac_training_v1.yaml` 中明确属于 SAC 的字段。
路径均相对项目根目录。

| 参数 | 当前值 | 文件路径 | 字段名 | 含义 | 是否建议轻易修改 |
|---|---:|---|---|---|---|
| total valid transitions | `10000` | `AstraDrone_ros1_ws/src/learning_speed_rl/config/sac_training_v1.yaml` | `training.target_valid_transitions` | 新 pilot 的 valid experience target | 否；先用受控规模和 evaluation 决定 |
| Replay Buffer capacity | `100000` | 同上 | `replay.capacity` | 最多保留多少条 transition，不是训练步数 | 否 |
| learning starts | `1000` | 同上 | `training.learning_starts` | 到多少条经验后启用 learner/stochastic training | 否 |
| batch size | `64` | 同上 | `sac.batch_size` | 每次 gradient update 的 replay 样本数 | 一般否 |
| actor LR | `1e-5` | 同上 | `sac.policy_learning_rate` | Actor Adam learning rate；已针对 raw 3267-D 输入做 stability qualification | 不建议 |
| critic LR | `1e-3` | 同上 | `sac.critic_learning_rate` | twin Q Adam learning rate | 不建议 |
| alpha LR | `1e-3` | 同上 | `sac.alpha_learning_rate` | automatic entropy coefficient learning rate | 不建议单独试改 |
| gamma | `0.99` | 同上 | `sac.gamma` | discounted return | 否 |
| tau | `0.005` | 同上 | `sac.tau` | target critic Polyak update rate | 否 |
| target entropy | `-1.0` | 同上 | `sac.target_entropy` | 一维 tanh policy 的 entropy target | 不建议 |
| log-std range | `[-3,-1]` | 同上 | `sac.log_std_min/max` | Normal exploration std 的可训练范围 | 不建议；旧 `[-5,2]` 已失败 |
| critic-only startup | `100 updates` | 同上 | `sac.critic_warmup_updates` | 隔离随机 critic transient 后才更新 actor/alpha | 不建议 |
| learner update Hz | `5.0` | 同上 | `sac.updates_per_second` | 独立 wall-time learner 频率 | 一般否；需复验 10 Hz scheduler |
| policy update frequency | `2` | 同上 | `sac.policy_frequency` | warm-up 后每 2 个 learner update 更新一次 actor/alpha | 否 |
| seed | `1` | 同上 | `sac.seed` | NumPy/PyTorch/replay sampling seed | 比较实验可显式换，但必须记录 |
| checkpoint steps | `[0,2000,...,10000]` | 同上 | `training.checkpoint_steps` | runner 实际保存 checkpoint 的 step allowlist | 修改 target 时同步修改 |
| checkpoint interval | `2000` | 同上 | `training.checkpoint_interval` | 当前为实验元数据；runner 真正读取 `checkpoint_steps` | 不要只改这一项 |
| evaluation interval | `2000` | 同上 | `training.evaluation_interval` / `evaluation_steps` | 当前为编排元数据；runner 不自动启动 evaluation | 用独立 evaluation 命令 |
| action minimum | `0.05 m/s` | `.../config/speed_adapter.yaml` 与 `sac_training_v1.yaml` | `safety.v_max_min` / `action.expected_v_max_min` | live safety 下限及 SAC 一致性门 | 不建议 |
| action maximum | `0.40 m/s` | worksite SAC launch 与 `sac_training_v1.yaml` | `v_max_max` / `action.expected_v_max_max` | training EGO/SafetyFilter 静态上限 | 不建议 |
| Hover | `(0,0,3)` | `.../launch/hector_worksite_training_episode_reset.launch` | `hover_x/y/z/yaw` include args | worksite reset checkpoint | 否 |
| ENTRY_GATE | `(-4.3148485145,5.8522070123,3)` | 同上 | `entry_x/y/z` include args | worksite 正式 Episode goal | 否 |
| max Episode time | `55 s` coordinator；`500 steps` env | `hector_worksite_sac_training.launch` / `sac_training_v1.yaml` | `max_episode_time` / `episode.max_steps` | 最先达到者形成 terminal/truncation | 谨慎，二者一起审计 |
| Observation C | `3267` | `.../config/observation_c_trajectory_fusion.yaml` | `expected_lidar_bins=3200`, `trajectory_sample_count=20` | 3200 + 60 + 3 + 3 + 1 | 禁止随意修改 |
| Mid360 rate | `10 Hz` | `.../urdf/quadrotor_mid360_training.gazebo.xacro` | `sensor/update_rate` | training raw PointCloud2 source rate | 否 |
| lidar history | `5 frames` | `.../config/observation_v2_lidar_surrogate.yaml` | `history_frames`, `minimum_history_frames` | truth-pose causal aligned history | 禁止随意修改 |
| Stage 1 Reward | 当前 calibrated v1 | `AstraDrone_ros1_ws/src/learning_speed_rl/config/stage1_reward.yaml` | `reward.*` | 唯一正式 Reward 配置 | 本轮不修改 |
| Reset coordinator | worksite overrides + base defaults | `.../config/training_episode_reset.yaml` 与 worksite launch | `hover_pose`, `entry_goal`, barriers/timeouts | terminal、teleport、generation 和 readiness owner | 禁止随意修改 |

表中的 `...` 分别指 `AstraDrone_ros1_ws/src/learning_speed_rl` 或
`AstraDrone_ros1_ws/src/MissionControl/hector_ego_training_backend` 的对应前缀。

## 6. 关键概念

- `1 step = 1 transition = Replay Buffer 中 1 条经验`；前提是 identity、causality、
  Observation 和 reward 合同全部闭合。
- `Replay Buffer capacity` 是最多保留多少条经验，**不等于** total training
  steps。当前 capacity 100000，pilot target 10000。
- 一个 Episode 由若干 step 组成；当前最多 500 step，也可能因真实 terminal 提前
  结束。真实 planner/collision/tracking failure 不得补跑来凑数。
- `gradient update` 从 Replay Buffer 抽一个 batch 更新网络，**不等于** environment
  step。当前 learner 约 5 update/s，而 environment scheduler 是 10 step/s。
- `learning_starts=1000` 表示先收集 1000 条 experience；达到后才启用 learner。
  当前又额外先做 100 次 critic-only update，再更新 actor/alpha。

## 7. 当前推荐训练流程

1. 使用当前 `sac_training_v1.yaml` 做 package tests 和 launch expansion。
2. 先做 5-Episode qualification，确认至少覆盖 1000 warm-up、正式 stochastic
   SAC 和多个 Episode；检查 action std/log-std/alpha/entropy、delta、planner、
   reset、10 Hz、identity、replay 与 numerical gates。
3. qualification PASS 后，创建**新的空 replay**受控训练；不要加载旧 10k pilot。
4. 到 checkpoint step 后停止或做独立 deterministic evaluation；比较 return、
   action distribution、planner/terminal 与 seed，而不是只看 loss。
5. 只有多个 checkpoint/evaluation 仍稳定且真实 failure 边界清楚，才讨论扩大规模。
   当前不默认启动 100k/1M。

## 8. 不要随意修改的项目

- Observation C 3267 contract；
- 3200-bin surrogate、semantic 和 unknown estimator；
- EGO core、clearance 和 planner 参数；
- Hector model、Pose/Twist PID；
- Learning Speed dynamic-`v_max` force-replan thresholds；
- SpeedSafetyFilter `[0.05,0.40]` 与禁止 slew/low-pass/hysteresis 的合同；
- request/action/applied Episode identity、reset generation 和 causal barriers；
- full-stack FAST-LIO + PX4/MAVROS 路径及其独立 provenance。

如需修改其中任何一项，应开启新的显式实验，使用全新 runtime ID，并保留真实
失败；不得为了让 SAC 实验 PASS 而顺手改变它们。
