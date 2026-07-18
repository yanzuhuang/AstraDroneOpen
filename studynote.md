# AstraDrone 仿真、控制与 EGO-Planner 学习笔记

## 1. 当前学习范围与顺序

本笔记按“基础控制 → 连续轨迹 → 定位建图 → 局部避障规划”的顺序组织。当前已经从 PX4/Gazebo/MAVROS/Offboard 基础控制进入 FAST-LIO 与 EGO-Planner 集成阶段；仍不需要修改 PX4 内环。

推荐顺序：

```text
跑通 pc_example.sh
  -> 理解 PX4、Gazebo、MAVROS 的分工
  -> 读懂默认 Offboard 状态机
  -> 起飞、悬停、降落
  -> 航点和轨迹
  -> 数据验证
  -> FAST-LIO 定位
  -> EGO-Planner 和避障
```

## 2. 把主控制链看成“领导关系”

```text
autoarming_control.cpp：任务层，决定“飞到哪里、何时起降”
        ↓ ROS 位置目标
MAVROS：翻译层，在 ROS 消息与 MAVLink/PX4 之间转换
        ↓
PX4：飞控层，根据目标计算速度、姿态、推力和电机输出
        ↓
Gazebo：物理层，模拟无人机、传感器、重力和运动
        ↓ 实际位置反馈
MAVROS -> autoarming_control.cpp，形成闭环
```

类比开车：控制节点是导航，MAVROS 是翻译员，PX4 是司机，Gazebo 是虚拟车辆和道路。

关键边界：`autoarming_control.cpp` 只生成上层 setpoint，不直接控制姿态和电机；PX4 内部控制器负责底层闭环。

默认的 `pc_example.sh + autoarming_control` 链路中，FAST-LIO 虽由脚本启动，但 `autoarming_control` 不订阅它的 `/Odometry`，EGO-Planner 也不会自动接管无人机。阶段 6 使用独立的 `stage6_gazebo.launch` 和 `ego_mavros_bridge`，才把 FAST-LIO、EGO-Planner 与 PX4/Gazebo 安全地连接起来。

## 3. 主控制链术语

| 术语 | 理解 |
|---|---|
| ROS1 | 机器人程序通信框架，节点通过 topic、service、parameter 协作。 |
| `roscore` | ROS1 的通信登记中心。 |
| node | 一个正在运行的 ROS 程序，如 `autoarming_control`。 |
| topic | 持续发布/订阅的数据通道，如位置和 setpoint。 |
| service | 一次请求和响应，如解锁、切换模式。 |
| launch | 同时启动节点并配置参数的 XML 文件。 |
| PX4 | 无人机飞控软件，执行状态估计和位置/速度/姿态/电机控制。 |
| SITL | Software In The Loop，在电脑上运行的虚拟 PX4 飞控。 |
| FCU | Flight Control Unit；在当前仿真中主要指 PX4 SITL。 |
| Gazebo | 模拟世界、无人机动力学、碰撞和传感器的物理仿真器。 |
| `iris_mid360` | 搭载 Mid-360 激光雷达的四旋翼仿真模型。 |
| MAVLink | PX4 与外部程序通信使用的无人机协议。 |
| MAVROS | ROS1 与 PX4/MAVLink 之间的桥梁，并处理许多坐标转换。 |
| Offboard | PX4 接受外部计算机持续发送目标值的飞行模式；不是绕过 PX4。 |
| setpoint | 目标值，例如“希望飞到 `(x,y,z)`”。 |
| local pose | 无人机当前的局部位置和姿态，不是经纬度。 |
| Arm/Disarm | 解锁/上锁；解锁后才允许电机产生正常飞行推力。 |
| QGC | QGroundControl 地面站，用于查看状态、告警和参数。 |
| FAST-LIO | 激光雷达与 IMU 定位建图算法；当前不在默认控制闭环中。 |
| planner | 根据状态、目标和环境约束计算路径或轨迹的规划器。 |
| path | 只描述经过哪些位置，不一定包含时间。 |
| trajectory | 描述每个时刻的位置、速度和加速度，是可执行的时间化路径。 |
| EGO-Planner | 基于局部占据地图和 B 样条优化的局部轨迹规划器。 |
| planning frame | 规划器统一使用的坐标系；阶段 6 默认为 `camera_init`。 |

三个重要话题：

```text
/mavros/state                    PX4 是否连接、当前模式、是否解锁
/mavros/local_position/pose      实际局部位姿反馈
/mavros/setpoint_position/local  控制节点发布的目标位置
```

## 4. 外部软件与仓库文件的区别

| 内容 | 主要位置 |
|---|---|
| 完整 PX4 源码 | `~/PX4-Autopilot` |
| 已安装 MAVROS | `/opt/ros/noetic/share/mavros` 等系统目录 |
| Astra 的 PX4/Gazebo 适配 | `simulation/px4_sim_files/` |
| Gazebo 场景 | `simulation/astra_gazebo_worlds/` |
| Offboard 控制代码 | `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/` |
| FAST-LIO 源码 | `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/` |
| EGO-Planner 源码 | `AstraDrone_ros1_ws/src/Planner/ego-planner/` |
| EGO 到 MAVROS 的集成与安全桥 | `AstraDrone_ros1_ws/src/MissionControl/ego_gazebo_bridge/` |

PX4、MAVROS、Gazebo 是独立软件；本仓库主要保存安装脚本、项目配置、模型、场景和上层控制代码。

下文的 `offboard/` 指 `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/`。

## 5. 现阶段最重要的文件

### `scripts/run_sh/pc_example.sh`

职责：用 tmux 一键启动 `roscore`、仓库内 `astra_example.launch`、FAST-LIO、Offboard 控制节点和 QGC。

何时修改：需要改变启动模块、启动顺序或等待时间时。它是启动器，不应放飞行轨迹算法。

### `simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch`

职责：配置并启动 PX4 SITL、Gazebo、MAVROS，设置 world、机型、出生位置、SDF 和通信地址。

常用参数：

| 参数 | 作用 | 何时修改 |
|---|---|---|
| `world` | Gazebo 场景 | 切换空场、森林、动态避障场景 |
| `vehicle` | 无人机型号 | 更换传感器或无 GPS 机型 |
| `x/y/z/R/P/Y` | 出生位置和姿态 | 调整初始位置 |
| `sdf` | 无人机模型文件 | 更换模型结构或传感器 |
| `fcu_url` | MAVROS 与 PX4 的通信地址 | 多机或端口变化时 |

当前 `pc_example.sh` 直接加载仓库内这份 launch，不再依赖 `~/PX4-Autopilot` 中的同名副本。

### `offboard/launch/autoarming_control.launch` 与 `stage4_trajectory.launch`

`autoarming_control.launch` 启动阶段 3 航点任务，从 YAML 读取相对 home 的航点；`stage4_trajectory.launch` 启动圆、方形、8 字或椭圆连续轨迹。两者都设置 `map -> camera_init` 静态 TF、集中配置实验参数，并可启动 RViz。只改 launch/YAML 参数通常不需要重新编译。

连续轨迹常用参数包括 `trajectory_type`、`speed`、`yaw_mode`、`target_laps`、`radius`、`side_length`、`ellipse_a/b`、`max_tracking_error` 和 `loop_rate`。新增参数时要同时完成：launch 定义参数，C++ 读取参数，并让参数真正参与计算。

### `offboard/src/autoarming_control.cpp`

职责：Offboard 会话、解锁、目标位置生成和任务状态机，是基础控制阶段最重要的修改文件。

当前流程：

```text
等待 FCU
  -> 以 20 Hz 预发送 100 个当前位置 setpoint
  -> 请求 OFFBOARD
  -> 请求 Arm
  -> TAKEOFF
  -> INITIAL_HOVER
  -> WAYPOINTS，或 TRAJECTORY_ENTRY -> TRACKING
  -> RETURN_HOME
  -> 请求 PX4 AUTO.LAND
  -> PX4 报告落地并自动上锁后 COMPLETED
```

适合修改：起飞点、悬停、航点、预设轨迹、yaw、状态转换、到达门限、超时和安全处理。当前源码已经包含有效位姿检查、home 记录、合法四元数、悬停、轨迹速度、跟踪误差门限、超时、返航和 PX4 原生降落。

### 其他文件何时关注

| 需求 | 文件位置 |
|---|---|
| 修改障碍物和场景 | `simulation/astra_gazebo_worlds/*.world` |
| 修改机体、传感器安装 | `simulation/px4_sim_files/px4_iris_sdf/*.sdf` |
| 修改 PX4 机架/EKF 参数 | `simulation/px4_sim_files/px4_iris_params/*`，基础阶段暂缓 |
| 新增 C++ 可执行节点 | `offboard/CMakeLists.txt` |
| 新增 ROS 依赖 | `offboard/package.xml` |
| 修改 RViz 显示 | `offboard/rviz_config/*.rviz` |
| 学习定位 | `SLAM/FAST_LIO/` 配置与源码 |
| 学习规划避障 | `Planner/ego-planner/`，基础控制稳定后再进入 |
| 将 EGO 轨迹交给 PX4 | `MissionControl/ego_gazebo_bridge/` |

不要修改 `build/`、`devel/` 生成文件，也不要同时运行多个节点向同一架无人机持续发布 setpoint。

当前 `position_control*` 缺少完整的 OFFBOARD/解锁和可靠循环，不作为第一条学习主线。

## 6. 启动、换场景和编译

完整启动命令保持不变：

```bash
cd ~/AstraDroneOpen
./scripts/run_sh/pc_example.sh
```

切换 world：只修改 `astra_example.launch` 第 15 行末尾的文件名，例如：

```xml
example.world
forest.world
dynamic_avoidance.world
```

修改 world/launch 后不用编译，但必须完全关闭旧 Gazebo/PX4 后重新启动。

修改 `autoarming_control.cpp` 后：

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make
source devel/setup.bash
```

只修改 `autoarming_control.launch` 参数，一般重新启动该 launch 即可。

## 7. 基础检查命令

```bash
rospack find px4
rospack find mavros
rospack find offboard

rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/local_position/pose
rostopic echo /mavros/setpoint_position/local
rostopic hz /mavros/setpoint_position/local
rostopic info /mavros/setpoint_position/local
```

排查顺序：

```text
FCU connected
  -> pose 是否持续有效
  -> setpoint 是否只有一个发布者且频率稳定
  -> 是否进入 OFFBOARD
  -> 是否 Armed
  -> 目标值是否正确
  -> 实际位置是否跟随
```

## 8. 基础控制阶段的第一个改造任务

先在 `autoarming_control.cpp` 实现：

```text
等待有效位姿
  -> 记录起飞点
  -> 垂直起飞到相对高度
  -> 定点悬停 10 秒
  -> 垂直降落
  -> 确认上锁
```

这项基础任务在当前 `autoarming_control.cpp` 中已经扩展为航点、连续轨迹、返航、故障悬停和 `AUTO.LAND` 状态机。后续重点是理解 EGO-Planner，并通过阶段 6 桥接完成真正的局部避障闭环。

## 9. EGO-Planner 与项目现有逻辑的关系

### 9.1 先给结论

仓库中的规划器并不是一个“与 EGO-Planner 类似的自研 planner”：`AstraDrone_ros1_ws/src/Planner/ego-planner/` 放的就是 EGO-Planner 源码。需要区分的是“规划器”“任务/轨迹发生器”和“控制桥”三个角色：

| 模块 | 它决定什么 | 是否读取障碍地图 | 是否局部重规划 | 是否直接向 MAVROS 发 setpoint |
|---|---|---:|---:|---:|
| EGO-Planner | 接下来怎样平滑、安全地绕障飞 | 是 | 是 | 否，先输出 B 样条和 `PositionCommand` |
| `autoarming_control` | 何时起飞、去哪个航点、沿哪种预设几何轨迹飞、何时返航/降落 | 否 | 否 | 是 |
| `ego_mavros_bridge` | 谁拥有控制权，以及何时接管、保持、返航或降落 | 不负责规划 | 不负责规划 | 是，把 EGO 指令转换为 PX4 位置目标 |
| `rc_obstacle_avoidance` | 把遥控量变成 Fast-Planner 目标并转发规划结果 | 自身不建图 | 依赖外部 Fast-Planner | 是，但包当前被 `CATKIN_IGNORE` 禁用 |

因此，`autoarming_control` 中的“圆/方形/8 字/椭圆轨迹逻辑”和 EGO 的共同点只是：都会产生随时间变化的位置目标，也都有任务状态和安全判断。核心算法并不相同：前者根据公式或航点查表，环境中即使出现障碍物也不会改变路线；后者根据里程计、目标和局部地图优化 B 样条，并在执行中检查碰撞、重新规划或急停。

`rc_obstacle_avoidance` 的 README 提到 Fast-Planner，但仓库内没有对应的 Fast-Planner 实现，而且该包当前被禁用，所以它不是当前 EGO 集成主线。

### 9.2 哪个更加好用

没有脱离任务场景的绝对优劣：

- 学习 OFFBOARD、验证 PX4 跟踪、空场按固定图形飞行：`autoarming_control` 更好用。依赖少、行为确定、参数直观，故障也更容易定位。
- 森林、仓库等有障碍环境，需要自主绕障和在线重规划：EGO-Planner 更好用。预设轨迹发生器不具备这一能力。
- 当前项目要做完整避障飞行：最合适的组合是“EGO-Planner 负责轨迹 + `ego_mavros_bridge` 负责 PX4 接管和安全”。两者不是二选一，桥接层补上了 EGO 原生输出与 MAVROS/PX4 之间的接口、坐标系和安全状态机。
- 只想验证规划算法是否运行：可用 EGO 自带 SO3 模拟器；要验收本项目：必须看 PX4 控制的 Gazebo 物理无人机是否真正跟踪并避障，不能只看 RViz 曲线。

## 10. EGO-Planner 是什么

EGO-Planner 是四旋翼无人机的局部轨迹规划器。它根据无人机当前状态、目标点和附近障碍物，生成安全、平滑且满足速度和加速度限制的三维轨迹。它回答“无人机接下来应该怎样飞”，但它不是相机、SLAM、飞控或底层控制器。

```text
相机 / 激光雷达
        ↓ 深度图 / 点云
定位与局部占据地图
        ↓
EGO-Planner
        ↓ B 样条轨迹
traj_server
        ↓ 当前时刻的位置、速度、加速度和 yaw
控制器 / 桥接层
        ↓ setpoint
PX4 -> Gazebo 中的无人机
```

### 10.1 路径和轨迹

- 路径（path）只描述从起点到终点经过哪些位置，不一定包含时间信息。
- 轨迹（trajectory）还描述什么时间到达什么位置，并可求出速度、加速度和 jerk。

可以记成：路径像地图上的路线；轨迹像带时间安排的行程表。EGO 输出的是轨迹，不只是离散路径点。

### 10.2 基本工作流程

```text
获取里程计、目标和障碍地图
  -> 生成初始轨迹
  -> 检查碰撞
  -> 为碰撞段寻找无碰撞引导路径
  -> 调整并优化 B 样条控制点
  -> 检查速度和加速度可行性
  -> 发布轨迹
  -> 执行一小段
  -> 根据新感知继续重规划
```

这种“感知 → 建图 → 规划 → 执行 → 再感知 → 再规划”就是局部重规划。

## 11. 占据地图、膨胀地图与 ESDF

### 11.1 占据地图

占据地图把三维空间划分成体素，记录空闲、占据或未知，回答“这里有没有障碍物”。它不能直接给出当前位置到障碍物的距离。

```text
□ □ ■ □
□ □ ■ □
□ □ □ □
```

### 11.2 障碍物膨胀地图

无人机不是质点。规划无人机中心轨迹时，需要按机身/旋翼尺寸、定位与地图误差以及安全余量，把障碍物向外扩大：

```text
原障碍物：      膨胀后：

    ■             ■ ■ ■
                  ■ ■ ■
                  ■ ■ ■
```

膨胀地图本质上仍是占据地图，它回答“为机体和误差留出空间后，这里还能不能通过”。项目 `grid_map.cpp` 同时维护原始占据缓存和膨胀占据缓存，阶段 6 的 `obstacles_inflation` 控制膨胀尺度。

### 11.3 ESDF

ESDF（Euclidean Signed Distance Field，欧氏有符号距离场）为地图位置保存到最近障碍物的欧氏距离。常见约定是障碍物外为正、表面为零、内部为负，但具体系统也可能相反。

```text
2.8  2.2  2.0  2.2  2.8
2.2  1.4  1.0  1.4  2.2
2.0  1.0  0.0  1.0  2.0
2.2  1.4  1.0  1.4  2.2
```

ESDF 既能回答“离障碍物多远”，其梯度还能表示“往哪个方向移动能最快远离障碍物”。

| 地图 | 保存的信息 | 回答的问题 |
|---|---|---|
| 占据地图 | 空闲、占据、未知 | 这里有没有障碍物？ |
| 膨胀地图 | 扩大后的占据区域 | 留出安全空间后能否通过？ |
| ESDF | 到最近障碍物的距离 | 离障碍物多远，往哪里更安全？ |

```text
点云 / 深度图
      ↓
占据地图
      ├── 障碍物膨胀 -> 膨胀地图
      └── 距离场计算 -> ESDF
```

记忆：占据地图看“有没有”，膨胀地图看“留出安全距离后能不能过”，ESDF 看“多远、往哪里躲”。

## 12. “ESDF-free”真正表示什么

完整 ESDF 需要为局部地图的大量体素计算并持续更新最近障碍物距离和方向。传感器带来新点云后，地图变化，已有距离也可能失效，持续更新这个数据结构就是“维护 ESDF”。

EGO-Planner 不提前构建并持续维护覆盖整个局部地图的完整 ESDF，而是围绕当前轨迹处理碰撞信息：

```text
检查当前轨迹
  -> 发现碰撞段
  -> 用 A* 等方法为该段寻找无碰撞引导路径
  -> 提取障碍物锚点和推出方向
  -> 优化碰撞附近的 B 样条控制点
```

所以 ESDF-free 不等于不需要地图、不检查碰撞、不考虑安全距离或不知道避障方向；它只是不把“持续维护完整 ESDF”作为前置条件。可以记成：轨迹碰到哪里，就重点处理哪里。

## 13. B 样条与轨迹优化

### 13.1 B 样条

B 样条用一组控制点生成平滑曲线。控制点像牵引一根柔软绳子的支点，会影响曲线形状，但曲线通常不逐个经过所有控制点。

```text
P0      P1      P2      P3
●-------●-------●-------●
```

它适合无人机规划，因为：

- 曲线平滑，容易求位置、速度和加速度；
- 修改一个控制点主要影响附近轨迹，适合局部避障；
- 控制点可以作为数值优化变量；
- 容易检查速度、加速度等动力学限制。

初始轨迹穿过障碍物时，EGO 会识别碰撞附近的控制点并沿安全方向移动，再由新控制点生成平滑轨迹。控制点是“塑造曲线的变量”，不是无人机必须逐一到达的航点。

### 13.2 时间与动力学可行性

B 样条包含时间参数，因此可在任意时刻求位置、速度和加速度。若轨迹超过最大速度或最大加速度，EGO 会重新分配时间或再次细化优化，而不是只判断几何上是否绕开障碍。

### 13.3 优化目标

源码中的主要代价可概括为：

```text
总代价
  = 平滑代价
  + 碰撞/安全距离代价
  + 动力学可行性代价
  + 细化阶段的轨迹贴合代价
```

- 平滑代价抑制突然转弯和抖动。
- 碰撞代价把控制点从障碍物附近推开。
- 可行性代价惩罚超过速度和加速度限制的轨迹。
- 贴合代价用于细化时避免轨迹偏离已有参考过多。

`lambda_smooth`、`lambda_collision`、`lambda_feasibility`、`lambda_fitness` 是相应权重，`dist0` 是优化器希望保持的避障距离尺度。参数越保守并不总越好：膨胀或 `dist0` 过大时，窄通道可能无解。

## 14. EGO 源码职责与状态机

| 包或文件 | 主要职责 |
|---|---|
| `plan_env/grid_map.cpp` | 融合点云/深度和里程计，维护占据与膨胀占据地图 |
| `path_searching/dyn_a_star.cpp` | 为碰撞轨迹段寻找无碰撞引导路径 |
| `bspline_opt/bspline_optimizer.cpp` | 构造并优化 B 样条控制点代价 |
| `plan_manage/planner_manager.cpp` | 生成全局参考、局部目标、初始轨迹并组织优化/时间重分配 |
| `plan_manage/ego_replan_fsm.cpp` | 接收目标和里程计，决定生成、执行、重规划或急停，并发布 B 样条 |
| `plan_manage/traj_server.cpp` | 按当前时间对 B 样条求值，持续输出位置、速度、加速度和 yaw |
| `ego_gazebo_bridge` | 校验坐标和输入新鲜度，把规划指令转换成 MAVROS 位置 setpoint，并管理接管、保持和降落 |

EGO 的核心 FSM 可简化为：

```text
INIT
  -> WAIT_TARGET
  -> GEN_NEW_TRAJ
  -> EXEC_TRAJ
       ├── 正常推进后 REPLAN_TRAJ -> EXEC_TRAJ
       ├── 发现未来碰撞后立即尝试重规划
       └── 障碍太近且重规划失败 -> EMERGENCY_STOP
  -> 到达后 WAIT_TARGET
```

这正是它与预设轨迹发生器最本质的区别：`autoarming_control` 的 FSM 管任务阶段和 PX4 会话；EGO 的 FSM 管局部轨迹的生命周期与碰撞风险。

## 15. ROS 数据流：原生演示与本项目阶段 6

### 15.1 EGO 原生 SO3 演示

```text
mockamap_node -> 仿真全局障碍物
pcl_render_node -> 局部点云
waypoint_generator -> 目标航点
ego_planner_node -> /planning/bspline
traj_server -> /planning/pos_cmd（经 launch 重映射）
so3_control -> 控制指令
quadrotor_simulator_so3 -> 新里程计 -> 反馈给规划器
```

原生演示常见输入为 `/visual_slam/odom`、`/map_generator/global_cloud` 和 `/pcl_render_node/cloud`。这些话题用于 EGO 自带模拟环境，不应机械地当成本项目阶段 6 的实际接口。

### 15.2 AstraDrone 阶段 6 实际数据流

```text
FAST-LIO /Odometry + /cloud_registered
                 ↓
      EGO 局部地图与规划 FSM

/move_base_simple/goal
        ↓ ego_mavros_bridge 校验并变换坐标
/planning/goal
        ↓ waypoint_generator
/waypoint_generator/waypoints
        ↓ ego_planner_node
/planning/bspline
        ↓ traj_server，100 Hz 对轨迹求值
/planning/pos_cmd
        ↓ ego_mavros_bridge，校验/限速/50 Hz 持续发布
/mavros/setpoint_position/local
        ↓ MAVROS -> PX4 -> Gazebo 物理无人机
        └──────── 位姿与传感器反馈
```

关键区别：`/planning/bspline` 是一整条新轨迹，只在生成或重规划时发布；`/planning/pos_cmd` 是对当前轨迹按时间求值后的瞬时参考，包含位置、速度、加速度和 yaw。阶段 6 第一版桥接只把位置与 yaw 转为 `PoseStamped`，没有把速度、加速度前馈交给 PX4。

### 15.3 主要节点

| 节点 | 作用 |
|---|---|
| `mockamap_node` | EGO 原生演示中生成仿真障碍物地图 |
| `pcl_render_node` | EGO 原生演示中模拟局部点云/深度感知 |
| `waypoint_generator` | 把目标转换为规划器航点 |
| `ego_planner_node` | 建图、局部重规划和 B 样条优化 |
| `traj_server` | 对 B 样条按时间求值 |
| `so3_control` | EGO 原生演示控制器，不是项目阶段 6 的 PX4 桥 |
| `quadrotor_simulator_so3` | EGO 原生动力学模拟器，不是 Gazebo/PX4 SITL |
| `odom_visualization` | 显示模型和实际飞行路径 |
| `ego_mavros_bridge` | 项目阶段 6 的目标转发、控制权、安全状态机和 MAVROS setpoint 桥 |

### 15.4 阶段 6 关键 Topic

| Topic | 含义 |
|---|---|
| `/Odometry` | FAST-LIO 给规划器的当前位置、姿态和速度 |
| `/cloud_registered` | 当前环境点云，作为局部地图输入 |
| `/move_base_simple/goal` | RViz 中给出的原始目标 |
| `/planning/goal` | 经过桥接层校验和坐标变换后的目标 |
| `/waypoint_generator/waypoints` | 发给 EGO FSM 的目标航点 |
| `/grid_map/occupancy` | 原始占据地图可视化 |
| `/grid_map/occupancy_inflate` | 膨胀占据地图可视化 |
| `/planning/bspline` | EGO 输出的完整 B 样条轨迹 |
| `/planning/pos_cmd` | 当前时刻的规划参考状态 |
| `/mavros/setpoint_position/local` | 桥接层持续发给 PX4 的位置和 yaw 目标 |

## 16. RViz 显示与判断方法

| 显示名称 | 常见 Topic | 含义 |
|---|---|---|
| `goal_point` | `/ego_planner_node/goal_point` | 当前目标点 |
| `optimal_traj` | `/ego_planner_node/optimal_list` | 优化后的局部轨迹 |
| `global_path` | `/ego_planner_node/global_list` | 全局参考路线 |
| `AStar` | `/ego_planner_node/a_star_list` | 碰撞段的无碰撞引导路径 |
| `InitTraj` | `/ego_planner_node/init_list` | 优化前的初始轨迹 |
| `drone_path` | `/odom_visualization/path` 或实际位姿历史 | 无人机已经飞过的路径 |
| `simulation_map` | `/map_generator/global_cloud` | 原生演示的完整仿真障碍物 |
| `map inflate` | `/grid_map/occupancy_inflate` | 膨胀占据地图 |
| `real_map` | `/grid_map/occupancy` | 原始占据地图 |
| `robot` | `/odom_visualization/robot` | 无人机模型 |

RViz 的颜色没有统一含义，应先看 Display 名称和 Topic。`optimal_traj` 是计划怎样飞，`drone_path` 是实际上已经怎样飞；前者避障不代表后者一定跟得上。

## 17. EGO 常用检查命令与排查顺序

```bash
rosnode list
rostopic list | sort
rosnode info /ego_planner_node

rostopic echo -n 1 /Odometry
rostopic hz /Odometry
rostopic hz /cloud_registered

rostopic info /planning/bspline
rostopic hz /planning/pos_cmd
rostopic info /mavros/setpoint_position/local

rqt_graph
```

排查时按数据链从上游向下游走：

```text
里程计和点云是否新鲜、frame 是否一致
  -> 目标是否到达 /planning/goal 和 waypoints
  -> EGO 是否从 WAIT_TARGET 进入 GEN_NEW_TRAJ / EXEC_TRAJ
  -> /planning/bspline 是否在规划时更新
  -> /planning/pos_cmd 是否连续
  -> bridge 是否允许 TRACK_EGO、是否有 watchdog 报警
  -> MAVROS setpoint 是否只有 bridge 一个发布者
  -> PX4 是否 OFFBOARD/Armed
  -> 实际轨迹是否跟踪计划轨迹并与障碍保持距离
```

阶段 6 默认 `enable_control: false`，即 dry run：节点会检查和转换数据，但不会创建 MAVROS setpoint 发布者，也不会请求解锁或切换模式。只有坐标对齐、地图、输入新鲜度、控制权唯一性和故障行为都验证后，才能启用控制。

## 18. EGO 核心记忆

1. EGO-Planner 是局部轨迹规划器，不是飞控，也不是底层控制器。
2. 仓库 `Planner/ego-planner/` 就是 EGO 源码；`autoarming_control` 不是另一个同类避障 planner。
3. 占据地图判断有无障碍，膨胀地图为机体和误差留空间，ESDF 表示距离和梯度。
4. EGO 的 ESDF-free 指不维护完整 ESDF，不代表不建图或不检查碰撞。
5. B 样条控制点影响曲线形状，不一定是必经航点。
6. EGO 通过移动控制点，兼顾平滑、碰撞代价和动力学可行性。
7. `/planning/bspline` 是完整轨迹，`/planning/pos_cmd` 是当前时刻的参考指令。
8. `optimal_traj` 是计划轨迹，`drone_path` 是实际轨迹，两者必须同时检查。
9. EGO 负责“怎样绕障飞”，桥接层负责“何时允许把这条轨迹交给 PX4”。
10. 同一时刻只能有一个节点持续发布 MAVROS setpoint；地图源、规划坐标和 PX4 坐标必须经过验证。
