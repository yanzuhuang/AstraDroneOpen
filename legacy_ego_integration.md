# 旧0–6路线记录：EGO-Planner 的 Gazebo 工程落地

> 本文是旧0–6学习路线的历史记录，不代表当前新阶段编号；其中可执行资产均已改为按功能命名。

> 面向已经完成基础 Offboard 控制的新手。本阶段的最终产物是 **PX4 SITL + Gazebo + MAVROS + EGO-Planner** 的可复现闭环，只做电脑仿真，不接真机。
>
> EGO 自带 SO3 模拟器只用于快速证明规划源码能运行，不作为最终验收。最终必须由 PX4 控制 Gazebo 中的 `iris_mid360`，并在 Gazebo 障碍场景中完成规划、跟踪、避障和安全降落。

## 1. 开始前复习专用术语

先把这些概念翻译成工程语言。现阶段不要求推导公式，但要能说出每个模块的输入、输出和失效后果。

| 术语 | 直白解释 | 本阶段中的作用 |
|---|---|---|
| EGO-Planner | 根据目标、当前位置和障碍物生成局部平滑轨迹的规划器 | 决定“接下来应该怎样绕障碍飞” |
| global trajectory | 从起点到终点的总体参考路线 | 给局部规划提供前进方向，不直接控制无人机 |
| local B-spline | 当前一小段平滑可执行轨迹 | EGO 通过 `/planning/bspline` 发布并不断替换 |
| control point / knot | 控制 B-spline 形状和时间分段的数据 | `traj_server` 用它们还原连续轨迹 |
| trajectory server | 按当前时间对 B-spline 求值的节点 | 约 `100 Hz` 输出位置、速度、加速度和 yaw |
| `PositionCommand` | EGO 自带的轨迹指令消息 | `/planning/pos_cmd` 是规划侧与控制侧的接口 |
| occupancy grid / voxel | 三维占据栅格 / 其中一个小方格 | EGO 用它判断空间是否被障碍占用 |
| resolution | 每个栅格的边长 | 越小越精细，但内存和计算量增长很快 |
| inflation | 在地图中把障碍向外加粗 | 给真实尺寸的机体和定位误差留余量 |
| `dist0` | 轨迹优化希望保持的避障距离尺度 | 越大通常越保守，窄通道也越容易无解 |
| planning horizon | 每次局部规划向前看的距离 | 必须与点云有效距离和飞行速度匹配 |
| feasibility | 轨迹是否满足速度、加速度等限制 | 不可行轨迹即使几何上不碰撞也不能执行 |
| FSM | 把任务拆成等待、规划、执行、重规划、急停等状态 | 决定何时生成新轨迹、何时停止 |
| replan | 飞行中重新生成后续轨迹 | 新障碍出现或轨迹执行一段后更新路线 |
| odometry / odom | 位置、姿态和速度估计 | EGO 和控制桥接都必须使用时效正常的 odom |
| point cloud | 三维点集合 | 向 EGO 表示 Gazebo 传感器看到的障碍物 |
| frame | 一组坐标轴和原点 | odom、点云、目标和指令必须能转换到同一坐标系 |
| ENU / FLU | 东北天世界系 / 前左上机体系 | ROS/MAVROS 常见坐标约定，不能与 NED/FRD 混用 |
| TF | ROS 管理坐标系变换的工具 | 把规划坐标转换成 MAVROS 局部坐标 |
| bridge / adapter | 在两套接口之间转换消息和坐标的节点 | 把 `PositionCommand` 变成 MAVROS setpoint |
| interface contract | 对话题、消息类型、坐标系、频率和超时的明确约定 | 防止“话题有数据但含义不一致” |
| setpoint | 发送给 PX4 的目标位置、速度或姿态 | 本阶段第一版发送位置和 yaw |
| prestream | 进入 OFFBOARD 前预先连续发送目标 | PX4 接受 OFFBOARD 的必要步骤 |
| watchdog | 检查消息是否超时的“看门狗” | 规划、odom 或点云中断时禁止继续盲飞 |
| command authority | 当前谁拥有控制输出权 | 任意时刻只能有一个节点发布 MAVROS setpoint |
| dry run | 只检查数据和输出，不解锁、不让飞机运动 | 在真正闭环前验证接口、坐标和方向 |
| fail-safe | 故障后的确定性安全处理 | 短时保持，持续故障则请求 `AUTO.LAND` |
| RMSE / P95 | 均方根误差 / 95% 分位误差 | 量化轨迹跟踪效果，而不是只看动画 |
| regression test | 每次修改后重复固定场景测试 | 防止修好一个问题却破坏已有功能 |
| `CATKIN_IGNORE` | 让 catkin 完全忽略某个包的标记 | 当前 EGO 包默认不会被工作空间编译 |

最重要的一句话：**EGO 只负责生成轨迹；PX4 才负责控制 Gazebo 无人机。两者之间必须有经过坐标、时序和失效保护设计的桥接层。**

## 2. 最终目标和仓库现状

### 2.1 最终工程闭环

```text
Gazebo 障碍场景与 iris_mid360
        ↓ LiDAR/IMU
定位与点云：/Odometry、/cloud_registered
        ↓
EGO-Planner：地图、FSM、局部 B-spline、重规划
        ↓ /planning/pos_cmd
ego_mavros_bridge：坐标转换、限幅、watchdog、任务状态机
        ↓ /mavros/setpoint_position/local
MAVROS → PX4 OFFBOARD → Gazebo 无人机
        ↑ /mavros/state、/mavros/local_position/*

RViz /move_base_simple/goal
        ↓ bridge 检查状态、odom、点云和 TF
    /planning/goal → waypoint_generator → EGO
```

### 2.2 当前仓库已经有什么

- EGO 规划、B-spline 优化、地图、FSM 和 `traj_server` 源码；
- EGO 自带 mock map、SO3 控制器和动力学模拟器；
- PX4 SITL、Gazebo、MAVROS、`iris_mid360`、FAST-LIO 仿真输入；
- 已验证的基础 Offboard 起飞、悬停、轨迹、返航和降落逻辑；
- 本阶段新增的 `ego_gazebo_bridge`、Gazebo 专用 launch、保守参数和单元测试。

### 2.3 本阶段已经落地的文件

不要再次执行 `catkin_create_pkg`，下面的包已经创建完成：

```text
AstraDrone_ros1_ws/src/MissionControl/ego_gazebo_bridge/
├── CMakeLists.txt
├── package.xml
├── config/ego_gazebo_bridge.yaml
├── launch/ego_gazebo_bridge.launch
├── include/ego_gazebo_bridge/
│   ├── command_utils.h
│   └── ego_mavros_bridge.h
├── src/
│   ├── command_utils.cpp
│   ├── ego_mavros_bridge.cpp
│   └── ego_mavros_bridge_node.cpp
└── test/command_utils_test.cpp
```

EGO 中影响集成的 `world`、`map` 和手动目标高度也已参数化。注意：源码已具备转换能力，不代表 `map` 与 `camera_init` 天然重合；实际 TF 仍必须通过 G2 验证。

## 3. 用六道工程门逐步完成

| 工程门 | 要解决的问题 | 通过条件 |
|---|---|---|
| G0：源码可构建 | EGO 默认被忽略、依赖是否完整 | 指定包稳定编译，启动文件可解析 |
| G1：规划器冒烟 | 先排除算法包本身的问题 | 自带模拟器中目标、B-spline、pos_cmd 正常 |
| G2：Gazebo 数据面 | Gazebo 的 odom、点云和坐标是否能供 EGO 使用 | 不解锁时 EGO 能生成合理轨迹 |
| G3：控制桥接 | `PositionCommand` 如何安全交给 PX4 | 单元测试和 dry run 通过，无实际起飞 |
| G4：空场闭环 | PX4 能否稳定跟踪低速 EGO 轨迹 | 完整起飞、目标、返航、降落，无模式丢失 |
| G5：障碍与故障 | 地图是否对应物理障碍，故障是否安全 | 森林避障成功，断流时保持或降落 |

必须按顺序通过。G2 没通过时调 `lambda_collision`，或者 G3 没有 watchdog 时直接起飞，都是典型的工程顺序错误。

## 4. G0：启用并编译 EGO

### 4.1 记录现状

```bash
cd ~/AstraDroneOpen
find AstraDrone_ros1_ws/src/Planner/ego-planner -name CATKIN_IGNORE -print | sort
git status --short
```

不要使用 `git clean`、`git reset` 或删除整个 `build/devel`，避免破坏前面阶段的修改。

### 4.2 启用自带仿真所需的最小包

进入 EGO 根目录，将下面 15 个包的 `CATKIN_IGNORE` 改名为 `CATKIN_IGNORE.disabled`：

```text
planner/{plan_env,path_searching,bspline_opt,traj_utils,plan_manage}
uav_simulator/Utils/{cmake_utils,pose_utils,quadrotor_msgs,uav_utils,
                     waypoint_generator,odom_visualization}
uav_simulator/{local_sensing,mockamap,so3_control,so3_quadrotor_simulator}
```

可以逐个手动改名；改完用下面命令确认数量，避免直接执行范围过大的删除命令：

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws/src/Planner/ego-planner
find . -name CATKIN_IGNORE.disabled -print | sort
find . -name CATKIN_IGNORE.disabled -print | wc -l
```

第二条应输出 `15`。需要恢复忽略状态时，将这 15 个文件改回 `CATKIN_IGNORE`，不要删除源码目录。

先不启用 `map_generator`、`multi_map_server`、`rviz_plugins`，最终 Gazebo 闭环也不依赖它们。

检查依赖并编译：

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
source /opt/ros/noetic/setup.bash
rosdep check --from-paths src/Planner/ego-planner --ignore-src

catkin_make -j2 -DCATKIN_WHITELIST_PACKAGES="cmake_utils;pose_utils;quadrotor_msgs;uav_utils;waypoint_generator;odom_visualization;local_sensing_node;mockamap;so3_control;so3_quadrotor_simulator;plan_env;path_searching;bspline_opt;traj_utils;ego_planner"
source devel/setup.bash
rospack find ego_planner
roslaunch --nodes ego_planner run_in_sim.launch
```

若编译失败，只处理日志中的**第一条真实错误**。不要同时改多个 CMakeLists，也不要看到大量连锁错误就逐条安装无关软件。

### 4.3 已完成的源码可靠性修复

本次已经完成，不需要你重复编辑：

- FSM 标志显式初始化；
- 拒绝空目标、NaN/Inf odom、非法飞行类型和越界 waypoint 数量；
- 手动目标高度改为 `fsm/manual_target_height`；
- 规划、可视化、目标和 `PositionCommand` 统一读取 frame 参数；
- 修复 B-spline 障碍分段索引可能未初始化的问题。

## 5. G1：EGO 自带模拟器只做冒烟测试

这一关控制在半天以内，不在自带模拟器中花数天调参数。

终端 1：

```bash
source ~/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash
roslaunch ego_planner run_in_sim.launch
```

终端 2：

```bash
source ~/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash
roslaunch ego_planner rviz.launch
```

在 RViz 用 2D Nav Goal 发送两个目标，并检查：

```bash
rostopic hz /visual_slam/odom
rostopic echo -n 1 /planning/bspline
rostopic hz /planning/pos_cmd
rostopic info /planning/pos_cmd
```

通过标准：odom 连续，目标触发 `GEN_NEW_TRAJ -> EXEC_TRAJ`，B-spline 的 `traj_id` 会随重规划变化，`/planning/pos_cmd` 接近 `100 Hz`。注意 `/planning/bspline` 只在生成或重规划时发布，不是固定高频话题。

到这里即停止使用自带 SO3 模拟器。后面的最终验收不再接受 `/visual_slam/odom` 或 mock map。

## 6. G2：先接通 Gazebo 数据面，不允许解锁

### 6.1 启动真实工程底座

第一次使用空场，分终端启动，且不要启动任何 Offboard 控制节点：

```bash
# 终端 1：PX4 + Gazebo + MAVROS
roslaunch ~/AstraDroneOpen/simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch \
  world:=~/AstraDroneOpen/simulation/astra_gazebo_worlds/example.world
```

```bash
# 终端 2：当前仿真机型的定位与点云链
source ~/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash
roslaunch fast_lio mapping_mid360.launch rviz:=false
```

等待 `IMU Initial Done`，然后检查接口：

```bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/local_position/odom
rostopic echo -n 1 /Odometry
rostopic echo -n 1 /cloud_registered/header
rostopic hz /Odometry
rostopic hz /cloud_registered
```

这里把 FAST-LIO 当作现有工程接口使用，不要求在阶段 6 推导算法；但必须理解 `/Odometry` 和 `/cloud_registered` 均以 `camera_init` 为世界参考。

### 6.2 先写接口契约

在写桥接代码前，把实际检查结果填入表格。下面是推荐设计，不允许凭名字猜 frame：

| 数据 | 推荐话题 | 类型 | 期望 frame | 最低频率 | 超时策略 |
|---|---|---|---|---:|---|
| EGO odom | `/Odometry` | `nav_msgs/Odometry` | `camera_init -> body` | 10 Hz | 超过 0.2 s 禁止推进任务 |
| 障碍点云 | `/cloud_registered` | `sensor_msgs/PointCloud2` | `camera_init` | 5 Hz | 超过 0.5 s 禁止接收新目标 |
| 规划轨迹 | `/planning/bspline` | `ego_planner/Bspline` | 规划 frame | 事件触发 | 无新轨迹时保留现有有效轨迹 |
| 轨迹指令 | `/planning/pos_cmd` | `quadrotor_msgs/PositionCommand` | 规划 frame | 50 Hz | 超过 0.2 s 转 HOLD |
| PX4 状态 | `/mavros/state` | `mavros_msgs/State` | 无 | 2 Hz | 失联则禁止解锁/继续任务 |
| PX4 实际位姿 | `/mavros/local_position/pose` | `PoseStamped` | MAVROS local ENU | 10 Hz | 超过 0.2 s 请求安全处理 |
| PX4 setpoint | `/mavros/setpoint_position/local` | `PoseStamped` | MAVROS local ENU | 20 Hz | 桥接节点必须稳定发布 30～50 Hz |

### 6.3 建立 Gazebo 专用 EGO launch

新建通用集成入口 `ego_gazebo_bridge.launch`（原旧路线名 `stage6_gazebo.launch`），不要直接复用 `run_in_sim.launch` 后仍包含 `simulator.xml`。它只应启动：

```text
ego_planner_node
traj_server
waypoint_generator
rviz（可选）
ego_mavros_bridge（G3 完成后启用）
```

必须删除/不包含：

```text
mockamap_node
pcl_render_node
so3_control
quadrotor_simulator_so3
odom_visualization
```

关键 remap：

```text
EGO odom  <- /Odometry
EGO cloud <- /cloud_registered
```

不要同时给 EGO 接 mock map 和 Gazebo 点云，否则你无法判断规划器避让的是哪套障碍。

### 6.4 统一规划坐标系

原始源码的 `traj_server.cpp`、`waypoint_generator.cpp` 和 `planning_visualization.cpp` 混有硬编码的 `world`、`map`；本次已统一改为参数。Gazebo launch 将 `planning_frame` 设为 `camera_init`，以下内容现在由同一个参数约束：

- grid map 发布 frame；
- 目标和可视化 frame；
- `PositionCommand.header.frame_id`；
- RViz Fixed Frame；
- 桥接节点的输入 frame。

手动目标高度也已从硬编码 `1.0` 改为 `fsm/manual_target_height`，第一次保持 `1.0 m`。不要改为直接使用 RViz 2D Nav Goal 的 z，因为标准 2D 目标通常给出 `z=0`。

执行：

```bash
rosrun tf tf_echo map camera_init
rosrun tf view_frames
```

如果采用现有的单位变换 `map -> camera_init`，必须先用静止和小范围人工 setpoint 证明两个坐标原点、轴方向和 yaw 确实一致。身份 TF 只能表达已经验证的事实，不能用来掩盖坐标差异。

### 6.5 Dry run 验收

桥接控制关闭、无人机保持上锁时启动 Gazebo 专用 launch：

```bash
source ~/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash
roslaunch ego_gazebo_bridge ego_gazebo_bridge.launch \
  enable_control:=false rviz:=true
```

发送一个地图内目标。必须看到 `/planning/bspline`、`/planning/pos_cmd` 和 `DRY_RUN`，但 MAVROS setpoint 话题没有该桥接发布者：

```bash
rostopic echo -n 1 /ego_mavros_bridge/state
rostopic info /mavros/setpoint_position/local
rostopic info /mavros/setpoint_raw/local
```

只有 odom、点云、目标和轨迹坐标都正确，才能进入 G3。

## 7. G3：实现 `ego_mavros_bridge`

### 7.1 第一版只做位置和 yaw

独立 ROS 包已经建好，不要把功能再复制进旧 `offboard` 节点。`command_utils` 只负责可单测的消息转换、TF、边界和变化率限制；`EgoMavrosBridge` 负责 ROS 接口与任务状态机；`ego_mavros_bridge_node.cpp` 只保留程序入口。这样后续增加速度前馈时不会把数学、通信和状态混在一起。

前面使用过 catkin 白名单，因此创建包后必须把 `ego_gazebo_bridge` 加进 `CATKIN_WHITELIST_PACKAGES`，或者清空白名单再编译。否则源码和 CMakeLists 都正确，catkin 仍会跳过新包。

当前版本订阅 `/planning/pos_cmd`，把 position 和 yaw 转成合法的 `geometry_msgs/PoseStamped`，发布到 `/mavros/setpoint_position/local`。暂不转发速度、加速度前馈；位置闭环稳定后再将它作为独立增强项。

桥接节点必须通过 TF 把 `PositionCommand.header.frame_id` 转到 MAVROS local ENU frame。转换内容包括位置和 yaw，不能只改 `header.frame_id`。

### 7.2 一个节点拥有完整控制权

复用前面阶段已经验证的 Offboard 思路，但把它放进同一个桥接/任务监督节点：

```text
WAIT_FCU
  -> WAIT_INPUTS
  -> PRESTREAM
  -> ARM_OFFBOARD
  -> TAKEOFF
  -> HOVER_READY
  -> TRACK_EGO
  -> HOLD（故障恢复窗口）
  -> LANDING
  -> DONE
```

`return_home` 不直接生成穿越障碍的直线 setpoint，而是把“家点上方”重新作为 EGO 目标；到达后再请求 `AUTO.LAND`。

任何时刻只能由这个节点发布 MAVROS setpoint。运行它时不得启动 `autoarming_control`、`position_control` 或其他控制节点。

### 7.3 必须实现的保护

- `/use_sim_time` 必须为 `true`，否则拒绝自动解锁；
- 没有新鲜 FCU state、PX4 pose、EGO odom、点云和规划指令时不得进入 TRACK_EGO；
- `PositionCommand` 超过 `0.2 s`：冻结轨迹进度并保持当前安全位置；
- 指令持续丢失超过 `3 s`：请求 `AUTO.LAND`；
- PX4 位姿过期：停止接受规划目标，并进入降落策略；
- 点云或规划 odom 过期：拒绝把 `/move_base_simple/goal` 转发到 `/planning/goal`，并禁止继续轨迹接管；
- setpoint 高度、水平范围、最大位置变化率和最大 yaw 变化率必须限幅；
- 接管前将 EGO odom 变换到 MAVROS local frame，并与 `/mavros/local_position/pose` 比较；位置差建议小于 `0.25 m`、yaw 差小于 `15°`，否则拒绝进入 TRACK_EGO；
- OFFBOARD 丢失时保持目标并按限频策略重试，降落状态不得抢回 OFFBOARD；
- 只有 PX4 报告 landed 且 `armed=false` 才进入 DONE；
- 启动时检查 setpoint 话题已有发布者，发现其他控制源就拒绝运行。

建议第一版参数：

```yaml
publish_rate: 50.0
command_timeout: 0.20
planner_odom_timeout: 0.20
cloud_timeout: 0.50
land_after_loss: 3.0
takeoff_height: 1.0
max_position_rate: 0.5
max_yaw_rate: 0.75
max_horizontal_radius: 10.0
min_relative_height: 0.3
max_relative_height: 2.0
```

### 7.4 CMakeLists 和 package.xml 要解决什么

桥接包至少依赖：

```text
roscpp
std_msgs
geometry_msgs
nav_msgs
mavros_msgs
quadrotor_msgs
tf2
tf2_ros
tf2_geometry_msgs
sensor_msgs
std_srvs
```

`CMakeLists.txt` 将纯逻辑编译为库，再让节点和测试共同链接该库；`package.xml` 声明对应的构建和运行依赖。以后不要在节点文件里复制同一套转换公式。

编译时显式包含新包：

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make -j2 -DCATKIN_WHITELIST_PACKAGES="cmake_utils;pose_utils;quadrotor_msgs;uav_utils;waypoint_generator;plan_env;path_searching;bspline_opt;traj_utils;ego_planner;ego_gazebo_bridge"
source devel/setup.bash
rospack find ego_gazebo_bridge
roslaunch --nodes ego_gazebo_bridge ego_gazebo_bridge.launch
```

### 7.5 起飞前测试

当前自动化单元测试已覆盖：

- `PositionCommand` 到位置/yaw 的转换；
- `camera_init -> map` 坐标转换；
- NaN/Inf 指令拒绝；
- 高度、水平半径限制；
- 位置和最短 yaw 方向的变化率限制。

执行：

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make run_tests_ego_gazebo_bridge -j2
catkin_test_results build/test_results/ego_gazebo_bridge
```

cmd 超时、持续超时和第二发布者属于 ROS 状态交互，放在 G4/G5 的低高度故障注入中验收，不用伪造一个与真实 ROS master 行为不同的“单元测试”。

Dry run 使用 `enable_control:=false`：节点订阅和计算，但不创建 MAVROS setpoint 发布者，也不调用解锁/模式服务，只把转换结果发布到 `/ego_mavros_bridge/debug_setpoint`，状态发布到 `/ego_mavros_bridge/state`。将它与 EGO 输入逐项比较后才能允许控制。

## 8. G4：空场完成第一次 Gazebo 闭环

### 8.1 保守参数

第一次使用：

```text
max_vel = 0.5 m/s
max_acc = 1.0 m/s²
目标高度 = 1.0 m
目标距离 = 2～3 m
obstacles_inflation = 0.3 m（首次仿真的保守起点）
```

`0.3 m` 仍需结合 `iris_mid360` 的实际碰撞模型、定位误差和跟踪误差复核，不能把它当成所有机型通用常数。

### 8.2 起飞前硬检查

```bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /Odometry
rostopic echo -n 1 /cloud_registered/header
rostopic info /mavros/setpoint_position/local
rosparam get /use_sim_time
```

必须满足：FCU 已连接、数据无 NaN、时间戳持续更新、`use_sim_time=true`、setpoint 没有其他持续发布者。

### 8.3 按固定顺序运行

```text
1. roscore
2. PX4 + Gazebo + MAVROS
3. FAST-LIO，等待初始化完成
4. 启动 `ego_gazebo_bridge.launch enable_control:=true`（暂不发目标）
5. bridge 自动完成预发送、OFFBOARD、解锁、起飞、悬停
6. 确认 HOVER_READY 后发送 RViz 目标，再显式启用轨迹接管
7. 到达后通过服务请求返航或降落
8. armed=false 后再停止节点
```

终端命令如下：

下面的 `publish_identity_planning_tf:=true` 只允许在 G2 已证明 `map` 与 `camera_init` 同原点、同轴向、同 yaw 后使用；若不满足，应发布测得的真实 TF，不能开启这个开关。

```bash
roslaunch ego_gazebo_bridge ego_gazebo_bridge.launch \
  enable_control:=true rviz:=true \
  publish_identity_planning_tf:=true

rostopic echo /ego_mavros_bridge/state
# 看到 HOVER_READY，且已在 RViz 发送安全目标后：
rosservice call /ego_mavros_bridge/enable_tracking "data: true"

# 正常结束任务，二选一：
rosservice call /ego_mavros_bridge/return_home "{}"
rosservice call /ego_mavros_bridge/land "{}"
```

禁止使用不带 `--base-only` 的 `pc_example.sh`，因为它会启动旧控制节点。更推荐第一次仍分终端启动，便于定位哪个进程失败。

### 8.4 录制工程证据

```bash
mkdir -p ~/stage6_evidence
rosbag record -O ~/stage6_evidence/g4_empty_world.bag \
  /mavros/state \
  /mavros/extended_state \
  /mavros/local_position/pose \
  /mavros/local_position/odom \
  /mavros/setpoint_position/local \
  /Odometry \
  /move_base_simple/goal \
  /planning/goal \
  /planning/bspline \
  /planning/pos_cmd \
  /ego_mavros_bridge/state \
  /rosout
```

空场通过标准：

- 完成“起飞—两个目标—返航—AUTO.LAND—上锁”；
- 无意外 OFFBOARD 丢失，无 NaN，无第二 setpoint publisher；
- 低速位置跟踪 RMSE 建议先做到 `< 0.35 m`，P95 `< 0.60 m`；
- 指令停止时 watchdog 行为与设计一致；
- 相同任务连续成功 3 次，才进入障碍场景。

## 9. G5：Gazebo 障碍场景和故障验收

### 9.1 障碍必须来自 Gazebo

改用 `forest.world`，并确认 EGO 地图中的障碍与 Gazebo 画面、`/cloud_registered` 一致。最终实验不得启动 mock map；否则规划轨迹可能避开虚拟障碍，却撞上 Gazebo 中的真实模型。

按难度完成：

1. 单个目标，直线路径上没有障碍；
2. 单个目标，直线路径被树木阻挡，必须产生明显绕行；
3. 连续三个目标，每个目标间至少发生一次重规划；
4. 目标放入障碍或地图边界外，系统拒绝/规划失败且无人机保持安全；
5. 静态场景稳定后，再尝试仓库中的动态障碍场景。

### 9.2 故障注入

只在 Gazebo 低速、低高度下进行：

| 故障 | 期望行为 |
|---|---|
| 停止 EGO/traj_server | 0.2 s 内 HOLD，持续 3 s 后 AUTO.LAND |
| 点云停止更新 | 禁止新目标，降低风险或进入 HOLD/LAND |
| 发送占用区域目标 | 不产生危险新轨迹，保持上一安全状态 |
| 短时丢失 OFFBOARD | 冻结任务、保持 setpoint、限频恢复 |
| 出现第二 setpoint publisher | bridge 拒绝接管或立即报告严重错误 |
| 指令出现 NaN/超范围 | 丢弃指令，不向 MAVROS 转发 |

### 9.3 最终量化指标

至少记录：任务成功率、规划失败次数、重规划次数、目标/实际 RMSE、P95、最大速度、最小障碍距离、OFFBOARD 丢失次数、cmd/odom/cloud 最大间隔和落地结果。

推荐阶段验收：

- 空场任务连续 5 次成功；
- 森林固定任务连续 5 次至少 4 次成功，失败能定位到明确模块；
- 全程无 Gazebo 碰撞、无 NaN、无越界、无多个控制发布者；
- 规划命令、实际轨迹和地图截图能解释每一次绕行；
- 所有故障注入均进入设计的 HOLD/LAND 分支。

这些阈值是学习用推荐值，不是 EGO 官方性能声明。

## 10. 参数调试顺序

严格按下面顺序调，前一层不稳定就不动后一层：

1. **系统接口**：话题、消息类型、时间戳、frame、TF、唯一发布者；
2. **安全状态机**：prestream、OFFBOARD、超时、HOLD、LAND；
3. **控制跟踪**：先降 `max_vel/max_acc`，确认 PX4 能跟上位置 setpoint；
4. **地图可靠性**：分辨率、点云频率、有效范围、膨胀；
5. **规划行为**：planning horizon、`dist0`、重规划阈值；
6. **优化权重**：最后才调整 smooth/collision/feasibility/fitness。

| 参数 | 工程作用 | 错误表现 |
|---|---|---|
| `max_vel/max_acc` | 同时限制规划激进程度和控制难度 | 太大时实际机滞后，避障余量被吃掉 |
| `resolution` | 地图精度与计算量的主要权衡 | 太粗会漏掉细障碍，太细会卡顿 |
| `obstacles_inflation` | 机体和误差的基础安全包络 | 太小易擦碰，太大则狭窄区域无解 |
| `dist0` | 优化器开始强烈避障的距离 | 太小贴障，太大难以通过 |
| `planning_horizon` | 局部轨迹前视距离 | 超过可靠感知过多会对未知空间过度乐观 |
| `thresh_replan` | 离开局部轨迹起点多远后重规划 | 太小重规划过频，太大响应变慢 |

每次只改一个参数，使用同一 world、seed、起点和目标，保留 YAML/launch 差异与 rosbag。

## 11. 源码阅读顺序：只读工程关键路径

1. `run_in_sim.launch`、`advanced_param.xml`：理解原始节点和参数；
2. `ego_replan_fsm.cpp`：目标、FSM、重规划和碰撞检查；
3. `planner_manager.cpp`：全局参考、局部目标和 B-spline 生成；
4. `traj_server.cpp`：B-spline 如何变成 `PositionCommand`；
5. `grid_map.cpp`：odom/点云如何变成占据地图；
6. `autoarming_control.cpp`：复用成熟的 Offboard、安全状态机思路；
7. `command_utils.cpp`：消息转换、TF、边界和变化率限制；
8. `ego_mavros_bridge.cpp`：ROS watchdog、唯一控制权与任务状态机。

`bspline_optimizer.cpp` 只定位四类代价和限制的入口，暂时不逐行研究 L-BFGS。工程尚未闭环时，深入优化器不会帮助你解决 frame 错误或 setpoint 断流。

## 12. 常见故障定位

| 现象 | 优先检查 |
|---|---|
| `ego_planner` 找不到 | `CATKIN_IGNORE`、编译结果、新终端是否 source |
| 自带仿真正常，Gazebo 无轨迹 | Gazebo 专用 launch 的 odom/cloud remap 和 frame |
| 地图与 Gazebo 障碍错位 | odom 与点云是否同 frame、TF 是否真实、时间戳是否同步 |
| EGO 有 pos_cmd，但飞机不动 | bridge 状态、setpoint 发布者、OFFBOARD、armed |
| 飞行方向旋转 90°或轴反了 | ENU/NED、FLU/FRD 或 yaw 转换错误，立即停止调参 |
| 飞机跟踪滞后并靠近障碍 | 先降低速度/加速度，再增加实际安全余量 |
| 经常无解 | 目标占用、膨胀/`dist0` 过大、地图边界或点云缺失 |
| 节点退出后飞机继续盲飞 | watchdog/任务监督设计不完整，禁止继续障碍实验 |
| RViz 好看但 Gazebo 碰撞 | 地图源与物理 world 不一致，或只看规划轨迹没看实际轨迹 |

## 13. 最终交付清单

- [x] EGO 包已启用并可重复构建，15 个标记保留为 `CATKIN_IGNORE.disabled`；
- [ ] 自带模拟器冒烟通过，但最终 launch 不启动自带 SO3 模拟器和 mock map；
- [x] 已写明话题、类型、frame、频率和超时接口契约；
- [x] `planning_frame` 已参数化，odom、点云、目标、可视化和 pos_cmd 使用同一配置；
- [x] 已新增 `ego_gazebo_bridge` 包、配置、launch 和纯逻辑测试；
- [x] bridge 具备唯一发布者检查、prestream、OFFBOARD、解锁、HOLD 和 LANDING 状态；
- [x] cmd、odom、点云、FCU 和越界指令均有 watchdog/限幅；
- [ ] dry run、空场、森林、占用目标和断流故障逐级通过；
- [ ] 保存固定实验配置、rosbag、日志、轨迹误差和失败样例；
- [ ] 能一键启动最终 Gazebo 链，也能分终端定位任一模块故障；
- [ ] 能明确说明当前仍是仿真工程，未经真机安全设计和验证，绝不能直接部署实机。

完成标准不是“RViz 出现一条彩色曲线”，而是：**Gazebo 中的物理无人机能由 PX4 稳定跟踪 EGO 轨迹，规划地图与 Gazebo 障碍一致，任何关键输入中断都有可验证的安全结果。**
