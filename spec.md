# AstraDroneOpen 仿真自主控制开发说明

> 本文基于当前工作区 `/home/yanzu/AstraDroneOpen` 的源码、launch、config、scripts、world、sdf、rviz 和 README 静态阅读整理。重点只放在“仿真环境中让无人机按照你的设想运动起来”。真机移植只在最后作为后续注意事项简单提及。  
> 文中行号基于当前工作区文件的 `nl -ba` 输出；如果你后续改过代码，行号可能前后移动。  
> 未找到或无法从当前代码确认的内容，会明确写“未能从代码中确认”。没有 README 说明但可从代码判断的内容，会标注“根据代码推断”。

## 1. 仿真相关整体结构

### 1.1 根目录关键文件

| 路径 | 作用 |
|---|---|
| `README.md` | 项目总览和默认运行说明，提到 `./scripts/run_sh/pc_example.sh` 是 PC 端仿真示例入口。 |
| `docs/00-AstraDrone开发教程.md` | 项目架构和基础开发教程。 |
| `docs/01-安装脚本详解.md` | 安装、编译、启动脚本说明，其中说明 `pc_example.sh` 会启动 PX4/Gazebo、FAST-LIO、Offboard 控制。 |
| `docs/02-仿真源码详细介绍.md` | 仿真目录说明，覆盖 Gazebo world、models、PX4 SDF、仿真工作区等。 |
| `docs/03-Ros源码详细介绍.md` | ROS 源码模块说明；部分描述和当前文件状态不完全一致，实际开发应以源码为准。 |
| `order.md` | 你的本地操作记录。第 18、41 行提到 `catkin_make`；第 22 行是 `pc_example.sh`；第 47 行是单独重启 `autoarming_control.launch`；第 55-61 行提醒 Gazebo 只用 `Reset Model Poses`，不要点 `Reset World`。 |
| `spec.md` | 当前文档。 |

### 1.2 启动脚本

| 路径 | 关键行 | 作用 |
|---|---:|---|
| `scripts/run_sh/pc_example.sh` | 15-32 | 默认仿真启动脚本：开 tmux，依次启动 `roscore`、PX4/Gazebo/MAVROS、FAST-LIO、Offboard 控制、QGroundControl。 |
| `scripts/run_sh/echo.sh` | 13-22 | 调试脚本：查看 `/mavros/local_position/pose`、`/mavros/state`、`/mavros/setpoint_position/local`、`/mavros/setpoint_raw/local`。 |
| `scripts/run_sh/record.sh` | 未逐行展开 | 录包脚本，用于记录图像、点云、MAVROS 状态、TF、Livox、battery 等。根据代码推断，适合验证仿真数据流。 |

默认仿真入口在 `scripts/run_sh/pc_example.sh`：

```bash
./scripts/run_sh/pc_example.sh
```

它的实际启动链路是：

```text
pc_example.sh:15        roscore
pc_example.sh:19        roslaunch px4 astra_example.launch
pc_example.sh:24        roslaunch fast_lio mapping_mid360.launch rviz:=false
pc_example.sh:29        roslaunch offboard autoarming_control.launch
pc_example.sh:32        qgc
```

### 1.3 PX4/Gazebo/MAVROS 仿真入口

核心文件：

```text
simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch
```

关键行：

| 行号 | 内容 | 作用 |
|---:|---|---|
| 6-11 | `x/y/z/R/P/Y` | 无人机初始位姿参数。 |
| 13 | `est=ekf2` | PX4 估计器类型。 |
| 14 | `vehicle=iris_mid360` | 默认无人机模型名。 |
| 15 | `world=.../example.world` | 默认 Gazebo 世界。 |
| 16 | `sdf=.../$(arg vehicle).sdf` | 按模型名加载 SDF。 |
| 25 | `fcu_url=udp://:14540@localhost:14557` | MAVROS 连接 PX4 SITL 的 UDP 地址。 |
| 30-46 | include `px4/launch/posix_sitl.launch` | 启动 PX4 SITL、Gazebo，并生成无人机模型。 |
| 48-53 | include `mavros/launch/px4.launch` | 启动 MAVROS。 |

如果你只想换 world 或初始位置，优先改这个 launch。

### 1.4 无人机模型和传感器模型

默认模型：

```text
simulation/px4_sim_files/px4_iris_sdf/iris_mid360/iris_mid360.sdf
```

关键行：

| 行号 | 内容 | 作用 |
|---:|---|---|
| 2 | `<model name='iris_mid360'>` | 默认无人机模型名。 |
| 3-5 | include `model://iris_without_GPS` | PX4 Iris 基础机体。 |
| 7-21 | include 并固定 `mid360` | Livox Mid360 雷达安装在机体上。 |
| 39-54 | include 并固定 `D435i` | RealSense D435i 相机。 |
| 57-71 | include 并固定 `fpv_cam` | 前视相机。 |

基础机体 SDF：

```text
simulation/px4_sim_files/px4_iris_sdf/iris_without_GPS/iris_without_GPS.sdf
```

关键行：

| 行号 | 内容 | 作用 |
|---:|---|---|
| 3 | `<model name='iris_without_GPS'>` | 基础无人机模型名。 |
| 452-471 | `mavlink_interface` 插件 | PX4/Gazebo/MAVLink 接口。 |
| 458 | `mavlink_tcp_port=4560` | MAVLink TCP 端口。 |
| 459 | `mavlink_udp_port=14560` | MAVLink UDP 端口。 |
| 464 | `qgc_udp_port=14550` | QGroundControl 端口。 |
| 466 | `sdk_udp_port=14540` | MAVROS SDK 端口，对应 `astra_example.launch` 第 25 行。 |
| 470-471 | `send_odometry=1`、`enable_lockstep=1` | 发送里程计并启用 lockstep。 |

Mid360 模型：

```text
simulation/astra_gazebo_models/mid360/mid360.sdf
```

关键行：

| 行号 | 内容 | 作用 |
|---:|---|---|
| 35-77 | ray sensor + `liblivox_laser_simulation.so` | 生成 Livox 点云。 |
| 74 | `<ros_topic>livox/lidar</ros_topic>` | 点云话题，实际 ROS 中通常为 `/livox/lidar`。 |
| 80-113 | IMU sensor + `libgazebo_ros_imu_sensor.so` | 生成 Livox IMU。 |
| 104-105 | namespace `/livox`、topic `/livox/imu` | IMU 话题。 |

多机传感器模型也存在：

- `simulation/astra_gazebo_models/mid360_0/mid360_0.sdf`：第 74 行 `uav0/livox/lidar`，第 105 行 `uav0/livox/imu`。
- `simulation/astra_gazebo_models/mid360_1/mid360_1.sdf`：第 74 行 `uav1/livox/lidar`，第 105 行 `uav1/livox/imu`。
- `simulation/astra_gazebo_models/mid360_2/mid360_2.sdf`：第 74 行 `uav2/livox/lidar`，第 105 行 `uav2/livox/imu`。

### 1.5 Gazebo world 和场景

world 文件目录：

```text
simulation/astra_gazebo_worlds
```

当前能看到的主要 world：

```text
cangku.world
craic_2026.world
dynamic_avoidance.world
example.world
forest.world
generated/craic_2026_runtime.world
suv.world
test.world
```

默认 world：

```text
simulation/astra_gazebo_worlds/example.world
```

关键行：

| 行号 | 内容 | 作用 |
|---:|---|---|
| 1-2 | SDF world 开始 | world 名为 `default`。 |
| 21-68 | `ground_plane` | 地面。 |
| 69-76 | gravity、physics | 重力和仿真物理步长。 |
| 90-165 | `Oak_tree` | 一棵橡树模型。 |
| 166-201 | `water_tower` | 水塔模型。 |
| 202-277 | `Pine_Tree` | 一棵松树模型。 |

动态避障 world：

```text
simulation/astra_gazebo_worlds/dynamic_avoidance.world
```

关键行：

| 行号 | 内容 | 作用 |
|---:|---|---|
| 94-337 | 静态障碍物 | 树、房子等。 |
| 343-347 | `obstacle_cylinder_1` | 可移动圆柱。 |
| 350-354 | `obstacle_cylinder_2` | 可移动圆柱。 |
| 357-361 | `obstacle_cylinder_3` | 可移动圆柱。 |
| 364-368 | `obstacle_box_1` | 可移动方块。 |
| 371-375 | `obstacle_box_2` | 可移动方块。 |

动态障碍物控制器：

| 路径 | 关键行 | 作用 |
|---|---:|---|
| `simulation/sim_workspace/src/dynamic_obstacle_controller/launch/astra_dynamic_avoidance_static.launch` | 10-12、24-46 | 启动 PX4/Gazebo/MAVROS，并加载 `dynamic_avoidance.world`。 |
| `simulation/sim_workspace/src/dynamic_obstacle_controller/launch/astra_dynamic_avoidance_moving.launch` | 3-7 | 先 include 静态 launch，再启动障碍物控制节点。 |
| `simulation/sim_workspace/src/dynamic_obstacle_controller/config/obstacle_params.yaml` | 6-44 | 配置移动障碍物的运动方式、速度、中心、半径、航点。 |
| `simulation/sim_workspace/src/dynamic_obstacle_controller/src/obstacle_controller.py` | 10-18、26-47、49-79、81-113 | 通过 `/gazebo/set_model_state` 服务移动模型。 |

### 1.6 Offboard 自主控制模块

核心目录：

```text
AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard
```

关键文件：

| 路径 | 关键行 | 作用 |
|---|---:|---|
| `src/autoarming_control.cpp` | 82-273 | 默认自动解锁、起飞、轨迹飞行、降落控制器。 |
| `launch/autoarming_control.launch` | 11-24 | 单机自动飞行参数和 MAVROS 话题。 |
| `launch/autoarming_Mult.launch` | 7-33 | 三机控制节点和 `/uav0`、`/uav1`、`/uav2` MAVROS remap。 |
| `src/position_control_lib.cpp` | 15-52 | 简单目标点控制：订阅 `/drone_control/goal_position`，发布 MAVROS setpoint。 |
| `launch/position_control.launch` | 4-10 | 位置控制节点启动文件。 |
| `CMakeLists.txt` | 127-148 | 编译 `position_control` 和 `autoarming_control`。 |
| `rviz_config/drone_path.rviz` | 未逐行展开 | Offboard 轨迹可视化配置。 |

`autoarming_control.cpp` 的关键结构：

| 行号 | 内容 | 作用 |
|---:|---|---|
| 15-18 | 全局状态和高度变量 | 保存 MAVROS 状态、当前位置、目标高度 `hight`。 |
| 21-26 | `FlightPhase` | 飞行阶段：起飞、轨迹、降落、完成。 |
| 28-34 | `state_cb`、`pose_cb` | 接收 MAVROS 状态和当前位置。 |
| 36-40 | `calculate_distance` | 计算当前点到目标点距离。 |
| 43-56 | `get_square_position` | 方形轨迹生成。 |
| 58-63 | `get_circle_position` | 圆形轨迹生成。 |
| 65-80 | `Lock` | 发送 MAVLink command 上锁。 |
| 88-97 | 读取参数 | `flight_mode`、`hight`、`target_laps`、`side_length`、`radius`。 |
| 99-104 | 订阅/发布/服务 | 连接 MAVROS topic 和 service。 |
| 106 | `ros::Rate rate(20.0)` | 控制循环 20Hz。 |
| 117-125 | 预热 setpoint | 切 Offboard 前先发送 100 个 setpoint。 |
| 135-144 | 初始化飞行阶段和轨迹长度 | 根据圆/方形计算轨迹长度。 |
| 145-270 | 主循环 | 切 Offboard、解锁、起飞、轨迹飞行、降落。 |
| 192-204 | `TAKEOFF` | 飞到 `(0,0,hight)`。 |
| 205-252 | `TRACKING` | 生成圆形/方形目标点并发布。 |
| 253-266 | `LANDING` | 回到 `(0,0,initial_height)`，接近地面后上锁。 |

### 1.7 FAST-LIO 仿真 SLAM

默认示例会启动 FAST-LIO，但默认 Offboard 控制器不直接使用 FAST-LIO 的 `/Odometry`。

| 路径 | 关键行 | 作用 |
|---|---:|---|
| `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/launch/mapping_mid360.launch` | 6-15 | 加载 `mid360.yaml` 并启动 `fastlio_mapping`。 |
| `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/config/mid360.yaml` | 1-35 | Mid360 点云、IMU、外参、发布配置。 |
| `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/src/laserMapping.cpp` | 762-769 | 读取 `lid_topic`、`imu_topic` 等参数。 |
| `laserMapping.cpp` | 847-861 | 订阅 `/livox/lidar`、`/livox/imu`，发布 `/cloud_registered`、`/Odometry`、`/path` 等。 |
| `laserMapping.cpp` | 592-620 | `/Odometry` frame 为 `camera_init`，child frame 为 `body`，并发布 `camera_init -> body` TF。 |

`mid360.yaml` 关键参数：

```yaml
common:
    lid_topic:  "/livox/lidar"   # 第 2 行
    imu_topic:  "/livox/imu"     # 第 3 行
mapping:
    fov_degree: 360              # 第 18 行
    det_range: 100.0             # 第 19 行
    extrinsic_T: [ -0.011, -0.02329, 0.04412 ]  # 第 21 行
publish:
    dense_publish_en: true       # 第 29 行
```

### 1.8 EGO-Planner 仿真规划器

目录：

```text
AstraDrone_ros1_ws/src/Planner/ego-planner
```

注意：当前 EGO-Planner 多个子包存在 `CATKIN_IGNORE`，默认可能没有编译，包括 `plan_manage`、`bspline_opt`、`path_searching`、`plan_env`、`traj_utils`、`waypoint_generator`、`quadrotor_msgs`、`so3_control` 等。运行 EGO 前要先确认这些包是否已启用。

核心文件：

| 路径 | 关键行 | 作用 |
|---|---:|---|
| `planner/plan_manage/launch/run_in_sim.launch` | 35-43、71-98 | 手动目标点模式，启动 EGO 自带仿真和规划节点。 |
| `planner/plan_manage/launch/simple_run.launch` | 35-43、47-67、71-98 | 预设航点模式。 |
| `planner/plan_manage/launch/advanced_param.xml` | 40-45、48-129 | EGO 主节点、话题 remap、地图、速度、加速度、避障参数。 |
| `planner/plan_manage/src/ego_replan_fsm.cpp` | 7-55 | 初始化状态机、订阅 odom、发布 B-spline。 |
| `ego_replan_fsm.cpp` | 57-68 | 读取预设航点并生成全局轨迹。 |
| `ego_replan_fsm.cpp` | 109-120 | 手动目标点回调；第 119 行把目标高度硬编码为 `1.0`。 |
| `ego_replan_fsm.cpp` | 420-460 | 局部重规划成功后发布 `/planning/bspline`。 |
| `planner/plan_manage/src/traj_server.cpp` | 27-69、163-229、238-240 | 订阅 `planning/bspline`，采样轨迹并发布 `/position_cmd`。 |

重要结论：

```text
EGO-Planner 默认链路:
waypoint / preset points
  -> /planning/bspline
  -> traj_server
  -> /position_cmd，launch 中 remap 成 planning/pos_cmd
  -> EGO 自带 SO3 仿真控制
```

未能从代码中确认：当前仓库没有现成的节点把 `/planning/pos_cmd` 直接转换为 `/mavros/setpoint_position/local`。所以 EGO-Planner 默认不能直接驱动 PX4/Gazebo 的 `iris_mid360`，需要你新增 bridge 或改造控制节点。

### 1.9 README 缺失或为空的模块

这些模块的顶层说明文件当前是 0 行：

```text
AstraDrone_ros1_ws/src/Control/control_readme.md
AstraDrone_ros1_ws/src/MissionControl/missioncontrol_readme.md
AstraDrone_ros1_ws/src/Planner/planner_readme.md
AstraDrone_ros1_ws/src/SLAM/slam_readme.md
AstraDrone_ros1_ws/src/Swarm/swarm_readme.md
AstraDrone_ros1_ws/src/Exploration/exploration_readme.md
AstraDrone_ros1_ws/src/Track/track_readme.md
AstraDrone_ros1_ws/src/Land/land_readme.md
AstraDrone_ros1_ws/src/Utils/utils_readme.md
```

因此对这些目录的说明主要是“根据代码推断”。尤其是 `Swarm` 和 `Exploration`，当前没有看到可直接运行的集群控制/自主探索实现。

## 2. 仿真启动流程

### 2.1 默认一键启动

运行：

```bash
./scripts/run_sh/pc_example.sh
```

流程：

1. `scripts/run_sh/pc_example.sh:15` 启动 `roscore`。
2. `pc_example.sh:19` 启动 `roslaunch px4 astra_example.launch`。
3. `astra_example.launch:30-46` include PX4 的 `posix_sitl.launch`，启动 PX4 SITL、Gazebo，并按第 14-16 行加载 `iris_mid360` 和 `example.world`。
4. `astra_example.launch:48-53` include MAVROS 的 `px4.launch`，MAVROS 通过第 25 行 `fcu_url` 连接 PX4。
5. `pc_example.sh:24` 启动 FAST-LIO。`mapping_mid360.launch:6-15` 加载 Mid360 配置并启动 `laserMapping`。
6. `pc_example.sh:29` 启动 `offboard autoarming_control.launch`。
7. `autoarming_control.launch:11-18` 设置轨迹模式、高度、圈数等参数。
8. `autoarming_control.cpp:99-104` 连接 MAVROS 话题和服务。
9. `autoarming_control.cpp:117-125` 先发送 100 次当前位置 setpoint，满足 PX4 Offboard 前置条件。
10. `autoarming_control.cpp:145-163` 周期性尝试切换 `OFFBOARD` 和解锁。
11. `autoarming_control.cpp:192-204` 起飞到 `hight`。
12. `autoarming_control.cpp:205-252` 按圆形或方形轨迹发布目标点。
13. `autoarming_control.cpp:253-266` 降落并上锁。

核心控制数据流：

```text
autoarming_control.cpp
  -> /mavros/setpoint_position/local
  -> MAVROS
  -> PX4 OFFBOARD
  -> Gazebo 中的 iris_mid360 运动
```

状态反馈流：

```text
Gazebo/PX4
  -> MAVROS
  -> /mavros/local_position/pose
  -> autoarming_control.cpp
```

SLAM 数据流：

```text
Gazebo mid360.sdf
  -> /livox/lidar, /livox/imu
  -> FAST-LIO laserMapping
  -> /Odometry, /cloud_registered, /path
```

### 2.2 单独调试控制节点

按 `order.md` 的记录，修改 `autoarming_control.cpp` 或 launch 后常用流程是：

```bash
cd /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make
source devel/setup.bash
roslaunch offboard autoarming_control.launch
```

验证状态：

```bash
./scripts/run_sh/echo.sh
```

或手动查看：

```bash
rostopic echo /mavros/state
rostopic echo /mavros/local_position/pose
rostopic echo /mavros/setpoint_position/local
```

判断是否生效：

- `/mavros/state` 中 `mode` 应为 `OFFBOARD`。
- `/mavros/state` 中 `armed` 应为 `True`。
- `/mavros/setpoint_position/local` 应持续变化。
- Gazebo 中无人机应按目标高度和轨迹运动。

### 2.3 RViz/Gazebo 的关系

- Gazebo 是物理仿真环境，模型和 world 来自 `simulation`。
- RViz 是可视化工具，不负责物理仿真。
- `autoarming_control.launch:27-29` 会按 `rviz` 参数启动 RViz，并加载 `offboard/rviz_config/drone_path.rviz`。
- FAST-LIO 的 RViz 配置在 `FAST_LIO/rviz_cfg/loam_livox.rviz`，由 `mapping_mid360.launch:17-19` 控制。

如果只想快速验证飞控，不需要 RViz：

```bash
roslaunch offboard autoarming_control.launch rviz:=false
```

## 3. 需求一：控制单个无人机的速度和高度

### 3.1 控制飞行高度

最直接修改：

```text
AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/launch/autoarming_control.launch
```

相关行：

```xml
11    <param name="flight_mode" value="circle" />
14    <param name="target_laps" value="2"/>
15    <param name="hight" value="3.0"/>        <!-- 轨迹飞行高度 -->
16    <param name="takeoff_height" value="1.0"/> <!-- 起飞高度 -->
17    <param name="side_length" value="8.0"/>
18    <param name="speed" value="1.5"/>
```

源码读取位置：

```text
autoarming_control.cpp:88-97
```

当前代码：

```cpp
88    std::string flight_mode;
89    int target_laps;
90    double side_length;
91    double radius;

93    nh.param<std::string>("flight_mode", flight_mode, "square");
94    nh_private.param("hight", hight, 3.0);
95    nh_private.param("target_laps", target_laps, 1);
96    nh_private.param("side_length", side_length, 8.0);
97    nh_private.param("radius", radius, 2.0);
```

当前代码作用：

- 第 94 行读取私有参数 `~hight`，赋值给全局变量 `hight`。
- 第 192-204 行起飞阶段把目标高度设置为 `hight`。
- 第 213-215 行轨迹阶段也持续把目标 z 设置为 `hight`。

修改前：

```xml
<param name="hight" value="3.0"/>
```

修改后，例如飞到 5 米：

```xml
<param name="hight" value="5.0"/>
```

注意：

- 参数名是 `hight`，不是 `height`。这是当前代码的实际拼写。
- `autoarming_control.launch:16` 的 `takeoff_height` 当前没有被 `autoarming_control.cpp` 读取，所以改它不会改变默认自动飞行高度。

运行验证：

```bash
cd /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make
source devel/setup.bash
roslaunch offboard autoarming_control.launch
```

另开终端：

```bash
rostopic echo /mavros/setpoint_position/local/pose/position/z
rostopic echo /mavros/local_position/pose/pose/position/z
```

预期：

- setpoint z 接近你设置的 `hight`。
- 当前 z 最终接近这个高度。

### 3.2 控制飞行速度

当前 launch 有速度参数：

```xml
autoarming_control.launch:18
<param name="speed" value="1.5"/>
```

但源码 `autoarming_control.cpp:88-97` 没有读取 `speed`。因此当前只改 launch 第 18 行不会生效。

当前轨迹推进逻辑在：

```text
autoarming_control.cpp:219-233
```

当前代码：

```cpp
219    double d = calculate_distance(current_pose, target_pose);
220    if (d < 1.0) {
221        double advance = 1.0 - d;
222        double delta_t = advance / trajectory_length;
223        t_target += delta_t;
224        if (t_target >= 1.0) {
225            t_target -= 1.0;
226            completed_laps++;
```

当前代码作用：

- `t_target` 是轨迹进度，范围大致是 0 到 1。
- 只有当无人机离目标点距离 `d < 1.0` 时，才推进轨迹。
- 推进量由 `1.0 - d` 决定，不是由 launch 中的 `speed` 决定。

建议修改位置：

```text
autoarming_control.cpp:88-97
autoarming_control.cpp:106
autoarming_control.cpp:219-233
```

修改前：

```cpp
double side_length;
double radius;

nh_private.param("radius", radius, 2.0);
```

修改后：

```cpp
double side_length;
double radius;
double speed;

nh_private.param("radius", radius, 2.0);
nh_private.param("speed", speed, 1.5);
```

再把第 219-233 行附近改成按速度推进。简单版本：

```cpp
double d = calculate_distance(current_pose, target_pose);
if (d < 1.0) {
    double delta_t = speed / trajectory_length / 20.0;  // 第 106 行 rate 是 20Hz
    t_target += delta_t;
    if (t_target >= 1.0) {
        t_target -= 1.0;
        completed_laps++;
        ROS_INFO("[TRACK] Completed lap %d/%d", completed_laps, target_laps);
        if (completed_laps >= target_laps) {
            ROS_INFO("[PHASE] All laps done, switching to LANDING");
            flight_phase = FlightPhase::LANDING;
        }
    }
}
```

更稳的版本是使用真实时间差。可在 `double t_target = 0.0;` 后面增加：

```cpp
ros::Time last_track_time = ros::Time::now();
```

然后在轨迹推进处：

```cpp
double d = calculate_distance(current_pose, target_pose);
double dt = (ros::Time::now() - last_track_time).toSec();
last_track_time = ros::Time::now();

if (d < 1.0) {
    t_target += speed * dt / trajectory_length;
    if (t_target >= 1.0) {
        t_target -= 1.0;
        completed_laps++;
        ROS_INFO("[TRACK] Completed lap %d/%d", completed_laps, target_laps);
        if (completed_laps >= target_laps) {
            flight_phase = FlightPhase::LANDING;
        }
    }
}
```

运行验证：

1. 修改 `autoarming_control.launch:18`：

   ```xml
   <param name="speed" value="0.8"/>
   ```

2. 编译并运行：

   ```bash
   cd /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws
   catkin_make
   source devel/setup.bash
   roslaunch offboard autoarming_control.launch
   ```

3. 观察控制节点终端输出：

   ```text
   [TRACK] t=...
   ```

4. 把 `speed` 改成 `2.0` 再运行。预期 `t` 增长更快，完成一圈时间更短。

注意：

- 这里的 `speed` 是“参考轨迹点沿轨迹前进的速度”，不是 PX4 内部最大速度。
- 如果设得太大，无人机可能追不上 setpoint，会切角、滞后或震荡。
- 如果后续使用 EGO-Planner，速度应改 `simple_run.launch:35-36` 和 `advanced_param.xml:111-129`。

## 4. 需求二：控制飞行轨迹并规划路线

### 4.1 修改默认圆形/方形轨迹

默认轨迹函数在：

```text
autoarming_control.cpp:43-63
```

方形轨迹：

```cpp
43    std::pair<double, double> get_square_position(double t, double side_length) {
44        double perimeter = 4 * side_length;
45        double distance = t * perimeter;
...
56    }
```

圆形轨迹：

```cpp
58    std::pair<double, double> get_circle_position(double t, double radius) {
59        double angle = t * 2 * M_PI;
60        double x = radius * cos(angle);
61        double y = radius * sin(angle);
62        return {x, y};
63    }
```

轨迹选择在：

```text
autoarming_control.cpp:205-245
```

当前代码：

```cpp
206    std::pair<double, double> target_xy;
207    if (flight_mode == "square") {
208        target_xy = get_square_position(t_target, side_length);
209    } else {
210        target_xy = get_circle_position(t_target, radius);
211    }
```

launch 中设置模式：

```xml
autoarming_control.launch:11
<param name="flight_mode" value="circle" />
```

修改前：

```xml
<param name="flight_mode" value="circle" />
<param name="side_length" value="8.0"/>
```

修改后，改成方形并扩大边长：

```xml
<param name="flight_mode" value="square" />
<param name="side_length" value="12.0"/>
```

注意：`autoarming_control.launch` 当前没有设置 `radius`，但 `autoarming_control.cpp:97` 支持读取私有参数 `radius`。如果想调圆半径，建议在 node 内增加：

```xml
<param name="radius" value="4.0"/>
```

### 4.2 增加自己的轨迹函数

例如增加 8 字形轨迹。

修改文件：

```text
AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src/autoarming_control.cpp
```

建议插入位置：

```text
autoarming_control.cpp:58-63 后面
```

新增代码：

```cpp
std::pair<double, double> get_figure8_position(double t, double radius) {
    double angle = t * 2 * M_PI;
    double x = radius * std::sin(angle);
    double y = radius * std::sin(angle) * std::cos(angle);
    return {x, y};
}
```

修改轨迹选择位置：

```text
autoarming_control.cpp:207-211
```

修改前：

```cpp
if (flight_mode == "square") {
    target_xy = get_square_position(t_target, side_length);
} else {
    target_xy = get_circle_position(t_target, radius);
}
```

修改后：

```cpp
if (flight_mode == "square") {
    target_xy = get_square_position(t_target, side_length);
} else if (flight_mode == "figure8") {
    target_xy = get_figure8_position(t_target, radius);
} else {
    target_xy = get_circle_position(t_target, radius);
}
```

注意：同一个逻辑在第 235-245 行又重复计算了一次目标点，也要同步增加 `figure8` 分支，否则最终发布的目标点仍然只会是 square 或 circle。

launch 修改：

```xml
<param name="flight_mode" value="figure8" />
<param name="radius" value="3.0"/>
```

运行验证：

```bash
cd /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make
source devel/setup.bash
roslaunch offboard autoarming_control.launch
```

观察：

```bash
rostopic echo /mavros/setpoint_position/local/pose/position
```

预期 x/y 按 8 字形变化。

### 4.3 用航点控制路线

已有目标点控制接口：

```text
AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src/position_control_lib.cpp
```

关键行：

| 行号 | 内容 | 作用 |
|---:|---|---|
| 18 | 订阅 `/drone_control/goal_position` | 外部给目标点。 |
| 19 | 发布 `mavros/setpoint_position/local` | 转发给 MAVROS。 |
| 24-28 | 循环发布 `goal_position` | 持续发送 setpoint。 |
| 34-39 | `ReadParams()` | 读取 `/goal_init_x/y/z`，但当前未看到调用。 |
| 42-50 | `position_CallBack` | 收到目标点后更新 `goal_position`。 |

启动文件：

```text
offboard/launch/position_control.launch
```

关键行：

```xml
4    <rosparam param="goal_init_x">0.0</rosparam> 
5    <rosparam param="goal_init_y">0.0</rosparam> 
6    <rosparam param="goal_init_z">1.0</rosparam> 
9    <node pkg="offboard" type="position_control" name="position_control" output="screen">
```

未能从代码中确认：`position_control_lib.cpp:34-39` 的 `ReadParams()` 当前没有在构造函数或主循环中被调用，因此 `goal_init_x/y/z` 是否实际生效需要进一步检查或修改。

如果要使用这个接口，你可以写一个航点发布节点，向 `/drone_control/goal_position` 发布：

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

rospy.init_node("route_publisher")
pub = rospy.Publisher("/drone_control/goal_position", PoseStamped, queue_size=10)
rate = rospy.Rate(1)

route = [(0, 0, 3), (4, 0, 3), (4, 4, 3), (0, 4, 3), (0, 0, 3)]

for x, y, z in route:
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0
    for _ in range(5):
        pub.publish(msg)
        rate.sleep()
```

但要注意：`position_control` 本身不负责切换 Offboard 和解锁。你需要确认 PX4 已进入 Offboard，或把 Offboard/arming 逻辑合并进自己的节点。

### 4.4 用 EGO-Planner 规划路线

适合做避障路线规划，但当前不是默认 PX4/MAVROS 控制链。

预设航点文件：

```text
AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/launch/simple_run.launch
```

关键行：

```xml
35    <arg name="max_vel" value="2.0" />
36    <arg name="max_acc" value="3.0" />
43    <arg name="flight_type" value="2" />
47    <arg name="point_num" value="5" />
49-67 point0 到 point4 的 x/y/z
72-77 traj_server
79-84 waypoint_generator
87-98 EGO 自带 simulator 和 RViz
```

修改前：

```xml
<arg name="point0_x" value="-15.0" />
<arg name="point0_y" value="0.0" />
<arg name="point0_z" value="1.0" />
```

修改后，例如把第一个点改成 `(0,0,2)`：

```xml
<arg name="point0_x" value="0.0" />
<arg name="point0_y" value="0.0" />
<arg name="point0_z" value="2.0" />
```

规划参数文件：

```text
planner/plan_manage/launch/advanced_param.xml
```

关键行：

| 行号 | 参数 | 作用 |
|---:|---|---|
| 48 | `fsm/flight_type` | 1 手动目标，2 预设航点。 |
| 55-70 | `fsm/waypoint*` | 预设航点。 |
| 72-79 | `grid_map/*` | 地图大小、分辨率、障碍膨胀。 |
| 104-108 | `virtual_ceil_height`、`frame_id` | 虚拟高度上限和规划坐标系。 |
| 111-113 | `manager/max_vel/max_acc/max_jerk` | 规划速度、加速度、jerk 限制。 |
| 119-129 | `optimization/*`、`bspline/*` | 轨迹优化和 B-spline 限制。 |

手动目标高度硬编码位置：

```text
ego_replan_fsm.cpp:109-120
```

当前代码：

```cpp
119    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
```

如果你希望 RViz 目标点或外部目标点的 z 生效，可改为：

```cpp
end_pt_ << msg->poses[0].pose.position.x,
           msg->poses[0].pose.position.y,
           msg->poses[0].pose.position.z;
```

未能从代码中确认：当前没有看到 EGO 到 MAVROS 的现成桥接节点。`traj_server.cpp:238-240` 订阅 `planning/bspline` 并发布 `/position_cmd`，`simple_run.launch:72-77` 把它 remap 到 `planning/pos_cmd`，但它不是 `/mavros/setpoint_position/local`。

要让 EGO 规划结果控制 PX4 仿真，你需要新增 bridge：

```text
/planning/pos_cmd
  -> bridge 节点
  -> /mavros/setpoint_position/local 或 /mavros/setpoint_raw/local
```

新增 C++ bridge 时，还要改：

```text
offboard/CMakeLists.txt:127-148
```

仿照第 145-148 行的 `autoarming_control` 增加：

```cmake
add_executable(ego_to_mavros_bridge src/ego_to_mavros_bridge.cpp)
target_link_libraries(ego_to_mavros_bridge
  ${catkin_LIBRARIES}
)
```

运行验证 EGO 自带仿真：

```bash
cd /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make
source devel/setup.bash
roslaunch ego_planner simple_run.launch
```

如果提示找不到包，先检查是否存在 `CATKIN_IGNORE`：

```bash
find AstraDrone_ros1_ws/src/Planner/ego-planner -name CATKIN_IGNORE
```

## 5. 需求三：增加仿真中的无人机数量

### 5.1 当前已有的多机控制文件

控制 launch：

```text
AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/launch/autoarming_Mult.launch
```

关键行：

```xml
7     <param name="flight_mode" value="circle" />
9-16  第 1 个 autoarming_control，remap 到 /uav0/mavros/...
18-25 第 2 个 autoarming_control，remap 到 /uav1/mavros/...
27-34 第 3 个 autoarming_control，remap 到 /uav2/mavros/...
```

当前问题：

- 第 9、18、27 行三个节点都叫 `name="autoarming_control"`，多机启动时可能发生节点重名。
- 这个 launch 只启动控制节点，不启动多架 PX4/Gazebo/MAVROS。
- 未能从代码中确认：当前仓库没有提供完整可直接运行的 `multi_uav_mavros_sitl.launch`。`AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/README.md:25` 提到了它，但实际搜索当前项目只找到 README 中的引用。

建议先改节点名：

修改前：

```xml
<node pkg="offboard" type="autoarming_control" name="autoarming_control" output="screen">
```

修改后：

```xml
<node pkg="offboard" type="autoarming_control" name="autoarming_control_uav0" output="screen">
```

另外两个改成：

```xml
name="autoarming_control_uav1"
name="autoarming_control_uav2"
```

### 5.2 当前已有的多机模型和端口

已有多机模型：

```text
simulation/px4_sim_files/px4_iris_sdf/iris_mid360_0/iris_mid360_0.sdf
simulation/px4_sim_files/px4_iris_sdf/iris_mid360_1/iris_mid360_1.sdf
simulation/px4_sim_files/px4_iris_sdf/iris_mid360_2/iris_mid360_2.sdf
```

基础机体端口：

| 模型 | 关键行 | 端口 |
|---|---:|---|
| `iris_without_GPS_0.sdf` | 456-464 | TCP 4560、UDP 14560、QGC 14550、SDK 14540 |
| `iris_without_GPS_1.sdf` | 457-465 | TCP 4561、UDP 14561、QGC 14551、SDK 14541 |
| `iris_without_GPS_2.sdf` | 457-465 | TCP 4562、UDP 14562、QGC 14552、SDK 14542 |

多机 PX4 airframe 参数：

```text
simulation/px4_sim_files/px4_iris_params/1048_gazebo-classic_iris_mid360_0
simulation/px4_sim_files/px4_iris_params/1049_gazebo-classic_iris_mid360_1
simulation/px4_sim_files/px4_iris_params/1050_gazebo-classic_iris_mid360_2
```

这些文件第 10-14 行附近设置了 EKF2 外部视觉/no GPS 相关参数，例如：

```text
EKF2_EV_DELAY
EKF2_EV_CTRL
EKF2_HGT_REF
EKF2_GPS_CTRL
```

### 5.3 增加第 4 架无人机的修改思路

未能从代码中确认：当前没有现成的 `iris_mid360_3`、`iris_without_GPS_3` 和完整四机 launch。需要你自己补齐。

最少需要：

1. 复制 `iris_without_GPS_2` 为 `iris_without_GPS_3`，修改模型名和端口。

   示例：

   ```xml
   <model name='iris_without_GPS_3'>
   <mavlink_tcp_port>4563</mavlink_tcp_port>
   <mavlink_udp_port>14563</mavlink_udp_port>
   <qgc_udp_port>14553</qgc_udp_port>
   <sdk_udp_port>14543</sdk_udp_port>
   ```

2. 复制 `iris_mid360_2` 为 `iris_mid360_3`，把 include 改为：

   ```xml
   <uri>model://iris_without_GPS_3</uri>
   ```

3. 新增 `mid360_3` 或复用普通 `mid360`。如果要每架机独立 SLAM，建议新增 `uav3/livox/lidar` 和 `uav3/livox/imu`。

4. 新增 PX4 airframe 参数文件，例如：

   ```text
   simulation/px4_sim_files/px4_iris_params/1051_gazebo-classic_iris_mid360_3
   ```

5. 新增或扩展多机 PX4 launch，让 Gazebo 生成第 4 个模型、启动第 4 个 PX4 实例和第 4 个 MAVROS namespace。

6. 在 `autoarming_Mult.launch` 增加 `/uav3/mavros/...` 的控制节点。

控制节点示例：

```xml
<node pkg="offboard" type="autoarming_control" name="autoarming_control_uav3" output="screen">
    <param name="hight" value="8.0" />
    <remap from="/mavros/state" to="/uav3/mavros/state" />
    <remap from="/mavros/local_position/pose" to="/uav3/mavros/local_position/pose" />
    <remap from="/mavros/setpoint_position/local" to="/uav3/mavros/setpoint_position/local" />
    <remap from="/mavros/set_mode" to="/uav3/mavros/set_mode" />
    <remap from="/mavros/cmd/arming" to="/uav3/mavros/cmd/arming" />
</node>
```

运行验证：

```bash
rostopic list | grep /uav
rostopic echo /uav0/mavros/state
rostopic echo /uav1/mavros/state
rostopic echo /uav2/mavros/state
```

预期：

- 每架机都有独立 `/uavX/mavros/state`。
- 每架机都有独立 `/uavX/mavros/local_position/pose`。
- 每架机的 PX4/MAVROS 端口不冲突。

## 6. 需求四：操控集群无人机

### 6.1 当前项目的集群基础

当前 `AstraDrone_ros1_ws/src/Swarm/swarm_readme.md` 是空文件，未能从代码中确认已有完整集群控制实现。

当前可利用的基础是：

- `autoarming_Mult.launch:9-34`：三个控制节点，分别 remap 到 `/uav0`、`/uav1`、`/uav2`。
- `autoarming_control.cpp:205-245`：每个控制节点独立生成轨迹并发布 setpoint。
- 多机 SDF 和端口变体：`iris_mid360_0/1/2`、`iris_without_GPS_0/1/2`。

### 6.2 最简单集群控制：同轨迹，不同高度

当前 `autoarming_Mult.launch` 已经这么做：

```xml
10    <param name="hight" value="5.0" />
19    <param name="hight" value="6.0" />
28    <param name="hight" value="7.0" />
```

作用：

- 三架机走同一 XY 圆形轨迹。
- 用不同 z 高度分层，降低碰撞概率。

问题：

- XY 轨迹完全重合，不是安全的真实编队控制。
- 所有节点名相同，需要先改唯一节点名。

验证：

```bash
rostopic echo /uav0/mavros/setpoint_position/local
rostopic echo /uav1/mavros/setpoint_position/local
rostopic echo /uav2/mavros/setpoint_position/local
```

预期：

- x/y 类似。
- z 分别是 5、6、7。

### 6.3 推荐改造：中心偏移和相位偏移

修改文件：

```text
offboard/src/autoarming_control.cpp
offboard/launch/autoarming_Mult.launch
```

建议在 `autoarming_control.cpp:88-97` 增加参数：

```cpp
double center_x, center_y, phase_offset;
nh_private.param("center_x", center_x, 0.0);
nh_private.param("center_y", center_y, 0.0);
nh_private.param("phase_offset", phase_offset, 0.0);
```

在 `autoarming_control.cpp:205-211` 使用相位：

```cpp
double t_eval = std::fmod(t_target + phase_offset, 1.0);

if (flight_mode == "square") {
    target_xy = get_square_position(t_eval, side_length);
} else {
    target_xy = get_circle_position(t_eval, radius);
}
```

在 `autoarming_control.cpp:213-215` 使用中心偏移：

```cpp
target_pose.pose.position.x = center_x + target_xy.first;
target_pose.pose.position.y = center_y + target_xy.second;
target_pose.pose.position.z = hight;
```

launch 示例：

```xml
<param name="center_x" value="0.0"/>
<param name="center_y" value="0.0"/>
<param name="phase_offset" value="0.0"/>
```

```xml
<param name="center_x" value="6.0"/>
<param name="center_y" value="0.0"/>
<param name="phase_offset" value="0.33"/>
```

```xml
<param name="center_x" value="-6.0"/>
<param name="center_y" value="0.0"/>
<param name="phase_offset" value="0.66"/>
```

注意：第 235-245 行重复计算目标点，也要同步使用 `t_eval`、`center_x`、`center_y`，否则最终发布点可能又回到未偏移轨迹。

运行验证：

```bash
rostopic echo /uav0/mavros/setpoint_position/local/pose/position
rostopic echo /uav1/mavros/setpoint_position/local/pose/position
rostopic echo /uav2/mavros/setpoint_position/local/pose/position
```

预期：

- 三架机 setpoint 的 x/y 不再完全重合。
- 同一时刻相位错开。

### 6.4 更完整的编队控制

根据代码推断，当前项目没有中心化 swarm controller。你可以新增一个节点，例如：

```text
offboard/src/swarm_formation_control.cpp
offboard/launch/swarm_formation_control.launch
```

节点逻辑：

```text
订阅 /uav0/mavros/local_position/pose
订阅 /uav1/mavros/local_position/pose
订阅 /uav2/mavros/local_position/pose

生成 leader 轨迹
为每架机加 formation offset

发布 /uav0/mavros/setpoint_position/local
发布 /uav1/mavros/setpoint_position/local
发布 /uav2/mavros/setpoint_position/local
```

新增 C++ 节点后，还要修改：

```text
offboard/CMakeLists.txt:127-148
```

仿照：

```cmake
add_executable(autoarming_control src/autoarming_control.cpp)
target_link_libraries(autoarming_control
  ${catkin_LIBRARIES}
)
```

增加：

```cmake
add_executable(swarm_formation_control src/swarm_formation_control.cpp)
target_link_libraries(swarm_formation_control
  ${catkin_LIBRARIES}
)
```

集群注意事项：

- 每架机必须持续收到自己的 setpoint，否则 PX4 会退出 Offboard。
- 每架机必须独立 arming、set_mode。
- 端口、命名空间、模型名、传感器话题都要唯一。
- 当前没有看到多机互相避障实现，初期应使用高度差、中心偏移、相位偏移保证安全。

## 7. 需求五：改变仿真世界、地图、障碍物或场景

### 7.1 切换 world

默认 world 在：

```text
simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch:15
```

当前：

```xml
<arg name="world" default="$(find env_map)../../../astra_gazebo_worlds/example.world"/>
```

修改为森林：

```xml
<arg name="world" default="$(find env_map)../../../astra_gazebo_worlds/forest.world"/>
```

或启动时传参：

```bash
roslaunch px4 astra_example.launch world:=/home/yanzu/AstraDroneOpen/simulation/astra_gazebo_worlds/forest.world
```

验证：

- Gazebo 加载的模型应变成对应 world 的场景。
- 终端没有 `Unable to find uri model://...`。
- 无人机仍能起飞并发布 `/mavros/local_position/pose`。

### 7.2 修改静态障碍物

直接改 world 文件，例如：

```text
simulation/astra_gazebo_worlds/example.world
```

可参考第 90-277 行已有模型写法。新增一个模型示例：

```xml
<include>
  <uri>model://box_target_red</uri>
  <name>my_red_box</name>
  <pose>3 2 0.5 0 0 0</pose>
</include>
```

模型来源：

```text
simulation/astra_gazebo_models/box_target_red
```

根据代码推断，`simulation/astra_gazebo_models` 是项目自己的 Gazebo 模型库；如果 world 中写 `model://xxx`，Gazebo 必须能在 model path 中找到对应目录。

### 7.3 修改动态障碍物

启动文件：

```text
simulation/sim_workspace/src/dynamic_obstacle_controller/launch/astra_dynamic_avoidance_moving.launch
```

关键行：

```xml
3    <include file="$(find dynamic_obstacle_controller)/launch/astra_dynamic_avoidance_static.launch"/>
5-7  启动 obstacle_controller.py 并加载 obstacle_params.yaml
```

world 中动态障碍物名字必须和 yaml 对上：

```text
dynamic_avoidance.world:343-375
```

yaml：

```text
dynamic_obstacle_controller/config/obstacle_params.yaml:6-44
```

修改前：

```yaml
10  - name: "obstacle_cylinder_1"
11    type: "linear"
12    axis: [1, 0, 0]
13    amplitude: 3.0
14    speed: 0.8
15    center: [0, 6, 2]
```

修改后，例如让它运动更慢、幅度更大：

```yaml
- name: "obstacle_cylinder_1"
  type: "linear"
  axis: [1, 0, 0]
  amplitude: 5.0
  speed: 0.4
  center: [0, 6, 2]
```

控制器源码：

```text
obstacle_controller.py:10-18     初始化节点、读取参数、连接 /gazebo/set_model_state
obstacle_controller.py:26-47     linear/circle 轨迹计算
obstacle_controller.py:49-79     waypoint 轨迹计算
obstacle_controller.py:81-113    设置模型位姿并循环更新
```

运行验证：

```bash
roslaunch dynamic_obstacle_controller astra_dynamic_avoidance_moving.launch
```

另开终端：

```bash
rosservice list | grep set_model_state
rostopic echo /gazebo/model_states
```

预期：

- Gazebo 中 `obstacle_cylinder_1` 等模型会移动。
- `/gazebo/model_states` 中对应模型 pose 持续变化。

### 7.4 改传感器仿真配置

如果你改了雷达话题或使用多机雷达，需要同步改 FAST-LIO 配置：

```text
AstraDrone_ros1_ws/src/SLAM/FAST_LIO/config/mid360.yaml:1-5
```

单机默认：

```yaml
lid_topic:  "/livox/lidar"
imu_topic:  "/livox/imu"
```

多机示例：

```yaml
lid_topic:  "/uav0/livox/lidar"
imu_topic:  "/uav0/livox/imu"
```

注意：多机时每架机都需要独立 FAST-LIO 节点名、参数命名空间、话题命名空间。当前默认 `mapping_mid360.launch:15` 节点名固定为 `laserMapping`，多机同时启动时需要改唯一名字或使用 namespace。

### 7.5 改模型传感器挂载位置

默认无人机传感器挂载在：

```text
simulation/px4_sim_files/px4_iris_sdf/iris_mid360/iris_mid360.sdf
```

例如 Mid360 位姿：

```xml
7    <include>
8      <uri>model://mid360</uri>
9      <pose>0 0 0.08 0 0 0</pose>
10   </include>
```

如果改了第 9 行的雷达安装位姿，可能还要同步改：

```text
AstraDrone_ros1_ws/src/SLAM/FAST_LIO/config/mid360.yaml:20-24
```

尤其是：

```yaml
extrinsic_T: [ -0.011, -0.02329, 0.04412 ]
extrinsic_R: [ 1, 0, 0,
               0, 1, 0,
               0, 0, 1]
```

否则仿真雷达和 SLAM 外参不一致。

## 8. 无法精确确认的内容和建议检查命令

### 8.1 多机 PX4 launch

未能从代码中确认：当前仓库没有找到完整可直接运行的多机 PX4/MAVROS launch。README 里提到：

```text
AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/README.md:25
roslaunch px4 multi_uav_mavros_sitl.launch
```

但当前项目内搜索只看到 README 引用，没有看到这个 launch 文件。

建议检查：

```bash
find /home/yanzu/AstraDroneOpen -name '*multi*launch*' -o -name 'multi_uav_mavros_sitl.launch'
```

### 8.2 EGO 到 MAVROS 的 bridge

未能从代码中确认：当前没有看到 `/planning/pos_cmd` 到 `/mavros/setpoint_position/local` 的桥接节点。

建议检查：

```bash
rg -n "planning/pos_cmd|PositionCommand|setpoint_position/local|setpoint_raw/local" AstraDrone_ros1_ws/src
```

### 8.3 默认被忽略的 ROS 包

很多包有 `CATKIN_IGNORE`，包括 EGO-Planner、rc_obstacle_avoidance、pix_tracker、Land 等。运行前检查：

```bash
find AstraDrone_ros1_ws/src -name CATKIN_IGNORE
```

如果某个包没有被编译，`roslaunch` 会找不到包或节点。启用方法要结合项目脚本或手动移除对应 `CATKIN_IGNORE`，再重新 `catkin_make`。

### 8.4 空 README 模块

`Swarm`、`Exploration`、`Planner`、`SLAM` 等顶层 readme 当前为空，本文对这些模块的说明是“根据代码推断”。如果要深入用某个模块，应优先读它的 launch、src、config，而不是依赖顶层 README。

## 9. 仿真开发路线建议

### 第 1 阶段：先跑通默认仿真

目标：确认 Gazebo、PX4、MAVROS、Offboard 控制都能工作。

运行：

```bash
./scripts/run_sh/pc_example.sh
```

验证：

```bash
rostopic echo /mavros/state
rostopic echo /mavros/local_position/pose
rostopic echo /mavros/setpoint_position/local
```

你要看到：

- `/mavros/state` connected 为 true。
- 控制节点进入 OFFBOARD。
- 无人机解锁、起飞、绕圈、降落。

### 第 2 阶段：只改高度和基础参数

先不要改 C++。

修改：

```text
offboard/launch/autoarming_control.launch:11-18
```

建议顺序：

1. 改 `hight`。
2. 改 `target_laps`。
3. 改 `flight_mode`。
4. 增加或改 `radius`。
5. 改 `side_length`。

验证：

```bash
roslaunch offboard autoarming_control.launch
```

### 第 3 阶段：让 `speed` 真正生效

修改：

```text
offboard/src/autoarming_control.cpp:88-97
offboard/src/autoarming_control.cpp:219-233
```

目标：

- 增加 `speed` 参数读取。
- 用 `speed` 推进 `t_target`。

验证：

- `speed=0.5` 时轨迹慢。
- `speed=2.0` 时轨迹快。
- 无人机仍能稳定跟踪。

### 第 4 阶段：改轨迹

修改：

```text
offboard/src/autoarming_control.cpp:43-63
offboard/src/autoarming_control.cpp:205-245
```

建议先做：

1. 改圆半径。
2. 改方形边长。
3. 加 8 字形。
4. 加航点数组。

验证：

```bash
rostopic echo /mavros/setpoint_position/local/pose/position
```

确认 setpoint 的 x/y/z 和你设计的轨迹一致。

### 第 5 阶段：做路线规划

先用简单航点，再用 EGO-Planner。

简单航点：

```text
position_control_lib.cpp:18-28
```

EGO-Planner：

```text
simple_run.launch:35-67
advanced_param.xml:48-129
ego_replan_fsm.cpp:109-120
traj_server.cpp:238-240
```

注意：

- EGO 先在自带仿真中跑通。
- 再写 bridge 接 PX4/MAVROS。

### 第 6 阶段：增加多机

先不要一口气做集群算法。

顺序：

1. 确认有 `/uav0/mavros/state`、`/uav1/mavros/state`、`/uav2/mavros/state`。
2. 改 `autoarming_Mult.launch` 的节点名。
3. 分别设置不同 `hight`。
4. 再增加 `center_x`、`center_y`、`phase_offset`。
5. 最后考虑新增第 4 架机。

重点文件：

```text
offboard/launch/autoarming_Mult.launch:7-34
offboard/src/autoarming_control.cpp:88-97, 205-245
simulation/px4_sim_files/px4_iris_sdf/iris_mid360_0/1/2
simulation/px4_sim_files/px4_iris_sdf/iris_without_GPS_0/1/2
```

### 第 7 阶段：做集群控制

先用多个 `autoarming_control` 节点做分散控制，再考虑中心化编队节点。

建议实现顺序：

1. 同轨迹不同高度。
2. 同轨迹不同相位。
3. 不同中心点的圆/方形。
4. leader-follower 编队。
5. 加入互避逻辑。
6. 每架机接入独立规划器或中心化规划器。

### 第 8 阶段：改仿真场景和障碍物

先改静态 world，再改动态障碍物。

重点文件：

```text
simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch:15
simulation/astra_gazebo_worlds/example.world
simulation/astra_gazebo_worlds/dynamic_avoidance.world:343-375
simulation/sim_workspace/src/dynamic_obstacle_controller/config/obstacle_params.yaml:6-44
simulation/sim_workspace/src/dynamic_obstacle_controller/src/obstacle_controller.py:10-113
```

验证：

- Gazebo 场景正确加载。
- 障碍物出现在预期位置。
- 动态障碍物按 yaml 移动。
- 无人机仍能进入 OFFBOARD 并运动。

## 10. 后续真机移植的简要注意事项

现阶段你只关注仿真即可。后续如果要移植真机，需要额外关注：

- Offboard 控制的安全保护、遥控接管和失控保护。
- PX4 参数、EKF 高度源、外部视觉/SLAM 输入是否正确。
- 传感器真实话题和仿真话题是否一致。
- 坐标系 ENU/NED、`camera_init/body/map/world` 的 TF 是否正确。
- 串口、数传、QGroundControl、MAVROS 连接方式。
- 起飞、降落、上锁逻辑不能照搬仿真自动执行，必须加安全确认。

对当前阶段来说，最稳的路径是：先在仿真中把 `autoarming_control.cpp` 改熟，让无人机稳定按你的高度、速度、轨迹和场景运动起来，再逐步接入规划、多机和集群控制。
