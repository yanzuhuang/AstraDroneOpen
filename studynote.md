# AstraDrone 仿真基础学习笔记

## 1. 当前学习范围与顺序

现阶段先掌握默认的 PX4/Gazebo/MAVROS/Offboard 基础控制链，不急于修改 PX4 内环，也暂不把 FAST-LIO、EGO-Planner 接入控制。

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

FAST-LIO 虽由脚本启动，但默认控制器不订阅它的 `/Odometry`；EGO-Planner 当前也没有直接控制这架 PX4/Gazebo 无人机。

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

### `offboard/launch/autoarming_control.launch`

职责：启动控制节点、设置 `map -> camera_init` 静态 TF、集中配置实验参数，并按需启动 RViz；参数值改变通常不需要重新编译。

| 参数 | 当前情况 |
|---|---|
| `flight_mode` | 有效：`circle` 或 `square` |
| `target_laps` | 有效：目标圈数 |
| `hight` | 有效：起飞和轨迹高度；源码拼写如此 |
| `side_length` | 有效：方形边长 |
| `radius` | C++ 会读取，默认 `2.0`，但 launch 尚未显式配置 |
| `takeoff_height` | 当前无效：launch 有定义，C++ 没有读取 |
| `speed` | 当前无效：launch 有定义，C++ 没有读取和使用 |

新增参数时要同时完成：launch 定义参数，C++ 读取并真正使用参数。

### `offboard/src/autoarming_control.cpp`

职责：Offboard 会话、解锁、目标位置生成和任务状态机，是基础控制阶段最重要的修改文件。

当前流程：

```text
等待 FCU
  -> 以 20 Hz 预发送 100 个当前位置 setpoint
  -> 请求 OFFBOARD
  -> 请求 Arm
  -> TAKEOFF
  -> TRACKING（圆形或方形）
  -> LANDING
  -> Disarm/COMPLETED
```

适合修改：起飞点、悬停、航点、轨迹、yaw、状态转换、到达门限、超时和安全处理。

当前需逐步改进：等待首帧有效 pose、记录 `home_x/y/z`、设置合法四元数 `w=1`、增加 `HOVER`、使用 `takeoff_height/speed`、增加阶段超时并确认真正上锁。

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

## 8. 当前最合适的第一个改造任务

先在 `autoarming_control.cpp` 实现：

```text
等待有效位姿
  -> 记录起飞点
  -> 垂直起飞到相对高度
  -> 定点悬停 10 秒
  -> 垂直降落
  -> 确认上锁
```

完成稳定的起飞—悬停—降落后，再学习多航点、圆/方形/8 字轨迹、速度与 yaw，最后进入 FAST-LIO 和 EGO-Planner。
