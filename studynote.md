# AstraDrone 仿真、控制与 EGO-Planner 学习笔记

# 1. 当前学习范围与顺序

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

# 2. 把主控制链看成“领导关系”

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

默认的 `pc_example.sh + autoarming_control` 链路中，FAST-LIO 虽由脚本启动，但 `autoarming_control` 不订阅它的 `/Odometry`，EGO-Planner 也不会自动接管无人机。旧路线阶段 6 当时使用独立集成 launch；该入口现按功能命名为 `ego_gazebo_bridge.launch`，由 `ego_mavros_bridge` 把 FAST-LIO、EGO-Planner 与 PX4/Gazebo 安全地连接起来。

# 3. 主控制链术语

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

# 4. 外部软件与仓库文件的区别

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


# 5. 启动、换场景和编译

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


# 6. ego——planner参数修改
## 参数
参数	|作用	|类比	|调大后的影响
obstacles_inflation	|扩大占据地图中的障碍物，形成硬禁区	|红色墙体	|障碍更“粗”，但可能封死通道或覆盖航点
dist0	|在膨胀障碍物外围设置软避障距离	|黄色警戒带宽度	|更早开始绕障，但不保证一定保持该距离
lambda_collision	|决定进入警戒带后，避障代价有多强	|排斥力强度	|更重视避障，但过大可能导致轨迹弯曲、优化困难

### 参数修改文件
1./home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/config/stage3_low_altitude.yaml
2./home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/launch/stage3_low_altitude.launch
