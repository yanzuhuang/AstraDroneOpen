# 用 AstraDroneOpen 从零认识 ROS

> 入口定位更新（2026-09-16）：本文中的 pc_example.sh 与 echo.sh 已退役删除，record.sh 继续保留，不能作为当前正式任务默认入口。正常任务使用 [项目整理文档·运行说明](项目整理文档.md#runbook)；能力差异与迁移边界见 [项目整理文档·模块一](项目整理文档.md#模块一启动脚本与-run_sh-整理)。旧 pc_example 组合仅作历史说明，不再提供运行命令；其原默认行为包含旧自动控制。
> 四路 MAVROS 监视现使用 [mavros_monitor.sh](scripts/run_sh/tools/mavros_monitor.sh)（DIAGNOSTIC）；默认根 namespace，可选 `--namespace /uav1`、`/uav2`、`/uav3`。旧 `echo.sh` 已退役删除。

> 这是一份写给 ROS 零基础读者的项目导读。
>
> 快速路线：先读第 0 节和第 4 节，约 5 分钟；再回到第 1～3 节逐个理解术语。第 5～10 节解释本项目，第 11 节用于动手练习，其余内容可按需查阅。

---

## 0. 先用 30 秒看懂 ROS

一台机器人通常需要同时运行很多程序：

- 相机程序负责取得图像；
- 雷达程序负责取得点云（许多三维测量点的集合）；
- 定位程序负责计算“我在哪里”；
- 规划程序负责计算“应该往哪里走”；
- 控制程序负责把目标交给飞控；
- 可视化程序负责显示状态。

这些程序必须互相传数据，还要能统一启动、配置和调试。**ROS 就是用来组织这些机器人程序并帮助它们通信的一套软件框架。**

可以先把 ROS 想成一家快递公司：

```text
程序 A ──把数据装成规定格式──> ROS 通信系统 ──> 程序 B
          消息 message             话题 topic
```

在本项目中，一个真实例子是：

```text
Gazebo（仿真器）中的虚拟 IMU（惯性传感器）
        │ 发布 sensor_msgs/Imu 消息
        ▼
   /livox/imu 话题
        │ FAST-LIO 订阅
        ▼
 /laserMapping 节点计算无人机位姿
```

先记住四句话：

1. ROS 不是飞控，也不是 Gazebo；它主要负责程序组织、通信、配置和工具支持。
2. 节点是“正在运行的一个 ROS 程序”。
3. 话题是节点之间持续传数据的通道，消息是通道里每一份数据的格式。
4. 本项目当前使用 ROS 1 Noetic；Noetic 是 ROS 1 的一个发行版本名称。

### 一张图记住 ROS 的构成与关系

```text
磁盘上的工程

工作空间 workspace
└── 功能包 package
    ├── 源码 ──编译并运行──> 节点 node
    ├── launch            批量启动节点
    └── config            保存配置


程序运行以后

roscore
  │ 帮助节点互相找到
  ▼
节点 A ──话题 topic / 消息 message──> 节点 B
节点 A ──服务 service 请求与响应────> 节点 C

参数 parameter：给节点提供高度、阈值、开关等配置
TF：告诉所有节点不同坐标系怎样互相转换
rosbag：把话题数据录下来，以后回放
```

关系可以概括为：**工作空间包含功能包，功能包中的源码运行后成为节点；节点用话题和服务通信，Launch 负责启动，参数负责配置，TF 负责坐标关系。**

---

## 1. 为什么机器人项目需要 ROS

假设没有 ROS，开发者要自己解决这些问题：

- 相机程序怎样找到定位程序；
- C++ 程序怎样把数据交给 Python 程序；
- 点云、位姿、速度分别用什么数据结构；
- 多个程序按什么顺序启动；
- 怎样修改参数而不改源码；
- 怎样查看当前有哪些程序和数据；
- 怎样记录一次实验并在以后回放。

ROS 已经提供了这些通用能力：

| ROS 提供的能力 | 在本项目中的例子 |
|---|---|
| 程序之间通信 | `/autoarming_control` 和 `/mavros` 交换当前位置与目标位置 |
| 标准数据格式 | 位姿使用 `geometry_msgs/PoseStamped` |
| 统一启动 | Launch 文件一次启动多个节点并加载参数 |
| 参数管理 | `hight=3.0` 控制默认飞行高度 |
| 坐标变换 | TF 保存 `map`、`camera_init`、`body` 的关系 |
| 调试工具 | `rosnode`、`rostopic`、`rqt_graph`、RViz、rosbag |

ROS 不负责所有事情。例如，本项目中的 PX4 才是真正的飞控软件，Gazebo 负责模拟物理世界，QGroundControl 是地面站。ROS 把算法节点和这些系统连接起来。

---

## 2. ROS 由哪些部分组成

ROS 同时包含“磁盘上的工程文件”和“运行起来后的通信网络”。新手最容易把这两层混在一起。

### 2.1 文件层：工作空间、功能包、源码和配置

```text
工作空间 workspace
└── src/
    ├── 功能包 package A
    │   ├── C++/Python 源码
    │   ├── msg/       自定义消息格式
    │   ├── srv/       自定义服务格式
    │   ├── launch/    启动说明
    │   └── config/    YAML/JSON 配置
    └── 功能包 package B
```

本项目有两个主要 ROS 1 工作空间：

```text
AstraDrone_ros1_ws/          核心算法和控制
simulation/sim_workspace/   Gazebo 插件与仿真辅助包
```

### 2.2 运行层：Master、节点和通信接口

```text
                         roscore
                 ┌──────────┴──────────┐
                 │ 节点登记簿          │ 参数服务器
                 └──────────┬──────────┘
                            │ 帮节点互相发现
            ┌───────────────┼────────────────┐
            ▼               ▼                ▼
          节点 A           节点 B           节点 C
             └──话题/服务直接通信────────────┘
```

下面逐个解释这些词。

### 2.3 ROS Master 和 `roscore`

ROS 1 中，节点需要先找到 ROS Master。Master 像通讯录：记录“哪个节点发布什么话题、哪个节点需要什么话题”。

启动命令是：

```bash
roscore
```

`roscore` 还会启动：

- 参数服务器：保存运行参数；
- `/rosout`：收集节点日志。

Master 主要帮助节点互相发现。节点建立连接后，大量传感器数据通常直接在节点之间传输，不是全部经过 Master 转发。

### 2.4 节点 node：一个正在运行的 ROS 程序

节点不是源码文件，也不是功能包。它是一个程序运行后的实例。

本项目中的转换过程：

```text
autoarming_control.cpp              源码
        │ catkin 编译
        ▼
devel/lib/offboard/autoarming_control  可执行文件
        │ roslaunch 启动
        ▼
/autoarming_control                节点
```

节点名前面的 `/` 表示它位于 ROS 名称空间的根目录。

几个真实节点：

| 节点 | 通俗解释 |
|---|---|
| `/mavros` | ROS 与 PX4/MAVLink 之间的翻译员 |
| `/autoarming_control` | 产生起飞、飞轨迹、降落目标的控制程序 |
| `/laserMapping` | FAST-LIO 定位建图程序 |
| `/gazebo` | Gazebo 对 ROS 提供的主接口 |
| `/rviz_a_loam` | 把坐标系、轨迹和点云画出来 |

一个功能包可以提供多个可执行文件；同一个可执行文件也可以用不同名字启动多个节点。

### 2.5 话题 topic：持续广播的数据通道

话题适合连续产生的数据，例如图像、IMU、点云、当前位置和控制目标。

```text
发布者 publisher ──> /某个话题 ──> 订阅者 subscriber
```

发布者只负责发，订阅者按需接收。两边不必同时写在一个程序中，也不必使用相同编程语言。

项目中的真实例子：

```text
/mavros
   │ 发布当前位置
   ▼
/mavros/local_position/pose
   │ 被订阅
   ▼
/autoarming_control
```

反方向还有一条话题：

```text
/autoarming_control
   │ 发布目标位置
   ▼
/mavros/setpoint_position/local
   │ 被订阅
   ▼
/mavros
```

这两条话题形成“读取当前位置，再发送目标位置”的控制闭环。

### 2.6 消息 message：话题中数据的格式

话题像快递路线，消息类型像统一的快递箱规格。同一个话题的发布者和订阅者必须同意数据格式。

例如：

```text
话题：/mavros/local_position/pose
类型：geometry_msgs/PoseStamped
```

`PoseStamped` 可以先粗略理解为：

```text
PoseStamped
├── header
│   ├── stamp       数据产生时间
│   └── frame_id    数据属于哪个坐标系
└── pose
    ├── position    x、y、z 位置
    └── orientation 四元数姿态（一种表示“朝向”的数学格式）
```

代码中的订阅语句是：

```cpp
ros::Subscriber pose_sub =
    nh.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 10, pose_cb);
```

逐段阅读：

- `geometry_msgs::PoseStamped`：期待的消息类型；
- `mavros/local_position/pose`：订阅的话题名；
- `10`：接收队列长度；
- `pose_cb`：每收到一条消息就调用的处理函数。

发布语句是：

```cpp
ros::Publisher local_pos_pub =
    nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);
```

### 2.7 服务 service：请求一次，回答一次

话题适合持续广播。服务适合“请执行一次操作，并告诉我结果”。

```text
服务客户端 ──请求──> 服务端
服务客户端 <──响应── 服务端
```

本项目的控制器会调用 MAVROS 服务：

| 服务 | 请求内容 |
|---|---|
| `/mavros/set_mode` | 请 PX4 切换到 `OFFBOARD` 模式 |
| `/mavros/cmd/arming` | 请 PX4 解锁 |
| `/mavros/cmd/command` | 发送 MAVLink 命令，本项目用于结束后上锁 |

简单类比：话题像广播电台，服务像打电话问一个问题并等待回答。

### 2.8 动作 action：可以反馈进度的长任务

动作适合持续较久、需要进度、结果和取消功能的任务，例如“导航到某个地点”。

```text
客户端发送 goal
        │
        ├──持续收到 feedback
        ├──可以 cancel
        └──最后收到 result
```

当前仓库没有 `.action` 文件，也没有实际的 `actionlib` 客户端或服务端。默认飞行过程由 `autoarming_control.cpp` 中自己编写的状态机完成。

### 2.9 参数 parameter：运行时配置项

参数适合保存高度、阈值、开关、话题名等低频配置。

例如 Launch 文件中有：

```xml
<param name="hight" value="3.0"/>
```

代码中读取：

```cpp
nh_private.param("hight", hight, 3.0);
```

最终参数名是 `/autoarming_control/hight`。这里的 `hight` 是源码中的真实拼写，并不是文档笔误。

注意：Launch 中写了参数，不代表程序一定使用。这个项目还设置了 `speed` 和 `takeoff_height`，但当前控制源码没有读取它们，所以它们不生效。

### 2.10 Launch 文件：一张批量启动清单

如果每个节点都手动启动，命令会很多。Launch 文件可以：

- 启动一个或多个节点；
- 加载参数；
- 修改节点名；
- 重映射话题名；
- include 另一个 Launch 文件；
- 按条件启动 RViz 等工具。

真实片段：

```xml
<node pkg="offboard"
      type="autoarming_control"
      name="autoarming_control"
      output="screen">
    <param name="hight" value="3.0"/>
</node>
```

- `pkg`：到哪个功能包找程序；
- `type`：运行哪个可执行文件；
- `name`：运行后的节点名。

“重映射”就是只在启动时把一个接口名换成另一个名字，不修改源码。例如同一控制程序可以把 `/mavros/state` 改接到 `/uav0/mavros/state`。

### 2.11 配置文件：参数的持久化版本

YAML、JSON、SDF 等文件通常保存较多配置：

- FAST-LIO 的雷达、IMU、外参：`fast_lio/config/mid360.yaml`；
- Livox 实机网络配置：`livox_ros_driver2/config/MID360_config.json`；
- Gazebo 虚拟雷达配置：`simulation/astra_gazebo_models/mid360/mid360.sdf`；
- 相机雷达外参：`lidar_cam_fusion/config/lidar_cam_fusion.yaml`。

Launch 可以把 YAML 一次加载到参数服务器。

### 2.12 TF：坐标系之间的关系

机器人上有很多坐标系。例如：

- 世界坐标系：描述无人机在场地中的位置；
- 机体坐标系：原点跟着无人机移动；
- 雷达坐标系：原点在雷达安装位置；
- 相机坐标系：原点在相机光心。

同一个点在不同坐标系下会有不同数值。TF 保存“坐标系 A 怎样变换到坐标系 B”。

本项目默认有：

```text
map ──静态变换──> camera_init ──FAST-LIO 动态变换──> body
```

`map -> camera_init` 被设置为重合，主要是让 RViz 中的坐标树连起来。这并不表示 PX4 已经在使用 FAST-LIO 的定位结果。

### 2.13 功能包 package 和工作空间 workspace

功能包是一组相关代码和资源。一个标准 ROS 1 功能包通常包含：

```text
package.xml      包名、版本和依赖
CMakeLists.txt   编译规则
src/             C++ 源码
script/          Python 脚本
launch/          Launch 文件
config/          配置文件
msg/、srv/       自定义通信格式
```

工作空间是多个功能包共同编译的容器。catkin 编译后会产生：

```text
src/     源码，主要阅读和修改这里
build/   编译中间文件，不要手改
devel/   可执行文件、生成的消息代码和环境脚本
```

每次编译后需要 `source devel/setup.bash`，告诉当前终端“新包和新消息在哪里”。

---

## 3. 把所有概念连起来

### 3.1 从源码到节点

```text
功能包中的源码
      │ CMakeLists.txt + catkin_make
      ▼
可执行文件和生成的消息代码
      │ source devel/setup.bash
      ▼
roslaunch 读取 Launch 和配置
      │
      ▼
节点开始运行
      │
      ├──发布/订阅话题
      ├──提供/调用服务
      ├──读取参数
      └──发布/查询 TF
```

### 3.2 最常用术语速查

| 术语 | 最简单的理解 | 项目实例 |
|---|---|---|
| 工作空间 workspace | 多个包的编译容器 | `AstraDrone_ros1_ws/` |
| 功能包 package | 一个功能模块 | `offboard`、`fast_lio` |
| 节点 node | 正在运行的程序 | `/autoarming_control` |
| 话题 topic | 持续传数据的通道 | `/mavros/state` |
| 消息 message | 一条数据的格式 | `mavros_msgs/State` |
| 服务 service | 一次请求和响应 | `/mavros/cmd/arming` |
| 动作 action | 可反馈/取消的长任务 | 当前项目未使用 |
| 参数 parameter | 运行配置 | `/autoarming_control/hight` |
| Launch | 批量启动说明 | `autoarming_control.launch` |
| TF | 坐标系关系 | `camera_init -> body` |
| rosbag | 数据录制和回放 | 录制 IMU、点云、位姿 |

---

## 4. 本项目整体在做什么

AstraDroneOpen 把无人机仿真、飞控通信、定位建图、控制、规划和视觉算法放在一个仓库中。

### 4.1 先分清 ROS 与周边系统

| 名称 | 是什么 | 是 ROS 节点吗 |
|---|---|---|
| PX4 SITL | 软件在环飞控，执行姿态/位置控制和安全检查 | `/sitl` 会出现在 Launch 节点表中，但 PX4 本身不是普通 ROS 算法 |
| Gazebo Classic | 模拟世界、物理、无人机和传感器 | 通过 `/gazebo` 和插件接入 ROS |
| MAVLink | PX4 使用的通信协议 | 不是节点 |
| MAVROS | MAVLink 与 ROS 消息/服务之间的桥 | 是，节点名 `/mavros` |
| QGroundControl | 地面站界面 | 不是 ROS 节点 |
| FAST-LIO | 雷达与 IMU 定位建图算法 | 是，默认节点 `/laserMapping` |
| Offboard 控制器 | 产生位置目标并调用 MAVROS | 是，默认节点 `/autoarming_control` |

### 4.2 项目里常见的非 ROS 词

这些词不是 ROS 的基本构成，但会频繁出现在无人机项目中：

| 词 | 通俗解释 |
|---|---|
| 飞控、FCU | 无人机底层控制系统；本项目使用 PX4 |
| SITL | Software In The Loop，用电脑软件模拟飞控，不接真实飞控板 |
| IMU | 惯性测量单元，测量加速度和角速度 |
| LiDAR | 激光雷达，用激光测距离并形成三维点云 |
| 位姿 pose | “在哪里”加“朝哪个方向”，即位置和姿态 |
| 里程计 odometry | 随时间估计的位置、姿态和速度 |
| SLAM | 一边估计自身位置，一边建立周围地图 |
| setpoint | 发给控制器的目标值，例如目标位置 |
| OFFBOARD | PX4 接受外部计算机控制指令的模式 |
| 外参 | 两个传感器坐标系之间的固定位置和旋转关系 |

### 4.3 默认演示的全景图

```text
                     QGroundControl
                           │ MAVLink
                           ▼
                    ┌─────────────┐
                    │  PX4 SITL   │
                    └──────┬──────┘
                       MAVLink│
                              ▼
                         /mavros
                       ▲          ▲
       状态与当前位姿  │          │ 目标位置与服务请求
                       │          │
                       └── /autoarming_control


Gazebo 虚拟 Mid360
  ├── /livox/lidar ──────┐
  └── /livox/imu ────────┤
                          ▼
                    /laserMapping
                          │
                          ├── /Odometry
                          ├── /cloud_registered
                          ├── /Laser_map
                          └── camera_init -> body TF
```

图中最重要的一点：**默认控制链和 FAST-LIO 链是并行的。**

`/autoarming_control` 读取的是 MAVROS 的 `/mavros/local_position/pose`，不是 FAST-LIO 的 `/Odometry`。仓库默认也没有把 `/Odometry` 送入 PX4。因此：

- FAST-LIO 启动了，不代表 PX4 正在用它定位；
- FAST-LIO 建出了地图，不代表无人机已经在避障；
- 要做自主避障，还需要把定位、点云、规划输出和 PX4 控制接口正确连接起来。

---

## 5. 本项目是 ROS 1 还是 ROS 2

结论：**当前仓库中能够构建和启动的主工程是 ROS 1 Noetic。**

判断依据：

| 证据 | 真实位置或写法 |
|---|---|
| catkin 工作空间 | `AstraDrone_ros1_ws/.catkin_workspace` |
| Noetic 的 catkin 顶层文件 | `AstraDrone_ros1_ws/src/CMakeLists.txt` 指向 `/opt/ros/noetic/share/catkin/cmake/toplevel.cmake` |
| ROS 1 C++ API | `ros::init`、`ros::NodeHandle`、`advertise`、`subscribe` |
| ROS 1 Python API | `rospy.init_node` |
| ROS 1 命令 | `roscore`、`roslaunch`、`rosnode`、`rostopic` |
| ROS 1 构建方式 | `catkin_make` |

`livox_ros_driver2` 中有 `package_ROS2.xml`、`launch_ROS2/` 等 ROS 2 兼容文件，但这只是一个第三方驱动同时支持两代 ROS，不能说明整个项目是 ROS 2。

顶层 README 提到 `AstraDrone_ros2_ws/`，当前仓库没有这个目录，因此完整 ROS 2 工程标记为 **待确认**。

---

## 6. 项目启动时发生了什么

### 6.1 已退役的历史默认入口

旧 `pc_example.sh` 一键组合已退役删除，以下仅保留历史流程说明；当前任务使用 [项目整理文档·运行说明](项目整理文档.md#runbook)。

它用 tmux 建立 5 个窗格：

| 时间 | 启动内容 | 作用 |
|---:|---|---|
| 立即 | `roscore` | 启动 ROS Master、参数服务器和日志 |
| 3 秒后 | `astra_example.launch` | 启动 PX4 SITL、Gazebo、机体模型和 MAVROS |
| 6 秒后 | FAST-LIO | 启动 `/laserMapping` |
| 10 秒后 | Offboard 控制 | 启动 `/autoarming_control`，会尝试解锁飞行 |
| 立即 | `qgc` | 启动 QGroundControl |

脚本使用固定 `sleep`，并没有检查上一步是否真的准备好。电脑较慢时，后续节点可能启动过早。

脚本还假定当前 shell 已有两个别名：

- `astra`：source 核心工作空间；
- `qgc`：启动 QGroundControl。

可先检查：

```bash
type astra
type qgc
```

如果 `astra` 不存在，可在对应 tmux 窗格中显式执行：

```bash
source ~/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash --extend
```

### 6.2 PX4/Gazebo/MAVROS Launch

文件：

```text
simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch
```

默认设置：

- 机型：`iris_mid360`；
- 世界：`simulation/astra_gazebo_worlds/dynamic_avoidance.world`；
- 初始位置：约 `(0, 0, 0.06)`；
- MAVROS 连接：`udp://:14540@localhost:14557`。

它继续 include 仓库外部的：

- PX4 `posix_sitl.launch`；
- MAVROS `px4.launch`。

因此 PX4 和 MAVROS 的一部分节点、话题和服务来自本机安装环境，不全部位于这个仓库中。

### 6.3 FAST-LIO Launch

命令：

```bash
roslaunch fast_lio mapping_mid360.launch rviz:=false
```

过程：

```text
读取 config/mid360.yaml
        │
        ├──输入话题 /livox/lidar
        ├──输入话题 /livox/imu
        ├──读取噪声、外参和发布开关
        ▼
启动 /laserMapping
```

### 6.4 Offboard Launch

命令：

```bash
roslaunch offboard autoarming_control.launch
```

它启动：

- `/map_to_camera_init`：静态 TF；
- `/autoarming_control`：自动飞行控制节点；
- `/rviz_a_loam`：可视化。

控制器内部状态依次为：

```text
等待 FCU 连接
      │
发送 100 帧当前位置目标
      │
请求 OFFBOARD 和解锁
      │
TAKEOFF 起飞
      │
TRACKING 飞圆形或方形轨迹
      │
LANDING 返回并下降
      │
COMPLETED 上锁并退出
```

### 6.5 安全提醒

已退役的 `pc_example.sh` 原来会启动 `autoarming_control.launch`；后者仍是底层历史控制组件，会尝试切换 OFFBOARD、解锁并发送飞行目标。

- 仿真学习时，先只启动 PX4/Gazebo/MAVROS；
- 实机上不要直接运行默认控制脚本；
- 先确认桨叶已拆除、坐标系正确、急停方式可用并完成 PX4 安全检查；
- 可以先跟随 `stage0.md` 建立不发送控制命令的基线。

---

## 7. 默认节点怎样传数据

### 7.1 飞行控制数据流

```text
PX4
 │ 飞行模式、连接和解锁状态
 ▼
/mavros
 │ /mavros/state
 │ mavros_msgs/State
 ▼
/autoarming_control

PX4/Gazebo 当前位姿
 │
 ▼
/mavros/local_position/pose
 │ geometry_msgs/PoseStamped
 ▼
/autoarming_control
 │ 计算下一个目标
 ▼
/mavros/setpoint_position/local
 │ geometry_msgs/PoseStamped
 ▼
/mavros ──MAVLink──> PX4
```

关键话题：

| 话题 | 发布者 | 订阅者 | 消息类型 | 含义 |
|---|---|---|---|---|
| `/mavros/state` | `/mavros` | `/autoarming_control` | `mavros_msgs/State` | 是否连接、当前模式、是否解锁 |
| `/mavros/local_position/pose` | `/mavros` | `/autoarming_control` | `geometry_msgs/PoseStamped` | 当前局部位置和姿态 |
| `/mavros/setpoint_position/local` | `/autoarming_control` | `/mavros` | `geometry_msgs/PoseStamped` | 希望 PX4 到达的位置 |

### 7.2 雷达、IMU 和 FAST-LIO 数据流

```text
Gazebo Mid360 雷达插件
 │ /livox/lidar
 │ Livox CustomMsg
 ├──────────────────────────┐
 │                          │
Gazebo IMU 插件             │
 │ /livox/imu               │
 │ sensor_msgs/Imu          │
 └──────────────────────────┤
                            ▼
                      /laserMapping
                            │
           ┌────────────────┼────────────────┐
           ▼                ▼                ▼
       /Odometry    /cloud_registered    /Laser_map
 nav_msgs/Odometry  PointCloud2          PointCloud2
```

关键话题：

| 话题 | 发布者 | 订阅者 | 消息类型 | 含义 |
|---|---|---|---|---|
| `/livox/lidar` | Gazebo Mid360 插件 | `/laserMapping` | Livox `CustomMsg` | 一帧雷达点和每个点的相对时间 |
| `/livox/imu` | Gazebo IMU 插件 | `/laserMapping` | `sensor_msgs/Imu` | 加速度和角速度 |
| `/Odometry` | `/laserMapping` | 默认控制链没有订阅者 | `nav_msgs/Odometry` | FAST-LIO 计算的位姿和速度 |
| `/cloud_registered` | `/laserMapping` | RViz/其他算法 | `sensor_msgs/PointCloud2` | 放到世界坐标系后的点云 |
| `/cloud_registered_body` | `/laserMapping` | 其他算法可订阅 | `sensor_msgs/PointCloud2` | 机体坐标系中的点云 |
| `/Laser_map` | `/laserMapping` | RViz/规划算法可订阅 | `sensor_msgs/PointCloud2` | 当前地图点云 |
| `/path` | `/laserMapping` | RViz | `nav_msgs/Path` | 运动轨迹；默认参数关闭，不发布 |

仿真插件发布的类型名可能显示为 `livox_laser_simulation/CustomMsg`，FAST-LIO 源码使用 `livox_ros_driver/CustomMsg`。当前构建中的两个消息定义兼容；实际运行时用 `rostopic type /livox/lidar` 确认。

### 7.3 TF 数据流

```text
/map_to_camera_init
       │ 发布静态 TF
       ▼
map -> camera_init
                  \
                   \ /laserMapping 发布动态 TF
                    ▼
              camera_init -> body
```

RViz 根据 TF 把来自不同坐标系的数据画到同一个三维场景中。TF 连通只代表“数学上知道如何换坐标”，不代表某个定位结果已经进入 PX4。

### 7.4 怎样判断一个话题是否真的连上

以 `/livox/lidar` 为例：

```bash
rostopic info /livox/lidar
```

重点看三项：

- `Type`：消息类型；
- `Publishers`：谁在发布；
- `Subscribers`：谁在订阅。

如果有发布者但没有订阅者，数据产生了但没人使用；如果有订阅者但没有发布者，节点只能等待。

---

## 8. 项目目录应该怎样看

### 8.1 根目录

| 路径 | 作用 | 新手优先级 |
|---|---|---|
| `AstraDrone_ros1_ws/` | 核心 ROS 1 工作空间 | 高 |
| `simulation/` | Gazebo 世界、模型、PX4 机型和仿真工作空间 | 高 |
| `scripts/run_sh/` | 启动、观察和录包脚本 | 高 |
| `scripts/env_sh/` | ROS、PX4、依赖和环境安装脚本 | 出现环境问题时看 |
| `docs/` | 项目说明；部分内容比当前代码旧 | 中 |
| `hardware/` | 机械、PCB、BOM | ROS 入门阶段跳过 |
| `third_party/` | Livox SDK、AprilTag、NLopt 等第三方库 | 先跳过 |

### 8.2 核心模块

| 模块 | 作用 | 当前状态 |
|---|---|---|
| `MissionControl/` | Offboard 飞行控制 | 默认编译和启动 |
| `SLAM/` | FAST-LIO 定位建图 | 默认编译和启动 |
| `Planner/` | EGO-Planner | 有源码，当前被忽略 |
| `Detection/` | AprilTag、ArUco、YOLO、目标预测 | 有源码，当前被忽略 |
| `Land/` | 精准降落 | 有源码，当前被忽略 |
| `Track/` | 像素/目标跟踪 | 有源码，当前被忽略 |
| `Control/` | RC 遥控和规划器桥接 | 有源码，当前被忽略 |
| `Communication/` | 串口通信示例 | 有源码，当前被忽略 |
| `Utils/` | 自定义消息、驱动、TF、点云和相机工具 | 部分编译 |
| `Exploration/`、`Swarm/` | 当前为空 | 待确认 |

带有 `CATKIN_IGNORE` 的功能包会被 catkin 跳过。“源码存在”不等于“当前已经编译并能运行”。

### 8.3 当前实际编译的包

当前 `AstraDrone_ros1_ws/build/catkin_generated/order_packages.cmake` 记录了 6 个包：

| 包 | 作用 |
|---|---|
| `offboard` | 默认自动飞行和位置控制实验 |
| `astra_custom_msgs` | 项目自定义消息与服务 |
| `cv_bridge` | ROS 图像与 OpenCV 图像互转 |
| `livox_ros_driver` | 第一代 Livox ROS 驱动和消息定义 |
| `fast_lio` | 雷达与 IMU 定位建图 |
| `livox_ros_driver2` | Livox Driver2 实机驱动 |

### 8.4 最值得先打开的文件

按这个顺序阅读：

1. [项目整理文档·运行说明](项目整理文档.md#runbook)：当前三机用户入口；旧 pc_example shell 已退役；
2. `simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch`：PX4、Gazebo 和 MAVROS；
3. `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/launch/autoarming_control.launch`：节点和控制参数；
4. `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src/autoarming_control.cpp`：默认飞行状态机；
5. `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/launch/mapping_mid360.launch`：FAST-LIO 怎样启动；
6. `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/config/mid360.yaml`：输入话题、外参和发布开关；
7. `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/src/laserMapping.cpp`：FAST-LIO 的 ROS 接口；
8. `simulation/astra_gazebo_models/mid360/mid360.sdf`：虚拟雷达与 IMU 怎样发布数据。

---

## 9. 项目中有哪些节点

第一次学习只需要记住默认节点。可选节点先知道属于哪个模块即可。

### 9.1 默认演示节点

| 节点 | 谁启动 | 作用 |
|---|---|---|
| `/rosout` | `roscore` | 收集 ROS 日志 |
| `/sitl` | PX4 Launch | 运行 PX4 软件在环飞控 |
| `/gazebo` | PX4/Gazebo Launch | 模拟世界、物理和传感器 |
| `/gazebo_gui` | PX4/Gazebo Launch | Gazebo 图形界面 |
| `/vehicle_spawn_*` | PX4/Gazebo Launch | 把 `iris_mid360` 放入世界；后缀会变化 |
| `/mavros` | MAVROS Launch | MAVLink 与 ROS 的桥 |
| `/laserMapping` | FAST-LIO Launch | 雷达/IMU 定位建图 |
| `/map_to_camera_init` | Offboard Launch | 静态发布 `map -> camera_init` |
| `/autoarming_control` | Offboard Launch | 自动起飞、轨迹、降落 |
| `/rviz_a_loam` | Offboard Launch | 三维可视化 |

QGroundControl 不是 ROS 节点，所以不会出现在 `rosnode list` 中。

### 9.2 已编译但默认不启动

| 程序/常用节点名 | 作用 |
|---|---|
| `position_control` / `/position_control` | 把 `/drone_control/goal_position` 转发为 MAVROS 位置目标 |
| `pub_origin` / `/pub_origin` | 发布 MAVROS 全球坐标原点 |
| `livox_ros_driver_node` / `/livox_lidar_publisher` | 第一代 Livox 实机驱动 |
| `livox_ros_driver2_node` / `/livox_lidar_publisher2` | MID360 等 Livox 实机驱动 |
| `fastlio_mapping_color` / `/laserMapping_color` | 彩色点云 FAST-LIO；其 Launch 还引用缺失的 `freedom` 包，完整启动待确认 |

### 9.3 可选模块中的主要节点

这些包当前多带有 `CATKIN_IGNORE`：

| 模块 | 主要节点 | 作用 |
|---|---|---|
| EGO-Planner | `/ego_planner_node`、`/traj_server`、`/waypoint_generator` | 路径搜索、轨迹优化和轨迹采样 |
| 规划仿真 | `/pcl_render_node`、`/so3_control`、`/quadrotor_simulator_so3` | 模拟感知、控制和四旋翼动力学 |
| ArUco | `/aruco`、`/EKF` | 识别标记并预测目标位置 |
| AprilTag | `/apriltag_ros_continuous_node` | 连续识别 AprilTag，发布像素、检测和 TF |
| YOLO | `/yolo_detect` | 目标检测 |
| 精准降落 | `/astra_static_land` | 根据目标估计向 MAVROS 发送降落指令 |
| 跟踪 | `/pix_tracker`、`/target_pulisher` | 选择目标像素并产生跟踪控制 |
| RC 避障桥 | `/rc_obstacle_avoidance` | 把遥控输入转成规划目标并转发规划结果 |
| 相机/点云工具 | `camera_sdk`、`camera_lidar_fusion`、`pixel2map` | 相机驱动、雷达相机融合、像素坐标变换 |
| TF 工具 | `tf_to_odometry_node`、`tf_publisher_node` | TF 与 Pose/Odometry 相互转换 |
| 动态障碍 | `obstacle_controller` | 调用 Gazebo 服务移动障碍物 |

可选节点的依赖和接口没有在当前最小构建中完整验证，使用前应逐包检查。

---

## 10. 服务、参数、Launch 和配置速查

### 10.1 默认控制服务

| 服务 | 类型 | 调用者 | 用途 |
|---|---|---|---|
| `/mavros/set_mode` | `mavros_msgs/SetMode` | `/autoarming_control` | 请求 `OFFBOARD` |
| `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | `/autoarming_control` | 请求解锁 |
| `/mavros/cmd/command` | `mavros_msgs/CommandLong` | `/autoarming_control` | 结束后发送上锁命令 |

其他源码中还定义或使用：

- `/astra_auto_land/activate`：激活精准降落；
- `/mavros/cmd/land`：MAVROS 降落服务；
- `single_image_tag_detection`：单张 AprilTag 检测；
- `~refresh_tag_params`：重新读取 AprilTag 参数；
- `~calibrate_attitude`：ArUco 姿态校准；
- `~estimate_depth`：根据像素查询融合深度；
- `/gazebo/set_model_state`：移动 Gazebo 模型。

`astra_custom_msgs/MissionCommand.srv` 定义了 `TAKEOFF/LAND/ABORT`，但没有找到对应服务端或客户端，目前只是预留接口。

### 10.2 默认 Offboard 参数

| 参数 | 默认值 | 是否被代码读取 | 作用 |
|---|---:|---|---|
| `/flight_mode` | `circle` | 是 | 选择圆形或方形轨迹 |
| `/autoarming_control/hight` | `3.0` | 是 | 起飞和巡航高度 |
| `/autoarming_control/target_laps` | `2` | 是 | 轨迹圈数 |
| `/autoarming_control/side_length` | `8.0` | 是 | 方形边长 |
| `/autoarming_control/radius` | 代码默认 `2.0` | 是 | 圆形半径；Launch 未设置，因此通常不会出现在参数服务器列表中 |
| `/autoarming_control/takeoff_height` | `1.0` | 否 | 当前不生效 |
| `/autoarming_control/speed` | `1.5` | 否 | 当前不生效 |

### 10.3 FAST-LIO 关键参数

| 参数 | 默认值 | 作用 |
|---|---|---|
| `/common/lid_topic` | `/livox/lidar` | 雷达输入 |
| `/common/imu_topic` | `/livox/imu` | IMU 输入 |
| `/preprocess/lidar_type` | `1` | 使用 Livox 自定义消息 |
| `/preprocess/blind` | `0.5` | 忽略过近点 |
| `/mapping/det_range` | `100.0` | 探测范围参数 |
| `/mapping/extrinsic_T` | `[-0.011,-0.02329,0.04412]` | 雷达到 IMU 的平移外参 |
| `/mapping/extrinsic_R` | 单位矩阵 | 雷达到 IMU 的旋转外参 |
| `/publish/path_en` | `false` | 默认不发布 `/path` |
| `/pcd_save/pcd_save_en` | `false` | 默认不保存 PCD |

### 10.4 主要 Launch

| Launch | 作用 |
|---|---|
| `simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch` | PX4、Gazebo、机体和 MAVROS |
| `fast_lio/launch/mapping_mid360.launch` | MID360 FAST-LIO |
| `offboard/launch/autoarming_control.launch` | 默认自动飞行 |
| `offboard/launch/position_control.launch` | 简单位置目标转发实验 |
| `offboard/launch/autoarming_Mult.launch` | 多机尝试，当前完整性待确认 |
| `livox_ros_driver2/launch_ROS1/msg_MID360.launch` | 实机 MID360 驱动 |
| `ego_planner/launch/run_in_sim.launch` | EGO-Planner 独立仿真 |
| `aruco_localization/launch/aruco_detect.launch` | ArUco 与 EKF |
| `astra_auto_land/launch/astra_static_land.launch` | 精准降落 |

表中的包相对路径可用 `rospack find 包名` 找到完整位置。例如：

```bash
rospack find fast_lio
rospack find offboard
```

---

## 11. 用命令亲手观察 ROS

### 11.1 第一个零风险实验：自己发布一个话题

这个实验不启动 PX4、Gazebo，也不会控制无人机。

终端 1：

```bash
source /opt/ros/noetic/setup.bash
roscore
```

终端 2：

```bash
source /opt/ros/noetic/setup.bash
rostopic pub -r 1 /hello std_msgs/String "data: 'hello ROS'"
```

终端 3：

```bash
source /opt/ros/noetic/setup.bash
rostopic echo /hello
```

你刚刚创建了：

```text
rostopic pub 进程 ──发布──> /hello ──订阅──> rostopic echo 进程
                              │
                       std_msgs/String
```

再执行：

```bash
rosnode list
rostopic list
rostopic info /hello
rostopic type /hello
rosmsg show std_msgs/String
```

这组实验能把节点、话题、发布者、订阅者和消息类型一次串起来。

### 11.2 source 两个工作空间

```bash
source /opt/ros/noetic/setup.bash
source ~/AstraDroneOpen/simulation/sim_workspace/devel/setup.bash
source ~/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash --extend
```

`--extend` 表示保留前一个工作空间，再叠加当前工作空间。可以检查：

```bash
rospack find env_map
rospack find fast_lio
```

完整 PX4 仿真还需要 `~/PX4-Autopilot`、MAVROS 和 Gazebo Classic 环境。安装脚本通常把这些设置写入 `~/.bashrc`。如果提示找不到 `px4` 或 `mavlink_sitl_gazebo`，检查：

```bash
rospack find px4
rospack find mavlink_sitl_gazebo
echo "$GAZEBO_MODEL_PATH" | tr ':' '\n'
```

### 11.3 编译

核心工作空间的项目构建器：

```bash
cd ~/AstraDroneOpen

./scripts/build_AstraDrone_ros1_x86.bin --list
./scripts/build_AstraDrone_ros1_x86.bin --state
./scripts/build_AstraDrone_ros1_x86.bin --small_build clean_build=false
```

手动编译：

```bash
source /opt/ros/noetic/setup.bash

cd ~/AstraDroneOpen/simulation/sim_workspace
catkin_make -j8
source devel/setup.bash

cd ~/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make -j8
source devel/setup.bash --extend
```

带 `CATKIN_IGNORE` 的包不会参与 `catkin_make`。不要为了“全部编译”直接删除所有忽略文件，因为可选包可能缺少依赖或接口尚未适配。

### 11.4 分阶段启动项目

比一键脚本更适合新手的方式：

下面每个新终端都要先按第 11.2 节 source 环境；省略 source 只会让示例更短，不表示它可以跳过。

```bash
# 终端 1
roscore
```

```bash
# 终端 2：PX4、Gazebo、MAVROS
roslaunch ~/AstraDroneOpen/simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch
```

```bash
# 终端 3：FAST-LIO
roslaunch fast_lio mapping_mid360.launch rviz:=false
```

确认系统安全以后才考虑终端 4：

```bash
# 会尝试 OFFBOARD、解锁和飞行
roslaunch offboard autoarming_control.launch
```

Launch 不运行时也可以静态查看：

```bash
roslaunch --nodes offboard autoarming_control.launch
roslaunch --dump-params fast_lio mapping_mid360.launch
```

### 11.5 查看节点

```bash
rosnode list
rosnode info /mavros
rosnode info /laserMapping
rosnode info /autoarming_control
rosnode ping /mavros
```

`rosnode info` 会列出一个节点发布、订阅和调用的接口，是理解陌生节点最实用的命令之一。

### 11.6 查看话题和消息

```bash
rostopic list
rostopic info /livox/lidar
rostopic type /livox/lidar
rosmsg show "$(rostopic type /livox/lidar)"

rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/local_position/pose
rostopic echo -n 1 /Odometry

rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic bw /cloud_registered
```

- `echo`：看实际数据；
- `hz`：看每秒发布多少次；
- `bw`：看带宽；
- `info`：看连接关系。

### 11.7 查看服务、参数和 TF

```bash
rosservice list
rosservice info /mavros/cmd/arming
rossrv show mavros_msgs/CommandBool

rosparam list
rosparam get /flight_mode
rosparam get /autoarming_control
rosparam get /common

rosrun tf tf_echo map camera_init
rosrun tf tf_echo camera_init body
```

不要为了测试随意调用解锁、切模式或降落服务。

### 11.8 图形化查看和录包

```bash
rqt_graph
rosrun rqt_tf_tree rqt_tf_tree
rviz
roswtf
```

录制最小诊断数据：

```bash
rosbag record -O astra_debug.bag \
  /livox/lidar /livox/imu /Odometry \
  /mavros/state /mavros/local_position/pose \
  /mavros/setpoint_position/local /tf /tf_static

rosbag info astra_debug.bag
```

rosbag 可以把一次运行的数据保存下来，之后不启动真实传感器也能回放分析。

---

## 12. 新手排错时按什么顺序检查

不要一看到错误就重装 ROS。沿数据流逐段检查。

### 12.1 找不到功能包

```bash
echo "$ROS_PACKAGE_PATH" | tr ':' '\n'
rospack find fast_lio
rospack find env_map
```

通常是没有 source，或者两个工作空间的 source 顺序不对。

### 12.2 MAVROS 没连接 PX4

```bash
rostopic echo -n 1 /mavros/state
rosnode info /mavros
```

检查：

- `connected` 是否为 `true`；
- `/sitl` 是否存在；
- UDP 地址是否是 Launch 中的 `14540/14557`；
- PX4 控制台是否报错。

### 12.3 FAST-LIO 没有 `/Odometry`

```bash
rostopic info /livox/lidar
rostopic info /livox/imu
rostopic hz /livox/lidar
rostopic hz /livox/imu
rosnode info /laserMapping
```

按顺序问：

1. 雷达有没有发布者？
2. IMU 有没有发布者？
3. `/laserMapping` 有没有订阅它们？
4. 消息类型是否匹配？
5. 时间戳是否持续向前？

### 12.4 无法进入 OFFBOARD 或解锁

```bash
rostopic echo /mavros/state
rostopic hz /mavros/setpoint_position/local
```

PX4 通常要求进入 OFFBOARD 前已经持续收到 setpoint。默认控制器先发送 100 帧，约 5 秒。还要检查 PX4 状态估计和安全检查，而不是反复强制解锁。

### 12.5 修改参数没有效果

依次检查：

1. `rosparam get` 看到的参数路径和值是否正确；
2. 源码是否调用 `param()` 或 `getParam()`；
3. 参数是在全局还是节点私有命名空间；
4. 修改后是否重启了读取参数的节点。

本项目中的真实反例是 `speed` 和 `takeoff_height`：Launch 设置了，但代码没有读取。

---

## 13. 推荐学习顺序

### 第 1 步：只做 `/hello` 实验

目标：亲眼看到发布者、话题、消息类型和订阅者。

### 第 2 步：只观察 MAVROS，不发送控制

启动 PX4/Gazebo/MAVROS，观察：

```text
/mavros/state
/mavros/local_position/pose
```

目标：分清 PX4、MAVLink、MAVROS 和 ROS 节点。

### 第 3 步：读懂 `autoarming_control.cpp`

沿着下面的顺序读：

```text
ros::init
  -> param
  -> subscribe/advertise
  -> serviceClient
  -> 等待连接
  -> TAKEOFF/TRACKING/LANDING 状态机
```

目标：把源码中的每个 ROS 接口对应到本文的数据流图。

### 第 4 步：单独启动 FAST-LIO

只关注：

```text
/livox/lidar + /livox/imu
        -> /laserMapping
        -> /Odometry + 点云 + TF
```

目标：理解消息频率、时间戳、坐标系和外参。

### 第 5 步：学会用 `rqt_graph` 和 rosbag

目标：遇到问题时能用数据证明“链路断在哪里”，而不是只看 Gazebo 画面猜测。

### 第 6 步：再学习规划和视觉模块

EGO-Planner、AprilTag、ArUco、YOLO、精准降落同时涉及规划、坐标变换和控制安全。先掌握默认通信链，再逐个恢复被忽略的包。

### 理解本项目最重要的知识

按优先级排序：

1. 节点、话题、消息、发布和订阅；
2. Launch、参数和功能包；
3. `source`、catkin 和 `CATKIN_IGNORE`；
4. PX4、MAVLink、MAVROS 的分工；
5. TF、坐标系和四元数；
6. 传感器时间戳、频率和外参；
7. 状态机和 OFFBOARD 安全规则；
8. rosbag 和可重复调试方法。

---

## 14. 当前需要标记为“待确认”的内容

1. README 提到的完整 `AstraDrone_ros2_ws/` 当前不存在。
2. 文档提到的 `scripts/run_sh/onboard_example.sh` 和 `demonstration.sh` 当前不存在。
3. MAVROS 和 PX4 的部分 Launch、节点和服务来自仓库外部，具体内容取决于本机安装版本。
4. `model://mid360` 的解析受 `GAZEBO_MODEL_PATH` 顺序影响，运行时应确认 `/livox/lidar` 和 `/livox/imu` 的真实发布者。
5. 实机的雷达外参、相机内参、相机雷达外参和噪声参数必须重新标定。
6. Detection、Land、Planner、Track 和多数 Utils 包当前被忽略，没有在最小构建中完整验证。
7. 默认没有把 FAST-LIO `/Odometry` 接入 PX4 外部里程计。
8. 默认没有完成 EGO-Planner 输出到 MAVROS setpoint 的桥接。
9. `autoarming_Mult.launch` 的多机节点命名和服务重映射不完整，不能视为已验证方案。
10. `mapping_mid360_color.launch` 引用当前仓库缺失的 `freedom` 包。
11. `Exploration/` 和 `Swarm/` 当前为空。
12. `scripts/*.bin` 是封装后的 ELF 文件，内部全部操作无法直接从源码确认。

---

## 15. 最后一张速记卡

```text
ROS 的作用：
  组织机器人程序，让它们通信、配置、启动和调试。

最核心的关系：
  工作空间包含功能包
  功能包中的源码编译成可执行文件
  可执行文件运行后成为节点
  节点通过话题持续传消息
  节点通过服务完成一次请求/响应
  参数保存运行配置
  Launch 批量启动节点并加载参数
  TF 保存坐标系之间的关系

本项目默认控制链：
  autoarming_control <-> MAVROS <-> PX4 <-> Gazebo

本项目默认定位链：
  Gazebo Mid360 -> livox/lidar + livox/imu
                 -> laserMapping
                 -> Odometry + 点云 + TF

最重要的项目事实：
  当前是 ROS 1 Noetic。
  FAST-LIO 默认没有进入 PX4 控制闭环。
  很多可选包带 CATKIN_IGNORE。
  默认控制 Launch 会尝试解锁和飞行。
```
