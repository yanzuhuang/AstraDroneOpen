# 阶段 0 教程：建立可复现的 PX4/Gazebo/MAVROS 仿真基线

> 适用对象：第一次接触 ROS、PX4 和无人机仿真的学习者。
>
> 对应路线：`studymap.md` 的“阶段 0：建立可复现的仿真基线”。
>
> 本阶段只观察和检查系统，**不启动 `autoarming_control`、不切换 OFFBOARD、不解锁飞机**。

## 1. 学完后你应该真正会什么

完成本阶段后，你应该能独立做到以下事情，而不是只会运行一键脚本：

1. 说清楚 Gazebo、PX4 SITL、MAVLink、MAVROS、ROS 节点各自负责什么。
2. 不用 `pc_example.sh`，分终端启动 `roscore` 和 PX4/Gazebo/MAVROS。
3. 判断 PX4 与 MAVROS 是否连接、局部位姿是否有效、某个话题由谁发布。
4. 区分 topic、service、parameter、TF 和 launch 的用途。
5. 理解 catkin 的 `src/`、`build/`、`devel/`，知道何时需要编译和 `source`。
6. 识别 ENU、NED、世界坐标系、机体坐标系和四元数，不把它们混用。
7. 用绝对路径切换 `example.world` 与 `forest.world`，确认加载结果并记录实时因子。
8. 正常停止所有进程，并按照固定顺序排查常见启动故障。

如果上面任何一项还只能“照抄命令但不知道结果是什么意思”，阶段 0 就还没有完成。

## 2. 先建立一张正确的系统地图

当前阶段的实际数据链可以简化为：

```text
Gazebo Classic
  负责虚拟世界、重力、碰撞、无人机动力学和传感器
       │ 模拟传感器数据                         ▲ 电机/执行器指令
       ▼                                        │
PX4 SITL
  运行飞控算法、状态估计、飞行模式和安全逻辑
       │                                        ▲
       └──────────── MAVLink 协议 ──────────────┘
                         │
                      MAVROS
              MAVLink 与 ROS 消息之间的桥梁
                         │
         ┌───────────────┴────────────────┐
         ▼                                ▲
 /mavros/state、local pose          ROS setpoint/service
         │                                │
         └──────── 将来才启动的控制节点 ───┘
```

本阶段没有控制节点，所以链路只运行到 MAVROS。无人机保持未解锁是正确现象。

### 2.1 五个角色分别做什么

| 组件 | 本项目中的角色 | 它不负责什么 |
|---|---|---|
| Gazebo | 模拟场景、物理运动、碰撞和传感器 | 不决定任务航点，不实现 ROS 控制状态机 |
| PX4 SITL | 在电脑中运行 PX4 飞控逻辑，完成估计、模式、安全检查及底层控制 | 不是三维场景渲染器，也不是 ROS 节点 |
| MAVLink | 定义 PX4 与外部程序交换状态和命令的数据协议 | 它只是协议，不是一个 ROS 节点 |
| MAVROS | 把 MAVLink 消息转换成 ROS topic/service，且处理常用坐标约定转换 | 不替代 PX4 的飞控算法 |
| Offboard 控制节点 | 将来持续发送“飞到哪里”的 setpoint，并请求模式切换与解锁 | 不直接计算每个电机的转速 |

一个好记的类比：

- 控制节点像导航员，给出目的地。
- MAVROS 像翻译员。
- PX4 像司机，负责真的把车开稳。
- Gazebo 像虚拟车辆和道路。

## 3. “需要掌握的知识点”逐项详解

### 3.1 ROS1：node、topic、service、parameter、launch、namespace、TF

#### 3.1.1 roscore 与 ROS Master

`roscore` 会启动 ROS Master 和参数服务器。Master 主要负责“登记和介绍”：

1. 节点向 Master 注册自己的名字、发布的话题和订阅的话题。
2. Master 告诉发布者和订阅者如何找到彼此。
3. 两个节点建立连接后，话题数据通常在节点之间直接传输，不需要每帧都经过 Master。

所以 Master 更像通讯录，而不是所有数据都必须经过的中转站。

没有可用的 Master 时，常见报错为：

```text
ERROR: Unable to communicate with master!
```

#### 3.1.2 node：正在运行的程序实例

node 是一个加入 ROS 通信图的运行实例。例如本项目启动后常见节点有：

```text
/mavros
/gazebo
/gazebo_gui
/sitl
```

“源码文件”“编译出的可执行文件”和“node”不是同一个概念：源码经过编译得到可执行文件；可执行文件运行起来并向 ROS 注册后，才成为节点。

常用命令：

```bash
rosnode list
rosnode info /mavros
```

`rosnode info` 会显示这个节点订阅、发布的话题及提供的服务。

#### 3.1.3 topic：连续的数据流

topic 使用发布/订阅模型：

- publisher 持续发送消息；
- subscriber 接收消息；
- 双方通过话题名和消息类型匹配；
- 一般是异步通信，不等待对方回复。

适合 topic 的数据包括位姿、图像、点云、IMU 和控制目标。一个话题可以有多个订阅者，也可能有多个发布者；但“技术上允许多个发布者”不等于“控制话题应该有多个发布者”。

常用命令：

```bash
rostopic list
rostopic info /mavros/local_position/pose
rostopic type /mavros/local_position/pose
rostopic echo -n 1 /mavros/local_position/pose
rostopic hz /mavros/local_position/pose
```

它们分别回答：有什么话题、谁在使用、消息类型是什么、一帧内容是什么、发布频率是多少。

#### 3.1.4 service：一次请求和一次响应

service 是同步的请求/响应通信，适合“执行一次动作并告诉我是否受理”，例如：

```text
/mavros/cmd/arming   请求解锁或上锁
/mavros/set_mode     请求切换飞行模式
```

本阶段只查看服务，**不要调用解锁或模式切换服务**：

```bash
rosservice list | grep mavros
rosservice type /mavros/cmd/arming
rossrv show mavros_msgs/CommandBool
```

topic 与 service 的核心区别：位姿需要持续更新，用 topic；解锁是一次请求并需要成功/失败结果，用 service。

#### 3.1.5 parameter：运行配置

parameter 是保存在参数服务器中的配置值，适合频率、阈值、开关和文件路径，不适合高频传感器数据。

```bash
rosparam list
rosparam get /use_sim_time
rosparam get /mavros
```

一个参数“存在”只说明它已被加载，不代表某段 C++ 代码真的读取并使用了它。这一点在后续检查 `speed`、`takeoff_height` 是否生效时非常重要。

#### 3.1.6 launch：批量启动和配置

`.launch` 是 XML 文件，可以：

- 启动 node；
- include 其他 launch；
- 声明和传递 `<arg>`；
- 写入 parameter；
- 设置 namespace 和 remap。

当前仓库的 `astra_example.launch` 自己不实现飞控，它主要 include：

```text
PX4 的 posix_sitl.launch -> PX4 SITL + Gazebo + 机体生成
MAVROS 的 px4.launch     -> MAVROS
```

要区分 `<arg>` 与 parameter：launch arg 用来在解析启动文件时传值；ROS parameter 会存入参数服务器，供运行中的节点读取。

#### 3.1.7 namespace：给名字分组

namespace 是 ROS 名字的前缀。例如 `/mavros/state` 中 `/mavros` 就起到了分组作用。多机时可以使用 `/uav1/mavros/state`、`/uav2/mavros/state` 避免重名。

三种常见名字：

- `/name`：全局绝对名；
- `name`：相对当前 namespace 解析；
- `~name`：节点私有名，最终通常变为 `/节点名/name`。

#### 3.1.8 TF：随时间维护坐标系关系

TF 不是“某个位置数值”，而是带时间的坐标系变换系统。它回答：

> 在某个时刻，坐标系 A 中的点转换到坐标系 B 后是什么坐标？

例如 `map -> base_link -> lidar_link` 可以表达世界、机体和雷达之间的关系。静态安装关系通常发布到 `/tf_static`，运动关系通常发布到 `/tf`。

检查方法：

```bash
rostopic info /tf
rostopic info /tf_static
rosrun rqt_tf_tree rqt_tf_tree
```

如果阶段 0 中没有完整的 `map -> base_link` TF，不要为了“让图好看”随便发布假变换；先记录是哪个节点本应提供它。

#### 3.1.9 它们之间到底是什么关系

最重要的一句话是：

> **launch 负责把系统组织并启动起来，node 是真正干活的程序，node 通过 topic 和 service 协作、从 parameter 读取配置、通过 TF 统一坐标系；namespace 为这些 ROS 名字分组。**

可以把它们画成下面这张关系图：

```text
                         launch 文件
               “启动谁、参数是什么、名字放哪里”
                    │       │       │
                    │       │       └── 设置 namespace / remap
                    │       └────────── 写入 parameter server
                    └────────────────── 启动 node
                                             │
                 ┌───────────────────────────┼───────────────────────────┐
                 │                           │                           │
                 ▼                           ▼                           ▼
          发布/订阅 topic              提供/调用 service            读取 parameter
          连续、异步的数据流            一次请求和响应                运行配置
                 │                           │
                 └───────────────┬───────────┘
                                 │
                         node 与 node 协作
                                 │
                                 ▼
                    TF broadcaster 发布坐标变换
                     TF listener 查询坐标关系

      namespace 作用于 node、topic、service、parameter 等 ROS 名字的解析
      ROS Master 负责节点登记、名称查找和参数服务器，不负责具体业务计算
```

这张图中有三个层次：

1. **组织层**：launch、namespace 和 parameter 决定系统如何启动和配置。
2. **运行层**：node 是实际运行和计算的主体。
3. **通信与空间层**：topic、service 负责节点通信，TF 负责不同坐标系的数据能够被正确转换。

##### 关系一：launch 与 node

launch 文件是“启动说明书”，node 是“被启动后真正工作的程序”。例如：

```xml
<node pkg="mavros" type="mavros_node" name="mavros" />
```

这段配置表达的是：从 `mavros` 包找到 `mavros_node` 可执行文件，运行它，并把 ROS 节点命名为 `mavros`。

需要注意：

- launch 文件自身不处理位姿、不计算控制量；
- 真正运行 launch 的是 `roslaunch` 进程，它还会监控和关闭由它启动的子进程；
- 一个 launch 可以启动多个 node，也可以 include 其他 launch；
- 同一个可执行文件可以被启动多次，只要节点名和资源不冲突。

##### 关系二：node 与 topic

node 是 topic 的发布者或订阅者。topic 离开 node 不会自己产生数据。

以本项目为例：

```text
/mavros 节点
    ├── 发布 /mavros/state
    ├── 发布 /mavros/local_position/pose
    └── 订阅 /mavros/setpoint_position/local

未来启动的 /autoarming_control 节点
    ├── 订阅 /mavros/state
    ├── 订阅 /mavros/local_position/pose
    └── 发布 /mavros/setpoint_position/local
```

topic 是两个 node 之间的“数据通道”，消息类型则规定通道里每帧数据的结构。如果发布者使用 `geometry_msgs/PoseStamped`，订阅者也必须使用相同类型。

还要注意：普通 topic 通常不保存完整历史。订阅者晚启动时，一般只能收到连接建立后的新消息，而不是自动获得之前所有消息；需要历史数据时应使用 rosbag 等工具。

##### 关系三：node 与 service

一个 node 可以作为 service server 提供功能，另一个 node 作为 service client 发起请求：

```text
/autoarming_control                 /mavros
     service client  ──请求解锁──>  service server
                     <──响应结果──
```

service 名称只是一条可查找的接口，真正执行请求的仍是提供该 service 的 node。若 `/mavros` 没有运行，即使你记得 `/mavros/cmd/arming` 这个名字，也没有服务端可以响应。

一个 node 可以同时：

- 发布多个 topic；
- 订阅多个 topic；
- 提供多个 service；
- 调用其他 node 的 service。

因此 node、topic 和 service 不是三种互斥的程序，而是“程序主体”和“两种通信方式”的关系。

##### 关系四：node 与 parameter

parameter 为 node 提供配置，例如频率、高度、开关和 frame 名称：

```text
parameter server 中保存 /autoarming_control/hight = 3.0
                              │
                              └── autoarming_control 节点主动读取
```

parameter 不会自动控制 node。必须由 node 的代码执行读取操作，例如 C++ 中调用 `nh.param(...)` 或 `getParam(...)`，参数才会影响程序逻辑。

这会带来三个结论：

1. launch 可以把 parameter 写入参数服务器；
2. node 决定是否读取、何时读取以及如何使用；
3. 运行中修改 parameter 后，node 不一定立刻变化——如果代码只在启动时读取一次，就需要重启 node 才能生效。

parameter 与 topic 也不能互相替代：

| 需求 | 应使用 | 原因 |
|---|---|---|
| 设置目标飞行高度 `3.0 m` | parameter | 低频配置值 |
| 持续接收当前位置 | topic | 高频、随时间变化的数据 |
| 请求切换一次模式并获取结果 | service | 一次请求/响应 |

##### 关系五：namespace 与所有 ROS 名字

namespace 不是通信方式，也不是单独运行的 node。它是一套名称解析规则，会影响 node、topic、service 和 parameter 的最终全名。

假设某节点位于 `/uav1` namespace 下，并使用相对名称：

```text
节点名：          mavros                 -> /uav1/mavros
topic 名：        mavros/state           -> /uav1/mavros/state
service 名：      mavros/cmd/arming      -> /uav1/mavros/cmd/arming
parameter 名：    takeoff_height         -> 根据读取句柄继续解析
```

多机系统可利用 namespace 形成：

```text
/uav1/mavros/state
/uav2/mavros/state
```

但 namespace 只是名字分组，不会自动创建两个 PX4 实例、隔离 UDP 端口或复制参数。多机还必须分别配置进程、端口和机体实例。

绝对名会绕过当前 namespace。例如节点处于 `/uav1` 下，但代码写死订阅 `/mavros/state`，它仍然指向全局 `/mavros/state`，而不是 `/uav1/mavros/state`。这正是多机程序应谨慎使用绝对名称的原因。

##### 关系六：TF 与 node、topic

TF 的数据也由 node 发布和接收，底层使用两个特殊 topic：

```text
/tf          动态变换，例如运动中的 map -> base_link
/tf_static   静态变换，例如 base_link -> lidar_link
```

典型关系为：

```text
定位节点（TF broadcaster）
    └── 发布 map -> base_link 到 /tf

机体描述/静态发布节点
    └── 发布 base_link -> lidar_link 到 /tf_static

感知节点（TF listener）
    └── 查询 map <- base_link <- lidar_link
        把 lidar_link 中的点云转换到 map 坐标系
```

虽然 TF 底层使用 topic，但它比普通 pose topic 多了一套语义和工具：

- 变换必须说明父、子 frame；
- 变换带时间戳，可以查询某一时刻的关系；
- 多段变换可以组成一棵坐标树；
- TF 库能自动计算两个已连通 frame 之间的组合变换。

`/mavros/local_position/pose` 和 TF 也不是同一件事：前者是一条位姿消息；后者维护整套坐标系之间随时间变化的关系。一个节点可以根据 pose 生成 TF，但不会因为存在 pose topic 就自动出现对应 TF。

##### 关系七：launch、namespace、parameter 如何一起作用

下面是一个只用于理解语法的简化例子：

```xml
<launch>
  <group ns="uav1">
    <node pkg="demo_pkg" type="controller" name="controller">
      <param name="takeoff_height" value="1.5" />
      <remap from="state" to="mavros/state" />
    </node>
  </group>
</launch>
```

解析后的关系大致是：

```text
launch
  ├── 在 /uav1 namespace 下启动 controller node
  │      最终节点名：/uav1/controller
  ├── 写入节点私有参数
  │      /uav1/controller/takeoff_height = 1.5
  └── remap 节点使用的相对 topic 名
         state -> /uav1/mavros/state
```

这里的 remap 是“改接口名称映射”，parameter 是“给程序配置数值”，namespace 是“决定名称前缀”。它们作用不同，不能因为都写在 launch 里就认为是一回事。

##### 用当前项目把全过程串起来

当你执行 `roslaunch .../astra_example.launch` 时，系统大致经历：

```text
1. roslaunch 解析 launch arg，例如 world、vehicle、fcu_url
2. include PX4 的 posix_sitl.launch 和 MAVROS 的 px4.launch
3. 启动 /sitl、/gazebo、/gazebo_gui、/mavros 等 node/进程
4. 各 node 向 ROS Master 注册自己的 ROS 名字和接口
5. /mavros 通过 MAVLink 与 PX4 SITL 连接
6. /mavros 开始发布 /mavros/state 和 local position topics
7. /mavros 提供 arming、set_mode 等 services
8. 其他 node 可以读取 parameters，并通过 topic/service 与 /mavros 协作
9. 涉及不同 frame 的数据，再利用 /tf 和 /tf_static 建立坐标关系
```

所以排错时也应沿着这条关系链逐层检查：

```text
launch 是否解析成功
  -> node 是否真的运行
  -> namespace 下的最终名字是否正确
  -> parameter 是否存在且被代码读取
  -> topic 是否有正确 publisher/subscriber
  -> service server 是否存在且响应
  -> TF 的父子 frame、时间戳和连通性是否正确
```

不要一看到“飞机没有反应”就直接修改控制算法。可能只是 node 没启动、话题名受 namespace 影响、parameter 没被读取，或 TF 坐标树不连通。

### 3.2 `use_sim_time`：为什么暂停 Gazebo 后 ROS 时间也会停

仿真可以不用电脑墙上时钟，而使用 Gazebo 发布的 `/clock`：

```bash
rosparam get /use_sim_time
rostopic echo -n 1 /clock
```

当 `/use_sim_time` 为 `true` 时，ROS 节点里的 `ros::Time::now()` 使用仿真时间。Gazebo 暂停，仿真时间也暂停。这让传感器、控制器和回放数据共享一致的时间轴。

常见误区：程序“定时器不走了”不一定是程序死锁，也可能是 Gazebo 被暂停且程序使用仿真时间。

### 3.3 catkin 工作空间与 `source`

ROS1 的 catkin 工作空间通常包含：

```text
workspace/
├── src/      源码、package.xml、CMakeLists.txt，应在这里修改
├── build/    CMake/catkin 中间产物，不手工修改
└── devel/    编译后的可执行文件、库和环境脚本，不手工修改
```

#### 3.3.1 `source devel/setup.bash` 做了什么

`source` 会修改**当前终端进程**的环境变量，让 ROS 找到这个工作空间的包、消息、库和可执行文件：

```bash
source devel/setup.bash
```

三个必须记住的结论：

1. 它只影响当前终端和该终端后续启动的子进程，不会自动影响已经打开的其他终端。
2. 每个新终端都要重新加载环境；写入 `.bashrc` 只是让新交互终端自动执行。
3. 多个工作空间存在 overlay 顺序，最后 source 的工作空间优先；顺序错误可能让先前的包消失。

#### 3.3.2 当前机器的特殊注意事项

当前环境的 `~/.bashrc` 已加载：

- ROS Noetic；
- `simulation/sim_workspace/devel/setup.bash`；
- PX4 Gazebo Classic 环境和 `ROS_PACKAGE_PATH`；
- Astra Gazebo 模型路径。

因此阶段 0 每个终端先执行：

```bash
source ~/.bashrc
```

当前 `astra` 是加载主 ROS1 工作区的 alias。经实际检查，在同一终端执行它后，`px4` 和 `env_map` 会暂时无法被 `rospack` 找到。因此：

- 阶段 0 的 PX4/Gazebo 启动终端不要执行 `astra`；
- 后续启动 Offboard 时，在单独终端执行 `astra`；
- 环境混乱时，关闭该终端重新打开，或重新 `source ~/.bashrc`。

#### 3.3.3 什么改动需要编译

| 修改内容 | 是否需要 catkin 编译 | 还需要什么 |
|---|---:|---|
| `.cpp`、自定义消息、CMake 配置 | 需要 | 重新 source 对应 `devel/setup.bash` |
| `.launch`、`.yaml`、`.world` | 通常不需要 | 停止并重启相关节点 |
| Gazebo 插件 C++ | 需要编译仿真工作区 | 确认加载了新 `.so` |
| 外部 PX4 中的 airframe/SDF | 视修改而定 | 确认仓库文件与运行副本同步 |

### 3.4 Gazebo：world、model/SDF、插件、物理步长和实时因子

#### 3.4.1 world

world 是整个仿真场景，通常包含：

- 地面、树、建筑和障碍物；
- 光照和天空；
- 物理引擎参数；
- 要预先放入场景的 model；
- world 级插件。

本阶段使用：

```text
simulation/astra_gazebo_worlds/example.world
simulation/astra_gazebo_worlds/forest.world
```

#### 3.4.2 model 与 SDF

SDF 用 XML 描述仿真对象。常见元素包括：

- `link`：具有质量、惯量的刚体；
- `joint`：连接两个 link；
- `visual`：看起来是什么样；
- `collision`：物理碰撞形状；
- `sensor`：相机、IMU、LiDAR 等；
- `plugin`：把算法或通信逻辑接入 Gazebo。

仓库中的 `iris_mid360.sdf` 会组合无 GPS Iris、Mid360、D435i 和 FPV 相机模型。当前 launch 的 `sdf` 默认值实际指向外部 PX4 Gazebo 模型目录中的运行副本，而不是直接读取仓库内 SDF。

#### 3.4.3 插件

插件是 Gazebo 运行时加载的共享库，例如：

- 模拟电机；
- 产生 IMU、气压计、LiDAR 或相机数据；
- 在 Gazebo 与 PX4 间传输模拟传感器和执行器数据；
- 向 ROS 发布话题或提供服务。

看到模型不代表插件一定加载成功。若终端出现 `Failed to load plugin ...so`，可能出现“外观存在但没有传感器数据或不能运动”。

#### 3.4.4 物理步长与实时因子

`example.world` 和 `forest.world` 当前都设置：

```xml
<max_step_size>0.001</max_step_size>
<real_time_update_rate>1000</real_time_update_rate>
```

含义是目标每个物理步长为 0.001 秒、每仿真秒进行约 1000 次更新，理论目标实时因子约为：

```text
0.001 × 1000 = 1.0
```

实时因子（Real Time Factor，RTF）定义为：

```text
仿真时间前进量 / 现实时间前进量
```

- RTF ≈ 1：仿真 10 秒大约需要现实 10 秒；
- RTF = 0.5：仿真 10 秒大约需要现实 20 秒；
- 短时波动正常，长期很低才说明本机算力或场景负载不足。

物理配置给出的是目标，不保证电脑一定能达到。森林模型、传感器、GUI 和其他算法都会降低实际 RTF。

### 3.5 PX4 SITL、MAVLink、MAVROS 与 OFFBOARD

#### 3.5.1 PX4 SITL

SITL 是 Software In The Loop。PX4 飞控软件运行在电脑进程中，不需要真实飞控板；Gazebo 提供模拟传感器数据，PX4 计算执行器输出再交回 Gazebo。

这仍然是一套飞控闭环，不是播放预先录制的动画。

#### 3.5.2 MAVLink 与 MAVROS

MAVLink 规定飞行器状态、位置目标、模式命令等消息如何编码。MAVROS 一侧连接 PX4 的 MAVLink 通道，另一侧提供 ROS topic 和 service。

例如：

```text
PX4 状态 --MAVLink--> MAVROS --ROS topic--> /mavros/state
ROS setpoint --> /mavros/setpoint_position/local --> MAVROS --MAVLink--> PX4
```

#### 3.5.3 OFFBOARD 为什么要求连续 setpoint

OFFBOARD 表示 PX4 接受机外计算机发送的目标。连续 setpoint 不只是“更新目标位置”，还是控制源仍然存活的心跳证据。

如果外部程序只发送一次目标后崩溃，PX4 不能无限相信这个失联程序。因此 PX4 要求先出现稳定的 setpoint 流，进入 OFFBOARD 后也必须持续接收；流中断时会触发退出或失控保护。当前项目控制器使用 20 Hz，并在请求 OFFBOARD 前预发送 100 个 setpoint。

本阶段不发布 setpoint，所以：

- `/mavros/setpoint_position/local` 没有 publisher 是正确结果；
- 模式不是 `OFFBOARD` 是正确结果；
- `armed: false` 是正确结果。

### 3.6 三个关键 MAVROS 话题怎么读

#### 3.6.1 `/mavros/state`

类型是 `mavros_msgs/State`，关键字段如下：

| 字段 | 含义 | 阶段 0 期望 |
|---|---|---|
| `connected` | MAVROS 是否收到了飞控连接 | `true` |
| `armed` | 是否解锁 | `false` |
| `mode` | 当前 PX4 飞行模式 | 不要求固定，但不应是由本阶段进入的 OFFBOARD |
| `guided` | MAVROS 对 guided 状态的表示 | 不是本阶段主要验收项 |
| `system_status` | MAVLink 系统状态枚举值 | 用于辅助诊断，不能单独判断所有健康状态 |

`connected: true` 只证明通信链通了，不等于局部位置一定有效，也不等于允许解锁。

#### 3.6.2 `/mavros/local_position/pose`

类型是 `geometry_msgs/PoseStamped`：

```text
header.stamp       这帧数据的时间戳
header.frame_id    该位姿是在哪个坐标系表达的
pose.position      x、y、z，单位通常是米
pose.orientation   x、y、z、w 四元数
```

检查时要看四件事：

1. 能否收到消息；
2. 时间戳是否继续变化；
3. 数值中是否出现 `nan` 或 `inf`；
4. 无人机静止时数值是否大体稳定。

它是 PX4 估计并经 MAVROS 转换的局部位姿接口，不能不加判断地当成 Gazebo 原始真值。

#### 3.6.3 `/mavros/local_position/odom`

类型是 `nav_msgs/Odometry`，比 `PoseStamped` 多出：

- `child_frame_id`；
- 带协方差的 pose；
- 线速度和角速度 twist；
- 速度协方差。

简单理解：pose 主要告诉你“在哪里、朝向哪里”；odom 还告诉你“如何运动”以及估计不确定度的表达位置。

### 3.7 ENU、NED、世界系、机体系与四元数

#### 3.7.1 世界系和机体系

- 世界/局部坐标系：坐标轴固定在环境中，无人机飞行时它不跟着机头转。
- 机体坐标系：固定在无人机上，飞机转向时坐标轴一起转。

“世界系向东 1 m/s”和“机头方向前进 1 m/s”不是一回事；机头转过 90° 后，后者对应的世界方向也改变。

#### 3.7.2 ENU 与 NED

| 约定 | X | Y | Z | 常见位置 |
|---|---|---|---|---|
| ENU | East 东 | North 北 | Up 上 | ROS/MAVROS 的常见本地接口 |
| NED | North 北 | East 东 | Down 下 | PX4 内部航空坐标约定 |

MAVROS 会为常用接口执行坐标转换。因此观察 `/mavros/local_position/pose` 时按它的 ROS frame 和 ENU 语义理解，不要因为知道 PX4 内部用 NED 就再次手工交换 x/y 或把 z 取反，否则会转换两次。

#### 3.7.3 四元数

四元数用 `(x, y, z, w)` 表示三维旋转。你现在只需掌握：

1. 它不是欧拉角，四个分量不能直接当作 roll、pitch、yaw。
2. 合法单位四元数应满足 `x²+y²+z²+w²≈1`。
3. 无旋转的单位四元数是 `(0, 0, 0, 1)`。
4. `(0, 0, 0, 0)` 不是合法旋转。
5. 四元数 `q` 和 `-q` 表示相同姿态，所以分量符号翻转不一定是姿态突变。

本阶段不要求手推 ENU/NED 变换矩阵，也不要求手算四元数乘法；要求是看消息时能识别坐标系和数据类型。

## 4. 仿真实践：从零完成阶段 0

### 4.1 实验规则和终端分工

至少准备三个终端：

| 终端 | 用途 | 本阶段是否允许输入飞行命令 |
|---|---|---:|
| 终端 1 | `roscore` | 否 |
| 终端 2 | PX4 SITL + Gazebo + MAVROS | 否 |
| 终端 3 | ROS 检查命令 | 只读检查 |
| 终端 4（可选） | `rqt_graph`、TF 或 `gz stats` | 只读检查 |

不要运行 `pc_example.sh`，因为它还会启动 FAST-LIO、Offboard 控制器和 QGC，阶段 0 不容易分清每个组件来自哪里。

每个终端开头都执行：

```bash
source ~/.bashrc
cd "$HOME/AstraDroneOpen"
```

以下命令默认仓库位于 `$HOME/AstraDroneOpen`，这与当前环境一致。

### 4.2 实验前检查环境

在终端 3 执行：

```bash
source ~/.bashrc
cd "$HOME/AstraDroneOpen"

rospack find px4
rospack find mavros
rospack find env_map

test -f simulation/astra_gazebo_worlds/example.world && echo "example.world: OK"
test -f simulation/astra_gazebo_worlds/forest.world && echo "forest.world: OK"
test -f simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch \
  && echo "astra_example.launch: OK"
```

当前机器的期望路径大致为：

```text
px4      -> /home/yanzu/PX4-Autopilot
mavros   -> /opt/ros/noetic/share/mavros
env_map  -> /home/yanzu/AstraDroneOpen/simulation/sim_workspace/src/env_map
```

如果 `px4` 或 `env_map` 找不到，先不要启动，直接看第 6 节排错。

还可以只解析 launch、不真正启动，用于提前发现 XML 和包路径错误：

```bash
roslaunch --nodes \
  "$HOME/AstraDroneOpen/simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch" \
  world:="$HOME/AstraDroneOpen/simulation/astra_gazebo_worlds/example.world"
```

当前应能看到类似节点：

```text
/sitl
/gazebo
/gazebo_gui
/vehicle_spawn_...
/mavros
```

### 4.3 终端 1：单独启动 roscore

```bash
source ~/.bashrc
roscore
```

看到 `started core service [/rosout]` 一类输出后，不要关闭该终端。

在终端 3 验证：

```bash
rosnode list
```

此时至少应看到 `/rosout`。如果出现无法连接 Master，检查终端 1 是否仍在运行。

### 4.4 终端 2：启动 PX4/Gazebo/MAVROS，但不启动控制器

使用仓库内 launch 的**绝对路径**和 world 的**绝对路径**：

```bash
source ~/.bashrc
cd "$HOME/AstraDroneOpen"

roslaunch \
  "$HOME/AstraDroneOpen/simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch" \
  world:="$HOME/AstraDroneOpen/simulation/astra_gazebo_worlds/example.world"
```

为什么这里不用更短的 `roslaunch px4 astra_example.launch`：

1. 绝对 launch 路径明确使用仓库当前文件；
2. `roslaunch px4 ...` 会从外部 `~/PX4-Autopilot` 包中找旧副本；
3. 当前外部副本与仓库文件确实不同；
4. 绝对 world 路径不依赖 launch 中的相对路径拼接。

启动后预期看到 Gazebo GUI 和一架 `iris_mid360`。保持终端 2 开启，不要在 PX4 shell 中输入 `commander arm` 等命令。

### 4.5 终端 3：先查节点和连接

```bash
rosnode list | sort
```

重点应该存在：

```text
/gazebo
/gazebo_gui
/mavros
/sitl
```

生成机体的节点名称可能带随机后缀，不要死记完整名字。

查看 MAVROS 详情：

```bash
rosnode info /mavros
```

再检查飞控连接：

```bash
rostopic echo -n 1 /mavros/state
```

合格的关键结果是：

```yaml
connected: true
armed: false
```

`mode` 的初始值可能受 PX4 状态和配置影响，本阶段不要求死记某个字符串。只要没有运行控制节点，就不应把“必须是 OFFBOARD”当作目标。

如果命令一直不返回，表示暂时没有消息；按 `Ctrl+C`，按照第 6 节从连接开始排查。

### 4.6 检查局部位姿和里程计

先看一帧位姿：

```bash
rostopic echo -n 1 /mavros/local_position/pose
```

你需要亲自指出：

- `header.frame_id` 是什么；
- position 的 x、y、z 是多少；
- orientation 的四元数是否接近单位长度；
- 是否存在 `nan` 或 `inf`。

然后测频率：

```bash
rostopic hz /mavros/local_position/pose
```

等待至少 10 秒，看到多次统计后按 `Ctrl+C`。不要只记录一瞬间的一个数字，应记录平均频率及范围。

再查看 odom：

```bash
rostopic echo -n 1 /mavros/local_position/odom
rostopic type /mavros/local_position/odom
rosmsg show nav_msgs/Odometry
```

尝试从输出中找到 pose、linear velocity 和 angular velocity，确认你理解它比 `PoseStamped` 多了什么。

### 4.7 检查 setpoint 发布者：阶段 0 最关键的一次判断

```bash
rostopic info /mavros/setpoint_position/local
```

没有启动 Offboard 控制器时，预期类似：

```text
Type: geometry_msgs/PoseStamped

Publishers: None

Subscribers:
 * /mavros (...)
```

含义是：MAVROS 正在等待某个控制节点发送目标，但现在没有控制节点发布。这不是故障，正是阶段 0 的安全基线。

如果 `Publishers` 下出现节点：

1. 记录节点名；
2. 执行 `rosnode info /节点名`；
3. 停止该控制节点；
4. 在发布者清空前不要继续实验。

以后任何时刻都只应有一个节点持续控制同一架无人机。多个 setpoint 发布者会让目标互相覆盖，产生跳变和不可预测行为。

### 4.8 检查参数、仿真时间和服务

```bash
rosparam get /use_sim_time
rostopic echo -n 1 /clock

rosservice list | grep -E '^/mavros|^/gazebo' | sort
rosservice type /mavros/cmd/arming
rossrv show mavros_msgs/CommandBool
```

本阶段只观察 service 类型，不执行：

```bash
# 本阶段不要执行：
# rosservice call /mavros/cmd/arming "value: true"
# rosservice call /mavros/set_mode ...
```

### 4.9 用 rqt_graph 画出 ROS 侧连接

在终端 4 执行：

```bash
source ~/.bashrc
rqt_graph
```

建议在界面中取消隐藏无连接项，并搜索 `mavros`。至少找到：

- `/mavros` 节点；
- `/mavros/state`；
- `/mavros/local_position/pose`；
- `/mavros/setpoint_position/local` 的订阅关系。

注意：`rqt_graph` 展示 ROS 通信图，不会完整展示 PX4 内部模块、MAVLink 数据流或 Gazebo 自己的 Transport 通信。所以图上看不到某条内部链路，不等于它不存在。

把你的图画成下面这种逻辑即可，不要求界面布局一模一样：

```text
PX4 SITL <--MAVLink--> /mavros
                         ├──> /mavros/state --> 检查终端
                         ├──> /mavros/local_position/pose --> 检查终端
                         └<── /mavros/setpoint_position/local
                              当前只有 /mavros 订阅，无发布者
```

### 4.10 记录 Gazebo 实时因子

方法一：直接观察 Gazebo GUI 中的 Real Time Factor。

方法二：在新终端运行：

```bash
gz stats
```

等待约 20～30 秒，记录稳定后的大致范围，然后 `Ctrl+C`。如果 GUI 刷新影响性能，可以另外做一次 `gui:=false` 对照：

```bash
roslaunch \
  "$HOME/AstraDroneOpen/simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch" \
  world:="$HOME/AstraDroneOpen/simulation/astra_gazebo_worlds/example.world" \
  gui:=false
```

做对照前必须先停止原来的终端 2，不能同时启动两个使用相同端口的 PX4/Gazebo 实例。

### 4.11 切换到 forest.world 并证明切换成功

#### 第一步：停止旧场景

在终端 2 按一次 `Ctrl+C`，等待它关闭 PX4、MAVROS、Gazebo 和机体生成进程。终端 1 的 `roscore` 可以继续运行。

检查残留：

```bash
pgrep -af 'px4|gzserver|gzclient|mavros'
```

若没有输出，再进行下一步。不要在旧 Gazebo 未退出时启动新场景。

#### 第二步：启动森林场景

在终端 2 执行：

```bash
source ~/.bashrc

roslaunch \
  "$HOME/AstraDroneOpen/simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch" \
  world:="$HOME/AstraDroneOpen/simulation/astra_gazebo_worlds/forest.world"
```

#### 第三步：用三种证据确认

1. 启动命令中的绝对路径明确为 `forest.world`；
2. Gazebo 画面与 `example.world` 的模型数量/布局不同；
3. 查看 Gazebo 当前模型列表：

```bash
rosservice call /gazebo/get_world_properties "{}"
```

分别在两个 world 中保存 `model_names` 和实时因子。不要仅凭“Gazebo 窗口打开了”判断切换成功。

### 4.12 理解仓库文件和外部 PX4 运行副本

执行：

```bash
cd "$HOME/AstraDroneOpen"

rospack find px4
rospack find mavlink_sitl_gazebo

find "$(rospack find px4)/launch" -name astra_example.launch -print

cmp simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch \
  "$HOME/PX4-Autopilot/launch/astra_launch/astra_example.launch"
echo "launch cmp exit code: $?"

cmp simulation/px4_sim_files/px4_iris_sdf/iris_mid360/iris_mid360.sdf \
  "$(rospack find mavlink_sitl_gazebo)/models/iris_mid360/iris_mid360.sdf"
echo "SDF cmp exit code: $?"
```

`cmp` 的退出码：

- `0`：两个文件相同；
- `1`：文件内容不同；
- 大于 `1`：读取、路径等发生错误。

当前仓库状态的已知事实：

1. 仓库内 `astra_example.launch` 的 world 路径已补上 `/`，默认 world 是 `dynamic_avoidance.world`；
2. `pc_example.sh` 当前直接启动仓库内这份 launch；
3. 外部 PX4 中的 `astra_example.launch` 仍是旧副本，默认指向 `example.world` 且路径缺少 `/`；
4. 当前外部 `iris_mid360.sdf` 与仓库 SDF 相同；
5. 即使 launch 从仓库直接读取，默认 `sdf` 参数仍解析到外部 `mavlink_sitl_gazebo/models/`。

因此本阶段统一传入绝对 world 路径，并直接指定仓库 launch 路径。以后修改 SDF/airframe 时，必须再次确认外部运行副本是否同步。

这也解释了一个经典问题：你明明修改了仓库文件，运行效果却没变——程序可能根本没有读取你改的那一份。

### 4.13 正确关闭仿真

推荐按反向依赖顺序停止：

1. 关闭 `rqt_graph`、TF 和 `gz stats` 等观察工具；
2. 在终端 2 按 `Ctrl+C`，等待 roslaunch 清理 PX4、MAVROS 和 Gazebo；
3. 用 `pgrep` 检查是否还有残留；
4. 最后在终端 1 按 `Ctrl+C` 关闭 `roscore`。

验证：

```bash
pgrep -af 'roscore|rosmaster|px4|gzserver|gzclient|mavros'
```

通常不应有相关残留。优先用启动终端的 `Ctrl+C` 正常退出，不要养成每次都 `kill -9` 的习惯，因为强制杀进程会掩盖正常清理问题。

如果终端异常关闭造成残留，且你确认没有其他 ROS/Gazebo 任务正在运行，可最后使用当前环境提供的 `si` alias 清理；它会影响本机其他 ROS/Gazebo 进程，使用前必须确认范围。

## 5. 你应该保存的实验记录

建议把下面模板复制到自己的学习记录中，每个 world 填一份：

```markdown
## 阶段 0 实验记录

- 日期：
- Git commit：`git rev-parse --short HEAD`
- launch 绝对路径：
- world 绝对路径：
- `rospack find px4`：
- `rospack find mavlink_sitl_gazebo`：
- `/mavros/state.connected`：
- `/mavros/state.armed`：
- `/mavros/state.mode`：
- local pose 的 `frame_id`：
- local pose 平均频率：
- pose 是否存在 NaN/Inf：
- setpoint publishers：
- `/use_sim_time`：
- RTF 大致范围：
- `/gazebo/get_world_properties` 中的模型特征：
- 终端警告/错误：
- 我的解释：
```

建议另外保留：

- `example.world` 与 `forest.world` 各一张截图；
- 一张自己画的 PX4—MAVROS—ROS 话题图；
- `rostopic info /mavros/setpoint_position/local` 的输出。

## 6. 常见故障：按这个顺序排查

### 6.1 `Resource not found: env_map`

原因：当前终端没有加载仿真工作区，或者被另一个 workspace overlay 覆盖。

处理：

```bash
source ~/.bashrc
rospack find env_map
```

当前机器阶段 0 的仿真终端不要执行 `astra`。如果仍找不到，检查：

```bash
test -f "$HOME/AstraDroneOpen/simulation/sim_workspace/devel/setup.bash" \
  && echo "sim workspace built"
```

### 6.2 `Resource not found: px4`

原因：PX4 路径没有加入 `ROS_PACKAGE_PATH`。

先恢复当前机器已经配置好的环境：

```bash
source ~/.bashrc
rospack find px4
```

若仍失败，检查 `~/PX4-Autopilot` 是否存在以及 `.bashrc` 中的 PX4 setup 配置，不要通过随便复制 launch 文件来掩盖环境错误。

### 6.3 Gazebo 报 `Failed to load plugin ...so`

依次检查：

```bash
source ~/.bashrc
echo "$GAZEBO_PLUGIN_PATH" | tr ':' '\n'
echo "$LD_LIBRARY_PATH" | tr ':' '\n'
```

确认 PX4 Gazebo Classic 和仿真工作区的插件目录存在。模型能显示但传感器无数据时，插件加载错误尤其值得优先检查。

### 6.4 模型找不到或场景缺模型

```bash
echo "$GAZEBO_MODEL_PATH" | tr ':' '\n'
test -d "$HOME/AstraDroneOpen/simulation/astra_gazebo_models" && echo OK
```

当前 `.bashrc` 已把 Astra 模型目录加入 `GAZEBO_MODEL_PATH`。重新 `source ~/.bashrc` 后再启动。

### 6.5 端口占用、重复模型或第二次启动失败

通常是上次的 PX4/Gazebo/MAVROS 未退出：

```bash
pgrep -af 'px4|gzserver|gzclient|mavros'
```

回到原启动终端按 `Ctrl+C`。不要同时启动两个默认单机 launch，它们会竞争 MAVLink 和 Gazebo 端口。

### 6.6 `/mavros/state` 一直没有输出或 `connected: false`

按顺序检查：

1. `rosnode list` 是否存在 `/mavros` 和 `/sitl`；
2. 终端 2 是否有 PX4 启动失败或端口错误；
3. `rostopic hz /mavros/state` 是否有数据；
4. launch 的 `fcu_url` 是否仍为本单机配置；
5. 是否误启动了另一个占用相同端口的实例。

不要在通信未连接时跳到 setpoint 或解锁问题。

### 6.7 state 已连接，但 local pose 没有消息

`connected` 只代表 MAVLink 通信成立。继续检查：

```bash
rostopic info /mavros/local_position/pose
rostopic hz /mavros/local_position/pose
rostopic echo -n 1 /mavros/local_position/odom
```

同时看 PX4 终端是否有 estimator 警告、Gazebo 是否暂停、仿真时间是否推进。不要用手工伪造 pose 的方法绕过估计器问题。

### 6.8 `rostopic echo -n 1` 像“卡死”

它不是固定打印缓存，而是在等待下一帧。如果没有 publisher 或仿真暂停，就会一直等。另开终端执行 `rostopic info` 和 `rostopic hz`；需要退出时按 `Ctrl+C`。

### 6.9 Gazebo 很卡、RTF 很低

先只运行阶段 0 的最小链路，不启动 FAST-LIO、RViz、QGC 和 Offboard。然后对比：

1. `example.world` 与 `forest.world`；
2. `gui:=true` 与 `gui:=false`；
3. 系统 CPU/GPU 是否被其他进程占用。

RTF 低意味着现实中等 10 秒，仿真时间可能只走了几秒；后续用仿真时间计时的任务也会显得变慢。

## 7. 阶段 0 自测题与答案

### 题 1：Gazebo 里的飞机在动，是谁直接决定电机输出？

答：任务节点只提供上层目标，PX4 根据估计状态和目标计算控制输出，Gazebo 再模拟电机和机体运动。不能把 Gazebo 画面变化等同于控制节点直接操作电机。

### 题 2：`/mavros/state.connected=true` 是否代表可以安全解锁？

答：不是。它只证明 MAVROS 与飞控通信已连接；估计器健康、模式条件、安全检查和 setpoint 流还要分别满足。

### 题 3：为什么阶段 0 的 setpoint 话题没有 publisher 反而是正确的？

答：因为本阶段不启动控制节点。MAVROS 可以订阅该话题等待未来目标，但此时没有任何节点应发布飞行目标。

### 题 4：为什么同一控制话题不能同时运行两个持续发布者？

答：PX4 会收到互相覆盖甚至跳变的目标，无法知道哪个是唯一任务意图。ROS 允许多个 publisher，但控制系统必须建立唯一控制权。

### 题 5：修改 `.world` 后需要 `catkin_make` 吗？

答：通常不需要；完全停止旧 Gazebo 并重新启动即可。修改 Gazebo 插件 C++ 才需要重新编译对应工作区。

### 题 6：为什么在一个终端 source 了工作空间，另一个终端仍找不到包？

答：环境变量属于进程。`source` 只改变当前 shell 及它之后启动的子进程，不会反向修改其他已存在的终端。

### 题 7：ROS 接口常用 ENU，PX4 内部常用 NED，是否应手工把 MAVROS pose 的 z 取反？

答：不应。MAVROS 已对常用接口做转换，应根据消息的 ROS frame 和接口约定使用。再次转换会造成双重变换。

### 题 8：四元数 `(0,0,0,0)` 能否表示“没有旋转”？

答：不能。无旋转的单位四元数是 `(0,0,0,1)`；全零四元数没有合法的单位旋转含义。

### 题 9：`/use_sim_time=true` 时，暂停 Gazebo 会怎样？

答：`/clock` 停止推进，使用 ROS 仿真时间的定时器和时间判断也会暂停。墙上时间仍然继续。

### 题 10：为什么修改仓库内 launch 后运行效果可能不变？

答：可能运行的是外部 PX4 包里的普通文件副本。要用实际启动路径、`rospack find`、`find` 和 `cmp` 证明程序读取了哪一份。

## 8. 最终验收清单

只有全部勾选后再进入阶段 1：

- [ ] 我能不用 `pc_example.sh` 启动 `roscore` 和 PX4/Gazebo/MAVROS。
- [ ] 我能在不启动控制节点时保持飞机未解锁。
- [ ] `/mavros/state` 中 `connected: true`。
- [ ] `/mavros/local_position/pose` 持续发布，时间戳推进，无 NaN/Inf。
- [ ] 我记录了 pose 的 frame、平均频率和四元数。
- [ ] 我能用 `rostopic info` 说出 publisher 和 subscriber。
- [ ] setpoint 话题没有意外 publisher。
- [ ] 我能解释 node、topic、service、parameter、launch、namespace、TF。
- [ ] 我能解释 PX4、Gazebo、MAVLink、MAVROS 的分工。
- [ ] 我能解释 ENU/NED、世界系/机体系、单位四元数。
- [ ] 我分别启动并确认了 `example.world` 和 `forest.world`。
- [ ] 我记录了两个场景的 RTF 和模型特征。
- [ ] 我能指出仓库 launch、外部 PX4 launch 和外部 SDF 运行副本。
- [ ] 我能按正确顺序停止进程，并确认没有残留。
- [ ] 我能不看本文，自己画出启动链和关键话题图。

## 9. 进入阶段 1 前最后提醒

阶段 1 才会启动：

```bash
roslaunch offboard autoarming_control.launch rviz:=false
```

在那之前，你必须已经能证明：连接正常、pose 有效、setpoint 当前没有其他发布者、Gazebo 未暂停、world 加载正确。阶段 0 建立的是以后每次飞行实验都要重复的“检查清单”，不是只做一次就忘掉的理论课。
