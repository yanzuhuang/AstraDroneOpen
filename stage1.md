# 阶段 1 教程：读懂并复现默认 Offboard 自动飞行闭环

> 适用对象：已经完成 `stage0.md`，第一次阅读 ROS C++ 飞行控制程序的学习者。
>
> 对应路线：`studymap.md` 的“阶段 1：读懂并复现默认 Offboard 闭环”。
>
> 本阶段只在 **PX4 SITL + Gazebo 仿真**中操作。`autoarming_control.launch` 会自动请求 OFFBOARD、自动解锁并让无人机起飞，不能把本文命令直接用于真机。

## 1. 这次任务到底要完成什么

阶段 0 解决的是“系统能不能稳定启动、我能不能看懂通信链”。阶段 1 要解决的是：

> **当 `/autoarming_control` 启动以后，它怎样根据 PX4 的状态和当前位置，持续产生位置目标，完成起飞、圆形或方形轨迹、降落与上锁？**

你不是来“看一遍飞机飞起来”就结束。完成阶段 1 后，你应该能独立做到：

1. 不使用一键脚本，分终端启动 PX4/Gazebo/MAVROS 和 Offboard 控制器。
2. 在启动控制器前完成连接、位姿、发布者、仿真时间等安全检查。
3. 解释 subscriber、publisher、service client、callback、`spinOnce()` 和 20 Hz 主循环在本程序中的作用。
4. 从源码中指出 setpoint 预热、请求 OFFBOARD、请求解锁、起飞、轨迹、降落和上锁的位置。
5. 分清 PX4 的飞行模式 `OFFBOARD` 与轨迹参数 `circle/square`，不把二者混为一谈。
6. 至少三次完整复现“连接—预热—OFFBOARD—解锁—起飞—轨迹—降落—确认上锁”。
7. 分别完成一次圆形和一次方形轨迹，并用话题、日志或 rosbag 留下证据。
8. 说明 `hight`、`target_laps`、`side_length`、`radius` 的真实作用。
9. 用源码和运行数据证明 `speed`、`takeoff_height` 当前不生效。
10. 解释默认控制闭环为什么没有使用 FAST-LIO 的 `/Odometry`。
11. 指出当前代码中的已知缺陷，但本阶段先不急着重写它们。

本阶段最终应提交或保存四类成果：

- 一张默认 Offboard 数据闭环图；
- 一份按源码顺序整理的状态机笔记；
- 至少三次完整飞行记录，其中包含圆形和方形；
- 一张“参数是否生效”的证据表。

## 2. 本阶段的边界与安全规则

### 2.1 只做仿真，不做真机

默认 launch 会主动调用模式切换和解锁服务。本文假定你看到的是 Gazebo 中的虚拟无人机，附近没有真实螺旋桨、电机或真机飞控。

### 2.2 本阶段先不改 C++ 控制逻辑

本阶段允许修改 `autoarming_control.launch` 中的实验参数，但先不要重写 `autoarming_control.cpp`。原因是你需要先看懂当前程序真实做了什么，建立一条可以重复运行的基线；阶段 2 才会实现真正的起飞—悬停—降落改造。

修改 launch 参数后不用 `catkin_make`。只有修改 `.cpp`、头文件、`CMakeLists.txt` 等编译内容后才需要重新编译。

### 2.3 同一时刻只能有一个 setpoint 发布者

本阶段唯一允许持续发布 `/mavros/setpoint_position/local` 的节点是：

```text
/autoarming_control
```

每次起飞前都执行：

```bash
rostopic info /mavros/setpoint_position/local
```

启动控制器前应看到 `Publishers: None`。控制器运行时应只看到 `/autoarming_control`。不要同时运行 `position_control`、另一个 Offboard 节点或手写的 setpoint 发布命令。

### 2.4 不要在空中随意停止控制节点或强制上锁

- 正常实验应让状态机自行进入 `LANDING`。
- 不要在空中调用 disarm/上锁服务。
- 如果仿真中的轨迹明显失控，优先在 QGroundControl 中执行 Land；必要时停止控制节点后让 PX4 的 Offboard failsafe 接管。
- 只有确认模型落地、`armed: false` 后才能开始下一轮。
- 仿真异常到位姿出现 NaN/Inf 时，应终止整套仿真并重新建立基线，不要继续解锁。

### 2.5 本阶段使用空场和保守参数

实践统一使用：

```text
simulation/astra_gazebo_worlds/example.world
```

建议基线值：

| 参数 | 建议值 | 理由 |
|---|---:|---|
| `hight` | `2.0` | 高度足够观察，又比默认 3 m 保守 |
| `target_laps` | `1` | 缩短单次实验时间 |
| `radius` | `2.0` | 圆形尺寸清楚且不太大 |
| `side_length` | `4.0` | 方形顶点位于 `x/y=±2` |
| `speed` | `1.5` | 保留原 launch 值，虽然当前无效 |
| `takeoff_height` | `1.0` | 保留原 launch 值，虽然当前无效 |

禁止使用 `radius=0` 或 `side_length=0`。当前代码会用轨迹总长度做除数，零尺寸存在除零风险。也不要使用负高度、负半径或非正圈数。

## 3. 先看懂完整控制闭环

### 3.1 默认数据流

```text
Gazebo 中的无人机实际运动
          │
          ▼
       PX4 SITL
          │ 当前飞行状态、局部位置
          ▼
        MAVROS
          │
          ├── /mavros/state ─────────────────────┐
          │   connected / mode / armed           │
          │                                      ▼
          └── /mavros/local_position/pose ─> /autoarming_control
                                                   │
                                                   │ 状态机 + 轨迹函数
                                                   ▼
                                  /mavros/setpoint_position/local
                                                   │ 目标位置
                                                   ▼
                                                MAVROS
                                                   │ MAVLink
                                                   ▼
                                                  PX4
                          位置环 -> 速度环 -> 姿态/角速度环 -> 电机输出
                                                   │
                                                   ▼
                                                Gazebo
```

这就是一个闭环：

1. 控制节点读取“实际位置”；
2. 控制节点算出“目标位置”；
3. PX4 根据目标与实际之间的误差控制虚拟无人机；
4. 新的实际位置再次反馈给控制节点。

`autoarming_control.cpp` 是任务层和位置目标生成器，不是电机控制器。它不会直接计算四个电机各转多少，也没有重写 PX4 的姿态内环。

### 3.2 FAST-LIO 为什么不在这条闭环里

一键脚本会另外启动 FAST-LIO，但默认控制器订阅的是：

```text
/mavros/local_position/pose
```

它没有订阅 FAST-LIO 输出的：

```text
/Odometry
```

所以即使 `/laserMapping` 和 `/Odometry` 正常存在，当前默认 Offboard 飞行仍然使用 MAVROS/PX4 提供的局部位姿。阶段 1 为了减少干扰，不需要启动 FAST-LIO。

### 3.3 两种容易混淆的“模式”

| 名称 | 可能的值 | 属于谁 | 作用 |
|---|---|---|---|
| PX4 飞行模式 | `OFFBOARD` 等 | `/mavros/state.mode` | 决定 PX4 是否接受外部 setpoint |
| 轨迹类型参数 | `circle`、`square` | `/flight_mode` | 决定控制节点生成圆还是方形 |

因此下面两句话含义完全不同：

- “PX4 已进入 OFFBOARD”：PX4 开始接受外部目标。
- “`flight_mode` 是 circle”：控制节点选择圆轨迹公式。

## 4. 必须掌握的 ROS 与 C++ 概念

### 4.1 subscriber 与 callback：接收最新状态

源码中的两个 subscriber 是：

```cpp
ros::Subscriber state_sub =
    nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

ros::Subscriber pose_sub =
    nh.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 10, pose_cb);
```

它们分别订阅状态和位置。新消息到来后，对应 callback 把整条消息复制到全局变量：

```cpp
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}
```

可以把它理解为：

```text
MAVROS 新消息到来
       │
       ▼
ROS 回调队列
       │ ros::spinOnce()
       ▼
执行 callback
       │
       ├──更新 current_state
       └──更新 current_pose
```

`ros::spinOnce()` 处理当前回调队列，然后立刻返回，所以主循环还能继续计算和发布目标。如果完全不调用它，`current_state` 和 `current_pose` 就不会随新消息更新，控制程序会一直使用旧数据。

订阅时的 `10` 是消息队列长度，不是 10 Hz。真正的消息频率由发布者决定。

### 4.2 publisher：持续发送位置设定点

```cpp
ros::Publisher local_pos_pub =
    nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);
```

publisher 发布的消息类型是 `geometry_msgs/PoseStamped`。它最重要的字段是：

```text
header.stamp             这条目标的时间戳
header.frame_id          目标属于哪个坐标系
pose.position.x/y/z      目标位置
pose.orientation.x/y/z/w 目标姿态四元数
```

当前代码的新目标会把 `frame_id` 写成 `map`，并不断更新时间戳。位置目标通过 MAVROS 交给 PX4，由 PX4 内部控制器跟踪。

注意：当前代码在 TAKEOFF/TRACKING/LANDING 新建的 `PoseStamped` 中没有显式设置 orientation，因此默认四元数为 `(0,0,0,0)`，它不是合法单位四元数。这是阶段 2 应修复的缺陷之一。本阶段先记录现状，不要误以为“全零就是没有旋转”；没有旋转的合法四元数应为 `(0,0,0,1)`。

### 4.3 service client：发出一次请求

程序创建三个 service client：

| client | ROS 服务 | 消息类型 | 用途 |
|---|---|---|---|
| `set_mode_client` | `/mavros/set_mode` | `mavros_msgs/SetMode` | 请求 PX4 进入 OFFBOARD |
| `arming_client` | `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | 请求解锁 |
| `land_client` | `/mavros/cmd/command` | `mavros_msgs/CommandLong` | 发送 MAVLink 400 上锁命令 |

topic 与 service 在这里的分工非常清楚：

- 目标位置必须连续发送，所以用 topic；
- 切模式和解锁是一次请求并返回是否受理，所以用 service。

但必须记住：

> **service 返回“请求发送/受理成功”，不等于状态已经改变。最终状态要从 `/mavros/state` 再次确认。**

例如 `offb_set_mode.response.mode_sent=true` 后，仍要等 `/mavros/state.mode` 真正变为 `OFFBOARD`。解锁和上锁也应以 `armed` 字段为准。

### 4.4 为什么主循环是 20 Hz

```cpp
ros::Rate rate(20.0);
```

循环末尾调用：

```cpp
ros::spinOnce();
rate.sleep();
```

理想情况下，每秒运行 20 次，即每 0.05 秒一次。每一轮主要做三件事：

1. 处理新的 state/pose 回调；
2. 根据当前阶段计算目标；
3. 发布一个新的位置 setpoint。

PX4 的 OFFBOARD 需要持续收到目标，不能只发一次“飞到 `(2,0,2)`”就退出。如果目标流中断，PX4 会根据 failsafe 配置退出 Offboard 或执行保护动作。

本机使用 `/use_sim_time=true` 时，`ros::Rate` 跟随仿真时钟。Gazebo 暂停后 ROS 时间也会暂停；实时因子低时，墙上时钟观察到的频率和等待时间可能变慢。因此 `rostopic hz` 不一定精确显示 20.000 Hz，但应该稳定、连续，不能长时间断流。

### 4.5 OFFBOARD 前为什么先发 100 个目标

源码在请求 OFFBOARD 前执行：

```cpp
for (int i = 100; ros::ok() && i > 0; --i) {
    geometry_msgs::PoseStamped hold = current_pose;
    // 更新时间戳和 frame 后发布
    local_pos_pub.publish(hold);
    ros::spinOnce();
    rate.sleep();
}
```

100 帧 ÷ 20 Hz = 约 5 秒仿真时间。这里发布的是“保持当前位姿”的目标，而不是立刻起飞。

预热的作用是让 PX4 在切入 OFFBOARD 前已经看到稳定、连续的外部 setpoint 流。否则模式请求可能被拒绝，或刚切入就因目标流不满足条件而退出。

当前程序预热完成后又把 `last_request` 设为当前时间，并等待超过 5 秒才第一次请求 OFFBOARD；请求成功后通常还会再等待超过 5 秒才请求解锁。因此从 FCU 已连接到真正解锁，常常不止预热的 5 秒。不要因为飞机十几秒没有起飞就立刻判断程序卡死，应该看日志和 `/mavros/state`。

### 4.6 全局参数与私有参数

代码创建两个 NodeHandle：

```cpp
ros::NodeHandle nh;
ros::NodeHandle nh_private("~");
```

然后分别读取：

```cpp
nh.param<std::string>("flight_mode", flight_mode, "square");
nh_private.param("hight", hight, 3.0);
nh_private.param("target_laps", target_laps, 1);
nh_private.param("side_length", side_length, 8.0);
nh_private.param("radius", radius, 2.0);
```

在当前 launch 命名下，最终参数名是：

| C++ 写法 | 当前解析后的名字 |
|---|---|
| `nh.param("flight_mode", ...)` | `/flight_mode` |
| `nh_private.param("hight", ...)` | `/autoarming_control/hight` |
| `nh_private.param("target_laps", ...)` | `/autoarming_control/target_laps` |
| `nh_private.param("side_length", ...)` | `/autoarming_control/side_length` |
| `nh_private.param("radius", ...)` | `/autoarming_control/radius` |

这里的 `hight` 是源码原本的拼写，不是本文笔误。现阶段必须按 `hight` 使用；写成 `height` 不会被当前代码读取。

`param` 的最后一个参数是“找不到 ROS 参数时采用的代码默认值”。当前 launch 没有设置 `radius`，所以程序会在内部使用 `2.0`，但参数服务器上不一定能查到 `/autoarming_control/radius`。为了做阶段 1 实验，后文会让你在 launch 中补一行 `radius`。

还有一个非常重要的规则：参数只在节点启动时读取一次。节点运行以后执行 `rosparam set`，当前程序不会自动重新读取。改变实验参数后必须重启这次控制 launch。

### 4.7 launch 中“存在参数”不代表 C++ 使用参数

当前 launch 写了：

```xml
<param name="takeoff_height" value="1.0"/>
<param name="speed" value="1.5"/>
```

但 `autoarming_control.cpp` 中没有 `takeoff_height`、`speed` 变量，也没有读取它们的 `param(...)`，后续计算更没有使用它们。

所以：

```text
rosparam get 能查到
        ≠
C++ 已读取
        ≠
控制算法真的使用
        ≠
实际飞行一定发生变化
```

判断参数是否生效需要形成完整证据链：

1. launch 是否把参数放进参数服务器；
2. C++ 是否读取参数；
3. 读出的变量是否参与目标计算或状态判断；
4. 运行中的 setpoint 和实际轨迹是否随它改变。

### 4.8 节点名为什么不是源码中的 `offb_node`

源码写了：

```cpp
ros::init(argc, argv, "offb_node");
```

launch 又写了：

```xml
<node pkg="offboard"
      type="autoarming_control"
      name="autoarming_control" ...>
```

通过 launch 启动时，launch 指定的名字会覆盖默认名，所以 `rosnode list` 中看到的是：

```text
/autoarming_control
```

这里还要分清：

- package：`offboard`；
- 可执行文件 type：`autoarming_control`；
- 运行时 node name：`/autoarming_control`；
- 源码默认名：`offb_node`，当前被 launch 覆盖。

## 5. 按源码顺序读懂 `autoarming_control.cpp`

本节对应文件：

```text
AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/
└── src/autoarming_control.cpp
```

第一次阅读时不要从轨迹公式中间跳着看。请严格沿下面的顺序，在自己的笔记里给每一段写一句“输入—处理—输出”。

### 5.1 消息头文件和全局状态：第 6～18 行

主要消息：

| 消息 | 关键字段 | 用途 |
|---|---|---|
| `mavros_msgs/State` | `connected/mode/armed` | 读取 PX4 状态 |
| `geometry_msgs/PoseStamped` | header、position、orientation | 读取实际位姿并发布目标位姿 |
| `mavros_msgs/SetMode` | `custom_mode` | 请求 OFFBOARD |
| `mavros_msgs/CommandBool` | `value` | 请求 arm/disarm |
| `mavros_msgs/CommandLong` | command、param1～7 | 发送通用 MAVLink 命令 |

全局变量 `current_state`、`current_pose` 由 callback 更新，主循环读取。`initial_height` 用来保存程序认为的初始地面高度。

### 5.2 FlightPhase：第 20～26 行

```text
TAKEOFF -> TRACKING -> LANDING -> COMPLETED
```

`FlightPhase` 是控制程序自己的任务状态，不是 PX4 的 flight mode。当前没有独立 `HOVER` 状态，所以达到目标高度后会立即进入轨迹阶段。

### 5.3 callback 和距离函数：第 28～41 行

`calculate_distance()` 计算两个三维位置之间的欧氏距离：

```text
d = sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)
```

TRACKING 中用它判断无人机离当前目标点有多远。

### 5.4 方形轨迹函数：第 43～56 行

`t` 是归一化轨迹进度，一圈内从 `0` 增长到 `1`。边长为 `L` 时：

| `t` | 目标点 |
|---:|---|
| `0.00` | `(-L/2, -L/2)` |
| `0.25` | `( L/2, -L/2)` |
| `0.50` | `( L/2,  L/2)` |
| `0.75` | `(-L/2,  L/2)` |
| `1.00` | `(-L/2, -L/2)` |

四条边围绕世界原点 `(0,0)`，总长是 `4L`。例如 `side_length=4` 时，理论顶点是：

```text
(-2,-2) -> (2,-2) -> (2,2) -> (-2,2) -> (-2,-2)
```

注意，TAKEOFF 的目标是 `(0,0,hight)`，TRACKING 的第一个方形目标却是 `(-L/2,-L/2,hight)`。因此无人机先会从方形中心附近斜着飞向第一个角，这段接入轨迹不属于方形四条边。

### 5.5 圆形轨迹函数：第 58～63 行

圆的解析式为：

```text
angle = 2πt
x = r cos(angle)
y = r sin(angle)
```

关键位置：

| `t` | 目标点 |
|---:|---|
| `0.00` | `( r, 0)` |
| `0.25` | `( 0, r)` |
| `0.50` | `(-r, 0)` |
| `0.75` | `( 0,-r)` |
| `1.00` | `( r, 0)` |

圆心也是世界原点。起飞结束时目标在 `(0,0,hight)`，圆轨迹第一点是 `(radius,0,hight)`，所以会先从圆心附近飞向圆周。

### 5.6 Lock()：第 65～80 行

这里通过 `/mavros/cmd/command` 发送 command 400，`param1=0` 表示上锁，`param2=21196` 是强制 arm/disarm 的确认值。

本函数存在一个需要牢记的验证缺口：

1. `Lock()` 没有返回 bool；
2. service 失败时主状态机不知道；
3. 主循环无论是否真正上锁，后面都会进入 `COMPLETED`；
4. `[DONE] Landing completed and disarmed` 日志因此不能单独证明 `armed=false`。

所以每轮结束必须执行：

```bash
rostopic echo -n 1 /mavros/state
```

### 5.7 ROS 接口和参数：第 82～106 行

这一段完成：

```text
初始化节点
  -> 读取参数
  -> 创建两个 subscriber
  -> 创建一个 publisher
  -> 创建三个 service client
  -> 设置主循环 20 Hz
```

你应该能不看本文，自己画出这些 ROS 接口的方向。

### 5.8 等待连接与记录初始高度：第 108～115 行

程序只等待：

```cpp
current_state.connected == true
```

然后立刻执行：

```cpp
initial_height = current_pose.pose.position.z;
```

这里没有单独等待第一帧有效 pose。`connected=true` 不严格保证 `pose_cb` 已经执行过，所以 `initial_height` 可能仍是全局对象的默认值 `0`。默认出生点接近地面且 z 接近 0 时问题可能不明显，但这是阶段 2 必须修复的时序风险。

### 5.9 setpoint 预热：第 117～125 行

程序以 20 Hz 发送 100 个“保持当前位置”的 setpoint。此时未请求 OFFBOARD、未请求解锁，飞机不应起飞。

### 5.10 模式与解锁请求：第 127～163 行

程序先准备：

```text
custom_mode = OFFBOARD
arm_cmd.value = true
```

主循环中每隔 5 秒检查：

```text
如果 mode 不是 OFFBOARD
    -> 请求 OFFBOARD
否则如果还没有 armed
    -> 请求解锁
```

因为使用 `if ... else if ...`，同一轮不会同时发送两个请求。切模式请求后还会等待下一次 5 秒间隔，再尝试解锁。

### 5.11 未解锁时持续保持：第 165～186 行

只要 `armed=false`，程序就复制当前位置作为 hold setpoint，持续发布，然后 `continue` 回到下一轮。这保证等待 OFFBOARD 和解锁期间不会断掉 setpoint 流，也不会提前执行 TAKEOFF。

### 5.12 TAKEOFF：第 192～204 行

起飞目标固定为：

```text
x = 0
y = 0
z = hight
```

阶段转换条件只有：

```text
abs(current_z - hight) < 0.1 m
```

这带来三个结论：

1. `hight` 是世界/局部坐标中的绝对 z 目标，不是“相对初始高度再上升多少”。
2. 起飞点不在原点时，程序还会让飞机在水平方向飞向 `(0,0)`。
3. 转入 TRACKING 时只检查 z 误差，没有检查 x/y 是否到达原点。

`takeoff_height` 在这里完全没有出现，所以当前不生效。

### 5.13 TRACKING：第 205～252 行

先根据轨迹类型求目标点，再计算当前位置到目标点的三维距离 `d`。只有 `d<1.0` 时才推进 `t_target`：

```text
advance = 1.0 - d
delta_t = advance / trajectory_length
t_target += delta_t
```

例子：圆半径 `r=2`，总长约 `12.57 m`。若当前距离目标 `d=0.8 m`：

```text
advance = 0.2 m
delta_t = 0.2 / 12.57 ≈ 0.0159
```

下一目标沿轨迹前进约 0.2 m。若飞机离目标超过或等于 1 m，轨迹进度暂时不推进，等飞机靠近。

这是一种根据跟踪距离推进目标的简单逻辑，不是“按 `speed` 米每秒积分”。代码没有使用时间增量 `dt`，也没有读取 `speed`，所以现在无法通过 `speed` 参数直接设定轨迹速度。飞行快慢会受 PX4 响应、20 Hz 循环、距离阈值和仿真实时因子等共同影响。

当 `t_target>=1.0` 时，一圈完成：

```text
t_target -= 1
completed_laps += 1
```

达到 `target_laps` 后切换 LANDING。

另一个容易漏掉的行为是：代码只明确判断 `flight_mode == "square"`；任何其他字符串都会走圆形分支。拼成 `sqaure` 不会报错，而会静默飞圆形。这是当前代码的输入校验缺陷。

### 5.14 LANDING 与 COMPLETED：第 253～272 行

降落目标固定为：

```text
x = 0
y = 0
z = initial_height
```

所以它不是严格意义上的“返回记录下来的起飞点”，只使用了初始 z，没有保存初始 x/y。

当：

```text
current_z - initial_height <= 0.10 m
```

程序调用 `Lock()`，立即把阶段改为 `COMPLETED`，主循环退出。这里同样没有确认水平误差、接地状态、垂直速度或实际上锁结果。

## 6. 把状态机画成你能口述的流程

```text
节点启动
  │
  ▼
等待 current_state.connected == true
  │
  ▼
记录 initial_height（当前未保证 pose 已有效）
  │
  ▼
以 20 Hz 预发 100 个当前位置 setpoint
  │
  ▼
每隔 5 s 请求 OFFBOARD
  │  /mavros/state.mode 尚未确认
  ├───────────────────────────────┐
  │确认 mode == OFFBOARD          │继续发布 hold setpoint
  ▼                               │然后重试
每隔 5 s 请求 Arm                 │
  │  /mavros/state.armed 尚未确认 │
  ├───────────────────────────────┘
  │确认 armed == true
  ▼
TAKEOFF：发布 (0,0,hight)
  │ abs(current_z-hight) < 0.1
  ▼
TRACKING：圆或方形目标
  │ completed_laps >= target_laps
  ▼
LANDING：发布 (0,0,initial_height)
  │ current_z-initial_height <= 0.1
  ▼
发送上锁命令
  │
  ▼
COMPLETED 并退出
  │
  └──实验者必须另外确认 /mavros/state.armed == false
```

你还应该能说出每条关键边对应的证据：

| 转换 | 代码判断 | 运行证据 |
|---|---|---|
| 等待连接结束 | `current_state.connected` | `/mavros/state connected: true` |
| 进入 OFFBOARD | `current_state.mode` | `/mavros/state mode: OFFBOARD` |
| 开始 TAKEOFF | `current_state.armed` | `/mavros/state armed: true` |
| TAKEOFF -> TRACKING | z 误差 `<0.1` | 日志 + pose/setpoint z |
| TRACKING -> LANDING | 圈数达到目标 | `[TRACK] Completed lap ...` |
| LANDING -> COMPLETED | 高度差 `<=0.1` | LAND 日志，但仍需检查 armed |

## 7. 仿真实践前的文件准备

### 7.1 本阶段重点文件

按顺序阅读：

1. `offboard/launch/autoarming_control.launch`：节点、参数和 RViz 开关；
2. `offboard/src/autoarming_control.cpp`：ROS 接口与状态机；
3. `offboard/CMakeLists.txt`：如何生成 `autoarming_control` 可执行文件；
4. `offboard/package.xml`：功能包依赖；
5. `scripts/run_sh/pc_example.sh`：理解一键脚本为什么不适合本阶段做变量控制。

本文中的 `offboard/` 完整路径是：

```text
AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/
```

### 7.2 把 launch 改成保守的一圈基线

用编辑器打开：

```text
AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/
launch/autoarming_control.launch
```

把参数区整理为：

```xml
<!-- square or circle -->
<param name="flight_mode" value="circle" />

<node pkg="offboard" type="autoarming_control"
      name="autoarming_control" output="screen">
    <param name="target_laps" value="1"/>
    <param name="hight" value="2.0"/>
    <param name="takeoff_height" value="1.0"/>
    <param name="side_length" value="4.0"/>
    <param name="radius" value="2.0"/>
    <param name="speed" value="1.5"/>

    <!-- 原有 remap 保持不变 -->
</node>
```

这里唯一新增的是 `radius` 参数。它能让当前实际使用的圆半径明确出现在 launch 和参数服务器中。

不要尝试这样切换轨迹：

```bash
# 当前 launch 没有定义 flight_mode 这个 arg，因此不要这样用：
roslaunch offboard autoarming_control.launch flight_mode:=square
```

当前文件只有 `rviz` 是 `<arg>`，`flight_mode` 是写死的 `<param>`。要切换圆/方形，现阶段直接编辑 `<param name="flight_mode" ...>`，保存并重新启动控制 launch。

### 7.3 是否需要编译

只改上面的 `.launch`：

```text
不需要 catkin_make
```

如果你只是想确认现有 C++ 已经能被找到，执行：

```bash
source "$HOME/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash" --extend
rospack find offboard
roslaunch --nodes offboard autoarming_control.launch rviz:=false
```

预期至少列出：

```text
/map_to_camera_init
/autoarming_control
```

`--nodes` 只解析并列出节点，不会真正起飞。

## 8. 仿真实践：第一次完整圆形飞行

### 8.1 终端分工

建议准备五个终端：

| 终端 | 用途 |
|---|---|
| 1 | `roscore` |
| 2 | PX4 SITL + Gazebo + MAVROS |
| 3 | 状态、参数、频率等检查 |
| 4 | rosbag 录制 |
| 5 | 启动 `/autoarming_control` 并观察状态机日志 |

每个新终端先执行：

```bash
source ~/.bashrc
source "$HOME/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash" --extend
cd "$HOME/AstraDroneOpen"
```

如果第二条 source 报文件不存在，说明工作空间还没有成功编译，先回到 `stage0.md` 的环境检查部分解决，不要跳过。

### 8.2 终端 1：启动 roscore

```bash
roscore
```

保持终端运行。

### 8.3 终端 2：只启动 PX4/Gazebo/MAVROS

```bash
roslaunch \
  "$HOME/AstraDroneOpen/simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch" \
  world:="$HOME/AstraDroneOpen/simulation/astra_gazebo_worlds/example.world"
```

看到 Gazebo 中的 `iris_mid360` 后先不要启动控制器。等 MAVROS 和局部位姿稳定。

### 8.4 终端 3：逐项执行起飞前检查

#### 检查 1：仿真没有暂停

确认 Gazebo 底部时间持续增加，再执行：

```bash
rosparam get /use_sim_time
rostopic echo -n 1 /clock
```

应看到 `/use_sim_time` 为 `true`，且重复查看 `/clock` 时数值会前进。

#### 检查 2：FCU 已连接且未解锁

```bash
rostopic echo -n 1 /mavros/state
```

最低要求：

```yaml
connected: true
armed: false
```

#### 检查 3：局部位姿有效

```bash
rostopic echo -n 1 /mavros/local_position/pose
rostopic hz /mavros/local_position/pose
```

观察至少 5～10 秒后按 `Ctrl+C` 停止 `hz`。位置不能出现 NaN/Inf，时间戳必须推进。

#### 检查 4：没有旧控制发布者

```bash
rostopic info /mavros/setpoint_position/local
```

启动控制器前必须是：

```text
Publishers: None
```

如果不是，先找出并停止旧节点：

```bash
rosnode info /实际发布者节点名
```

#### 检查 5：三个服务存在

```bash
rosservice info /mavros/set_mode
rosservice info /mavros/cmd/arming
rosservice info /mavros/cmd/command
```

本阶段不要手动调用它们，控制程序会按状态机调用。

只有五项都通过才继续。

### 8.5 终端 4：开始录制最小 rosbag

```bash
mkdir -p "$HOME/AstraDroneOpen/stage1_records"
cd "$HOME/AstraDroneOpen/stage1_records"

rosbag record -O run01_circle_baseline.bag \
  /mavros/state \
  /mavros/local_position/pose \
  /mavros/local_position/odom \
  /mavros/setpoint_position/local
```

先开始录包，再启动控制器，这样能保存 OFFBOARD 和 armed 的变化。保持该终端运行。

### 8.6 终端 5：启动默认控制器

```bash
source ~/.bashrc
source "$HOME/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash" --extend

roslaunch offboard autoarming_control.launch rviz:=false
```

`rviz:=false` 只是不打开 RViz，不影响控制节点、参数和静态 TF。

从这一刻开始不要只盯 Gazebo。按顺序寻找这些日志：

```text
FCU connected
[PREHEAT] Sending current position setpoint before OFFBOARD...
[ACTION] Attempting to set OFFBOARD mode...
[OK] OFFBOARD mode request sent
[WAIT] OFFBOARD active: waiting for arming...
[ACTION] Attempting to arm vehicle...
[OK] Vehicle armed
[TAKEOFF] ...
[PHASE] Reached takeoff altitude, switching to TRACKING
[TRACK] ...
[TRACK] Completed lap 1/1
[PHASE] All laps done, switching to LANDING
[LAND] ...
[DONE] Landing completed and disarmed
```

实际日志中间可能重复，等待时间也会受实时因子影响。你的任务是识别顺序和条件，不是要求每行一模一样。

### 8.7 控制器运行期间做四项观察

在终端 3 依次执行。某些命令会持续输出，用 `Ctrl+C` 结束该检查，不会影响飞行节点。

#### 观察 A：状态真的变化了吗

```bash
rostopic echo /mavros/state
```

亲眼确认大致变化：

```text
connected=true, mode 非 OFFBOARD, armed=false
  -> mode=OFFBOARD, armed=false
  -> mode=OFFBOARD, armed=true
  -> 最后 armed=false
```

#### 观察 B：setpoint 发布者只有一个

```bash
rostopic info /mavros/setpoint_position/local
```

应只显示：

```text
Publishers:
 * /autoarming_control
```

#### 观察 C：setpoint 连续发布

```bash
rostopic hz /mavros/setpoint_position/local
```

期望接近 20 Hz。记录平均值和范围，不要求严格等于 20.000。

#### 观察 D：目标与实际位置的区别

```bash
rostopic echo /mavros/setpoint_position/local
```

另开一个检查终端可同时看：

```bash
rostopic echo /mavros/local_position/pose
```

你应该看到：setpoint 是“希望到达哪里”，local pose 是“现在实际在哪里”。两者不应被误认为同一个话题；实际位置通常带有跟踪误差和动态滞后。

如果喜欢图形化观察，可运行：

```bash
rqt_plot \
  /mavros/local_position/pose/pose/position/z \
  /mavros/setpoint_position/local/pose/position/z
```

### 8.8 飞行结束后不要立刻开始下一轮

看到 `[DONE]` 后，在终端 3 执行：

```bash
rostopic echo -n 1 /mavros/state
rostopic info /mavros/setpoint_position/local
```

合格结果：

```yaml
armed: false
```

控制节点退出后，setpoint 话题的 publisher 应消失。若日志说 DONE 但仍 `armed: true`，本轮不能判定成功，也不要重新启动另一个控制器。先在 Gazebo/QGC 确认已经落地，再通过 QGC 安全上锁；随后完整重启仿真并记录这一异常。

### 8.9 停止录包并整理本轮证据

在终端 4 按 `Ctrl+C`，然后：

```bash
cd "$HOME/AstraDroneOpen/stage1_records"
rosbag info run01_circle_baseline.bag
```

检查四个话题都有消息，bag 时长覆盖控制节点启动至最终上锁。

可以导出目标和实际位姿为 CSV：

```bash
rostopic echo -b run01_circle_baseline.bag -p \
  /mavros/setpoint_position/local > run01_setpoint.csv

rostopic echo -b run01_circle_baseline.bag -p \
  /mavros/local_position/pose > run01_actual_pose.csv
```

CSV 可用表格软件打开。阶段 1 只要求能看出目标 z、圆形 x/y 的大致范围和实际位置跟随；严格误差统计留到阶段 5。

### 8.10 关闭本轮控制 launch

如果控制节点已经退出，终端 5 中的静态 TF 节点可能仍使 roslaunch 保持运行。在终端 5 按 `Ctrl+C` 结束本轮控制 launch。

终端 2 的 PX4/Gazebo/MAVROS 可以继续运行，但开始下一轮前再次确认：

```bash
rostopic echo -n 1 /mavros/state
rostopic info /mavros/setpoint_position/local
```

必须是已上锁、无 setpoint 发布者。如果模型已经安全落地但位置需要复位，可以在 Gazebo 使用：

```text
Edit -> Reset Model Poses
```

不要使用 `Reset World`，因为它会重置仿真时间。

## 9. 第二次实践：方形轨迹

### 9.1 只改两个与本轮有关的参数

编辑 `autoarming_control.launch`：

```xml
<param name="flight_mode" value="square" />
...
<param name="target_laps" value="1"/>
<param name="hight" value="2.0"/>
<param name="side_length" value="4.0"/>
<param name="radius" value="2.0"/>
```

`radius` 在 square 模式下不会参与目标计算，但保留基线值。保存后不需要编译。

### 9.2 开始第二个 bag

```bash
cd "$HOME/AstraDroneOpen/stage1_records"

rosbag record -O run02_square_baseline.bag \
  /mavros/state \
  /mavros/local_position/pose \
  /mavros/local_position/odom \
  /mavros/setpoint_position/local
```

然后在控制终端重新启动：

```bash
roslaunch offboard autoarming_control.launch rviz:=false
```

### 9.3 本轮必须回答的问题

1. 起飞后为什么先斜着飞向 `(-2,-2,2)` 附近？
2. 四个理论角点分别是什么？
3. 为什么实际轨迹的拐角可能比目标轨迹圆滑？
4. `side_length=4` 表示从 `x=-2` 到 `x=2`，还是从 `x=0` 到 `x=4`？
5. 切换为 square 后，PX4 的 `/mavros/state.mode` 是否也变成 `square`？

正确理解：轨迹类型只改变 setpoint 生成公式；PX4 飞行模式仍然应该是 `OFFBOARD`。实际拐角会受连续目标、PX4 动力学、惯性和跟踪误差影响，不一定是数学上无限尖锐的直角。

本轮结束后重复执行“确认 `armed:false`—停止录包—`rosbag info`—关闭控制 launch”。

## 10. 第三次实践：验证可重复性

第三轮任选 circle 或 square，但不要改任何参数，重复上一轮完整过程。bag 命名例如：

```text
run03_square_repeat.bag
```

“可重复”不是三次 Gazebo 窗口都打开，而是三次都满足：

1. 启动前 connected、pose、publisher 检查通过；
2. mode 真正进入 OFFBOARD；
3. armed 真正变为 true；
4. 完成目标圈数；
5. 进入 LANDING；
6. 最终 `/mavros/state.armed=false`；
7. bag 中四个关键话题完整；
8. 没有第二个 setpoint 发布者。

如果第三轮失败，不要把失败隐藏掉。保留失败 bag 和终端报错，按第 13 节排查，解决后再补一次成功运行。

## 11. 参数是否生效：用控制变量做实验

### 11.1 先建立源码证据

在项目根目录执行：

```bash
rg -n 'param|flight_mode|hight|target_laps|side_length|radius|speed|takeoff_height' \
  AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src/autoarming_control.cpp \
  AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/launch/autoarming_control.launch
```

你应该发现：

| 参数 | launch 中存在 | C++ 读取 | 参与计算/判断 | 当前是否有效 |
|---|---:|---:|---:|---:|
| `flight_mode` | 是 | 是 | 是，选择轨迹分支 | 是 |
| `hight` | 是 | 是 | 是，目标 z | 是 |
| `target_laps` | 是 | 是 | 是，结束圈数 | 是 |
| `side_length` | 是 | 是 | 是，square 轨迹 | 是，仅 square |
| `radius` | 原本否，实验中新增 | 是 | 是，circle 轨迹 | 是，仅 circle |
| `speed` | 是 | 否 | 否 | 否 |
| `takeoff_height` | 是 | 否 | 否 | 否 |

### 11.2 实验原则：一次只改变一个量

如果同时把半径、高度、圈数都改了，即使飞行不同，你也说不清是谁造成的。每轮都从下面的 circle 基线恢复，再只改目标参数：

```text
flight_mode=circle
hight=2.0
target_laps=1
radius=2.0
side_length=4.0
speed=1.5
takeoff_height=1.0
```

每轮都使用同一个 `example.world`、同一机型和同样的启动方式。

### 11.3 推荐的最小参数实验矩阵

你已经完成的 `run01_circle_baseline` 是对照组。继续做：

| 运行 | 只改变什么 | 期望从哪里看到效果 |
|---|---|---|
| `run04_hight_3` | `hight: 2.0 -> 3.0` | TAKEOFF/TRACKING setpoint z 变为 3 |
| `run05_laps_2` | `target_laps: 1 -> 2` | 日志出现 `1/2`、`2/2` 后才降落 |
| `run06_radius_3` | `radius: 2.0 -> 3.0` | circle 目标 x/y 理论范围由 ±2 变 ±3 |
| `run07_speed_4` | `speed: 1.5 -> 4.0` | 参数值改变，但源码/setpoint 推进公式不变 |
| `run08_takeoff_height_4` | `takeoff_height: 1.0 -> 4.0` | 参数值改变，但起飞目标 z 仍由 `hight=2` 决定 |
| `run09_square_side_6` | square 下 `side_length: 4.0 -> 6.0` | 方形理论顶点由 ±2 变 ±3 |

每次修改后：

1. 保存 launch；
2. 重新启动控制 launch，使参数重新读取；
3. 立即用 `rosparam get` 记录当前值；
4. 记录同名 bag；
5. 完整飞行并确认上锁；
6. 恢复基线，再进行下一项。

### 11.4 运行时核对参数

控制 launch 启动后执行：

```bash
rosparam get /flight_mode
rosparam get /autoarming_control/hight
rosparam get /autoarming_control/target_laps
rosparam get /autoarming_control/side_length
rosparam get /autoarming_control/radius
rosparam get /autoarming_control/speed
rosparam get /autoarming_control/takeoff_height
```

如果 `radius` 报参数不存在，说明你没有按第 7.2 节把它加入 launch；程序仍可能使用 C++ 默认值 2.0，但这次实验的配置证据不完整。

不要在节点已经运行后用 `rosparam set` 期待轨迹立即改变。当前节点没有动态参数回调，只在启动时读一次。

### 11.5 怎样证明 `hight` 生效

对比 `run01_circle_baseline.bag` 和 `run04_hight_3.bag`：

```bash
rostopic echo -b run01_circle_baseline.bag -p \
  /mavros/setpoint_position/local > run01_setpoint.csv

rostopic echo -b run04_hight_3.bag -p \
  /mavros/setpoint_position/local > run04_setpoint.csv
```

预热阶段的目标 z 是当时当前位置，解锁后的起飞/轨迹目标 z 才应分别稳定在约 2 和 3。不要把预热数据误当成飞行目标。

代码证据是 TAKEOFF、TRACKING 都把 `pose.position.z` 设为 `hight`。

### 11.6 怎样证明 `target_laps` 生效

对照终端日志：

```text
target_laps=1:
Completed lap 1/1 -> LANDING

target_laps=2:
Completed lap 1/2 -> 继续 TRACKING
Completed lap 2/2 -> LANDING
```

bag 时长一般也会增加，但时长只是辅助证据；最直接的是 completed_laps 的代码判断和日志阶段转换。

### 11.7 怎样证明 `radius` 生效

圆公式给出：

```text
x^2 + y^2 = radius^2
```

因此 `radius=2` 时 setpoint 理论范围是 `x/y ∈ [-2,2]`，`radius=3` 时是 `[-3,3]`。查看的是 setpoint，而不是要求实际 pose 每一帧都精确落在圆上。

### 11.8 怎样证明 `side_length` 生效

它只在 `flight_mode=square` 时有效：

```text
side_length=4 -> 顶点 x/y 为 ±2
side_length=6 -> 顶点 x/y 为 ±3
```

如果你在 circle 模式修改 `side_length` 后看不到变化，这是正常的分支行为，不能据此说该参数“完全无效”。

### 11.9 怎样证明 `speed` 当前无效

证据要写成三层：

1. 参数服务器能查到 `/autoarming_control/speed`；
2. `autoarming_control.cpp` 没有读取 `speed`；
3. 轨迹推进使用 `(1-d)/trajectory_length`，没有 `speed` 和 `dt`。

`run07_speed_4` 的实际飞行可作为运行佐证，但不要只凭两次飞行耗时接近就下结论，因为实时因子和跟踪误差会造成波动。源码数据流已经从根本上证明这个值没有进入控制算法。

### 11.10 怎样证明 `takeoff_height` 当前无效

同样使用三层证据：

1. 参数服务器能查到该值；
2. C++ 没有读取它；
3. TAKEOFF 直接使用 `pose.pose.position.z = hight`。

在 `run08_takeoff_height_4` 中保持 `hight=2.0`，即使把 `takeoff_height` 改为 `4.0`，起飞 setpoint z 仍是 2.0。这里能非常直观地区分“launch 中有这个参数”和“代码真的使用它”。

## 12. 实验记录模板

把下面内容复制到 `studynote.md` 或单独的记录文件。每一轮都填，不要只在成功时填。

```markdown
## Stage 1 - Run XX

- 日期与时间：
- bag 文件：
- 是否完整成功：是 / 否
- world：example.world
- vehicle：iris_mid360
- flight_mode：
- hight：
- target_laps：
- side_length：
- radius：
- speed：
- takeoff_height：
- 启动前 connected：
- 启动前 armed：
- 启动前 pose 是否有效：
- 启动前 setpoint publishers：
- setpoint 平均频率：
- 首次看到 OFFBOARD 的时间：
- 首次看到 armed=true 的时间：
- TAKEOFF -> TRACKING 的证据：
- 完成圈数：
- TRACKING -> LANDING 的证据：
- 最终 armed：
- 目标轨迹观察：
- 实际轨迹观察：
- 异常、警告或报错：
- 本轮只改变的变量：
- 结论：
```

最终参数表模板：

```markdown
| 参数 | 参数服务器可见 | C++ 读取位置 | 参与哪段计算 | 运行证据 | 结论 |
|---|---|---|---|---|---|
| flight_mode | | | | | |
| hight | | | | | |
| target_laps | | | | | |
| side_length | | | | | |
| radius | | | | | |
| speed | | | | | |
| takeoff_height | | | | | |
```

## 13. 常见故障：按顺序排查

### 13.1 `Resource not found: offboard`

原因通常是当前终端没有 source 工作空间：

```bash
source ~/.bashrc
source "$HOME/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash" --extend
rospack find offboard
```

如果 `devel/setup.bash` 不存在或包仍找不到，说明工作空间没有成功编译。回到工作空间检查 `catkin_make` 的第一条真实报错。

### 13.2 控制器一直显示 `Waiting for FCU connection...`

依次检查：

```bash
rosnode list | grep -E 'mavros|sitl|gazebo'
rostopic echo -n 1 /mavros/state
rosnode ping /mavros
```

如果 `/mavros/state` 没消息，问题在 PX4/MAVROS 链路，不在轨迹代码。不要通过反复启动更多控制节点解决连接问题。

### 13.3 一直 PREHEAT 或等待很久没有请求 OFFBOARD

先确认：

```bash
rostopic hz /mavros/setpoint_position/local
rostopic echo -n 2 /clock
```

Gazebo 暂停时 ROS 仿真时间不前进，`rate.sleep()` 和 5 秒计时都会停住。实时因子很低时，5 秒仿真时间也可能对应更长墙上时间。

### 13.4 请求 OFFBOARD 失败

检查：

1. setpoint 是否已连续发布；
2. 发布频率是否稳定；
3. `/mavros/state connected` 是否为 true；
4. Gazebo 是否暂停；
5. QGroundControl/PX4 终端是否给出 preflight 或模式拒绝原因；
6. 是否有另一个 setpoint 发布者干扰。

`mode_sent=true` 也不是最终证据，要看 state 的 mode。

### 13.5 已进入 OFFBOARD，但无法解锁

查看 QGroundControl 和 PX4 终端的 preflight 检查信息。README 提到 QGC 的 `No manual control input` 警告可通过开启虚拟游戏手柄处理，但不要把每一种解锁失败都归因于同一个警告。

排查时保留：

```bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/local_position/pose
```

不要手动强制解锁来掩盖传感器、估计器或连接错误。

### 13.6 解锁了但不起飞

比较目标与实际：

```bash
rostopic echo -n 1 /mavros/setpoint_position/local
rostopic echo -n 1 /mavros/local_position/pose
```

如果目标 z 仍接近地面，检查 `hight` 是否正确加载；如果目标正确但实际不动，看 PX4 模式是否掉出 OFFBOARD、位姿是否有效、Gazebo 是否暂停以及 PX4 终端警告。

### 13.7 飞机起飞后先横向或斜向移动

当前代码使用世界原点：

- TAKEOFF：`(0,0,hight)`；
- circle 第一目标：`(radius,0,hight)`；
- square 第一目标：`(-L/2,-L/2,hight)`；
- LANDING：`(0,0,initial_height)`。

所以某些横向接入运动是当前算法的预期结果，不代表随机失控。阶段 2 会改成记录起飞点并使用相对目标。

### 13.8 方形看起来不是完美直角

数学 setpoint 沿四条直线换边，但真实无人机有惯性、加速度限制和跟踪误差，PX4 也不会瞬间改变速度方向。先比较 setpoint 的几何形状，再看 actual pose；不要只凭 Gazebo 肉眼判断轨迹生成函数错误。

### 13.9 修改 `speed` 或 `takeoff_height` 没变化

这是当前已知事实：C++ 没有读取这两个参数。阶段 1 的任务正是用证据发现它们无效，不需要在本阶段把它们修好。

### 13.10 `rosparam set` 后飞行没有立即变化

当前参数只在 main 开始时读取一次。停止并确认上锁后，关闭控制 launch，修改 launch 或参数配置，再重新启动节点。

另外，重新运行 launch 会再次把 XML 中的值写回参数服务器，所以仅在 launch 前执行 `rosparam set` 也可能被 launch 覆盖。

### 13.11 命令行提示 `unused args [flight_mode]`

当前 launch 没有：

```xml
<arg name="flight_mode" .../>
```

它只有全局 `<param name="flight_mode" value="..."/>`，所以不能用 `flight_mode:=square` 作为 launch arg。按第 7.2 节直接编辑参数值。

### 13.12 `/autoarming_control/radius` 不存在

默认 launch 没写 `radius`，而 C++ 的默认值不会自动回写参数服务器。加入：

```xml
<param name="radius" value="2.0"/>
```

保存并重新启动控制 launch。

### 13.13 日志显示 DONE，但 `/mavros/state` 仍是 armed

这是当前 Lock/COMPLETED 逻辑的已知验证缺口。不要启动下一轮，也不要在空中强制上锁：

1. 确认模型是否已经接地；
2. 若仍在飞，使用 QGC 的 Land，而不是 Disarm；
3. 落地后通过 QGC 上锁；
4. 停止控制 launch和仿真；
5. 保存 bag 和日志；
6. 完整重启后再实验。

### 13.14 控制器完成后 `rostopic hz` 没数据

状态机进入 COMPLETED 后可执行文件退出，setpoint 发布者消失，这是正常现象。频率检查必须在 PREHEAT、等待解锁或飞行阶段进行。

### 13.15 第二轮启动后节点名冲突或出现旧发布者

上一轮 `roslaunch` 可能仍运行着静态 TF，或者旧控制节点并未退出。执行：

```bash
rosnode list | grep -E 'autoarming|map_to_camera_init'
rostopic info /mavros/setpoint_position/local
```

回到上一轮启动终端按 `Ctrl+C` 正常关闭。不要在发布者不明时继续解锁。

### 13.16 飞行中 mode 从 OFFBOARD 掉出

检查 setpoint 是否中断、Gazebo 是否卡顿、节点是否崩溃，以及 PX4/QGC 的 failsafe 信息。当前代码会每隔 5 秒再次请求 OFFBOARD，但“会重试”不代表可以忽略模式掉落。保留 bag，记录掉落前的频率和状态。

## 14. 当前代码的已知问题清单

阶段 1 的目标是准确识别，不是一次全部修完：

1. 只等待 FCU 连接，没有等待第一帧有效 pose。
2. `initial_height` 可能在 pose 尚未更新时记录。
3. 起飞和降落固定使用世界原点，没有保存 home x/y。
4. `hight` 是绝对 z，不是相对起飞高度。
5. 没有 HOVER 阶段，达到高度后立即开始轨迹。
6. 新构造的目标没有显式设置合法四元数。
7. `speed` 和 `takeoff_height` 没有被读取。
8. `radius` 被读取，但默认 launch 没有显式设置。
9. 轨迹速度不是按时间和指定速度推进。
10. 非 `square` 的任意字符串都会静默走 circle 分支。
11. TAKEOFF 转换只检查 z，不检查水平位置。
12. LANDING 只检查高度差，不检查接地、速度或水平误差。
13. 上锁是否成功没有反馈给状态机。
14. 日志 `[DONE]` 不能证明实际已经上锁。
15. 没有阶段超时和明确的异常状态。

你能说清楚这些问题，就已经为阶段 2 的安全改造建立了需求清单。

## 15. 自测题与答案

### 题 1：`autoarming_control` 是否直接控制四个电机？

不是。它发布上层位置 setpoint，PX4 内部控制器根据目标与实际状态计算速度、姿态、推力和电机输出。

### 题 2：为什么请求 OFFBOARD 前要发送 setpoint？

PX4 要先看到连续有效的外部目标流，才允许进入并维持 OFFBOARD。当前程序以 20 Hz 预发 100 帧，约 5 秒仿真时间。

### 题 3：`mode_sent=true` 是否等于 PX4 已处于 OFFBOARD？

不等于。它只说明服务请求已发送或受理；最终应检查 `/mavros/state.mode`。

### 题 4：`flight_mode=circle` 是否会让 `/mavros/state.mode` 变成 circle？

不会。`circle` 是控制节点的轨迹选择，PX4 模式仍是 `OFFBOARD`。

### 题 5：为什么 `speed` 能被 `rosparam get` 查到，却不影响飞行？

launch 把它放进了参数服务器，但 C++ 没有读取和使用它。参数存在不代表算法使用。

### 题 6：默认起飞高度由哪个参数决定？

由拼写为 `hight` 的私有参数决定。`takeoff_height` 当前没有被 C++ 读取。

### 题 7：`radius` 不在参数服务器上，为什么圆仍可能是 2 m 半径？

C++ 的 `nh_private.param("radius", radius, 2.0)` 会在参数缺失时使用内部默认值 2.0，但不会自动把默认值写回参数服务器。

### 题 8：`side_length=4` 的方形为什么从 `(-2,-2)` 开始？

方形以世界原点为中心，半边长为 2。函数在 `t=0` 返回 `(-L/2,-L/2)`。

### 题 9：当前程序如何控制轨迹速度？

它没有显式的米每秒速度控制。目标在当前距离小于 1 m 时按 `(1-d)/trajectory_length` 推进，`speed` 没参与。

### 题 10：为什么 `[DONE]` 不能证明已经上锁？

`Lock()` 的成功结果没有返回主状态机，主循环无论服务是否成功都会进入 COMPLETED 并打印 DONE。应检查 `/mavros/state.armed`。

### 题 11：FAST-LIO 是否参与默认飞行位置反馈？

没有。控制器订阅 `/mavros/local_position/pose`，不订阅 FAST-LIO 的 `/Odometry`。

### 题 12：修改 launch 参数为什么不用编译？

launch 是运行时 XML 配置；重新启动 launch 会重新加载。修改 C++ 才需要重新编译生成可执行文件。

### 题 13：为什么不能把 `radius` 设为 0 来模拟悬停？

圆轨迹总长会变为 0，TRACKING 中 `delta_t = advance / trajectory_length` 存在除零风险。真正的悬停应在阶段 2 增加 HOVER 状态并持续发布固定合法目标。

### 题 14：为什么 `rosparam set` 后当前节点没有立刻改变？

参数只在节点启动时读取一次，没有动态参数回调。必须安全结束当前飞行并重启节点。

## 16. 阶段 1 最终验收清单

只有全部勾选后再进入阶段 2：

- [ ] 我能分终端启动 roscore、PX4/Gazebo/MAVROS 和 Offboard 控制器。
- [ ] 我知道 `autoarming_control.launch` 会自动切模式、解锁和起飞。
- [ ] 每次起飞前我都会检查 connected、armed、pose、仿真时间和 setpoint 发布者。
- [ ] 我能解释 publisher、subscriber、callback、service client、`spinOnce()` 和 `ros::Rate(20)`。
- [ ] 我能画出 `/mavros/state`、local pose、setpoint、MAVROS、PX4、Gazebo 的闭环。
- [ ] 我能解释 OFFBOARD 前预发 100 帧 setpoint 的意义。
- [ ] 我能区分 PX4 的 OFFBOARD 与轨迹参数 circle/square。
- [ ] 我能按源码顺序口述 WAIT/PREHEAT/OFFBOARD/ARM/TAKEOFF/TRACKING/LANDING/COMPLETED。
- [ ] 我至少完成三次完整飞行，并且每次最终确认 `armed:false`。
- [ ] 三次记录中至少包含一次 circle 和一次 square。
- [ ] 我保存了对应 rosbag 或等价的终端/曲线证据。
- [ ] 飞行时 setpoint 只有 `/autoarming_control` 一个发布者。
- [ ] 我实测 setpoint 频率稳定并接近 20 Hz。
- [ ] 我能写出圆形四个关键点和方形四个顶点。
- [ ] 我知道圆和方形当前都围绕世界原点，而不是自动围绕起飞点。
- [ ] 我能用源码和数据解释 `hight`、`target_laps`、`radius`、`side_length`。
- [ ] 我能证明 `speed`、`takeoff_height` 当前无效。
- [ ] 我知道参数在当前节点中只在启动时读取一次。
- [ ] 我知道合法无旋转四元数是 `(0,0,0,1)`，不是全零。
- [ ] 我能解释 `[DONE]` 为什么不能替代实际 armed 状态检查。
- [ ] 我能解释 FAST-LIO 为什么没有进入当前默认闭环。
- [ ] 我列出了当前代码至少 5 个待改进点。
- [ ] 我没有修改 `build/`、`devel/` 中的生成文件。

## 17. 进入阶段 2 前你应该形成的结论

阶段 1 不是要证明当前代码已经完美，而是要建立一个经过观察和数据验证的基线。你应该得到下面这组清晰结论：

```text
默认闭环能够在仿真中完成：
连接 -> setpoint 预热 -> OFFBOARD -> 解锁
-> 绝对坐标起飞 -> 圆/方形 -> 回原点降落 -> 尝试上锁

它已经适合教学演示，但还不够安全、通用：
没有等待有效 pose
没有记录完整 home 点
没有悬停状态
没有合法姿态初始化
没有真正的速度参数
没有超时和失败状态
没有可靠确认最终上锁
```

阶段 2 将在这条已验证主链上实现最小可靠任务：

```text
等待首帧有效位姿
  -> 记录 home_x/home_y/home_z
  -> 相对起飞到指定高度
  -> 定点悬停指定时间
  -> 返回 home 上方
  -> 垂直降落
  -> 确认真正上锁
```

到那时再修改 `autoarming_control.cpp`。如果你现在还不能不看本文口述默认状态机，或者没有三次完整记录，就先不要进入阶段 2。