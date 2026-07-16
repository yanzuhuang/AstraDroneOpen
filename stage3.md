# 阶段 3 教程：航点控制与自主任务状态机

> 适用对象：已完成阶段 2，第一次学习航点队列、yaw 和任务状态机的新手。
>
> 对应 `studymap.md` 的“阶段 3：航点控制与自主任务状态机”。本文及配套代码只允许用于 **PX4 SITL + Gazebo 仿真**。

## 1. 这次任务要完成什么

阶段 2 只会原地起飞、悬停和降落。阶段 3 要让无人机无需人工逐条发命令，自动完成：

```text
等待 FCU 与有效位姿
  -> 预发送 setpoint
  -> OFFBOARD 并解锁
  -> 原地起飞、短暂悬停
  -> 航点 1、航点 2
  -> 在航点 2 原地转向
  -> 航点 3
  -> 返回 home 上空
  -> 垂直降落
  -> 确认 armed=false
```

我已经把本阶段需要的实现补进仓库：

- `offboard/src/autoarming_control.cpp`：完整任务状态机、yaw、到达判定和安全分支；
- `offboard/config/stage3_waypoints.yaml`：默认航点任务；
- `offboard/launch/autoarming_control.launch`：加载 YAML 和所有阶段 3 参数；
- `offboard/CMakeLists.txt`、`offboard/package.xml`：补齐消息和 YAML 参数解析依赖。

这里的 `offboard/` 是：

```text
AstraDrone_ros1_ws/src/MissionControl/
astra_uavoffbard_frame/offboard/
```

你本阶段的任务不是重新抄一遍代码，而是先读懂它，再按本文完成默认任务、修改航点、验证安全分支，最后用证据验收。

## 2. 开始学习前先看懂专业术语

下面不要求背诵。第一次遇到不懂的英文或缩写时，回到这里查它在本任务中代表什么。

### 2.1 仿真和 ROS 术语

| 术语 | 通俗解释 | 在阶段 3 中的作用 |
|---|---|---|
| PX4 | 无人机飞控软件，相当于无人机的“驾驶员” | 接收位置目标，完成姿态、推力和电机控制 |
| FCU | Flight Control Unit，飞行控制单元 | 本项目中通常指运行 PX4 的虚拟飞控 |
| SITL | Software In The Loop，软件在环仿真 | 不使用真飞控板，直接在电脑中运行 PX4 |
| Gazebo | 机器人三维物理仿真器 | 模拟无人机、重力、碰撞和场景 |
| ROS | 机器人软件通信框架 | 让控制器、MAVROS 和传感器交换数据 |
| node（节点） | 一个正在运行的 ROS 程序 | `/autoarming_control` 就是本阶段的控制节点 |
| topic（话题） | 节点之间持续传输数据的“频道” | 位姿和 setpoint 都通过话题传输 |
| publisher / subscriber | 话题的发布者 / 订阅者 | 控制器订阅实际位姿，发布目标位姿 |
| message / `PoseStamped` | message 是话题中传输的数据；`PoseStamped` 是带时间和坐标系的位置姿态消息 | 本程序用它接收实际位姿并发送目标位姿 |
| service（服务） | 发出一次请求并等待一次回答 | 切换 OFFBOARD、AUTO.LAND 和解锁使用服务 |
| parameter（参数） | 运行时可配置的数值或选项 | 高度、容差、停留时间和超时都是参数 |
| launch | 一次启动多个节点并设置参数的 ROS 文件 | `autoarming_control.launch` 启动本阶段任务 |
| YAML | 容易阅读的配置文件格式 | `stage3_waypoints.yaml` 保存航点列表 |
| catkin / workspace | ROS1 的构建工具 / 存放 ROS 包的工作空间 | 修改 C++ 后用 `catkin_make` 重新生成可执行文件 |

### 2.2 飞行控制术语

| 术语 | 通俗解释 | 在阶段 3 中的作用 |
|---|---|---|
| MAVLink | 飞控常用的通信协议 | PX4 和外部程序通过它交换命令与状态 |
| MAVROS | MAVLink 与 ROS 之间的转换桥梁 | 把 PX4 状态变成 ROS 话题，并把 ROS 目标传给 PX4 |
| armed / disarmed | 解锁 / 上锁 | 解锁后电机可以工作；落地后由 PX4 检测并自动上锁 |
| OFFBOARD | 由机外计算机持续提供目标的 PX4 模式 | 本阶段的 C++ 节点在该模式下控制无人机 |
| prestream | 进入 OFFBOARD 前预先连续发送一段目标 | 让 PX4 先确认外部目标流稳定，再接受 OFFBOARD |
| setpoint | 当前希望无人机达到的目标值 | 本任务发送目标位置和目标 yaw，而不是直接控制电机 |
| pose（位姿） | 位置与朝向的组合 | 用实际 pose 判断是否到达目标 |
| position / attitude | 位置 / 姿态 | position 是 `x/y/z`；attitude 是机身朝向 |
| localization / estimator | 定位 / 根据传感器估计无人机当前状态的模块 | PX4 必须先获得健康的位置估计，才能安全执行航点 |
| airframe | 某种机型对应的 PX4 参数和硬件配置 | 当前 `iris_mid360` airframe 使用视觉位置、禁用 GPS |
| home | 本次任务开始时记录的起飞点 | 所有航点都相对 home 计算，任务结束后回到这里 |
| waypoint（航点） | 无人机要依次到达的目标 | 每个航点包含位置、yaw 和停留时间 |
| yaw | 绕竖直轴旋转的机头航向角 | 决定无人机到点后机头朝哪里 |
| quaternion（四元数） | 计算机表示三维旋转的一种四元数字格式 | 程序把 yaw 转成四元数后放进 `PoseStamped` |
| control loop / Hz | 控制循环 / 每秒执行次数 | `20 Hz` 表示每秒更新并发布目标约 20 次 |

### 2.3 坐标和状态机术语

| 术语 | 通俗解释 | 在阶段 3 中的作用 |
|---|---|---|
| coordinate frame（坐标系） | 规定原点和三个轴方向的参考系 | 同一组数字在不同坐标系中可能代表不同位置 |
| map | 本任务使用的局部世界坐标系名称 | setpoint 的 `frame_id` 设置为 `map` |
| ENU | East-North-Up，东、北、上 | 本任务中 `+x/+y/+z` 分别对应东、北、上 |
| TF | ROS 中管理坐标系关系的机制 | 描述 `map`、`camera_init`、`body` 等坐标系如何连接 |
| state machine（状态机） | 把完整任务拆成若干有条件切换的阶段 | 依次执行起飞、悬停、航点、返航和降落 |
| state / transition | 状态 / 状态切换 | `WAYPOINTS` 是状态；到点保持完成后切换到下一状态 |
| queue / index | 队列 / 当前序号 | 航点按列表排队，index 指向当前正在执行的航点 |
| error / tolerance | 误差 / 允许范围 | 目标差多少叫误差；小于容差才认为到达 |
| hold time | 到达后连续稳定保持的时间 | 防止只是从目标附近经过就算完成 |
| timeout | 某动作允许等待的最长时间 | 超过时间仍未完成就进入安全分支 |
| failsafe | 检测到异常后的安全处理 | 本程序会保持、返航或降落，而不是继续盲飞 |

### 2.4 查看和记录工具

| 术语 | 通俗解释 | 在阶段 3 中的作用 |
|---|---|---|
| FAST-LIO | 使用 LiDAR 和 IMU 估计运动位置的算法 | 当前 `iris_mid360` 仿真用它给 PX4 提供视觉位置输入 |
| LiDAR / IMU | 激光雷达 / 惯性测量单元 | LiDAR 测量周围距离；IMU 测量加速度和角速度 |
| RViz | ROS 数据可视化工具 | 用来观察坐标系、轨迹和无人机位置 |
| rosbag | ROS 话题录制文件和录制工具 | 保存状态、实际位姿和目标，作为验收证据 |
| QGroundControl | 常用的无人机地面站软件 | 仿真异常时可观察状态，并在必要时发送 Land |
| `rostopic` / `rosparam` | 查看 ROS 话题 / 参数的命令行工具 | 检查连接、发布者、频率、位姿和任务参数 |

现在再学习下面的知识点。遇到术语时，先用上表把它翻译成直白含义，再思考它在程序中的作用。

## 3. 先掌握五个核心概念

### 3.1 任务层和控制输出层

状态机决定“现在去哪里”，`local_pos_pub.publish(setpoint)` 每个周期负责“继续把当前目标发给 PX4”。进入等待或转向状态也不能停止发布，否则 PX4 可能退出 OFFBOARD。

本程序以 `20 Hz` 发布 `/mavros/setpoint_position/local`，PX4 继续负责姿态、电机等底层闭环。

### 3.2 航点是相对 home 的 ENU 坐标

程序收到第一帧有效位姿后，只记录一次：

```text
home = (home_x, home_y, home_z)
```

YAML 中的 `(x, y, z)` 是相对 home 的偏移，实际目标为：

```text
(home_x + x, home_y + y, home_z + z)
```

ENU 中 `x/y/z` 分别表示局部坐标的东/北/上方向。这样即使模型不是从世界原点出生，任务仍会在出生点附近执行。

### 3.3 yaw 是机头航向

本任务用 `yaw_deg` 表示 map/ENU 中的绝对航向：

| `yaw_deg` | 机头方向 |
|---:|---|
| `0` | `+x` |
| `90` | `+y` |
| `180` | `-x` |
| `-90` | `-y` |

代码用 `sin(yaw/2)`、`cos(yaw/2)` 生成合法单位四元数。两个连续动作若位置相同、yaw 不同，就能练习“到点后原地转向”。

### 3.4 到点不等于完成

一个航点只有同时满足下面三项才完成：

```text
三维位置误差 <= waypoint_tolerance（默认 0.20 m）
yaw 误差      <= yaw_tolerance_deg（默认 10°）
连续保持时间   >= hold_sec（默认任务为 2 s）
```

如果保持期间飞出门限，计时会清零。这避免无人机偶然经过目标附近就错误推进任务。

### 3.5 超时不能当作成功

默认安全策略如下：

| 异常 | 程序行为 |
|---|---|
| 起飞超时 | 转入降落 |
| 航点超时 | 保持当前位置 5 秒，再返航；不推进航点索引 |
| 返航超时 | 在当前位置垂直降落 |
| 位姿过期 | 保持最后一个 setpoint，不继续任务 |
| 丢失 OFFBOARD | 保持目标、重新请求 OFFBOARD、冻结任务计时 |
| 降落超时 | 继续发布降落目标，不把超时当作已落地 |

这就是有限状态机最重要的思维：每个状态都要有持续动作、完成条件、超时和失败去向。

## 4. 读懂已经实现的任务

先打开 `autoarming_control.cpp`，按下面顺序阅读：

1. `Waypoint`：一个动作包含 `x/y/z/yaw/hold_sec`；
2. `FlightPhase`：列出起飞、悬停、航点、故障保持、返航、降落等状态；
3. `pose_cb()`：只接受有限数值，并记录最新位姿时间；
4. `load_waypoints()`：读取并检查 YAML；
5. `make_setpoint()`：同时生成位置和 yaw 目标；
6. 主循环中的各个 `FlightPhase` 分支：持续发布、判断完成或切换安全状态。

注意：`WAIT_POSE`、`PRESTREAM`、`ARM_AND_OFFBOARD` 在进入飞行循环前完成，所以没有全部写进 `FlightPhase` 枚举，但日志中能看到这些步骤。

默认 `stage3_waypoints.yaml` 等价于：

| 动作 | 相对 home 位置/m | yaw | 停留 |
|---|---|---:|---:|
| 航点 1 | `(1, 0, 3.0)` | `0°` | `2 s` |
| 航点 2 | `(1, 1, 3.0)` | `0°` | `2 s` |
| 原地转向 | `(1, 1, 3.0)` | `90°` | `2 s` |
| 航点 3 | `(0, 1, 3.0)` | `180°` | `2 s` |

三个不同位置组成一个小范围任务；额外的同位置动作专门验证 yaw。任务结束后，程序自动回到 home 上方再下降。

## 5. 安全规则

1. 只在 Gazebo 仿真使用。程序会自动切换 OFFBOARD 并解锁，因此仍用 `/use_sim_time=true` 阻止真机误运行；降落使用 PX4 `AUTO.LAND`，程序不发送上锁命令。
2. 首轮只用空场 `example.world`，不要立刻扩大航点或提高高度。
3. 同一时刻只能有 `/autoarming_control` 一个 setpoint 发布者。
4. 不要在空中 `Ctrl+C` 停控制器；正常任务应自行返航和降落。
5. 空中异常时优先在 QGroundControl 执行 `Land`，接地后再停止进程；不要把空中 `Disarm` 当急停。
6. 本阶段不要同时启动 `position_control` 或另一个 Offboard 节点。

## 6. 编译和静态检查

在新终端执行：

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make --pkg offboard -j2
source devel/setup.bash
roslaunch --nodes offboard autoarming_control.launch rviz:=false
```

正确结果应包含：

```text
Built target autoarming_control
/map_to_camera_init
/autoarming_control
```

若 `rospack find px4` 找不到 PX4，说明当前终端没有加载安装脚本写入的环境。先重新打开终端并复习阶段 0 的环境配置，不要急着运行控制器。

## 7. 第一次完整仿真实践

### 7.1 终端 1：启动 PX4、Gazebo 和 MAVROS

```bash
cd ~/AstraDroneOpen
roslaunch simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch \
  world:=/home/yanzu/AstraDroneOpen/simulation/astra_gazebo_worlds/example.world
```

等 Gazebo 出现无人机，并看到 MAVROS 已连接。不要用 `pc_example.sh` 做第一次验收，因为它会自动启动控制节点，不方便检查唯一发布者。

### 7.2 终端 2：启动当前机型所需的定位输入

当前 `iris_mid360` 的 airframe 配置为“视觉位置和航向、无 GPS”。本机实际测试中，不启动 FAST-LIO 时 PX4 的水平位置估计不健康并拒绝解锁。因此执行：

```bash
astra
roslaunch fast_lio mapping_mid360.launch rviz:=false
```

等到终端出现 `IMU Initial Done`，再等待 PX4 的航向估计稳定。本阶段不要求你理解 FAST-LIO 算法：航点节点仍只订阅 MAVROS 局部位姿；这里使用已有 FAST-LIO 的 `camera_init -> body` TF，经 MAVROS 给 PX4 提供视觉估计。FAST-LIO 的独立学习仍在阶段 7。

### 7.3 终端 3：起飞前检查

```bash
astra
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/estimator_status
rostopic info /mavros/setpoint_position/local
```

开始任务前应满足：

- `connected: True`；
- 水平位置估计标志已健康，不再长期显示 `pos_horiz_*: False`；
- setpoint 的 `Publishers` 为 `None`。

### 7.4 终端 4：录制最小证据

```bash
mkdir -p ~/AstraDroneOpen/records/stage3
cd ~/AstraDroneOpen/records/stage3
rosbag record -O stage3_run01.bag \
  /mavros/state \
  /mavros/local_position/pose \
  /mavros/local_position/odom \
  /mavros/setpoint_position/local
```

### 7.5 终端 5：启动任务

```bash
astra
roslaunch offboard autoarming_control.launch rviz:=true
```

另开终端观察发布频率和参数：

```bash
rostopic hz /mavros/setpoint_position/local
rosparam get /autoarming_control/waypoints
```

正常日志顺序应包含：

```text
[WAIT_POSE] -> [PRESTREAM] -> [MODE] -> [ARM]
[PHASE] TAKEOFF -> INITIAL_HOVER
WAYPOINT_1 -> WAYPOINT_2 -> WAYPOINT_3 -> WAYPOINT_4
RETURN_HOME -> AUTO.LAND -> [DONE]
```

`[WAYPOINT] ... left tolerance; hold timer reset` 不是故障，它证明“连续稳定停留”检查正在工作。最后必须看到：

```text
[MODE] AUTO.LAND confirmed; PX4 owns descent
[DONE] PX4 landing complete: armed=false, ground=extended_state
```

再执行：

```bash
rostopic echo -n 1 /mavros/state
```

确认 `armed: False` 后，停止 rosbag，再依次停止 FAST-LIO 和仿真。

## 8. 修改成你自己的航点任务

先复制默认文件，不必每次改原始基线：

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/config
cp stage3_waypoints.yaml stage3_my_waypoints.yaml
```

初学者建议只改成下面这种小范围任务：

```yaml
waypoints:
  - {x: 0.8, y: 0.0, z: 3.0, yaw_deg: 0.0,  hold_sec: 3.0}
  - {x: 0.8, y: 0.8, z: 3.0, yaw_deg: 90.0, hold_sec: 3.0}
  - {x: 0.0, y: 0.8, z: 3.0, yaw_deg: 180.0, hold_sec: 3.0}
```

使用自定义文件启动：

```bash
roslaunch offboard autoarming_control.launch rviz:=true \
  mission_file:=/home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/config/stage3_my_waypoints.yaml
```

规则：

- `x/y/z/yaw_deg` 必须是数值；
- `z > 0`，`hold_sec >= 0`；
- YAML 列表不能为空；
- 首轮航点离 home 不超过约 `1 m`，高度保持 `3.0 m`；
- 修改 YAML 后要重启 launch，但不必重新编译 C++。

常用 launch 参数可以直接覆盖：

```bash
roslaunch offboard autoarming_control.launch rviz:=false \
  waypoint_tolerance:=0.20 \
  yaw_tolerance_deg:=10.0 \
  waypoint_timeout:=30.0
```

不要通过放大门限来掩盖控制问题。门限越大，程序越容易把“只是路过”误判为完成。

## 9. 必做的三轮实验

### 实验 A：默认任务

目标：完整走完四个动作、返航、降落。记录每个航点是否出现 `reached` 和 `completed`，最终是否 `armed=false`。

### 实验 B：自己的三个航点

使用 `stage3_my_waypoints.yaml`，检查实际位置是否始终等于 `home + 相对偏移`。若从非零出生点启动，任务图形仍应整体跟随 home 平移。

### 实验 C：验证航点超时

只在空场做。把一个航点设得较远，同时把 `waypoint_timeout` 临时设为 `2.0`：

```bash
roslaunch offboard autoarming_control.launch rviz:=false waypoint_timeout:=2.0
```

应看到：

```text
[TIMEOUT] WP... timed out
[FAILSAFE_HOVER]
[PHASE] RETURN_HOME started after waypoint failure
```

程序不能输出该航点 `completed`，也不能继续盲目增加索引。验证完恢复 `30.0 s`。

## 10. 常见问题

### 一直 `Arm request failed`

先看：

```bash
rostopic echo -n 1 /mavros/estimator_status
```

若水平位置标志为 `False`，确认 LiDAR/IMU 话题存在、FAST-LIO 已出现 `IMU Initial Done`，然后再等几秒。不要绕过 PX4 健康检查强行解锁。

### 一直卡在某个航点

看日志中的 `pos_err`、`yaw_err` 和 `hold`：

- `pos_err` 大：位置尚未稳定；
- `yaw_err` 大：机头尚未转到目标；
- `hold` 反复归零：飞机多次飞出门限；
- 超过 `waypoint_timeout`：程序应进入安全保持和返航。

### YAML 读取失败

检查缩进、冒号和字段名，必须使用 `yaw_deg`，不能写成 `yaw`。可以先运行：

```bash
roslaunch --nodes offboard autoarming_control.launch rviz:=false \
  mission_file:=/你的绝对路径/stage3_my_waypoints.yaml
```

### 无人机突然飞向奇怪位置

立即检查是否有多个发布者：

```bash
rostopic info /mavros/setpoint_position/local
```

只允许 `/autoarming_control`。同时确认你没有把相对 home 的偏移误写成很大的世界坐标。

## 11. 已有验证与待重新验证项

此前的航点任务已完成以下检查；改用 PX4 `AUTO.LAND` 后，降落末端需要重新验证：

- `catkin_make --pkg offboard -j2` 编译通过；
- launch XML 和 YAML 解析通过，成功加载 4 个动作；
- 无界面 PX4/Gazebo + FAST-LIO 实飞通过；
- 四个动作均满足位置、yaw 和连续停留后才完成；
- 飞出 `0.2 m` 门限时，停留计时确实清零；
- 成功返航并垂直降落；
- 已移除 `21196` 强制上锁命令，改为请求 PX4 `AUTO.LAND`；
- `armed=false` 与 `extended_state=ON_GROUND` 的退出流程需要重新进行仿真验证。

关闭整套 Gazebo/PX4 时 PX4 进程曾在退出阶段报告一次段错误，但它发生在任务完成降落之后，不影响航点状态机的既有验收结果。

## 12. 阶段 3 验收清单

- [ ] 我能解释状态机为什么每个周期都要发布 setpoint。
- [ ] 我能解释相对 home 航点和世界绝对坐标的区别。
- [ ] 我知道 yaw 如何变成四元数，并完成一次原地转向。
- [ ] 飞行时只有 `/autoarming_control` 一个 setpoint 发布者。
- [ ] setpoint 频率稳定，任务期间没有长期退出 OFFBOARD。
- [ ] 每个航点进入 `0.2 m` 半径、yaw 误差小于 `10°`，并连续保持至少 `2 s`。
- [ ] 我验证过飞出门限会重置计时，而不是错误完成。
- [ ] 我验证过航点超时不会推进索引，而会保持并返航。
- [ ] 我保存了 rosbag、终端日志和自己的航点 YAML。
- [ ] 我连续完成三次任务，最终都确认 `armed: False`。

全部勾选后，阶段 3 才算真正掌握。阶段 4 再学习按真实时间和速度推进轨迹、切线 yaw，以及圆形、方形和 8 字轨迹；不要在阶段 3 提前把航点状态机改成复杂轨迹生成器。
