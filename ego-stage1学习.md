# AstraDroneOpen 阶段1学习：固定高度圆弧绕塔与八检查点

## 最常用：在终端启动和关闭阶段1仿真

### 启动完整任务并显示 Gazebo

打开终端，执行：

```bash
cd /home/yanzu/AstraDroneOpen
scripts/run_sh/stage1_tower.sh \
  --control --mode mission --waypoints 8 --attach
```

这条命令会依次启动 PX4 SITL、Gazebo GUI、MAVROS、FAST-LIO 和阶段1任务节点，并在完整 preflight 后自动请求 OFFBOARD、解锁、起飞和绕塔。要看到 Gazebo 窗口，不要添加 `--headless`。

`--attach` 会让当前终端进入 `stage1_tower` 的 tmux 日志界面。若只想退出日志界面但保持仿真运行，先按 `Ctrl-b`，松开后再按 `d`。之后可用下面的命令重新进入：

```bash
tmux attach -t stage1_tower
```

### 正常关闭整套仿真

正常任务应先等待 `/tower_mission/final_result` 显示 `SUCCESS`，并确认无人机已经降落和上锁。然后打开另一个终端执行：

```bash
cd /home/yanzu/AstraDroneOpen
scripts/run_sh/stage1_tower.sh --stop
```

如果只有当前这一个终端，先按 `Ctrl-b`，松开后按 `d` 退出 tmux，再执行同一条 `--stop` 命令。它会安全停止任务节点、FAST-LIO、PX4、Gazebo 和本脚本创建的 tmux 会话。不要只在 tmux 中按 `Ctrl-C`，那可能只停止当前窗口中的一个组件。

### 只预览路线，不启动 Gazebo，也不解锁

```bash
cd /home/yanzu/AstraDroneOpen
scripts/run_sh/stage1_tower.sh --preview --waypoints 8 --attach
```

预览模式只启动路线节点和 RViz，不启动 PX4/Gazebo，也不会创建 MAVROS 控制发布者。关闭方式仍是 `scripts/run_sh/stage1_tower.sh --stop`。

> 适用范围：ROS1 Noetic、PX4 SITL、Gazebo Classic、MAVROS、FAST-LIO，单机仿真。
>
> 实施与验证日期：2026-07-21。
>
> 本阶段不启动 EGO-Planner，不做避障、多层、螺旋、动态障碍或多机。本文中的“通过”只代表对应参数和 `forest.world` 的 Gazebo 验证，不代表真机可直接使用。
>
> 2026-07-22 后续修改：当前默认world已改为 `worksite.world`，目标为运行时 `radio_tower=(-10.0551,19.7104)`，保持8 m高度、10 m半径、严格圆周、8个不停留检查点和绕塔朝塔yaw。进场与返航均沿前进方向，现有 `forest.world` 数据只作为历史飞行证据。当前只完成构建、单测、模型几何复核和无控制preview，分级实飞前不得写成“worksite已通过”。

## 1. 阶段目标和完整状态流程

阶段1的目标不是“生成八个点”，而是让一架无人机安全地完成一整套任务：

```text
WAIT_INPUTS（初始化与解锁前检查）
  -> PRESTREAM（预发送 OFFBOARD setpoint）
  -> ARM_OFFBOARD（进入 OFFBOARD 并解锁）
  -> TAKEOFF（起飞）
  -> INITIAL_HOVER（初始稳定悬停）
  -> MISSION（前向进场，再沿严格圆周连续通过 8 个检查角）
  -> RETURN_HOME（返回 home 上方）
  -> PRELAND_HOVER（降落前稳定）
  -> LANDING（OFFBOARD 受控下降并正常解锁）
  -> DONE
```

任一步出现输入断流、控制发布者冲突、OFFBOARD 丢失、进场超时、整体超时、降落超时或服务失败，都会进入有限的 `ERROR` 终态，不会无限等待。严重异常时节点停止自定义 setpoint，让 PX4 已配置的 failsafe 接管。

1点和4点模式只用于逐级验证检查角计数；实际任务默认保留8个检查点。完整圆周、返航、OFFBOARD降落、最终 `armed=false`，并再重复一次，才算当前参数通过。

## 2. 为什么阶段1暂时不让EGO控制

EGO-Planner负责根据障碍物和当前状态生成局部轨迹；它不应该同时承担塔巡检点设计、解锁、控制权仲裁、返航和降落。阶段1先隔离规划变量，验证更基础的事实：

- `map` 坐标中的塔中心是否正确；
- 圆周半径、八检查角顺序、进场/绕塔 yaw 是否正确；
- PX4 OFFBOARD setpoint 能否连续、唯一地发送；
- 到达判定、超时、返航和受控降落是否可靠；
- 失败能否进入明确终态。

这样进入阶段2时，若出现问题，才能判断是 EGO 规划/地图问题，还是基础任务和飞控链问题。阶段1仍启动 MID360 和 FAST-LIO，使本次锁定的 SITL 定位链健康并产生 `/Odometry`；但任务管理器的实际位置反馈是 `/mavros/local_position/pose`，没有订阅 EGO 轨迹，也没有发布 EGO 目标。

## 3. 节点分工和完整数据流

| 节点/进程 | 阶段1职责 | 是否发送 MAVROS setpoint |
|---|---|---|
| Gazebo `gzserver` | `forest.world`、iris_mid360、传感器和动力学 | 否 |
| PX4 SITL | 飞行控制、OFFBOARD 模式、解锁状态和 failsafe | 接收 MAVROS 转发的 setpoint |
| `/mavros` | ROS 与 PX4 之间的状态、服务和 setpoint 桥 | 转发，不生成任务 |
| `/laserMapping` | MID360 LiDAR/IMU 到 FAST-LIO `/Odometry` | 否 |
| `/tower_mission` | 圆周参考、检查点进度、状态流程、限速、返航、降落、结果记录 | 是，且必须是唯一任务节点 |
| `/stage1_rviz` | 显示预览路线、目标、塔安全圈和实际轨迹 | 否 |

控制数据流：

```text
stage1_tower.yaml
  -> /tower_mission 生成绝对 map 圆周和 8 个检查角
  -> 进场：直线限速参考 + 沿前进方向 yaw
  -> 绕塔：逐周期严格圆周参考 + 逐周期朝塔 yaw
  -> 正常飞行：/mavros/setpoint_position/local (PoseStamped)
  -> 受控下降：/mavros/setpoint_raw/local (PositionTarget)
  -> MAVROS -> PX4 OFFBOARD -> Gazebo iris_mid360

Gazebo/MID360 -> FAST-LIO -> /Odometry（启动链健康检查）
PX4 -> MAVROS -> /mavros/local_position/pose + velocity_local
                    -> /tower_mission 圆周跟踪误差和状态记录
```

位置控制和降落 raw-local 是两个不同 Topic，但都只由同一个 `/tower_mission` 节点注册。正常飞行只发布 position，降落阶段只发布 raw-local。节点还会通过 ROS master 周期检查 position、raw local、velocity、attitude 和 thrust 五类控制 Topic；发现外部发布者就拒绝解锁或进入 `ERROR`。

## 4. 圆周、检查点、map、home、ENU/NED和yaw

设塔中心为 `(cx, cy)`、绕塔半径为 `R`、固定高度为 `h`，沿任务方向累计的非负角进度为 `p`。连续圆周参考为：

```text
逆时针：theta(p) = start_angle + p
顺时针：theta(p) = start_angle - p
x(p) = cx + R*cos(theta(p))
y(p) = cy + R*sin(theta(p))
z(p) = h
```

检查点只是 `p_i = i*2*pi/N` 的 8 个等角度事件，用于记录和进度，不再作为会减速、到达、停留的离散位置目标。参考线速度按 `maximum_acceleration` 加速到 `maximum_speed`，角进度按 `delta_p = speed*dt/R` 推进；因此每个控制周期生成的参考点都严格位于半径 `R` 的圆上。

绕塔阶段每个控制周期都由当前圆周参考点重新计算朝塔 yaw：

```text
camera_yaw = atan2(cy-y, cx-x)
body_yaw = normalize(camera_yaw - camera_yaw_offset)
```

`normalize` 把角度归一化到 `[-pi, pi]`，避免从 `179°` 到 `-179°` 被误判成相差 `358°`。相机正前方若与机体 `+X` 同向，补偿为 `0°`；若安装有偏角，应填写从机体 `+X` 到相机光轴的 yaw 偏角。

进场阶段尚未到达圆周入口时，yaw 使用 `atan2(entry_y-current_y, entry_x-current_x)`，即机头沿当前前进方向。进入圆周后立即切换为逐周期朝塔 yaw。当前圆周从塔南侧 `-90°` 开始，默认逆时针；预览 Path 用至少 180 段显示圆，而 `waypoint_poses` 仍只包含8个检查点。

### `map` 与 home 不相同

- 塔中心和航点：`map` 中的绝对坐标，不能随着起飞点移动。
- home：通过解锁前的 MAVROS 本地位姿捕获，只用于起飞、返航和降落。
- 当前 world 在 `(0, 0)` 生成无人机；preflight 要求 home 与配置期望原点的水平偏差不大于 2 m。若更换 spawn 点，必须同步审查坐标契约，不能把塔坐标偷偷改成相对 home。

### ENU 与 NED

ROS/MAVROS 本地接口使用 ENU：`x` 向东、`y` 向北、`z` 向上，正 yaw 逆时针。PX4 内部常用 NED；轴转换由 MAVROS 完成，任务代码不再手动交换或取反。降落用的 `PositionTarget` 也向 MAVROS 填 ENU 值，`velocity.z < 0` 表示向下。

阶段1不依赖 EGO 的 `map -> camera_init` 假设；FAST-LIO 自己发布 `camera_init -> body`。正式统一 TF/外参仍是阶段2前的风险项。

## 5. 进场、连续检查点与OFFBOARD控制

### 检查点不停留

进场到第1检查角时只要求实际位置进入 `mission/position_tolerance` 且内部限速参考已到圆周入口，不要求在该点保持，也不等待进场 yaw 满足容差。随后按连续角进度判断跨过 CP2…CP8 和闭环 CP1：

```text
angular_progress >= next_checkpoint_progress
=> 记录 CHECKPOINT_PASS，立即继续推进圆周参考
```

中间检查点没有到达保持、没有把速度降为零，也不调用离散直线插值。`waypoint_timeout` 目前只约束到圆周入口的进场；`mission_timeout` 约束进场加一整圈，`overall_timeout` 约束整套流程。起飞、初始悬停、返航和降落前悬停仍保留各自的稳定保持条件。

### 为什么先预发送 setpoint

PX4 进入 OFFBOARD 前必须已经收到连续 setpoint。`PRESTREAM` 默认先发送 3 s，然后才请求 OFFBOARD 和解锁。飞行循环为 20 Hz，高于 PX4 对 OFFBOARD 连续性的最低要求并留有余量。

### 参考点不是一步跳到目标

任务管理器在进场、返航等直线阶段用 `maximum_speed`、`maximum_acceleration` 和 `maximum_yaw_rate_deg_s` 平滑推进参考点；绕塔阶段使用严格圆周采样，并校验 `maximum_speed/radius` 不超过配置 yaw 速率。CSV 同时记录：

- `position_error_m`：进场时为实际位置到入口的误差，绕塔时为实际位置到当前动态圆周参考的误差；
- `reference_tracking_error_m`：实际位置到平滑参考点的误差，用于观察控制跟踪；
- `yaw_error_deg`、`hold_time_s`、当前状态、航点号、目标和实际位姿。

### OFFBOARD受控降落

返回 home 上方并悬停后，节点保持 OFFBOARD，用 `/mavros/setpoint_raw/local` 锁住 home 的 `x/y/yaw`，发送向下速度。下降速度由复用的 `offboard/landing_profile.h` 平滑地从 `cruise_speed` 过渡到 `touchdown_speed`。确认接近地面、垂直速度足够小并持续满足接触时间后，再等待 PX4 `ON_GROUND`，最后调用正常 disarm。这里的下降 setpoint 速度不是 PX4 参数 `MPC_LAND_SPEED`。

## 6. 新增和修改文件说明

| 文件 | 作用 |
|---|---|
| `AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/package.xml` | 新包依赖和元数据 |
| `.../astra_tower_mission/CMakeLists.txt` | 构建路线库、限幅库、任务节点和 gtest |
| `.../include/astra_tower_mission/tower_route.h` | 圆周配置、方向、检查点和任意角进度采样接口 |
| `.../src/tower_route.cpp` | 严格圆周公式、顺逆时针、逐点朝塔 yaw 和参数检查 |
| `.../include/astra_tower_mission/motion_limiter.h`、`src/motion_limiter.cpp` | 直线限幅及连续圆周角进度参考生成 |
| `.../src/tower_mission_node.cpp` | 进场/圆周状态、检查点计数、控制权、超时、返航、降落和 CSV |
| `.../config/stage1_tower.yaml` | 塔、航线、飞行、安全、Topic、服务和输出的唯一默认配置 |
| `.../launch/stage1_tower.launch` | preview/control、点数、RViz 和报告文件入口 |
| `.../rviz/stage1_tower.rviz` | 新手可直接使用的路线/目标/实际轨迹显示 |
| `.../test/tower_route_test.cpp` | 顺逆时针、严格半径、逐周期朝塔 yaw、检查点不停留和限幅单测 |
| `scripts/run_sh/stage1_tower.sh` | 默认安全预览；显式 `--control` 才启动完整 PX4/Gazebo/FAST-LIO/任务链 |
| `offboard/CMakeLists.txt` | 导出并安装现有 `offboard/landing_profile.h`，供新包复用 |
| `ego-stage1学习.md` | 本文 |

没有修改 `autoarming_control.cpp`、EGO 核心、FAST-LIO 源码、外部 PX4 或 world。

## 7. Topic、消息、服务和TF表

### 任务管理器输入与控制输出

| 名称 | 类型 | 发布者/服务端 | 使用者 | 作用 |
|---|---|---|---|---|
| `/mavros/state` | `mavros_msgs/State` | MAVROS | tower_mission | connected、armed、mode |
| `/mavros/extended_state` | `mavros_msgs/ExtendedState` | MAVROS | tower_mission | `ON_GROUND` 确认 |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | MAVROS | tower_mission | 当前 ENU 位置和 yaw |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | MAVROS | tower_mission | 接地前垂直速度判断 |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | tower_mission | MAVROS/PX4 | 起飞、悬停、进场、圆周、返航 |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | tower_mission | MAVROS/PX4 | OFFBOARD 受控下降 |
| `/mavros/cmd/arming` | `mavros_msgs/CommandBool` 服务 | MAVROS | tower_mission | 正常解锁/上锁 |
| `/mavros/set_mode` | `mavros_msgs/SetMode` 服务 | MAVROS | tower_mission | 请求/恢复 OFFBOARD |

### 观察和记录输出（默认均在 `/tower_mission/` 私有命名空间）

| Topic | 类型 | 作用 |
|---|---|---|
| `/tower_mission/route_preview` | `nav_msgs/Path` | 至少180段采样的严格圆周预览，包含闭合端点 |
| `/tower_mission/waypoint_poses` | `geometry_msgs/PoseArray` | N 个等角度检查点及其朝塔方向 |
| `/tower_mission/route_markers` | `visualization_msgs/MarkerArray` | 塔中心、碰撞圈、安全圈、航线标记 |
| `/tower_mission/current_target` | `geometry_msgs/PoseStamped` | 进场最终目标或当前动态圆周参考 |
| `/tower_mission/actual_path` | `nav_msgs/Path` | 实际飞行轨迹 |
| `/tower_mission/state` | `std_msgs/String` | 当前状态 |
| `/tower_mission/progress` | `std_msgs/Float64` | 已通过圆弧段数占 N 段的比例 |
| `/tower_mission/waypoint_index` | `std_msgs/UInt32` | 最近通过的圆周检查点编号，显示时从1理解 |
| `/tower_mission/position_error` | `std_msgs/Float64` | 到入口或当前动态圆周参考的位置误差，m |
| `/tower_mission/yaw_error_deg` | `std_msgs/Float64` | 归一化 yaw 误差，deg |
| `/tower_mission/hold_time` | `std_msgs/Float64` | 当前连续满足容差的时间，s |
| `/tower_mission/final_result` | `std_msgs/String` | latched 的 SUCCESS 或 ERROR 终态 |
| `/tower_mission/report_file` | `std_msgs/String` | 本轮 CSV 路径 |

### 定位、传感器和TF

| 名称 | 类型/关系 | 阶段1意义 |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | MID360 点云输入 |
| `/livox/imu` | `sensor_msgs/Imu` | MID360 IMU 输入 |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO 健康检查；本任务节点不直接订阅 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST-LIO 注册点云；留给观察和阶段2 |
| `camera_init -> body` | TF | FAST-LIO 局部世界到机体关系 |
| `map` | 消息 `frame_id` | MAVROS 本地 ENU 控制与塔绝对坐标；阶段1不伪造到 `camera_init` 的正式标定 |

## 8. 编译、启动、RViz观察和逐级测试方法

以下命令均从仓库根目录执行。第一次只看路线，确认无误后才能显式使用 `--control`。

### 8.1 编译和单元测试

```bash
source /opt/ros/noetic/setup.bash
cd /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES='offboard;astra_tower_mission' -j2
catkin_make -DCATKIN_WHITELIST_PACKAGES='offboard;astra_tower_mission' \
  run_tests_offboard run_tests_astra_tower_mission -j2
catkin_test_results build/test_results
```

期望看到 `0 errors, 0 failures`。只编译成功不能替代仿真验收。

### 8.2 不解锁的RViz路线预览

```bash
cd /home/yanzu/AstraDroneOpen
scripts/run_sh/stage1_tower.sh --preview --waypoints 8 --attach
```

预览模式不启动 PX4/Gazebo，不创建 MAVROS 控制发布者，也不会解锁。RViz 中检查：

- Fixed Frame 是 `map`；
- 路径在 `z=8 m`；
- 路线从塔南侧开始并逆时针；
- 8个检查点箭头朝塔；
- Path 是平滑圆周而不是八边形，默认有181个 Pose 且首尾重合。

可在另一终端检查：

```bash
source /opt/ros/noetic/setup.bash
source /home/yanzu/AstraDroneOpen/simulation/sim_workspace/devel/setup.bash
source /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash --extend
rostopic echo -n 1 /tower_mission/state
rostopic echo -n 1 /tower_mission/route_preview
rosnode info /tower_mission
rostopic info /mavros/setpoint_position/local
```

预览结束：

```bash
/home/yanzu/AstraDroneOpen/scripts/run_sh/stage1_tower.sh --stop
```

### 8.3 当前改动的严格逐级控制验证（尚未执行）

当前换到较近塔后，已有只读审计指出塔心约 12.8 m 外存在 Pine，而环线半径是 10 m。阶段1没有避障，所以仅看塔自身 3.59 m 径向余量不足以证明对树安全。必须先在 Gazebo/RViz 对 8 m 高度的塔 mesh、树干/树冠 collision、南侧进场和完整圆周做静态净空复核；未通过前只运行 `--preview`，不要运行下面的 `--control`。

每一级都应从干净的新 PX4/Gazebo 启动，等到 `DONE`、确认 `armed=false`，再安全停止后进行下一级：

```bash
# 1. 起飞、初始悬停、返回和降落，不进塔航线
scripts/run_sh/stage1_tower.sh --control --mode hover --headless --attach

# 2. 单检查角计数模式，仍执行一整圈、返航和降落
scripts/run_sh/stage1_tower.sh --control --mode mission --waypoints 1 --headless --attach

# 3. 四检查角计数模式，圆周几何不变
scripts/run_sh/stage1_tower.sh --control --mode mission --waypoints 4 --headless --attach

# 4. 完整八检查点
scripts/run_sh/stage1_tower.sh --control --mode mission --waypoints 8 --headless --attach

# 5. 停止后，再完整运行一次八检查点
scripts/run_sh/stage1_tower.sh --control --mode mission --waypoints 8 --headless --attach
```

每条控制命令前都先检查上一轮已停止：

```bash
scripts/run_sh/stage1_tower.sh --stop
```

不使用 `--headless` 可同时打开 Gazebo GUI 和 RViz，但图形负载更大。运行中另开终端观察：

```bash
rostopic echo /tower_mission/state
rostopic echo /tower_mission/progress
rostopic echo /tower_mission/position_error
rostopic echo /tower_mission/yaw_error_deg
rostopic echo -n 1 /tower_mission/final_result
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/extended_state
```

只有明确输入 `--control` 才会自动请求 OFFBOARD 和解锁。第一次自行复现时，应有人观察并能立即执行 `--stop`；不要与 `autoarming_control` 或 EGO bridge 控制模式同时启动。

## 9. 2026-07-21原离散航点版本的历史测试结果

本节数据仅对应 `radio_tower_0`、4 m 高度、离散直线航段和每点保持 2 s 的旧实现。保留这些数据是为了可追溯，不代表当前严格圆周/较近塔/8 m 默认配置已经完成飞行验收。

### 9.1 固定环境和参数

| 项目 | 实际值 |
|---|---|
| Git 分支/任务开始 HEAD | `ego-project` / `dcd5bd5697ae141712df43766e9a0483e912a294` |
| world | `simulation/astra_gazebo_worlds/forest.world` |
| 机型 | `iris_mid360` |
| 外部 PX4 | detached `99c40407ffd7ac184e2d7b4b293f36f10fe561ef`，未修改 |
| 塔 | `radio_tower_0`，中心 `(24.4614, 39.3288)` m |
| 航线 | 半径 10 m，高度 4 m，起始角 -90°，逆时针 |
| 安全裕量 | 塔 mesh 径向包络 6.41 m；原始径向余量 3.59 m；配置要求至少 2 m |
| 控制限幅 | 0.30 m/s、0.50 m/s²、30 deg/s |
| 到达条件 | 0.40 m、12°、连续 2.0 s |
| 节点 | `/mavros`、`/laserMapping`、`/tower_mission`；没有启动 EGO |
| 唯一控制发布者 | position/raw-local 图上均只有 `/tower_mission`；正常飞行和降落分阶段发布 |
| 证据目录 | `/tmp/astra_stage1_evidence/`；CSV 为 5 Hz |

### 9.2 按要求的验证顺序

| 顺序 | 结果 | 关键证据 |
|---|---|---|
| 单元测试 | 通过 | 最终命令实际执行新包6/6、offboard 7/7、bridge 9/9，共22/22；当前 `build/test_results` 汇总44 tests、0 errors、0 failures |
| 不解锁 RViz 路线预览 | 通过 | `PREVIEW_ONLY`；9 Pose；8个唯一点加闭环；没有 MAVROS setpoint publisher |
| 起飞与悬停 | 通过 | 起飞、3 s 稳定悬停、返航、受控降落、正常 disarm；任务终态 42 s |
| 单航点 | 通过 | WP1 达到并保持、闭环、返航、降落；终态 300 s |
| 4航点 | 通过 | 南→东→北→西→南闭环、返航、降落；终态 494 s |
| 首次8航点完整一圈 | 通过 | 8点+闭环、返航、降落、`armed=false`；终态 518 s |
| 再重复一次完整任务 | 通过 | 第二次独立冷启动同样完成；`armed=false`、`ON_GROUND`；终态 518 s |

`终态时间` 是节点从任务流程开始到生成 `SUCCESS` 的仿真时间；CSV 文件还包含启动等待和 DONE 后短暂记录，因此 CSV 总时间跨度更长。

### 9.3 两次完整八点误差

| 指标 | 第1次 | 第2次 |
|---|---:|---:|
| MISSION/CLOSE_LOOP 段耗时 | 349.950 s | 349.850 s |
| 任务段最大参考跟踪误差 | 0.463 m | 0.436 m |
| 任务段最大高度误差 | 0.070 m | 0.086 m |
| 降落阶段最大 home 水平误差 | 0.098 m | 0.095 m |
| 返航到达位置/yaw误差 | 0.094 m / 0.006° | 0.087 m / 0.001° |
| 最终状态 | `SUCCESS`、上锁、`ON_GROUND` | `SUCCESS`、上锁、`ON_GROUND` |

各航点在进入下一段前的采样误差如下；连续保持条件为 2.0 s，表中 CSV 采样通常记录到 1.8–1.95 s 后下一状态已切换：

| 目标 | 第1次位置/yaw | 第2次位置/yaw |
|---|---:|---:|
| WP1 | 0.038 m / 0.009° | 0.035 m / 0.009° |
| WP2 | 0.026 m / 0.007° | 0.029 m / 0.009° |
| WP3 | 0.008 m / 0.014° | 0.012 m / 0.011° |
| WP4 | 0.080 m / 0.008° | 0.077 m / 0.008° |
| WP5 | 0.034 m / 0.011° | 0.097 m / 0.029° |
| WP6 | 0.048 m / 0.010° | 0.045 m / 0.015° |
| WP7 | 0.063 m / 0.008° | 0.059 m / 0.000° |
| WP8 | 0.048 m / 0.020° | 0.043 m / 0.018° |
| 闭环回 WP1 | 0.040 m / 0.002° | 0.079 m / 0.019° |

最终结果中的 `max_reference_position_error_m≈1.15 m` 包含起飞初始参考建立瞬态；`max_yaw_error_deg≈90°` 包含在 home 起飞后从初始朝向转向塔的预期首轮 yaw 转动。它们不是航点到达误差，因此另在表中给出了 MISSION 段和航点保持时的实际数据。

证据文件：

- 悬停：`/tmp/astra_stage1_evidence/stage1_hover_only_1_20260721_145030.csv`
- 单点：`/tmp/astra_stage1_evidence/stage1_mission_1_20260721_145710.csv`
- 四点：`/tmp/astra_stage1_evidence/stage1_mission_4_20260721_150813.csv`
- 八点第1次：`/tmp/astra_stage1_evidence/stage1_mission_8_20260721_152324.csv`
- 八点第2次：`/tmp/astra_stage1_evidence/stage1_mission_8_20260721_154009.csv`

### 9.4 一次有价值的失败记录

最初只启动 PX4/Gazebo、未启动 FAST-LIO 的悬停尝试被 preflight 正确拦住，PX4 报告 `ekf2 missing data`，从未解锁。随后一键脚本加入 FAST-LIO 窗口，并在任务启动前等待 `/Odometry/header`；之后所有分级任务通过。对应失败 CSV 是 `stage1_hover_only_1_20260721_144719.csv`，不能算成功证据。

## 10. 常见错误和排查方法

| 现象 | 优先检查 | 处理原则 |
|---|---|---|
| `ekf2 missing data`、拒绝解锁 | `/livox/imu`、`/Odometry`、`/mavros/local_position/pose` 是否有新消息 | 先修复传感器/FAST-LIO启动链，绝不绕过 preflight |
| 启动脚本报告可能冲突进程 | `pgrep -a -x` 输出和原启动终端/tmux | 查清归属，按原启动方式停止；不要粗暴 `killall` |
| 控制权冲突 | 对五类 MAVROS setpoint Topic 执行 `rostopic info` | 保证只有 `/tower_mission`；停止 autoarming 或 bridge 控制模式 |
| 一直 `WAIT_INPUTS` | `/mavros/state`、extended state、pose、velocity 的频率/有限值 | 读 tower_mission 日志中的具体 preflight 原因 |
| 一直 `ARM_OFFBOARD` | PX4 preflight、setpoint 预发送、模式/解锁服务 | 不减小安全门限来掩盖根因 |
| 进场后不开始圆周 | position error、内部参考是否已到 CP1、`waypoint_timeout` | 检查入口坐标和位置跟踪；检查点本身不等待 hold/yaw |
| 圆周仍像八边形 | `/tower_mission/route_preview` 与实际 setpoint | 确认重新编译并重启新节点；Path 应至少180段 |
| yaw 背对塔 | 塔中心、`camera_yaw_offset_deg`、相机安装方向 | 先 preview，不要在控制运行中试错 |
| 路线镜像或方向反 | ENU、`direction`、起始角 | ROS 任务层不手工做 NED 翻轴 |
| 高度或半径非法 | 启动日志中的参数校验错误 | 半径必须满足 `R - collision_radius >= minimum_safety_distance`，高度必须在上下限内 |
| Gazebo端口占用 | 是否已有本项目 roslaunch/gzserver/px4，`ss` 和精确进程信息 | 先结束拥有它的 launch；不要同时开两套仿真 |
| 节点重名 | `rosnode list`、`rosnode info /tower_mission` | 一个 ROS master 只启动一个阶段1会话 |
| 找不到包/节点 | `rospack find astra_tower_mission` | 按顺序 source；改 C++ 后重新编译 |

## 11. 阶段1与阶段2的关系

阶段1已经验证“任务层给绝对目标，基础执行层安全完成整套 OFFBOARD 流程”。阶段2才会把任务目标交给 EGO，以 FAST-LIO 点云建立局部地图，让 EGO 输出带位置、速度、加速度和 yaw 的动态轨迹，并由专门的 raw `PositionTarget` 执行器跟踪。

不能把阶段1的圆周位置 setpoint 包装成“已经接入 EGO”或“已经避障”。进入阶段2前仍需：正式确认 `map/camera_init/body/base_link/sensor` 外参、解决目标 z 被 EGO `manual_target_height` 覆盖的语义冲突、验证静态障碍/断流/不可达目标，并保持唯一 MAVROS 控制出口。本文完成后停止，等待项目负责人检查和亲自提交。

# 如果我想实现这个功能，我应该修改哪些参数

默认参数集中在：

`/home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/config/stage1_tower.yaml`

除非表中另有说明，修改 YAML 后不需要重新编译，但必须安全停止并重新启动节点。角度单位是度，代码内部转换为弧度。

### | 参数名 | 所在文件 | 当前值 | 单位 | 作用 | 调大/调小的影响 | 建议范围 | 是否需要重新编译 |
|---|---|---:|---|---|---|---|---|
| `tower/name` | `stage1_tower.yaml` | `radio_tower` | - | 日志和结果中的塔名 | 只改标签不会移动塔；必须与中心一起审查 | 与 world/测绘名称一致 | 否 |
| `tower/frame_id` | `stage1_tower.yaml` | `map` | frame | 塔和路线坐标系 | 不能随意改；改后所有坐标/TF契约都变 | 当前只能 `map` | 否，但需架构审查 |
| `tower/center/x` | `stage1_tower.yaml` | -17.4209 | m | 塔中心 x | 平移整条路线 | 必须来自审计/测绘 | 否 |
| `tower/center/y` | `stage1_tower.yaml` | 22.29 | m | 塔中心 y | 平移整条路线 | 必须来自审计/测绘 | 否 |
| `tower/collision_radius` | `stage1_tower.yaml` | 6.41 | m | 沿用旧4–5 m切片的包络，8 m尚待复核 | 调大更保守并可能拒绝半径；调小可能撞塔 | 8 m实测值或更保守值 | 否 |
| `mission/radius` | `stage1_tower.yaml` | 10.0 | m | 无人机中心到塔中心半径 | 调大离塔远但路程长；调小更危险 | 当前 world 首测 10–15 | 否 |
| `mission/height` | `stage1_tower.yaml` | 8.0 | m | 绝对 `map` 高度 | 调高可能遇到塔结构/地图上限；调低接近地面 | 当前默认8，换高度须重审 | 否 |
| `mission/waypoint_count` | `stage1_tower.yaml` | 8 | 个 | 一圈的等角度检查点数 | 只影响检查事件和进度，不改变圆周几何或速度 | 正式任务8；1/4仅分级测试 | 否 |
| `mission/start_angle_deg` | `stage1_tower.yaml` | -90.0 | deg | 第一点相对塔中心的方位 | 改变进场和闭环位置 | `[-180,180]` 易读，其他值会归一化 | 否 |
| `mission/direction` | `stage1_tower.yaml` | `counter_clockwise` | - | 航点角度增减方向 | `clockwise` 顺时针；默认值逆时针 | 两者之一 | 否 |
| `mission/camera_yaw_offset_deg` | `stage1_tower.yaml` | 0.0 | deg | 相机光轴相对机体+X偏角 | 正值会让机体 yaw 反向补偿同样角度 | 按标定，通常 `[-180,180]` | 否 |
| `mission/maximum_speed` | `stage1_tower.yaml` | 0.30 | m/s | 水平/三维参考最大速度 | 调大更快但制动距离和误差增大；调小更稳但可能触发超时 | 初次 0.2–0.5 | 否 |
| `mission/maximum_acceleration` | `stage1_tower.yaml` | 0.50 | m/s² | 参考加速度上限 | 调大响应快但更激烈；调小更平滑但更慢 | 初次 0.3–1.0 | 否 |
| `mission/maximum_yaw_rate_deg_s` | `stage1_tower.yaml` | 30.0 | deg/s | 机头转速上限 | 调大转向快；调小可能 yaw 到达较慢 | 15–45 | 否 |
| `mission/position_tolerance` | `stage1_tower.yaml` | 0.40 | m | 圆周入口及返航等位置到达容差 | 调大易进入下一阶段但精度差；调小可能超时 | 0.25–0.60 | 否 |
| `mission/yaw_tolerance_deg` | `stage1_tower.yaml` | 12.0 | deg | 起飞、悬停、返航等到达条件的 yaw 容差 | 不控制检查点停留；绕塔 yaw 误差用于观测 | 5–15 | 否 |
| `mission/waypoint_timeout` | `stage1_tower.yaml` | 240.0 | s | 到圆周入口的进场上限 | 调大更能容忍慢速；调小更快失败 | 120–300，须匹配距离/速度 | 否 |
| `mission/mission_timeout` | `stage1_tower.yaml` | 600.0 | s | 进场和完整圆周总上限 | 调大允许更慢轨迹；调小可能中途失败 | 480–900 | 否 |
| `mission/overall_timeout` | `stage1_tower.yaml` | 1200.0 | s | 从等待到结束的总上限 | 调大容忍启动/服务慢；调小更快终止 | 900–1800 | 否 |
| `mission/minimum_height` | `stage1_tower.yaml` | 2.0 | m | 航线高度下限 | 调高排除低飞；调低减少地面保护 | 按 world/法规/定位确定 | 否 |
| `mission/maximum_height` | `stage1_tower.yaml` | 10.0 | m | 航线高度上限 | 调高允许高飞；调低更保守 | 当前仿真不建议超过10 | 否 |
| `mission/minimum_safety_distance` | `stage1_tower.yaml` | 2.0 | m | 航线半径减塔包络后的最小余量 | 调大更保守；调小风险增加 | 首测至少2 | 否 |
| `mission/maximum_home_distance` | `stage1_tower.yaml` | 60.0 | m | home到任何航点的包络 | 调大允许更远任务；调小会拒绝当前路线 | 略高于几何最大距离 | 否 |
| `flight/takeoff_height` | `stage1_tower.yaml` | 8.0 | m | home上方起飞目标 | 调大起飞更高更久；调小需仍安全 | 当前默认8，须与任务高度协调 | 否 |
| `flight/takeoff_tolerance` | `stage1_tower.yaml` | 0.25 | m | 起飞高度到达容差 | 同位置容差影响 | 0.2–0.5 | 否 |
| `flight/takeoff_hold_time` | `stage1_tower.yaml` | 1.0 | s | 起飞到达连续保持 | 调大更稳、调小更快 | 1–3 | 否 |
| `flight/takeoff_timeout` | `stage1_tower.yaml` | 60.0 | s | 起飞阶段超时 | 过小会误失败；过大延迟故障终止 | 45–120 | 否 |
| `flight/initial_hover_duration` | `stage1_tower.yaml` | 3.0 | s | 入塔前稳定悬停 | 调大更稳但任务更久 | 2–10 | 否 |
| `flight/initial_hover_timeout` | `stage1_tower.yaml` | 30.0 | s | 初始悬停阶段超时 | 应大于悬停时长并留误差收敛余量 | 20–60 | 否 |
| `flight/return_height` | `stage1_tower.yaml` | 8.0 | m | home上方返航高度 | 与障碍和起飞高度协调 | 当前默认8，须审查返程净空 | 否 |
| `flight/return_hold_time` | `stage1_tower.yaml` | 2.0 | s | home上方连续保持 | 调大更稳但更久 | 1–5 | 否 |
| `flight/return_timeout` | `stage1_tower.yaml` | 240.0 | s | 返航超时 | 需匹配最远点和速度 | 180–360 | 否 |
| `flight/preland_hover_duration` | `stage1_tower.yaml` | 3.0 | s | 下降前稳定时间 | 调大更稳、调小更快 | 2–10 | 否 |
| `landing/cruise_speed` | `stage1_tower.yaml` | 0.40 | m/s | 高处OFFBOARD下降速度 | 调大降得快且风险增；调小更慢 | 0.2–0.5 | 否 |
| `landing/touchdown_speed` | `stage1_tower.yaml` | 0.20 | m/s | 近地下降速度 | 调大接地更重；调小可能更易受噪声影响 | 0.10–0.25 | 否 |
| `landing/slowdown_height` | `stage1_tower.yaml` | 1.20 | m | 开始减速高度 | 调大更早减速；调小更晚 | 0.8–2.0 | 否 |
| `landing/flare_height` | `stage1_tower.yaml` | 0.30 | m | 近地最终速度目标高度 | 调大更保守；调小更贴地才变慢 | 0.2–0.5 | 否 |
| `landing/timeout` | `stage1_tower.yaml` | 90.0 | s | 整个降落上限 | 调大容忍慢降；调小可能未接地就失败 | 60–150 | 否 |
| `safety/prestream_duration` | `stage1_tower.yaml` | 3.0 | s | 请求OFFBOARD前预发送时间 | 调大等待更久更稳；不建议调小 | 2–5 | 否 |
| `safety/input_timeout` | `stage1_tower.yaml` | 2.0 | s | MAVROS输入新鲜度 | 调大掩盖断流；调小对调度抖动敏感 | 1–3 | 否 |
| `safety/home_origin_tolerance` | `stage1_tower.yaml` | 2.0 | m | 实际home与期望spawn差 | 调大可能掩盖坐标错位；调小可能因初始化漂移拒绝 | 1–3 | 否 |

YAML 中还有接触确认、disarm 重试、循环频率、CSV 周期以及 Topic/服务名。初学时不要只为“让它通过”而放宽 safety 参数。

## 修改示例

修改前先复制 YAML 作为可回退的个人备份，改后先运行 preview。

### 让无人机离塔更远

```yaml
mission:
  radius: 12.0
```

### 降低飞行速度

```yaml
mission:
  maximum_speed: 0.20
```

速度降低后要检查 `waypoint_timeout`、`mission_timeout` 和 `overall_timeout` 是否仍足够。

### 临时把检查角改成12个（不改变圆周几何）

永久默认值：

```yaml
mission:
  waypoint_count: 12
```

临时预览/运行也可使用 `--waypoints 12`，它会覆盖 YAML：

```bash
scripts/run_sh/stage1_tower.sh --preview --waypoints 12
```

### 切换顺时针

```yaml
mission:
  direction: clockwise
```

### 修改绕塔高度

```yaml
mission:
  height: 5.0
```

同时确认 5 m 在 `minimum_height`、`maximum_height` 内，并重新审查该高度切片的塔外伸结构和安全余量。

### 保持8个不停留检查点

```yaml
mission:
  waypoint_count: 8
```

当前实现没有 `waypoint_hold_time`：检查点只记录通过事件，不能通过 YAML 恢复逐点停留。改变检查点数量也不会把圆周变成多边形；正式任务保持8，1/4只用于分级测试。

# 我怎么用终端重新启动

## 1. 先理解“重新编译”和“重新启动”

- 重新编译：把 C++ 源码变成新的可执行文件。改 `.cpp`、`.h`、`CMakeLists.txt` 或 `package.xml` 后需要。
- 重新启动：停止旧进程，再启动新进程。改 YAML 或 launch 通常只需重新启动，因为 ROS 参数在节点启动时加载。
- 编译不会自动替换正在运行进程的内存；改 C++ 后必须“先停止 → 编译 → 再启动”。

## 2. 第一次运行前如何编译

项目有下层仿真工作空间和上层主工作空间。按这个顺序：

```bash
source /opt/ros/noetic/setup.bash

cd /home/yanzu/AstraDroneOpen/simulation/sim_workspace
catkin_make -j2
source devel/setup.bash

cd /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES='offboard;astra_tower_mission' -j2
```

构建会写 `build/` 和 `devel/`，不要手改这些生成目录。

## 3. 修改C++后如何重新编译

先安全停止当前阶段1会话：

```bash
/home/yanzu/AstraDroneOpen/scripts/run_sh/stage1_tower.sh --stop
```

再编译并测试：

```bash
source /opt/ros/noetic/setup.bash
source /home/yanzu/AstraDroneOpen/simulation/sim_workspace/devel/setup.bash
cd /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES='offboard;astra_tower_mission' -j2
catkin_make -DCATKIN_WHITELIST_PACKAGES='offboard;astra_tower_mission' \
  run_tests_offboard run_tests_astra_tower_mission -j2
catkin_test_results build/test_results
```

通过后再启动。不要在控制任务飞行中覆盖可执行文件。

## 4. 只修改YAML或launch后是否需要重新编译

不需要重新编译，但需要停止并重新启动 `/tower_mission`，因为参数只在启动时读取。最安全流程：

```bash
cd /home/yanzu/AstraDroneOpen
scripts/run_sh/stage1_tower.sh --stop
scripts/run_sh/stage1_tower.sh --preview --waypoints 8 --attach
```

先重新 preview，确认参数，再决定是否进行控制测试。

## 5. 每个新终端需要source什么

普通 ROS/阶段1终端按以下顺序：

```bash
source /opt/ros/noetic/setup.bash
source /home/yanzu/AstraDroneOpen/simulation/sim_workspace/devel/setup.bash
source /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash --extend
cd /home/yanzu/AstraDroneOpen
```

一键脚本会自行做这些 source。手工启动 PX4/Gazebo 的终端还要再执行：

```bash
source /home/yanzu/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
  /home/yanzu/PX4-Autopilot \
  /home/yanzu/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:/home/yanzu/PX4-Autopilot:/home/yanzu/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
```

## 6. 使用一键脚本重新启动阶段1

最常用的完整重启流程：

```bash
cd /home/yanzu/AstraDroneOpen

# 只停止本脚本拥有的 stage1_tower tmux 会话
scripts/run_sh/stage1_tower.sh --stop

# 先做无解锁预览
scripts/run_sh/stage1_tower.sh --preview --waypoints 8 --attach
```

预览确认后，退出 attach 视图可按 `Ctrl-b` 再按 `d`，随后：

```bash
cd /home/yanzu/AstraDroneOpen
scripts/run_sh/stage1_tower.sh --stop
scripts/run_sh/stage1_tower.sh --control --mode mission --waypoints 8 --attach
```

也可用 `--restart` 让脚本只重启自己拥有的会话：

```bash
scripts/run_sh/stage1_tower.sh --control --mode mission --waypoints 8 --restart --attach
```

不要把 `--restart` 当成系统级清理工具；如果冲突进程不是本脚本创建的，它会拒绝误杀。

## 7. 如果没有一键脚本：分别启动各终端

优先使用一键脚本。若需要理解底层顺序，可手工启动；每个终端都先执行前述普通 source。

### 终端1：PX4 SITL、Gazebo和MAVROS

在普通 source 后再 source PX4 Gazebo 环境，然后：

```bash
roslaunch /home/yanzu/AstraDroneOpen/simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch \
  world:=/home/yanzu/AstraDroneOpen/simulation/astra_gazebo_worlds/forest.world \
  gui:=true interactive:=false
```

### 终端2：FAST-LIO

先确认 MID360 IMU 有数据：

```bash
rostopic echo -n 1 /livox/imu/header
roslaunch fast_lio mapping_mid360.launch rviz:=false
```

### 终端3：阶段1任务管理器和RViz

先确认三项输入：

```bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /Odometry/header
rostopic echo -n 1 /mavros/local_position/pose
```

然后才运行：

```bash
roslaunch astra_tower_mission stage1_tower.launch \
  enable_control:=true run_mode:=mission waypoint_count:=8 rviz:=true \
  report_file:=/tmp/astra_stage1_manual.csv
```

手工启动时按相反顺序在各自终端按一次 `Ctrl-C`：终端3 → 终端2 → 终端1，并等待 roslaunch 完成清理。

## 8. 如何确认各组件正常启动

新开一个已 source 的检查终端：

```bash
# ROS master 与节点
rosnode info /rosout
rosnode list | sort

# Gazebo 仿真时钟和服务
rostopic echo -n 1 /clock
rosservice list | rg '^/gazebo/(get_world_properties|pause_physics|unpause_physics)$'

# PX4/MAVROS连接、模式、解锁和落地状态
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/extended_state

# FAST-LIO
rostopic hz /Odometry
rostopic hz /cloud_registered

# 任务管理器
rosnode info /tower_mission
rostopic echo -n 1 /tower_mission/state
rostopic echo -n 1 /tower_mission/report_file
```

`rostopic hz` 会持续运行，看到稳定频率后按 `Ctrl-C`。任务开始前，日志应先打印塔名、中心、半径、高度、速度和安全距离；这是首次解锁前人工复核点。

## 9. 如何检查关键Topic、TF和唯一setpoint发布者

```bash
# 关键定位和控制消息
rostopic type /mavros/local_position/pose
rostopic hz /mavros/local_position/pose
rostopic hz /mavros/setpoint_position/local

# FAST-LIO TF；出现连续数值后按 Ctrl-C
rosrun tf tf_echo camera_init body

# 阶段1的 map 是 MAVROS消息 frame，不伪造 map->camera_init 标定
rostopic echo -n 1 /mavros/local_position/pose/header
rostopic echo -n 1 /tower_mission/route_preview/header
```

逐一检查所有受监控控制出口：

```bash
rostopic info /mavros/setpoint_position/local
rostopic info /mavros/setpoint_position/global
rostopic info /mavros/setpoint_position/global_to_local
rostopic info /mavros/setpoint_raw/local
rostopic info /mavros/setpoint_velocity/cmd_vel
rostopic info /mavros/setpoint_velocity/cmd_vel_unstamped
rostopic info /mavros/setpoint_attitude/attitude
rostopic info /mavros/setpoint_attitude/cmd_vel
rostopic info /mavros/setpoint_raw/attitude
rostopic info /mavros/setpoint_attitude/thrust
```

position 和 raw-local 的 `Publishers` 应只有 `/tower_mission`；其余控制 Topic 应没有外部发布者。若看到 `/autoarming_control`、`/ego_mavros_bridge` 或未知节点，停止任务并查明来源。

## 10. 如何正常结束任务和关闭全部阶段1进程

正常情况先等：

```bash
rostopic echo -n 1 /tower_mission/final_result
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/extended_state
```

确认 `SUCCESS`、`armed: False`、`landed_state: 1` 后：

```bash
/home/yanzu/AstraDroneOpen/scripts/run_sh/stage1_tower.sh --stop
```

`--stop` 只向 `stage1_tower` tmux 会话的窗口发送 `Ctrl-C`，等待 roslaunch 清理，再结束该会话。它不会用模糊名称强杀系统进程。

## 11. 如何检查并清理残留，避免误杀无关程序

先只读检查：

```bash
tmux list-sessions
rosnode list 2>/dev/null | sort

for name in rosmaster rosout gzserver gzclient px4 mavros fastlio_mapping tower_mission_node stage1_rviz; do
  pgrep -a -x "$name" || true
done

ss -ltnup | rg ':(11311|14540|14557)\b' || true
```

若脚本创建的会话还在，再执行一次 `stage1_tower.sh --stop`。若发现其他会话或进程：

1. 先用 `ps -o pid,ppid,lstart,args -p <PID>` 确认完整命令、父进程和启动时间；
2. 回到拥有该进程的原终端或 tmux，对对应 roslaunch 按 `Ctrl-C`；
3. 只有确认某个精确 PID 属于本次残留后，才可 `kill -INT <PID>`；等待后仍不退出再考虑 `kill -TERM <PID>`；
4. ROS master 已退出但 `rosnode list` 显示陈旧注册时，可在 master 恢复后执行 `rosnode cleanup`，逐项确认。

禁止使用 `killall gzserver`、`pkill -f roslaunch`、模糊正则或递归杀进程，这些命令可能终止别人的仿真、其他项目 ROS master 或无关任务。

## 12. 常见重启问题

- 端口占用：ROS master 常用 11311；Gazebo master 常用 11345 系列但可变化。用 `ss` 找精确 PID，再追溯原 launch，不要直接抢占。
- Gazebo残留：常见原因是直接关闭终端或 GUI。优先回到 roslaunch/tmux 发送 `Ctrl-C`，等模型和插件卸载。
- 节点重名：第二套 `/tower_mission` 会把第一套挤下 ROS graph，并触发控制风险。一键脚本在启动前检查自己的 tmux 和常见精确进程名。
- 忘记 source：表现为 `package not found`、找不到节点或消息。依次 source ROS → 下层仿真 workspace → 主 workspace。
- 修改 YAML 没生效：旧节点仍持有旧参数。执行 `--stop` 后重启，并用 `rosparam get /tower_mission/mission` 核对。
- tmux attach 后不知道退出：`Ctrl-b`，松开，再按 `d` 是 detach，不会停止任务；真正停止必须用 `--stop`。
- `/use_sim_time` 下时间不走：检查 `/clock` 和 Gazebo 是否暂停；不要移除仿真时间保护。
- 一键脚本拒绝启动：它检测到可能冲突的精确进程名。先确认进程归属，这是安全门，不应绕过。

最常用、最安全的一套流程就是：`--stop` → 编译（仅 C++ 改动需要）→ `--preview` → `--stop` → 显式 `--control` → 等 `SUCCESS`/上锁/落地 → `--stop` → 精确残留检查。
