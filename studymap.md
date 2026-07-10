# AstraDroneOpen 仿真阶段无人机自主控制学习路线

## 1. 路线定位

本文只面向当前仓库中的仿真学习，不覆盖真机接线、实机解锁、硬件标定、机载部署或飞行安全审批。最终目标是循序渐进地掌握：

```text
看懂并稳定启动仿真
  -> 复现默认 Offboard 飞行
  -> 起飞、悬停、降落
  -> 航点与动作状态机
  -> 可控速度/航向的轨迹跟踪
  -> 参数调试与量化验证
  -> 理解仿真感知和定位
  -> 单独掌握 EGO-Planner
  -> 将规划器接入 PX4/Gazebo
  -> 静态/动态场景中的自主任务
```

本文依据当前仓库的 `README.md`、`spec.md`、仿真资源、Offboard 源码、FAST-LIO 配置和 EGO-Planner 源码编写。文中遵循以下约定：

- **仓库事实**：当前源码、launch、world、model 或 config 可以直接确认。
- **根据代码结构推测**：仓库没有完整说明或尚未形成可运行闭环，仅依据模块边界、话题或命名提出的学习/实现方向。
- **推荐验收阈值**：为了让学习结果可以检查而给出的建议值，不是仓库原作者声明的性能指标；应根据本机实时因子和仿真结果调整。

为缩短后文路径，`offboard/` 指 `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/`，`plan_manage/` 指 `AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/`，`FAST_LIO/` 指 `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/`，`dynamic_obstacle_controller/` 指 `simulation/sim_workspace/src/dynamic_obstacle_controller/`。

## 2. 先建立正确的项目认识

### 2.1 当前最适合学习的主控制链

当前默认演示的实际主链是：

```text
scripts/run_sh/pc_example.sh
  -> PX4 SITL + Gazebo + iris_mid360
  -> MAVROS
  -> offboard/autoarming_control.cpp
  -> /mavros/setpoint_position/local
  -> PX4 内部位置/速度/姿态控制器
  -> Gazebo 中的无人机模型
```

这条链适合先学习基础飞行控制，因为 `autoarming_control.cpp` 已经包含：等待 FCU、预发送 setpoint、切换 OFFBOARD、解锁、起飞、圆/方形轨迹、返回和上锁等基本流程。

需要特别区分三件事：

1. `autoarming_control.cpp` 负责生成上层位置目标和任务阶段，真正的姿态、电机等内环由 PX4 完成。初期不应把“修改位置 setpoint”与“重写底层飞控”混为一谈。
2. FAST-LIO 虽在 `pc_example.sh` 中启动，但默认 Offboard 控制器不订阅 FAST-LIO 的 `/Odometry`，所以它当前不是默认飞行闭环的一部分。
3. EGO-Planner 当前使用自己的 SO3 动力学模拟器，且相关包带有 `CATKIN_IGNORE`；它不会直接控制默认 PX4/Gazebo 无人机。

### 2.2 当前仓库中的重要边界

- 默认 `autoarming_control.launch` 中的 `speed` 和 `takeoff_height` 参数当前没有被源码读取；修改它们不会产生预期效果。
- 圆轨迹的 `radius` 在源码中读取，默认值为 `2.0`，但默认 launch 没有显式设置该参数。
- 默认控制器没有独立 `HOVER` 阶段；达到起飞高度后立即进入 `TRACKING`。
- `position_control` 当前存在参数未读取、构造函数进入无限循环、没有发布频率控制、没有 OFFBOARD/解锁流程等问题，不适合作为第一条可运行主线。
- `simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch` 的默认 world 路径拼接缺少 `/`。学习初期可用绝对 `world:=...` 参数绕过。
- PX4 自定义 launch、airframe 和 SDF 会复制到外部 `~/PX4-Autopilot`；修改仓库源文件后，实际运行的外部副本不一定自动更新。
- 任意时刻只应有一个节点持续发布 MAVROS setpoint。不得同时让 `autoarming_control`、`position_control` 和规划桥接节点控制同一架无人机。
- `build/`、`devel/` 是生成目录；学习和修改应以 `src/` 下的源文件为准。

## 3. 总体阶段表

| 阶段 | 学习主题 | 完成后能实现的效果 |
|---|---|---|
| 0 | 仓库、ROS 与仿真基线 | 能独立启动并解释 PX4/Gazebo/MAVROS 链路 |
| 1 | 默认 Offboard 闭环 | 能复现并观察自动起飞、圆/方形、降落 |
| 2 | 起飞、悬停与安全降落 | 能让无人机起飞后稳定定点悬停，再受控降落 |
| 3 | 航点与任务状态机 | 能执行“起飞—悬停—多个航点—返航—降落” |
| 4 | 轨迹速度、航向与形状 | 能按可解释的速度和 yaw 跟踪圆、方形、8 字等轨迹 |
| 5 | 参数调试与结果验证 | 能用 rosbag 和误差指标判断改动是否真的有效 |
| 6 | 模型、传感器与定位 | 能理解 Gazebo 真值、PX4 局部位姿与 FAST-LIO 位姿的差别 |
| 7 | EGO-Planner 独立仿真 | 能在其自带模拟器中设置目标并观察 B-spline 重规划 |
| 8 | EGO/PX4 静态避障闭环 | 能让 PX4/Gazebo 无人机跟随规划轨迹绕开静态障碍 |
| 9 | 自主任务与动态场景 | 能在任务逻辑驱动下处理动态障碍、超时、失败和结束条件 |

建议严格按顺序推进。阶段 0～5 是单机自主控制的基础主线；阶段 6～9 才逐步加入感知、规划和更高程度的自主性。

## 4. 阶段 0：建立可复现的仿真基线

### 阶段目标

不急于改控制代码，先能独立启动、停止、检查和解释 PX4 SITL、Gazebo、MAVROS、ROS 节点及话题之间的关系，并建立每次实验都可复现的操作习惯。

### 需要掌握的知识点

- ROS1 的 node、topic、service、parameter、launch、namespace、TF 和 `use_sim_time`。
- catkin 工作空间、`source devel/setup.bash`、源目录与生成目录的区别。
- Gazebo world、model/SDF、插件和物理步长的基本概念。
- PX4 SITL、MAVLink、MAVROS 的角色，以及 OFFBOARD 模式为什么要求连续 setpoint。
- `/mavros/state`、`/mavros/local_position/pose`、`/mavros/local_position/odom` 的基本含义。
- ENU/NED、世界系/机体系、位置/姿态四元数的基本区别；本阶段只要求能识别，不要求推导完整变换。
- 进程分开启动、`rosnode list`、`rostopic list/info/hz/echo`、`rosservice list`、`rosparam get`、`rqt_graph` 的使用。

### 本项目中应该重点阅读的文件/文件夹路径

- `README.md`
- `spec.md`
- `docs/00-AstraDrone开发教程.md`
- `docs/02-仿真源码详细介绍.md`
- `docs/03-Ros源码详细介绍.md`
- `scripts/run_sh/README.md`
- `scripts/run_sh/pc_example.sh`
- `simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch`
- `simulation/sim_workspace/src/env_map/launch/start.launch`
- `simulation/sim_workspace/src/env_map/launch/map_test.launch`
- `simulation/astra_gazebo_worlds/example.world`
- `simulation/astra_gazebo_worlds/forest.world`
- `simulation/px4_sim_files/px4_iris_sdf/iris_mid360/iris_mid360.sdf`

### 推荐做的仿真实践任务

1. 不使用一键脚本，分终端启动 `roscore` 和 PX4/Gazebo/MAVROS，以便观察每个进程的输出。
2. 通过绝对路径指定 world，绕过当前默认路径拼接问题，例如：

   ```bash
   roslaunch px4 astra_example.launch \
     world:=$HOME/AstraDroneOpen/simulation/astra_gazebo_worlds/example.world
   ```

3. 在不启动 Offboard 控制节点的情况下，检查：

   ```bash
   rostopic echo -n 1 /mavros/state
   rostopic echo -n 1 /mavros/local_position/pose
   rostopic hz /mavros/local_position/pose
   rostopic info /mavros/setpoint_position/local
   ```

4. 分别打开 `example.world` 和 `forest.world`，确认 world 切换实际生效，并记录 Gazebo 实时因子是否明显下降。
5. 用 `rospack find px4` 找到实际 PX4 包，理解为何仓库内 launch/SDF 与外部 PX4 副本可能不一致。
6. 画出一张自己的启动链和话题图，至少包含 PX4、Gazebo、MAVROS、控制节点、setpoint 和 local pose。

### 完成该阶段的验收标准

- 能不依赖 `pc_example.sh` 启动并关闭仿真。
- 能解释 PX4、Gazebo、MAVROS 各自负责什么。
- `/mavros/state` 显示已连接，局部位姿持续更新且没有 NaN。
- 能用 `rostopic info` 判断 setpoint 是否有发布者，并解释为何同一时间只能有一个控制发布者。
- 能指出仓库源文件、catkin 生成文件和外部 PX4 副本的区别。
- 能在命令行切换至少两个 world，并确认实际加载的场景正确。

### 可能需要修改的关键文件或参数位置

- 本阶段原则上不需要修改代码。
- 若以后修正默认 world 路径：`simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch` 中 `world` 参数应在 `$(find env_map)` 后补 `/`。
- 同类路径问题还出现在：
  - `simulation/sim_workspace/src/env_map/launch/map_test.launch`
  - `simulation/sim_workspace/src/dynamic_obstacle_controller/launch/astra_dynamic_avoidance_static.launch`
  - `simulation/sim_workspace/src/craic_sim/launch/astra_craic_2026.launch`
  - `simulation/sim_workspace/src/craic_sim/launch/astra_craic_2026_runtime.launch`
- 若修改 `simulation/px4_sim_files/**`，还要确认 `~/PX4-Autopilot` 中运行副本已同步。

## 5. 阶段 1：读懂并复现默认 Offboard 闭环

### 阶段目标

先不重写控制器，完整理解并复现当前默认的“连接—setpoint 预热—OFFBOARD—解锁—起飞—圆/方形—降落”流程，明确代码中哪些参数真实生效。

### 需要掌握的知识点

- PX4 进入 OFFBOARD 前持续发送 setpoint 的原因。
- ROS subscriber、publisher、service client 与 20 Hz 主循环。
- `mavros_msgs/State`、`geometry_msgs/PoseStamped`、解锁和模式切换服务。
- 位置设定点与 PX4 内部闭环的分工。
- `FlightPhase` 状态机、进入条件、完成条件和循环发布。
- 全局参数与私有参数：`nh.param(...)` 和 `nh_private.param(...)`。
- 圆/方形解析轨迹函数和当前位置到目标点的距离判断。

### 本项目中应该重点阅读的文件/文件夹路径

- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/README.md`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src/autoarming_control.cpp`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/launch/autoarming_control.launch`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/CMakeLists.txt`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/package.xml`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/rviz_config/drone_path.rviz`
- `scripts/run_sh/pc_example.sh`

### 推荐做的仿真实践任务

1. 先只启动 PX4/Gazebo/MAVROS，再单独启动：

   ```bash
   roslaunch offboard autoarming_control.launch rviz:=false
   ```

2. 观察 `/mavros/state` 的 `connected`、`mode` 和 `armed` 变化，记录从控制节点启动到进入 OFFBOARD 的时间。
3. 分别运行 `flight_mode=circle` 和 `flight_mode=square`，每次只飞一圈，并在空场使用较保守的高度和尺寸。
4. 在运行前后核对：

   ```bash
   rosparam get /flight_mode
   rosparam get /autoarming_control/hight
   rosparam get /autoarming_control/target_laps
   rostopic hz /mavros/setpoint_position/local
   ```

5. 做一个“参数是否生效”表：分别改变 `hight`、`target_laps`、`side_length`、`speed`、`takeoff_height`，用实际 setpoint/odom 证明结果。预期会发现后两个当前无效。
6. 阅读源码并按顺序口述各阶段，而不是只看 Gazebo 画面判断成功。

### 完成该阶段的验收标准

- 能连续复现至少三次完整的默认飞行流程，并保留终端输出或 rosbag。
- 能指出 setpoint 预热、OFFBOARD 请求、解锁、起飞、轨迹跟踪和降落对应的源码段。
- 能准确说明 `hight`、`target_laps`、`side_length`、`radius` 的实际作用。
- 能用代码证据说明 `speed` 和 `takeoff_height` 当前为什么不生效。
- 能解释 FAST-LIO 即使启动也没有参与当前 Offboard 控制闭环。

### 可能需要修改的关键文件或参数位置

- `offboard/launch/autoarming_control.launch`：
  - `/flight_mode`
  - `~target_laps`
  - `~hight`
  - `~side_length`
  - 可新增 `~radius`
- `offboard/src/autoarming_control.cpp`：
  - 参数读取区
  - `FlightPhase`
  - `get_square_position()`、`get_circle_position()`
  - TAKEOFF/TRACKING/LANDING 分支
- 本阶段不要通过设置 `radius=0` 伪造悬停，因为当前轨迹长度会变为 0，推进计算存在除零风险。

## 6. 阶段 2：实现基础起飞、悬停和安全降落

### 阶段目标

在默认主链上实现最小但可靠的自主动作：记录起飞点，垂直起飞到指定高度，稳定悬停一段时间，然后垂直降落并确认上锁。此阶段先追求安全、稳定和可解释，不追求复杂轨迹。

### 需要掌握的知识点

- 定点悬停本质：持续发布同一个合法位置/姿态 setpoint，由 PX4 完成底层闭环。
- 初始位姿有效性：连接 FCU 不等于已经收到第一帧 pose。
- 合法四元数；无旋转时至少应使用 `(x,y,z,w)=(0,0,0,1)`。
- 起飞点相对坐标与世界原点的区别。
- 高度误差、水平误差、到达门限、悬停计时和阶段超时。
- 主循环持续发布与阻塞式 `sleep` 的风险。
- 上锁服务结果、模式变化和异常时保持最后安全 setpoint。

### 本项目中应该重点阅读的文件/文件夹路径

- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src/autoarming_control.cpp`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/launch/autoarming_control.launch`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src/position_control.cpp`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src/position_control_lib.cpp`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/include/offboard/position_control.h`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/launch/position_control.launch`

阅读 `position_control*` 的目的主要是识别反例：构造函数阻塞、无循环限频、参数函数未调用、缺少 OFFBOARD/解锁状态机。推荐继续演进 `autoarming_control`，保持单一 setpoint 发布者。

### 推荐做的仿真实践任务

1. 为状态机增加真正的 `HOVER` 阶段：`TAKEOFF -> HOVER -> LANDING -> COMPLETED`。
2. 等到收到首帧有效 pose 后，再记录 `home_x/home_y/home_z`。
3. 起飞目标使用 `(home_x, home_y, home_z + takeoff_height)`，不要固定飞向世界原点。
4. 到达目标高度后悬停 10 秒、30 秒、60 秒，分别记录目标位置和实际位置。
5. 所有阶段持续发布 setpoint；悬停计时只决定状态转换，不停止发布。
6. 给每个阶段增加超时。超时时进入保持/降落等安全分支，而不是无限等待。
7. 验证在非零出生点启动时，无人机仍原地垂直起飞和降落。

### 完成该阶段的验收标准

- 能自主完成“原地起飞—定点悬停—原地降落—确认上锁”。
- 起飞前确实等待了有效 pose；所有发布的 `PoseStamped` 都有时间戳、frame 和合法四元数。
- 在非零出生点测试时，不会先横移到 `(0,0)`。
- 悬停期间没有退出 OFFBOARD，setpoint 频率稳定高于 PX4 所需的最低连续流要求。
- 推荐验收阈值：在无风空场悬停 30 秒，三轴最大位置误差先控制在 `0.2 m` 量级；这是学习建议，不是仓库性能承诺。
- 降落后根据 service response 和 `/mavros/state` 确认已经上锁，而不是只调用一次服务就宣布完成。

### 可能需要修改的关键文件或参数位置

- `offboard/src/autoarming_control.cpp`：
  - 增加 `have_pose` 标志和首帧等待。
  - 记录 `home_x/home_y/home_z`。
  - 为所有 pose 设置合法 orientation。
  - 扩展 `FlightPhase`，增加 `HOVER`。
  - 增加阶段进入时间、超时和上锁结果检查。
- `offboard/launch/autoarming_control.launch`：
  - 让 `takeoff_height` 真正被源码读取。
  - 建议新增 `hover_duration`、`takeoff_tolerance`、`landing_tolerance`、`phase_timeout`。
- 上述新增参数是**根据代码结构推测**的合理扩展，当前仓库没有这些完整实现。

## 7. 阶段 3：航点控制与自主任务状态机

### 阶段目标

从“固定悬停”发展到“按动作序列自主运动”：能够连续执行起飞、悬停、到达多个航点、等待、改变航向、返航和降落，并对每个动作设置完成条件与超时。

### 需要掌握的知识点

- 有限状态机：状态、进入动作、持续动作、退出条件、超时和错误分支。
- 航点队列、当前索引、位置到达门限、停留时间。
- 位置与 yaw 的联合目标；四元数和 yaw 角转换。
- 世界系航点、相对 home 航点和机体系动作的区别。
- 状态机与控制循环解耦：任务层决定“去哪”，控制输出层负责连续发 setpoint。
- 模式丢失、目标超时、定位失效时的保持/返航/降落策略。
- 单一控制权：任务节点不能与另一个 setpoint 节点同时控制同一话题。

### 本项目中应该重点阅读的文件/文件夹路径

- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src/autoarming_control.cpp`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src/position_control_lib.cpp`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/launch/position_control.launch`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/launch/autoarming_control.launch`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/launch/autoarming_Mult.launch`，只用于理解 namespace/remap，不建议此时做多机。
- `AstraDrone_ros1_ws/src/Control/rc_obstacle_avoidance/src/rc_obstacle_avoidance_node.cpp`，只借鉴目标缓存、超时和转发思路。

### 推荐做的仿真实践任务

1. 设计以下单机任务：

   ```text
   WAIT_POSE
     -> PRESTREAM
     -> ARM_AND_OFFBOARD
     -> TAKEOFF
     -> HOVER
     -> WAYPOINT_1
     -> WAIT
     -> WAYPOINT_2
     -> TURN
     -> RETURN_HOME
     -> LAND
     -> DISARM
   ```

2. 先实现三个相对 home 的航点，例如 `(1,0,1.5)`、`(1,1,1.5)`、`(0,1,1.5)`，逐点到达后停留 2～3 秒。
3. 增加固定 yaw 和“到点后转向”两种模式，并在 RViz/odom 中验证航向。
4. 为每个航点设置到达门限和超时；超时后进入 HOVER，而不是继续推进索引。
5. 把任务参数从硬编码逐步移到 launch/YAML。若使用 YAML，明确参数命名空间并用 `rosparam get` 验证。
6. 在运行中故意停止任务目标更新，验证最后安全 setpoint 仍会持续发布。

### 完成该阶段的验收标准

- 能一键执行完整航点任务，无需人工逐条发送目标。
- 每个阶段都有清晰日志，能看到进入时间、目标、误差、完成或超时原因。
- 航点只有在同时满足位置误差和停留时间后才算完成。
- 任一航点超时后不会索引越界或继续盲飞，能够进入预设的保持/返航/降落策略。
- 推荐验收阈值：简单空场中，每个航点稳定进入 `0.2 m` 半径并保持 2 秒，连续完成三次任务且不丢失 OFFBOARD。

### 可能需要修改的关键文件或参数位置

- 首选继续扩展 `offboard/src/autoarming_control.cpp`，避免同时维护两个争抢 setpoint 的节点。
- `offboard/launch/autoarming_control.launch` 可增加：航点列表、`waypoint_tolerance`、`waypoint_hold_time`、`waypoint_timeout`、`return_home`。
- **根据代码结构推测**：当任务规模变大时，宜把“OFFBOARD 会话/连续发布”和“任务状态机/航点队列”拆成类或独立模块，但当前仓库没有现成的完整任务管理器可直接复用。
- 不建议直接启用当前 `position_control`；若选择修复它，必须先补全参数读取、循环限频、合法 header/orientation、FCU/OFFBOARD/arming 和安全状态机。

## 8. 阶段 4：实现可控速度、航向和自定义轨迹跟踪

### 阶段目标

把当前“根据跟踪距离推进目标点”的轨迹逻辑改造成基于时间和弧长的参考轨迹，使 `speed` 具有明确的 m/s 含义，并逐步支持圆、方形、8 字、椭圆或自定义轨迹以及切线航向。

### 需要掌握的知识点

- 路径与轨迹的区别：路径只有几何形状，轨迹还包含时间、速度和加速度。
- 采样周期 `dt`、循环频率、弧长参数化、轨迹总长和圈数。
- 参考点速度与无人机实际速度的区别。
- 跟踪误差门限：误差过大时是否暂停参考点推进。
- 圆/方形的参数方程；方形拐角为何会产生速度/加速度突变。
- 8 字和椭圆的参数化；非匀速参数曲线为何需要弧长查表才能做到近似恒速。
- yaw、yaw rate、角度展开，以及机头沿轨迹切线的计算。
- 位置、速度、加速度前馈的概念；本阶段可以先用位置 setpoint，不必立即切换 `PositionTarget`。

### 本项目中应该重点阅读的文件/文件夹路径

- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src/autoarming_control.cpp`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/launch/autoarming_control.launch`
- `AstraDrone_ros1_ws/src/Planner/ego-planner/uav_simulator/Utils/waypoint_generator/src/sample_waypoints.h`，只作为轨迹形状参考。
- `AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/src/traj_server.cpp`，用于理解规划轨迹如何连续输出 position/velocity/acceleration/yaw。
- `AstraDrone_ros1_ws/src/Planner/ego-planner/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg`
- `scripts/run_sh/record.sh`

### 推荐做的仿真实践任务

1. 让源码真正读取 `speed` 和 `max_tracking_error`。
2. 用 `speed * dt / trajectory_length` 推进圆/方形轨迹参数；当实际跟踪误差过大时暂停推进。
3. 按 `0.5 m/s -> 1.0 m/s -> 1.5 m/s` 逐级实验，不要一次提高多项参数。
4. 将轨迹中心改为相对 home，而不是固定世界原点。
5. 依次实现并验证：
   - 固定 yaw 的圆；
   - 切线 yaw 的圆；
   - 方形及拐角减速；
   - 8 字；
   - **根据代码结构推测**：预采样并建立累计弧长表，使 8 字近似恒速。
6. 同时记录目标 pose、实际 odom、实际速度和 yaw，画出目标/实际轨迹对比。
7. 检查改变主循环频率后，设定的轨迹速度是否仍大致不变；若明显变化，说明实现仍依赖循环次数而不是真实时间。

### 完成该阶段的验收标准

- `speed` 的改变能从 setpoint 随时间的变化中被量化验证，而不是只凭肉眼感觉。
- 在 20 Hz 与另一种循环频率下，相同 `speed` 的总完成时间基本一致。
- 能解释参考速度、实际机体速度和位置误差三者关系。
- 所有轨迹的四元数合法；切线 yaw 连续，不在 `±π` 附近突然反转。
- 推荐验收阈值：空场、低速圆轨迹位置 RMSE 先达到 `0.3 m` 以内，且最大误差受控；该阈值是学习建议，应根据仿真实时因子调整。
- 轨迹结束后能可靠进入返航/降落阶段，不因 `t` 越界或圈数计算重复执行。

### 可能需要修改的关键文件或参数位置

- `offboard/src/autoarming_control.cpp`：
  - 参数区：`speed`、`max_tracking_error`、轨迹中心、方向、yaw 模式。
  - TRACKING 入口：保存 `last_track_time`。
  - TRACKING 推进逻辑：改为基于 `dt`。
  - 轨迹函数：新增 8 字/椭圆/自定义曲线。
  - orientation：设置固定 yaw 或切线 yaw。
- `offboard/launch/autoarming_control.launch`：
  - 让 `speed` 真正对应源码。
  - 新增 `radius`、`center_x/y`、`clockwise`、`yaw_mode`、`max_tracking_error`。
- **根据代码结构推测**：若以后要利用 EGO 的速度/加速度前馈，应改用 `mavros_msgs/PositionTarget`，而不只是 `PoseStamped`。

## 9. 阶段 5：参数调试、实验设计与结果验证

### 阶段目标

建立“提出假设—只改一个变量—录制数据—计算指标—比较结果”的调试方法，能区分启动问题、轨迹生成问题、PX4 跟踪问题、仿真实时性问题和定位问题。

### 需要掌握的知识点

- 单变量实验、基线、对照组、重复实验和参数快照。
- rosbag、`rosparam dump`、Git diff、world/模型版本记录。
- 位置 RMSE、最大误差、稳态误差、上升时间、超调、完成时间、速度/加速度峰值和 yaw 误差。
- Gazebo `max_step_size`、`real_time_update_rate`、实际 real-time factor 对实验的影响。
- setpoint 频率、轨迹速度、误差门限、到达门限和 PX4 响应之间的关系。
- 上层参数与底层动力学参数的边界：优先调轨迹和任务参数，最后才考虑质量、电机常数或 PX4 内环。
- 常见故障的定位顺序：连接 -> pose -> setpoint publisher -> setpoint 频率 -> OFFBOARD/armed -> 目标值 -> 实际响应。

### 本项目中应该重点阅读的文件/文件夹路径

- `scripts/run_sh/record.sh`
- `spec.md` 中“推荐操作流程”“常见现象与定位顺序”“PX4 机型与控制参数边界”部分
- `simulation/astra_gazebo_worlds/example.world`
- `simulation/astra_gazebo_worlds/dynamic_avoidance.world`
- `simulation/px4_sim_files/px4_iris_params/1046_gazebo-classic_iris_mid360`
- `simulation/px4_sim_files/px4_iris_sdf/iris_without_GPS/iris_without_GPS.sdf`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src/autoarming_control.cpp`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/launch/autoarming_control.launch`

### 推荐做的仿真实践任务

1. 为每次实验建立记录：Git 提交/差异、launch 参数、world、目标轨迹、实时因子和结果指标。
2. 扩展或单独使用 rosbag 命令，至少记录：
   - `/mavros/state`
   - `/mavros/setpoint_position/local`
   - `/mavros/setpoint_raw/local`（若以后使用）
   - `/mavros/local_position/pose`
   - `/mavros/local_position/odom`
   - `/tf`
3. 用同一条圆轨迹，只改变 `speed`，比较完成时间、RMSE、最大误差和速度峰值。
4. 用同一速度，只改变 `max_tracking_error`，观察目标暂停推进与实际滞后的关系。
5. 将 Gazebo GUI/RViz/FAST-LIO 分别关闭，比较实时因子和轨迹误差，判断性能瓶颈是否影响结果。
6. 建立一张调参表：参数、预期影响、实际影响、是否保留、证据文件。
7. 最后才做动力学敏感性实验，例如质量或电机响应时间；每次只改一项并能恢复基线。

### 完成该阶段的验收标准

- 不再用“看起来飞得不错”作为唯一结论，至少能输出目标/实际轨迹图和一组误差指标。
- 每项参数变更都有实验假设和结果，能判断改动是否真正生效。
- 连续运行同一任务至少五次，结果分布基本稳定；若不稳定，能从日志定位失败阶段。
- 能区分“参考轨迹太激进”“PX4 跟踪滞后”“定位输入异常”“Gazebo 运行过慢”四类问题。
- 推荐验收阈值：基础航点/低速轨迹连续五次无 OFFBOARD 丢失、无 NaN、无意外 setpoint publisher，且误差指标没有明显离群。

### 可能需要修改的关键文件或参数位置

- `scripts/run_sh/record.sh`：可按实验目的补充话题、输出目录和 bag 命名；不要重复记录同一话题。
- `offboard/launch/autoarming_control.launch`：集中管理实验参数。
- `offboard/src/autoarming_control.cpp`：把硬编码的误差门限、阶段超时和最大高度改为参数。
- `simulation/astra_gazebo_worlds/*.world`：物理步长、更新率和障碍物位置。
- `iris_without_GPS.sdf`：质量、惯量、电机时间常数、最大转速、推力/力矩常数。只有在上层控制已稳定、实验目标明确时再改。
- `1046_gazebo-classic_iris_mid360`：PX4 EKF/无 GPS airframe 参数。修改前先保存基线并确认外部 PX4 副本同步。

## 10. 阶段 6：理解仿真模型、传感器、定位和坐标系

### 阶段目标

在基础控制稳定后，再学习 Gazebo 模型如何产生 LiDAR/IMU 数据、FAST-LIO 如何输出里程计，以及 PX4 局部位姿与 FAST-LIO 位姿为什么不能直接假定完全一致。此阶段先把 FAST-LIO 当作观察和研究对象，不急于接管 PX4 控制闭环。

### 需要掌握的知识点

- SDF 的 include、link、joint、sensor、plugin 和 frame。
- LiDAR 扫描频率、范围、采样、噪声；IMU 更新率、噪声和外参。
- FAST-LIO 的 LiDAR/IMU 输入、外参、检测范围、下采样和输出 frame。
- Gazebo 真值、PX4 EKF/MAVROS local odom、FAST-LIO `/Odometry` 三类状态来源。
- `map`、`camera_init`、`body`、MAVROS local frame 之间的 TF 与初始原点。
- 外参、时间戳、初始 yaw、ENU/NED 变换错误可能造成的旋转、平移或高度偏置。

### 本项目中应该重点阅读的文件/文件夹路径

- `simulation/px4_sim_files/px4_iris_sdf/iris_mid360/iris_mid360.sdf`
- `simulation/px4_sim_files/px4_iris_sdf/iris_without_GPS/iris_without_GPS.sdf`
- `simulation/astra_gazebo_models/mid360/mid360.sdf`
- `simulation/sim_workspace/src/sensors/Mid360_simulation_plugin/`
- `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/launch/mapping_mid360.launch`
- `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/config/mid360.yaml`
- `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/src/laserMapping.cpp`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/launch/autoarming_control.launch` 中 `map -> camera_init` 静态 TF

### 推荐做的仿真实践任务

1. 只启动 PX4/Gazebo 和 FAST-LIO，不启动轨迹控制；检查 `/livox/lidar`、`/livox/imu`、`/Odometry` 和 TF。
2. 在静止状态比较 MAVROS local pose 与 FAST-LIO odom 的原点、轴向和 yaw。
3. 让无人机只沿世界 x、y、z 单轴运动，分别对比两套 odometry 的符号和尺度。
4. 在 RViz 同时显示点云、FAST-LIO path、MAVROS odom；记录是否存在固定旋转或平移。
5. 修改仿真雷达频率或范围做一次受控实验，验证配置变更从 SDF 到话题频率/点云范围确实生效。
6. 暂时不要把 `/Odometry` 直接 remap 给控制器；先完成静止和单轴对齐报告。

### 完成该阶段的验收标准

- 能从 SDF 找到 Mid360 安装位姿、LiDAR/IMU 话题、频率和噪声参数。
- 能从 FAST-LIO 配置找到输入话题、外参、探测范围和输出设置。
- 能画出 PX4 local frame 与 FAST-LIO frame 的关系，并用单轴实验支持结论。
- 能明确说明默认 `autoarming_control` 为什么不依赖 FAST-LIO。
- 在考虑接入规划器前，能证明 odom 与点云 frame、时间戳和初始原点相互一致或给出所需变换。

### 可能需要修改的关键文件或参数位置

- `simulation/astra_gazebo_models/mid360/mid360.sdf`：`update_rate`、range、noise、samples、IMU 参数和话题。
- `simulation/px4_sim_files/px4_iris_sdf/iris_mid360/iris_mid360.sdf`：雷达/相机安装位姿与 joint。
- `FAST_LIO/config/mid360.yaml`：LiDAR/IMU 话题、外参、FOV、检测距离和发布选项。
- `FAST_LIO/launch/mapping_mid360.launch`：滤波分辨率、迭代次数和 RViz。
- **根据代码结构推测**：若以后用 FAST-LIO 驱动规划/控制，需要新增或配置稳定的 frame 对齐层；当前仓库没有一条已确认可直接把 FAST-LIO `/Odometry` 接入默认 PX4 Offboard 控制器的完整链。

## 11. 阶段 7：单独掌握 EGO-Planner 自带仿真链

### 阶段目标

先把 EGO-Planner 当作一个独立系统学习：理解目标输入、占据地图、重规划 FSM、B-spline、轨迹服务器、SO3 控制器和自带动力学模拟器。此阶段不连接 PX4/Gazebo 无人机。

### 需要掌握的知识点

- 栅格占据地图、分辨率、膨胀、局部更新范围和虚拟天花板。
- 全局参考轨迹、局部 B-spline、速度/加速度/jerk 约束。
- 轨迹优化代价：平滑、碰撞、可行性、贴合。
- EGO 重规划 FSM、目标类型、重规划阈值和应急停止。
- `planning/bspline -> traj_server -> planning/pos_cmd -> SO3 control -> simulator -> odom`。
- RViz 2D Nav Goal、预设航点和 waypoint generator 内置轨迹。
- catkin 依赖与 `CATKIN_IGNORE` 的作用。

### 本项目中应该重点阅读的文件/文件夹路径

- `AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/launch/run_in_sim.launch`
- `AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/launch/advanced_param.xml`
- `AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/launch/simulator.xml`
- `AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/src/ego_replan_fsm.cpp`
- `AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/src/planner_manager.cpp`
- `AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/src/traj_server.cpp`
- `AstraDrone_ros1_ws/src/Planner/ego-planner/uav_simulator/Utils/waypoint_generator/src/waypoint_generator.cpp`
- `AstraDrone_ros1_ws/src/Planner/ego-planner/uav_simulator/Utils/waypoint_generator/src/sample_waypoints.h`
- `AstraDrone_ros1_ws/src/Planner/ego-planner/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg`
- `AstraDrone_ros1_ws/src/Planner/ego-planner/**/CATKIN_IGNORE`

### 推荐做的仿真实践任务

1. 先列出 EGO 所有被忽略的包和依赖，规划一个单独的启用/编译实验，不要与已稳定的 Offboard 基线同时改动。
2. 成功运行 `run_in_sim.launch` 自带的 mock map、SO3 controller 和动力学模拟器。
3. 使用 RViz 设置单目标，观察 `/planning/bspline` 和 `/planning/pos_cmd`。
4. 把 `flight_type` 改为预设航点模式，修改五个 waypoint，比较全局参考与局部重规划。
5. 依次只改变：`max_vel`、`max_acc`、地图分辨率、障碍膨胀、`dist0`，记录轨迹和规划失败率变化。
6. 修正或验证手动目标回调中 z 被硬编码为 `1.0` 的行为，再检查虚拟天花板 `2.5 m` 和地图 z 范围。
7. 试用 waypoint generator 的 `circle`、`eight`、`points/point`，注意 goal 回调和 trigger 回调的字符串命名不一致。

### 完成该阶段的验收标准

- 能独立启动 EGO 自带仿真，并明确它控制的是 `so3_quadrotor_simulator`，不是 PX4。
- 能从 RViz 目标触发轨迹，并观察 odom、B-spline 和 `PositionCommand` 连续更新。
- 能解释 `max_vel/max_acc`、地图膨胀、规划视野和优化权重的作用。
- 能让预设航点与手动目标的高度按预期生效。
- 能制造一次规划失败或应急重规划，并根据日志说明原因。

### 可能需要修改的关键文件或参数位置

- 各依赖包的 `CATKIN_IGNORE`：启用 EGO 前需要有计划地处理；实际最小依赖集合应以 catkin 构建结果为准。
- `plan_manage/launch/run_in_sim.launch`：地图大小、odom、感知输入、`max_vel/max_acc`、目标类型和预设航点。
- `plan_manage/launch/advanced_param.xml`：FSM、地图、膨胀、天花板、动力学限制和优化权重。
- `plan_manage/src/ego_replan_fsm.cpp`：手动目标 z 当前硬编码为 `1.0`。
- `waypoint_generator/src/sample_waypoints.h`：内置轨迹形状、高度和尺度。
- **根据代码结构推测**：启用 EGO 可能需要逐个补齐 catkin 依赖；仓库没有提供当前工作区已验证成功的一键最小启用清单。

## 12. 阶段 8：把 EGO-Planner 接入 PX4/Gazebo，完成静态避障

### 阶段目标

在 EGO 独立仿真和 PX4 基础控制都稳定后，构建唯一的一条规划控制闭环，让真实的 PX4/Gazebo 无人机使用规划器输出绕开静态障碍。

### 需要掌握的知识点

- 系统集成的输入输出契约：odom、点云、目标、轨迹命令和 frame。
- `quadrotor_msgs/PositionCommand` 到 MAVROS setpoint 的字段映射。
- `mavros_msgs/PositionTarget` 的坐标系、type mask、position/velocity/acceleration/yaw/yaw_rate。
- 规划器 odom 与 PX4 控制 odom 的一致性要求。
- 点云 frame、局部地图范围、障碍膨胀与无人机安全半径。
- EGO 自带模拟器与 PX4/Gazebo 不能同时作为受控对象。
- OFFBOARD 会话管理、轨迹新鲜度、超时悬停和唯一 setpoint publisher。

### 本项目中应该重点阅读的文件/文件夹路径

- `AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/launch/run_in_sim.launch`
- `AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/launch/simulator.xml`
- `AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/src/traj_server.cpp`
- `AstraDrone_ros1_ws/src/Planner/ego-planner/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src/autoarming_control.cpp`
- `AstraDrone_ros1_ws/src/Control/rc_obstacle_avoidance/src/rc_obstacle_avoidance_node.cpp`
- `AstraDrone_ros1_ws/src/Control/rc_obstacle_avoidance/config/rc_obstacle_avoidance.yaml`
- `AstraDrone_ros1_ws/src/Control/rc_obstacle_avoidance/launch/rc_obstacle_avoidance.launch`
- `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/config/mid360.yaml`
- `simulation/astra_gazebo_worlds/dynamic_avoidance.world`
- `simulation/sim_workspace/src/dynamic_obstacle_controller/launch/astra_dynamic_avoidance_static.launch`

`rc_obstacle_avoidance` 只能作为“规划输出超时检查与 MAVROS 转发”的参考：它当前依赖仓库中不存在的 Fast-Planner 话题约定，包被忽略，YAML 又被加载到全局命名空间而源码读取私有参数，不能直接当成 EGO/PX4 桥接器。

### 推荐做的仿真实践任务

1. 先做无障碍桥接：只验证 `PositionCommand -> MAVROS` 后 PX4 能跟随一条简单规划轨迹。
2. 将 EGO odom 输入改为 `/mavros/local_position/odom`，或使用阶段 6 已验证对齐的 FAST-LIO odom。
3. 将感知输入替换成实际仿真点云，并确认 frame、时间戳、范围和地图更新正常。
4. 停用 `simulator.xml` 中的 SO3 模拟器、SO3 控制器和自带 sensing 链，只保留真正需要的规划节点。
5. 实现桥接器，订阅 `/planning/pos_cmd`，持续发布 MAVROS raw local setpoint，并处理 OFFBOARD、解锁、超时和降落。
6. 运行桥接器时停掉 `autoarming_control` 的 setpoint 发布；用 `rostopic info` 验证唯一 publisher。
7. 先在单个静态障碍物场景测试，再进入 `dynamic_avoidance.world` 的静态模式。
8. 记录最小障碍距离、重规划次数、位置误差和任务完成时间。

### 完成该阶段的验收标准

- PX4/Gazebo 无人机而非 EGO 自带模拟器在运动。
- `/planning/pos_cmd` 到 MAVROS setpoint 的频率稳定，轨迹超时时自动悬停或进入安全状态。
- 控制话题只有一个发布者；OFFBOARD 在任务中没有意外丢失。
- 静态障碍被规划地图正确占据和膨胀，无人机能够绕行而非穿模。
- 目标、odom、点云和 setpoint 使用的 frame 关系有明确记录。
- 推荐验收标准：在固定静态场景重复五次，全部无碰撞到达目标，并满足事先设定的最小安全距离。

### 可能需要修改的关键文件或参数位置

- `plan_manage/launch/run_in_sim.launch`：odom、cloud、是否 include `simulator.xml`。
- `plan_manage/launch/advanced_param.xml`：frame、地图大小、局部范围、膨胀、速度/加速度和安全距离相关参数。
- **根据代码结构推测**：需要新增 `PositionCommand -> mavros_msgs/PositionTarget` 桥接节点；当前仓库没有完整可运行的 EGO/PX4 桥接实现。
- **根据代码结构推测**：桥接节点适合放在 Offboard/MissionControl 相关包中并复用已有 OFFBOARD 会话逻辑，但具体文件名和类结构应在实现阶段再确定。
- `rc_obstacle_avoidance.launch` 若用于参考改造，应把 `<rosparam>` 放入 `<node>` 内，使 YAML 进入私有命名空间；它的话题接口还必须从 Fast-Planner 改为 EGO 的 `PositionCommand`。

## 13. 阶段 9：自主任务逻辑、动态障碍与复杂场景

### 阶段目标

在“规划器能稳定控制 PX4 绕静态障碍”的基础上，增加更高层的自主任务决策：根据任务状态自动选择目标、等待、重试、绕行、返航或结束，并在动态障碍和复杂 world 中量化成功率。

### 需要掌握的知识点

- 分层自主系统：任务层、规划层、轨迹/控制桥接层、PX4 内环。
- 任务状态机与规划 FSM 的区别：任务层决定下一目标，规划器负责到达当前目标。
- 动态障碍预测与仅靠频繁重规划的边界。
- 目标可达性、规划超时、轨迹超时、碰撞风险、定位失效和模式丢失的处理。
- geofence、最大高度、最大速度、最小障碍距离和任务总超时。
- 成功率、失败原因分类、最小距离、重规划频率和完成时间等评价指标。
- 场景由简到繁：单静态障碍 -> 多静态障碍 -> 单动态障碍 -> 多动态障碍 -> CRAIC/森林。

### 本项目中应该重点阅读的文件/文件夹路径

- `simulation/astra_gazebo_worlds/dynamic_avoidance.world`
- `simulation/sim_workspace/src/dynamic_obstacle_controller/launch/astra_dynamic_avoidance_static.launch`
- `simulation/sim_workspace/src/dynamic_obstacle_controller/launch/astra_dynamic_avoidance_moving.launch`
- `simulation/sim_workspace/src/dynamic_obstacle_controller/config/obstacle_params.yaml`
- `simulation/sim_workspace/src/dynamic_obstacle_controller/src/obstacle_controller.py`
- `simulation/astra_gazebo_worlds/craic_2026.world`
- `simulation/sim_workspace/src/craic_sim/launch/astra_craic_2026.launch`
- `simulation/sim_workspace/src/craic_sim/launch/astra_craic_2026_runtime.launch`
- `simulation/sim_workspace/src/craic_sim/scripts/generate_craic_2026_world.py`
- `AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/src/ego_replan_fsm.cpp`
- `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src/autoarming_control.cpp`
- `AstraDrone_ros1_ws/src/Exploration/` 和 `AstraDrone_ros1_ws/src/Swarm/`，用于确认当前仓库没有可直接使用的完整自主探索/集群实现。

### 推荐做的仿真实践任务

1. 先启动动态障碍 world 的静态模式，确认规划和控制基线不受启动文件变化影响。
2. 启动 moving 模式，逐个启用障碍物；先把速度设低，再逐步增加。
3. 设计一个任务：起飞 -> 到达观察点 A -> 到达 B -> 如果规划失败则悬停并重试 -> 成功后返航 -> 降落。
4. 给任务增加总超时、单目标最大重试次数、最小安全距离告警和紧急悬停。
5. 用不同障碍轨迹测试：linear、circle、waypoint，并核对 YAML 中模型名与 world 中模型名完全一致。
6. 最后切换 CRAIC 或 forest world，先只用少量目标，再逐渐增加任务复杂度。
7. 对每类场景重复多次，统计成功率、失败阶段、最小障碍距离、重规划次数和完成时间。

### 完成该阶段的验收标准

- 任务可以在无人逐条发目标的情况下自动执行到结束。
- 规划失败或轨迹超时不会导致旧指令无限继续，任务层能进入悬停、重试、返航或降落分支。
- 动态障碍模型按 YAML 设定运动，规划器能感知到其变化；不能只根据 Gazebo 画面猜测。
- 每次失败都能归类为感知、定位、规划、桥接、PX4 模式或任务逻辑问题。
- 推荐验收标准：为一个固定动态场景定义明确的成功条件并重复至少十次；先达到可接受的稳定成功率，再提高障碍速度或场景复杂度。

### 可能需要修改的关键文件或参数位置

- `dynamic_obstacle_controller/config/obstacle_params.yaml`：`update_rate`、障碍物类型、中心、振幅、半径、速度和航点。
- `dynamic_obstacle_controller/src/obstacle_controller.py`：障碍轨迹计算和 Gazebo model state 更新。
- `dynamic_avoidance.world`：静态/动态模型位置、尺寸和名称。
- `craic_sim/scripts/generate_craic_2026_world.py`：场地、障碍和坐标换算。
- EGO 的地图范围、更新范围、膨胀、重规划阈值、速度/加速度参数。
- **根据代码结构推测**：完整任务管理器需要在 MissionControl 中新增或重构；当前 `Exploration/` 只有说明占位，仓库没有可直接运行的自主探索闭环，不应把 README 的模块描述当成现成功能。
- 多机属于后续独立课题；在单机动态任务稳定前不建议进入 `autoarming_Mult.launch`、端口和 namespace 调试。

## 14. 贯穿所有阶段的实验纪律

1. **一次只换一个控制源**：启动前检查 `/mavros/setpoint_position/local`、`/mavros/setpoint_raw/local` 和速度 setpoint 的发布者。
2. **先空场、低高度、低速度**：控制和轨迹稳定后再加障碍、感知和规划。
3. **先位置 setpoint，后 raw setpoint**：先理解最简单闭环，再引入速度/加速度前馈和 type mask。
4. **先真正确认参数被读取**：`rosparam get` 只能证明参数存在，不能证明源码使用了它。
5. **所有阶段持续发布 setpoint**：状态切换、计时、服务调用都不能长时间阻塞控制流。
6. **记录实际运行副本**：尤其是外部 `~/PX4-Autopilot` 中的 launch、SDF 和 airframe。
7. **量化而非目测**：每次至少保存目标、实际 odom、状态和参数。
8. **把 frame 当成接口的一部分**：odom、点云、目标和轨迹不对齐时，先解决坐标关系，不靠调增益掩盖。
9. **不要过早调 PX4 内环**：大多数初期问题来自无效参数、错误 setpoint、任务推进、frame 或多个 publisher。
10. **保留可回退基线**：每完成一个阶段，保存一次能够重复运行的配置和实验结果，再进入下一阶段。

## 15. 建议的最终学习顺序总结

### 第一步：先学环境和链路

先掌握 ROS1、Gazebo、PX4 SITL、MAVROS、launch、话题和参数，能分进程启动并检查连接。完成后，你可以独立把仿真环境稳定跑起来，并知道问题发生在哪一层。

### 第二步：再读懂默认 Offboard

沿 `pc_example.sh -> astra_example.launch -> autoarming_control.cpp` 阅读并复现当前圆/方形演示。完成后，你可以让无人机自动解锁、起飞、飞默认轨迹并降落，也能判断哪些参数真实生效。

### 第三步：先把悬停做好

补齐有效 pose 等待、合法四元数、相对 home 起飞、HOVER 阶段、超时和可靠上锁。完成后，你可以稳定实现最基础的自主起飞—悬停—降落，这是后续所有任务的安全基线。

### 第四步：加入航点和动作状态机

实现多个航点、停留、转向、返航、超时和错误分支。完成后，无人机可以不用人工逐条控制，按照预设动作序列自主运动。

### 第五步：再做轨迹速度和航向

把轨迹推进改成基于时间/弧长，真正使用 `speed`，再加入圆、方形、8 字和切线 yaw。完成后，无人机可以按你设想的几何轨迹、速度、高度和航向运动。

### 第六步：建立量化调试能力

用 rosbag、目标/实际曲线、RMSE、最大误差、完成时间和重复实验调参。完成后，你能用数据决定下一步改什么，而不是靠反复试参数碰运气。

### 第七步：学习传感器、定位和 frame

理解 Mid360 SDF、FAST-LIO 配置以及 MAVROS/FAST-LIO odom 的差别。完成后，你能判断定位和点云是否足以提供给规划器，并避免错误坐标系导致无人机飞向错误方向。

### 第八步：单独学会 EGO-Planner

先在它自己的 SO3 模拟器中掌握目标、地图、B-spline、速度/加速度约束和重规划。完成后，你可以在独立规划仿真中生成平滑避障轨迹，但此时还没有控制 PX4 无人机。

### 第九步：最后桥接规划器和 PX4

对齐 odom/点云 frame，停用 EGO 内部模拟器，实现 `PositionCommand -> MAVROS PositionTarget`，保证唯一控制发布者。完成后，PX4/Gazebo 无人机能够跟随 EGO 轨迹绕开静态障碍。

### 第十步：提升到任务级自主

加入动态障碍、复杂 world、任务状态机、超时、重试、返航和量化成功率。完成后，无人机能够按任务目标自主选择和执行动作，并在仿真环境变化时进行重规划和安全处置。

这条顺序的核心是：先把“能稳定控制”做扎实，再加入“按轨迹运动”，然后加入“感知和规划”，最后加入“任务决策”。对当前仓库而言，阶段 0～5 应优先围绕 `autoarming_control` 完成；只有需要避障和在线重规划时，才进入 FAST-LIO/EGO/PX4 的系统集成。
