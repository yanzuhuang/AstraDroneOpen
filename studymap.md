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
  -> 单独掌握 EGO-Planner 自带仿真链
  -> 最后学习相机接口与 YOLO 2D 目标检测
```

本文依据当前仓库的 `README.md`、`spec.md`、仿真资源、Offboard 源码、EGO-Planner 源码和 YOLO 检测包编写。文中遵循以下约定：

- **仓库事实**：当前源码、launch、world、model 或 config 可以直接确认。
- **根据代码结构推测**：仓库没有完整说明或尚未形成可运行闭环，仅依据模块边界、话题或命名提出的学习/实现方向。
- **推荐验收阈值**：为了让学习结果可以检查而给出的建议值，不是仓库原作者声明的性能指标；应根据本机实时因子和仿真结果调整。

为缩短后文路径，`offboard/` 指 `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/`，`plan_manage/` 指 `AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/`，`yolo_detect/` 指 `AstraDrone_ros1_ws/src/Detection/yolo_detect/`。

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
2. `pc_example.sh` 虽然会额外启动 FAST-LIO，但默认 Offboard 控制器不使用其 `/Odometry`。FAST-LIO、Mid360 和 SLAM 已退出本学习路线；学习阶段 0～5 时可拆分启动流程或关闭这些非必需节点。
3. EGO-Planner 当前使用自己的 SO3 动力学模拟器，且相关包带有 `CATKIN_IGNORE`；它不会直接控制默认 PX4/Gazebo 无人机。

YOLO 同样不是默认飞行闭环的一部分。本路线最后只学习独立的 2D 检测链：

```text
相机 RGB 图像
  -> yolo_detect.py
  -> 类别、置信度、边界框和标注图
  -> 离线/rosbag 评估
```

当前路线不继续做 RGB-D 三维定位、目标跟踪或 YOLO 驱动飞行。检测节点不得发布 MAVROS setpoint；完成结构化 2D 检测和结果评估即达到阶段目标。

### 2.2 当前仓库中的重要边界

- 默认 `autoarming_control.launch` 中的 `speed` 和 `takeoff_height` 参数当前没有被源码读取；修改它们不会产生预期效果。
- 圆轨迹的 `radius` 在源码中读取，默认值为 `2.0`，但默认 launch 没有显式设置该参数。
- 默认控制器没有独立 `HOVER` 阶段；达到起飞高度后立即进入 `TRACKING`。
- `position_control` 当前存在参数未读取、构造函数进入无限循环、没有发布频率控制、没有 OFFBOARD/解锁流程等问题，不适合作为第一条可运行主线。
- `simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch` 的默认 world 路径拼接缺少 `/`。学习初期可用绝对 `world:=...` 参数绕过。
- PX4 自定义 launch、airframe 和 SDF 会复制到外部 `~/PX4-Autopilot`；修改仓库源文件后，实际运行的外部副本不一定自动更新。
- 任意时刻只应有一个节点持续发布 MAVROS setpoint。不得同时让 `autoarming_control`、`position_control` 和规划桥接节点控制同一架无人机。
- `yolo_detect/` 当前带有 `CATKIN_IGNORE`；移除前应先确认依赖和工作空间构建范围，不要把“目录里有源码”当成“默认可运行”。
- `yolo_detect.py` 当前订阅 `/csi_camera/image_raw`，而 `iris_mid360` 搭载的 D435i 默认发布 `/realsense_d435i/color/image_raw`；必须通过参数/remap 对齐，不能照抄 launch 后只等画面。
- `yolo_detect.py` 当前只发布标注图 `/yolo/detect_image`，没有发布类别、置信度、框和时间戳组成的结构化检测数组，暂时不能作为可靠任务输入。
- 当前 `yolo_detect.launch` 只启动 2D 节点，正好与本路线范围一致；`yolo_detect_fusion.py` 作为范围外文件不学习、不修改。
- 当前仓库自带 `yolov8n.pt` 和 `bus.jpg`，适合做环境冒烟测试，不代表模型能识别你的最终无人机任务目标；自定义类别需要单独的数据集、训练、验证和部署闭环。
- **当前明确不学**：Mid360/LiDAR/IMU 仿真参数、FAST-LIO、SLAM 算法、SLAM 坐标对齐、EGO 与 PX4/SLAM 的系统桥接、RGB-D 三维目标定位以及动态感知任务。默认模型或脚本中出现这些模块时，只需确认它们不影响当前阶段，不进入源码和参数学习。
- `build/`、`devel/` 是生成目录；学习和修改应以 `src/` 下的源文件为准。

## 3. 总体阶段表

本路线收缩为阶段 0～7：阶段 0～5 保留原有飞行控制主线，阶段 6 专门学习 EGO-Planner 自带仿真，阶段 7 最后学习 YOLO 2D 检测。Mid360、FAST-LIO、其他 SLAM、EGO/PX4-SLAM 集成、RGB-D 三维定位和感知驱动飞行均不在当前必学范围内。

| 阶段 | 学习主题 | 完成后能实现的效果 |
|---|---|---|
| 0 | 仓库、ROS 与仿真基线 | 能独立启动并解释 PX4/Gazebo/MAVROS 链路 |
| 1 | 默认 Offboard 闭环 | 能复现并观察自动起飞、圆/方形、降落 |
| 2 | 起飞、悬停与安全降落 | 能让无人机起飞后稳定定点悬停，再受控降落 |
| 3 | 航点与任务状态机 | 能执行“起飞—悬停—多个航点—返航—降落” |
| 4 | 轨迹速度、航向与形状 | 能按可解释的速度和 yaw 跟踪圆、方形、8 字等轨迹 |
| 5 | 参数调试与结果验证 | 能用 rosbag 和误差指标判断改动是否真的有效 |
| 6 | EGO-Planner 独立仿真 | 能在自带模拟器中理解地图、B-spline、FSM 和重规划 |
| 7 | 相机接口与 YOLO 2D 检测 | 能完成可复现的离线/ROS 检测并量化准确率和延迟 |

### 3.1 范围边界

阶段 6 只学习 EGO-Planner 自带的 mock map、规划节点、轨迹服务器、SO3 控制器和动力学模拟器，不连接 FAST-LIO、Mid360 或真实 PX4/Gazebo 无人机。示例中的 `/visual_slam/odom` 只是 EGO 仿真链使用的话题名，本路线只把它当作 odom 接口，不学习其 SLAM 实现。

阶段 7 只学习 YOLO 环境、图像接口、2D 检测节点和按需训练。深度融合、三维目标坐标、视觉跟踪及视觉控制飞行均作为以后按项目需要再增加的可选课题。

### 3.2 推进方式

严格按阶段 0～7 推进。每次只做一个可验收里程碑，并保存运行命令、Git 差异、参数或模型版本、输入数据、指标和失败样例。只有满足验收标准才标记完成；“代码看完了”或“出现检测框”都不等于阶段完成。

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
- 能说明默认脚本中的 FAST-LIO 不参与当前 Offboard 闭环，并能在不学习 SLAM 的情况下独立运行控制基线。

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

阶段 2 的正确实现方式是“主循环不阻塞、每一周期都发布目标、计时只负责切换状态”。下面是与当前 `autoarming_control.cpp` 结构相符的示例骨架，实际修改时应把 `home_*`、`hover_start` 和 `hover_duration` 放到合适的状态变量中：

```cpp
enum class FlightPhase { TAKEOFF, HOVER, LANDING, COMPLETED };

// 收到首帧 pose 后只记录一次 home；不要在 FCU 刚连接时直接读取零值。
if (!have_pose) {
    ros::spinOnce();
    rate.sleep();
    continue;
}

geometry_msgs::PoseStamped setpoint;
setpoint.header.stamp = ros::Time::now();
setpoint.header.frame_id = "map";
setpoint.pose.position.x = home_x;
setpoint.pose.position.y = home_y;
setpoint.pose.position.z = home_z + takeoff_height;
setpoint.pose.orientation.w = 1.0;  // 合法的单位四元数
local_pos_pub.publish(setpoint);    // TAKEOFF/HOVER 中都保持 20 Hz 发布

if (flight_phase == FlightPhase::TAKEOFF &&
    std::abs(current_pose.pose.position.z - setpoint.pose.position.z) < takeoff_tolerance) {
    flight_phase = FlightPhase::HOVER;
    hover_start = ros::Time::now();
} else if (flight_phase == FlightPhase::HOVER &&
           ros::Time::now() - hover_start >= ros::Duration(hover_duration)) {
    flight_phase = FlightPhase::LANDING;
}
```

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
- 模式丢失、目标超时、MAVROS 局部位姿失效时的保持/返航/降落策略。
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

阶段 3 可以先用小型 C++ 航点表验证状态机，再迁移到 YAML；不要一开始就引入复杂任务框架：

```cpp
struct Waypoint {
    double x;
    double y;
    double z;
    double yaw;
    double hold_sec;
};

// 这些量均为相对 home 的 ENU 偏移。
const std::vector<Waypoint> mission{
    {1.0, 0.0, 1.5, 0.0,        2.0},
    {1.0, 1.0, 1.5, M_PI_2,     2.0},
    {0.0, 1.0, 1.5, M_PI,       2.0},
};

const Waypoint& wp = mission.at(current_waypoint);
target.pose.position.x = home_x + wp.x;
target.pose.position.y = home_y + wp.y;
target.pose.position.z = home_z + wp.z;
target.pose.orientation = tf::createQuaternionMsgFromYaw(wp.yaw);
```

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

阶段 4 的正确推进量应由实际时间差决定。以圆轨迹为例，弧长 `s` 和角度 `theta` 的更新可写成：

```cpp
const ros::Time now = ros::Time::now();
const double dt = std::min((now - last_track_time).toSec(), 0.1);
last_track_time = now;

if (tracking_error <= max_tracking_error) {
    arc_length += speed * dt;  // speed 的单位现在才真正是 m/s
}
const double theta = arc_length / radius;
target.pose.position.x = center_x + radius * std::cos(theta);
target.pose.position.y = center_y + radius * std::sin(theta);
target.pose.position.z = center_z;
target.pose.orientation =
    tf::createQuaternionMsgFromYaw(theta + M_PI_2);  // 逆时针圆的切线 yaw
```

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

建立“提出假设—只改一个变量—录制数据—计算指标—比较结果”的调试方法，能区分启动问题、轨迹生成问题、PX4 跟踪问题、仿真实时性问题和 MAVROS 局部位姿输入问题。

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
5. 将 Gazebo GUI、RViz 和其他非必需节点分别关闭，比较实时因子和轨迹误差，判断性能瓶颈是否影响结果。
6. 建立一张调参表：参数、预期影响、实际影响、是否保留、证据文件。
7. 最后才做动力学敏感性实验，例如质量或电机响应时间；每次只改一项并能恢复基线。

阶段 5 的最小可复现实验示例是先保存参数，再用固定话题集合录包；每次只改变一个 launch 参数：

```bash
mkdir -p "$HOME/astra_experiments/circle_speed_05"
rosparam dump "$HOME/astra_experiments/circle_speed_05/params.yaml"
rosbag record -O "$HOME/astra_experiments/circle_speed_05/run01.bag" \
  /mavros/state \
  /mavros/setpoint_position/local \
  /mavros/local_position/pose \
  /mavros/local_position/odom \
  /tf /tf_static /clock
```

### 完成该阶段的验收标准

- 不再用“看起来飞得不错”作为唯一结论，至少能输出目标/实际轨迹图和一组误差指标。
- 每项参数变更都有实验假设和结果，能判断改动是否真正生效。
- 连续运行同一任务至少五次，结果分布基本稳定；若不稳定，能从日志定位失败阶段。
- 能区分“参考轨迹太激进”“PX4 跟踪滞后”“MAVROS 局部位姿异常”“Gazebo 运行过慢”四类问题。
- 推荐验收阈值：基础航点/低速轨迹连续五次无 OFFBOARD 丢失、无 NaN、无意外 setpoint publisher，且误差指标没有明显离群。

### 可能需要修改的关键文件或参数位置

- `scripts/run_sh/record.sh`：可按实验目的补充话题、输出目录和 bag 命名；不要重复记录同一话题。
- `offboard/launch/autoarming_control.launch`：集中管理实验参数。
- `offboard/src/autoarming_control.cpp`：把硬编码的误差门限、阶段超时和最大高度改为参数。
- `simulation/astra_gazebo_worlds/*.world`：物理步长、更新率和障碍物位置。
- `iris_without_GPS.sdf`：质量、惯量、电机时间常数、最大转速、推力/力矩常数。只有在上层控制已稳定、实验目标明确时再改。

## 10. 阶段 6：单独掌握 EGO-Planner 自带仿真链

### 阶段目标

先把 EGO-Planner 当作一个独立系统学习：理解目标输入、占据地图、重规划 FSM、B-spline、轨迹服务器、SO3 控制器和自带动力学模拟器。此阶段不连接 PX4/Gazebo 无人机。

本阶段不接入 Mid360、FAST-LIO 或其他 SLAM，也不实现 `PositionCommand -> MAVROS` 桥接。`/visual_slam/odom` 仅作为 EGO 自带模拟器输出的 odom 话题使用，不要求学习视觉 SLAM。

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

完成依赖启用和编译后，阶段 6 可用下面的最小运行检查作为示例；本阶段看到的 odom 仍应来自 EGO 自带模拟器：

```bash
source AstraDrone_ros1_ws/devel/setup.bash
roslaunch ego_planner run_in_sim.launch

# 在另一个终端检查独立规划链
rostopic hz /visual_slam/odom
rostopic hz /planning/bspline
rostopic hz /planning/pos_cmd
rostopic info /planning/pos_cmd
```

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

## 11. 阶段 7：相机接口与 YOLO 2D 目标检测

### 阶段目标与边界

最后学习一个独立、可回放、可量化的 YOLO 2D 检测链：从图片或 ROS RGB 图像中输出类别、置信度、原图坐标边界框和标注图。此阶段不做深度融合、三维坐标、目标跟踪或飞行任务集成。

```text
图片/视频或 sensor_msgs/Image
  -> YOLO 推理
  -> Detection2DArray（类别、置信度、边界框、原始 header）
  -> 标注图与离线指标
```

### 7.1 YOLO 环境与离线推理

#### 需要掌握的知识点

- 目标检测与图像分类的区别；类别、置信度、边界框、IoU 和 NMS。
- Ultralytics 结果中的 `boxes.xyxy`、`boxes.cls`、`boxes.conf` 和类别名映射。
- `imgsz`、`conf`、`iou`、设备和模型大小对速度及准确率的影响。
- 首帧加载/预热时间、稳定推理时间、处理 FPS 和端到端延迟的区别。
- 当前权重类别与实际任务类别的边界；不能只凭权重文件名假定类别。

#### 推荐实践任务

1. 运行 `yolo_detect/script/testEnv.py`，只验证 Python、Ultralytics、OpenCV、权重和样例图片。
2. 将无限显示循环改成可配置的单次或固定次数测试，支持无 GUI 保存结果并正常退出。
3. 用仓库 `bus.jpg`、一段短视频和至少 20 张接近目标场景的图片推理，保存成功及失败样例。
4. 预热后统计至少 100 帧的平均/P95 延迟和 FPS，注明硬件、模型和输入分辨率。
5. 一次只改变 `conf`、`imgsz` 或模型尺寸中的一项，记录漏检、误检和速度变化。

#### 验收标准

- 可以用固定命令重复完成离线推理，模型路径不依赖当前工作目录。
- 能打印并解释每个检测的类别、置信度和原图坐标框。
- 有可复现的延迟、FPS 和失败样例记录。
- 能判断最终目标是否属于当前权重类别；不属于时再进入 7.4。

### 7.2 ROS 图像接口基线

#### 需要掌握的知识点

- `sensor_msgs/Image` 的 encoding、step、header、时间戳和 frame。
- `cv_bridge`、原始/压缩图像、输入频率、队列积压和 rosbag 回放。
- 相机话题与检测节点硬编码话题不一致时的参数化/remap 方法。

#### 本项目中应该重点阅读的文件/文件夹路径

- `AstraDrone_ros1_ws/src/Utils/camera_sdk/`
- `simulation/astra_gazebo_models/D435i/model.sdf`
- `yolo_detect/script/yolo_detect.py`
- `yolo_detect/launch/yolo_detect.launch`

当前 `yolo_detect.py` 订阅 `/csi_camera/image_raw`，而仓库 D435i RGB 默认话题为 `/realsense_d435i/color/image_raw`。本阶段只使用 RGB 图像，不研究同一模型中附带的 Mid360、深度或 IMU 接口。

#### 推荐实践任务

1. 不启动 YOLO，先确认实际 RGB 话题的类型、encoding、frame、时间戳和频率。
2. 用 `rqt_image_view` 或保存单帧确认图像方向、分辨率、视场和遮挡。
3. 录制一段只包含 RGB 和 `/clock` 的短 bag，确保无需 Gazebo 也能回放。
4. 先用 remap 跑通现有节点，再把输入/输出话题改为私有参数。
5. 测试图像中断、恢复、bag 暂停和节点重启。

#### 验收标准

- RGB 图像持续发布，encoding、frame、时间戳和实际频率有记录。
- rosbag 回放能够稳定复现输入。
- 检测节点不再依赖硬编码的相机话题。

### 7.3 ROS 2D 检测节点工程化

#### 推荐实践任务

1. 参数化 `model_path`、`image_topic`、`annotated_topic`、`conf`、`iou`、`imgsz`、`device`、`classes`、`frame_skip` 和 `publish_annotated`。
2. 使用 `cv_bridge` 按 encoding 转换输入；输出标注图继承原图的 `header.stamp` 和 `header.frame_id`。
3. 发布结构化检测数组，至少包含原图 header、宽高，以及每个目标的类别、置信度和 `xmin/ymin/xmax/ymax`。
4. 队列保持为 1，并采用“只处理最新帧”的机制，避免推理慢于输入时延迟不断累积。
5. 补齐 `package.xml`、`CMakeLists.txt`、消息生成和 Python 安装规则。
6. 在固定 bag 上测试空画面、多目标、快速运动、输入中断和节点重启，并测量端到端延迟。
7. 只有 PyTorch 基线稳定后才考虑 ONNX/TensorRT，并对比导出前后的类别、框和置信度。

#### 验收标准

- launch 参数可以切换模型、话题、阈值和设备，源码中没有个人绝对路径。
- 结构化检测与标注图保留原始时间戳/frame，框坐标正确映射回原图。
- 无检测时发布带 header 的空数组；输入速率高于推理速率时延迟仍然有界。
- 固定 bag 重复运行得到稳定的检测数量、类别和延迟统计。
- 节点不发布 3D pose、TF 或任何 MAVROS 控制话题。

### 7.4 自定义数据集与训练（按需）

如果目标不在当前权重类别中，或航拍/仿真视角与现有训练数据差异明显，再进入本小节；否则可以跳过。

#### 推荐实践任务

1. 写清类别、最小目标像素、距离/视角范围、允许漏检/误检和推理延迟预算。
2. 按场景或录制序列划分 train/val/test，避免相邻视频帧跨集合造成数据泄漏。
3. 保留正样本、困难样本和无目标负样本，检查漏标、框越界和类别不平衡。
4. 先训练小模型基线，一次只调整数据、增强、分辨率或模型规模中的一项。
5. 报告每类 precision、recall、PR、mAP50、mAP50-95、固定硬件延迟和典型失败样例。
6. 保存数据版本、类别 YAML、训练配置、随机种子、权重和评估结果。

#### 验收标准

- 独立测试集没有同源相邻帧泄漏。
- 指标覆盖每个任务类别，并有按目标尺寸或视角划分的失败分析。
- 阈值来自验证/测试结果，而不是凭感觉设定。
- 新权重能通过 7.3 的同一 bag 回放测试，ROS 接口行为保持不变。

### 阶段 7 优先修改位置

- `yolo_detect/script/testEnv.py`：有限次数运行、命令行参数、无 GUI 输出和基准统计。
- `yolo_detect/script/yolo_detect.py`：参数化、标准图像转换、结构化 2D 消息、原始 header、最新帧处理和延迟统计。
- `yolo_detect/launch/yolo_detect.launch`：模型、输入/输出话题、阈值、设备和可视化参数。
- `yolo_detect/CMakeLists.txt`、`yolo_detect/package.xml`：消息生成、真实运行依赖和 Python 安装。
- `yolo_detect/CATKIN_IGNORE`：只在依赖清单、构建范围和回退方式明确后处理。
- **不修改** `yolo_detect_fusion.py`：深度融合和三维定位不在当前路线范围内。

## 12. 贯穿所有阶段的实验纪律

1. **一次只换一个控制源**：启动前检查 `/mavros/setpoint_position/local`、`/mavros/setpoint_raw/local` 和速度 setpoint 的发布者。
2. **先空场、低高度、低速度**：控制和轨迹稳定后再加障碍、感知和规划。
3. **先位置 setpoint，后 raw setpoint**：先理解最简单闭环，再引入速度/加速度前馈和 type mask。
4. **先真正确认参数被读取**：`rosparam get` 只能证明参数存在，不能证明源码使用了它。
5. **所有阶段持续发布 setpoint**：状态切换、计时、服务调用都不能长时间阻塞控制流。
6. **记录实际运行副本**：尤其是外部 `~/PX4-Autopilot` 中的 launch、SDF 和 airframe。
7. **量化而非目测**：每次至少保存目标、实际 odom、状态和参数。
8. **把 frame 当成接口的一部分**：MAVROS odom、setpoint 和相机图像的 frame/时间含义必须明确，不靠调增益掩盖接口错误。
9. **不要过早调 PX4 内环**：大多数初期问题来自无效参数、错误 setpoint、任务推进、frame 或多个 publisher。
10. **保留可回退基线**：每完成一个阶段，保存一次能够重复运行的配置和实验结果，再进入下一阶段。
11. **视觉结果必须保留原始时间信息**：检测结果与标注图都继承输入图像时间戳/frame，不能用“当前时间”掩盖处理延迟。
12. **空结果也是接口的一部分**：无检测、图像中断或结果过期时发布明确状态，不能让旧检测无限有效。
13. **YOLO 与飞控保持隔离**：YOLO 只发布 2D 检测结果和标注图，不发布 pose、TF 或 MAVROS setpoint。

## 13. 建议的最终学习顺序总结

### 第一步：建立环境和链路基线（阶段 0）

掌握 ROS1、Gazebo、PX4 SITL、MAVROS、launch、话题和参数，能分进程启动并检查连接；Mid360/FAST-LIO 即使出现在默认脚本中，也不进入学习范围。

### 第二步：读懂默认 Offboard（阶段 1）

沿 `pc_example.sh -> astra_example.launch -> autoarming_control.cpp` 阅读并复现圆/方形演示，判断哪些参数真实生效。

### 第三步：完成安全悬停（阶段 2）

补齐有效 pose 等待、合法四元数、相对 home 起飞、HOVER、超时和可靠上锁。

### 第四步：加入航点任务状态机（阶段 3）

实现多个航点、停留、转向、返航、超时和错误分支。

### 第五步：掌握轨迹速度和航向（阶段 4）

把轨迹推进改成基于时间/弧长，真正使用 `speed`，再加入圆、方形、8 字和切线 yaw。

### 第六步：建立量化调试能力（阶段 5）

使用 rosbag、目标/实际曲线、RMSE、最大误差、完成时间和重复实验判断改动是否有效。

### 第七步：单独学习 EGO-Planner（阶段 6）

只运行 EGO 自带 mock map、SO3 控制器和动力学模拟器，理解地图、FSM、B-spline、动力学约束和重规划；不接入 Mid360、FAST-LIO、其他 SLAM 或 PX4 桥接。

### 第八步：最后学习 YOLO（阶段 7）

依次完成离线推理、ROS RGB 图像接口、结构化 2D 检测和按需自定义训练；不做 RGB-D 三维定位、目标跟踪或视觉控制飞行。

最终顺序固定为：**阶段 0～5 飞行控制基础 -> 阶段 6 EGO-Planner 独立仿真 -> 阶段 7 YOLO 2D 检测**。当前路线到此结束，Mid360、FAST-LIO、SLAM 和多模块自主闭环均留作以后按需求扩展。
