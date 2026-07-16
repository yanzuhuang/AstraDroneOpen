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
  -> 分成两条可并行的感知支线
     |-> Mid360 基线 -> FAST-LIO -> EGO-Planner -> PX4 静态避障
     `-> 相机基线 -> YOLO 2D 检测 -> 深度/TF 目标定位
  -> 汇合为静态/动态场景中的感知驱动自主任务
```

本文依据当前仓库的 `README.md`、`spec.md`、仿真资源、Offboard 源码、FAST-LIO 配置、EGO-Planner 源码和 YOLO 检测包编写。文中遵循以下约定：

- **仓库事实**：当前源码、launch、world、model 或 config 可以直接确认。
- **根据代码结构推测**：仓库没有完整说明或尚未形成可运行闭环，仅依据模块边界、话题或命名提出的学习/实现方向。
- **推荐验收阈值**：为了让学习结果可以检查而给出的建议值，不是仓库原作者声明的性能指标；应根据本机实时因子和仿真结果调整。

为缩短后文路径，`offboard/` 指 `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/`，`plan_manage/` 指 `AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/`，`FAST_LIO/` 指 `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/`，`yolo_detect/` 指 `AstraDrone_ros1_ws/src/Detection/yolo_detect/`，`dynamic_obstacle_controller/` 指 `simulation/sim_workspace/src/dynamic_obstacle_controller/`。

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

YOLO 同样不是默认飞行闭环的一部分。当前仓库的视觉链应理解为：

```text
D435i/其他相机 RGB 图像
  -> yolo_detect.py（2D 检测与标注图）
  -> yolo_detect_fusion.py（尝试结合深度和 TF 做 3D 定位）
  -> 待完善的结构化检测接口
  -> 任务层决定观察、搜索、跟随或返航
  -> 已验收的航点/规划/Offboard 控制链执行动作
```

YOLO 负责回答“画面里有什么、在哪里”，不应绕过任务层直接持续发布 MAVROS setpoint。第一轮学习只做离线推理、ROS 图像检测和结果评估；等基础飞控、深度、TF 和消息接口分别通过验收后，再接任务逻辑。

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
- `yolo_detect_fusion.py` 使用 `/camera/image_raw`、`/fused/depth_image`、`/camera/camera_info` 和 `fmu <- camera` TF，与当前 D435i 默认接口并不一致；它还采用“最新深度图”而非严格的 RGB/深度同步。3D 定位前必须补齐话题参数化、时间同步和 frame 验证。
- 当前 `yolo_detect.launch` 只启动 2D 节点；`yolo_detect_fusion.py` 没有对应 launch 且当前文件没有可执行位。它是待完善的参考实现，不是已经接通的默认融合链。
- 当前仓库自带 `yolov8n.pt` 和 `bus.jpg`，适合做环境冒烟测试，不代表模型能识别你的最终无人机任务目标；自定义类别需要单独的数据集、训练、验证和部署闭环。
- `build/`、`devel/` 是生成目录；学习和修改应以 `src/` 下的源文件为准。

## 3. 总体阶段表

主控制/定位/规划路线保持阶段 0～10；YOLO 作为 V0～V4 视觉支线并行推进。两条路线只在接口稳定后汇合，避免为了学 YOLO 阻塞基础飞控，也避免检测节点尚未验收就参与飞行决策。

### 3.1 主线：飞控、定位与规划

| 阶段 | 学习主题 | 完成后能实现的效果 |
|---|---|---|
| 0 | 仓库、ROS 与仿真基线 | 能独立启动并解释 PX4/Gazebo/MAVROS 链路 |
| 1 | 默认 Offboard 闭环 | 能复现并观察自动起飞、圆/方形、降落 |
| 2 | 起飞、悬停与安全降落 | 能让无人机起飞后稳定定点悬停，再受控降落 |
| 3 | 航点与任务状态机 | 能执行“起飞—悬停—多个航点—返航—降落” |
| 4 | 轨迹速度、航向与形状 | 能按可解释的速度和 yaw 跟踪圆、方形、8 字等轨迹 |
| 5 | 参数调试与结果验证 | 能用 rosbag 和误差指标判断改动是否真的有效 |
| 6 | Mid360 传感器仿真与真值基线 | 能验证 LiDAR/IMU 的话题、频率、时间戳、噪声和安装关系 |
| 7 | FAST-LIO 独立定位与建图 | 能独立运行 FAST-LIO，理解参数并用仿真参考轨迹评价里程计和地图 |
| 8 | EGO-Planner 独立仿真 | 能在其自带模拟器中设置目标并观察 B-spline 重规划 |
| 9 | FAST-LIO/EGO/PX4 静态避障闭环 | 能用仿真 LiDAR/IMU 定位建图，并让 PX4/Gazebo 无人机跟随规划轨迹绕障 |
| 10 | 自主任务与动态场景 | 能在任务逻辑驱动下处理动态障碍、超时、失败和结束条件 |

建议主线严格按顺序推进。阶段 0～5 是单机自主控制的基础；阶段 6 建立 Mid360 仿真传感器基线；阶段 7 专门学习 FAST-LIO；阶段 8 单独掌握规划器；阶段 9～10 才把定位、建图、规划和任务逻辑接成更高程度的自主闭环。

### 3.2 视觉支线：相机与 YOLO

| 支线阶段 | 前置条件 | 学习主题 | 完成后能实现的效果 |
|---|---|---|---|
| V0 | 阶段 0 | YOLO 环境与离线推理 | 能用仓库权重稳定完成图片/视频推理并读懂检测结果 |
| V1 | V0 | ROS 图像与仿真相机基线 | 能确认 RGB、深度、CameraInfo、时间戳和 optical frame |
| V2 | V1 | ROS 2D 检测节点工程化 | 能发布可消费的类别、置信度、检测框和标注图，并量化延迟/FPS |
| V3 | V2；若需自定义目标 | 数据集、训练与模型评估 | 能用独立测试集说明模型适用边界，而不是只展示几张成功图片 |
| V4 | V2、阶段 3；3D 任务还需可靠深度/TF | 深度融合、3D 定位与任务接口 | 能把稳定目标位置交给任务层，完成“发现—确认—执行—丢失处置” |

推荐的并行节奏是：阶段 0 完成后即可连续做 V0～V2，同时主线推进阶段 1～5；只有任务需要识别自定义类别时才进入 V3；V4 必须等阶段 3 的安全任务状态机和 V2 的结构化检测接口都稳定后再做。YOLO 不是 FAST-LIO 或 EGO 的前置条件，阶段 6～9 可与 V1～V3 并行。

### 3.3 每周推进方式

每次只选一个主里程碑和一个不互相干扰的支线里程碑。例如“主线完成悬停 30 秒验收 + 支线完成离线 YOLO 基线”。每个里程碑都保存：运行命令、Git 差异、参数/模型版本、输入数据、指标和失败样例。只有满足对应验收标准才标记完成；“代码看完了”不等于阶段完成。

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
- 能区分“参考轨迹太激进”“PX4 跟踪滞后”“定位输入异常”“Gazebo 运行过慢”四类问题。
- 推荐验收阈值：基础航点/低速轨迹连续五次无 OFFBOARD 丢失、无 NaN、无意外 setpoint publisher，且误差指标没有明显离群。

### 可能需要修改的关键文件或参数位置

- `scripts/run_sh/record.sh`：可按实验目的补充话题、输出目录和 bag 命名；不要重复记录同一话题。
- `offboard/launch/autoarming_control.launch`：集中管理实验参数。
- `offboard/src/autoarming_control.cpp`：把硬编码的误差门限、阶段超时和最大高度改为参数。
- `simulation/astra_gazebo_worlds/*.world`：物理步长、更新率和障碍物位置。
- `iris_without_GPS.sdf`：质量、惯量、电机时间常数、最大转速、推力/力矩常数。只有在上层控制已稳定、实验目标明确时再改。
- `1046_gazebo-classic_iris_mid360`：PX4 EKF/无 GPS airframe 参数。修改前先保存基线并确认外部 PX4 副本同步。

## 10. 阶段 6：建立 Mid360 传感器仿真与真值基线

### 阶段目标

先不启动 FAST-LIO，只研究 Gazebo 中的 `iris_mid360`、LiDAR 和 IMU 数据是怎样产生的，验证话题类型、发布频率、时间戳、frame、量程、噪声和安装关系。阶段结束时应得到一套可信的原始传感器数据基线，供下一阶段判断 FAST-LIO 问题究竟来自算法还是仿真输入。

### 需要掌握的知识点

- SDF 的 include、link、joint、sensor、plugin、pose 和 frame。
- `iris_mid360.sdf` 中“整台传感器相对机身的安装位姿”与 `mid360.sdf` 中“LiDAR—IMU 内部外参”的区别。
- LiDAR 扫描频率、采样数、downsample、量程、盲区和噪声。
- IMU 更新率、角速度/加速度噪声、bias 和时间戳。
- Gazebo 仿真时间、`/clock`、ROS 消息时间戳和 real-time factor。
- `/livox/lidar`、`/livox/imu`、`/gazebo/model_states`、MAVROS local pose 各自代表什么。
- 原始点云 frame 与机体系/world 系的区别；RViz 中 Fixed Frame 选错会造成的假象。

### 本项目中应该重点阅读的文件/文件夹路径

- `simulation/px4_sim_files/px4_iris_sdf/iris_mid360/iris_mid360.sdf`
- `simulation/px4_sim_files/px4_iris_sdf/iris_without_GPS/iris_without_GPS.sdf`
- `simulation/astra_gazebo_models/mid360/mid360.sdf`
- `simulation/sim_workspace/src/sensors/Mid360_simulation_plugin/livox_laser_simulation/`
- `simulation/sim_workspace/src/sensors/Mid360_simulation_plugin/livox_laser_simulation/src/livox_points_plugin.cpp`
- `simulation/sim_workspace/src/sensors/Mid360_simulation_plugin/livox_laser_simulation/msg/CustomMsg.msg`
- `simulation/astra_gazebo_worlds/example.world`
- `simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch`

### 推荐做的仿真实践任务

1. 只启动 PX4/Gazebo/MAVROS，不启动 FAST-LIO 和 Offboard 控制器。
2. 检查原始接口：

   ```bash
   rostopic type /livox/lidar
   rostopic type /livox/imu
   rostopic hz /livox/lidar
   rostopic hz /livox/imu
   rostopic echo -n 1 /livox/lidar/header
   rostopic echo -n 1 /livox/imu/header
   ```

3. 在 RViz 中只显示原始点云，确认静止时地面、树木和建筑的空间位置与 Gazebo 场景一致。
4. 录制“静止 30 秒”和“只沿单轴慢速移动”两份原始 bag，包含 `/clock`、LiDAR、IMU、MAVROS pose/odom、Gazebo model states 和 TF。
5. 只改一个 SDF 参数做实验，例如把 LiDAR `update_rate` 从 10 Hz 改为另一个值，重启后用 `rostopic hz` 证明改动生效，再恢复基线。
6. 比较 GUI/RViz 开关前后的 real-time factor 和传感器实际频率，确认电脑负载不会破坏后续 FAST-LIO 实验。

### 完成该阶段的验收标准

- 能准确指出 Mid360 相对机身的安装 pose，以及 LiDAR 与内部 IMU 的外参分别在哪里定义。
- 能说明当前 LiDAR/IMU 的话题类型、frame、标称频率和实测频率。
- 原始点云在正确 Fixed Frame 下与 Gazebo 场景几何关系一致，没有明显时间戳回退或长时间断流。
- 已保存静止和单轴运动两份可复现 bag，并记录对应 world、SDF 参数和 Git 版本。
- 能区分“传感器输入不正确”和“SLAM 算法输出不正确”，再进入阶段 7。

### 可能需要修改的关键文件或参数位置

- `simulation/astra_gazebo_models/mid360/mid360.sdf`：`update_rate`、range、noise、samples、downsample、消息类型、LiDAR/IMU frame 和内部外参。
- `simulation/px4_sim_files/px4_iris_sdf/iris_mid360/iris_mid360.sdf`：Mid360 与相机相对机身的安装 pose 和 joint。
- `simulation/astra_gazebo_worlds/*.world`：物理步长和更新率会影响传感器仿真节奏。
- 改 Gazebo 插件 C++ 后需要重编译 `simulation/sim_workspace`；只改 SDF/world 通常重启仿真即可。
- 修改 PX4 机型 SDF 后还要确认外部 `~/PX4-Autopilot` 中实际运行副本已同步。

## 11. 阶段 7：独立掌握 FAST-LIO 定位与建图

### 阶段目标

使用阶段 6 已验收的仿真 LiDAR/IMU 输入，独立启动 FAST-LIO，理解其输入预处理、IMU 传播、激光更新、ikd-Tree 地图维护和里程计/点云输出。通过 Gazebo/MAVROS 参考轨迹评价 FAST-LIO，而不是一启动就把 `/Odometry` 接入规划器或控制器。

### 需要掌握的知识点

- LIO 的基本结构：IMU 状态传播、点云去畸变、扫描到地图匹配、误差状态迭代更新。
- `lidar_type`、`scan_line`、`blind`、`point_filter_num` 对预处理的影响。
- `acc_cov`、`gyr_cov`、bias covariance 对滤波器信任关系的影响。
- `extrinsic_T/R` 是 LiDAR 与 IMU 的内部外参，不是 world/map 对齐参数。
- `time_sync_en`、`time_offset_lidar_to_imu` 与仿真统一时钟；没有证据时不要随意开启软件时间同步。
- `filter_size_surf`、`filter_size_map`、`det_range`、FOV 与精度/算力的权衡。
- `/Odometry`、`/cloud_registered`、`/cloud_registered_body`、`/path` 和 `camera_init -> body` TF 的含义。
- FAST-LIO 提供局部里程计与地图，但仓库中没有显示其包含全局回环；长时间运行仍可能累计漂移。
- Gazebo 真值、PX4/MAVROS local odom、FAST-LIO odom 是三套不同状态来源，不能因为坐标数值接近就认为可以直接互换。

### 本项目中应该重点阅读的文件/文件夹路径

- `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/launch/mapping_mid360.launch`
- `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/config/mid360.yaml`
- `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/src/preprocess.h`
- `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/src/preprocess.cpp`
- `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/src/IMU_Processing.hpp`
- `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/src/laserMapping.cpp`
- `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/include/use-ikfom.hpp`
- `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/include/ikd-Tree/ikd_Tree.h`
- `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/rviz_cfg/loam_livox.rviz`
- `scripts/run_sh/pc_example.sh`

### 推荐做的仿真实践任务

1. 分层启动 PX4/Gazebo 和 FAST-LIO，暂不启动轨迹控制：

   ```bash
   # PX4/Gazebo 正常后，在新终端执行
   cd "$HOME/AstraDroneOpen"
   source AstraDrone_ros1_ws/devel/setup.bash
   roslaunch fast_lio mapping_mid360.launch rviz:=true
   ```

2. 检查 FAST-LIO 是否真正订阅成功并持续输出：

   ```bash
   rostopic info /livox/lidar
   rostopic info /livox/imu
   rostopic hz /Odometry
   rostopic hz /cloud_registered
   rostopic hz /cloud_registered_body
   rosrun tf tf_echo camera_init body
   ```

3. 注意当前插件在 `publish_pointcloud_type=3` 时发布 `livox_laser_simulation/CustomMsg`，FAST-LIO 源码订阅类型写成 `livox_ros_driver/CustomMsg`。两份消息字段及当前生成头文件的 MD5 一致，但包类型名不同；是否成功连接必须以运行时 `rostopic info`、FAST-LIO 日志和 `/Odometry` 实际输出为准，不能只看话题名推断。
4. 完成三组由简到难的轨迹：静止初始化、单轴慢速移动、低速小正方形。每组同时录制 FAST-LIO odom、MAVROS odom、原始传感器、registered cloud 和 TF。
5. 以阶段 5 的方法比较 FAST-LIO 与参考轨迹的相对位移、方向、尺度、起终点误差和重复性；先比较相对轨迹，不直接比较不同原点下的绝对坐标。
6. 做单变量参数实验：
   - `point_filter_num`：观察计算量和点数；
   - `filter_size_surf/filter_size_map`：观察地图密度和实时性；
   - `blind/det_range`：观察近距离过滤与有效范围；
   - `acc_cov/gyr_cov`：只在已有基线数据支持时微调。
7. 静止与 x/y/z 单轴运动时比较 FAST-LIO 和 MAVROS 的轴向、符号、初始 yaw 和高度，形成一张 frame 对齐表。
8. 故意停止 LiDAR 或 IMU 输入一次，观察日志和输出如何退化；恢复后重新启动形成明确的故障定位流程。

### 完成该阶段的验收标准

- 能从源码画出 `/livox/lidar + /livox/imu -> preprocess/IMU processing -> iterated filter/ikd-Tree -> /Odometry + registered clouds` 的数据流。
- FAST-LIO 在静止、单轴和低速小方形三类实验中均能持续输出，没有 NaN、时间戳回退或无界发散。
- 能解释并实测 `point_filter_num`、体素滤波大小、blind、det_range 至少四类参数的作用。
- 能说明 `/cloud_registered` 与 `/cloud_registered_body` 的 frame/用途差别，并在 RViz 中正确显示。
- 已用同一批 bag 对比 FAST-LIO、MAVROS 和仿真参考轨迹，保存参数、结果图和异常日志。
- 已得到 `camera_init/body` 与 MAVROS local frame 的轴向、平移、yaw 关系；未完成对齐前不得进入阶段 9 的系统集成。

### 可能需要修改的关键文件或参数位置

- `FAST_LIO/launch/mapping_mid360.launch`：`point_filter_num`、`max_iteration`、`filter_size_surf`、`filter_size_map`、`cube_side_length` 和 RViz 开关。
- `FAST_LIO/config/mid360.yaml`：输入话题、`lidar_type`、blind、协方差、FOV、det_range、外参、发布选项和 PCD 保存。
- 仿真统一使用 `/clock` 时，优先保持 `time_sync_en: false`、`time_offset_lidar_to_imu: 0.0`；只有时间戳证据表明确有偏移才修改。
- `publish/path_en` 当前为 false；需要 `/path` 时可设为 true，但重新验证性能。
- `pcd_save_en` 长时间开启可能占用大量内存/磁盘，只在短实验中使用。
- `preprocess.cpp`、`IMU_Processing.hpp`、`laserMapping.cpp` 是算法研究阶段才修改的源码；普通调参先通过 launch/YAML 完成。
- **根据代码结构推测**：若要让 FAST-LIO 直接成为 PX4 EKF 的外部视觉输入，还需要 odometry/vision 消息桥接、frame 转换、时间同步和 PX4 参数验证；当前仓库没有一条已确认完成的这类闭环，因此不放在本路线的基础阶段。

### 阶段 7 之后的接口选择

进入规划集成前，推荐先采用下面的职责分工：

```text
FAST-LIO /Odometry + /cloud_registered
          -> EGO-Planner 的定位/地图输入
EGO PositionCommand
          -> MAVROS setpoint 桥接
PX4 自身 estimator + 内环
          -> Gazebo 无人机运动
```

这条路线仍要求 FAST-LIO frame 与 MAVROS local frame 对齐，因为规划器计算出的目标最终要交给 PX4 执行。不要在第一轮集成中同时做“FAST-LIO 接 EGO”和“FAST-LIO 注入 PX4 EKF”两项改造。

## 12. 阶段 8：单独掌握 EGO-Planner 自带仿真链

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

完成依赖启用和编译后，阶段 8 可用下面的最小运行检查作为示例；本阶段看到的 odom 仍应来自 EGO 自带模拟器：

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

## 13. 阶段 9：连接 FAST-LIO、EGO-Planner 与 PX4/Gazebo，完成静态避障

### 阶段目标

在 FAST-LIO 独立定位建图、EGO 独立规划和 PX4 基础控制都通过验收后，构建唯一的一条自主闭环：仿真 Mid360 提供 LiDAR/IMU，FAST-LIO 提供规划所需的 odom/点云，EGO 生成轨迹，桥接器把轨迹转换为 MAVROS setpoint，最后由 PX4 控制 Gazebo 无人机绕开静态障碍。

### 需要掌握的知识点

- 系统集成的输入输出契约：odom、点云、目标、轨迹命令和 frame。
- `quadrotor_msgs/PositionCommand` 到 MAVROS setpoint 的字段映射。
- `mavros_msgs/PositionTarget` 的坐标系、type mask、position/velocity/acceleration/yaw/yaw_rate。
- 规划器 odom 与 PX4 控制 odom 的一致性要求。
- FAST-LIO `camera_init/body` 与 MAVROS local ENU 的原点、轴向和 yaw 对齐。
- odom、点云和规划目标应处于同一规划 frame；frame 名称 remap 不能代替真实坐标变换。
- 当前 `GridMap::cloudCallback()` 直接使用点坐标并与 odom 位置相减，没有根据点云 `header.frame_id` 查询 TF；因此接入前必须在数据层完成坐标对齐。
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
- `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/launch/mapping_mid360.launch`
- `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/src/laserMapping.cpp`
- `simulation/astra_gazebo_worlds/dynamic_avoidance.world`
- `simulation/sim_workspace/src/dynamic_obstacle_controller/launch/astra_dynamic_avoidance_static.launch`

`rc_obstacle_avoidance` 只能作为“规划输出超时检查与 MAVROS 转发”的参考：它当前依赖仓库中不存在的 Fast-Planner 话题约定，包被忽略，YAML 又被加载到全局命名空间而源码读取私有参数，不能直接当成 EGO/PX4 桥接器。

### 推荐做的仿真实践任务

1. 先做无障碍控制桥接：暂用 `/mavros/local_position/odom` 作为 EGO odom，只验证 `PositionCommand -> MAVROS` 后 PX4 能跟随简单规划轨迹。此时把“桥接问题”和“FAST-LIO frame 问题”分开。
2. 停用 `simulator.xml` 中的 SO3 模拟器、SO3 控制器和自带 sensing 链，只保留真正需要的规划节点。
3. 根据阶段 7 的对齐结果，把 EGO odom 切换为 FAST-LIO `/Odometry`，点云切换为与其同 frame 的 `/cloud_registered`；先在静止场景确认占据地图位置正确。
4. 若 `camera_init` 与 MAVROS local ENU 不是数值同一坐标系，在送入 MAVROS 前对 `PositionCommand` 做明确的旋转和平移；不要只把 `header.frame_id` 改成 `map`。
5. 实现桥接器，订阅 `/planning/pos_cmd`，持续发布 MAVROS raw local setpoint，并处理 OFFBOARD、解锁、轨迹新鲜度、超时悬停、返航和降落。
6. 运行桥接器时停掉 `autoarming_control` 的 setpoint 发布；用 `rostopic info` 验证位置、速度和 raw local setpoint 没有第二个控制发布者。
7. 按“无障碍 -> 单个静态障碍 -> 多个静态障碍 -> `dynamic_avoidance.world` 静态模式”逐级测试；每级失败都先回退到前一级。
8. 同时记录原始 LiDAR/IMU、FAST-LIO odom/点云、EGO B-spline/PositionCommand、MAVROS setpoint/odom、TF、PX4 state 和最小障碍距离。

下面只展示阶段 9 桥接器中最关键的“规划 frame 转 MAVROS local ENU + 字段映射”骨架。`R_local_planning` 和 `t_local_planning` 必须来自阶段 7 的实测对齐，不能照抄常数；完整节点还必须复用阶段 1～3 的 OFFBOARD、解锁、连续发布和超时状态机。此桥接器是**根据代码结构推测**的待实现内容，文档不会替你创建源码文件。

```cpp
void commandCallback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd) {
    const Eigen::Vector3d p_p(cmd->position.x, cmd->position.y, cmd->position.z);
    const Eigen::Vector3d v_p(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
    const Eigen::Vector3d a_p(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);

    // 点需要旋转和平移；速度、加速度只能旋转，不能加平移。
    const Eigen::Vector3d p_l = R_local_planning * p_p + t_local_planning;
    const Eigen::Vector3d v_l = R_local_planning * v_p;
    const Eigen::Vector3d a_l = R_local_planning * a_p;
    const Eigen::Vector3d heading_l =
        R_local_planning * Eigen::Vector3d(std::cos(cmd->yaw), std::sin(cmd->yaw), 0.0);

    mavros_msgs::PositionTarget out;
    out.header.stamp = ros::Time::now();
    out.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    out.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    out.position.x = p_l.x(); out.position.y = p_l.y(); out.position.z = p_l.z();
    out.velocity.x = v_l.x(); out.velocity.y = v_l.y(); out.velocity.z = v_l.z();
    out.acceleration_or_force.x = a_l.x();
    out.acceleration_or_force.y = a_l.y();
    out.acceleration_or_force.z = a_l.z();
    out.yaw = std::atan2(heading_l.y(), heading_l.x());
    raw_local_pub.publish(out);
    last_planner_command = out.header.stamp;
}
```

这里 ROS 侧的局部数值按 MAVROS 的 ENU 约定准备，MAVROS 插件负责发送 MAVLink 前的 ENU/NED 转换；不要因为常量名是 `FRAME_LOCAL_NED` 又手动交换一次 x/y/z。第一轮桥接若不确定加速度前馈是否配置正确，可先在 `type_mask` 中增加 `IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ`，只验证位置、速度和 yaw，再逐项启用。

### 完成该阶段的验收标准

- PX4/Gazebo 无人机而非 EGO 自带模拟器在运动。
- FAST-LIO 持续输出，EGO 的 odom 和点云均来自已经验收的 FAST-LIO 链路，而不是残留的 `/visual_slam/odom` 或 `pcl_render_node`。
- EGO 占据地图中的障碍位置与 Gazebo/registered cloud 一致；静止时不会因 frame 错误让地图跟着飞机漂移。
- `/planning/pos_cmd` 到 MAVROS setpoint 的频率稳定，轨迹超时时自动悬停或进入安全状态。
- 控制话题只有一个发布者；OFFBOARD 在任务中没有意外丢失。
- 静态障碍被规划地图正确占据和膨胀，无人机能够绕行而非穿模。
- 目标、odom、点云和 setpoint 使用的 frame 关系有明确记录。
- 推荐验收标准：在固定静态场景重复五次，全部无碰撞到达目标，并满足事先设定的最小安全距离。

### 可能需要修改的关键文件或参数位置

- `plan_manage/launch/run_in_sim.launch`：odom、cloud、是否 include `simulator.xml`。
- `plan_manage/launch/advanced_param.xml`：frame、地图大小、局部范围、膨胀、速度/加速度和安全距离相关参数。
- `FAST_LIO/config/mid360.yaml` 与 `FAST_LIO/launch/mapping_mid360.launch`：只使用阶段 7 已验收的配置，不在系统集成时重新大范围调参。
- **根据代码结构推测**：需要新增 `PositionCommand -> mavros_msgs/PositionTarget` 桥接节点；当前仓库没有完整可运行的 EGO/PX4 桥接实现。
- **根据代码结构推测**：若 FAST-LIO frame 与 MAVROS local frame 不重合，桥接节点还需要使用已测得的变换转换 position、velocity、acceleration 和 yaw；当前仓库没有现成完整实现。
- **根据代码结构推测**：桥接节点适合放在 Offboard/MissionControl 相关包中并复用已有 OFFBOARD 会话逻辑，但具体文件名和类结构应在实现阶段再确定。
- `rc_obstacle_avoidance.launch` 若用于参考改造，应把 `<rosparam>` 放入 `<node>` 内，使 YAML 进入私有命名空间；它的话题接口还必须从 Fast-Planner 改为 EGO 的 `PositionCommand`。

## 14. 阶段 10：自主任务逻辑、动态障碍与复杂场景

### 阶段目标

在“规划器能稳定控制 PX4 绕静态障碍”的基础上，增加更高层的自主任务决策：根据任务状态自动选择目标、等待、重试、绕行、返航或结束，并在动态障碍和复杂 world 中量化成功率。若 YOLO 支线已完成 V4，再把经过确认的视觉目标作为一种任务输入；视觉未验收时仍使用预设目标，不让它阻塞动态避障主线。

### 需要掌握的知识点

- 分层自主系统：任务层、规划层、轨迹/控制桥接层、PX4 内环。
- 任务状态机与规划 FSM 的区别：任务层决定下一目标，规划器负责到达当前目标。
- 预设目标与感知目标的区别：YOLO 观测需要多帧确认、新鲜度、置信度、空间有效性和丢失处置后才能成为任务目标。
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
- `AstraDrone_ros1_ws/src/Detection/yolo_detect/`，仅在 V0～V4 均按任务需求通过验收后接入。
- `AstraDrone_ros1_ws/src/Exploration/` 和 `AstraDrone_ros1_ws/src/Swarm/`，用于确认当前仓库没有可直接使用的完整自主探索/集群实现。

### 推荐做的仿真实践任务

1. 先启动动态障碍 world 的静态模式，确认规划和控制基线不受启动文件变化影响。
2. 启动 moving 模式，逐个启用障碍物；先把速度设低，再逐步增加。
3. 设计一个任务：起飞 -> 到达观察点 A -> 到达 B -> 如果规划失败则悬停并重试 -> 成功后返航 -> 降落。
4. 给任务增加总超时、单目标最大重试次数、最小安全距离告警和紧急悬停。
5. 用不同障碍轨迹测试：linear、circle、waypoint，并核对 YAML 中模型名与 world 中模型名完全一致。
6. 最后切换 CRAIC 或 forest world，先只用少量目标，再逐渐增加任务复杂度。
7. 对每类场景重复多次，统计成功率、失败阶段、最小障碍距离、重规划次数和完成时间。
8. 可选视觉任务：先完成“发现指定类别后悬停并记录”，再完成“飞到目标附近的安全观察点”，最后才尝试目标跟随；每一级都保留预设目标模式作为对照组。

阶段 10 先只启用一个低速动态障碍。以下 YAML 字段与当前 `obstacle_controller.py` 的读取逻辑一致，且 `name` 必须与 world 中的 Gazebo model 名完全相同：

```yaml
update_rate: 50
obstacles:
  - name: "obstacle_cylinder_1"
    type: "linear"
    axis: [1, 0, 0]
    amplitude: 2.0
    speed: 0.3
    center: [0, 6, 2]
```

### 完成该阶段的验收标准

- 任务可以在无人逐条发目标的情况下自动执行到结束。
- 规划失败或轨迹超时不会导致旧指令无限继续，任务层能进入悬停、重试、返航或降落分支。
- 动态障碍模型按 YAML 设定运动，规划器能感知到其变化；不能只根据 Gazebo 画面猜测。
- 每次失败都能归类为感知、定位、规划、桥接、PX4 模式或任务逻辑问题。
- 若接入 YOLO，单帧误检不会立刻改变飞行目标，视觉观测过期或目标丢失会进入明确的悬停、搜索、返航或降落分支。
- 推荐验收标准：为一个固定动态场景定义明确的成功条件并重复至少十次；先达到可接受的稳定成功率，再提高障碍速度或场景复杂度。

### 可能需要修改的关键文件或参数位置

- `dynamic_obstacle_controller/config/obstacle_params.yaml`：`update_rate`、障碍物类型、中心、振幅、半径、速度和航点。
- `dynamic_obstacle_controller/src/obstacle_controller.py`：障碍轨迹计算和 Gazebo model state 更新。
- `dynamic_avoidance.world`：静态/动态模型位置、尺寸和名称。
- `craic_sim/scripts/generate_craic_2026_world.py`：场地、障碍和坐标换算。
- EGO 的地图范围、更新范围、膨胀、重规划阈值、速度/加速度参数。
- YOLO 结构化观测到任务目标的适配器：类别白名单、多帧确认、观测超时、坐标 frame、geofence 和安全观察距离。
- **根据代码结构推测**：完整任务管理器需要在 MissionControl 中新增或重构；当前 `Exploration/` 只有说明占位，仓库没有可直接运行的自主探索闭环，不应把 README 的模块描述当成现成功能。
- 多机属于后续独立课题；在单机动态任务稳定前不建议进入 `autoarming_Mult.launch`、端口和 namespace 调试。

## 15. YOLO 视觉支线：从离线检测到感知驱动任务

### 支线目标与边界

这条支线的最终目标不是“把 YOLO 跑出框”，而是建立一条可验证的视觉接口：相机稳定提供带时间戳和 frame 的图像，检测器输出结构化 2D 结果，必要时结合深度与 TF 得到 3D 目标，任务层经过多帧确认后再调用已经验收的航点或规划能力。

```text
RGB + CameraInfo [+ aligned depth]
        -> YOLO 推理
        -> Detection2DArray（类别、置信度、框、原始 header）
        -> 深度/TF 定位（可选）
        -> TargetObservationArray（目标位置、frame、时间、有效性）
        -> 多帧确认/目标选择/丢失处置
        -> 阶段 3 或阶段 9 已验收的控制接口
```

2D 检测、3D 定位、目标跟踪和飞行决策是四个不同问题。每一层都应能单独录包、回放和验收；不要把“检测到了”直接等同于“可以安全跟随”。

当前新增任务的第一个迭代只做四件事：跑通 `testEnv.py` 并记录离线基准；确认 D435i 实际 RGB/深度/CameraInfo 接口；用 remap 跑通 2D ROS 检测；让节点发布带原图 header 的结构化检测数组。这个迭代完成前，暂不训练自定义模型、不做 TensorRT、不做 3D 融合，也不接飞行控制。

### V0：YOLO 环境、离线推理与输出理解

#### 需要掌握的知识点

- 目标检测与图像分类的区别；类别、置信度、边界框、IoU、NMS 的含义。
- Ultralytics 推理结果中的 `boxes.xyxy`、`boxes.cls`、`boxes.conf` 和类别名映射。
- `imgsz`、`conf`、`iou`、设备、精度和模型大小对速度/准确率的影响。
- 首帧加载/预热时间与稳定推理时间的区别；输入 FPS、处理 FPS 和端到端延迟的区别。
- 当前权重的类别集合与实际无人机任务类别之间的边界；不能只凭权重文件名假定类别。

#### 推荐实践任务

1. 先运行 `yolo_detect/script/testEnv.py`，只验证 Python、Ultralytics、OpenCV、权重和样例图片能否正确加载；本阶段不启动 ROS、Gazebo 或飞控。
2. 将无限显示循环改成可配置的单次/固定次数测试，输出模型路径、设备、输入尺寸、类别、置信度和每阶段耗时，确保在无图形界面时也能保存结果并正常退出。
3. 分别用仓库 `bus.jpg`、一段短视频和至少 20 张与你任务场景接近的图片推理，保存成功与失败样例。
4. 预热后统计至少 100 帧的平均值、P95 推理延迟和有效处理 FPS；CPU 与 GPU/TensorRT 结果分开记录。
5. 改变 `conf`、`imgsz` 和模型尺寸时一次只改一项，记录漏检、误检和速度变化。

#### 验收标准

- 可以用一条固定命令重复完成离线推理，权重路径不依赖当前工作目录。
- 能打印并解释每个检测的类别、置信度和原图坐标框，而不只看标注图片。
- 有预热后的平均/P95 延迟与 FPS 记录，并注明硬件、模型、输入分辨率和软件环境。
- 能明确回答最终任务目标是否属于当前 `yolov8n.pt` 的类别；若不属于，V3 为必做项。

### V1：ROS 图像接口与仿真相机基线

#### 需要掌握的知识点

- `sensor_msgs/Image` 的 encoding、step、header、时间戳和 frame。
- `sensor_msgs/CameraInfo` 的 K/D/P，相机分辨率，以及 RGB 与深度是否配准。
- 相机 link 与 optical frame 的轴向约定；SDF 安装 pose、Gazebo 插件 frame 和 TF 的关系。
- 图像发布频率、仿真时间、队列积压、压缩/原始图像的取舍。

#### 本项目中应该重点阅读的文件/文件夹路径

- `simulation/px4_sim_files/px4_iris_sdf/iris_mid360/iris_mid360.sdf`
- `simulation/astra_gazebo_models/D435i/model.sdf`
- `simulation/sim_workspace/src/sensors/realsense_ros_gazebo/`
- `AstraDrone_ros1_ws/src/Utils/camera_sdk/`
- `yolo_detect/script/yolo_detect.py`
- `yolo_detect/script/yolo_detect_fusion.py`

`iris_mid360` 已包含 D435i 和 FPV 相机。当前 D435i SDF 明确配置的主要话题是：

```text
/realsense_d435i/color/image_raw
/realsense_d435i/color/camera_info
/realsense_d435i/depth/image_raw
/realsense_plugin/camera/local_pointclouds
```

这些是仓库配置事实，但仍应以实际运行时的 `rostopic list/info` 为准；外部 PX4/Gazebo 模型副本、插件加载失败或 namespace 都可能改变最终接口。

#### 推荐实践任务

1. 只启动相机仿真，不启动 YOLO，检查 RGB、深度和 CameraInfo：

   ```bash
   rostopic info /realsense_d435i/color/image_raw
   rostopic hz /realsense_d435i/color/image_raw
   rostopic echo -n 1 /realsense_d435i/color/camera_info
   rostopic echo -n 1 /realsense_d435i/depth/image_raw/encoding
   ```

2. 用 `rqt_image_view` 或保存单帧确认图像方向、视场和遮挡；在 Gazebo 中放置一个已知尺寸/位置的目标，验证画面变化符合相机安装方向。
3. 对比 RGB 与深度的分辨率、时间戳、frame 和目标轮廓；不要因为话题同时存在就假设已经像素对齐。
4. 静止和缓慢移动各录一段只包含 RGB、深度、CameraInfo、TF、`/clock` 和 Gazebo 真值的短 bag，作为后续检测回放输入。
5. 暂时用 remap 将实际 RGB 话题接到 2D 节点，确认能出标注图；随后再把话题改成参数，而不是长期依赖硬编码。

#### 验收标准

- RGB、深度和 CameraInfo 连续发布，无异常时间倒退，实际频率和 encoding 有记录。
- 能画出 `base_link -> camera_link -> optical frame` 的 TF/安装关系，并解释图像 x/y 与相机三维坐标轴。
- 已确认 RGB/深度是否对齐；若未对齐，V4 前必须先完成配准或选择已对齐深度话题。
- rosbag 回放时无需 Gazebo 也能稳定复现相机输入。

### V2：ROS 2D 检测节点工程化

当前 `yolo_detect.py` 可以订阅图像并发布标注图，但这只是演示节点。它硬编码模型和话题，手工转换图像，没有结构化检测消息，发布图又没有继承原始 header；这些问题应在接任务层前解决。

#### 推荐实践任务

1. 把以下内容改成私有参数并在 launch 中给出默认值：`model_path`、`image_topic`、`annotated_topic`、`conf`、`iou`、`imgsz`、`device`、`classes`、`frame_skip` 和 `publish_annotated`。
2. 使用 `cv_bridge` 按消息 encoding 转换输入，捕获转换异常；输出标注图继承输入消息的 `header.stamp` 和 `header.frame_id`。
3. 定义结构化检测数组。若选用现成 `vision_msgs`，先确认 ROS1 版本和依赖可用；否则在包内定义等价消息，至少包含：

   ```text
   header                 # 必须来自原图
   image_width/height
   detections[]:
     class_id/class_name
     confidence
     xmin/ymin/xmax/ymax  # 明确是原图像素坐标
   ```

4. 回调队列保持为 1，并采用“只处理最新帧”的工作线程或等价机制，避免推理慢于输入时延迟不断累积；不要仅用跳帧掩盖旧帧排队。
5. 补齐 `package.xml`、`CMakeLists.txt` 和 Python 安装规则。当前包只声明了基础 ROS 依赖，而源码实际还使用 `sensor_msgs`、`geometry_msgs`、`cv_bridge`、`tf2_ros` 等；只按实际节点拆分声明，不盲目增加依赖。
6. 在 bag 回放上测试空画面、多个目标、快速运动、输入中断和节点重启；测量 `输出时间 - 输入 header.stamp` 的端到端延迟。
7. 再决定是否导出 ONNX/TensorRT。先固定 PyTorch 基线，并验证导出模型的类别、框和置信度与基线在容许误差内一致；`pt2eng.py` 当前硬编码相对路径，不能直接作为可复现部署流程。

#### 验收标准

- launch 参数可以切换模型、话题、阈值和设备，源码中没有依赖个人绝对路径。
- 每一帧结构化检测结果与标注图使用原始图像时间戳/frame，框坐标能正确映射回原图。
- 输入速率高于推理速率时，延迟有界，不会持续增长；无检测时也会发布带 header 的空数组。
- 在固定 bag 上重复运行得到稳定的检测数量、类别和延迟统计。
- 尚未启用 V4 时，节点不发布目标 3D pose，更不发布任何 MAVROS 控制话题。

### V3：自定义数据集、训练和模型评估（按任务需要）

如果任务目标不在当前权重的类别集合中，或者仿真/航拍视角与现有训练数据差异明显，就必须做 V3。若只是学习当前权重已有的通用类别，可以先跳过，待任务定义明确后再返回。

#### 推荐实践任务

1. 先写清任务合同：需要识别哪些类别、最远距离、最小目标像素尺寸、允许漏检/误检、昼夜/遮挡/视角范围，以及推理硬件预算。
2. 数据按“场景/录制序列”划分 train/val/test，不能把相邻视频帧随机分到三个集合造成数据泄漏。
3. 同时保留正样本、困难样本和无目标负样本；检查漏标、框越界、类别不平衡和重复图片。
4. 先训练小模型建立基线，再一次只调整数据、增强、分辨率或模型规模中的一项。
5. 至少报告每类 precision、recall、PR 曲线、mAP50、mAP50-95、混淆情况、固定硬件延迟和典型失败样例。
6. Gazebo 合成数据可用于补姿态和距离覆盖，但必须单独保留真实/目标域测试集；不能用合成验证集的高分替代真实任务验证。
7. 保存数据版本、类别 YAML、训练配置、随机种子、最佳权重和评估脚本。模型文件名应包含任务/数据版本，避免所有实验都叫 `best.pt`。

#### 验收标准

- 独立测试集没有与训练集同源的相邻帧泄漏。
- 指标覆盖每个任务类别，且有按距离、遮挡或目标尺寸分组的失败分析。
- 已选择满足任务最低召回率和延迟预算的阈值；这个阈值来自验证/测试结果，不是凭感觉设定。
- 新模型能通过 V2 的同一 bag 回放测试，接口和时间戳行为没有因换权重而改变。

### V4：RGB-D 目标定位、稳定观测与任务集成

当前 `yolo_detect_fusion.py` 已展示“检测框中心取深度—相机内参反投影—TF 转换—发布 PoseStamped”的基本方向，但不能直接作为最终接口：RGB/深度没有严格同步，话题和 frame 硬编码，一帧多个目标被发布为无法区分类别/实例的多个 `PoseStamped`，TF 异常也只在日志中体现。

#### 需要掌握的知识点

- RGB/深度精确或近似时间同步、已对齐深度、深度 encoding/尺度和无效值。
- 用框中心深度、框内中位数、中心区域鲁棒统计或点云关联的取舍。
- 针孔反投影、畸变、图像缩放后坐标映射和相机内参一致性。
- 用图像时间戳查询 TF；点、方向和协方差在 frame 变换中的区别。
- 多帧确认、去抖、目标 ID/数据关联、超时和目标丢失状态机。

#### 推荐实践任务

1. 使用 `message_filters` 同步 RGB、已对齐深度和 CameraInfo，结构化检测结果通过原始 header 关联；记录同步时间差和丢帧率。
2. 不只取单个中心像素。先在检测框中心区域过滤 0、NaN、越界和明显离群值，再用中位数等鲁棒统计得到深度，并发布深度有效标志/失败原因。
3. 发布结构化 `TargetObservationArray`，至少携带类别、置信度、2D 框、3D 点、目标 frame、观测时间和有效性；多目标不能丢失身份信息。
4. 用观测时间查询 `target_frame <- camera_optical_frame` TF。若只能取得最新 TF，应把结果标为降级数据，不能悄悄当成严格同步结果。
5. 在 Gazebo 放置已知位置的单目标，按不同距离、方位和无人机姿态比较估计位置与真值，计算每轴误差、3D RMSE、P95 误差和无效深度比例。
6. 先让无人机悬停、目标静止，再测试无人机移动，最后才测试目标移动。每次只增加一种动态因素。
7. 任务层使用独立状态机，例如：

   ```text
   SEARCH
     -> CANDIDATE（单帧发现）
     -> CONFIRMED（N/M 帧满足类别、置信度和位置一致性）
     -> HOLD_AND_OBSERVE
     -> SEND_GOAL（调用已验收的任务/规划接口）
     -> REACHED / TARGET_LOST / TIMEOUT
     -> RETURN_OR_LAND
   ```

8. 初次闭环只做“检测到目标后悬停并记录位置”，再做“飞到预设观察点”，最后才做跟随。目标相对位置不得直接映射成无边界速度指令。

#### 验收标准

- 静止目标的 3D 位置误差有真值对比，且 TF、深度或同步失败会产生明确的无效结果，而不是发布 `(0,0,0)`。
- 多目标输出保留类别、置信度和时间戳，任务层有确定的选择规则和多帧确认门限。
- 视觉输入中断或目标丢失时，旧目标不会无限有效；任务层进入悬停、重新搜索、返航或降落中的预设分支。
- YOLO/融合节点始终不拥有 MAVROS setpoint 发布权；控制仍由阶段 3 或阶段 9 的唯一控制节点负责。
- 推荐第一项闭环验收：固定场景重复十次“起飞—悬停—发现指定目标—保持观察—返航—降落”，检测、任务和飞行失败原因均可区分。

### YOLO 支线优先修改位置

- `yolo_detect/script/testEnv.py`：有限次数运行、命令行参数、无 GUI 输出和基准统计。
- `yolo_detect/script/yolo_detect.py`：参数化、标准图像转换、结构化 2D 检测消息、原始 header、最新帧处理和延迟统计。
- `yolo_detect/script/yolo_detect_fusion.py`：RGB-D 同步、话题/frame 参数化、鲁棒深度、多目标数组、TF 时间戳和无效观测。
- `yolo_detect/launch/yolo_detect.launch`：模型、输入/输出话题、阈值、设备和可视化参数；D435i 话题通过参数/remap 对齐。
- `yolo_detect/CMakeLists.txt`、`yolo_detect/package.xml`：消息生成、真实运行依赖和 Python 安装。
- `yolo_detect/CATKIN_IGNORE`：只在依赖清单、构建范围和回退方式明确后处理。
- `simulation/px4_sim_files/px4_iris_sdf/iris_mid360/iris_mid360.sdf` 与 `simulation/astra_gazebo_models/D435i/model.sdf`：相机安装 pose、插件话题和 frame；修改后确认实际运行副本。
- **根据代码结构推测**：结构化检测/目标观测消息和视觉任务管理器需要新增或重构；当前仓库没有一条已验收的 YOLO 驱动自主飞行闭环。

## 16. 贯穿所有阶段的实验纪律

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
11. **视觉结果必须保留原始时间信息**：检测、深度和 TF 都以输入图像时间戳为准；不能用“当前时间”掩盖处理延迟。
12. **空结果和无效结果也是接口的一部分**：无检测、无深度、TF 失败或观测过期时发布明确状态，绝不把零坐标当成有效目标。
13. **感知节点不直接拥有飞行控制权**：YOLO/融合层只发布观测，任务层负责确认和限界，唯一控制节点负责执行。

## 17. 建议的最终学习顺序总结

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

### 并行视觉支线：从 YOLO 离线推理到目标观测

阶段 0 后即可完成 V0～V2：离线推理、相机基线和 ROS 2D 检测；任务需要自定义类别时再训练和评估；阶段 3 的安全任务状态机完成后，才结合已对齐深度、CameraInfo 与 TF 做 V4。完成后，你得到的是带类别、置信度、位置、frame 和时间戳的可靠观测接口，而不是一个直接控制无人机的检测脚本。

### 第七步：先验收 Mid360 仿真输入

理解 Mid360 SDF、LiDAR/IMU 插件、话题类型、频率、时间戳、噪声和安装关系，保存静止及单轴运动基线 bag。完成后，你能先判断原始传感器输入是否可信，避免把仿真输入问题误判成 FAST-LIO 算法问题。

### 第八步：独立学会 FAST-LIO

从 `mapping_mid360.launch` 和 `mid360.yaml` 入手，掌握预处理、IMU 传播、点云匹配、外参、滤波分辨率和 registered cloud，并用 MAVROS/Gazebo 参考轨迹评价 `/Odometry`。完成后，你能独立判断 FAST-LIO 是否稳定、参数是否合理，以及它与 MAVROS local frame 的关系。

### 第九步：单独学会 EGO-Planner

先在 EGO 自己的 SO3 模拟器中掌握目标、地图、B-spline、速度/加速度约束和重规划。完成后，你可以在独立规划仿真中生成平滑避障轨迹，但此时还没有控制 PX4 无人机。

### 第十步：连接 FAST-LIO、EGO 和 PX4

先验证无障碍 `PositionCommand -> MAVROS PositionTarget` 桥接，再用阶段 7 验收的 `/Odometry` 和 `/cloud_registered` 替换 EGO 内部模拟输入，完成 frame 转换并保证唯一控制发布者。完成后，PX4/Gazebo 无人机能够基于 FAST-LIO 定位建图结果跟随 EGO 轨迹绕开静态障碍。

### 第十一步：提升到感知驱动的任务级自主

加入动态障碍、复杂 world、任务状态机、超时、重试、返航和量化成功率。若 YOLO V4 已通过验收，再加入“发现—确认—观察/接近—目标丢失处置”。完成后，无人机能够按预设或视觉感知的任务目标自主选择和执行动作，并在仿真环境变化时进行重规划和安全处置。

这条顺序的核心是：先把“能稳定控制”做扎实，再加入“按轨迹运动”；随后把 Mid360/FAST-LIO/EGO 主线与相机/YOLO 视觉支线分别验收，最后才在任务层汇合。对当前仓库而言，阶段 0～5 优先围绕 `autoarming_control`，阶段 6～7 完成 Mid360/FAST-LIO，阶段 8 学规划，阶段 9～10 进入 FAST-LIO/EGO/PX4 自主闭环；YOLO 按 V0～V4 并行推进，不直接成为飞控前置条件或 setpoint 发布者。
