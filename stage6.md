# 阶段 6 教程：EGO-Planner 独立仿真与重规划

> 面向第一次接触运动规划的新手。本文对应 `studymap.md` 的阶段 6，文件名为 `stage6.md`。
>
> 本阶段只运行 **EGO-Planner 自带的地图、SO3 控制器和动力学模拟器**，不启动 PX4、Gazebo、MAVROS、FAST-LIO 或 Mid360，也不编写 `PositionCommand -> MAVROS` 桥接。

## 1. 开始前先复习专用术语

先把英文翻译成直白含义。现在不必推公式，但必须能说清每个模块“输入什么、输出什么”。

| 术语 | 直白解释 | 在本阶段中的作用 |
|---|---|---|
| EGO-Planner | 一种面向四旋翼的局部轨迹规划器 | 根据目标、里程计和障碍地图反复生成安全轨迹 |
| odometry / odom | 估计的当前位置、姿态和速度 | `/visual_slam/odom` 在这里来自自带模拟器，不是视觉 SLAM |
| occupancy grid | 把空间切成小格并标记空闲、占用或未知 | 规划器用它判断哪些位置可能碰撞 |
| voxel / resolution | 三维小方格 / 每格边长 | 默认 `0.1 m`；越小越精细，也越耗内存和计算 |
| inflation | 把障碍在地图中向外“加粗” | 给有体积的无人机留安全余量 |
| local sensing | 只模拟无人机附近可见的障碍 | `/pcl_render_node/cloud` 给规划器提供局部点云 |
| sensing horizon | 传感器能看到的最大距离 | 默认约 `5 m`，看不到的远处障碍以后才会出现 |
| planning horizon | 每次局部规划向前看的距离 | 默认 `7.5 m`，本例按感知距离的约 1.5 倍设置 |
| global trajectory | 从起点到最终目标的全局参考路线 | 告诉局部规划器总体向哪里走，不等于最终控制轨迹 |
| local trajectory | 当前一小段可执行轨迹 | 随着无人机运动和新障碍出现而不断重规划 |
| B-spline | 由控制点和节点向量定义的平滑曲线 | `/planning/bspline` 携带局部轨迹控制点、节点和起始时间 |
| control point / knot | 控制曲线形状的点 / 定义曲线时间分段的数值 | 轨迹服务器用它们还原位置、速度和加速度 |
| velocity / acceleration / jerk | 速度 / 加速度 / 加速度变化率 | 分别约束飞得多快、变速多猛、动作多突兀 |
| feasibility | 轨迹能否满足速度和加速度等动力学限制 | 不可行轨迹需要延长时间或重新优化 |
| cost function | 优化器要尽量减小的综合“代价” | 平滑、避障、可行性和贴合参考路线会互相权衡 |
| `lambda_*` | 各项代价的权重 | 权重越大，优化器越重视对应目标；不是越大越好 |
| `dist0` | 优化器希望与障碍保持的距离尺度 | 增大通常更保守，但狭窄区域更可能无解 |
| FSM | 有限状态机，把规划过程分成若干状态 | 管理等目标、生成轨迹、执行、重规划和急停 |
| replan | 飞行中用当前状态重新生成后续轨迹 | 遇到新障碍或飞过一段距离后更新局部轨迹 |
| emergency stop | 来不及找到安全新轨迹时生成停止轨迹 | 对应 `EMERGENCY_STOP`，不是关闭节点或瞬间停住 |
| trajectory server | 把 B-spline 按当前时间连续求值的节点 | 输出位置、速度、加速度和 yaw 指令 |
| `PositionCommand` | EGO 自带控制链的轨迹指令消息 | `/planning/pos_cmd` 约 `100 Hz` 连续送给 SO3 控制器 |
| SO3 controller | 根据期望位置、速度、加速度计算姿态和推力 | 输出 `so3_cmd`，它不是 PX4 的 OFFBOARD 控制器 |
| dynamics simulator | 用动力学方程模拟四旋翼怎样运动 | 接收 `so3_cmd`，输出 `/visual_slam/odom` |
| mock map | 程序生成的假障碍地图 | `/map_generator/global_cloud` 是本阶段的环境真值地图 |
| 2D Nav Goal | RViz 中用鼠标给出平面位置和朝向的工具 | 触发手动目标；当前源码把目标高度固定为 `1.0 m` |
| `CATKIN_IGNORE` | 告诉 catkin 忽略某个包的空文件 | 当前 EGO 相关包默认不会被工作空间编译 |
| rosbag | 录制 ROS 话题的数据文件 | 保存 odom、B-spline 和控制指令作为验收证据 |

牢记这条主线：**地图和 odom 进入规划器，规划器发布 B-spline，轨迹服务器连续生成指令，SO3 控制器驱动模拟器，模拟器再产生新的 odom。**

## 2. 本次任务与完成标准

你要亲手完成六件事：

1. 有计划地启用 EGO 最小包集合，并独立编译成功；
2. 启动 mock map、规划节点、轨迹服务器、SO3 控制器和动力学模拟器；
3. 用 RViz 发送手动目标，沿话题链证明无人机确实由 EGO 自带模拟器控制；
4. 改为五个预设航点，理解全局参考轨迹和局部重规划的区别；
5. 一次只修改 `max_vel`、`max_acc`、分辨率、膨胀或 `dist0`，用数据解释变化；
6. 修正手动目标高度，制造一次可解释的规划失败，并保存日志和 rosbag。

最终你应该能不看本文画出以下闭环：

```text
RViz 目标 / 预设航点
          ↓
ego_planner_node + 局部占据地图 ← /pcl_render_node/cloud
          ↓ /planning/bspline
traj_server
          ↓ /planning/pos_cmd
SO3 controller
          ↓ so3_cmd
so3_quadrotor_simulator
          ↓ /visual_slam/odom
          └──────────────→ 规划器、局部感知和控制器
```

## 3. 先认清仓库现状和边界

本文中的 EGO 根目录为：

```text
AstraDrone_ros1_ws/src/Planner/ego-planner/
```

当前源码有四个容易踩坑的事实：

- EGO 相关包带有 `CATKIN_IGNORE`，所以现在执行 `rospack find ego_planner` 很可能显示找不到包；这不是 ROS 坏了。
- `run_in_sim.launch` 是手动目标模式，但不包含 RViz；需要单独启动 `rviz.launch`。
- `/planning/bspline` 仅在新规划或重规划时发布，频率稀疏且不固定；连续高频控制输出是 `/planning/pos_cmd`。
- 当前 `waypointCallback()` 只使用 `nav_msgs/Path` 的 `poses[0]`。waypoint generator 即使发布圆或 8 字的一组点，规划器也只取第一个点作为手动目标。

不要同时运行本仓库的 PX4/Gazebo 主链和 EGO 自带模拟器。两套仿真没有在本阶段建立坐标、动力学或控制接口关系。

## 4. 第一步：启用并编译最小包集合

### 4.1 先记录现状

新终端执行：

```bash
cd ~/AstraDroneOpen
find AstraDrone_ros1_ws/src/Planner/ego-planner -name CATKIN_IGNORE -print | sort
git status --short
```

不要使用 `git clean`、`git reset` 或删除整个 `build/devel`。当前工作区可能还有你前面阶段的修改。

### 4.2 只启用本仿真链需要的 15 个包

重命名标记是可恢复操作。下面保留 `map_generator`、`multi_map_server` 和 `rviz_plugins` 的忽略状态，因为 `run_in_sim.launch` 的最小链不需要它们。

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws/src/Planner/ego-planner

for EGO_PKG in \
  planner/plan_env \
  planner/path_searching \
  planner/bspline_opt \
  planner/traj_utils \
  planner/plan_manage \
  uav_simulator/Utils/cmake_utils \
  uav_simulator/Utils/pose_utils \
  uav_simulator/Utils/quadrotor_msgs \
  uav_simulator/Utils/uav_utils \
  uav_simulator/Utils/waypoint_generator \
  uav_simulator/Utils/odom_visualization \
  uav_simulator/local_sensing \
  uav_simulator/mockamap \
  uav_simulator/so3_control \
  uav_simulator/so3_quadrotor_simulator
do
  mv "$EGO_PKG/CATKIN_IGNORE" "$EGO_PKG/CATKIN_IGNORE.disabled"
done
```

检查结果：

```bash
find . -name CATKIN_IGNORE -print | sort
find . -name CATKIN_IGNORE.disabled -print | wc -l
```

第二条应输出 `15`。第一条应只剩上述三个非必需包；若某个 `mv` 报“文件不存在”，先检查是否以前已经重命名，不要盲目创建文件。

### 4.3 检查依赖并编译

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
source /opt/ros/noetic/setup.bash
rosdep check --from-paths src/Planner/ego-planner --ignore-src
```

`rosdep check` 若报告缺少系统依赖，先按它列出的键补齐；不要凭感觉改 CMake。仓库的部分 `package.xml` 较旧，若 rosdep 与编译日志冲突，以 CMake 的第一条真实缺失依赖为准。

只编译本阶段包，避免把工作区其他实验包一并拉进来：

```bash
catkin_make -j2 -DCATKIN_WHITELIST_PACKAGES="cmake_utils;pose_utils;quadrotor_msgs;uav_utils;waypoint_generator;odom_visualization;local_sensing_node;mockamap;so3_control;so3_quadrotor_simulator;plan_env;path_searching;bspline_opt;traj_utils;ego_planner"
source devel/setup.bash

rospack find ego_planner
rospack find mockamap
rospack find local_sensing_node
roslaunch --nodes ego_planner run_in_sim.launch
```

最后一条只解析 launch，不会起飞。能列出节点才进入下一步。`CATKIN_WHITELIST_PACKAGES` 会留在 CMake 缓存中；阶段 6 完成后若要恢复全工作区构建，使用：

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```

若要重新禁用 EGO，则把这 15 个 `CATKIN_IGNORE.disabled` 改回 `CATKIN_IGNORE`，再重新编译；不要删除源码目录。

### 4.4 首次运行前补一个确定性修正

打开 `planner/plan_manage/src/ego_replan_fsm.cpp`，在 `EGOReplanFSM::init()` 开头已有初始化语句附近补上：

```cpp
trigger_ = false;
have_new_target_ = false;
flag_escape_emergency_ = false;
```

当前这三个布尔量没有全部显式初始化，可能产生不稳定的初始行为。修改后重新执行：

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make --pkg ego_planner -j2
source devel/setup.bash
```

## 5. 第二步：第一次完整手动目标仿真

每个命令放在独立终端。所有终端都先 `source /opt/ros/noetic/setup.bash` 和工作空间的 `devel/setup.bash`。

### 终端 1：启动 EGO 自带仿真

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch ego_planner run_in_sim.launch
```

正常情况下会出现这些核心节点：

```text
/ego_planner_node              规划 FSM、地图与轨迹优化
/traj_server                   B-spline 连续求值
/waypoint_generator            转发 RViz 目标
/mockamap_node                 生成全局障碍地图
/pcl_render_node               生成无人机附近的局部点云
/so3_control                   位置指令转姿态/推力指令
/quadrotor_simulator_so3       四旋翼动力学模拟
/odom_visualization            模型和里程计可视化
```

### 终端 2：启动 RViz

```bash
source /opt/ros/noetic/setup.bash
source ~/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash
roslaunch ego_planner rviz.launch
```

RViz 的 Fixed Frame 应为 `world`。先看到地图和位于约 `(-18, 0, 0)` 的模型，再发送目标。

### 终端 3：目标发送前检查链路

```bash
source /opt/ros/noetic/setup.bash
source ~/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash
rosnode list
rostopic hz /visual_slam/odom
rostopic hz /pcl_render_node/cloud
rostopic info /planning/pos_cmd
```

预期：odom 接近 `200 Hz`，局部点云接近 `30 Hz`；目标发送前 `/planning/pos_cmd` 可以已有发布者但还没有有效连续消息。频率会受电脑负载影响，不要求数字完全相等。

### 终端 4：录制验收数据

```bash
mkdir -p ~/stage6_evidence
rosbag record -O ~/stage6_evidence/manual_baseline.bag \
  /visual_slam/odom \
  /planning/bspline \
  /planning/pos_cmd \
  /waypoint_generator/waypoints \
  /rosout
```

不要默认录制全局和局部点云，它们会让 bag 很快变大；需要研究地图时再单独短录。

### 在 RViz 发送目标

1. 点击工具栏的 **2D Nav Goal**；
2. 在地图内空闲区域单击并拖出箭头，第一次目标可选起点前方数米的位置；
3. 观察终端中的 FSM 状态和 RViz 中的新轨迹；
4. 到达后再发送第二个目标，验证能够从 `WAIT_TARGET` 再次开始。

你应该看到类似状态变化：

```text
INIT -> WAIT_TARGET -> GEN_NEW_TRAJ -> EXEC_TRAJ
                           ↕
                       REPLAN_TRAJ
到达目标后：EXEC_TRAJ -> WAIT_TARGET
```

同时检查：

```bash
rostopic echo -n 1 /planning/bspline
rostopic hz /planning/pos_cmd
rostopic echo -n 1 /planning/pos_cmd
rostopic echo -n 1 /visual_slam/odom
```

关键判断：

- `/planning/bspline` 的 `traj_id` 在重规划后增加，但该话题不会以固定高频持续发布；
- `/planning/pos_cmd` 应接近 `100 Hz`，含 position、velocity、acceleration、yaw；
- odom 中的位置应追随 `PositionCommand`，它来自 `quadrotor_simulator_so3`，不是 MAVROS；
- 到达后 `PositionCommand` 保持终点、速度和加速度归零。

完成两次有效目标后停止 rosbag，再正常 `Ctrl+C` 停止 launch。本阶段没有 PX4 的 armed/disarmed 概念。

## 6. 第三步：把源码读成一条因果链

按下面顺序阅读，不要从优化公式开始硬啃：

1. `launch/run_in_sim.launch`：找出地图尺寸、目标类型、速度限制和各节点；
2. `launch/advanced_param.xml`：按 FSM、grid map、manager、optimization、B-spline 五组看参数；
3. `src/ego_replan_fsm.cpp`：重点看 `waypointCallback()`、`execFSMCallback()`、`checkCollisionCallback()`；
4. `src/planner_manager.cpp`：看全局轨迹、`reboundReplan()` 和 `EmergencyStop()`；
5. `planner/bspline_opt/src/bspline_optimizer.cpp`：只定位四项权重与 `dist0` 如何进入代价；
6. `src/traj_server.cpp`：看 B-spline 如何求出 position、velocity、acceleration 和 yaw；
7. `launch/simulator.xml`：确认 `pos_cmd -> so3_cmd -> odom` 的自带控制闭环。

读完后，用自己的话回答：谁发现障碍、谁决定轨迹、谁把曲线变成当前指令、谁真正改变模拟无人机状态？答不出来就沿话题再查一次，不要继续调参。

## 7. 第四步：预设五航点实验

`simple_run.launch` 已设置 `flight_type=2`，会读取五个预设航点并自动开始，而且包含 RViz。先复制一份实验入口，避免混淆基线：

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/launch
cp simple_run.launch stage6_preset.launch
```

把 `stage6_preset.launch` 中五个航点改成地图范围内的一条闭合路线，例如：

```text
point0 = (-14,  0, 1.0)
point1 = (-10,  4, 1.0)
point2 = ( -6,  0, 1.2)
point3 = (-10, -4, 1.0)
point4 = (-14,  0, 1.0)
```

启动前关闭手动模式的 `run_in_sim.launch`，保证同一 ROS master 上只有一套 EGO 仿真：

```bash
source ~/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash
roslaunch ego_planner stage6_preset.launch
```

预设航点先生成一条分段全局参考轨迹，EGO 执行的仍是朝局部目标生成的 B-spline，并会在飞行中重规划。不要把 RViz 中较长的全局线与当前短 B-spline 当成同一条曲线。

验收：五个点全部位于 `x/y ∈ [-20, 20]`、`z ∈ [0, 2.5)`，无人机按全局路线推进，`traj_id` 至少变化一次，并最终停在最后一点附近。

## 8. 第五步：一次只改一个参数

先保存基线文件和 bag。每轮都使用同一组起点、目标、mock map seed，只改一项；改完要重新启动节点，因为这些参数只在初始化时读取。

| 实验 | 基线 → 对照 | 你应观察什么 |
|---|---|---|
| 最大速度 | `max_vel: 2.0 → 1.0` | `/planning/pos_cmd/velocity` 峰值下降，完成时间通常增加 |
| 最大加速度 | `max_acc: 3.0 → 1.5` | 起停和转弯更缓，速度变化斜率降低 |
| 地图分辨率 | `resolution: 0.1 → 0.2` | 栅格更粗、计算量下降，但窄障碍表达变差 |
| 障碍膨胀 | `obstacles_inflation: 0.099 → 0.20` | 轨迹离障碍更远，窄通道更容易无解 |
| 优化安全距离 | `dist0: 0.5 → 0.8` | 碰撞代价更早生效，轨迹更保守，优化难度可能增加 |

位置：`max_vel/max_acc` 在运行 launch 中传入；其余三项在 `advanced_param.xml`。`max_vel/max_acc` 会同时传给 manager、optimization 和 B-spline 限制，不要只改其中一处造成参数不一致。

每轮建议记录：

```bash
rosbag record -O ~/stage6_evidence/实验名.bag \
  /visual_slam/odom /planning/bspline /planning/pos_cmd /rosout
```

并用下面的工具观察三个方向的指令，而不是只看动画：

```bash
rqt_plot \
  /planning/pos_cmd/velocity/x \
  /planning/pos_cmd/velocity/y \
  /planning/pos_cmd/velocity/z
```

实验记录表至少包含：改动前值、改动后值、同一目标、完成/失败、耗时、B-spline 次数、速度峰值和你的解释。一次改多项得出的结果不能用于判断单个参数作用。

## 9. 第六步：修正手动目标高度

当前 `ego_replan_fsm.cpp` 的手动目标写成：

```cpp
end_pt_ << msg->poses[0].pose.position.x,
           msg->poses[0].pose.position.y,
           1.0;
```

因此 RViz 箭头的 z 不会生效。不要直接改成消息中的 z：标准 2D Nav Goal 通常给出的 z 是 `0`，那会把目标放到地面。更稳妥的练习是增加参数 `fsm/manual_target_height`：

1. 在 `ego_replan_fsm.h` 的参数成员中增加 `double manual_target_height_;`；
2. 在 `init()` 中用 `nh.param("fsm/manual_target_height", manual_target_height_, 1.0);` 读取；
3. 把上面的 `1.0` 改为 `manual_target_height_`；
4. 在 `advanced_param.xml` 的 FSM 参数区加入：

   ```xml
   <param name="fsm/manual_target_height" value="1.2" type="double"/>
   ```

5. 重新编译 `ego_planner`，启动后用 `rosparam get /ego_planner_node/fsm/manual_target_height` 核对；
6. 发新目标，检查 `/planning/pos_cmd` 和最终 odom 的 z 接近 `1.2 m`。

高度必须同时满足地图 z 范围和虚拟天花板。当前地图高 `3.0 m`、`virtual_ceil_height=2.5 m`，所以第一次实验使用 `1.0～1.5 m`，不要把目标设置到 `2.5 m` 以上。

## 10. 第七步：制造并解释一次失败

最安全、最容易复现的失败是不合适的目标，而不是人为扰乱动力学：

1. 保持低速基线；
2. 在 RViz 中把目标点放到明显的障碍内部，或把 `dist0` 和 inflation 调大后选择狭窄区域；
3. 观察是否反复出现 `GEN_NEW_TRAJ`、`final_plan_success=0` 或无法产生新的 B-spline；
4. 保存 `/rosout` 和终端日志；
5. 把目标改到空闲区域，确认系统恢复并生成有效轨迹。

解释时必须回答：目标是否被膨胀地图判为占用？失败发生在全局参考生成还是局部 B-spline 优化？有没有新的 `traj_id`？`/planning/pos_cmd` 是继续上一轨迹、保持终点，还是没有新指令？

若局部轨迹前方突然发现碰撞，安全检查会先尝试重规划；剩余碰撞时间小于 `emergency_time_` 且重规划失败时才进入 `EMERGENCY_STOP`。急停是生成一条在当前位置减速停止的轨迹，不是杀死节点。新手阶段完成一次有证据的规划失败即可，不要为了看到急停而设置危险的极端速度。

## 11. waypoint generator 的形状实验：只做接口观察

`sample_waypoints.h` 提供 `circle()`、`eight()` 和 `point()`。默认尺度有些点超出当前 `40 × 40 × 3 m` 地图或超过 `2.5 m` 天花板，直接使用前应缩小 `scale/r/h` 并重新编译 `waypoint_generator`。

还要注意两个源码事实：

- goal 回调接受字符串 `circle`、`eight`、`points`；trigger 回调使用 `circle`、`eight`、`point`，命名不一致；
- 手动模式的 EGO 回调只读取收到 Path 的第一个 pose，所以当前代码不会沿整组点飞出圆或 8 字。

本实验只要求把 `run_in_sim.launch` 中 `waypoint_type` 改为相应字符串，触发一次后执行：

```bash
rostopic echo -n 1 /waypoint_generator/waypoints
```

数清 Path 中有多少 pose，再对照 `ego_replan_fsm.cpp` 说明为什么 EGO 只选第一个目标。若以后要跟完整路径，应另行设计“逐点推进或整条参考路径”的接口；这不属于本阶段必做任务。

## 12. 常见故障速查

| 现象 | 最可能原因 | 检查与处理 |
|---|---|---|
| `package 'ego_planner' not found` | `CATKIN_IGNORE` 未改名、未编译或新终端未 source | 查标记、看编译第一处错误、重新 source `devel/setup.bash` |
| CMake 找不到 `quadrotor_msgs/cmake_utils` | 最小依赖包未同时启用 | 核对 15 个 `.disabled`，不要只启用 `plan_manage` |
| `run_in_sim` 后没有 RViz | 该 launch 本来就不包含 RViz | 另开终端运行 `roslaunch ego_planner rviz.launch` |
| 日志一直 `no odom` | 动力学模拟器没启动或话题 remap 错 | `rosnode info /quadrotor_simulator_so3`，再查 `/visual_slam/odom` |
| 日志 `wait for goal` | 手动模式尚未收到合法目标 | 检查 RViz Fixed Frame=`world` 和 `/move_base_simple/goal` |
| 有目标但无 B-spline | 目标占用、地图未更新或规划失败 | 查 `/pcl_render_node/cloud`、日志和目标位置 |
| 有 B-spline 但无 pos_cmd | `traj_server` 未运行或话题不匹配 | `rosnode info /traj_server`，检查订阅 `/planning/bspline` |
| pos_cmd 有、odom 不动 | SO3 控制器或模拟器链断开 | 检查 `so3_cmd` 的发布者/订阅者及两个节点状态 |
| RViz 地图不显示 | Fixed Frame 或显示话题不对 | 使用 `world`，检查 `/map_generator/global_cloud` |
| 改参数后毫无变化 | 参数只在启动时读取，或改错命名空间 | 完全重启 launch，用 `rosparam get` 核对实际值 |
| 频繁重规划或失败 | 速度太高、膨胀/`dist0` 太大、通道太窄 | 先恢复基线，再一次只改一项定位原因 |

## 13. 最终验收清单

只有下面全部做到，阶段 6 才算完成：

- [ ] 能独立编译并找到 15 个最小链相关包，知道剩余三个包为何没启用；
- [ ] 能指出 EGO 自带模拟器的节点和 odom 来源，明确它不是 PX4/Gazebo；
- [ ] 手动发送至少两个有效目标，看到 FSM、B-spline、`PositionCommand` 和 odom 的因果关系；
- [ ] 能解释为什么 B-spline 话题稀疏而 `PositionCommand` 连续；
- [ ] 五个预设航点全部在地图和天花板范围内，并最终到达最后一点；
- [ ] 完成至少三组单变量对照实验，其中必须包含速度限制和一个地图/避障参数；
- [ ] 手动目标高度参数化后，命令和 odom 的 z 按预期生效；
- [ ] 制造一次规划失败，能根据日志、地图和 `traj_id` 说明原因；
- [ ] 保存 launch/参数差异、终端日志、rosbag、RViz 截图和一张实验结果表；
- [ ] 能在两分钟内不看文档画出完整闭环并解释每个模块。

最后提醒：阶段 6 的价值不是“让模型动起来”，而是学会用输入、状态机、轨迹消息、控制输出和反馈证据解释一次规划。只有当你能定位“没有目标、没有地图、规划失败、轨迹服务器断链、控制器断链”分别发生在哪里，才算真正学会 EGO-Planner 的基本使用。
