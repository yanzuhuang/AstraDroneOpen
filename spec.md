# AstraDroneOpen 仿真自主控制开发说明

> 审阅基线：当前工作区 `HEAD 754f3f5b536bf595db27d5edafc0565303c053eb`，审阅日期 2026-07-10。本文行号均以该基线的源码为准；以后源码增删行后，应重新用 `nl -ba <文件>` 核对。

本文面向当前阶段的核心目标：理解并修改项目，使 PX4 无人机在 Gazebo 仿真中按指定轨迹、速度、高度、航向和动作序列自主运动。本文只记录当前代码事实、明确标注的推断和可实施建议，不把 README 中的规划性描述当成已经实现的功能。

标记约定：

- **代码事实**：可以从当前仓库源码、launch、配置或构建产物直接确认。
- **推断**：根据调用链或命名推断，尚未通过一次完整运行实验确认。
- **建议**：面向后续仿真实验的修改方案，本文没有替用户修改原代码。
- **当前阻断**：不先处理就无法形成预期闭环的问题。

为缩短表格，文中的 `offboard/` 统一指 `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/`，`plan_manage/` 统一指 `AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/`，`uav_simulator/` 统一指 `AstraDrone_ros1_ws/src/Planner/ego-planner/uav_simulator/`。首次分析某文件时仍给出完整仓库路径。

## 0. 先看结论

当前最直接、最适合先修改的飞行控制链是：

```text
scripts/run_sh/pc_example.sh
  -> PX4 SITL + Gazebo + iris_mid360
  -> MAVROS
  -> offboard/autoarming_control.cpp
  -> /mavros/setpoint_position/local
  -> PX4 位置控制器
  -> Gazebo 无人机运动
```

若只想改变默认圆/方形轨迹，优先修改：

1. `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/launch/autoarming_control.launch`
2. `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src/autoarming_control.cpp`

最重要的当前事实：

- 默认演示真正控制 PX4/Gazebo 无人机的是 `autoarming_control.cpp`，不是 EGO-Planner。
- `autoarming_control.launch` 第 18 行的 `speed=1.5` 当前**无效**，源码没有读取 `speed`。
- launch 第 16 行的 `takeoff_height=1.0` 当前也**无效**；起飞目标直接使用拼写为 `hight` 的参数。
- 圆半径由源码第 97 行的私有参数 `radius` 控制，但默认 launch 没有设置它，因此当前默认半径是 `2.0 m`。
- EGO-Planner 及其 SO3 模拟器当前全部带 `CATKIN_IGNORE`，而且它输出到自带四旋翼动力学模拟器，不会直接驱动默认 PX4/Gazebo 无人机。
- FAST-LIO 在默认演示中负责建图/里程计输出，但没有代码把 `/Odometry` 接入默认 Offboard 控制器；默认飞行不依赖 FAST-LIO 规划。
- `simulation/px4_sim_files` 中的 PX4 launch、机型和 airframe 会被复制到外部 `~/PX4-Autopilot`。`roslaunch px4 astra_example.launch` 实际读取外部副本，而不是仓库内原文件。
- 多个 world 参数把 `$(find <package>)` 与 `../../../...` 直接拼接且中间缺少 `/`；当前 `rospack find` 输出没有尾斜杠，因此应修正路径或用绝对 `world:=...` 参数绕过。
- 任意时刻只应有一个节点持续发布 MAVROS setpoint；不要同时运行 `autoarming_control`、`position_control` 和另一个规划桥接器。

## 1. 项目结构与当前可用状态

### 1.1 根目录

| 路径 | 实际作用 | 仿真阶段是否重点 |
|---|---|---|
| `AstraDrone_ros1_ws/` | ROS1 catkin 工作空间；包含 Offboard、FAST-LIO、EGO-Planner 源码及工具包 | 是 |
| `simulation/` | Gazebo 模型、world、PX4 自定义资源和独立仿真 catkin 工作空间 | 是 |
| `scripts/` | 环境初始化、工作空间构建、一键运行、录包和实验工具 | 是 |
| `docs/` | 项目原始说明；部分内容是概览或规划性描述 | 参考 |
| `third_party/` | Livox SDK、NLopt、AprilTag、GeographicLib、RealSense 等第三方源码 | 间接相关 |
| `hardware/` | PCB、BOM、结构设计占位 | 当前不是重点 |
| `media/`、`system_images/` | 图片、视频、系统镜像占位 | 当前不是重点 |
| `README.md` | 项目总览和安装入口 | 参考 |
| `spec.md` | 本文，作为仿真控制开发主文档 | 是 |

`AstraDrone_ros1_ws/build/`、`AstraDrone_ros1_ws/devel/`、`simulation/sim_workspace/build/` 和 `devel/` 是生成目录。修改逻辑应改 `src/` 下源码，不要编辑生成文件。

**代码事实**：顶层 README 描述了 ROS2 工作空间，但当前工作区根目录并不存在 `AstraDrone_ros2_ws/`。本文只分析实际存在的 ROS1/PX4/Gazebo 内容。

### 1.2 ROS1 模块真实状态

| 模块 | 当前仓库内容 | 编译状态/说明 |
|---|---|---|
| `Communication/` | `serial_tool` | 带 `CATKIN_IGNORE` |
| `Control/` | `rc_obstacle_avoidance` | 带 `CATKIN_IGNORE`；依赖仓库中不存在的 Fast-Planner 话题约定 |
| `Detection/` | AprilTag、ArUco、目标预测、YOLO | 当前都带 `CATKIN_IGNORE`，不是默认仿真链 |
| `Exploration/` | 只有空说明文件 | 无可运行探索节点 |
| `Land/` | `astra_auto_land` | 带 `CATKIN_IGNORE`；默认演示不用它 |
| `MissionControl/` | `offboard` | 默认控制核心；当前已生成 `autoarming_control` 等可执行文件 |
| `Planner/` | EGO-Planner 及其独立 UAV simulator | 所有相关包均带 `CATKIN_IGNORE` |
| `SLAM/` | FAST-LIO | 默认演示启动；当前已生成 `fastlio_mapping` |
| `Swarm/` | 只有空说明文件 | 无完整集群算法实现 |
| `Track/` | `pix_tracker` | 带 `CATKIN_IGNORE` |
| `Utils/` | 自定义消息、相机/雷达驱动、坐标工具等 | 混合状态；Livox 驱动、`cv_bridge` 等有构建产物，很多工具被忽略 |

这意味着 README 中的“控制、规划、集群、探索”等模块名称不能直接等同于当前已经形成闭环的功能。对本阶段而言，可直接工作的主线是 `offboard + PX4 SITL + Gazebo`。

### 1.3 `simulation/` 结构

```text
simulation/
├── astra_gazebo_models/          # 数百个 Gazebo 模型资源
├── astra_gazebo_worlds/          # example/forest/cangku/suv/test/动态避障/CRAIC 场景
├── px4_sim_files/
│   ├── px4_iris_params/          # PX4 airframe 启动参数 1046~1054
│   ├── px4_iris_sdf/             # iris_mid360、无 GPS、多机和下视相机机型
│   └── px4_launch/               # 默认 astra_example.launch
└── sim_workspace/src/
    ├── env_map/                  # Gazebo world 基础启动包
    ├── sensors/                  # Mid360、通用雷达、RealSense Gazebo 插件
    ├── dynamic_obstacle_controller/ # 动态障碍物轨迹控制
    ├── craic_sim/                # CRAIC 2026 场景生成与启动
    └── ugv_gazebo_sim/           # Hunter/Hunter SE 地面车仿真
```

与无人机运动最相关的 world：

| 路径 | 用途 |
|---|---|
| `simulation/astra_gazebo_worlds/example.world` | 默认演示场景 |
| `simulation/astra_gazebo_worlds/forest.world` | 森林场景 |
| `simulation/astra_gazebo_worlds/dynamic_avoidance.world` | 静态树木/房屋和 5 个可移动障碍物 |
| `simulation/astra_gazebo_worlds/craic_2026.world` | 固定 CRAIC 赛场 |
| `simulation/astra_gazebo_worlds/generated/craic_2026_runtime.world` | 运行时生成的 CRAIC 场景 |

## 2. 默认仿真的真实启动链

### 2.1 一键脚本

文件：`scripts/run_sh/pc_example.sh`

| 行号 | 行为 | 结果 |
|---:|---|---|
| 2-13 | 重建并划分 `pc_example` tmux 会话 | 创建 5 个运行窗格 |
| 14-15 | 启动 `roscore` | ROS master |
| 17-19 | 延时 3 秒后运行 `roslaunch px4 astra_example.launch` | PX4 SITL、Gazebo、MAVROS |
| 21-24 | 延时 6 秒，执行 `astra` 后启动 FAST-LIO | 加载主 ROS 工作空间并建图 |
| 26-29 | 延时 10 秒，执行 `astra` 后启动 Offboard | 自动解锁和轨迹控制 |
| 31-32 | 执行 `qgc` | QGroundControl |
| 34 | 附着 tmux | 显示所有进程 |

`astra` 和 `qgc` 是安装阶段写入用户 shell 的 alias，不是仓库内可执行文件。`astra` 的作用是 source `AstraDrone_ros1_ws/devel/setup.bash`。

**建议**：开发阶段不要每次都从一键脚本开始。先分别启动 PX4/Gazebo、FAST-LIO、控制节点，更容易判断问题属于仿真、定位还是控制。

### 2.2 PX4/Gazebo/MAVROS 启动文件

仓库源文件：`simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch`

| 行号 | 参数/节点 | 当前值或作用 |
|---:|---|---|
| 6-11 | `x y z R P Y` | 初始位置 `(0,0,0.06)`、初始姿态全 0 |
| 14 | `vehicle` | `iris_mid360` |
| 15 | `world` | 意图指向 `simulation/astra_gazebo_worlds/example.world`，但当前表达式缺少路径分隔符，见下文 |
| 16 | `sdf` | 从 PX4 Gazebo 模型目录解析机型 SDF |
| 19-23 | Gazebo 参数 | GUI 开启、非暂停、默认不 respawn |
| 25 | `fcu_url` | `udp://:14540@localhost:14557` |
| 30-46 | `posix_sitl.launch` | 启动 PX4 SITL、Gazebo 和模型 |
| 48-53 | `mavros/px4.launch` | 启动 MAVROS |

可以不修改文件，直接切换 world：

```bash
roslaunch px4 astra_example.launch \
  world:=$HOME/AstraDroneOpen/simulation/astra_gazebo_worlds/forest.world
```

**当前阻断**：第 15 行实际写成 `$(find env_map)../../../astra_gazebo_worlds/example.world`。`rospack find env_map` 在当前环境返回的路径不带尾 `/`，所以拼接结果形如 `.../env_map../../../...`，不是有效的父目录路径。应改成：

```xml
<arg name="world"
     default="$(find env_map)/../../../astra_gazebo_worlds/example.world"/>
```

修正仓库源文件后还要按第 2.3 节同步外部 PX4 副本。开发时传入上述绝对 `world:=...` 参数可立即绕过该问题。

### 2.3 仓库资源与外部 PX4 副本

当前环境中：

- `rospack find px4` 指向 `~/PX4-Autopilot`。
- `astra_example.launch` 实际运行副本位于 `~/PX4-Autopilot/launch/astra_launch/astra_example.launch`。
- `iris_mid360.sdf` 实际运行副本位于 PX4 Gazebo 模型目录。
- `1046_gazebo-classic_iris_mid360` 实际运行副本位于 PX4 的 `build/px4_sitl_default/etc/init.d-posix/airframes/`。
- 审阅时仓库源文件与这三份外部副本内容一致，但它们是普通文件，不是符号链接。

因此，修改 `simulation/px4_sim_files/**` 后必须先确认外部副本已同步。可用：

```bash
cmp simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch \
    "$HOME/PX4-Autopilot/launch/astra_launch/astra_example.launch"
```

若 `cmp` 返回非 0，运行项目的仿真构建/安装流程使其重新复制，或者在明确目标路径后手动同步。只改仓库源文件但不同步，不会改变 `roslaunch px4 astra_example.launch` 的结果。

### 2.4 默认话题闭环

```text
Gazebo iris_mid360
  ├─ /livox/lidar ─┐
  └─ /livox/imu ───┴─> FAST-LIO ─> /Odometry, /cloud_registered, TF

PX4 SITL <─MAVLink─> MAVROS
  ├─ /mavros/state --------------------> autoarming_control::state_cb
  ├─ /mavros/local_position/pose ------> autoarming_control::pose_cb
  ├─ /mavros/set_mode <---------------- autoarming_control
  ├─ /mavros/cmd/arming <-------------- autoarming_control
  └─ /mavros/setpoint_position/local <- autoarming_control
```

**代码事实**：`autoarming_control.cpp` 不订阅 `/Odometry` 或 `/cloud_registered`。`autoarming_control.launch` 第 7-8 行仅发布 `map -> camera_init` 静态 TF，不会把 FAST-LIO 里程计送进 PX4，也不会让默认轨迹自动避障。

**推断**：无 GPS 模型 SDF 的 MAVLink 插件第 469-470 行配置 `send_vision_estimation=0`、`send_odometry=1`，结合 PX4 airframe 的外部视觉 EKF 参数，默认 PX4 局部位姿主要由 Gazebo 模拟里程计链提供，而不是由另行启动的 FAST-LIO 提供。需要运行时检查 PX4 estimator 状态才能最终确认。

## 3. 默认 Offboard 控制器逐段分析

核心文件：`AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src/autoarming_control.cpp`

### 3.1 类/函数/代码段索引

该文件不是类封装，而是全局状态加 `main()` 状态机。

| 行号 | 函数/符号 | 作用 |
|---:|---|---|
| 15-18 | `current_state`、`current_pose`、`hight`、`initial_height` | 保存 MAVROS 状态、当前位置和高度 |
| 21-26 | `enum class FlightPhase` | `TAKEOFF -> TRACKING -> LANDING -> COMPLETED` |
| 28-30 | `state_cb` | 接收 `/mavros/state` |
| 32-34 | `pose_cb` | 接收 `/mavros/local_position/pose` |
| 36-41 | `calculate_distance` | 三维目标误差 |
| 43-56 | `get_square_position` | 正方形按周长归一化参数 `t∈[0,1)` 取点 |
| 58-63 | `get_circle_position` | 圆形按角度 `2πt` 取点 |
| 65-80 | `Lock` | 通过 MAVLink command 400 请求上锁 |
| 82-273 | `main` | 参数读取、连接、预热、模式切换、解锁、飞行和降落 |
| 93-97 | 参数读取 | `flight_mode`、`hight`、圈数、边长、半径 |
| 99-104 | ROS 接口 | MAVROS 订阅、发布和 service client |
| 106 | `ros::Rate(20.0)` | 主循环 20 Hz |
| 108-115 | FCU 等待和初始高度 | 只等待 state 连接后记录当前 z |
| 117-125 | 预热 | 100 个当前位置 setpoint，即约 5 秒 |
| 127-163 | OFFBOARD/解锁 | 每隔 5 秒尝试切模式或解锁 |
| 192-204 | `TAKEOFF` | 飞到 `(0,0,hight)` |
| 205-252 | `TRACKING` | 生成圆/方形位置 setpoint |
| 253-266 | `LANDING` | 飞回 `(0,0,initial_height)` 并上锁 |

### 3.2 launch 参数：哪些生效，哪些不生效

文件：`AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/launch/autoarming_control.launch`

| launch 行 | 参数 | 当前值 | 是否生效 | 修改效果 |
|---:|---|---:|---|---|
| 11 | `/flight_mode` | `circle` | 是 | `square` 走方形；任何非 `square` 字符串都会落入圆形分支 |
| 14 | `~target_laps` | `2` | 是 | 改变轨迹圈数 |
| 15 | `~hight` | `3.0` | 是 | 同时改变起飞、巡航目标高度；保留当前拼写 |
| 16 | `~takeoff_height` | `1.0` | **否** | 源码未读取，修改无效果 |
| 17 | `~side_length` | `8.0` | 方形模式生效 | 改变正方形边长 |
| 18 | `~speed` | `1.5` | **否** | 源码未读取，修改无效果 |
| 未设置 | `~radius` | 源码默认 `2.0` | 圆形模式生效 | 应在 node 内新增 `<param name="radius" value="..."/>` |
| 4 | launch arg `rviz` | `true` | 是 | `rviz:=false` 可不启动 Offboard RViz |

注意命名空间：`flight_mode` 当前在 `<node>` 外，是全局参数，源码第 93 行用普通 `nh` 读取；其他参数在 `<node>` 内，源码第 94-97 行用私有 `nh_private` 读取。若把 `flight_mode` 移进 node，必须同时把第 93 行改为 `nh_private.param(...)`，否则会读不到。

最小参数实验示例：

```xml
<!-- 保持 flight_mode 在 node 外，或同步修改源码为私有参数 -->
<param name="flight_mode" value="circle" />
<node pkg="offboard" type="autoarming_control" name="autoarming_control" output="screen">
  <param name="target_laps" value="3"/>
  <param name="hight" value="2.0"/>
  <param name="radius" value="4.0"/>
</node>
```

效果：无人机目标高度变为 2 m，以原点为圆心、4 m 为半径飞 3 圈后返回原点降落。这里仍没有可靠的速度控制。

### 3.3 当前轨迹推进算法为什么不是速度控制

关键代码：`autoarming_control.cpp` 第 219-233 行。

```cpp
double d = calculate_distance(current_pose, target_pose);
if (d < 1.0) {
    double advance = 1.0 - d;
    double delta_t = advance / trajectory_length;
    t_target += delta_t;
    ...
}
```

当前逻辑的含义是：当无人机离当前目标小于 1 m 时，把目标沿轨迹向前推 `1-d` 米左右。它没有使用真实时间 `dt`，因此参考速度受循环频率、跟踪误差和 PX4 响应共同影响。把循环从 20 Hz 改成 40 Hz 通常会加快目标推进，但这不是可复现的“m/s”速度控制。

**推断**：在误差很小时，每个循环最多前推约 1 m，20 Hz 下理论参考点推进可非常激进；实际飞机会因误差变大而停止推进，形成不均匀的追赶行为。方形拐角尤其可能出现切角或停顿。

### 3.4 当前控制器的实验风险点

| 位置 | 问题 | 对仿真实验的影响 | 建议 |
|---|---|---|---|
| 第 93 行 | 非 `square` 一律当圆处理 | 拼写错误不会报警 | 显式校验 `circle/square/custom` |
| 第 115 行 | 连接后立即记录 `current_pose.z` | pose 尚未到达时可能记录默认 0 | 增加 `have_pose` 标志并等待首帧 |
| 第 143、188-190 行 | 默认构造的 orientation 未赋合法四元数 | 发布的四元数可能为 `(0,0,0,0)` | 至少设置 `orientation.w=1.0` |
| 第 192-195 行 | 起飞目标固定 `(0,0,hight)` | 非原点出生时会先横移 | 记录初始 x/y 并相对起飞 |
| 第 207-210 行 | 圆/方形中心固定在世界原点 | 更换出生点后轨迹仍围绕原点 | 增加 `center_x/center_y` |
| 第 219-233 行 | 轨迹推进不基于时间 | `speed` 无效、速度不稳定 | 改成弧长/时间参数化 |
| 第 220 行 | 跟踪门限硬编码 1 m | 小场景偏大，快速轨迹可能停住 | 参数化为 `max_tracking_error` |
| 第 253-265 行 | 降落直线回 `(0,0)` | 可能穿过障碍物 | 增加返航路径/规划 |
| 第 260-263 行 | 无论 `Lock()` 是否成功都结束状态机 | 可能尚未真正上锁 | 让 `Lock` 返回 bool，成功后再结束 |
| 全文件 | 无碰撞检测、无 geofence、无急停输入 | 轨迹会直接穿越障碍物 | 先在空场验证，再接规划器 |

## 4. 如何精确修改无人机运动方式

### 4.1 只改高度、圈数、尺寸和场景

无需改 C++ 的修改点：

| 目标 | 文件与行号 | 修改内容 | 结果 |
|---|---|---|---|
| 巡航高度 | `offboard/launch/autoarming_control.launch:15` | `hight` | 起飞和轨迹 z 同时改变 |
| 圈数 | 同文件 `:14` | `target_laps` | 完成指定圈数后降落 |
| 方形边长 | 同文件 `:17` | `side_length` | `flight_mode=square` 时改变边长 |
| 圆半径 | 同文件 node 内新增参数 | `radius` | 覆盖源码第 97 行默认 2 m |
| 圆/方形选择 | 同文件 `:11` | `circle` 或 `square` | 切换轨迹函数 |
| 初始位置 | `simulation/.../astra_example.launch:6-11` | `x y z R P Y` | 改模型出生位姿 |
| world | 同文件 `:15` 或 roslaunch arg | world 路径 | 切换环境和障碍物 |

`hight` 是当前代码真实参数名。只把 launch 改成 `height` 会导致源码继续使用默认 3 m；若要纠正拼写，必须同时改源码第 17、94、143、195、202、215 行附近的变量/读取逻辑。

### 4.2 让 `speed` 真正生效

修改文件：`offboard/src/autoarming_control.cpp`

建议涉及的精确位置：

1. 第 12-13 行 include 区新增 `#include <algorithm>`，供 `std::max` 使用。
2. 第 88-92 行变量区新增 `double speed; double max_tracking_error;`。
3. 第 94-97 行参数读取区新增：

   ```cpp
   nh_private.param("speed", speed, 1.5);
   nh_private.param("max_tracking_error", max_tracking_error, 0.5);
   ```

4. 第 140-142 行状态变量区新增 `ros::Time last_track_time;`。
5. 第 197-201 行进入 `TRACKING` 时设置：

   ```cpp
   last_track_time = ros::Time::now();
   ```

6. 用以下思路替换第 219-233 行基于 `1-d` 的推进逻辑：

   ```cpp
   const ros::Time now = ros::Time::now();
   const double dt = std::max(0.0, (now - last_track_time).toSec());
   last_track_time = now;

   const double d = calculate_distance(current_pose, target_pose);
   if (d <= max_tracking_error) {
       t_target += speed * dt / trajectory_length;
       while (t_target >= 1.0) {
           t_target -= 1.0;
           ++completed_laps;
       }
       if (completed_laps >= target_laps) {
           flight_phase = FlightPhase::LANDING;
       }
   }
   ```

7. launch 第 18 行的 `speed` 保持为私有参数，并在 node 内增加 `max_tracking_error`。

效果：圆和正方形的参考点按约 `speed m/s` 沿弧长推进；当飞机落后超过门限时暂停参考点，避免目标持续跑远。`speed=0.5` 更平缓，`speed=2.0` 更快。位置 setpoint 方式下，这仍是“参考轨迹速度”，实际机体速度要用 `/mavros/local_position/odom` 的 twist 验证。

建议再加保护：

```cpp
speed = std::max(0.05, speed);
max_tracking_error = std::max(0.1, max_tracking_error);
```

### 4.3 改圆/方形的中心、起点和方向

当前 `get_square_position` 位于第 43-56 行，`get_circle_position` 位于第 58-63 行。

圆形建议改为支持中心和方向：

```cpp
std::pair<double, double> get_circle_position(
    double t, double radius, double center_x, double center_y, bool clockwise) {
    const double sign = clockwise ? -1.0 : 1.0;
    const double angle = sign * t * 2.0 * M_PI;
    return {center_x + radius * std::cos(angle),
            center_y + radius * std::sin(angle)};
}
```

并在第 88-97 行附近读取 `center_x`、`center_y`、`clockwise`，在第 210、241 行调用时传入。效果是轨迹不再强制绕世界原点，可顺/逆时针飞行。

正方形当前起点是 `(-side/2,-side/2)`。无人机起飞到 `(0,0,hight)` 后会先追向左下角。若希望从当前点平滑进入轨迹，可：

- 把方形中心平移到初始位置；
- 增加 `APPROACH_START` 阶段，先到首点再开始计时；
- 或把轨迹函数的 `t=0` 设计成当前起飞点。

### 4.4 增加“8 字、椭圆、螺旋”等轨迹

修改位置：在 `get_circle_position` 后、第 65 行前新增函数，并在 `TRACKING` 的第 207-211、236-244 行增加分支。

8 字轨迹示例：

```cpp
std::pair<double, double> get_figure8_position(
    double t, double a, double b, double cx, double cy) {
    const double u = 2.0 * M_PI * t;
    return {cx + a * std::sin(u),
            cy + b * std::sin(u) * std::cos(u)};
}
```

效果：x 方向幅值为 `a`，y 方向幅值约为 `b/2`，在中心交叉。若要求 `speed` 代表严格恒定弧长速度，不能直接假设 `t` 与弧长线性；应预采样轨迹、累计弧长，再按弧长查表。

螺旋上升不能只返回 `(x,y)`，还需在第 215 行把固定 `z=hight` 改为随总相位变化，例如：

```cpp
const double total_phase = completed_laps + t_target;
target_pose.pose.position.z = base_height + climb_per_lap * total_phase;
```

并设置 `max_height` 限制。效果是每圈上升 `climb_per_lap` 米。

### 4.5 控制航向角 yaw

当前圆/方形轨迹没有设置目标姿态，`target_pose` 的四元数默认无效。至少在第 143 行后设置：

```cpp
target_pose.pose.orientation.w = 1.0;
```

另外在第 188-190 行创建每周期 `pose` 后设置 `pose.pose.orientation.w = 1.0`，否则 TAKEOFF 和 LANDING 使用的临时消息仍是零四元数。

若希望机头始终朝速度方向，可在生成当前点和下一个小步点后计算：

```cpp
const double yaw = std::atan2(next_y - current_y, next_x - current_x);
target_pose.pose.orientation.z = std::sin(yaw * 0.5);
target_pose.pose.orientation.w = std::cos(yaw * 0.5);
```

同时把 `x/y` 四元数分量置 0。效果是无人机沿轨迹切线转头。为避免过零点或方形拐角瞬间跳变，应对 yaw 做角度展开和最大角速度限制。

### 4.6 动作序列：起飞—悬停—前进—转向—降落

最适合修改的位置是第 21-26 行的 `FlightPhase` 和第 192-266 行状态机。建议扩展为：

```text
TAKEOFF -> HOVER -> MOVE_TO_WAYPOINT -> TURN -> TRACKING -> RETURN_HOME -> LANDING
```

每个阶段需要：目标、进入时间、完成条件和超时。示例逻辑可放在源码中，但本次没有实际改动：

```cpp
case HOVER:
  publish(hold_pose);
  if ((ros::Time::now() - phase_start).toSec() >= hover_seconds)
    enter(MOVE_TO_WAYPOINT);
  break;
```

不要只用固定 `sleep` 控制飞行动作；回调会停、setpoint 也会中断，PX4 可能退出 Offboard。状态机主循环必须持续发布 setpoint。

### 4.7 用航点控制，而不是写解析轨迹

项目已有 `position_control`，但当前实现不建议直接用于实验：

| 文件/行号 | 当前问题 |
|---|---|
| `offboard/src/position_control_lib.cpp:6-10` | 构造函数直接进入无限循环 |
| 同文件 `:15-31` | 循环没有 `ros::Rate::sleep()`，会满速发布并占用 CPU |
| 同文件 `:34-39` | `ReadParams()` 定义了但从未调用，launch 的初始目标不会读取 |
| 同文件 `:42-51` | 只复制位置，不更新 header 和合法 orientation |
| `offboard/src/position_control.cpp:5-14` | 没有 FCU 等待、OFFBOARD 切换或解锁逻辑 |
| `offboard/launch/position_control.launch:4-6` | 初始目标参数存在，但因上述原因当前无效 |

若继续使用它，至少应：

1. 构造函数只初始化 pub/sub、调用 `ReadParams()`，不要阻塞。
2. 使用 timer 或带 `ros::Rate(20)` 的主循环。
3. 每次发布更新 `header.stamp`、`frame_id` 和 orientation。
4. 复用 `autoarming_control` 的 FCU、OFFBOARD、arming 状态机。
5. 增加航点数组、当前索引、到达门限和每段超时。

**当前阻断**：`position_control` 与 `autoarming_control` 同时运行会争抢 `/mavros/setpoint_position/local`，不能用前者发航点、后者只负责解锁而不做隔离。应合并为单一控制节点，或让一个节点独占 setpoint 发布。

## 5. EGO-Planner：能改什么，以及为何默认不控制 PX4

### 5.1 当前 EGO 独立仿真链

```text
RViz goal / 预设航点
 -> waypoint_generator
 -> EGOReplanFSM
 -> /planning/bspline
 -> traj_server
 -> /planning/pos_cmd (quadrotor_msgs/PositionCommand)
 -> SO3ControlNodelet
 -> so3_cmd
 -> so3_quadrotor_simulator
 -> /visual_slam/odom
```

证据：

- `plan_manage/launch/simulator.xml:76-86` 启动自带 `so3_quadrotor_simulator`。
- 同文件 `:88-104` 启动 SO3 控制器并订阅 `/planning/pos_cmd`。
- `traj_server.cpp:238-242` 订阅 B-spline、发布 `PositionCommand`，定时器为 0.01 秒即 100 Hz。
- `simulator.xml:82` 把自带动力学模拟器里程计发布为 `/visual_slam/odom`。
- 这条链中没有 MAVROS publisher/service。

所有 `planner/*`、`uav_simulator/*` 相关包当前都有 `CATKIN_IGNORE`。因此默认主工作空间重新编译时不会编译它们。

### 5.2 EGO 的精确可修改点

入口：`AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/launch/run_in_sim.launch`

| 行号 | 参数 | 效果 |
|---:|---|---|
| 3-5 | `map_size_x/y/z` | 规划占据栅格范围 |
| 8 | `odom_topic=/visual_slam/odom` | 规划器和模拟器的里程计输入 |
| 21-26 | 相机 pose/depth/cloud 话题 | 障碍物感知输入 |
| 35-36 | `max_vel=2.0`、`max_acc=3.0` | 规划轨迹的速度/加速度上限 |
| 39 | `planning_horizon=7.5` | 局部规划视野 |
| 43 | `flight_type=1` | 1 为 RViz 单目标；2 为预设航点 |
| 47-67 | 5 个预设航点 | `flight_type=2` 时生效 |
| 72-77 | `traj_server` | B-spline 转连续控制指令 |
| 79-84 | `waypoint_generator` | RViz `/move_base_simple/goal` 转规划目标 |
| 87-96 | `simulator.xml` | 启动独立地图、SO3 控制器和动力学模拟器 |

高级参数：`plan_manage/launch/advanced_param.xml`

| 行号 | 参数组 | 作用 |
|---:|---|---|
| 48-53 | `fsm/*` | 重规划距离、时间和应急阈值 |
| 55-70 | `fsm/waypoint*` | 把预设航点传给 FSM |
| 72-81 | `grid_map` 尺寸/分辨率/膨胀 | 障碍物地图和安全边界 |
| 88-102 | 深度过滤与概率参数 | 深度图融合质量 |
| 104-105 | 虚拟天花板 2.5 m | 限制可规划高度；高于此值的目标不合适 |
| 111-116 | `manager/max_vel/max_acc/max_jerk` | 轨迹动力学约束 |
| 119-125 | 优化权重 | 平滑、碰撞、可行性、贴合代价 |
| 127-129 | B-spline 限制 | 最终速度/加速度约束 |

非常重要的高度问题：`ego_replan_fsm.cpp:109-120` 的手动目标回调在第 119 行把目标 z **硬编码为 1.0**：

```cpp
end_pt_ << msg->poses[0].pose.position.x,
           msg->poses[0].pose.position.y,
           1.0;
```

若要让 RViz 目标高度生效，应把 `1.0` 改为 `msg->poses[0].pose.position.z`，并确保它低于 `advanced_param.xml:104` 的虚拟天花板，或同步提高天花板和地图 z 范围。

预设航点由 `ego_replan_fsm.cpp:22-28` 读取，`planGlobalTrajbyGivenWps()` 位于第 57-107 行。修改 `run_in_sim.launch:47-67` 可改变航点顺序、高度和闭合路线。

### 5.3 waypoint generator 中的内置轨迹

文件：`uav_simulator/Utils/waypoint_generator/src/sample_waypoints.h`

| 行号 | 函数 | 可改参数 |
|---:|---|---|
| 8-59 | `point()` | 第 15-16 行高度 `h`、缩放 `scale`，以及各离散点 |
| 62-128 | `circle()` | 第 64-65 行 `h/scale`；实际是离散航点组合，不是解析圆 |
| 131-212 | `eight()` | 第 134-137 行偏移、半径 `r`、高度 `h` |

但 `run_in_sim.launch:83` 当前设置 `waypoint_type=manual-lonely-waypoint`，因此修改这些函数不会自动生效。`waypoint_generator.cpp:158-179` 显示，只有把类型改为 `circle`、`eight` 或 `points` 并触发 goal，才会调用相应函数。

另有命名不一致：goal 回调第 166 行识别 `points`，trigger 回调第 235 行识别 `point`。实验时要根据实际触发入口使用对应字符串，或统一源码命名。

### 5.4 把 EGO 接到 PX4/Gazebo 所需的工作

这是可行方向，但当前仓库没有完整桥接。至少需要：

1. **启用包**：移除相关包的 `CATKIN_IGNORE` 并重新编译。最小集合包括 `quadrotor_msgs`、`pose_utils/uav_utils` 等依赖、`bspline_opt`、`path_searching`、`plan_env`、`traj_utils`、`plan_manage`、`waypoint_generator`。实际依赖应以 catkin 报错继续补齐。
2. **替换里程计**：把 `run_in_sim.launch:8` 从 `/visual_slam/odom` 改为 `/mavros/local_position/odom`，或经过坐标对齐后的 FAST-LIO `/Odometry`。
3. **替换感知输入**：把第 26 行 cloud 改为适合规划的点云，例如 `/cloud_registered`，并确认 frame、时间戳和局部范围一致。
4. **停用内部模拟器**：不要再 include `run_in_sim.launch:87-96` 的 `simulator.xml`，否则会同时存在另一套无人机动力学和 `/visual_slam/odom`。
5. **实现桥接器**：订阅 `/planning/pos_cmd` 的 `quadrotor_msgs/PositionCommand`，转换成 MAVROS 的位置/速度/加速度/yaw setpoint。
6. **实现 OFFBOARD/arming 状态机**：桥接器持续预热 setpoint、切换 OFFBOARD、解锁并监控 failsafe。
7. **保证唯一 publisher**：接入 EGO 时停掉 `autoarming_control` 的 setpoint 发布。

桥接消息建议优先使用 `mavros_msgs/PositionTarget`，可保留 `PositionCommand` 中的 position、velocity、acceleration、yaw、yaw_dot。只转 `PoseStamped` 会丢失速度和加速度前馈。

**推断**：FAST-LIO 使用 `camera_init/body`，MAVROS 局部坐标通常使用另一套 ENU/local frame。直接 remap 而不做初始原点和姿态对齐，可能导致轨迹旋转、平移或高度偏置。接入前应在静止和单轴移动实验中比较两个 odometry。

## 6. `rc_obstacle_avoidance`：可借鉴，但当前不能直接闭环

文件：`AstraDrone_ros1_ws/src/Control/rc_obstacle_avoidance/src/rc_obstacle_avoidance_node.cpp`

该节点的设计是：RC 输入生成目标，Fast-Planner 生成位置/速度轨迹，节点再转发到 MAVROS。关键位置：

| 行号 | 类/函数 | 作用 |
|---:|---|---|
| 14-329 | `RcObstacleAvoidance` | 完整节点类 |
| 28-58 | 构造参数读取 | 全部通过私有 NodeHandle `~` 读取 |
| 60-78 | pub/sub/timer | RC、pose、规划输出与 MAVROS 输出 |
| 139-150 | `normalizeChannel` | PWM 映射到 `[-1,1]` |
| 152-173 | `poseCallback` | 初始 1 m 起飞目标 |
| 191-250 | `rcCallback` | 机体系 RC 偏移转世界系目标 |
| 252-282 | `publishTarget` | 发布规划目标和超时悬停 |
| 284-313 | `forwardPlannerCommand` | 优先位置、其次速度地转发到 MAVROS |

当前有三个阻断：

1. 包根目录存在 `CATKIN_IGNORE`。
2. 仓库内没有名为 Fast-Planner 的实现；只有 EGO-Planner，且话题和消息接口不同。
3. `launch/rc_obstacle_avoidance.launch:3` 在 node 外加载 YAML，参数进入全局命名空间；源码第 28-58 行却读取私有参数。因此当前 YAML 的值不会覆盖源码默认值。

第 3 点的正确 launch 结构应是：

```xml
<node pkg="rc_obstacle_avoidance"
      type="rc_obstacle_avoidance_node"
      name="rc_obstacle_avoidance"
      output="screen">
  <rosparam command="load"
            file="$(find rc_obstacle_avoidance)/config/rc_obstacle_avoidance.yaml"/>
</node>
```

当前真正会使用的源码默认值在第 28-58 行，例如 `max_xy_step=1.0`、`max_z_step=0.6`、`publish_rate=20`；配置文件第 17-21 行写的 `2.0/2.0/30.0` 目前不会生效。

该节点也不负责切 OFFBOARD 或解锁。它适合作为未来“规划输出转 MAVROS”的参考，但不能直接当作当前自主避障方案。

## 7. 仿真场景、动态障碍物与传感器

### 7.1 切换静态场景

推荐通过 launch 参数切换，不先修改默认文件：

```bash
roslaunch px4 astra_example.launch \
  world:=$HOME/AstraDroneOpen/simulation/astra_gazebo_worlds/dynamic_avoidance.world
```

同类缺少 `/` 的表达式还存在于：

- `simulation/sim_workspace/src/env_map/launch/map_test.launch:5`
- `simulation/sim_workspace/src/dynamic_obstacle_controller/launch/astra_dynamic_avoidance_static.launch:11`
- `simulation/sim_workspace/src/craic_sim/launch/astra_craic_2026.launch:12`
- `simulation/sim_workspace/src/craic_sim/launch/astra_craic_2026_runtime.launch:14`

分别把 `$(find env_map)../../../...`、`$(find craic_sim)../../../...` 改为 `$(find env_map)/../../../...`、`$(find craic_sim)/../../../...`，或在命令行传入绝对 world 路径。

修改 world 中 `<model>` 的 `<pose>x y z roll pitch yaw</pose>` 会改变障碍物位置；修改 `<static>` 决定其是否受物理作用。做轨迹实验时应同时记录无人机轨迹和障碍物位置，不能只看 Gazebo 画面。

### 7.2 动态障碍物

启动文件：

- 静态障碍场景：`simulation/sim_workspace/src/dynamic_obstacle_controller/launch/astra_dynamic_avoidance_static.launch`
- 动态障碍场景：`.../astra_dynamic_avoidance_moving.launch`
- 运动参数：`.../config/obstacle_params.yaml`
- 运动实现：`.../src/obstacle_controller.py`

`obstacle_params.yaml` 的可修改点：

| 行号 | 参数 | 效果 |
|---:|---|---|
| 6 | `update_rate=100` | Gazebo model state 更新频率 |
| 10-15 | `obstacle_cylinder_1` | x 轴正弦往返；幅值 3 m、最大速度约 0.8 m/s |
| 18-23 | `obstacle_cylinder_2` | y 轴往返；幅值 4 m |
| 26-31 | `obstacle_cylinder_3` | 对角线往返 |
| 34-38 | `obstacle_box_1` | 半径 3 m 的圆周运动 |
| 41-44 | `obstacle_box_2` | 航点巡逻 |

实现对应关系：

- `ObstacleMover.compute_linear()`：第 26-36 行，`amplitude*sin(speed*t/amplitude)`，`speed` 是最大线速度。
- `compute_circle()`：第 38-47 行，角速度为 `speed/radius`，切向速度为 `speed`。
- `compute_waypoint()`：第 49-79 行，按路径长度以指定速度循环。
- `set_model_pose()`：第 81-92 行，通过 `/gazebo/set_model_state` 设置位姿。
- `run()`：第 94-114 行，按 `update_rate` 更新全部障碍物。

YAML 中的 `name` 必须与 `dynamic_avoidance.world:345-374` 的模型名称完全一致，否则 service 调用不会移动目标模型。

**代码事实**：动态障碍控制器只移动 Gazebo 模型，不提供无人机避障。默认 `autoarming_control` 会继续沿圆/方形飞行，可能直接碰撞。必须另接占据地图和规划器才能自主绕行。

### 7.3 CRAIC 场景

- 固定场景启动：`craic_sim/launch/astra_craic_2026.launch:3-47`
- 运行时场景启动：`craic_sim/launch/astra_craic_2026_runtime.launch:3-64`
- 场景生成器：`craic_sim/scripts/generate_craic_2026_world.py`

生成器第 21-41 行集中定义场地、障碍物、起飞区和圆环尺寸/位置。第 48-49 行 `rule_to_world_xy()` 定义竞赛规则坐标到 Gazebo 世界坐标的换算。修改竞赛轨迹前，应先统一使用规则坐标还是 world 坐标，不要在控制器中重复平移/反转 y。

### 7.4 Mid360 与 FAST-LIO

机体挂载：`simulation/px4_sim_files/px4_iris_sdf/iris_mid360/iris_mid360.sdf`

| 行号 | 内容 | 修改效果 |
|---:|---|---|
| 3-5 | include `iris_without_GPS` | 基础机体 |
| 7-21 | Mid360 include、pose、joint | 改第 9 行可移动雷达安装位置 |
| 39-54 | D435i include/joint | 第 41 行改变相机外参 |
| 57-71 | FPV camera include/joint | 第 59 行改变 FPV 相机外参 |

Mid360 模型：`simulation/astra_gazebo_models/mid360/mid360.sdf`

| 行号 | 参数 | 当前值/效果 |
|---:|---|---|
| 35-39 | LiDAR sensor/update | 10 Hz |
| 45-56 | 水平/垂直采样角 | 当前均为 `[-π,π]` |
| 58-67 | range/noise | 0.2~40 m、零高斯噪声 |
| 70-75 | samples/downsample/topic | 20000 点，`livox/lidar` |
| 80-112 | IMU | `/livox/imu`、外参和噪声 |

FAST-LIO：

| 文件/行号 | 参数/代码 | 作用 |
|---|---|---|
| `FAST_LIO/launch/mapping_mid360.launch:6-15` | 加载 YAML、滤波分辨率、启动 mapping | 默认 SLAM 入口 |
| `FAST_LIO/config/mid360.yaml:2-3` | `/livox/lidar`、`/livox/imu` | 与模型话题匹配 |
| 同 YAML `:18-24` | FOV、探测距离、外参 | 改感知范围/雷达-IMU 标定 |
| 同 YAML `:26-34` | 点云/path/PCD 发布 | 改输出和保存 |
| `laserMapping.cpp:592-620` | `/Odometry` frame/TF | `camera_init -> body` |
| `laserMapping.cpp:762-794` | 参数读取 | YAML 到运行变量 |
| `laserMapping.cpp:847-860` | 订阅和发布 | LiDAR/IMU 输入，点云/里程计输出 |

雷达模型第 81 行 IMU 平移外参与 FAST-LIO YAML 第 21 行一致。修改挂载或 IMU 外参时必须同步，否则建图会漂移或扭曲。

## 8. PX4 机型与控制参数边界

默认 airframe：`simulation/px4_sim_files/px4_iris_params/1046_gazebo-classic_iris_mid360`

| 行号 | 参数 | 含义 |
|---:|---|---|
| 8 | source `10015_gazebo-classic_iris` | 继承 Iris 基础参数 |
| 11 | `EKF2_EV_DELAY=5` | 外部视觉延迟参数 |
| 12 | `EKF2_EV_CTRL=15` | 外部视觉融合配置 |
| 13 | `EKF2_HGT_REF=3` | 高度参考配置 |
| 14 | `EKF2_GPS_CTRL=0` | 禁用 GPS 融合 |
| 15 | `SYS_HAS_MAG=0` | 声明无磁罗盘 |
| 16 | `COM_RC_IN_MODE=1` | RC 输入模式 |

机体动力学主要在 `iris_without_GPS.sdf`：

- 第 8-15 行：机体质量和惯量。
- 第 348-420 行：多旋翼基座和四个电机模型；电机时间常数、最大转速、推力/力矩常数会改变响应。
- 第 446-470 行：气压计和 MAVLink interface。

除非实验目标就是系统辨识或底层动力学，不建议先改质量、电机常数或 PX4 内环增益。先用上层轨迹参数验证 setpoint 链，再逐步调 PX4 参数，避免把规划、Offboard 和动力学问题混在一起。

## 9. 多机与集群的当前真实状态

项目有多机资源，但没有完整可直接运行的集群闭环：

- `px4_iris_params/1048~1053` 和 `px4_iris_sdf/iris_mid360_0~2`、`iris_without_GPS_0~2` 提供三机变体。
- `offboard/launch/autoarming_Mult.launch:9-34` 尝试为 `/uav0`、`/uav1`、`/uav2` 启动三个控制器。
- `Swarm/` 当前没有控制源码。

`autoarming_Mult.launch` 存在明显问题：第 9、18、27 行三个 node 都叫 `autoarming_control`，同一 namespace 下节点名重复，roslaunch 不能可靠启动三者。应分别命名为 `autoarming_control_uav0/uav1/uav2` 或放入各自 namespace。

该 launch 也只启动控制节点，不启动三套 PX4 SITL/MAVROS；README 提到的 `multi_uav_mavros_sitl.launch` 不在本仓库的自定义 launch 中。要做多机实验，还需明确每架机的：

- PX4 instance、MAV_SYS_ID；
- MAVLink UDP/TCP 端口；
- Gazebo 模型名和出生点；
- ROS namespace；
- MAVROS `fcu_url`；
- setpoint publisher；
- 局部坐标原点和编队坐标变换。

**建议**：先完成单机轨迹、速度、yaw 和安全降落，再复制为第二架机；不要一开始同时排查三套端口、namespace 和控制逻辑。

## 10. 面向仿真实验的推荐操作流程

### 10.0 修改后是否需要编译

- 只改 `.launch`、`.yaml` 或 `.world`：通常重启相应节点即可；PX4 外部副本例外，仍需按第 2.3 节同步。
- 改 `autoarming_control.cpp` 或 `position_control*.cpp`：必须重新编译主 ROS1 工作空间并重新 source。`offboard/CMakeLists.txt:130-148` 分别构建位置控制库、`position_control` 和 `autoarming_control`。
- 改 Gazebo 插件 C++：必须重新编译 `simulation/sim_workspace`，并确认加载的是新生成的 `.so`。
- 改 EGO 源码：先处理 `CATKIN_IGNORE` 和依赖，再重新编译；不能只依赖当前 `devel/` 中可能残留的旧产物。

增量编译 Offboard 的常用方式：

```bash
cd "$HOME/AstraDroneOpen/AstraDrone_ros1_ws"
catkin_make --pkg offboard
source devel/setup.bash
```

`scripts/build_AstraDrone_ros1_x86.bin` 和 `scripts/build_sim_workspace_x86.bin` 当前是已编译 ELF 工具，无法像 shell 脚本一样静态审阅内部每一步。顶层 README 说明它们会清理并重建工作空间；日常单文件开发优先增量 `catkin_make`，需要重新部署 PX4/Gazebo 资源时再运行项目构建工具。

### 10.1 分阶段启动

终端 1：

```bash
roscore
```

终端 2：

```bash
source simulation/sim_workspace/devel/setup.bash
roslaunch px4 astra_example.launch
```

终端 3，可选建图：

```bash
source AstraDrone_ros1_ws/devel/setup.bash
roslaunch fast_lio mapping_mid360.launch rviz:=false
```

终端 4，最后启动控制：

```bash
source AstraDrone_ros1_ws/devel/setup.bash
roslaunch offboard autoarming_control.launch rviz:=false
```

### 10.2 启动控制前检查

```bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/local_position/pose
rostopic hz /mavros/local_position/pose
rostopic info /mavros/setpoint_position/local
```

重点确认：

- `connected: True`；
- 局部 pose 持续更新且没有 NaN；
- setpoint topic 没有意外的其他 publisher；
- Gazebo 未暂停；
- QGC 没有 estimator/failsafe 严重报警。

### 10.3 运行中验证“改动是否真的生效”

```bash
rosparam get /flight_mode
rosparam get /autoarming_control/hight
rosparam get /autoarming_control/target_laps
rosparam get /autoarming_control/radius
rostopic hz /mavros/setpoint_position/local
rostopic echo /mavros/setpoint_position/local
rostopic echo /mavros/local_position/odom
```

如果 `speed` 尚未按第 4.2 节接入源码，`rosparam get` 能看到它也不代表代码在使用它。

### 10.4 记录实验

`scripts/run_sh/record.sh:5-8` 已录制 MAVROS pose/odom/setpoint、Livox、TF、电池等关键话题。建议每次实验另外记录：

- 参数文件或 `rosparam dump`；
- Git commit/diff；
- world 名称；
- 目标轨迹；
- 实际位置、速度和偏航误差；
- 碰撞/超时/模式切换事件。

评价轨迹不能只看“飞起来了”，至少计算：

```text
位置 RMSE、最大位置误差、速度峰值、加速度峰值、完成时间、最小障碍距离、OFFBOARD 丢失次数
```

## 11. 常见现象与定位顺序

### 无人机不解锁或不进入 OFFBOARD

1. 检查 `/mavros/state` 是否 connected。
2. 检查 setpoint 是否以 20 Hz 左右持续发布；源码第 117-125 行会预热约 5 秒。
3. 检查 PX4 estimator 和 QGC 报警。
4. 检查是否有多个 setpoint publisher。
5. 检查 `fcu_url` 是否与当前 PX4 instance 匹配。

### 改了 `speed` 但速度不变

这是当前预期行为：launch 有参数，源码没有读取。按第 4.2 节修改并重新编译主 ROS 工作空间。

### 改了 `takeoff_height` 但还是直接飞到 3 m

也是当前预期行为：源码 `TAKEOFF` 第 195 行直接使用 `hight`。要么只改 `hight`，要么新增并实际读取 `takeoff_height`，再在达到该高度后进入下一阶段。

### 改了仓库中的 `astra_example.launch` 但场景没变

大概率运行的是 `~/PX4-Autopilot/launch/...` 外部副本。先用 `rospack find px4` 和 `cmp` 检查同步状态。

### FAST-LIO 正常建图，但飞机不避障

这是当前架构预期：FAST-LIO 输出地图，默认控制器不订阅地图也不调用规划器。需要按第 5.4 节建立 EGO/其他规划器到 MAVROS 的闭环。

### EGO 的 `max_vel` 改了，PX4 无人机没有变化

EGO 当前控制的是 `so3_quadrotor_simulator`，不是 PX4/Gazebo 模型；且包默认被忽略。先完成桥接和编译启用。

### `position_control.launch` 的初始目标没有生效

`DroneControl::ReadParams()` 当前没有调用；而且构造函数已进入无限循环。需按第 4.7 节整改。

### `rc_obstacle_avoidance.yaml` 改了但参数没变

launch 把参数加载到全局，代码从私有 namespace 读取。把 `<rosparam>` 放入 `<node>` 内后重启。

## 12. 推荐开发路线

### 阶段 A：建立可复现实验基线

- 空场启动默认单机。
- 只改 `hight`、`radius/side_length`、`target_laps`。
- 记录 setpoint 和实际 odom。
- 修复合法四元数、首帧 pose 等待和上锁结果检查。

### 阶段 B：实现真正的轨迹速度和航向

- 按第 4.2 节把 `speed` 接入时间/弧长参数化。
- 参数化跟踪误差、到达误差、最大高度和超时。
- 加入切线 yaw 和 yaw rate 限制。
- 用圆、方形、8 字依次验证。

### 阶段 C：改为航点/动作状态机

- 用单一节点管理解锁、起飞、悬停、航点、返航、降落。
- 每段都有到达条件和超时；始终持续发布 setpoint。
- 增加急停/悬停指令和 geofence。

### 阶段 D：接入静态避障规划

- 先启用 EGO 的独立 simulator，确认其原始示例可运行。
- 修复手动目标 z 硬编码。
- 再替换成 MAVROS odom 和真实仿真点云。
- 实现 `PositionCommand -> MAVROS PositionTarget` 桥接。
- 停用 EGO 内部 SO3 模拟器和默认 autoarming setpoint。

### 阶段 E：动态障碍和复杂场景

- 使用 `dynamic_avoidance_static.launch` 验证静态避障。
- 再使用 moving launch 和 `obstacle_params.yaml` 调速度/轨迹。
- 最后切 CRAIC/森林等复杂 world。

### 阶段 F：多机

- 先复制第二套 PX4/MAVROS 并验证独立悬停。
- 修复重复节点名、端口和 namespace。
- 每机独立控制器稳定后，再实现共享航点或编队控制。

## 13. 最终修改索引

| 想改变的行为 | 首选文件 | 函数/参数/行号 | 修改后效果 |
|---|---|---|---|
| 高度 | `offboard/launch/autoarming_control.launch` | `hight`，第 15 行 | 起飞和巡航高度改变 |
| 圈数 | 同上 | `target_laps`，第 14 行 | 飞行圈数改变 |
| 圆半径 | 同上 + `autoarming_control.cpp` | launch 新增 `radius`；源码第 97 行读取 | 圆大小改变 |
| 方形边长 | 同上 | `side_length`，第 17 行 | 方形大小改变 |
| 圆/方形 | 同上 | `flight_mode`，第 11 行 | 选择轨迹 |
| 真实参考速度 | `autoarming_control.cpp` | 参数区第 88-97 行、推进区第 219-233 行 | `speed` 以 m/s 参与轨迹推进 |
| 轨迹形状 | 同上 | `get_square_position` 第 43-56 行、`get_circle_position` 第 58-63 行、TRACKING 第 205-252 行 | 可做 8 字、椭圆、螺旋、自定义曲线 |
| 轨迹中心/出生点 | 同上 | TAKEOFF 第 192-204 行、轨迹函数 | 轨迹相对初始点而非固定原点 |
| 航向 | 同上 | target orientation，第 143、213-217 行附近 | 固定 yaw 或沿切线转头 |
| 动作顺序 | 同上 | `FlightPhase` 第 21-26 行、状态机第 192-266 行 | 悬停、前进、转向、返航等 |
| 航点控制 | `position_control_lib.cpp` 或新节点 | `DroneControl_main` 第 15-31 行、callback 第 42-51 行 | 外部航点驱动；需先修复当前实现 |
| EGO 速度/加速度 | `plan_manage/launch/run_in_sim.launch` | 第 35-36 行 | 仅 EGO 轨迹约束，默认不影响 PX4 |
| EGO 预设路线 | 同上 | 第 43、47-67 行 | 独立 EGO 仿真按预设航点飞 |
| EGO 手动目标高度 | `ego_replan_fsm.cpp` | `waypointCallback` 第 109-120 行，尤其第 119 行 | RViz 目标 z 可生效 |
| EGO 地图安全距离 | `advanced_param.xml` | 第 72-105、119-129 行 | 分辨率、膨胀、碰撞代价改变 |
| world | `astra_example.launch` | 第 15 行/launch arg | 仿真场景改变 |
| 动态障碍速度/路径 | `obstacle_params.yaml` | 第 6-44 行 | 障碍物按新轨迹运动 |
| 雷达范围/频率 | `mid360.sdf` | 第 35-75 行 | 点云频率、范围、采样改变 |
| FAST-LIO 输入/外参 | `mid360.yaml` | 第 2-24 行 | SLAM 话题、范围、标定改变 |
| 多机控制 | `autoarming_Mult.launch` | 第 9-34 行 | 需先修复节点名、PX4 实例和 namespace |

以上索引中，A/B/C 阶段均可围绕 `autoarming_control` 完成；只有需要绕障碍、动态重规划时，才值得进入 EGO 桥接改造。这样能把“轨迹生成问题”“PX4 Offboard 问题”“定位问题”“规划问题”逐层分离，便于快速得到可复现实验结果。
