# AstraDroneOpen 阶段二：EGO-Planner 接入与验收记录

## 启动方式（先看这里）

本任务不是同时启动阶段1和阶段2两套脚本，而是只使用阶段2入口。阶段2已经复用阶段1的绕塔任务语义，并接入 FAST-LIO、EGO 局部重规划、raw-local 执行和分段 yaw 策略。两套入口同时运行会造成 ROS master、Gazebo/PX4 和控制权冲突。

如果之前启动了阶段1，先只停止阶段1自己的 tmux 会话：

```bash
cd /home/yanzu/AstraDroneOpen
scripts/run_sh/fixed_orbit_inspection.sh --stop
```

首次先运行带 Gazebo GUI 和 EGO/FAST-LIO RViz 的 dry-run。此命令会启动 PX4/Gazebo、FAST-LIO、EGO、`traj_server`、bridge 和任务管理器，但不创建 MAVROS setpoint publisher、不解锁、不起飞：

```bash
scripts/run_sh/ego_waypoint_inspection.sh --scenario dry-run --gui --rviz --attach
```

检查 RViz 中存在 `FAST-LIO/Registered Cloud`、`EGO-Planner/Inflated Occupancy`，并确认 preflight 健康后，按 `Ctrl+B`、再按 `D` 脱离 tmux，然后停止 dry-run：

```bash
scripts/run_sh/ego_waypoint_inspection.sh --stop
```

正式执行“起飞/进场4 m → worksite单塔30 m、14 m半径绕一圈 → EGO局部避障/重规划 → 塔前下降4 m → 返航 → 降落”的完整任务：

```bash
scripts/run_sh/ego_waypoint_inspection.sh --control --scenario tower --waypoints 8 --gui --rviz --attach
```

该入口中，EGO高度上限为32 m，地图覆盖至32.5 m，绕塔参考高度为30 m、半径为14 m，8个圆周航点以塔心整体逆时针旋转22.5°并从-67.5°开始，任务层发布包含4/30/4 m高度变化的闭合全局参考，EGO沿参考逐段生成局部轨迹。`use_goal_height=true` 使EGO消费每个目标自身的 z；绕塔时机头朝塔心，进场和返程按 `PositionCommand` 水平速度前向。任务结束或异常时使用同一停止命令：

```bash
scripts/run_sh/ego_waypoint_inspection.sh --stop
```

> 日期：2026-07-21  
> 分支：`ego-project`  
> 实施前/当前 HEAD：`85dc37b1d32233cec69abf237083d0e6fdbe772e`  
> 范围：单机 Gazebo/PX4 SITL、FAST-LIO、EGO-Planner、traj_server、MAVROS raw-local 执行  
> 不包含：阶段三专门静态障碍验收、多层/螺旋、动态障碍、多机、真机

## 0. 当前配置增量（尚未飞行验收）

下文第1至第8节保留的是 `forest.world/radio_tower_0` 的历史固定高度阶段二验收记录，不能当作当前 `worksite.world` 4/30/4 m配置的飞行证据。当前工作树已通过一个默认关闭、显式启用的最小 EGO vendor patch 支持目标 z：

- EGO地图从-0.5 m覆盖至32.5 m，以32.0 m虚拟天花板和bridge高度上限形成一致门禁；绕塔参考高度为30.0 m、半径为14.0 m；
- 任务层使用 `worksite.world` 运行时单塔 `radio_tower=(-10.0551,19.7104)`；
- `/tower_mission/global_reference` 保存4 m入口、30 m闭环和4 m出口的全局参考，8个圆周点从-67.5°开始等间隔分布，任务层逐段给目标，EGO继续负责每段局部避障与重规划；
- 进场第一段和返航由bridge按 `PositionCommand` 水平速度重算机头前向；进入圆周后，bridge根据 `/tower_mission/selected_tower_center` 重算并限速执行朝塔yaw，闭环完成前切回速度前向；
- 起飞、进场和返航高度为4.0 m，绕塔高度为30.0 m；bridge高度上限为32.0 m。
- 实测EGO在塔前水平进场末端会稳定在目标约0.43--0.48 m处，原0.40 m到达阈值造成约17 s等待；现在只对4 m塔前升降切换使用0.50 m阈值并保留1 s稳定保持，普通绕塔检查点仍为0.40 m。

当前代码已通过目标包构建、88项回归测试、launch参数展开和阶段1无控制preview。本次塔前切换修正使用现有CSV回放验证，尚未重新执行控制飞行。

## 1. 结论

阶段二在当前固定版本和 `forest.world` 中完成：

- `enable_control=false` 下验证 FAST-LIO → EGO → traj_server，且两个 MAVROS setpoint Topic 均无发布者；
- `astra_tower_mission` 新增独立阶段二任务节点，逐个发布现有 `/move_base_simple/goal`，只在新 trajectory id 和实际 `/Odometry` 到达保持都满足后推进；
- bridge 把 `/planning/pos_cmd` 的 position、velocity、acceleration、yaw、yaw_rate 全部写入 `/mavros/setpoint_raw/local`；
- dry-run、控制权冲突、指令断流安全终态、空场单目标、双目标、低速 8 点加首点闭环绕塔按顺序完成；
- 最终完整闭环任务用时 653.900 s 仿真时间，任务记录最大跟踪误差 0.295 m，完成 EGO 返航、`AUTO.LAND`、解锁和 `ON_GROUND`；
- 没有修改外部 PX4、FAST-LIO 源码或 EGO C++ 核心，没有升级依赖，没有 commit/push。

这只证明当前固定高度 Gazebo 链路。绕塔场景中虽然存在铁塔，但本轮没有布置“挡住直线路径”的专门障碍、测量最小净空或验证不可达目标，因此不能写成阶段三静态避障已经通过。

## 2. 最终数据流

```text
Gazebo iris_mid360
  -> /livox/lidar + /livox/imu
  -> FAST-LIO laserMapping
  -> /Odometry + /cloud_registered
  -> ground_cloud_filter
  -> /stage2/cloud_registered_filtered
  -> EGO ego_planner_node
  -> /planning/bspline
  -> traj_server
  -> /planning/pos_cmd
  -> ego_mavros_bridge
  -> mavros_msgs/PositionTarget /mavros/setpoint_raw/local
  -> MAVROS ENU-to-NED conversion
  -> PX4 OFFBOARD
```

目标和任务推进：

```text
ego_waypoint_mission_node
  -> one current PoseStamped /move_base_simple/goal
  -> bridge validation + transform
  -> /planning/goal
  -> waypoint_generator
  -> /waypoint_generator/waypoints
  -> EGO new Bspline / trajectory_id
  -> actual /Odometry arrival + hold
  -> next goal
```

`ground_cloud_filter` 是按功能命名的过滤节点，阶段二通过内部适配 Topic 使用它，不改变消息类型或对外任务接口。它只保留 `camera_init` 中 `z >= 0.2 m` 的点，以避免地面起点被 EGO 膨胀后判为“无人机在障碍内”。阶段三必须重新审查这一阈值，因为它也可能隐藏非常低矮的障碍。

## 3. TF 与物理语义契约

| frame | 物理语义 | 当前发布者/来源 | 当前关系与限制 |
|---|---|---|---|
| `map` | PX4/MAVROS local ENU 世界坐标；home、塔和任务绝对目标均在此表达 | MAVROS local position 消息语义；不是本轮新增 TF 根 | 东/北/上；home 只用于起飞点、返航和降落 |
| `camera_init` | FAST-LIO 初始化世界坐标，也是 EGO 规划世界 | FAST-LIO 消息和 `camera_init -> body` TF | 仿真中由 `/verified_map_to_planning_frame` 发布 `map -> camera_init` 单位静态 TF；这是仿真假设，不是标定结果 |
| `body` | FAST-LIO IMU 状态/机体估计原点 | FAST-LIO `laserMapping` 动态发布 `camera_init -> body` | 用于 FAST-LIO 状态，不等同于已经标定的 PX4 `base_link` |
| `base_link` | MAVROS/PX4 控制机体原点语义 | `/mavros/local_position/pose` 表达车辆本地位姿；当前未验证独立 `body -> base_link` TF 发布者 | bridge 用规划 odom 与 MAVROS pose 做位置/yaw 对齐门禁；正式外参仍待标定 |
| `mid360_link` | Gazebo MID360 点云传感器 frame | Livox Gazebo plugin，`/livox/lidar.header.frame_id` | 原始点云 frame；FAST-LIO 输出已变换到 `camera_init` |
| `mid360::lidar_link` | Gazebo MID360 IMU plugin frame | Gazebo IMU plugin，`/livox/imu.header.frame_id` | 名字与点云 frame 不同；FAST-LIO 通过配置消费，不应靠字符串假定相同 |
| sensor extrinsic | IMU/LiDAR 内部外参 | FAST-LIO YAML，不是 ROS TF 发布者 | `R=I`，`T=[-0.011,-0.02329,0.04412] m`；真机必须重新标定 |

运行证据中 `map -> camera_init` 的平移为零、RPY 为零；`/Odometry` 和过滤后点云均为 `camera_init`。raw 转换只允许规划世界与 MAVROS 世界重力对齐，roll/pitch 偏差超过 1° 即拒绝。

## 4. PositionTarget 契约

阶段二使用：

- Topic：`/mavros/setpoint_raw/local`
- 类型：`mavros_msgs/PositionTarget`
- `coordinate_frame = FRAME_LOCAL_NED (1)`
- ROS 消息字段按 MAVROS 约定仍是 ENU；MAVROS 在发送 MAVLink/PX4 前转换为 NED
- `type_mask = 0`：position、velocity、acceleration、yaw、yaw_rate 全部有效
- `FORCE` bit 不置位，因此 `acceleration_or_force` 表示加速度，不是力

转换规则：

- position 做完整 `camera_init -> map` 位姿变换；
- velocity、acceleration 只做旋转，不叠加平移；
- yaw 加世界坐标 yaw 偏移并归一化到 `[-pi, pi]`；
- yaw_rate 保持符号，只做绝对值限幅；
- raw 限幅为速度 0.35 m/s、加速度 0.60 m/s²、yaw_rate 0.75 rad/s；
- 起飞和 HOLD 也发布同一 raw 类型，但速度、加速度和 yaw_rate 为零；EGO 跟踪不再做位置追赶或丢弃导数。

实测 raw 示例为 `frame_id=map`、`coordinate_frame=1`、`type_mask=0`，同时出现非零 velocity 和 acceleration；Topic 图上唯一发布者是 `/ego_mavros_bridge`。

### yaw 的实际含义

历史阶段二验收时，EGO `traj_server` 根据轨迹前视方向生成 yaw/yaw_rate，所以执行的是“沿运动方向看”的 EGO yaw。任务目标虽然携带朝塔 orientation，但 EGO manual goal 链不消费该 orientation，不能把历史证据描述成相机始终朝塔。

当前增量已经采用外部适配方案：圆周段由bridge按最近塔中心重算 yaw/yaw_rate；进场和返航按 `PositionCommand` 的水平速度重算 yaw/yaw_rate，低速时保持上一航向防止抖动，两者均以 `max_yaw_rate` 做连续限幅。EGO vendor保持不变，新增策略尚无飞行证据。

## 5. 任务与安全状态

阶段二任务节点不创建任何 MAVROS 控制 publisher。场景为：

- `dry_run`：一个 3 m 空场目标，只验证到新轨迹；
- `single`：`(3,0,4)`；
- `dual`：`(3,0,4)`、`(3,3,4)`；
- `tower`：`radio_tower_0`，中心 `(24.4614,39.3288)`，半径 10 m，高度 4 m，南侧起点，逆时针；N个唯一点后自动追加首点形成闭环。

每个目标都有规划总超时和到达总超时。新目标必须产生比发布前更新的 trajectory id；到达使用实际 `/Odometry` 三维误差和保持时间，不使用“发布完成”冒充“飞到”。

bridge/任务监控：

- FCU、extended state、MAVROS pose、planner odom、cloud、goal、PositionCommand 的 frame、有限值、源时间戳和新鲜度；
- planner/MAVROS 位置和 yaw 对齐；
- 全部五类 MAVROS 控制 Topic 发布者；
- raw 指令包络、跟踪误差、OFFBOARD 状态、规划/起飞/返航/降落超时；
- 20 ms 跨进程相邻 `/clock` future 容差，0.60 s command stale 上限；超过边界仍拒绝；
- HOLD 为锁存状态，不因下一条偶然新鲜消息自动恢复。任务观察到 HOLD 后等待 3 s 再请求降落；即使任务失效，bridge 也在 5 s 后独立进入降落。

所有等待和重试都有总时限。正常返回和当前异常落地仍使用 bridge 既有 `AUTO.LAND`；这与项目长期目标“正常流程 OFFBOARD 受控降落”不一致，列为未解决风险，不在阶段二顺手重设计。

## 6. 实测证据

### 6.1 dry-run

- `/Odometry`：约 10.0 Hz，`camera_init`
- `/cloud_registered`：约 10.0 Hz，`camera_init`
- `/stage2/cloud_registered_filtered`：约 10.0 Hz，`camera_init`
- `/planning/pos_cmd`：100 Hz，`camera_init`
- `/planning/bspline`：事件式新轨迹消息
- `/mavros/setpoint_raw/local`：无 publisher
- `/mavros/setpoint_position/local`：无 publisher
- 结果：`SUCCESS: FAST-LIO -> EGO -> traj_server chain verified`
- CSV：`/tmp/astra_stage2_evidence/dry_run_20260721_181935.csv`

### 6.2 故障注入

1. 隔离 ROS master 中注入外部 velocity publisher：bridge 在创建 raw publisher 前进入 `ERROR`，`/mavros/setpoint_raw/local` 不存在。
2. 倾斜 world TF 单测：roll/pitch 超过重力对齐容差时 raw 转换拒绝。
3. 实际指令断流路径：bridge 进入锁存 `HOLD`，任务进入 `FAILURE_HOLD -> FAILURE_LANDING`，最终 `DONE`、解锁、接地，没有继续目标或无限恢复。
4. 实飞定位出跨进程 `/clock` 1 ms 偏差；新增 20 ms 有界 future 容差及边界测试。stale 超过 0.60 s 仍进入上述安全路径。

### 6.3 分级飞行

| 场景 | 仿真时间 | 任务最大跟踪误差 | EGO 参考峰值 v | EGO 参考峰值 a | 结果 |
|---|---:|---:|---:|---:|---|
| 单目标 | 61.150 s | 0.139 m | 0.353 m/s | 0.099 m/s² | 目标、返航、降落、解锁、接地成功 |
| 双目标 | 77.600 s | 0.135 m | 0.413 m/s | 0.094 m/s² | 两个新 trajectory id、实测到达、返航、降落成功 |
| 8 点+首点闭环 | 653.900 s | 0.295 m | 0.317 m/s | 0.100 m/s² | 8个唯一点、首点复访、EGO返航、降落、解锁、接地成功 |

EGO 参考的 yaw_rate 峰值可到 `pi rad/s`；raw 输出限为 0.75 rad/s。EGO 参考速度偶尔高于规划名义值，raw 输出仍受 0.35 m/s 最终门限。CSV 同时记录目标、实际位置、p/v/a/yaw/yaw_rate、goal error、tracking error、trajectory id 和 bridge state：

- `/tmp/astra_stage2_evidence/single.csv`
- `/tmp/astra_stage2_evidence/dual_pass.csv`
- `/tmp/astra_stage2_evidence/tower8_closed.csv`

最终闭环终态：bridge 最大跟踪误差 0.353 m；任务以其订阅采样记录最大 0.295 m；最终 `armed=false`、`LANDED_STATE_ON_GROUND`。

## 7. 构建、测试和启动

目标包构建与回归：

```bash
source /opt/ros/noetic/setup.bash
cd /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES='offboard;astra_tower_mission;ego_gazebo_bridge' -j2
catkin_make -DCATKIN_WHITELIST_PACKAGES='offboard;astra_tower_mission;ego_gazebo_bridge' \
  run_tests_offboard run_tests_astra_tower_mission run_tests_ego_gazebo_bridge -j2
catkin_test_results build/test_results
```

启动入口默认 dry-run：

```bash
scripts/run_sh/ego_waypoint_inspection.sh
scripts/run_sh/ego_waypoint_inspection.sh --control --scenario single
scripts/run_sh/ego_waypoint_inspection.sh --control --scenario dual
scripts/run_sh/ego_waypoint_inspection.sh --control --scenario tower --waypoints 8
scripts/run_sh/ego_waypoint_inspection.sh --stop
```

飞行场景必须同时显式提供 `--control` 和非 dry-run scenario。脚本默认 headless，并阻断已有 ROS/PX4/Gazebo/控制进程冲突。

## 8. 未解决风险与下一步边界

1. `map -> camera_init` 仍是仿真单位假设；`body -> base_link` 和 sensor 外参没有形成真机标定链。
2. EGO vendor 已增加最小 `fsm/use_goal_height` 开关，默认关闭以保持旧任务兼容；阶段二显式开启后消费4 m和30 m目标 z。该patch已做单元测试，但worksite多高度闭环仍待飞行验收。
3. 当前已在外部bridge实现“圆周朝塔、其他段速度前向”的yaw策略，但只有数学单测、构建和无控制参数证据，尚未完成worksite 4/30/4 m飞行验收。
4. 地面带过滤阈值可能隐藏低矮障碍；阶段三前必须结合障碍几何和地图策略复核。
5. 正常返航后的 bridge 降落仍为 `AUTO.LAND`，尚未统一为项目长期要求的 OFFBOARD 受控降落。
6. 本轮没有不可达目标、专门静态障碍最小净空、长期循环、干净 clone、install-space 或真机证据。
7. 外部 PX4 仍是 dirty/detached 固定树；本轮只读使用，没有修改。

因此，历史4 m阶段二“固定高度单机 Gazebo EGO 接入”验收通过；当前 `worksite.world` 4/30/4 m单塔与分段yaw增量仍待从dry-run开始重新验收。不得自动进入阶段三，不得把本轮称为静态避障验收、多高度巡检或真机验收。
