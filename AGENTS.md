# AstraDroneOpen 项目协作总览

本文件供后续 GPT/Codex 在几分钟内建立共同事实。它是项目导航，不替代源码、launch、配置、测试证据或详细学习路线。

## 1. 项目目标与当前范围

AstraDroneOpen 正在把已有无人机学习代码整理成可复现、可维护的工程基线。当前只验收单机 Gazebo/PX4 SITL 仿真，主线是 ROS1 Noetic、Gazebo Classic、PX4、MAVROS、FAST-LIO 与 EGO-Planner。核心任务、规划、安全和执行模块不得写死 Gazebo 接口，以便后续迁移真机。

当前重点是先稳定单机的定位、规划、控制和安全契约，再按阶段完成固定高度绕塔、EGO 静态避障、工程安全状态机、动态障碍和多机。相机检测、QGIS/Cloud、真机和集群仍属于后续范围，不能写成已完成。

事实冲突时按以下顺序判断：

1. 当前源码、`package.xml`、`CMakeLists.txt`、launch、配置和测试；
2. 当前分支、HEAD、工作区、历史和运行证据；
3. `CODE_AUDIT_REPORT.md`；
4. README、教程和学习笔记；
5. 未来规划或口头设想。

“文件存在”不等于能编译、启动或通过飞行验收；没有当前证据时标记“待验证”。

## 2. 基线、环境与版本边界

- 已确认的学习和 Gazebo 开发基线：`ego-project@498c7c6`。
- 2026-07-21 阶段1开始时实际 HEAD：`dcd5bd5697ae141712df43766e9a0483e912a294`（提交主题“前置优化”）。开始时工作区干净，先前的 `forest.world` 默认值修改和前置优化已进入 HEAD。每个新任务仍须重新检查，不能假定 HEAD 不变。
- 2026-07-21 阶段2开始时实际 HEAD：`85dc37b1d32233cec69abf237083d0e6fdbe772e`，分支 `ego-project` 跟踪 `origin/ego-project`，工作区干净；这说明阶段1成果已进入当前 HEAD，不能再按旧文档假定其未提交。
- 2026-07-21 最近塔/30-16 m/yaw增量开始时实际 HEAD：`988e75a73375bb9c19d94ea98edf976ee74b5744`（阶段2成果已由负责人提交），分支相对远端领先1。开始时工作区仅有受保护的 `FAST_LIO/Log/mat_pre.txt` 修改；后续代理必须保护并先查看 diff。
- 2026-07-22 `worksite.world` 迁移开始时实际 HEAD：`2c8f3223920afabbc1d0f1a8b67e568e1fbe216c`，分支相对远端领先3，工作区干净。Gazebo Classic 实测加载 `<state>` 中的 `radio_tower=(-10.0551,19.7104)`，不是world前部模型声明里的旧坐标。
- 已观察技术栈：Ubuntu 20.04.6、ROS1 Noetic、Gazebo Classic 11.15.1、MAVROS 1.20.1、CMake 3.16.3、GCC 9.4、Python 3.8。
- 外部 PX4 位于 `/home/yanzu/PX4-Autopilot`，只读审计为 detached `99c40407ffd7ac184e2d7b4b293f36f10fe561ef`、`v1.15.4-dirty`。未经批准不得修改、清理、切换或升级。
- 当前 ROS、MAVROS、Gazebo、PX4、EGO 和 FAST-LIO 版本全部锁定；无项目负责人批准不升级。

该基线是受审计的学习/仿真开发基线，不是正式生产或真机基线。

## 3. 仓库与主线包

| 路径 | 用途与状态 |
|---|---|
| `AstraDrone_ros1_ws/` | 主 ROS1 catkin 工作空间；`build/`、`devel/` 是生成物，禁止手改 |
| `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/` | 既有起飞、悬停、多航点、连续轨迹、返航和降落 |
| `AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/` | 阶段1独立固定航线；阶段2新增逐目标EGO任务节点、分级场景和CSV证据 |
| `AstraDrone_ros1_ws/src/MissionControl/ego_gazebo_bridge/` | EGO 到 MAVROS/PX4 的安全桥；默认 dry-run |
| `AstraDrone_ros1_ws/src/Planner/ego-planner/` | EGO vendor 源码及上游模拟工具；存在项目历史修改 |
| `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/` | MID360 激光雷达-惯性里程计和注册点云 |
| `AstraDrone_ros1_ws/src/Utils/astra_custom_msgs/` | 项目消息/服务；尚不是统一 Cloud/QGIS 接口 |
| `simulation/sim_workspace/` | Gazebo 传感器和仿真插件的下层 catkin 工作空间 |
| `simulation/px4_sim_files/` | 仓库内保存的 PX4 SITL airframe、SDF 和 launch 部署资产 |
| `simulation/astra_gazebo_worlds/` | `example`、`forest`、`dynamic_avoidance` 等 world |
| `simulation/astra_gazebo_models/` | Gazebo 模型资源；存在模型不代表算法已接入 |
| `scripts/run_sh/` | 旧基础演示与 Stage 6 tmux 编排入口 |
| `third_party/` | 第三方源码；不得随意升级或大改 |
| `Detection/`、`Track/`、`Control/`、`Land/`、`Exploration/`、`Swarm/` | 候选、禁用或占位方向，不是当前主链 |

README 中的 ROS2、完整实机、探索和蜂群描述超前于当前代码；仓库当前没有 `AstraDrone_ros2_ws/`，必须以 ROS1 仿真源码为准。

## 4. 已具备能力与验收状态

已实现并有一定历史/当前证据：

- PX4 SITL、Gazebo、MAVROS、MID360、FAST-LIO 基础链；
- `offboard` 起飞、悬停、多航点、yaw、返航和 OFFBOARD 受控降落状态机；
- 圆、方形、8 字、椭圆轨迹生成、真实 `speed * dt` 推进、误差暂停和 rosbag 分析；
- EGO odom、点云、目标、B 样条、`PositionCommand` 与 bridge 的代码链路；
- bridge 默认 dry-run，dry-run 不创建或发布任何 MAVROS 控制 Topic；
- 2026-07-20 当前源码目标构建通过；`offboard` 7/7、bridge 9/9 单测通过，其中包含五类 MAVROS 控制出口冲突检查；隔离 dry-run 验证了全 preflight 健康和 raw-local 冲突阻断。
- 2026-07-21 阶段1新增独立 `astra_tower_mission`：固定高度N点加闭环、朝塔yaw、限速/限加速度、到达保持、超时、进度/CSV、返航和OFFBOARD受控降落；preview不创建控制publisher。最终目标包回归为新包6/6、offboard 7/7、bridge 9/9，共22/22。
- 阶段1按悬停→1点→4点→8点→第二次8点完成 `forest.world`/`radio_tower_0` 实测；两次完整任务均518 s，最终 `SUCCESS`、`armed=false`、`ON_GROUND`，控制图上唯一发布者为 `/tower_mission`。详细误差见 `fixed_orbit_inspection学习.md`。
- 2026-07-21 阶段2实现 FAST-LIO→过滤点云→EGO→traj_server→raw-local `PositionTarget` 闭环；`type_mask=0` 保留p/v/a/yaw/yaw_rate，raw唯一发布者为 `/ego_mavros_bridge`。任务逐目标等待新trajectory id和实际odom到达；HOLD锁存并有限时间降落。
- 阶段2按dry-run→控制冲突/指令断流故障注入→单目标→双目标→8个唯一点加首点闭环的低速绕塔完成验证。最终闭环653.900 s，任务最大跟踪误差0.295 m，最终`SUCCESS`、`armed=false`、`ON_GROUND`。详细证据见 `ego_waypoint_inspection学习.md`。
- 2026-07-22 当前未提交增量将主仿真和阶段1/2默认world改为 `worksite.world`；两阶段统一为 `radio_tower=(-10.0551,19.7104)`、10 m半径、8 m单层。阶段2任务层发布闭合全局参考，EGO逐段生成局部避障/重规划轨迹；圆周段朝塔，进场/返航沿水平速度前向。三包构建、39个本轮目标单测、launch参数展开和阶段1无控制preview通过；未启动 `--control` 或飞行。

仍未完成运行验收：

- 专门静态障碍、最小净空、不可达目标和规划失败场景；阶段2绕塔成功不能替代阶段3验收；
- `worksite.world` 8 m整圈、FAST-LIO/EGO局部轨迹、实际净空和分段yaw尚未从dry-run开始重新飞行验收；通用目标z仍受`manual_target_height`覆盖；
- 动态障碍预测、多机、相机检测主链接入、QGIS/Cloud、真机和干净 clone 交付。

## 5. 节点、数据流、frame 与重要 Topic

核心节点：

| 节点 | 职责 |
|---|---|
| `laserMapping` / `fastlio_mapping` | `/livox/lidar` + `/livox/imu` → `/Odometry`、`/cloud_registered`、`camera_init -> body` TF |
| `ego_planner_node` | 局部占据地图、搜索、B 样条优化和重规划 FSM |
| `traj_server` | `/planning/bspline` → `quadrotor_msgs/PositionCommand` |
| `waypoint_generator` | 规划目标 → EGO 使用的 `nav_msgs/Path` |
| `ego_mavros_bridge` | 目标/TF 适配、preflight、状态门禁、MAVROS 服务与 position setpoint |
| `autoarming_control` | 不经 EGO 的基础 OFFBOARD 飞行、航点/轨迹、返航和降落 |
| `tower_mission` | launch中的阶段1或阶段2任务节点名；阶段2只发EGO目标，不创建MAVROS控制publisher |

主数据流：

```text
Gazebo iris_mid360
  -> /livox/lidar + /livox/imu
  -> FAST-LIO
  -> /Odometry + /cloud_registered
  -> EGO-Planner -> /planning/bspline
  -> traj_server -> /planning/pos_cmd
  -> ego_mavros_bridge
  -> /mavros/setpoint_position/local
  -> MAVROS -> PX4 OFFBOARD -> Gazebo
```

目标链：`/move_base_simple/goal -> bridge -> /planning/goal -> waypoint_generator -> /waypoint_generator/waypoints -> EGO FSM`。

当前约定与待落实契约：

- 继续使用 FAST-LIO；统一契约覆盖 `map/camera_init/body/base_link/sensor`。
- 塔和任务航点使用 `map` 绝对坐标；home 只用于起飞点、返航和降落。
- 当前 EGO 规划 frame 默认 `camera_init`，MAVROS 本地 frame 默认 `map`；launch 中单位 `map -> camera_init` 只是仿真假设，不是标定结果。
- EGO 点云回调不做 TF 转换，`/cloud_registered` 必须已经在规划 frame。
- `iris_mid360` 的 D435i 相机朝机体前方，但当前 Stage 6 规划使用 MID360/FAST-LIO 点云，不使用 D435 深度图。

bridge 在 EGO 控制启用前监控的 MAVROS 控制类别包括 position、raw local、velocity、attitude 和 thrust。正常情况下任意时刻只能有一个控制出口。

阶段1数据流独立于EGO：`YAML -> tower_mission -> position setpoint（飞行）/raw local（下降）-> MAVROS -> PX4`；MAVROS local pose/velocity反馈用于到达和接地判断。脚本同时启动FAST-LIO并等待 `/Odometry` 健康，但任务节点不订阅EGO轨迹。

阶段2数据流：`YAML -> stage2任务逐目标 -> EGO -> traj_server PositionCommand -> bridge raw PositionTarget -> MAVROS/PX4`。内部`/stage2/cloud_registered_filtered`移除`camera_init`中低于0.2 m的地面带；阶段3前须复核低矮障碍风险。

## 6. 启动、构建与测试入口

| 用途 | 入口 |
|---|---|
| PX4 SITL + Gazebo + MAVROS | `simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch` |
| FAST-LIO MID360 | `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/launch/mapping_mid360.launch` |
| 基础多航点 | `offboard/launch/autoarming_control.launch` + `offboard/config/relative_waypoint_mission.yaml` |
| 连续轨迹 | `offboard/launch/continuous_trajectory.launch` |
| 阶段1预览/控制 | `scripts/run_sh/fixed_orbit_inspection.sh`；默认preview，`--control`才允许自动飞行 |
| 阶段2EGO接入 | `scripts/run_sh/ego_waypoint_inspection.sh`；默认dry-run，控制必须显式`--control --scenario single|dual|tower` |
| EGO/PX4 通用集成 | `ego_gazebo_bridge/launch/ego_gazebo_bridge.launch` + `config/ego_gazebo_bridge.yaml` |
| EGO规划栈通用编排 | `scripts/run_sh/ego_planner_stack.sh`；默认 dry-run，`--control` 才允许自动控制 |

两个工作空间按下层仿真、上层主工作区顺序构建。阶段1与既有包回归可使用：

```bash
source /opt/ros/noetic/setup.bash
cd /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES='offboard;astra_tower_mission;ego_gazebo_bridge' -j2
catkin_make -DCATKIN_WHITELIST_PACKAGES='offboard;astra_tower_mission;ego_gazebo_bridge' \
  run_tests_offboard run_tests_astra_tower_mission run_tests_ego_gazebo_bridge -j2
catkin_test_results build/test_results
```

构建会写 `build/`、`devel/`。不要编辑生成物；`.bin` 构建器可能清理构建目录，安装器和环境脚本可能改系统，未经明确批准不得运行。

## 7. 当前阶段与下一步

2026-07-20 已完成“阶段1开始前的最小前置优化”：

- 将确属 PX4 `MPC_LAND_SPEED` 的要求从非法 0.30 改为 v1.15.4 支持的 0.60 m/s；OFFBOARD 下降 setpoint 速度没有重新设计；
- EGO bridge 增加五类控制 Topic 唯一发布者检查；
- EGO dry-run/preflight 增加 MAVROS、odom、点云、goal、command 的 frame/时戳/有限值、TF、对齐、任务包络和控制权诊断；
- 完成 EGO、外部 PX4、`forest.world` 两座塔和相机方向的只读审计。

2026-07-21 已完成阶段1和阶段2。阶段2没有修改EGO C++核心或外部PX4，新增外部适配、任务节点和必要launch参数；固定高度单机Gazebo EGO闭环通过。下一步只等待项目负责人检查并亲自提交；不自动进入阶段3静态障碍。

## 8. 已确认的架构决策

1. 使用 `ego-project@498c7c6` 作为学习和 Gazebo 仿真开发基线。
2. 当前只验收 Gazebo 仿真，核心模块不能写死 Gazebo 接口。
3. 阶段1新建独立任务管理器；不继续扩展大型 offboard 状态机，不把任务逻辑写进 EGO 核心。
4. 阶段2已通过 `/mavros/setpoint_raw/local` 的 `PositionTarget` 执行完整 EGO p/v/a/yaw/yaw_rate；`type_mask=0`，ROS字段为ENU、MAVROS转换到PX4 NED。
5. 继续使用 FAST-LIO，并建立统一 `map/camera_init/body/base_link/sensor` 坐标契约。
6. 铁塔和任务航点使用 `map` 绝对坐标；home 只用于起飞点、返航和降落。
7. 阶段1采用固定高度8航点，按单点→4点→8点验证；yaw 朝塔；方向参数化、默认逆时针。
8. 任务目标 z 必须保留并受高度上下限约束，禁止 EGO 静默覆盖。
9. EGO 优先通过外部适配层扩展；必要核心修改须先批准并形成独立 vendor patch。
10. 大型状态机随阶段增量拆分，不一次性重写。
11. 正常起飞、悬停、任务、返航和受控降落均使用 PX4 OFFBOARD。
12. 任意时刻只允许一个 MAVROS 控制出口；严重 OFFBOARD 断流时允许 PX4 failsafe 接管。
13. 当前不做动态障碍预测，阶段6再决策。
14. 先完成单机；单机稳定后再做集群，阶段8再决定是否使用 EGO-Swarm。
15. 锁定当前 ROS、MAVROS、Gazebo、PX4、EGO 和 FAST-LIO 版本，无批准不升级。
16. Codex 不得 commit 或 push，所有提交由项目负责人亲自完成。

## 9. 风险、禁止事项与工作纪律

主要遗留风险：

- 当前 EGO vendor 没有可识别的精确上游 commit，直接修改尚未形成可重放 patch；`manual_target_height` 仍覆盖目标 z。当前增量只允许固定8 m并在启动时严格匹配；通用多高度前仍需适配层方案或批准vendor patch。
- traj_server仍生成轨迹前视航向，不消费任务目标orientation；当前增量已在外部bridge按任务模式重算圆周朝塔yaw并限速，进场/返航保留前视yaw，但尚无飞行证据。
- bridge 仍是位置跟随器，不是完整 EGO 动态轨迹执行器；不得把它描述成避障闭环已验收。
- 已确认的正常降落架构是 OFFBOARD 受控降落，但 bridge 现有异常/land 状态仍会请求 `AUTO.LAND`；本轮明确不重设计降落控制，后续须按批准阶段消除该差异。
- `map -> camera_init`、`body -> base_link` 和传感器外参尚未形成实测正式契约。
- `autoarming_control.cpp`、bridge 状态机仍较大；只允许随阶段增量拆分。
- 外部 PX4 dirty、detached；仓库部署资产与外部树尚无可复现同步机制。
- `forest.world/radio_tower_0` 的阶段1固定航线和阶段2 EGO闭环只是历史证据。当前 `worksite.world/radio_tower` 8 m方案只完成静态几何复核和无控制验证，没有专门静态障碍最小净空或飞行证据。
- bridge阶段2新增有限超时、锁存HOLD、跟踪误差和断流/控制冲突证据；完整不可达目标、全部服务失败、长期稳定性仍未验收。

任何 GPT/Codex 开始任务前必须只读执行并报告 `git branch --show-current`、`git rev-parse HEAD`、`git status --short --branch`、相关 diff；现有修改、删除、未跟踪文件均视为用户资产。

未经明确授权禁止：

- 恢复、覆盖、清理、暂存、提交或 push 用户工作区；切分支或改 Git 历史；
- 修改或升级外部 PX4、第三方依赖、ROS/MAVROS/Gazebo/EGO/FAST-LIO；
- 编辑 `build/`、`devel/`、生成消息、日志或缓存；
- 改公共 Topic/消息/TF 契约，或直接修改 EGO 核心；
- 移除 `/use_sim_time` 保护，把仿真参数当真机参数；
- 在控制权、preflight 和明确批准不足时使用 `--control`、解锁、起飞或飞行。

正常任务的目标架构保持 PX4 OFFBOARD。`MPC_LAND_SPEED` 是 PX4 降落参数；OFFBOARD 受控降落的下降 setpoint 速度是另一概念。本轮只修复前者的非法要求，没有重设计现有降落状态或 setpoint。

## 10. 详细资料索引

- `CODE_AUDIT_REPORT.md`：`498c7c6` 基线的完整代码、构建、依赖和风险审计；它是历史快照，当前结论须与源码复核。
- `ego_planner_工程落地学习.md`：详细数据流、基础知识、决策门、阶段1至阶段9路线、验收标准和本轮前置优化记录。
- `fixed_orbit_inspection学习.md`：阶段1实现、参数、Topic/TF、真实分级仿真证据和新手重启/排障手册。
- `ego_waypoint_inspection学习.md`：阶段2 raw/type mask/ENU语义、TF契约、逐目标任务、安全监控、分级飞行与CSV证据。
- `legacy_ego_integration.md`：旧0–6路线的EGO/PX4/FAST-LIO集成历史记录，不代表当前阶段编号。
- `offboard/include/offboard/trajectory_reference.h`、`landing_profile.h`：连续轨迹与 OFFBOARD 软降落数学。
- `ego_gazebo_bridge/config/ego_gazebo_bridge.yaml`：EGO bridge 通用安全、Topic、frame 和任务包络参数；原旧路线名称 `stage6_gazebo.yaml` 已停用，避免与当前阶段编号混淆。
