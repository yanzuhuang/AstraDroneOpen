# AstraDroneOpen 项目协作说明

本文件适用于整个仓库，供后续 GPT/Codex 在开始任务前快速建立共同事实。它不是安装教程，也不替代源码、launch、配置或测试证据。

## 1. 事实优先级与项目定位

发生冲突时按以下顺序判断：

1. 当前工作区中的源码、`package.xml`、`CMakeLists.txt`、launch、配置和脚本；
2. 当前 Git 分支、工作区状态、提交历史及仓库内保留的运行证据；
3. `CODE_AUDIT_REPORT.md` 等审计材料；
4. README、教程和学习笔记；
5. 未来规划或口头设想。

任何“文件存在”只表示有实现入口，不等于当前能够编译、启动或通过飞行验收。没有运行证据时必须写“待验证”或“待确认”。

审查快照（2026-07-20）：

- 分支/提交：`ego-project@498c7c6`；开始新任务时必须重新检查，不能假定仍未变化。
- 定位：受审计的学习与仿真集成候选基线，不是正式交付、生产或真机基线。
- 当前主线：Ubuntu 20.04、ROS1 Noetic、Gazebo Classic、PX4 SITL、MAVROS、FAST-LIO 和 EGO-Planner 的单机仿真。
- 当前阶段：基础飞行、多航点和连续轨迹已有实现及部分仿真证据；EGO 数据链和控制桥已接通，但完整避障闭环尚未完成运行验收。
- 审查时工作区非干净，包含用户已有删除和未跟踪文件。精确状态以任务开始时的 `git status` 为准，禁止擅自恢复、清理或提交。

项目正在把学习代码整理成可复现、可维护、可正式交付的工程基线。未来方向包括自主规划与避障、多机/蜂群协同感知、相机和检测算法接入，以及向 QGIS/Cloud 统一提供轨迹、位姿、速度、图像和检测结果。上述未来方向目前均不能写成已完成。

## 2. 仓库地图

| 路径 | 作用 | 当前状态 |
|---|---|---|
| `AstraDrone_ros1_ws/` | 主 ROS1 catkin 工作空间 | 当前核心工作区；`build/`、`devel/` 是生成物，禁止手改 |
| `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/` | 基础 Offboard 飞行、航点任务和连续轨迹 | 当前已启用 |
| `AstraDrone_ros1_ws/src/MissionControl/ego_gazebo_bridge/` | EGO 到 MAVROS/PX4 的仿真安全桥 | 当前已启用，默认 dry-run |
| `AstraDrone_ros1_ws/src/Planner/ego-planner/` | EGO 规划器及其上游模拟器/工具 | 已启用多个包；包含直接修改的 vendor 代码 |
| `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/` | MID360 激光雷达-惯性里程计和注册点云 | 当前主定位/点云链 |
| `AstraDrone_ros1_ws/src/Utils/astra_custom_msgs/` | 项目自定义消息和服务 | 已启用，但尚不是统一 Cloud/QGIS 接口 |
| `AstraDrone_ros1_ws/src/Detection/`、`Track/`、`Control/`、`Land/` | YOLO、AprilTag、ArUco、预测、跟踪、遥控避障、降落等候选模块 | 大多存在 `CATKIN_IGNORE`，不是当前主链 |
| `AstraDrone_ros1_ws/src/Exploration/`、`Swarm/` | 自主探索和蜂群方向 | 当前仅有占位说明，无可用实现 |
| `simulation/sim_workspace/` | Gazebo 传感器、场景和其他仿真包的独立 catkin 工作空间 | Stage 6 的下层工作空间 |
| `simulation/px4_sim_files/` | PX4 SITL airframe、SDF 和启动文件 | 依赖仓库外部 PX4 源码树 |
| `simulation/astra_gazebo_worlds/` | `example`、`forest`、`dynamic_avoidance` 等 world | 仿真场景资源 |
| `simulation/astra_gazebo_models/` | 大型 Gazebo 模型库 | 资源目录，不代表相关算法已实现 |
| `scripts/run_sh/` | 旧基础演示和 Stage 6 tmux 编排 | 运行入口；存在平台和路径约束 |
| `scripts/*.bin`、`scripts/env_sh/` | 封装的构建器、安装器和环境脚本 | 可能清理构建目录或修改系统；未经明确批准不得运行 |
| `third_party/` | Livox、AprilTag、ArUco、GeographicLib、NLopt、RealSense 等第三方源码 | 不得随意升级或大改 |
| `docs/`、根目录 Markdown | 上游教程、学习记录和审计材料 | 部分内容过时或超前，必须与代码复核 |
| `hardware/`、`system_images/` | 硬件与镜像规划 | 当前基本为占位内容 |

README 声称存在 ROS2 工作空间和完整实机/蜂群/探索能力，但仓库当前没有 `AstraDrone_ros2_ws/`，多个所述包也不存在。当前工程认知必须以 ROS1 仿真代码为准。

## 3. 当前参与主线的 ROS 包和节点

主工作区当前可被 catkin 发现的关键包包括：

- `offboard`：`autoarming_control`、`position_control`、`pub_origin`；
- `ego_gazebo_bridge`：`ego_mavros_bridge`；
- EGO：`plan_env`、`path_searching`、`bspline_opt`、`traj_utils`、`ego_planner`、`quadrotor_msgs`、`waypoint_generator`，以及若干原生 mock/SO3 模拟器包；
- 定位/驱动：`fast_lio`、`livox_ros_driver`、`livox_ros_driver2`；
- 公共消息：`astra_custom_msgs`。

核心节点职责：

| 节点 | 职责 |
|---|---|
| `fastlio_mapping` / launch 中的 `laserMapping` | 读取 `/livox/lidar`、`/livox/imu`，发布 `/Odometry`、`/cloud_registered` 和 `camera_init -> body` TF |
| `autoarming_control` | 仿真保护、OFFBOARD/解锁、起飞、航点或轨迹任务、返航和降落状态机 |
| `ego_planner_node` | 局部体素地图、全局参考、局部搜索、B 样条优化和重规划 FSM |
| `traj_server` | 将 `/planning/bspline` 按时间采样成 `quadrotor_msgs/PositionCommand` |
| `waypoint_generator` | 将目标转换为 EGO 使用的 `nav_msgs/Path` |
| `ego_mavros_bridge` | 目标/TF 适配、输入时效和包络检查、飞行状态机、MAVROS 服务调用及 setpoint 输出 |

旧 `autoarming_Mult.launch` 只是过时的多实例启动片段，存在重复节点名和旧参数，不能视为多机协同、任务分配或机间避碰实现。

## 4. 关键文件与启动入口

| 用途 | 文件 |
|---|---|
| PX4 SITL + Gazebo + MAVROS | `simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch` |
| FAST-LIO MID360 | `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/launch/mapping_mid360.launch` |
| FAST-LIO 参数 | `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/config/mid360.yaml` |
| 多航点任务 | `offboard/launch/autoarming_control.launch` + `offboard/config/stage3_waypoints.yaml` |
| 圆/方形/8 字/椭圆轨迹 | `offboard/launch/stage4_trajectory.launch` |
| 轨迹生成与软降落数学 | `offboard/include/offboard/trajectory_reference.h`、`landing_profile.h` |
| 轨迹 rosbag 分析 | `offboard/scripts/stage4_analyze_bag.py` |
| EGO 参数入口 | `Planner/ego-planner/planner/plan_manage/launch/advanced_param.xml` |
| EGO/PX4 集成 launch | `MissionControl/ego_gazebo_bridge/launch/stage6_gazebo.launch` |
| bridge 安全/Topic 参数 | `MissionControl/ego_gazebo_bridge/config/stage6_gazebo.yaml` |
| 旧基础演示 | `scripts/run_sh/pc_example.sh`；默认会启动旧控制器，`--base-only` 才不启动 |
| Stage 6 编排 | `scripts/run_sh/stage6_planner.sh`；默认 dry-run，`--control` 才允许自动控制 |

路径表中的 `offboard/` 和 `MissionControl/` 均位于 `AstraDrone_ros1_ws/src/MissionControl/` 下。

## 5. 当前运行链和模块关系

基础飞行链：

```text
Gazebo/PX4 SITL -> MAVROS -> /mavros/local_position/*
                         ^
                         | Pose/PositionTarget setpoint
                 autoarming_control
```

Stage 6 规划链：

```text
Gazebo iris_mid360
  -> /livox/lidar + /livox/imu
  -> FAST-LIO
  -> /Odometry + /cloud_registered
  -> EGO-Planner
  -> /planning/bspline
  -> traj_server
  -> /planning/pos_cmd (PositionCommand)
  -> ego_mavros_bridge
  -> /mavros/setpoint_position/local
  -> MAVROS -> PX4 OFFBOARD -> Gazebo 无人机
```

目标链为 `/move_base_simple/goal -> bridge -> /planning/goal -> waypoint_generator -> /waypoint_generator/waypoints -> EGO FSM`。

当前规划坐标系默认是 `camera_init`，MAVROS 本地坐标系默认是 `map`。`stage6_gazebo.launch` 默认发布单位变换 `map -> camera_init`；这只是特定仿真的配置假设，不是自动标定结果。EGO 的点云回调不做 TF 转换，因此 `/cloud_registered` 必须已经处于规划坐标系。

当前 bridge 只把 `PositionCommand` 的位置和 yaw 转为 `PoseStamped`，未把速度、加速度和 `yaw_dot` 传入 MAVROS。它目前是限速的位置追随器，不是完整的 EGO 动态轨迹执行器。

相机/检测模块未来应作为并行感知链产生图像、检测框、类别、置信度和带时间/frame 的空间结果；经过明确接口后再供任务管理、协同感知或 Cloud 使用，不能直接混入 EGO 核心。QGIS/Cloud 应作为下游数据消费者，统一接口和协议当前尚不存在。

## 6. 功能状态

### 已实现且有一定验证证据

- PX4 SITL、Gazebo、MAVROS、MID360 和 FAST-LIO 的基础仿真链；
- `offboard` 的起飞、悬停、多航点、yaw、返航和降落状态机；
- 圆形、方形、8 字和椭圆轨迹生成、真实 `speed * dt` 推进、跟踪误差暂停和分析脚本；
- Git 历史记录了基础飞行、多航点以及圆/方形/8 字仿真完成；
- `stage1_records/run01_circle_baseline.bag` 保留了约 228 秒的状态、位姿、里程计和 setpoint 证据；
- 历史阶段记录中有一次低速圆的量化结果，但不能代替今后修改后的回归测试。

### 代码已存在，但仍需当前基线重新验证

- 最新降落逻辑的落地状态和最终 `armed=false` 闭环；
- 当前 checkout 的完整 `catkin_make`；
- `offboard` 和 `ego_gazebo_bridge` 当前源码对应的单元测试。已有 7/7 和 6/6 结果来自早于 HEAD 的构建产物，只能作辅助证据；
- EGO 必需包、`stage6_gazebo.launch`、bridge dry-run、watchdog、限幅、返航和降落状态机；
- EGO 原生 mock/SO3 仿真包是否继续作为正式支持范围。

### 尚未完成或未通过工程验收

- EGO 控制 PX4/Gazebo 的稳定空场闭环；
- 静态障碍避让、持续重规划、不可达目标和故障注入验收；
- 可靠的动态障碍跟踪、预测和动态避障；
- 感知包与当前规划/任务主链的正式接入；
- 多机命名空间、任务分配、轨迹共享、协同感知和机间互避；
- 真机外参、时间同步、安全策略和飞行验证；
- QGIS/Cloud 数据模型、协议、网关、鉴权、缓存和传输实现；
- 干净 clone 构建、依赖锁定、CI、install-space 和长期稳定性验收。

## 7. 环境、依赖与构建

审查机器的已观察环境：Ubuntu 20.04.6、ROS Noetic、Gazebo Classic 11.15.1、MAVROS 1.20.1、CMake 3.16.3、GCC 9.4、Python 3.8。它描述当前机器，不代表项目已正式锁定所有版本。

外部 PX4 默认位于 `$HOME/PX4-Autopilot`，审查时为 `v1.15.4-dirty`、detached HEAD，并有 Gazebo 子模块及自定义 launch 改动。该目录不在本仓库中，禁止自动清理、切换版本或修改。`PX4_AUTOPILOT_ROOT` 可供 Stage 6 脚本覆盖默认位置。

两个 catkin 工作空间应按下层仿真、上层主工作区的顺序处理：

```bash
source /opt/ros/noetic/setup.bash

cd /home/yanzu/AstraDroneOpen/simulation/sim_workspace
catkin_make
source devel/setup.bash

cd /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash --extend
```

只修改单个主工作区包时，可在依赖已构建的前提下使用目标包构建，例如：

```bash
cd /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make --pkg offboard ego_gazebo_bridge -j2
```

相关测试入口：

```bash
catkin_make run_tests_offboard run_tests_ego_gazebo_bridge
catkin_test_results
```

构建会写入 `build/`、`devel/`；只读审查不得运行。不要编辑这些生成目录，也不要用已有二进制代替当前源码验证。封装的 `.bin` 构建器可能删除旧构建结果，安装器和 `env_sh` 会修改系统环境；仅在用户明确授权且已说明影响时使用。

## 8. 常用仿真启动流程

### Stage 6 默认安全入口

```bash
cd /home/yanzu/AstraDroneOpen
./scripts/run_sh/stage6_planner.sh
```

默认依次启动 PX4/Gazebo/MAVROS、FAST-LIO、EGO、bridge 和 RViz，但 bridge 为 dry-run，不解锁且不发布 MAVROS setpoint。只有完成起飞前检查并获得明确授权后才可使用 `--control`。脚本即使使用 `--headless` 仍要求 `nvidia-smi`，并对 NVIDIA 驱动版本有本机策略限制；这不是通用平台支持证明。

### 分终端定位问题

1. 启动 `astra_example.launch`，默认机型为 `iris_mid360`；
2. 确认 `/mavros/state`、`/livox/lidar`、`/livox/imu`；
3. 启动 `fast_lio mapping_mid360.launch rviz:=false`；
4. 确认 `/Odometry`、`/cloud_registered` 和 TF；
5. 启动 `ego_gazebo_bridge stage6_gazebo.launch enable_control:=false`；
6. 检查 EGO 目标、B 样条、PositionCommand 和 dry-run 调试输出；
7. 未完成坐标、输入时效、目标包络和唯一控制权检查前不得解锁。

基础多航点和轨迹 launch 会自动尝试 OFFBOARD/解锁，只允许在 `/use_sim_time=true` 的 SITL/Gazebo 中运行。同一时刻只能有一个 MAVROS setpoint 发布者；不要同时运行 `autoarming_control`、`position_control` 和 `ego_mavros_bridge --control`。

## 9. 已知差异和风险

- 根 README 和 `docs/` 中有 ROS2、实机、探索、蜂群、Fast-Planner、FreeDOM、`uav_bridge` 等当前代码不支持或不存在的描述；
- 历史阶段教程已从当前工作区删除，只能作为 Git 历史参考，不能列为当前入口；
- `stage3_waypoints.yaml` 第三个动作同时改变高度，其“原地转向”注释不准确；
- 旧文档中的强制上锁、AUTO.LAND 和 Stage 6 架构说明与当前代码存在冲突；
- `autoarming_control.launch` 和 `stage4_trajectory.launch` 默认软降落参数包含超出当前 PX4 声明范围的值；
- bridge 的规划/MAVROS 对齐检查发生在起飞后，dry-run 检查范围不足；
- bridge 只检查一个 position setpoint Topic，未覆盖 raw、velocity、attitude/thrust 等全部控制接口；
- 默认单位 `map -> camera_init` TF、`body` 与 `base_link` 外参尚未形成正式坐标契约；
- `autoarming_control.cpp` 和 `ego_mavros_bridge.cpp` 均超过 1,100 行，状态机、ROS I/O 和安全逻辑耦合；
- EGO 上游核心被直接修改，尚无固定上游版本和可重放 patch 管理；
- `pc_example.sh` 硬编码 `$HOME/AstraDroneOpen`，默认启动旧控制器；Stage 6 脚本也绑定当前平台和 NVIDIA 策略；
- 运行日志和构建产物不能作为源码事实，完整可复现环境尚未建立。

## 10. 开发边界

后续 GPT/Codex 必须遵守：

1. 不得擅自修改系统环境、依赖版本、外部 PX4、第三方源码、Git 历史、分支、远端或 submodule。
2. 修改前先执行并报告当前分支、`git status --short --branch`、相关 diff、目标文件和调用链。
3. 工作区已有改动均视为用户资产；不得擅自恢复、覆盖、清理、暂存、提交或混入任务改动。
4. 不得把 README、注释、文件名、旧二进制或计划当作运行事实；不确定内容统一标记“待确认”。
5. 涉及架构、消息/Topic/TF 契约、依赖、目录迁移、vendor 更新、控制语义或兼容性破坏时，先提交方案、影响文件、风险、验证和回退建议，等待用户决定后再执行。
6. 优先最小范围修改，保护当前已跑通的基础飞行、多航点和轨迹能力；不要顺手大范围重构或统一格式。
7. 不编辑 `build/`、`devel/`、生成消息、日志或缓存。源文件、launch 和配置必须在其源码位置修改。
8. 自动控制代码默认只按仿真处理；不得移除 `/use_sim_time` 保护或把未经验证参数迁移到真机。
9. 同一控制 Topic 只能有一个发布者。涉及解锁、OFFBOARD、降落或控制权的改动必须先给出安全风险。
10. 安装依赖、运行安装器、改变驱动/PX4/ROS 版本、删除构建目录、切换分支、commit 或 push 均需用户明确授权。
11. 修改后必须说明：改了什么、为什么、验证命令、实际结果、未运行项、遗留问题和对已有功能的影响。

## 11. 开始工作前检查清单

- [ ] 读取本文件和与任务直接相关的源码、launch、配置、测试及审计结论；
- [ ] 执行 `git branch --show-current` 和 `git status --short --branch`；
- [ ] 查看相关文件的现有 diff，确认不会覆盖用户改动；
- [ ] 区分目标能力是已验证、代码已存在待验证，还是尚未实现；
- [ ] 画清输入/输出 Topic、消息、frame、服务、参数和唯一控制发布者；
- [ ] 确认任务是否仅限仿真，是否会写构建产物或修改外部环境；
- [ ] 确认最小修改范围、验证方法和回退方式；
- [ ] 若涉及架构、接口、依赖、目录或兼容性，先向用户提交方案并等待决定。

## 12. 任务完成后的交付清单

- [ ] 仅改动获准文件，复核 `git diff --check` 和 `git status`；
- [ ] 列出每个改动文件及其作用，不把用户原有改动算作本次成果；
- [ ] 记录实际执行的编译、测试、launch 解析、静态检查或仿真实验命令；
- [ ] 将结果明确分为“通过、失败、未执行”，不以旧构建或目测替代验证；
- [ ] 若做仿真，记录 world、机型、关键参数、Topic/TF、控制发布者、最终模式和 `armed` 状态；
- [ ] 说明对基础飞行、多航点、轨迹和 Stage 6 链路的回归影响；
- [ ] 列出遗留问题、风险、待确认事项和建议的下一步；
- [ ] 未经用户明确要求，不 commit、不 push、不切分支、不修改 Git 历史。

## 13. 待确认事项

- 正式支持并锁定的 PX4、Gazebo、ROS 包、airframe、GPU/headless 环境和安装方式；
- EGO 原生 mock/SO3 模拟器是否属于正式保留范围；
- `PositionCommand` 应通过 MAVROS raw setpoint 保留位置/速度/加速度，还是引入独立轨迹跟踪器；
- `map`、`camera_init`、`body`、`base_link` 的正式 TF/外参和原点/航向契约；
- 受支持的降落参数、故障终态和完整控制权检查范围；
- Stage 6 的空场、静态障碍、不可达目标、断流和长期稳定性验收矩阵；
- 首个正式相机/检测方案、消息字段、时间同步和坐标语义；
- 多机数量、命名空间、通信、任务分配、轨迹共享及是否引入 EGO-Swarm；
- QGIS/Cloud 的数据模型、坐标参考系、协议、频率、图像策略、鉴权和离线缓存；
- CI、依赖锁定、容器/镜像、install-space 和正式版本发布规则。
