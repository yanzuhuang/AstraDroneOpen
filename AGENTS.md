# AstraDroneOpen 项目核心记忆与 AI 协作入口

> 最近事实审计：2026-08-17
>
> 审计分支：`scene01-3uav-circuit-mission`
>
> 审计 HEAD：`bdeff9cd613482bb422bf9c887c1fb3a25828603`

本文件面向进入仓库的 Codex、ChatGPT 和新协作者，记录项目当前架构、已验证能力、研究边界与长期规则。它不是 changelog，也不替代源码、运行配置、`rule.md` 或实验工件。开始任务时必须重新检查 Git 状态；上面的分支和 HEAD 只是最近一次审计快照。

## 1. 如何判断项目事实

发生冲突时按以下原则处理：

1. 当前源码、`package.xml`、`CMakeLists.txt`、launch、YAML、消息定义和测试决定“代码现在做什么”。
2. 当前 Git 分支、HEAD、工作区 diff 和 `runtime_artifacts/` 中可复核的运行结果决定“本次工作基于什么、实际验证到哪里”。
3. `rule.md` 是八扇区、航点候选、ENTRY/EXIT、净空与重定位语义的设计基准；若实现与它不一致，应报告冲突并做专项修复，不能自行选择一套新语义。
4. 根目录专题报告记录相应功能的审计与验收边界；优先阅读报告最后的当前结论，不能把同一报告中的历史失败段落误当最终状态。
5. `ego_planner_工程落地学习.md`、`legacy_ego_integration.md`、教程和论文笔记包含历史路线与学习材料，不代表当前完成度。

“文件存在”“能编译”“dry-run 通过”“真实飞行通过”是不同证据等级。没有运行证据时写“待验证”，不要推断为已完成。

### 当前工作区特别说明

2026-08-17 审计时，分支相对远端领先 5，工作区不是干净状态。已有用户修改集中在：

- `learning_speed_rl` 的 Observation C、时间戳、训练数据合同与人工标定链；
- 单机固定速度人工标定 launch/脚本及汇总工具；
- `dual_tower_inspection.launch`、`triple_tower_inspection.launch` 的场景参数化/冲突残余修复；
- `FAST_LIO/Log/mat_pre.txt` 运行写入；
- 一个用户未跟踪说明文档。

这些都属于用户资产。不得恢复、覆盖、清理、暂存或提交。尤其不能擅自还原 `FAST_LIO/Log/mat_pre.txt`。

## 2. 项目简介与当前研究位置

AstraDroneOpen 是一个以 ROS1 为核心的无人机自主巡检研究工程。当前主要在 Gazebo Classic + PX4 SITL 中研究：

- Livox Mid-360 + FAST-LIO 定位与注册点云；
- EGO-Swarm 局部规划、静态避障、在线重规划和多机轨迹避碰；
- 任务层八扇区绕塔、动态进场、候选航点、安全返航与受控降落；
- 三机任务协调、状态/轨迹共享、机间安全和可视化；
- D435 RGB-D 仿真接口与独立 PPE YOLO 感知；
- Learning Speed：在不改变任务目标和安全边界的前提下，研究 EGO 最大速度约束的自适应。

当前工程阶段可概括为：**三机低空绕塔工程基线已有真实 SITL 闭环证据；Learning Speed 的 Stage 1 Reward、0.1 s causal transition、Episode identity、training-only Hector execution/truth odometry/Mid360 Observation C/worksite teleport reset 与 SAC training loop 均已集成。** 首次 10k pilot 在 4207 valid transitions 因真实 `NO_FEASIBLE_TRAJECTORY` 与 action exploration instability fail-closed；随后完成 SAC 根因审计和 2500-transition 短程资格验证，当前 exploration stability 配置已通过。新的第一轮正式 10k 配置与 worksite 随机 XY reset 已完成代码/config/静态测试整理，但尚未启动训练或 evaluation；旧 pilot 不允许 resume，也不能把短程 PASS 写成收敛或长期训练安全。Training backend 与 FAST-LIO + PX4/MAVROS full-stack 保持 launch/process/publisher 互斥；后者仍用于后续高保真验证。真机、长程 SAC 收敛、Stage 2、动态障碍预测和三机视觉巡检业务闭环仍未完成。

核心任务、规划、安全和 Learning Speed 接口不得写死 Gazebo API；仿真假设和真机契约必须分开描述。

## 3. 已确认技术栈与版本边界

当前验证环境：

| 组件 | 当前事实 |
|---|---|
| OS | Ubuntu 20.04.6 LTS |
| ROS | ROS1 Noetic |
| 仿真 | Gazebo Classic 11.15.1 |
| 飞控链 | PX4 SITL + MAVROS 1.20.1 + OFFBOARD |
| 定位/点云 | FAST-LIO + Livox Mid-360 仿真插件 |
| 规划 | 项目适配的 EGO-Swarm 核心；包/节点仍沿用 `ego_planner` / `ego_planner_node` 名称 |
| 相机 | 三机各一套 D435 RGB-D 仿真接口，默认启用 |
| 视觉检测 | ROS1 `yolo_detect` + Ultralytics YOLO，模型权重由外部路径显式提供 |
| Learning Speed | Python/ROS + PyTorch SAC；动作仅为 EGO 动态 `v_max`，training-only 与 full-stack backend 互斥 |
| 构建 | catkin、CMake 3.16.3、GCC 9.4、Python 3.8 |

外部 PX4 位于 `/home/yanzu/PX4-Autopilot`，当前为 detached `99c40407ffd7ac184e2d7b4b293f36f10fe561ef`、`v1.15.4-dirty`。仓库只保存定制 PX4/Gazebo 资产，不包含完整 PX4 源码。未经项目负责人批准，不得修改、清理、切换、升级外部 PX4，也不得升级 ROS、MAVROS、Gazebo、EGO-Swarm、FAST-LIO 或第三方依赖。

## 4. 重要目录与职责

| 路径 | 当前职责 |
|---|---|
| `AstraDrone_ros1_ws/` | 主 ROS1 catkin 工作空间；`build/`、`devel/` 是生成物，禁止手改 |
| `AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/` | 单机巡塔任务层：八扇区、候选、ENTRY/EXIT、多层、HOLD/重试/返航与任务证据 |
| `AstraDrone_ros1_ws/src/MissionControl/ego_gazebo_bridge/` | `PositionCommand` 到 MAVROS raw-local 的安全执行桥、控制权检查和飞行状态机 |
| `AstraDrone_ros1_ws/src/MissionControl/hector_ego_training_backend/` | 独立 training-only Hector 执行、Gazebo truth odometry、truth cloud registration、worksite Episode/teleport reset 与 SAC launch；默认基础入口不启动控制 |
| `AstraDrone_ros1_ws/src/Planner/ego-planner/` | 当前 EGO-Swarm vendor-derived 规划核心、B 样条、traj_server 和共享轨迹接口 |
| `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/` | Mid-360 激光惯性里程计、注册点云和 `camera_init -> body` |
| `AstraDrone_ros1_ws/src/Swarm/` | 三机 bringup、manager、safety、TF、感知过滤和多机消息 |
| `AstraDrone_ros1_ws/src/learning_speed_rl/` | Observation v1/v2/C、speed adapter、Reward/transition、AstraDroneEnv、SAC/replay/checkpoint 与推理边界 |
| `AstraDrone_ros1_ws/src/Detection/yolo_detect/` | 参数化、只读的单机/三机 PPE YOLO 推理节点 |
| `AstraDrone_ros1_ws/src/Utils/astra_custom_msgs/` | 项目消息，包括 YOLO 检测消息；不是完整 Cloud/QGIS 业务接口 |
| `simulation/sim_workspace/` | 下层传感器/Gazebo 插件工作空间，含 Mid-360 与 RealSense 插件 |
| `simulation/px4_sim_files/` | PX4 SITL airframe、SDF 和 launch 部署资产 |
| `simulation/astra_gazebo_worlds/` | `worksite.world`、`outdoor_village.world`、Learning Speed 验证场景等 |
| `simulation/astra_gazebo_models/` | 仿真模型与 PPE 人物/平台资源 |
| `scripts/run_sh/` | 面向操作者的 bringup、实验和录制入口；默认入口应无控制或显式要求 `--control` |
| `runtime_artifacts/` | 唯一运行产物根目录，永不提交 Git |
| `docs/`、根目录专题报告、`essay/` | 使用说明、验收报告和论文学习资料；详细事实需回到源码/工件复核 |

`Detection/`、`Control/`、`Land/`、`Track/` 中除上表明确进入当前主链的包外，仍有历史、候选或独立模块；目录存在不表示已接入三机主任务。

根目录当前没有 `README.md`；项目入口应使用本文件，专题说明则查看 `docs/`、各 ROS 包 README 和根目录报告。不要继续引用历史根 README 中超前的 ROS2、真机或完整蜂群描述。

## 5. 当前主链与坐标契约

每架无人机的主执行链是：

```text
Gazebo iris_mid360_d435
  -> /uavN/livox/lidar + /uavN/livox/imu
  -> FAST-LIO raw odom/cloud
  -> frame adapter + teammate/self cloud filter
  -> /uavN/Odometry + /uavN/stage3/cloud_registered_filtered
  -> 本机 EGO-Swarm -> /uavN/planning/bspline
  -> traj_server -> /uavN/planning/pos_cmd
  -> ego_gazebo_bridge -> /uavN/mavros/setpoint_raw/local
  -> MAVROS -> PX4 OFFBOARD -> Gazebo
```

三机横向链：本机 EGO-Swarm 把带绝对时间的 B 样条转换到公共 `world` 后共享；各规划器消费队友轨迹做时空避碰。`astra_swarm_manager` 负责联合进场、角色放行、相位与退出/落地许可，`astra_swarm_safety` 负责心跳、定位、状态时效和机间安全门。队友点云过滤用于避免机体残影成为静态障碍，不替代共享轨迹避碰。

坐标约定：

- `world`：三机公共坐标、共享轨迹、协调、安全和 RViz；
- `uavN/map`：第 N 架机 MAVROS/任务局部参考，通过出生点平移接入 `world`；
- `uavN/camera_init`：FAST-LIO/EGO-Swarm 规划 frame；仿真中与本机 `map` 使用已验证的单位对齐假设；
- `uavN/body`、`base_link`、`mid360_link`、D435 optical frames：机体/传感器分支。

仿真静态 TF 不是实测外参，不能直接写成真机已标定。EGO 地图回调不替输入点云自动做任意 TF 修正，frame/stamp 错误必须 fail closed。

D435 不参与当前 Mid-360 → FAST-LIO → EGO-Swarm 规划链。YOLO 只消费 D435 彩色图像并发布检测消息，不控制任务、规划或飞行。

## 6. 已完成并有验证证据的核心能力

### 6.1 基础单机飞行与 EGO 执行

- PX4 SITL、Gazebo、MAVROS、Mid-360、FAST-LIO 基础链已经多轮运行。
- 既有 offboard 能力覆盖起飞、悬停、多航点/连续轨迹、yaw、返航、OFFBOARD 受控降落和解除武装。
- EGO-Swarm 轨迹经 `traj_server` 输出 p/v/a/yaw/yaw-rate，再由 bridge 以 raw-local `PositionTarget` 执行；bridge 默认关闭控制并检查五类 MAVROS 控制出口，任一时刻只允许一个控制发布者。
- 单机巡塔任务已具备八扇区、正式闭环、候选换点、EGO 不可达有限重试、HOLD、ENTRY/EXIT、逆进场返航和明确任务终态。

### 6.2 八扇区、低空避障和多高度

- 八个固定扇区中心为 `0/45/90/135/180/225/270/315°`，覆盖/编号不随入口角改变。
- `worksite.world` 单机低空 EGO 巡塔、候选净空和安全返航已在当前实验链重复运行；Learning Speed 两环境固定速度矩阵中的 6 次 worksite 正式 run 均完成 mission success。
- 高空任务代码支持多层巡塔、层间分段过渡和高度候选；Git 历史已有“两层绕塔”及 `34→30→26→22 m` 高度参数化验证。它不是当前三机默认配置，修改后需要单独回归；连续螺旋不是已验收能力。
- 当前三机默认是 **3.0 m 等高、单层、`height_offsets=[0]`**，不得把高空下降候选带入低空任务。

### 6.3 三机低空绕塔

三机 `worksite.world` 最终 SITL 闭环已经通过：

- UAV3 leader、UAV2 middle、UAV1 trailing，逆时针；
- 动态 `PRE_ENTRY -> ENTRY_GATE -> ORBIT_STAGING` 联合候选；
- UAV3 先放行，随后按前机领先 `65°～70°` 依次放行 UAV2、UAV1，目标相邻相位为 `67.5°`；
- 三机均完成独立 360°、8 扇区、独立 EXIT_GATE、实际进场路线逆序且按最新地图重规划、HOME、落地与解除武装；
- 最终三机均为 `armed=false`、`ON_GROUND`、bridge/task/flight `DONE`；全任务实测最小机间距离高于 3.0 m 门限。

详细最终结论见 `ego_swarm_three_uav_integration_report.md` 最后一节。该报告前部保留了修复前失败过程，不能据此否定后部最终 PASS，也不能把历史 `test_evidence/stage*` 路径继续用于新实验。

### 6.4 三机传感器、场景与可视化

- 三机模型均有独立 PX4/MAVROS、Mid-360、D435、FAST-LIO、TF、局部 EGO-Swarm 和命名空间隔离。
- `triple_tower.rviz` 能显示三机姿态、点云、占据地图、目标、轨迹、任务/编队/安全状态；Gazebo GUI 与 RViz 可同时启动，但会竞争 GPU/CPU，GUI FPS 不等于仿真 RTF。
- `outdoor_village.world` 已有三机**初始化专用**入口：保留 Gazebo/PX4/MAVROS、传感器、FAST-LIO、EGO 和 RViz，硬关闭任务/控制；已验证三机连接、在地面未解锁且无 raw setpoint publisher。
- `outdoor_village.world` 也已用于 UAV1 Learning Speed 环境 A 的 6 次正式飞行；这不等于“三机 outdoor_village 巡塔闭环已验收”。

### 6.5 YOLO 接口

- `yolo_detect` 已进入 catkin，支持三架机各自订阅 `/uavN/d435/color/image_raw`，发布 `/uavN/yolo/detections` 和可选标注图像。
- 三路节点已验证模型加载、CPU/GPU 推理、命名空间隔离和结构化 `AstraDetection2DArray` 输出；节点是只读感知模块，不依赖任务状态机。
- 权重不随仓库交付，必须显式传入并包含预期 PPE 类别。
- 尚未完成“三机稳定绕塔过程中产生非空违规检测并形成业务结果”的联合验收。因此 YOLO 接口完成，不等于视觉巡检任务闭环完成。

## 7. 任务层与 EGO-Swarm 的关键设计决策

### 7.1 八扇区与候选选择

任务层决定“去哪一个正式目标”，EGO-Swarm 决定“怎样安全到达”。普通扇区和第一巡塔点共用候选网格：名义半径 `12.5 m`，再考虑 `14.5/16.5 m` 外圈与同扇区角度偏移；低空高度不变。

选择顺序是硬约束优先：

```text
扇区/高度包络、塔体 keep-out、地图/粗几何净空、方向进度
  -> 第一个仍有安全候选的半径层
  -> 保持仍安全的锁定候选
  -> 若中心角名义点安全，直接选名义点
  -> 否则只在同一半径层按评分选候选
  -> EGO 不可达时有限重试，再切同扇区下一 candidate ID
  -> 候选耗尽才 fail closed
```

外圈高分点不能替代安全的内圈名义点；评分不能绕过硬约束。锁存用于防止地图抖动导致逐帧跳点。

### 7.2 净空与安全优先级

- 已膨胀 EGO occupied map：任务层只加 `map_additional_clearance=0.5 m`；
- 原始/过滤点云和已知粗几何：使用 `minimum_clearance=1.0 m`，只应用一次；
- 塔体 keep-out、frame/stamp/freshness、任务包络、控制权和机间距离都是硬门；
- EGO `obstacles_inflation`、优化器 `dist0`、任务层 additional clearance 与 `swarm_clearance` 含义不同，禁止叠加、互换或把软代价写成硬保证；
- 三机最小 3D 距离为 `3.0 m`，EGO `swarm_clearance=1.5 m`；不能为通过实验而降低。

具体公式、边界和重试语义以 `rule.md` 为准。

### 7.3 ENTRY_GATE / EXIT_GATE

- `PRE_ENTRY` 是进入正式任务通道前的外围参考点；
- `ENTRY_GATE` 是正式入口与三机联合放行锚点；
- `ORBIT_STAGING` 是第一个普通扇区候选，仍须走统一候选/EGO 状态机；
- `EXIT_GATE` 提供独立、已检查的离场锚点；正常返航沿本机真实进场采样反向、用最新地图重新规划，再回 HOME；
- 任一地图、定位、轨迹、协调或安全门失败时优先 HOLD、有限重试、返航或受控降落，不能包装为任务成功。

### 7.4 EGO-Swarm 核心边界

当前规划核心不是未修改的单机 EGO-Planner：仓库已经迁入 EGO-Swarm 的共享 B 样条/机间优化能力，并增加项目 frame 适配、状态接口和动态 `v_max` 接口。稳定搜索、碰撞、B 样条优化和重规划逻辑属于受控 vendor-derived 核心：

- 新任务逻辑、候选策略、业务状态优先放在任务层/适配层；
- 不为某次实验随意修改成熟 EGO 核心、关闭安全检查或改 world 迎合算法；
- 确需修改核心时，先说明必要性、接口边界和回归范围，保持改动最小且可审计。

## 8. Learning Speed / 强化学习当前状态

### 8.1 它负责什么

Learning Speed 只研究一个受限动作：EGO 的最大速度约束 `v_max`。动作链为：

```text
fixed/mock request
  -> SpeedSafetyFilter（finite validation + reviewed min/max clamp）
  -> /uavN/learning_speed/v_max
  -> EGO 动态速度上限；相邻 action delta 超出 [-0.3,+0.5] m/s 时额外强制一次重规划
  -> /uavN/learning_speed/applied_v_max 回执
```

合法范围内 action 不再做 slew、low-pass、hysteresis 或 maximum-step 动态
整形，因此正常语义是 `requested_v_max == filtered_v_max == applied_v_max`。
区间内未触发 Learning Speed 强制重规划时，EGO 保持自身原生 replanning
rules；旧的 replan delta、cooldown 和 filtered-speed 累计机制已删除。

它不选择 waypoint，不生成轨迹，不负责碰撞检查、三机协调、MAVROS/PX4 控制或安全状态机。默认关闭；启用后也只能在 launch 审核过的静态上限内降低/恢复速度。

### 8.2 Observation 状态

- Observation v1：保留 `[4,16,48,48]` 地图张量 + 22 维低维向量合同，但当前 EGO 导出不能完整区分 free/unknown，完整轨迹通道也不足，所以 `observation_ready=false`。
- Observation v2：Mid-360 球面 surrogate，80×40=3200 bins，区分 obstacle/free/unknown；full-stack 使用 FAST-LIO pose history，training 使用 `gazebo_truth_training` causal pose history。
- Observation C：把原子 v2 surrogate、EGO 官方 `/planning/bspline` 未来位置、时间对齐的实际速度、tracking error 和 previous applied `v_max` 融合到同一 body frame/stamp。五项值合同不随 backend 改变；source metadata 必须真实，任何缺失、越界、frame/stamp 不一致或未来数据都会 fail closed。

Observation C 的五项 policy input 已冻结为：

1. `lidar_surrogate[3200]`；
2. `future_positions_body[20][3]`；
3. `actual_velocity_body[3]`；
4. `tracking_error_body[3]`；
5. `previous_applied_v_max`。

mission/planner 状态、clearance、clutter/density、lidar masks 和 diagnostics 只作审计，不进入 policy input。

### 8.3 Training Data Contract

`learning_speed_sac_transition_v1.3` 已冻结：

```text
state_t -> requested_v_max -> filtered_v_max -> applied_v_max
        -> state_t+1 -> reward -> terminated/truncated
```

五项 policy input 保持不变。新增的
`learning_speed_progress_reward_context_v1.0` 只作 reward context：在
`state_t/state_t+1` 各自记录 headerless `/tower_mission/progress`、mission
state 及两者 receipt ROS time，并记录 run/episode provenance、`P_t`、
`P_t_plus_1` 和 `Delta_P`；它们不进入 Observation C 或 policy input。
采集器检查 Observation C、正式 B 样条、action、applied acknowledgement
和 progress context 的因果顺序；真实任务/规划/安全失败必须保留，只有基础设施失败允许重试。当前 recorder 对 valid、同 episode、非 truncated 候选在线写入：

```text
reward = finite Stage 1 r_t
reward_defined = true
training_ready = true
```

`astradrone_stage1_reward_v1.0` 仍只在 `training/reward.py` 实现，并由现有 `config/stage1_reward.yaml` 选择 `reward.mode=stage1`；recorder 只把与 offline replay 同语义的 runtime signals 构造成 `Stage1RewardInput` 并调用 `Stage1Reward.evaluate()`，没有第二套公式、Reward node/service/topic。v1.3 contract 只允许 valid、同 episode、非 truncated transition 绑定 versioned 分项 reward；invalid/truncated 仍不得变成 training-ready。输出保留 `reward_total/reward_speed/reward_smoothing/reward_danger/phi_1/phi_2/complexity_context`，progress context 仍不进入 Stage 1 reward。历史冻结 calibration 工件保持 reward-null，不回写。该 online 路径已通过 unit/offline 一致性验证。

Stage 1 使用 Candidate C 的连续 N+D+Unknown `phi_2` 与 `phi_1=1.75-phi_2`（Unknown 固定 `phi_1=1.25, phi_2=0.5`），并连续混合论文 Eq. (10) 三个分支；最终项目标定值为 `lambda_phi_1/2=0.65/0.35`、`lambda_speed_1/2/3=1.00/0.80/0.25`、`lambda_smoothing=0.10`、`lambda_danger=2.00`。这些都是 AstraDroneOpen-specific，不是论文原参数。Reward 不含 tracking/progress；danger 只在冻结的 collision proxy / emergency / continuous tracking-safety terminal 上按当前实际速度平方产生。现有 tracking gate 不变。

仓库已有 training-only Gaussian Actor、twin Q/target Q、automatic entropy、Replay Buffer、异步 learner、checkpoint 与 qualification/pilot/evaluation runner；没有 PER、已收敛模型、生产 inference 策略或 Stage 2。`inference/model_runner.py` 仍只是 reviewed model 的 fail-closed 边界。

`training/astra_drone_env.py` 的正式入口是 `run_episode()`：按 0.1 s ROS/sim-time grid 发布 request，不等待上一 transition 闭合；每个 pending step 独立保存 episode/step ID、strict causal state、request marker、atomic action、applied ack、trajectory provenance 与 post-hold next Observation，并一次性消费 action/applied/next-state event。full-stack readiness 使用 mission/bridge/planner；training profile 使用 Hector coordinator/adapter identity。`AstraDroneEnv.reset()` 仍不直接拥有 reset，正式 training reset 由外部 coordinator 独占。

SpeedAdapter 的 Episode 专用 `mock_request_driven=true` 模式禁用独立 mock timer；每个 `SpeedRequestStamped` 立即经过同一 `MockSpeedPolicy -> SpeedSafetyFilter -> SpeedActionStamped -> EGO` 核心。request、action 和 `SpeedAppliedStamped` 都携带相同 `episode_id/step_index/request_id`；EGO 直接消费 stamped action，scalar `v_max/applied_v_max` 只保留为状态镜像，不构成第二套正式 pairing。100 Hz repeated-value burst rostest 为 ID 1..100 的 100/100 exact FIFO action。最终 UAV1 runtime 为 100 scheduled request、100 action、100 applied ack、100 transition，10.0 Hz，median/p95 均约 0.100 s，scheduler 0 deadline miss/drop、0 timeout、0 causal mismatch，Episode window Observation C 102/102 valid（bag 宽窗口 107/107）、0 collision/emergency，100/100 finite reward；`max_episode_steps` 正确形成 `terminated=false, truncated=true`。该次 full mission 在 Episode 完成后因 final-home `NO_FEASIBLE_TRAJECTORY` 进入 mission failure landing，最终 disarmed/ON_GROUND；失败保持为 Episode-window 外的真实结果，不覆盖 identity/causal PASS，也不写成 full-mission PASS。

Progress reward context 已用独立 instrumentation run
`progress_ctx_A_v125_r01`（Environment A、1.25 m/s）完成一次完整闭环验证：
原始 progress 为 `0 -> 1` 且无下降，waypoint `8 -> 1` 时不 reset；452
条 transition 的 progress/state receipt、run/episode provenance、
`P_t/P_t_plus_1/Delta_P` 因果检查全通过，负 `Delta_P` 为 0；EXIT/return/
landing 保持 progress=1。该 run 不属于、也未修改当前 16-cell calibration
matrix。原报告已合并到 `AstraDroneOpen_项目技术演进与LearningSpeed阶段汇总.md`。

### 8.4 Environment A/B 与数据质量

已完成 UAV1 固定速度人工标定：每个环境使用固定且各自一致的路线，速度为 `0.30/0.50/0.75/1.00/1.25/1.50 m/s`，共 12 个正式 run：

- Environment A：`outdoor_village.world`，spawn `(-14,0)`，较开放路线；
- Environment B：`worksite.world`，spawn `(0,0)`，较密集塔区路线。

12/12 均有正式 terminal、mission PASS、完整指标、有效 requested/filtered/applied 链和合法 null-reward 边界。首轮 Observation C 单 run valid ratio 为 `0.545～0.834`，高速度下 tracking error 明显变差；少量恢复后的 planner `CANCELLED` 被保留为真实结果。

首轮低 valid ratio 的两项根因已经审计：重规划后的旧 lidar 需要按 source stamp 选择因果轨迹历史；大 sim time 下 ROS `sec/nsec -> float -> sec/nsec` 会损失 1 ns。0.05 s 同步门限未放宽。后续已经完成独立的 14 条件 post-fix 矩阵（A/B，`0.30–1.75 m/s`，统一 `2.00 m/s` ceiling）：A 七档均成功；B 到 1.50 成功，B/1.75 保留为真实 mission failure。该代际与首轮 `1.50 m/s` ceiling 数据保持隔离，不得混池。

高速 qualification 独立代际使用 EGO static `max_vel=4.0 m/s`、`max_acc=3.0 m/s²`、`feasibility_tolerance=0.0` 和 `planning_horizon=7.5 m`，UAV1 专用 bridge 以三维范数包络覆盖 EGO 逐轴约束。Environment B progressive qualification 已按每档一次、真实 failure 即停执行：1.75 和 2.0 m/s PASS；2.5 m/s 虽最终 mission success、disarmed、ON_GROUND，但过程中触发 `CURRENT_POSITION_IN_OCCUPANCY` 并锁存 collision/dangerous proxy，qualification FAIL；3.0/3.5 未运行。当前最高稳定 qualification 速度为 2.0 m/s。三档 unexplained invalid、trajectory timestamp mismatch 和 kinematic gap 均为 0，但 2.5 training-active valid ratio 降至约 0.836。最终 bag/ULog 速度链审计确认三档无 bridge velocity saturation，raw→PX4 input 原生时间戳对齐误差低于 `1.2e-7 m/s`，2.5 failure 前也无 acceleration saturation；configured 高于 sustained actual 来自轨迹/转弯/加减速与跟踪，而不是隐藏速度 ceiling。PX4 Z 速度 `3.0/1.5 m/s` 仍不构成完整 3D 4.0 m/s 合同。

权威工件：原 Markdown 结论与删除前 hash 已合并到
`AstraDroneOpen_项目技术演进与LearningSpeed阶段汇总.md`；非 Markdown 原始分析继续
保留为 `runtime_artifacts/learning_speed/calibration/manual_calibration_analysis.json`。

### 8.5 强化学习 / Learning Speed Training

研究目标仍是只用一维 SAC action 自适应 EGO `v_max`，不让 policy 选择 waypoint、生成轨迹或直接控制 Hector/PX4。当前 training-only 正式链是：

```text
worksite.world + Hector UAV
  -> Gazebo truth odometry (`gazebo_truth_training`, world/base_link)
  + simulated Mid360 PointCloud2 (10 Hz, mid360_link)
  -> truth-pose five-frame alignment -> frozen 3200 surrogate
  -> Observation C 3267
  -> SAC normalized action -> v_max [0.05, 0.40] m/s
  -> SpeedSafetyFilter -> stamped EGO dynamic v_max
  -> EGO -> traj_server -> Hector Pose/Twist controllers
  -> terminal -> controller stop/pause/teleport zero-twist/start/engage
  -> generation barrier + five-frame warm-up -> next Episode
```

Training backend 不启动 FAST-LIO、PX4、MAVROS 或 EgoMavrosBridge。Full-stack 仍是 `Mid360 -> FAST-LIO -> EGO -> bridge -> MAVROS/PX4`，用于后续高保真验证；两条 `/uav1/Odometry`/execution path 必须通过 launch、process 和 publisher 互斥，不能同时运行。

已完成里程碑：Hector trajectory backend；Gazebo truth 替代 training FAST-LIO state；Mid360 + truth pose 的五帧 3200 surrogate 与 Observation C 3267；正式 worksite Episode/teleport reset；SAC actor/critic/replay/checkpoint training-loop integration；一次 fail-closed 10k pilot；随后 action exploration stability 根因修复与短程 PASS。详细历史统一见根目录 `AstraDroneOpen_项目技术演进与LearningSpeed阶段汇总.md`。

当前正式 SAC 配置是 `learning_speed_rl/config/sac_training_v1.yaml`：第一轮目标 `10000` 个 valid active-Episode transitions，从空 Replay 开始；Replay Buffer logical capacity `100000`，`learning_starts=1000`，batch `64`，Actor/Critic LR `1e-5/1e-3`，alpha LR `1e-3`，100 次 critic-only startup update，log-std `[-3,-1]`，automatic alpha/target entropy `-1`，action/v_max `[0.05,0.40] m/s`，SAC seed `1`。`valid_transition_count == 10000` 是唯一正常 training 完成条件；固定 `episode_count` 在 training 模式禁用，Episode 提前 success/failure/truncated 后继续 reset 和累计，直到精确 10000，且不得生成第 10001 条 Replay experience。只在 step `5000` 和 `10000` 保存 checkpoint；training 中不运行 evaluation。训练后用独立 `evaluation` mode 加载指定 checkpoint，使用固定 Episode 数、deterministic Actor、无网络更新、无 training Replay。正式 launch 默认加载该配置；`sac_training_smoke.yaml` 只保留旧 integration 基线。

正式 training reset 使用 `hector_ego_training_backend/config/worksite_training_reset.yaml`：nominal Hover `(0,0,3)`；Episode 1 使用 nominal spawn，之后每次 reset 在 `x/y offset=[-1,+1] m` 的完整安全矩形内按独立 seed `1001` 可复现采样，`z=3.0 m`、yaw `0` 固定，ENTRY_GATE 仍为 `(-4.3148485145,5.8522070123,3)`。candidate 经过配置化 bounds 与 worksite 静态障碍净空验证，最多 32 次，耗尽即 fail closed；每个 Episode/reset 记录 candidate、seed、sample/attempt、validation，并继续经过 adapter target acknowledgement、zero-twist teleport、controller stop/start、Observation temporal clear、generation barrier、五帧 Mid360 warm-up和 fresh EGO trajectory。Evaluation 明确关闭随机 reset，使用 fixed nominal Hover。上述新 reset/10k 行为只有代码与静态/单测证据，尚无 runtime PASS。

10k pilot 在 4207 valid transitions fail-closed，旧 checkpoint/replay 不允许恢复继续。修复后的 qualification 覆盖 2500 transitions、1500 stochastic steps、5 Episodes，action boundary/delta force-replan、planner/collision、reset、identity、10 Hz、replay、loss/Q/gradient/checkpoint 门均通过。该 PASS 只放行**新的、从空 replay 开始的受控训练**；下一步应先看 checkpoint 和 deterministic evaluation，再决定是否扩大规模，不能默认直接进入 100k/1M。training resume 当前未实现；Stage 2、PER、模型部署、full-stack transfer 和真机验证均待完成。

长期规则：不得为训练 PASS 给 SafetyFilter 加 slew/low-pass/hysteresis，不改 `[0.05,0.40]`、Learning Speed replan 阈值、Reward、Observation C、EGO core 或 Hector PID；真实 planner/collision/tracking failure 必须保留。SAC exploration 参数必须写入 config 并经独立 runtime 资格验证。

## 9. 已实现但仍属部分验证 / 待验证

- YOLO：三路推理接口已通过；三机绕塔中的非空 PPE 检测、覆盖率和业务告警闭环待验证。
- outdoor_village：三机初始化和 UAV1 Learning Speed 飞行已通过；三机任务闭环待验证。
- Observation C：接口与定向数据质量门通过；修复后的完整 12-run A/B 矩阵未重跑。
- Learning Speed：fixed/mock、Reward、causal Episode identity、training-only SAC loop 和 action exploration 短程稳定性均已通过；新的随机 reset 正式 10k 配置为代码/config ready，10k 尚未启动。长程收敛、resume、Stage 2、模型推理、full-stack transfer 与泛化未验证。4.0/3.0 高速代际在 Environment B 验证到 2.0 m/s，2.5 collision-proxy FAIL 后停止。
- Training simulator：Hector execution、truth odometry、Mid360/Observation C、worksite Episode/teleport reset 与 SAC 已集成；这是 lightweight training backend，不等于 PX4/FAST-LIO high-fidelity 或 production reset。
- D435：三机 RGB-D topics/TF 和 YOLO 彩色输入已接通；不参与当前规划，真实硬件外参/同步待验证。
- 动态障碍：没有可靠目标跟踪、未来状态预测和时空动态避障闭环；静态占据更新不能称为动态避障。
- 连续螺旋、QGIS/Cloud、ROS2、真机、集群部署和干净 clone 复现不属于当前已验收能力。

## 10. 主要入口

| 用途 | 入口与安全语义 |
|---|---|
| 三机 worksite 巡塔 | `scripts/run_sh/three_uav_inspection.sh`；默认无控制，自动飞行必须显式 `--control` |
| 三机 outdoor 初始化 | `scripts/run_sh/three_uav_outdoor_village.sh`；专用入口拒绝 `--control`，不得启动任务/飞行 |
| 单机八扇区 | `scripts/run_sh/sector_inspection.sh`；控制需显式授权 |
| 单机 EGO waypoint | `scripts/run_sh/ego_waypoint_inspection.sh`；默认 dry-run |
| 固定航线环塔 | `scripts/run_sh/fixed_orbit_inspection.sh`；默认 preview |
| Learning Speed 固定速度矩阵 | `scripts/run_sh/learning_speed_manual_batch.sh`；真实飞行前检查场景、路线指纹和进程 |
| 单次固定速度标定/qualification | `scripts/run_sh/learning_speed_manual_run.sh`；Environment A/B、速度、ceiling/acceleration 代际为显式参数；高速档仍需 `--control` 与 live preflight |
| Hector training-only backend | `hector_ego_training_backend/launch/hector_ego_training_backend.launch`；默认 `enable_control=false`、`run_qualification=false`，必须显式授权才执行 qualification；不启动 PX4/MAVROS/FAST-LIO/bridge |
| Worksite SAC training | `hector_ego_training_backend/launch/hector_worksite_sac_training.launch`；默认 headless、`runner_mode=training`、累计精确10000 valid transitions、每Episode最多500 step、training固定Episode数禁用、随机 XY reset，正式 config 为 `sac_training_v1.yaml`，必须显式给唯一 `output_dir/run_id`；`max_training_episodes=1000`仅作异常fail-safe；不支持 resume；evaluation 使用同一 launch 的固定Episode数独立只读 mode 与独立 output |
| 三路 YOLO | `AstraDrone_ros1_ws/src/Detection/yolo_detect/launch/ppe_yolo_three_uav.launch`；必须显式给模型路径/Python/设备 |

`three_uav_inspection.sh` 的 `light/full` 录制写入 `runtime_artifacts/`，`none` 不创建正式结果目录。不要仅相信 wrapper 的“started/success”文字；应检查 `roslaunch.log`、`gzserver/gzclient`、MAVROS 状态、任务节点和 setpoint publisher。

两个 catkin 工作空间按 `simulation/sim_workspace` 后 `AstraDrone_ros1_ws` 的顺序构建。安装器和 `.bin` 构建器可能修改系统或清理构建目录，未经明确批准不得运行。

## 11. 长期开发规则

### 11.1 运行产物与命名

- 所有 rosbag、ROS/Gazebo/PX4 日志、CSV/JSON、轨迹图、截图、视频、临时报告、调试输出和验证结果统一写入根目录 `runtime_artifacts/<功能名>_<时间戳>/`。
- `runtime_artifacts/` 已被 `.gitignore` 忽略，任何内容都不得加入 Git。
- 禁止重新创建、使用或向 `test_evidence/`、`trc_picture/` 写入数据；报告里的这些路径只是历史证据。
- 新文件、脚本、目录和运行结果禁止以 `stage1/stage2/stage3/stage5/stageX` 命名，应使用稳定功能名。现存 `/stage3/...`、`/stage5/...` Topic 和历史源码名称属于兼容遗留，未经接口迁移评审不要顺手重命名，也不得据此继续制造新阶段式名称。

### 11.2 修改纪律

- 修改前先理解现有实现、调用链、frame/topic 和已有测试；优先最小、局部、可回归的变化。
- 稳定工作的单机/三机、候选、净空、EGO、bridge、FAST-LIO、D435、YOLO 与 Learning Speed 默认关闭行为不得为新实验随意破坏。
- 任务语义优先在任务层/适配层解决，不轻易修改成熟 EGO-Swarm 核心；不得通过关闭安全门、降低净空/机间距离、放宽 tracking gate 或修改 world 来制造 PASS。
- 正常起飞、任务、返航和受控降落保持 PX4 OFFBOARD；严重断流可由 PX4 failsafe 接管。`MPC_LAND_SPEED` 与 OFFBOARD 下降 setpoint 速度不是同一参数。
- 任意时刻只允许一个 MAVROS 控制出口。无明确控制授权、preflight 或用户许可时，禁止 `--control`、解锁、起飞或飞行。
- 保留 `/use_sim_time` 保护；仿真参数、单位 TF 和外参不得冒充真机配置。
- 不编辑 `build/`、`devel/`、生成消息、缓存或外部 PX4；不升级锁定依赖。
- Codex 不得 commit 或 push；提交由项目负责人完成。

### 11.3 验证与结论

- 明确区分：代码完成、静态检查、构建/单测、dry-run、SITL 飞行、真机验证。
- 实验失败若来自真实 tracking/planner/safety 行为，应保留为结果；只重试基础设施失败，不覆盖原结果。资格验证需标记 `qualification_only=true`，不混入正式计数。
- 比较实验必须保证同环境内路线和配置一致，并记录 obstacle、speed request/filter/applied、planner、terminal、Observation C 和 route fingerprint。
- 不确定内容写“待验证”；一次成功、合成输入或离线 replay 不得扩大解释范围。

## 12. AI / Codex 启动与收尾规则

每次开始任务必须先只读执行并报告：

```bash
git branch --show-current
git rev-parse HEAD
git status --short --branch
git diff -- <相关文件>
```

所有现有修改、删除和未跟踪文件均视为用户资产。若任务涉及 `rule.md` 语义，必须同时核对对应 YAML、实现、测试和专题报告。

每完成一个**真正的新功能或重要架构修改**并完成必要验证后，都必须检查根目录 `AGENTS.md`。若变化影响以下任一当前事实，应同步更新本文件：

- 当前能力或验证等级；
- 架构、数据流、Topic/消息/TF 接口；
- 重要参数或安全契约；
- 目录、启动入口或稳定基线；
- 开发规则或当前研究阶段。

更新时只改“当前事实”，删除失效描述，并链接详细报告；不要追加流水账。普通小 bug、一次性实验、临时参数和每次运行数据无需写入。若功能未完成真实飞行验证，必须保留“代码完成/待飞行验证”的边界。

## 13. 详细资料索引

- `rule.md`：八扇区、候选、净空、ENTRY/EXIT、重试和三机角色规则。
- `studynote.md`：当前三机任务、EGO-Swarm 五层主链、Topic/TF 和 RViz 解释。
- `ego_swarm_three_uav_integration_report.md`：三机动态进场、协同规划、安全和最终闭环验收；以最后的最终结论为准。
- `三机绕塔项目依赖与目录说明.md`：环境依赖、外部 PX4 和三机包职责。
- `docs/05-三机YOLO部署与交接说明.md`：YOLO 接口、模型要求和验证边界；其中三机绕塔失败描述已被后续三机最终 PASS 取代，但 YOLO 非空联合验收仍未完成。
- `learning_speed_adapter_integration_report.md`：动态 `v_max` 接口、默认关闭和 Mock/fixed 验证边界。
- `learning_speed_observation_v2_report.md`：Mid-360 surrogate、时间对齐与 PARTIAL 历史边界。
- `scheme_c_trajectory_fusion_report.md`：Observation C 特征和官方 B 样条融合合同。
- `learning_speed_training_data_contract_report.md`：SAC transition 合同与 null-reward 边界；“尚未在线标定”已被后续 12-run 工件取代。
- `fixed_speed_baseline_report.md`：固定速度基线和 mission completion 记账。
- `high_speed_parameter_chain_update_report.md`：4.0 m/s ceiling、3.0 m/s² acceleration、bridge 逐轴/范数一致性与静态 qualification 边界。
- `high_speed_progressive_qualification_report.md`：Environment B 逐级高速结果、2.5 m/s collision-proxy failure、Observation C/控制链趋势与 Reward 前置结论。
- `final_velocity_execution_chain_audit_report.md`：三档 request→EGO→bridge→PX4→actual 最终审计、隐藏速度限幅排除与 Stage 1 Reward 设计入口结论。
- `AstraDroneOpen_项目技术演进与LearningSpeed阶段汇总.md`：统一吸收原 `runtime_artifacts/**/*.md` 与 progress validation 的 planner/clearance/high-speed/Observation/Reward/Episode/reset/Hector/SAC 历史；含完整删除前 source manifest、PASS/NO-GO 演进和当前正式 10k 边界。原 runtime Markdown 已按该 manifest 合并删除，非 Markdown 原始证据仍保留。
- `clearance_semantics_cleanup_report.md`、`pre_entry_clearance_root_cause_report.md`：净空语义与历史残余清理。
- `worksite_mid360_startup_root_cause_report.md`：worksite terrain collision、Mid-360 和 1/2/3 机启动根因。
- `ego_planner_工程落地学习.md`、`legacy_ego_integration.md`：历史学习路线，仅作背景，不作为当前完成度入口。
