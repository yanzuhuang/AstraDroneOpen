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

当前工程阶段可概括为：**三机低空绕塔工程基线已完成并有真实 SITL 闭环证据；当前活跃研究转向 Learning Speed 的 pre-reward / pre-training 数据与 Observation C 质量闭环。** 真机、正式 SAC 训练、动态障碍预测和三机视觉巡检业务闭环仍未完成。

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
| Learning Speed | Python/ROS 节点 + EGO 动态 `v_max` 小接口；当前只有 fixed/mock source，无 SAC policy |
| 构建 | catkin、CMake 3.16.3、GCC 9.4、Python 3.8 |

外部 PX4 位于 `/home/yanzu/PX4-Autopilot`，当前为 detached `99c40407ffd7ac184e2d7b4b293f36f10fe561ef`、`v1.15.4-dirty`。仓库只保存定制 PX4/Gazebo 资产，不包含完整 PX4 源码。未经项目负责人批准，不得修改、清理、切换、升级外部 PX4，也不得升级 ROS、MAVROS、Gazebo、EGO-Swarm、FAST-LIO 或第三方依赖。

## 4. 重要目录与职责

| 路径 | 当前职责 |
|---|---|
| `AstraDrone_ros1_ws/` | 主 ROS1 catkin 工作空间；`build/`、`devel/` 是生成物，禁止手改 |
| `AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/` | 单机巡塔任务层：八扇区、候选、ENTRY/EXIT、多层、HOLD/重试/返航与任务证据 |
| `AstraDrone_ros1_ws/src/MissionControl/ego_gazebo_bridge/` | `PositionCommand` 到 MAVROS raw-local 的安全执行桥、控制权检查和飞行状态机 |
| `AstraDrone_ros1_ws/src/Planner/ego-planner/` | 当前 EGO-Swarm vendor-derived 规划核心、B 样条、traj_server 和共享轨迹接口 |
| `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/` | Mid-360 激光惯性里程计、注册点云和 `camera_init -> body` |
| `AstraDrone_ros1_ws/src/Swarm/` | 三机 bringup、manager、safety、TF、感知过滤和多机消息 |
| `AstraDrone_ros1_ws/src/learning_speed_rl/` | Observation v1/v2/C、fixed/mock speed adapter、数据合同、标定记录与未来推理边界 |
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
  -> SpeedSafetyFilter（clamp、hysteresis、slew、低通）
  -> /uavN/learning_speed/v_max
  -> EGO 动态速度上限 + 必要时连续重规划
  -> /uavN/learning_speed/applied_v_max 回执
```

它不选择 waypoint，不生成轨迹，不负责碰撞检查、三机协调、MAVROS/PX4 控制或安全状态机。默认关闭；启用后也只能在 launch 审核过的静态上限内降低/恢复速度。

### 8.2 Observation 状态

- Observation v1：保留 `[4,16,48,48]` 地图张量 + 22 维低维向量合同，但当前 EGO 导出不能完整区分 free/unknown，完整轨迹通道也不足，所以 `observation_ready=false`。
- Observation v2：只读 Mid-360 球面 surrogate，80×40=3200 bins，区分 obstacle/free/unknown；使用带时间戳的 FAST-LIO pose 历史，输出不被 policy/EGO/控制链消费。
- Observation C：把原子 v2 surrogate、EGO 官方 `/planning/bspline` 未来位置、时间对齐的 FAST-LIO 实际速度、tracking error 和 previous applied `v_max` 融合到同一 body frame/stamp。任何缺失、越界、frame/stamp 不一致或未来数据都会 fail closed。

Observation C 的五项 policy input 已冻结为：

1. `lidar_surrogate[3200]`；
2. `future_positions_body[20][3]`；
3. `actual_velocity_body[3]`；
4. `tracking_error_body[3]`；
5. `previous_applied_v_max`。

mission/planner 状态、clearance、clutter/density、lidar masks 和 diagnostics 只作审计，不进入 policy input。

### 8.3 Training Data Contract

`learning_speed_sac_transition_v1.0` 已冻结：

```text
state_t -> requested_v_max -> filtered_v_max -> applied_v_max
        -> state_t+1 -> reward -> terminated/truncated
```

采集器检查 Observation C、正式 B 样条、action 和 applied acknowledgement 的因果顺序；真实任务/规划/安全失败必须保留，只有基础设施失败允许重试。当前每条候选仍是：

```text
reward = null
reward_defined = false
training_ready = false
```

仓库没有 SAC actor/critic、replay/PER、训练循环或已选模型；`inference/model_runner.py` 只是未来 reviewed model 的 fail-closed 边界。**Reward stage 1 / stage 2 当前均未定义、未实现、未开始，不得猜测 φ1/φ2 或权重。**

### 8.4 Environment A/B 与数据质量

已完成 UAV1 固定速度人工标定：每个环境使用固定且各自一致的路线，速度为 `0.30/0.50/0.75/1.00/1.25/1.50 m/s`，共 12 个正式 run：

- Environment A：`outdoor_village.world`，spawn `(-14,0)`，较开放路线；
- Environment B：`worksite.world`，spawn `(0,0)`，较密集塔区路线。

12/12 均有正式 terminal、mission PASS、完整指标、有效 requested/filtered/applied 链和合法 null-reward 边界。首轮 Observation C 单 run valid ratio 为 `0.545～0.834`，高速度下 tracking error 明显变差；少量恢复后的 planner `CANCELLED` 被保留为真实结果。

首轮低 valid ratio 的两项根因已经审计：重规划后的旧 lidar 需要按 source stamp 选择因果轨迹历史；大 sim time 下 ROS `sec/nsec -> float -> sec/nsec` 会损失 1 ns。当前工作区已做最小时间戳/轨迹历史修复，0.05 s 同步门限未放宽。环境 A post-fix 实飞通过；环境 B 定向实飞定位根因，同源因果 replay 的 training-active valid ratio 为 `0.995863`，最终数据质量门结论为 PASS。**修复后没有重跑完整 12 次矩阵**，不能把首轮 12 次历史数据改写成全量 post-fix 结果。

权威工件：

- `runtime_artifacts/learning_speed/calibration/manual_calibration_report.md`；
- `runtime_artifacts/learning_speed/calibration/manual_calibration_analysis.json`；
- `runtime_artifacts/learning_speed/calibration/observation_c_final_data_quality_report.md`。

### 8.5 下一步边界

当前允许的自然下一步仍是 pre-reward 数据审计：复核 post-fix 样本质量、环境覆盖、action 可辨识性、terminal/collision provenance 和训练/验证划分需求。除非项目负责人另行明确授权，不开始 SAC、不定义 reward/φ1/φ2/权重，也不让任何未审查模型进入控制链。

## 9. 已实现但仍属部分验证 / 待验证

- YOLO：三路推理接口已通过；三机绕塔中的非空 PPE 检测、覆盖率和业务告警闭环待验证。
- outdoor_village：三机初始化和 UAV1 Learning Speed 飞行已通过；三机任务闭环待验证。
- Observation C：接口与定向数据质量门通过；修复后的完整 12-run A/B 矩阵未重跑。
- Learning Speed：fixed/mock 动态限速链和固定速度标定已通过；安全 action range、reward、SAC、模型推理与泛化均未完成。
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
| 单次固定速度标定 | `scripts/run_sh/learning_speed_manual_run.sh`；Environment A/B 与允许速度为显式参数 |
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
- `runtime_artifacts/learning_speed/calibration/manual_calibration_report.md`：当前两环境 12-run 正式结果。
- `runtime_artifacts/learning_speed/calibration/observation_c_final_data_quality_report.md`：Observation C 时间戳根因、修复边界与最终数据质量门。
- `clearance_semantics_cleanup_report.md`、`pre_entry_clearance_root_cause_report.md`：净空语义与历史残余清理。
- `worksite_mid360_startup_root_cause_report.md`：worksite terrain collision、Mid-360 和 1/2/3 机启动根因。
- `ego_planner_工程落地学习.md`、`legacy_ego_integration.md`：历史学习路线，仅作背景，不作为当前完成度入口。
