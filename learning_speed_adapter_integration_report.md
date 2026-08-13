# Learning Speed Speed Adapter 集成报告

日期：2026-08-13  
分支：`scene01-3uav-circuit-mission`  
审计 HEAD：`f7745f2ed568ef28e814bb5e9ebe614bbf34c2e0`

## 1. 结论与验收边界

本阶段已完成 Learning Speed 的工程接口和 Mock 数据链路：

```text
现有 EGO Occupancy/里程计/规划状态
  -> learning_speed_rl Observation 合同
  -> MockSpeedPolicy
  -> 裁剪 + 滞回 + 变化率限制 + 单步限制 + 低通
  -> /uavN/learning_speed/v_max
  -> EGO FSM 动态速度接口
  -> PlannerManager 与 BsplineOptimizer 同步更新
  -> 必要时从当前轨迹状态触发 EGO 重规划
```

实现保留了 EGO-Planner；RL 不生成航点、轨迹或避障动作，也不发布 MAVROS/PX4
控制。三机接入默认关闭，必须显式使用 `--learning-speed`。

当前验证状态：

| 项目 | 状态 | 证据/说明 |
|---|---|---|
| 新包、Observation、Mock、滤波接口 | 已完成 | 单元测试、构建、实时 ROS 图 |
| 动态值送入 EGO 并得到回执 | 已完成 | 三机 0.20 → 0.08/0.12 → 0.20 飞行 bag |
| EGO/三机默认行为不变 | 已完成（静态与无控制） | 默认开关 false；launch 展开、回归测试通过；任务源码未改 |
| 高→低→高实际飞行 | **接口通过** | 两组实际三机 SITL 周期；raw/safe/applied、期望/实际速度均有 bag |
| EGO 动态重规划健康 | **通过** | 两组动作后最大连续规划失败为 0，未触发 EGO emergency stop |
| 完整三机任务兼容 | **部分通过** | UAV2/UAV3 有继续任务样本；0.08 长保持触发 ENTRY 无进展返航，0.12 轮 UAV1 也受既有占据/无进展保护返航 |
| 专门静态避障与最小净空 | **待验证** | 轻量 world 只能验证链路，不能替代 worksite/障碍场景验收 |
| RL 训练/正式模型 | 未开始，符合本阶段要求 | 飞行节点只允许 Mock，未训练网络不会进入控制链 |

因此可以确认“Speed Adapter → EGO 动态规划约束”的工程链已打通并在飞行中可逆响应；
不能声称任意低速动作都与当前三机任务超时兼容，也不能声称完整避障飞行已经验收。

## 2. 已阅读和审计的范围

完整阅读了：

- `AGENTS.md`、`rule.md`；
- `essay/Learning_Speed_Adaptation_详细学习笔记.md`；
- 当前三机启动脚本、三级 launch、任务节点、swarm manager/state/safety/policy；
- EGO FSM、PlannerManager、BsplineOptimizer、UniformBspline、GridMap、traj_server
  及相关配置和 launch。

没有修改外部 PX4、FAST-LIO、三机任务状态机、航点选择、ENTRY_GATE、EXIT_GATE、
协同策略或 EGO 搜索/碰撞/轨迹代价函数。

## 3. 原始 EGO 最大速度机制

### 3.1 三机任务中的参数来源

当前三机入口在 `triple_tower_inspection.launch` 中定义：

```text
max_vel = 0.20 m/s
max_acc = 0.50 m/s²
```

调用链为：

```text
triple_tower_inspection.launch
  -> dual_tower_inspection.launch / uav_tower_stack.launch
  -> ego_gazebo_bridge.launch
  -> advanced_param.xml
       ├─ manager/max_vel       = max_vel
       ├─ optimization/max_vel  = max_vel
       └─ bspline/limit_vel     = max_vel（当前源码没有直接读取该 ROS 参数）
```

`max_acc` 同样写入 `manager/max_acc`、`optimization/max_acc` 和
`bspline/limit_acc`。

### 3.2 EGO 内部读取和使用位置

```text
manager/max_vel
  -> EGOPlannerManager::pp_.max_vel_
  -> 初始控制点时间间隔
  -> 全局/局部轨迹时间分配
  -> FSM 局部目标采样步长和制动距离
  -> UniformBspline::setPhysicalLimits(max_vel, max_acc, tolerance)

optimization/max_vel
  -> BsplineOptimizer::max_vel_
  -> 可行性代价及梯度
  -> 约束优化后的 B-spline

B-spline
  -> /uavN/planning/bspline
  -> traj_server
  -> /uavN/planning/pos_cmd
  -> ego_gazebo_bridge
  -> MAVROS/PX4
```

当前优化器逐轴检查 `vx/vy/vz`，因此它是分量上限，而不是严格的欧氏速度模长上限；
这与论文公式的语义差异必须在以后训练和标定时保留在模型合同中。

### 3.3 与最大加速度的耦合

`v_max` 和 `max_acc` 是两个独立参数，本次只动态改变 `v_max`，保持
`max_acc=0.50 m/s²`。但两者在时间分配、制动距离 `v²/(2a)`、梯形速度时间估计和
B-spline 可行性检查中共同起作用，所以动力学效果并非完全解耦。

### 3.4 原系统为什么不能安全地直接动态修改

`manager/max_vel` 和 `optimization/max_vel` 都只在节点初始化时读取一次：

- 运行中 `rosparam set` 只改参数服务器，不改 C++ 成员；
- 系统没有 dynamic_reconfigure 或等价的运行时 setter；
- bridge 的 `orbit_speed_scale` 是执行端 p/v/a/yaw_rate 缩放，不会让 EGO 用新速度
  重新做时间分配和可行性优化，不能替代规划器速度约束。

所以只在外部改 ROS 参数或缩放 PX4 指令都不满足“RL 只给 EGO 决定动态速度上限”的
架构要求。

## 4. 最低侵入性的动态 `v_max` 接入

### 4.1 接口设计

EGO 增加一个默认关闭的标量接口：

```text
/uavN/learning_speed/v_max          std_msgs/Float64  Adapter -> EGO
/uavN/learning_speed/applied_v_max  std_msgs/Float64  EGO -> Adapter（latched ack）
```

FSM 回调完成以下工作：

1. 拒绝 NaN、Inf、非正数和配置范围外的值；
2. 动态上限不得超过启动时审计过的静态 `manager/max_vel`；
3. 在同一 ROS 回调中同步更新 `PlannerManager::pp_.max_vel_` 和
   `BsplineOptimizer::max_vel_`；
4. 发布实际应用值回执；
5. 在 `EXEC_TRAJ` 且有目标时，累计变化达到 `replan_delta` 并通过 cooldown 后，转入
   `REPLAN_TRAJ`；
6. 重规划使用当前轨迹的 p/v/a 边界状态，不从零速度或新位置硬接。

三机默认参数为：

```text
dynamic_speed_limit/minimum: 0.05
dynamic_speed_limit/maximum: 0.20   # 启动静态上限
dynamic_speed_limit/replan_delta: 0.03
dynamic_speed_limit/replan_cooldown: 1.0 s
```

### 4.2 连续性和实时生效语义

收到合法消息后，manager 和 optimizer 的内存值立即更新，但已经发布的旧轨迹不会被
直接重采样。累计变化达到 0.03 m/s 后请求重规划；小于阈值的变化将在下一次常规重规划
时使用。

这意味着：

- 参数回执“立即”，新轨迹“阈值/cooldown 后”；
- 由当前 p/v/a 作为重规划起点可避免位置或速度硬跳；
- 从高速降低上限时，当前速度不可能瞬间满足新上限，应按固定最大加速度平稳减速；
- 过快或过大的下降仍可能增加重规划失败风险，所以 Adapter 必须先平滑；
- 当前 `v_max` 是 EGO 规划约束，不是执行端紧急制动器或硬实时 safety shield。

在多机 EGO 路径中，vendor 源码对 `drone_id > 0` 关闭了一部分可行性 refine；当前优化
可行性仍是软代价。这是既有多机限制，不应通过 RL 训练掩盖，后续飞行标定必须覆盖。

### 4.3 为什么有少量 EGO vendor 接口修改

由于原参数缓存且 bridge 缩放不等价于规划约束，完全零修改无法让新值同时进入 manager
时间分配和 optimizer 可行性代价。本次只增加 setter、订阅/回执、验证门和重规划触发；
没有更改搜索、碰撞检测、地图融合、B-spline 代价或任务算法。这是当前最低侵入方案。

## 5. `learning_speed_rl` package

新包位于：

```text
AstraDrone_ros1_ws/src/learning_speed_rl/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── launch/
│   └── speed_adapter.launch
├── config/
│   └── speed_adapter.yaml
├── scripts/
│   └── speed_adapter_node.py
├── src/learning_speed_rl/
│   ├── observation/
│   │   ├── types.py
│   │   ├── builder.py
│   │   └── voxelizer.py
│   ├── policy/
│   │   ├── base.py
│   │   ├── mock_policy.py
│   │   ├── safety_filter.py
│   │   └── network_contract.py
│   ├── training/
│   │   └── environment_interface.py
│   └── inference/
│       └── model_runner.py
├── models/
│   └── README.md
├── test/
│   ├── test_observation.py
│   └── test_safety_filter.py
└── README.md
```

训练和飞行推理在模块层面分离。正式节点不导入 `training/`；当前节点若配置非
`policy/mode: mock` 会拒绝启动。`models/` 中没有伪造或未训练权重。

## 6. Speed Adapter ROS 接口

以下 topic 均相对于 UAV namespace：

| 方向 | Topic | 类型 | 用途 |
|---|---|---|---|
| 订阅 | `Odometry` | `nav_msgs/Odometry` | 位姿、速度/加速度估计 |
| 订阅 | `stage3/occupancy_inflate` | `sensor_msgs/PointCloud2` | EGO 当前膨胀占用几何 |
| 订阅 | `planning/pos_cmd` | `quadrotor_msgs/PositionCommand` | 同 frame 的期望状态、tracking error |
| 订阅 | `move_base_simple/goal` | `geometry_msgs/PoseStamped` | 同 frame 时的局部目标相对量；跨 frame 拒绝 |
| 订阅 | `learning_speed/mock_v_max` | `std_msgs/Float64` | Mock 原始动作 |
| 订阅 | `learning_speed/applied_v_max` | `std_msgs/Float64` | EGO 应用回执 |
| 发布 | `learning_speed/raw_v_max` | `std_msgs/Float64` | 原始 policy 输出 |
| 发布 | `learning_speed/v_max` | `std_msgs/Float64` | 过滤后、仅发给 EGO 的上限 |
| 发布 | `learning_speed/observation/low_dim` | `std_msgs/Float32MultiArray` | 归一化低维向量 |
| 发布 | `learning_speed/observation_ready` | `std_msgs/Bool` | 未来 RL 严格就绪门 |
| 发布 | `learning_speed/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 新鲜度、frame、shape、回执 |

该 package 没有 MAVROS、PX4、航点、目标、轨迹、避障或 swarm coordination publisher。

主要参数集中在 `config/speed_adapter.yaml`：

```text
policy_rate_hz:       10.0
v_max_min/max:        0.05 / 0.20 m/s
initial_v_max:        0.20 m/s
rise/fall_rate:       0.08 / 0.12 m/s²
maximum_step:         0.02 m/s
low_pass_alpha:       0.45
hysteresis:           0.005 m/s
applied_tolerance:    0.005 m/s
```

三机 launch 会用当前任务静态 `max_vel` 覆盖 Adapter 的最大值、初值和 Mock 默认值，
因此 policy 只能降低并恢复既有上限，不能越过静态审计上限。

## 7. Mock Policy 和安全处理

稳定 policy 接口为：

```text
PolicyObservation -> SpeedPolicy.predict() -> raw_v_max
```

`MockSpeedPolicy` 由 topic 设置标量；未来 `FeatureFusionSpeedPolicy` 使用同一个
`predict()` 边界。每个 backend 的输出都经过：

```text
有限值检查
  -> [v_max_min, v_max_max] 裁剪
  -> 目标滞回
  -> 上升/下降变化率限制
  -> 最大单步限制
  -> 低通滤波
  -> safe v_max
```

ROS 时间倒退时会重置滤波时间基准，避免仿真复位产生异常 `dt`。

## 8. Mid360 → FAST-LIO → Occupancy Map 数据链路

当前三机实际链路为：

```text
/uavN/livox/lidar + /uavN/livox/imu
  -> FAST-LIO
  -> /uavN/Odometry + /uavN/cloud_registered
  -> teammate/self/ground cloud filter
  -> /uavN/stage3/cloud_registered_filtered
  -> EGO GridMap::cloudCallback
  -> EGO occupancy_buffer_inflate_
  -> /uavN/drone_<id>_ego_planner_node/grid_map/occupancy_inflate
  -> occupancy_stamp_adapter
  -> /uavN/stage3/occupancy_inflate
  -> speed_adapter_node
```

三机当前地图配置：

- 全局边界：90 × 90 × 46 m；
- EGO 分辨率：0.25 m；
- low-altitude override 的局部半范围：12.5 × 12.5 × 4.5 m，即约
  25 × 25 × 9 m 的更新窗口；
- 规划 frame：`uavN/camera_init`；
- GridMap 可视化 timer：0.11 s，理论约 9.1 Hz；实际频率仍应在正常飞行中测量；
- 局部窗口围绕当前 odom/camera position 更新；
- 输入已经是 FAST-LIO/过滤点云，不需要第二套 SLAM。

### 8.1 当前地图是否适合作为 CNN 输入

结论是：**几何数据可复用并可固定裁剪，但当前语义不足以直接作为论文式 RL 输入。**

当前使用独立 PointCloud2 的 `GridMap::cloudCallback` 路径只重置局部 buffer、标记并膨胀
障碍物；它不对点云做 raycast free-space 更新。发布的 `occupancy_inflate` 也只含 occupied
点。虽然 launch 有 `/grid_map/unknown` remap，当前活动代码没有对应 unknown publisher。
普通 `occupancy` 依赖的概率 buffer 在这条点云快捷路径中也没有得到完整 hit/miss 更新。

因此，仅凭现有 topic 无法区分：

```text
observed free  vs.  unknown
```

把所有非 occupied 体素当 free 会引入危险的虚假可通行信息。当前实现选择保守语义：
occupied 点标为 occupied，其余全部标为 unknown，并保持
`map_semantics_complete=false`。

后续最小方案应是给 EGO GridMap 增加只读、分类化局部地图导出，明确输出
free/occupied/unknown；不应为 RL 再建一套 SLAM。该 vendor 接口应单独审批和测试。

## 9. CNN 输入格式

版本 1 的固定合同：

```text
dtype: float32
layout: [C, Z, Y, X]
channels: [free, occupied, unknown, on_trajectory]
crop: 24 × 24 × 8 m
resolution: 0.5 m
shape: [4, 16, 48, 48]
center: 当前无人机位置
axes: EGO planning frame 固定轴，不随无人机 yaw 旋转
```

该 crop 在三机 25 × 25 × 9 m 的局部更新窗内，且将 EGO 0.25 m 数据降采样到 0.5 m，
避免不必要的张量体积。四通道张量约 147456 个 float32（约 0.56 MiB/帧）。

当前只有 occupied 和 unknown 通道可信；`on_trajectory` 暂时只放一个当前
PositionCommand 点，不等于论文中的完整 preplanned trajectory。因此
`trajectory_context_complete=false`，RL 推理仍被禁止。

## 10. MLP 低维输入和归一化

固定 22 维顺序为：

```text
position xyz                  3
velocity xyz                  3
acceleration xyz              3
tracking_error xyz            3  # odom position - PositionCommand position
local_goal_delta xyz          3  # goal - current position
desired_velocity xyz          3
desired_acceleration xyz      3
previous_v_max                1
                               = 22
```

速度优先使用 odom 位姿有限差分并低通；加速度由速度差分并低通。所有 normalization
scale 在 YAML 中配置，字段顺序、scale、地图 shape 和 frame 约定都必须进入未来模型
版本元数据，禁止训练/推理各自维护不同顺序。

Adapter 只允许 odom、PositionCommand、occupancy 和 goal 在同一 frame 中做数值融合；不做
隐式 TF。当前三机 odom/PositionCommand/occupancy 为 `uavN/camera_init`，原始
`move_base_simple/goal` 为 `uavN/map`，因此 goal 会被拒绝并在 diagnostics 标记，
`local_goal_delta` 保持 0。未来若需要该观测，应在现有已验证 TF 契约上增加只读变换，
不能直接把两套坐标相减。当前 CNN crop 以 UAV 为中心但轴仍与 planning frame 对齐，
是否改成随 yaw 旋转的 egocentric tensor 必须在 Observation v2 中显式版本化。

## 11. CNN + MLP 融合接口

`FeatureFusionContract(version=1)` 定义：

```text
[4,16,48,48] categorical local map
  -> trajectory-conditioned CNN
  -> environment_feature

normalized 22-D state
  -> MLP
  -> state_feature

concat(environment_feature, state_feature)
  -> fusion MLP / RL actor
  -> one scalar raw_v_max_mps
  -> common SpeedSafetyFilter
```

接口不绑定 PyTorch/ONNX/TensorRT。`ReviewedModelRunner` 要求显式 backend、存在的模型
文件和完整 Observation；任一条件不满足即 fail closed。

## 12. 三机任务集成

`learning_speed_enabled` 及每机 override 已沿 triple → dual → UAV stack 传递。
默认值全部为 false；脚本新增：

```bash
scripts/run_sh/three_uav_inspection.sh --learning-speed
```

只有这个开关为 true 时，每个 UAV namespace 才启动一个 `speed_adapter` 并启用对应 EGO
订阅。light/full rosbag 已加入 Learning Speed 的 raw/safe/applied/ready/low_dim/diagnostics
topic，并加入 `/uavN/mavros/local_position/velocity_local`，使“规划期望速度”和“PX4
实际速度”可以分开审计。

没有修改：

- 航点和候选点选择；
- ENTRY_GATE / EXIT_GATE；
- `tower_mission` 状态机；
- EGO 避障逻辑；
- swarm manager、安全节点和三机协同流程；
- PX4/MAVROS 控制权规则。

## 13. 验证结果

### 13.1 构建、测试和 launch

- 相关包构建通过；
- `catkin_test_results build/test_results`：**466 tests, 0 errors, 0 failures,
  0 skipped**；
- 新增 Python Observation/滤波测试 6 项通过；
- 新增训练产物路径边界测试 2 项通过；
- 新增 EGO dynamic speed gate 测试 2 项通过；
- EGO、bridge、tower mission、swarm manager 既有回归均通过；
- touched XML 通过 `xmllint`，脚本通过 `bash -n`；
- standalone 和三机 `roslaunch --nodes` 解析通过；
- 三机 launch 参数展开确认每架 EGO/Adapter 范围均为 0.05–0.20 m/s；
- dry-run/无控制模式下任务节点不创建 MAVROS 控制 publisher。

### 13.2 三机实时 Mock 接口试验

在 `start_sim=false`、`enable_control=false`、wall time 的三机 ROS 图中进行
0.20 → 0.08 → 0.20 m/s：

- 高到低输出连续，代表步进约 `-0.0054 m/s/100 ms`；
- 低到高输出连续，代表步进约 `+0.0036 m/s/100 ms`；
- 未出现单帧突变；
- 三架最终 safe 输出均为 0.20 m/s；
- 三架 EGO applied 回执均约为 0.1999994 m/s；
- `observation_ready=false`，并明确标出 map/trajectory 语义不完整。

详细采样记录在：
`runtime_artifacts/learning_speed/evaluation/20260813_mock_interface_test.md`。

### 13.3 `worksite.world` 性能根因与验证场景

最初两次控制启动停在模型加载阶段。继续隔离后确认并非 Speed Adapter 或 EGO 死锁：

- `worksite.world` 包含大规模高多边形网格/heightmap；
- Gazebo Livox 插件使用 ODE multi-ray，在当前主机上单架默认 20000 rays 已占约 3.3 GiB；
- 即使把单架 downsample 提高到 200，首个 0.2 s 仿真时间仍需约 87 s 墙钟；
- 三架 downsample=200 可以依次插入并进入 lockstep，但每个 1 ms physics step 仍需数十秒，
  不具备动态飞行试验条件。

为只验证本阶段接口，新增 `learning_speed_validation.world`：保留任务使用的塔坐标
`(-10.0551, 19.7104)`、地面和基本物理，不替代正式 worksite。模型增加两个默认不改变
原行为的验证参数：`d435_enabled=true`、`lidar_downsample=1`；脚本只有显式指定
`--disable-d435 --lidar-downsample N --world FILE` 才改变本次运行。轻量场景下三架模型约
5 s 插入完成，MAVROS 均连接，Mid360/FAST-LIO 点云约 9.6–10.0 Hz。

无控制证据：

`runtime_artifacts/three_uav_inspection_dry_run_20260813_learning_speed_validation/`

### 13.4 0.20 → 0.08 → 0.20 压力飞行

主证据：

`runtime_artifacts/three_uav_inspection_control_20260813_learning_speed_high_low_high/`

bag 时长 320.49 s、645636 条消息。三架均在 EGO `ENTRY_GATE_TRANSIT` 后收到两条
Mock 动作：约 116.595 s 降到 0.08，约 234.539 s 恢复 0.20。

| 指标 | 结果 |
|---|---|
| safe/EGO applied 到 0.08 | 三架约 2.304–2.305 s 稳定 |
| safe/EGO applied 恢复 0.20 | 三架约 3.262–3.411 s 稳定 |
| EGO 连续规划失败 | 三架最大值均为 0 |
| EGO emergency stop | 三架均未触发 |
| UAV1 期望速度模长均值（高/低/恢复） | 0.0716 / 0.0137 / 0.0000 m/s；恢复段 HOLD |
| UAV2 期望速度模长均值（高/低/恢复） | 0.0872 / 0.0409 / 0.0613 m/s |
| UAV3 期望速度模长均值（高/低/恢复） | 0.1333 / 0.0442 / 0.1333 m/s |

期望轨迹速度对动作有清晰且可逆的响应。实际 MAVROS 速度没有被标量硬裁剪：PX4 跟踪
瞬态、返航/降落状态，以及 EGO 当前逐轴而非模长约束，都会让实际三维速度短时高于
`v_max`；这符合“规划约束而非执行端 safety shield”的接口语义。

任务层结果并非全通过：UAV1 到达 `ENTRY_READY`，UAV2/UAV3 分别在 138.254 s 和
183.758 s 因现有 `ENTRY_GATE no progress` 保护转入 `RETURN_HOME` 并安全结束。因此
0.08 长时间保持不适配当前 0.20 m/s 低速三机任务的超时尺度，不能作为获准动作下限。

### 13.5 0.20 → 0.12 → 0.20 飞行

主证据：

`runtime_artifacts/three_uav_inspection_control_20260813_learning_speed_compatible_cycle2/`

bag 时长 247.50 s、499289 条消息。0.12 动作约在 133.13–133.22 s，恢复约在
178.81–178.91 s；三架下降稳定需 1.48–1.70 s，恢复需 2.20–2.31 s。动作后最大连续
规划失败仍为 0，均无 EGO emergency stop。UAV2/UAV3 无 goal/current collision，并继续
到 `ENTRY_READY`/`ENTRY_GATE_TRANSIT`；它们的期望速度均在低速段下降并在恢复段回升。

UAV1 在 136.003 s 因当前点占据/既有 ENTRY 无进展保护返航。它从 37.65 s 已进入 ENTRY，
动作前存在频繁协同 HOLD，返航仅比 0.12 动作晚约 2.8 s，不能把根因归于限速动作；但
它仍使本轮无法获得“完整三机任务通过”的证据。另一次试验在任何 Mock 动作前，两台
bridge 已因 FAST-LIO 点云过期从起飞 HOLD 转入 LANDING，也不计入动态限速结果。

完整离线统计记录在：

`runtime_artifacts/learning_speed/evaluation/20260813_gazebo_dynamic_speed_test.md`。

所有仿真进程均已受控停止，FAST-LIO 的受控日志 `Log/mat_pre.txt` 已精确恢复到 HEAD
内容。

## 14. 对现有 EGO 避障和三机任务的影响判断

代码与静态回归层面：EGO 搜索、碰撞、地图、轨迹代价、航点和任务代码未改，默认开关
关闭，466 项测试全部通过，因此没有发现对原流程的回归。

运行飞行层面：两组动态周期中 manager/optimizer 更新、applied 回执、EGO 重规划和速度
响应均工作；没有连续规划失败或 emergency stop。未修改的 swarm/任务保护仍会 HOLD、
返航和降落，说明 Speed Adapter 没有越过现有安全状态机。

但本轮不能给出“避障能力完全不受影响”或“三机任务已完整正常”的最终结论：轻量场景
不是专门障碍/净空场景，UAV1 出现过 current-position occupancy，0.08 长保持触发两架
ENTRY 无进展返航，0.12 轮也只有两架继续任务。当前结论是：**动态规划接口通过，默认
关闭时不影响原任务；启用后的动作运行域与完整三机任务仍待标定和验收。**

## 15. 后续动态限速飞行矩阵

必须先取得在同一可实时场景中的稳定三机基线，不改任务逻辑，然后：

1. `learning_speed_enabled=false` 做一次完整基线飞行；
2. 开启 Mock，保持 0.20 m/s 完成一次等价回归；
3. 先从 0.20 → 0.16/0.18 → 0.20 的短窗口开始，逐级标定允许下限和保持时间；
4. 在任务超时按路径长度/生效限速自适应之前，不把 0.08 作为三机任务允许动作；
5. 记录 raw/safe/applied、实际速度模长、PositionCommand、B-spline id、planner status、
   tracking error、占用点云、任务阶段和三机距离；
6. 验证每次累计变化跨过 0.03 m/s 时的重规划次数与 cooldown；
7. 检查速度峰值、减速时间、轨迹 p/v/a 连续性、规划失败、HOLD/RECOVERY、障碍净空、
   相位差和最小间距；
8. 三架完成 ENTRY、绕塔、EXIT、返航、受控降落并确认 `armed=false` 后才判定通过。

建议将本轮新数据写入 `runtime_artifacts/learning_speed/evaluation/`，而不是源码目录。

## 16. 真正开始 RL 训练前还需完成

1. 为 `worksite.world` 建立可实时的碰撞/传感器仿真配置，或在经审批的等价轻量场景中
   取得稳定三机基线；D435 不是主因，核心瓶颈是复杂碰撞几何 × 多套 Mid360 ODE rays；
2. 增加 EGO 只读 categorical map export，可靠区分 free/occupied/unknown；
3. 从 `/planning/bspline` 采样完整预规划局部轨迹，填充 `on_trajectory`，并验证 frame/
   timestamp；
4. 在正常飞行中测量 map、odom、trajectory、policy、EGO replan 的频率和端到端延迟；
5. 固化 Observation version、归一化统计、动作范围、模型 hash 和 backend 版本；
6. 建立仅训练时使用的 Gazebo environment reset/step、随机障碍与 domain randomization；
7. 按学习笔记设计 reward：进度/速度收益、碰撞/近障、跟踪误差、规划失败、动作变化、
   舒适性和任务完成；
8. 选择并复现实验算法、replay/checkpoint/evaluation 流程，不把训练依赖带入正式 launch；
9. 对静态上限、低速恢复、不可达目标、传感器陈旧、地图丢失、模型超时和异常输出做
   fail-safe 测试；
10. 将任务 no-progress/segment timeout 与生效 `v_max` 建立显式契约，标定允许动作下限和
    最大低速保持时间，避免 RL 的合法低速被任务层误判为停滞；
11. 明确多机 EGO soft feasibility 的边界，再决定是否需要经审批的 vendor patch；
12. 所有 checkpoint、TensorBoard、CSV、图片、episode 和评估数据写入：

```text
runtime_artifacts/learning_speed/
├── checkpoints/
├── logs/
├── plots/
└── evaluation/
```

13. 只有通过离线、SITL 分级和人工评审的精选模型，才复制到
    `learning_speed_rl/models/`；当前目录没有正式模型。

## 17. 新增和修改文件

### 新增

- 完整 `AstraDrone_ros1_ws/src/learning_speed_rl/` package；
- `plan_manage/include/plan_manage/dynamic_speed_limit.h`；
- `plan_manage/test/dynamic_speed_limit_test.cpp`；
- `simulation/astra_gazebo_worlds/learning_speed_validation.world`（仅接口验证用）；
- `runtime_artifacts/learning_speed/evaluation/20260813_mock_interface_test.md`；
- `runtime_artifacts/learning_speed/evaluation/20260813_gazebo_dynamic_speed_test.md`；
- 本报告。

### 修改

- `bspline_opt/include/bspline_opt/bspline_optimizer.h`；
- `bspline_opt/src/bspline_optimizer.cpp`；
- `plan_manage/include/plan_manage/planner_manager.h`；
- `plan_manage/src/planner_manager.cpp`；
- `plan_manage/include/plan_manage/ego_replan_fsm.h`；
- `plan_manage/src/ego_replan_fsm.cpp`；
- `plan_manage/CMakeLists.txt`；
- `plan_manage/launch/advanced_param.xml`；
- `ego_gazebo_bridge/launch/ego_gazebo_bridge.launch`；
- `astra_swarm_bringup/launch/uav_tower_stack.launch`；
- `astra_swarm_bringup/launch/dual_tower_inspection.launch`；
- `astra_swarm_bringup/launch/triple_px4_mavros.launch`；
- `astra_swarm_bringup/launch/triple_tower_inspection.launch`；
- `astra_swarm_bringup/models/iris_mid360_d435.sdf.xacro`；
- `astra_swarm_bringup/package.xml`；
- `scripts/run_sh/three_uav_inspection.sh`。

模型与脚本新增的 D435、LiDAR downsample、world 参数都保持原默认行为；验证开关不会
静默进入正式任务。未 commit、未 push、未修改外部 PX4。
