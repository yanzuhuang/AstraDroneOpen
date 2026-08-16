# 方案 C Future Trajectory / System State Fusion 接口报告

日期：2026-08-15

分支：`scene01-3uav-circuit-mission`

审计 HEAD：`e7e17986042ffd7000523d4cde1fcda3afbae594`

## 1. 结论与本阶段边界

本阶段已建立独立、只读、fail-closed 的方案 C observation 接口：

```text
Observation v2 atomic stamped surrogate
  + EGO official /planning/bspline
  + timestamp-matched FAST-LIO pose/derived velocity
  + EGO applied v_max state at-or-before observation time
  -> ObservationC (current body frame)
```

没有训练 SAC，没有设置 reward，没有增加网络 fusion architecture，没有让 RL 输出
waypoint/trajectory/velocity/thrust/body rate，也没有发布 MAVROS/PX4 控制。本阶段没有修改
EGO 搜索、碰撞、优化、B-spline 或重规划算法，没有修改 FAST-LIO 和 Observation v2
点云语义。

验证范围是确定性 unit、离线几何/时间缩放观察、ROS 消息生成、launch namespace 展开和
合成正式 ROS 消息的节点级 integration；没有启动 `--control`、Gazebo 飞行或正式 SAC。

## 2. EGO 正式 future B-spline 审计

### 2.1 正式来源

正式输入是每架 UAV namespace 下的 `planning/bspline`，类型为
`traj_utils/Bspline`。它不是 visualization path，也不是从 `PositionCommand` 复制出的
近似轨迹。

源码链如下：

1. `EGOPlannerManager::updateTrajInfo()` 把规划成功的
   `UniformBspline position_traj` 写入 `local_data_.position_traj_`，由它生成一阶
   `velocity_traj_`、二阶 `acceleration_traj_`，同时记录 `start_time_`、`duration_` 并
   递增 `traj_id_`。
2. `EGOReplanFSM::callReboundReplan()` 在规划/优化成功后，把该正式 position B-spline
   的控制点、完整 knot、degree=3、`start_time_`、`traj_id_` 和 `status_frame_id_`
   发布为 `traj_utils/Bspline`。
3. `traj_server` 订阅同一个 `planning/bspline`，用完全相同的控制点、degree、knot
   重建 `UniformBspline`，再调用 `getDerivative()` 获得 velocity/acceleration，并以
   100 Hz 生成正式 `planning/pos_cmd`。

方案 C 订阅的正是第 2/3 步之间的正式消息，没有创建或缓存第二套规划轨迹。

### 2.2 timestamp、frame 与更新频率

`traj_utils/Bspline.start_time` 是 EGO 在 `updateTrajInfo(position_traj,
ros::Time::now())` 时记录的 ROS time；B-spline 参数 elapsed time 为：

```text
t_bspline = observation_stamp - trajectory.start_time
```

本地正式消息的 `frame_id` 是 EGO 的 `status_frame_id_`，当前单机默认是
`camera_init`，并由配置中的 `observation_c/frames/world` 严格检查。它不是 body frame。

轨迹消息没有固定发布 Hz。EGO FSM 以 0.01 s timer 检查状态，但只在首次规划、成功
重规划或 emergency-stop 轨迹形成时发布一次新 B-spline。`traj_server` 的 100 Hz 是
命令求值频率，不能写成 B-spline 更新频率。

### 2.3 position / velocity / acceleration 求值

实现按项目 `UniformBspline` 的 De Boor 语义重建正式曲线：有效参数范围为
`[u[p], u[m-p]]`，elapsed time 先加 `u[p]`，再执行 De Boor recursion。

正式导数控制点沿用 EGO 公式：

```text
Q_i = p * (P_(i+1) - P_i) / (u_(i+p+1) - u_(i+1))
```

导数 B-spline degree 减 1，并移除 knot 首尾各一个；再求一次导数得到 acceleration。
本阶段 Observation C 只输出 position-based future feature；velocity/acceleration evaluator
实现并受测试，用于审计正确性和 tracking-state 候选分析，没有无理由塞进 policy feature。

### 2.4 replanning 替换

`ActiveTrajectoryStore` 对正式消息做单调替换：start time 更新，或相同 start time 下
`traj_id` 更大时，才原子替换 active trajectory。旧消息/重复消息不能恢复旧缓存。
ROS time 倒退时 trajectory、kinematic state 和 speed-state buffer 全部清空并重新
warm-up。节点级 integration 已验证 `traj_id 1 -> 2` 后继续使用 2，并拒绝重新发送的 1。

## 3. 与 Observation v2 的统一 timestamp / frame

原有 v2 multi-array topic 没有 ROS Header，不能安全地用多个“最新数组”拼接。因此本阶段
只给 Observation v2 增加一个不改变其语义的 `LidarSurrogateStamped` 原子镜像：

```text
header.stamp    = ObservationV2.stamp_sec = current cloud/history output time
header.frame_id = ObservationV2.frame_id  = current body frame
surrogate / valid_mask / unknown_mask / semantic = 同一次 v2 build 的四个数组
```

Observation C 以该 header stamp `t` 为唯一 fusion timestamp。FAST-LIO pose/velocity
必须在 `t` 精确命中，或由现有 v2 相同原则的有界前后样本插值获得；不会用 latest odom。
EGO B-spline 必须满足 `start_time <= t <= end_time`。任何一项无法匹配都发布 invalid。

world trajectory 转 current body 的公式为：

```text
p_body(t) = R_world_body(t)^T * (p_world - translation_world_body(t))
```

点云 surrogate 与 future positions 因而共享同一个 `header.stamp`、同一个 current body
frame 和显式的 trajectory source frame metadata。

## 4. Trajectory sampling

### 4.1 distance / arc-length（默认）

从 `t` 对应的正式 B-spline 点开始，在剩余 knot spans 上进行自适应空间细分；细分条件
由最大 chord length 和 midpoint chord error 共同控制，当前近似分辨率为 0.02 m。
对得到的有序 polyline 累计弧长，再在目标距离上做段内线性插值。采样点始终来自正式
B-spline 的未来段，不把控制点直接当轨迹点。

剩余轨迹短于所需 horizon 时，后续点安全地饱和到正式轨迹终点，offset 同时报告实际
可用距离。当前可配置实验值是：

```text
trajectory_sample_count:   20
trajectory_sample_spacing: 0.25   # distance mode: m
trajectory_max_distance:   5.0    # m
trajectory_sampling_mode:  distance
```

这些是与当前 5 m EGO planning horizon 对齐的工程实验值，不是最终论文参数。

### 4.2 time（保留用于消融）

`trajectory_sampling_mode=time` 完整保留。此时
`trajectory_sample_spacing` 解释为 seconds；仍沿正式 B-spline 时间参数求值，同时用
`trajectory_max_distance` 限制最大空间弧长，超过后饱和在曲线上的对应点，不删除该模式。

## 5. Trajectory feature 选择

当前每点只包含：

```text
[x_body, y_body, z_body]
```

并附带 sample offsets、sampling mode/spacing/max distance、trajectory id/start time 和
source frame metadata。

本阶段没有加入 tangent、velocity magnitude、curvature：

- tangent 可由正式 position B-spline 一阶导数得到，方向性有解释价值，但当前 body-frame
  position sequence 已表达局部走向，先避免重复 feature；
- velocity magnitude 来自正式一阶导数，但明显受 `v_max` 和 B-spline 时间重参数化影响，
  容易把 action/时间参数化再次编码进输入，本阶段不加入；
- curvature 可由一、二阶导数计算，几何意义较强、对纯时间缩放较稳定，但数值稳定性和
  是否提供额外收益尚无证据，留作后续消融。

## 6. System State

### 6.1 actual velocity

当前 FAST-LIO `publish_odometry()` 设置 pose、`header.frame_id=camera_init`、
`child_frame_id=body` 和 lidar-end timestamp，但源码没有给 `odomAftMapped.twist` 赋值。
因此不能把默认零 twist 当实际速度。

Observation C 沿用现有 Speed Adapter 的工程来源：对连续、带 stamp 的 FAST-LIO
position 做因果差分，并以 `velocity_filter_alpha=0.30` 低通。结果先在 world frame
估计，再用 observation timestamp 的姿态旋转到 current body frame。首帧、dt 非法、
间隔过大或无法在 `t` 匹配时 fail closed。

### 6.2 tracking error

当前工程定义为：

```text
e_world(t) = p_desired_official_bspline(t) - p_actual_FAST_LIO(t)
e_body(t)  = R_world_body(t)^T * e_world(t)
tracking_error_norm = ||e_body(t)||_2
```

消息同时保留 body xyz vector 和 scalar norm。选择该定义的原因是 desired 与 actual 都能
在同一个 observation timestamp 求值，且 desired 直接来自正式 B-spline。

保留但未写死为当前论文定义的候选包括：

1. `planning/pos_cmd.position - actual position`：控制链语义直观，但 `PositionCommand`
   是 100 Hz 瞬时消息，若没有 timestamp buffer 容易变成 latest-value mismatch；
2. desired/actual velocity error：反映速度跟踪，但 FAST-LIO 当前没有正式 twist，实际速度
   依赖差分滤波，误差噪声和定义耦合更强；
3. position xyz、horizontal norm、3D norm：当前同时保留 xyz 和 3D norm，后续训练合同再
   决定是否只输入其中之一。

### 6.3 previous v_max

来源是 EGO 已应用回执 `learning_speed/applied_v_max`，不是未经 safety filter 的 raw
policy request。现有回执为无 Header 的 `std_msgs/Float64`；Observation C 在收到回执时
记录当前 ROS time，并保存有界历史。在 observation stamp `t` 只使用 `<= t` 的最近一项
做 zero-order hold，绝不把较新的 applied 值拼到较旧 observation。没有可用历史时
invalid。单测已覆盖未来 update 不得泄漏到过去 observation。

## 7. Observation C 完整合同

ROS 原子消息 `learning_speed_rl/ObservationC`：

```text
header: {stamp=t, frame_id=current_body}
version / valid / diagnostics

lidar_surrogate[3200]
lidar_valid_mask[3200]
lidar_unknown_mask[3200]
lidar_semantic[3200]

future_positions_body[N]          # geometry_msgs/Point
future_sample_offsets[N]          # distance m or time s
trajectory_sampling_mode
trajectory_sample_spacing
trajectory_max_distance
trajectory_id
trajectory_start_time
trajectory_source_frame

actual_velocity_body xyz
tracking_error_body xyz
tracking_error_norm
previous_v_max
```

Python 侧使用独立的 `ObservationC`、`LidarSurrogateFeature`、
`FutureTrajectoryFeature` 和 `SystemStateFeature` dataclass；没有决定 CNN、PointNet、
Transformer、attention 或 MLP 层数。

## 8. Fail-closed

以下情况均不会形成 `ready_for_policy=true` 的正式 observation：

- Observation v2 valid=false、数组维度/有限值/semantic 非法；
- trajectory 未收到、frame 错、消息结构/knot 非法、timestamp 不在 active interval；
- pose 缺失、frame 错、插值间隔过大、ROS time reset；
- actual velocity 尚不可用；
- previous applied v_max 缺失、非法或只有 observation 之后的值；
- future trajectory 已无有效未来段或 sampler 失败；
- observation 超过 freshness timeout。

失败时节点发布 `ObservationC.valid=false`、原因 diagnostics 和 latched
`observation_c/valid=false`。节点没有 policy、`v_max` 或任何控制 publisher。现有
`SpeedSafetyFilter` 未修改；未训练模型仍不能接管 `v_max`。

## 9. 验证与结果

### 9.1 静态审计

- 正式 source、timestamp、frame、De Boor 和 derivative 与当前 EGO/traj_server 源码
  对照完成；
- `git diff` 确认本阶段对 EGO planner 和 FAST-LIO 源码没有修改；
- ROS message generation、`rosmsg show`、XML/YAML 和 Python syntax 检查通过；
- launch 展开：`uav1` 仅 C 节点为 `/uav1/observation_c`；`uav2` v2+C 为
  `/uav2/observation_v2`、`/uav2/observation_c`，topic 配置均为相对 namespace。

### 9.2 几何、重规划和 speed observation

确定性验证结果记录在
`runtime_artifacts/scheme_c_trajectory_fusion/offline_validation.json`：

- 直线：最小 future body +X 为 0.25 m；
- 转弯：future body +Y 最大 3.0 m，正确表达左转弯曲；
- yaw +90°：同一 world 直线在 body X 的最大残差约 `1.11e-15 m`，旋转到 body -Y；
- replanning：节点 integration 正确切换新 `traj_id`，旧消息不能恢复旧轨迹；
- `MockSpeedPolicy` 从 0.20 改为 0.10 m/s，并对同一几何、2 倍 knot 时间尺度的两条
  正式格式 B-spline 做工程观察：distance-sampled L2 delta 为 0.0 m，time-sampled
  L2 delta 约 6.697 m。

最后一项只说明在该确定性同几何/时间缩放构造中 distance representation 更稳定；没有
把它写成真实 EGO 动态重规划或论文泛化结论，也没有进行飞行。

### 9.3 自动测试

最终 `learning_speed_rl` catkin 测试汇总：

```text
34 tests, 0 errors, 0 failures, 0 skipped
```

其中方案 C unit 12 项覆盖正式 cubic B-spline/derivative、malformed 输入、distance/time
sampling、直线、转弯、yaw、timestamp mismatch、FAST-LIO state matching/interpolation、
previous v_max 时间因果性、replanning update、fail-closed 和 namespace isolation。
ROS integration 1 项（由 rostest/rosunit 两层结果记录）覆盖正式消息序列化、节点运行、
20 点输出、actual velocity、previous v_max、replanning、旧轨迹拒绝和 lidar invalid。
Observation v1/v2、SpeedSafetyFilter 与 artifact boundary 回归继续通过。

## 10. 最终回答

1. EGO 正式 future B-spline：`planning/bspline` 的 `traj_utils/Bspline`，与
   `traj_server` 同源。
2. timestamp/frame：EGO `start_time_` + knot elapsed time；source frame 为当前配置的
   `camera_init`，输出为 observation timestamp 的 current `body`。
3. world→body：用 `pose(t)` 做完整 SE(3) inverse transform。
4. distance sampling：正式 B-spline 自适应空间细分、累计弧长、等距插值。
5. time sampling：保留，可配置用于消融。
6. 默认实验值：20 点、0.25 m、最大 5.0 m、distance。
7. trajectory feature：当前只含 body-frame position xyz，加明确 sampling/trajectory
   metadata；未加入 tangent/speed/curvature。
8. actual velocity：FAST-LIO timestamped position 因果差分和低通，再转 body。
9. tracking error：正式 B-spline desired position(t) 减 FAST-LIO actual position(t)，
   输出 body xyz 与 3D norm；其它候选已保留比较。
10. previous v_max：`applied_v_max` 在 observation time 之前最近的 EGO applied 值。
11. Observation C：见第 7 节原子消息与 Python dataclass。
12. Mid360/trajectory 同步：v2 stamped packet 的唯一 `t/body` + 同 stamp pose lookup +
    active trajectory interval 检查，禁止 latest-value join。
13. replanning：可正确切换新 trajectory，拒绝旧缓存回退。
14. Mock speed：本次确定性构造中 distance delta 0.0 m，time delta 约 6.697 m；仅作工程
    观察。
15. fail-closed：unit 和 ROS integration 均通过。
16. 测试：34/34 通过。
17. EGO 核心算法：本阶段未修改。

```text
SCHEME C TRAJECTORY FUSION INTERFACE: PASS
```

> 方案 C observation 输入接口已经具备，下一阶段可以进行固定速度 baseline、mission completion 记账修复和训练数据合同设计；当前仍不开始正式 SAC 训练。
