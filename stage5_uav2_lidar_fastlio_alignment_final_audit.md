# Stage 5 UAV2 LiDAR / FAST-LIO 对齐最终受控短窗审计

日期：2026-08-03  
分支：`ego-swarm`  
HEAD：`f68f07137db57698c15f39d093f812ff9851da15`

## 1. 最终判定

**本轮没有合法地复现历史飞行窗口，因而不能把根因判定为“主要是 LiDAR 仿真假回波”或“主要是 FAST-LIO/Gazebo 真值漂移”。** 可证实的最终边界是：

- 历史异常点在 FAST-LIO 注册/规划坐标中真实存在，经过 peer、self、ground 三层过滤后仍进入 EGO；其后 0.40 m 栅格膨胀和已膨胀图上的 0.55 m 搜索按实现语义触发占据。
- 静止完整录制已证明 FAST-LIO 的 body/world cloud、外参公式和 `camera_init -> body` TF 自洽；它**不能**证明飞行过程相对 Gazebo 真值没有漂移。
- 历史异常点投到已审计 world 几何后并不对应已知 collision 表面；它**不能**据此证明是插件假回波，因为该历史窗没有对应的 Gazebo 真值和完整 TF。
- 本次唯一可用的受控飞行入口被原有安全链拒绝，且没有规避：隔离时低空 bridge 缺少 `orbit_speed_scale` 心跳；启用原生 swarm manager 后，因 UAV1/UAV3 按要求未启动，manager 明确输出 `SAFETY_INHIBIT`、UAV2 takeoff permission=false、speed scale=0。UAV2 未 arm、未起飞、未发布目标或执行轨迹。

因此本报告的根因结论为：**未成功复现；仍缺“同一飞行窗内的 Gazebo 真值与完整 TF 对应关系”，不能继续泛化猜测。** 不允许据此改 self mask、0.40 m、0.55 m、地图分辨率、speed scale、轨迹或 EGO vendor。

## 2. 受控复现流程与安全结果

目标设计保持为仅 UAV2、无 UAV1/UAV3 实体、`worksite.world` 不变、`max_vel=0.20 m/s`、`max_acc=0.50 m/s²`、low-altitude 既有配置、任务节点禁用。计划是在起飞稳定后向 `/uav2/move_base_simple/goal` 发送历史规划坐标 `(-14.94, 8.26, 3.01)` m，并在相近 yaw 85.7°附近短时悬停/扫描；实际**没有进入这一步**。

| 尝试 | 仿真时间窗 / s | 原生安全结果 | 飞行/目标结果 |
| --- | ---: | --- | --- |
| isolated preflight | 11966.741–12024.876 | `require_orbit_speed_scale=true`，隔离编排没有该心跳，bridge 停在 `WAIT_INPUTS` | 未 arm、未起飞、未发 goal |
| isolated repeat | 11958.693–11999.833 | 同一门禁仍未满足 | 未 arm、未起飞、未发 goal |
| native-manager gate check | 11934 起的新会话 | `/swarm/coordinator/status`: `SAFETY_INHIBIT`；reason=`heartbeat/localization/trajectory/PX4/safety gate not clear`；`/uav2/swarm/takeoff_permission=false`；`/uav2/swarm/orbit_speed_scale=0.0` | 未 arm、未起飞、未发 goal，随后正常停止 |

没有手工发布合成的 scale、没有关闭其门禁，也没有把低空 map 参数换成别的 profile；这些都会改变本轮禁止触碰的安全/scale 契约。没有执行完整三机、完整绕塔或任一控制权旁路。

## 3. 录制与封存

复用并补齐了 `scripts/run_sh/uav2_occupancy_record_bag.sh`；新增记录但不发布任何控制的 topic 是 `/uav2/move_base_simple/goal`、`/uav2/mavros/imu/data`、`/uav2/mavros/imu/data_raw`、`/uav2/tower_mission/candidate_id` 和 `/uav2/swarm/orbit_speed_scale`。原有集合已经包含 `/clock`、`/tf`、`/tf_static`、Gazebo model/link states、Livox raw/IMU、FAST-LIO body/world、所有 peer/self/ground 阶段云、EGO 输入、膨胀 occupancy、FAST-LIO/MAVROS odometry、planner/goal/B-spline/bridge/status。

两个 bag 均为索引完成的 ROS bag（不是 `.active`）且没有覆盖历史证据：

- `test_evidence/stage5_uav2_lidar_fastlio_alignment_20260803/uav2_controlled_historical_position_yaw_scan.bag`：58.135 s、204,902 messages、343,238,279 bytes；
- `test_evidence/stage5_uav2_lidar_fastlio_alignment_20260803/uav2_controlled_historical_position_yaw_scan_unity_scale.bag`：41.140 s、145,029 messages、243,325,292 bytes。

第二个文件名保留了最初的测试意图；实际没有成功发布 unity scale，安全执行策略阻止了该操作。两份 bag 都包含完整点云/TF/Gazebo链；它们只覆盖地面静止/预检，不伪称已经覆盖历史位置或 yaw 扫描。

对应只读分析产物为：

- `test_evidence/stage5_uav2_lidar_fastlio_alignment_20260803/preflight_gate_attempt_analysis.json`
- `test_evidence/stage5_uav2_lidar_fastlio_alignment_20260803/isolated_gate_attempt_analysis.json`

## 4. Gazebo 真值与 FAST-LIO 残差

历史飞行窗仍没有 `/gazebo/model_states`、`/gazebo/link_states`、完整 `/tf` 或 `/tf_static`，所以不能反推该窗的逐帧真值残差。此次两个可录制短窗都未产生平移/转动飞行；因此它们只能提供静止基线，不能编造速度、角速度、yaw 相关性、滞后或飞行累计误差。

已存在的完整静止基线（`stage5_uav2_near_occupancy_source_audit.md`，11975.224–11991.771 s）给出以下独立核验：165 对同时间戳 FAST-LIO body/world cloud、每帧最多 100 点，共 16,500 点，按

`p_world = R_imu_world * (R_lidar_imu * p_lidar + T_lidar_imu) + p_imu_world`

重算，最大误差 `2.8865e-6 m`、均值 `3.2277e-7 m`。配置为 `R_lidar_imu=I`、`T_lidar_imu=(-0.011,-0.02329,0.04412) m`、`extrinsic_est_en=false`。静止动态 TF `uav2/camera_init -> uav2/body` 无跳变，配合静态 `world -> uav2/map=(4,0,0)`、`uav2/map -> uav2/camera_init=I`、`body -> base_link=I`、`base_link -> mid360_link=(0,0,0.08)`，可排除静止时的简单外参/adapter 公式错误。

这不是飞行真值残差：本轮没有可靠的飞行 Gazebo base_link 样本，故以下量均为**未测量**而非零：FAST-LIO–Gazebo XYZ、MAVROS–Gazebo XYZ、RPY residual、残差与速度/角速度/yaw 的相关性、时间滞后造成的空间误差。

## 5. 异常点坐标、双路径与 Gazebo collision 核对

历史首个可精确重建触发帧为 13008.712 s（最终 cloud 为 13008.732990 s，差 20.990 ms）：

| 项目 | 证据值 |
| --- | --- |
| `/uav2/Odometry`，`uav2/camera_init` | `(-14.941914, 8.264589, 3.010588)` m |
| 最近最终原始点 | `(-15.510577, 8.219481, 3.458909)` m，0.725537 m |
| 可贡献占据的 raw 点 | `(-15.488401, 8.421498, 3.497711)` m |
| 经 odom 逆变换的 body | `(0.115338, 0.560295, 0.483037)` m |
| 经 FAST-LIO外参反算的 LiDAR | `(0.126338, 0.583585, 0.438917)` m |
| 历史簇（0.35 m） | 130 点，world 质心 `(-15.634667,8.240142,3.464664)` m |

路径 B（LiDAR -> FAST-LIO 外参 -> odom -> planning）由上述数值和静止 16,500 点核验支持。路径 A（LiDAR -> **同一时刻 TF tree** -> Gazebo world）在历史 bag 不可做：缺该时刻 TF 和 Gazebo link pose。用新静止 bag 的静态 `world <- uav2/map` 仅作条件转换时，贡献点会位于 Gazebo world `(-11.488401,8.421498,3.497711)` m；这不是历史时刻的真值测量，不能代替路径 A。

条件位置对已审计 collision mesh 的最近距离：Oak mesh AABB 最少 2.258 m、Pine 6.054 m、radio tower 8.863 m、telephone pole 远离。Oak/Pine/tower 的 collision 与 visual 同一 DAE，telephone pole collision 为已列明 cylinder/crossbar，未发现可解释该点的 collision-only link。此结果排除了“按静态映射，该点就是已审计表面”的说法；它不等于已完成逐帧 Gazebo ray cast。

因没有到达历史位置/姿态，当前 bag 没有这一异常点簇，亦没有可对其执行的 Gazebo 精确 mesh ray hit。所缺的不是 AABB 计算，而是该原始点与同一时间的 LiDAR pose、Gazebo sensor link pose 和完整 collision scene 的共同证据。

## 6. 时间同步与 yaw 扫描

完整静止基线中，point cloud、odom 和 TF 的发布链时间戳新鲜，且 world/body cloud 同一帧的数值对齐。历史异常窗的点云–占据差为 20.990 ms，原先审计还确认当时 odom/raw odom/适配 cloud 同 stamp；这反驳了已记录链路中的长时延旧图残留。

但是，历史窗没有 Gazebo state receipt/stamp 对，应答无法计算 LiDAR–IMU–FAST-LIO–TF–Gazebo state 的逐帧差分，也不能用速度和角速度给出“足以/不足以解释 0.7 m”的定量结论。本轮车辆未飞，yaw 扫描次数为 0；故没有以静态朝向替代 85.7°附近视角试验。

## 7. 地图触发复核

历史点的完整传播已被只读重建：

```text
13008.732990 /uav2/cloud_registered
  -> peer: 0 removed
  -> self: 0 removed
  -> ground: 57–95 low points removed，异常簇保留
  -> /uav2/stage3/cloud_registered_filtered
  -> source voxel (-15.375, 8.375, 3.375) m
  -> offset (+2, 0, -1) cells，inflated (-14.875, 8.375, 3.125) m
  -> 距 UAV odom 0.172506 m
  -> /uav2/stage3/occupancy_inflate
  -> status adapter 在 <0.55 m 搜索，触发 CURRENT_POSITION_IN_OCCUPANCY
```

参数未变：resolution 0.25 m、XY `ceil(0.40/0.25)=2` cells、Z ±1 cell。用 FAST-LIO位置重建会触发；用**历史对应的 Gazebo 真值**重建无法计算，因为该真值缺失。故不能用“Gazebo 真值不触发”来选择漂移根因，也不能用“无 Gazebo surface”来选择插件假回波根因。

本轮两个未起飞 bag 的最终点云重建均为 0 个 0.55 m 内膨胀贡献点；self 前后序列化 cloud hash 相同。这是地面静止点云正常性的证据。注意低空 map 的 `ground_height=2.0 m`，而未起飞 adapted odom z 约 `-0.02 m`，所以 planner status 在预检地面阶段仍记录 collision（582/582 和 412/412 status frame）；它是当前位置低于低空 map 下边界的预期预检状态，**不是**历史结构外点簇复现，也不应计为本问题已修复或复发。

## 8. 根因分级与下一轮最小修复位置

已证明：

- self echo 不是历史结构外簇；self filter 保留，不能扩大。
- peer/self/ground filter、GridMap 体素化/0.40 m 膨胀和 status adapter 不制造 raw 点。
- 静止 FAST-LIO 变换链自洽；已审计 Gazebo visual/collision 不解释条件 world 点。
- 0.40 m 膨胀加 0.55 m 搜索形成约 0.95 m 名义两级安全包络（离散保守上界约 1.17 m）；这是触发放大器，不是 raw 点来源。

尚未证明：

- LiDAR 插件/模型在特定 yaw 产生无 collision ray hit 的假回波；
- FAST-LIO 时间同步、姿态或 world/home 对齐在飞行时相对 Gazebo 产生漂移；
- 二者共同存在及各自量级。

下一轮最小修改应只解决**可运行、原生安全允许的单机受控采集编排**：为隔离模式提供由已有安全协调器正式支持的单机 policy（不手工伪造 permission/scale，不修改 scale 数值、膨胀、碰撞半径或轨迹）。在该 policy 获得明确批准并存在后，再采集一次到历史位置/yaw 的完整 bag，并逐帧做路径 A/B 与 collision ray 比较。该最小修复位置属于测试/协调编排，不属于 self filter、FAST-LIO外参、地图或 EGO核心。

## 9. 是否可以进入八边形直线边段走廊修改

**不可以。** 历史异常簇的上游二选一（LiDAR 仿真回波 vs 飞行中的 FAST-LIO/Gazebo 对齐）尚未被新飞行真值证据判定；先补齐经原生安全许可的单机飞行短窗，再决定感知/TF/插件层的最小修复。此次没有修改八边形、Tier、ENTRY_GATE、航向、speed scale 或任何 EGO/EGO-Swarm vendor 代码。

