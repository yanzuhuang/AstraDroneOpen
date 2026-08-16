# Learning Speed Observation v2：Mid360 point-cloud surrogate 原型报告

日期：2026-08-15  
分支：`scene01-3uav-circuit-mission`  
基线 HEAD：`e7e17986042ffd7000523d4cde1fcda3afbae594`

## 1. 结论

```text
Observation v2 prototype：PARTIAL
```

原型代码、版本化数据合同、时间戳 pose 对齐、3200-bin angular partition、
obstacle/free/unknown 三态、只读 ROS 接口和 20/20 单测均通过。既有验证完成了
`outdoor_village.world` 当前版本受控单机在线飞行：任务安全落地，正式 filtered
cloud、odom、surrogate、semantic、unknown、valid、diagnostics、aligned history、
visualization 和实际 pose/velocity 均已录包。在线平移、yaw、平移+yaw 和
空旷→障碍→空旷有有效证据，0.20 m voxel ghost 指标在 52 个运动样本中均为
1.0 overlap；Observation v2 仍不参与控制。

2026-08-15 在 terrain collision 修复后的当前 `worksite.world` 又完成了两项正式
补证：三机共享栈 UAV1-only 的 formal filtered/odom 在线观察，以及正式三机
无控制基线与三套 v2 同时启用的对照。三机链已形成且 namespace 隔离、无
backlog/drop；但单机既有任务在进塔前按地图净空规则安全拒绝入口，未形成塔旁
圆周运动。因此结论仍为 PARTIAL。

仍判为 PARTIAL，不能改成 PASS：

1. 既有稳定航迹没有经过可确认的建筑拐角，Test D 的遮挡后 unknown
   增减过程缺少专项在线证据。
2. 当前正式 `worksite.world` 单机链已恢复，但既有 PRE_ENTRY 目标被当前占据地图
   按 1.0 m 净空要求拒绝，飞行在约 22 m 塔距处安全终止。录到的是返航/下降段，
   不是塔旁绕塔段，不能验证 tower occlusion unknown、切向障碍和持续 yaw ghost。
3. 正式三机并发已经通过无控制链验证，但未执行 active EGO planning 或三机飞行；
   该证据只回答 v2 并发、隔离和系统存活，不替代旧任务完整回归。
4. outdoor 在线 ROS stamp 为 10 Hz、Observation p95 72.73 ms，但 Gazebo RTF
   中位数 0.55、墙钟输出中位数 5.54 Hz；算法有约 10 Hz 余量，不等于完整仿真
   墙钟稳定 10 Hz。

## 2. 最终数据链与代码边界

最终原型链如下：

```text
Mid360 /livox/lidar + /livox/imu
  -> FAST-LIO deskew / world registration
  -> existing ground/self filtering
  -> /uavN/stage3/cloud_registered_filtered (PointCloud2, uavN/camera_init)
  +  /uavN/Odometry (uavN/camera_init -> uavN/body)
  -> timestamped PoseBuffer (exact or bracketed interpolation)
  -> five CloudFrame records stored in their sample-time body frames
  -> every frame explicitly reprojected to current body(t)
  -> 0.05 m voxel fusion
  -> 4.5 deg full-sphere angular partition
  -> nearest distance + historical-FoV observed range
  -> ObservationV2 lidar_surrogate/masks/semantic/metadata
  -> read-only debug topics and RViz markers
```

实现位于
`AstraDrone_ros1_ws/src/learning_speed_rl/src/learning_speed_rl/observation/v2/`，
ROS wrapper 为 `scripts/observation_v2_node.py`，方向评估器为
`scripts/observation_v2_evaluator.py`。所有论文参考参数位于
`config/observation_v2_lidar_surrogate.yaml`，没有在核心算法中写死。

本阶段没有增加第二套 SLAM，没有复制 EGO map，没有改 EGO、FAST-LIO、航点、
ENTRY_GATE、EXIT_GATE、绕塔状态机，也没有 SAC/PPO、推力/bodyrate action、
Flying yaw/reward 或正式 RL 模型。

## 3. Topic 与 frame 审计

正式输入 topic 是 `/uavN/stage3/cloud_registered_filtered`。既有 outdoor 单机验证
使用无 namespace 等价 topic `/stage3/cloud_registered_filtered`；本轮 worksite
UAV1-only 验证使用精确正式 topic `/uav1/stage3/cloud_registered_filtered`。
历史 worksite bag 缺少该 topic，早期回放才显式覆盖为 `/uav1/cloud_registered`，
报告中始终不把该 raw-cloud 回放称为正式输入。

frame 合同如下：

| 数据 | frame / 语义 |
|---|---|
| FAST-LIO registered cloud | `camera_init`；多机 adapter 后是 `uavN/camera_init` |
| FAST-LIO odometry header | 同一 world/planning frame：`camera_init` 或 `uavN/camera_init` |
| FAST-LIO odometry child | IMU/body：`body` 或 `uavN/body` |
| 项目 `map` | 仿真中通过显式单位静态 TF 接到 `camera_init`；不是实测标定 |
| Mid360 sensor | `mid360_link`；FoV 使用 FAST-LIO 配置中的 LiDAR→IMU/body 外参 |
| RViz/surrogate output | 当前时刻 UAV body，ROS FLU |

FAST-LIO 的 registered cloud 已在 world frame，不能再次当成 sensor-frame 数值。
每条 cloud 必须满足 cloud frame 与配置的 world frame 完全一致；odom 的
`header.frame_id` 和 `child_frame_id` 也必须完全一致，否则 `valid=false`。

审计到的 FAST-LIO `mid360.yaml` LiDAR→IMU/body 外参是平移
`[-0.011, -0.02329, 0.04412] m`、单位旋转。Gazebo model 的
`base_link -> mid360_link` z=0.08 m 是模型 link 布置；两者不是可互换的同一个
frame contract。原型使用前者放置历史传感器 FoV，并把输出定义在 FAST-LIO
`body`。

## 4. 历史缓存、pose 获取和当前 body 对齐

`CloudHistoryBuffer` 保留最近 5 帧。每一帧保存：

- cloud stamp；
- source world frame；
- 该 stamp 对应的 `Pose3D`；
- world cloud 显式转换到该 stamp 的 body frame 后的有限、量程内、下采样点；
- 输入/有效点数、pose lookup mode 和下采样耗时。

`PoseBuffer` 容量和最大插值间隔均由 YAML 配置。优先使用完全同 stamp 的 odom；
没有精确样本时，仅在前后两个 odom 都存在且间隔不超过 0.05 s 时插值：位置用
线性插值，姿态用 quaternion SLERP。绝不拿“最新 odom”替代历史时刻 pose。
ROS 回调中 cloud 可能先于同 stamp odom 到达，因此有一个有界 pending queue；
对应 odom 到达后再处理，而不是先错配。

第 `k` 帧到当前 `t` 的对齐公式是：

```text
p_world   = T_world_body(k) * p_body(k)
p_body(t) = inverse(T_world_body(t)) * p_world
```

缺 pose、cloud 早于/晚于可用 pose 区间、插值跨度过大、frame 不符、pose age 超过
0.20 s 均 fail closed。ROS time 倒退时 pose、pending cloud、5 帧 history 一并清空，
重新 warm-up，并累计 reset 诊断计数。

## 5. Angular partition 与 surrogate 合同

参考配置是 4.5° 全空间等角 partition：

- azimuth：`[-180°, 180°)`，`atan2(+Y_left, +X_forward)`；80 bins；
- elevation：`[-90°, 90°]`，`atan2(+Z_up, hypot(X,Y))`；40 bins；
- body 方向：ROS FLU，`+X` 前、`+Y` 左、`+Z` 上；
- flat order：elevation-major、azimuth-fast，
  `flat = elevation_index * 80 + azimuth_index`；
- 总维数：`80 * 40 = 3200`。

上边界 elevation clamp 到最后一 bin；+180° azimuth wrap 到 -180°。NaN、Inf、
零 range、≤0.2 m 自体/近场点和 >10 m 点不参与 nearest obstacle。多个点进入
同一 bin 时只保留最近距离。

六个轴向单测的参考 flat index 为：前 1640、后 1600、左 1660、右 1620、
上 3160、下 40。

`ObservationV2` 合同版本为 `lidar_surrogate_v2.0`，包含：

```text
lidar_surrogate[3200]
lidar_valid_mask[3200]
unknown_mask[3200]
semantic[3200]
nearest_obstacle_distance[3200]
observed_free_range[3200]
timestamp / frame_id / metadata / version
```

ROS 标准 multi-array 无 header；权威 sample stamp 和 body frame 由 diagnostics 的
`observation_stamp_sec`、`output_frame` 提供，aligned cloud 与 MarkerArray 中的
Marker header 携带相同 stamp/frame。未来 recorder 必须按这些版本化元数据同步，
不能按回调到达时间抓“最新 odom”。

## 6. obstacle / free / unknown

每 bin 的 semantic code 是：

| code | 含义 | surrogate 编码 |
|---:|---|---:|
| 2 | known obstacle | `(0, 10]` m 的最近回波距离 |
| 1 | observed free | `10` m |
| 0 | unknown | `20 - observed_free_range`，范围 `(10, 20)` |

此外用独立 mask 保留三态，不要求后续网络从一个标量猜语义；unknown 的
`lidar_valid_mask=0`、`unknown_mask=1`。因此“没有点”不会自动等于 free。

unknown estimator 是明确标注的工程近似
`historical_fov_radial_sampling_v1_approximation`：对每个当前 body bin-center ray
按 0.25 m 采样，把 sample 变换到每一个历史 Mid360 sensor frame，检查是否落入
经过审计的 Mid360 FoV；多帧取 observed-region 并集。每个历史 scan 中同一粗
angular cell 的最近回波会截断其后的可见区域，以表达遮挡。

FoV 来自项目实际 Gazebo Mid360 pattern CSV：azimuth 近 360°，zenith
37.836°..97.2123°，即 elevation -7.2123°..52.164°。

与 Flying on Point Clouds 一致的是：5 帧历史、10 Hz reference、0.05 m voxel、
4.5°、3200 directions、10 m clip、历史 FoV/位姿区分 unknown，以及固定长度
point-cloud surrogate。不同点是：论文未公开可复现的 `20-d_unknown` 细节，
而 filtered PointCloud2 也不保留每条 ray 的 miss/max-range record，所以这里采用
可审计的 FoV radial sampling + nearest-return occlusion，绝不声称与论文原版
bit-for-bit 一致。

## 7. 调试和可视化接口

每个 UAV namespace 下发布：

- `learning_speed/observation_v2/surrogate`；
- `learning_speed/observation_v2/valid`；
- `learning_speed/observation_v2/lidar_valid_mask`；
- `learning_speed/observation_v2/unknown_mask`；
- `learning_speed/observation_v2/semantic`；
- `learning_speed/observation_v2/diagnostics`；
- `learning_speed/observation_v2/aligned_history`；
- `learning_speed/observation_v2/visualization`。

RViz markers 以紫色表示 unknown、绿色表示 observed free、红色表示 nearest
obstacle rays；aligned history 是当前 body frame 的 PointCloud2。diagnostics
同时输出 frame、stamp、history、pending、pose mode、ROS-stamp/墙钟频率、各阶段
耗时、CPU、RSS、bin 计数和 reset 次数。所有 topic 均为只读 debug output，运行
检查确认没有 `/mavros/setpoint_position/local` 或
`/mavros/setpoint_raw/local` publisher，也没有任何 v2 topic subscriber 接入 EGO。

## 8. 测试结果

### 8.1 构建和单元测试

`learning_speed_rl;ego_gazebo_bridge;offboard;astra_tower_mission` 白名单构建通过；
`learning_speed_rl` 包测试为 20/20，launch 参数展开检查通过：

- v2 12 项：六方向 indexing、边界/NaN/Inf/0、bin order、pose interpolation、
  禁止 latest-pose 替代、插值 gap fail-closed、time reset、平移 ghost、yaw ghost、
  三态固定维度、nearest/range、回波后遮挡 unknown；
- 原有 v1 observation 2 项、安全 filter 4 项、artifact boundary 2 项全部继续通过。

平移测试中，同一世界障碍在 UAV 前移 1 m 后从 body x=5 m 变为 x=4 m；正确对齐
融合只留下 x=4 m，未经补偿会同时留下 5 m/4 m ghost。yaw 测试中 UAV 转 90°，
相同世界障碍在当前 body 中重合为 `[0,-5,0]`，没有重复角度。

### 8.2 A. outdoor_village 当前单机在线运动验证

只使用现有 `outdoor_village.world`，复用未修改的
`offboard/config/relative_waypoint_mission.yaml` 和既有 PX4/MAVROS OFFBOARD
状态机；Observation v2 只读运行。飞行结束日志为 `PX4 landing complete`，最终
`armed=false`、`landed_state=ON_GROUND`。87.205 s ROS 时间内记录 873 帧 odom、
873 帧 registered/正式 filtered cloud、872 帧 Observation；航迹总长 22.58 m，
yaw 覆盖 181.09°。所有 ready diagnostics 均为 `valid=true`、`history=5`、
`pose_lookup=exact`、pending/drop=0。

| 在线测试 | 结果 | 当前证据 |
|---|---|---|
| A 直线平移 | PASS | `straight_wp1` 平移 2.74 m、yaw 仅 -0.07°；固定世界点的对齐历史与在线五帧 world-cloud union 在 0.20 m voxel 下 overlap 中位数/p05 均为 1.0，无补偿基线仅 0.818 中位数 |
| B yaw 原地或低速旋转 | PASS | 近似同 xy 段伴随竖直运动并 yaw 89.58°；跟踪同一固定 world voxel，其 body azimuth 172.55°→83.51°（-89.04°），与 inverse-yaw 期望误差 0.54°，小于一个 4.5° bin；azimuth bin 78→58，历史 exact 且无丢弃；前后左右轴向语义由同版单测锁定 |
| C 平移+yaw | PASS | `translation_yaw_wp2` 平移 1.93 m 同时 yaw 87.85°；全体 52 个运动 ghost 样本的正确对齐 overlap p05/中位数均为 1.0，未见历史障碍双影 |
| D 建筑拐角/遮挡 | PARTIAL | unknown 三态和回波后遮挡算法在线持续有效，但现有航迹未绕过可确认的建筑拐角，不能证明“遮挡增加→转角后减少”的专项过程 |
| E 空旷→障碍→再空旷 | PASS | 代表段最近障碍中位数 2.82→1.04→2.96 m；unknown ratio、free/obstacle bins 同步变化，没有把所有无回波 bin 置为 free |

在线 Observation 性能如下：

- 输入和输出 ROS-stamp cadence 中位数均为 10.000 Hz；
- Gazebo RTF 中位数 0.55（p05 0.51、p95 0.62），墙钟输出中位数 5.539 Hz；
- history alignment 0.387/0.919 ms（median/p95）；
- unknown 59.740/67.208 ms；总计算 65.810/72.730 ms，最大 81.700 ms；
- Observation 进程 CPU 中位数 49.8%、p95 59.5%，最大 RSS 83.64 MiB；
- unknown ratio 跨代表运动段约 0.638..0.731，最近障碍、free/obstacle/unknown
  均发生连续变化。

完整证据位于
`runtime_artifacts/learning_speed/observation_v2/online_motion/outdoor_village_current_20260813_231825/`：
`online_motion.bag`、`online_motion_summary.json`、`motion_segments.csv`、
`ghost_alignment_samples.csv`、`yaw_bin_tracking_summary.json`、两张 PNG、
Gazebo/process 采样和安全落地状态均保留。

### 8.3 Worksite current formal validation

结果：**PARTIAL（正式链和真实运动已形成，但未进入塔旁绕塔段）**。

本次使用 2026-08-15 工作区中的当前修复版 `worksite.world`，通过正式三机共享栈
的 UAV1-only 模式启动 Mid360、FAST-LIO、existing filters、EGO、bridge 与既有
任务。Observer 的输入严格为
`/uav1/stage3/cloud_registered_filtered` + `/uav1/Odometry`，frame 严格为
`uav1/camera_init -> uav1/body`，sensor frame 为 `uav1/mid360_link`；没有用历史
raw bag 或 `/uav1/cloud_registered` 代替。89/89 条 diagnostics 均 ready、exact、
history=5、`control_output=none`，pending/drop/reset 均为 0。441 条录包期输出为
`valid=true`，3 条启动期 `valid=false`；v2 没有 MAVROS/EGO 控制出口。

飞行确实起飞并稳定到约 3 m，但进入塔场前，既有安全状态机拒绝 PRE_ENTRY 目标
`(-3.1668, 3.08057, 3)`：地图最近占据点 `(-4.125, 3.125, 2.875)` 距目标
0.967342 m，小于既有 1.0 m endpoint clearance，错误条件为
`ENTRY_GATE_TARGET_OCCUPIED: MAP_ENDPOINT_OCCUPIED`。没有修改 waypoint、
ENTRY_GATE、world 或任务逻辑来绕开该保护。任务随后进入 HOLD/返航；近距离返航
goal 未在等待窗口内获得新 trajectory acknowledgement，既有异常路径请求
`AUTO.LAND`，最终 `connected=true`、`armed=false`、`landed_state=ON_GROUND`、
bridge=`DONE`、task=`ERROR`。这不是任务成功，也不是正常 OFFBOARD 完整降落证据，
但飞行安全终止并已解除武装。

正式 evidence bag 从该安全终止后的下降段开始，共 44.21 s、442 帧 odom、442 帧
formal filtered cloud。其 odom 轨迹长 4.06 m、端点位移 2.83 m，但水平跨度仅
0.062 m、z 从 2.814 m 降到 -0.022 m、yaw span 仅 0.144°。方向评估器覆盖 510 帧，
塔水平距离始终为 22.010..22.129 m（中位 22.049 m），并非 tower-close 样本：

| 方向 | semantic 计数 | nearest distance 中位数 | 验收解释 |
|---|---|---:|---|
| inward | obstacle 510 / free 0 / unknown 0 | 9.268 m | 能实时计算塔心方向并返回有限 obstacle，但距离过远，不能归因于近距离塔体 |
| CW tangent | obstacle 510 / free 0 / unknown 0 | 5.185 m | 当前远场方向有障碍；不能回答“塔近但切线安全” |
| CCW tangent | obstacle 35 / free 475 / unknown 0 | 4.189 m（仅 obstacle 帧） | 能区分两侧不同语义，但不是圆周切线运动证据 |

全局 3200 bins 的 unknown 中位数为 2312（ratio 0.7225，p05/p95 ratio
0.6826/0.7269），说明正式链没有把无回波一律置 free；但 inward/CW/CCW 三个局部
方向均未出现 tower-behind unknown，且航迹没有经过塔后遮挡区。因此 tower
occlusion unknown 仍为**无足够专项证据**。同理，本段几乎无水平运动/yaw，不能
用来验证持续朝塔 yaw 下的 5 帧 ghost、angular-bin 旋转或切向障碍接近响应；这些
项保持未通过，不用单元测试或 outdoor 证据冒充 worksite 证据。

性能统计如下。频率均明确区分 ROS stamp 与墙钟；bag 内 Gazebo
`PerformanceMetrics` 在下降后快速跨越不同负载区间，瞬时 RTF 中位 0.922、
p05/p95 0.288/1.724，不代表稳态绕塔。更直接的 Observation 墙钟输出中位
4.758 Hz，而 ROS-stamp 输入/输出为 10.000 Hz，反映这段仿真墙钟约半速；算法
p95 84.66 ms 本身仍低于 100 ms。

| 指标 | 结果 |
|---|---:|
| Mid360 IMU / LiDAR ROS stamp | 100.000 / 9.999 Hz |
| FAST-LIO odom / formal filtered ROS stamp | 9.992 / 9.992 Hz |
| Observation input / output ROS stamp 中位 | 10.000 / 10.000 Hz |
| Observation wall Hz 中位（p05/p95） | 4.758（4.683 / 5.153） |
| history alignment median/p95 | 0.364 / 0.918 ms |
| angular binning median/p95 | 0.974 / 1.847 ms |
| unknown estimator median/p95 | 62.310 / 76.621 ms |
| Observation total median/p95/max | 67.845 / 84.657 / 108.173 ms |
| observer CPU median/p95 | 45.8% / 51.6% |
| observer max RSS | 80.57 MiB |
| pending max / dropped / reset | 0 / 0 / 0 |
| diagnostics invalid / bag valid false | 0 / 3（仅启动 warm-up） |

最终 bag、JSON/CSV 和明确标注“not a tower orbit”的离线时序图位于
`runtime_artifacts/learning_speed/observation_v2/worksite_current/formal_single_control_20260815/`。
其中 `evaluation/formal_single_summary.json` 是权威汇总，
`evaluation/formal_single_direction_timeline.png` 展示塔距、三方向 semantic 和下降
轨迹。2026-08-13 的启动失败目录仅保留为修复前历史证据，不再代表当前链状态。

### 8.4 独立计算压力测试

固定随机种子、20,000 世界点、5 个带平移/yaw 的历史 pose、融合后 12,881 点，
运行 20 次：

| 阶段 | median ms | p95 ms |
|---|---:|---:|
| 历史对齐 | 1.404 | 1.694 |
| fusion downsample | 14.146 | 14.493 |
| angular binning | 2.390 | 2.493 |
| unknown | 30.696 | 31.284 |
| build total | 48.838 | 49.845 |
| wall total | 48.997 | 50.000 |

p95 计算容量约 20.0 Hz，最大 RSS 49.90 MiB，说明纯 Observation v2 算法具备
10 Hz 外层策略所需的计算余量。在线 PC2 decode、ROS serialization、RViz 发布和
Gazebo RTF 则必须看上一节的实测，不能用该压力测试替代。

### 8.5 Formal three-UAV concurrent validation

结果：**PASS（限于无控制正式并发与性能边界）**。使用当前修复版
`worksite.world`、正式 `triple_tower_inspection.launch`、D435 开启、LiDAR
`downsample=1`，保持 `enable_control=false`，先运行不启用 v2 的基线，再在同样
配置上同时启动三套只读 observer。两次运行都形成三套 Mid360 → FAST-LIO →
formal filtered cloud → EGO/MAVROS/PX4 图；本阶段按要求没有做完整三机飞行回归。

namespace 自动审计结果：每机 49/49 diagnostics 都只报告自身
`uavN/camera_init -> uavN/body`、自身 `uavN/mid360_link`、`pose_lookup=exact`、
`control_output=none`。三套 formal input 各自只有对应 observer subscriber，三套
surrogate/valid/diagnostics 各自只有对应 publisher；没有出现其他 UAV frame、topic
或 timestamp 来源。由独立 ROS 进程实例可知其 PoseBuffer/history buffer 也不共享。

| UAV | filtered / Observation ROS stamp Hz | Observation wall Hz | total median/p95 ms | unknown median/p95 ms | align median/p95 ms | CPU median/p95 | max RSS MiB | pending/drop | valid true/false |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| uav1 | 8.181 / 8.181 | 1.807 | 73.143 / 81.629 | 67.083 / 73.665 | 0.528 / 1.040 | 19.2 / 21.6% | 73.99 | 0 / 0 | 203 / 0 |
| uav2 | 8.174 / 8.182 | 1.805 | 74.669 / 84.503 | 66.943 / 75.652 | 0.555 / 1.195 | 19.7 / 22.2% | 74.23 | 0 / 0 | 202 / 0 |
| uav3 | 8.178 / 8.185 | 1.809 | 74.266 / 81.937 | 68.128 / 74.200 | 0.524 / 0.960 | 19.1 / 22.0% | 81.54 | 0 / 0 | 203 / 0 |

三机 v2 总进程 CPU（每机 diagnostics 中位数求和）为 58.0%，每机 max RSS 求和
229.76 MiB。三个 observer 的 total p95 均小于 85 ms，对应纯计算容量仍高于
约 11.8 Hz；实际墙钟输出约 1.81 Hz 是 8.18 Hz ROS cadence 乘以约 0.223 RTF，
不能归因成 Observation 算法只算得出 1.81 Hz。

| 正式三机整机指标 | v2 OFF 基线 | v2 ON | 观测差异 |
|---|---:|---:|---:|
| Gazebo RTF median（p05/p95） | 0.239（0.230 / 0.253） | 0.223（0.208 / 0.238） | -6.9% |
| uav1 formal filtered ROS Hz | 8.337 | 8.181 | -1.9% |
| uav1 formal filtered wall Hz | 1.967 | 1.822 | -7.4% |
| 三机 Mid360 IMU ROS Hz | 99.95..100.00 | 100.00 | 未见掉频 |
| 三机 FAST-LIO/filtered ROS Hz | 8.337..8.354 | 8.174..8.181 | 轻微下降，无断流 |
| gzserver 代表快照 CPU / RSS | 156.2% / 约 6.7 GiB | 153.3% / 约 6.7 GiB | 单点快照噪声内，无新增量结论 |

两次对照为顺序运行，不是相同 scheduler 状态下的严格配对实验，因此 -6.9% 只能
称为“观测到的差异”，不能全部因果归于 v2。它与 formal filtered 墙钟频率约
-7.4% 一致，表明 v2 带来可测但有限的整机开销。三机没有 callback backlog、
cloud drop、time reset、observer invalid 或进程失效；三个 MAVROS 均保持
`connected=true`、`armed=false`、`AUTO.LOITER`。FAST-LIO 和 EGO 节点持续存活，
formal filtered 持续发布；由于没有目标和控制，本次未覆盖 active EGO planning
性能，不能声称 EGO 飞行规划性能已验证。

基线证据位于
`runtime_artifacts/learning_speed/observation_v2/three_uav_formal/baseline_no_v2_20260815/`，
启用证据位于同级 `three_v2_enabled_20260815/`。权威汇总为
`three_v2_summary.json` 与同级 `comparison_summary.json`；完整轻量 bag、topic
ROS/wall rate、process/top/vmstat 和最终 MAVROS/控制 publisher 快照均保留。

### 8.6 三实例合成输入补充基准（非正式三机验收）

为单独回答三套 Observation 计算是否在约 10 Hz 输入下自身积压，使用本轮
outdoor 有效 bag 向 `/uav1..3` 三个隔离 namespace 各 relay 一份相同 odom/cloud，
并发运行 60 s。该测试不含三套 live Mid360、FAST-LIO、EGO、MAVROS/PX4，不能替代
8.5。namespace 审计显示三套独立 publisher，Observation 输出 subscriber 均为 0。

| UAV | 输入 Hz | 输出 bag Hz | total p50/p95 ms | unknown p50/p95 ms | align p50/p95 ms | pending max / drop | external ps CPU p50/p95 | RSS p50/p95 MiB |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| uav1 | 9.951 | 9.835 | 61.90 / 81.91 | 56.30 / 75.53 | 0.363 / 0.931 | 0 / 0 | 58.2 / 78.4% | 50.4 / 76.9 |
| uav2 | 9.951 | 9.801 | 63.81 / 90.98 | 58.53 / 83.10 | 0.364 / 0.888 | 1 / 0 | 57.8 / 78.1% | 59.1 / 80.1 |
| uav3 | 9.951 | 9.801 | 64.94 / 87.39 | 58.94 / 81.44 | 0.366 / 0.835 | 0 / 0 | 58.5 / 79.2% | 54.8 / 77.9 |

三实例 external `ps` 中位 CPU 合计 174.5%（约 1.75 核），中位 RSS 合计
164.34 MiB；所有 ready pose 均 exact、history=5、drop=0。初始化阶段 cloud 先于
odom 会发布 fail-closed `valid=false`，不是 backlog 丢帧。结果位于
`runtime_artifacts/learning_speed/observation_v2/three_uav_profiling/synthetic_three_instance_10hz_20260813_233728/`。

## 9. 对既有系统的影响

- Observation v1 `[4,16,48,48] -> CNN contract` 源码未修改、未删除，原测试通过；
- v1/v2 可独立启用，v2 launch 不被正式任务 launch include；
- v2 只有 debug publishers，无 `v_max`、goal、trajectory、EGO 或 MAVROS publisher；
- 未训练/加载 SAC 或 PPO，未建立正式 RL inference；
- 未修改 FAST-LIO、EGO vendor、外部 PX4、world、任务几何或三机 ROS graph；
- outdoor 单机受控飞行中 v2 只读，既有任务完成并安全落地，未观察到控制冲突；
- 当前 worksite 单机和正式三机 Mid360/FAST-LIO/formal filtered 前置链均已恢复；
- worksite 单机 v2 在真实下降运动中持续 exact/valid、无 backlog/drop，但任务在
  进塔前按既有净空保护安全终止，不能据此宣称塔旁几何验收通过；
- 正式三机 v2 并发无 backlog/drop/invalid，三机 frame/topic 隔离成立；与 OFF
  基线相比观测到约 6.9% RTF 和 1.9% filtered ROS-rate 下降，属于有限且可测开销，
  不能写成“零影响”；
- 三机没有 active target/control，故只证明 EGO 进程与数据链持续存活，不证明
  active planning 或完整飞行性能。

## 10. 后续 trajectory fusion 接口准备度

已具备加入 fusion 的数据边界，但尚未实现 fusion：`ObservationV2` 把 lidar
surrogate、valid/unknown/semantic、stamp、frame 和 metadata 独立封装；未来可在
其外部增加 EGO preplanned B-spline trajectory、tracking error、actual velocity
和 previous `v_max`，再进入 environment encoder / Fusion / SAC Actor。当前 v2
没有被写成“直接进入 policy 并输出控制”的死结构。

本阶段最终验收清单：

- ☑ outdoor_village 在线平移/yaw 无明显 ghost；
- △ outdoor_village unknown 动态变化存在，但建筑拐角专项过程缺失；
- △ worksite 当前正式 filtered/odom 在线运行和真实下降运动已完成，但未进入绕塔；
- ☐ 当前绕塔 inward/tangent 与塔后 unknown 通过；
- ☐ 持续 yaw 的 worksite 历史点云稳定通过；
- ☑ 正式三机同时启用 v2，namespace 隔离且无 backlog/drop；
- ☑ 正式三机 ON/OFF 对照显示有限开销，FAST-LIO/formal filtered/MAVROS 链未失效；
- ☑ 单机在线、纯计算、合成三实例和正式三机均显示 Observation 算法本身具有
  约 10 Hz 计算余量。

因此最终结论保持 `Observation v2 prototype：PARTIAL`。后续只能在不改正式 world、
不改任务几何和控制边界的前提下，等待既有安全任务能够自然形成代表性塔旁运动后，
再补 tower occlusion、inward/tangent、持续 yaw ghost 和切向障碍专项证据；不应为
迎合测试放宽净空或修改场景。正式三机无控制并发不需要重跑完整旧三机任务。
在单机塔旁专项证据通过前，不得开始 trajectory fusion、SAC/PPO、RL inference
或 `v_max` 控制。
