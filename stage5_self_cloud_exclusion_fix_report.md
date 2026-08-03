# Stage 5 第一轮 self cloud exclusion 修复与短时验证报告

日期：2026-08-03  
分支：`ego-swarm`  
审计/修改起点 HEAD：`f68f07137db57698c15f39d093f812ff9851da15`

## 1. 结论

本轮已实现并接入职责独立的、使用实时 odometry 姿态和 FAST-LIO LiDAR→IMU 外参的结构化 self cloud filter；已增加三机独立话题、peer/self/ground/最终云计数、过滤前后最近点、膨胀图最近点和 0.55 m 内点数诊断，并完成目标包编译、单元测试、三机 launch 隔离检查和 UAV2 失败窗 5.795 s 无控制回放。

但是，失败 bag 的真实回放否定了“报告所指最近点簇位于实际机体结构遮罩内”这一前提：47 帧中 self filter 删除点始终为 0，最近点过滤前后完全相同，原有膨胀图在 0.55 m 内仍为 13～21 点，planner 仍报告当前位置占据。因此，**不能声称本轮已经消除该次 `CURRENT_POSITION_IN_OCCUPANCY`，也不能进入下一轮八边形直线边段走廊修改**。

没有通过扩大球形/盒形阈值强行删除该簇。这样做会删除机体结构外、在 world 坐标中高度稳定的真实近距离表面，违反“不得删除真实近障”和“不能降低安全标准”的限制。

## 2. 修改前后的点云链路

修改前：

```text
/uavN/cloud_registered
  -> teammate_cloud_filter（peer envelope；无 self pose）
  -> /uavN/cloud_registered_peer_filtered
  -> ground_cloud_filter（z < 0.20 m）
  -> /uavN/stage3/cloud_registered_filtered
  -> EGO GridMap
```

修改后：

```text
/uavN/cloud_registered
  -> teammate_cloud_filter（只负责 peer；保留原逻辑）
  -> /uavN/cloud_registered_peer_filtered
  -> self_cloud_filter（odom 同步 + LiDAR/IMU 外参 + 组合结构遮罩）
       -> /uavN/stage5/cloud_before_self
       -> /uavN/stage5/cloud_after_self
       -> /uavN/cloud_registered_self_filtered
  -> ground_cloud_filter（只负责 ground）
  -> /uavN/stage3/cloud_registered_filtered
  -> EGO GridMap
  -> /uavN/stage3/occupancy_inflate
```

peer、自体和地面过滤分别输出独立诊断；没有把 peer/self 合并成一个近距离阈值。

## 3. self exclusion 实现与几何

实现位于 `astra_swarm_perception` 独立节点 `self_cloud_filter_node`。输入云和 `/uavN/Odometry` 使用最多 0.03 s 的 ApproximateTime 同步；cloud frame 必须等于 odom parent frame，否则 fail closed、不发布过滤云。

对注册到规划坐标的每个点执行：

```text
p_imu   = R(q_odom)^T * (p_cloud - t_odom)
p_lidar = R_LI^T * (p_imu - t_LI)
```

其中当前 FAST-LIO 外参为：

```text
t_LI = [-0.011, -0.02329, +0.04412] m
R_LI = I
```

遮罩全部定义在 MID360 测量坐标中，由下列组合几何组成：

- 机身盒体：`0.340 × 0.520 × 0.155 m`；
- 4 个按机臂方位旋转的窄盒体；
- 4 个旋翼薄圆柱：中心使用 SDF 的 `(±0.13, ±0.20/0.22)`，半径 `0.140 m`；
- MID360 外壳/安装支架盒体和半径 `0.070 m` 的小圆柱；
- 前向相机安装支架盒体。

没有设置“删除机体周围全部近距离点”的球形范围。配置校验拒绝非有限值、负尺寸、空遮罩和非单位外参旋转列。

## 4. 过滤尺寸依据

尺寸来自当前模型与配置，而非经验放大：

- `iris.stl` 实测 bounds：x `[-0.1563, +0.1431]`、y `[-0.2371, +0.2406]`、z `[-0.0670, +0.0466] m`；
- `iris_without_GPS_N.sdf`：四个 rotor 中心和 `0.128 m` 半径；
- `iris_mid360_N.sdf` / `uav_sensor_frames.launch`：`base_link -> MID360 = [0, 0, +0.08] m`；
- `FAST_LIO/config/mid360.yaml`：上述 LiDAR→IMU 外参；
- 模型和旋翼外缘只增加约 `0.012～0.021 m` 离散/装配余量。

失败报告在 13008.733 s 的最近点，若只做规划坐标直接相减，是 `(-0.5687,-0.0451,+0.4483) m`。加入当时 odom yaw（约 85.7°）并应用外参后，其 MID360 坐标约为：

```text
(-0.0768, +0.5902, +0.4001) m
```

它高于 MID360/支架上界约 0.32 m，横向也超过机体/旋翼物理外缘约 0.24 m；不能安全并入模型遮罩。

## 5. 修改文件

新增：

- `AstraDrone_ros1_ws/src/Swarm/astra_swarm_perception/include/astra_swarm_perception/self_cloud_filter.h`
- `AstraDrone_ros1_ws/src/Swarm/astra_swarm_perception/src/self_cloud_filter.cpp`
- `AstraDrone_ros1_ws/src/Swarm/astra_swarm_perception/src/self_cloud_filter_node.cpp`
- `AstraDrone_ros1_ws/src/Swarm/astra_swarm_perception/config/iris_mid360_self_exclusion.yaml`
- `AstraDrone_ros1_ws/src/Swarm/astra_swarm_perception/test/self_cloud_filter_test.cpp`
- `scripts/run_sh/stage5_self_filter_record_bag.sh`
- 本报告；
- 两份只增不删的短时证据 bag，位于 `test_evidence/stage5_self_cloud_exclusion_fix_20260803/`。

修改：

- `astra_swarm_perception/{CMakeLists.txt,package.xml}`；
- `astra_swarm_perception/src/teammate_cloud_filter_node.cpp`（只加 peer 精确诊断）；
- `ego_gazebo_bridge/{CMakeLists.txt,package.xml}`；
- `ego_gazebo_bridge/src/ground_cloud_filter_node.cpp`（只加 ground 精确诊断）；
- `ego_gazebo_bridge/launch/ego_gazebo_bridge.launch`；
- `astra_swarm_bringup/launch/{dual_tower_inspection,triple_tower_inspection,uav_tower_stack}.launch`。

未修改 EGO/EGO-Swarm vendor、FAST-LIO 源码、world、塔/障碍模型、航点/Tier/扇区/方向/角色、重规划周期、0.40 m 膨胀参数、0.55 m 判定阈值或机间安全阈值。

## 6. 诊断话题

每个 UAV 均有独立命名空间：

- `/uavN/cloud_registered`：self 之前的原始来源；
- `/uavN/cloud_registered_peer_filtered`：peer 后；
- `/uavN/stage5/cloud_before_self`：self 节点实际输入；
- `/uavN/stage5/cloud_after_self`：self 后；
- `/uavN/cloud_registered_self_filtered`：送往 ground 的 self 输出；
- `/uavN/stage3/cloud_registered_filtered`：最终进入 EGO；
- `/uavN/stage3/occupancy_inflate`：最终膨胀占据图；
- `/uavN/stage5/cloud_filter_diagnostics`：peer/self/ground/inflated 统计。

诊断包含：输入、peer 删除、self 删除、ground 删除、非有限点删除、静态点加入、最终输出、self 前后最近点、膨胀图最近点、0.55 m 内膨胀点数和 cloud/odom stamp 差。

新增 recorder 同时列出 UAV1/UAV2/UAV3 的 `/tf`、`/tf_static`、各级点云、raw/inflated occupancy、odom 和 planner status。此次历史失败 bag 本身不含 `/tf`/`/tf_static`，离线回放不能凭空补造，因此短时证据 bag 中仍无这两个消息；后续真实无控制/单机复核可直接用该 recorder 补齐。

## 7. 单元测试与命名空间隔离

白名单构建：

```text
catkin_make -DCATKIN_WHITELIST_PACKAGES='astra_swarm_perception;ego_gazebo_bridge' -j2
结果：通过
```

目标测试：

```text
astra_swarm_perception：4 个 GTest case，0 failures
ego_gazebo_bridge：26 个既有 GTest case，0 failures
```

新增 self-filter GTest 的 4 个测试均通过：

1. 机身、旋翼、MID360/支架模型点被删除；
2. 90° yaw 时仍依赖完整 odom 旋转正确删除，不使用轴对齐距离；
3. 位于 0.35～0.45 m、但在结构体积外的真实近障全部保留；
4. 非法/负尺寸几何被拒绝。

三机 launch 展开检查：UAV1/UAV2/UAV3 各恰有一个 `teammate_cloud_filter`、`self_cloud_filter`、`ground_cloud_filter`。dump-params 证明三套 input/odom/output/before/after/diagnostic/inflated topic 分别指向 `/uav1`、`/uav2`、`/uav3`，没有跨命名空间订阅或发布。

## 8. UAV2 失败窗短时无控制回放

证据：`test_evidence/stage5_self_cloud_exclusion_fix_20260803/uav2_failure_window_full_filter_chain.bag`

- 时窗：13006.825～13012.620 s，5.795 s；
- 只播放 `/clock`、UAV2 odom、原始注册云、原膨胀图和 planner status；
- 未启动 PX4/Gazebo 控制、未解锁、未发布控制话题；
- 完整运行 `peer -> self -> ground`，47 个同步点云帧；
- 记录原始、peer 后、self 前后、self 输出、最终 EGO 输入、原膨胀图和诊断。

统计：

| 指标 | 最小 | 中位 | 最大 |
|---|---:|---:|---:|
| 输入点 | 3097 | 3214 | 3352 |
| peer 删除 | 0 | 0 | 0 |
| self 删除 | 0 | 0 | 0 |
| ground 删除 | 57 | 68 | 95 |
| 最终 EGO 输入点 | 3032 | 3148 | 3267 |
| self 前最近点 / m | 0.6870 | 0.7164 | 0.7539 |
| self 后最近点 / m | 0.6870 | 0.7164 | 0.7539 |
| 原膨胀图最近点 / m | 0.1147 | 0.1468 | 0.2677 |
| 原膨胀图 0.55 m 内点 | 13 | 17 | 21 |

47/47 个 self 前后点云的点数和 SHA-256 均相同。planner 在窗口中仍产生 40 条 collision/failure 状态记录。

对 13007.0～13012.2 s 的 43 帧邻近点做 5 cm 体素重复性检查：在 world 坐标中有 117 个体素至少出现于 35 帧，而在 odom 姿态校正后的 body 坐标中为 0 个。这与“固定环境表面”更一致，与“随本机刚体运动的结构回波”不一致。该证据也解释了为什么物理 self mask 不应命中它。

## 9. 真实近距离障碍物保留

- 单元测试证明距传感器约 0.35～0.45 m、但不在机身/机臂/旋翼/支架组合体内的点全部保留；
- 失败窗中报告所指结构外近点簇 47/47 帧被保留，没有用扩大的球或经验盒删除；
- ground filter 仍只删除 `<0.20 m` 地面带；peer filter 保持原 1.2 m peer envelope 逻辑；
- 没有改变任何安全阈值来让结果“通过”。

## 10. 验收判断与下一步门槛

| 问题 | 判断 |
|---|---|
| 是否已建立明确、结构化的本机点云排除能力 | 是 |
| 是否证明模型内 self 点会删除、结构外近障会保留 | 是，单测通过 |
| 是否稳定删除失败报告所指点簇 | 否，47 帧删除数均为 0 |
| 最终膨胀图 0.55 m 内点是否归零 | 否，历史原图仍为 13～21 点；本轮未伪造重建图 |
| 是否不再出现 `CURRENT_POSITION_IN_OCCUPANCY` | 否，失败窗仍复现 |
| 是否可进入“八边形直线边段走廊约束”修改 | **否** |

进入下一轮前应先做一个新的、仍不改安全阈值的感知定位专项：在 live 无控制或 UAV2 单机短窗中同时录制原始 Livox 点、FAST-LIO body/world cloud、完整 `/tf`/`/tf_static`、Gazebo model/link states、最终 filtered cloud 和 raw/inflated occupancy，以确定该结构外、world 稳定表面的真实来源。若它是模型中未审计的 link/碰撞体，应修正模型/外参契约后再更新组合遮罩；若它是真实环境点，则应保留并修正规划/地图对当前位置的解释，不能归入 self filter。
