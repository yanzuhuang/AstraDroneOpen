# EGO-Swarm 三机工程接入与安全验收报告

日期：2026-07-30

项目：`/home/yanzu/AstraDroneOpen`

分支：`ego-swarm`

代码验收 HEAD：`a3d2f97953f45f18efb4b58e4645823d4fb0667c`

官方参考：`/home/yanzu/ego-planner-swarm_reference`

## 1. 最终结论

本轮完成了 EGO-Planner 单机接入前基线、官方 EGO-Swarm 核心验证、五个
核心包迁移、三机工程实例化、公共坐标轨迹适配、三机地面仿真和无控制通信
验证。统一入口为：

```bash
roslaunch astra_swarm_bringup triple_tower_inspection.launch \
  takeoff_interval_sec:=3.0
```

同时起飞参数接口也已保留：

```bash
roslaunch astra_swarm_bringup triple_tower_inspection.launch \
  takeoff_interval_sec:=0.0
```

入口默认 `enable_control=false`，不会创建 bridge 的 MAVROS setpoint publisher，
不会解锁或起飞。

任务要求的 2/3/4 m 三层实际飞行没有执行，也不能标记为通过。原因不是构建或
启动失败，而是安全模型给出了确定的禁止条件：

- EGO-Swarm 优化器实际使用 `CLEARANCE = 2 * swarm_clearance`；
- 优化器的椭球距离将 z 差除以 2；
- 当前参数 `swarm_clearance=1.5 m` 对应水平最低间距 3.0 m、同一水平位置的
  垂直最低间距 6.0 m；
- 用户固定层 2/3/4 m 的相邻间隔只有 1.0 m。

协调器运行时输出 `SAFETY_INHIBIT`，所有起飞许可保持 `false`：

```text
fixed mission layers violate 6.00 m required vertical separation
```

因此阶段 7 的 3 秒间隔实际飞行和阶段 8 的同时起飞实际飞行均按要求安全停止，
没有降低 `swarm_clearance`、关闭多机碰撞检查或篡改 2/3/4 m 任务高度来制造
成功结果。相同圆周和 ENTRY/EXIT 几何下，最低建议固定层为 2/8/14 m；如果必须
保留 2/3/4 m，则必须重新设计为经证明不同时占用同一水平走廊的时空分离任务，
并重新完成冲突注入、最小间距和全程飞行验收。

## 2. Git 基线和线性历史

本轮没有创建新分支，没有 rebase、reset、force 或 push。验收时代码提交历史：

```text
a3d2f97 validate three uav swarm communication
d01bdaa migrate official ego swarm planner core
fae30f1 test current ego planner sitl baseline
64bd99c 接口一致性修复
b2b7fa0 统一双机配置为之前单机配置
488ddfc origin/ego-swarm
```

关键 SHA：

| 角色 | commit |
|---|---|
| 接入前工程基线 | `64bd99cc21feb7f90b26335ace40112557aa6a82` |
| 单机基线报告提交 | `fae30f144396d1979c34f62b9d93c8fe78f63f07` |
| 五包迁移提交 | `d01bdaa00314c92d31706be7ad7d13b2f7827ffe` |
| 三机无控制工程提交 | `a3d2f97953f45f18efb4b58e4645823d4fb0667c` |
| 官方 EGO-Swarm 参考 | `92fe9f7227b2da819133eb8e0e8c7fc000f6ae20` |

`FAST_LIO/Log/mat_pre.txt` 在每次 FAST-LIO 运行后均恢复到 HEAD；没有提交
build、devel、ROS/PX4 日志、bag、CSV、图片或缓存。

## 3. 阶段门结果

| 阶段 | 结果 | 证据或阻塞项 |
|---|---|---|
| 1 当前 EGO-Planner 单机基线 | 通过 | dry-run、fresh goal、cancel/restart、26→22 m 实际巡塔和降落；详见 `ego_single_sitl_regression_report.md` |
| 2 官方 EGO-Swarm | 核心通过 | 官方五包 Release 白名单构建通过；完整参考工作区被可选 `multi_map_server` 缺失生成头阻塞 |
| 3 五包迁移 | 通过 | 同一官方 SHA 的五包整体迁移，正式工作区只有一套同名包 |
| 4 EGO-Swarm 单机 | 部分通过 | `drone_id=0` dry-run、goal/cancel/restart和接口通过；迁移后的 26→22 m 与固定 2 m 实际飞行没有重新执行 |
| 5 三机工程实例化 | 地面验证通过 | 三个模型、PX4、MAVROS、FAST-LIO、Mid360、D435 和 Topic/frame 独立，三机均未解锁 |
| 6 三机无控制通信 | 部分通过 | 3 发布/3 订阅、全部三对外部预测冲突测试和 fail-closed 门禁通过；未完成带真实 EGO 轨迹的冲突重规划运行注入 |
| 7 间隔 3 秒实际飞行 | 安全阻塞，未执行 | 2/3/4 m 违反 6 m 垂直安全要求 |
| 8 同时起飞实际飞行 | 安全阻塞，未执行 | 阶段 7 未通过，且相同高度冲突仍存在 |
| 9 报告、清理、提交 | 完成 | 本报告、线性本地提交、受保护文件干净；未 push |

阶段 5/6 的工程实现和无控制证据用于定位安全阻塞，不代表跳过阶段 4、7、8
的实际飞行门槛。

## 4. 官方 EGO-Swarm 验证

参考仓库状态：

```text
path:   /home/yanzu/ego-planner-swarm_reference
branch: master
HEAD:   92fe9f7227b2da819133eb8e0e8c7fc000f6ae20
status: clean
```

`libarmadillo-dev` 可用。官方完整工作区构建到可选 `multi_map_server` 时因
`multi_map_server/MultiOccupancyGrid.h` 未生成而失败。以下核心白名单 Release
构建成功：

```text
quadrotor_msgs;plan_env;path_searching;traj_utils;bspline_opt;
ego_planner;waypoint_generator
```

没有修改官方参考仓库，也没有把 fake_drone、local_sensing、SO3 控制器、
`drone_detect` 或 `rosmsg_tcp_bridge` 接入正式项目。

## 5. 五个核心包迁移矩阵

| 包 | 官方迁移内容 | 保留/追加的 Astra 工程补丁 |
|---|---|---|
| `bspline_opt` | EGO-Swarm `drone_id`、`SwarmTrajData`、多机椭球碰撞代价和 `swarm_clearance` | 保留已测试的非法占据段边界保护；未删除多机代价 |
| `path_searching` | 官方同版本 A* 搜索包 | 仅保留工程构建依赖；没有塔或场景硬编码 |
| `plan_env` | 官方地图、对象预测和动态对象接口 | 保留 Mid360/FAST-LIO 外部输入参数化；未合并三机完整点云 |
| `plan_manage` | 官方 swarm FSM、planner manager、轨迹缓冲、发布/接收和碰撞检查 | 相对 Topic、namespace、状态、取消、goal z、odom 有限值、公共 world 变换、轨迹过期门和 traj_server frame 门 |
| `traj_utils` | 官方 `Bspline`、`MultiBsplines`、`DataDisp` 和 swarm 容器 | marker frame/namespace/颜色参数化；`Bspline` 增加显式 `frame_id` |

迁移以官方实现为主体，没有用旧版整文件覆盖 `planner_manager.cpp`、
`ego_replan_fsm.cpp`、`traj_server.cpp` 或 `bspline_optimizer.cpp`。

## 6. 消息和本地接口补丁

官方消息：

- `traj_utils/Bspline`：阶数、开始时间、无人机 ID、轨迹 ID、控制点和 knot；
- `traj_utils/MultiBsplines`：连续 `drone_id_from` 和 B-spline 链；
- `traj_utils/DataDisp`：规划调试数据。

本轮给 `Bspline` 增加 `string frame_id`。本机给 `traj_server` 的 B-spline 使用
本地 `uavN/camera_init`；广播前控制点执行 local→world 数值转换并设置
`frame_id=world`；接收后先验证 frame、ID、阶数、控制点、knot 和时间，再执行
world→local 数值转换。它不是只改 frame 字符串。

保留的本地接口补丁：

- 目标、状态、取消、waypoint、B-spline、PositionCommand 使用相对或显式
  可配置 Topic；
- fresh goal 时间戳门和 cancel 后旧轨迹拒绝；
- `fsm/use_goal_height` 和 `fsm/manual_target_height`；
- `PlannerStatus`；
- planning、status、waypoint、PositionCommand、RViz frame 一致；
- RViz marker namespace 与三机颜色参数；
- yaw 首帧/异常 dt 数值保护；
- odometry NaN/Inf 防御；
- bridge 的 PX4/MAVROS preflight、控制权唯一性和默认 dry-run。

## 7. 三机实例、端口和固定任务参数

| 车辆 | namespace | EGO `drone_id` | 固定任务高度 | spawn(world) | Gazebo model |
|---|---|---:|---:|---|---|
| UAV1 | `/uav1` | 0 | 2.0 m | (0, 0, 0.06) | `iris_mid360_00` |
| UAV2 | `/uav2` | 1 | 3.0 m | (4, 0, 0.06) | `iris_mid360_11` |
| UAV3 | `/uav3` | 2 | 4.0 m | (8, 0, 0.06) | `iris_mid360_22` |

任务层给每机传入单层 `layer_offsets=[0.0]` 和
`candidate/height_offsets_m=[0.0]`，不执行层间升降。ENTRY_GATE、EXIT_GATE、
8 扇区、相机朝塔、返航和降落规则未写入 EGO-Swarm 底层。

| 车辆 | PX4 instance | PX4 system ID | sim UDP/TCP | MAVROS FCU URL | video/camera UDP |
|---|---:|---:|---|---|---|
| UAV1 | 0 | 1 | 14560 / 4560 | `udp://:14540@localhost:14580` | 5600 / 14530 |
| UAV2 | 1 | 2 | 14561 / 4561 | `udp://:14541@localhost:14581` | 5601 / 14531 |
| UAV3 | 2 | 3 | 14562 / 4562 | `udp://:14542@localhost:14582` | 5602 / 14532 |

地面运行实测三路 `/mavros/state` 均 `connected=true`、
`armed=false`、`mode=AUTO.LOITER`。PX4 参数门最终验证三机均为
`EKF2_HGT_REF=3`、`EKF2_EV_CTRL=11`。

## 8. 三套定位和传感器 Topic

下表中的 `N` 为 1、2、3；每一路均实测收到非零仿真时间戳。

| 功能 | Topic | 实测 frame |
|---|---|---|
| Mid360 点云 | `/uavN/livox/lidar` | `uavN/mid360_link` |
| Mid360 IMU | `/uavN/livox/imu` | `uavN/mid360_link` |
| D435 RGB | `/uavN/d435/color/image_raw` | `uavN/d435_color_optical_frame` |
| D435 depth | `/uavN/d435/depth/image_raw` | `uavN/d435_depth_optical_frame` |
| D435 RGB info | `/uavN/d435/color/camera_info` | `uavN/d435_color_optical_frame` |
| D435 depth info | `/uavN/d435/depth/camera_info` | `uavN/d435_depth_optical_frame` |
| FAST-LIO odom | `/uavN/Odometry` | `uavN/camera_init` |
| FAST-LIO 注册点云 | `/uavN/cloud_registered` | `uavN/camera_init` |
| 过滤点云 | `/uavN/cloud_registered_peer_filtered` | 保持本机规划 frame |
| EGO B-spline | `/uavN/planning/bspline` | `uavN/camera_init` |
| PositionCommand | `/uavN/planning/pos_cmd` | `uavN/camera_init` |

每台 teammate filter 同时订阅另外两台 `/uavM/swarm/state`，并从自己的点云中
移除所有存活队友的安全包络；没有共享或融合三机完整点云。

## 9. TF 与公共坐标转换

仿真公共树：

```text
world
├── uav1/map  (0, 0, 0)
│   └── uav1/camera_init ── uav1/body ── uav1/base_link
│                                      ├── uav1/mid360_link
│                                      └── uav1/d435_link ── optical frames
├── uav2/map  (4, 0, 0)
│   └── uav2/camera_init ── ...
└── uav3/map  (8, 0, 0)
    └── uav3/camera_init ── ...
```

地面实测 Gazebo world 位置约为 x=0/4/8 m，而三套 FAST-LIO 本地 odom 均在各自
原点附近；`/uavN/swarm/state` 数值加入 home 偏移后回到 world 的 0/4/8 m。
三次 `/clock` 采样依次为 11966.591、11966.968、11967.333 s，单调递增。

`SwarmFrameTransform` 使用参数化平移和 yaw：

```text
p_world = Rz(origin_yaw) * p_local + origin_xyz
p_local = Rz(origin_yaw)^T * (p_world - origin_xyz)
```

本轮仿真 yaw 均为 0，origin x 为 0/4/8 m。往返变换有独立 gtest。三机只在
`world` 广播轨迹，各规划器收到后转回自己的 local planning frame 再写入
`swarm_trajs_buf_`。

MAVROS 自带的通用 `map_ned`、`odom_ned`、`base_link_frd` 曾在共享
`/tf_static` 上形成重复 child frame。本轮已分别隔离到
`/uavN/mavros/tf_static`；共享 `/tf_static` 上只保留项目拥有的前缀化 TF。

该 world→local 关系是当前 Gazebo 固定 spawn/yaw 假设，不是实机标定结果。

## 10. 三机轨迹通信

公共 ROS 总线：

| Topic | 类型 | 发布者 | 订阅者 |
|---|---|---:|---:|
| `/swarm/trajectories` | `traj_utils/MultiBsplines` | 3 | 3 |
| `/swarm/broadcast_bspline` | `traj_utils/Bspline` | 3 | 3 |

实测发布者/订阅者分别是：

```text
/uav1/drone_0_ego_planner_node
/uav2/drone_1_ego_planner_node
/uav3/drone_2_ego_planner_node
```

接收门检查：

- `drone_id` 从 0 连续编号；
- `MultiBsplines.traj.size == drone_id_from + 1`；
- 每个元素 ID 与数组下标一致；
- 三阶 B-spline、控制点和 knot 数量合法；
- `frame_id=world`；
- 开始时间非零，拒绝过远未来和“轨迹持续时间 + 3 s”后的旧轨迹；
- 自己的直接广播不写成其他无人机；
- 每次成功重规划刷新公共链，不只在启动时发布一次。

运行图验证了三机连通性。由于未向三机注入可执行任务目标，运行中没有生成三条
真实任务 B-spline，因此不能把图连通性描述成“三机真实轨迹冲突重规划已通过”。

## 11. 冲突检测证据和限制

外部安全监控不再只写死 UAV1/UAV2，而是对配置的 `[1,2,3]` 生成：

```text
(1,2), (1,3), (2,3)
```

单元测试覆盖：

- UAV3 所在的全部两两组合；
- 当前距离安全但未来路径相交为 0 m 时拒绝；
- 未来垂直间距不足时拒绝；
- 任一 heartbeat/localization 超时 fail-closed；
- 固定 2/3/4 m 层配置被拒绝。

官方 `BsplineOptimizer::calcSwarmCost()` 仍遍历 `swarm_trajs_` 全部有效 ID，
对其他无人机未来轨迹计算椭球距离并将 `swarm_clearance` 加入代价；未删除
`checkCollision(id)` 或 `REPLAN_TRAJ` 分支。

尚缺证据：没有在三个实际 EGO FSM 同时拥有 odom、goal 和 B-spline 时注入
相交轨迹，所以“EGO-Swarm 对 UAV2 的冲突触发实际重规划并生成新轨迹”仍是
待验证项。外部预测安全拒绝和源码/单测证据不能替代该运行验收。

## 12. 起飞时序实现

一个 swarm manager 使用共同 `schedule_started`，只有全部配置车辆状态健康、
三套 PX4 参数门通过、固定层安全配置通过时才启动时钟。第 `index` 台的许可时刻：

```text
index * max(0, takeoff_interval_sec)
```

因此：

| 参数 | UAV1 | UAV2 | UAV3 |
|---:|---:|---:|---:|
| 3.0 s | t=0 | t=3 s | t=6 s |
| 0.0 s | t=0 | t=0 | t=0 |

没有在三处写 `sleep(3)`。纯策略测试验证了 2.99 s 仍拒绝 UAV2、3.0 s 允许
UAV2、6.0 s 允许 UAV3，以及安全配置成立时 0.0 s 同时许可。后续车辆还要求
实时 `/swarm/safety/clear=true`；降落许可对任意数量车辆串行化。

当前 2/3/4 m 配置不安全，所以共同起飞时钟不会启动。`takeoff_interval_sec`
逻辑通过不等于实际起飞时间已经测量。

## 13. `swarm_clearance` 依据

模型 SDF 中每个旋翼碰撞圆柱半径为 0.128 m，旋翼中心最大 y 偏移 0.22 m，
得到约 0.696 m 的最大旋翼外廓直径。接入前单机任务实测参考误差 p95 为
0.1794 m，任务段历史最大跟踪误差约 0.295 m。还必须覆盖定位误差、通信延迟、
轨迹采样和重规划误差，因此没有采用官方示例的 0.5 m，也没有为了 1 m 层间隔
降低阈值。

当前配置：

```text
optimization/swarm_clearance = 1.5 m
external minimum_3d_separation = 3.0 m
configured minimum_vertical_separation = 4.0 m
effective vertical separation = max(4.0, 4*1.5) = 6.0 m
```

6.0 m 是相同 XY/时间轨迹的最低层间建议，不是实机安全认证值。

## 14. 单机回归结果

接入前 26→22 m 单机控制基线：

| 指标 | 结果 |
|---|---:|
| 仿真时长 | 1039.2 s |
| 目标数 | 25 |
| 最大高度 | 26.1421 m |
| p95 参考误差 | 0.1794 m |
| 塔几何最小净空 | 4.8895 m |
| HOLD / emergency | 0 / 0 |
| 返航和降落 | `DONE`、disarmed、`ON_GROUND` |

迁移后 `drone_id=0` dry-run 确认：

- `/planning/goal` 和 `/planning/cancel` namespace 正确；
- fresh goal 进入 `EXEC_TRAJ`；
- cancel 后进入 `WAIT_TARGET`；
- 新 fresh goal 可恢复；
- `/planner/status` 和 `/planning/pos_cmd` 接口存在；
- 单机没有等待不存在前序无人机的启动死锁。

迁移后的 26→22 m 和固定 2 m 受控单机任务没有重新飞行，因此不能比较迁移后
规划成功率、耗时和 HOLD 与接入前基线，也不能将阶段 4 标记为完全通过。

## 15. 三机地面仿真和无控制结果

使用 `start_sim=true enable_control=false gui=false` 实测：

- Gazebo 同时存在 `iris_mid360_00/11/22`；
- 三个 PX4 SITL 与三个 MAVROS 均连接，system ID 为 1/2/3；
- 三路 Mid360、D435 RGB/depth/camera_info、FAST-LIO odom/点云均有数据；
- world→local 初始偏移与 0/4/8 m spawn 一致；
- `/swarm/px4_params_ready=true`；
- 三架 bridge 均不创建 MAVROS setpoint publisher；
- position、raw local、velocity、attitude、thrust 五类控制 Topic 均
  `NO_PUBLISHER`；
- 三机保持 `armed=false`；
- 协调器保持 `SAFETY_INHIBIT`。

无仿真、wall-time 启动用于检查 64 个工程节点和公共 Topic 图；实际 Gazebo
地面启动另外验证了模型、端口、传感器和 `/clock`。

## 16. 阶段 7 和阶段 8

### 3 秒间隔

未解锁、未起飞、未执行绕塔。没有实际起飞时间、2/3/4 m 飞行高度、最小
两两距离、规划频率、重规划次数、跟踪误差、HOLD 或返航数据。原因是任务高度
在起飞许可前已被 6 m 固定层门拒绝。

### 同时起飞

未执行。阶段 7 没有通过，不满足“稳定后再验证 0.0”的前置条件；相同
2/3/4 m 高度冲突也没有消失。参数接口保留，但当前不建议作为工程默认值。

这些指标一律记为“未测/不适用”，没有用地面 spawn 距离冒充飞行最小间距。

## 17. cancel、重启和旧轨迹

| 项目 | 当前证据 |
|---|---|
| 接入前单机 cancel/restart | 运行通过：cancel 后旧 PositionCommand 停止，新 fresh goal 恢复 |
| 迁移后单机 cancel/restart | dry-run 通过：FSM `EXEC_TRAJ -> WAIT_TARGET -> EXEC_TRAJ` |
| 一机 cancel 不影响其他两机 | Topic/namespace 静态隔离成立；三机真实轨迹运行未验证 |
| 轨迹过期 | 接收门按 start time、duration 和 3 s timeout 拒绝 |
| 节点重启不使用旧轨迹 | 进程内 buffer 重建和时间门存在；三机运行重启注入未完成 |

因此旧轨迹的源码门和单机 cancel 有证据，三机节点重启实验仍待补。

## 18. 构建与测试

最终 Release 白名单：

```bash
source /opt/ros/noetic/setup.bash
source simulation/sim_workspace/devel/setup.bash
cd AstraDrone_ros1_ws
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_WHITELIST_PACKAGES='astra_custom_msgs;traj_utils;bspline_opt;\
path_searching;plan_env;ego_planner;waypoint_generator;ego_gazebo_bridge;\
astra_tower_mission;astra_swarm_msgs;astra_swarm_manager;astra_swarm_safety;\
astra_swarm_tf;astra_swarm_perception;astra_swarm_bringup' -j2
```

回归：

```bash
catkin_make \
  -DCATKIN_WHITELIST_PACKAGES='astra_custom_msgs;traj_utils;bspline_opt;\
path_searching;plan_env;ego_planner;waypoint_generator;ego_gazebo_bridge;\
astra_tower_mission;astra_swarm_msgs;astra_swarm_manager;astra_swarm_safety;\
astra_swarm_tf;astra_swarm_perception;astra_swarm_bringup' \
  run_tests_ego_planner run_tests_astra_swarm_manager \
  run_tests_astra_swarm_safety run_tests_ego_gazebo_bridge \
  run_tests_astra_tower_mission -j2
catkin_test_results build/test_results
```

结果：

```text
260 tests, 0 errors, 0 failures, 0 skipped
```

其中 manager/safety 最终单独回归为 11 + 5 个 Python 测试；EGO 新增公共坐标
往返 gtest 通过。VTK 安装导出缺文件仍只产生历史环境警告，没有导致目标失败。

Launch 展开检查：

```bash
roslaunch astra_swarm_bringup triple_tower_inspection.launch \
  start_sim:=false enable_control:=false use_sim_time:=false --nodes
```

成功展开 `drone_0/1/2`、三套 FAST-LIO/EGO/traj_server/bridge/task 和统一
manager/safety。

## 19. 未解决问题

1. 2/3/4 m 固定层与当前 6 m 有效垂直安全模型不兼容；这是阶段 7/8 的硬阻塞。
2. 迁移后的单机 26→22 m 与固定 2 m 实际飞行没有重新验收，阶段 4 不完整。
3. 三条真实 EGO B-spline 的冲突注入、实际 optimizer 重规划和 UAV2 参与证据
   尚未完成；当前只有图连通、源码和外部安全测试。
4. 三机 cancel 隔离、节点重启和旧轨迹失效没有做完整运行注入。
5. 三机实际飞行高度、最小水平/垂直/三维间距、绕塔、返航和降落均未测。
6. `CoordinationStatus` 仍保留旧双机布尔字段；UAV3 许可有独立 Topic，但消息
   schema 后续应升级为数组式 fleet status。
7. FAST-LIO world/local 关系目前是 Gazebo spawn/yaw 假设；实机必须标定并
   验证时间同步、外参和公共坐标原点。
8. D435 三路接口已独立，但当前规划仍使用 Mid360/FAST-LIO 点云，未接入视觉
   检测主链。
9. 官方参考完整工作区的可选 `multi_map_server` 消息生成问题未修；正式工程
   不依赖该可选包。
10. 外部 PX4 保持原 detached dirty 基线，未修改、清理或升级。

## 20. 推荐配置和后续条件

当前推荐默认：

```text
takeoff_interval_sec=3.0
```

原因是它在安全配置成立后给启动、定位、通信和 takeoff corridor 留出诊断时间。
`0.0` 只保留为实验参数，不建议作为当前工程默认值。

继续实际多机前至少需要：

1. 项目负责人批准把高度改为至少 2/8/14 m，或批准一套经几何和时序证明安全的
   2/3/4 m 分离任务；
2. 先补齐迁移后单机 26→22 m 和固定 2 m 实飞回归；
3. 完成三机无控制真实轨迹冲突、重规划、cancel、重启和过期注入；
4. 再按 3 秒低速短距离、3 秒完整绕塔、0 秒低速悬停、0 秒完整绕塔顺序验收；
5. 用 bag/CSV 计算真实最小距离和跟踪误差。

当前代码具备继续做同一 ROS Master 下三机 SITL 验证的工程基础，但不具备宣称
“2/3/4 m 三机绕塔已安全通过”的证据。它也尚不具备直接迁移真实多机或
`rosmsg_tcp_bridge` 的条件；需要先完成公共坐标实机标定、时钟同步、链路延迟、
丢包/重连和数组式 fleet status 设计。
