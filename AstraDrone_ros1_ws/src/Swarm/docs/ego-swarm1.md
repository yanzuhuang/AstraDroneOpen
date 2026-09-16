# ego-swarm1：双无人机异步协同绕塔

## 1. 范围与结论

本阶段采用“两套独立 PX4/MAVROS/FAST-LIO/EGO-Planner/单机任务执行器，加 AstraDrone 集群协调与安全层”，没有替换为 EGO-Swarm，也没有修改 EGO vendor 核心、外部 PX4 或 YOLO。

默认任务为：

- UAV1：世界坐标起飞点 `(0, 0, 0)`，高度 `34 → 30 m`；
- UAV2：世界坐标起飞点 `(2, 0, 0)`，高度 `28 → 24 m`；
- 两机均逆时针，一架到达自己的 ENTRY_GATE 后立即开始；
- UAV2 起飞必须同时满足 UAV1 起飞满6秒、双方状态健康、PX4参数正确、起飞竖直走廊和预测安全检查；
- UAV2 先在现有闭圈锚点原地 `28 → 24 m`，确认稳定后才放行 UAV1 `34 → 30 m`；
- 最终圈完成后从各自当前高度的 EXIT_GATE 退出、独立返航；重叠落地区一次只发给一架机许可。

2026-07-26 已完成一次双机完整控制仿真，两机均完成两层闭合巡塔、返航、落地并解除武装。现有单机入口及默认行为保留。

## 2. 包结构

```text
Swarm/
├── astra_swarm_msgs/        # 状态、预测轨迹、切层确认、安全事件、协调状态
├── astra_swarm_manager/     # 错时起飞、切层联锁、返航和降落许可
├── astra_swarm_safety/      # 心跳、定位、当前/预测距离和高度组合检查
├── astra_swarm_tf/          # FAST-LIO硬编码frame的命名空间适配、PX4外部里程计
├── astra_swarm_perception/  # 队友包络点云过滤、可选显示用世界点云融合
├── astra_swarm_bringup/     # 双PX4/MAVROS/EGO/任务及集群统一launch
├── scripts/run_dual_tower.sh
└── docs/ego-swarm1.md
```

对现有单机代码只有两项默认关闭的最小扩展：

1. `sector_inspection_mission_node` 增加切层和落地许可等待；
2. `ego_mavros_bridge` 增加起飞许可门，以及规划话题 launch 参数。

单机 launch 不设置这些开关时，行为与原来相同。

## 3. 启动与停止

先构建：

```bash
source /opt/ros/noetic/setup.bash
cd /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES='astra_swarm_msgs;astra_swarm_manager;astra_swarm_safety;astra_swarm_tf;astra_swarm_perception;astra_swarm_bringup;astra_tower_mission;ego_gazebo_bridge' -j2
```

默认无控制检查，不解锁：

```bash
cd /home/yanzu/AstraDroneOpen
AstraDrone_ros1_ws/src/Swarm/scripts/run_dual_tower.sh --headless
```

完整双机任务：

```bash
cd /home/yanzu/AstraDroneOpen
AstraDrone_ros1_ws/src/Swarm/scripts/run_dual_tower.sh --control
```

启动完整双机任务并同时打开已配置好的 RViz：

```bash
cd /home/yanzu/AstraDroneOpen
AstraDrone_ros1_ws/src/Swarm/scripts/run_dual_tower.sh --control --rviz
```

如果双机仿真已经运行，可在新终端单独打开 RViz：

```bash
source /opt/ros/noetic/setup.bash
source /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash
rosrun rviz rviz -f world -d "$(rospack find astra_swarm_bringup)/config/dual_tower.rviz"
```

该配置以 `world` 为固定坐标系，视角默认对准铁塔，并显示两机经过队友过滤的注册点云、膨胀占据点云、里程计和 TF。UAV1 使用橙色，UAV2 使用蓝色。仅需中央世界点云用于显示或离线分析时，在启动参数中增加 `--cloud-fusion`；该融合点云不作为两机实时避障的唯一输入。

任务结束且两台 PX4 均已解除武装后，可在原 `roslaunch` 终端按 `Ctrl-C`，或从另一终端执行：

```bash
cd /home/yanzu/AstraDroneOpen
AstraDrone_ros1_ws/src/Swarm/scripts/run_dual_tower.sh --stop
```

`--stop` 只匹配 `dual_tower_inspection.launch`，向其发送 `SIGINT`，等待 roslaunch 正常关闭 PX4、Gazebo、RViz 和其余 ROS 节点，不使用强制杀进程。不要在无人机仍处于解锁飞行状态时停止仿真。

无界面控制运行可加 `--headless`；即使 Gazebo 无界面，也可以组合 `--headless --rviz` 单独保留 RViz。

单机独立 shell 入口已退役，当前任务使用 [项目整理文档·运行说明](../../../../项目整理文档.md#runbook)。`sector_inspection_mission_node`、`sector_inspection.yaml`、occupancy adapter 和 EGO bridge 仍由三机直接复用，底层 launch、节点及测试保留。

## 4. PX4、端口、命名空间与坐标

| 项目 | UAV1 | UAV2 |
|---|---:|---:|
| PX4 launch ID | 0 | 1 |
| MAVLink system ID | 1 | 2 |
| Gazebo simulator TCP | 4560 | 4561 |
| SDF MAVLink UDP | 14560 | 14561 |
| MAVROS本地UDP | 14540 | 14541 |
| PX4 onboard远端UDP | 14580 | 14581 |
| 相机MAVLink UDP | 14530 | 14531 |
| ROS命名空间 | `/uav1` | `/uav2` |
| 世界坐标home | `(0,0,0)` | `(2,0,0)` |

PX4启动前由仓库内 `config/px4-rc.params` 设置 `EKF2_HGT_REF=3`、`EKF2_EV_CTRL=11`、`EKF2_GPS_CTRL=0`；运行时守卫只验证，不在飞行前临时更改需重启参数。FAST-LIO 通过每架机独立的 MAVROS `odometry/out` 外部里程计入口进入 PX4。

仿真坐标链：

```text
world -> uav1/map -> uav1/camera_init -> uav1/body
world -> uav2/map -> uav2/camera_init -> uav2/body
```

`world → uav1/map` 为零平移，`world → uav2/map` 为 `(2,0,0)`。UAV2任务局部塔心为 `(-12.0551,19.7104)`，转换到世界后与 UAV1 的塔心 `(-10.0551,19.7104)` 重合。

关键话题全部隔离：

| 类别 | UAV1示例 | UAV2示例 |
|---|---|---|
| MAVROS | `/uav1/mavros/...` | `/uav2/mavros/...` |
| FAST-LIO里程计 | `/uav1/Odometry` | `/uav2/Odometry` |
| 注册点云 | `/uav1/cloud_registered` | `/uav2/cloud_registered` |
| EGO命令 | `/uav1/planning/pos_cmd` | `/uav2/planning/pos_cmd` |
| EGO目标 | `/uav1/planning/goal` | `/uav2/planning/goal` |
| 控制出口 | `/uav1/mavros/setpoint_raw/local` | `/uav2/mavros/setpoint_raw/local` |
| 任务状态 | `/uav1/tower_mission/state` | `/uav2/tower_mission/state` |
| 集群状态 | `/uav1/swarm/state` | `/uav2/swarm/state` |
| 预测轨迹 | `/uav1/swarm/predicted_trajectory` | `/uav2/swarm/predicted_trajectory` |
| 三类许可 | `/uav1/swarm/{takeoff,transition,landing}_permission` | `/uav2/swarm/{takeoff,transition,landing}_permission` |

全局协调接口为 `/swarm/coordinator/status`、`/swarm/layer_confirmation`、`/swarm/safety/clear`、`/swarm/safety/event` 和 `/swarm/px4_params_ready`。

## 5. 参数

主要参数位于：

- `astra_swarm_manager/config/dual_tower.yaml`
- `astra_swarm_safety/config/dual_tower.yaml`
- `astra_swarm_perception/config/dual_tower.yaml`
- `astra_swarm_bringup/config/uav{1,2}_bridge.yaml`
- `astra_swarm_bringup/launch/uav_tower_stack.launch`

当前值：

| 参数 | 值 |
|---|---|
| `takeoff_delay` | 6.0 s |
| `height_sequence` | UAV1 `[34,30]`；UAV2 `[28,24]` |
| `tower_center` | world `(-10.0551,19.7104)` |
| `orbit_radius` | 12.5 m |
| `orbit_direction` | `counter_clockwise` |
| `entry_sector` | 7（世界平面从+X逆时针编号） |
| `transition_anchor` | 完整闭圈后记录的本机首航点锚点 |
| `height_tolerance` | 0.4 m |
| `minimum_vertical_separation` | 4.0 m |
| `minimum_3d_separation` | 3.0 m |
| `trajectory_prediction_horizon` | 4.0 s |
| `trajectory_prediction_period` | 0.25 s |
| `heartbeat_timeout` | 1.0 s |
| `landing_protection_radius` | 3.0 m |
| `teammate_filter_radius` | 1.2 m |

状态共享包含 ID、时间、世界位姿、低通速度/加速度估计、yaw、桥状态、当前高度/目标、任务阶段、心跳、定位有效性和协方差。预测采用有速度和加速度限幅的短时常加速度模型；FAST-LIO当前零 `twist` 不再导致预测退化成静止点。

## 6. 状态机与安全规则

单机任务主状态：

```text
WAIT_INPUTS
 -> STAGING_POINT
 -> SEGMENTED_CLIMB
 -> ENTRY_GATE_TRANSIT
 -> TARGET_LOCKED/NAVIGATING/EVALUATING
 -> WAIT_TRANSITION_PERMISSION
 -> LAYER_TRANSITION
 -> TARGET_LOCKED/NAVIGATING/EVALUATING
 -> GO_TO_EXIT_GATE
 -> RETURN_HOME
 -> DONE
```

桥负责 `WAIT_FCU → WAIT_INPUTS → PRESTREAM → ARM_OFFBOARD → TAKEOFF → HOVER_READY → TRACK_EGO → HOME_HOVER → LANDING → DONE`。集群协调状态覆盖 `SYSTEM_READY`、`UAV1_TAKEOFF_STARTED`、`UAV2_TAKEOFF_ALLOWED`、`FIRST_LAYER_PARALLEL`、`UAV2_TRANSITION_ALLOWED`、`UAV1_TRANSITION_ALLOWED`、`SECOND_LAYER_PARALLEL`、`RETURN_COORDINATION` 和 `MISSION_COMPLETE`。

安全行为：

- 6秒只是 UAV2 起飞的必要条件；还检查双方健康、PX4参数、UAV2完整竖直起飞走廊和预测轨迹；
- 心跳、定位、FAILSAFE或安全预测异常时，不发新的起飞、切层和降落许可；
- UAV1 在34 m闭圈锚点等待；UAV2先同XY分段下降到24 m；
- 只有 UAV2 离开切层状态、稳定进入24 m容差后，才发布确认并放行 UAV1；
- 规划失败不允许另一架跨高度接管；
- 队友机体附近1.2 m点云从本机静态建图输入中删除，队友作为动态共享状态处理；
- 两机home相距2 m、落地保护区重叠，许可持有者在落地完成前保持独占。

## 7. 验证结果

完整控制仿真证据使用 Gazebo 仿真时间：

| 事件 | 结果 |
|---|---|
| UAV1进入TAKEOFF | 10630.944 s |
| UAV2进入TAKEOFF | 10659.124 s，晚28.180 s；超过6秒且起飞走廊已清空 |
| UAV1第一圈闭合并等待 | 11207.550 s，34 m锚点 |
| UAV2获得切层并开始下降 | 11216.100 s，28 m锚点 |
| UAV2稳定进入第二层 | 11232.048 s，24 m |
| UAV1收到确认后才下降 | 11232.200 s，34→30 m |
| UAV2最终EXIT_GATE | 11589.398 s，24 m |
| UAV1最终EXIT_GATE | 11608.298 s，30 m |
| UAV2落地完成 | 11713.125 s，解除武装、ON_GROUND |
| UAV1落地完成 | 11750.945 s，解除武装、ON_GROUND |

控制飞行期间两机CSV写入 `/tmp/astra_swarm_uav1.csv` 和 `/tmp/astra_swarm_uav2.csv`，当时共有5575/5396行，任务失败、重定位和恢复计数均为0。随后最终dry-run复用了这两个临时文件名，因此完整控制CSV不是仓库交付物；上表关键控制事件已从该次完整运行的 `rosout` 与CSV交叉核对后固化在本文。运行抽查中 `/swarm/safety/clear` 持续为 `true`；UAV2先落地完成，UAV1约26.7秒后才进入落地阶段。

回归结果：

- 相关8包构建通过；
- `astra_swarm_manager` 7/7；
- `astra_swarm_safety` 3/3；
- `astra_tower_mission` 63/63，包括单机无控制集成测试；
- `ego_gazebo_bridge` 26/26；
- 当前聚合结果：212 tests，0 errors，0 failures；
- 最终双机 dry-run 中四个 position/raw MAVROS控制话题均无发布者，PX4参数守卫为true，预测轨迹正常发布。

没有执行或修改任何YOLO测试。

## 8. 明确限制与后续建议

1. 当前协调策略按双机设计；高度、延迟、home、命名空间、安全阈值均已参数化，但扩展到三架及以上仍需把二机专用切层策略推广为通用资源调度。
2. `world → uavX/map` 是仿真静态外参，不是真机标定结果。
3. 两机保持独立局部地图；中央融合点云仅可视化，尚未用于实时联合避障。
4. 预测轨迹为限幅常加速度模型，不是完整共享EGO B样条；后续可增加带时间戳的B样条交换，但不应替换本地EGO。
5. bridge现有正常返航末端仍请求 PX4 `AUTO.LAND`，本阶段没有重设计既有降落架构。
6. 外部 PX4 仍是锁定的 dirty/detached v1.15.4；本实现没有改动它，依赖仓库内 `px4-rc.params` 在EKF2启动前建立视觉高度参考。
7. 已验证一次完整双机仿真和现有单机回归；真机、动态障碍预测、通信丢包长期压力和三机以上不在本阶段验收范围。
