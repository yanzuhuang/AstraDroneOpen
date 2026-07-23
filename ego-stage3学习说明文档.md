# 第三阶段启动方式

本页是当前工作区阶段三的可复制入口和验收记录。阶段三只针对 ROS1
Noetic、Gazebo Classic、PX4 SITL、MAVROS、FAST-LIO 和 EGO-Planner 单机仿真；
不包含动态障碍预测、多机、相机检测、Cloud/QGIS、真机或 PX4/EGO 核心升级。

## 环境准备

先确认没有旧的 roslaunch、阶段一/二 tmux、Gazebo、PX4、FAST-LIO、EGO、
traj_server、bridge 或 stage3 进程。外部 PX4 `/home/yanzu/PX4-Autopilot`
只读使用。主工作区和下层仿真工作区按以下顺序准备：

```bash
source /opt/ros/noetic/setup.bash
cd /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws
source ../simulation/sim_workspace/devel/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES='astra_custom_msgs;offboard;astra_tower_mission;ego_gazebo_bridge' -j2
```

若要同时验证 EGO 外层 FSM、真实 `PlannerStatus` 和 `traj_server`：

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES='astra_custom_msgs;plan_env;path_searching;traj_utils;bspline_opt;ego_planner' -j2
```

## 标准 dry-run 启动（默认，不解锁）

```bash
cd /home/yanzu/AstraDroneOpen
scripts/run_sh/stage3_ego.sh --sector-limit 1 \
  --bag /tmp/stage3_dryrun_evidence.bag --gui --rviz --attach
```

脚本会按 PX4/Gazebo → FAST-LIO → EGO/traj_server → bridge → 阶段三任务层
启动完整链；`enable_control=false` 时 bridge 不注册或发布任何 MAVROS setpoint，
不会解锁、起飞或进入 OFFBOARD。查看日志：

```bash
tmux attach -t stage3_ego
```

标准停止方式：

```bash
cd /home/yanzu/AstraDroneOpen
scripts/run_sh/stage3_ego.sh --stop
```

## 录制 rosbag

推荐由启动脚本在 bridge/mission 启动前自动开始录包：

```bash
scripts/run_sh/stage3_ego.sh --sector-limit 1 \
  --bag /tmp/stage3_dryrun_evidence.bag
```

脚本会等待 `/stage3_evidence_recorder` 存在后才启动 bridge/mission，因而控制
模式下也能保证 bag 从 ARM/OFFBOARD 前开始。目标 `.bag` 或 `.active` 已存在
时拒绝覆盖。也可以使用只录制、不启动控制权的独立脚本：

```bash
bash scripts/run_sh/stage3_record_bag.sh /tmp/stage3_evidence.bag
```

至少应包含 `/livox/lidar`、`/cloud_registered`、
`/stage3/cloud_registered_filtered`、`/Odometry`、MAVROS 状态/位姿、
`/grid_map/occupancy(_inflate)`、`/stage3/occupancy_inflate`、
`/planning/goal`、`/planning/bspline`、`/planning/pos_cmd`、
`/planner/status`、`/ego_mavros_bridge/state`、`/ego_mavros_bridge/tracking_error`、
`/tower_mission/*`、`/planning/cancel`、`/tf`、`/tf_static` 和 `/rosout`。

## 带控制启动前检查与安全边界

以下命令只允许在项目负责人明确批准后执行，会解锁、起飞并进入 OFFBOARD。
已经批准并完成的最终单 ENTRY_GATE/单扇区验证见第 1.4 节；没有批准时只运行
上面的 dry-run：

```bash
cd /home/yanzu/AstraDroneOpen
scripts/run_sh/stage3_ego.sh --control --sector-limit 1 \
  --bag /tmp/stage3_single_sector_repeat.bag --gui --rviz --attach
```

批准前必须完成：

1. dry-run 的点云、过滤点云、`/Odometry`、TF、占据图和 planning frame 对齐；
2. `/planner/status` 为 EGO FSM 直接发布，状态时间戳/失败计数有更新；
3. `rosnode info /ego_mavros_bridge` 显示只有 bridge 发布 MAVROS 控制 topic；
4. preflight、home、任务边界和 `worksite.world` 塔/吊机几何已审阅；
5. 首次批准后必须添加 `--sector-limit 1`，按“单 ENTRY_GATE → 单扇区 →
   故障恢复 → 返航”顺序验收，禁止直接执行完整 8 扇区控制飞行。

# 1. 2026-07-23 rosbag 时间线与根因

## 1.1 首次飞行：00-33-53

目标文件：`/home/yanzu/bag/astra_flight_2026-07-23-00-33-53.bag`。
`rosbag info` 证据：仿真时间 `10624.219–10830.094`，205.875 s，5.8 MB，
共 14,057 条消息。实际录入的 topic 只有：

```text
/mavros/local_position/odom
/mavros/local_position/pose
/mavros/state
/mavros/extended_state
/rosout
```

因此该 bag 没有 `/Odometry`、`/planning/goal`、`/planning/bspline`、
`/planning/pos_cmd`、`/planner/status`、bridge/mission 状态、占据图、点云或
TF。下面的结论严格区分“bag 直接证据”和“不能由该 bag 证明的推断”。

| 仿真时间 | bag 直接证据 |
|---:|---|
| 10632.028 | bridge `TAKEOFF -> HOVER_READY` |
| 10632.043 | mission `WAIT_INPUTS -> APPROACH` |
| 10632.093 | bridge 转发目标 `(-2.78, 2.16, 4.00)`；随后进入 `TRACK_EGO` |
| 10644.393 | bridge 启用 `FACE_SELECTED_TOWER` yaw |
| 10645.243 | bridge 转发固定 XY 的爬升目标 `(-2.78, 2.16, 30.00)` |
| 10645.262–10767.158 | MAVROS 位姿由约 `(-2.57,1.95,4.06)` 到 `(-2.76,1.72,29.65)`；最大水平偏移约 1.62 m，高度单调爬升 |
| 10758.970 | EGO 首次连续报告 `terminal point ... is in obstacle` |
| 10766.119、10766.869 | EGO 报 `Suddenly discovered obstacles. emergency stop!` |
| 10767.143 | mission 记录 `APPROACH -> EVALUATING`；说明任务层当时仍把爬升视为完成 |
| 10767.196 | mission 锁定 `s0_c27`；bridge 转发扇区目标 `(-7.02, 6.04, 30.00)` |
| 10772.195 | bridge `TRACK_EGO -> HOLD`，mission `NAVIGATING -> HOLDING: REPLAN_FAILED`；同时发布取消，EGO/traj_server 明确停止旧轨迹 |
| 10774.244–10774.293 | 选 clockwise 恢复，发布 R1 `(0.36,-1.91,35.00)`，bridge `HOLD -> HOVER_READY` |
| 10775.329 | bridge 因 `planner/MAVROS alignment failure` 再次进入 HOLD |
| 10783.348 | bridge `HOLD -> LANDING: hold timeout` |
| 10783.369 | bridge 请求 `AUTO.LAND`；MAVROS 状态转 `AUTO.LAND` |
| 10815.929 | PX4 报告已解锁/接地，bridge `LANDING -> DONE` |

### 三个问题的证据结论

* **弧形爬升归因**：不能由该 bag 确认是 EGO B-spline、`PositionCommand`、
  bridge 转换/限速还是 PX4 跟踪偏移。bag 只证明任务/bridge 发的是固定 XY
  目标，而实际 MAVROS 位姿存在约 1.62 m 水平偏移；缺少 `/planning/bspline`、
  `/planning/pos_cmd` 和 bridge tracking error，不能把偏移冒充为 EGO 规划结果。
* **吊机/障碍关系**：bag 没有点云、占据图、TF，不能识别真实占据体素。已审计
  的 worksite 几何中，扇区目标距 `radio_tower` 约 14.00 m、距
  `tower_crane` 约 20.62 m、距低处行人约 11.70 m；所以“目标几何上直接撞吊机”
  也没有证据。根因只能确认到 EGO 终点被其局部地图判占据/重规划失败，具体是
  吊机、陈旧体素、frame 错位或其他回波仍待完整 bag 验证。
* **快速降落触发**：不是 mission 直接调用降落，也不是 PlannerStatus 在 bag
  中可见。可见链是恢复目标发布后 bridge 对齐失败 → 监督 HOLD 被旧代码释放 →
  HOLD 超时 → `AUTO.LAND`。现已保持恢复型监督 HOLD 锁存，只有任务层显式
  `return_home`/`land` 才解除。

## 1.2 修复后复飞前证据：13-15-39

目标文件：`/home/yanzu/bag/astra_flight_2026-07-23-13-15-39.bag`。
仿真时间 `10621.017–10665.594`，44.6 s，1.2 MB，共 2,983 条消息。
该 bag 仍只录入 MAVROS 状态/位姿和 `/rosout`，没有 `/Odometry`、
`/planning/pos_cmd`、`/planner/status`、bridge/mission 状态、TF 或占据图。

| 仿真时间 | bag 直接证据 |
|---:|---|
| 10632.077 | bridge `TAKEOFF -> HOVER_READY`，MAVROS 高度约 3.98 m |
| 10632.119 | mission 选择 `ENTRY_GATE_a8=(-2.40,1.23,30.00)`，生成 5 个三维滚动目标 |
| 10632.167 | bridge 转发首个滚动目标并切换 `FACE_SELECTED_TOWER` |
| 10632.201 | bridge `HOVER_READY -> TRACK_EGO`，说明目标与命令当时均已通过 preflight |
| 10632.697 | bridge `TRACK_EGO -> HOLD`；位置对齐误差 0.126 m，yaw 误差 0.285 rad |
| 10640.697 | 恰好 HOLD 8 s 后进入 LANDING |
| 10640.719–10640.926 | bridge 请求成功、PX4 状态进入 `AUTO.LAND` |
| 10660.939 | 已解锁并接地，bridge `LANDING -> DONE` |

当前位置阈值 0.25 m，yaw 阈值 0.261799 rad（15°），所以触发项是 yaw，
不是位置。MAVROS yaw 在 `10632.193–10632.694` 由约 0.001 rad 快速变化到
0.956 rad，而高度仍只有 4.02 m；这与同一时刻首次启用塔向 yaw 一致。

根因由两部分组成：

1. bridge 首次执行塔向 yaw 覆盖时，没有从 MAVROS 当前实测 yaw 初始化限速器，
   第一帧 setpoint 直接跳到塔向目标；
2. `TRACK_EGO` 将单周期 yaw 对齐超限立即升级为 HOLD；该 bridge-local HOLD
   尚未被 mission 调用 cancel 接管，因此 `supervised_hold=false`，8 s 后按
   `land_after_loss` 请求 `AUTO.LAND`。

该 bag 没有规划 topic，不能据此评价 EGO B-spline 或障碍物；但
`HOVER_READY -> TRACK_EGO -> alignment HOLD -> 8 s timeout -> AUTO.LAND`
触发链有 bridge 和 MAVROS 直接证据。它不是任务层直接降落、command timeout、
PlannerStatus 失败或 PX4 自主 failsafe。

## 1.3 批准后的单 ENTRY_GATE/单扇区带控制验证

原始文件：`/tmp/stage3_single_entry_sector_20260723.bag`；持久化归档：
`/home/yanzu/bag/astra_stage3_single_entry_sector_2026-07-23.bag`，
SHA-256 为
`2aaefe7ce1333b99bf05366e1fd4be9445c5c2aefd3c3bf6b669ab1d9508e04c`。
该 bag 记录了完整
规划、任务、bridge、MAVROS、点云、占据图、TF 和 `/rosout`，仿真时间
`10626.22–11030.83`，约 404 s，793.2 MB（压缩），144,541 条消息。执行命令
为 `stage3_ego.sh --control --sector-limit 1`；没有执行 8 扇区控制飞行。

| 仿真时间 | 直接证据 |
|---:|---|
| 10623.948–10634.069 | bridge `ARM_OFFBOARD -> TAKEOFF -> HOVER_READY` |
| 10634.094–10752.841 | 选择 `ENTRY_GATE_a8=(-2.40,1.23,30)`；5 个三维滚动目标依次到达，EGO trajectory id 1–15 均成功 |
| 10634.188 | `HOVER_READY -> TRACK_EGO`，没有复现上一 bag 的即时 yaw 对齐 HOLD |
| 10752.841–10752.895 | ENTRY_GATE 到达，进入最近扇区目标 `s0_c27=(-7.02,6.04,30)` |
| 10761.076–10761.224 | trajectory id 18 的真实 `REPLAN_TRAJ` 失败，`consecutive_plan_failures=3`、`REPLAN_FAILED`；同时 `goal_in_collision=false`、`current_position_in_collision=false` |
| 10761.192–10763.308 | bridge 进入监督 HOLD，mission 进入 `HOLDING` 并取消旧轨迹；没有触发 bridge 的 8 s 自动降落 |
| 10763.243–10871.341 | mission 依次执行 R1、R2 和 re-entry；新 trajectory id 19–33 成功，重新锁定扇区目标 |
| 10872.442–10992.628 | 发布 home `(0.02,-0.01,3.99)`，到达后正常进入返航/降落 |
| 10992.648–11011.949 | bridge 请求 `AUTO.LAND`，PX4 `landed_state=ON_GROUND`，最终 `armed=false`、bridge `DONE`、mission `DONE` |

这次飞行直接验证了两件事。第一，首帧 yaw 实测初始化、最短角限速和
1.0 s yaw-only 对齐窗口生效：ENTRY_GATE 五段进场没有出现
`TRACK_EGO -> HOLD -> 8 s -> AUTO.LAND`，`/planning/pos_cmd` 的 yaw 是连续变化
的。第二，单扇区失败是 EGO 的真实走廊重规划失败，而不是目标点被简单标成
占据：失败时 `/planner/status` 的 `REPLAN_FAILED` 与失败计数均来自 EGO FSM，
且两个 occupancy 标志都为 false。

失败时刻 `10760.924` 的 `/stage3/occupancy_inflate` 进一步显示：当前点到
`(-7.02,6.04,30)` 的连线有约 2,361 个占据点落在 3 m 走廊内，当前点附近
最近膨胀点约 0.323 m；目标点本身最近膨胀点约 3.44 m，说明阻塞主要在进场
走廊而非终点。bag 中的 `map -> camera_init` 静态 TF 为单位变换，点云和占据
图 frame 一致。

控制飞行后的模型审计进一步确认了障碍语义：`worksite.world` 的 `<state>`
不是第二个模型实例，而是同名模型的保存位姿覆盖。实际 `tower_crane` 位姿为
`(6.36257,21.7297,0,yaw=-0.479608)`；其 DAE 使用 inch 单位，换算后的局部
包围为 `x=[-3.2731,3.2731]`、`y=[-27.3637,11.7209]`、
`z=[0,35.0282]`。失败走廊代表点 `(-4.70,3.50,30)` 距吊机网格最近顶点约
0.641 m，距铁塔网格最近顶点约 9.898 m；当前位置距吊机网格最近顶点约
0.713 m。因此可把本次走廊占据确定为吊机长吊臂区域，而不是塔中心、终点占据
或重复模型实例。EGO 的连续重规划失败与该几何证据一致。

本节属于“已在 Gazebo/PX4 SITL 带控制验证”：单 ENTRY_GATE、单扇区、一次
规划失败恢复、返航和正常 AUTO.LAND 均完成。它不代表完整 8 扇区、所有
ENTRY_GATE 候选、动态障碍或真机已经验收。

## 1.4 吊机 OBB 修复后的失败复现与最终成功

本轮先复现失败、再按完整 bag 修复，最后重新执行同一个
`--control --sector-limit 1`，没有扩大到两/四/八扇区。

### 修复前完整失败 bag

持久化文件：
`/home/yanzu/bag/astra_stage3_single_sector_hold_bug_2026-07-23.bag`，
CSV：
`/home/yanzu/bag/astra_stage3_single_sector_hold_bug_2026-07-23.csv`，
SHA-256：
`17998baac1922d463b2413af1a61df9c1bf0392df0ef16ac8394e05641982cd9`。
bag 覆盖仿真时间 `10614.802–10989.049`，374.247 s，528,579 条消息。

| 仿真时间 | 直接证据 |
|---:|---|
| 10635.105–10757.254 | `ENTRY_GATE_a23=(5.81,7.54,30)` 的 5 个三维子目标全部成功，到达 gate |
| 10757.304 | 最近几何扇区 `sector_id=1` 的 45 个候选全部被吊机 OBB 以 `KNOWN_OBSTACLE_CLEARANCE` 拒绝；下一扇区 `sector_id=2` 已有 3 个安全候选 |
| 10757.306 | 旧逻辑没有选择下一安全扇区，而是进入 HOLD/cancel |
| 10757.298→10757.318 | `/mavros/setpoint_raw/local` 从 `(5.694,7.381,29.473)` 突变为陈旧的 `(0.024,-0.004,3.997)` |
| 10757.598–10759.462 | MAVROS 实测速度最高约 5 m/s，飞机向旧起飞 setpoint 快速回撤/下降 |
| 10759.354 | 恢复 re-entry 错误继承为 `recovery_reentry_ENTRY_GATE_a23` |
| 10759.739 | 高速运动造成 planner/MAVROS 位置对齐误差 0.290 m，再次 HOLD |
| 10821.906–10823.955 | 恢复超时后任务安全请求返航；监督 HOLD 没有自动降落 |
| 10948.778–10967.953 | 到达 home 后正常请求 `AUTO.LAND`，最终解锁、接地、bridge/mission `DONE` |

根因不是 EGO 无故规划回 home。bridge 的 raw EGO 跟踪直接发布
`PositionTarget`，但内部 `output_setpoint_` 仍停在约 4 m 的起飞点；
HOLD 切回位置 setpoint 时从该陈旧缓存开始输出，制造了危险跳变。另一个任务层
问题是只按几何最近扇区开始，明知下一扇区有安全候选仍先进入恢复；无已锁定
扇区目标时，恢复 re-entry 又错误引用了当前 ENTRY_GATE/上一恢复目标。

对应修复：

1. gate 到达后对扇区按当前位置距离比较，只从通过新鲜地图、占据、塔/吊机
   净空和边界硬检查的扇区中选择最近者；选中后旋转扇区序列，仍保留全部 8 个
   唯一扇区的后续顺序；
2. 每次进入 HOLD/LANDING 都从 MAVROS 实测位姿同时锁存 `hold_pose_` 和
   `output_setpoint_`，不再从陈旧 takeoff/home 缓存起步；
3. recovery re-entry 只允许使用当前扇区已锁定目标，或同扇区的活动巡检目标；
   ENTRY_GATE、R1/R2 不再被串入 re-entry。

### 修复后 dry-run

持久化文件：
`/home/yanzu/bag/astra_stage3_dryrun_safe_sector_fix_2026-07-23.bag`，
SHA-256：
`be6a69e34d9580940fadfca06d36b77054407f4842b43ab1e16694223c3442d9`。
27 个 gate 全部生成；`a8` 以 `clearance=-0.033 m` 被 OBB 拒绝，
`a23` 以 `clearance=2.388 m` 被选中；恢复配置为 38 m。两个 MAVROS
position/raw-local topic 均无 publisher，PX4 始终 `armed=false`、
`landed_state=ON_GROUND`。

### 最终成功控制 bag

持久化文件：
`/home/yanzu/bag/astra_stage3_single_sector_success_2026-07-23.bag`，
CSV：
`/home/yanzu/bag/astra_stage3_single_sector_success_2026-07-23.csv`，
SHA-256：
`9b5ed50ccfa09b7c599e683c3f737dac7a77cfab0b2e6914ace59cd35c001408`。
bag 覆盖仿真时间 `10614.802–11049.347`，434.545 s，约 1.2 GB，
616,715 条消息。

| 仿真时间 | 最终直接证据 |
|---:|---|
| 10620.922–10635.138 | `WAIT_INPUTS → PRESTREAM → ARM_OFFBOARD → TAKEOFF → HOVER_READY` |
| 10635.157–10759.667 | 选择 `ENTRY_GATE_a23`；5 个三维滚动目标依次到 `(5.812,7.535,30)`，trajectory 0–14 成功 |
| 10759.667 | 最近几何扇区 1 的 45/45 候选被 OBB 拒绝；选择最近安全扇区 2 |
| 10759.667–10856.757 | 锁定 `s2_c36=(1.752,27.233,30)`；EGO 持续滚动重规划并完成单扇区 |
| 10856.757 | 扇区记为 covered，任务正常进入 `RETURN_HOME` |
| 11013.438–11013.459 | 到达 home hover，bridge 主动请求正常 `AUTO.LAND` |
| 11030.334–11032.956 | PX4 `ON_GROUND`、`armed=false`，bridge/mission 均为 `DONE` |

最终 bag 没有 `/planning/cancel` 消息，也没有 bridge/mission `HOLD`、
`RECOVERING` 或异常降落状态。`PlannerStatus` 最大连续失败数为 0，
`goal_in_collision/current_position_in_collision/emergency_stop` 全程为 false。
raw setpoint 全部 `type_mask=0`，唯一发布连接为 `/ego_mavros_bridge`；
最大跟踪误差 0.307 m、均值 0.048 m、P95 0.081 m。扇区完成时位置误差约
0.334 m、MAVROS 实测速度 0.157 m/s、yaw 误差约 0.018 rad，满足当前到达
门限。`/Odometry`、点云、占据图、goal 和 `PositionCommand` 均为
`camera_init`，raw/MAVROS 为 `map`；静态 `map -> camera_init` 为单位变换。

本节属于“已在 Gazebo/PX4 SITL 带控制验证”：新 OBB、27 个 gate 候选、
`ENTRY_GATE_a23`、最近安全扇区、单扇区、返航和接地均有最终 bag 证据。
38 m 参数已在控制启动配置中加载，但最终成功路径未触发 R1/R2，因此“实际飞到
38 m 恢复高度”仍是未飞行验证，不能借本次成功宣称。

# 2. 修复设计与状态机

## ENTRY_GATE

取消旧的“低空接近点 → 固定 XY 原地爬升”。任务层现在在巡检高度生成特殊
`ENTRY_GATE` 候选（`sector_id=-1`，不计入 8 个扇区）：

* 角度候选扩展为
  `start_angle + [-40,-30,-20,-10,0,10,20,30,40]°`；
* 半径候选 `mission/radius + [2,4,6] m`；
* 每个候选强制 `inspection_height`；
* 依次检查地图新鲜度、塔 keep-out、静态吊机/障碍、占据/净空、
  半径/高度/任务边界；
* 通过硬过滤后按安全净空和当前距离评分，距离是主项，blocked straight
  corridor 只作为风险惩罚；
* 从悬停当前位置到 gate 以最大 6 m 三维段滚动生成子目标，水平位移和爬升
  同时发生；每段都回到 EGO 重新感知、规划和执行；
* gate 终点要求 yaw 到达并面向塔。到达后按当前位置距离顺序，选择第一个
  通过全部硬检查的最近安全扇区；被吊机 OBB 完全覆盖的几何最近扇区不会先触发
  无意义恢复。选中后旋转巡检序列而不是删除扇区，原 8 扇区仍全部保留；
* 扇区内仍是候选过滤、锁定、迟滞和自适应目标。

控制 bag 之后又增加了吊机定向三维包围盒预过滤。包围盒使用 Gazebo `<state>`
的 yaw 和 DAE 实际尺寸，不再把长约 39 m 的吊机简化成中心半径 3 m 圆柱。
旧飞行选择的 `ENTRY_GATE_a8` 会被新几何拒绝，扩展后的角度候选仍保留安全
gate。该优化已通过单元、无控制集成、实际栈 dry-run 和单扇区控制飞行。

gate 目标被占据、地图过期或滚动子目标持续失败时，任务取消旧轨迹、进入
监督 HOLD，重新评估未用 gate；候选耗尽才走返航/降落。R1/R2/re-entry 仍由
任务层生成，EGO 负责每段局部规划。

## 三层状态机契约

```text
任务：WAIT_INPUTS → APPROACH(ENTRY_GATE rolling)
     → EVALUATING → TARGET_LOCKED → NAVIGATING
     → HOLDING → RELOCATING/RECOVERING(R1,R2,P')
     → RETURN_HOME → DONE
     ↘ FAILURE_LANDING/ERROR

EGO：INIT → WAIT_TARGET → GEN_NEW_TRAJ/REPLAN_TRAJ
     → EXEC_TRAJ ↔ EMERGENCY_STOP

bridge：WAIT_FCU → WAIT_INPUTS → PRESTREAM → ARM_OFFBOARD
        → TAKEOFF → HOVER_READY → TRACK_EGO
        → HOLD → LANDING → DONE/ERROR
```

取消链现在同时完成三件事：bridge 清掉有效目标、向 EGO FSM 发布
`/planning/cancel`、通知 traj_server 丢弃旧 B-spline/PositionCommand。bridge 的
generation gate 只接受取消之后、且对应新目标/新轨迹代次的命令。恢复型 HOLD
不会因 `resume_ego` 或短暂对齐失败自动降落；返航或降落服务才显式解除监督锁存。
进入 HOLD/LANDING 时还会把 MAVROS 实测位姿同时写入 HOLD 位姿和位置输出
缓存，避免 raw EGO 跟踪后切换 HOLD 时回跳到陈旧起飞 setpoint。

针对 `13-15-39` bag 新增三层保护：

* yaw 策略变化、取消后恢复或新一轮 `HOVER_READY` 时，首个 yaw 命令从
  MAVROS 当前实测航向开始，并按 `max_yaw_rate` 走最短角增量，不再首帧跳变；
* 位置/TF 对齐超限仍立即 HOLD；只有位置正常而 yaw 超限时才允许 1.0 s
  暂态窗口，持续超限才 HOLD；
* bridge 在活跃任务中自行进入 HOLD 时，mission 立即调用 cancel/disable
  tracking 接管为监督 HOLD。ENTRY_GATE 阶段会换 gate，巡检阶段进入既有
  R1/R2/re-entry 链，不再静默等待 bridge 的 8 s 自动降落。

`/planner/status` 改为 EGO 外层 FSM 直接发布真实 `planner_state`、
`trajectory_id`、实际规划调用的连续失败计数、goal/current occupancy 和
emergency-stop 时长；周期发布本身不会增加失败计数。任务层不再把 20 Hz
适配器循环计数当成 EGO 规划尝试。

# 3. 关键参数与 topic

阶段三配置：`AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/config/stage3_ego.yaml`

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `inspection_height` | 30 m | ENTRY_GATE 和巡检扇区高度 |
| `recovery_height` | 38 m（上限 40 m） | 当前 R1/R2 高度；已加载并通过配置/单测/dry-run，最终成功飞行未进入恢复，尚未实际飞到 38 m |
| `virtual_ceil_height` | 45 m | EGO/任务虚拟上限 |
| `radius` / `sector_count` | 14 m / 8 | 原 8 个扇区保持不变 |
| `sector_limit` | 8 | 配置默认策略；本轮 dry-run 和唯一控制飞行均显式覆盖为 1 |
| ENTRY_GATE 角度/半径 | -40° 至 +40° 九组 / +2,+4,+6 m | 27 个候选，覆盖吊臂两侧安全进场区域 |
| `entry_gate/maximum_segment_length` | 6 m | 三维滚动子目标 |
| `minimum_clearance` | 2 m | 塔、吊机、点云膨胀共同门槛 |
| 吊机 OBB | center `(2.7535,14.7908)`、half extent `(3.2732,19.5424)`、yaw `-0.479608`、z `0–35.0283` | 由 `<state>` 位姿和 DAE inch→m 包围计算 |
| `max_recovery_attempts` | 2 | 每扇区有界恢复 |
| `alignment_yaw_tolerance` | 0.261799 rad | planner/MAVROS yaw 对齐阈值 |
| `alignment_yaw_error_duration` | 1.0 s | 仅 yaw 超限的持续门禁；位置超限不延迟 |

主要 topic：

```text
/Odometry                         FAST-LIO 规划里程计
/stage3/cloud_registered_filtered 过滤后障碍点云
/stage3/occupancy_inflate          外部盖章后的占据图
/planning/goal                     任务 → waypoint_generator/EGO
/planning/bspline                  EGO FSM → traj_server
/planning/pos_cmd                  traj_server → bridge
/planner/status                    EGO FSM 真实状态
/planning/cancel                   取消旧轨迹隔离
/ego_mavros_bridge/state
/ego_mavros_bridge/tracking_error
/tower_mission/state/current_target/current_sector/candidate_targets
/tf /tf_static
```

bridge 仍是唯一 MAVROS setpoint 出口；dry-run 不创建该 publisher。

# 4. 修改文件

* `astra_tower_mission/include/.../stage3_planner.h`、
  `src/stage3_planner.cpp`：ENTRY_GATE 候选、硬过滤、近距离评分、三维滚动
  子目标、吊机 yaw 定向三维包围盒、最近已接受扇区选择、已有候选/恢复逻辑；
* `src/stage3_ego_mission_node.cpp`：ENTRY_GATE 状态、最近安全扇区、gate 重试、
  recovery re-entry 目标隔离、yaw 到达、HOLD/返航链；
* `config/stage3_ego.yaml`、`test/stage3_planner_test.cpp`、
  `test/stage3_no_control_integration.{test,py}`；
* `ego_gazebo_bridge/src/ego_mavros_bridge.cpp`、
  `include/.../ego_mavros_bridge.h`、`command_utils.{h,cpp}`、
  `include/.../recovery_control.h`：首帧 yaw 实测初始化与限速、yaw 持续性
  对齐门禁、HOLD 输出缓存锁存、恢复监督 HOLD、cancel/resume/return 链和
  旧轨迹 generation gate；
* `ego-planner/.../ego_replan_fsm.{h,cpp}`、
  `planning_status_tracker.h`、`traj_server.cpp`：真实 FSM 状态/失败计数、
  cancel 传播、旧 B-spline 隔离；未修改 EGO 优化数学、PX4 或 FAST-LIO；
* `scripts/run_sh/stage3_ego.sh`、`stage3_record_bag.sh`：修正 odom 等待，
  支持 `--bag` 在任务启动前集成录制，补齐控制出口和诊断 topic，并拒绝覆盖
  已有 bag。

# 5. 测试和验证证据

自动化与编译验证：

```text
stage3_planner_test                  15/15
ego_gazebo_bridge 回归                 24/24
stage3_no_control_integration         1/1
ego_planner goal/status tracker       5/5
offboard 回归                          7/7
ego_planner direct FSM 编译             通过
主工作区阶段三节点/bridge 编译           通过
本轮相关独立用例合计                   68/68
```

无控制集成测试已观察到：

```text
首个 ENTRY_GATE → bridge HOLD → mission 接管/cancel → 换 ENTRY_GATE
→ 单扇区 → PlannerStatus REPLAN_FAILED
→ HOLD → R1/R2/re-entry → 重新锁定 → RETURN_HOME
```

2026-07-23 16:58 运行了修复后完整实际栈无控制 dry-run，使用
`--sector-limit 1`，没有使用 `--control`：

```text
ENTRY_GATE_a8：KNOWN_OBSTACLE_CLEARANCE，clearance=-0.033 m
ENTRY_GATE_a23：accepted，clearance=2.388 m
最终选择：ENTRY_GATE_a23=(5.81,7.54,30.00)，rolling_goals=6
bridge state：DRY_RUN
/mavros/setpoint_raw/local publishers：None
/mavros/setpoint_position/local publishers：None
PX4：armed=false，landed_state=1
```

这证明新吊机 OBB 会拒绝旧飞行所选、位于吊臂包围内的 `ENTRY_GATE_a8`，同时
扩展后的候选集仍能找到满足当前静态净空的 gate。bridge 最终报告 full
preflight healthy，但没有注册或发布 MAVROS 控制 topic。dry-run 会让 EGO 从
地面 odom 规划首个滚动目标，因此最初若干低于最小相对高度的
`PositionCommand` 会被 bridge 拒绝；高度进入合法包络后 preflight 恢复健康，
该现象没有产生控制输出。

最终控制飞行已验证：

```text
ENTRY_GATE_a23 五段三维进场：通过
最近几何 sector 1：45/45 被吊机 OBB 拒绝
最近安全 sector 2 / s2_c36：通过并完成单扇区
PlannerStatus：最大连续失败 0，无 occupancy/emergency-stop
bridge：全程无 HOLD，唯一 raw-local 出口，type_mask=0
跟踪误差：max 0.307 m，P95 0.081 m
返航/降落：正常 AUTO.LAND，armed=false，ON_GROUND，双方 DONE
```

仍未飞行验证：实际 38 m R1/R2 恢复、新逻辑下 ENTRY_GATE 换点、两/四/八
扇区、动态障碍、自适应多目标长时稳定性和真机 TF 外参。受控 bag 的录制结束
时间晚于 DONE，是录包停止顺序造成的，不是飞行状态延迟。

# 6. 已知风险与下一步

* 当前 `map -> camera_init` 单位变换已由最终 bag 复核，但仍只是仿真假设，
  不是实测标定结果；
* 占据图盖章适配只修时间戳，不证明体素坐标正确；
* EGO direct `PlannerStatus` 是外层 FSM 真值，不是优化器内部每个候选体素的
  解释器；不可达目标、地图陈旧和 current occupancy 仍需现场注入验证；
* 任务层 `tower_crane` OBB 是由网格轴对齐包围转换得到的保守预过滤，会包含
  网格空隙；它用于 gate/扇区目标硬过滤，路径避障仍由 FAST-LIO/EGO 地图负责。
  最终目标 `s2_c36` 带 `STRAIGHT_CORRIDOR_BLOCKED` 软风险，但 EGO 实际成功
  重规划通过；未来仍需自动计算真实网格/占据最小净空；
* bridge 仍执行 raw-local `PositionTarget`，只允许其一个 MAVROS 控制出口；
* yaw 暂态门禁只放宽“位置对齐正常、单独 yaw 超限”的短窗口；真实 TF 或
  持续 estimator yaw 错位仍会 HOLD。下次完整 bag 必须同时核对
  `/Odometry` 与 MAVROS yaw，不能仅凭不再降落就判定 frame 正确；
* 本轮仅批准并执行 `--sector-limit 1`；不得把这次结果扩展描述成完整 8 扇区
  飞行验收。
* `/Odometry` 在当前 FAST-LIO 仿真输出中的 twist 为零，任务速度到达条件可能
  低估真实速度；本次扇区切换时独立 MAVROS 速度为 0.157 m/s，仍低于 0.2 m/s
  门限，但后续应把有效速度源契约显式化；
* 新 OBB、27 个候选、`a23` 和最近安全扇区已控制验证；38 m 恢复高度仅完成
  配置、单测和 dry-run，需在单独批准的恢复场景中验证。
* HOLD 输出缓存锁存和 re-entry 隔离已有故障 bag 根因、单元/无控制集成证据；
  最终成功飞行没有进入 HOLD/RECOVERING，因此修复后的受控故障分支尚未专门
  注入验证。

下一步可复制、但不会自动解锁的命令：

```bash
cd /home/yanzu/AstraDroneOpen
scripts/run_sh/stage3_ego.sh --sector-limit 1 \
  --bag /tmp/stage3_dryrun_evidence.bag --gui --rviz --attach

source /opt/ros/noetic/setup.bash
rosbag info /tmp/stage3_dryrun_evidence.bag
rostopic hz /Odometry
rostopic hz /planning/bspline
rostopic hz /planning/pos_cmd
rostopic hz /planner/status
rostopic echo -n 1 /tf
rostopic echo -n 1 /stage3/occupancy_inflate
rosnode info /ego_mavros_bridge
scripts/run_sh/stage3_ego.sh --stop
```

已经完成的单扇区控制命令如下。它仍会解锁、起飞和进入 OFFBOARD，后续每次
重跑都必须重新取得明确批准：

```bash
scripts/run_sh/stage3_ego.sh --control --sector-limit 1 \
  --bag /tmp/stage3_single_sector_repeat.bag --gui --rviz --attach
```

最终成功单扇区没有触发恢复。下一步先做无控制的 gate 占据/地图过期和恢复
故障注入；任何再次带控制、两扇区或更大任务都需要新的明确批准。不要把
`--sector-limit 1` 直接改成 8。
