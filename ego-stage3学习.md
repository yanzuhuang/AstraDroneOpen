# 第三阶段启动方式

本页是当前工作区阶段三的可复制入口和验收记录。阶段三只针对 ROS1
Noetic、Gazebo Classic、PX4 SITL、MAVROS、FAST-LIO 和 EGO-Planner 单机仿真；
不包含动态障碍预测、多机、相机检测、Cloud/QGIS、真机或 PX4/EGO 核心升级。

## 2026-07-24 两层分层巡塔（当前实现）

当前默认任务已由旧 30 m 单层闭环更新为固定入口的 26/22 m 两层闭环：

```text
TAKEOFF_HOVER
→ VERTICAL_ASCENT（home XY，10→18→26 m）
→ TRANSIT_TO_ENTRY_GATE / ENTRY_CHECK（sector 3，R=15 m，Z=26 m）
→ APPROACH_FIRST_WAYPOINT
→ INSPECT_LAYER_26（3→4→5→6→7→0→1→2→3）
→ LAYER_TRANSITION（sector 3 同 XY，26→24→22 m）
→ INSPECT_LAYER_22（3→4→5→6→7→0→1→2→3）
→ RETURN_EGRESS（R=24 m、Z=38 m 塔外弧到 HOME 径向门）
→ RETURN_HOME → HOME_HOVER → PX4 AUTO.LAND
```

`mission/inspection_heights=[26.0,22.0]`、`inspection_radius=12.5`、
`entry_gate/{entry_sector,radius,height}={3,15.0,26.0}`、
`mission/{inspection_start_sector,transition_sector}={3,3}`、
`staging/ascent_step_heights=[10.0,18.0,26.0]` 和
`layer_transition/step_heights=[24.0,22.0]` 均集中在
`astra_tower_mission/config/stage3_ego.yaml`。入口不再搜索其他侧或低空等待点；
静态障碍仍只允许向名义 12.5 m 半径外侧选点。垂直柱体在下降前一次性检查
占据、地图覆盖、塔 keep-out、吊机 OBB 与高度包络，失败只进入有界 HOLD。

日常观看完整两层任务（启动 Gazebo GUI、RViz，不录 rosbag）：

```bash
cd /home/yanzu/AstraDroneOpen
scripts/run_sh/stage3_ego.sh --control --sector-limit 8 --gui --rviz --attach
```

Gazebo 服务端默认启动；`--gui` 打开 Gazebo 窗口，`--rviz` 打开 RViz，且省略
`--bag` 不会启动 rosbag recorder。需要证据包时再显式添加 `--bag FILE`。

最终控制证据保存在忽略目录
`AstraDrone_ros1_ws/log/stage3_two_layer/attempt8.bag`，自动分析为
`attempt8_analysis.json` 且 `passed=true`。飞行历时 1709.341 s；两层均按
`3→4→5→6→7→0→1→2→3` 闭环；实际直升最大 XY 偏差 0.357 m，层间下降最大
XY 偏差 0.200 m；朝塔 yaw 最大误差 0.0214 rad、P95 0.0117 rad；跟踪误差
最大 0.731 m、P95 0.207 m；采样最小占据净空 0.864 m。唯一控制发布者为
`/ego_mavros_bridge`，raw `type_mask=0`；最终 task/bridge `DONE`、
`armed=false`、`ON_GROUND`，HOME 落点水平误差约 0.309 m。
最终三包回归为 `166 tests, 0 errors, 0 failures, 0 skipped`。

实飞同时修复了三项旧参数/路径不一致：34 m 固定入口在 0.30 m/s 下使用
180 s 有界超时；阶段三 MAVROS state/extended-state 以 2.0 s 判陈旧以匹配
约 1 Hz 发布周期；RETURN_EGRESS 的圆弧终点按 HOME 实际径向计算，不再沿
ENTRY_GATE 角度生成穿过吊机上方的长弦。速度、仿真倍率和动态避障范围均未改。

2026-07-23 的旧版本曾按 `2 → 4 → 8` 扇区完成实际控制飞行；该历史
8 扇区 bag 证明的是旧状态机的 8 个不重复目标、`RETURN_EGRESS` 和
AUTO.LAND，不证明本页 2026-07-24 收尾设计的 `1→…→8→1` 闭环、分段直升、
正常反向进场返航或 HOME_HOVER 落地门禁。

本次两层改造前，2026-07-24 已按 bag 根因完成进场、HOLD、返航和落地状态机
修复，并完成单层 8 扇区控制验收。该历史方案不再搜索低空水平等待点，而是在 home hover 捕获并
锁定 XY，分 9 段近竖直上升到 30 m，驻留建图后只锁定一个 ENTRY_GATE，再
水平转场。最终完整 bag 的离线分析为 `passed=true`。
最新总表见
[`阶段三任务完成与待办总结.md`](./阶段三任务完成与待办总结.md)。

## 0. 2026-07-24 单层历史设计与验证结论

### 0.1 bag 根因

最初复现 bag `/tmp/astra_stage3_repro_20260724_0220.bag` 在环境重启前完成
分析：386.829 s、535,885 条消息。任务把 30 m ENTRY_GATE 的 XY 投影到
`z=4 m` 作为低空等待点，在多个远端 gate 间切换并靠近树木；planner 出现
current collision 117 次、no feasible 79 次、最大连续失败 741，采样最小
净空仅 0.087 m。该 `/tmp` 文件随后随执行环境重启丢失，不作为可复核交付件，
但分析结论促成了后续持久化复飞。

旧 HOLD 的直接闭环缺陷是：入场前没有可用 RETURN_EGRESS，land 服务又只接受
HOME_HOVER；任务进入 ERROR 后，bridge 的 mission-supervised HOLD 会抑制自动
loss landing。因此既不能恢复、返航或降落，形成无限 HOLD。与此同时，
`no_progress_timeout` 未真正驱动状态转换，旧 EGO 目标在覆盖等待时仍活跃，
`Close to goal`/失败计数被重复累计。

持久化失败 bag
`/home/yanzu/bag/astra_stage3_vertical_entry_final_2026-07-24.bag`
进一步发现 gate 判定不一致：`ENTRY_GATE_a23` 端点已经通过硬过滤，但首个
滚动目标 `(2.919,3.764,29.963)` 又被保守起重机整块 OBB 判为占据。bag 中
该点距实际膨胀占据点最近 `4.242 m`，只是在 OBB 的 2 m 膨胀边界内
`0.038 m`。该轮在 2 s HOLD 后自动选择 NORMAL_RETURN，并完成
HOME_HOVER/AUTO.LAND/DONE，证明无限 HOLD 已消除；但没有完成巡检，所以不记
为成功。

### 0.2 最终进场方案

当前控制流程是：

```text
起飞到 home hover
→ 捕获并锁定 home-local XY
→ 3 m 步长分段垂直上升到 30 m
→ 安全高度驻留 4 s 并持续建图
→ 生成、评估并只锁定一个 ENTRY_GATE
→ 分段水平飞向塔外侧
→ 1→…→8→1 巡检闭环
→ 反向复用成功进场目标正常返航
→ HOME_HOVER → PX4 AUTO.LAND
```

主上升通道正常时绝不改变 XY。只有已知占据、路径阻挡或两次真实 EGO 不可达
等硬失败，且当前通道至少保持 5 s，才按 `[home, +3x, -3x, +3y, -3y] m`
选择附近通道；最多切换 2 次。地图未知、地图过期、目标被占据、路径阻挡和
EGO 不可达分别记录，不再共用一个失败状态。MID-360 hit-only 点云不能用
“没有命中”证明自由空间，因此上升段的 unknown 只作诊断；实际占据图终点检查
和 EGO 规划仍是硬安全门禁。

最终 gate 端点仍经过塔 keep-out、静态障碍、实际占据、任务边界和传感器可观测
体检查。滚动中间点若被实际地图占据则报告 `ENTRY_GATE_TARGET_OCCUPIED` 并
HOLD；若只与保守稀疏网格 OBB 相交，则报告
`ENTRY_GATE_STATIC_CORRIDOR_RISK`，保持同一 gate 并交给 EGO 用实时占据图
绕行。这消除了“端点按软走廊接受、首段按同一 OBB 硬拒绝”的矛盾。

### 0.3 巡检目标、HOLD 与返航

每扇区的 30 m 名义点绝对优先。只有名义点未通过硬约束或同一目标两次真实规划
失败，才按最小角度、最小半径、29～31 m、小水平改动、最后 38 m 的字典序选择
备用点；数值评分不能跨优先级补偿。目标锁定后保留迟滞和最小保持，不因一次
重规划切换候选。

任务状态为 `STAGING_POINT → SEGMENTED_CLIMB → ENTRY_GATE_TRANSIT →
TARGET_LOCKED/NAVIGATING`，异常进入 `HOLDING`，再按同一目标有限重试、
附近上升通道、NORMAL_RETURN、RETURN_EGRESS、FAILURE_LANDING 的顺序退化。
APPROACH/ENTRY_GATE、HOLD、恢复、正常返航、备用返航和落地各有独立超时、
重试上限和无进展检测。bridge 在 mission-supervised HOLD 中允许通过严格
输入/姿态/控制权门禁请求紧急 AUTO.LAND，因此任何失败链都不能无限 HOLD。

正常返航优先反向复用实际成功进场目标；有界重试耗尽才进入 24 m 塔外侧、
38 m 高度的 RETURN_EGRESS。到达 home hover 后由任务请求 PX4 AUTO.LAND，
成功必须同时满足 task/bridge `DONE`、`armed=false`、`ON_GROUND` 和 home
接近约束。

### 0.4 构建、故障注入与控制复飞

相关包构建通过；任务/bridge gtest 和无控制集成测试最新汇总为
`160 tests, 0 errors, 0 failures, 0 skipped`。无控制集成注入了 ENTRY_GATE
期间 bridge HOLD、同一目标恢复、连续两次 planner failure 后最小备用点、
NORMAL_RETURN；bridge 单测覆盖 HOME_HOVER 正常落地和 supervised HOLD 紧急
落地门禁。

持久化失败复飞：

- bag：`/home/yanzu/bag/astra_stage3_vertical_entry_final_2026-07-24.bag`
- 263.121 s、370,200 条消息；复现 OBB/实际占据不一致；
- `SEGMENTED_CLIMB → ENTRY_GATE_TRANSIT → HOLDING → NORMAL_RETURN →
  RETURN_HOME → DONE`；
- HOME_HOVER 后 AUTO.LAND，最大跟踪误差 0.345 m，最终解锁接地。

首次垂直进场成功复飞（后续 yaw/候选高度复测见第 7 节）：

- bag：
  `/home/yanzu/bag/astra_stage3_vertical_entry_final_retry1_2026-07-24.bag`
- analysis：
  `/home/yanzu/bag/astra_stage3_vertical_entry_final_retry1_2026-07-24.analysis.json`
- SHA-256：`cfc5c451749edd7a1131ee5844ec7319e7074bf2e879616a42e8230c56305dd5`
- 850.201 s、1,214,398 条消息，离线 `passed=true`；
- 9 个上升目标只有 1 个 XY，z 为
  `[6.90,9.79,12.67,15.56,18.45,21.34,24.22,27.11,30.00] m`；
  实际最大 XY 偏差 0.234 m，无大于 0.05 m 的下降跳变；
- ENTRY_GATE 仅锁定一次：`ENTRY_GATE_a23`；
- 扇区顺序 `[1,2,3,4,5,6,7,0,1]`，闭环有效，均为同扇区 `c3`
  38 m 保底点；30 m 名义点仍先评估，只因保守吊机硬几何不可用；
- sector 0 一次无进展 HOLD 后对同一目标恢复成功，`cancel_count=1`；
- planner `NONE=16864`，最大连续失败 0，无 collision/emergency；
- 最小采样膨胀占据净空 2.737 m；
- 跟踪误差 max 0.511 m、mean 0.122 m、P95 0.278 m，无持续超限；
- 41,650 条 raw setpoint 的 `type_mask` 全为 0，控制发布者唯一为 bridge；
- NORMAL_RETURN 优先，未使用 RETURN_EGRESS；HOME_HOVER 后 AUTO.LAND，
  最终 `armed=false`、`ON_GROUND`、task/bridge `DONE`。

### 0.5 尚存风险和限制

- 当前 FAST-LIO 注册点云是 hit-only，没有正式 free/unknown 体素；本实现把
  unknown 与已知占据分开，不能外推为真机自由空间证明。
- 起重机 OBB 是保守整块包围盒，包含稀疏网格空隙；最终 gate/巡检终点仍硬过滤，
  中间路径以实时占据和 EGO 为准。
- 最新逐扇区复飞中 s3～s7 的 30 m 名义点可执行；s2 使用 31 m 最小备用点，
  s0/s1 因保守起重机几何使用 38 m 备用点。
- `map→camera_init` 单位变换、FAST-LIO 零 twist 和仿真传感器模型不是真机契约。
- 只验证单机静态障碍；动态避障、多机、真机、相机、Cloud/QGIS 均未扩展。

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
scripts/run_sh/stage3_ego.sh --sector-limit 8 \
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
scripts/run_sh/stage3_ego.sh --sector-limit 8 \
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

## 不录 bag，只查看 8 扇区飞行

如果只想打开 Gazebo、RViz 并观看完整 8 扇区任务，可以不填写 `--bag`：

```bash
cd /home/yanzu/AstraDroneOpen
scripts/run_sh/stage3_ego.sh \
  --control \
  --sector-limit 8 \
  --gui \
  --rviz \
  --attach
```

该命令仍会解锁、起飞、进入 OFFBOARD 并执行完整 8 扇区；`--gui` 打开 Gazebo，
`--rviz` 打开 RViz，`--attach` 让当前终端附着到 `stage3_ego` tmux 会话并显示
任务日志。命令中不带 `--bag`，因此不会启动 rosbag recorder。

`--report FILE` 是任务进度 CSV，不是 rosbag；其中记录状态转换、目标和时间线。
如果省略 `--report`，脚本仍会自动把 CSV 写到 `/tmp/astra_stage3_evidence/`。
如果也不想指定 CSV 路径，可以继续省略它。

不使用 `--attach` 时，可以在启动后另开终端查看日志：

```bash
tmux attach -t stage3_ego
```

飞行完成并确认日志出现 `DONE`、`armed=false`、`ON_GROUND` 后，另开终端优雅停止
仿真：

```bash
cd /home/yanzu/AstraDroneOpen
scripts/run_sh/stage3_ego.sh --stop
```

每次带控制启动前仍须确认没有旧仿真进程，且已获得本轮解锁/起飞批准。

## 带控制启动前检查与安全边界

以下命令会解锁、起飞并进入 OFFBOARD，只允许在项目负责人明确批准后执行。
下面的小扇区命令只属于 2026-07-23 历史记录，当前脚本会拒绝带控制的小于
8 扇区运行；历史批准不自动授权下一次复飞：

```bash
cd /home/yanzu/AstraDroneOpen
scripts/run_sh/stage3_ego.sh --control --sector-limit 8 \
  --bag /tmp/stage3_single_sector_repeat.bag --gui --rviz --attach
```

批准前必须完成：

1. dry-run 的点云、过滤点云、`/Odometry`、TF、占据图和 planning frame 对齐；
2. `/planner/status` 为 EGO FSM 直接发布，状态时间戳/失败计数有更新；
3. `rosnode info /ego_mavros_bridge` 显示只有 bridge 发布 MAVROS 控制 topic；
4. preflight、home、任务边界和 `worksite.world` 塔/吊机几何已审阅；
5. 当前只验收完整 8 扇区闭环；不得用旧 1/2/4 扇区结果替代新状态机验收。

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

该历史轮次先复现失败、再按完整 bag 修复，最后重新执行同一个
`--control --sector-limit 1`；当时没有扩大到两/四/八扇区。后续递增验证见
第 1.5 节。

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
38 m 参数已在该轮控制启动配置中加载，但该轮成功路径未触发 R1/R2，因此当时
仍不能宣称“实际飞到 38 m 恢复高度”；后续控制证据见第 1.5 节。

## 1.5 递增 2/4/8 扇区控制飞行

本节是在 1.4 的单扇区基线之后继续执行的实际控制验证。每次飞行均在
bridge/mission 启动前开始完整录包；发现失败后先从 bag 定位根因、修改和回归，
再执行下一次控制飞行。

### 2 扇区

第一次：

```text
bag: /home/yanzu/bag/astra_stage3_two_sector_2026-07-23.bag
SHA-256: 872b5015de9986ce3a9e0ec5670b8c7bc90911c080737c2c128a8e1e3a32f93f
```

两个扇区和 EGO 返航均已完成，但 mission 把返航和落地共用
`goal_timeout=180 s`。返航消耗接近 180 s，进入正常 AUTO.LAND 约 5.4 s 后
被任务误判为 `FAILURE_LANDING`；PX4 最终仍安全解锁接地。修复为独立的
`return_timeout=300 s` 和 `landing_timeout=90 s`，并使用 bridge 状态转换进入
时刻计时。

成功复测：

```text
bag: /home/yanzu/bag/astra_stage3_two_sector_success_2026-07-23.bag
analysis: /home/yanzu/bag/astra_stage3_two_sector_success_2026-07-23.analysis.json
SHA-256: 3ba225d9237500ec3fa03b1d616ffc91337d486637b5c5e02e21e87a0524c56d
duration/messages: 489.343 s / 694,889
sectors: [2,3]
tracking: max 0.248 m, mean 0.047 m, P95 0.076 m
terminal: armed=false, ON_GROUND, bridge/task DONE
```

### 4 扇区：失败链与安全返航设计

4 扇区共经历四次失败和第五次成功。失败均安全落地，但不能算任务通过：

1. `astra_stage3_four_sector_2026-07-23.bag`：
   sector 5 直接到 home 的返航弦穿过塔 keep-out，形成 1,213 次
   current-position collision、461 次 emergency；最后在塔旁落地。mission
   还错误地仅凭 bridge `DONE` 判成功。
2. `astra_stage3_four_sector_success_2026-07-23.bag`：
   文件名是历史遗留，实际失败。瞬时 FCU stale 后 R1/R2/re-entry 成功；
   38 m egress 的 planner/MAVROS 对齐误差 0.287–0.313 m 超过旧 0.25 m 门限，
   在塔外安全落地。
3. `astra_stage3_four_sector_attempt3_2026-07-23.bag`：
   egress 到达径向目标时先处理 `TRAJECTORY_EXPIRED`，重建后重复发布近目标；
   EGO 返回 `Close to goal`，重试耗尽后塔外落地。
4. `astra_stage3_four_sector_attempt4_2026-07-23.bag`：
   前述状态转换已修复，多段返航圆弧成功；后续瞬时位置对齐 0.401 m 略超
   0.40 m，重试耗尽后塔外落地。

由此新增：

* `RETURN_EGRESS`：从当前扇区径向外移到 24 m、升至 38 m，按不穿塔/吊机
  保守包络的方向走最大 30° 圆弧段，再到 home 上空；
* `RETURN_HOME_OVERHEAD` 后才让 bridge/EGO 沿 home 垂直走廊下降；
* 每段先判断位置到达，再处理规划失败/轨迹过期；重建时跳过小于 0.5 m 的
  近重复目标；
* bridge `DONE` 只有在距捕获 home 不超过 1.5 m 时才允许 task `DONE`；
* 被吊机 OBB 覆盖的 sector 1 增加同扇区 `+8 m` 高度候选，保持角扇区唯一；
* Stage 3 专用位置对齐门限为 0.75 m，通用 bridge/Stage 2 仍为 0.25 m；
  独立的 1.0 m 持续 1.0 s 跟踪保护保持不变。

第五次成功：

```text
bag: /home/yanzu/bag/astra_stage3_four_sector_attempt5_2026-07-23.bag
analysis: /home/yanzu/bag/astra_stage3_four_sector_attempt5_2026-07-23.analysis.json
SHA-256: 8c21304b5e7ec162d480efe2456e974a2a0cebe087dbff4133edd551d6c75fac
duration/messages: 923.540 s / 1,319,838
sectors: [1,2,3,4]
planner: no collision/emergency, max consecutive failures 0
tracking: max 1.086 m, P95 0.441 m
above 1.0 m: maximum continuous duration 0.779 s < 1.0 s protection
terminal: armed=false, ON_GROUND, bridge/task DONE
```

这次 recorder 被旧 `--stop` 在索引写完前随 tmux 强杀，留下 `.bag.active`。
`rosbag reindex` 恢复了 4,885 个 LZ4 chunk，原未索引备份保留为
`.bag.orig.active`。停止脚本随后改成先独立 SIGINT recorder、最长等待 120 s
完成索引，再停止其他窗口；最终 8 扇区验证了该顺序可以直接生成正式 bag。

### 最终 8 扇区

证据：

```text
bag: /home/yanzu/bag/astra_stage3_eight_sector_2026-07-23.bag
csv: /home/yanzu/bag/astra_stage3_eight_sector_2026-07-23.csv
analysis: /home/yanzu/bag/astra_stage3_eight_sector_2026-07-23.analysis.json
SHA-256: 08f973610f9f0071bd0c228c401df06641ba0dc589556da3f3bb5437790e80d6
time: 10614.800–11539.550, 924.750 s
messages/chunks: 1,320,782 / 4,873 LZ4
```

完整任务时间线：

| 仿真时间 | bag 直接证据 |
|---:|---|
| 10618.486–10633.041 | `WAIT_INPUTS → PRESTREAM → ARM_OFFBOARD → TAKEOFF → HOVER_READY` |
| 10633.070 | 选择 `ENTRY_GATE_a23`，开始三维分段 `APPROACH` |
| 10752.680–10790.969 | `s1_c47`，sector 1 在 38 m 完成 |
| 10791.020–10854.569 | `s2_c39`，sector 2 完成 |
| 10854.620–10918.969 | `s3_c0`，sector 3 完成 |
| 10919.025–10973.770 | `s4_c0`，sector 4 完成 |
| 10973.822–11019.769 | `s5_c0`，sector 5 完成 |
| 11019.822–11075.170 | `s6_c0`，sector 6 完成 |
| 11075.221–11121.320 | `s7_c0`，sector 7 完成 |
| 11121.370–11174.969 | `s0_c39`，sector 0 完成；8 个扇区无重复 |
| 11174.969–11350.022 | `RETURN_EGRESS` 在 38 m 沿塔外侧分段到 home 上空 |
| 11350.022–11497.641 | `RETURN_HOME`，bridge 通过 EGO 沿 home 垂直下降 |
| 11497.662 | home hover 到达，PX4 接受 `AUTO.LAND` |
| 11516.943–11516.969 | `armed=false`、`ON_GROUND`、bridge/task `DONE` |

自动审计结果：

```text
unique sectors: [1,2,3,4,5,6,7,0], repeated=[]
PlannerStatus: 18,403 × failure_reason=NONE
collision/emergency/max consecutive failures: false/false/0
raw PositionTarget: 44,900 × type_mask=0
MAVROS control publisher: /ego_mavros_bridge only
tracking error: max 0.597 m, mean 0.073 m, P95 0.195 m
MAVROS speed: max 0.774 m/s, P95 0.325 m/s
sampled inflated-occupancy voxel-center distance: minimum 3.247 m
final position: (0.045,-0.039,-0.008)
```

这只证明 2026-07-23 旧状态机在当时单机 Gazebo/PX4 SITL 静态场景通过。
它没有 `8→1` 闭环、当前分段直升覆盖门禁、NORMAL_RETURN 或 HOME_HOVER，
不能替代 2026-07-24 新状态机验收。

# 2. 2026-07-23 旧设计与状态机（历史）

## ENTRY_GATE

本节记录旧版本，当前设计以第 0 节和 YAML 为准。旧版本曾取消
“低空接近点 → 固定 XY 原地爬升”，并在巡检高度生成特殊
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
     → RETURN_EGRESS → RETURN_HOME → DONE
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

# 3. 2026-07-23 历史参数与当前 topic

下表的 rolling ENTRY_GATE、递增 sector_limit 和主动落地描述属于旧版本。
当前新增参数和语义见第 0 节以及 `stage3_ego.yaml`。

阶段三配置：`AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/config/stage3_ego.yaml`

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `inspection_height` | 30 m | ENTRY_GATE 和巡检扇区高度 |
| `recovery_height` | 38 m（上限 40 m） | attempt 2 的 R1/R2/re-entry 和最终 RETURN_EGRESS/sector 1 高空候选已实际飞到该高度 |
| `virtual_ceil_height` | 45 m | EGO/任务虚拟上限 |
| `radius` / `sector_count` | 14 m / 8 | 原 8 个扇区保持不变 |
| `sector_limit` | 8 | 配置默认策略；已按 2、4、8 递增完成控制验证 |
| ENTRY_GATE 角度/半径 | -40° 至 +40° 九组 / +2,+4,+6 m | 27 个候选，覆盖吊臂两侧安全进场区域 |
| `entry_gate/maximum_segment_length` | 6 m | 三维滚动子目标 |
| `minimum_clearance` | 2 m | 塔、吊机、点云膨胀共同门槛 |
| 吊机 OBB | center `(2.7535,14.7908)`、half extent `(3.2732,19.5424)`、yaw `-0.479608`、z `0–35.0283` | 由 `<state>` 位姿和 DAE inch→m 包围计算 |
| `max_recovery_attempts` | 2 | 每扇区有界恢复 |
| `return_timeout` / `landing_timeout` | 300 s / 90 s | EGO 返航和 AUTO.LAND 独立计时 |
| RETURN_EGRESS | 24 m / 38 m / 最大 30° | 塔外侧径向、圆弧和 home-overhead 分段 |
| `return_egress/minimum_goal_separation` | 0.5 m | 重建时跳过近重复目标 |
| `return_egress/home_xy_tolerance` | 1.5 m | 只允许 home 附近落地成为 task DONE |
| `alignment_position_tolerance` | 0.75 m | 仅 Stage 3；通用 bridge/Stage 2 仍为 0.25 m |
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

# 4. 修改文件（历史与本轮）

* `astra_tower_mission/include/.../stage3_planner.h`、
  `src/stage3_planner.cpp`：本轮增加名义点优先和严格字典序、覆盖硬约束、
  直线走廊软风险、固定 XY 分段和显式闭环多圈；历史上还包括三维滚动
  子目标、吊机 yaw 定向三维包围盒、最近已接受扇区选择、同扇区高空候选、
  安全 RETURN_EGRESS 生成、home 落地点约束和已有候选/恢复逻辑；
* `src/stage3_ego_mission_node.cpp`：本轮增加 provisional gate、STAGING_POINT、
  真实覆盖等待、固定 XY 分段、最多 3 次 gate 重定位、按 goal generation
  消费真实规划结果、闭环多圈、NORMAL_RETURN 和 HOME_HOVER 终态证明；历史
  RETURN_EGRESS 和 recovery 链保留为备用；
* `config/stage3_ego.yaml`、`test/stage3_planner_test.cpp`、
  `test/stage3_no_control_integration.{test,py}`；
* `ego_gazebo_bridge/src/ego_mavros_bridge.cpp`、
  `include/.../ego_mavros_bridge.h`、`command_utils.{h,cpp}`、
  `include/.../recovery_control.h`：首帧 yaw 实测初始化与限速、yaw 持续性
  对齐门禁、HOLD 输出缓存锁存、恢复监督 HOLD、cancel/resume/return 链和
  旧轨迹 generation gate；本轮增加 HOME_HOVER 和正常 AUTO.LAND 服务门禁；
* `ego-planner/.../ego_replan_fsm.{h,cpp}`、
  `planning_status_tracker.h`、`traj_server.cpp`：真实 FSM 状态/失败计数、
  cancel 传播、旧 B-spline 隔离；未修改 EGO 优化数学、PX4 或 FAST-LIO；
* `scripts/run_sh/stage3_ego.sh`、`stage3_record_bag.sh`：修正 odom 等待，
  支持 `--bag` 在任务启动前集成录制，补齐控制出口和诊断 topic，并拒绝覆盖
  已有 bag；停止时先等待 recorder 完成索引；
* `scripts/tool/analyze_stage3_bag.py`：bag 必需 topic、状态、扇区唯一性、规划
  安全、持续跟踪误差、控制权、frame、终态和采样占据净空审计。

# 5. 测试和验证证据

本节后续大段是 2026-07-23 历史证据。2026-07-24 当前代码的测试与失败控制
复测以第 0.6 节为准。

自动化与编译验证：

```text
相关包与 stage3_ego_mission_node 构建     通过
catkin_test_results                   146 tests
errors / failures                     0 / 0
stage3_ego.sh / recorder bash -n       通过
analyze_stage3_bag.py py_compile       通过
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

2026-07-23 旧状态机的递增控制飞行曾验证：

```text
ENTRY_GATE_a23 五段三维进场：通过
2 扇区：首轮超时误判修复后复测通过
4 扇区：4 轮失败逐项修复，第 5 轮通过
8 扇区：[1,2,3,4,5,6,7,0] 全部唯一完成
RETURN_EGRESS：38 m 塔外侧多段返航通过
PlannerStatus：18,403 条 NONE，最大连续失败 0
bridge：全程无 HOLD，唯一 raw-local 出口，type_mask=0
最终 8 扇区跟踪误差：max 0.597 m，P95 0.195 m
采样膨胀占据体素中心距离：min 3.247 m
返航/降落：正常 AUTO.LAND，armed=false，ON_GROUND，双方 DONE
```

attempt 2 已在实际控制中触发并完成 38 m R1/R2/re-entry，但该轮随后因返航
对齐门限失败；最终 4/8 扇区成功轮没有故意注入 HOLD。仍未飞行验证的是全部
服务失败、长期地图过期、不可达目标、动态障碍和真机 TF 外参。最终 8 扇区的
新停止顺序已直接生成正式 `.bag`，没有遗留 `.active`。

# 6. 已知风险与下一步

* 当前没有阻止单机静态 SITL 验收的关键问题。MID-360/FAST-LIO 命中点链仍
  没有正式 free/unknown 体素；最终实现把 unknown 作为独立诊断，已知占据和
  EGO 作为硬门禁。该仿真结论不能外推为真机自由空间证明；
* 当前 `map -> camera_init` 单位变换已由最终 bag 复核，但仍只是仿真假设，
  不是实测标定结果；
* 占据图盖章适配只修时间戳，不独立证明体素坐标正确；
* EGO direct `PlannerStatus` 是外层 FSM 真值，不是优化器内部每个候选体素的
  解释器；长期地图过期、全部服务同时失败和持续控制断流仍需继续专门注入；
* `tower_crane` OBB 是保守预过滤，会包含网格空隙。路径避障仍由
  FAST-LIO/EGO 地图负责；2026-07-24 最新 bag 的采样膨胀体素中心最小距离
  2.172 m，不是连续机体表面净空证明。逐扇区复核后，s3～s7 使用 30 m
  名义点，s2 使用 31 m 最小改动备用点，s0/s1 使用 38 m 备用点；
* bridge 仍执行 raw-local `PositionTarget`，只允许其一个 MAVROS 控制出口；
* Stage 3 位置对齐门限 0.75 m 来自长航段实测。最新 8 扇区最大跟踪误差
  0.643 m、P95 0.128 m，仍需在更长时和扰动场景监控 estimator 漂移；
* `/Odometry` 在当前 FAST-LIO 仿真输出中的 twist 为零，任务速度到达条件可能
  低估真实速度；分析器已用 MAVROS odom 独立统计速度，但速度源契约仍应显式化；
* 最终成功轮在 sector 0 实际触发一次 no-progress HOLD，并保持同一目标恢复；
  无控制集成还注入了 bridge HOLD 和两次 planner failure，但不能据此宣称全部
  故障组合均已控制验收；
* 本阶段正常落地已统一为 EGO home hover 后由任务请求 PX4 AUTO.LAND；
  bridge 的控制权严重故障 failsafe 仍是独立安全分支；
* 动态障碍、多机、相机检测、Cloud/QGIS、真机和干净 clone 交付均未验证；
* 当前工作区未 commit、未 push，FAST-LIO `Log/mat_pre.txt` 是仿真运行写入，
  未被清理。

无控制复核命令：

```bash
cd /home/yanzu/AstraDroneOpen
scripts/run_sh/stage3_ego.sh --sector-limit 8 \
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

2026-07-23 旧 8 扇区 bag 的离线复核：

```bash
source /opt/ros/noetic/setup.bash
rosbag info \
  /home/yanzu/bag/astra_stage3_eight_sector_2026-07-23.bag
sha256sum \
  /home/yanzu/bag/astra_stage3_eight_sector_2026-07-23.bag

cd /home/yanzu/AstraDroneOpen
python3 scripts/tool/analyze_stage3_bag.py \
  /home/yanzu/bag/astra_stage3_eight_sector_2026-07-23.bag \
  --occupancy-stride 10 \
  --output \
  /home/yanzu/bag/astra_stage3_eight_sector_2026-07-23.analysis.json
```

任何后续带控制复测仍须重新确认环境、控制权和明确批准。

# 7. 2026-07-24 yaw 与逐扇区候选高度修正

## 7.1 bag 根因

上一成功 bag
`/home/yanzu/bag/astra_stage3_vertical_entry_final_retry1_2026-07-24.bag`
证明了两个相互独立的问题：

1. 纯垂直段水平速度接近零，前进方向没有定义；bridge 旧逻辑此时保留
   traj_server 输入 yaw。该输入在分段轨迹切换处可瞬时跳变，虽有 30 deg/s
   限速，机体仍会明显转头。上一 bag 上升实际 yaw 速率 p95/max 为
   44.29/60.27 deg/s，而 roll/pitch 均小于约 1.3 deg，因此上升画面乱晃的
   主因是 yaw，不是姿态倾斜。
2. 候选本来逐扇区重新评估，但从 s1 的 38 m 备用点出发后，s3～s7 的 30 m
   名义点位于 MID360 当前向下视场之外，被瞬时视场启发式误判为
   `UNKNOWN_REGION`，从而形成“前一扇区升高，后续扇区全部保持高空”的高度
   级联。bag 中这些名义点的静态净空为 7.19 m，并非被占据或 EGO 不可达。
3. 绕塔画面晃动不是 tower-facing yaw 失稳：旧稳定 bag 与上一 bag 的实际
   yaw 速率 p95 分别为 1.45 和 1.51 deg/s；真正变化的是 roll p95 从
   1.10 增到 2.43 deg。D435i 与机身刚性连接，无云台，EGO 分段轨迹的横向
   加减速和重规划姿态会直接进入画面。

## 7.2 最终修改

- bridge 的 velocity-facing yaw 在水平速度低于阈值时，改为保持实测/上一
  有效机体 yaw，不再接受未定义的轨迹前视 yaw；因此 home-local 垂直上升锁定
  起飞悬停 yaw。
- 任务管理器只有 `GoalKind::kSector` 发布朝塔模式；ENTRY_GATE 水平进场、
  recovery、NORMAL_RETURN 和 RETURN_EGRESS 均保留朝运动方向，非巡检目标
  不再等待塔向 arrival yaw。
- ENTRY_GATE 的安全高度建图覆盖约束保持不变。对巡检扇区终点，由于当前
  FAST-LIO 链只有 obstacle hits、没有持久 free/unknown 体素，瞬时传感器视场
  unknown 改为 `UNKNOWN_REGION_DIAGNOSTIC`；地图新鲜度、实际占据、塔/吊机
  静态净空和 EGO 不可达仍是各自独立硬约束。
- 名义点与备用点继续在每个扇区独立生成、过滤和锁定；可用 `c0` 绝对优先，
  只有硬约束拒绝名义点后才选择改动最小备用点。

## 7.3 构建、测试与控制复飞证据

相关三包构建通过，`catkin_test_results` 为 160 tests、0 errors、
0 failures、0 skipped；新增覆盖“低水平速度保持机体 yaw”“unknown 可仅作
扇区诊断”和 ENTRY_GATE 非朝塔策略。无控制集成测试通过。

最终完整 8 扇区 bag：

```text
/home/yanzu/bag/astra_stage3_yaw_sectorfix_2026-07-24.bag
/home/yanzu/bag/astra_stage3_yaw_sectorfix_2026-07-24.analysis.json
```

关键结果：

```text
SEGMENTED_CLIMB duration             101.250 s
上升 yaw 起点/最大偏差                0.040 / 0.035 deg
上升实际 yaw rate p95/max            0.223 / 0.456 deg/s
上升最大 XY 漂移                     0.071 m
目标序列                              s1_c3(38), s2_c50(31),
                                      s3_c0～s7_c0(30),
                                      s0_c3(38), s1_c3(38闭环)
HOLD / planner连续失败                0 / 0
跟踪误差 mean/p95/max                 0.061 / 0.128 / 0.643 m
采样膨胀占据最小距离                  2.172 m
绕塔 roll p95/max                     1.074 / 2.204 deg
绕塔 pitch p95/max                    0.821 / 1.245 deg
终态                                  正常返航 → HOME_HOVER →
                                      AUTO.LAND → disarmed/ON_GROUND/DONE
```

s1 的 30 m 名义点以 `KNOWN_OBSTACLE_CLEARANCE=-3.360 m` 被拒绝，s2 的
30 m 名义点净空 0.957 m，故选择 31 m 最小备用点；s0 的 30 m 名义点净空
-0.761 m，故选择 38 m。s3 的 30 m 名义点虽记录
`UNKNOWN_REGION_DIAGNOSTIC`，但静态净空 7.190 m、实际占据检查通过，由 EGO
成功规划并到达；s4～s7 也均使用 30 m 名义点。这是逐扇区独立选择且不发生
高度级联的直接证据。

尚存限制：相机仍无云台，EGO 为避障产生的真实 roll/pitch 不可能由 yaw 策略
完全消除；最新复飞已恢复到旧稳定 bag 的姿态水平，但更复杂障碍和更高速度下
仍需继续监控画面稳定性。当前 unknown 处理仅适用于这条仿真 obstacle-hit
点云链，不能外推为真机自由空间证明。
