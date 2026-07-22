# 阶段三学习说明：EGO 静态障碍、不可达目标与安全恢复

更新日期：2026-07-22。本文对应 `ego-project` 当前工作区，不替代
`AGENTS.md`、源码、launch 或现场记录。阶段三只针对单机 ROS1 Noetic、Gazebo
Classic、PX4 SITL、MAVROS、FAST-LIO 和 EGO-Planner；YOLO、动态障碍、多机和
真机部署不是本阶段范围。

## 1. 目标、范围和非目标

原来的任务层把 8 个固定点依次交给 EGO。阶段三将其变成 8 个有角度、半径和
高度边界的巡检扇区：任务层在有限候选集合中做硬安全过滤和评分，锁定一个
目标后再交给 EGO；目标占据、净空不足、地图过期或路径长时间无进展不会盲跳
下一点，而是进入 HOLD、同扇区重定位、R1/R2/P' 恢复，最后有界地返航或降落。

已实现的主链是 `stage3_ego_mission_node`、`planner_status_adapter_node`、
`stage3_planner` 和 bridge 的高层服务。没有任何新节点发布 MAVROS setpoint。

## 2. 原故障与证据边界

2026-07-22 bag 的第二个 30 m 绕塔目标触发 EGO “terminal point ... obstacle”、
“emergency stop”和“drone is in obstacle”。原 bag 只有 MAVROS 状态/位姿和
`/rosout`，没有点云、TF、占据图、EGO B 样条、PositionCommand 或 bridge 状态，
因此只能证明无人机停在约 `(3.0, 13.7, 29.8)`，不能证明哪一个体素、哪一类
障碍或哪个 frame 错位导致占据。阶段三录包脚本加入了原始/注册/过滤点云、
Odometry、occupancy、EGO、PlannerStatus、任务/bridge 状态和 TF；在这些证据
对齐前，不应把故障根因写成“吊塔一定挡住了路”。

当前文档中的故障根因仍是**未确认**：可能是 tower crane 未进入任务层几何、
点云/odom frame 不一致、陈旧膨胀体素或局部地图上限太低。阶段三的静态几何
只作保守预过滤，EGO 点云和占据图仍是避障权威。

## 3. 三层状态机职责

* 任务层：`WAIT_INPUTS → APPROACH → EVALUATING/TARGET_LOCKED/NAVIGATING →
  HOLDING/RELOCATING/RECOVERING → RETURN_HOME/FAILURE_LANDING`。它负责扇区顺序、
  候选点、锁定/迟滞、失败计数和最终政策。
* EGO FSM：继续负责局部占据、A*、B 样条优化和重规划。本阶段不改 vendor
  优化数学核心；外部 `planner_status_adapter_node` 把可观测输入转换成机器可读
  的成功、失败、无进展和 emergency-stop 事件。
* bridge/PX4：唯一允许发布 `/mavros/setpoint_raw/local` 的控制层，负责 preflight、
  OFFBOARD、HOLD 悬停、轨迹失效、返航和降落。任务层只调用高层 service。

## 4. 外侧观察点与分段进场

`mission/observation_radius_offset` 默认 5 m，观察点位于巡检圆外侧；从当前
Odometry 到观察点按 `approach_segment_length`（默认 6 m）拆成有限的局部子目标，
最后再从 `transit_height`（4 m）爬升到 `inspection_height`（30 m）。这避免一次把
远处 30 m 目标直接交给局部地图。bridge 仍先进入 `HOVER_READY`，首个合法 EGO
命令和显式 `resume_ego` 通过后才进入 `TRACK_EGO`。

## 5. 扇区数据结构

`stage3_planner.h` 中的 `Sector` 包含 `sector_id`、`layer_id`、名义角度/半径/高度、
允许角度/半径/高度范围、候选集合、当前锁定索引、状态、恢复次数和失败原因。
当前配置强制 `sector_count=8`、`layer_count=1`；数据结构保留 layer 字段但不实现
多层巡检。扇区状态为 `PENDING/EVALUATING/TARGET_LOCKED/NAVIGATING/HOLDING/
COVERED/RELOCATING/RECOVERING/FAILED`。

## 6. 候选生成、硬过滤和评分

每个扇区有限生成 `5 × 3 × 3` 个候选（角度 `0,±5,±10°`，半径 `0,+2,+4 m`，
高度 `0,±1 m`），所有偏移都可在 YAML 中替换。硬过滤顺序包括：

1. 点云地图在 `map_timeout` 内且 frame 为 planning frame；
2. 目标高度、扇区角度和半径边界有效；
3. 塔保守禁区外仍有 `minimum_clearance`；
4. 已知静态障碍圆柱禁区外有净空；
5. 目标点和当前位置到目标的采样走廊不撞点云/障碍；
6. 未知区域比例不超过配置上限（当前适配器以点云新鲜作为最小证据）。

任何硬约束失败都直接记录 `MAP_STALE/OUT_OF_BOUNDS/TOWER_KEEP_OUT/
KNOWN_OBSTACLE_CLEARANCE/OCCUPANCY_OR_CLEARANCE/NO_LOCAL_CORRIDOR/UNKNOWN_REGION`，
不参与加权折中。通过后才按净空、名义偏差、当前位置距离、连续性和未知比例评分。
`/tower_mission/candidate_targets` 发布每个候选的 accepted、reason、clearance、
score 和 map timestamp。

## 7. 目标锁定与迟滞

`chooseBestCandidate` 只在候选明显优于当前锁定目标（`target_replacement_margin`）
时替换；EGO 已开始生成有效轨迹后，正常情况下保持该 target id。只有目标进入
占据、地图失效或规划失败才解除锁定。任务评估当前扇区时同时发布下一扇区的
候选调试结果，避免到达后才首次发现“没有候选”。

## 8. 航点重定位

名义目标或当前锁定目标不安全时，任务将其拒绝并在同一扇区重新筛选；不直接跳
到下一扇区。全部候选拒绝后先进入 HOLD，再按恢复次数执行恢复；没有安全恢复才
走返航/降落终态。

## 9. 路径不可达判断

任务层订阅 `/planner/status`，综合 `goal_in_collision`、
`current_position_in_collision`、连续规划失败计数、emergency-stop 持续时间、
轨迹过期和适配器的“速度接近零且距离窗口内无明显下降”判断。适配器不解析
`/rosout`；它只依据命令、odom、goal 和点云的机器消息。PlannerStatus 的失败原因
至少包含 `NONE/MAP_STALE/GOAL_IN_OCCUPANCY/CURRENT_POSITION_IN_OCCUPANCY/
NO_FEASIBLE_TRAJECTORY/REPLAN_FAILED/EMERGENCY_STOP_TIMEOUT/TRAJECTORY_EXPIRED`。

## 10. HOLD → R1 → R2 → P' 恢复

失败后任务调用 bridge `cancel_current_trajectory` 再调用
`enable_tracking(false)`，bridge 将旧目标/旧轨迹失效并锁存 `HOLD`。HOLD 窗口
结束后：

* R1 沿当前位置到塔中心的径向外移 `radial_step`；
* R2 在更大半径上沿配置顺/逆时针切向移动 `tangent_step`；
* P' 回到当前扇区的名义半径和角度，重新走候选过滤；
* 每次恢复都计数，达到 `max_recovery_attempts` 即不再循环。

恢复目标只能由任务层生成，实际从当前状态到恢复点仍由 EGO 规划。若当前位置
被占据图判定为碰撞，任务不会直接生成 R1/R2：必须先 HOLD，等待新鲜点云并确认
无人机邻域存在可靠自由空间；无法确认则返航，返航失败再请求降落。

## 11. PlannerStatus 与 bridge 接口

消息文件为 `astra_custom_msgs/msg/PlannerStatus.msg`，topic 为 `/planner/status`。
`InspectionCandidate`/`InspectionCandidateArray` 用于候选调试。bridge 新增：

* `/ego_mavros_bridge/cancel_current_trajectory` (`std_srvs/Trigger`)：清除有效目标
  和 planner target，进入锁存 HOLD；
* `/ego_mavros_bridge/resume_ego` (`std_srvs/Trigger`)：仅在 HOLD/HOVER_READY 接收，
  仍经过新鲜命令和完整 preflight；
* 原有 `/ego_mavros_bridge/enable_tracking`：阶段三把 false 视为高层 HOLD 请求，
  true 仍只允许 bridge 自己恢复 TRACK_EGO；
* `/ego_mavros_bridge/return_home` 与 `/ego_mavros_bridge/land`：最终安全政策。

## 12. 当前点被判占据

这是最高优先级故障：取消旧轨迹、HOLD、检查点云/TF/odom/地图新鲜度和邻域自由空间。
在检查通过前不允许跳扇区、发布下一目标或生成恢复点。阶段三控制路径在
`Stage3EgoMissionNode::buildRecovery` 和 HOLD 分支中落实了这一门禁；若地图仍陈旧
或邻域仍不可信，进入返航/降落。

## 13. 高度、topic 和参数

正常巡检高度 `inspection_height=30.0`，恢复高度上限 `recovery_height_max=40.0`，
EGO 绝对上限 `virtual_ceil_height=45.0`，全部在 `config/stage3_ego.yaml` 和
stage3 launch 中传递。普通目标仍受扇区高度范围限制，不会因虚拟上限提高而自由爬升。
关键 topic：`/stage3/cloud_registered_filtered`、`/Odometry`、`/planning/goal`、
`/planning/bspline`、`/planning/pos_cmd`、`/planner/status`、
`/tower_mission/state/current_target/current_sector/candidate_targets`、
`/ego_mavros_bridge/state/tracking_error`、`/grid_map/occupancy(_inflate)`、`/tf` 和
`/tf_static`。`scripts/run_sh/stage3_record_bag.sh` 只录包，不启动控制权。

## 14. 修改文件与入口

* `Utils/astra_custom_msgs/msg/{PlannerStatus,InspectionCandidate,InspectionCandidateArray}.msg`
  及其 CMake/package 依赖；
* `MissionControl/astra_tower_mission/include/.../stage3_planner.h`、
  `src/stage3_planner.cpp`：纯逻辑生成、过滤、评分、迟滞和恢复；
* `src/stage3_ego_mission_node.cpp`：任务状态机；
* `src/planner_status_adapter_node.cpp`：机器可读 planner 外层状态；
* `config/stage3_ego.yaml`、`launch/stage3_ego.launch`；
* `ego_gazebo_bridge` header/cpp、`config/stage3_ego.yaml` 和通用 launch：
  HOLD/CANCEL/RESUME 与过滤点云 topic；
* `scripts/run_sh/stage3_record_bag.sh`：调试录包；
* `ego-stage3学习说明文档.md`：本说明。

## 15. 编译、启动、录包和调试

```bash
source /opt/ros/noetic/setup.bash
cd /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES='astra_custom_msgs;astra_tower_mission;ego_gazebo_bridge' -j2
catkin_make -DCATKIN_WHITELIST_PACKAGES='astra_custom_msgs;astra_tower_mission;ego_gazebo_bridge' \
  run_tests_astra_tower_mission run_tests_ego_gazebo_bridge -j2
catkin_test_results build/test_results
source devel/setup.bash
roslaunch astra_tower_mission stage3_ego.launch enable_control:=false rviz:=true
```

另一个终端可执行 `scripts/run_sh/stage3_record_bag.sh /tmp/stage3.bag`。启动后先用
`rostopic hz/echo` 检查 `/cloud_registered`、`/stage3/cloud_registered_filtered`、
`/Odometry`、`/tf`、`/grid_map/occupancy_inflate` 和 `/planner/status`，再用 RViz
核对 planning frame、塔模型、点云和膨胀图。只有项目负责人审阅 dry-run/preflight
和控制权唯一性证据后，才可考虑 `enable_control:=true`；本轮没有执行带控制权命令。

## 16. 故障注入与预期状态迁移

纯逻辑测试覆盖目标在障碍、塔净空不足、局部走廊阻挡、正常候选、地图过期、锁定
迟滞和 R1/R2 几何。ROS/SITL 验收应逐项注入：目标 occupancy → `HOLDING →
RELOCATING → NAVIGATING`；目标安全但路径挡住 → `HOLDING → RECOVERING(R1,R2,P')`；
连续失败/emergency 超时 → bounded recovery；当前位置 occupancy → HOLD + 自由空间
门禁；地图过期 → HOLD 后返航/降落；候选全拒绝或恢复耗尽 → 返航，返航不可行则
安全降落。每项记录 YAML、启动命令、状态迁移、关键 topic、结论和已知问题。

## 17. 验收标准与证据等级

已实现并在本工作区自动化测试通过：消息生成、构建、候选硬过滤、局部走廊检查、
评分/迟滞、恢复点几何、旧 tower/offboard/bridge 回归，共当前测试报告 100 tests、
0 errors、0 failures。已实现但尚未飞行验证：stage3 任务节点、PlannerStatus 适配器、
bridge HOLD/CANCEL/RESUME、worksite 30/40/45 m 高度配置、分段进场和完整状态迁移。
仅设计预留：真实 occupancy 体素查询、双向恢复方案择优、动态障碍、未知区域精确
比例、多层巡检和真实机外参。尚未确认：原 bag 的具体障碍体素、点云/TF 是否错位、
Gazebo 碰撞几何与回波是否一致。

## 18. 限制、风险和排障顺序

当前适配器是外层可测试启发式，不是 EGO vendor 内部 FSM 的完整真值；它不能替代
EGO 核心状态发布。候选走廊使用点云采样和已知圆柱保守模型，不能证明任意复杂障碍
的全局可达性。bridge 既有异常降落路径仍可能请求 `AUTO.LAND`，这是后续独立架构
任务。排障顺序固定为：时间戳和 frame → TF 对齐 → 原始/注册/过滤点云 → occupancy/
inflate → PlannerStatus → bridge 控制权 → 任务候选/恢复 CSV；不要从 `/rosout` 文本
反推安全状态，也不要在证据不足时直接改 EGO/PX4 核心。
