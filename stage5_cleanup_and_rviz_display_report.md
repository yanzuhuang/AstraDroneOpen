# Stage 5 临时数据清理与 RViz 显示报告

日期：2026-08-03  
分支：`ego-swarm`  
检查时 HEAD：`f68f07137db57698c15f39d093f812ff9851da15`

## 1. 清理结果

最新且文件组成完整的证据组 `test_evidence/stage5_false_obstacle_fix_20260803/` 已整组保留，没有按单个文件修改时间拆分。该组是最新的定向验证证据组；根据其正式报告，它验证的是 WP4～WP5 窗口，不将其误述为完整绕塔和安全落地验收。

原计划直接删除全部旧 Stage 5 目录，但安全审查认为多组 2026-08-03 专项审计的替代关系不明确。最终遵守“不确定时保留”，执行了仓库既有 `test_evidence/stage5_cleanup_audit.md` 中逐项审计过的安全清单：共删除 83 个精确目标，其中包含 828 个 Git 已跟踪文件，按审计记录释放约 23.382 GB。

### 1.1 删除列表

| 旧实验目录 | 删除内容 |
|---|---|
| `stage5_control_20260802_142342`、`stage5_control_20260802_150614` | `stage5.bag`、`candidates.jsonl`、`trajectory_xy.png`、`ros_logs/` |
| `stage5_dry_run_20260802_141530`、`stage5_dry_run_20260802_141804` | `stage5.bag`、`candidates.jsonl`、`trajectory_xy.png`、`ros_logs/`、巨大顶层 `roslaunch.log` |
| `stage5_dry_run_20260802_142039`、`stage5_dry_run_20260802_150230` | `stage5.bag`、`candidates.jsonl`、`trajectory_xy.png`、`ros_logs/` |
| `stage5_final_validation_20260802_170608`、`171112`、`171440`、`181011`、`181324`、`191822`、`192104`、`192512`、`192759`、`203328` | 各目录的 `stage5.bag`、`candidates.jsonl`、`trajectory_xy.png`、`trajectory_3d.png`、`ros_logs/` |
| `stage5_final_validation_20260802_191948` | `stage5.bag`、`candidates.jsonl`、`swarm.csv`、`uav1.csv`、`uav2.csv`、`uav3.csv`、`ros_logs/`；该次仅运行 1.077 s，数据为空或重复 |

上述旧目录中的 `summary.json`、`run_metadata.txt`、必要顶层日志、有效 CSV 和不可重建的根因 JSON 保留，避免丢失故障结论。

### 1.2 保留的最新实验数据

完整保留 `test_evidence/stage5_false_obstacle_fix_20260803/`（约 8.7 GB），包括：

- `gazebo_collision_view.png`；
- `uav1/` 下的 `uav1.csv`、`swarm.csv`、`summary.json`、`candidates.jsonl`、`fake_center_crossing_layers.bag` 和 ROS 日志；
- `uav2/fake_center_crossing_layers.bag`；
- `uav3/fake_center_crossing_layers.bag`；
- `triple_validation/` 下的 `stage5.bag`、`wp3_wp5_layers_full.bag`、`summary.json`、`run_metadata.txt`、`swarm.csv`、三机 CSV、`candidates.jsonl`、`trajectory_xy.png`、`trajectory_3d.png`、`roslaunch.log`、`rosbag.log` 和 64 个 ROS 节点日志。

### 1.3 因用途或替代关系不确定而保留

- `test_evidence/ego_swarm_three_uav/`；
- `test_evidence/stage5_dynamic_entry_20260801*`、`stage5_full_orbit_20260802/`；
- 最终成功预检与完整控制证据 `stage5_final_validation_20260802_212525/`、`stage5_final_validation_20260802_212843/`；
- 2026-08-03 的 `stage5_control_20260803_143131/`、首点 Tier、八边形优化、self-cloud、UAV2 占据/对齐/根因专项数据；
- 上述已瘦身旧目录中的轻量摘要、metadata、有效 CSV 和根因 JSON；
- `artifacts/stage3_*`、`AstraDrone_ros1_ws/log/stage3_two_layer/`、文档引用的 `trc_picture/`；
- 已存在用户修改的 `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/Log/mat_pre.txt`；
- 源码/仿真模型/第三方目录内作为资源或配置使用的 `.json`、`.csv`、`.png` 和日志样例。

正式报告、项目文档、脚本、launch、RViz、world 和源码均未删除。

## 2. RViz 基准和话题关系

单机低空避障基准是：

- 启动：`astra_tower_mission/launch/low_altitude_inspection.launch`；
- 中间 include：`ego_gazebo_bridge/launch/ego_gazebo_bridge.launch`；
- 实际 RViz 配置：`ego_gazebo_bridge/rviz/ego_gazebo_bridge.rviz`。

三机实际启动 `astra_swarm_bringup/launch/triple_tower_inspection.launch` 加载 `astra_swarm_bringup/config/triple_tower.rviz`。修改前确认的话题为：

- 三机任务层全局参考：`/uav1/tower_mission/mission_route`、`/uav2/tower_mission/mission_route`、`/uav3/tower_mission/mission_route`；
- 三机实际历史轨迹：`/uav1/swarm/actual_path`、`/uav2/swarm/actual_path`、`/uav3/swarm/actual_path`；
- 8 个主航点 Marker：`/swarm/rviz/mission_markers`；
- EGO 局部轨迹：`/uav1/drone_0_ego_planner_node/optimal_list`、`/uav2/drone_1_ego_planner_node/optimal_list`、`/uav3/drone_2_ego_planner_node/optimal_list`。

实际历史轨迹由 `swarm_rviz_diagnostics.py` 订阅三机各自 `/uavN/Odometry` 后按 0.03 m 位移阈值累计为 `nav_msgs/Path`，不是任务层预设路径。

## 3. RViz 修改

修改文件：

- `astra_swarm_bringup/config/triple_tower.rviz`；
- `astra_swarm_bringup/scripts/swarm_rviz_diagnostics.py`。

未修改 launch；三机实际 launch 原本已经永久加载上述 `triple_tower.rviz`。

后续因完整三机 GUI 运行触发主机 OOM，按用户要求恢复到此前三机 RViz 的轻量显示风格：2 m 网格、深色背景，以及三色、半透明的 `/uavN/cloud_registered_peer_filtered` 点云。EGO 仍只显示三机局部 `optimal_list` 与局部 goal。

主航点仍由同一个中心发布器按原塔心、原半径、原顺序和固定 8 扇区生成；话题为 `/swarm/rviz/mission_markers`，位置 namespace 为 `main_waypoints`，短标签 namespace 为 `main_waypoint_labels`。显示内容只有一组 8 个球体和 `WP1`～`WP8`；没有 24 个重复航点。

已从三机 RViz 配置移除/关闭：

- `/uav1/tower_mission/mission_route`；
- `/uav2/tower_mission/mission_route`；
- `/uav3/tower_mission/mission_route`；
- 预设绕塔圆环、ENTRY/EXIT/PRE_ENTRY/FIRST 标记及 home→pre-entry→entry→first 参考连线；
- 长篇状态/协调调试文字（状态 Marker 仅显示 `uav1`、`uav2`、`uav3` 和 `swarm`）。

保留：

- 三机 RobotModel，Display Name 为 `uav1`、`uav2`、`uav3`；
- 三机 FAST-LIO/规划输入点云；
- 三机 EGO 局部轨迹和局部 goal；
- 三机当前局部目标 `/uavN/tower_mission/current_target`；
- `/uav1/swarm/actual_path`、`/uav2/swarm/actual_path`、`/uav3/swarm/actual_path`；
- 一组 `/swarm/rviz/mission_markers` 的 `WP1`～`WP8`；
- Grid 和 TF（TF 名称只在画面中隐藏，没有修改任何 TF frame）。

## 4. 验证

- `roslaunch ... triple_tower_inspection.launch --nodes start_sim:=false start_rviz:=false enable_control:=false`：参数/launch 展开通过，确认实际 RViz/诊断节点和三机独立 EGO/FAST-LIO 节点关系；未启动节点。
- Python 语法检查：通过。
- Marker 隔离测试：临时 ROS master 中消息为 1 个 `DELETEALL`、8 个 `main_waypoints` 球体和 8 个 `main_waypoint_labels`，文字严格为 `WP1`～`WP8`。
- 实际轨迹链测试：向隔离 master 注入三条测试 Odometry 后，三个 `/uavN/swarm/actual_path` 均由 `/swarm_rviz_diagnostics` 发布。
- RViz 启动：RViz 1.14.26、Qt 5.12.8、OGRE 1.9.0、OpenGL 4.6 正常初始化；补入项目既有只读 PX4 Gazebo package 路径后，三架 `iris.stl` 不再报资源缺失。12 秒后由 `timeout` 主动结束，退出码 124 为预期。
- 本任务相关两个文件的 `git diff --check` 通过。全工作区检查仍会报告既有 `FAST_LIO/Log/mat_pre.txt` 行尾空格，该文件未由本任务修改或清理。

本次没有启动 Gazebo/PX4/MAVROS，没有解锁或飞行，没有修改 ROS namespace、Topic 通信关系、TF frame、8 个任务航点、ENTRY_GATE、EXIT_GATE、同步、返航、避障、EGO/FAST-LIO 参数或任何飞行/轨迹规划逻辑。

## 5. 2026-08-03 OOM 故障复核与显示回退

- 内核日志明确记录 `ReceiveQueue invoked oom-killer`，随后以 OOM 原因杀死 `gzserver`；被杀时 `gzserver` 的匿名常驻内存约 5.77 GB。因此“长时间不起飞”不是航点或规划参数问题，而是仿真进程在严重资源压力下失去实时性并最终退出。
- 同一时刻 RViz 诊断日志记录 `/clock` 连接断开。后续 RViz 空白是 Gazebo 时钟和数据发布者消失后的结果，没有发现 RViz 配置损坏或 RViz 独立崩溃的证据。
- RViz 已回退到此前三机配置的显示风格，但未恢复三条 `/uavN/tower_mission/mission_route` 全局参考 Path；真实历史轨迹 `/uavN/swarm/actual_path` 继续保留。
- RobotModel 名称为 `uav1`、`uav2`、`uav3`；其他各机 Display Name 也统一使用 `uavN cloud/EGO/goal/path/target` 短名称。TF 名称继续隐藏，只改变画面标签，不改变 frame。
- 回退后完成 Python AST、RViz YAML、launch 展开和话题静态检查；没有再次启动 Gazebo、PX4、MAVROS 或执行飞行。

## 6. 最新录制数据追加清理

按用户后续指示，已整组删除最新录制目录 `test_evidence/stage5_control_20260803_180328/`（约 1.6 GB）。删除内容包括 `stage5.bag`、三机 CSV、`swarm.csv`、`candidates.jsonl`、`summary.json`、二维/三维轨迹图、运行 metadata、rosbag/roslaunch 日志及该次 ROS 节点日志。没有删除此前两次 OOM 故障目录或其他用途不确定的实验数据。

## 7. 2026-08-05 缓存清理后续

本报告第 1 节记录的是 2026-08-03 当时的保留状态。2026-08-05 后续缓存清理会从当前版本移除原始 rosbag、ROS 节点日志、`master.log`、`roslaunch.log` 与 `rosout.log` 等可再生运行产物；最终实验的 CSV、JSON、轨迹图、分析脚本和报告继续保留。八边形根因分析所需的两组 `roslaunch.log` 与各自 `rosout.log` 例外保留。文中对原始 bag/日志“保留”或“封存”的陈述应理解为该报告写作时的历史事实，而非清理后的当前文件清单。
