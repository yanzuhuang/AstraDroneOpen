# AstraDroneOpen 比赛提交版

环境：Ubuntu 20.04.6 LTS / ROS 1 Noetic / Gazebo Classic 11.15.1 / Python 3.8.10；CMake 3.16.3、GCC 9.4、MAVROS 1.20.1。本文件是唯一项目说明入口，维护三机同层/异层巡检与 SAC / Learning Speed 两部分。当前源码决定行为，历史运行工件决定验证范围。

## 依赖与编译

需要安装 ROS Noetic（含 catkin、RViz）、MAVROS、Gazebo ROS 插件、Eigen/PCL/OpenCV/Boost、Protobuf、APR、Python NumPy/Matplotlib/PyYAML，以及消息与 TF 工具。以下是已有 Ubuntu 20.04/Noetic 环境的依赖清单，不在本次整理中执行系统安装或升级：

```bash
sudo apt install build-essential cmake git pkg-config python3-dev python3-pip \
  python3-rosdep python3-numpy python3-matplotlib python3-yaml python3-nose \
  libeigen3-dev libpcl-dev libopencv-dev libboost-all-dev libprotobuf-dev \
  protobuf-compiler libapr1-dev ros-noetic-desktop-full \
  ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-plugins ros-noetic-pcl-ros ros-noetic-cv-bridge \
  ros-noetic-image-transport ros-noetic-camera-info-manager \
  ros-noetic-tf2-geometry-msgs ros-noetic-xacro ros-noetic-controller-manager \
  ros-noetic-controller-manager-msgs
```

MAVROS 需要 GeographicLib 数据（仓库 `third_party/GeographicLib/` 保留现有副本）。首次部署可用系统 MAVROS 的 `install_geographiclib_datasets.sh`，已配置机器不重复覆盖。Livox-SDK 源码保留在 `third_party/Livox-SDK/`，需要已有对应安装库；嵌套 Livox gitlink 缺少 `.gitmodules` 映射，不能假定 `git submodule update` 能重建全部环境。

外部 PX4 不包含在仓库：本机 `/home/yanzu/PX4-Autopilot` 使用 v1.15.4 项目定制树（基准提交 `99c40407ffd7ac184e2d7b4b293f36f10fe561ef`）。必须保留已部署的 1048/1049/1050 三机 airframe、SITL 二进制和 Gazebo 插件。新机器需另行取得该锁定环境；本轮编译不是从零部署验证。脚本支持 `ASTRA_PX4_ROOT=/你的/PX4-Autopilot`，不要直接替换为上游新版。

在仓库根目录编译，先传感器工作空间，再主工作空间：

```bash
source /opt/ros/noetic/setup.bash
cd simulation/sim_workspace
catkin_make -DCATKIN_WHITELIST_PACKAGES='livox_laser_simulation;realsense_ros_gazebo;env_map' -j2
source devel/setup.bash
cd ../../AstraDrone_ros1_ws
catkin_make -j2
source devel/setup.bash
cd ..
```

主工作空间保留 `learning_speed_rl`、`hector_ego_training_backend` 及其消息、launch、配置、测试和 Python 模块。RL 另需 PyTorch 和 Hector 源码 overlay：准备脚本为 `scripts/run_sh/reinforcement_learning/prepare_hector_training_overlay.sh`，默认源码 `/home/yanzu/rl_reference/controllers/hector-quadrotor-noetic`，overlay `/tmp/astra_hector_training_overlay`，可用 `ASTRA_HECTOR_SOURCE` / `ASTRA_HECTOR_OVERLAY` 覆盖。`runtime_artifacts/sac_python_packages/` 是现有 PyTorch 依赖目录，不是可删除的实验缓存。此轮不提供或执行强化学习启动命令。

## 三机启动命令

从仓库根目录执行。统一入口自动加载 Noetic、两个 catkin 工作空间以及 PX4/Gazebo 搜索路径。

```bash
# 同层 3 m，一圈后返航降落
./scripts/run_sh/three_uav/three_uav_inspection.sh --same-layer --altitude 3 --control --gui --rviz --record light

# 同层 30 m，一圈后返航降落
./scripts/run_sh/three_uav/three_uav_inspection.sh --same-layer --altitude 30 --control --gui --rviz --record light

# 异层：UAV1/2/3 = 26/20/14 m，各一圈后返航降落
./scripts/run_sh/three_uav/three_uav_inspection.sh --multi-layer --control --gui --rviz --record light

# 异层下降：第一圈后降至 22/16/10 m，再各巡检一圈后返航降落
./scripts/run_sh/three_uav/three_uav_inspection.sh --multi-layer --layer-descent --control --gui --rviz --record light
```

结束进程用启动终端的 Ctrl+C，正常飞行先等待三机 DONE、落地且解除武装。Ctrl+C 是停止进程，不代表任务成功或受控降落。

| 参数 | 含义及默认值 |
|---|---|
| `--same-layer` | 三机同高度、单圈；默认模式，与 `--multi-layer` 互斥 |
| `--altitude H` | 同层高度，单位 m，默认 3；允许有限数值 3–30。同步任务、bridge、manager、EGO ceiling、返航与恢复高度；起飞超时为 max(60,4H) 秒。新高度仍需实际飞行验证 |
| `--multi-layer` | 三机异层 26/20/14 m；默认仅一圈，不下降 |
| `--layer-descent` | 仅与 `--multi-layer` 合用，开启第二层，各下降 4 m；沿用既有切层状态机 |
| `--control` | 授权控制与任务启动；默认 false。不带此参数仍启动仿真和节点，不是只读检查 |
| `--gui` | 打开 Gazebo GUI；默认 false |
| `--rviz` | 打开三机 RViz；默认 false；与 GUI 独立 |
| `--record none` | 默认；不保存任务 CSV、bag、summary、轨迹图与包装器日志；临时 ROS 日志退出时清理。外部 PX4 自身日志规则不由此开关承诺 |
| `--record light` | 状态、任务、规划轨迹等 100 个唯一话题；保存 CSV/JSON、bag、日志、轨迹图，不含点云/model_states |
| `--record full` | 122 个唯一话题，增加三机注册点云、占据膨胀图等；不等于包含完整 TF、原始雷达或完整 RViz 重建材料 |
| `--check-height-profile` | 仅展开参数并核对三机高度合同，不启动节点/ROS master/Gazebo；light/full 时会创建检查工件目录 |
| `--results-dir DIR` | light/full 的独立输出目录，绝对路径须在本仓库 `runtime_artifacts/three_uav_inspection_*` 下；拒绝已有目录，禁止覆盖历史数据 |
| `--duration SEC` | 正整数墙钟停止上限；默认不限时，不作为成功判据 |
| `--world FILE` | 显式 world；默认 `simulation/astra_gazebo_worlds/worksite.world`，换场景不继承任务验收 |
| `--disable-d435` | 关闭相机仿真插件；默认 D435 开启 |
| `--lidar-downsample N` | 正整数，默认 1；改变雷达负载与感知输入，非默认配置待验证 |
| `--learning-speed` | 同层研究用 mock speed adapter 开关；默认关闭；异层明确禁止。不是加载已训练 SAC 策略 |
| `--profile NAME` | 保留历史 same_3m / same_30m / multi_height_low_3m / multi_height_legacy；不得混用新的模式/高度参数 |
| `--single-layer` | 旧 profile 的单层兼容参数；新命令用上面的两种模式 |
| `--help` | 查看 shell 参数，不启动系统 |

`three_uav_multi_height_inspection.sh` 只保留为历史兼容转发器；其中旧 `--multi-layer` 仍表示下降再巡检（等价于新入口 `--multi-layer --layer-descent`），旧 `--single-layer` 等价于新入口 `--multi-layer`。内部 `--legacy-layer-descent` 用于这层兼容，不是新用户命令。摄像头查看与 outdoor_village 初始化脚本保留为辅助工具，不作为第二套巡检入口。

## 三机代码结构与共享依赖

以下路径相对仓库根目录。主链：Mid360/IMU → FAST-LIO → frame adapter/自机与队友过滤 → EGO-Swarm → traj_server → ego_mavros_bridge → MAVROS/PX4。每机拥有独立定位、地图、规划与控制命名空间；公共 world 共享状态和带绝对时间的轨迹，用于协同避碰。

| 模块 / 文件 | 核心文件及作用 |
|---|---|
| `scripts/run_sh/three_uav/three_uav_inspection.sh` | 唯一用户巡检入口，模式选择、参数透传、录制与退出清理 |
| `AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/launch/triple_tower_height_profile.launch`；同包 `scripts/check_height_profile.py` | 同层高度合同及无节点参数检查 |
| 总 launch | `AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/launch/triple_tower_inspection.launch`：三机总入口、高度/速度/地图和安全参数传递；`AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/launch/uav_tower_stack.launch`：单机任务、规划和 bridge 接线 |
| PX4 启动 | `AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/launch/triple_px4_mavros.launch`；同目录 `dual_px4_mavros.launch`、`single_vehicle_spawn_verified.launch`：模型与 PX4/MAVROS 启动 |
| 传感器模型 | `AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/models/iris_mid360_d435.sdf.xacro`；`AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/launch/uav_sensor_frames.launch`：传感器模型、频率和 TF |
| FAST-LIO | `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/launch/mapping_mid360.launch`；`AstraDrone_ros1_ws/src/SLAM/FAST_LIO/config/mid360.yaml`；`AstraDrone_ros1_ws/src/SLAM/FAST_LIO/src/laserMapping.cpp`：激光惯性定位建图 |
| EGO-Swarm | `AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/src/ego_replan_fsm.cpp`；同级 `planner_manager.cpp`、`traj_server.cpp`；`AstraDrone_ros1_ws/src/Planner/ego-planner/planner/bspline_opt/src/bspline_optimizer.cpp`；`AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_env/src/grid_map.cpp`；`AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/config/swarm_low_altitude_ego.yaml`：共享轨迹、地图、优化和输出 |
| 控制桥 | `AstraDrone_ros1_ws/src/MissionControl/ego_gazebo_bridge/src/ego_mavros_bridge.cpp`；`AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/config/uav1_bridge.yaml`、同目录 `uav2_bridge.yaml`、`uav3_bridge.yaml`：控制状态机与执行限制；最终值服从总 launch 与异层 launch 覆盖 |
| 协调与许可 | `AstraDrone_ros1_ws/src/Swarm/astra_swarm_manager/scripts/swarm_manager_node.py`；同目录 `swarm_state_publisher.py`；`AstraDrone_ros1_ws/src/Swarm/astra_swarm_manager/src/astra_swarm_manager/policy.py`；`AstraDrone_ros1_ws/src/Swarm/astra_swarm_manager/config/three_uav_inspection_formation.yaml`：状态、角色、许可和切层交接 |
| 机间安全 | `AstraDrone_ros1_ws/src/Swarm/astra_swarm_safety/scripts/swarm_safety_node.py`；`AstraDrone_ros1_ws/src/Swarm/astra_swarm_safety/src/astra_swarm_safety/prediction.py`：机间安全检查；`AstraDrone_ros1_ws/src/Swarm/astra_swarm_msgs/msg/SwarmState.msg`、同目录 `PredictedTrajectory.msg`：共享数据合同 |
| 记录与可视化 | `scripts/run_sh/three_uav/three_uav_inspection.sh`；`scripts/run_sh/three_uav/three_uav_multi_height_inspection.sh`；`scripts/tool/plot_three_uav_trajectory.py`；`scripts/tool/analyze_three_uav_orbit.py`；`AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/scripts/swarm_evidence_recorder.py`；`AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/config/triple_tower.rviz`：运行记录、绘图、RViz |
| 坐标与过滤 | `AstraDrone_ros1_ws/src/Swarm/astra_swarm_tf/scripts/frame_adapter_node.py`；`AstraDrone_ros1_ws/src/Swarm/astra_swarm_perception/src/teammate_cloud_filter_node.cpp`、同目录 `self_cloud_filter_node.cpp`：frame 适配、队友/自身点云过滤 |
| 感知扩展 | `AstraDrone_ros1_ws/src/Swarm/astra_swarm_perception/src/world_cloud_fusion_node.cpp`；`AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/launch/dual_tower_inspection.launch`：默认关闭的双机可视化点云拼接；`AstraDrone_ros1_ws/src/Detection/yolo_detect/launch/ppe_yolo_three_uav.launch`：独立 YOLO 扩展 |
| 异层配置 | `AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/launch/triple_tower_multi_height_inspection.launch`：26/20/14 m profile、两层开关、禁止 Learning Speed |
| 任务状态机 | `AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/src/sector_inspection_mission_node.cpp`；`AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/config/low_altitude_inspection.yaml`；同目录 `sector_inspection.yaml`；本文件设计基准：候选、进出场、绕塔、切层和返航语义 |

三机总 launch 继续 include `dual_tower_inspection.launch`，每机复用 `uav_tower_stack.launch` 与 `sector_inspection_mission_node.cpp`。dual、sector、single_vehicle 等命名不代表可删除。地图膨胀显示 `grid_map/inflated_cloud` 只用于局部真实障碍可视化；规划仍使用原占据图与虚拟限高。

ROS frame 契约为公共 `world`、各机 `uavN/map`、`uavN/camera_init`、机体/传感器 frame。仿真单位对齐和静态 TF 不等于真机外参标定。D435/YOLO 是独立感知扩展，不参与当前 Mid360→FAST-LIO→EGO 规划控制主链。

## SAC / Learning Speed 核心文件与作用

RL 仅决策 EGO 最大速度约束，不选择航点、不替代碰撞检查或控制器。观测输入冻结为 3200 个雷达 surrogate、20×3 个未来位置、实际速度 3 维、跟踪误差 3 维与上一 applied v_max，共 3267 维。SAC 动作 [-1,1] 映射为 1.025+0.725a m/s，范围 [0.30,1.75]。

| 模块 / 文件 | 核心文件及作用 |
|---|---|
| 训练后端 | `AstraDrone_ros1_ws/src/MissionControl/hector_ego_training_backend/launch/hector_forest_sac_training.launch`；同目录 `hector_worksite_sac_training.launch`、`hector_training_observation_c.launch`；`AstraDrone_ros1_ws/src/MissionControl/hector_ego_training_backend/config/worksite_training_reset.yaml`：Gazebo training-only 后端与 reset |
| SAC 与环境 | `AstraDrone_ros1_ws/src/learning_speed_rl/config/sac_training_v1.yaml`；`AstraDrone_ros1_ws/src/learning_speed_rl/scripts/sac_training_runner.py`；`AstraDrone_ros1_ws/src/learning_speed_rl/src/learning_speed_rl/training/sac.py`、同目录 `sac_replay.py`、`astra_drone_env.py`：SAC、Replay、Episode 与训练/评估参数 |
| 观测与动作 | `AstraDrone_ros1_ws/src/learning_speed_rl/src/learning_speed_rl/observation/v2/lidar_surrogate.py`；`AstraDrone_ros1_ws/src/learning_speed_rl/src/learning_speed_rl/observation/scheme_c/builder.py`；`AstraDrone_ros1_ws/src/learning_speed_rl/scripts/speed_adapter_node.py`；`AstraDrone_ros1_ws/src/learning_speed_rl/src/learning_speed_rl/policy/safety_filter.py`：Observation、动作适配和范围保护 |
| Reward | `AstraDrone_ros1_ws/src/learning_speed_rl/src/learning_speed_rl/training/reward.py`；`AstraDrone_ros1_ws/src/learning_speed_rl/config/stage1_reward.yaml`：唯一 Reward 实现与当前 v3.1 参数 |
| `AstraDrone_ros1_ws/src/MissionControl/hector_ego_training_backend/scripts/training_episode_reset_coordinator.py` | Episode 身份、terminal 闭合、控制器停启、zero-twist teleport、generation barrier 与五帧预热 |
| `scripts/run_sh/reinforcement_learning/prepare_hector_training_overlay.sh` | 从保留的外部 Hector 源码准备可写 overlay；不是 PX4 替代部署脚本 |
| `AstraDrone_ros1_ws/src/learning_speed_rl/msg/`、`test/` | stamped request/action/applied 身份合同及观测、SAC、Replay、Reward 回归 |

`training/astra_drone_env.py` 每 Episode 最多一个未闭合 transition，按 causal action/ACK/Observation 写有序 Replay，terminal 闭合后才能 reset。`training/sac.py` 管理 Gaussian Actor、双 Q/target Q 与自动熵；`sac_replay.py` 保存经验；runner 管理训练/checkpoint 与独立 evaluation。当前 `stage1_reward.yaml` 为 v3.1，唯一 Reward owner 为 `LearningSpeedReward.evaluate()`；旧 UNKNOWN 固定权重已不代表当前实现。

主配置计划 10000 completed Episodes、每 500 Episodes checkpoint、Replay 100000、learning_starts=1000；这些是配置，不是已完成规模。已有 100 Episode endurance 与 87 Episode rollback 记录，但总体仍 NO-GO；不得描述为收敛策略、正式训练放行或三机部署，不恢复旧失败 run。后续正式训练与独立 evaluation 仍需专门资格验证。

三机与 RL 共用 EGO 地图/规划/B 样条、Gazebo world/model、Mid360 插件、ROS 消息和速度安全过滤。full-stack RL 共用 FAST-LIO→MAVROS/PX4 执行；training-only 则使用明确标记 `gazebo_truth_training` 的 truth odometry 与 Hector 控制，不启动 FAST-LIO/PX4/MAVROS/bridge。两种 `/uav1/Odometry` 与控制发布者必须互斥。Hector 控制器是独立依赖，不可因三机不用而删。

## 保留的实验数据及用途

全部现有 `runtime_artifacts/` 与外部 PX4 原始日志均保留，不迁移、不删减。以下为验证报告实际使用的代表数据；目录内 bag、CSV、JSON、日志、元数据、轨迹图和原始文件一并保留。历史报告的完整原文可从整理前提交 `9b5b2049899e421b2c728e2c7547615ddc0501f7` 查询。

| 编号 | 实际工件目录 | 可直接引用的材料与证据 |
|---|---|---|
| E1 | `runtime_artifacts/three_uav_inspection_control_20260911_183305/` | 低空同层完整任务；`summary.json`、`swarm.csv`、`uav1.csv/uav2.csv/uav3.csv`、`run_metadata.txt`、`roslaunch.log`、`candidates.jsonl`、`trajectory_xy.png`、`trajectory_3d.png`；bag 文件 64900482 字节 |
| E2 | `runtime_artifacts/three_uav_inspection_control_20260906_113711/` | 异层两层完整任务；同类工件齐备；`roslaunch.log` 保存 26/20/14 m、`[0,-4]`、0.60 m/s 和 0.50 m/s² 参数；bag 文件 102665686 字节 |
| E3 | `runtime_artifacts/three_uav_inspection_control_20260906_132714/` | 另一组低空同层三机 DONE、disarmed、`0xFF`；全记录最小三维距离 3.891 m；可作为多轮定性运行的辅助证据 |
| E4 | `runtime_artifacts/rl_training/ego_speed_async_100ep_endurance_20260901_114200_r01/` | `sac_runtime_summary.json`、`qualification_summary.json`；100 Episode 数据闭合/早期学习记录，总体 NO-GO |
| E5 | `runtime_artifacts/rl_training/post_lifecycle_rollback_100ep_baseline_20260901_202221_r03/` | 相同两个 summary；87 completed Episode，Replay 26826，总体中断/NO-GO；勿将第 88 个计为完成 |

E1/E2 的 bag 文件原验证报告确认存在及大小，未播放或重新解析全量 bag；原报告数字来自 JSON/CSV 和日志。`light` 录制不包含点云，也不包含 Gazebo model states，不能承诺仅靠这些 bag 重建完整点云 RViz 场景。`full` 虽增加点云等话题，仍需检查 TF 与所需话题后再判断可否完整回放。当前 `record none` 本就不保留任务证据，因而没有逐次工件不代表实验没有进行。（见本文件录制参数）

| 量化项目 | E1：低空同层 | E2：异层两层 | 来源/口径 |
|---|---|---|---|
| 最终任务/bridge/flight | 三机均 DONE | 三机均 DONE | `summary.json` 三组 final 字段 |
| 最终解锁/落地 | 三机 armed=false、landed_state=1 | 同左 | final_armed、final_extended_landed_state；1 对应 ON_GROUND |
| 扇区覆盖 | 三机均 `0xFF` | 三机均 `0xFF` | visited_sector_mask；多层需同时核对层过程，单个 mask 不独立证明两层 |
| 全记录最小三维机间距 | 3.840 m | 3.842 m | `pairwise_minimum.*.three_d` 的最小值；状态采样距离，不是连续时间真值保证 |
| 并发绕塔窗口最小三维机间距 | 10.079 m | 10.643 m | `concurrent_orbit_pairwise_minimum.*.three_d` 的最小值，与全记录窗口不同 |
| 最大跟踪误差 UAV1/2/3 | 0.503 / 0.518 / 0.228 m | 0.386 / 0.405 / 0.420 m | `maximum_tracking_error`；bridge 中 MAVROS 位置与规划目标位置的距离，不是定位 ATE，也不是 RMSE |
| 起飞到首次 DONE 时长 UAV1/2/3 | 421.400 / 413.300 / 387.299 s | 789.300 / 729.499 / 712.300 s | 原报告只读计算：`swarm.csv` 首个 uN_phase=DONE 的 sim_time 减 summary.takeoff_times[N]；仿真时间 |
| 记录时间跨度/CSV 行数 | 549.900 s / 5500 行 | 812.900 s / 8130 行 | CSV 最后一行减第一行 sim_time；含任务前后，不等于飞行时长 |
| ERROR/FAILURE_LANDING 进入次数 | 0 / 0 / 0 | 0 / 0 / 0 | `emergency_count`，recorder 按进入任务错误/失败降落状态计数；不能直接命名为 Gazebo 碰撞次数 |
| 规划轨迹 ID 变化次数 UAV1/2/3 | 259 / 280 / 295 | 585 / 555 / 555 | `planner_trajectory_changes`；不是规划失败次数，也不等于静态障碍避让次数 |

任务成功次数不从以上代表性记录外推；比赛报告可展示这些完整实例并使用定性多轮结论。轨迹变化、候选拒绝计数会多次观察同一事件，不能机械相加作为“避障次数”或“规划失败次数”。三机物理接触碰撞总数、全场景定位真值误差、规划耗时均值、带宽等没有在本文建立统一统计，若报告需要这些数字则**待人工补充**。

SAC 已有真实统计可单独列示，但应放在创新模块验证边界中，不混入三机巡检成果：

| 项目 | E4 | E5 | 来源/解释 |
|---|---|---|---|
| completed Episodes | 100 | 87 | 两份 summary 的 episode_count；闭合不是飞行成功 |
| success / failure / truncated | 1 / 91 / 8 | 1 / 76 / 10 | qualification_summary；互斥终态分类 |
| planner_failure_count / collision_count | 59 / 27 | 55 / 20 | qualification_summary 的原始字段；collision 为训练终止/代理口径，不能改称 Gazebo 接触事件，也不要与 failure 重复相加 |
| global_environment_step / Replay | 29543 / 29543 | 26826 / 26826 | sac_runtime_summary 与 replay_audit；E5 含未闭合 Episode 88 的非终止数据 |
| learner/gradient 更新 | 24007 | 22490 | E4 learner_update_count；E5 gradient_update_step |
| 实际速度 median / p95 / max | 0.905 / 1.435 / 2.662 m/s | 0.858 / 1.408 / 2.961 m/s | qualification_summary.actual_speed；分别 48159 / 44289 个监测样本，不是 Replay 条数或成功 Episode 专用窗口 |
| 跟踪误差 median / p95 / max | 0.036 / 0.211 / 2.779 m | 0.038 / 0.229 / 9.754 m | qualification_summary.tracking_error；保留监测窗口异常峰值，不据此宣称训练全程稳定 |
| 最终结论 | NO-GO，final audit invalid_observation | NO-GO，中断于活动 Episode | sac_runtime_summary.status/failure/verdict |

这些数据证明 SAC 确实执行过在线交互与更新，同时也清楚表明当前不能宣称学得了稳定有效的最终策略。避免仅引用 Replay 数量而省略总体结论。实际速度监测峰值与动态最大速度约束是不同量，不能将约束范围解释为实际速度全程严格上界。监测统计的采集实现见 `AstraDrone_ros1_ws/src/MissionControl/hector_ego_training_backend/scripts/training_episode_reset_coordinator.py`。


历史三机 PASS 不等于这次入口修改后的新飞行 PASS；30 m 命名目录中的未完成记录不能作为 30 m 成功证据。完整 RViz 回放、比赛视频与截图关联仍需人工选择已有材料。所有失败和中断记录保留，不用成功样例覆盖。

## 整理与验证记录

本次仅整理入口、参数合同和项目文档，未修改已验证的任务状态机、EGO 算法、安全阈值、世界或 RL 实现。构建/检查记录保存在 `runtime_artifacts/submission_cleanup_20260919/`。不把参数展开或编译视为飞行验收。

- 两个工作空间 catkin 编译 PASS；主工作空间保留全包配置，包含 RL 消息与后端。已有 PCL deprecated/strncpy 编译警告保留，未改第三方实现。
- 最终入口/launch 回归 16 项 PASS（包含 3 m、12 m、30 m、异层单圈/下降、旧入口兼容、GUI/RViz/三种录制、非法参数与已有目录保护）；3 m/30 m 新旧完整参数字典一致。
- RL 本轮 16 组 nosetests、170 项 PASS；不将结果目录中旧 integration XML 额外计入本轮。Hector overlay `--check-only` PASS，PyTorch 2.4.1+cpu 导入 PASS。
- shell 语法与 `git diff --check` PASS；录制 topic 数量 light=100/full=122，录制与停止处理代码保持原样。
- 原有 30,577 个 runtime 文件路径/大小/mtime 不变；无非 Markdown 文件删除，RL 实现和任务核心文件 SHA256 不变。三组报告用三机目录另有逐文件 SHA256 清单 `retained_three_uav_files.json`。
- 删除 42 份项目说明性 Markdown，保留第三方来源/许可证与运行原始工件；设计规则并入下节。
- 本轮未启动 Gazebo、ROS 节点、飞行或训练，实际飞行回归待人工验证。

删除范围为根目录旧项目报告/学习说明、docs 教程、essay 笔记、项目自有包说明与脚本 README；其当前必要内容合并于本文件。第三方组件自带 README、许可证、API/构建材料保留，用于来源、再构建与授权说明，不作为第二份项目说明；不对 third_party 或 vendor 源码批量删 md。删除文件清单和原 SHA256 记录在本次整理工件中，并可通过 Git 查阅历史。

## 维护规则

开始任务先检查 Git 分支、HEAD、status 与相关 diff，保护已有用户修改，特别是 FAST_LIO/Log/mat_pre.txt。源码/配置决定当前行为，运行原始数据决定验证范围。只维护本项目说明，不再另建平行项目 runbook。

不得按文件名、没有文本引用或“无运行依赖”推断可删除；特别保留 dual launch、sector mission、单机基础链、RL/Hector、EGO、FAST-LIO、PX4/MAVROS、模型和实验数据。runtime_artifacts 不加入 Git；比赛交付必须另外携带此节列出的数据与必要外部依赖，单独 Git clone 不包含它们。

不编辑生成 build/devel 文件，不升级外部 PX4 或锁定依赖，不降低 3 m 机间安全门、1.5 m swarm_clearance 或现有净空规则，不启动未经授权的 Gazebo/训练。默认不提交或 push；本轮用户已要求提供 commit hash，允许提交本轮变更，禁止 push。真实失败保留，静态/编译/飞行证据分开表述。

<a id="mission-design"></a>
## 巡检设计基准（合并原 rule.md）

以下保留既有任务语义，不随入口整理修改。原 rule.md 的设计基准身份迁入此节；程序不读取 Markdown。调整语义时须同步此节、YAML、源码与测试。

### AstraDrone 统一扇区、航点与净空规则

版本：1.0
适用范围：ROS1 Noetic、PX4 SITL/真机适配层、FAST-LIO、EGO-Planner、EGO-Swarm 任务层
性质：设计规范，不是运行时输入文件

#### 1. 文档与参数职责

本文件“巡检设计基准”节 是扇区划分、航点生成、重定位和净空语义的唯一设计基准。程序不得在代码中单独增加与本文件冲突的扇区或航点规则。

- 实际运行参数放在 YAML、launch 或参数服务器中；YAML 是运行参数来源。
- 程序不得解析 Markdown；本文件“巡检设计基准”节 不作为运行时配置读取。
- 修改规则时必须同步更新 本文件“巡检设计基准”节、相关 YAML、实现代码、单元测试和集成报告。
- 只改变某次仿真的数值时，必须确认没有改变本文件定义的语义。
- `build/`、`devel/`、日志和生成文件不是规则来源。

##### 1.1 飞行试验数据与运行产物

所有飞行试验产生的 rosbag、ROS 日志、CSV、JSON、轨迹图、截图、视频、临时报告、调试输出和验证结果，必须写入仓库根目录的 `runtime_artifacts/`。推荐每次试验使用独立的时间戳目录，例如 `runtime_artifacts/<任务名称>_<时间戳>/`。

- 禁止将运行数据写入源码、配置、world、launch、文档或其他项目目录。
- 禁止重新创建、使用或向 `test_evidence/` 与 `trc_picture/` 写入任何数据。
- `runtime_artifacts/` 中的文件不得加入 Git 版本管理。
- 只有项目负责人明确要求长期归档时，才允许从 `runtime_artifacts/` 挑选少量最终结果整理到正式文档目录；归档不得重新使用 `test_evidence/` 或 `trc_picture/`。

##### 1.2 文件、目录与运行结果命名

禁止新建或继续使用带有 `stageX`、`stage1`、`stage2`、`stage3`、`stage4`、`stage5` 等开发阶段标识的文件名、脚本名、目录名或运行结果目录名。名称必须描述实际功能，例如 `three_uav_inspection`、`sector_inspection`、`trajectory_analysis`。

- 运行目录必须采用 `runtime_artifacts/<任务名称>_<时间戳>/` 格式。
- 所有新建文件和目录都必须使用清晰、稳定的功能名称，不得以开发阶段代替功能含义。
- `runtime_artifacts/` 始终不加入 Git 版本管理。
- 历史报告中为准确说明过去阶段而保留的阶段文字不构成新命名，也不得用作新的输出路径或活动文件名。

#### 2. 坐标、塔心与角度

任务几何使用规划坐标系中的 `map` 绝对位置；FAST-LIO/EGO 适配层负责把输入转换到任务规划 frame，不能在任务节点中假设 Gazebo API。

- 塔心：`C=(tower.center.x, tower.center.y)`，当前 worksite 默认 `(-10.0551, 19.7104)`。
- 角度零点：从塔心指向 `+X` 为 `0°`。
- 角度正方向：逆时针增加；顺时针任务使用负方向推进。
- 角度统一归一化到 `[0°,360°)`，跨越 `0°` 时使用最短有向角差。
- 航点极坐标：`x=Cx+r*cos(theta)`，`y=Cy+r*sin(theta)`。
- 任务航点的 yaw 默认朝向塔心；入口、出口和返航段可按任务状态使用速度前向 yaw。

#### 3. 八个扇区

任务固定八个扇区，每个中心线相隔 `45°`：

| 扇区 | 中心线 | 原则角范围 |
|---:|---:|---:|
| 1 | `0°` | `[-22.5°,22.5°)` |
| 2 | `45°` | `[22.5°,67.5°)` |
| 3 | `90°` | `[67.5°,112.5°)` |
| 4 | `135°` | `[112.5°,157.5°)` |
| 5 | `180°` | `[157.5°,202.5°)` |
| 6 | `225°` | `[202.5°,247.5°)` |
| 7 | `270°` | `[247.5°,292.5°)` |
| 8 | `315°` | `[292.5°,337.5°)` |

边界归属必须保持半开区间和统一角度归一化，不得因入口角或候选偏移临时改变扇区编号。任务可以从任意入口角开始，但覆盖、进度和闭环判定仍按这八个固定扇区计算。

#### 4. 名义巡塔、PRE_ENTRY 与 ENTRY_GATE

当前三机低空任务的名义巡塔参数：

- 名义巡塔半径：`12.5 m`；
- 三机巡塔高度：`3.0 m`；
- `PRE_ENTRY`：从悬停/进场位置进入任务通道前的外围参考点，三机默认半径 `18.0 m`，允许在配置范围内向外搜索；
- `ENTRY_GATE`：进入第一巡塔点前的正式任务入口，三机默认半径 `15.0 m`，允许在配置范围内径向搜索；
- `ORBIT_STAGING`：第一巡塔点，必须来自普通扇区候选网格，不得固定为唯一 `12.5 m` 点；
- ENTRY/PRE_ENTRY 仅负责安全进场和联合放行，不能替代正式八扇区巡塔目标。

动态 PRE_ENTRY/ENTRY_GATE 可以使用角色名义角附近的连续角度，但 ORBIT_STAGING 必须使用同一套普通扇区候选生成、筛选、评分和 EGO 重试状态机。

#### 5. 单机和多机共用的候选网格

普通巡塔扇区和第一巡塔点使用同一个候选生成器。原稳定高空候选规则为：

```text
angle_offsets_deg  = [0, -5, +5, -10, +10, -12, +12]
radius_offsets_m   = [0, +2, +4]
height_offsets_m   = [0, -1, -2, -3]
nominal_radius_m   = 12.5
```

候选半径为 `12.5、14.5、16.5 m`，角度必须留在当前扇区边界内。

当前三机任务为低空任务，强制使用：

```text
height_offsets_m = [0]
inspection_height = 3.0 m
```

低空任务不得通过候选重定位下降到 `2 m、1 m 或 0 m`。高度偏移 `[0,-1,-2,-3] m` 只属于通用高空规则，只有高空任务确认最低安全高度、地图和 EGO 约束后才能启用。

候选排序不能因为候选数组的枚举顺序而“找到第一个就使用”。当前低空任务的选择顺序必须明确为：

```text
先筛掉硬约束失败者
→ 半径按 12.5 m → 14.5 m → 16.5 m 分层检查
→ 只使用第一个仍存在安全候选的半径层
→ 若该层已有仍安全的已锁定候选，保持锁定，避免地图抖动跳点
→ 否则，若该层扇区中心角的名义点安全，直接选择该名义点
→ 否则，仅在该半径层剩余安全候选中按评分选择最高者
```

因此，若 `12.5 m + 扇区中心角 + 当前任务高度` 的名义点满足全部硬约束，任务必须直接选择它；即使外圈候选的评分更高，也不得替代它。名义点的“直接选择”不绕过地图占据、净空、塔体 keep-out、扇区边界、方向进度和已确认不可达等检查。

#### 6. 净空语义

必须首先判断输入地图是否已经膨胀：

| 输入表示 | 使用的任务层净空 | 说明 |
|---|---:|---|
| 已膨胀 occupied map | `map_additional_clearance=0.5 m` | EGO 已包含车辆包络，任务层只加操作余量 |
| 未膨胀原始/过滤点云 | `minimum_clearance=1.0 m`，且只应用一次 | 不得误认为已膨胀地图，也不得再叠加任务层 inflation |
| 已知粗几何 | `minimum_clearance=1.0 m`，且只应用一次 | 端点 keep-out 是硬条件；走廊风险按策略处理 |

其他约束：

- 塔体 keep-out 是硬约束；当前塔体碰撞半径 `6.41 m`、任务安全距离 `2.0 m`，塔心最小半径为 `8.41 m`。
- EGO `obstacles_inflation` 是地图/轨迹碰撞包络，不等于任务层附加净空。
- EGO `optimization/dist0` 是碰撞代价的软优化目标，不能当成硬净空。
- `swarm_clearance=1.5 m` 是 EGO 椭球机间约束，不能当作障碍物净空。
- 三机同步预测必须继续满足最小 3D 机间距离 `3.0 m` 和 EGO 椭球距离 `2*swarm_clearance=3.0 m`。

#### 7. 检查职责边界

任务层负责：

1. 候选端点是否在扇区边界和高度包络内；
2. 塔体 keep-out 和已知粗几何端点硬约束；
3. 根据地图表示选择 `0.5 m` 或 `1.0 m` 的正确任务层净空；
4. 候选评分、锁存、重试计数和任务状态机。

EGO/FAST-LIO 负责：

1. FAST-LIO 提供带正确 frame 和时戳的里程计/点云；
2. EGO 对局部占据图生成局部轨迹和重规划；
3. EGO 对膨胀地图执行轨迹级碰撞检查；
4. EGO `dist0` 作为优化代价，不向任务层伪装成硬安全保证。

三机管理器负责：

1. UAV3 leader → UAV2 middle → UAV1 trailing 的角色顺序；
2. ENTRY 候选的有向角间隔、完整路径不交叉和同步预测冲突检查；
3. `minimum_3d_separation=3.0 m` 和 `swarm_clearance=1.5 m` 约束；
4. 只有联合候选安全时发布选择，不修改单机候选的障碍语义。

#### 8. 候选评分与锁存

候选评分只用于“第一个仍有安全候选的半径层”中、且没有可保持的旧锁定目标和安全中心角名义点时。评分优先考虑：

1. 净空更大；
2. 偏离名义角度、名义半径和任务高度更少；
3. 路径更短、与上一航点更连续；
4. 走廊风险和未知区域风险更低。

评分不能跨越半径层：外圈候选即使评分更高，也不能取代内圈的安全候选。它同样不能推翻仍安全的锁定目标，除非锁定目标重新验证失败、EGO 不可达达到上限或联合安全条件失效。

通用评分应显式记录：候选 ID、扇区、角度、半径、高度、净空、走廊风险、当前距离和连续性代价。锁定后不能因普通地图帧抖动逐帧跳点；只有候选重新验证失败、EGO 不可达达到上限或联合安全条件失效时才能清锁并产生新 generation。

第一巡塔点和普通扇区的 `CandidatePoint` ID 必须统一进入任务日志，使 `PLANNER_UNREACHABLE` 能准确排除一个候选而不是排除整个扇区。

#### 9. EGO 失败、HOLD、重试与换候选

统一处理顺序：

```text
生成同扇区角度/半径候选
→ 正确检查端点硬约束
→ 检查塔体 keep-out 和粗几何
→ 计算地图/路径走廊风险
→ 对可用候选评分并锁存
→ 交给 EGO
→ EGO 不可达后 HOLD
→ 同一候选有限重试
→ 达到上限后标记 PLANNER_UNREACHABLE
→ 切换同扇区下一个候选
→ 候选全部耗尽才 fail-closed
```

默认参数：

- `planner_unreachable_attempt_limit=2`；
- `failure_hold_duration=2.0 s`；
- 地图或规划状态陈旧时保持 fail-closed；
- 候选耗尽不得标记为“扇区已完成”；低空任务进入安全返航/降落路径。

三机 ENTRY 的联合选择若因某机候选失败，必须产生新的候选 generation 并重新做联合选择；不得在联合选择之外偷偷切换单机目标而绕过机间路径检查。

#### 10. 第一巡塔点统一规则

第一巡塔点不能再使用单独的“固定 `12.5 m`、occupied map 小于 `1.0 m` 全部拒绝”逻辑。

它必须：

- 使用与普通扇区相同的角度、半径和高度候选网格；
- 对已膨胀 map 使用 `0.5 m` 附加净空；
- 通过普通候选端点、塔体和粗几何检查后再评分；
- 交给 EGO 验证可达性；
- EGO 失败后在同一扇区尝试下一个 candidate ID；
- 允许 `14.5 m` 或 `16.5 m` 临时第一巡塔半径；
- 在正式绕塔开始后，在安全且 EGO 可达的前提下平滑回归 `12.5 m` 名义圆；
- 不得突然从外圈径向切入名义圆，也不得将临时候选当成整圈固定半径。

#### 11. 三机角色与编队

当前三机必须保持：

- 所有无人机高度 `3.0 m`；
- 逆时针绕塔；
- UAV3 leader，UAV2 middle，UAV1 trailing；
- 名义角：UAV3 `337.5°`、UAV2 `315°`、UAV1 `292.5°`；
- UAV2 以第 8 扇区中心线作为其入口基准；
- 动态 PRE_ENTRY 与 ENTRY_GATE；
- 三机第一巡塔点联合选择；
- 实际集结相邻角度默认 `20°～30°`；
- 三条进场路径不交叉；
- 同步预测轨迹冲突检查；
- UAV3 先绕塔；
- `phase_3_2` 进入 `65°～70°` 后放行 UAV2；
- `phase_2_1` 进入 `65°～70°` 后放行 UAV1；
- 正式目标相位差为 `67.5°—67.5°`；
- 各机独立完成一圈、EXIT、返航和降落。

联合选择只能约束和筛选候选，不得将单机候选规则改成固定半径或错误的地图净空语义。

#### 12. ENTRY、EXIT 与返航

ENTRY 应按 `home/hover → PRE_ENTRY → ENTRY_GATE → ORBIT_STAGING` 的顺序规划。动态路径可以使用局部 EGO 重规划，但任务层必须记录完整路径和每个锚点。

正式绕塔后：

- 先完成联合放行和第一巡塔点到达；
- 后续扇区按逆时针和固定扇区覆盖顺序执行；
- 若使用外圈临时候选，后续航点应在安全、平滑和 EGO 可达的条件下逐步回归名义半径；
- EXIT 使用已验证的出口/逆进场走廊；
- 返航路径必须继续检查地图、走廊、定位、轨迹时效和 SAFETY_INHIBIT；
- 任一安全条件失败时优先 HOLD、返航或受控降落，不得包装为任务成功。

#### 13. 低空与高空差异

低空三机当前规则：

- 高度固定 `3.0 m`；
- `height_offsets=[0]`；
- 已膨胀 occupied map 附加净空 `0.5 m`；
- 不得通过扇区重定位下降到 `2/1/0 m`；
- EGO 局部避障和任务层固定高度走廊共同负责可达性。

高空单机/后续高空任务：

- 可以在批准的最低安全高度内启用 `[0,-1,-2,-3] m` 临时下降候选；
- 必须重新核对塔体、树冠、地图垂直膨胀和 EGO 轨迹约束；
- 高度候选不能被低空配置或三机入口参数隐式启用。

#### 14. 修改与验收要求

任何扇区、候选、净空、ENTRY/EXIT 或重定位规则变更，必须同时完成：

1. 更新本 本文件“巡检设计基准”节；
2. 更新对应 YAML/launch；
3. 更新代码和单元测试；
4. 运行相关包构建和自动回归；
5. 记录候选生成数、候选切换、EGO 重试、实际角度/半径/高度、净空和最终状态；
6. 更新阶段集成报告；
7. 若真实三机闭环未完成，明确记录失败阶段、失败候选和安全降落结果，不能将安全终止称为任务成功。

禁止通过以下方式规避规则：

- 在代码中另写第一巡塔点固定半径规则；
- 对已经膨胀的 occupied map 统一套用原始点云 `1.0 m` 门限；
- 降低 `swarm_clearance` 或三机最小 3D 距离；
- 缩小障碍膨胀包络；
- 关闭定位、心跳、轨迹时效或 `SAFETY_INHIBIT`；
- 修改 `worksite.world` 来迎合候选；
- 将 `dist0` 当作硬净空；
- 在没有新联合选择的情况下绕过三机管理器单独换点。


#### 15. 高空异步多层任务补充

本节仅适用于显式启用多层协调的高空任务；第 11 节的同高度基线配置与 single-layer 协调保持原样。

- `formation_role_order` 定义角色链，当前为 `[3,2,1]`。任意时刻最多一个 sticky transition owner；安全或 freshness 失效撤销许可，不把未完成 owner 转给其他机。
- 首角色完成自己的当前层即可申请切层；其余角色还须等待直接 predecessor 完成目标层 transition。取消全机 orbit-complete 与全机 transition-complete barrier。
- transition complete 必须有 owner 实际进入 `LAYER_TRANSITION` 的证据，随后到达目标高度、速度稳定并进入 `ORBIT_STAGING_READY`。每机独立保存层号，只重建该机的新层 release。
- 首角色到达新层后即可放行；followers 优先使用同层直接 predecessor 的正常运动 phase release（65°～70°）。若 follower 完成 transition 时 predecessor 已安全越过窗口，则目标层允许在有界 `<180°` 角色顺序内立即接续；freshness、前向速度、当前/预测 3D 与椭球净空、global safety 仍全部为硬门。不同层之间不做编队相位/软速度跟随；全局机间安全继续生效。
- 最终层闭环后，若直接 follower 尚未获得该层 release，predecessor 继续执行当前层经候选 admission 的巡航，不进入 EXIT。handoff 携带 follower 已获 release 的层号及接收 freshness，旧层确认不能满足新层条件。没有 follower 或 follower 已获该层 release 才可结束；异常 completed-predecessor fallback 保留。
- `candidate/high_altitude_policy=true` 在高空 YAML 显式启用：普通扇区使用固定 `0/45/.../315°` 网格；第一点、普通点和下一层第一点复用已有候选筛选与半径分层选择。ENTRY 角色参考角、generation 协议、HOME_OVERHEAD 与 LAND serialization 不变。
- 高空普通候选（包括第一巡塔点）复用真实成功的单机高空语义：塔体径向 keep-out 与 fresh inflated occupancy 是硬门，`tower_crane` 整体 coarse OBB 只保留为端点/走廊 soft risk，不得用它封死整圈候选。live-map 直线走廊仍是 EGO 绕行风险提示，不能将直线阻塞等同于所有路径不可达。
- 高空现有 `minimum_clearance=2.0 m` 比通用规则的 1.0 m 更严格，本轮不降低；已膨胀地图附加净空仍为 0.5 m。三机候选高度仍为 `[0]`，层偏移仍为 `[0,-4]`。
- 三机高空默认高度为 Layer 0 `UAV1/UAV2/UAV3 = 26/20/14 m`，Layer 1 在统一 `[0,-4]` 偏移下为 `22/16/10 m`。高度变化不改变候选网格、EGO、安全距离、transition 顺序或 predecessor exit guard。
