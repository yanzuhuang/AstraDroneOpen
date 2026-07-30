# EGO-Planner 单机 SITL 回归报告

日期：2026-07-30  
分支：`ego-swarm`  
接入前基线 commit：`64bd99cc21feb7f90b26335ace40112557aa6a82`  
世界：`simulation/astra_gazebo_worlds/worksite.world`  
仿真链：PX4 SITL、Gazebo Classic、MAVROS、FAST-LIO MID360、EGO-Planner、
`ego_gazebo_bridge`、`stage3_ego_mission_node`

## 结论

阶段 1 通过。当前接口一致性修复没有破坏单机 EGO-Planner 链路。dry-run、
近距离 fresh-goal/cancel/restart 验证和 26 m→22 m 两层、8 扇区受控巡塔均
完成；未观察到跨 namespace/重复控制 publisher、旧轨迹在 cancel 后继续输出、
时间戳门禁失效或固定重规划次数。

## 构建与测试

使用的白名单包为：

```text
astra_custom_msgs;bspline_opt;path_searching;plan_env;traj_utils;ego_planner;
waypoint_generator;ego_gazebo_bridge;astra_tower_mission;offboard
```

执行：

```bash
source /opt/ros/noetic/setup.bash
source simulation/sim_workspace/devel/setup.bash
cd AstraDrone_ros1_ws
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_WHITELIST_PACKAGES='astra_custom_msgs;bspline_opt;path_searching;plan_env;traj_utils;ego_planner;waypoint_generator;ego_gazebo_bridge;astra_tower_mission;offboard' -j2
catkin_make -DCATKIN_WHITELIST_PACKAGES='astra_custom_msgs;bspline_opt;path_searching;plan_env;traj_utils;ego_planner;waypoint_generator;ego_gazebo_bridge;astra_tower_mission;offboard' run_tests -j2
catkin_test_results build/test_results
```

结果：`251 tests, 0 errors, 0 failures, 0 skipped`。

## dry-run 和接口证据

入口：

```bash
scripts/run_sh/stage3_ego.sh --report /tmp/astra_stage1_baseline_dryrun.csv
```

实测 topic/frame：

| 接口 | 结果 |
|---|---|
| `/Odometry` | `nav_msgs/Odometry`，frame=`camera_init`，新鲜时间戳 |
| `/cloud_registered` | `sensor_msgs/PointCloud2`，frame=`camera_init` |
| `/stage3/cloud_registered_filtered` | frame=`camera_init`，由 ground filter 唯一发布 |
| `/planning/goal` | bridge 接收 `map` 目标并转发为 `camera_init` |
| `/waypoint_generator/waypoints` | frame=`camera_init`，保留目标时间戳 |
| `/planning/bspline` | `traj_id=1`，frame 由规划链一致传递 |
| `/planner/status` | frame=`camera_init`，状态 `EXEC_TRAJ`、`last_plan_success=true` |
| `/planning/pos_cmd` | dry-run 下由 traj_server 以约 100 Hz 连续输出；bridge 不发布 MAVROS 控制 |
| `map -> camera_init` | 仿真假设下实测单位变换 |

dry-run 中实际发布者检查：

- `/mavros/setpoint_raw/local`：无发布者；
- `/mavros/setpoint_position/local`、velocity、attitude、thrust：无发布者；
- `/planning/pos_cmd`：仅 `/traj_server`；
- `/planning/bspline`：仅 `/ego_planner_node`；
- `/planning/goal`：仅 `/ego_mavros_bridge`；
- `/planner/status`：仅 `/ego_planner_node`。

### fresh goal、cancel、restart

使用 `rospy.Time.now()` 生成非零目标时间戳，向 `/move_base_simple/goal` 发布
近距离目标：

1. `(3.0, 0.0, 1.0)`：bridge 转为 `camera_init`，Path 和 goal 时间戳一致，
   `traj_id=1`，B-spline 的末端接近目标；
2. 发布 `/planning/cancel` 后 FSM 为 `WAIT_TARGET`，traj_server 停止
   `/planning/pos_cmd`；3 秒监听无旧 PositionCommand；
3. 新鲜目标 `(2.0, 1.0, 1.5)` 恢复执行，`traj_id=2`，状态重新为
   `EXEC_TRAJ`、`last_plan_success=true`；
4. 零时间戳目标被 bridge 拒绝，符合 fail-closed 时间戳契约。

dry-run 的首次 command 在相对高度低于 0.3 m 时被 bridge 高度包络拒绝，车辆
进入有效高度后 preflight 报告为 healthy；没有因此创建第二个控制出口。

## 26 m→22 m 受控基线

附件要求的“此前稳定 26 m→22 m”使用 `/tmp/astra_stage1_26_22.yaml` 运行副本：
只把当前 YAML 的 `mission/inspection_top_height` 从 34.0 改为 26.0，保留
`layer_offsets=[0,-4]`、ENTRY_GATE、EXIT_GATE、8 扇区、返航和降落逻辑；
仓库默认配置未修改。

入口：

```bash
scripts/run_sh/stage3_ego.sh --control --report /tmp/astra_stage1_26_22_control.csv
```

任务节点在控制模式中仍不创建 MAVROS publisher；唯一控制出口为
`/ego_mavros_bridge -> /mavros/setpoint_raw/local`。

### 结果

证据 CSV：`/tmp/astra_stage1_26_22_control.csv`  
最小 topic bag：`/tmp/astra_stage1_26_22_minimal.bag`（804 MB，未纳入 Git）

| 指标 | 结果 |
|---|---:|
| CSV 采样行 | 5106 |
| 任务仿真时长 | 1039.2 s |
| 目标发布数 | 25 |
| 规划/执行轨迹 ID | 正常滚动到 `25`，非固定重规划上限 |
| 最大实际高度 | 26.1421 m |
| 最大三维参考误差 | 4.0592 m（起飞/返航垂直切换） |
| 95% 采样参考误差 | 0.1794 m |
| 塔中心最小距离 | 11.2995 m |
| 塔 collision_radius=6.41 m 的几何最小净空 | 4.8895 m |
| `failures` 最大值 | 0 |
| `recovery_count` 最大值 | 0 |
| `lap_planning_failures` 最大值 | 0 |
| `lap_relocations` 最大值 | 0 |
| HOLD / emergency | 0 / 0 |
| 返航和降落 | `DONE`，`AUTO.LAND`，`armed=false`，`landed_state=ON_GROUND` |

任务状态序列包含：

```text
STAGING_POINT -> SEGMENTED_CLIMB -> ENTRY_GATE_TRANSIT
-> NAVIGATING (8 sectors, top layer)
-> LAYER_TRANSITION -> NAVIGATING (8 sectors, lower layer)
-> GO_TO_EXIT_GATE -> RETURN_HOME -> DONE
```

bag 中的关键 topic 均有数据：`/Odometry`、`/mavros/local_position/pose`、
`/mavros/state`、`/mavros/extended_state`、`/mavros/setpoint_raw/local`、
`/planning/goal`、`/planning/bspline`、`/planning/pos_cmd`、
`/planner/status`、`/tower_mission/*`、`/tf`。控制图中没有
`setpoint_position`、velocity、attitude 或 thrust 的竞争 publisher。

## 阶段门结论和遗留项

- 单机定位、规划、控制、cancel/restart 和受控降落满足阶段 1 门槛。
- 4 m 起飞/悬停阶段的低于最小相对高度 command 拒绝是已配置安全包络行为，
  不是规划失败。
- 本报告的净空是从任务 CSV 与 `worksite.world` 塔 collision radius 计算的
  几何结果，不等同于阶段 3 专门静态障碍最小净空验收。
- 本阶段没有修改 EGO vendor 核心、PX4、FAST-LIO 或 worksite world。
- `/tmp` bag、CSV 和临时 26→22 YAML 不属于仓库提交。

