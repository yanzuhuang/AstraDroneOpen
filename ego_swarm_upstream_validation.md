# EGO-Swarm 官方参考与当前工程迁移记录

日期：2026-07-30  
当前分支：`ego-swarm`

## 参考仓库审计

官方参考目录为 `/home/yanzu/ego-planner-swarm_reference`，分支为
`master`，HEAD 为 `92fe9f7227b2da819133eb8e0e8c7fc000f6ae20`。仓库只包含
README 更新提交，没有修改外部 PX4、ROS、Gazebo 或本工程的锁定依赖。

官方完整工作区构建到可选的 `multi_map_server` 时失败。失败原因是参考仓库
自身的消息生成目标不完整：

```text
fatal error: multi_map_server/MultiOccupancyGrid.h: No such file or directory
target multi_map_server_messages_cpp was not created
```

这不是五个核心规划包的编译错误。使用官方五包白名单
`quadrotor_msgs;plan_env;path_searching;traj_utils;bspline_opt;ego_planner;waypoint_generator`
进行 Release 构建成功。

官方接口要点：

- `traj_utils` 提供 `Bspline`, `MultiBsplines`, `DataDisp` 消息及
  `plan_container.hpp`；
- `bspline_opt` 使用 `SwarmTrajData`、`drone_id` 和 `optimization/swarm_clearance`
  参与多机碰撞代价；
- `ego_planner` FSM 使用 `MultiBsplines` 交换各机完整轨迹链；
- 官方旧实现把轨迹总线硬编码为前后相邻无人机的绝对 Topic，且只在启动时发布
  完整轨迹链；
- 官方手动目标回调把 z 固定为 `1.0`，没有取消/新目标时间门；
- 官方 `traj_server` 固定发布 `/position_cmd`、`world` frame，并在首个 yaw
  定时回调可能出现零时间间隔除零。

## 当前工程迁移

五个核心包已从参考仓库同步到：

```text
AstraDrone_ros1_ws/src/Planner/ego-planner/planner/
  bspline_opt
  path_searching
  plan_env
  plan_manage
  traj_utils
```

保留了当前工程的 FAST-LIO、MAVROS/PX4 bridge、任务节点和测试；未修改外部
`/home/yanzu/PX4-Autopilot`。当前接口适配包括：

1. 规划消息统一使用 `traj_utils/Bspline` 和 `traj_utils/MultiBsplines`；
2. 轨迹、取消、目标、状态和输出 Topic 均可通过 launch 参数配置；
3. 多机 B-spline 改为公共绝对总线 `/swarm/trajectories`，每次成功重规划都刷新
   完整链；
4. 收到的轨迹链检查无人机编号、阶数、控制点、时间戳，并拒绝未来或已过期轨迹；
5. 手动目标保留有效 z，高度策略由 `fsm/use_goal_height` 和
   `fsm/manual_target_height` 控制；
6. 恢复 `PlannerStatus`、取消门、目标时间单调性、odom 有限值检查和
   `traj_server` 的 frame/topic 参数；
7. `traj_server` 拒绝取消前轨迹，并在零/无效 yaw 时间间隔时保持有限 yaw；
8. 现有 `ego_gazebo_bridge` 已改为把 EGO FSM 连接到带时间戳的
   `planning/goal`，避免把 `nav_msgs/Path` 错接到 `PoseStamped`。

## 验证结果

- 主工作区 Release 白名单构建：通过；
- 主工作区回归测试：`251 tests, 0 errors, 0 failures, 0 skipped`；
- `roslaunch --nodes ego_gazebo_bridge ego_gazebo_bridge.launch`：通过；
- `roslaunch --nodes ego_planner swarm.launch`：通过，官方 10 机示例可展开；
- 受保护的 `FAST_LIO/Log/mat_pre.txt` 未纳入本次迁移提交。

官方完整工作区的 `multi_map_server` 可选失败仍记录为上游已知问题；当前工程
只把通过核心白名单构建的五包作为迁移验收基线。
