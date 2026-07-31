# EGO-Swarm 三机低空绕塔集成报告

日期：2026-07-31
分支：`ego-swarm`
工作区：`/home/yanzu/AstraDroneOpen`

## 结论

本轮完成了迁移后单机回归、三机 EGO-Swarm 实例化、公共坐标/真实 B-spline
通信、三对冲突重规划、取消隔离、轨迹过期拒绝、时空协调器和低空安全点云
适配。代码构建和无控制验收通过；三机实际 2/3/4 m 整圈验收仍未通过，因而
没有把部分飞行结果包装成成功。`takeoff_interval_sec:=0.0` 的实际飞行尚未
开始。

硬性安全参数保持不变：

```text
optimization/swarm_clearance = 1.5 m
EGO 椭球所需归一化距离 = 3.0
固定任务高度 = UAV1 2 m / UAV2 3 m / UAV3 4 m
```

没有写死 `safety=true`，没有降低垂直门限来放行，也没有修改官方参考仓库或
PX4 外部树。定位、心跳、预测轨迹、B-spline 过期、MAVROS 位姿时间戳和
`SAFETY_INHIBIT` 仍然 fail-closed。

## 分阶段状态

| 项目 | 状态 | 当前证据 |
|---|---|---|
| 迁移后单机 drone_id=0 | 通过 | 26→22 m、固定 2 m 绕塔、goal/cancel/fresh goal、返航、AUTO.LAND；无 HOLD/emergency 的回归已完成 |
| 代码接入和构建 | 通过 | 三套 planner/FSM/bridge、相位/ENTRY/EXIT/landing 协调器；选定包构建通过 |
| 无控制/安全门禁 | 通过 | 277 tests，0 errors，0 failures；launch 参数展开、三机 frame/Topic、静态几何和唯一控制出口检查通过 |
| 真实三条 B-spline | 部分通过 | 单机真实轨迹和三对 UAV1-2、UAV2-3、UAV3-1 冲突注入均触发 EGO 重规划；完整三机同时持续轨迹尚未形成可验收闭环 |
| 3 s 间隔实际飞行 | 未通过 | 早期运行在定位时间戳失效时安全 HOLD/返航；优化静态点后 UAV1 可进入绕塔，但整圈/三机相位闭环未完成；代表性证据见 `test_evidence/ego_swarm_three_uav/3s_r7/` |
| 0 s 同时起飞 | 未执行 | 必须先通过 3 s 间隔验收 |

代表性 3 s 运行的真实数据（r7）显示：起飞时间为仿真时钟
11952.91/11956.01/11959.81，UAV1 最小水平距离约 3.88 m，椭球距离约
3.89 m；但 UAV2/UAV3 没有进入绕塔，UAV1 运行在人工安全收尾前进入错误态，
所以这些数字不是三机整圈通过证据。此前 r5/r6 的定位失效均触发原桥接器
fail-closed，未发生碰撞。

## 关键实现

- `swarm_clearance=1.5` 全链保留；manager 使用实时状态、心跳、定位、预测
  轨迹、走廊几何和未来相位预测，不再以固定层垂直间距永久拒绝所有任务。
- UAV1/UAV2/UAV3 默认相位约 270°/45°/0°，ENTRY_GATE/EXIT_GATE 由唯一
  占用者串行发放；到达 ENTRY 后必须满足实时约 120° 相位才解除
  `WAIT_ORBIT_PERMISSION`。
- 不能因为 ENTRY 走廊和 2/3/4 m 垂直层差而放行同 XY。协调器重新保留
  `projected_entry_phase_clear`，预测到达时若可能追上前机则继续 fail-closed。
- 三机静态松树点云改为半径 1.90 m 的保守包络、0.50 m 缓存格点（实测半径
  1.65 m），从每帧约 1408 点降至约 430 点；安全包络没有缩小。
- `mavros_pose_timeout` 为 1.5 s，仍同时检查消息接收时间和源 header 时间戳；
  过期、零时间、未来时间戳照常 HOLD/禁止降落。

## 修改文件

主要修改位于：

- `MissionControl/astra_tower_mission/src/stage3_ego_mission_node.cpp`
- `MissionControl/ego_gazebo_bridge/` 与三份 `uav*_bridge.yaml`
- `Planner/ego-planner/planner/plan_manage/` 的迁移适配与轨迹门
- `Swarm/astra_swarm_manager/`、`astra_swarm_safety/`、`astra_swarm_perception/`
- `Swarm/astra_swarm_bringup/launch/{dual,triple}_tower_inspection.launch`
- `MissionControl/astra_tower_mission/config/stage3_low_altitude_ego.yaml`
- `Swarm/astra_swarm_bringup/config/swarm_low_altitude_ego.yaml`
- 冲突注入、证据记录和传输探针脚本

## 构建、测试与启动

```bash
source /opt/ros/noetic/setup.bash
cd /home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES='astra_custom_msgs;quadrotor_msgs;ego_planner;plan_manage;traj_utils;astra_tower_mission;ego_gazebo_bridge;astra_swarm_manager;astra_swarm_safety;astra_swarm_perception;astra_swarm_bringup' -j2
catkin_make -DCATKIN_WHITELIST_PACKAGES='astra_custom_msgs;quadrotor_msgs;ego_planner;plan_manage;traj_utils;astra_tower_mission;ego_gazebo_bridge;astra_swarm_manager;astra_swarm_safety;astra_swarm_perception;astra_swarm_bringup' run_tests -j2
catkin_test_results build/test_results
```

无控制检查：

```bash
roslaunch astra_swarm_bringup triple_tower_inspection.launch \
  start_sim:=false enable_control:=false gui:=false
```

实际验收入口（仍需重新完成并记录完整闭环）：

```bash
roslaunch astra_swarm_bringup triple_tower_inspection.launch \
  enable_control:=true gui:=false takeoff_interval_sec:=3.0 inspection_laps:=1
```

只有上述 3 s 运行完整通过后，才允许：

```bash
roslaunch astra_swarm_bringup triple_tower_inspection.launch \
  enable_control:=true gui:=false takeoff_interval_sec:=0.0 inspection_laps:=1
```

## 尚存风险和下一步

1. 需要在当前协调器的预测 ENTRY 许可下重新完成一整轮 3 s 实际任务，确认三机
   都进入独立 ENTRY、保持 120° 相位、各完成至少一圈、经独立 EXIT 返回并串行
   AUTO.LAND。
2. 需要随后执行 0 s 起飞、节点重启/旧轨迹过期和真实三机持续 B-spline 证据，
   并计算全程最小水平/垂直/三维/椭球距离、重规划次数、跟踪误差和 HOLD 次数。
3. `map↔camera_init`、真实机外参和仿真调度负载仍不是实机标定结论；不得把本
   报告的部分实际运行描述为真机通过。

本地代表性证据保留在 `test_evidence/ego_swarm_three_uav/3s_r7/`；临时 bag、
日志和旧 CSV 未作为验收结论使用。受保护的历史图片未纳入本次提交。
