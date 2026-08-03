# WP4–WP5 虚假长方体点云根因审计与修复

## 结论

虚假障碍不来自 Gazebo collision、Mid360、FAST-LIO、TF/外参或时间同步。根因是 swarm launch 把 `worksite.world` 中已被 `<state>` 覆盖的旧电线杆声明坐标 `(-21.0347, 24.6252)` 误当作 `pine_northwest`，并在每帧实时点云中人工注入半径 1.90 m、z=0.15–4.70 m、分辨率 0.50 m 的实心圆柱点阵。EGO 0.25 m 体素化与 0.40 m 膨胀后，RViz 中呈现为长方体占据区。任务层又使用同一虚假障碍做候选点净空检查，因此 UAV1/UAV3 在该扇区选择了外扩轨迹。

## Gazebo 审计

- `worksite.world` 的 `<state>` 实际将三根现场电线杆放在 `(-21.958,19)`、`(-21.8672,10)`、`(-22.0129,28)`；第四根在 `(-88.8978,3321.81)`，不在任务场地。
- 每根杆 collision 为半径 `0.0779475 m`、长 `9.144 m` 的细圆柱，加 `2.11606×0.041763×0.177806 m` 横担；无大长方体 collision。
- Gazebo Classic collision 可视化截图显示三根杆均为细杆+短横担，旧坐标处无模型。
- 未发现透明大模型、仅 collision 大模型、重复生成或旧实例残留。`my_asphalt_plane` 的 `80×80×0.1 m` collision 仅位于地面，不可能生成 z=2–5 m 的虚假体。

## 分层证据

修复前证据使用 `test_evidence/stage5_control_20260803_143131/stage5.bag`，UAV3 在 sim time 12419.5–12420.5 的旧坐标附近：

| 层 | 2.2/2.6 m 半径内飞行高度点 | 证据 |
|---|---:|---|
| FAST-LIO `/uav3/fast_lio/cloud_registered_raw` | 0 | 985 points/frame |
| 适配后 `/uav3/cloud_registered` | 0 | 985 points/frame |
| `/uav3/stage3/occupancy_inflate` | 2112 | 完整近区 3244 点，bbox `[-31.375,22.125,2.125]` 到 `[-26.625,27.125,4.875]` |

同一运行日志显示三机每帧均人工添加 `static_added=430`。因此“最早出现层”是 FAST-LIO 之后的 peer-filter/地图输入层；在当时已录制的三层 bag 中首次可见于 occupancy。

修复后三个隔离短窗 bag 的统计：

| UAV | 短窗 | Mid360 有效原始回波 | FAST-LIO/各过滤层 | occupancy 飞行高度 z=0.15–4.0 m |
|---|---:|---:|---:|---:|
| UAV1 | 32.4 s | 0 | 0 | 0 |
| UAV2 | 11.0 s | 0 | 0 | 0 |
| UAV3 | 6.7 s | 0 | 0 | 0 |

occupancy 在附近剩余点全部严格位于 `z=4.375 m`，是原 EGO 虚拟顶棚平面，不是长方体或飞行高度障碍。修复后日志为 `static_added=0`。

## 修复

只修改三个 launch 文件：

1. `dual_tower_inspection.launch`：删除 UAV1/UAV2 旧坐标和 `static_cylinders/*` 点云注入。
2. `triple_tower_inspection.launch`：删除 UAV3 旧坐标和 `static_cylinders/*` 点云注入。
3. `uav_tower_stack.launch`：从任务层 `static_obstacles` 中删除虚假 `pine_northwest`，仅保留 crane 和 `<state>` 确认的三根真实电线杆。

未修改 12.5 m 八边形航点、任务状态机、EGO 分辨率/膨胀/速度/加速度参数、world、FAST-LIO、EGO vendor 或外部 PX4。

## 重启清理

- FAST-LIO 每次重启均创建新进程内 IKD-Tree/缓冲区；`pcd_save_en=false`，无 PCD/map 加载链。
- EGO `GridMap::initMap()` 每次将 occupancy buffer 初始化为 unknown，inflated buffer 初始化为 0；无持久化局部图加载。
- 过去“重启后仍出现”是因为 launch 在每次启动时重新生成同一批 430 点，不是旧地图未清空。

## 同任务复测

以旧虚假坐标相对塔中心的 155.89° 扇区做几何对齐比较：

| UAV | 修复前半径均值/最大值 | 修复后半径均值/最大值 | 修复前/后最近旧坐标 | 修复前/后重定位 |
|---|---:|---:|---:|---:|
| UAV1 | 14.978 / 16.180 m | 11.692 / 12.348 m | 2.681 / 0.279 m | 2 / 0 |
| UAV2 | 9.701 / 11.841 m | 11.897 / 12.166 m | 2.629 / 0.286 m | 0 / 0 |
| UAV3 | 14.267 / 15.285 m | 11.721 / 12.371 m | 2.312 / 0.257 m | 1 / 0 |

UAV1/UAV3 的明显外突消失，三机均在无重定位、无该虚假占据体的情况下通过 WP4–WP5 验证窗口。

主复测在该窗口之后继续运行：UAV3 完成 364.33°，UAV2 完成 334.38°，UAV1 完成 277.33°；随后 UAV1 在真实西侧电线杆附近触发 `CURRENT_POSITION_IN_OCCUPANCY`，且 Gazebo/PX4 进程后续被系统 `Killed`。因此本轮只验收用户指定的 WP4–WP5 虚假障碍窗口，不声称整圈成功或安全落地；未对这个窗口之后的真实障碍事件进行参数优化。

## 验证与证据

- 目标包构建通过；`catkin_test_results`: **326 tests, 0 errors, 0 failures**。
- launch XML/参数展开通过；三机无 `static_cylinders`，任务障碍数组仅包含 crane+三根真实电线杆。
- 证据根目录：`test_evidence/stage5_false_obstacle_fix_20260803/`
- collision 截图：`gazebo_collision_view.png`
- 三机分层总 bag：`triple_validation/wp3_wp5_layers_full.bag`
- 三机隔离短窗：`uav1|uav2|uav3/fake_center_crossing_layers.bag`
- 轨迹：`triple_validation/trajectory_xy.png`、`trajectory_3d.png`

Git 基线：分支 `ego-swarm`，HEAD `f68f07137db57698c15f39d093f812ff9851da15`。未 commit、未 push，未清理或覆盖已有工作区修改。
