# Stage 5 UAV2 邻近占据点真实来源专项审计

日期：2026-08-03。范围只包括来源定位、录制和离线诊断；没有修改 0.40 m
地图膨胀、0.55 m 当前位置判定、机间阈值、world、航迹、Tier、八边形约束或
EGO/EGO-Swarm vendor。

## 结论

历史触发的原始点并非 UAV2 自身模型回波，也不能与 `worksite.world` 在该次
运行的任何已知 Gazebo 实体表面对应。它在 FAST-LIO 的 `uav2/camera_init` 中
确实是一个进入 EGO 的原始点；EGO 对它进行了预期的栅格量化和 0.40 m 膨胀，
随后 `planner_status_adapter` 又在已膨胀图上以 0.55 m 搜索，因而正确地产生了
`CURRENT_POSITION_IN_OCCUPANCY`。状态适配器不是原始点的来源，但它把两层裕量
叠加为约 0.95 m 的名义拒绝包络（离散栅格后上界约 1.17 m）。

已证明的上游问题类别是“飞行窗中 LiDAR 模拟回波或 FAST-LIO 相对 Gazebo world
的配准失配”；现有历史证据无法在这两个子类之间作可靠二选一。新的单机无控制
短窗证明静止基线的 FAST-LIO 外参公式、TF 转换和点云发布链自洽，但没有把 UAV2
带到历史失败的姿态/位置，故不能用它证明飞行时不存在漂移。不能据此扩大 self
mask、降低膨胀或降低当前位置安全半径。

因此，本轮结论不是“真实近障导致失败”。在可核对的运行时 Gazebo 几何下，失败点
映射到的 world 坐标距最近已知 collision mesh AABB 至少 2.26 m；历史 bag 又缺少
该时刻的 `/tf`、`/tf_static`、`/gazebo/model_states` 和 `/gazebo/link_states`，无法
把规划坐标中的 UAV2 位置与真实机体位姿逐时刻闭环。实际几何净空目前为**未证明**，
而不是零或小于安全距离。

## 新录制数据与复现方式

新增只录制脚本：

- `scripts/run_sh/stage5_uav2_near_occupancy_record_bag.sh`
- `scripts/tool/analyze_stage5_uav2_near_occupancy.py`

脚本拒绝覆盖已有路径，不发布控制、任务、解锁或 MAVROS setpoint。它录制 `/tf`、
`/tf_static`、Gazebo model/link states、Livox、FAST-LIO world/body、适配 cloud、
peer/self/ground 各级 cloud、原始/膨胀 occupancy、MAVROS 与 FAST-LIO odom、
planner status、goal、B-spline、命令、任务状态和 candidate topics。

本轮先进行了 UAV2 单机无控制短窗。启动参数为 `uav1_enabled=false`、
`uav2_enabled=true`、`uav3_enabled=false`、`enable_control=false`、
`uav2_mission_enabled=false`、`swarm_services_enabled=false`、`gui=false`。没有 arm、
起飞或发布控制指令。采集期间 ROS 节点只出现 `/uav2/...` 点云链；没有 U1/U3
点云处理节点。

完整数据在：

- `test_evidence/stage5_uav2_near_occupancy_20260803/uav2_no_control_full_chain_with_livox.bag.active`
- `test_evidence/stage5_uav2_near_occupancy_20260803/uav2_no_control_analysis.json`

录制原始文件在停止时未完成索引，保留为
`uav2_no_control_full_chain_with_livox.bag.orig.active`；随后由 `rosbag reindex` 生成
可读的 `.bag.active` 索引版本，两个文件均保留，未删除或覆盖。可读版本时间范围
11975.224–11991.771 s、16.547 s，包含：Livox 166 帧、FAST-LIO world/body 各 165
帧、`/tf` 166 帧、各 peer/self/ground cloud 164–166 帧、Gazebo model/link states
各 16,553/16,554 帧、膨胀 occupancy 151 帧。

第一次无控制录制（`uav2_no_control_full_chain.bag`，24.117 s）因手工启动环境遗漏
下层 Gazebo plugin path，未产出 Livox/注册点云；该 bag 保留，仅作为启动失败证据，
不用于结论。第二次使用项目既有 Stage 5 的 `LD_LIBRARY_PATH` 和
`GAZEBO_PLUGIN_PATH` 后才得到上述完整链路。

## 历史触发点簇逐帧追踪

以下是历史失败回放中首个可精确对齐的状态帧。输入证据为
`test_evidence/stage5_self_cloud_exclusion_fix_20260803/uav2_failure_window_full_filter_chain.bag`；
选择 status/膨胀图时间戳 13008.712 s，最近最终点云时间戳 13008.732990 s
（差 20.990 ms）。47 帧窗口为约 13007–13012 s。

| 坐标/阶段 | 数值或行为 |
| --- | --- |
| `uav2/camera_init` UAV2 odom | `(-14.941914, 8.264589, 3.010588)` m |
| 最近最终原始点 | `(-15.510577, 8.219481, 3.458909)` m，距离 0.725537 m |
| 可重建膨胀贡献原始点 | `(-15.488401, 8.421498, 3.497711)` m |
| 贡献点在 IMU/body | `(0.115338, 0.560295, 0.483037)` m |
| 依 FAST-LIO 外参反算的 LiDAR 点 | `(0.126338, 0.583585, 0.438917)` m |
| 最近发布膨胀体素 | `(-14.875, 8.375, 3.125)` m，距离 odom 0.172506 m |
| 0.35 m 种子簇 | 130 点；AABB `[-15.850874,7.900421,3.384377]`–`[-15.488023,8.555533,3.534522]` m；质心 `(-15.634667,8.240142,3.464664)` m |

点簇先前以 5 cm world voxel 统计：43 个可用帧中有 117 个 world voxel 出现在至少
35 帧；经正确的 odom 逆变换到 body 后没有对应的长期固定 voxel。机体在窗口内移动
约 0.28 m、yaw 约 0.7 度，故该统计支持“固定于规划 world”，反驳“刚性固定于
本机的 self echo”。该点在 body/LiDAR 中约位于右侧 0.56–0.58 m、上方 0.44–0.48 m，
也在已审计 Iris 机身、四个旋翼圆柱、机臂盒、MID360 和相机支架组合遮罩之外。扩大
遮罩会删除真实近障，违反本阶段限制。

滤波链证据：这 47 帧 raw 与 peer cloud 相同；self 前后逐帧点数和序列化 hash 相同，
`self_removed=0`。ground 仅删除 57–95 个低点，簇保留在最终输入中。因此 peer filter
和 self filter 均不是簇的产生者，也没有把两类职责混用。

历史 bag 没有 TF/Gazebo states，故无法对该窗口给出 Gazebo link 局部坐标、逐帧真实
机体速度或点簇相对某条 link 的速度；这是本轮保留的证据缺口，而非用推测补齐。

## Gazebo 几何核对

UAV2 的规划 frame 到 Gazebo world 在新 bag 中有静态链：

`world -> uav2/map: (4, 0, 0), identity`，
`uav2/map -> uav2/camera_init: identity`。

按此已验证的静态变换，历史最近原始点在 Gazebo world 为
`(-11.510577, 8.219481, 3.458909)` m；可重建贡献点为
`(-11.488401, 8.421498, 3.497711)` m。新 bag 的同一 `worksite.world` 运行时
`/gazebo/model_states` 中离前者最近的模型原点如下：

| 模型（运行时 state） | 模型原点距点 | 几何结论 |
| --- | ---: | --- |
| `Oak_tree_0` `(-15.0907,5.38341,0)` | 5.729 m | collision 是完整 `oak_tree.dae`；mesh AABB 为 x `[-2.331,1.846]`、y `[-1.648,1.694]`、z `[-0.028,2.570]` m，最近 AABB 距离 2.258 m |
| `Pine_Tree_6_clone_0` `(-5.0111,7.89805,0)` | 7.370 m | pine mesh AABB 最近距离 6.054 m |
| `radio_tower` `(-10.0551,19.7104,0)` | 12.088 m | tower mesh AABB 最近距离 8.863 m |
| `telephone_pole_clone_0` `(-21.8672,10, -0.448185)` | 11.211 m | pole cylinder/crossbar 均远离该点 |

一个容易误判的事实是：world 文件模型声明中 `Oak_tree_0` 的 pose 为
`(-10.0443,10.2228,0)`，但 `<state>`（也是运行时 `/gazebo/model_states`）实际覆盖为
`(-15.0907,5.38341,0)`。本审计使用后者，而非视觉浏览时容易看到的前者。即便错误地
使用旧声明，点也不在 oak AABB 内。

`Oak_tree`、`Pine_Tree`、`radio_tower` 都让 collision 使用与 visual 相同的 DAE mesh；
树的 visual 只把同一 mesh 分为 branch/bark submesh，没有额外 collision-only 的枝条。
telephone pole 的 collision 明确为半径 0.07795 m、长 9.144 m 圆柱和
2.11606 x 0.04176 x 0.17781 m 横担，visual 为 DAE；它也不接近该点。已经核对的模型
中未发现隐藏 link、透明 visual 之外的大 collision、scale 或 pose 可将该点解释为表面。

因此“UAV2 在这个原始点所宣称的规划 world 位置真实撞近了上述模型”没有证据支持。
但历史失败时没有 Gazebo model/link states，不能把该次真实 UAV2 机体位置同规划 odom
做逐帧对齐，故也不能声称真实净空已经量化完成。

## TF 与 FAST-LIO 外参核对

新短窗记录了完整链及发布源：

| 链路 | parent -> child | 类型/发布者 | 核对结果 |
| --- | --- | --- | --- |
| world 原点 | `world -> uav2/map` | 静态 `world_to_uav2_map` | `(4,0,0)`，identity rotation，单次 static |
| 规划原点 | `uav2/map -> uav2/camera_init` | 静态 `verified_map_to_planning_frame` | `(0,0,0)`，identity，单次 static |
| FAST-LIO odom | `uav2/camera_init -> uav2/body` | 动态 `frame_adapter` 转发 FAST-LIO | 166 帧、约 10 Hz、单一父子关系；静止窗只有毫米级估计抖动，无跳变/冲突 |
| 机体模型 | `uav2/body -> uav2/base_link` | 静态 | identity |
| MID360 安装 | `uav2/base_link -> uav2/mid360_link` | 静态 | `(0,0,0.08)`，identity |
| D435 安装 | `uav2/base_link -> uav2/d435_link` | 静态 | `(0.12,0,0.03)` 与 launch 一致 |

FAST-LIO 配置为 `extrinsic_R=I`、`extrinsic_T=(-0.011,-0.02329,0.04412)` m、
`extrinsic_est_en=false`。代码变换为：

`p_world = R_imu_world * (R_lidar_imu * p_lidar + T_lidar_imu) + p_imu_world`。

在新 bag 的 165 个同时间戳 world/body cloud 帧上，对每帧最多 100 个同序号点独立用
该公式重算，合计 16,500 点：最大 world 误差 `2.8865e-6 m`、均值
`3.2277e-7 m`。这同时核验了 FAST-LIO 输出、`/uav2/Odometry` 和转发 TF 的一致性。
它排除了“静止基线中 FAST-LIO 外参公式或 frame adapter 把同一原始点算错”的假设。

这个核验不覆盖历史飞行姿态；原失败 bag 缺少这些 TF 和 Gazebo link states，不能据此
排除飞行时 FAST-LIO 与 Gazebo 真值的相对漂移，也不能证明 LiDAR 仿真在该视角不存在
非模型回波。

## 原始点到膨胀体素的逐级传播

历史回放的确切传播路径为：

```text
/uav2/cloud_registered
  3269 points, nearest 0.725537 m
  -> peer filter: 3269 points (0 removed)
  -> self filter: 3269 points (0 removed)
  -> ground filter: 3201 points
  -> /uav2/stage3/cloud_registered_filtered
  -> GridMap: origin (-45,-45,-0.5), resolution 0.25 m
     contributor (-15.488401,8.421498,3.497711)
     source cell center (-15.375,8.375,3.375)
     x/y/z offset (+2,0,-1) cells
  -> inflated center (-14.875,8.375,3.125), range 0.172506 m
  -> /uav2/stage3/occupancy_inflate
  -> planner_status_adapter radius < 0.55 m
  -> CURRENT_POSITION_IN_OCCUPANCY
```

GridMap 对当前 cloud 的实际实现是：`ceil(0.40 / 0.25)=2` 个 XY cell，即每个输入点
XY 各扩 ±0.50 m；Z 固定扩 ±1 cell，即 ±0.25 m。输入点先以 `floor()` 定位到 cell，
发布为 cell 中心，因此单轴量化误差最多 0.125 m、三维最多约 0.2165 m。该帧根据最终
cloud 重建出 1,561 个（含重复贡献）可落入 0.55 m 搜索球的膨胀 cell；发布占据图实际
去重后为报告中的 13–21 个近点。

`occupancy_stamp_adapter`只复制点云并在 zero stamp 时补 receipt sim time，不进行第二次
膨胀或体素化。`planner_status_adapter`订阅的正是
`/uav2/stage3/occupancy_inflate`，而非 `/uav2/grid_map/occupancy`；代码在该已膨胀点集上
执行 `< collision_radius_`，其当前值是 0.55 m。

所以 0.40 m 与 0.55 m 是两级连续安全裕量，不是同一个体素被“重复加入”。其对原始
障碍点的名义平面拒绝距离约 `0.40 + 0.55 = 0.95 m`，包含 0.25 m 栅格量化与方形
XY 膨胀时，理论保守上界约 `0.95 + sqrt(3)*0.125 = 1.1665 m`。这在当前代码中缺少
“第二层是机体半径还是独立紧急保护”的显式语义说明，故应视为未被命名的重复保守设计；
它是实际触发放大器，但不是虚假 raw 点的制造者。本轮不修改任一数值。

## 无控制短窗结果与命名空间隔离

新短窗在 11991.759 s 取样：raw/peer 最近点均为 1.437562 m；self 前后 2,769 点的
序列化 hash 和点数相同；ground 后最终点最近 1.886943 m；膨胀图最近点 1.152546 m；
重建的 0.55 m 内膨胀体素为 0，planner status 的
`current_position_in_collision` 为 0/166 帧。这是正常地面静止场景，不能作为历史位置
修复成功的替代证据。

实际运行节点列举仅有：`/uav2/laserMapping`、`/uav2/frame_adapter`、
`/uav2/teammate_cloud_filter`、`/uav2/self_cloud_filter`、
`/uav2/ground_cloud_filter`、`/uav2/drone_1_ego_planner_node`、
`/uav2/occupancy_stamp_adapter` 和对应 bridge/MAVROS；录制到的所有新增 topic 都以
`/uav2/` 开头。UAV1/UAV3 没有被启动，未发现跨命名空间 cloud 或参数串扰。

脚本静态测试通过：`bash -n`、Python `py_compile`、已有文件路径的拒绝覆盖测试（返回
2）。本轮仅新增 Bash/Python 诊断工具；没有需要重新编译的 C++ 目标。前一轮 self
filter 的白名单编译与 4 个 self-filter GTest 已通过，且本轮没有修改该实现。

## 成功样本对照

可用成功样本为 `test_evidence/stage5_final_validation_20260802_192759`。UAV2 CSV 在
13008.0–13010.0 s 显示 `NAVIGATING`、`planner_state=EXEC_TRAJ`、空 planner reason；
其坐标约 `(-6.28,10.47,3.08)` 至 `(-6.19,10.50,3.10)`（UAV2 local），与失败窗口 odom
`(-14.94,8.26,3.01)` 不是同一空间位置或扇区。全成功 CSV 中的最近 local 样本是
12927.1 s 的 `(-14.1693,8.11347,3.17384)`，距失败 local odom 0.804 m，处于 sector 7、
`EXEC_TRAJ`；该成功 bag 没有本轮新增 self 前/后、TF、link states 和原始 occupancy，
不能对这一“最近但非同位姿”样本做要求的逐点来源比较。

因此可证实的是：成功运行没有在其已记录轨迹上触发该状态；不能诚实地声称成功样本
已经覆盖了失败的相同世界位置、航向和姿态。新增 recorder 正是为下一次短窗把这项对照
补齐。

## 根因分级与下一轮最小修改位置

已证明：

- self/peer/ground filter 都不是历史簇的来源；self 过滤保持。
- raw 点是经过 GridMap 的真正输入，0.40 m 膨胀、体素中心和 status 的 0.55 m 搜索均按
  当前代码语义工作。
- 在可验证的静止基线，TF 链、FAST-LIO 外参和 world/body cloud 变换自洽。
- 将失败点按已记录 `world -> uav2/map` 投到运行时 Gazebo world 后，未落在已审计的
  Oak/Pine/tower/pole/ground collision 表面；没有发现 visual/collision 尺寸不一致能解释它。

高概率原因：

- 飞行状态下 LiDAR 模拟回波包含一个不对应审计 Gazebo 几何的表面，或 FAST-LIO 相对
  Gazebo 真值的姿态/位置在该窗口失配，使真实环境回波被注册到错误的规划 world 坐标。
  两者均在 self filter 之前、GridMap 之前发生。

未证明假设：

- 该点是某一个特定 tree、tower、pole、隐藏模型或真实近障；当前证据反而不支持它们。
- FAST-LIO 运行时外参配置本身错误；静止跨 16,500 点核验反对该假设，但不能排除飞行
  初始化/时间同步/漂移。
- status adapter 的第二层裕量本身是设计错误；已证明它具备重复保守效果，是否保留要由
  安全语义设计决定，不能以本次失败直接下结论。

下一轮的最小位置应当是**诊断性**短窗复现，不是安全参数或轨迹修改：以本脚本启动，
在 UAV2 接近历史 local `(-14.94,8.26,3.01)`、相近 yaw 的受控低速飞行中录制新 bag，
并用 `/gazebo/link_states` 与 `/uav2/mavros/local_position/pose` 对 FAST-LIO odom 做
逐帧真值残差；同时把相同 raw LiDAR 点分别经 TF 和 FAST-LIO 外参计算到 world。若该点
在 LiDAR frame 固定则审计 LiDAR SDF/仿真插件自回波；若 LiDAR frame 正常而 world 残差
增大则修正 FAST-LIO/真值对齐接线。两种结论出现前，不修改 mask、膨胀或轨迹。

## 是否可进入八边形直线边段走廊修改

**否。**自身点云问题已经排除为该簇来源，但 UAV2 的上游点云/Gazebo-world 对齐来源
尚未在历史飞行姿态下得到唯一证据。先完成上述带完整 TF 和 Gazebo states 的短窗复现并
确认该点的 LiDAR-frame 来源，才能安全地将问题与八边形直线边段走廊约束解耦。
