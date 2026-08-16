# worksite 单机/三机 Mid360 启动失败根因报告

日期：2026-08-15（Asia/Shanghai）  
分支：`scene01-3uav-circuit-mission`  
基线 HEAD：`e7e17986042ffd7000523d4cde1fcda3afbae594`

## 1. 结论

本次故障的明确根因是：

> `worksite.world` 中 `vrc_driving_terrain` 的 Gazebo/ODE heightfield
> collision，与当前 Livox 插件一次维护 20,000 条 ODE ray 的自定义
> MultiRay 求交路径组合后出现病态性能；物理更新线程长时间被首次射线求交
> 占用，随后 PX4/Gazebo lockstep 等待仿真推进并报 simulator poll timeout。

根因分类为 **Gazebo ray performance**。更完整地说，它是
`worksite heightfield collision × Mid360 ODE multi-ray` 的组合问题；
`PX4/Gazebo lockstep` 是下游症状，不是首因。

以下候选均已被实际实验排除为首因：

- 单机 PX4 instance、MAVROS port 或 target system ID 冲突；
- 三机 namespace/remap/frame 配置破坏单机；
- `worksite.world` 本身无法加载；
- `<state>` 中的 `11934.010` 本身冻结仿真；
- UAV 基础模型或 PX4/MAVROS 在没有 Mid360 时阻塞；
- 人物高面数 visual/collision 是决定性根因；
- FAST-LIO、filtered cloud、EGO 或 Observation v2 导致启动失败。

最小修复是把地形 collision 从 ODE heightfield 表示改为由同一张
129×129 高度图全部像素生成的三角网格：16,641 个顶点、32,768 个三角形。
地形 visual、高度样本、尺寸、偏移、全部其他 worksite 对象以及 Mid360
正式参数均未改变。

修复后，worksite 的单机、双机和三机无控制完整链均形成；三机三套
Mid360 → FAST-LIO → formal filtered cloud 均持续出消息。

## 2. 用户假设的判定

“之前三机绕塔开发是否导致当前单机绕塔/单机 Mid360 链路不再兼容？”

**NO。**

证据：

1. 当前共享栈用 `triple_tower_inspection.launch` 的 `uavN_enabled` 参数选择
   实例。只启用 UAV1，并把 PX4 guard namespace 列表设为 `[/uav1]` 后，
   实际跑通了 PX4、MAVROS、Mid360、FAST-LIO、frame adapter、peer/self/
   ground filter 和 EGO；不是只靠静态 launch 审计得出的结论。
2. UAV1-only、UAV1+UAV2、UAV1+UAV2+UAV3 使用同一套模型 xacro、spawn
   helper、frame adapter 和 `uav_tower_stack.launch`。三种规模都通过，
   且每架 topic/frame 都落在各自 `/uavN` 下。
3. worksite + PX4/MAVROS + 正式多机 UAV1 基础模型、但不加载 Mid360 时，
   `/clock` 正常、PX4 simulator_mavlink 正常、MAVROS `connected=True`。
4. 修复前同一正式 UAV1/Mid360 配置在 `outdoor_village.world` 正常输出
   IMU/LiDAR；只换成 worksite 才阻塞。
5. 修复只涉及地形 collision。没有修改单机/多机 namespace、PX4、MAVROS、
   FAST-LIO 或 frame 配置，但两种规模同时恢复。

需要区分两个“单机”概念：

- 旧名义单机入口 `astra_example.launch` 只负责 Gazebo + UAV + PX4 + MAVROS；
  它不是包含 FAST-LIO/filter/EGO 的完整工程栈。修复后它的全局
  `/livox/imu`、`/livox/lidar` 和 MAVROS 仍能正常运行。
- 当前真正完整、与三机一致的单机工程栈，是共享的三机 launch 只启用
  UAV1。它不是一份复制出来的单机配置，而是同一参数化栈的单实例模式。

因此，项目中存在可独立运行且与三机配置一致的完整单机链，但入口是共享
launch 的单实例配置，不是旧 `astra_example.launch` 独自完成全部模块。

## 3. `/clock = 11934.010` 的解释

`worksite.world` 保存了：

```xml
<state world_name='default'>
  <sim_time>11934 9000000</sim_time>
  <real_time>12588 332487621</real_time>
  <wall_time>1784852526 439025746</wall_time>
  <iterations>0</iterations>
</state>
```

所以 Gazebo 加载后第一帧仿真时间就是：

```text
11934 s + 9,000,000 ns = 11934.009 s
```

Topic/日志按毫秒显示时即 `11934.010`。world-only 实验中时钟从该偏移继续
推进，RTF 为 0.99–1.00，证明 `<state>` 只是保存的初始仿真时间，不是冻结
原因。

修复前加入 Mid360 后，首次重型 ODE ray/heightfield 求交恰好发生在这个初始
时间附近，physics thread 未能完成下一步，所以表面上像“固定停在
11934.010”。精确三角网格 collision 修复后仍保留 `<state>`，时钟可从同一
偏移继续推进，进一步排除了 `<state>` 根因。

## 4. 当前真实启动链

### 4.1 旧名义单机基础入口

```text
worksite.world
  → gazebo_ros empty_world
  → external PX4 iris_mid360.sdf（model=iris_mid360）
  → PX4 SITL instance 0 / TCP 4560
  → MAVROS（全局 /mavros，udp://:14540@localhost:14557）
  → Mid360 plugin（全局 /livox/imu、/livox/lidar）
```

该入口没有继续 include FAST-LIO、过滤链或 EGO，不能把它称为完整规划栈。

### 4.2 当前完整单机工程栈

```text
worksite.world
  → triple_px4_mavros.launch（仅 uav1_enabled=true）
  → single_vehicle_spawn_verified.launch
  → iris_without_GPS_0 + iris_mid360_d435.sdf.xacro
     model=iris_mid360_00, robotNamespace=/uav1
  → PX4 instance 0 / MAV_SYS_ID 1 / TCP 4560
  → MAVROS /uav1（14540 ↔ 14580, target system 1）
  → /uav1/livox/imu + /uav1/livox/lidar
  → FAST-LIO raw topics
  → frame_adapter
  → /uav1/Odometry + /uav1/cloud_registered
  → teammate_cloud_filter
  → self_cloud_filter
  → ground_cloud_filter
  → /uav1/stage3/cloud_registered_filtered
  → EGO drone_0（无目标、无控制）
```

### 4.3 三机工程栈

```text
worksite.world → one gzserver
  ├─ UAV1: model iris_mid360_00, PX4 i0/sys1, TCP4560,
  │        MAVROS 14540↔14580, drone_id=0, /uav1/...
  ├─ UAV2: model iris_mid360_11, PX4 i1/sys2, TCP4561,
  │        MAVROS 14541↔14581, drone_id=1, /uav2/...
  └─ UAV3: model iris_mid360_22, PX4 i2/sys3, TCP4562,
           MAVROS 14542↔14582, drone_id=2, /uav3/...

每个 /uavN：Mid360 → FAST-LIO → frame adapter → peer/self/ground filter → EGO
```

## 5. 单机/三机静态审计对照

| 项目 | 旧 `astra_example` | 当前共享栈 UAV1 | UAV2 | UAV3 |
|---|---|---|---|---|
| 模型来源 | 外部 PX4 `iris_mid360.sdf` | 参数化 xacro | 同左 | 同左 |
| Gazebo model name | `iris_mid360` | `iris_mid360_00` | `iris_mid360_11` | `iris_mid360_22` |
| ROS namespace | 全局 | `/uav1` | `/uav2` | `/uav3` |
| PX4 instance | 0/default | 0 | 1 | 2 |
| MAV_SYS_ID / target system | 1 | 1 | 2 | 3 |
| simulator TCP | 4560 | 4560 | 4561 | 4562 |
| MAVROS bind/remote | 14540/14557 | 14540/14580 | 14541/14581 | 14542/14582 |
| Mid360 topic | `/livox/*` | `/uav1/livox/*` | `/uav2/livox/*` | `/uav3/livox/*` |
| lidar frame | `mid360_link` | `uav1/mid360_link` | `uav2/mid360_link` | `uav3/mid360_link` |
| planning frame | 未启动 | `uav1/camera_init` | `uav2/camera_init` | `uav3/camera_init` |
| FAST-LIO remap | 未启动 | 完整 | 完整 | 完整 |
| formal filtered cloud | 未启动 | `/uav1/stage3/...` | `/uav2/stage3/...` | `/uav3/stage3/...` |
| EGO drone id | 未启动 | 0 | 1 | 2 |

外部 PX4 与仓库部署副本的 `iris_mid360.sdf` SHA-256 均为
`27c623fb392b44ebac92f280eebbb664890e520ef64b20fdeccf15d71903137d`，
旧单机没有因两份模型漂移而失败。

## 6. 分层隔离实验

所有实验均禁用任务控制，未解锁、未起飞；Observation v2 未启动。

| 层级 | 配置 | 修复前结果 | 结论 |
|---|---|---|---|
| Test 1 | worksite only | PASS；`/clock` 持续，RTF 0.99–1.00 | world/state 本身正常 |
| Test 2 | worksite + 正式编号基础机体，无 PX4/传感器 | PASS；spawn 1.17 s，时钟持续 | 基础模型正常 |
| Test 3 | worksite + UAV1 PX4/MAVROS，无 Mid360 | PASS；MAVROS connected，时钟持续 | ID/port/lockstep 基础链正常 |
| Test 4 | worksite + 正式 UAV1 Mid360，无 FAST-LIO/EGO | FAIL；59 s 无首条 IMU/LiDAR，时钟停在 11934.010 | 缩小到 world × Mid360 |
| 对照 | outdoor + 同一正式 UAV1 Mid360 | PASS；IMU约83–100 Hz，LiDAR约10 Hz | Mid360/namespace 单独正常 |
| 隔离 A | worksite 删除13个人物后加 Mid360 | FAIL；RSS约3.27 GiB，仍无消息 | 人物 mesh 非决定因素 |
| 隔离 B | worksite 运行时只删除 terrain 后加 Mid360 | PASS；IMU约100 Hz，LiDAR约10 Hz，RTF约0.92 | 锁定 terrain |
| 隔离 C | terrain 改用全部129×129高度样本的临时 mesh | PASS；IMU约100 Hz，LiDAR约10 Hz，RTF 0.53–0.54 | 锁定 ODE heightfield 表示 |
| Test 5–7 | 正式修复后加入 FAST-LIO、过滤链、EGO | PASS | 完整单机链恢复 |

修复前正式 Mid360 单机在约59 s时：

- `gzserver` CPU 约105%；
- RSS 3,287,064 KiB；
- 虚拟地址约12 GiB；
- 无 `/clock` 下一步、无 IMU、无 LiDAR；
- PX4 持续 simulator poll timeout。

它不是“没有负载”，而是 physics thread 在高度图射线求交中耗尽计算。

## 7. 修复后正式验证

### 7.1 worksite 单机与 outdoor 对照

两次均使用同一参数化 UAV1 栈、同一 PX4/MAVROS/Mid360/FAST-LIO/filter/EGO，
`d435_enabled=true`、`lidar_downsample=1`，只替换 world。

| 项目 | outdoor_village | worksite（修复后） |
|---|---:|---:|
| Gazebo RTF | 0.59–0.61 | 0.49–0.50 |
| model verified | 约5.8 s | 约5.9 s |
| 首条 IMU | 启动探针窗口内，随后约83–100 Hz | 启动探针窗口内，随后约100 Hz |
| 首条 LiDAR | 启动探针窗口内，随后约10 Hz | 启动探针窗口内，随后约10 Hz |
| FAST-LIO 首条 odom | 25 s验收窗口前已形成，约10 Hz | 25 s验收窗口前已形成，约10 Hz |
| formal filtered cloud | 25 s验收窗口前已形成，约10 Hz | 25 s验收窗口前已形成，约10 Hz |
| gzserver CPU | 约105% | 约135% |
| gzserver RSS | 2,817,052 KiB | 3,071,160 KiB |
| MAVROS | connected, unarmed | connected, unarmed |

“首条消息”这一轮保存的是明确上界：订阅验收在启动25 s后开始，各 topic
均立即返回。plugin/spawn 日志另证实模型约6 s完成。没有把后置订阅时间误写成
消息真实首次发布时间。

worksite 仍比 outdoor 更重，但已从完全阻塞恢复为稳定可运行；这支持根因是
特定 collision 表示的病态求交，而不是笼统的“worksite 模型太多”。

### 7.2 1/2/3 架扩展性

| 规模 | spawn verified | MAVROS | IMU | LiDAR / odom / filtered | RTF | gzserver RSS |
|---|---:|---|---:|---:|---:|---:|
| 1架 | 约5.9 s | 1/1 connected | 约100 Hz | 约9.6–10.8 Hz | 0.49–0.50 | 3,071,160 KiB |
| 2架 | 约13.0 s，两架均存在 | 2/2 connected | 各约100 Hz | 各约9.6–10.3 Hz | 约0.29 | 5,023,296 KiB |
| 3架 | 约18.0 s，三架均存在 | 3/3 connected | 各约98–103 Hz | 各约8.16–8.32 Hz | 约0.23 | 7,032,632 KiB |

三机性能随传感器数量下降，但没有出现“1机正常、3机彻底卡死”。三架的
namespace、model name、PX4 instance、MAVROS port、frame 和 formal filtered
cloud 均彼此独立。现有主机上三机启动链通过，但 RTF 0.23 表明后续带飞行任务
验收仍应关注算力余量；本次范围不进入飞行。

## 8. 代码修改

### `simulation/astra_gazebo_worlds/worksite.world`

只把 `vrc_driving_terrain/link/collision` 的 geometry 从 `<heightmap>` 改为
`heightmap_collision.obj` mesh。保留：

- `<state>`；
- 全部模型、pose 和 world plugin；
- heightmap visual；
- 原有 grass plane collision；
- 原 collision 的 surface/contact/friction 配置。

### `simulation/astra_gazebo_models/vrc_driving_terrain/model.sdf`

同步模型源定义，避免以后通过 `model://vrc_driving_terrain` 重新 include 时又
回到有问题的 heightfield collision。visual heightmap 未改。

### `simulation/astra_gazebo_models/vrc_driving_terrain/meshes/heightmap_collision.obj`

从原 `heightmap.png` 每个像素一一生成：

- 129×129 = 16,641 vertices；
- 128×128×2 = 32,768 triangles；
- x/y 范围与原 geometry 同为 500×500 m；
- z 使用原 `size.z=118` 和 `pos.z=-15`；
- 不做降采样。

源高度图 SHA-256：
`bbc5cc3e95b24bc65e9e1985d2dd69faf18bbb0ab6262181eae5e152b3c71ce9`。

新 collision mesh SHA-256：
`e7d408857687fb9609ba6db17c2dd7eb7b5f50a0916a5ef7114516fab418bb86`。

## 9. 是否改变正式 world / Mid360 精度

### 正式 world

**改变了 collision 的内部表示，但没有改变视觉场景或高度样本分辨率。**

原 heightfield visual 仍存在；所有 16,641 个原始高度样本都进入 collision
mesh，没有删地形、删建筑、删人物，也没有新建替代 world。车辆物理仍与地形
和原平面 collision 交互。

### Mid360

**没有改变。**

- `samples=20000`；
- `downsample=1`；
- scan CSV、update rate、range、noise、topic 和 frame 未修改；
- 单机/双机/三机正式验证都使用 `lidar_downsample=1`。

因此本次没有通过降低射线数或传感器更新率换取 PASS。

## 10. 当前状态

| 验收项 | 状态 | 证据摘要 |
|---|---|---|
| worksite world only | **PASS** | `/clock` 持续，RTF 0.99–1.00 |
| single UAV spawn | **PASS** | 旧单机和共享栈 UAV1 都实际存在 |
| single UAV PX4/MAVROS | **PASS** | connected=true、unarmed、AUTO.LOITER |
| single UAV Mid360 | **PASS** | IMU约100 Hz、LiDAR约10 Hz |
| FAST-LIO | **PASS** | `/uav1/Odometry`、`/uav1/cloud_registered` 约10 Hz |
| filtered cloud | **PASS** | `/uav1/stage3/cloud_registered_filtered` 约10 Hz，frame正确、finite/dense |
| single UAV EGO | **PASS** | EGO存活并订阅正式 filtered cloud；无目标、无飞行 |
| two UAV | **PASS** | 两套完整链约10 Hz，RTF约0.29 |
| three UAV | **PASS** | 三套完整链约8.2 Hz，RTF约0.23；仅启动链验收 |

`/uav1/mavros/setpoint_raw/local` 在单机验收中没有 publisher。Observation v2
node/topic 均不存在，故也明确证明原故障发生在 Observation 之前。

## 11. 证据位置与范围边界

本轮证据位于：

`runtime_artifacts/worksite_mid360_startup_20260815/`

其中保留 world-only、model-only、PX4-no-Mid360、正式失败、人物/地形隔离、
精确 mesh 诊断、outdoor 对照以及 1/2/3 架最终成功证据。

本轮没有：

- 修改或启动 Observation v2；
- 修改 Learning Speed、RL reward、动态 `v_max`；
- 修改 EGO、FAST-LIO、PX4 或任务/航点；
- 解锁、起飞或执行绕塔任务；
- 新建 world；
- commit 或 push。

本报告只宣告 worksite 的启动/感知/无控制 EGO 链恢复，不把它扩大为飞行、
绕塔、Observation v2 或训练验收。
