# AstraDroneOpen 项目技术演进与 Learning Speed 阶段汇总

> 文档角色：AstraDroneOpen Learning Speed / SAC 阶段唯一技术演进总索引
>
> 本次整理日期：2026-09-02（Asia/Shanghai）
>
> 当前仓库：`scene01-3uav-circuit-mission`，HEAD `ad857b9816a431a4c64ae46960e18b35ef76b8a3`，工作区 dirty
>
> 本轮增量来源：清点时 `runtime_artifacts/**/*.md` 65 份 + 用户指定旧汇总 3 份，共 68 份；其中原第 21 节已覆盖 41 份 runtime 报告，本轮新增审计 24 份 runtime 报告和 3 份根目录汇总
> 操作边界：只整理、审计和清理 Markdown；没有修改代码、配置或参数，没有 commit，没有启动 ROS、Gazebo、PX4、SAC、training、evaluation 或测试；完整吸收后按用户授权删除 68 份源 Markdown，非 Markdown 原始工件全部保留

第 1～21 节是 2026-08-29 的历史基线索引；第 22～29 节是本轮结合当前源码、8 月 30 日全项目/Reward 审计与 9 月 1 日最新 runtime 的增量裁决。发生冲突时，以第 22～29 节及其引用的当前源码/最新工件为准。工作区仍包含大量未提交用户修改；历史 runtime 的真实 FAIL/NO-GO、基础设施无效尝试和 qualification-only PASS 均原样保留，不因后续修复或新 run 被抹去。

标记约定：`[论文明确要求]`、`[项目原始设计]`、`[AstraDroneOpen实现选择]`、`[为了修复runtime问题后来加入]`、`[qualification-only]`、`[已被替代]`、`[已废弃]`、`[当前仍生效]`、`[状态待审计]`。代码索引另用 `CORE`、`CORRECTNESS_REQUIRED`、`PAPER_ALIGNED`、`ASTRA_IMPLEMENTATION`、`RUNTIME_PATCH`、`QUALIFICATION_ONLY`、`LEGACY_CANDIDATE`、`DELETION_CANDIDATE`、`UNKNOWN`。

## 1. 项目与研究目标

AstraDroneOpen 的飞行基线是 ROS1 Noetic、Gazebo Classic、PX4 SITL、MAVROS、Livox Mid-360、FAST-LIO 与 EGO-Swarm。任务层决定 waypoint、ENTRY/EXIT、八扇区、重试、返航和终态；EGO 负责局部轨迹、碰撞检查与重规划；bridge/traj_server/controller 负责执行。

Learning Speed 的研究动作只有 EGO 动态最大速度约束 `v_max`：

```text
Observation C -> SAC normalized action
 -> SpeedRequestStamped -> SpeedAdapter / SpeedSafetyFilter
 -> SpeedActionStamped -> EGO dynamic v_max
 -> SpeedAppliedStamped -> causal transition / Replay
```

`[论文明确要求]` 外层 learned policy 给 planner 速度约束，不输出 waypoint、轨迹或底层控制；论文明确的频率是 depth 15 Hz、outer policy 10 Hz、position controller 50 Hz，感知与 action 异步、仿真时钟持续推进。`[AstraDroneOpen实现选择]` Hector training-only backend、Observation C 的精确 3267 维、stamped identity、one-open-transition、fixed absolute grid、Forest scheduler、Replay/terminal handshake 均是本项目实现，不应写成论文原始要求。

## 2. 原始飞行系统基线

历史 Git 证据把原始工程追溯到 2026-07-08 的初版，2026-07-30 迁入 official EGO-Swarm core，随后完成三机安全与动态限速接口。当前资料确认两条运行链必须互斥：

```text
full-stack:
Mid360 -> FAST-LIO -> /uavN/Odometry + registered cloud
 -> EGO -> traj_server -> EgoMavrosBridge -> MAVROS/PX4 -> Gazebo

training-only:
Gazebo truth odometry + Mid360 PointCloud2
 -> Observation C -> SAC v_max -> EGO -> traj_server
 -> Hector Pose/Twist controller -> Gazebo
```

`[项目原始设计][当前仍生效]` EGO 的规划、traj_server 的执行、控制器与传感器异步连续运行；D435/YOLO 不进入 Learning Speed 主链。Forest 报告进一步确认 EGO 3D A*、rebound optimizer、FSM periodic/safety replan 仍是局部避障 owner；Mission 2D A* 只用于特定 ENTRY corridor，不接管 Forest。

## 3. Learning Speed 方案形成

早期阶段先验证 dynamic-speed interface 与 fixed speed，再形成以下职责边界：

- `[论文明确要求][PAPER_ALIGNED]` policy 输出 speed constraint，EGO 仍输出轨迹。
- `[AstraDroneOpen实现选择][当前仍生效]` 合法范围内 `requested_v_max == filtered_v_max == applied_v_max`；SafetyFilter 仅 finite/check/clamp，不做 slew、low-pass、hysteresis 或 maximum-step shaping。
- `[AstraDroneOpen实现选择][当前仍生效]` 相邻 action delta 超出 `[-0.3,+0.5] m/s` 时额外 force-replan；范围内保留 EGO native replan。
- `[为了修复runtime问题后来加入][当前仍生效]` request/action/applied 使用 `(episode_id, step_index, request_id)`，scalar topic 只是状态镜像。
- `[已被替代]` 旧 action `[0.05,0.40] m/s`、transition-count normal stop、training 内 evaluation、value/timestamp 猜配和多 pending transition。

正式候选动作范围后来固定为 `[0.30,1.75] m/s`，映射 `v_max=1.025+0.725*action`。下游 capability 与 policy range 被明确分开：SAC 最大 1.75，而 SpeedSafetyFilter/EGO capability 可为 4.0；r01 因错误要求二者相等而在 Episode 0 前失败，r02 修正为“policy interval 被 capability interval 包含”。

## 4. Observation V2 / C / 3200 探针演进

### 4.1 从 V1 到 V2

Observation V1 预留地图 tensor 与低维量，但 EGO 导出无法完整表达 FREE/UNKNOWN，轨迹通道也不足，因而没有成为训练 ready 的正式输入。Observation V2 改成 80×40=3200 个固定球面方向 bin，以五帧 Mid360 点云和各帧 causal pose 构建：

```text
0 = UNKNOWN
1 = observed FREE
2 = known OCCUPIED
```

`[AstraDroneOpen实现选择][当前仍生效]` 这是固定方向 surrogate，不是 EGO inflated map，也不是物体数量或体素体积分数。五帧、FoV、occlusion margin、bin 顺序和 UNKNOWN 编码属于冻结数值合同。

### 4.2 Observation C

Observation C 把 V2 与 EGO 正式轨迹和运动状态融合，冻结 policy input 为 3267 维：

1. `lidar_surrogate[3200]`；
2. `future_positions_body[20][3]`；
3. `actual_velocity_body[3]`；
4. `tracking_error_body[3]`；
5. `previous_applied_v_max`。

`[当前仍生效]` mission/planner state、clearance、complexity labels、lidar masks、diagnostics、run/episode provenance 不进入 policy input。每个字段必须有真实 frame/stamp/source；缺失、未来数据、generation mismatch、无正式 trajectory 或无法构造未来段时 fail closed。

### 4.3 时戳与 trajectory 选择

早期 valid-ratio 问题来自两项根因：replan 后按“当前最新轨迹”而非 V2 source stamp 选轨迹；以及 ROS `sec/nsec -> float -> sec/nsec` 在大 sim time 下丢 1 ns。修复后保留原 0.05 s kinematic interpolation gate，使用 bounded causal trajectory history，并保留原始 sec/nsec。

2026-08-27 又出现不同的 1 ns 问题：同一个 official trajectory 经 Bspline 与 Observation C 序列化后 start stamp 低 1 ns，scheduler 对 `(start_time_float,id)` 精确排序产生 false negative。后续只对同一 monotonic ID 允许 5 ns 容差；低 ID、未来 trajectory 和不单调 start 仍拒绝。两次 1 ns 问题对象不同，不能合并成一个历史事件。

### 4.4 stale、mailbox 与性能

R04 的两次 `invalid_observation:stale` 最初只能确认 coordinator 用 0.35 s wall receipt-age 将 callback gap 升级为环境终态；后续 telemetry 定位为 `PRODUCER_COMPUTE_STALL`，且存在 stale candidate 被新 callback 超越的 TOCTOU。

修复链为：

```text
receipt-only stale terminal
 -> source-aware raw/V2/C classification + latch 前加锁重读
 -> C subscriber callback 改为 O(1) single-slot latest mailbox
 -> worker coalesces obsolete input，reset 清 mailbox/barrier
 -> V2 NumPy decode、缓存 occlusion table、跳过已覆盖 ray
 -> C trajectory sampler 从整条剩余 B-spline 改为固定 5 m policy prefix
```

`[为了修复runtime问题后来加入][当前仍生效]` mailbox/coalescing 保护的是“只算最新样本”；它不是 Reward 或 policy input。V2 优化保持 bitwise-equivalent 3200 值。5 m prefix 来自 `20*0.25 m`：旧逻辑对约 53 m 全尾部递归采样，isolated benchmark 281.0 ms；固定 prefix 19.443 ms，14.45×；5-Episode runtime worker max 69.263 ms，未改 0.35 s gate。

## 5. Reward 设计演进

### 5.1 Stage 1 / Reward v1

旧 `astradrone_stage1_reward_v1.0` 使用项目自定义 `phi_1/phi_2` 与三个 speed branch，并以 applied `v_max` 计算 speed term，缺论文 Eq. (8) tracking-error term。它是 `[AstraDroneOpen实现选择][已被替代]`，不是论文精确复现。

### 5.2 Reward v2

Reward v2 收敛到唯一 owner `training/reward.py::LearningSpeedReward.evaluate()`：

```text
r = r_speed + r_smoothing + r_error + r_danger
```

- `[论文明确要求]` 四分项结构；`r_speed/r_danger` 用 actual speed，`r_smoothing` 用相邻 applied constraint，`r_error` 用 tracking error。
- `[AstraDroneOpen实现选择]` `lambda_error=2.0,e_max=0.40`、其它 lambda、phi 计算及 surrogate 特征。
- `[当前仍生效]` progress、success bonus、clearance、planner shaping 不进 Reward；invalid/truncated 不伪造成 training-ready。

v2 unit/offline/online 复算一致，但 4档×5 Episode fixed runtime 的 4161 step 中 Low=0，85.65% 状态反事实偏低速，结论为 `REWARD V2 RUNTIME QUALIFICATION NO-GO`。后续诊断把根因先分类为 route coverage：Hover→ENTRY 本身是 clutter corridor；并非已有证据证明真实 open 状态被错误映射。

### 5.3 Reward v3 与 v3.1

v3 保留 normalization 与四分项，只把 `phi_2=max(q_N,q_D)` 改为：

```text
phi_2 = 1 - (1-q_N)^0.46 * (1-q_D)^0.54
phi_1 = 1.75 - phi_2
```

三个 branch 改为 quadratic Bernstein 连续权重。`[AstraDroneOpen实现选择]` 0.46/0.54 来自项目真实 strata 的 class-balanced offline fit。v3 获得 unit/frozen replay/landscape PASS，但当时只授权 bounded Reward-only runtime，未授权 SAC training。

Scheme C cleanup 进一步形成 v3.1：`[当前仍生效] UNKNOWN influences POLICY INPUT, not REWARD TARGET`。UNKNOWN 保留在 V2/C；Reward 不读取 `unknown_majority`，也没有 UNKNOWN-specific anchor/penalty。历史 Candidate B/FASTER runtime branch 与 UNKNOWN Reward target 被删除/替代，但相关报告仍作为架构试验史保留。

## 6. dynamic v_max 与 EGO 执行链

当前历史合同为：

```text
Actor normalized action
 -> ActionMapping [0.30,1.75]
 -> SpeedRequestStamped
 -> SpeedAdapter / SpeedSafetyFilter
 -> SpeedActionStamped
 -> EGO dynamic v_max
 -> SpeedAppliedStamped
```

`[CORRECTNESS_REQUIRED][当前仍生效]` request/action/applied identity 必须完全一致，正式 pairing 不按值或时间猜测。`[已被替代]` r01 把 filter capability 当 policy domain；修复后 runner 从 SAC config 构建 mapping，只验证 policy interval 位于 downstream capability 内。

Fixed-v_max 六档最初只有 0.30/0.75 PASS，1.00–1.75 因 trajectory end 后 `trajectory_unavailable` NO-GO。诊断证明 traj_server 仍发布终点 zero-speed command，Observation C 正确拒绝无未来 B-spline；缺失的是 fixed qualification terminal-convergence 语义。`[qualification-only]` 加入严格 hold guard 后 1.00–1.75 顺序重测 8/8 PASS，连同早期 0.30/0.75 形成六档 fixed backend PASS；此补丁不能直接套到 SAC action-owner terminal closure。

## 7. Gazebo RL / Hector / Forest 接入

Hector backend 是 `[AstraDroneOpen实现选择]`：保留 EGO/traj_server，用 Gazebo truth odometry 替代 training FAST-LIO，用薄 adapter 将 PositionCommand 转给 Hector Pose/Twist controller。Reset 顺序包括 terminal closure、candidate validation、controller stop、pause/teleport zero twist、temporal clear、generation barrier、五帧 warm-up、fresh trajectory。

Forest 引入十张冻结 world、logical/raw seed 映射、training seed0–7、evaluation seed8–9、每图 100 completed Episodes、每轮 800 Episodes 的 balanced shuffled schedule。31-Episode smoke 只用 qualification override 每图 10 Episode，正式 default 不变。Map switch 必须删除旧模型、校验 18/18 新模型、0 residual、old absent、清 EGO environment、重建 generation barrier，同时保持 Actor/Critic/optimizer/Replay 连续。

## 8. SAC 训练架构

`[当前仍生效]` 训练组件包括 Gaussian Actor、twin Q/target Q、automatic entropy、Replay、异步 learner、checkpoint、training/evaluation runner。正式候选为：10000 completed Episodes、每 500 Episodes checkpoint、Replay 100000、learning starts 1000 transitions、batch 64、Actor/Critic/alpha LR `1e-5/1e-3/1e-3`、100 critic-only startup updates、log-std `[-3,-1]`、seed 1。

旧 10k pilot 在 4207 valid transitions 因真实 `NO_FEASIBLE_TRAJECTORY` 与 action exploration instability fail closed，禁止 resume。后续短程 2500-transition qualification 只证明数值/identity/checkpoint 的 bounded PASS，不是收敛或长期安全。

`[已被替代]` 2026-08-23 legacy cleanup 当时仍以 10000 transitions 为 stop；随后迁移到 10000 completed Episodes。因此旧报告中的 5000/10000 transition checkpoint 只是历史快照。

## 9. Replay / Actor / transition 状态机演进

### 9.1 multiple-in-flight 的失败

旧 scheduler 把每个 0.1 s tick 当成无条件 publication cadence，允许多个 `PendingCausalStep` 同时等待 ACK、trajectory 和 post-hold Observation。r02 在 terminal cancellation 时产生 `276 missing -> 277 present -> 278 missing -> 279 present`，Replay audit `episode_step_sequence` FAIL。

### 9.2 one-open-transition

`[为了修复runtime问题后来加入][当前仍生效]` 环境一次最多一个 open transition；只有 transition consumer 完成 Replay commit 后，下一 step 才可分配。Replay 二次检查 contiguous `step_index`、generation、唯一 terminal 与 post-terminal 禁止。缺 Observation 的 tick 不分配 step/request。

### 9.3 Actor ownership race

r03 中 runner 在 environment acceptance 前把推理结果记为 open Actor action；terminal lock 随后拒绝 `_begin_pending_step()`，形成 ghost Actor。修复后 Actor inference 只是 candidate，唯一 ownership transfer linearization point 是 environment 在 terminal lock 内真正接受并发布 request。`None` 表示 candidate 被丢弃，不是 open action。

R04 31/31 证明 r02 hole 与 r03 ghost race 未复现：5581 transitions、31 ordered closures、max open=1、holes/post-terminal=0、每 Episode 恰一 terminal；但这不覆盖 R04 的 stale NO-GO。

## 10. 异步 Hz 与 fixed 10 Hz timing 演进

### 10.1 早期异步设计

`[论文明确要求]` 15 Hz perception、10 Hz policy、50 Hz controller，并非全局同步 barrier。Astra 最早 causal step 以 `step.duration=0.1 s` 实现：action/applied/provenance → hold 0.1 s → 再等待 post-hold C → commit → next action。它保证因果但把 0.1 s 变成串行内部下界，R04 effective rate 约 4 Hz。

### 10.2 fixed absolute grid

只读 timing audit 建议、后续实现：

```text
T[k] = T0 + k*0.1 s
at tick:
  latest causal-valid snapshot
  -> close/commit transition[k-1]
  -> terminal recheck
  -> Actor
  -> environment accept/publish action[k]
```

`[为了修复runtime问题后来加入][当前仍生效]` `policy_tick_index` 与 Replay `step_index/request_id` 分离；tick miss 保持旧 action、不新建 ID、不 catch-up、只到下一个绝对 tick 重试。fixed grid 不是 multiple-in-flight。

10-Episode timing q02：pre-learning 9.167 Hz，learner/stochastic 2.925 Hz，总体 4.161 Hz；70% miss 来自 `trajectory_snapshot_provenance`，所以结论仍 NO-GO。后续 provenance 修复后 7-Episode q02 总体 6.016 Hz、learner-dominant 4.916 Hz，明显改善但仍不等于稳定 nominal 10 Hz。

## 11. trajectory provenance 演进

演进顺序：

1. Forest 首 action failure：`begin_external_episode()` 清空 coordinator 已验收的 trajectory history；修复为 generation-aware 保留/重新 readiness。
2. force-replan：需首个 post-action official trajectory；不能用 action 前旧轨迹冒充。
3. Observation C 同时记录 `selected_trajectory`（V2 source stamp 当时 active，供 policy）与 `latest_trajectory_at_lookup`（C 已消费的最新 official trajectory，供 provenance proof）。
4. 旧 scheduler 强制“new trajectory 必须成为 source-time selected trajectory”，把过去的 causal state错误要求成使用未来轨迹；后续允许 latest-at-lookup 证明 C 已消费新轨迹，同时 policy future positions 仍来自 source-time active trajectory。
5. 同 ID start stamp 1 ns round-trip false negative 采用窄 5 ns identity tolerance。
6. near-lifetime gate：action acceptance 前两次检查 selected official trajectory 是否覆盖 request time + 0.1 s + 1 µs；不足则 tick miss，无 request、无 ID、无 fake row。

`[当前仍生效]` future trajectory substitution 仍禁止；force-replan official trajectory receipt、source stamp strictly post-action、C consumption proof与 one-open/Replay ordering均保留。

## 12. Observation stale / producer / 性能演进

完整链条是：R04 receipt-only stale → telemetry 判定 false environment terminal → producer V2/C backlog → mailbox/V2 optimization → q02 stale=0但 provenance 成主瓶颈 → 31 retry r01 在长 53 m trajectory 上再次 producer stall → 5 m prefix修复 → bounded 5 Episode max worker 69.263 ms → 31 retry v2 producer不再失败。

这说明“mailbox已加”不等于所有 producer compute 已有界：single-slot 只能防 FIFO backlog，不能阻止 worker 对单个最新但很长的 B-spline 做整尾计算。该链是后续反向审计的重要补丁链。

## 13. terminal 与 Episode lifecycle 演进

- `terminated`：真实 collision/planner/tracking/environment terminal；`truncated`：max steps/time 等合法外部截断。普通终态必须保留并在 exactly-once terminal row 后继续 reset。
- `infrastructure:*`：先闭合真实 final transition、验证 Replay/Actor/open=0，再在 closure v1.1 中携带 exact fail-closed reason；coordinator ACK 后禁止 reset/map switch/next Episode。
- fixed-only trajectory-end：严格 terminal convergence guard 仅服务 fixed qualification。
- external `max_episode_time` closure：可使用 terminal latch 前已收到且真实 causal 的最新 C，不要求一帧 post-latch C；仍保留 normal provenance，不伪造 Observation。
- true environment terminal：历史修复后仍要求真实 post-latch terminal state；不能把 pre-terminal C 标成终态。
- near-lifetime action prevention：用于避免在 trajectory 将耗尽时再接受一个无法获得合法 `state_t+1` 的 action。
- terminal identity handshake：latest near-end targeted run 中 environment/Replay 已 CLOSED，但 runner 等不到 coordinator 对应 terminal identity，仍判 NO-GO。

## 14. 历次 runtime qualification / 31-Episode / retry 时间线

| 时间 / RUN_ID | 阶段 | 结果与失败原因 | 后续 |
|---|---|---|---|
| 2026-08-23 `sac_training_10k_20260823_194751` | 旧 pilot | 4207 valid transition 后真实 planner failure + exploration instability，NO-GO | 禁止 resume；SAC稳定性与正式 stop合同后改 |
| 2026-08-24 fixed-vmax 首轮 | 六档资格 | 0.30/0.75 PASS；1.00–1.75 `trajectory_unavailable` NO-GO | terminal-convergence诊断后重测 8/8 PASS；仅 fixed-only |
| 2026-08-24 `forest_map_switch_smoke_220ep_...` | Forest scheduler | 首试 path错误；retry Episode 1 首 action无 official provenance，0 completed | 保留失败，先修 provenance |
| 2026-08-24 `forest_provenance_31ep_20260824_r01` | 31 smoke | Episode 1 collision，NO-GO | fixed seed6 根因检查 |
| 2026-08-24/25 seed6 diagnostics | EGO/mapper | EGO真实重规划并执行，但路径沿中心线穿 medium_4；发现 surface-only current-frame map 假 Z corridor | 三态 raycast/log-odds mapper |
| 2026-08-25 `forest_seed6_raw36_shared_mapper_v075_...` | shared mapper | 假 Z corridor消除，出现 lateral A*；near-start guidance/edge sampling仍失败 | swept supercover + short-segment guidance |
| 2026-08-25 swept-guidance run | planner fix | accepted paths snapshot-swept free、tracking/reset PASS，但 UNKNOWN中仍有解析实体 penetration，NO-GO | Candidate B/SUPER/FASTER审计 |
| 2026-08-25 Candidate B/SUPER | frozen A/B | KNOWN_FREE-only 0/4；endpoint/coverage deadlock，NO-GO | 不进入 production |
| 2026-08-25 FASTER-inspired commit run | candidate/committed | retained committed status可工作，但 safe-stop后无 replan、trajectory耗尽，NO-GO | Scheme C随后删除此 production branch |
| 2026-08-26 Scheme C seed6 | native EGO | mission success、final B-spline/UAV安全；初判因 A* analytic penetration FAIL，后重分类为 guidance warning | worksite regression PASS，准许31 smoke |
| 2026-08-26 `forest_31episode_smoke_..._r01` | 31 smoke | Episode 0 bounds equality startup bug | runner containment修复 |
| 2026-08-26 r02 | 31 smoke | 3/31 closed；Episode4 Replay holes / terminal cancellation mismatch | one-open-transition |
| 2026-08-26 r03 | 31 smoke | 6/31 closed；Episode7 ghost Actor ownership | acceptance linearization fix |
| 2026-08-26/27 r04 | 31 smoke | 31/31闭合、3/3 map switch、Replay/Actor/learner PASS；Episodes23/30 stale，最终 NO-GO | stale/producer/timing专项 |
| 2026-08-27 `forest_r04_targeted...t01` | telemetry-only | 定位 producer compute stall + stale TOCTOU；约4 Hz | timing refactor |
| 2026-08-27 `forest_rl_timing_fix...q02` | 10-Episode timing | stale/backlog/V2/fixed-grid局部 PASS；总体4.161 Hz，70% provenance miss；NO-GO | provenance minimal fix |
| 2026-08-27 `forest_trajectory_provenance_fix...q02` | 7-Episode | 2306 rows；r02/r03与truncation语义 PASS；总体6.016 Hz；GO仅针对下一31 qualification | 运行31 timing qualification |
| 2026-08-27 `forest_31episode_timing_qualification...r01` | 31 qualification | 19 closed；Episode20外部 truncation 后 open transition 等不到 post-latch C，closure timeout | external truncation closure fix |
| 2026-08-27 `forest_terminal_closure_fix...q03` | 3-Episode | cached real causal C闭合 external truncation，1233 rows contiguous，GO for retry | 31 retry r01 |
| 2026-08-27 `forest_31episode_retry...r01` | 31 retry | 12 closed；Episode12 producer stall；runner又启动Episode13，operator STOP | 5m prefix + infrastructure no-next-Episode |
| 2026-08-27 `forest_producer_stall_failclosed_bounded5...q01` | 5-Episode | 5/5，producer max69.263ms，infrastructure fail-closed合同通过 | 31 retry v2 |
| 2026-08-27 `forest_31episode_retry_v2...r02` | 31 retry | 18 closed；Episode19 trajectory ended，request321 open，true terminal需post-latch valid C而死锁 | near-lifetime acceptance gate |
| 2026-08-27 `trajectory_end_terminal_fix...r01` | 5-Episode | 5/5普通终态闭合，但未覆盖 near-end branch，NO-GO | qualification relay |
| 2026-08-27 `trajectory_end_near_lifetime...r04` | targeted near-end | 4 tick正确skip并恢复；后续tracking terminal environment已CLOSED，但coordinator terminal identity超时 | 最终仍NO-GO，31未再运行 |

当前时间线终点：**没有一轮后续 31-Episode PASS；没有 100-Episode endurance；没有正式 10000-Episode training；没有 evaluation。**

## 15. legacy cleanup 与已废弃设计

明确已废弃/替代：

- 旧 `sac_training_smoke.yaml`、旧 pilot summarizer、training-internal evaluation 开关；
- 10000-transition normal stop 与 transition checkpoint；
- headerless/value/timestamp pairing；
- multiple pending transitions、runner Actor dictionary、out-of-order ready commit；
- legal action slew/low-pass/hysteresis/cooldown shaping；
- direct Hector `/cmd_vel`、自写第二套 B-spline/PID；
- Candidate B KNOWN_FREE-only production 目标；
- FASTER-inspired H/R/safe-stop/CommittedTrajectoryStore 临时 production branch；
- global `unknown_planning_policy` compatibility branch与 UNKNOWN-specific Reward target；
- fixed-only terminal hold 对 SAC owner 的直接复用。

`[qualification-only]` near-lifetime relay、special launch/topic seam 只用于制造真实 near-end条件，不能进入 production。`DELETION_CANDIDATE` 仅表示后续应审计引用与隔离，本文没有删除任何文件。

## 16. 当前 production contract 快照

依据最新来源、待源码复核的当前快照：

- `[当前仍生效]` Scheme C：EGO native binary planning；UNKNOWN 属于 Observation C policy input，不属于 Reward target。
- `[当前仍生效]` Observation C 五项 3267 维、真实 provenance、future/frame/stamp fail closed。
- `[当前仍生效]` Reward v3.1 唯一 owner；progress 不进 Reward；v3 参数是 Astra implementation。
- `[当前仍生效]` SAC action `[0.30,1.75]` 与 downstream capability 分离；stamped identity唯一配对。
- `[当前仍生效]` fixed absolute 10 Hz grid、no catch-up、独立 `policy_tick_index`、最多一个 open transition。
- `[当前仍生效]` force-replan trajectory receipt + source-time selected/latest-at-lookup双语义 + 窄ns identity容差。
- `[当前仍生效]` latest-sample mailbox、source-aware stale/TOCTOU、V2数值等价优化、5m C trajectory prefix。
- `[当前仍生效]` Replay contiguous、terminal exactly once、Actor ownership after environment acceptance、reset only after closure ACK。
- `[状态待审计]` true terminal near-end与coordinator terminal identity handshake 尚无端到端通过证据。
- `[状态待审计]` 10 Hz在 learner/stochastic窗口仍明显低于 nominal。

## 17. 当前未解决问题

1. near-lifetime targeted run 后 coordinator/runner terminal identity handshake timeout；这是当前最晚的硬阻塞。
2. 31-Episode retry 没有在上述修复后重跑，更没有 PASS。
3. fixed grid 在 pre-learning 可接近9 Hz，learner/stochastic约4.9 Hz；是否满足最终算法时序目标未解决。
4. genuine post-action official trajectory unavailable 仍贡献大量 tick miss；不能靠放宽 provenance掩盖。
5. Forest 普通 planner/collision terminals数量高；它们是环境结果而非基础设施 hard-stop，但意味着没有成功轨迹/训练质量保证。
6. Scheme C 对 UNKNOWN可搜索的安全边界仍依赖 EGO final trajectory与runtime checks；analytic A* guidance warning的长期意义值得源码审计。
7. Reward v3.1 尚无新的正式 SAC runtime qualification；更无长期训练/收敛/evaluation证据。
8. 大量改动处于 dirty workspace，报告所称“当前”需要以现源码逐项反查。

## 18. 技术演进时间轴

| 时间证据 | 阶段 | 原始问题 | 修改 / 涉及源码 | runtime结果 | 替代关系 / 当前状态 |
|---|---|---|---|---|---|
| 2026-07-08 | 初版 | 原始工程基线 | EGO/mission/full-stack | Git历史 | `[项目原始设计]` |
| 2026-07-30 | EGO-Swarm迁入 | 单机核心迁移/三机能力 | `grid_map.cpp`,`ego_replan_fsm.cpp`等 | Git历史；非本轮runtime | `[当前仍生效]` vendor-derived core |
| 2026-08-13 | dynamic v_max | policy到EGO接口 | FSM动态限速、adapter | interface tests | 后续stamped identity强化 |
| 2026-08-15–17 | V2/C | FREE/UNKNOWN/OCCUPIED与trajectory融合 | V2 3200、C 3267、causal history、1ns stamp fix | worksite/postfix validation | `[当前仍生效]`，后续性能重构不改数值 |
| 2026-08-20–23 | calibration/Hector/SAC | training backend、reset、SAC闭环 | truth odom、Hector adapter、Replay/learner | fixed matrix、2500短程；旧pilot NO-GO | 正式training仍未授权 |
| 2026-08-24 | stop/vmax/reward/Forest | 10000 Episode合同、Reward偏置、Forest首次失败 | schedule、v2/v3、Forest assets/provenance | fixed六档最终PASS；Reward v2 NO-GO；Forest NO-GO | v3/v3.1替代v1/v2 production |
| 2026-08-25 | mapper/planner/UNKNOWN探索 | medium_4假通道与near-start失败 | three-state mapper、swept edge、guidance；Candidate B/FASTER trials | 多个seed6 NO-GO | Scheme C删除临时branch |
| 2026-08-26 | Scheme C + r01–r04 | production职责收敛、Replay/Actor races | EGO-native cleanup、one-open、Actor ownership | R04 31/31但stale NO-GO | correctness合同保留 |
| 2026-08-27 上午 | stale/timing | false stale、约4Hz | telemetry、mailbox、V2优化、fixed grid | 10ep NO-GO，provenance瓶颈 | producer局部PASS，timing未完成 |
| 2026-08-27 下午 | provenance/terminal | 1ns与selected gate、external truncation closure | `astra_drone_env.py` provenance与terminal分类 | 7ep/3ep bounded PASS | 允许retry，不等于31 PASS |
| 2026-08-27 晚 | retry/producer/near-end | producer long-trajectory stall、Episode19 deadlock | 5m prefix、closure v1.1、lifetime gate | retry 12/31、18/31；near-end identity timeout | 最新状态NO-GO |

除文档明确 Date/RUN_ID 外，时间排序也参考报告之间的前序/后续引用。所有来源 birth time 均不可得，统一标记 `TIME_UNCERTAIN`；mtime只作辅助，绝不等同于代码首次加入时间。

## 19. 代码历史追溯索引

以下“报告次数”是 51 份来源中按文件名出现的文档数，只用于判断历史讨论密度，不等同于 Git commit 次数。

### 19.1 高频核心文件

| 源码 | 报告次数 | 演进与关键逻辑 | 当前标记 / 后续审计 |
|---|---:|---|---|
| `learning_speed_rl/training/astra_drone_env.py` | 7 | causal step、multiple-in-flight→one-open、fixed grid、policy_tick、provenance selected/latest、1ns tolerance、terminal closure、lifetime gate | `CORE CORRECTNESS_REQUIRED RUNTIME_PATCH`；最高优先级 |
| `hector_ego_training_backend/scripts/training_episode_reset_coordinator.py` | 8 | Episode identity、terminal/reset、stale 0.35s、TOCTOU、infrastructure fail-closed、map switch | `CORE CORRECTNESS_REQUIRED RUNTIME_PATCH` |
| `learning_speed_rl/config/sac_training_v1.yaml` | 8 | action范围、10000 Episodes、checkpoint、Forest/timing qualification gate | `CORE ASTRA_IMPLEMENTATION`；检查qualification字段是否隔离 |
| `learning_speed_rl/scripts/sac_training_runner.py` | 5 | Actor ownership、bounds containment、Replay/learner、closure ACK、Forest schedule | `CORE CORRECTNESS_REQUIRED RUNTIME_PATCH` |
| `Planner/.../plan_env/src/grid_map.cpp` | 5 | upstream surface-only→三态raycast/log-odds/XYZ inflation、environment clear | `CORE RUNTIME_PATCH`；与full-stack共享语义需审计 |
| `Planner/.../path_searching/src/dyn_a_star.cpp` | 5 | 26-neighbor A*、swept supercover、UNKNOWN policy历史 | `CORE CORRECTNESS_REQUIRED` |
| `learning_speed_rl/training/reward.py` | 4 | Reward v1→v2→v3/v3.1唯一owner、UNKNOWN removal | `CORE PAPER_ALIGNED ASTRA_IMPLEMENTATION` |
| `Planner/.../bspline_opt/src/bspline_optimizer.cpp` | 4 | rebound guidance、near-start short segment、统一collision verdict；临时commit branch历史 | `CORE RUNTIME_PATCH` |
| `Planner/.../plan_manage/src/planner_manager.cpp` | 4 | dynamic v_max、candidate/committed临时链、final publication checks | `CORE RUNTIME_PATCH LEGACY_CANDIDATE` |
| `Planner/.../plan_manage/src/ego_replan_fsm.cpp` | 3 | dynamic v_max、force-replan、PlannerStatus/retained committed历史 | `CORE CORRECTNESS_REQUIRED LEGACY_CANDIDATE` |

Git 已提交历史显示：`astra_drone_env.py` 在 2026-08-23～24 的 `37260cb/5a69412/b1e1614/5ec45c6/ad857b9` 演进；`observation_c_node.py`/`observation_v2_node.py` 可追到 2026-08-16/17 的 `214de22/1164b3d`；`ego_replan_fsm.cpp` 的 dynamic v_max 可追到 2026-08-13 `e7e1798`。2026-08-25～27 的大量修复仍在 dirty workspace，不能伪造 commit 对应关系。

### 19.2 其它关键文件与符号

| 文件 / 符号 | 历史责任 | 标签 | 审计问题 |
|---|---|---|---|
| `observation_c_node.py::_lidar_callback/_process_lidar` | C mailbox、worker、provenance lookup、publish telemetry | `CORE RUNTIME_PATCH` | mailbox、barrier与shutdown是否有竞态 |
| `ObservationCBuilder` / `TrajectorySampler._adaptive_polyline` | 20×0.25m future path；整尾→5m prefix | `CORRECTNESS_REQUIRED RUNTIME_PATCH` | prefix exact-equivalence与极端曲率 |
| `observation_v2_node.py`、`v2/unknown_estimator.py`、`pointcloud_decode.py` | 3200 bins、五帧、decode/occlusion优化 | `CORE ASTRA_IMPLEMENTATION` | bitwise合同、CPU余量 |
| `ObservationC.msg` | 五项值+source/queue/fusion诊断 | `CORE CORRECTNESS_REQUIRED` | 诊断字段不得进入policy/reward |
| `training/sac_replay.py::SacReplayBuffer.add` | contiguous step、generation、terminal/post-terminal门 | `CORE CORRECTNESS_REQUIRED` | 与runner closure双owner边界 |
| `training/actor_action_ownership.py` | inferred candidate到accepted action线性化 | `CORRECTNESS_REQUIRED RUNTIME_PATCH` | terminal并发 |
| `training/formal_training_contract.py` | 10000 Episode、checkpoint allowlist、evaluation isolation | `CORE ASTRA_IMPLEMENTATION` | 旧transition stop残留 |
| `training/forest_schedule.py::BalancedForestMapScheduler` | seed0–7、100/block、800/round、snapshot/restore | `CORE ASTRA_IMPLEMENTATION` | 与checkpoint/resume；resume当前未实现 |
| `forest_map_manager.py` / `forest_map_contract.py` | hot-swap、18/18、old absent、map_ready | `CORE CORRECTNESS_REQUIRED` | shutdown与partial switch |
| `episode_reset_contract.py` | source-aware stale、reset generation、infrastructure分类 | `CORE RUNTIME_PATCH` | receipt/source/TOCTOU |
| `speed_adapter` / `SpeedSafetyFilter` | finite+clamp、stamped action、capability范围 | `CORE PAPER_ALIGNED` | scalar mirror不得重新成为pair owner |
| `planner_manager.cpp::reboundReplan` | EGO candidate、publication；临时commit逻辑曾在此 | `CORE LEGACY_CANDIDATE` | Scheme C后是否完全清除FASTER残留 |
| `grid_map.h/.cpp::isSweptSegmentFree/isSweptPolylineFree` | same-snapshot swept supercover | `CORRECTNESS_REQUIRED RUNTIME_PATCH` | planner/optimizer/safety是否同一snapshot |
| `short_segment_guidance.h` | CP2→CP3 near-start guidance | `RUNTIME_PATCH LEGACY_CANDIDATE` | 是否属于结构修复还是局部补丁 |
| `trajectory_collision_checker.h` | optimizer/publication/safety verdict统一 | `CORRECTNESS_REQUIRED RUNTIME_PATCH` | 重复collision owner风险 |
| `trajectory_lifetime_qualification_relay.py` | 延迟policy-side真实Bspline制造near-end | `QUALIFICATION_ONLY DELETION_CANDIDATE` | production launch不可引用 |
| `trajectory_end_near_lifetime_forest_qualification.launch` | targeted harness | `QUALIFICATION_ONLY DELETION_CANDIDATE` | 完成审计后是否保留测试资产 |
| `RL_legacy` 删除的 smoke YAML / pilot summarizer | 旧训练入口 | `DELETION_CANDIDATE`（历史已删除） | 只检查无引用，不恢复 |

### 19.3 ROS topic / message / state machine 索引

- topics：`/uav1/learning_speed/{speed_request_stamped,speed_action_stamped,speed_applied_stamped,observation_c}`、`/uav1/planning/bspline`、`/uav1/planner/status`、`/uav1/Odometry`、`/uav1/training/episode_identity`、Forest `map_ready`/assignment topics。
- messages/fields：`episode_id,step_index,request_id`；C 的 selected/latest trajectory id/start/end、source/receipt/generation、queue/fusion diagnostics；closure v1.1 的 `fail_closed_after_closure/fail_closed_reason`。
- state machines：Environment `IDLE/ACTIVE/TERMINATING/CLOSED`；EGO `GEN_NEW_TRAJ/REPLAN_TRAJ/EXEC_TRAJ/WAIT_TARGET/EMERGENCY_STOP`；coordinator terminal→closure ACK→reset/map switch。
- config：`policy_period_sec=0.1`；旧 `step.duration_sec=0.1` hold 已删除；stale threshold 0.35 s未放宽；lifetime floor 0.100001 s；Forest smoke block 10仅qualification override。

## 20. 后续架构反向审计入口

只建议审计，不在本文下架构结论：

1. **Observation C—trajectory—Replay—fixed grid—terminal耦合链**：source-time selected、latest-at-lookup、post-action official、lifetime、terminal snapshot分别由谁拥有；是否能减少重复gate而不弱化因果。
2. **补丁链 A：multiple-in-flight → one-open → fixed grid → provenance mode → lifetime gate**。核对每个补丁是否仍解决独立不变量，是否有已失去前提的条件。
3. **补丁链 B：receipt stale → TOCTOU → mailbox → V2优化 → 5m prefix → infrastructure closure v1.1**。区分 freshness、compute budget、terminal classification与schedule stop四种责任。
4. **补丁链 C：surface-only map → three-state mapper → swept edge → short-segment guidance → Candidate B/FASTER → Scheme C cleanup**。确认临时 candidate/committed/UNKNOWN参数与status残留是否彻底退出 production。
5. **terminal identity双owner**：environment已CLOSED而coordinator不发matching closure ACK的最新失败；审计 terminal source、identity、超时与shutdown顺序。

建议顺序：先做静态 call graph/launch expansion与dirty diff attribution；再做历史补丁 dead-code/duplicate-owner candidate清单；只有显式授权后才做最小代码调整和 bounded runtime。不要用参数调优、fallback、seed skip或放宽因果门制造 PASS。

## 21. 原始文档来源清单

统计：`runtime_artifacts` 发现 41；指定文件找到 10/10；去重后纳入 51；`NOT_FOUND=0`。以下绝对路径均成功纳入。文档 Date 优先于 mtime；`RUN_ID=无显式主RUN` 表示报告未声明自身主运行，不代表正文没有引用历史 run。birth time 全部不可得，统一为 `TIME_UNCERTAIN`。

| 完整路径 | 文件名 | Date / 主 RUN_ID | mtime / birth | SHA-256 | 技术阶段 | 纳入 |
|---|---|---|---|---|---|---|
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/astar_ego_obstacle_avoidance_audit.md` | `astar_ego_obstacle_avoidance_audit.md` | 2026-08-24 / `forest_provenance_31ep_20260824_r01` | 2026-08-24 22:18:08 / TIME_UNCERTAIN | `4207a4b7131bf8764fe0ff42c4f17e7046e347a14f9462c7b359709a02062a41` | A*/EGO只读审计 | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/candidate_b_final_31ep_readiness_report.md` | `candidate_b_final_31ep_readiness_report.md` | 2026-08-25 / frozen `forest_seed6_raw36_swept_guidance_v075_20260825_r02` | 2026-08-25 21:00:19 / TIME_UNCERTAIN | `7ae7f735df3ce62b6ddc61921e81a21c281779e488520ee5764daa1f10329e13` | Candidate B NO_PATH | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/ego_faster_committed_safe_stop_and_speed_chain_31ep_gate.md` | `ego_faster_committed_safe_stop_and_speed_chain_31ep_gate.md` | 2026-08-25 / `forest_seed6_raw36_faster_commit_v075_20260825_r01` | 2026-08-26 00:00:28 / TIME_UNCERTAIN | `8ea7483e35390e140ab7ab93f127857abe9014837e41842aa53177206c63e678` | 临时FASTER-style | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/ego_gridmap_directcloud_raycast_upstream_comparison_audit.md` | `ego_gridmap_directcloud_raycast_upstream_comparison_audit.md` | 2026-08-25 / 无显式主RUN | 2026-08-25 01:13:29 / TIME_UNCERTAIN | `a78fb72d6a7041c8ff1de7e5885727d23f93e29600bd3f41c72a994538897192` | mapper历史审计 | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/ego_nearstart_swept_collision_fix_report.md` | `ego_nearstart_swept_collision_fix_report.md` | 2026-08-25 / `forest_seed6_raw36_swept_guidance_v075_20260825_r01` | 2026-08-25 17:26:27 / TIME_UNCERTAIN | `b8514f88b668e53e8a08712d786aaf10415efb19473a9a8594c5d8c214827619` | swept/guidance修复 | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/ego_shared_mid360_robust_pointcloud_mapping_report.md` | `ego_shared_mid360_robust_pointcloud_mapping_report.md` | 2026-08-25 / `forest_seed6_raw36_shared_mapper_v075_20260825_r01` | 2026-08-25 12:24:18 / TIME_UNCERTAIN | `315fdfaba5530609ced3e383dc6c47b11de963bfece783991b54024f35f358b0` | 三态共享mapper | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/ego_swept_guidance_seed6_runtime_qualification_report.md` | `ego_swept_guidance_seed6_runtime_qualification_report.md` | 2026-08-25 / `forest_seed6_raw36_swept_guidance_v075_20260825_r02` | 2026-08-25 18:45:23 / TIME_UNCERTAIN | `699e59ee7ad713cf872a89570d58d7ee2eb73308881d44c3499fd5bd52eb4408` | seed6 runtime | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/forest_31episode_retry_report.md` | `forest_31episode_retry_report.md` | 2026-08-27 / `forest_31episode_retry_20260827_r01` | 2026-08-27 18:07:35 / TIME_UNCERTAIN | `d44b70aa3998eccb2f7ec6f90de6f46a6a4f796e350ef5662de7e0cdcbf7530d` | 31 retry producer fail | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/forest_31episode_retry_v2_failure_summary.md` | `forest_31episode_retry_v2_failure_summary.md` | 2026-08-27 / `forest_31episode_retry_v2_20260827_r02` | 2026-08-27 19:35:51 / TIME_UNCERTAIN | `19ae4f5685df83b426089c1d4344101ed4cb960b1e714688206b5b02cd2dac12` | retry v2摘要 | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/forest_31episode_retry_v2_report.md` | `forest_31episode_retry_v2_report.md` | 2026-08-27 / `forest_31episode_retry_v2_20260827_r02` | 2026-08-27 19:33:50 / TIME_UNCERTAIN | `5c7353f98067d9315fcf2cc7ee83da216f8fe48250b2552bea6653ccd1dbc046` | retry v2 lifecycle fail | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/forest_31episode_smoke_r02_report.md` | `forest_31episode_smoke_r02_report.md` | 2026-08-26 / `forest_31episode_smoke_20260826_r02` | 2026-08-26 21:35:52 / TIME_UNCERTAIN | `8ad3cbd23647988b6e69f5d36aff8a2e52edfb36072c6158985ebe7252ddaf07` | Replay holes | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/forest_31episode_smoke_r03_report.md` | `forest_31episode_smoke_r03_report.md` | 2026-08-26 / `forest_31episode_smoke_20260826_r03` | 2026-08-26 22:50:03 / TIME_UNCERTAIN | `e060966df18ad6a88cade14d7e2d7c8d0e05ac9406e904f2365dde76b07769e2` | Actor race | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/forest_31episode_smoke_r04_analysis_handoff.md` | `forest_31episode_smoke_r04_analysis_handoff.md` | 2026-08-26–27 / `forest_31episode_smoke_20260826_r04` | 2026-08-27 00:05:19 / TIME_UNCERTAIN | `8cfb3cfd15f8c4ab3ccf8b0b8b508f56746f9687e1dfde5332724978efdfc1a9` | R04 handoff | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/forest_31episode_smoke_r04_report.md` | `forest_31episode_smoke_r04_report.md` | 2026-08-26–27 / `forest_31episode_smoke_20260826_r04` | 2026-08-27 00:05:19 / TIME_UNCERTAIN | `2391f2acd72a3fbfe7175c1ff16c77a75c80a9f2fdf1b2aacbec55e3f13974bb` | 31/31 stale NO-GO | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/forest_31episode_smoke_report.md` | `forest_31episode_smoke_report.md` | 2026-08-26 / `forest_31episode_smoke_20260826_r01` | 2026-08-26 21:09:19 / TIME_UNCERTAIN | `d1f4b2bee99f64ea550be9eac8b3ac4668c6577dc2d770eaa4447dcc8c75a4e9` | bounds startup fail | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/forest_31episode_timing_qualification_fail_summary.md` | `forest_31episode_timing_qualification_fail_summary.md` | 2026-08-27 / `forest_31episode_timing_qualification_20260827_r01` | 2026-08-27 15:52:02 / TIME_UNCERTAIN | `2cf65281a8a4c6a872384c84805a151c3ca358cb1a1ecf26cd65129ea8f000a6` | closure fail摘要 | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/forest_31episode_timing_qualification_handoff.md` | `forest_31episode_timing_qualification_handoff.md` | 2026-08-27 / `forest_31episode_timing_qualification_20260827_r01` | 2026-08-27 13:51:53 / TIME_UNCERTAIN | `4f60cba6f76af104dc1285eeded3ecea5f5a4fbc3dbcf3694dd433cd76cd4c54` | closure handoff | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/forest_31episode_timing_qualification_report.md` | `forest_31episode_timing_qualification_report.md` | 2026-08-27 / `forest_31episode_timing_qualification_20260827_r01` | 2026-08-27 13:52:34 / TIME_UNCERTAIN | `b2dafce42a753f49fcfecfa456c27366f505ec5839b5d30a934c1f75b1dd9f3c` | external truncation closure | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/forest_ego_previous_vs_current_avoidance_rootcause_report.md` | `forest_ego_previous_vs_current_avoidance_rootcause_report.md` | 2026-08-24 / `forest_astar_rebound_internal_seed6_raw36_v075_20260824_retry03` | 2026-08-24 23:38:29 / TIME_UNCERTAIN | `d0cf0e966564fdf0e16579d0f08dc57856f4b9871efd01a80be5ecd4b844a9aa` | old/current EGO对比 | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/forest_ego_replan_execution_chain_audit.md` | `forest_ego_replan_execution_chain_audit.md` | 2026-08-24 / `forest_ego_replan_execution_chain_seed6_raw36_v075_20260824_r01` | 2026-08-24 23:14:54 / TIME_UNCERTAIN | `3dc04c50c8ad50cacecb069ba0a35184d9558887ce43af8690ee1b9c3122b813` | execution chain | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/forest_medium4_false_vertical_free_corridor_rootcause.md` | `forest_medium4_false_vertical_free_corridor_rootcause.md` | 2026-08-25 / 无显式主RUN | 2026-08-25 00:14:53 / TIME_UNCERTAIN | `066e522618bc2a783ac2438c9462c523a2304e0c474811967a40ea599509e826` | 假Z通道 | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/forest_newmapper_lateral_rebound_optimizer_rootcause.md` | `forest_newmapper_lateral_rebound_optimizer_rootcause.md` | 2026-08-25 / `forest_seed6_raw36_shared_mapper_v075_20260825_r01` | 2026-08-25 16:48:31 / TIME_UNCERTAIN | `82950a02cfc0ec82163daff81500bd5b36c39d50771700af68ec20112c7e7775` | rebound根因 | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/forest_random_map_scheduler_preflight_report.md` | `forest_random_map_scheduler_preflight_report.md` | 2026-08-26 / `forest_31ep_preflight_20260826_r02` | 2026-08-26 21:07:27 / TIME_UNCERTAIN | `001989d5c772e79311c273656e79213f539f8f287e4b786b7d3a4cddd2ed2b9e` | scheduler preflight | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/forest_randomization_training_integration_v2_report.md` | `forest_randomization_training_integration_v2_report.md` | TIME_UNCERTAIN / `forest_map_switch_smoke_220ep_retry01_20260824_190500` | 2026-08-24 19:24:11 / TIME_UNCERTAIN | `6195f362804728a3f86314ba9f949a4da046793e05dbffcd81fb94b92e65d63f` | Forest集成 | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/learning_speed_scheme_c_ego_native_unknown_observation_cleanup_report.md` | `learning_speed_scheme_c_ego_native_unknown_observation_cleanup_report.md` | 2026-08-26 / 无显式主RUN | 2026-08-26 01:55:49 / TIME_UNCERTAIN | `cdead18d415cf37a02a6b42d4f1f036e0aee951a84ff8f29819ba0a9af526ca4` | Scheme C cleanup | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/producer_stall_failclosed_fix_report.md` | `producer_stall_failclosed_fix_report.md` | 2026-08-27 / `forest_producer_stall_failclosed_bounded5_20260827_q01` | 2026-08-27 18:35:53 / TIME_UNCERTAIN | `e7b3c67c6f111937960a1df4e23387f725620e5351559dc64fb6cad3c93b8119` | 5m prefix/infra closure | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/r04_stale_and_policy_rate_final_rootcause_report.md` | `r04_stale_and_policy_rate_final_rootcause_report.md` | 2026-08-27 / `forest_r04_targeted_logical4_raw23_20260827_t01` | 2026-08-27 01:26:42 / TIME_UNCERTAIN | `fefe0a82f4e0f35b2c9e1e3aac0af37d3e16f4081cc0502891ecb5864d4058e1` | stale最终根因 | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/r04_stale_observation_policy_rate_rootcause_report.md` | `r04_stale_observation_policy_rate_rootcause_report.md` | 2026-08-27 / `forest_31episode_smoke_20260826_r04` | 2026-08-27 00:50:41 / TIME_UNCERTAIN | `71718bf5dee7a40c8861cfeb6874f4561115ab510777a323e44fe43cf04ae18c` | stale只读诊断 | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/retained_committed_safe_stop_seed6_requalification_report.md` | `retained_committed_safe_stop_seed6_requalification_report.md` | 2026-08-26 / `forest_seed6_raw36_retained_commit_v075_20260826_r02` | 2026-08-26 00:47:06 / TIME_UNCERTAIN | `b0df7e6ea5ea763dc6a165dd7fbbef23a561db14c4b65f8522977014fe87773c` | safe-stop requal | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/rl_timing_architecture_reference_audit.md` | `rl_timing_architecture_reference_audit.md` | 2026-08-27 / 无显式主RUN | 2026-08-27 11:44:03 / TIME_UNCERTAIN | `a3602539d27468207ad548f8d378058b0fa82330a0a202a0900e7f1dccb44c38` | timing参考审计 | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/rl_timing_targeted_fix_qualification_report.md` | `rl_timing_targeted_fix_qualification_report.md` | 2026-08-27 / `forest_rl_timing_fix_logical4_raw23_20260827_q02` | 2026-08-27 12:42:33 / TIME_UNCERTAIN | `e89e8a5903eb9e766d2f2b0fec03b5e9e1a67cbca2f8f61f1c28c4f0a67d2303` | fixed grid q02 | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/rogmap_super_unknown_planning_reference_audit.md` | `rogmap_super_unknown_planning_reference_audit.md` | 2026-08-25 / frozen seed6 r02 | 2026-08-25 19:44:43 / TIME_UNCERTAIN | `1db429233d89617b9d8722c65a85ff384b9a73cd504405af9da0750a07a23d23` | ROG/SUPER审计 | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/sac_actor_action_acceptance_race_fix_report.md` | `sac_actor_action_acceptance_race_fix_report.md` | 2026-08-26 / 无runtime | 2026-08-26 23:19:41 / TIME_UNCERTAIN | `1e4bd5d2e5a2ae4fdb9be8631c1b5abb35a927ea06c828cf639766d4947f839a` | Actor race修复 | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/sac_sequential_transition_scheduler_fix_report.md` | `sac_sequential_transition_scheduler_fix_report.md` | 2026-08-26 / 无runtime | 2026-08-26 22:15:35 / TIME_UNCERTAIN | `2c109e39f0a31c9bf1ec7b5e30a35c01415e3cf6bc5216fb1280fbce34f1b307` | one-open修复 | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/scheme_c_seed6_reclassification_worksite_31ep_gate_report.md` | `scheme_c_seed6_reclassification_worksite_31ep_gate_report.md` | 2026-08-26 / seed6 r01 + worksite r01 | 2026-08-26 20:36:05 / TIME_UNCERTAIN | `3ec425b5d75ecc009d9639baf34ac353174f1d47b44eb973b77a85a66fa46a9c` | Scheme C gate | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/scheme_c_seed6_worksite_runtime_qualification_report.md` | `scheme_c_seed6_worksite_runtime_qualification_report.md` | 2026-08-26 / `scheme_c_seed6_raw36_v075_20260826_r01` | 2026-08-26 19:58:28 / TIME_UNCERTAIN | `c59d4fd0ca52b7ec7ca075e9cbae13c68cb0eab1be559385ea0393844a2e98b5` | Scheme C runtime | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/super_end_to_end_unknown_endpoint_reference_and_31ep_gate.md` | `super_end_to_end_unknown_endpoint_reference_and_31ep_gate.md` | 2026-08-25 / frozen seed6 r02 | 2026-08-25 21:33:07 / TIME_UNCERTAIN | `4f0432355d7ef1ced298301ce6cbcc830745ee1c7c7e2b78e774d5e745680216` | SUPER shadow | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/terminal_transition_closure_fix_report.md` | `terminal_transition_closure_fix_report.md` | 2026-08-27 / `forest_terminal_closure_fix_logical1_raw10_20260827_q03` | 2026-08-27 17:35:59 / TIME_UNCERTAIN | `a352dcbb1b1b05e9c90d7a288fcda7ed00cdac3d2b6ee795ba2cfa554757bedb` | terminal closure | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/trajectory_end_near_lifetime_runtime_qualification.md` | `trajectory_end_near_lifetime_runtime_qualification.md` | 2026-08-27 / `trajectory_end_near_lifetime_q_1ep_20260827_r04` | 2026-08-27 20:55:52 / TIME_UNCERTAIN | `7db774b861c81bbb9ac9e3078b192e1cb2c970d2f445c41e038e958dd0440fde` | near-end runtime | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/trajectory_end_terminal_lifecycle_fix_report.md` | `trajectory_end_terminal_lifecycle_fix_report.md` | 2026-08-27 / `trajectory_end_terminal_fix_bounded_5ep_20260827_r01` | 2026-08-27 20:36:47 / TIME_UNCERTAIN | `4eaf0ef5d2aef7699bdc61c2bdc00d8e368ba1344f86794d946fafe725e9e5f7` | lifetime gate | YES |
| `/home/yanzu/AstraDroneOpen/runtime_artifacts/learning_speed/trajectory_provenance_timing_fix_report.md` | `trajectory_provenance_timing_fix_report.md` | 2026-08-27 / `forest_trajectory_provenance_fix_logical4_raw23_20260827_q02` | 2026-08-27 13:18:59 / TIME_UNCERTAIN | `eb977c09cae7576059a5033610c149518f95902c86b5ec9637f25ee5c1d5eaa3` | provenance timing | YES |
| `/home/yanzu/AstraDroneOpen/LearningSpeed_phi_branch_distribution_diagnosis_report.md` | `LearningSpeed_phi_branch_distribution_diagnosis_report.md` | 2026-08-24 / offline | 2026-08-24 13:06:10 / TIME_UNCERTAIN | `4d79c72a3bc35fe4acd5a0471345ff61fa8c016b27c27fc6e4f23c154cb4c6c3` | phi route诊断 | YES |
| `/home/yanzu/AstraDroneOpen/LearningSpeed_reward_v2_paper_alignment_report.md` | `LearningSpeed_reward_v2_paper_alignment_report.md` | 2026-08-24 / offline，引用旧10k | 2026-08-24 02:31:59 / TIME_UNCERTAIN | `8a6f5ba726e287c87aa0c9b9f101edf64a2324070ce0f950e0c2512de85eaf5e` | Reward v2 | YES |
| `/home/yanzu/AstraDroneOpen/LearningSpeed_reward_v2_runtime_qualification_report.md` | `LearningSpeed_reward_v2_runtime_qualification_report.md` | 2026-08-24 / q03–q06多RUN | 2026-08-24 12:40:51 / TIME_UNCERTAIN | `e67d0c054057bb677f08d393916ea6021f463ae6460eb60b3375207643c97e7d` | Reward v2 runtime | YES |
| `/home/yanzu/AstraDroneOpen/LearningSpeed_reward_v3_optimization_report.md` | `LearningSpeed_reward_v3_optimization_report.md` | 2026-08-24 / offline | 2026-08-24 14:18:59 / TIME_UNCERTAIN | `02eb43e5460198b762b0ad5ebf96c2522d86f05f6a96975a301e87d08f5b9297` | Reward v3 | YES |
| `/home/yanzu/AstraDroneOpen/ego_faster_unknown_planning_execution_contract_audit.md` | `ego_faster_unknown_planning_execution_contract_audit.md` | 2026-08-25 / 只读 | 2026-08-25 22:41:36 / TIME_UNCERTAIN | `54ad26e2afe5e5aeb8dd947a6c95b367e972b75179152311dedb40453f2b3a70` | EGO/FASTER审计 | YES |
| `/home/yanzu/AstraDroneOpen/Forest_Episode1_collision专项根因检查报告.md` | `Forest_Episode1_collision专项根因检查报告.md` | 2026-08-24 / `forest_provenance_31ep_20260824_r01` | 2026-08-24 21:58:02 / TIME_UNCERTAIN | `ba79c963052351737a4e1adb222169b72254a1906d998bcda2c06b4d3bc69f0c` | Episode1 collision | YES |
| `/home/yanzu/AstraDroneOpen/Forest_SAC_runtime阻塞修复与31Episode_smoke总结.md` | `Forest_SAC_runtime阻塞修复与31Episode_smoke总结.md` | 2026-08-24 / `forest_provenance_31ep_20260824_r01` | 2026-08-24 21:22:13 / TIME_UNCERTAIN | `536dd62d9dfa4345554a968d110411e93a0f8ea727e94426e9b7920693bdb58b` | provenance/31 smoke | YES |
| `/home/yanzu/AstraDroneOpen/RL_legacy_code_cleanup_report.md` | `RL_legacy_code_cleanup_report.md` | 2026-08-23 / 无runtime | 2026-08-23 23:28:19 / TIME_UNCERTAIN | `2cf5256433e7df4134c5be4ed48e94f3b641b6e8575847d392fa6cddd24bce0a` | legacy cleanup | YES |
| `/home/yanzu/AstraDroneOpen/SAC_training_parameter_and_vmax_qualification_report.md` | `SAC_training_parameter_and_vmax_qualification_report.md` | 2026-08-24 / fixed多RUN | 2026-08-24 01:57:58 / TIME_UNCERTAIN | `b1ec258f5e823d4160b6ddfb3b7db5acfacc8e261ef7eee6e9291c913a90accc` | stop/vmax资格 | YES |
| `/home/yanzu/AstraDroneOpen/trajectory_terminal_timing_diagnosis_report.md` | `trajectory_terminal_timing_diagnosis_report.md` | 2026-08-24 / fixed诊断与重测 | 2026-08-24 01:57:19 / TIME_UNCERTAIN | `54e91ddda99909864154a5b4cdb605286010310cd18c72ce79a06cccb15ca7dd` | fixed terminal timing | YES |

### 完整性结论

```text
runtime_artifacts MD discovered = 41
specified exact files found     = 10 / 10
unique sources included         = 51
NOT_FOUND                       = 0
source MD deleted               = 0
code/config/parameter changes   = 0
commit                          = 0
Gazebo/SAC/training/evaluation  = NOT RUN
```

## 22. 2026-09-02 全项目事实边界

### 22.1 AstraDroneOpen 主系统与 Learning Speed 的关系

当前可成立的 AstraDroneOpen 主体成果仍是 ROS1 Noetic + Gazebo Classic + 三套 PX4 SITL/MAVROS + Mid-360/FAST-LIO + 项目适配 EGO-Swarm 的三机 3 m 同高度低空绕塔系统。任务层负责动态 PRE_ENTRY/ENTRY_GATE/ORBIT_STAGING、8 个固定扇区、独立 EXIT_GATE、有限重试、返航和终态；EGO-Swarm 负责本机局部 B-spline、静态障碍规避与公共 `world` 中的带时间轨迹交换；manager/safety 负责 UAV3→UAV2→UAV1 放行、相位/HOLD、预测安全和落地许可。历史完整闭环证据包含三机各自 360°、8 扇区、EXIT、逆真实进场路线返航、HOME、接地和解除武装。

以下边界仍必须保留：

- 历史三机 `2/3/4 m` 分层方案因相邻仅 1 m、违反 EGO-Swarm 同 XY 椭球约束而在起飞前 NO-GO；不能写成已完成模式。已验证的分层能力是双机 `34→30 m` 与 `28→24 m`，并行层差 6 m、每机换层 4 m。
- 当前三机目标相邻相位是 `67.5°`，实际放行窗口为 `65°～70°`，不是恒定刚性编队。理论 RGB 方位并集约 `204.39°/56.78%`，不是 YOLO 实测覆盖率。
- D435/YOLO 是独立只读感知链，不参与 Mid360→FAST-LIO→EGO 规划，也不控制任务或飞行；三机绕塔中的非空业务识别闭环仍未验收。
- 三机 full-stack 与 Learning Speed training-only 是不同证据层。前者为 PX4/FAST-LIO/MAVROS 执行链；后者为单机 Forest/Hector/Gazebo truth 链。二者不得同时发布 `/uav1/Odometry` 或执行控制，也不能把 training-only 结果外推为三机/full-stack PASS。

### 22.2 两条当前运行链

```text
full-stack:
Mid360 + IMU -> FAST-LIO -> registered cloud / odometry
 -> EGO-Swarm -> traj_server -> EgoMavrosBridge -> MAVROS/PX4 -> Gazebo

training-only:
Gazebo truth odometry + Mid360 15 Hz
 -> truth registration + persistent EGO GridMap
 -> Observation V2/C -> SAC dynamic v_max
 -> EGO -> traj_server -> Hector controller -> Gazebo
```

当前 EGO PointCloud2 路径已由 Astra 的 ray HIT/MISS、log-odds、FREE/UNKNOWN/OCCUPIED、局部窗口累计与引用计数 inflation 扩展为跨帧 occupancy owner；V2 则独立保存五帧点云并按 causal pose 对齐。二者是不同的时间抽象，不存在“FAST-LIO 移除后只剩单帧、无 mapping owner”的结论。这个 mapping 架构在源码层合理，但 8 月 30 日审计本身没有产生新的 runtime PASS。

## 23. 当前 Learning Speed / Reward / SAC production contract

### 23.1 Observation C：冻结的 3267 维输入

当前源码仍冻结五项 policy input：

1. `lidar_surrogate[3200]`：80×40 固定球面方向，值为 UNKNOWN/FREE/OCCUPIED；
2. `future_positions_body[20][3]`：source-time active official EGO B-spline 的未来位置；
3. `actual_velocity_body[3]`；
4. `tracking_error_body[3]`；
5. `previous_applied_v_max[1]`。

总维度为 `3200+60+3+3+1=3267`。mission/planner 状态、progress、clearance、复杂度 label、mask、diagnostics 与 run/episode provenance 不进入 policy input。frame/stamp/generation、五帧历史、active official trajectory 或任何数值缺失时继续 fail closed；不得复用旧 C、延长过期轨迹或生成 synthetic trajectory。

### 23.2 dynamic `v_max`、SpeedAdapter 与 EGO

```text
SAC normalized action [-1,1]
 -> ActionMapping: v_max = 1.025 + 0.725 * action
 -> SpeedRequestStamped
 -> SpeedAdapter / SpeedSafetyFilter
 -> SpeedActionStamped
 -> EGO dedicated speed callback queue
 -> immutable DynamicVmaxSnapshot commit
 -> SpeedAppliedStamped exact ACK
 -> causal transition -> OrderedTransitionWriter -> Replay
```

- policy 范围为 `[0.30,1.75] m/s`；EGO static `4.0 m/s` 是 planner/downstream capability ceiling，不是 SAC action，也不是 UAV 实际速度命令。
- 合法范围内 `requested_v_max == filtered_v_max == applied_v_max`；SafetyFilter 只做 finite、范围和 clamp，不做 slew、low-pass、hysteresis 或 maximum-step shaping。
- 相邻 action delta 超出 `[-0.3,+0.5] m/s` 时才额外请求 force-replan；范围内沿用 EGO 原生 replanning rules。
- 正式配对只认 `(episode_id,step_index,request_id)`；scalar topic 只是镜像，不是第二个 owner。
- 9 月 1 日后 speed callback 使用独立 `ros::CallbackQueue + AsyncSpinner(1)`；`DynamicVmaxSnapshotStore::commit()` 是 ACK 线性化点。每次 planner invocation 捕获一个 immutable `(version,v_max,force_generation)`，manager/optimizer 全程绑定同一版本；speed thread 不直接修改 FSM、map 或 optimizer，也没有启用全局 `AsyncSpinner(4)`。

### 23.3 当前 Reward v3.1 与 Stage 1 / Stage 2

当前源码与 YAML 的唯一 owner 为 `training/reward.py::LearningSpeedReward.evaluate()`，版本 `astradrone_paper_guided_reward_v3.1`，默认 `stage_1`：

```text
r = r_speed + r_smoothing + r_error + r_danger
```

令 `u=||actual_velocity_body||`、`e=||tracking_error_body||`、`a_t/a_{t-1}` 为当前/上一 stamped applied constraint，`N` 为最近 known occupied probe，`D=known occupied bins/3200`：

```text
q_N = clip((6.0-N)/3.5); N=None 时 q_N=0
q_D = clip((D-0.040)/0.040)
phi_2 = 1-(1-q_N)^0.46*(1-q_D)^0.54
phi_1 = 1.75-phi_2
w_safe=(1-phi_2)^2
w_middle=2*phi_2*(1-phi_2)
w_dangerous=phi_2^2
```

Stage 1：

```text
r_speed = w_safe*0.80*(u-phi_1)
        + w_middle*0.25*u
        + w_dangerous*1.00*(phi_1-u)
r_smoothing = -0.10*(a_t-a_t-1)^2
r_error = -2.00*min(e,0.40)^2
r_danger = -2.00*u^2，仅 explicit dangerous terminal
```

Stage 2 代码只把 `r_speed` 换成 `0.25*u`，其余三项复用；当前未选择、未训练、未评估。

最重要的当前源码裁决是：**v3.1 已删除 UNKNOWN 专用 `phi_2=0.5/phi_1=1.25` 分支。** `unknown_majority` 只作诊断；当 `N=None,D=0` 时当前 Reward 得到 `phi_2=0,phi_1=1.75`。UNKNOWN 仍通过 3200 维 policy input 影响 Actor，但不再是手工 Reward target。旧 v3.0/AGENTS 快照中“UNKNOWN 固定 1.25/0.5”的表述已过时。

Reward 不读取 progress、success bonus、clearance、planner shaping 或 `state_t+1` 数值。它评价 `state_t` 的实际速度/跟踪误差，因此新 action 的直接即时项主要是 smoothing，存在一拍 credit assignment 与 planner/controller 混杂。公式、离线 replay 和 finite runtime 已验证，不等于行为目标、无 reward hacking、收敛或迁移已证明。

### 23.4 SAC 与 Episode 生命周期

当前唯一维护配置仍是 3267-D/1-D Gaussian SAC：twin Q/target Q、automatic entropy、Replay、异步 learner 与 checkpoint。关键值为 `gamma=0.99`、`tau=0.005`、batch 64、Actor/Critic/alpha LR `1e-5/1e-3/1e-3`、100 次 critic-only warm-up、log-std `[-3,-1]`、Replay capacity 100000、learning starts 1000、seed 1。

正式目标仍是从空 Replay 完成 10000 个 training Episodes，每 500 Episode checkpoint，共 20 个；Episode 10000 后不得启动 10001。evaluation 是独立 mode，加载指定 checkpoint，固定 100 Episodes、deterministic Actor、无 learner/训练 Replay。当前两者都没有放行或完成。

运行时保持：absolute 0.1 s nominal grid、no catch-up、最多一个 active ACTION INTERVAL、transition core 形成后 immutable、单 worker ordered persistence、Replay contiguous、terminal exactly-once、writer drain/closure ACK 后才允许 reset。`success/collision/planner_failure/max_episode_time/invalid_observation` 都是真实 terminal/truncation 分类；合法环境失败可以完成 Episode 并进入 Replay，但任何 final audit failure 仍使对应 qualification 总体 NO-GO。

## 24. 8 月 30 日至 9 月 1 日关键技术演进

| 阶段 / RUN | 真实结果 | 当前替代关系 |
|---|---|---|
| mapping/timing 反向审计 | mapping owner 清楚；旧 one-open 把 policy cadence 与 Replay/provenance/terminal 串成最慢 closure throughput | `MAPPING ARCHITECTURE OK`；timing 要解耦 |
| Timing Phase 1 static | immutable transition + `OrderedTransitionWriter`；178 个 Learning Speed 测试、39 个 backend 测试等通过 | 只放行 bounded runtime |
| Phase 1 bounded，4 Episodes | 1266 transitions；Replay/ownership/terminal全通过；pre-learning `9.975 Hz`、learner-active `9.638 Hz`；真实 planner/collision 不构成任务 PASS | timing/correctness bounded PASS |
| Phase 2A cleanup | 删除 post-action/latest trajectory 的 hard gate，保留 selected trajectory 与 near-lifetime gate | static PASS |
| Phase 2A bounded，6 Episodes | 1712 transitions结构全通过，但 pre-learning `8.264 Hz`、learner-active `6.775 Hz`，`causal_snapshot_unavailable=671`；Ep4/5约5.4 Hz | runtime FAIL；不能进入 Phase2B |
| causal root-cause | 10 Hz C 与 10 Hz policy 等频相位竞争：C晚1～20 ms到达会等下一tick，0.1 s放大为0.2 s | 不是放宽 causal gate；选择15/10设计 |
| 15 Hz perception / 10 Hz policy Phase A，6 Episodes | 2368 transitions；V2/C约14.92 Hz；causal miss `101/2473=4.08%`，较10/10下降85.5%；无5 Hz Episode；0 success、4 failure、2 truncation、1 collision proxy、3 planner failure | timing/correctness PASS，不是飞行 PASS |
| 首次 100EP early-learning | 47/100 completed 后 Ep48 request194 `applied_identity_ack` timeout；0/47 success、34 planner failure、6 collision、7 truncation | FAIL-CLOSED，不得扩大 |
| Ep48 根因 | request194 到 SpeedAdapter/EGO transport 后，EGO default single-thread spinner 被同步 replan 4808/4809 占用至少7.291 s，`speedLimitCallback()`未执行，ACK未生成 | 不是 runner拒绝、CPU泛化或timeout太短 |
| EGO async fix | 独立 speed queue + immutable snapshot + main-thread force handoff；main queue忙4.50009 s时100/100 exact ACK，max `0.295469 ms` | source/unit/ROS PASS |
| `ego_speed_async_100ep_endurance_20260901_114200_r01` | 100/100 closure、29,543 exact request/action/ACK/Replay、0 applied timeout、100 terminal exactly-once；但 Ep53/54/77/95/98 为 `invalid_observation:continuous` | EGO async子系统 PASS；RL整体 NO-GO |
| continuous-invalid 根因 | final planned trajectory结束后 EGO进入 `WAIT_TARGET`，但 Hector actual position/speed/sustain未满足；C正确返回 `trajectory_unavailable` | primary 为 final-goal lifecycle，不是 C/V2/SAC |
| final-goal handshake 6-case r01～r05 | 静态测试通过；多轮分别被 sandbox、formal reset、supervisor、torch path、Hector overlay阻塞；均保留0/6或无专项覆盖结果 | 不得把基础设施排除写成逻辑 PASS |
| r06 full 100EP environment | 6/6 Episode闭合，5 success/1 planner failure；只注入2/6 case。Ep4真实 continuation request=1，但 accepted real trajectory=0，最终 `NO_FEASIBLE_TRAJECTORY` | `FINAL GOAL LIFECYCLE LOGIC FAIL` |
| lifecycle rollback | handshake production增量全部移除；EGO async fix保留，相关 package/ROS gates通过 | 当前源码状态；不再继续补 lifecycle patch |
| rollback baseline r02 | 31 Episodes 后 reset readiness timeout | 独立基础设施 NO-GO，不与后续混池 |
| rollback baseline r03 | 87/100 completed；Episode88外部 shutdown、无 closure；26,761 closed Replay + 65 partial Replay | 最新总体 baseline：INCOMPLETE/NO-GO |

## 25. 最新 100-Episode、速度、碰撞与 planner failure 审计

### 25.1 两个不能混淆的 100EP 结论

1. `ego_speed_async_100ep_endurance_20260901_114200_r01` **确实完成 100/100**，证明 EGO async speed chain 在 29,543 次 exposure 下消除了 Ep48 ACK starvation；但 5 个真实 invalid-observation terminal 使总体仍为 `RL CLOSED LOOP 100EP FAIL`。它不是 training PASS、收敛或 evaluation 证据。
2. lifecycle 回滚后的 `post_lifecycle_rollback_100ep_baseline_20260901_202221_r03` 只完成 **87/100**；Episode88 有 65 条非 terminal Replay 后外部 shutdown，不能补算 completion，也不能 resume。它是当前源码状态下更新的 baseline，最终仍为 INCOMPLETE/NO-GO。

### 25.2 rollback baseline r03

| 项目 | 当前最新事实 |
|---|---:|
| completed / configured Episodes | 87 / 100 |
| success | 1 |
| `planner_failure:NO_FEASIBLE_TRAJECTORY` | 55 |
| collision terminal | 20 |
| `max_episode_time` | 10 |
| `invalid_observation:continuous` | 1 |
| closed-Episode Replay | 26,761 |
| Episode88 partial Replay | 65 |
| total persisted Replay | 26,826 |
| requested/action/applied mismatch | 0 |
| duplicate / step hole | 0 / 0 |
| finite Reward | 26,826 / 26,826 |
| Observation C persisted dimension/valid | 3267；26,826/26,826 |
| coordinator C valid ratio | 44,170/44,184 = 0.999683 |
| median / mean action period | 0.100 / 0.109913 s |
| learner updates | 22,490，全部 finite |

`requested_v_max == applied_v_max` 为 26,826/26,826，最大差为 0，最大值 `1.746997 m/s`，没有越过 1.75。EGO `4.0 m/s` 只是静态 capability ceiling。

Replay policy-visible actual-speed 最大 `2.960955 m/s`；490/26,826（1.826%）、67 个 Episode identity 超过1.75。coordinator raw Gazebo-truth twist 的最大值相同。原因边界是 EGO约束逐轴 B-spline velocity，而统计为三维实际速度范数，且还有轨迹切换/执行瞬态；缺少逐样本 PositionCommand，不能把每次超限拆成范数组合与 controller overshoot。

### 25.3 planner failure：`PLANNER-LIMITED`

55 次 planner failure 不支持“主要由高速 SAC action 驱动”：

- final requested median `1.307 m/s`，低于非 planner 的 `1.489 m/s`；最后1 s median `1.273` 对 `1.496 m/s`。
- 13/55 final action `<=1.0 m/s`，只有8/55 `>=1.5 m/s`；按 exposure 归一后 planner failure 在 `[1.20,1.50)` 最高、在 `[1.50,1.75]` 反而较低。
- terminal 前5→0.5 s，mean `v_max` 从1.302降到1.262；nearest mean从1.714降到0.673 m、density从0.0803升到0.1146、`phi_2`趋近1。
- 55/55最后3 s有 plan-fail，50/55有 `First 3 control points in obstacles`，同时仍有成功 replan 与 active selected trajectory。典型链是局部 replan成功/失败交错，最后候选耗尽或安全停，而不是 dynamic action没有应用。

因此主分类为 `MIXED`，planner-failure 次分类为 `PLANNER-LIMITED`。现有证据不支持先降 action upper bound来修 55 次 planner failure。

### 25.4 collision：速度相关，但不是单因果

20 次 collision 的 final requested median `1.522 m/s`，高于非 collision 的 `1.307 m/s`；最后1 s actual median `1.057` 对 `0.802 m/s`，density也显著更高。collision exposure-normalized rate随 action bin上升，说明高速是风险相关量。

但 17/20 collision 在最后3 s已有 `plan_success=0`，且多数伴随高 density/first-3/emergency。当前 `collision` 证据层是 EGO `current_position_in_occupancy` proxy，不是独立 Gazebo contact truth。只能写“速度、障碍密度和 planner degradation 的混合结果”，不能写“1.75 m/s 单独导致碰撞”。

### 25.5 SAC 与 Reward 的新疑点

- action 分布从 Episodes1–20 的 mean 1.022 m/s 上升到41–60的1.614、61–87的1.593；后两阶段 `[1.50,1.75]` 占84.56%/80.03%，存在明显 upper-region 偏置，但没有 exact-bound saturation。
- late-stage复杂状态仍有条件降速：`phi_2<0.25` 与 `phi_2>=0.75` 的 action mean 为1.660与1.477 m/s；policy并非完全忽略复杂度。
- 14/55 planner failure 与5/20 collision 的 terminal `r_danger=0`；10个危险 terminal total reward仍为正。唯一 success return `-180.884`，而 planner/collision median return为 `-86.317/-105.597`，短失败累积较少负 step reward，存在 Episode 长度偏差与 terminal taxonomy 疑点。
- 这些证据足以标记 `REWARD-SUSPECT`，但不足以立即修改公式、系数或 action upper bound；也不能把 planner failure 倒推为 Reward 过快。

## 26. 主要新旧结论冲突与裁决

| 冲突 | 旧结论 | 当前裁决 |
|---|---|---|
| 100-Episode 是否运行过 | 8月29日写“未启动” | 已有一轮100/100，但总体因5个 invalid observation NO-GO；rollback后新baseline仅87/100 |
| 最新 lifecycle blocker | terminal identity handshake timeout | 后续定位为 final planned trajectory与actual acceptance脱节；handshake方案r06逻辑FAIL并已从production回滚 |
| EGO async状态 | design READY / unit READY | dedicated queue/snapshot已实现；29,543-action run和rollback窗口均0 applied timeout，子系统runtime PASS |
| 15/10时序 | 尚无runtime | Phase A 6ep PASS并消除5 Hz双稳态；长run保持约9 Hz但causal miss后段增加，不能称全程稳定10 Hz |
| Reward UNKNOWN | v3.0/旧快照固定 `phi_2=0.5,phi_1=1.25` | 当前v3.1源码删除UNKNOWN target；`N=None,D=0` 得0/1.75，UNKNOWN只进policy input |
| R04 31/31 | 容易误读为31ep PASS | 仍因invalid observation为NO-GO；31/31只证明closure/map-switch等子门 |
| fixed-vmax 0.30～1.75 PASS | 可能被外推为Forest SAC范围安全 | 仅证明worksite fixed-only backend可执行；不覆盖Forest、动态SAC、terminal或三机/full-stack |
| final-goal 6-case | unit/base 6ep可误写为修复成功 | r06唯一真实continuation case request=1/accepted trajectory=0并planner fail；方案已回滚 |
| planner failure与高速 | 直觉上归因高action | 55次证据更符合PLANNER-LIMITED，failure rate不随speed单调增加 |
| collision与高速 | 要么全归高速、要么全归planner | 当前为MIXED：高速显著相关，17/20同时有planner degradation，且collision是occupancy proxy |
| finite learner/Reward | 可能写成训练健康/收敛 | 只能证明数值有限；0/47 success、100ep总体FAIL、rollback 1/87 success均否定放行 |

## 27. 当前最新状态总表

| 子系统/阶段 | 最新状态 | 证据边界 |
|---|---|---|
| 三机3 m同高度绕塔 | **已完成的历史工程基线** | 完整360°/8扇区/EXIT/返航/落地；不是本轮重跑 |
| PX4/FAST-LIO full-stack | **已有基线** | 与training-only互斥；未做本轮RL transfer |
| Observation V2/C 3267-D | **CURRENT / 数据合同成立** | 当前源码、Replay维度与valid证据；无权放宽fail-closed |
| SpeedAdapter/SafetyFilter | **CURRENT** | finite+clamp、stamped identity；无action shaping |
| EGO dynamic-vmax async | **子系统 runtime PASS** | 100-action stress、29,543 exposure及rollback窗口0 applied timeout |
| Reward v3.1 Stage1 | **CURRENT / EXPERIMENTAL** | single owner、公式/finite成立；terminal coverage与行为目标待审计 |
| Reward Stage2 | **PLANNED** | 只有公式，未训练/评估 |
| SAC implementation | **CURRENT / bounded functionality** | Actor/Q/Replay/learner/checkpoint实现与finite运行；无收敛结论 |
| 15 Hz perception / 10 Hz policy | **架构局部PASS、长期PARTIAL** | 无5 Hz回退；mean约9 Hz、causal miss仍存在 |
| final-goal handshake | **SUPERSEDED / ROLLED BACK** | r06 logic FAIL；production增量已移除 |
| rollback 100EP baseline | **INCOMPLETE / NO-GO** | 87/100，Episode88未闭合，1 success/55 planner/20 collision/1 invalid/10 max-time |
| formal 10000-Episode training | **未完成/不放行** | 旧pilot禁止resume；当前系统门未通过 |
| 独立100-Episode evaluation | **未完成** | 无合格checkpoint与evaluation run |
| Forest→worksite/full-stack/三机迁移 | **未完成** | world、backend、定位、轨迹与多机分布均不同 |

## 28. 本轮来源吸收与清理审计

### 28.1 数量闭合

```text
runtime_artifacts Markdown discovered before cleanup = 65
already represented by Section 21 baseline           = 41
new runtime Markdown audited in this increment       = 24
specified root summaries audited                     = 3 / 3
total source Markdown scanned and merged             = 68
NOT_FOUND                                             = 0
runtime Markdown deleted after absorption             = 65
specified root summaries deleted                      = 3
target main document deleted                          = 0
non-Markdown artifacts deleted                        = 0
code/config/parameter changes                         = 0
ROS/Gazebo/PX4/training/evaluation/tests run          = 0
```

第21节的原始 manifest 已逐项记录前41份 runtime 报告。本轮新增24份 runtime 报告和3份指定旧汇总如下；SHA-256为删除前值：

| 来源 | 删除前 SHA-256 | 吸收主题 |
|---|---|---|
| `runtime_artifacts/learning_speed/upstream_mapping_and_async_timing_reverse_audit_handoff.md` | `4efd7b9e12101941eab500e0b0d78f24d4785378671bc7237cd0807e4de1fa11` | mapping/timing handoff |
| `runtime_artifacts/learning_speed/upstream_mapping_and_async_timing_reverse_audit.md` | `b70be4cca04d31c15e5c273ea52c3c9ed1f6738ccc351c5b5f37b9b4eff2d9b0` | mapping owner与async目标架构 |
| `runtime_artifacts/learning_speed/async_timing_refactor_phase1_report.md` | `e5f7d7f9db39c4c8ad537ac939bb6dbf8f92389882683ec5f3b9d4ed1ff6ad93` | immutable transition/ordered writer |
| `runtime_artifacts/learning_speed/async_timing_refactor_phase1_bounded_runtime_report.md` | `a479214995018181afef09dff8afde1ca10bc74491fb0308e80fb5a492573924` | Phase1 4ep/1266 runtime |
| `runtime_artifacts/learning_speed/async_timing_refactor_phase2a_cleanup_report.md` | `80398a21aff79505f9fb439354bef4d9341de8215091fe6c9ef24c8f205b8002` | Phase2A cleanup |
| `runtime_artifacts/learning_speed/async_timing_refactor_phase2a_bounded_runtime_report.md` | `d2166671f90688a3ce3b99a65321e9becd0d029c3c4d027f8a89c131b433fc8e` | Phase2A cadence FAIL |
| `runtime_artifacts/learning_speed/causal_snapshot_unavailable_root_cause_audit.md` | `14c6f64ffae5da42b63494e3779067930b483f0f7c587b924169714a4a0a5d42` | 10/10 phase race |
| `runtime_artifacts/learning_speed/perception_policy_rate_redesign_audit.md` | `39536fc7408bdefc39a929a6fad95e9eb6a97ec5d6c225b03c2a35aaa47644ac` | 15/10设计 |
| `runtime_artifacts/learning_speed/perception15_policy10_phaseA_runtime_report.md` | `875b9eeb85b23618bc9fa06c86ec5d69842bdef21d2a3d3e65d433a88893d9f3` | 6ep PhaseA PASS/flight NO-GO |
| `runtime_artifacts/learning_speed/perception15_policy10_100episode_report.md` | `4c8593b91ed8d896bdd51ff8609559b13b3892361be9626c5c204087d747793b` | 47/100 ACK FAIL |
| `runtime_artifacts/learning_speed/applied_identity_ack_episode48_root_cause_audit.md` | `2f4a833487c7456d92080563d09bc9ba231e392dadc034c28ec68b97e909414e` | request194根因 |
| `runtime_artifacts/learning_speed/learning_speed_ego_async_execution_design_audit.md` | `4a0dc0133182765d554945b9655307af8bc620ec541dedbada2be33d3f9abc30` | EGO async方案裁决 |
| `runtime_artifacts/learning_speed/ego_speed_async_queue_snapshot_fix_report.md` | `e81b2be867ee44bd436d0b86b5fda147b2065726224ee1a9ce6748a126fb5e52` | dedicated queue/snapshot实现 |
| `runtime_artifacts/learning_speed/ego_speed_async_100episode_endurance_report.md` | `85306d09fd53462d70f1a2302b57e3dee9cbbea000d3ffc87e2c713e17afdce3` | 100/100总体NO-GO |
| `runtime_artifacts/learning_speed/observation_c_continuous_invalid_root_cause_audit.md` | `bb9b59b04724a36aa3a6d2082e78c2d13a9208db997d948d2ca43d62582aef0b` | final trajectory lifecycle |
| `runtime_artifacts/learning_speed/final_goal_lifecycle_handshake_fix_report.md` | `d85b02cf962388bbbcf7d5317d305652b66185daea462d14bc2295e7d9109a97` | handshake实现/0ep FAIL |
| `runtime_artifacts/learning_speed/final_goal_lifecycle_6case_runtime_qualification.md` | `4d7fbf844691cceae58f8d84c37f089f0f632673b60a138476160df47aea73f7` | r03 case未覆盖 |
| `runtime_artifacts/learning_speed/final_goal_lifecycle_6case_20260901_r04/qualification_stop_report.md` | `b0ba4fa5b9367096ed3a266094f8a27f2e25b1a23abbfbd7ed0bdfe39db57b01` | r04 sandbox STOP |
| `runtime_artifacts/learning_speed/final_goal_lifecycle_6case_r04_external_runtime_verification.md` | `86e7890fa8f7533d37410989b5730224379d8759024dbd40acdcfbda97eb646a` | torch环境阻塞 |
| `runtime_artifacts/learning_speed/sixcase_python_environment_mismatch_audit.md` | `6347c40feb463102893ab44ca882feaa1a80ac0b523704bec69bdbe73d428037` | Python/PYTHONPATH根因 |
| `runtime_artifacts/learning_speed/final_goal_lifecycle_6case_r05_sameenv_rerun.md` | `907079b3ceba5b37ed55596db7283351e13436e6bb80130c458953226568a570` | Hector overlay环境阻塞 |
| `runtime_artifacts/learning_speed/final_goal_lifecycle_6case_r06_full100epenv_runtime.md` | `86302a56e0876514a078605040b786520d09b3f16212ad7ed52ebcc241502bbb` | handshake logic FAIL |
| `runtime_artifacts/learning_speed/post_lifecycle_rollback_100episode_baseline_report.md` | `43b4ee7c67075b9270c60160801894760aa2e293d080ca4c5bdce5e0b8a21562` | rollback与87/100 baseline |
| `runtime_artifacts/rl_training/post_lifecycle_rollback_speed_failure_root_cause_audit.md` | `750bd8db01de9ccdec5c0b8af05fbbed0985427e097a2519ec181a56204a4c06` | speed/collision/planner/Reward审计 |
| `AstraDroneOpen_研究报告全项目技术审计与证据索引.md` | `e6d789483725421aea0f0b74d619899dcf74a5e3f208bc4b77d39dc368620e91` | 三机/PX4/FAST-LIO/EGO全项目事实 |
| `LearningSpeed_reward_current_audit_and_merged_report.md` | `2a80302217d4d27494af3978be57526ab674916355dd06061c61ae65dd579bcf` | Reward v3.1当前公式与历史替代 |
| `AstraDroneOpen_项目技术演进汇总_handoff.md` | `b5cd77faa1b93597b29100cc10770dcafa076897906370fb1459fd745aade0d9` | 原51源演进/handoff |

清理只针对上述清点时存在的65份 `runtime_artifacts/**/*.md` 和用户点名的3份根目录旧汇总。CSV、JSON、JSONL、log、bag、checkpoint、NPZ、图片、模型、ROS日志、原始实验目录和源码均未删除；本主文档保留并成为后续 RL 阶段唯一主参考。

## 29. 当前完成度、问题与强化学习下一步

### 29.1 当前已完成内容

- 三机3 m同高度低空绕塔工程基线，以及PX4/MAVROS/Mid360/FAST-LIO/EGO-Swarm/任务协调的职责与证据边界已经整理清楚。
- Learning Speed一维 dynamic `v_max` 链、SpeedAdapter/SpeedSafetyFilter、exact stamped identity、commit-before-ACK和EGO dedicated async snapshot实现已完成；Ep48 callback-starvation在后续大规模exposure中未复现。
- Observation V2五帧3200-bin与Observation C 3267-D合同、15 Hz perception/10 Hz nominal policy、one active interval、immutable transition、ordered Replay、terminal exactly-once和reset barrier均已有源码及运行证据。
- Reward v3.1 Stage1的唯一owner、`phi_1/phi_2`、四分项、UNKNOWN当前语义和Stage2候选边界已审计；当前runtime内所有已持久化Reward finite。
- 已完成一轮100/100 EGO-async endurance和一轮rollback后87-Episode baseline，真实失败、partial Episode与NO-GO均保留。

### 29.2 已跑通但仍需优化的问题

- EGO async speed子系统已跑通，但RL整体仍受planner capability、final trajectory生命周期和invalid observation影响。
- 15/10消除了稳定5 Hz双稳态，但长期effective rate约9 Hz，mean period 0.109913 s、p95 0.2 s，causal miss和deadline error仍需观测。
- Replay/Actor/terminal结构在已闭合Episodes内正确，但100EP baseline没有完整闭合；Episode88不可补算或resume。
- SAC已形成明显高速度偏置，同时对高`phi_2`有条件降速；目前只有1/87 success，不能解释为有效学习。
- collision与高速显著相关，但多数collision同时已有planner degradation；需要更强的contact/command/制动证据后才能作单因果判断。
- Reward的dangerous terminal覆盖和Episode长度偏差存在真实疑点，但还没有证据支持立即改Reward或降低1.75上限。

### 29.3 尚未完成内容

- 当前源码/参数下完整、无基础设施中断且final audit PASS的100-Episode baseline。
- Forest中稳定的planner/collision/success表现，以及同一当前source上的固定速度dose-response。
- Reward v3.1行为专项qualification、Stage2训练、Reward消融和长期无stall/hacking证明。
- 正式10000-Episode training、合格checkpoint、独立100-Episode deterministic evaluation。
- Forest unseen-map泛化、worksite training-backend评估、FAST-LIO/PX4单机transfer、三机shadow/closed-loop transfer与真机验证。
- YOLO与三机巡检业务结果的联合验收。

### 29.4 强化学习下一步建议

**最高优先级不是立刻重跑100EP、修改Reward或降低action上限，而是对rollback r03的55次 terminal replan做只读EGO failure-signature与occupancy snapshot聚类。** 首先区分 first-3 control points、A*/rebound、current-position occupancy、trajectory retention/expiry与局部地图几何，固定同一当前source和真实失败口径，不调参数。

随后按顺序：

1. 专项审计19个危险 coordinator terminal 为何没有 `r_danger`，并用 discounted return、等长度窗口和 outcome-conditioned counterfactual 检查短失败比长成功“较少负”的长度偏差；证据完成前不改Reward。
2. 若planner/Reward审计提出明确候选，再设计同一Forest/source、SAC-off、相同reset/route的有界固定速度dose-response，并补 PositionCommand/contact/制动与完整 planner signature telemetry；用它区分速度、控制瞬态和几何能力。
3. 只有上述静态/有界门通过，才用全新RUN_ID、空Replay做新的bounded SAC qualification；失败即保留并停止，不resume、不skip、不放宽因果或timeout。
4. bounded gate通过后再重做完整100-Episode baseline；只有总体final audit PASS，才讨论正式10000-Episode training。训练完成后仍需独立evaluation与逐级full-stack transfer。

截至2026-09-02的总裁决：**AstraDroneOpen三机工程基线成立；Learning Speed数据链和EGO async dynamic-`v_max`子系统成立；Forest/Gazebo RL整体仍为NO-GO，正式training与evaluation未放行。**
