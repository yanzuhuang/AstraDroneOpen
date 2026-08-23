# AstraDroneOpen 项目技术演进与 Learning Speed 阶段汇总

> 更新日期：2026-08-24
>
> 当前边界：10000-Episode/500-checkpoint/100-evaluation 与 `v_max=[0.30,1.75]` 已完成代码/config/静态验证；fixed terminal-convergence bug 已最小修复，lightweight fixed-v_max 六档最终 PASS；正式 training、evaluation 均未启动，fixed PASS 不等于 SAC transition-closure runtime PASS
>
> 证据原则：当前源码/配置/launch 优先于旧报告；历史 FAIL/NO-GO 不改写为 PASS；training-only 结论不外推到 PX4/FAST-LIO full-stack 或真机

本文把 `runtime_artifacts/` 下全部 34 个 Markdown、根目录
`stage1_progress_transition_validation.md`，以及本文件旧版已整理的系统主线，重构为
唯一技术演进入口。CSV、JSON、JSONL、NPZ、PT、bag、ULog、日志、图像、视频和
runtime 目录继续作为原始证据保留。

## 1. 项目总体目标与系统架构

AstraDroneOpen 是 ROS1 Noetic + Gazebo Classic + PX4 SITL + MAVROS + Livox
Mid-360 + FAST-LIO + EGO-Swarm 的自主巡检研究工程。任务层负责八扇区、候选、
ENTRY/EXIT、重试、返航和终态；EGO-Swarm 负责局部 B 样条、避障、重规划和队友
时空轨迹避碰；bridge 是唯一 MAVROS raw-local 控制出口。

Learning Speed 的动作严格限于 EGO 动态 `v_max`。policy 不选择 waypoint、不生成
轨迹、不直接控制 Hector/PX4，也不拥有 clearance、碰撞检查或任务终态。

```text
full-stack：Mid360 -> FAST-LIO -> EGO -> traj_server
            -> EgoMavrosBridge -> MAVROS/PX4 -> Gazebo

training-only：Gazebo truth odom + Mid360 PointCloud2
               -> Observation C -> SAC v_max -> EGO -> traj_server
               -> Hector Pose/Twist controllers -> Gazebo
```

Training 不启动 PX4、MAVROS、FAST-LIO 或 EgoMavrosBridge；full-stack 不启动 Hector
execution/truth adapter。两条 `/uav1/Odometry` 路径通过 launch、进程和 publisher
exclusivity 互斥，禁止依赖 last-publisher-wins。

## 2. PX4 / FAST-LIO / EGO-Swarm 基础系统

三机 full-stack 已形成真实 SITL 闭环：Gazebo 传感器进入各机 FAST-LIO，frame
adapter 与 teammate/self/ground filter 形成 `/uavN/Odometry` 和规划点云，本机
EGO-Swarm 输出 B 样条，经 traj_server 和 bridge 交给 PX4 OFFBOARD。公共 `world`
用于轨迹共享与协调，本机 `uavN/camera_init` 用于规划。

worksite 早期 Mid360 启动阻塞定位为 ODE heightfield collision 与 20,000-ray
MultiRay 求交的病态性能，而非 PX4、FAST-LIO 或 namespace。collision mesh 替换后
1/2/3 UAV 感知链均可启动；三机 RTF 约 0.23 仍是算力风险，不能写成飞行验收。
EGO 核心是受控 vendor-derived 区域，不为单次实验改 core、关闭安全检查或改 world。

## 3. 单机与三机绕塔基线

单机已具备八扇区、候选换点、ENTRY/EXIT、有限重试、HOLD、逆进场路径返航和受控
降落。三机 worksite 最终闭环 PASS：UAV3/UAV2/UAV1 依次放行，目标相邻相位
67.5°，三机分别完成 360°、8 扇区、EXIT、HOME、落地和解除武装；最小机间距离
高于 3.0 m 门限。

历史输入/时效、地图边界、候选、许可和返航失败保留，修复没有降低安全门。
outdoor_village 只通过三机 initialization-only 与 UAV1 Learning Speed 飞行，不等于
三机 outdoor 任务闭环；YOLO 三路接口通过也不等于视觉业务闭环。

## 4. 障碍物膨胀、clearance 与真实距离历史问题

历史问题是把 inflated voxel center distance 当成 raw/真实障碍距离，并在任务层重复
套用 raw clearance。PRE_ENTRY 的 0.967 m 拒绝由此产生；对应 filtered raw point
约 1.62 m，来自真实松树回波。

| 表示 | 当前语义 |
|---|---|
| raw/filtered cloud、粗几何 | `minimum_clearance=1.0 m`，只应用一次 |
| EGO inflated occupancy | `map_additional_clearance=0.5 m` |
| EGO collision/A*/B-spline | 使用 inflated occupancy |
| Observation complexity | 从 filtered returns/surrogate 计算，与 planner clearance 分开 |

回归确认没有同一 occupancy buffer 的 double inflation；hard inflated boundary、方向性
optimizer `dist0` 与任务层 0.5 m operational margin 同时存在，但不能机械相加成一个
“真实安全半径”。

## 5. EGO 高速飞行与轨迹跟踪问题

旧 2.0 m/s 探索出现 tracking FAIL。根因是 EGO 已产生约 1.5 m/s 参考，而 bridge
仍按三维范数限到 velocity 0.5 m/s、acceleration 1.0 m/s²；PX4 水平参数不是首要
瓶颈。该旧失败不能代表修复后平台绝对上限。

Environment B 的另一条真实风险是 `CURRENT_POSITION_IN_OCCUPANCY`：确定性初始化
失败后 random polynomial fallback 生成下潜轨迹，靠近真实树和 inflated voxel 后触发
collision。证据支持“random fallback + 中段高度合同不足”机制，但缺失败当次完整
A* base point/direction/gradient，不能宣称唯一 planner-core 单点根因。

## 6. 高速参数链 / dynamic v_max 前置工作

高速独立代际使用 EGO `max_vel=4.0 m/s`、`max_acc=3.0 m/s²`、
`feasibility_tolerance=0.0`、`planning_horizon=7.5 m`，bridge norm envelope 覆盖 EGO
逐轴盒。Environment B progressive：1.75 PASS、2.0 PASS、2.5 FAIL；3.0/3.5 按真实
failure 即停未运行。2.5 虽最终 mission success/落地，过程中锁存 collision/dangerous
proxy，qualification 仍 FAIL。bridge raw 到 PX4 input 最大差低于 `1.2e-7 m/s`，
三档 velocity saturation 为 0：`HIDDEN VELOCITY LIMIT: NO`。最高稳定 qualification
是 2.0 m/s。

dynamic `v_max` 后续按 paper-guided 语义清理：合法 action 不再 slew、low-pass、
hysteresis 或 maximum-step shaping；相邻 delta 超出 `[-0.3,+0.5] m/s` 才额外强制
重规划，其余保留 EGO native replanning。正常语义为 `requested=filtered=applied`；
scalar topic 只是镜像，stamped identity 才是正式链。

## 7. Observation v2 / C 与 3200 点云 surrogate

Observation v2 用 80×40=3200 球面 bin 表达 obstacle/free/unknown，5 帧点云按各自
source stamp 的 pose 对齐到当前 body。worksite 塔旁 inward、CW/CCW tangent、
tower-behind unknown、continuous yaw/history 均 PASS；它仍是 surrogate，不是稠密
3D occupancy。

Observation C 冻结为 3267 维：`lidar_surrogate[3200]`、
`future_positions_body[20][3]`、`actual_velocity_body[3]`、
`tracking_error_body[3]`、`previous_applied_v_max`。mission/planner/clearance、lidar masks
和 diagnostics 不进入 policy input。

早期 A/B valid ratio 偏低的根因是 replan 后未按 source stamp 选 causal trajectory，
以及大 sim time 下 `sec/nsec -> float -> sec/nsec` 丢 1 ns。修复保留 0.05 s 门限，
没有放宽时间窗。Training 用 Mid360 PointCloud2 type 2、`mid360_link` 和 truth-pose
causal history；full-stack 保留 CustomMsg type 3 + FAST-LIO。metadata 必须真实写
`gazebo_truth_training`。

`N=NA,D=0` 表示 unknown-majority/no-known-obstacle，不得自动解释为 open/Low。

## 8. Learning Speed Adapter 与 action identity 链

最早 causal step v0.1 只有 headerless/value/timestamp 配对，0.5 s 与 1.0 s runtime
均 NO-GO；0.1 s 异步 causal step 删除了 state/action 间 native replan rejection gate，
通过 48/48 live steps。

Episode v0.1 初版虽接近 10 Hz，重叠 request 仍只能按 Float64/value 配对，正式 NO-GO。
最终新增 `SpeedRequestStamped -> SpeedActionStamped -> SpeedAppliedStamped`，以
`(episode_id, step_index, request_id)` 贯通，formal rule 为 request_id equality。
100 Hz burst 为 100/100 exact FIFO；UAV1 live Episode 为 100 request/action/applied/
transition、10.0 Hz、0 mismatch/timeout/drop，正确以 max steps truncated。Episode 后的
final-home planner failure 保留，不能写成 full-mission PASS。

## 9. Stage 1 Reward 设计与人工标定

首轮 A/B 0.30–1.50 m/s 共 12 run 均 terminal/mission PASS；post-fix 另有 A/B
0.30–1.75 共 14 条，A 全 PASS、B/1.75 保留 failure；current generation 有 16 个
selected cell、13 PASS/3 FAIL。不同 ceiling、acceleration、planner/bridge generation
不得混池。B 1.25–1.50 风险可重复但非确定性；时序敏感性有证据，唯一 planner-core
cause 没有。

Stage 1 `astradrone_stage1_reward_v1.0` 使用 Candidate C 连续 `phi_1/phi_2`，Unknown
固定 1.25/0.5；项目 λ 为 0.65/0.35、speed 分支 1.00/0.80/0.25、smoothing 0.10、
danger 2.00。这些是 AstraDroneOpen-specific，不是论文原参数。Reward 不含 tracking/
progress；danger 只来自冻结 dangerous terminal。

## 10. Stage 1 Reward / transition validation

Recorder 只对 valid、同 episode、非 truncated candidate 调用唯一
`Stage1Reward.evaluate()`；invalid/truncated 不变成 training-ready，历史 reward-null
calibration 不回写。online/offline equality 与 frozen replay validation PASS。

根目录 progress 报告的 `progress_ctx_A_v125_r01` 已吸收：progress 0→1、无下降，
waypoint 8→1 不 reset；452 条 transition 的 receipt、run/episode provenance、
`P_t/P_t+1/Delta_P` 因果检查通过，负 Delta_P 为 0；EXIT/return/landing 保持 1。
它证明 reward context 可记录，但 progress 不进 policy input，也未加入 Reward。

## 11. Gazebo 轻量 RL 训练架构决策

只读审计比较直接 teleport、受控飞回、full-stack relaunch 和事务式 airborne reset。
对 PX4/FAST-LIO full-stack，单独 set_model_state 无法清 estimator、map、controller、
trajectory 和 identity，不安全。训练主线因此选择独立 Hector backend。

Hector 方案中，直接 `/cmd_vel` 因 frame 与 position/yaw feedback 不完整被废弃；当前
不存在可确认的直接 trajectory interface；正式采用 `PositionCommand -> Hector
Pose/Twist controllers` 薄 adapter，保留 EGO/traj_server，不自写 PID/B-spline。

## 12. Hector + EGO execution backend

qualification03 为权威 runtime PASS：hover、straight、turning/in-flight re-goal、
continuous replan、cancel/hold、20/20 controller stop/teleport/start/engage reset 与
post-reset fresh EGO tracking 均通过，RTF 约 0.9883。planned-vs-actual error 非零；
一次 0.569 m/s overshoot 保留，没有调 PID。qualification01 的错误 hold 目标造成
1.83 m spike，qualification02 有 teardown 异常；二者是历史失败。

## 13. Gazebo truth odometry

`/ground_truth/state -> gazebo_truth_odometry_adapter.py -> /uav1/Odometry` 只规范 frame
并做 zero/future/stale/non-finite/quaternion/frame/order/publisher checks。9067 个
raw/adapted exact-stamp pair 的 position/velocity/yaw 最大差为 0，relay p95 约 0.001 s。
metadata 为 `gazebo_truth_training`、`fast_lio_provenance=false`。

## 14. simulated Mid360 + Observation C training backend

Training 复用 Livox Gazebo plugin、20000 rays/scan、10 Hz PointCloud2。五帧 raw cloud
用 `t_pose <= t_cloud` truth pose 对齐后进入冻结 3200 surrogate，再与 EGO B-spline、
速度、tracking 和 applied v_max 组成 3267。generation barrier、type 2 datatype、frame
与 publisher exclusivity 已通过 qualification；type 3 只属于 full-stack。

## 15. Episode / teleport reset

正式 reset：terminal + SAC closure → candidate validation → adapter target ack → cancel/
hold → clear Observation C/v2/truth-cloud history → stop controllers → pause → teleport +
zero twist → unpause → restart/engage → generation barrier → five-frame warm-up → fresh
trajectory → next Episode。reset generation 与 identity 一次一增；旧 generation、旧
trajectory、重复 request 均 fail closed。Random sampler 不改变 ledger identity。

## 16. worksite.world 正式 training environment

nominal Hover `(0,0,3)`，ENTRY_GATE `(-4.3148485145,5.8522070123,3)`。Episode 1
用 nominal spawn；training 后续以 seed 1001 在 X/Y offset `[-1,+1] m` 安全矩形采样，
z=3、yaw=0 固定，最多 32 次。最不利角点静态审计仍约有 1.33 m 余量。
这是 code/config/static-test ready；新 random reset 正式 10000-Episode 尚无 runtime PASS。
Evaluation 关闭随机化，固定 nominal Hover。

## 17. SAC training-loop integration

已有 Gaussian Actor、twin Q/target Q、automatic entropy、Replay、异步 learner、
checkpoint 与 qualification/training/evaluation runner。该轮集成时一维 action 使用旧
`[0.05,0.40] m/s`；当前正式候选已由第 20–22 节替换为 `[0.30,1.75]`。Replay 始终只收
完整 causal identity transition。

短程 qualification 覆盖 2500 transitions、1500 stochastic steps、5 Episodes，通过
identity、10 Hz、Replay、loss/Q/gradient、checkpoint、reset 与 planner/collision 门。
该 PASS 只放行从空 Replay 开始的新训练，不表示收敛或长期稳定。

## 18. 旧 10k pilot NO-GO

旧 pilot 在 4207 valid transitions fail closed，出现真实 `NO_FEASIBLE_TRAJECTORY`；
同时 stochastic action 从上界频繁跨到下界，引发大负 delta force-replan。Action
exploration instability 与 environment/planner failure 共同构成 blocker。结论严格为
NO-GO，旧 checkpoint/replay 禁止 resume，也不能被后续 qualification 回写成成功。

## 19. SAC action exploration instability 根因与修复

旧 `log_std=[-5,2]`、Actor LR 3e-4 在真实 3267-D replay 上出现 Actor/entropy 激烈
变化，alpha 由约 0.2 累积到约 1.6，action 触及双边界。修复不动 Reward、Observation、
EGO、Hector PID、SafetyFilter 和 v_max，只把 Actor LR 改为 `1e-5`、log-std 改为
`[-3,-1]`，并先做 100 个 critic-only updates。短程结论为
`SAC ACTION EXPLORATION STABILITY PASS`；不等于 10k 完成或多 seed 收敛。

## 20. 正式 10000-Episode 配置与停止条件

当前正式候选以 completed Episode 数作为唯一正常停止单位：

```text
PRIMARY STOP: completed_episode_count == total_training_episodes == 10000
checkpoint: every 500 completed Episodes, 500 ... 10000 (20 total)
1 valid environment step = 1 transition = 1 Replay experience
```

每 Episode 保持 `max_steps=500` 和 `max_episode_time=55 s`。Episode 9999 不能正常结束
training；Episode 10000 的 terminal transition、Replay boundary 和 closure 完成后才正常
结束，保存 final checkpoint，且 coordinator 不 reset、不启动 Episode 10001。success、
collision、planner terminal、max-steps/max-time truncated 等真实 terminal 均保留并在正式
closure 后计为一个 completed Episode；基础设施、identity、causality、Observation、
controller、Replay 或 checkpoint 异常仍 fail closed。

`learning_starts=1000` 的单位保持 transition：前 1000 条 experience 进入 Replay，达到
第 1000 条才启用 learner，不是等待 1000 Episodes。Replay capacity 100000、batch 64、
100 critic-only startup updates 保持。Training 分支没有 evaluation 调用；evaluation 只能
独立运行 100 Episodes，固定 nominal Hover、deterministic Actor、无 learner/network
update、无 training Replay。

## 21. 当前正式候选配置

| 项目 | 当前值 |
|---|---|
| normal stop / Replay | 10000 completed Episodes / capacity 100000 |
| Episode ceiling | 500 steps；`max_episode_time=55 s`；不是正常 stop owner |
| learning starts / batch | 1000 transitions / 64 |
| startup | 100 critic-only updates |
| Actor/Critic/alpha LR | `1e-5 / 1e-3 / 1e-3` |
| gamma / tau | `0.99 / 0.005` |
| entropy / log-std | target `-1` / `[-3,-1]` |
| normalized action / v_max | `[-1,1]` / `[0.30,1.75] m/s` |
| mapping | `v_max = 1.025 + 0.725 * action` |
| checkpoint | 每 500 completed Episodes；500…10000，共 20 个；10000 为 final |
| evaluation during training | 无调用路径 |
| independent evaluation | 100 Episodes、deterministic、fixed Hover、no update、no Replay |
| training/evaluation root | `runtime_artifacts/rl_training/<RUN_ID>/` / `runtime_artifacts/rl_evaluation/<EVAL_ID>/` |

最新命令已同步到 `studynote.md`。正式 training 操作者入口仍是：

```bash
/home/yanzu/AstraDroneOpen/scripts/run_sh/learning_speed_sac_training.sh
```

命令只是准备完成；本轮没有执行正式 training。Final checkpoint 命名为
`sac_checkpoint_episode_10000.pt`；其他 checkpoint 例如 Episode 500 为
`sac_checkpoint_episode_0500.pt`。独立 evaluation 只读引用选定 training checkpoint，
自身写入 `runtime_artifacts/rl_evaluation/<EVAL_ID>/`。

## 22. 新 v_max 链与 lightweight fixed-v_max qualification

正式展开检查确认 Actor → `SpeedRequestStamped` → `SpeedActionStamped` →
SpeedSafetyFilter → `SpeedAppliedStamped` → EGO dynamic `v_max` 的 active bounds 均为
`[0.30,1.75]`；EGO `manager/max_vel`、optimization/bspline ceiling 与 dynamic maximum
展开为 1.75，dynamic minimum 与 SpeedSafetyFilter minimum 展开为 0.30。旧 0.05/0.40
没有作为正式 active clamp。源码仍有 generic fallback、传感器时间门、feasibility
tolerance 和 `minimum_active_speed_mps=0.05` 等同值 literal；它们不是正式 action bound，
且正式 launch 的 action bounds 已显式覆盖。未加入 slew、low-pass、hysteresis、cooldown
或 action shaping，force-replan delta 语义未改。

2026-08-24 首轮复用现有 worksite Episode/reset coordinator 做六档、每档 2 Episodes 的短程
qualification；Episode 1 nominal Hover，Episode 2 使用 seed 1001 random Hover。只把 EGO
静态 ceiling 设为新正式上限 1.75，其余 EGO 参数、Hector PID、Reward、Observation C、
random Hover 范围和安全门均未改。

| fixed v_max | 结果 | Episode | Obs C valid | tracking p95/max m | actual p95/max m/s | accepted replans | reset |
|---:|---|---:|---:|---:|---:|---:|---:|
| 0.30 | PASS | 2/2 success | 1.000 | 0.0178 / 0.0675 | 0.2099 / 0.3023 | 76 | 2/2 |
| 0.75 | PASS | 2/2 success | 1.000 | 0.0568 / 0.1051 | 0.5315 / 0.5538 | 30 | 2/2 |
| 1.00 | NO-GO | 0/2；continuous invalid | 0.9828 | 0.0896 / 0.1304 | 0.7372 / 0.7891 | 21 | 2/2 |
| 1.25 | NO-GO | 0/2；continuous invalid | 0.9826 | 0.1146 / 0.1482 | 0.9690 / 1.0212 | 17 | 2/2 |
| 1.50 | NO-GO | 0/2；continuous invalid | 0.9810 | 0.1442 / 0.1816 | 1.2168 / 1.3622 | 15 | 2/2 |
| 1.75 | NO-GO | 0/2；continuous invalid | 0.9789 | 0.1699 / 0.2041 | 1.5630 / 1.6972 | 13 | 2/2 |

六档 planner failure、`NO_FEASIBLE_TRAJECTORY`、collision、controller/reset failure 均为
0；12/12 reset 成功。1.00–1.75 的每个 Episode 都在 terminal convergence 对齐到
`trajectory_unavailable`，超过冻结的 invalid grace 后以
`invalid_observation:continuous` fail-closed；真实失败不重跑、不降门限。0.75 首次 q01
在 0 Episode 因 gzserver exit 139/physical readiness timeout 属基础设施无效，保留原工件，
只以新 ID q02 重试一次并得到上述有效结果。

首轮结论为最高稳定通过速度 0.75 m/s，完整 `[0.30,1.75]` 动作范围 NO-GO。现有固定速度
Reward 标定数据覆盖 0.30–1.75 m/s，Stage 1 配置的 speed anchors 为
0.75/1.25/1.75，公式接受正速度，未发现明显速度域缺口；本轮 Reward 文件和实现均未改。
这只支持单独讨论是否保持 Reward，不覆盖 runtime qualification。

随后用外部 rosbag 对 0.75 PASS 与 1.00 NO-GO 各建立最后 100 个 Observation step
时间线。1.00 的 B-spline 在 sim 11952.795 正常结束；第一帧
`trajectory_unavailable` 为 11952.825，此时 UAV 距 ENTRY 0.0771 m、速度 0.2447 m/s，
尚未满足 success 速度门。UAV 到 11952.985 才进入原 0.25 m/0.20 m/s 包络，pre-fix
11953.106 failure latch 时只保持约 0.121 s，未达到原 0.30 s sustain。因此不存在
“success 已满足却先检查 invalid”的简单 ordering bug，也没有 EGO/traj_server 过早停止：
traj_server 始终以相同 trajectory ID、READY flag 持续发布终点零速 PositionCommand。

确认的 bug 是 fixed Episode coordinator 缺少 post-B-spline terminal convergence 语义：
Observation C 正确拒绝无 future trajectory 的样本，但 coordinator 把“正式轨迹已结束、
PositionCommand 健康持有 ENTRY、Hector 正在消除 tracking lag”的预期终端阶段当作正常
飞行轨迹丢失。最小修复只在 `action_owner=fixed` 增加严格 terminal-hold guard：必须是
fresh 且唯一 reason 为 `trajectory_unavailable`、lookup=`no_active_trajectory_at_stamp`、
source stamp 晚于正式 trajectory end、Observation/B-spline/PositionCommand ID 一致、
fresh READY PositionCommand 在原 ENTRY/速度容差内、实际位置已在原 ENTRY 容差内，且
planner/collision/controller 门已先通过。Observation C 仍保持 invalid；原 0.5 s invalid
grace、0.25 m/0.20 m/s/0.30 s success、EGO/Hector/Reward/random Hover/SAC/v_max/replan
均未改。SAC action-owner 不启用该 fixed-only guard。

修复后严格按 1.00→1.25→1.50→1.75 顺序各跑 2 Episodes（nominal + random）：四档均
2/2 PASS、合计 8/8 reset，planner/collision/controller failure 全为 0。terminal hold 使用
次数为 2/2、2/2、2/2、1/2；1.75 nominal 未使用 guard 即正常 success，证明不是无条件
放行。结合此前 0.30/0.75 各 2/2 PASS，当前 lightweight fixed-v_max 六档
`[0.30,1.75]` 最终 **PASS**。fixed path 没有 AstraDroneEnv transition/Replay/SAC closure，
所以该结论不外推为正式 SAC training runtime PASS。完整诊断见
`trajectory_terminal_timing_diagnosis_report.md`。

## 23. RL Legacy Code Cleanup 与废弃边界

当前只维护 `sac_training_v1.yaml`、`sac_training_runner.py` 和正式 Episode/reset owner。
旧 transition-count normal stop、transition checkpoint、100-Episode/20间隔配置、旧 pilot
resume、training 内 evaluation、headerless/value/timestamp pairing、合法 action shaping、
伪 FAST-LIO provenance 与第二套 B-spline/PID 均废弃。Checkpoint 只由显式 20 个 Episode
allowlist 在 closure 后触发；独立 evaluation mode 才加载 checkpoint，并硬编码为
deterministic、无 training Replay、无 learner/network update。

## 24. 当前边界与下一阶段

- 正式 10000-Episode training：**未启动**；fixed-v_max blocker 已清除，但本轮没有启动或
  runtime 验证 SAC action-owner/transition closure，也不自动构成正式 training 授权；
- 正式 100-Episode evaluation：**未启动，且没有正式 checkpoint 可评估**；
- random Hover fixed-v_max reset 与完整新动作范围六档均已通过；
- fixed-only terminal hold 不改变 Observation C valid 规则，也未验证 SAC pending transition
  在同类终端阶段的 closure；若后续获准正式 training，仍须以全新 RUN_ID、空 Replay、
  fail-closed 监控开始，不能 resume 旧 pilot；
- training 完成并产生正式 checkpoint 后，才可独立执行 100-Episode evaluation。

## 25. Source Manifest 与章节映射

删除前共有 35 个源 Markdown：34 个 `runtime_artifacts/**/*.md`，另 1 个根目录 progress
报告。下表固定 path/size/SHA256/title-role/merged-section。依赖包内 torchgen README
也按“全部 Markdown”纳入，但不构成项目技术结论。

| Source path | bytes | SHA256 | title/role | section |
|---|---:|---|---|---|
| `runtime_artifacts/AstraDroneOpen_强化学习训练环境与SAC交接汇总.md` | 25847 | `75ee58c28e0f080ddf2f7fd2ba69899bdef5454398ccd6c67967cef0a031b4ec` | RL/Hector/SAC 交接 | 11–24 |
| `runtime_artifacts/astra_drone_action_identity_episode_revalidation_report.md` | 11129 | `2240dd1d24441eee0cfbcb045865bc087b8a6b13e28ede1e69483645565fc7d5` | identity runtime PASS | 8,15 |
| `runtime_artifacts/astra_drone_env_paper_aligned_causality_report.md` | 14987 | `b251b4b85e12789ce391fe4348e7128178002e426e643d564671b5cade4286aa` | async causality | 8 |
| `runtime_artifacts/astra_drone_env_runtime_step_timing_report.md` | 11808 | `625eccc20f4a5d3cd2fe2b9c1702bbdbb055fa4b4b98db0b8906c06a6b502861` | 0.5/1.0 s NO-GO | 8,22 |
| `runtime_artifacts/astra_drone_env_step_v01_report.md` | 7779 | `31ac28bf3bfa73f06b0c20221c8b5e4c2c67685d553dcf9f906413e7c65ac4cd` | first causal step | 8 |
| `runtime_artifacts/astra_drone_episode_v01_report.md` | 10121 | `7d12bd6880bb067d4b8e2797b67537ee9681af8fffd3a323419c77a60ba175f0` | headerless Episode NO-GO | 8,22 |
| `runtime_artifacts/astra_drone_training_reset_reference_audit.md` | 36526 | `e807ebae11a6af8d5d91bf30ea418bcd82ea3878e782487c71d66dbe2f6caab4` | reset risk audit | 11,15 |
| `runtime_artifacts/astra_gazebo_rl_environment_readonly_audit.md` | 50637 | `3d844fc35f7e19aac4857b1c23c84921504075001692f8dda76a21b0d7383170` | Gazebo-RL comparison | 11,22 |
| `runtime_artifacts/high_speed_exploration/high_speed_exploration_report.md` | 6733 | `75a8106ee102311840fa5a80b246e51f249643d404754d33873cffe4e42a19f9` | old high-speed FAIL | 5 |
| `runtime_artifacts/learning_speed/calibration/manual_calibration_report.md` | 4539 | `f0e1115969fd665f921352029947341d72ccb9b87b45f109d76e2f987af000ea` | first A/B matrix | 9 |
| `runtime_artifacts/learning_speed/calibration/observation_c_environment_a_postfix_validation.md` | 3680 | `05329a2ea0041466e567b7a8c9d85941609538ba592a26e0048f34c84c0f13db` | post-fix A runtime | 7 |
| `runtime_artifacts/learning_speed/calibration/observation_c_final_data_quality_report.md` | 8596 | `bcad23252eaeaf6c7713ddd654bd18a8385c0a349f62c94f9b9511dc9d166fdc` | timestamp closure | 7 |
| `runtime_artifacts/learning_speed/calibration/observation_c_invalid_audit_report.md` | 6992 | `a42a87bf81a6571e4867131ebb7d16bb5401ce79f651bdce57335aeb5c974a2d` | invalid audit | 7 |
| `runtime_artifacts/learning_speed/calibration/postfix_stage1_20260818/manual_calibration_report.md` | 2571 | `5c08f5dfeadf5e221fb216126518265b1931499ad483b5a1fa531c4a9724a993` | template/missing-row artifact | 9,25 |
| `runtime_artifacts/learning_speed/calibration/stage1_reward_calibration_postfix_report.md` | 39554 | `6eebcc1ff0886d1dd86b057805f9b5bb3cabac9562e3083dc43c5fdf59eaa455` | 14-run post-fix | 7,9 |
| `runtime_artifacts/learning_speed/evaluation/20260813_gazebo_dynamic_speed_test.md` | 3823 | `c63cc7799b28f9fe4eedaa64bb8e03aab3f4f307ae63a27014b7bff39e0dbaed` | dynamic-speed interface | 6 |
| `runtime_artifacts/learning_speed/evaluation/20260813_mock_interface_test.md` | 1936 | `975f8169f5f48007e67f19f0ef1e9407fa98706476feb6dde7c6ca5c9763c66f` | mock interface | 6 |
| `runtime_artifacts/learning_speed/high_speed_progressive_qualification_20260820/manual_calibration_report.md` | 3023 | `f0e667d0fe4100a1ec9cdb4ede5e11c0e3b7447c5f3be681cfdd27a09d094f12` | progressive qualification | 5,6 |
| `runtime_artifacts/learning_speed/observation_v2/evaluation/evidence_manifest.md` | 628 | `56e99d2379f0ac5d6ff45a3ec6ec36fb860fd06667eefc464c46af1f90879099` | v2 evidence index | 7 |
| `runtime_artifacts/learning_speed/progress_transition_validation_20260821/manual_calibration_report.md` | 2854 | `ea4e5f4b7ab9240c35864c8c40c28e5e942283267be919973004f1caca663d6c` | progress run summary | 10 |
| `runtime_artifacts/learning_speed/stage1_reward_calibration_current_generation_20260820_231428/manual_calibration_report.md` | 5288 | `b9bde9dec07ae19e9ffd86898aae434e7b4da87b785e58f4751d644dedd73c94` | current 16-cell summary | 9 |
| `runtime_artifacts/learning_speed/stage1_reward_calibration_current_generation_20260820_231428/stage1_B_enhanced_planner_diagnostic_report.md` | 16596 | `36c9b8e968db033c4005949462f72820d5fe90de409cac2c44689ab871557129` | enhanced planner diagnostics | 5,9 |
| `runtime_artifacts/learning_speed/stage1_reward_calibration_current_generation_20260820_231428/stage1_B_mid_speed_failure_root_cause_analysis.md` | 18248 | `e6dd9e6931d5798faa21db31676cc2183d503e6349c6f6a1a448e79c28e04128` | failure cause bounds | 5,9 |
| `runtime_artifacts/learning_speed/stage1_reward_calibration_current_generation_20260820_231428/stage1_B_mid_speed_repeat_analysis.md` | 9859 | `b00065a7ea61efd331c0f5610b4cac2a668d1b7044fd35e29d41183493fd59b3` | repeatability | 9 |
| `runtime_artifacts/learning_speed/stage1_reward_calibration_current_generation_20260820_231428/stage1_effective_flight_speed_analysis.md` | 14501 | `ed244d672422f509862e415a32b14afed95c0de7073e2e9ad28130bdad0b9ec8` | actual speed | 5,6,9 |
| `runtime_artifacts/learning_speed/stage1_reward_calibration_current_generation_20260820_231428/stage1_na_zero_obstacle_free_unknown_analysis.md` | 9268 | `1c1b86a0860486ec79982eb9fb4475299bf20d7da9d3aa9995ef0ce212a11cd4` | unknown semantics | 7,9 |
| `runtime_artifacts/learning_speed/stage1_reward_calibration_current_generation_20260820_231428/stage1_nearest_density_complexity_analysis.md` | 19921 | `6fa9731c8c1ddfce2c83c09756aca390c301c4970d3f5bb48eb42317e8292346` | complexity | 9 |
| `runtime_artifacts/learning_speed/stage1_reward_calibration_current_generation_20260820_231428/stage1_progress_signal_audit.md` | 20335 | `da67639af3f4bc48889dceeaed99001690569e8cb450b9ba15945badb6f28744` | progress semantics | 10 |
| `runtime_artifacts/learning_speed/stage1_reward_calibration_current_generation_20260820_231428/stage1_reward_calibration_current_generation_report.md` | 14530 | `acf2223b87ebfb9a4b5eb4f17d2671ef5406d2a9aea3f8fa27caada0f61723fa` | current generation | 9 |
| `runtime_artifacts/learning_speed/stage1_reward_calibration_current_generation_20260820_231428/stage1_reward_v1_report.md` | 16703 | `b55e88176814fc344648e3801fb0304433feb4059cfbf10456a2eea8ea707a3f` | Reward offline PASS | 9,10 |
| `runtime_artifacts/learning_speed_replan_paper_alignment_report.md` | 9747 | `8030072424814745a2d9fc1b10357893b7b0d0cd88ff9bedc17008159a1e7466` | replan semantics | 6,8,22 |
| `runtime_artifacts/observation_v2_worksite_tower_validation_20260815/OBSERVATION_V2_WORKSITE_TOWER_VALIDATION_REPORT.md` | 10224 | `0a3bc9ee16aa0e1f243444ab19f5b1305e4c2aa17e4929f9359a5ead92c76257` | v2 tower PASS | 7 |
| `runtime_artifacts/sac_python_packages/torchgen/packaged/autograd/README.md` | 147 | `846894cc1682b34c33061117077be45945d4e0b8a66bacb9ff03ca8c0a0a43e4` | dependency build note | 17,25 |
| `runtime_artifacts/stage3_online_reward_transition_integration_report.md` | 7322 | `97e7dc05054df13e3d0025829550a665a424aa03049481703225ae7d055efa7c` | online Reward integration | 10 |
| `stage1_progress_transition_validation.md` | 6304 | `4c4e16ea46bbdf7b6903a0ad27e2e4f83ac6f845fa494b216162e3783ae22513` | 452-transition progress | 10 |

## Source Cleanup Audit

```text
MERGED_SOURCE_MD_COUNT = 35
SOURCE_MD_COUNT = 35
DELETED_RUNTIME_MD_COUNT = 34
DELETED_ROOT_MD_COUNT = 1
PRESERVED_NON_MD_RUNTIME_EVIDENCE_COUNT = 27284
NON_MD_MANIFEST_SHA256 = 6fe756204c10bc63b3cd1fd9a1b60c77ab87f81c83709c7fbd7646eac0eef8ca
cleanup_status = PASS
```

### MERGED_SOURCE_MD_FILES / DELETED_RUNTIME_MD_FILES

Source Manifest 前 34 个 `runtime_artifacts/...` 路径即完整 merged/deleted allowlist；
删除后 `find runtime_artifacts -type f -name '*.md'` 为 0。

### DELETED_ROOT_MD_FILES

- `/home/yanzu/AstraDroneOpen/stage1_progress_transition_validation.md`

### PRESERVED_LONG_TERM_DOCS

- `/home/yanzu/AstraDroneOpen/AGENTS.md`
- `/home/yanzu/AstraDroneOpen/studynote.md`
- 源码/package 内长期 README（不在本轮删除范围）

35/35 来源均已映射；旧 pilot/step/Episode NO-GO、真实 planner/high-speed failure、
后续 qualification PASS 与当前 10k stop-condition 修复均保留。非 Markdown path+size
manifest 删除前后完全一致，故只删除精确 allowlist 中的 35 个源 Markdown。
