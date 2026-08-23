# AstraDroneOpen RL Legacy Code Cleanup Report

> 历史快照（已被 100-Episode 正式停止迁移取代）：本文件以下 10000-transition stop、
> 5000/10000 checkpoint 和 `max_training_episodes` 描述只记录当时清理结果，不是当前
> 正式配置。当前事实见 `AGENTS.md`、`studynote.md` 和统一阶段汇总。

审计日期：2026-08-23

审计基线：`scene01-3uav-circuit-mission` / `b1e1614c4cca15e2832989dd8ba6a90cb8345e0f`

范围：代码、config、launch、test 静态清理；未启动 Gazebo、正式 training 或正式 evaluation。

## 1. CLEANUP STATUS

**PASS**

权威源码范围内的旧 training-internal evaluation、旧 2k checkpoint/evaluation
schedule、旧 smoke exploration profile 和冗余周期 checkpoint 开关已清除。正式
10000-transition training、独立 evaluation、qualification/debug、full-stack 边界均保留。

## 2. 删除与修改清单

### 删除的文件

- `learning_speed_rl/config/sac_training_smoke.yaml`：旧 integration smoke profile，含
  Actor LR `3e-4`、log-std `[-5,2]`、Replay 5000、固定 5 Episodes 等过时设置；没有
  launch、test 或正式 qualification 引用。
- `learning_speed_rl/scripts/summarize_sac_10k_pilot.py`：只服务旧 pilot 的
  `0/2000/4000/6000/8000/10000` checkpoint/evaluation 汇总；历史结论已进入统一总汇总，
  当前正式训练、测试与 qualification 均不引用。
- 同步从 `learning_speed_rl/CMakeLists.txt` 删除旧 pilot 汇总器安装项。

### 删除的函数、字段与参数读取

- 删除未被生产路径使用的 `mode_updates_networks()` 及 package export。
- `FormalTrainingSchedule` 删除冗余 `checkpoint_interval` 与
  `evaluation_during_training` 字段；checkpoint 只由显式 step allowlist 判断。
- `sac_training_runner.py` 删除：
  - `~training/checkpoint_interval`；
  - `~training/evaluation_during_training`；
  - `~evaluation/deterministic_actor`；
  - `~evaluation/write_training_replay`；
  - `~evaluation/network_updates`。
- `sac_training_v1.yaml` 删除对应 5 个字段。Evaluation 的 deterministic、无 Replay、
  无 learner/network update 现在是 mode 固有行为，不再是可打开的兼容开关。
- runner fallback 从旧 smoke 值收敛到正式值：run ID、Actor LR、log-std、Replay
  capacity/initial allocation、Episode max steps 均与正式 profile 对齐。
- qualification 新生成的临时 checkpoint 从 `sac_smoke_checkpoint.pt` 更名为
  `sac_qualification_checkpoint.pt`；历史 runtime checkpoint 未改名、未删除。

### Launch 参数

没有删除独立 evaluation 所需的 `runner_mode`、`evaluation_checkpoint_path`、
`evaluation_checkpoint_step` 和 `episode_count`。审计未发现 training-internal
evaluation 专用 launch 参数；这些保留参数只在显式 `runner_mode=evaluation` 时生效。

## 3. Training-internal evaluation

最终结果：**不存在**。

- `training` namespace 不再有 `evaluation_during_training` 字段或周期 evaluation 开关。
- training transition 路径只执行 Replay/learner、显式 checkpoint 和 10000 stop；没有
  evaluation 调用。
- runner mode 末端分派为 qualification、training、evaluation 三个互斥分支；
  `_finalize_evaluation()` 只在显式 evaluation 分支调用。
- checkpoint 加载只发生在 evaluation 的无 training-Replay/learner 分支；training 从空
  Replay 开始，不加载旧 checkpoint。
- `rg/git grep` 后，旧字段、旧 smoke profile 和旧 2k schedule 在权威源码/config/launch
  中均无可达引用；测试仅以 `assertNotIn` 锁定字段必须缺失。

## 4. 保留的旧功能及理由

- `runner_mode=qualification` 与正式 config 内 `qualification.*`：已有 2500-transition
  action stability qualification 价值，继续用于短程 fail-closed 准入，不是正式训练停止条件。
- `hector_backend_qualifier.py`：独立 Hector execution/truth/trajectory/reset qualification
  owner，显式 `run_qualification` 才启用；其 `/gazebo/set_model_state` 是 qualification 用途。
- `training_episode_reset_coordinator.py`：当前唯一正式 Episode/reset owner；其
  pause/zero-twist/teleport/controller restart/generation barrier/five-frame warm-up 是正式
  lightweight training reset，不是 PX4/FAST-LIO production reset。
- scalar `raw_v_max`、`v_max`、`applied_v_max`：保留为 fixed/mock/full-stack 状态与诊断
  mirror；正式 action pairing 仍只接受 `SpeedRequestStamped -> SpeedActionStamped ->
  SpeedAppliedStamped` 的 `(episode_id, request_id)` 相等。
- fixed/mock、Observation、Stage 1 Reward、manual/high-speed qualification/debug 工具：
  仍有明确测试、标定或 qualification 价值。
- full-stack PX4/FAST-LIO/EGO/bridge、Control/Land 包：不属于可证明 dead 的 training-only
  源码，本轮未修改。

## 5. UNKNOWN

- `AstraDrone_ros1_ws/devel/lib/learning_speed_rl/summarize_sac_10k_pilot.py`
  仍是旧 Catkin build 生成的 relay，指向已删除的源码，因此不可执行。`build/`、`devel/`
  是禁止手改的生成物；本轮没有做 destructive clean build。权威 source/CMake install
  manifest 已清除该入口，新干净 build 不会生成它。
- repo 其他非 training-only Control/Land/full-stack 路径中的 `/cmd_vel` 不能证明 dead，
  且可能承担既有飞行/遥控职责；按边界未删除。两个本轮 RL/Hector package 内不存在
  Hector `/cmd_vel` prototype 或第二套 PID/B-spline 实现。

除上述生成物/非 RL 范围外，本轮审计的 9 类权威 RL 源码项无其他 UNKNOWN。

## 6. 当前正式 training 主路径

```text
hector_worksite_sac_training.launch
  -> action_owner=sac
  -> sac_training_runner.py（唯一 SAC action/Replay/learner/checkpoint owner）
  -> AstraDroneEnv.run_episode()（10 Hz stamped identity/causal transitions）
  -> SpeedAdapter -> SpeedSafetyFilter -> stamped EGO dynamic v_max
  -> EGO -> traj_server -> position_command_to_hector
  -> training_episode_reset_coordinator.py（唯一正式 Episode/reset owner）
  -> controller stop/pause/zero-twist teleport/restart/generation barrier/warm-up
```

Training backend 与 PX4/MAVROS/FAST-LIO/full-stack launch/process/publisher 保持互斥。

## 7. 当前正式参数变化

**正式数值参数未发生变化。** 删除的是 dead profile、冗余开关和旧 fallback。

| 契约 | 最终值 |
|---|---:|
| valid transition target | 10000 |
| Replay capacity | 100000 |
| learning starts | 1000 |
| batch | 64 |
| critic-only startup | 100 updates |
| checkpoint steps | 5000 / 10000，显式各一次 |
| training evaluation | 无配置字段、无调用路径 |
| normal training stop | 只看 `valid_transition_count == 10000` |
| fixed Episode count in training | disabled，coordinator `episode_count=0` |
| abnormal Episode fail-safe | `max_training_episodes=1000`，不是正常 stop |
| Actor/Critic/alpha LR | `1e-5 / 1e-3 / 1e-3` |
| log-std | `[-3,-1]` |
| v_max | `[0.05,0.40] m/s` |
| random Hover reset | 当前正式 `+/-1 m` XY、z=3、yaw=0、seed=1001 |

Reward、Observation C、EGO、Hector PID、Learning Speed replan 阈值均未修改。

## 8. Training / evaluation 入口

`roslaunch --nodes` 对 training 和 evaluation 两种显式 mode 均成功展开相同的
training-only backend 节点集合，没有 PX4、MAVROS、FAST-LIO 或 ego bridge 节点。

`--dump-params` 结果：

- training：mode=training、target=10000、Replay=100000、learning_starts=1000、batch=64、
  critic warm-up=100、checkpoint=`[5000,10000]`、coordinator episode_count=0、
  random reset=true；
- evaluation：mode=evaluation、外部 checkpoint path/step 可传入、coordinator
  episode_count=3、random reset=false。

独立 evaluation 仍调用 `SacAgent.load_checkpoint()` 并核对 checkpoint 内
`global_environment_step` 与请求 step；离线 checkpoint save/load/state-equality 单测通过。

## 9. 测试结果

- Python compile：PASS（两个 package 的 src/scripts/tests，pycache 定向到 `/tmp`）。
- YAML parse：PASS，13 个相关 YAML。
- XML parse：PASS，相关 launch/test/package XML 全部通过 `xmllint --noout`。
- scoped Catkin build：PASS；最终 underlay 顺序为 ROS -> simulation -> Hector overlay ->
  Astra main workspace。
- Catkin tests：
  - `learning_speed_rl`：113 tests，0 errors，0 failures；系统 Python 下 3 个可选
    PyTorch tests initially skipped；
  - `hector_ego_training_backend`：25 tests，0 errors，0 failures；
  - 4 个 rostest：全部 PASS。第一次沙箱内运行被 `netifaces.interfaces()` 权限阻断，
    在允许本机 ROS interface discovery 后重跑通过，不属于代码失败。
- 可选 PyTorch：只读使用现有 training-only dependency 重跑 3/3 SAC tests，全部 PASS，
  包括 checkpoint round-trip/state equality。
- `roslaunch --nodes` / `--dump-params`：training/evaluation 两个 mode 均 PASS；仅静态
  展开，未启动节点。
- `FAST_LIO/Log/mat_pre.txt`：Git diff 为 0；未恢复、覆盖或写入。
- `runtime_artifacts/**/*.md`：最终为 0。
- `git diff --check`：PASS。

## 10. 是否启动过正式 training

**NO**

正式 evaluation 也未启动。没有启动 Gazebo、gzserver/gzclient、正式 SAC runner 或
Episode reset coordinator；只运行了 Python/unit/rostest、scoped build 与 roslaunch 静态解析。
