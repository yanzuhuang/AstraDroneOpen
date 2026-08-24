# Learning Speed Reward v2 论文对齐与离线审计

> 日期：2026-08-24  
> 结论：**REWARD AUDIT NO-GO（代码/离线一致性 PASS；正式训练 NO-GO）**  
> 边界：未启动 SAC training/evaluation，未修改 EGO、Hector PID、Observation C、reset、random Hover、动作范围、replan threshold 或 SAC 超参数。

## 1. 论文公式与旧 Reward 对比

论文 *Learning Speed Adaptation for Flight in Clutter* Eq. (6) 把 Reward 分为四个带符号分项：

```text
r = r_speed + r_smoothing + r_error + r_danger
```

其中 Eq. (7)--(9) 的后三项自身为非正值。项目记录也采用这一带符号形式，不再额外减一次。

| 项目 | 论文 | 旧 `astradrone_stage1_reward_v1.0` | 新 `astradrone_paper_guided_reward_v2.0` |
|---|---|---|---|
| `r_speed` | Eq. (10)/(11) 使用 `||v_t||` | Eq. (10) 分支错误使用 `applied_v_max` | 使用 `state_t.actual_velocity_body` 的范数 |
| `r_smoothing` | `-lambda_smoothing * (v_max_t-v_max_t-1)^2` | 使用 applied constraint | 保持；只用 stamped identity 链 canonical applied constraint |
| `r_error` | `-lambda_error * min(||e_t||,e_max)^2` | 缺失 | 使用现有 Observation C `tracking_error_body` 范数 |
| `r_danger` | terminal 时 `-lambda_danger*||v_t||^2` | frozen dangerous terminal + actual speed | 保持 |
| Stage 2 | Eq. (11) | 未实现 | Reward mode 已实现，未启动训练 |

论文未公开完整 `phi` 组合系数、`lambda_error`、`e_max` 或全部 Reward 权重。当前 `phi_1/phi_2`、平滑 blend 和全部数值只能称为 **paper-guided AstraDroneOpen implementation**，不能称为论文精确复现。

## 2. 实际修改内容

- 唯一公式 owner 收敛为 `training/reward.py::LearningSpeedReward.evaluate()`；删除旧 `SpeedRewardTerm`/Stage1 专用策略层。
- 一个实现支持显式 `stage_1` 和 `stage_2`；当前 YAML 仍选择 `stage_1`。
- online recorder、`AstraDroneEnv` 和 offline replay 都通过 `reward_input_from_signals()` 构造同一输入。
- `SacTransitionV1` 的分项合同新增 `reward_error`；仍禁止 `r_progress`、success bonus、clearance reward 和 planner shaping。
- 历史 frozen calibration/replay 不回写；新 replay 分析写入独立 JSON，不覆盖 v1 分析。
- 未加入 slew、low-pass、hysteresis、cooldown 或 action shaping。

## 3. actual speed 与 v_max 语义修正

新实现严格区分：

```text
actual speed = ||state_t.actual_velocity_body||
canonical constraint = action_t.applied_v_max
```

`actual speed` 只进入 `r_speed` 和 `r_danger`；canonical constraint 只进入 `r_smoothing`。单测固定 actual speed、改变 applied constraint 后，`r_speed` 完全不变；改变 actual speed 后 `r_speed` 才变化。

在 current-generation 正式范围样本上，旧/新离线重算的 `r_speed` 均值分别为 `-0.112543/-0.101674`；差异来自真实飞行速度通常低于 constraint，尤其转弯、加减速和跟踪阶段。这个变化是语义修复，不是权重调参。

## 4. tracking error 分布与参数标定

数据源是 current-generation merged CSV 中 `training_active=true`、`observation_valid=true`、目标范围 `0.30--1.75 m/s` 的 33,180 行。`tracking_error_norm_m` 已由同一 Observation C `tracking_error_body[3]` 生成；未新增第二套跟踪误差。

| count | mean | median | p95 | p99 | max |
|---:|---:|---:|---:|---:|---:|
| 33,180 | 0.12536 m | 0.10663 m | 0.26766 m | 0.39746 m | 0.79465 m |

选择 `e_max=0.40 m` 的依据是它近似 p99；仅 320/33,180（0.964%）样本被 clip，极端 outlier 不会二次方爆炸。

固定 `e_max=0.40 m` 后的候选尺度：

| `lambda_error` | median penalty | p95 penalty | p99 penalty | cap |
|---:|---:|---:|---:|---:|
| 0.5 | -0.0057 | -0.0358 | -0.0790 | -0.080 |
| 1.0 | -0.0114 | -0.0716 | -0.1580 | -0.160 |
| **2.0** | **-0.0227** | **-0.1433** | **-0.3159** | **-0.320** |
| 3.0 | -0.0341 | -0.2149 | -0.4739 | -0.480 |

推荐的离线候选是 `lambda_error=2.0, e_max=0.40 m`：典型惩罚较小，p95/cap 足以可见，但仍低于 Stage 1 low/high 主分支的全范围跨度 `1.16/1.45`。这两个值不是论文值，也尚不是获准正式训练的参数。

## 5. Stage 1 最终实现公式

```text
r_stage1 = r_speed_stage1 + r_smoothing + r_error + r_danger

r_smoothing = -0.10 * (applied_v_max_t - applied_v_max_t-1)^2
r_error     = -2.00 * min(||tracking_error_body_t||, 0.40 m)^2
r_danger    = -2.00 * ||actual_velocity_body_t||^2  if dangerous_terminal
              0                                     otherwise
```

`r_speed_stage1` 是 Eq. (10) 的连续 paper-guided 项目实现：用冻结 Candidate C 计算 `phi_1/phi_2`，在 safe/middle/dangerous 三个分支间 smoothstep blend，但每个分支都使用 actual speed：

```text
dangerous branch = 1.00 * (phi_1 - actual_speed)
safe branch      = 0.80 * (actual_speed - phi_1)
middle branch    = 0.25 * actual_speed
```

项目参数继续为 `lambda_phi_1/2=0.65/0.35`，anchors 为 Low/Medium/High/Unknown `1.75/1.25/0.75/1.25 m/s`。这些参数保留为历史 baseline，不代表作者公开参数。

## 6. Stage 2 候选公式与架构边界

```text
r_stage2 = 0.25 * actual_speed + r_smoothing + r_error + r_danger
```

Stage 2 仅切换 `r_speed`，其他三项复用同一 owner。当前 SAC 并非论文的共享 CNN encoder 架构，因此没有机械实现“冻结 CNN”。是否冻结当前网络的任何部分必须另行设计、审计和授权；本轮没有 Stage 2 training。

## 7. Reward landscape

下表把 actual speed 固定在用户指定网格；`r_error` 使用 current-generation 中同复杂度、同 fixed-speed cell 的 tracking-error median 作为代表值。所有行均为 steady constraint（`Delta applied_v_max=0`）、非 terminal，所以 `r_smoothing=0`、`r_danger=0`。这是一张受控 Reward surface，不宣称每个 fixed constraint 样本的 actual speed 恰等于表中值。

| complexity | actual m/s | tracking m | `r_speed` | `r_smoothing` | `r_error` | `r_danger` | total |
|---|---:|---:|---:|---:|---:|---:|---:|
| Low | 0.30 | 0.1123 | -1.1600 | 0 | -0.0252 | 0 | -1.1852 |
| Low | 0.50 | 0.1001 | -1.0000 | 0 | -0.0200 | 0 | -1.0200 |
| Low | 0.75 | 0.1213 | -0.8000 | 0 | -0.0294 | 0 | -0.8294 |
| Low | 1.00 | 0.1205 | -0.6000 | 0 | -0.0291 | 0 | -0.6291 |
| Low | 1.25 | 0.1216 | -0.4000 | 0 | -0.0296 | 0 | -0.4296 |
| Low | 1.50 | 0.1610 | -0.2000 | 0 | -0.0519 | 0 | -0.2519 |
| Low | 1.75 | 0.2080 | 0.0000 | 0 | -0.0865 | 0 | -0.0865 |
| Medium | 0.30 | 0.0738 | 0.0750 | 0 | -0.0109 | 0 | 0.0641 |
| Medium | 0.50 | 0.0971 | 0.1250 | 0 | -0.0189 | 0 | 0.1061 |
| Medium | 0.75 | 0.1006 | 0.1875 | 0 | -0.0202 | 0 | 0.1673 |
| Medium | 1.00 | 0.0992 | 0.2500 | 0 | -0.0197 | 0 | 0.2303 |
| Medium | 1.25 | 0.1246 | 0.3125 | 0 | -0.0311 | 0 | 0.2814 |
| Medium | 1.50 | 0.1416 | 0.3750 | 0 | -0.0401 | 0 | 0.3349 |
| Medium | 1.75 | 0.1782 | 0.4375 | 0 | -0.0635 | 0 | 0.3740 |
| High | 0.30 | 0.0854 | 0.4500 | 0 | -0.0146 | 0 | 0.4354 |
| High | 0.50 | 0.1004 | 0.2500 | 0 | -0.0202 | 0 | 0.2298 |
| High | 0.75 | 0.1061 | 0.0000 | 0 | -0.0225 | 0 | -0.0225 |
| High | 1.00 | 0.1187 | -0.2500 | 0 | -0.0282 | 0 | -0.2782 |
| High | 1.25 | 0.1614 | -0.5000 | 0 | -0.0521 | 0 | -0.5521 |
| High | 1.50 | 0.1590 | -0.7500 | 0 | -0.0505 | 0 | -0.8005 |
| High | 1.75 | 0.1779 | -1.0000 | 0 | -0.0633 | 0 | -1.0633 |

附加尺度：完整 constraint 跳变 `0.30 -> 1.75` 的 smoothing 为 `-0.21025`。danger terminal 在速度网格上的惩罚依次为 `-0.18/-0.50/-1.125/-2.00/-3.125/-4.50/-6.125`；非 dangerous terminal、success、ordinary truncation 和 reset 均为 0。

### A--F 回答

- A：是。Low 从 `-1.1852` 单调改善到 `-0.0865`，明确鼓励较高 actual speed。
- B：否。代表性 Medium 从 `0.0641` 单调升到 `0.3740`，没有内部“中速峰值”。这是 Eq. (10) middle branch 的结果，不应伪造一个中速 shaping 项。
- C：是。High 在 0.30 最高，超过 0.75 后转负，合理鼓励减速。
- D：不是所有环境都永远选择 0.30；Low/Medium 相反。但历史 10k NO-GO 的状态分布中 80.99% 的 Eq. (10) 反事实 `r_speed(1.75)-r_speed(0.30)` 为负，仍存在全局低速塌缩风险。
- E：是，但尺度受控。明显失配 `e>=0.40 m` 固定贡献 `-0.32`；例如 Medium/1.75 的 speed 奖励从 `0.4375` 降至最多 `0.1175`，Low/1.75 从 0 降至 `-0.32`。
- F：否。danger 只在 frozen dangerous terminal 出现，普通 step 完全为 0；它在高速 terminal 上强，但不会逐步压制全部探索。五个冻结 terminal scale probe 的总 Reward 均为负，最轻为 `-0.5364`。

## 8. 是否仍存在低速局部最优

存在，但不是全空间无条件存在：High 分支的局部最优就是下界 0.30。更严重的是历史 `sac_training_10k_20260823_194751` 的 10,000 条 transition：

- action 前 1,000 条 mean `0.2323 m/s`，后 1,000 条 mean `0.0728 m/s`（旧动作域 `[0.05,0.40]`）；
- 80.99% 状态的 Eq. (10) 反事实偏向较低速度，仅 19.01% 偏向较高速度；
- 旧 Reward mean `0.5616`，按新公式重算 mean `0.5985`，并没有逆转该状态分布的低速偏好；
- tracking error 在该 Hector run 极小（median `0.00360 m`、p95 `0.03218 m`），所以新增 `r_error` 无法解释或修复该次低速塌缩。

结论：actual-speed 语义修复是必须的，但它本身不足以证明低速偏置已消失。旧 4207-transition pilot 的真实 planner failure 与后续 10,000-transition NO-GO 也都不能被 Reward 重算改写为 PASS。

## 9. 新旧 Reward 对比

对 33,180 条正式范围 active-valid calibration rows 做只读重算：

| 指标 | 旧 v1 | 新 v2 candidate |
|---|---:|---:|
| speed term mean | -0.11254（applied constraint） | -0.10167（actual speed） |
| total mean | -0.11260 | -0.14432 |
| total median | 0.07500 | 0.04094 |
| total p05 / p95 | -1.1600 / 0.5780 | -1.2924 / 0.7180 |

新 total 的下降主要来自 `r_error` mean `-0.04259`；actual-speed 修正本身让 speed term mean 上升 `0.01087`。这两个效应不能混为一次“权重变化”。

旧训练向低速端塌缩与 Reward surface **可能相关且有定量支持**：历史状态分布大多落在保守分支，低速反事实优势很强。但策略/entropy/action exploration、planner failure 和状态访问分布也共同作用，所以不能把 Reward 宣称为唯一根因。

## 10. 参数分类与正式训练建议

### 保留的项目 baseline

- complexity/anchors：`6.0/2.5 m`、`0.040/0.080`、`1.75/1.25/0.75/1.25 m/s`；
- `lambda_phi_1/2=0.65/0.35`；
- `lambda_speed_1/2/3=1.00/0.80/0.25`；
- `lambda_smoothing=0.10`；
- `lambda_danger=2.00`。

它们有项目历史依据，本轮没有同时改动；但仍是 AstraDroneOpen baseline，不是论文值，也不构成正式训练授权。

### 仅候选参数

- **推荐离线候选：`lambda_error=2.00, e_max=0.40 m`**；已进入 v2 配置以便一致性验证，但尚未 runtime qualification。
- Stage 2 mode/公式：已实现，未获准 training。
- `lambda_speed_1=0.75/0.50` 的敏感性检查仍分别有 80.35%/78.82% 历史状态偏向低速，不能解决符号级偏置，故不推荐改动。

当前不建议启动正式 10,000-Episode training。原因不是代码不完整，而是 Reward v2 尚无 runtime qualification，且历史状态分布的低速偏置仍未消失；同时 Stage 2 网络冻结策略未决定。

## 11. 验证与仍需人工决定事项

验证结果：

- 108 个直接 unit tests PASS，3 个既有条件测试 skip；
- 12,209 条冻结 replay transition（formal 9,670、enhanced/repeat 2,402、safety-boundary 137）全部 finite；
- online/offline equality、actual speed、tracking clipping、smoothing、danger terminal、stage switch、terminal/truncated contract 均有定向测试；
- 5 个 frozen dangerous-terminal scale probe 总 Reward 全为负；
- 未启动 Gazebo、SAC training 或 evaluation。

仍需人工决定：

1. 是否接受 `lambda_error=2.0,e_max=0.40 m` 进入一次新的、受限的 Reward-only runtime qualification；
2. 中复杂度是否应忠实保留 Eq. (10) 的单调加速，还是另开研究问题调整 `phi`/branch 分布；
3. 如何处理历史状态分布 80.99% 的低速分支偏置；
4. Stage 2 对当前非论文 CNN 架构是否冻结任何网络部分。

在这些问题解决前：**正式 10,000-Episode training/evaluation 不允许启动。**
