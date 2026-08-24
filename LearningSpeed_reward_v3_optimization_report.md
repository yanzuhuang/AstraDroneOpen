# Learning Speed Reward v3 优化与离线审计

> 日期：2026-08-24  
> 结论：**REWARD V3 OFFLINE PASS；允许进入新的 bounded Reward-only runtime qualification；正式 SAC training/evaluation 仍为 NO-GO**  
> 边界：本轮仅由包级 rostest 临时启动测试 ROS master；未启动 Gazebo、SAC training、100-Episode qualification、10000-Episode formal training 或 evaluation；未修改 EGO、Hector PID、Observation C、`v_max=[0.30,1.75] m/s`、reset、random Hover、SAC 超参数或 replan threshold。

## 1. Reward v2 问题

Reward v2 的 Eq. (6) 四分项、actual-speed、tracking-error、smoothing 和 dangerous-terminal 语义均已通过 unit/offline/runtime 一致性审计；本轮不改这些结构。真正的问题在 Stage 1 complexity 到 speed reward 的链：

```text
q_N = clip01((6.0-N)/3.5)
q_D = clip01((D-0.040)/0.040)
phi_2 = max(q_N,q_D)
phi_1 = 1.75-phi_2
```

在 Reward v2 的 4161 个 Hover→ENTRY runtime step 中，`q_N>q_D` 为 4161/4161，density 对最终 `phi_2` 的数值贡献为 0。另一方面，v2 smoothstep branch 的速度斜率在 `phi_2≈0.5431` 已转负，而诊断 `Medium` 仍覆盖整个 `0<phi_2<1`；因此 1109 个 `Medium` step 已是 100% dangerous branch。label 与连续计算语义错位，历史 10k 状态也有 80.99% 的严格反事实偏低速。

## 2. Open / Medium / High 数据来源

所有源均只读，历史工件未回写。为了让 v2/v3 使用相同分母比较，离线标定使用 v2 的物理 anchor 仅作共同诊断 strata；label 不进入 v3 Reward 计算。

| 数据 | 用途 | 有效状态 | Open | Medium | High |
|---|---|---:|---:|---:|---:|
| Environment B 六个正式完整任务 `B_0.30–B_1.50` | 真实 worksite Low/open 与完整任务对照 | 14784 | 2102 | 7167 | 5515 |
| Reward v2 fixed runtime 四档正式 run | Hover→ENTRY transition/clutter | 4161 | 0 | 2032 | 2129 |
| `sac_training_10k_20260823_194751` | 历史低速塌缩反事实 | 10000 | 0 | 8354 | 1646 |

fusion 拟合按 class-balanced MSE 使用 Open 2102、Medium 9199（完整任务 Medium + runtime Medium）、High 2129（Hover→ENTRY runtime High），避免大样本类别单纯靠计数压倒其他 strata。完整来源、allowlist 与 SHA-256 记录在 `runtime_artifacts/reward_v3_offline_optimization/20260824_reward_v3_offline_analysis.json`；该目录只含 JSON，没有 Markdown。

## 3. `q_nearest` / `q_density` 分布与 surrogate 解释

| 来源/stratum | `q_N` median / p95 | `q_D` median / p95 | `q_N>=q_D` |
|---|---:|---:|---:|
| 完整任务 Open | 0 / 0 | 0 / 0 | 100%（两者相等） |
| 完整任务 Medium | 0.4766 / 0.9336 | 0.0234 / 0.6094 | 82.42% |
| 完整任务 High | 1 / 1 | 0.5938 / 1 | 96.74% |
| Hover→ENTRY Medium | 0.6895 / 0.9849 | 0.1484 / 0.2891 | 100% |
| Hover→ENTRY High | 1 / 1 | 0.4844 / 0.6719 | 100% |
| 历史 10k Medium | 0.6387 / 0.9580 | 0.1641 / 0.6172 | 97.80% |

当前 `N` 是 3200-bin surrogate 中 known-obstacle probe 的最小距离；`D=K/3200`，其中 `K` 是 occupied angular-bin count。它们可作为论文“最近障碍距离 + 障碍密度/数量”的 AstraDroneOpen surrogate，但 `D` 不是物体实例数、3D volume fraction 或 EGO inflated clearance，不能声称论文精确复现。

本轮比较了 nearest safe `5.5/6.0/6.5 m`、nearest dangerous `2.0/2.5/3.0 m`，以及 density `[0.035,0.075]`、`[0.040,0.070]`、`[0.040,0.080]`。虽然同时移动两个端点或把 nearest safe 改为 6.5 能降低 label-target MSE，但会把已有真实 Open 样本重新映射为非零风险，且存在用旧 label 目标自我优化的问题。对最终 geometric fusion，保留 `[6.0,2.5] m` 与 `[0.040,0.080]` 的 balanced MSE 为 `0.024111`，略优于只把 density dangerous 改为 0.070 的 `0.024516`。因此 normalization 与 threshold 保留；density 弱表达的主因确认是 max fusion 压制，而不是 3200 分母本身失效。

## 4. 为什么不能继续使用 max fusion

`max(q_N,q_D)` 是连续函数，但每个状态只保留更大通道。Hover→ENTRY 的 4161 个状态全部由 nearest 独占，density 即使与路线变密显著相关也无法改变 `phi_2`。其 class-balanced MSE 为 `0.027043`，同时历史 10k 的低速反事实比例没有改善。

这不是说 nearest 不重要：Hover→ENTRY High 的确全部满足 nearest high。问题是 max 把“某一通道更大”错误扩大成“另一通道完全没有数值贡献”，无法表达两个风险同时渐增。

## 5. fusion 候选比较

| 候选 | normalization | 拟合/公式 | balanced MSE | 关键结果 |
|---|---|---|---:|---|
| A：v2 max | 原 `[6,2.5]` / `[.04,.08]` | `max(q_N,q_D)` | 0.027043 | runtime density 4161/4161 不参与最终值 |
| B1：convex linear | 原 normalization | `0.84q_N+0.16q_D` | 0.025790 | 两通道参与，但单通道到 1 时不能到 High anchor |
| B2：convex linear | density high=0.070 | `0.78q_N+0.22q_D` | 0.024596 | 分数更低，但仍破坏单特征 High endpoint，并多改一个 threshold |
| C1：unweighted probabilistic OR | 原 normalization | `1-(1-q_N)(1-q_D)` | 0.030385 | 保留端点，但整体 phi 偏高、低速偏置加重 |
| **C2：weighted geometric survival** | **原 normalization** | **`1-(1-q_N)^0.46(1-q_D)^0.54`** | **0.024111** | 保留 Open/High 端点、两通道连续、历史偏置显著下降 |

权重不是经验设定：在 `w_N+w_D=1`、两者均为正的约束下，以 0.01 网格对三个 class 等权拟合 `Open→0, Medium→0.5, High→1`，最优为 `w_N=0.46,w_D=0.54`。该 fusion 还满足：

```text
q_N=q_D=q  => phi_2=q
q_N=0,q_D=0 => phi_2=0
q_N=1 or q_D=1 => phi_2=1
```

在 9199 个 Medium 标定状态中，density 对 `phi_2` 有非零增量的比例为 64.40%；Open 两项均为 0，High 中 nearest 已饱和时 density 增量为 0，这是端点饱和语义，不是 max 式的全路线永久失效。

## 6. 最终 `phi_2` 公式

```text
q_N = clip01((6.0 m - N) / (6.0 m - 2.5 m))
q_D = clip01((D - 0.040) / (0.080 - 0.040))

phi_2 = 1 - (1-q_N)^0.46 * (1-q_D)^0.54
```

Unknown-majority 继续使用冻结值 `phi_2=0.5`，不伪装为 Open。Low/Medium/High label 仍按端点生成，仅用于日志/报告，不选择公式分支。

## 7. `phi_1` 与 anchor

保留：

```text
phi_1 = 1.75 - phi_2
```

因此 `phi_2=0/0.5/1` 对应 `phi_1=1.75/1.25/0.75 m/s`，与当前动作范围 `[0.30,1.75] m/s`、既有 Low/Medium/High anchor 历史一致；Unknown 仍为 `1.25 m/s`。现有数据没有支持调整这些 anchor，故不为了降低偏置同时漂移 anchor。

## 8. Reward v3 Stage 1 完整公式

保持论文 Eq. (6) 的带符号结构：

```text
r = r_speed + r_smoothing + r_error + r_danger
```

令 `p=phi_2`，v3 的连续 branch 权重为 quadratic Bernstein basis：

```text
w_safe      = (1-p)^2
w_middle    = 2p(1-p)
w_dangerous = p^2
```

Stage 1 speed term 为：

```text
r_safe      = 0.80 * (actual_speed - phi_1)
r_middle    = 0.25 * actual_speed
r_dangerous = 1.00 * (phi_1 - actual_speed)

r_speed = w_safe*r_safe + w_middle*r_middle + w_dangerous*r_dangerous
```

对 actual speed 的斜率是：

```text
d(r_speed)/d(actual_speed)
  = 0.80(1-p)^2 + 0.50p(1-p) - p^2
  = 0.80 - 1.10p - 0.70p^2
```

它连续单调下降，并在 `p=0.5410125` 穿过 0；不再需要 `0.35/0.50/0.65` 的离散 blend threshold，也不会出现 `Medium label` 控制 dangerous branch 的语义。

冻结的其他分项完全不变：

```text
r_smoothing = -0.10 * (applied_v_max_t-applied_v_max_t-1)^2
r_error     = -2.0 * min(||tracking_error_body||,0.40)^2
r_danger    = -2.0 * ||actual_velocity_body||^2  if dangerous_terminal
              0                                  otherwise
```

未加入 progress、success、clearance、tracking progress 或 planner shaping。

## 9. v2 vs v3 Reward landscape

下表不是构造一个“Medium 唯一最优速度”，而是在 18945 个 current real state（完整任务 + Hover→ENTRY）上固定复杂度，扫描 actual speed，并报告各 stratum 的 mean `r_speed`。`r_error/r_smoothing/r_danger` 不改变速度斜率，故此处只比较 speed term。

| stratum | version | 0.30 | 0.50 | 0.75 | 1.00 | 1.25 | 1.50 | 1.75 m/s |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| Open | v2/v3 | -1.160 | -1.000 | -0.800 | -0.600 | -0.400 | -0.200 | 0.000 |
| Medium | v2 | -0.040 | -0.063 | -0.091 | -0.120 | -0.148 | -0.177 | -0.205 |
| Medium | **v3** | **-0.223** | **-0.178** | **-0.122** | **-0.066** | **-0.010** | **0.046** | **0.102** |
| High | v2/v3 | 0.450 | 0.250 | 0.000 | -0.250 | -0.500 | -0.750 | -1.000 |

Open 明确奖励更高 actual speed；High 明确奖励更低 actual speed；Medium 的总体 slope 从 v2 的错误负均值变为 v3 正均值，但不是每个 Medium 状态都偏高速。真实 route 的 v3 `phi_2` median 从 start `0.3379`、middle `0.8453` 到 ENTRY `1.0`，连续跨过零斜率点；start p99 `0.5453` 直接覆盖接近零的 transition 区域。

## 10. v2 vs v3 低速偏置

定义：

```text
Delta = r_speed(actual_speed=1.75)-r_speed(actual_speed=0.30)
Delta > +0.01 : 偏高速
|Delta|<=0.01 : 近中性
Delta < -0.01 : 偏低速
```

| 数据/stratum | version | 偏高速 | 近中性 | 偏低速 |
|---|---|---:|---:|---:|
| 历史 10k / ALL | v2 | 1897 (18.97%) | 9 (0.09%) | 8094 (80.94%) |
| 历史 10k / ALL | **v3** | **5566 (55.66%)** | **129 (1.29%)** | **4305 (43.05%)** |
| current pooled / ALL | v2 | 6935 (36.61%) | 6 (0.03%) | 12004 (63.36%) |
| current pooled / ALL | **v3** | **8576 (45.27%)** | **63 (0.33%)** | **10306 (54.40%)** |
| current pooled / Open | v3 | 2102 (100%) | 0 | 0 |
| current pooled / Medium | v3 | 6474 (70.38%) | 63 (0.68%) | 2662 (28.94%) |
| current pooled / High | v3 | 0 | 0 | 7644 (100%) |
| Hover→ENTRY / ALL | v2 | 597 (14.35%) | 2 (0.05%) | 3562 (85.60%) |
| Hover→ENTRY / ALL | **v3** | **1238 (29.75%)** | **17 (0.41%)** | **2906 (69.84%)** |

历史 10k 的严格 `<0` 比例由 v2 的 80.99% 降到 v3 的 43.76%；上表按预注册 `±0.01` near-zero 带分类。v3 不再表现为“几乎所有历史训练状态都偏好 0.30”，同时没有把 Hover→ENTRY 的真实 clutter 强行改成整体偏高速：该路线仍有 2129/4161 High，且 v3 High 2129/2129 偏低速。

## 11. 为什么选择该方案

选择 weighted geometric survival + Bernstein branch，原因是：

1. 公式短、无新特征、无学习模块，且每项单调连续；
2. `0.46/0.54` 来自明确数据 strata 的约束拟合，不是 0.5/0.5 猜测；
3. 保留原 normalization 与 `1.75/1.25/0.75` anchor，避免同时漂移多个定义；
4. 任一风险端点都能到 `phi_2=1`，优于 linear fusion 的 anchor 破坏；
5. density 在非饱和 Medium 状态实际参与，不再被 max 长期丢弃；
6. 速度斜率从正到负连续穿越 0，不依赖诊断 label；
7. 历史 10k 低速塌缩风险显著下降，但当前 clutter route 的低速偏好仍真实保留。

这仍是 paper-guided AstraDroneOpen implementation，不是论文完整特征或参数复现。

## 12. 代码与配置修改

- `training/reward.py`：version 更新为 `astradrone_paper_guided_reward_v3.0`；唯一 owner 实现 weighted-geometric `phi_2` 和 Bernstein continuous branch；保留 Eq. (6) 其他分项。
- `config/stage1_reward.yaml`：写入 v3 version 与 `fusion.nearest_weight=0.46/density_weight=0.54`；删除 v2 的 `0.35/0.65` branch threshold；其他冻结参数不变。
- `scripts/analyze_reward_v3.py`：新增可复核离线候选、landscape、counterfactual 和 owner-equality 审计。
- `CMakeLists.txt`：把新的离线审计脚本纳入 package 可执行脚本安装清单。
- `scripts/replay_stage1_reward.py`：默认生成 v3 replay JSON，并让 continuity probe 覆盖完整 `phi_2=[0,1]`。
- `scripts/summarize_manual_calibration.py`：兼容读取 v3 version，同时保留 v1/v2 历史兼容。
- `test/test_stage1_reward.py`：新增 fusion、归一化、anchor、全域 continuity、斜率过零和 weight fail-closed 测试；保留 actual-speed、tracking、smoothing、danger、Stage 1/2、online/offline equality 与 finite coverage。
- 同步更新 package README、`studynote.md`、`AGENTS.md` 与长期技术汇总；v2 三份报告继续作为冻结历史证据，不回写。

## 13. 测试与离线结果

| 验证 | 结果 |
|---|---|
| Reward 定向 unit | 21/21 PASS |
| `learning_speed_rl` Catkin package | 119 tests，0 error，0 failure，3 个既有条件 skip |
| v3 frozen replay | 9670 formal + 2402 enhanced/repeat + 137 safety-boundary = 12209，全部 finite/PASS |
| dangerous terminal scale | 5/5 总 Reward 为负 |
| owner vs 独立 v3 公式 | 28945 个真实状态 × 7 speed = 202615 次；max abs diff `4.44e-16` |
| offline optimization gates | 7/7 PASS |
| Python syntax | `reward.py`、analysis/replay/summarizer 全部 `py_compile` PASS |

第一次包测试在受限沙箱内有 4 个 rostest 因 `netifaces.interfaces(): PermissionError` 无法枚举本机接口；这不是 Reward 失败。按相同源码和参数在获准的本机网络权限下重跑后，四个 rostest 均 PASS，最终 Catkin 结果为 119/119 无 failure（3 个既有条件 skip）。

派生 JSON：

- `runtime_artifacts/reward_v3_offline_optimization/20260824_reward_v3_offline_analysis.json`；
- `runtime_artifacts/learning_speed/stage1_reward_calibration_current_generation_20260820_231428/paper_guided_reward_v3_replay_analysis.json`。

## 14. 是否允许进入 Reward v3 runtime qualification

**允许，但只允许新的 bounded Reward-only runtime qualification。** 建议复用既有 fixed Reward-only 框架，以新 ID 明确覆盖：

- 完整任务或独立片段中的真实 Open；
- `phi_2` 穿过约 `0.54` 的 transition；
- 当前 Hover→ENTRY High/clutter；
- online/offline equality、finite、actual-speed、tracking-error、danger negative-control；
- 真实 planner/collision/controller/Observation/reset failure 原样保留。

这不是 100-Episode SAC qualification 授权，更不是 10000-Episode training/evaluation 授权。Reward v3 在新的 runtime 证据完成前，正式 SAC training/evaluation 继续 **NO-GO**。
