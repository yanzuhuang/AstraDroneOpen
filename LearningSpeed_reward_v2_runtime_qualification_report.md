# Learning Speed Reward v2 Runtime Qualification

> 日期：2026-08-24  
> 最终结论：**REWARD V2 RUNTIME QUALIFICATION NO-GO**  
> 边界：fixed-speed、qualification-only；未启动 SAC training/evaluation，未创建 Replay，未修改 Reward、phi、EGO、Hector PID、Observation C、random Hover、SAC 或 `v_max=[0.30,1.75] m/s` 参数。

## 1. 最终结论

本轮复用现有 `worksite.world + Gazebo truth + Mid360 + Observation C + EGO + Hector + formal reset` fixed qualification。四档 `0.30/0.75/1.25/1.75 m/s` 各完成 5 个 Episode，共 20/20 success、20/20 reset；4161 个可评估 Reward runtime step 全部 finite，online/offline 逐行复算最大绝对差为 0。

`lambda_error=2.0,e_max=0.40 m` 没有压倒 `r_speed`。问题在复杂度访问分布和实际连续 branch blend：4161 step 中 Low 为 0，Medium 为 2032（48.83%），High 为 2129（51.17%）；按实际 branch weights，3443/4161（82.74%）由 dangerous branch 权重主导。所有 High 和 70.62% Medium 状态的反事实 `r_speed(1.75)-r_speed(0.30)` 为负，合计 3564/4161（85.65%）状态偏低速。

因此本轮不能证明“Low/open runtime 明显鼓励高速”，且实际 Medium 分布不符合离线 landscape 中 `phi_2=0.5` 代表点的单调加速结论。公式/实现一致，但代表性覆盖不足并存在明确低速塌缩风险，故判定 **NO-GO**，不允许进入 100-Episode SAC Stage 1 qualification。

## 2. 范围、方法与工件

fixed path 没有 `AstraDroneEnv` 的 0.1 s request/transition/Replay owner。本轮没有伪造 SAC transition，而是在原 `training_episode_reset_coordinator.py` 中加入默认关闭的 Reward-only 审计开关：每个 active、valid Observation C 样本定义为一个 `reward_runtime_step`，直接调用唯一 `LearningSpeedReward.evaluate()`。Episode 终态只绑定到最后一个可评估 step。

正式计数工件：

| fixed `v_max` | runtime 目录 | Episode | Reward steps |
|---:|---|---:|---:|
| 0.30 | `runtime_artifacts/reward_v2_runtime_qualification/20260824_q03_v030_formal55_retry/` | 5 | 2258 |
| 0.75 | `runtime_artifacts/reward_v2_runtime_qualification/20260824_q04_v075/` | 5 | 914 |
| 1.25 | `runtime_artifacts/reward_v2_runtime_qualification/20260824_q05_v125/` | 5 | 585 |
| 1.75 | `runtime_artifacts/reward_v2_runtime_qualification/20260824_q06_v175/` | 5 | 404 |

聚合原始统计为 `runtime_artifacts/reward_v2_runtime_qualification/20260824_reward_v2_runtime_aggregate_analysis.json`。每个有效目录包含 `reward_runtime_steps.jsonl`、`episode_results.json`、`reset_results.json`、`qualification_summary.json`、`qualification_events.jsonl`、ROS logs 和 console log；`runtime_artifacts/` 中未新建 Markdown。

另保留两个不计数尝试：

- `20260824_q01/v030`：0 Episode 前因受限 ROS `netifaces.interfaces()` 权限失败，属于基础设施无效；
- `20260824_q02_v030_infrastructure_retry`：误用 45 s 而非当前 formal 55 s Episode 上限，Episode 1 形成 `max_episode_time` truncation 后人工停止；配置不等价，未纳入 20 Episode。随后以新 ID 和 55 s 上限重跑，原工件未覆盖。

## 3. 4 档 × 5 Episode 结果

`start` 中 `N` 表示 nominal Hover，`R` 表示 seed 1001 random Hover。`hold` 是既有 fixed-only terminal convergence guard；它不放宽 Observation C，也不用于 SAC action owner。

| `v_max` | Ep | start | outcome | Reward steps | Obs C valid | actual median / p95 m/s | tracking p95 m | mean `r_speed` | mean `r_error` | mean total | hold |
|---:|---:|:---:|---|---:|---:|---:|---:|---:|---:|---:|:---:|
| 0.30 | 1 | N | SUCCESS | 447 | 1.0000 | 0.1970 / 0.2133 | 0.0220 | 0.5188 | -0.000254 | 0.5186 | no |
| 0.30 | 2 | R | SUCCESS | 502 | 1.0000 | 0.1986 / 0.2058 | 0.0142 | 0.5098 | -0.000158 | 0.5097 | no |
| 0.30 | 3 | R | SUCCESS | 440 | 1.0000 | 0.1925 / 0.2097 | 0.0177 | 0.6085 | -0.000182 | 0.6083 | no |
| 0.30 | 4 | R | SUCCESS | 393 | 1.0000 | 0.1725 / 0.2122 | 0.0167 | 0.6385 | -0.000186 | 0.6383 | no |
| 0.30 | 5 | R | SUCCESS | 476 | 1.0000 | 0.1976 / 0.2073 | 0.0156 | 0.4481 | -0.000170 | 0.4479 | no |
| 0.75 | 1 | N | SUCCESS | 181 | 1.0000 | 0.4923 / 0.5304 | 0.0596 | 0.3268 | -0.001538 | 0.3253 | no |
| 0.75 | 2 | R | SUCCESS | 195 | 1.0000 | 0.4909 / 0.5346 | 0.0380 | 0.3346 | -0.001009 | 0.3336 | no |
| 0.75 | 3 | R | SUCCESS | 171 | 1.0000 | 0.4879 / 0.5401 | 0.0397 | 0.3578 | -0.001089 | 0.3568 | no |
| 0.75 | 4 | R | SUCCESS | 158 | 1.0000 | 0.4648 / 0.5280 | 0.0456 | 0.4036 | -0.001130 | 0.4025 | no |
| 0.75 | 5 | R | SUCCESS | 209 | 1.0000 | 0.4742 / 0.5239 | 0.0362 | 0.3079 | -0.001063 | 0.3069 | no |
| 1.25 | 1 | N | SUCCESS | 112 | 0.8819 | 0.7836 / 0.9984 | 0.1341 | 0.1514 | -0.006345 | 0.1450 | yes |
| 1.25 | 2 | R | SUCCESS | 125 | 0.9615 | 0.8051 / 0.9237 | 0.1034 | 0.1810 | -0.003169 | 0.1779 | yes |
| 1.25 | 3 | R | SUCCESS | 110 | 0.9649 | 0.7875 / 0.9665 | 0.1146 | 0.1069 | -0.003918 | 0.1030 | yes |
| 1.25 | 4 | R | SUCCESS | 109 | 0.9646 | 0.7229 / 1.0119 | 0.1185 | 0.2127 | -0.004639 | 0.2081 | yes |
| 1.25 | 5 | R | SUCCESS | 129 | 0.9021 | 0.7715 / 0.9627 | 0.1069 | 0.1269 | -0.003726 | 0.1231 | yes |
| 1.75 | 1 | N | SUCCESS | 86 | 0.8515 | 0.9533 / 1.6097 | 0.1905 | -0.0635 | -0.020619 | -0.0841 | yes |
| 1.75 | 2 | R | SUCCESS | 89 | 0.9468 | 1.0742 / 1.5149 | 0.1676 | 0.0149 | -0.013153 | 0.0018 | yes |
| 1.75 | 3 | R | SUCCESS | 80 | 1.0000 | 1.0412 / 1.4119 | 0.1503 | -0.1724 | -0.011086 | -0.1835 | no |
| 1.75 | 4 | R | SUCCESS | 63 | 1.0000 | 1.1050 / 1.5810 | 0.1928 | -0.1750 | -0.020135 | -0.1951 | no |
| 1.75 | 5 | R | SUCCESS | 86 | 0.8515 | 1.0678 / 1.4773 | 0.1733 | -0.1256 | -0.011761 | -0.1374 | yes |

20 个终态全部为 `SUCCESS/entry_gate_reached`。各档均为 5/5 reset success，合计 planner failure=0、collision=0、controller failure=0。全窗口 Observation C 为 4161/4238=0.98183；77 个 invalid 只出现在 1.25/1.75 的既有 terminal convergence，未进入 Reward step。

## 4. Reward 分项与四档分布

下表使用 `[p05, median, p95]`；所有统计来自实际 speed，不把 fixed `v_max` 当实际速度。

| `v_max` | actual speed mean / p95 | `r_speed` mean `[p05, med, p95]` | `r_error` mean `[p05, med, p95]` | total mean `[p05, med, p95]` |
|---:|---:|---:|---:|---:|
| 0.30 | 0.1572 / 0.2092 | 0.5402 `[-0.1571,0.6355,0.8443]` | -0.000189 `[-0.000591,-0.000028,-0.000002]` | 0.5400 `[-0.1575,0.6354,0.8443]` |
| 0.75 | 0.3905 / 0.5332 | 0.3432 `[-0.1039,0.3616,0.5995]` | -0.001162 `[-0.005408,-0.000196,-0.000005]` | 0.3421 `[-0.1065,0.3616,0.5993]` |
| 1.25 | 0.6146 / 0.9678 | 0.1554 `[-0.1665,0.0985,0.6250]` | -0.004315 `[-0.029781,-0.001097,-0.000039]` | 0.1511 `[-0.1707,0.0913,0.6237]` |
| 1.75 | 0.8989 / 1.5172 | -0.0984 `[-0.5872,-0.1766,0.5506]` | -0.015125 `[-0.059455,-0.007898,-0.000372]` | -0.1135 `[-0.5961,-0.1889,0.5394]` |

全体 total 范围为 `[-0.86339,0.92308]`，mean/median 为 `0.37841/0.54764`。fixed constraint 稳态下 `r_smoothing` 仅有浮点噪声（绝对值不超过 `1.42e-17`）；没有 dangerous terminal，因此 `r_danger=0`。

## 5. Branch 占比与实际权重

### 5.1 离散复杂度 label

| `v_max` | Low | Medium | High | Unknown |
|---:|---:|---:|---:|---:|
| 0.30 | 0 | 1066（47.21%） | 1192（52.79%） | 0 |
| 0.75 | 0 | 454（49.67%） | 460（50.33%） | 0 |
| 1.25 | 0 | 291（49.74%） | 294（50.26%） | 0 |
| 1.75 | 0 | 221（54.70%） | 183（45.30%） | 0 |
| 合计 | **0（0%）** | **2032（48.83%）** | **2129（51.17%）** | **0（0%）** |

### 5.2 连续 Eq. (10) branch weights

实际实现不是只按 label 选择一个离散分支，而是按 `phi_2` smoothstep blend。按最大权重分类：safe 94（2.26%）、middle 624（15.00%）、dangerous 3443（82.74%）；平均 weights 为 safe/middle/dangerous=`0.03024/0.14027/0.82950`。因此 `Medium` label 不等于 pure middle branch。

## 6. Tracking error 与 `r_error` 尺度

- tracking error 随 fixed speed 增大：p95 为 `0.0172/0.0520/0.1220/0.1724 m`，最大值为 `0.0658/0.1041/0.1507/0.2056 m`；4161 step 均低于 `e_max=0.40 m`，clip count=0。
- 全体 `|r_error|` mean/p95/max 为 `0.00243/0.01358/0.08454`；`|r_speed|` mean/p95/max 为 `0.46862/0.79160/0.92361`。
- 在 `|r_speed|>=0.05` 的 3912 step 中，`|r_error|/|r_speed|` median/p95 为 `0.000108/0.05118`。
- 只有 22/4161（0.529%）step 的 `|r_error|>|r_speed|`，均发生在 `r_speed` 接近零的位置；没有持续或分布级 error domination。

结论 A：`lambda_error=2.0,e_max=0.40 m` 在本轮 runtime **没有导致 tracking penalty 尺度过大**，不构成 NO-GO 原因。

## 7. Low / Medium / High 速度偏好

对每个实际 runtime 状态固定复杂度与 tracking error，只把 actual speed 反事实设为 0.30 和 1.75，比较 `Delta r_speed = r_speed(1.75)-r_speed(0.30)`：

| label | count | positive | negative | mean Delta | 结论 |
|---|---:|---:|---:|---:|---|
| Low | 0 | 0 | 0 | NA | runtime 未覆盖，不能证明开阔态鼓励高速 |
| Medium | 2032 | 597（29.38%） | 1435（70.62%） | -0.7678 | 实际 blend 多数偏低速，与 pure-middle 代表点不一致 |
| High | 2129 | 0 | 2129（100%） | -1.4500 | 明确鼓励低速 |

结论 D：公式/离线 Low surface 鼓励高速，但本轮 Low=0，**runtime 证据不足，PASS 门槛未满足**。

结论 E：High 的 2129/2129 状态均明确偏低速，PASS。

结论 F：online/offline 对同一 runtime state 完全一致；但旧离线 landscape 的代表性 Medium 使用 `phi_2=0.5`、pure middle weight=1。实际 Medium 中 70.62% 由连续 blend 得到负速度斜率，所以 runtime **不支持把整个 Medium label 写成单调奖励速度**。这是状态/label/weight 覆盖问题，不是第二套公式或在线实现漂移。

全体 3564/4161（85.65%）状态偏向低端，仅 597/4161（14.35%）偏向高端；比此前历史状态分布 80.99% 的低速偏向更强，存在明确低速塌缩风险。

## 8. Danger、finite 与异常值

结论 G：20 个 terminal step 全部是成功终态，`dangerous_terminal=false`；4161/4161 的 `r_danger=0`，映射 mismatch=0。它证明了 runtime negative control（普通/成功终态不误触发），没有人为制造 collision/emergency 来取得 positive runtime probe。定向单测已覆盖 dangerous terminal 使用 state_t actual speed 平方和非 dangerous terminal 为零。

结论 H：4161/4161 Reward 全部 finite，NaN/Inf=0；online/offline numeric mismatch=0、branch mismatch=0、component-sum mismatch=0。`|total|>2` 和 `|total|>10` 均为 0。绝对值最大分项为 `r_speed` 的 step 占 4139/4161（99.47%），为 `r_error` 的只有 22/4161（0.53%）；fixed 稳态使 smoothing 近零、无 danger 使 danger 为零，这不是异常值。没有 `r_error` 压倒总 Reward，但主速度项在本固定动作实验中按设计占主导。

## 9. PASS / NO-GO 门逐项判定

| 门 | 结果 | 证据 |
|---|---|---|
| Reward online/offline 语义一致 | PASS | 4161/4161，max abs diff 0 |
| 20 Episode 运行链稳定 | PASS | 20/20 success，20/20 reset |
| `r_error` 不压倒 `r_speed` | PASS | mean `0.00243` vs `0.46862`；仅 0.529% 局部反超 |
| `r_danger` 只在正确 terminal | PASS（negative control） | 20 success terminal 全为 0；无 dangerous runtime event |
| Low 总体倾向高速 | **未满足** | Low runtime count=0 |
| High 总体倾向低速 | PASS | 2129/2129 反事实为负 |
| Medium 与离线代表 landscape 一致 | **未满足** | 70.62% runtime Medium 为负斜率 |
| Reward finite / 无极端异常 | PASS | 4161/4161 finite；total 范围 `[-0.863,0.923]` |
| Reward 未引入 planner/collision/controller/reset 异常 | PASS | 0/0/0；20/20 reset |

总判定：**REWARD V2 RUNTIME QUALIFICATION NO-GO**。

## 10. 建议与下一步边界

本轮不修改参数。建议在任何 SAC qualification 前先：

1. 只读解释 worksite ENTRY route 为什么没有产生 Low label，并按 `phi_2` 与 safe/middle/dangerous weights 分层，不再把整个 Medium label 当作 pure middle branch；
2. 设计新的、显式授权的 bounded Reward-only coverage qualification，要求产生足够 Low/open runtime state，同时保留当前公式、Observation C、EGO、Hector PID、random Hover 和动作范围；
3. 在覆盖 Low 与 Medium weight strata 后重新检查状态级反事实偏好和低速塌缩比例；
4. dangerous positive runtime 不应通过故意制造碰撞取得；继续依靠冻结真实 dangerous terminal 和定向测试，或另行设计不会扩展飞行风险的安全注入审计。

当前不建议、也不允许进入 100-Episode SAC Stage 1 qualification；正式 10000-Episode training 和正式 evaluation 继续保持禁止。
