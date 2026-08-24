# Learning Speed phi / branch 分布诊断

> 日期：2026-08-24  
> 根因分类：**A. ROUTE COVERAGE ISSUE**  
> 边界：纯离线、只读；未启动 ROS/Gazebo/SAC training/evaluation，未修改 Reward、`phi1/phi2`、normalization、threshold、anchor、lambda、EGO、Hector PID、Observation C、`v_max` 或 reset。

## 1. 最终结论

当前 worksite Hover→ENTRY route 的 Low=0 和 85.65% 状态偏低速，主因是**路线本身沿松树走廊持续进入近障区域，缺少满足当前 Low/open 定义的覆盖**，不是现有证据所支持的“真实开阔状态被错误映射到危险侧”。

4161 个 runtime step 中：

- `nearest_obstacle_distance_m` 只有 `1.4366–4.6899 m`，0 step 达到 Low 所需 `N>=6.0 m`；
- occupied angular-bin count 为 `130–231`，density 为 `0.040625–0.0721875`，0 step 达到 Low 所需 `D<=0.040`（3200 bins 下等价于 occupied bins `<=128`）；
- `nearest_risk` 在 4161/4161 step 中都严格大于 `density_risk`，最小领先仍为 0.10687，故 `phi2=max(q_N,q_D)` 实际完全由 nearest 通道拥有；
- 2129 个 High 全由 `N<=2.5 m` 触发并使 nearest risk clip/saturate 到 1；density 从未达到 `D>=0.080`，也从未发生 density high saturation；
- `phi2` 的 median/p95/max 均为 1，路线后段是确定性的高复杂度覆盖。

Medium 中 70.62% 偏低速，不是 smoothstep 算错，而是两个事实叠加：

1. `Medium` label 覆盖所有非端点复杂度，即 `0<phi2<1`，因此允许 `phi2>=0.65` 时仍叫 Medium，但 continuous weight 已是 100% dangerous；
2. runtime Medium 的 `phi2` median/p95 为 `0.6895/0.9849`，1109/2032（54.58%）已在 `phi2>=0.65` 的 full-dangerous 区域，另有 310 step 位于 0.55–0.65 的 dangerous-side blend。

当前不需要修改 phi。下一步应先做新的 Reward-only route coverage qualification，把明确开阔 segment 与当前 ENTRY clutter segment 组合，验证 Low/Medium/High 三类覆盖；在这之前不应调 normalization 或 threshold。

## 2. 数据与可复核输出

主要数据是 Reward v2 fixed qualification 的 4161 个 active-valid runtime step：

- `0.30 m/s`：2258；
- `0.75 m/s`：914；
- `1.25 m/s`：585；
- `1.75 m/s`：404。

本轮导出均为非 Markdown runtime 证据：

- `runtime_artifacts/reward_v2_phi_branch_diagnosis/20260824_d01/phi_branch_distribution_summary.json`：聚合统计与根因；
- `.../phi_branch_derived_steps.csv`：4161 个逐步派生值；
- `.../medium_phi2_layers.csv`：Medium 分层；
- `.../relative_local_open_samples.csv`：72 个 data-relative 局部开阔候选；
- `.../phi2_input_relationships.png`：phi2 histogram 与输入关系；
- `.../route_and_medium_diagnosis.png`：路线/松树代理与 Medium weights。

![phi2 输入关系](runtime_artifacts/reward_v2_phi_branch_diagnosis/20260824_d01/phi2_input_relationships.png)

![路线与 Medium 分层](runtime_artifacts/reward_v2_phi_branch_diagnosis/20260824_d01/route_and_medium_diagnosis.png)

## 3. Observation C → phi1 / phi2 完整公式链

### 3.1 Mid360 五帧与 3200 球面 probes

当前 training backend 使用 5 帧 causal truth-pose aligned Mid360 点云。每帧输入先限制为 finite 且 range 在 `(0.20,10.0] m`，五帧对齐到当前 body 后做 `0.05 m` voxel downsample。

球面角划分为：

```text
azimuth  [-180°, 180°), 80 bins
elevation[-90°,  90°], 40 bins
angular resolution = 4.5°
total bins = 80 * 40 = 3200
```

每个 bin 对落入点取最小 range：

```text
nearest_bin[i] = min(||point_body|| in angular bin i)
```

semantic 为：

```text
0 = UNKNOWN
1 = OBSERVED_FREE
2 = KNOWN_OBSTACLE
```

Observation C 只复制原子 v2 的 `lidar_surrogate` 和 `lidar_semantic`，不在 C 层重新计算、膨胀或改变几何。Reward complexity 是 diagnostic/reward context，不进入冻结的五项 policy input。

### 3.2 实际参与 complexity 的几何统计

```text
K = count(semantic[i] == KNOWN_OBSTACLE)
D = K / 3200
N = min(lidar_surrogate[i] for semantic[i] == KNOWN_OBSTACLE)
```

这里：

- `K` 是 occupied angular-bin count，不是障碍物实例数；
- `D` 是球面角 bin fraction，不是 3D volume fraction；
- Reward 没有 obstacle volume、物体数量、inflated clearance 或 raw point count 输入；
- `N` 是已知障碍 probe 的最小距离，不是 EGO inflated voxel clearance。

若 `N=NA,D=0,unknown_count>1600`，才进入 Unknown 特例：`phi2=0.5,phi1=1.25`。本轮 4161 step 全有 numeric N，Unknown count=0；虽然 unknown bins 很多，它们不直接进入 numeric phi。

### 3.3 Normalization、clip 与 phi

设 `clip01(x)=min(1,max(0,x))`：

```text
q_N = clip01((6.0 - N) / (6.0 - 2.5))
    = clip01((6.0 - N) / 3.5)

q_D = clip01((D - 0.040) / (0.080 - 0.040))
    = clip01((D - 0.040) / 0.040)

phi2 = max(q_N, q_D)
phi1 = 1.75 + phi2 * (0.75 - 1.75)
     = 1.75 - phi2
```

数值范围按实现为：

```text
q_N, q_D, phi2 in [0,1]
phi1 in [0.75,1.75] m/s
```

本轮实际只有 `phi2=[0.37431,1]`、`phi1=[0.75,1.37569] m/s`。

### 3.4 Low / Medium / High label

```text
Low    : N >= 6.0 AND D <= 0.040
High   : N <= 2.5 OR  D >= 0.080
Medium : 其余 numeric 状态
Unknown: N=NA,D=0 且 unknown-majority
```

Low/High 使用 normalization 的两个**端点**，不是 continuous weights 的 0.35/0.50/0.65 内部阈值。

### 3.5 Continuous branch weights

```text
smoothstep(z) = clip01(z)^2 * (3 - 2*clip01(z))
```

令 `p=phi2`：

```text
p <= 0.35:
  (w_safe,w_middle,w_dangerous) = (1,0,0)

0.35 < p < 0.50:
  s = smoothstep((p-0.35)/0.15)
  weights = (1-s,s,0)

0.50 <= p < 0.65:
  d = smoothstep((p-0.50)/0.15)
  weights = (0,1-d,d)

p >= 0.65:
  weights = (0,0,1)
```

Stage 1 speed branches为：

```text
safe       = 0.80 * (actual_speed - phi1)
middle     = 0.25 * actual_speed
dangerous  = 1.00 * (phi1 - actual_speed)
r_speed    = w_safe*safe + w_middle*middle + w_dangerous*dangerous
```

因此对固定 phi：

```text
d(r_speed)/d(actual_speed)
  = 0.80*w_safe + 0.25*w_middle - 1.00*w_dangerous

Delta r_speed(1.75-0.30)
  = 1.45 * above_slope
```

该斜率在 `phi2=0.543071...` 变号；并不是等到 label High 才开始偏低速。

## 4. 4161-step 输入与 phi 分布

| 值 | min | p05 | median | p95 | max |
|---|---:|---:|---:|---:|---:|
| `N` nearest m | 1.43656 | 1.78984 | 2.48185 | 4.39915 | 4.68991 |
| `K` occupied bins | 130 | 140 | 167 | 210 | 231 |
| `D=K/3200` | 0.040625 | 0.043750 | 0.052188 | 0.065625 | 0.072188 |
| observed-free bins（派生） | 713 | 723 | 756 | 792 | 849 |
| unknown bins | 2177 | 2238 | 2278 | 2298 | 2304 |
| `q_N` | 0.37431 | 0.45739 | 1.00000 | 1.00000 | 1.00000 |
| `q_D` | 0.01563 | 0.09375 | 0.30469 | 0.64063 | 0.80469 |
| `phi2` | 0.37431 | 0.45739 | 1.00000 | 1.00000 | 1.00000 |
| `phi1` m/s | 0.75000 | 0.75000 | 0.75000 | 1.29261 | 1.37569 |

### 4.1 Normalization 与 saturation 审计

- nearest safe clip：`N>=6` 为 0/4161，所以 `q_N=0` 从未出现；
- nearest dangerous clip：`N<=2.5` 为 2129/4161，所以 `q_N=1` 恰好覆盖全部 High；
- density safe clip：`D<=0.040` 为 0/4161；3200 bins 下阈值是 128 bins，但观测最小为 130；
- density dangerous clip：`D>=0.080` 为 0/4161，`q_D=1` 从未出现；
- `q_N > q_D` 为 4161/4161，最小 margin 仍为 0.10687，因此 max fusion 完全丢弃 density 通道的数值贡献。

两个输入经 normalization 后同为无量纲 `[0,1]`，没有原始米与 fraction 直接相加的量纲错误；但在当前 route 分布上存在**有效通道失衡**：nearest 100% owner、density 0% owner。它是 route/bounds 联合作用的实证结果，不等于已证明 normalization 参数本身普遍错误。

### 4.2 相关性

| 关系 | Pearson | Spearman | 解释 |
|---|---:|---:|---|
| phi2 vs nearest | -0.9726 | -0.9306 | 实际 owner，接近确定性反向映射 |
| phi2 vs density | 0.6969 | 0.8410 | 与路线变密相关，但从未赢得 `max()` |
| phi2 vs actual speed | -0.0505 | -0.1804 | 弱相关，不是高 phi 的主要来源 |
| phi2 vs route progress proxy | 0.8789 | 0.9025 | 随接近 ENTRY 强烈升高 |

## 5. Route 位置与 phi2

现有 Reward JSONL 未保存逐帧 odometry。本报告使用：

```text
每 Episode 按 Observation stamps 对 actual speed 梯形积分
→ 累积距离归一化为 route progress proxy
→ 从记录的 reset start 到 ENTRY 作近直线路径插值，得到 XY proxy
```

它不是实测 position；只用于顺序/空间分段。其合理性由既有 Episode 的最大横向偏差仅约 `0.016–0.042 m` 支持，但报告不把 proxy 写成真实 odometry。

| segment | steps | phi2 p05/median/p95 | N p05/median/p95 m | D p05/median/p95 | label | 低速偏好 |
|---|---:|---:|---:|---:|---|---:|
| start `<0.2` | 846 | 0.4017 / 0.5071 / 0.7223 | 3.4719 / 4.2250 / 4.5941 | 0.04281 / 0.04563 / 0.05000 | 846 Medium | 36.29% |
| middle `0.2–0.8` | 1993 | 0.5850 / 0.9751 / 1.0000 | 2.1936 / 2.5871 / 3.9525 | 0.04344 / 0.05000 / 0.05656 | 1182 Medium / 811 High | 97.09% |
| ENTRY `>=0.8` | 1322 | 1 / 1 / 1 | 1.6670 / 2.1774 / 2.4445 | 0.05625 / 0.06250 / 0.06750 | 4 Medium / 1318 High | 100% |

worksite 同一 Pine_Tree mesh 的 `z=3 m` 半径代理为 0.859 m。五个 reset start 到 ENTRY 的近直线对西北松树的最小表面净空代理仅约 `1.04–1.36 m`；ENTRY 对其西北侧另一棵松树约 1.30 m。该静态代理与 probe N 向 ENTRY 降至 1.44–2.52 m 的趋势一致：当前路线是明确的 clutter corridor，不是开阔直线。

## 6. 真正开阔样本检查

为避免用现有 label 自证，本报告先按数据相对几何定义“局部最开阔”：

```text
N >= runtime p90 = 4.28977 m
AND
D <= runtime p10 = 0.044375
```

共有 72 step，全部集中在 route progress `0–0.246` 的起点附近。结果：

| 指标 | min | median | max |
|---|---:|---:|---:|
| N m | 4.2902 | 4.4114 | 4.6820 |
| D | 0.04281 | 0.04406 | 0.04438 |
| phi2 | 0.3766 | 0.4539 | 0.4885 |
| `Delta r_speed` | +0.3758 | +0.5423 | +1.0938 |

72/72 是 Medium label，但平均 weights 为 safe/middle/dangerous=`0.3422/0.6578/0`，全部明确鼓励较高速度；**这些相对开阔状态没有被 phi 映射到 dangerous side**。

不过它们未来 5 m 的松树表面净空代理只有 `1.09–1.36 m`，0/72 达到 3 m 的诊断性开阔走廊条件。也就是说，它们只是局部 probe 上的“本数据最开阔”，前方仍直接进入松树走廊，不构成完整的 Low/open segment。

### 必答 A

**NO。** 最开阔的可用局部样本 phi2 不高（0.3766–0.4885）、dangerous weight=0、速度偏好为正；当前数据没有“真实开阔却仍被压成高 phi2”的证据。严格的局部+前方开阔样本本身为 0，说明缺的是 route coverage。

同一 `worksite.world` 的 current-generation Environment B 完整任务提供独立对照：6 个 `0.30–1.50 m/s` full-mission run 的 14,784 个 active-valid numeric row 中有 2102（14.22%）Low，N max 达 7.44 m、D min 达 0.0181，Low 都出现在 `NAVIGATING`。该对照使用不同 backend/完整任务路线，未与 4161 主数据混池；它只证明 worksite 和当前 mapping 在其他路线片段能够产生 Low，进一步支持“当前 ENTRY route 覆盖问题”。

## 7. Medium 为何 70% 以上偏低速

Medium 的 phi2 为：min/p05/median/p95/max=`0.3743/0.4289/0.6895/0.9849/0.99994`。它并没有集中在 pure-middle 的 0.5 附近，而是明显靠 dangerous 一侧。

| Medium phi2 | count | mean safe | mean middle | mean dangerous | mean `Delta r_speed` | negative |
|---|---:|---:|---:|---:|---:|---:|
| 0.35–0.40 | 40 | 0.8363 | 0.1637 | 0 | +1.0295 | 0% |
| 0.40–0.45 | 150 | 0.4691 | 0.5309 | 0 | +0.7366 | 0% |
| 0.45–0.50 | 252 | 0.0873 | 0.9127 | 0 | +0.4321 | 0% |
| 0.50–0.55 | 171 | 0 | 0.9079 | 0.0921 | +0.1955 | 9.36% |
| 0.55–0.60 | 194 | 0 | 0.5219 | 0.4781 | -0.5040 | 100% |
| 0.60–0.65 | 116 | 0 | 0.0945 | 0.9055 | -1.2787 | 100% |
| 0.65–1.00 | 1109 | 0 | 0 | 1 | -1.4500 | 100% |

结论：

- smoothstep 本身按源码精确运行，没有数值异常；
- 速度斜率在 phi2≈0.5431 变负，所以 0.50–0.55 末端已有少量负值；
- 1435 个 negative Medium 中，1109（77.28%）已是 `phi2>=0.65` 的 full-dangerous weight，310 位于 0.55–0.65 dangerous-side blend，只有 16 位于 0.50–0.55；
- 因此主要原因是**phi2 实际集中在 Medium label 的 dangerous 侧**，而 `Medium` label 又语义过宽，可包含 100% dangerous weight。label 与 continuous weight 的命名/解释确实不一致，但没有证据表明 open 状态的 numeric mapping 错误。

## 8. 根因分类与是否需要改 phi

最终分类：**A. ROUTE COVERAGE ISSUE**。

不是 PHI MAPPING ISSUE 的依据：

1. 最开阔的可用局部样本 phi2 低于 0.5、dangerous weight=0，全部偏高速；
2. phi2 随 route progress 强增，ENTRY 1322 step 几乎全为 High，符合静态松树走廊；
3. 同一 worksite 的完整任务路线能产生 14.22% Low，说明 mapping 并非普遍压制 Low；
4. 4161 step 中 nearest 100% 拥有 phi2，density 虽相关却不参与 max 结果；低速偏置来自当前路线的 nearest 分布。

是否需要改 phi：**当前 NO**。应记录并修正报告/分析中“Medium等同middle branch”的解释，但本轮不修改公式或阈值。只有下一轮明确开阔 segment 仍出现高 phi2，才有证据进入 normalization range、nearest/density fusion 或 threshold 的候选重标定讨论。

## 9. 下一轮 Reward-only coverage qualification 建议

本轮不执行。建议下一轮仍复用现有 fixed Reward-only framework，不启动 SAC/Replay：

1. 先做静态 world + sensor-only hover 审计，选取一个同时满足较大 probe N、较低 D、前方 5 m 无近障的明确 open segment；不直接改正式 training mission/map；
2. 优先从同一 worksite 完整任务历史中能产生 Low 的 `NAVIGATING` 片段反查具体位置，再形成独立 qualification goal/segment；若位置证据不足，先只增加 odometry 与 forward-trajectory probe 只读记录；
3. qualification 组合为：open segment → transition/Medium segment → 当前 ENTRY clutter segment，显式要求三类 coverage，而不是只按总 Episode success；
4. 每个 step 保留 N、D、K、unknown/free、q_N/q_D owner、phi、weights、真实 odometry、forward 5 m 轨迹邻域 probe 与 fixed action；
5. 预注册 coverage 门，例如 Low/open、phi2 0.35–0.50、0.50–0.65、High 各有足够样本；先验证分布，再决定是否授权 100-Episode SAC Stage 1 qualification；
6. 保持 Reward、phi、EGO、Hector PID、Observation C、random Hover、`v_max` 与 reset 不变。真实 planner/Observation/safety failure 继续保留。

正式 SAC training/evaluation 仍禁止；100-Episode SAC Stage 1 qualification 也不因本次诊断自动放行。
