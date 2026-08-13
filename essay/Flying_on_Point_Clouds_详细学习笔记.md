# Flying on Point Clouds with Reinforcement Learning——详细学习笔记

> 第一事实来源：[`Flying on Point Clouds with Reinforcement Learning.pdf`](<Flying on Point Clouds with Reinforcement Learning.pdf>)。
>
> 复核范围：PDF 全部 8 页，包括正文、Figure 1–6、奖励公式、网络结构、训练实现、仿真统计、实机演示、限制与参考文献语境。本文没有编号 Table，也没有编号 Equation。
>
> 文件确认：8 页，SHA-256 `0953bdddab2df769078eea37840a35cc013363144b9985837c8bf2d075729d3f`。PDF 首页标注 arXiv:2503.00496v1，日期为 2025-03-01。

# 1. 先用一句话说清论文

这篇论文把 Livox Mid-360 的多帧点云压成一个带 unknown 信息的 3200 维“方向—距离”表示，再把它与无人机速度、姿态、高度、目标方向和上一动作一起输入 PPO 策略；策略以 50 Hz 直接输出总推力与三个机体系角速度，由 PX4 执行，系统中没有 EGO-Planner，也没有“先生成轨迹、再跟踪轨迹”的层级。

完整链路是：

```text
Livox Mid-360 点云（10 Hz） + PX4 IMU
               ↓
        Fast-LIO 局部状态估计
               ↓
k 帧点云变换到当前时刻机体系
               ↓
远点过滤 + 0.05 m 均匀降采样
               ↓
3200 个等角方向分区
               ↓
每个方向：最近回波距离，或到 unknown 的距离编码
               ↓
3200 维 point-cloud surrogate
               ↓
MLP encoder + UAV 状态/目标/上一动作融合
               ↓
PPO Actor
               ↓ 50 Hz
[期望总推力 T，期望机体系角速度 ωx,ωy,ωz]
               ↓
PX4 内层飞控
```

# 2. 六个概念必须分开

| 概念 | 在本文中是什么 | 不是什么 |
|---|---|---|
| Perception（感知） | Mid-360 采集几何回波；IMU 与点云经 Fast-LIO 得到局部状态 | 不是策略本身 |
| Observation（观测） | 3200 维点云 surrogate，速度/姿态/高度等自身状态、目标方向和上一动作 | 不是完整真实环境状态，也不是完整原始点云 |
| Policy（策略） | 把 observation 映射为动作分布/动作的神经网络 | 不是 EGO-Planner，也不是点云建图器 |
| Action（动作） | 期望总推力和三个期望机体系角速度，共 4 维 | 不是局部目标、速度上限、轨迹控制点或电机 PWM |
| Trajectory planning（轨迹规划） | 本文方法没有独立轨迹规划层 | 不能把网络隐式产生的闭环运动称作显式规划轨迹 |
| Low-level control（低层控制） | PX4 接收推力/体率指令并执行姿态/电机内环 | RL 没有直接输出单个电机转速 |

## 2.1 它算不算“端到端”？

可以说它是“从紧凑观测到低层控制接口的端到端策略”，因为 observation 经过网络后直接产生推力/体率，中间没有轨迹生成和轨迹跟踪。

但不能说它是“原始点云到电机”的纯端到端系统，因为前面仍有：

- 手工设计的点云 surrogate；
- Fast-LIO 状态估计；
- 历史点云坐标变换；
- PX4 内层飞控。

# 3. 论文究竟解决什么问题

目标是在未知杂乱环境中，仅依赖机载 3D LiDAR、IMU、机载计算和局部状态估计，让四旋翼从一点飞到另一点并避开障碍。（PDF 第 1–3 页）

作者面对的是两个同时存在的问题：

1. 传统规划链路有局部建图、路径/轨迹优化、状态机和轨迹跟踪，多级延迟会在高速时累积。
2. RL 若直接接收几帧原始点云，点数达到约 `10^5` 量级，训练和机载推理都太重；粗暴降采样又可能丢失细杆、细线和窄通道。

## 3.1 为什么选择 3D LiDAR

作者给出的理由是：

- 相比视觉传感器，LiDAR 对周围几何的表达更直接、测距更准确；
- 3D LiDAR 具有高分辨率、长距离和直接三维感知能力，适合发现小障碍；
- 设备正变得更小、更便宜，逐渐适合微型无人机；
- 论文希望在细障碍和窄空间中保留比低分辨率深度图更多的几何细节。（PDF 第 1–2 页）

这不等于论文证明“LiDAR 在所有条件下都优于相机”。作者在结论中明确承认：真实物体反射率会导致 Mid-360 回波缺失，细障碍并非总能避开。（PDF 第 7 页）

## 3.2 为什么不用传统 occupancy map 直接作为 RL 输入

作者提出两类问题：

- 对长量程、高分辨率 LiDAR 做概率占据建图，传统 raycasting 计算代价很高；
- 概率栅格可能因大量穿过同一 cell 的非障碍射线，把含细障碍的 cell 更新成 free，从而丢失细杆；
- 均匀三维栅格维度很高。在论文对照中，0.2 m 分辨率的局部 occupancy map 是 `50×50×15=37,500` 个 cell；3D/2D CNN 的内存开销限制 PPO mini-batch。

需要注意：论文比较的是它自己设置的完整 occupancy-map baseline，不是 Learning Speed 那种“把预规划轨迹画入地图、再沿轨迹抽 2D 切片”的特定表示。因此 Figure 4 不能直接证明 surrogate 一定优于 Learning Speed 的轨迹条件化地图。（PDF 第 5–6 页）

## 3.3 为什么不直接输入完整原始点云

几帧原始点云约为 `10^5` 点量级，而且点云：

- 点数随场景变化，不是固定维度；
- 本质无序；
- 对 RL 从零训练和机载推理都过重。

常见的 uniform/random/farthest-point sampling 能固定点数，却是“任务无关”的采样：为了压到很少的点，可能删掉细杆回波；保留很多点又失去轻量优势。仅凭点坐标还不能直接区分“已经观察但没有障碍”和“传感器从未看见”。（PDF 第 3–4 页）

## 3.4 为什么提出 point-cloud surrogate

surrogate 可译为“代理表示/替代表示”。它不是重建完整地图，而是只保留避障最相关的信息：

- 每个方向最近障碍有多远；
- 这个方向是否很快进入传感器未观察区域；
- 使用固定 3200 维，便于 MLP 和大 mini-batch；
- 近处障碍的方向分辨率较细，细小障碍的一次近距离回波不会像概率 occupancy 更新那样被大量 free 观测冲掉。

代价是空间拓扑和一个方向上第二个、第三个表面都被丢弃；作者还承认远处小障碍可能被表示成更大的障碍。（PDF 第 4 页）

## 3.5 与 EGO-Planner 的根本区别

```text
EGO-Planner：
局部地图 → 搜索/优化 B-spline 或其他轨迹 → 轨迹跟踪器 → 飞控

本文：
点云 surrogate + 状态 + 目标 → RL policy → 推力/体率 → PX4
```

EGO 显式求一条带时间的轨迹，能检查/惩罚碰撞、平滑和动力学项；本文策略不显式产生一条可供检查的未来轨迹，而是每 20 ms 重新闭环决策。论文用大量仿真采样学习这种反应与长期行为，但没有给出传统规划器意义上的确定性安全保证。

# 4. Mid-360 / LiDAR 数据如何进入 RL

这是全文最关键的工程部分，原文集中在 PDF 第 3–5 页。

## 4.1 传感器与频率

- 实机明确使用 **Livox Mid-360**；Figure 2 的 FoV 示意也来自 Mid-360 页面。
- 点云按 **10 Hz** 采样并构造历史表示。
- 策略动作以 **50 Hz** 输出。
- Fast-LIO 状态估计被称为 high-frequency，但论文没有给状态估计的数值频率。

## 4.2 原文可以确认的数据流水线

```text
LiDAR 原始点云
→ 过滤离机器人较远的点（过滤距离阈值：论文未明确说明）
→ 以 0.05 m 分辨率均匀降采样
→ 保存 k 帧、每帧采样频率 10 Hz
→ 用局部状态估计把 k 帧全部变换到当前时刻 t 的 body frame
→ 以 UAV 为球心，把空间按相等角度分成 n 个方向锥体
→ 每个方向只保留一个标量
→ 按固定顺序排列成 n 维向量
→ 输入 MLP encoder
```

### “过滤远点”与“10 m 截断”不要混为一谈

论文说实机预处理先过滤 far-from-robot points，并做 0.05 m 降采样，但没有给前一项的阈值。随后构造 surrogate 时，最近点距离超过 10 m 会截断为 10 m。这是两个不同步骤；不能据此断言前面的过滤半径就是 10 m。

## 4.3 历史帧有多少

方法定义使用 `k` 帧历史点云。实验设置中明确写 `k=5`：

- occupancy 对照实验：`k=5`；
- 三类 benchmark：正文写 “The k is set as 5 in our experiments.”

因此可以确认论文实验使用 5 帧。论文没有单独列出“实机部署 k 是否另设”的参数表，也没有说明 5 帧是包含当前帧还是只含过去 5 帧。按 10 Hz 相邻采样，最早与最新样本时间差若含当前共 5 帧通常约 0.4 s；这是采样关系推导，不是论文明确给出的“历史时窗=0.4 s”，因此不能写成作者参数。

## 4.4 历史点云为何要变换到当前机体系

无人机在移动和转动。若把上一帧点坐标原样叠加，静止树干会因 UAV 位姿变化看起来移动，形成重影。作者使用局部状态估计，把历史帧统一转换到当前时刻 `t` 的 body frame：

```text
历史帧中的点 p_(t-i)
→ 通过历史位姿与当前位姿的相对变换
→ 当前机体系中的点 p_t
```

这样，所有点相对于“现在的无人机朝向和位置”表达。论文没有写出齐次变换矩阵公式、时间同步误差模型或 deskew 细节。

## 4.5 空间如何划分

以 UAV 为球心，在当前 body frame 内把周围球面按相等角度分成 `n` 个锥形方向分区。Figure 2(a) 的红色方锥是分区示例；红点是点云。（PDF 第 3 页）

论文实现：

- `n=3200`；
- 对应角分辨率 `4.5°`；
- 每个分区产生一个数。

### 3200 维怎样得到

论文明确给出“3200 对应 4.5° 角分辨率”，但没有显式写 `80×40` 的索引公式。若按完整球面的方位角 `360°` 与俯仰跨度 `180°` 做等角网格：

```text
360° / 4.5° = 80 个方位角格
180° / 4.5° = 40 个俯仰格
80 × 40 = 3200
```

这是与论文数字一致的直接几何推导。具体边界是否含端点、分区编号顺序以及极区如何处理，论文未明确说明。

Mid-360 的瞬时 FoV 并不覆盖整个球面；未被历史 FoV 覆盖的方向正是通过 unknown 编码处理，而不是删掉这些维度，所以输入长度始终是 3200。

## 4.6 每一维代表什么

设某个角度分区为 `i`。

### 情况 A：分区内至少有一个点

- 取 UAV 到该分区内**最近点**的距离，单位 m；
- 若距离大于 10 m，记为 10。

因此障碍回波的数值范围是 `(0,10]`。一个方向即使有多个表面，也只保留最近表面。

### 情况 B：分区内没有点

作者不直接把它标成“free”。他们利用历史 `k` 帧 LiDAR FoV，计算从 UAV 沿该方向到 unknown 区域的距离 `d_unknown`，其中：

```text
0 < d_unknown < 10
surrogate[i] = 20 - d_unknown
```

所以 unknown 编码落在 `(10,20)`。unknown 越靠近 UAV，`d_unknown` 越小，编码越接近 20；已观察空间延伸得越远，编码越接近 10。

论文用 Figure 2(b) 表示多帧 FoV 的并集和其外部 unknown，但没有给 `d_unknown` 的离散求交算法、FoV 标定误差或数值边界处理。

## 4.7 free、occupied、unknown 到底是否存在

它不是 occupancy map，因此没有给每个 voxel 显式存储三态标签。

- **occupied surface**：由 `(0,10]` 的最近回波距离表示；
- **free space**：从 UAV 到最近回波/unknown 边界之间的空间被距离值隐式表达；
- **unknown**：由 `(10,20)` 的 `20-d_unknown` 显式区分。

所以可以说它“表达了已知障碍、已知自由距离和 unknown”，但不能说每个输入维度就是一个 free/occupied/unknown 分类值。

## 4.8 LiDAR FoV 如何参与 unknown

历史每一帧都带有校准的 LiDAR FoV。变换到当前机体系后，多帧 FoV 共同形成已观察区域；这个区域的外部被视为 unknown。其价值是：

- 点云为空可能是“射线看过且没有近障碍”；
- 也可能是“传感器从未覆盖”；
- 单看点坐标无法区别，FoV 几何可以区别。

## 4.9 是否保留细杆、窄缝

作者的设计目标是尽量保留：

- 4.5° 的细角分区会保留某一方向的最近回波；
- 不像随机抽点那样直接删掉少量细杆点；
- Figure 6(c)(d) 展示了绕过直径 10 mm 线障碍的两个实机案例；
- Figure 6(a)(b) 展示箱体间自由空间通过。

但正确结论必须带限制：

- 这些细线实验是案例展示，不是统计成功率；
- 作者明确说实机不能总是避开细障碍；
- 材料反射率会导致真实 LiDAR 漏检；
- 仿真没有复现 Mid-360 面对墙时出现的大量半径为 0 的 invalid measurements；
- surrogate 相比 occupancy map 有信息损失，远处小障碍可能看起来更大。

# 5. 神经网络结构

## 5.1 总体结构

```text
Livox Mid-360（10 Hz）
        ↓
k 帧点云对齐 + 3200 维 surrogate
        ↓
3 层 MLP encoder
        ↓
128 维 environment hidden state
                         \
当前机体系速度 v ---------\
姿态四元数 q --------------\
估计高度 z ------------------→ 4 层 MLP fusion → projection → 4 维 action
目标在 xy 平面的单位相对方向 g --/
上一期望总推力 T_last -------/
上一期望体率 ω_last --------/
```

Figure 3 的图注列出 `v,q,g,T_last,ω_last`；正文 Observation Space 另外明确列出估计高度 `z`，所以完整阅读时不能漏掉高度。（PDF 第 3–4 页）

## 5.2 Actor 输入

### 外感知 exteroception

- 3200 维 surrogate；
- 先经 MLP encoder 变成 128 维隐藏状态。

### 本体感知 proprioception

- 状态估计器给出的当前**机体中心/ego-centric velocity**；
- IMU 给出的 attitude，Figure 3 用四元数 `q` 表示；
- 估计高度，即世界坐标 z 轴位置；
- 上一次动作：上一期望总推力与三个期望体率。

### 目标信息

- 指向目标的 x-y 平面归一化相对方向；
- 论文没有说把到目标的距离、目标高度差或完整三维目标向量作为输入。

## 5.3 MLP 层

原文写法存在一个需要谨慎复述的地方：

- encoder 被称为“3 layers”，其 hidden-state 输出维度依次为 `128,64,64`；
- 同时又说 encoder 把 3200 维外感知编码为 `128-dimension hidden state`。

这两句对“最终环境特征究竟是 128 还是最后一层 64”在文字上不完全一致。Figure 3 只画模块，不给每层尺寸。应保留这项论文歧义，不能自行改网络。

fusion module：4 层 MLP，隐藏输出维度依次为：

```text
128 → 256 → 256 → 128
```

最后 projection layer 映射为 4 维动作。

## 5.4 是否使用 CNN

- 本文提出的 surrogate policy 使用 **MLP encoder + MLP fusion**，不用 CNN。
- occupancy-map 对照方法使用 3 层 3D CNN 或 2D CNN，并各接 2 层 MLP。

不要把 baseline 的 CNN 写成本文主方法网络。

## 5.5 Critic 输入和结构

论文只明确写：

- 使用 PPO；
- actor 与 critic 的权重不共享。

论文没有逐层给出 critic 网络结构，也没有明确列出 critic 是否接收与 actor 完全相同的 observation、是否有 privileged state。因此这些内容必须标记：**论文未明确说明**。

## 5.6 最终 action

```text
a = [T_hat, ωx_hat, ωy_hat, ωz_hat]
```

- `T_hat`：期望 collective thrust；
- `ω_hat`：期望 body rates；
- 输出频率：50 Hz；
- PX4 内层飞控执行这些命令。

论文没有给推力归一化范围、体率上下限、动作分布参数化和饱和处理。

# 6. 强化学习训练

## 6.1 使用什么算法

使用 **PPO（Proximal Policy Optimization）**。（PDF 第 5 页）

作者没有专门给出“为什么选择 PPO 而不是 SAC”的理论论证。论文只围绕 PPO 的 mini-batch 能力说明：输入更轻后可以使用更大的 batch，而小 batch 可能让 PPO 表现变差。不能把这段改写成作者系统比较 PPO/SAC 后证明 PPO 最优。

## 6.2 Actor 与 Critic 对初学者意味着什么

- Actor：看 observation，产生推力/体率动作。
- Critic：估计当前决策未来能拿多少回报，帮助 PPO 判断这次动作比预期好还是差。
- PPO：收集一批交互数据后更新网络，同时限制新旧策略变化不要过猛。

上述是 PPO 的基础解释；论文没有给 advantage estimator、clip coefficient、value loss 等实现细节。

## 6.3 Observation、Action、Episode

### Observation

3200 维 surrogate + 速度 + 姿态 + 高度 + x-y 目标方向 + 上一动作。

### Action

期望总推力 + 三轴期望体率，4 维，50 Hz。

### Episode termination

论文说 agent **only terminates** 于：

1. z 轴位置超出用户按场景设定的上下范围；
2. UAV 与环境碰撞。

碰撞在训练中通过预构建的 inflated grid map 高效检查；终止后 UAV 重置到 free space。

论文没有明确把“到达目标”列为 termination，也没有给 episode 最大时长。因此到达目标怎样重置、是否继续换目标、超时条件如何定义：**论文未提供**。

## 6.4 Reward 全式

论文给出：

\[
r=r_{forward}+r_{thrust}+r_{smoothness}+r_{max\ speed}
+r_z+r_{ESDF}+r_{collision}+r_{yaw}.
\]

### 1. 前进奖励

\[
r_{forward}=\|p_{goal}-p\|-\|p_{goal}-p_{last}\|.
\]

论文紧接着把 `p_last` 解释为当前时刻位置、`p` 解释为上一时刻位置，所以它等价于“上一步到目标距离减当前到目标距离”：靠近目标为正。符号命名有些反直觉，复现时应以原文定义为准。

### 2. 推力项

\[
r_{thrust}=\|T-g\|.
\]

`T` 为总推力幅值，`g` 为重力加速度幅值。按行为意图它像一个“偏离悬停推力的代价”。但原文总式直接相加且此项为正范数，作者又明确说各 reward component 的权重被省略。因此不能仅凭 PDF 确定它实际乘的是负权重还是公式存在省略。

### 3. 平滑项

\[
r_{smoothness}=\|\omega\|+\|a-a_{last}\|.
\]

- `ω=[ωx,ωy,ωz]`；
- `a=[T_hat,ωx_hat,ωy_hat,ωz_hat]`；
- `a_last` 为上一动作。

它同样是正的 cost-like quantity；有效符号/权重未给。

### 4. 最大速度约束项

\[
r_{max\ speed}=-e^{\max(0,\|v\|-v_{max})}+1.
\]

- 当 `||v||≤v_max` 时为 0；
- 超速后为负，且超得越多惩罚越大；
- `v_max` 是用户给定速度约束，不是策略动作。

### 5. 高度越界项

\[
r_z=\max\{z-z_{max},z_{min}-z,0\}.
\]

它表示超出高度带的程度，也呈正的 cost-like 形式；有效权重未给。

### 6. 离障碍塑形项

\[
r_{ESDF}=\lambda(1-e^{-kd^2}),\qquad \lambda>0,k>0.
\]

`d` 是 UAV 到环境最近点距离。离障碍越远，此项越接近 `λ`；它为稀疏碰撞惩罚提供稠密塑形。虽然名字叫 ESDF，论文的方法输入不是 occupancy/ESDF；这里是训练环境可以查询的最近障碍距离奖励。

### 7. 碰撞惩罚

\[
r_{collision}=-10
\]

只在检测到碰撞时施加。

### 8. 朝向项

\[
r_{yaw}=x_{body}\cdot\frac{v}{\|v\|}.
\]

`x_body` 是机体系 x 轴单位向量；点积是夹角余弦，鼓励机头与运动方向一致。论文没有说明 `||v||=0` 时如何避免除零。

### 奖励可复现性结论

作者明确省略各分量权重。至少 `r_thrust`、`r_smoothness`、`r_z` 按印刷形式更像应最小化的代价，却在总式中相加。因此仅凭 PDF 不能完整复现奖励符号与权重。

## 6.5 一次训练交互示例

```text
1. 仿真器准备一个有障碍的环境和目标
2. 模拟 Mid-360 以 10 Hz 产生点云
3. 取 k 帧、变换到当前机体系、构造 3200 维 surrogate
4. 拼接速度/姿态/高度/目标方向/上一动作
5. Actor 输出推力和体率
6. 带空气阻力与一阶电机延迟的动力学推进 20 ms
7. 计算前进、速度限制、障碍距离、碰撞等 reward
8. 得到下一 observation
9. 若碰撞或高度越界，episode 终止并重置到 free space
10. 1024 个环境并行重复；累计 300 步后进行一次 PPO 更新
```

“随机生成环境”的具体训练分布、每个 episode 的目标采样规则和最大时长，论文未完整说明。

## 6.6 并行、硬件、步数与超参数

| 项目 | 论文明确内容 |
|---|---|
| RL 算法 | PPO |
| 并行环境 | 1024 |
| 两次 policy update 间数据长度 | 每个环境 300 time steps |
| mini-batch | 70,000 |
| Actor/Critic | 权重不共享 |
| 数据采集硬件 | Intel i9-14900K CPU |
| 网络优化硬件 | PDF 印刷为 “NVIDIA RTK 4090 GPU”；很可能是型号拼写问题，但不能擅自改成 RTX |
| 训练步数 | Figure 4 横轴画到 40M steps；正文未明确说最终模型固定训练恰好 40M |
| 训练墙钟时间 | 论文未提供 |
| learning rate | 论文未提供 |
| PPO clip/epoch/GAE/discount/entropy coefficient | 论文未提供 |
| optimizer | 论文未提供 |
| 随机种子 | 论文未提供 |
| reward weights | 论文明确省略 |

## 6.7 Exploration

PPO 策略在训练时通常通过随机动作分布探索，但论文没有给动作分布、初始方差、entropy bonus 或 exploration schedule。作者另外指出大幅空气阻力随机化有助于 agent 探索，但这不等于完整的探索配置。

# 7. 仿真与 sim-to-real

## 7.1 动力学模拟

- 使用参考文献 [42] 开源代码中的 air-drag-augmented quadrotor model；
- 电机延迟用一阶系统模拟。

具体质量、惯量、时间常数、控制上下限：论文未提供。

## 7.2 Dynamics domain randomization

在进入动力学模型前：

- thrust 随机化 `±10%`；
- bodyrates 随机化 `±8%`；
- drag coefficients 随机化 `±30%`。

作者说约 5%–10% 的前两类随机化系数应当也能类似工作，这是经验判断，不是消融表。大幅 drag randomization 被用于避免精确辨识阻力，并借助显式速度估计维持策略表现。（PDF 第 4–5 页）

## 7.3 LiDAR 仿真

作者没有简单地在理想 FoV 内均匀打射线，而是：

1. 从真实特定 LiDAR 在 10 Hz 采样时的极坐标中，拟合其独特时空扫描 pattern；
2. 在仿真中用这些极坐标，直接索引预构建障碍表面点云；
3. 障碍表面点云分辨率为 0.05 m。

这减少计算、保留传感器扫描 pattern，却没有模拟真实 Mid-360 的全部 invalid returns 和材料反射率，成为作者明确指出的 sim-to-real gap。

## 7.4 是否做其他 domain randomization

论文明确写的是推力、体率和空气阻力随机化。点云噪声、回波丢失、反射率、外参、时钟偏差、状态估计漂移、风场随机化等没有给出明确随机化参数。不要自行补充。

# 8. 实机部署

## 8.1 硬件与软件链

| 模块 | 原文确认 |
|---|---|
| LiDAR | Livox Mid-360 |
| 机载计算 | Jetson Orin NX，运行状态估计与 GPU inference |
| 飞控 | PX4 Autopilot |
| 状态估计 | Fast-LIO：融合飞控 IMU 与点云，生成局部高频状态估计 |
| 动作接口 | 期望 collective thrust + desired bodyrates |
| Policy 频率 | 50 Hz |
| 点云采样 | 10 Hz |
| 飞行器型号/尺寸/质量 | 论文未明确说明 |
| PX4 版本、ROS/MAVROS 接口 | 论文未提供 |

飞控经过细致调参，正文报告 bodyrate command delay 小于 20 ms。

## 8.2 Action 如何进入飞控

Figure 3 明确画出 MLP 输出 `T,ω` 回到 PX4 flight controller。论文只说明这些命令由 onboard PX4 执行，没有给消息格式、坐标约定、归一化方式或具体 PX4 offboard API。

## 8.3 仿真模型是否直接部署

论文的表述是：在 simulator 中训练的 policy 可以控制 physical quadrotor，并将此归因于：

- 轻量但贴近真实扫描 pattern 的 LiDAR 仿真；
- Fast-LIO 的准确局部状态；
- dynamics domain randomization；
- 同一类 surrogate 将仿真/真实感知转换成固定表示。

论文没有报告真实数据再训练或实机 fine-tuning。因此准确说法是：**论文报告仿真训练策略直接用于实机演示，未报告实机重新训练**。

## 8.4 实机实验能证明到什么程度

- 室内：箱体、窄空间和直径 10 mm 的线障碍案例；速度约束设为 2.0 m/s。
- 室外：不均匀树林、风扰和超量程 invalid observations；训练时最大速度约束 3.0 m/s；不同试验中飞行距离超过 25 m。

没有给实机试验次数、成功率、碰撞率、置信区间或 baseline 实机对照。因此它证明“存在可工作的部署案例”，不能证明某个真实成功率或全面优于 EGO。

# 9. Figure、Table 与 Equation 逐项解释

## Figure 1：室外树林飞行（PDF 第 1 页）

- 上方左图从起点方向看红色执行轨迹；右图为俯视轨迹；背景彩色点云图由 Fast-LIO 建图模块生成。
- 下方四张照片是沿轨迹不同进度的 UAV 状态。
- 想说明：RL 控制器、机载 LiDAR 和 Fast-LIO 能组成室外闭环。
- 这是一个轨迹案例，不是统计比较。

## Figure 2：surrogate 构造（PDF 第 3 页）

- (a) 球面网格表示等角 partition；红色方锥是一个方向 bin，红点是原始点云，取最近点距离。
- (b) 上图是 Mid-360 FoV 示意；下图把多帧 FoV 合并，灰色区域标为 unknown。
- 想说明：固定维度不仅来自角度划分，unknown 还来自历史 FoV 几何，而不是“没有点就当 free”。

## Figure 3：系统与网络（PDF 第 4 页）

- Mid-360 点云和 PX4 IMU 进入 estimation algorithms；点云与位姿又用于 surrogate。
- MLP encoder 提取环境特征；`v,q,g,T_last,ω_last` 等进入 MLP fusion；输出 `T,ω` 给 PX4。
- Jetson Orin NX 负责估计和推理。
- 图中没有 trajectory planner/tracker，这是区分本文与 EGO 系列的最直接证据。

## Figure 4：surrogate 与 occupancy 训练曲线（PDF 第 5 页）

- 横轴 steps，画到 40M；纵轴 undiscounted/normalized return（图轴标为 Normalized Return，caption 称 undiscounted return）。
- 彩色实线分别比较本文表示与 2D/3D CNN occupancy 输入，以及不同 `N_e`、`N_bs`。
- 同一算力下，本文表示可用较大 mini-batch；最佳本文曲线后期约 60，occupancy 曲线明显较低。数值为读图近似。
- 对照条件：速度约束 3.0 m/s、`k=5`、固定环境分布；occupancy 分辨率 0.2 m。
- 能支持：在作者这套 PPO-from-scratch 设置中，surrogate 更易训练。
- 不能支持：所有 occupancy 表示、所有 RL 算法、或 Learning Speed 的轨迹切片表示都必然更差。

## Figure 5：仿真 benchmark（PDF 第 6 页）

### 三类场景

- Scenario I：`40 m×10 m`，约 100 个障碍，半径 `0.5–0.7 m`；
- Scenario II：半径仍为 `0.5–0.7 m`，约 130 个障碍；
- Scenario III：约 90 个障碍，半径 `0.5–1.2 m`。

每个速度约束、每个场景做 50 trials；对比 Fast-Planner、EGO-Planner、EGO-Planner v2 和本文 RL。各方法使用相同 sensing FoV 与 frequency；policy 未在评价环境训练。

### (a)–(c) 成功率曲线

- 低速 1 m/s 时各方法都接近高成功率；
- 速度约束增大后传统规划方法下降更快；
- Scenario III、4.0 m/s 时，作者正文报告本文约 80%，其他低于 40%。

曲线没有误差条或置信区间，且每点 50 次，不能从图中声称统计显著性。

### (d) 单次成功轨迹

Scenario III、3.0 m/s 的成功示例，图上标注：

- 本文 RL：44.9 m；
- EGO-Planner：49.6 m；
- EGO-Planner v2：48.6 m；
- Fast-Planner：45.1 m。

它说明这个案例中本文路线较直接；只是一例，不能把这四个距离当作平均路径长度。

## Figure 6：室内实机（PDF 第 7 页）

- (a)(b) 两个箱体杂乱场景，箭头表示飞行方向；
- (c)(d) 是两个向上越过细线障碍的高亮案例；
- 线直径 10 mm，速度约束 2.0 m/s。

这张图支持“系统曾成功感知并绕过细线”，不支持“对所有材料、角度和细线都可靠”。作者紧接着在限制中明确反驳了这种过强解读。

## Tables

本文没有编号 Table。训练参数、硬件和场景参数散落在正文与图注中。

## Equations

本文没有编号 Equation。主要数学式只有奖励总式和八个分量，已在第 6.4 节逐项解释。由于省略权重，奖励不能仅凭 PDF 完整复现。

# 10. 实验真正证明了什么

## 有统计支撑

- 在三个参数化随机仿真场景、每点 50 trials 的设置中，本文在较高速度约束下保持更高成功率。
- 在固定训练环境分布中，本文 surrogate 的 PPO 训练回报明显高于作者实现的 occupancy CNN baselines。

## 只有案例支撑

- 室内避让 10 mm 线障碍；
- 室外树林超过 25 m 飞行；
- Figure 5(d) 中更短、更直接的单次轨迹。

## 作者的解释，不是被单变量实验完全证明

- 传统规划在高速下下降来自贪心优化、手工 FSM、在线计算延迟、动力学可行性牺牲和跟踪误差的共同作用；论文没有逐因素隔离。
- 大幅 dynamics randomization 帮助抵抗室外风扰，是作者的合理假设，没有风扰消融表。

# 11. 论文的限制

作者明确写出：

- 实机不能总是避开细障碍；
- 材料反射率导致 LiDAR 射线漏检；
- 当前 LiDAR 仿真没有复现真实 Mid-360 的大量零半径 invalid observations；
- 大障碍场景中策略会贪心地追求速度约束下最快前进，统一参数不能让所有仿真场景都成功；
- 需要更真实又高效的 LiDAR simulator，以及更通用的训练 recipe/policy architecture。

从 PDF 还能看出的限制：

- reward 权重和多个关键 PPO 超参数缺失；
- critic 结构和输入未说明；
- 真实实验没有统计基线；
- 终止条件没有说明到达目标与超时；
- surrogate 丢失同一方向上的多层表面和完整拓扑；
- 依赖 Fast-LIO、准确历史坐标变换和 PX4 调参，不能把全部成功归因于 policy；
- 训练环境与目标分布没有完整披露；
- 论文是 arXiv v1，不能把尚未给出的同行评审结论写进去。

# 12. 给强化学习初学者的一个完整例子

假设无人机正朝树林中的目标飞，前方 6 m 有树干，右上方有一根细枝，左侧是一片当前未被 Mid-360 覆盖的区域。

1. 当前点云和前 4 帧点云合计 5 帧，都变换到当前机体系。
2. 0.05 m 降采样后，树干所在几个角度 bin 的最近距离约 6 m；细枝若有回波，其所在少数 bin 记录更近距离。
3. 左侧没有点，但历史 FoV 显示只观察到 2 m 就进入 unknown，于是该 bin 记录 `20-2=18`，而不是当作 free。
4. 3200 个标量经 MLP 编码，与当前速度、姿态、高度、目标方向和上一动作融合。
5. Actor 可能减小推力、输出向上/侧向体率，开始绕开树干和细枝。
6. PX4 执行体率内环；20 ms 后策略再次决策。
7. 若靠近目标，`r_forward` 增加；远离障碍可提高 `r_ESDF`；碰撞则得到 -10 并终止。
8. 训练时 PPO 用 1024 个环境的批量经验更新；部署时只做前向推理，不在线更新。

这个过程中没有一条显式 B 样条，也没有先算好未来 2–3 秒的完整轨迹。

# 13. 最后一分钟复习

1. 真实传感器是 Livox Mid-360，点云 10 Hz，策略 50 Hz。
2. `k=5` 在实验中明确给出；历史点云统一变换到当前 body frame。
3. 3200 维是 3200 个等角方向各一个距离标量，不是 3200 个点。
4. 障碍回波是 `(0,10]` 最近距离；point-free 方向用 `20-d_unknown` 得到 `(10,20)` unknown 编码。
5. 主网络是 MLP，不是 CNN；occupancy baseline 才使用 CNN。
6. RL 动作是总推力与三轴体率，不是速度上限、局部目标或轨迹。
7. 使用 PPO、1024 并行环境、300 步/更新、mini-batch 70,000。
8. 实机用 Mid-360 + Fast-LIO + Jetson Orin NX + PX4；没有报告实机重训练。
9. 细线和室外飞行是有价值的实机案例，但缺少真实统计，且作者明确承认漏检风险。
10. 对 AstraDroneOpen 来说，最可迁移的首先是点云表示，不是其低层 RL 控制架构。
