# 阶段 4 教程：可控速度、航向与连续轨迹

> 面向已完成阶段 3 的新手，只用于 **PX4 SITL + Gazebo 仿真**。先复习术语，再开始操作。

## 1. 开始前先复习专用术语

| 术语 | 直白解释 | 本阶段中的作用 |
|---|---|---|
| path（路径） | 只说明“从哪里经过”，没有时间信息 | 圆、方形、8 字和椭圆都是路径形状 |
| trajectory（轨迹） | 路径加上“何时到哪里” | 给路径规定 `speed` 后才成为可执行轨迹 |
| setpoint / reference | 控制器发给 PX4 的目标点 / 参考点 | `/mavros/setpoint_position/local` 中的目标位姿 |
| actual pose | 无人机实际到达的位置和朝向 | 用 `/mavros/local_position/odom` 与参考点比较 |
| `Hz` / loop rate | 每秒执行循环的次数 | 默认 `20 Hz`，即约每 `0.05 s` 更新一次 |
| `dt` | 本次与上次更新之间的真实时间差 | 每次推进距离为 `speed × dt`，不再依赖循环次数 |
| arc length（弧长） | 沿曲线走过的实际距离 | 让 `speed` 的单位真正成为 m/s |
| parameterization（参数化） | 用一个参数生成曲线坐标 | 例如圆的 `x=r cosθ, y=r sinθ` |
| cumulative arc-length table | 预先采样曲线并累计每小段长度的表 | 让 8 字、椭圆也能近似匀速 |
| tangent（切线） | 曲线在当前点的前进方向 | `yaw_mode=tangent` 时机头沿切线转动 |
| yaw | 绕竖直轴的机头航向角 | ENU 中 `0°/90°` 分别朝 `+x/+y` |
| yaw unwrap（角度展开） | 把 `179° -> -179°` 解释成继续转 `2°` | 避免在 `±180°` 附近错误反转一整圈 |
| quaternion（四元数） | ROS 表示三维朝向的格式 | 程序把 yaw 转为合法单位四元数再发布 |
| tracking error | 实际位置与当前参考点的三维距离 | 超过门限时暂停参考点，等待无人机追上 |
| corner slowdown | 接近方形拐角时降低参考速度 | 缓解理想直角造成的速度、加速度突变 |
| RMSE | 所有位置误差的均方根，越小越好 | 本阶段低速圆建议先做到 `< 0.30 m` |
| P95 | 95% 的误差都不超过该值 | 比只看平均值更容易发现持续的大误差 |
| position setpoint | 只发送位置和朝向目标 | 本阶段使用它；PX4 仍负责姿态和电机内环 |
| feedforward（前馈） | 额外发送期望速度、加速度帮助跟踪 | 本阶段暂不实现，后续可改用 `PositionTarget` |

记住最关键的一句：**参考点以设定速度运动，无人机实际速度由 PX4 跟踪结果决定，两者不一定相等。**

## 2. 本次任务和已经补好的文件

这次要完成：

```text
等待定位 -> OFFBOARD/解锁 -> 起飞/悬停 -> 到达轨迹起点
-> 按 speed×dt 跟踪轨迹 -> 误差过大暂停 -> 完成指定圈数
-> 返航 -> AUTO.LAND -> PX4 确认落地并自动上锁
```

我已经完成配套实现，你不需要从零抄代码：

- `offboard/src/autoarming_control.cpp`：保留阶段 3 航点模式，新增轨迹入口、真实 `dt`、误差暂停和完整安全状态机；
- `offboard/include/offboard/trajectory_reference.h`：圆、方形、8 字、椭圆及累计弧长表；
- `offboard/launch/stage4_trajectory.launch`：阶段 4 独立启动入口；
- `offboard/scripts/stage4_analyze_bag.py`：从 rosbag 计算速度、RMSE、P95 和 yaw 指标；
- `offboard/test/trajectory_reference_test.cpp`：轨迹长度、8 字等弧长、拐角减速和 yaw 展开测试。

本文中的 `offboard/` 指：

```text
AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/
```

你的任务是读懂关键逻辑，亲自完成下面的仿真和量化验收，而不是只看 Gazebo 中“好像飞了一个圆”。

## 3. 先理解实现原理

### 3.1 为什么旧式“每循环加一点”不是真实速度

若每次循环固定前进 `0.02 m`，`20 Hz` 时速度是 `0.4 m/s`，改成 `30 Hz` 就变成 `0.6 m/s`。正确做法是：

```text
本次推进距离 ds = speed × dt
累计弧长     s  = s + ds
```

无论循环是 `20 Hz` 还是 `30 Hz`，一秒累计距离仍接近 `speed`。程序还把异常大的 `dt` 限制到 `max_track_dt=0.1 s`，避免仿真卡顿后目标突然跳远。

### 3.2 为什么 8 字需要弧长表

等量改变曲线参数，不代表走过等量距离。程序先把一圈采样成 2000 个小线段，记录累计长度；运行时用累计弧长反查坐标。因此圆、8 字和椭圆都能用统一的“走了多少米”驱动。

### 3.3 为什么要先去轨迹起点

圆的第一个参考点通常不在 home。程序先进入 `TRAJECTORY_ENTRY`，以普通位置目标安全到达起点并稳定 1 秒，再进入 `TRACKING`，避免参考点突然跳变。

### 3.4 误差暂停不是任务失败

当上一参考点与实际位置的误差大于 `max_tracking_error` 时：

```text
参考弧长不增加 -> 目标点停住 -> PX4 继续追赶 -> 误差恢复后继续
```

日志出现 `PAUSED` 是保护生效。若频繁暂停，先降低 `speed`，不要先放宽门限掩盖问题。

### 3.5 四种轨迹

| `trajectory_type` | 尺寸参数 | 特点 |
|---|---|---|
| `circle` | `radius` | 最适合第一次验证速度和切线 yaw |
| `square` | `side_length` | 直角处切线天然跳变 90°，程序会减速 |
| `figure8` | `ellipse_a/b` | 用弧长表实现近似匀速，中心处会交叉 |
| `ellipse` | `ellipse_a/b` | 参数角速度恒定不等于线速度恒定，弧长表负责修正 |

`center_x/center_y` 是**相对 home** 的轨迹中心，轨迹高度等于 `takeoff_height`。`clockwise=false/true` 表示逆时针/顺时针。

### 3.6 最短启动命令

`stage4_trajectory.launch` 默认会启动 RViz，并使用 `yaw_mode=tangent` 让机头跟随前进方向。使用默认尺寸、速度、航向和一圈轨迹时，只需要选择轨迹形状：

```bash
# 圆形（circle 本身就是默认值）
roslaunch offboard stage4_trajectory.launch

# 方形
roslaunch offboard stage4_trajectory.launch trajectory_type:=square

# 8 字形
roslaunch offboard stage4_trajectory.launch trajectory_type:=figure8

# 椭圆
roslaunch offboard stage4_trajectory.launch trajectory_type:=ellipse
```

只有需要改变尺寸、速度、航向或圈数时，才在命令末尾追加相应参数。只有无显示器运行或专门测试性能时，才使用 `rviz:=false`。

## 4. 安全规则

1. 只在 SITL/Gazebo 使用；源码会检查 `/use_sim_time=true`。
2. 同时只能有一个 `/mavros/setpoint_position/local` 发布者。
3. 第一次只用空场、`radius=1.0`、`speed=0.25`，确认闭环后再加速。
4. 不要在空中关闭控制节点；等日志出现 `[DONE] ... armed=false` 后再 `Ctrl+C`。
5. 异常时在 QGroundControl 执行 `Land`，不要在空中强制 Disarm。
6. 本阶段只借用已有 FAST-LIO 提供当前机型需要的定位输入，不学习其算法。

## 5. 编译和代码检查

新终端执行：

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make --pkg offboard -j2
catkin_make run_tests_offboard -j2
catkin_test_results build/test_results/offboard
source devel/setup.bash
roslaunch --nodes offboard stage4_trajectory.launch
```

应看到 `autoarming_control` 编译成功、轨迹测试无失败，launch 节点只有：

```text
/map_to_camera_init
/autoarming_control
/rviz_a_loam
```

源码建议按此顺序阅读：

1. `TrajectoryReference::rawPoint()`：四种几何形状；
2. `TrajectoryReference::sample()`：累计弧长反查参考点和切线；
3. `speedScale()`：方形拐角减速；
4. `TRAJECTORY_ENTRY`：先去轨迹起点；
5. `TRACKING`：`dt`、误差暂停、圈数完成和切线 yaw；
6. `RETURN_HOME`、`LANDING`：复习阶段 3 的安全收尾。

## 6. 第一次完整仿真实践

每个命令放在一个独立终端。如果习惯一键启动，可以执行 `./scripts/run_sh/pc_example.sh --base-only`：它会启动 roscore、PX4/Gazebo/MAVROS、FAST-LIO 和 QGroundControl，但不会启动旧的航点控制节点；随后只需另开终端录包并启动阶段 4 控制器。不要在阶段 4 直接使用不带参数的 `pc_example.sh`，因为它还会自动启动 `autoarming_control.launch`。

### 终端 1：启动 PX4、Gazebo 和 MAVROS

```bash
cd ~/AstraDroneOpen
roslaunch simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch \
  world:=/home/yanzu/AstraDroneOpen/simulation/astra_gazebo_worlds/example.world
```

### 终端 2：启动当前机型的定位输入

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch fast_lio mapping_mid360.launch rviz:=false
```

等待 `IMU Initial Done`。这里不需要阅读 FAST-LIO 源码。

### 终端 3：起飞前检查

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/local_position/pose
rostopic info /mavros/setpoint_position/local
```

必须满足：`connected: True`、位姿不是 NaN、setpoint 话题还没有其他控制发布者。

### 终端 4：录制验收数据

```bash
source /opt/ros/noetic/setup.bash
source ~/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash
mkdir -p ~/bag
rosbag record -o "$HOME/bag/stage4" \
  /mavros/state \
  /mavros/extended_state \
  /mavros/local_position/pose \
  /mavros/local_position/odom \
  /mavros/setpoint_position/local \
  /autoarming_control/flight_phase \
  /autoarming_control/tracking_active \
  /autoarming_control/trajectory_progress \
  /rosout
```

### 终端 5：先飞固定 yaw 的低速圆

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rospack find offboard   # 应输出本工作空间中的 offboard 路径
roslaunch offboard stage4_trajectory.launch \
  trajectory_type:=circle radius:=1.0 speed:=0.25 \
  yaw_mode:=fixed fixed_yaw_deg:=0 target_laps:=1
```

如果这是同一个终端，后续重新启动 launch 不需要重复 `source`；如果换了新终端，必须再次执行前三行。`source` 的作用是告诉 ROS 去哪里寻找本工作空间里的 `offboard` 包和 `stage4_trajectory.launch`。

依次确认日志出现：

```text
TAKEOFF -> INITIAL_HOVER -> TRAJECTORY_ENTRY -> TRACKING
-> RETURN_HOME -> AUTO.LAND -> DONE
```

出现 `DONE` 后停止 rosbag。检查发布频率和最终状态：

```bash
rostopic hz /mavros/setpoint_position/local
rostopic echo -n 1 /mavros/state
```

最终必须是 `armed: False`。

## 7. 分步完成阶段 4 实验

每次只改一项，并为每次实验另存 rosbag。

### 7.1 切线 yaw

```bash
roslaunch offboard stage4_trajectory.launch \
  trajectory_type:=circle radius:=1.0 speed:=0.25 \
  yaw_mode:=tangent max_tracking_error:=0.30
```

观察机头沿圆的切线转动。跨过 `±180°` 时应连续转动，不能反向绕一整圈。

### 7.2 速度阶梯

依次测试 `0.5 -> 1.0 -> 1.5 m/s`，例如：

```bash
roslaunch offboard stage4_trajectory.launch speed:=0.5
```

先保持半径、yaw 和误差门限不变。速度升高后，实际速度可能落后、RMSE 变大、`PAUSED` 增多，这是实验结论，不是把门限调大就算成功。

### 7.3 验证与循环频率无关

用相同 `radius=1.0 speed=0.25` 分别运行：

```bash
roslaunch offboard stage4_trajectory.launch loop_rate:=20.0
roslaunch offboard stage4_trajectory.launch loop_rate:=30.0
```

上面两行不是同时输入：先运行 `20.0`，等待 `DONE` 和 `armed: False`；保存这一轮 bag 后，再重新录包并运行 `30.0`。这两个命令只启动阶段 4 控制器，前提是基础仿真已经通过手动终端或 `pc_example.sh --base-only` 启动。

圆的理想一圈时间为：

```text
T = 2πr / speed = 2π×1/0.25 ≈ 25.13 s
```

实际 `TRACKING` 还包含暂停和末端稳定时间，所以会稍长；两种频率的结果应接近，而不能相差 50%。

### 7.4 方形、8 字和椭圆

下面假设所有终端都已经关闭，并且要从头依次完成方形、8 字和椭圆实验。三个实验共用一次基础仿真；不要同时运行两个轨迹控制器。

#### 第一步：终端 1 一键启动基础环境

```bash
cd ~/AstraDroneOpen
./scripts/run_sh/pc_example.sh --base-only
```

这个终端会进入 tmux，并启动 roscore、PX4/Gazebo/MAVROS、FAST-LIO 和 QGroundControl，但不会启动旧控制器。保持它运行，等待 Gazebo 中无人机出现、MAVROS 建立连接，并看到 FAST-LIO 的 `IMU Initial Done`。

#### 第二步：终端 2 检查起飞条件

另开一个普通终端：

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/local_position/pose
rostopic info /mavros/setpoint_position/local
```

必须确认 `connected: True`、位置不是 `NaN`，并且 setpoint 话题没有其他控制程序作为 Publisher。

#### 第三步：终端 3 录制当前形状

先录方形，终端保持在 `Recording...` 状态：

```bash
source /opt/ros/noetic/setup.bash
source ~/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash
mkdir -p "$HOME/bag"
rosbag record -o "$HOME/bag/stage4_square" \
  /mavros/state \
  /mavros/extended_state \
  /mavros/local_position/pose \
  /mavros/local_position/odom \
  /mavros/setpoint_position/local \
  /autoarming_control/flight_phase \
  /autoarming_control/tracking_active \
  /autoarming_control/trajectory_progress \
  /rosout
```

#### 第四步：终端 4 运行方形

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch offboard stage4_trajectory.launch trajectory_type:=square side_length:=2.0 speed:=0.25 yaw_mode:=tangent
```

方形拐角附近会自动把速度降低到 `speed×corner_speed_ratio`。等待日志出现 `DONE`，并确认 `armed: False`；然后到终端 3 按 `Ctrl+C`，正常完成方形 bag。

#### 第五步：用相同终端依次运行 8 字和椭圆

基础仿真不需要重启。每次都先在终端 3 重新执行录制命令，只把文件名前缀分别改为：

```text
$HOME/bag/stage4_figure8
$HOME/bag/stage4_ellipse
```

录制开始后，在已经加载过工作空间的终端 4 一次运行一个命令。

8 字：

```bash
roslaunch offboard stage4_trajectory.launch trajectory_type:=figure8 ellipse_a:=1.5 ellipse_b:=0.8 speed:=0.25
```

椭圆：

```bash
roslaunch offboard stage4_trajectory.launch trajectory_type:=ellipse ellipse_a:=1.5 ellipse_b:=0.8 speed:=0.25
```

每一轮都严格按照这个顺序：开始录制 → 启动轨迹 → 等待 `DONE` → 确认 `armed: False` → 在录制终端按 `Ctrl+C`。如果没有正常降落或仍为 `armed: True`，不要启动下一个形状。

#### 第六步：确认三个 bag 已保存

```bash
ls -lh "$HOME/bag"/stage4_square*.bag
ls -lh "$HOME/bag"/stage4_figure8*.bag
ls -lh "$HOME/bag"/stage4_ellipse*.bag
```

理想方形拐角的切线会跳变 `90°`，这是路径本身不光滑造成的；需要检查的是没有 `±360°` 的错误翻转，且拐角确实减速。

## 8. 用脚本量化结果

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun offboard stage4_analyze_bag.py ~/bag/stage4_circle.bag
```

重点看：

- 参考点移动速度中位数是否接近设置的 `speed`；
- 位置 RMSE 是否 `< 0.30 m`；
- 最大相邻参考 yaw 步长是否合理；
- `20 Hz` 与 `30 Hz` 的 TRACKING 时间是否接近。

本次在当前机器上实际验证的 `30 Hz`、`radius=1.0`、`speed=0.25`、切线 yaw、`max_tracking_error=0.30` 结果为：

| 指标 | 实测值 |
|---|---:|
| 参考点速度中位数 | `0.250 m/s` |
| 实际速度中位数 | `0.252 m/s` |
| 位置 RMSE | `0.272 m` |
| P95 / 最大位置误差 | `0.306 / 0.308 m` |
| yaw RMSE | `5.61°` |
| 相邻参考 yaw 最大步长 | `0.54°` |
| TRACKING 时间 | `27.63 s` |

这说明 `speed×dt`、误差暂停、切线 yaw、返航和降落闭环均已在实际仿真中运行通过。不同电脑的实时因子不同，你仍要保存自己的结果。

## 9. 常用参数速查

| 参数 | 默认值 | 何时修改 |
|---|---:|---|
| `trajectory_type` | `circle` | 选择四种轨迹 |
| `speed` | `0.25` | 参考点速度，先慢后快 |
| `yaw_mode` | `tangent` | 默认让机头跟随前进方向；需要固定机头时改为 `fixed` |
| `radius` | `1.0` | 圆半径 |
| `side_length` | `3.0` | 方形边长 |
| `ellipse_a/b` | `2.0/1.0` | 8 字、椭圆尺寸 |
| `center_x/y` | `0/0` | 相对 home 移动轨迹中心 |
| `max_tracking_error` | `0.30` | 超过即暂停参考点 |
| `target_laps` | `1` | 轨迹圈数 |
| `clockwise` | `false` | 顺/逆时针 |
| `loop_rate` | `20` | 验证时间驱动时改为 30 |
| `corner_speed_ratio` | `0.35` | 方形拐角最低速度比例 |

## 10. 常见问题

### 无法解锁或一直等位姿

确认 FAST-LIO 已出现 `IMU Initial Done`，再检查 `/mavros/estimator_status`。不要靠反复手动解锁绕过定位问题。

### 进度长期显示 PAUSED

飞机跟不上参考点。依次检查定位是否稳定、是否有多个发布者，然后降低 `speed`；不要先增大 `max_tracking_error`。

### 改参数后没有变化

重新编译并在当前终端执行 `source devel/setup.bash`，再用：

```bash
rosparam get /autoarming_control/speed
rosparam get /autoarming_control/trajectory_type
```

### 轨迹飞向奇怪位置

`center_x/y` 是相对 home 的米制 ENU 偏移，不是经纬度。并检查：

```bash
rostopic info /mavros/setpoint_position/local
```

只允许 `/autoarming_control` 一个发布者。

## 11. 阶段 4 验收清单

- [ ] 我能解释路径、轨迹、参考速度、实际速度和跟踪误差的区别。
- [ ] 我能解释为什么必须用 `speed×dt`，并能计算圆的理想完成时间。
- [ ] 固定 yaw 圆和切线 yaw 圆都完整完成并自动上锁。
- [ ] 我验证了 yaw 穿过 `±180°` 时没有整圈反转。
- [ ] 我看到并解释过误差暂停，知道应先降速而不是放宽门限。
- [ ] 我分别运行了方形、8 字和椭圆，并理解方形拐角为何减速。
- [ ] `20 Hz` 与 `30 Hz` 下相同速度的完成时间基本一致。
- [ ] rosbag 分析显示参考点速度接近设定值，低速圆 RMSE 先达到约 `0.30 m` 内。
- [ ] 最终状态为 `armed: False`，且保存了 launch 参数、rosbag 和分析输出。

全部完成后再进入阶段 5；阶段 5 会系统学习实验设计、参数调试和更完整的误差分析。
