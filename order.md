# AstraDrone 仿真启动命令速查

这个文件只记录常用启动、编译和重启命令，方便开发时快速查看。

## 1. 完整重启仿真

如果上一次仿真状态不干净，先清场：

```bash
tmux kill-server
```

进入 ROS 工作空间：

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
```

编译代码：

```bash
catkin_make
```

回到项目根目录并启动默认仿真：

```bash
cd ~/AstraDroneOpen
./scripts/run_sh/pc_example.sh
```

这个脚本会依次启动：

- `roscore`
- PX4 / Gazebo / MAVROS
- FAST-LIO
- Offboard 控制节点
- QGroundControl

## 2. 日常改代码后的快速迭代

适合只修改 `autoarming_control.cpp` 之类的控制代码，不想每次完全重启仿真。

### 第一步：停止当前控制节点

在运行 `roslaunch` 的 VSCode 终端里按：

```text
Ctrl + C
```

无人机会因为收不到控制指令而触发保护，通常会在 Gazebo 中缓慢降落。

### 第二步：等待无人机锁定

在 Gazebo 或 QGroundControl 里确认：

- 无人机已经落地
- 螺旋桨完全停转
- 飞控进入 `Disarmed` 状态

不要刚按完 `Ctrl + C` 就立刻重新启动。

### 第三步：修改并编译

保存代码后，在 ROS 工作空间里编译：

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make
```

### 第四步：重新启动控制节点

```bash
source devel/setup.bash
roslaunch offboard autoarming_control.launch
```

无人机会在当前位置重新解锁、起飞，然后飞向控制程序设定的目标。

## 3. 只把飞机位置重置回原点

如果 Gazebo、PX4、MAVROS 都还正常，只是不想让飞机从当前位置飞回原点，可以只重置模型位置，不需要重新运行 `./scripts/run_sh/pc_example.sh`。

1. 先等无人机落地并停桨。
2. 在 Gazebo 菜单中点击：

```text
Edit -> Reset Model Poses
```

也可以使用快捷键：

```text
Ctrl + Shift + R
```

如果代码没有改，只需要重新启动控制节点：

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
source devel/setup.bash
roslaunch offboard autoarming_control.launch
```

如果刚改过代码，再先编译：

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make
source devel/setup.bash
roslaunch offboard autoarming_control.launch
```

## 4. 重要避坑

Gazebo 里只用：

```text
Reset Model Poses
```

不要点：

```text
Reset World
```

`Reset World` 会把仿真时间也重置，容易导致 ROS、PX4 和 MAVROS 的时间戳错乱。出现这种情况通常只能重新清场：

```bash
tmux kill-server
```

然后再完整重启仿真：

```bash
cd ~/AstraDroneOpen
./scripts/run_sh/pc_example.sh
```

注意：上面这两行是误点 `Reset World` 后才需要的完整重启命令，正常使用 `Reset Model Poses` 时不需要运行。

### 4.1 阶段 4 轨迹控制启动

阶段 4 的 launch 文件在 `AstraDrone_ros1_ws` 工作空间中。**每个新开的终端都要先加载 ROS 工作空间**；`source` 只对当前终端生效：

如果希望像平常一样一键启动基础仿真，使用：

```bash
cd ~/AstraDroneOpen
./scripts/run_sh/pc_example.sh --base-only
```

`--base-only` 会启动 roscore、PX4/Gazebo/MAVROS、FAST-LIO 和 QGroundControl，但不会自动启动旧的 `autoarming_control.launch`。不带参数的 `pc_example.sh` 仍保留原来的默认演示行为，因此不适合直接用于阶段 4 对比实验。

基础环境启动后，另开一个终端加载工作空间：

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rospack find offboard
```

最后一条命令应输出类似：

```text
/home/yanzu/AstraDroneOpen/AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard
```

确认能找到包后，再启动阶段 4：

```bash
roslaunch offboard continuous_trajectory.launch \
  trajectory_type:=circle radius:=1.0 speed:=0.25 \
  yaw_mode:=tangent target_laps:=1
```

阶段 4 默认启动 RViz，并默认使用 `yaw_mode:=tangent` 让机头跟随前进方向；不需要额外填写 `rviz:=true` 或 `yaw_mode:=tangent`，也不要在常规飞行命令中填写 `rviz:=false`。使用 launch 中的默认参数时，四种轨迹可以简写为：

```bash
roslaunch offboard continuous_trajectory.launch
roslaunch offboard continuous_trajectory.launch trajectory_type:=square
roslaunch offboard continuous_trajectory.launch trajectory_type:=figure8
roslaunch offboard continuous_trajectory.launch trajectory_type:=ellipse
```

如果 `rospack find offboard` 仍然提示找不到包，先编译一次：

```bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make --pkg offboard -j2
source devel/setup.bash
```

然后重新执行上面的 `roslaunch`。报错“不是 launch 文件”通常不是文件内容错误，而是忘记 `source devel/setup.bash`，或在错误的 ROS 工作空间中启动。

## 5. rosbag 飞行数据记录与分析

`rosbag` 相当于 ROS 的“数据黑匣子”：它把指定 topic 上的消息按照时间顺序保存到 `.bag` 文件。它不会自动录制 Gazebo 画面，只保存选择的 ROS 数据。

### 5.1 先固定保存目录

本项目的学习记录统一保存到 `~/bag`，以后不再混用 `stage2_records` 等目录：

```bash
mkdir -p "$HOME/bag"
ls -lh "$HOME/bag"
```

不要输入：

```bash
cd ~/AstraDroneOpen/stage2 records
```

这条命令既不是当前保存目录，`stage2 records` 中的空格还会被 Bash 分成两个参数。使用绝对输出路径后，不需要先 `cd` 到保存目录，也不会再猜文件放在哪里。

你当前已经录好的阶段 4 文件是：

```text
/home/yanzu/bag/stage4_circle.bag
```

因此应使用 `stage4_circle.bag` 查看，不能改写成并不存在的 `run01_hover10.bag`。

### 5.2 开始录制

当前 PX4/Gazebo Offboard 主线（阶段 0～5）统一使用下面这一条命令，不再按阶段区分。请在启动 `autoarming_control` 之前，新开终端执行：

```bash
mkdir -p "$HOME/bag"
rosbag record -o "$HOME/bag/astra_flight" \
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

命令含义：

```text
rosbag record                 开始录制
-o                           指定文件名前缀，并自动追加日期和时间
$HOME/bag/astra_flight       所有记录统一保存到 ~/bag
后面的 topic                 需要保存的数据
```

生成的文件名类似：

```text
astra_flight_2026-07-17-15-30-21.bag
```

时间戳由 rosbag 自动生成，因此连续录制不会覆盖上一轮。录制过程中会暂时出现 `.bag.active`；正常停止后才会完成索引并变成 `.bag`。

这组公共话题同时覆盖：连接和模式、解锁状态、实际位姿、实际速度、位置目标、任务阶段、轨迹进度和程序日志。某项实验没有使用轨迹功能时，对应轨迹话题没有有效数据也不影响其他内容。

先开始录包，再启动控制节点，才能保存 OFFBOARD、解锁、起飞、悬停/航点/轨迹、降落和最终上锁的完整过程。

### 5.3 录制不能暂停，只能正确停止

ROS1 的 `rosbag record` 没有像播放器那样的空格暂停功能。飞行结束并确认 `/mavros/state` 中 `armed: false` 后，在**录制终端**按：

```text
Ctrl + C    停止录制并完成 bag 索引
```

等待终端返回命令提示符后，再查看文件：

```bash
ls -lh "$HOME/bag"
```

不要使用 `Ctrl+Z` 冒充暂停，也不要直接关闭终端；这会挂起或杀死录制进程，可能留下未完成的 `.bag.active` 文件。

如果确实想把一次实验分成两段，应先用 `Ctrl+C` 正常结束第一段，再重新执行统一录制命令。`-o` 会生成新的时间戳文件名。

rosbag 不会把第二次录制追加到第一份文件中。

### 5.4 查看 bag 文件和基本信息

先按修改时间列出真实文件名：

```bash
ls -lht "$HOME/bag"
```

复制需要查看的完整文件名，再执行：

```bash
rosbag info "$HOME/bag/astra_flight_日期-时间.bag"
```

你截图中的报错是因为 `~/bag` 中实际存在 `stage4_circle.bag`，但执行的是：

```bash
rosbag info run01_hover10.bag
```

当前目录里没有这个名字，自然会报告 `No such file or directory`。如果忘记保存位置，可以查找：

```bash
find "$HOME" -maxdepth 3 -type f -name '*.bag' -print
```

`rosbag info` 重点查看：

```text
duration    录制持续时间
start/end   开始和结束时间
size        文件大小
topics      保存了哪些话题
messages    每个话题保存了多少条消息
```

#### 怎样读懂 `rosbag info` 输出

你当前的 `stage4_circle.bag` 可以这样理解：

| 输出项 | 你的结果 | 新手应该怎样理解 |
|---|---:|---|
| `path` | `/home/yanzu/bag/stage4_circle.bag` | bag 的真实保存位置 |
| `version` | `2.0` | rosbag 文件格式版本，不是 ROS 版本 |
| `duration` | `2:50s (170s)` | 从第一条到最后一条消息共 170 秒 |
| `start/end` | `Jan 01 1970 ...` | 当前使用 Gazebo 仿真时间，不是真实日期，出现 1970 是正常的 |
| `size` | `4.2 MB` | 文件实际占用空间，当前大小正常 |
| `messages` | `10823` | 所有话题合计保存了 10823 条消息 |
| `compression` | `none [6/6 chunks]` | 未压缩；数据分成 6 个内部数据块，不是 6 个 bag 文件 |
| `types` | 多种消息类型 | 说明 bag 使用了哪些 ROS 消息格式；方括号中的长字符串是消息定义校验值，新手暂时不用分析 |
| `topics` | 7 个话题 | 每一行显示“话题名、消息数量、消息类型” |

`topics` 部分最重要。你这份文件的每一行含义如下：

| 话题 | 你的消息数 | 保存的内容 |
|---|---:|---|
| `/autoarming_control/flight_phase` | `1119` | TAKEOFF、TRACKING、RETURN_HOME 等任务阶段 |
| `/autoarming_control/tracking_active` | `1119` | 是否正处于轨迹跟踪阶段，`true/false` |
| `/autoarming_control/trajectory_progress` | `1119` | 轨迹完成比例，通常从 `0.0` 增加到 `1.0` |
| `/mavros/extended_state` | `852` | PX4 的在地面、起飞、空中、降落等扩展状态 |
| `/mavros/local_position/odom` | `5111` | 无人机实际位置、姿态和实际速度 |
| `/mavros/setpoint_position/local` | `1332` | 控制程序持续发送给 PX4 的目标位置和目标 yaw |
| `/mavros/state` | `171` | FCU 连接、飞行模式和 `armed` 解锁状态 |

消息数不需要彼此相等。不同话题发布频率不同，例如 odom 通常比 state 快很多，所以 `5111` 条 odom、`171` 条 state 是正常的。

#### 用三步判断 bag 是否有效

1. `size` 不能是 `0 B`，`messages` 必须大于 0；
2. 需要的话题必须出现在 `topics` 列表中，而且各自消息数大于 0；
3. `duration` 应覆盖起飞到最终上锁，不能只录到任务的一小段。

你的文件大小为 `4.2 MB`、包含 `10823` 条消息，阶段 4 所需的 7 个话题都有数据，因此这份 bag **录制有效**。`duration=170s` 比实际轨迹时间长，表示你提前开始录制并在任务结束一段时间后才停止；这不会破坏数据，只会包含一些任务前后的等待数据。

`rosbag info` 只能说明“录到了什么”，不能直接告诉你轨迹误差是否合格。阶段 4 的速度、位置误差和 yaw 指标要继续使用下面的分析脚本。

连续轨迹实验还可以把真实文件名交给阶段 4 量化脚本：

```bash
astra
rosrun offboard analyze_trajectory_bag.py "$HOME/bag/stage4_circle.bag"
```

### 5.5 查看 bag 中的具体数值

先在当前终端设置一次文件变量，后面的命令就不用反复输入长路径：

```bash
BAG="$HOME/bag/stage4_circle.bag"
```

这个变量只在当前终端有效。关闭终端后，重新执行一次即可。

#### 查看飞控是否连接、解锁以及处于什么模式

```bash
rostopic echo -b "$BAG" -n 5 /mavros/state
```

重点看：

```text
connected: True    MAVROS 已连接 PX4
armed: True/False  无人机已解锁/已上锁
mode: "OFFBOARD"  外部控制节点正在控制
```

`-n 5` 表示只显示前 5 条，避免 171 条消息一次全部刷满终端。

#### 查看无人机实际位置

```bash
rostopic echo -b "$BAG" -n 5 \
  /mavros/local_position/odom/pose/pose/position
```

输出中的 `x/y/z` 单位是米，表示 ENU 局部坐标中的实际位置。`z` 增大表示无人机上升。

#### 查看无人机实际速度

```bash
rostopic echo -b "$BAG" -n 5 \
  /mavros/local_position/odom/twist/twist/linear
```

`x/y/z` 单位是 m/s。这是飞机真正飞出来的速度，不是 launch 中设置的参考速度。

#### 查看控制程序发送的目标位置

```bash
rostopic echo -b "$BAG" -n 5 \
  /mavros/setpoint_position/local/pose/position
```

把这里的目标 `x/y/z` 与 odom 的实际 `x/y/z` 比较，就能理解“控制器想去哪里”和“飞机实际在哪里”。两者的三维距离就是位置跟踪误差。

#### 查看任务按什么顺序执行

下面的命令会去掉连续重复状态，只打印阶段切换顺序：

```bash
rostopic echo -b "$BAG" -p /autoarming_control/flight_phase | \
  awk -F, 'NR > 1 && $2 != previous {print $2; previous=$2}'
```

你当前这份 bag 的结果是：

```text
TAKEOFF
INITIAL_HOVER
TRAJECTORY_ENTRY
TRACKING
RETURN_HOME
LANDING
```

这证明任务确实依次完成了起飞、悬停、进入轨迹、跟踪、返航和降落。

#### 查看轨迹进度原始数据

```bash
rostopic echo -b "$BAG" -n 10 \
  /autoarming_control/trajectory_progress
```

`data: 0.0` 表示 0%，`data: 0.5` 表示 50%，`data: 1.0` 表示 100%。前几条通常为 `0.0`，因为任务还处于起飞阶段；要查看完整变化，应导出 CSV 或使用下一节的分析脚本。

#### 这些数据最终有什么用

| 想回答的问题 | 应查看的数据 |
|---|---|
| 是否成功连接、进入 OFFBOARD、解锁和上锁？ | `/mavros/state` |
| 是否真正落地？ | `/mavros/extended_state` |
| 飞机实际飞到哪里、速度多快？ | `/mavros/local_position/odom` |
| 控制程序要求飞机去哪里？ | `/mavros/setpoint_position/local` |
| 任务有没有按正确顺序执行？ | `/autoarming_control/flight_phase` |
| 轨迹是否从 0% 执行到 100%？ | `/autoarming_control/trajectory_progress` |
| 目标与实际相差多大、飞行是否合格？ | 配套分析脚本计算 RMSE、P95、速度和 yaw 误差 |

所以录 bag 不是为了把一万多条数字逐条读完，而是保存一次完整实验，让你以后能针对问题查看相应话题，并用脚本得到可比较的指标。

### 5.6 导出 CSV 表格

导出实际位姿和位置目标：

```bash
rostopic echo -b "$HOME/bag/stage4_circle.bag" -p \
  /mavros/local_position/odom > "$HOME/bag/stage4_actual_odom.csv"

rostopic echo -b "$HOME/bag/stage4_circle.bag" -p \
  /mavros/setpoint_position/local > "$HOME/bag/stage4_setpoint.csv"
```

CSV 可以使用表格软件打开。阶段 2 主要比较悬停时的目标/实际 `x/y/z`；阶段 4 使用配套分析脚本计算轨迹速度、RMSE、P95 和 yaw 误差。

### 5.7 离线回放、暂停和继续

这里的“暂停”只适用于 `rosbag play` 回放。回放会重新发布历史话题，因此必须先停止 PX4、MAVROS、Gazebo 和所有控制节点，只保留新的 `roscore`。

终端 1：

```bash
roscore
```

终端 2：

```bash
rosparam set /use_sim_time true
rosbag play --clock --pause "$HOME/bag/stage4_circle.bag"
```

`--pause` 表示启动后先停在第一帧。确保鼠标焦点位于播放终端，再使用：

```text
空格键    暂停或继续回放
s         暂停时单步前进一条消息
Ctrl + C  结束回放
```

回放时可以使用 RViz、`rostopic echo` 或 `rqt_plot` 观察历史数据。例如：

```bash
rqt_plot \
  /mavros/local_position/odom/pose/pose/position/z \
  /mavros/setpoint_position/local/pose/position/z
```

### 5.8 重要安全提醒

不要在正在运行的 PX4/MAVROS 仿真或真机环境中执行 `rosbag play`。当前 bag 包含：

```text
/mavros/setpoint_position/local
```

回放会重新发布历史控制目标，可能与当前控制节点争夺控制权。分析包含 setpoint 的 bag 时，只进行离线回放。

也不建议新手使用 `rosbag record -a`。它会录制所有话题，激光雷达、相机和点云可能迅速生成非常大的文件。

### 5.9 bag 没有正常关闭时修复索引

先确认没有 rosbag 进程仍在写入：

```bash
pgrep -af "rosbag record"
```

若没有输出，但目录中留下 `.bag.active`，可以尝试：

```bash
rosbag reindex "$HOME/bag/文件名.bag.active"
mv "$HOME/bag/文件名.bag.active" "$HOME/bag/文件名.bag"
rosbag info "$HOME/bag/文件名.bag"
```

异常恢复不能保证找回最后一段数据，因此正常实验仍应使用 `Ctrl+C` 停止录制。
