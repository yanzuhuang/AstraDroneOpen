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

## 5. rosbag 飞行数据记录与分析

`rosbag` 相当于 ROS 的“数据黑匣子”：它把指定 topic 上的消息按照时间顺序保存到 `.bag` 文件。它不会自动录制 Gazebo 画面，只保存选择的 ROS 数据。

阶段 2 建议记录：

- `/mavros/state`：FCU 连接、飞行模式和解锁状态；
- `/mavros/local_position/pose`：无人机实际位置和姿态；
- `/mavros/local_position/odom`：实际位置、姿态和速度；
- `/mavros/setpoint_position/local`：控制节点发布的目标位置；
- `/rosout`：控制节点的阶段切换和错误日志。

### 5.1 开始录制

新开一个终端，先创建记录目录：

```bash
mkdir -p ~/AstraDroneOpen/stage2_records
cd ~/AstraDroneOpen/stage2_records
```

在启动 `autoarming_control` 之前执行：

```bash
rosbag record -O run01_hover10.bag \
  /mavros/state \
  /mavros/local_position/pose \
  /mavros/local_position/odom \
  /mavros/setpoint_position/local \
  /rosout
```

命令含义：

```text
rosbag record       开始录制
-O                  指定输出文件名
run01_hover10.bag   本轮生成的 bag 文件
后面的 topic        本轮需要保存的数据
```

先开始录包，再启动控制节点，这样才能保存 OFFBOARD、解锁、起飞、悬停、降落和最终上锁的完整过程。

### 5.2 正确停止录制

飞行结束并确认 `/mavros/state` 中 `armed: false` 后，在录包终端按：

```text
Ctrl + C
```

等待终端正常返回命令提示符。不要直接关闭录包终端，否则 bag 可能没有正确写完索引。

推荐按实验内容命名：

```text
run01_hover10.bag
run02_hover30.bag
run03_nonzero_home.bag
run04_hover60.bag
```

不要重复使用同一个文件名覆盖上一轮证据。

### 5.3 查看 bag 基本信息

```bash
cd ~/AstraDroneOpen/stage2_records
rosbag info run01_hover10.bag
```

重点查看：

```text
duration    录制持续时间
start/end   开始和结束时间
size        文件大小
topics      保存了哪些话题
messages    每个话题保存了多少条消息
```

五个目标 topic 都有消息，并且 duration 覆盖整个飞行过程，才算有效记录。

### 5.4 导出 CSV 表格

导出实际位姿：

```bash
rostopic echo -b run01_hover10.bag -p \
  /mavros/local_position/pose > run01_actual_pose.csv
```

导出位置目标：

```bash
rostopic echo -b run01_hover10.bag -p \
  /mavros/setpoint_position/local > run01_setpoint.csv
```

CSV 可以使用表格软件打开。重点比较：

```text
目标 x 与实际 x
目标 y 与实际 y
目标 z 与实际 z
```

阶段 2 悬停误差可按下面计算：

```text
e_x = |x_actual - x_target|
e_y = |y_actual - y_target|
e_z = |z_actual - z_target|
```

只统计稳定进入 HOVER 后的数据，不要把起飞和降落过程混进悬停误差。

### 5.5 离线回放

回放会重新发布 bag 中的话题。为了防止历史 setpoint 再次控制无人机，必须先停止 PX4、MAVROS、Gazebo 和所有控制节点，只保留一个新的 `roscore`。

终端 1：

```bash
roscore
```

终端 2：

```bash
rosparam set /use_sim_time true
cd ~/AstraDroneOpen/stage2_records
rosbag play --clock run01_hover10.bag
```

可以暂停和继续回放：

```text
空格键    暂停或继续
Ctrl + C  停止回放
```

回放时可以使用 RViz、`rostopic echo` 或 `rqt_plot` 观察历史数据。例如：

```bash
rqt_plot \
  /mavros/local_position/pose/pose/position/z \
  /mavros/setpoint_position/local/pose/position/z
```

### 5.6 重要安全提醒

不要在正在运行的 PX4/MAVROS 仿真或真机环境中执行：

```bash
rosbag play run01_hover10.bag
```

因为 bag 中包含：

```text
/mavros/setpoint_position/local
```

回放会重新发布历史控制目标，可能与当前控制节点争夺控制权。分析包含 setpoint 的 bag 时，只进行离线回放。

也不建议新手直接使用：

```bash
rosbag record -a
```

`-a` 会录制所有 topic，激光雷达、相机和点云可能迅速生成非常大的文件。当前阶段只录制前面列出的五个必要 topic。

### 5.7 bag 没有正常关闭时修复索引

如果录制终端异常退出，可能留下 `.bag.active` 文件。确认没有 rosbag 进程仍在写入后，可以尝试：

```bash
rosbag reindex 文件名.bag.active
mv 文件名.bag.active 文件名.bag
```

`reindex` 会在旁边保留原始备份；将修复后的 `.bag.active` 改回 `.bag` 后检查：

```bash
rosbag info 文件名.bag
```

异常恢复不能保证找回最后一段数据，因此正常实验仍应使用 `Ctrl+C` 停止录制。
