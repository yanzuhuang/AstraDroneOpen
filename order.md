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
