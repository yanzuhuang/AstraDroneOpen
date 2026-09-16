# 三机 YOLO 部署、使用与交接说明

## 1. 本次交付结论

本次工作已将 PPE YOLO 推理节点以独立、只读的 ROS1 感知模块接入三架 Gazebo/PX4 SITL 无人机。三架无人机分别订阅自己的 D435 彩色图像，并分别发布结构化检测结果。

已经跑通的最小链路为：

```text
/uav1/d435/color/image_raw -> /uav1/ppe_yolo -> /uav1/yolo/detections
/uav2/d435/color/image_raw -> /uav2/ppe_yolo -> /uav2/yolo/detections
/uav3/d435/color/image_raw -> /uav3/ppe_yolo -> /uav3/yolo/detections
```

三路节点已分别在 CPU 和 NVIDIA GPU（`cuda:0`）模式下成功加载模型并持续发布 `astra_custom_msgs/AstraDetection2DArray` 消息。GPU 实测环境为 RTX 4060 Laptop、PyTorch 2.4.1 + CUDA 12.1。

本次没有完成“三机稳定绕塔并得到非空违规行为检测”的最终飞行验收。原因是现有三机入口路径与编队安全逻辑发生冲突，而不是 YOLO 节点安装或推理失败。该问题见第 8 节，交由三机路径规划负责人继续处理。

## 2. 实现边界

YOLO 节点只负责：

- 订阅一条 `sensor_msgs/Image` 相机话题；
- 加载指定 Ultralytics YOLO 权重；
- 按置信度、IoU、类别白名单进行推理和过滤；
- 发布结构化检测框；
- 可选发布带标注图像。

YOLO 节点不会：

- 发布 MAVROS、PX4、EGO-Planner 或任务控制指令；
- 修改无人机航迹、速度、高度或安全许可；
- 解锁、起飞、降落或切换飞行模式；
- 依赖三机任务状态机才能启动。

因此关闭 YOLO 节点不会影响队友原有三机绕塔程序；三机任务失败时，YOLO 节点也仍可独立接收相机图像并发布检测消息。

## 3. 完成的代码和资源修改

### 3.1 YOLO ROS 包

修改或新增：

```text
AstraDrone_ros1_ws/src/Detection/yolo_detect/
├── CMakeLists.txt
├── package.xml
├── config/ppe_three_uav.yaml
├── launch/ppe_yolo_uav.launch
├── launch/ppe_yolo_three_uav.launch
└── script/yolo_detect.py
```

主要内容：

- 移除 `CATKIN_IGNORE`，使 `yolo_detect` 进入 catkin 构建；
- 增加 `rospy`、`sensor_msgs`、`cv_bridge` 和 `astra_custom_msgs` 依赖；
- 将原先写死相机话题和模型路径的脚本改为 ROS 参数化节点；
- 支持 `cpu`、`cuda:0` 等 Ultralytics 设备参数；
- 增加模型文件存在性和预期类别检查；
- 为每架无人机配置独立命名空间、相机名称和输出话题；
- 默认关闭标注图像发布，减少三机运行时的 CPU、显存和 ROS 带宽占用。

### 3.2 自定义检测消息

新增：

```text
AstraDrone_ros1_ws/src/Utils/astra_custom_msgs/msg/AstraDetection2D.msg
AstraDrone_ros1_ws/src/Utils/astra_custom_msgs/msg/AstraDetection2DArray.msg
```

并修改：

```text
AstraDrone_ros1_ws/src/Utils/astra_custom_msgs/CMakeLists.txt
```

单个检测框包含类别编号、类别名称、置信度、中心坐标、宽度和高度。数组消息还包含原始图像时间戳、相机名称、图像尺寸和单帧推理耗时。

### 3.3 三机 Gazebo PPE 测试场景

修改：

```text
simulation/astra_gazebo_worlds/worksite.world
```

新增模型：

```text
simulation/astra_gazebo_models/helmet_sampling_bridge/
simulation/astra_gazebo_models/helmet_worker_platform/
simulation/astra_gazebo_models/helmet_worker_standing/
simulation/astra_gazebo_models/worker_no_helmet/
```

在目标铁塔周围加入：

- 6 个戴安全帽人物；
- 6 个未戴安全帽人物；
- 8 块支撑平台；
- 4 段连接支撑桥。

人物、平台和连接板参考 `002_core_forest_helmet_sampling.world` 的相对布局，整体适配到三机任务使用的铁塔中心。未改变三机出生点、任务高度、绕塔半径、EGO 参数或安全距离。

### 3.4 构建依赖与本机 PX4 路径

为解决完整工作空间中消息生成顺序问题，修改：

```text
AstraDrone_ros1_ws/src/SLAM/FAST_LIO/CMakeLists.txt
simulation/sim_workspace/src/sensors/Mid360_simulation_plugin/livox_laser_simulation/CMakeLists.txt
```

修改以下脚本，使不同开发机可以通过环境变量指定 PX4，同时保留队友原默认路径：

```text
scripts/run_sh/three_uav/three_uav_inspection.sh
```

使用方式：

```bash
export ASTRA_PX4_ROOT=/实际路径/PX4-Autopilot
```

如果没有设置该变量，脚本仍使用原默认值 `/home/yanzu/PX4-Autopilot`。

## 4. 环境与模型要求

基础环境：

- Ubuntu 20.04；
- ROS1 Noetic；
- Gazebo Classic；
- 已构建的仿真和主 ROS 工作空间；
- Ultralytics、PyTorch、OpenCV Python 和 ROS `cv_bridge` 可用；
- YOLO 权重类别至少包含配置文件中的 `expected_class_names`。

当前配置预期类别为：

```yaml
[person, helmet, no_helmet, safety_vest, no_safety_vest]
```

默认实际发布的类别白名单为：

```yaml
[person, no_helmet, no_safety_vest]
```

可在 `config/ppe_three_uav.yaml` 中调整阈值、推理尺寸、最大推理频率和类别白名单。

训练权重通常不应直接提交到 Git。建议使用项目制品存储、Git LFS 或发布附件分发，并在启动时通过 `model_path` 指定本机路径。

## 5. 构建方法

先构建下层仿真工作空间，再构建主 ROS 工作空间：

```bash
cd /path/to/AstraDroneOpen/simulation/sim_workspace
source /opt/ros/noetic/setup.bash
catkin_make -j2

cd /path/to/AstraDroneOpen/AstraDrone_ros1_ws
source /opt/ros/noetic/setup.bash
source ../simulation/sim_workspace/devel/setup.bash
catkin_make -j2
```

本次已在当前开发机完成两个工作空间的完整构建，主工作空间共 30 个包构建通过。

## 6. 启动和调用 YOLO

### 6.1 先启动三机仿真

只预览、不飞行：

```bash
cd /path/to/AstraDroneOpen
export ASTRA_PX4_ROOT=/path/to/PX4-Autopilot
./scripts/run_sh/three_uav/three_uav_inspection.sh --gui --record none
```

启用三机自动控制：

```bash
cd /path/to/AstraDroneOpen
export ASTRA_PX4_ROOT=/path/to/PX4-Autopilot
./scripts/run_sh/three_uav/three_uav_inspection.sh --control --gui --record none
```

注意：`--control` 会在 Gazebo/PX4 SITL 中解锁并控制无人机。仅查看场景或验证 YOLO ROS 链路时不要添加该参数。

### 6.2 启动三路 GPU YOLO

另开终端：

```bash
cd /path/to/AstraDroneOpen/AstraDrone_ros1_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch yolo_detect ppe_yolo_three_uav.launch \
  model_path:=/path/to/best.pt \
  python_exec:=/path/to/yolo-venv/bin/python3 \
  device:=cuda:0
```

如果电脑没有兼容 CUDA 的 NVIDIA GPU，使用：

```bash
roslaunch yolo_detect ppe_yolo_three_uav.launch \
  model_path:=/path/to/best.pt \
  python_exec:=/path/to/yolo-venv/bin/python3 \
  device:=cpu
```

CPU 三路同时推理速度较慢。本次 640 输入实测单帧约 1.3–2.7 秒；GPU 模式已确认三个进程均成功使用 `cuda:0`。

### 6.3 单独启动某一架无人机的 YOLO

```bash
roslaunch yolo_detect ppe_yolo_uav.launch \
  vehicle_ns:=uav1 \
  camera_name:=uav1_d435 \
  model_path:=/path/to/best.pt \
  python_exec:=/path/to/yolo-venv/bin/python3 \
  device:=cuda:0
```

将 `uav1` 替换为 `uav2` 或 `uav3` 即可。

## 7. 验证方法与已取得证据

确认节点：

```bash
rosnode list | grep ppe_yolo
```

预期：

```text
/uav1/ppe_yolo
/uav2/ppe_yolo
/uav3/ppe_yolo
```

确认相机连接：

```bash
rostopic info /uav1/d435/color/image_raw
rostopic info /uav2/d435/color/image_raw
rostopic info /uav3/d435/color/image_raw
```

确认检测输出：

```bash
rostopic echo -n 1 /uav1/yolo/detections
rostopic echo -n 1 /uav2/yolo/detections
rostopic echo -n 1 /uav3/yolo/detections
```

确认频率：

```bash
rostopic hz --wall-time /uav1/yolo/detections
rostopic hz --wall-time /uav2/yolo/detections
rostopic hz --wall-time /uav3/yolo/detections
```

本次已确认：

- Gazebo 是三路 D435 图像的发布者；
- 三个 YOLO 节点分别订阅对应无人机图像；
- 三个检测话题分别由对应 YOLO 节点发布；
- 消息包含正确的 `camera_name`、光学坐标系、`640x480` 尺寸和推理耗时；
- 三路 CPU 和 GPU 模型加载均成功；
- 无人机停在起点、人物不在画面时仍正常发布空数组 `detections: []`。

空数组表示该帧没有通过阈值和白名单的目标，不表示节点没有运行。

## 8. 当前遗留问题：三机未形成绕塔轨迹

本次控制仿真没有形成可用于人物识别的绕塔轨迹，现象为：

- UAV1 最终进入 `ERROR` 和 `AUTO.LAND`，落回地面；
- UAV2、UAV3 在约 3.2 m 高度进入 `HOLDING/OFFBOARD`；
- 编队协调器进入 `SAFETY_INHIBIT`；
- `formation_orbit_active=false`，三机均未得到绕塔释放许可。

入口联合选择本身成功，选择结果为 `OK_TIER_1`。随后 UAV1 的 EGO 规划出现 `NO_FEASIBLE_TRAJECTORY`，安全模块预测 UAV1 与 UAV2 的最小距离约为 2.79–2.98 m，低于项目配置的 3.0 m 最小三维安全距离，触发：

```text
PREDICTED_SEPARATION_CONFLICT
```

系统按 fail-closed 策略处理：UAV1 返航/降落，UAV2 和 UAV3 悬停，所有新入口和绕塔许可被抑制。因此 D435 相机没有获得稳定、近距离的塔上人物视角，当前只能确认三路 YOLO 推理链路，不能确认实际飞行中的非空违规行为识别结果。

后续路径规划负责人需要重点检查：

1. 三机起飞点和入口路径初段的空间间距；
2. UAV1 入口目标与 UAV2 预测轨迹的冲突；
3. EGO `NO_FEASIBLE_TRAJECTORY` 的地图净空和目标可达性；
4. 入口走廊释放时序及三机是否应错峰进入；
5. 在不降低既有安全边界的前提下，使三机稳定到达绕塔半径并进入 `formation_orbit_active=true`。

不要通过直接关闭安全模块或盲目降低 3.0 m 安全距离绕过该问题。

## 9. 建议上传清单

建议提交本次功能相关文件：

```text
AstraDrone_ros1_ws/src/Detection/yolo_detect/
AstraDrone_ros1_ws/src/Utils/astra_custom_msgs/CMakeLists.txt
AstraDrone_ros1_ws/src/Utils/astra_custom_msgs/msg/AstraDetection2D.msg
AstraDrone_ros1_ws/src/Utils/astra_custom_msgs/msg/AstraDetection2DArray.msg
simulation/astra_gazebo_worlds/worksite.world
simulation/astra_gazebo_models/helmet_sampling_bridge/
simulation/astra_gazebo_models/helmet_worker_platform/
simulation/astra_gazebo_models/helmet_worker_standing/
simulation/astra_gazebo_models/worker_no_helmet/
simulation/sim_workspace/src/sensors/Mid360_simulation_plugin/livox_laser_simulation/CMakeLists.txt
AstraDrone_ros1_ws/src/SLAM/FAST_LIO/CMakeLists.txt
scripts/run_sh/three_uav/three_uav_inspection.sh
docs/05-三机YOLO部署与交接说明.md
```

提交前应单独检查以下运行生成文件，不要把它混入本次功能提交：

```text
AstraDrone_ros1_ws/src/SLAM/FAST_LIO/Log/mat_pre.txt
```

本次工作没有修改外部 PX4 源码，没有升级 ROS、Gazebo、MAVROS、EGO-Planner、FAST-LIO、Ultralytics 或 PyTorch。
