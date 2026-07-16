# 阶段 2 教程：实现原地起飞、定点悬停与安全降落

> 适用对象：已经完成 `stage0.md` 和 `stage1.md`，第一次亲手修改 ROS C++ 飞行状态机的学习者。
>
> 对应路线：`studymap.md` 的“阶段 2：实现基础起飞、悬停和安全降落”。
>
> 本文只用于 **PX4 SITL + Gazebo 仿真**。当前降落末端使用 `21196` 强制上锁，并由 `/use_sim_time=true` 硬性保护；严禁直接用于真机。

## 1. 这次任务到底要完成什么

阶段 1 中的默认程序会起飞后立刻飞圆形或方形，而且起飞、降落目标固定在世界原点。阶段 2 要把它改造成一条更小、更可靠、更容易解释的任务链：

```text
等待 FCU 和有效位姿
  -> 记录本次起飞点 home
  -> 连续预发送 setpoint
  -> 请求 OFFBOARD
  -> 请求解锁
  -> 在 home 正上方起飞
  -> 定点悬停指定时间
  -> 回到 home 高度
  -> 稳定接地后请求上锁
  -> 从 /mavros/state 确认 armed=false
```

完成后，你应该能独立做到：

1. 解释“悬停”为什么仍要持续发布同一个 setpoint。
2. 等到第一帧有效位姿后才记录 `home_x/home_y/home_z`。
3. 使用相对起飞高度，而不是把目标 z 写成固定世界坐标。
4. 为每个 `PoseStamped` 设置时间戳、坐标系和合法四元数。
5. 写出不使用阻塞式 `sleep()` 的 `TAKEOFF -> HOVER -> LANDING -> COMPLETED` 状态机。
6. 起飞或降落超时时进入明确的安全行为，不把超时误判成成功。
7. 降落后同时检查上锁服务结果和 `/mavros/state.armed`。
8. 完成 10 秒、30 秒、60 秒悬停测试，以及一次非零出生点测试。
9. 用 rosbag、话题频率和位置误差证明任务真的完成。

本阶段应保存四类成果：

- 修改后的 `autoarming_control.cpp` 和 `autoarming_control.launch`；
- 四份有效 rosbag，覆盖 10 秒、30 秒、60 秒悬停和非零出生点测试；
- 一张包含 home、目标、最大误差、setpoint 频率和最终 armed 状态的实验表；
- 一段你自己写的总结，解释这次改造解决了阶段 1 的哪些问题。

## 2. 先记住六条安全规则

### 2.1 只在仿真中练习

本文默认操作 Gazebo 中的虚拟 `iris_mid360`。不要连接真机飞控，不要在有真实电机、螺旋桨的环境中运行本节点。

### 2.2 同一时刻只能有一个 setpoint 发布者

本阶段唯一允许持续发布 `/mavros/setpoint_position/local` 的节点是：

```text
/autoarming_control
```

每轮起飞前执行：

```bash
rostopic info /mavros/setpoint_position/local
```

启动控制器前必须看到 `Publishers: None`；运行时只能看到 `/autoarming_control`。不要同时运行 `position_control`、阶段 1 的另一份控制节点或手写 setpoint 发布器。

### 2.3 不要在空中停止控制节点

OFFBOARD 依赖连续 setpoint。正常实验要让状态机自行降落，不要在空中按 `Ctrl+C`，也不要在空中强制上锁。

若仿真中飞机明显失控：

1. 优先在 QGroundControl 中执行 `Land`；
2. 等模型接地后再上锁；
3. 保存 bag 和终端报错；
4. 停止整套仿真并重新建立基线。

`Disarm` 不是空中急停按钮。本文的强制上锁只为解决当前仿真兼容问题；源码在 `/use_sim_time` 不为 `true` 时会拒绝启动。

### 2.4 先用空场和保守参数

第一次实践使用：

```text
simulation/astra_gazebo_worlds/example.world
```

建议基线参数：

| 参数 | 基线值 | 含义 |
|---|---:|---|
| `takeoff_height` | `2.0 m` | 相对 home 的起飞高度 |
| `hover_duration` | `10 s` | 首轮悬停时间 |
| `takeoff_tolerance` | `0.15 m` | 允许的高度误差 |
| `horizontal_tolerance` | `0.20 m` | 允许的水平误差 |
| `force_disarm_height` | `0.10 m` | 仅仿真：低于此相对高度时强制上锁 |
| `phase_timeout` | `60 s` | 起飞和降落阶段的超时阈值 |
| `pose_timeout` | `1.0 s` | 多久收不到新位姿算反馈失效 |

这些是学习用推荐值，不是仓库作者承诺的性能指标。

### 2.5 修改源文件，不修改生成目录

只修改：

```text
AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/
```

不要编辑 `AstraDrone_ros1_ws/build/` 或 `AstraDrone_ros1_ws/devel/` 中的文件；它们会被重新编译覆盖。

### 2.6 保留阶段 1 的证据

开始前先看改动：

```bash
cd "$HOME/AstraDroneOpen"
git status --short
git diff -- \
  AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src/autoarming_control.cpp \
  AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/launch/autoarming_control.launch
```

如果这些文件已有你尚未保存的阶段 1 修改，先用 Git 提交，或在编辑器中另存副本。不要用 `git reset --hard` 清理，因为它会丢弃未提交内容。

## 3. 先理解原理，再写代码

### 3.1 定点悬停不等于“不发送命令”

位置控制中的悬停是：每个控制周期都发送同一个目标。

```text
目标位置：home_x, home_y, home_z + takeoff_height
                    │
                    ▼
             PX4 位置控制器
                    │ 比较目标与实际位置
                    ▼
          姿态、推力和电机控制
                    │
                    ▼
               Gazebo 无人机
                    │ 实际位置反馈
                    └──────────────> 下一周期继续比较
```

如果达到高度后停止发布，PX4 会失去连续的外部目标流，可能退出 OFFBOARD。正确做法是目标不变、发布不停。

### 3.2 `connected=true` 不等于已经收到位姿

`/mavros/state.connected` 只说明 MAVROS 已与 FCU 通信，不能证明 `/mavros/local_position/pose` 已经回调过。

阶段 1 的代码在 FCU 刚连接后立刻读取 `current_pose`。全局变量默认可能全为 0，于是程序可能错误地把 `(0,0,0)` 当作起飞点。阶段 2 必须增加 `have_pose`：

```text
connected == true  AND  have_pose == true
                 │
                 ▼
       才允许记录 home 和预发送目标
```

收到消息还不够，x、y、z 必须是有限数，不能是 `NaN` 或 `Inf`。

### 3.3 home 是本次起飞点，不是世界原点

假设无人机出生在：

```text
home = (3.0, -2.0, 0.06)
takeoff_height = 2.0
```

正确起飞目标是：

```text
(3.0, -2.0, 2.06)
```

不是 `(0,0,2.0)`，也不是 `(3,-2,2.0)`。公式必须是：

```cpp
target_z = home_z + takeoff_height;
```

整个任务中 `home_x/home_y/home_z` 只记录一次，不能每个循环都用当前位置覆盖，否则“家”会跟着飞机移动。

### 3.4 一个合法的位置目标应包含什么

本阶段发布 `geometry_msgs/PoseStamped`，至少要设置：

```cpp
setpoint.header.stamp = ros::Time::now();
setpoint.header.frame_id = "map";
setpoint.pose.position.x = ...;
setpoint.pose.position.y = ...;
setpoint.pose.position.z = ...;
setpoint.pose.orientation.x = 0.0;
setpoint.pose.orientation.y = 0.0;
setpoint.pose.orientation.z = 0.0;
setpoint.pose.orientation.w = 1.0;
```

四元数 `(0,0,0,0)` 不表示“没有旋转”，它是非法零四元数。无旋转应使用单位四元数 `(0,0,0,1)`。

### 3.5 为什么不能用阻塞式 `sleep(30)` 悬停

下面是错误写法：

```cpp
local_pos_pub.publish(hover_pose);
sleep(30);  // 错误：30 秒内不处理回调，也不继续发 setpoint
```

阻塞期间会同时停止：

- setpoint 发布；
- `/mavros/state` 更新；
- 实际位姿更新；
- 超时和异常判断。

正确写法是每个 20 Hz 周期都发布，只用时间差决定何时切换状态：

```cpp
if (ros::Time::now() - phase_start >= ros::Duration(hover_duration)) {
    flight_phase = FlightPhase::LANDING;
}
```

### 3.6 到达目标不能只看一眼 Gazebo

本阶段使用两个误差：

```text
高度误差 = |current_z - target_z|
水平误差 = sqrt((current_x-home_x)^2 + (current_y-home_y)^2)
```

只有高度误差和水平误差同时小于门限，才从 `TAKEOFF` 进入 `HOVER`。这样可避免飞机恰好经过目标高度、但仍离 home 很远时误判。

### 3.7 调用上锁服务不等于已经上锁

完整证据链是：

```text
到达 `force_disarm_height` 范围
  -> 调用 /mavros/cmd/command，发送 command=400、param2=21196
  -> 检查 service response.success
  -> 继续读取 /mavros/state
  -> 只有 armed=false 才进入 COMPLETED
```

服务调用失败时不能打印“任务完成”，也不能退出节点；应继续发布安全的降落目标并间隔重试。

## 4. 先设计状态机

### 4.1 正常状态转换

```text
等待连接与位姿
       │
       ▼
记录 home + PRESTREAM
       │
       ▼
请求 OFFBOARD 和解锁
       │ mode=OFFBOARD 且 armed=true
       ▼
    TAKEOFF
       │ 高度误差、水平误差都达标
       ▼
     HOVER
       │ 悬停时间到
       ▼
    LANDING
       │ 近地且稳定 -> 请求上锁
       │ /mavros/state.armed=false
       ▼
   COMPLETED
```

### 4.2 每个状态必须回答五个问题

| 状态 | 持续发布的目标 | 完成条件 | 超时行为 | 禁止行为 |
|---|---|---|---|---|
| `TAKEOFF` | `(home_x, home_y, home_z + H)` | z 和 xy 误差达标 | 转入 `LANDING` | 无限等待、停止发布 |
| `HOVER` | 与起飞目标完全相同 | `hover_duration` 到时 | 本状态用正常计时结束 | 阻塞式 sleep |
| `LANDING` | `(home_x, home_y, home_z)` | 进入仿真强制上锁高度且最终 `armed=false` | 保持降落目标并报警 | 在真机使用 21196 |
| `COMPLETED` | 不再飞行 | 已确认上锁 | 无 | `armed=true` 时退出 |

当前仓库的 `AUTO.LAND` 与仿真高度反馈组合可能触地反弹，普通上锁又可能因 land detector 未确认接地而被拒绝，因此这里沿用原项目的 `21196` 强制上锁方式。它只适用于仿真；真机必须使用正常 land detector/AUTO.LAND。

## 5. 修改控制代码

### 5.1 打开正确文件

修改：

```text
AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/
src/autoarming_control.cpp
```

本阶段不使用圆形、方形轨迹，因此可以把阶段 1 的轨迹函数和 `TRACKING` 分支暂时移除。下面给出完整参考实现。建议先按小节理解，再亲手输入或替换；至少要能解释每个状态的目标和转换条件。

### 5.2 阶段 2 参考实现

```cpp
/**
 * @file autoarming_control.cpp
 * @brief PX4 Offboard 仿真：原地起飞、定点悬停、原地降落并确认上锁
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include <string>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
bool have_pose = false;
ros::Time last_pose_time;

enum class FlightPhase {
    TAKEOFF,
    HOVER,
    LANDING,
    COMPLETED
};

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    const double x = msg->pose.position.x;
    const double y = msg->pose.position.y;
    const double z = msg->pose.position.z;

    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
        current_pose = *msg;
        have_pose = true;
        last_pose_time = ros::Time::now();
    } else {
        ROS_WARN_THROTTLE(1.0, "[POSE] Rejected NaN/Inf local pose");
    }
}

geometry_msgs::PoseStamped make_setpoint(double x, double y, double z) {
    geometry_msgs::PoseStamped setpoint;
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "map";
    setpoint.pose.position.x = x;
    setpoint.pose.position.y = y;
    setpoint.pose.position.z = z;

    // 单位四元数：本阶段保持 yaw=0。
    setpoint.pose.orientation.x = 0.0;
    setpoint.pose.orientation.y = 0.0;
    setpoint.pose.orientation.z = 0.0;
    setpoint.pose.orientation.w = 1.0;
    return setpoint;
}

double horizontal_error(double x, double y, double home_x, double home_y) {
    const double dx = x - home_x;
    const double dy = y - home_y;
    return std::sqrt(dx * dx + dy * dy);
}

bool force_disarm(ros::ServiceClient& command_client) {
    mavros_msgs::CommandLong command;
    command.request.broadcast = false;
    command.request.command = 400;
    command.request.confirmation = 0;
    command.request.param1 = 0.0;
    command.request.param2 = 21196.0;  // 仅限仿真
    return command_client.call(command) && command.response.success;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    bool use_sim_time = false;
    nh.param("/use_sim_time", use_sim_time, false);
    if (!use_sim_time) {
        ROS_FATAL("[SAFETY] Force-disarm landing is simulation-only");
        return 1;
    }

    double takeoff_height;
    double hover_duration;
    double takeoff_tolerance;
    double horizontal_tolerance;
    double force_disarm_height;
    double phase_timeout;
    double pose_timeout;

    pnh.param("takeoff_height", takeoff_height, 2.0);
    pnh.param("hover_duration", hover_duration, 10.0);
    pnh.param("takeoff_tolerance", takeoff_tolerance, 0.15);
    pnh.param("horizontal_tolerance", horizontal_tolerance, 0.20);
    pnh.param("force_disarm_height", force_disarm_height, 0.10);
    pnh.param("phase_timeout", phase_timeout, 60.0);
    pnh.param("pose_timeout", pose_timeout, 1.0);

    if (takeoff_height <= 0.0 || hover_duration < 0.0 ||
        takeoff_tolerance <= 0.0 || horizontal_tolerance <= 0.0 ||
        force_disarm_height <= 0.0 ||
        force_disarm_height >= takeoff_height ||
        phase_timeout <= 0.0 || pose_timeout <= 0.0) {
        ROS_FATAL("[PARAM] Invalid parameter: check height, duration, tolerance and timeout");
        return 1;
    }

    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub =
        nh.subscribe<geometry_msgs::PoseStamped>(
            "mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub =
        nh.advertise<geometry_msgs::PoseStamped>(
            "mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client =
        nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client =
        nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient command_client =
        nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

    ros::Rate rate(20.0);

    // FCU 已连接和收到有效 pose 两个条件必须同时成立。
    while (ros::ok() && (!current_state.connected || !have_pose)) {
        ROS_INFO_THROTTLE(
            2.0, "[WAIT] connected=%s, have_pose=%s",
            current_state.connected ? "true" : "false",
            have_pose ? "true" : "false");
        ros::spinOnce();
        rate.sleep();
    }
    if (!ros::ok()) {
        return 0;
    }

    // home 只在这里记录一次。
    const double home_x = current_pose.pose.position.x;
    const double home_y = current_pose.pose.position.y;
    const double home_z = current_pose.pose.position.z;
    const double target_z = home_z + takeoff_height;

    ROS_INFO("[HOME] x=%.3f, y=%.3f, z=%.3f", home_x, home_y, home_z);
    ROS_INFO("[TARGET] takeoff_z=%.3f (relative height %.3f)",
             target_z, takeoff_height);

    geometry_msgs::PoseStamped setpoint =
        make_setpoint(home_x, home_y, home_z);

    // OFFBOARD 前以 20 Hz 预发送 100 帧，约 5 秒仿真时间。
    for (int i = 0; ros::ok() && i < 100; ++i) {
        setpoint.header.stamp = ros::Time::now();
        local_pos_pub.publish(setpoint);
        ROS_INFO_THROTTLE(1.0, "[PRESTREAM] Holding home before OFFBOARD");
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offboard_request;
    offboard_request.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_request;
    arm_request.request.value = true;

    FlightPhase flight_phase = FlightPhase::TAKEOFF;
    bool flight_started = false;
    ros::Time phase_start;
    ros::Time last_request(0);
    ros::Time last_force_disarm_request(0);

    while (ros::ok() && flight_phase != FlightPhase::COMPLETED) {
        // 先处理回调，本周期使用尽可能新的 state 和 pose。
        ros::spinOnce();
        const ros::Time now = ros::Time::now();

        // 始终保持 setpoint 时间戳更新。
        setpoint.header.stamp = now;

        // 持续请求 OFFBOARD；飞行开始后即使意外掉模式，也继续发目标并重试。
        if (current_state.mode != "OFFBOARD" &&
            now - last_request > ros::Duration(5.0)) {
            if (set_mode_client.call(offboard_request) &&
                offboard_request.response.mode_sent) {
                ROS_INFO("[MODE] OFFBOARD request sent");
            } else {
                ROS_WARN("[MODE] OFFBOARD request failed");
            }
            last_request = now;
        } else if (!flight_started && !current_state.armed &&
                   now - last_request > ros::Duration(5.0)) {
            if (arming_client.call(arm_request) && arm_request.response.success) {
                ROS_INFO("[ARM] Arm request accepted");
            } else {
                ROS_WARN("[ARM] Arm request failed");
            }
            last_request = now;
        }

        // 起飞前只保持 home；确认 mode 和 armed 后才启动任务计时。
        if (!flight_started) {
            setpoint = make_setpoint(home_x, home_y, home_z);
            local_pos_pub.publish(setpoint);

            if (current_state.mode == "OFFBOARD" && current_state.armed) {
                flight_started = true;
                phase_start = now;
                ROS_INFO("[PHASE] TAKEOFF started");
            } else {
                ROS_INFO_THROTTLE(
                    1.0, "[WAIT] mode=%s, armed=%s",
                    current_state.mode.c_str(),
                    current_state.armed ? "true" : "false");
            }
            rate.sleep();
            continue;
        }

        // 飞行开始后禁止自动重新解锁。只有已到近地范围，
        // 才能把 LANDING 中的 armed=false 判定为正常完成。
        if (!current_state.armed) {
            const double height_above_home =
                current_pose.pose.position.z - home_z;
            const double disarmed_xy_error = horizontal_error(
                current_pose.pose.position.x, current_pose.pose.position.y,
                home_x, home_y);
            const bool pose_is_fresh =
                now - last_pose_time <= ros::Duration(pose_timeout);
            const bool safely_near_home =
                pose_is_fresh &&
                height_above_home <= force_disarm_height + 0.05 &&
                disarmed_xy_error <= horizontal_tolerance;

            if (flight_phase == FlightPhase::LANDING && safely_near_home) {
                ROS_INFO("[DONE] Landing complete and armed=false confirmed");
                flight_phase = FlightPhase::COMPLETED;
                break;
            }

            ROS_ERROR(
                "[ABORT] Vehicle disarmed unexpectedly or away from home");
            return 2;
        }

        // 位姿变旧时保持最后一个安全目标，不推进状态机。
        if (now - last_pose_time > ros::Duration(pose_timeout)) {
            ROS_ERROR_THROTTLE(
                1.0, "[POSE] Local pose is stale; holding last setpoint");
            setpoint.header.stamp = now;
            local_pos_pub.publish(setpoint);
            rate.sleep();
            continue;
        }

        const double current_x = current_pose.pose.position.x;
        const double current_y = current_pose.pose.position.y;
        const double current_z = current_pose.pose.position.z;
        const double xy_error =
            horizontal_error(current_x, current_y, home_x, home_y);

        if (flight_phase == FlightPhase::TAKEOFF) {
            setpoint = make_setpoint(home_x, home_y, target_z);
            const double z_error = std::abs(current_z - target_z);

            ROS_INFO_THROTTLE(
                1.0, "[TAKEOFF] z=%.2f/%.2f, z_err=%.2f, xy_err=%.2f",
                current_z, target_z, z_error, xy_error);

            if (z_error <= takeoff_tolerance &&
                xy_error <= horizontal_tolerance) {
                flight_phase = FlightPhase::HOVER;
                phase_start = now;
                ROS_INFO("[PHASE] HOVER started for %.1f s", hover_duration);
            } else if (now - phase_start > ros::Duration(phase_timeout)) {
                flight_phase = FlightPhase::LANDING;
                phase_start = now;
                ROS_ERROR("[TIMEOUT] TAKEOFF timed out; switching to LANDING");
            }
        } else if (flight_phase == FlightPhase::HOVER) {
            setpoint = make_setpoint(home_x, home_y, target_z);
            const double elapsed = (now - phase_start).toSec();

            ROS_INFO_THROTTLE(
                1.0, "[HOVER] %.1f/%.1f s, pos=(%.2f, %.2f, %.2f), xy_err=%.2f",
                elapsed, hover_duration,
                current_x, current_y, current_z, xy_error);

            if (now - phase_start >= ros::Duration(hover_duration)) {
                flight_phase = FlightPhase::LANDING;
                phase_start = now;
                ROS_INFO("[PHASE] LANDING started");
            }
        } else if (flight_phase == FlightPhase::LANDING) {
            setpoint = make_setpoint(home_x, home_y, home_z);
            const double height_above_home = current_z - home_z;
            const bool force_disarm_ready =
                height_above_home <= force_disarm_height &&
                xy_error <= horizontal_tolerance;

            ROS_INFO_THROTTLE(
                1.0,
                "[LANDING] height=%.2f, xy_err=%.2f, force_ready=%s",
                height_above_home, xy_error,
                force_disarm_ready ? "true" : "false");

            if (force_disarm_ready &&
                now - last_force_disarm_request > ros::Duration(1.0)) {
                if (force_disarm(command_client)) {
                    ROS_WARN(
                        "[FORCE_DISARM] Command accepted; waiting for armed=false");
                } else {
                    ROS_ERROR(
                        "[FORCE_DISARM] Command failed; continuing landing target");
                }
                last_force_disarm_request = now;
            }

            if (now - phase_start > ros::Duration(phase_timeout)) {
                ROS_ERROR_THROTTLE(
                    2.0,
                    "[TIMEOUT] LANDING timed out; still publishing home target. "
                    "Check PX4/QGC and use Land if necessary");
            }
        }

        // 无论处在哪个飞行阶段，每轮都发布一次目标。
        setpoint.header.stamp = now;
        local_pos_pub.publish(setpoint);
        rate.sleep();
    }

    return 0;
}
```

### 5.3 这份实现解决了什么

| 阶段 1 问题 | 阶段 2 的处理 |
|---|---|
| FCU 连接后立刻读取默认 pose | 同时等待 `connected` 和 `have_pose` |
| 固定飞向世界原点 | 只记录一次 `home_x/home_y/home_z` |
| `hight` 是绝对高度 | 使用 `home_z + takeoff_height` |
| 没有 HOVER | 新增独立 `HOVER` 状态 |
| 新目标可能是零四元数 | 显式设置 `(0,0,0,1)` |
| `takeoff_height` 未读取 | 使用私有参数读取并校验 |
| 起飞只检查 z | 同时检查 z 和 xy 误差 |
| 没有阶段超时 | 起飞超时转降落，降落超时持续报警 |
| 调一次 Lock 就打印 DONE | 仿真强制上锁后仍等待 `armed=false` |
| 未解锁时每 5 秒总会重试解锁 | 任务开始后不再自动重新解锁 |
| 位姿失效仍推进任务 | 位姿过期时保持最后目标，不推进状态机 |

### 5.4 你必须能解释的几个变量

- `have_pose`：是否至少收到过一帧有限的局部位置。
- `home_*`：控制器启动后记录一次的起飞点。
- `target_z`：`home_z + takeoff_height`，是世界坐标中的目标 z。
- `flight_started`：是否已经确认 `OFFBOARD + armed`；它阻止任务中途自动重新解锁。
- `phase_start`：当前阶段的开始时间。
- `force_disarm_height`：仅仿真使用的强制上锁相对高度。
- `last_pose_time`：最后一帧有效位姿到达的 ROS 时间。
- `setpoint`：本周期应持续发布的安全目标。

## 6. 修改 launch 参数

打开：

```text
AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/
launch/autoarming_control.launch
```

保留原来的静态 TF 和 RViz 部分，把轨迹参数改成阶段 2 参数。推荐把参数定义成 launch arguments，这样不同悬停时长可从命令行切换，不必每轮编辑 XML。

```xml
<?xml version="1.0"?>
<launch>
    <arg name="rviz" default="true"/>
    <arg name="takeoff_height" default="2.0"/>
    <arg name="hover_duration" default="10.0"/>
    <arg name="takeoff_tolerance" default="0.15"/>
    <arg name="horizontal_tolerance" default="0.20"/>
    <arg name="force_disarm_height" default="0.10"/>
    <arg name="phase_timeout" default="60.0"/>
    <arg name="pose_timeout" default="1.0"/>

    <node pkg="tf2_ros" type="static_transform_publisher"
          name="map_to_camera_init"
          args="0 0 0  0 0 0 1  map  camera_init"/>

    <node pkg="offboard" type="autoarming_control"
          name="autoarming_control" output="screen">
        <param name="takeoff_height" value="$(arg takeoff_height)"/>
        <param name="hover_duration" value="$(arg hover_duration)"/>
        <param name="takeoff_tolerance" value="$(arg takeoff_tolerance)"/>
        <param name="horizontal_tolerance" value="$(arg horizontal_tolerance)"/>
        <param name="force_disarm_height" value="$(arg force_disarm_height)"/>
        <param name="phase_timeout" value="$(arg phase_timeout)"/>
        <param name="pose_timeout" value="$(arg pose_timeout)"/>

        <remap from="/mavros/state" to="/mavros/state"/>
        <remap from="/mavros/local_position/pose"
               to="/mavros/local_position/pose"/>
        <remap from="/mavros/setpoint_position/local"
               to="/mavros/setpoint_position/local"/>
        <remap from="/mavros/set_mode" to="/mavros/set_mode"/>
        <remap from="/mavros/cmd/arming" to="/mavros/cmd/arming"/>
        <remap from="/mavros/cmd/command" to="/mavros/cmd/command"/>
    </node>

    <group if="$(arg rviz)">
        <node launch-prefix="nice" pkg="rviz" type="rviz"
              name="rviz_a_loam"
              args="-d $(find offboard)/rviz_config/drone_path.rviz"/>
    </group>
</launch>
```

阶段 1 的 `flight_mode`、`target_laps`、`hight`、`side_length`、`radius` 和 `speed` 在阶段 2 不再使用，应从这个 launch 中移除，以免让人误以为它们仍会影响任务。

现在可以直接指定悬停时间：

```bash
roslaunch offboard autoarming_control.launch \
  rviz:=false hover_duration:=30.0 takeoff_height:=2.0
```

## 7. 编译与静态检查

### 7.1 修改 C++ 后必须编译

```bash
cd "$HOME/AstraDroneOpen/AstraDrone_ros1_ws"
catkin_make --pkg offboard
```

编译成功后，每个新终端都要重新加载环境：

```bash
source ~/.bashrc
source "$HOME/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash" --extend
```

只修改 `.launch` 不需要重新编译；修改 `.cpp` 后必须重新编译。

### 7.2 不要只看最后一行

合格结果是：

```text
Built target autoarming_control
```

如果编译失败，从输出中找到第一条 `error:`，不要被后面的连锁报错干扰。常见输入错误包括：

- 漏掉分号；
- 中文引号代替英文 `"`；
- 大括号没有配对；
- 把 `FlightPhase::HOVER` 拼错；
- 漏掉 `#include <cmath>`。

### 7.3 确认 launch 能解析

```bash
source "$HOME/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash" --extend
roslaunch --nodes offboard autoarming_control.launch rviz:=false
roslaunch offboard autoarming_control.launch --ros-args
```

第一条只列节点，不会启动飞机；第二条应列出新增的 launch arguments。重点确认存在 `hover_duration`、`takeoff_height` 和各项 tolerance/timeout。

### 7.4 核对源码确实读取了参数

```bash
rg -n 'pnh.param|FlightPhase::HOVER|have_pose|home_x|armed=false' \
  "$HOME/AstraDroneOpen/AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src/autoarming_control.cpp"
```

如果你的系统没有 `rg`，可以用编辑器搜索这些关键词。

## 8. 仿真实践总安排

按下面顺序做，不要一开始就跳到 60 秒测试：

| 轮次 | 出生点 | 悬停时间 | 目的 |
|---|---|---:|---|
| Run 01 | `(0,0,0.06)` | 10 s | 验证完整状态机和自动上锁 |
| Run 02 | `(0,0,0.06)` | 30 s | 验证稳定悬停并记录误差 |
| Run 03 | `(3,-2,0.06)` | 30 s | 证明使用相对 home，不会飞向原点 |
| Run 04 | `(0,0,0.06)` | 60 s | 验证长时间连续发布和稳定性 |

每轮只改变表中指定变量。不要同时修改高度、门限和超时，否则出现问题时无法判断原因。

## 9. Run 01：10 秒完整流程

### 9.1 准备五个终端

| 终端 | 用途 |
|---|---|
| 1 | `roscore` |
| 2 | PX4 SITL + Gazebo + MAVROS |
| 3 | 起飞前检查与飞行中观察 |
| 4 | rosbag 录制 |
| 5 | 启动控制器并观察状态机日志 |

每个新终端先执行：

```bash
source ~/.bashrc
source "$HOME/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash" --extend
cd "$HOME/AstraDroneOpen"
```

### 9.2 终端 1：启动 ROS master

```bash
roscore
```

保持运行。

### 9.3 终端 2：启动空场仿真

```bash
roslaunch \
  "$HOME/AstraDroneOpen/simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch" \
  world:="$HOME/AstraDroneOpen/simulation/astra_gazebo_worlds/example.world" \
  x:=0 y:=0 z:=0.06
```

等 Gazebo 模型出现、仿真时间前进、MAVROS 连接稳定后再继续。阶段 2 不需要启动 FAST-LIO，也不要使用 `pc_example.sh`，因为该脚本会自动启动 Offboard 控制器，不利于逐项检查。

### 9.4 终端 3：完成起飞前检查

按顺序执行。

检查仿真没有暂停：

```bash
rosparam get /use_sim_time
rostopic echo -n 1 /clock
```

检查 FCU 已连接、尚未解锁：

```bash
rostopic echo -n 1 /mavros/state
```

最低要求：

```yaml
connected: true
armed: false
```

检查位姿存在、持续更新、没有 NaN/Inf：

```bash
rostopic echo -n 1 /mavros/local_position/pose
rostopic hz /mavros/local_position/pose
```

观察 5～10 秒后用 `Ctrl+C` 停止 `hz`。

检查没有旧 setpoint 发布者：

```bash
rostopic info /mavros/setpoint_position/local
```

必须是：

```text
Publishers: None
```

检查所需服务存在：

```bash
rosservice info /mavros/set_mode
rosservice info /mavros/cmd/arming
```

任何一项失败，都先排错，不要启动控制器。

### 9.5 终端 4：开始录包

```bash
mkdir -p "$HOME/AstraDroneOpen/stage2_records"
cd "$HOME/AstraDroneOpen/stage2_records"

rosbag record -O run01_hover10_origin.bag \
  /mavros/state \
  /mavros/local_position/pose \
  /mavros/local_position/odom \
  /mavros/setpoint_position/local \
  /rosout
```

先录包，再启动控制器，这样才能保存 OFFBOARD、armed 和状态机日志的完整变化。

### 9.6 终端 5：启动控制器

```bash
roslaunch offboard autoarming_control.launch \
  rviz:=false hover_duration:=10.0 takeoff_height:=2.0
```

正常日志顺序应接近：

```text
[WAIT] connected=..., have_pose=...
[HOME] x=..., y=..., z=...
[TARGET] takeoff_z=...
[PRESTREAM] Holding home before OFFBOARD
[MODE] OFFBOARD request sent
[ARM] Arm request accepted
[PHASE] TAKEOFF started
[TAKEOFF] ...
[PHASE] HOVER started for 10.0 s
[HOVER] ...
[PHASE] LANDING started
[LANDING] ...
[LANDING] ... force_ready=true
[FORCE_DISARM] Command accepted; waiting for armed=false
[DONE] Landing complete and armed=false confirmed
```

实际等待时间会受 PX4 状态和 Gazebo 实时因子影响。日志文本略有差异不重要，状态顺序和完成条件必须一致。

### 9.7 飞行中做四项观察

终端 3 中依次执行；持续输出的命令用 `Ctrl+C` 结束，不会停止控制节点。

确认状态变化：

```bash
rostopic echo /mavros/state
```

你应看到：

```text
connected=true, armed=false
  -> mode=OFFBOARD
  -> armed=true
  -> 飞行完成后 armed=false
```

确认只有一个发布者：

```bash
rostopic info /mavros/setpoint_position/local
```

确认 setpoint 连续且接近 20 Hz：

```bash
rostopic hz /mavros/setpoint_position/local
```

确认悬停时目标位置不变：

```bash
rostopic echo /mavros/setpoint_position/local
```

在 `HOVER` 日志期间，x、y 应等于 home，z 应等于 `home_z + 2.0`，但每条消息的时间戳持续变化。

如果想直观看目标与实际高度：

```bash
rqt_plot \
  /mavros/local_position/pose/pose/position/z \
  /mavros/setpoint_position/local/pose/position/z
```

### 9.8 结束后完成双重确认

看到 `[DONE]` 后执行：

```bash
rostopic echo -n 1 /mavros/state
rostopic info /mavros/setpoint_position/local
```

合格结果：

```yaml
armed: false
```

控制可执行文件退出后，setpoint publisher 应消失。若日志显示 `[FORCE_DISARM] Command accepted`，但状态仍是 `armed: true`，本轮不能判定成功。

### 9.9 保存记录

在终端 4 按 `Ctrl+C` 停止录包，然后：

```bash
cd "$HOME/AstraDroneOpen/stage2_records"
rosbag info run01_hover10_origin.bag
```

确认五个话题都有消息，bag 时长覆盖控制器启动到最终上锁。

可导出 CSV：

```bash
rostopic echo -b run01_hover10_origin.bag -p \
  /mavros/setpoint_position/local > run01_setpoint.csv

rostopic echo -b run01_hover10_origin.bag -p \
  /mavros/local_position/pose > run01_actual_pose.csv
```

## 10. Run 02：30 秒悬停与误差验收

确认 Run 01 已落地并 `armed:false`。可以继续使用当前仿真，但启动下一轮前必须再次确认没有旧 publisher：

```bash
rostopic info /mavros/setpoint_position/local
```

开始新的 bag：

```bash
cd "$HOME/AstraDroneOpen/stage2_records"
rosbag record -O run02_hover30_origin.bag \
  /mavros/state \
  /mavros/local_position/pose \
  /mavros/local_position/odom \
  /mavros/setpoint_position/local \
  /rosout
```

另一个终端启动：

```bash
roslaunch offboard autoarming_control.launch \
  rviz:=false hover_duration:=30.0 takeoff_height:=2.0
```

本轮重点不是再次证明“能飞”，而是回答：

1. `HOVER` 是否持续了约 30 秒仿真时间？
2. 悬停期间 setpoint 的 x、y、z 是否保持不变？
3. setpoint 是否始终接近 20 Hz？
4. 实际 x、y、z 是否围绕目标小幅波动？
5. 悬停期间 `/mavros/state.mode` 是否始终为 `OFFBOARD`？
6. 最终是否确认 `armed:false`？

注意：代码使用 `ros::Time`。当 Gazebo 实时因子小于 1 时，30 秒仿真时间可能需要超过 30 秒现实时间；这不是悬停计时错误。

### 10.1 推荐误差检查

悬停目标为：

```text
x_target = home_x
y_target = home_y
z_target = home_z + takeoff_height
```

对悬停阶段的每个实际位置计算：

```text
e_x = |x_actual - x_target|
e_y = |y_actual - y_target|
e_z = |z_actual - z_target|
```

先采用以下学习验收值：

```text
无风空场，稳定进入 HOVER 后的 30 秒内：
max(e_x), max(e_y), max(e_z) 尽量均不超过 0.20 m
```

切换状态瞬间的爬升和下降样本不要混入悬停误差。阶段 2 可用 `rqt_plot` 或 CSV/表格软件手工查看；严格自动对齐时间戳和统计留到阶段 5。

## 11. Run 03：非零出生点测试

这是阶段 2 最关键的功能验证。它证明程序使用记录的 home，而不是碰巧在世界原点工作。

### 11.1 完整重启仿真

先确认上一轮已上锁，停止控制 launch、PX4/Gazebo 和 roscore，再重新启动。终端 2 改为：

```bash
roslaunch \
  "$HOME/AstraDroneOpen/simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch" \
  world:="$HOME/AstraDroneOpen/simulation/astra_gazebo_worlds/example.world" \
  x:=3.0 y:=-2.0 z:=0.06
```

重新执行第 9.4 节的全部起飞前检查。

### 11.2 录包并启动 30 秒测试

```bash
cd "$HOME/AstraDroneOpen/stage2_records"
rosbag record -O run03_hover30_nonzero_home.bag \
  /mavros/state \
  /mavros/local_position/pose \
  /mavros/local_position/odom \
  /mavros/setpoint_position/local \
  /rosout
```

另一个终端：

```bash
roslaunch offboard autoarming_control.launch \
  rviz:=false hover_duration:=30.0 takeoff_height:=2.0
```

### 11.3 本轮合格表现

- `[HOME]` 中 x、y 应接近本次局部坐标的非零起点；不要要求一定精确等于 Gazebo 的 `3,-2`，局部坐标原点取决于 PX4/MAVROS 的估计方式。
- 关键不是数值是否等于 `3,-2`，而是 setpoint 的 x/y 必须等于程序记录的 `home_x/home_y`。
- 起飞时飞机应主要垂直运动，不能先飞向 `(0,0)`。
- HOVER 与 LANDING 的 x/y 目标都必须保持为同一个 home。
- 最后应回到本次 home 附近并确认 `armed:false`。

如果 local pose 仍以起飞位置为 `(0,0)`，这是局部坐标系的合法表现。此时仅靠修改 Gazebo 出生点不能形成“local pose 非零”测试。你仍应通过 `[HOME]` 与 setpoint 一致性验证代码；若要强制测试非零局部 home，可在阶段 3 再引入明确的坐标偏移测试，不要在本阶段伪造定位数据。

## 12. Run 04：60 秒稳定性测试

前三轮全部通过后再执行。先按前面的格式启动一份名为 `run04_hover60_origin.bag` 的 rosbag，然后运行：

```bash
roslaunch offboard autoarming_control.launch \
  rviz:=false hover_duration:=60.0 takeoff_height:=2.0
```

重点检查：

- 60 秒内没有退出 OFFBOARD；
- setpoint 时间戳持续推进；
- setpoint 频率没有长时间中断；
- 实际位置没有持续漂移；
- 仍能正常降落和上锁。

## 13. 如何读懂一次实验结果

### 13.1 不要只看 Gazebo 画面

证据优先级建议是：

```text
/mavros/state             证明连接、模式和 armed
/mavros/setpoint_position/local  证明控制器发了什么
/mavros/local_position/pose      证明飞机实际到了哪里
控制器日志                    证明何时、为何切换状态
Gazebo 画面                  辅助观察
```

### 13.2 目标与实际不完全相等是正常的

setpoint 是命令，local pose 是反馈。真实闭环存在惯性、控制误差和采样延迟。目标固定而实际在目标附近小幅波动，并不等于程序错误；持续偏离、振荡扩大或目标本身变化才需要重点排查。

### 13.3 本阶段的合格标准

- 起飞前确实等待 `have_pose=true`。
- `[HOME]` 只打印和记录一次。
- 目标 z 等于 `home_z + takeoff_height`。
- TAKEOFF、HOVER、LANDING 中 setpoint 都持续发布，运行时接近 20 Hz。
- 所有 setpoint 都有更新时间戳、`frame_id` 和单位四元数。
- 悬停阶段目标 x/y/z 不变。
- 30 秒悬停时三轴最大误差先达到 `0.20 m` 量级。
- 非零出生点测试中不主动飞向世界原点。
- 降落近地条件必须连续保持一段时间才请求上锁。
- 只有 `/mavros/state` 显示 `armed:false` 才算完成。

## 14. 实验记录模板

每轮复制一份填写：

```markdown
## Stage 2 - Run XX

- 日期：
- Git commit 或 `git diff` 已保存：是 / 否
- world：example.world
- Gazebo spawn：x=，y=，z=
- takeoff_height：
- hover_duration：
- phase_timeout：
- home（控制器日志）：x=，y=，z=
- 起飞目标：x=，y=，z=
- FCU connected：true / false
- 起飞前 armed：false / 其他
- 唯一 setpoint publisher：
- setpoint 平均频率：
- 是否进入 TAKEOFF：
- 是否进入 HOVER：
- 实际悬停仿真时间：
- HOVER max |e_x|：
- HOVER max |e_y|：
- HOVER max |e_z|：
- 悬停期间是否保持 OFFBOARD：
- 是否进入 LANDING：
- disarm service response：
- 最终 /mavros/state.armed：
- bag 文件：
- 异常日志：
- 本轮结论：通过 / 不通过
- 如果不通过，下一轮只改哪一项：
```

## 15. 常见故障：按顺序排查

### 15.1 `Resource not found: offboard`

```bash
source ~/.bashrc
source "$HOME/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash" --extend
rospack find offboard
```

若 `devel/setup.bash` 不存在或包仍找不到，先解决工作空间编译问题。

### 15.2 修改代码后表现仍和阶段 1 一样

如果仍在飞圆或方形，通常是没有重新编译、当前终端没有重新 source，或旧控制节点还活着。

```bash
cd "$HOME/AstraDroneOpen/AstraDrone_ros1_ws"
catkin_make --pkg offboard
source devel/setup.bash --extend
rosnode list | grep -E 'autoarming|offb|position_control'
rostopic info /mavros/setpoint_position/local
```

确认编译目标是 `autoarming_control`，运行时只有一个发布者。

### 15.3 一直显示 `have_pose=false`

```bash
rostopic info /mavros/local_position/pose
rostopic echo -n 1 /mavros/local_position/pose
rostopic hz /mavros/local_position/pose
```

若话题没有发布者，问题在 PX4/MAVROS/估计器链路，不在状态机。若消息出现 NaN/Inf，停止整套仿真并重启，不要绕过有限数检查。

### 15.4 一直停在 PRESTREAM 或计时明显变慢

```bash
rostopic echo -n 2 /clock
```

Gazebo 暂停时 ROS 仿真时间不会前进。实时因子低时，5 秒仿真时间也会消耗更长现实时间。

### 15.5 请求 OFFBOARD 失败

检查：

```bash
rostopic hz /mavros/setpoint_position/local
rostopic echo -n 1 /mavros/state
rostopic info /mavros/setpoint_position/local
```

确认 setpoint 连续、FCU 已连接、Gazebo 未暂停且没有第二个发布者。`mode_sent=true` 只表示请求已发送，最终证据仍是 `/mavros/state.mode: "OFFBOARD"`。

### 15.6 OFFBOARD 成功但无法解锁

查看 PX4 终端和 QGroundControl 的 preflight 信息。不要用强制解锁掩盖传感器、估计器或安全检查错误。README 提到的 `No manual control input` 只是可能原因之一。

### 15.7 解锁后飞机不起飞

```bash
rostopic echo -n 1 /mavros/setpoint_position/local
rostopic echo -n 1 /mavros/local_position/pose
rosparam get /autoarming_control/takeoff_height
```

目标 z 应约等于 `[HOME] z + takeoff_height`。若目标正确但实际不动，检查模式是否掉出 OFFBOARD、Gazebo 是否暂停以及 PX4 错误。

### 15.8 起飞后飞向 `(0,0)`

检查 `[HOME]` 日志和 setpoint x/y。源码中 TAKEOFF、HOVER、LANDING 的 x/y 都应使用 `home_x/home_y`。如果 setpoint 仍为 0，可能运行了旧二进制，或你仍保留阶段 1 的 `pose.position.x = 0`、`y = 0`。

### 15.9 到达高度却不进入 HOVER

查看日志中的 `z_err` 和 `xy_err`。两者必须同时达标。先判断是否有持续偏差，再谨慎调整门限；不要一遇到问题就把门限改得很大。

推荐顺序：

1. 确认 home 和目标正确；
2. 确认实际位置稳定；
3. 确认无第二发布者；
4. 最后才把 `takeoff_tolerance` 从 `0.15` 小幅调到 `0.20`。

### 15.10 HOVER 立刻结束或时间不对

```bash
rosparam get /autoarming_control/hover_duration
rosparam get /use_sim_time
```

参数只在节点启动时读取一次。重新运行 launch 会用命令行 arg 或默认值写入参数服务器。计时使用仿真时间，不要只看现实世界秒表。

### 15.11 飞行中出现 `Local pose is stale`

```bash
rostopic hz /mavros/local_position/pose
rostopic echo -n 2 /clock
```

短暂出现可能是机器负载过高；持续出现说明位置反馈中断。代码会保持最后 setpoint 并暂停状态推进，但这不代表可以忽略故障。仿真中使用 QGC Land，保存证据后重启。

### 15.12 降落后不触发强制上锁

观察：

```text
height_above_home <= force_disarm_height
xy_error <= horizontal_tolerance
```

两个条件同时满足时日志应出现 `force_ready=true`。先检查 height 和 xy_err；不要为了提前上锁而大幅增大 `force_disarm_height`。

### 15.13 FORCE_DISARM 显示 accepted，但一直 `armed:true`

不要宣布完成，也不要启动下一轮：

1. 确认 Gazebo 模型确实接地；
2. 查看 PX4/QGC 为什么拒绝或延迟上锁；
3. 继续让节点发布降落目标；
4. 必要时在 QGC 中执行 Land，落地后再安全上锁；
5. 保存 bag 后完整重启。

### 15.14 LANDING 超时一直报警

这是故意设计的安全行为：程序不知道飞机是否仍在空中，因此不能因超时直接退出。查看 local pose、模式、Gazebo 接触状态和 PX4/QGC 日志。解决原因或安全 Land，而不是把 timeout 设成极大值掩盖问题。

### 15.15 悬停误差超过 0.20 m

先检查数据边界：是否把爬升、状态切换或下降样本算进 HOVER。再检查实时因子、setpoint 频率、第二发布者、位置反馈和目标是否稳定。本阶段先不要调 PX4 内环参数；如果默认空场闭环都不稳定，应先恢复阶段 0/1 基线。

## 16. 为什么本阶段不改 `position_control`

仓库中的 `position_control.cpp`、`position_control_lib.cpp` 和 `position_control.h` 可以用来识别反例，但不适合阶段 2 主线：

- 构造函数直接进入无限循环；
- `ReadParams()` 没被调用；
- 主循环没有 `ros::Rate` 限频；
- 目标头和四元数没有完整设置；
- 没有等待位姿、PRESTREAM、OFFBOARD、解锁、降落和上锁流程。

因此本阶段继续演进已经有完整 MAVROS 主链的 `autoarming_control`。不要同时运行两个节点控制同一话题。

## 17. 自测题与答案

### 题 1：悬停时 setpoint 是否可以停止发布？

不可以。悬停是持续发布同一个目标，PX4 仍依赖连续外部 setpoint 保持 OFFBOARD。

### 题 2：为什么不能在 `connected=true` 后立刻记录 home？

因为状态连接和位姿消息是两条不同的数据链；位姿回调可能尚未发生，变量可能仍是默认零值。

### 题 3：出生点 z 为 0.06，`takeoff_height=2.0`，目标 z 是多少？

约 2.06，因为参数表示相对高度，公式是 `home_z + takeoff_height`。

### 题 4：零旋转四元数是什么？

`(x,y,z,w)=(0,0,0,1)`。`(0,0,0,0)` 是非法四元数。

### 题 5：为什么 HOVER 不能用 `sleep(30)`？

它会阻塞主循环，使回调、setpoint 和安全判断都停止。

### 题 6：为什么 TAKEOFF 同时检查 z 和 xy？

只检查 z 可能在飞机横向偏离 home 时误判起飞已经完成。

### 题 7：为什么 `force_disarm_height` 不能设置得很大？

因为 `21196` 会立即停止电机。门限过大会在飞机仍明显离地时上锁；当前 `0.10 m` 只用于本仓库仿真。

### 题 8：服务返回 success 是否足以宣布完成？

不足。还必须从后续 `/mavros/state` 确认 `armed=false`。

### 题 9：降落超时为什么不直接退出？

因为飞机可能仍在空中，退出会中断 setpoint。程序继续发布降落目标并报警，等待安全处理。

### 题 10：为什么任务开始后不自动重新解锁？

如果飞机在异常或降落过程中上锁，自动重解锁可能造成二次起飞。任务前可重试解锁，任务开始后不应盲目重解锁。

### 题 11：修改 launch 参数是否需要重新编译？

不需要；修改 C++ 源码才需要重新编译。但每次改参数后要重启节点，因为参数只在启动时读取。

### 题 12：为什么代码要求 `/use_sim_time=true`？

因为降落末端使用了仿真强制上锁。若不在仿真时间环境中，程序直接退出，防止误用于真机。

### 题 13：FAST-LIO 是否参与本阶段控制闭环？

不参与。控制器仍订阅 `/mavros/local_position/pose`，不订阅 FAST-LIO 的 `/Odometry`。

## 18. 阶段 2 最终验收清单

### 代码

- [ ] 我增加了 `have_pose`，并拒绝 NaN/Inf 位姿。
- [ ] 我只记录一次 `home_x/home_y/home_z`。
- [ ] 起飞目标是 `home_z + takeoff_height`。
- [ ] 状态机包含 TAKEOFF、HOVER、LANDING、COMPLETED。
- [ ] 所有状态都在 20 Hz 主循环中持续发布目标。
- [ ] 所有 setpoint 都有时间戳、frame 和合法四元数。
- [ ] 我没有使用阻塞式 `sleep()`。
- [ ] 我增加了起飞/降落超时和位姿过期处理。
- [ ] 任务开始后不会自动重新解锁。
- [ ] 代码仅在 `/use_sim_time=true` 时允许强制上锁。
- [ ] `force_disarm_height` 保持为保守的小值。
- [ ] 只有确认 `armed=false` 才进入 COMPLETED。
- [ ] `catkin_make --pkg offboard` 编译成功。

### 仿真

- [ ] Run 01 完成 10 秒“起飞—悬停—降落—上锁”。
- [ ] Run 02 完成 30 秒悬停并记录误差。
- [ ] Run 03 完成非零出生点测试，没有主动飞向世界原点。
- [ ] Run 04 完成 60 秒悬停并正常降落、上锁。
- [ ] 运行时只有一个 setpoint 发布者。
- [ ] setpoint 发布频率接近 20 Hz，没有持续中断。
- [ ] 悬停期间保持 OFFBOARD。
- [ ] 30 秒悬停三轴最大误差达到约 0.20 m 量级。
- [ ] 每轮最终 `/mavros/state` 都显示 `armed:false`。
- [ ] 每轮 bag 覆盖了状态变化、setpoint、实际 pose 和 `/rosout`。

### 理解

- [ ] 我能画出本阶段数据闭环和状态机。
- [ ] 我能解释 connected、have_pose、mode 和 armed 的区别。
- [ ] 我能解释绝对 z 与相对起飞高度的区别。
- [ ] 我能解释为什么悬停仍要持续发送 setpoint。
- [ ] 我能解释为什么服务调用成功不等于最终上锁。
- [ ] 我能说明超时、位姿失效和模式丢失时程序会做什么。

全部勾选后，阶段 2 才算完成。此时你已经掌握了最小可靠自主飞行动作。阶段 3 再在这条可靠主线上增加航点队列、等待动作、返航和更完整的任务状态机。
