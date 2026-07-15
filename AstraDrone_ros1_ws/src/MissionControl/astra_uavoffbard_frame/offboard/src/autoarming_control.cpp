/**
 * @file autoarming_control.cpp
 * @brief PX4 Offboard 自动解锁及圆形/方形轨迹飞行示例
 *
 * 程序总体流程：等待 FCU -> 预发送 setpoint -> 请求 OFFBOARD -> 请求解锁
 *              -> TAKEOFF -> TRACKING -> LANDING -> COMPLETED。
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandLong.h>
#include <cmath>
#include <vector>

/*
 * 功能块 1：保存控制器运行时需要使用的最新飞控反馈和高度信息。
 * current_state、current_pose 由 ROS 回调函数持续更新；hight 是当前源码中
 * 使用的目标绝对高度，initial_height 用于记录起飞前的局部 z 坐标。
 */
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
double hight;
double initial_height = 0.0;  // record height before takeoff

/*
 * 功能块 2：定义任务层飞行阶段。
 * 这是本节点自己的状态机，不是 PX4 的 OFFBOARD 等飞行模式。
 */
enum class FlightPhase {
    TAKEOFF,
    TRACKING,
    LANDING,
    COMPLETED
};

/*
 * 功能块 3：接收 MAVROS 发布的飞控状态和局部位姿。
 * state_cb 更新连接、模式和解锁状态；pose_cb 更新无人机的实际位置与姿态。
 * 主循环通过 ros::spinOnce() 触发这些回调。
 */
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

/*
 * 功能块 4：计算两个三维位置之间的欧氏距离。
 * TRACKING 阶段用该距离判断无人机是否已经接近当前轨迹目标。
 */
double calculate_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    double dz = p1.pose.position.z - p2.pose.position.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

/*
 * 功能块 5：根据归一化进度 t 生成一圈方形轨迹的 XY 坐标。
 * t 从 0 增长到 1，方形以世界原点为中心，四条边的总长度为 4*side_length。
 */
std::pair<double, double> get_square_position(double t, double side_length) {
    double perimeter = 4 * side_length;
    double distance = t * perimeter;
    double half_side = side_length / 2.0;
    if (distance <= side_length) {
        return {-half_side + distance, -half_side};
    } else if (distance <= 2 * side_length) {
        return {half_side, -half_side + (distance - side_length)};
    } else if (distance <= 3 * side_length) {
        return {half_side - (distance - 2 * side_length), half_side};
    } else {
        return {-half_side, half_side - (distance - 3 * side_length)};
    }
}

/*
 * 功能块 6：根据归一化进度 t 生成一圈圆形轨迹的 XY 坐标。
 * 圆心位于世界原点，t=0 对应 (radius, 0)，t=1 时完成一整圈。
 */
std::pair<double, double> get_circle_position(double t, double radius) {
    double angle = t * 2 * M_PI;
    double x = radius * cos(angle);
    double y = radius * sin(angle);
    return {x, y};
}

/*
 * 功能块 7：任务结束时通过 MAVROS 通用命令服务请求上锁。
 * command=400 是 MAVLink 的解锁/上锁命令，param1=0 表示上锁。
 */
void Lock(ros::ServiceClient& land_client) {
    mavros_msgs::CommandLong cmd;
    cmd.request.broadcast = false;
    cmd.request.command = 400;
    cmd.request.confirmation = 0;
    cmd.request.param1 = 0.0;
    cmd.request.param2 = 21196.0;
    cmd.request.param3 = 0.0;
    cmd.request.param4 = 0.0;
    cmd.request.param5 = 0.0;
    cmd.request.param6 = 0.0;
    cmd.request.param7 = 0.0;
    if(land_client.call(cmd) && cmd.response.success){
        ROS_INFO_THROTTLE(2, "Vehicle disarmed!");
    }
}

int main(int argc, char **argv)
{
    /*
     * 功能块 8：初始化 ROS 节点并读取轨迹任务参数。
     * nh 读取普通命名空间参数，nh_private("~") 读取节点私有参数。
     */
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string flight_mode;
    int target_laps;
    double side_length;
    double radius;

    nh.param<std::string>("flight_mode", flight_mode, "square");
    nh_private.param("hight", hight, 3.0);
    nh_private.param("target_laps", target_laps, 1);
    nh_private.param("side_length", side_length, 8.0);
    nh_private.param("radius", radius, 2.0);

    /*
     * 功能块 9：建立本节点与 MAVROS 之间的 ROS 通信接口。
     * 两个订阅者读取状态和实际位姿；一个发布者持续发送目标位姿；
     * 三个服务客户端分别负责解锁、切换模式和任务结束后的上锁。
     */
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

    ros::Rate rate(20.0);

    /*
     * 功能块 10：等待 MAVROS 与 PX4 FCU 建立连接，然后记录初始高度。
     * 循环保持 20 Hz，并通过 spinOnce() 更新 current_state/current_pose。
     */
    while(ros::ok() && !current_state.connected){
        ROS_INFO_THROTTLE(3, "Waiting for FCU connection...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO_STREAM("FCU connected");

    initial_height = current_pose.pose.position.z;

    /*
     * 功能块 11：进入 OFFBOARD 前预发送 100 帧当前位置 setpoint。
     * 在 20 Hz 下约为 5 秒，使 PX4 先检测到连续有效的外部目标流。
     */
    for(int i = 100; ros::ok() && i > 0; --i){
        geometry_msgs::PoseStamped hold = current_pose;
        hold.header.stamp = ros::Time::now();
        hold.header.frame_id = "map";
        local_pos_pub.publish(hold);
        ROS_INFO_THROTTLE(1, "[PREHEAT] Sending current position setpoint before OFFBOARD...");
        ros::spinOnce();
        rate.sleep();
    }

    /*
     * 功能块 12：准备 OFFBOARD/解锁服务请求和轨迹状态变量。
     * t_target 表示一圈内的归一化进度，completed_laps 记录已完成圈数。
     */
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    FlightPhase flight_phase = FlightPhase::TAKEOFF;
    double trajectory_length = (flight_mode == "square")
        ? (4 * side_length)
        : (2 * M_PI * radius);

    double t_target = 0.0;
    int completed_laps = 0;
    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.position.z = hight;

    /*
     * 功能块 13：执行任务主循环。
     * 循环先按 5 秒间隔请求 OFFBOARD 和解锁；未解锁时持续发布当前位置
     * hold setpoint；解锁成功后再根据 FlightPhase 执行具体飞行任务。
     */
    while(ros::ok() && flight_phase != FlightPhase::COMPLETED){
        if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            ROS_INFO("[ACTION] Attempting to set OFFBOARD mode...");
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("[OK] OFFBOARD mode request sent");
            } else {
                ROS_WARN("[WARN] Failed to send OFFBOARD request");
            }
            last_request = ros::Time::now();
        }
        else if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
            ROS_INFO("[ACTION] Attempting to arm vehicle...");
            if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                ROS_INFO("[OK] Vehicle armed");
            } else {
                ROS_WARN("[WARN] Failed to arm vehicle");
            }
            last_request = ros::Time::now();
        }

        if (!current_state.armed) {
            geometry_msgs::PoseStamped hold = current_pose;
            hold.header.stamp = ros::Time::now();
            hold.header.frame_id = "map";
            local_pos_pub.publish(hold);

            if (current_state.mode != "OFFBOARD") {
                ROS_INFO_THROTTLE(1, "[WAIT] OFFBOARD not active: sending current position setpoint...");
            } else {
                ROS_INFO_THROTTLE(1, "[WAIT] OFFBOARD active: waiting for arming, sending current position setpoint...");
            }
            ROS_INFO_THROTTLE(2, "[STATE] mode=%s, armed=%s, pos(%.2f, %.2f, %.2f)",
                              current_state.mode.c_str(),
                              current_state.armed ? "true" : "false",
                              current_pose.pose.position.x,
                              current_pose.pose.position.y,
                              current_pose.pose.position.z);

            ros::spinOnce();
            rate.sleep();
            continue;
        }

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();

        /*
         * TAKEOFF：持续发布世界坐标 (0,0,hight)。
         * 实际 z 与目标高度的误差小于 0.1 m 后切换到 TRACKING。
         */
        if (flight_phase == FlightPhase::TAKEOFF) {
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = hight;
            local_pos_pub.publish(pose);
            if (std::abs(current_pose.pose.position.z - hight) < 0.1) {
                ROS_INFO("[PHASE] Reached takeoff altitude, switching to TRACKING");
                flight_phase = FlightPhase::TRACKING;
                t_target = 0.0;
            } else {
                ROS_INFO_THROTTLE(1, "[TAKEOFF] Target=%.2f, Current=%.2f", hight, current_pose.pose.position.z);
            }
        }
        /*
         * TRACKING：根据 flight_mode 选择方形或圆形目标。
         * 当飞机距离当前目标小于 1 m 时推进 t_target；t_target 每到 1
         * 记为完成一圈，达到 target_laps 后切换到 LANDING。
         */
        else if (flight_phase == FlightPhase::TRACKING) {
            std::pair<double, double> target_xy;
            if (flight_mode == "square") {
                target_xy = get_square_position(t_target, side_length);
            } else {
                target_xy = get_circle_position(t_target, radius);
            }

            target_pose.pose.position.x = target_xy.first;
            target_pose.pose.position.y = target_xy.second;
            target_pose.pose.position.z = hight;
            target_pose.header.stamp = ros::Time::now();
            target_pose.header.frame_id = "map";

            double d = calculate_distance(current_pose, target_pose);
            if (d < 1.0) {
                double advance = 1.0 - d;
                double delta_t = advance / trajectory_length;
                t_target += delta_t;
                if (t_target >= 1.0) {
                    t_target -= 1.0;
                    completed_laps++;
                    ROS_INFO("[TRACK] Completed lap %d/%d", completed_laps, target_laps);
                    if (completed_laps >= target_laps) {
                        ROS_INFO("[PHASE] All laps done, switching to LANDING");
                        flight_phase = FlightPhase::LANDING;
                    }
                }
            }

            if (flight_phase == FlightPhase::TRACKING) {
                if (flight_mode == "square") {
                    auto xy = get_square_position(t_target, side_length);
                    target_pose.pose.position.x = xy.first;
                    target_pose.pose.position.y = xy.second;
                } else {
                    auto xy = get_circle_position(t_target, radius);
                    target_pose.pose.position.x = xy.first;
                    target_pose.pose.position.y = xy.second;
                }
                local_pos_pub.publish(target_pose);
                ROS_INFO_THROTTLE(1, "[TRACK] t=%.3f target(%.2f,%.2f,%.2f) dist=%.2f",
                                  t_target,
                                  target_pose.pose.position.x,
                                  target_pose.pose.position.y,
                                  target_pose.pose.position.z, d);
            }
        }
        /*
         * LANDING：持续发布世界坐标 (0,0,initial_height)，让飞机返回原点并下降。
         * 高度差不超过 0.10 m 时调用 Lock() 请求上锁，然后结束状态机。
         */
        else if (flight_phase == FlightPhase::LANDING) {
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = initial_height;
            double height_above_ground = current_pose.pose.position.z - initial_height;
            ROS_INFO_THROTTLE(1, "[LAND] Height above ground: %.2f m", height_above_ground);
            if (height_above_ground <= 0.10) {
                Lock(land_client);
                flight_phase = FlightPhase::COMPLETED;
                ROS_INFO("[DONE] Landing completed and disarmed");
            }
            local_pos_pub.publish(pose);
        }

        /*
         * 每轮末尾处理最新 ROS 回调并维持 20 Hz。
         * 持续循环发布是 PX4 保持 OFFBOARD 所必需的条件。
         */
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
