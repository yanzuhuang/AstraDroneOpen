/**
 * @file autoarming_control.cpp
 * @brief PX4 Offboard 仿真：原地起飞、定点悬停、降落并强制上锁
 *
 * 重要：本程序的降落末端使用 MAVLink 21196 强制上锁，只允许在
 * /use_sim_time=true 的仿真中运行，严禁直接用于真机。
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <cmath>

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
    command.request.command = 400;       // MAV_CMD_COMPONENT_ARM_DISARM
    command.request.confirmation = 0;
    command.request.param1 = 0.0;        // 0：上锁
    command.request.param2 = 21196.0;    // 强制上锁魔数，仅限仿真
    command.request.param3 = 0.0;
    command.request.param4 = 0.0;
    command.request.param5 = 0.0;
    command.request.param6 = 0.0;
    command.request.param7 = 0.0;

    return command_client.call(command) && command.response.success;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    bool use_sim_time = false;
    nh.param("/use_sim_time", use_sim_time, false);
    if (!use_sim_time) {
        ROS_FATAL(
            "[SAFETY] Force-disarm landing is simulation-only. "
            "Refusing to run because /use_sim_time is not true");
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
        ROS_FATAL("[PARAM] Invalid height, duration, tolerance or timeout");
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

    const double home_x = current_pose.pose.position.x;
    const double home_y = current_pose.pose.position.y;
    const double home_z = current_pose.pose.position.z;
    const double target_z = home_z + takeoff_height;

    ROS_INFO("[HOME] x=%.3f, y=%.3f, z=%.3f", home_x, home_y, home_z);
    ROS_INFO("[TARGET] takeoff_z=%.3f (relative height %.3f)",
             target_z, takeoff_height);
    ROS_WARN(
        "[SAFETY] Simulation force-disarm enabled at %.2f m above home",
        force_disarm_height);

    geometry_msgs::PoseStamped setpoint =
        make_setpoint(home_x, home_y, home_z);

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
        ros::spinOnce();
        const ros::Time now = ros::Time::now();
        setpoint.header.stamp = now;

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
                ROS_INFO(
                    "[DONE] Simulation landing complete and armed=false confirmed");
                flight_phase = FlightPhase::COMPLETED;
                break;
            }

            ROS_ERROR(
                "[ABORT] Vehicle disarmed unexpectedly or away from home");
            return 2;
        }

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
                1.0,
                "[HOVER] %.1f/%.1f s, pos=(%.2f, %.2f, %.2f), xy_err=%.2f",
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
                    "[TIMEOUT] LANDING timed out; still publishing home target");
            }
        }

        setpoint.header.stamp = now;
        local_pos_pub.publish(setpoint);
        rate.sleep();
    }

    return 0;
}
