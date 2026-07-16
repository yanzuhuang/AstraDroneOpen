/**
 * @file autoarming_control.cpp
 * @brief PX4 Offboard 仿真：起飞、航点任务、返航和 PX4 原生降落
 *
 * 航点从私有参数 ~waypoints 读取，位置是相对 home 的 ENU 偏移，yaw 是
 * map/ENU 坐标系中的绝对角度。返航后切换到 AUTO.LAND，由 PX4 完成下降、
 * 落地检测和自动上锁；程序不会发送任何上锁命令。仅允许在仿真中运行。
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <cmath>
#include <string>
#include <vector>

namespace {

// 全局状态缓存与任务阶段定义：回调更新飞控状态，主循环据此推进任务。
constexpr double kPi = 3.14159265358979323846;

mavros_msgs::State current_state;
mavros_msgs::ExtendedState current_extended_state;
geometry_msgs::PoseStamped current_pose;
bool have_pose = false;
bool have_extended_state = false;
ros::Time last_pose_time;
ros::Time last_extended_state_time;

struct Waypoint {
    // x/y/z 为相对 home 的 ENU 偏移（m），yaw 为 map/ENU 绝对航向（rad）。
    double x;
    double y;
    double z;
    double yaw;
    double hold_sec;
};

enum class FlightPhase {
    TAKEOFF,
    INITIAL_HOVER,
    WAYPOINTS,
    FAILSAFE_HOVER,
    RETURN_HOME,
    LANDING,
    COMPLETED
};

// MAVROS 数据回调：缓存连接、模式、解锁状态以及最新的有效本地位姿。
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg) {
    current_extended_state = *msg;
    have_extended_state = true;
    last_extended_state_time = ros::Time::now();
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

// 位姿与目标辅助函数：完成航向换算、目标位姿构造和到达误差计算。
double yaw_from_pose(const geometry_msgs::PoseStamped& pose) {
    const geometry_msgs::Quaternion& q = pose.pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::PoseStamped make_setpoint(
    double x, double y, double z, double yaw) {
    geometry_msgs::PoseStamped setpoint;
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "map";
    setpoint.pose.position.x = x;
    setpoint.pose.position.y = y;
    setpoint.pose.position.z = z;
    setpoint.pose.orientation.x = 0.0;
    setpoint.pose.orientation.y = 0.0;
    setpoint.pose.orientation.z = std::sin(yaw * 0.5);
    setpoint.pose.orientation.w = std::cos(yaw * 0.5);
    return setpoint;
}

double position_error(const geometry_msgs::PoseStamped& pose,
                      const geometry_msgs::PoseStamped& target) {
    const double dx = pose.pose.position.x - target.pose.position.x;
    const double dy = pose.pose.position.y - target.pose.position.y;
    const double dz = pose.pose.position.z - target.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double horizontal_error(double x, double y, double target_x, double target_y) {
    const double dx = x - target_x;
    const double dy = y - target_y;
    return std::sqrt(dx * dx + dy * dy);
}

double angle_error(double actual, double target) {
    return std::abs(std::atan2(std::sin(actual - target),
                              std::cos(actual - target)));
}

// 航点参数解析：从私有参数 ~waypoints 读取并校验任务航点列表。
bool xml_number(const XmlRpc::XmlRpcValue& value, double* output) {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        *output = static_cast<double>(value);
        return true;
    }
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        *output = static_cast<int>(value);
        return true;
    }
    return false;
}

bool waypoint_member(const XmlRpc::XmlRpcValue& item,
                     const std::string& name,
                     double* output) {
    if (!item.hasMember(name)) {
        ROS_ERROR("[PARAM] Waypoint is missing '%s'", name.c_str());
        return false;
    }
    if (!xml_number(item[name], output) || !std::isfinite(*output)) {
        ROS_ERROR("[PARAM] Waypoint '%s' must be a finite number", name.c_str());
        return false;
    }
    return true;
}

bool load_waypoints(ros::NodeHandle& pnh,
                    double default_hold_sec,
                    std::vector<Waypoint>* waypoints) {
    XmlRpc::XmlRpcValue list;
    if (!pnh.getParam("waypoints", list)) {
        ROS_ERROR("[PARAM] Missing private parameter ~waypoints");
        return false;
    }
    if (list.getType() != XmlRpc::XmlRpcValue::TypeArray || list.size() == 0) {
        ROS_ERROR("[PARAM] ~waypoints must be a non-empty YAML list");
        return false;
    }

    for (int i = 0; i < list.size(); ++i) {
        const XmlRpc::XmlRpcValue item = list[i];
        if (item.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR("[PARAM] Waypoint %d must be a YAML dictionary", i + 1);
            return false;
        }

        Waypoint waypoint;
        double yaw_deg = 0.0;
        if (!waypoint_member(item, "x", &waypoint.x) ||
            !waypoint_member(item, "y", &waypoint.y) ||
            !waypoint_member(item, "z", &waypoint.z) ||
            !waypoint_member(item, "yaw_deg", &yaw_deg)) {
            return false;
        }

        waypoint.hold_sec = default_hold_sec;
        if (item.hasMember("hold_sec") &&
            (!xml_number(item["hold_sec"], &waypoint.hold_sec) ||
             !std::isfinite(waypoint.hold_sec))) {
            ROS_ERROR("[PARAM] Waypoint %d hold_sec must be a finite number", i + 1);
            return false;
        }
        if (waypoint.z <= 0.0 || waypoint.hold_sec < 0.0) {
            ROS_ERROR("[PARAM] Waypoint %d requires z > 0 and hold_sec >= 0", i + 1);
            return false;
        }

        waypoint.yaw = yaw_deg * kPi / 180.0;
        waypoints->push_back(waypoint);
        ROS_INFO(
            "[MISSION] WP%d relative=(%.2f, %.2f, %.2f), yaw=%.1f deg, hold=%.1f s",
            i + 1, waypoint.x, waypoint.y, waypoint.z,
            yaw_deg, waypoint.hold_sec);
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    // 初始化 ROS 节点，并阻止自动 OFFBOARD/解锁任务误用于真机。
    ros::init(argc, argv, "autoarming_control");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    bool use_sim_time = false;
    nh.param("/use_sim_time", use_sim_time, false);
    if (!use_sim_time) {
        ROS_FATAL(
            "[SAFETY] Autonomous OFFBOARD mission is simulation-only. "
            "Refusing to run because /use_sim_time is not true");
        return 1;
    }

    double takeoff_height;
    double initial_hover_duration;
    double takeoff_tolerance;
    double horizontal_tolerance;
    double waypoint_tolerance;
    double yaw_tolerance_deg;
    double default_waypoint_hold;
    double waypoint_timeout;
    double return_hold_time;
    double return_timeout;
    double failsafe_hover_duration;
    double land_mode_retry_interval;
    double extended_state_timeout;
    double phase_timeout;
    double pose_timeout;

    // 关键可调参数（均为私有参数，可在 launch/YAML 中通过 ~参数名覆盖）。
    pnh.param("takeoff_height", takeoff_height, 3.0);  // 相对 home 起飞高度（m）
    pnh.param("initial_hover_duration", initial_hover_duration, 3.0);  // 起飞后悬停时间（s）
    pnh.param("takeoff_tolerance", takeoff_tolerance, 0.15);  // 起飞高度允许误差（m）
    pnh.param("horizontal_tolerance", horizontal_tolerance, 0.20);  // 起降水平允许误差（m）
    pnh.param("waypoint_tolerance", waypoint_tolerance, 0.20);  // 航点位置允许误差（m）
    pnh.param("yaw_tolerance_deg", yaw_tolerance_deg, 10.0);  // 航点航向允许误差（deg）
    pnh.param("default_waypoint_hold", default_waypoint_hold, 2.0);  // 航点默认停留时间（s）
    pnh.param("waypoint_timeout", waypoint_timeout, 30.0);  // 单个航点超时（s）
    pnh.param("return_hold_time", return_hold_time, 2.0);  // 返航到位停留时间（s）
    pnh.param("return_timeout", return_timeout, 45.0);  // 返航超时，超时后原地降落（s）
    pnh.param("failsafe_hover_duration", failsafe_hover_duration, 5.0);  // 航点失败后悬停时间（s）
    pnh.param("land_mode_retry_interval", land_mode_retry_interval, 1.0);  // AUTO.LAND 重试间隔（s）
    pnh.param("extended_state_timeout", extended_state_timeout, 3.0);  // 落地状态新鲜度/确认窗口（s）
    pnh.param("phase_timeout", phase_timeout, 60.0);  // 起飞/降落阶段超时提示阈值（s）
    pnh.param("pose_timeout", pose_timeout, 1.0);  // 位姿数据失效阈值（s）

    // 参数合法性检查，避免错误的高度、容差或超时配置进入飞行流程。
    const double yaw_tolerance = yaw_tolerance_deg * kPi / 180.0;
    if (takeoff_height <= 0.0 || initial_hover_duration < 0.0 ||
        takeoff_tolerance <= 0.0 || horizontal_tolerance <= 0.0 ||
        waypoint_tolerance <= 0.0 || yaw_tolerance_deg <= 0.0 ||
        yaw_tolerance_deg > 180.0 || default_waypoint_hold < 0.0 ||
        waypoint_timeout <= 0.0 || return_hold_time < 0.0 ||
        return_timeout <= 0.0 || failsafe_hover_duration < 0.0 ||
        land_mode_retry_interval <= 0.0 ||
        extended_state_timeout <= 0.0 ||
        phase_timeout <= 0.0 || pose_timeout <= 0.0) {
        ROS_FATAL("[PARAM] Invalid mission height, tolerance, duration or timeout");
        return 1;
    }

    std::vector<Waypoint> waypoints;
    if (!load_waypoints(pnh, default_waypoint_hold, &waypoints)) {
        return 1;
    }

    // 建立 MAVROS 状态/位姿通信，以及模式切换和解锁服务。
    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber extended_state_sub =
        nh.subscribe<mavros_msgs::ExtendedState>(
            "mavros/extended_state", 10, extended_state_cb);
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

    ros::Rate rate(20.0);  // 关键参数：OFFBOARD 目标点发布与控制循环频率（Hz）

    // 等待飞控连接、位姿和 PX4 落地状态，再将当前位置记录为 home。
    while (ros::ok() &&
           (!current_state.connected || !have_pose || !have_extended_state)) {
        ROS_INFO_THROTTLE(
            2.0,
            "[WAIT_FCU] connected=%s, have_pose=%s, have_extended_state=%s",
            current_state.connected ? "true" : "false",
            have_pose ? "true" : "false",
            have_extended_state ? "true" : "false");
        ros::spinOnce();
        rate.sleep();
    }
    if (!ros::ok()) {
        return 0;
    }

    const double home_x = current_pose.pose.position.x;
    const double home_y = current_pose.pose.position.y;
    const double home_z = current_pose.pose.position.z;
    const double home_yaw = yaw_from_pose(current_pose);
    const double safe_z = home_z + takeoff_height;

    double landing_x = home_x;
    double landing_y = home_y;
    double landing_hold_z = home_z;
    double landing_yaw = home_yaw;

    ROS_INFO("[HOME] x=%.3f, y=%.3f, z=%.3f, yaw=%.1f deg",
             home_x, home_y, home_z, home_yaw * 180.0 / kPi);
    ROS_INFO("[MISSION] %zu waypoints loaded; safe_z=%.3f",
             waypoints.size(), safe_z);
    ROS_INFO(
        "[SAFETY] Landing uses PX4 AUTO.LAND; no disarm command will be sent");

    geometry_msgs::PoseStamped setpoint =
        make_setpoint(home_x, home_y, home_z, home_yaw);

    // 切入 OFFBOARD 前预发送目标点，满足 PX4 对连续 setpoint 数据流的要求。
    // 关键参数：100 次 / 20 Hz，约预发送 5 秒。
    for (int i = 0; ros::ok() && i < 100; ++i) {
        setpoint.header.stamp = ros::Time::now();
        local_pos_pub.publish(setpoint);
        ROS_INFO_THROTTLE(1.0, "[PRESTREAM] Holding home before OFFBOARD");
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offboard_request;
    offboard_request.request.custom_mode = "OFFBOARD";
    mavros_msgs::SetMode land_request;
    land_request.request.custom_mode = "AUTO.LAND";
    mavros_msgs::CommandBool arm_request;
    arm_request.request.value = true;

    // 初始化任务状态机、超时计时器和故障悬停目标。
    FlightPhase flight_phase = FlightPhase::TAKEOFF;
    bool flight_started = false;
    bool recovering_offboard = false;
    bool target_reached = false;
    bool auto_land_confirmed = false;
    std::size_t waypoint_index = 0;
    ros::Time phase_start;
    ros::Time reached_start;
    ros::Time last_request(0);
    ros::Time last_land_request(0);
    ros::Time landing_disarmed_start(0);
    geometry_msgs::PoseStamped failsafe_setpoint = setpoint;

    while (ros::ok() && flight_phase != FlightPhase::COMPLETED) {
        ros::spinOnce();
        const ros::Time now = ros::Time::now();
        setpoint.header.stamp = now;

        // 降落前自动请求 OFFBOARD 和解锁；关键参数：失败后每 5 秒重试一次。
        if (current_state.connected &&
            flight_phase != FlightPhase::LANDING &&
            current_state.mode != "OFFBOARD" &&
            now - last_request > ros::Duration(5.0)) {
            if (set_mode_client.call(offboard_request) &&
                offboard_request.response.mode_sent) {
                ROS_INFO("[MODE] OFFBOARD request sent");
            } else {
                ROS_WARN("[MODE] OFFBOARD request failed");
            }
            last_request = now;
        } else if (current_state.connected && !flight_started &&
                   !current_state.armed &&
                   now - last_request > ros::Duration(5.0)) {
            if (arming_client.call(arm_request) && arm_request.response.success) {
                ROS_INFO("[ARM] Arm request accepted");
            } else {
                ROS_WARN("[ARM] Arm request failed");
            }
            last_request = now;
        }

        // OFFBOARD 与解锁均成功前固定在 home，不开始任务计时。
        if (!flight_started) {
            setpoint = make_setpoint(home_x, home_y, home_z, home_yaw);
            local_pos_pub.publish(setpoint);

            if (current_state.mode == "OFFBOARD" && current_state.armed) {
                flight_started = true;
                phase_start = now;
                ROS_INFO("[PHASE] TAKEOFF started");
            } else {
                ROS_INFO_THROTTLE(
                    1.0, "[ARM_AND_OFFBOARD] mode=%s, armed=%s",
                    current_state.mode.c_str(),
                    current_state.armed ? "true" : "false");
            }
            rate.sleep();
            continue;
        }

        // 飞行安全检查：连接/位姿异常时保持目标，意外上锁时中止任务。
        if (!current_state.connected) {
            ROS_ERROR_THROTTLE(
                1.0, "[FCU] Connection lost; holding last setpoint");
            local_pos_pub.publish(setpoint);
            rate.sleep();
            continue;
        }

        if (!current_state.armed) {
            if (flight_phase != FlightPhase::LANDING) {
                ROS_ERROR("[ABORT] Vehicle disarmed unexpectedly before landing");
                return 2;
            }

            const bool extended_state_is_fresh =
                have_extended_state &&
                now - last_extended_state_time <=
                    ros::Duration(extended_state_timeout);
            const bool px4_confirms_landing =
                extended_state_is_fresh &&
                current_extended_state.landed_state ==
                    mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;

            if (px4_confirms_landing) {
                ROS_INFO(
                    "[DONE] PX4 landing complete: armed=false, "
                    "landed_state=ON_GROUND");
                flight_phase = FlightPhase::COMPLETED;
                break;
            }

            if (landing_disarmed_start.isZero()) {
                landing_disarmed_start = now;
            }
            if (now - landing_disarmed_start <=
                ros::Duration(extended_state_timeout)) {
                ROS_WARN_THROTTLE(
                    1.0,
                    "[LANDING] armed=false; waiting for fresh ON_GROUND state");
                rate.sleep();
                continue;
            }

            ROS_ERROR(
                "[ABORT] Vehicle disarmed but PX4 did not confirm ON_GROUND");
            return 2;
        }
        landing_disarmed_start = ros::Time(0);

        if (flight_phase != FlightPhase::LANDING &&
            now - last_pose_time > ros::Duration(pose_timeout)) {
            ROS_ERROR_THROTTLE(
                1.0, "[POSE] Local pose is stale; holding last setpoint");
            setpoint.header.stamp = now;
            local_pos_pub.publish(setpoint);
            rate.sleep();
            continue;
        }
        if (flight_phase == FlightPhase::LANDING &&
            !auto_land_confirmed &&
            now - last_pose_time > ros::Duration(pose_timeout)) {
            ROS_WARN_THROTTLE(
                1.0,
                "[POSE] Local pose is stale while requesting AUTO.LAND");
        }

        // 非降落阶段丢失 OFFBOARD 时冻结任务；AUTO.LAND 不应被抢回 OFFBOARD。
        if (flight_phase != FlightPhase::LANDING &&
            current_state.mode != "OFFBOARD") {
            if (!recovering_offboard) {
                ROS_ERROR("[MODE] OFFBOARD lost; freezing mission progress");
                recovering_offboard = true;
            }
            setpoint.header.stamp = now;
            local_pos_pub.publish(setpoint);
            rate.sleep();
            continue;
        }
        if (recovering_offboard) {
            ROS_WARN("[MODE] OFFBOARD recovered; current phase timeout restarted");
            recovering_offboard = false;
            phase_start = now;
            target_reached = false;
        }

        const double current_x = current_pose.pose.position.x;
        const double current_y = current_pose.pose.position.y;
        const double current_z = current_pose.pose.position.z;
        const double current_yaw = yaw_from_pose(current_pose);

        // 起飞：保持 home 水平位置和航向，上升到安全任务高度。
        if (flight_phase == FlightPhase::TAKEOFF) {
            setpoint = make_setpoint(home_x, home_y, safe_z, home_yaw);
            const double z_error = std::abs(current_z - safe_z);
            const double xy_error =
                horizontal_error(current_x, current_y, home_x, home_y);
            ROS_INFO_THROTTLE(
                1.0, "[TAKEOFF] z=%.2f/%.2f, z_err=%.2f, xy_err=%.2f",
                current_z, safe_z, z_error, xy_error);

            if (z_error <= takeoff_tolerance &&
                xy_error <= horizontal_tolerance) {
                flight_phase = FlightPhase::INITIAL_HOVER;
                phase_start = now;
                ROS_INFO("[PHASE] INITIAL_HOVER started for %.1f s",
                         initial_hover_duration);
            } else if (now - phase_start > ros::Duration(phase_timeout)) {
                flight_phase = FlightPhase::LANDING;
                phase_start = now;
                landing_x = current_x;
                landing_y = current_y;
                landing_hold_z = current_z;
                landing_yaw = current_yaw;
                auto_land_confirmed = false;
                last_land_request = ros::Time(0);
                ROS_ERROR(
                    "[TIMEOUT] TAKEOFF timed out; requesting AUTO.LAND in place");
            }
        } else if (flight_phase == FlightPhase::INITIAL_HOVER) {
            // 初始悬停：在任务高度稳定指定时间后开始执行航点。
            setpoint = make_setpoint(home_x, home_y, safe_z, home_yaw);
            const double elapsed = (now - phase_start).toSec();
            ROS_INFO_THROTTLE(1.0, "[HOVER] %.1f/%.1f s",
                              elapsed, initial_hover_duration);

            if (elapsed >= initial_hover_duration) {
                flight_phase = FlightPhase::WAYPOINTS;
                phase_start = now;
                target_reached = false;
                ROS_INFO("[PHASE] WAYPOINT_1 started");
            }
        } else if (flight_phase == FlightPhase::WAYPOINTS) {
            // 航点任务：依次飞向各航点，进入位置/航向容差后累计停留时间。
            const Waypoint& waypoint = waypoints.at(waypoint_index);
            setpoint = make_setpoint(
                home_x + waypoint.x, home_y + waypoint.y,
                home_z + waypoint.z, waypoint.yaw);
            const double pos_error = position_error(current_pose, setpoint);
            const double yaw_error = angle_error(current_yaw, waypoint.yaw);
            const bool inside =
                pos_error <= waypoint_tolerance && yaw_error <= yaw_tolerance;

            if (inside && !target_reached) {
                target_reached = true;
                reached_start = now;
                ROS_INFO("[WAYPOINT] WP%zu reached; hold timer started",
                         waypoint_index + 1);
            } else if (!inside && target_reached) {
                target_reached = false;
                ROS_WARN("[WAYPOINT] WP%zu left tolerance; hold timer reset",
                         waypoint_index + 1);
            }

            const double held = target_reached
                ? (now - reached_start).toSec() : 0.0;
            ROS_INFO_THROTTLE(
                1.0,
                "[WAYPOINT] WP%zu/%zu pos_err=%.2f m, yaw_err=%.1f deg, hold=%.1f/%.1f s",
                waypoint_index + 1, waypoints.size(), pos_error,
                yaw_error * 180.0 / kPi, held, waypoint.hold_sec);

            if (target_reached && held >= waypoint.hold_sec) {
                ROS_INFO("[WAYPOINT] WP%zu completed", waypoint_index + 1);
                ++waypoint_index;
                target_reached = false;
                phase_start = now;
                if (waypoint_index >= waypoints.size()) {
                    flight_phase = FlightPhase::RETURN_HOME;
                    ROS_INFO("[PHASE] RETURN_HOME started");
                } else {
                    ROS_INFO("[PHASE] WAYPOINT_%zu started", waypoint_index + 1);
                }
            } else if (now - phase_start > ros::Duration(waypoint_timeout)) {
                failsafe_setpoint = make_setpoint(
                    current_x, current_y, current_z, current_yaw);
                flight_phase = FlightPhase::FAILSAFE_HOVER;
                phase_start = now;
                target_reached = false;
                ROS_ERROR(
                    "[TIMEOUT] WP%zu timed out; hold current position, then return home",
                    waypoint_index + 1);
            }
        } else if (flight_phase == FlightPhase::FAILSAFE_HOVER) {
            // 航点超时保护：冻结当前位置短暂悬停，然后转入返航。
            setpoint = failsafe_setpoint;
            const double elapsed = (now - phase_start).toSec();
            ROS_WARN_THROTTLE(
                1.0, "[FAILSAFE_HOVER] %.1f/%.1f s before return",
                elapsed, failsafe_hover_duration);
            if (elapsed >= failsafe_hover_duration) {
                flight_phase = FlightPhase::RETURN_HOME;
                phase_start = now;
                target_reached = false;
                ROS_WARN("[PHASE] RETURN_HOME started after waypoint failure");
            }
        } else if (flight_phase == FlightPhase::RETURN_HOME) {
            // 返航：回到 home 上方安全高度；若超时则改为在当前位置垂直降落。
            setpoint = make_setpoint(home_x, home_y, safe_z, home_yaw);
            const double pos_error = position_error(current_pose, setpoint);
            const double yaw_error = angle_error(current_yaw, home_yaw);
            const bool inside =
                pos_error <= waypoint_tolerance && yaw_error <= yaw_tolerance;

            if (inside && !target_reached) {
                target_reached = true;
                reached_start = now;
                ROS_INFO("[RETURN_HOME] Home reached; hold timer started");
            } else if (!inside && target_reached) {
                target_reached = false;
                ROS_WARN("[RETURN_HOME] Left tolerance; hold timer reset");
            }

            const double held = target_reached
                ? (now - reached_start).toSec() : 0.0;
            ROS_INFO_THROTTLE(
                1.0, "[RETURN_HOME] pos_err=%.2f m, hold=%.1f/%.1f s",
                pos_error, held, return_hold_time);

            if (target_reached && held >= return_hold_time) {
                flight_phase = FlightPhase::LANDING;
                phase_start = now;
                landing_x = home_x;
                landing_y = home_y;
                landing_hold_z = current_z;
                landing_yaw = home_yaw;
                auto_land_confirmed = false;
                last_land_request = ros::Time(0);
                ROS_INFO("[PHASE] AUTO.LAND at home requested");
            } else if (now - phase_start > ros::Duration(return_timeout)) {
                flight_phase = FlightPhase::LANDING;
                phase_start = now;
                landing_x = current_x;
                landing_y = current_y;
                landing_hold_z = current_z;
                landing_yaw = current_yaw;
                auto_land_confirmed = false;
                last_land_request = ros::Time(0);
                ROS_ERROR(
                    "[TIMEOUT] RETURN_HOME timed out; requesting AUTO.LAND in place");
            }
        } else if (flight_phase == FlightPhase::LANDING) {
            // AUTO.LAND 接管前保持当前高度；接管后只监视模式和落地状态。
            const double height_above_ground = current_z - home_z;
            const double xy_error = horizontal_error(
                current_x, current_y, landing_x, landing_y);

            if (current_state.mode == "AUTO.LAND") {
                // 跟踪当前高度，使模式意外退出时的备用 OFFBOARD 目标不会爬升。
                landing_hold_z = current_z;
                if (!auto_land_confirmed) {
                    auto_land_confirmed = true;
                    phase_start = now;
                    ROS_INFO("[MODE] AUTO.LAND confirmed; PX4 owns descent");
                }
            } else {
                if (auto_land_confirmed) {
                    ROS_ERROR("[MODE] AUTO.LAND lost; requesting it again");
                    auto_land_confirmed = false;
                }
                if (now - last_land_request >
                    ros::Duration(land_mode_retry_interval)) {
                    if (set_mode_client.call(land_request) &&
                        land_request.response.mode_sent) {
                        ROS_INFO("[MODE] AUTO.LAND request sent");
                    } else {
                        ROS_ERROR(
                            "[MODE] AUTO.LAND request failed; holding altitude");
                    }
                    last_land_request = now;
                }
            }
            setpoint = make_setpoint(
                landing_x, landing_y, landing_hold_z, landing_yaw);

            const unsigned int landed_state = have_extended_state
                ? current_extended_state.landed_state
                : mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED;
            ROS_INFO_THROTTLE(
                1.0,
                "[LANDING] mode=%s, height=%.2f, xy_err=%.2f, landed_state=%u",
                current_state.mode.c_str(), height_above_ground, xy_error,
                landed_state);

            if (now - phase_start > ros::Duration(phase_timeout)) {
                ROS_ERROR_THROTTLE(
                    2.0,
                    "[TIMEOUT] AUTO.LAND not complete; check PX4 land detector "
                    "and COM_DISARM_LAND");
            }
        }

        // 每个控制周期都持续发布当前阶段的目标位姿。
        setpoint.header.stamp = now;
        local_pos_pub.publish(setpoint);
        rate.sleep();
    }

    return 0;
}
