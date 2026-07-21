/**
 * @file tower_mission_node.cpp
 * @brief Stage 1 fixed-height tower mission and PX4 OFFBOARD executor.
 *
 * The node intentionally has no Gazebo API dependency. Tower coordinates and
 * every ROS interface are provided by parameters. In preview mode it does not
 * advertise any MAVROS control topic. In control mode this single node owns
 * both configured setpoint topics, publishing position setpoints in flight and
 * raw-local vertical-velocity setpoints only during controlled landing.
 */

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <visualization_msgs/MarkerArray.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "astra_tower_mission/motion_limiter.h"
#include "astra_tower_mission/tower_route.h"
#include "offboard/landing_profile.h"

namespace astra_tower_mission {
namespace {

enum class MissionState {
  kPreviewOnly,
  kWaitInputs,
  kPrestream,
  kArmOffboard,
  kTakeoff,
  kInitialHover,
  kMission,
  kCloseLoop,
  kReturnHome,
  kPrelandHover,
  kLanding,
  kDone,
  kError,
};

const char* stateName(MissionState state) {
  switch (state) {
    case MissionState::kPreviewOnly: return "PREVIEW_ONLY";
    case MissionState::kWaitInputs: return "WAIT_INPUTS";
    case MissionState::kPrestream: return "PRESTREAM";
    case MissionState::kArmOffboard: return "ARM_OFFBOARD";
    case MissionState::kTakeoff: return "TAKEOFF";
    case MissionState::kInitialHover: return "INITIAL_HOVER";
    case MissionState::kMission: return "MISSION";
    case MissionState::kCloseLoop: return "CLOSE_LOOP";
    case MissionState::kReturnHome: return "RETURN_HOME";
    case MissionState::kPrelandHover: return "PRELAND_HOVER";
    case MissionState::kLanding: return "LANDING";
    case MissionState::kDone: return "DONE";
    case MissionState::kError: return "ERROR";
  }
  return "UNKNOWN";
}

double degreesToRadians(double degrees) {
  return degrees * kPi / 180.0;
}

double radiansToDegrees(double radians) {
  return radians * 180.0 / kPi;
}

double quaternionYaw(const geometry_msgs::Quaternion& quaternion) {
  return std::atan2(
      2.0 * (quaternion.w * quaternion.z +
             quaternion.x * quaternion.y),
      1.0 - 2.0 * (quaternion.y * quaternion.y +
                   quaternion.z * quaternion.z));
}

bool finiteQuaternion(const geometry_msgs::Quaternion& quaternion) {
  const double norm_squared =
      quaternion.x * quaternion.x + quaternion.y * quaternion.y +
      quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return std::isfinite(norm_squared) && norm_squared > 1e-6;
}

geometry_msgs::Quaternion yawQuaternion(double yaw) {
  geometry_msgs::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

double positionDistance(const geometry_msgs::PoseStamped& first,
                        const geometry_msgs::PoseStamped& second) {
  const double dx = first.pose.position.x - second.pose.position.x;
  const double dy = first.pose.position.y - second.pose.position.y;
  const double dz = first.pose.position.z - second.pose.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double horizontalDistance(const geometry_msgs::PoseStamped& first,
                          const geometry_msgs::PoseStamped& second) {
  const double dx = first.pose.position.x - second.pose.position.x;
  const double dy = first.pose.position.y - second.pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

double yawError(const geometry_msgs::PoseStamped& first,
                const geometry_msgs::PoseStamped& second) {
  return std::abs(normalizeAngle(quaternionYaw(first.pose.orientation) -
                                 quaternionYaw(second.pose.orientation)));
}

bool stringListParam(ros::NodeHandle* node, const std::string& name,
                     std::vector<std::string>* values) {
  XmlRpc::XmlRpcValue list;
  if (!node->getParam(name, list) ||
      list.getType() != XmlRpc::XmlRpcValue::TypeArray || list.size() == 0) {
    return false;
  }
  values->clear();
  for (int index = 0; index < list.size(); ++index) {
    if (list[index].getType() != XmlRpc::XmlRpcValue::TypeString) {
      return false;
    }
    values->push_back(static_cast<std::string>(list[index]));
  }
  return true;
}

struct NodeConfig {
  bool enable_control{false};
  std::string run_mode{"mission"};
  RouteConfig route;

  double maximum_speed{0.0};
  double maximum_acceleration{0.0};
  double maximum_yaw_rate{0.0};
  double position_tolerance{0.0};
  double yaw_tolerance{0.0};
  double waypoint_timeout{0.0};
  double mission_timeout{0.0};
  double overall_timeout{0.0};
  double maximum_home_distance{0.0};

  double takeoff_height{0.0};
  double takeoff_tolerance{0.0};
  double takeoff_hold_time{0.0};
  double takeoff_timeout{0.0};
  double initial_hover_duration{0.0};
  double initial_hover_timeout{0.0};
  double return_height{0.0};
  double return_hold_time{0.0};
  double return_timeout{0.0};
  double preland_hover_duration{0.0};
  double preland_hover_timeout{0.0};

  double landing_cruise_speed{0.0};
  double landing_touchdown_speed{0.0};
  double landing_slowdown_height{0.0};
  double landing_flare_height{0.0};
  double landing_contact_height{0.0};
  double landing_contact_velocity_max{0.0};
  double landing_contact_confirm_time{0.0};
  double landing_contact_descent_speed{0.0};
  double landing_ground_confirm_time{0.0};
  double landing_timeout{0.0};
  double disarm_retry_interval{0.0};
  double disarm_timeout{0.0};

  double loop_rate{0.0};
  double prestream_duration{0.0};
  double preflight_timeout{0.0};
  double input_timeout{0.0};
  double service_retry_interval{0.0};
  double arm_offboard_timeout{0.0};
  double authority_check_interval{0.0};
  double mode_recovery_timeout{0.0};
  double expected_home_x{0.0};
  double expected_home_y{0.0};
  double home_origin_tolerance{0.0};
  int maximum_actual_path_points{0};
  double report_period{0.0};
  std::string report_file;

  std::string state_topic;
  std::string extended_state_topic;
  std::string pose_topic;
  std::string velocity_topic;
  std::string position_setpoint_topic;
  std::string raw_setpoint_topic;
  std::string arming_service;
  std::string set_mode_service;
  std::vector<std::string> monitored_control_topics;

  std::string route_preview_topic;
  std::string waypoint_poses_topic;
  std::string marker_topic;
  std::string current_target_topic;
  std::string actual_path_topic;
  std::string mission_state_topic;
  std::string progress_topic;
  std::string waypoint_index_topic;
  std::string position_error_topic;
  std::string yaw_error_topic;
  std::string hold_time_topic;
  std::string final_result_topic;
  std::string report_file_topic;
};

}  // namespace

class TowerMissionNode {
 public:
  TowerMissionNode() : nh_(), pnh_("~") {
    if (!loadConfig()) {
      initialized_ = false;
      return;
    }
    setupTelemetryPublishers();
    publishRoutePreview();
    printMissionConfiguration("CONFIG");

    if (!config_.enable_control) {
      transitionTo(MissionState::kPreviewOnly,
                   "route preview ready; MAVROS control is disabled");
      if (!waypoints_.empty()) {
        setTarget(waypoints_.front());
      }
      publishTelemetry(ros::Time::now());
      ROS_INFO("[PREVIEW] strict circular route with %zu inspection "
               "checkpoints; no MAVROS setpoint publisher or flight service "
               "was created",
               waypoints_.size());
      return;
    }

    bool use_sim_time = false;
    nh_.param("/use_sim_time", use_sim_time, false);
    if (!use_sim_time) {
      ROS_FATAL("[SAFETY] Stage 1 control is simulation-only; "
                "/use_sim_time must be true");
      initialized_ = false;
      return;
    }

    setupControlInterfaces();
    openReport();
    transitionTo(MissionState::kWaitInputs,
                 "waiting for finite, fresh MAVROS preflight inputs");
    timer_ = nh_.createTimer(ros::Duration(1.0 / config_.loop_rate),
                             &TowerMissionNode::timerCallback, this);
  }

  bool initialized() const { return initialized_; }

 private:
  bool loadConfig() {
    pnh_.param("enable_control", config_.enable_control, false);
    pnh_.param<std::string>("run_mode", config_.run_mode, "mission");
    pnh_.param<std::string>("tower/name", config_.route.tower_name, "");
    pnh_.param<std::string>("tower/frame_id", config_.route.frame_id, "");
    pnh_.param("tower/center/x", config_.route.center_x,
               std::numeric_limits<double>::quiet_NaN());
    pnh_.param("tower/center/y", config_.route.center_y,
               std::numeric_limits<double>::quiet_NaN());
    pnh_.param("tower/collision_radius", config_.route.tower_collision_radius,
               std::numeric_limits<double>::quiet_NaN());

    double start_angle_deg = 0.0;
    double camera_yaw_offset_deg = 0.0;
    std::string direction_text;
    pnh_.param("mission/radius", config_.route.radius, 0.0);
    pnh_.param("mission/height", config_.route.height, 0.0);
    pnh_.param("mission/waypoint_count", config_.route.waypoint_count, 0);
    pnh_.param("mission/start_angle_deg", start_angle_deg, 0.0);
    pnh_.param<std::string>("mission/direction", direction_text, "");
    pnh_.param("mission/camera_yaw_offset_deg", camera_yaw_offset_deg, 0.0);
    pnh_.param("mission/minimum_safety_distance",
               config_.route.minimum_safety_distance, 0.0);
    pnh_.param("mission/minimum_height", config_.route.minimum_height, 0.0);
    pnh_.param("mission/maximum_height", config_.route.maximum_height, 0.0);
    config_.route.start_angle_rad = degreesToRadians(start_angle_deg);
    config_.route.camera_yaw_offset_rad =
        degreesToRadians(camera_yaw_offset_deg);
    if (!parseDirection(direction_text, &config_.route.direction)) {
      ROS_FATAL("[PARAM] mission/direction must be counter_clockwise/ccw "
                "or clockwise/cw, got '%s'", direction_text.c_str());
      return false;
    }

    pnh_.param("mission/maximum_speed", config_.maximum_speed, 0.0);
    pnh_.param("mission/maximum_acceleration", config_.maximum_acceleration,
               0.0);
    double maximum_yaw_rate_deg = 0.0;
    double yaw_tolerance_deg = 0.0;
    pnh_.param("mission/maximum_yaw_rate_deg_s", maximum_yaw_rate_deg, 0.0);
    pnh_.param("mission/position_tolerance", config_.position_tolerance, 0.0);
    pnh_.param("mission/yaw_tolerance_deg", yaw_tolerance_deg, 0.0);
    pnh_.param("mission/waypoint_timeout", config_.waypoint_timeout, 0.0);
    pnh_.param("mission/mission_timeout", config_.mission_timeout, 0.0);
    pnh_.param("mission/overall_timeout", config_.overall_timeout, 0.0);
    pnh_.param("mission/maximum_home_distance",
               config_.maximum_home_distance, 0.0);
    config_.maximum_yaw_rate = degreesToRadians(maximum_yaw_rate_deg);
    config_.yaw_tolerance = degreesToRadians(yaw_tolerance_deg);

    pnh_.param("flight/takeoff_height", config_.takeoff_height, 0.0);
    pnh_.param("flight/takeoff_tolerance", config_.takeoff_tolerance, 0.0);
    pnh_.param("flight/takeoff_hold_time", config_.takeoff_hold_time, 0.0);
    pnh_.param("flight/takeoff_timeout", config_.takeoff_timeout, 0.0);
    pnh_.param("flight/initial_hover_duration",
               config_.initial_hover_duration, 0.0);
    pnh_.param("flight/initial_hover_timeout",
               config_.initial_hover_timeout, 0.0);
    pnh_.param("flight/return_height", config_.return_height, 0.0);
    pnh_.param("flight/return_hold_time", config_.return_hold_time, 0.0);
    pnh_.param("flight/return_timeout", config_.return_timeout, 0.0);
    pnh_.param("flight/preland_hover_duration",
               config_.preland_hover_duration, 0.0);
    pnh_.param("flight/preland_hover_timeout",
               config_.preland_hover_timeout, 0.0);

    pnh_.param("landing/cruise_speed", config_.landing_cruise_speed, 0.0);
    pnh_.param("landing/touchdown_speed", config_.landing_touchdown_speed, 0.0);
    pnh_.param("landing/slowdown_height", config_.landing_slowdown_height,
               0.0);
    pnh_.param("landing/flare_height", config_.landing_flare_height, 0.0);
    pnh_.param("landing/contact_height", config_.landing_contact_height, 0.0);
    pnh_.param("landing/contact_velocity_max",
               config_.landing_contact_velocity_max, 0.0);
    pnh_.param("landing/contact_confirm_time",
               config_.landing_contact_confirm_time, 0.0);
    pnh_.param("landing/contact_descent_speed",
               config_.landing_contact_descent_speed, 0.0);
    pnh_.param("landing/ground_confirm_time",
               config_.landing_ground_confirm_time, 0.0);
    pnh_.param("landing/timeout", config_.landing_timeout, 0.0);
    pnh_.param("landing/disarm_retry_interval",
               config_.disarm_retry_interval, 0.0);
    pnh_.param("landing/disarm_timeout", config_.disarm_timeout, 0.0);

    pnh_.param("safety/loop_rate", config_.loop_rate, 0.0);
    pnh_.param("safety/prestream_duration", config_.prestream_duration, 0.0);
    pnh_.param("safety/preflight_timeout", config_.preflight_timeout, 0.0);
    pnh_.param("safety/input_timeout", config_.input_timeout, 0.0);
    pnh_.param("safety/service_retry_interval",
               config_.service_retry_interval, 0.0);
    pnh_.param("safety/arm_offboard_timeout",
               config_.arm_offboard_timeout, 0.0);
    pnh_.param("safety/authority_check_interval",
               config_.authority_check_interval, 0.0);
    pnh_.param("safety/mode_recovery_timeout",
               config_.mode_recovery_timeout, 0.0);
    pnh_.param("safety/expected_home_x", config_.expected_home_x, 0.0);
    pnh_.param("safety/expected_home_y", config_.expected_home_y, 0.0);
    pnh_.param("safety/home_origin_tolerance",
               config_.home_origin_tolerance, 0.0);
    pnh_.param("record/maximum_actual_path_points",
               config_.maximum_actual_path_points, 0);
    pnh_.param("record/report_period", config_.report_period, 0.0);
    pnh_.param<std::string>("record/report_file", config_.report_file, "");

    pnh_.param<std::string>("topics/mavros_state", config_.state_topic, "");
    pnh_.param<std::string>("topics/mavros_extended_state",
                            config_.extended_state_topic, "");
    pnh_.param<std::string>("topics/mavros_pose", config_.pose_topic, "");
    pnh_.param<std::string>("topics/mavros_velocity",
                            config_.velocity_topic, "");
    pnh_.param<std::string>("topics/setpoint_position",
                            config_.position_setpoint_topic, "");
    pnh_.param<std::string>("topics/setpoint_raw_local",
                            config_.raw_setpoint_topic, "");
    pnh_.param<std::string>("services/arming", config_.arming_service, "");
    pnh_.param<std::string>("services/set_mode", config_.set_mode_service, "");
    if (!stringListParam(&pnh_, "topics/monitored_control",
                         &config_.monitored_control_topics)) {
      ROS_FATAL("[PARAM] topics/monitored_control must be a non-empty "
                "string list");
      return false;
    }

    pnh_.param<std::string>("outputs/route_preview",
                            config_.route_preview_topic, "");
    pnh_.param<std::string>("outputs/waypoint_poses",
                            config_.waypoint_poses_topic, "");
    pnh_.param<std::string>("outputs/markers", config_.marker_topic, "");
    pnh_.param<std::string>("outputs/current_target",
                            config_.current_target_topic, "");
    pnh_.param<std::string>("outputs/actual_path",
                            config_.actual_path_topic, "");
    pnh_.param<std::string>("outputs/state", config_.mission_state_topic, "");
    pnh_.param<std::string>("outputs/progress", config_.progress_topic, "");
    pnh_.param<std::string>("outputs/waypoint_index",
                            config_.waypoint_index_topic, "");
    pnh_.param<std::string>("outputs/position_error",
                            config_.position_error_topic, "");
    pnh_.param<std::string>("outputs/yaw_error",
                            config_.yaw_error_topic, "");
    pnh_.param<std::string>("outputs/hold_time",
                            config_.hold_time_topic, "");
    pnh_.param<std::string>("outputs/final_result",
                            config_.final_result_topic, "");
    pnh_.param<std::string>("outputs/report_file",
                            config_.report_file_topic, "");

    std::string route_reason;
    if (!validateRouteConfig(config_.route, &route_reason)) {
      ROS_FATAL("[PARAM] Invalid tower route: %s", route_reason.c_str());
      return false;
    }
    waypoints_ = generateTowerWaypoints(config_.route);
    if (waypoints_.empty()) {
      ROS_FATAL("[PARAM] Route generation returned no waypoint");
      return false;
    }
    return validateNodeConfig();
  }

  bool validateNodeConfig() const {
    const bool run_mode_valid =
        config_.run_mode == "mission" || config_.run_mode == "hover_only";
    const double positive_values[] = {
        config_.maximum_speed, config_.maximum_acceleration,
        config_.maximum_yaw_rate, config_.position_tolerance,
        config_.yaw_tolerance, config_.waypoint_timeout,
        config_.mission_timeout, config_.overall_timeout,
        config_.maximum_home_distance, config_.takeoff_height,
        config_.takeoff_tolerance, config_.takeoff_timeout,
        config_.initial_hover_timeout, config_.return_height,
        config_.return_timeout, config_.preland_hover_timeout,
        config_.landing_cruise_speed, config_.landing_touchdown_speed,
        config_.landing_slowdown_height, config_.landing_timeout,
        config_.disarm_retry_interval, config_.disarm_timeout,
        config_.landing_contact_velocity_max,
        config_.landing_contact_confirm_time,
        config_.landing_contact_descent_speed,
        config_.landing_ground_confirm_time,
        config_.loop_rate, config_.prestream_duration,
        config_.preflight_timeout, config_.input_timeout,
        config_.service_retry_interval, config_.arm_offboard_timeout,
        config_.authority_check_interval, config_.mode_recovery_timeout,
        config_.home_origin_tolerance, config_.report_period};
    if (!run_mode_valid) {
      ROS_FATAL("[PARAM] run_mode must be mission or hover_only");
      return false;
    }
    for (double value : positive_values) {
      if (!std::isfinite(value) || value <= 0.0) {
        ROS_FATAL("[PARAM] Positive finite flight/safety parameter required");
        return false;
      }
    }
    const double nonnegative_values[] = {
        config_.takeoff_hold_time, config_.initial_hover_duration,
        config_.return_hold_time,
        config_.preland_hover_duration, config_.landing_flare_height,
        config_.landing_contact_height};
    for (double value : nonnegative_values) {
      if (!std::isfinite(value) || value < 0.0) {
        ROS_FATAL("[PARAM] Non-negative finite duration/height required");
        return false;
      }
    }
    if (config_.landing_cruise_speed < config_.landing_touchdown_speed ||
        config_.landing_slowdown_height <= config_.landing_flare_height ||
        config_.landing_contact_descent_speed <
            config_.landing_touchdown_speed ||
        config_.landing_contact_descent_speed > config_.landing_cruise_speed ||
        config_.maximum_actual_path_points < 100 ||
        config_.loop_rate < 2.0 ||
        config_.waypoint_timeout > config_.mission_timeout ||
        config_.mission_timeout > config_.overall_timeout) {
      ROS_FATAL("[PARAM] Landing profile or path recording relation invalid");
      return false;
    }
    if (config_.maximum_speed / config_.route.radius >
        config_.maximum_yaw_rate) {
      ROS_FATAL("[PARAM] maximum_yaw_rate_deg_s is too low to keep the "
                "camera facing the tower at maximum_speed");
      return false;
    }
    if (!std::isfinite(config_.expected_home_x) ||
        !std::isfinite(config_.expected_home_y)) {
      ROS_FATAL("[PARAM] Expected home coordinates must be finite");
      return false;
    }
    const std::string required_names[] = {
        config_.state_topic, config_.extended_state_topic,
        config_.pose_topic, config_.velocity_topic,
        config_.position_setpoint_topic, config_.raw_setpoint_topic,
        config_.arming_service, config_.set_mode_service,
        config_.route_preview_topic, config_.waypoint_poses_topic,
        config_.marker_topic, config_.current_target_topic,
        config_.actual_path_topic, config_.mission_state_topic,
        config_.progress_topic, config_.waypoint_index_topic,
        config_.position_error_topic, config_.yaw_error_topic,
        config_.hold_time_topic, config_.final_result_topic,
        config_.report_file_topic};
    for (const auto& name : required_names) {
      if (name.empty()) {
        ROS_FATAL("[PARAM] All configured Topic/service names must be non-empty");
        return false;
      }
    }
    return true;
  }

  void setupTelemetryPublishers() {
    route_preview_pub_ =
        pnh_.advertise<nav_msgs::Path>(config_.route_preview_topic, 1, true);
    waypoint_poses_pub_ =
        pnh_.advertise<geometry_msgs::PoseArray>(config_.waypoint_poses_topic,
                                                 1, true);
    marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>(
        config_.marker_topic, 1, true);
    current_target_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>(
        config_.current_target_topic, 1, true);
    actual_path_pub_ =
        pnh_.advertise<nav_msgs::Path>(config_.actual_path_topic, 1, true);
    state_pub_ =
        pnh_.advertise<std_msgs::String>(config_.mission_state_topic, 1, true);
    progress_pub_ =
        pnh_.advertise<std_msgs::Float64>(config_.progress_topic, 10);
    waypoint_index_pub_ =
        pnh_.advertise<std_msgs::UInt32>(config_.waypoint_index_topic, 10);
    position_error_pub_ =
        pnh_.advertise<std_msgs::Float64>(config_.position_error_topic, 10);
    yaw_error_pub_ =
        pnh_.advertise<std_msgs::Float64>(config_.yaw_error_topic, 10);
    hold_time_pub_ =
        pnh_.advertise<std_msgs::Float64>(config_.hold_time_topic, 10);
    final_result_pub_ =
        pnh_.advertise<std_msgs::String>(config_.final_result_topic, 1, true);
    report_file_pub_ =
        pnh_.advertise<std_msgs::String>(config_.report_file_topic, 1, true);
  }

  void setupControlInterfaces() {
    state_sub_ = nh_.subscribe(config_.state_topic, 20,
                               &TowerMissionNode::stateCallback, this);
    extended_state_sub_ = nh_.subscribe(
        config_.extended_state_topic, 20,
        &TowerMissionNode::extendedStateCallback, this);
    pose_sub_ = nh_.subscribe(config_.pose_topic, 50,
                              &TowerMissionNode::poseCallback, this);
    velocity_sub_ = nh_.subscribe(config_.velocity_topic, 50,
                                  &TowerMissionNode::velocityCallback, this);
    position_setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        config_.position_setpoint_topic, 20);
    raw_setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
        config_.raw_setpoint_topic, 20);
    arming_client_ =
        nh_.serviceClient<mavros_msgs::CommandBool>(config_.arming_service);
    set_mode_client_ =
        nh_.serviceClient<mavros_msgs::SetMode>(config_.set_mode_service);
  }

  void openReport() {
    if (config_.report_file.empty()) {
      return;
    }
    report_.open(config_.report_file, std::ios::out | std::ios::trunc);
    if (!report_.is_open()) {
      ROS_ERROR("[RECORD] Cannot open report file: %s",
                config_.report_file.c_str());
      return;
    }
    report_ << "sim_time,state,waypoint_index,completed_legs,total_legs,"
               "target_x,target_y,target_z,target_yaw_deg,actual_x,actual_y,"
               "actual_z,actual_yaw_deg,position_error_m,yaw_error_deg,"
               "hold_time_s,reference_tracking_error_m,armed,mode\n";
    report_.flush();
    std_msgs::String file_message;
    file_message.data = config_.report_file;
    report_file_pub_.publish(file_message);
    ROS_INFO("[RECORD] CSV evidence: %s", config_.report_file.c_str());
  }

  void publishRoutePreview() {
    const ros::Time stamp = ros::Time::now();
    nav_msgs::Path path;
    path.header.frame_id = config_.route.frame_id;
    path.header.stamp = stamp;
    geometry_msgs::PoseArray poses;
    poses.header = path.header;
    for (const auto& waypoint : waypoints_) {
      poses.poses.push_back(waypointPose(waypoint, stamp).pose);
    }
    // The checkpoints remain discrete, but the commanded inspection route is
    // a circle. Publish a dense circle here so preview does not suggest that
    // the executor will fly the old polygonal chords.
    const int preview_segment_count = std::max(180, config_.route.waypoint_count);
    for (int index = 0; index <= preview_segment_count; ++index) {
      const double progress =
          2.0 * kPi * index / static_cast<double>(preview_segment_count);
      path.poses.push_back(waypointPose(
          towerWaypointAtProgress(config_.route, progress), stamp));
    }
    route_preview_pub_.publish(path);
    waypoint_poses_pub_.publish(poses);

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker tower;
    tower.header = path.header;
    tower.ns = "tower_collision_envelope";
    tower.id = 0;
    tower.type = visualization_msgs::Marker::CYLINDER;
    tower.action = visualization_msgs::Marker::ADD;
    tower.pose.position.x = config_.route.center_x;
    tower.pose.position.y = config_.route.center_y;
    tower.pose.position.z = config_.route.height * 0.5;
    tower.pose.orientation.w = 1.0;
    tower.scale.x = 2.0 * config_.route.tower_collision_radius;
    tower.scale.y = 2.0 * config_.route.tower_collision_radius;
    tower.scale.z = config_.route.height;
    tower.color.r = 0.9F;
    tower.color.g = 0.1F;
    tower.color.b = 0.1F;
    tower.color.a = 0.35F;
    markers.markers.push_back(tower);

    visualization_msgs::Marker ring;
    ring.header = path.header;
    ring.ns = "inspection_ring";
    ring.id = 1;
    ring.type = visualization_msgs::Marker::LINE_STRIP;
    ring.action = visualization_msgs::Marker::ADD;
    ring.pose.orientation.w = 1.0;
    ring.scale.x = 0.15;
    ring.color.r = 0.1F;
    ring.color.g = 0.9F;
    ring.color.b = 0.2F;
    ring.color.a = 1.0F;
    for (const auto& pose : path.poses) {
      ring.points.push_back(pose.pose.position);
    }
    markers.markers.push_back(ring);
    marker_pub_.publish(markers);
  }

  geometry_msgs::PoseStamped waypointPose(const TowerWaypoint& waypoint,
                                          const ros::Time& stamp) const {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = config_.route.frame_id;
    pose.header.stamp = stamp;
    pose.pose.position.x = waypoint.x;
    pose.pose.position.y = waypoint.y;
    pose.pose.position.z = waypoint.z;
    pose.pose.orientation = yawQuaternion(waypoint.yaw);
    return pose;
  }

  geometry_msgs::PoseStamped makePose(double x, double y, double z,
                                      double yaw,
                                      const ros::Time& stamp) const {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = config_.route.frame_id;
    pose.header.stamp = stamp;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation = yawQuaternion(yaw);
    return pose;
  }

  void stateCallback(const mavros_msgs::State::ConstPtr& message) {
    current_state_ = *message;
    have_state_ = true;
    last_state_time_ = ros::Time::now();
  }

  void extendedStateCallback(
      const mavros_msgs::ExtendedState::ConstPtr& message) {
    current_extended_state_ = *message;
    have_extended_state_ = true;
    last_extended_state_time_ = ros::Time::now();
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    if (message->header.frame_id != config_.route.frame_id) {
      ROS_ERROR_THROTTLE(2.0,
                         "[POSE] frame mismatch: expected=%s actual=%s",
                         config_.route.frame_id.c_str(),
                         message->header.frame_id.c_str());
      return;
    }
    const auto& position = message->pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) ||
        !finiteQuaternion(message->pose.orientation)) {
      ROS_ERROR_THROTTLE(2.0, "[POSE] rejected non-finite/invalid pose");
      return;
    }
    current_pose_ = *message;
    have_pose_ = true;
    last_pose_time_ = ros::Time::now();

    if (actual_path_.poses.empty() ||
        positionDistance(actual_path_.poses.back(), current_pose_) >= 0.05) {
      actual_path_.header.frame_id = config_.route.frame_id;
      actual_path_.header.stamp = last_pose_time_;
      geometry_msgs::PoseStamped sample = current_pose_;
      sample.header.frame_id = config_.route.frame_id;
      actual_path_.poses.push_back(sample);
      if (static_cast<int>(actual_path_.poses.size()) >
          config_.maximum_actual_path_points) {
        actual_path_.poses.erase(actual_path_.poses.begin());
      }
      actual_path_pub_.publish(actual_path_);
    }
  }

  void velocityCallback(
      const geometry_msgs::TwistStamped::ConstPtr& message) {
    const auto& linear = message->twist.linear;
    if (!std::isfinite(linear.x) || !std::isfinite(linear.y) ||
        !std::isfinite(linear.z)) {
      ROS_ERROR_THROTTLE(2.0, "[VELOCITY] rejected non-finite velocity");
      return;
    }
    current_velocity_ = *message;
    have_velocity_ = true;
    last_velocity_time_ = ros::Time::now();
  }

  bool inputsFresh(const ros::Time& now, std::string* reason) const {
    const ros::Duration timeout(config_.input_timeout);
    if (!have_state_ || now - last_state_time_ > timeout) {
      *reason = "MAVROS state missing or stale";
      return false;
    }
    if (!current_state_.connected) {
      *reason = "FCU is not connected";
      return false;
    }
    if (!have_pose_ || now - last_pose_time_ > timeout) {
      *reason = "MAVROS local pose missing or stale";
      return false;
    }
    if (!current_pose_.header.stamp.isZero() &&
        now >= current_pose_.header.stamp &&
        now - current_pose_.header.stamp > timeout) {
      *reason = "MAVROS pose source stamp is stale";
      return false;
    }
    if (!have_velocity_ || now - last_velocity_time_ > timeout) {
      *reason = "MAVROS local velocity missing or stale";
      return false;
    }
    if (!have_extended_state_ || now - last_extended_state_time_ > timeout) {
      *reason = "MAVROS extended state missing or stale";
      return false;
    }
    reason->clear();
    return true;
  }

  bool hasControlConflict(std::string* detail) const {
    XmlRpc::XmlRpcValue arguments;
    XmlRpc::XmlRpcValue response;
    XmlRpc::XmlRpcValue payload;
    arguments[0] = ros::this_node::getName();
    if (!ros::master::execute("getSystemState", arguments, response, payload,
                              false) ||
        payload.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        payload.size() < 1) {
      *detail = "unable to query ROS master for control authority";
      return true;
    }

    std::vector<std::string> resolved_topics;
    for (const auto& topic : config_.monitored_control_topics) {
      resolved_topics.push_back(nh_.resolveName(topic));
    }
    const auto& publishers = payload[0];
    for (int entry_index = 0; entry_index < publishers.size(); ++entry_index) {
      if (publishers[entry_index].getType() !=
              XmlRpc::XmlRpcValue::TypeArray ||
          publishers[entry_index].size() < 2 ||
          publishers[entry_index][0].getType() !=
              XmlRpc::XmlRpcValue::TypeString ||
          publishers[entry_index][1].getType() !=
              XmlRpc::XmlRpcValue::TypeArray) {
        continue;
      }
      const std::string topic =
          static_cast<std::string>(publishers[entry_index][0]);
      if (std::find(resolved_topics.begin(), resolved_topics.end(), topic) ==
          resolved_topics.end()) {
        continue;
      }
      const auto& nodes = publishers[entry_index][1];
      for (int node_index = 0; node_index < nodes.size(); ++node_index) {
        if (nodes[node_index].getType() !=
            XmlRpc::XmlRpcValue::TypeString) {
          continue;
        }
        const std::string node = static_cast<std::string>(nodes[node_index]);
        if (node != ros::this_node::getName()) {
          *detail = "control Topic " + topic +
                    " already has foreign publisher " + node;
          return true;
        }
      }
    }
    detail->clear();
    return false;
  }

  bool controlAuthorityValid(const ros::Time& now, std::string* detail) {
    if (last_authority_check_time_.isZero() ||
        now < last_authority_check_time_ ||
        now - last_authority_check_time_ >=
            ros::Duration(config_.authority_check_interval)) {
      cached_authority_conflict_ = hasControlConflict(&cached_conflict_detail_);
      last_authority_check_time_ = now;
    }
    if (cached_authority_conflict_) {
      *detail = cached_conflict_detail_;
      return false;
    }
    detail->clear();
    return true;
  }

  bool fullPreflight(const ros::Time& now, std::string* reason) {
    if (!inputsFresh(now, reason)) {
      return false;
    }
    if (current_state_.armed) {
      *reason = "vehicle is already armed";
      return false;
    }
    if (current_extended_state_.landed_state !=
        mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
      *reason = "PX4 does not report ON_GROUND";
      return false;
    }
    if (!arming_client_.exists() || !set_mode_client_.exists()) {
      *reason = "MAVROS arming or set_mode service is unavailable";
      return false;
    }
    if (!controlAuthorityValid(now, reason)) {
      return false;
    }
    const double home_origin_error = std::hypot(
        current_pose_.pose.position.x - config_.expected_home_x,
        current_pose_.pose.position.y - config_.expected_home_y);
    if (home_origin_error > config_.home_origin_tolerance) {
      std::ostringstream stream;
      stream << "home/map origin mismatch " << home_origin_error
             << " m exceeds " << config_.home_origin_tolerance << " m";
      *reason = stream.str();
      return false;
    }
    const double maximum_route_distance =
        std::hypot(config_.route.center_x - current_pose_.pose.position.x,
                   config_.route.center_y - current_pose_.pose.position.y) +
        config_.route.radius;
    if (maximum_route_distance > config_.maximum_home_distance) {
      *reason = "continuous circle exceeds maximum_home_distance";
      return false;
    }
    reason->clear();
    return true;
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (now.isZero()) {
      return;
    }
    if (state_entered_time_.isZero()) {
      state_entered_time_ = now;
      node_started_time_ = now;
    }
    double dt = 1.0 / config_.loop_rate;
    if (!last_timer_time_.isZero() && now >= last_timer_time_) {
      dt = std::min(0.25, std::max(0.001, (now - last_timer_time_).toSec()));
    }
    last_timer_time_ = now;

    publishTelemetry(now);
    writeReport(now);
    if (state_ == MissionState::kDone || state_ == MissionState::kError) {
      return;
    }

    if (state_ == MissionState::kWaitInputs) {
      handleWaitInputs(now);
      return;
    }

    std::string health_reason;
    if (!inputsFresh(now, &health_reason)) {
      terminalError("flight input failure: " + health_reason);
      return;
    }
    if (!controlAuthorityValid(now, &health_reason)) {
      terminalError("control authority lost: " + health_reason +
                    "; PX4 failsafe must take over");
      return;
    }

    const bool airborne_phase =
        state_ == MissionState::kTakeoff ||
        state_ == MissionState::kInitialHover ||
        state_ == MissionState::kMission ||
        state_ == MissionState::kCloseLoop ||
        state_ == MissionState::kReturnHome ||
        state_ == MissionState::kPrelandHover;
    if (airborne_phase && !current_state_.armed) {
      terminalError("vehicle disarmed unexpectedly before landing");
      return;
    }
    if (airborne_phase && current_state_.mode != "OFFBOARD") {
      handleOffboardLoss(now);
      publishPositionReference(now);
      return;
    }
    mode_loss_start_ = ros::Time(0);

    if (!armed_time_.isZero() &&
        now - armed_time_ > ros::Duration(config_.overall_timeout) &&
        state_ != MissionState::kReturnHome &&
        state_ != MissionState::kPrelandHover &&
        state_ != MissionState::kLanding) {
      failAndReturn("overall task timeout");
    }

    switch (state_) {
      case MissionState::kPrestream:
        handlePrestream(now);
        break;
      case MissionState::kArmOffboard:
        handleArmOffboard(now);
        break;
      case MissionState::kTakeoff:
        handleTakeoff(now, dt);
        break;
      case MissionState::kInitialHover:
        handleInitialHover(now, dt);
        break;
      case MissionState::kMission:
        handleMission(now, dt);
        break;
      case MissionState::kCloseLoop:
        handleMission(now, dt);
        break;
      case MissionState::kReturnHome:
        handleReturnHome(now, dt);
        break;
      case MissionState::kPrelandHover:
        handlePrelandHover(now, dt);
        break;
      case MissionState::kLanding:
        handleLanding(now);
        break;
      case MissionState::kPreviewOnly:
      case MissionState::kWaitInputs:
      case MissionState::kDone:
      case MissionState::kError:
        break;
    }
  }

  void handleWaitInputs(const ros::Time& now) {
    std::string reason;
    if (!fullPreflight(now, &reason)) {
      ROS_INFO_THROTTLE(2.0, "[PREFLIGHT] blocked: %s", reason.c_str());
      if (now - state_entered_time_ >
          ros::Duration(config_.preflight_timeout)) {
        terminalError("preflight timeout: " + reason);
      }
      return;
    }
    home_pose_ = current_pose_;
    home_pose_.header.frame_id = config_.route.frame_id;
    home_yaw_ = quaternionYaw(home_pose_.pose.orientation);
    landing_x_ = home_pose_.pose.position.x;
    landing_y_ = home_pose_.pose.position.y;
    reference_.x = home_pose_.pose.position.x;
    reference_.y = home_pose_.pose.position.y;
    reference_.z = home_pose_.pose.position.z;
    reference_.yaw = home_yaw_;
    setTarget(home_pose_);
    printMissionConfiguration("PREFLIGHT_PASS");
    ROS_INFO("[HOME] frame=%s x=%.3f y=%.3f z=%.3f yaw=%.1f deg",
             config_.route.frame_id.c_str(), home_pose_.pose.position.x,
             home_pose_.pose.position.y, home_pose_.pose.position.z,
             radiansToDegrees(home_yaw_));
    transitionTo(MissionState::kPrestream, "full preflight passed");
  }

  void handlePrestream(const ros::Time& now) {
    setTarget(home_pose_);
    publishPositionReference(now);
    if (now - state_entered_time_ >=
        ros::Duration(config_.prestream_duration)) {
      transitionTo(MissionState::kArmOffboard,
                   "OFFBOARD setpoint prestream complete");
    }
  }

  void handleArmOffboard(const ros::Time& now) {
    setTarget(home_pose_);
    publishPositionReference(now);
    if (now - state_entered_time_ >
        ros::Duration(config_.arm_offboard_timeout)) {
      terminalError("OFFBOARD/arming timeout before takeoff");
      return;
    }
    if (current_state_.mode != "OFFBOARD") {
      requestMode(now);
      return;
    }
    if (!current_state_.armed) {
      requestArm(now);
      return;
    }
    armed_time_ = now;
    const geometry_msgs::PoseStamped takeoff = makePose(
        home_pose_.pose.position.x, home_pose_.pose.position.y,
        home_pose_.pose.position.z + config_.takeoff_height, home_yaw_, now);
    setTarget(takeoff);
    transitionTo(MissionState::kTakeoff, "OFFBOARD and armed confirmed");
  }

  void handleTakeoff(const ros::Time& now, double dt) {
    advanceReference(dt);
    publishPositionReference(now);
    if (arrivalHeld(now, config_.takeoff_tolerance, config_.yaw_tolerance,
                    config_.takeoff_hold_time)) {
      transitionTo(MissionState::kInitialHover,
                   "takeoff target reached and held");
      return;
    }
    if (now - state_entered_time_ > ros::Duration(config_.takeoff_timeout)) {
      failAndLandInPlace("takeoff timeout");
    }
  }

  void handleInitialHover(const ros::Time& now, double dt) {
    advanceReference(dt);
    publishPositionReference(now);
    if (arrivalHeld(now, config_.position_tolerance, config_.yaw_tolerance,
                    config_.initial_hover_duration)) {
      if (config_.run_mode == "hover_only") {
        startReturn("hover-only verification completed");
      } else {
        current_waypoint_index_ = 0;
        completed_legs_ = 0;
        mission_started_time_ = now;
        setIngressTarget(now);
        transitionTo(MissionState::kMission,
                     "initial hover stable; forward-facing ingress started");
      }
      return;
    }
    if (now - state_entered_time_ >
        ros::Duration(config_.initial_hover_timeout)) {
      failAndLandInPlace("initial hover timeout");
    }
  }

  void handleMission(const ros::Time& now, double dt) {
    if (!mission_started_time_.isZero() &&
        now - mission_started_time_ > ros::Duration(config_.mission_timeout)) {
      failAndReturn("mission timeout before completing the circular orbit");
      return;
    }
    if (!orbit_started_) {
      updateIngressTargetYaw(now);
      advanceReference(dt);
      publishPositionReference(now);
      ROS_INFO_THROTTLE(
          1.0,
          "[INGRESS] target=CP1/%zu pos_err=%.2f m yaw_err=%.1f deg "
          "heading=%.1f deg",
          waypoints_.size(), currentPositionError(),
          radiansToDegrees(currentYawError()), radiansToDegrees(target_.yaw));
      const double reference_entry_error = std::hypot(
          reference_.x - waypoints_.front().x,
          reference_.y - waypoints_.front().y);
      if (currentPositionError() <= config_.position_tolerance &&
          reference_entry_error <= 1e-6) {
        circular_reference_ =
            initializeCircularMotionReference(config_.route);
        reference_ = circular_reference_.motion;
        orbit_started_ = true;
        setDynamicOrbitTarget(now);
        transitionTo(MissionState::kMission,
                     "circle entry reached; continuous strict arc started");
        ROS_INFO("[CHECKPOINT_ENTRY] CP1/%zu reached; no checkpoint dwell",
                 waypoints_.size());
      } else if (now - state_entered_time_ >
                 ros::Duration(config_.waypoint_timeout)) {
        failAndReturn("circle-entry timeout");
      }
      return;
    }

    circular_reference_ = stepCircularMotionReference(
        circular_reference_, config_.route, dt, config_.maximum_speed,
        config_.maximum_acceleration);
    reference_ = circular_reference_.motion;
    setDynamicOrbitTarget(now);
    recordTrackingErrors(now);
    publishPositionReference(now);

    const double checkpoint_step =
        2.0 * kPi / static_cast<double>(waypoints_.size());
    while (completed_legs_ < waypoints_.size() &&
           circular_reference_.angular_progress + 1e-12 >=
               (completed_legs_ + 1U) * checkpoint_step) {
      ++completed_legs_;
      current_waypoint_index_ = completed_legs_ % waypoints_.size();
      ROS_INFO("[CHECKPOINT_PASS] CP%zu/%zu passed at %.1f deg; continuing "
               "without dwell",
               current_waypoint_index_ + 1U, waypoints_.size(),
               radiansToDegrees(circular_reference_.angular_progress));
    }
    ROS_INFO_THROTTLE(
        1.0,
        "[ORBIT] progress=%.1f%% checkpoint=CP%zu/%zu speed=%.2f m/s "
        "tracking_err=%.2f m yaw_err=%.1f deg",
        100.0 * circular_reference_.angular_progress / (2.0 * kPi),
        current_waypoint_index_ + 1U, waypoints_.size(),
        circular_reference_.speed, currentPositionError(),
        radiansToDegrees(currentYawError()));
    if (circular_reference_.complete) {
      orbit_completed_ = true;
      ROS_INFO("[ORBIT_DONE] strict circle and %zu checkpoints complete; "
               "elapsed=%.2f s; checkpoints were pass-through",
               waypoints_.size(),
               (now - mission_started_time_).toSec());
      startReturn("full fixed-height circular orbit completed");
    }
  }

  void startReturn(const std::string& reason) {
    const ros::Time now = ros::Time::now();
    const geometry_msgs::PoseStamped return_pose = makePose(
        home_pose_.pose.position.x, home_pose_.pose.position.y,
        home_pose_.pose.position.z + config_.return_height, home_yaw_, now);
    setTarget(return_pose);
    transitionTo(MissionState::kReturnHome, reason);
  }

  void handleReturnHome(const ros::Time& now, double dt) {
    advanceReference(dt);
    publishPositionReference(now);
    ROS_INFO_THROTTLE(1.0,
                      "[RETURN_HOME] pos_err=%.2f m yaw_err=%.1f deg "
                      "hold=%.1f/%.1f s",
                      currentPositionError(),
                      radiansToDegrees(currentYawError()),
                      currentHoldDuration(now), config_.return_hold_time);
    if (arrivalHeld(now, config_.position_tolerance, config_.yaw_tolerance,
                    config_.return_hold_time)) {
      returned_home_ = true;
      startPrelandHover(home_pose_.pose.position.x,
                        home_pose_.pose.position.y,
                        "home-above reached and held");
      return;
    }
    if (now - state_entered_time_ > ros::Duration(config_.return_timeout)) {
      mission_failed_ = true;
      appendFailure("return-home timeout; landing in place");
      startPrelandHover(current_pose_.pose.position.x,
                        current_pose_.pose.position.y,
                        "return timeout fallback");
    }
  }

  void startPrelandHover(double x, double y, const std::string& reason) {
    landing_x_ = x;
    landing_y_ = y;
    const geometry_msgs::PoseStamped preland = makePose(
        landing_x_, landing_y_,
        home_pose_.pose.position.z + config_.return_height, home_yaw_,
        ros::Time::now());
    setTarget(preland);
    transitionTo(MissionState::kPrelandHover, reason);
  }

  void handlePrelandHover(const ros::Time& now, double dt) {
    advanceReference(dt);
    publishPositionReference(now);
    if (arrivalHeld(now, config_.position_tolerance, config_.yaw_tolerance,
                    config_.preland_hover_duration)) {
      transitionTo(MissionState::kLanding,
                   "pre-landing hover stable; controlled descent started");
      landing_started_time_ = now;
      return;
    }
    if (now - state_entered_time_ >
        ros::Duration(config_.preland_hover_timeout)) {
      mission_failed_ = true;
      appendFailure("pre-landing hover timeout");
      landing_x_ = current_pose_.pose.position.x;
      landing_y_ = current_pose_.pose.position.y;
      transitionTo(MissionState::kLanding,
                   "preland timeout; controlled descent in place");
      landing_started_time_ = now;
    }
  }

  void handleLanding(const ros::Time& now) {
    if (landing_started_time_.isZero()) {
      landing_started_time_ = now;
    }
    if (now - landing_started_time_ > ros::Duration(config_.landing_timeout)) {
      terminalError("controlled landing timeout; PX4 failsafe must take over");
      return;
    }
    if (current_state_.armed && current_state_.mode != "OFFBOARD") {
      terminalError("OFFBOARD lost during controlled landing; "
                    "PX4 failsafe must take over");
      return;
    }

    const double height_above_home_ground =
        current_pose_.pose.position.z - home_pose_.pose.position.z;
    const bool velocity_fresh =
        have_velocity_ &&
        now - last_velocity_time_ <= ros::Duration(config_.input_timeout);
    const bool contact_candidate =
        velocity_fresh &&
        height_above_home_ground <= config_.landing_contact_height &&
        current_velocity_.twist.linear.z <=
            config_.landing_contact_velocity_max;
    if (!landing_contact_assist_ && contact_candidate) {
      if (contact_candidate_since_.isZero()) {
        contact_candidate_since_ = now;
      } else if (now - contact_candidate_since_ >=
                 ros::Duration(config_.landing_contact_confirm_time)) {
        landing_contact_assist_ = true;
        ROS_INFO("[LANDING] contact-assist descent intent enabled");
      }
    } else if (!landing_contact_assist_) {
      contact_candidate_since_ = ros::Time(0);
    }

    const double profile_speed = offboard::landingDescentSpeed(
        height_above_home_ground, config_.landing_cruise_speed,
        config_.landing_touchdown_speed, config_.landing_slowdown_height,
        config_.landing_flare_height);
    const double descent_speed = landing_contact_assist_
        ? std::max(profile_speed, config_.landing_contact_descent_speed)
        : profile_speed;
    publishLandingSetpoint(now, descent_speed);

    const bool on_ground =
        current_extended_state_.landed_state ==
        mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
    if (on_ground) {
      if (ground_confirm_since_.isZero()) {
        ground_confirm_since_ = now;
        ROS_INFO("[LANDING] PX4 ON_GROUND confirmation started");
      }
      if (now - ground_confirm_since_ >=
          ros::Duration(config_.landing_ground_confirm_time)) {
        if (!current_state_.armed) {
          landed_and_disarmed_ = true;
          finishAfterLanding(now);
          return;
        }
        if (disarm_started_time_.isZero()) {
          disarm_started_time_ = now;
        }
        if (now - disarm_started_time_ >
            ros::Duration(config_.disarm_timeout)) {
          terminalError("PX4 remained armed after ON_GROUND disarm timeout");
          return;
        }
        requestDisarm(now);
      }
    } else {
      ground_confirm_since_ = ros::Time(0);
      disarm_started_time_ = ros::Time(0);
    }
    ROS_INFO_THROTTLE(
        1.0,
        "[LANDING] height=%.2f m vz_cmd=-%.2f m/s xy_err=%.2f m "
        "landed_state=%u armed=%s",
        height_above_home_ground, descent_speed,
        horizontalDistance(current_pose_,
                           makePose(landing_x_, landing_y_,
                                    current_pose_.pose.position.z, home_yaw_,
                                    now)),
        current_extended_state_.landed_state,
        current_state_.armed ? "true" : "false");
  }

  void finishAfterLanding(const ros::Time& now) {
    const double elapsed = armed_time_.isZero() ? 0.0 : (now - armed_time_).toSec();
    std::ostringstream result;
    result << std::fixed << std::setprecision(3)
           << (mission_failed_ ? "FAILED_SAFE_LANDING" : "SUCCESS")
           << "; tower=" << config_.route.tower_name
           << "; waypoints=" << waypoints_.size()
           << "; orbit_completed=" << (orbit_completed_ ? "true" : "false")
           << "; returned_home=" << (returned_home_ ? "true" : "false")
           << "; landed_and_disarmed=true"
           << "; elapsed_s=" << elapsed
           << "; max_reference_position_error_m="
           << maximum_reference_tracking_error_
           << "; max_yaw_error_deg=" << radiansToDegrees(maximum_yaw_error_);
    if (!failure_reason_.empty()) {
      result << "; reason=" << failure_reason_;
    }
    final_result_ = result.str();
    std_msgs::String message;
    message.data = final_result_;
    final_result_pub_.publish(message);
    ROS_INFO("[FINAL] %s", final_result_.c_str());
    transitionTo(mission_failed_ ? MissionState::kError : MissionState::kDone,
                 final_result_);
    if (report_.is_open()) {
      report_.flush();
    }
  }

  void handleOffboardLoss(const ros::Time& now) {
    if (mode_loss_start_.isZero()) {
      mode_loss_start_ = now;
      ROS_ERROR("[MODE] OFFBOARD lost in %s; holding reference and retrying",
                stateName(state_));
    }
    requestMode(now);
    if (now - mode_loss_start_ >
        ros::Duration(config_.mode_recovery_timeout)) {
      terminalError("OFFBOARD recovery timeout; PX4 failsafe must take over");
    }
  }

  void requestMode(const ros::Time& now) {
    if (!last_mode_request_.isZero() &&
        now - last_mode_request_ <
            ros::Duration(config_.service_retry_interval)) {
      return;
    }
    mavros_msgs::SetMode request;
    request.request.custom_mode = "OFFBOARD";
    if (set_mode_client_.call(request) && request.response.mode_sent) {
      ROS_INFO("[MODE] OFFBOARD request accepted");
    } else {
      ROS_WARN("[MODE] OFFBOARD request failed");
    }
    last_mode_request_ = now;
  }

  void requestArm(const ros::Time& now) {
    if (!last_arm_request_.isZero() &&
        now - last_arm_request_ <
            ros::Duration(config_.service_retry_interval)) {
      return;
    }
    mavros_msgs::CommandBool request;
    request.request.value = true;
    if (arming_client_.call(request) && request.response.success) {
      ROS_INFO("[ARM] arm request accepted");
    } else {
      ROS_WARN("[ARM] arm request failed");
    }
    last_arm_request_ = now;
  }

  void requestDisarm(const ros::Time& now) {
    if (!last_disarm_request_.isZero() &&
        now - last_disarm_request_ <
            ros::Duration(config_.disarm_retry_interval)) {
      return;
    }
    mavros_msgs::CommandBool request;
    request.request.value = false;
    if (arming_client_.call(request) && request.response.success) {
      ROS_INFO("[LANDING] normal disarm accepted after ON_GROUND");
    } else {
      ROS_WARN("[LANDING] disarm not accepted yet; touchdown stream continues");
    }
    last_disarm_request_ = now;
  }

  void failAndReturn(const std::string& reason) {
    mission_failed_ = true;
    appendFailure(reason);
    if (have_pose_ && current_state_.armed &&
        current_state_.mode == "OFFBOARD") {
      startReturn("failure recovery: " + reason);
    } else {
      terminalError(reason);
    }
  }

  void failAndLandInPlace(const std::string& reason) {
    mission_failed_ = true;
    appendFailure(reason);
    if (have_pose_ && current_state_.armed &&
        current_state_.mode == "OFFBOARD") {
      startPrelandHover(current_pose_.pose.position.x,
                        current_pose_.pose.position.y,
                        "failure landing: " + reason);
    } else {
      terminalError(reason);
    }
  }

  void appendFailure(const std::string& reason) {
    if (!failure_reason_.empty()) {
      failure_reason_ += " | ";
    }
    failure_reason_ += reason;
  }

  void terminalError(const std::string& reason) {
    if (state_ == MissionState::kError) {
      return;
    }
    mission_failed_ = true;
    appendFailure(reason);
    final_result_ = "ERROR; " + failure_reason_;
    std_msgs::String message;
    message.data = final_result_;
    final_result_pub_.publish(message);
    ROS_FATAL("[FINAL] %s", final_result_.c_str());
    transitionTo(MissionState::kError, reason);
    if (report_.is_open()) {
      report_.flush();
    }
  }

  void transitionTo(MissionState next, const std::string& reason) {
    const MissionState previous = state_;
    state_ = next;
    state_entered_time_ = ros::Time::now();
    arrival_since_ = ros::Time(0);
    std_msgs::String message;
    message.data = stateName(next);
    state_pub_.publish(message);
    ROS_INFO("[STATE] %s -> %s: %s", stateName(previous), stateName(next),
             reason.c_str());
  }

  void setTarget(const TowerWaypoint& waypoint) {
    setTarget(waypointPose(waypoint, ros::Time::now()));
  }

  void setTarget(const geometry_msgs::PoseStamped& pose) {
    target_pose_ = pose;
    target_pose_.header.frame_id = config_.route.frame_id;
    target_pose_.header.stamp = ros::Time::now();
    target_.x = target_pose_.pose.position.x;
    target_.y = target_pose_.pose.position.y;
    target_.z = target_pose_.pose.position.z;
    target_.yaw = quaternionYaw(target_pose_.pose.orientation);
    current_target_pub_.publish(target_pose_);
    arrival_since_ = ros::Time(0);
  }

  double ingressForwardYaw() const {
    const double from_x = have_pose_ ? current_pose_.pose.position.x
                                     : reference_.x;
    const double from_y = have_pose_ ? current_pose_.pose.position.y
                                     : reference_.y;
    const double dx = waypoints_.front().x - from_x;
    const double dy = waypoints_.front().y - from_y;
    if (std::hypot(dx, dy) > 1e-6) {
      return std::atan2(dy, dx);
    }
    if (std::hypot(reference_.vx, reference_.vy) > 1e-6) {
      return std::atan2(reference_.vy, reference_.vx);
    }
    return reference_.yaw;
  }

  void setIngressTarget(const ros::Time& now) {
    geometry_msgs::PoseStamped ingress = waypointPose(waypoints_.front(), now);
    ingress.pose.orientation = yawQuaternion(ingressForwardYaw());
    setTarget(ingress);
  }

  void updateIngressTargetYaw(const ros::Time& now) {
    target_.yaw = ingressForwardYaw();
    target_pose_.header.stamp = now;
    target_pose_.pose.orientation = yawQuaternion(target_.yaw);
    current_target_pub_.publish(target_pose_);
  }

  void setDynamicOrbitTarget(const ros::Time& now) {
    target_.x = reference_.x;
    target_.y = reference_.y;
    target_.z = reference_.z;
    target_.yaw = reference_.yaw;
    target_pose_ = makePose(target_.x, target_.y, target_.z, target_.yaw, now);
    current_target_pub_.publish(target_pose_);
  }

  void recordTrackingErrors(const ros::Time& now) {
    if (!have_pose_) {
      return;
    }
    const geometry_msgs::PoseStamped reference_pose = makePose(
        reference_.x, reference_.y, reference_.z, reference_.yaw, now);
    maximum_reference_tracking_error_ = std::max(
        maximum_reference_tracking_error_,
        positionDistance(current_pose_, reference_pose));
    maximum_yaw_error_ = std::max(maximum_yaw_error_, currentYawError());
  }

  void advanceReference(double dt) {
    reference_ = stepMotionReference(
        reference_, target_, dt, config_.maximum_speed,
        config_.maximum_acceleration, config_.maximum_yaw_rate);
    recordTrackingErrors(ros::Time::now());
  }

  void publishPositionReference(const ros::Time& now) {
    geometry_msgs::PoseStamped setpoint = makePose(
        reference_.x, reference_.y, reference_.z, reference_.yaw, now);
    position_setpoint_pub_.publish(setpoint);
  }

  void publishLandingSetpoint(const ros::Time& now, double descent_speed) {
    mavros_msgs::PositionTarget target;
    target.header.frame_id = config_.route.frame_id;
    target.header.stamp = now;
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target.type_mask =
        mavros_msgs::PositionTarget::IGNORE_PZ |
        mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    target.position.x = landing_x_;
    target.position.y = landing_y_;
    // MAVROS accepts ENU values. Negative z velocity means down in ENU.
    target.velocity.z = -descent_speed;
    target.yaw = home_yaw_;
    raw_setpoint_pub_.publish(target);
  }

  bool arrivalHeld(const ros::Time& now, double position_tolerance,
                   double yaw_tolerance, double hold_duration) {
    const bool inside =
        currentPositionError() <= position_tolerance &&
        currentYawError() <= yaw_tolerance;
    if (!inside) {
      arrival_since_ = ros::Time(0);
      return false;
    }
    if (arrival_since_.isZero()) {
      arrival_since_ = now;
    }
    return now - arrival_since_ >= ros::Duration(hold_duration);
  }

  double currentPositionError() const {
    return have_pose_ ? positionDistance(current_pose_, target_pose_)
                      : std::numeric_limits<double>::infinity();
  }

  double currentYawError() const {
    return have_pose_ ? yawError(current_pose_, target_pose_)
                      : std::numeric_limits<double>::infinity();
  }

  double currentHoldDuration(const ros::Time& now) const {
    if (arrival_since_.isZero() || now < arrival_since_) {
      return 0.0;
    }
    return (now - arrival_since_).toSec();
  }

  void publishTelemetry(const ros::Time& now) {
    std_msgs::String state_message;
    state_message.data = stateName(state_);
    state_pub_.publish(state_message);

    std_msgs::Float64 progress;
    const double total_legs = static_cast<double>(waypoints_.size());
    progress.data = total_legs > 0.0 ? completed_legs_ / total_legs : 0.0;
    if (orbit_completed_) {
      progress.data = 1.0;
    }
    progress_pub_.publish(progress);

    std_msgs::UInt32 waypoint_index;
    waypoint_index.data =
        waypoints_.empty() ? 0U
                           : static_cast<std::uint32_t>(current_waypoint_index_ + 1U);
    waypoint_index_pub_.publish(waypoint_index);

    std_msgs::Float64 position_error;
    position_error.data = currentPositionError();
    position_error_pub_.publish(position_error);
    std_msgs::Float64 yaw_error_message;
    yaw_error_message.data = radiansToDegrees(currentYawError());
    yaw_error_pub_.publish(yaw_error_message);
    std_msgs::Float64 hold_time;
    hold_time.data = currentHoldDuration(now);
    hold_time_pub_.publish(hold_time);
  }

  void writeReport(const ros::Time& now) {
    if (!report_.is_open() || !have_pose_ ||
        (!last_report_time_.isZero() &&
         now - last_report_time_ < ros::Duration(config_.report_period))) {
      return;
    }
    last_report_time_ = now;
    const geometry_msgs::PoseStamped reference_pose = makePose(
        reference_.x, reference_.y, reference_.z, reference_.yaw, now);
    report_ << std::fixed << std::setprecision(6) << now.toSec() << ','
            << stateName(state_) << ',' << current_waypoint_index_ + 1U << ','
            << completed_legs_ << ',' << waypoints_.size() << ','
            << target_pose_.pose.position.x << ','
            << target_pose_.pose.position.y << ','
            << target_pose_.pose.position.z << ','
            << radiansToDegrees(quaternionYaw(target_pose_.pose.orientation))
            << ',' << current_pose_.pose.position.x << ','
            << current_pose_.pose.position.y << ','
            << current_pose_.pose.position.z << ','
            << radiansToDegrees(quaternionYaw(current_pose_.pose.orientation))
            << ',' << currentPositionError() << ','
            << radiansToDegrees(currentYawError()) << ','
            << currentHoldDuration(now) << ','
            << positionDistance(current_pose_, reference_pose) << ','
            << (current_state_.armed ? "true" : "false") << ','
            << current_state_.mode << '\n';
  }

  void printMissionConfiguration(const char* prefix) const {
    const double clearance =
        config_.route.radius - config_.route.tower_collision_radius;
    ROS_INFO("[%s] tower=%s frame=%s center=(%.4f, %.4f) radius=%.2f m "
             "height=%.2f m trajectory=strict_circle checkpoints=%d "
             "direction=%s start=%.1f deg",
             prefix, config_.route.tower_name.c_str(),
             config_.route.frame_id.c_str(), config_.route.center_x,
             config_.route.center_y, config_.route.radius,
             config_.route.height, config_.route.waypoint_count,
             directionName(config_.route.direction),
             radiansToDegrees(config_.route.start_angle_rad));
    ROS_INFO("[%s] speed=%.2f m/s acceleration=%.2f m/s^2 camera_yaw_offset="
             "%.1f deg tower_clearance=%.2f m required_safety=%.2f m",
             prefix, config_.maximum_speed, config_.maximum_acceleration,
             radiansToDegrees(config_.route.camera_yaw_offset_rad), clearance,
             config_.route.minimum_safety_distance);
    ROS_INFO("[%s] takeoff=%.2f m return=%.2f m landing=%.2f->%.2f m/s "
             "height_bounds=[%.2f, %.2f] m run_mode=%s control=%s",
             prefix, config_.takeoff_height, config_.return_height,
             config_.landing_cruise_speed, config_.landing_touchdown_speed,
             config_.route.minimum_height, config_.route.maximum_height,
             config_.run_mode.c_str(),
             config_.enable_control ? "ENABLED" : "DISABLED");
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  NodeConfig config_;
  bool initialized_{true};
  std::vector<TowerWaypoint> waypoints_;

  ros::Subscriber state_sub_;
  ros::Subscriber extended_state_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber velocity_sub_;
  ros::Publisher position_setpoint_pub_;
  ros::Publisher raw_setpoint_pub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::Timer timer_;

  ros::Publisher route_preview_pub_;
  ros::Publisher waypoint_poses_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher current_target_pub_;
  ros::Publisher actual_path_pub_;
  ros::Publisher state_pub_;
  ros::Publisher progress_pub_;
  ros::Publisher waypoint_index_pub_;
  ros::Publisher position_error_pub_;
  ros::Publisher yaw_error_pub_;
  ros::Publisher hold_time_pub_;
  ros::Publisher final_result_pub_;
  ros::Publisher report_file_pub_;

  mavros_msgs::State current_state_;
  mavros_msgs::ExtendedState current_extended_state_;
  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::TwistStamped current_velocity_;
  bool have_state_{false};
  bool have_extended_state_{false};
  bool have_pose_{false};
  bool have_velocity_{false};
  ros::Time last_state_time_;
  ros::Time last_extended_state_time_;
  ros::Time last_pose_time_;
  ros::Time last_velocity_time_;

  MissionState state_{MissionState::kWaitInputs};
  geometry_msgs::PoseStamped home_pose_;
  geometry_msgs::PoseStamped target_pose_;
  MotionReference reference_;
  MotionTarget target_;
  CircularMotionReference circular_reference_;
  nav_msgs::Path actual_path_;
  double home_yaw_{0.0};
  double landing_x_{0.0};
  double landing_y_{0.0};
  std::size_t current_waypoint_index_{0};
  std::size_t completed_legs_{0};

  ros::Time node_started_time_;
  ros::Time state_entered_time_;
  ros::Time last_timer_time_;
  ros::Time armed_time_;
  ros::Time mission_started_time_;
  ros::Time arrival_since_;
  ros::Time mode_loss_start_;
  ros::Time landing_started_time_;
  ros::Time contact_candidate_since_;
  ros::Time ground_confirm_since_;
  ros::Time disarm_started_time_;
  ros::Time last_mode_request_;
  ros::Time last_arm_request_;
  ros::Time last_disarm_request_;
  ros::Time last_authority_check_time_;
  ros::Time last_report_time_;

  bool cached_authority_conflict_{false};
  std::string cached_conflict_detail_;
  bool landing_contact_assist_{false};
  bool orbit_started_{false};
  bool orbit_completed_{false};
  bool returned_home_{false};
  bool landed_and_disarmed_{false};
  bool mission_failed_{false};
  std::string failure_reason_;
  std::string final_result_;
  double maximum_reference_tracking_error_{0.0};
  double maximum_yaw_error_{0.0};
  std::ofstream report_;
};

}  // namespace astra_tower_mission

int main(int argc, char** argv) {
  ros::init(argc, argv, "tower_mission");
  astra_tower_mission::TowerMissionNode node;
  if (!node.initialized()) {
    return 1;
  }
  ros::spin();
  return 0;
}
