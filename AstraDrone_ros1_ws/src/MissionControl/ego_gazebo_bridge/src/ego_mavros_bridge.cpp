#include "ego_gazebo_bridge/ego_mavros_bridge.h"

#include <ros/master.h>
#include <ros/this_node.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ego_gazebo_bridge {
namespace {

constexpr double kAuthorityCheckInterval = 1.0;
constexpr double kTfTimeout = 0.05;

bool isFresh(const ros::Time& now, const ros::Time& received,
             double timeout, double future_tolerance) {
  return isTimestampUsable(now, received, timeout, future_tolerance);
}

bool isStampedFresh(const ros::Time& now, const ros::Time& received,
                    const ros::Time& source_stamp, double timeout,
                    double future_tolerance) {
  return isFresh(now, received, timeout, future_tolerance) &&
         isTimestampUsable(now, source_stamp, timeout, future_tolerance);
}

bool pointCloudHasFiniteXyz(const sensor_msgs::PointCloud2& cloud,
                            std::string* reason) {
  if (cloud.width == 0 || cloud.height == 0 || cloud.data.empty()) {
    *reason = "point cloud is empty";
    return false;
  }
  try {
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
        *reason = "point cloud contains NaN/Inf coordinates";
        return false;
      }
    }
  } catch (const std::runtime_error& exception) {
    *reason = std::string("point cloud xyz fields are unusable: ") +
              exception.what();
    return false;
  }
  reason->clear();
  return true;
}

geometry_msgs::PoseStamped odometryPose(const nav_msgs::Odometry& odometry) {
  geometry_msgs::PoseStamped pose;
  pose.header = odometry.header;
  pose.pose = odometry.pose.pose;
  return pose;
}

}  // namespace

const char* bridgeStateName(BridgeState state) {
  switch (state) {
    case BridgeState::kDryRun:
      return "DRY_RUN";
    case BridgeState::kWaitFcu:
      return "WAIT_FCU";
    case BridgeState::kWaitInputs:
      return "WAIT_INPUTS";
    case BridgeState::kPrestream:
      return "PRESTREAM";
    case BridgeState::kArmOffboard:
      return "ARM_OFFBOARD";
    case BridgeState::kTakeoff:
      return "TAKEOFF";
    case BridgeState::kHoverReady:
      return "HOVER_READY";
    case BridgeState::kTrackEgo:
      return "TRACK_EGO";
    case BridgeState::kHold:
      return "HOLD";
    case BridgeState::kHomeHover:
      return "HOME_HOVER";
    case BridgeState::kLanding:
      return "LANDING";
    case BridgeState::kDone:
      return "DONE";
    case BridgeState::kError:
      return "ERROR";
  }
  return "UNKNOWN";
}

EgoMavrosBridge::EgoMavrosBridge(ros::NodeHandle node_handle,
                                 ros::NodeHandle private_node_handle)
    : node_handle_(std::move(node_handle)),
      private_node_handle_(std::move(private_node_handle)),
      tf_listener_(tf_buffer_) {
  loadConfig();
  setupRosInterfaces();

  state_entered_time_ = ros::Time::now();
  if (!validateConfig()) {
    transitionTo(BridgeState::kError, "invalid bridge configuration");
    return;
  }

  bool use_sim_time = false;
  node_handle_.param("/use_sim_time", use_sim_time, false);
  if (config_.require_sim_time && !use_sim_time) {
    transitionTo(BridgeState::kError,
                 "simulation-only bridge requires /use_sim_time=true");
    return;
  }

  if (!config_.enable_control) {
    transitionTo(BridgeState::kDryRun,
                 "control disabled; transformed commands are debug-only");
    return;
  }

  std::string conflict;
  if (hasControlConflict(&conflict)) {
    transitionTo(BridgeState::kError, conflict);
    return;
  }

  setpoint_publisher_ = node_handle_.advertise<mavros_msgs::PositionTarget>(
      config_.setpoint_topic, 20);
  transitionTo(BridgeState::kWaitFcu, "bridge initialized");
}

void EgoMavrosBridge::loadConfig() {
  private_node_handle_.param("enable_control", config_.enable_control,
                             config_.enable_control);
  private_node_handle_.param("require_sim_time", config_.require_sim_time,
                             config_.require_sim_time);
  private_node_handle_.param("auto_track_on_command",
                             config_.auto_track_on_command,
                             config_.auto_track_on_command);
  private_node_handle_.param("require_start_permission",
                             config_.require_start_permission,
                             config_.require_start_permission);
  private_node_handle_.param("require_orbit_speed_scale",
                             config_.require_orbit_speed_scale,
                             config_.require_orbit_speed_scale);
  private_node_handle_.param("tower_yaw_override_enabled",
                             config_.tower_yaw_override_enabled,
                             config_.tower_yaw_override_enabled);
  private_node_handle_.param("publish_rate", config_.publish_rate,
                             config_.publish_rate);
  private_node_handle_.param("prestream_duration",
                             config_.prestream_duration,
                             config_.prestream_duration);
  private_node_handle_.param("request_interval", config_.request_interval,
                             config_.request_interval);
  private_node_handle_.param("takeoff_height", config_.takeoff_height,
                             config_.takeoff_height);
  config_.return_height = config_.takeoff_height;
  private_node_handle_.param("return_height", config_.return_height,
                             config_.return_height);
  private_node_handle_.param("takeoff_tolerance",
                             config_.takeoff_tolerance,
                             config_.takeoff_tolerance);
  private_node_handle_.param("hover_duration", config_.hover_duration,
                             config_.hover_duration);
  private_node_handle_.param("command_timeout", config_.command_timeout,
                             config_.command_timeout);
  private_node_handle_.param("timestamp_future_tolerance",
                             config_.timestamp_future_tolerance,
                             config_.timestamp_future_tolerance);
  private_node_handle_.param("goal_timeout", config_.goal_timeout,
                             config_.goal_timeout);
  private_node_handle_.param("fcu_state_timeout", config_.fcu_state_timeout,
                             config_.fcu_state_timeout);
  private_node_handle_.param("extended_state_timeout",
                             config_.extended_state_timeout,
                             config_.extended_state_timeout);
  private_node_handle_.param("mavros_pose_timeout",
                             config_.mavros_pose_timeout,
                             config_.mavros_pose_timeout);
  private_node_handle_.param("planner_odom_timeout",
                             config_.planner_odom_timeout,
                             config_.planner_odom_timeout);
  private_node_handle_.param("cloud_timeout", config_.cloud_timeout,
                             config_.cloud_timeout);
  private_node_handle_.param("land_after_loss", config_.land_after_loss,
                             config_.land_after_loss);
  private_node_handle_.param("wait_fcu_timeout", config_.wait_fcu_timeout,
                             config_.wait_fcu_timeout);
  private_node_handle_.param("wait_inputs_timeout",
                             config_.wait_inputs_timeout,
                             config_.wait_inputs_timeout);
  private_node_handle_.param("arm_offboard_timeout",
                             config_.arm_offboard_timeout,
                             config_.arm_offboard_timeout);
  private_node_handle_.param("takeoff_timeout", config_.takeoff_timeout,
                             config_.takeoff_timeout);
  private_node_handle_.param("landing_timeout", config_.landing_timeout,
                             config_.landing_timeout);
  private_node_handle_.param("max_position_rate", config_.max_position_rate,
                             config_.max_position_rate);
  private_node_handle_.param("max_yaw_rate", config_.max_yaw_rate,
                             config_.max_yaw_rate);
  private_node_handle_.param("max_velocity", config_.max_velocity,
                             config_.max_velocity);
  private_node_handle_.param("max_acceleration", config_.max_acceleration,
                             config_.max_acceleration);
  private_node_handle_.param("gravity_alignment_tolerance",
                             config_.gravity_alignment_tolerance,
                             config_.gravity_alignment_tolerance);
  private_node_handle_.param("tracking_error_limit",
                             config_.tracking_error_limit,
                             config_.tracking_error_limit);
  private_node_handle_.param("tracking_error_duration",
                             config_.tracking_error_duration,
                             config_.tracking_error_duration);
  private_node_handle_.param("alignment_position_tolerance",
                             config_.alignment_position_tolerance,
                             config_.alignment_position_tolerance);
  private_node_handle_.param("alignment_yaw_tolerance",
                             config_.alignment_yaw_tolerance,
                             config_.alignment_yaw_tolerance);
  private_node_handle_.param("alignment_yaw_error_duration",
                             config_.alignment_yaw_error_duration,
                             config_.alignment_yaw_error_duration);
  private_node_handle_.param("return_tolerance", config_.return_tolerance,
                             config_.return_tolerance);
  private_node_handle_.param("return_hold_duration",
                             config_.return_hold_duration,
                             config_.return_hold_duration);
  private_node_handle_.param("tower_camera_yaw_offset",
                             config_.tower_camera_yaw_offset,
                             config_.tower_camera_yaw_offset);
  private_node_handle_.param("forward_yaw_min_speed",
                             config_.forward_yaw_min_speed,
                             config_.forward_yaw_min_speed);
  private_node_handle_.param("orbit_speed_scale_timeout",
                             config_.orbit_speed_scale_timeout,
                             config_.orbit_speed_scale_timeout);
  private_node_handle_.param("min_relative_height",
                             config_.bounds.min_relative_height,
                             config_.bounds.min_relative_height);
  private_node_handle_.param("max_relative_height",
                             config_.bounds.max_relative_height,
                             config_.bounds.max_relative_height);
  private_node_handle_.param("max_horizontal_radius",
                             config_.bounds.max_horizontal_radius,
                             config_.bounds.max_horizontal_radius);

  private_node_handle_.param("planning_frame", config_.planning_frame,
                             config_.planning_frame);
  private_node_handle_.param("mavros_frame", config_.mavros_frame,
                             config_.mavros_frame);
  private_node_handle_.param("command_topic", config_.command_topic,
                             config_.command_topic);
  private_node_handle_.param("planner_odom_topic",
                             config_.planner_odom_topic,
                             config_.planner_odom_topic);
  private_node_handle_.param("cloud_topic", config_.cloud_topic,
                             config_.cloud_topic);
  private_node_handle_.param("mavros_state_topic",
                             config_.mavros_state_topic,
                             config_.mavros_state_topic);
  private_node_handle_.param("mavros_extended_state_topic",
                             config_.mavros_extended_state_topic,
                             config_.mavros_extended_state_topic);
  private_node_handle_.param("mavros_pose_topic",
                             config_.mavros_pose_topic,
                             config_.mavros_pose_topic);
  private_node_handle_.param("setpoint_topic", config_.setpoint_topic,
                             config_.setpoint_topic);
  private_node_handle_.param("arming_service", config_.arming_service,
                             config_.arming_service);
  private_node_handle_.param("set_mode_service", config_.set_mode_service,
                             config_.set_mode_service);
  private_node_handle_.param("input_goal_topic", config_.input_goal_topic,
                             config_.input_goal_topic);
  private_node_handle_.param("planner_goal_topic",
                             config_.planner_goal_topic,
                             config_.planner_goal_topic);
  private_node_handle_.param("planning_cancel_topic",
                             config_.planning_cancel_topic,
                             config_.planning_cancel_topic);
  private_node_handle_.param("tower_center_topic",
                             config_.tower_center_topic,
                             config_.tower_center_topic);
  private_node_handle_.param("tower_yaw_mode_topic",
                             config_.tower_yaw_mode_topic,
                             config_.tower_yaw_mode_topic);
  private_node_handle_.param("start_permission_topic",
                             config_.start_permission_topic,
                             config_.start_permission_topic);
  private_node_handle_.param("orbit_speed_scale_topic",
                             config_.orbit_speed_scale_topic,
                             config_.orbit_speed_scale_topic);
  private_node_handle_.getParam("control_topics/position",
                                config_.position_control_topics);
  private_node_handle_.getParam("control_topics/raw_local",
                                config_.raw_local_control_topics);
  private_node_handle_.getParam("control_topics/velocity",
                                config_.velocity_control_topics);
  private_node_handle_.getParam("control_topics/attitude",
                                config_.attitude_control_topics);
  private_node_handle_.getParam("control_topics/thrust",
                                config_.thrust_control_topics);
}

bool EgoMavrosBridge::validateConfig() const {
  const bool valid_positive_values =
      config_.publish_rate >= 20.0 && config_.prestream_duration > 0.0 &&
      config_.request_interval > 0.0 && config_.takeoff_height > 0.0 &&
      config_.return_height > 0.0 &&
      config_.takeoff_tolerance > 0.0 && config_.hover_duration >= 0.0 &&
      config_.command_timeout > 0.0 && config_.goal_timeout > 0.0 &&
      config_.timestamp_future_tolerance >= 0.0 &&
      config_.fcu_state_timeout > 0.0 &&
      config_.extended_state_timeout > 0.0 &&
      config_.mavros_pose_timeout > 0.0 &&
      config_.planner_odom_timeout > 0.0 && config_.cloud_timeout > 0.0 &&
      config_.land_after_loss > config_.command_timeout &&
      config_.wait_fcu_timeout > 0.0 && config_.wait_inputs_timeout > 0.0 &&
      config_.arm_offboard_timeout > 0.0 && config_.takeoff_timeout > 0.0 &&
      config_.landing_timeout > 0.0 &&
      config_.max_position_rate > 0.0 && config_.max_yaw_rate > 0.0 &&
      config_.max_velocity > 0.0 && config_.max_acceleration > 0.0 &&
      config_.gravity_alignment_tolerance >= 0.0 &&
      config_.tracking_error_limit > 0.0 &&
      config_.tracking_error_duration > 0.0 &&
      config_.alignment_position_tolerance > 0.0 &&
      config_.alignment_yaw_tolerance > 0.0 &&
      config_.alignment_yaw_error_duration > 0.0 &&
      config_.return_tolerance > 0.0 &&
      config_.return_hold_duration >= 0.0 &&
      std::isfinite(config_.tower_camera_yaw_offset) &&
      std::isfinite(config_.forward_yaw_min_speed) &&
      config_.forward_yaw_min_speed > 0.0 &&
      config_.orbit_speed_scale_timeout > 0.0 &&
      config_.bounds.min_relative_height >= 0.0 &&
      config_.bounds.max_relative_height >
          config_.bounds.min_relative_height &&
      config_.takeoff_height <= config_.bounds.max_relative_height &&
      config_.return_height <= config_.bounds.max_relative_height &&
      config_.bounds.max_horizontal_radius > 0.0;
  const bool valid_names =
      !config_.planning_frame.empty() && !config_.mavros_frame.empty() &&
      !config_.command_topic.empty() &&
      !config_.planner_odom_topic.empty() && !config_.cloud_topic.empty() &&
      !config_.mavros_state_topic.empty() &&
      !config_.mavros_extended_state_topic.empty() &&
      !config_.mavros_pose_topic.empty() &&
      !config_.setpoint_topic.empty() && !config_.arming_service.empty() &&
      !config_.set_mode_service.empty() &&
      !config_.input_goal_topic.empty() &&
      !config_.planner_goal_topic.empty() &&
      !config_.planning_cancel_topic.empty() &&
      config_.input_goal_topic != config_.planner_goal_topic &&
      (!config_.tower_yaw_override_enabled ||
       (!config_.tower_center_topic.empty() &&
        !config_.tower_yaw_mode_topic.empty())) &&
      (!config_.require_orbit_speed_scale ||
       !config_.orbit_speed_scale_topic.empty());
  const auto valid_topic_group = [](const std::vector<std::string>& topics) {
    return !topics.empty() &&
           std::all_of(topics.begin(), topics.end(),
                       [](const std::string& topic) { return !topic.empty(); });
  };
  const bool valid_control_topics =
      valid_topic_group(config_.position_control_topics) &&
      valid_topic_group(config_.raw_local_control_topics) &&
      valid_topic_group(config_.velocity_control_topics) &&
      valid_topic_group(config_.attitude_control_topics) &&
      valid_topic_group(config_.thrust_control_topics);

  if (!valid_positive_values) {
    ROS_FATAL("[BRIDGE] Invalid numeric safety parameter.");
  }
  if (!valid_names) {
    ROS_FATAL("[BRIDGE] Topic, service and frame names must not be empty.");
  }
  if (!valid_control_topics) {
    ROS_FATAL("[BRIDGE] Every MAVROS control category must contain a topic.");
  }
  return valid_positive_values && valid_names && valid_control_topics;
}

void EgoMavrosBridge::setupRosInterfaces() {
  fcu_state_subscriber_ = node_handle_.subscribe<mavros_msgs::State>(
      config_.mavros_state_topic, 10,
      &EgoMavrosBridge::fcuStateCallback, this);
  extended_state_subscriber_ =
      node_handle_.subscribe<mavros_msgs::ExtendedState>(
          config_.mavros_extended_state_topic, 10,
          &EgoMavrosBridge::extendedStateCallback, this);
  mavros_pose_subscriber_ =
      node_handle_.subscribe<geometry_msgs::PoseStamped>(
          config_.mavros_pose_topic, 10,
          &EgoMavrosBridge::mavrosPoseCallback, this);
  planner_odom_subscriber_ = node_handle_.subscribe<nav_msgs::Odometry>(
      config_.planner_odom_topic, 10,
      &EgoMavrosBridge::plannerOdomCallback, this);
  cloud_subscriber_ = node_handle_.subscribe<sensor_msgs::PointCloud2>(
      config_.cloud_topic, 1, &EgoMavrosBridge::cloudCallback, this);
  command_subscriber_ =
      node_handle_.subscribe<quadrotor_msgs::PositionCommand>(
          // PositionCommand is a real-time reference: after any callback
          // backlog only the newest sample is useful and safe to execute.
          config_.command_topic, 1,
          &EgoMavrosBridge::commandCallback, this);
  goal_subscriber_ = node_handle_.subscribe<geometry_msgs::PoseStamped>(
      config_.input_goal_topic, 10, &EgoMavrosBridge::goalCallback, this);
  if (config_.tower_yaw_override_enabled) {
    tower_center_subscriber_ =
        node_handle_.subscribe<geometry_msgs::PointStamped>(
            config_.tower_center_topic, 1,
            &EgoMavrosBridge::towerCenterCallback, this);
    tower_yaw_mode_subscriber_ = node_handle_.subscribe<std_msgs::Bool>(
        config_.tower_yaw_mode_topic, 1,
        &EgoMavrosBridge::towerYawModeCallback, this);
  }
  if (config_.require_start_permission) {
    start_permission_subscriber_ = node_handle_.subscribe<std_msgs::Bool>(
        config_.start_permission_topic, 5,
        &EgoMavrosBridge::startPermissionCallback, this);
  }
  if (config_.require_orbit_speed_scale) {
    orbit_speed_scale_subscriber_ = node_handle_.subscribe<std_msgs::Float64>(
        config_.orbit_speed_scale_topic, 5,
        &EgoMavrosBridge::orbitSpeedScaleCallback, this);
  }

  debug_setpoint_publisher_ =
      private_node_handle_.advertise<geometry_msgs::PoseStamped>(
          "debug_setpoint", 10);
  state_publisher_ = private_node_handle_.advertise<std_msgs::String>(
      "state", 1, true);
  input_health_publisher_ = private_node_handle_.advertise<std_msgs::String>(
      "input_health", 10, true);
  tracking_error_publisher_ =
      private_node_handle_.advertise<std_msgs::Float64>(
          "tracking_error", 10);
  goal_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>(
      config_.planner_goal_topic, 1, false);
  planning_cancel_publisher_ = node_handle_.advertise<std_msgs::Empty>(
      config_.planning_cancel_topic, 1, false);

  arming_client_ = node_handle_.serviceClient<mavros_msgs::CommandBool>(
      config_.arming_service);
  set_mode_client_ = node_handle_.serviceClient<mavros_msgs::SetMode>(
      config_.set_mode_service);
  tracking_service_ = private_node_handle_.advertiseService(
      "enable_tracking", &EgoMavrosBridge::trackingService, this);
  land_service_ = private_node_handle_.advertiseService(
      "land", &EgoMavrosBridge::landService, this);
  return_home_service_ = private_node_handle_.advertiseService(
      "return_home", &EgoMavrosBridge::returnHomeService, this);
  cancel_current_trajectory_service_ = private_node_handle_.advertiseService(
      "cancel_current_trajectory",
      &EgoMavrosBridge::cancelCurrentTrajectoryService, this);
  resume_ego_service_ = private_node_handle_.advertiseService(
      "resume_ego", &EgoMavrosBridge::resumeEgoService, this);

  const double period = 1.0 / std::max(20.0, config_.publish_rate);
  control_timer_ = node_handle_.createTimer(
      ros::Duration(period), &EgoMavrosBridge::controlTimerCallback, this);
}

void EgoMavrosBridge::fcuStateCallback(
    const mavros_msgs::State::ConstPtr& message) {
  fcu_state_ = *message;
  have_fcu_state_ = true;
  last_fcu_state_time_ = ros::Time::now();
}

void EgoMavrosBridge::extendedStateCallback(
    const mavros_msgs::ExtendedState::ConstPtr& message) {
  extended_state_ = *message;
  have_extended_state_ = true;
  last_extended_state_time_ = ros::Time::now();
}

void EgoMavrosBridge::startPermissionCallback(
    const std_msgs::Bool::ConstPtr& message) {
  start_permission_ = message->data;
}

void EgoMavrosBridge::orbitSpeedScaleCallback(
    const std_msgs::Float64::ConstPtr& message) {
  if (!std::isfinite(message->data) || message->data < 0.0 ||
      message->data > 1.0) {
    have_orbit_speed_scale_ = false;
    orbit_speed_scale_ = 0.0;
    ROS_ERROR_THROTTLE(
        1.0, "[BRIDGE] Invalid orbit speed scale rejected: %.6f",
        message->data);
    return;
  }
  orbit_speed_scale_ = message->data;
  have_orbit_speed_scale_ = true;
  last_orbit_speed_scale_time_ = ros::Time::now();
}

void EgoMavrosBridge::mavrosPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& message) {
  have_mavros_pose_ = false;
  if (!isFinitePose(*message)) {
    ROS_ERROR_THROTTLE(1.0, "[BRIDGE] Rejected invalid MAVROS pose.");
    return;
  }
  if (message->header.frame_id != config_.mavros_frame) {
    ROS_ERROR_THROTTLE(
        1.0,
        "[BRIDGE] MAVROS pose frame '%s' must equal mavros_frame '%s'.",
        message->header.frame_id.c_str(), config_.mavros_frame.c_str());
    return;
  }
  std::string reason;
  if (!transformToMavros(*message, &mavros_pose_, &reason)) {
    ROS_ERROR_THROTTLE(1.0, "[BRIDGE] MAVROS pose transform failed: %s",
                       reason.c_str());
    return;
  }
  have_mavros_pose_ = true;
  last_mavros_pose_time_ = ros::Time::now();
  last_mavros_pose_stamp_ = message->header.stamp;
}

void EgoMavrosBridge::plannerOdomCallback(
    const nav_msgs::Odometry::ConstPtr& message) {
  have_planner_odom_ = false;
  const geometry_msgs::PoseStamped pose = odometryPose(*message);
  if (!isFinitePose(pose)) {
    ROS_ERROR_THROTTLE(1.0, "[BRIDGE] Rejected invalid planner odometry.");
    return;
  }
  if (message->header.frame_id != config_.planning_frame) {
    ROS_ERROR_THROTTLE(
        1.0,
        "[BRIDGE] Planner odom frame '%s' must equal planning_frame '%s'.",
        message->header.frame_id.c_str(), config_.planning_frame.c_str());
    return;
  }
  planner_odom_ = *message;
  have_planner_odom_ = true;
  last_planner_odom_time_ = ros::Time::now();
  last_planner_odom_stamp_ = message->header.stamp;
}

void EgoMavrosBridge::cloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& message) {
  have_cloud_ = false;
  if (message->header.frame_id.empty()) {
    ROS_WARN_THROTTLE(1.0, "[BRIDGE] Point cloud has no frame_id.");
    return;
  }
  const std::size_t minimum_row_size =
      static_cast<std::size_t>(message->width) * message->point_step;
  const bool invalid_layout =
      message->height == 0 || message->width == 0 ||
      message->point_step == 0 || message->row_step == 0 ||
      message->row_step < minimum_row_size ||
      message->data.size() / message->row_step < message->height;
  if (invalid_layout) {
    ROS_ERROR_THROTTLE(
        1.0, "[BRIDGE] Empty or structurally invalid cloud was rejected.");
    return;
  }
  if (message->header.frame_id != config_.planning_frame) {
    ROS_ERROR_THROTTLE(
        1.0,
        "[BRIDGE] Cloud frame '%s' must equal planning_frame '%s'; EGO does not transform this cloud input.",
        message->header.frame_id.c_str(), config_.planning_frame.c_str());
    return;
  }
  std::string reason;
  if (!pointCloudHasFiniteXyz(*message, &reason)) {
    ROS_ERROR_THROTTLE(1.0, "[BRIDGE] Point cloud rejected: %s",
                       reason.c_str());
    return;
  }
  have_cloud_ = true;
  last_cloud_time_ = ros::Time::now();
  last_cloud_stamp_ = message->header.stamp;
}

void EgoMavrosBridge::towerCenterCallback(
    const geometry_msgs::PointStamped::ConstPtr& message) {
  if (message->header.frame_id.empty() ||
      !std::isfinite(message->point.x) ||
      !std::isfinite(message->point.y) ||
      !std::isfinite(message->point.z)) {
    ROS_ERROR("[BRIDGE] Invalid selected tower center was rejected.");
    have_tower_center_ = false;
    return;
  }
  tower_center_ = *message;
  have_tower_center_ = true;
  ROS_INFO("[BRIDGE] Selected tower center received: frame=%s, "
           "center=(%.3f, %.3f, %.3f).",
           tower_center_.header.frame_id.c_str(), tower_center_.point.x,
           tower_center_.point.y, tower_center_.point.z);
}

void EgoMavrosBridge::towerYawModeCallback(
    const std_msgs::Bool::ConstPtr& message) {
  if (tower_yaw_mode_ == message->data) {
    return;
  }
  tower_yaw_mode_ = message->data;
  // The desired yaw policy can change discontinuously. Seed the next limiter
  // step from the measured vehicle yaw, never from the new policy target.
  have_effective_yaw_ = false;
  last_effective_yaw_time_ = ros::Time(0);
  ROS_INFO("[BRIDGE] Yaw policy changed to %s.",
           tower_yaw_mode_ ? "FACE_SELECTED_TOWER" : "VELOCITY_FORWARD");
}

void EgoMavrosBridge::commandCallback(
    const quadrotor_msgs::PositionCommand::ConstPtr& message) {
  if (state_ == BridgeState::kLanding || state_ == BridgeState::kDone ||
      state_ == BridgeState::kError) {
    ROS_INFO_THROTTLE(
        1.0,
        "[BRIDGE] PositionCommand ignored after terminal landing ownership.");
    return;
  }
  if (message->trajectory_flag !=
      quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY) {
    ROS_WARN_THROTTLE(
        1.0,
        "[BRIDGE] PositionCommand ignored: trajectory_flag=%u, expected READY=%u.",
        static_cast<unsigned int>(message->trajectory_flag),
        static_cast<unsigned int>(
            quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY));
    return;
  }
  if (!isFinitePositionCommand(*message)) {
    ROS_ERROR_THROTTLE(
        1.0,
        "[BRIDGE] PositionCommand rejected: NaN/Inf in p=(%.3f, %.3f, %.3f), v=(%.3f, %.3f, %.3f), a=(%.3f, %.3f, %.3f), yaw=%.3f, yaw_dot=%.3f.",
        message->position.x, message->position.y, message->position.z,
        message->velocity.x, message->velocity.y, message->velocity.z,
        message->acceleration.x, message->acceleration.y,
        message->acceleration.z, message->yaw, message->yaw_dot);
    return;
  }
  if (message->header.frame_id != config_.planning_frame) {
    ROS_ERROR_THROTTLE(
        1.0,
        "[BRIDGE] PositionCommand frame '%s' must equal planning_frame '%s'.",
        message->header.frame_id.c_str(), config_.planning_frame.c_str());
    return;
  }
  const ros::Time now = ros::Time::now();
  const double command_age = message->header.stamp.isZero()
                                 ? std::numeric_limits<double>::infinity()
                                 : (now - message->header.stamp).toSec();
  if (!isTimestampUsable(now, message->header.stamp,
                         config_.command_timeout,
                         config_.timestamp_future_tolerance)) {
    ROS_ERROR_THROTTLE(
        1.0,
        "[BRIDGE] PositionCommand timestamp rejected: age=%.3f s, timeout=%.3f s, now=%.6f, stamp=%.6f.",
        command_age, config_.command_timeout, now.toSec(),
        message->header.stamp.toSec());
    return;
  }
  if (trajectory_gate_.active()) {
    if (!trajectory_gate_.allows(message->trajectory_id,
                                 message->header.stamp,
                                 have_valid_goal_)) {
      ROS_WARN_THROTTLE(
          1.0,
          "[BRIDGE] Cancelled/old trajectory %u ignored while waiting for a new goal generation.",
          static_cast<unsigned int>(message->trajectory_id));
      return;
    }
    ROS_INFO("[BRIDGE] New post-cancel trajectory %u accepted.",
             static_cast<unsigned int>(message->trajectory_id));
  }

  quadrotor_msgs::PositionCommand effective_command = *message;
  if (config_.require_orbit_speed_scale) {
    effective_command.velocity.x *= orbit_speed_scale_;
    effective_command.velocity.y *= orbit_speed_scale_;
    effective_command.velocity.z *= orbit_speed_scale_;
    effective_command.acceleration.x *= orbit_speed_scale_;
    effective_command.acceleration.y *= orbit_speed_scale_;
    effective_command.acceleration.z *= orbit_speed_scale_;
    effective_command.yaw_dot *= orbit_speed_scale_;
  }
  const bool yaw_history_fresh =
      have_effective_yaw_ && !last_effective_yaw_time_.isZero() &&
      now > last_effective_yaw_time_ &&
      now - last_effective_yaw_time_ <=
          ros::Duration(config_.command_timeout);
  const double yaw_reference =
      yaw_history_fresh
          ? effective_yaw_
          : (have_mavros_pose_
                 ? yawFromQuaternion(mavros_pose_.pose.orientation)
                 : effective_command.yaw);
  const double yaw_dt =
      yaw_history_fresh
          ? (now - last_effective_yaw_time_).toSec()
          : 1.0 / config_.publish_rate;
  if (config_.tower_yaw_override_enabled && tower_yaw_mode_) {
    if (!have_tower_center_) {
      ROS_ERROR_THROTTLE(
          1.0,
          "[BRIDGE] Tower-facing yaw requested without a selected tower center.");
      return;
    }
    geometry_msgs::PointStamped center = tower_center_;
    center.header.stamp = message->header.stamp;
    geometry_msgs::PointStamped planning_center;
    try {
      if (center.header.frame_id == config_.planning_frame) {
        planning_center = center;
      } else {
        const auto transform = tf_buffer_.lookupTransform(
            config_.planning_frame, center.header.frame_id,
            message->header.stamp, ros::Duration(kTfTimeout));
        tf2::doTransform(center, planning_center, transform);
      }
    } catch (const tf2::TransformException& exception) {
      ROS_ERROR_THROTTLE(1.0, "[BRIDGE] Tower center TF failed: %s",
                         exception.what());
      return;
    }
    std::string yaw_reason;
    if (!applyPointFacingYaw(planning_center.point,
                             config_.tower_camera_yaw_offset,
                             &effective_command, &yaw_reason)) {
      ROS_ERROR_THROTTLE(1.0, "[BRIDGE] Tower yaw command rejected: %s",
                         yaw_reason.c_str());
      return;
    }
  } else if (config_.tower_yaw_override_enabled) {
    std::string yaw_reason;
    if (!applyVelocityFacingYaw(config_.forward_yaw_min_speed,
                                yaw_reference,
                                &effective_command, &yaw_reason)) {
      ROS_ERROR_THROTTLE(1.0, "[BRIDGE] Forward yaw command rejected: %s",
                         yaw_reason.c_str());
      return;
    }
  }

  double next_effective_yaw = yaw_reference;
  double next_effective_yaw_rate = 0.0;
  if (!limitYawCommand(yaw_reference, effective_command.yaw,
                       config_.max_yaw_rate, yaw_dt,
                       &next_effective_yaw, &next_effective_yaw_rate)) {
    ROS_ERROR_THROTTLE(1.0, "[BRIDGE] Yaw rate limiter rejected invalid input.");
    return;
  }
  effective_command.yaw = next_effective_yaw;
  effective_command.yaw_dot = next_effective_yaw_rate;

  geometry_msgs::PoseStamped planning_pose =
      commandToPose(effective_command, config_.planning_frame);
  geometry_msgs::PoseStamped mavros_target;
  std::string reason;
  if (!transformToMavros(planning_pose, &mavros_target, &reason)) {
    ROS_ERROR_THROTTLE(1.0, "[BRIDGE] Command transform failed: %s",
                       reason.c_str());
    return;
  }

  if (have_home_ &&
      !isWithinBounds(mavros_target, home_pose_, config_.bounds, &reason)) {
    ROS_ERROR_THROTTLE(
        1.0,
        "[BRIDGE] Command rejected: %s; target=(%.2f, %.2f, %.2f), home=(%.2f, %.2f, %.2f), horizontal_distance=%.2f m, radius_limit=%.2f m.",
        reason.c_str(), mavros_target.pose.position.x,
        mavros_target.pose.position.y, mavros_target.pose.position.z,
        home_pose_.pose.position.x, home_pose_.pose.position.y,
        home_pose_.pose.position.z,
        horizontalDistance(mavros_target, home_pose_),
        config_.bounds.max_horizontal_radius);
    return;
  }

  geometry_msgs::TransformStamped planning_to_mavros;
  if (!planningToMavrosTransform(message->header.stamp,
                                 &planning_to_mavros, &reason)) {
    ROS_ERROR_THROTTLE(1.0, "[BRIDGE] Command TF rejected: %s",
                       reason.c_str());
    return;
  }
  RawCommandLimits limits;
  limits.max_velocity = config_.max_velocity;
  limits.max_acceleration = config_.max_acceleration;
  limits.max_yaw_rate = config_.max_yaw_rate;
  limits.gravity_alignment_tolerance =
      config_.gravity_alignment_tolerance;
  mavros_msgs::PositionTarget raw_target;
  if (!commandToRawTarget(effective_command, planning_to_mavros, limits, now,
                          &raw_target, &reason)) {
    ROS_ERROR_THROTTLE(1.0, "[BRIDGE] Raw command conversion failed: %s",
                       reason.c_str());
    return;
  }

  planner_target_mavros_ = mavros_target;
  planner_raw_target_ = raw_target;
  have_planner_target_ = true;
  last_command_time_ = now;
  last_command_stamp_ = message->header.stamp;
  effective_yaw_ = effective_command.yaw;
  have_effective_yaw_ = true;
  last_effective_yaw_time_ = now;
  trajectory_gate_.accept(message->trajectory_id);
  debug_setpoint_publisher_.publish(planner_target_mavros_);

  if (config_.auto_track_on_command &&
      state_ == BridgeState::kHoverReady) {
    tracking_requested_ = true;
  }
}

void EgoMavrosBridge::goalCallback(
    const geometry_msgs::PoseStamped::ConstPtr& message) {
  have_valid_goal_ = false;
  const bool state_accepts_goals =
      state_ == BridgeState::kDryRun ||
      state_ == BridgeState::kWaitInputs ||
      state_ == BridgeState::kHoverReady ||
      state_ == BridgeState::kTrackEgo || state_ == BridgeState::kHold;
  if (!state_accepts_goals) {
    ROS_WARN("[BRIDGE] Goal rejected in state %s.", bridgeStateName(state_));
    return;
  }
  if (!isFinitePose(*message)) {
    ROS_ERROR(
        "[BRIDGE] Goal rejected because it contains NaN/Inf or an invalid quaternion.");
    return;
  }

  const ros::Time now = ros::Time::now();
  std::string reason;
  if (!baseInputsFresh(now, &reason)) {
    ROS_WARN("[BRIDGE] Goal rejected because EGO inputs are invalid: %s.",
             reason.c_str());
    return;
  }
  if (!isTimestampUsable(now, message->header.stamp,
                         config_.goal_timeout,
                         config_.timestamp_future_tolerance)) {
    ROS_ERROR("[BRIDGE] Goal rejected because its timestamp is zero, future or stale.");
    return;
  }
  if (!have_home_) {
    ROS_WARN(
        "[BRIDGE] Goal rejected until dry-run/preflight captures a disarmed ON_GROUND home pose.");
    return;
  }

  geometry_msgs::PoseStamped planning_goal;
  if (!transformToPlanning(*message, &planning_goal, &reason)) {
    ROS_ERROR("[BRIDGE] Goal transform failed: %s", reason.c_str());
    return;
  }

  geometry_msgs::PoseStamped mavros_goal;
  if (!transformToMavros(planning_goal, &mavros_goal, &reason)) {
    ROS_ERROR("[BRIDGE] Goal validation transform failed: %s", reason.c_str());
    return;
  }
  const double goal_horizontal_distance =
      horizontalDistance(mavros_goal, home_pose_);
  if (!isWithinBounds(mavros_goal, home_pose_, config_.bounds, &reason)) {
    ROS_WARN(
        "[BRIDGE] Goal rejected before planning: %s; goal=(%.2f, %.2f, %.2f) map, home=(%.2f, %.2f, %.2f) map, horizontal_distance=%.2f m.",
        reason.c_str(),
        mavros_goal.pose.position.x, mavros_goal.pose.position.y,
        mavros_goal.pose.position.z,
        home_pose_.pose.position.x, home_pose_.pose.position.y,
        home_pose_.pose.position.z,
        goal_horizontal_distance);
    return;
  }
  validated_goal_mavros_ = mavros_goal;
  have_valid_goal_ = true;
  trajectory_gate_.noteGoal(now);
  planning_goal.header.stamp = now;
  goal_publisher_.publish(planning_goal);
  ROS_INFO(
      "[BRIDGE] Validated goal forwarded to EGO-Planner: planning=(%.2f, %.2f, %.2f), horizontal_distance_from_home=%.2f m.",
      planning_goal.pose.position.x, planning_goal.pose.position.y,
      planning_goal.pose.position.z,
      goal_horizontal_distance);
}

bool EgoMavrosBridge::trackingService(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response) {
  if (!config_.enable_control) {
    response.success = false;
    response.message = "control is disabled (dry-run mode)";
    return true;
  }
  if (state_ == BridgeState::kLanding || state_ == BridgeState::kDone ||
      state_ == BridgeState::kError) {
    response.success = false;
    response.message = "tracking cannot change in the current bridge state";
    return true;
  }

  tracking_requested_ = request.data;
  response.success = true;
  response.message = request.data ? "tracking requested" : "tracking disabled";
  if (!request.data && (state_ == BridgeState::kTrackEgo ||
                        state_ == BridgeState::kHoverReady ||
                        state_ == BridgeState::kHold)) {
    supervised_hold_ = true;
    return_in_progress_ = false;
    latchHoldAtCurrentPose();
    if (state_ != BridgeState::kHold) {
      transitionTo(BridgeState::kHold,
                   "tracking disabled; latched safety HOLD");
    } else {
      publishState();
    }
  }
  return true;
}

bool EgoMavrosBridge::cancelCurrentTrajectoryService(
    std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
  if (!config_.enable_control) {
    response.success = false;
    response.message = "control is disabled (dry-run mode)";
    return true;
  }
  if (state_ == BridgeState::kLanding || state_ == BridgeState::kDone ||
      state_ == BridgeState::kError) {
    response.success = false;
    response.message = "cannot cancel trajectory in terminal state";
    return true;
  }
  have_valid_goal_ = false;
  have_planner_target_ = false;
  tracking_requested_ = false;
  return_in_progress_ = false;
  supervised_hold_ = true;
  trajectory_gate_.cancel();
  planning_cancel_publisher_.publish(std_msgs::Empty());
  startHold("current EGO trajectory cancelled by mission supervisor");
  response.success = true;
  response.message = "trajectory invalidated and HOLD requested";
  return true;
}

bool EgoMavrosBridge::resumeEgoService(
    std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
  if (!config_.enable_control) {
    response.success = false;
    response.message = "control is disabled (dry-run mode)";
    return true;
  }
  if (state_ != BridgeState::kHold && state_ != BridgeState::kHoverReady) {
    response.success = false;
    response.message = "resume requires HOLD or HOVER_READY";
    return true;
  }
  tracking_requested_ = true;
  // A mission-supervised recovery HOLD stays latched until the supervisor
  // explicitly requests return_home or land.  Resuming EGO is not permission
  // to turn an alignment/command failure into an automatic landing.
  if (state_ == BridgeState::kHold) {
    // Resume only re-opens HOVER_READY. TRACK_EGO still requires a fresh,
    // bounded PositionCommand and full preflight in the timer state machine;
    // allowing this transition before the new goal callback avoids a
    // cross-node service/topic scheduling race after CANCEL_CURRENT_TRAJECTORY.
    transitionTo(BridgeState::kHoverReady, "explicit EGO resume requested");
  }
  response.success = true;
  response.message = "EGO resume requested; preflight remains enforced";
  return true;
}

bool EgoMavrosBridge::landService(std_srvs::Trigger::Request&,
                                  std_srvs::Trigger::Response& response) {
  if (!config_.enable_control) {
    response.success = false;
    response.message = "control is disabled (dry-run mode)";
    return true;
  }
  if (state_ == BridgeState::kLanding) {
    response.success = true;
    response.message = "landing is already in progress";
    return true;
  }
  if (state_ == BridgeState::kDone || state_ == BridgeState::kError) {
    response.success = false;
    response.message = "landing cannot start in the current bridge state";
    return true;
  }
  if (!have_fcu_state_ || !fcu_state_.connected) {
    response.success = false;
    response.message = "FCU state is unavailable or disconnected";
    return true;
  }
  if (!fcu_state_.armed) {
    response.success = false;
    response.message = "vehicle is already disarmed";
    return true;
  }
  const ros::Time now = ros::Time::now();
  std::string preflight_reason;
  if (!baseInputsFresh(now, &preflight_reason)) {
    response.success = false;
    response.message =
        "AUTO.LAND preflight failed: " + preflight_reason;
    return true;
  }
  geometry_msgs::PoseStamped home_hover = home_pose_;
  home_hover.pose.position.z += config_.return_height;
  const double home_error =
      have_mavros_pose_
          ? positionDistance(mavros_pose_, home_hover)
          : std::numeric_limits<double>::infinity();
  const bool pose_fresh =
      have_mavros_pose_ && isFinitePose(mavros_pose_) &&
      isFresh(now, last_mavros_pose_time_, config_.mavros_pose_timeout,
              config_.timestamp_future_tolerance);
  const bool verified_home_hover =
      have_home_ && homeHoverAllowsAutoLand(
                        state_ == BridgeState::kHomeHover, true,
                        fcu_state_.armed, fcu_state_.mode == "OFFBOARD",
                        home_error, config_.return_tolerance);
  const bool emergency_supervised_hold =
      supervisedHoldAllowsEmergencyAutoLand(
          state_ == BridgeState::kHold, supervised_hold_, true,
          fcu_state_.armed, fcu_state_.mode == "OFFBOARD", pose_fresh);
  if (!verified_home_hover && !emergency_supervised_hold) {
    response.success = false;
    response.message =
        "AUTO.LAND requires verified HOME_HOVER or mission-supervised HOLD";
    return true;
  }
  if (!pose_fresh) {
    response.success = false;
    response.message =
        "cannot start supervised landing without a fresh, valid pose";
    return true;
  }
  tower_yaw_mode_ = false;
  supervised_hold_ = false;
  land_requested_ = true;
  response.success = true;
  response.message = emergency_supervised_hold
                         ? "emergency AUTO.LAND requested from supervised HOLD"
                         : "AUTO.LAND requested";
  return true;
}

bool EgoMavrosBridge::returnHomeService(
    std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
  if (!config_.enable_control || !have_home_) {
    response.success = false;
    response.message = "control or home pose is unavailable";
    return true;
  }
  if (state_ != BridgeState::kHoverReady &&
      state_ != BridgeState::kTrackEgo && state_ != BridgeState::kHold) {
    response.success = false;
    response.message = "return home is unavailable in the current state";
    return true;
  }

  tower_yaw_mode_ = false;
  std::string reason;
  if (!publishReturnGoal(&reason)) {
    response.success = false;
    response.message = reason;
    return true;
  }
  supervised_hold_ = false;
  if (state_ == BridgeState::kHold) {
    transitionTo(BridgeState::kHoverReady,
                 "return-home request released safety HOLD; preflight remains enforced");
  }
  response.success = true;
  response.message = "home goal sent through EGO-Planner";
  return true;
}

void EgoMavrosBridge::controlTimerCallback(const ros::TimerEvent&) {
  const ros::Time now = ros::Time::now();
  if (now.isZero()) {
    return;
  }
  if (!last_control_time_.isZero() &&
      now + ros::Duration(config_.timestamp_future_tolerance) <
          last_control_time_) {
    handleClockRollback(now);
    last_control_time_ = now;
    publishInputHealth(now);
    return;
  }
  last_control_time_ = now;
  publishInputHealth(now);
  // With /use_sim_time the constructor can run before the first /clock
  // sample. Anchor finite state timeouts on the first usable simulation time.
  if (state_entered_time_.isZero()) {
    state_entered_time_ = now;
  }

  if (state_ == BridgeState::kDryRun) {
    std::string reason;
    if (!baseInputsFresh(now, &reason)) {
      ROS_INFO_THROTTLE(2.0, "[DRY_RUN] Waiting for inputs: %s",
                        reason.c_str());
    } else if (!have_home_ && !captureHomeIfSafe(&reason)) {
      ROS_INFO_THROTTLE(2.0, "[DRY_RUN] Preflight blocked: %s",
                        reason.c_str());
    } else if (!fullPreflightValid(now, &reason)) {
      ROS_INFO_THROTTLE(2.0, "[DRY_RUN] Preflight blocked: %s",
                        reason.c_str());
    } else {
      ROS_INFO_THROTTLE(
          2.0,
          "[DRY_RUN] Full preflight healthy; no MAVROS control topic is advertised or published.");
    }
    return;
  }
  if (state_ == BridgeState::kDone || state_ == BridgeState::kError) {
    return;
  }

  std::string conflict;
  if (!controlAuthorityValid(now, &conflict)) {
    if (have_fcu_state_ && fcu_state_.armed) {
      ROS_FATAL("[BRIDGE] %s; requesting AUTO.LAND.", conflict.c_str());
      land_requested_ = true;
    } else {
      transitionTo(BridgeState::kError, conflict);
      return;
    }
  }

  if (land_requested_ && state_ != BridgeState::kLanding) {
    transitionTo(BridgeState::kLanding, "landing requested");
  }

  const bool state_requires_armed_vehicle =
      state_ == BridgeState::kTakeoff ||
      state_ == BridgeState::kHoverReady ||
      state_ == BridgeState::kTrackEgo || state_ == BridgeState::kHold ||
      state_ == BridgeState::kHomeHover;
  if (state_requires_armed_vehicle && have_fcu_state_ &&
      !fcu_state_.armed) {
    transitionTo(BridgeState::kError,
                 "vehicle disarmed unexpectedly before landing");
    return;
  }

  if (have_fcu_state_ && !fcu_state_.connected &&
      state_ != BridgeState::kWaitFcu && state_ != BridgeState::kLanding) {
    if (fcu_state_.armed) {
      startHold("FCU connection lost");
    } else {
      transitionTo(BridgeState::kWaitFcu, "FCU connection lost");
    }
  }

  switch (state_) {
    case BridgeState::kWaitFcu:
      if (have_fcu_state_ && fcu_state_.connected) {
        transitionTo(BridgeState::kWaitInputs, "FCU connected");
      } else if (now - state_entered_time_ >=
                 ros::Duration(config_.wait_fcu_timeout)) {
        transitionTo(BridgeState::kError, "FCU connection timeout");
      }
      break;

    case BridgeState::kWaitInputs: {
      std::string reason;
      if (baseInputsFresh(now, &reason) && !have_home_) {
        captureHomeIfSafe(&reason);
      }
      if (config_.require_start_permission && !start_permission_) {
        ROS_INFO_THROTTLE(
            2.0,
            "[WAIT_INPUTS] flight inputs monitored; waiting for swarm "
            "takeoff permission");
        state_entered_time_ = now;
        break;
      }
      if (flightPreflightValid(now, &reason)) {
        output_setpoint_ = home_pose_;
        hold_pose_ = home_pose_;
        have_output_setpoint_ = true;
        transitionTo(BridgeState::kPrestream,
                     "flight preflight passed; EGO goal is accepted after hover");
      } else if (now - state_entered_time_ >=
                 ros::Duration(config_.wait_inputs_timeout)) {
        transitionTo(BridgeState::kError,
                     "flight preflight timeout: " + reason);
      } else {
        ROS_INFO_THROTTLE(2.0, "[WAIT_INPUTS] %s", reason.c_str());
      }
      break;
    }

    case BridgeState::kPrestream: {
      std::string reason;
      if (!flightPreflightValid(now, &reason)) {
        transitionTo(BridgeState::kWaitInputs,
                     "preflight failed during prestream: " + reason);
        break;
      }
      publishSetpoint(home_pose_, now);
      if (now - state_entered_time_ >=
          ros::Duration(config_.prestream_duration)) {
        transitionTo(BridgeState::kArmOffboard, "prestream complete");
      }
      break;
    }

    case BridgeState::kArmOffboard: {
      std::string reason;
      if (!flightPreflightValid(now, &reason)) {
        if (fcu_state_.armed) {
          startHold("arming preflight failure: " + reason);
        } else {
          transitionTo(BridgeState::kWaitInputs,
                       "arming preflight failure: " + reason);
        }
        break;
      }
      publishSetpoint(home_pose_, now);
      if (now - state_entered_time_ >=
          ros::Duration(config_.arm_offboard_timeout)) {
        transitionTo(BridgeState::kError,
                     "OFFBOARD/arming confirmation timeout");
        break;
      }
      if (fcu_state_.mode != "OFFBOARD") {
        requestMode("OFFBOARD", now);
      } else if (!fcu_state_.armed) {
        requestArm(now);
      } else {
        transitionTo(BridgeState::kTakeoff, "OFFBOARD and armed confirmed");
      }
      break;
    }

    case BridgeState::kTakeoff: {
      std::string reason;
      if (!baseInputsFresh(now, &reason)) {
        startHold("takeoff input failure: " + reason);
        break;
      }
      if (now - state_entered_time_ >=
          ros::Duration(config_.takeoff_timeout)) {
        startHold("takeoff timeout");
        break;
      }
      if (fcu_state_.mode != "OFFBOARD") {
        hold_pose_ = mavros_pose_;
        requestMode("OFFBOARD", now);
        publishHold(now);
        break;
      }

      geometry_msgs::PoseStamped takeoff = home_pose_;
      takeoff.pose.position.z += config_.takeoff_height;
      publishSetpoint(takeoff, now);
      if (positionDistance(mavros_pose_, takeoff) <=
          config_.takeoff_tolerance) {
        if (!takeoff_inside_tolerance_) {
          takeoff_inside_tolerance_ = true;
          takeoff_inside_since_ = now;
        }
        if (now - takeoff_inside_since_ >=
            ros::Duration(config_.hover_duration)) {
          hold_pose_ = takeoff;
          transitionTo(BridgeState::kHoverReady,
                       "takeoff height reached and stabilized");
        }
      } else {
        takeoff_inside_tolerance_ = false;
        takeoff_inside_since_ = ros::Time(0);
      }
      break;
    }

    case BridgeState::kHoverReady: {
      std::string reason;
      publishSetpoint(hold_pose_, now);
      if (!baseInputsFresh(now, &reason)) {
        startHold("hover input failure: " + reason);
        break;
      }
      if (tracking_requested_ && commandFresh(now)) {
        if (fullPreflightValid(now, &reason)) {
          transitionTo(BridgeState::kTrackEgo,
                       "fresh bounded EGO command and aligned odometry");
        } else {
          ROS_ERROR_THROTTLE(1.0, "[HOVER_READY] %s", reason.c_str());
        }
      }
      break;
    }

    case BridgeState::kTrackEgo: {
      std::string reason;
      if (!baseInputsFresh(now, &reason)) {
        startHold("tracking input failure: " + reason);
        break;
      }
      if (!commandFresh(now) || !have_planner_target_) {
        startHold("planner command timeout");
        break;
      }
      double alignment_position_error = 0.0;
      double alignment_yaw_error = 0.0;
      if (!plannerAlignmentErrors(&alignment_position_error,
                                  &alignment_yaw_error, &reason)) {
        startHold("planner/MAVROS alignment failure: " + reason);
        break;
      }
      if (alignment_position_error >
          config_.alignment_position_tolerance) {
        std::ostringstream stream;
        stream << "planner/MAVROS alignment failure: alignment error position="
               << alignment_position_error << " m, yaw="
               << alignment_yaw_error << " rad";
        startHold(stream.str());
        break;
      }
      if (alignment_yaw_error > config_.alignment_yaw_tolerance) {
        if (!alignment_yaw_error_active_) {
          alignment_yaw_error_active_ = true;
          alignment_yaw_error_since_ = now;
          ROS_WARN("[BRIDGE] Transient planner/MAVROS yaw mismatch: %.6f rad; "
                   "waiting %.2f s before HOLD.",
                   alignment_yaw_error,
                   config_.alignment_yaw_error_duration);
        } else if (now - alignment_yaw_error_since_ >=
                   ros::Duration(config_.alignment_yaw_error_duration)) {
          std::ostringstream stream;
          stream << "planner/MAVROS alignment failure: persistent yaw error="
                 << alignment_yaw_error << " rad for "
                 << (now - alignment_yaw_error_since_).toSec() << " s";
          startHold(stream.str());
          break;
        }
      } else {
        alignment_yaw_error_active_ = false;
        alignment_yaw_error_since_ = ros::Time(0);
      }
      if (fcu_state_.mode != "OFFBOARD") {
        requestMode("OFFBOARD", now);
        startHold("OFFBOARD mode lost");
        break;
      }

      const double tracking_error =
          positionDistance(mavros_pose_, planner_target_mavros_);
      maximum_tracking_error_ =
          std::max(maximum_tracking_error_, tracking_error);
      std_msgs::Float64 tracking_message;
      tracking_message.data = tracking_error;
      tracking_error_publisher_.publish(tracking_message);
      if (tracking_error > config_.tracking_error_limit) {
        if (!tracking_error_active_) {
          tracking_error_active_ = true;
          tracking_error_since_ = now;
        } else if (now - tracking_error_since_ >=
                   ros::Duration(config_.tracking_error_duration)) {
          std::ostringstream stream;
          stream << "tracking error " << tracking_error
                 << " m exceeds limit " << config_.tracking_error_limit
                 << " m";
          startHold(stream.str());
          break;
        }
      } else {
        tracking_error_active_ = false;
        tracking_error_since_ = ros::Time(0);
      }

      publishTrajectorySetpoint(now);
      updateReturnProgress(now);
      break;
    }

    case BridgeState::kHold: {
      publishHold(now);
      // HOLD is a latched safety state. A transiently fresh sample must not
      // silently resume a failed trajectory; the mission supervisor requests
      // landing, and the bridge independently lands after the finite timeout.
      if (shouldAutoLandFromHold(
              supervised_hold_, (now - state_entered_time_).toSec(),
              config_.land_after_loss)) {
        land_requested_ = true;
        transitionTo(BridgeState::kLanding,
                     "hold timeout: " + hold_reason_);
      } else if (fcu_state_.connected && fcu_state_.armed &&
                 fcu_state_.mode != "OFFBOARD") {
        requestMode("OFFBOARD", now);
      }
      if (supervised_hold_) {
        ROS_INFO_THROTTLE(
            2.0,
            "[BRIDGE] Mission-supervised recovery HOLD remains latched; automatic loss landing is suppressed.");
      }
      break;
    }

    case BridgeState::kHomeHover: {
      publishHold(now);
      std::string reason;
      if (!baseInputsFresh(now, &reason)) {
        startHold("home hover input failure: " + reason);
      } else if (fcu_state_.mode != "OFFBOARD") {
        requestMode("OFFBOARD", now);
      }
      break;
    }

    case BridgeState::kLanding: {
      if (fcu_state_.mode != "AUTO.LAND") {
        requestMode("AUTO.LAND", now);
      }

      const bool extended_fresh =
          have_extended_state_ &&
          isFresh(now, last_extended_state_time_,
                  config_.extended_state_timeout,
                  config_.timestamp_future_tolerance);
      if (!fcu_state_.armed && extended_fresh &&
          extended_state_.landed_state ==
              mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
        transitionTo(BridgeState::kDone,
                     "PX4 reports disarmed and on ground");
      } else if (now - state_entered_time_ >=
                 ros::Duration(config_.landing_timeout)) {
        transitionTo(BridgeState::kError, "landing timeout");
      }
      break;
    }

    case BridgeState::kDryRun:
      if (!have_home_) {
        std::string reason;
        if (baseInputsFresh(now, &reason)) {
          captureHomeIfSafe(&reason);
        }
      }
      break;
    case BridgeState::kDone:
    case BridgeState::kError:
      break;
  }
}

void EgoMavrosBridge::transitionTo(BridgeState next_state,
                                   const std::string& reason) {
  if (state_ == next_state) {
    publishState();
    return;
  }
  ROS_WARN("[BRIDGE] %s -> %s: %s", bridgeStateName(state_),
           bridgeStateName(next_state), reason.c_str());
  if (next_state == BridgeState::kTrackEgo) {
    tracking_error_active_ = false;
    tracking_error_since_ = ros::Time(0);
    alignment_yaw_error_active_ = false;
    alignment_yaw_error_since_ = ros::Time(0);
  }
  if (next_state == BridgeState::kHoverReady) {
    // A new trajectory or yaw policy must always start from the measured
    // vehicle heading. This also isolates cancelled trajectory generations.
    have_effective_yaw_ = false;
    last_effective_yaw_time_ = ros::Time(0);
  }
  if (next_state == BridgeState::kLanding) {
    // AUTO.LAND is the sole controller from this transition onward. Invalidate
    // both the planner and bridge command generations before requesting the
    // mode, and never publish an OFFBOARD hold/setpoint in kLanding.
    have_valid_goal_ = false;
    have_planner_target_ = false;
    tracking_requested_ = false;
    return_in_progress_ = false;
    return_inside_tolerance_ = false;
    tower_yaw_mode_ = false;
    supervised_hold_ = false;
    trajectory_gate_.cancel();
    planning_cancel_publisher_.publish(std_msgs::Empty());
    ROS_WARN("[BRIDGE] EGO trajectory cleared; AUTO.LAND has exclusive "
             "control and OFFBOARD setpoint publication is stopped.");
  }
  if (next_state == BridgeState::kDone || next_state == BridgeState::kError) {
    ROS_WARN("[BRIDGE] Terminal maximum tracking error: %.3f m",
             maximum_tracking_error_);
  }
  state_ = next_state;
  state_entered_time_ = ros::Time::now();
  publishState();
}

void EgoMavrosBridge::publishState() {
  if (!state_publisher_) {
    return;
  }
  std_msgs::String message;
  message.data = bridgeStateName(state_);
  state_publisher_.publish(message);
}

void EgoMavrosBridge::publishInputHealth(const ros::Time& now) {
  if (!input_health_publisher_ || now.isZero()) return;
  if (!last_input_health_publish_time_.isZero() &&
      now >= last_input_health_publish_time_ &&
      now - last_input_health_publish_time_ < ros::Duration(0.1)) {
    return;
  }
  last_input_health_publish_time_ = now;
  const auto age = [&now](const ros::Time& stamp) {
    return stamp.isZero() ? std::numeric_limits<double>::quiet_NaN()
                          : (now - stamp).toSec();
  };
  const auto append_age = [](std::ostringstream* stream, const char* name,
                             double value) {
    *stream << '\"' << name << "\":";
    if (std::isfinite(value)) {
      *stream << value;
    } else {
      *stream << "null";
    }
  };

  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(6);
  stream << "{\"now\":" << now.toSec()
         << ",\"clock_epoch\":" << clock_epoch_
         << ",\"bridge_state\":\"" << bridgeStateName(state_) << "\""
         << ",\"fcu_have\":" << (have_fcu_state_ ? "true" : "false")
         << ',';
  append_age(&stream, "fcu_receive_age", age(last_fcu_state_time_));
  stream << ",\"extended_have\":"
         << (have_extended_state_ ? "true" : "false") << ',';
  append_age(&stream, "extended_receive_age", age(last_extended_state_time_));
  stream << ",\"mavros_pose_have\":"
         << (have_mavros_pose_ ? "true" : "false") << ',';
  append_age(&stream, "mavros_pose_receive_age", age(last_mavros_pose_time_));
  stream << ',';
  append_age(&stream, "mavros_pose_stamp_age", age(last_mavros_pose_stamp_));
  stream << ",\"mavros_pose_fresh\":"
         << (have_mavros_pose_ &&
                     isStampedFresh(now, last_mavros_pose_time_,
                                    last_mavros_pose_stamp_,
                                    config_.mavros_pose_timeout,
                                    config_.timestamp_future_tolerance)
                 ? "true" : "false")
         << ",\"planner_odom_have\":"
         << (have_planner_odom_ ? "true" : "false") << ',';
  append_age(&stream, "planner_odom_receive_age", age(last_planner_odom_time_));
  stream << ',';
  append_age(&stream, "planner_odom_stamp_age", age(last_planner_odom_stamp_));
  stream << ",\"planner_odom_fresh\":"
         << (have_planner_odom_ &&
                     isStampedFresh(now, last_planner_odom_time_,
                                    last_planner_odom_stamp_,
                                    config_.planner_odom_timeout,
                                    config_.timestamp_future_tolerance)
                 ? "true" : "false")
         << ",\"cloud_have\":" << (have_cloud_ ? "true" : "false")
         << ',';
  append_age(&stream, "cloud_receive_age", age(last_cloud_time_));
  stream << ',';
  append_age(&stream, "cloud_stamp_age", age(last_cloud_stamp_));
  stream << ",\"cloud_fresh\":"
         << (have_cloud_ &&
                     isStampedFresh(now, last_cloud_time_, last_cloud_stamp_,
                                    config_.cloud_timeout,
                                    config_.timestamp_future_tolerance)
                 ? "true" : "false")
         << ",\"command_have\":"
         << (have_planner_target_ ? "true" : "false") << ',';
  append_age(&stream, "command_receive_age", age(last_command_time_));
  stream << ',';
  append_age(&stream, "command_stamp_age", age(last_command_stamp_));
  stream << ",\"command_fresh\":"
         << (commandFresh(now) ? "true" : "false") << '}';
  std_msgs::String message;
  message.data = stream.str();
  input_health_publisher_.publish(message);
}

void EgoMavrosBridge::handleClockRollback(const ros::Time& now) {
  const double rollback = (last_control_time_ - now).toSec();
  ++clock_epoch_;
  ROS_ERROR("[BRIDGE_TIME] ROS clock rollback detected: previous=%.6f "
            "now=%.6f rollback=%.6fs epoch=%llu; invalidating all timed "
            "inputs and waiting for new samples.",
            last_control_time_.toSec(), now.toSec(), rollback,
            static_cast<unsigned long long>(clock_epoch_));

  if (have_mavros_pose_ && isFinitePose(mavros_pose_)) {
    latchHoldAtCurrentPose();
  }
  have_fcu_state_ = false;
  have_extended_state_ = false;
  have_mavros_pose_ = false;
  have_planner_odom_ = false;
  have_cloud_ = false;
  have_planner_target_ = false;
  have_valid_goal_ = false;
  have_orbit_speed_scale_ = false;
  last_fcu_state_time_ = ros::Time(0);
  last_extended_state_time_ = ros::Time(0);
  last_mavros_pose_time_ = ros::Time(0);
  last_mavros_pose_stamp_ = ros::Time(0);
  last_planner_odom_time_ = ros::Time(0);
  last_planner_odom_stamp_ = ros::Time(0);
  last_cloud_time_ = ros::Time(0);
  last_cloud_stamp_ = ros::Time(0);
  last_command_time_ = ros::Time(0);
  last_command_stamp_ = ros::Time(0);
  last_orbit_speed_scale_time_ = ros::Time(0);
  last_input_health_publish_time_ = ros::Time(0);
  trajectory_gate_.cancel();
  planning_cancel_publisher_.publish(std_msgs::Empty());

  const bool airborne_state =
      state_ == BridgeState::kTakeoff || state_ == BridgeState::kHoverReady ||
      state_ == BridgeState::kTrackEgo || state_ == BridgeState::kHold ||
      state_ == BridgeState::kHomeHover;
  if (airborne_state) {
    startHold("ROS clock epoch changed; timed inputs invalidated");
  } else if (state_ != BridgeState::kDryRun &&
             state_ != BridgeState::kLanding &&
             state_ != BridgeState::kDone && state_ != BridgeState::kError) {
    transitionTo(BridgeState::kWaitInputs,
                 "ROS clock epoch changed; waiting for fresh inputs");
  }
}

void EgoMavrosBridge::publishSetpoint(
    const geometry_msgs::PoseStamped& desired, const ros::Time& now) {
  if (!config_.enable_control || !setpoint_publisher_) {
    return;
  }
  if (!isFinitePose(desired)) {
    ROS_ERROR_THROTTLE(1.0,
                       "[BRIDGE] Refusing to publish an invalid setpoint.");
    return;
  }

  if (!have_output_setpoint_) {
    output_setpoint_ = desired;
    have_output_setpoint_ = true;
  } else {
    const double control_period = 1.0 / config_.publish_rate;
    output_setpoint_ = stepToward(output_setpoint_, desired,
                                  config_.max_position_rate * control_period,
                                  config_.max_yaw_rate * control_period);
  }
  output_setpoint_.header.stamp = now;
  output_setpoint_.header.frame_id = config_.mavros_frame;
  setpoint_publisher_.publish(poseToRawTarget(output_setpoint_, now));
}

void EgoMavrosBridge::publishTrajectorySetpoint(const ros::Time& now) {
  if (!config_.enable_control || !setpoint_publisher_ ||
      !have_planner_target_) {
    return;
  }
  if (config_.require_orbit_speed_scale && orbit_speed_scale_ <= 1.0e-3) {
    if (!orbit_speed_scale_hold_active_) {
      latchHoldAtCurrentPose();
      orbit_speed_scale_hold_active_ = true;
      ROS_WARN("[BRIDGE] Orbit speed scale reached zero; current pose "
               "latched until the coordinator permits forward recovery.");
    }
    publishHold(now);
    return;
  }
  orbit_speed_scale_hold_active_ = false;
  mavros_msgs::PositionTarget target = planner_raw_target_;
  target.header.stamp = now;
  target.header.frame_id = config_.mavros_frame;
  setpoint_publisher_.publish(target);
}

void EgoMavrosBridge::publishHold(const ros::Time& now) {
  if (!isFinitePose(hold_pose_)) {
    if (have_mavros_pose_ && isFinitePose(mavros_pose_)) {
      latchHoldAtCurrentPose();
    } else {
      ROS_ERROR_THROTTLE(
          1.0, "[BRIDGE] Cannot publish HOLD without a valid vehicle pose.");
      return;
    }
  }
  publishSetpoint(hold_pose_, now);
}

void EgoMavrosBridge::latchHoldAtCurrentPose() {
  const geometry_msgs::PoseStamped pose =
      have_mavros_pose_ ? mavros_pose_ : output_setpoint_;
  const HoldSetpointLatch latch = makeHoldSetpointLatch(pose);
  hold_pose_ = latch.hold_pose;
  hold_pose_.header.frame_id = config_.mavros_frame;
  output_setpoint_ = latch.output_setpoint;
  output_setpoint_.header.frame_id = config_.mavros_frame;
  have_output_setpoint_ = latch.have_output_setpoint;
}

bool EgoMavrosBridge::baseInputsFresh(const ros::Time& now,
                                      std::string* reason) const {
  if (!have_fcu_state_ || !fcu_state_.connected ||
      !isFresh(now, last_fcu_state_time_, config_.fcu_state_timeout,
               config_.timestamp_future_tolerance)) {
    *reason = "FCU state is unavailable, disconnected or stale";
    return false;
  }
  if (!have_extended_state_ ||
      !isFresh(now, last_extended_state_time_,
               config_.extended_state_timeout,
               config_.timestamp_future_tolerance)) {
    *reason = "PX4 extended state is unavailable or stale";
    return false;
  }
  if (!have_mavros_pose_ ||
      !isStampedFresh(now, last_mavros_pose_time_,
                      last_mavros_pose_stamp_,
                      config_.mavros_pose_timeout,
                      config_.timestamp_future_tolerance)) {
    *reason =
        "MAVROS local pose is unavailable, stale or has an unusable timestamp";
    return false;
  }
  if (!have_planner_odom_ ||
      !isStampedFresh(now, last_planner_odom_time_,
                      last_planner_odom_stamp_,
                      config_.planner_odom_timeout,
                      config_.timestamp_future_tolerance)) {
    *reason =
        "planner odometry is unavailable, stale or has an unusable timestamp";
    return false;
  }
  if (!have_cloud_ ||
      !isStampedFresh(now, last_cloud_time_, last_cloud_stamp_,
                      config_.cloud_timeout,
                      config_.timestamp_future_tolerance)) {
    *reason =
        "obstacle point cloud is unavailable, stale or has an unusable timestamp";
    return false;
  }
  if (config_.require_orbit_speed_scale &&
      (!have_orbit_speed_scale_ ||
       !isFresh(now, last_orbit_speed_scale_time_,
                config_.orbit_speed_scale_timeout,
                config_.timestamp_future_tolerance))) {
    *reason = "orbit speed scale is unavailable, invalid or stale";
    return false;
  }
  reason->clear();
  return true;
}

bool EgoMavrosBridge::commandFresh(const ros::Time& now) const {
  return have_planner_target_ &&
         isFresh(now, last_command_time_, config_.command_timeout,
                 config_.timestamp_future_tolerance) &&
         isTimestampUsable(now, last_command_stamp_,
                           config_.command_timeout,
                           config_.timestamp_future_tolerance);
}

bool EgoMavrosBridge::plannerAlignmentErrors(
    double* position_error, double* yaw_error,
    std::string* reason) const {
  if (position_error == nullptr || yaw_error == nullptr ||
      reason == nullptr) {
    return false;
  }
  if (!have_planner_odom_ || !have_mavros_pose_) {
    *reason = "planner or MAVROS pose is unavailable";
    return false;
  }

  geometry_msgs::PoseStamped planner_pose_mavros;
  if (!transformToMavros(odometryPose(planner_odom_),
                         &planner_pose_mavros, reason)) {
    return false;
  }

  *position_error = positionDistance(planner_pose_mavros, mavros_pose_);
  *yaw_error = std::abs(angularDistance(
      yawFromQuaternion(planner_pose_mavros.pose.orientation),
      yawFromQuaternion(mavros_pose_.pose.orientation)));
  reason->clear();
  return true;
}

bool EgoMavrosBridge::plannerAlignmentValid(std::string* reason) const {
  double position_error = 0.0;
  double yaw_error = 0.0;
  if (!plannerAlignmentErrors(&position_error, &yaw_error, reason)) {
    return false;
  }
  if (position_error > config_.alignment_position_tolerance ||
      yaw_error > config_.alignment_yaw_tolerance) {
    std::ostringstream stream;
    stream << "alignment error position=" << position_error
           << " m, yaw=" << yaw_error << " rad";
    *reason = stream.str();
    return false;
  }
  reason->clear();
  return true;
}

bool EgoMavrosBridge::captureHomeIfSafe(std::string* reason) {
  if (have_home_) {
    reason->clear();
    return true;
  }
  const bool confirmed_on_ground =
      have_fcu_state_ && have_extended_state_ && !fcu_state_.armed &&
      extended_state_.landed_state ==
          mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
  if (!confirmed_on_ground) {
    *reason = "home capture requires PX4 disarmed and ON_GROUND";
    return false;
  }
  if (!have_mavros_pose_ || !isFinitePose(mavros_pose_)) {
    *reason = "home capture requires a finite MAVROS pose";
    return false;
  }

  home_pose_ = mavros_pose_;
  home_pose_.header.frame_id = config_.mavros_frame;
  hold_pose_ = home_pose_;
  output_setpoint_ = home_pose_;
  have_home_ = true;
  have_output_setpoint_ = true;
  ROS_INFO("[BRIDGE] Home captured at (%.3f, %.3f, %.3f).",
           home_pose_.pose.position.x, home_pose_.pose.position.y,
           home_pose_.pose.position.z);
  reason->clear();
  return true;
}

bool EgoMavrosBridge::flightPreflightValid(const ros::Time& now,
                                           std::string* reason) {
  if (!baseInputsFresh(now, reason)) {
    return false;
  }
  if (!have_home_) {
    *reason = "home pose has not been captured";
    return false;
  }
  if (!plannerAlignmentValid(reason)) {
    *reason = "planner/MAVROS alignment or TF invalid: " + *reason;
    return false;
  }
  if (!controlAuthorityValid(now, reason)) {
    return false;
  }
  reason->clear();
  return true;
}

bool EgoMavrosBridge::fullPreflightValid(const ros::Time& now,
                                         std::string* reason) {
  if (!flightPreflightValid(now, reason)) {
    return false;
  }
  if (!have_valid_goal_) {
    *reason = "no validated goal has been received";
    return false;
  }
  if (!isWithinBounds(validated_goal_mavros_, home_pose_, config_.bounds,
                      reason)) {
    *reason = "validated goal is outside the configured envelope: " +
              *reason;
    return false;
  }
  if (!commandFresh(now)) {
    *reason =
        "planner command is unavailable, stale or has an unusable timestamp";
    return false;
  }
  if (!isWithinBounds(planner_target_mavros_, home_pose_, config_.bounds,
                      reason)) {
    *reason = "planner command is outside the configured envelope: " +
              *reason;
    return false;
  }
  reason->clear();
  return true;
}

bool EgoMavrosBridge::planningToMavrosTransform(
    const ros::Time& stamp, geometry_msgs::TransformStamped* transform,
    std::string* reason) const {
  if (transform == nullptr || reason == nullptr) {
    return false;
  }
  if (config_.planning_frame == config_.mavros_frame) {
    transform->header.stamp = stamp;
    transform->header.frame_id = config_.mavros_frame;
    transform->child_frame_id = config_.planning_frame;
    transform->transform.rotation.w = 1.0;
    reason->clear();
    return true;
  }
  try {
    *transform = tf_buffer_.lookupTransform(
        config_.mavros_frame, config_.planning_frame, stamp,
        ros::Duration(kTfTimeout));
    reason->clear();
    return true;
  } catch (const tf2::TransformException& exception) {
    *reason = exception.what();
    return false;
  }
}

bool EgoMavrosBridge::transformToMavros(
    const geometry_msgs::PoseStamped& input,
    geometry_msgs::PoseStamped* output, std::string* reason) const {
  if (input.header.frame_id.empty()) {
    *reason = "input pose has an empty frame_id";
    return false;
  }
  if (input.header.frame_id == config_.mavros_frame) {
    *output = input;
    output->header.frame_id = config_.mavros_frame;
    return true;
  }

  try {
    const auto transform = tf_buffer_.lookupTransform(
        config_.mavros_frame, input.header.frame_id, input.header.stamp,
        ros::Duration(kTfTimeout));
    *output = transformPose(input, transform);
    return isFinitePose(*output);
  } catch (const tf2::TransformException& exception) {
    *reason = exception.what();
    return false;
  }
}

bool EgoMavrosBridge::transformToPlanning(
    const geometry_msgs::PoseStamped& input,
    geometry_msgs::PoseStamped* output, std::string* reason) const {
  if (input.header.frame_id.empty()) {
    *reason = "input pose has an empty frame_id";
    return false;
  }
  if (input.header.frame_id == config_.planning_frame) {
    *output = input;
    output->header.frame_id = config_.planning_frame;
    return true;
  }

  try {
    const auto transform = tf_buffer_.lookupTransform(
        config_.planning_frame, input.header.frame_id, input.header.stamp,
        ros::Duration(kTfTimeout));
    *output = transformPose(input, transform);
    return isFinitePose(*output);
  } catch (const tf2::TransformException& exception) {
    *reason = exception.what();
    return false;
  }
}

bool EgoMavrosBridge::hasControlConflict(std::string* detail) const {
  XmlRpc::XmlRpcValue arguments;
  XmlRpc::XmlRpcValue response;
  XmlRpc::XmlRpcValue payload;
  arguments[0] = ros::this_node::getName();
  if (!ros::master::execute("getSystemState", arguments, response, payload,
                            false) || payload.getType() !=
                                          XmlRpc::XmlRpcValue::TypeArray ||
      payload.size() < 1) {
    *detail = "unable to query ROS master for control authority";
    return true;
  }

  const XmlRpc::XmlRpcValue& publishers = payload[0];
  std::vector<TopicPublishers> publisher_state;
  for (int index = 0; index < publishers.size(); ++index) {
    if (publishers[index].getType() != XmlRpc::XmlRpcValue::TypeArray ||
        publishers[index].size() < 2 ||
        publishers[index][0].getType() !=
            XmlRpc::XmlRpcValue::TypeString ||
        publishers[index][1].getType() !=
            XmlRpc::XmlRpcValue::TypeArray) {
      continue;
    }
    TopicPublishers entry;
    entry.topic = static_cast<std::string>(publishers[index][0]);
    const XmlRpc::XmlRpcValue& nodes = publishers[index][1];
    for (int node_index = 0; node_index < nodes.size(); ++node_index) {
      if (nodes[node_index].getType() ==
          XmlRpc::XmlRpcValue::TypeString) {
        entry.nodes.push_back(
            static_cast<std::string>(nodes[node_index]));
      }
    }
    publisher_state.push_back(entry);
  }

  ControlConflict conflict;
  if (findControlConflict(monitoredControlTopics(), publisher_state,
                          ros::this_node::getName(), &conflict)) {
    *detail = "MAVROS control authority conflict: category=" +
              conflict.category + ", topic=" + conflict.topic +
              ", publisher=" + conflict.node;
    return true;
  }
  detail->clear();
  return false;
}

bool EgoMavrosBridge::controlAuthorityValid(const ros::Time& now,
                                            std::string* detail) {
  if (last_authority_check_time_.isZero() || now < last_authority_check_time_ ||
      now - last_authority_check_time_ >=
          ros::Duration(kAuthorityCheckInterval)) {
    cached_control_conflict_ =
        hasControlConflict(&cached_control_conflict_detail_);
    last_authority_check_time_ = now;
  }
  if (cached_control_conflict_) {
    *detail = cached_control_conflict_detail_;
    return false;
  }
  detail->clear();
  return true;
}

std::vector<MonitoredControlTopic>
EgoMavrosBridge::monitoredControlTopics() const {
  std::vector<MonitoredControlTopic> result;
  const auto add_topics =
      [this, &result](const std::string& category,
                      const std::vector<std::string>& topics) {
        for (const auto& topic : topics) {
          const MonitoredControlTopic candidate{
              category, node_handle_.resolveName(topic)};
          const bool duplicate = std::any_of(
              result.begin(), result.end(),
              [&candidate](const MonitoredControlTopic& existing) {
                return existing.topic == candidate.topic;
              });
          if (!duplicate) {
            result.push_back(candidate);
          }
        }
      };

  add_topics("position", config_.position_control_topics);
  std::vector<std::string> raw_local_topics =
      config_.raw_local_control_topics;
  raw_local_topics.push_back(config_.setpoint_topic);
  add_topics("raw local", raw_local_topics);
  add_topics("velocity", config_.velocity_control_topics);
  add_topics("attitude", config_.attitude_control_topics);
  add_topics("thrust", config_.thrust_control_topics);
  return result;
}

bool EgoMavrosBridge::requestMode(const std::string& mode,
                                  const ros::Time& now) {
  if (!last_mode_request_time_.isZero() &&
      now - last_mode_request_time_ <
          ros::Duration(config_.request_interval)) {
    return false;
  }
  mavros_msgs::SetMode request;
  request.request.custom_mode = mode;
  const bool accepted = set_mode_client_.call(request) &&
                        request.response.mode_sent;
  if (accepted) {
    ROS_INFO("[BRIDGE] Mode request accepted: %s", mode.c_str());
  } else {
    ROS_WARN("[BRIDGE] Mode request failed: %s", mode.c_str());
  }
  last_mode_request_time_ = now;
  return accepted;
}

bool EgoMavrosBridge::requestArm(const ros::Time& now) {
  if (!last_arm_request_time_.isZero() &&
      now - last_arm_request_time_ <
          ros::Duration(config_.request_interval)) {
    return false;
  }
  mavros_msgs::CommandBool request;
  request.request.value = true;
  const bool accepted = arming_client_.call(request) &&
                        request.response.success;
  if (accepted) {
    ROS_INFO("[BRIDGE] Arm request accepted.");
  } else {
    ROS_WARN("[BRIDGE] Arm request failed.");
  }
  last_arm_request_time_ = now;
  return accepted;
}

void EgoMavrosBridge::startHold(const std::string& reason) {
  if (state_ == BridgeState::kLanding || state_ == BridgeState::kDone ||
      state_ == BridgeState::kError) {
    return;
  }
  latchHoldAtCurrentPose();
  hold_reason_ = reason;
  transitionTo(BridgeState::kHold, reason);
}

bool EgoMavrosBridge::publishReturnGoal(std::string* reason) {
  const ros::Time now = ros::Time::now();
  if (!have_planner_odom_ ||
      !isFresh(now, last_planner_odom_time_, config_.planner_odom_timeout,
               config_.timestamp_future_tolerance) ||
      !have_cloud_ ||
      !isFresh(now, last_cloud_time_, config_.cloud_timeout,
               config_.timestamp_future_tolerance)) {
    *reason = "planner odometry or cloud is stale";
    return false;
  }

  geometry_msgs::PoseStamped home_hover = home_pose_;
  home_hover.header.stamp = now;
  home_hover.header.frame_id = config_.mavros_frame;
  home_hover.pose.position.z += config_.return_height;

  geometry_msgs::PoseStamped planning_goal;
  if (!transformToPlanning(home_hover, &planning_goal, reason)) {
    return false;
  }
  planning_goal.header.stamp = now;
  geometry_msgs::PoseStamped mavros_goal;
  if (!transformToMavros(planning_goal, &mavros_goal, reason)) {
    return false;
  }
  validated_goal_mavros_ = mavros_goal;
  have_valid_goal_ = true;
  trajectory_gate_.noteGoal(now);
  goal_publisher_.publish(planning_goal);
  return_in_progress_ = true;
  return_inside_tolerance_ = false;
  tracking_requested_ = true;
  ROS_INFO("[BRIDGE] Home goal published through EGO-Planner.");
  return true;
}

void EgoMavrosBridge::updateReturnProgress(const ros::Time& now) {
  if (!return_in_progress_ || !have_mavros_pose_) {
    return;
  }
  geometry_msgs::PoseStamped home_hover = home_pose_;
  home_hover.pose.position.z += config_.return_height;
  const bool inside =
      positionDistance(mavros_pose_, home_hover) <= config_.return_tolerance;
  if (inside && !return_inside_tolerance_) {
    return_inside_tolerance_ = true;
    return_inside_since_ = now;
  } else if (!inside) {
    return_inside_tolerance_ = false;
    return_inside_since_ = ros::Time(0);
  }

  if (return_inside_tolerance_ &&
      now - return_inside_since_ >=
          ros::Duration(config_.return_hold_duration)) {
    return_in_progress_ = false;
    tracking_requested_ = false;
    latchHoldAtCurrentPose();
    transitionTo(BridgeState::kHomeHover,
                 "home hover reached; waiting for supervised AUTO.LAND request");
    ROS_INFO("[BRIDGE] Home reached; AUTO.LAND remains gated by mission checks.");
  }
}

}  // namespace ego_gazebo_bridge
