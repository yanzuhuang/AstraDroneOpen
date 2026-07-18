#include "ego_gazebo_bridge/ego_mavros_bridge.h"

#include <ros/master.h>
#include <ros/this_node.h>
#include <tf2/exceptions.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <utility>
#include <vector>

namespace ego_gazebo_bridge {
namespace {

constexpr double kAuthorityCheckInterval = 1.0;
constexpr double kTfTimeout = 0.05;

bool isFresh(const ros::Time& now, const ros::Time& received,
             double timeout) {
  return !received.isZero() && now >= received &&
         (now - received) <= ros::Duration(timeout);
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

  setpoint_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>(
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
  private_node_handle_.param("publish_rate", config_.publish_rate,
                             config_.publish_rate);
  private_node_handle_.param("prestream_duration",
                             config_.prestream_duration,
                             config_.prestream_duration);
  private_node_handle_.param("request_interval", config_.request_interval,
                             config_.request_interval);
  private_node_handle_.param("takeoff_height", config_.takeoff_height,
                             config_.takeoff_height);
  private_node_handle_.param("takeoff_tolerance",
                             config_.takeoff_tolerance,
                             config_.takeoff_tolerance);
  private_node_handle_.param("hover_duration", config_.hover_duration,
                             config_.hover_duration);
  private_node_handle_.param("command_timeout", config_.command_timeout,
                             config_.command_timeout);
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
  private_node_handle_.param("max_position_rate", config_.max_position_rate,
                             config_.max_position_rate);
  private_node_handle_.param("max_yaw_rate", config_.max_yaw_rate,
                             config_.max_yaw_rate);
  private_node_handle_.param("alignment_position_tolerance",
                             config_.alignment_position_tolerance,
                             config_.alignment_position_tolerance);
  private_node_handle_.param("alignment_yaw_tolerance",
                             config_.alignment_yaw_tolerance,
                             config_.alignment_yaw_tolerance);
  private_node_handle_.param("return_tolerance", config_.return_tolerance,
                             config_.return_tolerance);
  private_node_handle_.param("return_hold_duration",
                             config_.return_hold_duration,
                             config_.return_hold_duration);
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
}

bool EgoMavrosBridge::validateConfig() const {
  const bool valid_positive_values =
      config_.publish_rate >= 20.0 && config_.prestream_duration > 0.0 &&
      config_.request_interval > 0.0 && config_.takeoff_height > 0.0 &&
      config_.takeoff_tolerance > 0.0 && config_.hover_duration >= 0.0 &&
      config_.command_timeout > 0.0 && config_.fcu_state_timeout > 0.0 &&
      config_.extended_state_timeout > 0.0 &&
      config_.mavros_pose_timeout > 0.0 &&
      config_.planner_odom_timeout > 0.0 && config_.cloud_timeout > 0.0 &&
      config_.land_after_loss > config_.command_timeout &&
      config_.max_position_rate > 0.0 && config_.max_yaw_rate > 0.0 &&
      config_.alignment_position_tolerance > 0.0 &&
      config_.alignment_yaw_tolerance > 0.0 &&
      config_.return_tolerance > 0.0 &&
      config_.return_hold_duration >= 0.0 &&
      config_.bounds.min_relative_height >= 0.0 &&
      config_.bounds.max_relative_height >
          config_.bounds.min_relative_height &&
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
      config_.input_goal_topic != config_.planner_goal_topic;

  if (!valid_positive_values) {
    ROS_FATAL("[BRIDGE] Invalid numeric safety parameter.");
  }
  if (!valid_names) {
    ROS_FATAL("[BRIDGE] Topic, service and frame names must not be empty.");
  }
  return valid_positive_values && valid_names;
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
          config_.command_topic, 10,
          &EgoMavrosBridge::commandCallback, this);
  goal_subscriber_ = node_handle_.subscribe<geometry_msgs::PoseStamped>(
      config_.input_goal_topic, 10, &EgoMavrosBridge::goalCallback, this);

  debug_setpoint_publisher_ =
      private_node_handle_.advertise<geometry_msgs::PoseStamped>(
          "debug_setpoint", 10);
  state_publisher_ = private_node_handle_.advertise<std_msgs::String>(
      "state", 1, true);
  goal_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>(
      config_.planner_goal_topic, 1, false);

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

void EgoMavrosBridge::mavrosPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& message) {
  if (!isFinitePose(*message)) {
    ROS_ERROR_THROTTLE(1.0, "[BRIDGE] Rejected invalid MAVROS pose.");
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
}

void EgoMavrosBridge::plannerOdomCallback(
    const nav_msgs::Odometry::ConstPtr& message) {
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
}

void EgoMavrosBridge::cloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& message) {
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
  have_cloud_ = true;
  last_cloud_time_ = ros::Time::now();
}

void EgoMavrosBridge::commandCallback(
    const quadrotor_msgs::PositionCommand::ConstPtr& message) {
  if (message->trajectory_flag !=
          quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY ||
      !isFinitePositionCommand(*message)) {
    ROS_ERROR_THROTTLE(1.0,
                       "[BRIDGE] Rejected invalid/non-ready PositionCommand.");
    return;
  }

  geometry_msgs::PoseStamped planning_pose =
      commandToPose(*message, config_.planning_frame);
  geometry_msgs::PoseStamped mavros_target;
  std::string reason;
  if (!transformToMavros(planning_pose, &mavros_target, &reason)) {
    ROS_ERROR_THROTTLE(1.0, "[BRIDGE] Command transform failed: %s",
                       reason.c_str());
    return;
  }

  if (have_home_ &&
      !isWithinBounds(mavros_target, home_pose_, config_.bounds, &reason)) {
    ROS_ERROR_THROTTLE(1.0, "[BRIDGE] Command rejected: %s",
                       reason.c_str());
    return;
  }

  planner_target_mavros_ = mavros_target;
  have_planner_target_ = true;
  last_command_time_ = ros::Time::now();
  debug_setpoint_publisher_.publish(planner_target_mavros_);

  if (config_.auto_track_on_command &&
      state_ == BridgeState::kHoverReady) {
    tracking_requested_ = true;
  }
}

void EgoMavrosBridge::goalCallback(
    const geometry_msgs::PoseStamped::ConstPtr& message) {
  const bool state_accepts_goals =
      state_ == BridgeState::kDryRun ||
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
  if (!have_planner_odom_ ||
      !isFresh(now, last_planner_odom_time_, config_.planner_odom_timeout) ||
      !have_cloud_ ||
      !isFresh(now, last_cloud_time_, config_.cloud_timeout)) {
    ROS_WARN("[BRIDGE] Goal rejected because planner odometry or cloud is stale.");
    return;
  }

  geometry_msgs::PoseStamped planning_goal;
  std::string reason;
  if (!transformToPlanning(*message, &planning_goal, &reason)) {
    ROS_ERROR("[BRIDGE] Goal transform failed: %s", reason.c_str());
    return;
  }
  planning_goal.header.stamp = now;
  goal_publisher_.publish(planning_goal);
  ROS_INFO("[BRIDGE] Validated goal forwarded to EGO-Planner.");
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
  if (!request.data && state_ == BridgeState::kTrackEgo) {
    return_in_progress_ = false;
    hold_pose_ = mavros_pose_;
    transitionTo(BridgeState::kHoverReady,
                 "tracking disabled; holding current pose");
  }
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
  if (!have_mavros_pose_ || !isFinitePose(mavros_pose_) ||
      !isFresh(now, last_mavros_pose_time_, config_.mavros_pose_timeout)) {
    response.success = false;
    response.message =
        "cannot start supervised landing without a fresh, valid pose";
    return true;
  }
  land_requested_ = true;
  response.success = true;
  response.message = "AUTO.LAND requested";
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

  std::string reason;
  if (!publishReturnGoal(&reason)) {
    response.success = false;
    response.message = reason;
    return true;
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

  if (state_ == BridgeState::kDryRun) {
    std::string reason;
    if (!baseInputsFresh(now, &reason)) {
      ROS_INFO_THROTTLE(2.0, "[DRY_RUN] Waiting for inputs: %s",
                        reason.c_str());
    } else {
      ROS_INFO_THROTTLE(2.0,
                        "[DRY_RUN] Inputs healthy; no MAVROS setpoint is sent.");
    }
    return;
  }
  if (state_ == BridgeState::kDone || state_ == BridgeState::kError) {
    return;
  }

  if (last_authority_check_time_.isZero() ||
      now - last_authority_check_time_ >=
          ros::Duration(kAuthorityCheckInterval)) {
    std::string conflict;
    if (hasControlConflict(&conflict)) {
      if (have_fcu_state_ && fcu_state_.armed) {
        ROS_FATAL("[BRIDGE] %s; requesting AUTO.LAND.", conflict.c_str());
        land_requested_ = true;
      } else {
        transitionTo(BridgeState::kError, conflict);
        return;
      }
    }
    last_authority_check_time_ = now;
  }

  if (land_requested_ && state_ != BridgeState::kLanding) {
    hold_pose_ = have_mavros_pose_ ? mavros_pose_ : output_setpoint_;
    transitionTo(BridgeState::kLanding, "landing requested");
  }

  const bool state_requires_armed_vehicle =
      state_ == BridgeState::kTakeoff ||
      state_ == BridgeState::kHoverReady ||
      state_ == BridgeState::kTrackEgo || state_ == BridgeState::kHold;
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
      }
      break;

    case BridgeState::kWaitInputs: {
      std::string reason;
      if (baseInputsFresh(now, &reason) && !have_home_) {
        const bool confirmed_on_ground =
            !fcu_state_.armed &&
            extended_state_.landed_state ==
                mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
        if (!confirmed_on_ground) {
          ROS_WARN_THROTTLE(
              1.0,
              "[WAIT_INPUTS] Refusing to capture home unless PX4 is disarmed and ON_GROUND.");
          break;
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
      }
      if (baseInputsFresh(now, &reason) && have_home_) {
        output_setpoint_ = home_pose_;
        hold_pose_ = home_pose_;
        have_output_setpoint_ = true;
        transitionTo(BridgeState::kPrestream, "all required inputs are fresh");
      } else {
        ROS_INFO_THROTTLE(2.0, "[WAIT_INPUTS] %s", reason.c_str());
      }
      break;
    }

    case BridgeState::kPrestream: {
      std::string reason;
      if (!baseInputsFresh(now, &reason)) {
        transitionTo(BridgeState::kWaitInputs,
                     "input became stale during prestream: " + reason);
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
      if (!baseInputsFresh(now, &reason)) {
        if (fcu_state_.armed) {
          startHold("arming input failure: " + reason);
        } else {
          transitionTo(BridgeState::kWaitInputs,
                       "arming input failure: " + reason);
        }
        break;
      }
      publishSetpoint(home_pose_, now);
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
        if (plannerAlignmentValid(&reason)) {
          transitionTo(BridgeState::kTrackEgo,
                       "fresh planner command and aligned odometry");
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
      if (!plannerAlignmentValid(&reason)) {
        startHold("planner/MAVROS alignment failure: " + reason);
        break;
      }
      if (fcu_state_.mode != "OFFBOARD") {
        requestMode("OFFBOARD", now);
        startHold("OFFBOARD mode lost");
        break;
      }

      publishSetpoint(planner_target_mavros_, now);
      updateReturnProgress(now);
      break;
    }

    case BridgeState::kHold: {
      publishHold(now);
      std::string reason;
      const bool recovered = tracking_requested_ && commandFresh(now) &&
                             baseInputsFresh(now, &reason) &&
                             plannerAlignmentValid(&reason) &&
                             fcu_state_.mode == "OFFBOARD";
      if (recovered) {
        transitionTo(BridgeState::kTrackEgo,
                     "all tracking inputs recovered");
      } else if (now - state_entered_time_ >=
                 ros::Duration(config_.land_after_loss)) {
        land_requested_ = true;
        transitionTo(BridgeState::kLanding,
                     "hold timeout: " + hold_reason_);
      } else if (fcu_state_.connected && fcu_state_.armed &&
                 fcu_state_.mode != "OFFBOARD") {
        requestMode("OFFBOARD", now);
      }
      break;
    }

    case BridgeState::kLanding: {
      publishHold(now);
      if (fcu_state_.mode != "AUTO.LAND") {
        requestMode("AUTO.LAND", now);
      }

      const bool extended_fresh =
          have_extended_state_ &&
          isFresh(now, last_extended_state_time_,
                  config_.extended_state_timeout);
      if (!fcu_state_.armed && extended_fresh &&
          extended_state_.landed_state ==
              mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
        transitionTo(BridgeState::kDone,
                     "PX4 reports disarmed and on ground");
      }
      break;
    }

    case BridgeState::kDryRun:
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
  setpoint_publisher_.publish(output_setpoint_);
}

void EgoMavrosBridge::publishHold(const ros::Time& now) {
  if (!isFinitePose(hold_pose_)) {
    if (have_mavros_pose_ && isFinitePose(mavros_pose_)) {
      hold_pose_ = mavros_pose_;
    } else {
      ROS_ERROR_THROTTLE(
          1.0, "[BRIDGE] Cannot publish HOLD without a valid vehicle pose.");
      return;
    }
  }
  publishSetpoint(hold_pose_, now);
}

bool EgoMavrosBridge::baseInputsFresh(const ros::Time& now,
                                      std::string* reason) const {
  if (!have_fcu_state_ || !fcu_state_.connected ||
      !isFresh(now, last_fcu_state_time_, config_.fcu_state_timeout)) {
    *reason = "FCU state is unavailable, disconnected or stale";
    return false;
  }
  if (!have_extended_state_ ||
      !isFresh(now, last_extended_state_time_,
               config_.extended_state_timeout)) {
    *reason = "PX4 extended state is unavailable or stale";
    return false;
  }
  if (!have_mavros_pose_ ||
      !isFresh(now, last_mavros_pose_time_,
               config_.mavros_pose_timeout)) {
    *reason = "MAVROS local pose is unavailable or stale";
    return false;
  }
  if (!have_planner_odom_ ||
      !isFresh(now, last_planner_odom_time_,
               config_.planner_odom_timeout)) {
    *reason = "planner odometry is unavailable or stale";
    return false;
  }
  if (!have_cloud_ ||
      !isFresh(now, last_cloud_time_, config_.cloud_timeout)) {
    *reason = "obstacle point cloud is unavailable or stale";
    return false;
  }
  reason->clear();
  return true;
}

bool EgoMavrosBridge::commandFresh(const ros::Time& now) const {
  return have_planner_target_ &&
         isFresh(now, last_command_time_, config_.command_timeout);
}

bool EgoMavrosBridge::plannerAlignmentValid(std::string* reason) const {
  if (!have_planner_odom_ || !have_mavros_pose_) {
    *reason = "planner or MAVROS pose is unavailable";
    return false;
  }

  geometry_msgs::PoseStamped planner_pose_mavros;
  if (!transformToMavros(odometryPose(planner_odom_),
                         &planner_pose_mavros, reason)) {
    return false;
  }

  const double position_error =
      positionDistance(planner_pose_mavros, mavros_pose_);
  const double yaw_error = std::abs(angularDistance(
      yawFromQuaternion(planner_pose_mavros.pose.orientation),
      yawFromQuaternion(mavros_pose_.pose.orientation)));
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

  const std::string resolved_topic =
      node_handle_.resolveName(config_.setpoint_topic);
  const std::string own_node = ros::this_node::getName();
  const XmlRpc::XmlRpcValue& publishers = payload[0];
  for (int index = 0; index < publishers.size(); ++index) {
    const std::string topic = publishers[index][0];
    if (topic != resolved_topic) {
      continue;
    }
    const XmlRpc::XmlRpcValue& nodes = publishers[index][1];
    for (int node_index = 0; node_index < nodes.size(); ++node_index) {
      const std::string node_name = nodes[node_index];
      if (node_name != own_node) {
        *detail = "setpoint topic already has publisher " + node_name;
        return true;
      }
    }
  }
  detail->clear();
  return false;
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
  hold_pose_ = have_mavros_pose_ ? mavros_pose_ : output_setpoint_;
  hold_pose_.header.frame_id = config_.mavros_frame;
  hold_reason_ = reason;
  transitionTo(BridgeState::kHold, reason);
}

bool EgoMavrosBridge::publishReturnGoal(std::string* reason) {
  const ros::Time now = ros::Time::now();
  if (!have_planner_odom_ ||
      !isFresh(now, last_planner_odom_time_, config_.planner_odom_timeout) ||
      !have_cloud_ ||
      !isFresh(now, last_cloud_time_, config_.cloud_timeout)) {
    *reason = "planner odometry or cloud is stale";
    return false;
  }

  geometry_msgs::PoseStamped home_hover = home_pose_;
  home_hover.header.stamp = now;
  home_hover.header.frame_id = config_.mavros_frame;
  home_hover.pose.position.z += config_.takeoff_height;

  geometry_msgs::PoseStamped planning_goal;
  if (!transformToPlanning(home_hover, &planning_goal, reason)) {
    return false;
  }
  planning_goal.header.stamp = now;
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
  home_hover.pose.position.z += config_.takeoff_height;
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
    land_requested_ = true;
    ROS_INFO("[BRIDGE] Home reached; AUTO.LAND will be requested.");
  }
}

}  // namespace ego_gazebo_bridge
