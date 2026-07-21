/**
 * EGO tower mission sequencer. The mission layer selects the nearest tower
 * and owns the closed global reference path. EGO receives one reference point
 * at a time and owns collision-aware local replanning. This node never
 * advertises a MAVROS control topic.
 */

#include "astra_tower_mission/ego_task_utils.h"
#include "astra_tower_mission/tower_route.h"

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace astra_tower_mission {
namespace {

enum class TaskState {
  kWaitInputs,
  kPublishGoal,
  kWaitPlan,
  kTrackGoal,
  kReturnHome,
  kFailureHold,
  kFailureLanding,
  kDone,
  kError,
};

const char* stateName(TaskState state) {
  switch (state) {
    case TaskState::kWaitInputs: return "WAIT_INPUTS";
    case TaskState::kPublishGoal: return "PUBLISH_GOAL";
    case TaskState::kWaitPlan: return "WAIT_PLAN";
    case TaskState::kTrackGoal: return "TRACK_GOAL";
    case TaskState::kReturnHome: return "RETURN_HOME";
    case TaskState::kFailureHold: return "FAILURE_HOLD";
    case TaskState::kFailureLanding: return "FAILURE_LANDING";
    case TaskState::kDone: return "DONE";
    case TaskState::kError: return "ERROR";
  }
  return "UNKNOWN";
}

bool finiteCommand(const quadrotor_msgs::PositionCommand& command) {
  return std::isfinite(command.position.x) &&
         std::isfinite(command.position.y) &&
         std::isfinite(command.position.z) &&
         std::isfinite(command.velocity.x) &&
         std::isfinite(command.velocity.y) &&
         std::isfinite(command.velocity.z) &&
         std::isfinite(command.acceleration.x) &&
         std::isfinite(command.acceleration.y) &&
         std::isfinite(command.acceleration.z) &&
         std::isfinite(command.yaw) && std::isfinite(command.yaw_dot);
}

geometry_msgs::Quaternion yawQuaternion(double yaw) {
  geometry_msgs::Quaternion quaternion;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

geometry_msgs::PoseStamped odomPose(const nav_msgs::Odometry& odom) {
  geometry_msgs::PoseStamped pose;
  pose.header = odom.header;
  pose.pose = odom.pose.pose;
  return pose;
}

}  // namespace

class Stage2EgoMissionNode {
 public:
  Stage2EgoMissionNode()
      : node_(), private_node_("~"), tf_listener_(tf_buffer_) {
    loadConfig();
    setupRos();
    if (scenario_ != "tower") {
      buildValidationGoals();
    }
    openReport();
    state_entered_ = ros::Time::now();
    mission_started_ = state_entered_;
    publishState();
    timer_ = node_.createTimer(ros::Duration(1.0 / loop_rate_),
                               &Stage2EgoMissionNode::timerCallback, this);
    ROS_WARN("[STAGE2_TASK] scenario=%s control=%s initial_goals=%zu "
             "tower_candidates=%zu; no MAVROS control publisher is created",
             scenario_.c_str(), enable_control_ ? "true" : "false",
             goals_.size(), tower_candidates_.size());
  }

 private:
  void loadConfig() {
    private_node_.param("enable_control", enable_control_, false);
    private_node_.param<std::string>("scenario", scenario_, "dry_run");
    private_node_.param<std::string>("planning_frame", planning_frame_,
                                     "camera_init");
    private_node_.param("manual_target_height", manual_target_height_, 16.0);
    private_node_.param("height_match_tolerance", height_match_tolerance_,
                        1e-6);
    private_node_.param("loop_rate", loop_rate_, 20.0);
    private_node_.param("input_timeout", input_timeout_, 0.3);
    private_node_.param("goal_retry_period", goal_retry_period_, 0.5);
    private_node_.param("planning_timeout", planning_timeout_, 8.0);
    private_node_.param("goal_timeout", goal_timeout_, 180.0);
    private_node_.param("arrival_tolerance", arrival_tolerance_, 0.4);
    private_node_.param("arrival_hold_duration", arrival_hold_duration_, 1.0);
    private_node_.param("tracking_error_limit", tracking_error_limit_, 1.0);
    private_node_.param("failure_hold_duration", failure_hold_duration_, 3.0);
    private_node_.param("return_timeout", return_timeout_, 300.0);
    private_node_.param("overall_timeout", overall_timeout_, 1200.0);
    private_node_.param("report_period", report_period_, 0.2);
    private_node_.param<std::string>("report_file", report_file_,
                                     "/tmp/astra_stage2_latest.csv");

    private_node_.param<std::string>("topics/odom", odom_topic_,
                                     "/Odometry");
    private_node_.param<std::string>("topics/command", command_topic_,
                                     "/planning/pos_cmd");
    private_node_.param<std::string>("topics/bridge_state",
                                     bridge_state_topic_,
                                     "/ego_mavros_bridge/state");
    private_node_.param<std::string>("topics/goal", goal_topic_,
                                     "/move_base_simple/goal");
    private_node_.param<std::string>("outputs/global_reference",
                                     global_reference_output_,
                                     "global_reference");
    private_node_.param<std::string>("outputs/selected_tower_center",
                                     selected_tower_center_output_,
                                     "selected_tower_center");
    private_node_.param<std::string>("outputs/selected_tower_name",
                                     selected_tower_name_output_,
                                     "selected_tower_name");
    private_node_.param<std::string>("outputs/face_tower",
                                     tower_yaw_mode_output_,
                                     "face_tower");
    private_node_.param<std::string>("services/tracking", tracking_service_,
                                     "/ego_mavros_bridge/enable_tracking");
    private_node_.param<std::string>("services/return_home", return_service_,
                                     "/ego_mavros_bridge/return_home");
    private_node_.param<std::string>("services/land", land_service_,
                                     "/ego_mavros_bridge/land");

    private_node_.param<std::string>("tower/name", route_.tower_name,
                                     "radio_tower");
    private_node_.param<std::string>("tower/frame_id", route_.frame_id,
                                     "map");
    private_node_.param("tower/center/x", route_.center_x, -17.4209);
    private_node_.param("tower/center/y", route_.center_y, 22.29);
    private_node_.param("tower/collision_radius",
                        route_.tower_collision_radius, 6.41);
    private_node_.param("mission/radius", route_.radius, 10.0);
    private_node_.param("mission/height", route_.height, 16.0);
    private_node_.param("mission/waypoint_count", route_.waypoint_count, 8);
    double start_angle_degrees = -90.0;
    double yaw_offset_degrees = 0.0;
    std::string direction = "counter_clockwise";
    private_node_.param("mission/start_angle_deg", start_angle_degrees,
                        start_angle_degrees);
    private_node_.param("mission/camera_yaw_offset_deg", yaw_offset_degrees,
                        yaw_offset_degrees);
    private_node_.param<std::string>("mission/direction", direction,
                                     direction);
    route_.start_angle_rad = start_angle_degrees * kPi / 180.0;
    route_.camera_yaw_offset_rad = yaw_offset_degrees * kPi / 180.0;
    if (!parseDirection(direction, &route_.direction)) {
      throw std::runtime_error("invalid mission direction");
    }
    private_node_.param("mission/minimum_height", route_.minimum_height, 2.0);
    private_node_.param("mission/maximum_height", route_.maximum_height, 30.0);
    private_node_.param("mission/minimum_safety_distance",
                        route_.minimum_safety_distance, 2.0);

    std::vector<std::string> tower_names;
    std::vector<double> tower_center_x;
    std::vector<double> tower_center_y;
    std::vector<double> tower_collision_radii;
    std::string tower_frame = route_.frame_id;
    private_node_.param<std::string>("tower_candidates/frame_id", tower_frame,
                                     tower_frame);
    const bool have_candidate_names =
        private_node_.getParam("tower_candidates/names", tower_names);
    const bool have_candidate_x =
        private_node_.getParam("tower_candidates/center_x", tower_center_x);
    const bool have_candidate_y =
        private_node_.getParam("tower_candidates/center_y", tower_center_y);
    const bool have_candidate_radii = private_node_.getParam(
        "tower_candidates/collision_radius", tower_collision_radii);
    if (have_candidate_names || have_candidate_x || have_candidate_y ||
        have_candidate_radii) {
      const std::size_t candidate_count = tower_names.size();
      if (!have_candidate_names || !have_candidate_x || !have_candidate_y ||
          !have_candidate_radii || candidate_count == 0U ||
          tower_center_x.size() != candidate_count ||
          tower_center_y.size() != candidate_count ||
          tower_collision_radii.size() != candidate_count ||
          tower_frame.empty()) {
        throw std::runtime_error("invalid tower_candidates arrays");
      }
      tower_candidates_.reserve(candidate_count);
      for (std::size_t index = 0; index < candidate_count; ++index) {
        tower_candidates_.push_back(
            {tower_names[index], tower_frame, tower_center_x[index],
             tower_center_y[index], tower_collision_radii[index]});
      }
    } else {
      tower_candidates_.push_back(
          {route_.tower_name, route_.frame_id, route_.center_x,
           route_.center_y, route_.tower_collision_radius});
    }

    for (const auto& candidate : tower_candidates_) {
      RouteConfig candidate_route = route_;
      candidate_route.tower_name = candidate.name;
      candidate_route.frame_id = candidate.frame_id;
      candidate_route.center_x = candidate.center_x;
      candidate_route.center_y = candidate.center_y;
      candidate_route.tower_collision_radius = candidate.collision_radius;
      std::string route_reason;
      if (!validateRouteConfig(candidate_route, &route_reason)) {
        throw std::runtime_error("invalid tower candidate " +
                                 candidate.name + ": " + route_reason);
      }
    }
    route_.frame_id = tower_candidates_.front().frame_id;

    private_node_.param("validation/single/x", single_x_, 3.0);
    private_node_.param("validation/single/y", single_y_, 0.0);
    private_node_.param("validation/second/x", second_x_, 3.0);
    private_node_.param("validation/second/y", second_y_, 3.0);

    const bool scenario_valid = scenario_ == "dry_run" ||
                                scenario_ == "single" ||
                                scenario_ == "dual" ||
                                scenario_ == "tower";
    const bool numeric_valid =
        std::isfinite(manual_target_height_) &&
        std::isfinite(height_match_tolerance_) &&
        height_match_tolerance_ >= 0.0 && loop_rate_ >= 10.0 &&
        input_timeout_ > 0.0 && goal_retry_period_ > 0.0 &&
        planning_timeout_ > goal_retry_period_ && goal_timeout_ > 0.0 &&
        arrival_tolerance_ > 0.0 && arrival_hold_duration_ >= 0.0 &&
        tracking_error_limit_ > 0.0 && failure_hold_duration_ > 0.0 &&
        return_timeout_ > 0.0 && overall_timeout_ > return_timeout_ &&
        report_period_ > 0.0;
    if (!scenario_valid || !numeric_valid || planning_frame_.empty() ||
        odom_topic_.empty() || command_topic_.empty() ||
        bridge_state_topic_.empty() || goal_topic_.empty() ||
        global_reference_output_.empty() ||
        selected_tower_center_output_.empty() ||
        selected_tower_name_output_.empty() ||
        tower_yaw_mode_output_.empty()) {
      throw std::runtime_error("invalid Stage 2 task configuration");
    }
    if (enable_control_ && scenario_ == "dry_run") {
      throw std::runtime_error("dry_run cannot enable control");
    }
    if (!enable_control_ && scenario_ != "dry_run") {
      throw std::runtime_error("flight scenarios require enable_control=true");
    }
  }

  geometry_msgs::PoseStamped makeGoal(double x, double y, double yaw) const {
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = route_.frame_id;
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = manual_target_height_;
    goal.pose.orientation = yawQuaternion(yaw);
    return goal;
  }

  void validateGoalHeights() const {
    std::string reason;
    if (!validateFixedHeightGoals(goals_, manual_target_height_,
                                  height_match_tolerance_, &reason) ||
        std::abs(route_.height - manual_target_height_) >
            height_match_tolerance_) {
      throw std::runtime_error(
          reason.empty() ? "mission height and manual_target_height differ"
                         : reason);
    }
  }

  void buildValidationGoals() {
    goals_.push_back(makeGoal(single_x_, single_y_, 0.0));
    if (scenario_ == "dual") {
      goals_.push_back(makeGoal(second_x_, second_y_, 0.0));
    }
    validateGoalHeights();
  }

  bool buildNearestTowerReference(const ros::Time& now,
                                  std::string* reason) {
    geometry_msgs::PoseStamped vehicle = odomPose(odom_);
    geometry_msgs::PoseStamped vehicle_in_tower_frame;
    try {
      if (vehicle.header.frame_id == route_.frame_id) {
        vehicle_in_tower_frame = vehicle;
      } else {
        const auto transform = tf_buffer_.lookupTransform(
            route_.frame_id, vehicle.header.frame_id, vehicle.header.stamp,
            ros::Duration(0.05));
        tf2::doTransform(vehicle, vehicle_in_tower_frame, transform);
      }
    } catch (const tf2::TransformException& exception) {
      *reason = exception.what();
      return false;
    }

    const int nearest_index = nearestTowerIndex(
        tower_candidates_, vehicle_in_tower_frame.pose.position.x,
        vehicle_in_tower_frame.pose.position.y);
    if (nearest_index < 0) {
      *reason = "no valid nearest tower candidate";
      return false;
    }
    const auto& selected =
        tower_candidates_.at(static_cast<std::size_t>(nearest_index));
    route_.tower_name = selected.name;
    route_.frame_id = selected.frame_id;
    route_.center_x = selected.center_x;
    route_.center_y = selected.center_y;
    route_.tower_collision_radius = selected.collision_radius;

    goals_.clear();
    for (const auto& waypoint : generateTowerWaypoints(route_)) {
      goals_.push_back(makeGoal(waypoint.x, waypoint.y, waypoint.yaw));
    }
    goals_ = appendClosureGoal(goals_);
    validateGoalHeights();

    global_reference_.header.frame_id = route_.frame_id;
    global_reference_.header.stamp = now;
    global_reference_.poses = goals_;
    for (auto& pose : global_reference_.poses) {
      pose.header = global_reference_.header;
    }
    global_reference_pub_.publish(global_reference_);

    geometry_msgs::PointStamped center;
    center.header = global_reference_.header;
    center.point.x = route_.center_x;
    center.point.y = route_.center_y;
    center.point.z = route_.height;
    selected_tower_center_pub_.publish(center);
    std_msgs::String selected_name;
    selected_tower_name_ = route_.tower_name;
    selected_name.data = selected_tower_name_;
    selected_tower_name_pub_.publish(selected_name);
    setTowerYawMode(false);
    have_global_reference_ = true;
    ROS_WARN("[STAGE2_TASK] nearest tower=%s center=(%.3f, %.3f), "
             "global reference=%zu points at %.2f m; EGO owns local replanning",
             route_.tower_name.c_str(), route_.center_x, route_.center_y,
             goals_.size(), route_.height);
    reason->clear();
    return true;
  }

  void setTowerYawMode(bool face_tower) {
    if (tower_yaw_active_ == face_tower && tower_yaw_mode_published_) {
      return;
    }
    tower_yaw_active_ = face_tower;
    tower_yaw_mode_published_ = true;
    std_msgs::Bool mode;
    mode.data = face_tower;
    tower_yaw_mode_pub_.publish(mode);
  }

  void setupRos() {
    odom_sub_ = node_.subscribe(odom_topic_, 20,
                                &Stage2EgoMissionNode::odomCallback, this);
    command_sub_ = node_.subscribe(command_topic_, 50,
                                   &Stage2EgoMissionNode::commandCallback,
                                   this);
    bridge_state_sub_ = node_.subscribe(
        bridge_state_topic_, 10,
        &Stage2EgoMissionNode::bridgeStateCallback, this);
    goal_pub_ = node_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1);

    state_pub_ = private_node_.advertise<std_msgs::String>("state", 1, true);
    result_pub_ =
        private_node_.advertise<std_msgs::String>("final_result", 1, true);
    progress_pub_ =
        private_node_.advertise<std_msgs::Float64>("progress", 10);
    index_pub_ =
        private_node_.advertise<std_msgs::UInt32>("waypoint_index", 10);
    error_pub_ =
        private_node_.advertise<std_msgs::Float64>("position_error", 10);
    tracking_error_pub_ = private_node_.advertise<std_msgs::Float64>(
        "tracking_error", 10);
    target_pub_ = private_node_.advertise<geometry_msgs::PoseStamped>(
        "current_target", 1, true);
    actual_path_pub_ =
        private_node_.advertise<nav_msgs::Path>("actual_path", 1, true);
    global_reference_pub_ = private_node_.advertise<nav_msgs::Path>(
        global_reference_output_, 1, true);
    selected_tower_center_pub_ =
        private_node_.advertise<geometry_msgs::PointStamped>(
            selected_tower_center_output_, 1, true);
    selected_tower_name_pub_ = private_node_.advertise<std_msgs::String>(
        selected_tower_name_output_, 1, true);
    tower_yaw_mode_pub_ = private_node_.advertise<std_msgs::Bool>(
        tower_yaw_mode_output_, 1, true);

    tracking_client_ =
        node_.serviceClient<std_srvs::SetBool>(tracking_service_);
    return_client_ = node_.serviceClient<std_srvs::Trigger>(return_service_);
    land_client_ = node_.serviceClient<std_srvs::Trigger>(land_service_);
  }

  void openReport() {
    if (report_file_.empty()) {
      return;
    }
    report_.open(report_file_, std::ios::out | std::ios::trunc);
    if (!report_) {
      ROS_ERROR("[STAGE2_TASK] cannot open report %s", report_file_.c_str());
      return;
    }
    report_ << "sim_time,state,goal_index,trajectory_id,target_x,target_y,"
               "target_z,actual_x,actual_y,actual_z,ref_x,ref_y,ref_z,"
               "ref_vx,ref_vy,ref_vz,ref_ax,ref_ay,ref_az,ref_yaw,"
               "ref_yaw_rate,goal_error,tracking_error,max_tracking_error,"
               "bridge_state,selected_tower,yaw_policy\n";
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    const auto pose = odomPose(*message);
    if (message->header.frame_id != planning_frame_ ||
        message->header.stamp.isZero() ||
        !std::isfinite(pose.pose.position.x) ||
        !std::isfinite(pose.pose.position.y) ||
        !std::isfinite(pose.pose.position.z)) {
      return;
    }
    odom_ = *message;
    have_odom_ = true;
    odom_received_ = ros::Time::now();
    actual_path_.header = message->header;
    if (actual_path_.poses.size() < 25000) {
      actual_path_.poses.push_back(pose);
    }
  }

  void commandCallback(
      const quadrotor_msgs::PositionCommand::ConstPtr& message) {
    if (message->header.frame_id != planning_frame_ ||
        message->header.stamp.isZero() || !finiteCommand(*message) ||
        message->trajectory_flag !=
            quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY) {
      return;
    }
    command_ = *message;
    have_command_ = true;
    command_received_ = ros::Time::now();
  }

  void bridgeStateCallback(const std_msgs::String::ConstPtr& message) {
    bridge_state_ = message->data;
    have_bridge_state_ = true;
    bridge_state_received_ = ros::Time::now();
  }

  bool fresh(const ros::Time& now, const ros::Time& received,
             double timeout) const {
    return !received.isZero() && now >= received &&
           now - received <= ros::Duration(timeout);
  }

  void transition(TaskState next, const std::string& reason) {
    if (state_ == next) return;
    ROS_WARN("[STAGE2_TASK] %s -> %s: %s", stateName(state_),
             stateName(next), reason.c_str());
    state_ = next;
    state_entered_ = ros::Time::now();
    publishState();
  }

  void publishState() {
    std_msgs::String message;
    message.data = stateName(state_);
    state_pub_.publish(message);
  }

  bool transformCurrentGoal(const ros::Time& now, std::string* reason) {
    geometry_msgs::PoseStamped goal = goals_.at(goal_index_);
    goal.header.stamp = now;
    try {
      const auto transform = tf_buffer_.lookupTransform(
          planning_frame_, goal.header.frame_id, now, ros::Duration(0.05));
      tf2::doTransform(goal, planning_goal_, transform);
    } catch (const tf2::TransformException& exception) {
      *reason = exception.what();
      return false;
    }
    if (std::abs(planning_goal_.pose.position.z - manual_target_height_) >
        height_match_tolerance_) {
      *reason = "TF changed the fixed-height goal contract";
      return false;
    }
    reason->clear();
    return true;
  }

  void publishGoal(const ros::Time& now) {
    geometry_msgs::PoseStamped goal = goals_.at(goal_index_);
    goal.header.stamp = now;
    goal_pub_.publish(goal);
    target_pub_.publish(goal);
    last_goal_publish_ = now;
  }

  void beginGoal(const ros::Time& now) {
    std::string reason;
    if (!transformCurrentGoal(now, &reason)) {
      fail("goal TF failure: " + reason);
      return;
    }
    trajectory_baseline_ = have_command_ ? command_.trajectory_id : 0;
    goal_sent_ = now;
    publishGoal(now);
    transition(TaskState::kWaitPlan, "current goal published to EGO");
  }

  void requestTracking(bool enabled) {
    std_srvs::SetBool service;
    service.request.data = enabled;
    if (!tracking_client_.call(service) || !service.response.success) {
      ROS_WARN("[STAGE2_TASK] tracking service rejected: %s",
               service.response.message.c_str());
    }
  }

  void fail(const std::string& reason) {
    if (state_ == TaskState::kDone || state_ == TaskState::kError ||
        state_ == TaskState::kFailureHold ||
        state_ == TaskState::kFailureLanding) {
      return;
    }
    failure_reason_ = reason;
    setTowerYawMode(false);
    if (!enable_control_) {
      finish(false, reason);
      return;
    }
    requestTracking(false);
    transition(TaskState::kFailureHold, reason);
  }

  void finish(bool success, const std::string& reason) {
    setTowerYawMode(false);
    std_msgs::String result;
    result.data = success ? "SUCCESS: " + reason : "FAILURE: " + reason;
    result_pub_.publish(result);
    transition(success ? TaskState::kDone : TaskState::kError, reason);
    ROS_WARN("[STAGE2_TASK] %s max_tracking_error=%.3f m report=%s",
             result.data.c_str(), maximum_tracking_error_,
             report_file_.c_str());
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (now.isZero() || state_ == TaskState::kDone ||
        state_ == TaskState::kError) {
      return;
    }
    // Constructors can precede the first /clock sample in simulation. Keep
    // the overall timeout finite by anchoring it on the first usable time.
    if (mission_started_.isZero()) {
      mission_started_ = now;
    }
    if (state_entered_.isZero()) {
      state_entered_ = now;
    }
    if (!mission_started_.isZero() &&
        now - mission_started_ >= ros::Duration(overall_timeout_)) {
      fail("overall mission timeout");
    }

    switch (state_) {
      case TaskState::kWaitInputs: {
        const bool inputs_fresh =
            have_odom_ && have_bridge_state_ &&
            fresh(now, odom_received_, input_timeout_) &&
            fresh(now, bridge_state_received_, 2.0);
        const bool bridge_ready =
            (!enable_control_ && bridge_state_ == "DRY_RUN") ||
            (enable_control_ && bridge_state_ == "HOVER_READY");
        if (inputs_fresh && bridge_ready) {
          if (scenario_ == "tower" && !have_global_reference_) {
            std::string reason;
            if (!buildNearestTowerReference(now, &reason)) {
              ROS_WARN_THROTTLE(
                  1.0,
                  "[STAGE2_TASK] waiting to build nearest-tower reference: %s",
                  reason.c_str());
              break;
            }
          }
          transition(TaskState::kPublishGoal, "odom and bridge are ready");
        }
        break;
      }
      case TaskState::kPublishGoal:
        beginGoal(now);
        break;
      case TaskState::kWaitPlan:
        if (have_command_ &&
            isNewTrajectory(trajectory_baseline_, command_.trajectory_id,
                            goal_sent_, command_.header.stamp)) {
          if (!enable_control_) {
            finish(true, "FAST-LIO -> EGO -> traj_server chain verified");
          } else {
            requestTracking(true);
            arrival_active_ = false;
            transition(TaskState::kTrackGoal, "new trajectory id received");
          }
        } else if (now - goal_sent_ >= ros::Duration(planning_timeout_)) {
          fail("planning timeout: no new trajectory id");
        } else if (now - last_goal_publish_ >=
                   ros::Duration(goal_retry_period_)) {
          publishGoal(now);
        }
        break;
      case TaskState::kTrackGoal: {
        if (bridge_state_ == "HOLD" || bridge_state_ == "ERROR" ||
            bridge_state_ == "LANDING" || bridge_state_ == "DONE") {
          fail("unexpected bridge state while tracking: " + bridge_state_);
          break;
        }
        if (!have_command_ || !have_odom_ ||
            !fresh(now, command_received_, input_timeout_) ||
            !fresh(now, odom_received_, input_timeout_)) {
          fail("odom or PositionCommand input expired");
          break;
        }
        const geometry_msgs::PoseStamped actual = odomPose(odom_);
        geometry_msgs::PoseStamped reference;
        reference.header = command_.header;
        reference.pose.position = command_.position;
        reference.pose.orientation.w = 1.0;
        current_goal_error_ =
            posePositionDistance(actual, planning_goal_);
        current_tracking_error_ =
            posePositionDistance(actual, reference);
        maximum_tracking_error_ =
            std::max(maximum_tracking_error_, current_tracking_error_);
        if (current_tracking_error_ > tracking_error_limit_) {
          fail("task tracking error limit exceeded");
          break;
        }
        if (current_goal_error_ <= arrival_tolerance_) {
          if (!arrival_active_) {
            arrival_active_ = true;
            arrival_since_ = now;
          } else if (now - arrival_since_ >=
                     ros::Duration(arrival_hold_duration_)) {
            ++goal_index_;
            if (goal_index_ < goals_.size()) {
              if (scenario_ == "tower" && goal_index_ == 1U) {
                setTowerYawMode(true);
              }
              transition(TaskState::kPublishGoal,
                         "measured odometry reached current goal");
            } else {
              setTowerYawMode(false);
              std_srvs::Trigger service;
              if (!return_client_.call(service) ||
                  !service.response.success) {
                fail("return_home service failed: " +
                     service.response.message);
              } else {
                transition(TaskState::kReturnHome,
                           "all goals reached; return requested");
              }
            }
          }
        } else {
          arrival_active_ = false;
        }
        if (now - goal_sent_ >= ros::Duration(goal_timeout_)) {
          fail("goal arrival timeout");
        }
        break;
      }
      case TaskState::kReturnHome:
        if (bridge_state_ == "DONE") {
          finish(true, "all goals, return and landing completed");
        } else if (bridge_state_ == "ERROR") {
          fail("bridge failed during return/landing");
        } else if (now - state_entered_ >= ros::Duration(return_timeout_)) {
          fail("return/landing timeout");
        }
        break;
      case TaskState::kFailureHold:
        if (now - state_entered_ >= ros::Duration(failure_hold_duration_)) {
          std_srvs::Trigger service;
          if (!land_client_.call(service) || !service.response.success) {
            finish(false, failure_reason_ + "; land service rejected: " +
                              service.response.message);
          } else {
            transition(TaskState::kFailureLanding,
                       "HOLD complete; supervised landing requested");
          }
        }
        break;
      case TaskState::kFailureLanding:
        if (bridge_state_ == "DONE" || bridge_state_ == "ERROR") {
          finish(false, failure_reason_ + "; terminal bridge state=" +
                            bridge_state_);
        } else if (now - state_entered_ >= ros::Duration(return_timeout_)) {
          finish(false, failure_reason_ + "; failure landing timeout");
        }
        break;
      case TaskState::kDone:
      case TaskState::kError:
        break;
    }
    publishTelemetry(now);
  }

  void publishTelemetry(const ros::Time& now) {
    std_msgs::Float64 progress;
    progress.data = goals_.empty()
                        ? 0.0
                        : static_cast<double>(goal_index_) / goals_.size();
    progress_pub_.publish(progress);
    std_msgs::UInt32 index;
    index.data = static_cast<std::uint32_t>(goal_index_);
    index_pub_.publish(index);
    std_msgs::Float64 error;
    error.data = current_goal_error_;
    error_pub_.publish(error);
    std_msgs::Float64 tracking;
    tracking.data = current_tracking_error_;
    tracking_error_pub_.publish(tracking);
    if (have_odom_) actual_path_pub_.publish(actual_path_);

    if (report_ &&
        (last_report_.isZero() ||
         now - last_report_ >= ros::Duration(report_period_))) {
      const auto actual = have_odom_ ? odom_.pose.pose.position
                                     : geometry_msgs::Point();
      report_ << std::fixed << std::setprecision(6) << now.toSec() << ','
              << stateName(state_) << ',' << goal_index_ << ','
              << (have_command_ ? command_.trajectory_id : 0) << ','
              << planning_goal_.pose.position.x << ','
              << planning_goal_.pose.position.y << ','
              << planning_goal_.pose.position.z << ',' << actual.x << ','
              << actual.y << ',' << actual.z << ',' << command_.position.x
              << ',' << command_.position.y << ',' << command_.position.z
              << ',' << command_.velocity.x << ',' << command_.velocity.y
              << ',' << command_.velocity.z << ','
              << command_.acceleration.x << ',' << command_.acceleration.y
              << ',' << command_.acceleration.z << ',' << command_.yaw
              << ',' << command_.yaw_dot << ',' << current_goal_error_ << ','
              << current_tracking_error_ << ',' << maximum_tracking_error_
              << ',' << bridge_state_ << ',' << selected_tower_name_ << ','
              << (tower_yaw_active_ ? "FACE_TOWER" : "VELOCITY_FORWARD")
              << '\n';
      report_.flush();
      last_report_ = now;
    }
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber odom_sub_;
  ros::Subscriber command_sub_;
  ros::Subscriber bridge_state_sub_;
  ros::Publisher goal_pub_;
  ros::Publisher state_pub_;
  ros::Publisher result_pub_;
  ros::Publisher progress_pub_;
  ros::Publisher index_pub_;
  ros::Publisher error_pub_;
  ros::Publisher tracking_error_pub_;
  ros::Publisher target_pub_;
  ros::Publisher actual_path_pub_;
  ros::Publisher global_reference_pub_;
  ros::Publisher selected_tower_center_pub_;
  ros::Publisher selected_tower_name_pub_;
  ros::Publisher tower_yaw_mode_pub_;
  ros::ServiceClient tracking_client_;
  ros::ServiceClient return_client_;
  ros::ServiceClient land_client_;
  ros::Timer timer_;

  bool enable_control_{false};
  std::string scenario_;
  std::string planning_frame_;
  double manual_target_height_{16.0};
  double height_match_tolerance_{1e-6};
  double loop_rate_{20.0};
  double input_timeout_{0.3};
  double goal_retry_period_{0.5};
  double planning_timeout_{8.0};
  double goal_timeout_{180.0};
  double arrival_tolerance_{0.4};
  double arrival_hold_duration_{1.0};
  double tracking_error_limit_{1.0};
  double failure_hold_duration_{3.0};
  double return_timeout_{300.0};
  double overall_timeout_{1200.0};
  double report_period_{0.2};
  double single_x_{3.0};
  double single_y_{0.0};
  double second_x_{3.0};
  double second_y_{3.0};
  std::string report_file_;
  std::string odom_topic_;
  std::string command_topic_;
  std::string bridge_state_topic_;
  std::string goal_topic_;
  std::string global_reference_output_;
  std::string selected_tower_center_output_;
  std::string selected_tower_name_output_;
  std::string tower_yaw_mode_output_;
  std::string tracking_service_;
  std::string return_service_;
  std::string land_service_;
  std::string selected_tower_name_;
  RouteConfig route_;
  std::vector<TowerCandidate> tower_candidates_;
  std::vector<geometry_msgs::PoseStamped> goals_;

  TaskState state_{TaskState::kWaitInputs};
  std::size_t goal_index_{0};
  std::uint32_t trajectory_baseline_{0};
  bool have_odom_{false};
  bool have_command_{false};
  bool have_bridge_state_{false};
  bool arrival_active_{false};
  bool have_global_reference_{false};
  bool tower_yaw_active_{false};
  bool tower_yaw_mode_published_{false};
  nav_msgs::Odometry odom_;
  quadrotor_msgs::PositionCommand command_;
  geometry_msgs::PoseStamped planning_goal_;
  nav_msgs::Path actual_path_;
  nav_msgs::Path global_reference_;
  std::string bridge_state_;
  std::string failure_reason_;
  ros::Time state_entered_;
  ros::Time mission_started_;
  ros::Time odom_received_;
  ros::Time command_received_;
  ros::Time bridge_state_received_;
  ros::Time goal_sent_;
  ros::Time last_goal_publish_;
  ros::Time arrival_since_;
  ros::Time last_report_;
  double current_goal_error_{0.0};
  double current_tracking_error_{0.0};
  double maximum_tracking_error_{0.0};
  std::ofstream report_;
};

}  // namespace astra_tower_mission

int main(int argc, char** argv) {
  ros::init(argc, argv, "stage2_ego_mission");
  try {
    astra_tower_mission::Stage2EgoMissionNode node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("[STAGE2_TASK] %s", exception.what());
    return 1;
  }
  return 0;
}
