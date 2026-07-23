/**
 * Stage 3 EGO static-obstacle mission supervisor.
 *
 * This node owns task policy only.  It never publishes a MAVROS setpoint;
 * bridge services are the sole HOLD/RESUME/RETURN/LAND control boundary.
 */
#include "astra_tower_mission/stage3_planner.h"

#include <astra_custom_msgs/InspectionCandidate.h>
#include <astra_custom_msgs/InspectionCandidateArray.h>
#include <astra_custom_msgs/PlannerStatus.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace astra_tower_mission {
namespace {

enum class MissionState {
  kWaitInputs,
  kApproach,
  kEvaluate,
  kTargetLocked,
  kNavigate,
  kHolding,
  kRelocating,
  kRecovering,
  kReturnHome,
  kFailureLanding,
  kDone,
  kError,
};

const char* missionStateName(MissionState state) {
  switch (state) {
    case MissionState::kWaitInputs: return "WAIT_INPUTS";
    case MissionState::kApproach: return "APPROACH";
    case MissionState::kEvaluate: return "EVALUATING";
    case MissionState::kTargetLocked: return "TARGET_LOCKED";
    case MissionState::kNavigate: return "NAVIGATING";
    case MissionState::kHolding: return "HOLDING";
    case MissionState::kRelocating: return "RELOCATING";
    case MissionState::kRecovering: return "RECOVERING";
    case MissionState::kReturnHome: return "RETURN_HOME";
    case MissionState::kFailureLanding: return "FAILURE_LANDING";
    case MissionState::kDone: return "DONE";
    case MissionState::kError: return "ERROR";
  }
  return "UNKNOWN";
}

enum class GoalKind { kEntryGate, kSector, kRecovery };

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double angleError(double a, double b) { return normalizeAngle(a - b); }

geometry_msgs::Point pointOf(const nav_msgs::Odometry& odom) {
  return odom.pose.pose.position;
}

geometry_msgs::Quaternion yawQuaternion(double yaw) {
  geometry_msgs::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

}  // namespace

class Stage3EgoMissionNode {
 public:
  Stage3EgoMissionNode() : node_(), private_node_("~") {
    loadConfig();
    setupRos();
    state_entered_ = ros::Time::now();
    mission_started_ = state_entered_;
    publishState();
    timer_ = node_.createTimer(ros::Duration(1.0 / loop_rate_),
                               &Stage3EgoMissionNode::timerCallback, this);
    ROS_WARN("[STAGE3_TASK] control=%s sectors=%d inspection_height=%.2f "
             "recovery_max=%.2f virtual_ceil=%.2f; no MAVROS publisher",
             enable_control_ ? "true" : "false", sector_count_,
             inspection_height_, recovery_height_max_, virtual_ceil_height_);
  }

 private:
  void loadConfig() {
    private_node_.param("enable_control", enable_control_, false);
    private_node_.param<std::string>("planning_frame", planning_frame_,
                                     "camera_init");
    private_node_.param("loop_rate", loop_rate_, 20.0);
    private_node_.param("input_timeout", input_timeout_, 0.7);
    private_node_.param("map_timeout", map_timeout_, 0.7);
    private_node_.param("planning_timeout", planning_timeout_, 10.0);
    private_node_.param("goal_timeout", goal_timeout_, 180.0);
    private_node_.param("arrival_tolerance", arrival_tolerance_, 0.5);
    private_node_.param("arrival_velocity_threshold", arrival_velocity_threshold_, 0.2);
    private_node_.param("arrival_yaw_tolerance", arrival_yaw_tolerance_, 0.26);
    private_node_.param("arrival_hold_duration", arrival_hold_duration_, 1.0);
    private_node_.param("tracking_error_limit", tracking_error_limit_, 1.2);
    private_node_.param("no_progress_window", no_progress_window_, 12.0);
    private_node_.param("no_progress_epsilon", no_progress_epsilon_, 0.15);
    private_node_.param("consecutive_plan_failure_limit",
                        consecutive_plan_failure_limit_, 3);
    private_node_.param("emergency_stop_timeout", emergency_stop_timeout_, 6.0);
    private_node_.param("failure_hold_duration", failure_hold_duration_, 2.0);
    private_node_.param("max_recovery_attempts", max_recovery_attempts_, 2);
    private_node_.param("recovery_target_timeout", recovery_target_timeout_, 60.0);
    private_node_.param("overall_timeout", overall_timeout_, 2400.0);
    private_node_.param("report_period", report_period_, 0.2);
    private_node_.param<std::string>("report_file", report_file_,
                                     "/tmp/astra_stage3_latest.csv");

    private_node_.param<std::string>("topics/odom", odom_topic_, "/Odometry");
    private_node_.param<std::string>("topics/command", command_topic_,
                                     "/planning/pos_cmd");
    private_node_.param<std::string>("topics/cloud", cloud_topic_,
                                     "/stage2/cloud_registered_filtered");
    private_node_.param<std::string>("topics/occupancy_inflate",
                                     occupancy_topic_,
                                     "/grid_map/occupancy_inflate");
    private_node_.param<std::string>("topics/planner_status", planner_status_topic_,
                                     "/planner/status");
    private_node_.param<std::string>("topics/bridge_state", bridge_state_topic_,
                                     "/ego_mavros_bridge/state");
    private_node_.param<std::string>("topics/goal", goal_topic_,
                                     "/move_base_simple/goal");
    private_node_.param<std::string>("outputs/state", state_topic_,
                                     "/tower_mission/state");
    private_node_.param<std::string>("outputs/current_target", target_topic_,
                                     "/tower_mission/current_target");
    private_node_.param<std::string>("outputs/current_sector", sector_topic_,
                                     "/tower_mission/current_sector");
    private_node_.param<std::string>("outputs/candidate_targets", candidates_topic_,
                                     "/tower_mission/candidate_targets");
    private_node_.param<std::string>("outputs/progress", progress_topic_,
                                     "/tower_mission/progress");
    private_node_.param<std::string>("outputs/face_tower", face_tower_topic_,
                                     "/tower_mission/face_tower");
    private_node_.param<std::string>("outputs/selected_tower_center",
                                     tower_center_topic_,
                                     "/tower_mission/selected_tower_center");
    private_node_.param<std::string>("services/tracking", tracking_service_,
                                     "/ego_mavros_bridge/enable_tracking");
    private_node_.param<std::string>("services/cancel", cancel_service_,
                                     "/ego_mavros_bridge/cancel_current_trajectory");
    private_node_.param<std::string>("services/resume", resume_service_,
                                     "/ego_mavros_bridge/resume_ego");
    private_node_.param<std::string>("services/return_home", return_service_,
                                     "/ego_mavros_bridge/return_home");
    private_node_.param<std::string>("services/land", land_service_,
                                     "/ego_mavros_bridge/land");

    private_node_.param<std::string>("tower/name", route_.tower_name,
                                     "radio_tower");
    private_node_.param<std::string>("tower/frame_id", route_.frame_id, "map");
    private_node_.param("tower/center/x", route_.center_x, -10.0551);
    private_node_.param("tower/center/y", route_.center_y, 19.7104);
    private_node_.param("tower/collision_radius", route_.tower_collision_radius,
                        6.41);
    private_node_.param("mission/inspection_height", inspection_height_, 30.0);
    private_node_.param("mission/recovery_height_max", recovery_height_max_, 40.0);
    private_node_.param("mission/virtual_ceil_height", virtual_ceil_height_, 45.0);
    private_node_.param("mission/radius", route_.radius, 14.0);
    private_node_.param("mission/sector_count", sector_count_, 8);
    private_node_.param("mission/sector_limit", sector_limit_, 1);
    private_node_.param("mission/layer_count", layer_count_, 1);
    private_node_.param("mission/start_angle_deg", start_angle_deg_, -67.5);
    private_node_.param<std::string>("mission/direction", direction_,
                                     "counter_clockwise");
    private_node_.param("mission/minimum_height", route_.minimum_height, 2.0);
    private_node_.param("mission/maximum_height", route_.maximum_height,
                        virtual_ceil_height_);
    private_node_.param("mission/minimum_safety_distance",
                        route_.minimum_safety_distance, 2.0);
    private_node_.param("mission/transit_height", transit_height_, 4.0);
    private_node_.param("mission/observation_radius_offset",
                        observation_radius_offset_, 5.0);
    private_node_.param("mission/observation_angle_deg", observation_angle_deg_,
                        start_angle_deg_);
    private_node_.param("mission/approach_segment_length", approach_segment_length_,
                        6.0);
    private_node_.param("entry_gate/minimum_radius",
                        entry_gate_config_.minimum_radius, route_.radius + 2.0);
    private_node_.param("entry_gate/maximum_radius",
                        entry_gate_config_.maximum_radius, route_.radius + 10.0);
    private_node_.param("entry_gate/maximum_horizontal_distance",
                        entry_gate_config_.maximum_horizontal_distance, 60.0);
    private_node_.param("entry_gate/minimum_clearance",
                        entry_gate_config_.minimum_clearance, 2.0);
    private_node_.param("entry_gate/cloud_inflation",
                        entry_gate_config_.cloud_inflation, 0.4);
    private_node_.param("entry_gate/corridor_sample_step",
                        entry_gate_config_.corridor_sample_step, 0.5);
    private_node_.param("entry_gate/clearance_weight",
                        entry_gate_config_.clearance_weight, 0.1);
    private_node_.param("entry_gate/distance_weight",
                        entry_gate_config_.distance_weight, 1.0);
    private_node_.param("entry_gate/blocked_corridor_penalty",
                        entry_gate_config_.blocked_corridor_penalty, 5.0);
    private_node_.param("entry_gate/maximum_segment_length",
                        entry_gate_segment_length_, 6.0);
    private_node_.param("mission/sector_angle_half_width_deg",
                        sector_angle_half_width_deg_, 12.0);
    private_node_.param("mission/sector_radius_half_width", sector_radius_half_width_,
                        4.0);
    private_node_.param("mission/sector_height_half_width", sector_height_half_width_,
                        1.0);
    private_node_.param("mission/target_replacement_margin",
                        target_replacement_margin_, 2.0);

    private_node_.param("candidate/minimum_clearance", filter_config_.minimum_clearance,
                        2.0);
    private_node_.param("candidate/cloud_inflation", filter_config_.cloud_inflation,
                        0.4);
    private_node_.param("candidate/corridor_sample_step",
                        filter_config_.corridor_sample_step, 0.5);
    private_node_.param("candidate/blocked_corridor_penalty",
                        filter_config_.blocked_corridor_penalty, 5.0);
    private_node_.param("recovery/radial_step", recovery_config_.radial_step, 3.0);
    private_node_.param("recovery/tangent_step", recovery_config_.tangent_step, 4.0);
    private_node_.param("recovery/maximum_radius", recovery_config_.maximum_radius,
                        24.0);
    private_node_.param("recovery/direction", recovery_direction_, direction_);
    private_node_.param("recovery/recovery_height", recovery_height_, 38.0);
    recovery_height_ = std::min(recovery_height_, recovery_height_max_);
    recovery_config_.recovery_height = recovery_height_;

    route_.height = inspection_height_;
    route_.waypoint_count = sector_count_;
    route_.start_angle_rad = start_angle_deg_ * kPi / 180.0;
    if (!parseDirection(direction_, &route_.direction))
      throw std::runtime_error("invalid mission/direction");
    if (!parseDirection(recovery_direction_, &recovery_config_.direction))
      throw std::runtime_error("invalid recovery/direction");
    route_.maximum_height = virtual_ceil_height_;
    if (route_.maximum_height < recovery_height_max_)
      throw std::runtime_error("virtual_ceil_height must cover recovery_height_max");

    std::vector<double> angle_offsets, radius_offsets, height_offsets;
    private_node_.param("candidate/angle_offsets_deg", angle_offsets,
                        std::vector<double>{0.0, -5.0, 5.0, -10.0, 10.0});
    private_node_.param("candidate/radius_offsets_m", radius_offsets,
                        std::vector<double>{0.0, 2.0, 4.0});
    private_node_.param("candidate/height_offsets_m", height_offsets,
                        std::vector<double>{0.0, -1.0, 1.0});
    for (double angle : angle_offsets)
      for (double radius : radius_offsets)
        for (double height : height_offsets)
          offsets_.push_back({angle, radius, height});
    private_node_.param("entry_gate/angle_offsets_deg",
                        entry_gate_angle_offsets_,
                        std::vector<double>{-40.0, -30.0, -20.0, -10.0, 0.0,
                                            10.0, 20.0, 30.0, 40.0});
    private_node_.param("entry_gate/radius_offsets_m",
                        entry_gate_radius_offsets_,
                        std::vector<double>{2.0, 4.0, 6.0});
    entry_gate_config_.inspection_height = inspection_height_;
    entry_gate_config_.minimum_height = route_.minimum_height;
    entry_gate_config_.maximum_height = virtual_ceil_height_;

    loadStaticObstacles();
    const auto validation = validateRouteConfig(route_, &validation_reason_);
    if (!validation || sector_count_ != 8 || sector_limit_ < 1 ||
        sector_limit_ > sector_count_ || layer_count_ != 1 ||
        inspection_height_ > virtual_ceil_height_ ||
        recovery_height_max_ > virtual_ceil_height_ || loop_rate_ < 5.0) {
      throw std::runtime_error(validation_reason_.empty()
                                   ? "invalid stage3 configuration"
                                   : validation_reason_);
    }
    if (!std::isfinite(inspection_height_) || !std::isfinite(recovery_height_max_) ||
        !std::isfinite(virtual_ceil_height_) || !std::isfinite(transit_height_) ||
        transit_height_ < route_.minimum_height ||
        transit_height_ > route_.maximum_height ||
        recovery_config_.radial_step <= 0.0 || recovery_config_.tangent_step <= 0.0 ||
        recovery_config_.maximum_radius < route_.radius ||
        recovery_config_.maximum_radius <= 0.0 ||
        entry_gate_config_.minimum_radius <= route_.tower_collision_radius ||
        entry_gate_config_.maximum_radius < entry_gate_config_.minimum_radius ||
        entry_gate_segment_length_ <= 0.0 ||
        recovery_height_ < route_.minimum_height ||
        recovery_height_ > recovery_height_max_) {
      throw std::runtime_error("invalid stage3 height or recovery envelope");
    }
  }

  void loadStaticObstacles() {
    std::vector<std::string> names;
    std::vector<double> xs, ys, zs, radii, zmins, zmaxs;
    std::vector<double> half_extent_xs, half_extent_ys, yaws;
    const bool any = private_node_.getParam("static_obstacles/names", names) ||
                     private_node_.getParam("static_obstacles/x", xs) ||
                     private_node_.getParam("static_obstacles/y", ys) ||
                     private_node_.getParam("static_obstacles/z", zs) ||
                     private_node_.getParam("static_obstacles/radius", radii);
    if (!any) {
      StaticObstacle tower;
      tower.id = route_.tower_name;
      tower.x = route_.center_x;
      tower.y = route_.center_y;
      tower.radius = route_.tower_collision_radius;
      tower.z_min = route_.minimum_height;
      tower.z_max = virtual_ceil_height_;
      obstacles_.push_back(tower);
      return;
    }
    private_node_.getParam("static_obstacles/names", names);
    private_node_.getParam("static_obstacles/x", xs);
    private_node_.getParam("static_obstacles/y", ys);
    private_node_.getParam("static_obstacles/z", zs);
    private_node_.getParam("static_obstacles/radius", radii);
    private_node_.getParam("static_obstacles/z_min", zmins);
    private_node_.getParam("static_obstacles/z_max", zmaxs);
    private_node_.getParam("static_obstacles/half_extent_x", half_extent_xs);
    private_node_.getParam("static_obstacles/half_extent_y", half_extent_ys);
    private_node_.getParam("static_obstacles/yaw", yaws);
    if (names.empty() || names.size() != xs.size() || names.size() != ys.size() ||
        names.size() != zs.size() || names.size() != radii.size()) {
      throw std::runtime_error("static_obstacles arrays must have equal nonzero length");
    }
    const auto optionalSizeValid =
        [&names](const std::vector<double>& values) {
          return values.empty() || values.size() == names.size();
        };
    if (!optionalSizeValid(zmins) || !optionalSizeValid(zmaxs) ||
        !optionalSizeValid(half_extent_xs) ||
        !optionalSizeValid(half_extent_ys) || !optionalSizeValid(yaws)) {
      throw std::runtime_error(
          "optional static_obstacles arrays must be empty or match names");
    }
    for (std::size_t i = 0; i < names.size(); ++i) {
      StaticObstacle obstacle;
      obstacle.id = names[i]; obstacle.x = xs[i]; obstacle.y = ys[i];
      obstacle.z = zs[i]; obstacle.radius = radii[i];
      obstacle.z_min = i < zmins.size() ? zmins[i] : -1.0e9;
      obstacle.z_max = i < zmaxs.size() ? zmaxs[i] : 1.0e9;
      obstacle.half_extent_x =
          i < half_extent_xs.size() ? half_extent_xs[i] : 0.0;
      obstacle.half_extent_y =
          i < half_extent_ys.size() ? half_extent_ys[i] : 0.0;
      obstacle.yaw = i < yaws.size() ? yaws[i] : 0.0;
      const bool box_requested =
          obstacle.half_extent_x > 0.0 || obstacle.half_extent_y > 0.0;
      if (!std::isfinite(obstacle.x) || !std::isfinite(obstacle.y) ||
          !std::isfinite(obstacle.radius) || obstacle.radius < 0.0 ||
          !std::isfinite(obstacle.z_min) || !std::isfinite(obstacle.z_max) ||
          obstacle.z_max <= obstacle.z_min ||
          !std::isfinite(obstacle.half_extent_x) ||
          !std::isfinite(obstacle.half_extent_y) ||
          !std::isfinite(obstacle.yaw) ||
          (box_requested &&
           (obstacle.half_extent_x <= 0.0 ||
            obstacle.half_extent_y <= 0.0))) {
        throw std::runtime_error("invalid static obstacle geometry: " +
                                 obstacle.id);
      }
      obstacles_.push_back(obstacle);
    }
    // Treat the inspected tower itself as a corridor obstacle as well as a
    // radial endpoint constraint; this prevents a local straight segment
    // from cutting through the tower between two individually safe points.
    StaticObstacle tower;
    tower.id = route_.tower_name;
    tower.x = route_.center_x;
    tower.y = route_.center_y;
    tower.radius = route_.tower_collision_radius;
    tower.z_min = route_.minimum_height;
    tower.z_max = virtual_ceil_height_;
    obstacles_.push_back(tower);
  }

  void setupRos() {
    odom_sub_ = node_.subscribe(odom_topic_, 30,
                                &Stage3EgoMissionNode::odomCallback, this);
    command_sub_ = node_.subscribe(command_topic_, 50,
                                   &Stage3EgoMissionNode::commandCallback, this);
    cloud_sub_ = node_.subscribe(cloud_topic_, 5,
                                 &Stage3EgoMissionNode::cloudCallback, this);
    occupancy_sub_ = node_.subscribe(
        occupancy_topic_, 5, &Stage3EgoMissionNode::occupancyCallback, this);
    planner_status_sub_ = node_.subscribe(
        planner_status_topic_, 10, &Stage3EgoMissionNode::plannerStatusCallback,
        this);
    bridge_state_sub_ = node_.subscribe(
        bridge_state_topic_, 10, &Stage3EgoMissionNode::bridgeStateCallback,
        this);
    goal_pub_ = node_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1);
    state_pub_ = node_.advertise<std_msgs::String>(state_topic_, 5, true);
    target_pub_ = node_.advertise<geometry_msgs::PoseStamped>(target_topic_, 1,
                                                               true);
    sector_pub_ = node_.advertise<std_msgs::UInt32>(sector_topic_, 5, true);
    candidates_pub_ = node_.advertise<astra_custom_msgs::InspectionCandidateArray>(
        candidates_topic_, 5, true);
    progress_pub_ = node_.advertise<std_msgs::Float64>(progress_topic_, 5);
    face_tower_pub_ = node_.advertise<std_msgs::Bool>(face_tower_topic_, 1, true);
    tower_center_pub_ = node_.advertise<geometry_msgs::PointStamped>(
        tower_center_topic_, 1, true);
    geometry_msgs::PointStamped tower_center;
    tower_center.header.stamp = ros::Time::now();
    tower_center.header.frame_id = route_.frame_id;
    tower_center.point.x = route_.center_x;
    tower_center.point.y = route_.center_y;
    tower_center.point.z = 0.0;
    tower_center_pub_.publish(tower_center);
    tracking_client_ = node_.serviceClient<std_srvs::SetBool>(tracking_service_);
    cancel_client_ = node_.serviceClient<std_srvs::Trigger>(cancel_service_);
    resume_client_ = node_.serviceClient<std_srvs::Trigger>(resume_service_);
    return_client_ = node_.serviceClient<std_srvs::Trigger>(return_service_);
    land_client_ = node_.serviceClient<std_srvs::Trigger>(land_service_);
    report_.open(report_file_, std::ios::out | std::ios::trunc);
    if (report_) {
      report_ << "sim_time,state,sector,target_id,target_x,target_y,target_z,"
                 "x,y,z,planner_state,planner_reason,failures,recovery_count\n";
    }
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (msg->header.frame_id != planning_frame_ || msg->header.stamp.isZero())
      return;
    if (!std::isfinite(msg->pose.pose.position.x) ||
        !std::isfinite(msg->pose.pose.position.y) ||
        !std::isfinite(msg->pose.pose.position.z)) return;
    odom_ = *msg; have_odom_ = true; odom_received_ = ros::Time::now();
  }

  void commandCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    if (msg->header.frame_id != planning_frame_ || msg->header.stamp.isZero())
      return;
    if (!std::isfinite(msg->position.x) || !std::isfinite(msg->position.y) ||
        !std::isfinite(msg->position.z)) return;
    command_ = *msg; have_command_ = true; command_received_ = ros::Time::now();
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (msg->header.frame_id != planning_frame_ || msg->header.stamp.isZero())
      return;
    cloud_points_.clear();
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
      for (; x != x.end() && cloud_points_.size() < 200000U; ++x, ++y, ++z) {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
          geometry_msgs::Point point;
          point.x = *x; point.y = *y; point.z = *z;
          cloud_points_.push_back(point);
        }
      }
    } catch (const std::exception&) { cloud_points_.clear(); }
    have_cloud_ = !cloud_points_.empty(); cloud_received_ = ros::Time::now();
  }

  void occupancyCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (msg->header.frame_id != planning_frame_ || msg->header.stamp.isZero())
      return;
    occupancy_points_.clear();
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
      for (; x != x.end() && occupancy_points_.size() < 300000U; ++x, ++y, ++z) {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
          geometry_msgs::Point point;
          point.x = *x; point.y = *y; point.z = *z;
          occupancy_points_.push_back(point);
        }
      }
    } catch (const std::exception&) { occupancy_points_.clear(); }
    have_occupancy_ = true; occupancy_received_ = ros::Time::now();
  }

  void plannerStatusCallback(const astra_custom_msgs::PlannerStatus::ConstPtr& msg) {
    planner_status_ = *msg; have_planner_status_ = true;
    planner_status_received_ = ros::Time::now();
  }

  void bridgeStateCallback(const std_msgs::String::ConstPtr& msg) {
    bridge_state_ = msg->data; have_bridge_state_ = true;
    bridge_state_received_ = ros::Time::now();
  }

  bool fresh(const ros::Time& now, const ros::Time& received, double timeout) const {
    return !received.isZero() && now >= received &&
           now - received <= ros::Duration(timeout);
  }

  geometry_msgs::PoseStamped makeGoal(const CandidatePoint& candidate) const {
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = route_.frame_id;
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = candidate.x; goal.pose.position.y = candidate.y;
    goal.pose.position.z = candidate.z; goal.pose.orientation = yawQuaternion(candidate.yaw);
    return goal;
  }

  bool buildApproachGoals(int excluded_gate_index = -1) {
    approach_goals_.clear();
    entry_gate_index_ = -1;
    if (!have_odom_ || !mapFresh(ros::Time::now())) return false;
    const geometry_msgs::Point start = pointOf(odom_);
    if (!have_home_position_) {
      home_position_ = start;
      have_home_position_ = true;
    }
    entry_gate_candidates_ = buildEntryGateCandidates(
        route_, entry_gate_angle_offsets_, entry_gate_radius_offsets_,
        inspection_height_);
    for (auto& candidate : entry_gate_candidates_) {
      evaluateEntryGateCandidate(
          &candidate, route_, start, home_position_, planningMapPoints(),
          obstacles_, mapFresh(ros::Time::now()), entry_gate_config_);
    }
    if (excluded_gate_index >= 0 &&
        excluded_gate_index <
            static_cast<int>(entry_gate_candidates_.size())) {
      entry_gate_candidates_[excluded_gate_index].accepted = false;
      entry_gate_candidates_[excluded_gate_index].rejection_reason =
          "RETRY_AFTER_PATH_FAILURE";
    }
    entry_gate_index_ = chooseBestEntryGateCandidate(entry_gate_candidates_);
    for (const auto& candidate : entry_gate_candidates_) {
      const std::string reason = candidate.rejection_reason.empty()
                                     ? ""
                                     : " reason=" + candidate.rejection_reason;
      ROS_INFO("[STAGE3_TASK] %s %s score=%.3f clearance=%.3f%s",
               candidate.id.c_str(), candidate.accepted ? "accepted" : "rejected",
               candidate.score, candidate.clearance, reason.c_str());
    }
    if (entry_gate_index_ < 0) {
      ROS_WARN("[STAGE3_TASK] no safe ENTRY_GATE candidate in fresh map");
      return false;
    }
    const CandidatePoint entry_gate =
        entry_gate_candidates_[entry_gate_index_];
    approach_goals_ = buildRollingApproachGoals(
        start, entry_gate, entry_gate_segment_length_);
    if (approach_goals_.empty()) return false;
    ROS_WARN("[STAGE3_TASK] ENTRY_GATE selected: %s (%.2f, %.2f, %.2f), "
             "rolling_goals=%zu",
             entry_gate.id.c_str(), entry_gate.x, entry_gate.y, entry_gate.z,
             approach_goals_.size());
    return true;
  }

  bool mapFresh(const ros::Time& now) const {
    return have_cloud_ && have_occupancy_ &&
           fresh(now, cloud_received_, map_timeout_) &&
           fresh(now, occupancy_received_, map_timeout_);
  }

  const std::vector<geometry_msgs::Point>& planningMapPoints() const {
    return occupancy_points_.empty() ? cloud_points_ : occupancy_points_;
  }

  void publishCandidateDebug(const ros::Time& now) {
    astra_custom_msgs::InspectionCandidateArray message;
    message.header.stamp = now; message.header.frame_id = planning_frame_;
    message.current_sector = current_sector_;
    if (current_sector_ < sectors_.size() && sectors_[current_sector_].locked_index >= 0)
      message.locked_candidate_id = sectors_[current_sector_].candidates[
          sectors_[current_sector_].locked_index].id;
    const std::size_t lookahead_end =
        std::min(sectors_.size(), current_sector_ + 2U);
    for (std::size_t sector_index = current_sector_;
         sector_index < lookahead_end; ++sector_index) {
      for (const auto& candidate : sectors_[sector_index].candidates) {
        astra_custom_msgs::InspectionCandidate item;
        item.header = message.header; item.candidate_id = candidate.id;
        item.sector_id = candidate.sector_id; item.layer_id = candidate.layer_id;
        item.target.x = candidate.x; item.target.y = candidate.y; item.target.z = candidate.z;
        item.yaw = static_cast<float>(candidate.yaw); item.accepted = candidate.accepted;
        item.rejection_reason = candidate.accepted ? candidate.risk_reason
                                                    : candidate.rejection_reason;
        item.clearance = static_cast<float>(candidate.clearance);
        item.score = static_cast<float>(candidate.score);
        item.unknown_ratio = static_cast<float>(candidate.unknown_ratio);
        message.candidates.push_back(item);
      }
    }
    candidates_pub_.publish(message);
  }

  bool lockNearestSafeInitialSector(const ros::Time& now) {
    if (sectors_.empty() || !mapFresh(now)) return false;
    const geometry_msgs::Point current = pointOf(odom_);
    std::size_t geometric_nearest = 0;
    double geometric_nearest_distance =
        std::numeric_limits<double>::infinity();

    for (std::size_t index = 0; index < sectors_.size(); ++index) {
      auto& sector = sectors_[index];
      sector.state = SectorState::kEvaluating;
      sector.locked_index = -1;
      const double nominal_x =
          sector.center_x +
          sector.nominal_radius * std::cos(sector.nominal_angle_rad);
      const double nominal_y =
          sector.center_y +
          sector.nominal_radius * std::sin(sector.nominal_angle_rad);
      const double dx = current.x - nominal_x;
      const double dy = current.y - nominal_y;
      const double dz = current.z - sector.nominal_height;
      const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (distance < geometric_nearest_distance) {
        geometric_nearest_distance = distance;
        geometric_nearest = index;
      }
      for (auto& candidate : sector.candidates) {
        evaluateCandidate(&candidate, sector, current, planningMapPoints(),
                          obstacles_, true, filter_config_, nullptr);
      }
    }

    const int safe_index =
        nearestSectorIndexWithAcceptedCandidate(current, sectors_);
    if (safe_index < 0) {
      for (auto& sector : sectors_) {
        sector.state = SectorState::kRelocating;
        sector.failure_reason = "ALL_CANDIDATES_REJECTED";
      }
      return false;
    }

    if (static_cast<std::size_t>(safe_index) != geometric_nearest) {
      int obstacle_rejections = 0;
      for (const auto& candidate : sectors_[geometric_nearest].candidates) {
        if (candidate.rejection_reason == "KNOWN_OBSTACLE_CLEARANCE") {
          ++obstacle_rejections;
        }
      }
      ROS_WARN("[STAGE3_TASK] nearest geometric sector %d is unavailable "
               "(known-obstacle rejections=%d/%zu); selecting nearest safe "
               "sector %d",
               sectors_[geometric_nearest].sector_id, obstacle_rejections,
               sectors_[geometric_nearest].candidates.size(),
               sectors_[safe_index].sector_id);
    }

    Sector& safe_sector = sectors_[safe_index];
    const int selected =
        chooseBestCandidate(safe_sector, nullptr, target_replacement_margin_);
    if (selected < 0) return false;
    safe_sector.locked_index = selected;
    safe_sector.state = SectorState::kTargetLocked;
    const int safe_sector_id = safe_sector.sector_id;
    for (std::size_t index = 0; index < sectors_.size(); ++index) {
      if (index != static_cast<std::size_t>(safe_index)) {
        sectors_[index].state = SectorState::kPending;
      }
    }
    std::rotate(sectors_.begin(), sectors_.begin() + safe_index,
                sectors_.end());
    current_sector_ = 0;
    active_target_ =
        sectors_.front().candidates[sectors_.front().locked_index];
    publishCandidateDebug(now);
    ROS_WARN("[STAGE3_TASK] initial safe sector selected: sector_id=%d, "
             "target=%s, distance_from_ENTRY_GATE=%.3f m",
             safe_sector_id, active_target_.id.c_str(),
             std::sqrt((current.x - active_target_.x) *
                           (current.x - active_target_.x) +
                       (current.y - active_target_.y) *
                           (current.y - active_target_.y) +
                       (current.z - active_target_.z) *
                           (current.z - active_target_.z)));
    transition(MissionState::kTargetLocked,
               "nearest safe inspection sector locked after ENTRY_GATE");
    return true;
  }

  bool evaluateSector(const ros::Time& now) {
    if (current_sector_ >= sectors_.size()) return false;
    auto& sector = sectors_[current_sector_];
    sector.state = SectorState::kEvaluating;
    const CandidatePoint* previous = nullptr;
    if (current_sector_ > 0 && sectors_[current_sector_ - 1].locked_index >= 0)
      previous = &sectors_[current_sector_ - 1].candidates[
          sectors_[current_sector_ - 1].locked_index];
    for (auto& candidate : sector.candidates) {
      evaluateCandidate(&candidate, sector, pointOf(odom_), planningMapPoints(),
                        obstacles_, mapFresh(now), filter_config_, previous);
    }
    // Evaluate the next sector with the same fresh map while the current
    // target is executing. It is deliberately not locked or published as a
    // goal until the current sector is covered.
    if (current_sector_ + 1U < sectors_.size()) {
      auto& next = sectors_[current_sector_ + 1U];
      next.state = SectorState::kEvaluating;
      for (auto& candidate : next.candidates) {
        evaluateCandidate(&candidate, next, pointOf(odom_), planningMapPoints(),
                          obstacles_, mapFresh(now),
                          filter_config_,
                          sector.locked_index >= 0
                              ? &sector.candidates[sector.locked_index]
                              : previous);
      }
    }
    publishCandidateDebug(now);
    const CandidatePoint* locked = sector.locked_index >= 0
                                       ? &sector.candidates[sector.locked_index]
                                       : nullptr;
    const int selected = chooseBestCandidate(sector, locked, target_replacement_margin_);
    if (selected < 0) {
      sector.failure_reason = mapFresh(now) ? "ALL_CANDIDATES_REJECTED" : "MAP_STALE";
      sector.state = SectorState::kRelocating;
      return false;
    }
    sector.locked_index = selected; sector.state = SectorState::kTargetLocked;
    active_target_ = sector.candidates[selected];
    transition(MissionState::kTargetLocked, "safe candidate locked: " + active_target_.id);
    return true;
  }

  bool requestTracking(bool enabled) {
    if (!enable_control_) return true;
    std_srvs::SetBool service; service.request.data = enabled;
    if (!tracking_client_.call(service) || !service.response.success) {
      ROS_ERROR("[STAGE3_TASK] tracking %s rejected: %s", enabled ? "resume" : "hold",
                service.response.message.c_str());
      return false;
    }
    return true;
  }

  bool requestHold(const std::string& reason) {
    failure_reason_ = reason;
    if (enable_control_) {
      std_srvs::Trigger cancel;
      const bool cancelled = cancel_client_.call(cancel) && cancel.response.success;
      if (!cancelled) ROS_WARN("[STAGE3_TASK] cancel trajectory unavailable: %s",
                               cancel.response.message.c_str());
      if (!requestTracking(false)) return false;
    }
    transition(MissionState::kHolding, reason);
    return true;
  }

  bool requestResume() {
    if (!enable_control_) return true;
    std_srvs::Trigger service;
    if (!resume_client_.call(service) || !service.response.success) {
      ROS_WARN("[STAGE3_TASK] EGO resume pending: %s",
               service.response.message.c_str());
      return requestTracking(true);
    }
    return true;
  }

  bool plannerFailure(const ros::Time& now, std::string* reason) const {
    if (!have_planner_status_ || !fresh(now, planner_status_received_, input_timeout_)) {
      *reason = "planner status stale"; return true;
    }
    if (planner_status_.current_position_in_collision) {
      *reason = astra_custom_msgs::PlannerStatus::CURRENT_POSITION_IN_OCCUPANCY;
      return true;
    }
    if (planner_status_.goal_in_collision) {
      *reason = astra_custom_msgs::PlannerStatus::GOAL_IN_OCCUPANCY;
      return true;
    }
    if (planner_status_.consecutive_plan_failures >=
            static_cast<uint32_t>(consecutive_plan_failure_limit_) ||
        (planner_status_.emergency_stop_active &&
         planner_status_.emergency_stop_duration >= emergency_stop_timeout_)) {
      *reason = planner_status_.failure_reason.empty()
                    ? astra_custom_msgs::PlannerStatus::REPLAN_FAILED
                    : planner_status_.failure_reason;
      return true;
    }
    if (have_command_ && now - command_received_ > ros::Duration(input_timeout_)) {
      *reason = astra_custom_msgs::PlannerStatus::TRAJECTORY_EXPIRED; return true;
    }
    return false;
  }

  bool buildRecovery(const ros::Time& now) {
    if (current_sector_ >= sectors_.size()) return false;
    auto& sector = sectors_[current_sector_];
    if (sector.recovery_count >= max_recovery_attempts_) return false;
    if (have_planner_status_ && planner_status_.current_position_in_collision) {
      if (!mapFresh(now)) return false;
      // A current-occupancy report is not allowed to jump directly to R1/R2.
      // Wait for a fresh map and a free vehicle neighbourhood first.
      for (const auto& point : planningMapPoints()) {
        if (std::hypot(point.x - odom_.pose.pose.position.x,
                       point.y - odom_.pose.pose.position.y) < 1.0 &&
            std::abs(point.z - odom_.pose.pose.position.z) < 1.0) return false;
      }
    }
    const CandidatePoint* locked = nullptr;
    if (sector.locked_index >= 0) {
      locked = &sector.candidates[sector.locked_index];
    } else if (active_goal_kind_ == GoalKind::kSector &&
               active_target_.sector_id == sector.sector_id) {
      locked = &active_target_;
    }
    RecoveryConfig clockwise = recovery_config_;
    clockwise.direction = OrbitDirection::kClockwise;
    RecoveryConfig counter_clockwise = recovery_config_;
    counter_clockwise.direction = OrbitDirection::kCounterClockwise;
    const RecoveryTargets clockwise_targets = makeRecoveryTargets(
        route_, pointOf(odom_), sector, clockwise, locked);
    const RecoveryTargets counter_clockwise_targets = makeRecoveryTargets(
        route_, pointOf(odom_), sector, counter_clockwise, locked);
    const RecoveryAssessment clockwise_assessment = assessRecoveryTargets(
        pointOf(odom_), clockwise_targets, planningMapPoints(), obstacles_,
        filter_config_.minimum_clearance,
        filter_config_.corridor_sample_step);
    const RecoveryAssessment counter_clockwise_assessment = assessRecoveryTargets(
        pointOf(odom_), counter_clockwise_targets, planningMapPoints(), obstacles_,
        filter_config_.minimum_clearance,
        filter_config_.corridor_sample_step);
    if (!clockwise_assessment.endpoints_safe &&
        !counter_clockwise_assessment.endpoints_safe) return false;
    const bool choose_clockwise = clockwise_assessment.endpoints_safe &&
        (!counter_clockwise_assessment.endpoints_safe ||
         clockwise_assessment.score > counter_clockwise_assessment.score);
    recovery_targets_ = choose_clockwise ? clockwise_targets
                                         : counter_clockwise_targets;
    recovery_config_.direction = choose_clockwise
                                     ? OrbitDirection::kClockwise
                                     : OrbitDirection::kCounterClockwise;
    const RecoveryAssessment& selected_assessment = choose_clockwise
                                                        ? clockwise_assessment
                                                        : counter_clockwise_assessment;
    ROS_WARN("[STAGE3_TASK] recovery selected %s; blocked straight corridors=%d "
             "(soft risk only), re-entry=%s",
             choose_clockwise ? "clockwise" : "counter_clockwise",
             selected_assessment.blocked_corridors,
             recovery_targets_.reentry.id.c_str());
    recovery_step_ = 0;
    have_sent_goal_ = false;
    arrival_since_ = ros::Time(0);
    sector.recovery_count++;
    sector.state = SectorState::kRecovering;
    transition(MissionState::kRecovering, "HOLD complete; executing R1/R2/re-entry");
    return true;
  }

  CandidatePoint recoveryTarget() const {
    if (recovery_step_ == 0) return recovery_targets_.r1;
    if (recovery_step_ == 1) return recovery_targets_.r2;
    return recovery_targets_.reentry;
  }

  void publishGoal(const CandidatePoint& target, GoalKind kind) {
    active_target_ = target; active_goal_kind_ = kind;
    geometry_msgs::PoseStamped goal = makeGoal(target);
    std_msgs::Bool face_tower;
    face_tower.data = target.face_tower;
    face_tower_pub_.publish(face_tower);
    goal_pub_.publish(goal); target_pub_.publish(goal);
    requestResume();
    goal_sent_ = ros::Time::now(); arrival_since_ = ros::Time(0);
    trajectory_baseline_ = have_command_ ? command_.trajectory_id : 0;
    have_sent_goal_ = true;
    if (kind == GoalKind::kSector) {
      transition(MissionState::kNavigate, "sector target sent to EGO");
    }
  }

  bool arrived(const ros::Time& now) const {
    if (!have_odom_) return false;
    const auto& p = odom_.pose.pose.position;
    const double distance = std::sqrt((p.x - active_target_.x) * (p.x - active_target_.x) +
                                      (p.y - active_target_.y) * (p.y - active_target_.y) +
                                      (p.z - active_target_.z) * (p.z - active_target_.z));
    const auto& v = odom_.twist.twist.linear;
    const double speed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    const double yaw = yawFromQuaternion(odom_.pose.pose.orientation);
    const bool yaw_aligned = !active_target_.require_arrival_yaw ||
        std::abs(angleError(yaw, active_target_.yaw)) <= arrival_yaw_tolerance_;
    const bool stable = distance <= arrival_tolerance_ &&
                        speed <= arrival_velocity_threshold_ && yaw_aligned;
    (void)now;
    return stable;
  }

  void transition(MissionState next, const std::string& reason) {
    if (state_ == next) return;
    ROS_WARN("[STAGE3_TASK] %s -> %s: %s", missionStateName(state_),
             missionStateName(next), reason.c_str());
    state_ = next; state_entered_ = ros::Time::now(); publishState();
  }

  void publishState() {
    std_msgs::String message; message.data = missionStateName(state_);
    state_pub_.publish(message);
    std_msgs::UInt32 sector; sector.data = static_cast<uint32_t>(current_sector_);
    sector_pub_.publish(sector);
  }

  void failTerminal(const std::string& reason) {
    failure_reason_ = reason;
    if (!enable_control_) {
      transition(MissionState::kError, reason); return;
    }
    std_srvs::Trigger land;
    if (land_client_.call(land) && land.response.success) {
      transition(MissionState::kFailureLanding, reason);
    } else {
      transition(MissionState::kError, reason + "; land rejected");
    }
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (now.isZero() || state_ == MissionState::kDone ||
        state_ == MissionState::kError) return;
    if (mission_started_.isZero()) mission_started_ = now;
    if (now - mission_started_ > ros::Duration(overall_timeout_)) {
      failTerminal("overall mission timeout"); return;
    }
    const bool active_mission_state =
        state_ == MissionState::kApproach ||
        state_ == MissionState::kEvaluate ||
        state_ == MissionState::kTargetLocked ||
        state_ == MissionState::kNavigate ||
        state_ == MissionState::kRelocating ||
        state_ == MissionState::kRecovering;
    if (enable_control_ && active_mission_state &&
        have_bridge_state_ && fresh(now, bridge_state_received_, 2.0) &&
        bridge_state_ == "HOLD") {
      // A bridge-local safety HOLD must be acknowledged by the mission before
      // the bridge's finite loss timeout can request AUTO.LAND. Cancelling
      // here turns it into a mission-supervised HOLD and preserves the normal
      // ENTRY_GATE relocation / R1-R2 recovery chain.
      if (state_ == MissionState::kApproach) {
        entry_gate_failure_pending_ = true;
      }
      requestHold(std::string("bridge entered HOLD during ") +
                  missionStateName(state_));
      return;
    }
    if (state_ == MissionState::kWaitInputs) {
      const bool bridge_ready = (!enable_control_ && bridge_state_ == "DRY_RUN") ||
                                (enable_control_ && bridge_state_ == "HOVER_READY");
      if (have_odom_ && fresh(now, odom_received_, input_timeout_) &&
          mapFresh(now) &&
          have_bridge_state_ && fresh(now, bridge_state_received_, 2.0) && bridge_ready) {
        if (!have_home_position_) {
          home_position_ = pointOf(odom_);
          have_home_position_ = true;
        }
        sectors_ = buildInspectionSectors(
            route_, sector_count_, layer_count_, sector_angle_half_width_deg_,
            sector_radius_half_width_, sector_height_half_width_, offsets_);
        if (sectors_.size() != static_cast<std::size_t>(sector_count_)) {
          failTerminal("sector generation failed");
          return;
        }
        if (!buildApproachGoals()) {
          requestHold("ENTRY_GATE has no safe fresh-map candidate");
          requestReturnOrLand("ENTRY_GATE selection failed");
          return;
        }
        transition(MissionState::kApproach,
                   "odom/map ready; rolling 3-D ENTRY_GATE selected");
      }
    } else if (state_ == MissionState::kApproach) {
      if (!have_sent_goal_) {
        publishGoal(approach_goals_[approach_index_], GoalKind::kEntryGate);
      } else {
        std::string failure;
        const bool planner_failed =
            have_planner_status_ &&
            fresh(now, planner_status_received_, input_timeout_) &&
            plannerFailure(now, &failure);
        if (planner_failed) {
          entry_gate_failure_pending_ = true;
          requestHold("ENTRY_GATE planner failure: " + failure);
        } else if (arrived(now)) {
          ++approach_index_; have_sent_goal_ = false;
          if (approach_index_ >= approach_goals_.size()) {
            if (!lockNearestSafeInitialSector(now)) {
              requestHold("ENTRY_GATE reached but no inspection sector has "
                          "a safe fresh-map candidate");
            }
          }
        } else if (now - goal_sent_ > ros::Duration(goal_timeout_)) {
          entry_gate_failure_pending_ = true;
          requestHold("ENTRY_GATE rolling subgoal timeout");
        }
      }
    } else if (state_ == MissionState::kEvaluate || state_ == MissionState::kRelocating) {
      if (!mapFresh(now)) {
        if (now - state_entered_ > ros::Duration(map_timeout_)) {
          requestHold("MAP_STALE before target selection");
          requestReturnOrLand("map stale before target selection");
        }
      } else if (evaluateSector(now)) {
        publishGoal(active_target_, GoalKind::kSector);
      } else {
        requestHold("no safe candidate in sector");
      }
    } else if (state_ == MissionState::kTargetLocked) {
      publishGoal(active_target_, GoalKind::kSector);
    } else if (state_ == MissionState::kNavigate) {
      std::string failure;
      if (plannerFailure(now, &failure)) {
        requestHold(failure);
      } else if (arrived(now)) {
        if (arrival_since_.isZero()) arrival_since_ = now;
        if (now - arrival_since_ >= ros::Duration(arrival_hold_duration_)) {
          sectors_[current_sector_].state = SectorState::kCovered;
          ++visited_sector_count_;
          ++current_sector_; have_sent_goal_ = false; arrival_since_ = ros::Time(0);
          if (visited_sector_count_ >= static_cast<std::size_t>(sector_limit_) ||
              current_sector_ >= sectors_.size()) {
            requestReturnOrLand("configured sector limit covered");
          } else {
            transition(MissionState::kEvaluate, "sector covered");
          }
        }
      } else {
        arrival_since_ = ros::Time(0);
        if (now - goal_sent_ > ros::Duration(goal_timeout_)) {
          requestHold("sector target timeout");
          if (!buildRecovery(now)) requestReturnOrLand("target timeout; recovery exhausted");
        }
      }
    } else if (state_ == MissionState::kHolding) {
      if (now - state_entered_ >= ros::Duration(failure_hold_duration_)) {
        if (entry_gate_failure_pending_) {
          entry_gate_failure_pending_ = false;
          if (buildApproachGoals(entry_gate_index_)) {
            have_sent_goal_ = false;
            transition(MissionState::kApproach,
                       "fresh map selected alternate ENTRY_GATE after HOLD");
          } else {
            requestReturnOrLand("ENTRY_GATE recovery exhausted");
          }
        } else if (failure_reason_ == astra_custom_msgs::PlannerStatus::GOAL_IN_OCCUPANCY) {
          have_sent_goal_ = false;
          transition(MissionState::kRelocating,
                     "target rejected after safety HOLD");
        } else if ((failure_reason_ == "planner status stale" ||
                    failure_reason_ == astra_custom_msgs::PlannerStatus::MAP_STALE) &&
                   !mapFresh(now)) {
          requestReturnOrLand("planner/map status remained stale after HOLD");
        } else if (failure_reason_ == astra_custom_msgs::PlannerStatus::CURRENT_POSITION_IN_OCCUPANCY &&
                   !mapFresh(now)) {
          requestReturnOrLand("current position occupancy with stale map");
        } else if (!buildRecovery(now)) {
          requestReturnOrLand(failure_reason_ + "; no safe recovery");
        }
      }
    } else if (state_ == MissionState::kRecovering) {
      CandidatePoint target = recoveryTarget();
      if (!have_sent_goal_) publishGoal(target, GoalKind::kRecovery);
      else if (arrived(now)) {
        ++recovery_step_; have_sent_goal_ = false;
        if (recovery_step_ >= 3) {
          sectors_[current_sector_].state = SectorState::kRelocating;
          transition(MissionState::kRelocating, "R1/R2/re-entry complete");
        }
      } else if (now - goal_sent_ > ros::Duration(recovery_target_timeout_)) {
        requestHold("recovery target timeout");
      }
    } else if (state_ == MissionState::kReturnHome) {
      if (bridge_state_ == "DONE") transition(MissionState::kDone, "return and landing completed");
      else if (bridge_state_ == "ERROR" || now - state_entered_ > ros::Duration(goal_timeout_))
        failTerminal("return/landing failed");
    } else if (state_ == MissionState::kFailureLanding) {
      if (bridge_state_ == "DONE" || bridge_state_ == "ERROR")
        transition(MissionState::kError, failure_reason_ + "; terminal landing state");
    }
    publishTelemetry(now);
  }

  void requestReturnOrLand(const std::string& reason) {
    failure_reason_ = reason;
    if (!enable_control_) { transition(MissionState::kError, reason); return; }
    std_srvs::Trigger service;
    if (return_client_.call(service) && service.response.success) {
      transition(MissionState::kReturnHome, reason + "; return requested");
    } else {
      ROS_ERROR("[STAGE3_TASK] return rejected: %s", service.response.message.c_str());
      std_srvs::Trigger land;
      if (land_client_.call(land) && land.response.success)
        transition(MissionState::kFailureLanding, reason + "; landing fallback");
      else transition(MissionState::kError, reason + "; return and land rejected");
    }
  }

  void publishTelemetry(const ros::Time& now) {
    std_msgs::Float64 progress;
    progress.data = sectors_.empty() ? 0.0 : static_cast<double>(current_sector_) /
                                           static_cast<double>(sectors_.size());
    progress_pub_.publish(progress);
    publishCandidateDebug(now);
    if (report_) {
      if (last_report_.isZero() || now - last_report_ >= ros::Duration(report_period_)) {
        const auto position = pointOf(odom_);
        report_ << now.toSec() << ',' << missionStateName(state_) << ','
                << current_sector_ << ',' << active_target_.id << ','
                << active_target_.x << ',' << active_target_.y << ','
                << active_target_.z << ',' << position.x << ',' << position.y
                << ',' << position.z << ','
                << (have_planner_status_ ? planner_status_.planner_state : "")
                << ',' << (have_planner_status_ ? planner_status_.failure_reason : "")
                << ',' << (have_planner_status_ ? planner_status_.consecutive_plan_failures : 0)
                << ',' << (current_sector_ < sectors_.size()
                               ? sectors_[current_sector_].recovery_count : 0)
                << '\n';
        report_.flush(); last_report_ = now;
      }
    }
  }

  ros::NodeHandle node_, private_node_;
  ros::Subscriber odom_sub_, command_sub_, cloud_sub_, occupancy_sub_, planner_status_sub_, bridge_state_sub_;
  ros::Publisher goal_pub_, state_pub_, target_pub_, sector_pub_, candidates_pub_, progress_pub_, face_tower_pub_, tower_center_pub_;
  ros::ServiceClient tracking_client_, cancel_client_, resume_client_, return_client_, land_client_;
  ros::Timer timer_;
  std::string planning_frame_, odom_topic_, command_topic_, cloud_topic_, occupancy_topic_, planner_status_topic_,
      bridge_state_topic_, goal_topic_, state_topic_, target_topic_, sector_topic_, candidates_topic_,
      progress_topic_, face_tower_topic_, tower_center_topic_, tracking_service_, cancel_service_, resume_service_, return_service_, land_service_;
  bool enable_control_{false};
  double loop_rate_{20.0}, input_timeout_{0.7}, map_timeout_{0.7}, planning_timeout_{10.0}, goal_timeout_{180.0};
  double arrival_tolerance_{0.5}, arrival_velocity_threshold_{0.2}, arrival_yaw_tolerance_{0.26}, arrival_hold_duration_{1.0};
  double tracking_error_limit_{1.2}, no_progress_window_{12.0}, no_progress_epsilon_{0.15}, emergency_stop_timeout_{6.0};
  double failure_hold_duration_{2.0}, recovery_target_timeout_{60.0}, overall_timeout_{2400.0}, report_period_{0.2};
  int consecutive_plan_failure_limit_{3}, max_recovery_attempts_{2}, sector_count_{8}, sector_limit_{1}, layer_count_{1};
  double inspection_height_{30.0}, recovery_height_max_{40.0}, virtual_ceil_height_{45.0}, transit_height_{4.0};
  double observation_radius_offset_{5.0}, observation_angle_deg_{-67.5}, approach_segment_length_{6.0};
  double sector_angle_half_width_deg_{12.0}, sector_radius_half_width_{4.0}, sector_height_half_width_{1.0};
  double target_replacement_margin_{2.0}, recovery_height_{35.0}, start_angle_deg_{-67.5};
  double entry_gate_segment_length_{6.0};
  std::string direction_, recovery_direction_, report_file_, validation_reason_;
  RouteConfig route_;
  CandidateFilterConfig filter_config_;
  RecoveryConfig recovery_config_;
  std::vector<CandidateOffset> offsets_;
  std::vector<double> entry_gate_angle_offsets_, entry_gate_radius_offsets_;
  EntryGateConfig entry_gate_config_;
  std::vector<StaticObstacle> obstacles_;
  std::vector<Sector> sectors_;
  std::vector<CandidatePoint> approach_goals_;
  std::vector<CandidatePoint> entry_gate_candidates_;
  RecoveryTargets recovery_targets_;
  std::vector<geometry_msgs::Point> cloud_points_, occupancy_points_;
  std::size_t current_sector_{0}, approach_index_{0}, recovery_step_{0}, visited_sector_count_{0};
  int entry_gate_index_{-1};
  CandidatePoint active_target_;
  GoalKind active_goal_kind_{GoalKind::kEntryGate};
  MissionState state_{MissionState::kWaitInputs};
  std::string bridge_state_, failure_reason_;
  nav_msgs::Odometry odom_;
  quadrotor_msgs::PositionCommand command_;
  astra_custom_msgs::PlannerStatus planner_status_;
  bool have_odom_{false}, have_command_{false}, have_cloud_{false}, have_occupancy_{false}, have_planner_status_{false},
      have_bridge_state_{false}, have_sent_goal_{false};
  bool have_home_position_{false}, entry_gate_failure_pending_{false};
  geometry_msgs::Point home_position_;
  ros::Time odom_received_, command_received_, cloud_received_, occupancy_received_, planner_status_received_, bridge_state_received_;
  ros::Time state_entered_, mission_started_, goal_sent_, arrival_since_, last_report_;
  std::uint32_t trajectory_baseline_{0};
  std::ofstream report_;
};

}  // namespace astra_tower_mission

int main(int argc, char** argv) {
  ros::init(argc, argv, "stage3_ego_mission");
  try {
    astra_tower_mission::Stage3EgoMissionNode node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("[STAGE3_TASK] %s", exception.what());
    return 1;
  }
  return 0;
}
