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
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>
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
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace astra_tower_mission {
namespace {

enum class MissionState {
  kWaitInputs,
  kStaging,
  kSegmentedClimb,
  kEntryGateTransit,
  kEvaluate,
  kTargetLocked,
  kNavigate,
  kHolding,
  kRelocating,
  kRecovering,
  kLayerTransition,
  kNormalReturn,
  kReturnEgress,
  kReturnHome,
  kFailureLanding,
  kDone,
  kError,
};

const char* missionStateName(MissionState state) {
  switch (state) {
    case MissionState::kWaitInputs: return "WAIT_INPUTS";
    case MissionState::kStaging: return "STAGING_POINT";
    case MissionState::kSegmentedClimb: return "SEGMENTED_CLIMB";
    case MissionState::kEntryGateTransit: return "ENTRY_GATE_TRANSIT";
    case MissionState::kEvaluate: return "EVALUATING";
    case MissionState::kTargetLocked: return "TARGET_LOCKED";
    case MissionState::kNavigate: return "NAVIGATING";
    case MissionState::kHolding: return "HOLDING";
    case MissionState::kRelocating: return "RELOCATING";
    case MissionState::kRecovering: return "RECOVERING";
    case MissionState::kLayerTransition: return "LAYER_TRANSITION";
    case MissionState::kNormalReturn: return "NORMAL_RETURN";
    case MissionState::kReturnEgress: return "RETURN_EGRESS";
    case MissionState::kReturnHome: return "RETURN_HOME";
    case MissionState::kFailureLanding: return "FAILURE_LANDING";
    case MissionState::kDone: return "DONE";
    case MissionState::kError: return "ERROR";
  }
  return "UNKNOWN";
}

enum class GoalKind {
  kStaging,
  kEntryGate,
  kSector,
  kRecovery,
  kLayerTransition,
  kNormalReturn,
  kReturnEgress
};

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
    ROS_WARN("[STAGE3_TASK] control=%s sectors=%d layers=%zu "
             "inspection_height=%.2f radius=%.2f entry_sector=%d "
             "recovery_max=%.2f virtual_ceil=%.2f; no MAVROS publisher",
             enable_control_ ? "true" : "false", sector_count_,
             inspection_heights_.size(), inspection_height_, route_.radius,
             entry_sector_, recovery_height_max_, virtual_ceil_height_);
  }

 private:
  void loadConfig() {
    private_node_.param("enable_control", enable_control_, false);
    private_node_.param<std::string>("planning_frame", planning_frame_,
                                     "camera_init");
    private_node_.param("loop_rate", loop_rate_, 20.0);
    private_node_.param("input_timeout", input_timeout_, 0.7);
    private_node_.param("entry_fcu_state_timeout",
                        entry_fcu_state_timeout_, 2.0);
    private_node_.param("map_timeout", map_timeout_, 0.7);
    private_node_.param("coverage/wait_timeout", coverage_wait_timeout_, 12.0);
    private_node_.param("coverage/angular_bin_deg", coverage_angular_bin_deg_,
                        2.0);
    private_node_.param("coverage/neighborhood_radius",
                        coverage_neighborhood_radius_, 3.0);
    private_node_.param("coverage/sample_resolution",
                        coverage_sample_resolution_, 1.0);
    private_node_.param("coverage/minimum_range", coverage_minimum_range_,
                        0.5);
    private_node_.param("coverage/maximum_range", coverage_maximum_range_,
                        40.0);
    private_node_.param("coverage/unknown_ratio_limit",
                        filter_config_.unknown_ratio_limit, 0.65);
    private_node_.param("coverage/safe_altitude_dwell",
                        safe_altitude_map_dwell_, 4.0);
    private_node_.param("coverage/sensor_min_elevation_deg",
                        sensor_min_elevation_deg_, -7.2);
    private_node_.param("coverage/sensor_max_elevation_deg",
                        sensor_max_elevation_deg_, 52.2);
    private_node_.param("planning_timeout", planning_timeout_, 10.0);
    private_node_.param("goal_timeout", goal_timeout_, 180.0);
    private_node_.param("return_timeout", return_timeout_, 300.0);
    private_node_.param("landing_timeout", landing_timeout_, 90.0);
    private_node_.param("return_egress/target_timeout",
                        return_egress_target_timeout_, 90.0);
    private_node_.param("return_egress/home_xy_tolerance",
                        return_home_xy_tolerance_, 1.5);
    private_node_.param("return_egress/orbit_radius",
                        return_egress_config_.orbit_radius, 24.0);
    private_node_.param("return_egress/transit_height",
                        return_egress_config_.transit_height, 38.0);
    double return_egress_angle_step_deg = 30.0;
    private_node_.param("return_egress/maximum_angle_step_deg",
                        return_egress_angle_step_deg, 30.0);
    return_egress_config_.maximum_angle_step_rad =
        return_egress_angle_step_deg * kPi / 180.0;
    private_node_.param("return_egress/obstacle_inflation",
                        return_egress_config_.obstacle_inflation, 2.0);
    private_node_.param("return_egress/corridor_sample_step",
                        return_egress_config_.corridor_sample_step, 0.5);
    private_node_.param("return_egress/minimum_goal_separation",
                        return_egress_config_.minimum_goal_separation, 0.5);
    private_node_.param("return_egress/maximum_retries",
                        max_return_egress_retries_, 2);
    private_node_.param("normal_return/maximum_retries",
                        max_normal_return_retries_, 2);
    private_node_.param("arrival_tolerance", arrival_tolerance_, 0.5);
    private_node_.param("arrival_velocity_threshold", arrival_velocity_threshold_, 0.2);
    private_node_.param("arrival_yaw_tolerance", arrival_yaw_tolerance_, 0.26);
    private_node_.param("arrival_hold_duration", arrival_hold_duration_, 1.0);
    private_node_.param("tracking_error_limit", tracking_error_limit_, 1.2);
    private_node_.param("no_progress_window", no_progress_window_, 12.0);
    private_node_.param("no_progress_epsilon", no_progress_epsilon_, 0.15);
    private_node_.param("consecutive_plan_failure_limit",
                        consecutive_plan_failure_limit_, 3);
    private_node_.param("planner_unreachable_attempt_limit",
                        planner_unreachable_attempt_limit_, 2);
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
    private_node_.param<std::string>("topics/mavros_state", mavros_state_topic_,
                                     "/mavros/state");
    private_node_.param<std::string>("topics/mavros_extended_state",
                                     mavros_extended_state_topic_,
                                     "/mavros/extended_state");
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
    private_node_.param("mission/inspection_heights", inspection_heights_,
                        std::vector<double>{inspection_height_});
    if (!inspection_heights_.empty()) {
      inspection_height_ = inspection_heights_.front();
    }
    private_node_.param("mission/recovery_height_max", recovery_height_max_, 40.0);
    private_node_.param("mission/virtual_ceil_height", virtual_ceil_height_, 45.0);
    private_node_.param("mission/radius", route_.radius, 14.0);
    private_node_.param("mission/sector_count", sector_count_, 8);
    private_node_.param("mission/sector_limit", sector_limit_, 1);
    private_node_.param("mission/inspection_laps", inspection_laps_, 1);
    layer_count_ = static_cast<int>(inspection_heights_.size());
    private_node_.param("mission/inspection_start_sector",
                        inspection_start_sector_, 0);
    private_node_.param("mission/transition_sector", transition_sector_,
                        inspection_start_sector_);
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
    private_node_.param("entry_gate/entry_sector", entry_sector_,
                        inspection_start_sector_);
    private_node_.param("entry_gate/radius", fixed_entry_gate_radius_,
                        route_.radius + 2.5);
    private_node_.param("entry_gate/height", fixed_entry_gate_height_,
                        inspection_height_);
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
    private_node_.param("entry_gate/target_timeout",
                        entry_gate_target_timeout_, 90.0);
    private_node_.param("entry_gate/maximum_relocations",
                        max_entry_gate_relocations_, 3);
    private_node_.param("staging/height", staging_height_, transit_height_);
    private_node_.param("staging/climb_height_step", climb_height_step_, 3.0);
    private_node_.param("staging/ascent_step_heights",
                        ascent_step_heights_,
                        std::vector<double>{10.0, 18.0, inspection_height_});
    private_node_.param("layer_transition/step_heights",
                        transition_step_heights_,
                        std::vector<double>{24.0, 22.0});
    private_node_.param("layer_transition/target_timeout",
                        layer_transition_target_timeout_, 90.0);
    private_node_.param("staging/minimum_channel_hold",
                        minimum_channel_hold_, 5.0);
    private_node_.param("staging/maximum_channel_switches",
                        max_entry_gate_relocations_, 2);
    private_node_.param("staging/use_configured_xy",
                        use_configured_staging_xy_, false);
    private_node_.param("staging/x", configured_staging_x_, 0.0);
    private_node_.param("staging/y", configured_staging_y_, 0.0);
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
    private_node_.param("candidate/unknown_is_hard_constraint",
                        filter_config_.unknown_is_hard_constraint, false);
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
    entry_gate_config_.inspection_height = inspection_height_;
    entry_gate_config_.minimum_height = route_.minimum_height;
    entry_gate_config_.maximum_height = virtual_ceil_height_;

    loadStaticObstacles();
    const auto validation = validateRouteConfig(route_, &validation_reason_);
    if (!validation || sector_count_ != 8 || sector_limit_ < 1 ||
        sector_limit_ > sector_count_ || layer_count_ < 1 ||
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
        entry_gate_target_timeout_ <= 0.0 ||
        max_entry_gate_relocations_ < 0 ||
        entry_sector_ < 0 || entry_sector_ >= sector_count_ ||
        inspection_start_sector_ != entry_sector_ ||
        transition_sector_ != entry_sector_ ||
        !std::isfinite(fixed_entry_gate_radius_) ||
        fixed_entry_gate_radius_ <= route_.tower_collision_radius ||
        std::abs(fixed_entry_gate_height_ - inspection_height_) > 1.0e-6 ||
        inspection_heights_.empty() ||
        transition_step_heights_.empty() ||
        ascent_step_heights_.empty() ||
        layer_transition_target_timeout_ <= 0.0 ||
        entry_fcu_state_timeout_ <= 0.0 ||
        return_timeout_ <= 0.0 || landing_timeout_ <= 0.0 ||
        return_egress_target_timeout_ <= 0.0 ||
        return_home_xy_tolerance_ <= 0.0 ||
        return_egress_config_.orbit_radius <=
            route_.tower_collision_radius +
                return_egress_config_.obstacle_inflation ||
        return_egress_config_.transit_height < route_.minimum_height ||
        return_egress_config_.transit_height > virtual_ceil_height_ ||
        return_egress_config_.maximum_angle_step_rad <= 0.0 ||
        return_egress_config_.maximum_angle_step_rad > kPi ||
        return_egress_config_.obstacle_inflation < 0.0 ||
        return_egress_config_.corridor_sample_step <= 0.0 ||
        return_egress_config_.minimum_goal_separation <= 0.0 ||
        max_return_egress_retries_ < 0 ||
        max_normal_return_retries_ < 0 || inspection_laps_ < 1 ||
        planner_unreachable_attempt_limit_ < 1 ||
        coverage_wait_timeout_ <= 0.0 || coverage_angular_bin_deg_ <= 0.0 ||
        safe_altitude_map_dwell_ <= 0.0 ||
        sensor_min_elevation_deg_ >= sensor_max_elevation_deg_ ||
        coverage_neighborhood_radius_ <= 0.0 ||
        coverage_sample_resolution_ <= 0.0 ||
        coverage_minimum_range_ < 0.0 ||
        coverage_maximum_range_ <= coverage_minimum_range_ ||
        filter_config_.unknown_ratio_limit < 0.0 ||
        filter_config_.unknown_ratio_limit > 1.0 ||
        staging_height_ < route_.minimum_height ||
        staging_height_ >= inspection_height_ || climb_height_step_ <= 0.0 ||
        minimum_channel_hold_ < failure_hold_duration_ ||
        (use_configured_staging_xy_ &&
         (!std::isfinite(configured_staging_x_) ||
          !std::isfinite(configured_staging_y_))) ||
        overall_timeout_ <= return_timeout_ ||
        overall_timeout_ <= landing_timeout_ ||
        recovery_height_ < route_.minimum_height ||
        recovery_height_ > recovery_height_max_) {
      throw std::runtime_error("invalid stage3 height or recovery envelope");
    }
    for (std::size_t index = 0; index < inspection_heights_.size(); ++index) {
      const double height = inspection_heights_[index];
      if (!std::isfinite(height) || height < route_.minimum_height ||
          height > route_.maximum_height ||
          (index > 0U && height >= inspection_heights_[index - 1U])) {
        throw std::runtime_error(
            "mission/inspection_heights must be finite and strictly descending");
      }
    }
    double previous_ascent_height = staging_height_;
    for (double height : ascent_step_heights_) {
      if (!std::isfinite(height) || height <= previous_ascent_height ||
          height > inspection_height_) {
        throw std::runtime_error(
            "staging/ascent_step_heights must strictly ascend to the first layer");
      }
      previous_ascent_height = height;
    }
    if (std::abs(previous_ascent_height - inspection_height_) > 1.0e-6) {
      throw std::runtime_error(
          "staging/ascent_step_heights must end at first inspection height");
    }
    double previous_transition_height = inspection_heights_.front();
    for (double height : transition_step_heights_) {
      if (!std::isfinite(height) || height >= previous_transition_height ||
          height < route_.minimum_height) {
        throw std::runtime_error(
            "layer_transition/step_heights must strictly descend");
      }
      previous_transition_height = height;
    }
    if (inspection_heights_.size() > 1U &&
        std::abs(previous_transition_height - inspection_heights_[1]) >
            1.0e-6) {
      throw std::runtime_error(
          "layer transition must end at second inspection height");
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
    fcu_state_sub_ = node_.subscribe(
        mavros_state_topic_, 10,
        &Stage3EgoMissionNode::fcuStateCallback, this);
    extended_state_sub_ = node_.subscribe(
        mavros_extended_state_topic_, 10,
        &Stage3EgoMissionNode::extendedStateCallback, this);
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
      report_ << "sim_time,state,layer,sector,target_id,target_x,target_y,target_z,"
                 "x,y,z,planner_state,planner_reason,failures,recovery_count,"
                 "lap,waypoint,lap_relocations,lap_planning_failures,"
                 "lap_recoveries,normal_return_retries\n";
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
    have_cloud_ = !cloud_points_.empty();
    cloud_received_ = ros::Time::now();
    rebuildCoverageRays();
  }

  std::int64_t coverageKey(int azimuth_bin, int elevation_bin) const {
    const int azimuth_bin_count =
        std::max(1, static_cast<int>(
                        std::round(2.0 * kPi /
                                   (coverage_angular_bin_deg_ * kPi / 180.0))));
    const int wrapped_azimuth_bin =
        ((azimuth_bin % azimuth_bin_count) + azimuth_bin_count) %
        azimuth_bin_count;
    const std::uint64_t packed =
        (static_cast<std::uint64_t>(
             static_cast<std::uint32_t>(wrapped_azimuth_bin))
         << 32) |
        static_cast<std::uint32_t>(elevation_bin);
    return static_cast<std::int64_t>(packed);
  }

  void rebuildCoverageRays() {
    coverage_rays_.clear();
    if (!have_odom_ || cloud_points_.empty()) return;
    coverage_origin_ = pointOf(odom_);
    const double bin_rad = coverage_angular_bin_deg_ * kPi / 180.0;
    for (const auto& point : cloud_points_) {
      const double dx = point.x - coverage_origin_.x;
      const double dy = point.y - coverage_origin_.y;
      const double dz = point.z - coverage_origin_.z;
      const double horizontal = std::hypot(dx, dy);
      const double range = std::hypot(horizontal, dz);
      if (!std::isfinite(range) || range < coverage_minimum_range_ ||
          range > coverage_maximum_range_) {
        continue;
      }
      const int azimuth_bin =
          static_cast<int>(std::floor(std::atan2(dy, dx) / bin_rad));
      const int elevation_bin =
          static_cast<int>(std::floor(std::atan2(dz, horizontal) / bin_rad));
      const std::int64_t key = coverageKey(azimuth_bin, elevation_bin);
      auto iterator = coverage_rays_.find(key);
      if (iterator == coverage_rays_.end() || iterator->second < range) {
        coverage_rays_[key] = range;
      }
    }
    coverage_received_ = ros::Time::now();
  }

  bool pointHasRayCoverage(const geometry_msgs::Point& point) const {
    if (coverage_rays_.empty()) return false;
    const double dx = point.x - coverage_origin_.x;
    const double dy = point.y - coverage_origin_.y;
    const double dz = point.z - coverage_origin_.z;
    const double horizontal = std::hypot(dx, dy);
    const double range = std::hypot(horizontal, dz);
    if (range < 0.25) return true;
    const double bin_rad = coverage_angular_bin_deg_ * kPi / 180.0;
    const int azimuth_bin =
        static_cast<int>(std::floor(std::atan2(dy, dx) / bin_rad));
    const int elevation_bin =
        static_cast<int>(std::floor(std::atan2(dz, horizontal) / bin_rad));
    for (int da = -1; da <= 1; ++da) {
      for (int de = -1; de <= 1; ++de) {
        const auto iterator =
            coverage_rays_.find(coverageKey(azimuth_bin + da,
                                            elevation_bin + de));
        if (iterator != coverage_rays_.end() &&
            range <= iterator->second + coverage_sample_resolution_) {
          return true;
        }
      }
    }
    return false;
  }

  double coverageUnknownRatio(const geometry_msgs::Point& center) const {
    std::size_t total = 0U;
    std::size_t unknown = 0U;
    const double radius = coverage_neighborhood_radius_;
    const double resolution = coverage_sample_resolution_;
    for (double x = -radius; x <= radius + 1.0e-9; x += resolution) {
      for (double y = -radius; y <= radius + 1.0e-9; y += resolution) {
        for (double z = -radius; z <= radius + 1.0e-9; z += resolution) {
          if (x * x + y * y + z * z > radius * radius) continue;
          geometry_msgs::Point sample = center;
          sample.x += x;
          sample.y += y;
          sample.z += z;
          ++total;
          if (!pointHasRayCoverage(sample)) ++unknown;
        }
      }
    }
    return total == 0U ? 1.0
                       : static_cast<double>(unknown) /
                             static_cast<double>(total);
  }

  double corridorUnknownRatio(const geometry_msgs::Point& from,
                              const geometry_msgs::Point& to) const {
    const double length = std::sqrt(
        (to.x - from.x) * (to.x - from.x) +
        (to.y - from.y) * (to.y - from.y) +
        (to.z - from.z) * (to.z - from.z));
    const int samples = std::max(
        1, static_cast<int>(std::ceil(
               length / coverage_neighborhood_radius_)));
    double maximum_unknown = 0.0;
    for (int index = 1; index <= samples; ++index) {
      const double ratio = static_cast<double>(index) / samples;
      geometry_msgs::Point sample;
      sample.x = from.x + ratio * (to.x - from.x);
      sample.y = from.y + ratio * (to.y - from.y);
      sample.z = from.z + ratio * (to.z - from.z);
      maximum_unknown =
          std::max(maximum_unknown, coverageUnknownRatio(sample));
    }
    return maximum_unknown;
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
    if (!have_bridge_state_ || bridge_state_ != msg->data) {
      bridge_state_entered_ = ros::Time::now();
    }
    bridge_state_ = msg->data; have_bridge_state_ = true;
    bridge_state_received_ = ros::Time::now();
  }

  void fcuStateCallback(const mavros_msgs::State::ConstPtr& msg) {
    fcu_state_ = *msg;
    have_fcu_state_ = true;
    fcu_state_received_ = ros::Time::now();
  }

  void extendedStateCallback(
      const mavros_msgs::ExtendedState::ConstPtr& msg) {
    extended_state_ = *msg;
    have_extended_state_ = true;
    extended_state_received_ = ros::Time::now();
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

  bool configureAscentChannel(int channel_index, bool reset_history) {
    if (!have_odom_ || !have_home_position_ || channel_index != 0) {
      return false;
    }
    approach_goals_.clear();
    approach_index_ = 0U;
    if (reset_history) {
      successful_ingress_goals_.clear();
      entry_gate_index_ = -1;
      provisional_entry_gate_index_ = -1;
      entry_gate_locked_ = false;
      entry_gate_candidates_.clear();
      failed_entry_gate_positions_.clear();
      entry_gate_relocations_ = 0;
      normal_return_attempted_ = false;
    }

    const geometry_msgs::Point current = pointOf(odom_);
    CandidatePoint staging;
    staging.id = "ASCENT_CHANNEL_" + std::to_string(channel_index) +
                 "_STAGING";
    staging.sector_id = -1;
    staging.x = home_position_.x;
    staging.y = home_position_.y;
    staging.z = std::max(current.z, staging_height_);
    staging.yaw = normalizeAngle(
        std::atan2(route_.center_y - staging.y,
                   route_.center_x - staging.x) -
        route_.camera_yaw_offset_rad);
    staging.require_arrival_yaw = false;
    staging.face_tower = true;

    approach_goals_ = buildVerticalGoalsAtHeights(
        staging, ascent_step_heights_, "VERTICAL_ASCENT");
    if (approach_goals_.empty()) return false;
    for (auto& goal : approach_goals_) {
      goal.id = "ASCENT_CHANNEL_" + std::to_string(channel_index) + "_" +
                goal.id;
      goal.face_tower = true;
      goal.require_arrival_yaw = false;
    }
    staging_target_ = staging;
    active_target_ = staging_target_;
    ascent_channel_index_ = channel_index;
    ascent_goal_attempts_ = 0;
    channel_selected_time_ = ros::Time::now();
    coverage_wait_started_ = ros::Time(0);
    ROS_WARN("[STAGE3_TASK] fixed vertical ascent locked at XY=(%.2f, %.2f); "
             "current_z=%.2f climb_goals=%zu (no lateral channel search)",
             staging.x, staging.y, current.z,
             approach_goals_.size());
    return true;
  }

  bool buildApproachGoals() {
    if (!have_odom_ || !mapFresh(ros::Time::now())) return false;
    if (!have_home_position_) {
      home_position_ = pointOf(odom_);
      have_home_position_ = true;
    }
    return configureAscentChannel(0, true);
  }

  bool selectAlternateAscentChannel() {
    ROS_ERROR("[STAGE3_TASK] alternate ascent channels are disabled; "
              "the fixed home XY remains locked");
    return false;
  }

  std::string staticEndpointRisk(const CandidatePoint& target) const {
    geometry_msgs::Point point;
    point.x = target.x;
    point.y = target.y;
    point.z = target.z;
    for (const auto& obstacle : obstacles_) {
      if (pointInObstacle(point, obstacle,
                          filter_config_.minimum_clearance)) {
        return obstacle.id;
      }
    }
    return std::string();
  }

  bool mappedEndpointClear(const CandidatePoint& target) const {
    if (target.z < route_.minimum_height ||
        target.z > route_.maximum_height) {
      return false;
    }
    for (const auto& occupied : planningMapPoints()) {
      const double dx = target.x - occupied.x;
      const double dy = target.y - occupied.y;
      const double dz = target.z - occupied.z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) <
          filter_config_.minimum_clearance) {
        return false;
      }
    }
    return true;
  }

  bool mappedCorridorSafe(const geometry_msgs::Point& from,
                          const geometry_msgs::Point& to) const {
    static const std::vector<StaticObstacle> no_static_obstacles;
    return lineCorridorSafe(
        from, to, planningMapPoints(), no_static_obstacles,
        filter_config_.minimum_clearance + filter_config_.cloud_inflation,
        filter_config_.corridor_sample_step);
  }

  bool climbTargetReady(const CandidatePoint& target, const ros::Time& now,
                        std::string* reason) {
    if (!mapFresh(now)) {
      *reason = astra_custom_msgs::PlannerStatus::MAP_STALE;
      return false;
    }
    if (!mappedEndpointClear(target)) {
      *reason = "ASCENT_TARGET_OCCUPIED";
      return false;
    }
    const geometry_msgs::Point current = pointOf(odom_);
    geometry_msgs::Point target_point;
    target_point.x = target.x;
    target_point.y = target.y;
    target_point.z = target.z;
    if (!mappedCorridorSafe(current, target_point)) {
      *reason = "ASCENT_PATH_BLOCKED";
      return false;
    }
    const double endpoint_unknown = coverageUnknownRatio(target_point);
    const double corridor_unknown =
        corridorUnknownRatio(current, target_point);
    current_coverage_unknown_ratio_ =
        std::max(endpoint_unknown, corridor_unknown);
    ROS_INFO_THROTTLE(
        1.0,
        "[STAGE3_TASK] climb coverage target=%s endpoint_unknown=%.3f "
        "corridor_unknown=%.3f limit=%.3f",
        target.id.c_str(), endpoint_unknown, corridor_unknown,
        filter_config_.unknown_ratio_limit);
    if (current_coverage_unknown_ratio_ >
        filter_config_.unknown_ratio_limit) {
      ROS_WARN_THROTTLE(
          2.0,
          "[STAGE3_TASK] MAP_UNKNOWN on vertical channel is distinct from "
          "occupancy; the fixed target remains locked and EGO receives the "
          "segment for bounded planning");
    }
    reason->clear();
    return true;
  }

  bool candidateInObservedSensorVolume(
      const geometry_msgs::Point& point) const {
    const geometry_msgs::Point origin = pointOf(odom_);
    const double dx = point.x - origin.x;
    const double dy = point.y - origin.y;
    const double dz = point.z - origin.z;
    const double horizontal = std::hypot(dx, dy);
    const double range = std::hypot(horizontal, dz);
    if (range < coverage_minimum_range_ || range > coverage_maximum_range_) {
      return false;
    }
    const double elevation_deg =
        std::atan2(dz, horizontal) * 180.0 / kPi;
    return elevation_deg >= sensor_min_elevation_deg_ &&
           elevation_deg <= sensor_max_elevation_deg_;
  }

  double candidateUnknownRatio(
      const geometry_msgs::Point& point) const {
    // The Livox Gazebo plugin emits only returns.  Absence of a return is not
    // an occupied voxel and the hit-only cloud cannot prove free space with
    // coverageUnknownRatio().  A candidate currently inside the configured
    // sensor frustum is observable; known occupancy is still rejected
    // independently by evaluateCandidate()/evaluateEntryGateCandidate().
    return candidateInObservedSensorVolume(point) ? 0.0 : 1.0;
  }

  bool lockFinalEntryGate(const ros::Time& now) {
    if (!mapFresh(now)) return false;
    entry_gate_candidates_.clear();
    entry_gate_candidates_.push_back(buildFixedEntryGate(
        route_, sector_count_, entry_sector_, fixed_entry_gate_radius_,
        fixed_entry_gate_height_));
    for (auto& candidate : entry_gate_candidates_) {
      geometry_msgs::Point point;
      point.x = candidate.x;
      point.y = candidate.y;
      point.z = candidate.z;
      const double unknown_ratio = candidateUnknownRatio(point);
      evaluateEntryGateCandidate(
          &candidate, route_, pointOf(odom_), home_position_,
          planningMapPoints(), obstacles_, true, entry_gate_config_,
          unknown_ratio, filter_config_.unknown_ratio_limit);
      if (!candidate.accepted) {
        ROS_ERROR("[STAGE3_TASK] fixed ENTRY_GATE rejected: id=%s "
                  "reason=%s radius=%.9f xyz=(%.2f, %.2f, %.2f) "
                  "clearance=%.3f unknown=%.3f",
                  candidate.id.c_str(),
                  candidate.rejection_reason.c_str(),
                  std::hypot(candidate.x - route_.center_x,
                             candidate.y - route_.center_y),
                  candidate.x, candidate.y, candidate.z,
                  candidate.clearance, candidate.unknown_ratio);
      }
    }
    entry_gate_index_ = entry_gate_candidates_.front().accepted ? 0 : -1;
    if (entry_gate_index_ < 0) return false;
    provisional_entry_gate_index_ = entry_gate_index_;
    provisional_entry_gate_ = entry_gate_candidates_[entry_gate_index_];
    entry_gate_locked_ = true;
    ROS_WARN("[STAGE3_TASK] fixed ENTRY_GATE locked after safe-altitude map "
             "dwell: %s sector=%d radius=%.2f xyz=(%.2f, %.2f, %.2f) "
             "unknown_ratio=%.3f",
             provisional_entry_gate_.id.c_str(),
             entry_sector_, fixed_entry_gate_radius_,
             provisional_entry_gate_.x, provisional_entry_gate_.y,
             provisional_entry_gate_.z,
             provisional_entry_gate_.unknown_ratio);
    return true;
  }

  bool startEntryGateTransit() {
    if (!have_odom_ || !entry_gate_locked_) return false;
    // Publish only the configured safe endpoint. Straight interpolation
    // points can land inside a crane voxel even when the final ENTRY_GATE is
    // valid, which turns task-layer sampling into an unintended path
    // constraint. EGO owns the static-obstacle path to this fixed gate.
    entry_gate_transit_goals_.assign(1U, provisional_entry_gate_);
    entry_gate_transit_index_ = 0U;
    ascent_goal_attempts_ = 0;
    have_sent_goal_ = false;
    arrival_since_ = ros::Time(0);
    coverage_wait_started_ = ros::Time(0);
    transition(MissionState::kEntryGateTransit,
               "safe altitude reached; final ENTRY_GATE locked for "
               "EGO-planned transit without task-layer straight-line "
               "intermediate targets");
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
    message.current_sector =
        current_sector_ < sectors_.size()
            ? static_cast<uint32_t>(sectors_[current_sector_].sector_id)
            : 0U;
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

  bool buildInspectionLayer(int layer_index) {
    if (layer_index < 0 ||
        layer_index >= static_cast<int>(inspection_heights_.size())) {
      return false;
    }
    RouteConfig layer_route = route_;
    layer_route.height = inspection_heights_[layer_index];
    sectors_ = buildInspectionSectors(
        layer_route, sector_count_, 1, sector_angle_half_width_deg_,
        sector_radius_half_width_, sector_height_half_width_, offsets_);
    if (sectors_.size() != static_cast<std::size_t>(sector_count_)) {
      return false;
    }
    const auto start = std::find_if(
        sectors_.begin(), sectors_.end(), [this](const Sector& sector) {
          return sector.sector_id == inspection_start_sector_;
        });
    if (start == sectors_.end()) return false;
    std::rotate(sectors_.begin(), start, sectors_.end());
    for (auto& sector : sectors_) {
      sector.layer_id = layer_index;
      for (auto& candidate : sector.candidates) {
        candidate.layer_id = layer_index;
        candidate.id = "l" + std::to_string(layer_index) + "_" + candidate.id;
      }
    }
    route_.height = inspection_heights_[layer_index];
    current_layer_ = layer_index;
    lap_visit_sequence_ = buildClosedLapVisitSequence(
        static_cast<std::size_t>(sector_limit_), inspection_laps_);
    visit_cursor_ = 0U;
    current_sector_ = 0U;
    current_lap_ = 1;
    waypoint_in_lap_ = 1;
    completed_laps_ = 0;
    have_last_inspection_target_ = false;
    planner_unreachable_candidates_.clear();
    ROS_WARN("[STAGE3_TASK] inspection layer %d prepared: height=%.2f "
             "start_sector=%d sequence=%d->...->%d",
             current_layer_, route_.height, inspection_start_sector_,
             inspection_start_sector_, inspection_start_sector_);
    return !lap_visit_sequence_.empty();
  }

  bool lockFixedInitialSector(const ros::Time& now,
                              bool require_transition_xy = false) {
    if (sectors_.empty() || !mapFresh(now)) return false;
    const geometry_msgs::Point current = pointOf(odom_);
    Sector& safe_sector = sectors_.front();
    safe_sector.state = SectorState::kEvaluating;
    safe_sector.locked_index = -1;
    for (auto& candidate : safe_sector.candidates) {
      geometry_msgs::Point candidate_point;
      candidate_point.x = candidate.x;
      candidate_point.y = candidate.y;
      candidate_point.z = candidate.z;
      evaluateCandidate(&candidate, safe_sector, current, planningMapPoints(),
                        obstacles_, true, filter_config_, nullptr,
                        candidateUnknownRatio(candidate_point));
    }
    int selected = -1;
    if (require_transition_xy) {
      for (std::size_t index = 0; index < safe_sector.candidates.size();
           ++index) {
        const auto& candidate = safe_sector.candidates[index];
        if (candidate.accepted &&
            std::hypot(candidate.x - layer_transition_anchor_.x,
                       candidate.y - layer_transition_anchor_.y) < 1.0e-6) {
          selected = static_cast<int>(index);
          break;
        }
      }
    } else {
      selected = chooseBestCandidate(
          safe_sector, nullptr, target_replacement_margin_);
    }
    if (selected < 0) return false;
    safe_sector.locked_index = selected;
    safe_sector.state = SectorState::kTargetLocked;
    active_target_ = safe_sector.candidates[selected];
    current_target_plan_attempt_ = 0;
    publishCandidateDebug(now);
    ROS_WARN("[STAGE3_TASK] fixed first waypoint locked: layer=%d "
             "sector_id=%d target=%s xyz=(%.2f, %.2f, %.2f) "
             "distance_from_current=%.3f m",
             current_layer_, safe_sector.sector_id,
             active_target_.id.c_str(), active_target_.x, active_target_.y,
             active_target_.z,
             std::sqrt((current.x - active_target_.x) *
                           (current.x - active_target_.x) +
                       (current.y - active_target_.y) *
                           (current.y - active_target_.y) +
                       (current.z - active_target_.z) *
                           (current.z - active_target_.z)));
    transition(MissionState::kTargetLocked,
               "configured first inspection sector locked");
    return true;
  }

  bool evaluateSector(const ros::Time& now) {
    if (current_sector_ >= sectors_.size()) return false;
    auto& sector = sectors_[current_sector_];
    sector.state = SectorState::kEvaluating;
    const CandidatePoint* previous = nullptr;
    if (have_last_inspection_target_) previous = &last_inspection_target_;
    for (auto& candidate : sector.candidates) {
      geometry_msgs::Point candidate_point;
      candidate_point.x = candidate.x;
      candidate_point.y = candidate.y;
      candidate_point.z = candidate.z;
      evaluateCandidate(&candidate, sector, pointOf(odom_), planningMapPoints(),
                        obstacles_, mapFresh(now), filter_config_, previous,
                        candidateUnknownRatio(candidate_point));
      if (planner_unreachable_candidates_.count(candidate.id) > 0U) {
        candidate.accepted = false;
        candidate.planner_unreachable = true;
        candidate.target_invalid = false;
        candidate.rejection_reason = "PLANNER_UNREACHABLE";
      }
    }
    // Evaluate the next sector with the same fresh map while the current
    // target is executing. It is deliberately not locked or published as a
    // goal until the current sector is covered.
    if (visit_cursor_ + 1U < lap_visit_sequence_.size() &&
        lap_visit_sequence_[visit_cursor_ + 1U] != current_sector_) {
      auto& next = sectors_[lap_visit_sequence_[visit_cursor_ + 1U]];
      next.state = SectorState::kEvaluating;
      for (auto& candidate : next.candidates) {
        geometry_msgs::Point candidate_point;
        candidate_point.x = candidate.x;
        candidate_point.y = candidate.y;
        candidate_point.z = candidate.z;
        evaluateCandidate(&candidate, next, pointOf(odom_), planningMapPoints(),
                          obstacles_, mapFresh(now),
                          filter_config_,
                          sector.locked_index >= 0
                              ? &sector.candidates[sector.locked_index]
                              : previous,
                          candidateUnknownRatio(candidate_point));
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
    if (active_target_.id != sector.candidates[selected].id) {
      current_target_plan_attempt_ = 0;
    }
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
    state_before_hold_ = state_;
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
    const bool current_goal_generation =
        planner_target_baseline_.empty()
            ? !planner_status_.target_id.empty()
            : planner_status_.target_id != planner_target_baseline_;
    if (!current_goal_generation) {
      if (have_sent_goal_ && !goal_sent_.isZero() &&
          now - goal_sent_ > ros::Duration(planning_timeout_)) {
        *reason = "planner goal acknowledgement timeout";
        return true;
      }
      return false;
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
    const bool current_goal_trajectory =
        have_command_ && command_.trajectory_id != trajectory_baseline_;
    if (current_goal_trajectory &&
        now - command_received_ > ros::Duration(input_timeout_)) {
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
    ++lap_recoveries_;
    sector.state = SectorState::kRecovering;
    transition(MissionState::kRecovering, "HOLD complete; executing R1/R2/re-entry");
    return true;
  }

  CandidatePoint recoveryTarget() const {
    if (recovery_step_ == 0) return recovery_targets_.r1;
    if (recovery_step_ == 1) return recovery_targets_.r2;
    return recovery_targets_.reentry;
  }

  bool entrySystemsHealthy(const ros::Time& now, std::string* reason) const {
    if (!have_odom_ || !fresh(now, odom_received_, input_timeout_)) {
      *reason = "ENTRY_CHECK_ODOM_STALE";
      return false;
    }
    if (!mapFresh(now)) {
      *reason = "ENTRY_CHECK_MAP_STALE";
      return false;
    }
    if (!have_planner_status_ ||
        !fresh(now, planner_status_received_, input_timeout_)) {
      *reason = "ENTRY_CHECK_PLANNER_STALE";
      return false;
    }
    if (planner_status_.current_position_in_collision ||
        planner_status_.goal_in_collision ||
        planner_status_.emergency_stop_active) {
      *reason = "ENTRY_CHECK_PLANNER_UNHEALTHY";
      return false;
    }
    if (enable_control_) {
      if (!have_fcu_state_) {
        *reason = "ENTRY_CHECK_PX4_MAVROS_STATE_UNAVAILABLE";
        return false;
      }
      // /mavros/state is normally published at about 1 Hz. Use a dedicated
      // deadline here; the sub-second odometry/planner timeout would
      // intermittently reject a healthy, unchanged OFFBOARD state.
      if (!fresh(now, fcu_state_received_, entry_fcu_state_timeout_)) {
        *reason = "ENTRY_CHECK_PX4_MAVROS_STATE_STALE";
        return false;
      }
      if (!fcu_state_.connected || !fcu_state_.armed ||
          fcu_state_.mode != "OFFBOARD") {
        *reason = "ENTRY_CHECK_PX4_MAVROS_OFFBOARD_UNHEALTHY";
        return false;
      }
      // The bridge state topic is latched and event-driven, not a heartbeat.
      // Its value remains authoritative until a new state transition arrives.
      if (!have_bridge_state_ ||
          (bridge_state_ != "TRACK_EGO" &&
           bridge_state_ != "HOVER_READY")) {
        *reason = "ENTRY_CHECK_BRIDGE_UNHEALTHY";
        return false;
      }
    }
    reason->clear();
    return true;
  }

  bool startLayerTransition(const ros::Time& now, std::string* reason) {
    if (current_layer_ + 1 >=
        static_cast<int>(inspection_heights_.size())) {
      *reason = "LAYER_TRANSITION_NO_NEXT_LAYER";
      return false;
    }
    if (!mapFresh(now)) {
      *reason = "LAYER_TRANSITION_MAP_STALE";
      return false;
    }
    if (active_target_.sector_id != transition_sector_) {
      *reason = "LAYER_TRANSITION_WRONG_SECTOR";
      return false;
    }
    if (std::abs(active_target_.z - inspection_heights_[current_layer_]) >
        1.0e-6) {
      *reason = "LAYER_TRANSITION_WRONG_START_HEIGHT";
      return false;
    }
    layer_transition_anchor_ = active_target_;
    layer_transition_anchor_.require_arrival_yaw = true;
    layer_transition_anchor_.face_tower = true;
    layer_transition_goals_ = buildVerticalGoalsAtHeights(
        layer_transition_anchor_, transition_step_heights_,
        "LAYER_TRANSITION");
    if (layer_transition_goals_.empty()) {
      *reason = "LAYER_TRANSITION_GOAL_GENERATION_FAILED";
      return false;
    }
    for (auto& goal : layer_transition_goals_) {
      goal.sector_id = transition_sector_;
      goal.layer_id = current_layer_ + 1;
      goal.x = layer_transition_anchor_.x;
      goal.y = layer_transition_anchor_.y;
      goal.yaw = layer_transition_anchor_.yaw;
      goal.require_arrival_yaw = true;
      goal.face_tower = true;
      if (goal.z < route_.minimum_height ||
          goal.z > route_.maximum_height) {
        *reason = "LAYER_TRANSITION_HEIGHT_OUT_OF_BOUNDS";
        return false;
      }
    }
    geometry_msgs::Point top;
    top.x = layer_transition_anchor_.x;
    top.y = layer_transition_anchor_.y;
    top.z = layer_transition_anchor_.z;
    geometry_msgs::Point bottom = top;
    bottom.z = layer_transition_goals_.back().z;
    if (!lineCorridorSafe(
            top, bottom, planningMapPoints(), obstacles_,
            filter_config_.minimum_clearance +
                filter_config_.cloud_inflation,
            filter_config_.corridor_sample_step)) {
      *reason = "LAYER_TRANSITION_VERTICAL_COLUMN_OCCUPIED";
      return false;
    }
    layer_transition_index_ = 0U;
    layer_transition_attempts_ = 0;
    have_sent_goal_ = false;
    arrival_since_ = ros::Time(0);
    ROS_WARN("[STAGE3_TASK] layer transition column accepted: layer=%d->%d "
             "sector=%d fixed_xy=(%.2f, %.2f) heights=%.2f->%.2f",
             current_layer_, current_layer_ + 1, transition_sector_,
             top.x, top.y, top.z, bottom.z);
    transition(MissionState::kLayerTransition,
               "closed upper layer; fixed-XY descent column accepted");
    reason->clear();
    return true;
  }

  bool layerTransitionSegmentSafe(const CandidatePoint& target,
                                  const ros::Time& now,
                                  std::string* reason) const {
    if (!mapFresh(now)) {
      *reason = "LAYER_TRANSITION_MAP_STALE";
      return false;
    }
    if (std::hypot(target.x - layer_transition_anchor_.x,
                   target.y - layer_transition_anchor_.y) > 1.0e-6) {
      *reason = "LAYER_TRANSITION_XY_CHANGED";
      return false;
    }
    geometry_msgs::Point from = pointOf(odom_);
    geometry_msgs::Point to;
    to.x = target.x;
    to.y = target.y;
    to.z = target.z;
    if (!lineCorridorSafe(
            from, to, planningMapPoints(), obstacles_,
            filter_config_.minimum_clearance +
                filter_config_.cloud_inflation,
            filter_config_.corridor_sample_step)) {
      *reason = "LAYER_TRANSITION_SEGMENT_OCCUPIED";
      return false;
    }
    reason->clear();
    return true;
  }

  bool startNormalReturn(const std::string& reason,
                         bool include_exit_gate = true) {
    if (successful_ingress_goals_.empty() ||
        (include_exit_gate && !entry_gate_locked_)) {
      return false;
    }
    normal_return_goals_.clear();
    if (include_exit_gate) {
      CandidatePoint exit_gate =
          entry_gate_candidates_[entry_gate_index_];
      exit_gate.id = "EXIT_GATE";
      exit_gate.require_arrival_yaw = false;
      exit_gate.face_tower = true;
      normal_return_goals_.push_back(exit_gate);
    }
    for (auto iterator = successful_ingress_goals_.rbegin();
         iterator != successful_ingress_goals_.rend(); ++iterator) {
      if (!normal_return_goals_.empty()) {
        const double separation = std::sqrt(
            (iterator->x - normal_return_goals_.back().x) *
                (iterator->x - normal_return_goals_.back().x) +
            (iterator->y - normal_return_goals_.back().y) *
                (iterator->y - normal_return_goals_.back().y) +
            (iterator->z - normal_return_goals_.back().z) *
                (iterator->z - normal_return_goals_.back().z));
        if (separation < return_egress_config_.minimum_goal_separation) {
          continue;
        }
      }
      CandidatePoint goal = *iterator;
      goal.id = "INGRESS_REVERSE_" + iterator->id;
      goal.require_arrival_yaw = false;
      normal_return_goals_.push_back(goal);
    }
    if (normal_return_goals_.empty()) return false;
    normal_return_index_ = 0U;
    normal_return_retries_ = 0;
    normal_return_attempted_ = true;
    have_sent_goal_ = false;
    arrival_since_ = ros::Time(0);
    transition(MissionState::kNormalReturn,
               reason + "; reverse ingress selected");
    return true;
  }

  bool normalReturnGoalSafe(const CandidatePoint& target,
                            const ros::Time& now) const {
    if (!mapFresh(now) || !mappedEndpointClear(target)) return false;
    geometry_msgs::Point from = pointOf(odom_);
    geometry_msgs::Point to;
    to.x = target.x;
    to.y = target.y;
    to.z = target.z;
    const bool straight_clear = lineCorridorSafe(
        from, to, planningMapPoints(), obstacles_,
        filter_config_.minimum_clearance +
            filter_config_.cloud_inflation,
        filter_config_.corridor_sample_step);
    if (!straight_clear) {
      ROS_WARN_THROTTLE(
          1.0,
          "[STAGE3_TASK] normal-return straight corridor blocked; "
          "target remains valid and is handed to EGO");
    }
    return true;
  }

  bool landingZoneSafe(const ros::Time& now) const {
    if (!mapFresh(now) || !have_odom_ || !have_planner_status_) return false;
    if (!fresh(now, planner_status_received_, input_timeout_) ||
        planner_status_.current_position_in_collision ||
        planner_status_.goal_in_collision ||
        planner_status_.emergency_stop_active) {
      return false;
    }
    const auto current = pointOf(odom_);
    if (!returnLandingNearHome(current, home_position_,
                               return_home_xy_tolerance_)) {
      return false;
    }
    for (const auto& point : planningMapPoints()) {
      const double horizontal =
          std::hypot(point.x - home_position_.x,
                     point.y - home_position_.y);
      if (horizontal < 1.0 &&
          point.z > home_position_.z + 0.2 &&
          point.z < current.z + 0.5) {
        return false;
      }
    }
    return true;
  }

  bool requestAutoLandAtHome(const ros::Time& now) {
    if (!landingZoneSafe(now)) return false;
    std_srvs::Trigger land;
    if (!land_client_.call(land) || !land.response.success) {
      ROS_ERROR("[STAGE3_TASK] AUTO.LAND rejected at home hover: %s",
                land.response.message.c_str());
      return false;
    }
    landing_requested_time_ = now;
    return true;
  }

  bool buildReturnEgress() {
    if (!have_odom_ || !have_home_position_ || entry_gate_index_ < 0 ||
        entry_gate_index_ >=
            static_cast<int>(entry_gate_candidates_.size())) {
      return false;
    }
    return_egress_goals_ = buildSafeReturnEgressGoals(
        route_, pointOf(odom_), home_position_, obstacles_,
        return_egress_config_);
    return_egress_index_ = 0;
    have_sent_goal_ = false;
    arrival_since_ = ros::Time(0);
    if (return_egress_goals_.empty()) return false;
    ROS_WARN("[STAGE3_TASK] safe return egress selected: %s, goals=%zu, "
             "orbit_radius=%.2f, transit_height=%.2f",
             return_egress_goals_.back().id.c_str(),
             return_egress_goals_.size(),
             return_egress_config_.orbit_radius,
             return_egress_config_.transit_height);
    return true;
  }

  void publishGoal(const CandidatePoint& target, GoalKind kind) {
    active_target_ = target;
    active_goal_kind_ = kind;
    // Ascent, fixed ENTRY_GATE, inspection, recovery and the fixed-XY layer
    // transition all use the tower-facing policy. The validated return path
    // keeps its velocity-facing behavior.
    active_target_.face_tower =
        kind == GoalKind::kStaging || kind == GoalKind::kEntryGate ||
        kind == GoalKind::kSector || kind == GoalKind::kRecovery ||
        kind == GoalKind::kLayerTransition;
    if (!active_target_.face_tower) {
      active_target_.require_arrival_yaw = false;
    }
    geometry_msgs::PoseStamped goal = makeGoal(active_target_);
    std_msgs::Bool face_tower;
    face_tower.data = active_target_.face_tower;
    face_tower_pub_.publish(face_tower);
    planner_target_baseline_ =
        have_planner_status_ ? planner_status_.target_id : std::string();
    trajectory_baseline_ = have_command_ ? command_.trajectory_id : 0;
    goal_pub_.publish(goal); target_pub_.publish(goal);
    ROS_WARN("[STAGE3_TASK] goal phase=%s layer=%d sector=%d id=%s "
             "xyz=(%.2f, %.2f, %.2f) face_tower=%s",
             missionStateName(state_), current_layer_,
             active_target_.sector_id, active_target_.id.c_str(),
             active_target_.x, active_target_.y, active_target_.z,
             active_target_.face_tower ? "true" : "false");
    requestResume();
    goal_sent_ = ros::Time::now(); arrival_since_ = ros::Time(0);
    best_goal_distance_ = std::numeric_limits<double>::infinity();
    last_progress_time_ = goal_sent_;
    have_sent_goal_ = true;
    if (kind == GoalKind::kSector) {
      ++current_target_plan_attempt_;
    }
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

  double activeGoalDistance() const {
    if (!have_odom_) return std::numeric_limits<double>::infinity();
    const auto& position = odom_.pose.pose.position;
    return std::sqrt(
        (position.x - active_target_.x) *
            (position.x - active_target_.x) +
        (position.y - active_target_.y) *
            (position.y - active_target_.y) +
        (position.z - active_target_.z) *
            (position.z - active_target_.z));
  }

  bool noProgressTimedOut(const ros::Time& now) {
    const double distance = activeGoalDistance();
    if (!std::isfinite(distance)) return true;
    if (!std::isfinite(best_goal_distance_) ||
        distance + no_progress_epsilon_ < best_goal_distance_) {
      best_goal_distance_ = distance;
      last_progress_time_ = now;
      return false;
    }
    return !last_progress_time_.isZero() &&
           now - last_progress_time_ >=
               ros::Duration(no_progress_window_);
  }

  bool returnEgressGoalReached() const {
    if (!have_odom_) return false;
    const auto& p = odom_.pose.pose.position;
    return std::sqrt(
               (p.x - active_target_.x) * (p.x - active_target_.x) +
               (p.y - active_target_.y) * (p.y - active_target_.y) +
               (p.z - active_target_.z) * (p.z - active_target_.z)) <=
           arrival_tolerance_;
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
    std_msgs::UInt32 sector;
    sector.data =
        current_sector_ < sectors_.size()
            ? static_cast<uint32_t>(sectors_[current_sector_].sector_id)
            : 0U;
    sector_pub_.publish(sector);
  }

  void failTerminal(const std::string& reason) {
    failure_reason_ = reason;
    if (!enable_control_) {
      transition(MissionState::kError, reason); return;
    }
    // Emergency AUTO.LAND is accepted only from a mission-supervised HOLD.
    // Establish that state explicitly so a failed return cannot leave the
    // bridge's automatic loss landing suppressed forever.
    if (bridge_state_ != "HOLD" && bridge_state_ != "HOME_HOVER") {
      std_srvs::Trigger cancel;
      cancel_client_.call(cancel);
      requestTracking(false);
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
        state_ == MissionState::kStaging ||
        state_ == MissionState::kSegmentedClimb ||
        state_ == MissionState::kEntryGateTransit ||
        state_ == MissionState::kEvaluate ||
        state_ == MissionState::kTargetLocked ||
        state_ == MissionState::kNavigate ||
        state_ == MissionState::kRelocating ||
        state_ == MissionState::kRecovering ||
        state_ == MissionState::kLayerTransition ||
        state_ == MissionState::kNormalReturn ||
        state_ == MissionState::kReturnEgress;
    if (enable_control_ && active_mission_state &&
        have_bridge_state_ && fresh(now, bridge_state_received_, 2.0) &&
        bridge_state_ == "HOLD") {
      // A bridge-local safety HOLD must be acknowledged by the mission before
      // the bridge's finite loss timeout can request AUTO.LAND. Cancelling
      // here turns it into a mission-supervised HOLD and preserves the normal
      // ENTRY_GATE relocation / R1-R2 recovery chain.
      if (state_ == MissionState::kStaging ||
          state_ == MissionState::kSegmentedClimb ||
          state_ == MissionState::kEntryGateTransit) {
        entry_gate_failure_pending_ = true;
        ascent_retry_pending_ = true;
        ++ascent_goal_attempts_;
      } else if (state_ == MissionState::kNormalReturn) {
        normal_return_failure_pending_ = true;
      } else if (state_ == MissionState::kReturnEgress) {
        return_egress_failure_pending_ = true;
      } else if (state_ == MissionState::kLayerTransition) {
        layer_transition_failure_pending_ = true;
        layer_transition_retry_pending_ = true;
        ++layer_transition_attempts_;
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
        if (!buildInspectionLayer(0)) {
          failTerminal("sector generation failed");
          return;
        }
        if (!buildApproachGoals()) {
          requestHold("ENTRY_GATE has no safe fresh-map candidate");
          requestReturnOrLand("ENTRY_GATE selection failed");
          return;
        }
        transition(MissionState::kStaging,
                   "odom/map ready; home-local ascent channel locked");
      }
    } else if (state_ == MissionState::kStaging) {
      if (!have_sent_goal_) {
        active_target_ = staging_target_;
        if (arrived(now)) {
          successful_ingress_goals_.push_back(staging_target_);
          approach_index_ = 0U;
          coverage_wait_started_ = ros::Time(0);
          transition(MissionState::kSegmentedClimb,
                     "home/alternate ascent XY reached; fixed channel locked");
        } else {
          publishGoal(staging_target_, GoalKind::kStaging);
        }
      } else {
        // EGO legitimately publishes no trajectory when a goal is already
        // inside its close-goal threshold.  Arrival therefore has precedence
        // over an expired previous PositionCommand.
        if (arrived(now)) {
          successful_ingress_goals_.push_back(staging_target_);
          have_sent_goal_ = false;
          approach_index_ = 0U;
          coverage_wait_started_ = ros::Time(0);
          transition(MissionState::kSegmentedClimb,
                     "low STAGING_POINT reached; fixed-XY segmented climb begins");
        } else {
          std::string failure;
          const bool planner_failed =
              have_planner_status_ &&
              fresh(now, planner_status_received_, input_timeout_) &&
              plannerFailure(now, &failure);
          if (planner_failed) {
            entry_gate_failure_pending_ = true;
            ascent_retry_pending_ = true;
            ++ascent_goal_attempts_;
            requestHold("STAGING_POINT planner failure: " + failure);
          } else if (noProgressTimedOut(now)) {
            entry_gate_failure_pending_ = true;
            ascent_retry_pending_ = true;
            ++ascent_goal_attempts_;
            requestHold("STAGING_POINT no progress");
          } else if (now - goal_sent_ > ros::Duration(goal_timeout_)) {
            entry_gate_failure_pending_ = true;
            ascent_retry_pending_ = true;
            ++ascent_goal_attempts_;
            requestHold("STAGING_POINT timeout");
          }
        }
      }
    } else if (state_ == MissionState::kSegmentedClimb) {
      if (approach_index_ >= approach_goals_.size()) {
        if (coverage_wait_started_.isZero()) {
          coverage_wait_started_ = now;
        } else if (now - coverage_wait_started_ >=
                   ros::Duration(safe_altitude_map_dwell_)) {
          if (lockFinalEntryGate(now) && startEntryGateTransit()) {
            coverage_wait_started_ = ros::Time(0);
          } else if (now - coverage_wait_started_ >=
                     ros::Duration(coverage_wait_timeout_)) {
            entry_gate_failure_pending_ = true;
            requestHold("ENTRY_GATE_MAP_UNKNOWN_OR_INVALID after "
                        "safe-altitude coverage dwell");
          }
        }
      } else if (!have_sent_goal_) {
        const CandidatePoint& climb_target = approach_goals_[approach_index_];
        std::string readiness_reason;
        if (!climbTargetReady(climb_target, now, &readiness_reason)) {
          if (coverage_wait_started_.isZero()) coverage_wait_started_ = now;
          const bool hard_block =
              readiness_reason == "ASCENT_TARGET_OCCUPIED" ||
              readiness_reason == "ASCENT_PATH_BLOCKED";
          if (hard_block ||
              now - coverage_wait_started_ >=
                  ros::Duration(coverage_wait_timeout_)) {
            entry_gate_failure_pending_ = true;
            ascent_retry_pending_ = false;
            requestHold(readiness_reason);
          }
        } else {
          coverage_wait_started_ = ros::Time(0);
          publishGoal(climb_target, GoalKind::kEntryGate);
        }
      } else {
        if (arrived(now)) {
          successful_ingress_goals_.push_back(
              approach_goals_[approach_index_]);
          ++approach_index_;
          ascent_goal_attempts_ = 0;
          have_sent_goal_ = false;
          coverage_wait_started_ = ros::Time(0);
        } else {
          std::string failure;
          if (plannerFailure(now, &failure)) {
            entry_gate_failure_pending_ = true;
            ascent_retry_pending_ = true;
            ++ascent_goal_attempts_;
            requestHold("segmented climb planner failure: " + failure);
          } else if (noProgressTimedOut(now)) {
            entry_gate_failure_pending_ = true;
            ascent_retry_pending_ = true;
            ++ascent_goal_attempts_;
            requestHold("segmented climb no progress");
          } else if (now - goal_sent_ > ros::Duration(goal_timeout_)) {
            entry_gate_failure_pending_ = true;
            ascent_retry_pending_ = true;
            ++ascent_goal_attempts_;
            requestHold("segmented climb target timeout");
          }
        }
      }
    } else if (state_ == MissionState::kEntryGateTransit) {
      if (entry_gate_transit_index_ >= entry_gate_transit_goals_.size()) {
        std::string entry_reason;
        if (!entrySystemsHealthy(now, &entry_reason)) {
          entry_gate_failure_pending_ = true;
          ascent_retry_pending_ = false;
          requestHold(entry_reason);
        } else if (!lockFixedInitialSector(now)) {
          entry_gate_failure_pending_ = true;
          ascent_retry_pending_ = false;
          requestHold("CONFIGURED_FIRST_WAYPOINT_UNSAFE_OR_UNREACHABLE");
        }
      } else if (!have_sent_goal_) {
        const CandidatePoint& target =
            entry_gate_transit_goals_[entry_gate_transit_index_];
        if (!mapFresh(now)) {
          if (coverage_wait_started_.isZero()) coverage_wait_started_ = now;
          if (now - coverage_wait_started_ >=
              ros::Duration(coverage_wait_timeout_)) {
            entry_gate_failure_pending_ = true;
            ascent_retry_pending_ = true;
            ++ascent_goal_attempts_;
            requestHold(astra_custom_msgs::PlannerStatus::MAP_STALE +
                        std::string(" during ENTRY_GATE transit"));
          }
        } else if (!mappedEndpointClear(target)) {
          entry_gate_failure_pending_ = true;
          ascent_retry_pending_ = false;
          requestHold("ENTRY_GATE_TARGET_OCCUPIED");
        } else {
          const std::string static_risk = staticEndpointRisk(target);
          if (!static_risk.empty()) {
            // The final gate endpoint has already passed the conservative
            // static-geometry hard filter.  A rolling intermediate point can
            // still touch the solid OBB that bounds a sparse crane mesh.  Bag
            // evidence showed such a point 4.24 m from the nearest occupied
            // map sample while only 0.038 m inside the OBB's 2 m inflation.
            // Treat that mismatch as a corridor risk and let EGO use the
            // current occupancy map instead of contradicting gate selection.
            ROS_WARN("[STAGE3_TASK] ENTRY_GATE_STATIC_CORRIDOR_RISK at %s "
                     "for %s; mapped endpoint is clear and remains locked "
                     "for EGO planning",
                     static_risk.c_str(), target.id.c_str());
          }
          coverage_wait_started_ = ros::Time(0);
          publishGoal(target, GoalKind::kEntryGate);
        }
      } else if (returnEgressGoalReached()) {
        successful_ingress_goals_.push_back(
            entry_gate_transit_goals_[entry_gate_transit_index_]);
        ++entry_gate_transit_index_;
        ascent_goal_attempts_ = 0;
        have_sent_goal_ = false;
        arrival_since_ = ros::Time(0);
      } else {
        std::string failure;
        if (plannerFailure(now, &failure)) {
          entry_gate_failure_pending_ = true;
          ascent_retry_pending_ = true;
          ++ascent_goal_attempts_;
          requestHold("ENTRY_GATE transit planner failure: " + failure);
        } else if (noProgressTimedOut(now)) {
          entry_gate_failure_pending_ = true;
          ascent_retry_pending_ = true;
          ++ascent_goal_attempts_;
          requestHold("ENTRY_GATE transit no progress");
        } else if (now - goal_sent_ >
                   ros::Duration(entry_gate_target_timeout_)) {
          entry_gate_failure_pending_ = true;
          ascent_retry_pending_ = true;
          ++ascent_goal_attempts_;
          requestHold("ENTRY_GATE transit target timeout");
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
      // Confirm a stable arrival before interpreting a completed trajectory as
      // stale. This preserves the inspection hold when traj_server naturally
      // stops publishing at a reached endpoint.
      if (arrived(now)) {
        if (arrival_since_.isZero()) arrival_since_ = now;
        if (now - arrival_since_ >= ros::Duration(arrival_hold_duration_)) {
          sectors_[current_sector_].state = SectorState::kCovered;
          last_inspection_target_ = active_target_;
          have_last_inspection_target_ = true;
          ++visited_sector_count_;
          ++visit_cursor_;
          have_sent_goal_ = false;
          arrival_since_ = ros::Time(0);
          current_target_plan_attempt_ = 0;
          if (visit_cursor_ >= lap_visit_sequence_.size()) {
            completed_laps_ = inspection_laps_;
            ++completed_layer_count_;
            if (current_layer_ + 1 <
                static_cast<int>(inspection_heights_.size())) {
              std::string transition_reason;
              if (!startLayerTransition(now, &transition_reason)) {
                layer_transition_failure_pending_ = true;
                layer_transition_retry_pending_ = false;
                requestHold(transition_reason);
              }
            } else if (buildReturnEgress()) {
              transition(
                  MissionState::kReturnEgress,
                  "final closed inspection layer completed; "
                  "tower-exterior high return selected");
            } else {
              requestReturnOrLand(
                  "safe RETURN_EGRESS could not be initialized after "
                  "final layer");
            }
          } else {
            current_sector_ = lap_visit_sequence_[visit_cursor_];
            const std::size_t visits_after_initial = visit_cursor_;
            const int next_lap = std::min(
                inspection_laps_,
                1 + static_cast<int>(
                        (visits_after_initial - 1U) /
                        static_cast<std::size_t>(sector_limit_)));
            if (next_lap != current_lap_) {
              current_lap_ = next_lap;
              lap_candidate_relocations_ = 0;
              lap_planning_failures_ = 0;
              lap_recoveries_ = 0;
            }
            waypoint_in_lap_ =
                current_sector_ == 0U
                    ? 1
                    : static_cast<int>(current_sector_) + 1;
            transition(MissionState::kEvaluate, "sector covered");
          }
        }
      } else {
        arrival_since_ = ros::Time(0);
        std::string failure;
        if (plannerFailure(now, &failure)) {
          ++lap_planning_failures_;
          if (failure ==
              astra_custom_msgs::PlannerStatus::GOAL_IN_OCCUPANCY) {
            requestHold(failure);
          } else {
            sector_retry_pending_ = true;
            requestHold("planner_unreachable attempt for " +
                        active_target_.id + ": " + failure);
          }
        } else if (noProgressTimedOut(now)) {
          sector_retry_pending_ = true;
          requestHold("planner_unreachable attempt: sector no progress");
        } else if (now - goal_sent_ > ros::Duration(goal_timeout_)) {
          sector_retry_pending_ = true;
          requestHold("planner_unreachable attempt: sector target timeout");
        }
      }
    } else if (state_ == MissionState::kLayerTransition) {
      if (layer_transition_index_ >= layer_transition_goals_.size()) {
        const int next_layer = current_layer_ + 1;
        if (!buildInspectionLayer(next_layer) ||
            !lockFixedInitialSector(now, true)) {
          layer_transition_failure_pending_ = true;
          layer_transition_retry_pending_ = false;
          requestHold(
              "LOWER_LAYER_FIRST_WAYPOINT_NOT_SAFE_AT_FIXED_XY");
        }
      } else if (!have_sent_goal_) {
        const CandidatePoint& target =
            layer_transition_goals_[layer_transition_index_];
        std::string safety_reason;
        if (!layerTransitionSegmentSafe(target, now, &safety_reason)) {
          layer_transition_failure_pending_ = true;
          layer_transition_retry_pending_ = false;
          requestHold(safety_reason);
        } else {
          ROS_WARN("[STAGE3_TASK] LAYER_TRANSITION layer=%d->%d "
                   "step=%zu/%zu sector=%d target=(%.2f, %.2f, %.2f)",
                   current_layer_, current_layer_ + 1,
                   layer_transition_index_ + 1U,
                   layer_transition_goals_.size(), transition_sector_,
                   target.x, target.y, target.z);
          publishGoal(target, GoalKind::kLayerTransition);
        }
      } else if (arrived(now)) {
        if (arrival_since_.isZero()) arrival_since_ = now;
        if (now - arrival_since_ >=
            ros::Duration(arrival_hold_duration_)) {
          ++layer_transition_index_;
          layer_transition_attempts_ = 0;
          have_sent_goal_ = false;
          arrival_since_ = ros::Time(0);
        }
      } else {
        arrival_since_ = ros::Time(0);
        std::string failure;
        if (plannerFailure(now, &failure)) {
          layer_transition_failure_pending_ = true;
          layer_transition_retry_pending_ = true;
          ++layer_transition_attempts_;
          requestHold("LAYER_TRANSITION planner failure: " + failure);
        } else if (noProgressTimedOut(now)) {
          layer_transition_failure_pending_ = true;
          layer_transition_retry_pending_ = true;
          ++layer_transition_attempts_;
          requestHold("LAYER_TRANSITION no progress");
        } else if (now - goal_sent_ >
                   ros::Duration(layer_transition_target_timeout_)) {
          layer_transition_failure_pending_ = true;
          layer_transition_retry_pending_ = true;
          ++layer_transition_attempts_;
          requestHold("LAYER_TRANSITION target timeout");
        }
      }
    } else if (state_ == MissionState::kHolding) {
      if (now - state_entered_ >= ros::Duration(failure_hold_duration_)) {
        if (layer_transition_failure_pending_) {
          layer_transition_failure_pending_ = false;
          const bool retry_same_target =
              layer_transition_retry_pending_ &&
              layer_transition_attempts_ <
                  planner_unreachable_attempt_limit_;
          layer_transition_retry_pending_ = false;
          if (retry_same_target) {
            have_sent_goal_ = false;
            transition(MissionState::kLayerTransition,
                       "bounded retry of unchanged vertical transition target");
          } else {
            requestReturnOrLand(
                failure_reason_ +
                "; fixed descent column recovery exhausted");
          }
        } else if (entry_gate_failure_pending_) {
          entry_gate_failure_pending_ = false;
          const bool retry_same_channel =
              ascent_retry_pending_ &&
              ascent_goal_attempts_ < planner_unreachable_attempt_limit_;
          ascent_retry_pending_ = false;
          if (retry_same_channel) {
            have_sent_goal_ = false;
            const MissionState retry_state =
                active_goal_kind_ == GoalKind::kStaging
                    ? MissionState::kStaging
                    : (state_before_hold_ ==
                               MissionState::kEntryGateTransit
                           ? MissionState::kEntryGateTransit
                           : MissionState::kSegmentedClimb);
            transition(retry_state,
                       "bounded retry of unchanged locked ascent target");
          } else {
            requestReturnOrLand(
                "fixed vertical ascent/ENTRY_GATE recovery exhausted; "
                "lateral channel search disabled");
          }
        } else if (normal_return_failure_pending_) {
          normal_return_failure_pending_ = false;
          ++normal_return_retries_;
          if (normal_return_retries_ <= max_normal_return_retries_ &&
              normal_return_index_ < normal_return_goals_.size() &&
              normalReturnGoalSafe(
                  normal_return_goals_[normal_return_index_], now)) {
            have_sent_goal_ = false;
            transition(MissionState::kNormalReturn,
                       "bounded normal-return retry");
          } else {
            requestReturnOrLand(
                "normal return retries exhausted; RETURN_EGRESS fallback");
          }
        } else if (sector_retry_pending_) {
          sector_retry_pending_ = false;
          if (current_target_plan_attempt_ <
              planner_unreachable_attempt_limit_) {
            have_sent_goal_ = false;
            transition(MissionState::kTargetLocked,
                       "bounded retry of unchanged safe endpoint");
          } else {
            planner_unreachable_candidates_.insert(active_target_.id);
            sectors_[current_sector_].locked_index = -1;
            ++candidate_relocations_;
            ++lap_candidate_relocations_;
            have_sent_goal_ = false;
            transition(MissionState::kRelocating,
                       "planner_unreachable confirmed after bounded attempts");
          }
        } else if (return_egress_failure_pending_) {
          return_egress_failure_pending_ = false;
          ++return_egress_retries_;
          if (return_egress_retries_ <= max_return_egress_retries_ &&
              mapFresh(now) && buildReturnEgress()) {
            transition(MissionState::kReturnEgress,
                       "fresh map rebuilt safe return egress after HOLD");
          } else {
            failTerminal("safe return egress recovery exhausted");
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
      } else if (noProgressTimedOut(now)) {
        requestHold("recovery target no progress");
      }
    } else if (state_ == MissionState::kNormalReturn) {
      if (normal_return_index_ >= normal_return_goals_.size()) {
        requestBridgeReturnOrLand("reverse ingress completed");
      } else if (!have_sent_goal_) {
        if (!normalReturnGoalSafe(normal_return_goals_[normal_return_index_],
                                  now)) {
          normal_return_failure_pending_ = true;
          requestHold("normal return endpoint invalid in latest map");
        } else {
          publishGoal(normal_return_goals_[normal_return_index_],
                      GoalKind::kNormalReturn);
        }
      } else if (returnEgressGoalReached()) {
        ++normal_return_index_;
        have_sent_goal_ = false;
        arrival_since_ = ros::Time(0);
      } else {
        std::string failure;
        if (plannerFailure(now, &failure)) {
          normal_return_failure_pending_ = true;
          requestHold("normal return planner failure: " + failure);
        } else if (noProgressTimedOut(now)) {
          normal_return_failure_pending_ = true;
          requestHold("normal return no progress");
        } else if (now - goal_sent_ >
                   ros::Duration(return_egress_target_timeout_)) {
          normal_return_failure_pending_ = true;
          requestHold("normal return target timeout");
        }
      }
    } else if (state_ == MissionState::kReturnEgress) {
      if (return_egress_index_ >= return_egress_goals_.size()) {
        requestBridgeReturnOrLand("safe return gate reached");
      } else if (!have_sent_goal_) {
        publishGoal(return_egress_goals_[return_egress_index_],
                    GoalKind::kReturnEgress);
      } else {
        std::string failure;
        // Egress goals are transit points. Advance as soon as their position
        // envelope is reached so a completed traj_server command cannot
        // expire while the task waits for an inspection-style zero-speed hold.
        if (returnEgressGoalReached()) {
          ++return_egress_index_;
          have_sent_goal_ = false;
          arrival_since_ = ros::Time(0);
          if (return_egress_index_ >= return_egress_goals_.size()) {
            requestBridgeReturnOrLand("safe return gate reached");
          }
        } else if (plannerFailure(now, &failure)) {
          return_egress_failure_pending_ = true;
          requestHold("return egress planner failure: " + failure);
        } else if (noProgressTimedOut(now)) {
          return_egress_failure_pending_ = true;
          requestHold("return egress no progress");
        } else {
          if (now - goal_sent_ >
              ros::Duration(return_egress_target_timeout_)) {
            return_egress_failure_pending_ = true;
            requestHold("return egress target timeout");
          }
        }
      }
    } else if (state_ == MissionState::kReturnHome) {
      if (bridge_state_ == "HOME_HOVER") {
        if (landing_requested_time_.isZero()) {
          if (!requestAutoLandAtHome(now) &&
              now - bridge_state_entered_ >=
                  ros::Duration(coverage_wait_timeout_)) {
            failTerminal("home hover landing checks failed");
          }
        }
      } else if (bridge_state_ == "DONE") {
        const bool terminal_fresh =
            have_fcu_state_ && have_extended_state_ &&
            fresh(now, fcu_state_received_, input_timeout_) &&
            fresh(now, extended_state_received_, input_timeout_);
        if (have_odom_ && terminal_fresh && !fcu_state_.armed &&
            extended_state_.landed_state ==
                mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND &&
            returnLandingNearHome(pointOf(odom_), home_position_,
                                  return_home_xy_tolerance_)) {
          transition(MissionState::kDone,
                     "home-proximate landing, disarm and ON_GROUND completed");
        } else {
          transition(MissionState::kError,
                     "bridge DONE without complete home/disarm/ground proof");
        }
      } else if (bridge_state_ == "ERROR") {
        failTerminal("return/landing failed");
      } else if (bridge_state_ == "HOLD") {
        failTerminal("bridge entered HOLD during final home return");
      } else {
        const bool landing_active =
            bridge_state_ == "LANDING" || bridge_state_ == "HOME_HOVER";
        const double return_elapsed = (now - state_entered_).toSec();
        const double landing_elapsed =
            landing_active ? (now - bridge_state_entered_).toSec() : 0.0;
        std::string failure;
        if (!landing_active && plannerFailure(now, &failure)) {
          failTerminal("final home return planner failure: " + failure);
          publishTelemetry(now);
          return;
        }
        if (returnOrLandingTimedOut(
                landing_active, return_elapsed, landing_elapsed,
                return_timeout_, landing_timeout_)) {
          failTerminal(landing_active ? "landing completion timeout"
                                      : "EGO return timeout");
        }
      }
    } else if (state_ == MissionState::kFailureLanding) {
      if (bridge_state_ == "DONE" || bridge_state_ == "ERROR")
        transition(MissionState::kError, failure_reason_ + "; terminal landing state");
      else if (now - state_entered_ >= ros::Duration(landing_timeout_))
        transition(MissionState::kError,
                   failure_reason_ + "; emergency landing timeout");
    }
    publishTelemetry(now);
  }

  void requestReturnOrLand(const std::string& reason) {
    failure_reason_ = reason;
    if (!enable_control_) { transition(MissionState::kError, reason); return; }
    if (!normal_return_attempted_ && !successful_ingress_goals_.empty()) {
      normal_return_attempted_ = true;
      if (startNormalReturn(reason + "; normal reverse ingress first", false)) {
        return;
      }
    }
    if (have_odom_ && have_home_position_ &&
        !returnLandingNearHome(pointOf(odom_), home_position_,
                               return_home_xy_tolerance_)) {
      return_egress_retries_ = 0;
      if (buildReturnEgress()) {
        transition(MissionState::kReturnEgress,
                   reason + "; safe tower-exterior egress started");
        return;
      }
      ROS_WARN("[STAGE3_TASK] RETURN_EGRESS unavailable; attempting bounded "
               "bridge/EGO normal home return before emergency AUTO.LAND");
      requestBridgeReturnOrLand(reason + "; RETURN_EGRESS unavailable");
      return;
    }
    requestBridgeReturnOrLand(reason);
  }

  void requestBridgeReturnOrLand(const std::string& reason) {
    failure_reason_ = reason;
    if (!enable_control_) { transition(MissionState::kError, reason); return; }
    std_srvs::Trigger service;
    planner_target_baseline_ =
        have_planner_status_ ? planner_status_.target_id : std::string();
    trajectory_baseline_ = have_command_ ? command_.trajectory_id : 0;
    if (return_client_.call(service) && service.response.success) {
      goal_sent_ = ros::Time::now();
      have_sent_goal_ = true;
      transition(MissionState::kReturnHome, reason + "; return requested");
    } else {
      ROS_ERROR("[STAGE3_TASK] return rejected: %s", service.response.message.c_str());
      failTerminal(reason + "; return rejected: " +
                   service.response.message);
    }
  }

  void publishTelemetry(const ros::Time& now) {
    std_msgs::Float64 progress;
    const double layer_progress =
        lap_visit_sequence_.size() <= 1U
            ? 0.0
            : static_cast<double>(visit_cursor_) /
                  static_cast<double>(lap_visit_sequence_.size() - 1U);
    progress.data =
        inspection_heights_.empty()
            ? 0.0
            : std::min(
                  1.0,
                  (static_cast<double>(current_layer_) + layer_progress) /
                      static_cast<double>(inspection_heights_.size()));
    progress_pub_.publish(progress);
    publishCandidateDebug(now);
    if (report_) {
      if (last_report_.isZero() || now - last_report_ >= ros::Duration(report_period_)) {
        const auto position = pointOf(odom_);
        report_ << now.toSec() << ',' << missionStateName(state_) << ','
                << current_layer_ << ','
                << (current_sector_ < sectors_.size()
                        ? sectors_[current_sector_].sector_id
                        : -1)
                << ',' << active_target_.id << ','
                << active_target_.x << ',' << active_target_.y << ','
                << active_target_.z << ',' << position.x << ',' << position.y
                << ',' << position.z << ','
                << (have_planner_status_ ? planner_status_.planner_state : "")
                << ',' << (have_planner_status_ ? planner_status_.failure_reason : "")
                << ',' << (have_planner_status_ ? planner_status_.consecutive_plan_failures : 0)
                << ',' << (current_sector_ < sectors_.size()
                               ? sectors_[current_sector_].recovery_count : 0)
                << ',' << current_lap_ << ',' << waypoint_in_lap_
                << ',' << lap_candidate_relocations_
                << ',' << lap_planning_failures_
                << ',' << lap_recoveries_
                << ',' << normal_return_retries_
                << '\n';
        report_.flush(); last_report_ = now;
      }
    }
  }

  ros::NodeHandle node_, private_node_;
  ros::Subscriber odom_sub_, command_sub_, cloud_sub_, occupancy_sub_,
      planner_status_sub_, bridge_state_sub_, fcu_state_sub_,
      extended_state_sub_;
  ros::Publisher goal_pub_, state_pub_, target_pub_, sector_pub_, candidates_pub_, progress_pub_, face_tower_pub_, tower_center_pub_;
  ros::ServiceClient tracking_client_, cancel_client_, resume_client_, return_client_, land_client_;
  ros::Timer timer_;
  std::string planning_frame_, odom_topic_, command_topic_, cloud_topic_, occupancy_topic_, planner_status_topic_,
      bridge_state_topic_, goal_topic_, state_topic_, target_topic_, sector_topic_, candidates_topic_,
      progress_topic_, face_tower_topic_, tower_center_topic_,
      mavros_state_topic_, mavros_extended_state_topic_, tracking_service_,
      cancel_service_, resume_service_, return_service_, land_service_;
  bool enable_control_{false};
  bool use_configured_staging_xy_{false};
  double loop_rate_{20.0}, input_timeout_{0.7},
      entry_fcu_state_timeout_{2.0}, map_timeout_{0.7},
      planning_timeout_{10.0}, goal_timeout_{180.0};
  double coverage_wait_timeout_{12.0}, coverage_angular_bin_deg_{2.0},
      coverage_neighborhood_radius_{3.0}, coverage_sample_resolution_{1.0},
      coverage_minimum_range_{0.5}, coverage_maximum_range_{40.0},
      current_coverage_unknown_ratio_{1.0};
  double safe_altitude_map_dwell_{4.0}, sensor_min_elevation_deg_{-7.2},
      sensor_max_elevation_deg_{52.2};
  double return_timeout_{300.0}, landing_timeout_{90.0};
  double return_egress_target_timeout_{90.0}, return_home_xy_tolerance_{1.5};
  double arrival_tolerance_{0.5}, arrival_velocity_threshold_{0.2}, arrival_yaw_tolerance_{0.26}, arrival_hold_duration_{1.0};
  double tracking_error_limit_{1.2}, no_progress_window_{12.0}, no_progress_epsilon_{0.15}, emergency_stop_timeout_{6.0};
  double failure_hold_duration_{2.0}, recovery_target_timeout_{60.0}, overall_timeout_{2400.0}, report_period_{0.2};
  int consecutive_plan_failure_limit_{3}, planner_unreachable_attempt_limit_{2},
      max_recovery_attempts_{2}, max_entry_gate_relocations_{3},
      max_return_egress_retries_{2},
      max_normal_return_retries_{2}, sector_count_{8}, sector_limit_{1},
      layer_count_{1}, inspection_laps_{1};
  double inspection_height_{30.0}, recovery_height_max_{40.0}, virtual_ceil_height_{45.0}, transit_height_{4.0};
  double observation_radius_offset_{5.0}, observation_angle_deg_{-67.5}, approach_segment_length_{6.0};
  double sector_angle_half_width_deg_{12.0}, sector_radius_half_width_{4.0}, sector_height_half_width_{1.0};
  double target_replacement_margin_{2.0}, recovery_height_{35.0}, start_angle_deg_{-67.5};
  double entry_gate_segment_length_{6.0}, entry_gate_target_timeout_{90.0};
  double fixed_entry_gate_radius_{15.0}, fixed_entry_gate_height_{26.0},
      layer_transition_target_timeout_{90.0};
  double staging_height_{4.0}, climb_height_step_{3.0},
      minimum_channel_hold_{5.0};
  double configured_staging_x_{0.0}, configured_staging_y_{0.0};
  std::string direction_, recovery_direction_, report_file_, validation_reason_;
  RouteConfig route_;
  CandidateFilterConfig filter_config_;
  RecoveryConfig recovery_config_;
  ReturnEgressConfig return_egress_config_;
  std::vector<CandidateOffset> offsets_;
  std::vector<double> inspection_heights_, ascent_step_heights_,
      transition_step_heights_;
  EntryGateConfig entry_gate_config_;
  std::vector<StaticObstacle> obstacles_;
  std::vector<Sector> sectors_;
  std::vector<CandidatePoint> approach_goals_;
  std::vector<CandidatePoint> entry_gate_transit_goals_;
  std::vector<CandidatePoint> successful_ingress_goals_;
  std::vector<CandidatePoint> normal_return_goals_;
  std::vector<CandidatePoint> return_egress_goals_;
  std::vector<CandidatePoint> entry_gate_candidates_;
  std::vector<CandidatePoint> layer_transition_goals_;
  std::vector<CandidatePoint> failed_entry_gate_positions_;
  RecoveryTargets recovery_targets_;
  std::vector<geometry_msgs::Point> cloud_points_, occupancy_points_;
  std::unordered_map<std::int64_t, double> coverage_rays_;
  std::unordered_set<std::string> planner_unreachable_candidates_;
  std::vector<std::size_t> lap_visit_sequence_;
  std::size_t current_sector_{0}, approach_index_{0}, recovery_step_{0},
      entry_gate_transit_index_{0}, return_egress_index_{0},
      normal_return_index_{0}, layer_transition_index_{0},
      visited_sector_count_{0}, visit_cursor_{0};
  int return_egress_retries_{0}, normal_return_retries_{0},
      current_target_plan_attempt_{0}, entry_gate_relocations_{0},
      current_lap_{1},
      completed_laps_{0}, waypoint_in_lap_{0}, candidate_relocations_{0},
      lap_candidate_relocations_{0}, lap_planning_failures_{0},
      lap_recoveries_{0};
  int entry_sector_{0}, inspection_start_sector_{0}, transition_sector_{0},
      current_layer_{0}, completed_layer_count_{0},
      layer_transition_attempts_{0};
  int entry_gate_index_{-1};
  int provisional_entry_gate_index_{-1};
  int ascent_channel_index_{0}, ascent_goal_attempts_{0};
  CandidatePoint provisional_entry_gate_, staging_target_,
      last_inspection_target_, layer_transition_anchor_;
  CandidatePoint active_target_;
  GoalKind active_goal_kind_{GoalKind::kEntryGate};
  MissionState state_{MissionState::kWaitInputs};
  MissionState state_before_hold_{MissionState::kWaitInputs};
  std::string bridge_state_, failure_reason_;
  nav_msgs::Odometry odom_;
  quadrotor_msgs::PositionCommand command_;
  astra_custom_msgs::PlannerStatus planner_status_;
  mavros_msgs::State fcu_state_;
  mavros_msgs::ExtendedState extended_state_;
  bool have_odom_{false}, have_command_{false}, have_cloud_{false}, have_occupancy_{false}, have_planner_status_{false},
      have_bridge_state_{false}, have_sent_goal_{false};
  bool have_home_position_{false}, entry_gate_failure_pending_{false},
      return_egress_failure_pending_{false},
      normal_return_failure_pending_{false}, sector_retry_pending_{false},
      ascent_retry_pending_{false}, normal_return_attempted_{false},
      entry_gate_locked_{false}, have_last_inspection_target_{false},
      layer_transition_failure_pending_{false},
      layer_transition_retry_pending_{false},
      have_fcu_state_{false}, have_extended_state_{false};
  geometry_msgs::Point home_position_, coverage_origin_;
  ros::Time odom_received_, command_received_, cloud_received_,
      occupancy_received_, planner_status_received_, bridge_state_received_,
      coverage_received_, fcu_state_received_, extended_state_received_;
  ros::Time state_entered_, mission_started_, goal_sent_, arrival_since_,
      last_report_, bridge_state_entered_, coverage_wait_started_,
      landing_requested_time_, channel_selected_time_, last_progress_time_;
  double best_goal_distance_{std::numeric_limits<double>::infinity()};
  std::uint32_t trajectory_baseline_{0};
  std::string planner_target_baseline_;
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
