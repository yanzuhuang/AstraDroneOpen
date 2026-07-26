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
#include <nav_msgs/Path.h>
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
  kGoToExitGate,
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
    case MissionState::kGoToExitGate: return "GO_TO_EXIT_GATE";
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
  kExitGate,
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

double positiveAngleDegrees(double angle_rad) {
  double degrees = std::fmod(angle_rad * 180.0 / kPi, 360.0);
  if (degrees < 0.0) degrees += 360.0;
  if (degrees >= 360.0 - 1.0e-9) degrees = 0.0;
  return degrees;
}

const char* entrySectorDirectionName(int entry_sector_user) {
  static const char* const names[] = {
      "+X RIGHT", "NE RIGHT_UP", "+Y UP", "NW LEFT_UP",
      "-X LEFT", "SW LEFT_DOWN", "-Y DOWN", "SE RIGHT_DOWN"};
  return entry_sector_user >= 1 && entry_sector_user <= 8
             ? names[entry_sector_user - 1]
             : "INVALID";
}

}  // namespace

class Stage3EgoMissionNode {
 public:
  Stage3EgoMissionNode() : node_(), private_node_("~") {
    loadConfig();
    logEntryGateSectorDefinitions();
    setupRos();
    state_entered_ = ros::Time::now();
    mission_started_ = state_entered_;
    publishState();
    timer_ = node_.createTimer(ros::Duration(1.0 / loop_rate_),
                               &Stage3EgoMissionNode::timerCallback, this);
    ROS_WARN("[STAGE3_TASK] control=%s sectors=%d layers=%zu cycles=%d "
             "inspection_height=%.2f radius=%.2f "
             "entry_sector_user=%d entry_sector_index=%d "
             "recovery_max=%.2f virtual_ceil=%.2f; no MAVROS publisher",
             enable_control_ ? "true" : "false", sector_count_,
             inspection_heights_.size(), planned_cycles_, inspection_height_,
             route_.radius,
             entry_sector_user_, entry_sector_index_,
             recovery_height_max_, virtual_ceil_height_);
  }

 private:
  void logEntryGateSectorDefinitions() const {
    ROS_WARN("[STAGE3_TASK] ENTRY_GATE map sectors: CCW from +X, "
             "user numbering 1..8, width=45deg");
    for (int user_sector = 1; user_sector <= 8; ++user_sector) {
      const double center_deg = (user_sector - 1) * 45.0;
      double lower_deg = center_deg - 22.5;
      double upper_deg = center_deg + 22.5;
      if (lower_deg < 0.0) lower_deg += 360.0;
      if (upper_deg >= 360.0) upper_deg -= 360.0;
      if (lower_deg > upper_deg) {
        ROS_WARN("[STAGE3_TASK] ENTRY_GATE sector %d: %-13s center=%.1fdeg "
                 "range=[%.1f,360) U [0,%.1f]deg",
                 user_sector, entrySectorDirectionName(user_sector),
                 center_deg, lower_deg, upper_deg);
      } else {
        ROS_WARN("[STAGE3_TASK] ENTRY_GATE sector %d: %-13s center=%.1fdeg "
                 "range=[%.1f,%.1f]deg",
                 user_sector, entrySectorDirectionName(user_sector),
                 center_deg, lower_deg, upper_deg);
      }
    }
    const double selected_center_deg =
        static_cast<double>(entry_sector_user_ - 1) * 45.0;
    double selected_lower_deg = selected_center_deg - 22.5;
    double selected_upper_deg = selected_center_deg + 22.5;
    if (selected_lower_deg < 0.0) selected_lower_deg += 360.0;
    if (selected_upper_deg >= 360.0) selected_upper_deg -= 360.0;
    ROS_WARN("[STAGE3_TASK] configured ENTRY_GATE sector=%d direction=%s "
             "center=%.1fdeg range=[%.1f,%.1f]deg; cross-sector fallback=false",
             entry_sector_user_, entrySectorDirectionName(entry_sector_user_),
             selected_center_deg, selected_lower_deg, selected_upper_deg);
  }

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
    private_node_.param("low_altitude/enabled", low_altitude_mode_, false);
    private_node_.param("low_altitude/height", level_path_config_.altitude,
                        3.0);
    private_node_.param("low_altitude/vertical_half_extent",
                        level_path_config_.vertical_half_extent, 0.7);
    private_node_.param("low_altitude/additional_clearance",
                        level_path_config_.additional_clearance, 0.6);
    private_node_.param("low_altitude/grid_resolution",
                        level_path_config_.resolution, 0.4);
    private_node_.param("low_altitude/boundary_margin",
                        level_path_config_.boundary_margin, 6.0);
    private_node_.param("low_altitude/maximum_segment_length",
                        level_path_config_.maximum_segment_length, 5.0);
    private_node_.param("low_altitude/altitude_tolerance",
                        low_altitude_tolerance_, 0.35);
    private_node_.param("low_altitude/no_path_confirmations",
                        low_no_path_confirmation_limit_, 3);
    private_node_.param("low_altitude/no_path_confirmation_period",
                        low_no_path_confirmation_period_, 1.0);
    private_node_.param("low_altitude/maximum_ingress_replans",
                        max_low_ingress_replans_, 5);
    private_node_.param("low_altitude/maximum_orbit_replans",
                        max_low_orbit_replans_, 12);

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
    private_node_.param<std::string>("outputs/level_ingress_path",
                                     level_ingress_path_topic_,
                                     "/tower_mission/level_ingress_path");
    private_node_.param<std::string>("outputs/level_orbit_path",
                                     level_orbit_path_topic_,
                                     "/tower_mission/level_orbit_path");
    private_node_.param<std::string>("outputs/altitude_policy",
                                     altitude_policy_topic_,
                                     "/tower_mission/altitude_policy");
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
    std::vector<double> layer_offsets;
    if (!private_node_.getParam("mission/inspection_top_height",
                                inspection_height_) ||
        !private_node_.getParam("mission/layer_offsets", layer_offsets)) {
      throw std::runtime_error(
          "mission/inspection_top_height and mission/layer_offsets are required");
    }
    inspection_heights_ =
        deriveInspectionHeights(inspection_height_, layer_offsets);
    private_node_.param("mission/recovery_height_max", recovery_height_max_, 40.0);
    private_node_.param("mission/virtual_ceil_height", virtual_ceil_height_, 45.0);
    private_node_.param("mission/radius", route_.radius, 14.0);
    private_node_.param("mission/sector_count", sector_count_, 8);
    private_node_.param("mission/sector_limit", sector_limit_, 1);
    private_node_.param("mission/inspection_laps", inspection_laps_, 1);
    private_node_.param("mission/planned_cycles", planned_cycles_, 1);
    layer_count_ = static_cast<int>(inspection_heights_.size());
    private_node_.param("mission/inspection_start_sector",
                        inspection_start_sector_, 0);
    private_node_.param("mission/transition_sector", transition_sector_,
                        inspection_start_sector_);
    private_node_.param("mission/start_angle_deg", start_angle_deg_, 0.0);
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
    private_node_.param("entry_gate/entry_sector", entry_sector_user_, 1);
    entry_sector_index_ = entry_sector_user_ - 1;
    private_node_.param("entry_gate/preferred_radius",
                        entry_gate_config_.preferred_radius,
                        15.0);
    private_node_.param("entry_gate/minimum_radius",
                        entry_gate_config_.minimum_radius, 15.0);
    private_node_.param("entry_gate/maximum_radius",
                        entry_gate_config_.maximum_radius, 15.0);
    private_node_.param("entry_gate/angular_sample_step_deg",
                        entry_gate_config_.angular_sample_step_deg, 7.5);
    private_node_.param("entry_gate/radial_sample_step",
                        entry_gate_config_.radial_sample_step, 1.0);
    private_node_.param("entry_gate/maximum_horizontal_distance",
                        entry_gate_config_.maximum_horizontal_distance, 60.0);
    private_node_.param("entry_gate/minimum_clearance",
                        entry_gate_config_.minimum_clearance, 2.0);
    private_node_.param("entry_gate/cloud_inflation",
                        entry_gate_config_.cloud_inflation, 0.4);
    private_node_.param("entry_gate/corridor_sample_step",
                        entry_gate_config_.corridor_sample_step, 0.5);
    private_node_.param("entry_gate/clearance_weight",
                        entry_gate_config_.clearance_weight, 0.2);
    private_node_.param("entry_gate/distance_weight",
                        entry_gate_config_.distance_weight, 0.1);
    private_node_.param("entry_gate/sector_center_weight",
                        entry_gate_config_.sector_center_weight, 4.0);
    private_node_.param("entry_gate/preferred_radius_weight",
                        entry_gate_config_.preferred_radius_weight, 4.0);
    private_node_.param("entry_gate/first_waypoint_weight",
                        entry_gate_config_.first_waypoint_weight, 0.8);
    private_node_.param("entry_gate/blocked_corridor_penalty",
                        entry_gate_config_.blocked_corridor_penalty, 8.0);
    private_node_.param("entry_gate/maximum_segment_length",
                        entry_gate_segment_length_, 6.0);
    private_node_.param("entry_gate/prefer_safe_overflight",
                        prefer_safe_overflight_, true);
    private_node_.param("entry_gate/target_timeout",
                        entry_gate_target_timeout_, 90.0);
    private_node_.param("entry_gate/maximum_relocations",
                        max_entry_gate_relocations_, 3);
    private_node_.param("staging/height", staging_height_, transit_height_);
    private_node_.param("staging/climb_height_step", climb_height_step_, 3.0);
    private_node_.param("staging/ascent_intermediate_heights",
                        ascent_step_heights_,
                        std::vector<double>{10.0, 18.0});
    ascent_step_heights_.push_back(inspection_height_);
    private_node_.param("layer_transition/target_timeout",
                        layer_transition_target_timeout_, 90.0);
    private_node_.param("layer_transition/maximum_vertical_step",
                        layer_transition_maximum_vertical_step_, 2.0);
    private_node_.param("layer_transition/same_xy_tolerance",
                        layer_transition_same_xy_tolerance_, 0.25);
    private_node_.param("staging/minimum_channel_hold",
                        minimum_channel_hold_, 5.0);
    private_node_.param("staging/maximum_channel_switches",
                        max_ascent_channel_switches_, 0);
    private_node_.param("staging/use_configured_xy",
                        use_configured_staging_xy_, false);
    private_node_.param("staging/x", configured_staging_x_, 0.0);
    private_node_.param("staging/y", configured_staging_y_, 0.0);
    private_node_.param("mission/sector_angle_half_width_deg",
                        sector_angle_half_width_deg_, 12.0);
    private_node_.param("mission/sector_radius_half_width", sector_radius_half_width_,
                        4.0);
    private_node_.param("mission/sector_height_half_width", sector_height_half_width_,
                        3.0);
    private_node_.param("mission/maximum_temporary_descent",
                        maximum_temporary_descent_, 3.0);
    private_node_.param("mission/target_replacement_margin",
                        target_replacement_margin_, 2.0);

    private_node_.param("candidate/minimum_clearance", filter_config_.minimum_clearance,
                        2.0);
    private_node_.param("candidate/cloud_inflation", filter_config_.cloud_inflation,
                        0.4);
    private_node_.param("candidate/unknown_is_hard_constraint",
                        filter_config_.unknown_is_hard_constraint, false);
    private_node_.param("candidate/known_obstacle_is_hard_constraint",
                        filter_config_.known_obstacle_is_hard_constraint,
                        true);
    private_node_.param("candidate/corridor_sample_step",
                        filter_config_.corridor_sample_step, 0.5);
    private_node_.param("candidate/blocked_corridor_penalty",
                        filter_config_.blocked_corridor_penalty, 5.0);
    private_node_.param("candidate/score_clearance_weight",
                        filter_config_.score_clearance_weight, 0.2);
    private_node_.param("candidate/score_radius_weight",
                        filter_config_.score_radius_weight, 6.0);
    private_node_.param("candidate/score_sector_weight",
                        filter_config_.score_sector_weight, 4.0);
    private_node_.param("candidate/score_height_weight",
                        filter_config_.score_height_weight, 14.0);
    private_node_.param("candidate/score_distance_weight",
                        filter_config_.score_distance_weight, 0.1);
    private_node_.param("candidate/score_continuity_weight",
                        filter_config_.score_continuity_weight, 0.2);
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
    recovery_config_.maximum_radius = std::min(
        recovery_config_.maximum_radius,
        route_.radius + sector_radius_half_width_);
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
                        std::vector<double>{0.0, -1.0, -2.0, -3.0});
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
        !std::isfinite(entry_gate_config_.minimum_radius) ||
        !std::isfinite(entry_gate_config_.maximum_radius) ||
        !std::isfinite(entry_gate_config_.preferred_radius) ||
        !std::isfinite(entry_gate_config_.angular_sample_step_deg) ||
        !std::isfinite(entry_gate_config_.radial_sample_step) ||
        !std::isfinite(entry_gate_config_.clearance_weight) ||
        !std::isfinite(entry_gate_config_.distance_weight) ||
        !std::isfinite(entry_gate_config_.sector_center_weight) ||
        !std::isfinite(entry_gate_config_.preferred_radius_weight) ||
        !std::isfinite(entry_gate_config_.first_waypoint_weight) ||
        !std::isfinite(entry_gate_config_.blocked_corridor_penalty) ||
        entry_gate_config_.minimum_radius <= route_.tower_collision_radius ||
        entry_gate_config_.maximum_radius < entry_gate_config_.minimum_radius ||
        entry_gate_config_.preferred_radius <
            entry_gate_config_.minimum_radius ||
        entry_gate_config_.preferred_radius >
            entry_gate_config_.maximum_radius ||
        entry_gate_config_.angular_sample_step_deg <= 0.0 ||
        entry_gate_config_.angular_sample_step_deg > 22.5 ||
        entry_gate_config_.radial_sample_step <= 0.0 ||
        entry_gate_config_.clearance_weight < 0.0 ||
        entry_gate_config_.distance_weight < 0.0 ||
        entry_gate_config_.sector_center_weight < 0.0 ||
        entry_gate_config_.preferred_radius_weight < 0.0 ||
        entry_gate_config_.first_waypoint_weight < 0.0 ||
        entry_gate_config_.blocked_corridor_penalty < 0.0 ||
        std::abs(entry_gate_config_.minimum_radius - 15.0) > 1.0e-9 ||
        std::abs(entry_gate_config_.maximum_radius - 15.0) > 1.0e-9 ||
        std::abs(entry_gate_config_.preferred_radius - 15.0) > 1.0e-9 ||
        entry_gate_segment_length_ <= 0.0 ||
        maximum_temporary_descent_ < 1.0 ||
        maximum_temporary_descent_ > 3.0 ||
        sector_height_half_width_ + 1.0e-9 <
            maximum_temporary_descent_ ||
        entry_gate_target_timeout_ <= 0.0 ||
        max_entry_gate_relocations_ < 0 ||
        max_ascent_channel_switches_ != 0 ||
        entry_sector_user_ < 1 || entry_sector_user_ > 8 ||
        entry_sector_index_ != entry_sector_user_ - 1 ||
        std::abs(normalizeAngle(route_.start_angle_rad)) > 1.0e-9 ||
        inspection_start_sector_ < 0 ||
        inspection_start_sector_ >= sector_count_ ||
        transition_sector_ < 0 || transition_sector_ >= sector_count_ ||
        inspection_heights_.empty() ||
        ascent_step_heights_.empty() ||
        layer_transition_target_timeout_ <= 0.0 ||
        layer_transition_maximum_vertical_step_ <= 0.0 ||
        layer_transition_same_xy_tolerance_ < 0.0 ||
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
        planned_cycles_ < 1 ||
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
    if (offsets_.empty()) {
      throw std::runtime_error("candidate offsets must not be empty");
    }
    for (const auto& offset : offsets_) {
      if (!std::isfinite(offset.height_m) || offset.height_m > 1.0e-9 ||
          offset.height_m < -maximum_temporary_descent_ - 1.0e-9) {
        throw std::runtime_error(
            "candidate height offsets must stay within the configured "
            "downward envelope");
      }
    }
    if (low_altitude_mode_) {
      const bool low_envelope_valid =
          inspection_heights_.size() == 1U &&
          std::abs(inspection_height_ - level_path_config_.altitude) <=
              1.0e-6 &&
          std::abs(transit_height_ - level_path_config_.altitude) <=
              1.0e-6 &&
          level_path_config_.vertical_half_extent > 0.0 &&
          level_path_config_.additional_clearance >= 0.0 &&
          level_path_config_.resolution > 0.0 &&
          level_path_config_.boundary_margin > 0.0 &&
          level_path_config_.maximum_segment_length > 0.0 &&
          low_altitude_tolerance_ > 0.0 &&
          low_no_path_confirmation_limit_ >= 1 &&
          low_no_path_confirmation_period_ > 0.0 &&
          max_low_ingress_replans_ >= 1 &&
          max_low_orbit_replans_ >= 1;
      const bool nominal_height_only = std::all_of(
          offsets_.begin(), offsets_.end(), [](const CandidateOffset& offset) {
            return std::abs(offset.height_m) <= 1.0e-9;
          });
      if (!low_envelope_valid || !nominal_height_only) {
        throw std::runtime_error(
            "low-altitude mode requires one nominal-height-only 3 m layer");
      }
    }
    for (std::size_t index = 0; index < inspection_heights_.size(); ++index) {
      const double height = inspection_heights_[index];
      if (!std::isfinite(height) || height < route_.minimum_height ||
          height > route_.maximum_height ||
          (index > 0U && height >= inspection_heights_[index - 1U])) {
        throw std::runtime_error(
            "derived inspection heights must be finite and strictly descending");
      }
    }
    double previous_ascent_height = staging_height_;
    for (double height : ascent_step_heights_) {
      if (!std::isfinite(height) || height <= previous_ascent_height ||
          height > inspection_height_) {
        throw std::runtime_error(
            "derived ascent heights must strictly ascend to the first layer");
      }
      previous_ascent_height = height;
    }
    if (std::abs(previous_ascent_height - inspection_height_) > 1.0e-6) {
      throw std::runtime_error(
          "derived ascent heights must end at first inspection height");
    }
    layer_visit_sequence_ =
        buildLayerVisitSequence(inspection_heights_.size(), planned_cycles_);
    if (layer_visit_sequence_.empty() ||
        layer_visit_sequence_.front() != 0) {
      throw std::runtime_error("mission layer visit sequence is empty");
    }
    layer_sector_data_.resize(inspection_heights_.size());
    layer_sector_runtime_.resize(inspection_heights_.size());
    layer_entry_gates_.resize(inspection_heights_.size());
    layer_entry_gate_valid_.assign(inspection_heights_.size(), false);
    layer_start_anchors_.resize(inspection_heights_.size());
    layer_start_anchor_valid_.assign(inspection_heights_.size(), false);
    for (std::size_t layer = 0U; layer < inspection_heights_.size(); ++layer) {
      RouteConfig layer_route = route_;
      layer_route.height = inspection_heights_[layer];
      layer_sector_data_[layer] = buildInspectionSectors(
          layer_route, sector_count_, 1, sector_angle_half_width_deg_,
          sector_radius_half_width_, sector_height_half_width_, offsets_);
      if (layer_sector_data_[layer].size() !=
          static_cast<std::size_t>(sector_count_)) {
        throw std::runtime_error("failed to preserve complete layer waypoint data");
      }
      for (auto& sector : layer_sector_data_[layer]) {
        sector.layer_id = static_cast<int>(layer);
        for (auto& candidate : sector.candidates) {
          candidate.layer_id = static_cast<int>(layer);
          candidate.id = "l" + std::to_string(layer) + "_" + candidate.id;
        }
      }
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
    level_ingress_path_pub_ =
        node_.advertise<nav_msgs::Path>(level_ingress_path_topic_, 1, true);
    level_orbit_path_pub_ =
        node_.advertise<nav_msgs::Path>(level_orbit_path_topic_, 1, true);
    altitude_policy_pub_ =
        node_.advertise<std_msgs::String>(altitude_policy_topic_, 1, true);
    geometry_msgs::PointStamped tower_center;
    tower_center.header.stamp = ros::Time::now();
    tower_center.header.frame_id = route_.frame_id;
    tower_center.point.x = route_.center_x;
    tower_center.point.y = route_.center_y;
    tower_center.point.z = 0.0;
    tower_center_pub_.publish(tower_center);
    if (low_altitude_mode_) {
      publishAltitudePolicy("WAITING_FOR_3M_HOVER_AND_FRESH_MAP");
    }
    tracking_client_ = node_.serviceClient<std_srvs::SetBool>(tracking_service_);
    cancel_client_ = node_.serviceClient<std_srvs::Trigger>(cancel_service_);
    resume_client_ = node_.serviceClient<std_srvs::Trigger>(resume_service_);
    return_client_ = node_.serviceClient<std_srvs::Trigger>(return_service_);
    land_client_ = node_.serviceClient<std_srvs::Trigger>(land_service_);
    report_.open(report_file_, std::ios::out | std::ios::trunc);
    if (report_) {
      report_ << "sim_time,state,layer,sector,target_id,target_x,target_y,target_z,"
                 "x,y,z,actual_yaw,horizontal_speed,yaw_mode,"
                 "ref_x,ref_y,ref_z,planner_state,planner_reason,"
                 "failures,recovery_count,"
                 "lap,waypoint,lap_relocations,lap_planning_failures,"
                 "lap_recoveries,normal_return_retries,layer_visit_index,"
                 "cycle,planned_cycles,final_return,altitude_policy,"
                 "horizontal_path_available,vertical_escape_allowed,"
                 "low_ingress_replans,low_orbit_replans\n";
    }
  }

  void publishAltitudePolicy(const std::string& policy) {
    if (!low_altitude_mode_) return;
    altitude_policy_ = policy;
    std_msgs::String message;
    message.data = policy;
    altitude_policy_pub_.publish(message);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (msg->header.frame_id != planning_frame_ || msg->header.stamp.isZero())
      return;
    if (!std::isfinite(msg->pose.pose.position.x) ||
        !std::isfinite(msg->pose.pose.position.y) ||
        !std::isfinite(msg->pose.pose.position.z)) return;
    odom_ = *msg; have_odom_ = true; odom_received_ = ros::Time::now();
    recordLowIngressTrace(msg->pose.pose.position);
  }

  void recordLowIngressTrace(const geometry_msgs::Point& position) {
    if (!low_altitude_mode_ ||
        state_ != MissionState::kEntryGateTransit) {
      return;
    }
    constexpr double kTraceSpacing = 0.75;
    if (!successful_ingress_trace_.empty()) {
      const auto& previous = successful_ingress_trace_.back();
      if (std::hypot(position.x - previous.x,
                     position.y - previous.y) < kTraceSpacing) {
        return;
      }
    }
    CandidatePoint trace;
    trace.id = "EXECUTED_INGRESS_TRACE_" +
               std::to_string(successful_ingress_trace_.size());
    trace.sector_id = -1;
    trace.layer_id = 0;
    trace.x = position.x;
    trace.y = position.y;
    trace.z = position.z;
    trace.require_arrival_yaw = false;
    trace.face_tower = false;
    successful_ingress_trace_.push_back(trace);
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
      successful_ingress_trace_.clear();
      entry_gate_index_ = -1;
      provisional_entry_gate_index_ = -1;
      entry_gate_locked_ = false;
      entry_gate_candidates_.clear();
      failed_entry_gate_positions_.clear();
      unreachable_entry_gate_candidates_.clear();
      entry_gate_relocations_ = 0;
      entry_gate_last_failure_.clear();
      entry_gate_no_safe_candidate_hold_ = false;
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

  bool mappedEndpointClear(const CandidatePoint& target,
                           double clearance) const {
    if (target.z < route_.minimum_height ||
        target.z > route_.maximum_height ||
        !std::isfinite(clearance) || clearance < 0.0) {
      return false;
    }
    for (const auto& occupied : planningMapPoints()) {
      const double dx = target.x - occupied.x;
      const double dy = target.y - occupied.y;
      const double dz = target.z - occupied.z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) <
          clearance) {
        return false;
      }
    }
    return true;
  }

  bool mappedEndpointClear(const CandidatePoint& target) const {
    return mappedEndpointClear(target, filter_config_.minimum_clearance);
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

  CandidatePoint firstInspectionReference(int layer_index) const {
    CandidatePoint first;
    first.id = "FIRST_INSPECTION_REFERENCE";
    first.layer_id = layer_index;
    RouteConfig layer_route = route_;
    layer_route.height = inspection_heights_[layer_index];
    const auto references = buildInspectionSectors(
        layer_route, sector_count_, 1, sector_angle_half_width_deg_,
        sector_radius_half_width_, sector_height_half_width_,
        std::vector<CandidateOffset>{{0.0, 0.0, 0.0}});
    const double entry_angle =
        entryGateSectorCenterAngleRad(entry_sector_user_);
    const auto order =
        directionalSectorOrder(entry_angle, references, route_.direction);
    if (order.empty()) return first;
    first.sector_id = references[order.front()].sector_id;
    const double angle = references[order.front()].nominal_angle_rad;
    first.x = route_.center_x + route_.radius * std::cos(angle);
    first.y = route_.center_y + route_.radius * std::sin(angle);
    first.z = inspection_heights_[layer_index];
    return first;
  }

  bool lockLayerGate(int layer_index, const ros::Time& now) {
    entry_gate_last_failure_.clear();
    if (!mapFresh(now) || layer_index < 0 ||
        layer_index >= static_cast<int>(inspection_heights_.size())) {
      return false;
    }
    if (entry_gate_candidates_.empty() ||
        provisional_entry_gate_.layer_id != layer_index) {
      // Candidate ids are reused at each height. A new layer must not inherit
      // the previous layer's rejected index or relocation budget.
      unreachable_entry_gate_candidates_.clear();
      entry_gate_relocations_ = 0;
    }
    entry_gate_locked_ = false;
    entry_gate_index_ = -1;
    provisional_entry_gate_index_ = -1;
    const double layer_height = inspection_heights_[layer_index];
    EntryGateConfig layer_config = entry_gate_config_;
    layer_config.inspection_height = layer_height;
    const CandidatePoint first_inspection =
        firstInspectionReference(layer_index);
    entry_gate_candidates_.clear();
    entry_gate_candidates_ = buildEntryGateCandidates(
        route_, entry_sector_user_, layer_config.minimum_radius,
        layer_config.maximum_radius, layer_config.preferred_radius,
        layer_config.angular_sample_step_deg,
        layer_config.radial_sample_step, layer_height);
    for (auto& candidate : entry_gate_candidates_) {
      candidate.layer_id = layer_index;
      if (!entryGatePointInSector(
              candidate, route_, entry_sector_user_)) {
        candidate.accepted = false;
        candidate.target_invalid = true;
        candidate.rejection_reason = "ENTRY_GATE_SECTOR_ESCAPE";
        continue;
      }
      geometry_msgs::Point point;
      point.x = candidate.x;
      point.y = candidate.y;
      point.z = candidate.z;
      const double unknown_ratio = candidateUnknownRatio(point);
      evaluateEntryGateCandidate(
          &candidate, route_, pointOf(odom_), home_position_,
          planningMapPoints(), obstacles_, true, layer_config,
          unknown_ratio, filter_config_.unknown_ratio_limit,
          &first_inspection);
      if (unreachable_entry_gate_candidates_.count(candidate.id) > 0U) {
        candidate.accepted = false;
        candidate.planner_unreachable = true;
        candidate.target_invalid = false;
        candidate.rejection_reason = "PLANNER_UNREACHABLE";
      }
      const double radius = std::hypot(
          candidate.x - route_.center_x, candidate.y - route_.center_y);
      const double angle_deg = positiveAngleDegrees(std::atan2(
          candidate.y - route_.center_y,
          candidate.x - route_.center_x));
      const double center_error_deg =
          std::abs(normalizeAngle(
              std::atan2(candidate.y - route_.center_y,
                         candidate.x - route_.center_x) -
              entryGateSectorCenterAngleRad(entry_sector_user_))) *
          180.0 / kPi;
      const double first_waypoint_distance = std::hypot(
          candidate.x - first_inspection.x,
          candidate.y - first_inspection.y);
      ROS_WARN("[STAGE3_TASK] ENTRY_GATE candidate layer=%d id=%s "
               "sector=%d angle=%.1fdeg center_error=%.1fdeg radius=%.2f "
               "first_waypoint_distance=%.2f clearance=%.2f score=%.3f "
               "accepted=%s reason=%s",
               layer_index, candidate.id.c_str(), entry_sector_user_,
               angle_deg, center_error_deg, radius, first_waypoint_distance,
               candidate.clearance, candidate.score,
               candidate.accepted ? "true" : "false",
               candidate.accepted
                   ? (candidate.risk_reason.empty()
                          ? "SAFE"
                          : candidate.risk_reason.c_str())
                   : candidate.rejection_reason.c_str());
      if (!candidate.accepted) {
        ROS_DEBUG("[STAGE3_TASK] layer gate candidate rejected: layer=%d "
                  "user_sector=%d id=%s reason=%s radius=%.3f "
                  "xyz=(%.2f, %.2f, %.2f) clearance=%.3f unknown=%.3f",
                  layer_index, entry_sector_user_, candidate.id.c_str(),
                  candidate.rejection_reason.c_str(),
                  std::hypot(candidate.x - route_.center_x,
                             candidate.y - route_.center_y),
                  candidate.x, candidate.y, candidate.z,
                  candidate.clearance, candidate.unknown_ratio);
      }
    }
    entry_gate_index_ =
        chooseBestEntryGateCandidate(entry_gate_candidates_);
    if (entry_gate_index_ < 0) {
      std::ostringstream reason;
      reason << "ENTRY_GATE_SECTOR_" << entry_sector_user_
             << "_NO_SAFE_CANDIDATE";
      entry_gate_last_failure_ = reason.str();
      ROS_ERROR_THROTTLE(
          1.0,
          "[STAGE3_TASK] %s: direction=%s layer=%d candidates=%zu; "
          "cross-sector fallback is forbidden, entering HOLD",
          entry_gate_last_failure_.c_str(),
          entrySectorDirectionName(entry_sector_user_), layer_index,
          entry_gate_candidates_.size());
      return false;
    }
    provisional_entry_gate_index_ = entry_gate_index_;
    provisional_entry_gate_ = entry_gate_candidates_[entry_gate_index_];
    entry_gate_locked_ = true;
    layer_entry_gates_[layer_index] = provisional_entry_gate_;
    layer_entry_gate_valid_[layer_index] = true;
    const double selected_radius = std::hypot(
        provisional_entry_gate_.x - route_.center_x,
        provisional_entry_gate_.y - route_.center_y);
    const double selected_angle_deg = positiveAngleDegrees(std::atan2(
        provisional_entry_gate_.y - route_.center_y,
        provisional_entry_gate_.x - route_.center_x));
    const double first_waypoint_distance = std::hypot(
        provisional_entry_gate_.x - first_inspection.x,
        provisional_entry_gate_.y - first_inspection.y);
    ROS_WARN("[STAGE3_TASK] ENTRY_GATE selected: layer=%d %s "
             "user_sector=%d internal_index=%d direction=%s "
             "angle=%.3fdeg radius=%.2f preferred_radius=%.2f "
             "xyz=(%.3f, %.3f, %.3f) score=%.3f "
             "first_waypoint_distance=%.3f unknown_ratio=%.3f; "
             "stored independently for this layer",
             layer_index,
             provisional_entry_gate_.id.c_str(),
             entry_sector_user_, entry_sector_index_,
             entrySectorDirectionName(entry_sector_user_),
             selected_angle_deg, selected_radius,
             layer_config.preferred_radius,
             provisional_entry_gate_.x, provisional_entry_gate_.y,
             provisional_entry_gate_.z,
             provisional_entry_gate_.score, first_waypoint_distance,
             provisional_entry_gate_.unknown_ratio);
    return true;
  }

  bool lockFinalEntryGate(const ros::Time& now) {
    return lockLayerGate(0, now);
  }

  bool startEntryGateTransit() {
    if (!have_odom_ || !entry_gate_locked_) return false;
    const geometry_msgs::Point current = pointOf(odom_);
    geometry_msgs::Point gate;
    gate.x = provisional_entry_gate_.x;
    gate.y = provisional_entry_gate_.y;
    gate.z = provisional_entry_gate_.z;
    if (low_altitude_mode_) {
      geometry_msgs::Point level_start = current;
      level_start.z = level_path_config_.altitude;
      geometry_msgs::Point level_gate = gate;
      level_gate.z = level_path_config_.altitude;
      const LevelPathResult level_path = planLevelPath(
          level_start, level_gate, planningMapPoints(), level_path_config_);
      nav_msgs::Path path_message;
      path_message.header.stamp = ros::Time::now();
      path_message.header.frame_id = planning_frame_;
      for (const auto& point : level_path.points) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_message.header;
        pose.pose.position = point;
        pose.pose.orientation.w = 1.0;
        path_message.poses.push_back(pose);
      }
      level_ingress_path_pub_.publish(path_message);

      if (level_path.reachable) {
        low_no_path_confirmations_ = 0;
        last_low_no_path_check_ = ros::Time(0);
        horizontal_path_available_ = true;
        vertical_escape_allowed_ = false;
        entry_gate_transit_goals_.clear();
        for (std::size_t index = 1U; index < level_path.points.size();
             ++index) {
          CandidatePoint target = provisional_entry_gate_;
          target.id = index + 1U == level_path.points.size()
                          ? provisional_entry_gate_.id
                          : "LIVE_LEVEL_PATH_" + std::to_string(index);
          target.x = level_path.points[index].x;
          target.y = level_path.points[index].y;
          target.z = level_path_config_.altitude;
          target.require_arrival_yaw = false;
          target.face_tower = true;
          entry_gate_transit_goals_.push_back(target);
        }
        publishAltitudePolicy("HORIZONTAL_3M_PATH_REQUIRED");
        ROS_WARN("[STAGE3_TASK] LOW_ALTITUDE horizontal path confirmed from "
                 "fresh occupancy: goals=%zu occupied_cells=%zu z=%.2f; "
                 "EGO receives live-map local goals and vertical escape is "
                 "forbidden",
                 entry_gate_transit_goals_.size(),
                 level_path.occupied_cell_count,
                 level_path_config_.altitude);
      } else {
        const bool level_ingress_committed =
            !successful_ingress_goals_.empty();
        const bool relocate_before_vertical_escape =
            level_path.reason == "LEVEL_PATH_GOAL_OCCUPIED" ||
            level_path.reason == "NO_LEVEL_PATH" ||
            level_path.reason == "LEVEL_PATH_START_OCCUPIED";
        if (relocate_before_vertical_escape &&
            entry_gate_locked_ &&
            entry_gate_relocations_ < max_entry_gate_relocations_ &&
            low_no_path_confirmations_ == 0 &&
            !level_ingress_committed) {
          // A live map can invalidate the originally selected gate after the
          // vehicle reveals an occluded obstacle. Keep the configured sector
          // and exact radius, reject only this candidate, and let the normal
          // weighted selector choose another same-sector angle before any
          // vertical escape is considered.
          const std::string rejected_gate_id = provisional_entry_gate_.id;
          if (unreachable_entry_gate_candidates_
                  .insert(rejected_gate_id)
                  .second) {
            failed_entry_gate_positions_.push_back(provisional_entry_gate_);
            ++entry_gate_relocations_;
          }
          entry_gate_locked_ = false;
          ROS_WARN("[STAGE3_TASK] LOW_ALTITUDE live map result %s invalidated "
                   "ENTRY_GATE %s; retrying same-sector same-radius gate "
                   "before any vertical escape (relocation=%d/%d)",
                   level_path.reason.c_str(),
                   rejected_gate_id.c_str(), entry_gate_relocations_,
                   max_entry_gate_relocations_);
          const ros::Time relocation_now = ros::Time::now();
          if (lockLayerGate(0, relocation_now) && startEntryGateTransit()) {
            return true;
          }
          entry_gate_locked_ = false;
        }
        const ros::Time now = ros::Time::now();
        const bool confirmation_due =
            last_low_no_path_check_.isZero() ||
            now - last_low_no_path_check_ >=
                ros::Duration(low_no_path_confirmation_period_);
        // A goal-occupied result normally means that this particular
        // candidate is invalid and is handled by the bounded same-sector
        // relocation above.  Once those relocations are exhausted, the
        // remaining candidate has also failed the conservative 3 m
        // reachability test.  Treat it like a confirmed absence of a
        // horizontal passage so the mission can perform the explicitly
        // permitted temporary 3-D escape instead of returning immediately.
        const bool can_confirm_no_horizontal_path =
            level_path.reason == "NO_LEVEL_PATH" ||
            // The vehicle can be surrounded by a newly revealed inflated
            // obstacle voxel even when the gate itself is clear.  In that
            // case there is no safe 3 m departure channel from the measured
            // pose; after fresh-map confirmation it is the same policy
            // decision as a full level wall.
            level_path.reason == "LEVEL_PATH_START_OCCUPIED" ||
            (level_path.reason == "LEVEL_PATH_GOAL_OCCUPIED" &&
             (entry_gate_relocations_ >= max_entry_gate_relocations_ ||
              low_no_path_confirmations_ > 0 ||
              level_ingress_committed));
        if (!can_confirm_no_horizontal_path) {
          ROS_WARN_THROTTLE(
              1.0,
              "[STAGE3_TASK] LOW_ALTITUDE horizontal reachability check "
              "deferred: %s",
              level_path.reason.c_str());
          return false;
        }
        if (confirmation_due) {
          ++low_no_path_confirmations_;
          last_low_no_path_check_ = now;
        }
        if (low_no_path_confirmations_ <
            low_no_path_confirmation_limit_) {
          publishAltitudePolicy("CONFIRMING_NO_HORIZONTAL_3M_PATH");
          ROS_WARN_THROTTLE(
              1.0,
              "[STAGE3_TASK] LOW_ALTITUDE no horizontal path on fresh map "
              "(confirmation=%d/%d); remaining at 3 m and rechecking",
              low_no_path_confirmations_,
              low_no_path_confirmation_limit_);
          return false;
        }
        horizontal_path_available_ = false;
        vertical_escape_allowed_ = true;
        // If the vehicle has already reached a live 3 m ingress point, do
        // not ask EGO to start a 3-D detour from the newly revealed obstacle
        // voxel. Backtrack to the last confirmed-safe live point, climb
        // there, then approach the fixed 3 m ENTRY_GATE and recover to 3 m.
        // This escape is generated from the current map/progress, not a
        // preset obstacle route.
        entry_gate_transit_goals_.clear();
        if (!successful_ingress_goals_.empty()) {
          CandidatePoint safe_staging = successful_ingress_goals_.back();
          safe_staging.id = "VERTICAL_ESCAPE_SAFE_STAGING";
          safe_staging.z = level_path_config_.altitude;
          safe_staging.require_arrival_yaw = false;
          safe_staging.face_tower = true;
          if (std::hypot(current.x - safe_staging.x,
                         current.y - safe_staging.y) >
              arrival_tolerance_) {
            entry_gate_transit_goals_.push_back(safe_staging);
          }
          CandidatePoint overflight = safe_staging;
          overflight.id = "VERTICAL_ESCAPE_OVERFLIGHT";
          overflight.z = std::min(
              std::max(level_path_config_.altitude + 2.0, 5.0),
              virtual_ceil_height_ - 0.5);
          entry_gate_transit_goals_.push_back(overflight);
        }
        entry_gate_transit_goals_.push_back(provisional_entry_gate_);
        publishAltitudePolicy("VERTICAL_ESCAPE_ALLOWED_AFTER_NO_LEVEL_PATH");
        ROS_WARN("[STAGE3_TASK] LOW_ALTITUDE no 3 m horizontal passage "
                 "confirmed on %d fresh-map checks; EGO may now use a "
                 "temporary 3-D detour, while ENTRY_GATE remains fixed at "
                 "z=%.2f for smooth recovery",
                 low_no_path_confirmations_,
                 provisional_entry_gate_.z);
      }
    } else if (prefer_safe_overflight_ && mappedCorridorSafe(current, gate)) {
      entry_gate_transit_goals_ = buildRollingApproachGoals(
          current, provisional_entry_gate_, entry_gate_segment_length_);
      for (std::size_t index = 0;
           index + 1U < entry_gate_transit_goals_.size(); ++index) {
        entry_gate_transit_goals_[index].id =
            "ENTRY_OVERFLIGHT_" + std::to_string(index + 1U);
      }
      ROS_WARN("[STAGE3_TASK] fresh map confirms a clear high corridor; "
               "ENTRY_GATE transit uses %zu bounded overflight goals at "
               "z=%.2f",
               entry_gate_transit_goals_.size(),
               provisional_entry_gate_.z);
    } else {
      // If the high straight corridor is occupied, keep only the safe gate
      // endpoint and let EGO bend around the local obstacle. This avoids
      // forcing an interpolated waypoint into a crane/tree voxel.
      entry_gate_transit_goals_.assign(1U, provisional_entry_gate_);
      ROS_WARN("[STAGE3_TASK] high straight corridor is not fully clear; "
               "ENTRY_GATE endpoint remains locked and EGO owns the local "
               "detour");
    }
    entry_gate_transit_index_ = 0U;
    ascent_goal_attempts_ = 0;
    have_sent_goal_ = false;
    arrival_since_ = ros::Time(0);
    coverage_wait_started_ = ros::Time(0);
    entry_gate_relocation_pending_ = false;
    entry_gate_no_safe_candidate_hold_ = false;
    low_ingress_replan_pending_ = false;
    transition(MissionState::kEntryGateTransit,
               low_altitude_mode_
                   ? "3 m hover complete; live-map ENTRY_GATE transit prepared"
                   : "safe altitude reached; high-corridor ENTRY_GATE transit "
                     "prepared");
    return true;
  }

  bool prepareLowOrbitTransit(const CandidatePoint& final_target,
                              const ros::Time& now) {
    if (!low_altitude_mode_ || !have_odom_ || !mapFresh(now)) {
      return false;
    }
    geometry_msgs::Point level_start = pointOf(odom_);
    level_start.z = level_path_config_.altitude;
    geometry_msgs::Point level_goal;
    level_goal.x = final_target.x;
    level_goal.y = final_target.y;
    level_goal.z = level_path_config_.altitude;
    const LevelPathResult level_path = planLevelPath(
        level_start, level_goal, planningMapPoints(), level_path_config_);

    nav_msgs::Path path_message;
    path_message.header.stamp = now;
    path_message.header.frame_id = planning_frame_;
    for (const auto& point : level_path.points) {
      geometry_msgs::PoseStamped pose;
      pose.header = path_message.header;
      pose.pose.position = point;
      pose.pose.orientation.w = 1.0;
      path_message.poses.push_back(pose);
    }
    level_orbit_path_pub_.publish(path_message);
    low_orbit_final_target_ = final_target;

    if (!level_path.reachable &&
        level_path.reason == "LEVEL_PATH_GOAL_OCCUPIED") {
      planner_unreachable_candidates_.insert(final_target.id);
      sectors_[current_sector_].locked_index = -1;
      ++candidate_relocations_;
      ++lap_candidate_relocations_;
      low_orbit_transit_goals_.clear();
      low_orbit_transit_active_ = false;
      active_target_ = low_orbit_final_target_;
      transition(MissionState::kRelocating,
                 "live map invalidated low-altitude orbit endpoint");
      ROS_WARN("[STAGE3_TASK] LOW_ALTITUDE orbit endpoint %s is occupied "
               "on the latest inflated map; selecting another candidate "
               "inside the same numbered sector",
               final_target.id.c_str());
      return false;
    }

    if (level_path.reachable) {
      low_orbit_no_path_confirmations_ = 0;
      last_low_orbit_no_path_check_ = ros::Time(0);
      horizontal_path_available_ = true;
      vertical_escape_allowed_ = false;
      low_orbit_transit_goals_.clear();
      for (std::size_t index = 1U; index < level_path.points.size();
           ++index) {
        CandidatePoint target = final_target;
        const bool final_point = index + 1U == level_path.points.size();
        target.id = final_point
                        ? final_target.id
                        : "LIVE_ORBIT_PATH_" + final_target.id + "_" +
                              std::to_string(index);
        target.x = level_path.points[index].x;
        target.y = level_path.points[index].y;
        target.z = level_path_config_.altitude;
        if (!final_point) target.require_arrival_yaw = false;
        target.face_tower = true;
        low_orbit_transit_goals_.push_back(target);
      }
      if (low_orbit_transit_goals_.empty()) {
        low_orbit_transit_goals_.push_back(final_target);
      }
      low_orbit_transit_index_ = 0U;
      low_orbit_transit_active_ = true;
      active_target_ = low_orbit_transit_goals_.front();
      publishAltitudePolicy("HORIZONTAL_3M_ORBIT_PATH_REQUIRED");
      ROS_WARN("[STAGE3_TASK] LOW_ALTITUDE live-map horizontal orbit path "
               "confirmed for %s: goals=%zu occupied_cells=%zu z=%.2f; "
               "EGO receives map-derived local goals and vertical escape "
               "is forbidden",
               low_orbit_final_target_.id.c_str(),
               low_orbit_transit_goals_.size(),
               level_path.occupied_cell_count,
               level_path_config_.altitude);
      return true;
    }

    const bool confirmation_due =
        last_low_orbit_no_path_check_.isZero() ||
        now - last_low_orbit_no_path_check_ >=
            ros::Duration(low_no_path_confirmation_period_);
    if (confirmation_due) {
      ++low_orbit_no_path_confirmations_;
      last_low_orbit_no_path_check_ = now;
    }
    if (low_orbit_no_path_confirmations_ <
        low_no_path_confirmation_limit_) {
      publishAltitudePolicy("CONFIRMING_NO_HORIZONTAL_3M_ORBIT_PATH");
      ROS_WARN_THROTTLE(
          1.0,
          "[STAGE3_TASK] LOW_ALTITUDE orbit target %s has no confirmed "
          "horizontal path yet (%s confirmation=%d/%d); holding at 3 m",
          final_target.id.c_str(), level_path.reason.c_str(),
          low_orbit_no_path_confirmations_,
          low_no_path_confirmation_limit_);
      return false;
    }

    horizontal_path_available_ = false;
    vertical_escape_allowed_ = true;
    low_orbit_transit_goals_.clear();
    const double maximum_escape_height =
        std::min(recovery_height_max_, virtual_ceil_height_ - 0.5);
    const std::vector<StaticObstacle> no_static_obstacles;
    const double actual_safe_height = odom_.pose.pose.position.z;
    const double first_escape_height =
        std::max(level_path_config_.altitude + 1.0, actual_safe_height);
    for (double escape_height = first_escape_height;
         escape_height <= maximum_escape_height + 1.0e-6;
         escape_height += 1.0) {
      LevelPathConfig escape_config = level_path_config_;
      escape_config.altitude = escape_height;
      // At an elevated plane only obstacles intersecting the vehicle's
      // actual vertical envelope should block the 2-D search; projecting the
      // entire 0.2--5.8 m low-altitude band would make safe overflight
      // mathematically impossible.
      escape_config.vertical_half_extent = 0.75;
      geometry_msgs::Point escape_start = pointOf(odom_);
      escape_start.z = escape_height;
      geometry_msgs::Point escape_goal = level_goal;
      escape_goal.z = escape_height;
      const LevelPathResult escape_path = planLevelPath(
          escape_start, escape_goal, planningMapPoints(), escape_config);
      if (!escape_path.reachable) continue;

      // A previous escape may have stopped above 3 m after discovering that
      // the descent column became occupied. Resume the next climb from the
      // actual safe hover altitude; projecting that pose down to the blocked
      // 3 m plane would incorrectly reject every elevated alternative.
      geometry_msgs::Point climb_bottom = pointOf(odom_);
      geometry_msgs::Point climb_top = escape_start;
      geometry_msgs::Point descent_bottom = level_goal;
      geometry_msgs::Point descent_top = escape_goal;
      if (!lineCorridorSafe(
              climb_bottom, climb_top, planningMapPoints(),
              no_static_obstacles, level_path_config_.additional_clearance,
              filter_config_.corridor_sample_step) ||
          !lineCorridorSafe(
              descent_top, descent_bottom, planningMapPoints(),
              no_static_obstacles, level_path_config_.additional_clearance,
              filter_config_.corridor_sample_step)) {
        continue;
      }

      const auto append_escape_goal =
          [&](const std::string& id, double x, double y, double z,
              bool final_point) {
            CandidatePoint target = final_target;
            target.id = final_point ? final_target.id : id;
            target.x = x;
            target.y = y;
            target.z = z;
            if (!final_point) target.require_arrival_yaw = false;
            target.face_tower = true;
            low_orbit_transit_goals_.push_back(target);
          };
      // Never descend back into a 3 m cell that the fresh map has just
      // rejected.  A rebuilt escape starts at the actual safe hover pose and
      // only climbs (if required) before traversing the elevated plane.
      for (double z = actual_safe_height + 1.0;
           z < escape_height - 1.0e-6; z += 1.0) {
        append_escape_goal(
            "VERTICAL_ORBIT_CLIMB_" + std::to_string(
                low_orbit_transit_goals_.size()),
            escape_start.x, escape_start.y, z, false);
      }
      if (escape_height >
          actual_safe_height + low_altitude_tolerance_) {
        append_escape_goal(
            "VERTICAL_ORBIT_CLIMB_" + std::to_string(
                low_orbit_transit_goals_.size()),
            escape_start.x, escape_start.y, escape_height, false);
      }
      for (std::size_t index = 1U; index < escape_path.points.size();
           ++index) {
        append_escape_goal(
            "VERTICAL_ORBIT_LEVEL_" + std::to_string(index),
            escape_path.points[index].x, escape_path.points[index].y,
            escape_height, false);
      }
      for (double z = escape_height - 1.0;
           z > level_path_config_.altitude + 1.0e-6; z -= 1.0) {
        append_escape_goal(
            "VERTICAL_ORBIT_DESCEND_" + std::to_string(
                low_orbit_transit_goals_.size()),
            level_goal.x, level_goal.y, z, false);
      }
      append_escape_goal(final_target.id, level_goal.x, level_goal.y,
                         level_path_config_.altitude, true);

      nav_msgs::Path escape_path_message;
      escape_path_message.header.stamp = now;
      escape_path_message.header.frame_id = planning_frame_;
      for (const auto& point : escape_path.points) {
        geometry_msgs::PoseStamped pose;
        pose.header = escape_path_message.header;
        pose.pose.position = point;
        pose.pose.orientation.w = 1.0;
        escape_path_message.poses.push_back(pose);
      }
      level_orbit_path_pub_.publish(escape_path_message);
      low_orbit_transit_index_ = 0U;
      low_orbit_transit_active_ = true;
      active_target_ = low_orbit_transit_goals_.front();
      publishAltitudePolicy(
          "VERTICAL_ORBIT_ESCAPE_AFTER_NO_LEVEL_PATH");
      ROS_WARN("[STAGE3_TASK] LOW_ALTITUDE no 3 m horizontal orbit path "
               "to %s confirmed on %d fresh maps; selected a live-map "
               "%.1f m horizontal escape with %zu bounded goals, ending "
               "at the unchanged z=%.2f waypoint",
               low_orbit_final_target_.id.c_str(),
               low_orbit_no_path_confirmations_, escape_height,
               low_orbit_transit_goals_.size(), final_target.z);
      return true;
    }

    planner_unreachable_candidates_.insert(final_target.id);
    sectors_[current_sector_].locked_index = -1;
    ++candidate_relocations_;
    ++lap_candidate_relocations_;
    low_orbit_transit_active_ = false;
    low_orbit_transit_goals_.clear();
    active_target_ = final_target;
    publishAltitudePolicy("NO_BOUNDED_VERTICAL_ORBIT_ESCAPE");
    transition(MissionState::kRelocating,
               "no bounded low-altitude route; selecting another "
               "candidate in the same sector");
    ROS_ERROR("[STAGE3_TASK] LOW_ALTITUDE no safe bounded route was found "
              "from the actual z=%.2f hover through z=%.2f for %s; "
              "rejecting this endpoint and selecting another candidate "
              "inside the same numbered sector",
              actual_safe_height, maximum_escape_height,
              final_target.id.c_str());
    return false;
  }

  bool relocateEntryGateWithinConfiguredSector(
      const ros::Time& now, const std::string& cause) {
    if (entry_gate_locked_) {
      const bool newly_rejected =
          unreachable_entry_gate_candidates_
              .insert(provisional_entry_gate_.id)
              .second;
      if (newly_rejected) {
        failed_entry_gate_positions_.push_back(provisional_entry_gate_);
        ++entry_gate_relocations_;
      }
      ROS_ERROR("[STAGE3_TASK] ENTRY_GATE candidate %s rejected as "
                "planner-unreachable after bounded attempts: %s "
                "(sector=%d relocation=%d/%d)",
                provisional_entry_gate_.id.c_str(), cause.c_str(),
                entry_sector_user_, entry_gate_relocations_,
                max_entry_gate_relocations_);
    }
    entry_gate_locked_ = false;
    have_sent_goal_ = false;
    arrival_since_ = ros::Time(0);
    if (entry_gate_relocations_ <= max_entry_gate_relocations_ &&
        lockLayerGate(0, now) && startEntryGateTransit()) {
      ROS_WARN("[STAGE3_TASK] ENTRY_GATE relocated inside configured sector %d; "
               "no neighboring sector was evaluated",
               entry_sector_user_);
      return true;
    }
    std::ostringstream reason;
    reason << "ENTRY_GATE_SECTOR_" << entry_sector_user_
           << "_NO_SAFE_REACHABLE_CANDIDATE";
    entry_gate_last_failure_ = reason.str();
    entry_gate_no_safe_candidate_hold_ = true;
    entry_gate_recheck_time_ = now;
    ROS_ERROR("[STAGE3_TASK] %s after %d rejected candidates; "
              "cross-sector fallback is forbidden and mission remains HOLD",
              entry_gate_last_failure_.c_str(), entry_gate_relocations_);
    return false;
  }

  bool mapFresh(const ros::Time& now) const {
    return have_cloud_ && have_occupancy_ &&
           fresh(now, cloud_received_, map_timeout_) &&
           fresh(now, occupancy_received_, map_timeout_);
  }

  const std::vector<geometry_msgs::Point>& planningMapPoints() const {
    return occupancy_points_.empty() ? cloud_points_ : occupancy_points_;
  }

  const std::vector<StaticObstacle>& hardPlanningObstacles() const {
    static const std::vector<StaticObstacle> no_coarse_obstacles;
    return filter_config_.known_obstacle_is_hard_constraint
               ? obstacles_
               : no_coarse_obstacles;
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
        layer_index >= static_cast<int>(inspection_heights_.size()) ||
        layer_index >= static_cast<int>(layer_sector_data_.size())) {
      return false;
    }
    // The immutable per-layer collections remain available for later
    // 22->26 re-entry. Only the active working copy is reset here.
    sectors_ = layer_sector_data_[layer_index];
    if (sectors_.size() != static_cast<std::size_t>(sector_count_)) {
      return false;
    }
    route_.height = inspection_heights_[layer_index];
    current_layer_ = layer_index;
    lap_visit_sequence_.clear();
    visit_cursor_ = 0U;
    current_sector_ = 0U;
    current_lap_ = 1;
    waypoint_in_lap_ = 1;
    completed_laps_ = 0;
    have_last_inspection_target_ = false;
    have_layer_start_anchor_ = false;
    layer_start_anchor_ = CandidatePoint();
    planner_unreachable_candidates_.clear();
    active_target_ = CandidatePoint();
    active_goal_kind_ = GoalKind::kSector;
    have_sent_goal_ = false;
    arrival_since_ = ros::Time(0);
    current_target_plan_attempt_ = 0;
    sector_retry_pending_ = false;
    recovery_step_ = 0U;
    initial_waypoint_pending_ = true;
    ROS_WARN("[STAGE3_TASK] inspection layer %d prepared: height=%.2f; "
             "active waypoint indices reset; immutable layer data retained; "
             "legacy inspection_start_sector=%d ignored until ENTRY_GATE "
             "directional ordering is evaluated",
             current_layer_, route_.height, inspection_start_sector_);
    return true;
  }

  bool firstWaypointTowerCorridorSafe(
      const geometry_msgs::Point& from,
      const CandidatePoint& candidate) const {
    StaticObstacle tower;
    tower.id = route_.tower_name;
    tower.x = route_.center_x;
    tower.y = route_.center_y;
    tower.radius = route_.tower_collision_radius;
    tower.z_min = route_.minimum_height;
    tower.z_max = virtual_ceil_height_;
    geometry_msgs::Point target;
    target.x = candidate.x;
    target.y = candidate.y;
    target.z = candidate.z;
    return lineCorridorSafe(
        from, target, {}, {tower},
        filter_config_.minimum_clearance + filter_config_.cloud_inflation,
        filter_config_.corridor_sample_step);
  }

  bool lockDirectionalInitialSector(const ros::Time& now,
                                    bool skip_current_sector = false) {
    if (sectors_.empty() || !mapFresh(now)) return false;
    const geometry_msgs::Point current = pointOf(odom_);
    CandidatePoint entry_reference = provisional_entry_gate_;
    const char* entry_reference_name = "ENTRY_GATE";
    if (current_layer_ > 0) {
      // A lower layer starts where the closed upper-layer anchor descended.
      // Recompute from that actual position; reusing the original ENTRY_GATE
      // angle can select a waypoint behind the current orbit direction.
      entry_reference.x = current.x;
      entry_reference.y = current.y;
      entry_reference_name = "LAYER_ENTRY";
    }
    entry_reference.z = inspection_heights_[current_layer_];
    const double entry_angle = normalizeAngle(std::atan2(
        entry_reference.y - route_.center_y,
        entry_reference.x - route_.center_x));
    const char* direction_name =
        route_.direction == OrbitDirection::kCounterClockwise
            ? "counter_clockwise"
            : "clockwise";
    ROS_WARN("[STAGE3_TASK] FIRST_WAYPOINT %s layer=%d "
             "xyz=(%.3f, %.3f, %.3f) polar=%.3fdeg direction=%s",
             entry_reference_name, current_layer_,
             entry_reference.x, entry_reference.y,
             entry_reference.z, positiveAngleDegrees(entry_angle),
             direction_name);

    const int skipped_sector_id =
        skip_current_sector && current_sector_ < sectors_.size()
            ? sectors_[current_sector_].sector_id
            : -1;
    const auto order =
        directionalSectorOrder(entry_angle, sectors_, route_.direction);
    std::vector<Sector> direction_ordered;
    direction_ordered.reserve(sectors_.size());
    for (std::size_t index : order) {
      direction_ordered.push_back(std::move(sectors_[index]));
    }
    sectors_ = std::move(direction_ordered);

    int selected_order_index = -1;
    int selected_candidate_index = -1;
    for (std::size_t order_index = 0; order_index < sectors_.size();
         ++order_index) {
      Sector& sector = sectors_[order_index];
      sector.state = SectorState::kEvaluating;
      sector.locked_index = -1;
      const double nominal_angle = sector.nominal_angle_rad;
      const double direction_delta = directedAngularDifference(
          entry_angle, nominal_angle, route_.direction);
      const double nominal_x = route_.center_x +
                               sector.nominal_radius *
                                   std::cos(nominal_angle);
      const double nominal_y = route_.center_y +
                               sector.nominal_radius *
                                   std::sin(nominal_angle);
      const double nominal_distance = std::hypot(
          nominal_x - entry_reference.x, nominal_y - entry_reference.y);
      bool tower_corridor_blocked = false;
      bool planner_unreachable = false;
      std::string rejection_summary;
      int candidate_index = -1;
      for (std::size_t index = 0; index < sector.candidates.size(); ++index) {
        CandidatePoint& candidate = sector.candidates[index];
        const double candidate_radius = std::hypot(
            candidate.x - route_.center_x,
            candidate.y - route_.center_y);
        const double candidate_angle = normalizeAngle(std::atan2(
            candidate.y - route_.center_y,
            candidate.x - route_.center_x));
        if (std::abs(candidate_radius - sector.nominal_radius) > 1.0e-6 ||
            std::abs(normalizeAngle(candidate_angle - nominal_angle)) >
                1.0e-6 ||
            std::abs(candidate.z - sector.nominal_height) > 1.0e-6) {
          continue;
        }
        geometry_msgs::Point candidate_point;
        candidate_point.x = candidate.x;
        candidate_point.y = candidate.y;
        candidate_point.z = candidate.z;
        evaluateCandidate(&candidate, sector, current, planningMapPoints(),
                          obstacles_, true, filter_config_, nullptr,
                          candidateUnknownRatio(candidate_point));
        if (planner_unreachable_candidates_.count(candidate.id) > 0U) {
          candidate.accepted = false;
          candidate.planner_unreachable = true;
          candidate.target_invalid = false;
          candidate.rejection_reason = "PLANNER_UNREACHABLE";
          planner_unreachable = true;
        }
        if (candidate.accepted &&
            !firstWaypointTowerCorridorSafe(current, candidate)) {
          candidate.accepted = false;
          candidate.target_invalid = true;
          candidate.rejection_reason = "FIRST_LEG_TOWER_KEEP_OUT";
          tower_corridor_blocked = true;
        }
        if (candidate.accepted) {
          candidate_index = static_cast<int>(index);
        } else {
          rejection_summary = candidate.rejection_reason;
        }
        break;
      }
      const bool skipped = sector.sector_id == skipped_sector_id;
      const bool safe = candidate_index >= 0 && !skipped;
      const CandidatePoint* candidate =
          candidate_index >= 0 ? &sector.candidates[candidate_index] : nullptr;
      const double candidate_distance =
          candidate == nullptr
              ? nominal_distance
              : std::hypot(candidate->x - entry_reference.x,
                           candidate->y - entry_reference.y);
      std::string safety = "NO_ACCEPTED_ENDPOINT";
      if (skipped) {
        safety = "PLANNER_UNREACHABLE_FROM_ENTRY";
      } else if (candidate != nullptr) {
        safety = "SAFE_PENDING_EGO_REACHABILITY";
      } else if (!rejection_summary.empty()) {
        safety = "REJECT_" + rejection_summary;
      }
      ROS_WARN("[STAGE3_TASK] FIRST_WAYPOINT candidate waypoint=%d "
               "internal_index=%d angle=%.3fdeg direction_delta=%.3fdeg "
               "distance=%.3fm clearance=%.3fm endpoint=%s "
               "tower_corridor=%s planner=%s target=%s",
               sector.sector_id + 1, sector.sector_id,
               positiveAngleDegrees(nominal_angle),
               direction_delta * 180.0 / kPi, candidate_distance,
               candidate == nullptr ? -1.0 : candidate->clearance,
               safety.c_str(),
               tower_corridor_blocked ? "BLOCKED" : "SAFE",
               planner_unreachable ? "UNREACHABLE" : "PENDING_OR_CLEAR",
               candidate == nullptr ? "none" : candidate->id.c_str());
      if (selected_order_index < 0 && safe) {
        selected_order_index = static_cast<int>(order_index);
        selected_candidate_index = candidate_index;
      }
    }

    if (selected_order_index < 0 || selected_candidate_index < 0) {
      entry_gate_last_failure_ =
          "NO_DIRECTIONAL_SAFE_REACHABLE_FIRST_WAYPOINT";
      ROS_ERROR("[STAGE3_TASK] %s layer=%d; reverse/opposite-side "
                "fallback is forbidden",
                entry_gate_last_failure_.c_str(), current_layer_);
      return false;
    }
    std::rotate(sectors_.begin(),
                sectors_.begin() + selected_order_index,
                sectors_.end());
    Sector& safe_sector = sectors_.front();
    safe_sector.locked_index = selected_candidate_index;
    safe_sector.state = SectorState::kTargetLocked;
    active_target_ = safe_sector.candidates[selected_candidate_index];
    layer_start_anchor_ = active_target_;
    have_layer_start_anchor_ = true;
    layer_start_anchors_[current_layer_] = active_target_;
    layer_start_anchor_valid_[current_layer_] = true;
    transition_sector_ = safe_sector.sector_id;
    lap_visit_sequence_ = buildClosedLapVisitSequence(
        static_cast<std::size_t>(sector_limit_), inspection_laps_);
    visit_cursor_ = 0U;
    current_sector_ = 0U;
    current_lap_ = 1;
    waypoint_in_lap_ = 1;
    completed_laps_ = 0;
    current_target_plan_attempt_ = 0;
    have_last_inspection_target_ = false;
    have_sent_goal_ = false;
    arrival_since_ = ros::Time(0);
    if (skipped_sector_id >= 0) {
      for (const auto& sector : sectors_) {
        if (sector.sector_id != skipped_sector_id) continue;
        for (const auto& candidate : sector.candidates) {
          planner_unreachable_candidates_.erase(candidate.id);
        }
      }
    }
    publishCandidateDebug(now);
    std::ostringstream execution_order;
    for (std::size_t index = 0; index < sectors_.size(); ++index) {
      if (index > 0U) execution_order << "->";
      execution_order << (sectors_[index].sector_id + 1)
                      << '('
                      << positiveAngleDegrees(
                             sectors_[index].nominal_angle_rad)
                      << "deg)";
    }
    ROS_WARN("[STAGE3_TASK] FIRST_WAYPOINT selected waypoint=%d "
             "internal_index=%d angle=%.3fdeg target=%s "
             "xyz=(%.2f, %.2f, %.2f) "
             "distance_from_current=%.3fm",
             safe_sector.sector_id + 1, safe_sector.sector_id,
             positiveAngleDegrees(safe_sector.nominal_angle_rad),
             active_target_.id.c_str(), active_target_.x,
             active_target_.y, active_target_.z,
             std::sqrt((current.x - active_target_.x) *
                           (current.x - active_target_.x) +
                       (current.y - active_target_.y) *
                           (current.y - active_target_.y) +
                       (current.z - active_target_.z) *
                           (current.z - active_target_.z)));
    ROS_WARN("[STAGE3_TASK] FIRST_WAYPOINT reordered execution sequence "
             "layer=%d direction=%s order=%s",
             current_layer_, direction_name, execution_order.str().c_str());
    ROS_WARN("[STAGE3_TASK] first waypoint remains pending actual EGO "
             "reachability; failures advance only in %s direction",
             direction_name);
    transition(MissionState::kTargetLocked,
               "ENTRY_GATE directional first inspection waypoint locked");
    return !lap_visit_sequence_.empty();
  }

  bool evaluateSector(const ros::Time& now) {
    if (current_sector_ >= sectors_.size()) return false;
    auto& sector = sectors_[current_sector_];
    sector.state = SectorState::kEvaluating;
    const CandidatePoint* previous = nullptr;
    if (have_last_inspection_target_) previous = &last_inspection_target_;
    const bool closing_at_layer_anchor =
        have_layer_start_anchor_ && visit_cursor_ > 0U &&
        current_sector_ == 0U;
    if (closing_at_layer_anchor) {
      const auto anchor = std::find_if(
          sector.candidates.begin(), sector.candidates.end(),
          [this](const CandidatePoint& candidate) {
            return candidate.id == layer_start_anchor_.id;
          });
      if (anchor == sector.candidates.end()) {
        sector.failure_reason = "LAYER_START_ANCHOR_MISSING";
        sector.state = SectorState::kRelocating;
        return false;
      }
      geometry_msgs::Point anchor_point;
      anchor_point.x = anchor->x;
      anchor_point.y = anchor->y;
      anchor_point.z = anchor->z;
      evaluateCandidate(&(*anchor), sector, pointOf(odom_),
                        planningMapPoints(), obstacles_, mapFresh(now),
                        filter_config_, previous,
                        candidateUnknownRatio(anchor_point));
      if (planner_unreachable_candidates_.count(anchor->id) > 0U) {
        anchor->accepted = false;
        anchor->planner_unreachable = true;
        anchor->target_invalid = false;
        anchor->rejection_reason = "PLANNER_UNREACHABLE";
      }
      publishCandidateDebug(now);
      if (!anchor->accepted) {
        sector.failure_reason =
            anchor->rejection_reason.empty()
                ? "LAYER_START_ANCHOR_REJECTED"
                : anchor->rejection_reason;
        sector.state = SectorState::kRelocating;
        return false;
      }
      sector.locked_index =
          static_cast<int>(std::distance(sector.candidates.begin(), anchor));
      sector.state = SectorState::kTargetLocked;
      active_target_ = *anchor;
      current_target_plan_attempt_ = 0;
      ROS_WARN("[STAGE3_TASK] closing layer=%d lap=%d at start anchor "
               "waypoint=%d angle=%.3fdeg target=%s",
               current_layer_, current_lap_, sector.sector_id + 1,
               positiveAngleDegrees(sector.nominal_angle_rad),
               active_target_.id.c_str());
      transition(MissionState::kTargetLocked,
                 "returning to recorded layer start anchor");
      return true;
    }
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
    low_orbit_transit_active_ = false;
    low_orbit_transit_goals_.clear();
    low_orbit_no_path_confirmations_ = 0;
    last_low_orbit_no_path_check_ = ros::Time(0);
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

  void requestLowIngressReplan(const std::string& reason) {
    if (!low_altitude_mode_ || low_ingress_replan_pending_) return;
    const bool initial_gate_transit =
        state_ == MissionState::kEntryGateTransit &&
        entry_gate_locked_;
    const bool gate_candidate_may_be_invalid =
        reason.find("ENTRY_GATE_TARGET_OCCUPIED") != std::string::npos ||
        reason.find("ENTRY_GATE transit planner failure") !=
            std::string::npos ||
        reason.find("ENTRY_GATE transit no progress") !=
            std::string::npos ||
        reason.find("ENTRY_GATE transit target timeout") !=
            std::string::npos;
    if (initial_gate_transit && gate_candidate_may_be_invalid &&
        entry_gate_relocations_ < max_entry_gate_relocations_) {
      const std::string rejected_gate_id = provisional_entry_gate_.id;
      if (unreachable_entry_gate_candidates_.insert(rejected_gate_id).second) {
        failed_entry_gate_positions_.push_back(provisional_entry_gate_);
        ++entry_gate_relocations_;
        entry_gate_locked_ = false;
        // These are intermediate goals belonging to the rejected gate. They
        // must not prevent the next same-sector candidate from being treated
        // as a fresh ingress attempt.
        successful_ingress_goals_.clear();
        ROS_WARN(
            "[STAGE3_TASK] LOW_ALTITUDE ingress failure invalidated "
            "ENTRY_GATE %s; next HOLD retry will select another "
            "same-sector same-radius candidate (relocation=%d/%d): %s",
            rejected_gate_id.c_str(), entry_gate_relocations_,
            max_entry_gate_relocations_, reason.c_str());
      }
    }
    ++low_ingress_replans_;
    low_ingress_replan_pending_ = true;
    publishAltitudePolicy("HOLD_FOR_FRESH_MAP_INGRESS_REPLAN");
    requestHold("LOW_ALTITUDE ingress replan: " + reason);
  }

  void requestLowOrbitReplan(const std::string& reason) {
    if (!low_altitude_mode_ || !low_orbit_transit_active_ ||
        low_orbit_replan_pending_) {
      return;
    }
    ++low_orbit_replans_;
    active_target_ = low_orbit_final_target_;
    low_orbit_transit_active_ = false;
    low_orbit_transit_goals_.clear();
    low_orbit_transit_index_ = 0U;
    low_orbit_no_path_confirmations_ = 0;
    last_low_orbit_no_path_check_ = ros::Time(0);
    horizontal_path_available_ = true;
    vertical_escape_allowed_ = false;
    low_orbit_replan_pending_ = true;
    current_target_plan_attempt_ = 0;
    publishAltitudePolicy("HOLD_FOR_FRESH_MAP_ORBIT_REPLAN");
    requestHold("LOW_ALTITUDE orbit replan: " + reason);
  }

  bool lowIngressCommandViolatesAltitude(const ros::Time& now) const {
    const bool horizontal_low_altitude_phase =
        state_ == MissionState::kEntryGateTransit ||
        (state_ == MissionState::kNavigate &&
         low_orbit_transit_active_);
    if (!low_altitude_mode_ || vertical_escape_allowed_ ||
        !horizontal_low_altitude_phase || !have_command_ ||
        !fresh(now, command_received_, input_timeout_) ||
        command_.trajectory_id == trajectory_baseline_) {
      return false;
    }
    const double command_error =
        std::abs(command_.position.z - level_path_config_.altitude);
    if (command_error <= low_altitude_tolerance_) return false;

    // A trajectory that starts at a small takeoff overshoot and converges
    // toward 3 m is recovery to the preferred plane, not an obstacle-driven
    // vertical escape. Reject only a command that moves farther away from
    // 3 m than the vehicle already is.
    if (have_odom_ && fresh(now, odom_received_, input_timeout_)) {
      const double current_error =
          std::abs(odom_.pose.pose.position.z - level_path_config_.altitude);
      if (command_error <= current_error + 0.08) return false;
    }
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
    const bool current_goal_trajectory =
        have_command_ && command_.trajectory_id != trajectory_baseline_;
    const bool current_goal_command_fresh =
        current_goal_trajectory &&
        fresh(now, command_received_, input_timeout_);
    if (planner_status_.goal_in_collision) {
      // EGO reports occupancy of its moving local-horizon endpoint here, not
      // necessarily occupancy of the mission goal. During live-map low
      // ingress, a successful active trajectory must be allowed to use EGO's
      // normal local replanning. The bounded mission-level recovery below
      // still takes over after an actual failed plan is recorded.
      if (low_altitude_mode_ &&
          state_ == MissionState::kEntryGateTransit &&
          ((planner_status_.last_plan_success &&
            planner_status_.consecutive_plan_failures == 0U) ||
           (vertical_escape_allowed_ && current_goal_command_fresh &&
            !planner_status_.emergency_stop_active))) {
        return false;
      }
      *reason = astra_custom_msgs::PlannerStatus::GOAL_IN_OCCUPANCY;
      return true;
    }
    const bool sustained_emergency_stop =
        planner_status_.emergency_stop_active &&
        planner_status_.emergency_stop_duration >= emergency_stop_timeout_;
    const bool plan_failure_limit_reached =
        planner_status_.consecutive_plan_failures >=
        static_cast<uint32_t>(consecutive_plan_failure_limit_);
    if (plan_failure_limit_reached || sustained_emergency_stop) {
      // During an explicitly authorized 3-D escape, EGO frequently records a
      // failed rebound iteration before succeeding on the next local
      // replan. As long as traj_server is still providing a fresh trajectory
      // for this goal and no sustained emergency stop is active, let EGO
      // continue internally. The existing command-expiry, no-progress and
      // goal-timeout gates remain the bounded mission-level fallback.
      if (low_altitude_mode_ && vertical_escape_allowed_ &&
          state_ == MissionState::kEntryGateTransit &&
          current_goal_command_fresh && !sustained_emergency_stop) {
        return false;
      }
      *reason = planner_status_.failure_reason.empty()
                    ? astra_custom_msgs::PlannerStatus::REPLAN_FAILED
                    : planner_status_.failure_reason;
      return true;
    }
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
    clockwise.recovery_height = std::max(
        route_.minimum_height,
        sector.nominal_height - maximum_temporary_descent_);
    RecoveryConfig counter_clockwise = recovery_config_;
    counter_clockwise.direction = OrbitDirection::kCounterClockwise;
    counter_clockwise.recovery_height = clockwise.recovery_height;
    const RecoveryTargets clockwise_targets = makeRecoveryTargets(
        route_, pointOf(odom_), sector, clockwise, locked);
    const RecoveryTargets counter_clockwise_targets = makeRecoveryTargets(
        route_, pointOf(odom_), sector, counter_clockwise, locked);
    static const std::vector<StaticObstacle> no_coarse_obstacles;
    const auto& recovery_obstacles =
        filter_config_.known_obstacle_is_hard_constraint
            ? obstacles_
            : no_coarse_obstacles;
    const RecoveryAssessment clockwise_assessment = assessRecoveryTargets(
        pointOf(odom_), clockwise_targets, planningMapPoints(),
        recovery_obstacles,
        filter_config_.minimum_clearance,
        filter_config_.corridor_sample_step);
    const RecoveryAssessment counter_clockwise_assessment = assessRecoveryTargets(
        pointOf(odom_), counter_clockwise_targets, planningMapPoints(),
        recovery_obstacles,
        filter_config_.minimum_clearance,
        filter_config_.corridor_sample_step);
    const bool clockwise_local =
        clockwise_assessment.endpoints_safe &&
        recoveryTargetsStayInSector(
            clockwise_targets, sector, maximum_temporary_descent_);
    const bool counter_clockwise_local =
        counter_clockwise_assessment.endpoints_safe &&
        recoveryTargetsStayInSector(
            counter_clockwise_targets, sector,
            maximum_temporary_descent_);
    if (!clockwise_local && !counter_clockwise_local) return false;
    const bool choose_clockwise = clockwise_local &&
        (!counter_clockwise_local ||
         clockwise_assessment.score > counter_clockwise_assessment.score);
    recovery_targets_ = choose_clockwise ? clockwise_targets
                                         : counter_clockwise_targets;
    recovery_config_.direction = choose_clockwise
                                     ? OrbitDirection::kClockwise
                                     : OrbitDirection::kCounterClockwise;
    const RecoveryAssessment& selected_assessment = choose_clockwise
                                                        ? clockwise_assessment
                                                        : counter_clockwise_assessment;
    ROS_WARN("[STAGE3_TASK] sector-local recovery selected %s at z=%.2f; "
             "blocked straight corridors=%d (soft risk only), re-entry=%s",
             choose_clockwise ? "clockwise" : "counter_clockwise",
             recovery_targets_.r1.z,
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

  bool prepareNextLayerAnchor(int next_layer, const ros::Time& now,
                              std::string* reason) {
    if (next_layer < 0 ||
        next_layer >= static_cast<int>(layer_sector_data_.size()) ||
        !mapFresh(now)) {
      *reason = "NEXT_LAYER_DATA_OR_MAP_UNAVAILABLE";
      return false;
    }
    pending_layer_sectors_ = layer_sector_data_[next_layer];
    const double source_angle = std::atan2(
        layer_transition_anchor_.y - route_.center_y,
        layer_transition_anchor_.x - route_.center_x);
    const auto order = directionalSectorOrder(
        source_angle, pending_layer_sectors_, route_.direction);
    std::vector<Sector> ordered;
    ordered.reserve(pending_layer_sectors_.size());
    for (std::size_t index : order) {
      ordered.push_back(std::move(pending_layer_sectors_[index]));
    }
    pending_layer_sectors_ = std::move(ordered);
    const auto source_sector = std::find_if(
        pending_layer_sectors_.begin(), pending_layer_sectors_.end(),
        [this](const Sector& sector) {
          return sector.sector_id == layer_transition_anchor_.sector_id;
        });
    if (source_sector == pending_layer_sectors_.end()) {
      *reason = "NEXT_LAYER_START_SECTOR_MISSING";
      return false;
    }
    std::rotate(pending_layer_sectors_.begin(), source_sector,
                pending_layer_sectors_.end());
    Sector& start_sector = pending_layer_sectors_.front();
    int selected = -1;
    double selected_xy_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index < start_sector.candidates.size();
         ++index) {
      CandidatePoint& candidate = start_sector.candidates[index];
      geometry_msgs::Point point;
      point.x = candidate.x;
      point.y = candidate.y;
      point.z = candidate.z;
      evaluateCandidate(
          &candidate, start_sector, pointOf(odom_), planningMapPoints(),
          obstacles_, true, filter_config_, &layer_transition_anchor_,
          candidateUnknownRatio(point));
      if (!candidate.accepted) continue;
      const double xy_distance = std::hypot(
          candidate.x - layer_transition_anchor_.x,
          candidate.y - layer_transition_anchor_.y);
      if (selected < 0 || xy_distance < selected_xy_distance) {
        selected = static_cast<int>(index);
        selected_xy_distance = xy_distance;
      }
    }
    if (selected < 0) {
      *reason = "NEXT_LAYER_FIRST_WAYPOINT_UNSAFE";
      return false;
    }
    start_sector.locked_index = selected;
    start_sector.state = SectorState::kTargetLocked;
    pending_layer_start_anchor_ = start_sector.candidates[selected];
    const bool same_xy =
        selected_xy_distance <= layer_transition_same_xy_tolerance_;
    ROS_WARN("[STAGE3_TASK] next layer start selected: layer=%d "
             "waypoint=%d target=%s xyz=(%.3f, %.3f, %.3f) "
             "source_xy=(%.3f, %.3f) xy_distance=%.3f tolerance=%.3f "
             "mode=%s",
             next_layer, start_sector.sector_id + 1,
             pending_layer_start_anchor_.id.c_str(),
             pending_layer_start_anchor_.x, pending_layer_start_anchor_.y,
             pending_layer_start_anchor_.z, layer_transition_anchor_.x,
             layer_transition_anchor_.y, selected_xy_distance,
             layer_transition_same_xy_tolerance_,
             same_xy ? "VERTICAL" : "DIAGONAL");
    reason->clear();
    return true;
  }

  bool startLayerTransition(const ros::Time& now, std::string* reason) {
    if (current_layer_visit_index_ + 1U >= layer_visit_sequence_.size()) {
      *reason = "LAYER_TRANSITION_NO_NEXT_LAYER";
      return false;
    }
    const int next_layer =
        layer_visit_sequence_[current_layer_visit_index_ + 1U];
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
    if (!prepareNextLayerAnchor(next_layer, now, reason)) return false;
    layer_transition_goals_ = buildLayerTransitionGoals(
        layer_transition_anchor_, pending_layer_start_anchor_,
        layer_transition_maximum_vertical_step_,
        layer_transition_same_xy_tolerance_, "LAYER_TRANSITION");
    if (layer_transition_goals_.empty()) {
      *reason = "LAYER_TRANSITION_GOAL_GENERATION_FAILED";
      return false;
    }
    for (auto& goal : layer_transition_goals_) {
      goal.sector_id = pending_layer_start_anchor_.sector_id;
      goal.layer_id = next_layer;
      goal.yaw = pending_layer_start_anchor_.yaw;
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
    geometry_msgs::Point bottom;
    bottom.x = layer_transition_goals_.back().x;
    bottom.y = layer_transition_goals_.back().y;
    bottom.z = layer_transition_goals_.back().z;
    if (!lineCorridorSafe(
            top, bottom, planningMapPoints(), hardPlanningObstacles(),
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
    const bool same_xy =
        std::hypot(bottom.x - top.x, bottom.y - top.y) <=
        layer_transition_same_xy_tolerance_;
    ROS_WARN("[STAGE3_TASK] layer transition accepted: layer=%d->%d "
             "waypoint=%d start=(%.2f, %.2f, %.2f) "
             "end=(%.2f, %.2f, %.2f) mode=%s cycle=%d/%d",
             current_layer_, next_layer,
             pending_layer_start_anchor_.sector_id + 1,
             top.x, top.y, top.z, bottom.x, bottom.y, bottom.z,
             same_xy ? "VERTICAL" : "DIAGONAL",
             static_cast<int>(current_layer_visit_index_ /
                              inspection_heights_.size()) +
                 1,
             planned_cycles_);
    transition(MissionState::kLayerTransition,
               "current layer closed; next-layer first waypoint activated");
    publishGoal(layer_transition_goals_.front(),
                GoalKind::kLayerTransition);
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
    geometry_msgs::Point from = pointOf(odom_);
    geometry_msgs::Point to;
    to.x = target.x;
    to.y = target.y;
    to.z = target.z;
    if (!lineCorridorSafe(
            from, to, planningMapPoints(), hardPlanningObstacles(),
            filter_config_.minimum_clearance +
                filter_config_.cloud_inflation,
            filter_config_.corridor_sample_step)) {
      *reason = "LAYER_TRANSITION_SEGMENT_OCCUPIED";
      return false;
    }
    reason->clear();
    return true;
  }

  bool activatePendingLayerAfterTransition(const ros::Time& now) {
    if (pending_layer_sectors_.empty() ||
        current_layer_visit_index_ + 1U >= layer_visit_sequence_.size()) {
      return false;
    }
    const int next_layer =
        layer_visit_sequence_[current_layer_visit_index_ + 1U];
    // Evaluate and store the next layer's ENTRY/EXIT_GATE only after the
    // vehicle and sensor have actually reached that height. Evaluating a
    // lower gate from the upper layer incorrectly classifies it as unknown.
    if (!lockLayerGate(next_layer, now)) return false;
    layer_sector_runtime_[current_layer_] = sectors_;
    ++current_layer_visit_index_;
    current_layer_ = layer_visit_sequence_[current_layer_visit_index_];
    route_.height = inspection_heights_[current_layer_];
    sectors_ = pending_layer_sectors_;
    pending_layer_sectors_.clear();
    lap_visit_sequence_ = buildClosedLapVisitSequence(
        static_cast<std::size_t>(sector_limit_), inspection_laps_);
    if (lap_visit_sequence_.size() < 2U || sectors_.empty()) return false;
    current_sector_ = 0U;
    sectors_.front().state = SectorState::kCovered;
    layer_start_anchor_ = pending_layer_start_anchor_;
    have_layer_start_anchor_ = true;
    layer_start_anchors_[current_layer_] = pending_layer_start_anchor_;
    layer_start_anchor_valid_[current_layer_] = true;
    transition_sector_ = pending_layer_start_anchor_.sector_id;
    active_target_ = pending_layer_start_anchor_;
    last_inspection_target_ = pending_layer_start_anchor_;
    have_last_inspection_target_ = true;
    initial_waypoint_pending_ = false;
    current_lap_ = 1;
    completed_laps_ = 0;
    waypoint_in_lap_ = 1;
    visit_cursor_ = 1U;
    ++visited_sector_count_;
    current_sector_ = lap_visit_sequence_[visit_cursor_];
    have_sent_goal_ = false;
    arrival_since_ = ros::Time(0);
    current_target_plan_attempt_ = 0;
    planner_unreachable_candidates_.clear();
    ROS_WARN("[STAGE3_TASK] next-layer first waypoint confirmed by actual "
             "arrival: layer=%d height=%.2f waypoint=%d target=%s; "
             "continuing the closed lap without republishing that same goal",
             current_layer_, route_.height,
             pending_layer_start_anchor_.sector_id + 1,
             pending_layer_start_anchor_.id.c_str());
    transition(MissionState::kEvaluate,
               "next-layer first waypoint reached and counted");
    return true;
  }

  bool startExitGate(const ros::Time& now, const std::string& reason) {
    if (current_layer_ < 0 ||
        current_layer_ >= static_cast<int>(inspection_heights_.size())) {
      return false;
    }
    if (!layer_entry_gate_valid_[current_layer_] &&
        !lockLayerGate(current_layer_, now)) {
      return false;
    }
    CandidatePoint exit_gate = layer_entry_gates_[current_layer_];
    std::ostringstream id;
    id << "EXIT_GATE_L" << current_layer_ << "_Z"
       << inspection_heights_[current_layer_];
    exit_gate.id = id.str();
    exit_gate.z = inspection_heights_[current_layer_];
    exit_gate.layer_id = current_layer_;
    exit_gate.require_arrival_yaw = false;
    exit_gate.face_tower = true;
    final_return_ = true;
    have_sent_goal_ = false;
    arrival_since_ = ros::Time(0);
    ROS_WARN("[STAGE3_TASK] final return selected: layer=%d height=%.2f "
             "EXIT_GATE=(%.3f, %.3f, %.3f) cycle=%d/%d; "
             "reverse 26m ingress replay is disabled",
             current_layer_, route_.height, exit_gate.x, exit_gate.y,
             exit_gate.z, planned_cycles_, planned_cycles_);
    transition(MissionState::kGoToExitGate, reason);
    publishGoal(exit_gate, GoalKind::kExitGate);
    return true;
  }

  bool startNormalReturn(const std::string& reason,
                         bool include_exit_gate = true) {
    const bool have_low_trace =
        low_altitude_mode_ && !successful_ingress_trace_.empty();
    if ((!have_low_trace && successful_ingress_goals_.empty()) ||
        (include_exit_gate &&
         (!entry_gate_locked_ || provisional_entry_gate_.layer_id !=
                                      current_layer_))) {
      return false;
    }
    normal_return_goals_.clear();
    if (include_exit_gate) {
      CandidatePoint exit_gate = layer_entry_gates_[current_layer_];
      std::ostringstream id;
      id << "EXIT_GATE_L" << current_layer_ << "_Z"
         << inspection_heights_[current_layer_];
      exit_gate.id = id.str();
      exit_gate.require_arrival_yaw = false;
      exit_gate.face_tower = true;
      normal_return_goals_.push_back(exit_gate);
      ROS_WARN("[STAGE3_TASK] final-layer EXIT_GATE selected: layer=%d "
               "xyz=(%.2f, %.2f, %.2f); no previous-layer gate retained",
               current_layer_, exit_gate.x, exit_gate.y, exit_gate.z);
    }
    const std::vector<CandidatePoint>& ingress_return_source =
        have_low_trace ? successful_ingress_trace_
                       : successful_ingress_goals_;
    for (auto iterator = ingress_return_source.rbegin();
         iterator != ingress_return_source.rend(); ++iterator) {
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
      goal.id = have_low_trace
                    ? "EXECUTED_INGRESS_REVERSE_" + iterator->id
                    : "INGRESS_REVERSE_" + iterator->id;
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
               reason +
                   (have_low_trace
                        ? "; reverse executed ingress trace selected"
                        : "; reverse ingress goals selected"));
    return true;
  }

  bool normalReturnGoalSafe(const CandidatePoint& target,
                            const ros::Time& now) const {
    // The stored low-altitude trace was physically executed with EGO's
    // already-inflated occupancy map. Recheck it with the same additional
    // trajectory clearance used by the live horizontal planner; applying
    // candidate clearance here would add the vehicle envelope twice and
    // reject a path that was just flown safely.
    const double endpoint_clearance =
        low_altitude_mode_ ? level_path_config_.additional_clearance
                           : filter_config_.minimum_clearance;
    if (!mapFresh(now) ||
        !mappedEndpointClear(target, endpoint_clearance)) {
      return false;
    }
    geometry_msgs::Point from = pointOf(odom_);
    geometry_msgs::Point to;
    to.x = target.x;
    to.y = target.y;
    to.z = target.z;
    const bool straight_clear = lineCorridorSafe(
        from, to, planningMapPoints(), hardPlanningObstacles(),
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
    const CandidatePoint previous_target = last_published_target_;
    active_target_ = target;
    active_goal_kind_ = kind;
    // Low-altitude staging and ENTRY_GATE ingress must follow the actual EGO
    // trajectory tangent. Tower-facing yaw starts only when the gate has been
    // reached and the first inspection target is published. High-altitude
    // behavior remains unchanged.
    const bool low_velocity_facing_ingress =
        low_altitude_mode_ &&
        (kind == GoalKind::kStaging || kind == GoalKind::kEntryGate);
    active_target_.face_tower =
        !low_velocity_facing_ingress &&
        (kind == GoalKind::kStaging || kind == GoalKind::kEntryGate ||
         kind == GoalKind::kSector || kind == GoalKind::kRecovery ||
         kind == GoalKind::kLayerTransition || kind == GoalKind::kExitGate);
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
    const double goal_xy_change =
        std::hypot(active_target_.x - previous_target.x,
                   active_target_.y - previous_target.y);
    const bool same_xy =
        have_last_published_target_ &&
        goal_xy_change <= layer_transition_same_xy_tolerance_;
    const char* switch_mode =
        kind == GoalKind::kLayerTransition
            ? (same_xy ? "VERTICAL" : "DIAGONAL")
            : "NORMAL";
    const CandidatePoint* entry_exit_gate =
        current_layer_ >= 0 &&
                current_layer_ <
                    static_cast<int>(layer_entry_gate_valid_.size()) &&
                layer_entry_gate_valid_[current_layer_]
            ? &layer_entry_gates_[current_layer_]
            : nullptr;
    ROS_WARN("[STAGE3_TASK] goal phase=%s layer=%d sector=%d id=%s "
             "xyz=(%.2f, %.2f, %.2f) face_tower=%s; "
             "previous=%s(%.2f, %.2f, %.2f) new=%s(%.2f, %.2f, %.2f) "
             "same_xy=%s mode=%s waypoint_index=%zu cycle=%d/%d "
             "ENTRY_EXIT_GATE=(%.2f, %.2f, %.2f) final_return=%s",
             missionStateName(state_), current_layer_,
             active_target_.sector_id, active_target_.id.c_str(),
             active_target_.x, active_target_.y, active_target_.z,
             active_target_.face_tower ? "true" : "false",
             have_last_published_target_ ? previous_target.id.c_str() : "none",
             previous_target.x, previous_target.y, previous_target.z,
             active_target_.id.c_str(), active_target_.x, active_target_.y,
             active_target_.z, same_xy ? "true" : "false", switch_mode,
             visit_cursor_,
             static_cast<int>(current_layer_visit_index_ /
                              inspection_heights_.size()) +
                 1,
             planned_cycles_,
             entry_exit_gate == nullptr ? 0.0 : entry_exit_gate->x,
             entry_exit_gate == nullptr ? 0.0 : entry_exit_gate->y,
             entry_exit_gate == nullptr ? 0.0 : entry_exit_gate->z,
             final_return_ ? "true" : "false");
    last_published_target_ = active_target_;
    have_last_published_target_ = true;
    requestResume();
    goal_sent_ = ros::Time::now(); arrival_since_ = ros::Time(0);
    best_goal_distance_ = std::numeric_limits<double>::infinity();
    last_progress_time_ = goal_sent_;
    have_sent_goal_ = true;
    const bool live_orbit_subgoal =
        low_orbit_transit_active_ &&
        active_target_.id.rfind("LIVE_ORBIT_PATH_", 0U) == 0U;
    if (kind == GoalKind::kSector && !live_orbit_subgoal) {
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
    ROS_WARN("[STAGE3_TASK] %s -> %s: %s; layer=%d height=%.2f "
             "waypoint_index=%zu active=%s(%.2f, %.2f, %.2f) "
             "cycle=%d/%d final_return=%s",
             missionStateName(state_), missionStateName(next), reason.c_str(),
             current_layer_, route_.height, visit_cursor_,
             active_target_.id.empty() ? "none" : active_target_.id.c_str(),
             active_target_.x, active_target_.y, active_target_.z,
             static_cast<int>(current_layer_visit_index_ /
                              inspection_heights_.size()) +
                 1,
             planned_cycles_, final_return_ ? "true" : "false");
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
        state_ == MissionState::kGoToExitGate ||
        state_ == MissionState::kNormalReturn ||
        state_ == MissionState::kReturnEgress;
    if (enable_control_ && active_mission_state &&
        have_bridge_state_ && fresh(now, bridge_state_received_, 2.0) &&
        bridge_state_ == "HOLD") {
      // A bridge-local safety HOLD must be acknowledged by the mission before
      // the bridge's finite loss timeout can request AUTO.LAND. Cancelling
      // here turns it into a mission-supervised HOLD and preserves the normal
      // ENTRY_GATE relocation / R1-R2 recovery chain.
      if (low_altitude_mode_ &&
          state_ == MissionState::kEntryGateTransit) {
        ++low_ingress_replans_;
        low_ingress_replan_pending_ = true;
        publishAltitudePolicy("HOLD_FOR_FRESH_MAP_INGRESS_REPLAN");
      } else if (low_altitude_mode_ &&
                 state_ == MissionState::kNavigate &&
                 low_orbit_transit_active_) {
        ++low_orbit_replans_;
        active_target_ = low_orbit_final_target_;
        low_orbit_transit_active_ = false;
        low_orbit_transit_goals_.clear();
        low_orbit_transit_index_ = 0U;
        low_orbit_no_path_confirmations_ = 0;
        last_low_orbit_no_path_check_ = ros::Time(0);
        current_target_plan_attempt_ = 0;
        low_orbit_replan_pending_ = true;
        publishAltitudePolicy("HOLD_FOR_FRESH_MAP_ORBIT_REPLAN");
      } else if (state_ == MissionState::kStaging ||
          state_ == MissionState::kSegmentedClimb ||
          state_ == MissionState::kEntryGateTransit) {
        entry_gate_failure_pending_ = true;
        ascent_retry_pending_ = true;
        entry_gate_relocation_pending_ =
            state_ == MissionState::kEntryGateTransit;
        ++ascent_goal_attempts_;
      } else if (state_ == MissionState::kNormalReturn) {
        normal_return_failure_pending_ = true;
      } else if (state_ == MissionState::kGoToExitGate) {
        exit_gate_failure_pending_ = true;
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
    if (lowIngressCommandViolatesAltitude(now)) {
      std::ostringstream reason;
      reason << "planned z=" << command_.position.z
             << " left the required 3 m band while a horizontal path exists";
      if (state_ == MissionState::kNavigate &&
          low_orbit_transit_active_) {
        requestLowOrbitReplan(reason.str());
      } else {
        requestLowIngressReplan(reason.str());
      }
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
        if (low_altitude_mode_) {
          // The bridge has already completed and stabilized the vertical
          // takeoff at 3 m before exposing HOVER_READY. Do not feed that same
          // point through the generic high-altitude staged-climb corridor
          // checks: a zero-length goal can overlap the vehicle's freshly
          // observed local occupancy and be misclassified as a blocked
          // ascent. Keep the coverage dwell, then build the live-map level
          // ingress directly from the measured hover pose.
          approach_index_ = approach_goals_.size();
          coverage_wait_started_ = now;
          transition(MissionState::kSegmentedClimb,
                     "stable 3 m hover confirmed by bridge; generic high-"
                     "altitude climb bypassed");
        } else {
          transition(MissionState::kStaging,
                     "odom/map ready; home-local ascent channel locked");
        }
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
          } else if (!entry_gate_last_failure_.empty()) {
            entry_gate_failure_pending_ = true;
            ascent_retry_pending_ = false;
            entry_gate_no_safe_candidate_hold_ = true;
            requestHold(entry_gate_last_failure_);
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
        if (low_altitude_mode_) {
          publishAltitudePolicy("NOMINAL_3M_ENTRY_AND_ORBIT");
        }
        std::string entry_reason;
        if (!entrySystemsHealthy(now, &entry_reason)) {
          entry_gate_failure_pending_ = true;
          ascent_retry_pending_ = false;
          requestHold(entry_reason);
        } else if (!lockDirectionalInitialSector(now)) {
          entry_gate_failure_pending_ = true;
          ascent_retry_pending_ = false;
          entry_gate_relocation_pending_ = false;
          requestHold("CONFIGURED_FIRST_WAYPOINT_UNSAFE_OR_UNREACHABLE");
        }
      } else if (!have_sent_goal_) {
        const CandidatePoint& target =
            entry_gate_transit_goals_[entry_gate_transit_index_];
        if (!mapFresh(now)) {
          if (coverage_wait_started_.isZero()) coverage_wait_started_ = now;
          if (now - coverage_wait_started_ >=
              ros::Duration(coverage_wait_timeout_)) {
            if (low_altitude_mode_) {
              requestLowIngressReplan(
                  astra_custom_msgs::PlannerStatus::MAP_STALE +
                  std::string(" during ENTRY_GATE transit"));
            } else {
              entry_gate_failure_pending_ = true;
              ascent_retry_pending_ = true;
              entry_gate_relocation_pending_ = true;
              ++ascent_goal_attempts_;
              requestHold(astra_custom_msgs::PlannerStatus::MAP_STALE +
                          std::string(" during ENTRY_GATE transit"));
            }
          }
        } else {
          const bool live_level_path_target =
              low_altitude_mode_ &&
              target.id.rfind("LIVE_LEVEL_PATH_", 0U) == 0U;
          const double endpoint_clearance =
              live_level_path_target
                  ? level_path_config_.additional_clearance
                  : filter_config_.minimum_clearance;
          if (!mappedEndpointClear(target, endpoint_clearance)) {
            if (low_altitude_mode_) {
              if (vertical_escape_allowed_) {
                // The level planner has already confirmed that no safe 3 m
                // departure exists on fresh maps. Keep the same fixed
                // ENTRY_GATE endpoint, but now let EGO search a bounded 3-D
                // detour and descend smoothly back to z=3 m at the gate.
                coverage_wait_started_ = ros::Time(0);
                ROS_WARN(
                    "[STAGE3_TASK] LOW_ALTITUDE authorized temporary 3-D "
                    "escape: mapped 3 m endpoint is occupied, EGO retains "
                    "ENTRY_GATE target (%.2f, %.2f, %.2f) and must recover to "
                    "z=%.2f",
                    target.x, target.y, target.z,
                    level_path_config_.altitude);
                publishGoal(target, GoalKind::kEntryGate);
              } else {
                requestLowIngressReplan(
                    live_level_path_target
                        ? "LIVE_LEVEL_PATH_TARGET_OCCUPIED"
                        : "ENTRY_GATE_TARGET_OCCUPIED");
              }
            } else {
              entry_gate_failure_pending_ = true;
              ascent_retry_pending_ = false;
              entry_gate_relocation_pending_ = true;
              requestHold("ENTRY_GATE_TARGET_OCCUPIED");
            }
          } else {
            const std::string static_risk = staticEndpointRisk(target);
            if (!static_risk.empty()) {
              // The final gate endpoint has already passed the conservative
              // static-geometry hard filter. A rolling intermediate point can
              // still touch the solid OBB that bounds a sparse crane mesh.
              // Bag evidence showed such a point 4.24 m from the nearest
              // occupied map sample while only 0.038 m inside the OBB's 2 m
              // inflation. Treat that mismatch as a corridor risk and let EGO
              // use the current occupancy map instead of contradicting gate
              // selection.
              ROS_WARN("[STAGE3_TASK] ENTRY_GATE_STATIC_CORRIDOR_RISK at %s "
                       "for %s; mapped endpoint is clear and remains locked "
                       "for EGO planning",
                       static_risk.c_str(), target.id.c_str());
            }
            coverage_wait_started_ = ros::Time(0);
            publishGoal(target, GoalKind::kEntryGate);
          }
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
          if (low_altitude_mode_) {
            requestLowIngressReplan(
                "ENTRY_GATE transit planner failure: " + failure);
          } else {
            entry_gate_failure_pending_ = true;
            ascent_retry_pending_ = true;
            entry_gate_relocation_pending_ = true;
            ++ascent_goal_attempts_;
            requestHold("ENTRY_GATE transit planner failure: " + failure);
          }
        } else if (noProgressTimedOut(now)) {
          if (low_altitude_mode_) {
            requestLowIngressReplan("ENTRY_GATE transit no progress");
          } else {
            entry_gate_failure_pending_ = true;
            ascent_retry_pending_ = true;
            entry_gate_relocation_pending_ = true;
            ++ascent_goal_attempts_;
            requestHold("ENTRY_GATE transit no progress");
          }
        } else if (now - goal_sent_ >
                   ros::Duration(entry_gate_target_timeout_)) {
          if (low_altitude_mode_) {
            requestLowIngressReplan("ENTRY_GATE transit target timeout");
          } else {
            entry_gate_failure_pending_ = true;
            ascent_retry_pending_ = true;
            entry_gate_relocation_pending_ = true;
            ++ascent_goal_attempts_;
            requestHold("ENTRY_GATE transit target timeout");
          }
        }
      }
    } else if (state_ == MissionState::kEvaluate || state_ == MissionState::kRelocating) {
      if (!mapFresh(now)) {
        if (now - state_entered_ > ros::Duration(map_timeout_)) {
          requestHold("MAP_STALE before target selection");
          requestReturnOrLand("map stale before target selection");
        }
      } else if (initial_waypoint_pending_) {
        if (lockDirectionalInitialSector(now, true)) {
          ROS_WARN("[STAGE3_TASK] first standard waypoint "
                   "planner-unreachable; advanced to the next standard "
                   "waypoint in the configured orbit direction");
        } else {
          requestHold("no safe standard waypoint in orbit direction");
        }
      } else if (evaluateSector(now)) {
        // Low mode must pass every newly selected orbit endpoint through the
        // fresh-map horizontal path gate in kTargetLocked. Publishing here in
        // the same timer cycle would bypass that gate after the first sector.
        if (!low_altitude_mode_) {
          publishGoal(active_target_, GoalKind::kSector);
        }
      } else {
        requestHold("no safe candidate in sector");
      }
    } else if (state_ == MissionState::kTargetLocked) {
      if (low_altitude_mode_ && !low_orbit_transit_active_ &&
          !prepareLowOrbitTransit(active_target_, now)) {
        return;
      }
      publishGoal(active_target_, GoalKind::kSector);
    } else if (state_ == MissionState::kNavigate) {
      // Confirm a stable arrival before interpreting a completed trajectory as
      // stale. This preserves the inspection hold when traj_server naturally
      // stops publishing at a reached endpoint.
      if (arrived(now)) {
        if (arrival_since_.isZero()) arrival_since_ = now;
        if (now - arrival_since_ >= ros::Duration(arrival_hold_duration_)) {
          if (low_orbit_transit_active_ &&
              low_orbit_transit_index_ + 1U <
                  low_orbit_transit_goals_.size()) {
            ++low_orbit_transit_index_;
            active_target_ =
                low_orbit_transit_goals_[low_orbit_transit_index_];
            have_sent_goal_ = false;
            arrival_since_ = ros::Time(0);
            current_target_plan_attempt_ = 0;
            transition(MissionState::kTargetLocked,
                       "live-map 3 m orbit subgoal reached");
            return;
          }
          if (low_orbit_transit_active_) {
            active_target_ = low_orbit_final_target_;
            low_orbit_transit_active_ = false;
            low_orbit_transit_goals_.clear();
            low_orbit_transit_index_ = 0U;
          }
          sectors_[current_sector_].state = SectorState::kCovered;
          initial_waypoint_pending_ = false;
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
            if (current_layer_visit_index_ + 1U <
                layer_visit_sequence_.size()) {
              std::string transition_reason;
              if (!startLayerTransition(now, &transition_reason)) {
                layer_transition_failure_pending_ = true;
                layer_transition_retry_pending_ = false;
                requestHold(transition_reason);
              }
            } else if (startExitGate(
                           now,
                           "final closed inspection layer completed; "
                           "going to current-layer EXIT_GATE")) {
              // startExitGate publishes the new active target immediately.
            } else if (buildReturnEgress()) {
              transition(
                  MissionState::kReturnEgress,
                  "current-layer EXIT_GATE unavailable; bounded "
                  "tower-exterior fallback selected");
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
          if (low_orbit_transit_active_) {
            if (vertical_escape_allowed_ &&
                active_target_.id == low_orbit_final_target_.id) {
              planner_unreachable_candidates_.insert(
                  low_orbit_final_target_.id);
              sectors_[current_sector_].locked_index = -1;
              ++candidate_relocations_;
              ++lap_candidate_relocations_;
              active_target_ = low_orbit_final_target_;
              low_orbit_transit_active_ = false;
              low_orbit_transit_goals_.clear();
              low_orbit_transit_index_ = 0U;
              low_orbit_endpoint_relocation_pending_ = true;
              requestHold(
                  "LOW_ALTITUDE elevated descent endpoint invalidated: " +
                  failure);
            } else {
              requestLowOrbitReplan("planner failure at live horizontal "
                                    "subgoal: " + failure);
            }
          } else if (failure ==
              astra_custom_msgs::PlannerStatus::GOAL_IN_OCCUPANCY) {
            requestHold(failure);
          } else {
            sector_retry_pending_ = true;
            requestHold("planner_unreachable attempt for " +
                        active_target_.id + ": " + failure);
          }
        } else if (noProgressTimedOut(now)) {
          if (low_orbit_transit_active_) {
            requestLowOrbitReplan(
                "no progress at live horizontal subgoal");
          } else {
            sector_retry_pending_ = true;
            requestHold("planner_unreachable attempt: sector no progress");
          }
        } else if (now - goal_sent_ > ros::Duration(goal_timeout_)) {
          if (low_orbit_transit_active_) {
            requestLowOrbitReplan(
                "timeout at live horizontal subgoal");
          } else {
            sector_retry_pending_ = true;
            requestHold("planner_unreachable attempt: sector target timeout");
          }
        }
      }
    } else if (state_ == MissionState::kLayerTransition) {
      if (layer_transition_index_ >= layer_transition_goals_.size()) {
        if (!activatePendingLayerAfterTransition(now)) {
          layer_transition_failure_pending_ = true;
          layer_transition_retry_pending_ = false;
          requestHold("NEXT_LAYER_ACTIVATION_FAILED");
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
          const int next_layer =
              layer_visit_sequence_[current_layer_visit_index_ + 1U];
          ROS_WARN("[STAGE3_TASK] LAYER_TRANSITION layer=%d->%d "
                   "step=%zu/%zu sector=%d target=(%.2f, %.2f, %.2f)",
                   current_layer_, next_layer,
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
    } else if (state_ == MissionState::kGoToExitGate) {
      if (!have_sent_goal_) {
        const CandidatePoint exit_gate = active_target_;
        publishGoal(exit_gate, GoalKind::kExitGate);
      } else if (arrived(now)) {
        if (arrival_since_.isZero()) arrival_since_ = now;
        if (now - arrival_since_ >=
            ros::Duration(arrival_hold_duration_)) {
          if (low_altitude_mode_) {
            ROS_WARN("[STAGE3_TASK] current-layer EXIT_GATE reached: "
                     "layer=%d xyz=(%.3f, %.3f, %.3f); leaving tower-facing "
                     "mode and replaying the safe ingress in reverse",
                     current_layer_, active_target_.x, active_target_.y,
                     active_target_.z);
            if (!startNormalReturn(
                    "current-layer EXIT_GATE reached", false)) {
              requestBridgeReturnOrLand(
                  "current-layer EXIT_GATE reached; reverse ingress "
                  "unavailable");
            }
          } else {
            requestBridgeReturnOrLand(
                "current-layer EXIT_GATE reached; direct EGO return home");
          }
        }
      } else {
        arrival_since_ = ros::Time(0);
        std::string failure;
        if (plannerFailure(now, &failure)) {
          requestReturnOrLand("EXIT_GATE planner failure: " + failure);
        } else if (noProgressTimedOut(now)) {
          requestReturnOrLand("EXIT_GATE no progress");
        } else if (now - goal_sent_ >
                   ros::Duration(return_egress_target_timeout_)) {
          requestReturnOrLand("EXIT_GATE target timeout");
        }
      }
    } else if (state_ == MissionState::kHolding) {
      if (now - state_entered_ >= ros::Duration(failure_hold_duration_)) {
        if (low_ingress_replan_pending_) {
          if (low_ingress_replans_ > max_low_ingress_replans_) {
            low_ingress_replan_pending_ = false;
            requestReturnOrLand(
                "low-altitude ingress replan budget exhausted");
          } else if (!mapFresh(now)) {
            if (now - state_entered_ >=
                ros::Duration(coverage_wait_timeout_)) {
              low_ingress_replan_pending_ = false;
              requestReturnOrLand(
                  "low-altitude ingress map remained stale during HOLD");
            }
          } else {
            if (!entry_gate_locked_) {
              entry_gate_locked_ = lockLayerGate(0, now);
              if (entry_gate_locked_) {
                ROS_WARN(
                    "[STAGE3_TASK] LOW_ALTITUDE HOLD retry locked a new "
                    "same-sector ENTRY_GATE candidate: %s",
                    provisional_entry_gate_.id.c_str());
              }
            }
            const bool restarted =
                entry_gate_locked_ && startEntryGateTransit();
            if (restarted) {
              ROS_WARN("[STAGE3_TASK] LOW_ALTITUDE fresh map rebuilt ingress "
                       "after HOLD (replan=%d/%d policy=%s)",
                       low_ingress_replans_, max_low_ingress_replans_,
                       altitude_policy_.c_str());
            } else if (now - state_entered_ >=
                       ros::Duration(coverage_wait_timeout_)) {
              low_ingress_replan_pending_ = false;
              requestReturnOrLand(
                  "low-altitude ingress remained unreachable after "
                  "fresh-map HOLD replanning");
            }
          }
        } else if (low_orbit_endpoint_relocation_pending_) {
          low_orbit_endpoint_relocation_pending_ = false;
          vertical_escape_allowed_ = false;
          horizontal_path_available_ = false;
          low_orbit_no_path_confirmations_ = 0;
          last_low_orbit_no_path_check_ = ros::Time(0);
          have_sent_goal_ = false;
          transition(MissionState::kRelocating,
                     "elevated descent endpoint rejected; selecting "
                     "another candidate in the same sector");
        } else if (low_orbit_replan_pending_) {
          low_orbit_replan_pending_ = false;
          if (low_orbit_replans_ > max_low_orbit_replans_) {
            requestReturnOrLand(
                "low-altitude orbit replan budget exhausted");
          } else if (!mapFresh(now)) {
            low_orbit_replan_pending_ = true;
            if (now - state_entered_ >=
                ros::Duration(coverage_wait_timeout_)) {
              low_orbit_replan_pending_ = false;
              requestReturnOrLand(
                  "low-altitude orbit map remained stale during HOLD");
            }
          } else {
            have_sent_goal_ = false;
            transition(MissionState::kTargetLocked,
                       "fresh-map 3 m orbit path rebuild after HOLD");
          }
        } else if (entry_gate_no_safe_candidate_hold_) {
          if (entry_gate_recheck_time_.isZero() ||
              now - entry_gate_recheck_time_ >=
                  ros::Duration(safe_altitude_map_dwell_)) {
            entry_gate_recheck_time_ = now;
            if (lockLayerGate(0, now) && startEntryGateTransit()) {
              entry_gate_no_safe_candidate_hold_ = false;
              entry_gate_failure_pending_ = false;
              entry_gate_relocation_pending_ = false;
              ROS_WARN("[STAGE3_TASK] fresh map exposed a safe reachable "
                       "ENTRY_GATE candidate inside sector %d; leaving HOLD",
                       entry_sector_user_);
            } else {
              ROS_ERROR_THROTTLE(
                  1.0,
                  "[STAGE3_TASK] %s; no other sector will be searched, "
                  "mission remains HOLD",
                  entry_gate_last_failure_.c_str());
            }
          }
        } else if (layer_transition_failure_pending_) {
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
          } else if (entry_gate_relocation_pending_) {
            entry_gate_relocation_pending_ = false;
            relocateEntryGateWithinConfiguredSector(now, failure_reason_);
          } else {
            requestReturnOrLand(
                "fixed vertical ascent/ENTRY_GATE recovery exhausted; "
                "lateral channel search disabled");
          }
        } else if (exit_gate_failure_pending_) {
          exit_gate_failure_pending_ = false;
          requestReturnOrLand(
              failure_reason_ +
              "; EXIT_GATE tracking failed, using bounded safe fallback");
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
        layer_visit_sequence_.empty()
            ? 0.0
            : std::min(
                  1.0,
                  (static_cast<double>(current_layer_visit_index_) +
                   layer_progress) /
                      static_cast<double>(layer_visit_sequence_.size()));
    progress_pub_.publish(progress);
    publishCandidateDebug(now);
    if (report_) {
      if (last_report_.isZero() || now - last_report_ >= ros::Duration(report_period_)) {
        const auto position = pointOf(odom_);
        const double actual_yaw =
            yawFromQuaternion(odom_.pose.pose.orientation);
        const double horizontal_speed =
            std::hypot(odom_.twist.twist.linear.x,
                       odom_.twist.twist.linear.y);
        report_ << now.toSec() << ',' << missionStateName(state_) << ','
                << current_layer_ << ','
                << (current_sector_ < sectors_.size()
                        ? sectors_[current_sector_].sector_id
                        : -1)
                << ',' << active_target_.id << ','
                << active_target_.x << ',' << active_target_.y << ','
                << active_target_.z << ',' << position.x << ',' << position.y
                << ',' << position.z << ',' << actual_yaw << ','
                << horizontal_speed << ','
                << (active_target_.face_tower ? "FACE_TOWER"
                                              : "VELOCITY_FORWARD")
                << ','
                << (have_command_ ? command_.position.x : position.x) << ','
                << (have_command_ ? command_.position.y : position.y) << ','
                << (have_command_ ? command_.position.z : position.z) << ','
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
                << ',' << current_layer_visit_index_
                << ',' << (static_cast<int>(
                                current_layer_visit_index_ /
                                inspection_heights_.size()) +
                            1)
                << ',' << planned_cycles_
                << ',' << (final_return_ ? 1 : 0)
                << ',' << altitude_policy_
                << ',' << (horizontal_path_available_ ? 1 : 0)
                << ',' << (vertical_escape_allowed_ ? 1 : 0)
                << ',' << low_ingress_replans_
                << ',' << low_orbit_replans_
                << '\n';
        report_.flush(); last_report_ = now;
      }
    }
  }

  ros::NodeHandle node_, private_node_;
  ros::Subscriber odom_sub_, command_sub_, cloud_sub_, occupancy_sub_,
      planner_status_sub_, bridge_state_sub_, fcu_state_sub_,
      extended_state_sub_;
  ros::Publisher goal_pub_, state_pub_, target_pub_, sector_pub_,
      candidates_pub_, progress_pub_, face_tower_pub_, tower_center_pub_,
      level_ingress_path_pub_, level_orbit_path_pub_, altitude_policy_pub_;
  ros::ServiceClient tracking_client_, cancel_client_, resume_client_, return_client_, land_client_;
  ros::Timer timer_;
  std::string planning_frame_, odom_topic_, command_topic_, cloud_topic_, occupancy_topic_, planner_status_topic_,
      bridge_state_topic_, goal_topic_, state_topic_, target_topic_, sector_topic_, candidates_topic_,
      progress_topic_, face_tower_topic_, tower_center_topic_,
      level_ingress_path_topic_, level_orbit_path_topic_,
      altitude_policy_topic_,
      mavros_state_topic_, mavros_extended_state_topic_, tracking_service_,
      cancel_service_, resume_service_, return_service_, land_service_;
  bool enable_control_{false};
  bool low_altitude_mode_{false};
  bool use_configured_staging_xy_{false};
  bool prefer_safe_overflight_{true};
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
      max_ascent_channel_switches_{0},
      max_return_egress_retries_{2},
      max_normal_return_retries_{2}, sector_count_{8}, sector_limit_{1},
      layer_count_{1}, inspection_laps_{1}, planned_cycles_{1};
  double inspection_height_{std::numeric_limits<double>::quiet_NaN()},
      recovery_height_max_{40.0}, virtual_ceil_height_{45.0},
      transit_height_{4.0};
  double observation_radius_offset_{5.0}, observation_angle_deg_{-67.5}, approach_segment_length_{6.0};
  double sector_angle_half_width_deg_{12.0}, sector_radius_half_width_{4.0}, sector_height_half_width_{1.0};
  double maximum_temporary_descent_{3.0};
  double target_replacement_margin_{2.0}, recovery_height_{35.0}, start_angle_deg_{0.0};
  double entry_gate_segment_length_{6.0}, entry_gate_target_timeout_{90.0};
  double layer_transition_target_timeout_{90.0};
  double layer_transition_maximum_vertical_step_{2.0};
  double layer_transition_same_xy_tolerance_{0.25};
  double staging_height_{4.0}, climb_height_step_{3.0},
      minimum_channel_hold_{5.0};
  double configured_staging_x_{0.0}, configured_staging_y_{0.0};
  double low_altitude_tolerance_{0.35};
  double low_no_path_confirmation_period_{1.0};
  std::string direction_, recovery_direction_, report_file_,
      validation_reason_, altitude_policy_;
  RouteConfig route_;
  LevelPathConfig level_path_config_;
  CandidateFilterConfig filter_config_;
  RecoveryConfig recovery_config_;
  ReturnEgressConfig return_egress_config_;
  std::vector<CandidateOffset> offsets_;
  std::vector<double> inspection_heights_, ascent_step_heights_;
  EntryGateConfig entry_gate_config_;
  std::vector<StaticObstacle> obstacles_;
  std::vector<Sector> sectors_;
  std::vector<std::vector<Sector>> layer_sector_data_, layer_sector_runtime_;
  std::vector<Sector> pending_layer_sectors_;
  std::vector<CandidatePoint> approach_goals_;
  std::vector<CandidatePoint> entry_gate_transit_goals_;
  std::vector<CandidatePoint> successful_ingress_goals_;
  std::vector<CandidatePoint> successful_ingress_trace_;
  std::vector<CandidatePoint> low_orbit_transit_goals_;
  std::vector<CandidatePoint> normal_return_goals_;
  std::vector<CandidatePoint> return_egress_goals_;
  std::vector<CandidatePoint> entry_gate_candidates_;
  std::vector<CandidatePoint> layer_transition_goals_;
  std::vector<CandidatePoint> failed_entry_gate_positions_;
  std::vector<CandidatePoint> layer_entry_gates_, layer_start_anchors_;
  std::vector<bool> layer_entry_gate_valid_, layer_start_anchor_valid_;
  std::vector<int> layer_visit_sequence_;
  RecoveryTargets recovery_targets_;
  std::vector<geometry_msgs::Point> cloud_points_, occupancy_points_;
  std::unordered_map<std::int64_t, double> coverage_rays_;
  std::unordered_set<std::string> planner_unreachable_candidates_;
  std::unordered_set<std::string> unreachable_entry_gate_candidates_;
  std::vector<std::size_t> lap_visit_sequence_;
  std::size_t current_sector_{0}, approach_index_{0}, recovery_step_{0},
      entry_gate_transit_index_{0}, return_egress_index_{0},
      normal_return_index_{0}, layer_transition_index_{0},
      low_orbit_transit_index_{0},
      visited_sector_count_{0}, visit_cursor_{0},
      current_layer_visit_index_{0};
  int return_egress_retries_{0}, normal_return_retries_{0},
      current_target_plan_attempt_{0}, entry_gate_relocations_{0},
      current_lap_{1},
      completed_laps_{0}, waypoint_in_lap_{0}, candidate_relocations_{0},
      lap_candidate_relocations_{0}, lap_planning_failures_{0},
      lap_recoveries_{0}, low_no_path_confirmations_{0},
      low_orbit_no_path_confirmations_{0},
      low_no_path_confirmation_limit_{3}, low_ingress_replans_{0},
      low_orbit_replans_{0}, max_low_ingress_replans_{5},
      max_low_orbit_replans_{12};
  int entry_sector_user_{1}, entry_sector_index_{0},
      inspection_start_sector_{0}, transition_sector_{0},
      current_layer_{0}, completed_layer_count_{0},
      layer_transition_attempts_{0};
  int entry_gate_index_{-1};
  int provisional_entry_gate_index_{-1};
  int ascent_channel_index_{0}, ascent_goal_attempts_{0};
  CandidatePoint provisional_entry_gate_, staging_target_,
      last_inspection_target_, layer_start_anchor_, layer_transition_anchor_,
      pending_layer_start_anchor_, low_orbit_final_target_;
  CandidatePoint active_target_, last_published_target_;
  GoalKind active_goal_kind_{GoalKind::kEntryGate};
  MissionState state_{MissionState::kWaitInputs};
  MissionState state_before_hold_{MissionState::kWaitInputs};
  std::string bridge_state_, failure_reason_, entry_gate_last_failure_;
  nav_msgs::Odometry odom_;
  quadrotor_msgs::PositionCommand command_;
  astra_custom_msgs::PlannerStatus planner_status_;
  mavros_msgs::State fcu_state_;
  mavros_msgs::ExtendedState extended_state_;
  bool have_odom_{false}, have_command_{false}, have_cloud_{false}, have_occupancy_{false}, have_planner_status_{false},
      have_bridge_state_{false}, have_sent_goal_{false};
  bool have_home_position_{false}, entry_gate_failure_pending_{false},
      entry_gate_relocation_pending_{false},
      entry_gate_no_safe_candidate_hold_{false},
      exit_gate_failure_pending_{false},
      return_egress_failure_pending_{false},
      normal_return_failure_pending_{false}, sector_retry_pending_{false},
      ascent_retry_pending_{false}, normal_return_attempted_{false},
      entry_gate_locked_{false}, have_last_inspection_target_{false},
      have_layer_start_anchor_{false}, initial_waypoint_pending_{false},
      layer_transition_failure_pending_{false},
      layer_transition_retry_pending_{false},
      final_return_{false},
      have_last_published_target_{false},
      have_fcu_state_{false}, have_extended_state_{false},
      horizontal_path_available_{false}, vertical_escape_allowed_{false},
      low_ingress_replan_pending_{false},
      low_orbit_transit_active_{false},
      low_orbit_replan_pending_{false},
      low_orbit_endpoint_relocation_pending_{false};
  geometry_msgs::Point home_position_, coverage_origin_;
  ros::Time odom_received_, command_received_, cloud_received_,
      occupancy_received_, planner_status_received_, bridge_state_received_,
      coverage_received_, fcu_state_received_, extended_state_received_;
  ros::Time state_entered_, mission_started_, goal_sent_, arrival_since_,
      last_report_, bridge_state_entered_, coverage_wait_started_,
      landing_requested_time_, channel_selected_time_, last_progress_time_,
      entry_gate_recheck_time_, last_low_no_path_check_;
  ros::Time last_low_orbit_no_path_check_;
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
