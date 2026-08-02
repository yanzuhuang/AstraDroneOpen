/**
 * Stage 3 EGO static-obstacle mission supervisor.
 *
 * This node owns task policy only.  It never publishes a MAVROS setpoint;
 * bridge services are the sole HOLD/RESUME/RETURN/LAND control boundary.
 */
#include "astra_tower_mission/ego_task_utils.h"
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
  kWaitEntryPermission,
  kEntryGateTransit,
  kWaitOrbitStagingPermission,
  kWaitOrbitPermission,
  kEvaluate,
  kTargetLocked,
  kNavigate,
  kWaitLayerPermission,
  kHolding,
  kRelocating,
  kRecovering,
  kLayerTransition,
  kWaitExitPermission,
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
    case MissionState::kWaitEntryPermission: return "WAIT_ENTRY_PERMISSION";
    case MissionState::kEntryGateTransit: return "ENTRY_GATE_TRANSIT";
    case MissionState::kWaitOrbitStagingPermission: return "ENTRY_READY";
    case MissionState::kWaitOrbitPermission: return "ORBIT_STAGING_READY";
    case MissionState::kEvaluate: return "EVALUATING";
    case MissionState::kTargetLocked: return "TARGET_LOCKED";
    case MissionState::kNavigate: return "NAVIGATING";
    case MissionState::kWaitLayerPermission: return "WAIT_TRANSITION_PERMISSION";
    case MissionState::kHolding: return "HOLDING";
    case MissionState::kRelocating: return "RELOCATING";
    case MissionState::kRecovering: return "RECOVERING";
    case MissionState::kLayerTransition: return "LAYER_TRANSITION";
    case MissionState::kWaitExitPermission: return "WAIT_EXIT_PERMISSION";
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
  kSectorDetour,
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

double pointSegmentDistance(const geometry_msgs::Point& point,
                            const geometry_msgs::Point& start,
                            const geometry_msgs::Point& finish) {
  const double vx = finish.x - start.x;
  const double vy = finish.y - start.y;
  const double vz = finish.z - start.z;
  const double length_squared = vx * vx + vy * vy + vz * vz;
  if (length_squared <= 1.0e-12) {
    return std::sqrt((point.x - start.x) * (point.x - start.x) +
                     (point.y - start.y) * (point.y - start.y) +
                     (point.z - start.z) * (point.z - start.z));
  }
  const double projection = std::max(
      0.0, std::min(1.0,
                    ((point.x - start.x) * vx +
                     (point.y - start.y) * vy +
                     (point.z - start.z) * vz) /
                        length_squared));
  geometry_msgs::Point nearest;
  nearest.x = start.x + projection * vx;
  nearest.y = start.y + projection * vy;
  nearest.z = start.z + projection * vz;
  return std::sqrt((point.x - nearest.x) * (point.x - nearest.x) +
                   (point.y - nearest.y) * (point.y - nearest.y) +
                   (point.z - nearest.z) * (point.z - nearest.z));
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
    ROS_WARN("[STAGE3_TASK] uav_id=%d control=%s takeoff_delay=%.1f "
             "sectors=%d layers=%zu cycles=%d "
             "inspection_height=%.2f radius=%.2f "
             "entry_sector_user=%d entry_sector_index=%d "
             "entry_angle=%.1fdeg pre_entry_radius=%.2f "
             "recovery_max=%.2f virtual_ceil=%.2f; no MAVROS publisher",
             uav_id_, enable_control_ ? "true" : "false", takeoff_delay_,
             sector_count_,
             inspection_heights_.size(), planned_cycles_, inspection_height_,
             route_.radius,
             entry_sector_user_, entry_sector_index_, entry_angle_deg_,
             pre_entry_radius_,
             recovery_height_max_, virtual_ceil_height_);
  }

 private:
  struct EntryCorridorCandidate {
    std::string id;
    // ID of the ordinary sector candidate used as ORBIT_STAGING.  Keeping the
    // source ID makes the joint ENTRY selection and the normal sector
    // candidate/retry state machine operate on the same target identity.
    std::string orbit_candidate_id;
    CandidatePoint pre_entry;
    CandidatePoint entry_gate;
    CandidatePoint orbit_staging;
    std::vector<geometry_msgs::Point> planned_path;
    double angle_deg{0.0};
    double pre_radius{0.0};
    double entry_radius{0.0};
    double minimum_clearance{0.0};
    double score{-1.0e9};
  };

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
             "metadata_center=%.1fdeg actual_continuous_angle=%.1fdeg "
             "metadata_range=[%.1f,%.1f]deg; cross-sector fallback=false",
             entry_sector_user_, entrySectorDirectionName(entry_sector_user_),
             selected_center_deg, entry_angle_deg_, selected_lower_deg,
             selected_upper_deg);
  }

  void loadConfig() {
    private_node_.param("enable_control", enable_control_, false);
    private_node_.param("uav_id", uav_id_, 0);
    private_node_.param("takeoff_delay", takeoff_delay_, 0.0);
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
    private_node_.param("low_altitude/entry_height",
                        level_path_config_.altitude, 3.0);
    private_node_.param("low_altitude/vertical_half_extent",
                        level_path_config_.vertical_half_extent, 0.2);
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
    private_node_.param("low_altitude/path_score/path_length_weight",
                        level_path_config_.cost.path_length, 1.0);
    private_node_.param("low_altitude/path_score/goal_deviation_weight",
                        level_path_config_.cost.goal_deviation, 0.03);
    private_node_.param("low_altitude/path_score/tower_distance_weight",
                        level_path_config_.cost.tower_distance, 0.08);
    private_node_.param("low_altitude/path_score/turn_weight",
                        level_path_config_.cost.turn, 0.35);
    private_node_.param("low_altitude/path_score/clearance_weight",
                        level_path_config_.cost.obstacle_clearance, 0.05);

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
    private_node_.param<std::string>(
        "outputs/entry_corridor_candidates", entry_corridor_candidates_topic_,
        "/tower_mission/entry_corridor_candidates");
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
    private_node_.param<std::string>("outputs/mission_route",
                                     mission_route_topic_,
                                     "/tower_mission/mission_route");
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
    private_node_.param("coordination/require_transition_permission",
                        require_transition_permission_, false);
    private_node_.param("coordination/require_task_start_permission",
                        require_task_start_permission_, false);
    private_node_.param("coordination/require_entry_permission",
                        require_entry_permission_, false);
    private_node_.param("coordination/require_exit_permission",
                        require_exit_permission_, false);
    private_node_.param("coordination/require_orbit_permission",
                        require_orbit_permission_, false);
    private_node_.param("coordination/require_orbit_staging_permission",
                        require_orbit_staging_permission_, false);
    private_node_.param("coordination/require_landing_permission",
                        require_landing_permission_, false);
    private_node_.param("coordination/permission_timeout",
                        coordination_permission_timeout_, 1.0);
    private_node_.param<std::string>(
        "coordination/task_start_permission_topic",
        task_start_permission_topic_, "/tower_mission/task_start_permission");
    private_node_.param<std::string>(
        "coordination/transition_permission_topic",
        transition_permission_topic_,
        "/tower_mission/transition_permission");
    private_node_.param<std::string>(
        "coordination/entry_permission_topic",
        entry_permission_topic_, "/tower_mission/entry_permission");
    private_node_.param<std::string>(
        "coordination/entry_corridor_selection_topic",
        entry_corridor_selection_topic_,
        "/tower_mission/entry_corridor_selection");
    private_node_.param<std::string>(
        "coordination/exit_permission_topic",
        exit_permission_topic_, "/tower_mission/exit_permission");
    private_node_.param<std::string>(
        "coordination/orbit_permission_topic",
        orbit_permission_topic_, "/tower_mission/orbit_permission");
    private_node_.param<std::string>(
        "coordination/orbit_staging_permission_topic",
        orbit_staging_permission_topic_,
        "/tower_mission/orbit_staging_permission");
    private_node_.param<std::string>(
        "coordination/landing_permission_topic",
        landing_permission_topic_, "/tower_mission/landing_permission");

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
    level_path_config_.use_tower_constraint = low_altitude_mode_;
    level_path_config_.tower_x = route_.center_x;
    level_path_config_.tower_y = route_.center_y;
    level_path_config_.tower_keep_out_radius =
        route_.tower_collision_radius + route_.minimum_safety_distance;
    level_path_config_.preferred_tower_radius = route_.radius;
    private_node_.param("mission/transit_height", transit_height_, 4.0);
    private_node_.param("mission/observation_radius_offset",
                        observation_radius_offset_, 5.0);
    private_node_.param("mission/observation_angle_deg", observation_angle_deg_,
                        start_angle_deg_);
    private_node_.param("mission/approach_segment_length", approach_segment_length_,
                        6.0);
    private_node_.param("entry_gate/entry_sector", entry_sector_user_, 1);
    entry_sector_index_ = entry_sector_user_ - 1;
    entry_angle_deg_ = static_cast<double>(entry_sector_user_ - 1) * 45.0;
    private_node_.param("entry_gate/angle_deg", entry_angle_deg_,
                        entry_angle_deg_);
    entry_angle_deg_ = positiveAngleDegrees(entry_angle_deg_ * kPi / 180.0);
    entry_angle_rad_ = entry_angle_deg_ * kPi / 180.0;
    entry_nominal_angle_deg_ = entry_angle_deg_;
    // Each vehicle's continuous orbit reference begins at its own exact gate
    // angle.  The eight sectors remain coverage/progress bins and are shifted
    // consistently; no vehicle is snapped back to a shared 45-degree point.
    start_angle_deg_ = entry_angle_deg_;
    route_.start_angle_rad = entry_angle_rad_;
    private_node_.param("entry_gate/pre_entry_radius", pre_entry_radius_,
                        18.0);
    private_node_.param("entry_gate/pre_entry_maximum_radius",
                        pre_entry_maximum_radius_, 19.5);
    private_node_.param("entry_gate/pre_entry_radial_sample_step",
                        pre_entry_radial_sample_step_, 0.25);
    private_node_.param("entry_gate/angular_search_half_width_deg",
                        entry_angular_search_half_width_deg_, 7.5);
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
    private_node_.param("entry_gate/map_points_are_inflated",
                        entry_gate_config_.map_points_are_inflated, false);
    private_node_.param("entry_gate/map_additional_clearance",
                        entry_gate_config_.map_additional_clearance, 0.0);
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
    private_node_.param("planner_map/size_x", planner_map_size_x_, 0.0);
    private_node_.param("planner_map/size_y", planner_map_size_y_, 0.0);
    private_node_.param("planner_map/planning_horizon",
                        planner_map_planning_horizon_, 0.0);
    private_node_.param("mission/maximum_temporary_descent",
                        maximum_temporary_descent_, 3.0);
    private_node_.param("mission/target_replacement_margin",
                        target_replacement_margin_, 2.0);

    private_node_.param("candidate/minimum_clearance", filter_config_.minimum_clearance,
                        2.0);
    private_node_.param("candidate/cloud_inflation", filter_config_.cloud_inflation,
                        0.4);
    private_node_.param("candidate/map_points_are_inflated",
                        filter_config_.map_points_are_inflated, false);
    private_node_.param("candidate/map_additional_clearance",
                        filter_config_.map_additional_clearance, 0.0);
    private_node_.param("candidate/unknown_is_hard_constraint",
                        filter_config_.unknown_is_hard_constraint, false);
    private_node_.param("candidate/known_obstacle_is_hard_constraint",
                        filter_config_.known_obstacle_is_hard_constraint,
                        true);
    private_node_.param(
        "candidate/known_obstacle_corridor_is_hard_constraint",
        filter_config_.known_obstacle_corridor_is_hard_constraint, false);
    private_node_.param("candidate/prefer_clear_straight_corridor",
                        filter_config_.prefer_clear_straight_corridor, false);
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
    route_.start_angle_rad = entry_angle_rad_;
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
    if (low_altitude_mode_ && planner_map_size_x_ > 0.0 &&
        planner_map_size_y_ > 0.0) {
      const double maximum_radius_offset =
          radius_offsets.empty()
              ? 0.0
              : std::max(0.0, *std::max_element(radius_offsets.begin(),
                                                radius_offsets.end()));
      std::string map_contract;
      if (!plannerMapContainsOrbitEnvelope(
              route_.center_x, route_.center_y,
              route_.radius + maximum_radius_offset,
              planner_map_planning_horizon_, planner_map_size_x_,
              planner_map_size_y_, &map_contract)) {
        throw std::runtime_error(
            "EGO map does not contain the full orbit plus planning horizon: " +
            map_contract);
      }
      ROS_WARN("[STAGE3_PREFLIGHT] EGO map envelope verified: %s",
               map_contract.c_str());
    }
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
        !std::isfinite(route_.start_angle_rad) ||
        pre_entry_radius_ <= entry_gate_config_.maximum_radius ||
        pre_entry_maximum_radius_ < pre_entry_radius_ ||
        pre_entry_radial_sample_step_ <= 0.0 ||
        entry_angular_search_half_width_deg_ < 0.0 ||
        entry_angular_search_half_width_deg_ > 22.5 ||
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
    if (uav_id_ < 0 || !std::isfinite(takeoff_delay_) ||
        takeoff_delay_ < 0.0) {
      throw std::runtime_error("uav_id and takeoff_delay must be non-negative");
    }
    if (low_altitude_mode_) {
      const bool low_envelope_valid =
          !inspection_heights_.empty() &&
          std::abs(inspection_height_ - level_path_config_.altitude) <=
              1.0e-6 &&
          std::abs(transit_height_ - level_path_config_.altitude) <=
              1.0e-6 &&
          level_path_config_.vertical_half_extent > 0.0 &&
          level_path_config_.additional_clearance >= 0.0 &&
          level_path_config_.resolution > 0.0 &&
          level_path_config_.boundary_margin > 0.0 &&
          level_path_config_.maximum_segment_length > 0.0 &&
          level_path_config_.use_tower_constraint &&
          level_path_config_.tower_keep_out_radius >=
              route_.tower_collision_radius +
                  route_.minimum_safety_distance &&
          level_path_config_.preferred_tower_radius >=
              level_path_config_.tower_keep_out_radius &&
          level_path_config_.cost.path_length > 0.0 &&
          level_path_config_.cost.goal_deviation >= 0.0 &&
          level_path_config_.cost.tower_distance >= 0.0 &&
          level_path_config_.cost.turn >= 0.0 &&
          level_path_config_.cost.obstacle_clearance >= 0.0 &&
          entry_gate_config_.map_points_are_inflated &&
          filter_config_.map_points_are_inflated &&
          entry_gate_config_.map_additional_clearance >= 0.0 &&
          filter_config_.map_additional_clearance >= 0.0 &&
          std::abs(entry_gate_config_.map_additional_clearance -
                   level_path_config_.additional_clearance) <= 1.0e-9 &&
          std::abs(filter_config_.map_additional_clearance -
                   level_path_config_.additional_clearance) <= 1.0e-9 &&
          low_altitude_tolerance_ > 0.0 &&
          low_no_path_confirmation_limit_ >= 1 &&
          low_no_path_confirmation_period_ > 0.0 &&
          max_low_ingress_replans_ >= 1;
      const bool nominal_height_only = std::all_of(
          offsets_.begin(), offsets_.end(), [](const CandidateOffset& offset) {
            return std::abs(offset.height_m) <= 1.0e-9;
          });
      if (!low_envelope_valid || !nominal_height_only) {
        throw std::runtime_error(
            "low-altitude mode requires nominal-height-only layers and an "
            "entry height matching the first inspection layer");
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
    if (require_transition_permission_) {
      transition_permission_sub_ = node_.subscribe(
          transition_permission_topic_, 5,
          &Stage3EgoMissionNode::transitionPermissionCallback, this);
    }
    if (require_task_start_permission_) {
      task_start_permission_sub_ = node_.subscribe(
          task_start_permission_topic_, 5,
          &Stage3EgoMissionNode::taskStartPermissionCallback, this);
    }
    if (require_entry_permission_) {
      entry_permission_sub_ = node_.subscribe(
          entry_permission_topic_, 5,
          &Stage3EgoMissionNode::entryPermissionCallback, this);
      entry_corridor_selection_sub_ = node_.subscribe(
          entry_corridor_selection_topic_, 5,
          &Stage3EgoMissionNode::entryCorridorSelectionCallback, this);
    }
    if (require_exit_permission_) {
      exit_permission_sub_ = node_.subscribe(
          exit_permission_topic_, 5,
          &Stage3EgoMissionNode::exitPermissionCallback, this);
    }
    if (require_orbit_permission_) {
      orbit_permission_sub_ = node_.subscribe(
          orbit_permission_topic_, 5,
          &Stage3EgoMissionNode::orbitPermissionCallback, this);
    }
    if (require_orbit_staging_permission_) {
      orbit_staging_permission_sub_ = node_.subscribe(
          orbit_staging_permission_topic_, 5,
          &Stage3EgoMissionNode::orbitStagingPermissionCallback, this);
    }
    if (require_landing_permission_) {
      landing_permission_sub_ = node_.subscribe(
          landing_permission_topic_, 5,
          &Stage3EgoMissionNode::landingPermissionCallback, this);
    }
    goal_pub_ = node_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1);
    state_pub_ = node_.advertise<std_msgs::String>(state_topic_, 5, true);
    target_pub_ = node_.advertise<geometry_msgs::PoseStamped>(target_topic_, 1,
                                                               true);
    sector_pub_ = node_.advertise<std_msgs::UInt32>(sector_topic_, 5, true);
    candidates_pub_ = node_.advertise<astra_custom_msgs::InspectionCandidateArray>(
        candidates_topic_, 5, true);
    entry_corridor_candidates_pub_ =
        node_.advertise<astra_custom_msgs::InspectionCandidateArray>(
            entry_corridor_candidates_topic_, 1, true);
    progress_pub_ = node_.advertise<std_msgs::Float64>(progress_topic_, 5);
    face_tower_pub_ = node_.advertise<std_msgs::Bool>(face_tower_topic_, 1, true);
    tower_center_pub_ = node_.advertise<geometry_msgs::PointStamped>(
        tower_center_topic_, 1, true);
    level_ingress_path_pub_ =
        node_.advertise<nav_msgs::Path>(level_ingress_path_topic_, 1, true);
    level_orbit_path_pub_ =
        node_.advertise<nav_msgs::Path>(level_orbit_path_topic_, 1, true);
    mission_route_pub_ =
        node_.advertise<nav_msgs::Path>(mission_route_topic_, 1, true);
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
      publishAltitudePolicy("WAITING_FOR_ENTRY_HOVER_AND_FRESH_MAP");
    }
    tracking_client_ = node_.serviceClient<std_srvs::SetBool>(tracking_service_);
    cancel_client_ = node_.serviceClient<std_srvs::Trigger>(cancel_service_);
    resume_client_ = node_.serviceClient<std_srvs::Trigger>(resume_service_);
    return_client_ = node_.serviceClient<std_srvs::Trigger>(return_service_);
    land_client_ = node_.serviceClient<std_srvs::Trigger>(land_service_);
    report_.open(report_file_, std::ios::out | std::ios::trunc);
    if (report_) {
      report_ << "sim_time,state,layer,sector,target_id,target_x,target_y,target_z,"
                 "mission_target_id,mission_target_x,mission_target_y,"
                 "mission_target_z,previous_target_id,target_switch_reason,"
                 "distance_to_target,ego_goal_publish_count,"
                 "goal_publication_source,"
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
    if (low_altitude_mode_ && state_ == MissionState::kEntryGateTransit) {
      recordIngressTrace(msg->pose.pose.position);
    }
    const bool orbit_state = orbit_released_latched_ && (
        state_ == MissionState::kEvaluate ||
        state_ == MissionState::kTargetLocked ||
        state_ == MissionState::kNavigate ||
        state_ == MissionState::kRelocating ||
        state_ == MissionState::kRecovering ||
        (state_ == MissionState::kHolding && orbit_tracking_initialized_));
    if (orbit_state) updateOrbitProgress(msg->pose.pose.position);
  }

  void updateOrbitProgress(const geometry_msgs::Point& position) {
    const double angle = normalizeAngle(std::atan2(
        position.y - route_.center_y, position.x - route_.center_x));
    if (!orbit_tracking_initialized_) {
      // Each UAV starts its independent lap at its own formal release while
      // physically latched at the exact first orbit point.  ENTRY->staging
      // motion must never be credited toward the lap.
      orbit_start_angle_ = angle;
      previous_orbit_angle_ = angle;
      accumulated_orbit_angle_ = 0.0;
      visited_sector_mask_ = 0U;
      orbit_tracking_initialized_ = true;
    } else {
      const double delta = normalizeAngle(angle - previous_orbit_angle_);
      const double directed_delta =
          route_.direction == OrbitDirection::kCounterClockwise ? delta : -delta;
      // Reject localization jumps and never credit reverse motion toward a
      // completed lap.  A reverse segment is visible in diagnostics but cannot
      // make the independent completion predicate pass early.
      if (std::abs(delta) <= kPi / 2.0 && directed_delta > 0.0) {
        accumulated_orbit_angle_ += directed_delta;
      }
      if (directed_delta < -5.0 * kPi / 180.0) reverse_orbit_detected_ = true;
      previous_orbit_angle_ = angle;
    }
    double positive = positiveAngleDegrees(angle);
    const int coverage_sector = static_cast<int>(
        std::floor((positive + 22.5) / 45.0)) % 8;
    visited_sector_mask_ |= static_cast<std::uint8_t>(1U << coverage_sector);
  }

  bool independentOrbitComplete(std::string* reason) const {
    const double accumulated_deg = accumulated_orbit_angle_ * 180.0 / kPi;
    const double start_error_deg = std::abs(normalizeAngle(
        previous_orbit_angle_ - orbit_start_angle_)) * 180.0 / kPi;
    const bool angle_complete = accumulated_deg >= 345.0;
    const bool coverage_complete = visited_sector_mask_ == 0xFFU;
    const bool returned_to_start = start_error_deg <= 15.0;
    if (reason != nullptr) {
      std::ostringstream stream;
      stream << "accumulated=" << accumulated_deg
             << "deg sector_mask=0x" << std::hex
             << static_cast<int>(visited_sector_mask_) << std::dec
             << " start_error=" << start_error_deg
             << "deg reverse_detected="
             << (reverse_orbit_detected_ ? "true" : "false");
      *reason = stream.str();
    }
    return angle_complete && coverage_complete && returned_to_start &&
           !reverse_orbit_detected_;
  }

  void taskStartPermissionCallback(const std_msgs::Bool::ConstPtr& msg) {
    task_start_permission_ = msg->data;
    task_start_permission_received_ = ros::Time::now();
  }

  void transitionPermissionCallback(const std_msgs::Bool::ConstPtr& msg) {
    transition_permission_ = msg->data;
    transition_permission_received_ = ros::Time::now();
  }

  void entryPermissionCallback(const std_msgs::Bool::ConstPtr& msg) {
    entry_permission_ = msg->data;
    entry_permission_received_ = ros::Time::now();
  }

  void entryCorridorSelectionCallback(
      const std_msgs::String::ConstPtr& msg) {
    if (entry_corridor_locked_ || msg->data.empty()) return;
    const auto selected = std::find_if(
        entry_corridor_candidates_.begin(), entry_corridor_candidates_.end(),
        [&msg](const EntryCorridorCandidate& candidate) {
          return candidate.id == msg->data;
        });
    if (selected == entry_corridor_candidates_.end()) {
      ROS_WARN_THROTTLE(
          1.0,
          "[STAGE5_ENTRY] UAV%d ignored stale/unknown joint selection=%s",
          uav_id_, msg->data.c_str());
      return;
    }
    selected_entry_corridor_ = *selected;
    entry_corridor_locked_ = true;
    provisional_entry_gate_ = selected->entry_gate;
    provisional_entry_gate_index_ = 0;
    entry_gate_index_ = 0;
    entry_gate_locked_ = true;
    entry_gate_candidates_.assign(1U, selected->entry_gate);
    layer_entry_gates_[0] = selected->entry_gate;
    layer_entry_gate_valid_[0] = true;

    entry_angle_deg_ = selected->angle_deg;
    entry_angle_rad_ = entry_angle_deg_ * kPi / 180.0;
    pre_entry_radius_ = selected->pre_radius;
    ROS_WARN(
        "[STAGE5_ENTRY] UAV%d joint selection latched id=%s angle=%.1fdeg "
        "PRE_R=%.2f ENTRY_R=%.2f STAGING=%s STAGING_R=%.2f "
        "min_clearance=%.3f",
        uav_id_, selected->id.c_str(), selected->angle_deg,
        selected->pre_radius, selected->entry_radius,
        selected->orbit_candidate_id.c_str(),
        std::hypot(selected->orbit_staging.x - route_.center_x,
                   selected->orbit_staging.y - route_.center_y),
        selected->minimum_clearance);
  }

  void exitPermissionCallback(const std_msgs::Bool::ConstPtr& msg) {
    exit_permission_ = msg->data;
    exit_permission_received_ = ros::Time::now();
  }

  void orbitPermissionCallback(const std_msgs::Bool::ConstPtr& msg) {
    orbit_permission_ = msg->data;
    orbit_permission_received_ = ros::Time::now();
  }

  void orbitStagingPermissionCallback(
      const std_msgs::Bool::ConstPtr& msg) {
    orbit_staging_permission_ = msg->data;
    orbit_staging_permission_received_ = ros::Time::now();
  }

  void landingPermissionCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data && !landing_permission_) {
      ROS_WARN("[STAGE3_DIAG] uav=%d landing permission granted "
               "mission_state=%s sector=%d target=%s action=REQUEST_SAFE_LAND",
               uav_id_, missionStateName(state_), activeSectorId(),
               active_target_.id.empty() ? "none" : active_target_.id.c_str());
    }
    landing_permission_ = msg->data;
    landing_permission_received_ = ros::Time::now();
  }

  void commandCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    if (msg->header.frame_id != planning_frame_ || msg->header.stamp.isZero())
      return;
    if (!std::isfinite(msg->position.x) || !std::isfinite(msg->position.y) ||
        !std::isfinite(msg->position.z)) return;
    command_ = *msg; have_command_ = true; command_received_ = ros::Time::now();
    if (awaiting_fresh_trajectory_ &&
        msg->trajectory_id != trajectory_baseline_) {
      awaiting_fresh_trajectory_ = false;
      ROS_WARN("[STAGE3_DIAG] uav=%d EGO generated fresh trajectory "
               "trajectory_id=%u baseline=%u mission_state=%s sector=%d "
               "target=%s ego_replan_result=SUCCESS trajectory_expired=false "
               "action=TRACK_FRESH_BSPLINE",
               uav_id_, msg->trajectory_id, trajectory_baseline_,
               missionStateName(state_), activeSectorId(),
               active_target_.id.empty() ? "none" : active_target_.id.c_str());
    }
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
      if (msg->data == "HOME_HOVER") {
        ROS_WARN("[STAGE3_DIAG] uav=%d home hover reached "
                 "mission_state=%s sector=%d target=(%.2f,%.2f,%.2f) "
                 "action=WAIT_LANDING_PERMISSION",
                 uav_id_, missionStateName(state_), activeSectorId(),
                 active_target_.x, active_target_.y, active_target_.z);
      }
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

  std::string mappedEndpointBlockage(const CandidatePoint& target,
                                     double clearance) const {
    if (target.z < route_.minimum_height ||
        target.z > route_.maximum_height ||
        !std::isfinite(clearance) || clearance < 0.0) {
      return "TARGET_OR_CLEARANCE_INVALID";
    }
    double nearest_distance = std::numeric_limits<double>::infinity();
    geometry_msgs::Point nearest_point;
    bool blocked = false;
    for (const auto& occupied : planningMapPoints()) {
      const double dx = target.x - occupied.x;
      const double dy = target.y - occupied.y;
      const double dz = target.z - occupied.z;
      const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_point = occupied;
      }
      blocked = blocked || distance < clearance;
    }
    if (!blocked) return std::string();
    std::ostringstream reason;
    reason << "MAP_ENDPOINT_OCCUPIED target=" << target.id << " xyz=("
           << target.x << ',' << target.y << ',' << target.z
           << ") nearest_map_point=(" << nearest_point.x << ','
           << nearest_point.y << ',' << nearest_point.z << ") distance="
           << nearest_distance << " required_clearance=" << clearance;
    return reason.str();
  }

  bool mappedEndpointClear(const CandidatePoint& target) const {
    return mappedEndpointClear(
        target,
        filter_config_.map_points_are_inflated
            ? filter_config_.map_additional_clearance
            : filter_config_.minimum_clearance +
                  filter_config_.cloud_inflation);
  }

  double mappedTaskClearance() const {
    return filter_config_.map_points_are_inflated
               ? filter_config_.map_additional_clearance
               : filter_config_.minimum_clearance +
                     filter_config_.cloud_inflation;
  }

  bool mappedCorridorSafe(const geometry_msgs::Point& from,
                          const geometry_msgs::Point& to) const {
    static const std::vector<StaticObstacle> no_static_obstacles;
    const double map_clearance =
        filter_config_.map_points_are_inflated
            ? filter_config_.map_additional_clearance
            : filter_config_.minimum_clearance +
                  filter_config_.cloud_inflation;
    return lineCorridorSafe(
        from, to, planningMapPoints(), no_static_obstacles,
        map_clearance, filter_config_.corridor_sample_step);
  }

  double mappedPolylineClearance(
      const std::vector<geometry_msgs::Point>& points) const {
    double minimum = std::numeric_limits<double>::infinity();
    const auto& occupied = planningMapPoints();
    for (const auto& map_point : occupied) {
      for (std::size_t index = 1U; index < points.size(); ++index) {
        minimum = std::min(
            minimum,
            pointSegmentDistance(map_point, points[index - 1U],
                                 points[index]));
      }
    }
    return minimum;
  }

  geometry_msgs::Point candidatePoint(const CandidatePoint& candidate) const {
    geometry_msgs::Point point;
    point.x = candidate.x;
    point.y = candidate.y;
    point.z = candidate.z;
    return point;
  }

  bool entryCorridorCandidateSafe(
      EntryCorridorCandidate* candidate,
      const geometry_msgs::Point& start,
      std::string* failure_reason = nullptr) const {
    const auto reject = [failure_reason](const std::string& reason) {
      if (failure_reason != nullptr) *failure_reason = reason;
      return false;
    };
    if (candidate == nullptr) return false;
    // The subscribed occupancy topic is normally EGO's already-inflated map.
    // Do not charge the raw-cloud minimum a second time on that representation.
    const double map_clearance = mappedTaskClearance();
    if (!mappedEndpointClear(candidate->pre_entry, map_clearance))
      return reject("PRE_ENTRY_MAP_CLEARANCE");
    if (!mappedEndpointClear(candidate->entry_gate, map_clearance))
      return reject("ENTRY_GATE_MAP_CLEARANCE");
    if (!mappedEndpointClear(candidate->orbit_staging, map_clearance))
      return reject("ORBIT_STAGING_MAP_CLEARANCE");
    if (!staticEndpointRisk(candidate->pre_entry).empty())
      return reject("PRE_ENTRY_STATIC_CLEARANCE");
    if (!staticEndpointRisk(candidate->entry_gate).empty())
      return reject("ENTRY_GATE_STATIC_CLEARANCE");
    if (!staticEndpointRisk(candidate->orbit_staging).empty())
      return reject("ORBIT_STAGING_STATIC_CLEARANCE");
    const std::vector<geometry_msgs::Point> anchors{
        start, candidatePoint(candidate->pre_entry),
        candidatePoint(candidate->entry_gate),
        candidatePoint(candidate->orbit_staging)};
    const auto& map_points = planningMapPoints();
    candidate->planned_path.clear();
    candidate->planned_path.push_back(start);
    for (std::size_t index = 1U; index < anchors.size(); ++index) {
      std::vector<geometry_msgs::Point> segment;
      if (lineCorridorSafe(anchors[index - 1U], anchors[index], map_points,
                           {}, map_clearance,
                           filter_config_.corridor_sample_step)) {
        segment = {anchors[index - 1U], anchors[index]};
      } else {
        LevelPathConfig path_config = level_path_config_;
        path_config.altitude = anchors[index].z;
        path_config.additional_clearance = map_clearance;
        const LevelPathResult planned = planLevelPath(
            anchors[index - 1U], anchors[index], map_points, path_config);
        if (!planned.reachable || planned.points.size() < 2U) {
          return reject("LEG_" + std::to_string(index) + "_" +
                        planned.reason);
        }
        segment = planned.points;
      }
      for (std::size_t segment_index = 1U;
           segment_index < segment.size(); ++segment_index) {
        if (!lineCorridorSafe(segment[segment_index - 1U],
                              segment[segment_index], map_points, {},
                              map_clearance,
                              filter_config_.corridor_sample_step)) {
          return reject("MAP_CORRIDOR_CLEARANCE");
        }
        candidate->planned_path.push_back(segment[segment_index]);
      }
    }
    candidate->minimum_clearance =
        mappedPolylineClearance(candidate->planned_path);
    if (!std::isfinite(candidate->minimum_clearance) ||
        candidate->minimum_clearance + 1.0e-9 < map_clearance) {
      return reject("FULL_PATH_MAP_CLEARANCE");
    }
    if (failure_reason != nullptr) failure_reason->clear();
    return true;
  }

  void publishEntryCorridorCandidates(const ros::Time& now) {
    astra_custom_msgs::InspectionCandidateArray message;
    message.header.stamp = now;
    message.header.frame_id = planning_frame_;
    message.current_sector = static_cast<uint32_t>(entry_corridor_generation_);
    message.locked_candidate_id =
        entry_corridor_locked_ ? selected_entry_corridor_.id : std::string();
    const auto append = [&message](const EntryCorridorCandidate& corridor,
                                   const CandidatePoint& point,
                                   const std::string& suffix) {
      astra_custom_msgs::InspectionCandidate item;
      item.header = message.header;
      item.candidate_id = corridor.id + "/" + suffix;
      item.sector_id = 0U;
      item.layer_id = 0U;
      item.target.x = point.x;
      item.target.y = point.y;
      item.target.z = point.z;
      item.yaw = static_cast<float>(point.yaw);
      item.accepted = true;
      item.rejection_reason = "SAFE_FULL_CORRIDOR_PENDING_JOINT_SELECTION";
      item.clearance = static_cast<float>(corridor.minimum_clearance);
      item.score = static_cast<float>(corridor.score);
      item.unknown_ratio = 0.0F;
      message.candidates.push_back(item);
    };
    for (const auto& corridor : entry_corridor_candidates_) {
      append(corridor, corridor.pre_entry, "PRE_ENTRY");
      append(corridor, corridor.entry_gate, "ENTRY_GATE");
      append(corridor, corridor.orbit_staging, "ORBIT_STAGING");
      for (std::size_t index = 0U; index < corridor.planned_path.size();
           ++index) {
        CandidatePoint path_point;
        std::ostringstream suffix;
        suffix << "PATH_" << index;
        path_point.x = corridor.planned_path[index].x;
        path_point.y = corridor.planned_path[index].y;
        path_point.z = corridor.planned_path[index].z;
        append(corridor, path_point, suffix.str());
      }
    }
    entry_corridor_candidates_pub_.publish(message);
  }

  bool buildEntryCorridorCandidates(const ros::Time& now) {
    if (!mapFresh(now) || !have_odom_) return false;
    ++entry_corridor_generation_;
    entry_corridor_candidates_.clear();
    entry_corridor_locked_ = false;
    entry_gate_locked_ = false;
    const geometry_msgs::Point start = pointOf(odom_);
    std::vector<EntryCorridorCandidate> safe;
    std::unordered_map<std::string, int> rejected;
    const double map_clearance = mappedTaskClearance();

    // The first orbit point is not a special fixed-radius target.  Evaluate
    // the exact same candidate grid used by every numbered sector, then use
    // those accepted candidates as ORBIT_STAGING choices in the joint ENTRY
    // corridors.  This keeps candidate IDs valid for the normal EGO retry and
    // PLANNER_UNREACHABLE bookkeeping after the joint selection is latched.
    if (sectors_.empty()) return false;
    Sector& first_sector = sectors_.front();
    std::vector<CandidatePoint> orbit_candidates;
    orbit_candidates.reserve(first_sector.candidates.size());
    for (auto candidate : first_sector.candidates) {
      geometry_msgs::Point target = candidatePoint(candidate);
      evaluateCandidate(&candidate, first_sector, start, planningMapPoints(),
                        obstacles_, true, filter_config_, nullptr,
                        candidateUnknownRatio(target));
      if (planner_unreachable_candidates_.count(candidate.id) > 0U) {
        candidate.accepted = false;
        candidate.planner_unreachable = true;
        candidate.target_invalid = false;
        candidate.rejection_reason = "PLANNER_UNREACHABLE";
      }
      if (candidate.accepted &&
          !firstWaypointTowerCorridorSafe(start, candidate)) {
        candidate.accepted = false;
        candidate.target_invalid = true;
        candidate.rejection_reason = "FIRST_LEG_TOWER_KEEP_OUT";
      }
      if (candidate.accepted) orbit_candidates.push_back(candidate);
    }
    if (orbit_candidates.empty()) {
      ROS_WARN("[STAGE5_ENTRY] UAV%d no accepted ordinary-sector orbit "
               "candidate after unified endpoint evaluation", uav_id_);
      return false;
    }

    const int angle_steps = static_cast<int>(std::lround(
        entry_angular_search_half_width_deg_ /
        entry_gate_config_.angular_sample_step_deg));
    const int pre_steps = static_cast<int>(std::lround(
        (pre_entry_maximum_radius_ - pre_entry_radius_) /
        pre_entry_radial_sample_step_));
    const int entry_steps = static_cast<int>(std::lround(
        (entry_gate_config_.maximum_radius -
         entry_gate_config_.minimum_radius) /
        entry_gate_config_.radial_sample_step));
    std::size_t searched = 0U;
    for (int angle_step = -angle_steps; angle_step <= angle_steps;
         ++angle_step) {
      const double angle_deg = positiveAngleDegrees(
          (entry_nominal_angle_deg_ +
           entry_gate_config_.angular_sample_step_deg * angle_step) *
          kPi / 180.0);
      const double angle_rad = angle_deg * kPi / 180.0;
      for (int pre_step = 0; pre_step <= pre_steps; ++pre_step) {
        const double pre_radius =
            pre_entry_radius_ + pre_entry_radial_sample_step_ * pre_step;
        for (int entry_step = 0; entry_step <= entry_steps; ++entry_step) {
          const double entry_radius = entry_gate_config_.minimum_radius +
                                      entry_gate_config_.radial_sample_step *
                                          entry_step;
          for (const auto& orbit_base : orbit_candidates) {
            ++searched;
            EntryCorridorCandidate candidate;
            std::ostringstream id;
            id << "U" << uav_id_ << "_G" << entry_corridor_generation_
               << "_A" << angle_step << "_P" << pre_step
               << "_E" << entry_step << "_O" << orbit_base.id;
            candidate.id = id.str();
            candidate.orbit_candidate_id = orbit_base.id;
            candidate.angle_deg = angle_deg;
            candidate.pre_radius = pre_radius;
            candidate.entry_radius = entry_radius;
            const auto make_radial = [&](const std::string& name,
                                         double radius) {
              CandidatePoint point;
              point.id = name;
              point.layer_id = current_layer_;
              point.x = route_.center_x + radius * std::cos(angle_rad);
              point.y = route_.center_y + radius * std::sin(angle_rad);
              point.z = inspection_heights_.front();
              point.yaw = normalizeAngle(
                  std::atan2(route_.center_y - point.y,
                             route_.center_x - point.x) -
                  route_.camera_yaw_offset_rad);
              point.require_arrival_yaw = false;
              point.accepted = true;
              return point;
            };
            candidate.pre_entry = make_radial(candidate.id + "_PRE_ENTRY",
                                              pre_radius);
            candidate.entry_gate = make_radial(candidate.id + "_ENTRY_GATE",
                                               entry_radius);
            candidate.orbit_staging = orbit_base;
            candidate.orbit_staging.id = candidate.id + "_ORBIT_STAGING";
            std::string failure_reason;
            if (!entryCorridorCandidateSafe(&candidate, start,
                                            &failure_reason)) {
              ++rejected[failure_reason.empty() ? "UNSPECIFIED"
                                                : failure_reason];
              continue;
            }
            if (!std::isfinite(candidate.minimum_clearance) ||
                candidate.minimum_clearance + 1.0e-9 < map_clearance) {
              ++rejected["FULL_PATH_MAP_CLEARANCE"];
              continue;
            }
            const double staging_angle = std::atan2(
                orbit_base.y - route_.center_y,
                orbit_base.x - route_.center_x);
            const double angle_deviation = std::abs(normalizeAngle(
                staging_angle - entry_nominal_angle_deg_ * kPi / 180.0));
            const double radius_deviation = std::abs(
                std::hypot(orbit_base.x - route_.center_x,
                           orbit_base.y - route_.center_y) - route_.radius);
            const double ingress_deviation =
                std::abs(angle_step) * entry_gate_config_.angular_sample_step_deg +
                (pre_radius - pre_entry_radius_) +
                (entry_radius - entry_gate_config_.minimum_radius);
            // Hard checks have already passed.  The score follows the common
            // sector priorities: clearance first, then nominal angle/radius,
            // shorter ingress and continuity-friendly ordinary candidate cost.
            candidate.score =
                100.0 * candidate.minimum_clearance -
                4.0 * route_.radius * angle_deviation -
                6.0 * radius_deviation - 0.2 * ingress_deviation +
                0.05 * orbit_base.score;
            safe.push_back(candidate);
          }
        }
      }
    }
    std::sort(safe.begin(), safe.end(),
              [this](const EntryCorridorCandidate& left,
                     const EntryCorridorCandidate& right) {
                if (std::abs(left.minimum_clearance -
                             right.minimum_clearance) > 1.0e-6) {
                  return left.minimum_clearance > right.minimum_clearance;
                }
                const double left_angle = std::abs(normalizeAngle(
                    left.angle_deg * kPi / 180.0 -
                    entry_nominal_angle_deg_ * kPi / 180.0));
                const double right_angle = std::abs(normalizeAngle(
                    right.angle_deg * kPi / 180.0 -
                    entry_nominal_angle_deg_ * kPi / 180.0));
                if (std::abs(left_angle - right_angle) > 1.0e-9) {
                  return left_angle < right_angle;
                }
                return (left.pre_radius - pre_entry_radius_) +
                           (left.entry_radius -
                            entry_gate_config_.minimum_radius) <
                       (right.pre_radius - pre_entry_radius_) +
                           (right.entry_radius -
                            entry_gate_config_.minimum_radius);
              });
    std::unordered_map<int, int> retained_per_angle;
    for (const auto& candidate : safe) {
      const double staging_angle = std::atan2(
          candidate.orbit_staging.y - route_.center_y,
          candidate.orbit_staging.x - route_.center_x);
      const int angle_key = static_cast<int>(std::lround(
          normalizeAngle(staging_angle -
                         entry_nominal_angle_deg_ * kPi / 180.0) *
          180.0 / kPi / entry_gate_config_.angular_sample_step_deg));
      if (retained_per_angle[angle_key] >= 3) continue;
      entry_corridor_candidates_.push_back(candidate);
      ++retained_per_angle[angle_key];
    }
    publishEntryCorridorCandidates(now);
    std::ostringstream rejected_summary;
    bool first_rejection = true;
    for (const auto& item : rejected) {
      if (!first_rejection) rejected_summary << ',';
      rejected_summary << item.first << ':' << item.second;
      first_rejection = false;
    }
    ROS_WARN(
        "[STAGE5_ENTRY] UAV%d generation=%u searched=%zu safe=%zu "
        "published=%zu nominal_angle=%.1fdeg map_clearance=%.2f "
        "rejected=%s",
        uav_id_, entry_corridor_generation_, searched, safe.size(),
        entry_corridor_candidates_.size(), entry_nominal_angle_deg_,
        map_clearance, rejected_summary.str().c_str());
    return !entry_corridor_candidates_.empty();
  }

  bool selectedEntryCorridorStillSafe(const ros::Time& now) const {
    if (!entry_corridor_locked_ || !mapFresh(now) || !have_odom_) {
      return false;
    }
    EntryCorridorCandidate rechecked = selected_entry_corridor_;
    return entryCorridorCandidateSafe(&rechecked, pointOf(odom_));
  }

  void invalidateEntryCorridor(const ros::Time& now,
                               const std::string& reason) {
    ROS_ERROR("[STAGE5_ENTRY] UAV%d selected corridor invalidated before "
              "entry id=%s reason=%s; generating fresh candidates",
              uav_id_, selected_entry_corridor_.id.c_str(), reason.c_str());
    entry_corridor_locked_ = false;
    entry_gate_locked_ = false;
    selected_entry_corridor_ = EntryCorridorCandidate();
    entry_corridor_candidates_.clear();
    buildEntryCorridorCandidates(now);
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
    const double entry_angle = entry_angle_rad_;
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
    // Stage 5 uses three fixed, independent gates.  Keep only the preferred
    // radius/center candidate, then place it at the role-derived continuous
    // angle (UAV1/UAV3 are intentionally between legacy sector centers).
    entry_gate_candidates_.erase(
        std::remove_if(
            entry_gate_candidates_.begin(), entry_gate_candidates_.end(),
            [](const CandidatePoint& candidate) {
              return candidate.priority != 0;
            }),
        entry_gate_candidates_.end());
    for (auto& candidate : entry_gate_candidates_) {
      candidate.id = "ENTRY_GATE_UAV" + std::to_string(uav_id_) + "_A" +
                     std::to_string(entry_angle_deg_);
      candidate.x = route_.center_x +
                    layer_config.preferred_radius * std::cos(entry_angle_rad_);
      candidate.y = route_.center_y +
                    layer_config.preferred_radius * std::sin(entry_angle_rad_);
      candidate.yaw = normalizeAngle(
          std::atan2(route_.center_y - candidate.y,
                     route_.center_x - candidate.x) -
          route_.camera_yaw_offset_rad);
    }
    for (auto& candidate : entry_gate_candidates_) {
      candidate.layer_id = layer_index;
      const double candidate_angle = normalizeAngle(std::atan2(
          candidate.y - route_.center_y,
          candidate.x - route_.center_x));
      if (std::abs(normalizeAngle(candidate_angle - entry_angle_rad_)) >
          1.0e-6) {
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
              entry_angle_rad_)) *
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
      // The formal corridor has exactly one role-specific PRE_ENTRY anchor and
      // one independent ENTRY_GATE.  EGO still owns all local avoidance and
      // replanning between these fresh global references; neither point is a
      // replayed B-spline sample.
      CandidatePoint pre_entry = entry_corridor_locked_
                                     ? selected_entry_corridor_.pre_entry
                                     : provisional_entry_gate_;
      pre_entry.id = "PRE_ENTRY_UAV" + std::to_string(uav_id_) + "_" +
                     (entry_corridor_locked_
                          ? selected_entry_corridor_.id
                          : std::string("LEGACY"));
      if (!entry_corridor_locked_) {
        pre_entry.x = route_.center_x +
                      pre_entry_radius_ * std::cos(entry_angle_rad_);
        pre_entry.y = route_.center_y +
                      pre_entry_radius_ * std::sin(entry_angle_rad_);
      }
      pre_entry.require_arrival_yaw = false;
      entry_gate_transit_goals_ = {pre_entry, provisional_entry_gate_};
      entry_gate_transit_goals_.back().require_arrival_yaw = false;
      horizontal_path_available_ = false;
      vertical_escape_allowed_ = true;
      low_no_path_confirmations_ = 0;
      last_low_no_path_check_ = ros::Time(0);
      publishAltitudePolicy("DIRECT_FORMAL_ENTRY_GATE_EGO_LOCAL_AVOIDANCE");

      entry_gate_transit_index_ = 0U;
      ascent_goal_attempts_ = 0;
      have_sent_goal_ = false;
      arrival_since_ = ros::Time(0);
      coverage_wait_started_ = ros::Time(0);
      entry_gate_relocation_pending_ = false;
      entry_gate_no_safe_candidate_hold_ = false;
      low_ingress_replan_pending_ = false;
      successful_ingress_trace_.clear();
      recordIngressTrace(current);
      transition(MissionState::kEntryGateTransit,
                 "entry hover complete; independent PRE_ENTRY corridor handed "
                 "to EGO");
      ROS_WARN("[STAGE3_TASK] LOW_ALTITUDE corridor UAV%d PRE_ENTRY="
               "(%.2f, %.2f, %.2f) ENTRY_GATE=(%.2f, %.2f, %.2f) "
               "angle=%.1fdeg selected=%s clearance=%.3f; EGO owns local "
               "avoidance and replanning",
               uav_id_, pre_entry.x, pre_entry.y, pre_entry.z,
               provisional_entry_gate_.x, provisional_entry_gate_.y,
               provisional_entry_gate_.z, entry_angle_deg_,
               entry_corridor_locked_ ? selected_entry_corridor_.id.c_str()
                                      : "LEGACY",
               entry_corridor_locked_
                   ? selected_entry_corridor_.minimum_clearance
                   : -1.0);
      return true;
    }
    if (prefer_safe_overflight_ && mappedCorridorSafe(current, gate)) {
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
                   ? "entry hover complete; live-map ENTRY_GATE transit prepared"
                   : "safe altitude reached; high-corridor ENTRY_GATE transit "
                     "prepared");
    return true;
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
    orbit_tracking_initialized_ = false;
    orbit_released_latched_ = false;
    orbit_staging_arrived_ = false;
    reverse_orbit_detected_ = false;
    accumulated_orbit_angle_ = 0.0;
    visited_sector_mask_ = 0U;
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
        route_.minimum_safety_distance,
        filter_config_.corridor_sample_step);
  }

  void publishFixedMissionRoute(const ros::Time& now) {
    if (!low_altitude_mode_ || !have_home_position_ ||
        !entry_gate_locked_ || lap_visit_sequence_.empty()) {
      return;
    }
    nav_msgs::Path route;
    route.header.stamp = now;
    route.header.frame_id = planning_frame_;
    const auto append_point = [&route](double x, double y, double z) {
      geometry_msgs::PoseStamped pose;
      pose.header = route.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = z;
      pose.pose.orientation.w = 1.0;
      route.poses.push_back(pose);
    };

    // The leading home-hover pose visualizes the ingress leg; it is not
    // counted as an EGO goal until the final HOME_HOVER publication.
    append_point(home_position_.x, home_position_.y, home_position_.z);
    append_point(provisional_entry_gate_.x, provisional_entry_gate_.y,
                 provisional_entry_gate_.z);
    for (const double layer_height : inspection_heights_) {
      for (const std::size_t sector_index : lap_visit_sequence_) {
        if (sector_index >= sectors_.size()) continue;
        const Sector& sector = sectors_[sector_index];
        append_point(
            route_.center_x +
                sector.nominal_radius * std::cos(sector.nominal_angle_rad),
            route_.center_y +
                sector.nominal_radius * std::sin(sector.nominal_angle_rad),
            layer_height);
      }
    }
    append_point(provisional_entry_gate_.x, provisional_entry_gate_.y,
                 inspection_heights_.back());
    append_point(home_position_.x, home_position_.y, home_position_.z);
    mission_route_pub_.publish(route);
    ROS_WARN("[STAGE3_GOAL] fixed mission route published: "
             "HOME_HOVER(reference)->ENTRY_GATE->closed layer routes->"
             "EXIT_GATE->HOME_HOVER; layers=%zu poses=%zu",
             inspection_heights_.size(), route.poses.size());
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
    std::vector<int> best_candidate_indices(sectors_.size(), -1);
    for (std::size_t order_index = 0; order_index < sectors_.size();
         ++order_index) {
      Sector& sector = sectors_[order_index];
      sector.state = SectorState::kEvaluating;
      sector.locked_index = -1;
      for (auto& candidate : sector.candidates) {
        geometry_msgs::Point candidate_point = candidatePoint(candidate);
        evaluateCandidate(&candidate, sector, current, planningMapPoints(),
                          obstacles_, mapFresh(now), filter_config_, nullptr,
                          candidateUnknownRatio(candidate_point));
        if (planner_unreachable_candidates_.count(candidate.id) > 0U) {
          candidate.accepted = false;
          candidate.planner_unreachable = true;
          candidate.target_invalid = false;
          candidate.rejection_reason = "PLANNER_UNREACHABLE";
        }
        if (candidate.accepted &&
            !firstWaypointTowerCorridorSafe(current, candidate)) {
          candidate.accepted = false;
          candidate.target_invalid = true;
          candidate.rejection_reason = "FIRST_LEG_TOWER_KEEP_OUT";
        }
      }
      const int candidate_index = chooseBestCandidate(
          sector, nullptr, 0.0,
          filter_config_.prefer_clear_straight_corridor);
      best_candidate_indices[order_index] = candidate_index;
      const int selected_index = candidate_index;
      const bool skipped = sector.sector_id == skipped_sector_id;
      // skip_current_sector is retained for call-site compatibility, but an
      // unreachable first target must now try the next candidate in the same
      // sector before advancing to another sector.
      const bool safe = selected_index >= 0;
      const CandidatePoint* candidate =
          selected_index >= 0 ? &sector.candidates[selected_index] : nullptr;
      const double candidate_distance = candidate == nullptr
                                            ? std::numeric_limits<double>::infinity()
                                            : std::hypot(candidate->x - entry_reference.x,
                                                         candidate->y - entry_reference.y);
      const double candidate_angle = candidate == nullptr
                                         ? sector.nominal_angle_rad
                                         : std::atan2(candidate->y - route_.center_y,
                                                      candidate->x - route_.center_x);
      ROS_WARN("[STAGE3_TASK] FIRST_WAYPOINT candidate waypoint=%d "
               "internal_index=%d candidate_angle=%.3fdeg "
               "distance=%.3fm clearance=%.3fm selected=%s "
               "source=%s target=%s",
               sector.sector_id + 1, sector.sector_id,
               positiveAngleDegrees(candidate_angle), candidate_distance,
               candidate == nullptr ? -1.0 : candidate->clearance,
               safe ? "true" : "false",
               skipped ? "SAME_SECTOR_RETRY" : "DIRECTIONAL_SECTOR",
               candidate == nullptr ? "none" : candidate->id.c_str());
    }

    if (entry_corridor_locked_ &&
        !selected_entry_corridor_.orbit_candidate_id.empty()) {
      const auto locked = findAcceptedCandidateById(
          sectors_, selected_entry_corridor_.orbit_candidate_id);
      selected_order_index = locked.first;
      selected_candidate_index = locked.second;
      if (selected_order_index < 0) {
        ROS_ERROR("[STAGE5_ENTRY] UAV%d jointly locked orbit candidate %s "
                  "is no longer accepted; refusing a mismatched sector",
                  uav_id_,
                  selected_entry_corridor_.orbit_candidate_id.c_str());
      }
    } else {
      for (std::size_t order_index = 0;
           order_index < best_candidate_indices.size(); ++order_index) {
        if (best_candidate_indices[order_index] < 0) continue;
        selected_order_index = static_cast<int>(order_index);
        selected_candidate_index = best_candidate_indices[order_index];
        break;
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
    // The joint Stage-5 selector may deliberately choose a temporary
    // 14.5/16.5 m staging radius to keep the three ingress trajectories
    // separated.  Preserve that selected staging point for the first
    // waypoint; subsequent sector targets remain the nominal orbit radius,
    // allowing a smooth return to the 12.5 m inspection circle after release.
    if (entry_corridor_locked_ &&
        !selected_entry_corridor_.orbit_candidate_id.empty()) {
      const CandidatePoint& staging = selected_entry_corridor_.orbit_staging;
      const double staging_radius = std::hypot(
          staging.x - route_.center_x, staging.y - route_.center_y);
      const double nominal_radius = std::hypot(
          active_target_.x - route_.center_x,
          active_target_.y - route_.center_y);
      const double staging_delta = std::sqrt(
          (staging.x - active_target_.x) *
              (staging.x - active_target_.x) +
          (staging.y - active_target_.y) *
              (staging.y - active_target_.y) +
          (staging.z - active_target_.z) *
              (staging.z - active_target_.z));
      if (std::isfinite(staging_radius) &&
          std::isfinite(nominal_radius) &&
          std::isfinite(staging_delta) && staging_delta > 1.0e-6) {
        active_target_.x = staging.x;
        active_target_.y = staging.y;
        active_target_.z = staging.z;
        active_target_.yaw = staging.yaw;
        ROS_WARN(
            "[STAGE5_ENTRY] UAV%d applying jointly selected staging "
            "angle=%.1fdeg radius=%.2fm (nominal=%.2fm) for first waypoint; "
            "later sectors return to nominal orbit radius",
            uav_id_, positiveAngleDegrees(std::atan2(
                         staging.y - route_.center_y,
                         staging.x - route_.center_x)),
            staging_radius, nominal_radius);
      }
    }
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
    publishFixedMissionRoute(now);
    transition(MissionState::kTargetLocked,
               "ENTRY_GATE directional first inspection waypoint locked");
    return !lap_visit_sequence_.empty();
  }

  bool evaluateSector(const ros::Time& now) {
    if (current_sector_ >= sectors_.size()) return false;
    sector_detour_active_ = false;
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
    const int selected = chooseBestCandidate(
        sector, locked, target_replacement_margin_,
        filter_config_.prefer_clear_straight_corridor);
    if (selected < 0) {
      std::unordered_map<std::string, int> rejection_counts;
      for (const auto& candidate : sector.candidates) {
        const std::string reason = candidate.rejection_reason.empty()
                                       ? "UNSPECIFIED"
                                       : candidate.rejection_reason;
        ++rejection_counts[reason];
        const double angle = positiveAngleDegrees(std::atan2(
            candidate.y - route_.center_y,
            candidate.x - route_.center_x));
        const double radius = std::hypot(candidate.x - route_.center_x,
                                         candidate.y - route_.center_y);
        ROS_ERROR("[STAGE3_CANDIDATE_EXHAUSTED] uav=%d sector=%d id=%s "
                  "xyz=(%.6f,%.6f,%.6f) angle=%.6fdeg radius=%.6fm "
                  "accepted=%s planner_unreachable=%s clearance=%.6fm "
                  "reason=%s",
                  uav_id_, sector.sector_id + 1, candidate.id.c_str(),
                  candidate.x, candidate.y, candidate.z, angle, radius,
                  candidate.accepted ? "true" : "false",
                  candidate.planner_unreachable ? "true" : "false",
                  candidate.clearance, reason.c_str());
      }
      std::ostringstream breakdown;
      for (const auto& item : rejection_counts) {
        if (!breakdown.str().empty()) breakdown << '|';
        breakdown << item.first << ':' << item.second;
      }
      ROS_ERROR("[STAGE3_CANDIDATE_EXHAUSTED] uav=%d sector=%d total=%zu "
                "breakdown=%s map_clearance=%.3fm raw_or_coarse_clearance=%.3fm",
                uav_id_, sector.sector_id + 1, sector.candidates.size(),
                breakdown.str().c_str(), mappedTaskClearance(),
                filter_config_.minimum_clearance);
      CandidatePoint detour;
      CandidatePoint final_target;
      int final_index = -1;
      if (low_altitude_mode_ &&
          buildStaticSectorDetour(sector, &detour, &final_target,
                                  &final_index)) {
        sector.locked_index = final_index;
        sector.state = SectorState::kTargetLocked;
        pending_sector_target_ = final_target;
        active_target_ = detour;
        sector_detour_active_ = true;
        current_target_plan_attempt_ = 0;
        ROS_WARN("[STAGE3_TASK] uav=%d sector=%d known-obstacle "
                 "corridor detour selected: intermediate=%s"
                 "(%.2f,%.2f,%.2f) final=%s(%.2f,%.2f,%.2f); "
                 "both static segments clear; EGO will generate fresh "
                 "local trajectories for each segment",
                 uav_id_, sector.sector_id + 1, detour.id.c_str(),
                 detour.x, detour.y, detour.z, final_target.id.c_str(),
                 final_target.x, final_target.y, final_target.z);
        transition(MissionState::kTargetLocked,
                   "same-sector known-obstacle corridor detour locked");
        return true;
      }
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

  bool buildStaticSectorDetour(const Sector& sector,
                               CandidatePoint* detour,
                               CandidatePoint* final_target,
                               int* final_index) const {
    if (!have_odom_ || detour == nullptr || final_target == nullptr ||
        final_index == nullptr) {
      return false;
    }
    const geometry_msgs::Point current = pointOf(odom_);
    const double current_angle = std::atan2(
        current.y - route_.center_y, current.x - route_.center_x);
    const double direction_sign =
        route_.direction == OrbitDirection::kCounterClockwise ? 1.0 : -1.0;
    const double clearance = filter_config_.minimum_clearance +
                             filter_config_.cloud_inflation;
    static const std::vector<geometry_msgs::Point> no_map_points;
    // Keep static same-sector detours on the shared adaptive candidate grid;
    // inward radial offsets are not part of the design rule.
    const std::vector<double> radius_offsets{0.0, 2.0, 4.0};
    double best_score = std::numeric_limits<double>::infinity();

    for (std::size_t index = 0; index < sector.candidates.size(); ++index) {
      const CandidatePoint& candidate = sector.candidates[index];
      if (candidate.rejection_reason != "KNOWN_OBSTACLE_CORRIDOR" ||
          planner_unreachable_candidates_.count(candidate.id) > 0U) {
        continue;
      }
      geometry_msgs::Point final_point;
      final_point.x = candidate.x;
      final_point.y = candidate.y;
      final_point.z = candidate.z;
      double directional_delta = direction_sign * normalizeAngle(
          std::atan2(final_point.y - route_.center_y,
                     final_point.x - route_.center_x) - current_angle);
      if (directional_delta < 0.0) directional_delta += 2.0 * kPi;
      if (directional_delta <= 5.0 * kPi / 180.0 ||
          directional_delta > 90.0 * kPi / 180.0) {
        continue;
      }
      for (double step_deg = 5.0;
           step_deg * kPi / 180.0 < directional_delta - 1.0e-6;
           step_deg += 5.0) {
        const double theta = normalizeAngle(
            current_angle + direction_sign * step_deg * kPi / 180.0);
        for (double radius_offset : radius_offsets) {
          const double radius = route_.radius + radius_offset;
          if (radius < route_.radius - sector_radius_half_width_ ||
              radius > route_.radius + sector_radius_half_width_) {
            continue;
          }
          CandidatePoint intermediate;
          intermediate.id = "SECTOR_" +
                            std::to_string(sector.sector_id + 1) +
                            "_STATIC_DETOUR_" + candidate.id;
          intermediate.sector_id = sector.sector_id;
          intermediate.layer_id = candidate.layer_id;
          intermediate.x = route_.center_x + radius * std::cos(theta);
          intermediate.y = route_.center_y + radius * std::sin(theta);
          intermediate.z = candidate.z;
          intermediate.yaw = normalizeAngle(std::atan2(
              route_.center_y - intermediate.y,
              route_.center_x - intermediate.x));
          geometry_msgs::Point intermediate_point;
          intermediate_point.x = intermediate.x;
          intermediate_point.y = intermediate.y;
          intermediate_point.z = intermediate.z;
          bool endpoint_clear = true;
          for (const auto& obstacle : obstacles_) {
            if (pointInObstacle(intermediate_point, obstacle, clearance)) {
              endpoint_clear = false;
              break;
            }
          }
          if (!endpoint_clear || !mappedEndpointClear(intermediate)) continue;
          if (!lineCorridorSafe(current, intermediate_point, no_map_points,
                                obstacles_, clearance,
                                filter_config_.corridor_sample_step) ||
              !lineCorridorSafe(intermediate_point, final_point,
                                no_map_points, obstacles_, clearance,
                                filter_config_.corridor_sample_step)) {
            continue;
          }
          const double total_distance =
              std::hypot(intermediate.x - current.x,
                         intermediate.y - current.y) +
              std::hypot(candidate.x - intermediate.x,
                         candidate.y - intermediate.y);
          const double score = total_distance +
                               2.0 * std::abs(radius_offset) +
                               candidate.nominal_deviation;
          if (score < best_score) {
            best_score = score;
            *detour = intermediate;
            *final_target = candidate;
            *final_index = static_cast<int>(index);
          }
        }
      }
    }
    return *final_index >= 0;
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
    const ros::Time now = ros::Time::now();
    const double state_duration = state_entered_.isZero()
                                      ? 0.0
                                      : (now - state_entered_).toSec();
    const bool trajectory_expired =
        have_command_ && !fresh(now, command_received_, input_timeout_);
    ROS_ERROR("[STAGE3_DIAG] uav=%d HOLD requested mission_state=%s "
              "sector=%d target=%s(%.2f,%.2f,%.2f) condition=\"%s\" "
              "ego_replan_result=%s planner_reason=%s trajectory_expired=%s "
              "predicted_swarm_conflict=%s state_duration=%.3fs "
              "action=CANCEL_TRAJECTORY_AND_HOLD",
              uav_id_, missionStateName(state_), activeSectorId(),
              active_target_.id.empty() ? "none" : active_target_.id.c_str(),
              active_target_.x, active_target_.y, active_target_.z,
              reason.c_str(),
              have_planner_status_
                  ? (planner_status_.last_plan_success ? "SUCCESS" : "FAILED")
                  : "UNKNOWN",
              have_planner_status_
                  ? planner_status_.failure_reason.c_str()
                  : "NO_PLANNER_STATUS",
              trajectory_expired ? "true" : "false",
              phase_hold_pending_ ? "true" : "false", state_duration);
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
    // The formal ENTRY_GATE is published exactly once.  EGO owns replanning
    // while that goal is active; if the safety layer must cancel tracking, the
    // task returns home instead of publishing the same or a replacement goal.
    low_ingress_replan_pending_ = true;
    publishAltitudePolicy("HOLD_WITHOUT_MISSION_GOAL_REPUBLISH");
    requestHold("LOW_ALTITUDE ingress safety stop: " + reason);
  }

  bool lowIngressCommandViolatesAltitude(const ros::Time& now) const {
    const bool horizontal_low_altitude_phase =
        state_ == MissionState::kEntryGateTransit;
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
    // toward the configured entry layer is recovery to the preferred plane,
    // not an obstacle-driven vertical escape. Reject only a command that moves
    // farther away from the entry layer than the vehicle already is.
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
    bool resumed = false;
    if (!resume_client_.call(service) || !service.response.success) {
      ROS_WARN("[STAGE3_TASK] EGO resume pending: %s",
               service.response.message.c_str());
      resumed = requestTracking(true);
    } else {
      resumed = true;
    }
    if (resumed) {
      // Bridge state arrives asynchronously.  Ignore only the previously
      // latched HOLD during this bounded acknowledgement window; if the
      // bridge does not leave HOLD within one second, the normal gate below
      // cancels again and remains fail-closed.
      bridge_resume_grace_until_ =
          ros::Time::now() + ros::Duration(1.0);
    }
    return resumed;
  }

  bool plannerFailure(const ros::Time& now, std::string* reason) const {
    if (!have_planner_status_ || !fresh(now, planner_status_received_, input_timeout_)) {
      *reason = "planner status stale"; return true;
    }
    if (planner_status_.current_position_in_collision) {
      *reason = astra_custom_msgs::PlannerStatus::CURRENT_POSITION_IN_OCCUPANCY;
      return true;
    }
    const bool current_goal_trajectory =
        have_command_ && command_.trajectory_id != trajectory_baseline_;
    const bool current_goal_generation =
        planner_target_baseline_.empty()
            ? !planner_status_.target_id.empty()
            : planner_status_.target_id != planner_target_baseline_;
    // The migrated EGO-Swarm status target_id identifies the vehicle
    // ("drone_0", ...), not each goal generation.  Keep compatibility with
    // the legacy changing target_id while accepting the authoritative fresh
    // traj_server generation used by the bridge and cancel gate.
    const bool current_goal_acknowledged =
        current_goal_generation || current_goal_trajectory;
    if (!current_goal_acknowledged) {
      if (have_sent_goal_ && !goal_sent_.isZero() &&
          now - goal_sent_ > ros::Duration(planning_timeout_)) {
        *reason = "planner goal acknowledgement timeout";
        return true;
      }
      return false;
    }
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
      // Both coordination barriers deliberately disable trajectory tracking.
      // HOLD in these named wait states is the task-supervised position latch;
      // HOLD everywhere else remains fail-closed.
      const bool supervised_orbit_wait_hold =
          (state_ == MissionState::kWaitOrbitStagingPermission ||
           state_ == MissionState::kWaitOrbitPermission) &&
          bridge_state_ == "HOLD";
      if (!have_bridge_state_ ||
          (bridge_state_ != "TRACK_EGO" &&
           bridge_state_ != "HOVER_READY" &&
           !supervised_orbit_wait_hold)) {
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
    const std::vector<CandidatePoint>& ingress_reference =
        low_altitude_mode_ && successful_ingress_trace_.size() >= 2U
            ? successful_ingress_trace_
            : successful_ingress_goals_;
    if (ingress_reference.empty() ||
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
    const geometry_msgs::Point current = pointOf(odom_);
    for (auto iterator = ingress_reference.rbegin();
         iterator != ingress_reference.rend(); ++iterator) {
      const double current_separation = std::sqrt(
          (iterator->x - current.x) * (iterator->x - current.x) +
          (iterator->y - current.y) * (iterator->y - current.y) +
          (iterator->z - current.z) * (iterator->z - current.z));
      if (current_separation <
          return_egress_config_.minimum_goal_separation) {
        continue;
      }
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
               reason + "; reverse ingress goals selected");
    ROS_WARN("[STAGE3_DIAG] uav=%d return state entered "
             "reference=REVERSE_ACTUAL_INGRESS goals=%zu current=(%.2f,%.2f,%.2f) "
             "action=PUBLISH_FRESH_GOALS_TO_LOCAL_EGO",
             uav_id_, normal_return_goals_.size(), current.x, current.y,
             current.z);
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
    const bool straight_clear = mappedCorridorSafe(from, to);
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

  std::string formalMissionTargetId(const CandidatePoint& target,
                                    GoalKind kind) const {
    if (!low_altitude_mode_) return target.id;
    // PRE_ENTRY and ENTRY_GATE are separate immutable endpoints.  Reusing one
    // formal id makes the coordinate lock replace the second leg with the
    // first leg's coordinates, which can falsely report ENTRY_READY while the
    // vehicle is still at PRE_ENTRY.
    if (kind == GoalKind::kEntryGate) return target.id;
    if (kind == GoalKind::kSector && sector_limit_ > 0) {
      return "WP" + std::to_string(
          static_cast<int>(visit_cursor_ %
                           static_cast<std::size_t>(sector_limit_)) +
          1);
    }
    if (kind == GoalKind::kExitGate) return "EXIT_GATE";
    return target.id;
  }

  void lockFormalTargetCoordinates(const std::string& mission_target_id,
                                   CandidatePoint* target) {
    const bool entry_endpoint =
        mission_target_id == "ENTRY_GATE" ||
        mission_target_id.rfind("ENTRY_GATE_", 0U) == 0U ||
        mission_target_id.rfind("PRE_ENTRY_", 0U) == 0U;
    if (!low_altitude_mode_ || target == nullptr ||
        (!entry_endpoint &&
         mission_target_id != "EXIT_GATE" &&
         mission_target_id != "HOME_HOVER")) {
      return;
    }
    // Numbered waypoints deliberately do not enter this immutable-coordinate
    // map. Their active candidate remains unchanged during normal EGO
    // replanning and bounded retries, but the validated Stage 3 relocation
    // path may replace an occupied/unreachable endpoint inside the same
    // sector. ENTRY_GATE, EXIT_GATE and HOME_HOVER remain locked.
    const auto inserted = formal_target_coordinates_.emplace(
        mission_target_id, *target);
    if (inserted.second) return;
    const CandidatePoint& locked = inserted.first->second;
    const double change = std::sqrt(
        (target->x - locked.x) * (target->x - locked.x) +
        (target->y - locked.y) * (target->y - locked.y) +
        (target->z - locked.z) * (target->z - locked.z));
    if (change <= 1.0e-6) return;
    ROS_ERROR("[STAGE3_GOAL] blocked coordinate mutation for formal target "
              "%s: requested=(%.6f, %.6f, %.6f) "
              "locked=(%.6f, %.6f, %.6f)",
              mission_target_id.c_str(), target->x, target->y, target->z,
              locked.x, locked.y, locked.z);
    target->x = locked.x;
    target->y = locked.y;
    target->z = locked.z;
  }

  void noteEgoGoalPublication(const CandidatePoint& target,
                              const std::string& mission_target_id,
                              const std::string& reason,
                              const std::string& source) {
    previous_mission_target_id_ =
        mission_target_id_.empty() ? "NONE" : mission_target_id_;
    mission_target_id_ = mission_target_id;
    mission_target_x_ = target.x;
    mission_target_y_ = target.y;
    mission_target_z_ = target.z;
    target_switch_reason_ = reason.empty() ? "UNSPECIFIED" : reason;
    goal_publication_source_ = source;
    distance_to_target_at_publish_ = std::numeric_limits<double>::infinity();
    if (have_odom_) {
      const auto& position = odom_.pose.pose.position;
      distance_to_target_at_publish_ = std::sqrt(
          (position.x - target.x) * (position.x - target.x) +
          (position.y - target.y) * (position.y - target.y) +
          (position.z - target.z) * (position.z - target.z));
    }
    ++ego_goal_publish_count_;
    ROS_WARN("[STAGE3_GOAL] publication=FORMAL_MISSION_TARGET "
             "mission_target_id=%s mission_target_xyz=(%.6f, %.6f, %.6f) "
             "previous_target_id=%s target_switch_reason=\"%s\" "
             "distance_to_target=%.6f ego_goal_publish_count=%u source=%s",
             mission_target_id_.c_str(), mission_target_x_,
             mission_target_y_, mission_target_z_,
             previous_mission_target_id_.c_str(),
             target_switch_reason_.c_str(), distance_to_target_at_publish_,
             ego_goal_publish_count_, goal_publication_source_.c_str());
  }

  static std::string csvText(std::string value) {
    for (char& character : value) {
      if (character == ',' || character == '\n' || character == '\r') {
        character = ' ';
      }
    }
    return value;
  }

  void publishGoal(const CandidatePoint& target, GoalKind kind) {
    const CandidatePoint previous_target = last_published_target_;
    active_target_ = target;
    active_goal_kind_ = kind;
    const std::string mission_target_id =
        formalMissionTargetId(active_target_, kind);
    lockFormalTargetCoordinates(mission_target_id, &active_target_);
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
         kind == GoalKind::kSector || kind == GoalKind::kSectorDetour ||
         kind == GoalKind::kRecovery ||
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
    noteEgoGoalPublication(active_target_, mission_target_id,
                           last_transition_reason_, "TASK_POSESTAMPED");
    goal_pub_.publish(goal); target_pub_.publish(goal);
    awaiting_fresh_trajectory_ = true;
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
    if (kind == GoalKind::kSector || kind == GoalKind::kSectorDetour) {
      if (kind == GoalKind::kSector) {
        ++current_target_plan_attempt_;
      }
      transition(MissionState::kNavigate,
                 kind == GoalKind::kSectorDetour
                     ? "known-obstacle corridor detour sent to EGO"
                     : "sector target sent to EGO");
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

  std::string sectorFailureDetail() const {
    if (current_sector_ >= sectors_.size()) {
      return "SECTOR_INDEX_INVALID";
    }
    const Sector& sector = sectors_[current_sector_];
    std::unordered_map<std::string, int> counts;
    for (const CandidatePoint& candidate : sector.candidates) {
      const std::string key = candidate.rejection_reason.empty()
                                  ? "UNSPECIFIED_REJECTION"
                                  : candidate.rejection_reason;
      ++counts[key];
    }
    std::ostringstream stream;
    stream << "SECTOR_" << (sector.sector_id + 1)
           << "_NO_SAFE_REACHABLE_CANDIDATE total="
           << sector.candidates.size() << " reasons=";
    bool first = true;
    for (const auto& item : counts) {
      if (!first) stream << '|';
      stream << item.first << ':' << item.second;
      first = false;
    }
    return stream.str();
  }

  int activeSectorId() const {
    return current_sector_ < sectors_.size()
               ? sectors_[current_sector_].sector_id + 1
               : -1;
  }

  void recordIngressTrace(const geometry_msgs::Point& point) {
    if (!low_altitude_mode_) return;
    if (!successful_ingress_trace_.empty()) {
      const CandidatePoint& previous = successful_ingress_trace_.back();
      if (std::sqrt((point.x - previous.x) * (point.x - previous.x) +
                    (point.y - previous.y) * (point.y - previous.y) +
                    (point.z - previous.z) * (point.z - previous.z)) < 2.0) {
        return;
      }
    }
    CandidatePoint sample;
    sample.id = "INGRESS_TRACE_" +
                std::to_string(successful_ingress_trace_.size());
    sample.x = point.x;
    sample.y = point.y;
    sample.z = point.z;
    sample.require_arrival_yaw = false;
    sample.face_tower = false;
    successful_ingress_trace_.push_back(sample);
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
    last_transition_reason_ = reason;
    if (state_ == next) return;
    const double previous_duration = state_entered_.isZero()
                                         ? 0.0
                                         : (ros::Time::now() - state_entered_).toSec();
    ROS_WARN("[STAGE3_TASK] uav=%d %s -> %s: %s; layer=%d height=%.2f "
             "waypoint_index=%zu active=%s(%.2f, %.2f, %.2f) "
             "sector=%d cycle=%d/%d final_return=%s state_duration=%.3fs",
             uav_id_, missionStateName(state_), missionStateName(next), reason.c_str(),
             current_layer_, route_.height, visit_cursor_,
             active_target_.id.empty() ? "none" : active_target_.id.c_str(),
             active_target_.x, active_target_.y, active_target_.z,
             activeSectorId(),
             static_cast<int>(current_layer_visit_index_ /
                              inspection_heights_.size()) +
                 1,
             planned_cycles_, final_return_ ? "true" : "false",
             previous_duration);
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
    const auto permission_ready =
        [this, &now](bool required, bool permission,
                     const ros::Time& received) {
          return !required ||
                 (permission && !received.isZero() &&
                  fresh(now, received, coordination_permission_timeout_));
        };
    const bool transition_permission_ready = permission_ready(
        require_transition_permission_, transition_permission_,
        transition_permission_received_);
    const bool task_start_permission_ready = permission_ready(
        enable_control_ && require_task_start_permission_, task_start_permission_,
        task_start_permission_received_);
    const bool entry_permission_ready = permission_ready(
        require_entry_permission_, entry_permission_,
        entry_permission_received_);
    const bool orbit_permission_ready = permission_ready(
        require_orbit_permission_, orbit_permission_,
        orbit_permission_received_);
    const bool orbit_staging_permission_ready = permission_ready(
        require_orbit_staging_permission_, orbit_staging_permission_,
        orbit_staging_permission_received_);
    const bool exit_permission_ready = permission_ready(
        require_exit_permission_, exit_permission_,
        exit_permission_received_);
    const bool landing_permission_ready = permission_ready(
        require_landing_permission_, landing_permission_,
        landing_permission_received_);
    if (enable_control_ && require_transition_permission_ &&
        active_mission_state && !transition_permission_ready) {
      coordination_hold_pending_ = true;
      phase_hold_pending_ = false;
      requestHold("SWARM_SAFETY_INHIBIT: coordinator permission false/stale");
      return;
    }
    const bool active_orbit_state =
        orbit_released_latched_ && (
        state_ == MissionState::kEvaluate ||
        state_ == MissionState::kTargetLocked ||
        state_ == MissionState::kNavigate ||
        state_ == MissionState::kRelocating ||
        state_ == MissionState::kRecovering);
    if (enable_control_ && require_orbit_permission_ &&
        active_orbit_state && !orbit_permission_ready) {
      coordination_hold_pending_ = true;
      phase_hold_pending_ = true;
      requestHold(
          "SWARM_PHASE_HOLD: trailing vehicle reached phase lower bound");
      return;
    }
    if (enable_control_ && active_mission_state &&
        (bridge_resume_grace_until_.isZero() ||
         now >= bridge_resume_grace_until_) &&
        have_bridge_state_ && fresh(now, bridge_state_received_, 2.0) &&
        bridge_state_ == "HOLD") {
      // A bridge-local safety HOLD must be acknowledged by the mission before
      // the bridge's finite loss timeout can request AUTO.LAND. Cancelling
      // here turns it into a mission-supervised HOLD and preserves the normal
      // ENTRY_GATE relocation / R1-R2 recovery chain.
      if (low_altitude_mode_ &&
          state_ == MissionState::kEntryGateTransit) {
        low_ingress_replan_pending_ = true;
        publishAltitudePolicy("HOLD_WITHOUT_MISSION_GOAL_REPUBLISH");
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
             << " left the configured entry-height band while a horizontal "
                "path exists";
      requestLowIngressReplan(reason.str());
      return;
    }
    if (state_ == MissionState::kWaitInputs) {
      const bool bridge_ready = (!enable_control_ && bridge_state_ == "DRY_RUN") ||
                                (enable_control_ && bridge_state_ == "HOVER_READY");
      if (have_odom_ && fresh(now, odom_received_, input_timeout_) &&
          mapFresh(now) &&
          have_bridge_state_ && bridge_ready && task_start_permission_ready) {
        // Bridge state is an intentionally latched transition topic, not a
        // heartbeat.  Its latest exact state remains authoritative while
        // MAVROS/odom/map freshness is monitored independently.
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
          // takeoff at the entry height before exposing HOVER_READY. Do not
          // feed that same
          // point through the generic high-altitude staged-climb corridor
          // checks: a zero-length goal can overlap the vehicle's freshly
          // observed local occupancy and be misclassified as a blocked
          // ascent. Keep the coverage dwell, then build the live-map level
          // ingress directly from the measured hover pose.
          approach_index_ = approach_goals_.size();
          coverage_wait_started_ = now;
          transition(MissionState::kSegmentedClimb,
                     "stable entry hover confirmed by bridge; generic high-"
                     "altitude climb bypassed");
        } else {
          transition(MissionState::kStaging,
                     "odom/map ready; home-local ascent channel locked");
          }
      } else if (enable_control_ && require_task_start_permission_ &&
                 have_bridge_state_ && bridge_ready &&
                 !task_start_permission_ready) {
        ROS_INFO_THROTTLE(
            1.0,
            "[STAGE3_DIAG] uav=%d HOVER_READY waiting unified task-start "
            "barrier mission_state=%s action=HOLD_HOVER_FAIL_CLOSED",
            uav_id_, missionStateName(state_));
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
          if (!entry_permission_ready) {
            coverage_wait_started_ = ros::Time(0);
            transition(MissionState::kWaitEntryPermission,
                       "entry hover and map are ready; waiting for exclusive "
                       "swarm ENTRY corridor permission");
          } else if (lockFinalEntryGate(now) && startEntryGateTransit()) {
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
    } else if (state_ == MissionState::kWaitEntryPermission) {
      if (!mapFresh(now)) {
        ROS_INFO_THROTTLE(
            1.0,
            "[STAGE5_ENTRY] UAV%d waiting for fresh map before corridor "
            "candidate generation",
            uav_id_);
      } else if (entry_corridor_candidates_.empty() &&
                 !entry_corridor_locked_) {
        if (!buildEntryCorridorCandidates(now)) {
          entry_gate_failure_pending_ = true;
          entry_gate_no_safe_candidate_hold_ = true;
          requestHold("NO_FULL_CORRIDOR_CANDIDATE_AT_1M_CLEARANCE");
        }
      } else if (entry_corridor_locked_ &&
                 !selectedEntryCorridorStillSafe(now)) {
        invalidateEntryCorridor(now, "LIVE_MAP_CHANGED");
      } else if (entry_permission_ready && entry_corridor_locked_) {
        if (startEntryGateTransit()) {
          coverage_wait_started_ = ros::Time(0);
        }
      } else {
        publishEntryCorridorCandidates(now);
        ROS_INFO_THROTTLE(
            1.0,
            "[STAGE5_ENTRY] UAV%d waiting for joint corridor selection and "
            "common entry permission candidates=%zu locked=%s",
            uav_id_, entry_corridor_candidates_.size(),
            entry_corridor_locked_ ? "true" : "false");
      }
    } else if (state_ == MissionState::kEntryGateTransit) {
      if (entry_gate_transit_index_ >= entry_gate_transit_goals_.size()) {
        if (low_altitude_mode_) {
          publishAltitudePolicy("NOMINAL_ENTRY_AND_ORBIT");
        }
        std::string entry_reason;
        if (!entrySystemsHealthy(now, &entry_reason)) {
          entry_gate_failure_pending_ = true;
          ascent_retry_pending_ = false;
          requestHold(entry_reason);
        } else if (!orbit_staging_permission_ready) {
          requestTracking(false);
          transition(
              MissionState::kWaitOrbitStagingPermission,
              "ENTRY_GATE reached; ENTRY_READY waiting for common "
              "MOVE_TO_ORBIT_STAGING permission");
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
          const double endpoint_clearance = filter_config_.minimum_clearance;
          const std::string endpoint_blockage =
              mappedEndpointBlockage(target, endpoint_clearance);
          if (!endpoint_blockage.empty()) {
            if (low_altitude_mode_) {
              requestLowIngressReplan(
                  "ENTRY_GATE_TARGET_OCCUPIED: " + endpoint_blockage);
            } else {
              entry_gate_failure_pending_ = true;
              ascent_retry_pending_ = false;
              entry_gate_relocation_pending_ = true;
              requestHold(
                  "ENTRY_GATE_TARGET_OCCUPIED: " + endpoint_blockage);
            }
          } else {
            const std::string static_risk = staticEndpointRisk(target);
            if (!static_risk.empty()) {
              // The formal endpoint is clear. A blocked straight corridor is
              // only a risk hint; EGO owns the detour to this same endpoint.
              ROS_WARN("[STAGE3_TASK] ENTRY_GATE_STATIC_CORRIDOR_RISK at %s "
                       "for %s; mapped endpoint is clear and remains locked "
                       "for EGO planning",
                       static_risk.c_str(), target.id.c_str());
            }
            coverage_wait_started_ = ros::Time(0);
            publishGoal(target, GoalKind::kEntryGate);
          }
        }
      } else if (low_altitude_mode_ ? arrived(now)
                                    : returnEgressGoalReached()) {
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
            ROS_WARN_THROTTLE(
                1.0,
                "[STAGE3_TASK] EGO internal ENTRY_GATE replan status=%s; "
                "formal target remains unchanged and is not republished",
                failure.c_str());
          } else {
            entry_gate_failure_pending_ = true;
            ascent_retry_pending_ = true;
            entry_gate_relocation_pending_ = true;
            ++ascent_goal_attempts_;
            requestHold("ENTRY_GATE transit planner failure: " + failure);
          }
        } else if (noProgressTimedOut(now)) {
          if (low_altitude_mode_) {
            requestReturnOrLand(
                "ENTRY_GATE no progress; mission-level goal retry disabled");
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
            requestReturnOrLand(
                "ENTRY_GATE target timeout; mission-level goal retry disabled");
          } else {
            entry_gate_failure_pending_ = true;
            ascent_retry_pending_ = true;
            entry_gate_relocation_pending_ = true;
            ++ascent_goal_attempts_;
            requestHold("ENTRY_GATE transit target timeout");
          }
        }
      }
    } else if (state_ == MissionState::kWaitOrbitStagingPermission) {
      std::string entry_reason;
      if (!entrySystemsHealthy(now, &entry_reason)) {
        entry_gate_failure_pending_ = true;
        ascent_retry_pending_ = false;
        requestHold(entry_reason);
      } else if (orbit_staging_permission_ready) {
        if (!requestResume()) {
          requestHold("MOVE_TO_ORBIT_STAGING bridge resume rejected");
        } else if (!lockDirectionalInitialSector(now)) {
          entry_gate_failure_pending_ = true;
          ascent_retry_pending_ = false;
          requestHold("CONFIGURED_FIRST_WAYPOINT_UNSAFE_OR_UNREACHABLE");
        } else {
          ROS_WARN("[STAGE3_DIAG] uav=%d MOVE_TO_ORBIT_STAGING accepted "
                   "first_point_angle=%.1fdeg radius=%.2fm",
                   uav_id_, positiveAngleDegrees(std::atan2(
                                active_target_.y - route_.center_y,
                                active_target_.x - route_.center_x)),
                   std::hypot(active_target_.x - route_.center_x,
                              active_target_.y - route_.center_y));
        }
      }
    } else if (state_ == MissionState::kWaitOrbitPermission) {
      std::string entry_reason;
      if (!entrySystemsHealthy(now, &entry_reason)) {
        requestHold(entry_reason);
      } else if (orbit_permission_ready) {
        if (!requestResume()) {
          requestHold("ORBIT_RELEASE bridge resume rejected");
        } else {
          orbit_released_latched_ = true;
          updateOrbitProgress(pointOf(odom_));
          ++visit_cursor_;
          have_sent_goal_ = false;
          arrival_since_ = ros::Time(0);
          if (visit_cursor_ >= lap_visit_sequence_.size()) {
            requestHold("ORBIT_RELEASE invalid closed-lap sequence");
          } else {
            current_sector_ = lap_visit_sequence_[visit_cursor_];
            waypoint_in_lap_ = static_cast<int>(visit_cursor_) + 1;
            ROS_WARN("[STAGE3_DIAG] uav=%d individual ORBIT_RELEASE "
                     "accepted start_angle=%.3fdeg sector_mask=0x%02X "
                     "action=BEGIN_INDEPENDENT_LAP",
                     uav_id_, positiveAngleDegrees(orbit_start_angle_),
                     static_cast<int>(visited_sector_mask_));
            transition(MissionState::kEvaluate,
                       "individual ORBIT_RELEASE accepted; independent lap "
                       "counter initialized");
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
        requestHold(sectorFailureDetail());
      }
    } else if (state_ == MissionState::kTargetLocked) {
      // The locked endpoint is the formal numbered waypoint. Do not replace
      // it with task-level A* vertices or planning-horizon subgoals.
      publishGoal(active_target_, sector_detour_active_
                                      ? GoalKind::kSectorDetour
                                      : GoalKind::kSector);
    } else if (state_ == MissionState::kNavigate) {
      // In low-altitude inspection, the unchanged 0.5 m position tolerance is
      // only a pass-through trigger. It must not wait for low speed, dwell,
      // HOLD, or completion of the old B-spline.
      const bool waypoint_reached =
          low_altitude_mode_ ? returnEgressGoalReached() : arrived(now);
      if (waypoint_reached) {
        if (arrival_since_.isZero()) arrival_since_ = now;
        if (low_altitude_mode_ ||
            now - arrival_since_ >= ros::Duration(arrival_hold_duration_)) {
          if (sector_detour_active_ &&
              active_goal_kind_ == GoalKind::kSectorDetour) {
            ROS_WARN("[STAGE3_TASK] uav=%d sector=%d static detour reached; "
                     "publishing final same-sector target %s with a fresh "
                     "EGO trajectory",
                     uav_id_, activeSectorId(),
                     pending_sector_target_.id.c_str());
            active_target_ = pending_sector_target_;
            sector_detour_active_ = false;
            have_sent_goal_ = false;
            arrival_since_ = ros::Time(0);
            transition(MissionState::kTargetLocked,
                       "static detour reached; final same-sector target "
                       "locked");
            return;
          }
          if (initial_waypoint_pending_ &&
              require_orbit_staging_permission_ &&
              !orbit_released_latched_) {
            sectors_[current_sector_].state = SectorState::kCovered;
            initial_waypoint_pending_ = false;
            orbit_staging_arrived_ = true;
            last_inspection_target_ = active_target_;
            have_last_inspection_target_ = true;
            ++visited_sector_count_;
            have_sent_goal_ = false;
            arrival_since_ = ros::Time(0);
            if (!requestTracking(false)) {
              requestHold("ORBIT_STAGING position latch rejected");
              return;
            }
            const auto position = pointOf(odom_);
            ROS_WARN("[STAGE3_DIAG] uav=%d ORBIT_STAGING_READY "
                     "position=(%.3f,%.3f,%.3f) target=(%.3f,%.3f,%.3f) "
                     "error=%.3fm speed=%.3fm/s "
                     "action=LATCH_CURRENT_POSITION_WAIT_OWN_RELEASE",
                     uav_id_, position.x, position.y, position.z,
                     active_target_.x, active_target_.y, active_target_.z,
                     std::sqrt(
                         (position.x - active_target_.x) *
                             (position.x - active_target_.x) +
                         (position.y - active_target_.y) *
                             (position.y - active_target_.y) +
                         (position.z - active_target_.z) *
                             (position.z - active_target_.z)),
                     std::hypot(odom_.twist.twist.linear.x,
                                odom_.twist.twist.linear.y));
            transition(MissionState::kWaitOrbitPermission,
                       "first orbit point reached and position latched; "
                       "waiting for individual ORBIT_RELEASE");
            return;
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
            std::string orbit_completion_reason;
            if (low_altitude_mode_ && sector_limit_ == 8 &&
                !independentOrbitComplete(&orbit_completion_reason)) {
              requestHold("INDEPENDENT_ORBIT_INCOMPLETE: " +
                          orbit_completion_reason);
              return;
            }
            completed_laps_ = inspection_laps_;
            ++completed_layer_count_;
            ROS_WARN("[STAGE3_DIAG] uav=%d orbit complete laps=%d "
                     "sector=%d target=%s %s "
                     "action=REQUEST_EXIT_PERMISSION",
                     uav_id_, completed_laps_, activeSectorId(),
                     active_target_.id.c_str(),
                     orbit_completion_reason.c_str());
            if (current_layer_visit_index_ + 1U <
                layer_visit_sequence_.size()) {
              if (!transition_permission_ready) {
                requestTracking(false);
                transition(MissionState::kWaitLayerPermission,
                           "closed layer complete at transition anchor; "
                           "waiting for swarm interlock");
              } else {
                std::string transition_reason;
                if (!startLayerTransition(now, &transition_reason)) {
                  layer_transition_failure_pending_ = true;
                  layer_transition_retry_pending_ = false;
                  requestHold(transition_reason);
                }
              }
            } else if (!exit_permission_ready) {
              requestTracking(false);
              transition(MissionState::kWaitExitPermission,
                         "final closed inspection layer completed; waiting "
                         "for exclusive swarm EXIT/return corridor permission");
              ROS_WARN("[STAGE3_DIAG] uav=%d exit permission requested "
                       "mission_state=%s sector=%d target=%s",
                       uav_id_, missionStateName(state_), activeSectorId(),
                       active_target_.id.c_str());
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
    } else if (state_ == MissionState::kWaitLayerPermission) {
      if (transition_permission_ready) {
        std::string transition_reason;
        if (!startLayerTransition(now, &transition_reason)) {
          layer_transition_failure_pending_ = true;
          layer_transition_retry_pending_ = false;
          requestHold(transition_reason);
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
    } else if (state_ == MissionState::kWaitExitPermission) {
      if (exit_permission_ready) {
        ROS_WARN("[STAGE3_DIAG] uav=%d exit permission granted "
                 "mission_state=%s sector=%d target=%s action=MOVE_TO_EXIT_GATE",
                 uav_id_, missionStateName(state_), activeSectorId(),
                 active_target_.id.c_str());
        if (requestResume() &&
            startExitGate(
                now,
                "exclusive swarm EXIT/return corridor permission granted")) {
          // The existing EXIT_GATE and return logic resumes unchanged.
        }
      }
    } else if (state_ == MissionState::kGoToExitGate) {
      if (!have_sent_goal_) {
        const CandidatePoint exit_gate = active_target_;
        publishGoal(exit_gate, GoalKind::kExitGate);
      } else if (low_altitude_mode_ ? returnEgressGoalReached()
                                    : arrived(now)) {
        if (arrival_since_.isZero()) arrival_since_ = now;
        if (low_altitude_mode_ ||
            now - arrival_since_ >=
                ros::Duration(arrival_hold_duration_)) {
          if (low_altitude_mode_) {
            recordIngressTrace(pointOf(odom_));
            ROS_WARN("[STAGE3_DIAG] uav=%d exit gate reached layer=%d "
                     "sector=%d xyz=(%.3f,%.3f,%.3f) "
                     "action=REVERSE_ACTUAL_INGRESS_WITH_FRESH_EGO_PLANS",
                     uav_id_, current_layer_, activeSectorId(),
                     active_target_.x, active_target_.y, active_target_.z);
            if (!startNormalReturn(
                    "current-layer EXIT_GATE reached; reverse actual ingress",
                    false)) {
              requestReturnOrLand(
                  "EXIT_GATE reached but reverse ingress reference unavailable");
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
        if (coordination_hold_pending_) {
          const bool resuming_entry =
              state_before_hold_ == MissionState::kEntryGateTransit;
          const bool coordination_restored =
              transition_permission_ready &&
              (!phase_hold_pending_ || orbit_permission_ready) &&
              (!resuming_entry || entry_permission_ready);
          if (!coordination_restored) {
            ROS_ERROR_THROTTLE(
                1.0,
                "[STAGE3_TASK] swarm safety/phase permission remains "
                "fail-closed; the cancelled trajectory will not be resumed");
          } else if (resuming_entry) {
            coordination_hold_pending_ = false;
            phase_hold_pending_ = false;
            const double hold_duration = (now - state_entered_).toSec();
            // The safety layer cancelled the old B-spline, so it must never
            // be resumed or replayed.  Once both global safety and the
            // per-drone ENTRY ownership are fresh again, hand the unchanged
            // locked gate to EGO as a new goal generation.  The existing
            // ENTRY planner/no-progress/target and overall mission timeouts
            // remain the bounded failure path if the corridor cannot be
            // traversed after coordination is restored.
            if (requestResume() && entry_gate_locked_ &&
                startEntryGateTransit()) {
              ROS_WARN(
                  "[STAGE3_DIAG] uav=%d ENTRY safety HOLD recovered "
                  "mission_state=%s sector=%d target=%s(%.2f,%.2f,%.2f) "
                  "condition=SWARM_SAFETY_RESTORED "
                  "ego_replan_result=FRESH_GOAL_REQUESTED "
                  "trajectory_expired=true predicted_swarm_conflict=false "
                  "state_duration=%.3fs action=REPUBLISH_LOCKED_ENTRY_GATE",
                  uav_id_, missionStateName(state_), activeSectorId(),
                  provisional_entry_gate_.id.c_str(),
                  provisional_entry_gate_.x, provisional_entry_gate_.y,
                  provisional_entry_gate_.z,
                  hold_duration);
            } else {
              requestReturnOrLand(
                  "swarm safety restored after ENTRY safety stop but fresh "
                  "ENTRY goal generation was rejected");
            }
          } else if (requestResume()) {
            coordination_hold_pending_ = false;
            phase_hold_pending_ = false;
            have_sent_goal_ = false;
            const MissionState resume_state =
                state_before_hold_ == MissionState::kNavigate
                    ? MissionState::kTargetLocked
                    : state_before_hold_;
            transition(
                resume_state,
                "swarm safety/phase restored; resending a fresh goal "
                "generation");
          }
        } else if (low_ingress_replan_pending_) {
          low_ingress_replan_pending_ = false;
          requestReturnOrLand(
              failure_reason_ +
              "; formal ENTRY_GATE was not republished after safety HOLD");
        } else if (entry_gate_no_safe_candidate_hold_) {
          if (entry_gate_recheck_time_.isZero() ||
              now - entry_gate_recheck_time_ >=
                  ros::Duration(safe_altitude_map_dwell_)) {
            entry_gate_recheck_time_ = now;
            if (low_altitude_mode_ && require_entry_permission_) {
              entry_corridor_candidates_.clear();
              entry_corridor_locked_ = false;
              entry_gate_locked_ = false;
              if (buildEntryCorridorCandidates(now)) {
                entry_gate_no_safe_candidate_hold_ = false;
                entry_gate_failure_pending_ = false;
                entry_gate_relocation_pending_ = false;
                transition(
                    MissionState::kWaitEntryPermission,
                    "fresh live map exposed dynamic ENTRY corridor "
                    "candidates; waiting for a new joint selection");
              } else {
                ROS_ERROR_THROTTLE(
                    1.0,
                    "[STAGE5_ENTRY] UAV%d no dynamic full-corridor "
                    "candidate at map clearance %.2fm; legacy fixed gate "
                    "fallback is forbidden",
                    uav_id_, mappedTaskClearance());
              }
            } else if (lockLayerGate(0, now) && startEntryGateTransit()) {
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
            std::vector<bool> endpoint_safety(normal_return_goals_.size(),
                                              false);
            for (std::size_t index = normal_return_index_ + 1U;
                 index < normal_return_goals_.size(); ++index) {
              endpoint_safety[index] =
                  normalReturnGoalSafe(normal_return_goals_[index], now);
            }
            const std::size_t failed_index = normal_return_index_;
            const std::size_t next_index = nextSafeReturnGoalIndex(
                endpoint_safety, failed_index);
            if (next_index < normal_return_goals_.size()) {
              ROS_WARN(
                  "[STAGE3_DIAG] uav=%d blocked reverse-ingress endpoint "
                  "skipped failed_index=%zu failed_target=%s "
                  "next_index=%zu next_target=%s skipped=%zu "
                  "map_fresh=true action=CONTINUE_REVERSE_ACTUAL_INGRESS",
                  uav_id_, failed_index + 1U,
                  normal_return_goals_[failed_index].id.c_str(),
                  next_index + 1U,
                  normal_return_goals_[next_index].id.c_str(),
                  next_index - failed_index);
              normal_return_index_ = next_index;
              normal_return_retries_ = 0;
              have_sent_goal_ = false;
              transition(
                  MissionState::kNormalReturn,
                  "blocked historical ingress sample skipped after bounded "
                  "retries; continuing reverse actual ingress with the next "
                  "safe endpoint and a fresh EGO plan");
            } else {
              requestReturnOrLand(
                  "normal return retries exhausted and no later reverse "
                  "ingress endpoint is safe; RETURN_EGRESS fallback");
            }
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
          ROS_WARN("[STAGE3_DIAG] uav=%d return goal published "
                   "index=%zu/%zu target=%s(%.2f,%.2f,%.2f) "
                   "map_fresh=true action=EGO_FRESH_REPLAN",
                   uav_id_, normal_return_index_ + 1U,
                   normal_return_goals_.size(),
                   normal_return_goals_[normal_return_index_].id.c_str(),
                   normal_return_goals_[normal_return_index_].x,
                   normal_return_goals_[normal_return_index_].y,
                   normal_return_goals_[normal_return_index_].z);
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
        if (!landing_permission_ready) {
          ROS_INFO_THROTTLE(
              2.0,
              "[STAGE3_TASK] HOME_HOVER: waiting for swarm landing permission");
        } else if (landing_requested_time_.isZero()) {
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
    if (low_altitude_mode_) {
      requestBridgeReturnOrLand(
          reason + "; low-altitude task keeps one direct HOME_HOVER target");
      return;
    }
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
    CandidatePoint home_hover;
    home_hover.id = "HOME_HOVER";
    home_hover.x = home_position_.x;
    home_hover.y = home_position_.y;
    home_hover.z = home_position_.z;
    home_hover.require_arrival_yaw = false;
    home_hover.face_tower = false;
    lockFormalTargetCoordinates("HOME_HOVER", &home_hover);
    planner_target_baseline_ =
        have_planner_status_ ? planner_status_.target_id : std::string();
    trajectory_baseline_ = have_command_ ? command_.trajectory_id : 0;
    if (return_client_.call(service) && service.response.success) {
      goal_sent_ = ros::Time::now();
      have_sent_goal_ = true;
      active_target_ = home_hover;
      active_goal_kind_ = GoalKind::kNormalReturn;
      std_msgs::Bool face_tower;
      face_tower.data = false;
      face_tower_pub_.publish(face_tower);
      target_pub_.publish(makeGoal(home_hover));
      noteEgoGoalPublication(
          home_hover, "HOME_HOVER",
          reason + "; bridge return_home published the EGO goal",
          "BRIDGE_RETURN_SERVICE");
      awaiting_fresh_trajectory_ = true;
      ROS_WARN("[STAGE3_DIAG] uav=%d bridge accepted goal service=return_home "
               "target=HOME_HOVER(%.2f,%.2f,%.2f) namespace_goal=%s "
               "action=WAIT_FRESH_EGO_TRAJECTORY",
               uav_id_, home_hover.x, home_hover.y, home_hover.z,
               goal_topic_.c_str());
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
                << active_target_.z << ','
                << mission_target_id_ << ','
                << mission_target_x_ << ',' << mission_target_y_ << ','
                << mission_target_z_ << ','
                << previous_mission_target_id_ << ','
                << csvText(target_switch_reason_) << ','
                << distance_to_target_at_publish_ << ','
                << ego_goal_publish_count_ << ','
                << goal_publication_source_ << ','
                << position.x << ',' << position.y
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
      extended_state_sub_, task_start_permission_sub_, transition_permission_sub_,
      entry_permission_sub_, entry_corridor_selection_sub_,
      orbit_staging_permission_sub_,
      orbit_permission_sub_, exit_permission_sub_,
      landing_permission_sub_;
  ros::Publisher goal_pub_, state_pub_, target_pub_,
      sector_pub_,
      candidates_pub_, entry_corridor_candidates_pub_, progress_pub_,
      face_tower_pub_, tower_center_pub_,
      level_ingress_path_pub_, level_orbit_path_pub_, mission_route_pub_,
      altitude_policy_pub_;
  ros::ServiceClient tracking_client_, cancel_client_, resume_client_, return_client_, land_client_;
  ros::Timer timer_;
  std::string planning_frame_, odom_topic_, command_topic_, cloud_topic_, occupancy_topic_, planner_status_topic_,
      bridge_state_topic_, goal_topic_, state_topic_, target_topic_,
      sector_topic_, candidates_topic_,
      entry_corridor_candidates_topic_, entry_corridor_selection_topic_,
      progress_topic_, face_tower_topic_, tower_center_topic_,
      level_ingress_path_topic_, level_orbit_path_topic_,
      mission_route_topic_,
      altitude_policy_topic_,
      mavros_state_topic_, mavros_extended_state_topic_, tracking_service_,
      cancel_service_, resume_service_, return_service_, land_service_,
      task_start_permission_topic_, transition_permission_topic_, entry_permission_topic_,
      orbit_staging_permission_topic_, orbit_permission_topic_,
      exit_permission_topic_, landing_permission_topic_;
  bool enable_control_{false};
  bool require_task_start_permission_{false};
  bool require_transition_permission_{false};
  bool require_entry_permission_{false};
  bool require_orbit_staging_permission_{false};
  bool require_orbit_permission_{false};
  bool require_exit_permission_{false};
  bool require_landing_permission_{false};
  bool coordination_hold_pending_{false};
  bool phase_hold_pending_{false};
  bool task_start_permission_{false};
  bool transition_permission_{false};
  bool entry_permission_{false};
  bool orbit_staging_permission_{false};
  bool orbit_permission_{false};
  bool exit_permission_{false};
  bool landing_permission_{false};
  bool low_altitude_mode_{false};
  int uav_id_{0};
  bool use_configured_staging_xy_{false};
  bool prefer_safe_overflight_{true};
  double loop_rate_{20.0}, input_timeout_{0.7},
      entry_fcu_state_timeout_{2.0}, map_timeout_{0.7},
      planning_timeout_{10.0}, goal_timeout_{180.0},
      coordination_permission_timeout_{1.0};
  double coverage_wait_timeout_{12.0}, coverage_angular_bin_deg_{2.0},
      coverage_neighborhood_radius_{3.0}, coverage_sample_resolution_{1.0},
      coverage_minimum_range_{0.5}, coverage_maximum_range_{40.0},
      current_coverage_unknown_ratio_{1.0};
  double safe_altitude_map_dwell_{4.0}, sensor_min_elevation_deg_{-7.2},
      sensor_max_elevation_deg_{52.2};
  double return_timeout_{300.0}, landing_timeout_{90.0};
  double return_egress_target_timeout_{90.0}, return_home_xy_tolerance_{1.5};
  double arrival_tolerance_{0.5}, arrival_velocity_threshold_{0.2},
      arrival_yaw_tolerance_{0.26}, arrival_hold_duration_{1.0};
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
  double planner_map_size_x_{0.0}, planner_map_size_y_{0.0},
      planner_map_planning_horizon_{0.0};
  double maximum_temporary_descent_{3.0};
  double target_replacement_margin_{2.0}, recovery_height_{35.0}, start_angle_deg_{0.0};
  double entry_gate_segment_length_{6.0}, entry_gate_target_timeout_{90.0};
  double entry_angle_deg_{0.0}, entry_angle_rad_{0.0},
      entry_nominal_angle_deg_{0.0}, pre_entry_radius_{18.0},
      pre_entry_maximum_radius_{19.5},
      pre_entry_radial_sample_step_{0.25},
      entry_angular_search_half_width_deg_{7.5};
  double layer_transition_target_timeout_{90.0};
  double layer_transition_maximum_vertical_step_{2.0};
  double layer_transition_same_xy_tolerance_{0.25};
  double staging_height_{4.0}, climb_height_step_{3.0},
      minimum_channel_hold_{5.0};
  double configured_staging_x_{0.0}, configured_staging_y_{0.0};
  double takeoff_delay_{0.0};
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
  std::vector<CandidatePoint> normal_return_goals_;
  std::vector<CandidatePoint> return_egress_goals_;
  std::vector<CandidatePoint> entry_gate_candidates_;
  std::vector<EntryCorridorCandidate> entry_corridor_candidates_;
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
      visited_sector_count_{0}, visit_cursor_{0},
      current_layer_visit_index_{0};
  int return_egress_retries_{0}, normal_return_retries_{0},
      current_target_plan_attempt_{0}, entry_gate_relocations_{0},
      current_lap_{1},
      completed_laps_{0}, waypoint_in_lap_{0}, candidate_relocations_{0},
      lap_candidate_relocations_{0}, lap_planning_failures_{0},
      lap_recoveries_{0}, low_no_path_confirmations_{0},
      low_no_path_confirmation_limit_{3}, low_ingress_replans_{0},
      low_orbit_replans_{0}, max_low_ingress_replans_{5};
  int entry_sector_user_{1}, entry_sector_index_{0},
      inspection_start_sector_{0}, transition_sector_{0},
      current_layer_{0}, completed_layer_count_{0},
      layer_transition_attempts_{0};
  int entry_gate_index_{-1};
  int provisional_entry_gate_index_{-1};
  int ascent_channel_index_{0}, ascent_goal_attempts_{0};
  CandidatePoint provisional_entry_gate_, staging_target_,
      last_inspection_target_, layer_start_anchor_, layer_transition_anchor_,
      pending_layer_start_anchor_;
  CandidatePoint active_target_, last_published_target_;
  CandidatePoint pending_sector_target_;
  EntryCorridorCandidate selected_entry_corridor_;
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
  bool awaiting_fresh_trajectory_{false};
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
      low_ingress_replan_pending_{false}, sector_detour_active_{false};
  bool entry_corridor_locked_{false};
  std::uint32_t entry_corridor_generation_{0U};
  bool orbit_staging_arrived_{false}, orbit_released_latched_{false};
  geometry_msgs::Point home_position_, coverage_origin_;
  ros::Time odom_received_, command_received_, cloud_received_,
      occupancy_received_, planner_status_received_, bridge_state_received_,
      coverage_received_, fcu_state_received_, extended_state_received_,
      task_start_permission_received_, transition_permission_received_, entry_permission_received_,
      orbit_staging_permission_received_, orbit_permission_received_,
      exit_permission_received_,
      landing_permission_received_, bridge_resume_grace_until_;
  ros::Time state_entered_, mission_started_, goal_sent_, arrival_since_,
      last_report_, bridge_state_entered_, coverage_wait_started_,
      landing_requested_time_, channel_selected_time_, last_progress_time_,
      entry_gate_recheck_time_, last_low_no_path_check_;
  double best_goal_distance_{std::numeric_limits<double>::infinity()};
  double orbit_start_angle_{0.0}, previous_orbit_angle_{0.0},
      accumulated_orbit_angle_{0.0};
  std::uint8_t visited_sector_mask_{0U};
  bool orbit_tracking_initialized_{false}, reverse_orbit_detected_{false};
  std::uint32_t trajectory_baseline_{0};
  std::uint32_t ego_goal_publish_count_{0};
  std::string planner_target_baseline_;
  std::string mission_target_id_, previous_mission_target_id_,
      target_switch_reason_, goal_publication_source_,
      last_transition_reason_;
  double mission_target_x_{0.0}, mission_target_y_{0.0},
      mission_target_z_{0.0};
  double distance_to_target_at_publish_{
      std::numeric_limits<double>::infinity()};
  std::unordered_map<std::string, CandidatePoint> formal_target_coordinates_;
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
