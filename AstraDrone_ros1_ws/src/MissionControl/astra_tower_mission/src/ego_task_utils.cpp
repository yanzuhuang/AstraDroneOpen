#include "astra_tower_mission/ego_task_utils.h"

#include <cmath>
#include <limits>

namespace astra_tower_mission {

double posePositionDistance(const geometry_msgs::PoseStamped& first,
                            const geometry_msgs::PoseStamped& second) {
  const double dx = first.pose.position.x - second.pose.position.x;
  const double dy = first.pose.position.y - second.pose.position.y;
  const double dz = first.pose.position.z - second.pose.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool validateFixedHeightGoals(
    const std::vector<geometry_msgs::PoseStamped>& goals,
    double expected_height, double tolerance, std::string* reason) {
  if (reason == nullptr || goals.empty() || !std::isfinite(expected_height) ||
      !std::isfinite(tolerance) || tolerance < 0.0) {
    return false;
  }
  for (const auto& goal : goals) {
    if (goal.header.frame_id.empty() ||
        !std::isfinite(goal.pose.position.x) ||
        !std::isfinite(goal.pose.position.y) ||
        !std::isfinite(goal.pose.position.z) ||
        std::abs(goal.pose.position.z - expected_height) > tolerance) {
      *reason =
          "every Stage 2 goal must be finite and exactly match manual_target_height";
      return false;
    }
  }
  reason->clear();
  return true;
}

std::vector<geometry_msgs::PoseStamped> appendClosureGoal(
    const std::vector<geometry_msgs::PoseStamped>& unique_goals) {
  std::vector<geometry_msgs::PoseStamped> closed = unique_goals;
  if (closed.size() > 1U) {
    closed.push_back(closed.front());
  }
  return closed;
}

int nearestTowerIndex(const std::vector<TowerCandidate>& candidates,
                      double vehicle_x, double vehicle_y) {
  if (candidates.empty() || !std::isfinite(vehicle_x) ||
      !std::isfinite(vehicle_y)) {
    return -1;
  }

  int nearest_index = -1;
  double nearest_distance_squared = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    const auto& candidate = candidates[index];
    if (candidate.name.empty() || candidate.frame_id.empty() ||
        !std::isfinite(candidate.center_x) ||
        !std::isfinite(candidate.center_y) ||
        !std::isfinite(candidate.collision_radius) ||
        candidate.collision_radius < 0.0) {
      continue;
    }
    const double dx = candidate.center_x - vehicle_x;
    const double dy = candidate.center_y - vehicle_y;
    const double distance_squared = dx * dx + dy * dy;
    if (distance_squared < nearest_distance_squared) {
      nearest_distance_squared = distance_squared;
      nearest_index = static_cast<int>(index);
    }
  }
  return nearest_index;
}

bool isNewTrajectory(std::uint32_t baseline_id, std::uint32_t command_id,
                     const ros::Time& goal_stamp,
                     const ros::Time& command_stamp) {
  return !goal_stamp.isZero() && command_stamp >= goal_stamp &&
         command_id != baseline_id;
}

}  // namespace astra_tower_mission
