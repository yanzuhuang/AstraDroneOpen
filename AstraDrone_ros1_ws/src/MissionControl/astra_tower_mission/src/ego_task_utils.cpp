#include "astra_tower_mission/ego_task_utils.h"

#include <cmath>

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

bool isNewTrajectory(std::uint32_t baseline_id, std::uint32_t command_id,
                     const ros::Time& goal_stamp,
                     const ros::Time& command_stamp) {
  return !goal_stamp.isZero() && command_stamp >= goal_stamp &&
         command_id != baseline_id;
}

}  // namespace astra_tower_mission
