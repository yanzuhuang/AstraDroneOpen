#ifndef ASTRA_TOWER_MISSION_EGO_TASK_UTILS_H_
#define ASTRA_TOWER_MISSION_EGO_TASK_UTILS_H_

#include <geometry_msgs/PoseStamped.h>
#include <ros/time.h>

#include <cstdint>
#include <string>
#include <vector>

namespace astra_tower_mission {

struct TowerCandidate {
  std::string name;
  std::string frame_id;
  double center_x{0.0};
  double center_y{0.0};
  double collision_radius{0.0};
};

double posePositionDistance(const geometry_msgs::PoseStamped& first,
                            const geometry_msgs::PoseStamped& second);

bool validateFixedHeightGoals(
    const std::vector<geometry_msgs::PoseStamped>& goals,
    double expected_height, double tolerance, std::string* reason);

bool validateGoalHeightBounds(
    const std::vector<geometry_msgs::PoseStamped>& goals,
    double minimum_height, double maximum_height, std::string* reason);

std::vector<geometry_msgs::PoseStamped> appendClosureGoal(
    const std::vector<geometry_msgs::PoseStamped>& unique_goals);

int nearestTowerIndex(const std::vector<TowerCandidate>& candidates,
                      double vehicle_x, double vehicle_y);

bool isNewTrajectory(std::uint32_t baseline_id, std::uint32_t command_id,
                     const ros::Time& goal_stamp,
                     const ros::Time& command_stamp);

double selectArrivalTolerance(bool tower_scenario, std::size_t goal_index,
                              std::size_t descent_goal_index,
                              double nominal_tolerance,
                              double transit_transition_tolerance);

std::size_t nextSafeReturnGoalIndex(
    const std::vector<bool>& endpoint_safety, std::size_t failed_index);

}  // namespace astra_tower_mission

#endif  // ASTRA_TOWER_MISSION_EGO_TASK_UTILS_H_
