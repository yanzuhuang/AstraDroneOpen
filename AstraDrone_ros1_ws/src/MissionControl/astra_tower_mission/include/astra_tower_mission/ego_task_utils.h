#ifndef ASTRA_TOWER_MISSION_EGO_TASK_UTILS_H_
#define ASTRA_TOWER_MISSION_EGO_TASK_UTILS_H_

#include <geometry_msgs/PoseStamped.h>
#include <ros/time.h>

#include <cstdint>
#include <string>
#include <vector>

namespace astra_tower_mission {

double posePositionDistance(const geometry_msgs::PoseStamped& first,
                            const geometry_msgs::PoseStamped& second);

bool validateFixedHeightGoals(
    const std::vector<geometry_msgs::PoseStamped>& goals,
    double expected_height, double tolerance, std::string* reason);

std::vector<geometry_msgs::PoseStamped> appendClosureGoal(
    const std::vector<geometry_msgs::PoseStamped>& unique_goals);

bool isNewTrajectory(std::uint32_t baseline_id, std::uint32_t command_id,
                     const ros::Time& goal_stamp,
                     const ros::Time& command_stamp);

}  // namespace astra_tower_mission

#endif  // ASTRA_TOWER_MISSION_EGO_TASK_UTILS_H_
