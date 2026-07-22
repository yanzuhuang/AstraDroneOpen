#include "astra_tower_mission/ego_task_utils.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace astra_tower_mission {
namespace {

geometry_msgs::PoseStamped goal(double x, double y, double z) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation.w = 1.0;
  return pose;
}

TEST(EgoTaskGoals, RequiresConfiguredFixedHeight) {
  std::string reason;
  EXPECT_TRUE(validateFixedHeightGoals({goal(1.0, 2.0, 4.0)}, 4.0,
                                       1e-6, &reason));
  EXPECT_FALSE(validateFixedHeightGoals({goal(1.0, 2.0, 3.9)}, 4.0,
                                        1e-6, &reason));
}

TEST(EgoTaskGoals, AppendsFirstGoalToCloseMultiPointRoute) {
  std::vector<geometry_msgs::PoseStamped> goals(2);
  goals[0].pose.position.x = 1.0;
  goals[1].pose.position.x = 2.0;
  const auto closed = appendClosureGoal(goals);
  ASSERT_EQ(3u, closed.size());
  EXPECT_DOUBLE_EQ(1.0, closed.back().pose.position.x);

  EXPECT_EQ(1u,
            appendClosureGoal(
                std::vector<geometry_msgs::PoseStamped>(1)).size());
}

TEST(EgoTaskGoals, SelectsNearestValidTower) {
  const std::vector<TowerCandidate> candidates{
      {"far", "map", 10.7773, 22.4598, 6.41},
      {"radio_tower", "map", -10.0551, 19.7104, 6.41}};
  EXPECT_EQ(1, nearestTowerIndex(candidates, 0.0, 0.0));
  EXPECT_EQ(0, nearestTowerIndex(candidates, 10.0, 22.0));
}

TEST(EgoTaskGoals, IgnoresInvalidTowerAndRejectsInvalidVehiclePose) {
  TowerCandidate invalid;
  invalid.name = "invalid";
  invalid.frame_id = "map";
  invalid.center_x = std::numeric_limits<double>::quiet_NaN();
  invalid.collision_radius = 1.0;
  TowerCandidate valid{"valid", "map", 2.0, 3.0, 1.0};
  EXPECT_EQ(1, nearestTowerIndex({invalid, valid}, 0.0, 0.0));
  EXPECT_EQ(-1, nearestTowerIndex(
                    {valid}, std::numeric_limits<double>::infinity(), 0.0));
}

TEST(EgoTaskProgress, RequiresNewIdAfterGoalStamp) {
  EXPECT_TRUE(isNewTrajectory(4, 5, ros::Time(10.0), ros::Time(10.1)));
  EXPECT_FALSE(isNewTrajectory(4, 4, ros::Time(10.0), ros::Time(10.1)));
  EXPECT_FALSE(isNewTrajectory(4, 5, ros::Time(10.0), ros::Time(9.9)));
}

TEST(EgoTaskProgress, ComputesThreeDimensionalDistance) {
  EXPECT_DOUBLE_EQ(5.0,
                   posePositionDistance(goal(0.0, 0.0, 0.0),
                                        goal(3.0, 4.0, 0.0)));
}

}  // namespace
}  // namespace astra_tower_mission

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
