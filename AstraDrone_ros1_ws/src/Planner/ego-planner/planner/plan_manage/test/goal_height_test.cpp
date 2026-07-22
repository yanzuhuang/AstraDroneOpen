#include <plan_manage/goal_height.h>

#include <gtest/gtest.h>

#include <limits>

namespace ego_planner
{
namespace
{

TEST(GoalHeight, DefaultsToConfiguredFixedHeight)
{
  double resolved = -1.0;
  EXPECT_TRUE(resolveManualGoalHeight(16.0, 8.0, false, &resolved));
  EXPECT_DOUBLE_EQ(8.0, resolved);
}

TEST(GoalHeight, PreservesIncomingGoalHeightWhenEnabled)
{
  double resolved = -1.0;
  EXPECT_TRUE(resolveManualGoalHeight(4.0, 8.0, true, &resolved));
  EXPECT_DOUBLE_EQ(4.0, resolved);
  EXPECT_TRUE(resolveManualGoalHeight(16.0, 8.0, true, &resolved));
  EXPECT_DOUBLE_EQ(16.0, resolved);
}

TEST(GoalHeight, RejectsInvalidSelectedHeight)
{
  double resolved = -1.0;
  EXPECT_FALSE(resolveManualGoalHeight(
      std::numeric_limits<double>::quiet_NaN(), 8.0, true, &resolved));
  EXPECT_FALSE(resolveManualGoalHeight(4.0, -1.0, false, &resolved));
  EXPECT_FALSE(resolveManualGoalHeight(4.0, 8.0, true, nullptr));
}

}  // namespace
}  // namespace ego_planner

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
