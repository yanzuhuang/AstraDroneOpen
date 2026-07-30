#include <gtest/gtest.h>

#include <cmath>

#include <plan_manage/swarm_frame_transform.h>

namespace
{

TEST(SwarmFrameTransform, AppliesTranslationAndYaw)
{
  ego_planner::SwarmFrameTransform transform(
      4.0, -2.0, 1.0, 3.14159265358979323846 / 2.0);
  geometry_msgs::Point local;
  local.x = 2.0;
  local.y = 1.0;
  local.z = 3.0;
  const geometry_msgs::Point common = transform.localToCommon(local);
  EXPECT_NEAR(common.x, 3.0, 1e-9);
  EXPECT_NEAR(common.y, 0.0, 1e-9);
  EXPECT_NEAR(common.z, 4.0, 1e-9);
}

TEST(SwarmFrameTransform, RoundTripPreservesPoint)
{
  ego_planner::SwarmFrameTransform transform(8.0, 1.5, -0.5, 0.37);
  geometry_msgs::Point input;
  input.x = -3.0;
  input.y = 4.0;
  input.z = 2.0;
  const geometry_msgs::Point output =
      transform.commonToLocal(transform.localToCommon(input));
  EXPECT_NEAR(output.x, input.x, 1e-9);
  EXPECT_NEAR(output.y, input.y, 1e-9);
  EXPECT_NEAR(output.z, input.z, 1e-9);
}

}  // namespace

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
