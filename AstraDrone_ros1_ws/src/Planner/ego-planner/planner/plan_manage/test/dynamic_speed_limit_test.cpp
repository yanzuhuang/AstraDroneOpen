#include <gtest/gtest.h>

#include <limits>

#include <plan_manage/dynamic_speed_limit.h>

namespace ego_planner
{
namespace
{

TEST(DynamicSpeedLimitGate, RejectsInvalidConfigurationAndValues)
{
  EXPECT_FALSE(DynamicSpeedLimitGate(0.0, 1.0).validConfiguration());
  EXPECT_FALSE(DynamicSpeedLimitGate(1.0, 0.5).validConfiguration());

  const DynamicSpeedLimitGate gate(0.1, 1.0);
  EXPECT_FALSE(gate.accepts(0.09));
  EXPECT_FALSE(gate.accepts(1.01));
  EXPECT_FALSE(gate.accepts(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_TRUE(gate.accepts(0.1));
  EXPECT_TRUE(gate.accepts(1.0));
}

TEST(DynamicSpeedLimitGate, DoesNotForceReplanInsidePaperInterval)
{
  const DynamicSpeedLimitGate gate(0.1, 2.0);
  EXPECT_FALSE(gate.changed(0.8, 0.8));
  EXPECT_TRUE(gate.changed(0.8, 0.79));
  EXPECT_FALSE(gate.requiresForceReplan(0.8, 0.5));
  EXPECT_FALSE(gate.requiresForceReplan(1.0, 0.9));
  EXPECT_FALSE(gate.requiresForceReplan(1.0, 1.0));
  EXPECT_FALSE(gate.requiresForceReplan(0.8, 1.3));
}

TEST(DynamicSpeedLimitGate, ForcesReplanOutsidePaperInterval)
{
  const DynamicSpeedLimitGate gate(0.1, 2.0);
  EXPECT_TRUE(gate.requiresForceReplan(0.8, 0.499999));
  EXPECT_TRUE(gate.requiresForceReplan(0.8, 1.300001));
  EXPECT_FALSE(gate.requiresForceReplan(
      std::numeric_limits<double>::quiet_NaN(), 1.0));
}

TEST(DynamicSpeedLimitGate, AcceptsReviewedHighSpeedQualificationRequests)
{
  const DynamicSpeedLimitGate gate(0.05, 4.0);
  EXPECT_TRUE(gate.validConfiguration());
  for (const double requested : {1.75, 2.0, 2.5, 3.0, 3.5})
    EXPECT_TRUE(gate.accepts(requested));
  EXPECT_TRUE(gate.accepts(4.0));
  EXPECT_FALSE(gate.accepts(4.000001));
}

}  // namespace
}  // namespace ego_planner

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
