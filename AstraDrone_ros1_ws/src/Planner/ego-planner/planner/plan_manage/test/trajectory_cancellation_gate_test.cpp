#include <plan_manage/trajectory_cancellation_gate.h>

#include <gtest/gtest.h>

namespace ego_planner
{
namespace
{

const ros::Time kOldGoalTime(9, 0);
const ros::Time kCancelTime(10, 0);
const ros::Time kNewGoalTime(11, 0);
const ros::Time kNewTrajectoryTime(12, 0);

TEST(TrajectoryCancellationGate, AcceptsNormalGoalAndTrajectoryBeforeCancel)
{
  TrajectoryCancellationGate gate;
  EXPECT_TRUE(gate.noteGoal(kOldGoalTime));
  EXPECT_TRUE(gate.acceptsTrajectory(1U, kOldGoalTime));
}

TEST(TrajectoryCancellationGate, RejectsPreCancelGoal)
{
  TrajectoryCancellationGate gate;
  gate.noteCancel(kCancelTime, true, 7U);
  EXPECT_FALSE(gate.noteGoal(kOldGoalTime));
  EXPECT_TRUE(gate.postCancelGoalTime().isZero());
}

TEST(TrajectoryCancellationGate, RejectsZeroTimestampGoalAfterCancel)
{
  TrajectoryCancellationGate gate;
  gate.noteCancel(kCancelTime, true, 7U);
  EXPECT_FALSE(gate.noteGoal(ros::Time(0)));
  EXPECT_FALSE(gate.acceptsTrajectory(8U, kNewTrajectoryTime));
}

TEST(TrajectoryCancellationGate, RejectsGoalAtOrBeforeCancelTime)
{
  TrajectoryCancellationGate gate;
  gate.noteCancel(kCancelTime, true, 7U);
  EXPECT_FALSE(gate.noteGoal(kCancelTime));
  EXPECT_FALSE(gate.noteGoal(kOldGoalTime));
}

TEST(TrajectoryCancellationGate, AcceptsValidPostCancelGoal)
{
  TrajectoryCancellationGate gate;
  gate.noteCancel(kCancelTime, true, 7U);
  EXPECT_TRUE(gate.noteGoal(kNewGoalTime));
  EXPECT_EQ(kNewGoalTime, gate.postCancelGoalTime());
}

TEST(TrajectoryCancellationGate, AcceptsNewTrajectoryAfterValidGoal)
{
  TrajectoryCancellationGate gate;
  gate.noteCancel(kCancelTime, true, 7U);
  ASSERT_TRUE(gate.noteGoal(kNewGoalTime));
  EXPECT_TRUE(gate.acceptsTrajectory(8U, kNewTrajectoryTime));
}

TEST(TrajectoryCancellationGate, ContinuesRejectingCancelledTrajectory)
{
  TrajectoryCancellationGate gate;
  gate.noteCancel(kCancelTime, true, 7U);
  ASSERT_TRUE(gate.noteGoal(kNewGoalTime));
  ASSERT_TRUE(gate.acceptsTrajectory(8U, kNewTrajectoryTime));
  EXPECT_FALSE(gate.acceptsTrajectory(7U, ros::Time(13, 0)));
  EXPECT_FALSE(gate.acceptsTrajectory(9U, kOldGoalTime));
}

}  // namespace
}  // namespace ego_planner

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
