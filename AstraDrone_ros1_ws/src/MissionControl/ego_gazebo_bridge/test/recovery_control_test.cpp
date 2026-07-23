#include "ego_gazebo_bridge/recovery_control.h"

#include <gtest/gtest.h>

namespace ego_gazebo_bridge {
namespace {

TEST(TrajectoryGate, CancelRejectsOldTrajectoryUntilNewGoalGeneration) {
  TrajectoryGate gate;
  gate.accept(41U);
  gate.cancel();
  EXPECT_TRUE(gate.active());
  EXPECT_FALSE(gate.allows(41U, ros::Time(11.0), true));
  gate.noteGoal(ros::Time(12.0));
  EXPECT_FALSE(gate.allows(41U, ros::Time(12.1), true));
  EXPECT_FALSE(gate.allows(42U, ros::Time(11.9), true));
  EXPECT_TRUE(gate.allows(42U, ros::Time(12.1), true));
  gate.accept(42U);
  EXPECT_FALSE(gate.active());
}

TEST(TrajectoryGate, CancelReturnTrackChainRequiresFreshTrajectory) {
  TrajectoryGate gate;
  gate.accept(7U);
  gate.cancel();
  gate.noteGoal(ros::Time(20.0));  // return-home goal
  EXPECT_FALSE(gate.allows(7U, ros::Time(20.1), true));
  EXPECT_TRUE(gate.allows(8U, ros::Time(20.1), true));
}

TEST(RecoveryHold, MissionSupervisionSuppressesFiniteLossLanding) {
  EXPECT_FALSE(shouldAutoLandFromHold(true, 30.0, 8.0));
  EXPECT_FALSE(shouldAutoLandFromHold(false, 7.9, 8.0));
  EXPECT_TRUE(shouldAutoLandFromHold(false, 8.0, 8.0));
}

TEST(RecoveryHold, LatchReplacesStaleOutputWithMeasuredVehiclePose) {
  geometry_msgs::PoseStamped measured;
  measured.header.frame_id = "map";
  measured.pose.position.x = 5.69;
  measured.pose.position.y = 7.38;
  measured.pose.position.z = 29.47;
  measured.pose.orientation.w = 1.0;

  const HoldSetpointLatch latch = makeHoldSetpointLatch(measured);
  EXPECT_TRUE(latch.have_output_setpoint);
  EXPECT_DOUBLE_EQ(latch.hold_pose.pose.position.x, 5.69);
  EXPECT_DOUBLE_EQ(latch.hold_pose.pose.position.y, 7.38);
  EXPECT_DOUBLE_EQ(latch.hold_pose.pose.position.z, 29.47);
  EXPECT_DOUBLE_EQ(latch.output_setpoint.pose.position.x,
                   latch.hold_pose.pose.position.x);
  EXPECT_DOUBLE_EQ(latch.output_setpoint.pose.position.y,
                   latch.hold_pose.pose.position.y);
  EXPECT_DOUBLE_EQ(latch.output_setpoint.pose.position.z,
                   latch.hold_pose.pose.position.z);
}

}  // namespace
}  // namespace ego_gazebo_bridge

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
