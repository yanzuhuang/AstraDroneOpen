#include "ego_gazebo_bridge/command_utils.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

namespace ego_gazebo_bridge {
namespace {

constexpr double kPi = 3.14159265358979323846;

geometry_msgs::PoseStamped makePose(double x, double y, double z,
                                    double yaw,
                                    const std::string& frame = "map") {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation = quaternionFromYaw(yaw);
  return pose;
}

TEST(CommandValidation, RejectsNonFiniteInput) {
  quadrotor_msgs::PositionCommand command;
  EXPECT_TRUE(isFinitePositionCommand(command));

  command.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(isFinitePositionCommand(command));
}

TEST(CommandConversion, PreservesFrameAndYaw) {
  quadrotor_msgs::PositionCommand command;
  command.header.frame_id = "camera_init";
  command.position.x = 1.0;
  command.position.y = -2.0;
  command.position.z = 0.8;
  command.yaw = 0.7;

  const auto pose = commandToPose(command, "fallback");
  EXPECT_EQ("camera_init", pose.header.frame_id);
  EXPECT_DOUBLE_EQ(1.0, pose.pose.position.x);
  EXPECT_NEAR(0.7, yawFromQuaternion(pose.pose.orientation), 1e-12);
}

TEST(FrameTransform, AppliesRotationBeforeTranslation) {
  const auto input = makePose(1.0, 0.0, 0.0, 0.0, "camera_init");
  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "camera_init";
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.rotation = quaternionFromYaw(0.5 * kPi);

  const auto output = transformPose(input, transform);
  EXPECT_EQ("map", output.header.frame_id);
  EXPECT_NEAR(1.0, output.pose.position.x, 1e-9);
  EXPECT_NEAR(3.0, output.pose.position.y, 1e-9);
  EXPECT_NEAR(0.5 * kPi, yawFromQuaternion(output.pose.orientation), 1e-9);
}

TEST(CommandBoundsTest, EnforcesHeightAndHorizontalEnvelope) {
  const auto home = makePose(0.0, 0.0, 0.0, 0.0);
  CommandBounds bounds;
  bounds.min_relative_height = 0.3;
  bounds.max_relative_height = 2.0;
  bounds.max_horizontal_radius = 5.0;
  std::string reason;

  EXPECT_TRUE(isWithinBounds(makePose(3.0, 4.0, 1.0, 0.0), home,
                             bounds, &reason));
  EXPECT_FALSE(isWithinBounds(makePose(0.0, 0.0, 0.2, 0.0), home,
                              bounds, &reason));
  EXPECT_FALSE(isWithinBounds(makePose(5.1, 0.0, 1.0, 0.0), home,
                              bounds, &reason));
}

TEST(CommandBoundsTest, ReportsHorizontalDistance) {
  const auto home = makePose(-1.0, 2.0, 0.0, 0.0);
  const auto target = makePose(2.0, 6.0, 10.0, 0.0);
  EXPECT_NEAR(5.0, horizontalDistance(home, target), 1e-12);
}

TEST(CommandLimiter, LimitsPositionAndShortestYawStep) {
  const auto current = makePose(0.0, 0.0, 0.0, 170.0 * kPi / 180.0);
  const auto target = makePose(3.0, 4.0, 0.0, -170.0 * kPi / 180.0);

  const auto next = stepToward(current, target, 1.0, 0.1);
  EXPECT_NEAR(0.6, next.pose.position.x, 1e-12);
  EXPECT_NEAR(0.8, next.pose.position.y, 1e-12);
  EXPECT_NEAR(0.1,
              angularDistance(yawFromQuaternion(current.pose.orientation),
                              yawFromQuaternion(next.pose.orientation)),
              1e-12);
}

}  // namespace
}  // namespace ego_gazebo_bridge

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
