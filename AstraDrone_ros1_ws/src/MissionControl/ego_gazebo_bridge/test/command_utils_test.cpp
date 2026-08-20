#include "ego_gazebo_bridge/command_utils.h"

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

TEST(TimestampValidation, AllowsBoundedCrossProcessClockSkew) {
  const ros::Time now(100, 0);
  EXPECT_TRUE(isTimestampUsable(now, ros::Time(100, 1000000), 0.6, 0.02));
  EXPECT_TRUE(isTimestampUsable(now, ros::Time(99, 500000000), 0.6, 0.02));
  EXPECT_FALSE(isTimestampUsable(now, ros::Time(100, 21000000), 0.6, 0.02));
  EXPECT_FALSE(isTimestampUsable(now, ros::Time(99, 399000000), 0.6, 0.02));
  EXPECT_FALSE(isTimestampUsable(now, ros::Time(0), 0.6, 0.02));
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

TEST(RawCommandConversion, PreservesAllDerivativesAndUsesZeroMask) {
  quadrotor_msgs::PositionCommand command;
  command.header.frame_id = "camera_init";
  command.position.x = 1.0;
  command.position.y = 2.0;
  command.position.z = 3.0;
  command.velocity.x = 2.0;
  command.acceleration.y = 4.0;
  command.yaw = 0.25;
  command.yaw_dot = 1.5;

  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "camera_init";
  transform.transform.translation.x = 10.0;
  transform.transform.rotation = quaternionFromYaw(0.5 * kPi);

  RawCommandLimits limits;
  limits.max_velocity = 1.0;
  limits.max_acceleration = 2.0;
  limits.max_yaw_rate = 0.5;
  mavros_msgs::PositionTarget target;
  std::string reason;
  ASSERT_TRUE(commandToRawTarget(command, transform, limits, ros::Time(5.0),
                                 &target, &reason))
      << reason;
  EXPECT_EQ(mavros_msgs::PositionTarget::FRAME_LOCAL_NED,
            target.coordinate_frame);
  EXPECT_EQ(0u, target.type_mask);
  EXPECT_EQ("map", target.header.frame_id);
  EXPECT_NEAR(8.0, target.position.x, 1e-9);
  EXPECT_NEAR(1.0, target.position.y, 1e-9);
  EXPECT_NEAR(0.0, target.velocity.x, 1e-9);
  EXPECT_NEAR(1.0, target.velocity.y, 1e-9);
  EXPECT_NEAR(-2.0, target.acceleration_or_force.x, 1e-9);
  EXPECT_NEAR(0.25 + 0.5 * kPi, target.yaw, 1e-6);
  EXPECT_NEAR(0.5, target.yaw_rate, 1e-6);
}

TEST(RawCommandConversion, RejectsTiltedWorldFrames) {
  quadrotor_msgs::PositionCommand command;
  command.header.frame_id = "camera_init";
  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "camera_init";
  tf2::Quaternion tilted;
  tilted.setRPY(0.1, 0.0, 0.0);
  transform.transform.rotation = tf2::toMsg(tilted);

  mavros_msgs::PositionTarget target;
  std::string reason;
  EXPECT_FALSE(commandToRawTarget(command, transform, RawCommandLimits(),
                                  ros::Time(1.0), &target, &reason));
  EXPECT_NE(std::string::npos, reason.find("gravity-aligned"));
}

TEST(RawCommandConversion, HighSpeedNormEnvelopeDoesNotClipEgoAxisLimits) {
  quadrotor_msgs::PositionCommand command;
  command.header.frame_id = "camera_init";
  command.velocity.x = 4.0001;
  command.velocity.y = 4.0001;
  command.velocity.z = 4.0001;
  command.acceleration.x = 3.0001;
  command.acceleration.y = 3.0001;
  command.acceleration.z = 3.0001;

  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "camera_init";
  transform.transform.rotation.w = 1.0;

  RawCommandLimits limits;
  limits.max_velocity = std::sqrt(3.0) * 4.0001;
  limits.max_acceleration = std::sqrt(3.0) * 3.0001;
  mavros_msgs::PositionTarget target;
  std::string reason;
  ASSERT_TRUE(commandToRawTarget(command, transform, limits, ros::Time(5.0),
                                 &target, &reason)) << reason;
  EXPECT_NEAR(4.0001, target.velocity.x, 1e-9);
  EXPECT_NEAR(4.0001, target.velocity.y, 1e-9);
  EXPECT_NEAR(4.0001, target.velocity.z, 1e-9);
  EXPECT_NEAR(3.0001, target.acceleration_or_force.x, 1e-9);
  EXPECT_NEAR(3.0001, target.acceleration_or_force.y, 1e-9);
  EXPECT_NEAR(3.0001, target.acceleration_or_force.z, 1e-9);
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

TEST(YawPolicy, FacesFixedPointAndPreservesOrbitYawRate) {
  quadrotor_msgs::PositionCommand command;
  command.position.x = 10.0;
  command.velocity.y = 1.0;
  geometry_msgs::Point tower_center;
  std::string reason;

  ASSERT_TRUE(applyPointFacingYaw(tower_center, 0.0, &command, &reason))
      << reason;
  EXPECT_NEAR(kPi, std::abs(command.yaw), 1e-12);
  EXPECT_NEAR(0.1, command.yaw_dot, 1e-12);

  ASSERT_TRUE(
      applyPointFacingYaw(tower_center, 0.5 * kPi, &command, &reason));
  EXPECT_NEAR(0.5 * kPi, command.yaw, 1e-12);
}

TEST(YawPolicy, RejectsUndefinedTowerCenterBearing) {
  quadrotor_msgs::PositionCommand command;
  geometry_msgs::Point tower_center;
  std::string reason;
  EXPECT_FALSE(applyPointFacingYaw(tower_center, 0.0, &command, &reason));
  EXPECT_FALSE(reason.empty());
}

TEST(YawPolicy, FacesHorizontalVelocityAndComputesYawRate) {
  quadrotor_msgs::PositionCommand command;
  command.velocity.x = 0.0;
  command.velocity.y = 2.0;
  command.acceleration.x = -1.0;
  std::string reason;

  ASSERT_TRUE(applyVelocityFacingYaw(0.05, -0.7, &command, &reason)) << reason;
  EXPECT_NEAR(0.5 * kPi, command.yaw, 1e-12);
  EXPECT_NEAR(0.5, command.yaw_dot, 1e-12);
}

TEST(YawPolicy, HoldsVehicleYawWhenHorizontalVelocityIsTooSmall) {
  quadrotor_msgs::PositionCommand command;
  command.velocity.x = 0.01;
  command.yaw = -1.2;
  command.yaw_dot = 0.4;
  std::string reason;

  ASSERT_TRUE(applyVelocityFacingYaw(0.05, -0.7, &command, &reason)) << reason;
  EXPECT_DOUBLE_EQ(-0.7, command.yaw);
  EXPECT_DOUBLE_EQ(0.0, command.yaw_dot);
}

TEST(YawPolicy, FirstTowerFacingCommandIsRateLimitedFromMeasuredYaw) {
  double limited_yaw = 0.0;
  double limited_yaw_rate = 0.0;
  ASSERT_TRUE(limitYawCommand(
      0.0, 0.5 * kPi, 0.5, 0.02,
      &limited_yaw, &limited_yaw_rate));
  EXPECT_NEAR(0.01, limited_yaw, 1e-12);
  EXPECT_NEAR(0.5, limited_yaw_rate, 1e-12);
}

TEST(YawPolicy, RateLimiterUsesShortestWrappedTurn) {
  double limited_yaw = 0.0;
  double limited_yaw_rate = 0.0;
  ASSERT_TRUE(limitYawCommand(
      170.0 * kPi / 180.0, -170.0 * kPi / 180.0,
      1.0, 0.1, &limited_yaw, &limited_yaw_rate));
  EXPECT_NEAR(0.1, angularDistance(170.0 * kPi / 180.0, limited_yaw),
              1e-12);
  EXPECT_NEAR(1.0, limited_yaw_rate, 1e-12);
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
