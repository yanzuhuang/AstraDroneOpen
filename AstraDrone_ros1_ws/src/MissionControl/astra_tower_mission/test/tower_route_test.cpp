#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "astra_tower_mission/motion_limiter.h"
#include "astra_tower_mission/tower_route.h"

namespace astra_tower_mission {
namespace {

RouteConfig validConfig() {
  RouteConfig config;
  config.tower_name = "radio_tower_0";
  config.frame_id = "map";
  config.center_x = 24.4614;
  config.center_y = 39.3288;
  config.radius = 10.0;
  config.height = 4.0;
  config.waypoint_count = 8;
  config.start_angle_rad = -kPi / 2.0;
  config.direction = OrbitDirection::kCounterClockwise;
  config.camera_yaw_offset_rad = 0.0;
  config.tower_collision_radius = 6.41;
  config.minimum_safety_distance = 2.0;
  config.minimum_height = 2.0;
  config.maximum_height = 10.0;
  return config;
}

TEST(TowerRoute, CounterClockwiseEightPointsAndYawFaceCenter) {
  const RouteConfig config = validConfig();
  const auto points = generateTowerWaypoints(config);
  ASSERT_EQ(8U, points.size());
  EXPECT_NEAR(config.center_x, points[0].x, 1e-9);
  EXPECT_NEAR(config.center_y - config.radius, points[0].y, 1e-9);
  EXPECT_GT(normalizeAngle(points[1].theta - points[0].theta), 0.0);
  for (const auto& point : points) {
    EXPECT_NEAR(config.radius,
                std::hypot(point.x - config.center_x,
                           point.y - config.center_y),
                1e-9);
    const double expected_yaw =
        std::atan2(config.center_y - point.y, config.center_x - point.x);
    EXPECT_NEAR(0.0, normalizeAngle(point.yaw - expected_yaw), 1e-9);
  }
  const double closing_step =
      normalizeAngle(points.front().theta - points.back().theta);
  EXPECT_NEAR(kPi / 4.0, closing_step, 1e-9);
}

TEST(TowerRoute, ClockwiseReversesAngularOrder) {
  RouteConfig config = validConfig();
  config.waypoint_count = 4;
  config.direction = OrbitDirection::kClockwise;
  const auto points = generateTowerWaypoints(config);
  ASSERT_EQ(4U, points.size());
  EXPECT_LT(normalizeAngle(points[1].theta - points[0].theta), 0.0);
  EXPECT_NEAR(config.center_x - config.radius, points[1].x, 1e-9);
  EXPECT_NEAR(config.center_y, points[1].y, 1e-9);
}

TEST(TowerRoute, CameraCompensationAndYawNormalization) {
  RouteConfig config = validConfig();
  config.waypoint_count = 1;
  config.start_angle_rad = kPi - 0.01;
  config.camera_yaw_offset_rad = kPi / 2.0;
  const auto points = generateTowerWaypoints(config);
  ASSERT_EQ(1U, points.size());
  EXPECT_GE(points[0].yaw, -kPi);
  EXPECT_LE(points[0].yaw, kPi);
  const double camera_heading =
      normalizeAngle(points[0].yaw + config.camera_yaw_offset_rad);
  const double tower_heading = std::atan2(
      config.center_y - points[0].y, config.center_x - points[0].x);
  EXPECT_NEAR(0.0, normalizeAngle(camera_heading - tower_heading), 1e-9);
}

TEST(TowerRoute, ParsesDirectionAliasesAndRejectsUnknownValue) {
  OrbitDirection direction = OrbitDirection::kClockwise;
  EXPECT_TRUE(parseDirection("CCW", &direction));
  EXPECT_EQ(OrbitDirection::kCounterClockwise, direction);
  EXPECT_TRUE(parseDirection("clockwise", &direction));
  EXPECT_EQ(OrbitDirection::kClockwise, direction);
  EXPECT_FALSE(parseDirection("left-ish", &direction));
}

TEST(TowerRoute, RejectsIllegalParameters) {
  std::string reason;
  RouteConfig config = validConfig();
  EXPECT_TRUE(validateRouteConfig(config, &reason));

  config.waypoint_count = 0;
  EXPECT_FALSE(validateRouteConfig(config, &reason));
  config = validConfig();
  config.height = 20.0;
  EXPECT_FALSE(validateRouteConfig(config, &reason));
  config = validConfig();
  config.radius = 8.0;
  config.minimum_safety_distance = 2.0;
  config.tower_collision_radius = 6.41;
  EXPECT_FALSE(validateRouteConfig(config, &reason));
  config = validConfig();
  config.center_x = std::nan("");
  EXPECT_FALSE(validateRouteConfig(config, &reason));
}

TEST(MotionLimiter, AppliesSpeedAccelerationAndShortestYawLimits) {
  MotionReference current;
  current.yaw = 179.0 * kPi / 180.0;
  MotionTarget target;
  target.x = 10.0;
  target.yaw = -179.0 * kPi / 180.0;
  const MotionReference next =
      stepMotionReference(current, target, 1.0, 2.0, 0.5, 0.1);
  EXPECT_NEAR(0.5, next.vx, 1e-9);
  EXPECT_NEAR(0.5, next.x, 1e-9);
  EXPECT_GT(normalizeAngle(next.yaw - current.yaw), 0.0);
  EXPECT_LE(std::abs(normalizeAngle(next.yaw - current.yaw)), 0.1 + 1e-9);
}

}  // namespace
}  // namespace astra_tower_mission

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
