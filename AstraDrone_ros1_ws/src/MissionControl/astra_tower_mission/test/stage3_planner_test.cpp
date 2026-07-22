#include "astra_tower_mission/stage3_planner.h"

#include <gtest/gtest.h>

#include <cmath>

namespace astra_tower_mission {
namespace {

Sector makeSector() {
  Sector sector;
  sector.sector_id = 0;
  sector.layer_id = 0;
  sector.center_x = 0.0;
  sector.center_y = 0.0;
  sector.tower_collision_radius = 2.0;
  sector.nominal_angle_rad = 0.0;
  sector.nominal_radius = 8.0;
  sector.nominal_height = 5.0;
  sector.min_angle_rad = -0.5;
  sector.max_angle_rad = 0.5;
  sector.min_radius = 6.0;
  sector.max_radius = 12.0;
  sector.min_height = 3.0;
  sector.max_height = 7.0;
  return sector;
}

CandidatePoint candidate(double x, double y, double z = 5.0) {
  CandidatePoint point;
  point.id = "candidate";
  point.sector_id = 0;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

TEST(Stage3Planner, GoalInsideKnownObstacleIsRejected) {
  Sector sector = makeSector();
  CandidatePoint point = candidate(8.0, 0.0);
  StaticObstacle obstacle{"crane", 8.0, 0.0, 5.0, 1.0, 0.0, 10.0};
  CandidateFilterConfig config;
  config.minimum_clearance = 2.0;
  EXPECT_FALSE(evaluateCandidate(&point, sector, geometry_msgs::Point(), {},
                                 {obstacle}, true, config));
  EXPECT_EQ(point.rejection_reason, "KNOWN_OBSTACLE_CLEARANCE");
}

TEST(Stage3Planner, InsufficientTowerClearanceIsRejected) {
  Sector sector = makeSector();
  CandidatePoint point = candidate(3.0, 0.0);
  CandidateFilterConfig config;
  EXPECT_FALSE(evaluateCandidate(&point, sector, geometry_msgs::Point(), {}, {},
                                 true, config));
  EXPECT_EQ(point.rejection_reason, "TOWER_KEEP_OUT");
}

TEST(Stage3Planner, BlockedLocalCorridorIsRejected) {
  Sector sector = makeSector();
  CandidatePoint point = candidate(8.0, 0.0);
  geometry_msgs::Point current;
  current.x = 8.0;
  current.y = -6.0;
  current.z = 5.0;
  geometry_msgs::Point cloud;
  cloud.x = 8.0;
  cloud.y = -3.0;
  cloud.z = 5.0;
  CandidateFilterConfig config;
  config.minimum_clearance = 1.0;
  config.cloud_inflation = 0.5;
  EXPECT_FALSE(evaluateCandidate(&point, sector, current, {cloud}, {}, true,
                                 config));
  EXPECT_EQ(point.rejection_reason, "NO_LOCAL_CORRIDOR");
}

TEST(Stage3Planner, SafeCandidateIsAcceptedAndHysteresisKeepsLock) {
  Sector sector = makeSector();
  geometry_msgs::Point current;
  current.x = 8.0;
  current.y = 0.8;
  current.z = 5.0;
  CandidateFilterConfig config;
  CandidatePoint first = candidate(8.0, 0.0);
  first.id = "first";
  CandidatePoint second = candidate(8.0, 1.0);
  second.id = "second";
  ASSERT_TRUE(evaluateCandidate(&first, sector, current, {}, {}, true, config));
  ASSERT_TRUE(evaluateCandidate(&second, sector, current, {}, {}, true, config));
  sector.candidates = {first, second};
  EXPECT_EQ(chooseBestCandidate(sector, &first, 100.0), 0);
  EXPECT_EQ(chooseBestCandidate(sector, &first, -100.0), 1);
}

TEST(Stage3Planner, StaleMapRejectsBeforeAnyScoring) {
  Sector sector = makeSector();
  CandidatePoint point = candidate(8.0, 0.0);
  CandidateFilterConfig config;
  EXPECT_FALSE(evaluateCandidate(&point, sector, geometry_msgs::Point(), {}, {},
                                 false, config));
  EXPECT_EQ(point.rejection_reason, "MAP_STALE");
}

TEST(Stage3Planner, RecoveryProducesOutwardTangentAndReentry) {
  RouteConfig route;
  route.tower_name = "tower";
  route.frame_id = "map";
  route.center_x = 0.0;
  route.center_y = 0.0;
  route.radius = 8.0;
  route.height = 5.0;
  route.minimum_height = 2.0;
  route.maximum_height = 10.0;
  route.tower_collision_radius = 2.0;
  route.minimum_safety_distance = 1.0;
  Sector sector = makeSector();
  geometry_msgs::Point current;
  current.x = 8.0;
  current.y = 0.0;
  current.z = 5.0;
  RecoveryConfig config;
  config.radial_step = 3.0;
  config.tangent_step = 4.0;
  config.maximum_radius = 20.0;
  const auto targets = makeRecoveryTargets(route, current, sector, config);
  EXPECT_GT(std::hypot(targets.r1.x, targets.r1.y), 8.0);
  EXPECT_NEAR(targets.r2.x, targets.r1.x, 1e-9);
  EXPECT_GT(std::abs(targets.r2.y - targets.r1.y), 3.9);
  EXPECT_NEAR(std::hypot(targets.reentry.x, targets.reentry.y), 8.0, 1e-9);
}

}  // namespace
}  // namespace astra_tower_mission

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
