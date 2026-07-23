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

TEST(Stage3Planner, OrientedBoxUsesYawAndVerticalClearance) {
  StaticObstacle crane;
  crane.id = "crane";
  crane.x = 0.0;
  crane.y = 0.0;
  crane.z_min = 0.0;
  crane.z_max = 10.0;
  crane.half_extent_x = 1.0;
  crane.half_extent_y = 5.0;
  crane.yaw = kPi / 2.0;
  geometry_msgs::Point on_rotated_boom;
  on_rotated_boom.x = 4.0;
  on_rotated_boom.y = 0.0;
  on_rotated_boom.z = 5.0;
  EXPECT_TRUE(pointInObstacle(on_rotated_boom, crane, 0.0));
  on_rotated_boom.z = 12.1;
  EXPECT_FALSE(pointInObstacle(on_rotated_boom, crane, 2.0));
  on_rotated_boom.z = 11.9;
  EXPECT_TRUE(pointInObstacle(on_rotated_boom, crane, 2.0));
}

TEST(Stage3Planner, CraneBoomRejectsOldGateAndKeepsAlternateGate) {
  RouteConfig route;
  route.center_x = -10.0551;
  route.center_y = 19.7104;
  route.radius = 14.0;
  route.start_angle_rad = -67.5 * kPi / 180.0;
  route.tower_collision_radius = 6.41;
  StaticObstacle crane;
  crane.id = "tower_crane";
  crane.x = 2.7535;
  crane.y = 14.7908;
  crane.z_min = 0.0;
  crane.z_max = 35.0283;
  crane.half_extent_x = 3.2732;
  crane.half_extent_y = 19.5424;
  crane.yaw = -0.479608;
  EntryGateConfig config;
  config.inspection_height = 30.0;
  config.minimum_height = 2.0;
  config.maximum_height = 45.0;
  config.minimum_radius = 16.0;
  config.maximum_radius = 24.0;
  geometry_msgs::Point current;
  current.z = 4.0;
  const geometry_msgs::Point home = current;
  CandidatePoint old_gate;
  old_gate.x = -2.40;
  old_gate.y = 1.23;
  old_gate.z = 30.0;
  EXPECT_FALSE(evaluateEntryGateCandidate(
      &old_gate, route, current, home, {}, {crane}, true, config));
  EXPECT_EQ(old_gate.rejection_reason, "KNOWN_OBSTACLE_CLEARANCE");

  CandidatePoint alternate_gate;
  const double alternate_angle = route.start_angle_rad - 40.0 * kPi / 180.0;
  alternate_gate.x =
      route.center_x + 16.0 * std::cos(alternate_angle);
  alternate_gate.y =
      route.center_y + 16.0 * std::sin(alternate_angle);
  alternate_gate.z = 30.0;
  EXPECT_TRUE(evaluateEntryGateCandidate(
      &alternate_gate, route, current, home, {}, {crane}, true, config));
}

TEST(Stage3Planner, InsufficientTowerClearanceIsRejected) {
  Sector sector = makeSector();
  CandidatePoint point = candidate(3.0, 0.0);
  CandidateFilterConfig config;
  EXPECT_FALSE(evaluateCandidate(&point, sector, geometry_msgs::Point(), {}, {},
                                 true, config));
  EXPECT_EQ(point.rejection_reason, "TOWER_KEEP_OUT");
}

TEST(Stage3Planner, BlockedStraightCorridorIsSoftRisk) {
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
  EXPECT_TRUE(evaluateCandidate(&point, sector, current, {cloud}, {}, true,
                                config));
  EXPECT_TRUE(point.straight_corridor_blocked);
  EXPECT_EQ(point.risk_reason, "STRAIGHT_CORRIDOR_BLOCKED");
}

TEST(Stage3Planner, SafeCandidateIsAcceptedAndHysteresisKeepsLock) {
  Sector sector = makeSector();
  geometry_msgs::Point current;
  current.x = 8.0;
  current.y = 0.8;
  current.z = 5.0;
  CandidateFilterConfig config;
  CandidatePoint first = candidate(8.0 * std::cos(0.05),
                                   8.0 * std::sin(0.05));
  first.id = "first";
  CandidatePoint second = candidate(8.0 * std::cos(0.10),
                                    8.0 * std::sin(0.10));
  second.id = "second";
  ASSERT_TRUE(evaluateCandidate(&first, sector, current, {}, {}, true, config));
  ASSERT_TRUE(evaluateCandidate(&second, sector, current, {}, {}, true, config));
  first.score = 0.0;
  second.score = 10.0;
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

TEST(Stage3Planner, NominalTargetWinsWheneverHardChecksAcceptIt) {
  Sector sector = makeSector();
  CandidatePoint nominal = candidate(8.0, 0.0);
  nominal.id = "nominal";
  nominal.accepted = true;
  nominal.score = -100.0;
  CandidatePoint alternate = candidate(10.0, 0.0);
  alternate.id = "alternate";
  alternate.accepted = true;
  alternate.score = 100.0;
  sector.candidates = {alternate, nominal};
  EXPECT_EQ(chooseBestCandidate(sector, nullptr, 0.0), 1);

  sector.candidates[1].accepted = false;
  EXPECT_EQ(chooseBestCandidate(sector, nullptr, 0.0), 0);
}

TEST(Stage3Planner, NearestSectorBecomesFirstWithoutChangingOrbitOrder) {
  RouteConfig route;
  route.center_x = 0.0;
  route.center_y = 0.0;
  route.radius = 8.0;
  route.height = 5.0;
  route.minimum_height = 2.0;
  route.maximum_height = 10.0;
  route.tower_collision_radius = 2.0;
  route.start_angle_rad = 0.0;
  route.direction = OrbitDirection::kCounterClockwise;
  const std::vector<CandidateOffset> offsets{{0.0, 0.0, 0.0}};
  auto sectors = buildInspectionSectors(route, 8, 1, 12.0, 2.0, 1.0,
                                        offsets);
  geometry_msgs::Point current;
  current.x = 0.0;
  current.y = -8.0;
  current.z = 5.0;
  rotateSectorsToNearest(current, &sectors);
  ASSERT_EQ(sectors.size(), 8U);
  EXPECT_EQ(sectors.front().sector_id, 6);
  EXPECT_EQ(sectors[1].sector_id, 7);
  EXPECT_EQ(sectors[2].sector_id, 0);
}

TEST(Stage3Planner, NearestAcceptedSectorSkipsBlockedNearestSector) {
  RouteConfig route;
  route.center_x = 0.0;
  route.center_y = 0.0;
  route.radius = 8.0;
  route.height = 5.0;
  route.minimum_height = 2.0;
  route.maximum_height = 10.0;
  route.tower_collision_radius = 2.0;
  route.start_angle_rad = 0.0;
  route.direction = OrbitDirection::kCounterClockwise;
  const std::vector<CandidateOffset> offsets{{0.0, 0.0, 0.0}};
  auto sectors = buildInspectionSectors(route, 8, 1, 12.0, 2.0, 1.0,
                                        offsets);
  geometry_msgs::Point current;
  current.x = 8.2;
  current.y = 0.0;
  current.z = 5.0;

  // Sector 0 is geometrically nearest but blocked. Sector 1 is the nearest
  // sector that contains a hard-check-accepted target.
  sectors[0].candidates[0].accepted = false;
  sectors[1].candidates[0].accepted = true;
  sectors[2].candidates[0].accepted = true;
  EXPECT_EQ(nearestSectorIndexWithAcceptedCandidate(current, sectors), 1);
}

TEST(Stage3Planner, RecoveryUsesConfiguredHeightAndLockedSafeTarget) {
  RouteConfig route;
  route.center_x = 0.0;
  route.center_y = 0.0;
  route.radius = 8.0;
  route.height = 5.0;
  Sector sector = makeSector();
  geometry_msgs::Point current;
  current.x = 8.0;
  current.z = 5.0;
  CandidatePoint locked = candidate(10.0, 1.0, 6.0);
  locked.id = "safe_p_prime";
  RecoveryConfig config;
  config.recovery_height = 9.0;
  const auto targets = makeRecoveryTargets(route, current, sector, config,
                                           &locked);
  EXPECT_DOUBLE_EQ(targets.r1.z, 9.0);
  EXPECT_DOUBLE_EQ(targets.r2.z, 9.0);
  EXPECT_DOUBLE_EQ(targets.reentry.x, locked.x);
  EXPECT_DOUBLE_EQ(targets.reentry.y, locked.y);
  EXPECT_DOUBLE_EQ(targets.reentry.z, locked.z);
  EXPECT_NE(targets.reentry.id.find(locked.id), std::string::npos);
}

TEST(Stage3Planner, ClockwiseAndCounterClockwiseRecoveryAreBothAssessable) {
  RouteConfig route;
  route.center_x = 0.0;
  route.center_y = 0.0;
  route.radius = 8.0;
  Sector sector = makeSector();
  geometry_msgs::Point current;
  current.x = 8.0;
  current.z = 5.0;
  CandidatePoint locked = candidate(8.0, 0.0);
  RecoveryConfig clockwise;
  clockwise.direction = OrbitDirection::kClockwise;
  RecoveryConfig counter_clockwise = clockwise;
  counter_clockwise.direction = OrbitDirection::kCounterClockwise;
  const auto cw = makeRecoveryTargets(route, current, sector, clockwise,
                                      &locked);
  const auto ccw = makeRecoveryTargets(route, current, sector,
                                       counter_clockwise, &locked);
  EXPECT_LT(cw.r2.y, cw.r1.y);
  EXPECT_GT(ccw.r2.y, ccw.r1.y);
  EXPECT_TRUE(assessRecoveryTargets(current, cw, {}, {}, 1.0, 0.5)
                  .endpoints_safe);
  EXPECT_TRUE(assessRecoveryTargets(current, ccw, {}, {}, 1.0, 0.5)
                  .endpoints_safe);
}

TEST(Stage3Planner, EntryGateCandidatesAreAtInspectionHeightAndChooseNearestSafe) {
  RouteConfig route;
  route.center_x = 0.0;
  route.center_y = 0.0;
  route.radius = 8.0;
  route.start_angle_rad = 0.0;
  route.tower_collision_radius = 2.0;
  const auto candidates =
      buildEntryGateCandidates(route, {-10.0, 0.0, 10.0}, {2.0, 4.0}, 12.0);
  ASSERT_EQ(candidates.size(), 6U);
  for (const auto& candidate : candidates) {
    EXPECT_EQ(candidate.sector_id, -1);
    EXPECT_DOUBLE_EQ(candidate.z, 12.0);
    EXPECT_TRUE(candidate.face_tower);
  }
  geometry_msgs::Point current;
  current.x = 10.0;
  current.z = 4.0;
  geometry_msgs::Point home = current;
  EntryGateConfig config;
  config.inspection_height = 12.0;
  config.minimum_height = 2.0;
  config.maximum_height = 20.0;
  config.minimum_radius = 9.0;
  config.maximum_radius = 20.0;
  std::vector<CandidatePoint> evaluated = candidates;
  for (auto& candidate : evaluated) {
    EXPECT_TRUE(evaluateEntryGateCandidate(
        &candidate, route, current, home, {}, {}, true, config));
  }
  const int best = chooseBestEntryGateCandidate(evaluated);
  ASSERT_GE(best, 0);
  EXPECT_NEAR(evaluated[best].x, 10.0, 1.0e-6);
  EXPECT_NEAR(evaluated[best].y, 0.0, 1.0e-6);
}

TEST(Stage3Planner, RollingEntryGoalsMoveAndClimbTogether) {
  geometry_msgs::Point start;
  start.x = 0.0;
  start.y = 0.0;
  start.z = 4.0;
  CandidatePoint gate;
  gate.id = "ENTRY_GATE_a0";
  gate.x = 12.0;
  gate.y = 0.0;
  gate.z = 30.0;
  gate.require_arrival_yaw = true;
  const auto goals = buildRollingApproachGoals(start, gate, 6.0);
  ASSERT_GT(goals.size(), 1U);
  EXPECT_GT(goals.front().x, start.x);
  EXPECT_GT(goals.front().z, start.z);
  EXPECT_LT(goals.back().x, gate.x + 1.0e-9);
  EXPECT_DOUBLE_EQ(goals.back().z, gate.z);
  EXPECT_FALSE(goals.front().require_arrival_yaw);
  EXPECT_TRUE(goals.back().require_arrival_yaw);
}

}  // namespace
}  // namespace astra_tower_mission

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
