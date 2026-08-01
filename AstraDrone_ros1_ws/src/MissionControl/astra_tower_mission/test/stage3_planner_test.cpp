#include "astra_tower_mission/stage3_planner.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

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

TEST(Stage3Planner, LowAltitudeBlockedNominalUsesSameSectorCandidate) {
  RouteConfig route;
  route.center_x = -10.0551;
  route.center_y = 19.7104;
  route.radius = 12.5;
  route.height = 3.0;
  route.minimum_height = 2.0;
  route.maximum_height = 10.0;
  route.tower_collision_radius = 6.41;
  route.minimum_safety_distance = 2.0;
  route.start_angle_rad = 0.0;

  std::vector<CandidateOffset> offsets;
  for (double angle : {0.0, -5.0, 5.0, -10.0, 10.0, -12.0, 12.0}) {
    for (double radius : {0.0, 2.0, 4.0}) {
      offsets.push_back({angle, radius, 0.0});
    }
  }
  auto sectors =
      buildInspectionSectors(route, 8, 1, 12.0, 4.0, 1.0, offsets);
  ASSERT_EQ(sectors.size(), 8U);
  ASSERT_EQ(sectors.front().candidates.size(), 21U);

  geometry_msgs::Point occupied_nominal;
  occupied_nominal.x = route.center_x + route.radius;
  occupied_nominal.y = route.center_y;
  occupied_nominal.z = route.height;
  geometry_msgs::Point current;
  current.x = occupied_nominal.x;
  current.y = occupied_nominal.y - 5.0;
  current.z = route.height;

  CandidateFilterConfig config;
  config.minimum_clearance = 1.0;
  config.cloud_inflation = 0.4;
  config.map_points_are_inflated = true;
  config.map_additional_clearance = 0.5;
  config.unknown_is_hard_constraint = false;
  for (auto& point : sectors.front().candidates) {
    evaluateCandidate(&point, sectors.front(), current, {occupied_nominal}, {},
                      true, config);
  }

  EXPECT_FALSE(sectors.front().candidates.front().accepted);
  EXPECT_EQ(sectors.front().candidates.front().rejection_reason,
            "OCCUPANCY_OR_CLEARANCE");
  const int selected = chooseBestCandidate(sectors.front(), nullptr, 2.0);
  ASSERT_GE(selected, 0);
  const CandidatePoint& replacement = sectors.front().candidates[selected];
  EXPECT_NE(replacement.id, sectors.front().candidates.front().id);
  EXPECT_EQ(replacement.sector_id, sectors.front().sector_id);
  EXPECT_DOUBLE_EQ(replacement.z, 3.0);
  EXPECT_LE(std::abs(normalizeAngle(
                std::atan2(replacement.y - route.center_y,
                           replacement.x - route.center_x) -
                sectors.front().nominal_angle_rad)),
            12.0 * kPi / 180.0 + 1.0e-9);
  EXPECT_GE(std::hypot(replacement.x - route.center_x,
                       replacement.y - route.center_y),
            route.radius - 1.0e-9);
  EXPECT_LE(std::hypot(replacement.x - route.center_x,
                       replacement.y - route.center_y),
            route.radius + 4.0 + 1.0e-9);
}

TEST(Stage3Planner, LevelPathUsesLiveMapAndStaysAtConfiguredAltitude) {
  geometry_msgs::Point start;
  start.x = 0.0;
  start.y = 0.0;
  start.z = 3.0;
  geometry_msgs::Point goal;
  goal.x = 8.0;
  goal.y = 0.0;
  goal.z = 3.0;
  std::vector<geometry_msgs::Point> obstacles;
  for (double y = -1.2; y <= 1.2; y += 0.2) {
    geometry_msgs::Point point;
    point.x = 4.0;
    point.y = y;
    point.z = 3.0;
    obstacles.push_back(point);
  }
  LevelPathConfig config;
  config.additional_clearance = 0.4;
  config.resolution = 0.2;
  config.boundary_margin = 3.0;
  config.maximum_segment_length = 2.0;
  const LevelPathResult result =
      planLevelPath(start, goal, obstacles, config);
  ASSERT_TRUE(result.reachable) << result.reason;
  ASSERT_GE(result.points.size(), 3U);
  EXPECT_GT(result.occupied_cell_count, 0U);
  bool bent_around_obstacle = false;
  for (const auto& point : result.points) {
    EXPECT_DOUBLE_EQ(point.z, 3.0);
    bent_around_obstacle =
        bent_around_obstacle || std::abs(point.y) > 1.6;
  }
  EXPECT_TRUE(bent_around_obstacle);
  EXPECT_NEAR(result.points.front().x, start.x, 1.0e-9);
  EXPECT_NEAR(result.points.back().x, goal.x, 1.0e-9);
}

TEST(Stage3Planner, LevelPathReportsNoHorizontalPassageThroughFullWall) {
  geometry_msgs::Point start;
  start.x = 0.0;
  start.z = 3.0;
  geometry_msgs::Point goal;
  goal.x = 4.0;
  goal.z = 3.0;
  std::vector<geometry_msgs::Point> wall;
  for (double y = -1.2; y <= 1.2; y += 0.1) {
    geometry_msgs::Point point;
    point.x = 2.0;
    point.y = y;
    point.z = 3.0;
    wall.push_back(point);
  }
  LevelPathConfig config;
  config.additional_clearance = 0.2;
  config.resolution = 0.1;
  config.boundary_margin = 1.0;
  const LevelPathResult result = planLevelPath(start, goal, wall, config);
  EXPECT_FALSE(result.reachable);
  EXPECT_EQ(result.reason, "NO_LEVEL_PATH");
}

TEST(Stage3Planner, AlreadyInflatedMapIsNotProjectedAcrossHeightLayers) {
  geometry_msgs::Point start;
  start.z = 3.0;
  geometry_msgs::Point goal;
  goal.x = 4.0;
  goal.z = 3.0;
  std::vector<geometry_msgs::Point> overhead_wall;
  for (double y = -2.0; y <= 2.0; y += 0.1) {
    geometry_msgs::Point point;
    point.x = 2.0;
    point.y = y;
    point.z = 3.5;
    overhead_wall.push_back(point);
  }
  LevelPathConfig config;
  config.vertical_half_extent = 0.2;
  config.additional_clearance = 0.5;
  config.resolution = 0.2;
  config.boundary_margin = 1.0;
  const LevelPathResult result =
      planLevelPath(start, goal, overhead_wall, config);
  ASSERT_TRUE(result.reachable) << result.reason;
  EXPECT_EQ(result.occupied_cell_count, 0U);
  EXPECT_NEAR(result.path_length, 4.0, 1.0e-9);
}

TEST(Stage3Planner, LevelPathPrefersSafeTowerSideAndKeepsExactGoal) {
  geometry_msgs::Point start;
  start.x = 0.0;
  start.y = 0.0;
  start.z = 3.0;
  geometry_msgs::Point goal;
  goal.x = 10.0;
  goal.y = 0.0;
  goal.z = 3.0;
  std::vector<geometry_msgs::Point> obstacle;
  for (double y = -1.2; y <= 1.2; y += 0.1) {
    geometry_msgs::Point point;
    point.x = 5.0;
    point.y = y;
    point.z = 3.0;
    obstacle.push_back(point);
  }
  LevelPathConfig config;
  config.additional_clearance = 0.4;
  config.resolution = 0.2;
  config.boundary_margin = 4.0;
  config.use_tower_constraint = true;
  config.tower_x = 5.0;
  config.tower_y = -10.0;
  config.tower_keep_out_radius = 1.0;
  config.preferred_tower_radius = 8.0;
  config.cost.tower_distance = 1.0;
  const LevelPathResult result =
      planLevelPath(start, goal, obstacle, config);
  ASSERT_TRUE(result.reachable) << result.reason;
  ASSERT_GE(result.points.size(), 3U);
  EXPECT_LT(std::min_element(
                result.points.begin(), result.points.end(),
                [](const geometry_msgs::Point& left,
                   const geometry_msgs::Point& right) {
                  return left.y < right.y;
                })
                ->y,
            -1.4);
  EXPECT_NEAR(result.points.back().x, goal.x, 1.0e-9);
  EXPECT_NEAR(result.points.back().y, goal.y, 1.0e-9);
}

TEST(Stage3Planner, LevelPathHasNoHistoryAcrossMapChanges) {
  geometry_msgs::Point start;
  start.x = 0.0;
  start.z = 3.0;
  geometry_msgs::Point goal;
  goal.x = 8.0;
  goal.z = 3.0;
  geometry_msgs::Point obstacle;
  obstacle.x = 4.0;
  obstacle.z = 3.0;
  LevelPathConfig config;
  config.additional_clearance = 0.6;
  config.resolution = 0.2;
  config.boundary_margin = 2.0;
  const LevelPathResult blocked =
      planLevelPath(start, goal, {obstacle}, config);
  ASSERT_TRUE(blocked.reachable) << blocked.reason;
  const LevelPathResult clear = planLevelPath(start, goal, {}, config);
  ASSERT_TRUE(clear.reachable) << clear.reason;
  ASSERT_GE(clear.points.size(), 2U);
  EXPECT_NEAR(clear.path_length, 8.0, 1.0e-9);
  for (const auto& point : clear.points) {
    EXPECT_NEAR(point.y, 0.0, 1.0e-9);
  }
}

TEST(Stage3Planner, TowerKeepOutIsAHardConstraint) {
  geometry_msgs::Point start;
  start.x = -4.0;
  start.z = 3.0;
  geometry_msgs::Point goal;
  goal.x = 4.0;
  goal.z = 3.0;
  LevelPathConfig config;
  config.additional_clearance = 0.2;
  config.resolution = 0.2;
  config.boundary_margin = 4.0;
  config.use_tower_constraint = true;
  config.tower_keep_out_radius = 2.0;
  config.preferred_tower_radius = 3.0;
  const LevelPathResult result = planLevelPath(start, goal, {}, config);
  ASSERT_TRUE(result.reachable) << result.reason;
  for (const auto& point : result.points) {
    EXPECT_GE(std::hypot(point.x, point.y), 2.0 - config.resolution);
  }
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

TEST(Stage3Planner, BlockedKnownObstacleCorridorIsRejectedWhenHard) {
  Sector sector = makeSector();
  CandidatePoint point = candidate(8.0, 0.0);
  geometry_msgs::Point current;
  current.x = 8.0;
  current.y = -6.0;
  current.z = 5.0;
  StaticObstacle tree;
  tree.id = "tree";
  tree.x = 8.0;
  tree.y = -3.0;
  tree.radius = 0.5;
  tree.z_min = 0.0;
  tree.z_max = 8.0;
  CandidateFilterConfig config;
  config.minimum_clearance = 1.0;
  config.cloud_inflation = 0.5;
  config.known_obstacle_is_hard_constraint = true;
  config.known_obstacle_corridor_is_hard_constraint = true;
  EXPECT_FALSE(evaluateCandidate(&point, sector, current, {}, {tree}, true,
                                 config));
  EXPECT_TRUE(point.straight_corridor_blocked);
  EXPECT_EQ(point.rejection_reason, "KNOWN_OBSTACLE_CORRIDOR");
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

TEST(Stage3Planner, UnknownCoverageIsAHardConstraint) {
  Sector sector = makeSector();
  CandidatePoint point = candidate(8.0, 0.0);
  CandidateFilterConfig config;
  config.unknown_ratio_limit = 0.25;
  EXPECT_FALSE(evaluateCandidate(
      &point, sector, geometry_msgs::Point(), {}, {}, true, config,
      nullptr, 0.30));
  EXPECT_EQ(point.rejection_reason, "UNKNOWN_REGION");
  EXPECT_TRUE(point.target_invalid);
}

TEST(Stage3Planner, UnknownCoverageCanRemainDiagnosticForKnownClearSector) {
  Sector sector = makeSector();
  CandidatePoint point = candidate(8.0, 0.0);
  CandidateFilterConfig config;
  config.unknown_ratio_limit = 0.25;
  config.unknown_is_hard_constraint = false;
  EXPECT_TRUE(evaluateCandidate(
      &point, sector, geometry_msgs::Point(), {}, {}, true, config,
      nullptr, 0.30));
  EXPECT_TRUE(point.accepted);
  EXPECT_FALSE(point.target_invalid);
  EXPECT_EQ(point.risk_reason, "UNKNOWN_REGION_DIAGNOSTIC");
}

TEST(Stage3Planner, WeightedFallbackUsesConfiguredScoreAfterNominalIsBlocked) {
  Sector sector = makeSector();
  CandidateFilterConfig config;
  geometry_msgs::Point current;
  current.x = 8.0;
  current.z = 5.0;

  CandidatePoint small_angle =
      candidate(8.0 * std::cos(5.0 * kPi / 180.0),
                8.0 * std::sin(5.0 * kPi / 180.0), 5.0);
  small_angle.id = "small_angle";
  CandidatePoint small_radius = candidate(10.0, 0.0, 5.0);
  small_radius.id = "small_radius";
  CandidatePoint small_height = candidate(8.0, 0.0, 6.0);
  small_height.id = "small_height";
  ASSERT_TRUE(evaluateCandidate(&small_angle, sector, current, {}, {}, true,
                                config));
  ASSERT_TRUE(evaluateCandidate(&small_radius, sector, current, {}, {}, true,
                                config));
  ASSERT_TRUE(evaluateCandidate(&small_height, sector, current, {}, {}, true,
                                config));
  small_angle.score = -1000.0;
  small_angle.clearance = 2.0;
  small_radius.score = 1000.0;
  small_radius.clearance = 100.0;
  small_height.score = 2000.0;
  small_height.clearance = 200.0;
  sector.candidates = {small_height, small_radius, small_angle};
  EXPECT_EQ(chooseBestCandidate(sector, nullptr, 0.0), 0);
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

TEST(Stage3Planner, EntryGateSelectsNextCounterClockwiseWaypoint) {
  RouteConfig route;
  route.center_x = -10.0551;
  route.center_y = 19.7104;
  route.radius = 12.5;
  route.height = 26.0;
  route.start_angle_rad = 0.0;
  route.direction = OrbitDirection::kCounterClockwise;
  const auto sectors = buildInspectionSectors(
      route, 8, 1, 12.0, 4.0, 3.0,
      std::vector<CandidateOffset>{{0.0, 0.0, 0.0}});
  for (const auto& example :
       std::vector<std::pair<double, int>>{{247.5, 6},
                                           {269.0, 6},
                                           {270.0, 6},
                                           {271.0, 7},
                                           {292.5, 7}}) {
    const auto order = directionalSectorOrder(
        example.first * kPi / 180.0, sectors, route.direction);
    ASSERT_EQ(order.size(), 8U);
    EXPECT_EQ(sectors[order.front()].sector_id, example.second);
  }
}

TEST(Stage3Planner, EntryGateSelectsNextClockwiseWaypoint) {
  RouteConfig route;
  route.center_x = -10.0551;
  route.center_y = 19.7104;
  route.radius = 12.5;
  route.height = 26.0;
  route.start_angle_rad = 0.0;
  route.direction = OrbitDirection::kClockwise;
  const auto sectors = buildInspectionSectors(
      route, 8, 1, 12.0, 4.0, 3.0,
      std::vector<CandidateOffset>{{0.0, 0.0, 0.0}});
  for (const auto& example :
       std::vector<std::pair<double, int>>{{247.5, 5},
                                           {269.0, 5},
                                           {270.0, 6},
                                           {271.0, 6},
                                           {292.5, 6}}) {
    const auto order = directionalSectorOrder(
        example.first * kPi / 180.0, sectors, route.direction);
    ASSERT_EQ(order.size(), 8U);
    EXPECT_EQ(sectors[order.front()].sector_id, example.second);
  }
}

TEST(Stage3Planner, DirectionalReorderClosesOneFullLapAtSelectedWaypoint) {
  for (const OrbitDirection direction :
       {OrbitDirection::kCounterClockwise, OrbitDirection::kClockwise}) {
    RouteConfig route;
    route.radius = 12.5;
    route.height = 26.0;
    route.start_angle_rad = 0.0;
    route.direction = direction;
    const auto sectors = buildInspectionSectors(
        route, 8, 1, 12.0, 4.0, 3.0,
        std::vector<CandidateOffset>{{0.0, 0.0, 0.0}});
    const auto order = directionalSectorOrder(
        -90.0 * kPi / 180.0, sectors, direction);
    const auto visits = buildClosedLapVisitSequence(order.size(), 1);
    ASSERT_EQ(visits.size(), 9U);
    EXPECT_EQ(order[visits.front()], order[visits.back()]);
    std::vector<int> executed_sector_ids;
    for (std::size_t visit : visits) {
      executed_sector_ids.push_back(sectors[order[visit]].sector_id);
    }
    if (direction == OrbitDirection::kCounterClockwise) {
      EXPECT_EQ(executed_sector_ids,
                (std::vector<int>{6, 7, 0, 1, 2, 3, 4, 5, 6}));
    } else {
      EXPECT_EQ(executed_sector_ids,
                (std::vector<int>{6, 5, 4, 3, 2, 1, 0, 7, 6}));
    }
  }
}

TEST(Stage3Planner, ExactEntryWaypointIsNotWrappedBehindRoute) {
  RouteConfig route;
  route.radius = 12.5;
  route.height = 26.0;
  route.start_angle_rad = 0.0;
  route.direction = OrbitDirection::kClockwise;
  const auto sectors = buildInspectionSectors(
      route, 8, 1, 12.0, 4.0, 3.0,
      std::vector<CandidateOffset>{{0.0, 0.0, 0.0}});
  const auto order = directionalSectorOrder(
      270.0 * kPi / 180.0, sectors, route.direction);
  ASSERT_EQ(order.size(), 8U);
  EXPECT_EQ(sectors[order.front()].sector_id, 6);
  EXPECT_NEAR(directedAngularDifference(
                  270.0 * kPi / 180.0,
                  sectors[order.front()].nominal_angle_rad, route.direction),
              0.0, 1.0e-12);
}

TEST(Stage3Planner, DirectionalSelectionWrapsAcrossZeroWithoutReversing) {
  RouteConfig route;
  route.radius = 12.5;
  route.height = 26.0;
  route.start_angle_rad = 0.0;
  const auto sectors = buildInspectionSectors(
      route, 8, 1, 12.0, 4.0, 3.0,
      std::vector<CandidateOffset>{{0.0, 0.0, 0.0}});

  const auto counter_clockwise = directionalSectorOrder(
      350.0 * kPi / 180.0, sectors,
      OrbitDirection::kCounterClockwise);
  ASSERT_EQ(counter_clockwise.size(), 8U);
  EXPECT_EQ(sectors[counter_clockwise.front()].sector_id, 0);
  EXPECT_EQ(sectors[counter_clockwise[1]].sector_id, 1);

  const auto clockwise = directionalSectorOrder(
      10.0 * kPi / 180.0, sectors, OrbitDirection::kClockwise);
  ASSERT_EQ(clockwise.size(), 8U);
  EXPECT_EQ(sectors[clockwise.front()].sector_id, 0);
  EXPECT_EQ(sectors[clockwise[1]].sector_id, 7);
}

TEST(Stage3Planner, EntryToOppositeWaypointCrossesTowerKeepOut) {
  geometry_msgs::Point entry;
  entry.x = 0.0;
  entry.y = -15.0;
  entry.z = 26.0;
  geometry_msgs::Point forward;
  forward.x = 12.5 * std::cos(-67.5 * kPi / 180.0);
  forward.y = 12.5 * std::sin(-67.5 * kPi / 180.0);
  forward.z = 26.0;
  geometry_msgs::Point opposite;
  opposite.x = 12.5 * std::cos(67.5 * kPi / 180.0);
  opposite.y = 12.5 * std::sin(67.5 * kPi / 180.0);
  opposite.z = 26.0;
  StaticObstacle tower;
  tower.id = "tower";
  tower.radius = 6.41;
  tower.z_min = 0.0;
  tower.z_max = 40.0;
  EXPECT_TRUE(lineCorridorSafe(entry, forward, {}, {tower}, 2.4, 0.2));
  EXPECT_FALSE(lineCorridorSafe(entry, opposite, {}, {tower}, 2.4, 0.2));
}

TEST(Stage3Planner, EverySectorEntryToDirectionalFirstWaypointClearsTower) {
  RouteConfig route;
  route.radius = 12.5;
  route.height = 26.0;
  route.start_angle_rad = 0.0;
  const auto sectors = buildInspectionSectors(
      route, 8, 1, 12.0, 4.0, 3.0,
      std::vector<CandidateOffset>{{0.0, 0.0, 0.0}});
  StaticObstacle tower;
  tower.id = "tower";
  tower.radius = 6.41;
  tower.z_min = 0.0;
  tower.z_max = 45.0;

  for (int user_sector = 1; user_sector <= 8; ++user_sector) {
    const double center = entryGateSectorCenterAngleRad(user_sector);
    for (double offset_deg : {-22.5, 0.0, 22.5}) {
      const double gate_angle =
          normalizeAngle(center + offset_deg * kPi / 180.0);
      geometry_msgs::Point entry;
      entry.x = 15.0 * std::cos(gate_angle);
      entry.y = 15.0 * std::sin(gate_angle);
      entry.z = 26.0;
      for (OrbitDirection direction :
           {OrbitDirection::kCounterClockwise,
            OrbitDirection::kClockwise}) {
        const auto order =
            directionalSectorOrder(gate_angle, sectors, direction);
        ASSERT_EQ(order.size(), 8U);
        geometry_msgs::Point first;
        first.x = sectors[order.front()].nominal_radius *
                  std::cos(sectors[order.front()].nominal_angle_rad);
        first.y = sectors[order.front()].nominal_radius *
                  std::sin(sectors[order.front()].nominal_angle_rad);
        first.z = 26.0;
        EXPECT_TRUE(
            lineCorridorSafe(entry, first, {}, {tower}, 2.4, 0.2))
            << "entry_sector=" << user_sector
            << " offset_deg=" << offset_deg;
      }
    }
  }
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

TEST(Stage3Planner, EntryGateCandidatesStayInUserSectorAndUseWeightedPriorities) {
  RouteConfig route;
  route.center_x = 0.0;
  route.center_y = 0.0;
  route.radius = 8.0;
  route.start_angle_rad = -1.2;
  route.direction = OrbitDirection::kClockwise;
  route.tower_collision_radius = 2.0;
  const auto candidates =
      buildEntryGateCandidates(route, 2, 10.0, 12.0, 10.0,
                               7.5, 1.0, 12.0);
  ASSERT_EQ(candidates.size(), 21U);
  for (const auto& candidate : candidates) {
    EXPECT_EQ(candidate.sector_id, 1);
    EXPECT_DOUBLE_EQ(candidate.z, 12.0);
    EXPECT_TRUE(candidate.face_tower);
    EXPECT_TRUE(entryGatePointInSector(candidate, route, 2));
  }
  geometry_msgs::Point current;
  current.x = 10.0;
  current.z = 4.0;
  geometry_msgs::Point home = current;
  EntryGateConfig config;
  config.inspection_height = 12.0;
  config.minimum_height = 2.0;
  config.maximum_height = 20.0;
  config.minimum_radius = 10.0;
  config.maximum_radius = 12.0;
  config.preferred_radius = 10.0;
  CandidatePoint first_inspection;
  const double first_angle = 22.5 * kPi / 180.0;
  first_inspection.x = 8.0 * std::cos(first_angle);
  first_inspection.y = 8.0 * std::sin(first_angle);
  first_inspection.z = 12.0;
  std::vector<CandidatePoint> evaluated = candidates;
  for (auto& candidate : evaluated) {
    EXPECT_TRUE(evaluateEntryGateCandidate(
        &candidate, route, current, home, {}, {}, true, config, 0.0, 1.0,
        &first_inspection));
  }
  const int best = chooseBestEntryGateCandidate(evaluated);
  ASSERT_GE(best, 0);
  EXPECT_NEAR(std::hypot(evaluated[best].x, evaluated[best].y),
              config.preferred_radius, 1.0e-6);
  // Centerline is the first angular preference; the first inspection
  // waypoint breaks otherwise equivalent centerline/radius alternatives.
  EXPECT_NEAR(std::atan2(evaluated[best].y, evaluated[best].x),
              kPi / 4.0, 1.0e-6);

  const int first_best = best;
  evaluated[first_best].accepted = false;
  evaluated[first_best].planner_unreachable = true;
  evaluated[first_best].rejection_reason = "PLANNER_UNREACHABLE";
  const int reachable_fallback = chooseBestEntryGateCandidate(evaluated);
  ASSERT_GE(reachable_fallback, 0);
  EXPECT_NE(reachable_fallback, first_best);
  EXPECT_EQ(evaluated[reachable_fallback].sector_id, 1);
  EXPECT_TRUE(entryGatePointInSector(
      evaluated[reachable_fallback], route, 2));
}

TEST(Stage3Planner, UserEntrySectorNumbersMapCounterClockwiseFromPositiveX) {
  RouteConfig route;
  route.center_x = 2.0;
  route.center_y = -3.0;
  route.start_angle_rad = 1.1;
  route.direction = OrbitDirection::kClockwise;
  for (int user_sector = 1; user_sector <= 8; ++user_sector) {
    const CandidatePoint gate =
        buildFixedEntryGate(route, 8, user_sector, 10.0, 12.0);
    const double expected_angle = (user_sector - 1) * kPi / 4.0;
    ASSERT_FALSE(gate.target_invalid);
    EXPECT_EQ(gate.sector_id, user_sector - 1);
    EXPECT_NEAR(gate.x, route.center_x + 10.0 * std::cos(expected_angle),
                1.0e-9);
    EXPECT_NEAR(gate.y, route.center_y + 10.0 * std::sin(expected_angle),
                1.0e-9);
    EXPECT_TRUE(entryGatePointInSector(gate, route, user_sector));
  }
  EXPECT_TRUE(std::isnan(entryGateSectorCenterAngleRad(0)));
  EXPECT_TRUE(std::isnan(entryGateSectorCenterAngleRad(9)));
}

TEST(Stage3Planner, EntryGateSelectionDoesNotFallbackAcrossSectors) {
  RouteConfig route;
  const auto selected_sector =
      buildEntryGateCandidates(route, 7, 10.0, 12.0, 11.0,
                               7.5, 1.0, 12.0);
  ASSERT_FALSE(selected_sector.empty());
  for (const auto& candidate : selected_sector) {
    EXPECT_EQ(candidate.sector_id, 6);
    EXPECT_TRUE(entryGatePointInSector(candidate, route, 7));
  }
  EXPECT_EQ(chooseBestEntryGateCandidate(selected_sector), -1);
}

TEST(Stage3Planner, EntryGateKeepsFixedRadiusAndPrefersSectorCenter) {
  RouteConfig route;
  route.center_x = -10.0551;
  route.center_y = 19.7104;
  const auto candidates =
      buildEntryGateCandidates(route, 7, 15.0, 15.0, 15.0,
                               7.5, 1.0, 26.0);
  ASSERT_EQ(candidates.size(), 7U);
  for (const auto& candidate : candidates) {
    EXPECT_NEAR(std::hypot(candidate.x - route.center_x,
                           candidate.y - route.center_y),
                15.0, 1.0e-9);
  }

  std::vector<CandidatePoint> evaluated = candidates;
  for (auto& candidate : evaluated) {
    candidate.accepted = true;
    candidate.score = 1000.0 -
                      std::abs(normalizeAngle(
                          std::atan2(candidate.y - route.center_y,
                                     candidate.x - route.center_x) +
                          kPi / 2.0));
  }
  const int centered = chooseBestEntryGateCandidate(evaluated);
  ASSERT_GE(centered, 0);
  EXPECT_NEAR(normalizeAngle(std::atan2(
                  evaluated[centered].y - route.center_y,
                  evaluated[centered].x - route.center_x)),
              -kPi / 2.0, 1.0e-9);

  evaluated[centered].accepted = false;
  const int adjusted = chooseBestEntryGateCandidate(evaluated);
  ASSERT_GE(adjusted, 0);
  EXPECT_NEAR(std::abs(normalizeAngle(
                  std::atan2(evaluated[adjusted].y - route.center_y,
                             evaluated[adjusted].x - route.center_x) +
                  kPi / 2.0)),
              7.5 * kPi / 180.0, 1.0e-9);
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
  EXPECT_FALSE(goals.back().require_arrival_yaw);
  EXPECT_FALSE(goals.front().face_tower);
  EXPECT_FALSE(goals.back().face_tower);
}

TEST(Stage3Planner, SegmentedClimbKeepsConfiguredXYAndLocksTopOnce) {
  CandidatePoint staging;
  staging.x = 3.0;
  staging.y = -4.0;
  staging.z = 4.0;
  const auto goals = buildVerticalClimbGoals(staging, 30.0, 6.0);
  ASSERT_EQ(goals.size(), 5U);
  for (const auto& goal : goals) {
    EXPECT_DOUBLE_EQ(goal.x, staging.x);
    EXPECT_DOUBLE_EQ(goal.y, staging.y);
  }
  EXPECT_DOUBLE_EQ(goals.back().z, 30.0);
  EXPECT_EQ(goals.back().id, "SAFE_ALTITUDE_HOLD");
  EXPECT_FALSE(goals.back().require_arrival_yaw);
}

TEST(Stage3Planner, OneAndMultipleLapsAreExplicitlyClosedAtWaypointOne) {
  const auto one_lap = buildClosedLapVisitSequence(8U, 1);
  const std::vector<std::size_t> expected_one{
      0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 0U};
  EXPECT_EQ(one_lap, expected_one);

  const auto two_laps = buildClosedLapVisitSequence(8U, 2);
  ASSERT_EQ(two_laps.size(), 17U);
  EXPECT_EQ(two_laps.front(), 0U);
  EXPECT_EQ(two_laps[8], 0U);
  EXPECT_EQ(two_laps.back(), 0U);
  EXPECT_EQ(std::count(two_laps.begin(), two_laps.end(), 0U), 3);
}

TEST(Stage3Planner, LandingGetsADeadlineIndependentFromEgoReturn) {
  EXPECT_FALSE(returnOrLandingTimedOut(false, 299.0, 0.0, 300.0, 90.0));
  EXPECT_TRUE(returnOrLandingTimedOut(false, 300.0, 0.0, 300.0, 90.0));

  // A long return may consume nearly all of its own deadline. Once the bridge
  // enters LANDING, only time spent in LANDING is relevant.
  EXPECT_FALSE(returnOrLandingTimedOut(true, 305.0, 5.0, 300.0, 90.0));
  EXPECT_FALSE(returnOrLandingTimedOut(true, 380.0, 89.0, 300.0, 90.0));
  EXPECT_TRUE(returnOrLandingTimedOut(true, 390.0, 90.0, 300.0, 90.0));
}

TEST(Stage3Planner, FourSectorReturnUsesTowerExteriorArc) {
  RouteConfig route;
  route.center_x = -10.0551;
  route.center_y = 19.7104;
  route.tower_collision_radius = 6.41;
  route.minimum_height = 2.0;
  route.maximum_height = 45.0;

  StaticObstacle tower;
  tower.id = "radio_tower";
  tower.x = route.center_x;
  tower.y = route.center_y;
  tower.radius = route.tower_collision_radius;
  tower.z_min = route.minimum_height;
  tower.z_max = route.maximum_height;

  StaticObstacle crane;
  crane.id = "tower_crane";
  crane.x = 2.7535;
  crane.y = 14.7908;
  crane.z_min = 0.0;
  crane.z_max = 35.0283;
  crane.half_extent_x = 3.2732;
  crane.half_extent_y = 19.5424;
  crane.yaw = -0.479608;

  geometry_msgs::Point current;
  current.x = route.center_x + 14.0 * std::cos(157.5 * kPi / 180.0);
  current.y = route.center_y + 14.0 * std::sin(157.5 * kPi / 180.0);
  current.z = 30.0;
  geometry_msgs::Point home;
  home.x = 0.0;
  home.y = 0.0;
  home.z = 4.0;
  EXPECT_FALSE(lineCorridorSafe(current, home, {}, {crane, tower},
                                2.0, 0.5));

  ReturnEgressConfig config;
  const auto goals = buildSafeReturnEgressGoals(
      route, current, home, {crane, tower}, config);
  ASSERT_GT(goals.size(), 2U);
  EXPECT_TRUE(std::any_of(
      goals.begin(), goals.end(), [](const CandidatePoint& goal) {
        return goal.id.find("RETURN_GATE") != std::string::npos;
      }));
  EXPECT_EQ(goals.back().id, "RETURN_HOME_OVERHEAD");
  geometry_msgs::Point from = current;
  for (const auto& goal : goals) {
    geometry_msgs::Point to;
    to.x = goal.x;
    to.y = goal.y;
    to.z = goal.z;
    EXPECT_TRUE(lineCorridorSafe(from, to, {}, {crane, tower},
                                 config.obstacle_inflation,
                                 config.corridor_sample_step));
    from = to;
  }
  EXPECT_DOUBLE_EQ(goals.back().x, home.x);
  EXPECT_DOUBLE_EQ(goals.back().y, home.y);
  EXPECT_DOUBLE_EQ(goals.back().z, config.transit_height);
  const auto gate = std::find_if(
      goals.begin(), goals.end(), [](const CandidatePoint& goal) {
        return goal.id.find("RETURN_GATE") != std::string::npos;
      });
  ASSERT_NE(gate, goals.end());
  const double home_angle =
      std::atan2(home.y - route.center_y, home.x - route.center_x);
  EXPECT_NEAR(gate->x,
              route.center_x + config.orbit_radius * std::cos(home_angle),
              1.0e-9);
  EXPECT_NEAR(gate->y,
              route.center_y + config.orbit_radius * std::sin(home_angle),
              1.0e-9);
}

TEST(Stage3Planner, ReturnDoneRequiresHomeProximity) {
  geometry_msgs::Point home;
  geometry_msgs::Point landed = home;
  landed.x = 0.8;
  landed.y = -0.5;
  EXPECT_TRUE(returnLandingNearHome(landed, home, 1.5));
  landed.x = -10.68;
  landed.y = 16.67;
  EXPECT_FALSE(returnLandingNearHome(landed, home, 1.5));
}

TEST(Stage3Planner, FixedEntryGateSharesConfiguredSectorRadial) {
  RouteConfig route;
  route.center_x = -10.0551;
  route.center_y = 19.7104;
  route.radius = 12.5;
  route.height = 26.0;
  route.start_angle_rad = -67.5 * kPi / 180.0;
  route.direction = OrbitDirection::kCounterClockwise;
  const CandidatePoint gate =
      buildFixedEntryGate(route, 8, 4, 15.0, 26.0);
  ASSERT_FALSE(gate.target_invalid);
  EXPECT_EQ(gate.sector_id, 3);
  EXPECT_NEAR(std::hypot(gate.x - route.center_x,
                         gate.y - route.center_y),
              15.0, 1.0e-9);
  EXPECT_NEAR(std::atan2(gate.y - route.center_y,
                         gate.x - route.center_x),
              135.0 * kPi / 180.0, 1.0e-9);
}

TEST(Stage3Planner, FixedEntryGateAcceptsExactRadiusWithinFloatingTolerance) {
  RouteConfig route;
  route.center_x = -10.0551;
  route.center_y = 19.7104;
  route.radius = 12.5;
  route.start_angle_rad = -67.5 * kPi / 180.0;
  route.direction = OrbitDirection::kCounterClockwise;
  route.minimum_height = 2.0;
  route.maximum_height = 45.0;
  route.tower_collision_radius = 6.41;
  CandidatePoint gate =
      buildFixedEntryGate(route, 8, 3, 15.0, 26.0);
  EntryGateConfig config;
  config.inspection_height = 26.0;
  config.minimum_height = 2.0;
  config.maximum_height = 45.0;
  config.preferred_radius = 15.0;
  config.minimum_radius = 15.0;
  config.maximum_radius = 15.0;
  config.maximum_horizontal_distance = 60.0;
  geometry_msgs::Point current;
  geometry_msgs::Point home;
  EXPECT_TRUE(evaluateEntryGateCandidate(
      &gate, route, current, home, {}, {}, true, config, 0.0, 0.65));
  EXPECT_TRUE(gate.accepted);
}

TEST(Stage3Planner, ExplicitVerticalGoalsKeepXYAndRequestedHeights) {
  CandidatePoint reference;
  reference.x = -4.0;
  reference.y = 8.0;
  reference.z = 4.0;
  const auto ascent = buildVerticalGoalsAtHeights(
      reference, {10.0, 18.0, 26.0}, "ASCENT");
  ASSERT_EQ(ascent.size(), 3U);
  EXPECT_DOUBLE_EQ(ascent[0].z, 10.0);
  EXPECT_DOUBLE_EQ(ascent[1].z, 18.0);
  EXPECT_DOUBLE_EQ(ascent[2].z, 26.0);
  for (const auto& goal : ascent) {
    EXPECT_DOUBLE_EQ(goal.x, reference.x);
    EXPECT_DOUBLE_EQ(goal.y, reference.y);
  }
  reference.z = 26.0;
  const auto descent = buildVerticalGoalsAtHeights(
      reference, {24.0, 22.0}, "TRANSITION");
  ASSERT_EQ(descent.size(), 2U);
  EXPECT_DOUBLE_EQ(descent[0].z, 24.0);
  EXPECT_DOUBLE_EQ(descent[1].z, 22.0);
  for (const auto& goal : descent) {
    EXPECT_DOUBLE_EQ(goal.x, reference.x);
    EXPECT_DOUBLE_EQ(goal.y, reference.y);
  }
}

TEST(Stage3Planner, LayerTransitionIsVerticalOnlyWhenXYMatchesTolerance) {
  CandidatePoint upper;
  upper.x = -10.0;
  upper.y = 7.0;
  upper.z = 26.0;
  upper.layer_id = 0;
  CandidatePoint lower = upper;
  lower.x += 0.05;
  lower.y -= 0.05;
  lower.z = 22.0;
  lower.layer_id = 1;

  const auto vertical = buildLayerTransitionGoals(
      upper, lower, 2.0, 0.10, "TRANSITION");
  ASSERT_EQ(vertical.size(), 2U);
  for (const auto& goal : vertical) {
    EXPECT_DOUBLE_EQ(goal.x, upper.x);
    EXPECT_DOUBLE_EQ(goal.y, upper.y);
  }
  EXPECT_DOUBLE_EQ(vertical[0].z, 24.0);
  EXPECT_DOUBLE_EQ(vertical[1].z, 22.0);

  lower.x = -8.0;
  lower.y = 8.0;
  const auto diagonal = buildLayerTransitionGoals(
      upper, lower, 2.0, 0.10, "TRANSITION");
  ASSERT_EQ(diagonal.size(), 2U);
  EXPECT_DOUBLE_EQ(diagonal[0].x, -9.0);
  EXPECT_DOUBLE_EQ(diagonal[0].y, 7.5);
  EXPECT_DOUBLE_EQ(diagonal[1].x, lower.x);
  EXPECT_DOUBLE_EQ(diagonal[1].y, lower.y);
  EXPECT_DOUBLE_EQ(diagonal[1].z, lower.z);
}

TEST(Stage3Planner, LayerVisitSequencePreservesDataAndSupportsCycles) {
  EXPECT_EQ(buildLayerVisitSequence(2U, 1), (std::vector<int>{0, 1}));
  EXPECT_EQ(buildLayerVisitSequence(2U, 2),
            (std::vector<int>{0, 1, 0, 1}));
  EXPECT_TRUE(buildLayerVisitSequence(0U, 2).empty());
  EXPECT_TRUE(buildLayerVisitSequence(2U, 0).empty());
}

TEST(Stage3Planner, ReturnEgressSkipsCloseRadialGoalAfterRebuild) {
  RouteConfig route;
  route.center_x = 0.0;
  route.center_y = 0.0;
  route.tower_collision_radius = 2.0;
  route.minimum_height = 1.0;
  route.maximum_height = 20.0;
  geometry_msgs::Point current;
  current.x = 12.0;
  current.z = 10.0;
  geometry_msgs::Point home;
  home.x = 0.0;
  home.y = -20.0;
  home.z = 4.0;
  ReturnEgressConfig config;
  config.orbit_radius = 12.0;
  config.transit_height = 10.0;
  config.minimum_goal_separation = 0.5;
  const auto goals =
      buildSafeReturnEgressGoals(route, current, home, {}, config);
  ASSERT_FALSE(goals.empty());
  EXPECT_EQ(goals.front().id.find("_RADIAL"), std::string::npos);
}

TEST(Stage3Planner, CraneOccludedSectorUsesSameSectorAtSafeOverflightHeight) {
  RouteConfig route;
  route.center_x = -10.0551;
  route.center_y = 19.7104;
  route.radius = 14.0;
  route.height = 30.0;
  route.start_angle_rad = -67.5 * kPi / 180.0;
  route.direction = OrbitDirection::kCounterClockwise;
  route.minimum_height = 2.0;
  route.maximum_height = 45.0;
  route.tower_collision_radius = 6.41;

  const std::vector<CandidateOffset> offsets = {
      {0.0, 0.0, 0.0}, {0.0, 0.0, -1.0},
      {0.0, 0.0, 1.0}, {0.0, 0.0, 8.0},
  };
  auto sectors = buildInspectionSectors(
      route, 8, 1, 12.0, 4.0, 8.0, offsets);
  ASSERT_EQ(sectors.size(), 8U);
  Sector& crane_sector = sectors[1];
  ASSERT_EQ(crane_sector.sector_id, 1);

  StaticObstacle crane;
  crane.id = "tower_crane";
  crane.x = 2.7535;
  crane.y = 14.7908;
  crane.z_min = 0.0;
  crane.z_max = 35.0283;
  crane.half_extent_x = 3.2732;
  crane.half_extent_y = 19.5424;
  crane.yaw = -0.479608;
  CandidateFilterConfig config;
  geometry_msgs::Point current;
  current.x = route.center_x;
  current.y = route.center_y - route.radius;
  current.z = route.height;
  for (auto& point : crane_sector.candidates) {
    evaluateCandidate(&point, crane_sector, current, {}, {crane}, true,
                      config);
  }
  EXPECT_FALSE(crane_sector.candidates[0].accepted);
  EXPECT_EQ(crane_sector.candidates[0].rejection_reason,
            "KNOWN_OBSTACLE_CLEARANCE");
  EXPECT_TRUE(crane_sector.candidates[3].accepted);
  EXPECT_DOUBLE_EQ(crane_sector.candidates[3].z, 38.0);
  const int selected = chooseBestCandidate(crane_sector, nullptr, 2.0);
  ASSERT_GE(selected, 0);
  EXPECT_DOUBLE_EQ(crane_sector.candidates[selected].z, 38.0);
  EXPECT_EQ(crane_sector.candidates[selected].sector_id, 1);
}

TEST(Stage3Planner, CoarseCraneBoxCanRemainRiskHintWhenLiveMapIsClear) {
  Sector sector = makeSector();
  CandidatePoint point = candidate(8.0, 0.0);
  StaticObstacle coarse_box;
  coarse_box.id = "crane_obb";
  coarse_box.x = 8.0;
  coarse_box.y = 0.0;
  coarse_box.z_min = 0.0;
  coarse_box.z_max = 10.0;
  coarse_box.half_extent_x = 2.0;
  coarse_box.half_extent_y = 10.0;
  CandidateFilterConfig config;
  config.known_obstacle_is_hard_constraint = false;
  EXPECT_TRUE(evaluateCandidate(&point, sector, geometry_msgs::Point(), {},
                                {coarse_box}, true, config));
  EXPECT_TRUE(point.accepted);
  EXPECT_NE(point.risk_reason.find("COARSE_KNOWN_OBSTACLE_OVERLAP"),
            std::string::npos);
}

TEST(Stage3Planner, LocalDescentBeatsLargeRadialDeviationButNotSmallOne) {
  Sector sector = makeSector();
  geometry_msgs::Point current;
  current.x = 8.0;
  current.z = 5.0;
  CandidateFilterConfig config;

  CandidatePoint radial_two = candidate(10.0, 0.0, 5.0);
  radial_two.id = "radial_two";
  CandidatePoint radial_four = candidate(12.0, 0.0, 5.0);
  radial_four.id = "radial_four";
  CandidatePoint descend_one = candidate(8.0, 0.0, 4.0);
  descend_one.id = "descend_one";
  ASSERT_TRUE(evaluateCandidate(&radial_two, sector, current, {}, {}, true,
                                config));
  ASSERT_TRUE(evaluateCandidate(&radial_four, sector, current, {}, {}, true,
                                config));
  ASSERT_TRUE(evaluateCandidate(&descend_one, sector, current, {}, {}, true,
                                config));

  sector.candidates = {radial_two, descend_one};
  EXPECT_EQ(chooseBestCandidate(sector, nullptr, 0.0), 0);
  sector.candidates = {radial_four, descend_one};
  EXPECT_EQ(chooseBestCandidate(sector, nullptr, 0.0), 1);
}

TEST(Stage3Planner, RecoveryCannotLeaveCurrentSectorOrThreeMetreEnvelope) {
  Sector sector = makeSector();
  RecoveryTargets local;
  local.r1 = candidate(9.0, 0.0, 3.0);
  local.r2 = candidate(9.0 * std::cos(0.2), 9.0 * std::sin(0.2), 3.0);
  local.reentry = candidate(8.0, 0.0, 5.0);
  EXPECT_TRUE(recoveryTargetsStayInSector(local, sector, 3.0));

  RecoveryTargets wrong_sector = local;
  wrong_sector.r2.x = 9.0 * std::cos(0.6);
  wrong_sector.r2.y = 9.0 * std::sin(0.6);
  EXPECT_FALSE(recoveryTargetsStayInSector(wrong_sector, sector, 3.0));

  RecoveryTargets too_low = local;
  too_low.r1.z = 1.9;
  EXPECT_FALSE(recoveryTargetsStayInSector(too_low, sector, 3.0));
}

TEST(Stage3Planner, LayerGateHeightIsRegeneratedForEachInspectionLayer) {
  RouteConfig route;
  route.center_x = -10.0551;
  route.center_y = 19.7104;
  route.radius = 12.5;
  route.start_angle_rad = -67.5 * kPi / 180.0;
  route.direction = OrbitDirection::kCounterClockwise;
  const CandidatePoint upper =
      buildFixedEntryGate(route, 8, 3, 15.0, 26.0);
  const CandidatePoint lower =
      buildFixedEntryGate(route, 8, 3, 15.0, 22.0);
  EXPECT_DOUBLE_EQ(upper.x, lower.x);
  EXPECT_DOUBLE_EQ(upper.y, lower.y);
  EXPECT_NEAR(upper.x, route.center_x, 1.0e-9);
  EXPECT_NEAR(upper.y, route.center_y + 15.0, 1.0e-9);
  EXPECT_EQ(upper.sector_id, 2);
  EXPECT_DOUBLE_EQ(upper.z, 26.0);
  EXPECT_DOUBLE_EQ(lower.z, 22.0);
}

TEST(Stage3Planner, InspectionHeightsComeFromTopAndLayerOffsets) {
  const std::vector<double> baseline =
      deriveInspectionHeights(26.0, {0.0, -4.0});
  ASSERT_EQ(baseline.size(), 2U);
  EXPECT_DOUBLE_EQ(baseline[0], 26.0);
  EXPECT_DOUBLE_EQ(baseline[1], 22.0);

  const std::vector<double> raised =
      deriveInspectionHeights(34.0, {0.0, -4.0});
  ASSERT_EQ(raised.size(), 2U);
  EXPECT_DOUBLE_EQ(raised[0], 34.0);
  EXPECT_DOUBLE_EQ(raised[1], 30.0);
  EXPECT_TRUE(deriveInspectionHeights(
      std::numeric_limits<double>::quiet_NaN(), {0.0, -4.0}).empty());
  EXPECT_TRUE(deriveInspectionHeights(26.0, {}).empty());
  EXPECT_TRUE(deriveInspectionHeights(26.0, {-1.0, -4.0}).empty());
}

TEST(Stage3Planner, BlockedLayerHeightUsesLocalDescentThenRestoresNominal) {
  RouteConfig route;
  route.center_x = 0.0;
  route.center_y = 0.0;
  route.radius = 8.0;
  route.height = 5.0;
  route.minimum_height = 2.0;
  route.maximum_height = 10.0;
  route.tower_collision_radius = 2.0;
  route.direction = OrbitDirection::kCounterClockwise;
  const std::vector<CandidateOffset> offsets = {
      {0.0, 0.0, 0.0}, {0.0, 0.0, -1.0},
      {0.0, 0.0, -2.0}, {0.0, 0.0, -3.0},
      {0.0, 2.0, 0.0}, {0.0, 4.0, 0.0},
  };
  auto sectors = buildInspectionSectors(
      route, 8, 1, 12.0, 4.0, 3.0, offsets);
  ASSERT_EQ(sectors.size(), 8U);
  geometry_msgs::Point current;
  current.x = 8.0;
  current.z = 5.0;
  CandidateFilterConfig config;

  // Occupy every original-height endpoint in the current sector. The same
  // radial/angle location remains clear one metre below.
  std::vector<geometry_msgs::Point> blocked_layer_points;
  for (const auto& point : sectors[0].candidates) {
    if (std::abs(point.z - route.height) < 1.0e-9) {
      geometry_msgs::Point obstacle;
      obstacle.x = point.x;
      obstacle.y = point.y;
      obstacle.z = point.z;
      blocked_layer_points.push_back(obstacle);
    }
  }
  for (auto& point : sectors[0].candidates) {
    evaluateCandidate(&point, sectors[0], current, blocked_layer_points, {},
                      true, config);
  }
  const int lowered =
      chooseBestCandidate(sectors[0], nullptr, 0.0);
  ASSERT_GE(lowered, 0);
  EXPECT_DOUBLE_EQ(sectors[0].candidates[lowered].z, 2.0);
  EXPECT_NEAR(std::hypot(sectors[0].candidates[lowered].x,
                         sectors[0].candidates[lowered].y),
              route.radius, 1.0e-9);

  // Once the next sector is clear, its exact nominal point wins and restores
  // the configured layer height smoothly over the sector-to-sector leg.
  for (auto& point : sectors[1].candidates) {
    evaluateCandidate(&point, sectors[1], current, {}, {}, true, config,
                      &sectors[0].candidates[lowered]);
  }
  const int restored =
      chooseBestCandidate(sectors[1], nullptr, 0.0);
  ASSERT_GE(restored, 0);
  EXPECT_DOUBLE_EQ(sectors[1].candidates[restored].z, route.height);
  EXPECT_NEAR(std::hypot(sectors[1].candidates[restored].x,
                         sectors[1].candidates[restored].y),
              route.radius, 1.0e-9);
}

}  // namespace
}  // namespace astra_tower_mission

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
