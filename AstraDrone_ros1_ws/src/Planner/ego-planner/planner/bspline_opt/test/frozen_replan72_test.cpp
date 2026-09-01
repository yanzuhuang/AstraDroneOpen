#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#include "bspline_opt/bspline_optimizer.h"
#include "bspline_opt/short_segment_guidance.h"
#include "bspline_opt/trajectory_collision_checker.h"

#ifndef FROZEN_REPLAN72_DATA
#error "FROZEN_REPLAN72_DATA must point to the frozen occupancy CSV"
#endif
#ifndef FROZEN_REPLAN55_DATA
#error "FROZEN_REPLAN55_DATA must point to the frozen occupancy CSV"
#endif
#ifndef FROZEN_REPLAN69_DATA
#error "FROZEN_REPLAN69_DATA must point to the frozen occupancy CSV"
#endif

class GridMapTestAccess {
public:
  static GridMap::Ptr loadFrozen(const std::string& path,
                                 std::vector<Eigen::Vector3d>& occupied) {
    GridMap::Ptr map(new GridMap);
    map->mp_.resolution_ = 0.1;
    map->mp_.resolution_inv_ = 10.0;
    map->mp_.map_origin_ = Eigen::Vector3d(20.0, -5.0, 1.0);
    map->mp_.map_size_ = Eigen::Vector3d(20.0, 10.0, 3.0);
    map->mp_.map_min_boundary_ = map->mp_.map_origin_;
    map->mp_.map_max_boundary_ = map->mp_.map_origin_ + map->mp_.map_size_;
    map->mp_.clamp_min_log_ = -2.0;
    map->mp_.min_occupancy_log_ = 1.0;
    map->mp_.unknown_flag_ = 0.01;
    map->mp_.map_voxel_num_ = Eigen::Vector3i(200, 100, 30);
    map->initializeMapBuffers();
    std::fill(map->md_.occupancy_buffer_.begin(),
              map->md_.occupancy_buffer_.end(), 0.0);

    std::ifstream stream(path);
    EXPECT_TRUE(stream.good());
    std::string line;
    std::getline(stream, line);
    while (std::getline(stream, line)) {
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream row(line);
      Eigen::Vector3d point;
      if (!(row >> point.x() >> point.y() >> point.z()))
        continue;
      if (!map->isInMap(point))
        continue;
      Eigen::Vector3i id;
      map->posToIndex(point, id);
      map->md_.occupancy_buffer_inflate_[map->toAddress(id)] = 1;
      occupied.push_back(point);
    }
    return map;
  }

  static GridMap::Ptr loadReplan72(std::vector<Eigen::Vector3d>& occupied) {
    return loadFrozen(FROZEN_REPLAN72_DATA, occupied);
  }
};

namespace ego_planner {

class BsplineOptimizerTestAccess {
public:
  static void configure(BsplineOptimizer& optimizer, const GridMap::Ptr& map,
                        SwarmTrajData& empty_swarm) {
    optimizer.grid_map_ = map;
    optimizer.lambda1_ = 1.0;
    optimizer.lambda2_ = 0.5;
    optimizer.lambda3_ = 0.1;
    optimizer.lambda4_ = 1.0;
    optimizer.dist0_ = 0.5;
    optimizer.swarm_clearance_ = 1.5;
    optimizer.max_vel_ = 0.75;
    optimizer.max_acc_ = 0.8;
    optimizer.order_ = 3;
    optimizer.drone_id_ = 0;
    optimizer.swarm_trajs_ = &empty_swarm;
    optimizer.local_target_pt_ = Eigen::Vector3d(34.859909, -0.000056, 3.004797);
    optimizer.a_star_.reset(new AStar);
    optimizer.a_star_->initGridMap(map, Eigen::Vector3i(100, 100, 100));
  }
};

}  // namespace ego_planner

namespace {

Eigen::MatrixXd frozenControlPoints() {
  const double values[][3] = {
      {28.761292, -1.135844, 2.537161},
      {29.156949, -1.067315, 2.634937},
      {29.566398, -0.929099, 2.728702},
      {29.898084, -0.803345, 2.793707},
      {30.281439, -0.648640, 2.856534},
      {30.643644, -0.503026, 2.903575},
      {31.020887, -0.367523, 2.941399},
      {31.402068, -0.251819, 2.967528},
      {31.791228, -0.156051, 2.985375},
      {32.182006, -0.082228, 2.996577},
      {32.587019, -0.036448, 3.002199},
      {32.964875, -0.012396, 3.004366},
      {33.422710, -0.002459, 3.004892},
      {33.690041, -0.000162, 3.005059},
      {34.273577, -0.000104, 3.004930},
      {34.859909, -0.000056, 3.004797},
  };
  Eigen::MatrixXd points(3, 16);
  for (int index = 0; index < 16; ++index)
    points.col(index) = Eigen::Vector3d(
        values[index][0], values[index][1], values[index][2]);
  return points;
}

std::vector<Eigen::Vector3d> oldAStarPath() {
  return {
      {29.432241, -0.966222, 2.761205},
      {29.532241, -0.966222, 2.761205},
      {29.632241, -0.966222, 2.761205},
      {29.732241, -0.866222, 2.761205},
      {29.832241, -0.766222, 2.761205},
      {29.932241, -0.766222, 2.761205},
  };
}

double maxAbsY(const std::vector<Eigen::Vector3d>& path) {
  double value = 0.0;
  for (const auto& point : path)
    value = std::max(value, std::abs(point.y()));
  return value;
}

double minimumInflatedCenterLInf(
    ego_planner::UniformBspline& trajectory,
    const std::vector<Eigen::Vector3d>& occupied) {
  double start, end;
  trajectory.getTimeSpan(start, end);
  double minimum = std::numeric_limits<double>::infinity();
  for (double parameter = start; parameter <= end * 2.0 / 3.0 + 1.0e-12;
       parameter += 0.01) {
    const Eigen::Vector3d point = trajectory.evaluateDeBoorT(
        std::min(parameter, end * 2.0 / 3.0));
    for (const auto& voxel : occupied)
      minimum = std::min(minimum, (point - voxel).cwiseAbs().maxCoeff());
  }
  return minimum;
}

TEST(FrozenReplan72, BaselineControlReproducesDirectionLoss) {
  const Eigen::MatrixXd control_points = frozenControlPoints();
  const Eigen::Vector3d midpoint =
      0.5 * (control_points.col(2) + control_points.col(3));
  const std::vector<Eigen::Vector3d> old_path = oldAStarPath();
  // The legacy short-branch intersection is essentially the midpoint itself;
  // its fixed 1 cm gate therefore discarded all guidance.
  EXPECT_LT((old_path[3] - midpoint).norm(), 0.01);
}

TEST(FrozenReplan72, CandidateAStarIsDeterministicAndSweptFree) {
  std::vector<Eigen::Vector3d> occupied;
  GridMap::Ptr map = GridMapTestAccess::loadReplan72(occupied);
  EXPECT_FALSE(map->isSweptPolylineFree(oldAStarPath()));

  std::vector<Eigen::Vector3d> reference_path;
  std::size_t reference_nodes = 0;
  std::size_t reference_rejections = 0;
  for (int repetition = 0; repetition < 5; ++repetition) {
    AStar astar;
    astar.initGridMap(map, {100, 100, 100});
    ASSERT_TRUE(astar.AstarSearch(
        0.1, {29.566398, -0.929099, 2.728702},
        {29.898084, -0.803345, 2.793707}));
    const auto path = astar.getPath();
    ASSERT_TRUE(map->isSweptPolylineFree(path));
    EXPECT_GT(maxAbsY(path), 0.9);
    EXPECT_GT(astar.getLastSweptCollisionCount(), 0u);
    if (repetition == 0) {
      reference_path = path;
      reference_nodes = astar.getLastExpandedNodeCount();
      reference_rejections = astar.getLastSweptCollisionCount();
      std::cout << "FROZEN_REPLAN72_ASTAR path_count=" << path.size()
                << " node_count=" << reference_nodes
                << " swept_collision_count=" << reference_rejections
                << " max_abs_y=" << maxAbsY(path) << " path=";
      for (const auto& point : path)
        std::cout << point.transpose() << ";";
      std::cout << std::endl;
    } else {
      ASSERT_EQ(path.size(), reference_path.size());
      for (std::size_t index = 0; index < path.size(); ++index)
        EXPECT_TRUE(path[index].isApprox(reference_path[index], 0.0));
      EXPECT_EQ(astar.getLastExpandedNodeCount(), reference_nodes);
      EXPECT_EQ(astar.getLastSweptCollisionCount(), reference_rejections);
    }
  }
}

TEST(FrozenReplan72, GuidanceGradientAndOptimizerMovementAreNonZero) {
  std::vector<Eigen::Vector3d> occupied;
  GridMap::Ptr map = GridMapTestAccess::loadReplan72(occupied);
  ego_planner::BsplineOptimizer optimizer;
  ego_planner::SwarmTrajData empty_swarm;
  ego_planner::BsplineOptimizerTestAccess::configure(
      optimizer, map, empty_swarm);

  Eigen::MatrixXd control_points = frozenControlPoints();
  const auto segments = optimizer.initControlPoints(control_points, true);
  ASSERT_FALSE(segments.empty());
  const ego_planner::ControlPoints guided = optimizer.getControlPoints();
  ASSERT_FALSE(guided.direction[2].empty());
  ASSERT_FALSE(guided.direction[3].empty());
  EXPECT_NEAR(guided.direction[2][0].norm(), 1.0, 1.0e-9);
  const double distance =
      (guided.points.col(3) - guided.base_point[3][0]).dot(
          guided.direction[3][0]);
  const double distance_error = guided.clearance - distance;
  const Eigen::Vector3d collision_gradient =
      -0.5 * 3.0 * distance_error * distance_error *
      guided.direction[3][0];
  EXPECT_GT(collision_gradient.norm(), 1.0e-6);

  ego_planner::UniformBspline before(control_points, 3, 0.8);
  const double clearance_before = minimumInflatedCenterLInf(before, occupied);
  Eigen::MatrixXd optimized;
  const bool optimizer_success =
      optimizer.BsplineOptimizeTrajRebound(optimized, 0.8);
  EXPECT_GT((optimized.col(3) - control_points.col(3)).norm(), 1.0e-6);
  ego_planner::UniformBspline after(optimized, 3, 0.8);
  const double clearance_after = minimumInflatedCenterLInf(after, occupied);
  EXPECT_GT(clearance_after, clearance_before);

  double start, end;
  after.getTimeSpan(start, end);
  ego_planner::TrajectoryCollisionResult planner_result;
  ego_planner::TrajectoryCollisionResult safety_result;
  const bool planner_free = ego_planner::isUniformBsplineSweptCollisionFree(
      after, *map, start, end * 2.0 / 3.0, &planner_result);
  const bool safety_free = ego_planner::isUniformBsplineSweptCollisionFree(
      after, *map, start, end * 2.0 / 3.0, &safety_result);
  EXPECT_EQ(planner_free, safety_free);
  EXPECT_EQ(optimizer_success, planner_free);
  std::cout << "FROZEN_REPLAN72_OPTIMIZER success=" << optimizer_success
            << " direction=" << guided.direction[2][0].transpose()
            << " gradient_norm=" << collision_gradient.norm()
            << " cp3_delta="
            << (optimized.col(3) - control_points.col(3)).transpose()
            << " clearance_before=" << clearance_before
            << " clearance_after=" << clearance_after
            << " planner_free=" << planner_free
            << " safety_free=" << safety_free << std::endl;
}

TEST(FrozenPlannerRegression, Replan55LargeLateralDetourRemainsAvailable) {
  std::vector<Eigen::Vector3d> occupied;
  GridMap::Ptr map = GridMapTestAccess::loadFrozen(
      FROZEN_REPLAN55_DATA, occupied);
  const std::vector<Eigen::Vector3d> old_path = {
      {28.214, -0.638, 2.467}, {28.314, -0.738, 2.367},
      {28.414, -0.838, 2.367}, {28.514, -0.838, 2.367},
      {28.614, -0.838, 2.367}, {28.714, -0.838, 2.367},
      {28.814, -0.838, 2.367}, {28.914, -0.838, 2.367},
      {29.014, -0.838, 2.367}, {29.114, -0.838, 2.367},
  };
  ASSERT_TRUE(map->isSweptPolylineFree(old_path));
  AStar astar;
  astar.initGridMap(map, {100, 100, 100});
  ASSERT_TRUE(astar.AstarSearch(
      0.1, {28.141, -0.630, 2.427}, {28.888, -0.846, 2.308}));
  const auto candidate = astar.getPath();
  EXPECT_TRUE(map->isSweptPolylineFree(candidate));
  EXPECT_GE(maxAbsY(candidate), 0.7);
}

TEST(FrozenPlannerRegression, Replan69ShortSegmentGuidanceRemainsValid) {
  std::vector<Eigen::Vector3d> occupied;
  GridMap::Ptr map = GridMapTestAccess::loadFrozen(
      FROZEN_REPLAN69_DATA, occupied);
  const std::vector<Eigen::Vector3d> old_path = {
      {29.419, -0.979, 2.754}, {29.519, -0.979, 2.754},
      {29.619, -0.979, 2.754}, {29.719, -0.879, 2.754},
      {29.819, -0.779, 2.754}, {29.919, -0.779, 2.754},
  };
  ASSERT_TRUE(map->isSweptPolylineFree(old_path));
  ego_planner::ShortSegmentGuidance old_guidance;
  ASSERT_TRUE(ego_planner::computeShortSegmentGuidance(
      {29.542179, -0.941494, 2.724300},
      {29.853833, -0.815634, 2.783777}, old_path, old_guidance));
  EXPECT_LT(old_guidance.direction.y(), -0.9);
  AStar astar;
  astar.initGridMap(map, {100, 100, 100});
  ASSERT_TRUE(astar.AstarSearch(
      0.1, {29.542179, -0.941494, 2.724300},
      {29.853833, -0.815634, 2.783777}));
  const auto candidate = astar.getPath();
  ASSERT_TRUE(map->isSweptPolylineFree(candidate));
  ego_planner::ShortSegmentGuidance guidance;
  ASSERT_TRUE(ego_planner::computeShortSegmentGuidance(
      {29.542179, -0.941494, 2.724300},
      {29.853833, -0.815634, 2.783777}, candidate, guidance));
  EXPECT_NEAR(guidance.direction.norm(), 1.0, 1.0e-9);
  EXPECT_LT(guidance.direction.y(), -0.5);
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "frozen_replan72_test",
            ros::init_options::AnonymousName |
                ros::init_options::NoSigintHandler);
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
