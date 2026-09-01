#include <gtest/gtest.h>

#include <cmath>

#include "bspline_opt/trajectory_collision_checker.h"

class GridMapTestAccess {
public:
  static void initialize(GridMap& map, double resolution = 0.1) {
    map.mp_.resolution_ = resolution;
    map.mp_.resolution_inv_ = 1.0 / resolution;
    map.mp_.map_origin_ = Eigen::Vector3d::Zero();
    map.mp_.map_size_ = Eigen::Vector3d(4.0, 4.0, 4.0);
    map.mp_.map_min_boundary_ = map.mp_.map_origin_;
    map.mp_.map_max_boundary_ = map.mp_.map_origin_ + map.mp_.map_size_;
    map.mp_.clamp_min_log_ = -2.0;
    map.mp_.min_occupancy_log_ = 1.0;
    map.mp_.unknown_flag_ = 0.01;
    for (int axis = 0; axis < 3; ++axis)
      map.mp_.map_voxel_num_(axis) =
          static_cast<int>(std::ceil(map.mp_.map_size_(axis) / resolution));
    map.initializeMapBuffers();
    std::fill(map.md_.occupancy_buffer_.begin(),
              map.md_.occupancy_buffer_.end(), 0.0);
  }

  static void setInflated(GridMap& map, const Eigen::Vector3d& point,
                          bool occupied) {
    Eigen::Vector3i id;
    map.posToIndex(point, id);
    map.md_.occupancy_buffer_inflate_[map.toAddress(id)] = occupied ? 1 : 0;
  }
};

namespace {

ego_planner::UniformBspline makeStraightTrajectory() {
  Eigen::MatrixXd control_points(3, 8);
  for (int index = 0; index < control_points.cols(); ++index)
    control_points.col(index) = Eigen::Vector3d(0.5 + 0.3 * index, 1.0, 1.0);
  return ego_planner::UniformBspline(control_points, 3, 0.2);
}

bool plannerPostCheck(ego_planner::UniformBspline& trajectory, GridMap& map) {
  double start, end;
  trajectory.getTimeSpan(start, end);
  return ego_planner::isUniformBsplineSweptCollisionFree(
      trajectory, map, start, end * 2.0 / 3.0);
}

bool safetyCheck(ego_planner::UniformBspline& trajectory, GridMap& map) {
  double start, end;
  trajectory.getTimeSpan(start, end);
  return ego_planner::isUniformBsplineSweptCollisionFree(
      trajectory, map, start, end * 2.0 / 3.0);
}

TEST(TrajectoryCollisionConsistency, SameSnapshotAlwaysHasSameVerdict) {
  GridMap map;
  GridMapTestAccess::initialize(map);
  auto trajectory = makeStraightTrajectory();
  EXPECT_TRUE(plannerPostCheck(trajectory, map));
  EXPECT_EQ(plannerPostCheck(trajectory, map), safetyCheck(trajectory, map));

  GridMapTestAccess::setInflated(map, {1.4, 1.0, 1.0}, true);
  EXPECT_FALSE(plannerPostCheck(trajectory, map));
  EXPECT_EQ(plannerPostCheck(trajectory, map), safetyCheck(trajectory, map));
}

TEST(TrajectoryCollisionConsistency, SnapshotChangeIsExplicitlyDifferent) {
  GridMap map;
  GridMapTestAccess::initialize(map);
  auto trajectory = makeStraightTrajectory();
  const bool before_update = plannerPostCheck(trajectory, map);
  GridMapTestAccess::setInflated(map, {1.4, 1.0, 1.0}, true);
  const bool after_update = safetyCheck(trajectory, map);
  EXPECT_TRUE(before_update);
  EXPECT_FALSE(after_update);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
