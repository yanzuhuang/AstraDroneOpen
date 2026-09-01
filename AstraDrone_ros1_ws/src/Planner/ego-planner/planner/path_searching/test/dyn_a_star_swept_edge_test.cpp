#include <gtest/gtest.h>

#include <cmath>

#include "path_searching/dyn_a_star.h"

class GridMapTestAccess {
public:
  static GridMap::Ptr makeMap() {
    GridMap::Ptr map(new GridMap);
    map->mp_.resolution_ = 0.1;
    map->mp_.resolution_inv_ = 10.0;
    map->mp_.map_origin_ = Eigen::Vector3d::Zero();
    map->mp_.map_size_ = Eigen::Vector3d::Constant(3.0);
    map->mp_.map_min_boundary_ = map->mp_.map_origin_;
    map->mp_.map_max_boundary_ = map->mp_.map_origin_ + map->mp_.map_size_;
    map->mp_.clamp_min_log_ = -2.0;
    map->mp_.min_occupancy_log_ = 1.0;
    map->mp_.unknown_flag_ = 0.01;
    map->mp_.map_voxel_num_ = Eigen::Vector3i::Constant(30);
    map->initializeMapBuffers();
    std::fill(map->md_.occupancy_buffer_.begin(),
              map->md_.occupancy_buffer_.end(), 0.0);
    return map;
  }

  static Eigen::Vector3d center(const Eigen::Vector3i& id) {
    return 0.1 * (id.cast<double>() + Eigen::Vector3d::Constant(0.5));
  }

  static void block(GridMap& map, const Eigen::Vector3i& id) {
    map.md_.occupancy_buffer_inflate_[map.toAddress(id)] = 1;
  }
};

TEST(DynamicAStarSweptEdge, RejectsCornerCutAndReturnsOnlySweptFreeEdges) {
  GridMap::Ptr map = GridMapTestAccess::makeMap();
  const Eigen::Vector3i start_id(10, 10, 10);
  const Eigen::Vector3i direct_end_id(11, 11, 10);
  GridMapTestAccess::block(*map, {11, 10, 10});
  EXPECT_FALSE(map->isSweptSegmentFree(
      GridMapTestAccess::center(start_id),
      GridMapTestAccess::center(direct_end_id)));

  AStar astar;
  astar.initGridMap(map, {30, 30, 30});
  ASSERT_TRUE(astar.AstarSearch(
      0.1, GridMapTestAccess::center(start_id),
      GridMapTestAccess::center({13, 13, 10})));
  const std::vector<Eigen::Vector3d> path = astar.getPath();
  ASSERT_GE(path.size(), 2u);
  EXPECT_GT(astar.getLastSweptCollisionCount(), 0u);
  EXPECT_TRUE(map->isSweptPolylineFree(path));
}

int main(int argc, char** argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
