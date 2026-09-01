#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "plan_env/grid_map.h"

class GridMapTestAccess {
public:
  static void initialize(GridMap& map, double resolution) {
    map.mp_.resolution_ = resolution;
    map.mp_.resolution_inv_ = 1.0 / resolution;
    map.mp_.obstacles_inflation_ = 0.0;
    map.mp_.map_origin_ = Eigen::Vector3d::Zero();
    map.mp_.map_size_ = Eigen::Vector3d::Constant(4.0);
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

  static Eigen::Vector3d center(const GridMap& map,
                                const Eigen::Vector3i& id) {
    return map.mp_.map_origin_ + map.mp_.resolution_ *
        (id.cast<double>() + Eigen::Vector3d::Constant(0.5));
  }

  static void setInflated(GridMap& map, const Eigen::Vector3i& id,
                          bool occupied = true) {
    map.md_.occupancy_buffer_inflate_[map.toAddress(id)] = occupied ? 1 : 0;
  }

  static void setUnknown(GridMap& map, const Eigen::Vector3i& id) {
    map.md_.occupancy_buffer_[map.toAddress(id)] =
        map.mp_.clamp_min_log_ - map.mp_.unknown_flag_;
  }
};

namespace {

TEST(SweptCollisionChecker, EndpointFreeMiddleOccupiedIsRejected) {
  for (const double resolution : {0.1, 0.25}) {
    GridMap map;
    GridMapTestAccess::initialize(map, resolution);
    const Eigen::Vector3i start_id(4, 4, 4);
    const Eigen::Vector3i middle_id(5, 4, 4);
    const Eigen::Vector3i end_id(6, 4, 4);
    GridMapTestAccess::setInflated(map, middle_id);
    EXPECT_FALSE(map.isSweptSegmentFree(
        GridMapTestAccess::center(map, start_id),
        GridMapTestAccess::center(map, end_id)));
  }
}

TEST(SweptCollisionChecker, DiagonalCornerCutIsRejected) {
  GridMap map;
  GridMapTestAccess::initialize(map, 0.25);
  GridMapTestAccess::setInflated(map, {5, 4, 4});
  EXPECT_FALSE(map.isSweptSegmentFree(
      GridMapTestAccess::center(map, {4, 4, 4}),
      GridMapTestAccess::center(map, {5, 5, 4})));
}

TEST(SweptCollisionChecker, FullyFreeDiagonalIsAccepted) {
  for (const double resolution : {0.1, 0.25}) {
    GridMap map;
    GridMapTestAccess::initialize(map, resolution);
    EXPECT_TRUE(map.isSweptSegmentFree(
        GridMapTestAccess::center(map, {4, 4, 4}),
        GridMapTestAccess::center(map, {5, 5, 5})));
  }
}

TEST(SweptCollisionChecker, AllTwentySixNeighborDirectionsAreCovered) {
  for (const double resolution : {0.1, 0.25}) {
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0)
            continue;
          GridMap map;
          GridMapTestAccess::initialize(map, resolution);
          const Eigen::Vector3i start_id(6, 6, 6);
          const Eigen::Vector3i end_id =
              start_id + Eigen::Vector3i(dx, dy, dz);
          EXPECT_TRUE(map.isSweptSegmentFree(
              GridMapTestAccess::center(map, start_id),
              GridMapTestAccess::center(map, end_id)))
              << dx << " " << dy << " " << dz;

          if (std::abs(dx) + std::abs(dy) + std::abs(dz) >= 2) {
            Eigen::Vector3i corner = start_id;
            if (dx != 0)
              corner.x() += dx;
            else
              corner.y() += dy;
            GridMapTestAccess::setInflated(map, corner);
            EXPECT_FALSE(map.isSweptSegmentFree(
                GridMapTestAccess::center(map, start_id),
                GridMapTestAccess::center(map, end_id)))
                << dx << " " << dy << " " << dz;
          }
        }
  }
}

TEST(SweptCollisionChecker, InflationStateControlsVerdict) {
  GridMap map;
  GridMapTestAccess::initialize(map, 0.1);
  const Eigen::Vector3i obstacle(5, 4, 4);
  const Eigen::Vector3d start = GridMapTestAccess::center(map, {4, 4, 4});
  const Eigen::Vector3d end = GridMapTestAccess::center(map, {6, 4, 4});
  EXPECT_TRUE(map.isSweptSegmentFree(start, end));
  GridMapTestAccess::setInflated(map, obstacle, true);
  EXPECT_FALSE(map.isSweptSegmentFree(start, end));
  GridMapTestAccess::setInflated(map, obstacle, false);
  EXPECT_TRUE(map.isSweptSegmentFree(start, end));
}

TEST(SweptCollisionChecker, MapBoundaryBlocksAndUnknownRemainsSearchable) {
  GridMap map;
  GridMapTestAccess::initialize(map, 0.25);
  GridMapTestAccess::setUnknown(map, {5, 4, 4});
  EXPECT_TRUE(map.isSweptSegmentFree(
      GridMapTestAccess::center(map, {4, 4, 4}),
      GridMapTestAccess::center(map, {6, 4, 4})));
  GridMap::SweptCollisionResult result;
  EXPECT_FALSE(map.isSweptSegmentFree(
      GridMapTestAccess::center(map, {4, 4, 4}),
      Eigen::Vector3d(-0.1, 1.0, 1.0), &result));
  EXPECT_TRUE(result.outside_map);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
