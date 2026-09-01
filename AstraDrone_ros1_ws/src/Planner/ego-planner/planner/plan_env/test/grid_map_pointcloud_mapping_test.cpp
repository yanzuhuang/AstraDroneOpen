#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>
#include <vector>

#include "plan_env/grid_map.h"

class GridMapTestAccess {
public:
  static void initialize(
      GridMap& map, double resolution = 0.2, double inflation = 0.4,
      const Eigen::Vector3d& map_origin = Eigen::Vector3d(0.0, -10.0, 0.0),
      const Eigen::Vector3d& map_size = Eigen::Vector3d(30.0, 20.0, 10.0),
      const Eigen::Vector3d& local_range = Eigen::Vector3d(5.5, 5.5, 4.5),
      double max_ray_length = 8.0) {
    map.mp_.resolution_ = resolution;
    map.mp_.resolution_inv_ = 1.0 / resolution;
    map.mp_.obstacles_inflation_ = inflation;
    map.mp_.map_origin_ = map_origin;
    map.mp_.map_size_ = map_size;
    map.mp_.map_min_boundary_ = map_origin;
    map.mp_.map_max_boundary_ = map_origin + map_size;
    map.mp_.local_update_range_ = local_range;
    map.mp_.local_map_margin_ = 2;
    map.mp_.ground_height_ = map_origin.z();
    map.mp_.virtual_ceil_height_ = -1.0;
    map.mp_.visualization_truncate_height_ = map.mp_.map_max_boundary_.z();
    map.mp_.frame_id_ = "world";
    map.mp_.p_hit_ = 0.65;
    map.mp_.p_miss_ = 0.35;
    map.mp_.p_min_ = 0.12;
    map.mp_.p_max_ = 0.90;
    map.mp_.p_occ_ = 0.80;
    map.mp_.prob_hit_log_ = std::log(map.mp_.p_hit_ / (1.0 - map.mp_.p_hit_));
    map.mp_.prob_miss_log_ = std::log(map.mp_.p_miss_ / (1.0 - map.mp_.p_miss_));
    map.mp_.clamp_min_log_ = std::log(map.mp_.p_min_ / (1.0 - map.mp_.p_min_));
    map.mp_.clamp_max_log_ = std::log(map.mp_.p_max_ / (1.0 - map.mp_.p_max_));
    map.mp_.min_occupancy_log_ = std::log(map.mp_.p_occ_ / (1.0 - map.mp_.p_occ_));
    map.mp_.unknown_flag_ = 0.01;
    map.mp_.min_ray_length_ = 0.1;
    map.mp_.max_ray_length_ = max_ray_length;
    map.mp_.cloud_odom_timeout_ = 0.15;
    for (int axis = 0; axis < 3; ++axis)
      map.mp_.map_voxel_num_(axis) =
          static_cast<int>(std::ceil(map_size(axis) / resolution));
    map.initializeMapBuffers();
    map.md_.has_odom_ = true;
  }

  static void process(
      GridMap& map, const std::vector<Eigen::Vector3d>& points,
      const Eigen::Vector3d& origin) {
    map.processPointCloud(points, origin);
  }

  static void clear(GridMap& map) { map.clearForEnvironmentReset(); }

  static size_t rawOccupied(const GridMap& map) {
    return map.md_.last_raw_occupied_count_;
  }

  static double callbackWallMs(const GridMap& map) {
    return map.md_.last_callback_wall_ms_;
  }

  static double raycastWallMs(const GridMap& map) {
    return map.md_.last_raycast_wall_ms_;
  }

  static double inflationWallMs(const GridMap& map) {
    return map.md_.last_inflation_wall_ms_;
  }

  static size_t endpointVoxelCount(const GridMap& map) {
    return map.md_.last_endpoint_voxel_count_;
  }

  static double legacySurfaceOnlyFrame(
      GridMap& map, const std::vector<Eigen::Vector3d>& points,
    const Eigen::Vector3d& origin) {
    const ros::WallTime begin = ros::WallTime::now();
    Eigen::Vector3i clear_min, clear_max;
    map.posToIndex(origin - map.mp_.local_update_range_, clear_min);
    map.posToIndex(origin + map.mp_.local_update_range_, clear_max);
    map.boundIndex(clear_min);
    map.boundIndex(clear_max);
    for (int x = clear_min(0); x <= clear_max(0); ++x)
      for (int y = clear_min(1); y <= clear_max(1); ++y)
        for (int z = clear_min(2); z <= clear_max(2); ++z)
          map.md_.occupancy_buffer_inflate_[map.toAddress(x, y, z)] = 0;
    const int inflation_xy = static_cast<int>(std::ceil(
        map.mp_.obstacles_inflation_ / map.mp_.resolution_));
    for (const Eigen::Vector3d& point : points) {
      const Eigen::Vector3d delta = point - origin;
      if (!point.allFinite() ||
          (delta.cwiseAbs().array() >= map.mp_.local_update_range_.array()).any())
        continue;
      for (int x = -inflation_xy; x <= inflation_xy; ++x)
        for (int y = -inflation_xy; y <= inflation_xy; ++y)
          for (int z = -1; z <= 1; ++z) {
            const Eigen::Vector3d inflated = point + map.mp_.resolution_ *
                Eigen::Vector3d(x, y, z);
            Eigen::Vector3i id;
            map.posToIndex(inflated, id);
            if (map.isInMap(id))
              map.md_.occupancy_buffer_inflate_[map.toAddress(id)] = 1;
          }
    }
    return (ros::WallTime::now() - begin).toSec() * 1000.0;
  }
};

namespace {

void repeatScan(
    GridMap& map, const std::vector<Eigen::Vector3d>& points,
    const Eigen::Vector3d& origin, int count = 8) {
  for (int index = 0; index < count; ++index)
    GridMapTestAccess::process(map, points, origin);
}

std::vector<Eigen::Vector3d> verticalCylinderSurface(
    const Eigen::Vector3d& center, double radius, double height,
    double z_step = 0.4) {
  std::vector<Eigen::Vector3d> points;
  for (double z = center.z() - height / 2.0;
       z <= center.z() + height / 2.0 + 1.0e-9; z += z_step) {
    for (int angle_index = 0; angle_index < 8; ++angle_index) {
      const double angle = 2.0 * M_PI * angle_index / 8.0;
      points.emplace_back(
          center.x() + radius * std::cos(angle),
          center.y() + radius * std::sin(angle), z);
    }
  }
  return points;
}

TEST(GridMapPointCloudMapping, RayCreatesKnownFreeEndpointHitAndKeepsUnknown) {
  GridMap map;
  GridMapTestAccess::initialize(map);
  const Eigen::Vector3d origin(2.0, 0.0, 3.0);
  const Eigen::Vector3d endpoint(6.0, 0.0, 3.0);

  EXPECT_EQ(map.getVoxelState(Eigen::Vector3d(4.0, 2.0, 3.0)),
            GridMap::VoxelState::UNKNOWN);
  repeatScan(map, {endpoint}, origin);

  EXPECT_EQ(map.getVoxelState(Eigen::Vector3d(4.0, 0.0, 3.0)),
            GridMap::VoxelState::KNOWN_FREE);
  EXPECT_EQ(map.getVoxelState(endpoint), GridMap::VoxelState::OCCUPIED);
  EXPECT_EQ(map.getVoxelState(Eigen::Vector3d(4.0, 2.0, 3.0)),
            GridMap::VoxelState::UNKNOWN);
}

TEST(GridMapPointCloudMapping, MissingAndPartialScansDoNotEraseHistory) {
  GridMap map;
  GridMapTestAccess::initialize(map);
  const Eigen::Vector3d origin(2.0, 0.0, 3.0);
  const Eigen::Vector3d first(5.0, 1.0, 3.0);
  repeatScan(map, {first}, origin);
  ASSERT_EQ(map.getVoxelState(first), GridMap::VoxelState::OCCUPIED);

  for (int index = 0; index < 5; ++index)
    GridMapTestAccess::process(map, {}, origin);
  repeatScan(map, {Eigen::Vector3d(5.0, -2.0, 3.4)}, origin, 3);

  EXPECT_EQ(map.getVoxelState(first), GridMap::VoxelState::OCCUPIED);
  EXPECT_TRUE(map.getPlanningOccupancy(first));
}

TEST(GridMapPointCloudMapping, RepeatedMissEvidenceCanClearAnOldHit) {
  GridMap map;
  GridMapTestAccess::initialize(map);
  const Eigen::Vector3d origin(2.0, 0.0, 3.0);
  const Eigen::Vector3d old_hit(5.0, 0.0, 3.0);
  repeatScan(map, {old_hit}, origin, 12);
  ASSERT_EQ(map.getVoxelState(old_hit), GridMap::VoxelState::OCCUPIED);

  repeatScan(map, {Eigen::Vector3d(7.0, 0.0, 3.0)}, origin, 12);
  EXPECT_EQ(map.getVoxelState(old_hit), GridMap::VoxelState::KNOWN_FREE);
}

TEST(GridMapPointCloudMapping, InflationUsesTheSameRadiusInXYZ) {
  GridMap map;
  GridMapTestAccess::initialize(map, 0.1, 0.3);
  const Eigen::Vector3d origin(2.0, 0.0, 3.0);
  const Eigen::Vector3d hit(5.0, 0.0, 3.0);
  repeatScan(map, {hit}, origin);

  EXPECT_TRUE(map.getPlanningOccupancy(hit + Eigen::Vector3d(0.25, 0.0, 0.0)));
  EXPECT_TRUE(map.getPlanningOccupancy(hit + Eigen::Vector3d(0.0, 0.25, 0.0)));
  EXPECT_TRUE(map.getPlanningOccupancy(hit + Eigen::Vector3d(0.0, 0.0, 0.25)));
}

TEST(GridMapPointCloudMapping, CylindersAtDifferentHeightsAndRadiiPersist) {
  for (double uav_z : {2.0, 3.0, 4.0}) {
    for (double radius : {0.15, 0.40, 0.70}) {
      GridMap map;
      GridMapTestAccess::initialize(
          map, 0.2, 0.4, Eigen::Vector3d(0.0, -4.0, 0.0),
          Eigen::Vector3d(12.0, 8.0, 6.0), Eigen::Vector3d(4.5, 3.5, 3.5), 6.0);
      const Eigen::Vector3d origin(2.0, 0.0, uav_z);
      const double height = uav_z < 3.0 ? 2.0 : 5.0;
      const Eigen::Vector3d center(5.0, 0.0, height / 2.0);
      const auto points = verticalCylinderSurface(center, radius, height);
      repeatScan(map, points, origin, 7);
      EXPECT_GT(GridMapTestAccess::rawOccupied(map), 0U);
      EXPECT_TRUE(map.getPlanningOccupancy(
          Eigen::Vector3d(center.x() - radius, center.y(), uav_z)));
    }
  }
}

TEST(GridMapPointCloudMapping, ThinPoleWallAndHorizontalBeamAreRepresented) {
  GridMap map;
  GridMapTestAccess::initialize(map, 0.1, 0.2);
  const Eigen::Vector3d origin(2.0, 0.0, 3.0);
  std::vector<Eigen::Vector3d> points;
  for (double z = 1.0; z <= 5.0; z += 0.1)
    points.emplace_back(4.0, -2.0, z);  // thin pole
  for (double y = -1.0; y <= 1.0; y += 0.1)
    for (double z = 1.0; z <= 5.0; z += 0.2)
      points.emplace_back(6.0, y, z);  // wall
  for (double y = -2.0; y <= 2.0; y += 0.1)
    points.emplace_back(5.0, y, 4.0);  // horizontal beam
  repeatScan(map, points, origin);

  EXPECT_TRUE(map.getPlanningOccupancy(Eigen::Vector3d(4.0, -2.0, 2.0)));
  EXPECT_TRUE(map.getPlanningOccupancy(Eigen::Vector3d(6.0, 0.5, 3.0)));
  EXPECT_TRUE(map.getPlanningOccupancy(Eigen::Vector3d(5.0, 1.5, 4.0)));
}

TEST(GridMapPointCloudMapping, UnknownRemainsObservableButSearchable) {
  GridMap map;
  GridMapTestAccess::initialize(map);
  const Eigen::Vector3d unknown(4.0, 2.0, 3.0);
  EXPECT_EQ(map.getVoxelState(unknown), GridMap::VoxelState::UNKNOWN);
  EXPECT_FALSE(map.getPlanningOccupancy(unknown));
}

TEST(GridMapPointCloudMapping, SlidingWindowClearsExitedVoxelsToUnknown) {
  GridMap map;
  GridMapTestAccess::initialize(
      map, 0.2, 0.4, Eigen::Vector3d(0.0, -10.0, 0.0),
      Eigen::Vector3d(40.0, 20.0, 10.0), Eigen::Vector3d(4.0, 4.0, 4.0));
  const Eigen::Vector3d first_origin(3.0, 0.0, 3.0);
  const Eigen::Vector3d old_hit(5.0, 0.0, 3.0);
  repeatScan(map, {old_hit}, first_origin);
  ASSERT_EQ(map.getVoxelState(old_hit), GridMap::VoxelState::OCCUPIED);

  const Eigen::Vector3d moved_origin(12.0, 0.0, 3.0);
  repeatScan(map, {Eigen::Vector3d(14.0, 0.0, 3.0)}, moved_origin);
  EXPECT_EQ(map.getVoxelState(old_hit), GridMap::VoxelState::UNKNOWN);
  EXPECT_FALSE(map.getPlanningOccupancy(old_hit));
}

TEST(GridMapPointCloudMapping, FullClearRemovesAllEpisodeEvidence) {
  GridMap map;
  GridMapTestAccess::initialize(map);
  const Eigen::Vector3d origin(2.0, 0.0, 3.0);
  const Eigen::Vector3d hit(5.0, 0.0, 3.0);
  repeatScan(map, {hit}, origin);
  ASSERT_EQ(map.getVoxelState(hit), GridMap::VoxelState::OCCUPIED);

  GridMapTestAccess::clear(map);
  EXPECT_EQ(map.getVoxelState(hit), GridMap::VoxelState::UNKNOWN);
  EXPECT_FALSE(map.getPlanningOccupancy(hit));
}

TEST(GridMapPointCloudMapping, MovingSensorOriginProducesNewFreeEvidence) {
  GridMap map;
  GridMapTestAccess::initialize(map);
  repeatScan(map, {Eigen::Vector3d(6.0, 0.0, 3.0)},
             Eigen::Vector3d(2.0, 0.0, 3.0));
  repeatScan(map, {Eigen::Vector3d(6.0, 2.0, 3.0)},
             Eigen::Vector3d(2.0, 2.0, 3.0));
  EXPECT_EQ(map.getVoxelState(Eigen::Vector3d(4.0, 0.0, 3.0)),
            GridMap::VoxelState::KNOWN_FREE);
  EXPECT_EQ(map.getVoxelState(Eigen::Vector3d(4.0, 2.0, 3.0)),
            GridMap::VoxelState::KNOWN_FREE);
}

TEST(GridMapPointCloudMapping, RaycastGenerationWrapPreservesEvidence) {
  GridMap map;
  GridMapTestAccess::initialize(
      map, 0.2, 0.4, Eigen::Vector3d(0.0, -4.0, 0.0),
      Eigen::Vector3d(12.0, 8.0, 6.0), Eigen::Vector3d(4.0, 3.5, 3.0), 6.0);
  const Eigen::Vector3d origin(2.0, 0.0, 3.0);
  const Eigen::Vector3d hit(5.0, 0.0, 3.0);
  repeatScan(map, {hit}, origin, 300);

  EXPECT_EQ(map.getVoxelState(hit), GridMap::VoxelState::OCCUPIED);
  EXPECT_EQ(map.getVoxelState(Eigen::Vector3d(4.0, 0.0, 3.0)),
            GridMap::VoxelState::KNOWN_FREE);
  EXPECT_TRUE(map.getPlanningOccupancy(hit));
}

TEST(GridMapPointCloudMapping, Medium4MissingLowerReturnDoesNotOpenOldCorridor) {
  GridMap map;
  GridMapTestAccess::initialize(
      map, 0.1, 0.3, Eigen::Vector3d(20.0, -5.0, 0.0),
      Eigen::Vector3d(20.0, 10.0, 8.0), Eigen::Vector3d(5.5, 4.5, 4.0), 4.5);
  const Eigen::Vector3d early_origin(24.8, 0.0, 3.13);
  std::vector<Eigen::Vector3d> early_lower_shell;
  for (double y = -0.5; y <= 0.2; y += 0.1)
    early_lower_shell.emplace_back(28.6852, y, 2.75);
  repeatScan(map, early_lower_shell, early_origin, 8);
  ASSERT_TRUE(map.getPlanningOccupancy(Eigen::Vector3d(28.70, 0.0, 2.65)));

  const Eigen::Vector3d current_origin(25.659, 0.0, 3.13);
  std::vector<Eigen::Vector3d> current_upper_shell;
  for (double y = -0.5; y <= 0.2; y += 0.1)
    for (double z = 2.85; z <= 3.45; z += 0.1)
      current_upper_shell.emplace_back(28.6852, y, z);
  GridMapTestAccess::process(map, current_upper_shell, current_origin);

  EXPECT_TRUE(map.getPlanningOccupancy(Eigen::Vector3d(28.70, 0.0, 2.65)));
  EXPECT_TRUE(map.getPlanningOccupancy(Eigen::Vector3d(28.80, 0.0, 2.695)));

  // Historical seed6 replan43 descended through an unrepresented Z corridor.
  // The binary planning contract keeps UNKNOWN searchable, but the accumulated
  // occupied/inflated evidence must still reject this physical-cylinder path.
  const std::vector<Eigen::Vector3d> legacy_fake_z_corridor{
      {28.197008, 0.0, 2.995214}, {28.297008, 0.0, 2.895214},
      {28.297008, 0.0, 2.795214}, {28.397008, 0.0, 2.695214},
      {28.497008, 0.0, 2.695214}, {28.597008, 0.0, 2.695214},
      {28.697008, 0.0, 2.695214}, {28.797008, 0.0, 2.695214},
      {28.897008, 0.0, 2.695214}, {28.997008, 0.0, 2.695214},
      {29.097008, 0.0, 2.695214}, {29.197008, 0.0, 2.695214},
      {29.297008, 0.0, 2.695214}, {29.397008, 0.0, 2.795214},
      {29.497008, 0.0, 2.895214}, {29.597008, 0.0, 2.995214},
      {29.697008, 0.0, 2.995214},
  };
  GridMap::SweptCollisionResult result;
  EXPECT_FALSE(map.isSweptPolylineFree(legacy_fake_z_corridor, &result));
  EXPECT_EQ(result.reason, GridMap::BlockReason::OCCUPIED);
}

TEST(GridMapPointCloudMapping, TwentyThousandRayCallbackStaysWithinTenHertzBudget) {
  GridMap map;
  GridMapTestAccess::initialize(
      map, 0.1, 0.3, Eigen::Vector3d(0.0, -6.0, 0.0),
      Eigen::Vector3d(15.0, 12.0, 8.0), Eigen::Vector3d(5.5, 4.5, 3.5), 4.5);
  const Eigen::Vector3d origin(5.0, 0.0, 3.0);
  std::vector<Eigen::Vector3d> points;
  points.reserve(20000);
  const double golden_angle = M_PI * (3.0 - std::sqrt(5.0));
  for (int index = 0; index < 20000; ++index) {
    const double unit_z = 1.0 - 2.0 * (index + 0.5) / 20000.0;
    const double radial = std::sqrt(std::max(0.0, 1.0 - unit_z * unit_z));
    const double azimuth = golden_angle * index;
    const Eigen::Vector3d direction(
        radial * std::cos(azimuth), radial * std::sin(azimuth), unit_z);
    points.push_back(origin + 4.0 * direction);
  }

  std::vector<double> callback_ms;
  for (int frame = 0; frame < 8; ++frame) {
    GridMapTestAccess::process(map, points, origin);
    callback_ms.push_back(GridMapTestAccess::callbackWallMs(map));
  }
  std::sort(callback_ms.begin(), callback_ms.end());
  const size_t p95_index = static_cast<size_t>(
      std::ceil(0.95 * callback_ms.size())) - 1;
  const double p95 = callback_ms[p95_index];
  const double mean = std::accumulate(
      callback_ms.begin(), callback_ms.end(), 0.0) / callback_ms.size();

  GridMap legacy_map;
  GridMapTestAccess::initialize(
      legacy_map, 0.1, 0.3, Eigen::Vector3d(0.0, -6.0, 0.0),
      Eigen::Vector3d(15.0, 12.0, 8.0), Eigen::Vector3d(5.5, 4.5, 3.5), 4.5);
  std::vector<double> legacy_callback_ms;
  for (int frame = 0; frame < 8; ++frame)
    legacy_callback_ms.push_back(
        GridMapTestAccess::legacySurfaceOnlyFrame(legacy_map, points, origin));
  std::sort(legacy_callback_ms.begin(), legacy_callback_ms.end());
  const double legacy_mean = std::accumulate(
      legacy_callback_ms.begin(), legacy_callback_ms.end(), 0.0) /
      legacy_callback_ms.size();
  const double legacy_p95 = legacy_callback_ms[p95_index];

  testing::Test::RecordProperty("input_rays", points.size());
  testing::Test::RecordProperty(
      "deduplicated_endpoint_voxels",
      GridMapTestAccess::endpointVoxelCount(map));
  testing::Test::RecordProperty("callback_mean_ms", std::to_string(mean));
  testing::Test::RecordProperty("callback_p95_ms", std::to_string(p95));
  testing::Test::RecordProperty("callback_max_ms", std::to_string(callback_ms.back()));
  testing::Test::RecordProperty(
      "last_raycast_ms", std::to_string(GridMapTestAccess::raycastWallMs(map)));
  testing::Test::RecordProperty(
      "last_inflation_ms", std::to_string(GridMapTestAccess::inflationWallMs(map)));
  testing::Test::RecordProperty(
      "legacy_surface_only_mean_ms", std::to_string(legacy_mean));
  testing::Test::RecordProperty(
      "legacy_surface_only_p95_ms", std::to_string(legacy_p95));

  EXPECT_LT(GridMapTestAccess::raycastWallMs(map), 100.0);
  EXPECT_LT(GridMapTestAccess::inflationWallMs(map), 100.0);
  EXPECT_LT(p95, 100.0);
}

}  // namespace

int main(int argc, char** argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
