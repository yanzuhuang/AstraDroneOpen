#include "astra_swarm_perception/self_cloud_filter.h"

#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cmath>
#include <string>
#include <vector>

namespace astra_swarm_perception {
namespace {

SelfMaskConfiguration configuration() {
  SelfMaskConfiguration result;
  result.lidar_to_imu_translation = {-0.011, -0.02329, 0.04412};
  result.boxes.push_back({{-0.0066, 0.0017, -0.0900},
                          {0.3400, 0.5200, 0.1550}, 0.0});
  result.boxes.push_back({{0.0, 0.0, -0.035}, {0.10, 0.10, 0.11}, 0.0});
  result.cylinders.push_back({{0.13, -0.22, 0.0}, 0.14, -0.085, -0.025});
  result.cylinders.push_back({{0.0, 0.0, 0.0}, 0.07, -0.04, 0.08});
  return result;
}

sensor_msgs::PointCloud2 cloud(const std::vector<Vector3>& points,
                               const std::string& frame = "camera_init") {
  sensor_msgs::PointCloud2 result;
  result.header.frame_id = frame;
  result.height = 1U;
  sensor_msgs::PointCloud2Modifier modifier(result);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(result, "x");
  sensor_msgs::PointCloud2Iterator<float> y(result, "y");
  sensor_msgs::PointCloud2Iterator<float> z(result, "z");
  for (const auto& point : points) {
    *x = static_cast<float>(point.x);
    *y = static_cast<float>(point.y);
    *z = static_cast<float>(point.z);
    ++x;
    ++y;
    ++z;
  }
  return result;
}

Vector3 lidarPointInCloud(const Vector3& lidar,
                          const geometry_msgs::Pose& pose,
                          const SelfMaskConfiguration& config) {
  // Tests use the audited identity LiDAR-to-IMU rotation.
  const Vector3 imu{lidar.x + config.lidar_to_imu_translation.x,
                    lidar.y + config.lidar_to_imu_translation.y,
                    lidar.z + config.lidar_to_imu_translation.z};
  const double x = pose.orientation.x;
  const double y = pose.orientation.y;
  const double z = pose.orientation.z;
  const double w = pose.orientation.w;
  const double r00 = 1.0 - 2.0 * (y * y + z * z);
  const double r01 = 2.0 * (x * y - z * w);
  const double r02 = 2.0 * (x * z + y * w);
  const double r10 = 2.0 * (x * y + z * w);
  const double r11 = 1.0 - 2.0 * (x * x + z * z);
  const double r12 = 2.0 * (y * z - x * w);
  const double r20 = 2.0 * (x * z - y * w);
  const double r21 = 2.0 * (y * z + x * w);
  const double r22 = 1.0 - 2.0 * (x * x + y * y);
  return {pose.position.x + r00 * imu.x + r01 * imu.y + r02 * imu.z,
          pose.position.y + r10 * imu.x + r11 * imu.y + r12 * imu.z,
          pose.position.z + r20 * imu.x + r21 * imu.y + r22 * imu.z};
}

geometry_msgs::Pose pose(double yaw = 0.0) {
  geometry_msgs::Pose result;
  result.position.x = 10.0;
  result.position.y = -3.0;
  result.position.z = 2.0;
  result.orientation.z = std::sin(0.5 * yaw);
  result.orientation.w = std::cos(0.5 * yaw);
  return result;
}

TEST(SelfCloudFilter, DeletesFuselageRotorAndSensorSupportReturns) {
  const auto config = configuration();
  const auto imu_pose = pose();
  const std::vector<Vector3> lidar_points = {
      {0.0, 0.0, -0.09}, {0.13, -0.22, -0.055}, {0.0, 0.0, 0.04},
      {0.45, 0.0, 0.0}};
  std::vector<Vector3> map_points;
  for (const auto& point : lidar_points) {
    map_points.push_back(lidarPointInCloud(point, imu_pose, config));
  }
  sensor_msgs::PointCloud2 output;
  FilterStatistics statistics;
  std::string reason;
  ASSERT_TRUE(filterSelfCloud(cloud(map_points), imu_pose, config, &output,
                              &statistics, &reason))
      << reason;
  EXPECT_EQ(statistics.input_points, 4U);
  EXPECT_EQ(statistics.self_removed_points, 3U);
  EXPECT_EQ(statistics.output_points, 1U);
  EXPECT_EQ(output.width, 1U);
}

TEST(SelfCloudFilter, UsesFullOdomOrientationInsteadOfAxisAlignedDistance) {
  const auto config = configuration();
  const auto imu_pose = pose(0.5 * M_PI);
  const Vector3 rotor{0.13, -0.22, -0.055};
  const Vector3 obstacle{0.45, 0.0, 0.0};
  const auto input = cloud({lidarPointInCloud(rotor, imu_pose, config),
                            lidarPointInCloud(obstacle, imu_pose, config)});
  sensor_msgs::PointCloud2 output;
  FilterStatistics statistics;
  std::string reason;
  ASSERT_TRUE(filterSelfCloud(input, imu_pose, config, &output, &statistics,
                              &reason))
      << reason;
  EXPECT_EQ(statistics.self_removed_points, 1U);
  EXPECT_EQ(output.width, 1U);
}

TEST(SelfCloudFilter, RetainsRealCloseObstaclesOutsideStructuralVolumes) {
  const auto config = configuration();
  const auto imu_pose = pose();
  const std::vector<Vector3> obstacles = {
      {0.45, 0.0, 0.0}, {0.0, 0.45, -0.05}, {-0.35, -0.35, 0.02},
      {0.0, 0.0, 0.20}};
  std::vector<Vector3> map_points;
  for (const auto& point : obstacles) {
    map_points.push_back(lidarPointInCloud(point, imu_pose, config));
  }
  sensor_msgs::PointCloud2 output;
  FilterStatistics statistics;
  std::string reason;
  ASSERT_TRUE(filterSelfCloud(cloud(map_points), imu_pose, config, &output,
                              &statistics, &reason))
      << reason;
  EXPECT_EQ(statistics.self_removed_points, 0U);
  EXPECT_EQ(output.width, obstacles.size());
}

TEST(SelfCloudFilter, RejectsOversizedOrMalformedGeometry) {
  auto config = configuration();
  config.boxes.front().size.x = -1.0;
  std::string reason;
  EXPECT_FALSE(validateConfiguration(config, &reason));
  EXPECT_FALSE(reason.empty());
}

}  // namespace
}  // namespace astra_swarm_perception

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
