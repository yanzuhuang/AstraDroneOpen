#ifndef ASTRA_SWARM_PERCEPTION_SELF_CLOUD_FILTER_H_
#define ASTRA_SWARM_PERCEPTION_SELF_CLOUD_FILTER_H_

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace astra_swarm_perception {

struct Vector3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// All exclusion geometry is expressed in the LiDAR measurement frame.  The
// registered cloud is transformed map -> IMU with odometry and IMU -> LiDAR
// with the calibrated FAST-LIO extrinsic before these volumes are evaluated.
struct BoxVolume {
  Vector3 center;
  Vector3 size;
  double yaw{0.0};
};

struct CylinderVolume {
  Vector3 center;
  double radius{0.0};
  double z_min{0.0};
  double z_max{0.0};
};

struct SelfMaskConfiguration {
  Vector3 lidar_to_imu_translation;
  std::array<double, 9> lidar_to_imu_rotation{{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0}};
  std::vector<BoxVolume> boxes;
  std::vector<CylinderVolume> cylinders;
};

struct FilterStatistics {
  std::size_t input_points{0U};
  std::size_t self_removed_points{0U};
  std::size_t output_points{0U};
  double nearest_before{0.0};
  double nearest_after{0.0};
  bool have_nearest_before{false};
  bool have_nearest_after{false};
};

bool validateConfiguration(const SelfMaskConfiguration& configuration,
                           std::string* reason);

bool pointInsideSelfMask(const Vector3& point_in_lidar,
                         const SelfMaskConfiguration& configuration);

bool filterSelfCloud(const sensor_msgs::PointCloud2& input,
                     const geometry_msgs::Pose& imu_pose_in_cloud,
                     const SelfMaskConfiguration& configuration,
                     sensor_msgs::PointCloud2* output,
                     FilterStatistics* statistics,
                     std::string* reason);

}  // namespace astra_swarm_perception

#endif  // ASTRA_SWARM_PERCEPTION_SELF_CLOUD_FILTER_H_
