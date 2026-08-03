#include "astra_swarm_perception/self_cloud_filter.h"

#include <sensor_msgs/PointField.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace astra_swarm_perception {
namespace {

bool finite(double value) { return std::isfinite(value); }

bool fieldOffset(const sensor_msgs::PointCloud2& cloud,
                 const std::string& name, std::uint32_t* offset) {
  for (const auto& field : cloud.fields) {
    if (field.name == name && field.count == 1U &&
        field.datatype == sensor_msgs::PointField::FLOAT32) {
      *offset = field.offset;
      return true;
    }
  }
  return false;
}

float readFloat(const std::uint8_t* point, std::uint32_t offset) {
  float value = std::numeric_limits<float>::quiet_NaN();
  std::memcpy(&value, point + offset, sizeof(value));
  return value;
}

Vector3 transposeMultiply(const std::array<double, 9>& matrix,
                          const Vector3& vector) {
  return {
      matrix[0] * vector.x + matrix[3] * vector.y + matrix[6] * vector.z,
      matrix[1] * vector.x + matrix[4] * vector.y + matrix[7] * vector.z,
      matrix[2] * vector.x + matrix[5] * vector.y + matrix[8] * vector.z};
}

bool quaternionRotation(const geometry_msgs::Quaternion& quaternion,
                        std::array<double, 9>* rotation) {
  const double norm = std::sqrt(
      quaternion.x * quaternion.x + quaternion.y * quaternion.y +
      quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  if (!finite(norm) || norm < 1.0e-9) return false;
  const double x = quaternion.x / norm;
  const double y = quaternion.y / norm;
  const double z = quaternion.z / norm;
  const double w = quaternion.w / norm;
  *rotation = {{
      1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
      2.0 * (x * z + y * w),
      2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
      2.0 * (y * z - x * w),
      2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
      1.0 - 2.0 * (x * x + y * y)}};
  return true;
}

double squaredNorm(const Vector3& vector) {
  return vector.x * vector.x + vector.y * vector.y + vector.z * vector.z;
}

}  // namespace

bool validateConfiguration(const SelfMaskConfiguration& configuration,
                           std::string* reason) {
  if (reason == nullptr) return false;
  const Vector3& translation = configuration.lidar_to_imu_translation;
  if (!finite(translation.x) || !finite(translation.y) ||
      !finite(translation.z)) {
    *reason = "LiDAR-to-IMU translation must be finite";
    return false;
  }
  for (double value : configuration.lidar_to_imu_rotation) {
    if (!finite(value)) {
      *reason = "LiDAR-to-IMU rotation must be finite";
      return false;
    }
  }
  // This catches accidental scale/shear parameters without adding a matrix
  // dependency.  FAST-LIO's configured matrix is identity in this project.
  for (int column = 0; column < 3; ++column) {
    double norm = 0.0;
    for (int row = 0; row < 3; ++row) {
      const double value = configuration.lidar_to_imu_rotation[3 * row + column];
      norm += value * value;
    }
    if (std::abs(norm - 1.0) > 1.0e-3) {
      *reason = "LiDAR-to-IMU rotation columns must have unit norm";
      return false;
    }
  }
  for (const auto& box : configuration.boxes) {
    if (!finite(box.center.x) || !finite(box.center.y) ||
        !finite(box.center.z) || !finite(box.size.x) ||
        !finite(box.size.y) || !finite(box.size.z) ||
        !finite(box.yaw) || box.size.x <= 0.0 || box.size.y <= 0.0 ||
        box.size.z <= 0.0) {
      *reason = "self-exclusion boxes require finite positive dimensions";
      return false;
    }
  }
  for (const auto& cylinder : configuration.cylinders) {
    if (!finite(cylinder.center.x) || !finite(cylinder.center.y) ||
        !finite(cylinder.center.z) || !finite(cylinder.radius) ||
        !finite(cylinder.z_min) || !finite(cylinder.z_max) ||
        cylinder.radius <= 0.0 || cylinder.z_max <= cylinder.z_min) {
      *reason = "self-exclusion cylinders require valid finite geometry";
      return false;
    }
  }
  if (configuration.boxes.empty() && configuration.cylinders.empty()) {
    *reason = "at least one self-exclusion volume is required";
    return false;
  }
  reason->clear();
  return true;
}

bool pointInsideSelfMask(const Vector3& point,
                         const SelfMaskConfiguration& configuration) {
  for (const auto& box : configuration.boxes) {
    const double dx = point.x - box.center.x;
    const double dy = point.y - box.center.y;
    const double dz = point.z - box.center.z;
    const double cosine = std::cos(box.yaw);
    const double sine = std::sin(box.yaw);
    const double local_x = cosine * dx + sine * dy;
    const double local_y = -sine * dx + cosine * dy;
    if (std::abs(local_x) <= 0.5 * box.size.x &&
        std::abs(local_y) <= 0.5 * box.size.y &&
        std::abs(dz) <= 0.5 * box.size.z) {
      return true;
    }
  }
  for (const auto& cylinder : configuration.cylinders) {
    const double dx = point.x - cylinder.center.x;
    const double dy = point.y - cylinder.center.y;
    const double dz = point.z - cylinder.center.z;
    if (dx * dx + dy * dy <= cylinder.radius * cylinder.radius &&
        dz >= cylinder.z_min && dz <= cylinder.z_max) {
      return true;
    }
  }
  return false;
}

bool filterSelfCloud(const sensor_msgs::PointCloud2& input,
                     const geometry_msgs::Pose& imu_pose,
                     const SelfMaskConfiguration& configuration,
                     sensor_msgs::PointCloud2* output,
                     FilterStatistics* statistics,
                     std::string* reason) {
  if (output == nullptr || statistics == nullptr || reason == nullptr) {
    return false;
  }
  *statistics = FilterStatistics{};
  if (!validateConfiguration(configuration, reason)) return false;
  if (input.height == 0U || input.point_step == 0U ||
      input.row_step < input.width * input.point_step ||
      input.data.size() < static_cast<std::size_t>(input.row_step) * input.height) {
    *reason = "input cloud layout is invalid";
    return false;
  }
  std::uint32_t x_offset = 0U, y_offset = 0U, z_offset = 0U;
  if (!fieldOffset(input, "x", &x_offset) ||
      !fieldOffset(input, "y", &y_offset) ||
      !fieldOffset(input, "z", &z_offset) ||
      x_offset + sizeof(float) > input.point_step ||
      y_offset + sizeof(float) > input.point_step ||
      z_offset + sizeof(float) > input.point_step) {
    *reason = "cloud requires scalar FLOAT32 x/y/z fields";
    return false;
  }
  if (!finite(imu_pose.position.x) || !finite(imu_pose.position.y) ||
      !finite(imu_pose.position.z)) {
    *reason = "odometry position must be finite";
    return false;
  }
  std::array<double, 9> imu_to_cloud;
  if (!quaternionRotation(imu_pose.orientation, &imu_to_cloud)) {
    *reason = "odometry orientation is invalid";
    return false;
  }

  *output = input;
  output->height = 1U;
  output->width = 0U;
  output->row_step = 0U;
  output->data.clear();
  output->data.reserve(input.data.size());
  const Vector3 imu_position{imu_pose.position.x, imu_pose.position.y,
                             imu_pose.position.z};
  double nearest_before_squared = std::numeric_limits<double>::infinity();
  double nearest_after_squared = std::numeric_limits<double>::infinity();

  for (std::uint32_t row = 0U; row < input.height; ++row) {
    for (std::uint32_t column = 0U; column < input.width; ++column) {
      const std::size_t offset = static_cast<std::size_t>(row) * input.row_step +
                                 static_cast<std::size_t>(column) * input.point_step;
      const std::uint8_t* point_data = input.data.data() + offset;
      const float x = readFloat(point_data, x_offset);
      const float y = readFloat(point_data, y_offset);
      const float z = readFloat(point_data, z_offset);
      ++statistics->input_points;
      bool remove = false;
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
        const Vector3 point_in_cloud{x, y, z};
        const Vector3 relative_cloud{
            point_in_cloud.x - imu_position.x,
            point_in_cloud.y - imu_position.y,
            point_in_cloud.z - imu_position.z};
        nearest_before_squared =
            std::min(nearest_before_squared, squaredNorm(relative_cloud));
        const Vector3 point_in_imu =
            transposeMultiply(imu_to_cloud, relative_cloud);
        const Vector3 imu_relative_to_lidar{
            point_in_imu.x - configuration.lidar_to_imu_translation.x,
            point_in_imu.y - configuration.lidar_to_imu_translation.y,
            point_in_imu.z - configuration.lidar_to_imu_translation.z};
        const Vector3 point_in_lidar = transposeMultiply(
            configuration.lidar_to_imu_rotation, imu_relative_to_lidar);
        remove = pointInsideSelfMask(point_in_lidar, configuration);
        if (!remove) {
          nearest_after_squared =
              std::min(nearest_after_squared, squaredNorm(relative_cloud));
        }
      }
      if (remove) {
        ++statistics->self_removed_points;
        continue;
      }
      output->data.insert(output->data.end(), point_data,
                          point_data + input.point_step);
      ++output->width;
    }
  }
  output->row_step = output->width * output->point_step;
  output->data.resize(output->row_step);
  statistics->output_points = output->width;
  if (std::isfinite(nearest_before_squared)) {
    statistics->have_nearest_before = true;
    statistics->nearest_before = std::sqrt(nearest_before_squared);
  }
  if (std::isfinite(nearest_after_squared)) {
    statistics->have_nearest_after = true;
    statistics->nearest_after = std::sqrt(nearest_after_squared);
  }
  reason->clear();
  return true;
}

}  // namespace astra_swarm_perception
