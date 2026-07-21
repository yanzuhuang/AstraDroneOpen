#include "ego_gazebo_bridge/cloud_filter.h"

#include <sensor_msgs/PointField.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

namespace ego_gazebo_bridge {
namespace {

bool fieldOffset(const sensor_msgs::PointCloud2& cloud,
                 const std::string& name, std::uint32_t* offset) {
  for (const auto& field : cloud.fields) {
    if (field.name == name && field.count == 1 &&
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

}  // namespace

bool filterCloudByMinimumZ(const sensor_msgs::PointCloud2& input,
                           double minimum_z,
                           sensor_msgs::PointCloud2* output,
                           std::string* reason) {
  if (output == nullptr || reason == nullptr || !std::isfinite(minimum_z)) {
    return false;
  }
  if (input.height == 0 || input.width == 0 || input.point_step == 0 ||
      input.row_step < input.width * input.point_step ||
      input.data.size() <
          static_cast<std::size_t>(input.row_step) * input.height) {
    *reason = "input cloud layout is invalid";
    return false;
  }

  std::uint32_t x_offset = 0;
  std::uint32_t y_offset = 0;
  std::uint32_t z_offset = 0;
  if (!fieldOffset(input, "x", &x_offset) ||
      !fieldOffset(input, "y", &y_offset) ||
      !fieldOffset(input, "z", &z_offset) ||
      x_offset + sizeof(float) > input.point_step ||
      y_offset + sizeof(float) > input.point_step ||
      z_offset + sizeof(float) > input.point_step) {
    *reason = "cloud requires scalar FLOAT32 x/y/z fields";
    return false;
  }

  *output = input;
  output->height = 1;
  output->width = 0;
  output->row_step = 0;
  output->data.clear();
  output->data.reserve(input.data.size());

  for (std::uint32_t row = 0; row < input.height; ++row) {
    for (std::uint32_t column = 0; column < input.width; ++column) {
      const std::size_t offset =
          static_cast<std::size_t>(row) * input.row_step +
          static_cast<std::size_t>(column) * input.point_step;
      const std::uint8_t* point = input.data.data() + offset;
      const float x = readFloat(point, x_offset);
      const float y = readFloat(point, y_offset);
      const float z = readFloat(point, z_offset);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
          z < minimum_z) {
        continue;
      }
      output->data.insert(output->data.end(), point,
                          point + input.point_step);
      ++output->width;
    }
  }
  output->row_step = output->width * output->point_step;
  output->data.resize(output->row_step);
  output->is_dense = true;
  reason->clear();
  return true;
}

}  // namespace ego_gazebo_bridge
