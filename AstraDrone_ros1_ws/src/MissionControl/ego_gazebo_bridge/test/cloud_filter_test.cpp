#include "ego_gazebo_bridge/cloud_filter.h"

#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <string>

namespace ego_gazebo_bridge {
namespace {

sensor_msgs::PointCloud2 makeCloud() {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "camera_init";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(3);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  *x = 1.0F; *y = 0.0F; *z = 0.0F;
  ++x; ++y; ++z;
  *x = 2.0F; *y = 0.0F; *z = 0.3F;
  ++x; ++y; ++z;
  *x = 3.0F; *y = 0.0F; *z = 1.0F;
  return cloud;
}

TEST(CloudFilter, RemovesGroundBandAndPreservesHeader) {
  const auto input = makeCloud();
  sensor_msgs::PointCloud2 output;
  std::string reason;
  ASSERT_TRUE(filterCloudByMinimumZ(input, 0.2, &output, &reason));
  EXPECT_EQ("camera_init", output.header.frame_id);
  EXPECT_EQ(2u, output.width);
  sensor_msgs::PointCloud2ConstIterator<float> x(output, "x");
  EXPECT_FLOAT_EQ(2.0F, *x);
  ++x;
  EXPECT_FLOAT_EQ(3.0F, *x);
}

TEST(CloudFilter, RejectsMissingFields) {
  sensor_msgs::PointCloud2 input;
  input.height = 1;
  input.width = 1;
  input.point_step = 4;
  input.row_step = 4;
  input.data.resize(4);
  sensor_msgs::PointCloud2 output;
  std::string reason;
  EXPECT_FALSE(filterCloudByMinimumZ(input, 0.2, &output, &reason));
}

}  // namespace
}  // namespace ego_gazebo_bridge

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
