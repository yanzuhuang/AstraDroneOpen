#ifndef EGO_GAZEBO_BRIDGE_CLOUD_FILTER_H_
#define EGO_GAZEBO_BRIDGE_CLOUD_FILTER_H_

#include <sensor_msgs/PointCloud2.h>

#include <string>

namespace ego_gazebo_bridge {

bool filterCloudByMinimumZ(const sensor_msgs::PointCloud2& input,
                           double minimum_z,
                           sensor_msgs::PointCloud2* output,
                           std::string* reason);

}  // namespace ego_gazebo_bridge

#endif  // EGO_GAZEBO_BRIDGE_CLOUD_FILTER_H_
