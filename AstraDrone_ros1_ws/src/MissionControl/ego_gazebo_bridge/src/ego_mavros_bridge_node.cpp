#include "ego_gazebo_bridge/ego_mavros_bridge.h"

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "ego_mavros_bridge");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  ego_gazebo_bridge::EgoMavrosBridge bridge(node_handle,
                                             private_node_handle);
  ros::spin();
  return 0;
}
