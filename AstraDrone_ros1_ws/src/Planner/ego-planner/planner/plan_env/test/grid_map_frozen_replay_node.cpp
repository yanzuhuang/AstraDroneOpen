#include <ros/ros.h>

#include "plan_env/grid_map.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "grid_map_frozen_replay");
  ros::NodeHandle private_node("~");
  GridMap map;
  map.initMap(private_node);
  ros::spin();
  return 0;
}
