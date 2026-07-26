cmake_minimum_required(VERSION 3.0.2)
project(astra_swarm_tf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  tf2_msgs
)

catkin_package(CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs tf2_msgs)
catkin_install_python(PROGRAMS scripts/frame_adapter_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
