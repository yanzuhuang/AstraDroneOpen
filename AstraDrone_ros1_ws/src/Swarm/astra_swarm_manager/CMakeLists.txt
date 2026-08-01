cmake_minimum_required(VERSION 3.0.2)
project(astra_swarm_manager)

find_package(catkin REQUIRED COMPONENTS
  astra_custom_msgs
  astra_swarm_msgs
  geometry_msgs
  rospy
  std_msgs
)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS astra_custom_msgs astra_swarm_msgs geometry_msgs rospy std_msgs)

catkin_install_python(PROGRAMS
  scripts/swarm_manager_node.py
  scripts/swarm_state_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
