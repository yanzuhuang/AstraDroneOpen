cmake_minimum_required(VERSION 3.0.2)
project(astra_swarm_safety)

find_package(catkin REQUIRED COMPONENTS
  astra_swarm_msgs
  rospy
  std_msgs
)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS astra_swarm_msgs rospy std_msgs)

catkin_install_python(PROGRAMS scripts/swarm_safety_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
