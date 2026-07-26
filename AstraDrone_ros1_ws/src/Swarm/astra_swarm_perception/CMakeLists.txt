cmake_minimum_required(VERSION 3.0.2)
project(astra_swarm_perception)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)
find_package(catkin REQUIRED COMPONENTS
  astra_swarm_msgs
  roscpp
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS astra_swarm_msgs roscpp sensor_msgs)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(teammate_cloud_filter_node src/teammate_cloud_filter_node.cpp)
target_link_libraries(teammate_cloud_filter_node ${catkin_LIBRARIES})
add_dependencies(teammate_cloud_filter_node ${catkin_EXPORTED_TARGETS})

add_executable(world_cloud_fusion_node src/world_cloud_fusion_node.cpp)
target_link_libraries(world_cloud_fusion_node ${catkin_LIBRARIES})
add_dependencies(world_cloud_fusion_node ${catkin_EXPORTED_TARGETS})

install(TARGETS teammate_cloud_filter_node world_cloud_fusion_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
