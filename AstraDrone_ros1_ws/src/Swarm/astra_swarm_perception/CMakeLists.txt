cmake_minimum_required(VERSION 3.0.2)
project(astra_swarm_perception)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)
find_package(catkin REQUIRED COMPONENTS
  astra_swarm_msgs
  diagnostic_msgs
  geometry_msgs
  message_filters
  nav_msgs
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES astra_self_cloud_filter
  CATKIN_DEPENDS astra_swarm_msgs diagnostic_msgs geometry_msgs
    message_filters nav_msgs roscpp sensor_msgs)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(astra_self_cloud_filter src/self_cloud_filter.cpp)
target_link_libraries(astra_self_cloud_filter ${catkin_LIBRARIES})
add_dependencies(astra_self_cloud_filter ${catkin_EXPORTED_TARGETS})

add_executable(self_cloud_filter_node src/self_cloud_filter_node.cpp)
target_link_libraries(self_cloud_filter_node
  astra_self_cloud_filter ${catkin_LIBRARIES})
add_dependencies(self_cloud_filter_node ${catkin_EXPORTED_TARGETS})

add_executable(teammate_cloud_filter_node src/teammate_cloud_filter_node.cpp)
target_link_libraries(teammate_cloud_filter_node ${catkin_LIBRARIES})
add_dependencies(teammate_cloud_filter_node ${catkin_EXPORTED_TARGETS})

add_executable(world_cloud_fusion_node src/world_cloud_fusion_node.cpp)
target_link_libraries(world_cloud_fusion_node ${catkin_LIBRARIES})
add_dependencies(world_cloud_fusion_node ${catkin_EXPORTED_TARGETS})

install(TARGETS astra_self_cloud_filter self_cloud_filter_node
  teammate_cloud_filter_node world_cloud_fusion_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(self_cloud_filter_test test/self_cloud_filter_test.cpp)
  if(TARGET self_cloud_filter_test)
    target_link_libraries(self_cloud_filter_test
      astra_self_cloud_filter ${catkin_LIBRARIES})
  endif()
endif()
