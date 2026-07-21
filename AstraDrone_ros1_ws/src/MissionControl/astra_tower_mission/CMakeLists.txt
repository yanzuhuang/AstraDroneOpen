cmake_minimum_required(VERSION 3.0.2)
project(astra_tower_mission)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  offboard
  roscpp
  std_msgs
  visualization_msgs
  xmlrpcpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tower_route motion_limiter
  CATKIN_DEPENDS
    geometry_msgs
    mavros_msgs
    nav_msgs
    offboard
    roscpp
    std_msgs
    visualization_msgs
    xmlrpcpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(tower_route src/tower_route.cpp)
target_link_libraries(tower_route ${catkin_LIBRARIES})
add_dependencies(tower_route ${catkin_EXPORTED_TARGETS})

add_library(motion_limiter src/motion_limiter.cpp)
target_link_libraries(motion_limiter tower_route ${catkin_LIBRARIES})
add_dependencies(motion_limiter ${catkin_EXPORTED_TARGETS})

add_executable(tower_mission_node src/tower_mission_node.cpp)
target_link_libraries(tower_mission_node
  tower_route
  motion_limiter
  ${catkin_LIBRARIES}
)
add_dependencies(tower_mission_node ${catkin_EXPORTED_TARGETS})

install(TARGETS tower_route motion_limiter tower_mission_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(tower_route_test test/tower_route_test.cpp)
  if(TARGET tower_route_test)
    target_link_libraries(tower_route_test
      tower_route
      motion_limiter
      ${catkin_LIBRARIES}
    )
  endif()
endif()
