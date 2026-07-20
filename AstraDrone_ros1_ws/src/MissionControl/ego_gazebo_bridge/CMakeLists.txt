cmake_minimum_required(VERSION 3.0.2)
project(ego_gazebo_bridge)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  quadrotor_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ego_bridge_command_utils ego_bridge_control_authority
  CATKIN_DEPENDS
    geometry_msgs
    mavros_msgs
    nav_msgs
    quadrotor_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ego_bridge_command_utils
  src/command_utils.cpp
)
target_link_libraries(ego_bridge_command_utils ${catkin_LIBRARIES})
add_dependencies(ego_bridge_command_utils ${catkin_EXPORTED_TARGETS})

add_library(ego_bridge_control_authority
  src/control_authority.cpp
)
target_link_libraries(ego_bridge_control_authority ${catkin_LIBRARIES})
add_dependencies(ego_bridge_control_authority ${catkin_EXPORTED_TARGETS})

add_executable(ego_mavros_bridge
  src/ego_mavros_bridge.cpp
  src/ego_mavros_bridge_node.cpp
)
target_link_libraries(ego_mavros_bridge
  ego_bridge_command_utils
  ego_bridge_control_authority
  ${catkin_LIBRARIES}
)
add_dependencies(ego_mavros_bridge ${catkin_EXPORTED_TARGETS})

install(TARGETS ego_bridge_command_utils ego_bridge_control_authority ego_mavros_bridge
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
  catkin_add_gtest(command_utils_test test/command_utils_test.cpp)
  if(TARGET command_utils_test)
    target_link_libraries(command_utils_test
      ego_bridge_command_utils
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(control_authority_test test/control_authority_test.cpp)
  if(TARGET control_authority_test)
    target_link_libraries(control_authority_test
      ego_bridge_control_authority
      ${catkin_LIBRARIES}
    )
  endif()
endif()
