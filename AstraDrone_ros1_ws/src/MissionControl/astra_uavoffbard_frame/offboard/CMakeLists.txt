cmake_minimum_required(VERSION 3.0.2)
project(offboard)

# ROS Kinetic and newer support C++11.
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  std_msgs
  xmlrpcpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp rospy std_msgs xmlrpcpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(position_control_lib src/position_control_lib.cpp)
add_dependencies(position_control_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(position_control src/position_control.cpp)
add_dependencies(position_control
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(position_control
  position_control_lib
  ${catkin_LIBRARIES}
)

add_executable(autoarming_control src/autoarming_control.cpp)
target_link_libraries(autoarming_control
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(trajectory_reference_test test/trajectory_reference_test.cpp)
  if(TARGET trajectory_reference_test)
    target_link_libraries(trajectory_reference_test ${catkin_LIBRARIES})
  endif()
endif()

add_executable(pub_origin src/pub_origin.cpp)
target_link_libraries(pub_origin
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/analyze_trajectory_bag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
