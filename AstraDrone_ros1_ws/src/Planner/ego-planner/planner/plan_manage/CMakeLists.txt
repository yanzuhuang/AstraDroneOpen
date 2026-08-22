cmake_minimum_required(VERSION 2.8.3)
project(ego_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  astra_custom_msgs
  learning_speed_rl
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  bspline_opt
  traj_utils
  cv_bridge
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ego_planner
 CATKIN_DEPENDS astra_custom_msgs learning_speed_rl plan_env path_searching bspline_opt traj_utils
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(ego_planner_node
  src/ego_planner_node.cpp 
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
  )
target_link_libraries(ego_planner_node 
  ${catkin_LIBRARIES}
  )
add_dependencies(ego_planner_node ${catkin_EXPORTED_TARGETS})

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
add_dependencies(traj_server ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(goal_height_test test/goal_height_test.cpp)
  if(TARGET goal_height_test)
    target_link_libraries(goal_height_test ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(planning_status_tracker_test
    test/planning_status_tracker_test.cpp)
  if(TARGET planning_status_tracker_test)
    target_link_libraries(planning_status_tracker_test ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(trajectory_cancellation_gate_test
    test/trajectory_cancellation_gate_test.cpp)
  if(TARGET trajectory_cancellation_gate_test)
    target_link_libraries(trajectory_cancellation_gate_test ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(swarm_frame_transform_test
    test/swarm_frame_transform_test.cpp)
  if(TARGET swarm_frame_transform_test)
    target_link_libraries(swarm_frame_transform_test ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(dynamic_speed_limit_test
    test/dynamic_speed_limit_test.cpp)
  if(TARGET dynamic_speed_limit_test)
    target_link_libraries(dynamic_speed_limit_test ${catkin_LIBRARIES})
  endif()
endif()
