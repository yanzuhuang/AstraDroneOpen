cmake_minimum_required(VERSION 3.0.2)
project(astra_tower_mission)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  astra_custom_msgs
  mavros_msgs
  nav_msgs
  offboard
  quadrotor_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  xmlrpcpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tower_route motion_limiter ego_task_utils
            low_altitude_path_planner inspection_candidate_planner
  CATKIN_DEPENDS
    geometry_msgs
    astra_custom_msgs
    mavros_msgs
    nav_msgs
    offboard
    quadrotor_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
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

add_library(ego_task_utils src/ego_task_utils.cpp)
target_link_libraries(ego_task_utils ${catkin_LIBRARIES})
add_dependencies(ego_task_utils ${catkin_EXPORTED_TARGETS})

add_library(low_altitude_path_planner src/low_altitude_path_planner.cpp)
target_link_libraries(low_altitude_path_planner ${catkin_LIBRARIES})
add_dependencies(low_altitude_path_planner ${catkin_EXPORTED_TARGETS})

add_library(inspection_candidate_planner src/inspection_candidate_planner.cpp)
target_link_libraries(inspection_candidate_planner
  low_altitude_path_planner
  tower_route
  ${catkin_LIBRARIES}
)
add_dependencies(inspection_candidate_planner ${catkin_EXPORTED_TARGETS})

add_executable(tower_mission_node src/tower_mission_node.cpp)
target_link_libraries(tower_mission_node
  tower_route
  motion_limiter
  ${catkin_LIBRARIES}
)
add_dependencies(tower_mission_node ${catkin_EXPORTED_TARGETS})

add_executable(ego_waypoint_mission_node src/ego_waypoint_mission_node.cpp)
target_link_libraries(ego_waypoint_mission_node
  tower_route
  ego_task_utils
  ${catkin_LIBRARIES}
)
add_dependencies(ego_waypoint_mission_node ${catkin_EXPORTED_TARGETS})

add_executable(sector_inspection_mission_node src/sector_inspection_mission_node.cpp)
target_link_libraries(sector_inspection_mission_node
  inspection_candidate_planner
  tower_route
  ego_task_utils
  ${catkin_LIBRARIES}
)
add_dependencies(sector_inspection_mission_node ${catkin_EXPORTED_TARGETS})

add_executable(planner_status_adapter_node src/planner_status_adapter_node.cpp)
target_link_libraries(planner_status_adapter_node ${catkin_LIBRARIES})
add_dependencies(planner_status_adapter_node ${catkin_EXPORTED_TARGETS})

add_executable(occupancy_stamp_adapter_node src/occupancy_stamp_adapter_node.cpp)
target_link_libraries(occupancy_stamp_adapter_node ${catkin_LIBRARIES})
add_dependencies(occupancy_stamp_adapter_node ${catkin_EXPORTED_TARGETS})

install(TARGETS tower_route motion_limiter ego_task_utils
  low_altitude_path_planner inspection_candidate_planner
  tower_mission_node ego_waypoint_mission_node sector_inspection_mission_node
  planner_status_adapter_node occupancy_stamp_adapter_node
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
  find_package(rostest REQUIRED)
  catkin_add_gtest(tower_route_test test/tower_route_test.cpp)
  if(TARGET tower_route_test)
    target_link_libraries(tower_route_test
      tower_route
      motion_limiter
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(ego_task_utils_test test/ego_task_utils_test.cpp)
  if(TARGET ego_task_utils_test)
    target_link_libraries(ego_task_utils_test
      ego_task_utils
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(inspection_candidate_planner_test test/inspection_candidate_planner_test.cpp)
  if(TARGET inspection_candidate_planner_test)
    target_link_libraries(inspection_candidate_planner_test
      inspection_candidate_planner
      tower_route
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(mission_completion_test test/mission_completion_test.cpp)
  if(TARGET mission_completion_test)
    target_link_libraries(mission_completion_test ${catkin_LIBRARIES})
  endif()

  add_rostest(test/sector_inspection_no_control.test)
  add_rostest(test/sector_exit_admission_failure_no_control.test)
endif()
