cmake_minimum_required(VERSION 2.8.3)
project(plan_env)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_env
 CATKIN_DEPENDS roscpp std_msgs
 DEPENDS OpenCV
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( plan_env 
    src/grid_map.cpp 
    src/raycast.cpp
    src/obj_predictor.cpp 
    )
target_link_libraries( plan_env
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    )  

add_executable(obj_generator
    src/obj_generator.cpp 
)
target_link_libraries(obj_generator 
    ${catkin_LIBRARIES}
    )

if(CATKIN_ENABLE_TESTING)
  add_executable(grid_map_frozen_replay_node
      test/grid_map_frozen_replay_node.cpp)
  target_link_libraries(grid_map_frozen_replay_node
      plan_env
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      ${OpenCV_LIBS})

  catkin_add_gtest(grid_map_pointcloud_mapping_test
      test/grid_map_pointcloud_mapping_test.cpp)
  if(TARGET grid_map_pointcloud_mapping_test)
    target_link_libraries(grid_map_pointcloud_mapping_test
        plan_env
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS})
  endif()

  catkin_add_gtest(swept_collision_checker_test
      test/swept_collision_checker_test.cpp)
  if(TARGET swept_collision_checker_test)
    target_link_libraries(swept_collision_checker_test
        plan_env
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS})
  endif()
endif()
