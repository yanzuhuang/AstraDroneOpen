cmake_minimum_required(VERSION 2.8.3)
project(bspline_opt)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  cv_bridge
  traj_utils
  path_searching
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES bspline_opt
 CATKIN_DEPENDS plan_env 
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library( bspline_opt 
    src/uniform_bspline.cpp 
    src/bspline_optimizer.cpp 
    src/gradient_descent_optimizer.cpp
    )
target_link_libraries( bspline_opt
    ${catkin_LIBRARIES} 
    )  

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(short_segment_guidance_test
      test/short_segment_guidance_test.cpp)
  if(TARGET short_segment_guidance_test)
    target_link_libraries(short_segment_guidance_test ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(occupied_segment_guard_test
      test/occupied_segment_guard_test.cpp)
  if(TARGET occupied_segment_guard_test)
    target_link_libraries(occupied_segment_guard_test ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(trajectory_collision_consistency_test
      test/trajectory_collision_consistency_test.cpp)
  if(TARGET trajectory_collision_consistency_test)
    target_link_libraries(trajectory_collision_consistency_test
        bspline_opt ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(frozen_replan72_test
      test/frozen_replan72_test.cpp)
  if(TARGET frozen_replan72_test)
    target_compile_definitions(frozen_replan72_test PRIVATE
        FROZEN_REPLAN72_DATA="${CMAKE_CURRENT_SOURCE_DIR}/test/data/frozen_replan72_occupancy.csv"
        FROZEN_REPLAN55_DATA="${CMAKE_CURRENT_SOURCE_DIR}/test/data/frozen_replan55_occupancy.csv"
        FROZEN_REPLAN69_DATA="${CMAKE_CURRENT_SOURCE_DIR}/test/data/frozen_replan69_occupancy.csv")
    target_link_libraries(frozen_replan72_test
        bspline_opt ${catkin_LIBRARIES})
  endif()
endif()
