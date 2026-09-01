cmake_minimum_required(VERSION 3.0.2)
project(astra_three_zone_forest)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  roscpp
)
find_package(Eigen3 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(create_random_forest create_random_forest.cpp)
target_compile_features(create_random_forest PRIVATE cxx_std_14)
target_link_libraries(create_random_forest ${catkin_LIBRARIES})
