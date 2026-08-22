cmake_minimum_required(VERSION 3.0.2)
project(hector_ego_training_backend)

find_package(catkin REQUIRED COMPONENTS
  controller_manager_msgs
  diagnostic_msgs
  gazebo_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  traj_utils
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS controller_manager_msgs diagnostic_msgs gazebo_msgs geometry_msgs
                 nav_msgs quadrotor_msgs rospy sensor_msgs std_msgs std_srvs
                 traj_utils
)

catkin_install_python(PROGRAMS
  scripts/gazebo_truth_odometry_adapter.py
  scripts/position_command_to_hector.py
  scripts/hector_backend_qualifier.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_adapter_contract.py)
  catkin_add_nosetests(test/test_truth_odom_contract.py)
endif()
