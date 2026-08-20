cmake_minimum_required(VERSION 3.0.2)
project(learning_speed_rl)

find_package(catkin REQUIRED COMPONENTS
  astra_custom_msgs
  diagnostic_msgs
  geometry_msgs
  message_generation
  nav_msgs
  quadrotor_msgs
  rospy
  sensor_msgs
  std_msgs
  traj_utils
  visualization_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  LidarSurrogateStamped.msg
  ObservationC.msg
  SpeedActionStamped.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS astra_custom_msgs diagnostic_msgs geometry_msgs message_runtime nav_msgs
                 quadrotor_msgs rospy sensor_msgs std_msgs traj_utils
                 visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/speed_adapter_node.py
  scripts/observation_v2_node.py
  scripts/observation_v2_evaluator.py
  scripts/observation_c_node.py
  scripts/observation_c_offline_validation.py
  scripts/fixed_speed_baseline_recorder.py
  scripts/calibration_data_recorder.py
  scripts/enrich_fixed_speed_baseline.py
  scripts/summarize_fixed_speed_baseline.py
  scripts/summarize_high_speed_exploration.py
  scripts/summarize_manual_calibration.py
  scripts/summarize_observation_c_final.py
  scripts/high_speed_parameter_preflight.py
  scripts/summarize_high_speed_progressive_qualification.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_observation.py)
  catkin_add_nosetests(test/test_observation_v2.py)
  catkin_add_nosetests(test/test_observation_c.py)
  catkin_add_nosetests(test/test_safety_filter.py)
  catkin_add_nosetests(test/test_training_artifacts.py)
  catkin_add_nosetests(test/test_training_data_contract.py)
  catkin_add_nosetests(test/test_high_speed_parameter_chain.py)
  add_rostest(test/observation_c_integration.test)
  add_rostest(test/observation_v2_stamp_integration.test)
  add_rostest(test/speed_action_integration.test)
endif()
