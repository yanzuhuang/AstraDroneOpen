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
  std_srvs
  traj_utils
  visualization_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  LidarSurrogateStamped.msg
  ObservationC.msg
  SpeedAppliedStamped.msg
  SpeedActionStamped.msg
  SpeedRequestStamped.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS astra_custom_msgs diagnostic_msgs geometry_msgs message_runtime nav_msgs
                 quadrotor_msgs rospy sensor_msgs std_msgs traj_utils
                 std_srvs visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/speed_adapter_node.py
  scripts/observation_v2_node.py
  scripts/observation_c_node.py
  scripts/calibration_data_recorder.py
  scripts/sac_training_runner.py
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
  catkin_add_nosetests(test/test_latest_sample.py)
  catkin_add_nosetests(test/test_safety_filter.py)
  catkin_add_nosetests(test/test_training_artifacts.py)
  catkin_add_nosetests(test/test_training_data_contract.py)
  catkin_add_nosetests(test/test_stage1_reward.py)
  catkin_add_nosetests(test/test_astra_drone_env.py)
  catkin_add_nosetests(test/test_actor_action_ownership.py)
  catkin_add_nosetests(test/test_sac.py)
  catkin_add_nosetests(test/test_sac_replay.py)
  catkin_add_nosetests(test/test_action_capability_contract.py)
  catkin_add_nosetests(test/test_formal_training_contract.py)
  catkin_add_nosetests(test/test_forest_schedule.py)
  add_rostest(test/observation_c_integration.test)
  add_rostest(test/observation_v2_stamp_integration.test)
  add_rostest(test/speed_action_integration.test)
  add_rostest(test/mock_request_driven_integration.test)
endif()
