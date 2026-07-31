cmake_minimum_required(VERSION 3.0.2)
project(astra_swarm_bringup)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/px4_param_guard.py
  scripts/swarm_conflict_injector.py
  scripts/swarm_evidence_recorder.py
  scripts/swarm_transport_probe.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config models urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
