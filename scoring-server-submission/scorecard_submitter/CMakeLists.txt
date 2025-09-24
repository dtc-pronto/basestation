cmake_minimum_required(VERSION 3.0.2)
project(scorecard_submitter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/submission_node.py
  scripts/spoofer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
