cmake_minimum_required(VERSION 3.0.2)
project(basestation_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  ServerReport.msg
  InitialReport.msg
  FullReport.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    std_msgs 
    sensor_msgs 
    nav_msgs 
    geometry_msgs 
    message_runtime
)
