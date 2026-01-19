cmake_minimum_required(VERSION 3.8)
project(scorecard_submitter)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scorecard_submitter/submission_node.py
  scorecard_submitter/spoofer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install python dependencies
install(
  FILES
  scorecard_submitter/helpers.py
  scorecard_submitter/submission.py
  DESTINATION lib/${PROJECT_NAME}
)

# Insert config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

ament_package()
