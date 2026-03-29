cmake_minimum_required(VERSION 3.5)
project(rover_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(effort_controllers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(xacro REQUIRED)

install (
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/cmd_vel_to_drive_command.py
  scripts/send_drive_command.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
