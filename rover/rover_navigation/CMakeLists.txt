cmake_minimum_required(VERSION 3.8)
project(rover_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gnss_check.py
  scripts/imu_check.py
  scripts/localization_check.py
  scripts/nav_stack_check.py
  scripts/odom_calibration_check.py
  scripts/send_nav_goal.py
  scripts/wheel_velocity_odom_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
