cmake_minimum_required(VERSION 3.5)
project(rover_baseboard_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EncoderState.msg"
  "msg/MotorCommand.msg"
  "msg/SafetyState.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
