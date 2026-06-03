cmake_minimum_required(VERSION 3.8)
project(balance_car_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/CarEvent.msg"
  "msg/CarState.msg"
  "msg/SystemState.msg"
  "msg/VisionTarget.msg"
)

set(srv_files
  "srv/QueryState.srv"
  "srv/SetCarMode.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
