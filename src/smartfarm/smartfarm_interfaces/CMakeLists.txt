cmake_minimum_required(VERSION 3.8)
project(smartfarm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TomatoDetection.msg"
  "msg/TomatoDetectionArray.msg"
  "msg/TrayStatus.msg"
  "msg/TransportRequest.msg"
  "msg/HandoffEvent.msg"
  "msg/SlotAssignment.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
