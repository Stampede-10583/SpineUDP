cmake_minimum_required(VERSION 3.20)
project(spine_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SpineOdometry.msg"
  "msg/SpineTrigger.msg"
  "msg/FusedPose.msg"
  "msg/LinkStats.msg"
  "msg/LinkEvent.msg"
  "msg/AdapterStats.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
