cmake_minimum_required(VERSION 3.10)
project(depth_transport_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DepthStack.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
ament_package()
