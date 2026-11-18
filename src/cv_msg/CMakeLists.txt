cmake_minimum_required(VERSION 3.8)
project(cv_msg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MsgCenter.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
