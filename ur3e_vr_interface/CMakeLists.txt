cmake_minimum_required(VERSION 3.8)
project(ur3e_vr_interface)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PosRot.msg"
  "srv/SetGripper.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY
  msg
  srv
  ur3e_vr_interface  # สำหรับ __init__.py
  DESTINATION share/${PROJECT_NAME})

ament_package()
