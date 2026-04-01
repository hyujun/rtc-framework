cmake_minimum_required(VERSION 3.16)
project(shape_estimation_msgs VERSION 5.17.0 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ToFReadings.msg"
  "msg/TipPoses.msg"
  "msg/ToFSnapshot.msg"
  "msg/ShapeEstimate.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
