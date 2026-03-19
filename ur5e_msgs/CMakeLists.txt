cmake_minimum_required(VERSION 3.16)
project(ur5e_msgs VERSION 5.16.0 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointCommand.msg"
  "msg/HandCommand.msg"
  "msg/HandMotorState.msg"
  "msg/FingertipSensor.msg"
  "msg/HandSensorState.msg"
  "msg/FingertipForceTorque.msg"
  "msg/HandForceTorqueState.msg"
  DEPENDENCIES std_msgs
)

ament_package()
