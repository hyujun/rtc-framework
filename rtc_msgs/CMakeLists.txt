cmake_minimum_required(VERSION 3.16)
project(rtc_msgs VERSION 5.17.0 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointCommand.msg"
  "msg/FingertipSensor.msg"
  "msg/HandSensorState.msg"
  "msg/GuiPosition.msg"
  "msg/RobotTarget.msg"
  "msg/DeviceStateLog.msg"
  "msg/DeviceSensorLog.msg"
  "msg/GraspState.msg"
  "msg/SimSensor.msg"
  "msg/SimSensorState.msg"
  "msg/ToFSnapshot.msg"
  "msg/WbcState.msg"
  "msg/CalibrationCommand.msg"
  "msg/CalibrationStatus.msg"
  "msg/ControllerState.msg"
  "srv/ListControllers.srv"
  "srv/SwitchController.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
