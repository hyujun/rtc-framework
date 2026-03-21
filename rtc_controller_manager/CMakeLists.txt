cmake_minimum_required(VERSION 3.16)
project(rtc_controller_manager VERSION 5.16.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rtc_controller_interface REQUIRED)
find_package(rtc_controllers REQUIRED)
find_package(rtc_base REQUIRED)
find_package(rtc_communication REQUIRED)
find_package(rtc_status_monitor REQUIRED)
find_package(rtc_msgs REQUIRED)
find_package(ur5e_hand_udp REQUIRED)
find_package(ur5e_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Optional ONNX Runtime
find_package(onnxruntime QUIET)
if(NOT onnxruntime_FOUND)
  find_library(ONNXRUNTIME_LIB onnxruntime HINTS /opt/onnxruntime/lib)
  if(ONNXRUNTIME_LIB)
    add_compile_definitions(HAS_ONNXRUNTIME)
  endif()
else()
  add_compile_definitions(HAS_ONNXRUNTIME)
endif()

add_library(rt_controller_lib STATIC
  src/rt_controller_node.cpp
)
target_include_directories(rt_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rt_controller_lib PUBLIC pinocchio::pinocchio)
ament_target_dependencies(rt_controller_lib
  rclcpp std_msgs sensor_msgs
  rtc_controller_interface rtc_controllers rtc_base
  rtc_communication rtc_status_monitor rtc_msgs
  ur5e_hand_udp ur5e_msgs
  ament_index_cpp
)

add_executable(rt_controller src/rt_controller_main.cpp)
target_link_libraries(rt_controller PRIVATE rt_controller_lib)
ament_target_dependencies(rt_controller rclcpp rtc_base)

install(TARGETS rt_controller DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_export_include_directories(include)
ament_package()
