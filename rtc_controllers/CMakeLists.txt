cmake_minimum_required(VERSION 3.16)
project(rtc_controllers VERSION 5.16.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(rtc_controller_interface REQUIRED)
find_package(rtc_base REQUIRED)
find_package(rtc_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pinocchio REQUIRED)

add_library(rtc_controllers STATIC
  src/controllers/indirect/p_controller.cpp
  src/controllers/indirect/clik_controller.cpp
  src/controllers/direct/joint_pd_controller.cpp
  src/controllers/direct/operational_space_controller.cpp
)
target_include_directories(rtc_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rtc_controllers PUBLIC pinocchio::pinocchio)
ament_target_dependencies(rtc_controllers
  rtc_controller_interface rtc_base rtc_msgs rclcpp sensor_msgs
)

install(TARGETS rtc_controllers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rtc_controller_interface rtc_base rtc_msgs rclcpp sensor_msgs pinocchio)
ament_export_include_directories(include)
ament_package()
