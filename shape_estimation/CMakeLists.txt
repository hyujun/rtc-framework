cmake_minimum_required(VERSION 3.16)
project(shape_estimation VERSION 5.17.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
  )
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(shape_estimation_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rtc_msgs REQUIRED)

# ── Core library (ROS 비의존) ─────────────────────────────────────────────────
add_library(shape_estimation_core STATIC
  src/voxel_point_cloud.cpp
  src/fast_shape_classifier.cpp
  src/primitive_fitter.cpp
  src/exploration_motion.cpp
  src/snapshot_history.cpp
  src/protuberance_detector.cpp
)
target_include_directories(shape_estimation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(shape_estimation_core Eigen3::Eigen)
ament_target_dependencies(shape_estimation_core rclcpp)
set_target_properties(shape_estimation_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ── ROS library (core + ROS 래핑) ────────────────────────────────────────────
add_library(shape_estimation_ros STATIC
  src/msg_conversions.cpp
  src/rviz_markers.cpp
  src/shape_estimation_node.cpp
)
target_include_directories(shape_estimation_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(shape_estimation_ros shape_estimation_core)
ament_target_dependencies(shape_estimation_ros
  rclcpp rclcpp_action std_msgs std_srvs sensor_msgs geometry_msgs
  visualization_msgs shape_estimation_msgs rtc_msgs
)
set_target_properties(shape_estimation_ros PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ── Executable ───────────────────────────────────────────────────────────────
add_executable(shape_estimation_node src/shape_estimation_node_main.cpp)
target_link_libraries(shape_estimation_node shape_estimation_ros)
ament_target_dependencies(shape_estimation_node rclcpp)

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS shape_estimation_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS shape_estimation_core shape_estimation_ros
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# ── Export ────────────────────────────────────────────────────────────────────
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp rclcpp_action Eigen3 std_msgs std_srvs sensor_msgs
  geometry_msgs visualization_msgs shape_estimation_msgs rtc_msgs
)
ament_export_include_directories(include)

# ── Testing ──────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_tof_shape test/test_tof_shape.cpp)
  target_link_libraries(test_tof_shape shape_estimation_core)

  ament_add_gtest(test_exploration_motion test/test_exploration_motion.cpp)
  target_link_libraries(test_exploration_motion shape_estimation_core)

  ament_add_gtest(test_protuberance_detector test/test_protuberance_detector.cpp)
  target_link_libraries(test_protuberance_detector shape_estimation_core)
endif()

ament_package()
