cmake_minimum_required(VERSION 3.16)
project(rtc_controller_interface VERSION 5.17.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Coverage (gcov/lcov) ─────────────────────────────────────────────────────
option(ENABLE_COVERAGE "Enable gcov code coverage" OFF)
if(ENABLE_COVERAGE)
  add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
  add_link_options(--coverage)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wconversion
    -Wsign-conversion
  )
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rtc_base REQUIRED)
find_package(rtc_msgs REQUIRED)
find_package(rtc_urdf_bridge REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml-cpp REQUIRED)

# ── Library ───────────────────────────────────────────────────────────────────
add_library(rtc_controller_interface SHARED
  src/rt_controller_interface.cpp
  src/controller_registry.cpp
)
target_include_directories(rtc_controller_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rtc_controller_interface
  rtc_base
  rtc_msgs
  rtc_urdf_bridge
  pinocchio
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(rtc_controller_interface yaml-cpp)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS rtc_controller_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# ── Export ────────────────────────────────────────────────────────────────────
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rtc_base
  rtc_msgs
  rtc_urdf_bridge
  pinocchio
  rclcpp
  rclcpp_lifecycle
  yaml-cpp
)
ament_export_include_directories(include)

if(BUILD_TESTING)
  # NOTE: ament_lint_common (test_depend in package.xml) pulls in uncrustify,
  # which enforces ROS 2 / Eclipse-CDT style (Allman braces, `T *p`, `T &r`,
  # `template <T>`). That is fundamentally incompatible with our `.clang-format`
  # (Google base, PointerAlignment Left, Attach braces) — the PostToolUse hook
  # already enforces clang-format on every Edit, so uncrustify is dead lint.
  # The `_FOUND TRUE` skip trick only works for lints whose extras hook gates
  # on that variable; ament_cmake_uncrustify does not, so we depend on the
  # surviving component lints individually (see package.xml test_depend list)
  # instead of pulling them in via ament_lint_common.
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_controller_registry test/test_controller_registry.cpp)
  target_link_libraries(test_controller_registry rtc_controller_interface)

  ament_add_gtest(test_rt_controller_interface test/test_rt_controller_interface.cpp)
  target_link_libraries(test_rt_controller_interface rtc_controller_interface)
  ament_target_dependencies(test_rt_controller_interface rtc_urdf_bridge)

  ament_add_gtest(test_controller_log_set test/test_controller_log_set.cpp)
  target_link_libraries(test_controller_log_set rtc_controller_interface)

  ament_add_gtest(test_controller_log_set_switch
    test/test_controller_log_set_switch.cpp)
  target_link_libraries(test_controller_log_set_switch rtc_controller_interface)
endif()

ament_package()
