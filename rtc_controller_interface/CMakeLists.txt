cmake_minimum_required(VERSION 3.16)
project(rtc_controller_interface VERSION 5.17.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
find_package(urdf_pinocchio_bridge REQUIRED)
find_package(pinocchio REQUIRED)
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
  urdf_pinocchio_bridge
  pinocchio
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
  urdf_pinocchio_bridge
  pinocchio
  yaml-cpp
)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
