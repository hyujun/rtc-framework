cmake_minimum_required(VERSION 3.16)
project(rtc_controllers VERSION 5.17.0 LANGUAGES CXX)

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
find_package(rtc_controller_interface REQUIRED)
find_package(rtc_base REQUIRED)
find_package(rtc_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(urdf_pinocchio_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)

# ── Library ───────────────────────────────────────────────────────────────────
add_library(rtc_controllers STATIC
  src/controllers/indirect/p_controller.cpp
  src/controllers/indirect/clik_controller.cpp
  src/controllers/direct/joint_pd_controller.cpp
  src/controllers/direct/operational_space_controller.cpp
  src/controller_registration.cpp
)
target_include_directories(rtc_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rtc_controllers
  rtc_controller_interface
  rtc_base
  rtc_msgs
  pinocchio
  urdf_pinocchio_bridge
)
target_link_libraries(rtc_controllers yaml-cpp Eigen3::Eigen)

# Position-independent code for static library (needed when linked into shared libs)
set_target_properties(rtc_controllers PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS rtc_controllers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ── Export ────────────────────────────────────────────────────────────────────
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rtc_controller_interface
  rtc_base
  rtc_msgs
  Eigen3
  pinocchio
  urdf_pinocchio_bridge
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
