cmake_minimum_required(VERSION 3.16)
project(ur5e_bringup VERSION 5.17.0 LANGUAGES CXX)

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

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rtc_controller_interface REQUIRED)
find_package(rtc_controllers REQUIRED)
find_package(rtc_base REQUIRED)
find_package(rtc_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Pre-find Python3 with NumPy so eigenpy's cmake detection succeeds.
set(Python3_FIND_VIRTUALENV STANDARD)
find_package(Python3 COMPONENTS Interpreter Development NumPy)

find_package(pinocchio REQUIRED)
find_package(rtc_urdf_bridge REQUIRED)
find_package(rtc_controller_manager REQUIRED)
find_package(rtc_tsid REQUIRED)
find_package(rtc_mpc REQUIRED)
find_package(ur5e_description REQUIRED)

# ── Static library: demo controllers ──────────────────────────────────────────
add_library(${PROJECT_NAME}_demo_controllers STATIC
  src/controllers/demo_joint_controller.cpp
  src/controllers/demo_task_controller.cpp
  src/controllers/demo_wbc_controller.cpp
  src/controllers/demo_shared_config.cpp
  src/controllers/controller_registration.cpp
  src/controllers/owned_topics.cpp
  src/phase/grasp_phase_manager.cpp
)

target_include_directories(${PROJECT_NAME}_demo_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_demo_controllers
  ament_index_cpp
  rclcpp
  std_msgs
  sensor_msgs
  rtc_controller_interface
  rtc_controllers
  rtc_base
  rtc_msgs
  pinocchio
  rtc_urdf_bridge
  rtc_tsid
  rtc_mpc
  ur5e_description
)

target_link_libraries(${PROJECT_NAME}_demo_controllers
  yaml-cpp
)

# ── UR5e-specific executable (links built-in + demo controllers) ──────────────
# --whole-archive prevents the linker from stripping static-init registration
# TUs, eliminating the need for ForceDemoControllerRegistration().
add_executable(ur5e_rt_controller
  src/ur5e_rt_controller_main.cpp
)
target_link_libraries(ur5e_rt_controller
  -Wl,--whole-archive
  ${PROJECT_NAME}_demo_controllers
  -Wl,--no-whole-archive
)
ament_target_dependencies(ur5e_rt_controller
  rclcpp
  rclcpp_lifecycle
  rtc_base
  rtc_controller_manager
  rtc_controllers
)

# ── Install headers ───────────────────────────────────────────────────────────
install(
  DIRECTORY include/
  DESTINATION include
)

# ── Install library ───────────────────────────────────────────────────────────
install(
  TARGETS ${PROJECT_NAME}_demo_controllers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ── Install executable ────────────────────────────────────────────────────────
install(TARGETS ur5e_rt_controller
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install config ────────────────────────────────────────────────────────────
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ── Install launch files ──────────────────────────────────────────────────────
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── Install Python scripts ───────────────────────────────────────────────────
install(
  PROGRAMS scripts/demo_controller_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME demo_controller_gui
)

install(
  PROGRAMS scripts/motion_editor_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motion_editor_gui
)

# ── Export ─────────────────────────────────────────────────────────────────────
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  rtc_controller_interface
  rtc_controllers
  rtc_controller_manager
  rtc_base
  rtc_msgs
  pinocchio
  rtc_urdf_bridge
  rtc_tsid
  rtc_mpc
  yaml-cpp
)
ament_export_include_directories(include)

# ── Testing ───────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Eigen3 REQUIRED)

  # ── virtual_tcp header-only tests ──────────────────────────────────────────
  ament_add_gtest(test_virtual_tcp test/test_virtual_tcp.cpp)
  target_include_directories(test_virtual_tcp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
  )
  ament_target_dependencies(test_virtual_tcp
    rtc_base
    pinocchio
    Eigen3
  )

  # ── demo_wbc_controller tests ──────────────────────────────────────────────
  ament_add_gtest(test_demo_wbc_controller test/test_demo_wbc_controller.cpp)
  target_link_libraries(test_demo_wbc_controller
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_demo_wbc_controller
    rclcpp
    rtc_controllers
    rtc_base
    pinocchio
    rtc_tsid
    Eigen3
  )

  # ── demo_wbc_controller MPC integration tests (Phase 5) ────────────────────
  ament_add_gtest(test_demo_wbc_mpc_integration
    test/test_demo_wbc_mpc_integration.cpp)
  target_link_libraries(test_demo_wbc_mpc_integration
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_demo_wbc_mpc_integration
    rclcpp
    rtc_controllers
    rtc_base
    pinocchio
    rtc_tsid
    rtc_mpc
    Eigen3
  )

  # ── GraspPhaseManager FSM tests (Phase 7a) ─────────────────────────────────
  ament_add_gtest(test_grasp_phase_manager test/test_grasp_phase_manager.cpp)
  target_link_libraries(test_grasp_phase_manager
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_grasp_phase_manager
    rclcpp
    rtc_base
    pinocchio
    rtc_mpc
    Eigen3
  )

  # ── Grasp handler pipeline integration (Phase 7c) ──────────────────────────
  # Wires GraspPhaseManager + MPCFactory + HandlerMPCThread end-to-end on the
  # Panda fixture. Exercises the same surface DemoWbcController drives in
  # handler-mode, including the ForcePhase → cross-mode swap path.
  ament_add_gtest(test_grasp_pipeline test/test_grasp_pipeline.cpp)
  target_link_libraries(test_grasp_pipeline
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_grasp_pipeline
    rclcpp
    rtc_base
    pinocchio
    rtc_mpc
    Eigen3
  )

  # ── demo_shared_config tests (links the demo-controllers static lib) ───────
  ament_add_gtest(test_demo_shared_config test/test_demo_shared_config.cpp)
  target_link_libraries(test_demo_shared_config
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_demo_shared_config
    rclcpp
    rtc_controllers
    rtc_base
    pinocchio
    Eigen3
  )
endif()

ament_package()
