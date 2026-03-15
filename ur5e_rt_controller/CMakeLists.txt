cmake_minimum_required(VERSION 3.16)
project(ur5e_rt_controller VERSION 5.13.0 LANGUAGES CXX)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 디버깅: CMAKE_PREFIX_PATH 확인 (CI 문제 진단 후 제거 예정)
message(WARNING ">>> CMAKE_PREFIX_PATH = ${CMAKE_PREFIX_PATH}")
message(WARNING ">>> CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")

# ── Workspace 내부 패키지 먼저 찾기 ──────────────────────────────────────────
# pinocchio의 find_dependency() 체인이 CMake 3.31에서 CMAKE_PREFIX_PATH 스코프를
# 오염시켜 이후 find_package()가 colcon install 경로를 찾지 못하는 문제 방지
find_package(ur5e_rt_base REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ur5e_hand_udp REQUIRED)
find_package(ur5e_status_monitor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ur5e_msgs REQUIRED)

# Pre-find Python3 with NumPy so eigenpy's cmake detection succeeds.
# On Ubuntu 24.04 / ROS2 Jazzy, eigenpy's python.cmake fails to detect numpy
# when a venv is active: CMake defaults to FIRST (prefer venv Python) but the
# venv may not have numpy installed. Setting STANDARD forces system Python.
set(Python3_FIND_VIRTUALENV STANDARD)
find_package(Python3 COMPONENTS Interpreter Development NumPy)

find_package(pinocchio REQUIRED)

# ── rt_controller_lib (테스트에서 링크 가능한 라이브러리) ─────────────────────
add_library(rt_controller_lib STATIC
  src/rt_controller_interface.cpp
  src/rt_controller_node.cpp
  src/controllers/direct/joint_pd_controller.cpp
  src/controllers/direct/operational_space_controller.cpp
  src/controllers/indirect/p_controller.cpp
  src/controllers/indirect/clik_controller.cpp
  src/controllers/indirect/ur5e_hand_controller.cpp
)
target_include_directories(rt_controller_lib PUBLIC include)
ament_target_dependencies(rt_controller_lib PUBLIC
  rclcpp
  std_msgs
  sensor_msgs
  ur5e_rt_base
  ur5e_hand_udp
  ur5e_status_monitor
  ament_index_cpp
  ur5e_msgs
)
target_link_libraries(rt_controller_lib PUBLIC pinocchio::pinocchio yaml-cpp)

# ── rt_controller (실행 파일 — main만 컴파일) ─────────────────────────────────
add_executable(rt_controller src/rt_controller_main.cpp)
target_link_libraries(rt_controller PRIVATE rt_controller_lib)

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS rt_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
  scripts/setup_irq_affinity.sh
  scripts/setup_udp_optimization.sh
  scripts/build_rt_kernel.sh
  scripts/setup_nvidia_rt.sh
  scripts/cpu_shield.sh
  scripts/check_rt_setup.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trajectory test/test_trajectory.cpp)
  target_link_libraries(test_trajectory rt_controller_lib)
endif()

ament_package()
