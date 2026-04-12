cmake_minimum_required(VERSION 3.16)
project(rtc_tsid VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
  )
endif()

# ── Dependencies ──
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)

# ProxSuite (installed at /usr/local via robotpkg)
find_package(proxsuite REQUIRED)

# ── Library ──
add_library(rtc_tsid STATIC
  src/solver/qp_solver_wrapper.cpp
  src/types/wbc_types.cpp
  src/tasks/posture_task.cpp
  src/constraints/eom_constraint.cpp
  src/constraints/contact_constraint.cpp
  src/constraints/friction_cone_constraint.cpp
  src/constraints/torque_limit_constraint.cpp
  src/formulation/wqp_formulation.cpp
  src/formulation/hqp_formulation.cpp
  src/formulation/formulation_factory.cpp
  src/controller/tsid_controller.cpp
)

target_include_directories(rtc_tsid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# pinocchio via ament (sets include dirs + link libs)
ament_target_dependencies(rtc_tsid pinocchio)

target_link_libraries(rtc_tsid
  Eigen3::Eigen
  yaml-cpp
  proxsuite::proxsuite
)

set_target_properties(rtc_tsid PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ── Install ──
install(TARGETS rtc_tsid EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 pinocchio proxsuite yaml-cpp)
ament_export_include_directories(include)

# ── Tests ──
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_qp_solver_wrapper test/test_qp_solver_wrapper.cpp)
  target_link_libraries(test_qp_solver_wrapper rtc_tsid)

  ament_add_gtest(test_wbc_types test/test_wbc_types.cpp)
  target_link_libraries(test_wbc_types rtc_tsid)

  ament_add_gtest(test_posture_task test/test_posture_task.cpp)
  target_link_libraries(test_posture_task rtc_tsid)

  ament_add_gtest(test_eom_constraint test/test_eom_constraint.cpp)
  target_link_libraries(test_eom_constraint rtc_tsid)

  ament_add_gtest(test_contact_constraint test/test_contact_constraint.cpp)
  target_link_libraries(test_contact_constraint rtc_tsid)

  ament_add_gtest(test_friction_cone_constraint test/test_friction_cone_constraint.cpp)
  target_link_libraries(test_friction_cone_constraint rtc_tsid)

  ament_add_gtest(test_torque_limit_constraint test/test_torque_limit_constraint.cpp)
  target_link_libraries(test_torque_limit_constraint rtc_tsid)

  ament_add_gtest(test_tsid_wqp test/test_tsid_wqp.cpp)
  target_link_libraries(test_tsid_wqp rtc_tsid)

  ament_add_gtest(test_tsid_hqp test/test_tsid_hqp.cpp)
  target_link_libraries(test_tsid_hqp rtc_tsid)

  ament_add_gtest(test_tsid_wqp_hqp_compare test/test_tsid_wqp_hqp_compare.cpp)
  target_link_libraries(test_tsid_wqp_hqp_compare rtc_tsid)

  ament_add_gtest(test_tsid_performance test/test_tsid_performance.cpp)
  target_link_libraries(test_tsid_performance rtc_tsid)
endif()

ament_package()
