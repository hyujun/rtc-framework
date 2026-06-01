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

# ── Coverage (gcov/lcov) ─────────────────────────────────────────────────────
option(ENABLE_COVERAGE "Enable gcov code coverage" OFF)
if(ENABLE_COVERAGE)
  add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
  add_link_options(--coverage)
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
  src/contact/contact_manager.cpp
  src/contact/grasp_cache.cpp
  src/tasks/posture_task.cpp
  src/tasks/object_wrench_task.cpp
  src/tasks/internal_force_task.cpp
  src/tasks/object_se3_task.cpp
  src/tasks/se3_task.cpp
  src/tasks/force_task.cpp
  src/tasks/com_task.cpp
  src/tasks/momentum_task.cpp
  src/tasks/contact_consistency_task.cpp
  src/constraints/eom_constraint.cpp
  src/constraints/contact_constraint.cpp
  src/constraints/friction_cone_constraint.cpp
  src/constraints/torque_limit_constraint.cpp
  src/constraints/joint_limit_constraint.cpp
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

  # Path to the Panda URDF used by tests as a generic fixed-base 7-DOF fixture.
  # Default resolves to the repo-vendored copy under robot_descriptions (checked
  # in, so present at build time both locally and in CI — unlike the old
  # deps/src or /usr/local paths that the deps artifact does not ship).
  # pinocchio::buildModel parses kinematics/dynamics only, so the package://
  # meshes are not needed. Override-able via -D or env.
  get_filename_component(_rtc_panda_default
      "${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots/panda/urdf/panda.urdf"
      ABSOLUTE)
  set(RTC_PANDA_URDF_PATH "${_rtc_panda_default}"
      CACHE FILEPATH "Path to panda.urdf used by rtc_tsid tests")
  if(DEFINED ENV{RTC_PANDA_URDF_PATH})
    set(RTC_PANDA_URDF_PATH "$ENV{RTC_PANDA_URDF_PATH}" CACHE FILEPATH
        "Path to panda.urdf used by rtc_tsid tests" FORCE)
  endif()

  ament_add_gtest(test_qp_solver_wrapper test/test_qp_solver_wrapper.cpp)
  target_link_libraries(test_qp_solver_wrapper rtc_tsid)

  ament_add_gtest(test_wbc_types test/test_wbc_types.cpp)
  target_link_libraries(test_wbc_types rtc_tsid)

  ament_add_gtest(test_posture_task test/test_posture_task.cpp)
  target_link_libraries(test_posture_task rtc_tsid)

  ament_add_gtest(test_se3_task test/test_se3_task.cpp)
  target_link_libraries(test_se3_task rtc_tsid)

  ament_add_gtest(test_force_task test/test_force_task.cpp)
  target_link_libraries(test_force_task rtc_tsid)

  ament_add_gtest(test_com_task test/test_com_task.cpp)
  target_link_libraries(test_com_task rtc_tsid)

  ament_add_gtest(test_momentum_task test/test_momentum_task.cpp)
  target_link_libraries(test_momentum_task rtc_tsid)

  ament_add_gtest(test_contact_consistency_task test/test_contact_consistency_task.cpp)
  target_link_libraries(test_contact_consistency_task rtc_tsid)

  ament_add_gtest(test_phase3_integration test/test_phase3_integration.cpp)
  target_link_libraries(test_phase3_integration rtc_tsid)

  ament_add_gtest(test_eom_constraint test/test_eom_constraint.cpp)
  target_link_libraries(test_eom_constraint rtc_tsid)

  ament_add_gtest(test_contact_constraint test/test_contact_constraint.cpp)
  target_link_libraries(test_contact_constraint rtc_tsid)

  ament_add_gtest(test_friction_cone_constraint test/test_friction_cone_constraint.cpp)
  target_link_libraries(test_friction_cone_constraint rtc_tsid)

  ament_add_gtest(test_torque_limit_constraint test/test_torque_limit_constraint.cpp)
  target_link_libraries(test_torque_limit_constraint rtc_tsid)

  ament_add_gtest(test_joint_limit_constraint test/test_joint_limit_constraint.cpp)
  target_link_libraries(test_joint_limit_constraint rtc_tsid)

  ament_add_gtest(test_tsid_wqp test/test_tsid_wqp.cpp)
  target_link_libraries(test_tsid_wqp rtc_tsid)

  ament_add_gtest(test_tsid_hqp test/test_tsid_hqp.cpp)
  target_link_libraries(test_tsid_hqp rtc_tsid)

  ament_add_gtest(test_tsid_wqp_hqp_compare test/test_tsid_wqp_hqp_compare.cpp)
  target_link_libraries(test_tsid_wqp_hqp_compare rtc_tsid)

  ament_add_gtest(test_tsid_performance test/test_tsid_performance.cpp)
  target_link_libraries(test_tsid_performance rtc_tsid)

  # Stage A-5a: fixed-dim QP refactor regression.
  ament_add_gtest(test_fixed_dim_qp test/test_fixed_dim_qp.cpp)
  target_link_libraries(test_fixed_dim_qp rtc_tsid)

  # Stage A-5b: activation ramp + √s weight scaling.
  ament_add_gtest(test_activation_ramp test/test_activation_ramp.cpp)
  target_link_libraries(test_activation_ramp rtc_tsid)
  ament_add_gtest(test_weight_scaling test/test_weight_scaling.cpp)
  target_link_libraries(test_weight_scaling rtc_tsid)

  # Stage B-1: ObjectFrame + Grasp matrix G + ContactManager.
  ament_add_gtest(test_grasp_matrix test/test_grasp_matrix.cpp)
  target_link_libraries(test_grasp_matrix rtc_tsid)

  # Stage B-2: GraspCache (LDLT pinv + P_N) + ObjectWrenchTask.
  ament_add_gtest(test_grasp_cache test/test_grasp_cache.cpp)
  target_link_libraries(test_grasp_cache rtc_tsid)
  ament_add_gtest(test_object_wrench_task test/test_object_wrench_task.cpp)
  target_link_libraries(test_object_wrench_task rtc_tsid)

  # Stage B-3: InternalForceTask (rank-based nullity + P_N reference projection).
  ament_add_gtest(test_internal_force_task test/test_internal_force_task.cpp)
  target_link_libraries(test_internal_force_task rtc_tsid)

  # Stage B-4: ObjectSE3Task ((Gᵀ)⁺·J_c kinematic) + ObjectStateProvider.
  ament_add_gtest(test_object_se3_task test/test_object_se3_task.cpp)
  target_link_libraries(test_object_se3_task rtc_tsid)

  # Apply PANDA_URDF_PATH macro to every test target above.
  foreach(_t
      test_qp_solver_wrapper test_wbc_types test_posture_task test_se3_task
      test_force_task test_com_task test_momentum_task test_contact_consistency_task
      test_phase3_integration
      test_eom_constraint test_contact_constraint test_friction_cone_constraint
      test_torque_limit_constraint test_joint_limit_constraint test_tsid_wqp
      test_tsid_hqp test_tsid_wqp_hqp_compare test_tsid_performance
      test_fixed_dim_qp test_activation_ramp test_weight_scaling
      test_grasp_matrix test_grasp_cache test_object_wrench_task
      test_internal_force_task test_object_se3_task)
    target_compile_definitions(${_t} PRIVATE
        RTC_PANDA_URDF_PATH="${RTC_PANDA_URDF_PATH}")
  endforeach()
endif()

ament_package()
