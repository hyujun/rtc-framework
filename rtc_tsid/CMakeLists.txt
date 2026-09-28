cmake_minimum_required(VERSION 3.16)
project(rtc_tsid VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
    -Werror=switch  # enum switch 누락은 dead topic 을 낳는다 (#211)
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
find_package(rtc_math REQUIRED)
# #unified-kindyn Phase 1: PinocchioCache / ReducedDynamicsProvider 이관 목적지 (ARCH-2 순방향).
find_package(rtc_urdf_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)

# ProxSuite (installed at /usr/local via robotpkg)
find_package(proxsuite REQUIRED)

# ── Library ──
add_library(rtc_tsid STATIC
  src/solver/qp_solver_wrapper.cpp
  src/types/wbc_types.cpp
  src/contact/contact_manager.cpp
  src/contact/grasp_cache.cpp
  src/kinematics/clik_reference.cpp
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

# pinocchio + rtc_math (SE(3) error core) via ament (sets include dirs + link libs)
ament_target_dependencies(rtc_tsid pinocchio rtc_math rtc_urdf_bridge)

target_link_libraries(rtc_tsid
  pinocchio::pinocchio
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
ament_export_dependencies(Eigen3 pinocchio proxsuite yaml-cpp rtc_urdf_bridge)
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
  #
  # NOT an ARCH-5/E-10 violation (user decision, 2026-07 docs audit): this is a
  # test-only configure-time path — no find_package/<depend>/
  # ament_target_dependencies edge on robot_descriptions exists, so no colcon
  # build-graph edge is created. Do not "fix" it toward ament_index or
  # vendoring (regression risk for zero gain); the only harmless strengthening
  # would be a <test_depend>robot_descriptions</test_depend> for install order.
  get_filename_component(_rtc_panda_default
      "${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots/panda/urdf/panda.urdf"
      ABSOLUTE)
  set(RTC_PANDA_URDF_PATH "${_rtc_panda_default}"
      CACHE FILEPATH "Path to panda.urdf used by rtc_tsid tests")
  if(DEFINED ENV{RTC_PANDA_URDF_PATH})
    set(RTC_PANDA_URDF_PATH "$ENV{RTC_PANDA_URDF_PATH}" CACHE FILEPATH
        "Path to panda.urdf used by rtc_tsid tests" FORCE)
  endif()

  # One test binary per file: linked to rtc_tsid and given the Panda path
  # (test/panda_fixture.hpp).
  function(add_tsid_gtest name)
    ament_add_gtest(${name} test/${name}.cpp)
    target_link_libraries(${name} rtc_tsid)
    target_compile_definitions(${name} PRIVATE RTC_PANDA_URDF_PATH="${RTC_PANDA_URDF_PATH}")
  endfunction()

  add_tsid_gtest(test_qp_solver_wrapper)

  add_tsid_gtest(test_wbc_types)

  add_tsid_gtest(test_posture_task)

  add_tsid_gtest(test_se3_task)

  add_tsid_gtest(test_task_se3_error)

  add_tsid_gtest(test_force_task)

  add_tsid_gtest(test_com_task)

  add_tsid_gtest(test_momentum_task)

  add_tsid_gtest(test_contact_consistency_task)

  add_tsid_gtest(test_phase3_integration)

  add_tsid_gtest(test_eom_constraint)

  add_tsid_gtest(test_contact_constraint)

  add_tsid_gtest(test_friction_cone_constraint)

  add_tsid_gtest(test_torque_limit_constraint)

  add_tsid_gtest(test_joint_limit_constraint)

  add_tsid_gtest(test_tsid_wqp)

  add_tsid_gtest(test_tsid_hqp)

  add_tsid_gtest(test_tsid_wqp_hqp_compare)

  add_tsid_gtest(test_tsid_performance)

  # Stage A-5a: fixed-dim QP refactor regression.
  add_tsid_gtest(test_fixed_dim_qp)

  # Stage A-5b: activation ramp + √s weight scaling.
  add_tsid_gtest(test_activation_ramp)
  add_tsid_gtest(test_weight_scaling)

  # Stage B-1: ObjectFrame + Grasp matrix G + ContactManager.
  add_tsid_gtest(test_grasp_matrix)

  # Stage B-2: GraspCache (LDLT pinv + P_N) + ObjectWrenchTask.
  add_tsid_gtest(test_grasp_cache)
  add_tsid_gtest(test_object_wrench_task)

  # Stage B-3: InternalForceTask (rank-based nullity + P_N reference projection).
  add_tsid_gtest(test_internal_force_task)

  # Stage B-4: ObjectSE3Task ((Gᵀ)⁺·J_c kinematic) + ObjectStateProvider.
  add_tsid_gtest(test_object_se3_task)

  # Stage C-1: ClikReferenceGenerator (velocity-level CLIK reference).
  add_tsid_gtest(test_clik_reference)

  # dynamic_catching S2.2a: golden-vector regression of the default CLIK path.
  # Bit-exact only in the configuration the table was recorded in (Release, no
  # sanitizer); other builds compare with a relative tolerance (see the test
  # file header).
  add_tsid_gtest(test_clik_golden)
  target_include_directories(test_clik_golden PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  if(CMAKE_BUILD_TYPE STREQUAL "Release" AND NOT CMAKE_CXX_FLAGS MATCHES "sanitize")
    target_compile_definitions(test_clik_golden PRIVATE RTC_CLIK_GOLDEN_EXACT=1)
  endif()

  # dynamic_catching S2.2b: CLIK options (each defaults off).
  add_tsid_gtest(test_clik_options)

  # dynamic_catching decision K (S6-C2): kinematic / dynamic acceleration rows,
  # judged against pinocchio's RNEA / classical acceleration on a fresh Data.
  add_tsid_gtest(test_clik_accel_constraint)
endif()

ament_package()
