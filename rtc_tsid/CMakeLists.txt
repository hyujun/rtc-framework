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

  # URDF fixtures this package owns (test/tree_fixture.hpp).
  set(RTC_TSID_TEST_URDF_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test/urdf")

  # add_tsid_gtest(<target> [<stem>...]): one test binary linked to rtc_tsid and
  # given the Panda path (test/panda_fixture.hpp) and the fixture directory. Without stems it builds
  # test/<target>.cpp; with stems those files share the binary, so the suite
  # pays one process start and one link instead of one per file. A file that
  # replaces the global operator new (test/alloc_counter.hpp or an inline copy)
  # stays in a binary of its own — a replacement is one definition per program.
  # The timeout is the default 60 s per file, so a shared binary keeps the
  # budget each file had on its own.
  function(add_tsid_gtest name)
    set(_stems ${ARGN})
    if(NOT _stems)
      set(_stems ${name})
    endif()
    list(LENGTH _stems _n)
    math(EXPR _timeout "60 * ${_n}")
    list(TRANSFORM _stems PREPEND test/)
    list(TRANSFORM _stems APPEND .cpp)
    ament_add_gtest(${name} ${_stems} TIMEOUT ${_timeout})
    target_link_libraries(${name} rtc_tsid)
    target_compile_definitions(${name} PRIVATE
      RTC_PANDA_URDF_PATH="${RTC_PANDA_URDF_PATH}"
      RTC_TSID_TEST_URDF_DIR="${RTC_TSID_TEST_URDF_DIR}")
  endfunction()

  # QP wrapper, WBC types, the fixed-dim QP refactor (Stage A-5a) and the
  # activation ramp + √s weight scaling (Stage A-5b).
  add_tsid_gtest(test_tsid_core
    test_qp_solver_wrapper
    test_wbc_types
    test_fixed_dim_qp
    test_activation_ramp
    test_weight_scaling)

  # Task residuals / Jacobians.
  add_tsid_gtest(test_tsid_tasks
    test_posture_task
    test_task_se3_error
    test_force_task
    test_com_task
    test_momentum_task
    test_contact_consistency_task)

  # Equality / inequality constraints.
  add_tsid_gtest(test_tsid_constraints
    test_eom_constraint
    test_contact_constraint
    test_friction_cone_constraint
    test_torque_limit_constraint
    test_joint_limit_constraint)

  # WQP / HQP formulations end to end.
  add_tsid_gtest(test_tsid_formulations
    test_tsid_wqp
    test_tsid_hqp
    test_tsid_wqp_hqp_compare
    test_tsid_performance
    test_phase3_integration)

  # Stage B-1..B-3: ObjectFrame + grasp matrix G + ContactManager, GraspCache
  # (LDLT pinv + P_N) + ObjectWrenchTask, InternalForceTask (rank-based nullity
  # + P_N reference projection).
  add_tsid_gtest(test_tsid_grasp
    test_grasp_matrix
    test_grasp_cache
    test_object_wrench_task
    test_internal_force_task)

  # Own binaries from here on: test_se3_task / test_object_se3_task include
  # test/alloc_counter.hpp, the three CLIK suites carry an inline replacement,
  # and test_clik_golden needs its own compile definition.
  add_tsid_gtest(test_se3_task)

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

  # E2-F04 (#636): the multi-frame Compute() overload. Replays the universe
  # scenarios of the golden table through it (test/clik_golden_scenarios.hpp),
  # and runs on three models: the Panda, the 17-DoF tree above, and the
  # sibling rtc_urdf_bridge's serial 6-DoF fixture (source-tree path, test-only
  # — the layout rtc_controllers' tests already rely on), whose 6 columns put
  # the cost products on Eigen's coefficient-based kernel instead of GEMM.
  set(_rtc_urdf_bridge_test_include "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_urdf_bridge/test")
  add_tsid_gtest(test_clik_multiframe)
  target_include_directories(test_clik_multiframe PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test ${_rtc_urdf_bridge_test_include})

  # RT-1 for the multi-frame Compute(): the C-level malloc gate, which sees
  # allocations made inside librtc_tsid / libpinocchio / ProxQP — an
  # operator-new counter does not (Eigen has its own allocator). The gate is
  # rtc_base's never-installed test header (test_depend only; rtc_base does
  # not depend on rtc_tsid, so no cycle). It defines malloc in the executable,
  # hence a binary of its own.
  find_package(rtc_base REQUIRED)
  add_tsid_gtest(test_clik_multiframe_malloc)
  rtc_base_add_test_includes(test_clik_multiframe_malloc)
endif()

ament_package()
