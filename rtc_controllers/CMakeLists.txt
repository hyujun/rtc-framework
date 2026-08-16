cmake_minimum_required(VERSION 3.16)
project(rtc_controllers VERSION 5.17.0 LANGUAGES CXX)

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
    -Werror=switch  # enum switch 누락은 dead topic 을 낳는다 (#211)
  )
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rtc_base REQUIRED)
find_package(rtc_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rtc_math REQUIRED)
find_package(rtc_urdf_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)

# ── Library ───────────────────────────────────────────────────────────────────
add_library(rtc_controllers STATIC
  src/controllers/grasp/grasp_controller.cpp
  src/controllers/grasp/pull_force_estimator.cpp
  # Layer 1 of #135: robot-agnostic momentum-observer residual. Takes assembled
  # nv-vectors (p, C^T v, g, tau_m) — no Pinocchio, no matrices, no robot facts.
  src/controllers/estimation/momentum_observer.cpp
  src/controllers/estimation/inertial_estimator.cpp
  src/controllers/estimation/payload_estimator.cpp
  # G2 schema layer (#236 S7c-1): gains PODs + YAML parsers, lifted out of the
  # controllers/{direct,indirect} adapters so they would outlive them — which
  # they now do, the adapters having been deleted at S7c-2. Non-RT, yaml-cpp
  # only; a binding calls these once at configure time.
  src/params/joint_pd_params.cpp
  src/params/osc_params.cpp
  src/params/clik_params.cpp
  src/params/task_impedance_params.cpp
  src/params/task_admittance_params.cpp
  src/params/cascaded_compliance_params.cpp
  src/controller_registration.cpp
)
target_include_directories(rtc_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rtc_controllers
  rtc_base
  rtc_msgs
  pinocchio
  rtc_math
  rtc_urdf_bridge
)
target_link_libraries(rtc_controllers pinocchio::pinocchio yaml-cpp Eigen3::Eigen)

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

install(DIRECTORY examples/
  DESTINATION share/${PROJECT_NAME}/examples
)

# ── Export ────────────────────────────────────────────────────────────────────
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rtc_base
  rtc_msgs
  Eigen3
  pinocchio
  rtc_math
  rtc_urdf_bridge
  yaml-cpp
)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # rtc_controllers tests reuse the shared URDF-fixture path resolver from
  # rtc_urdf_bridge/test (test_urdf_path.hpp) — a test-only, never-installed
  # header pulled in by source-tree path. This is the same sibling-package
  # layout assumption these tests already rely on to locate the URDF fixtures
  # under rtc_urdf_bridge/test/urdf/.
  set(rtc_urdf_bridge_test_include
      "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_urdf_bridge/test")

  # Compliance core helpers (task dynamics / safety layer / state machine /
  # torque E-STOP) — pure-Eigen algebraic tests, no URDF fixture needed. Links
  # rtc_controllers for the include path + Eigen; the zero-allocation gate rides
  # rtc_base/testing/no_malloc_scope.hpp, a test-only, never-installed header
  # (rtc_base keeps its runtime Eigen-free and does not ship testing/), pulled in
  # by source-tree path — the same sibling-package layout used for the
  # rtc_urdf_bridge/test fixture headers above.
  # Path owned by rtc_base (cmake/rtc_base-test-include.cmake.in), which fails
  # loudly if its source tree is not reachable — see #356.
  rtc_base_test_include_dir(rtc_base_test_include)

  # This package's own test-only headers (bitwise-comparison primitives shared
  # by the #236 extraction golden-vector suites). Same never-installed,
  # source-tree-path idiom as the sibling-package includes above: ament's
  # symlink install ignores install(PATTERN EXCLUDE), so a test header placed
  # under include/ would ship.
  set(rtc_controllers_test_include "${CMAKE_CURRENT_SOURCE_DIR}/test/include")

  ament_add_gtest(test_compliance_core test/test_compliance_core.cpp)
  target_link_libraries(test_compliance_core rtc_controllers)
  target_include_directories(test_compliance_core PRIVATE
    ${rtc_base_test_include} ${rtc_controllers_test_include})

  # Slice-3 numerical core (#236): §7.2 semi-implicit integration + exp3 retract,
  # §7.5 workspace/velocity guards, §7.3 DLS + nullspace algebra. Model-free —
  # the URDF-bound half lives in test_task_admittance_controller.
  ament_add_gtest(test_admittance_core test/test_admittance_core.cpp)
  target_link_libraries(test_admittance_core rtc_controllers)

  ament_add_gtest(test_quintic_blend_trajectory test/test_quintic_blend_trajectory.cpp)
  target_link_libraries(test_quintic_blend_trajectory rtc_controllers)

  ament_add_gtest(test_quintic_spline_trajectory test/test_quintic_spline_trajectory.cpp)
  target_link_libraries(test_quintic_spline_trajectory rtc_controllers)

  ament_add_gtest(test_task_space_blend_trajectory test/test_task_space_blend_trajectory.cpp)
  target_link_libraries(test_task_space_blend_trajectory rtc_controllers)

  ament_add_gtest(test_task_space_spline_trajectory test/test_task_space_spline_trajectory.cpp)
  target_link_libraries(test_task_space_spline_trajectory rtc_controllers)

  ament_add_gtest(test_task_space_trajectory test/test_task_space_trajectory.cpp)
  target_link_libraries(test_task_space_trajectory rtc_controllers)

  ament_add_gtest(test_grasp_controller test/test_grasp_controller.cpp)
  target_link_libraries(test_grasp_controller rtc_controllers)

  ament_add_gtest(test_pull_force_estimator test/test_pull_force_estimator.cpp)
  target_link_libraries(test_pull_force_estimator rtc_controllers)

  ament_add_gtest(test_momentum_observer test/test_momentum_observer.cpp)
  target_link_libraries(test_momentum_observer rtc_controllers)

  ament_add_gtest(test_payload_estimator test/test_payload_estimator.cpp)
  target_link_libraries(test_payload_estimator rtc_controllers)

  ament_add_gtest(test_inertial_estimator test/test_inertial_estimator.cpp)
  target_link_libraries(test_inertial_estimator rtc_controllers)

  # #455 RT-1 gate for the Layer 2B estimator. TWO complementary sensors, and
  # the second one is the load-bearing half here: Update() forms Y₄ᵀY₄ from a
  # block of a dynamically-sized Eigen::Ref and then runs a self-adjoint
  # eigendecomposition + LDLT, so the allocation most likely to appear is
  # Eigen's own — and Eigen calls std::malloc directly, which global
  # operator-new counting cannot see. Hence ${rtc_base_test_include} alongside
  # ${rtc_controllers_test_include}, with the same program-wide
  # -Wmismatched-new-delete false positive the other alloc gates carry.
  ament_add_gtest(test_inertial_estimator_alloc test/test_inertial_estimator_alloc.cpp)
  target_link_libraries(test_inertial_estimator_alloc rtc_controllers)
  target_include_directories(test_inertial_estimator_alloc PRIVATE
    ${rtc_base_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_inertial_estimator_alloc PRIVATE -Wno-mismatched-new-delete)

  # #236 S1 golden-vector harness: the joint-space PD core pinned bit-for-bit
  # against a literal copy of the pre-extraction inline form. That literal oracle
  # is the durable one and is what remains; the transient oracle (the live
  # JointPDController driven through a shim) retired with the adapter at S7c-2,
  # taking the planar_3r URDF fixture — and so ${rtc_urdf_bridge_test_include} —
  # with it. JointPdShim stays as the reference recipe an S7 binding starts from,
  # unpinned by design. Needs the shared bitwise helpers. The RT allocation gate
  # replaces global operator new/delete, which makes GCC false-positive
  # -Wmismatched-new-delete program-wide.
  ament_add_gtest(test_joint_pd_core test/test_joint_pd_core.cpp)
  target_link_libraries(test_joint_pd_core rtc_controllers)
  target_include_directories(test_joint_pd_core PRIVATE ${rtc_controllers_test_include})
  target_compile_options(test_joint_pd_core PRIVATE -Wno-mismatched-new-delete)

  # #236 S2a golden-vector suite for the task-space acceleration core: bitwise
  # against a literal copy of the pre-extraction inline form (durable oracle).
  # The cross-check against the live OperationalSpaceController retired with the
  # adapter at S7c-2, with its verification provenance recorded in the file.
  # OscShim survives and is still driven here — shim-only, by the posture-gate
  # cases (분류 B) — so it keeps reproducing the whole Λ/τ/Nᵀ tail and still needs
  # the URDF fixture resolver for serial_7dof (non-zero gravity, redundant so Nᵀ
  # is not inert; the fixture itself is shared via
  # test/include/rtc_controllers/testing/serial7dof_fixture.hpp, which is what
  # pulls in ${rtc_urdf_bridge_test_include}). Same shared global-operator-new
  # gate as S1 with the same program-wide -Wmismatched-new-delete false positive,
  # PLUS rtc_base's Eigen allocation tripwire (${rtc_base_test_include}) —
  # operator-new counting cannot see Eigen's allocator, which calls std::malloc
  # directly.
  ament_add_gtest(test_task_accel_core test/test_task_accel_core.cpp)
  target_link_libraries(test_task_accel_core rtc_controllers)
  target_include_directories(test_task_accel_core PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_task_accel_core PRIVATE -Wno-mismatched-new-delete)

  # #236 S3a golden-vector suite for the task-space velocity core: bitwise
  # against literal copies of BOTH pre-extraction inline forms (six-axis and
  # translation-only are different expression shapes, so each needs its own
  # durable oracle). The cross-check against the live ClikController — all four
  # (control_6dof × enable_null_space) modes plus the π-rotation split — retired
  # with the adapter at S7c-2; ClikShim remains as the reference recipe for a
  # binding, unpinned by design. Reuses test_task_accel_core's independent-handle
  # premise via the same serial_7dof fixture (the reorder half of that premise is
  # measured here, which is what still needs
  # ${rtc_urdf_bridge_test_include}). The RT gate is
  # two complementary sensors: the shared global-operator-new gate as in S1/S2a
  # (hence ${rtc_controllers_test_include} and the same program-wide
  # -Wmismatched-new-delete false positive) PLUS rtc_base's Eigen allocation
  # tripwire (${rtc_base_test_include}) — operator-new counting cannot see Eigen's
  # allocator, which calls std::malloc directly.
  ament_add_gtest(test_task_vel_core test/test_task_vel_core.cpp)
  target_link_libraries(test_task_vel_core rtc_controllers)
  target_include_directories(test_task_vel_core PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_task_vel_core PRIVATE -Wno-mismatched-new-delete)

  # #236 S4 golden-vector suite for the §6.3 inertia-shaping core: bitwise
  # against a literal copy of the pre-extraction member (which keeps the
  # Eigen::MatrixXd scratch, so the suite MEASURES that D-S4a's max-size scratch
  # is bit-inert instead of inheriting the probe's claim). The cross-check
  # against the live TaskImpedanceController (selection × Λ_d × clamp, plus the
  # activation ramp scaling the core's f_ext input) retired with the adapter at
  # S7c-2. ImpedanceShim survives and is still driven shim-only by the Λ_d
  # factorisation-degrade case, so it keeps replicating the whole τ tail. The RT
  # gate is two complementary sensors as in S1/S2a/S3a — the shared global
  # operator-new gate (hence ${rtc_controllers_test_include} and the same
  # program-wide -Wmismatched-new-delete false positive) PLUS rtc_base's Eigen
  # tripwire (${rtc_base_test_include}), which is the only one that can see the
  # max-size scratch regressing to a heap-allocating dynamic type.
  ament_add_gtest(test_inertia_shaping_core test/test_inertia_shaping_core.cpp)
  target_link_libraries(test_inertia_shaping_core rtc_controllers)
  target_include_directories(test_inertia_shaping_core PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_inertia_shaping_core PRIVATE -Wno-mismatched-new-delete)

  # #236 S5 golden-vector suite for the §7.3 task-velocity REUSE. No new core:
  # the admittance law calls S3a's task::ComputeTaskVelocity, so the suite pins
  # that the call is inert — bitwise against a literal copy of the
  # pre-extraction inline shape (ν_c copied, K⊙e accumulated into it, i.e. the
  # operands the other way round from the core). The cross-check against the live
  # TaskAdmittanceController (integrate_from_measured × nullspace_kp × ik_kp,
  # plus the activation ramp feeding the §7.2 integrator) retired with the
  # adapter at S7c-2; AdmittanceShim remains as the reference recipe, composing
  # AdmittanceIntegrator / DifferentialIk rather than re-deriving them, and is
  # still what S6's CascadeShim is built from. Same two-sensor RT gate as
  # S1/S2a/S3a/S4 — the shared global operator-new gate (hence
  # ${rtc_controllers_test_include} and the program-wide
  # -Wmismatched-new-delete false positive) PLUS rtc_base's Eigen tripwire
  # (${rtc_base_test_include}), which is the only one that can see a fixed-size
  # marshalling argument regressing to a dynamic Eigen type.
  ament_add_gtest(test_admittance_task_vel_reuse test/test_admittance_task_vel_reuse.cpp)
  target_link_libraries(test_admittance_task_vel_reuse rtc_controllers)
  target_include_directories(test_admittance_task_vel_reuse PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_admittance_task_vel_reuse PRIVATE -Wno-mismatched-new-delete)

  # #236 S6 golden-vector suite for the two cores lifted out of the cascade: the
  # null-space posture law (joint/posture_law.hpp, PD and P — two functions
  # because the K_d = 0 reduction flips the sign of a zero on a reachable state)
  # and the §7.6 MUST-1 verdict (compliance/bandwidth_separation.hpp). Literal
  # oracles pin all three pre-extraction shapes. The cross-check against the live
  # CascadedComplianceController retired with the adapter at S7c-2, but
  # CascadeShim — composed from S4's ImpedanceShim and S5's AdmittanceShim —
  # survives and is driven shim-only by the §7.6 evaluation-gate cases. Both
  # gates are numerically INERT when closed, so the suite reads
  # diag.nullspace_active and diag.bandwidth_ratio rather than a torque lane, and
  # exercises both ways the §7.6 gate reopens (set_gains, and a retry after a
  # failed factorisation). Same two-sensor RT
  # gate as S1–S5 — the shared global operator-new gate (hence
  # ${rtc_controllers_test_include} and the program-wide
  # -Wmismatched-new-delete false positive) PLUS rtc_base's Eigen tripwire
  # (${rtc_base_test_include}), which is the only one that can see the Λ_S Ref
  # acquiring a stride it has to heap-copy.
  ament_add_gtest(test_posture_core test/test_posture_core.cpp)
  target_link_libraries(test_posture_core rtc_controllers)
  target_include_directories(test_posture_core PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_posture_core PRIVATE -Wno-mismatched-new-delete)

  # #236 S2b+S3b — the two-tier convergence harness for the constant-λ DLS
  # outliers (OSC's Λ/τ/Nᵀ, CLIK's damped pseudoinverse). Needs the Eigen
  # tripwire include as well: the migration's specific RT-1 risk is an
  # Eigen::Ref argument silently heap-copying, which operator-new interposition
  # alone cannot see.
  ament_add_gtest(test_dls_convergence test/test_dls_convergence.cpp)
  target_link_libraries(test_dls_convergence rtc_controllers)
  target_include_directories(test_dls_convergence PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_dls_convergence PRIVATE -Wno-mismatched-new-delete)

  # G2 schema layer contract (#236 S7c-1). Drives the ParseXxxParams free
  # functions directly — no controller, no URDF — so these cases outlive the
  # adapters. Also carries the over-long-sequence sensor that the adapter-driven
  # config suite structurally could not provide; see the file header.
  ament_add_gtest(test_params_schema test/test_params_schema.cpp)
  target_link_libraries(test_params_schema rtc_controllers)

  # QuinticPolynomial boundary math + T<=0 short-circuit (trajectory_utils.hpp).
  ament_add_gtest(test_trajectory_utils test/test_trajectory_utils.cpp)
  target_link_libraries(test_trajectory_utils rtc_controllers)
endif()

ament_package()
