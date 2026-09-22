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
# dynamic_catching S1.9 (D-26): the catch-pose IK task step is a
# box-constrained QP solved through rtc_tsid's QPSolverWrapper. No cycle —
# rtc_tsid does not depend on rtc_controllers (ARCH-2).
find_package(rtc_tsid REQUIRED)
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
  # Learned-policy I/O schema. Same G2 layer as its siblings above: POD +
  # yaml-cpp parser, no framework types, no inference engine — the law it owns
  # is "YAML order becomes tensor order", which a binding calls once at
  # configure time.
  src/params/policy_io_params.cpp
  src/params/reach_gate_params.cpp
  # dynamic_catching S1.7: YAML schema + G0-C validator for the catching core
  # (rtc::catching). Same G2 schema-layer shape as the siblings above — POD +
  # yaml-cpp parser, non-RT, called once at configure time — plus a fixed-
  # capacity, allocation-free ValidateCatchingParams() (L0 §5.3, §9 G0-C).
  src/params/catching_params.cpp
  # dynamic_catching S3.5a: the `planner.ik.*` / `planner.catchability.*` half
  # of the schema, producing the CatchPoseIkOptions S1.9 left without a parser.
  # Sibling of catching_params.cpp, not part of it — the G0-C validator subset
  # declares planner.ik.* out of its scope. Strict on unknown keys under the
  # section it wholly owns; retired keys are reported, as elsewhere in params/.
  src/params/catch_pose_ik_params.cpp
  # dynamic_catching S1.9: 5-row catch-pose IK + the catchability gate
  # (L3 §4.2). Pinocchio + Eigen only, no ROS and no controller types — the
  # offline catchability map (S3.5a) links the same object the planner will, so
  # this deliberately stays a plain library unit rather than a controller.
  src/catching/catch_pose_ik.cpp
  # dynamic_catching S3.5a: candidate/seed CSV I/O and the batch loop the
  # offline map's judge is built on. In the library, not the executable, so
  # the CSV contract (bit-exact columns) and the batch's order-independence
  # are reachable from a gtest instead of only from a subprocess.
  src/catching/catch_pose_ik_batch.cpp
  # dynamic_catching S3.5b: the rest of the planner's gate chain (reach time,
  # γ window, stopping point) over CSV. In the library for the same reason as
  # the file above — the CSV contract and the verdicts are gtest-reachable.
  src/catching/catch_gate_batch.cpp
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
  rtc_tsid
)
target_link_libraries(rtc_controllers pinocchio::pinocchio yaml-cpp Eigen3::Eigen)

# Position-independent code for static library (needed when linked into shared libs)
set_target_properties(rtc_controllers PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ── Offline tools ─────────────────────────────────────────────────────────────
# ARCH-7-exempt: an offline inspection tool, the category design-principles.md
# §"ARCH-7 의 범위" names (cf. rtc_inference_check). It knows no robot — model,
# frame, options and candidates all arrive by argv — owns no RT loop and no ROS
# node, and appears in no launch file or bringup chain. It exists so the S3.5a
# catchability map and the S6.2 runtime planner reach their verdict through the
# SAME CatchPoseIk::Solve and the same YAML keys (S1.9); re-deriving the
# judgement in the python map tool is what this replaces.
add_executable(catch_pose_ik_batch src/catching/catch_pose_ik_batch_main.cpp)
target_link_libraries(catch_pose_ik_batch rtc_controllers)

# ARCH-7-exempt: same category as above (an offline inspection tool) — the
# S3.5b gate-catchable map's judge. It loads no model at all (limits and
# constants arrive by argv, postures by CSV) and calls the runtime gate
# functions of time_feasibility.hpp.
add_executable(catch_gate_batch src/catching/catch_gate_batch_main.cpp)
target_link_libraries(catch_gate_batch rtc_controllers)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS rtc_controllers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS catch_pose_ik_batch catch_gate_batch
  DESTINATION lib/${PROJECT_NAME}
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
  rtc_tsid
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

  # §7.3 joint command tail (#478): the shared integrate → clamp → rebound →
  # round-trip step both position-output bindings run. Header-only, model-free.
  ament_add_gtest(test_joint_command_tail test/test_joint_command_tail.cpp)
  target_link_libraries(test_joint_command_tail rtc_controllers)

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

  # The dynamic half of the same observer. test_momentum_observer.cpp runs
  # entirely at q̇ == 0, where `p = M(q)q̇` and `Cᵀq̇` are identically zero, so M
  # and C are never exercised there (nor by a quasi-static sim run). This suite
  # drives the observer along a moving trajectory against a model-self-consistent
  # oracle — tau_m := RNEA(q,q̇,q̈) — so the verdict does not depend on any
  # simulator agreeing with the URDF. Needs ${rtc_urdf_bridge_test_include} for
  # the serial_7dof fixture resolver (non-zero gravity, populated Coriolis).
  ament_add_gtest(test_momentum_observer_model_oracle
    test/test_momentum_observer_model_oracle.cpp)
  target_link_libraries(test_momentum_observer_model_oracle rtc_controllers)
  target_include_directories(test_momentum_observer_model_oracle PRIVATE
    ${rtc_urdf_bridge_test_include})

  # dynamic_catching S1.9: catch-pose IK + catchability gate. Needs
  # ${rtc_urdf_bridge_test_include} for the URDF resolver and the two arm
  # fixtures (serial_6r_wrist for the roll sweep, serial_7dof for the redundant
  # case, serial_6dof as the rank-deficient negative). RT gate is two
  # complementary sensors — the shared global-operator-new gate
  # (${rtc_controllers_test_include}, with the same program-wide
  # -Wmismatched-new-delete false positive it forces everywhere else) PLUS
  # rtc_base's Eigen allocation tripwire (${rtc_base_test_include}), because
  # operator-new counting cannot see Eigen's allocator.
  ament_add_gtest(test_catch_pose_ik test/test_catch_pose_ik.cpp)
  target_link_libraries(test_catch_pose_ik rtc_controllers)
  target_include_directories(test_catch_pose_ik PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catch_pose_ik PRIVATE -Wno-mismatched-new-delete)

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

  # Learned-policy tensor marshalling + its YAML schema. The schema half is
  # where a retrained model's feature-order drift has to be caught, so the
  # cases are written as "the config is wrong" rather than "the parser works".
  ament_add_gtest(test_policy_io test/test_policy_io.cpp)
  target_link_libraries(test_policy_io rtc_controllers)

  # Reach gate (proximity ∨ tactile hold) core + its YAML. The trained formula
  # has to be reproduced exactly outside the network, so the step-count
  # boundaries and the ramp are pinned as numbers, not as "roughly works".
  ament_add_gtest(test_reach_gate test/test_reach_gate.cpp)
  target_link_libraries(test_reach_gate rtc_controllers)

  # ── catching core (dynamic_catching S1, rtc::catching) ─────────────────────
  # ROS-free numerical core for ball catching. The suites port the reference
  # tests under docs/dynamic_catching/ with their thresholds unchanged; the gate
  # IDs (G0-*, G2-*, G3-*, G4-*, G7-*) are those of the L0–L7 design docs.
  #
  # Ball flight fixture (G0-A). Test-only: it generates the reference
  # trajectories the other catching suites compare against, and the controller
  # never re-propagates the ball, so it lives under the never-installed
  # test/include.
  ament_add_gtest(test_catching_ball_fixture test/test_catching_ball_fixture.cpp)
  target_link_libraries(test_catching_ball_fixture rtc_controllers)
  target_include_directories(test_catching_ball_fixture PRIVATE ${rtc_controllers_test_include})

  # Time axes (G0-E): the cross-axis comparisons that must NOT compile are
  # static_asserts in this TU, so a regression there is a build break rather
  # than a failed case.
  ament_add_gtest(test_catching_time_types test/test_catching_time_types.cpp)
  target_link_libraries(test_catching_time_types rtc_controllers)

  # Trajectory snapshot + quintic Hermite sampler (G2-A..E, G2-G, G2-H). The RT
  # read path is gated by both allocation sensors (operator new and Eigen's own
  # allocator), hence both test include dirs and the program-wide
  # -Wmismatched-new-delete false positive of the alloc gate.
  ament_add_gtest(test_catching_traj_sampler test/test_catching_traj_sampler.cpp)
  target_link_libraries(test_catching_traj_sampler rtc_controllers)
  target_include_directories(test_catching_traj_sampler PRIVATE
    ${rtc_base_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_traj_sampler PRIVATE -Wno-mismatched-new-delete)

  # L1 ingress judgement: order / track epoch / staleness / horizon / jump
  # (S5.2, G1-E and the pure half of G1-H·G1-J). Header-only, no ROS: the
  # PointCloud2 decode and the SeqLock live in the integration binding, and
  # what is testable against a struct is tested here.
  ament_add_gtest(test_catching_traj_ingress test/test_catching_traj_ingress.cpp)
  target_link_libraries(test_catching_traj_ingress rtc_controllers)

  # Soft-catch translational reference (G4-A..C, G4-F, G4-G, G4-I). The γ-derate
  # cases of the reference are not ported (D-8) and the axis-alignment cases
  # (G4-D) belong to rtc_math se3 (S2.1). Same two-sensor allocation gate.
  ament_add_gtest(test_catching_soft_catch test/test_catching_soft_catch.cpp)
  target_link_libraries(test_catching_soft_catch rtc_controllers)
  target_include_directories(test_catching_soft_catch PRIVATE
    ${rtc_base_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_soft_catch PRIVATE -Wno-mismatched-new-delete)

  # Reachability gates (G3-A, G3-B): minimum joint time against a fixed table
  # from verify_l3.py's independent closed form, γ window with the D-9 TCP
  # speed, directional speed, stopping distance, error budget. Same two-sensor
  # allocation gate (the planner may run SCHED_FIFO).
  ament_add_gtest(test_catching_time_feasibility test/test_catching_time_feasibility.cpp)
  target_link_libraries(test_catching_time_feasibility rtc_controllers)
  target_include_directories(test_catching_time_feasibility PRIVATE
    ${rtc_base_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_time_feasibility PRIVATE -Wno-mismatched-new-delete)

  # L7 supervisor "pure pieces" (S1.8 / L7.1-L7.3): decel target (§4.3),
  # transition table as data (§4.1/§4.2), contact debounce (§4.4). Same
  # global operator-new counting gate as the other RT-1 suites above
  # (${rtc_controllers_test_include}) PLUS rtc_base's Eigen allocation
  # tripwire (${rtc_base_test_include}) — the decel target math is
  # Eigen::Vector3d, which operator-new counting cannot see.
  ament_add_gtest(test_catching_supervisor_core test/test_catching_supervisor_core.cpp)
  target_link_libraries(test_catching_supervisor_core rtc_controllers)
  target_include_directories(test_catching_supervisor_core PRIVATE
    ${rtc_base_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_supervisor_core PRIVATE -Wno-mismatched-new-delete)

  # Parameter validator (S1.7, G0-C): active-config TBD, D-9/L7 §4.3 cross-
  # constraints, ζ·ω·h discretization, hand caging gap (L6 §4.2), provisional
  # gating. Model-free — no URDF fixture needed.
  ament_add_gtest(test_catching_params test/test_catching_params.cpp)
  target_link_libraries(test_catching_params rtc_controllers)

  # Catch-pose IK schema (S3.5a): every planner.ik.* / planner.catchability.*
  # key reaches CatchPoseIkOptions, an absent section resolves to the struct's
  # own defaults (which is also how the doc-vs-code default agreement is
  # pinned), and a typo under planner.ik is refused rather than defaulted.
  # Model-free — the parser touches no URDF and no solver.
  ament_add_gtest(test_catch_pose_ik_params test/test_catch_pose_ik_params.cpp)
  target_link_libraries(test_catch_pose_ik_params rtc_controllers)

  # Batch layer for the S3.5a map (S1.9's Solve is tested in test_catch_pose_ik).
  # Two claims the solver's own suite cannot make: the CSV carries the solver's
  # doubles bit-exactly (the S6.2 equivalence oracle needs that), and a
  # candidate's verdict is independent of batch order (python shards the grid).
  # Drives the real 6R arm, hence the URDF fixture include dirs and the same
  # program-wide -Wmismatched-new-delete false positive as its sibling.
  ament_add_gtest(test_catch_pose_ik_batch test/test_catch_pose_ik_batch.cpp)
  target_link_libraries(test_catch_pose_ik_batch rtc_controllers)
  target_include_directories(test_catch_pose_ik_batch PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catch_pose_ik_batch PRIVATE -Wno-mismatched-new-delete)

  # dynamic_catching S3.5b: gate batch — CSV contract, verdict == direct call
  # of the runtime gate functions, order independence, fail-closed settings.
  ament_add_gtest(test_catch_gate_batch test/test_catch_gate_batch.cpp)
  target_link_libraries(test_catch_gate_batch rtc_controllers)
endif()

ament_package()
