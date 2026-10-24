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
# The catch-pose IK task step is a
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
  # G2 schema layer: gains PODs + YAML parsers, lifted out of the
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
  # YAML schema + validator for the catching core
  # (rtc::catching). Same G2 schema-layer shape as the siblings above — POD +
  # yaml-cpp parser, non-RT, called once at configure time — plus a fixed-
  # capacity, allocation-free ValidateCatchingParams().
  src/params/catching_params.cpp
  # The `planner.search.grid.ik.*` / `planner.search.grid.catchability.*` half
  # of the schema, producing the CatchPoseIkOptions.
  # Sibling of catching_params.cpp, not part of it — the validator subset
  # declares planner.search.grid.ik.* out of its scope. Strict on unknown keys under the
  # section it wholly owns; retired keys are reported, as elsewhere in params/.
  src/params/catch_pose_ik_params.cpp
  # 5-row catch-pose IK + the catchability gate
  # Pinocchio + Eigen only, no ROS and no controller types — the
  # offline catchability map links the same object the planner will, so
  # this deliberately stays a plain library unit rather than a controller.
  src/catching/catch_pose_ik.cpp
  # The catch frame's reach bound, read off the model: the necessary condition
  # both searches test before the catch-pose IK.
  src/catching/reach_bound.cpp
  # Candidate/seed CSV I/O and the batch loop the
  # offline map's judge is built on. In the library, not the executable, so
  # the CSV contract (bit-exact columns) and the batch's order-independence
  # are reachable from a gtest instead of only from a subprocess.
  src/catching/catch_pose_ik_batch.cpp
  # The rest of the planner's gate chain (reach time,
  # γ window, stopping point) over CSV. In the library for the same reason as
  # the file above — the CSV contract and the verdicts are gtest-reachable.
  src/catching/catch_gate_batch.cpp
  # The search itself (CatchSearch::Plan) over the
  # wakes of a throw, from CSV. In the library for the same reason as the two
  # files above — the CSV contract, the wake inputs and the binding are
  # gtest-reachable.
  src/catching/catch_search_batch.cpp
  # The planner thread's `planner.*` keys (sibling of the
  # two parsers above — this one describes how the thread runs, not what a
  # catch pose is) and one planner wake: inputs → PlanOnce → provenance
  # re-check → publish. ROS-free so the whole cycle is testable against plain
  # SeqLocks; the thread that calls it lives in integrated_bringup.
  src/params/planner_params.cpp
  # The docking keys — the hand's identified capture set
  # (`robot.hand.docking.*`) and the two docking functions' own maps.
  src/params/docking_params.cpp
  # What only shows with the docking maps side by side (one check for a
  # controller's configure and the offline whole-wake batch).
  src/catching/docking_config.cpp
  # `planner.groups` resolved against the arm
  # device — the joint groups the segment cores weight and block per group.
  src/catching/joint_groups.cpp
  src/catching/planner_cycle.cpp
  src/catching/grid_catch_search.cpp
  # The segment MPC numeric core — a jerk-input
  # condensed QP over the stop segment with first-order torque rows (pinocchio
  # RNEA derivatives) and slack, solved through rtc_tsid's QPSolverWrapper. A
  # plain library unit like catch_pose_ik.cpp; the planner thread calls it.
  src/catching/mpc_segment_core.cpp
  # The mpc_docking numeric core — the ball in
  # the capture frame and the rows built from it (relative state, corridor,
  # chance rows, impact, manipulability), each with its derivative.
  src/catching/mpc_docking_relative_state.cpp
  # ... and the SQP that assembles them: MpcDockingSegmentCore — approach,
  # dock and stop as one condensed jerk problem over ProxQP, with elastic hard
  # rows, an l1 merit and a deadline. Called by nothing but its tests yet.
  src/catching/mpc_docking_segment_core.cpp
  # The NLP catch search — the second
  # CatchSearch. One core per pre-catch node count; a wake screens the lattice
  # of absolute catch instants, solves the ranked candidates from the previous
  # wake's solutions and chooses by the solve's cost. Installed by nothing yet.
  src/catching/nlp_catch_search.cpp
  # The RT side of the segment payload —
  # closed-form joint reference between the published nodes, hand target by FK.
  src/catching/node_follower.cpp
  # The stop segment on the planner thread —
  # x0 prediction at the effective instant, replan instances with a fixed stop
  # end, the publish gate. ROS-free; PlannerCycle runs it and publishes.
  src/catching/mpc_segment_planner.cpp
  # The second segment planner — the docking
  # problem per segment (MpcDockingSegmentCore), installed in PlannerCycle
  # under `planner.segment.mode: mpc_docking`.
  src/catching/mpc_docking_segment_planner.cpp
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
# node, and appears in no launch file or bringup chain. It exists so the offline
# catchability map and the runtime planner reach their verdict through the
# SAME CatchPoseIk::Solve and the same YAML keys; re-deriving the
# judgement in the python map tool is what this replaces.
add_executable(catch_pose_ik_batch src/catching/catch_pose_ik_batch_main.cpp)
target_link_libraries(catch_pose_ik_batch rtc_controllers)

# ARCH-7-exempt: same category as above (an offline inspection tool) — the
# gate-catchable map's judge. It loads no model at all (limits and
# constants arrive by argv, postures by CSV) and calls the runtime gate
# functions of time_feasibility.hpp.
add_executable(catch_gate_batch src/catching/catch_gate_batch_main.cpp)
target_link_libraries(catch_gate_batch rtc_controllers)

# ARCH-7-exempt: same category as above (an offline inspection tool) — the
# catch search's verdict map. The model, the catching tree, the values a
# controller binds at configure time and the prediction snapshots arrive by
# argv; it calls CatchSearch::Plan as a planner wake does.
add_executable(catch_search_batch src/catching/catch_search_batch_main.cpp)
target_link_libraries(catch_search_batch rtc_controllers)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS rtc_controllers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS catch_pose_ik_batch catch_gate_batch catch_search_batch
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

  # Catch-pose IK + catchability gate. Needs
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

  # The catch frame's reach bound: a bound at any number of
  # iterations, tight on the revolute arms, infinite where it cannot be made.
  ament_add_gtest(test_catching_reach_bound test/test_catching_reach_bound.cpp)
  target_link_libraries(test_catching_reach_bound rtc_controllers)
  target_include_directories(test_catching_reach_bound PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_reach_bound PRIVATE -Wno-mismatched-new-delete)

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

  # G2 schema layer contract. Drives the ParseXxxParams free
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

  # ── catching core (rtc::catching) ───────────────────────────────────────────
  # ROS-free numerical core for ball catching.
  #
  # Ball flight fixture. Test-only: it generates the reference
  # trajectories the other catching suites compare against, and the controller
  # never re-propagates the ball, so it lives under the never-installed
  # test/include.
  ament_add_gtest(test_catching_ball_fixture test/test_catching_ball_fixture.cpp)
  target_link_libraries(test_catching_ball_fixture rtc_controllers)
  target_include_directories(test_catching_ball_fixture PRIVATE ${rtc_controllers_test_include})

  # Time axes: the cross-axis comparisons that must NOT compile are
  # static_asserts in this TU, so a regression there is a build break rather
  # than a failed case.
  ament_add_gtest(test_catching_time_types test/test_catching_time_types.cpp)
  target_link_libraries(test_catching_time_types rtc_controllers)

  # Trajectory snapshot + quintic Hermite sampler. The RT
  # read path is gated by both allocation sensors (operator new and Eigen's own
  # allocator), hence both test include dirs and the program-wide
  # -Wmismatched-new-delete false positive of the alloc gate.
  ament_add_gtest(test_catching_traj_sampler test/test_catching_traj_sampler.cpp)
  target_link_libraries(test_catching_traj_sampler rtc_controllers)
  target_include_directories(test_catching_traj_sampler PRIVATE
    ${rtc_base_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_traj_sampler PRIVATE -Wno-mismatched-new-delete)

  # The ball prediction at a planner node —
  # the shared sampler's mean plus the 6x6 covariance propagated from the
  # nearest sample. Header-only and RT-safe, hence both allocation sensors.
  ament_add_gtest(test_catching_ball_node_samples test/test_catching_ball_node_samples.cpp)
  target_link_libraries(test_catching_ball_node_samples rtc_controllers)
  target_include_directories(test_catching_ball_node_samples PRIVATE
    ${rtc_base_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_ball_node_samples PRIVATE -Wno-mismatched-new-delete)

  # L1 ingress judgement: order / track epoch / staleness / horizon / jump
  # Header-only, no ROS: the
  # PointCloud2 decode and the SeqLock live in the integration binding, and
  # what is testable against a struct is tested here.
  ament_add_gtest(test_catching_traj_ingress test/test_catching_traj_ingress.cpp)
  target_link_libraries(test_catching_traj_ingress rtc_controllers)

  # Soft-catch translational reference. The γ-derate
  # cases of the reference are not ported and the axis-alignment cases
  # belong to rtc_math se3. Same two-sensor allocation gate.
  ament_add_gtest(test_catching_soft_catch test/test_catching_soft_catch.cpp)
  target_link_libraries(test_catching_soft_catch rtc_controllers)
  target_include_directories(test_catching_soft_catch PRIVATE
    ${rtc_base_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_soft_catch PRIVATE -Wno-mismatched-new-delete)

  # Reachability gates: minimum joint time against a fixed table
  # from the reference script's (removed) independent closed form, γ window
  # with the TCP speed, directional speed, stopping distance, error budget.
  # Same two-sensor allocation gate (the planner may run SCHED_FIFO).
  ament_add_gtest(test_catching_time_feasibility test/test_catching_time_feasibility.cpp)
  target_link_libraries(test_catching_time_feasibility rtc_controllers)
  target_include_directories(test_catching_time_feasibility PRIVATE
    ${rtc_base_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_time_feasibility PRIVATE -Wno-mismatched-new-delete)

  # Supervisor "pure pieces": decel target,
  # transition table as data, contact debounce. Same
  # global operator-new counting gate as the other RT-1 suites above
  # (${rtc_controllers_test_include}) PLUS rtc_base's Eigen allocation
  # tripwire (${rtc_base_test_include}) — the decel target math is
  # Eigen::Vector3d, which operator-new counting cannot see.
  ament_add_gtest(test_catching_supervisor_core test/test_catching_supervisor_core.cpp)
  target_link_libraries(test_catching_supervisor_core rtc_controllers)
  target_include_directories(test_catching_supervisor_core PRIVATE
    ${rtc_base_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_supervisor_core PRIVATE -Wno-mismatched-new-delete)

  # Hand sequencer: close-tick rounding, phase rules,
  # ρ, hold rule, no allocation in Update. alloc_gate.hpp → its own binary.
  ament_add_gtest(test_catching_hand_sequencer test/test_catching_hand_sequencer.cpp)
  target_link_libraries(test_catching_hand_sequencer rtc_controllers)
  target_include_directories(test_catching_hand_sequencer PRIVATE
    ${rtc_base_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_hand_sequencer PRIVATE -Wno-mismatched-new-delete)

  # Plane crossing: the instant the ball crosses a plane of the catch
  # frame, by the secant method — closed-form roots, every refusal, no
  # allocation. alloc_gate.hpp → its own binary.
  ament_add_gtest(test_catching_plane_crossing test/test_catching_plane_crossing.cpp)
  target_link_libraries(test_catching_plane_crossing rtc_controllers)
  target_include_directories(test_catching_plane_crossing PRIVATE
    ${rtc_base_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_plane_crossing PRIVATE -Wno-mismatched-new-delete)

  # Hand-joint capture witness: each clause, the
  # fail-closed paths, no allocation, and synthetic-noise false alarms.
  ament_add_gtest(test_catching_hand_capture test/test_catching_hand_capture.cpp)
  target_link_libraries(test_catching_hand_capture rtc_controllers)
  target_include_directories(test_catching_hand_capture PRIVATE
    ${rtc_base_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_hand_capture PRIVATE -Wno-mismatched-new-delete)

  # Parameter validator: active-config TBD, cross-
  # constraints, ζ·ω·h discretization, hand caging gap, provisional
  # gating. Model-free — no URDF fixture needed.
  ament_add_gtest(test_catching_params test/test_catching_params.cpp)
  target_link_libraries(test_catching_params rtc_controllers)

  # Catch-pose IK schema: every planner.search.grid.ik.* / planner.search.grid.catchability.*
  # key reaches CatchPoseIkOptions, an absent section resolves to the struct's
  # own defaults (which is also how the doc-vs-code default agreement is
  # pinned), and a typo under planner.search.grid.ik is refused rather than defaulted.
  # Model-free — the parser touches no URDF and no solver.
  # Closed-form jerk segment (header-only).
  ament_add_gtest(test_catching_jerk_segment test/test_catching_jerk_segment.cpp)
  target_link_libraries(test_catching_jerk_segment rtc_controllers)

  # The MPC segment core. Its Solve() calls
  # into pinocchio (RNEA derivatives, frame Jacobian) and ProxQP — library code
  # whose Eigen/C allocations neither the operator-new gate nor rtc_base's Eigen
  # tripwire can see (testing-debug.md §RT-1). So the RT gate here is THREE
  # sensors: operator new (${rtc_controllers_test_include}/alloc_gate.hpp),
  # and from ${rtc_base_test_include} the Eigen tripwire and the C-level
  # malloc_gate.hpp, which defines malloc in this executable and so counts
  # allocations made inside the shared libraries too. Real arm URDFs come from
  # the sibling robot_descriptions tree by source path (test-only, like the
  # rtc_urdf_bridge fixtures) for the timing distribution at real dimensions.
  ament_add_gtest(test_catching_mpc_segment_core test/test_catching_mpc_segment_core.cpp TIMEOUT 300)
  target_link_libraries(test_catching_mpc_segment_core rtc_controllers)
  target_include_directories(test_catching_mpc_segment_core PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_mpc_segment_core PRIVATE -Wno-mismatched-new-delete)
  target_compile_definitions(test_catching_mpc_segment_core PRIVATE
    RTC_TEST_ROBOT_DESCRIPTIONS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots"
    RTC_TEST_BUILD_TYPE="${CMAKE_BUILD_TYPE}")

  # The pre-catch grid and the catch terms
  # of the same core. A separate binary — the malloc gate is one TU per binary,
  # and this suite's timing table needs its own timeout: a smoke by default,
  # the full 200-sample table with RTC_MPC_SEGMENT_CORE_TIMING_FULL=1,
  # which takes several minutes. Same three allocation sensors and the same arm
  # fixtures (mpc_segment_core_fixture.hpp) as the segment-core suite above.
  ament_add_gtest(test_catching_mpc_segment_core_approach
    test/test_catching_mpc_segment_core_approach.cpp TIMEOUT 1500)
  target_link_libraries(test_catching_mpc_segment_core_approach rtc_controllers)
  target_include_directories(test_catching_mpc_segment_core_approach PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_mpc_segment_core_approach PRIVATE -Wno-mismatched-new-delete)
  target_compile_definitions(test_catching_mpc_segment_core_approach PRIVATE
    RTC_TEST_ROBOT_DESCRIPTIONS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots"
    RTC_TEST_BUILD_TYPE="${CMAKE_BUILD_TYPE}")

  # The segment MPC core with joint groups —
  # per-group move blocking and the whole-body terms, on a synthetic tree.
  ament_add_gtest(test_catching_mpc_segment_core_groups
    test/test_catching_mpc_segment_core_groups.cpp TIMEOUT 600)
  target_link_libraries(test_catching_mpc_segment_core_groups rtc_controllers)
  target_include_directories(test_catching_mpc_segment_core_groups PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_mpc_segment_core_groups PRIVATE -Wno-mismatched-new-delete)
  target_compile_definitions(test_catching_mpc_segment_core_groups PRIVATE
    RTC_TEST_ROBOT_DESCRIPTIONS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots")

  # ... and the segment planners with joint groups, on a synthetic tree whose
  # device order is the reverse of the model's.
  ament_add_gtest(test_catching_segment_groups test/test_catching_segment_groups.cpp TIMEOUT 600)
  target_link_libraries(test_catching_segment_groups rtc_controllers)
  target_include_directories(test_catching_segment_groups PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_definitions(test_catching_segment_groups PRIVATE
    RTC_TEST_ROBOT_DESCRIPTIONS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots")

  # The mpc_docking relative-state seams — each
  # derivative against a finite difference, the crossing-plane covariance
  # against a Monte Carlo, the reference's sanity cases. The seams call into
  # pinocchio's shared object, so the allocation claim needs the C-level malloc
  # gate (one TU per binary) besides operator new.
  ament_add_gtest(test_catching_mpc_docking_relative_state
    test/test_catching_mpc_docking_relative_state.cpp TIMEOUT 300)
  target_link_libraries(test_catching_mpc_docking_relative_state rtc_controllers)
  target_include_directories(test_catching_mpc_docking_relative_state PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_mpc_docking_relative_state PRIVATE
    -Wno-mismatched-new-delete)
  target_compile_definitions(test_catching_mpc_docking_relative_state PRIVATE
    RTC_TEST_ROBOT_DESCRIPTIONS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots")

  # ... and the docking core's SQP on synthetic throws: constructed-feasible
  # cases that must converge with every hard row re-evaluated outside the core,
  # infeasible ones that must say so, the payload round trip through the RT
  # sampler, the per-stage allocation gate (C-level malloc gate: one TU per
  # binary) and the recorded timing.
  ament_add_gtest(test_catching_mpc_docking_segment_core
    test/test_catching_mpc_docking_segment_core.cpp TIMEOUT 900)
  target_link_libraries(test_catching_mpc_docking_segment_core rtc_controllers)
  target_include_directories(test_catching_mpc_docking_segment_core PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_mpc_docking_segment_core PRIVATE -Wno-mismatched-new-delete)
  target_compile_definitions(test_catching_mpc_docking_segment_core PRIVATE
    RTC_TEST_ROBOT_DESCRIPTIONS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots"
    RTC_TEST_BUILD_TYPE="${CMAKE_BUILD_TYPE}")

  # The docking core's whole-body terms — off,
  # the convergence contract, the values on the held coefficients, allocation.
  ament_add_gtest(test_catching_mpc_docking_segment_core_wholebody
    test/test_catching_mpc_docking_segment_core_wholebody.cpp TIMEOUT 900)
  target_link_libraries(test_catching_mpc_docking_segment_core_wholebody rtc_controllers)
  target_include_directories(test_catching_mpc_docking_segment_core_wholebody PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_mpc_docking_segment_core_wholebody PRIVATE
    -Wno-mismatched-new-delete)
  target_compile_definitions(test_catching_mpc_docking_segment_core_wholebody PRIVATE
    RTC_TEST_ROBOT_DESCRIPTIONS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots")

  # The NLP catch search on synthetic
  # predictions — the choice against an exhaustive solve by the core, order
  # independence of the solves, validity and reasons, the wakes after the RT
  # follows a plan, and the allocation gate over a whole Plan (C-level malloc
  # gate: one TU per binary) with the QP solvers bracketed out.
  ament_add_gtest(test_catching_nlp_catch_search
    test/test_catching_nlp_catch_search.cpp TIMEOUT 900)
  target_link_libraries(test_catching_nlp_catch_search rtc_controllers)
  target_include_directories(test_catching_nlp_catch_search PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_nlp_catch_search PRIVATE -Wno-mismatched-new-delete)
  target_compile_definitions(test_catching_nlp_catch_search PRIVATE
    RTC_TEST_ROBOT_DESCRIPTIONS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots"
    RTC_TEST_BUILD_TYPE="${CMAKE_BUILD_TYPE}")

  # The docking segment planner — configure and
  # its refusals, the first segment (the search's solution adopted, or solved
  # from rest), replans, "nothing is published after the catch", the interface
  # queries, and the allocation gate over PlanFirst / Replan (C-level malloc
  # gate: one TU per binary) with the QP solvers bracketed out.
  ament_add_gtest(test_catching_mpc_docking_segment_planner
    test/test_catching_mpc_docking_segment_planner.cpp TIMEOUT 900)
  target_link_libraries(test_catching_mpc_docking_segment_planner rtc_controllers)
  target_include_directories(test_catching_mpc_docking_segment_planner PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_mpc_docking_segment_planner PRIVATE
    -Wno-mismatched-new-delete)
  target_compile_definitions(test_catching_mpc_docking_segment_planner PRIVATE
    RTC_TEST_ROBOT_DESCRIPTIONS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots"
    RTC_TEST_BUILD_TYPE="${CMAKE_BUILD_TYPE}")

  # The NLP catch search's lattice, its
  # necessary conditions (reach, closing-speed window) and its outer cost —
  # pure functions of scalars, each checked against the reference's formula
  # from both sides of its boundary. Header-only: no model, no solver.
  ament_add_gtest(test_catching_nlp_catch_screening test/test_catching_nlp_catch_screening.cpp)
  target_link_libraries(test_catching_nlp_catch_screening rtc_controllers)

  # The ring of published segments both segment
  # planners keep (SegmentRing). Header-only pure logic: no model, no solver.
  ament_add_gtest(test_catching_segment_ring test/test_catching_segment_ring.cpp)
  target_link_libraries(test_catching_segment_ring rtc_controllers)

  # The segment node payload and its RT
  # sampler. Sample() calls into pinocchio's shared object (FK), so its
  # allocation claim needs the C-level malloc gate as well as operator new;
  # the informational CLIK residual drives rtc_tsid's ClikReferenceGenerator.
  ament_add_gtest(test_catching_node_follower test/test_catching_node_follower.cpp)
  target_link_libraries(test_catching_node_follower rtc_controllers)
  target_include_directories(test_catching_node_follower PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_node_follower PRIVATE -Wno-mismatched-new-delete)
  target_compile_definitions(test_catching_node_follower PRIVATE
    RTC_TEST_ROBOT_DESCRIPTIONS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots")

  # The segment payload's joint capacity — a model wider
  # than one arm (a tree built in code, 17 joints) through the payload, the RT
  # sampler and both segment planners' Configure, and the nlp search on the
  # 10-joint chain. No URDF: the joint count is the test's to choose.
  ament_add_gtest(test_catching_segment_capacity test/test_catching_segment_capacity.cpp
    TIMEOUT 300)
  target_link_libraries(test_catching_segment_capacity rtc_controllers)
  target_include_directories(test_catching_segment_capacity PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_definitions(test_catching_segment_capacity PRIVATE
    RTC_TEST_ROBOT_DESCRIPTIONS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots")

  # The planner.segment.mpc keys, and (the
  # second target) the RT's segment admission / switch rules. Pure functions —
  # no model, no solver. The MPC segment planner itself is
  # test_catching_approach_planner / _cycle below.
  ament_add_gtest(test_catching_mpc_segment_params test/test_catching_mpc_segment_params.cpp)
  target_link_libraries(test_catching_mpc_segment_params rtc_controllers)
  target_include_directories(test_catching_mpc_segment_params PRIVATE ${rtc_base_test_include})
  # The docking keys (`robot.hand.docking.*`,
  # `planner.search.nlp.*`, `planner.segment.mpc_docking.*`). Pure parsing — no
  # model, no solver.
  # `planner.groups` — parsed, resolved
  # against the device's joints, mapped to model order.
  ament_add_gtest(test_catching_joint_groups test/test_catching_joint_groups.cpp)
  target_link_libraries(test_catching_joint_groups rtc_controllers)
  target_include_directories(test_catching_joint_groups PRIVATE ${rtc_base_test_include})
  ament_add_gtest(test_catching_docking_params test/test_catching_docking_params.cpp)
  target_link_libraries(test_catching_docking_params rtc_controllers)
  target_include_directories(test_catching_docking_params PRIVATE ${rtc_base_test_include})
  ament_add_gtest(test_catching_segment_admission test/test_catching_segment_admission.cpp)
  target_link_libraries(test_catching_segment_admission rtc_controllers)
  target_include_directories(test_catching_segment_admission PRIVATE ${rtc_base_test_include})

  # The MPC segment planner's APPROACH–stop
  # solves on a fake clock (PlanFirst, Replan, the ball target, the publish
  # gates, the allocation boundary). Not test_catching_mpc_segment_core_approach
  # (the core alone). C-level malloc gate: one TU per binary.
  ament_add_gtest(test_catching_approach_planner test/test_catching_approach_planner.cpp
    TIMEOUT 300)
  target_link_libraries(test_catching_approach_planner rtc_controllers)
  target_include_directories(test_catching_approach_planner PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_approach_planner PRIVATE -Wno-mismatched-new-delete)
  target_compile_definitions(test_catching_approach_planner PRIVATE
    RTC_TEST_ROBOT_DESCRIPTIONS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots"
    RTC_TEST_BUILD_TYPE="${CMAKE_BUILD_TYPE}")

  # ... and the cycle that pairs the first segment with the search's plan and
  # replans it while the RT follows: the real search on the real
  # steady clock (RUN_SERIAL below).
  ament_add_gtest(test_catching_approach_cycle test/test_catching_approach_cycle.cpp TIMEOUT 300)
  target_link_libraries(test_catching_approach_cycle rtc_controllers)
  target_include_directories(test_catching_approach_cycle PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_approach_cycle PRIVATE -Wno-mismatched-new-delete)
  target_compile_definitions(test_catching_approach_cycle PRIVATE
    RTC_TEST_ROBOT_DESCRIPTIONS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots")

  # The whole-body terms through whole catches
  # of the planner cycle (search on a chain, segments on the tree).
  ament_add_gtest(test_catching_wholebody_cycle test/test_catching_wholebody_cycle.cpp TIMEOUT 600)
  target_link_libraries(test_catching_wholebody_cycle rtc_controllers)
  target_include_directories(test_catching_wholebody_cycle PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_definitions(test_catching_wholebody_cycle PRIVATE
    RTC_TEST_ROBOT_DESCRIPTIONS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots")

  ament_add_gtest(test_catch_pose_ik_params test/test_catch_pose_ik_params.cpp)
  target_link_libraries(test_catch_pose_ik_params rtc_controllers)

  # Batch layer for the catchability map (Solve is tested in test_catch_pose_ik).
  # Two claims the solver's own suite cannot make: the CSV carries the solver's
  # doubles bit-exactly (the runtime-planner equivalence oracle needs that), and a
  # candidate's verdict is independent of batch order (python shards the grid).
  # Drives the real 6R arm, hence the URDF fixture include dirs and the same
  # program-wide -Wmismatched-new-delete false positive as its sibling.
  ament_add_gtest(test_catch_pose_ik_batch test/test_catch_pose_ik_batch.cpp)
  target_link_libraries(test_catch_pose_ik_batch rtc_controllers)
  target_include_directories(test_catch_pose_ik_batch PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catch_pose_ik_batch PRIVATE -Wno-mismatched-new-delete)

  # Gate batch — CSV contract, verdict == direct call
  # of the runtime gate functions, order independence, fail-closed settings.
  ament_add_gtest(test_catch_gate_batch test/test_catch_gate_batch.cpp)
  target_link_libraries(test_catch_gate_batch rtc_controllers)

  # Search batch — a row is the direct Plan call's
  # result, the verdict of a throw does not depend on the order of the throws,
  # the CSV carries the doubles bit-exactly, the binding file is fail-closed.
  # Drives the real 6R arm (the grid search's rig), hence the fixture dirs.
  ament_add_gtest(test_catch_search_batch test/test_catch_search_batch.cpp)
  target_link_libraries(test_catch_search_batch rtc_controllers)
  target_include_directories(test_catch_search_batch PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catch_search_batch PRIVATE -Wno-mismatched-new-delete)
  # The whole-wake batch solves the mpc segment planner's dynamics: an arm
  # with inertias, as the approach-cycle suite drives.
  target_compile_definitions(test_catch_search_batch PRIVATE
    RTC_TEST_ROBOT_DESCRIPTIONS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots")

  # Planner data contract + one planner wake. The RT
  # admission rule, `planner.*` parsing, the cycle's
  # provenance handling (superseded trajectory, reset, activation) and
  # its RT contract (one cycle allocates nothing) — both allocation
  # sensors, hence both test include dirs.
  ament_add_gtest(test_catching_planner_cycle test/test_catching_planner_cycle.cpp)
  target_link_libraries(test_catching_planner_cycle rtc_controllers)
  target_include_directories(test_catching_planner_cycle PRIVATE
    ${rtc_base_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_planner_cycle PRIVATE -Wno-mismatched-new-delete)
  # The cycle's two source files are read as text: they must not name a
  # concrete search or segment planner.
  target_compile_definitions(test_catching_planner_cycle PRIVATE
    RTC_CONTROLLERS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

  # The planner's search on the 6R wrist fixture — q̇ᵘ
  # against an independent DLS, judgement vs rank gates,
  # settle, budget, switching/freeze, no allocation in a full search,
  # and the timing record. Needs the URDF fixtures, hence the extra dir.
  ament_add_gtest(test_catching_grid_catch_search test/test_catching_grid_catch_search.cpp)
  target_link_libraries(test_catching_grid_catch_search rtc_controllers)
  target_include_directories(test_catching_grid_catch_search PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_grid_catch_search PRIVATE -Wno-mismatched-new-delete)

  # γ rollout — the reference table through the runtime
  # sampler, the choice rule, coarse-to-fine, and no allocation.
  ament_add_gtest(test_catching_gamma_rollout test/test_catching_gamma_rollout.cpp)
  target_link_libraries(test_catching_gamma_rollout rtc_controllers)
  target_include_directories(test_catching_gamma_rollout PRIVATE
    ${rtc_base_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_catching_gamma_rollout PRIVATE -Wno-mismatched-new-delete)

  # ── Parallel ctest ─────────────────────────────────────────────────────────
  # colcon.pkg runs this package's tests four at a time (48 s -> 23 s; the two
  # segment-MPC suites are 34 s of it and now overlap). No test here opens a DDS
  # participant or writes outside its own result file, so none needs a lock.
  # The exception is a test that ASSERTS a measured wall-clock budget: it must
  # not share the cores with its siblings, and RUN_SERIAL gives it the box it
  # had while ctest ran one test at a time. Durations that are only RECORDED
  # (RecordProperty "worst_*_ns") are not protected -- quote those from a run
  # of that one test (`--ctest-args -R <name>`). testing-debug.md has the rule.
  set_tests_properties(test_catching_approach_cycle PROPERTIES RUN_SERIAL TRUE)
endif()

ament_package()
