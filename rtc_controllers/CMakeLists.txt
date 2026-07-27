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
find_package(rtc_controller_interface REQUIRED)
find_package(rtc_base REQUIRED)
find_package(rtc_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rtc_math REQUIRED)
find_package(rtc_urdf_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)

# ── Library ───────────────────────────────────────────────────────────────────
add_library(rtc_controllers STATIC
  src/controllers/indirect/p_controller.cpp
  src/controllers/indirect/clik_controller.cpp
  src/controllers/indirect/task_admittance_controller.cpp
  src/controllers/direct/joint_pd_controller.cpp
  src/controllers/direct/operational_space_controller.cpp
  src/controllers/direct/task_impedance_controller.cpp
  src/controllers/direct/cascaded_compliance_controller.cpp
  src/controllers/grasp/grasp_controller.cpp
  src/controllers/grasp/pull_force_estimator.cpp
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
  rtc_controller_interface
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
  set(rtc_base_test_include "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_base/test/include")

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

  ament_add_gtest(test_core_controllers test/test_core_controllers.cpp)
  target_link_libraries(test_core_controllers rtc_controllers)
  target_include_directories(test_core_controllers PRIVATE ${rtc_urdf_bridge_test_include})

  # #236 S1 golden-vector harness: the joint-space PD core pinned bit-for-bit
  # against a literal copy of the pre-extraction inline form (durable oracle)
  # AND against the live JointPDController driven through a shim that owns the
  # trajectory (transient oracle — it dies with the adapter at S7, and until
  # then it is what catches a correlated transcription error). Needs the URDF
  # fixture resolver for planar_3r (non-zero gravity) and the shared bitwise
  # helpers. The RT allocation gate replaces global operator new/delete, which
  # makes GCC false-positive -Wmismatched-new-delete program-wide.
  ament_add_gtest(test_joint_pd_core test/test_joint_pd_core.cpp)
  target_link_libraries(test_joint_pd_core rtc_controllers)
  target_include_directories(test_joint_pd_core PRIVATE
    ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_joint_pd_core PRIVATE -Wno-mismatched-new-delete)

  # #236 S2a golden-vector suite for the task-space acceleration core: bitwise
  # against a literal copy of the pre-extraction inline form (durable oracle)
  # AND against the live OperationalSpaceController driven through a shim that
  # owns the model and the trajectory. The OSC exposes no a_task, so the shim
  # reproduces the whole Λ/τ/Nᵀ tail to observe the core through the emitted
  # torque — hence the URDF fixture resolver for serial_7dof (non-zero gravity,
  # redundant so Nᵀ is not inert; the fixture itself is shared with S3a via
  # test/include/rtc_controllers/testing/serial7dof_fixture.hpp). Same shared
  # global-operator-new gate as S1 with the same program-wide
  # -Wmismatched-new-delete false positive, PLUS rtc_base's Eigen allocation
  # tripwire (${rtc_base_test_include}) — operator-new counting cannot see Eigen's
  # allocator, which calls std::malloc directly.
  ament_add_gtest(test_task_accel_core test/test_task_accel_core.cpp)
  target_link_libraries(test_task_accel_core rtc_controllers)
  target_include_directories(test_task_accel_core PRIVATE
    ${rtc_base_test_include} ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_task_accel_core PRIVATE -Wno-mismatched-new-delete)

  # #236 S3a golden-vector suite for the task-space velocity core: bitwise
  # against literal copies of BOTH pre-extraction inline forms (six-axis and
  # translation-only are different expression shapes, so each needs its own
  # durable oracle) AND against the live ClikController across all four
  # (control_6dof × enable_null_space) modes plus the π-rotation split. CLIK's
  # output IS the core's output pushed through J⁺, so the shim compares the
  # velocity, feedforward and integrated-command lanes directly. Reuses
  # test_task_accel_core's independent-handle premise via the same serial_7dof
  # fixture (the reorder half of that premise is measured here). The RT gate is
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
  # is bit-inert instead of inheriting the probe's claim) AND against the live
  # TaskImpedanceController across selection × Λ_d × clamp, plus the activation
  # ramp that scales the core's f_ext input. The adapter exposes no f_cmd, so the
  # shim replicates the whole τ tail and compares the final joint torque. The RT
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
  # TaskAdmittanceController now calls S3a's task::ComputeTaskVelocity, so the
  # suite pins that the call is inert — bitwise against a literal copy of THIS
  # controller's inline shape (ν_c copied, K⊙e accumulated into it, i.e. the
  # operands the other way round from the core) and against the live adapter
  # across integrate_from_measured × nullspace_kp × ik_kp, plus the activation
  # ramp that feeds the §7.2 integrator. The shim instantiates the adapter's own
  # AdmittanceIntegrator / DifferentialIk instead of re-deriving them, and
  # compares target_velocities and commands directly. Same two-sensor RT gate as
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
  # oracles pin all three pre-extraction shapes; a shim composed from S4's
  # ImpedanceShim and S5's AdmittanceShim pins the live cascade across the four
  # posture-gate corners plus the activation ramp. Both gates are numerically
  # INERT when closed, so the suite compares diag.nullspace_active and
  # diag.bandwidth_ratio every tick and exercises both ways the §7.6 gate reopens
  # (set_gains, and a retry after a failed factorisation). Same two-sensor RT
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

  ament_add_gtest(test_task_impedance_controller test/test_task_impedance_controller.cpp)
  target_link_libraries(test_task_impedance_controller rtc_controllers)
  target_include_directories(test_task_impedance_controller PRIVATE
    ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  # The zero-allocation gate replaces global operator new/delete (new→malloc,
  # delete→free). GCC's -Wmismatched-new-delete then false-positives on every
  # delete site program-wide; suppress it for this target only.
  target_compile_options(test_task_impedance_controller PRIVATE -Wno-mismatched-new-delete)

  # Slice-2 external wrench surface (#236): SetExternalWrench input contract,
  # conditioning + Ad^{-T} frame transform, staleness fade / DEGRADED, §6.3
  # inertia shaping incl. the T4.1 continuity boundary, zero-alloc with the
  # wrench path live. Same global operator-new interposition as
  # test_task_impedance_controller, hence the same -Wmismatched-new-delete
  # suppression.
  ament_add_gtest(test_task_impedance_wrench test/test_task_impedance_wrench.cpp)
  target_link_libraries(test_task_impedance_wrench rtc_controllers)
  target_include_directories(test_task_impedance_wrench PRIVATE
    ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_task_impedance_wrench PRIVATE -Wno-mismatched-new-delete)

  # Slice-4 cascade surface (#236): outer admittance → inner impedance as a
  # COMPOSITION — §7.6 MUST-4 (the wrench is spent once), MUST-1 (bandwidth
  # report, never a fault), MUST-3 (return vs hand-guiding), plus the F5/F7/F8
  # classes PR #256 found in the copied skeleton and the RT allocation gate.
  # Same global operator-new interposition as the impedance suites, hence the
  # same -Wmismatched-new-delete suppression.
  ament_add_gtest(test_cascaded_compliance_controller
    test/test_cascaded_compliance_controller.cpp)
  target_link_libraries(test_cascaded_compliance_controller rtc_controllers)
  target_include_directories(test_cascaded_compliance_controller
    PRIVATE ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_cascaded_compliance_controller
    PRIVATE -Wno-mismatched-new-delete)

  # Off-RT config / registry-hook surface: LoadConfig YAML, OnDeviceConfigsSet
  # (tip_link / joint_limits / safe_position), bounds guards, multi-device
  # passthrough, 6-DOF + gravity/Coriolis compensation branches.
  # Slice-3 controller surface (#236): the §11.4.1 P2/P3 sign contrast against
  # the impedance law, wrench→motion end to end, §10.6 staleness, the
  # position-domain E-STOP hold, configure contracts, and the RT allocation
  # gate. Same global operator-new interposition as the impedance wrench suite,
  # hence the same -Wmismatched-new-delete suppression.
  ament_add_gtest(test_task_admittance_controller test/test_task_admittance_controller.cpp)
  target_link_libraries(test_task_admittance_controller rtc_controllers)
  target_include_directories(test_task_admittance_controller
    PRIVATE ${rtc_urdf_bridge_test_include} ${rtc_controllers_test_include})
  target_compile_options(test_task_admittance_controller PRIVATE -Wno-mismatched-new-delete)

  ament_add_gtest(test_controller_config test/test_controller_config.cpp)
  target_link_libraries(test_controller_config rtc_controllers)
  target_include_directories(test_controller_config PRIVATE ${rtc_urdf_bridge_test_include})

  # #172 deeper AC: 7-DOF generalization (serial_7dof.urdf, no OOB gain/target
  # read), non-Z gravity/Coriolis magnitude vs the RNEA identity
  # (planar_3r.urdf, reference from an independent RtModelHandle), and the
  # device-order vs URDF-order identity contract.
  ament_add_gtest(test_dof_dynamics test/test_dof_dynamics.cpp)
  target_link_libraries(test_dof_dynamics rtc_controllers)
  target_include_directories(test_dof_dynamics PRIVATE ${rtc_urdf_bridge_test_include})

  # QuinticPolynomial boundary math + T<=0 short-circuit (trajectory_utils.hpp).
  ament_add_gtest(test_trajectory_utils test/test_trajectory_utils.cpp)
  target_link_libraries(test_trajectory_utils rtc_controllers)

  # RT-4 single-writer SeqLock race stress: SetDeviceTarget (off-RT) vs
  # Compute (RT) — verifies the SPSC marshal pattern keeps target_seqlock_
  # tear-free under concurrent producer/consumer access.
  ament_add_gtest(test_target_seqlock_race test/test_target_seqlock_race.cpp)
  target_link_libraries(test_target_seqlock_race rtc_controllers)
  target_include_directories(test_target_seqlock_race PRIVATE ${rtc_urdf_bridge_test_include})

  # Activation generation gate (#196 §3): stale targets queued while Inactive
  # must not reach the target slot after re-activation.
  ament_add_gtest(test_activation_generation_gate
    test/test_activation_generation_gate.cpp)
  target_link_libraries(test_activation_generation_gate rtc_controllers)
  target_include_directories(test_activation_generation_gate
    PRIVATE ${rtc_urdf_bridge_test_include})
endif()

ament_package()
