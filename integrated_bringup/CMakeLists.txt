cmake_minimum_required(VERSION 3.16)
project(integrated_bringup VERSION 5.17.0 LANGUAGES CXX)

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
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rtc_controller_interface REQUIRED)
find_package(rtc_controllers REQUIRED)
find_package(rtc_base REQUIRED)
find_package(rtc_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Pre-find Python3 with NumPy so eigenpy's cmake detection succeeds.
set(Python3_FIND_VIRTUALENV STANDARD)
find_package(Python3 COMPONENTS Interpreter Development NumPy)

find_package(pinocchio REQUIRED)
find_package(rtc_math REQUIRED)
find_package(rtc_urdf_bridge REQUIRED)
find_package(rtc_controller_manager REQUIRED)
find_package(rtc_tsid REQUIRED)
find_package(rtc_mpc REQUIRED)

# ── Static library: demo controllers ──────────────────────────────────────────
add_library(${PROJECT_NAME}_demo_controllers STATIC
  src/controllers/controller_registration.cpp
  src/controllers/joint/controller.cpp
  src/controllers/joint/compute.cpp
  src/controllers/joint/lifecycle.cpp
  src/controllers/joint/parameters.cpp
  src/controllers/task/controller.cpp
  src/controllers/task/compute.cpp
  src/controllers/task/lifecycle.cpp
  src/controllers/task/parameters.cpp
  src/controllers/wbc/controller.cpp
  src/controllers/wbc/compute.cpp
  src/controllers/wbc/lifecycle.cpp
  src/controllers/wbc/parameters.cpp
  src/controllers/wbc/phase.cpp
  src/controllers/wbc/grasp_phase_manager.cpp
  src/controllers/wbc/force_reference_updater.cpp
  src/support/combined_model_cache.cpp
  src/support/demo_shared_config.cpp
  src/support/pull_estimator_wiring.cpp
  src/support/closed_chain_hand_fk.cpp
  src/support/wbc_reduced_dynamics_provider.cpp
  src/support/owned_topics.cpp
  src/backends/mujoco_native_backend.cpp
  src/backends/ur_driver_native_backend.cpp
  src/backends/udp_hand_native_backend.cpp
)

target_include_directories(${PROJECT_NAME}_demo_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_demo_controllers
  ament_index_cpp
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_msgs
  rtc_controller_interface
  rtc_controllers
  rtc_controller_manager
  rtc_base
  rtc_msgs
  pinocchio
  rtc_math
  rtc_urdf_bridge
  rtc_tsid
  rtc_mpc
)

target_link_libraries(${PROJECT_NAME}_demo_controllers
  pinocchio::pinocchio
  yaml-cpp
)

# ── RT tracing (opt-in) ──────────────────────────────────────────────────────
# Activates the RTC_TRACE_SCOPE spans in the WBC controller (Compute →
# ComputeControl → sub-steps). Links rtc_base's exported `rtc_tracing` target
# (PUBLIC RTC_TRACING_ENABLED + lttng includes) only when built with
# -DRTC_ENABLE_TRACING=ON; default OFF is a pure no-op. See rtc_base/CMakeLists.txt.
option(RTC_ENABLE_TRACING "Compile rtc:* LTTng UST tracepoints (RT tracing)" OFF)
if(RTC_ENABLE_TRACING)
  target_link_libraries(${PROJECT_NAME}_demo_controllers rtc_base::rtc_tracing)
  # PRIVATE (not inherited from rtc_tracing) — see interface-leak note in
  # rtc_base/CMakeLists.txt. Keeps the opt-in local to this target.
  target_compile_definitions(${PROJECT_NAME}_demo_controllers PRIVATE RTC_TRACING_ENABLED)
endif()

# ── UR5e-specific executable (links built-in + demo controllers) ──────────────
# --whole-archive prevents the linker from stripping static-init registration
# TUs, eliminating the need for ForceDemoControllerRegistration().
add_executable(integrated_rt_controller
  src/integrated_rt_controller_main.cpp
)
target_link_libraries(integrated_rt_controller
  -Wl,--whole-archive
  ${PROJECT_NAME}_demo_controllers
  -Wl,--no-whole-archive
)
ament_target_dependencies(integrated_rt_controller
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
install(TARGETS integrated_rt_controller
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

# ── Install Python package (demo_gui modules) ───────────────────────────────
# scripts/demo_controller_gui.py is a thin shim that imports from this package.
ament_python_install_package(${PROJECT_NAME})

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

  # Path to the Panda URDF used by the grasp-pipeline tests as a generic 7-DOF
  # fixture (pinocchio::buildModel, kinematics only). Default resolves to the
  # repo-vendored copy under robot_descriptions (checked in, so present at build
  # time both locally and in CI — unlike the old /usr/local path that the deps
  # artifact does not ship). Override-able via -D or env.
  get_filename_component(_rtc_panda_default
      "${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots/panda/urdf/panda.urdf"
      ABSOLUTE)
  set(RTC_PANDA_URDF_PATH "${_rtc_panda_default}"
      CACHE FILEPATH "Path to panda.urdf used by integrated_bringup tests")
  if(DEFINED ENV{RTC_PANDA_URDF_PATH})
    set(RTC_PANDA_URDF_PATH "$ENV{RTC_PANDA_URDF_PATH}" CACHE FILEPATH
        "Path to panda.urdf used by integrated_bringup tests" FORCE)
  endif()

  # ── DeviceStateLog / DeviceSensorLog POD mirrors (header-only) ────────────
  ament_add_gtest(test_device_log_pods test/test_device_log_pods.cpp)
  target_include_directories(test_device_log_pods PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  # pull_estimator_log_pod.hpp mirrors rtc::grasp::PullEstimate (#167).
  ament_target_dependencies(test_device_log_pods
    rtc_controllers
    Eigen3
  )

  # ── virtual_tcp header-only tests ──────────────────────────────────────────
  ament_add_gtest(test_virtual_tcp test/test_virtual_tcp.cpp)
  target_include_directories(test_virtual_tcp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
  )
  target_link_libraries(test_virtual_tcp pinocchio::pinocchio)
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

  # ── demo_joint_controller grasp-detection tests ────────────────────────────
  # Empty-URDF unit test: grasp detection / fingertip derivation / ToF gate /
  # none no-op. No rtc_tsid (joint controller is joint-space only).
  # Activation generation gate (#196 §3): stale targets queued while Inactive
  # must not reach the target slot after re-activation.
  ament_add_gtest(test_activation_generation_gate
    test/test_activation_generation_gate.cpp)
  target_link_libraries(test_activation_generation_gate
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_activation_generation_gate
    rclcpp
    rclcpp_lifecycle
    rtc_controllers
    rtc_base
    pinocchio
    Eigen3
  )

  ament_add_gtest(test_demo_joint_controller test/test_demo_joint_controller.cpp)
  target_link_libraries(test_demo_joint_controller
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_demo_joint_controller
    rclcpp
    rtc_controllers
    rtc_base
    pinocchio
    Eigen3
  )

  # ── demo_task_controller unit tests (#149) ─────────────────────────────────
  # URDF-backed (iiwa7_leap via ament_index, no ROS node/DDS): CLIK 3/6-DOF +
  # π-split trajectory, hand trajectory, force-only grasp detection,
  # contact_stop freeze/latch/release, ToF snapshot, virtual TCP, E-STOP —
  # DemoTaskController gates ComputeControl behind the combined-model cache,
  # so unlike the joint/wbc sisters these paths need a real model.
  ament_add_gtest(test_demo_task_controller test/test_demo_task_controller.cpp)
  target_link_libraries(test_demo_task_controller
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_demo_task_controller
    ament_index_cpp
    rclcpp
    rtc_controllers
    rtc_base
    rtc_urdf_bridge
    pinocchio
    Eigen3
  )

  # ── demo_joint_controller URDF-backed tests (#149) ─────────────────────────
  # Model-dependent joint paths the empty-URDF sister cannot reach: arm quintic
  # trajectory, fingertip FK/TF, virtual TCP, ToF populate, E-STOP FK.
  ament_add_gtest(test_demo_joint_controller_urdf
    test/test_demo_joint_controller_urdf.cpp)
  target_link_libraries(test_demo_joint_controller_urdf
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_demo_joint_controller_urdf
    ament_index_cpp
    rclcpp
    rtc_controllers
    rtc_base
    rtc_urdf_bridge
    pinocchio
    Eigen3
  )

  # ── demo_wbc_controller base_frame consistency (F-2) ───────────────────────
  ament_add_gtest(test_demo_wbc_base_frame_consistency
    test/test_demo_wbc_base_frame_consistency.cpp)
  target_link_libraries(test_demo_wbc_base_frame_consistency
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_demo_wbc_base_frame_consistency
    rclcpp
    rtc_controllers
    rtc_base
    pinocchio
    rtc_tsid
    rtc_mpc
    Eigen3
  )

  # ── demo_wbc_controller shipped-config lint (C-0.2) ────────────────────────
  # Parses the shipped YAML directly (no model / lifecycle); RTC_WBC_CONFIG_DIR
  # points at integrated_bringup/config.
  ament_add_gtest(test_demo_wbc_config_lint test/test_demo_wbc_config_lint.cpp)
  target_link_libraries(test_demo_wbc_config_lint yaml-cpp)
  target_compile_definitions(test_demo_wbc_config_lint PRIVATE
    RTC_WBC_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")

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
  target_compile_definitions(test_grasp_phase_manager PRIVATE
    RTC_PANDA_URDF_PATH="${RTC_PANDA_URDF_PATH}")

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
  target_compile_definitions(test_grasp_pipeline PRIVATE
    RTC_PANDA_URDF_PATH="${RTC_PANDA_URDF_PATH}")
  # Aligator's contact_rich solver (gar StageFactor / ArenaMatrix) requires
  # mimalloc as the process allocator: it performs aligned/interior frees that
  # glibc free() rejects ("free(): invalid pointer"), but mimalloc tolerates.
  # libmimalloc is only *partially* adopted by default (pulled in transitively,
  # overriding C++ new/delete but not C malloc/free + Eigen) — a split that
  # corrupts the heap on the cross-mode swap. Preload it so the allocator is
  # uniform. Production launches preload it too (launch/*.launch.py: CM process
  # additional_env LD_PRELOAD=libmimalloc.so.2).
  find_package(mimalloc QUIET)
  if(TARGET mimalloc)
    set_property(TEST test_grasp_pipeline APPEND PROPERTY
      ENVIRONMENT "LD_PRELOAD=$<TARGET_FILE:mimalloc>")
  endif()

  # ── Stage A-3: ForceReferenceUpdater unit tests (pure helper, no model) ───
  ament_add_gtest(test_force_reference_updater test/test_force_reference_updater.cpp)
  target_link_libraries(test_force_reference_updater
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_force_reference_updater
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

  # ── Pull-estimator wiring test (#167 P3) ───────────────────────────────────
  ament_add_gtest(test_pull_estimator_wiring test/test_pull_estimator_wiring.cpp)
  target_link_libraries(test_pull_estimator_wiring
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_pull_estimator_wiring
    rclcpp
    rtc_controllers
    rtc_base
    pinocchio
    Eigen3
  )

  # ── Closed-chain hand FK helper test (#121 Phase 2c) ───────────────────────
  # Reuses rtc_urdf_bridge/test fixtures (closure_test_fixtures.hpp,
  # test_urdf_path.hpp) — test-only, never-installed headers resolved via a
  # source-relative include dir (same pattern as rtc_controllers tests).
  ament_add_gtest(test_closed_chain_hand_fk test/test_closed_chain_hand_fk.cpp)
  target_include_directories(test_closed_chain_hand_fk PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_urdf_bridge/test")
  target_link_libraries(test_closed_chain_hand_fk
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_closed_chain_hand_fk
    rclcpp
    rtc_urdf_bridge
    rtc_base
    pinocchio
    Eigen3
  )

  # #120: closed-chain 축약 동역학 provider (WbcReducedDynamicsProvider) 단위 테스트.
  # 좌표 정렬(이름 permutation)·scatter 값 일치·held last-good·open-chain fallback 검증.
  ament_add_gtest(test_wbc_reduced_dynamics_provider test/test_wbc_reduced_dynamics_provider.cpp)
  target_include_directories(test_wbc_reduced_dynamics_provider PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_urdf_bridge/test")
  target_link_libraries(test_wbc_reduced_dynamics_provider
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_wbc_reduced_dynamics_provider
    rclcpp
    rtc_urdf_bridge
    rtc_tsid
    rtc_base
    pinocchio
    Eigen3
  )

  # ── Phase 3 (unified kin&dyn): loop-consistent contact frame J·oMf ──────────
  # Proves WbcReducedDynamicsProvider::FillReducedFrameKinematics raises loop-
  # downstream contact frame J/oMf to loop-consistent values (matching the
  # non-RT converged handle, differing from frozen-loop), with L1 last-good hold
  # / L2-zero dJv / serial byte-for-byte. Reuses rtc_urdf_bridge/test fixtures.
  ament_add_gtest(test_wbc_contact_kinematics_loop_consistent
    test/test_wbc_contact_kinematics_loop_consistent.cpp)
  target_include_directories(test_wbc_contact_kinematics_loop_consistent PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_urdf_bridge/test")
  target_link_libraries(test_wbc_contact_kinematics_loop_consistent
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_wbc_contact_kinematics_loop_consistent
    rclcpp
    rtc_urdf_bridge
    rtc_tsid
    rtc_base
    pinocchio
    Eigen3
  )

  # ── Phase 0 unified kin/dyn RT-cost benchmark (controller-unification) ──────
  # Measures per-call latency of baseline arm-only FK/Jacobian (joint/task
  # controllers today) vs unified PinocchioCache::Update on the combined
  # arm+hand model, for iiwa7_leap + ur5e_p1b. Report-only (no timing asserts).
  ament_add_gtest(test_unified_kindyn_bench test/test_unified_kindyn_bench.cpp)
  target_link_libraries(test_unified_kindyn_bench
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_unified_kindyn_bench
    ament_index_cpp
    rtc_urdf_bridge
    rtc_tsid
    rtc_base
    pinocchio
    Eigen3
  )

  # ── Phase 2 (unified kin&dyn): arm TCP cache-vs-arm_handle equivalence ───────
  # Proves the WBC controller's clik_tcp/clik_base cache oMf equals the arm-only
  # arm_handle_ FK it replaces in FillLog/FillPublish/ComputeTcpError, on both
  # the serial (iiwa7_leap) and closed-chain (ur5e_p1b) profiles.
  ament_add_gtest(test_wbc_arm_tcp_cache_equivalence
    test/test_wbc_arm_tcp_cache_equivalence.cpp)
  target_link_libraries(test_wbc_arm_tcp_cache_equivalence
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_wbc_arm_tcp_cache_equivalence
    ament_index_cpp
    rtc_urdf_bridge
    rtc_tsid
    rtc_base
    pinocchio
    Eigen3
  )

  # Unified kin&dyn Task B (옵션 B): WBC cache.Update relocated to ComputeControl
  # so arm/contact oMf refresh every non-E-STOP tick (kFallback/non-TSID included).
  # Proves the registered clik_tcp/clik_base oMf TRACK measured across consecutive
  # per-tick Updates (not frozen) and equal arm_handle_ FK each tick (1e-12), on
  # both the serial (iiwa7_leap) and closed-chain (ur5e_p1b) profiles.
  ament_add_gtest(test_wbc_fallback_omf_freshness
    test/test_wbc_fallback_omf_freshness.cpp)
  target_link_libraries(test_wbc_fallback_omf_freshness
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_wbc_fallback_omf_freshness
    ament_index_cpp
    rtc_urdf_bridge
    rtc_tsid
    rtc_base
    pinocchio
    Eigen3
  )

  # Unified kin&dyn Phase 4: task arm TCP FK + tip Jacobian cache equivalence.
  # Proves DemoTaskController's move off arm_handle_ onto the combined-model
  # PinocchioCache is value-preserving — FK oMf AND the arm-column extraction of
  # the combined Jacobian — on both the serial (iiwa7_leap) and closed-chain
  # (ur5e_p1b) profiles.
  ament_add_gtest(test_task_arm_cache_equivalence
    test/test_task_arm_cache_equivalence.cpp)
  target_link_libraries(test_task_arm_cache_equivalence
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_task_arm_cache_equivalence
    ament_index_cpp
    rtc_urdf_bridge
    rtc_base
    pinocchio
    Eigen3
  )

  # Unified kin&dyn Phase 5: joint arm TCP FK cache equivalence (FK-only — the
  # joint controller has no Jacobian consumer). Proves DemoJointController's move
  # off arm_handle_ onto the combined-model PinocchioCache preserves the arm TCP
  # oMf on both the serial (iiwa7_leap) and closed-chain (ur5e_p1b) profiles.
  ament_add_gtest(test_joint_arm_cache_equivalence
    test/test_joint_arm_cache_equivalence.cpp)
  target_link_libraries(test_joint_arm_cache_equivalence
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_joint_arm_cache_equivalence
    ament_index_cpp
    rtc_urdf_bridge
    rtc_base
    pinocchio
    Eigen3
  )

  # Phase ③ scatter permutation on the REAL ur5e_p1b closure (non-identity
  # cache_v_idx_ column map, which the CrankRocker nv==1 cases cannot exercise).
  ament_add_gtest(test_wbc_contact_permutation_real
    test/test_wbc_contact_permutation_real.cpp)
  target_link_libraries(test_wbc_contact_permutation_real
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_wbc_contact_permutation_real
    ament_index_cpp
    rtc_urdf_bridge
    rtc_tsid
    rtc_base
    pinocchio
    Eigen3
  )

  # ── Device backend registration smoke test ─────────────────────────────────
  # Confirms the 3 backend TUs (mujoco_native / ur_driver_native /
  # udp_hand_native) static-init through --whole-archive and reach the
  # DeviceBackendRegistry singleton at startup.
  ament_add_gtest(test_device_backends_registered
    test/test_device_backends_registered.cpp)
  target_link_libraries(test_device_backends_registered
    -Wl,--whole-archive
    ${PROJECT_NAME}_demo_controllers
    -Wl,--no-whole-archive
  )
  ament_target_dependencies(test_device_backends_registered
    rclcpp
    rclcpp_lifecycle
    rtc_controller_manager
    rtc_base
    rtc_msgs
    sensor_msgs
    std_msgs
  )

  # ── JointStateReorder fixed-capacity helper test (issue #156) ──────────────
  # Header-only helper shared by the three device backends: behavior parity
  # with the former lazy std::vector reorder (named/un-named/unknown/over-
  # capacity) plus an operator-new counter guarding allocation-freedom on the
  # rt_callback lane.
  ament_add_gtest(test_joint_state_reorder test/test_joint_state_reorder.cpp)
  target_include_directories(test_joint_state_reorder PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_joint_state_reorder
    rtc_controller_manager
    rtc_base
    sensor_msgs
  )

  # ── MujocoNativeBackend fingertip wrench lane test ─────────────────────────
  # Layer C: verifies HasSensorState / ReadSensorState / OnWrench contract —
  # stride-7 inference_data packing, stale detection, NaN guard, per-tip
  # miss counter independence, recovery, and dead-slot zero-fill.
  ament_add_gtest(test_mujoco_native_backend test/test_mujoco_native_backend.cpp)
  target_link_libraries(test_mujoco_native_backend
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_mujoco_native_backend
    rclcpp
    rclcpp_lifecycle
    rtc_controller_manager
    rtc_base
    geometry_msgs
  )

  # ── Controller-owned RobotTarget cb_group invariant test ───────────────────
  # Spec §0d follow-up to Issue #100: locks that CreateOwnedTopics() routes
  # the controller-owned RobotTarget sub onto the LifecycleNode default
  # callback group (no SubscriptionOptions argument), so CM attaches it to
  # nrt_callback_executor instead of any RT-priority lane.
  ament_add_gtest(test_controller_target_cb_group_invariant
    test/test_controller_target_cb_group_invariant.cpp)
  target_link_libraries(test_controller_target_cb_group_invariant
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_controller_target_cb_group_invariant
    rclcpp
    rclcpp_lifecycle
    rtc_base
    rtc_controller_interface
    rtc_msgs
  )

  # ── demo_controller_gui static robot profiles (pure-Python) ────────────────
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_robot_profiles test/test_robot_profiles.py)
  ament_add_pytest_test(test_demo_gui_catalog test/test_demo_gui_catalog.py)
  ament_add_pytest_test(test_demo_gui_presets test/test_demo_gui_presets.py)
endif()

ament_package()
