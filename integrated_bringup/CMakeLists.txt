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
    -Werror=switch  # enum switch 누락은 dead topic 을 낳는다 (#211)
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
  # #135 Layer 1: momentum-observer wiring. Owns the coordinate contract
  # (RtModelHandle takes device order, returns Pinocchio order) and the
  # per-lane readability gate the observer itself cannot ask.
  src/support/momentum_observer_wiring.cpp
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

  # UR5e fixture for tests that need DemoWbcController::LoadConfig to actually
  # build a model: its fallback branch hardcodes base_link -> tool0, which this
  # URDF has and panda.urdf does not (panda_link0 / panda_link8).
  get_filename_component(_rtc_ur5e_default
      "${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots/ur5e/urdf/ur5e.urdf"
      ABSOLUTE)
  set(RTC_UR5E_URDF_PATH "${_rtc_ur5e_default}"
      CACHE FILEPATH "Path to ur5e.urdf used by integrated_bringup tests")
  if(DEFINED ENV{RTC_PANDA_URDF_PATH})
    set(RTC_PANDA_URDF_PATH "$ENV{RTC_PANDA_URDF_PATH}" CACHE FILEPATH
        "Path to panda.urdf used by integrated_bringup tests" FORCE)
  endif()

  # ── Fingertip force delta-spike guard (header-only, no ROS/Eigen deps) ────
  # Instantiated in the test TU so its TU-global operator new counter observes
  # every allocation the guard could make on the RT path.
  ament_add_gtest(test_fingertip_force_guard test/test_fingertip_force_guard.cpp)
  target_include_directories(test_fingertip_force_guard PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

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

  # ── F5 device-readability gate at the binding (#236 S7b) ───────────────────
  # Deliberately URDF-free: every assertion is a channel count or a joint
  # command, so the serial_6dof vacuity trap (§3.3) cannot reach it.
  ament_add_gtest(test_device_readability_gate test/test_device_readability_gate.cpp)
  target_link_libraries(test_device_readability_gate
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_device_readability_gate
    ament_index_cpp
    rclcpp
    rtc_controller_interface
    rtc_controllers
    rtc_base
    pinocchio
    Eigen3
  )

  # ── The closed gate's diagnostic (#307) ───────────────────────────────────
  # A SEPARATE executable from test_device_readability_gate on purpose:
  # RCLCPP_*_THROTTLE keeps its window in a static at the macro's expansion
  # point, so it is per-process, and that suite drives the same call sites with
  # narrow devices many times. Sharing a binary would let a suppressed line
  # masquerade as a line that was never written. See the file header.
  #
  # ONE executable, TWO ctest registrations. hand_dof_ has two sources and the
  # diagnostic names a different fix for each, but both variants leave the same
  # three call sites — so observing both needs two PROCESSES, and a second gtest
  # case in the same process would assert against a window the first one
  # consumed. Registering the same binary twice is cheaper than a second link
  # (pinocchio) and keeps the fixtures in one file.
  #
  # The second filter is the COMPLEMENT of the first, not a second positive
  # pattern. That makes the partition total: a test whose name matches neither
  # intent still runs, instead of being silently skipped by both registrations
  # while the suite reports green. ament_add_gtest_test derives its result XML
  # from TEST_NAME, so the two runs cannot overwrite each other's report.
  #
  # Neither run tolerates --gtest_repeat: log_clock_ is RCL_STEADY_TIME, so a
  # second iteration finds every throttle window already consumed. See the file
  # header.
  ament_add_gtest_executable(test_gate_closure_diagnostic
    test/test_gate_closure_diagnostic.cpp
  )
  target_link_libraries(test_gate_closure_diagnostic
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_gate_closure_diagnostic
    ament_index_cpp
    rclcpp
    rtc_controller_interface
    rtc_controllers
    rtc_base
    pinocchio
    Eigen3
  )
  ament_add_gtest_test(test_gate_closure_diagnostic
    TEST_NAME test_gate_closure_diagnostic_declared_width
    ENV "GTEST_FILTER=*DeclaredWidth*"
  )
  ament_add_gtest_test(test_gate_closure_diagnostic
    TEST_NAME test_gate_closure_diagnostic_device_report
    ENV "GTEST_FILTER=-*DeclaredWidth*"
  )

  # ── The same gate on the shared model scatter (#236 S7b, audit S1) ─────────
  # URDF-backed on purpose: the claim it pins ("a bound leaves the skipped slots
  # at their previous value") is about a buffer that persists between calls, so
  # it needs a real reorder map. Every assertion is a configuration vector.
  ament_add_gtest(test_combined_model_cache_gate test/test_combined_model_cache_gate.cpp)
  target_link_libraries(test_combined_model_cache_gate
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_combined_model_cache_gate
    ament_index_cpp
    rclcpp
    rtc_controller_interface
    rtc_base
    rtc_urdf_bridge
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

  # ── #282 two-tier DLS convergence harness ──────────────────────────────────
  # Pure numerics: the pre-#282 inline block (oracle) against
  # compliance::DifferentialIk. Its own binary because it installs the Eigen
  # allocation tripwire (no_malloc_scope.hpp defines EIGEN_RUNTIME_NO_MALLOC for
  # the whole TU) and a replacement operator new (alloc_gate.hpp, one TU per
  # binary) — neither belongs in a binary that also builds a live controller.
  # Test-only headers from rtc_base/rtc_controllers resolve via source-relative
  # include dirs, the same never-installed idiom test_closed_chain_hand_fk uses.
  ament_add_gtest(test_task_dls_convergence test/test_task_dls_convergence.cpp)
  target_include_directories(test_task_dls_convergence PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/test"
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_controllers/test/include")
  rtc_base_add_test_includes(test_task_dls_convergence)
  target_link_libraries(test_task_dls_convergence
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_task_dls_convergence
    ament_index_cpp
    rclcpp
    rtc_controllers
    rtc_base
    rtc_urdf_bridge
    pinocchio
    Eigen3
  )
  # alloc_gate.hpp pairs new→malloc with delete→free, which makes GCC
  # false-positive -Wmismatched-new-delete program-wide (alloc_gate.hpp item 2).
  target_compile_options(test_task_dls_convergence PRIVATE -Wno-mismatched-new-delete)

  # ── Task-velocity assembly convergence (#314) ──────────────────────────────
  # Compares two pure functions, so unlike its #282 sibling it needs no URDF, no
  # controller library and no pinocchio — only the core header and the
  # never-installed bit_compare.hpp, by source-relative include dir.
  ament_add_gtest(test_task_vel_binding test/test_task_vel_binding.cpp)
  target_include_directories(test_task_vel_binding PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_controllers/test/include")
  ament_target_dependencies(test_task_vel_binding
    rtc_controllers
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

  # ── demo_wbc_controller TSID-path tests (#189) ─────────────────────────────
  # The only DemoWbcController test that brings up a real TSID + CLIK stack:
  # it drives the iiwa7_leap URDF fixture through LoadConfig(tsid:) AND
  # on_configure (where InitClik lives), so the TSID routing subtree of
  # src/controllers/wbc/compute.cpp actually executes. That means a
  # LifecycleNode and therefore DDS, so it gets a dedicated ROS_DOMAIN_ID —
  # colcon test runs packages in parallel and discovery churn on the default
  # domain breaks other packages' pub/sub matching tests, or hangs them outright
  # in rmw endpoint create/destroy (#401). One id per package, shared by every
  # test here that opens a participant -- ctest runs a package's tests
  # sequentially, so siblings never overlap. Do not restate the allocation in a
  # comment: `validate_test_domains.py --list` derives it, and the gate of the
  # same name fails the build on a collision or on an unclaimed participant test
  # (this comment used to say "domains 44-51 are theirs", which stopped being
  # true the moment that package consolidated).
  # Slower than the model-less wbc tests: 4-contact QP solves at 2 ms over
  # multi-tick FSM scenarios, so the default 60 s ctest timeout is lifted.
  ament_add_gtest(test_demo_wbc_tsid_path
    test/test_demo_wbc_tsid_path.cpp
    ENV ROS_DOMAIN_ID=52
    TIMEOUT 300)
  target_link_libraries(test_demo_wbc_tsid_path
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_demo_wbc_tsid_path
    ament_index_cpp
    rclcpp
    rclcpp_lifecycle
    rtc_controllers
    rtc_base
    rtc_urdf_bridge
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

  # ── #175: closed-chain 사영 공유 (컨트롤러 레벨) ───────────────────────────
  # 실제 ur5e_p1b URDF+closure 와 shipped WBC config 로 bring-up 을 재생한다 —
  # 사영 2→1 계측과 입력 provenance 는 URDF 없는 하네스에서는 관측되지 않는다.
  ament_add_gtest(test_wbc_closed_chain_projection_sharing
    test/test_wbc_closed_chain_projection_sharing.cpp
    ENV ROS_DOMAIN_ID=52
    TIMEOUT 300)
  target_link_libraries(test_wbc_closed_chain_projection_sharing
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  target_compile_definitions(test_wbc_closed_chain_projection_sharing PRIVATE
    RTC_WBC_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  ament_target_dependencies(test_wbc_closed_chain_projection_sharing
    ament_index_cpp
    rclcpp
    rclcpp_lifecycle
    rtc_controllers
    rtc_base
    rtc_urdf_bridge
    rtc_tsid
    pinocchio
    Eigen3
  )

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
    ament_add_gtest(test_wbc_layout_profile_gate
    test/test_wbc_layout_profile_gate.cpp)
  target_link_libraries(test_wbc_layout_profile_gate
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  target_compile_definitions(test_wbc_layout_profile_gate PRIVATE
    RTC_UR5E_URDF_PATH="${RTC_UR5E_URDF_PATH}"
    RTC_WBC_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/ur5e_p1a/controllers/demo_wbc_controller.yaml")
  ament_target_dependencies(test_wbc_layout_profile_gate
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
  # Lets the deployed-tuning test parse the shipped YAML instead of a copy of it
  # — see DeployedForcePiTuning in that file.
  target_compile_definitions(test_demo_shared_config PRIVATE
    RTC_DEMO_SHARED_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
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

  # ── Shipped-profile pull estimator vs a known load (#177 crit#4) ───────────
  # Parses the shipped iiwa7_leap demo_shared.yaml (same RTC_DEMO_SHARED_CONFIG_DIR
  # route as test_demo_shared_config) so the deployed force_sign is what runs.
  ament_add_gtest(test_pull_estimator_known_wrench test/test_pull_estimator_known_wrench.cpp)
  target_link_libraries(test_pull_estimator_known_wrench
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  target_compile_definitions(test_pull_estimator_known_wrench PRIVATE
    RTC_DEMO_SHARED_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  ament_target_dependencies(test_pull_estimator_known_wrench
    rclcpp
    rtc_controllers
    rtc_base
    pinocchio
    Eigen3
  )

  # ── Compliance wrench source adapter (#469 S1) ─────────────────────────────
  # Shares pull_known_load_fixture.hpp with the crit#4 test above, so both drive
  # the same known load through the shipped profile.
  ament_add_gtest(test_compliance_wrench_source test/test_compliance_wrench_source.cpp)
  target_link_libraries(test_compliance_wrench_source
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  target_compile_definitions(test_compliance_wrench_source PRIVATE
    RTC_DEMO_SHARED_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  ament_target_dependencies(test_compliance_wrench_source
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

  # #135 Layer 1: the momentum-observer wiring — the lane gate plus the
  # coordinate contract (device order in, Pinocchio order out), checked against
  # a real iiwa7 arm sub-model because a mixed order produces a finite, smooth,
  # wrong residual that no gate or NaN check would catch.
  #
  # TIMEOUT because the cases here settle the observer over thousands of ticks,
  # each a full dynamics + Jacobian evaluation. That is fast in the gated
  # RelWithDebInfo build and roughly two orders slower under the coverage job's
  # Debug+gcov build, where the default 60 s bound was actually hit — and a
  # timeout there reads as "not verified", not as a slow pass.
  ament_add_gtest(test_momentum_observer_wiring
    test/test_momentum_observer_wiring.cpp
    TIMEOUT 180)
  target_link_libraries(test_momentum_observer_wiring
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_momentum_observer_wiring
    ament_index_cpp
    rtc_urdf_bridge
    rtc_controllers
    rtc_controller_interface
    rtc_base
    pinocchio
    Eigen3
  )

  # #135 Layer 1b: the same observer inside the three production controllers.
  # The residual has no consumer at this layer, so momentum_observer.csv is the
  # only surface — these cases register the channel the way on_configure does
  # and read the file back, which is also what keeps the header/row widths and
  # the per-tick push on the tested path.
  # §MomentumObserverLifecycleGate runs on_configure on a real LifecycleNode
  # (the only place the PayloadEstimate gate branch lives), so this target opens
  # a DDS participant and takes the package ROS_DOMAIN_ID like its siblings.
  ament_add_gtest(test_momentum_observer_embedding
    test/test_momentum_observer_embedding.cpp
    TIMEOUT 180
    ENV ROS_DOMAIN_ID=52)
  target_link_libraries(test_momentum_observer_embedding
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_momentum_observer_embedding
    ament_index_cpp
    rclcpp
    rclcpp_lifecycle
    rtc_urdf_bridge
    rtc_controllers
    rtc_controller_interface
    rtc_base
    rtc_tsid
    rtc_mpc
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
    rtc_controller_interface
    rtc_base
    sensor_msgs
  )

  # ── MujocoNativeBackend fingertip wrench lane test ─────────────────────────
  # Layer C: verifies HasSensorState / ReadSensorState / OnWrench contract —
  # stride-7 inference_data packing, stale detection, NaN guard, per-tip
  # miss counter independence, recovery, and dead-slot zero-fill.
  ament_add_gtest(test_mujoco_native_backend test/test_mujoco_native_backend.cpp
    ENV ROS_DOMAIN_ID=52)
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
    test/test_controller_target_cb_group_invariant.cpp
    ENV ROS_DOMAIN_ID=52)
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

  # #135 D12 PayloadEstimate wire surface. Opens a participant (the publisher
  # helpers create a real LifecyclePublisher on a LifecycleNode), so it takes
  # the package's ROS_DOMAIN_ID like its siblings above.
  ament_add_gtest(test_payload_estimate_topic
    test/test_payload_estimate_topic.cpp
    ENV ROS_DOMAIN_ID=52)
  target_link_libraries(test_payload_estimate_topic
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_payload_estimate_topic
    rclcpp
    rclcpp_lifecycle
    rtc_base
    rtc_controller_interface
    rtc_controllers
    rtc_msgs
  )

  # ── demo_controller_gui static robot profiles (pure-Python) ────────────────
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_robot_profiles test/test_robot_profiles.py)
  ament_add_pytest_test(test_cm_rt_params_wiring test/test_cm_rt_params_wiring.py)
  ament_add_pytest_test(test_launch_shield_wiring test/test_launch_shield_wiring.py)
  ament_add_pytest_test(test_launch_hand_affinity_wiring
    test/test_launch_hand_affinity_wiring.py)
  # Behavioural twin of the two AST tests above: evaluates every launch
  # description instead of reading it. Named in the gated-test CI job's ctest
  # regex (.github/workflows/ros2-advanced-ci.yml) — renaming this target
  # without updating that regex would drop it back out of the merge gate, which
  # --no-tests=error there turns into a red job rather than a silent skip.
  ament_add_pytest_test(test_launch_description_evaluates
    test/test_launch_description_evaluates.py)
  ament_add_pytest_test(test_launch_session_wiring
    test/test_launch_session_wiring.py)
  ament_add_pytest_test(test_demo_gui_catalog test/test_demo_gui_catalog.py)
  ament_add_pytest_test(test_demo_gui_presets test/test_demo_gui_presets.py)
  ament_add_pytest_test(test_demo_gui_pull test/test_demo_gui_pull.py)
  ament_add_pytest_test(test_demo_gui_task_frame test/test_demo_gui_task_frame.py)
  ament_add_pytest_test(test_demo_gui_grasp_mode test/test_demo_gui_grasp_mode.py)
endif()

ament_package()
