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
find_package(rtc_inference REQUIRED)
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
  # #469 S2: task-space admittance binding. Currently a rename-only copy of
  # src/controllers/task/ — the law lands in S3.
  src/controllers/compliance/controller.cpp
  src/controllers/compliance/compute.cpp
  src/controllers/compliance/lifecycle.cpp
  src/controllers/compliance/parameters.cpp
  src/controllers/wbc/controller.cpp
  src/controllers/wbc/compute.cpp
  src/controllers/wbc/lifecycle.cpp
  src/controllers/wbc/parameters.cpp
  src/controllers/wbc/phase.cpp
  src/controllers/wbc/grasp_phase_manager.cpp
  src/controllers/wbc/force_reference_updater.cpp
  # Learned-policy binding. Its law half lives in rtc_controllers/inference
  # (tensor marshalling) — this side owns the ControllerState/engine contract.
  src/controllers/inference/controller.cpp
  src/controllers/inference/compute.cpp
  src/controllers/inference/lifecycle.cpp
  src/controllers/inference/parameters.cpp
  # dynamic_catching S4.0 binding: arm hold + unshaped hand step, sim-only.
  # No law half in rtc_controllers yet — the catching law lands with S5.
  src/controllers/catching/controller.cpp
  src/controllers/catching/traj_input.cpp
  src/controllers/catching/lifecycle.cpp
  src/controllers/catching/planner_thread.cpp
  src/controllers/dualarm/controller.cpp
  src/controllers/dualarm/compute.cpp
  src/controllers/dualarm/lifecycle.cpp
  src/controllers/dualarm/parameters.cpp
  src/support/combined_model_cache.cpp
  src/support/demo_shared_config.cpp
  src/support/pull_estimator_wiring.cpp
  # #135 Layer 1: momentum-observer wiring. Owns the coordinate contract
  # (RtModelHandle takes device order, returns Pinocchio order) and the
  # per-lane readability gate the observer itself cannot ask.
  src/support/momentum_observer_wiring.cpp
  src/support/closed_chain_hand_fk.cpp
  src/support/hand_fk_wiring.cpp
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
  rtc_inference
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

install(
  PROGRAMS scripts/catching_sim_trials.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME catching_sim_trials
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
  rtc_inference
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

  # Shared binaries: files that share one keep their own claims; the binary only
  # saves the process start and the link. A file that replaces the global
  # operator new, installs the Eigen malloc tripwire, owns process state (env, a
  # throttle window, --whole-archive registration, LD_PRELOAD) or opens a DDS
  # participant keeps a binary of its own — the comments at those targets say
  # which. Sources stay literal in ament_add_gtest (validate_test_domains.py
  # reads them); ib_timeout_per_file then sets TIMEOUT to the default 60 s per
  # source, so a shared binary keeps the budget each file had on its own (the
  # Debug+gcov build runs about two orders slower; see
  # test_momentum_observer_wiring). Call it before anything else adds sources.
  function(ib_timeout_per_file target)
    get_target_property(_srcs ${target} SOURCES)
    list(LENGTH _srcs _n)
    math(EXPR _timeout "60 * ${_n}")
    set_tests_properties(${target} PROPERTIES TIMEOUT ${_timeout})
  endfunction()

  # ── Header-only helpers, one binary ────────────────────────────────────────
  #   test_device_log_pods   DeviceStateLog / DeviceSensorLog POD mirrors;
  #                          pull_estimator_log_pod.hpp mirrors
  #                          rtc::grasp::PullEstimate (#167).
  #   test_virtual_tcp       virtual_tcp.
  #   test_task_vel_binding  Task-velocity assembly convergence (#314): compares
  #                          two pure functions, so it needs no URDF, no
  #                          controller library and no pinocchio — only the core
  #                          header and the never-installed bit_compare.hpp, by
  #                          source-relative include dir.
  ament_add_gtest(test_header_only_helpers
    test/test_device_log_pods.cpp
    test/test_virtual_tcp.cpp
    test/test_task_vel_binding.cpp)
  ib_timeout_per_file(test_header_only_helpers)
  target_include_directories(test_header_only_helpers PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_controllers/test/include"
  )
  target_link_libraries(test_header_only_helpers pinocchio::pinocchio)
  ament_target_dependencies(test_header_only_helpers
    rtc_controllers
    rtc_base
    pinocchio
    Eigen3
  )

  # ── demo_wbc_controller, model-less and layout suites, one binary ─────────
  #   test_demo_wbc_controller               unit tests.
  #   test_demo_wbc_base_frame_consistency   base_frame consistency (F-2).
  #   test_demo_wbc_mpc_integration          MPC integration (Phase 5).
  #   test_wbc_layout_profile_gate           the shipped layout profile gate.
  #   test_wbc_position_margin_box           position_margin vs the CLIK
  #                                          position box (#473).
  #   test_demo_wbc_config_lint              shipped-config lint (C-0.2): parses
  #                                          the shipped YAML directly (no model
  #                                          / lifecycle); RTC_WBC_CONFIG_DIR
  #                                          points at integrated_bringup/config.
  ament_add_gtest(test_demo_wbc_suite
    test/test_demo_wbc_controller.cpp
    test/test_demo_wbc_base_frame_consistency.cpp
    test/test_demo_wbc_mpc_integration.cpp
    test/test_wbc_layout_profile_gate.cpp
    test/test_wbc_position_margin_box.cpp
    test/test_demo_wbc_config_lint.cpp)
  ib_timeout_per_file(test_demo_wbc_suite)
  target_link_libraries(test_demo_wbc_suite
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  target_compile_definitions(test_demo_wbc_suite PRIVATE
    RTC_UR5E_URDF_PATH="${RTC_UR5E_URDF_PATH}"
    RTC_WBC_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/ur5e_p1a/controllers/demo_wbc_controller.yaml"
    RTC_WBC_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  ament_target_dependencies(test_demo_wbc_suite
    rclcpp
    rtc_controllers
    rtc_base
    pinocchio
    rtc_tsid
    rtc_mpc
    Eigen3
  )

  # ── demo_joint_controller + activation generation gate, one binary ────────
  #   test_demo_joint_controller       empty-URDF unit test: grasp detection /
  #                                    fingertip derivation / ToF gate / none
  #                                    no-op. No rtc_tsid (joint controller is
  #                                    joint-space only).
  #   test_demo_joint_controller_urdf  URDF-backed (#149): model-dependent joint
  #                                    paths the empty-URDF sister cannot reach —
  #                                    arm quintic trajectory, fingertip FK/TF,
  #                                    virtual TCP, ToF populate, E-STOP FK.
  #   test_activation_generation_gate  #196 §3: stale targets queued while
  #                                    Inactive must not reach the target slot
  #                                    after re-activation.
  ament_add_gtest(test_demo_joint_suite
    test/test_demo_joint_controller.cpp
    test/test_demo_joint_controller_urdf.cpp
    test/test_activation_generation_gate.cpp)
  ib_timeout_per_file(test_demo_joint_suite)
  target_link_libraries(test_demo_joint_suite
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_demo_joint_suite
    ament_index_cpp
    rclcpp
    rclcpp_lifecycle
    rtc_controllers
    rtc_base
    rtc_urdf_bridge
    pinocchio
    Eigen3
  )

  # Learned-policy binding, driven with an injected fake engine so every success
  # path runs without a .onnx file — which is the point of the injection seam.
  # RT-1 gate on the real Compute(). Global operator-new counting reaches across
  # the library boundary; Eigen's malloc does not — see the file banner for what
  # that green does and does not cover. The shipped-config cases replay the real
  # ur5e_p1b model (robot_descriptions) and read the shipped controller YAML.
  # The ONLY test target that links --whole-archive, and it has to: the
  # registration TUs are anonymous-namespace static initializers, so without it
  # rtc::ControllerRegistry is EMPTY in the test process and every assertion in
  # this suite passes vacuously. The suite's first canary fails loudly if this
  # flag is ever dropped. No node, no model, no ONNX session — it constructs
  # each registered controller and probes LoadConfig with an empty node.
  ament_add_gtest(test_registered_controllers_have_shipped_config
    test/test_registered_controllers_have_shipped_config.cpp
    ENV ROS_DOMAIN_ID=52)
  target_link_libraries(test_registered_controllers_have_shipped_config
    -Wl,--whole-archive
    ${PROJECT_NAME}_demo_controllers
    -Wl,--no-whole-archive
    yaml-cpp
  )
  target_compile_definitions(test_registered_controllers_have_shipped_config PRIVATE
    RTC_DEMO_SHARED_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  ament_target_dependencies(test_registered_controllers_have_shipped_config
    ament_index_cpp
    rclcpp
    rclcpp_lifecycle
    rtc_controller_interface
    rtc_controllers
    rtc_inference
    rtc_base
    rtc_urdf_bridge
  )

  # Configures a real LifecycleNode, so it opens a DDS participant and takes the
  # package ROS_DOMAIN_ID like its siblings (#401).
  ament_add_gtest(test_demo_inference_alloc test/test_demo_inference_alloc.cpp
    ENV ROS_DOMAIN_ID=52
    TIMEOUT 300)
  target_include_directories(test_demo_inference_alloc PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_controllers/test/include")
  target_link_libraries(test_demo_inference_alloc
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  target_compile_definitions(test_demo_inference_alloc PRIVATE
    RTC_DEMO_SHARED_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  ament_target_dependencies(test_demo_inference_alloc
    ament_index_cpp
    rclcpp
    rclcpp_lifecycle
    rtc_controllers
    rtc_inference
    rtc_base
    rtc_urdf_bridge
    pinocchio
    Eigen3
  )

  # Same: a real LifecycleNode → DDS participant → the package ROS_DOMAIN_ID.
  ament_add_gtest(test_demo_inference_controller test/test_demo_inference_controller.cpp
    ENV ROS_DOMAIN_ID=52)
  target_link_libraries(test_demo_inference_controller
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_demo_inference_controller
    rclcpp
    rclcpp_lifecycle
    rtc_controllers
    rtc_inference
    rtc_base
    pinocchio
    Eigen3
  )

  # The same binding on the real ur5e_p1b model (robot_descriptions, #457) and
  # the SHIPPED controller YAML: link poses, the closed-chain fingertips, the
  # base ↔ base_link half turn and the reach gate need the model, and "the
  # shipped config observes the robot as the policy was trained" needs the file.
  ament_add_gtest(test_demo_inference_urdf test/test_demo_inference_urdf.cpp
    ENV ROS_DOMAIN_ID=52
    TIMEOUT 300)
  target_link_libraries(test_demo_inference_urdf
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  target_compile_definitions(test_demo_inference_urdf PRIVATE
    RTC_DEMO_SHARED_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  ament_target_dependencies(test_demo_inference_urdf
    ament_index_cpp
    rclcpp
    rclcpp_lifecycle
    rtc_controllers
    rtc_inference
    rtc_base
    rtc_urdf_bridge
    pinocchio
    Eigen3
  )

  # The same binding on the REAL policy through ONNX Runtime — local only. The
  # policy stays out of the repo, so every model case SKIPS (= unverified, not
  # passed) unless RTC_POLICY_DIR names the directory holding the .onnx the
  # shipped config points at; CI always skips. Run() latency and operator-new
  # counts land in the test XML (RecordProperty). One TU with alloc_gate.hpp,
  # hence the include path and the warning flag its contract (item 2) asks for.
  ament_add_gtest(test_demo_inference_real_model test/test_demo_inference_real_model.cpp
    ENV ROS_DOMAIN_ID=52
    TIMEOUT 300)
  target_include_directories(test_demo_inference_real_model PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_controllers/test/include")
  target_compile_options(test_demo_inference_real_model PRIVATE -Wno-mismatched-new-delete)
  target_link_libraries(test_demo_inference_real_model
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  target_compile_definitions(test_demo_inference_real_model PRIVATE
    RTC_DEMO_SHARED_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  ament_target_dependencies(test_demo_inference_real_model
    ament_index_cpp
    rclcpp
    rclcpp_lifecycle
    rtc_controllers
    rtc_inference
    rtc_base
    rtc_urdf_bridge
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

  # ── demo_compliance_controller ≡ demo_task_controller (#469 S2) ────────────
  # The only sensor that executes the copied controller: every other
  # per-controller suite in this package enumerates the three shipped names by
  # hand, so a fourth controller is otherwise never run. Needs the same URDF
  # fixture as test_demo_task_controller (both controllers gate ComputeControl
  # behind the combined-model cache) plus the shipped config dir, because the
  # config half compares the deployed YAMLs rather than inline strings.
  ament_add_gtest(test_compliance_task_equivalence test/test_compliance_task_equivalence.cpp)
  target_compile_definitions(test_compliance_task_equivalence PRIVATE
    RTC_DEMO_SHARED_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  target_link_libraries(test_compliance_task_equivalence
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_compliance_task_equivalence
    ament_index_cpp
    rclcpp
    rtc_controllers
    rtc_base
    rtc_urdf_bridge
    pinocchio
    Eigen3
  )

  # ── #504: activation re-seed vs. a standing hand gap ───────────────────────
  # Same fixture as the equivalence suite (URDF model + the shipped controller
  # YAML), a separate binary because the claim is neither "same as demo_task" nor
  # "the §7 law": it is what the shipped GRASP PROCEDURE rests on, and the file
  # that fails should name that.
  ament_add_gtest(test_compliance_activation_hand_gap
    test/test_compliance_activation_hand_gap.cpp)
  target_compile_definitions(test_compliance_activation_hand_gap PRIVATE
    RTC_DEMO_SHARED_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  target_link_libraries(test_compliance_activation_hand_gap
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_compliance_activation_hand_gap
    ament_index_cpp
    rclcpp
    rclcpp_lifecycle
    rtc_controllers
    rtc_base
    rtc_urdf_bridge
    pinocchio
    Eigen3
  )

  # ── #469 S3: the §7 admittance law as this binding wires it ────────────────
  # A separate binary from the equivalence suite on purpose: that one asserts
  # this controller is INDISTINGUISHABLE from its sibling, this one asserts what
  # makes it different. Sharing a binary would put the two claims in one place
  # and make "which of them broke" a reading exercise.
  # §ComplianceMarginGate drives on_configure on a real LifecycleNode — the only
  # place the §7.3 margin/band check can run, since the band does not exist until
  # the CM's Pass 2 — so this target opens a DDS participant and takes the
  # package ROS_DOMAIN_ID like its siblings (#401).
  ament_add_gtest(test_compliance_admittance_coupling
    test/test_compliance_admittance_coupling.cpp
    ENV ROS_DOMAIN_ID=52)
  target_compile_definitions(test_compliance_admittance_coupling PRIVATE
    RTC_DEMO_SHARED_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  target_link_libraries(test_compliance_admittance_coupling
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_compliance_admittance_coupling
    ament_index_cpp
    rclcpp
    rtc_controllers
    rtc_base
    rtc_urdf_bridge
    pinocchio
    Eigen3
  )

  # ── #469 S4: the §7 diagnostics lane ───────────────────────────────────────
  # Its own binary, and not part of the coupling suite: this one writes real
  # files into a scoped RTC_SESSION_DIR and reads them back, so it owns process
  # state (an env var) the law tests do not. Sharing a binary would make an
  # unrelated failure there look like a logging failure here.
  ament_add_gtest(test_compliance_diag_log
    test/test_compliance_diag_log.cpp)
  target_compile_definitions(test_compliance_diag_log PRIVATE
    RTC_DEMO_SHARED_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  target_link_libraries(test_compliance_diag_log
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_compliance_diag_log
    ament_index_cpp
    rclcpp
    rtc_controllers
    rtc_base
    rtc_urdf_bridge
    pinocchio
    Eigen3
  )

  # ── dynamic_catching S4.0/S4.1: catching controller skeleton ───────────────
  # URDF-free: this controller runs no model, so the suite needs neither
  # pinocchio nor a robot description. It DOES build a LifecycleNode (the
  # configure-time refusals are half of what it pins), hence ROS_DOMAIN_ID, and
  # it reads the shipped profiles, hence RTC_DEMO_SHARED_CONFIG_DIR.
  ament_add_gtest(test_demo_catching_controller test/test_demo_catching_controller.cpp
    ENV ROS_DOMAIN_ID=52
  )
  target_include_directories(test_demo_catching_controller PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/test")
  target_compile_definitions(test_demo_catching_controller PRIVATE
    RTC_DEMO_SHARED_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  target_link_libraries(test_demo_catching_controller
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_demo_catching_controller
    ament_index_cpp
    rclcpp
    rclcpp_lifecycle
    rtc_controller_interface
    rtc_controllers
    rtc_base
    pinocchio
    Eigen3
  )

  # Separate binary: alloc_gate.hpp replaces global operator new, which is one
  # TU per binary (alloc_gate.hpp contract item 1).
  ament_add_gtest(test_demo_catching_alloc test/test_demo_catching_alloc.cpp)
  target_include_directories(test_demo_catching_alloc PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_controllers/test/include")
  target_link_libraries(test_demo_catching_alloc
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_demo_catching_alloc
    ament_index_cpp
    rclcpp
    rtc_controller_interface
    rtc_controllers
    rtc_base
    pinocchio
    Eigen3
  )
  # alloc_gate.hpp pairs new→malloc with delete→free, which makes GCC
  # false-positive -Wmismatched-new-delete program-wide (alloc_gate.hpp item 2).
  target_compile_options(test_demo_catching_alloc PRIVATE -Wno-mismatched-new-delete)

  # dynamic_catching S7 (G7-D): one gated tick in every ARMED mode of a trial
  # on the real UR5e+P1b model (homing, law, freeze, hand sequencer, DECEL,
  # HOLD, return, joint-space stop). Own binary: alloc_gate.hpp replaces the
  # global operator new (one TU per binary).
  ament_add_gtest(test_demo_catching_alloc_s7 test/test_demo_catching_alloc_s7.cpp
    ENV ROS_DOMAIN_ID=52)
  target_include_directories(test_demo_catching_alloc_s7 PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_controllers/test/include")
  target_link_libraries(test_demo_catching_alloc_s7
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_demo_catching_alloc_s7
    rclcpp
    rclcpp_lifecycle
    rtc_controllers
    rtc_urdf_bridge
    rtc_base
    sensor_msgs
    ament_index_cpp
    pinocchio
    Eigen3
  )
  target_compile_options(test_demo_catching_alloc_s7 PRIVATE -Wno-mismatched-new-delete)
  # The second robot's gate (E1-F17 D7) brings the controller up from the
  # shipped iiwa7_leap profile.
  target_compile_definitions(test_demo_catching_alloc_s7 PRIVATE
    RTC_DEMO_SHARED_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")

  # S7 G8-A2 (truth half): the reset table's rows hold at run time — every
  # RT-owned member poisoned, a reset run, each member checked against its row.
  ament_add_gtest(test_catching_reset_probe test/test_catching_reset_probe.cpp)
  target_link_libraries(test_catching_reset_probe ${PROJECT_NAME}_demo_controllers)
  ament_target_dependencies(test_catching_reset_probe
    rclcpp
    rtc_controllers
    rtc_base
    Eigen3
  )

  # S9a: /rtc_cm/clear_estop and /rtc_cm/reset_fault over the wire, on a real
  # RtControllerNode hosting the catching controller and running the CM's own
  # tick body — the seam between the CM's latch services and the catching P-1
  # contract that the operator GUI drives. Opens participants (the CM node, a
  # service client, the controller's LifecycleNode), hence the package domain.
  ament_add_gtest(test_catching_cm_services test/test_catching_cm_services.cpp
    ENV ROS_DOMAIN_ID=52)
  target_link_libraries(test_catching_cm_services
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_catching_cm_services
    rclcpp
    rclcpp_lifecycle
    rtc_controller_manager
    rtc_controllers
    rtc_urdf_bridge
    rtc_base
    rtc_msgs
    sensor_msgs
    ament_index_cpp
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

  # ── Pull estimator + compliance wrench source, one binary ─────────────────
  #   test_pull_estimator_wiring         wiring (#167 P3).
  #   test_pull_estimator_known_wrench   shipped-profile estimator vs a known
  #                                      load (#177 crit#4): parses the shipped
  #                                      iiwa7_leap demo_shared.yaml so the
  #                                      deployed force_sign is what runs.
  #   test_pull_estimator_leakage_bound  shipped-profile leakage bound (#177
  #                                      crit#6 (1)): the regression the
  #                                      hardware runbook gates on, run against
  #                                      every shipped profile's own
  #                                      alignment_error_rad; the free-disk
  #                                      MuJoCo route cannot close this
  #                                      criterion (see the file banner).
  #   test_compliance_wrench_source      compliance wrench source adapter (#469
  #                                      S1), driven through the same known load.
  # The last three share pull_known_load_fixture.hpp.
  ament_add_gtest(test_pull_estimator_suite
    test/test_pull_estimator_wiring.cpp
    test/test_pull_estimator_known_wrench.cpp
    test/test_pull_estimator_leakage_bound.cpp
    test/test_compliance_wrench_source.cpp)
  ib_timeout_per_file(test_pull_estimator_suite)
  target_link_libraries(test_pull_estimator_suite
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  target_compile_definitions(test_pull_estimator_suite PRIVATE
    RTC_DEMO_SHARED_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  ament_target_dependencies(test_pull_estimator_suite
    rclcpp
    rtc_controllers
    rtc_base
    pinocchio
    Eigen3
  )

  # ── Closed-chain kinematics + unified kin&dyn cache equivalence, one binary
  # Reuses rtc_urdf_bridge/test fixtures (closure_test_fixtures.hpp,
  # test_urdf_path.hpp) — test-only, never-installed headers resolved via a
  # source-relative include dir (same pattern as rtc_controllers tests).
  #   test_closed_chain_hand_fk          closed-chain hand FK helper (#121
  #                                      Phase 2c).
  #   test_wbc_reduced_dynamics_provider #120: WbcReducedDynamicsProvider —
  #                                      coordinate order (name permutation),
  #                                      scatter values, held last-good,
  #                                      open-chain fallback.
  #   test_wbc_contact_kinematics_loop_consistent
  #                                      Phase 3: FillReducedFrameKinematics
  #                                      raises loop-downstream contact frame
  #                                      J/oMf to loop-consistent values
  #                                      (matching the non-RT converged handle,
  #                                      differing from frozen-loop), with L1
  #                                      last-good hold / L2-zero dJv / serial
  #                                      byte-for-byte.
  #   test_wbc_contact_permutation_real  Phase ③ scatter permutation on the REAL
  #                                      ur5e_p1b closure (non-identity
  #                                      cache_v_idx_ column map, which the
  #                                      CrankRocker nv==1 cases cannot reach).
  # The cache-equivalence trio proves each controller's move off arm_handle_
  # onto the combined-model PinocchioCache is value-preserving, on both the
  # serial (iiwa7_leap) and closed-chain (ur5e_p1b) profiles:
  #   test_wbc_arm_tcp_cache_equivalence Phase 2: the WBC clik_tcp/clik_base
  #                                      cache oMf equals the arm-only FK it
  #                                      replaces in FillLog/FillPublish/
  #                                      ComputeTcpError.
  #   test_wbc_fallback_omf_freshness    Task B (option B): cache.Update moved to
  #                                      ComputeControl, so arm/contact oMf
  #                                      refresh every non-E-STOP tick
  #                                      (kFallback/non-TSID included) — they
  #                                      TRACK measured across consecutive
  #                                      Updates and equal the FK each tick
  #                                      (1e-12).
  #   test_task_arm_cache_equivalence    Phase 4: DemoTaskController FK oMf AND
  #                                      the arm-column extraction of the
  #                                      combined Jacobian.
  #   test_joint_arm_cache_equivalence   Phase 5: DemoJointController arm TCP
  #                                      oMf (FK-only — no Jacobian consumer).
  ament_add_gtest(test_unified_kindyn_suite
    test/test_closed_chain_hand_fk.cpp
    test/test_wbc_reduced_dynamics_provider.cpp
    test/test_wbc_contact_kinematics_loop_consistent.cpp
    test/test_wbc_contact_permutation_real.cpp
    test/test_wbc_arm_tcp_cache_equivalence.cpp
    test/test_wbc_fallback_omf_freshness.cpp
    test/test_task_arm_cache_equivalence.cpp
    test/test_joint_arm_cache_equivalence.cpp)
  ib_timeout_per_file(test_unified_kindyn_suite)
  target_include_directories(test_unified_kindyn_suite PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_urdf_bridge/test")
  target_link_libraries(test_unified_kindyn_suite
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_unified_kindyn_suite
    ament_index_cpp
    rclcpp
    rtc_urdf_bridge
    rtc_tsid
    rtc_base
    pinocchio
    Eigen3
  )

  # dynamic_catching S2.3a: shipped urdf.extra_frames.catch_frame on the real
  # robot models (placement in every derived model, palm-normal sign, nv).
  ament_add_gtest(test_catch_frame_models test/test_catch_frame_models.cpp)
  target_compile_definitions(test_catch_frame_models PRIVATE
    RTC_BRINGUP_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  ament_target_dependencies(test_catch_frame_models
    ament_index_cpp
    rtc_urdf_bridge
    pinocchio
    Eigen3
    yaml-cpp
  )
  target_link_libraries(test_catch_frame_models yaml-cpp)

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

  # The frame NAMES DemoJointController's on_configure registers (TF slot table
  # + vector-payload frame id), pinned for the two robots a test can load. The
  # slot table is built only when the TF publisher exists, so this runs
  # on_configure on a real LifecycleNode and takes the package ROS_DOMAIN_ID.
  ament_add_gtest(test_joint_tf_slot_frames
    test/test_joint_tf_slot_frames.cpp
    TIMEOUT 120
    ENV ROS_DOMAIN_ID=52)
  target_link_libraries(test_joint_tf_slot_frames
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_joint_tf_slot_frames
    ament_index_cpp
    rclcpp
    rclcpp_lifecycle
    rtc_urdf_bridge
    rtc_controllers
    rtc_controller_interface
    rtc_base
    pinocchio
    Eigen3
  )

  # DemoJointController on a primary group that is a kinematic tree (waist +
  # two arms as one device, hand mounted on one branch). Runs on a synthetic
  # fixture owned by this repository — rtc_urdf_bridge/test/urdf/
  # dual_arm_tree_hand.urdf, reached through the never-installed
  # test_urdf_path.hpp (same source-relative include as the suite above).
  # Two cases run on_configure on a real LifecycleNode, hence the domain.
  ament_add_gtest(test_joint_tree_primary_group
    test/test_joint_tree_primary_group.cpp
    TIMEOUT 120
    ENV ROS_DOMAIN_ID=52)
  target_include_directories(test_joint_tree_primary_group PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_urdf_bridge/test")
  target_link_libraries(test_joint_tree_primary_group
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_joint_tree_primary_group
    ament_index_cpp
    rclcpp
    rclcpp_lifecycle
    rtc_urdf_bridge
    rtc_controllers
    rtc_controller_interface
    rtc_base
    pinocchio
    Eigen3
  )

  # DemoDualArmController (#637): several task frames of one device group in
  # one QP. Runs on a synthetic fixture owned by this repository —
  # rtc_urdf_bridge/test/urdf/dual_arm_trunk_hand.urdf (a trunk with two arms
  # as one group, a hand on one of them), reached through the never-installed
  # test_urdf_path.hpp. The lifecycle case runs on_configure on a real
  # LifecycleNode, hence the domain.
  ament_add_gtest(test_demo_dualarm_controller
    test/test_demo_dualarm_controller.cpp
    TIMEOUT 300
    ENV ROS_DOMAIN_ID=52)
  target_include_directories(test_demo_dualarm_controller PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/test"
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_urdf_bridge/test")
  target_link_libraries(test_demo_dualarm_controller
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_demo_dualarm_controller
    ament_index_cpp
    rclcpp
    rclcpp_lifecycle
    rtc_urdf_bridge
    rtc_controllers
    rtc_controller_interface
    rtc_tsid
    rtc_base
    pinocchio
    Eigen3
  )

  # DemoDualArmController's allocation gate (RT-1). Own binary: it defines the
  # global operator new (alloc_gate.hpp) AND the C malloc family (rtc_base's
  # malloc_gate.hpp) — each is one TU per executable. No node, so no domain.
  ament_add_gtest(test_demo_dualarm_alloc test/test_demo_dualarm_alloc.cpp
    TIMEOUT 120)
  target_include_directories(test_demo_dualarm_alloc PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/test"
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_urdf_bridge/test"
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_controllers/test/include")
  rtc_base_add_test_includes(test_demo_dualarm_alloc)
  target_link_libraries(test_demo_dualarm_alloc
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_demo_dualarm_alloc
    ament_index_cpp
    rclcpp
    rclcpp_lifecycle
    rtc_urdf_bridge
    rtc_controllers
    rtc_controller_interface
    rtc_tsid
    rtc_base
    pinocchio
    Eigen3
  )
  # alloc_gate.hpp pairs new→malloc with delete→free, which makes GCC
  # false-positive -Wmismatched-new-delete program-wide (alloc_gate.hpp item 2).
  target_compile_options(test_demo_dualarm_alloc PRIVATE -Wno-mismatched-new-delete)

  # Hand fingertip FK wiring (#685): the device's joint order on the serial
  # hand handle and the constant arm tip → hand root mount, as
  # support/hand_fk_wiring resolves them (synthetic fixture, reached through the
  # never-installed test_urdf_path.hpp) and as the four controllers that publish
  # fingertip poses use and refuse them (iiwa7_leap). The controller cases run
  # on_configure on a real LifecycleNode, hence the domain.
  ament_add_gtest(test_hand_fk_wiring
    test/test_hand_fk_wiring.cpp
    TIMEOUT 180
    ENV ROS_DOMAIN_ID=52)
  target_include_directories(test_hand_fk_wiring PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_urdf_bridge/test")
  target_link_libraries(test_hand_fk_wiring
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_hand_fk_wiring
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

  # An arm model whose tip frame did not resolve refuses the configure (#688):
  # the verdict's wording (support/arm_tip_resolution.hpp) and the four
  # controllers that report an arm tip pose, on iiwa7_leap, in both bring-up
  # orders. on_configure runs on a real LifecycleNode, hence the domain.
  ament_add_gtest(test_arm_tip_resolution
    test/test_arm_tip_resolution.cpp
    TIMEOUT 180
    ENV ROS_DOMAIN_ID=52)
  target_link_libraries(test_arm_tip_resolution
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_arm_tip_resolution
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

  # FindTreeModel (support/model_config_lookup.hpp): the one lookup every demo
  # controller makes for a group's tree model. Header-only, no node.
  ament_add_gtest(test_model_config_lookup
    test/test_model_config_lookup.cpp)
  target_include_directories(test_model_config_lookup PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include")
  ament_target_dependencies(test_model_config_lookup
    rtc_urdf_bridge
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

  # ── Vision ingress decode (S5.2, G1-A / G1-B) ─────────────────────────────
  # Owns the wire format: the ball_perception layout is reproduced in the
  # fixture on purpose, so this suite is what fails when the publisher moves.
  # The judgement half (order, staleness, the two time axes) is rtc_controllers'
  # test_catching_traj_ingress — no ROS, no message.
  ament_add_gtest(test_catching_traj_input test/test_catching_traj_input.cpp
    ENV ROS_DOMAIN_ID=52)
  target_link_libraries(test_catching_traj_input
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_catching_traj_input
    rtc_controllers
    rtc_base
    sensor_msgs
  )

  # ── Catching tracking law, closed loop on the real model (S5.3) ───────────
  # G4-H / G5-A / G5-B. Drives the shipped ur5e_p1b model with a perfect servo
  # and judges the result with a SECOND pinocchio model, so no assertion here
  # is the controller agreeing with itself. Sleeps one control period per tick
  # (the controller reads the steady clock, L0 §4.5), so it runs in about a
  # second rather than instantly.
  ament_add_gtest(test_catching_tracking test/test_catching_tracking.cpp
    ENV ROS_DOMAIN_ID=52)
  target_link_libraries(test_catching_tracking
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_catching_tracking
    rclcpp
    rclcpp_lifecycle
    rtc_controllers
    rtc_urdf_bridge
    rtc_base
    sensor_msgs
    ament_index_cpp
  )

  # ── dynamic_catching S7: the supervisor scenario table (L7 §9, G7-A/B/G) ──
  # The whole trial cycle closed loop on the real model: each case asserts the
  # mode sequence and the reason/outcome that names its scenario — normal,
  # homing, missed, the stale/saturation/QP aborts, E-STOP, fault reset, and
  # the S7 additions (disarm mid-return, stale across t_cmd..t_c, hand timeout
  # in HOLD, Q14 contact-then-abort, T_arm close timing, two trials, re-throw,
  # stale fingertip). Real-time ticks, ~20 s.
  ament_add_gtest(test_catching_supervisor_scenarios test/test_catching_supervisor_scenarios.cpp
    TIMEOUT 120
    ENV ROS_DOMAIN_ID=52)
  target_link_libraries(test_catching_supervisor_scenarios
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_catching_supervisor_scenarios
    rclcpp
    rclcpp_lifecycle
    rtc_controllers
    rtc_urdf_bridge
    rtc_base
    sensor_msgs
    ament_index_cpp
  )

  # ── dynamic_catching S6-A: the planner thread and the RT plan lane ────────
  # Thread half: eventfd coalescing, the reset drain, Join under a long
  # timeout, Pause. Lane half on the real UR5e+P1b model through CM's configure
  # path: the oracle reaching the law through the plan box, PlannerRtState
  # every tick, a previous-activation plan and a pre-E-STOP plan never
  # consumed (G3-L), and the stub planner leaving TRACKING on NO_CATCHABLE_PLAN.
  ament_add_gtest(test_catching_planner_lane test/test_catching_planner_lane.cpp
    ENV ROS_DOMAIN_ID=52)
  target_link_libraries(test_catching_planner_lane
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_catching_planner_lane
    rclcpp
    rclcpp_lifecycle
    rtc_controllers
    rtc_urdf_bridge
    rtc_base
    sensor_msgs
    ament_index_cpp
  )

  # ── dynamic_catching S6-C G3-C: 1000 synthetic throws through the planner ──
  # The shipped ur5e_p1b planner profile on the real catch sub-model: p99 of
  # the planning cycle < planner.search.grid.budget_s (asserted), rejection distribution and
  # rank-gate counts recorded.
  ament_add_gtest(test_catching_planner_g3c test/test_catching_planner_g3c.cpp
    TIMEOUT 300)
  target_include_directories(test_catching_planner_g3c PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/test")
  target_compile_definitions(test_catching_planner_g3c PRIVATE
    RTC_DEMO_SHARED_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  target_link_libraries(test_catching_planner_g3c
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_catching_planner_g3c
    rclcpp
    rtc_controllers
    rtc_urdf_bridge
    rtc_base
    ament_index_cpp
  )

  # ── dynamic_catching E1-F13 (#739): the mpc_docking core on the shipped
  # hand-locked sub-models ────────────────────────────────────────────────────
  # rtc_controllers solves on the plain arm URDFs; the catch sub-models are
  # xacro and only this package builds them. Recorded (no verdict): short-lead
  # grids and a real throw on `ur5e_catch` and `iiwa7_catch`, with the shipped
  # catch frame and joint ratings read from the shipped YAML. The throw
  # generator and the tally are rtc_controllers' test-only fixture, pulled in
  # by source-tree path like the other shared fixtures above.
  ament_add_gtest(test_catching_docking_core_shipped
    test/test_catching_docking_core_shipped.cpp TIMEOUT 300)
  target_include_directories(test_catching_docking_core_shipped PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/test"
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_controllers/test/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_urdf_bridge/test")
  target_compile_definitions(test_catching_docking_core_shipped PRIVATE
    RTC_DEMO_SHARED_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
    RTC_TEST_ROBOT_DESCRIPTIONS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots")
  target_link_libraries(test_catching_docking_core_shipped
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_catching_docking_core_shipped
    rclcpp
    rtc_controllers
    rtc_urdf_bridge
    rtc_base
    ament_index_cpp
  )

  # ── dynamic_catching E1-F14 (#740): the NLP catch search on the shipped
  # hand-locked sub-models ────────────────────────────────────────────────────
  # What a wake costs on `ur5e_catch` and `iiwa7_catch`, against the real
  # clock. Recorded (no verdict): Configure's time and memory, a candidate's
  # screening, a cold and a warm solve by node count, a whole wake by the
  # number of solves, and how far a solve runs past a short share. The models
  # come from the same loader as the docking core's suite above.
  ament_add_gtest(test_catching_nlp_search_shipped
    test/test_catching_nlp_search_shipped.cpp TIMEOUT 600)
  target_include_directories(test_catching_nlp_search_shipped PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/test"
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_controllers/test/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../rtc_urdf_bridge/test")
  target_compile_definitions(test_catching_nlp_search_shipped PRIVATE
    RTC_DEMO_SHARED_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
    RTC_TEST_ROBOT_DESCRIPTIONS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../robot_descriptions/robots")
  target_link_libraries(test_catching_nlp_search_shipped
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  ament_target_dependencies(test_catching_nlp_search_shipped
    rclcpp
    rtc_controllers
    rtc_urdf_bridge
    rtc_base
    ament_index_cpp
  )

  # ── dynamic_catching S6-A R-1: the planner shares the `mpc` layout role ────
  # DemoWbc's MPC thread and the catching planner in one process, switched in
  # CM's order (deactivate, activate): thread count by name, affinity against
  # the role's slot, Paused/Running per thread, policy when the host allows it
  # (NOT_EVALUATED(EPERM) recorded otherwise), and the mpc_off refusal.
  ament_add_gtest(test_catching_mpc_role_switch test/test_catching_mpc_role_switch.cpp
    ENV ROS_DOMAIN_ID=52)
  target_link_libraries(test_catching_mpc_role_switch
    ${PROJECT_NAME}_demo_controllers
    yaml-cpp
  )
  target_compile_definitions(test_catching_mpc_role_switch PRIVATE
    RTC_UR5E_URDF_PATH="${RTC_UR5E_URDF_PATH}"
    RTC_WBC_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/ur5e_p1a/controllers/demo_wbc_controller.yaml")
  ament_target_dependencies(test_catching_mpc_role_switch
    rclcpp
    rclcpp_lifecycle
    rtc_controllers
    rtc_base
    pinocchio
    rtc_tsid
    rtc_mpc
    Eigen3
  )

  # ── QP CLIK task-space characterisation across the ball envelope ─────────
  # 36 closed-loop cases (speed x descent angle x azimuth) on the SHIPPED
  # reference gains, writing clik_sweep.csv. Its assertions are about solver
  # HEALTH only — convergence and the solve budget, which must hold at every
  # speed; the position and axis errors are the measurement and are read from
  # the CSV rather than pinned here. Own domain: it runs 36 activations and a
  # participant collision with the tracking suite would show up as a lane that
  # goes quiet mid-sweep. ~2 min (one control period of sleep per tick).
  ament_add_gtest(test_catching_clik_sweep test/test_catching_clik_sweep.cpp
    ENV ROS_DOMAIN_ID=53
    TIMEOUT 600)
  target_link_libraries(test_catching_clik_sweep
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_catching_clik_sweep
    rclcpp
    rclcpp_lifecycle
    rtc_controllers
    rtc_urdf_bridge
    rtc_base
    sensor_msgs
    ament_index_cpp
  )

  # ── UdpHandNativeBackend sensor lane test ─────────────────────────────────
  # The other producer of the fingertip lane's per-group receipt time and
  # sample counter (dynamic_catching D-24 (a)), plus the inference-block decode
  # those assertions stand on. Same domain as its mujoco counterpart.
  ament_add_gtest(test_udp_hand_native_backend test/test_udp_hand_native_backend.cpp
    ENV ROS_DOMAIN_ID=52)
  target_link_libraries(test_udp_hand_native_backend
    ${PROJECT_NAME}_demo_controllers
  )
  ament_target_dependencies(test_udp_hand_native_backend
    rclcpp
    rclcpp_lifecycle
    rtc_controller_manager
    rtc_base
    rtc_msgs
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

  # Shipped mujoco_simulator.yaml files, through the same loader a node's
  # constructor reaches — the only gate that sees the constructs which abort a
  # node before on_configure (empty sequences, integer-literal double arrays).
  ament_add_gtest(test_shipped_sim_config test/test_shipped_sim_config.cpp)
  target_compile_definitions(test_shipped_sim_config PRIVATE
    RTC_DEMO_SHARED_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  ament_target_dependencies(test_shipped_sim_config rclcpp)

  # ── demo_controller_gui static robot profiles (pure-Python) ────────────────
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_robot_profiles test/test_robot_profiles.py)
  ament_add_pytest_test(test_cm_rt_params_wiring test/test_cm_rt_params_wiring.py)
  # Shipped initial_qpos: length/type (always) + MJCF keyframe agreement (skips
  # without the mujoco pip module, which CI installs and /usr/bin/python3 lacks).
  ament_add_pytest_test(test_shipped_initial_qpos test/test_shipped_initial_qpos.py)
  # Shipped catching.io.base_T_world vs where the shipped scene puts the arm
  # base: float types (always) + the compiled scene's base position (skips
  # without the mujoco pip module, like the keyframe lane above).
  ament_add_pytest_test(test_shipped_base_T_world test/test_shipped_base_T_world.py)
  # Shipped mujoco_simulator.yaml keys vs the node's declare_parameter list. An
  # undeclared key is read as its compiled default with no error anywhere, so
  # this is the cheap stand-in for the runtime launch that would otherwise be
  # the only way to notice.
  ament_add_pytest_test(test_shipped_sim_param_declared
    test/test_shipped_sim_param_declared.py)
  ament_add_pytest_test(test_launch_shield_wiring test/test_launch_shield_wiring.py)
  ament_add_pytest_test(test_launch_hand_affinity_wiring
    test/test_launch_hand_affinity_wiring.py)
  # Behavioural twin of the two AST tests above: evaluates every launch
  # description instead of reading it.
  ament_add_pytest_test(test_sim_overlay test/test_sim_overlay.py)
  # S8-B catch_lead_* overlays: a controller key one level off is created,
  # never read, and the run is silently the shipped profile.
  ament_add_pytest_test(test_catch_lead_overlays test/test_catch_lead_overlays.py)
  # The catching config in four files (MD-90): which key lives where,
  # and the Python loader against the CM's — the dump is the CM's loader as a
  # command the pytest can run.
  add_executable(controller_config_dump test/controller_config_dump.cpp)
  ament_target_dependencies(controller_config_dump rtc_controller_manager)
  target_link_libraries(controller_config_dump yaml-cpp)
  ament_add_pytest_test(test_shipped_catching_config
    test/test_shipped_catching_config.py
    ENV RTC_CONTROLLER_CONFIG_DUMP=$<TARGET_FILE:controller_config_dump>)
  ament_add_pytest_test(test_catching_sim_trials test/test_catching_sim_trials.py)
  ament_add_pytest_test(test_catching_sim_trials_s8 test/test_catching_sim_trials_s8.py)
  ament_add_pytest_test(test_catching_sim_trials_s8f test/test_catching_sim_trials_s8f.py)
  # #601 host load watch: a slow sim is found from the truth rows, the run
  # aborts / warns / stays silent by mode, the runner's own pytest is not load.
  ament_add_pytest_test(test_catching_sim_trials_host_watch
    test/test_catching_sim_trials_host_watch.py)
  ament_add_pytest_test(test_launch_description_evaluates
    test/test_launch_description_evaluates.py)
  ament_add_pytest_test(test_launch_session_wiring
    test/test_launch_session_wiring.py)
  ament_add_pytest_test(test_demo_gui_catalog test/test_demo_gui_catalog.py)
  ament_add_pytest_test(test_demo_gui_presets test/test_demo_gui_presets.py)
  ament_add_pytest_test(test_demo_gui_preset_toggle test/test_demo_gui_preset_toggle.py)
  ament_add_pytest_test(test_demo_gui_pull test/test_demo_gui_pull.py)
  ament_add_pytest_test(test_demo_gui_task_frame test/test_demo_gui_task_frame.py)
  ament_add_pytest_test(test_demo_gui_ball_launch test/test_demo_gui_ball_launch.py)
  ament_add_pytest_test(test_demo_gui_grasp_mode test/test_demo_gui_grasp_mode.py)
  ament_add_pytest_test(test_demo_gui_hand_step test/test_demo_gui_hand_step.py)
  ament_add_pytest_test(test_demo_gui_catching test/test_demo_gui_catching.py)
  ament_add_pytest_test(test_demo_gui_latch_clear test/test_demo_gui_latch_clear.py)
  ament_add_pytest_test(test_demo_gui_dualarm test/test_demo_gui_dualarm.py)
  # S7 G8-A2 (existence half): every RT-owned member of the catching
  # controller has a row in its header's reset table.
  ament_add_pytest_test(test_catching_reset_table test/test_catching_reset_table.py)
  # tools/catching_eval/summarize.py (not installed — read from the source
  # tree): the mechanical parts of the G-1 verdict, on planted inputs.
  ament_add_pytest_test(test_catching_eval_summarize test/test_catching_eval_summarize.py)
  # tools/catching_eval/unit_report.py (same: read from the source tree): one
  # unit's report on a planted unit — a solve cut at the core's deadline is
  # counted, never given a solve time.
  ament_add_pytest_test(test_catching_eval_unit_report test/test_catching_eval_unit_report.py)
  # The eval tools and the trial runner load a catching profile or a recorded
  # mirror: an old key is refused, an old mirror name is read through the alias
  # table (#711).
  ament_add_pytest_test(test_catching_keys_tools test/test_catching_keys_tools.py)
  # tools/docking_ident (not installed — read from the source tree, E1-F15): the
  # rig's inputs out of the shipped profiles, its copy of the simulator's ball
  # against rtc_mujoco_sim's C++, and report.py on planted stores. The rig
  # itself needs mujoco: test_docking_ident_rig.py skips under colcon and is
  # run by hand with the workspace venv (its header).
  ament_add_pytest_test(test_docking_ident_config test/test_docking_ident_config.py)
  ament_add_pytest_test(test_docking_ident_rig test/test_docking_ident_rig.py)

  # ── Parallel ctest ─────────────────────────────────────────────────────────
  # colcon.pkg runs this package's tests four at a time. Two kinds of test
  # cannot simply overlap:
  #
  # (1) A test that claims a ROS_DOMAIN_ID. The claim is per PACKAGE -- it keeps
  #     this package's participants off the bus of the packages colcon tests
  #     beside it, and says nothing about two tests of this package. That was
  #     safe only because ctest ran them one at a time; overlapped, they share
  #     the discovery bus and the Fast DDS shared-memory port, which is the hang
  #     of #401. So every claimant holds the lock of its domain and ctest never
  #     runs two of them at once. validate_test_domains.py refuses a claimant
  #     of a parallel package that is missing here.
  set_tests_properties(
    test_registered_controllers_have_shipped_config
    test_demo_inference_alloc
    test_demo_inference_controller
    test_demo_inference_urdf
    test_demo_inference_real_model
    test_compliance_admittance_coupling
    test_demo_catching_controller
    test_demo_catching_alloc_s7
    test_catching_cm_services
    test_demo_wbc_tsid_path
    test_wbc_closed_chain_projection_sharing
    test_momentum_observer_embedding
    test_joint_tf_slot_frames
    test_joint_tree_primary_group
    test_demo_dualarm_controller
    test_hand_fk_wiring
    test_arm_tip_resolution
    test_mujoco_native_backend
    test_catching_traj_input
    test_catching_tracking
    test_catching_supervisor_scenarios
    test_catching_planner_lane
    test_catching_mpc_role_switch
    test_udp_hand_native_backend
    test_controller_target_cb_group_invariant
    test_payload_estimate_topic
    PROPERTIES RESOURCE_LOCK ros_domain_52)
  set_tests_properties(test_catching_clik_sweep PROPERTIES RESOURCE_LOCK ros_domain_53)
  # (2) A test that ASSERTS a measured wall-clock budget: a planner p99
  #     (planner_g3c), a QP solve-time p99 / maximum (clik_sweep, tracking), a
  #     join that must return within 200 ms (planner_lane), a phase that must
  #     end within 10-20 ms of its deadline on the real clock
  #     (supervisor_scenarios, the *RealClock fixtures). It must not share the
  #     cores with its siblings; RUN_SERIAL gives it the box it had while ctest
  #     ran one test at a time. The lock above does not do that: it keeps the
  #     other domain claimants away, not the tests that claim nothing.
  #     No gate can find these -- a reviewer found two this list had missed --
  #     so a new test that compares a MEASURED duration with a bound goes here.
  #     Durations that are only recorded are not protected: quote those from a
  #     run of that one test.
  set_tests_properties(
    test_catching_planner_g3c
    test_catching_clik_sweep
    test_catching_planner_lane
    test_catching_tracking
    test_catching_supervisor_scenarios
    PROPERTIES RUN_SERIAL TRUE)

  # Every gtest binary logs into its own throwaway session directory
  # (test/session_dir_test_env.cpp). A controller configured from a shipped YAML
  # opens CSV logs at on_configure, and without RTC_SESSION_DIR they land in the
  # workspace's real logging_data. Linked here, after every test target exists,
  # so a new binary gets it without asking. Keep this block last.
  get_directory_property(_ib_targets BUILDSYSTEM_TARGETS)
  foreach(_ib_target IN LISTS _ib_targets)
    if(_ib_target MATCHES "^test_")
      get_target_property(_ib_type ${_ib_target} TYPE)
      if(_ib_type STREQUAL "EXECUTABLE")
        target_sources(${_ib_target} PRIVATE test/session_dir_test_env.cpp)
      endif()
    endif()
  endforeach()
endif()

ament_package()
