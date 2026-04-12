# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Identity

**RTC (Real-Time Control) Framework** — A robot-agnostic real-time control framework for URDF-based manipulators.

The `rtc_*` packages are **robot-agnostic**: variable-DOF, configurable control rate (500Hz–2kHz), transport abstraction (UDP/CAN-FD/EtherCAT/RS485), lock-free SPSC architecture, and E-STOP safety system. They can be applied to any URDF manipulator without modification.

The `ur5e_*` packages are **robot-specific**: UR5e + 10-DOF hand hardware drivers, launch files, demo controllers, and BehaviorTree coordinator. These serve as a reference integration built on top of the `rtc_*` framework.

| Item | Value |
|------|-------|
| Lang | C++20 (GCC 11+/13+), Python 3.10+ |
| OS | Ubuntu 22.04 (PREEMPT_RT optional) / 24.04 |
| Middleware | ROS 2 Humble / Jazzy, CycloneDDS |
| Build | CMake 3.22+, colcon, ament_cmake / ament_python |
| Framework Deps | Eigen 3.4, Pinocchio, ONNX Runtime |
| Optional Deps | MuJoCo 3.x (simulation), BehaviorTree.CPP v4 (coordination), ProxSuite (TSID QP solver) |
| Robot-Specific HW | UR5e (ros2_control RTDE), 10-DOF Hand (UDP), STM32 ToF (UART) |
| Test | GTest, pytest (41 tests across 9 packages) |
| CI | GitHub Actions (`ros2-advanced-ci.yml`), Codecov |
| Compiler flags | `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion` |

---

## Build & Run

**Prerequisites**: Ubuntu 22.04 (ROS2 Humble) or 24.04 (ROS2 Jazzy), `realtime` group with `rtprio 99` / `memlock unlimited`.

```bash
# Build
./build.sh sim            # simulation packages
./build.sh robot          # real robot (excludes MuJoCo)
./build.sh full           # all packages
./build.sh -p rtc_base    # single package

# Run
ros2 launch ur5e_bringup sim.launch.py                                        # MuJoCo sim
ros2 launch ur5e_bringup robot.launch.py robot_ip:=192.168.1.10              # real robot
ros2 launch ur5e_hand_driver hand_udp.launch.py target_ip:=192.168.1.2      # hand only

# Monitor
ros2 topic hz /forward_position_controller/commands   # ~500Hz
ros2 topic echo /system/estop_status                  # true = E-STOP active
```

---

## Repository Structure

19 ROS2 packages at repo root (no `src/`). 13 `rtc_*` (robot-agnostic) + 2 `shape_estimation_*` + 4 `ur5e_*` (robot-specific). Each has its own `README.md` with detailed API and configuration.

| Package | Type | Key Content |
|---------|------|-------------|
| `rtc_base` | Header-only | Types, SeqLock, SPSC buffers, threading (4/6/8/10/12/16-core), Bessel/Kalman filters, DataLogger |
| `rtc_communication` | Header-only | `TransportInterface`, `UdpSocket` RAII, `PacketCodec` concept, `Transceiver<T,C>` |
| `rtc_controller_interface` | Library | `RTControllerInterface` abstract base, `ControllerRegistry` singleton, `RTC_REGISTER_CONTROLLER` macro |
| `rtc_controllers` | Library | PController, JointPDController, ClikController, OSC, GraspController (adaptive PI force) |
| `rtc_controller_manager` | Executable | `RtControllerNode`: 500Hz RT loop, SPSC publish offload, CSV logging, E-STOP, digital twin |
| `rtc_inference` | Header-only | `InferenceEngine` abstract, `OnnxEngine` (IoBinding, pre-allocated buffers) |
| `rtc_msgs` | Messages | JointCommand, HandSensorState, GraspState, RobotTarget, SimSensor/SimSensorState, CalibrationCommand/Status, etc. |
| `rtc_mujoco_sim` | Executable | MuJoCo 3.x wrapper: sync-step, GLFW viewer, multi-group, position servo |
| `rtc_tools` | Python | controller_gui, plot_rtc_log, compare_mjcf_urdf, urdf_to_mjcf |
| `rtc_scripts` | Shell | PREEMPT_RT build, CPU shield, IRQ affinity, UDP optimization |
| `rtc_digital_twin` | Python | RViz2 multi-source JointState merge, URDF mimic auto-compute |
| `rtc_urdf_bridge` | Library | Robot-agnostic URDF parser + Pinocchio model builder |
| `rtc_tsid` | Library | TSID QP framework: WQP/HQP formulations, PostureTask, EOM/Contact/FrictionCone/TorqueLimit constraints, ProxSuite solver |
| `shape_estimation` | Executable | ToF-based voxel point cloud + primitive fitting |
| `ur5e_description` | Data | URDF + MJCF + meshes (DAE/STL/OBJ) |
| `ur5e_hand_driver` | Executable | UDP event-driven driver, SeqLock state, ONNX F/T inference |
| `ur5e_bt_coordinator` | Executable | BehaviorTree.CPP v4 task coordinator (20 Hz) |
| `ur5e_bringup` | Launch/Config | Launch files + DemoJoint/DemoTask controllers + GUI tools |

### Dependency Graph

```
rtc_msgs, rtc_base (independent)
  +-- rtc_communication, rtc_inference <-- rtc_base
  +-- rtc_controller_interface <-- rtc_base, rtc_msgs, rtc_urdf_bridge
  |     +-- rtc_controllers <-- rtc_controller_interface, rtc_urdf_bridge
  |           +-- rtc_controller_manager <-- rtc_controllers, rtc_communication
  +-- rtc_tsid <-- Pinocchio, ProxSuite, Eigen3, yaml-cpp
  +-- rtc_mujoco_sim <-- MuJoCo 3.x (optional)
rtc_urdf_bridge <-- Pinocchio, tinyxml2, yaml-cpp
ur5e_hand_driver <-- rtc_communication, rtc_inference, rtc_base
ur5e_bringup <-- rtc_controller_manager, ur5e_hand_driver, ur5e_description
```

---

## Architecture

### Core Data Types (`rtc_base/types/types.hpp`)

Key constants: `kNumRobotJoints=6`, `kMaxDeviceChannels=64`, `kMaxSensorChannels=128`, `kMaxFingertips=8`, `kNumHandMotors=10`.

Key types (all trivially copyable, RT-safe):
- **DeviceState**: positions/velocities/efforts[64], motor_*/sensor_data/inference_data arrays
- **ControllerState**: devices[4], num_devices, dt, iteration
- **ControllerOutput**: devices[4], task positions, valid, command_type, grasp_state
- **GraspStateData**: force_magnitude/contact_flag/inference_valid[8], grasp_phase, finger_s/filtered_force/force_error[8], grasp_target_force

### Threading Model (6-core example)

| Thread | Core | Sched | Prio | Role |
|--------|------|-------|------|------|
| rt_loop | 2 | FIFO | 90 | 500Hz ControlLoop + 50Hz CheckTimeouts |
| sensor_executor | 3 | FIFO | 70 | JointState/MotorState/SensorState/Target subs |
| log_executor | 4 | OTHER | nice -5 | CSV logging (SPSC drain) |
| publish_thread | 5 | OTHER | nice -3 | SPSC -> ROS2 publish |
| udp_recv | 5 | FIFO | 65 | Hand UDP receiver |

Core 0-1: OS/DDS/IRQ. Auto-selects 4/6/8/10/12/16-core layouts via `SelectThreadConfigs()`.

### Lock-Free Rules

- **SeqLock<T>**: single-writer/multi-reader, requires `is_trivially_copyable_v<T>`
- **SpscLogBuffer/SpscPublishBuffer<512>**: wait-free push (drops on full), power-of-2
- **try_lock only** on RT path (never block), `lock_guard` on non-RT `SetDeviceTarget()`
- **jthread + stop_token** for cooperative cancellation
- **Separate mutexes**: `state_mutex_`, `target_mutex_`, `hand_mutex_` -- never hold more than one

---

## Controllers

| Controller | Type | Space | Key Feature |
|------------|------|-------|-------------|
| PController | Position | Joint | `q + kp*error*dt` incremental |
| JointPDController | Torque | Joint | PD + Pinocchio RNEA + quintic trajectory |
| ClikController | Position | Cartesian 3/6-DOF | Damped Jacobian pseudoinverse + null-space |
| OSC | Torque | Cartesian 6-DOF | Full pose PD + SE3 quintic trajectory |
| GraspController | Internal | Hand 3x3-DOF | Adaptive PI force, 6-state FSM, per-finger stiffness EMA |
| DemoJointController | Position | Joint + Hand | Quintic trajectory, `grasp_controller_type: "contact_stop"\|"force_pi"` |
| DemoTaskController | Position | Cartesian + Hand | CLIK + trajectory, `grasp_controller_type: "contact_stop"\|"force_pi"` |

### Gains Layout (via `~/controller_gains` topic)

| Controller | Layout | Count |
|------------|--------|-------|
| PController | `[kp x 6]` | 6 |
| JointPDController | `[kp x 6, kd x 6, gravity(0/1), coriolis(0/1), traj_speed]` | 15 |
| ClikController | `[kp_trans x 3, kp_rot x 3, damping, null_kp, null_space(0/1), 6dof(0/1), traj_speed, traj_ang_speed, max_vel, max_ang_vel]` | 14 |
| OSC | `[kp_pos x 3, kd_pos x 3, kp_rot x 3, kd_rot x 3, damping, gravity(0/1), traj_speed, traj_ang_speed, max_vel, max_ang_vel]` | 18 |
| DemoJoint | `[robot_traj_speed, hand_traj_speed, robot_max_vel, hand_max_vel, grasp_contact_thresh, grasp_force_thresh, grasp_min_fingertips, grasp_cmd(0/1/2), grasp_target_force]` | 9 |
| DemoTask | `[kp_trans x 3, kp_rot x 3, damping, null_kp, null_space(0/1), 6dof(0/1), traj_speed, traj_ang_speed, hand_traj_speed, max_vel, max_ang_vel, hand_max_vel, grasp_contact_thresh, grasp_force_thresh, grasp_min_fingertips, grasp_cmd(0/1/2), grasp_target_force]` | 21 |

### GraspController (Force-PI, internal only)

Selected via `grasp_controller_type: "force_pi"` in demo controller YAML (default: `"contact_stop"`).

**FSM**: Idle -> Approaching (position ramp) -> Contact (settle) -> ForceControl (PI + force ramp) -> Holding (anomaly monitor) -> Releasing

Key params in `grasp_types.hpp`: `Kp_base=0.02`, `Ki_base=0.002`, `f_target=2.0N`, `f_contact=0.2N`, `ds_max=0.05/s`, `delta_s_max=0.15` deformation guard, slip detection at `5.0 N/s`.

---

## RtControllerNode

- **ControlLoop** (500Hz): E-STOP check -> assemble ControllerState -> `Compute()` -> SPSC publish + log
- **CheckTimeouts** (50Hz): per-group device timeout -> `TriggerGlobalEstop("{group}_timeout")`
- **E-STOP triggers**: group timeout, init timeout, >= 10 consecutive RT overruns, sim sync timeout
- **TriggerGlobalEstop**: idempotent (`compare_exchange_strong`), propagates to all controllers

---

## ROS2 Topics (Key)

**Controller Manager** (`/{ns}/`): `controller_type` (Sub, switch), `controller_gains` (Sub), `active_controller_name` (Pub), `/system/estop_status` (Pub)

**Dynamic** (per controller TopicConfig): Subscribe `kState`/`kMotorState`/`kSensorState`/`kTarget`; Publish `kJointCommand`/`kRos2Command`/`kGuiPosition`/`kGraspState`/`kDeviceStateLog`/`kDeviceSensorLog`

**Digital Twin**: `/{group}/digital_twin/joint_states` (RELIABLE) -> `rtc_digital_twin` merges -> RViz2

**Hand Driver**: `/hand/joint_states`, `/hand/motor_states`, `/hand/sensor_states` (Pub); `/hand/joint_command` (Sub)

**MuJoCo**: `<group.state_topic>` (Pub), `<group.command_topic>` (Sub), `/sim/status` (1Hz)

---

## Configuration Files

Key config files (see each file for full parameter documentation):

| Config | Path | Key Parameters |
|--------|------|----------------|
| RT controller manager | `ur5e_bringup/config/ur5e_robot.yaml` / `ur5e_sim.yaml` | `control_rate`, `initial_controller`, `devices`, `urdf`, `device_timeout_*` |
| MuJoCo simulator | `rtc_mujoco_sim/config/mujoco_simulator.yaml` | `model_path`, `physics_timestep`, `n_substeps`, `robot_response.groups` |
| MuJoCo solver | `rtc_mujoco_sim/config/solver_param.yaml` | `solver` (Newton/CG/PGS), `cone`, `integrator`, `noslip_iterations`, `contact_override` |
| Hand UDP driver | `ur5e_hand_driver/config/hand_udp_node.yaml` | `target_ip`, `recv_timeout_ms`, `communication_mode` (bulk/individual) |
| Digital twin | `rtc_digital_twin/config/digital_twin.yaml` | `display_rate`, `num_sources`, `auto_compute_mimic` |
| Controller YAMLs | `rtc_controllers/config/controllers/{direct\|indirect}/*.yaml` | Per-controller gains + `topics:` section for device-group routing |

---

## Common Modification Patterns

### Adding a New Controller

1. Header in `rtc_controllers/include/rtc_controllers/{direct|indirect}/` -- inherit `RTControllerInterface`, implement `Compute()`, `SetDeviceTarget()`, `InitializeHoldPosition()`, `Name()` (all `noexcept`)
2. Source in `rtc_controllers/src/controllers/{direct|indirect}/` -- `LoadConfig()` for YAML, `UpdateGainsFromMsg()` for runtime gains
3. YAML in `rtc_controllers/config/controllers/` -- must include `topics:` section
4. Register via `RTC_REGISTER_CONTROLLER()` macro. Robot-specific controllers go in `ur5e_bringup/` with registration in `ur5e_bringup/src/controllers/controller_registration.cpp`

### Adding a New Message Type

1. Create `rtc_msgs/msg/MyMessage.msg`, add to `CMakeLists.txt` `rosidl_generate_interfaces()`
2. If used in publish offload: add `PublishRole` enum in `rtc_base/types/types.hpp` + YAML mapping in `rtc_controller_interface/src/rt_controller_interface.cpp`

### Adding a New Device Group

1. Add device config in YAML under `devices:` + timeout entry in `device_timeout_names`/`values`
2. Add topic routing in each controller's YAML `topics:` section
3. If kinematics needed: add `sub_models` or `tree_models` entry under `urdf:`
4. Handle new device index in controller `Compute()` / `SetDeviceTarget()`

### Adding a New Thread

1. Define `ThreadConfig` for all core tiers in `rtc_base/threading/thread_config.hpp`
2. Add to `SystemThreadConfigs`, update `ValidateSystemThreadConfigs()` + `SelectThreadConfigs()`
3. Call `ApplyThreadConfig()` at thread entry; use SCHED_FIFO for RT threads

### Updating an Existing Package

When modifying code in any package (bug fix, feature addition, refactoring, API change, dependency update, etc.), you **MUST** complete ALL of the following steps before considering the task done. Do not skip any step. **Code changes without corresponding documentation and metadata updates are considered incomplete and must not be committed.**

#### 1. Unit Tests -- Update & Run

- **Identify affected tests**: Find all test files related to the changed code. Use the test table in the [Testing](#testing) section and check `<package>/test/` directory.
- **Update existing tests**: If the change modifies public API, function signatures, behavior, or data types, update corresponding test cases to match. Ensure assertions reflect the new expected behavior.
- **Add new tests**: If the change introduces new functionality (new function, new class, new branch/state), add test cases that cover:
  - Normal/happy path
  - Edge cases and boundary conditions
  - Error handling paths (if applicable)
- **For C++ tests (GTest)**: Add test entries in `<package>/CMakeLists.txt` under `ament_add_gtest()` if new test files are created.
- **For Python tests (pytest)**: Ensure new test files follow the `test_*.py` naming convention and are discoverable by pytest.
- **Run tests and verify**:
  ```bash
  ./build.sh -p <package_name>
  colcon test --packages-select <package_name> --event-handlers console_direct+
  colcon test-result --verbose
  ```
- **All tests must pass** before proceeding. If a test failure is unrelated to the current change, note it explicitly but do not ignore it silently.

#### 2. CMakeLists.txt -- Verify & Update

- **Source files**: If new `.cpp` files were added or existing ones renamed/removed, update `add_library()` or `add_executable()` target source lists.
- **Header install**: If new public headers were added, verify they are included in `install(DIRECTORY include/ ...)`.
- **Dependencies**: If new `find_package()` or `ament_target_dependencies()` are needed, add them.
- **Test targets**: If new test files were added, add corresponding `ament_add_gtest()` or `ament_add_pytest_test()` entries.
- **Message generation**: For `rtc_msgs`, if `.msg`/`.srv`/`.action` files were added or removed, update `rosidl_generate_interfaces()`.
- **Ensure the package builds cleanly** with `./build.sh -p <package_name>` after CMakeLists.txt changes.

#### 3. package.xml -- Verify & Update

- **Dependencies**: Add appropriate tags (`<build_depend>`, `<exec_depend>`, `<depend>`, `<test_depend>`).
- **Version**: Bump `<version>` if the change is significant (new feature, breaking API change). Follow semantic versioning.
- **Description**: Update `<description>` if the package scope has changed.
- **Consistency**: Ensure `package.xml` dependencies match `CMakeLists.txt` `find_package()` calls -- they must be in sync.

#### 4. Config Files (YAML) -- Verify & Update

- **Parameter changes**: If code changes add, remove, or rename configurable parameters, update all relevant YAML config files:
  - Controller configs: `rtc_controllers/config/controllers/{direct|indirect}/*.yaml`
  - Robot/sim configs: `ur5e_bringup/config/ur5e_robot.yaml`, `ur5e_sim.yaml`
  - MuJoCo configs: `rtc_mujoco_sim/config/*.yaml`
  - Hand driver config: `ur5e_hand_driver/config/hand_udp_node.yaml`
  - Digital twin config: `rtc_digital_twin/config/digital_twin.yaml`
- **Default values**: If default parameter values changed in code, update the corresponding YAML values to remain consistent.
- **Comments**: Add or update inline YAML comments for new/changed parameters to document valid ranges and units.
- **Topic routing**: If topic names or device groups changed, update the `topics:` section in affected controller YAMLs.

#### 5. Documentation -- Verify & Update

Every package modification **MUST** include corresponding documentation updates. Documentation that is out of sync with code is treated as a bug.

- **Package `README.md`** (mandatory for every code change):
  - Update public API documentation (functions, classes, parameters, return types)
  - Update usage examples if behavior changed
  - Update configuration parameter descriptions (name, type, default, valid range, units)
  - Update dependency information if new dependencies were added
  - If the package README contains architecture diagrams or data flow descriptions, update them

- **Cross-package documentation** (when the change affects behavior beyond the package boundary):
  - This `CLAUDE.md` file: update architecture, data flow, tables, key file locations, controller gains layout, threading model, or any other section that references the changed code
  - Supplementary docs in `docs/` if applicable: `RT_OPTIMIZATION.md`, `SHELL_SCRIPTS.md`, `VSCODE_DEBUGGING.md`
  - Root `README.md`: update if the change adds new packages, new features, or changes the architecture diagram

- **Inline code documentation**:
  - Add or update Doxygen-style comments (`///` or `/** */`) for new or changed public headers
  - Update comments in YAML config files to reflect valid ranges, units, and parameter descriptions
  - If a function's contract, preconditions, or thread-safety guarantees change, update the corresponding header comment

#### 6. Final Verification

After all updates are complete, perform a final check:
```bash
./build.sh -p <package_name>
colcon test --packages-select <package_name> [<dependent_packages>...] --event-handlers console_direct+
colcon test-result --verbose
```

If the change touches `rtc_base` or `rtc_msgs` (widely depended-upon packages), build and test all downstream packages per the dependency graph above.

---

## Data Flow

```
[Robot HW / MuJoCo Sim] --JointState--> [RtControllerNode: 500Hz RT loop]
    +--SPSC--> [publish_thread] --> /forward_position_controller/commands
    |                           +-> /{group}/digital_twin/joint_states
    +--SPSC--> [log_executor]   --> CSV (timing + per-device state + sensor)
    +--E-STOP--> /system/estop_status

[Hand HW] <--UDP--> [ur5e_hand_driver] <--SeqLock--> [ControlLoop]
[rtc_digital_twin]: merge digital_twin topics --> RViz2
[ur5e_bt_coordinator]: subscribes grasp_state/gui_position, publishes goals/gains
```

Session logs: `logging_data/YYMMDD_HHMM/{controller,monitor,device,sim,plots,motions}/`

Session/logging root resolution (4-tier chain, shared between `rtc_base/logging/session_dir.hpp` and `rtc_tools.utils.session_dir`):
1. `$RTC_SESSION_DIR` -> `$UR5E_SESSION_DIR` (legacy fallback) -- used as-is if set.
2. `$COLCON_PREFIX_PATH` first entry's parent + `/logging_data` (requires write access).
3. Walk up from `cwd` looking for `install/` + `src/` siblings -> that dir + `/logging_data`.
4. Final fallback: `$PWD/logging_data`.

Typical use: source `install/setup.bash` in the colcon ws, then `ros2 launch ...` -- step 2 keeps all sessions under `{ws}/logging_data` regardless of cwd.

---

## Code Conventions

- **Style guide**: Follows [ROS 2 Code Style](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html) and [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) as baseline
- **Namespace**: `rtc` (all packages)
- **Naming**: Google C++ Style — `snake_case` members with trailing `_`, `PascalCase` classes/types, `kConstant` for compile-time constants
- **Modern C++20**: Prefer `std::jthread`/`stop_token` over raw threads, `std::span` over pointer+size, `std::string_view` over `const std::string&` for non-owning, `concepts` for template constraints, `[[likely]]/[[unlikely]]` for branch hints, `constexpr` wherever possible, structured bindings, `std::optional`/`std::expected` over error codes
- **RAII**: All resource acquisition (sockets, file handles, memory) via RAII wrappers. Raw `new`/`delete` prohibited; use stack allocation or pre-allocated buffers
- **`noexcept`** on all RT paths (exceptions = process termination, intentional)
- **`[[nodiscard]]`** on status-returning functions; **`static_assert`** on template params
- **Include order**: project headers → ROS 2 / third-party → C++ stdlib (each group alphabetically sorted)
- **Eigen**: pre-allocated buffers, `noalias()`, zero heap on 500Hz path. Never use `auto` for Eigen expressions (expression template aliasing)
- **Compiler flags**: `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion`
- **ROS 2 Node**: Prefer `rclcpp_lifecycle::LifecycleNode` over `rclcpp::Node` for managed state transitions (unconfigured → inactive → active → finalized)
- **ROS 2 API**: Use `rclcpp::QoS` profiles explicitly (never rely on defaults), `MutuallyExclusiveCallbackGroup` for thread-safety, `ParameterDescriptor` with ranges for declared parameters

### Documentation Requirements

- **Doxygen for all public API**: Every public class, function, and non-trivial member must have Doxygen-style comments (`@brief`, `@param`, `@return`, `@note`)
- **Kinematics & control algorithms**: Functions implementing mathematical formulas must include:
  - The equation in LaTeX-compatible Doxygen notation (`@f$..@f$`)
  - Reference to the source paper/textbook (author, year, equation number)
  - Units and coordinate frame assumptions for each parameter
  - Example:
    ```cpp
    /**
     * @brief Compute damped Jacobian pseudoinverse.
     *
     * @f$ J^{\dagger} = J^T (J J^T + \lambda^2 I)^{-1} @f$
     *
     * Ref: Wampler (1986), Eq. 12. Uses body Jacobian in base frame.
     *
     * @param J Body Jacobian (6 x n_joints), base frame
     * @param damping Damping factor lambda [dimensionless], loaded from YAML
     * @return Damped pseudoinverse (n_joints x 6)
     * @note Singular values below 1e-6 are clamped. See Domain Conventions for frame rules.
     */
    ```
- **FSM and state transitions**: Document valid transitions, entry/exit conditions, and timeout behaviors
- **Thread safety**: Every shared data member must document its synchronization mechanism (SeqLock, SPSC, atomic, mutex with scope)

---

## Domain Conventions

These conventions apply to all `rtc_*` (robot-agnostic) packages. Robot-specific packages (`ur5e_*`) inherit these and may add hardware-specific constraints documented in their own README.

- **Coordinate frame**: Right-hand rule, ZYX Euler (roll-pitch-yaw)
- **Rotation representation**: Internal computation = quaternion (`Eigen::Quaterniond`, Hamilton convention). Euler conversion allowed only at API boundaries
- **Quaternion interpolation**: Always use `slerp` — `lerp`/`nlerp` are prohibited (no normalization guarantee, geodesic path distortion). Normalize quaternions after any arithmetic operation
- **Units**: SI base (m, rad, s, kg, N). Degree inputs must be explicitly converted to radians
- **Jacobian**: Body Jacobian by default. Spatial Jacobian requires `_spatial` suffix in function/variable names
- **Dynamics**: M(q)q̈ + C(q,q̇)q̇ + g(q) = τ notation. Pinocchio RNEA-based
- **Variable naming**: Respect paper notation — `J_b` (body Jacobian), `q_d` (desired joint), `x_e` (end-effector pose), `K_d` (desired stiffness)
- **Singularity handling**: Damped pseudoinverse required (`damping` parameter via YAML config), division-by-near-zero protection mandatory

---

## Anti-patterns (Never Generate)

| Forbidden Pattern | Reason | Alternative |
|-------------------|--------|-------------|
| `new`/`malloc`/`std::vector::push_back` on RT path | Breaks RT determinism | `std::array`, pre-allocated `Eigen::Matrix<fixed>` |
| `throw`/`catch` on RT path | `noexcept` violation -> process termination | Error code return, `std::optional` |
| `std::cout`/`RCLCPP_*` on RT path | Blocking I/O | SPSC queue -> delegate to logging thread |
| `std::mutex::lock` on RT path | Priority inversion | `try_lock`, SeqLock, SPSC |
| `auto` return type with Eigen | Expression template aliasing | Explicit types (`Eigen::Vector3d`, etc.) |
| `rclcpp::spin()` standalone | Single-thread bottleneck | `MultiThreadedExecutor` + CallbackGroup separation |
| `lerp` for quaternion interpolation | Non-normalized, path distortion | `slerp` required |
| Hardcoded control gains | Cannot tune at runtime | YAML parameters + `~/controller_gains` topic |
| Indiscriminate `// NOLINT` usage | Suppresses warnings without fixing | Fix root cause |
| Modifying existing test assertions to pass | Hides regressions | Fix new code to pass existing tests |
| Code change without doc/metadata update | Causes doc-code drift | Always update README, CMakeLists, package.xml, YAML together |

---

## AI Harness Engineering

This section defines behavioral protocols that Claude Code must follow when working on this project.

### Critical Thinking Protocol

Before writing any code, self-check the following questions. If any concern is found, raise it with the user before proceeding.

**Safety**
- Could this change compromise deterministic execution of the RT loop (500Hz)?
- Is the code safe under exceptional conditions (singularity, joint limit, communication loss, E-STOP)?
- Are numerically unstable operations (matrix inversion, near-zero division) properly guarded?

**Design Coherence**
- Is this consistent with existing architecture patterns (Strategy controllers, SPSC offload, SeqLock sharing)?
- Is there a simpler alternative to this approach?
- Is introducing a new dependency justified?

**Mathematical Correctness**
- Do coordinate frame (right-hand), units (SI), and rotation convention (Hamilton quaternion, ZYX Euler) match Domain Conventions?
- Are Jacobian/mass matrix dimensions consistent?

**Performance Impact**
- Can the computation complete within the control period (2ms @ 500Hz)?
- Are there unnecessary copies, redundant computations, or heap allocations?

When a concern is found, report it in this format:

```
[CONCERN] <one-line summary>
Severity: Critical | Warning | Info
Detail: <specific issue>
Alternative: <suggested approach>
```

- **Critical** (RT safety, numerical instability, data loss): Do NOT proceed without user confirmation
- **Warning** (performance degradation, design inconsistency): Raise concern but follow user's decision
- **Info** (better alternative, style suggestion): Mention briefly and proceed

If a user request appears technically problematic, provide evidence-based alternatives. However, if the user insists after hearing the rationale, comply — the user may have context that Claude does not.

### Self-QA Protocol

Before finalizing any generated code, verify that the output does not exhibit typical AI-generated slop:

**Slop Detection Checklist**
- No placeholder or stub implementations left behind (`// TODO: implement`, empty function bodies) unless explicitly agreed with the user
- No hallucinated APIs — every function call, method name, and class reference must exist in the actual codebase or linked library; when uncertain, read the source file first
- No generic variable names (`data`, `result`, `temp`, `value`) where domain-specific names exist (use `joint_positions`, `jacobian_body`, `force_error`)
- No redundant comments restating what the code obviously does (`// increment counter` above `++counter`)
- No over-abstraction — do not introduce wrapper classes, factory patterns, or inheritance hierarchies that the existing architecture does not use
- No copy-paste drift — if generating multiple similar blocks (per-joint, per-device), verify each instance uses the correct index, name, and configuration

**Edge Case Audit**
Before declaring implementation complete, explicitly verify handling of:
- **Singularity**: Jacobian rank deficiency near singular configurations → damped pseudoinverse with configurable `damping`
- **Joint limits**: `q` approaching `q_min`/`q_max` → clamp or null-space repulsion, never ignore
- **Communication loss**: State topic not received within timeout → E-STOP trigger, not silent stale-data usage
- **Array bounds**: Device index ≥ `num_devices`, channel index ≥ `kMaxDeviceChannels` → bounds check or `static_assert`
- **Numerical precision**: Quaternion normalization after arithmetic, rotation matrix orthogonality, `dt` near zero guard
- **E-STOP recovery**: Controller state after E-STOP clear → must re-initialize hold position, not resume from stale target
- **Empty/zero input**: Zero-length trajectory, identity rotation target, zero-force grasp command → graceful no-op or explicit rejection
- **Thread safety**: Any new shared state between RT and non-RT threads → must use SeqLock, SPSC, or atomic; document the chosen mechanism

If any edge case is intentionally left unhandled, document it explicitly in the response with rationale.

### Task Decomposition Policy

**Decomposition trigger** -- If any of the following conditions apply, present a task plan for user approval before starting implementation:
- 3+ files need to be created or modified
- Changes span 2+ packages (refer to dependency graph)
- New class hierarchy or interface design is involved
- Estimated code volume exceeds 300 lines

**Atomic task criteria**:
- Single responsibility: focus on one module or one feature
- Self-verifiable: can independently pass Verification Gates
- Interface-first: finalize headers/interfaces before implementation
- 100-200 lines of change (excluding tests)

**Decomposition order**: Interface definition -> core data structures -> algorithm implementation -> external integration -> tests -> config/docs

**Task plan format**:

```
Goal: <one line>
Scope: <package/directory list>

Step 1/N -- <title>
  Files: <target files>
  Work: <description>
  Depends: <prior Step or existing code>
  Verify: <applicable Gate list>

Step 2/N -- ...
```

Each Step must include [Files, Work, Depends, Verify]. When user says "proceed", execute one Step at a time. When user says "proceed all", execute all Steps sequentially and report verification results after each.

### Context Management

**Context Anchor** -- Record the following at the end of each Step completion during multi-step tasks:

```
[Context Anchor -- Step N/M complete]
- Finalized interfaces: <Class>::<key method signatures>
- Key design decisions: <decisions made in this step>
- Shared data structures: <types/constants referenced by next step>
- Remaining TODOs: <intentionally deferred items>
- Next Step: <what comes next>
```

**File-based context preservation** -- When conversations grow long, persist key design decisions to files to guard against context window eviction:

```bash
echo "## $(date +%Y-%m-%d): <decision summary>" >> docs/DESIGN_DECISIONS.md
echo "- [ ] <item>" >> docs/TODO.md
```

**Context recovery protocol** -- Execute before resuming work in a new conversation or when context exhaustion is suspected:

```bash
find . -maxdepth 2 -name '*.hpp' -path '*/include/*' | head -30
git log --oneline -10
git diff --stat HEAD~3
cat rtc_controller_interface/include/rtc_controller_interface/rt_controller_interface.hpp
cat docs/DESIGN_DECISIONS.md 2>/dev/null
cat docs/TODO.md 2>/dev/null
```

Report current state summary to the user, then resume work.

**Context exhaustion self-detection** -- When any of the following occurs, do NOT write code from guesswork. Re-read the relevant file(s) first:
- Cannot recall a function signature finalized in a previous Step
- Need to read the same file more than twice
- User points out inconsistency with a prior decision

If the conversation has grown too long, update Context Anchor + `docs/DESIGN_DECISIONS.md`, then suggest starting a new conversation.

### Large Change Management

**Interface-contract-first**: For changes spanning multiple packages: (1) Finalize all affected headers and confirm build passes -> (2) Implement each module independently -> (3) Integration tests after all implementations complete.

**Change scope limits**:
- Do not modify 4+ files simultaneously in a single Step (split into smaller Steps if exceeded)
- Do not mix refactoring and feature additions in the same Step
- Changes to `rtc_base` or `rtc_msgs` require full downstream build + test per dependency graph

**Git commit granularity**: Commit after each Step passes all Gates. Format: `[package_name] summary (Step N/M)`

### Tool Usage Policy

**Before modifying code**:
- Always read the current file content and understand its structure before editing
- Check related headers/interfaces first to prevent signature mismatches
- If existing tests exist, review them first and verify they still pass after modification

**Command execution rules**:
- Never run long-blocking commands (`ros2 launch`, `while true`, unbounded `ros2 topic echo`, etc.)
- Build only changed packages: `./build.sh -p <pkg>` or `colcon build --packages-select`
- On test failure, read the full log and analyze root cause before attempting fixes (no blind retries)

### Self-Verification Protocol

After generating or modifying code, pass the following Gates in order. On Gate failure, fix and re-verify (max 3 attempts). After 3 failures, write a Failure Report.

**Gate 1 -- Lint & Format**

```bash
clang-format --dry-run --Werror <modified_files>
clang-tidy <modified_files> -p build/ --warnings-as-errors='*'
ruff check <modified_python_files>
ruff format --check <modified_python_files>
```

- Respect project `.clang-tidy` and `.clang-format` settings
- On format mismatch, auto-fix and report the diff

**Gate 2 -- Build**

```bash
./build.sh -p <package_name>
```

- Treat warnings as errors (project compiler flags enforce this)
- On build failure, resolve errors sequentially starting from the first (do not batch-fix cascading errors)
- For template/constexpr errors, trace back to the instantiation point

**Gate 3 -- Test**

```bash
colcon test --packages-select <pkg> --event-handlers console_direct+
colcon test-result --verbose
```

- If existing tests break, fix the new code (never modify existing test assertions)
- New public functions/classes must have corresponding tests
- Numerical tests must specify tolerance (`EXPECT_NEAR`, `approx`)

**Gate 4 -- RT Safety Audit**

```bash
grep -rn 'new \|malloc\|throw \|std::cout\|std::cerr\|std::mutex' \
  rtc_controller_manager/src/ rtc_controllers/src/ ur5e_hand_driver/src/ \
  --include='*.cpp' --include='*.hpp'
grep -rn 'push_back\|emplace_back\|resize\|reserve' \
  rtc_controller_manager/src/ rtc_controllers/src/ ur5e_hand_driver/src/ \
  --include='*.cpp' --include='*.hpp'
```

- If any forbidden pattern is detected in RT paths, replace with pre-allocated or lock-free alternatives
- Non-RT paths (`rtc_tools/`, `rtc_digital_twin/`, `test/`) are exempt

**Gate 5 -- Documentation & Metadata Audit**

Verify that all required non-code files have been updated alongside the code change. **A code change without corresponding documentation and metadata updates must not pass this gate.**

```bash
git diff --name-only HEAD
git diff --name-only HEAD -- '<pkg>/src/' '<pkg>/include/' | head -1 && \
  git diff --name-only HEAD -- '<pkg>/README.md' | grep -q README.md || \
  echo "FAIL: source changed but README.md not updated"
```

Mandatory co-update matrix -- if any left-column artifact was changed, all corresponding right-column files **MUST** be updated:

| Changed Artifact | Required Co-updates |
|-----------------|---------------------|
| New/renamed `.cpp` or `.hpp` | `CMakeLists.txt` (source lists, install targets) |
| New `find_package()` / dependency | `CMakeLists.txt` + `package.xml` (must stay in sync) |
| New/changed public API | Package `README.md` (API docs, examples) + Doxygen header comments |
| New/changed configurable parameter | YAML config files (value + comment with range/units) + package `README.md` |
| New/changed topic name or device group | Controller YAML `topics:` section + package `README.md` |
| New test file | `CMakeLists.txt` (`ament_add_gtest` / `ament_add_pytest_test`) |
| New/changed `.msg`/`.srv`/`.action` | `CMakeLists.txt` (`rosidl_generate_interfaces`) + `package.xml` |
| Cross-package behavioral change | `CLAUDE.md` + `docs/*.md` + root `README.md` as applicable |
| Significant feature or breaking API | `package.xml` version bump (semver) |
| Function contract/precondition change | Header Doxygen comment (thread-safety, param constraints) |

### Failure Reporting Protocol

When self-verification fails after 3 retry attempts or the task cannot be completed, submit a structured failure report:

```
[UNRESOLVED] <one-line problem summary>

Symptom: <observed error message, verbatim>
Root cause analysis: <why it fails; mark as "suspected" if uncertain>
Attempted fixes:
  1. <attempt 1> -> <result>
  2. <attempt 2> -> <result>
  3. <attempt 3> -> <result>
Current state: <buildable/unbuildable, partial functionality, etc.>
Impact scope: <effect on other modules>
Suggestion: <areas for user to review, alternative approaches>
```

**Never do the following**:
- Optimistic hedging: "it will probably work", "might be an environment difference"
- Ignore errors and wrap up with "everything else works fine"
- Silently reduce feature scope without mentioning it
- Shift verification responsibility: "please test in your environment"
- Paste error logs without root cause analysis
- Commit code in a failing state

### Completion Checklist

Self-check after every code modification/generation task:

- [ ] clang-format / ruff format passed
- [ ] clang-tidy 0 warnings / ruff check 0 warnings
- [ ] `./build.sh -p <pkg>` succeeded (0 warnings)
- [ ] All existing tests pass
- [ ] New tests added for new public API
- [ ] No forbidden patterns detected in RT paths
- [ ] `CMakeLists.txt` updated (sources, deps, install targets, test targets)
- [ ] `package.xml` updated (deps in sync with CMakeLists.txt, version bump if needed)
- [ ] YAML config files updated (new/changed parameters, defaults, comments with range/units)
- [ ] Package `README.md` updated (API, parameters, usage examples, dependencies)
- [ ] Doxygen comments added/updated for new/changed public headers
- [ ] Cross-package docs updated if applicable (`CLAUDE.md`, `docs/*.md`, root `README.md`)
- [ ] Changed file list and modification summary reported to user

---

## RT Permissions

```bash
sudo groupadd realtime && sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# Re-login required. Optional: isolcpus, nohz_full, or cpu_shield.sh
```

---

## Debugging

| Symptom | Fix |
|---------|-----|
| `ApplyThreadConfig()` warns | `sudo usermod -aG realtime $USER` + re-login |
| E-STOP on startup | Set `init_timeout_sec: 0.0` for sim |
| High jitter (>200us) | Check `taskset` pinning, verify `isolcpus` |
| Hand timeout E-STOP | Check UDP link, `recv_timeout_ms: 0.4` |
| Controller not found | Use config_key (e.g. "p_controller") or Name() |

```bash
PID=$(pgrep -f rt_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
ros2 topic hz /forward_position_controller/commands
ros2 topic echo /system/estop_status
./rtc_scripts/scripts/check_rt_setup.sh --summary
```

---

## Testing

```bash
# All tests
colcon test --event-handlers console_direct+
colcon test-result --verbose

# Single package
colcon test --packages-select ur5e_bt_coordinator --event-handlers console_direct+

# Single test (C++)
colcon test --packages-select rtc_controllers --ctest-args -R test_grasp_controller

# Single test (Python)
colcon test --packages-select rtc_digital_twin --pytest-args -k test_urdf_parser
```

Test files by package (41 total):

| Package | Tests | Framework |
|---------|-------|-----------|
| `ur5e_bt_coordinator` | 14 C++ tests (`test/test_*.cpp`) | GTest |
| `rtc_tsid` | 11 C++ tests (QP solver, tasks, constraints, formulations, performance) | GTest |
| `rtc_controllers` | 6 C++ tests (trajectory + grasp) | GTest |
| `rtc_urdf_bridge` | 5 C++ tests (URDF/model parsing) | GTest |
| `shape_estimation` | 3 C++ tests (ToF + exploration) | GTest |
| `rtc_digital_twin` | 1 Python test | pytest |
| `rtc_tools` | 1 Python test | pytest |

---

## Key File Locations

| What | Path |
|------|------|
| Core types & constants | `rtc_base/include/rtc_base/types/types.hpp` |
| Thread configs | `rtc_base/include/rtc_base/threading/thread_config.hpp` |
| Controller abstract base | `rtc_controller_interface/include/rtc_controller_interface/rt_controller_interface.hpp` |
| Controller registry | `rtc_controller_interface/include/rtc_controller_interface/controller_registry.hpp` |
| RT control loop | `rtc_controller_manager/src/rt_controller_node.cpp` |
| Controller registration | `rtc_controllers/src/controller_registration.cpp` |
| Demo controller registration | `ur5e_bringup/src/controllers/controller_registration.cpp` |
| Grasp controller | `rtc_controllers/include/rtc_controllers/grasp/grasp_controller.hpp` |
| Grasp types/params | `rtc_controllers/include/rtc_controllers/grasp/grasp_types.hpp` |
| Main YAML (robot/sim) | `ur5e_bringup/config/ur5e_robot.yaml` / `ur5e_sim.yaml` |
| MuJoCo configs | `rtc_mujoco_sim/config/mujoco_simulator.yaml`, `solver_param.yaml` |
| Hand driver config | `ur5e_hand_driver/config/hand_udp_node.yaml` |
| UR5e URDF / MJCF | `ur5e_description/robots/ur5e/{urdf,mjcf}/` |
| BT trees + launch | `ur5e_bt_coordinator/trees/*.xml`, `launch/bt_coordinator.launch.py` |
| Arm poses | `ur5e_bt_coordinator/config/poses.yaml` |
| CycloneDDS config | `rtc_controller_manager/config/cyclone_dds.xml` |
| TSID controller | `rtc_tsid/include/rtc_tsid/controller/tsid_controller.hpp` |
| TSID formulations | `rtc_tsid/include/rtc_tsid/formulation/{wqp,hqp}_formulation.hpp` |
| TSID tasks/constraints | `rtc_tsid/include/rtc_tsid/core/{task_base,constraint_base}.hpp` |
| TSID types | `rtc_tsid/include/rtc_tsid/types/wbc_types.hpp` |
| Supplementary docs | `docs/` (RT_OPTIMIZATION.md, SHELL_SCRIPTS.md, VSCODE_DEBUGGING.md) |