# CLAUDE.md

## Project Purpose

**RTC (Real-Time Control) Framework** -- Robot-agnostic real-time control for URDF-based manipulators.
`rtc_*` packages: variable-DOF, 500Hz-2kHz, transport abstraction (UDP/CAN-FD/EtherCAT/RS485), lock-free SPSC, E-STOP.
`ur5e_*` packages: UR5e + 10-DOF hand drivers, demo controllers, BT coordinator (reference integration).

## Tech Stack

| Item | Value |
|------|-------|
| Lang | C++20 (GCC 11+/13+), Python 3.10+ |
| OS | Ubuntu 22.04 (PREEMPT_RT optional) / 24.04 |
| Middleware | ROS 2 Humble / Jazzy, CycloneDDS |
| Build | CMake 3.22+, colcon, ament_cmake / ament_python |
| Deps | Eigen 3.4, Pinocchio, ProxSuite, ONNX Runtime |
| Optional | MuJoCo 3.x (sim), BehaviorTree.CPP v4 |
| Test | GTest, pytest (238 tests across 12 packages) |

## Directory Map

19 ROS2 packages at repo root. 13 `rtc_*` (robot-agnostic) + 2 `shape_estimation_*` + 4 `ur5e_*` (robot-specific).

| Package | Role |
|---------|------|
| `rtc_base` | Types, SeqLock, SPSC, threading, filters, DataLogger |
| `rtc_communication` | TransportInterface, UdpSocket, PacketCodec, Transceiver |
| `rtc_controller_interface` | RTControllerInterface base, ControllerRegistry, RTC_REGISTER_CONTROLLER |
| `rtc_controllers` | PController, JointPD, CLIK, OSC, GraspController |
| `rtc_controller_manager` | RtControllerNode: 500Hz RT loop, SPSC publish, E-STOP |
| `rtc_inference` | InferenceEngine, OnnxEngine |
| `rtc_msgs` | ROS2 message definitions |
| `rtc_mujoco_sim` | MuJoCo 3.x wrapper |
| `rtc_tsid` | TSID QP: WQP/HQP, tasks, constraints, ProxSuite |
| `rtc_mpc` | MPC-RT interface: TripleBuffer, TrajectoryInterpolator, RiccatiFeedback, RobotModelHandler |
| `rtc_urdf_bridge` | URDF parser + Pinocchio model builder |
| `rtc_tools` | Python GUI/plotting tools |
| `rtc_scripts` | PREEMPT_RT, CPU shield, IRQ affinity scripts |
| `rtc_digital_twin` | RViz2 JointState merge |
| `ur5e_description` | URDF + MJCF + meshes |
| `ur5e_hand_driver` | UDP hand driver, ONNX F/T inference |
| `ur5e_bt_coordinator` | BehaviorTree.CPP v4 coordinator |
| `ur5e_bringup` | Launch files, demo controllers, config |

## Build & Test

**Every new shell (interactive `colcon test` / `ros2 launch`)**: `source rtc_scripts/scripts/setup_env.sh` — orders ROS 2 + `deps/install` (fmt/mimalloc/aligator) + `.venv` + workspace overlay, and sets `COLCON_DEFAULTS_FILE`. `build.sh` / `install.sh` auto-source it, so pure build flow doesn't need manual source.

```bash
source rtc_scripts/scripts/setup_env.sh   # PWD=src/rtc-framework, needed for ros2/colcon test

./build.sh sim            # simulation packages
./build.sh robot          # real robot
./build.sh full           # all packages
./build.sh -p rtc_base    # single package

ros2 launch ur5e_bringup sim.launch.py
ros2 launch ur5e_bringup robot.launch.py robot_ip:=192.168.1.10

colcon test --packages-select <pkg> --event-handlers console_direct+
colcon test-result --verbose
```

**Isolated deps** (see `ISOLATION_PLAN.md`): fmt 11.1.4 · mimalloc 2.1.7 · aligator 0.19.0 live at `<rtc_ws>/deps/install/` (built via `rtc_scripts/scripts/build_deps.sh` from `deps.repos`). Pinocchio · ProxSuite · hpp-fcl · eigenpy come from ROS Jazzy apt (no robotpkg / no /usr/local). Python deps locked in `requirements.lock`. `rtc_scripts/README.md` has detailed docs for all 14 scripts (10 RT + 4 isolation).

## Hard Rules

RT path (500Hz loop) -- never generate:
1. `new`/`malloc`/`push_back` -- use `std::array`, pre-allocated fixed-size Eigen
2. `throw`/`catch` -- use error codes, `std::optional`
3. `std::cout`/`RCLCPP_*` -- use SPSC queue to logging thread
4. `std::mutex::lock` -- use `try_lock`, SeqLock, SPSC
5. `auto` with Eigen -- expression template aliasing; use explicit types
6. `lerp` for quaternions -- use `slerp` only
7. Never modify existing test assertions to pass -- fix new code instead

Design rules for `rtc_*` packages:
- No robot names, joint counts, or HW identifiers hardcoded -- YAML/template only
- No upward dependencies -- respect the dependency graph (see `agent_docs/architecture.md`)
- Interface-first: abstract class/concept before concrete implementation
- Code change without doc/metadata update is incomplete (see `agent_docs/modification-guide.md`)

When a concern is found, report: `[CONCERN] summary / Severity: Critical|Warning|Info / Detail / Alternative`

## Post-Task Housekeeping

Run after a commit lands or the user signals task completion:
1. Save only *surprising / non-obvious* learnings to memory per auto-memory type rules (user / feedback / project / reference). Skip code patterns, git-derivable facts, ephemeral state.
2. Delete stale artifacts -- `~/.claude/plans/*.md` tied to completed work, repo-root / `/tmp` scratch files. Verify content is preserved elsewhere (git log, `agent_docs/*.md`, `docs/*.md`) before deleting.
3. Prune memory entries that are now outdated or wrong.
4. Report only items actually performed, one line each.

## Reference Docs (read when relevant)

- `agent_docs/architecture.md`        -- Threading, data flow, core types, lock-free rules, lifecycle, E-STOP
- `agent_docs/controllers.md`         -- Controller table, gains layout, GraspController FSM, topics, config files
- `agent_docs/modification-guide.md`  -- Adding controllers/messages/devices/threads, package update checklist
- `agent_docs/design-principles.md`   -- rtc_* 5 principles, rtc_* vs ur5e_* boundary rules
- `agent_docs/conventions.md`         -- Domain conventions, documentation requirements, edge case audit
- `agent_docs/testing-debug.md`       -- Test table, debugging tips, RT permissions
