# Controller Safety Improvements Plan

## Context

Deep analysis of `rtc_controller_interface`, `rtc_controller_manager`, `rtc_controllers`, `ur5e_bringup` identified thread safety, RT safety, and code quality issues across the controller stack. Work is tracked in phases.

## Completed (commit 3800403)

### Phase 0 — Unit Tests
- `rtc_controllers/test/test_core_controllers.cpp`: 33 tests for PController(10), JointPD(8), CLIK(8), OSC(7)
- CMakeLists.txt updated with `test_core_controllers` target
- Uses `rtc_urdf_bridge/test/urdf/serial_6dof.urdf` (all Z-axis revolute, 6-DOF)

### Phase 1 — Bool Flag Snapshots in Compute()
Prevent mid-tick branch inconsistency when `UpdateGainsFromMsg()` runs concurrently on aux thread.

| Controller | Snapshot variables | File |
|---|---|---|
| ClikController | `use_6dof`, `use_null_space` | `clik_controller.cpp` |
| JointPDController | `use_gravity`, `use_coriolis` | `joint_pd_controller.cpp` |
| OSC | `use_gravity` | `operational_space_controller.cpp` |

### Phase 2 — RT Principle Fixes
- **R-1:** `InitializeHoldPosition()` changed from `std::lock_guard` to `std::try_to_lock` in JointPD, CLIK, OSC
- **R-2:** `TriggerGlobalEstop()` — `estop_reason_` changed to `std::array<char, 128>`, logging deferred via `estop_log_pending_` atomic to `DrainLog()`
- **R-4:** `trajectory_speed` validation — `std::max(1e-6, val)` at 19 sites (rtc_controllers: 6, ur5e_bringup: 13)

## Remaining Work

### Phase 1b — SeqLock<Gains> Migration ✅ (complete)

All 7 controllers migrated to `rtc::SeqLock<Gains> gains_lock_` with single-snapshot
Load in RT path, Load/mutate/Store in aux-thread writers. Build + 33 Phase 0 tests pass.

Per-controller snapshot site:
- PController, JointPDController, ClikController, OperationalSpaceController:
  snapshot at top of `Compute()`.
- DemoJointController, DemoTaskController: snapshot at top of `ComputeControl()`;
  `UpdateVirtualTcp()` signature extended to take `const Gains&`.
- DemoWbcController: snapshot at top of `OnPhaseEnter()` (phase-transition path);
  UpdateGainsFromMsg keeps `grasp_cmd_` atomic write outside the gains snapshot.
- `SetDeviceTarget()` readers of `gains_.control_6dof` (Clik, DemoTask) use
  `gains_lock_.Load().control_6dof` — aux-thread serialized by ROS executor.

### Phase 3 — Code Quality

| ID | Task | Package | Detail |
|---|---|---|---|
| Q-1 | `kSafePosition` from YAML | ur5e_bringup ×3 | Replace `static constexpr` with runtime load from `DeviceNameConfig::safe_position` (already available via `OnDeviceConfigsSet()`) |
| Q-2 | `kFingerJointMap`/`kHandIdx*` from YAML | ur5e_bringup | Move hardcoded hand joint indices to `demo_shared.yaml` |
| Q-3 | Velocity clamp utility | rtc_controllers/rtc_base | Extract duplicated `ClampCommands()` loop into shared function |
| Q-4 | Device passthrough utility | rtc_controller_interface | Extract duplicated "Device 1+ passthrough" loop |
| Q-5 | `GetCurrentGains()` heap removal | rtc_controllers ×4 | Change return type from `std::vector<double>` to caller-provided `span<double>` or `std::array` |
| Q-6 | Registry duplicate check | rtc_controller_interface | Warn on duplicate `config_key` in `ControllerRegistry::Register()` |
| Q-7 | E-STOP ramp investigate | ur5e_bringup | Verify if DemoWbcController's instant-jump E-STOP (vs ramp in Joint/Task) is intentional |

### Phase 4 — Long-term

| ID | Task |
|---|---|
| L-1 | Clarify target double-buffering (RtControllerNode `device_targets_` vs controller internal targets) |
| L-2 | Document RTControllerInterface initialization order in header |
| L-3 | RtControllerNode integration tests |
| L-4 | GraspController RTControllerInterface wrapper evaluation |
| L-5 | WBC MPC thread lifecycle audit |
| L-6 | ClikController/OSC goal reporting snapshot (Low — GUI only) |

## Critical Analysis Notes

These severity assessments were refined during planning:

| Original Assessment | Revised | Reason |
|---|---|---|
| C-2 (target unprotected read) = Critical | **Low (L-6)** | Only affects GUI goal reporting, not control computation |
| H-1 (InitializeHoldPosition lock) = High | **Medium** | Called once per lifecycle, contention window ~µs |
| H-2/H-3 (E-STOP/init RCLCPP) = High | **Low** | One-shot paths, system dying/starting anyway |
| GainsDoubleBuffer template | **Use existing SeqLock** | Avoid new abstraction when codebase already has SeqLock |
| kSafePosition = "design rule violation" | **Code duplication** | ur5e_bringup is robot-specific package, hardcoding allowed |
| trajectory_speed = 0 "crash" | **Hang** | IEEE 754: 1/0 = INF, not crash. Fixed anyway |
