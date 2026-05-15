# Controller Safety Improvements Plan

> **Status: Closed** — Phase 0~4 모두 완료 (2026-04-26). 이 문서는 historical reference로 archive에 보관. 향후 controller safety task가 등장하면 별도 plan으로 분리한다. 본문 내 다른 문서 참조의 상대 경로는 archive 이동 전 기준 (`architecture.md` 등은 `../architecture.md`로 해석).

## Context

Deep analysis of `rtc_controller_interface`, `rtc_controller_manager`, `rtc_controllers`, `integrated_bringup` identified thread safety, RT safety, and code quality issues across the controller stack. Work is tracked in phases.

## Completed (commit 3800403)

### Phase 0 — Unit Tests
- `rtc_controllers/test/test_core_controllers.cpp`: 33 tests for PController(10), JointPD(8), CLIK(8), OSC(7)
- CMakeLists.txt updated with `test_core_controllers` target
- Uses `rtc_urdf_bridge/test/urdf/serial_6dof.urdf` (all Z-axis revolute, 6-DOF)

### Phase 1 — Bool Flag Snapshots in Compute()
Prevent mid-tick branch inconsistency when an aux-thread gains writer runs concurrently (originally `UpdateGainsFromMsg`; since the 2026-04-26 migration, the parameter callback `OnGainParametersSet`).

| Controller | Snapshot variables | File |
|---|---|---|
| ClikController | `use_6dof`, `use_null_space` | `clik_controller.cpp` |
| JointPDController | `use_gravity`, `use_coriolis` | `joint_pd_controller.cpp` |
| OSC | `use_gravity` | `operational_space_controller.cpp` |

### Phase 2 — RT Principle Fixes
- **R-1:** `InitializeHoldPosition()` changed from `std::lock_guard` to `std::try_to_lock` in JointPD, CLIK, OSC
- **R-2:** `TriggerGlobalEstop()` — `estop_reason_` changed to `std::array<char, 128>`, logging deferred via `estop_log_pending_` atomic to `DrainLog()`
- **R-4:** `trajectory_speed` validation — `std::max(1e-6, val)` at 19 sites (rtc_controllers: 6, integrated_bringup: 13)

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
  the parameter callback / `~/grasp_command` srv handler keep `grasp_cmd_`
  atomic write outside the gains snapshot.
- `SetDeviceTarget()` readers of `gains_.control_6dof` (Clik, DemoTask) use
  `gains_lock_.Load().control_6dof` — aux-thread serialized by ROS executor.

### Phase 3 — Code Quality (완료 2026-04-26)

Q-1~Q-4 (low-traffic, outbound-link 없음) — 한 줄 요약:
- **Q-1, Q-2**: integrated_bringup 의 robot-specific 상수 (`kSafePosition`, `kFingerJointMap`, `kHandIdx*`) 를 YAML 로 이관. `safe_position_` 멤버 + `DemoSharedConfig::hand_finger_joint_map` 추가.
- **Q-3, Q-4**: 공통 유틸 추출 — `rtc_base/utils/clamp_commands.hpp` (ClampSymmetric/Range), `rtc_base/utils/device_passthrough.hpp` (PassthroughSecondaryDevices). 4-7개 컨트롤러 리팩터.

| ID | Task | Package | Status |
|---|---|---|---|
| Q-6 | Registry duplicate check | rtc_controller_interface | **Done** — `ControllerRegistry::Register()` 가 duplicate `config_key` 시 RCLCPP_WARN. `RegisterDuplicateShadowsAndAppends` gtest 추가 |
| Q-7 | **E-STOP ramp** | integrated_bringup | **Done (2026-04-26)** — `DemoWbcController::ComputeEstop` now follows the Joint/Task ramp pattern: `out0.commands[i] = dev0.positions[i] + clamp(safe-cur, ±device_max_velocity_[0][i]) * dt`. Eliminates instant-jump hardware risk on real UR5e |
| Q-8 | **`kContactStopReleaseEps` → YAML** | integrated_bringup | **Done (2026-04-26)** — Joint mirrors Task: new `Gains::contact_stop_release_eps` (default 0.005), required `fsm.contact_stop_release_eps` key in `demo_joint_controller.yaml` (range-checked [0, 0.1]), use sites read from per-tick gains snapshot. `kContactStopReleaseEps` constexpr removed |

(Q-5 was dropped — `GetCurrentGains()` virtual itself removed by the gain → ROS 2 parameter migration on 2026-04-26.)

### Phase 4 — Long-term (재평가 완료 2026-04-26)

| ID | Task | Status |
|---|---|---|
| L-1 | Clarify target double-buffering (CM `device_targets_` aux-write vs `device_target_snapshots_` RT try_lock vs controller-internal trajectory intent) | **Doc-only, low** — 1-paragraph in architecture.md if revisited |
| L-2 | Document RTControllerInterface initialization order | **Closed** — already in `rt_controller_interface.hpp:56-90` |
| L-3 | RtControllerNode integration tests | **Substantially closed** — `test_controller_lifecycle` (9), `test_switch_service` (9), `test_demo_wbc_mpc_integration` (6) |
| L-4 | GraspController RTControllerInterface wrapper evaluation | **Closed (not needed)** — Force-PI vs TSID bifurcation settled; wrapper would add 3rd paradigm with no consumer |
| L-5 | WBC MPC thread lifecycle audit | **Superseded** — rtc_cm Phase 1.5 (`e3d2c70`) DemoWbc idempotent activate + MPCThread Pause/Resume + Phase 6 handler integration |
| L-6 | ClikController/OSC goal reporting snapshot | **Closed (already done)** — `output.actual_task_positions` populated by both, fan-out via publish thread |

→ Phase 4 는 **사실상 완료**. 새 항목 등장 시 별도 plan 으로 분리.

## Critical Analysis Notes

These severity assessments were refined during planning:

| Original Assessment | Revised | Reason |
|---|---|---|
| C-2 (target unprotected read) = Critical | **Low (L-6)** | Only affects GUI goal reporting, not control computation |
| H-1 (InitializeHoldPosition lock) = High | **Medium** | Called once per lifecycle, contention window ~µs |
| H-2/H-3 (E-STOP/init RCLCPP) = High | **Low** | One-shot paths, system dying/starting anyway |
| GainsDoubleBuffer template | **Use existing SeqLock** | Avoid new abstraction when codebase already has SeqLock |
| kSafePosition = "design rule violation" | **Code duplication** | integrated_bringup is robot-specific package, hardcoding allowed |
| trajectory_speed = 0 "crash" | **Hang** | IEEE 754: 1/0 = INF, not crash. Fixed anyway |
