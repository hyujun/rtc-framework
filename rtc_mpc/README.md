# rtc_mpc

**MPC ↔ RT control interface layer** for the RTC framework.

Robot-agnostic library providing the plumbing between a soft-RT MPC thread
(20 Hz) and the hard-RT control loop (500 Hz): lock-free solution delivery,
cubic Hermite trajectory interpolation, Riccati feedback, and an MPC thread
skeleton. Concrete solver integrations (Aligator ProxDDP) plug in via
`rtc::mpc::MPCThread` + `PhaseManagerBase` (Phase 2+).

## Module map

| Module | Header | Role |
|--------|--------|------|
| `types/` | `mpc_solution_types.hpp` | `MPCSolution`, `MPCStateSnapshot` (trivially copyable, fixed capacity) |
| `types/` | `contact_plan_types.hpp` | `ContactFrameInfo`, `ContactPhase`, `ContactPlan` (non-RT, OCP build path) |
| `model/` | `robot_model_handler.hpp` | Robot-agnostic wrapper over `pinocchio::Model` + YAML frame resolution |
| `phase/` | `phase_manager_base.hpp` | Pure-virtual FSM boundary (`Init`/`Update`/`SetTaskTarget`/`ForcePhase`); concrete impls live in downstream bringup packages |
| `phase/` | `phase_cost_config.hpp` | POD cost container (scalars, `W_placement`, `q_posture_ref`, `F_target`, `custom_weights`) + no-throw YAML factory |
| `phase/` | `phase_context.hpp` | `PhaseContext` bundle passed from manager to OCP builder (contact plan + cost config + ee target + `ocp_type` dispatch key) |
| `comm/` | `triple_buffer.hpp` | Lock-free triple buffer with zero-copy consumer acquire |
| `interpolation/` | `trajectory_interpolator.hpp` | Cubic Hermite interpolation between OCP nodes |
| `feedback/` | `riccati_feedback.hpp` | `u_fb = gain_scale · K · Δx` with optional accel-only mode |
| `manager/` | `mpc_solution_manager.hpp` | Facade combining TripleBuffer + Interpolator + Feedback + SeqLock |
| `thread/` | `mpc_thread.hpp` | `MPCThread` base (jthread + worker frame) and `MockMPCThread` |

## Dependencies

```
rtc_mpc ← rtc_base (SeqLock, threading), Eigen3, yaml-cpp,
          Pinocchio (robot model), fmt ≥ 10 (Aligator ABI)
```

`rtc_mpc` does **not** depend on `rtc_tsid`. Downstream controllers
(e.g. `ur5e_bringup::DemoWbcController`) inject MPC-generated references
into TSID tasks themselves.

### CMake workarounds (dual-install conflicts on dev machine)

`rtc_mpc/CMakeLists.txt` forces ROS-Jazzy hpp-fcl and source-built fmt
before any `find_package(pinocchio)` call. See
`docs/mpc_implementation_progress.md` §Phase 0 "CMake Constraints" for
rationale; any downstream package linking `rtc_mpc` inherits the
workarounds via `ament_export_dependencies`.

## Design invariants

- **Robot-agnostic**: no fixed DoF, no robot names, no hardcoded frame
  strings. All topology flows from `pinocchio::Model` + YAML config.
  Panda is used as a *generic* N-DoF test fixture only; any UR5e-specific
  integration lives in `ur5e_bringup`.
- **Trivially copyable RT data**: `MPCSolution` and `MPCStateSnapshot`
  travel through `SeqLock` / `TripleBuffer`, so no dynamic members. OCP
  build-path types (`ContactPlan`, etc.) may use `std::vector`.
- **Zero-copy consumer**: `TripleBuffer::try_acquire_latest()` returns a
  `const T*` — no memcpy on the RT path.
- **No-throw init**: `RobotModelHandler::Init` returns
  `RobotModelInitError`; never throws.
- **Fixed-base only**: Cubic Hermite assumes Euclidean `q` (revolute
  joints). Floating-base quaternion interpolation is out of scope.

## Status

| Phase | Scope | Status |
|-------|-------|--------|
| 0 | Aligator toolchain (fmt / mimalloc / aligator → /usr/local) | ✅ |
| 1 | `RobotModelHandler` + `contact_plan_types.hpp` | ✅ |
| 2 | `PhaseManagerBase` + `PhaseCostConfig` + `PhaseContext` | ✅ |
| 3 | `OCPHandlerBase` + `KinoDynamicsOCP` + `CostFactory` | ⬜ |
| 4-7 | FullDynamics, `MPCHandler`, thread integration, ur5e hook-up | ⬜ |

See `docs/mpc_implementation_progress.md` for the living roadmap.
