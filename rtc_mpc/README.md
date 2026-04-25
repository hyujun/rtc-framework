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
| `ocp/` | `ocp_handler_base.hpp` | Abstract OCP builder (`Build` / `UpdateReferences`); `OCPLimits` (control box / friction μ) and `OCPBuildError` enum |
| `ocp/` | `cost_factory.hpp` | Builds per-stage `aligator::CostStack` (frame placement + state reg + control reg), weight-gated, no-throw |
| `ocp/` | `light_contact_ocp.hpp` | Concrete `OCPHandlerBase` backed by `MultibodyConstraintFwdDynamicsTpl` (fixed-base `u = τ`); alloc-free `UpdateReferences` via cached polymorphic residual handles. Dispatch key `"light_contact"`. (Renamed from `kinodynamics_ocp.hpp` in Phase 4.-1.) |
| `ocp/` | `contact_rich_ocp.hpp` | Concrete `OCPHandlerBase` adding per-active-contact `ContactForceResidualTpl` cost + `MultibodyFrictionConeResidualTpl` / `NegativeOrthantTpl` inequality. Dispatch key `"contact_rich"`. Phase 4 header; `.cpp` lands in Step 4. Cold-start requires caller-side seeding (see class doc-comment). |
| `ocp/` | `grasp_quality_provider.hpp` | Pure-virtual extension seam for grasp-quality residuals on `ContactRichOCP` running/terminal stages. No concrete provider ships in Phase 4 — first implementation lands in Phase 4.5+ alongside a real consumer. |
| `comm/` | `triple_buffer.hpp` | Lock-free triple buffer with zero-copy consumer acquire |
| `interpolation/` | `trajectory_interpolator.hpp` | Cubic Hermite interpolation between OCP nodes |
| `feedback/` | `riccati_feedback.hpp` | `u_fb = gain_scale · K · Δx` with optional accel-only mode |
| `manager/` | `mpc_solution_manager.hpp` | Facade combining TripleBuffer + Interpolator + Feedback + SeqLock |
| `thread/` | `mpc_thread.hpp` | `MPCThread` base (jthread + worker frame) and `MockMPCThread` |
| `thread/` | `handler_mpc_thread.hpp` | Concrete `MPCThread` wiring a `PhaseManagerBase` FSM into an `MPCHandlerBase` solver: per-tick FK → `phase_manager.Update` → `handler.Solve` → `PublishSolution`; cross-mode swap via `MPCFactory` + `SeedWarmStart`; observability atomics |
| `handler/` | `mpc_handler_base.hpp` | Abstract MPC solve orchestrator: owns an `OCPHandlerBase` + `SolverProxDDP`, drives warm-started solves via `Init` / `Solve(PhaseContext, state, MPCSolution&)` / `SeedWarmStart`. Enums `MPCInitError`, `MPCSolveError`, POD `MPCSolverConfig`. |
| `handler/` | `light_contact_mpc.hpp` | Concrete `MPCHandlerBase` wrapping `LightContactOCP`. |
| `handler/` | `contact_rich_mpc.hpp` | Concrete `MPCHandlerBase` wrapping `ContactRichOCP`; forwards the grasp-quality provider seam. |
| `handler/` | `mpc_factory.hpp` | YAML-driven static factory: `Create(cfg, model, initial_ctx, &handler_out) → MPCFactoryStatus` dispatching on `ocp_type`. |

## Dependencies

```
rtc_mpc ← rtc_base (SeqLock, threading), Eigen3, yaml-cpp,
          Pinocchio (robot model), fmt ≥ 10 (Aligator ABI),
          Aligator 0.19.x (ProxDDP solver, residuals, stages)
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

## Observability (HandlerMPCThread)

`HandlerMPCThread::Solve` is `noexcept` and runs off the RT loop, so failure
paths log to `stderr` rather than via ROS. Each path (dim-mismatch,
cross-mode swap rebuild required, handler solve error) increments
`failed_solves_`/`total_solves_` atomics and calls `WarnThrottled(...)`
which emits one `fprintf(stderr, …)` line at most every 5 s with
`what=<cause> code=<int> total=N failed=M`. The null-handler setup error
retains its own one-shot `fprintf` (separate semantics: fatal setup
mistake, not runtime drift). Readers can also pair the stderr stream with
`<session>/controllers/<config_key>/mpc_solve_timing.csv` (writer:
[`rtc_mpc/logging/mpc_solve_timing_logger.hpp`](include/rtc_mpc/logging/mpc_solve_timing_logger.hpp);
each MPC-using controller's own LifecycleNode owns the 1 Hz aux timer)
— when MPC is enabled but Solve keeps failing, `DemoWbcController::
GetMpcSolveStats` returns a `count=0` sentinel row so the CSV still
proves the thread is alive.

## Status

| Phase | Scope | Status |
|-------|-------|--------|
| 0 | Aligator toolchain (fmt / mimalloc / aligator → /usr/local) | ✅ |
| 1 | `RobotModelHandler` + `contact_plan_types.hpp` | ✅ |
| 2 | `PhaseManagerBase` + `PhaseCostConfig` + `PhaseContext` | ✅ |
| 3 | `OCPHandlerBase` + `LightContactOCP` (renamed from `KinoDynamicsOCP` in 4.-1) + `CostFactory` + `OCPLimits` | ✅ |
| 4 | `ContactRichOCP` (contact-force cost + smooth conic friction cone) + `GraspQualityResidualProvider` seam + `test_utils/SeedGravityCompensation` | ✅ |
| 5 | `MPCHandlerBase` + `LightContactMPC` + `ContactRichMPC` + `MPCFactory` + horizon-shift warm-start (Aligator `cycleAppend`) | ✅ |
| 6 | `HandlerMPCThread` + `MockPhaseManager` (test-only) + alloc tracer (Phase 5 Exit #3 closed for LightContact; ContactRich informational) | ✅ |
| 7 | ur5e_bringup `GraspPhaseManager` + MPC YAML wiring + 16-DoF MuJoCo E2E | ⬜ |

See `docs/mpc_implementation_progress.md` for the living roadmap.
