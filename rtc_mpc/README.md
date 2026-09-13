# rtc_mpc

**MPC ↔ RT control interface layer** for the RTC framework.

Robot-agnostic library providing the plumbing between a soft-RT MPC thread
(typ. 20 Hz) and the hard-RT control loop (`control_rate`, default 500 Hz,
design 100 Hz–5 kHz): lock-free solution delivery,
cubic Hermite trajectory interpolation, Riccati feedback, and an MPC thread
skeleton. Concrete solver integrations (Aligator ProxDDP) plug in via
`rtc::mpc::MPCThread` + `PhaseManagerBase`.

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
| `ocp/` | `contact_light_ocp.hpp` | Concrete `OCPHandlerBase` backed by `MultibodyConstraintFwdDynamicsTpl` (fixed-base `u = τ`); alloc-free `UpdateReferences` via cached polymorphic residual handles. Dispatch key `"contact_light"`. |
| `ocp/` | `contact_rich_ocp.hpp` | Concrete `OCPHandlerBase` adding per-active-contact `ContactForceResidualTpl` cost + `MultibodyFrictionConeResidualTpl` / `NegativeOrthantTpl` inequality. Dispatch key `"contact_rich"`. Cold-start requires caller-side seeding (see class doc-comment). |
| `ocp/` | `grasp_quality_provider.hpp` | Pure-virtual extension seam for grasp-quality residuals on `ContactRichOCP` running/terminal stages. No concrete provider ships yet; the seam awaits a real consumer. |
| `comm/` | `triple_buffer.hpp` | Lock-free triple buffer with zero-copy consumer acquire |
| `interpolation/` | `trajectory_interpolator.hpp` | Cubic Hermite interpolation between OCP nodes |
| `feedback/` | `riccati_feedback.hpp` | `u_fb = gain_scale · K · Δx` with optional accel-only mode |
| `manager/` | `mpc_solution_manager.hpp` | Facade combining TripleBuffer + Interpolator + Feedback + SeqLock |
| `logging/` | `mpc_timing_logger.hpp` | `MpcTimingLogger` — thin wrapper resolving `<session>/timing/mpc_timing_log.csv` and pre-binding the unified `RtTickTimingPayload` header/row writers (shared 8-col schema with the CM RT loop / hand_udp EventLoop: `t_wall_ns,tick_count,run_id` from the logger + 5 payload columns; `run_id` separates restarts that share a session directory) |
| `thread/` | `mpc_thread.hpp` | `MPCThread` solve loop: inherits `rtc::PeriodicRtThread` for lifecycle / `clock_nanosleep` cadence / Pause/Resume / per-tick t0~t3 capture; subclass adds the `Solve(state, out)` virtual. `OnTick` runs `ReadState → MarkStateAcquired → Solve → MarkComputeDone → PublishSolution`. Solve is single-threaded — parallelism, if reintroduced, belongs to the solver's own OpenMP pool, not to externally owned thread handles. `MockMPCThread` is the deterministic test impl. `Pause()` / `Resume()` come from base and cv-gate the solve loop so an inactive controller can keep the thread alive without burning a core (used by `DemoWbcController::on_deactivate` under the rtc_cm lifecycle plan). |
| `thread/` | `handler_mpc_thread.hpp` | Concrete `MPCThread` wiring a `PhaseManagerBase` FSM into an `MPCHandlerBase` solver: per-tick FK → `phase_manager.Update` → `handler.Solve` → `PublishSolution`; cross-mode swap via `MPCFactory` + `SeedWarmStart`; observability atomics |
| `handler/` | `mpc_handler_base.hpp` | Abstract MPC solve orchestrator: owns an `OCPHandlerBase` + `SolverProxDDP`, drives warm-started solves via `Init` / `Solve(PhaseContext, state, MPCSolution&)` / `SeedWarmStart`. Enums `MPCInitError`, `MPCSolveError`, POD `MPCSolverConfig`. |
| `handler/` | `contact_light_mpc.hpp` | Concrete `MPCHandlerBase` wrapping `ContactLightOCP`. |
| `handler/` | `contact_rich_mpc.hpp` | Concrete `MPCHandlerBase` wrapping `ContactRichOCP`; forwards the grasp-quality provider seam. |
| `handler/` | `mpc_factory.hpp` | YAML-driven static factory: `Create(cfg, model, initial_ctx, &handler_out) → MPCFactoryStatus` dispatching on `ocp_type`. |

## Dependencies

```
rtc_mpc ← rtc_base (SeqLock, threading), Eigen3, yaml-cpp,
          Pinocchio 4.0 (robot model; collision backend is coal,
          hpp-fcl's successor -- no separate coal find_package needed,
          pinocchioConfig pulls it in via find_dependency),
          fmt >= 10 (Aligator ABI; deps/install ships 11.1.4),
          Aligator 0.19.x (ProxDDP solver, residuals, stages)
```

`rtc_mpc` does **not** depend on `rtc_tsid`. Downstream controllers
(e.g. `integrated_bringup::DemoWbcController`) inject MPC-generated references
into TSID tasks themselves.

### CMake / build notes

`fmt`, `pinocchio`, and `aligator` are isolated to `deps/install/` (see
`repo_scripts/scripts/setup_env.sh`, which prepends it to
`CMAKE_PREFIX_PATH`); no explicit `*_DIR` hints are needed.

Aligator's ProxDDP solve path allocates via mimalloc, but frees can route
through glibc across the pinocchio/aligator ABI boundary; under
Pinocchio 4.0 this surfaces as `free(): invalid pointer` in the
`contact_rich` solve tests. `CMakeLists.txt` `LD_PRELOAD`s mimalloc
(resolved via `RTC_DEPS_PREFIX` or the `aligator_DIR`-relative `lib/`)
into every aligator-solve gtest via `set_property(TEST ... ENVIRONMENT
LD_PRELOAD=...)` — the sanctioned workaround; do not deactivate it or run
the affected gtest binaries bare.

## Design invariants

- **Robot-agnostic**: no fixed DoF, no robot names, no hardcoded frame
  strings. All topology flows from `pinocchio::Model` + YAML config.
  Panda is used as a *generic* N-DoF test fixture only; any UR5e-specific
  integration lives in `integrated_bringup`.
- **Trivially copyable RT data**: `MPCSolution` and `MPCStateSnapshot`
  travel through `SeqLock` / `TripleBuffer`, so no dynamic members. OCP
  build-path types (`ContactPlan`, etc.) may use `std::vector`.
- **Zero-copy consumer**: `TripleBuffer::try_acquire_latest()` returns a
  `const T*` — no memcpy on the RT path.
- **No-throw init**: `RobotModelHandler::Init` returns
  `RobotModelInitError`; never throws.
- **Fixed-base only**: Cubic Hermite assumes Euclidean `q` (revolute
  joints). Floating-base quaternion interpolation is out of scope.
- **Explicit SE3 frames**: `RobotModelHandler` requires `end_effector_frame`
  (controlled tip) **and** `base_frame` (reference for SE3 control; F-4
  strict — silent universe fallback was removed). Missing `base_frame`
  returns `kInvalidYamlSchema`; an unresolvable name returns
  `kMissingBaseFrame`. Pass `base_frame: "universe"` (frame_id 0) to opt
  into the world-frame fast path. `HandlerMPCThread::Solve` extracts the
  per-tick TCP pose as `oMb⁻¹·oMf[ee]` (identity when base is universe)
  before passing it to the phase manager. `PhaseContext::ee_target` is
  interpreted in `base_frame`; `cost_factory::AddFramePlacement` lifts it
  to world via `base_oMf · ee_target` (captured at Init from neutral FK —
  valid because base must be a fixed frame) before constructing the
  Aligator residual. Universe base frames take a fast path that skips both
  transforms.

## Observability (HandlerMPCThread)

`HandlerMPCThread::Solve` is `noexcept` and runs off the RT loop, so failure
paths log to `stderr` rather than via ROS. Each path (dim-mismatch,
cross-mode swap rebuild required, handler solve error) increments
`failed_solves_`/`total_solves_` atomics and calls `WarnThrottled(...)`
which emits one `fprintf(stderr, …)` line at most every 5 s with
`what=<cause> code=<int> total=N failed=M`. The null-handler setup error
retains its own one-shot `fprintf` (separate semantics: fatal setup
mistake, not runtime drift). Readers can also pair the stderr stream with
`<session>/timing/mpc_timing_log.csv` (writer:
[`rtc_mpc/logging/mpc_timing_logger.hpp`](include/rtc_mpc/logging/mpc_timing_logger.hpp);
each MPC-using controller's own LifecycleNode owns a 1 Hz aux timer that
drains `MPCThread::TimingProducer()` per-tick SPSC into the CSV). Schema
is **per-MPC-tick raw** sharing the unified 7-col format with the CM RT
loop: `t_wall_ns,tick_count,t_state_us,t_compute_us,t_publish_us,t_total_us,jitter_us`,
one row per main-loop iteration. When MPC is enabled but `Solve` keeps
failing the CSV still grows one row per tick (publish phase is just
zero), so the failed-solve atomics above plus the periodic `RCLCPP_INFO`
aggregate (`MPCSolutionManager::GetSolveStats`, 256-sample sliding window
over handler-side `solve_duration_ns`) prove the thread is alive.
Backpressure: producer's `DropCount()` increments if the consumer falls
behind.

## Status

Production default engine is `handler` (`engine: "handler"` in
`integrated_bringup`'s `demo_wbc_controller.yaml`, all robots). History:
`git log --grep='rtc_mpc'`.
