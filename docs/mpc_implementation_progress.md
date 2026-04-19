# rtc_mpc MPC Controller Implementation Progress

> **Start date**: 2026-04-19
> **Source plan**: `mpc_controller_implementation_plan.md` v2.2 (2026-04-18)
> **Install guide**: `aligator_installation_guide.md` (2026-04-19)
> **Target**: Robot-agnostic Aligator-based KinoDynamics/FullDynamics MPC framework
> **Temporary doc**: delete at full completion (after Phase 7 acceptance).

---

## Status Legend

| Symbol | Meaning |
|--------|---------|
| ✅ | Complete |
| 🔄 | In progress |
| ⏸️ | Blocked / waiting |
| ⬜ | Not started |

---

## Environment Baseline (checked 2026-04-19)

| Component | Detected | Required | Status |
|-----------|----------|----------|--------|
| Ubuntu | 24.04.4 LTS (noble) | 22.04 / 24.04 | ✅ |
| ROS | Jazzy | Humble / Jazzy | ✅ |
| CMake | 3.28.3 | ≥ 3.22 | ✅ |
| GCC | 13.3.0 | ≥ 11 | ✅ |
| Pinocchio | 3.9.0 (`ros-jazzy-pinocchio`) | ≥ 3.4 | ✅ |
| Eigen3 | 3.4 (apt) | ≥ 3.3.7 | ✅ |
| **fmtlib** | **9.1.0 (apt)** | **≥ 10.0.0** | ❌ → source build |
| **mimalloc** | **missing** | **≥ 2.1.0** | ❌ → source build |
| **Aligator** | **missing** | **0.18.x** | ❌ → source build |

---

## Phase Plan (consolidated from v2.2)

| Phase | Scope | Package | Effort | Status |
|-------|-------|---------|--------|--------|
| **0** | Aligator toolchain + integration smoke tests | rtc_mpc | 1d | ✅ |
| **1** | RobotModelHandler + contact plan types | rtc_mpc | 1.5d | ✅ |
| **2** | PhaseManagerBase + PhaseCostConfig (abstract only) | rtc_mpc | 1.5d | ✅ |
| 3 | OCPHandlerBase + KinoDynamicsOCP + CostFactory | rtc_mpc | 3.5d | ⬜ |
| 4 | FullDynamicsOCPHandler | rtc_mpc | 2.5d | ⬜ |
| 5 | MPCHandler + warm-start + factory | rtc_mpc | 2.5d | ⬜ |
| 6 | MPCThread integration + MockPhaseManager | rtc_mpc | 2d | ⬜ |
| 7a | GraspPhaseManager (FSM) + phase_config.yaml | ur5e_bringup | 1.5d | ⬜ |
| 7b | MPC YAML configs + demo_wbc_controller wiring | ur5e_bringup | 0.5d | ⬜ |
| 7c | 16-DoF grasp scenario + MuJoCo E2E + perf | ur5e_bringup | 1d | ⬜ |
| **Total** | | | **~17.5d** | |

Robot-agnostic vs robot-specific boundary is enforced: `rtc_mpc` must never mention UR5e / tool0 / fingertip frames / nq=16. All such values flow through YAML + `pinocchio::Model`.

---

## Phase 0 — Aligator toolchain (COMPLETE 2026-04-19)

### Goal
Establish Aligator build pipeline + verify CMake/Pinocchio integration before any rtc_mpc code lands.

### Sub-steps

| # | Task | Status |
|---|------|--------|
| 0.1 | Build + install fmtlib 11.1.4 to `/usr/local` | ✅ |
| 0.2 | Build + install mimalloc 2.1.7 to `/usr/local` | ✅ |
| 0.3 | Build + install Aligator 0.19.0 to `/usr/local` | ✅ |
| 0.4 | `find_package(aligator)` standalone smoke test | ✅ |
| 0.5 | Minimal compile + link test (`SolverProxDDPTpl<double>`) | ✅ |
| 0.6 | Pinocchio Panda URDF load sanity | ✅ |
| 0.7 | Riccati gain matrix shape compatibility check against `rtc_mpc/feedback/` | ✅ |
| 0.8 | Update `install.sh` with MPC deps install + `verify` subcommand | ✅ (Phase 1) |

### Installed Versions
- fmtlib 11.1.4 → `/usr/local/lib/libfmt.so.11`
- mimalloc 2.1.7 → `/usr/local/lib/libmimalloc.so.2`
- Aligator 0.19.0 → `/usr/local/lib/libaligator.so.0.19.0`
- Sources retained in `~/libs/{fmt,mimalloc,aligator}` for uninstall

### Verified Behavior
- `SolverProxDDPTpl<double>` instantiates, fmt/pinocchio/Eigen headers resolve
- Panda URDF (`/usr/local/share/example-robot-data/robots/panda_description/urdf/panda.urdf`) loads with `nq=9, nv=9, njoints=10` (7 arm + 2 finger prismatic); RNEA at neutral runs
- Mock Aligator Riccati gain `K: nu × ndx` (ColMajor) converts losslessly to row-major layout expected by `rtc::mpc::RiccatiFeedback::SetGain(const double*, nu, nx)` (see §Riccati Compatibility below)

### ⚠️ CMake Constraints Discovered (must propagate to rtc_mpc CMakeLists)

Two dual-installation conflicts on this machine must be worked around in any downstream CMake:

1. **fmt version conflict**
   - apt `libfmt-dev` (9.1.0) at `/usr/lib/x86_64-linux-gnu/cmake/fmt/`
   - Source-built fmt 11.1.4 at `/usr/local/lib/cmake/fmt/`
   - On CMake 3.28 + Ubuntu 24.04, the multi-arch apt path is searched before `/usr/local`, so plain `find_package(fmt)` picks up v9.1.0 → link fails with `fmt::v11::vprint` unresolved (Aligator was built against v11).
   - **Required in consumer CMakeLists:**
     ```cmake
     find_package(fmt 10 REQUIRED HINTS /usr/local/lib/cmake/fmt)
     ```
     or set `fmt_DIR=/usr/local/lib/cmake/fmt` cache variable.

2. **hpp-fcl version conflict (ABI break)**
   - ROS Jazzy hpp-fcl 2.4.5 at `/opt/ros/jazzy/lib/x86_64-linux-gnu/cmake/hpp-fcl/`
     - `addVertices` signature: `Eigen::Matrix<double, -1, 3, 0, -1, 3>` (ColMajor, storage=0)
   - Pre-existing `/usr/local/lib/libhpp-fcl.so` (installed Jul 2024 for simple-mpc venv)
     - `addVertices` signature: `Eigen::Matrix<double, -1, 3, 1, -1, 3>` (RowMajor, storage=1) — **ABI incompatible**
   - First Aligator build picked up `/usr/local` hpp-fcl → consumer link fails because pinocchio_parsers (ROS) needs the ColMajor signature.
   - **Fix applied:** Aligator rebuilt with `-Dhpp-fcl_DIR=/opt/ros/jazzy/lib/x86_64-linux-gnu/cmake/hpp-fcl`, now links ROS hpp-fcl.
   - **Required in consumer CMakeLists** (because pinocchioConfig.cmake does transitive `find_dependency(hpp-fcl)`):
     ```cmake
     set(hpp-fcl_DIR /opt/ros/jazzy/lib/x86_64-linux-gnu/cmake/hpp-fcl CACHE PATH "" FORCE)
     ```
     must come **before** `find_package(pinocchio)` or `find_package(aligator)`.
   - Runtime: `source /opt/ros/jazzy/setup.bash` prepends ROS libdir to `LD_LIBRARY_PATH` so `libhpp-fcl.so` resolves to ROS version; colcon builds always have ROS sourced.
   - **Open question:** whether `/usr/local/lib/libhpp-fcl.so` should be removed outright (frees the conflict but may break simple-mpc venv at `~/git/simple-mpc/`). Deferred; current workaround sufficient.

### Riccati Compatibility (Sub-step 0.7)

| Attribute | Aligator ProxDDP output | `rtc::mpc::RiccatiFeedback` expectation | Compatible? |
|-----------|------------------------|----------------------------------------|-------------|
| Gain shape | `nu × ndx` where `ndx = 2·nv` | `nu × nx` where `nx = nq + nv` | ✅ for fixed-base (`nq == nv` → `ndx == nx`) |
| Storage order | Eigen default ColMajor | `SetGain(const double*)` expects row-major contiguous | Adapter: copy into `Eigen::Matrix<double, ..., RowMajor>` before passing pointer |
| Node indexing | Per-horizon-node gain | Nearest-neighbour lookup | ✅ Matches doc in `riccati_feedback.hpp:10-12` |
| Δx order | `[dq; dv]` tangent-space | `[q_curr - q_ref; v_curr - v_ref]` config-space | ✅ for fixed-base (Lie derivative trivial) |
| Feedback mode | Full `u_fb = K·Δx + u_ff` | `accel_only=true` (default) writes first `nv` rows only | ✅ Consumer controls via `SetAccelOnly` |

**Verdict:** no interface changes needed in `rtc_mpc::RiccatiFeedback`. A thin adapter in the MPC thread (Phase 5/6) that copies Aligator's ColMajor gain into a RowMajor buffer before `SetGain()` is sufficient.

### Verification Artifacts
- `/tmp/aligator_verify/find_test/` — `find_package(aligator)` smoke test
- `/tmp/aligator_verify/compile_test/` — minimal compile + `SolverProxDDP` instantiation
- `/tmp/aligator_verify/panda_test/` — Panda URDF load + RNEA + Riccati gain layout check
- (all compile with ROS sourced, minimal passes 4/4 checks)

### Risks (from plan §11) — Status Update
- #1 Aligator KinoDynamics API fixed-base support — still unverified, re-check in Phase 3
- #5 Riccati gain shape mismatch — **Resolved**: ColMajor → RowMajor adapter plan documented

---

## Phase 1 — RobotModelHandler + Types (COMPLETE 2026-04-19)

### Goal
Robot-agnostic wrapper around `pinocchio::Model` + contact plan types. All DoF/frame info flows from Pinocchio + YAML; no hardcoded robot names, joint counts, or frame strings.

### Files Delivered
| Path | Kind | Status |
|------|------|--------|
| `rtc_mpc/include/rtc_mpc/types/contact_plan_types.hpp` | new | ✅ |
| `rtc_mpc/include/rtc_mpc/model/robot_model_handler.hpp` | new | ✅ |
| `rtc_mpc/src/model/robot_model_handler.cpp` | new | ✅ |
| `rtc_mpc/test/test_robot_model_handler.cpp` | new (Panda 9-DoF, 9 cases) | ✅ |
| `rtc_mpc/CMakeLists.txt` | edit — pinocchio + fmt 10+ + hpp-fcl workarounds | ✅ |
| `rtc_mpc/package.xml` | edit — `<depend>pinocchio</depend>` | ✅ |
| `install.sh` | edit — `install_mpc_deps()` + `verify` subcommand (§0.8) | ✅ |

### Key Types (shipped)
- `ContactFrameInfo { int frame_id; int dim; std::string name; }` — non-RT.
- `ContactPhase { std::vector<int> active_frame_ids; double t_start, t_end; }` — non-RT.
- `ContactPlan { std::vector<ContactFrameInfo> frames; std::vector<ContactPhase> phases; }` — non-RT (headers note RT interchange goes via `mpc_solution_types.hpp`).
- `RobotModelHandler::Init(const pinocchio::Model&, const YAML::Node&) → RobotModelInitError` — no-throw, resolves frame-name strings to Pinocchio ids once at init.

### Verified Behavior
- Build: `./build.sh -p rtc_mpc` → `Finished <<< rtc_mpc [15.9s]` (pinocchio + fmt 11 linked successfully, hpp-fcl workaround effective).
- Tests: `colcon test --packages-select rtc_mpc` → **9/9 passed** (previous 7 + new `test_robot_model_handler` with 8 fixture cases + 1 standalone case).
- `RobotModelHandler` on Panda: `nq()==9, nv()==9, nu()==9, n_contacts()==2` for left/right finger.
- Error-path coverage: missing EE frame → `kMissingEndEffectorFrame`, missing contact → `kMissingContactFrame`, dim ∉ {3,6} → `kInvalidContactDim`, missing top-level key → `kInvalidYamlSchema`, double-init → `kModelAlreadyInitialised`. All no-throw.
- Uninitialised handler: dim accessors return 0, `FrameId()` returns `nullopt` (safe).
- Robot-agnostic audit: `grep -E '\\b(UR5e|ur5e|tool0|fingertip|panda)\\b' rtc_mpc/include rtc_mpc/src` → only pre-existing rationale comment in `mpc_solution_types.hpp:21` (Phase 0 scope); test file matches are intentional (Panda = generic fixture).

### install.sh §0.8 — Delivered
- New CLI: `./install.sh verify` runs prereqs + `verify_mpc_deps` + workspace package check, skips deps/build/RT.
- New flag: `--skip-mpc` opts out of source-building fmt/mimalloc/aligator (CI path).
- New functions: `install_fmt_from_source` (11.1.4), `install_mimalloc_from_source` (2.1.7), `install_aligator_from_source` (0.19.0 with `-Dhpp-fcl_DIR=/opt/ros/$ROS_DISTRO/...` + `-Dfmt_DIR=/usr/local/...`), `install_mpc_deps` wrapper, `verify_mpc_deps` artifact checker.
- Idempotency: re-running on a host with the libraries already at `/usr/local/lib/lib{fmt,mimalloc,aligator}.so.<ver>` logs "already installed" and skips clone/configure/make.
- User-facing: sudo credential prompt handled by `sudo cmake --install` (same pattern as `install_pinocchio`); verified on this machine via `./install.sh verify` — 4/4 artifacts (fmt + mimalloc + aligator + Panda URDF) detected.

### Cross-Phase Invariants Upheld
1. **Robot-agnostic**: no UR5e / tool0 / finger / panda identifiers in `rtc_mpc/{include,src}`. Example YAML in header uses `<ee_frame_name>` placeholder.
2. **RT-safety**: `RobotModelHandler::Init` uses YAML-cpp (non-RT). The class is called on OCP build path, not the 500 Hz loop. Accessors (`nq()`, `FrameId`) are `noexcept` and trivially const.
3. **Interface-first**: `RobotModelInitError` enum documents all failure modes before consumer code lands.
4. **CMake hygiene**: Phase-0 hpp-fcl + fmt workarounds now live in `rtc_mpc/CMakeLists.txt` and will propagate to any package depending on `rtc_mpc` via `ament_export_dependencies`.
5. **Config-driven**: YAML schema is the single source of frame names; Pinocchio provides dims.

### Risks (from plan §11) — Status Update
- #5 Riccati gain shape mismatch — still Resolved (Phase 0).
- No new risks introduced in Phase 1.

---

## Phase 2 — PhaseManagerBase + PhaseCostConfig (COMPLETE 2026-04-19)

### Goal
Ship only the **abstract** FSM interface + **generic** cost container. Concrete FSM (APPROACH/CLOSURE/etc.) must NOT appear in `rtc_mpc`.

### Files Delivered
| Path | Kind | Status |
|------|------|--------|
| `rtc_mpc/include/rtc_mpc/phase/phase_cost_config.hpp` | new | ✅ |
| `rtc_mpc/include/rtc_mpc/phase/phase_context.hpp` | new (split header) | ✅ |
| `rtc_mpc/include/rtc_mpc/phase/phase_manager_base.hpp` | new (pure-virtual) | ✅ |
| `rtc_mpc/src/phase/phase_cost_config.cpp` | new — YAML factory | ✅ |
| `rtc_mpc/test/test_phase_cost_config.cpp` | new (10 cases on Panda) | ✅ |
| `rtc_mpc/config/mpc_default.yaml` | new (reference template) | ✅ |
| `rtc_mpc/CMakeLists.txt` | edit — source + test target + config install | ✅ |

### Interface Shape (shipped)
```cpp
struct PhaseContext {
  int phase_id{0};
  std::string phase_name{};
  bool phase_changed{false};
  ContactPlan contact_plan{};
  PhaseCostConfig cost_config{};
  std::string ocp_type{"kinodynamics"};   // "kinodynamics" | "fulldynamics"
  pinocchio::SE3 ee_target{pinocchio::SE3::Identity()};
};

class PhaseManagerBase {
public:
  virtual ~PhaseManagerBase() = default;
  virtual void Init(const YAML::Node& cfg) = 0;
  virtual PhaseContext Update(const Eigen::VectorXd& q,
                              const Eigen::VectorXd& v,
                              const Eigen::VectorXd& sensor,
                              const pinocchio::SE3& tcp, double t) = 0;
  virtual void SetTaskTarget(const YAML::Node& target) = 0;
  [[nodiscard]] virtual int CurrentPhaseId() const = 0;
  [[nodiscard]] virtual std::string CurrentPhaseName() const = 0;
  virtual void ForcePhase(int phase_id) = 0;
};
```
**Naming deviation from v2.2 plan:** method names use CamelCase (`Init`/`Update`/…) to match `RobotModelHandler::Init` and `RiccatiFeedback::SetGain` conventions already in rtc_mpc. Deliberate; decided 2026-04-19.

### PhaseCostConfig (shipped)
- Scalars: `w_frame_placement`, `w_state_reg`, `w_control_reg`, `w_contact_force`, `w_centroidal_momentum` (all ≥ 0 enforced)
- Vectors: `W_placement` (fixed `Eigen::Matrix<double,6,1>`), `q_posture_ref` (nq), `F_target` (Σ contact dims)
- Timing: `horizon_length` (> 0), `dt` (> 0)
- Extension point: `std::map<std::string,double> custom_weights` + `CustomWeight(key)` lookup returning 0.0 for absent keys
- Factory signature (per user decision on coupling): `static PhaseCostConfigError LoadFromYaml(const YAML::Node&, const RobotModelHandler&, PhaseCostConfig& out) noexcept` — model is the single source of `nq` / `Σ dim` for dimension validation.

### Error Enum (shipped)
`PhaseCostConfigError`: `kNoError`, `kModelNotInitialised`, `kInvalidYamlSchema`, `kInvalidWeightSign`, `kInvalidHorizon`, `kInvalidDt`, `kPostureRefDimMismatch`, `kForceTargetDimMismatch`, `kPlacementWeightDimMismatch`.

### Verified Behavior
- Build: `./build.sh -p rtc_mpc` → `Finished <<< rtc_mpc [14.4s]`, no warnings.
- Tests: `colcon test --packages-select rtc_mpc` → **10/10** gtest targets pass (Phase 1's 9/9 plus new `test_phase_cost_config` with 10 cases: round-trip, absent custom_weights, present/absent lookup, 6 dim/sign error paths, uninitialised-model rejection).
- Robot-agnostic audit: `grep -rnE '\b(APPROACH|CLOSURE|HOLD|RETREAT|RELEASE|PRE_GRASP|MANIPULATE|UR5e|ur5e|tool0|fingertip|panda)\b' rtc_mpc/{include,src}` → only the pre-existing Phase-0 rationale comment at `mpc_solution_types.hpp:21`. Tests legitimately mention `panda_*` frames (generic fixture).
- `PhaseCostConfig` is a passive POD container; all FSM logic deferred to Phase 7 `GraspPhaseManager`.

### Exit Criteria — Met
- ✅ `rtc_mpc` source tree: no `APPROACH`, `CLOSURE`, `UR5e`, `finger_*_tip`, `tool0`, `fingertip`, `panda` identifiers outside the pre-existing Phase-0 comment + test fixtures.
- ✅ YAML parsing test: all numeric weights + `custom_weights` round-trip correctly (ValidYamlRoundTrip test).
- ✅ `custom_weights` absent key → 0.0 default, no throw (AbsentCustomWeightsDefaultsToEmptyMap, CustomWeightLookupPresentAndAbsent).

### Cross-Phase Invariants Upheld
1. **Robot-agnostic**: new files use `<ee_frame_name>` / `<contact_frame_0>` placeholders only; `phase_id` / `phase_name` are opaque integers/strings to rtc_mpc.
2. **RT-safety**: `PhaseCostConfig` lives on the OCP build / reconfigure path, not the 500Hz loop — `Eigen::VectorXd` and `std::map` are permitted here, documented in the header. `CustomWeight` constructs a `std::string` for lookup (non-RT call site only).
3. **Interface-first**: `PhaseManagerBase` ships with zero concrete implementers in rtc_mpc — first concrete derived class lives in Phase 6 (MockPhaseManager test-only) then Phase 7 (`GraspPhaseManager` in ur5e_bringup).
4. **CMake hygiene**: Phase-0 workarounds (hpp-fcl_DIR, fmt HINTS) preserved. Added `install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)` so downstream packages can locate `mpc_default.yaml`.
5. **Config-driven**: all dimensions flow from `RobotModelHandler::nq()` + `contact_frames()[i].dim`; zero hardcoded joint counts or contact counts in `rtc_mpc/{include,src}`.

### Risks (from plan §11) — Status Update
- No new risks introduced in Phase 2 (interface-only phase; no solver/dynamics coupling yet).
- Phase 3 starts with the open §11 #1 (fixed-base KinoDynamics) + #2 (contact-force residual) — both unchanged from Phase 0 status.

---

## Phase 3 — OCPHandlerBase + KinoDynamicsOCP + CostFactory (3.5d, rtc_mpc) ⭐

### Goal
Core OCP build pipeline: `PhaseCostConfig → Aligator stages/costs/constraints`. This is the largest and most novel phase — budget generously.

### Files
| Path | Kind |
|------|------|
| `rtc_mpc/include/rtc_mpc/ocp/ocp_handler_base.hpp` | new |
| `rtc_mpc/include/rtc_mpc/ocp/cost_factory.hpp` | new |
| `rtc_mpc/src/ocp/cost_factory.cpp` | new |
| `rtc_mpc/include/rtc_mpc/ocp/kinodynamics_ocp.hpp` | new |
| `rtc_mpc/src/ocp/kinodynamics_ocp.cpp` | new |
| `rtc_mpc/test/test_kinodynamics_ocp.cpp` | new (Panda offline solve) |

### Scope
- `OCPHandlerBase`: build `aligator::TrajOptProblem` given state/config/contact plan
- `CostFactory`: map `PhaseCostConfig` → `QuadraticResidualCost` instances (state reg, control reg, frame placement, frame velocity, centroidal, contact force)
- `KinoDynamicsOCP`: `x = [q; v] ∈ R^{nq+nv}`, `u = [τ; f_c] ∈ R^{nu+3K}`, `aligator::dynamics::KinodynamicsFwdDynamicsTpl<double>`
- Constraints: friction cone (N-facet), joint box, torque box
- `custom_weights` keys handled generically: lookup in config, build residual for `joint_range` subset (joint_range comes from robot-specific YAML)

### Exit Criteria
- Panda offline solve: residual `< 1e-4` after ≤ 30 iterations
- Solve p50 < 5ms, p99 < 10ms on laptop
- No `auto` with Eigen expressions anywhere (CLAUDE.md rule 5)
- No `new`/`malloc` inside solve path

### Open Risks
- **§11 #1**: Aligator `KinodynamicsFwdDynamicsTpl` fixed-base support — if API requires floating-base, adapt via single floating-base joint or use `MultibodyConstraintFwdDynamics` fallback. Must verify in first 2 hours of Phase 3.
- **§11 #2**: Contact force cost residual formulation (`||f_ci - f_target||²`) — check Aligator has a first-class residual or must author one.

---

## Phase 4 — FullDynamicsOCP (2.5d, rtc_mpc)

### Goal
Add second dynamics mode for contact-rich phases (grasp closure, hold).

### Files
| Path | Kind |
|------|------|
| `rtc_mpc/include/rtc_mpc/ocp/fulldynamics_ocp.hpp` | new |
| `rtc_mpc/src/ocp/fulldynamics_ocp.cpp` | new |
| `rtc_mpc/test/test_fulldynamics_ocp.cpp` | new |

### Scope
- `x = [q; v]`, `u = τ`; internal `M(q)·a + h = τ + Jcᵀ·λ` via `aligator::dynamics::MultibodyConstraintFwdDynamics`
- Share state-space layout with KinoDynamics (same `nq+nv`) so warm-start transfers between modes on phase switch
- Reuse `CostFactory` unchanged

### Exit Criteria
- Panda offline solve comparable residual to Phase 3
- Solve p50 < 15ms, p99 < 30ms
- Warm-start reuse test: run KinoDyn → FullDyn with same `x` sequence, verify solver iteration count drops ≥ 40% vs cold

---

## Phase 5 — MPCHandler + Warm-Start + Factory (2.5d, rtc_mpc)

### Goal
Wrap the two OCP handlers in a runtime-switchable `MPCHandler`, add horizon-shift warm-starting.

### Files
| Path | Kind |
|------|------|
| `rtc_mpc/include/rtc_mpc/handler/mpc_handler_base.hpp` | new |
| `rtc_mpc/include/rtc_mpc/handler/kinodynamics_mpc.hpp` | new |
| `rtc_mpc/src/handler/kinodynamics_mpc.cpp` | new |
| `rtc_mpc/include/rtc_mpc/handler/fulldynamics_mpc.hpp` | new |
| `rtc_mpc/src/handler/fulldynamics_mpc.cpp` | new |
| `rtc_mpc/include/rtc_mpc/handler/mpc_factory.hpp` | new |
| `rtc_mpc/src/handler/mpc_factory.cpp` | new |
| `rtc_mpc/test/test_kinodynamics_mpc.cpp` | new |
| `rtc_mpc/test/test_fulldynamics_mpc.cpp` | new |

### Scope
- `solve(PhaseContext, x_current) → MPCSolution` (`q`, `v`, `τ`, `λ`, `K`)
- Riccati gain extraction: for each horizon node, call Aligator's `results.controlFeedbacks()[k]` (ColMajor) → copy into RowMajor buffer matching `rtc::mpc::RiccatiFeedback::SetGain` (see Phase 0 §Riccati Compatibility)
- Warm-start: shift previous solution by one node; on phase change, if `ocp_type` unchanged just resize horizon; if changed, state-copy + reset multipliers
- `MPCFactory::create(YAML::Node)` dispatches on `ocp_type`

### Exit Criteria
- KinoDyn → KinoDyn warm-start: solver iters drop ≥ 50%
- KinoDyn → FullDyn switch mid-sequence: solve still converges, p99 bounded
- No heap alloc inside `solve()` after first call (verify with tracer test)

---

## Phase 6 — MPCThread integration + MockPhaseManager (2d, rtc_mpc)

### Goal
Wire `PhaseManagerBase` into existing `rtc_mpc::MPCThread`; prove full pipeline works without any robot-specific code via a `MockPhaseManager`.

### Files
| Path | Kind |
|------|------|
| `rtc_mpc/include/rtc_mpc/thread/mpc_thread.hpp` | edit — add `set_phase_manager(std::unique_ptr<PhaseManagerBase>)` |
| `rtc_mpc/src/thread/mpc_thread.cpp` | edit — solve loop: `state → manager->update → handler->solve → TripleBuffer` |
| `rtc_mpc/test/mock_phase_manager.hpp` | new (test-only, 2-phase FSM: contact-off ↔ contact-on) |
| `rtc_mpc/test/test_mpc_thread_integration.cpp` | new — Panda + MockPhaseManager E2E |

### Scope
- Keep existing `MockMPCThread` usable as a no-solver baseline
- New solve loop calls `phase_manager_->update()` → if `phase_changed`, reconfigure handler → `handler_->solve()` → publish via `SolutionManager`
- Error path: `phase_manager_` null → log once + skip solve (don't crash)

### Exit Criteria
- `colcon test --packages-select rtc_mpc` passes with real solver
- `MockPhaseManager` transitions phase mid-run → handler picks up new `PhaseContext` within 1 tick
- No UR5e / hand / fingertip references in `rtc_mpc/` (enforced by grep in CI idea: document but defer)

---

## Phase 7 — ur5e_bringup integration (3d total)

### 7a — GraspPhaseManager FSM (1.5d)

| Path | Kind |
|------|------|
| `ur5e_bringup/include/ur5e_bringup/phase/grasp_target.hpp` | new |
| `ur5e_bringup/include/ur5e_bringup/phase/grasp_phase_manager.hpp` | new |
| `ur5e_bringup/src/phase/grasp_phase_manager.cpp` | new |
| `ur5e_bringup/config/controllers/phase_config.yaml` | new |
| `ur5e_bringup/test/test_grasp_phase_manager.cpp` | new |

- 7-phase FSM: `IDLE → APPROACH → PRE_GRASP → CLOSURE → HOLD → MANIPULATE → RETREAT → RELEASE`
- Transition thresholds YAML: `approach_tolerance`, `pregrasp_tolerance`, `force_threshold`, `max_failures`
- `custom_weights["hand_posture"]` activates hand joint subset cost in rtc_mpc
- `ContactPlan`: fingertip frame IDs on/off per phase

### 7b — MPC YAML + Controller wiring (0.5d)

| Path | Kind |
|------|------|
| `ur5e_bringup/config/controllers/mpc_kinodynamics.yaml` | new (ee_frame=tool0, contact_frames=[finger_0_tip,...], reference_config=[16 joints]) |
| `ur5e_bringup/config/controllers/mpc_fulldynamics.yaml` | new |
| `ur5e_bringup/src/controllers/demo_wbc_controller.cpp` | edit — inject GraspPhaseManager; replace MockMPC with real factory |

### 7c — 16-DoF scenario + MuJoCo E2E + perf (1d)

| Path | Kind |
|------|------|
| `ur5e_bringup/test/test_ur5e_mpc_kinodynamics.cpp` | new (16-DoF SE3 reaching) |
| `ur5e_bringup/test/test_ur5e_mpc_grasp_scenario.cpp` | new (APPROACH→CLOSURE→HOLD) |

### Exit Criteria (full system)
| Metric | KinoDynamics | FullDynamics |
|--------|-------------|--------------|
| Solve p50 | < 10ms | < 25ms |
| Solve p99 | < 20ms | < 45ms |
| EE tracking RMSE | < 5mm | < 3mm |
| Phase transition latency | < 100ms | < 200ms |

After 7c passes: delete this progress doc per CLAUDE.md Post-Task Housekeeping.

---

## Cross-Phase Invariants (enforce every phase)

1. **Robot-agnostic enforcement**: grep `rtc_mpc/` for `UR5e|ur5e|tool0|finger|hand|panda|nq = 16` — must be empty (Panda is only referenced in test files as a generic N-DoF example, ideally via `example-robot-data` path constant).
2. **RT-safety on `solve()` path**: no `new`/`malloc`/`throw`/`std::mutex::lock` inside the MPC thread's solve loop (see CLAUDE.md Hard Rules + `.claude/rules/rt-safety.md`).
3. **Interface-first**: every new concrete class must inherit from a pure-virtual base already present or introduced in the same phase.
4. **CMake hygiene**: every `rtc_mpc/CMakeLists.txt` edit preserves Phase-0 workarounds (hpp-fcl_DIR, fmt HINTS).
5. **Config-driven**: no joint counts, frame names, or topic names baked into `rtc_mpc` C++ — all via YAML or Pinocchio Model.

---

## Resumption Notes (for the next conversation)

When resuming:

1. **Read this doc first** — full environment + workaround context.
2. **Reference docs (provided as attachments in the original session):**
   - `mpc_controller_implementation_plan.md` v2.2 — full phase breakdown (re-attach if needed)
   - `aligator_installation_guide.md` — toolchain guide (Phase 0 superseded it with documented workarounds above)
3. **Starting point:** Phase 3 — `OCPHandlerBase` + `KinoDynamicsOCP` + `CostFactory`. The largest phase (3.5d); budget generously and verify Aligator `KinodynamicsFwdDynamicsTpl` fixed-base support in the first 2 hours (risk §11 #1). Phase 2 shipped `PhaseManagerBase` + `PhaseCostConfig` + `PhaseContext`; Phase 3 will consume `PhaseContext.cost_config` and `RobotModelHandler` directly.
4. **Test robot:** Panda URDF at `/usr/local/share/example-robot-data/robots/panda_description/urdf/panda.urdf` (nq=9, nv=9). Do **not** use UR5e in `rtc_mpc/test/` — ur5e-specific tests belong in `ur5e_bringup/test/` (Phase 7).
5. **CMake hygiene for `rtc_mpc/CMakeLists.txt`** (see Phase 0 §"CMake Constraints"):
   ```cmake
   set(hpp-fcl_DIR /opt/ros/jazzy/lib/x86_64-linux-gnu/cmake/hpp-fcl CACHE PATH "" FORCE)
   find_package(fmt 10 REQUIRED HINTS /usr/local/lib/cmake/fmt)
   find_package(aligator REQUIRED)
   find_package(pinocchio REQUIRED)
   ```
6. **Verification sandboxes** under `/tmp/aligator_verify/` are scratch — do not rely on them persisting across reboots. Source is at `~/libs/{fmt,mimalloc,aligator}`.
7. **Delete this doc** only after Phase 7c passes end-to-end acceptance.

---

## Change Log

| Date | Entry |
|------|-------|
| 2026-04-19 | Doc created, environment baseline captured, Phase 0 started |
| 2026-04-19 | Phase 0 complete: fmt 11.1.4 + mimalloc 2.1.7 + Aligator 0.19.0 installed; 4 smoke tests pass; CMake dual-install conflicts (fmt, hpp-fcl) documented with required workarounds for rtc_mpc |
| 2026-04-19 | Phase 1 complete: `RobotModelHandler` + `contact_plan_types.hpp` landed in rtc_mpc (Panda 9-DoF test, 9/9 pass). install.sh §0.8 delivered: `install_mpc_deps` + `verify` subcommand + `--skip-mpc` flag. Robot-agnostic invariant verified. |
| 2026-04-19 | Phase 2 complete: `PhaseManagerBase` (pure-virtual), `PhaseCostConfig` (POD + YAML factory via `RobotModelHandler`), `PhaseContext`, `mpc_default.yaml` reference template. 10/10 test_phase_cost_config cases pass. Method naming uses CamelCase (`Init`/`Update`) — deliberate deviation from v2.2 plan for consistency with existing rtc_mpc conventions. Robot-agnostic invariant re-verified. |
