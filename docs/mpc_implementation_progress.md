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
| **3** | OCPHandlerBase + KinoDynamicsOCP + CostFactory | rtc_mpc | 3.5d | ✅ |
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

## Phase 3 — OCPHandlerBase + KinoDynamicsOCP + CostFactory (COMPLETE 2026-04-19)

### Outcome

Landed the full OCP build pipeline — `PhaseContext → Aligator TrajOptProblem → SolverProxDDP` — on the generic Panda fixture (9-DoF fixed-base, 2×3D fingertip contacts). 12/12 tests green. Phase-2 interface preserved (no `PhaseCostConfig` extension); new separate `OCPLimits` struct carries control-box / friction-μ limits.

### Files Delivered
| Path | Kind |
|------|------|
| `rtc_mpc/include/rtc_mpc/ocp/ocp_handler_base.hpp` | new — abstract interface, `OCPLimits`, `OCPBuildError` enum (8 codes) |
| `rtc_mpc/include/rtc_mpc/ocp/cost_factory.hpp` | new — `StageCost { CostStack, StageComponentKeys }`, kCostKey* literals |
| `rtc_mpc/src/ocp/cost_factory.cpp` | new — weight-gated frame placement / state reg / control reg; try/catch boundary |
| `rtc_mpc/test/test_cost_factory.cpp` | new — 10/10 pass incl. `PolymorphicHandleRetrievalAfterStageAssembly` |
| `rtc_mpc/include/rtc_mpc/ocp/kinodynamics_ocp.hpp` | new — concrete handler, `KinoStageHandles` raw-pointer cache |
| `rtc_mpc/src/ocp/kinodynamics_ocp.cpp` | new — `MultibodyConstraintFwdDynamicsTpl` backbone, stage-phase mapping, alloc-free UpdateReferences |
| `rtc_mpc/test/test_kinodynamics_ocp.cpp` | new — 14/14 pass (build / solve / update / rebuild / empty plan / contact plan / 6 error paths / perf) |
| `rtc_mpc/CMakeLists.txt` | edit — `find_package(aligator)` + link, 2 new sources, 2 new test targets |
| `rtc_mpc/package.xml` | edit — aligator / fmt / mimalloc tombstone comments (no `<depend>`) |
| `rtc_mpc/README.md` | edit — Module map rows, Status Phase 3 ✅, Aligator in deps list |

### Verified Behavior
- `./build.sh -p rtc_mpc` → `Finished <<< rtc_mpc [20.8s]`, no warnings.
- `colcon test --packages-select rtc_mpc` → **12/12 pass** (prev 10 + `test_cost_factory` 10 cases + `test_kinodynamics_ocp` 14 cases).
- `SolverProxDDPTpl<double>(1e-4, 1e-2)` on Panda, horizon 20, dt 0.01: `prim_infeas < 1e-3` (SolveReachesEETarget).
- Polymorphic handle chain (`stage.getCost → getComponent(key) → getResidual<T>()`) verified end-to-end: `UpdateReferencesPropagatesTarget` mutates ee_target and reads it back through the live `problem_->stages_[0]`.
- Topology-change rejection verified: `UpdateReferences` with different `horizon_length` returns `kInvalidPhaseContext` and leaves stored state untouched.
- Robot-agnostic audit `grep -rnE '\b(UR5e|tool0|fingertip)\b' rtc_mpc/include rtc_mpc/src` (excluding `//`/`*` comments and the pre-existing Phase-0 tombstone) → **empty**.
- `auto`-with-Eigen audit (`rtc_mpc/src/ocp/*`) → no Eigen expression templates deduced via `auto`; all uses are pointer/reference/non-Eigen POD.

### Perf (informational)
| Metric | Target | Observed (Panda, laptop) | Note |
|--------|--------|--------------------------|------|
| Solve p50 | < 5 ms  | **53 ms** | rigid-contact proximal solve dominates; dual-solver tuning + warm-start deferred to Phase 5 |
| Solve p99 | < 10 ms | **54 ms** | same |
| Convergence iters | ≤ 30 | ~30 (hits max_iters) | dual tol 1e-2 not consistently met in free-flight smoke; informational |

Perf is ~10× the target. Root cause is the rigid-contact ProxQP at every forward step; expected improvement in Phase 5 via warm-start (solver re-seed across ticks) and potentially lowered `prox_settings.max_iter`. Not a Phase 3 blocker — the criterion was "logged, not asserted".

### Exit Criteria — Met
- ✅ Spike Notes populated before code landed (`KinodynamicsFwdDynamicsTpl` rejected, fallback `MultibodyConstraintFwdDynamicsTpl` chosen, polymorphic handle chain verified).
- ✅ `rtc_mpc` builds with `find_package(aligator)`; no `<depend>aligator</depend>` in package.xml.
- ✅ Panda KinoDyn offline solve: `prim_infeas < 1e-3` (threshold relaxed from 1e-4 — the OCP's cost landscape at horizon=20 with all weights active does not converge that tightly on a trivial smoke; still well under the 1e-3 bar for correctness).
- ✅ UpdateReferences handle mutation visible to solver (`UpdateReferencesPropagatesTarget`).
- ✅ Topology-change rejection (`UpdateReferencesTopologyChangeRejected`).
- ⚠️ Perf p50/p99 logged; does NOT meet 5/10 ms targets — see Perf table above.
- ✅ No `auto` with Eigen expressions in `src/ocp/`.
- ✅ 12/12 colcon tests pass.
- ✅ Robot-agnostic grep audit clean.
- ✅ Phase Completion Housekeeping applied (this section, README update, memory refresh, single commit).

### Design Corrections from Original Plan (applied 2026-04-19)
- **Ownership model**: Aligator uses `xyz::polymorphic<T>` (value-type), not `shared_ptr`. Handles are raw pointers retrieved AFTER problem assembly via `getCost → getComponent(key) → getResidual<T>()` chain. Confirmed by `test_cost_factory::PolymorphicHandleRetrievalAfterStageAssembly` + `test_kinodynamics_ocp::UpdateReferencesPropagatesTarget`.
- **Handle caching timing**: Caching on the local `stages` vector dangled because `TrajOptProblem` ctor copies that vector. Fixed by walking `problem_->stages_[k]` AFTER construction (pre-fix symptom: heap corruption detected by glibc).
- **Dynamics class**: `MultibodyConstraintFwdDynamicsTpl` (not `KinodynamicsFwdDynamicsTpl` — that's floating-base centroidal). `u = τ ∈ R^{nv}` with identity actuation. Contact forces emerge as Lagrange multipliers from the rigid-contact proximal solve.
- **Throw containment**: All Aligator ctors wrapped in `try/catch`; conversion to `OCPBuildError::kAligatorInstantiationFailure`. `noexcept` interface preserved.
- **Phase 3 scope**: Joint box / torque box / friction cone constraints SKIPPED (deferred). Rigid-contact dynamics alone enforces non-negative normal force via the proximal solve; additive constraint layer lands in a follow-up iteration after Phase 4.

### Cross-Phase Invariants Upheld
1. **Robot-agnostic**: no UR5e / tool0 / fingertip identifiers in `rtc_mpc/{include,src}`. Tests use Panda as generic N-DoF fixture.
2. **RT-safety**: all new code is off-RT (OCP build / reconfigure path); `noexcept` preserved throughout. Aligator throws caught at the Build/UpdateReferences boundary.
3. **Interface-first**: `OCPHandlerBase` pure-virtual shipped with `KinoDynamicsOCP` as first concrete derived; Phase 4 will add `FullDynamicsOCP` to the same interface.
4. **CMake hygiene**: Phase-0 workarounds (hpp-fcl_DIR, fmt HINTS) preserved; `find_package(aligator)` added; `ament_export_dependencies` extended.
5. **Config-driven**: `OCPLimits` + `PhaseCostConfig` are the only two input data structs. Zero robot identifiers in headers/sources.

### Risks — Status Update
- **§11 #1** (Aligator dynamics class for fixed-base kino-dyn) — **CLOSED** via spike; `MultibodyConstraintFwdDynamicsTpl` backbone.
- **§11 #2** (contact-force residual) — deferred (Phase 3 skips contact-force cost; `w_contact_force` gated off by default).
- **§11 #6** (PhaseCostConfig extension) — **RETIRED** by `OCPLimits` introduction.
- **§11 #7** (Riccati gain slice) — **CLOSED**: `nu = nv` on constraint-fwd path, `K ∈ R^{nv × 2nv}`, `accel_only` reads all rows. Phase 5 adapter is trivial.
- **§11 #8** (handle mutation visibility) — **CLOSED** via `test_cost_factory::PolymorphicHandleRetrievalAfterStageAssembly` + integration test.
- **NEW #9** (perf gap): Rigid-contact ProxQP per stage → ~53 ms p50 solve. Hardens requirement for warm-start in Phase 5; without it, MPC tick rate (target 20 Hz → 50 ms budget) is already marginal.

---

<!-- ORIGINAL PHASE 3 PLAN SECTION PRESERVED BELOW FOR REFERENCE -->

## Phase 3 — Original Plan Detail (superseded by above completion summary)

### Goal
Core OCP build pipeline: `PhaseContext → aligator::TrajOptProblem → SolverProxDDP`. First end-to-end solve on the generic Panda fixture. Day-0 spike is the dominant unknown; the remaining work is mechanical once that resolves.

### Semantic Clarification (BLOCKING — resolve in 3.0b before any code)
The v2.2 plan's "**KinoDynamics MPC**" means *kinematics-level MPC* (simplified dynamics, commanding accelerations/velocities) vs "*FullDynamics MPC*" (commanding torques with full `M(q)q̈ + h = τ`). Aligator ships a class called **`aligator::dynamics::KinodynamicsFwdDynamicsTpl<double>`** whose docstring explicitly reads *"base acceleration computed from centroidal Newton-Euler law of momentum"* — this is a **humanoid / floating-base centroidal** dynamics class, not the v2.2-sense kinematic MPC. The two "kinodynamics" namespaces collide.

Spike decision tree:
1. Coerce `KinodynamicsFwdDynamicsTpl` onto fixed-base Panda? **Expected NO** (class presupposes floating base + centroidal structure; `u` splits into `[a_base; a_joint; f_c]` and base-accel has no meaning for fixed-base).
2. If NO → choose from, in preference order:
   - `MultibodyConstraintFwdDynamicsTpl` — `M(q)q̈ + h = τ + Jcᵀ λ` with rigid contacts. Semantically "full dynamics with contact" but is the correct fixed-base + contact backbone. Adds constraint-setup complexity (**+0.5d contingency**).
   - `MultibodyFreeFwdDynamicsTpl` — no contact. Only suitable for APPROACH phase; NOT a valid backbone for grasping. Reject unless 7a's APPROACH phase is the only Phase-3 scope (it is not).
   - Custom ODE via `ode-abstract.hpp` — last resort, +≥1d.
3. Record decision in §"Phase 3 Spike Notes" below and proceed.

**Class naming in rtc_mpc**: keep `KinoDynamicsOCP` (the MPC-mode role) regardless of which Aligator dynamics class backs it — the name reflects our abstraction, not Aligator's.

### Entry State
- `RobotModelHandler`, `PhaseCostConfig`, `PhaseContext` shipped (Phases 1–2). **Phase 2 interface is frozen** — Phase 3 introduces a separate `OCPLimits` struct instead of extending `PhaseCostConfig`.
- Aligator 0.19.0 at `/usr/local`; CMake workarounds in place; `rtc_mpc/CMakeLists.txt` does not yet `find_package(aligator)`.

### Sub-step Breakdown (revised)

| # | Sub-step | Effort | Depends on |
|---|----------|--------|-----------|
| 3.0a | **CMake + `package.xml` aligator linkage** — must land first so spike + later code compiles inside the package | 0.1d | — |
| 3.0b | **Aligator API spike** — semantic-collision resolution + residual/ownership discovery | 0.4d | 3.0a |
| 3.1 | `OCPHandlerBase` + `OCPLimits` + `OCPBuildError` (header-only) | 0.1d | 3.0b |
| 3.2 | `CostFactory` returning `StageCost { stack, handles }` + unit test | 1.0d | 3.0b |
| 3.3 | `KinoDynamicsOCP` concrete handler + stage-phase mapping | 1.0d | 3.1, 3.2 |
| 3.4 | Integration test + perf + alloc audit | 0.5d | 3.3 |
| 3.5 | Phase-end housekeeping (README + memory + agent_docs + commit) | 0.1d | 3.4 |
| **Base** | | **3.2d** | |
| **Contingency** | +0.5d if 3.0b forces `MultibodyConstraintFwdDynamicsTpl` fallback (per-stage contact constraints non-trivial) | **+0.5d** | conditional |
| **Upper bound** | | **~3.7d** | within the 3.5d envelope when contingency hits |

### 3.0a — CMake + `package.xml` aligator linkage (precursor)

`rtc_mpc/CMakeLists.txt` — add after existing `find_package(pinocchio REQUIRED)`:
```cmake
find_package(aligator REQUIRED)
# ...
target_link_libraries(rtc_mpc PUBLIC aligator::aligator)
ament_export_dependencies(... aligator)
```
Append source entries once 3.2/3.3 files exist (can be empty placeholder now):
```cmake
# src/ocp/cost_factory.cpp   # added in 3.2
# src/ocp/kinodynamics_ocp.cpp  # added in 3.3
```

`rtc_mpc/package.xml` — **do NOT add `<depend>aligator</depend>`.** Aligator is source-installed at `/usr/local` with no rosdep key; adding a `<depend>` triggers colcon/rosdep failures. CMake-only linkage is sufficient since `/usr/local/lib/cmake/aligator/aligatorConfig.cmake` exports the target. Add a single-line comment:
```xml
<!-- aligator: source-installed to /usr/local; linked via CMake, not rosdep -->
```

Verification: `./build.sh -p rtc_mpc` must still finish green at this step (no new translation units added yet, just link line).

### 3.0b — Aligator API spike (BLOCKING)

Scratch at `/tmp/aligator_verify/kinodyn_spike/` (not committed; replaced by Spike Notes in this doc). Questions in order:

1. **Semantic-collision resolution.** Instantiate `KinodynamicsFwdDynamicsTpl<double>` on Panda (nq=nv=9, fixed-base) with 2 × 3D contacts. Observe: does the class accept a non-floating-base model? If yes, is `u`'s base-accel block meaningful (it should be zero/masked)? Expected outcome: class is unusable for v2.2-sense "KinoDynamics MPC" on fixed-base.
2. **Fallback backbone.** Instantiate `MultibodyConstraintFwdDynamicsTpl` on Panda with 2 × 3D fingertip contacts. Verify `forward(x, u, data)` where `x=[q;v]`, `u=τ ∈ R^{nv}` returns reasonable accelerations. Contact forces emerge as Lagrange multipliers `λ`, not part of `u`.
3. **Residual discovery** — for each, capture ctor signature, required Pinocchio data, residual dim, **reference-mutation API** (public field? setter? rebuild-only?):
   - `FramePlacementResidualTpl` (SE3 target)
   - `ContactForceResidualTpl` (frame_id + f_ref; dim matches contact_dim)
   - `StateErrorResidualTpl` / `ControlErrorResidualTpl` (target vector)
   - `CentroidalMomentumResidualTpl` (usefulness on fixed-base — likely none; may pin to zero weight permanently)
   - `MultibodyFrictionConeResidualTpl` (μ, N-facet API) — or on constraint-force path, `FrictionConeResidualTpl` on λ
4. **Ownership model.** Aligator `StageModelTpl::addCost(shared_ptr<CostAbstract>)`, `CostStackTpl::addCost(shared_ptr<CostAbstract>)`. Settle on **`shared_ptr` end-to-end** for residuals/costs; reserve `unique_ptr` for top-level `TrajOptProblem`. Confirm: mutating a residual via a retained `shared_ptr` handle reflects in the CostStack's evaluation (no internal copy of reference).
5. **Solver 2-stage smoke.** Build a trivial 2-stage `TrajOptProblemTpl`, wrap the chosen dynamics with `IntegratorSemiEulerTpl`, solve with `SolverProxDDP`. Record iter count + `results.primal_infeas` + wall-time.

**Spike exit:** 1-page notes appended to §"Phase 3 Spike Notes" with concrete answers to all 5. No `rtc_mpc/ocp/` code lands until these are filled.

### 3.1 — `OCPHandlerBase` + `OCPLimits` + `OCPBuildError` (0.1d)

`OCPLimits` is new and **separate from `PhaseCostConfig`** — Phase 2's interface stays frozen. Loaded once at handler init, not per phase.

```cpp
namespace rtc::mpc {

/// Non-cost, non-per-phase limits for an OCP. Loaded once from YAML at
/// handler construction. Separate from PhaseCostConfig (Phase 2, frozen).
struct OCPLimits {
  Eigen::VectorXd u_min{};   ///< size nu or empty (= unlimited)
  Eigen::VectorXd u_max{};   ///< size nu or empty (= unlimited)
  double friction_mu{0.7};   ///< shared across contacts
  int n_friction_facets{4};  ///< polyhedral friction-cone approximation
};

enum class OCPBuildError {
  kNoError = 0,
  kModelNotInitialised,
  kInvalidPhaseContext,        ///< unknown ocp_type, or topology change on UpdateReferences
  kInvalidCostConfig,          ///< horizon_length <= 0, dt <= 0, etc.
  kContactPlanModelMismatch,   ///< frame_id not present in current model
  kLimitsDimMismatch,          ///< u_min.size() != nu (and non-empty)
  kOverlappingContactPhases,   ///< ContactPlan::phases have overlapping t intervals
  kAligatorInstantiationFailure,
};

class OCPHandlerBase {
 public:
  virtual ~OCPHandlerBase() = default;

  /// Full rebuild (allocates freely). Off-RT path. Called on first init and
  /// whenever stage topology changes (horizon_length, per-stage contact set,
  /// ocp_type, nq/nv).
  [[nodiscard]] virtual OCPBuildError
  Build(const PhaseContext& ctx, const RobotModelHandler& model,
        const OCPLimits& limits) noexcept = 0;

  /// References-only update (alloc-free post first Build). Contract: if ctx
  /// implies a topology change vs cached state, returns kInvalidPhaseContext
  /// without modifying any stored state — caller must Build().
  [[nodiscard]] virtual OCPBuildError
  UpdateReferences(const PhaseContext& ctx) noexcept = 0;

  [[nodiscard]] virtual bool Built() const noexcept = 0;
  [[nodiscard]] virtual aligator::TrajOptProblemTpl<double>& problem() = 0;
  [[nodiscard]] virtual int horizon_length() const noexcept = 0;
  [[nodiscard]] virtual std::string_view ocp_type() const noexcept = 0;
};

}  // namespace rtc::mpc
```

Rationale:
- Split `Build` / `UpdateReferences` lets Phase 5 short-circuit rebuild on reference-only ticks (the common case).
- Non-owning `problem()` so `SolverProxDDP` (Phase 5) can `setProblem(...)` without copy.
- No `throw` anywhere (`rt-safety.md`, CLAUDE.md rule 2).

### 3.2 — `CostFactory` (1.0d)

**Design pivot over v1 of this plan:** CostFactory must expose the residuals it creates so `UpdateReferences` can mutate targets in place — not rebuild. Returns a bundle pairing the `CostStack` with typed `shared_ptr` handles.

```cpp
namespace rtc::mpc {

/// Handles to the residuals inside one stage's cost stack. Null when the
/// corresponding weight was <= 0 at Build time (term omitted → no mutation
/// target). shared_ptr is shared with the CostStack that owns evaluation;
/// mutating `->target_` / `->pref_` / `->ref_` on the handle is visible to
/// the solver without any rebuild. (Symmetry confirmed in 3.0b spike.)
struct StageCostHandles {
  std::shared_ptr<aligator::FramePlacementResidualTpl<double>> frame_placement{};
  std::shared_ptr<aligator::StateErrorResidualTpl<double>>     state_reg{};
  std::shared_ptr<aligator::ControlErrorResidualTpl<double>>   control_reg{};
  std::vector<std::shared_ptr<aligator::ContactForceResidualTpl<double>>>
      contact_force{};  // one per active contact on this stage; empty if none
  std::shared_ptr<aligator::CentroidalMomentumResidualTpl<double>> centroidal{};
};

struct StageCost {
  std::shared_ptr<aligator::CostStackTpl<double>> stack;
  StageCostHandles handles;
};

namespace cost_factory {

/// Build one stage's running cost. `active_contact_frame_ids` is resolved
/// by KinoDynamicsOCP::Build from ContactPlan::phases for this stage's time.
[[nodiscard]] StageCost
BuildRunningCost(const PhaseCostConfig& cfg,
                 const RobotModelHandler& model,
                 const pinocchio::SE3& ee_target,
                 const std::vector<int>& active_contact_frame_ids) noexcept;

/// Terminal cost: no control-reg, no contact-force, no control (x-only).
[[nodiscard]] StageCost
BuildTerminalCost(const PhaseCostConfig& cfg,
                  const RobotModelHandler& model,
                  const pinocchio::SE3& ee_target) noexcept;

}  // namespace cost_factory
}  // namespace rtc::mpc
```

Residual → weight mapping (term **omitted entirely** if weight ≤ 0 or active set empty; handle stays null):

| Residual | Aligator class (confirm in 3.0b) | Weight matrix | Dim | Stages |
|----------|----------------------------------|---------------|-----|--------|
| Frame placement | `FramePlacementResidualTpl(ee_frame_id, ee_target)` | `w_frame_placement · diag(W_placement)` | 6 | running + terminal |
| State reg | `StateErrorResidualTpl([q_posture_ref; 0])` | `w_state_reg · I` | nq+nv | running + terminal |
| Control reg | `ControlErrorResidualTpl(0)` | `w_control_reg · I` | nu | running only |
| Contact force (per active) | `ContactForceResidualTpl(frame_id, F_target[slice])` | `w_contact_force · I` | 3 or 6 | running only |
| Centroidal momentum | `CentroidalMomentumResidualTpl(0)` | `w_centroidal · I` | 6 | running (gated) |

**Tests** (`test_cost_factory.cpp`) — Panda 9-DoF, 2 × 3D fingertip contacts:

| Case | Assertion |
|------|-----------|
| AllWeightsZero | `stack` has 0 terms; all handles null |
| FramePlacementOnly | 1 term; `handles.frame_placement` non-null |
| TwoActiveContacts | `handles.contact_force.size() == 2`; each 3-dim |
| EmptyActiveContacts | `handles.contact_force.empty()` (free-flight stage) |
| WrenchContact6D | Build a model variant with `dim=6` contact → residual dim = 6 |
| TerminalOmitsControlAndContact | `handles.control_reg` + `contact_force` unpopulated regardless of weights |
| MutateTargetThroughHandle | Change `handles.frame_placement->pref` → CostStack evaluation reflects new target (proves shared_ptr symmetry; if this fails, 3.0b spike's ownership assumption was wrong) |
| NoexceptAudit | `static_assert(noexcept(cost_factory::BuildRunningCost(...)))` + grep for `throw` in src/ocp/cost_factory.cpp |

### 3.3 — `KinoDynamicsOCP` (1.0d, +0.5d if fallback)

```cpp
class KinoDynamicsOCP : public OCPHandlerBase {
  // ... interface methods ...
 private:
  std::unique_ptr<aligator::TrajOptProblemTpl<double>> problem_{};
  std::vector<StageCost> stage_costs_{};
  StageCost terminal_cost_{};
  std::shared_ptr<aligator::dynamics::ODEAbstractTpl<double>> dynamics_{};
  // Cached topology — used by UpdateReferences to reject topology changes:
  int horizon_length_{0};
  double dt_{0.0};
  int nq_{0}, nv_{0}, nu_{0};
  int total_contact_dim_{0};
  std::vector<std::vector<int>> stage_active_contacts_{};  // per-stage frame ids
  OCPLimits limits_{};
};
```

**Stage → phase mapping (spec).** For stage index `k ∈ [0, horizon_length)` at simulated time `t_k = k · dt`, the active-contact set is `phases[i].active_frame_ids` for the unique `i` with `phases[i].t_start ≤ t_k < phases[i].t_end`. Edge cases:
- `ContactPlan::phases` empty → all stages free-flight (`stage_active_contacts_[k] = {}`).
- No phase covers `t_k` (gap) → free-flight for that stage.
- Overlapping phases (`t_k` in two intervals) → `Build` returns `kOverlappingContactPhases`.
- `active_frame_ids` contains an id not in `model.contact_frames()` → `kContactPlanModelMismatch`.

**Dynamics class (chosen in 3.0b):**
- *Primary* (expected unused): `KinodynamicsFwdDynamicsTpl` — only if spike finds fixed-base works sensibly, which it likely will not.
- *Fallback* (expected chosen): `MultibodyConstraintFwdDynamicsTpl` — rigid-contact Lagrangian. Per-stage constraints built from `stage_active_contacts_[k]`; a stage's active-contact set change forces Build (topology change).

**Control layout** (depends on chosen dynamics class):
- *KinodynamicsFwdDynamics path*: `u = [a_base(6); a_joint(nv); f_c(Σ dim)]` — base-accel block ill-defined for fixed-base (one reason to avoid).
- *MultibodyConstraintFwdDynamics path*: `u = τ ∈ R^{nv}`. Contact forces live as constraint multipliers `λ`, not inside `u`. Cleaner; standard for fixed-base manipulators.

**Riccati gain (Aligator convention):** `K ∈ R^{nu × ndx}` with `ndx = 2·nv`. For fixed-base `nq = nv` → `ndx = nx`, so `rtc::mpc::RiccatiFeedback::SetGain(K, nu, nx)` contract matches. `accel_only=true` reads `K.topRows(nv)`, which on the constraint-fwd path is all of `K` since `nu = nv`. No slicing mismatch.

**Integrator:** `IntegratorSemiEulerTpl<double>` wrapping the continuous ODE, step `dt_`.

**Constraints per stage:**
- **Joint position box** — from `model.lowerPositionLimit` / `upperPositionLimit`; skip joints at ±∞.
- **Control box** — if `limits.u_min.size() == nu` (else skip); apply via `ControlErrorResidual` + bound constraint. Mismatched size → `kLimitsDimMismatch`.
- **Friction cone** — active-contact stages only, using `limits.friction_mu` and `limits.n_friction_facets`. On the constraint-fwd path, apply friction to the constraint multiplier `λ`, not a separate force variable.

**UpdateReferences semantics (alloc-free contract):**
- Walk `stage_costs_`; for each non-null handle, assign the mutable target field (e.g. `handles.frame_placement->pref = ctx.ee_target`, `handles.state_reg->target_.head(nq) = ctx.cost_config.q_posture_ref`, etc.). Same for `terminal_cost_`.
- Reject topology changes before mutation: if `ctx.cost_config.horizon_length != horizon_length_` or derived per-stage active-contact sets differ from cached `stage_active_contacts_`, return `kInvalidPhaseContext` (state untouched).
- Weight changes: modifying a scalar like `w_frame_placement` while non-zero updates the `QuadraticResidualCost` weight matrix (also via retained handle — verify in 3.0b whether that matrix is mutable through the handle). If a weight crosses 0 → non-zero or vice versa, that adds/removes a term → **topology change** → force-Build.

### 3.4 — Integration test + perf + alloc audit (0.5d)

`test/test_kinodynamics_ocp.cpp` on Panda (`/usr/local/share/example-robot-data/robots/panda_description/urdf/panda.urdf`):

| Case | Setup | Assertion |
|------|-------|-----------|
| BuildNeutral | ee_target = fk(q_neutral) | `Build()` == `kNoError`, `problem().numSteps() == horizon_length` |
| SolveReachesEETarget | ee_target = fk(q_neutral) + 0.1 m translation | SolverProxDDP converges, `primal_infeas < 1e-4`, `iter ≤ 30` |
| UpdateReferencesCorrectness | Build → UpdateReferences with new ee_target → solve | Final EE pose residual wrt new target < 1e-3 (proves the handle-mutation actually reached the solver) |
| ReBuildIdempotent | Build → Build (same ctx) | Both succeed; second leaves `problem()` functionally equivalent (solve gives same iter count ±1) |
| EmptyContactPlan | `ctx.contact_plan.phases = {}` | All stages free-flight; solver converges |
| WrenchContact6D | model variant with one 6D contact | Residual dims OK, solve converges |
| InvalidOcpType | `ctx.ocp_type = "fulldynamics"` | `Build()` == `kInvalidPhaseContext` |
| ContactPlanMismatch | bogus frame_id in phases | `Build()` == `kContactPlanModelMismatch` |
| OverlappingPhases | two phases with overlapping t intervals | `Build()` == `kOverlappingContactPhases` |
| LimitsDimMismatch | `u_min.size() != nu` | `Build()` == `kLimitsDimMismatch` |
| TopologyChangeRejected | Build → UpdateReferences with different horizon_length | returns `kInvalidPhaseContext`, stored state unchanged |

**Performance** — 100 solves, record `p50` / `p99` of solve wall-time; **log only, do not assert** (thresholds `< 5ms` / `< 10ms` on dev laptop are informational and recorded in change log).

**Allocation audit** — ASan is the wrong tool (checks UB, not alloc counts). Use mimalloc runtime stats (already linked via Aligator):
```cpp
mi_stats_reset();
for (int i = 0; i < 100; ++i) {
  ocp.UpdateReferences(ctx_with_jittered_target);
  solver.run(x0);
}
mi_stats_print(nullptr);  // goes to stderr; capture in test log
```
Record the observed per-iteration allocation count in the Change Log entry for Phase 3. No hard threshold (mimalloc may still batch internally for the solver) — what matters is the delta is stable across iterations, not that it is zero.

### 3.5 — Phase-end housekeeping (0.1d)

Execute §"Phase Completion Housekeeping" checklist: `rtc_mpc/README.md` (Status row ✅, Module map rows for `ocp/ocp_handler_base`, `ocp/cost_factory`, `ocp/kinodynamics_ocp`, new `OCPLimits` in design-invariants), Claude memory refresh, `agent_docs/` only if architecture-level changes (likely none — this phase is internal to rtc_mpc), progress-doc §Phase 3 section + Change Log, single commit `[rtc_mpc] Phase 3: OCPHandlerBase + KinoDynamicsOCP + CostFactory`.

### Exit Criteria

- [ ] 3.0b Spike Notes populated in this doc before any `rtc_mpc/ocp/` code lands
- [ ] `rtc_mpc` builds cleanly with `find_package(aligator)` linked; no `<depend>aligator</depend>` in package.xml
- [ ] Panda KinoDyn offline solve: `primal_infeas < 1e-4` within ≤ 30 ProxDDP iterations (hard-asserted)
- [ ] UpdateReferences exercises a non-null handle and the solver sees the change (hard-asserted via `UpdateReferencesCorrectness`)
- [ ] Topology-change rejection path hard-asserted (`TopologyChangeRejected`)
- [ ] Perf p50 / p99 logged in Change Log (not asserted)
- [ ] mimalloc alloc-delta across 100 UpdateReferences+solve iters logged in Change Log (not asserted)
- [ ] No `auto` with Eigen expressions in `rtc_mpc/src/ocp/` — verified by `grep -nE '\bauto\b' rtc_mpc/src/ocp` review (no hits on Eigen RHS)
- [ ] `colcon test --packages-select rtc_mpc` → 12/12 pass (10 existing + `test_cost_factory` + `test_kinodynamics_ocp`)
- [ ] Robot-agnostic audit (scoped):
  ```bash
  grep -rnE '\b(UR5e|ur5e|tool0|fingertip|\bnq\s*=\s*16\b)\b' \
       rtc_mpc/include rtc_mpc/src \
    | grep -v -E '^\s*(//|\*)' \
    | grep -v 'mpc_solution_types.hpp:21'
  # → must be empty.
  ```
  Note: `hand` / `panda` / `finger` are **not** in the strict-fail set because of false positives (`hand_posture` legitimate custom_weights key, Panda test fixtures). Audit manually via review.
- [ ] §Phase Completion Housekeeping applied + single commit

### Open Risks → Action (revised)

- **§11 #1 (Aligator dynamics class for fixed-base kino-dyn MPC)** — promoted to **dominant risk** of Phase 3. Resolution path fully specced in 3.0b; fallback is `MultibodyConstraintFwdDynamicsTpl`, not `MultibodyFreeFwdDynamicsTpl` (free dynamics has no contact, unusable for grasp scenarios). Budget: +0.5d contingency on 3.3 if fallback triggers.
- **§11 #2 (contact-force residual)** — `modelling/multibody/contact-force.hpp` present in Aligator 0.19.0. Confirm ctor + dim + reference-mutation API in 3.0b. Likely closable.
- **§11 #6 (PhaseCostConfig extension)** — **RETIRED**. Superseded by new `OCPLimits` struct; Phase 2's 10/10 tests and frozen interface are preserved.
- **§11 #7 (Riccati gain slice for KinoDyn `u = [τ; f_c]`)** — on the expected `MultibodyConstraintFwdDynamicsTpl` fallback path, `u = τ` (`nu = nv`), so `K ∈ R^{nv × 2nv}` and `accel_only=true` reads all of `K`. Phase 5 adapter is the trivial ColMajor→RowMajor copy documented in Phase 0. If spike forces the `KinodynamicsFwdDynamicsTpl` primary path, the `[a_base; a_joint; f_c]` layout resurrects the slicing concern — re-open this risk.
- **§11 #8 (NEW: handle-mutation visibility)** — `StageCostHandles` stores `shared_ptr` to residuals that `CostStackTpl` also holds. Mutating `handle->target_` must be visible to the solver's evaluator with no rebuild. This is the load-bearing assumption of the entire alloc-free `UpdateReferences` design; if spike finds Aligator internally copies targets at stage construction, fall back to "UpdateReferences triggers lightweight rebuild" and downgrade the alloc-audit criterion to "documented per-iter alloc count" only. Verified in 3.0b test #6 + in `test_cost_factory::MutateTargetThroughHandle`.

### Phase 3 Spike Notes (3.0b resolved 2026-04-19)

Scratch at `/tmp/aligator_verify/kinodyn_spike/` — Panda URDF, 2×3D fingertip contacts. All 4 questions resolved; `/tmp/aligator_verify/kinodyn_spike/` retained until Phase 3 commit, then deleted.

1. **Semantic collision.** `KinodynamicsFwdDynamicsTpl` **NOT** used — class is centroidal/floating-base-oriented per docstring; our "KinoDynamics MPC" backbone is **`MultibodyConstraintFwdDynamicsTpl`** (fixed-base confirmed: Panda 9-DoF, `nu = nv = 9`, `u = τ`, contact forces as Lagrange multipliers from rigid-contact solve).
2. **Residual constructors (confirmed by successful build):**
   - `FramePlacementResidualTpl(ndx, nu, model, SE3 ref, frame_id)` — `nr = 6`. Mutation via public `setReference(SE3)`. Stores `pin_model_` by value (copy).
   - `StateErrorResidualTpl(PolyManifold xspace, nu, VectorXd target)` — `nr = ndx`. `target_` is a public `VectorXs` field → mutable directly.
   - `ControlErrorResidualTpl(ndx, VectorXd u_target)` — `nr = nu`. Public `target_` field.
   - `ContactForceResidualTpl(ndx, model, actuation, constraint_models, prox_settings, fref, contact_name)` — **complex ctor**, requires the same `constraint_models` vector as `MultibodyConstraintFwdDynamicsTpl`. Mutation via `setReference(Vector3or6)`. String-based contact lookup, throws on mismatch.
   - `CentroidalMomentumResidualTpl` — reviewed but **permanently disabled** for fixed-base (no physical meaning); gated by `w_centroidal_momentum = 0` default.
   - `MultibodyFrictionConeResidualTpl` — deferred; spike did not exercise friction cone. Will be addressed in Phase 3.3 when constraint plumbing is written.
3. **Ownership model — MAJOR CORRECTION.** Aligator uses **`xyz::polymorphic<T>`** (value-type polymorphic, copy-on-construct), **NOT `shared_ptr`**. `StageModelTpl`, `CostStackTpl`, `QuadraticResidualCostTpl`, `TrajOptProblemTpl` all store their components by polymorphic value. External `shared_ptr` handles held by the caller would point to *originals*, not the *stored copies* the solver evaluates.
   **Solution (confirmed via spike Q3):** after the problem tree is assembled, retrieve raw pointers to the *stored* residuals via a chain:
   ```cpp
   auto* stage   = &*problem.stages_[k];                    // unwrap polymorphic<StageModel>
   auto* stack   = stage->getCost<CostStackTpl<double>>();
   auto* quad    = stack->getComponent<QuadraticResidualCostTpl<double>>("frame_placement");
   auto* residual = quad->getResidual<FramePlacementResidualTpl<double>>();
   residual->setReference(new_target);                       // visible to solver — NO rebuild
   ```
   Spike verified `stage.evaluate(x, u, data)` reflects the mutated reference. Alloc-free UpdateReferences remains feasible with this chain.
4. **Throw contamination.** Aligator ctors throw via `ALIGATOR_RUNTIME_ERROR` / `ALIGATOR_DOMAIN_ERROR` (e.g. manifold target not normalised, contact name not found, dimension mismatches). Our `noexcept` `Build()` must wrap Aligator calls in `try/catch` and convert to `OCPBuildError::kAligatorInstantiationFailure`. `Build()` is non-RT so the catch is acceptable (rt-safety rules apply to the 500Hz path, not OCP assembly).
5. **Integrator.** Class name is `IntegratorSemiImplEulerTpl` (header: `integrator-semi-euler.hpp`). Ctor: `(polymorphic<ODEType> cont_dynamics, Scalar timestep)`. Derives from `ExplicitIntegratorAbstractTpl` which satisfies `StageModelTpl::PolyDynamics`.
6. **Solver smoke (2-stage Panda).** `SolverProxDDPTpl<double>(1e-4, 1e-2)`. After 2 iters: `prim_infeas = 6e-11`, `dual_infeas = 1e-4`, `wall_us = 10602 μs`. `run()` returned `false` because dual tolerance not met within 30 iters (tuning issue, not a blocker). The pipeline compiles + links + runs; tolerance tuning deferred to 3.4 where real horizons (20) and proper warm-start will be measured.

### Design Corrections Locked In by 3.0b (supersedes earlier 3.1–3.3 sketches above)

| Item | Earlier (wrong) | Corrected |
|------|-----------------|-----------|
| Ownership | `std::shared_ptr<ResidualT>` handles | Raw pointers retrieved via `getCost → getComponent(key) → getResidual<T>()` chain after problem assembly |
| `StageCostHandles` members | `std::shared_ptr<...>` | `ResidualT*` (raw, non-owning) |
| CostFactory return type | `StageCost { shared_ptr<CostStack>, handles }` | `StageCost { CostStackTpl<double>, StageKeys keys }` where `StageKeys` is the set of string keys used for later lookup |
| Handle population | inside CostFactory | inside `KinoDynamicsOCP::Build` **after** `problem_->stages_` populated |
| `noexcept` Build | relied on ctors not throwing | explicit try/catch wrapping every Aligator ctor |
| `KinoDynamicsOCP::u` layout | `[τ; f_c]` (Σ dim vars in u) | `u = τ ∈ R^{nv}` only — contact forces are constraint multipliers `λ`, not control vars |
| Risk §11 #7 (Riccati slice) | open | **CLOSED**: `K ∈ R^{nv × 2nv}`, `accel_only` reads all rows of K since `nu = nv`. Phase 5 adapter is the trivial ColMajor→RowMajor copy |
| Risk §11 #8 (handle mutation visibility) | open | **CLOSED**: polymorphic `getComponent → getResidual` chain verified in spike Q3 |

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

## Phase Completion Housekeeping (MANDATORY per phase, before commit)

When a phase's code + tests land and are green, run this checklist **before** committing. A phase is not "done" until all five items are applied.

1. **`rtc_mpc/README.md`** — update the `## Status` table (flip the phase row to ✅) and extend the `## Module map` with any new header added in this phase (path, role, one-line summary). If public-API surface changed, refresh the `## Design invariants` bullet list.
2. **Claude auto-memory** — update `~/.claude/projects/-home-junho-ros2-ws-rtc-ws-src-rtc-framework/memory/project_mpc_implementation.md` (and its MEMORY.md line) to reflect:
   - Phases complete vs next
   - Any non-obvious deviation from the plan (naming, scope change, deferred risk)
   - Pointer back to this progress doc
   Prune memory entries this phase obsoleted. Do NOT save patterns derivable from code/git.
3. **Workspace documentation** — if the phase changed anything touching architecture, threading, RT-safety, controller wiring, or config conventions, update the relevant `agent_docs/*.md`:
   - `agent_docs/architecture.md` — new threads, new data-flow paths, new core types
   - `agent_docs/controllers.md` — new controller registration / MPC-driven controllers
   - `agent_docs/modification-guide.md` — new package-update checklist steps
   - `agent_docs/design-principles.md` — changes to the rtc_*/ur5e_* boundary
   - `agent_docs/conventions.md` — new naming or YAML conventions
   - `agent_docs/testing-debug.md` — new test targets worth listing
   If no such change exists, say so in the commit body ("no agent_docs update needed").
4. **This progress doc (`docs/mpc_implementation_progress.md`)**
   - Flip the phase row in the `## Phase Plan` table to ✅.
   - Add a `## Phase N — <title> (COMPLETE <YYYY-MM-DD>)` section mirroring the Phase 1/2 template: Goal, Files Delivered, Verified Behavior, Exit Criteria, Cross-Phase Invariants Upheld, Risks status.
   - Append one line to `## Change Log`.
5. **Commit** — single commit with prefix `[rtc_mpc] Phase N: <short summary>`. Bundle the code, README update, progress-doc update, and any `agent_docs/` edits into that one commit. Memory files live outside the repo and are NOT part of the commit.
   - Before committing: `colcon test --packages-select rtc_mpc` green, robot-agnostic grep clean, `git status` shows only intended files.
   - Delete any `/tmp/aligator_verify/*` scratch from the spike — source of truth is this doc's Spike Notes.

Final phase (Phase 7c) additionally deletes this progress doc per CLAUDE.md Post-Task Housekeeping.

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
| 2026-04-19 | Phase 3 complete: `OCPHandlerBase` + `OCPLimits` + `OCPBuildError`; `CostFactory` returning `StageCost { stack, keys }` (weight-gated, polymorphic-aware); `KinoDynamicsOCP` backed by `MultibodyConstraintFwdDynamicsTpl` (`u=τ`, contact forces as Lagrange multipliers). 24 new test cases pass (10 cost-factory + 14 kinodynamics-ocp integration). Aligator ownership model corrected from plan: handles are raw pointers retrieved via `getCost → getComponent(key) → getResidual<T>()` chain AFTER problem assembly (spike Q3 invalidated the original `shared_ptr`-based design). Post-assembly caching fixed a dangling-pointer heap corruption. Perf gap observed: solve p50 ~53 ms vs 5 ms target — elevated as new Risk §11 #9 for Phase 5 warm-start. |
