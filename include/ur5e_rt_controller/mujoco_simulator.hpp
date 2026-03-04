#ifndef UR5E_RT_CONTROLLER_MUJOCO_SIMULATOR_HPP_
#define UR5E_RT_CONTROLLER_MUJOCO_SIMULATOR_HPP_

// ── Includes: project, then MuJoCo, then C++ stdlib ───────────────────────────
#include <mujoco/mujoco.h>

#ifdef MUJOCO_HAVE_GLFW
#include <GLFW/glfw3.h>
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ur5e_rt_controller {

// ── MuJoCoSimulator ────────────────────────────────────────────────────────────
//
// Thread-safe wrapper around a MuJoCo physics model.
//
// Simulation modes:
//   kFreeRun  — advances mj_step() as fast as possible (up to max_rtf).
//   kSyncStep — publishes state, waits for one command, steps once.
//               Step latency ≈ controller Compute() time.
//
// Physics features:
//   - Joint efforts (torques) via data_->qfrc_actuator in StateCallback
//   - Gravity toggle: EnableGravity(false) for zero-g testing
//   - Body perturbation: Ctrl+drag in viewer applies mjvPerturb spring forces
//   - External force API: SetExternalForce() / ClearExternalForce()
//   - Contact toggle: SetContactEnabled(false) for free-space debugging
//   - Solver stats: GetSolverStats() → {improvement, gradient, iter, ncon}
//   - Integrator: Euler / RK4 / Implicit / ImplicitFast (SetIntegrator)
//   - Solver: PGS / CG / Newton (SetSolverType)
//   - Iterations / tolerance: SetSolverIterations(), SetSolverTolerance()
//
// Runtime controls (thread-safe):
//   Pause() / Resume() / IsPaused()
//   RequestReset()          — reinitialise to cfg_.initial_qpos
//   SetMaxRtf(double)       — adjust speed cap at runtime
//   EnableGravity(bool)     — toggle gravity (picked up by sim thread)
//   SetContactEnabled(bool) — enable/disable contact constraints
//   SetIntegrator(int)      — mjINT_EULER/RK4/IMPLICIT/IMPLICITFAST
//   SetSolverType(int)      — mjSOL_PGS/CG/NEWTON
//   SetSolverIterations(int) — max constraint solver iterations
//   SetSolverTolerance(double) — convergence tolerance
//   SetExternalForce()      — apply world-frame wrench to a body
//   ClearExternalForce()    — remove all external forces
//
// Viewer keyboard shortcuts (MUJOCO_HAVE_GLFW):
//   F1            — toggle detailed help overlay (keys + current state)
//   Space         — pause / resume
//   + / KP_ADD   — double max_rtf (unlimited → 2x)
//   - / KP_SUB   — halve max_rtf (≤0.5x → unlimited)
//   R             — reset simulation to initial pose
//   G             — toggle gravity
//   N             — toggle contact constraints
//   I             — cycle integrator (Euler→RK4→Implicit→ImplFast)
//   S             — cycle solver type (PGS→CG→Newton)
//   ]             — double solver iterations
//   [             — halve solver iterations
//   C             — toggle contact point markers
//   F             — toggle contact force arrows
//   V             — toggle collision geometry display
//   T             — toggle transparency
//   F3            — toggle RTF profiler graph
//   F4            — toggle solver stats overlay
//   Backspace     — reset visualisation options
//   Escape        — reset camera to default position
//
// Viewer mouse controls:
//   Left drag          — orbit (rotate) camera
//   Right drag         — pan (translate) camera
//   Scroll             — zoom in / out
//   Ctrl + Left drag   — apply perturbation force to hovered body
//
// Threading model:
//   SimLoop thread  — physics; sole writer of model_/data_
//   ViewerLoop thread — renders at ~60 Hz via GLFW (optional)
//   Caller thread   — SetCommand(), GetPositions(), SetExternalForce(), etc.
//
// Synchronisation:
//   cmd_mutex_   — pending_cmd_
//   cmd_pending_ — lock-free flag for FreeRun fast path
//   sync_cv_     — wakes SimLoopSyncStep on command/resume/reset
//   state_mutex_ — latest_positions_ / latest_velocities_ / latest_efforts_
//   viz_mutex_   — viz_qpos_ / viz_ncon_ (try_lock, never blocks SimLoop)
//   pert_mutex_  — shared_pert_ (viewer → sim perturbation transfer)
//
class MuJoCoSimulator {
 public:
  enum class SimMode {
    kFreeRun,   // Maximum speed (throttled by max_rtf)
    kSyncStep,  // 1:1 synchronised with controller commands
  };

  struct Config {
    std::string model_path;
    SimMode     mode{SimMode::kFreeRun};
    bool        enable_viewer{true};
    int         publish_decimation{1};   // kFreeRun: publish every N steps
    double      sync_timeout_ms{50.0};   // kSyncStep: command wait timeout
    double      max_rtf{0.0};           // 0.0 = unlimited
    std::array<double, 6> initial_qpos{
        0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0};

    // ── Physics solver ────────────────────────────────────────────────────────
    // integrator_type: mjINT_EULER(0) | mjINT_RK4(1) | mjINT_IMPLICIT(2)
    //                  mjINT_IMPLICITFAST(3)
    int    integrator_type{mjINT_EULER};
    // solver_type: mjSOL_PGS(0) | mjSOL_CG(1) | mjSOL_NEWTON(2)
    int    solver_type{mjSOL_NEWTON};
    int    solver_iterations{100};       // max constraint solver iterations
    double solver_tolerance{1e-8};       // convergence tolerance (0 = disabled)
  };

  // Invoked from SimLoop after each publish step.
  // efforts = data_->qfrc_actuator (actuator forces in Nm, indexed by DOF).
  using StateCallback = std::function<void(
      const std::array<double, 6>& positions,
      const std::array<double, 6>& velocities,
      const std::array<double, 6>& efforts)>;

  explicit MuJoCoSimulator(Config cfg) noexcept;
  ~MuJoCoSimulator();

  MuJoCoSimulator(const MuJoCoSimulator&)            = delete;
  MuJoCoSimulator& operator=(const MuJoCoSimulator&) = delete;
  MuJoCoSimulator(MuJoCoSimulator&&)                 = delete;
  MuJoCoSimulator& operator=(MuJoCoSimulator&&)      = delete;

  // Load MJCF model and resolve joint indices.  Must be called before Start().
  [[nodiscard]] bool Initialize() noexcept;

  // Start simulation and viewer threads.
  void Start() noexcept;

  // Signal stop and join all threads.
  void Stop() noexcept;

  // Write a position command into the pending buffer.  Thread-safe.
  void SetCommand(const std::array<double, 6>& cmd) noexcept;

  // Register the state callback (positions, velocities, efforts).
  void SetStateCallback(StateCallback cb) noexcept;

  [[nodiscard]] std::array<double, 6> GetPositions()  const noexcept;
  [[nodiscard]] std::array<double, 6> GetVelocities() const noexcept;
  [[nodiscard]] std::array<double, 6> GetEfforts()    const noexcept;

  // Solver statistics snapshot captured after each mj_step().
  struct SolverStats {
    double improvement{0.0};  // constraint violation improvement (last iter)
    double gradient{0.0};     // gradient norm at convergence
    int    iter{0};           // solver iterations actually used
    int    ncon{0};           // number of active contacts
  };
  [[nodiscard]] SolverStats GetSolverStats() const noexcept;

  // ── Physics solver controls (thread-safe) ─────────────────────────────────

  // Integrator: mjINT_EULER(0) | mjINT_RK4(1) | mjINT_IMPLICIT(2) | mjINT_IMPLICITFAST(3)
  // Applied by sim thread before the next mj_step().
  void SetIntegrator(int type) noexcept {
    solver_integrator_.store(type, std::memory_order_relaxed);
  }
  [[nodiscard]] int GetIntegrator() const noexcept {
    return solver_integrator_.load(std::memory_order_relaxed);
  }

  // Solver: mjSOL_PGS(0) | mjSOL_CG(1) | mjSOL_NEWTON(2)
  void SetSolverType(int type) noexcept {
    solver_type_.store(type, std::memory_order_relaxed);
  }
  [[nodiscard]] int GetSolverType() const noexcept {
    return solver_type_.load(std::memory_order_relaxed);
  }

  // Max solver iterations (clamped to [1, 1000]).
  void SetSolverIterations(int iters) noexcept {
    solver_iterations_.store(
        std::max(1, std::min(iters, 1000)), std::memory_order_relaxed);
  }
  [[nodiscard]] int GetSolverIterations() const noexcept {
    return solver_iterations_.load(std::memory_order_relaxed);
  }

  // Solver convergence tolerance (0.0 = disabled).
  void SetSolverTolerance(double tol) noexcept {
    solver_tolerance_.store(tol < 0.0 ? 0.0 : tol, std::memory_order_relaxed);
  }
  [[nodiscard]] double GetSolverTolerance() const noexcept {
    return solver_tolerance_.load(std::memory_order_relaxed);
  }

  // Enable / disable contact constraints (mjDSBL_CONTACT).
  void SetContactEnabled(bool enabled) noexcept {
    contacts_enabled_.store(enabled, std::memory_order_relaxed);
  }
  [[nodiscard]] bool IsContactEnabled() const noexcept {
    return contacts_enabled_.load(std::memory_order_relaxed);
  }

  // ── Physics controls (thread-safe) ────────────────────────────────────────

  // Pause / resume physics.  The viewer continues to render.
  void Pause()   noexcept { paused_.store(true,  std::memory_order_relaxed); }
  void Resume()  noexcept {
    paused_.store(false, std::memory_order_relaxed);
    sync_cv_.notify_all();
  }
  [[nodiscard]] bool IsPaused() const noexcept {
    return paused_.load(std::memory_order_relaxed);
  }

  // Request reset to cfg_.initial_qpos (sim thread executes it).
  void RequestReset() noexcept {
    reset_requested_.store(true, std::memory_order_relaxed);
    sync_cv_.notify_all();
  }

  // Adjust RTF speed cap at runtime (0.0 = unlimited).
  void SetMaxRtf(double rtf) noexcept {
    current_max_rtf_.store(rtf < 0.0 ? 0.0 : rtf, std::memory_order_relaxed);
  }
  [[nodiscard]] double GetMaxRtf() const noexcept {
    return current_max_rtf_.load(std::memory_order_relaxed);
  }

  // Toggle gravity.  Sim thread reads gravity_enabled_ before each mj_step().
  void EnableGravity(bool enable) noexcept {
    gravity_enabled_.store(enable, std::memory_order_relaxed);
  }
  [[nodiscard]] bool IsGravityEnabled() const noexcept {
    return gravity_enabled_.load(std::memory_order_relaxed);
  }

  // Apply an external 6-DOF wrench [Fx,Fy,Fz,Tx,Ty,Tz] (world frame, SI units)
  // to a specific body by index.  body_id == 0 is the world body (no effect).
  // The force is applied every sim step until ClearExternalForce() is called.
  void SetExternalForce(int body_id,
                        const std::array<double, 6>& wrench_world) noexcept;
  void ClearExternalForce() noexcept;

  // Transfer a mjvPerturb state from the viewer to the sim thread.
  // Called by ViewerLoop when Ctrl+drag is active.
  void UpdatePerturb(const mjvPerturb& pert) noexcept;
  void ClearPerturb() noexcept;

  // ── Status accessors ──────────────────────────────────────────────────────
  [[nodiscard]] bool     IsRunning()  const noexcept { return running_.load(); }
  [[nodiscard]] uint64_t StepCount()  const noexcept { return step_count_.load(); }
  [[nodiscard]] double   SimTimeSec() const noexcept { return sim_time_sec_.load(); }
  [[nodiscard]] int      NumJoints()  const noexcept { return model_ ? model_->nq : 0; }
  [[nodiscard]] double   GetRtf()     const noexcept {
    return rtf_.load(std::memory_order_relaxed);
  }

 private:
  Config   cfg_;
  mjModel* model_{nullptr};
  mjData*  data_{nullptr};

  std::atomic<bool>     running_{false};
  std::atomic<uint64_t> step_count_{0};
  std::atomic<double>   sim_time_sec_{0.0};

  // ── Runtime control flags ─────────────────────────────────────────────────
  std::atomic<bool>   paused_{false};
  std::atomic<bool>   reset_requested_{false};
  std::atomic<double> current_max_rtf_{0.0};
  std::atomic<bool>   gravity_enabled_{true};
  double              original_gravity_z_{-9.81};  // from model, set in Initialize()

  // ── Physics solver atomics (applied in PreparePhysicsStep) ────────────────
  std::atomic<int>    solver_integrator_{mjINT_EULER};
  std::atomic<int>    solver_type_{mjSOL_NEWTON};
  std::atomic<int>    solver_iterations_{100};
  std::atomic<double> solver_tolerance_{1e-8};
  std::atomic<bool>   contacts_enabled_{true};

  // ── Solver statistics (written by sim thread, read by viewer) ─────────────
  mutable std::mutex solver_stats_mutex_;
  SolverStats        latest_solver_stats_{};

  // ── Command buffer ────────────────────────────────────────────────────────
  mutable std::mutex    cmd_mutex_;
  std::atomic<bool>     cmd_pending_{false};
  std::array<double, 6> pending_cmd_{};

  std::mutex              sync_mutex_;
  std::condition_variable sync_cv_;

  // ── State buffer (under state_mutex_) ─────────────────────────────────────
  mutable std::mutex    state_mutex_;
  std::array<double, 6> latest_positions_{};
  std::array<double, 6> latest_velocities_{};
  std::array<double, 6> latest_efforts_{};   // qfrc_actuator per joint DOF

  // ── Viewer double-buffer ──────────────────────────────────────────────────
  mutable std::mutex  viz_mutex_;
  std::vector<double> viz_qpos_{};
  int                 viz_ncon_{0};
  bool                viz_dirty_{false};

  StateCallback state_cb_{nullptr};

  std::jthread sim_thread_;
  std::jthread viewer_thread_;

  std::array<int, 6> joint_qpos_indices_{0, 1, 2, 3, 4, 5};
  std::array<int, 6> joint_qvel_indices_{0, 1, 2, 3, 4, 5};

  // ── RTF measurement ───────────────────────────────────────────────────────
  std::chrono::steady_clock::time_point rtf_wall_start_{};
  double                                rtf_sim_start_{0.0};
  std::atomic<double>                   rtf_{0.0};

  // ── Max-RTF throttle (sim thread only) ───────────────────────────────────
  std::chrono::steady_clock::time_point throttle_wall_start_{};
  double                                throttle_sim_start_{0.0};
  double                                throttle_rtf_{0.0};

  // ── External forces / perturbation (under pert_mutex_) ───────────────────
  mutable std::mutex pert_mutex_;
  mjvPerturb         shared_pert_{};     // viewer → sim perturbation
  bool               pert_active_{false};
  // Flat xfrc_applied buffer: 6 * nbody doubles.
  // Used by SetExternalForce(); shared_pert_ is applied separately.
  std::vector<double> ext_xfrc_{};
  bool                ext_xfrc_dirty_{false};

  // ── Internal helpers ───────────────────────────────────────────────────────
  void ResolveJointIndices() noexcept;
  void ApplyCommand() noexcept;
  void ReadState() noexcept;
  // Capture solver statistics from data_ after mj_step().
  void ReadSolverStats() noexcept;
  void InvokeStateCallback() noexcept;
  void UpdateVizBuffer() noexcept;
  void UpdateRtf(uint64_t step) noexcept;
  void ThrottleIfNeeded() noexcept;
  void HandleReset() noexcept;
  // Apply solver params, gravity, external forces, and perturbation before mj_step().
  void PreparePhysicsStep() noexcept;
  // Clear xfrc_applied after mj_step().
  void ClearContactForces() noexcept;

  void SimLoopFreeRun(std::stop_token stop) noexcept;
  void SimLoopSyncStep(std::stop_token stop) noexcept;
  void ViewerLoop(std::stop_token stop) noexcept;
};

// ── Joint name table ───────────────────────────────────────────────────────────
static constexpr std::array<const char*, 6> kMjJointNames = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
};

// ═════════════════════════════════════════════════════════════════════════════
// Inline implementation
// ═════════════════════════════════════════════════════════════════════════════

inline MuJoCoSimulator::MuJoCoSimulator(Config cfg) noexcept
    : cfg_(std::move(cfg)) {
  current_max_rtf_.store(cfg_.max_rtf, std::memory_order_relaxed);
  mjv_defaultPerturb(&shared_pert_);
}

inline MuJoCoSimulator::~MuJoCoSimulator() {
  Stop();
  if (data_)  { mj_deleteData(data_);   data_  = nullptr; }
  if (model_) { mj_deleteModel(model_); model_ = nullptr; }
}

inline void MuJoCoSimulator::ResolveJointIndices() noexcept {
  for (std::size_t i = 0; i < 6; ++i) {
    const int jnt_id = mj_name2id(model_, mjOBJ_JOINT, kMjJointNames[i]);
    if (jnt_id < 0) {
      fprintf(stderr,
              "[MuJoCoSimulator] Joint '%s' not found — using static index %zu\n",
              kMjJointNames[i], i);
      joint_qpos_indices_[i] = static_cast<int>(i);
      joint_qvel_indices_[i] = static_cast<int>(i);
    } else {
      joint_qpos_indices_[i] = model_->jnt_qposadr[jnt_id];
      joint_qvel_indices_[i] = model_->jnt_dofadr[jnt_id];
      fprintf(stdout, "[MuJoCoSimulator] '%s' → qpos[%d]  qvel[%d]\n",
              kMjJointNames[i], joint_qpos_indices_[i], joint_qvel_indices_[i]);
    }
  }
}

inline bool MuJoCoSimulator::Initialize() noexcept {
  char error[1000] = {};
  model_ = mj_loadXML(cfg_.model_path.c_str(), nullptr, error, sizeof(error));
  if (!model_) {
    fprintf(stderr, "[MuJoCoSimulator] Failed to load '%s': %s\n",
            cfg_.model_path.c_str(), error);
    return false;
  }

  data_ = mj_makeData(model_);
  if (!data_) {
    fprintf(stderr, "[MuJoCoSimulator] mj_makeData failed\n");
    mj_deleteModel(model_);
    model_ = nullptr;
    return false;
  }

  // Store original gravity for toggle
  original_gravity_z_ = static_cast<double>(model_->opt.gravity[2]);

  // Apply initial solver configuration from Config
  model_->opt.integrator  = static_cast<mjtIntegrator>(cfg_.integrator_type);
  model_->opt.solver      = static_cast<mjtSolver>(cfg_.solver_type);
  model_->opt.iterations  = cfg_.solver_iterations;
  model_->opt.tolerance   = static_cast<mjtNum>(cfg_.solver_tolerance);
  // Sync atomics with config values
  solver_integrator_.store(cfg_.integrator_type,  std::memory_order_relaxed);
  solver_type_.store(cfg_.solver_type,             std::memory_order_relaxed);
  solver_iterations_.store(cfg_.solver_iterations, std::memory_order_relaxed);
  solver_tolerance_.store(cfg_.solver_tolerance,   std::memory_order_relaxed);

  // Pre-size external force buffer
  viz_qpos_.assign(static_cast<std::size_t>(model_->nq), 0.0);
  ext_xfrc_.assign(static_cast<std::size_t>(model_->nbody) * 6, 0.0);

  ResolveJointIndices();

  const int njoints = std::min(6, model_->nq);
  for (int i = 0; i < njoints; ++i) {
    const double q0 = cfg_.initial_qpos[static_cast<std::size_t>(i)];
    data_->qpos[joint_qpos_indices_[static_cast<std::size_t>(i)]] = q0;
    data_->ctrl[i] = q0;
  }
  mj_forward(model_, data_);
  ReadState();

  fprintf(stdout,
          "[MuJoCoSimulator] Loaded '%s'  nq=%d  nv=%d  nu=%d  nbody=%d"
          "  dt=%.4f s  mode=%s\n",
          cfg_.model_path.c_str(),
          model_->nq, model_->nv, model_->nu, model_->nbody,
          static_cast<double>(model_->opt.timestep),
          cfg_.mode == SimMode::kFreeRun ? "free_run" : "sync_step");
  return true;
}

inline void MuJoCoSimulator::Start() noexcept {
  if (running_.exchange(true)) { return; }

  if (cfg_.mode == SimMode::kFreeRun) {
    sim_thread_ = std::jthread([this](std::stop_token st) { SimLoopFreeRun(st); });
  } else {
    sim_thread_ = std::jthread([this](std::stop_token st) { SimLoopSyncStep(st); });
  }
  if (cfg_.enable_viewer) {
    viewer_thread_ = std::jthread([this](std::stop_token st) { ViewerLoop(st); });
  }
}

inline void MuJoCoSimulator::Stop() noexcept {
  running_.store(false);
  sync_cv_.notify_all();
  if (sim_thread_.joinable()) {
    sim_thread_.request_stop();
    sim_thread_.join();
  }
  if (viewer_thread_.joinable()) {
    viewer_thread_.request_stop();
    viewer_thread_.join();
  }
}

inline void MuJoCoSimulator::SetCommand(const std::array<double, 6>& cmd) noexcept {
  { std::lock_guard lock(cmd_mutex_); pending_cmd_ = cmd; }
  cmd_pending_.store(true, std::memory_order_release);
  if (cfg_.mode == SimMode::kSyncStep) { sync_cv_.notify_one(); }
}

inline void MuJoCoSimulator::SetStateCallback(StateCallback cb) noexcept {
  state_cb_ = std::move(cb);
}

inline std::array<double, 6> MuJoCoSimulator::GetPositions() const noexcept {
  std::lock_guard lock(state_mutex_);
  return latest_positions_;
}
inline std::array<double, 6> MuJoCoSimulator::GetVelocities() const noexcept {
  std::lock_guard lock(state_mutex_);
  return latest_velocities_;
}
inline std::array<double, 6> MuJoCoSimulator::GetEfforts() const noexcept {
  std::lock_guard lock(state_mutex_);
  return latest_efforts_;
}

// ── Physics controls ──────────────────────────────────────────────────────────

inline void MuJoCoSimulator::SetExternalForce(
    int body_id, const std::array<double, 6>& wrench_world) noexcept {
  if (body_id <= 0 || body_id >= model_->nbody) { return; }
  std::lock_guard lock(pert_mutex_);
  const std::size_t offset = static_cast<std::size_t>(body_id) * 6;
  for (std::size_t i = 0; i < 6; ++i) {
    ext_xfrc_[offset + i] = wrench_world[i];
  }
  ext_xfrc_dirty_ = true;
}

inline void MuJoCoSimulator::ClearExternalForce() noexcept {
  std::lock_guard lock(pert_mutex_);
  std::fill(ext_xfrc_.begin(), ext_xfrc_.end(), 0.0);
  ext_xfrc_dirty_ = false;
}

inline void MuJoCoSimulator::UpdatePerturb(const mjvPerturb& pert) noexcept {
  std::lock_guard lock(pert_mutex_);
  shared_pert_  = pert;
  pert_active_  = (pert.active != 0);
}

inline void MuJoCoSimulator::ClearPerturb() noexcept {
  std::lock_guard lock(pert_mutex_);
  mjv_defaultPerturb(&shared_pert_);
  pert_active_ = false;
}

// ── Private helpers ────────────────────────────────────────────────────────────

inline void MuJoCoSimulator::ApplyCommand() noexcept {
  std::lock_guard lock(cmd_mutex_);
  if (!model_) { return; }
  const int nact = std::min(6, model_->nu);
  for (int i = 0; i < nact; ++i) {
    data_->ctrl[i] = pending_cmd_[static_cast<std::size_t>(i)];
  }
}

inline void MuJoCoSimulator::ReadState() noexcept {
  if (!model_ || !data_) { return; }
  std::lock_guard lock(state_mutex_);
  for (std::size_t i = 0; i < 6; ++i) {
    latest_positions_[i]  = data_->qpos[joint_qpos_indices_[i]];
    latest_velocities_[i] = data_->qvel[joint_qvel_indices_[i]];
    // qfrc_actuator: net actuator force on each DOF (Nm for revolute joints).
    latest_efforts_[i]    = data_->qfrc_actuator[joint_qvel_indices_[i]];
  }
}

inline void MuJoCoSimulator::ReadSolverStats() noexcept {
  if (!data_) { return; }
  SolverStats s{};
  s.ncon = data_->ncon;
  s.iter = data_->solver_iter;
  // mjSolverStat[0] holds aggregate stats for the last solve.
  if (s.iter > 0) {
    s.improvement = static_cast<double>(data_->solver[0].improvement);
    s.gradient    = static_cast<double>(data_->solver[0].gradient);
  }
  std::lock_guard lock(solver_stats_mutex_);
  latest_solver_stats_ = s;
}

inline MuJoCoSimulator::SolverStats MuJoCoSimulator::GetSolverStats() const noexcept {
  std::lock_guard lock(solver_stats_mutex_);
  return latest_solver_stats_;
}

inline void MuJoCoSimulator::InvokeStateCallback() noexcept {
  if (!state_cb_) { return; }
  std::array<double, 6> pos{}, vel{}, eff{};
  {
    std::lock_guard lock(state_mutex_);
    pos = latest_positions_;
    vel = latest_velocities_;
    eff = latest_efforts_;
  }
  state_cb_(pos, vel, eff);
}

inline void MuJoCoSimulator::UpdateVizBuffer() noexcept {
  if (viz_mutex_.try_lock()) {
    std::memcpy(viz_qpos_.data(), data_->qpos,
                static_cast<std::size_t>(model_->nq) * sizeof(double));
    viz_ncon_  = data_->ncon;
    viz_dirty_ = true;
    viz_mutex_.unlock();
  }
}

inline void MuJoCoSimulator::UpdateRtf(uint64_t step) noexcept {
  if (step % 200 != 0) { return; }
  const auto   wall_now = std::chrono::steady_clock::now();
  const double wall_dt  =
      std::chrono::duration<double>(wall_now - rtf_wall_start_).count();
  const double sim_dt   = data_->time - rtf_sim_start_;
  if (wall_dt > 0.01) {
    rtf_.store(sim_dt / wall_dt, std::memory_order_relaxed);
    rtf_wall_start_ = wall_now;
    rtf_sim_start_  = data_->time;
  }
}

inline void MuJoCoSimulator::ThrottleIfNeeded() noexcept {
  const double max_rtf = current_max_rtf_.load(std::memory_order_relaxed);
  if (max_rtf != throttle_rtf_) {
    throttle_wall_start_ = std::chrono::steady_clock::now();
    throttle_sim_start_  = data_->time;
    throttle_rtf_        = max_rtf;
  }
  if (max_rtf <= 0.0) { return; }
  const double sim_elapsed = data_->time - throttle_sim_start_;
  const double target_wall = sim_elapsed / max_rtf;
  const double actual_wall = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - throttle_wall_start_).count();
  if (actual_wall < target_wall) {
    std::this_thread::sleep_for(
        std::chrono::duration<double>(target_wall - actual_wall));
  }
}

// ── PreparePhysicsStep ─────────────────────────────────────────────────────────
//
// Called on the sim thread immediately before mj_step():
//   1. Apply gravity state (toggle on/off).
//   2. Apply user-specified external forces (SetExternalForce).
//   3. Apply mjvPerturb spring force (viewer Ctrl+drag).
//
inline void MuJoCoSimulator::PreparePhysicsStep() noexcept {
  // 1. Physics solver parameters (integrator, solver type, iterations, tolerance)
  model_->opt.integrator =
      static_cast<mjtIntegrator>(solver_integrator_.load(std::memory_order_relaxed));
  model_->opt.solver =
      static_cast<mjtSolver>(solver_type_.load(std::memory_order_relaxed));
  model_->opt.iterations = solver_iterations_.load(std::memory_order_relaxed);
  model_->opt.tolerance  =
      static_cast<mjtNum>(solver_tolerance_.load(std::memory_order_relaxed));

  // 2. Contact enable / disable (mjDSBL_CONTACT flag)
  if (contacts_enabled_.load(std::memory_order_relaxed)) {
    model_->opt.disableflags &= ~mjDSBL_CONTACT;
  } else {
    model_->opt.disableflags |= mjDSBL_CONTACT;
  }

  // 3. Gravity toggle (cheap relaxed load)
  model_->opt.gravity[2] =
      gravity_enabled_.load(std::memory_order_relaxed)
      ? static_cast<mjtNum>(original_gravity_z_)
      : static_cast<mjtNum>(0.0);

  // 2. External forces and perturbation (under pert_mutex_)
  if (pert_mutex_.try_lock()) {
    // User-specified wrench via SetExternalForce()
    if (ext_xfrc_dirty_) {
      const std::size_t n = static_cast<std::size_t>(model_->nbody) * 6;
      std::memcpy(data_->xfrc_applied, ext_xfrc_.data(), n * sizeof(double));
    } else {
      mju_zero(data_->xfrc_applied, model_->nbody * 6);
    }
    // 3. Viewer perturbation spring (additive on top of ext_xfrc_)
    if (pert_active_ && shared_pert_.select > 0) {
      mjv_applyPerturbForce(model_, data_, &shared_pert_);
    }
    pert_mutex_.unlock();
  }
}

inline void MuJoCoSimulator::ClearContactForces() noexcept {
  // xfrc_applied is re-set each step from the external force buffer.
  // Clear it here so stale forces don't persist if pert_mutex_ was not acquired.
  if (!pert_active_ && !ext_xfrc_dirty_) {
    mju_zero(data_->xfrc_applied, model_->nbody * 6);
  }
}

// ── HandleReset ───────────────────────────────────────────────────────────────
inline void MuJoCoSimulator::HandleReset() noexcept {
  mj_resetData(model_, data_);
  const int n = std::min(6, model_->nq);
  for (int i = 0; i < n; ++i) {
    const std::size_t ui = static_cast<std::size_t>(i);
    data_->qpos[joint_qpos_indices_[ui]] = cfg_.initial_qpos[ui];
    data_->ctrl[i]                       = cfg_.initial_qpos[ui];
  }
  // Restore gravity (may have been zeroed before reset)
  model_->opt.gravity[2] =
      gravity_enabled_.load(std::memory_order_relaxed)
      ? static_cast<mjtNum>(original_gravity_z_)
      : static_cast<mjtNum>(0.0);
  mj_forward(model_, data_);
  {
    std::lock_guard lock(pert_mutex_);
    mjv_defaultPerturb(&shared_pert_);
    pert_active_ = false;
    std::fill(ext_xfrc_.begin(), ext_xfrc_.end(), 0.0);
    ext_xfrc_dirty_ = false;
  }
  step_count_.store(0,   std::memory_order_relaxed);
  sim_time_sec_.store(0.0, std::memory_order_relaxed);
  rtf_.store(0.0, std::memory_order_relaxed);
  const auto now       = std::chrono::steady_clock::now();
  rtf_wall_start_      = now;  rtf_sim_start_      = data_->time;
  throttle_wall_start_ = now;  throttle_sim_start_ = data_->time;
  throttle_rtf_        = current_max_rtf_.load(std::memory_order_relaxed);
  ReadState();
  fprintf(stdout, "[MuJoCoSimulator] Reset to initial pose\n");
}

// ── SimLoopFreeRun ─────────────────────────────────────────────────────────────
inline void MuJoCoSimulator::SimLoopFreeRun(std::stop_token stop) noexcept {
  if (!model_ || !data_) { return; }

  const auto decim = static_cast<uint64_t>(
      cfg_.publish_decimation > 0 ? cfg_.publish_decimation : 1);
  uint64_t step = 0;

  const auto loop_start = std::chrono::steady_clock::now();
  rtf_wall_start_      = loop_start;  rtf_sim_start_      = data_->time;
  throttle_wall_start_ = loop_start;  throttle_sim_start_ = data_->time;
  throttle_rtf_        = current_max_rtf_.load(std::memory_order_relaxed);

  while (!stop.stop_requested() && running_.load()) {
    // ── Pause ─────────────────────────────────────────────────────────────
    if (paused_.load(std::memory_order_relaxed)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }
    // ── Reset ─────────────────────────────────────────────────────────────
    if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
      HandleReset();
      step = 0;
      continue;
    }
    // ── Command (lock-free fast path) ─────────────────────────────────────
    if (cmd_pending_.load(std::memory_order_acquire)) {
      ApplyCommand();
      cmd_pending_.store(false, std::memory_order_release);
    }
    // ── Physics step ──────────────────────────────────────────────────────
    PreparePhysicsStep();
    mj_step(model_, data_);
    ClearContactForces();
    ReadSolverStats();

    ++step;
    step_count_.store(step, std::memory_order_relaxed);
    sim_time_sec_.store(data_->time, std::memory_order_relaxed);

    if (step % decim == 0) {
      ReadState();
      InvokeStateCallback();
    }
    UpdateRtf(step);
    ThrottleIfNeeded();
    if ((step % 8 == 0) && cfg_.enable_viewer) { UpdateVizBuffer(); }
  }

  fprintf(stdout,
          "[MuJoCoSimulator] FreeRun exited — steps=%lu  sim_time=%.3f s\n",
          static_cast<unsigned long>(step_count_.load()), sim_time_sec_.load());
}

// ── SimLoopSyncStep ────────────────────────────────────────────────────────────
inline void MuJoCoSimulator::SimLoopSyncStep(std::stop_token stop) noexcept {
  if (!model_ || !data_) { return; }

  const auto timeout = std::chrono::milliseconds(
      static_cast<int64_t>(cfg_.sync_timeout_ms > 0.0 ? cfg_.sync_timeout_ms : 50.0));
  uint64_t step = 0;

  const auto loop_start = std::chrono::steady_clock::now();
  rtf_wall_start_      = loop_start;  rtf_sim_start_      = data_->time;
  throttle_wall_start_ = loop_start;  throttle_sim_start_ = data_->time;
  throttle_rtf_        = current_max_rtf_.load(std::memory_order_relaxed);

  while (!stop.stop_requested() && running_.load()) {
    // ── Pause ─────────────────────────────────────────────────────────────
    if (paused_.load(std::memory_order_relaxed)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }
    // ── Reset ─────────────────────────────────────────────────────────────
    if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
      HandleReset();
      step = 0;
      continue;
    }
    // 1. Publish current state
    ReadState();
    InvokeStateCallback();

    // 2. Wait for command (or timeout / stop / resume / reset)
    {
      std::unique_lock lock(sync_mutex_);
      sync_cv_.wait_for(lock, timeout, [this, &stop] {
        return cmd_pending_.load(std::memory_order_relaxed)
            || stop.stop_requested()
            || !running_.load()
            || reset_requested_.load(std::memory_order_relaxed);
      });
    }
    if (stop.stop_requested() || !running_.load()) { break; }
    if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
      HandleReset();
      step = 0;
      continue;
    }
    // 3. Apply command and step
    if (cmd_pending_.load(std::memory_order_acquire)) {
      ApplyCommand();
      cmd_pending_.store(false, std::memory_order_release);
    }
    PreparePhysicsStep();
    mj_step(model_, data_);
    ClearContactForces();
    ReadSolverStats();

    ++step;
    step_count_.store(step, std::memory_order_relaxed);
    sim_time_sec_.store(data_->time, std::memory_order_relaxed);

    UpdateRtf(step);
    ThrottleIfNeeded();
    if ((step % 8 == 0) && cfg_.enable_viewer) { UpdateVizBuffer(); }
  }

  fprintf(stdout,
          "[MuJoCoSimulator] SyncStep exited — steps=%lu  sim_time=%.3f s\n",
          static_cast<unsigned long>(step_count_.load()), sim_time_sec_.load());
}

// ── ViewerLoop ─────────────────────────────────────────────────────────────────
inline void MuJoCoSimulator::ViewerLoop(std::stop_token stop) noexcept {
#ifdef MUJOCO_HAVE_GLFW
  if (!glfwInit()) {
    fprintf(stderr, "[MuJoCoSimulator] glfwInit failed — viewer disabled\n");
    return;
  }

  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);

  GLFWwindow* window = glfwCreateWindow(
      1280, 960, "UR5e MuJoCo Simulator", nullptr, nullptr);
  if (!window) {
    fprintf(stderr, "[MuJoCoSimulator] glfwCreateWindow failed\n");
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // ── MuJoCo rendering ──────────────────────────────────────────────────────
  mjvCamera  cam;  mjv_defaultCamera(&cam);
  mjvOption  opt;  mjv_defaultOption(&opt);
  mjvScene   scn;  mjv_defaultScene(&scn);
  mjrContext con;  mjr_defaultContext(&con);

  mjv_makeScene(model_, &scn, 2000);
  mjr_makeContext(model_, &con, mjFONTSCALE_150);

  cam.type      = mjCAMERA_FREE;
  cam.distance  = 2.5;
  cam.azimuth   = 90.0;
  cam.elevation = -20.0;

  // ── Visualization-only mjData ─────────────────────────────────────────────
  mjData* vis_data = mj_makeData(model_);
  {
    std::lock_guard lock(state_mutex_);
    for (std::size_t i = 0; i < 6; ++i) {
      vis_data->qpos[joint_qpos_indices_[i]] = latest_positions_[i];
    }
  }
  mj_forward(model_, vis_data);

  // ── RTF profiler figure ───────────────────────────────────────────────────
  mjvFigure fig_profiler;
  mjv_defaultFigure(&fig_profiler);
  std::strncpy(fig_profiler.title,       "RTF History",
               sizeof(fig_profiler.title) - 1);
  std::strncpy(fig_profiler.xlabel,      "Frames (~60 Hz)",
               sizeof(fig_profiler.xlabel) - 1);
  std::strncpy(fig_profiler.linename[0], "RTF",
               sizeof(fig_profiler.linename[0]) - 1);
  fig_profiler.figurergba[0] = 0.08f;
  fig_profiler.figurergba[1] = 0.08f;
  fig_profiler.figurergba[2] = 0.08f;
  fig_profiler.figurergba[3] = 0.88f;
  fig_profiler.linergb[0][0] = 0.2f;
  fig_profiler.linergb[0][1] = 1.0f;
  fig_profiler.linergb[0][2] = 0.4f;
  fig_profiler.linewidth     = 1.5f;
  fig_profiler.range[0][0]   = 0;    fig_profiler.range[0][1] = 200;
  fig_profiler.range[1][0]   = 0;    fig_profiler.range[1][1] = 30;
  fig_profiler.flg_extend    = 1;

  // ── Viewer interaction state (shared with GLFW callbacks via user ptr) ────
  struct ViewerState {
    // MuJoCo handles (non-owning pointers, all local to ViewerLoop)
    mjvCamera*       cam;
    mjvOption*       opt;
    mjvScene*        scn;
    const mjModel*   model;
    mjData*          vis_data;
    MuJoCoSimulator* sim;

    // Mouse tracking
    bool   btn_left{false};
    bool   btn_right{false};
    bool   ctrl_held{false};
    double lastx{0.0};
    double lasty{0.0};

    // Perturbation state (local to viewer, transferred to sim via UpdatePerturb)
    mjvPerturb pert;

    // UI toggles
    bool show_profiler{false};
    bool show_help{false};
    bool show_solver{false};  // F4: solver stats overlay

    // RTF rolling buffer (200 samples at ~60 Hz ≈ 3.3 s window)
    static constexpr int kProfLen = 200;
    float rtf_history[kProfLen]{};
    int   rtf_head{0};
    int   rtf_count{0};

    void push_rtf(float v) noexcept {
      rtf_history[rtf_head] = v;
      rtf_head  = (rtf_head + 1) % kProfLen;
      if (rtf_count < kProfLen) { ++rtf_count; }
    }

    void update_figure(mjvFigure& fig) const noexcept {
      const int n      = rtf_count;
      if (n == 0) { return; }
      const int oldest = (rtf_count < kProfLen) ? 0 : rtf_head;
      for (int i = 0; i < n; ++i) {
        const int idx = (oldest + i) % kProfLen;
        fig.linedata[0][i * 2]     = static_cast<float>(i);
        fig.linedata[0][i * 2 + 1] = rtf_history[idx];
      }
      fig.linepnt[0] = n;
    }
  } vs{&cam, &opt, &scn, model_, vis_data, this};
  mjv_defaultPerturb(&vs.pert);

  glfwSetWindowUserPointer(window, &vs);

  // ── GLFW callbacks (captureless lambdas → implicit function pointers) ─────

  // Keyboard
  glfwSetKeyCallback(window,
    [](GLFWwindow* w, int key, int /*scan*/, int action, int /*mods*/) noexcept {
      auto* s = static_cast<ViewerState*>(glfwGetWindowUserPointer(w));
      if (!s) { return; }

      // Track Ctrl key — needs both PRESS and RELEASE.
      if (key == GLFW_KEY_LEFT_CONTROL || key == GLFW_KEY_RIGHT_CONTROL) {
        s->ctrl_held = (action != GLFW_RELEASE);
        if (!s->ctrl_held && s->pert.active) {
          s->pert.active = 0;
          s->sim->ClearPerturb();
        }
        return;
      }

      if (action == GLFW_RELEASE) { return; }

      switch (key) {
        // ── Help overlay ──────────────────────────────────────────────────
        case GLFW_KEY_F1:
          s->show_help = !s->show_help;
          break;

        // ── Simulation controls ──────────────────────────────────────────
        case GLFW_KEY_SPACE:
          if (s->sim->IsPaused()) { s->sim->Resume(); }
          else                    { s->sim->Pause();  }
          break;

        case GLFW_KEY_EQUAL:
        case GLFW_KEY_KP_ADD: {
          const double cur = s->sim->GetMaxRtf();
          s->sim->SetMaxRtf(cur <= 0.0 ? 2.0 : cur * 2.0);
          break;
        }
        case GLFW_KEY_MINUS:
        case GLFW_KEY_KP_SUBTRACT: {
          const double cur = s->sim->GetMaxRtf();
          if (cur > 0.0 && cur <= 0.5) { s->sim->SetMaxRtf(0.0); }
          else if (cur > 0.5)          { s->sim->SetMaxRtf(cur / 2.0); }
          break;
        }

        case GLFW_KEY_R:
          s->sim->RequestReset();
          break;

        // ── Physics toggles ───────────────────────────────────────────────
        case GLFW_KEY_G:
          s->sim->EnableGravity(!s->sim->IsGravityEnabled());
          fprintf(stdout, "[Viewer] Gravity %s\n",
                  s->sim->IsGravityEnabled() ? "ON" : "OFF");
          break;

        // ── Visualisation toggles ─────────────────────────────────────────
        case GLFW_KEY_C:
          // Contact point markers
          s->opt->flags[mjVIS_CONTACTPOINT] ^= 1;
          break;

        case GLFW_KEY_F:
          // Contact force arrows
          s->opt->flags[mjVIS_CONTACTFORCE] ^= 1;
          break;

        case GLFW_KEY_V:
          // Toggle collision geometry group 0
          s->opt->geomgroup[0] ^= 1;
          break;

        case GLFW_KEY_T:
          // Transparency for all bodies
          s->opt->flags[mjVIS_TRANSPARENT] ^= 1;
          break;

        // ── Physics solver controls ───────────────────────────────────────
        case GLFW_KEY_I: {
          // Cycle integrator: Euler → RK4 → Implicit → ImplicitFast → Euler
          static constexpr int kIntegrators[] = {
              mjINT_EULER, mjINT_RK4, mjINT_IMPLICIT, mjINT_IMPLICITFAST};
          static constexpr const char* kIntNames[] = {
              "Euler", "RK4", "Implicit", "ImplicitFast"};
          const int cur = s->sim->GetIntegrator();
          int next = 0;
          for (int k = 0; k < 4; ++k) {
            if (kIntegrators[k] == cur) { next = (k + 1) % 4; break; }
          }
          s->sim->SetIntegrator(kIntegrators[next]);
          fprintf(stdout, "[Viewer] Integrator → %s\n", kIntNames[next]);
          break;
        }

        case GLFW_KEY_S: {
          // Cycle solver: PGS → CG → Newton → PGS
          static constexpr int kSolvers[] = {mjSOL_PGS, mjSOL_CG, mjSOL_NEWTON};
          static constexpr const char* kSolNames[] = {"PGS", "CG", "Newton"};
          const int cur = s->sim->GetSolverType();
          int next = 0;
          for (int k = 0; k < 3; ++k) {
            if (kSolvers[k] == cur) { next = (k + 1) % 3; break; }
          }
          s->sim->SetSolverType(kSolvers[next]);
          fprintf(stdout, "[Viewer] Solver → %s\n", kSolNames[next]);
          break;
        }

        case GLFW_KEY_RIGHT_BRACKET:
          // ] → double solver iterations (max 1000)
          s->sim->SetSolverIterations(s->sim->GetSolverIterations() * 2);
          fprintf(stdout, "[Viewer] Solver iterations → %d\n",
                  s->sim->GetSolverIterations());
          break;

        case GLFW_KEY_LEFT_BRACKET:
          // [ → halve solver iterations (min 1)
          s->sim->SetSolverIterations(s->sim->GetSolverIterations() / 2);
          fprintf(stdout, "[Viewer] Solver iterations → %d\n",
                  s->sim->GetSolverIterations());
          break;

        case GLFW_KEY_N:
          // Toggle contact constraints
          s->sim->SetContactEnabled(!s->sim->IsContactEnabled());
          fprintf(stdout, "[Viewer] Contacts %s\n",
                  s->sim->IsContactEnabled() ? "ENABLED" : "DISABLED");
          break;

        // ── Viewer controls ───────────────────────────────────────────────
        case GLFW_KEY_F3:
          s->show_profiler = !s->show_profiler;
          break;

        case GLFW_KEY_F4:
          s->show_solver = !s->show_solver;
          break;

        case GLFW_KEY_BACKSPACE:
          mjv_defaultOption(s->opt);
          break;

        case GLFW_KEY_ESCAPE:
          mjv_defaultCamera(s->cam);
          s->cam->distance  = 2.5;
          s->cam->azimuth   = 90.0;
          s->cam->elevation = -20.0;
          break;

        default: break;
      }
    });

  // Mouse buttons — orbit/pan or perturbation selection (Ctrl held)
  glfwSetMouseButtonCallback(window,
    [](GLFWwindow* w, int button, int action, int /*mods*/) noexcept {
      auto* s = static_cast<ViewerState*>(glfwGetWindowUserPointer(w));
      if (!s) { return; }
      const bool pressed = (action == GLFW_PRESS);

      if (s->ctrl_held && pressed && button == GLFW_MOUSE_BUTTON_LEFT) {
        // ── Ctrl+Left: body selection for perturbation ───────────────────
        double xp = 0.0, yp = 0.0;
        glfwGetCursorPos(w, &xp, &yp);
        int width = 0, height = 0;
        glfwGetWindowSize(w, &width, &height);
        if (width > 0 && height > 0) {
          mjtNum selpnt[3] = {};
          int selgeom = -1, selskin = -1;
          const int selobj = mjv_select(
              s->model, s->vis_data, s->opt,
              static_cast<mjtNum>(width) / static_cast<mjtNum>(height),
              static_cast<mjtNum>(xp)   / static_cast<mjtNum>(width),
              static_cast<mjtNum>(height - yp) / static_cast<mjtNum>(height),
              s->scn, selpnt, &selgeom, &selskin);

          if (selobj > 0) {  // > 0: a UR5e link (0 = world body)
            s->pert.select = selobj;
            mju_copy(s->pert.localpos, selpnt, 3);
            mjv_initPerturb(s->model, s->vis_data, s->scn, &s->pert);
            s->pert.active = mjPERT_TRANSLATE;
            s->sim->UpdatePerturb(s->pert);
            fprintf(stdout, "[Viewer] Perturbing body %d\n", selobj);
          }
        }
      } else if (!pressed && button == GLFW_MOUSE_BUTTON_LEFT && s->pert.active) {
        // ── Release left button: clear perturbation ───────────────────────
        s->pert.active = 0;
        s->sim->ClearPerturb();
      }

      if (!s->ctrl_held) {
        if (button == GLFW_MOUSE_BUTTON_LEFT)  { s->btn_left  = pressed; }
        if (button == GLFW_MOUSE_BUTTON_RIGHT) { s->btn_right = pressed; }
      }
      if (pressed) { glfwGetCursorPos(w, &s->lastx, &s->lasty); }
    });

  // Cursor move — camera orbit/pan, or perturbation drag (Ctrl)
  glfwSetCursorPosCallback(window,
    [](GLFWwindow* w, double xpos, double ypos) noexcept {
      auto* s = static_cast<ViewerState*>(glfwGetWindowUserPointer(w));
      if (!s) { return; }

      int width = 0, height = 0;
      glfwGetWindowSize(w, &width, &height);
      if (width == 0 || height == 0) { s->lastx = xpos; s->lasty = ypos; return; }

      const double dx = xpos - s->lastx;
      const double dy = ypos - s->lasty;
      s->lastx = xpos;
      s->lasty = ypos;
      const double nx = dx / static_cast<double>(width);
      const double ny = dy / static_cast<double>(height);

      if (s->ctrl_held && s->pert.active) {
        // ── Ctrl+drag: move perturbation reference point ──────────────────
        mjv_movePerturb(s->model, s->vis_data,
                        mjMOUSE_MOVE_H, nx, 0.0, s->scn, &s->pert);
        mjv_movePerturb(s->model, s->vis_data,
                        mjMOUSE_MOVE_V, 0.0, ny, s->scn, &s->pert);
        s->sim->UpdatePerturb(s->pert);
      } else if (s->btn_left) {
        // ── Left drag: orbit ───────────────────────────────────────────────
        mjv_moveCamera(s->model, mjMOUSE_ROTATE_H, nx, 0.0, s->scn, s->cam);
        mjv_moveCamera(s->model, mjMOUSE_ROTATE_V, 0.0, ny, s->scn, s->cam);
      } else if (s->btn_right) {
        // ── Right drag: pan ────────────────────────────────────────────────
        mjv_moveCamera(s->model, mjMOUSE_MOVE_H, nx, 0.0, s->scn, s->cam);
        mjv_moveCamera(s->model, mjMOUSE_MOVE_V, 0.0, ny, s->scn, s->cam);
      }
    });

  // Scroll — zoom
  glfwSetScrollCallback(window,
    [](GLFWwindow* w, double /*xoffset*/, double yoffset) noexcept {
      auto* s = static_cast<ViewerState*>(glfwGetWindowUserPointer(w));
      if (!s) { return; }
      mjv_moveCamera(s->model, mjMOUSE_ZOOM, 0.0,
                     -0.05 * yoffset, s->scn, s->cam);
    });

  fprintf(stdout,
          "[MuJoCoSimulator] Viewer ready — press F1 in the window for help\n"
          "  Simulation: Space=pause  +/-=speed  R=reset\n"
          "  Physics:    G=gravity  N=contacts\n"
          "  Solver:     I=integrator  S=solver  ]/[=iterations  F4=stats\n"
          "  Visualise:  C=cpoints  F=cforces  V=geoms  T=transp  F3=profiler\n"
          "  Camera:     Left-drag=orbit  Right-drag=pan  Scroll=zoom  Esc=reset\n"
          "  Perturb:    Ctrl+Left-drag=apply force to body\n");

  // ── Render loop ────────────────────────────────────────────────────────────
  while (!stop.stop_requested() && running_.load() &&
         !glfwWindowShouldClose(window)) {

    // Sync physics state to vis_data
    int ncon_snap = 0;
    {
      std::lock_guard lock(viz_mutex_);
      if (viz_dirty_) {
        std::memcpy(vis_data->qpos, viz_qpos_.data(),
                    static_cast<std::size_t>(model_->nq) * sizeof(double));
        viz_dirty_ = false;
      }
      ncon_snap = viz_ncon_;
    }
    mj_forward(model_, vis_data);

    // Sample RTF into rolling buffer
    const float cur_rtf = static_cast<float>(rtf_.load(std::memory_order_relaxed));
    vs.push_rtf(cur_rtf);
    vs.update_figure(fig_profiler);

    // ── Render ───────────────────────────────────────────────────────────────
    int width = 0, height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    mjrRect viewport{0, 0, width, height};

    // Pass &vs.pert so MuJoCo renders the perturbation grab indicator
    mjv_updateScene(model_, vis_data, &opt, &vs.pert, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // ── Status overlay (top-right) ────────────────────────────────────────────
    {
      const double max_rtf_val = current_max_rtf_.load(std::memory_order_relaxed);
      const bool   is_paused   = paused_.load(std::memory_order_relaxed);
      const bool   grav_on     = gravity_enabled_.load(std::memory_order_relaxed);
      const bool   perturbing  = (vs.pert.active != 0);

      char limit_str[32];
      if (max_rtf_val > 0.0) {
        std::snprintf(limit_str, sizeof(limit_str), "%.1fx", max_rtf_val);
      } else {
        std::strncpy(limit_str, "unlimited", sizeof(limit_str) - 1);
        limit_str[sizeof(limit_str) - 1] = '\0';
      }

      // Solver stats snapshot
      const SolverStats ss = GetSolverStats();
      static constexpr const char* kIntNames[]  = {"Euler","RK4","Implicit","ImplFast"};
      static constexpr const char* kSolNames[]  = {"PGS","CG","Newton"};
      const int int_idx = std::max(0, std::min(GetIntegrator(), 3));
      const int sol_idx = std::max(0, std::min(GetSolverType(), 2));

      char labels[512], values[512];
      std::snprintf(labels, sizeof(labels),
                    "Mode\nRTF\nLimit\nSim Time\nSteps\nContacts\nGravity\nStatus\n"
                    "Integrator\nSolver\nIterations\nResidual");
      std::snprintf(values, sizeof(values),
                    "%s\n%.1fx\n%s\n%.2f s\n%lu\n%d/%s\n%s\n%s\n"
                    "%s\n%s\n%d/%d\n%.2e",
                    cfg_.mode == SimMode::kFreeRun ? "free_run" : "sync_step",
                    static_cast<double>(cur_rtf),
                    limit_str,
                    sim_time_sec_.load(std::memory_order_relaxed),
                    static_cast<unsigned long>(step_count_.load()),
                    ss.ncon,
                    contacts_enabled_.load(std::memory_order_relaxed) ? "on" : "OFF",
                    grav_on   ? "ON"     : "OFF",
                    is_paused ? "PAUSED" : (perturbing ? "perturb" : "running"),
                    kIntNames[int_idx],
                    kSolNames[sol_idx],
                    ss.iter, GetSolverIterations(),
                    ss.improvement);
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport,
                  labels, values, &con);
    }

    // ── Help hint (bottom-left, always visible) ───────────────────────────────
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport,
                "F1: toggle help",
                nullptr, &con);

    // ── Detailed help overlay (top-left, F1 toggle) ───────────────────────────
    if (vs.show_help) {
      const double max_rtf_now = current_max_rtf_.load(std::memory_order_relaxed);
      char rtf_str[32];
      if (max_rtf_now > 0.0) {
        std::snprintf(rtf_str, sizeof(rtf_str), "%.1fx", max_rtf_now);
      } else {
        std::strncpy(rtf_str, "unlimited", sizeof(rtf_str) - 1);
        rtf_str[sizeof(rtf_str) - 1] = '\0';
      }

      // Current solver labels for help overlay
      static constexpr const char* kHelpIntNames[] =
          {"Euler","RK4","Implicit","ImplFast"};
      static constexpr const char* kHelpSolNames[] = {"PGS","CG","Newton"};
      const int h_int = std::max(0, std::min(GetIntegrator(), 3));
      const int h_sol = std::max(0, std::min(GetSolverType(), 2));

      char help_keys[768], help_vals[768];
      std::snprintf(help_keys, sizeof(help_keys),
          "── Simulation ──\n"
          "Space\n"
          "+  /  KP_ADD\n"
          "-  /  KP_SUB\n"
          "R\n"
          "── Physics ──\n"
          "G\n"
          "N\n"
          "── Solver ──\n"
          "I\n"
          "S\n"
          "]  /  [\n"
          "F4\n"
          "── Visualisation ──\n"
          "C\n"
          "F\n"
          "V\n"
          "T\n"
          "F3\n"
          "Backspace\n"
          "Escape\n"
          "── Camera ──\n"
          "Left drag\n"
          "Right drag\n"
          "Scroll\n"
          "── Perturb ──\n"
          "Ctrl + Left drag\n"
          "── Help ──\n"
          "F1");
      std::snprintf(help_vals, sizeof(help_vals),
          "\n"
          "Pause / Resume\n"
          "2x speed  [now %s]\n"
          "0.5x speed\n"
          "Reset to initial pose\n"
          "\n"
          "Toggle gravity  [%s]\n"
          "Toggle contacts  [%s]\n"
          "\n"
          "Cycle integrator  [%s]\n"
          "Cycle solver type  [%s]\n"
          "Increase / decrease iterations  [%d]\n"
          "Toggle solver stats overlay  [%s]\n"
          "\n"
          "Toggle contact points  [%s]\n"
          "Toggle contact forces  [%s]\n"
          "Toggle collision geoms  [%s]\n"
          "Toggle transparency  [%s]\n"
          "Toggle RTF profiler  [%s]\n"
          "Reset visualisation options\n"
          "Reset camera to default\n"
          "\n"
          "Orbit camera\n"
          "Pan camera\n"
          "Zoom in / out\n"
          "\n"
          "Apply spring force to body\n"
          "\n"
          "Hide help",
          rtf_str,
          gravity_enabled_.load(std::memory_order_relaxed) ? "ON"  : "OFF",
          contacts_enabled_.load(std::memory_order_relaxed) ? "ON" : "OFF",
          kHelpIntNames[h_int],
          kHelpSolNames[h_sol],
          GetSolverIterations(),
          vs.show_solver                  ? "ON" : "OFF",
          (opt.flags[mjVIS_CONTACTPOINT]) ? "ON" : "OFF",
          (opt.flags[mjVIS_CONTACTFORCE]) ? "ON" : "OFF",
          (opt.geomgroup[0])              ? "ON" : "OFF",
          (opt.flags[mjVIS_TRANSPARENT])  ? "ON" : "OFF",
          vs.show_profiler                ? "ON" : "OFF");

      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
                  help_keys, help_vals, &con);
    }

    // ── Solver stats overlay (bottom-left extended, F4) ─────────────────────
    if (vs.show_solver) {
      const SolverStats ss2 = GetSolverStats();
      static constexpr const char* kOvIntNames[] =
          {"Euler","RK4","Implicit","ImplFast"};
      static constexpr const char* kOvSolNames[] = {"PGS","CG","Newton"};
      const int oi = std::max(0, std::min(GetIntegrator(), 3));
      const int os = std::max(0, std::min(GetSolverType(), 2));

      char sk[256], sv[256];
      std::snprintf(sk, sizeof(sk),
                    "Integrator\nSolver\nMax iter\nUsed iter\n"
                    "Improvement\nGradient\nContacts\nTimestep");
      std::snprintf(sv, sizeof(sv),
                    "%s\n%s\n%d\n%d\n"
                    "%.3e\n%.3e\n%d\n%.4f ms",
                    kOvIntNames[oi], kOvSolNames[os],
                    GetSolverIterations(), ss2.iter,
                    ss2.improvement, ss2.gradient,
                    ss2.ncon,
                    model_ ? static_cast<double>(model_->opt.timestep) * 1e3 : 0.0);
      mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport, sk, sv, &con);
    }

    // ── Profiler (bottom-right, F3) ───────────────────────────────────────────
    if (vs.show_profiler && vs.rtf_count > 0) {
      const int fw = width  / 3;
      const int fh = height / 3;
      mjr_figure(mjrRect{width - fw, 0, fw, fh}, &fig_profiler, &con);
    }

    glfwSwapBuffers(window);
    glfwPollEvents();
    std::this_thread::sleep_for(std::chrono::milliseconds(16));  // ~60 Hz
  }

  mj_deleteData(vis_data);
  mjv_freeScene(&scn);
  mjr_freeContext(&con);
  glfwDestroyWindow(window);
  glfwTerminate();
  fprintf(stdout, "[MuJoCoSimulator] Viewer closed\n");

#else
  fprintf(stdout,
          "[MuJoCoSimulator] Viewer not available (MUJOCO_HAVE_GLFW not set)\n");
  (void)stop;
#endif
}

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_CONTROLLER_MUJOCO_SIMULATOR_HPP_
