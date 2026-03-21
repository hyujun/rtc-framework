#ifndef RTC_MUJOCO_SIM_MUJOCO_SIMULATOR_HPP_
#define RTC_MUJOCO_SIM_MUJOCO_SIMULATOR_HPP_

// ── Includes: project, then MuJoCo, then C++ stdlib ───────────────────────────
#include <mujoco/mujoco.h>

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

namespace rtc {

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

    // ── Physics timestep validation & position servo gain scaling ────────────
    // 0.0 → XML 값 그대로 사용 (검증 없음)
    // > 0 → XML <option timestep>과 비교; 불일치 시 ERROR + XML 우선 적용
    // gainprm_yaml = servo_kp / physics_timestep (position servo 시 사용)
    double physics_timestep{0.0};

    // ── Position servo gains (use_yaml_servo_gains = true 시 적용) ───────────
    // false: XML 원본 gainprm/biasprm 사용
    // true : servo_kp / physics_timestep → gainprm, -servo_kd → biasprm[2]
    bool   use_yaml_servo_gains{false};
    // servo_kp: velocity gain (Nm·s/rad), force = servo_kp*dq_cmd - servo_kd*dq_actual
    // 크기는 XML에서 발견된 로봇 조인트 수와 일치해야 함
    std::vector<double> servo_kp{500.0, 500.0, 500.0, 150.0, 150.0, 150.0};
    std::vector<double> servo_kd{400.0, 400.0, 400.0, 100.0, 100.0, 100.0};

    // ── Robot topics (fake_hand_response 패턴과 동일) ─────────────────────────
    // 로봇 커맨드 수신 토픽 (JointCommand msg, command_type으로 position/torque 결정)
    std::string command_topic{"/ur5e/joint_command"};
    // 로봇 상태 퍼블리시 토픽
    std::string state_topic{"/joint_states"};
  };

  // Invoked from SimLoop after each publish step.
  // efforts = data_->qfrc_actuator (actuator forces in Nm, indexed by DOF).
  using StateCallback = std::function<void(
      const std::vector<double>& positions,
      const std::vector<double>& velocities,
      const std::vector<double>& efforts)>;

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
  void SetCommand(const std::vector<double>& cmd) noexcept;

  // Register the state callback (positions, velocities, efforts).
  void SetStateCallback(StateCallback cb) noexcept;

  [[nodiscard]] std::vector<double> GetPositions()  const noexcept;
  [[nodiscard]] std::vector<double> GetVelocities() const noexcept;
  [[nodiscard]] std::vector<double> GetEfforts()    const noexcept;

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

  // Advance exactly one physics step while paused.
  // Wakes the sim thread immediately via sync_cv_.
  void StepOnce() noexcept {
    step_once_.store(true, std::memory_order_release);
    sync_cv_.notify_all();
  }

  // Return the simulation mode (kFreeRun / kSyncStep) set at construction.
  [[nodiscard]] SimMode GetSimMode() const noexcept { return cfg_.mode; }

  // Toggle gravity.  Sim thread reads gravity_enabled_ before each mj_step().
  // Position servo 모드에서는 gravity가 잠겨 있으므로 호출이 무시됩니다.
  void EnableGravity(bool enable) noexcept {
    if (gravity_locked_by_servo_.load(std::memory_order_relaxed)) {
      return;  // position servo 모드 중 gravity 변경 차단
    }
    gravity_enabled_.store(enable, std::memory_order_relaxed);
  }
  [[nodiscard]] bool IsGravityEnabled() const noexcept {
    return gravity_enabled_.load(std::memory_order_relaxed);
  }
  [[nodiscard]] bool IsGravityLockedByServo() const noexcept {
    return gravity_locked_by_servo_.load(std::memory_order_relaxed);
  }

  // Position command를 수신할 때마다 호출하여 gravity OFF + lock을 보장합니다.
  // SetControlMode(false)와 달리 actuator 파라미터를 재설정하지 않으므로 경량입니다.
  void EnforcePositionServoGravity() noexcept {
    gravity_enabled_.store(false, std::memory_order_relaxed);
    gravity_locked_by_servo_.store(true, std::memory_order_relaxed);
  }

  // Switch actuator mode: true = direct torque, false = position servo.
  // The change is picked up by the sim thread before the next physics step.
  //
  // Gravity management:
  //   → position servo (false): gravity OFF + 변경 잠금
  //   → torque         (true) : 잠금 해제  + gravity ON (servo로 꺼진 경우)
  void SetControlMode(bool torque_mode) noexcept {
    const bool was_locked =
        gravity_locked_by_servo_.load(std::memory_order_relaxed);
    torque_mode_.store(torque_mode, std::memory_order_relaxed);
    control_mode_pending_.store(true, std::memory_order_release);

    if (!torque_mode) {
      // → Position servo: gravity OFF + 잠금
      gravity_enabled_.store(false, std::memory_order_relaxed);
      gravity_locked_by_servo_.store(true, std::memory_order_relaxed);
    } else {
      // → Torque: 잠금 해제, gravity ON (servo로 꺼진 경우만)
      gravity_locked_by_servo_.store(false, std::memory_order_relaxed);
      if (was_locked) {
        gravity_enabled_.store(true, std::memory_order_relaxed);
      }
    }
  }
  [[nodiscard]] bool IsInTorqueMode() const noexcept {
    return torque_mode_.load(std::memory_order_relaxed);
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
  [[nodiscard]] const std::vector<std::string>& GetJointNames() const noexcept {
    return joint_names_;
  }

  [[nodiscard]] bool     IsRunning()  const noexcept { return running_.load(); }
  [[nodiscard]] uint64_t StepCount()  const noexcept { return step_count_.load(); }
  [[nodiscard]] double   SimTimeSec() const noexcept { return sim_time_sec_.load(); }
  [[nodiscard]] int      NumJoints()  const noexcept { return model_ ? model_->nq : 0; }
  [[nodiscard]] int      NumRobotJoints() const noexcept { return num_robot_joints_; }
  [[nodiscard]] double   GetRtf()     const noexcept {
    return rtf_.load(std::memory_order_relaxed);
  }
  [[nodiscard]] double   GetPhysicsTimestep() const noexcept { return xml_timestep_; }

  // Re-resolve joint indices from a new set of names (e.g. from command message).
  // Returns true if all names were found in XML.
  bool ResolveJointIndices(const std::vector<std::string>& names) noexcept;

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
  std::atomic<bool>   step_once_{false};       // advance exactly one step while paused
  std::atomic<double> current_max_rtf_{0.0};
  std::atomic<bool>   gravity_enabled_{false};  // 초기값: position servo → gravity OFF
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
  std::vector<double>   pending_cmd_{};

  std::mutex              sync_mutex_;
  std::condition_variable sync_cv_;

  // ── State buffer (under state_mutex_) ─────────────────────────────────────
  mutable std::mutex    state_mutex_;
  std::vector<double>   latest_positions_{};
  std::vector<double>   latest_velocities_{};
  std::vector<double>   latest_efforts_{};   // qfrc_actuator per joint DOF

  // ── Viewer double-buffer ──────────────────────────────────────────────────
  mutable std::mutex  viz_mutex_;
  std::vector<double> viz_qpos_{};
  int                 viz_ncon_{0};
  bool                viz_dirty_{false};

  StateCallback state_cb_{nullptr};

  std::jthread sim_thread_;
  std::jthread viewer_thread_;

  // XML에서 발견된 로봇 조인트 수 (Initialize()에서 설정)
  int num_robot_joints_{0};

  std::vector<int> joint_qpos_indices_{};
  std::vector<int> joint_qvel_indices_{};

  // XML에서 자동 발견된 조인트 이름 (Initialize()에서 설정)
  std::vector<std::string> joint_names_;

  // XML keyframe 또는 0으로 초기화된 초기 관절 위치
  std::vector<double> initial_qpos_{};

  // ── RTF measurement ───────────────────────────────────────────────────────
  std::chrono::steady_clock::time_point rtf_wall_start_{};
  double                                rtf_sim_start_{0.0};
  std::atomic<double>                   rtf_{0.0};

  // ── Max-RTF throttle (sim thread only) ───────────────────────────────────
  std::chrono::steady_clock::time_point throttle_wall_start_{};
  double                                throttle_sim_start_{0.0};
  double                                throttle_rtf_{0.0};

  // ── Actuator mode control (sim thread reads, caller writes) ──────────────
  std::atomic<bool> torque_mode_{false};
  std::atomic<bool> control_mode_pending_{false};

  // ── Gravity lock (position servo 모드에서 gravity 변경 차단) ─────────────
  // true: position servo 모드 → EnableGravity() 호출 무시
  std::atomic<bool> gravity_locked_by_servo_{true};  // 초기값: position servo

  // ── YAML servo gains (Initialize()에서 physics_timestep 기반으로 계산) ───
  // gainprm_yaml_[i]  = servo_kp[i] / xml_timestep_
  // biasprm2_yaml_[i] = -servo_kd[i]
  double xml_timestep_{0.002};
  std::vector<double> gainprm_yaml_{};
  std::vector<double> biasprm2_yaml_{};
  struct ActuatorParams {
    double gainprm0{0.0};
    double biasprm0{0.0};
    double biasprm1{0.0};
    double biasprm2{0.0};
  };
  std::vector<ActuatorParams> orig_actuator_params_;  // sized in Initialize()

  // ── External forces / perturbation (under pert_mutex_) ───────────────────
  mutable std::mutex pert_mutex_;
  mjvPerturb         shared_pert_{};     // viewer → sim perturbation
  bool               pert_active_{false};
  // Flat xfrc_applied buffer: 6 * nbody doubles.
  // Used by SetExternalForce(); shared_pert_ is applied separately.
  std::vector<double> ext_xfrc_{};
  bool                ext_xfrc_dirty_{false};

  // ── Internal helpers ───────────────────────────────────────────────────────
  // XML에서 hinge joint + actuator 연결된 조인트 자동 발견
  bool DiscoverRobotJoints() noexcept;
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

}  // namespace rtc

#endif  // RTC_MUJOCO_SIM_MUJOCO_SIMULATOR_HPP_
