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

// ── JointGroupConfig ─────────────────────────────────────────────────────────
// Per-group configuration loaded from YAML (robot_response / fake_response).

struct JointGroupConfig {
  std::string name;                       // 임의 이름 (ur5e, hand, kuka, ...)
  std::vector<std::string> joint_names;   // 하위호환: command/state 미지정 시 사용
  std::vector<std::string> command_joint_names;  // command용 joint names (빈 경우 joint_names 사용)
  std::vector<std::string> state_joint_names;    // state용 joint names (빈 경우 XML 전체)
  std::string command_topic;
  std::string state_topic;
  bool is_robot{true};                    // true=robot_response, false=fake_response
  double filter_alpha{0.1};               // fake_response용 LPF 계수
  std::vector<double> servo_kp;           // 비어있으면 글로벌 값 상속
  std::vector<double> servo_kd;
};

// ── JointGroup ───────────────────────────────────────────────────────────────
// Runtime state for a single joint group (robot or fake).

struct JointGroup {
  std::string name;

  // ── Command joints (actuator 매칭용) ──────────────────────────────
  std::vector<std::string> command_joint_names;
  int num_command_joints{0};

  // ── State joints (state publish용) ────────────────────────────────
  std::vector<std::string> state_joint_names;
  int num_state_joints{0};

  bool is_robot{true};                    // robot_response 여부
  bool is_primary{false};                 // sync_step 대기 대상

  // ── MuJoCo 인덱스: command용 (is_robot==true, 이름 기반 비연속 가능)
  std::vector<int> qpos_indices;
  std::vector<int> qvel_indices;
  std::vector<int> actuator_indices;

  // ── MuJoCo 인덱스: state용 ────────────────────────────────────────
  std::vector<int> state_qpos_indices;
  std::vector<int> state_qvel_indices;

  // ── Command 버퍼 ──────────────────────────────────────────────────
  std::vector<double> pending_cmd;
  std::vector<double> initial_qpos;

  // ── State 버퍼 (state_joint_names 기준) ───────────────────────────
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
  std::atomic<bool>   cmd_pending{false};
  mutable std::mutex  cmd_mutex;
  mutable std::mutex  state_mutex;

  // ── Per-group control mode ──────────────────────────────────────
  std::atomic<bool> torque_mode{false};
  std::atomic<bool> control_mode_pending{false};

  // ── Per-group servo gains ───────────────────────────────────────
  std::vector<double> gainprm_yaml;
  std::vector<double> biasprm2_yaml;

  // ── State callback ──────────────────────────────────────────────
  using StateCallback = std::function<void(
      const std::vector<double>& positions,
      const std::vector<double>& velocities,
      const std::vector<double>& efforts)>;
  StateCallback state_cb{nullptr};

  // ── Fake response (is_robot==false일 때) ────────────────────────
  double filter_alpha{0.1};
  std::vector<double> fake_state;
  std::vector<double> fake_target;
  mutable std::mutex  fake_mutex;

  // ── ROS2 토픽 ──────────────────────────────────────────────────
  std::string command_topic;
  std::string state_topic;

  // Non-copyable, non-movable (due to mutex members)
  JointGroup() = default;
  JointGroup(const JointGroup&) = delete;
  JointGroup& operator=(const JointGroup&) = delete;
  JointGroup(JointGroup&&) = delete;
  JointGroup& operator=(JointGroup&&) = delete;
};

// ── MuJoCoSimulator ────────────────────────────────────────────────────────────
//
// Thread-safe wrapper around a MuJoCo physics model with multi-group support.
//
// Simulation modes:
//   kFreeRun  — advances mj_step() as fast as possible (up to max_rtf).
//   kSyncStep — publishes state, waits for one command, steps once.
//               Step latency ≈ controller Compute() time.
//
// Joint groups (robot_response / fake_response):
//   Each group has independent command/state buffers, control mode, and topics.
//   robot groups use MuJoCo physics; fake groups use LPF echo-back.
//
// Threading model:
//   SimLoop thread  — physics; sole writer of model_/data_
//   ViewerLoop thread — renders at ~60 Hz via GLFW (optional)
//   Caller thread   — SetCommand(), GetPositions(), SetExternalForce(), etc.
//
class MuJoCoSimulator {
 public:
  enum class SimMode {
    kFreeRun,   // Maximum speed (throttled by max_rtf)
    kSyncStep,  // 1:1 synchronised with controller commands
  };

  using StateCallback = JointGroup::StateCallback;

  struct Config {
    std::string model_path;
    SimMode     mode{SimMode::kFreeRun};
    bool        enable_viewer{true};
    int         publish_decimation{1};   // kFreeRun: publish every N steps
    double      sync_timeout_ms{50.0};   // kSyncStep: command wait timeout
    double      max_rtf{0.0};           // 0.0 = unlimited
    double      physics_timestep{0.0};

    // 글로벌 servo gain (그룹별 미지정 시 상속)
    bool   use_yaml_servo_gains{false};
    std::vector<double> servo_kp{500.0, 500.0, 500.0, 150.0, 150.0, 150.0};
    std::vector<double> servo_kd{400.0, 400.0, 400.0, 100.0, 100.0, 100.0};

    // 멀티 그룹 설정 (robot_response + fake_response)
    std::vector<JointGroupConfig> groups;
  };

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

  // ── Group-indexed API ─────────────────────────────────────────────────────

  // Write a command into the pending buffer for a specific group.
  void SetCommand(std::size_t group_idx, const std::vector<double>& cmd) noexcept;

  // Register the state callback for a specific group.
  void SetStateCallback(std::size_t group_idx, StateCallback cb) noexcept;

  [[nodiscard]] std::vector<double> GetPositions(std::size_t group_idx)  const noexcept;
  [[nodiscard]] std::vector<double> GetVelocities(std::size_t group_idx) const noexcept;
  [[nodiscard]] std::vector<double> GetEfforts(std::size_t group_idx)    const noexcept;

  [[nodiscard]] const std::vector<std::string>& GetJointNames(std::size_t group_idx) const noexcept;
  [[nodiscard]] const std::vector<std::string>& GetStateJointNames(std::size_t group_idx) const noexcept;
  [[nodiscard]] int  NumGroupJoints(std::size_t group_idx) const noexcept;
  [[nodiscard]] int  NumStateJoints(std::size_t group_idx) const noexcept;
  [[nodiscard]] bool IsGroupRobot(std::size_t group_idx) const noexcept;
  [[nodiscard]] std::size_t NumGroups() const noexcept { return groups_.size(); }

  // Per-group control mode (robot groups only).
  void SetControlMode(std::size_t group_idx, bool torque_mode) noexcept;
  [[nodiscard]] bool IsInTorqueMode(std::size_t group_idx) const noexcept;

  // Per-group gravity enforcement for position servo.
  void EnforcePositionServoGravity() noexcept {
    gravity_enabled_.store(false, std::memory_order_relaxed);
    gravity_locked_by_servo_.store(true, std::memory_order_relaxed);
  }

  // ── Backward-compatible API (delegates to group 0) ────────────────────────

  void SetCommand(const std::vector<double>& cmd) noexcept { SetCommand(0, cmd); }
  void SetStateCallback(StateCallback cb) noexcept { SetStateCallback(0, std::move(cb)); }
  [[nodiscard]] std::vector<double> GetPositions()  const noexcept { return GetPositions(0); }
  [[nodiscard]] std::vector<double> GetVelocities() const noexcept { return GetVelocities(0); }
  [[nodiscard]] std::vector<double> GetEfforts()    const noexcept { return GetEfforts(0); }
  [[nodiscard]] const std::vector<std::string>& GetJointNames() const noexcept { return GetJointNames(0); }
  [[nodiscard]] int NumRobotJoints() const noexcept { return NumGroupJoints(0); }
  void SetControlMode(bool torque_mode) noexcept { SetControlMode(0, torque_mode); }
  [[nodiscard]] bool IsInTorqueMode() const noexcept { return IsInTorqueMode(0); }

  // ── Fake response API (Node 타이머에서 호출) ──────────────────────────────

  void SetFakeTarget(std::size_t group_idx, const std::vector<double>& target) noexcept;
  void AdvanceFakeLPF(std::size_t group_idx) noexcept;
  [[nodiscard]] std::vector<double> GetFakeState(std::size_t group_idx) const noexcept;

  // ── Solver statistics ─────────────────────────────────────────────────────

  struct SolverStats {
    double improvement{0.0};
    double gradient{0.0};
    int    iter{0};
    int    ncon{0};
  };
  [[nodiscard]] SolverStats GetSolverStats() const noexcept;

  // ── Physics solver controls (thread-safe) ─────────────────────────────────

  void SetIntegrator(int type) noexcept {
    solver_integrator_.store(type, std::memory_order_relaxed);
  }
  [[nodiscard]] int GetIntegrator() const noexcept {
    return solver_integrator_.load(std::memory_order_relaxed);
  }

  void SetSolverType(int type) noexcept {
    solver_type_.store(type, std::memory_order_relaxed);
  }
  [[nodiscard]] int GetSolverType() const noexcept {
    return solver_type_.load(std::memory_order_relaxed);
  }

  void SetSolverIterations(int iters) noexcept {
    solver_iterations_.store(
        std::max(1, std::min(iters, 1000)), std::memory_order_relaxed);
  }
  [[nodiscard]] int GetSolverIterations() const noexcept {
    return solver_iterations_.load(std::memory_order_relaxed);
  }

  void SetSolverTolerance(double tol) noexcept {
    solver_tolerance_.store(tol < 0.0 ? 0.0 : tol, std::memory_order_relaxed);
  }
  [[nodiscard]] double GetSolverTolerance() const noexcept {
    return solver_tolerance_.load(std::memory_order_relaxed);
  }

  void SetContactEnabled(bool enabled) noexcept {
    contacts_enabled_.store(enabled, std::memory_order_relaxed);
  }
  [[nodiscard]] bool IsContactEnabled() const noexcept {
    return contacts_enabled_.load(std::memory_order_relaxed);
  }

  // ── Physics controls (thread-safe) ────────────────────────────────────────

  void Pause()   noexcept { paused_.store(true,  std::memory_order_relaxed); }
  void Resume()  noexcept {
    paused_.store(false, std::memory_order_relaxed);
    sync_cv_.notify_all();
  }
  [[nodiscard]] bool IsPaused() const noexcept {
    return paused_.load(std::memory_order_relaxed);
  }

  void RequestReset() noexcept {
    reset_requested_.store(true, std::memory_order_relaxed);
    sync_cv_.notify_all();
  }

  void SetMaxRtf(double rtf) noexcept {
    current_max_rtf_.store(rtf < 0.0 ? 0.0 : rtf, std::memory_order_relaxed);
  }
  [[nodiscard]] double GetMaxRtf() const noexcept {
    return current_max_rtf_.load(std::memory_order_relaxed);
  }

  void StepOnce() noexcept {
    step_once_.store(true, std::memory_order_release);
    sync_cv_.notify_all();
  }

  [[nodiscard]] SimMode GetSimMode() const noexcept { return cfg_.mode; }

  void EnableGravity(bool enable) noexcept {
    if (gravity_locked_by_servo_.load(std::memory_order_relaxed)) {
      return;
    }
    gravity_enabled_.store(enable, std::memory_order_relaxed);
  }
  [[nodiscard]] bool IsGravityEnabled() const noexcept {
    return gravity_enabled_.load(std::memory_order_relaxed);
  }
  [[nodiscard]] bool IsGravityLockedByServo() const noexcept {
    return gravity_locked_by_servo_.load(std::memory_order_relaxed);
  }

  // ── External forces / perturbation ────────────────────────────────────────

  void SetExternalForce(int body_id,
                        const std::array<double, 6>& wrench_world) noexcept;
  void ClearExternalForce() noexcept;
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
  [[nodiscard]] double   GetPhysicsTimestep() const noexcept { return xml_timestep_; }

 private:
  Config   cfg_;
  mjModel* model_{nullptr};
  mjData*  data_{nullptr};

  std::atomic<bool>     running_{false};
  std::atomic<uint64_t> step_count_{0};
  std::atomic<double>   sim_time_sec_{0.0};

  // ── Multi-group storage ─────────────────────────────────────────────────
  std::vector<std::unique_ptr<JointGroup>> groups_;

  // XML에서 발견된 전체 hinge+actuator 조인트 (Initialize()에서 설정)
  std::vector<std::string> all_xml_joint_names_;

  // ── Runtime control flags ─────────────────────────────────────────────────
  std::atomic<bool>   paused_{false};
  std::atomic<bool>   reset_requested_{false};
  std::atomic<bool>   step_once_{false};
  std::atomic<double> current_max_rtf_{0.0};
  std::atomic<bool>   gravity_enabled_{false};
  double              original_gravity_z_{-9.81};

  // ── Physics solver atomics ────────────────────────────────────────────────
  std::atomic<int>    solver_integrator_{mjINT_EULER};
  std::atomic<int>    solver_type_{mjSOL_NEWTON};
  std::atomic<int>    solver_iterations_{100};
  std::atomic<double> solver_tolerance_{1e-8};
  std::atomic<bool>   contacts_enabled_{true};

  // ── Solver statistics ─────────────────────────────────────────────────────
  mutable std::mutex solver_stats_mutex_;
  SolverStats        latest_solver_stats_{};

  // ── Sync step ─────────────────────────────────────────────────────────────
  std::mutex              sync_mutex_;
  std::condition_variable sync_cv_;

  // ── Viewer double-buffer ──────────────────────────────────────────────────
  mutable std::mutex  viz_mutex_;
  std::vector<double> viz_qpos_{};
  int                 viz_ncon_{0};
  bool                viz_dirty_{false};

  std::jthread sim_thread_;
  std::jthread viewer_thread_;

  // ── RTF measurement ───────────────────────────────────────────────────────
  std::chrono::steady_clock::time_point rtf_wall_start_{};
  double                                rtf_sim_start_{0.0};
  std::atomic<double>                   rtf_{0.0};

  // ── Max-RTF throttle (sim thread only) ───────────────────────────────────
  std::chrono::steady_clock::time_point throttle_wall_start_{};
  double                                throttle_sim_start_{0.0};
  double                                throttle_rtf_{0.0};

  // ── Gravity lock (position servo 모드에서 gravity 변경 차단) ─────────────
  std::atomic<bool> gravity_locked_by_servo_{true};

  // ── Original actuator params (전체 actuator, Initialize()에서 저장) ──────
  double xml_timestep_{0.002};
  struct ActuatorParams {
    double gainprm0{0.0};
    double biasprm0{0.0};
    double biasprm1{0.0};
    double biasprm2{0.0};
  };
  std::vector<ActuatorParams> orig_actuator_params_;

  // ── External forces / perturbation (under pert_mutex_) ───────────────────
  mutable std::mutex pert_mutex_;
  mjvPerturb         shared_pert_{};
  bool               pert_active_{false};
  std::vector<double> ext_xfrc_{};
  bool                ext_xfrc_dirty_{false};

  // ── Internal helpers ───────────────────────────────────────────────────────
  // XML에서 모든 hinge+actuator 조인트를 발견하여 all_xml_joint_names_에 저장
  bool DiscoverAllXmlJoints() noexcept;
  // robot 그룹의 command_joint_names를 XML과 양방향 검증 후 인덱스 매핑
  bool ValidateAndMapRobotGroups() noexcept;
  // state_joint_names 검증 (XML에 존재 여부) 및 인덱스 매핑
  bool ValidateAndMapStateJoints() noexcept;
  // 단일 robot 그룹의 command용 인덱스 매핑
  bool MapGroupIndices(JointGroup& group) noexcept;
  // 단일 그룹의 state용 인덱스 매핑
  bool MapStateIndices(JointGroup& group) noexcept;

  void ApplyCommand() noexcept;
  void ReadState() noexcept;
  void ReadSolverStats() noexcept;
  void InvokeStateCallback() noexcept;
  void UpdateVizBuffer() noexcept;
  void UpdateRtf(uint64_t step) noexcept;
  void ThrottleIfNeeded() noexcept;
  void HandleReset() noexcept;
  void PreparePhysicsStep() noexcept;
  void ClearContactForces() noexcept;

  void SimLoopFreeRun(std::stop_token stop) noexcept;
  void SimLoopSyncStep(std::stop_token stop) noexcept;
  void ViewerLoop(std::stop_token stop) noexcept;
};

}  // namespace rtc

#endif  // RTC_MUJOCO_SIM_MUJOCO_SIMULATOR_HPP_
