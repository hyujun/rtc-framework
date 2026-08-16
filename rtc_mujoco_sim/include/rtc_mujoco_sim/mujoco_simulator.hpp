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
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace rtc {

// ── JointControlMode ─────────────────────────────────────────────────────────
// Per-group actuator dispatch mode (robot groups only).
//   kPosition       — affine PD servo, body_gravcomp ON  (MuJoCo cancels gravity).
//   kTorque         — force pass-through, body_gravcomp OFF (controller does gravity).
//   kPdFeedforward  — PD servo (same actuator params as kPosition) + per-dof
//                     feedforward torque via qfrc_applied; body_gravcomp OFF
//                     (feedforward must include gravity). See README.
enum class JointControlMode { kPosition, kTorque, kPdFeedforward };

// ── SolverConfig ─────────────────────────────────────────────────────────────
// MuJoCo constraint solver parameters loaded from YAML (solver_param.yaml).
// XML <option> 속성이 명시적으로 설정된 경우 XML 값이 우선됩니다.

struct SolverConfig {
  // Algorithm selection
  std::string solver{"Newton"};     // "PGS", "CG", "Newton"
  std::string cone{"pyramidal"};    // "pyramidal", "elliptic"
  std::string jacobian{"auto"};     // "dense", "sparse", "auto"
  std::string integrator{"Euler"};  // "Euler", "RK4", "implicit", "implicitfast"

  // Iteration parameters
  int iterations{100};
  double tolerance{1e-8};
  int ls_iterations{50};
  double ls_tolerance{0.01};
  int noslip_iterations{0};
  double noslip_tolerance{1e-6};
  int ccd_iterations{50};
  double ccd_tolerance{1e-6};
  int sdf_iterations{10};
  int sdf_initpoints{40};

  // Physics
  double impratio{1.0};

  // Flags
  bool warmstart{true};
  bool refsafe{true};
  bool island{false};
  bool eulerdamp{true};
  bool filterparent{true};
  // Defaults below mirror MuJoCo's own defaults so behavior is unchanged
  // unless the user opts in:
  //   multiccd  — OFF (single contact point per convex pair)
  //   autoreset — ON  (silently mj_resetData on NaN; turn OFF to debug blow-up)
  //   nativeccd — ON  (3.3+ native CCD; OFF reverts to legacy MPR fallback)
  bool multiccd{false};
  bool autoreset{true};
  bool nativeccd{true};

  // Contact override
  struct ContactOverride {
    bool enable{false};
    double o_margin{0.0};
    std::array<double, 2> o_solref{0.02, 1.0};
    std::array<double, 5> o_solimp{0.9, 0.95, 0.001, 0.5, 2.0};
    std::array<double, 5> o_friction{1.0, 1.0, 0.005, 0.0001, 0.0001};
  };

  ContactOverride contact_override;
};

// ── JointGroupConfig ─────────────────────────────────────────────────────────
// Per-group configuration loaded from YAML (robot_response / fake_response).

struct JointGroupConfig {
  std::string name;                      // 임의 그룹 식별자 (YAML 에서 자유 지정)
  std::vector<std::string> joint_names;  // 하위호환: command/state 미지정 시 사용
  std::vector<std::string> command_joint_names;  // command용 joint names (빈 경우 joint_names 사용)
  std::vector<std::string> state_joint_names;  // state용 joint names (빈 경우 XML 전체)
  std::string command_topic;
  std::string state_topic;
  bool is_robot{true};           // true=robot_response, false=fake_response
  double filter_alpha{0.1};      // fake_response용 LPF 계수
  std::vector<double> servo_kp;  // 비어있으면 글로벌 값 상속
  std::vector<double> servo_kd;

  // ── Sensor publishing (optional) ──────────────────────────────
  std::string sensor_topic;               // 빈 문자열이면 센서 publish 안 함
  std::vector<std::string> sensor_names;  // XML sensor names (빈 경우 = 그룹에 센서 없음)

  // ── Contact wrench publishing (optional, mjSENS_CONTACT auto-discovery) ──
  // Scans this group's sensor_infos for mjSENS_CONTACT entries (dim==17,
  // data="found force torque dist pos normal tangent" reduce="netforce"), then
  // resolves each to a (target, ft_site, body, frame_id) tuple. World-frame
  // wrench is transformed into the ft_site's body frame at every sim tick.
  // Per-target topics: <topic_prefix>/<target>/contact_wrench (WrenchStamped).
  struct ContactWrenchConfig {
    bool enabled{false};
    std::string topic_prefix;  // 빈 문자열 = 비활성 (enabled=false 와 동등)
    // Suffix list ordered longest-first so "_tip_contact" matches before "_contact".
    std::vector<std::string> sensor_name_suffixes{"_tip_contact", "_contact"};
    std::vector<std::string> reference_site_suffixes{"_tip_ft_site", "_ft_site"};
    bool publish_state{false};  // <target>/contact_state (std_msgs/Bool)
    bool publish_debug{false};  // <target>/contact_point + contact_depth
    bool allow_partial_discovery{false};  // false=fail if any sensor unresolved
  };
  ContactWrenchConfig contact_wrench;
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

  bool is_robot{true};     // robot_response 여부
  bool is_primary{false};  // sync_step 대기 대상

  // ── MuJoCo 인덱스: command용 (is_robot==true, 이름 기반 비연속 가능)
  std::vector<int> qpos_indices;
  std::vector<int> qvel_indices;
  std::vector<int> actuator_indices;

  // ── MuJoCo 인덱스: per-group gravity compensation 대상 body
  // (joint 의 child body, position-servo 모드에서 body_gravcomp=1.0 으로 설정)
  std::vector<int> body_indices;

  // ── MuJoCo 인덱스: state용 ────────────────────────────────────────
  std::vector<int> state_qpos_indices;
  std::vector<int> state_qvel_indices;

  // ── Command 버퍼 (모두 cmd_mutex 보호) ───────────────────────────
  std::vector<double> pending_cmd;
  std::vector<double> initial_qpos;
  // pd_feedforward staging — single cmd_mutex region so the SimLoop sees a
  // consistent (mode, cmd, feedforward, gains) tuple per cmd_pending publish.
  std::vector<double> pending_feedforward;  // tau_ff (Nm), sized num_command_joints
  std::vector<double> staged_kp;            // runtime PD gain, empty = keep current
  std::vector<double> staged_kd;
  bool staged_has_gains{false};
  JointControlMode staged_mode{JointControlMode::kPosition};

  // ── State 버퍼 (state_joint_names 기준) ───────────────────────────
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
  std::atomic<bool> cmd_pending{false};
  mutable std::mutex cmd_mutex;
  mutable std::mutex state_mutex;

  // ── Per-group control mode ──────────────────────────────────────
  std::atomic<JointControlMode> control_mode{JointControlMode::kPosition};
  // Initial value true so the first PreparePhysicsStep applies the configured
  // mode (position-servo by default) to every actuator — necessary on
  // bare-actuator MJCFs where MuJoCo's parser leaves gainprm/biasprm at the
  // pass-through default (force = ctrl).
  std::atomic<bool> control_mode_pending{true};

  // ── Per-group servo gains (SimLoop-only after Initialize) ───────
  // gainprm_yaml[i]=kp, biasprm2_yaml[i]=-kd. Written at Initialize and by the
  // SimLoop thread (ApplyCommand) on runtime gain updates — never by the ROS
  // callback thread (avoids a data race with PreparePhysicsStep reads).
  std::vector<double> gainprm_yaml;
  std::vector<double> biasprm2_yaml;
  // Once a runtime SetGains/StageCommand override lands, the PD dispatch uses
  // gainprm_yaml regardless of cfg_.use_yaml_servo_gains. SimLoop-only.
  bool gains_overridden{false};

  // ── State callback ──────────────────────────────────────────────
  using StateCallback = std::function<void(const std::vector<double>& positions,
                                           const std::vector<double>& velocities,
                                           const std::vector<double>& efforts)>;
  StateCallback state_cb{nullptr};

  // ── Fake response (is_robot==false일 때) ────────────────────────
  double filter_alpha{0.1};
  std::vector<double> fake_state;
  std::vector<double> fake_target;
  mutable std::mutex fake_mutex;

  // ── Sensor info (populated during Initialize from XML) ─────────
  struct SensorInfo {
    std::string name;
    int type{0};  // mjtSensor enum
    int adr{0};   // index into data_->sensordata
    int dim{0};   // number of scalar outputs
  };

  std::vector<SensorInfo> sensor_infos;
  std::vector<double> sensor_buffer;  // flat readback buffer [sum(dims)]

  using SensorCallback =
      std::function<void(const std::vector<SensorInfo>& infos, const std::vector<double>& values)>;
  SensorCallback sensor_cb{nullptr};

  // ── Contact wrench info (populated by DiscoverContactWrenches) ──────────
  // Each entry pairs a mjSENS_CONTACT sensor (dim==17, reduce=netforce) with a
  // reference site and body. The body frame is used as ROS frame_id and as the
  // target of the world→link wrench transform.
  struct ContactWrenchInfo {
    std::string target_name;       // e.g. "index_tip" (sensor name minus suffix)
    std::string frame_id;          // body name owning ft_site (e.g. "index_tip_head")
    int sensor_id{-1};             // mjModel sensor id
    int sensor_adr{0};             // mjModel.sensor_adr[sensor_id]
    int ft_site_id{-1};            // mjModel site id (torque reference origin)
    int body_id{-1};               // mjModel.site_bodyid[ft_site_id]
  };

  // Transformed wrench sample (link frame). Heap-free POD.
  struct ContactWrenchSample {
    bool found{false};
    std::array<double, 3> force{0.0, 0.0, 0.0};   // object-on-fingertip, in link frame
    std::array<double, 3> torque{0.0, 0.0, 0.0};  // about ft_site origin, in link frame
    std::array<double, 3> point_world{0.0, 0.0, 0.0};  // contact point in world (debug)
    double dist{0.0};
  };

  std::vector<ContactWrenchInfo> contact_wrench_infos;
  std::vector<ContactWrenchSample> contact_wrench_buffer;  // size == infos.size()

  using ContactWrenchCallback = std::function<void(
      const std::vector<ContactWrenchInfo>& infos,
      const std::vector<ContactWrenchSample>& samples)>;
  ContactWrenchCallback contact_wrench_cb{nullptr};

  // ── ROS2 토픽 ──────────────────────────────────────────────────
  std::string command_topic;
  std::string state_topic;
  std::string sensor_topic;

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
// Simulation loop:
//   Publishes state, waits for one command, steps once, throttles by max_rtf.
//   Step latency ≈ controller Compute() time + DDS round-trip.
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
  using StateCallback = JointGroup::StateCallback;

  struct Config {
    std::string model_path;
    std::string window_title;  // viewer window title (empty = "MuJoCo Simulator")
    bool enable_viewer{true};
    double sync_timeout_ms{50.0};  // command wait timeout
    double max_rtf{0.0};           // 0.0 = unlimited
    double physics_timestep{0.0};
    int n_substeps{1};                 // substeps per control cycle (1 = legacy)
    double viewer_refresh_rate{60.0};  // viewer target refresh rate (Hz)

    // 글로벌 servo gain (그룹별 미지정 시 상속)
    bool use_yaml_servo_gains{false};
    std::vector<double> servo_kp{500.0, 500.0, 500.0, 150.0, 150.0, 150.0};
    std::vector<double> servo_kd{400.0, 400.0, 400.0, 100.0, 100.0, 100.0};

    // Solver 설정 (solver_param.yaml, XML 우선)
    SolverConfig solver_config;

    // 멀티 그룹 설정 (robot_response + fake_response)
    std::vector<JointGroupConfig> groups;
  };

  explicit MuJoCoSimulator(Config cfg) noexcept;
  ~MuJoCoSimulator();

  MuJoCoSimulator(const MuJoCoSimulator&) = delete;
  MuJoCoSimulator& operator=(const MuJoCoSimulator&) = delete;
  MuJoCoSimulator(MuJoCoSimulator&&) = delete;
  MuJoCoSimulator& operator=(MuJoCoSimulator&&) = delete;

  // ── Pure parse helpers (testable without MuJoCo init) ────────────────────
  // String → MuJoCo enum conversion. Unknown input returns the documented
  // default (mjSOL_NEWTON / mjCONE_PYRAMIDAL / mjJAC_AUTO / mjINT_EULER).
  [[nodiscard]] static int SolverNameToEnum(std::string_view name) noexcept;
  [[nodiscard]] static int ConeNameToEnum(std::string_view name) noexcept;
  [[nodiscard]] static int JacobianNameToEnum(std::string_view name) noexcept;
  [[nodiscard]] static int IntegratorNameToEnum(std::string_view name) noexcept;

  // In-place LPF step: state[i] += alpha * (target[i] - state[i]).
  // Processes min(state.size(), target.size()) elements; no-op if either empty.
  // NaN target values are skipped (state unchanged for that index).
  static void ApplyFakeLpfStep(std::vector<double>& state, const std::vector<double>& target,
                               double alpha) noexcept;

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

  // Register the sensor callback for a specific group.
  void SetSensorCallback(std::size_t group_idx, JointGroup::SensorCallback cb) noexcept;

  // Sensor accessors.
  [[nodiscard]] const std::vector<JointGroup::SensorInfo>& GetSensorInfos(
      std::size_t group_idx) const noexcept;
  [[nodiscard]] bool HasSensors(std::size_t group_idx) const noexcept;

  // Register the contact wrench callback for a specific group.
  void SetContactWrenchCallback(std::size_t group_idx,
                                JointGroup::ContactWrenchCallback callback) noexcept;

  // Contact wrench accessors (populated only when ContactWrenchConfig.enabled).
  [[nodiscard]] const std::vector<JointGroup::ContactWrenchInfo>& GetContactWrenchInfos(
      std::size_t group_idx) const noexcept;
  [[nodiscard]] bool HasContactWrenches(std::size_t group_idx) const noexcept;

  [[nodiscard]] std::vector<double> GetPositions(std::size_t group_idx) const noexcept;
  [[nodiscard]] std::vector<double> GetVelocities(std::size_t group_idx) const noexcept;
  [[nodiscard]] std::vector<double> GetEfforts(std::size_t group_idx) const noexcept;

  [[nodiscard]] const std::vector<std::string>& GetJointNames(std::size_t group_idx) const noexcept;
  [[nodiscard]] const std::vector<std::string>& GetStateJointNames(
      std::size_t group_idx) const noexcept;
  [[nodiscard]] int NumGroupJoints(std::size_t group_idx) const noexcept;
  [[nodiscard]] int NumStateJoints(std::size_t group_idx) const noexcept;
  [[nodiscard]] bool IsGroupRobot(std::size_t group_idx) const noexcept;

  [[nodiscard]] std::size_t NumGroups() const noexcept { return groups_.size(); }

  // Per-group control mode (robot groups only).
  void SetControlMode(std::size_t group_idx, JointControlMode mode) noexcept;
  [[nodiscard]] JointControlMode GetControlMode(std::size_t group_idx) const noexcept;

  // Bool compat shims (true=torque, false=position) — used by existing tests.
  void SetControlMode(std::size_t group_idx, bool torque_mode) noexcept {
    SetControlMode(group_idx,
                   torque_mode ? JointControlMode::kTorque : JointControlMode::kPosition);
  }
  [[nodiscard]] bool IsInTorqueMode(std::size_t group_idx) const noexcept {
    return GetControlMode(group_idx) == JointControlMode::kTorque;
  }

  // Single-shot staging for the ROS callback thread: bundles mode + command +
  // feedforward + (optional) PD gains into one cmd_mutex region so the SimLoop
  // never observes a half-updated tuple. feedforward/kp/kd may be empty.
  //   - feedforward: per-command-joint tau_ff (Nm); empty → all zeros.
  //   - kp/kd: runtime PD gains (>=0), sticky; both empty → keep current gains.
  void StageCommand(std::size_t group_idx, JointControlMode mode,
                    const std::vector<double>& cmd, const std::vector<double>& feedforward,
                    const std::vector<double>& kp, const std::vector<double>& kd) noexcept;

  // ── Test-only synchronous step + observers ────────────────────────────────
  // The SimLoop thread normally owns all mjData mutation; these run one full
  // tick (ApplyCommand → PreparePhysicsStep → mj_step → ReadState) inline so a
  // test can stage a command and observe the resulting joint forces. Do NOT
  // call while SimLoop is running.
  void StepForTest() noexcept;
  [[nodiscard]] double GetActuatorForceForTest(std::size_t group_idx,
                                               std::size_t joint_idx) const noexcept;
  [[nodiscard]] double GetAppliedForceForTest(std::size_t group_idx,
                                              std::size_t joint_idx) const noexcept;
  [[nodiscard]] double GetActuatorGainForTest(std::size_t group_idx,
                                              std::size_t joint_idx) const noexcept;

  // Per-group gravity-compensation status (set by SetControlMode).
  // Position servo → per-body gravcomp ON for the group's body chain.
  // Torque mode    → per-body gravcomp OFF (controller computes its own gravity comp).
  // World gravity (`opt.gravity`) is unaffected — free objects still fall.
  [[nodiscard]] bool IsGroupGravcompEnabled(std::size_t group_idx) const noexcept;

  // ── Backward-compatible API (delegates to group 0) ────────────────────────

  void SetCommand(const std::vector<double>& cmd) noexcept { SetCommand(0, cmd); }

  void SetStateCallback(StateCallback cb) noexcept { SetStateCallback(0, std::move(cb)); }

  [[nodiscard]] std::vector<double> GetPositions() const noexcept { return GetPositions(0); }

  [[nodiscard]] std::vector<double> GetVelocities() const noexcept { return GetVelocities(0); }

  [[nodiscard]] std::vector<double> GetEfforts() const noexcept { return GetEfforts(0); }

  [[nodiscard]] const std::vector<std::string>& GetJointNames() const noexcept {
    return GetJointNames(0);
  }

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
    int iter{0};
    int ncon{0};
  };

  [[nodiscard]] SolverStats GetSolverStats() const noexcept;

  // ── Physics solver controls (thread-safe) ─────────────────────────────────

  void SetIntegrator(int type) noexcept {
    solver_integrator_.store(type, std::memory_order_relaxed);
  }

  [[nodiscard]] int GetIntegrator() const noexcept {
    return solver_integrator_.load(std::memory_order_relaxed);
  }

  void SetSolverType(int type) noexcept { solver_type_.store(type, std::memory_order_relaxed); }

  [[nodiscard]] int GetSolverType() const noexcept {
    return solver_type_.load(std::memory_order_relaxed);
  }

  void SetSolverIterations(int iters) noexcept {
    solver_iterations_.store(std::max(1, std::min(iters, 1000)), std::memory_order_relaxed);
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

  void SetCone(int cone) noexcept { solver_cone_.store(cone, std::memory_order_relaxed); }

  [[nodiscard]] int GetCone() const noexcept {
    return solver_cone_.load(std::memory_order_relaxed);
  }

  void SetImpratio(double ratio) noexcept {
    solver_impratio_.store(ratio < 0.0 ? 1.0 : ratio, std::memory_order_relaxed);
  }

  [[nodiscard]] double GetImpratio() const noexcept {
    return solver_impratio_.load(std::memory_order_relaxed);
  }

  void SetNoslipIterations(int iters) noexcept {
    solver_noslip_iterations_.store(std::max(0, std::min(iters, 1000)), std::memory_order_relaxed);
  }

  [[nodiscard]] int GetNoslipIterations() const noexcept {
    return solver_noslip_iterations_.load(std::memory_order_relaxed);
  }

  // ── Physics controls (thread-safe) ────────────────────────────────────────

  void Pause() noexcept { paused_.store(true, std::memory_order_relaxed); }

  void Resume() noexcept {
    paused_.store(false, std::memory_order_relaxed);
    sync_cv_.notify_all();
  }

  [[nodiscard]] bool IsPaused() const noexcept { return paused_.load(std::memory_order_relaxed); }

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

  // World-gravity toggle (debugging only — affects every body in the scene).
  // Per-robot gravity compensation goes through SetControlMode, not this.
  void EnableWorldGravity(bool enable) noexcept {
    world_gravity_enabled_.store(enable, std::memory_order_relaxed);
  }

  [[nodiscard]] bool IsWorldGravityEnabled() const noexcept {
    return world_gravity_enabled_.load(std::memory_order_relaxed);
  }

  // ── External forces / perturbation ────────────────────────────────────────

  /// MJCF body name -> body id, for callers that must not hard-code ids
  /// (ARCH-1). Returns -1 for an unknown name AND for the world body, which is
  /// the same set SetExternalWrench refuses: a wrench on the world body is
  /// absorbed by the fixed base and would look identical to a dropped request.
  /// Safe to call from any thread — reads the immutable compiled model only.
  [[nodiscard]] int FindBodyId(const char* body_name) const noexcept;

  /// Latch a world-frame wrench acting at `point_body` (in `body_id`'s LOCAL
  /// frame, metres) until it is cleared, overwritten, or the sim is reset.
  /// Returns false — and stages nothing — if `body_id` names no movable body.
  ///
  /// THE POINT IS HONOURED, WHICH IS NOT WHAT MuJoCo DOES ON ITS OWN.
  /// mjData::xfrc_applied acts at the body CENTRE OF MASS (`xipos`), so a pure
  /// force staged verbatim would pick up a moment equal to the CoM offset
  /// crossed into it. That offset is not a corner case: it is non-zero on every
  /// body of iiwa7_leap, 35 mm on `ee_link`, and non-zero even on the massless
  /// bodies whose centre of mass is a compiler artefact rather than a physical
  /// quantity. At 10 N that is 0.25 N.m of torque nobody asked for — five times
  /// the payload estimator's measured noise floor, and indistinguishable from a
  /// real payload hanging off-centre. SimLoop therefore re-derives the
  /// CoM-referenced equivalent from `xpos`/`xipos`/`xmat` EVERY tick, which also
  /// keeps the contract true as the body rotates; a correction computed once at
  /// request time would go stale the moment the arm moved.
  ///
  /// Verified against pinocchio at the home pose: MuJoCo's `ee_link` body frame
  /// origin and the pinocchio `ee_link` frame origin agree to 1e-9 m, so a
  /// wrench requested here at point zero is the wrench a frame Jacobian of that
  /// name projects — which is what makes this usable as an estimator positive
  /// control (#135).
  [[nodiscard]] bool SetExternalWrenchAtPoint(int body_id,
                                              const std::array<double, 3>& point_body,
                                              const std::array<double, 6>& wrench_world) noexcept;

  /// SetExternalWrenchAtPoint at the body frame ORIGIN — the common case, and
  /// the one that lines up with a model frame of the same name.
  [[nodiscard]] bool SetExternalForce(int body_id,
                                      const std::array<double, 6>& wrench_world) noexcept;

  /// Drop every staged wrench in the scene. A single body's load is removed by
  /// staging a zero wrench on it instead.
  void ClearExternalForce() noexcept;
  void UpdatePerturb(const mjvPerturb& pert) noexcept;
  void ClearPerturb() noexcept;

  // ── Status accessors ──────────────────────────────────────────────────────

  [[nodiscard]] bool IsRunning() const noexcept { return running_.load(); }

  [[nodiscard]] uint64_t StepCount() const noexcept { return step_count_.load(); }

  [[nodiscard]] double SimTimeSec() const noexcept { return sim_time_sec_.load(); }

  // model_->nq became mjtSize (int64_t) in MuJoCo 3.7 — explicit narrowing.
  [[nodiscard]] int NumJoints() const noexcept {
    return model_ ? static_cast<int>(model_->nq) : 0;
  }

  // Read-only handles for tests / viewer. Caller must respect the SimLoop
  // ownership: do NOT mutate, and only read while the SimLoop is paused or
  // stopped (no internal locking is provided).
  [[nodiscard]] const mjModel* GetModel() const noexcept { return model_; }

  [[nodiscard]] const mjData* GetData() const noexcept { return data_; }

  [[nodiscard]] double GetRtf() const noexcept { return rtf_.load(std::memory_order_relaxed); }

  [[nodiscard]] double GetPhysicsTimestep() const noexcept { return xml_timestep_; }

  [[nodiscard]] int GetNumSubsteps() const noexcept { return cfg_.n_substeps; }

  [[nodiscard]] double GetPhysicsLoad() const noexcept {
    return physics_load_.load(std::memory_order_relaxed);
  }

 private:
  Config cfg_;
  mjModel* model_{nullptr};
  mjData* data_{nullptr};

  std::atomic<bool> running_{false};
  std::atomic<uint64_t> step_count_{0};
  std::atomic<double> sim_time_sec_{0.0};

  // ── Multi-group storage ─────────────────────────────────────────────────
  std::vector<std::unique_ptr<JointGroup>> groups_;

  // XML에서 발견된 전체 hinge+actuator 조인트 (Initialize()에서 설정)
  std::vector<std::string> all_xml_joint_names_;

  // ── Runtime control flags ─────────────────────────────────────────────────
  std::atomic<bool> paused_{false};
  std::atomic<bool> reset_requested_{false};
  std::atomic<bool> step_once_{false};
  std::atomic<double> current_max_rtf_{0.0};
  // World gravity is on by default — free bodies (objects to lift) always fall.
  std::atomic<bool> world_gravity_enabled_{true};
  double original_gravity_z_{-9.81};

  // ── Physics solver atomics (runtime-changeable via viewer/API) ─────────────
  std::atomic<int> solver_integrator_{mjINT_EULER};
  std::atomic<int> solver_type_{mjSOL_NEWTON};
  std::atomic<int> solver_iterations_{100};
  std::atomic<double> solver_tolerance_{1e-8};
  std::atomic<bool> contacts_enabled_{true};

  // ── Physics solver atomics (additional, from solver_param.yaml) ───────────
  std::atomic<int> solver_cone_{mjCONE_PYRAMIDAL};
  std::atomic<int> solver_jacobian_{mjJAC_AUTO};
  std::atomic<int> solver_ls_iterations_{50};
  std::atomic<double> solver_ls_tolerance_{0.01};
  std::atomic<int> solver_noslip_iterations_{0};
  std::atomic<double> solver_noslip_tolerance_{1e-6};
  std::atomic<double> solver_impratio_{1.0};

  // ── Solver statistics ─────────────────────────────────────────────────────
  mutable std::mutex solver_stats_mutex_;
  SolverStats latest_solver_stats_{};

  // ── Sync step ─────────────────────────────────────────────────────────────
  std::mutex sync_mutex_;
  std::condition_variable sync_cv_;

  // ── Viewer double-buffer ──────────────────────────────────────────────────
  mutable std::mutex viz_mutex_;
  std::vector<double> viz_qpos_{};
  int viz_ncon_{0};
  bool viz_dirty_{false};

  std::jthread sim_thread_;
  std::jthread viewer_thread_;

  // ── RTF measurement ───────────────────────────────────────────────────────
  std::chrono::steady_clock::time_point rtf_wall_start_{};
  double rtf_sim_start_{0.0};
  std::atomic<double> rtf_{0.0};
  std::atomic<double> physics_load_{0.0};
  uint64_t viz_update_interval_{8};
  int viewer_sleep_ms_{16};

  // ── Max-RTF throttle (sim thread only) ───────────────────────────────────
  std::chrono::steady_clock::time_point throttle_wall_start_{};
  double throttle_sim_start_{0.0};
  double throttle_rtf_{0.0};

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
  mjvPerturb shared_pert_{};
  bool pert_active_{false};
  // Wrench AS REQUESTED (about ext_point_, world axes), nbody*6. Converted to
  // the CoM-referenced xfrc_applied by PreparePhysicsStep, never stored
  // converted: the conversion depends on the body's current pose.
  std::vector<double> ext_xfrc_{};
  // Application point per body, in that body's LOCAL frame, nbody*3.
  std::vector<double> ext_point_{};
  bool ext_xfrc_dirty_{false};

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
  // XML에 정의된 모든 센서를 로그로 출력 (YAML 설정과 무관하게 항상 호출)
  void LogAllXmlSensors() const noexcept;
  // "auto" 키워드 시 XML 전체 센서 이름 목록 반환
  std::vector<std::string> CollectAllXmlSensorNames() const noexcept;
  // 그룹의 sensor_names를 XML 센서와 매핑
  void MapSensorInfos(JointGroup& group, const std::vector<std::string>& sensor_names) noexcept;
  // 그룹의 sensor_infos 에서 mjSENS_CONTACT 만 추려 contact_wrench_infos 생성.
  // sensor name suffix 와 site name suffix 로 target/site/body 를 자동 resolve.
  // allow_partial_discovery=false 면 unresolved 1건이라도 있으면 false 반환.
  [[nodiscard]] bool DiscoverContactWrenches(JointGroup& group) noexcept;

  // MJCF XML의 <option> 요소를 파싱하여 명시적으로 설정된 속성 이름 집합을 반환.
  // XML에 없는 속성에 대해서만 SolverConfig(YAML) 값을 적용하기 위한 헬퍼.
  void ApplySolverConfig() noexcept;

  // body_gravcomp[] 변경 후 호출. mj_passive() 의 ngravcomp==0 early-out 게이트를
  // 우회하기 위해 nonzero entry 수를 다시 카운트. 단독으로 mjModel 을 mutate 하는
  // SimLoop / Init 컨텍스트에서만 호출.
  void RefreshNgravcomp() noexcept;

  void ApplyCommand() noexcept;
  void ReadState() noexcept;
  void ReadSensors() noexcept;
  void ReadContactWrenches() noexcept;
  void ReadSolverStats() noexcept;
  void InvokeStateCallback() noexcept;
  void InvokeSensorCallback() noexcept;
  void InvokeContactWrenchCallback() noexcept;
  void UpdateVizBuffer() noexcept;
  void UpdateRtf(uint64_t step) noexcept;
  void ThrottleIfNeeded() noexcept;
  void HandleReset() noexcept;
  void PreparePhysicsStep() noexcept;
  // Requested wrenches -> data_->xfrc_applied, re-referenced to each body's
  // centre of mass. Caller holds pert_mutex_.
  void StageExternalWrenches() noexcept;
  void ClearContactForces() noexcept;

  void SimLoop(std::stop_token stop) noexcept;
  void ViewerLoop(std::stop_token stop) noexcept;
};

}  // namespace rtc

#endif  // RTC_MUJOCO_SIM_MUJOCO_SIMULATOR_HPP_
