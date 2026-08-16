#ifndef UR5E_BRINGUP_CONTROLLERS_DEMO_WBC_CONTROLLER_H_
#define UR5E_BRINGUP_CONTROLLERS_DEMO_WBC_CONTROLLER_H_

// Project headers (order: RTC base → interface → controllers → bridge → tsid)
#include "integrated_bringup/controllers/hand_sensor_layout.hpp"
#include "integrated_bringup/controllers/wbc/force_reference_updater.hpp"
#include "integrated_bringup/controllers/wbc/grasp_phase_manager.hpp"
#include "integrated_bringup/controllers/wbc/wbc_state.hpp"
#include "integrated_bringup/logging/device_sensor_log_pod.hpp"
#include "integrated_bringup/logging/device_state_log_pod.hpp"
#include "integrated_bringup/logging/device_wbc_log_pod.hpp"
#include "integrated_bringup/logging/pull_estimator_log_pod.hpp"
#include "integrated_bringup/logging/wbc_diag_log_pod.hpp"
#include "integrated_bringup/support/bringup_logging.hpp"
#include "integrated_bringup/support/closed_chain_hand_fk.hpp"
#include "integrated_bringup/support/combined_model_cache.hpp"
#include "integrated_bringup/support/momentum_observer_wiring.hpp"
#include "integrated_bringup/support/owned_topics.hpp"
#include "integrated_bringup/support/pull_estimator_wiring.hpp"
#include "integrated_bringup/support/wbc_reduced_dynamics_provider.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controller_interface/controller_log_set.hpp"
#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/trajectory/joint_space_trajectory.hpp"
#include "rtc_controllers/trajectory/task_space_trajectory.hpp"
#include "rtc_mpc/handler/mpc_factory.hpp"
#include "rtc_mpc/handler/mpc_handler_base.hpp"
#include "rtc_mpc/logging/mpc_timing_logger.hpp"
#include "rtc_mpc/manager/mpc_solution_manager.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/thread/handler_mpc_thread.hpp"
#include "rtc_mpc/thread/mock_mpc_thread.hpp"
#include "rtc_mpc/thread/mpc_thread.hpp"
#include "rtc_tsid/contact/contact_manager.hpp"
#include "rtc_tsid/contact/grasp_cache.hpp"
#include "rtc_tsid/contact/object_state_provider.hpp"
#include "rtc_tsid/controller/tsid_controller.hpp"
#include "rtc_tsid/kinematics/clik_reference.hpp"
#include "rtc_tsid/types/object_frame.hpp"
#include "rtc_tsid/types/qp_types.hpp"
#include "rtc_tsid/types/wbc_types.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

// Third-party
#include <rtc_msgs/srv/grasp_command.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/parameter.hpp>

#include <Eigen/Core>

// C++ stdlib
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace integrated_bringup {

using rtc::CommandType;
using rtc::ControllerOutput;
using rtc::ControllerState;
using rtc::GoalType;
using rtc::kMaxDeviceChannels;
using rtc::RTControllerInterface;
namespace trajectory = rtc::trajectory;

// ── WBC Phase (6-state reachable FSM, slots 2 & 5 reserved) ────────────────
//
// All non-fallback phases run TSID QP → position. Values 2 and 5 are reserved
// holes: value 5 was kRetreat in older WBC builds, and value 2 was kPreGrasp
// (a separate fine-positioning phase folded into kApproach — approach now
// drives straight to closure on the tight epsilon_pregrasp_ threshold). Both
// stay reserved to keep WbcState.msg PHASE_RETREAT=5 / PHASE_PRE_GRASP=2 stable
// for downstream consumers (demo_gui, BT, rosbag). Reintroducing either
// semantic should reuse the slot rather than shift values.
enum class WbcPhase : uint8_t {
  kIdle = 0,      ///< SE3 hold at current TCP via TSID
  kApproach = 1,  ///< TSID drives TCP toward grasp pose (quintic SE3 ramp),
                  ///< straight through to closure on epsilon_pregrasp_
  // 2 reserved (was kPreGrasp — merged into kApproach; PHASE_PRE_GRASP=2 deprecated)
  kClosure = 3,  ///< TSID with contact-forming tasks
  kHold = 4,     ///< TSID grasp holding
  // 5 reserved (was kRetreat — removed; WbcState.msg PHASE_RETREAT=5 deprecated)
  kRelease = 6,  ///< Contact ramp-down → finger-open trajectory
  kFallback = 7  ///< Safety: position hold at last valid q
};

// ── Stage C-3: source of the hand feedforward torque (τ_ff) overlaid on the
// pd_feedforward position backbone.
//   kGravityComp — pure hand gravity comp (g[hand]); conservative default.
//   kTsidTau     — the TSID-solved actuated torque for the hand joints
//                  (computed-torque FF: τ = τ_TSID + PD on the position hold).
// Either source is scaled by hand_tauff_gravity_gain and offset by
// hand_tauff_closure_bias, so the gain/bias/clamp/finite pipeline is identical.
enum class HandTauffSource : uint8_t {
  kGravityComp = 0,
  kTsidTau = 1,
};

// ── DemoWbcController ────────────────────────────────────────────────────────
//
// Whole-body controller demo for UR5e + 10-DoF hand using two WBC QPs that
// share one Common-stage reference (decision 5):
//   Kinematic WBC (CLIK-QP) — owns position: q_ref = q + v_ref·dt, the sole
//     position backbone (ComputeKinematicWbc).
//   Dynamic WBC (TSID-ID QP) — owns the hand τ_ff overlay; its a_opt is now
//     log-only (ComputeDynamicWbc).
//
// Runtime tunable parameters (per-controller LifecycleNode):
//   ROS 2 parameters declared on /demo_wbc_controller/<name>: see
//   DeclareGainParameters() in demo_wbc_controller.cpp. Read-only caps
//   arm_max_traj_velocity, hand_max_traj_velocity. mpc_enable is gated by
//   structural mpc_enabled_ (LoadConfig from YAML — toggling without an
//   MPC thread is a no-op). Force-PI grasp transitions: ~/grasp_command
//   srv (rtc_msgs/GraspCommand) — handler updates grasp_cmd_ atomic +
//   gains.grasp_target_force, which the FSM consumes.
class DemoWbcController final : public RTControllerInterface {
 public:
  // Fixed-size POD capacity (conservative for humanoid-class robots).
  // Actual DoF held in arm_dof_ / hand_dof_ / full_dof_ runtime members,
  // resolved in LoadConfig (arm) and OnDeviceConfigsSet (hand, full).
  static constexpr int kMaxArmDof = 32;
  static constexpr int kMaxHandDof = 32;
  static constexpr int kMaxFullDof = kMaxArmDof + kMaxHandDof;
  static constexpr int kNumPhases = 8;
  // ±2π fallback joint-position clamp [rad] when a device omits explicit limits.
  static constexpr double kJointLimitFallbackRad = 6.2832;

  struct Gains {
    double arm_trajectory_speed{0.5};    ///< Joint-space traj speed [rad/s]
    double hand_trajectory_speed{1.0};   ///< Hand traj speed [rad/s]
    double arm_max_traj_velocity{2.0};   ///< Max arm joint velocity [rad/s]
    double hand_max_traj_velocity{4.0};  ///< Max hand motor velocity [rad/s]
    double grasp_target_force{2.0};      ///< Target grasp force [N]
    // TCP task-space trajectory speeds (MPC-disabled SE3 ramp). Quintic
    // rest-to-rest; duration = max(d/speed, 1.875·d/max_vel, ...).
    double tcp_trajectory_speed{0.1};           ///< TCP translational speed [m/s]
    double tcp_trajectory_angular_speed{0.5};   ///< TCP angular speed [rad/s]
    double tcp_max_traj_velocity{0.5};          ///< TCP translational vel cap [m/s]
    double tcp_max_traj_angular_velocity{1.0};  ///< TCP angular vel cap [rad/s]
    double pi_rotation_margin{0.15};            ///< split when ang_dist > π - margin
    double se3_weight{100.0};                   ///< SE3Task weight (runtime tuning)
    double force_weight{10.0};                  ///< ForceTask weight
    double posture_weight{1.0};                 ///< PostureTask weight
    /// Grasp detection thresholds (mirror joint/task controllers).
    /// grasp_contact_threshold gates the native contact_flag probability and
    /// is consulted only when has_native_contact_=true (sensor A path); on
    /// sensor B paths the |F| threshold alone decides in_contact.
    float grasp_contact_threshold{0.5f};  ///< Native contact prob threshold (0..1)
    float grasp_force_threshold{1.0f};    ///< |F| threshold [N]
    int grasp_min_fingertips{2};          ///< grasp_detected = active_count ≥ N
    // ── Stage C-2: CLIK (Kinematic WBC) runtime gains ────────────────────
    // Forwarded to clik_ each tick (kx broadcast as [pos×3, rot×3]). CLIK
    // structural config (damping_sq / v_limit) is Init-time, not here. CLIK is
    // the sole position backbone (the integrator A/B path was removed).
    double clik_kx_pos{5.0};  ///< CLIK TCP position task gain
    double clik_kx_rot{5.0};  ///< CLIK TCP rotation task gain
    double clik_ka{1.0};      ///< CLIK arm nullspace posture gain
    double clik_kh{1.0};      ///< CLIK hand posture gain
    // ── Stage C-3: hand feedforward torque (pd_feedforward command) ───────
    // Opt-in. When enabled, the hand device is driven with command_type
    // kPdFeedforward: values = hold/closure pose (PD backbone), feedforward =
    // τ_ff. τ_ff = gravity_gain · g[hand] + closure_bias (uniform Nm), active
    // only in kClosure/kHold, clamped to ±tauff_max, zeroed on non-finite /
    // limit / E-STOP. Default (gravity_comp, gain 1, bias 0) = pure hand
    // gravity comp. hand_tauff_source selects gravity_comp vs the full TSID
    // actuated torque (computed-torque FF + PD); the latter is the eventual
    // intent, left opt-in pending a gain-overlap (TSID task gain vs hand PD)
    // study — flip live via the hand_tauff_source ROS param.
    bool hand_tauff_enable{false};  ///< Drive hand via pd_feedforward
    HandTauffSource hand_tauff_source{HandTauffSource::kGravityComp};  ///< τ_ff source
    double hand_tauff_gravity_gain{1.0};  ///< Scale on the τ_ff source [-]
    double hand_tauff_closure_bias{0.0};  ///< Uniform additive τ bias [Nm]
    double hand_tauff_max{5.0};           ///< Per-joint |τ_ff| clamp [Nm]
  };

  explicit DemoWbcController(std::string_view urdf_path);

  // Destructor stops the MPC solve thread *before* member auto-destruction
  // begins. Default destructor would tear down `mpc_model_handler_` /
  // `phase_manager_owned_` / `mpc_manager_` first (member declaration order
  // → reverse-order destruction), leaving the still-running `mpc_main`
  // thread reading freed Pinocchio model memory inside `Solve()`. Observed
  // shutdown SEGVs (sim Ctrl+C) all faulted in `pinocchio::CATForwardStep`
  // visitors with use-after-free addresses. Joining first eliminates the
  // race; subsequent member destruction is then safe.
  ~DemoWbcController() override;

  // ── RTControllerInterface overrides ──────────────────────────────────────
  [[nodiscard]] ControllerOutput Compute(const ControllerState& state) noexcept override;

  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override;

  // Commanded SE3 (arm task goal). Marshals a task-tagged entry onto the base
  // mailbox; the RT thread converts (x,y,z,r,p,y)→SE3 into the TargetSlot's
  // commanded slot in ApplyPendingTarget. Joint targets keep using
  // SetDeviceTarget — this controller is the reason the mailbox carries the
  // is_task tag at all, since the two goals must not clobber each other.
  void SetDeviceTaskTarget(int device_idx, std::span<const double> task6) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override { return "DemoWbcController"; }

  void TriggerEstop() noexcept override;
  void ClearEstop() noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void SetHandEstop(bool active) noexcept override;

  // ── Phase 4: controller-owned sub/pub lifecycle ─────────────────────────
  CallbackReturn on_configure(const rclcpp_lifecycle::State& prev,
                              rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                              const YAML::Node& yaml) noexcept override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) noexcept override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) noexcept override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) noexcept override;
  void PublishNonRtSnapshot(const rtc::PublishSnapshot& snap) noexcept override;

  // ── Test accessors (const snapshots, not RT-safe) ───────────────────────
  struct FingertipReport {
    float force_magnitude{0.0f};
    float force_rate{0.0f};
    float contact_flag{0.0f};
    bool valid{false};
  };

  [[nodiscard]] FingertipReport GetFingertipReportForTesting(int fingertip_idx) const noexcept;

  [[nodiscard]] int GetNumActiveFingertipsForTesting() const noexcept {
    return num_active_fingertips_;
  }

  [[nodiscard]] WbcPhase GetPhaseForTesting() const noexcept { return phase_; }

  // Commanded-SE3 path inspection (wbc-kinematic-dynamic-split). The RT working
  // SE3 goal, its validity, the commanded-slot validity, and a joint-slot value
  // (current_target_slot_), so tests can assert task→SE3 routing and joint/task
  // slot independence without a real URDF/TSID stack.
  [[nodiscard]] const pinocchio::SE3& GetTcpGoalForTesting() const noexcept { return tcp_goal_; }

  [[nodiscard]] bool IsTcpGoalValidForTesting() const noexcept { return tcp_goal_valid_; }

  /// Test-only: bind the momentum_observer CSV channel from a test-owned
  /// ControllerLogSet. The URDF fixtures stop at SetDeviceNameConfigs, so every
  /// production-bound log handle is unbound and UpdateMomentumObserverChannels
  /// skips the CSV lane; a row assertion written against that state would pass
  /// with the push deleted outright. (The topic lane is guarded separately and
  /// is unaffected by this handle.)
  void SetMomentumObserverLogHandleForTesting(
      rtc::LogHandle<integrated_bringup::MomentumObserverLogPod> h) {
    momentum_observer_log_handle_ = std::move(h);
  }

  /// Test-only: the configure-time error BuildMomentumObserverWiring raised, or
  /// empty when the wiring resolved (enabled, or deliberately disabled).
  /// on_configure consumes the same string — see OnDeviceConfigsSet.
  [[nodiscard]] const std::string& MomentumObserverConfigErrorForTesting() const {
    return momentum_config_error_;
  }

  [[nodiscard]] bool IsTcpCmdValidForTesting() const noexcept {
    return current_target_slot_.tcp_cmd_valid;
  }

  [[nodiscard]] double GetJointTargetForTesting(int device_idx, int channel) const noexcept {
    return current_target_slot_
        .targets[static_cast<std::size_t>(device_idx)][static_cast<std::size_t>(channel)];
  }

  // Phase 4c: ControllerOutput::wbc_state field was removed — tests now read
  // the post-Compute() staging buffer directly. The wbc_state_lock_ SeqLock
  // is what the publish thread sees; this accessor exposes the same data
  // without going through SeqLock's retry loop.
  [[nodiscard]] const ::integrated_bringup::WbcStateData& GetWbcStateForTesting() const noexcept {
    return wbc_state_;
  }

  /// Test-only: the body the publish thread would Load right now (#234 P-1) —
  /// i.e. through the SeqLock rather than the staging buffer above, so a tick
  /// that fills wbc_state_ without storing it is detectable.
  [[nodiscard]] ::integrated_bringup::WbcStateData GetPublishedWbcStateForTesting() const noexcept {
    return wbc_state_lock_.Load();
  }

  void ForcePhaseForTesting(WbcPhase p) noexcept {
    phase_ = p;
    // Force-injection tests rely on the first post-ForcePhase tick treating
    // any change in fingertip force as a fresh measurement (no spurious
    // df/dt spike). Reset the initialization flag so the next ReadState
    // tick skips the EMA delta computation.
    force_rate_initialized_ = false;
  }

  void SetGraspCmdForTesting(int v) noexcept { grasp_cmd_.store(v, std::memory_order_release); }

  // #1: inject a stale hand τ_ff active flag so a test can assert Compute()
  // clears it before WriteJointCommand on a non-TSID (e.g. forced kFallback) tick.
  void SetHandTauffActiveForTesting(bool v) noexcept { hand_tauff_active_ = v; }

  [[nodiscard]] bool GetHandTauffActiveForTesting() const noexcept { return hand_tauff_active_; }

  // #4: expose the WBC diag pod fill so a test can assert the logged command_type
  // reflects the PER-DEVICE DeviceOutput::command_type (e.g. hand kPdFeedforward
  // → 2), not the global ControllerOutput default.
  [[nodiscard]] ::integrated_bringup::DeviceWbcLogPod FillDeviceWbcLogPodForTesting(
      const ControllerState& state, const ControllerOutput& output, std::size_t device_idx,
      std::uint8_t role) const noexcept {
    ::integrated_bringup::DeviceWbcLogPod pod{};
    FillDeviceWbcLogPod(state, output, device_idx, role, pod);
    return pod;
  }

  // #175: "closed-chain 사영이 tick 당 1회" 는 밖에서 관측할 물리량이 없다 — 두 소비자가 같은
  // 사영을 읽으면 값이 같아서 pose 만 보고는 1회인지 2회인지 알 수 없다. 그래서 wrapper 축
  // 카운터를 노출한다 (rtc_urdf_bridge 에는 virtual seam 도 rtc_base 의존도 없어 그 패키지에
  // 계측을 넣을 수 없다). 배선 축: fingertip FK 가 provider 사영을 빌렸는가.
  [[nodiscard]] bool HandFkSharesProjectionForTesting() const noexcept {
    return closed_hand_fk_.active() && !closed_hand_fk_.owns_projection();
  }

  /// fingertip FK 가 **직접** 돌린 사영 횟수 (공유 중이면 영원히 0).
  [[nodiscard]] std::uint32_t HandFkProjectionCountForTesting() const noexcept {
    return closed_hand_fk_.projection_count();
  }

  /// 축약 동역학 provider 가 돌린 사영 횟수 (tick 당 1 증가).
  [[nodiscard]] std::uint32_t ReducedProjectionSeqForTesting() const noexcept {
    return wbc_reduced_dynamics_.projection_seq();
  }

  [[nodiscard]] int GetReleaseStageForTesting() const noexcept { return release_stage_; }

  [[nodiscard]] double GetReleaseElapsedSecForTesting() const noexcept {
    return release_elapsed_s_;
  }

  // Pollute the release sub-FSM state so a subsequent kRelease entry can
  // verify OnPhaseEnter's reset overrode the dirty value.
  // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
  void ForceReleaseStateForTesting(int stage, double elapsed) noexcept {
    release_stage_ = stage;
    release_elapsed_s_ = elapsed;
  }

  // F-2 test access: seed captured base_frame entries directly so
  // OnDeviceConfigsSet can be exercised without a real Pinocchio model.
  void SetBaseFrameEntriesForTesting(
      std::vector<std::pair<std::string, std::string>> entries) noexcept {
    base_frame_yaml_entries_ = std::move(entries);
    base_frame_mismatch_ = false;
    base_frame_mismatch_detail_.clear();
  }

  [[nodiscard]] bool IsBaseFrameMismatchForTesting() const noexcept { return base_frame_mismatch_; }

  [[nodiscard]] const std::string& GetBaseFrameMismatchDetailForTesting() const noexcept {
    return base_frame_mismatch_detail_;
  }

  void set_gains(Gains gains) noexcept { gains_lock_.Store(gains); }

  [[nodiscard]] Gains get_gains() const noexcept { return gains_lock_.Load(); }

  // ── Registry hooks ──────────────────────────────────────────────────────
  void LoadConfig(const YAML::Node& cfg) override;

  /// Layout profile ids. Mirror of repo_scripts/config/thread_layout.yaml's
  /// `profiles:` block — the launch resolves the id (rtc_tools.launch.cpu_shield
  /// ::mpc_layout_profile) and hands the same string to both the cset shield and
  /// this controller, so the two cannot disagree about what is reserved.
  static constexpr std::string_view kDefaultLayoutProfile = "mpc_on";
  static constexpr std::string_view kMpcOffLayoutProfile = "mpc_off";

  /// Record this run's layout profile (issue #350).
  ///
  /// Called from on_configure with the `rt_layout_profile` node parameter. The
  /// launch owns the decision because MPC activation is not observable from the
  /// host: `mpc.enabled` is controller YAML and the thread is spawned in
  /// on_activate, so a controller switch can turn it on mid-session.
  void SetLayoutProfile(std::string_view profile) noexcept;

  /// True when `profile` returns the MPC cores to the system cpuset.
  ///
  /// Anything other than a recognised opt-out — including an empty or
  /// misspelled id — reads as the default profile, with the cores reserved.
  /// That direction is deliberate: reserving cores nobody uses wastes them,
  /// while treating an unrecognised id as an opt-out would refuse activation
  /// on a box whose shield still holds those cores.
  [[nodiscard]] static bool LayoutProfileDropsMpc(std::string_view profile) noexcept;
  void OnDeviceConfigsSet() override;

  [[nodiscard]] CommandType GetCommandType() const noexcept override { return command_type_; }

  // Pure assembly of per-DoF posture gains from an arm/hand split. External
  // joint i (< arm_dof → arm gains, else hand gains) is written into
  // kp_out/kd_out at its Pinocchio velocity index ext_to_pin_v[i]. The
  // generic PostureTask stores gains in Pinocchio order, where arm/hand are
  // not contiguous (ext_to_pin_v is a permutation) — so this mapping lives in
  // the controller, keeping rtc_tsid robot-agnostic (ARCH-1). Static + no
  // member access so the permutation mapping is unit-testable without a URDF.
  // kp_out/kd_out must be pre-sized to nv and pre-filled with a default; slots
  // outside [0, full_dof) are left untouched.
  static void AssemblePostureGains(int arm_dof, int full_dof, int nv,
                                   const std::array<int, kMaxFullDof>& ext_to_pin_v, double kp_arm,
                                   double kd_arm, double kp_hand, double kd_hand,
                                   Eigen::VectorXd& kp_out, Eigen::VectorXd& kd_out) noexcept;

  // Stage C-2: build the CLIK arm/hand velocity-index sets (Pinocchio order)
  // from the external→Pinocchio reorder map. External joint i maps to
  // ext_to_pin_v[i]; arm = external [0, arm_dof), hand = [arm_dof, full_dof).
  // The result feeds ClikReferenceGenerator::Config (robot-agnostic injection
  // point, ARCH-1). Indices ≥ nv (or < 0) are skipped. Static + no member
  // access so the permutation slice is unit-testable without a URDF.
  static void BuildClikJointIndexSets(int arm_dof, int full_dof, int nv,
                                      const std::array<int, kMaxFullDof>& ext_to_pin_v,
                                      std::vector<int>& arm_v_idx,
                                      std::vector<int>& hand_v_idx) noexcept;

 private:
  // ── Model initialization ────────────────────────────────────────────────
  void InitModels(const rtc_urdf_bridge::ModelConfig& config);
  // Build the serial hand tree-model handle + resolve fingertip / hand-root
  // frame ids from the secondary device's tree_model. Called from InitModels
  // (frame ids only; SetJointOrder + closed-chain wiring need device configs,
  // so they run in OnDeviceConfigsSet). No-op when there is no secondary device
  // or no matching tree_model. (#123 Phase 2)
  void InitHandModel(const rtc_urdf_bridge::ModelConfig& config);

  // ── Hand fingertip FK dispatch (#123 Phase 2 — mirrors task/joint) ────────
  // ConfigureClosedChainHandFk: non-RT wiring of closed_hand_fk_ + serial
  //   hand_handle_ joint order; called from OnDeviceConfigsSet. No-op (serial
  //   path) when the model has no loop closure / no downstream fingertip.
  // ComputeHandFingertipFk: per-tick RT dispatch — runs the closed or serial
  //   hand FK, composes each fingertip to the base frame via the arm TCP FK,
  //   caches into fingertip_positions_/rotations_. RT-safe; call after the arm
  //   FK (tcp is the base→tool0 placement). Returns false if no hand FK ran.
  void ConfigureClosedChainHandFk();
  bool ComputeHandFingertipFk(const ControllerState& state, const pinocchio::SE3& tcp) noexcept;

  // #120: closed-chain 축약 동역학 provider 배선 (non-RT; LoadConfig 의
  // combined_cache_.cache().Init
  //   직후 호출). control model 이 actuated(closed-chain) 모델일 때만 provider 를 Configure 해
  //   combined_cache_.cache().reduced_provider 로 주입 → TSID EOM 의 M/h/g 를 축약값으로 대체.
  //   비-extended (GetActuatedModel()==null) 이거나 정렬 미매칭이면 미주입 → open-chain 경로
  //   byte-for-byte.
  void ConfigureReducedDynamicsProvider();

  // Stage C-2: initialise the CLIK reference generator (registers the
  // se3_tcp tip/base frames on combined_cache_.cache() — by frame_id, so it reuses
  // the SE3Task registration — and Init's clik_ with the arm/hand v-index
  // sets). Leaves clik_enabled_ false unless TSID is built, the reorder map is
  // valid, and the control model is nq == nv (CLIK contract). Called from
  // on_configure after LoadConfig + OnDeviceConfigsSet, before the first RT
  // cache.Update() locks frame registration. CLIK is the sole position backbone,
  // so on_configure FAILS the lifecycle transition when this leaves CLIK
  // disabled on a TSID-initialised controller (DEC-1 ⓐ — no integrator remains).
  void InitClik() noexcept;

  // ── TSID task/constraint YAML factory ───────────────────────────────────
  //
  // Dispatches on the `type:` field of each entry under `tsid.tasks` /
  // `tsid.constraints`. Supported tasks: posture, se3, force. Supported
  // constraints: eom, joint_limit, friction_cone. Unknown types log ERROR
  // and skip. Called once in LoadConfig after TSIDController::Init().
  void BuildTsidTasks(const YAML::Node& tsid_node);
  void BuildTsidConstraints(const YAML::Node& tsid_node);

  // Parse the optional arm/hand posture-gain split from the posture task
  // config (`tsid.tasks.posture.{arm,hand}.{kp,kd}`). Sets posture_split_gains_
  // only when both `arm` and `hand` sub-maps are present; otherwise leaves it
  // false so PostureTask::Init's scalar/vector `kp`/`kd` path stands.
  void ParsePostureSplitGains(const YAML::Node& posture_cfg);
  // Assemble per-DoF posture kp/kd from the arm/hand split and push to the
  // PostureTask via SetGains. No-op unless posture_split_gains_ &&
  // tsid_initialized_ && combined_cache_.reorder_valid(). Called at the end of
  // on_configure, after the reorder map (OnDeviceConfigsSet) and the final
  // task build (LoadConfig) are both in place.
  void ApplyPostureGains() noexcept;

  // ── 3-phase pipeline (RT path) ──────────────────────────────────────────
  void ReadState(const ControllerState& state) noexcept;
  void ComputeControl(const ControllerState& state, double dt) noexcept;
  // WriteOutput was split into 3 explicit-intent methods (see
  // demo_joint_controller.hpp for the bucket contract). Compute() calls
  // them in order WriteJointCommand → FillLogOutput → FillPublishOutput.
  [[nodiscard]] ControllerOutput WriteJointCommand(const ControllerState& state) noexcept;
  void FillLogOutput(const ControllerState& state, ControllerOutput& output) noexcept;
  void FillPublishOutput(const ControllerState& state, ControllerOutput& output) noexcept;
  // Shared by FillLogOutput / FillPublishOutput. Fills output.actual_task_positions
  // + task_goal_positions from the current arm FK (caller must ensure FK is fresh;
  // arm_handle_ must be non-null) and returns the computed TCP SE3 so the publish
  // path can reuse it for arm_tip_pose.
  pinocchio::SE3 FillTaskPosePods(ControllerOutput& output) noexcept;

  // Sensor-derived WbcState aggregates (per-fingertip |F| / contact flags /
  // grasp detection). Sourced from fingertip_data_, which ReadState refreshes
  // every tick including E-STOP — hence shared by FillLogOutput and
  // FillEstopPublishState. Does NOT touch the TSID-derived fields.
  void FillWbcSensorAggregates() noexcept;

  // E-STOP counterpart of FillLogOutput's SeqLock store (#234 P-1). The E-8
  // rule this path used to enforce ("do not push the WBC state on the E-STOP
  // path — tsid_output_ is stale there") stopped the store but not the
  // publish: the CM stamps a snapshot every tick and the publish thread
  // re-loads the SeqLock, so skipping the store shipped the pre-E-STOP body
  // under the current stamp. Storing an explicitly E-STOP-shaped body keeps
  // the rule's intent — sensor aggregates are refreshed, TSID health is
  // reported as not-solved (tsid_solver_ok=false, tsid_solve_us=0) instead of
  // replaying the last solve, and the pull estimate runs its E-STOP tick.
  // RT tick path — noexcept, heap-free.
  void FillEstopPublishState(double dt) noexcept;

  // ── WBC CSV fill (controller-private data: a_opt / SE3 ramp / fingertip
  //    force / TSID-QP diagnostics — see ~/.claude/plans/wbc-csv-logging.md) ─
  // role 0 = arm (SE3 task block), role 1 = hand (motor + fingertip force).
  void FillDeviceWbcLogPod(const ControllerState& state, const ControllerOutput& output,
                           std::size_t device_idx, std::uint8_t role,
                           ::integrated_bringup::DeviceWbcLogPod& pod) const noexcept;
  void FillWbcDiagLogPod(const ControllerState& state,
                         ::integrated_bringup::WbcDiagLogPod& pod) const noexcept;

  // ── FSM ─────────────────────────────────────────────────────────────────
  WbcPhase phase_{WbcPhase::kIdle};
  WbcPhase prev_phase_{WbcPhase::kIdle};

  void UpdatePhase(const ControllerState& state) noexcept;
  void OnPhaseEnter(WbcPhase new_phase, const ControllerState& state) noexcept;

  // ── Control modes ───────────────────────────────────────────────────────
  void ComputePositionMode(double dt) noexcept;
  // ComputeTSIDPosition orchestrates the position-mode tick across the common
  // stage and the two WBC QPs, sharing one gains snapshot:
  //   ComputeWbcCommon    — shared stage (decision 5): state extract, pinocchio
  //                         + contact/grasp cache, MPC ref, and the per-tick
  //                         joint/SE3 references both QPs consume. No solve.
  //   ComputeKinematicWbc — Kinematic WBC: arm/hand position via the CLIK-QP
  //                         backbone. CLIK is the SOLE position backbone, so a
  //                         CLIK failure here is CRITICAL (kin_qp_fail_count_ →
  //                         hold-last → kFallback).
  //   ComputeDynamicWbc   — Dynamic WBC (TSID-ID QP): solves tsid_output_.a_opt/
  //                         tau (a_opt now log-only), then overlays hand τ_ff
  //                         (kPdFeedforward). A QP failure is NON-critical
  //                         (decision 6): it drops the hand τ_ff this tick
  //                         (dyn_qp_fail_count_); position is unaffected.
  // Order is Common → Kinematic → Dynamic: the two QPs consume the same Common
  // references independently (decision 5), so neither depends on the other.
  void ComputeTSIDPosition(const ControllerState& state, double dt) noexcept;
  void ComputeWbcCommon(const ControllerState& state, double dt, const Gains& gains_now) noexcept;
  void ComputeKinematicWbc(double dt, const Gains& gains_now) noexcept;
  void ComputeDynamicWbc(const Gains& gains_now) noexcept;
  void ComputeReleaseMode(const ControllerState& state, double dt) noexcept;
  void ComputeFallback() noexcept;

  // ── E-STOP ──────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};
  bool estop_active_{false};

  /// F5 device-readability gate for the arm (device 0), evaluated once at the
  /// very top of Compute() — before ReadState, so the model scatter, the FSM
  /// dispatch, the wire command and ComputeEstop all see one answer for this
  /// tick. False means device 0 did not report the arm_dof_ channels this
  /// controller reads, so TSID, the posture reference and the emitted command
  /// would run at a partially ZERO configuration. The answer is silence on
  /// device 0 (zero-length), NOT nc0 zeros — see
  /// rtc_controller_interface/device_readability.hpp and §3.7 of
  /// rtc_controllers/docs/compliance-conventions.md. RT-thread-only.
  bool arm_readable_{false};

  /// The same F5 gate on the SECONDARY axis (issue #291). arm_readable_ is
  /// device 0's policy and deliberately does not silence the hand (§3.7,
  /// "secondary passthrough 유지"); this flag is the hand's answer about
  /// ITSELF. False means device 1 did not report the hand_dof_ channels the
  /// hand loops read — hand_dof_ comes from YAML joint_state_names.size()
  /// while num_channels is the wire length, so a start-up mismatch would read
  /// the unreported finger joints as a finite 0 and ramp them origin-ward.
  /// The answer is the same as the arm's: silence device 1 (zero-length) with
  /// the reference lanes held at the measurement. RT-thread-only.
  bool hand_readable_{false};

  /// Arm joint position the E-STOP path drives to. Authoritative source is
  /// LoadConfig(cfg["estop"]["arm_safe_position"]); only the first
  /// `arm_dof_` slots are read by ComputeEstop. Zero-initialized by default
  /// so unit tests that skip LoadConfig still see a deterministic value.
  std::array<double, kMaxArmDof> safe_position_{};

  [[nodiscard]] ControllerOutput ComputeEstop(const ControllerState& state) noexcept;

  // ── Configuration ───────────────────────────────────────────────────────
  rtc::SeqLock<Gains> gains_lock_;
  std::string urdf_path_;
  CommandType command_type_{CommandType::kPosition};

  // Grasp command from ~/grasp_command srv (0=idle/abort, 1=approach,
  // 2=release)
  std::atomic<int> grasp_cmd_{0};

  // F-2: SE3/MPC base_frame YAML values captured during LoadConfig and
  // validated against the primary device's `urdf.root_link` in
  // OnDeviceConfigsSet. on_configure consults `base_frame_mismatch_` to
  // FAIL the lifecycle transition when frame names disagree, since
  // OnDeviceConfigsSet itself runs from a non-throwing CM call site.
  // Each entry is (label_for_logs, base_frame_yaml_value); empty labels
  // skip the comparison (universe fallback policy stays in effect).
  std::vector<std::pair<std::string, std::string>> base_frame_yaml_entries_;
  bool base_frame_mismatch_{false};
  std::string base_frame_mismatch_detail_;

  // ── rtc_urdf_bridge ─────────────────────────────────────────────────────
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> arm_handle_;
  // The same arm sub-model, kept as a shared_ptr so the momentum observer can
  // build its OWN handle over it (RtModelHandle::GetModel() hands back a
  // reference, not ownership).
  std::shared_ptr<const pinocchio::Model> arm_model_;
  pinocchio::FrameIndex tip_frame_id_{0};
  pinocchio::FrameIndex root_frame_id_{0};
  bool use_root_frame_{false};

  // ── Hand fingertip FK (#123 Phase 2 — publish/observation surface only) ──
  // Loop-consistent fingertip poses for kRobotTransforms. Mirrors the
  // task/joint pattern: the serial hand tree-model (secondary device, e.g.
  // "p1b") drives non-extended hands byte-for-byte; closed_hand_fk_ takes over
  // for extended-URDF (loop-closure) hands whose fingertips are downstream of a
  // loop-passive joint (proto_1b thumb/index/middle DIP). Fingertip poses feed
  // ControllerOutput::task_link_poses (NOT the TSID contact dynamics — that
  // stays on the actuated control model, Phase 3). hand_root frame ("base_adapter"
  // ≡ tool0) makes HandFingertipPose hand-root-relative; ComputeHandFingertipFk
  // composes with the arm TCP FK to base frame.
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> hand_handle_;
  static constexpr std::size_t kNumFingertips = ClosedChainHandFk::kMaxFingertips;
  std::array<pinocchio::FrameIndex, kNumFingertips> fingertip_frame_ids_{};
  pinocchio::FrameIndex hand_root_frame_id_{0};
  bool use_hand_root_frame_{false};
  std::array<Eigen::Vector3d, kNumFingertips> fingertip_positions_{};
  std::array<Eigen::Matrix3d, kNumFingertips> fingertip_rotations_{};
  // Per-tick: did ComputeHandFingertipFk produce a real pose for fingertip f?
  // False for a downstream tip until the closed-chain loop first converges, so
  // FillPublishOutput must not publish the zero-init cache as a valid TF.
  std::array<bool, kNumFingertips> fingertip_pose_valid_{};
  Eigen::VectorXd hand_q_;  // pre-allocated for serial hand FK
  ClosedChainHandFk closed_hand_fk_;
  /// #175: 직전 tick 에 본 provider 사영 카운터. borrowed 모드에서 "이번 tick 에 사영이 돌았는가"
  /// 를 이 값과의 차이로 판정한다 (입력 provenance 는 별개 축 — arm_readable_ && hand_readable_).
  std::uint32_t last_projection_seq_{0};

  // #120: closed-chain 축약 동역학 provider (extended 로봇의 TSID EOM M/h/g 대체). 활성 시
  // combined_cache_.cache().reduced_provider 가 이것을 가리킨다 → 수명은 controller 가 보장.
  // 비활성이면 cache.reduced_provider==nullptr (open-chain, byte-for-byte).
  WbcReducedDynamicsProvider wbc_reduced_dynamics_;

  // Shared combined arm+hand control-model cache wiring (model select, ext→
  // Pinocchio reorder map, per-tick state scatter, arm-TCP FK / Jacobian source,
  // and the PinocchioCache the whole TSID/MPC stack consumes) — see
  // CombinedModelCache (#174). WBC is the superset consumer: it selects the model
  // early (InitModels) via SelectModel, defers the cache Init until its TSID
  // contact frame ids parse (LoadConfig), and injects the reduced-dynamics
  // provider through cache(). InitModels prefers the builder's actuated closed-
  // chain model (extended hands: locks only the loop-passives, keeping every
  // actuated joint movable → nq == nv == 16 for UR5e + 10-DoF hand), else the
  // reduced tree `wbc`, else the raw URDF full model (nq=26, nv=21 with first-
  // class mimic). TSID + MPC share this model.
  CombinedModelCache combined_cache_;

  // Runtime DoF (resolved by LoadConfig/OnDeviceConfigsSet from YAML +
  // device configs). arm_dof_ from the required YAML `arm_dof` key (one
  // source, #340); hand_dof_ from the secondary group's joint_state_names size
  // when declared, else the device's own first reported num_channels.
  // full_dof_ = arm_dof_ + hand_dof_. All RT-path loops iterate over these.
  int arm_dof_{0};
  int hand_dof_{0};
  int full_dof_{0};
  // Which of hand_dof_'s two sources won (#307) — see demo_joint_controller.hpp
  // for why the diagnostic cannot name one key unconditionally.
  bool hand_dof_from_config_{false};

  // ── Posture task split gains (arm vs hand) ──────────────────────────────
  // Parsed from `tsid.tasks.posture.{arm,hand}.{kp,kd}` (ParsePostureSplitGains)
  // and applied per-DoF via the reorder map in ApplyPostureGains. Defaults are
  // critically damped (ζ=1, Kd=2√Kp); ωn is set below each group's joint-
  // position-controller inner-loop dominant pole (servo kp/kd): arm ≈5 rad/s
  // (servo 3000/600=750/150), hand ≈20 rad/s (servo 2.5/0.125). When the split
  // keys are absent (posture_split_gains_=false) these are unused.
  bool posture_split_gains_{false};
  double posture_kp_arm_{36.0};  ///< ωn=6 rad/s, ζ=1
  double posture_kd_arm_{12.0};
  double posture_kp_hand_{49.0};  ///< ωn=7 rad/s, ζ=1
  double posture_kd_hand_{14.0};

  // Sensor capability cache — populated in OnDeviceConfigsSet from
  // GetSensorLayout(secondary). See demo_joint_controller.hpp for rationale.
  bool has_native_contact_{false};
  bool has_native_displacement_{false};

  // ══════════════════════════════════════════════════════════════════════════
  // Common WBC stage (ComputeWbcCommon) — shared per-tick state extraction,
  // pinocchio + contact/grasp cache, and the joint/SE3 references BOTH QPs
  // consume. Produces no command and runs no solve; its outputs (cache,
  // control_ref_, ctrl_state_, n_lambda_active_, tcp_traj_state_) feed both
  // ComputeDynamicWbc and ComputeKinematicWbc. (SE3-trajectory members live in
  // the Trajectory section below, next to their init helpers.)
  // ══════════════════════════════════════════════════════════════════════════
  // (The PinocchioCache lives in combined_cache_ above; the whole TSID/MPC stack
  // consumes it via combined_cache_.cache().)
  rtc::tsid::ContactState contact_state_;
  rtc::tsid::ControlReference control_ref_;
  rtc::tsid::RobotModelInfo robot_info_;
  rtc::tsid::ContactManagerConfig contact_mgr_config_;
  // ControlState built at the end of ComputeWbcCommon; consumed by the Dynamic
  // solve. Declared in Common (it is a Common-stage output, not a Dynamic member).
  rtc::tsid::ControlState ctrl_state_;

  // Posture reference (Pinocchio joint order, full model). The measured per-tick
  // q/v live in combined_cache_ (combined_cache_.q()/.v()); this posture target
  // shares their layout so it is a drop-in q_des for the RT path.
  Eigen::VectorXd
      q_des_target_full_;  ///< [nv] posture reference = external joint target
                           ///< (active driving phases). Same layout as combined_cache_.q().

  // Active contact-force dimension of this tick, computed once in
  // ComputeWbcCommon (grasp cache) and reused by ComputeDynamicWbc's
  // throttled solve-summary log so the value is not recomputed.
  int n_lambda_active_{0};

  // Stage A-3: per-tick force-reference closed-loop helper. Replaces the
  // open-loop λ_des push that used to fire on phase entry only. Updated
  // every RT tick inside ComputeTSIDPosition while phase ∈ {kClosure,
  // kHold}; reset on the kClosure entry edge so integrators start clean.
  ::integrated_bringup::wbc::ForceReferenceUpdater force_ref_updater_;
  // Pre-allocated λ_des vector (size = contact_mgr_config_.max_contact_vars),
  // sized once in LoadConfig so RT path never resizes.
  Eigen::VectorXd force_lambda_des_;

  // Stage B-5: object-level WBC components shared across the three
  // object-level tasks (object_wrench / internal_force / object_se3). The
  // controller owns these instances so the tasks can hold non-owning
  // references and the cache is populated *exactly once per RT tick* in
  // ComputeTSIDPosition. Call-order invariant (plan handoff): cache.Update
  // → contact_state_.UpdateActivation → RecomputeActive →
  // contact_mgr_.ActiveLambdaDim → contact_mgr_.ComputeGraspMatrix(G_workspace_)
  // → grasp_cache_.Compute(G_workspace_, n_active). After that, the three
  // tasks only read GPinv/GTPinv/ProjN/Rank — never re-Compute.
  // NOTE (declaration order): these are declared before tsid_controller_ (Dynamic
  // section) so they out-live the TSID tasks that hold non-owning references to
  // them — on teardown the tasks (in tsid_controller_) destruct first.
  rtc::tsid::ContactManager contact_mgr_;
  rtc::tsid::GraspCache grasp_cache_;
  rtc::tsid::ObjectFrame object_frame_;
  rtc::tsid::IdentityObjectStateProvider object_state_provider_;
  // Pre-allocated 6 × max_contact_vars buffer for ComputeGraspMatrix.
  // Resized once in LoadConfig; RT path only writes top-left active block.
  Eigen::MatrixXd grasp_G_workspace_;
  // Pre-allocated max_contact_vars buffer for SetSqueezeReference (Stage
  // B-5 keeps the squeeze reference at zero — placeholder for a future
  // dynamic squeeze planner).
  Eigen::VectorXd squeeze_lambda_des_;
  // Object mass [kg] from `tsid.object_frame.mass`. Used to seed
  // w_obj_des = [0, 0, m·g, 0, 0, 0] on kClosure/kHold entry. 0.0 keeps
  // ObjectWrenchTask a no-op (residual on zero wrench).
  double object_mass_kg_{0.0};

  // Phase presets — pre-resolved from YAML at init for RT-safe access.
  // Indexed by static_cast<int>(WbcPhase).
  std::array<rtc::tsid::PhasePreset, kNumPhases> phase_presets_{};
  std::array<bool, kNumPhases> phase_preset_valid_{};

  // ── Fingertip sensor data (parsed in ReadState) ─────────────────────────
  //
  // Populated each tick from state.devices[1].inference_data slots 1..3
  // (fx, fy, fz). Backend (hardware-facing) packs only force; controller
  // (behavior-facing) derives in_contact / force_rate. Consumed by contact
  // detection (kClosure -> kHold) and slip monitoring (kHold |df/dt| ->
  // kFallback). Deformation guard is TODO(layer-d) — current fingertip
  // sensors do not publish displacement.
  struct FingertipSensorData {
    std::array<float, 3> force{};      ///< fx, fy, fz (link frame, N)
    float force_magnitude{0.0f};       ///< ||force||  (cached, N)
    float prev_force_magnitude{0.0f};  ///< previous tick, for df/dt
    float force_rate{0.0f};            ///< df/dt [N/s] (smoothed, derived)
    float contact_flag{0.0f};          ///< native sigmoid prob (sensor A only)
    bool valid{false};                 ///< inference_enable[f] (backend fresh)
    bool in_contact{false};            ///< capability-aware: |F|>grasp_force_threshold
                                       ///< AND (sensor A also requires native
                                       ///< prob > grasp_contact_threshold).
                                       ///< Separate from FSM force_contact_threshold_
                                       ///< (phase.cpp gates on force_magnitude).
  };

  std::array<FingertipSensorData, rtc::kMaxSensorGroups> fingertip_data_{};
  int num_active_fingertips_{0};
  bool force_rate_initialized_{false};

  // ── In-plane pull-force estimator (#167) ──────────────────────────────────
  // Measured R_i·f_i over the TSID contact geometry (NOT λ_opt): slots resolve
  // against tsid.contacts frame names (1:1 with the fingertip sensor lanes),
  // and per-tick R_i/p_i come from combined_cache_.cache().contact_frames[i].oMf.
  // contact_geometry_fresh_ marks ticks where ComputeWbcCommon refreshed that
  // cache — non-TSID ticks (kFallback / uninitialized) feed invalid inputs so
  // the estimate decays instead of consuming stale frames. Output rides
  // wbc_state_.pull.
  PullEstimatorWiring pull_wiring_;

  // ── Generalized-momentum observer (#135 Layer 1b) ─────────────────────────
  // Parsed in LoadConfig (the YAML is there) and BUILT in OnDeviceConfigsSet,
  // which is the first point the arm device's joint_state_names exist — the
  // wiring pins that order on its own model handle, so it cannot be built any
  // earlier. Disabled without a `momentum_observer` block. Output is the
  // joint-space residual r; it goes to momentum_observer.csv and nowhere else
  // this layer (#135 D12 — the PayloadEstimate wire surface arrives with
  // Layer 2A, when there is a payload estimate to put in it).
  MomentumObserverParams momentum_params_;
  MomentumObserverWiring momentum_wiring_;
  /// RT → publish-thread egress for the PayloadEstimate topic (#135 D12).
  /// Written on every enabled tick by UpdateMomentumObserverChannels, read by
  /// PublishNonRtSnapshot. The SAME row the CSV lane pushes, so the file and
  /// the topic cannot report different numbers for one tick.
  rtc::SeqLock<integrated_bringup::MomentumObserverLogPod> payload_estimate_lock_;
  /// Non-empty when BuildMomentumObserverWiring threw in OnDeviceConfigsSet.
  /// That hook cannot propagate — CM calls SetDeviceNameConfigs outside any
  /// try/catch — so the failure is latched here and on_configure turns it into
  /// CallbackReturn::FAILURE. A config error must not degrade to "the observer
  /// quietly did not run".
  std::string momentum_config_error_;
  bool contact_geometry_fresh_{false};

  /// Stage the per-contact inputs from fingertip_data_ + contact frames and
  /// run the shared estimator update (end of ComputeControl). noexcept,
  /// heap-free (RT tick path); no-op while the estimator is disabled.
  void UpdatePullEstimate(double dt) noexcept;

  // ══════════════════════════════════════════════════════════════════════════
  // Dynamic WBC (ComputeDynamicWbc) — TSID inverse-dynamics QP: solves the
  // whole-body acceleration/torque (tsid_output_.a_opt / .tau), then overlays
  // the hand feedforward torque (kPdFeedforward). tsid_output_.a_opt is now
  // log-only — the Kinematic CLIK-QP owns position and no longer consumes it.
  // A QP failure is NON-critical (decision 6): it drops the hand τ_ff this tick
  // only; position is owned by the Kinematic CLIK-QP. (tsid_* keep their
  // library-type names — TSID = Task-Space Inverse *Dynamics* — rather than a
  // dyn_qp_ rename; see plan Phase 5 Part B.)
  // ══════════════════════════════════════════════════════════════════════════
  rtc::tsid::TSIDController tsid_controller_;
  rtc::tsid::CommandOutput tsid_output_;
  bool tsid_initialized_{false};
  // dyn_qp_fail_count_ — Dynamic WBC (TSID-ID) QP. NON-critical: a failure drops
  //   the hand τ_ff this tick only; the Kinematic CLIK-QP still owns position.
  //   Mirrors the rtc_msgs/WbcState `qp_fail_count` + `tsid_solver_ok` fields
  //   (both always referred to the TSID solve). [counterpart: kin_qp_fail_count_]
  int dyn_qp_fail_count_{0};
  // Stage C-3: true for the ticks where the hand is driven via kPdFeedforward
  // (hand_tauff_enable + closure/hold + all-finite τ_ff). WriteJointCommand
  // reads this to set the hand device command_type + feedforward; ComputeEstop
  // never sets it, so E-STOP always falls back to a plain position hold (E-8).
  // Owns only hand_computed_.feedforward (the τ_ff overlay); positions/velocities
  // are Kinematic-owned (see ComputedTrajectory below).
  bool hand_tauff_active_{false};

  // ══════════════════════════════════════════════════════════════════════════
  // Kinematic WBC (ComputeKinematicWbc) — CLIK-QP position backbone. CLIK is the
  // SOLE position backbone (the integrator A/B path was removed once verified),
  // so a CLIK failure here is CRITICAL (kin_qp_fail_count_ → hold-last →
  // kFallback). (clik_* keep their CLIK names — the public ClikReferenceGenerator
  // API and the ROS-param/YAML/CSV keys depend on them; a kin_qp_ rename would
  // break external interfaces — see plan Phase 5 Part B.)
  // ══════════════════════════════════════════════════════════════════════════
  // CLIK reference path (Stage C-2). clik_ is Init'd in on_configure (InitClik).
  // clik_enabled_ gates the whole thing — InitClik enforces the nq==nv CLIK
  // contract and on_configure FAILS the lifecycle transition if CLIK cannot be
  // enabled (DEC-1 ⓐ), so on the accepted config path clik_enabled_ is always
  // true (there is no longer an integrator fallback to degrade to).
  rtc::tsid::ClikReferenceGenerator clik_;
  bool clik_enabled_{false};
  double clik_damping_sq_{1e-4};       ///< μ² damped right-inverse (YAML clik.damping_sq)
  double clik_v_limit_{1.5};           ///< per-joint |v_ref| clamp [rad/s] (YAML clik.v_limit)
  double clik_w_task_{1.0};            ///< L1 TCP tracking weight (YAML clik.w_task)
  double clik_w_arm_{1e-2};            ///< L2 arm posture weight (YAML clik.w_arm)
  double clik_w_hand_{1e-2};           ///< L3 hand posture weight (YAML clik.w_hand)
  double clik_anchor_drift_max_{0.0};  ///< carry-forward anti-windup clamp [rad] (YAML
                                       ///< clik.anchor_drift_max; ≤0 → off)
  int clik_tcp_frame_idx_{-1};         ///< combined_cache_.cache().registered_frames index (tip)
  int clik_base_frame_idx_{-1};        ///< registered_frames index (base; < 0 = universe)
  bool clik_compute_ok_{false};        ///< clik_.Compute() succeeded this tick
  double clik_tcp_err_{0.0};           ///< ‖e_x‖ [m+rad] (CLIK diagnostic)
  double clik_manip_{0.0};             ///< √det(JJᵀ+μ²I) (CLIK diagnostic)
  // RT-thread-only. Set on a new joint/SE3 target arrival, first tick, and
  // kIdle/kRelease phase entry; consumed (and cleared) by ComputeKinematicWbc,
  // which passes it as clik_.Compute(reseed_anchor=...). When set, CLIK re-anchors
  // q_ref to the measured state; otherwise q_ref carries forward from the previous
  // desired (DemoTaskController's desired_q_ pattern).
  bool clik_reseed_pending_{false};
  // Runtime gain vector forwarded to clik_.SetTaskGain each tick (pre-sized).
  Eigen::Matrix<double, 6, 1> clik_kx_{Eigen::Matrix<double, 6, 1>::Zero()};

  // kin_qp_fail_count_ — Kinematic WBC (CLIK) QP. CRITICAL: CLIK is the position
  //   backbone, so a failure (while CLIK drives) holds last this tick and trips
  //   kFallback after max_qp_fail_before_fallback_ consecutive fails.
  int kin_qp_fail_count_{0};
  int max_qp_fail_before_fallback_{5};

  // Command limits (Pinocchio joint order). Consumed by the CLIK-QP
  // position/velocity box constraints (forwarded to clik_ in InitClik).
  Eigen::VectorXd q_min_clamped_;  ///< [nv] q_lower + margin
  Eigen::VectorXd q_max_clamped_;  ///< [nv] q_upper - margin
  Eigen::VectorXd v_limit_;        ///< [nv] velocity limit

  // Per-tick command output (intermediate, device order). robot_computed_ and
  // hand_computed_.positions/.velocities are Kinematic-owned; hand_computed_.
  // feedforward is Dynamic-owned (filled by ComputeDynamicWbc's τ_ff overlay).
  struct ComputedTrajectory {
    std::array<double, kMaxDeviceChannels> positions{};
    std::array<double, kMaxDeviceChannels> velocities{};
    // Per-joint feedforward torque [Nm], populated only for the hand device
    // when hand_tauff_enable (kPdFeedforward). Arm leaves it zero. [Dynamic-owned]
    std::array<double, kMaxDeviceChannels> feedforward{};
  };

  ComputedTrajectory robot_computed_{};
  ComputedTrajectory hand_computed_{};

  // Fills one device's trajectory_* + goal_positions from a computed trajectory.
  // Shared by FillLogOutput / FillPublishOutput (declared here for nested-type
  // visibility of ComputedTrajectory).
  void FillDeviceTrajectoryPods(rtc::DeviceOutput& out, int num_channels,
                                const ComputedTrajectory& computed, int target_slot) noexcept;

  // ── Target management ───────────────────────────────────────────────────
  // RT thread is the SOLE writer of target_seqlock_. Off-RT SetDeviceTarget /
  // SetDeviceTaskTarget callers marshal onto the base mailbox; the RT thread
  // drains it in Compute() (#206).
  static constexpr std::size_t kSE3RotDoubles = 9;
  static constexpr std::size_t kSE3TransDoubles = 3;

  struct TargetSlot {
    std::array<std::array<double, kMaxDeviceChannels>, ControllerState::kMaxDevices> targets{};
    std::array<double, kSE3RotDoubles> tcp_goal_rot{};
    std::array<double, kSE3TransDoubles> tcp_goal_t{};
    bool tcp_goal_valid{false};
    // Commanded SE3 target (off-RT SetDeviceTaskTarget → RT). Kept separate from
    // the tcp_goal_* FK-seed mirror above (which kIdle/kApproach overwrite from
    // the current FK pose): this holds the externally commanded pose so the seed
    // logic can prefer it over measured self-hold. Materialised from (x,y,z,r,p,y)
    // to a rotation matrix at drain time, mirroring DemoTask's 6→SE3 convention.
    std::array<double, kSE3RotDoubles> tcp_cmd_rot{};
    std::array<double, kSE3TransDoubles> tcp_cmd_t{};
    bool tcp_cmd_valid{false};
  };

  static_assert(std::is_trivially_copyable_v<TargetSlot>,
                "TargetSlot must be trivially copyable for SeqLock<TargetSlot>");

  // One drained target → this controller's slot. Called on the RT tick from
  // DrainTargetSlot(), with device_idx and values already bounded by the base.
  // `is_task` routes between the two independent slots: joint posture
  // (targets[]) and the commanded SE3 (tcp_cmd_*).
  void ApplyPendingTarget(int device_idx, std::span<const double> values,
                          bool is_task) noexcept override;

  // Set by ApplyPendingTarget when a drained entry actually reached the slot.
  // Not every surviving entry does — a task goal on a device with no SE3 slot,
  // or one too short to form a pose, is dropped on the floor — so the drain
  // cannot infer "slot changed" from the entry count alone. RT-thread-only.
  bool drained_slot_dirty_{false};

  rtc::SeqLock<TargetSlot> target_seqlock_;
  std::atomic<bool> target_initialized_{false};

  // Base hook — see demo_joint_controller.hpp. Previously duplicated inside
  // on_activate (#196 §3). ClearEstop keeps its own reset: E-STOP recovery is a
  // separate trigger that does not go through a lifecycle transition.
  void ResetTargetInitialization() noexcept override {
    target_initialized_.store(false, std::memory_order_release);
  }

  TargetSlot current_target_slot_{};
  bool robot_new_target_pending_{false};     // RT-thread-only
  bool hand_new_target_pending_{false};      // RT-thread-only
  bool arm_task_new_target_pending_{false};  // RT-thread-only: new commanded SE3 arrived

  // RT-thread-only: refresh current_target_slot_ from the SeqLock + drain
  // pending entries. Also flips robot/hand _pending_ flags for the FSM.
  void DrainTargetSlot(const ControllerState& state) noexcept;

  // RT-thread-only: rebuild q_des_target_full_ from current_target_slot_.targets
  // (external order → Pinocchio order via combined_cache_.ext_to_pin_q, mirroring
  // combined_cache_.ExtractFullState). Called at phase entry alongside the SE3 goal so posture +
  // SE3 reference share one consistent target snapshot. No-op until
  // combined_cache_.reorder_valid().
  void BuildTargetPosture(const ControllerState& state) noexcept;

  // RT-thread-only: fold the hand joint target (current_target_slot_.targets[1])
  // into the hand block of q_des_target_full_ (external → Pinocchio order). Split
  // out of BuildTargetPosture so Compute() can refresh it per-tick on a fresh
  // hand target, making the hand joint target a live command in every phase
  // (except kRelease/kFallback) rather than consumed only on a closure edge.
  // Returns true iff the fold applied (reorder map ready + hand device valid).
  bool BuildHandTargetPosture(const ControllerState& state) noexcept;

  // RT-thread-only: snapshot the current measured configuration as the idle
  // hold target. Sets current_target_slot_.targets[] (joint_goal mirror) =
  // measured, rebuilds q_des_target_full_ (posture reference), and seeds
  // tcp_goal_ from the current FK (base_frame → tip) with its SeqLock-POD
  // mirror. Called on first-tick self-init and on every kIdle entry so idle
  // regulates toward a fixed init snapshot, re-seeded to where the robot is
  // *now* on each entry. Caller persists current_target_slot_ to the SeqLock.
  void SeedHoldFromMeasured(const ControllerState& state) noexcept;

  // RT-thread-only. If current_target_slot_.tcp_cmd_valid, overrides tcp_goal_
  // (and its SeqLock-POD mirror tcp_goal_*) with the commanded SE3 so the
  // commanded pose takes priority over the measured/joint-target FK seed.
  // Returns true if a commanded pose was applied. Does NOT clear tcp_cmd_valid
  // — it persists across ticks until a grasp/release/fallback entry clears it.
  // Caller is responsible for persisting current_target_slot_ to the SeqLock and
  // for (re)initialising the TCP trajectory when an SE3 ramp is needed.
  bool ApplyCommandedSe3IfPresent() noexcept;

  // Aux-thread spawn of MPC thread (idempotent). Called from on_activate so
  // the heap-allocating Factory::Create + thread.Start happen off the RT
  // path. MPCFactory is given a zero-initialised PhaseContext, matching the
  // legacy InitializeHoldPosition semantics.
  void SpawnMpcThreadIfNeeded() noexcept;

  // LoadConfig section 7: parse `mpc:` and pre-build the handler-mode model /
  // phase-manager preconditions. Self-contained (reads cfg, sets mpc_* members);
  // extracted from LoadConfig to keep that method readable.
  void ConfigureMpc(const YAML::Node& cfg);

  // RT-thread working SE3 (materialised from current_target_slot_.tcp_goal_*).
  pinocchio::SE3 tcp_goal_{pinocchio::SE3::Identity()};
  bool tcp_goal_valid_{false};

  // ── Trajectory (position mode phases) ───────────────────────────────────
  // Templates fixed at compile-time capacity; only the first arm_dof_ /
  // hand_dof_ slots are initialised + read at runtime (caller-trim pattern).
  trajectory::JointSpaceTrajectory<kMaxArmDof> robot_trajectory_;
  trajectory::JointSpaceTrajectory<kMaxHandDof> hand_trajectory_;
  double robot_trajectory_time_{0.0};
  double hand_trajectory_time_{0.0};

  // TCP SE3 trajectory — used in MPC-disabled mode only, gated by
  // se3_task_active_in_phase_[phase_]. On phase entry (SE3-inactive →
  // SE3-active) InitTcpTrajectory builds a quintic rest-to-rest segment
  // from current FK pose to tcp_goal_ (with π-rotation split when needed).
  // MPC-enabled mode keeps the legacy OnPhaseEnter SE3 step path.
  trajectory::TaskSpaceTrajectory tcp_trajectory_;
  trajectory::TaskSpaceTrajectory::State tcp_traj_state_{};
  double tcp_trajectory_time_{0.0};
  bool tcp_trajectory_active_{false};
  pinocchio::SE3 pending_tcp_goal_{pinocchio::SE3::Identity()};
  double pending_tcp_duration_{0.0};
  bool has_pending_tcp_segment_{false};
  // True iff phase_presets_[p].task_presets contains active se3_tcp entry.
  // Filled once in LoadConfig — phase ID hardcoding 0 (YAML is SSoT).
  std::array<bool, kNumPhases> se3_task_active_in_phase_{};

  void InitTcpTrajectory(const ControllerState& state) noexcept;

  [[nodiscard]] bool Se3TaskActiveInPhase(WbcPhase p) const noexcept {
    const auto idx = static_cast<std::size_t>(p);
    return idx < kNumPhases && se3_task_active_in_phase_[idx];
  }

  // ── Device limits ───────────────────────────────────────────────────────
  std::array<std::vector<double>, ControllerState::kMaxDevices> device_max_velocity_;
  std::array<std::vector<double>, ControllerState::kMaxDevices> device_position_lower_;
  std::array<std::vector<double>, ControllerState::kMaxDevices> device_position_upper_;

  // ── MPC integration (Phase 5 + 7b) ──────────────────────────────────────
  //
  // When `mpc_enabled_` is true, `SpawnMpcThreadIfNeeded()` (called from
  // on_activate on the aux thread) spawns one of two MPC thread
  // implementations:
  //
  //   `mpc.engine: "mock"`     (Phase 5, default) — MockMPCThread publishes
  //                            a self-regularising hold target. Keeps the
  //                            Phase 4 fixed-reference + Riccati-scaled
  //                            path alive without pulling in Aligator.
  //
  //   `mpc.engine: "handler"`  (Phase 7b, opt-in) — HandlerMPCThread drives
  //                            a real ContactLight / ContactRich solve via
  //                            MPCFactory, with GraspPhaseManager supplying
  //                            phase context. Cross-mode swap between
  //                            contact_light and contact_rich is handled
  //                            inside HandlerMPCThread.
  //
  // If `mpc_enabled_` is false, no MPC thread is spawned and the Phase 4
  // self-regularising hold path runs exclusively.
  enum class MpcEngine { kMock, kHandler };

  bool mpc_enabled_{false};
  // Launch-profile opt-out (issue #350). True when this run's layout profile
  // dropped the MPC cores, i.e. the cset shield handed them back to the system
  // cpuset. Read once at configure from the `rt_layout_profile` node parameter
  // — the launch owns that decision because MPC activation is not observable
  // from the host: `mpc.enabled` is controller YAML and the thread is spawned
  // in on_activate, so a controller switch can turn it on mid-session.
  //
  // With this true, on_activate refuses a structurally MPC-enabled config
  // instead of spawning a SCHED_FIFO thread onto a core that is no longer
  // shielded. It does NOT refuse the controller: a TSID-only configuration
  // (`mpc.enabled: false`) activates normally, which is the whole point of the
  // profile.
  bool layout_profile_drops_mpc_{false};
  MpcEngine mpc_engine_{MpcEngine::kMock};
  // MPC solve-loop frequency (Hz). Loaded from YAML `mpc.target_frequency_hz`
  // and forwarded to MpcThreadLaunchConfig in SpawnMpcThreadIfNeeded. Default
  // 20 Hz preserves prior hardcoded behaviour.
  double mpc_target_frequency_hz_{20.0};
  rtc::mpc::MPCSolutionManager mpc_manager_;
  std::unique_ptr<rtc::mpc::MPCThread> mpc_thread_;

  // Handler-mode dependencies (null when engine != kHandler).
  //
  // `mpc_model_handler_` stays owned by the controller because HandlerMPCThread
  // only holds a non-owning reference to it; its lifetime must bracket the
  // MPC thread. `phase_manager_owned_` holds the manager between LoadConfig
  // (build + validate YAML) and SpawnMpcThreadIfNeeded (ownership transferred
  // into HandlerMPCThread::Configure). `phase_manager_ptr_` is a borrowed
  // raw pointer that stays valid for the thread's lifetime; the controller
  // uses it to bridge WBC FSM edges onto the grasp FSM command bus
  // (`SetCommand` / `ForcePhase`).
  std::unique_ptr<rtc::mpc::RobotModelHandler> mpc_model_handler_;
  std::unique_ptr<integrated_bringup::phase::GraspPhaseManager> phase_manager_owned_;
  integrated_bringup::phase::GraspPhaseManager* phase_manager_ptr_{nullptr};

  // Pre-parsed YAML nodes (kept alive for handler-mode startup and for
  // cross-mode swap inside HandlerMPCThread).
  YAML::Node mpc_light_cfg_;
  YAML::Node mpc_rich_cfg_;
  YAML::Node phase_cfg_;

  // Reference buffers sized to combined_cache_.model()->nq/nv (= reduced tree when
  // available). Populated each tick by ComputeReference, consumed by TSID
  // via control_ref_.{q_des,v_des,a_des}.
  Eigen::VectorXd mpc_q_ref_;
  Eigen::VectorXd mpc_v_ref_;
  Eigen::VectorXd mpc_a_ff_;
  Eigen::VectorXd mpc_lambda_ref_;
  Eigen::VectorXd mpc_u_fb_;

  // ── MPC tick-timing observability (aux thread, non-RT) ────────────────
  // Owned 1 Hz timer (spawned in on_activate when mpc_enabled_) drains the
  // MPCThread's TimingProducer SPSC ring and appends one row per MPC tick
  // to <session>/timing/mpc_timing_log.csv via
  // MpcTimingLogger (a thin wrapper over the generic
  // ThreadTimingCsvLogger<RtTickTimingPayload>, schema unified with the CM
  // RT loop). Aggregate INFO line every 10 ticks (~10 s) for tmux watchers.
  rclcpp::CallbackGroup::SharedPtr mpc_timing_cb_group_;
  rclcpp::TimerBase::SharedPtr mpc_timing_timer_;
  rtc::mpc::MpcTimingLogger mpc_timing_logger_;
  std::uint32_t mpc_timing_tick_{0};
  // Logger / timer setup is one-shot per controller lifetime — gated on this
  // flag so repeated activate/deactivate cycles (Phase 2 lifecycle switch)
  // don't truncate the CSV or churn timer registration.
  bool mpc_timing_initialized_{false};
  void LogMpcSolveTimingTick() noexcept;

  // ── FSM thresholds ──────────────────────────────────────────────────────
  double epsilon_pregrasp_{0.005};       ///< m, approach → closure (TCP-to-goal)
  double force_contact_threshold_{0.2};  ///< N, contact detection
  int min_contacts_for_hold_{2};         ///< # fingertips required -> kHold
  double slip_rate_threshold_{5.0};      ///< N/s, |df/dt| slip guard (kHold)
  double deformation_threshold_{0.015};  ///< m, ||disp|| guard (kHold)

  // Integration safety margins
  double position_margin_{0.02};  ///< rad, from joint limits
  double velocity_scale_{0.95};   ///< fraction of max velocity
  float force_rate_alpha_{0.1f};  ///< EMA smoothing for df/dt (500Hz→~20Hz BW)

  // Stage A-5b: contact activation ramp time (seconds). Phase transitions
  // call SetActivationTarget(i, 1.0|0.0, contact_ramp_sec_); per-tick
  // ContactState::UpdateActivation(dt) linearly progresses s_i toward the
  // target. YAML: `tsid.contacts_default_ramp_sec` (default 0.1 = 100 ms).
  double contact_ramp_sec_{0.1};

  // kRelease 2-stage state. Stage 0: contact activation_target ramps to 0
  // over release_ramp_sec_; stage 1: hand finger-open trajectory plays.
  // release_done_ flips on stage-1 completion → UpdatePhase routes back to
  // kIdle. YAML: `fsm.release_ramp_sec` (default 0.03 = 30 ms).
  int release_stage_{0};
  double release_elapsed_s_{0.0};
  double release_ramp_sec_{0.03};
  bool release_done_{false};

  // ── Utility ─────────────────────────────────────────────────────────────
  [[nodiscard]] double ComputeTcpError(const pinocchio::SE3& target) noexcept;

  // ── Logging ─────────────────────────────────────────────────────────────
  rclcpp::Logger logger_{integrated_bringup::logging::DemoWbcLogger()};
  rclcpp::Clock log_clock_{RCL_STEADY_TIME};

  // ── Phase 4: controller-owned topic sub/pub handles ───────────────────
  ControllerTopicHandles owned_topics_;

  // RT compute → publish-thread handoff for the controller-owned wbc_state
  // topic. Compute() fills `wbc_state_` then stores into `wbc_state_lock_`;
  // the publish thread Loads it inside PublishNonRtSnapshot.
  ::integrated_bringup::WbcStateData wbc_state_{};
  rtc::SeqLock<::integrated_bringup::WbcStateData> wbc_state_lock_;
  // ── Phase D (gain→parameter migration): per-controller ROS 2 parameters ──
  //
  // Tunable: arm_trajectory_speed, hand_trajectory_speed, se3_weight,
  //          force_weight, posture_weight, mpc_enable (runtime gate),
  //          riccati_gain_scale.
  // Read-only (D-2): arm_max_traj_velocity, hand_max_traj_velocity.
  //
  // Force-PI grasp on DemoWbc has different semantics than DemoTask/DemoJoint:
  // there is no `grasp_controller_` — the GRASP/RELEASE codes drive the WBC
  // FSM via grasp_cmd_ atomic + gains.grasp_target_force. The grasp_command
  // srv handler sets both atomically. NONE is rejected (use lifecycle
  // deactivate to abort).
  void DeclareGainParameters() noexcept;

  // Close open log channels and unbind every typed LogHandle (#238). Called
  // from on_cleanup and from on_configure's failure-rollback paths so a leaked
  // channel never makes the next RegisterLog() return an unbound handle.
  void ResetLogState() noexcept;
  rcl_interfaces::msg::SetParametersResult OnGainParametersSet(
      const std::vector<rclcpp::Parameter>& params) noexcept;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::Service<rtc_msgs::srv::GraspCommand>::SharedPtr grasp_command_srv_;

  // ── Phase C (controller-owned CSV logging) ────────────────────────────
  struct ParsedLogEntry {
    std::string msg_type;
    std::string instance;
  };

  std::vector<ParsedLogEntry> parsed_log_entries_;

  rtc::ControllerLogSet log_set_{"demo_wbc_controller"};

  // Lifetime CSV drop count already reported by the drain timer (#234 P-20) —
  // DrainControllerLogs warns once per new burst rather than once per drain.
  std::uint64_t log_drops_reported_{0};
  // WBC arm/hand state channels use the WBC-specific superset POD
  // (DeviceWbcLog) instead of the generic DeviceStateLog — adds a_opt
  // acceleration, SE3 trajectory (arm), and fingertip force (hand). The hand
  // sensor channel stays generic (DeviceSensorLog). wbc_diag is a single
  // per-tick TSID/QP diagnostics channel. See ~/.claude/plans/wbc-csv-logging.md.
  rtc::LogHandle<integrated_bringup::DeviceWbcLogPod> primary_wbc_log_handle_;
  rtc::LogHandle<integrated_bringup::DeviceWbcLogPod> secondary_wbc_log_handle_;
  rtc::LogHandle<integrated_bringup::DeviceSensorLogPod> secondary_sensor_log_handle_;
  rtc::LogHandle<integrated_bringup::WbcDiagLogPod> wbc_diag_log_handle_;
  rtc::LogHandle<integrated_bringup::PullEstimatorLogPod> pull_estimator_log_handle_;
  rtc::LogHandle<integrated_bringup::MomentumObserverLogPod> momentum_observer_log_handle_;

  std::vector<std::string> primary_joint_names_;
  std::vector<std::string> secondary_joint_names_;
  std::vector<std::string> secondary_motor_names_;
  std::vector<std::string> secondary_sensor_names_;
  /// Hand DeviceSensorLayout stride, cached for the CSV header/row writers. 0
  /// on a force-only hand → no barometer/ToF columns at all.
  std::size_t secondary_sensor_values_per_group_{
      integrated_bringup::DeviceSensorLogPod::kSensorValuesPerFingertip};

  rclcpp::CallbackGroup::SharedPtr log_drain_cb_group_;
  rclcpp::TimerBase::SharedPtr log_drain_timer_;
};

}  // namespace integrated_bringup

#endif  // UR5E_BRINGUP_CONTROLLERS_DEMO_WBC_CONTROLLER_H_
