// ── locomotion_controller — the humanoid whole-body controller ────────────────
//
// One tick: the measured state (MeasuredStateSource) → the full model's
// kinematics (every end-effector and contact frame, hands at their measured
// pose) → the reduced model's dynamics (hands locked at neutral: the hands are
// held still and their mass is folded into the arm ends) → the whole-body QP
// (rtc_tsid LocomotionFormulationBuilder) → pd_feedforward on the body group:
// a position target plus the QP's torque as feedforward, with the controller's
// own non-dominant PD gains on the wire. Hand groups get a kPosition hold at the
// pose they had when the reference latched.
//
// The reference (LocomotionDiagLogPod::ReferenceSource):
//   idle   the latched pose: the measured pose at the first valid tick, zero
//          velocity and acceleration, and the static contact wrench of that
//          pose (two contacts: the weight split and the centres of pressure
//          placed so that they balance gravity at the CoM; otherwise an equal
//          share of the weight). Without an MPC (`mpc:` absent or
//          mpc.enabled false) this is the only reference.
//   stand  the MPC plan, sampled each tick: q / v / a and the contact wrench
//          copied block for block into the QP's reference (base twist and
//          acceleration in the base LOCAL frame, wrenches on world axes, the
//          model's contact order), every contact in support.
//   walk   the same with a contact in swing (the plan's contact mask).
//   fault  the latch below.
// idle → stand needs a usable plan (finite, of the current model and state
// epochs, younger than mpc.max_policy_age_s, past the plausibility check) that
// is not an unconverged standing solve, and every contact in support. A plan
// published unconverged by the online budget keeps stand: the producer's
// publish rule bounds its residual. stand / walk → idle only when the plan is
// lost (evicted, refused, implausible); the pose re-latches from the
// measurement if every contact frame is within a few millimetres of the
// lowest and the base is nearly still — otherwise the fault latches
// (kPlanLoss): re-latching a swing foot would load it.
// A plan the plausibility check refuses stays refused for every later sample.
//
// Modes (LocomotionDiagLogPod::Mode):
//   waiting   no valid state since activation — damping on the raw body lane
//   nominal   the QP's torque
//   degraded  the QP missed this tick — the last valid torque is held (zero
//             until the first solve); the solve health classifies the run
//   fault     latched — damping only: feedforward −Kd_fault·v (clamped to the
//             joint torque limit), position target = the measured q, the PD
//             gains on the wire unchanged. The robot settles slowly under
//             gravity; that is the intent (no large torque from a bad state).
// Fault causes: the QP's health reached Fault; the source was not ok for more
// than fault.max_source_bad_ticks ticks in a row; the output was not finite;
// the state's model epoch is not the controller's; the plan was lost off a
// still double support (above).
//
// Clearing the fault: ResetFault() (the CM's reset service) — the next tick
// resets the QP, re-latches the reference from the measurement and returns to
// nominal; if the cause is still there it latches again on that tick, which
// HasLatchedFault() reports. A reset, an activation and either E-STOP edge
// also restart the MPC's state stream (a new state epoch: no plan of the old
// one is sampled again) and its gait schedule. Unlike the controllers whose fault outlives a
// deactivate / activate cycle, ACTIVATING this controller also clears it: it is
// activated by an operator's switch from another controller that has been
// holding the robot, and a latch from an earlier session describes a robot
// state that switch has already replaced. The first tick re-latches as above
// and logs once that a latched fault was dropped.
//
// YAML (`locomotion_controller:`): model.* (humanoid_model_config.hpp),
// whole_body_qp.* (rtc_tsid; `health:` sets the degraded / fault rule),
// estimator.* (measured_state_source.hpp), and
//   command_type: pd_feedforward      # the only accepted value
//   pd_feedforward: {kp: [...], kd: [...]}   # body roster order, ≥ 0 [N·m/rad, N·m·s/rad]
//   fault: {max_source_bad_ticks: N (≥ 1), damping_kd: [...] (body roster order, ≥ 0)}
//   topics: {<body group>: {...}, <hand group>: {...}}   # body first, then model.hand_groups
//   logs: [...]   # DeviceStateLog per group, JointFeedforwardLog (body), LocomotionDiagLog
//   mpc:          # optional; absent = no MPC (the idle reference only)
//     enabled: bool             # false = as if absent
//     period_nodes: N (≥ 1)     # the MPC thread's period, in OCP node intervals
//     max_policy_age_s: s (≥ 0) # a plan older than this is dropped (0 = the horizon only)
//     riccati: {enabled: bool, ...}   # rtc_mpc MPCSolutionManager
//     sanity: {joint_delta_rad, base_speed, joint_accel, wrench_scale}   # all > 0, below
//     ocp_type / solver / kinodynamic / max_stale_solutions   # rtc_mpc MPCFactory · manager
// With mpc.enabled the planner's sections are read too: command.* (the
// command filter: its base_height is the stance height), gait.*, swing.*,
// terrain.*, and reference.* (its q_default names the body joints and the
// hand joints; the hands are held at their q_default values, and the body
// joints go to the MPC's stage reference).
// The plausibility check of a sampled plan: a contact mask of at least one of
// the model's contacts; a unit base quaternion within sanity.joint_delta_rad
// of the measured orientation; every joint inside its model range widened by
// joint_delta_rad and within joint_delta_rad of its measured value; the base
// height above the lowest contact frame inside [command.h_min, command.h_max];
// the base speed at most sanity.base_speed [m/s]; every joint acceleration (and
// the feedback, when on) at most sanity.joint_accel [rad/s²]; no contact
// pulling by more than 2 % of the weight, and the total normal force between
// 1 / sanity.wrench_scale and sanity.wrench_scale × the robot's weight.
// The MPC thread runs on the `mpc` layout role: activation is refused under a
// launch profile that dropped it (enable_mpc:=false) while mpc.enabled is true.
#ifndef INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_CONTROLLER_HPP_
#define INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_CONTROLLER_HPP_

#include "integrated_bringup/controllers/locomotion/humanoid_model_config.hpp"
#include "integrated_bringup/controllers/locomotion/measured_state_source.hpp"
#include "integrated_bringup/logging/device_state_log_pod.hpp"
#include "integrated_bringup/logging/joint_feedforward_log_pod.hpp"
#include "integrated_bringup/logging/locomotion_diag_log_pod.hpp"
#include "integrated_bringup/support/owned_topics.hpp"
#include "rtc_controller_interface/controller_log_set.hpp"
#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_mpc/handler/mpc_handler_base.hpp"
#include "rtc_mpc/logging/mpc_timing_logger.hpp"
#include "rtc_mpc/manager/mpc_solution_manager.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/phase/locomotion_phase_manager.hpp"
#include "rtc_mpc/reference/command_filter.hpp"
#include "rtc_mpc/thread/handler_mpc_thread.hpp"
#include "rtc_mpc/types/wbc_reference.hpp"
#include "rtc_tsid/locomotion/locomotion_formulation.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/timer.hpp>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace integrated_bringup {

/// Body joints the controller drives (the joint log's capacity).
inline constexpr std::size_t kLocomotionMaxBodyJoints = LocomotionDiagLogPod::kMaxJoints;

/// @brief `mpc:` (header comment). `present` false = the section is absent.
struct LocomotionMpcConfig {
  bool present{false};
  bool enabled{false};
  int period_nodes{1};
  double max_policy_age_s{0.0};  ///< checked here; the plan sampler reads it from `node`
  bool riccati_enabled{false};
  double sanity_joint_delta_rad{0.0};  ///< [rad]
  double sanity_base_speed{0.0};       ///< [m/s]
  double sanity_joint_accel{0.0};      ///< [rad/s²]
  double sanity_wrench_scale{0.0};     ///< × the robot's weight
  YAML::Node node;                     ///< the whole section (factory and manager keys)
};

/// @brief The controller's own sections of its YAML (Pass 1, schema only).
struct LocomotionControllerConfig {
  locomotion::HumanoidModelConfig model;
  locomotion::EstimatorConfig estimator;
  std::vector<double> pd_kp;  ///< body roster order; length checked against the roster
  std::vector<double> pd_kd;
  int max_source_bad_ticks{1};
  std::vector<double> damping_kd;
  YAML::Node whole_body_qp;
  LocomotionMpcConfig mpc;
};

/// @throws std::invalid_argument naming the key on a missing / unknown key of
///         `pd_feedforward`, `fault` or `mpc`, a gain that is negative or not finite, a
///         `command_type` other than pd_feedforward, or a model / estimator section
///         its own parser refuses.
[[nodiscard]] LocomotionControllerConfig ParseLocomotionControllerConfig(const YAML::Node& cfg);

class LocomotionController final : public rtc::RTControllerInterface {
 public:
  using Mode = LocomotionDiagLogPod::Mode;
  using FaultCause = LocomotionDiagLogPod::FaultCause;
  using ReferenceSource = LocomotionDiagLogPod::ReferenceSource;

  explicit LocomotionController(std::string_view urdf_path);
  ~LocomotionController() override;

  [[nodiscard]] rtc::ControllerOutput Compute(const rtc::ControllerState& state) noexcept override;

  void SetDeviceTarget(int /*device_idx*/, std::span<const double> /*target*/) noexcept override {}

  [[nodiscard]] GoalKinds AcceptedGoalKinds(int /*device_idx*/) const noexcept override {
    return {.joint = false, .task = false};
  }

  [[nodiscard]] std::string_view Name() const noexcept override { return "LocomotionController"; }

  [[nodiscard]] rtc::CommandType GetCommandType() const noexcept override {
    return rtc::CommandType::kPdFeedforward;
  }

  void TriggerEstop() noexcept override;
  void ClearEstop() noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void ResetFault() noexcept override;
  [[nodiscard]] bool HasLatchedFault() const noexcept override;

  void LoadConfig(const YAML::Node& cfg) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state,
                              rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                              const YAML::Node& yaml_cfg) noexcept override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) noexcept override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) noexcept override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) noexcept override;
  void PublishNonRtSnapshot(const rtc::PublishSnapshot& snap) noexcept override;

  // ── Read-only views (tests, diagnostics; the RT thread's last tick) ────────
  [[nodiscard]] const std::string& ConfigError() const noexcept { return config_error_; }

  [[nodiscard]] const LocomotionDiagLogPod& LastDiag() const noexcept { return diag_; }

  [[nodiscard]] const JointFeedforwardLogPod& LastJointLog() const noexcept { return joint_log_; }

  [[nodiscard]] const locomotion::MeasuredRobotState& LastMeasuredState() const noexcept {
    return measured_;
  }

  [[nodiscard]] const rtc::tsid::ControlReference& Reference() const noexcept { return ref_; }

  [[nodiscard]] const rtc::tsid::CommandOutput* LastSolve() const noexcept { return last_solve_; }

  [[nodiscard]] const pinocchio::Model& FullModel() const noexcept { return *full_model_; }

  [[nodiscard]] const pinocchio::Model& ReducedModel() const noexcept { return *reduced_model_; }

  /// A frame of the full model, from the last tick's forward kinematics.
  [[nodiscard]] std::optional<pinocchio::SE3> FullFramePlacement(std::string_view frame) const;

  [[nodiscard]] const locomotion::DeviceIndices& Indices() const noexcept { return indices_; }

  /// True when this configuration runs the MPC (mpc.enabled).
  [[nodiscard]] bool MpcEnabled() const noexcept { return mpc_enabled_; }

  /// The MPC thread, or null before the first activation that spawned it.
  [[nodiscard]] const rtc::mpc::MPCThread* MpcThread() const noexcept { return mpc_thread_.get(); }

  [[nodiscard]] const rtc::mpc::MPCSolutionManager& MpcManager() const noexcept {
    return mpc_manager_;
  }

  /// The last tick's sample of the plan (`valid` false when none was usable).
  [[nodiscard]] const rtc::mpc::WbcReference& LastPlanSample() const noexcept { return wbc_ref_; }

  [[nodiscard]] ReferenceSource Source() const noexcept {
    return fault_latched_tick_ ? ReferenceSource::kFault : ref_source_;
  }

 protected:
  void OnDeviceConfigsSet() override;

 private:
  friend struct LocomotionControllerTestAccess;

  struct ParsedLogEntry {
    std::string msg_type;
    std::string instance;
  };

  void InitModels();
  [[nodiscard]] std::string WireDevices();
  void ResetLogState() noexcept;

  // The MPC (mpc_reference.cpp). Off the RT thread: BuildMpc (Pass 1),
  // SpawnMpcThread (first activation; a null handler = the factory's),
  // PauseMpc, StopMpcThread, LogMpcTick (the 1 Hz aux timer).
  void BuildMpc(const YAML::Node& cfg);
  [[nodiscard]] bool SpawnMpcThread(std::unique_ptr<rtc::mpc::MPCHandlerBase> handler) noexcept;
  void StopMpcThread() noexcept;
  void PauseMpc() noexcept;  ///< sampling off, the thread paused (deactivation)
  void LogMpcTick() noexcept;
  // On the RT thread, in tick order.
  void WriteMpcState() noexcept;
  void FollowPlan(const rtc::ControllerState& state) noexcept;
  [[nodiscard]] std::uint8_t Plausible(const rtc::mpc::WbcReference& r) const noexcept;
  void CopyPlan() noexcept;
  void ApplyContactMask(std::uint32_t mask) noexcept;
  void LosePlan(const rtc::ControllerState& state) noexcept;
  /// The lowest and the highest contact frame z, from the tick's kinematics.
  [[nodiscard]] std::pair<double, double> ContactHeights() const noexcept;
  void RestartMpcStream() noexcept;

  // Compute() pieces, in tick order.
  void ServiceRequests() noexcept;
  void ReadState(const rtc::ControllerState& state, int64_t now_ns) noexcept;
  void LatchReference(const rtc::ControllerState& state) noexcept;
  void UpdateModels() noexcept;
  void Solve() noexcept;
  void Latch(FaultCause cause) noexcept;
  void FillOutput(const rtc::ControllerState& state, rtc::ControllerOutput& out) noexcept;
  void FillDamping(const rtc::ControllerState& state, rtc::DeviceOutput& body) noexcept;
  void FillHands(const rtc::ControllerState& state, rtc::ControllerOutput& out) noexcept;
  void FillLogs(const rtc::ControllerState& state, const rtc::ControllerOutput& out, int64_t now_ns,
                double tick_us) noexcept;
  void RestartReference() noexcept;  ///< re-latch on the next valid tick, QP reset
  void ClearLatch() noexcept;

  rclcpp::Logger logger_;

  // ── Configuration (Pass 1 / Pass 3; read-only once configured) ────────────
  LocomotionControllerConfig cfg_;
  bool cfg_loaded_{false};
  std::string config_error_{"the controller configuration has not been loaded"};
  std::vector<ParsedLogEntry> parsed_log_entries_;
  std::string urdf_path_;
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder_;
  std::shared_ptr<const pinocchio::Model> full_model_;     ///< body + hands (kinematics)
  std::shared_ptr<const pinocchio::Model> reduced_model_;  ///< hands locked (dynamics)
  std::vector<std::string> full_joint_order_;
  std::vector<std::string> body_joint_names_;
  std::vector<std::vector<std::string>> hand_joint_names_;
  locomotion::DeviceIndices indices_{};
  /// Reduced joint j's slot in the full model's joint order.
  std::vector<int> reduced_to_full_;
  std::vector<pinocchio::FrameIndex> contact_frames_full_;
  std::vector<std::string> contact_names_;
  std::size_t nb_{0};  ///< body joints
  std::array<double, kLocomotionMaxBodyJoints> kp_{};
  std::array<double, kLocomotionMaxBodyJoints> kd_{};
  std::array<double, kLocomotionMaxBodyJoints> damping_kd_{};
  std::array<double, kLocomotionMaxBodyJoints> tau_max_{};  ///< +inf when the roster has none
  double total_mass_{0.0};
  double weight_{0.0};  ///< total_mass_ · |g| [N]
  uint32_t model_epoch_{1};
  std::unique_ptr<locomotion::TruthStateSource> source_;

  // ── QP (built at Pass 1, ticked on the RT thread) ──────────────────────────
  rtc::tsid::RobotModelInfo info_;
  rtc::tsid::PinocchioCache cache_;
  rtc::tsid::LocomotionFormulationBuilder qp_;
  rtc::tsid::ControlState qp_state_;
  rtc::tsid::ControlReference ref_;
  rtc::tsid::ContactState contacts_;
  std::unique_ptr<pinocchio::Data> full_data_;
  Eigen::VectorXd q_full_;

  // ── RT tick state ──────────────────────────────────────────────────────────
  locomotion::MeasuredRobotState measured_{};
  const rtc::tsid::CommandOutput* last_solve_{nullptr};
  bool source_ok_{false};
  bool reference_latched_{false};
  bool fault_latched_tick_{false};
  std::atomic<FaultCause> fault_cause_{FaultCause::kNone};  ///< read by on_activate
  Mode mode_{Mode::kWaiting};
  uint32_t source_bad_ticks_{0};
  uint32_t fault_count_{0};
  uint64_t tick_count_{0};
  bool estop_active_{false};
  uint32_t serviced_generation_{0};
  bool activation_seen_{false};
  uint32_t serviced_reset_epoch_{0};
  uint32_t serviced_estop_epoch_{0};
  std::array<double, kLocomotionMaxBodyJoints> tau_{};       ///< this tick's feedforward
  std::array<double, kLocomotionMaxBodyJoints> tau_prev_{};  ///< the last valid QP torque
  /// The hand joints' hold targets, by full-model joint index; bit h = hand h is held.
  std::array<double, locomotion::kMaxStateJoints> hand_hold_{};
  uint32_t hand_hold_mask_{0};
  LocomotionDiagLogPod diag_{};
  JointFeedforwardLogPod joint_log_{};

  // ── Cross-thread ───────────────────────────────────────────────────────────
  std::atomic<bool> estop_requested_{false};
  std::atomic<uint32_t> estop_epoch_{0};
  std::atomic<uint32_t> fault_reset_epoch_{0};
  std::atomic<bool> fault_latched_{false};

  // ── Topics, logs ───────────────────────────────────────────────────────────
  ControllerTopicHandles owned_topics_;
  rtc::ControllerLogSet log_set_{"locomotion_controller"};
  std::vector<rtc::LogHandle<DeviceStateLogPod>> state_log_handles_;
  rtc::LogHandle<JointFeedforwardLogPod> joint_log_handle_;
  rtc::LogHandle<LocomotionDiagLogPod> diag_log_handle_;
  rclcpp::CallbackGroup::SharedPtr log_drain_cb_group_;
  rclcpp::TimerBase::SharedPtr log_drain_timer_;
  std::uint64_t log_drops_reported_{0};

  // ── MPC (built at Pass 1 when mpc.enabled; the thread at the first activation) ──
  bool mpc_enabled_{false};
  bool layout_profile_drops_mpc_{false};
  bool mpc_spawn_refused_{false};
  rtc::MpcWorkerSlots mpc_worker_slots_{};
  int mpc_horizon_nodes_{0};
  double mpc_dt_{0.0};
  std::unique_ptr<rtc::mpc::RobotModelHandler> mpc_model_;
  /// Held until the thread takes it; `phase_` stays a borrowed pointer to it.
  std::unique_ptr<rtc::mpc::LocomotionPhaseManager> phase_owned_;
  rtc::mpc::LocomotionPhaseManager* phase_{nullptr};
  rtc::mpc::CommandFilter command_filter_;
  rtc::mpc::MPCSolutionManager mpc_manager_;
  /// The hand joints' hold targets under the MPC (q_default), by full-model joint index.
  std::array<double, locomotion::kMaxStateJoints> q_default_full_{};
  std::array<double, kLocomotionMaxBodyJoints> joint_lo_{};  ///< reduced model, body order
  std::array<double, kLocomotionMaxBodyJoints> joint_hi_{};
  std::uint32_t all_contacts_mask_{0};
  std::int64_t tick_dt_ns_{0};
  // RT tick state.
  rtc::mpc::WbcReference wbc_ref_{};
  ReferenceSource ref_source_{
      ReferenceSource::kIdle};  ///< idle / stand / walk; the latch reads as fault
  std::uint64_t mpc_t_ns_{0};   ///< this tick on the MPC's clock (iteration · dt)
  std::uint32_t mpc_state_epoch_{0};
  std::uint32_t contact_mask_{0};
  std::uint64_t rejected_plan_id_{0};
  std::atomic<std::uint64_t> plausibility_rejects_{0};  ///< written by the RT tick only
  std::atomic<std::uint8_t> last_reject_check_{0};      ///< the check the last refusal failed
  double u_fb_norm_{0.0};
  // The 1 Hz aux timer: the MPC tick-timing CSV and the counters.
  rtc::mpc::MpcTimingLogger mpc_timing_logger_;
  rclcpp::CallbackGroup::SharedPtr mpc_timing_cb_group_;
  rclcpp::TimerBase::SharedPtr mpc_timing_timer_;
  std::uint64_t mpc_timing_ticks_{0};
  std::uint64_t mpc_failed_reported_{0};
  /// Declared last: destroyed first, before everything it borrows (the
  /// destructor stops it before any member goes).
  std::unique_ptr<rtc::mpc::HandlerMPCThread> mpc_thread_;
};

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_CONTROLLER_HPP_
