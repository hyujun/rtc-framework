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
// The reference is fixed for now: the measured pose at the first valid tick,
// zero velocity and acceleration, and the static contact wrench of that pose
// (two contacts: the weight split and the centres of pressure placed so that
// they balance gravity at the CoM; otherwise an equal share of the weight). A planner replaces it later through the same ControlReference.
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
// the state's model epoch is not the controller's.
//
// Clearing the fault: ResetFault() (the CM's reset service) — the next tick
// resets the QP, re-latches the reference from the measurement and returns to
// nominal; if the cause is still there it latches again on that tick, which
// HasLatchedFault() reports. Unlike the controllers whose fault outlives a
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
// The other sections of the file (command, gait, swing, terrain, reference,
// mpc) belong to the planner and are not read here.
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
#include <vector>

namespace integrated_bringup {

/// Body joints the controller drives (the joint log's capacity).
inline constexpr std::size_t kLocomotionMaxBodyJoints = LocomotionDiagLogPod::kMaxJoints;

/// @brief The controller's own sections of its YAML (Pass 1, schema only).
struct LocomotionControllerConfig {
  locomotion::HumanoidModelConfig model;
  locomotion::EstimatorConfig estimator;
  std::vector<double> pd_kp;  ///< body roster order; length checked against the roster
  std::vector<double> pd_kd;
  int max_source_bad_ticks{1};
  std::vector<double> damping_kd;
  YAML::Node whole_body_qp;
};

/// @throws std::invalid_argument naming the key on a missing / unknown key of
///         `pd_feedforward` or `fault`, a gain that is negative or not finite, a
///         `command_type` other than pd_feedforward, or a model / estimator section
///         its own parser refuses.
[[nodiscard]] LocomotionControllerConfig ParseLocomotionControllerConfig(const YAML::Node& cfg);

class LocomotionController final : public rtc::RTControllerInterface {
 public:
  using Mode = LocomotionDiagLogPod::Mode;
  using FaultCause = LocomotionDiagLogPod::FaultCause;

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
  void FillLogs(const rtc::ControllerState& state, const rtc::ControllerOutput& out,
                double tick_us) noexcept;

  rclcpp::Logger logger_;
  rclcpp::Clock log_clock_{RCL_STEADY_TIME};

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
  FaultCause fault_cause_{FaultCause::kNone};
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
  /// The hand joints' hold targets, by full-model joint index.
  std::array<double, locomotion::kMaxStateJoints> hand_hold_{};
  bool hand_hold_set_{false};
  LocomotionDiagLogPod diag_{};
  JointFeedforwardLogPod joint_log_{};

  // ── Cross-thread ───────────────────────────────────────────────────────────
  std::atomic<bool> estop_requested_{false};
  std::atomic<uint32_t> estop_epoch_{0};
  std::atomic<uint32_t> fault_reset_epoch_{0};
  std::atomic<bool> fault_latched_{false};
  std::atomic<bool> dropped_on_activate_{false};

  // ── Topics, logs ───────────────────────────────────────────────────────────
  ControllerTopicHandles owned_topics_;
  rtc::ControllerLogSet log_set_{"locomotion_controller"};
  std::vector<rtc::LogHandle<DeviceStateLogPod>> state_log_handles_;
  rtc::LogHandle<JointFeedforwardLogPod> joint_log_handle_;
  rtc::LogHandle<LocomotionDiagLogPod> diag_log_handle_;
  rclcpp::CallbackGroup::SharedPtr log_drain_cb_group_;
  rclcpp::TimerBase::SharedPtr log_drain_timer_;
  std::uint64_t log_drops_reported_{0};
};

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_CONTROLLER_HPP_
