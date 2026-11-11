#ifndef INTEGRATED_BRINGUP_CONTROLLERS_HAND_JOINT_CONTROLLER_HPP_
#define INTEGRATED_BRINGUP_CONTROLLERS_HAND_JOINT_CONTROLLER_HPP_

// ── HandJointController: one hand, joint goals + model feedforward torque ───
//
// The binding for a robot that is ONE position-servoed device group — a hand
// on its own — whose servo (firmware PD, simulator <position> actuator) is the
// device's and whose controller contributes a joint position reference plus a
// per-joint feedforward torque computed from the hand's dynamic model. Every
// tick it sends `pd_feedforward`: `commands` is a joint-space quintic
// reference toward the last `<group>/joint_goal`, `feedforward` is
//
//     τ_ff = clamp(k_g · g(q_meas) + k_M · M(q_meas) · q̈_ref, ±τ_max)
//                                     (rtc_controllers joint/dynamics_feedforward_law.hpp)
//
// with the Coriolis slot of that law left at 0 (the term is a later addition;
// the law already carries it). q̈_ref is the quintic's acceleration — 0 off a
// running trajectory, so the inertia term acts only while a goal is being
// followed and a held pose sees gravity alone. k_g is `dynamics_feedforward.
// gravity_gain`, k_M `dynamics_feedforward.inertia_gain` (absent = 0; the
// robot's YAML sets it). The model inertia is
// the links' only (a URDF carries no rotor inertia), and with a servo whose D
// term acts on the measured velocity the tracking error in motion is mostly
// the servo's viscous lag, so the term's effect there is small (the commit
// that added the term records the sim measurement). The purpose of
// the lane is to make the device's PD hold a pose with no sag at LOW gains —
// the feedforward carries the weight, the servo only the error — and to be
// the one place that reference and feedforward are tuned, in the simulator
// and on the hardware.
//
// Model:
//   - the group's `urdf.tree_models.<group>` is the hand (root = the link the
//     fingertip poses are expressed in; tips = the fingertip frames);
//   - on an EXTENDED URDF (loop closures) the gravity term is the reduced
//     g_a = Gᵀ g of rtc_urdf_bridge::RtClosedChainHandle, and the inertia the
//     reduced M_a = Gᵀ M G, both evaluated at the measured actuated positions
//     (UpdateDynamics computes them together) — the same projection also serves the
//     fingertip FK (ClosedChainHandFk in borrowed mode, fed this controller's
//     handle every tick) so one projection per tick serves both. The handle's
//     independent joints must ALL be joints of this device group, in any
//     order (the bridge is by name); a loop whose independent joint lives on
//     another group cannot be projected from this one and refuses configure;
//   - on a plain URDF the tree model is the model: g (and, while k_M ≠ 0 and
//     a goal is being followed, M) from the serial handle, FK from the same
//     handle, the device's joint order installed on it. A
//     device that does not list every joint of the tree exactly once is
//     refused (a loop-passive joint left in a plain tree is the usual cause).
//
// Hold policy of the feedforward (the device PD has no such choice to make):
//   - before the first tick with a valid model term: 0 N·m;
//   - a held / singular projection or an unreadable device: the model terms
//     (g and M) keep their LAST valid values (a walk-in after activation is a
//     run of held ticks — see RtClosedChainHandle::Status — and zeroing the
//     weight for them would drop the hand on every activation). q̈_ref is
//     always the reference's own: it follows a running trajectory through a
//     held projection, and it is 0 on an unreadable tick, where the reference
//     is frozen — a frozen reference is at rest. The gains are this tick's, so
//     a gain changed during an outage takes effect on it;
//   - a non-finite term from the law: 0 N·m on every joint (all-or-nothing,
//     the law's rule);
//   - `dynamics_feedforward.enabled: false` (YAML or runtime parameter): 0 N·m,
//     the
//     command type stays pd_feedforward. This is the A/B switch the sim gate
//     uses — switching the COMMAND TYPE instead would let the simulator
//     re-enable its own gravity compensation and change the plant.
//
// Modes (`mode`, YAML key and runtime parameter; the device's PD gains are
// not this controller's to change, so a mode only chooses what the two lanes
// carry):
//   - `tracking` (default): the two lanes above — joint goals are followed;
//   - `damping`: `commands` = the measured positions every tick and
//     `feedforward` = 0, so only the device's own derivative gain acts (a
//     "damping mode" in the sense of the Unitree / MAB motor firmwares: kp
//     effectively 0, kd as the device has it);
//   - `damping_feedforward`: `commands` = the measured positions, the
//     feedforward lane stays as in `tracking` (dynamics_feedforward.*: switch,
//     gains, clamp, hold policy) — the lane under test, with no servo error under it.
//     There is no trajectory here, so q̈_ref = 0 and the inertia term is 0.
//   A goal that lands mid-motion restarts the quintic from rest (the
//   reference velocity steps to 0 for that tick); the inertia term follows the
//   new quintic and does not feed the step forward — a torque impulse would be
//   the wrong answer to it, a velocity-continuous retarget the right one.
//   A joint goal while the mode is not `tracking` is drained and refused
//   (GoalRejectCount) — unlike E-STOP, which discards pending goals silently.
//   On the tick the mode returns to `tracking` the reference is re-seeded from
//   the measurement (goal = "here"); leaving `tracking` drops a running
//   trajectory. E-STOP dominates every mode.
//
// E-STOP: the measured positions are commanded and the feedforward is 0; on
// release the reference is re-seeded from the measurement and the goal
// becomes "where the hand is now". An unreadable device holds the last
// command and the last torque, and freezes the trajectory clock.

#include "integrated_bringup/logging/device_state_log_pod.hpp"
#include "integrated_bringup/logging/joint_feedforward_log_pod.hpp"
#include "integrated_bringup/support/closed_chain_hand_fk.hpp"
#include "integrated_bringup/support/owned_topics.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controller_interface/controller_log_set.hpp"
#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/params/dynamics_feedforward_params.hpp"
#include "rtc_controllers/trajectory/joint_space_trajectory.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_closed_chain_handle.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

// Pinocchio headers (warnings suppressed)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/timer.hpp>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace integrated_bringup {

/// Capacity of the fixed per-joint buffers; a device group wider than this is
/// refused at configure. Not a joint count of any hand.
inline constexpr int kHandJointMaxDof = static_cast<int>(JointFeedforwardLogPod::kMaxJoints);
/// Fingertip frames this controller publishes (TF + ControllerOutput
/// task_link_poses): the tree model's tip_links, capped here.
inline constexpr std::size_t kHandJointMaxTips = std::min<std::size_t>(
    ClosedChainHandFk::kMaxFingertips, static_cast<std::size_t>(rtc::kMaxTaskLinks));
/// Floor of the trajectory speed (NUM-4): a speed divides a distance.
inline constexpr double kHandJointMinSpeed = 1e-6;

/// What the two lanes carry (header banner "Modes"). The integer values are
/// the `mode` column of joint_ff.csv — append only.
enum class HandJointMode : std::uint8_t {
  kTracking = 0,            ///< reference follows joint goals, feedforward lane on
  kDamping = 1,             ///< reference = measurement, feedforward 0
  kDampingFeedforward = 2,  ///< reference = measurement, feedforward lane on
};

/// The YAML / parameter spelling of each mode, indexed by the enum value. The
/// one place the words live (demo_controller_gui pins its list to these).
inline constexpr std::array<std::string_view, 3> kHandJointModeNames = {"tracking", "damping",
                                                                        "damping_feedforward"};

/// Spelling → mode. Non-RT: throws std::runtime_error naming the three
/// accepted values on anything else.
[[nodiscard]] HandJointMode ParseHandJointMode(std::string_view text);
[[nodiscard]] std::string_view HandJointModeName(HandJointMode mode) noexcept;

/// The controller's YAML, parsed and range-checked. Pure data.
struct HandJointConfig {
  double trajectory_speed{1.0};                  ///< [rad/s], > 0
  double max_traj_velocity{2.0};                 ///< [rad/s], ≥ trajectory_speed
  rtc::params::DynamicsFeedforwardParams ff;     ///< the `dynamics_feedforward` block
  HandJointMode mode{HandJointMode::kTracking};  ///< `mode` key, default tracking
  /// ‖φ‖ above which a closed-chain projection is not trusted for the
  /// fingertip FK (ClosedChainHandFk). The torque hold is the handle's own
  /// `held` / `singular`, not this.
  double closure_error_threshold{1e-3};
};

/// The block key of the feedforward lane in the controller YAML and in the
/// runtime parameters (`dynamics_feedforward.enabled`,
/// `dynamics_feedforward.gravity_gain`, `dynamics_feedforward.inertia_gain`):
/// every term of the law this controller ships, plus the switch and the bound.
/// The demo GUI builds its Gains rows from this literal (its test reads it).
inline constexpr std::string_view kHandJointFfBlockKey = "dynamics_feedforward";

/// Parse and range-check the controller's YAML node. Throws std::runtime_error
/// naming the key. `command_type` must be `pd_feedforward`: this controller
/// exists for that lane, and a position command would silently drop the
/// torque it computes.
[[nodiscard]] HandJointConfig ParseHandJointConfig(const YAML::Node& cfg);

class HandJointController final : public rtc::RTControllerInterface {
 public:
  /// Runtime-tunable values, as the tick reads them (trivially copyable).
  /// `mode` is the enum, never the string: the parse lives in the parameter
  /// layer and the tick only compares.
  struct Gains {
    double trajectory_speed{1.0};
    double max_traj_velocity{2.0};
    bool ff_enabled{true};
    /// k_g of the gravity term k_g · g(q_meas).
    double gravity_gain{1.0};
    HandJointMode mode{HandJointMode::kTracking};
    /// k_M of the inertia term k_M · M(q_meas) · q̈_ref. 0 = gravity alone.
    double inertia_gain{0.0};
  };

  static_assert(std::is_trivially_copyable_v<Gains>, "Gains rides a SeqLock");

  explicit HandJointController(std::string_view urdf_path);
  ~HandJointController() override;

  [[nodiscard]] rtc::ControllerOutput Compute(const rtc::ControllerState& state) noexcept override;

  /// The group's joint goal: exactly the group's joints (the base lets a short
  /// positional goal through; here it would move some joints and hold the
  /// rest at the old reference). Counted when refused.
  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override;

  /// Joint goals only — a task goal on the group topic is refused by the base
  /// before it reaches SetDeviceTarget.
  [[nodiscard]] GoalKinds AcceptedGoalKinds(int /*device_idx*/) const noexcept override {
    return {.joint = true, .task = false};
  }

  [[nodiscard]] std::string_view Name() const noexcept override { return "HandJointController"; }

  [[nodiscard]] rtc::CommandType GetCommandType() const noexcept override {
    return rtc::CommandType::kPdFeedforward;
  }

  void TriggerEstop() noexcept override;
  void ClearEstop() noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;

  void LoadConfig(const YAML::Node& cfg) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state,
                              rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                              const YAML::Node& yaml_cfg) noexcept override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) noexcept override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) noexcept override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) noexcept override;

  void PublishNonRtSnapshot(const rtc::PublishSnapshot& snap) noexcept override;

  // ── Read-only views (tests, diagnostics) ─────────────────────────────────

  /// Why the model could not be wired to the device, or empty. on_configure
  /// refuses on a non-empty string (OnDeviceConfigsSet cannot refuse itself).
  [[nodiscard]] const std::string& ConfigError() const noexcept { return config_error_; }

  /// The record of the last Compute(). RT-owned: read it from the thread that
  /// calls Compute().
  [[nodiscard]] const JointFeedforwardLogPod& LastTick() const noexcept { return tick_; }

  /// True when the gravity term comes from the closed-chain projection.
  [[nodiscard]] bool ClosedChainActive() const noexcept { return closed_handle_ != nullptr; }

  /// Closed-chain status of the last projection (defaults on a plain model).
  [[nodiscard]] rtc_urdf_bridge::RtClosedChainHandle::Status LastProjectionStatus() const noexcept {
    return kin_status_;
  }

  [[nodiscard]] int Dof() const noexcept { return dof_; }

  [[nodiscard]] std::uint32_t GoalRejectCount() const noexcept {
    return goal_rejects_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] Gains GetGains() const noexcept { return gains_lock_.Load(); }

  void SetGains(const Gains& gains) noexcept { gains_lock_.Store(gains); }

  /// The per-joint torque bound in force (min of dynamics_feedforward.max_abs and the
  /// device rating), device order, [0, Dof()).
  [[nodiscard]] std::span<const double> TorqueBounds() const noexcept {
    return {ff_bound_.data(), static_cast<std::size_t>(std::max(dof_, 0))};
  }

  /// M(q)·q̈_ref of the last Compute() (before k_M and the clamp), device order,
  /// [0, Dof()) [N·m]. 0 on every tick the lane did not use the term or the
  /// reference was at rest. RT-owned.
  [[nodiscard]] std::span<const double> LastInertiaTerm() const noexcept {
    return {tau_m_.data(), static_cast<std::size_t>(std::max(dof_, 0))};
  }

  /// The reference acceleration of the last Compute(), device order [rad/s²].
  [[nodiscard]] std::span<const double> LastReferenceAcceleration() const noexcept {
    return {qdd_ref_.data(), static_cast<std::size_t>(std::max(dof_, 0))};
  }

 protected:
  void OnDeviceConfigsSet() override;
  void ApplyPendingTarget(int device_idx, std::span<const double> values,
                          bool is_task) noexcept override;
  void ResetTargetInitialization() noexcept override;

 private:
  friend struct HandJointTestAccess;

  struct ParsedLogEntry {
    std::string msg_type;
    std::string instance;
  };

  // ── Non-RT setup ─────────────────────────────────────────────────────────
  /// Build the model handles from the system model config (LoadConfig).
  void InitModel();
  /// Tie the model to the device (OnDeviceConfigsSet): joint order / q_a
  /// bridge, torque bounds, limits, fingertip frames.
  [[nodiscard]] std::string WireDevice();
  void DeclareGainParameters() noexcept;
  void DeclareGainParametersImpl();
  [[nodiscard]] rcl_interfaces::msg::SetParametersResult OnGainParametersSet(
      const std::vector<rclcpp::Parameter>& params) noexcept;
  void ResetLogState() noexcept;

  // ── RT tick steps ────────────────────────────────────────────────────────
  void EvaluateModel(const rtc::ControllerState& state) noexcept;
  void AdvanceReference(const rtc::ControllerState& state, double dt) noexcept;
  /// Reference = the measurement, velocity 0, goal = "here", trajectory off.
  /// E-STOP and the two non-tracking modes. Counts as a seed (the output is
  /// valid from this tick).
  void HoldAtMeasurement(const rtc::ControllerState& state) noexcept;
  void ComputeFeedforward() noexcept;
  /// tau_m_ = M · q̈_ref in device order, from the handle's own matrix (the
  /// closed-chain handle keeps its last valid M_a through a held projection;
  /// the plain handle's M is this tick's).
  void ComputeInertiaTerm(std::size_t n) noexcept;
  void FillOutput(const rtc::ControllerState& state, rtc::ControllerOutput& out) noexcept;
  void FillTickRecord(const rtc::ControllerState& state) noexcept;
  void PushLogs(const rtc::ControllerState& state, const rtc::ControllerOutput& out) noexcept;

  // ── Configuration ────────────────────────────────────────────────────────
  rclcpp::Logger logger_;
  rclcpp::Clock log_clock_{RCL_STEADY_TIME};
  HandJointConfig cfg_;
  bool cfg_loaded_{false};
  std::string config_error_{"the controller configuration has not been loaded"};
  std::vector<ParsedLogEntry> parsed_log_entries_;
  std::string urdf_path_;

  // ── Model ────────────────────────────────────────────────────────────────
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder_;
  /// The group's tree model: FK + gravity on a plain URDF, frame names on both.
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> hand_handle_;
  /// Owned closed-chain projection (extended URDF only).
  std::unique_ptr<rtc_urdf_bridge::RtClosedChainHandle> closed_handle_;
  ClosedChainHandFk closed_fk_;
  std::string root_link_;
  std::vector<std::string> tip_links_;
  std::size_t num_tips_{0};
  std::array<pinocchio::FrameIndex, kHandJointMaxTips> tip_fid_{};  ///< plain model
  pinocchio::FrameIndex root_fid_{0};                               ///< plain model
  std::vector<std::string> joint_names_;
  int dof_{0};
  /// Closed chain: independent joint k ← device channel bridge_[k].
  std::vector<int> bridge_;
  Eigen::VectorXd q_a_;    ///< n_a, preallocated
  Eigen::VectorXd q_dev_;  ///< dof_, preallocated (plain model input)
  std::array<double, kHandJointMaxDof> q_min_{};
  std::array<double, kHandJointMaxDof> q_max_{};
  /// Per-joint peak velocity of a goal trajectory [rad/s]: min(device rating, YAML cap).
  std::array<double, kHandJointMaxDof> v_max_{};
  std::array<double, kHandJointMaxDof> ff_bound_{};

  // ── RT state ─────────────────────────────────────────────────────────────
  bool estop_active_{false};
  bool readable_{false};
  bool seeded_{false};
  /// Set by ResetTargetInitialization (lifecycle thread) and by the tick
  /// (E-STOP release, return to tracking); consumed by the tick. Atomic for
  /// the first writer, relaxed: it is a flag, not a hand-off of data.
  std::atomic<bool> need_reseed_{true};
  /// The mode the previous tick ran in: the edge detector of the re-seed.
  HandJointMode mode_prev_{HandJointMode::kTracking};
  std::array<double, kHandJointMaxDof> q_ref_{};
  std::array<double, kHandJointMaxDof> qd_ref_{};
  std::array<double, kHandJointMaxDof> goal_{};
  rtc::trajectory::JointSpaceTrajectory<kHandJointMaxDof> traj_;
  double traj_time_{0.0};
  bool traj_active_{false};
  /// Last valid model gravity term, device order; zero until the first valid tick.
  std::array<double, kHandJointMaxDof> g_dev_{};
  bool have_g_{false};
  bool model_fresh_{false};
  /// The handle holds a mass matrix the inertia term may use. Closed chain:
  /// set with g_a (the handle keeps M_a through a hold). Plain: M was computed
  /// THIS tick — only while k_M ≠ 0 and a goal is being followed, so a matrix
  /// from another pose is never read.
  bool have_m_{false};
  /// Reference acceleration [rad/s²], device order; 0 off a running trajectory.
  std::array<double, kHandJointMaxDof> qdd_ref_{};
  /// M · q̈_ref, device order [N·m], before k_M (LastInertiaTerm).
  std::array<double, kHandJointMaxDof> tau_m_{};
  /// The k_M the law applied this tick (0 when the term was off or had no M).
  double inertia_gain_applied_{0.0};
  /// Model-order scratch of the inertia product (closed: n_a, plain: nv),
  /// sized in WireDevice.
  Eigen::VectorXd qdd_model_;
  Eigen::VectorXd tau_model_;
  std::array<double, kHandJointMaxDof> tau_ff_{};
  bool ff_finite_{false};
  std::uint64_t clamped_mask_{0};
  rtc_urdf_bridge::RtClosedChainHandle::Status kin_status_{};
  Gains gains_tick_{};
  JointFeedforwardLogPod tick_{};

  // ── Cross-thread ─────────────────────────────────────────────────────────
  /// One logical writer: LoadConfig, DeclareGainParametersImpl, the parameter
  /// callback and SetGains all run on the lifecycle / executor side, never
  /// concurrently with each other (rclcpp serializes parameter sets); the tick
  /// only Loads.
  rtc::SeqLock<Gains> gains_lock_;
  std::atomic<bool> estopped_{false};
  std::atomic<std::uint32_t> goal_rejects_{0};

  // ── Topics, logs, parameters ─────────────────────────────────────────────
  ControllerTopicHandles owned_topics_;
  rtc::ControllerLogSet log_set_{"hand_joint_controller"};
  rtc::LogHandle<DeviceStateLogPod> state_log_handle_;
  rtc::LogHandle<JointFeedforwardLogPod> ff_log_handle_;
  rclcpp::CallbackGroup::SharedPtr log_drain_cb_group_;
  rclcpp::TimerBase::SharedPtr log_drain_timer_;
  std::uint64_t log_drops_reported_{0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_CONTROLLERS_HAND_JOINT_CONTROLLER_HPP_
