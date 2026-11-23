// ── locomotion_controller — the humanoid whole-body controller ────────────────
//
// One tick: the measured state (MeasuredStateSource) → the full model's
// kinematics (every end-effector and contact frame, hands at their measured
// pose) → the reduced model's dynamics (hands locked at neutral: the hands are
// held still and their mass is folded into the arm ends) → the whole-body QP
// (rtc_tsid LocomotionFormulationBuilder) → pd_feedforward on the body group:
// a position target plus the QP's torque as feedforward, with the controller's
// own non-dominant PD gains on the wire. Hand groups get a kPosition hold at the
// pose they had on their first readable tick after the last restart (an
// activation, a reset, either E-STOP edge) — before the body's reference is
// latched, so a hand never follows a push while the body waits. Under the MPC,
// whose model locks the hands at reference.q_default, that hold is q_default
// from the same first tick (the hands go there while the body may still be
// waiting: the dynamics model has them nowhere else).
//
// The reference (LocomotionDiagLogPod::ReferenceSource):
//   idle   the latched pose: the measured pose at the first valid tick the
//          pose guard passes (below), zero velocity and acceleration, and the
//          static contact wrench of that pose (two contacts: the weight split
//          by the lever rule along the line between them — half each when
//          they are closer than 5 cm — and the centres of pressure placed,
//          within ±0.1 m of each contact's origin, so that they balance
//          gravity at the CoM; otherwise an equal share of the weight).
//          Without an MPC (`mpc:` absent or mpc.enabled false) this is the
//          only reference.
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
// measurement if the pose guard passes it WITH the stance checks — every
// contact frame within 15 mm of the lowest, the base slower than 0.1 m/s, the
// tilt below the limit — otherwise the fault latches at once (kPlanLoss):
// re-latching a swing foot would load it, and there is no time to wait.
// A plan the plausibility check refuses stays refused for every later sample.
// In stand / walk, a refused plan gives way to the plan followed before it,
// if that one is still held (its epochs, younger than mpc.max_policy_age_s)
// and passes the check on this tick; it is then followed as before, until a
// plausible plan replaces it or it is lost — the rule for a plan that
// arrives late. Idle → stand does not take it (idle follows nothing), and a
// held plan is not held again past a second refusal. Each hold is counted
// (plan_holds).
//
// The pose guard (locomotion/pose_guard.hpp): every latch of the reference —
// at an activation, after a fault reset, after an E-STOP clear, on a lost plan
// — first asks whether the measured pose is one to stand on: the base lane
// holds a real sample, the base quaternion, velocity and contact heights are
// finite, the quaternion is unit, and the body's tilt from vertical is at most
// fault.pose_guard.base_tilt_rad (R(2,2) ≥ cos of it). Pass → the reference
// latches on that tick. Fail → the controller stays in `waiting` (damping),
// asks again on every valid tick, and after 0.5 s of refusals (a code
// constant, in ticks of the control rate; the count restarts at every entry,
// so time spent under the E-STOP is not counted) latches the fault
// (kPoseGuard) — a robot that is down is held, not stood up. The verdict is
// logged as `pose_guard_mask` (0 = pass). A tick without a valid state is not
// a verdict: `waiting` stays, without the clock (ActivationWithTheCauseStill-
// ThereWaitsOnDamping). The guard does not know the CM's post-clear policy:
// after a clear it runs whether or not the CM then switches away.
//
// Modes (LocomotionDiagLogPod::Mode):
//   waiting   no latched reference — before the first valid state, under the
//             E-STOP, or while the pose guard refuses the pose: damping on the
//             raw body lane, nothing solved, no state to the MPC
//   nominal   the QP's torque
//   degraded  the QP missed this tick — the last valid torque is held (zero
//             until the first solve); the solve health classifies the run
//   fault     latched — damping only: feedforward −Kd_fault·v (clamped to the
//             joint torque limit), position target = the measured q, the PD
//             gains on the wire unchanged. The robot settles slowly under
//             gravity; that is the intent (no large torque from a bad state).
// Fault causes: the QP's health reached Fault; the source was not ok for more
// than fault.max_source_bad_ticks ticks in a row while a reference was
// latched (an unlatched controller only waits); the source restarted its
// state (a new state epoch) while a reference was latched; an estimating
// source had not started (any status) for estimator.inekf.init.timeout_s of
// unlatched ticks outside the E-STOP, counted since the last restart of the
// reference (not necessarily in a row); the output was not finite;
// the state's model epoch is not the controller's; the plan was lost off a
// still double support; the pose guard refused the pose for the settle time.
//
// The E-STOP (TriggerEstop / ClearEstop, the CM's hooks on another thread):
// while it holds, the tick still reads the state, updates the models and
// writes its logs and status row, but latches no reference, solves nothing,
// samples no plan and writes no state to the MPC — the mode is `waiting` and
// the output damping (the CM substitutes its own hold anyway). Either edge
// restarts the reference (the robot was not following this controller in
// between), so the first valid tick after a clear goes through the pose guard
// like an activation. A fault latched before the E-STOP stays latched through
// it; a reset under the E-STOP releases it and the guard runs after the clear.
// The flag and the edge counter travel in ONE atomic word (bit 0 = active,
// the rest an edge count), so a tick reads both from the same load.
//
// Clearing the fault: ResetFault() (the CM's reset service) — the next tick
// resets the QP, re-latches the reference from the measurement (through the
// pose guard) and returns to nominal; if the cause is still there it latches
// again, which HasLatchedFault() reports. A reset, an activation and either
// E-STOP edge also restart the MPC's state stream (a new state epoch: no plan
// of the old one is sampled again) and its gait schedule. Unlike the
// controllers whose fault outlives a deactivate / activate cycle, ACTIVATING
// this controller also clears it: it is activated by an operator's switch from
// another controller that has been holding the robot, and a latch from an
// earlier session describes a robot state that switch has already replaced.
// The first tick re-latches as above and logs once that a latched fault was
// dropped.
//
// The state source (estimator.source, fixed at configure): truth copies the
// simulator's lanes, inekf estimates the base (inekf_state_source.hpp) on the
// body group's model.imu.primary_site with estimator.inekf.imu_frame as its
// frame. Each tick the source is given the contact mask of the previous tick
// (all contacts after an activation or a restart of the MPC stream) as its
// prior. A change of the source's state epoch from one tick to the next — the
// estimate restarted — restarts the MPC's state stream (no plan made on the
// old state is sampled again) and, if a reference was latched, latches the
// fault (kSource) on that tick: the base it was following jumped. Before the
// latch the restart only waits for the source to start again. The activation's
// own Reset() of the source is not such a change.
//
// YAML (`locomotion_controller:`): model.* (humanoid_model_config.hpp),
// whole_body_qp.* (rtc_tsid; `health:` sets the degraded / fault rule),
// estimator.* (measured_state_source.hpp), and
//   command_type: pd_feedforward      # the only accepted value
//   pd_feedforward: {kp: [...], kd: [...]}   # body roster order, ≥ 0 [N·m/rad, N·m·s/rad]
//   fault:
//     max_source_bad_ticks: N (≥ 1)
//     damping_kd: [...]                # body roster order, ≥ 0
//     pose_guard: {base_tilt_rad: θ}   # (0, π/2]; every key required, no other key
//   topics: {<body group>: {...}, <hand group>: {...}}   # body first, then model.hand_groups
//   logs: [...]   # DeviceStateLog per group, JointFeedforwardLog (body), LocomotionDiagLog,
//                 # LocomotionTruthLog (the T tier — locomotion-controller.md "Log tiers")
//   mpc:          # optional; absent = no MPC (the idle reference only)
//     enabled: bool             # false = as if absent
//     period_nodes: N (≥ 1)     # the MPC thread's period, in OCP node intervals
//     max_policy_age_s: s (≥ 0) # a plan older than this is dropped (0 = the horizon only)
//     riccati: {enabled: bool, ...}   # rtc_mpc MPCSolutionManager
//     sanity: {joint_delta_rad, base_speed, wrench_scale}   # all > 0, below
//     ocp_type / solver / kinodynamic / max_stale_solutions   # rtc_mpc MPCFactory · manager
//     sync:         # optional; absent = free_run
//       mode: free_run | lockstep   # lockstep: the RT tick triggers the solve every
//                                   # N = period_nodes·dt/tick ticks (N integer) and waits
//                                   # for it L ticks later; the plan is taken at exactly
//                                   # L ticks of age. Sim only: configure is refused
//                                   # unless the CM declared cm_sim_time_sync true.
//       delivery_ticks: L           # 1..N; 0 = N (one period)
//       wait_timeout_s: s           # > 0; the tick's wait for a solve past this latches
//                                   # the fault mpc_wait
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
// the base speed at most sanity.base_speed [m/s]; no contact pulling by more
// than 2 % of the weight, and the total normal force between
// 1 / sanity.wrench_scale and sanity.wrench_scale × the robot's weight. The
// plan's accelerations are not judged; the logged check numbers 6 and 11 are
// retired (Plausible).
// The MPC thread runs on the `mpc` layout role: activation is refused under a
// launch profile that dropped it (enable_mpc:=false) while mpc.enabled is true.
//
// The velocity command (mpc.enabled, and `velocity_command: {topic}` present):
//   velocity_command: {topic: <name>}   # geometry_msgs/Twist; absent = no command path
// The external wrench lane (truth log only — nothing in the control law reads it):
//   external_wrench: {topic: <name>, max_age_ms: <ms>}   # absent or topic "" = no lane
// geometry_msgs/WrenchStamped, the wrench a simulator applies to the robot
// (rtc_mujoco_sim `external_wrench_report`). The subscription (depth 1, the
// node's default callback group) stores the message with the steady time it
// arrived; the tick copies the latest into the truth log's truth_ext_* block,
// fresh while younger than max_age_ms. Subscribed with or without the MPC.
//
// linear.x / linear.y / angular.z = v_x / v_y / yaw rate, linear.z = the base
// height RATE (rtc_mpc command_filter.hpp). The subscription (depth 1, the
// node's default callback group) only stores the message with the steady time
// it arrived; a wall timer at command.sample_rate_hz (a mutually exclusive
// group, on the CM's nrt_callback executor) steps the command filter with it
// (stale after command.t_stale, a non-finite axis keeps its last accepted
// value) and hands the output to the phase manager — the filter's one writer.
// Nothing stepped yet, or stale: the command decays to rest at
// command.base_height. The timer runs from activation to deactivation; an
// activation resets the filter (the height back to command.base_height). The
// key is accepted without mpc.enabled and does nothing (the plan is the
// command's only consumer), so a profile may keep it while the MPC is off.
// Each step also hands the output and the phase manager's gait to the RT tick
// through a wait-free triple buffer, for the diagnostic log only (the RT tick
// never reads the phase manager or a SeqLock the timer writes).
//
// The diagnostic log's measured contact force: contact c reads group c of the
// body group's inference lane (fx, fy, fz in values 1..3 of a group
// `sensor_layout.inference_values_per_group` wide), so the profile lists that
// lane's sources in the model's contact order. A lane narrower than 4 values,
// or with another group count than the model's contacts, logs NaN.
//
// The status topic `<body group>/locomotion_status` (diagnostic_msgs/
// DiagnosticStatus, published with the CM's non-RT snapshot, ≈ 50 Hz): level
// and values from locomotion/locomotion_status.hpp — the reference source, the
// mode, the fault cause, the plan (id, age, convergence, contact mask, epochs),
// the MPC thread's counters (as of the last 1 Hz read), the QP tick, and the
// command the filter last put out. Published with or without an MPC.
#ifndef INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_CONTROLLER_HPP_
#define INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_CONTROLLER_HPP_

#include "integrated_bringup/controllers/locomotion/humanoid_model_config.hpp"
#include "integrated_bringup/controllers/locomotion/inekf_state_source.hpp"
#include "integrated_bringup/controllers/locomotion/joy_command.hpp"
#include "integrated_bringup/controllers/locomotion/locomotion_status.hpp"
#include "integrated_bringup/controllers/locomotion/measured_state_source.hpp"
#include "integrated_bringup/controllers/locomotion/truth_log.hpp"
#include "integrated_bringup/logging/device_state_log_pod.hpp"
#include "integrated_bringup/logging/joint_feedforward_log_pod.hpp"
#include "integrated_bringup/logging/locomotion_diag_log_pod.hpp"
#include "integrated_bringup/logging/locomotion_truth_log_pod.hpp"
#include "integrated_bringup/support/owned_topics.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controller_interface/controller_log_set.hpp"
#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_mpc/comm/triple_buffer.hpp"
#include "rtc_mpc/handler/mpc_handler_base.hpp"
#include "rtc_mpc/logging/mpc_timing_logger.hpp"
#include "rtc_mpc/manager/mpc_solution_manager.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/phase/locomotion_phase_manager.hpp"
#include "rtc_mpc/reference/command_filter.hpp"
#include "rtc_mpc/thread/handler_mpc_thread.hpp"
#include "rtc_mpc/thread/solve_trigger_channel.hpp"
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

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace integrated_bringup {

/// Body joints the controller drives (the joint log's capacity).
inline constexpr std::size_t kLocomotionMaxBodyJoints = LocomotionDiagLogPod::kMaxJoints;

/// The stance checks of a lost plan (header): every contact frame this close
/// to the lowest one, the base slower than this.
inline constexpr double kRelatchFootSpreadM = 0.015;
inline constexpr double kRelatchBaseSpeed = 0.1;  ///< [m/s]

/// @brief `mpc:` (header comment). `present` false = the section is absent.
struct LocomotionMpcConfig {
  bool present{false};
  bool enabled{false};
  int period_nodes{1};
  double max_policy_age_s{0.0};  ///< checked here; the plan sampler reads it from `node`
  bool riccati_enabled{false};
  double sanity_joint_delta_rad{0.0};  ///< [rad]
  double sanity_base_speed{0.0};       ///< [m/s]
  double sanity_wrench_scale{0.0};     ///< × the robot's weight
  bool lockstep{false};                ///< `sync.mode: lockstep` (header comment)
  int delivery_ticks{0};               ///< `sync.delivery_ticks`; 0 = one period
  double wait_timeout_s{2.0};          ///< `sync.wait_timeout_s`
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
  double pose_guard_base_tilt_rad{0.0};  ///< `fault.pose_guard.base_tilt_rad`, (0, π/2]
  YAML::Node whole_body_qp;
  LocomotionMpcConfig mpc;
  std::string velocity_command_topic;  ///< `velocity_command.topic`; empty = no command path
  /// `velocity_command.type`: false = geometry_msgs/Twist (the default), true = sensor_msgs/Joy.
  bool velocity_command_joy{false};
  locomotion::JoyCommandMapping velocity_command_joy_map{};  ///< `velocity_command.joy` (type joy)
  std::string external_wrench_topic;       ///< `external_wrench.topic`; empty = no such lane
  double external_wrench_max_age_ms{0.0};  ///< `external_wrench.max_age_ms`, > 0 with a topic
};

/// The command timer's output with its source (one SeqLock: never torn apart).
struct LocomotionCommandOut {
  rtc::mpc::FilteredCommand cmd{};
  LocomotionCommandSource source{};
};

static_assert(std::is_trivially_copyable_v<LocomotionCommandOut>,
              "LocomotionCommandOut travels through a SeqLock");

/// @brief One Twist as the subscription stores it for the command timer.
struct RawVelocityCommandSample {
  rtc::mpc::RawVelocityCommand twist{};
  /// Which callback wrote it and what it saw (a Twist: input 0, deadman −1).
  LocomotionCommandSource source{};
  /// Steady clock at the callback. Before the first message: a stamp no clock
  /// is within t_stale of, so the filter reads it as stale (its age test is
  /// overflow-safe).
  std::int64_t recv_steady_ns{std::numeric_limits<std::int64_t>::min()};
  std::uint64_t sequence{0};  ///< +1 per message
};

static_assert(std::is_trivially_copyable_v<RawVelocityCommandSample>,
              "RawVelocityCommandSample travels through a SeqLock");

/// @brief What one command timer step hands the RT tick for the diagnostic log.
struct LocomotionCommandLogSample {
  double v_x{0.0};
  double v_y{0.0};
  double omega_z{0.0};
  double base_height{0.0};
  bool fresh{false};        ///< the filter's input was fresh at this step
  std::int32_t gait{-1};    ///< the phase manager's gait index; −1 = none
  std::uint8_t input{0};    ///< LocomotionCommandSource::input of the step's sample
  std::int8_t deadman{-1};  ///< LocomotionCommandSource::deadman of the step's sample
};

static_assert(std::is_trivially_copyable_v<LocomotionCommandLogSample>,
              "LocomotionCommandLogSample travels through a triple buffer");

/// @throws std::invalid_argument naming the key on a missing / unknown key of
///         `pd_feedforward`, `fault`, `fault.pose_guard`, `mpc`, `velocity_command` or
///         `external_wrench`, a gain that is negative or not finite, a `base_tilt_rad`
///         outside (0, π/2] or not finite, a `command_type` other than pd_feedforward, an
///         empty command topic, an external wrench age that is not finite and > 0, or a
///         model / estimator section its own parser refuses.
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

  /// The last tick's T-tier record, filled every tick (pushed only when a
  /// truth lane was fresh and the log is configured).
  [[nodiscard]] const LocomotionTruthLogPod& LastTruthLog() const noexcept { return truth_log_; }

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

  /// The measured-state source (null until the device configs wire it).
  [[nodiscard]] const locomotion::MeasuredStateSource* StateSource() const noexcept {
    return source_.get();
  }

  /// True when this configuration runs the MPC (mpc.enabled).
  [[nodiscard]] bool MpcEnabled() const noexcept { return mpc_enabled_; }

  /// True when the MPC runs in lock-step with the RT tick (`mpc.sync.mode: lockstep`).
  [[nodiscard]] bool MpcLockstep() const noexcept { return mpc_lockstep_; }

  /// Lock-step: the trigger period and the delivery delay, in ticks.
  [[nodiscard]] std::int64_t MpcGridTicks() const noexcept { return mpc_grid_ticks_; }

  [[nodiscard]] std::int64_t MpcDeliveryTicks() const noexcept { return mpc_delivery_ticks_; }

  /// Lock-step: the trigger channel (tests arm / read it; the tick requests on it).
  [[nodiscard]] const rtc::mpc::SolveTriggerChannel& MpcChannel() const noexcept {
    return mpc_channel_;
  }

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

  /// True when a command subscription exists (mpc.enabled and a topic configured):
  /// the Twist one or, under `velocity_command.type: joy`, the Joy one.
  [[nodiscard]] bool VelocityCommandSubscribed() const noexcept {
    return velocity_sub_ != nullptr || joy_sub_ != nullptr;
  }

  /// True when the external wrench lane has a subscription (`external_wrench.topic`).
  [[nodiscard]] bool ExternalWrenchSubscribed() const noexcept {
    return external_wrench_sub_ != nullptr;
  }

  /// The command filter's last output (what the phase manager was last given).
  [[nodiscard]] rtc::mpc::FilteredCommand LastCommand() const noexcept {
    return command_out_.Load().cmd;
  }

  /// The input behind LastCommand: type, deadman, Joy rejects.
  [[nodiscard]] LocomotionCommandSource LastCommandSource() const noexcept {
    return command_out_.Load().source;
  }

  /// Messages (Twist or Joy) the subscription stored since the last activation.
  [[nodiscard]] std::uint64_t VelocityCommandsReceived() const noexcept {
    return raw_command_.Load().sequence;
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
  [[nodiscard]] std::vector<std::string> ContactFrameNames() const;
  [[nodiscard]] std::string WireDevices();
  void ResetLogState() noexcept;

  // The MPC (mpc_reference.cpp). Off the RT thread: BuildMpc (Pass 1),
  // SpawnMpcThread (first activation; a null handler = the factory's),
  // PauseMpc, StopMpcThread, LogMpcTick (the 1 Hz aux timer).
  void BuildMpc(const YAML::Node& cfg);
  [[nodiscard]] bool SpawnMpcThread(std::unique_ptr<rtc::mpc::MPCHandlerBase> handler) noexcept;
  void StopMpcThread() noexcept;
  void PauseMpc() noexcept;  ///< the command timer off, sampling off, the thread paused
  /// Lock-step: disarm the channel and give a waiting tick a few ms to leave
  /// Compute — before the thread is paused or stopped (the channel's contract).
  void ReleaseLockstepTick() noexcept;
  void LogMpcTick() noexcept;
  // The velocity command (command.cpp). Off the RT thread: the subscription
  // callback (Store only), the timer (the filter's one writer).
  void OnVelocityCommand(const geometry_msgs::msg::Twist& msg) noexcept;
  void OnJoyCommand(const sensor_msgs::msg::Joy& msg) noexcept;
  /// The external wrench subscription callback (off the RT thread; the lane's
  /// one writer): the message with the steady time it arrived.
  void OnExternalWrench(const geometry_msgs::msg::WrenchStamped& msg) noexcept;
  /// The sample before any message (and after an activation): stale, at rest,
  /// tagged with the configured input.
  [[nodiscard]] RawVelocityCommandSample RestCommandSample() const noexcept;
  void StepCommand() noexcept;
  void PushCommand(const rtc::mpc::FilteredCommand& cmd,
                   const LocomotionCommandSource& source) noexcept;
  void StartCommandTimer() noexcept;  ///< activation: the filter reset, the timer up
  void StopCommandTimer() noexcept;   ///< from PauseMpc and StopMpcThread
  void StoreMpcCounters() noexcept;   ///< the 1 Hz timer (and the thread's stop: zeros)
  // On the RT thread, in tick order.
  void WriteMpcState() noexcept;
  void FollowPlan(const rtc::ControllerState& state) noexcept;
  /// Lock-step (header): wait for a due solve, follow, trigger on the grid.
  void LockstepTick(const rtc::ControllerState& state) noexcept;
  /// Lock-step: block until the outstanding solve is answered (or the timeout
  /// latches the fault, or the channel is disarmed).
  void WaitForSolve() noexcept;
  [[nodiscard]] std::uint8_t Plausible(const rtc::mpc::WbcReference& r) const noexcept;
  /// Record the sampled plan for the diag log, before the check.
  void RecordSample(bool sampled) noexcept;
  /// Count the plausibility refusal of wbc_ref_'s plan (refused for good).
  void NoteRefusal(std::uint8_t check) noexcept;
  /// The sampled plan was refused: follow the one followed before it
  /// (header). True when it is held and plausible, in wbc_ref_.
  [[nodiscard]] bool HoldEarlierPlan() noexcept;
  /// Idle follows nothing: no plan to go back to.
  void ForgetFollowedPlans() noexcept;
  void CopyPlan() noexcept;
  void ApplyContactMask(std::uint32_t mask) noexcept;
  void LosePlan(const rtc::ControllerState& state) noexcept;
  /// The contact frames' world z from the tick's kinematics, in contact order;
  /// returns how many were written (the pose guard and the plausibility check).
  [[nodiscard]] std::size_t ContactHeights(
      std::array<double, LocomotionDiagLogPod::kMaxContacts>& z) const noexcept;
  void RestartMpcStream() noexcept;

  // Compute() pieces, in tick order.
  void ServiceRequests() noexcept;
  void ReadState(const rtc::ControllerState& state, int64_t now_ns) noexcept;
  /// The pose guard on this tick's measured state (header); `stance` adds the
  /// contact-spread and base-speed checks of a lost plan. Logs the verdict.
  [[nodiscard]] std::uint32_t GuardPose(bool stance) noexcept;
  /// The stance entries: guard → latch, or wait and after the settle time latch kPoseGuard.
  void TryLatchReference(const rtc::ControllerState& state) noexcept;
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
  double cos_tilt_{1.0};  ///< cos(fault.pose_guard.base_tilt_rad), once at LoadConfig
  std::uint32_t guard_settle_ticks_{1};  ///< refusals before kPoseGuard: round(0.5 s · rate)
  double total_mass_{0.0};
  double weight_{0.0};  ///< total_mass_ · |g| [N]
  uint32_t model_epoch_{1};
  std::unique_ptr<locomotion::MeasuredStateSource> source_;
  /// Unlatched kNotInitialized ticks before kSource (estimator.inekf.init.timeout_s).
  std::uint32_t init_timeout_ticks_{std::numeric_limits<std::uint32_t>::max()};

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
  uint32_t source_epoch_seen_{0};   ///< the source's state epoch of the last tick
  bool source_epoch_known_{false};  ///< false until the first tick after an activation
  uint32_t init_wait_ticks_{0};     ///< kNotInitialized ticks since the last restart
  uint32_t fault_count_{0};
  uint64_t tick_count_{0};
  bool estop_active_{false};
  uint32_t serviced_generation_{0};
  bool activation_seen_{false};
  uint32_t serviced_reset_epoch_{0};
  uint32_t serviced_estop_state_{0};  ///< the estop_state_ word the last tick saw
  uint32_t guard_fail_ticks_{0};      ///< guard refusals since the last restart
  uint32_t pose_guard_mask_{0};       ///< the guard's last verdict of this episode
  std::array<double, kLocomotionMaxBodyJoints> tau_{};       ///< this tick's feedforward
  std::array<double, kLocomotionMaxBodyJoints> tau_prev_{};  ///< the last valid QP torque
  /// The hand joints' hold targets, by full-model joint index; bit h = hand h is held.
  std::array<double, locomotion::kMaxStateJoints> hand_hold_{};
  uint32_t hand_hold_mask_{0};
  LocomotionDiagLogPod diag_{};
  LocomotionTruthLogPod truth_log_{};
  JointFeedforwardLogPod joint_log_{};

  // ── Cross-thread ───────────────────────────────────────────────────────────
  /// Bit 0 = E-STOP active; the rest counts the edges, so one load gives the
  /// tick both and a trigger-and-clear between two ticks is still seen (header).
  std::atomic<uint32_t> estop_state_{0};
  static_assert(std::atomic<uint32_t>::is_always_lock_free, "the E-STOP word must be lock-free");
  std::atomic<uint32_t> fault_reset_epoch_{0};
  std::atomic<bool> fault_latched_{false};

  // ── Topics, logs ───────────────────────────────────────────────────────────
  ControllerTopicHandles owned_topics_;
  rtc::SeqLock<LocomotionDiagLogPod> status_lock_;  ///< the tick's row, RT tick → publish thread
  rtc::ControllerLogSet log_set_{"locomotion_controller"};
  std::vector<rtc::LogHandle<DeviceStateLogPod>> state_log_handles_;
  rtc::LogHandle<JointFeedforwardLogPod> joint_log_handle_;
  rtc::LogHandle<LocomotionDiagLogPod> diag_log_handle_;
  rtc::LogHandle<LocomotionTruthLogPod> truth_log_handle_;
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
  // ── Lock-step (mpc.sync.mode: lockstep; header) ──
  bool mpc_lockstep_{false};
  std::int64_t mpc_grid_ticks_{0};             ///< N: the trigger period in ticks
  std::int64_t mpc_delivery_ticks_{0};         ///< L: the delivery delay in ticks
  std::int64_t mpc_wait_timeout_ns_{0};        ///< the tick's wait for a solve
  rtc::mpc::SolveTriggerChannel mpc_channel_;  ///< owned here; the thread holds a pointer
  // RT tick state of the lock-step.
  bool mpc_trigger_outstanding_{false};
  std::uint64_t mpc_trigger_iter_{0};
  bool mpc_grid_known_{false};
  std::uint64_t mpc_grid_origin_{0};  ///< the first latched tick of the stream
  double mpc_wait_us_{0.0};           ///< this tick's wait
  std::uint64_t mpc_wait_count_{0};
  std::uint64_t mpc_grid_skips_{0};      ///< grid ticks without a trigger (no state to send)
  std::atomic<bool> in_compute_{false};  ///< the RT tick is inside Compute (deactivate waits on it)
  // RT tick state.
  rtc::mpc::WbcReference wbc_ref_{};
  ReferenceSource ref_source_{
      ReferenceSource::kIdle};  ///< idle / stand / walk; the latch reads as fault
  std::uint64_t mpc_t_ns_{0};   ///< this tick on the MPC's clock (iteration · dt)
  std::uint32_t mpc_state_epoch_{0};
  std::uint32_t contact_mask_{0};
  std::uint64_t rejected_plan_id_{0};
  std::uint64_t followed_id_{0};         ///< the plan followed last (0 = none since idle)
  std::uint64_t followed_before_id_{0};  ///< the one followed before it
  std::atomic<std::uint64_t> plausibility_rejects_{0};  ///< written by the RT tick only
  std::atomic<std::uint64_t> plan_holds_{0};            ///< written by the RT tick only
  std::atomic<std::uint8_t> last_reject_check_{0};      ///< the check the last refusal failed
  double u_fb_norm_{0.0};
  LocomotionPlanSampleDiag sample_diag_{};  ///< this tick's sample, for the diag log
  // The velocity command: the subscription stores, the timer steps the filter.
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;  ///< type joy, instead
  locomotion::JoyCommandScale joy_scale_{};  ///< command.v_max / hdot_max, set at configure
  rtc::SeqLock<RawVelocityCommandSample> raw_command_;  ///< callback → timer
  rclcpp::CallbackGroup::SharedPtr command_cb_group_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rtc::SeqLock<LocomotionCommandOut> command_out_;  ///< timer → publish thread, tests
  /// timer → RT tick (the diagnostic log): one producer (PushCommand), one consumer (FillLogs).
  rtc::mpc::TripleBuffer<LocomotionCommandLogSample> command_log_;
  /// The wrench a simulator reports it applies to the robot (the truth log's
  /// truth_ext_* block): subscription callback → RT tick. A triple buffer, not
  /// a SeqLock — the tick must not wait on a writer of lower priority.
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr external_wrench_sub_;
  rtc::mpc::TripleBuffer<locomotion::ExternalWrenchSample> external_wrench_;
  std::uint64_t external_wrench_count_{0};  ///< the callback's own message count
  /// MPC thread → RT tick (the diagnostic log): the thread's last solve attempt
  /// (SetSolveRecordSink; reset to SolveRecord{} before each thread starts).
  rtc::mpc::TripleBuffer<rtc::mpc::SolveRecord> mpc_solve_record_;
  /// The body group's inference values per group (0 = no lane), from the device configs.
  int contact_force_stride_{0};
  /// The body group's sensor_layout.has_native_moment (the T tier's moment columns).
  bool contact_moment_lane_{false};
  /// The MPC thread's counters as the 1 Hz timer last read them (the publish
  /// thread reads this, never the thread).
  rtc::SeqLock<LocomotionMpcCounters> mpc_counters_;
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
