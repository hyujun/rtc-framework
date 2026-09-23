#ifndef UR5E_BRINGUP_CONTROLLERS_DEMO_CATCHING_CONTROLLER_H_
#define UR5E_BRINGUP_CONTROLLERS_DEMO_CATCHING_CONTROLLER_H_

// ── Catching controller, S5.1 skeleton + E-STOP contract (dynamic_catching) ──
//
// WHAT THIS IS. The catching controller's frame: lifecycle, the P-1 minimum
// E-STOP/fault contract (E-8, approved 2026-09-22), the supervisor mode driven
// off S1.8's transition table, the operator arm/disarm latch, and the re-arm
// reset. The arm still HOLDS the pose it was activated in — the tracking law
// (CLIK, reference generation) is S5.3 — and the hand still accepts unshaped
// step targets so S4.2's measurement rig keeps working until the sequencer
// takes the hand in S7.1.
//
// WHY THE ARM STILL HOLDS. S5.1 is the step that opens the arm command path,
// and opening it is the E-8 change. The contract has to be in place and
// testable BEFORE a law is writing to it, or the first thing exercising the
// E-STOP path would also be the first thing moving the arm.
//
// THE P-1 CONTRACT (plan §4.4 S5.1, L7 §4.1), in the four parts it was
// approved in:
//   (a) TriggerEstop / ClearEstop / ResetFault / ResetTargetInitialization
//       update an atomic REQUEST or epoch and nothing else. The one writer of
//       the reset itself — the code that puts values back — is the RT tick.
//   (b) Invalidation (plan, trajectory, covariance, hand, FSM, timers) is
//       performed by that tick in D-23 order, keyed off the activation
//       generation rather than off a quiescence wait: PeriodicRtThread::Pause
//       does not stop an iteration already in flight, so "no tick is running"
//       is not a state a lifecycle hook can establish.
//   (c) No automatic resume. A clear returns the supervisor to IDLE and LOWERS
//       the operator's arm latch, so restarting requires a deliberate act.
//   (d) ClearEstop does not touch the controller fault latch, and ResetFault
//       does not touch the global E-STOP. They are separate paths on purpose
//       (base contract, issue #260): one clear must not launder the other.
//
// WHY EPOCHS RATHER THAN FLAGS. A flag cannot distinguish "still estopped"
// from "estopped, cleared and estopped again between two ticks", and the
// second case must still reseed. Counters make the tick's question "has the
// count moved since I last looked", which is answerable without a lock and
// without the hook ever touching controller state.
//
// SIM-ONLY GUARD, REPLACED (A-S5-1, 2026-09-22). S4.0 refused to ACTIVATE
// unless every claimed device was the `mujoco_native` backend, because
// commanding a real arm at all was the pending E-8 decision. That decision has
// been made, so the guard that stood in for it is gone. What remains is the
// rule it was standing in for: L0 §5.3 / L8 §5.4 — a provisional value blocks
// a REAL-ARM configuration and only warns in sim. So the backend check now
// selects the validator's `real_arm_config` axis instead of gating activation
// by itself, and a real-arm bring-up whose profile is still provisional lands
// in the same DISABLED state (configure SUCCESS, activate refused) for a
// reason that is about the VALUES rather than about the backend.
//
// WHY DISABLED RATHER THAN A CONFIGURE REFUSAL (2026-09-21, unchanged). A
// configure failure takes the whole robot down: sim and real share
// `config/<variant>/controllers/`, so the real p1b bring-up instantiates this
// controller too, and CM's Pass 3 latches `bring_up_failed` on ANY
// controller's configure failure and then refuses to configure EVERY
// controller (rt_controller_node_params.cpp). Nothing is commanded until a
// controller is active, so refusing activation carries the intent exactly.
//
// THE ARM/DISARM CHANNEL (A-S5-3) is the read-write ROS parameter
// `catching.enable` on this controller's node. It is not a message or a
// service: §13's GUI needs one channel, and a new msg/srv here would be an E-3
// decision for something a parameter already expresses. The parameter callback
// stores an atomic; the RT tick is what acts on it, and the tick LOWERS it on
// E-STOP and on a fault latch, which is what makes (c) mechanical rather than
// a property of operator discipline.
//
// THE PLANNER THREAD (S6-A, D-7, E-7 decision J). With `planner.enabled` the
// controller owns a CatchingPlannerThread on the `mpc` layout role — thread name
// `mpc_main`, the same core and scheduling as DemoWbc's MPC solver, because the
// planner plays the same role. Lifecycle: buffers and the wake eventfd at
// configure, layout-profile gate as on_activate's first statement, lazy spawn +
// Resume at activate, Pause at deactivate, JOIN at cleanup — unlike DemoWbc's
// MPC thread, which lives to the destructor, because a planner that outlived
// its configuration would resume beside an oracle as a second box writer. The RT tick stores
// `PlannerRtState` every tick and loads the plan box every tick (D-21); a published plan is taken
// only when rtc::catching::JudgePlan admits it (L3 §5.2 (a)-(f)). The oracle plan (A-S5-8) writes
// the SAME box from the RT tick, so there is one consumption path — and a profile enabling both
// writers is parked.
//
// THE HAND STEP IS STILL A DIAGNOSTIC MODE, gated on `diagnostic.hand_step`.
// From S7.1 the sequencer owns the hand and the default false makes an
// operator step a refusal rather than a race. A profile that turns the step
// on runs WITHOUT the sequencer (S4's measurement rig), so the two never
// command the hand at once.
//
// THE S7 CYCLE (L7 §4.1, #537 S7). Armed, the arm homes to `planner.wait_pose`
// in joint space and the hand waits at q_pre (IDLE → ARMED); a plan is
// followed (APPROACH), frozen at t_c − T_freeze (COMMITTED), the hand closes at
// t_cmd (CLOSING), the reference decelerates to rest after t_c (DECEL), holds
// for T_hold (HOLD), and the arm returns to the wait pose (RETREAT) and re-arms
// through ResetForRearm. Which member each reset puts back, and which it
// leaves, is the table below the RT members (G8-A2).

#include "integrated_bringup/controllers/catching/planner_thread.hpp"
#include "integrated_bringup/controllers/catching/traj_input.hpp"
#include "integrated_bringup/logging/catching_diag_log_pod.hpp"
#include "integrated_bringup/logging/device_state_log_pod.hpp"
#include "integrated_bringup/support/combined_model_cache.hpp"
#include "integrated_bringup/support/layout_profile.hpp"
#include "integrated_bringup/support/owned_topics.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_base/timing/rt_tick_timing_sample.hpp"
#include "rtc_base/timing/thread_timing_csv_logger.hpp"
#include "rtc_controller_interface/controller_log_set.hpp"
#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/catching/catch_pose_ik_params.hpp"
#include "rtc_controllers/catching/catching_params.hpp"
#include "rtc_controllers/catching/contact_debounce.hpp"
#include "rtc_controllers/catching/decel_target.hpp"
#include "rtc_controllers/catching/hand_sequencer.hpp"
#include "rtc_controllers/catching/joint_home.hpp"
#include "rtc_controllers/catching/planner_cycle.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/soft_catch.hpp"
#include "rtc_controllers/catching/traj_sampler.hpp"
#include "rtc_controllers/catching/transition_table.hpp"
#include "rtc_tsid/kinematics/clik_reference.hpp"

#include <rclcpp/callback_group.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <array>
#include <atomic>
#include <cstdint>
#include <fstream>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace integrated_bringup {

using rtc::CommandType;
using rtc::ControllerOutput;
using rtc::ControllerState;
using rtc::kMaxDeviceChannels;
using rtc::RTControllerInterface;

/// Backend tag that marks a claimed device as the simulator. A registry tag,
/// not a device name — robot-agnostic (ARCH-1).
///
/// Since A-S5-1 this no longer gates activation: it selects the validator's
/// `real_arm_config` axis (L0 §5.3 — provisional values warn in sim and block
/// a real arm). A device that declares no backend at all counts as NOT proven
/// to be the simulator, so an unprovable config is judged by the stricter
/// rule rather than the laxer one.
inline constexpr const char* kCatchingSimBackendType = "mujoco_native";

/// The operator arm/disarm channel (A-S5-3): a read-write ROS parameter on
/// this controller's own LifecycleNode (`/<config_key>/catching.enable`).
///
/// A parameter rather than a message or a service because §13's GUI needs one
/// channel and a new msg/srv would be an E-3 decision for something a
/// parameter already expresses. The callback stores an atomic; the RT tick is
/// what acts on it, and the tick also LOWERS it on E-STOP and on a fault
/// latch, which is what makes "no automatic resume" mechanical.
inline constexpr const char* kCatchingEnableParam = "catching.enable";

/// The spacing floor, as a fraction of the expected prediction interval.
///
/// A derived floor rather than a configured one: the number that matters is
/// "much denser than vision can possibly be", and expressing it against the
/// interval the profile already declares keeps the two from drifting apart.
/// A tenth leaves room for a publisher that doubles its rate without warning
/// while still refusing the degenerate spacings the Hermite basis divides by.
inline constexpr double kTrajSpacingFloorFraction = 0.1;

/// Velocity bound [rad/s] the hand's joints get in the CLIK solve.
///
/// Not zero, because the solver's box requires strictly positive limits, and
/// not a "small" number chosen for feel: it is below the resolution of any
/// joint this controller commands, so a solution that uses it is numerically
/// indistinguishable from one that does not move the hand at all.
inline constexpr double kLockedJointVelocity = 1e-9;

/// Why an instance was parked at configure (configure SUCCESS, activation
/// refused). A park commands nothing and keeps the rest of the robot up — CM
/// refuses EVERY controller when one fails configure, so parking is how this
/// controller says "not me" without saying "not the robot".
enum class CatchingParkReason : std::uint8_t {
  kNone = 0,
  /// A value this controller consumes is provisional or still TBD on a real
  /// arm (L0 §5.3, A-S5-1), or still TBD in sim (A-S5-12).
  kConsumedValues,
  /// `planner.enabled` and `diagnostic.oracle_plan.enabled` are both true: the
  /// plan box would have two writers (S6-A).
  kPlannerOracleConflict,
  /// The planner is enabled but a value it cannot guess is unset or TBD
  /// (`planner.sub_model`, `freeze.T_freeze`, `workspace.catch_box`,
  /// `hand.d_eff`, `hand.r_cap`) — S6-B.
  kPlannerUnset,
  /// The tracking law is wired, so trials can run, but a value the S7
  /// supervisor needs for one is unset or wrong: `planner.freeze.T_freeze`
  /// (the commit instant, and at least T_close_e2e + T_arm + h), the
  /// `planner.wait_pose` homing target, the derived acceleration box the
  /// joint-space motions ramp with, `supervisor.decel.a_dec`, or a hand profile
  /// the sequencer can run.
  kSupervisorUnset,
};

/// Controller-local device indices. This controller claims exactly two groups
/// in `topics:` order: the arm it holds and the hand it steps. Indices rather
/// than names because that is what ControllerState / ControllerOutput are
/// indexed by; the NAMES come from YAML, so no robot is hard-coded.
inline constexpr int kCatchingArmDeviceIdx = 0;
inline constexpr int kCatchingHandDeviceIdx = 1;

/// Fixed POD capacity; the runtime widths live in arm_dof_ / hand_dof_,
/// resolved from the device configs (never from a robot constant).
inline constexpr int kDemoCatchingMaxArmDof = 32;
inline constexpr int kDemoCatchingMaxHandDof = static_cast<int>(rtc::catching::kMaxHandDof);

class DemoCatchingControllerResetProbe;  // test_catching_reset_probe.cpp (G8-A2)

class DemoCatchingController final : public RTControllerInterface {
  // The reset table's runtime check: poisons the RT-owned members, runs a
  // reset, and holds each member to its row. Test-only; the one friend.
  friend class DemoCatchingControllerResetProbe;

 public:
  explicit DemoCatchingController(std::string_view urdf_path);

  /// Joins the planner thread and closes its wake eventfd. The thread reads
  /// boxes this object owns, so it must be gone before they are.
  ~DemoCatchingController() override;

  DemoCatchingController(const DemoCatchingController&) = delete;
  DemoCatchingController& operator=(const DemoCatchingController&) = delete;
  DemoCatchingController(DemoCatchingController&&) = delete;
  DemoCatchingController& operator=(DemoCatchingController&&) = delete;

  [[nodiscard]] ControllerOutput Compute(const ControllerState& state) noexcept override;

  /// Publish the catching state (D-20). Runs on CM's publish jthread —
  /// SCHED_OTHER, NOT the RT tick, despite the name.
  void PublishNonRtSnapshot(const rtc::PublishSnapshot& snap) noexcept override;

  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override { return "DemoCatchingController"; }

  void LoadConfig(const YAML::Node& cfg) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& prev,
                              rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                              const YAML::Node& yaml) noexcept override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) noexcept override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) noexcept override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) noexcept override;

  // ── E-STOP and fault (P-1 minimum contract, E-8) ─────────────────────────
  // All four are called OFF the RT thread (CM service callbacks / the global
  // E-STOP propagation) and all four are wait-free stores. None of them
  // resets anything: they raise a request or bump an epoch, and Compute()
  // is the single writer that acts on it — P-1 (a).

  void TriggerEstop() noexcept override;
  void ClearEstop() noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void ResetFault() noexcept override;
  [[nodiscard]] bool HasLatchedFault() const noexcept override;

  // ── Observation surface (tools, GUI, tests) ──────────────────────────────
  // The measurement tools read the profile the CONTROLLER loaded, never the
  // shipped YAML directly: a run whose controller read something else must not
  // be analysed against the file on disk. on_configure mirrors these into
  // read-only ROS parameters for the off-process readers.

  [[nodiscard]] const rtc::catching::CatchingParams& GetCatchingParams() const noexcept {
    return params_;
  }

  [[nodiscard]] const rtc::catching::CatchingValidationReport& GetValidationReport()
      const noexcept {
    return report_;
  }

  [[nodiscard]] bool IsHandStepEnabled() const noexcept { return hand_step_enabled_; }

  // ── Planner (S6-A) ───────────────────────────────────────────────────────

  [[nodiscard]] const rtc::catching::PlannerParams& GetPlannerParams() const noexcept {
    return planner_params_;
  }

  /// Record this run's layout profile (issue #350). on_configure reads it from
  /// the `rt_layout_profile` node parameter; this setter exists for fixtures
  /// that bring the controller up through LoadConfig, which never reaches
  /// on_configure. Same shape as DemoWbcController::SetLayoutProfile.
  void SetLayoutProfile(std::string_view profile) noexcept {
    layout_profile_drops_mpc_ = LayoutProfileDropsMpc(profile);
  }

  /// The planner thread, or null before the first activation that spawned it
  /// (and always null with `planner.enabled: false`).
  [[nodiscard]] const CatchingPlannerThread* GetPlannerThread() const noexcept {
    return planner_thread_.get();
  }

  /// The wake eventfd, -1 until a configure with the planner enabled made one.
  [[nodiscard]] int GetPlannerWakeFd() const noexcept {
    return planner_wake_fd_.load(std::memory_order_acquire);
  }

  /// What the RT stored for the planner on its last tick.
  [[nodiscard]] rtc::catching::PlannerRtState GetPlannerRtState() const noexcept {
    return planner_rt_box_.Load();
  }

  /// The plan box as the RT will load it next tick.
  [[nodiscard]] rtc::catching::PlanSnapshot GetPublishedPlan() const noexcept {
    return plan_box_.Load();
  }

  /// The box itself, for a test that plays the planner — ONLY with the planner
  /// and the oracle both disabled, or the box has two writers.
  [[nodiscard]] rtc::SeqLock<rtc::catching::PlanSnapshot>& PlanBoxForTesting() noexcept {
    return plan_box_;
  }

  /// Why the last tick did not take the plan in the box (`kNone` = it did, or
  /// it took one earlier and is following it).
  [[nodiscard]] rtc::catching::PlanRefusal GetLastPlanRefusal() const noexcept {
    return static_cast<rtc::catching::PlanRefusal>(
        plan_refusal_observed_.load(std::memory_order_relaxed));
  }

  /// Plans that replaced the one being followed in APPROACH (§4.7, S6-B).
  /// The vision → model-world transform the ingress applies (plan §11),
  /// resolved at configure from `catching.io.arm_base_frame` +
  /// `catching.io.base_T_world` and the model.
  [[nodiscard]] const TrajInputConfig& GetTrajInputConfig() const noexcept {
    return traj_input_.Config();
  }

  [[nodiscard]] std::uint64_t GetPlanReplacedCount() const noexcept {
    return plan_replaced_count_.load(std::memory_order_relaxed);
  }

  /// Whether the planner has a model to search in (S6-B). False = the S6-A
  /// stub ("no plan") — a profile with no system model, e.g. a unit fixture.
  [[nodiscard]] bool IsPlannerSearchConfigured() const noexcept {
    return planner_cycle_.SearchConfigured();
  }

  /// Plans the RT has adopted (TRACKING → APPROACH edges it took on a plan).
  [[nodiscard]] std::uint64_t GetPlanAdmittedCount() const noexcept {
    return plan_admitted_count_.load(std::memory_order_relaxed);
  }

  /// The plan being followed (RT-owned; read off the RT thread by tests only).
  [[nodiscard]] const rtc::catching::PlanSnapshot& GetFollowedPlanForTesting() const noexcept {
    return plan_;
  }

  /// The supervisor mode the last RT tick left behind, and the reason that put
  /// it there. Read off the RT thread (tests, and the state message from
  /// S5.4): a relaxed load of a value one writer stores, which is all a
  /// diagnostic read needs. Mode::kIdle until the first tick runs.
  [[nodiscard]] rtc::catching::Mode GetMode() const noexcept {
    return static_cast<rtc::catching::Mode>(mode_observed_.load(std::memory_order_relaxed));
  }

  [[nodiscard]] rtc::catching::Reason GetLastReason() const noexcept {
    return static_cast<rtc::catching::Reason>(reason_observed_.load(std::memory_order_relaxed));
  }

  // ── S7 supervisor (RT-owned; read off the RT thread by tests only) ───────

  /// The hand sequencer's output on the last tick (`active` false = the hand
  /// is on the controller's latch).
  [[nodiscard]] const rtc::catching::HandSequencerOutput& GetHandOutputForTesting() const noexcept {
    return hand_out_;
  }

  /// How the last attempt ended (L7 §4.7). Kept across a re-arm — it is the
  /// LAST attempt's verdict — and cleared by an activation or E-STOP reset.
  [[nodiscard]] rtc::catching::Outcome GetOutcomeForTesting() const noexcept { return outcome_; }

  /// Whether trials can run at all: the law is wired and every supervisor
  /// value it needs is present (false = the S6 behaviour: hold arm and hand).
  [[nodiscard]] bool AreTrialsEnabled() const noexcept { return trials_enabled_; }

  [[nodiscard]] bool IsHomingForTesting() const noexcept { return homing_; }

  /// The carried arm command (device order) — what the joint-space motions
  /// integrate and what the wire carries once the controller drives the arm.
  [[nodiscard]] const std::array<double, kDemoCatchingMaxArmDof>& GetArmCommandForTesting()
      const noexcept {
    return arm_q_cmd_;
  }

  [[nodiscard]] const std::array<double, kDemoCatchingMaxArmDof>& GetArmVelocityCommandForTesting()
      const noexcept {
    return arm_qd_cmd_;
  }

  /// The operator arm latch (`catching.enable`) as the RT tick currently sees
  /// it. NOT the parameter's value: the tick lowers this on E-STOP and on a
  /// fault latch (P-1 (c)), so the two disagree exactly when something has
  /// disarmed the controller out from under the operator — which is the case
  /// worth being able to observe.
  [[nodiscard]] bool IsArmRequested() const noexcept {
    return arm_requested_.load(std::memory_order_relaxed);
  }

  /// How many times the RT tick has performed a reset because a hook moved an
  /// epoch (P-1 (a)/(b)). The counter exists so a race test can assert that a
  /// burst of concurrent Trigger/Clear/deactivate calls produced resets only
  /// on tick boundaries, and that the reset writer is the tick.
  [[nodiscard]] std::uint64_t GetRtResetCount() const noexcept {
    return rt_reset_count_.load(std::memory_order_relaxed);
  }

  /// The vision ingress, for the counters and the last message's diagnostics.
  /// Read off the RT thread; the ingress itself is only ever touched by the
  /// subscription callback.
  [[nodiscard]] const CatchingTrajInput& GetTrajInput() const noexcept { return traj_input_; }

  /// What the last RT tick concluded about the trajectory snapshot it held.
  [[nodiscard]] rtc::catching::TrajView GetTrajView() const noexcept { return traj_view_; }

  /// The last CLIK solve's diagnostics — status, iteration count, solve time,
  /// `bound_conflict`. Read off the RT thread; this is the input to the S5.4
  /// state message and to the G5-C timing gate.
  [[nodiscard]] const rtc::tsid::ClikReferenceGenerator::SolveDiagnostics& GetLastSolve()
      const noexcept {
    return clik_.LastSolve();
  }

  /// ‖q_meas − q_cmd‖ [rad] of the last tick (the TRACK_ERR watchdog's input).
  [[nodiscard]] double GetTrackError() const noexcept { return track_err_; }

  /// Whether a plan is being followed, and whether the law is wired at all.
  [[nodiscard]] bool IsPlanActive() const noexcept { return plan_active_; }

  [[nodiscard]] bool IsClikEnabled() const noexcept { return clik_enabled_; }

  /// Ticks that ran with the global E-STOP request raised.
  [[nodiscard]] std::uint64_t GetEstopTickCount() const noexcept {
    return estop_tick_count_.load(std::memory_order_relaxed);
  }

  /// The record the last RT tick left behind — the SAME object the CSV row and
  /// the state message are built from, so a test that reads it is reading what
  /// the operator sees. PROC-7: it is rebuilt from scratch every tick, so a
  /// field that is still zero is a field this tick did not compute.
  [[nodiscard]] CatchingDiagLogPod GetLastTickRecord() const noexcept {
    return catching_state_lock_.Load();
  }

  /// The ingress counters as the subscription last published them. Loaded off
  /// the RT thread; see CatchingIngressSnapshot for why this is a SeqLock
  /// rather than a direct read of `traj_input_`.
  [[nodiscard]] CatchingIngressSnapshot GetIngressSnapshot() const noexcept {
    return ingress_diag_box_.Load();
  }

  /// Bind the CSV channel from a test-owned ControllerLogSet.
  ///
  /// The unit fixtures bring the controller up through LoadConfig, which never
  /// reaches on_configure and therefore leaves every production log handle
  /// UNBOUND — and a row assertion written against that state passes with the
  /// push deleted outright (#424). Same seam, same reason, as
  /// DemoComplianceController::SetComplianceDiagLogHandleForTesting.
  void SetCatchingDiagLogHandleForTesting(rtc::LogHandle<CatchingDiagLogPod> handle) noexcept {
    catching_diag_log_handle_ = std::move(handle);
  }

  /// The ARM's position box as the solver received it — margined, device
  /// order, empty when the box is incomplete. Exposed so a test can assert
  /// that the abort ramp and CLIK were handed the SAME box rather than
  /// re-deriving the margin at the call site and comparing a copy to a copy.
  [[nodiscard]] const std::vector<double>& GetArmPositionBoxLowerForTesting() const noexcept {
    return arm_q_min_margined_;
  }

  [[nodiscard]] const std::vector<double>& GetArmPositionBoxUpperForTesting() const noexcept {
    return arm_q_max_margined_;
  }

  [[nodiscard]] int GetArmDof() const noexcept { return arm_dof_; }

  [[nodiscard]] int GetHandDof() const noexcept { return hand_dof_; }

  /// Group names whose backend is not `mujoco_native` (or which declare no
  /// `backend:` block at all). Non-empty ⇒ the instance configures DISABLED.
  [[nodiscard]] const std::vector<std::string>& GetNonSimGroups() const noexcept {
    return non_sim_groups_;
  }

  /// True once on_configure has parked this instance (GetParkReason() says
  /// why): a REAL-ARM configuration (no proof that every claimed device is the
  /// simulator) whose profile still carries a value this controller consumes as
  /// provisional or TBD (L0 §5.3, A-S5-1); a SIM configuration whose consumed
  /// value is still TBD (A-S5-12); or a profile that enables both the planner
  /// and the oracle plan (S6-A). A disabled instance holds no step lane, no log
  /// channels and no profile parameters, and on_activate refuses — so it
  /// commands nothing.
  ///
  /// The name is kept from S4.0, where the same state meant "not the
  /// simulator" on its own; today it is about the VALUES. See IsRealArmConfig().
  [[nodiscard]] bool IsSimOnlyDisabled() const noexcept { return sim_only_disabled_; }

  /// Why IsSimOnlyDisabled() is true — kNone while it is false.
  [[nodiscard]] CatchingParkReason GetParkReason() const noexcept { return park_reason_; }

  /// Whether the claimed devices failed to prove they are all the simulator —
  /// the validator's `real_arm_config` axis for this configure.
  [[nodiscard]] bool IsRealArmConfig() const noexcept { return real_arm_config_; }

  /// Targets refused because they addressed the arm. The arm slot has exactly
  /// one writer here (the activation latch), so a goal sent to it is a mistake
  /// worth counting rather than a command worth obeying.
  [[nodiscard]] std::uint64_t GetArmTargetRejectCount() const noexcept {
    return arm_target_reject_count_.load(std::memory_order_relaxed);
  }

  /// Hand targets refused because `diagnostic.hand_step` is false.
  [[nodiscard]] std::uint64_t GetHandStepDisabledRejectCount() const noexcept {
    return hand_step_disabled_reject_count_.load(std::memory_order_relaxed);
  }

  /// Hand targets refused because they did not carry one value per hand joint.
  /// A step commands every joint or none — see SetDeviceTarget.
  [[nodiscard]] std::uint64_t GetHandTargetWidthRejectCount() const noexcept {
    return hand_target_width_reject_count_.load(std::memory_order_relaxed);
  }

  /// Task-space goals refused. This controller has no task lane; the base's
  /// default would forward the six pose numbers into the hand's joint lane.
  [[nodiscard]] std::uint64_t GetTaskTargetRejectCount() const noexcept {
    return task_target_reject_count_.load(std::memory_order_relaxed);
  }

  /// Hand step targets applied on an RT tick since construction.
  [[nodiscard]] std::uint64_t GetHandStepAppliedCount() const noexcept {
    return hand_step_applied_count_.load(std::memory_order_relaxed);
  }

 protected:
  void ApplyPendingTarget(int device_idx, std::span<const double> values,
                          bool is_task) noexcept override;
  void SetDeviceTaskTarget(int device_idx, std::span<const double> task6) noexcept override;
  void OnDeviceConfigsSet() override;
  void ResetTargetInitialization() noexcept override;

 private:
  /// Resolve per-device widths, joint limits, log header names and the backend
  /// check from `device_name_configs_` ∩ `topic_config_.groups`. Idempotent:
  /// CM calls SetDeviceNameConfigs after PreConfigure (so the groups exist),
  /// but a fixture that calls on_configure directly sets the configs first, so
  /// on_configure re-runs this rather than trusting the hook to have seen them.
  void ResolveDevices();

  /// Mirror the loaded profile into read-only ROS parameters. Non-RT.
  void DeclareProfileParameters();

  /// Close every CSV channel and unbind the handles (#238 — a re-configure
  /// must not land in a channel the previous configure owned).
  void ResetLogState() noexcept;

  /// Release everything on_configure builds: drain and close the CSV channels,
  /// stop the drain timer, and destroy the owned topics. Used by on_cleanup and
  /// by on_configure's own catch, so a throw cannot leave a half-built instance.
  void TearDownConfiguredResources() noexcept;

  /// Declare / update the `catching.enable` parameter channel (A-S5-3) and
  /// install the set-parameter callback. Non-RT.
  void DeclareArmParameter();

  /// Build the model, register the catch frame, size the CLIK boxes and
  /// initialise the generator. Non-RT; leaves `clik_enabled_` false (and logs
  /// why) rather than throwing, so a profile that cannot run the law still
  /// configures and still holds the arm.
  void SetupArmCommand();

  /// Read the D-16 acceleration box (A-S5-6) from the file the YAML names.
  /// Returns false and logs when the file is missing, not adopted, or does not
  /// cover the arm — the box is a DERIVED artefact, so an absent one is a
  /// configuration error and not something to substitute a default for.
  [[nodiscard]] bool LoadDerivedAccelLimits();

  /// Assemble the position / velocity / acceleration boxes CLIK is given, in
  /// Pinocchio order. Non-RT.
  void BuildClikBoxes(int nv, rtc::tsid::ClikReferenceGenerator::Config& cfg);
  /// Decision K (S6-C2): puts `joint_cmd.accel_constraint`'s form into `cfg`
  /// after BuildClikBoxes. False (logged) when `dynamic` is selected but the
  /// arm device has no usable `joint_limits.max_torque` — the arm is then held.
  [[nodiscard]] bool ConfigureAccelConstraint(int nv,
                                              rtc::tsid::ClikReferenceGenerator::Config& cfg);

  /// One tick of the tracking law: sample → reference → CLIK → arm command.
  /// Returns the reason to report, `kNone` when the tick was healthy. RT only.
  [[nodiscard]] rtc::catching::Reason RunTrackingTick(
      const ControllerState& state, const rtc::catching::TrajectorySnapshot& snapshot) noexcept;

  /// Walk the arm command to a stop without the QP (A-S5-10). RT only.
  void RunJointSpaceAbort(const ControllerState& state) noexcept;

  /// Seed the command state, the CLIK anchor and the reference generator from
  /// the measured pose. RT only — called on the tick that starts following a
  /// plan and on every reset.
  void SeedArmCommand(const ControllerState& state) noexcept;

  /// Build the fixed plan the `diagnostic.oracle_plan` block describes
  /// (A-S5-8) and STORE it in the plan box — the same box the planner writes,
  /// so an oracle run exercises the same consumption path. RT only; called on
  /// every TRACKING tick without a plan, so the box always holds this tick's
  /// oracle plan rather than one built for an earlier instant.
  void StoreOraclePlan(rtc::catching::NowReal now) noexcept;

  /// Take an admitted plan: it becomes the plan being followed. RT only.
  void AdoptPlan(const rtc::catching::PlanSnapshot& plan) noexcept;

  /// Fill and store this tick's PlannerRtState. RT only, every tick.
  void StorePlannerRtState(const ControllerState& state, rtc::catching::NowReal now) noexcept;

  /// Configure-time planner setup: arm-width checks, the wake eventfd, the
  /// cycle's box binding, and (S6-B) the search's model on the catch
  /// sub-model. Non-RT. Returns false (and logs) on a configuration the
  /// planner cannot run with.
  [[nodiscard]] bool SetupPlanner();

  /// Build the search's model: a reorder-free handle on `planner.sub_model`,
  /// the catch frame, the model↔device joint map, velocity/acceleration
  /// limits and the profile constants. Non-RT; false (and logged) on a model
  /// the planner cannot use.
  [[nodiscard]] bool SetupPlannerSearch();

  /// The first planner value that is a decision and is unset, or nullptr.
  [[nodiscard]] const char* PlannerDecisionMissing() const noexcept;

  /// Spawn the planner thread on the `mpc` role (E-7 J) once per
  /// configuration, and open its timing CSV + 1 Hz drain timer. Non-RT
  /// (on_activate). Idempotent within a configuration.
  void SpawnPlannerThreadIfNeeded() noexcept;

  /// Join and drop the planner thread. Non-RT (lifecycle). Join wakes the
  /// poll (OnRequestStop) and waits for a wake already in flight, which is
  /// bounded by one PlannerCycle::Run.
  void StopPlannerThread() noexcept;

  /// 1 Hz aux timer: drain the planner timing ring into the CSV. Non-RT.
  void DrainPlannerTiming() noexcept;

  /// Copy this tick's verdict on the vision snapshot into the record. RT only.
  void RecordInputLane(const rtc::catching::TrajectorySnapshot& snapshot) noexcept;

  /// Copy one L4 reference step into this tick's record. RT only — called
  /// from the tracking tick at the moment the value exists, so a tick that
  /// never reached the generator leaves the block zero.
  void RecordReference(const rtc::catching::TranslationOutput& ref) noexcept;

  /// Copy one CLIK solve's diagnostics into this tick's record. RT only, same
  /// reason as RecordReference.
  void RecordClikSolve(const rtc::tsid::ClikReferenceGenerator::SolveDiagnostics& solve) noexcept;

  /// Fill `tick_record_` from the state this tick computed and hand it to both
  /// the CSV ring and the state-message SeqLock. RT tick only, called ONCE at
  /// the end of Compute() — PROC-7: every tick publishes a body, and the
  /// record was default-constructed at the top of the tick so anything this
  /// tick did not compute is zero rather than last tick's value.
  void PublishTickRecord(const ControllerState& state) noexcept;

  /// Create the prediction subscription and configure the ingress. Non-RT.
  void SetupTrajInput();
  /// The model builder: CM's shared one, else one of our own (fixtures). Left
  /// null (logged) without a system model config. Acquired at the start of the
  /// ingress setup, because the vision-frame transform needs the model BEFORE
  /// the subscription exists.
  void AcquireModelBuilder();
  /// Fills `cfg`'s vision → model-world transform (plan §11). False (logged)
  /// on a named frame the model lacks or that is not rigid to its root.
  [[nodiscard]] bool ResolveVisionFrame(TrajInputConfig& cfg);

  /// The subscription callback (non-RT). Samples the receive instants, hands
  /// the message to the ingress and, on acceptance, publishes both snapshots.
  void OnTrajectoryCloud(const sensor_msgs::msg::PointCloud2& msg) noexcept;

  /// Service any reset a lifecycle or E-STOP hook requested since the last
  /// tick, and return true if one was performed. RT tick only — this is the
  /// single writer P-1 (a) names. The order is D-23's: decide on the
  /// activation generation and the epochs first, then invalidate, then reseed.
  bool ServiceResetRequests(const ControllerState& state) noexcept;

  /// Put every piece of per-trial state back to its start-of-trial value
  /// (L7 §4.8's re-arm list, restricted to what exists so far). RT tick only.
  ///
  /// `reset_mode` says whether the supervisor mode is part of the reset. It is
  /// for an ACTIVATION (a fresh controller starts in IDLE) and it is not for
  /// an E-STOP, where the transition table decides — and where forcing IDLE
  /// would erase a latched FAULT the table keeps through a stop (P-1 (d)).
  void ResetTrialState(bool reset_mode) noexcept;

  /// What this tick tells the supervisor: a reason, and whether to take an
  /// edge at all.
  ///
  /// The second field exists because `Reason::kNone` means two different
  /// things depending on where the controller is. In IDLE it is "ready,
  /// proceed to ARMED"; in ARMED with no ball in sight it is "nothing is
  /// wrong AND nothing should happen". Collapsing them would need a
  /// nothing-to-report reason code in the table, which is a change to S1.8's
  /// data for a decision that belongs to the driver.
  struct ReasonDecision {
    rtc::catching::Reason reason{rtc::catching::Reason::kNone};
    bool advance{false};
  };

  /// Decide this tick's (reason, advance). RT tick only.
  [[nodiscard]] ReasonDecision EvaluateReason(
      const ControllerState& state, const rtc::catching::TrajectorySnapshot& snapshot) noexcept;

  /// Apply one (mode, reason) edge from S1.8's table. A reason with no row in
  /// the current mode is inapplicable there and leaves the mode alone — that
  /// is the table's contract, not a silent failure.
  void AdvanceMode(rtc::catching::Reason reason) noexcept;

  // ── S7 supervisor pieces (RT only unless noted) ──────────────────────────

  /// One tick of JointSpaceDecelStep on the carried command. True once every
  /// joint's commanded velocity is zero (or there is no command to ramp).
  [[nodiscard]] bool RampArmToStop(const ControllerState& state) noexcept;
  /// Evaluation-state prelude shared by every CLIK tick (D-6). False = the law
  /// cannot run this tick (nothing wired, arm unreadable).
  [[nodiscard]] bool PrepareLawTick(const ControllerState& state) noexcept;
  /// L4 step toward `target` at profile time `t_rel`, then CLIK, command and
  /// TRACK_ERR — the half every CLIK tick shares. `count_saturation` feeds the
  /// REF_SATURATED streak (ball ticks only; DECEL/HOLD have no row for it).
  [[nodiscard]] rtc::catching::Reason StepReferenceAndSolve(
      const ControllerState& state, const rtc::catching::TargetState& target, double t_rel,
      bool count_saturation) noexcept;
  /// The DECEL/HOLD law tick: the virtual target at τ = now_lead − t_s.
  [[nodiscard]] rtc::catching::Reason RunDecelLawTick(const ControllerState& state) noexcept;
  /// Freeze the reference state as DECEL's entry and take the τ = 0 step.
  [[nodiscard]] rtc::catching::Reason EnterDecel(const ControllerState& state) noexcept;
  /// Per-mode decisions split out of EvaluateReason (R-ORDER).
  [[nodiscard]] ReasonDecision EvaluateIdle(const ControllerState& state) noexcept;
  [[nodiscard]] ReasonDecision EvaluateRetreat() noexcept;
  [[nodiscard]] ReasonDecision EvaluateCommitted(const ControllerState& state) noexcept;
  [[nodiscard]] ReasonDecision EvaluateDecelOrHold(const ControllerState& state) noexcept;
  /// A CLIK failure this tick counts toward the n_qp fault latch.
  void NoteLawVerdict(rtc::catching::Reason law) noexcept;
  /// Edge actions of a mode change (freeze, hold stamp, retreat, re-arm).
  void OnModeEntered(rtc::catching::Mode prev) noexcept;
  /// The post-decision motion stage: the joint-space stop, homing and return
  /// (IDLE, RETREAT, ABORT_SAFE, FAULT). The CLIK modes moved in EvaluateReason.
  void RunArmMotion(const ControllerState& state) noexcept;
  void RunIdleMotion(const ControllerState& state) noexcept;
  void RunRetreatMotion(const ControllerState& state) noexcept;
  /// One homing-law tick toward the wait pose; true once the command and the
  /// measured arm are both there.
  [[nodiscard]] bool StepTowardWaitPose(const ControllerState& state) noexcept;
  /// The hand stage: sequencer Update (or the latch), after the motion stage.
  void RunHandStage(const ControllerState& state) noexcept;
  /// Copy the sequencer's last target into the hand latch, so handing the
  /// hand back to the latch does not snap it to an older pose.
  void LatchHandFromSequencer() noexcept;
  [[nodiscard]] bool ArmAtWaitPose(const ControllerState& state) const noexcept;
  [[nodiscard]] bool ArmCommandStopped() const noexcept;
  [[nodiscard]] bool HandSettledAtPre(const ControllerState& state) const noexcept;
  /// ‖q_meas − q_cmd‖ into `track_err_` (and this tick's record).
  void UpdateTrackError(const ControllerState& state) noexcept;
  /// Configure-time: resolve the supervisor's values and the sequencer. Non-RT.
  void SetupSupervisor();
  /// The first supervisor value a trial needs that is missing, or nullptr.
  [[nodiscard]] const char* SupervisorValueMissing() const noexcept;
  /// RETREAT → ARMED (L7 §4.8, S7.4). RT only, from OnModeEntered.
  void ResetForRearm() noexcept;
  /// The fingertip lane (L7 §4.4, S7.3): baseline in ARMED/TRACKING at q_pre,
  /// debounced contact from COMMITTED to HOLD, freshness (TIP_STALE) and the
  /// window [t_cmd, t_c + T_confirm]. Fed per SAMPLE (inference_sequence),
  /// not per tick. RT only, before the decision.
  void RunContactLane(const ControllerState& state) noexcept;
  /// The attempt's verdict at the end of HOLD (L7 §4.4).
  [[nodiscard]] rtc::catching::Outcome JudgeOutcome() const noexcept;

  /// One tick's worth of per-device command writing, shared by both axes.
  /// `latch` supplies the held command; an unreadable device is SILENCED
  /// (zero-length = "no update") rather than commanded to zeros.
  void WriteDeviceCommand(const ControllerState& state, ControllerOutput& output, int device_idx,
                          bool readable) noexcept;

  /// Emit the F5 gate-closure diagnostics for one axis. RT-3's throttle
  /// exception: primitive-only format, no assembly. Expanded per axis so each
  /// gets its own throttle window.
  void ReportGateClosure(const ControllerState& state, int device_idx, int dof,
                         const char* axis) noexcept;

  /// Per-device latch. `width` is the device's channel count at latch time and
  /// 0 while unlatched — one field answers both "is it latched" and "how wide".
  struct HoldLatch {
    std::array<double, kMaxDeviceChannels> commands{};
    int width{0};

    [[nodiscard]] bool IsLatched() const noexcept { return width > 0; }
  };

  /// Duck-typed by RegisterControllerLogs (reads `.instance` / `.msg_type`).
  struct ParsedLogEntry {
    std::string msg_type;
    std::string instance;
  };

  std::string urdf_path_;

  // ── Config (LoadConfig, non-RT) ──────────────────────────────────────────
  rtc::catching::CatchingParams params_{};
  rtc::catching::CatchingValidationReport report_{};
  bool hand_step_enabled_{false};
  bool catching_section_present_{false};
  std::vector<ParsedLogEntry> parsed_log_entries_;

  // ── Devices (ResolveDevices, non-RT) ─────────────────────────────────────
  int arm_dof_{0};
  int hand_dof_{0};
  bool device_configs_seen_{false};
  /// Set by on_configure, cleared by on_cleanup. See IsSimOnlyDisabled().
  bool sim_only_disabled_{false};
  CatchingParkReason park_reason_{CatchingParkReason::kNone};
  /// Which validator axis this configure used (A-S5-1). Decided from the
  /// claimed devices' backends, re-decided on every configure.
  bool real_arm_config_{true};
  std::vector<std::string> non_sim_groups_;
  std::vector<std::string> arm_joint_names_;
  std::vector<std::string> hand_joint_names_;
  std::vector<std::string> hand_motor_names_;
  std::array<std::vector<double>, ControllerState::kMaxDevices> device_position_lower_{};
  std::array<std::vector<double>, ControllerState::kMaxDevices> device_position_upper_{};
  std::array<std::vector<double>, ControllerState::kMaxDevices> device_max_velocity_{};

  // ── RT state ─────────────────────────────────────────────────────────────
  // Written and read by the RT tick only, except the latch clear in
  // ResetTargetInitialization (lifecycle thread, while no tick runs).
  // RT-OWNED BEGIN — tick state; every member here has a row in the reset table
  HoldLatch arm_hold_{};
  HoldLatch hand_hold_{};
  /// Last accepted step target, UNCLAMPED — the CSV's joint_goal lane. Kept
  /// apart from the clamped command so a clamp is visible in the log instead
  /// of silently rewriting what the operator asked for.
  std::array<double, kMaxDeviceChannels> hand_target_raw_{};
  int hand_target_width_{0};
  bool arm_readable_{false};
  bool hand_readable_{false};
  // RT-OWNED END

  // ── E-STOP / fault / arm request (non-RT writers, RT reader) ─────────────
  // Every one of these is a REQUEST. None of them is the state it asks about:
  // the tick owns that, and the tick is the only writer of anything these
  // lead to (P-1 (a)).
  //
  // The two epochs are counters rather than booleans so a trigger→clear pair
  // that lands entirely between two ticks is still seen: the tick compares
  // against the value it last serviced, so it observes "moved", not "is set".
  // RT-OWNED BEGIN — tick state; every member here has a row in the reset table
  std::atomic<bool> estop_requested_{false};
  std::atomic<std::uint32_t> estop_epoch_{0};
  /// Bumped by ResetTargetInitialization. Redundant with the activation
  /// generation in the path CM drives today, and kept anyway: it is the hook's
  /// own record that it ran, so a future caller that does NOT come with a
  /// generation bump still gets its reset instead of silently getting none.
  std::atomic<std::uint32_t> reset_requested_{0};
  std::atomic<std::uint32_t> fault_reset_epoch_{0};
  /// Raised by Compute() alone; the fault path is a latch, so nothing off the
  /// RT thread may lower it except through fault_reset_epoch_ (P-1 (d)).
  std::atomic<bool> fault_latched_{false};
  /// The operator arm latch (`catching.enable`). Written by the parameter
  /// callback (non-RT) AND by the tick, which lowers it on E-STOP or fault so
  /// that a clear cannot resume anything by itself (P-1 (c)).
  std::atomic<bool> arm_requested_{false};

  // Observation surface for off-RT readers (tests now, the S5.4 state message
  // later). Stored by the tick, read with relaxed loads: one writer, and a
  // diagnostic that is one tick stale is still the truth about a tick.
  std::atomic<std::uint8_t> mode_observed_{static_cast<std::uint8_t>(rtc::catching::Mode::kIdle)};
  std::atomic<std::uint8_t> reason_observed_{
      static_cast<std::uint8_t>(rtc::catching::Reason::kNone)};
  std::atomic<std::uint64_t> rt_reset_count_{0};
  std::atomic<std::uint64_t> estop_tick_count_{0};
  // RT-OWNED END

  // ── RT-owned supervisor state ────────────────────────────────────────────
  // Read and written by Compute() only. Not atomics: a second writer is
  // exactly what P-1 (a) forbids, so making them atomic would advertise a
  // sharing that must not exist.
  // RT-OWNED BEGIN — tick state; every member here has a row in the reset table
  rtc::catching::Mode mode_{rtc::catching::Mode::kIdle};
  rtc::catching::Reason last_reason_{rtc::catching::Reason::kNone};
  /// The epoch values the tick has already acted on.
  std::uint32_t serviced_estop_epoch_{0};
  std::uint32_t serviced_fault_reset_epoch_{0};
  std::uint32_t serviced_reset_epoch_{0};
  /// Set when a fault reset was actually performed, consumed by the next
  /// EvaluateReason so the FSM edge and the state reset land on one tick.
  bool fault_reset_serviced_{false};
  /// The activation generation the tick last reseeded for. D-23: a change here
  /// is how the tick learns an activation boundary was crossed, without
  /// waiting for a quiescence the lifecycle thread cannot establish.
  std::uint32_t serviced_activation_generation_{0};
  bool activation_seen_{false};
  /// G0-C verdict for the ACTIVE configuration, decided at configure. The tick
  /// reads it rather than the report so the arming question is one bool.
  bool armable_{false};
  /// True while this tick's E-STOP request is up. Read once at the top of
  /// Compute and then used by the target lane (drain → discard) and by the
  /// command writer (no step overlay), so one tick cannot act on two different
  /// answers.
  ///
  /// This is a SECOND layer and it knows it: while the global latch is up, CM
  /// substitutes its own hold for this controller's entire output, so nothing
  /// computed here reaches an actuator either way. It is kept because the
  /// layer doing the work belongs to a different component, and because
  /// discarding rather than queueing is what stops a step issued mid-stop from
  /// arriving late once the stop clears.
  bool estop_active_{false};

  std::atomic<std::uint64_t> arm_target_reject_count_{0};
  std::atomic<std::uint64_t> hand_step_disabled_reject_count_{0};
  std::atomic<std::uint64_t> hand_target_width_reject_count_{0};
  std::atomic<std::uint64_t> task_target_reject_count_{0};
  std::atomic<std::uint64_t> hand_step_applied_count_{0};
  // RT-OWNED END

  // ── Vision ingress (S5.2) ────────────────────────────────────────────────
  // The subscription is created by on_configure and lives on the controller
  // LifecycleNode's DEFAULT callback group, so it runs on the CM's non-RT
  // executor (G1-6). It is NOT declared in the `topics:` block: that block's
  // `role:` vocabulary is a closed set owned by rtc_controller_interface, and
  // a PointCloud2 prediction lane is not one of its roles. Adding a role for a
  // single consumer is the same kind of change E-11 refuses on the publish
  // side, so this follows the inference controller's precedent and subscribes
  // directly.
  //
  // It also stays alive while the controller is INACTIVE (lifecycle gates
  // publishers, not subscriptions), which is exactly why the snapshot carries
  // an activation generation (D-23).
  std::string traj_topic_;
  /// Compared against every message's `frame_id`. A publisher that changes
  /// frame mid-run keeps publishing entirely plausible numbers in a different
  /// space; S3.4 measured `world` on both robots, so this guards against a
  /// change rather than performing a conversion (L1 §4.3's transform is not
  /// implemented because nothing needs it).
  std::string expected_frame_{"world"};
  /// `catching.io.arm_base_frame` — the URDF frame the vision world is
  /// measured against — and `catching.io.base_T_world` (p_base = Rz(yaw)·p_world
  /// + t, the map tool's `--world-yaw-deg` / `--world-translation-m`). Empty
  /// frame ⇒ the vision frame is taken AS the model world (warned).
  std::string vision_base_frame_;
  double vision_yaw_deg_{0.0};
  std::array<double, 3> vision_translation_{0.0, 0.0, 0.0};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr traj_sub_;
  CatchingTrajInput traj_input_;

  /// Non-RT writer (the callback), RT reader (the tick). The payloads are
  /// trivially copyable PODs; the covariance rides its own lock because the
  /// tick never needs it and would otherwise copy 11.5 KB per cycle (A-3).
  rtc::SeqLock<rtc::catching::TrajectorySnapshot> traj_box_{};
  rtc::SeqLock<rtc::catching::CovarianceSnapshot> cov_box_{};

  /// RT-owned. The payload-side "have I seen this" memory D-21 requires
  /// (never SeqLock::sequence()). It carries the epoch and an explicit `seen`
  /// flag as well as the number — see `ConsumedToken` for why the number
  /// alone cannot express "nothing consumed yet".
  // RT-OWNED BEGIN — tick state; every member here has a row in the reset table
  rtc::catching::ConsumedToken consumed_{};
  std::uint64_t last_track_generation_{0};
  bool track_seen_{false};
  /// Latched on the tick a snapshot from a DIFFERENT track arrives, consumed
  /// by the next EvaluateReason. Latched rather than compared in place
  /// because the comparison is only meaningful on the arrival tick, while the
  /// supervisor may take another tick to act on it.
  bool traj_new_track_{false};
  rtc::catching::TrajView traj_view_{};
  // RT-OWNED END
  std::int64_t t_stale_ns_{0};
  std::int64_t t_arm_ns_{0};
  std::int64_t traj_horizon_min_ns_{0};
  /// L1 §4.4's J threshold [m]; <= 0 disables the warning.
  double traj_jump_warn_m_{-1.0};
  double track_err_abort_rad_{0.0};
  int n_qp_fault_{0};
  double limit_margin_{0.05};

  // ── Oracle plan (A-S5-8, the S5.3/S5.5 stand-in for the S6 planner) ──────
  std::string accel_limits_package_{"integrated_bringup"};
  std::string accel_limits_path_;
  std::string accel_limits_group_;
  std::vector<double> arm_qdd_max_;  // device order, from the derived file
  /// The ARM's position box after `limit_margin_`, device order — the same
  /// box handed to CLIK, cached here because the abort ramp needs it too.
  /// `JointSpaceDecelStep` documents its bounds as "the caller's box, already
  /// margined"; passing the raw device limits instead lets a stop integrate
  /// out to the limit CLIK was kept away from, and the backend then clamps it
  /// invisibly — the unattributable command/solution mismatch the margin was
  /// introduced to prevent. Empty when the box is incomplete, which is the
  /// same all-or-nothing rule CLIK's box follows.
  std::vector<double> arm_q_min_margined_;
  std::vector<double> arm_q_max_margined_;

  bool oracle_enabled_{false};
  std::array<double, 3> oracle_p_c_{};
  std::array<double, 3> oracle_a_d_{0.0, 0.0, -1.0};
  double oracle_t_c_offset_s_{1.0};
  double oracle_gamma_f_{0.3};

  // ── Arm command path (S5.3) ──────────────────────────────────────────────
  // The model is the COMBINED arm+hand one, because the catch frame hangs off
  // the hand: a frame on a link the arm-only model does not contain cannot be
  // registered, let alone driven.
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder_;
  CombinedModelCache combined_cache_;
  /// Indices into `cache().registered_frames`. −1 means "not registered", and
  /// for the base it also means "universe", which is what this robot uses.
  int catch_frame_idx_{-1};
  int base_frame_idx_{-1};
  std::string catch_frame_name_{"catch_frame"};
  int full_dof_{0};

  rtc::tsid::ClikReferenceGenerator clik_;
  /// False when anything the law needs is missing. The controller still
  /// configures and still holds the arm — a robot that cannot catch is not a
  /// robot that should refuse to start.
  bool clik_enabled_{false};
  Eigen::VectorXd q_posture_;  // [nq] posture target handed to CLIK
  Eigen::VectorXd q_eval_;     // [nq] evaluation state: q_c on the arm, measured on the hand
  Eigen::VectorXd v_eval_;     // [nv]
  /// D-16 box in PINOCCHIO order, or empty when the file did not provide one.
  Eigen::VectorXd qdd_max_pin_;

  /// L4's soft-catch translational reference. Constructed at configure because
  /// its parameters are validated in the constructor — an optional rather than
  /// a default-constructed member so "not configured" is a state the tick can
  /// see instead of a silently wrong gain set.
  std::optional<rtc::catching::SoftCatchTranslation> reference_;

  // ── RT-owned command state ───────────────────────────────────────────────
  /// The commanded arm configuration and its velocity, in DEVICE order. These
  /// are the controller's own carry-forward state: CLIK integrates its anchor
  /// internally, and these mirror it so the abort path can keep going when
  /// CLIK cannot.
  // RT-OWNED BEGIN — tick state; every member here has a row in the reset table
  std::array<double, kDemoCatchingMaxArmDof> arm_q_cmd_{};
  std::array<double, kDemoCatchingMaxArmDof> arm_qd_cmd_{};
  bool arm_cmd_seeded_{false};
  /// The L4 generator is reset from the measured TCP pose on the first tick of
  /// a plan — not at seed time, because the pose it needs comes from the model
  /// cache and the cache is only current inside the tick.
  bool reference_seeded_{false};
  /// Consecutive failed CLIK solves. `supervisor.n_qp` of them latch a fault.
  int qp_fail_streak_{0};
  /// Whether the joint-space stop has brought every joint to rest. The
  /// supervisor holds ABORT_SAFE until it has.
  bool abort_stopped_{false};
  /// ‖q_meas − q_cmd‖ of the last tick, the TRACK_ERR watchdog's input.
  double track_err_{0.0};
  // RT-OWNED END

  // ── S7 supervisor: configuration (SetupSupervisor, non-RT) ───────────────
  rtc::catching::HandSequencer hand_seq_;
  /// The sequencer owns the hand: configured, and `diagnostic.hand_step` off.
  bool hand_seq_enabled_{false};
  /// A trial can run: the law is wired and every value below is present.
  bool trials_enabled_{false};
  std::array<double, kDemoCatchingMaxArmDof> wait_pose_{};
  double pose_tol_{0.02};
  double homing_v_max_{0.5};
  double homing_eta_a_{0.5};
  double homing_qd_tol_{0.02};
  double decel_a_dec_{0.0};
  std::int64_t t_hold_ns_{0};
  std::int64_t t_close_e2e_ns_{0};
  std::int64_t stale_committed_max_ns_{0};
  int sat_ticks_{5};

  // ── S7 supervisor: RT-owned state ────────────────────────────────────────
  // Every member here is in the reset table below.
  /// This tick's clock, read ONCE after the vision load (R-DECEL-ENTRY): the
  /// law, the sequencer and every edge judge the same instant. The plan
  /// admission alone reads its own (see Compute).
  // RT-OWNED BEGIN — tick state; every member here has a row in the reset table
  rtc::catching::NowReal tick_now_{};
  rtc::catching::NowLead tick_now_lead_{};
  rtc::catching::HandSequencerOutput hand_out_{};
  bool homing_{false};       // IDLE: the homing law drives the arm
  bool homing_done_{false};  // IDLE: at the wait pose, hand told Ready
  enum class RetreatStage : std::uint8_t { kStop, kReturn, kRelease };
  /// kStop, then kReturn, then kRelease. The hand is released only on
  /// arrival at kRelease, whatever the verdict (#537 S7, 2026-09-24).
  RetreatStage retreat_stage_{RetreatStage::kStop};
  /// A plan was adopted this trial (an abort from here on is an attempt).
  bool trial_active_{false};
  bool trial_committed_{false};
  std::int64_t committed_t_c_ns_{0};
  std::int64_t committed_t_cmd_ns_{0};
  /// R-TRACK (Q15): the track the frozen plan is about, and the last snapshot
  /// of it — sampled after the freeze whatever the box holds.
  std::uint64_t committed_generation_{0};
  rtc::catching::TrajectorySnapshot law_snapshot_{};
  /// R-TRACK (Q15): the previous trial's track, refused by ARMED until a new
  /// one arrives.
  std::uint64_t last_trial_generation_{0};
  bool last_trial_generation_valid_{false};
  rtc::catching::DecelEntryState decel_entry_{};
  std::int64_t decel_t_s_ns_{0};
  bool decel_stopped_{false};
  std::int64_t hold_entry_ns_{0};
  int sat_streak_{0};
  /// The law ran past the prediction's horizon this tick (recorded, COMMITTED
  /// and CLOSING continue on the extrapolation — R-ORDER).
  bool law_horizon_extrap_{false};
  rtc::catching::Outcome outcome_{rtc::catching::Outcome::kNone};
  // RT-OWNED END

  // ── S7.3 contact lane ────────────────────────────────────────────────────
  // Configuration (SetupSupervisor, non-RT).
  rtc::catching::ContactDebouncer contact_;
  bool contact_configured_{false};
  int tip_stride_{7};
  int contact_m_min_{2};
  int contact_n_baseline_min_{20};
  std::int64_t contact_t_stale_ns_{0};
  std::int64_t contact_t_confirm_ns_{0};
  // RT-owned (reset table below).
  static constexpr std::size_t kTips = rtc::catching::kMaxFingertips;
  /// The last sample of each fingertip already fed — the "is it new" memory.
  // RT-OWNED BEGIN — tick state; every member here has a row in the reset table
  std::array<std::uint64_t, kTips> tip_last_seq_{};
  /// Baseline samples learned this trial, per fingertip.
  std::array<int, kTips> tip_baseline_n_{};
  /// This tick's lane (recorded, and read by the decision).
  int tip_count_{0};
  std::array<double, kTips> tip_force_now_{};
  std::array<bool, kTips> tip_fresh_now_{};
  std::array<bool, kTips> tip_contact_now_{};
  int tip_confirmed_now_{0};
  bool tip_stale_now_{false};
  /// Inside [t_cmd, t_c + T_confirm]: m_min fingertips confirmed at once / a
  /// fingertip went stale.
  bool window_confirmed_seen_{false};
  bool window_stale_seen_{false};
  // RT-OWNED END

  // ── Reset table (L7 §4.8, G8-A2) ─────────────────────────────────────────
  // Every RT-owned member (the RT-OWNED ranges of this header), and which
  // reset puts it back. R = ResetForRearm (RETREAT → ARMED), T =
  // ResetTrialState (activation / E-STOP / explicit). "exempt" names why it
  // survives. test_catching_reset_table.py checks every member has a row;
  // test_catching_reset_probe.cpp checks the rows are true.
  // clang-format off
  //   arm_hold_, hand_hold_, hand_target_raw_, hand_target_width_     T
  //   mode_, last_reason_                        T (activation only; an E-STOP leaves the mode to the table)
  //   plan_, plan_active_, admitted_plan_        R, T
  //   reset_floor_ns_, planner_reset_epoch_      R, T (same place, C-7)
  //   arm_cmd_seeded_, arm_q_cmd_                T; exempt from R: the carried command IS the wait pose
  //   arm_qd_cmd_                                R, T
  //   reference_seeded_, traj_hint_              R, T
  //   qp_fail_streak_                            T; exempt from R (C-29: a retry cycle has no solve in it)
  //   track_err_                                 R, T
  //   abort_stopped_                             exempt: written on ABORT_SAFE entry
  //   consumed_, last_track_generation_, track_seen_, traj_new_track_, traj_view_
  //                                              T; exempt from R: the next ball is judged against them
  //   hand_seq_ (phase/commit)                   R (Ready), T (Deactivate)
  //   hand_out_                                  T; exempt from R: the hand stage rewrites it every tick
  //   homing_, homing_done_                      R (done), T
  //   retreat_stage_                             R, T
  //   trial_active_, trial_committed_, committed_t_c_ns_, committed_t_cmd_ns_, committed_generation_
  //                                              R, T
  //   law_snapshot_                              R, T
  //   last_trial_generation_, last_trial_generation_valid_   R writes, T clears
  //   decel_entry_, decel_t_s_ns_, decel_stopped_, hold_entry_ns_     R, T
  //   sat_streak_, law_horizon_extrap_           R, T
  //   outcome_                                   T (Aborted when an E-STOP ends an attempt);
  //                                              exempt from R: it reports the LAST attempt
  //   contact_                                   R, T
  //   tip_baseline_n_, window_confirmed_seen_, window_stale_seen_     R, T
  //   tip_last_seq_                              T; exempt from R: a sample fed before the re-arm is not new
  //   tip_count_, tip_force_now_, tip_fresh_now_, tip_contact_now_, tip_confirmed_now_, tip_stale_now_
  //                                              exempt: rewritten every tick
  //   tick_now_, tick_now_lead_, tick_record_, plan_in_, plan_refusal_, arm_readable_, hand_readable_,
  //   estop_active_                              exempt: rewritten every tick
  //   serviced_estop_epoch_, serviced_fault_reset_epoch_, serviced_reset_epoch_, fault_reset_serviced_,
  //   serviced_activation_generation_, activation_seen_
  //                                              exempt: the reset's own bookkeeping
  //   armable_                                   exempt: the configure's G0-C verdict, read-only on the tick
  //   oracle_plan_id_                            exempt: monotone across trials (JudgePlan (d) never repeats)
  //   estop_requested_, estop_epoch_, reset_requested_, fault_reset_epoch_, arm_requested_
  //                                              exempt: hook REQUESTS (P-1 (a)); the tick lowers arm_requested_
  //   fault_latched_                             cleared only by a fault reset (P-1 (d))
  //   mode_observed_, reason_observed_, rt_reset_count_, estop_tick_count_, arm_target_reject_count_,
  //   hand_step_disabled_reject_count_, hand_target_width_reject_count_, task_target_reject_count_,
  //   hand_step_applied_count_, plan_refusal_observed_, plan_admitted_count_, plan_replaced_count_
  //                                              exempt: observation counters
  //   planner wake eventfd                       exempt: never drained on any reset (C-7)
  // clang-format on

  // ── The tick record (S5.4, D-20 + L8 §5.2) ───────────────────────────────
  // ONE object feeds the CSV row and the state message, so the file and the
  // operator's screen cannot disagree about a tick. It is a member rather than
  // a tick-local so Compute()'s helpers can fill their own blocks without
  // threading it through five signatures — and it is ASSIGNED A FRESH
  // default-constructed value at the top of every tick, which is what makes
  // "not computed this tick" indistinguishable from "zero" by construction
  // (PROC-7) instead of by everybody remembering to clear their own fields.
  // RT-OWNED BEGIN — tick state; every member here has a row in the reset table
  CatchingDiagLogPod tick_record_{};
  // RT-OWNED END
  /// RT writer, publish-thread reader.
  rtc::SeqLock<CatchingDiagLogPod> catching_state_lock_{};
  /// Subscription writer, publish-thread reader. The counters advance on
  /// message arrival, so they cannot ride the per-tick record.
  rtc::SeqLock<CatchingIngressSnapshot> ingress_diag_box_{};
  /// Relative, so it resolves under the controller's own node namespace
  /// (`/<config_key>/catching_state`). Not a YAML key: one controller, one
  /// state topic, and a configurable name is a name two tools can disagree on.
  std::string catching_state_topic_{"catching_state"};
  /// Fingertip sensor names of the hand group — they NAME and COUNT the
  /// per-tip columns and wire arrays.
  std::vector<std::string> hand_sensor_names_;

  // ── The plan being followed ──────────────────────────────────────────────
  // Written only by AdoptPlan (RT), from a plan the box delivered and
  // JudgePlan admitted — the planner's or the oracle's, through one path.
  // RT-OWNED BEGIN — tick state; every member here has a row in the reset table
  rtc::catching::PlanSnapshot plan_{};
  bool plan_active_{false};
  int traj_hint_{0};  // L2 sampler cursor, reset when the snapshot changes
  // RT-OWNED END

  // ── Planner thread (S6-A, D-7, E-7 decision J) ───────────────────────────
  rtc::catching::PlannerParams planner_params_{};
  /// The iteration body the thread calls. Owned here so it outlives the
  /// thread (which holds a reference to it).
  rtc::catching::PlannerCycle planner_cycle_;
  /// RT writer (every tick), planner reader.
  rtc::SeqLock<rtc::catching::PlannerRtState> planner_rt_box_;
  /// ONE writer — the planner thread, or the RT's oracle stand-in, never both
  /// (a profile enabling both is parked). RT reader, every tick (D-21).
  rtc::SeqLock<rtc::catching::PlanSnapshot> plan_box_;
  /// Non-blocking eventfd. Written by the vision subscription (non-RT) on every
  /// accepted trajectory, drained by the planner thread. Created at the first
  /// configure that enables the planner and closed only in the destructor,
  /// after the thread is joined: closing it earlier would let a still-polling
  /// thread see a recycled fd number. Atomic because the subscription reads it
  /// from the executor thread.
  std::atomic<int> planner_wake_fd_{-1};
  /// Lives for ONE configuration: spawned at the first activation of a
  /// configure that enabled the planner, joined in on_cleanup (and before any
  /// re-bind in on_configure). A thread that outlived its configuration would
  /// resume under the next one — and if that one enables the oracle instead,
  /// the plan box would have two writers (SeqLock::Store is not RMW-safe:
  /// interleaved writers can leave the sequence odd and hang every Load).
  std::unique_ptr<CatchingPlannerThread> planner_thread_;
  /// The per-wake timing ring, owned HERE rather than by the thread so the
  /// 1 Hz drain timer never dereferences planner_thread_ (which is joined and
  /// replaced across configurations while the timer may be mid-callback).
  CatchingPlannerThread::TimingBuffer planner_timing_{};
  /// Per-wake planner records → planner_events.csv (decision E). Owned here
  /// for the same reason as the timing ring.
  CatchingPlannerThread::EventQueue planner_events_{};
  std::ofstream planner_events_file_;
  /// `planner.ik.*` / `planner.catchability.*` — the same parser and keys the
  /// offline catchability map uses (S3.5a), so map and runtime solve alike.
  rtc::catching::CatchPoseIkConfig catch_pose_ik_config_{};
  /// The planner thread's OWN model handle on `planner.sub_model` (R-3,
  /// thread-per-handle). Replaced only while no planner thread exists.
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> planner_handle_;
  /// `planner.freeze.T_freeze` in ns (0 = no freeze), the RT's own defence
  /// against replacing a plan inside the freeze window (decision G).
  std::int64_t plan_freeze_ns_{0};
  /// Launch layout profile dropped the `mpc` role's core (#350). Read at
  /// configure; see SetLayoutProfile.
  bool layout_profile_drops_mpc_{false};
  rtc::ThreadTimingCsvLogger<rtc::RtTickTimingPayload> planner_timing_logger_;
  rclcpp::CallbackGroup::SharedPtr planner_timing_cb_group_;
  rclcpp::TimerBase::SharedPtr planner_timing_timer_;

  // RT-owned plan lane state (Compute() only).
  /// This tick's Load of the plan box (D-21: once per tick, unconditionally).
  // RT-OWNED BEGIN — tick state; every member here has a row in the reset table
  rtc::catching::PlanSnapshot plan_in_{};
  rtc::catching::PlanRefusal plan_refusal_{rtc::catching::PlanRefusal::kInvalid};
  /// Payload-side memory of the plan last adopted (never SeqLock::sequence()).
  rtc::catching::AdmittedPlan admitted_plan_{};
  /// Steady instant of the last trial reset — plans published before it
  /// belong to the trial it ended (JudgePlan (f)).
  std::int64_t reset_floor_ns_{0};
  /// Bumped on every trial reset and carried to the planner in PlannerRtState.
  std::uint32_t planner_reset_epoch_{0};
  /// The oracle stand-in's own monotone plan id (A-S5-8).
  std::uint32_t oracle_plan_id_{0};
  std::atomic<std::uint8_t> plan_refusal_observed_{
      static_cast<std::uint8_t>(rtc::catching::PlanRefusal::kInvalid)};
  std::atomic<std::uint64_t> plan_admitted_count_{0};
  std::atomic<std::uint64_t> plan_replaced_count_{0};
  // RT-OWNED END

  // ── Controller-owned topics (`topics:` block) ────────────────────────────
  // The hand step arrives on the group's `joint_goal`, which only exists if
  // CreateOwnedTopics runs — the YAML entry alone creates no endpoint. Since
  // S5.4 it also holds the catching state publisher (D-20), which is why this
  // controller has an on_activate/on_deactivate pair: a lifecycle gate applies
  // to publishers, and an inactive one drops every message while returning
  // normally.
  ControllerTopicHandles owned_topics_;

  // ── Logging (Phase C `logs:` block) ──────────────────────────────────────
  /// `<session>/controllers/<key>/` for the tick record AND planner_events.csv.
  static constexpr const char* kCatchingLogKey = "demo_catching_controller";
  rtc::ControllerLogSet log_set_{kCatchingLogKey};
  rtc::LogHandle<DeviceStateLogPod> arm_state_log_handle_;
  rtc::LogHandle<DeviceStateLogPod> hand_state_log_handle_;
  rtc::LogHandle<CatchingDiagLogPod> catching_diag_log_handle_;
  rclcpp::CallbackGroup::SharedPtr log_drain_cb_group_;
  rclcpp::TimerBase::SharedPtr log_drain_timer_;
  std::uint64_t log_drops_reported_{0};

  /// Kept alive for as long as the parameter channel exists — dropping the
  /// handle silently unregisters the callback, which would leave a
  /// `catching.enable` the operator can set and the tick never sees.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr arm_param_cb_handle_;

  rclcpp::Logger logger_{rclcpp::get_logger("integrated_bringup.demo_catching_controller")};
  rclcpp::Clock log_clock_{RCL_STEADY_TIME};
};

}  // namespace integrated_bringup

#endif  // UR5E_BRINGUP_CONTROLLERS_DEMO_CATCHING_CONTROLLER_H_
