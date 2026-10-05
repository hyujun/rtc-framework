#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "integrated_bringup/controllers/hand_sensor_layout.hpp"
#include "integrated_bringup/support/controller_log_registration.hpp"
#include "integrated_bringup/support/owned_topics.hpp"
#include "rtc_base/logging/session_dir.hpp"
#include "rtc_base/threading/thread_utils.hpp"
#include "rtc_controllers/catching/catch_pose_ik_batch.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/parameter.hpp>

#include <sys/eventfd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace integrated_bringup {

namespace {

/// Human-readable tag for one validation entry. Static strings only — this
/// runs in a lifecycle callback, but the report itself is allocation-free by
/// contract and there is no reason to make its rendering the exception.
[[nodiscard]] const char* ReasonText(rtc::catching::CatchingValidationReason reason) noexcept {
  using R = rtc::catching::CatchingValidationReason;
  switch (reason) {
    case R::kActiveConfigTbd:
      return "still TBD in the active configuration";
    case R::kRemovedValue:
      return "holds a value that no longer exists";
    case R::kControlRateOutOfRange:
      return "control_rate out of range";
    case R::kRangeViolation:
      return "value outside its documented range";
    case R::kZetaNotCriticallyDamped:
      return "reference.zeta must be exactly 1 in v1";
    case R::kEtaVOutOfRange:
      return "eta_v must be in (0, 1]";
    case R::kDecelExceedsAMax:
      return "a_dec exceeds reference.a_max";
    case R::kUnstableDiscretization:
      return "omega*h is at or above the discrete stability limit";
    case R::kDiscretizationAccuracy:
      return "omega*h exceeds the accuracy recommendation";
    case R::kHandCagingGapTooSmall:
      return "caging joint moves by less than rho_eps between q_pre and q_close";
    case R::kProvisionalOnRealArm:
      return "provisional value blocks a real-arm configuration";
    case R::kProvisionalWarning:
      return "provisional value (sim only)";
    case R::kWeightOrdering:
      return "CLIK weight ordering broken (w_task/w_a must dominate w_arm, w_arm must dominate "
             "damping_sq)";
    case R::kCloseTimeoutNotAboveE2e:
      return "T_close_timeout must exceed T_close_e2e";
    case R::kReleaseTimeoutNotAboveE2e:
      return "T_release_timeout must exceed T_close_e2e";
    case R::kFreezeShorterThanClose:
      return "T_freeze is shorter than T_close_e2e + T_arm + one tick — the close command would "
             "be due before the commit";
  }
  return "unknown";
}

/// Whether a validation entry names a key this controller actually consumes.
///
/// G0-C (L0 §5.3) blocks ARMING on a TBD active key. This controller does arm
/// now, so the gate matters — but the key set it consumes at S5.1 is still
/// small, and refusing to configure over a value that a LATER step decides
/// would invert the dependency the plan's own DAG records (S4.2 → S4.4 →
/// S3.5b: the measurement chain this controller feeds is what decides
/// `reference.v_max`, `supervisor.decel.a_dec` and the planner's values).
///
/// So the gate is the consumed subset and everything else is reported as a
/// warning naming the step that owns it. The subset grows with the
/// controller: S5.2 adds `io.*` / `prediction.*`, S5.3 adds `joint_cmd.*`,
/// `reference.*` and `robot.arm.*`, S6 adds `planner.*`, S7 the rest of
/// `supervisor.*`.
///
/// `robot.hand.T_close_e2e` is excluded BY NAME: it is the number S4.2
/// produces with this controller, so gating the CONFIGURE on it would make the
/// measurement its own precondition. `T_close_timeout` and
/// `T_release_timeout` (TBD exactly when T_close_e2e is, being derived from
/// it) are excluded with it. Their consumer
/// from S7.1 is the hand sequencer, and SupervisorValueMissing() parks a
/// configuration whose law is wired but whose closure time is not decided —
/// the step rig (`diagnostic.hand_step`) needs neither.
[[nodiscard]] bool ConsumedByCatchingSkeleton(const char* key) noexcept {
  const std::string_view k{key};
  if (k == "control_rate") {
    return true;
  }
  // S5.2's vision ingress and S5.3's tracking law. Each prefix joined this set
  // in the step that started reading it — the gate and the code that consumes
  // the value move together, or the gate stops meaning anything.
  if (k.starts_with("io.") || k.starts_with("prediction.") || k.starts_with("joint_cmd.") ||
      k.starts_with("robot.arm.") || k.starts_with("reference.") ||
      k == "supervisor.track_err_abort" || k == "supervisor.n_qp") {
    return true;
  }
  // #537 S9b (D-S9-D1): the motion deadlines, and — without a trailing dot,
  // like `robot.hand` below — their block's provisional flag, which must be
  // consumed for the real-arm gate to see it.
  if (k == "supervisor.deadline" || k.starts_with("supervisor.deadline.")) {
    return true;
  }
  if (k == "sim.io.future_tol") {
    return true;
  }
  // Same shape as `robot.hand` below: `reference` WITHOUT a trailing dot is
  // the block-wide provisional flag's own key (L4 §6, L0 §5.3).
  if (k == "reference") {
    return true;
  }
  // `robot.hand` WITHOUT a trailing dot is the provisional flag's own key
  // (catching_params.cpp reports the profile as a whole, not a field of it).
  // Matching only the dotted prefix silently let a provisional hand profile
  // through the real-arm gate — the one thing L0 §5.3 asks this gate to stop.
  if (k == "robot.hand") {
    return true;
  }
  // The catch frame's flag (D-17). on_configure reports it only when a model
  // is configured — exactly when the tracking law reads the frame — so the key
  // is consumed whenever it appears. Left out of this set, a provisional frame
  // on a real arm ends as a "not consumed" warning and the arm goes on to arm.
  if (k == rtc::catching::kCatchFrameProvisionalKey) {
    return true;
  }
  return k.starts_with("robot.hand.") && k != "robot.hand.T_close_e2e" &&
         k != "robot.hand.T_close_timeout" && k != "robot.hand.T_release_timeout";
}

/// Whether the catch frame still waits for the user's confirmation (D-17), or
/// nullopt when no model is configured: the arm is then held, not driven
/// (SetupArmCommand), nothing reads the frame, and there is nothing to judge.
/// The condition is AcquireModelBuilder's own.
///
/// With a model, a catch frame that is not an `urdf.extra_frames` entry has no
/// flag to read and counts as provisional. Silence does not prove the user
/// looked at the frame — the rule `real_arm_config_` follows for the backends.
[[nodiscard]] std::optional<bool> CatchFrameProvisional(const rtc_urdf_bridge::ModelConfig* cfg,
                                                        std::string_view name) {
  if (cfg == nullptr || cfg->urdf_path.empty()) {
    return std::nullopt;
  }
  for (const auto& frame : cfg->extra_frames) {
    if (frame.name == name) {
      return frame.provisional;
    }
  }
  return true;
}

/// Read-only descriptor for the mirrored profile parameters. They exist so an
/// off-process reader (the S4.2 runner, the GUI panel) uses what the CONTROLLER
/// loaded instead of re-reading the YAML — a run whose controller read
/// something else must not be analysed against the file on disk.
[[nodiscard]] rcl_interfaces::msg::ParameterDescriptor ReadOnlyDescriptor(std::string description) {
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = std::move(description);
  d.read_only = true;
  return d;
}

}  // namespace

rtc::catching::SoftCatchTranslation::Params DemoCatchingController::ResolvedReferenceParams()
    const {
  // A TBD a_max / v_max runs the struct default (L4 §6) — the one place that
  // says so, read by the reference's setup and by the mirror alike.
  rtc::catching::SoftCatchTranslation::Params p;
  p.omega = params_.reference_omega.value;
  p.zeta = params_.reference_zeta.value;
  p.a_max = params_.reference_a_max.tbd ? p.a_max : params_.reference_a_max.value;
  p.v_max = params_.reference_v_max.tbd ? p.v_max : params_.reference_v_max.value;
  return p;
}

int DemoCatchingController::ResolvedTrajNMin() const {
  // The decode's floor when the key is TBD: two points make one interval.
  // Mirrored as run (like the other resolvers) — a 0 in run_meta.json would
  // read as a condition value, not as "absent".
  return params_.io_n_min > 0 ? params_.io_n_min : 2;
}

double DemoCatchingController::ResolvedPlannerEtaV() const {
  // A TBD η_v runs the planner's own default (D-9) — as above, one place.
  return params_.planner_gamma_eta_v.tbd ? rtc::catching::GridCatchSearchConstants{}.eta_v
                                         : params_.planner_gamma_eta_v.value;
}

void DemoCatchingController::DeclareProfileParameters() {
  if (!node_) {
    return;
  }
  const auto& hand = params_.hand;
  const auto dof = static_cast<std::size_t>(hand.dof);

  std::vector<double> q_open(hand.q_open.begin(), hand.q_open.begin() + dof);
  std::vector<double> q_pre(hand.q_pre.begin(), hand.q_pre.begin() + dof);
  std::vector<double> q_close(hand.q_close.begin(), hand.q_close.begin() + dof);
  std::vector<bool> caging(hand.caging_mask.begin(), hand.caging_mask.begin() + dof);

  // Guarded, like every sibling binding's parameters.cpp. These are read_only,
  // so on_cleanup CANNOT undeclare them — a configure → cleanup → configure on
  // the same LifecycleNode would otherwise throw ParameterAlreadyDeclaredException
  // and turn a legal re-configure into a configure failure.
  const auto declare = [this](const std::string& name, const auto& value, std::string description) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter(name, value, ReadOnlyDescriptor(std::move(description)));
    }
  };

  declare("hand.q_open", q_open, "L6 §5.1 open pose [rad]");
  declare("hand.q_pre", q_pre, "L6 §5.1 preshape pose [rad]");
  declare("hand.q_close", q_close, "L6 §5.1 closed pose [rad]");
  declare("hand.caging_mask", caging, "L6 §4.2 caging joint set C");
  declare("hand.eta_close", hand.eta_close.value, "L6 §4.2 closure threshold eta");
  declare("hand.rho_eps", hand.rho_eps, "L6 §4.2 caging gap floor [rad]");
  declare("hand.T_close_e2e", hand.T_close_e2e.value, "L6 §4.2 end-to-end closure time [s]");
  // What RETREAT actually waits (S8-C): usually DERIVED from the poses, η and
  // q_tol, so the YAML alone does not show it.
  declare("hand.T_release_timeout", hand.T_release_timeout.value,
          "RETREAT wait for the hand at q_pre [s] (derived when the key is absent)");
  // The RT tick period the analyser needs for its tick axis and its dropped-row
  // test. It is mirrored here for the same reason as the poses: the off-process
  // reader must use what the CONTROLLER resolved, not a constant. The runner
  // cannot read `control_rate` itself — that parameter lives on the CM's node,
  // not on this per-controller one — and a hard-coded 0.002 silently scales the
  // tick axis and widens the drop gate on any bring-up that is not 500 Hz.
  declare("control.dt", GetDefaultDt(), "RT tick period [s] = 1/control_rate");
  declare("diagnostic.hand_step", hand_step_enabled_, "accept unshaped hand step targets (S4a)");

  // The S8 trial runner's inputs (S8-A). A sim overlay can move the
  // wait pose and switch the lead axis on, and the installed YAML cannot tell
  // the runner which overlay the controller loaded: a runner that homed to the
  // file's pose would refuse every trial of an overlay run, and a summary that
  // named the file's T_arm would attribute a lead-on run to lead-off.
  const auto wait_n = static_cast<std::size_t>(planner_params_.wait_pose_n);
  std::vector<double> wait_pose(planner_params_.wait_pose.begin(),
                                planner_params_.wait_pose.begin() + wait_n);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  declare("planner.wait_pose", wait_pose,
          "L3 §6 wait pose [rad], arm joint order — the YAML value. With "
          "planner.wait_pose_source 'current' the pose in force is the arm's switched-in pose "
          "(diag wait_pose_*, adoption log), not this");
  declare("planner.wait_pose_source",
          std::string(planner_params_.wait_pose_source ==
                              rtc::catching::PlannerParams::WaitPoseSource::kCurrent
                          ? "current"
                          : "yaml"),
          "L3 §6 where the wait pose comes from: 'yaml' (planner.wait_pose) or 'current' (the "
          "arm's pose on each activation's first readable tick, S8-I)");
  declare("planner.freeze.T_freeze", planner_params_.t_freeze, "L3 §4.11 commit lead [s]");
  // T_arm is mirrored whether or not the lead is on: the freeze-window check
  // reads it either way (C-25), so a lead-off run with T_arm 0.2 is a
  // different configuration from one with T_arm 0.
  declare("joint_cmd.lag.T_arm",
          params_.joint_cmd_lag_t_arm.tbd ? nan : params_.joint_cmd_lag_t_arm.value,
          "L5 §6 arm lag T_arm [s] (NaN = TBD)");
  declare("joint_cmd.lag.lead_enable", params_.joint_cmd_lag_lead_enable,
          "L5 §4.5 lead axis on (now_lead = now + T_arm)");
  // The arm-budget layers (dynamic_catching S8-G, #537): an overlay moves any
  // of them, and an off-process analysis must read what THIS controller
  // RUNS, not the file — the lesson the mirror exists for (S8-B: an overlay
  // one level short ran the shipped profile with no warning). A TBD leaf is
  // therefore mirrored as the value the setup substituted for it (the
  // reference's struct default, the planner's η_v default), through the same
  // resolvers SetupArmCommand / SetupGridCatchSearch use.
  declare("reference.omega", params_.reference_omega.value,
          "L4 §6 reference natural frequency ω [rad/s]");
  declare("reference.a_max", ResolvedReferenceParams().a_max,
          "L4 §6 reference acceleration limit [m/s²] as run (struct default when TBD)");
  declare("reference.v_max", ResolvedReferenceParams().v_max,
          "L4 §6 reference TCP speed limit [m/s] as run (struct default when TBD)");
  declare("planner.gamma.eta_v", ResolvedPlannerEtaV(),
          "L3 §4.5 speed margin η_v on v_max and the joint ratings (D-9) as run");
  declare("planner.time.margin", planner_params_.time_margin,
          "L3 §4.3 reach-time margin T_margin [s] the planner's gate uses");
  declare("robot.arm.qdd_max", arm_qdd_max_,
          "acceleration box the planner's reach time judges with [rad/s²], arm joint "
          "order (empty = no box loaded)");
  // The prediction grid the controller expects (E0-F04, #647). The
  // vision profile sets the grid and these three must follow it, but nothing
  // checks the pair: a `planner.slice.dt` left at 0.05 on a 25 ms grid thins
  // the candidates to every other point without a warning, and the run
  // measures the 50 ms grid under the other grid's name.
  declare("prediction.dt_expected",
          params_.prediction_dt_expected.tbd ? nan : params_.prediction_dt_expected.value,
          "expected vision prediction spacing [s] (NaN = TBD); the decode's spacing floor");
  declare("io.n_min", static_cast<std::int64_t>(ResolvedTrajNMin()),
          "fewest prediction points a message may carry, as run (the decode's default when TBD)");
  declare("planner.slice.dt", planner_params_.slice_dt,
          "L3 §4 candidate spacing [s]; the vision grid is thinned to it");
  // The two the E1-F10 tuning moves per arm (MD-72, MD-74): an
  // overlay one level short would otherwise run the shipped value under the
  // candidate's name. One launch configures once, which is what a unit driver
  // reads; like every mirror here they keep the FIRST configure's value.
  declare("planner.slice.t_lead_min", planner_params_.LeadMin(),
          "L3 §4 smallest candidate lead t_c - t_plan [s] as run (planner.freeze.T_freeze when "
          "the key is absent). As of the FIRST configure of this node — read_only mirrors "
          "cannot follow a re-configure");
  declare("joint_cmd.accel_constraint",
          std::string(rtc::catching::AccelConstraintName(params_.joint_cmd_accel_constraint)),
          "L5 §4.3 CLIK acceleration constraint form the profile selects (decision K) — the parsed "
          "key, also when the CLIK refused it and the arm is held. As of the FIRST configure of "
          "this node — read_only mirrors cannot follow a re-configure");
  // The decel MPC as run (MPC E1-F03): an off-process analysis must read the
  // horizon, window and thresholds this controller used, not the file.
  const auto& mpc_segment = planner_params_.mpc_segment;
  declare("planner.decel_mpc.horizon.n_nodes", static_cast<std::int64_t>(mpc_segment.n_nodes),
          "decel MPC nodes N_s; N_s * dt_s is the stopping time (MD-21)");
  declare("planner.decel_mpc.horizon.dt_s", mpc_segment.dt_s, "decel MPC node spacing dt_s [s]");
  declare("planner.decel_mpc.horizon.blocks",
          std::vector<std::int64_t>(mpc_segment.blocks.begin(),
                                    mpc_segment.blocks.begin() + mpc_segment.n_blocks),
          "decel MPC move-blocking pattern of the stop part (sum = n_nodes)");
  declare("planner.decel_mpc.m_q", mpc_segment.m_q,
          "decel MPC position margin inside the limits [rad]");
  declare("planner.decel_mpc.replan.k_max", static_cast<std::int64_t>(mpc_segment.k_max),
          "decel MPC post-catch replans at grid points k <= k_max (MD-31)");
  declare("planner.decel_mpc.eta_tau", mpc_segment.eta_tau,
          "decel MPC torque row fraction of tau_max");
  declare("planner.decel_mpc.publish.slack_max", mpc_segment.slack_max,
          "decel MPC publish threshold on the torque slack, nodes 1..N (MD-33)");
  declare("planner.decel_mpc.publish.slack_terminal_max", mpc_segment.slack_terminal_max,
          "decel MPC publish threshold on the terminal (static) torque slack (MD-33)");
  // The APPROACH–stop keys (MPC E1-F08): all provisional.
  declare("planner.decel_mpc.approach.n_pre_max", static_cast<std::int64_t>(mpc_segment.n_pre_max),
          "decel MPC pre-catch intervals before t_c, at most (MD-54); 0 = no decel planner "
          "(mode mpc parks, MD-70)");
  declare("planner.decel_mpc.approach.dt_pre_s", mpc_segment.dt_pre_s,
          "decel MPC pre-catch node spacing [s] (MD-54)");
  declare("planner.decel_mpc.approach.rest_tol", mpc_segment.rest_tol,
          "decel MPC first solve: largest |q_dot_cmd| read as at rest [rad/s]");
  declare("planner.decel_mpc.budget.first_s", mpc_segment.budget_first_s,
          "decel MPC first-segment solve budget and lead [s] (MD-56)");
  declare("planner.decel_mpc.budget.replan_s", mpc_segment.budget_replan_s,
          "decel MPC replan solve budget and lead [s] (MD-56)");
  declare("planner.decel_mpc.replan.same_point", mpc_segment.replan_same_point,
          "decel MPC re-solves a pre-catch grid point with the newer prediction (MD-58)");
  declare("planner.decel_mpc.publish.catch_pos_err_max", mpc_segment.catch_pos_err_max,
          "decel MPC publish threshold on the catch-node position error [m] (MD-62)");
  declare("planner.decel_mpc.catch.w_axis", mpc_segment.w_axis, "decel MPC approach-axis weight");
  declare("planner.decel_mpc.catch.w_v_par", mpc_segment.w_v_par,
          "decel MPC relative-velocity weight along the ball's travel");
  declare("planner.decel_mpc.catch.w_v_perp", mpc_segment.w_v_perp,
          "decel MPC relative-velocity weight across the ball's travel");
  declare("planner.decel_mpc.catch.gamma_ref", mpc_segment.gamma_ref,
          "decel MPC velocity target fraction of the ball's velocity (MD-53)");
  declare("planner.decel_mpc.catch.kappa", mpc_segment.kappa,
          "decel MPC position weight gain: W_p = kappa (Sigma_p + sigma_floor^2 I)^-1 (MD-63)");
  declare("planner.decel_mpc.catch.sigma_floor", mpc_segment.sigma_floor,
          "decel MPC position weight tracking-error floor [m]");
  declare("planner.decel_mpc.catch.w_max", mpc_segment.w_max,
          "decel MPC position weight eigenvalue cap [1/m^2]");
  declare("planner.decel_mpc.catch.w_const", mpc_segment.w_const,
          "decel MPC position weight without a usable covariance [1/m^2]");
  declare("planner.decel_mpc.catch.sigma_ref", mpc_segment.sigma_ref,
          "decel MPC w_delta schedule reference, compared with tr Sigma_p [m]");
  declare("planner.decel_mpc.catch.rho_v", mpc_segment.rho_v,
          "decel MPC relative-velocity slack penalty at the catch node; 0 = no slack row. The "
          "slack is recorded (planner_events decel_slack_v), never a publish gate");
  declare("planner.decel_mpc.catch.v_rel_allow", mpc_segment.v_rel_allow,
          "decel MPC per-axis relative velocity the hand absorbs [m/s]; read when rho_v > 0");
  // The core's own design values (YAML keys), as run. jerk_weight is the profile's
  // list in arm joint order; empty = the core's all-ones.
  declare("planner.decel_mpc.cost.jerk_weight", mpc_segment.jerk_weight,
          "decel MPC jerk weight R_j per arm joint (arm order); empty = all 1");
  declare("planner.decel_mpc.cost.u_scale", mpc_segment.u_scale,
          "decel MPC jerk scale [rad/s^3]; the jerk cost is (u/u_scale)^2");
  declare("planner.decel_mpc.cost.w_delta", mpc_segment.w_delta,
          "decel MPC pull toward the reference [1/rad^2]");
  declare("planner.decel_mpc.cost.rho_tau", mpc_segment.rho_tau,
          "decel MPC torque slack penalty; 0 = torque rows off (the publish slack condition is "
          "then vacuous)");
  declare("planner.decel_mpc.cost.w_perp", mpc_segment.w_perp,
          "decel MPC stop-path weight [1/m^2]: distance of the catch frame, from the catch on, "
          "from the line through the ball's predicted catch position along its travel; 0 = off, at "
          "most 1e4");
  declare("planner.decel_mpc.catch.axis_theta_max", mpc_segment.axis_theta_max,
          "decel MPC largest axis error of the reference the approach-axis term linearises at "
          "[rad]");
  declare("planner.decel_mpc.linearization.delta_tr", mpc_segment.delta_tr,
          "decel MPC trust-region half-width around the reference [rad]");
  declare("planner.decel_mpc.linearization.reference_rest_tol", mpc_segment.reference_rest_tol,
          "decel MPC bound on the supplied reference's terminal speed and acceleration");
  declare("planner.decel_mpc.linearization.ref_speed_fraction", mpc_segment.ref_speed_fraction,
          "decel MPC first-solve reference speed as a fraction of eta_v * qdot_max");
  declare("planner.decel_mpc.solver.max_iter",
          static_cast<std::int64_t>(mpc_segment.solver_max_iter),
          "decel MPC ProxQP outer iteration cap");
  declare("planner.decel_mpc.solver.max_iter_in",
          static_cast<std::int64_t>(mpc_segment.solver_max_iter_in),
          "decel MPC ProxQP inner iteration cap per outer step");
  declare("planner.decel_mpc.solver.eps_abs", mpc_segment.solver_eps_abs,
          "decel MPC ProxQP absolute tolerance");
  declare("planner.decel_mpc.solver.eps_rel", mpc_segment.solver_eps_rel,
          "decel MPC ProxQP relative tolerance");
  // #537 S9b (D-S9-D1): what the controller escalates on, as run — an overlay
  // can move either, and a FAULT is read against the value in force.
  declare("supervisor.deadline.stop_s", params_.supervisor_deadline_stop_s.value,
          "D-S9-D1 motion deadline for a stop (ABORT_SAFE ramp, RETREAT stop stage) [s]");
  declare("supervisor.deadline.return_s", params_.supervisor_deadline_return_s.value,
          "D-S9-D1 motion deadline for RETREAT's return to the wait pose [s]");
  // The DECEL law (MPC MD-44): one per configuration, never mixed. Like every
  // mirror here, read_only — it keeps the FIRST configure's value.
  declare(
      "supervisor.decel.mode",
      std::string(decel_mode_ == rtc::catching::CatchingDecelMode::kMpc ? "mpc" : "closed_form"),
      "MPC MD-44: the DECEL law — closed_form (v1 L7) or mpc (the decel MPC's stop segment). "
      "As of the FIRST configure of this node — read_only mirrors cannot follow a re-configure");
  declare("supervisor.decel.switch_margin", decel_switch_margin_,
          "MPC MD-39: rho_max of the decel segment switch gate (mode mpc). As of the FIRST "
          "configure of this node — read_only mirrors cannot follow a re-configure");
}

void DemoCatchingController::DeclareArmParameter() {
  if (!node_) {
    return;
  }
  // Read-WRITE, unlike the mirrored profile parameters: this one is an input.
  // Declared with `false` so a bring-up never starts armed.
  if (!node_->has_parameter(kCatchingEnableParam)) {
    rcl_interfaces::msg::ParameterDescriptor d;
    // What the tick lowers on an E-STOP or a fault is its own copy of the
    // request (arm_requested_, an atomic), not this parameter — a parameter
    // write is a ROS call the RT tick may not make. So the value can
    // read `true` on a disarmed controller, and the description has to say so
    // (#537 S9a finding).
    d.description =
        "Arm request for the catching supervisor (A-S5-3). Each write of true is a request; the "
        "RT tick drops it on E-STOP and on a latched fault (P-1 (c), no automatic resume) WITHOUT "
        "changing this value, so it can read true while disarmed — re-arm by setting true again.";
    node_->declare_parameter(kCatchingEnableParam, false, d);
  }
  // Mirror whatever the declaration left behind (a parameter override on the
  // command line can make that `true`), so the tick and the parameter agree
  // from the first tick rather than from the first CHANGE.
  arm_requested_.store(node_->get_parameter(kCatchingEnableParam).as_bool(),
                       std::memory_order_release);

  if (arm_param_cb_handle_) {
    return;  // a re-configure keeps the one callback; registering twice would double-handle
  }
  arm_param_cb_handle_ =
      node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& p : params) {
          if (p.get_name() != kCatchingEnableParam) {
            continue;
          }
          if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
            result.successful = false;
            result.reason = "catching.enable must be a bool";
            continue;
          }
          // The ONLY thing this callback does. Not a reset, not a mode change,
          // not a command — the tick owns all three (P-1 (a)). It runs on the
          // controller's non-RT callback group.
          arm_requested_.store(p.as_bool(), std::memory_order_release);
        }
        return result;
      });
}

void DemoCatchingController::ApplyArmAccelBox() {
  arm_qdd_max_.clear();
  arm_qdd_provisional_ = true;
  if (!arm_qdd_cfg_present_) {
    RCLCPP_WARN(logger_,
                "no `robot.arm.qdd_max`: the joint-space stop and homing ramps have no "
                "acceleration limit to run on (D-16)");
    return;
  }
  if (arm_qdd_cfg_malformed_ || static_cast<int>(arm_qdd_cfg_.size()) != arm_dof_) {
    RCLCPP_ERROR(logger_, "robot.arm.qdd_max must be a list of %d numbers (the arm's)", arm_dof_);
    return;
  }
  for (std::size_t i = 0; i < arm_qdd_cfg_.size(); ++i) {
    if (!std::isfinite(arm_qdd_cfg_[i]) || arm_qdd_cfg_[i] <= 0.0) {
      RCLCPP_ERROR(logger_, "robot.arm.qdd_max[%zu] is not a positive number", i);
      return;
    }
  }
  arm_qdd_max_ = arm_qdd_cfg_;
  arm_qdd_provisional_ = arm_qdd_provisional_cfg_;
  RCLCPP_INFO(logger_, "acceleration box: robot.arm.qdd_max (%zu joints%s)", arm_qdd_max_.size(),
              arm_qdd_provisional_ ? ", provisional" : "");
}

void DemoCatchingController::BuildClikBoxes(int nv,
                                            rtc::tsid::ClikReferenceGenerator::Config& cfg) {
  const auto& map = combined_cache_.ext_to_pin_v_map();
  const auto place = [&map, nv](int ext_idx, Eigen::VectorXd& dst, double value) {
    const int pv = map[static_cast<std::size_t>(ext_idx)];
    if (pv >= 0 && pv < nv) {
      dst[pv] = value;
    }
  };

  // ── Position box ────────────────────────────────────────────────────────
  // The device's own limits, pulled `limit_margin` inwards (L5 §4.3): the
  // backend clamps to the device limits, so a CLIK box that reached them would
  // let the solver return a value the backend then changes — and the
  // difference would show up as a tracking error with no attributable cause.
  //
  // The box is all-or-nothing by CLIK's contract, so ONE joint without a
  // finite limit drops it for every joint. That is the honest outcome: a
  // partial box would silently leave that joint unbounded while the operator
  // reads "position limits on".
  Eigen::VectorXd q_min = Eigen::VectorXd::Zero(nv);
  Eigen::VectorXd q_max = Eigen::VectorXd::Zero(nv);
  arm_q_min_margined_.assign(static_cast<std::size_t>(arm_dof_), 0.0);
  arm_q_max_margined_.assign(static_cast<std::size_t>(arm_dof_), 0.0);
  bool box_complete = true;
  for (int dev = 0; dev < 2 && box_complete; ++dev) {
    const int dof = (dev == kCatchingArmDeviceIdx) ? arm_dof_ : hand_dof_;
    const int base = (dev == kCatchingArmDeviceIdx) ? 0 : arm_dof_;
    const auto& lower = device_position_lower_[static_cast<std::size_t>(dev)];
    const auto& upper = device_position_upper_[static_cast<std::size_t>(dev)];
    if (static_cast<int>(lower.size()) < dof || static_cast<int>(upper.size()) < dof) {
      box_complete = false;
      break;
    }
    for (int i = 0; i < dof; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      if (!std::isfinite(lower[ui]) || !std::isfinite(upper[ui]) || !(lower[ui] <= upper[ui])) {
        box_complete = false;
        break;
      }
      // The margin narrows from BOTH sides but never inverts the box: a joint
      // whose whole range is narrower than twice the margin keeps its midpoint
      // rather than becoming an empty interval that Init would throw on.
      const double mid = 0.5 * (lower[ui] + upper[ui]);
      const double lo = std::min(lower[ui] + limit_margin_, mid);
      const double hi = std::max(upper[ui] - limit_margin_, mid);
      place(base + i, q_min, lo);
      place(base + i, q_max, hi);
      // The arm half is kept in device order too: the QP-independent abort
      // ramp (A-S5-10) has to clamp against the SAME box, and deriving it a
      // second time there would be a second copy of this formula to keep in
      // step. Filled here rather than in a separate loop so a future edit
      // cannot narrow one and not the other.
      if (dev == kCatchingArmDeviceIdx) {
        arm_q_min_margined_[ui] = lo;
        arm_q_max_margined_[ui] = hi;
      }
    }
  }
  if (box_complete) {
    cfg.q_min = q_min;
    cfg.q_max = q_max;
  } else {
    // All-or-nothing, and the abort ramp's copy goes with it: a half-filled
    // margined box would clamp some joints to the margin and leave the rest
    // at whatever the loop had reached when it bailed.
    arm_q_min_margined_.clear();
    arm_q_max_margined_.clear();
    RCLCPP_WARN(logger_,
                "device position limits incomplete — the CLIK position box is OFF (the backend "
                "clamp is then the only bound, and its clamp is invisible to the solver)");
  }

  // ── Per-joint velocity box ──────────────────────────────────────────────
  Eigen::VectorXd v_limit = Eigen::VectorXd::Zero(nv);
  bool v_complete = true;
  for (int dev = 0; dev < 2 && v_complete; ++dev) {
    const int dof = (dev == kCatchingArmDeviceIdx) ? arm_dof_ : hand_dof_;
    const int base = (dev == kCatchingArmDeviceIdx) ? 0 : arm_dof_;
    const auto& vmax = device_max_velocity_[static_cast<std::size_t>(dev)];
    if (static_cast<int>(vmax.size()) < dof) {
      v_complete = false;
      break;
    }
    for (int i = 0; i < dof; ++i) {
      const double value = vmax[static_cast<std::size_t>(i)];
      if (!std::isfinite(value) || value <= 0.0) {
        v_complete = false;
        break;
      }
      // THE HAND IS LOCKED IN THIS SOLVE. Its joints are decision variables —
      // the catch frame hangs off the palm, so they appear in the frame's
      // Jacobian — but this controller does not command them: the sequencer
      // does (L6, D-11). A solver allowed to use them would satisfy part of
      // the task with motion that never happens, and the arm would then
      // under-deliver by exactly that part. It shows up as a steady-state
      // position error that no gain fixes (measured 2.7 mm before this lock,
      // against the 1 mm G5-A asks for).
      //
      // Locked through the velocity box rather than by dropping the columns:
      // the box is the one place a joint can be told "you do not move" without
      // changing the problem's shape, and the hand's REAL motion still reaches
      // the solve every tick through the measured configuration.
      const bool is_hand = (dev != kCatchingArmDeviceIdx);
      place(base + i, v_limit, is_hand ? kLockedJointVelocity : value);
    }
  }
  if (v_complete) {
    cfg.v_limit_per_joint = v_limit;
  }
  clik_v_box_complete_ = v_complete;
}

bool DemoCatchingController::ConfigureAccelConstraint(
    int nv, rtc::tsid::ClikReferenceGenerator::Config& cfg) {
  using Form = rtc::catching::CatchingAccelConstraint;
  using Clik = rtc::tsid::ClikReferenceGenerator;
  switch (params_.joint_cmd_accel_constraint) {
    case Form::kUnset:
    case Form::kRemovedBox:
      // on_configure parks a profile that selects no form before the CLIK is
      // set up. This is the second line: a form must never be picked for a
      // profile that named none.
      RCLCPP_ERROR(logger_,
                   "joint_cmd.accel_constraint selects no form (kinematic or dynamic) — the arm "
                   "will be held");
      return false;
    case Form::kKinematic:
      cfg.accel_constraint = Clik::AccelConstraint::kKinematic;
      cfg.task_accel_max_linear = params_.joint_cmd_task_accel_max_linear.value;
      cfg.task_accel_max_angular = params_.joint_cmd_task_accel_max_angular.value;
      RCLCPP_INFO(logger_,
                  "CLIK acceleration constraint: kinematic (task rows ≤ %.3g m/s², %.3g rad/s²)",
                  cfg.task_accel_max_linear, cfg.task_accel_max_angular);
      return true;
    case Form::kDynamic:
      break;
  }
  // dynamic: τ_max is the ARM device's own `joint_limits.max_torque` — the
  // same numbers the D-16 derivation spent (provenance in the derived file).
  // The hand entries are never read (the rows sit on the arm indices).
  const auto* arm_cfg = GetDeviceNameConfig(GetPrimaryDeviceName());
  const std::vector<double>* torque = nullptr;
  if (arm_cfg != nullptr && arm_cfg->joint_limits.has_value()) {
    torque = &arm_cfg->joint_limits->max_torque;
  }
  if (torque == nullptr || static_cast<int>(torque->size()) < arm_dof_) {
    RCLCPP_ERROR(logger_,
                 "joint_cmd.accel_constraint is dynamic but the arm device has no "
                 "joint_limits.max_torque for its %d joints — the arm will be held",
                 arm_dof_);
    return false;
  }
  Eigen::VectorXd tau_max = Eigen::VectorXd::Zero(nv);
  const auto& map = combined_cache_.ext_to_pin_v_map();
  for (int i = 0; i < arm_dof_; ++i) {
    const double value = (*torque)[static_cast<std::size_t>(i)];
    const int pv = map[static_cast<std::size_t>(i)];
    if (!std::isfinite(value) || !(value > 0.0) || pv < 0 || pv >= nv) {
      RCLCPP_ERROR(logger_,
                   "joint_cmd.accel_constraint is dynamic but arm joint %d has max_torque %g "
                   "(or no model index) — the arm will be held",
                   i, value);
      return false;
    }
    tau_max[pv] = value;
  }
  cfg.accel_constraint = Clik::AccelConstraint::kDynamic;
  cfg.tau_max = tau_max;
  cfg.eta_tau = params_.joint_cmd_eta_tau.value;
  RCLCPP_INFO(logger_,
              "CLIK acceleration constraint: dynamic (|M·v̇ + h| ≤ %.2f·max_torque on %d arm "
              "joints; no rotor inertia in M)",
              cfg.eta_tau, arm_dof_);
  return true;
}

void DemoCatchingController::SetupArmCommand() {
  // Resolved HERE, not in the ingress setup: `limit_margin_` is read by the
  // box builder a few lines down, and a value set in another function is a
  // value whose ordering has to be remembered rather than seen.
  track_err_abort_rad_ =
      params_.supervisor_track_err_abort.tbd ? 0.0 : params_.supervisor_track_err_abort.value;
  n_qp_fault_ = params_.supervisor_n_qp;
  limit_margin_ = params_.robot_arm_limit_margin.tbd ? 0.05 : params_.robot_arm_limit_margin.value;

  clik_enabled_ = false;
  catch_frame_idx_ = -1;
  base_frame_idx_ = -1;
  // Before the early returns below: a reconfigure that takes one of them never
  // reaches ApplyArmAccelBox(), and the park check in on_configure must
  // not judge the box (or its flag) the PREVIOUS configure loaded.
  arm_qdd_max_.clear();
  arm_qdd_provisional_ = true;

  // The builder was acquired by SetupTrajInput (the vision frame needs the
  // model before the subscription exists); null means no model to drive.
  if (!builder_) {
    RCLCPP_WARN(logger_, "no system model: the arm will be held, not driven");
    return;
  }
  if (!combined_cache_.InitModel(*builder_, /*contact_frame_ids=*/{}, "[catching]", logger_)) {
    RCLCPP_ERROR(logger_, "combined model/cache init failed");
    return;
  }
  full_dof_ = arm_dof_ + hand_dof_;
  const auto* arm_cfg = GetDeviceNameConfig(GetPrimaryDeviceName());
  const auto* hand_cfg = GetDeviceNameConfig(GetSecondaryDeviceName());
  combined_cache_.BuildReorderMap(arm_cfg != nullptr ? &arm_cfg->joint_state_names : nullptr,
                                  hand_cfg != nullptr ? &hand_cfg->joint_state_names : nullptr,
                                  full_dof_, "[catching]", logger_);
  if (!combined_cache_.reorder_valid() || !combined_cache_.model()) {
    RCLCPP_ERROR(logger_, "joint reorder map is not valid — the arm will be held");
    return;
  }
  const auto& model = *combined_cache_.model();
  // CLIK integrates q with v, so the two must have the same dimension. A model
  // with a floating or continuous joint does not, and the failure would show
  // up as a command that drifts rather than as an error.
  if (model.nq != model.nv) {
    RCLCPP_ERROR(logger_, "model nq (%d) != nv (%d) — CLIK needs a reduced model", model.nq,
                 model.nv);
    return;
  }

  // The catch frame is an `urdf.extra_frames` entry (D-10/D-17) and is the
  // whole target of this controller: without it there is nothing to align.
  try {
    const auto frame_id = rtc::catching::ResolveCatchFrame(model, catch_frame_name_);
    catch_frame_idx_ = combined_cache_.cache().RegisterFrame(catch_frame_name_, frame_id);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "catch frame '%s' not in the model: %s — the arm will be held",
                 catch_frame_name_.c_str(), e.what());
    return;
  }
  if (catch_frame_idx_ < 0) {
    RCLCPP_ERROR(logger_, "catch frame registration refused (cache already locked?)");
    return;
  }

  rtc::tsid::ClikReferenceGenerator::Config cfg;
  BuildArmHandVelocityIndexSets(arm_dof_, full_dof_, model.nv, combined_cache_.ext_to_pin_v_map(),
                                cfg.arm_v_idx, cfg.hand_v_idx);
  if (cfg.arm_v_idx.empty()) {
    RCLCPP_ERROR(logger_, "no arm velocity indices resolved — the arm will be held");
    return;
  }
  cfg.damping_sq = params_.joint_cmd_damping_sq.value;
  cfg.w_task = params_.joint_cmd_w_task.value;
  cfg.w_axis = params_.joint_cmd_w_axis.value;
  cfg.w_arm = params_.joint_cmd_w_arm.value;
  cfg.w_hand = params_.joint_cmd_w_arm.value;
  cfg.w_smooth = params_.joint_cmd_w_smooth.value;
  cfg.max_iter = params_.joint_cmd_max_iter;
  // D-6: evaluate along the COMMANDED path. The measured state would fold the
  // servo lag into the loop and double-count it against the lead compensation
  // (L5 §4.2); `anchor_drift_max` must then stay off, and Init throws if it
  // does not — the tracking watchdog (TRACK_ERR) is what supervises the gap.
  cfg.evaluate_at_command = true;
  cfg.anchor_drift_max = 0.0;

  const int nv = model.nv;
  ApplyArmAccelBox();
  BuildClikBoxes(nv, cfg);
  if (!ConfigureAccelConstraint(nv, cfg)) {
    return;  // logged; the arm is held
  }
  try {
    clik_.Init(nv, cfg);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "CLIK init failed: %s — the arm will be held", e.what());
    return;
  }
  Eigen::Matrix<double, 6, 1> kx;
  const double k_p = params_.joint_cmd_k_p.value;
  kx << k_p, k_p, k_p, 0.0, 0.0, 0.0;  // rotation rows are the axis task's, not this one's
  clik_.SetTaskGain(kx);
  clik_.SetAxisGain(params_.joint_cmd_k_axis.value);
  clik_.SetPostureGains(params_.joint_cmd_k_posture.value, params_.joint_cmd_k_posture.value);

  q_posture_ = Eigen::VectorXd::Zero(model.nq);
  q_posture_segment_ = Eigen::VectorXd::Zero(model.nq);
  q_eval_ = Eigen::VectorXd::Zero(model.nq);
  v_eval_ = Eigen::VectorXd::Zero(nv);

  const rtc::catching::SoftCatchTranslation::Params ref_params = ResolvedReferenceParams();
  reference_.emplace(ref_params);
  if (!reference_->ParamsValid()) {
    RCLCPP_ERROR(logger_, "reference parameters rejected (omega/zeta/a_max/v_max)");
    reference_.reset();
    return;
  }

  clik_enabled_ = true;
  RCLCPP_INFO(logger_,
              "arm command path ready: nv=%d, catch frame '%s' (idx %d), acceleration "
              "constraint %s",
              nv, catch_frame_name_.c_str(), catch_frame_idx_,
              rtc::catching::AccelConstraintName(params_.joint_cmd_accel_constraint));
}

void DemoCatchingController::AcquireModelBuilder() {
  builder_.reset();
  const auto* sys_cfg = GetSystemModelConfig();
  if (sys_cfg == nullptr || sys_cfg->urdf_path.empty()) {
    return;  // SetupArmCommand reports what that means for the arm
  }
  // Prefer the builder CM injected so the URDF is parsed once for the whole
  // bring-up; build our own only when running outside CM (fixtures).
  if (auto shared = GetSharedModelBuilder()) {
    builder_ = std::move(shared);
    return;
  }
  try {
    builder_ = std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(*sys_cfg);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "model build failed: %s", e.what());
    builder_.reset();
  }
}

bool DemoCatchingController::ResolveVisionFrame(TrajInputConfig& cfg) {
  cfg.to_model = false;
  if (vision_base_frame_.empty()) {
    RCLCPP_WARN(logger_,
                "catching.io.arm_base_frame is not set: the vision frame '%s' is taken AS the "
                "model world. That is wrong on a robot whose URDF root is not the frame vision "
                "is measured against (L3 §4.2 — ur5e_p1b's root is base_link, 180° from base)",
                expected_frame_.c_str());
    return true;
  }
  if (!builder_) {
    // No model means no arm to drive (SetupArmCommand holds it) and no
    // planner model either — nothing downstream consumes model coordinates.
    RCLCPP_WARN(logger_,
                "catching.io.arm_base_frame '%s' is set but there is no robot model: the vision "
                "frame is left as is (the arm is held)",
                vision_base_frame_.c_str());
    return true;
  }
  const auto model =
      builder_->GetActuatedModel() ? builder_->GetActuatedModel() : builder_->GetFullModel();
  if (!model || !model->existFrame(vision_base_frame_)) {
    RCLCPP_ERROR(logger_, "catching.io.arm_base_frame '%s' is not a frame of the robot model",
                 vision_base_frame_.c_str());
    return false;
  }
  const auto& frame = model->frames[model->getFrameId(vision_base_frame_)];
  if (frame.parentJoint != 0) {
    // A frame behind a moving joint would make the transform a function of q.
    RCLCPP_ERROR(logger_,
                 "catching.io.arm_base_frame '%s' is not rigid to the model root (it hangs off "
                 "joint %zu)",
                 vision_base_frame_.c_str(), static_cast<std::size_t>(frame.parentJoint));
    return false;
  }
  // model_world_T_world = model_world_T_base · base_T_world. On the universe,
  // a frame's placement IS model_world_T_frame.
  const Eigen::Matrix3d r_mb = frame.placement.rotation();
  const Eigen::Vector3d t_mb = frame.placement.translation();
  const Eigen::Matrix3d r_bw =
      Eigen::AngleAxisd(vision_yaw_deg_ * M_PI / 180.0, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  const Eigen::Vector3d t_bw(vision_translation_[0], vision_translation_[1],
                             vision_translation_[2]);
  const Eigen::Matrix3d r = r_mb * r_bw;
  const Eigen::Vector3d t = r_mb * t_bw + t_mb;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      cfg.r_model_world[static_cast<std::size_t>(i * 3 + j)] = r(i, j);
    }
    cfg.t_model_world[static_cast<std::size_t>(i)] = t(i);
  }
  cfg.to_model = !(r.isIdentity(1e-12) && t.norm() < 1e-12);
  const double yaw = std::atan2(r(1, 0), r(0, 0)) * 180.0 / M_PI;
  RCLCPP_INFO(logger_,
              "vision frame '%s' → model world via '%s': yaw %.1f deg, t [%.3f %.3f %.3f] m%s",
              expected_frame_.c_str(), vision_base_frame_.c_str(), yaw, t.x(), t.y(), t.z(),
              cfg.to_model ? "" : " (identity — not applied)");
  return true;
}

void DemoCatchingController::SetupTrajInput() {
  // Resolve the ingress configuration from the parsed params. Every number
  // comes from the same `catching:` tree the validator judged, so a value that
  // reaches the wire decode is one the report has already had an opinion on.
  const auto to_ns = [](double seconds) { return static_cast<std::int64_t>(seconds * 1e9); };
  TrajInputConfig cfg;
  cfg.n_min = ResolvedTrajNMin();
  // The snapshot capacity, because that is the only upper bound this schema
  // carries: S3.6's 20 points is a property of the vision PROFILE (it is what
  // ball_perception's sim profile is set to), not a limit the controller is
  // given a key for. A longer message is accepted up to the capacity and
  // refused above it — the capacity is what the decode can physically hold,
  // and the profile change would show up as a point count in the diagnostics
  // rather than as a rejection.
  cfg.n_max = rtc::catching::kCap;
  if (!params_.prediction_dt_expected.tbd && params_.prediction_dt_expected.value > 0.0) {
    // The spacing floor is derived, not configured: a fraction of the expected
    // interval. Anything at or below this is not a denser prediction, it is a
    // malformed one, and the sampler's Hermite basis divides by h².
    cfg.dt_min_ns = std::max<std::int64_t>(
        1, to_ns(params_.prediction_dt_expected.value * kTrajSpacingFloorFraction));
  }
  const auto future_tol = rtc::catching::EffectiveFutureTol(params_, real_arm_config_);
  cfg.future_tol_ns = future_tol.tbd ? 0 : to_ns(future_tol.value);
  cfg.horizon_min_ns = params_.io_horizon_min.tbd ? 0 : to_ns(params_.io_horizon_min.value);
  cfg.track_eval_offset_ns =
      params_.io_track_eval_offset.tbd ? 0 : to_ns(params_.io_track_eval_offset.value);
  cfg.j_warn_m = params_.io_track_j_warn.tbd ? -1.0 : params_.io_track_j_warn.value;
  cfg.expected_frame = expected_frame_;
  AcquireModelBuilder();
  if (!ResolveVisionFrame(cfg)) {
    throw std::runtime_error("DemoCatchingController: the vision frame could not be resolved");
  }
  traj_input_.Configure(cfg);

  traj_horizon_min_ns_ = cfg.horizon_min_ns;
  traj_jump_warn_m_ = cfg.j_warn_m;
  t_stale_ns_ = params_.io_t_stale.tbd ? 0 : to_ns(params_.io_t_stale.value);
  // The lead axis (L5 §4.5). OFF unless the profile says otherwise: leading a
  // delay the arm does not have moves the command EARLY by exactly T_arm, and
  // the real arm's T_arm is unidentified until S10. The sim arm is a
  // first-order lag of the servo's kd/kp (0.05 s on ur5e_p1b, #537 D-S8-13),
  // and S8-B runs it with the lead on through the catch_lead sim overlays; the
  // G5-E fixture supplies its own pure delay.
  t_arm_ns_ = 0;
  if (params_.joint_cmd_lag_lead_enable && !params_.joint_cmd_lag_t_arm.tbd) {
    t_arm_ns_ = static_cast<std::int64_t>(params_.joint_cmd_lag_t_arm.value * 1e9);
  }

  if (!node_ || traj_topic_.empty()) {
    return;
  }
  // best_effort KEEP_LAST(1), measured rather than assumed: S3.4 compared a
  // best_effort subscription against a reliable one over 856 messages on two
  // robots and found them identical, including under 50 ms delay and 30 %
  // drop injection. Depth 1 is not a choice — ARCH-6 fixes it, and it is also
  // what G1-I asserts: under a backlog the controller must take the NEWEST
  // prediction, not work through a queue of old ones.
  rclcpp::QoS qos{rclcpp::KeepLast(1)};
  qos.best_effort();
  traj_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      traj_topic_, qos,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { OnTrajectoryCloud(*msg); });
  RCLCPP_INFO(logger_, "vision prediction lane: '%s' (best_effort, depth 1)", traj_topic_.c_str());
}

RTControllerInterface::CallbackReturn DemoCatchingController::on_configure(
    const rclcpp_lifecycle::State& prev, rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const YAML::Node& yaml) noexcept {
  const auto ret = RTControllerInterface::on_configure(prev, node, yaml);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  try {
    // CM calls SetDeviceNameConfigs between PreConfigure and here, so the hook
    // has already run with the groups resolved. A fixture that calls
    // on_configure directly sets the configs FIRST, so the hook then saw no
    // groups — re-run rather than trust the ordering (idempotent).
    ResolveDevices();
    // Owned by this function alone, and re-decided on every configure: a
    // reconfigure after the backends changed must not inherit the old verdict.
    sim_only_disabled_ = false;
    park_reason_ = CatchingParkReason::kNone;
    // Launch layout profile (#350): the planner thread runs on the `mpc` role
    // (E-7 J), so the same opt-out that stops DemoWbc's MPC thread stops it.
    if (node_) {
      SetLayoutProfile(ReadLayoutProfile(*node_));
    }

    if (topic_config_.groups.size() < 2) {
      RCLCPP_ERROR(logger_,
                   "refusing to configure: this controller claims an arm group and a hand group, "
                   "but `topics:` declares %zu",
                   topic_config_.groups.size());
      return CallbackReturn::FAILURE;
    }

    // ── Which axis this configure is judged on (A-S5-1) ──────────────────
    // S4.0 refused to ACTIVATE here unless every claimed device was the
    // simulator, standing in for the then-pending E-8 decision. That decision
    // was made on 2026-09-22, so the stand-in is gone and what is left is the
    // rule it was standing in for: a provisional or still-TBD value blocks a
    // REAL-ARM configuration and only warns in sim (L0 §5.3, L8 §5.4).
    //
    // ResolveDevices has already set `real_arm_config_` from the backends —
    // fail-closed, so a group whose config has not resolved yet counts as
    // unproven and gets the strict rule.
    if (real_arm_config_) {
      for (const auto& group : non_sim_groups_) {
        const auto* cfg = GetDeviceNameConfig(group);
        RCLCPP_INFO(logger_,
                    "device group '%s' is bound to backend '%s' (not '%s') — this configure is "
                    "judged as a REAL-ARM configuration: provisional and TBD values block it.",
                    group.c_str(),
                    (cfg != nullptr && cfg->backend.has_value()) ? cfg->backend->type.c_str()
                                                                 : "<none declared>",
                    kCatchingSimBackendType);
      }
      if (!device_configs_seen_) {
        RCLCPP_INFO(logger_,
                    "no device config resolved for any declared group — judged as a REAL-ARM "
                    "configuration (silence does not prove the simulator).");
      }
    }

    // ── Hand profile (G0-C, L0 §5.3) ─────────────────────────────────────
    if (!catching_section_present_) {
      RCLCPP_ERROR(logger_,
                   "refusing to configure: no `catching:` section. The hand profile has no "
                   "defensible default — see L6 §6.");
      return CallbackReturn::FAILURE;
    }
    // A key or a value that no longer exists parks, sim and real arm alike:
    // the old overlay must not silently run as something else. A FAILURE here
    // would make CM refuse every controller on the robot.
    // Every one is named, so one configure tells the operator all of it.
    const bool removed_box =
        params_.joint_cmd_accel_constraint == rtc::catching::CatchingAccelConstraint::kRemovedBox;
    if (!removed_arm_box_keys_.empty() || removed_box) {
      sim_only_disabled_ = true;
      park_reason_ = CatchingParkReason::kRemovedKey;
      for (const std::string& key : removed_arm_box_keys_) {
        RCLCPP_ERROR(logger_,
                     "DISABLED: 'catching.%s' was removed — set the acceleration box as "
                     "'catching.robot.arm.qdd_max' (rad/s², one per arm joint) and "
                     "'catching.robot.arm.qdd_provisional'. This controller will refuse to "
                     "activate; the robot still comes up.",
                     key.c_str());
      }
      if (removed_box) {
        RCLCPP_ERROR(logger_,
                     "DISABLED: 'catching.joint_cmd.accel_constraint: box' was removed — set the "
                     "key to kinematic or dynamic (the CLIK no longer takes the acceleration box; "
                     "'catching.robot.arm.qdd_max' stays, for the search and for the stop and "
                     "homing ramps). This controller will refuse to activate; the robot still "
                     "comes up.");
      }
      return CallbackReturn::SUCCESS;
    }
    report_ =
        rtc::catching::ValidateCatchingParams(params_, 1.0 / GetDefaultDt(), real_arm_config_);
    // The catch frame's provisional flag lives in the robot config
    // (`urdf.extra_frames`), not in `catching:`, so the validator above cannot
    // see it. It follows the rule of every other provisional flag: it parks a
    // real-arm configuration and warns in sim (L0 §5.3).
    if (const auto provisional = CatchFrameProvisional(GetSystemModelConfig(), catch_frame_name_);
        provisional.has_value()) {
      if (*provisional) {
        RCLCPP_WARN(logger_,
                    "catch frame '%s' is not confirmed: 'urdf.extra_frames.%s.provisional' is "
                    "true, or the frame is not an urdf.extra_frames entry and has no flag.",
                    catch_frame_name_.c_str(), catch_frame_name_.c_str());
      }
      rtc::catching::CheckCatchFrameProvisional(report_, *provisional, real_arm_config_);
    }
    // The DECEL law (MPC MD-44) — decided here, once, for the whole
    // configuration: the planner's setup below builds the decel cores only
    // for it, and the tick never changes it.
    decel_mode_ = params_.supervisor_decel_mode;
    decel_switch_margin_ = params_.supervisor_decel_switch_margin;
    if (stale_decel_mpc_disabled_key_ && decel_mode_ == rtc::catching::CatchingDecelMode::kMpc) {
      RCLCPP_WARN(logger_,
                  "catching.planner.decel_mpc.enabled: false is ignored — the key was removed. "
                  "The mpc law runs because supervisor.decel.mode is mpc; set "
                  "supervisor.decel.mode: closed_form to turn it off.");
    }
    // Written by SetupMpcSegmentPlanner only when this configure builds it.
    mpc_segment_planner_q_min_.fill(0.0);
    mpc_segment_planner_q_max_.fill(0.0);
    clik_v_box_complete_ = false;
    for (std::size_t i = 0; i < report_.warning_count; ++i) {
      const auto& w = report_.warnings[i];
      RCLCPP_WARN(logger_, "catching config warning: %s — %s", w.key, ReasonText(w.reason));
    }
    bool consumed_failure = false;
    // Whether every consumed failure is "not decided yet" (a TBD) rather than
    // "decided wrongly" (a range, ordering or consistency violation). Only the
    // first kind parks a SIM configuration — A-S5-12, see below.
    bool consumed_tbd_only = true;
    for (std::size_t i = 0; i < report_.failure_count; ++i) {
      const auto& f = report_.failures[i];
      // S6 joins `planner.*` to the consumed set — but only when the planner
      // runs: a profile with the planner off consumes none of it, and gating
      // on a value nothing reads is how a gate stops meaning anything.
      const bool planner_key =
          planner_params_.enabled && std::string_view(f.key).starts_with("planner.");
      if (!ConsumedByCatchingSkeleton(f.key) && !planner_key) {
        RCLCPP_WARN(logger_,
                    "catching config: %s — %s. Not consumed at this step; the step that owns "
                    "the value decides it.",
                    f.key, ReasonText(f.reason));
        continue;
      }
      consumed_failure = true;
      if (f.reason != rtc::catching::CatchingValidationReason::kActiveConfigTbd) {
        consumed_tbd_only = false;
      }
      if (f.index >= 0) {
        RCLCPP_ERROR(logger_, "catching config error: %s[%d] — %s", f.key, f.index,
                     ReasonText(f.reason));
      } else {
        RCLCPP_ERROR(logger_, "catching config error: %s — %s", f.key, ReasonText(f.reason));
      }
    }
    if (consumed_failure) {
      // A real-arm configuration is PARKED, not refused (A-S5-1). The failure
      // there is "this profile is not cleared for hardware", and refusing the
      // configure would take the whole robot down with it — sim and real share
      // `config/<variant>/controllers/`, and CM latches `bring_up_failed` on
      // ANY controller's configure failure and then refuses to configure EVERY
      // controller. Nothing is commanded until a controller is active, so an
      // instance that can never activate can never command.
      //
      // SIM PARKS TOO, for a TBD (A-S5-12, adopted 2026-09-23). The reason a
      // real arm parks — CM refuses EVERY controller when one fails configure —
      // holds in sim word for word, and it was observed there: on 2026-09-22 a
      // shipped sim profile whose consumed key was still TBD took the whole
      // robot down, and the symptom was "the robot does not start", not "the
      // catching controller does not start" (A-S5-11). A TBD is the
      // normal state of a key whose owning step has not landed yet, and every
      // step that widens the consumed set can produce one.
      //
      // A value that is decided and WRONG — out of range, weights out of
      // order, a caging joint that does not travel — still refuses the sim
      // configure. That is a configuration mistake with a fix available now,
      // and parking it would hide it behind a controller that quietly never
      // activates.
      if (real_arm_config_ || consumed_tbd_only) {
        sim_only_disabled_ = true;
        park_reason_ = CatchingParkReason::kConsumedValues;
        if (real_arm_config_) {
          RCLCPP_ERROR(logger_,
                       "DISABLED: real-arm configuration with values this controller consumes "
                       "that are provisional or still TBD (L0 §5.3). It will refuse to activate; "
                       "nothing was commanded.");
        } else {
          RCLCPP_ERROR(logger_,
                       "DISABLED: a value this controller consumes is still TBD (listed above). "
                       "The robot still comes up — this controller will refuse to activate "
                       "(A-S5-12). Decide the value to enable catching.");
        }
        return CallbackReturn::SUCCESS;
      }
      return CallbackReturn::FAILURE;
    }
    // Two writers for one plan box (S6-A). The oracle stand-in and the planner
    // both STORE into it, and a SeqLock with two writers can hand the RT a
    // torn plan. Parked rather than refused for the same reason as above: the
    // mistake is in this controller's profile, and it must not take the rest
    // of the robot down with it.
    if (planner_params_.enabled && oracle_enabled_) {
      sim_only_disabled_ = true;
      park_reason_ = CatchingParkReason::kPlannerOracleConflict;
      RCLCPP_ERROR(logger_,
                   "DISABLED: `planner.enabled` and `diagnostic.oracle_plan.enabled` are both "
                   "true — the plan box would have two writers. Turn one of them off. This "
                   "controller will refuse to activate; the robot still comes up.");
      return CallbackReturn::SUCCESS;
    }
    // The decel MPC (MPC E1-F03) runs on the planner thread, and its torque
    // box plus publish slack must fit inside the CLIK's (MD-33). A profile
    // mistake: park, name it, keep the robot up. Only under the law that runs
    // it and only with the planner on — closed_form builds no decel core and
    // reads none of its keys (MD-44), and a planner-less mpc profile is
    // DecelModeUnmet's to judge (the oracle profile is exempt there).
    if (planner_params_.enabled && decel_mode_ == rtc::catching::CatchingDecelMode::kMpc) {
      if (const char* why = MpcSegmentConfigInvalid(); why != nullptr) {
        sim_only_disabled_ = true;
        park_reason_ = CatchingParkReason::kMpcSegmentInvalid;
        RCLCPP_ERROR(logger_,
                     "DISABLED: supervisor.decel.mode is mpc but %s. This controller will "
                     "refuse to activate; the robot still comes up.",
                     why);
        return CallbackReturn::SUCCESS;
      }
    }
    // The planner's DECISION values (S6-B) — values nobody may guess. Same rule
    // as a consumed TBD: park, name the key, keep the robot up (A-S5-12).
    if (planner_params_.enabled) {
      if (const char* missing = PlannerDecisionMissing(); missing != nullptr) {
        sim_only_disabled_ = true;
        park_reason_ = CatchingParkReason::kPlannerUnset;
        RCLCPP_ERROR(logger_,
                     "DISABLED: planner.enabled is true but '%s' is unset or TBD — the planner "
                     "cannot guess it. This controller will refuse to activate; the robot still "
                     "comes up.",
                     missing);
        return CallbackReturn::SUCCESS;
      }
      if (real_arm_config_ && planner_params_.provisional) {
        sim_only_disabled_ = true;
        park_reason_ = CatchingParkReason::kConsumedValues;
        RCLCPP_ERROR(logger_,
                     "DISABLED: real-arm configuration with `planner.provisional: true` (L0 "
                     "§5.3). Nothing was commanded.");
        return CallbackReturn::SUCCESS;
      }
    }
    // One bool for the tick's arming question (§4.5-1).
    //
    // NOT `report_.armable`: that verdict covers every key the schema knows,
    // including the ones a LATER step decides (`reference.v_max`,
    // `planner.*`, `supervisor.decel.*`). Arming on it would mean this
    // controller can never arm until S7 is finished, which inverts the plan's
    // own DAG — the measurements that decide those values are taken WITH this
    // controller. The gate is the same consumed subset the configure refusal
    // uses, so the two cannot drift apart: what refuses a sim configure is
    // exactly what refuses to arm.
    armable_ = !consumed_failure;

    if (params_.hand.dof != hand_dof_) {
      RCLCPP_ERROR(logger_,
                   "refusing to configure: hand profile declares %d joints but the hand device "
                   "reports %d (L6 §5.1: n must equal the hand device's channel count)",
                   params_.hand.dof, hand_dof_);
      return CallbackReturn::FAILURE;
    }

    // ── Controller-owned CSVs ────────────────────────────────────────────
    // Instance keys are derived from the device names so a YAML `instance:`
    // tracks the active config_variant, exactly as the other bindings do.
    const auto arm_key = GetPrimaryDeviceName() + "_state";
    const auto hand_key = GetSecondaryDeviceName() + "_state";
    LogRegistrationContext ctx{
        .logger = logger_,
        .log_set = log_set_,
        .state_logs =
            {
                {arm_key, {arm_joint_names_, std::vector<std::string>{}}},
                {hand_key, {hand_joint_names_, hand_motor_names_}},
            },
    };
    // The per-tick record (S5.4). Always registered when the YAML asks for it:
    // unlike grasp_diag there is no optional block to gate on — every tick of
    // this controller produces a row, including the ones that decide to do
    // nothing, which is the whole of PROC-7.
    ctx.catching_diag_enabled = true;
    ctx.catching_diag_arm_joint_names = arm_joint_names_;
    ctx.catching_diag_tip_names = hand_sensor_names_;
    auto reg = RegisterControllerLogs(parsed_log_entries_, ctx);
    if (reg.status == LogRegistrationStatus::kMissingInstance) {
      ResetLogState();
      return CallbackReturn::FAILURE;
    }
    if (auto it = reg.handles.state.find(arm_key); it != reg.handles.state.end()) {
      arm_state_log_handle_ = it->second;
    }
    if (auto it = reg.handles.state.find(hand_key); it != reg.handles.state.end()) {
      hand_state_log_handle_ = it->second;
    }
    catching_diag_log_handle_ = reg.handles.catching_diag;

    // Drain timer on a non-RT callback group (10 Hz), same as every other
    // binding: the RT tick only pushes into the SPSC ring.
    if (!log_set_.empty() && node_) {
      log_drain_cb_group_ =
          node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      log_drain_timer_ = node_->create_wall_timer(
          std::chrono::milliseconds(100),
          [this]() { DrainControllerLogs(log_set_, logger_, log_drops_reported_); },
          log_drain_cb_group_);
    }

    // ── Controller-owned topics ──────────────────────────────────────────
    // Creates the hand group's `joint_goal` subscription declared in the
    // `topics:` block. Last, and only past every refusal above, so a
    // controller that is going to fail configure never exposes a step lane.
    // Without this the YAML entry is inert: the topic name parses, the group
    // shows up in topic_config_, and no endpoint is ever created — the step
    // then vanishes with no error anywhere (the step path is what S4.2
    // measures through, so it fails as silence, not as a failure).
    CreateOwnedTopics(*this, owned_topics_);
    // The state topic (D-20) sits directly under the controller's own
    // namespace rather than under a device group's: it describes the
    // CONTROLLER, not one device's lane, and the two groups it drives would
    // both have an equal claim to the prefix.
    SetupCatchingStatePublisher(*this, owned_topics_, catching_state_topic_, arm_joint_names_,
                                hand_sensor_names_);
    SetupTrajInput();
    // After the topics and before the parameters: the arm path needs the
    // device configs (resolved above) and it must not be half-built if a
    // later step throws — the catch block below tears everything down.
    SetupArmCommand();
    // After the devices are resolved (the planner checks the arm's width) and
    // before anything is declared, so a planner that cannot run fails the
    // configure before the node grows parameters.
    if (!SetupPlanner()) {
      TearDownConfiguredResources();
      return CallbackReturn::FAILURE;
    }
    SetupSegmentFollower();

    // The S7 supervisor's values (commit instant, wait pose, stop box, hand
    // sequencer). A configuration whose law is wired but which cannot run a
    // TRIAL is parked, exactly like a consumed TBD (A-S5-12): the robot comes
    // up, this controller refuses to activate, and the log names the value.
    SetupSupervisor();
    if (clik_enabled_ && !trials_enabled_) {
      const char* missing = SupervisorValueMissing();
      sim_only_disabled_ = true;
      park_reason_ = CatchingParkReason::kSupervisorUnset;
      RCLCPP_ERROR(logger_,
                   "DISABLED: the tracking law is wired but '%s' is unset or invalid — a trial "
                   "cannot run without it (S7 supervisor). This controller will refuse to "
                   "activate; the robot still comes up.",
                   missing != nullptr ? missing : "<unknown>");
      TearDownConfiguredResources();
      return CallbackReturn::SUCCESS;
    }
    // MPC MD-34 / MD-44 / MD-45: under mpc the arm follows a segment from
    // APPROACH to the end of the stop, so a missing prerequisite would leave
    // every trial without one. Parked like any other profile mistake: the
    // robot comes up, this controller does not activate.
    if (decel_mode_ == rtc::catching::CatchingDecelMode::kMpc) {
      if (const char* why = DecelModeUnmet(); why != nullptr) {
        sim_only_disabled_ = true;
        park_reason_ = CatchingParkReason::kDecelModeUnmet;
        RCLCPP_ERROR(logger_,
                     "DISABLED: supervisor.decel.mode is mpc but %s. This controller will refuse "
                     "to activate; the robot still comes up.",
                     why);
        TearDownConfiguredResources();
        return CallbackReturn::SUCCESS;
      }
      if (oracle_enabled_) {
        RCLCPP_WARN(logger_,
                    "supervisor.decel.mode is mpc under the oracle plan profile: no planner "
                    "publishes a stop segment here, so every DECEL aborts unless a test writes "
                    "the decel box");
      }
    }
    // #537 pre-S10 R3 (Q4): the box the search and every joint-space ramp run
    // on is still provisional. Judged only on a box that LOADED — an
    // absent one is the supervisor's verdict above — and after it, so the
    // more basic refusal is the one the operator reads. The flag has no
    // validator row in `catching:`, hence not a validator failure; the park is
    // the same one (L0 §5.3), and the log names the key.
    if (!real_arm_config_ && !arm_qdd_max_.empty() && arm_qdd_provisional_) {
      RCLCPP_WARN(logger_,
                  "catching config warning: robot.arm.qdd_provisional — the acceleration box "
                  "(robot.arm.qdd_max) is provisional (sim only); a real-arm configuration is "
                  "parked on it");
    }
    if (real_arm_config_ && !arm_qdd_max_.empty() && arm_qdd_provisional_) {
      sim_only_disabled_ = true;
      park_reason_ = CatchingParkReason::kConsumedValues;
      RCLCPP_ERROR(logger_,
                   "DISABLED: real-arm configuration with robot.arm.qdd_provisional true (or "
                   "absent) — the acceleration box (robot.arm.qdd_max) is not cleared for this "
                   "arm (L0 §5.3). It will refuse to activate; nothing was commanded.");
      TearDownConfiguredResources();
      return CallbackReturn::SUCCESS;
    }

    DeclareProfileParameters();
    DeclareArmParameter();

    RCLCPP_INFO(logger_,
                "configured: arm=%s(%d) hand=%s(%d), hand_step=%s, config=%s, armable=%s "
                "(arm it with the '%s' parameter)",
                GetPrimaryDeviceName().c_str(), arm_dof_, GetSecondaryDeviceName().c_str(),
                hand_dof_, hand_step_enabled_ ? "enabled" : "disabled",
                real_arm_config_ ? "real-arm" : "sim", armable_ ? "yes" : "no",
                kCatchingEnableParam);
  } catch (const std::exception& e) {
    // Undo everything this function may already have built. Without this a
    // throw past the log registration leaves a half-configured instance —
    // live log channels, a live 100 ms timer capturing `this`, and a live step
    // subscription — behind a FAILURE that the operator reads as "nothing
    // happened". CM then refuses the bring-up and destroys the node anyway,
    // but a fixture that retries configure would inherit the wreckage.
    RCLCPP_ERROR(logger_, "on_configure failed: %s", e.what());
    TearDownConfiguredResources();
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn DemoCatchingController::on_activate(
    const rclcpp_lifecycle::State& prev) noexcept {
  // The park check lives here, not in on_configure: refusing to configure took
  // the whole robot down with it (see the header). Nothing is commanded until
  // a controller is active, so refusing here is what the check actually needs
  // to do.
  if (sim_only_disabled_) {
    RCLCPP_ERROR(logger_,
                 "refusing to activate: this instance was parked at configure (%s). Nothing was "
                 "commanded — see the configure log for the values involved.",
                 park_reason_ == CatchingParkReason::kPlannerOracleConflict
                     ? "planner and oracle plan both enabled"
                 : park_reason_ == CatchingParkReason::kMpcSegmentInvalid
                     ? "planner.decel_mpc cannot run in this profile"
                 : park_reason_ == CatchingParkReason::kDecelModeUnmet
                     ? "supervisor.decel.mode mpc lacks a prerequisite"
                 : park_reason_ == CatchingParkReason::kRemovedKey
                     ? "the profile sets a removed key or value"
                     : "a consumed value is provisional or TBD");
    return CallbackReturn::FAILURE;
  }
  // Launch-profile gate (#350, E-7 J). Before the first side effect, for the
  // reason DemoWbc's gate gives: on a failed on_activate the controller
  // manager leaves the previous controller active and never calls
  // on_deactivate for this one, so anything done before a FAILURE leaks.
  // Under `mpc_off` the shield has handed the `mpc` role's core back to the
  // system cpuset, and the planner thread would put SCHED_FIFO on a core
  // arbitrary user work now shares.
  if (planner_params_.enabled && layout_profile_drops_mpc_) {
    RCLCPP_ERROR(logger_,
                 "DemoCatchingController on_activate refused: this launch ran with layout profile "
                 "'%s' (enable_mpc:=false), which returned the MPC cores to the system cpuset, "
                 "but this controller's config has planner.enabled: true. Activating would spawn "
                 "the planner (mpc_main, SCHED_FIFO) onto a core the CPU shield no longer "
                 "protects. Relaunch without enable_mpc:=false, or set planner.enabled: false.",
                 std::string(kMpcOffLayoutProfile).c_str());
    return CallbackReturn::FAILURE;
  }
  // Base first, always: it bumps the activation generation and calls
  // ResetTargetInitialization, and the generation bump is what the first tick
  // reads to know an activation boundary was crossed (D-23, #196 §3).
  const auto ret = RTControllerInterface::on_activate(prev);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  // An activation does NOT arm the controller (P-1 (c) in spirit, A-S5-3): the
  // operator arms it through `catching.enable`, and a controller that armed
  // itself on activation would resume catching after any deactivate/activate
  // cycle — including the one an E-STOP recovery goes through.
  arm_requested_.store(false, std::memory_order_release);
  // The accepted-sequence memory and the previous prediction belong to the
  // previous activation. Keeping them would refuse the first message of this
  // one (its sequence is below the old high-water mark after a vision
  // restart) and would compare this activation's first prediction against a
  // trajectory the operator has no reason to think is still relevant.
  traj_input_.Reset();
  // And republish what the reset left behind. Reset() puts `jump_m` back to
  // "not compared" and forgets the accepted sequence, but the BOX still holds
  // the previous activation's numbers — so without this the topic keeps
  // reporting a jump and an origin delay measured against a trajectory this
  // activation has no reason to think is still relevant.
  ingress_diag_box_.Store(traj_input_.Snapshot());
  // The state publisher has to be activated or every Store this activation
  // makes goes nowhere — a lifecycle gate applies to publishers, and an
  // inactive one drops the message while returning perfectly normally.
  ActivateOwnedTopics(prev, owned_topics_);
  // After the base bumped the activation generation, so the planner's first
  // wake of this activation can only publish against it.
  SpawnPlannerThreadIfNeeded();
  if (planner_thread_ && planner_params_.enabled) {
    planner_thread_->Resume();
  }
  if (node_ && node_->has_parameter(kCatchingEnableParam)) {
    // Bring the parameter back in line with the latch, so the value an
    // operator reads is the value the tick will act on. Without this the
    // parameter would still read `true` from a previous run while the tick is
    // disarmed — and the next thing anyone does is set it to true, which is a
    // no-op change that fires no callback.
    //
    // Guarded because this override is `noexcept` and `set_parameter` throws:
    // any OTHER on-set callback registered on this node may reject the change,
    // and an immutable-parameter or not-declared exception here would be
    // std::terminate rather than a failed activation. The latch above is what
    // actually governs the tick, so a parameter that could not be written back
    // is a display inconsistency to report, not a reason to take the process
    // down. (Lifecycle callback — non-RT, so RT-2's try/catch ban
    // does not apply here.)
    try {
      node_->set_parameter(rclcpp::Parameter(kCatchingEnableParam, false));
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_,
                  "could not write '%s' back to false on activation (%s) — the controller is "
                  "DISARMED regardless; the parameter's displayed value may disagree until it is "
                  "set again",
                  kCatchingEnableParam, e.what());
    }
  }
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn DemoCatchingController::on_deactivate(
    const rclcpp_lifecycle::State& prev) noexcept {
  // Stop the planner burning its core while this controller is inactive. A
  // wake already in flight may still publish once; the RT refuses it by its
  // activation generation (D-23), so Pause need not be synchronous.
  if (planner_thread_) {
    planner_thread_->Pause();
  }
  DeactivateOwnedTopics(prev, owned_topics_);
  log_set_.DrainAll();  // flush in-flight log SPSC residue
  return RTControllerInterface::on_deactivate(prev);
}

void DemoCatchingController::ResetLogState() noexcept {
  log_set_.Reset();
  // Reset() destroys every channel's drop counter, so the high-water mark has
  // to go with it or the next session's first drop burst is swallowed (#238).
  log_drops_reported_ = 0;
  arm_state_log_handle_ = {};
  hand_state_log_handle_ = {};
  catching_diag_log_handle_ = {};
}

void DemoCatchingController::TearDownConfiguredResources() noexcept {
  // The planner timing timer captures `this`. The thread is JOINED here —
  // unlike DemoWbc's MPC thread, which lives until the destructor. Keeping it
  // across a cleanup lets it resume under a configuration that did not ask for
  // it, and if that configuration enables the oracle the plan box gets a
  // second writer. Join is safe: it waits for any wake in flight to finish
  // publishing into boxes this object still owns.
  planner_timing_timer_.reset();
  planner_timing_cb_group_.reset();
  // The planner thread belongs to this configuration: joined here, respawned
  // by the next activation under the next configuration's parameters.
  StopPlannerThread();
  // Closed with the configuration: the next configure may run in a new
  // session, and a stream left open would keep appending to the old one's
  // file (2026-09-23 /code-review). Anything left in the ring is this
  // configuration's and goes to this file first.
  DrainPlannerTiming();
  if (planner_events_file_.is_open()) {
    planner_events_file_.close();
  }
  log_set_.DrainAll();
  // Tear the timer down BEFORE Reset() so no drain callback runs against
  // channels that are being destroyed.
  log_drain_timer_.reset();
  log_drain_cb_group_.reset();
  ResetLogState();
  ResetOwnedTopics(owned_topics_);
  // The subscription captures `this`; dropping it here is what keeps a
  // callback from arriving against a half-torn-down controller.
  traj_sub_.reset();
  traj_input_.Reset();
}

RTControllerInterface::CallbackReturn DemoCatchingController::on_cleanup(
    const rclcpp_lifecycle::State& prev) noexcept {
  TearDownConfiguredResources();
  // The next configure re-decides this from the backends it is given.
  sim_only_disabled_ = false;
  park_reason_ = CatchingParkReason::kNone;
  return RTControllerInterface::on_cleanup(prev);
}

// ── Planner thread (S6-A) ───────────────────────────────────────────────────

void DemoCatchingController::StopPlannerThread() noexcept {
  planner_thread_.reset();
}

const char* DemoCatchingController::PlannerDecisionMissing() const noexcept {
  const auto& p = planner_params_;
  if (p.sub_model.empty()) {
    return "planner.sub_model";
  }
  if (!std::isfinite(p.t_freeze)) {
    return "planner.freeze.T_freeze";
  }
  if (!p.catch_box.set) {
    return "planner.workspace.catch_box";
  }
  if (!std::isfinite(p.d_eff)) {
    return "planner.hand.d_eff";
  }
  if (!std::isfinite(p.r_cap)) {
    return "planner.hand.r_cap";
  }
  return nullptr;
}

void DemoCatchingController::SetupSupervisor() {
  // T_freeze is the SUPERVISOR's commit instant as well as the planner's
  // replacement freeze (decision G), so it is resolved whether or not the
  // planner runs — an oracle profile commits on it too.
  plan_freeze_ns_ = std::isfinite(planner_params_.t_freeze)
                        ? static_cast<std::int64_t>(std::llround(planner_params_.t_freeze * 1e9))
                        : 0;
  wait_pose_yaml_.fill(0.0);
  for (int i = 0; i < planner_params_.wait_pose_n && i < kDemoCatchingMaxArmDof; ++i) {
    wait_pose_yaml_[static_cast<std::size_t>(i)] =
        planner_params_.wait_pose[static_cast<std::size_t>(i)];
  }
  wait_pose_ = wait_pose_yaml_;
  wait_pose_from_current_ =
      planner_params_.wait_pose_source == rtc::catching::PlannerParams::WaitPoseSource::kCurrent;
  wait_pose_adopted_ = false;
  pose_tol_ = params_.supervisor_ready_pose_tol;
  homing_v_max_ = params_.supervisor_homing_v_max;
  homing_eta_a_ = params_.supervisor_homing_eta_a;
  homing_qd_tol_ = params_.supervisor_homing_qd_tol;
  decel_a_dec_ = params_.supervisor_decel_a_dec.tbd ? 0.0 : params_.supervisor_decel_a_dec.value;
  const auto to_ns = [](const rtc::catching::TbdDouble& v) -> std::int64_t {
    return (v.tbd || !std::isfinite(v.value))
               ? 0
               : static_cast<std::int64_t>(std::llround(v.value * 1e9));
  };
  t_hold_ns_ = to_ns(params_.hand.T_hold);
  t_close_e2e_ns_ = to_ns(params_.hand.T_close_e2e);
  t_release_timeout_ns_ = to_ns(params_.hand.T_release_timeout);
  stop_deadline_ns_ = to_ns(params_.supervisor_deadline_stop_s);
  return_deadline_ns_ = to_ns(params_.supervisor_deadline_return_s);
  stale_committed_max_ns_ = to_ns(params_.supervisor_stale_committed_max_s);
  sat_ticks_ = params_.supervisor_sat_ticks;
  // The sequencer owns the hand unless the S4 step rig does (never both).
  hand_seq_enabled_ = false;
  if (!hand_step_enabled_ && params_.hand.dof == hand_dof_) {
    hand_seq_enabled_ =
        hand_seq_.Configure(rtc::catching::HandSequencerConfig::FromProfile(params_.hand));
  }
  // The contact lane (S7.3). The stride is the hand device's own sensor
  // layout — the runtime SSoT (hand_sensor_layout.hpp) — or the 7-value union
  // when the device declares none.
  rtc::catching::ContactDebounceConfig contact_cfg;
  contact_cfg.f_min = params_.supervisor_contact_f_min;
  contact_cfg.k_sigma = params_.supervisor_contact_k_sigma;
  contact_cfg.n_debounce =
      static_cast<std::uint32_t>(std::max(params_.supervisor_contact_n_debounce, 1));
  contact_cfg.baseline_alpha = params_.supervisor_contact_baseline_alpha;
  contact_configured_ = contact_.Configure(contact_cfg);
  tip_stride_ = static_cast<int>(kHandInferenceValuesPerFingertipCapacity);
  if (const auto* hand_cfg = GetDeviceNameConfig(GetSecondaryDeviceName());
      hand_cfg != nullptr && hand_cfg->sensor_layout.has_value() &&
      hand_cfg->sensor_layout->inference_values_per_group >= 4) {
    tip_stride_ = hand_cfg->sensor_layout->inference_values_per_group;
  }
  contact_m_min_ = params_.supervisor_contact_m_min;
  contact_n_baseline_min_ = params_.supervisor_contact_n_baseline_min;
  contact_t_stale_ns_ =
      static_cast<std::int64_t>(std::llround(params_.supervisor_contact_t_stale * 1e9));
  contact_t_confirm_ns_ =
      static_cast<std::int64_t>(std::llround(params_.supervisor_contact_t_confirm * 1e9));
  // The hand-joint capture witness (D-S8-8 (b)). Its torque scale is the HAND
  // device's own joint_limits.max_torque, so no robot constant enters here; a
  // profile that asks for the witness on a device without those limits is
  // refused by SupervisorValueMissing, not silently judged by fingertips.
  hand_capture_cfg_ = rtc::catching::HandCaptureConfig{};
  hand_capture_enabled_ = false;
  hand_capture_t_persist_ns_ = to_ns(params_.hand.capture.t_persist);
  if (params_.hand.capture.enabled && hand_seq_enabled_) {
    std::span<const double> tau_max{};
    if (const auto* hand_cfg = GetDeviceNameConfig(GetSecondaryDeviceName());
        hand_cfg != nullptr && hand_cfg->joint_limits.has_value()) {
      tau_max = hand_cfg->joint_limits->max_torque;
    }
    hand_capture_cfg_ = rtc::catching::HandCaptureConfig::FromProfile(params_.hand, tau_max);
    hand_capture_enabled_ = hand_capture_cfg_.Valid() && !params_.hand.capture.t_persist.tbd;
    if (hand_capture_enabled_) {
      RCLCPP_INFO(logger_,
                  "supervisor: hand-joint capture witness on — rho in [%.3f, %.3f], effort >= "
                  "%.3f max_torque, >= %d joint(s), %.3f s",
                  hand_capture_cfg_.rho_min, hand_capture_cfg_.rho_max,
                  hand_capture_cfg_.effort_frac_min, hand_capture_cfg_.min_joints,
                  static_cast<double>(hand_capture_t_persist_ns_) * 1e-9);
    }
  }
  trials_enabled_ = clik_enabled_ && SupervisorValueMissing() == nullptr;
  if (trials_enabled_) {
    RCLCPP_INFO(logger_,
                "supervisor: trials enabled — commit at t_c − %.3f s, wait pose (%d joints, tol "
                "%.3f rad), hand %s, T_hold %.3f s, T_release_timeout %.3f s%s, motion deadlines "
                "stop %.3f s / return %.3f s%s",
                static_cast<double>(plan_freeze_ns_) * 1e-9, planner_params_.wait_pose_n, pose_tol_,
                hand_seq_enabled_ ? "sequenced" : "on the step rig",
                static_cast<double>(t_hold_ns_) * 1e-9,
                static_cast<double>(t_release_timeout_ns_) * 1e-9,
                params_.hand.T_release_timeout_derived ? " (derived)" : "",
                static_cast<double>(stop_deadline_ns_) * 1e-9,
                static_cast<double>(return_deadline_ns_) * 1e-9,
                params_.supervisor_deadline_provisional ? " (provisional)" : "");
  }
}

const char* DemoCatchingController::SupervisorValueMissing() const noexcept {
  if (!clik_enabled_) {
    return nullptr;  // no trial can start: nothing here is consumed
  }
  if (!std::isfinite(planner_params_.t_freeze) || plan_freeze_ns_ <= 0) {
    return "planner.freeze.T_freeze";
  }
  rtc::catching::CatchingValidationReport freeze{};
  rtc::catching::CheckFreezeCoversClose(freeze, params_, planner_params_.t_freeze,
                                        1.0 / GetDefaultDt());
  if (freeze.failure_count > 0) {
    return "planner.freeze.T_freeze (shorter than T_close_e2e + T_arm + one tick)";
  }
  if (planner_params_.wait_pose_n != arm_dof_) {
    return "planner.wait_pose";
  }
  const auto n = static_cast<std::size_t>(arm_dof_);
  if (arm_qdd_max_.size() < n) {
    return "robot.arm.qdd_max (the acceleration box the stop and homing ramp with)";
  }
  if (arm_q_min_margined_.size() < n || arm_q_max_margined_.size() < n) {
    return "the arm's joint_limits (the position box the stop and homing stay inside)";
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (wait_pose_[i] < arm_q_min_margined_[i] || wait_pose_[i] > arm_q_max_margined_[i]) {
      return "planner.wait_pose (outside the margined position box)";
    }
  }
  if (!(decel_a_dec_ > 0.0)) {
    return "supervisor.decel.a_dec";
  }
  // D-S9-D1: a trial whose stop or return has no deadline can hang in either
  // with the arm commanded and nothing escalating.
  if (stop_deadline_ns_ <= 0 || return_deadline_ns_ <= 0) {
    return "supervisor.deadline.stop_s / return_s";
  }
  if (t_hold_ns_ <= 0 && params_.hand.T_hold.tbd) {
    return "robot.hand.T_hold";
  }
  if (!hand_step_enabled_ && !hand_seq_enabled_) {
    return "robot.hand (a profile the sequencer can run: poses, eta_close, T_close_e2e, "
           "T_close_timeout)";
  }
  if (!hand_step_enabled_ && params_.hand.T_close_e2e.tbd) {
    return "robot.hand.T_close_e2e";
  }
  // Above T_close_e2e as well as positive: the validator's own line for this
  // key is only a warning here (the key is exempt from the consumed gate with
  // the T_close_e2e it is derived from), and an explicit value at or below the
  // closure time would disarm after every release.
  if (hand_seq_enabled_ &&
      (t_release_timeout_ns_ <= 0 || t_release_timeout_ns_ <= t_close_e2e_ns_)) {
    return "robot.hand.T_release_timeout (resolved, and above T_close_e2e)";
  }
  if (hand_seq_enabled_ && params_.hand.capture.enabled && !hand_capture_enabled_) {
    return "robot.hand.capture (every threshold resolved, and the hand device's "
           "joint_limits.max_torque for each joint)";
  }
  return nullptr;
}

bool DemoCatchingController::SetupPlanner() {
  // No thread may be running while the cycle is re-bound: Pause does not stop
  // a wake in flight, and Bind/Configure write what Run reads. on_cleanup has
  // already joined it on the normal path; this covers a configure that did not
  // go through one.
  StopPlannerThread();
  // Bound on every configure, enabled or not: the boxes are members and the
  // binding costs nothing, and a re-configure that turns the planner on must
  // not depend on the previous configure having done it.
  planner_cycle_.Configure(planner_params_);
  if (!planner_cycle_.Bind({&traj_box_, &cov_box_, &planner_rt_box_, &plan_box_, &segment_box_})) {
    RCLCPP_ERROR(logger_, "planner: could not bind the plan/trajectory boxes");
    return false;
  }
  // Dropped on every configure (the thread is joined above): a re-configure
  // with the planner or the decel MPC off must not report the previous one.
  planner_cycle_.ClearSegmentPlanner();
  if (!planner_params_.enabled) {
    return true;
  }
  if (arm_dof_ > static_cast<int>(rtc::catching::kMaxPlanNv)) {
    RCLCPP_ERROR(logger_,
                 "planner: the arm has %d joints but a plan carries at most %d (kMaxPlanNv) — "
                 "set planner.enabled: false or raise the capacity",
                 arm_dof_, static_cast<int>(rtc::catching::kMaxPlanNv));
    return false;
  }
  if (planner_params_.wait_pose_n != 0 && planner_params_.wait_pose_n != arm_dof_) {
    RCLCPP_ERROR(logger_,
                 "planner.wait_pose has %d entries but the arm has %d joints (arm joint order, "
                 "decision L)",
                 planner_params_.wait_pose_n, arm_dof_);
    return false;
  }
  if (planner_wake_fd_.load(std::memory_order_acquire) < 0) {
    const int fd = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (fd < 0) {
      RCLCPP_ERROR(logger_, "planner: eventfd() failed (errno %d)", errno);
      return false;
    }
    planner_wake_fd_.store(fd, std::memory_order_release);
  }
  // ── The search's model (S6-B, R-3) ────────────────────────────────────────
  planner_cycle_.ClearSearch();
  planner_handle_.reset();
  if (!builder_) {
    RCLCPP_WARN(logger_,
                "planner: no system model — the thread runs, but its search is the stub "
                "(it publishes \"no plan\")%s",
                decel_mode_ == rtc::catching::CatchingDecelMode::kMpc
                    ? " and the decel MPC does not run"
                    : "");
  } else if (!SetupGridCatchSearch()) {
    return false;
  }
  RCLCPP_INFO(logger_,
              "planner enabled: wake timeout %.3f s, budget %.3f s, wait pose %s — the thread "
              "spawns on the `mpc` layout role (mpc_main) at activation%s",
              planner_params_.wake_timeout_s, planner_params_.budget_s,
              planner_params_.wait_pose_n > 0 ? "set" : "absent",
              layout_profile_drops_mpc_ ? ", which this launch's profile will REFUSE" : "");
  return true;
}

bool DemoCatchingController::ResolveCatchSubModel(
    const char* who, std::shared_ptr<const pinocchio::Model>& model, pinocchio::FrameIndex& frame,
    std::array<int, rtc::catching::kMaxPlanNv>& device_of_model) {
  const std::string& name = planner_params_.sub_model;
  if (!builder_) {
    RCLCPP_ERROR(logger_, "%s: no system model to take sub-model '%s' from", who, name.c_str());
    return false;
  }
  try {
    model = builder_->GetReducedModel(name);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_,
                 "%s: urdf.sub_models has no '%s' (%s) — declare the catch sub-model in the "
                 "robot config (arm root → the catch frame's parent link, R-3)",
                 who, name.c_str(), e.what());
    return false;
  }
  if (!model || model->nq != model->nv || model->nv <= 0 ||
      model->nv > static_cast<int>(rtc::catching::kMaxPlanNv)) {
    RCLCPP_ERROR(logger_, "%s: sub-model '%s' is unusable (nq %d, nv %d, capacity %d)", who,
                 name.c_str(), model ? model->nq : -1, model ? model->nv : -1,
                 static_cast<int>(rtc::catching::kMaxPlanNv));
    return false;
  }
  try {
    frame = rtc::catching::ResolveCatchFrame(*model, catch_frame_name_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "%s: catch frame '%s' is not in sub-model '%s': %s", who,
                 catch_frame_name_.c_str(), name.c_str(), e.what());
    return false;
  }
  for (pinocchio::JointIndex jid = 1; jid < static_cast<pinocchio::JointIndex>(model->njoints);
       ++jid) {
    const int qi = model->joints[jid].idx_q();
    const std::string& jname = model->names[jid];
    const auto it = std::find(arm_joint_names_.begin(), arm_joint_names_.end(), jname);
    if (it == arm_joint_names_.end() || qi < 0 || qi >= model->nv) {
      RCLCPP_ERROR(logger_,
                   "%s: sub-model '%s' joint '%s' is not an arm joint of this controller — "
                   "the catch sub-model must contain the arm's joints only (the hand locked)",
                   who, name.c_str(), jname.c_str());
      return false;
    }
    device_of_model[static_cast<std::size_t>(qi)] =
        static_cast<int>(std::distance(arm_joint_names_.begin(), it));
  }
  if (model->nv != arm_dof_) {
    RCLCPP_ERROR(logger_, "%s: sub-model '%s' has %d joints, the arm %d", who, name.c_str(),
                 model->nv, arm_dof_);
    return false;
  }
  return true;
}

bool DemoCatchingController::SetupGridCatchSearch() {
  const std::string& name = planner_params_.sub_model;
  std::shared_ptr<const pinocchio::Model> model;
  rtc::catching::GridCatchSearchModel pm;
  pinocchio::FrameIndex frame = 0;
  if (!ResolveCatchSubModel("planner", model, frame, pm.device_of_model)) {
    return false;
  }
  // No SetJointOrder on this handle: CatchPoseIk refuses a reordered one (its
  // Jacobian columns and box rows are model order). The model↔device mapping
  // is carried explicitly instead.
  planner_handle_ = std::make_unique<rtc_urdf_bridge::RtModelHandle>(model);
  pm.handle = planner_handle_.get();
  pm.catch_frame = frame;
  pm.nv = model->nv;
  const auto& vmax = device_max_velocity_[static_cast<std::size_t>(kCatchingArmDeviceIdx)];
  for (int q = 0; q < pm.nv; ++q) {
    const auto u = static_cast<std::size_t>(q);
    const auto d = static_cast<std::size_t>(pm.device_of_model[u]);
    pm.qdot_max[u] = d < vmax.size() ? vmax[d] : 0.0;
    pm.qddot_max[u] = d < arm_qdd_max_.size() ? arm_qdd_max_[d] : 0.0;
  }
  pm.accel_box = static_cast<int>(arm_qdd_max_.size()) == arm_dof_;

  // Profile constants outside planner.* — NaN where the profile says TBD, so
  // the gate that needs one fails instead of using a guess.
  const auto val = [](const rtc::catching::TbdDouble& v) {
    return v.tbd ? std::numeric_limits<double>::quiet_NaN() : v.value;
  };
  rtc::catching::GridCatchSearchConstants pc;
  pc.eta_v = ResolvedPlannerEtaV();
  pc.v_max = val(params_.reference_v_max);
  pc.a_dec = val(params_.supervisor_decel_a_dec);
  pc.t_arm_s = static_cast<double>(t_arm_ns_) * 1e-9;
  pc.t_close_e2e = val(params_.hand.T_close_e2e);
  // T_close,tot = T_close,e2e + h/2 (L3 §4.5): the tick quantisation budget.
  pc.t_close_total = pc.t_close_e2e + 0.5 * GetDefaultDt();
  pc.ball_mass = val(params_.ball.mass);
  // The L4 reference the rollout replays (§4.8): the controller's own ω, ζ and
  // a_max, and the control period as the confirmation step.
  pc.ref_omega = val(params_.reference_omega);
  pc.ref_zeta = val(params_.reference_zeta);
  pc.ref_a_max = val(params_.reference_a_max);
  pc.control_dt = GetDefaultDt();

  if (!planner_cycle_.ConfigureGridCatchSearch(pm, pc, catch_pose_ik_config_.options)) {
    RCLCPP_ERROR(logger_,
                 "planner: the search refused its model (nv %d, wait pose %d entries — "
                 "planner.wait_pose must give one per arm joint)",
                 pm.nv, planner_params_.wait_pose_n);
    return false;
  }
  // MD-44: the decel cores exist only for a configuration that follows them.
  // Without a pre-catch grid there is no decel planner to build (MD-70) — a
  // profile mistake DecelModeUnmet parks on, not a configure failure.
  if (decel_mode_ == rtc::catching::CatchingDecelMode::kMpc &&
      planner_params_.mpc_segment.n_pre_max > 0 && !SetupMpcSegmentPlanner(model, pm)) {
    return false;
  }
  RCLCPP_INFO(logger_,
              "planner search ready: sub-model '%s' (nv %d), catch frame '%s', accel box %s, "
              "T_freeze %.3f s, max_ik %d",
              name.c_str(), pm.nv, catch_frame_name_.c_str(), pm.accel_box ? "on" : "OFF",
              planner_params_.t_freeze, planner_params_.max_ik);
  return true;
}

const char* DemoCatchingController::MpcSegmentConfigInvalid() const noexcept {
  const auto& d = planner_params_.mpc_segment;
  // The CLIK's torque box only exists in the dynamic form; the kinematic form
  // has none to exceed, so the pair is not judged there.
  if (params_.joint_cmd_accel_constraint == rtc::catching::CatchingAccelConstraint::kDynamic &&
      !params_.joint_cmd_eta_tau.tbd &&
      !(d.eta_tau + d.slack_max <= params_.joint_cmd_eta_tau.value)) {
    return "planner.decel_mpc.eta_tau + publish.slack_max exceeds joint_cmd.eta_tau (a published "
           "stop could ask for torque the CLIK's torque box refuses, MD-33)";
  }
  return nullptr;
}

bool DemoCatchingController::SetupMpcSegmentPlanner(
    const std::shared_ptr<const pinocchio::Model>& model,
    const rtc::catching::GridCatchSearchModel& pm) {
  if (pm.nv > rtc::catching::kMaxSegmentNv) {
    RCLCPP_ERROR(logger_,
                 "planner.decel_mpc: the arm has %d joints but a decel segment carries at most "
                 "%d (kMaxDecelNv) — set supervisor.decel.mode: closed_form or raise the capacity",
                 pm.nv, rtc::catching::kMaxSegmentNv);
    return false;
  }
  const auto* arm_cfg = GetDeviceNameConfig(GetPrimaryDeviceName());
  const std::vector<double>* torque = nullptr;
  if (arm_cfg != nullptr && arm_cfg->joint_limits.has_value()) {
    torque = &arm_cfg->joint_limits->max_torque;
  }
  if (torque == nullptr || static_cast<int>(torque->size()) < pm.nv) {
    RCLCPP_ERROR(logger_,
                 "planner.decel_mpc: the arm device has no joint_limits.max_torque for its %d "
                 "joints — the decel MPC's torque rows need them",
                 pm.nv);
    return false;
  }
  rtc::catching::MpcSegmentPlannerModel dm;
  dm.arm = model;
  dm.catch_frame = pm.catch_frame;
  dm.nv = pm.nv;
  dm.device_of_model = pm.device_of_model;
  // MD-42: the stop must be executable by the CLIK, so its position box is
  // the URDF's AND the CLIK's margined device box (m_q then sits inside it).
  // URDF limits alone let a published node reach past the CLIK's box (iiwa7
  // A7 by 2.6e-5 rad on the shipped values).
  const bool clik_box = static_cast<int>(arm_q_min_margined_.size()) == arm_dof_ &&
                        static_cast<int>(arm_q_max_margined_.size()) == arm_dof_;
  for (int m = 0; m < pm.nv; ++m) {
    const auto u = static_cast<std::size_t>(m);
    const auto d = static_cast<std::size_t>(pm.device_of_model[u]);
    dm.q_min[u] = model->lowerPositionLimit[m];
    dm.q_max[u] = model->upperPositionLimit[m];
    if (clik_box) {
      dm.q_min[u] = std::max(dm.q_min[u], arm_q_min_margined_[d]);
      dm.q_max[u] = std::min(dm.q_max[u], arm_q_max_margined_[d]);
    }
    mpc_segment_planner_q_min_[d] = dm.q_min[u];
    mpc_segment_planner_q_max_[d] = dm.q_max[u];
    dm.qdot_max[u] = pm.qdot_max[u];
    dm.tau_max[u] = (*torque)[static_cast<std::size_t>(pm.device_of_model[u])];
  }
  rtc::catching::MpcSegmentPlannerConstants dc;
  dc.eta_v = ResolvedPlannerEtaV();
  dc.t_arm_s = static_cast<double>(t_arm_ns_) * 1e-9;
  dc.control_dt = GetDefaultDt();
  // The ball's direction of travel is undefined below the IK's own floor.
  dc.v_eps = catch_pose_ik_config_.options.v_eps;
  std::string error;
  if (!planner_cycle_.ConfigureMpcSegmentPlanner(dm, dc, &error)) {
    RCLCPP_ERROR(logger_, "planner.decel_mpc: %s", error.c_str());
    return false;
  }
  const auto& d = planner_params_.mpc_segment;
  RCLCPP_INFO(logger_,
              "decel MPC ready: stop part %d x %.3f s (%.3f s), %d blocks, post-catch replans to "
              "k = %d, eta_tau %.2f, publish slack <= %.2f / terminal %.2f, armature 0 (MD-25)",
              d.n_nodes, d.dt_s, d.n_nodes * d.dt_s, d.n_blocks, d.k_max, d.eta_tau, d.slack_max,
              d.slack_terminal_max);
  RCLCPP_INFO(logger_,
              "decel MPC approach grid: up to %d x %.3f s before t_c, budgets first %.3f s / "
              "replan %.3f s, same-point re-solve %s, catch error <= %.3f m",
              d.n_pre_max, d.dt_pre_s, d.budget_first_s, d.budget_replan_s,
              d.replan_same_point ? "on" : "off", d.catch_pos_err_max);
  if (!d.horizon_explicit) {
    RCLCPP_WARN(logger_,
                "planner.decel_mpc.horizon is not set: the stop part runs the code default "
                "%d x %.3f s, not the MD-54 grid",
                d.n_nodes, d.dt_s);
  }
  return true;
}

void DemoCatchingController::SetupSegmentFollower() {
  // Unbound on every configure: a re-configure to closed_form must not keep
  // the previous sampler, and DecelModeUnmet reads Initialized().
  segment_follower_ = rtc::catching::NodeTrajectoryFollower{};
  segment_qd_max_.fill(0.0);
  segment_eta_v_ = ResolvedPlannerEtaV();
  segment_k_p_ = params_.joint_cmd_k_p.tbd ? 0.0 : params_.joint_cmd_k_p.value;
  segment_k_n_ = params_.joint_cmd_k_posture.tbd ? 0.0 : params_.joint_cmd_k_posture.value;
  if (decel_mode_ != rtc::catching::CatchingDecelMode::kMpc) {
    return;
  }
  // The planner's catch sub-model — resolved whether or not the planner runs:
  // the oracle profile follows segments a test writes, on the same model.
  std::shared_ptr<const pinocchio::Model> model;
  pinocchio::FrameIndex frame = 0;
  std::array<int, rtc::catching::kMaxPlanNv> device_of_model{};
  if (!ResolveCatchSubModel("supervisor.decel.mode mpc", model, frame, device_of_model)) {
    return;
  }
  if (model->nv > rtc::catching::kMaxSegmentNv ||
      !segment_follower_.Init(
          model, frame,
          std::span<const int>(device_of_model.data(), static_cast<std::size_t>(model->nv)))) {
    RCLCPP_ERROR(logger_,
                 "supervisor.decel.mode mpc: the segment sampler refused the sub-model "
                 "(nv %d, capacity %d)",
                 model->nv, rtc::catching::kMaxSegmentNv);
    segment_follower_ = rtc::catching::NodeTrajectoryFollower{};
    return;
  }
  const auto& vmax = device_max_velocity_[static_cast<std::size_t>(kCatchingArmDeviceIdx)];
  for (int d = 0; d < arm_dof_ && d < rtc::catching::kMaxSegmentNv; ++d) {
    const auto u = static_cast<std::size_t>(d);
    segment_qd_max_[u] = u < vmax.size() ? vmax[u] : 0.0;
  }
  RCLCPP_INFO(logger_,
              "DECEL law: mpc — takes a plan with its first segment and follows the MPC's "
              "segments from APPROACH to the end of the stop (no soft-catch reference, no "
              "closed-form fallback; MD-44, MD-45); switch margin %.2f, admission age <= %.3f s",
              decel_switch_margin_, static_cast<double>(kSegmentAdmissionMaxAgeNs) * 1e-9);
}

const char* DemoCatchingController::DecelModeUnmet() const noexcept {
  if (!clik_enabled_) {
    return "the arm command path is not wired (no model or CLIK)";
  }
  if (!segment_follower_.Initialized() || segment_follower_.Nv() != arm_dof_) {
    return "the segment sampler has no catch sub-model (planner.sub_model, the catch frame, "
           "at most kMaxDecelNv arm joints)";
  }
  if (!(segment_k_n_ > 0.0)) {
    return "joint_cmd.K_n is not above 0 (the posture feedforward divides by it, MD-36)";
  }
  if (!(segment_eta_v_ < 1.0)) {
    return "planner.gamma.eta_v is not below 1 (the switch gate's headroom is (1 - eta_v) "
           "q_dot_max, MD-39)";
  }
  for (int i = 0; i < arm_dof_; ++i) {
    const double v = segment_qd_max_[static_cast<std::size_t>(i)];
    if (!std::isfinite(v) || !(v > 0.0)) {
      return "the arm device lacks a positive joint_limits.max_velocity for every joint";
    }
  }
  if (!clik_v_box_complete_) {
    return "the CLIK's per-joint velocity box is off (every arm and hand joint needs "
           "max_velocity)";
  }
  if (static_cast<int>(arm_q_min_margined_.size()) != arm_dof_) {
    return "the CLIK's position box is off (device position limits incomplete)";
  }
  // planner.workspace.catch_box is not one of these (MD-73): the RT does not
  // judge where a stop ends. The search needs it, and says so itself
  // (kPlannerUnset).
  // MD-45, MD-70: the arm follows a segment from APPROACH, and a plan is
  // published only together with one that starts before t_c. Without a
  // pre-catch grid no decel planner is built — no trial would ever start. The
  // oracle profile has no planner: its test writes the box.
  if (!oracle_enabled_ && planner_params_.enabled && !(planner_params_.mpc_segment.n_pre_max > 0)) {
    return "planner.decel_mpc.approach.n_pre_max is 0 (the RT takes a plan only with a segment "
           "that starts before t_c, MD-45)";
  }
  if (!oracle_enabled_ && !(planner_params_.enabled && planner_cycle_.SegmentPlannerConfigured())) {
    return "no decel planner runs (planner.enabled is false, or no model the cores accept)";
  }
  // MD-37: a segment for the next grid point waits in the box while the
  // pending slot holds the one before it — at most the replan lead and three
  // ticks (kSegmentAdmissionMaxAgeNs). An age bound below that would refuse
  // those segments.
  const double min_age_s = planner_params_.mpc_segment.budget_replan_s + 3.0 * GetDefaultDt();
  if (planner_params_.enabled &&
      !(static_cast<double>(kSegmentAdmissionMaxAgeNs) * 1e-9 > min_age_s)) {
    return "planner.decel_mpc.budget.replan_s + 3 control periods is not below the decel "
           "admission age bound";
  }
  return nullptr;
}

void DemoCatchingController::SpawnPlannerThreadIfNeeded() noexcept {
  if (!planner_params_.enabled || planner_thread_ || !planner_cycle_.Bound()) {
    return;
  }
  const int fd = planner_wake_fd_.load(std::memory_order_acquire);
  if (fd < 0) {
    return;
  }
  try {
    // The `mpc` layout role, exactly as DemoWbc's SpawnMpcThreadIfNeeded takes
    // it (E-7 decision J): same slot, same scheduling, same thread name.
    const auto thread_configs = rtc::SelectThreadConfigs();
    planner_thread_ = std::make_unique<CatchingPlannerThread>(
        planner_cycle_, fd, planner_params_.wake_timeout_s, planner_timing_, planner_events_);
    planner_thread_->StartWith(thread_configs.mpc.main);
    RCLCPP_INFO(logger_, "planner thread started: %s on slot %d, policy %d prio %d",
                thread_configs.mpc.main.name, thread_configs.mpc.main.cpu_core,
                thread_configs.mpc.main.sched_policy, thread_configs.mpc.main.sched_priority);
  } catch (const std::exception& e) {
    // Lifecycle callback, non-RT. The controller keeps running without a
    // planner — which is what it did before S6 — rather than taking the
    // activation down with a thread-spawn failure.
    RCLCPP_ERROR(logger_, "planner thread spawn failed: %s — no plans will be published", e.what());
    planner_thread_.reset();
    return;
  }

  // Timing CSV + 1 Hz drain, one-shot per controller lifetime (a re-activation
  // must not truncate the file or re-register the timer).
  if (!planner_timing_logger_.IsOpen()) {
    try {
      const auto timing_dir = rtc::TimingDir(rtc::ResolveSessionDir());
      std::error_code ec;
      std::filesystem::create_directories(timing_dir, ec);
      if (!planner_timing_logger_.Open(timing_dir / "planner_timing_log.csv",
                                       &rtc::WriteRtTickTimingHeader, &rtc::WriteRtTickTimingRow)) {
        RCLCPP_WARN(logger_, "planner timing CSV could not be opened — timing not recorded");
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_, "planner timing CSV disabled: %s", e.what());
    }
  }
  if (!planner_events_file_.is_open()) {
    try {
      // The tick record's directory (log_set_'s key), so the two files of
      // one session sit side by side.
      const auto dir = rtc::ResolveSessionDir() / "controllers" / kCatchingLogKey;
      std::error_code ec;
      std::filesystem::create_directories(dir, ec);
      const auto path = dir / "planner_events.csv";
      const bool fresh = !std::filesystem::exists(path);
      planner_events_file_.open(path, std::ios::app);
      if (planner_events_file_.is_open() && fresh) {
        WritePlannerEventsHeader(planner_events_file_);
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_, "planner_events.csv disabled: %s", e.what());
    }
  }
  if (node_ && !planner_timing_timer_) {
    planner_timing_cb_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    planner_timing_timer_ = node_->create_wall_timer(
        std::chrono::seconds(1), [this]() { DrainPlannerTiming(); }, planner_timing_cb_group_);
  }
}

void DemoCatchingController::DrainPlannerTiming() noexcept {
  // Touches only members that live as long as this object — never the thread,
  // which a cleanup may be joining on another executor thread right now.
  rtc::catching::PlannerCycleRecord rec{};
  while (planner_events_.Pop(rec)) {
    if (planner_events_file_.is_open()) {
      WritePlannerEventsRow(planner_events_file_, rec);
    }
  }
  if (planner_events_file_.is_open()) {
    planner_events_file_.flush();
  }
  planner_timing_.Drain(
      [this](const rtc::RtTickTimingSample& s) { planner_timing_logger_.Log(s); });
}

}  // namespace integrated_bringup
