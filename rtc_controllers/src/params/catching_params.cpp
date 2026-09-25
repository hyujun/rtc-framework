#include "rtc_controllers/catching/catching_params.hpp"

// For kCap: `io.n_min` above the snapshot capacity is unsatisfiable, and the
// capacity is owned by trajectory.hpp (one definition, not a second literal).
#include "catching_yaml_read.hpp"
#include "rtc_controllers/catching/trajectory.hpp"
#include <rtc_base/types/types.hpp>

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace rtc::catching {

namespace {

// ── Invented YAML keys ───────────────────────────────────────────────────
// None of these are named by any dynamic_catching doc as a schema field —
// each place a doc marks a value "provisional" (L0 §5.3), it does so in
// prose or a table 근거 column, not with a boolean key, EXCEPT the catch
// frame (D-17, plan §10), which already has one (`provisional: true`) but
// lives in the robot's `urdf:` tree, a different YAML root than `catching:` —
// CheckCatchFrameProvisional (S2.3a) applies the same rule to it from the
// model config. For the three provisional groups this validator DOES own,
// the chosen key sits as a sibling of the group's other fields, mirroring
// the catch frame's own placement:
//   - `reference.provisional`                              (L4 §6, whole block)
//   - `core.ball.provisional`                              (D-12 공 사양)
//   - `planner.catchability.manipulability_min.provisional` (D-18)
//   - `robot.hand.provisional`                              (L6 §6, whole profile)
// All three default to `true` (fail-closed: unconfirmed until the YAML says
// otherwise), matching how every other provisional flag in this repo defaults
// to blocking the real arm rather than trusting silence.

template <typename... Parts>
[[noreturn]] void Reject(Parts&&... parts) {
  std::string msg = "catching: ";
  (msg.append(std::forward<Parts>(parts)), ...);
  throw std::invalid_argument(msg);
}

template <typename T>
T ReadOptional(const YAML::Node& node, const char* key, T fallback) {
  const YAML::Node v = node[key];
  if (!v) {
    return fallback;
  }
  try {
    return v.as<T>();
  } catch (const YAML::Exception&) {
    Reject("'", key, "' is present but does not parse");
  }
}

/// Child section `key` of `parent`. An absent (or empty `key:`) section reads
/// as an empty node, so every key under it takes its doc default — itself TBD
/// where the doc leaves it open, which the validator then refuses in the
/// active configuration (fail-closed). A present section that is not a map is
/// refused. The READING is `params_detail::ReadSectionNode`, shared with
/// catch_pose_ik_params.cpp; only the refusal's wording is this parser's.
YAML::Node ReadSection(const YAML::Node& parent, const char* key) {
  const params_detail::SectionRead sec = params_detail::ReadSectionNode(parent, key);
  if (sec.kind == params_detail::SectionKind::kNotAMap) {
    Reject("section '", key, "' must be a map");
  }
  return sec.node;
}

/// Read a scalar that may be the literal string "TBD" (or, per L0 §5.3, an
/// unparseable/non-finite number — treated the same way). An absent key takes
/// `fallback` (the doc default, itself possibly still-TBD); a present key
/// that is neither a number nor "TBD" is refused. The READING is
/// `params_detail::ReadTbdScalar`, shared with catch_pose_ik_params.cpp. No
/// range is applied here: an out-of-range number parses, and
/// ValidateCatchingParams reports it (kRangeViolation) — unlike the sibling
/// parser, which throws. That difference is by design and is pinned by
/// test_catch_pose_ik_params.cpp (SharedKey* cases).
/// A positive whole COUNT (points, iterations, retries). Absent or the literal
/// `TBD` → `fallback` (0 where the validator should report an active TBD);
/// anything else that is not a positive whole number is refused here.
///
/// Refused rather than defaulted, like every other malformed key in this file:
/// defaulting reads a typo as "still TBD", and the operator then sees a config
/// that will not arm with no mention of the key they got wrong. Read as a
/// double and checked for integrality because `as<int>` would accept `10.9`
/// by truncation — a point count is not a value to round.
int ReadPositiveCount(const YAML::Node& node, const char* key, int fallback = 0) {
  const params_detail::TbdRead read = params_detail::ReadTbdScalar(node, key);
  switch (read.kind) {
    case params_detail::TbdReadKind::kAbsent:
      return fallback;
    case params_detail::TbdReadKind::kNotANumber:
      Reject("'", key, "' must be a positive whole number or the literal 'TBD'");
    case params_detail::TbdReadKind::kRead:
      break;
  }
  if (read.value.tbd) {
    return 0;
  }
  const double v = read.value.value;
  if (!(v > 0.0) || std::floor(v) != v) {
    Reject("'", key, "' must be a positive whole number, got ", params_detail::Spelling(read.node));
  }
  return static_cast<int>(v);
}

TbdDouble ReadTbdDouble(const YAML::Node& node, const char* key, TbdDouble fallback) {
  const params_detail::TbdRead read = params_detail::ReadTbdScalar(node, key);
  switch (read.kind) {
    case params_detail::TbdReadKind::kAbsent:
      return fallback;
    case params_detail::TbdReadKind::kNotANumber:
      Reject("'", key, "' is present but is neither a number nor the literal 'TBD'");
    case params_detail::TbdReadKind::kRead:
      break;
  }
  return read.value;
}

/// Read one hand pose array (`q_open`/`q_pre`/`q_close`) into `dst`, refusing
/// a non-sequence, a wrong length or a non-finite entry. `key` names the YAML
/// key in every rejection so a typo is attributable.
void ReadHandPoseArray(const YAML::Node& seq, const char* key, std::size_t dof,
                       std::array<double, kMaxHandDof>& dst) {
  if (!seq.IsSequence()) {
    Reject("robot.hand.", key, " must be a sequence of ", std::to_string(dof), " numbers");
  }
  if (seq.size() != dof) {
    Reject("robot.hand.", key, " must have the same length as q_pre/q_close");
  }
  for (std::size_t i = 0; i < dof; ++i) {
    double v = std::numeric_limits<double>::quiet_NaN();
    try {
      v = seq[i].as<double>();
    } catch (const YAML::Exception&) {
      Reject("robot.hand.", key, "[", std::to_string(i), "] does not parse as a number");
    }
    if (!std::isfinite(v)) {
      Reject("robot.hand.", key, "[", std::to_string(i), "] must be finite");
    }
    dst[i] = v;
  }
}

/// Parse `robot.hand.q_pre`/`q_close` (+ optional `caging_mask`, `q_open`).
/// `q_pre`/`q_close` must be TBD together or given together — a doc has no
/// notion of "half a hand profile" (L6 §6). `caging_mask` defaults to "every
/// joint is checked" when absent, the fail-closed reading of L6 §4.2 (it names
/// no default). `q_open` (L6 §5.1) is not part of the §4.2 caging pair: an
/// absent one leaves `q_open_tbd` set for the validator to report as a
/// still-TBD active key, while a present one must match the pair's length.
HandProfile ReadHandProfile(const YAML::Node& hand_node) {
  HandProfile out;
  out.rho_eps = ReadOptional(hand_node, "rho_eps", out.rho_eps);
  out.provisional = ReadOptional(hand_node, "provisional", true);
  out.eta_close = ReadTbdDouble(hand_node, "eta_close", out.eta_close);
  out.T_close_e2e = ReadTbdDouble(hand_node, "T_close_e2e", out.T_close_e2e);

  // ── Sequencer keys (S7.1) ─────────────────────────────────────────────────
  // `T_pre` is refused by name: #537 S7 decision Q4 removed the time-based
  // preshape (the hand waits at q_pre from the moment the arm is at its wait
  // pose), and a profile still carrying the key was written for a sequencer
  // that no longer exists — its author should hear that, not have it ignored.
  if (hand_node["T_pre"]) {
    Reject(
        "robot.hand.T_pre was removed (S7: the hand waits at q_pre, there is no timed "
        "preshape) — delete the key");
  }
  out.T_hold = ReadTbdDouble(hand_node, "T_hold", out.T_hold);
  // Absent → the documented rule, 2 × T_close_e2e (#537 S7 D-S7-1), derived
  // here rather than stored in the profile so a re-measured T_close_e2e moves
  // the timeout with it. An explicit value (or an explicit `TBD`) overrides.
  if (hand_node["T_close_timeout"]) {
    out.T_close_timeout = ReadTbdDouble(hand_node, "T_close_timeout", out.T_close_timeout);
  } else if (!out.T_close_e2e.tbd) {
    out.T_close_timeout = TbdDouble::Resolved(kCloseTimeoutPerE2e * out.T_close_e2e.value);
  }
  // An explicit value (or `TBD`) overrides; absent, the default is derived at
  // the end, once the poses it depends on are read.
  const bool release_timeout_given = static_cast<bool>(hand_node["T_release_timeout"]);
  if (release_timeout_given) {
    out.T_release_timeout = ReadTbdDouble(hand_node, "T_release_timeout", out.T_release_timeout);
  }
  out.q_tol = ReadOptional(hand_node, "q_tol", out.q_tol);
  out.qd_tol = ReadOptional(hand_node, "qd_tol", out.qd_tol);
  // `capture` (D-S8-8 (b)): present = enabled, and then every threshold must
  // be given — a half-written block is the validator's to report, not a
  // default to guess.
  // Presence is tested on the raw key: ReadSection answers an absent section
  // with an empty (but truthy) map, which would read as "enabled".
  if (hand_node["capture"]) {
    const YAML::Node capture = ReadSection(hand_node, "capture");
    out.capture.enabled = true;
    out.capture.rho_min = ReadTbdDouble(capture, "rho_min", out.capture.rho_min);
    out.capture.rho_max = ReadTbdDouble(capture, "rho_max", out.capture.rho_max);
    out.capture.effort_frac_min =
        ReadTbdDouble(capture, "effort_frac_min", out.capture.effort_frac_min);
    out.capture.t_persist = ReadTbdDouble(capture, "t_persist", out.capture.t_persist);
    out.capture.min_joints = ReadOptional(capture, "min_joints", out.capture.min_joints);
    out.capture.provisional = ReadOptional(capture, "provisional", true);
  }
  const YAML::Node hold = ReadSection(hand_node, "hold");
  const std::string hold_mode = ReadOptional<std::string>(hold, "mode", "close_target");
  if (hold_mode == "close_target") {
    out.hold_mode = HandHoldMode::kCloseTarget;
  } else if (hold_mode == "measured_offset") {
    out.hold_mode = HandHoldMode::kMeasuredOffset;
  } else {
    Reject("robot.hand.hold.mode must be 'close_target' or 'measured_offset', got '", hold_mode,
           "'");
  }
  out.hold_delta_rad = ReadOptional(hold, "delta_rad", out.hold_delta_rad);

  const YAML::Node q_pre = hand_node["q_pre"];
  const YAML::Node q_close = hand_node["q_close"];
  const bool pre_is_seq = q_pre && q_pre.IsSequence();
  const bool close_is_seq = q_close && q_close.IsSequence();

  if (!pre_is_seq && !close_is_seq) {
    // Whole profile still TBD: `dof` is unknown, so `q_open` cannot be length-
    // checked either and stays TBD whatever the YAML says. The validator
    // reports the pair; there is no state where q_open alone is meaningful.
    out.tbd = true;
    out.dof = 0;
    return out;
  }
  if (pre_is_seq != close_is_seq) {
    Reject("robot.hand.q_pre and q_close must both be 'TBD' or both given as arrays");
  }
  if (q_pre.size() != q_close.size()) {
    Reject("robot.hand.q_pre and q_close must have the same length");
  }
  if (q_pre.size() == 0) {
    Reject("robot.hand.q_pre/q_close must not be empty (use 'TBD' for an unset profile)");
  }
  if (q_pre.size() > kMaxHandDof) {
    Reject("robot.hand.q_pre/q_close length exceeds kMaxHandDof (", std::to_string(kMaxHandDof),
           ")");
  }
  const auto dof = static_cast<std::size_t>(q_pre.size());

  const YAML::Node caging = hand_node["caging_mask"];
  const bool caging_is_seq = caging && caging.IsSequence();
  if (caging && !caging_is_seq) {
    Reject("robot.hand.caging_mask must be a sequence when present");
  }
  if (caging_is_seq && caging.size() != dof) {
    Reject("robot.hand.caging_mask must have the same length as q_pre/q_close");
  }

  for (std::size_t i = 0; i < dof; ++i) {
    double pre = std::numeric_limits<double>::quiet_NaN();
    double close = std::numeric_limits<double>::quiet_NaN();
    try {
      pre = q_pre[i].as<double>();
      close = q_close[i].as<double>();
    } catch (const YAML::Exception&) {
      Reject("robot.hand.q_pre/q_close[", std::to_string(i), "] does not parse as a number");
    }
    if (!std::isfinite(pre) || !std::isfinite(close)) {
      Reject("robot.hand.q_pre/q_close[", std::to_string(i), "] must be finite");
    }
    out.q_pre[i] = pre;
    out.q_close[i] = close;
    if (caging_is_seq) {
      try {
        out.caging_mask[i] = caging[i].as<bool>();
      } catch (const YAML::Exception&) {
        Reject("robot.hand.caging_mask[", std::to_string(i), "] does not parse as a bool");
      }
    } else {
      out.caging_mask[i] = true;  // fail-closed default: check every joint
    }
  }

  // q_open (L6 §5.1): absent or the literal "TBD" leaves the flag set; a
  // present value must be a well-formed array of the pair's length, because
  // defaulting a malformed one would read a typo as "still TBD".
  const YAML::Node q_open = hand_node["q_open"];
  const bool q_open_tbd =
      !q_open || q_open.IsNull() || (q_open.IsScalar() && q_open.Scalar() == "TBD");
  if (!q_open_tbd) {
    ReadHandPoseArray(q_open, "q_open", dof, out.q_open);
    out.q_open_tbd = false;
  }

  out.dof = static_cast<int>(dof);
  out.tbd = false;
  if (!release_timeout_given && !out.T_close_e2e.tbd && !out.eta_close.tbd) {
    double s_max = 0.0;
    for (std::size_t i = 0; i < dof; ++i) {
      s_max = std::max(s_max, std::abs(out.q_close[i] - out.q_pre[i]));
    }
    const double m = ReleaseTimeoutPerE2e(out.eta_close.value, s_max, out.q_tol);
    if (std::isfinite(m)) {
      out.T_release_timeout = TbdDouble::Resolved(m * out.T_close_e2e.value);
      out.T_release_timeout_derived = true;
    }
  }
  return out;
}

BallSpec ReadBallSpec(const YAML::Node& ball_node) {
  BallSpec out;
  out.diameter = ReadTbdDouble(ball_node, "diameter", out.diameter);
  out.mass = ReadTbdDouble(ball_node, "mass", out.mass);
  out.restitution = ReadTbdDouble(ball_node, "restitution", out.restitution);
  out.provisional = ReadOptional(ball_node, "provisional", true);
  return out;
}

}  // namespace

CatchingParams ParseCatchingParams(const YAML::Node& node) {
  if (!node || !node.IsMap()) {
    Reject("must be a map (the `catching:` tree, CATCHING_MASTER.md §6)");
  }
  CatchingParams out;

  const YAML::Node reference = ReadSection(node, "reference");
  out.reference_omega = ReadTbdDouble(reference, "omega", out.reference_omega);
  out.reference_zeta = ReadTbdDouble(reference, "zeta", out.reference_zeta);
  out.reference_v_max = ReadTbdDouble(reference, "v_max", out.reference_v_max);
  out.reference_a_max = ReadTbdDouble(reference, "a_max", out.reference_a_max);
  out.reference_provisional = ReadOptional(reference, "provisional", true);

  const YAML::Node planner = ReadSection(node, "planner");
  const YAML::Node gamma = ReadSection(planner, "gamma");
  out.planner_gamma_eta_v = ReadTbdDouble(gamma, "eta_v", out.planner_gamma_eta_v);

  const YAML::Node catchability = ReadSection(planner, "catchability");
  const YAML::Node manip_min = ReadSection(catchability, "manipulability_min");
  out.planner_catchability_manip_min_arm5row =
      ReadTbdDouble(manip_min, "arm_5row", out.planner_catchability_manip_min_arm5row);
  out.planner_catchability_manip_min_provisional = ReadOptional(manip_min, "provisional", true);

  const YAML::Node supervisor = ReadSection(node, "supervisor");
  const YAML::Node decel = ReadSection(supervisor, "decel");
  out.supervisor_decel_a_dec = ReadTbdDouble(decel, "a_dec", out.supervisor_decel_a_dec);

  const YAML::Node io = ReadSection(node, "io");
  out.io_n_min = ReadPositiveCount(io, "n_min");
  out.io_t_stale = ReadTbdDouble(io, "t_stale", out.io_t_stale);
  out.io_future_tol = ReadTbdDouble(io, "future_tol", out.io_future_tol);
  out.io_horizon_min = ReadTbdDouble(io, "horizon_min", out.io_horizon_min);
  const YAML::Node track = ReadSection(io, "track");
  out.io_track_eval_offset = ReadTbdDouble(track, "eval_offset", out.io_track_eval_offset);
  out.io_track_j_warn = ReadTbdDouble(track, "j_warn", out.io_track_j_warn);

  const YAML::Node prediction = ReadSection(node, "prediction");
  out.prediction_dt_expected = ReadTbdDouble(prediction, "dt_expected", out.prediction_dt_expected);

  const YAML::Node joint_cmd = ReadSection(node, "joint_cmd");
  out.joint_cmd_k_p = ReadTbdDouble(joint_cmd, "K_p", out.joint_cmd_k_p);
  out.joint_cmd_k_axis = ReadTbdDouble(joint_cmd, "K_a", out.joint_cmd_k_axis);
  out.joint_cmd_k_posture = ReadTbdDouble(joint_cmd, "K_n", out.joint_cmd_k_posture);
  out.joint_cmd_w_task = ReadTbdDouble(joint_cmd, "w_task", out.joint_cmd_w_task);
  out.joint_cmd_w_axis = ReadTbdDouble(joint_cmd, "w_a", out.joint_cmd_w_axis);
  out.joint_cmd_w_arm = ReadTbdDouble(joint_cmd, "w_arm", out.joint_cmd_w_arm);
  out.joint_cmd_w_smooth = ReadTbdDouble(joint_cmd, "w_smooth", out.joint_cmd_w_smooth);
  out.joint_cmd_damping_sq = ReadTbdDouble(joint_cmd, "damping_sq", out.joint_cmd_damping_sq);
  {
    const std::string form = ReadOptional<std::string>(joint_cmd, "accel_constraint", "box");
    if (form == "box") {
      out.joint_cmd_accel_constraint = CatchingAccelConstraint::kBox;
    } else if (form == "kinematic") {
      out.joint_cmd_accel_constraint = CatchingAccelConstraint::kKinematic;
    } else if (form == "dynamic") {
      out.joint_cmd_accel_constraint = CatchingAccelConstraint::kDynamic;
    } else {
      Reject("'joint_cmd.accel_constraint' must be box, kinematic or dynamic, got '", form, "'");
    }
    // A form's keys under another form would read as if they were in force.
    const auto form_key = [&](const char* key, CatchingAccelConstraint owner, const char* name) {
      if (joint_cmd[key] && out.joint_cmd_accel_constraint != owner) {
        Reject("'joint_cmd.", key, "' belongs to accel_constraint ", name,
               " but the selected form is ", form);
      }
    };
    form_key("task_accel_max_linear", CatchingAccelConstraint::kKinematic, "kinematic");
    form_key("task_accel_max_angular", CatchingAccelConstraint::kKinematic, "kinematic");
    form_key("eta_tau", CatchingAccelConstraint::kDynamic, "dynamic");
  }
  out.joint_cmd_task_accel_max_linear =
      ReadTbdDouble(joint_cmd, "task_accel_max_linear", out.joint_cmd_task_accel_max_linear);
  out.joint_cmd_task_accel_max_angular =
      ReadTbdDouble(joint_cmd, "task_accel_max_angular", out.joint_cmd_task_accel_max_angular);
  out.joint_cmd_eta_tau = ReadTbdDouble(joint_cmd, "eta_tau", out.joint_cmd_eta_tau);
  const YAML::Node qp = ReadSection(joint_cmd, "qp");
  out.joint_cmd_max_iter = ReadPositiveCount(qp, "max_iter", out.joint_cmd_max_iter);
  const YAML::Node lag = ReadSection(joint_cmd, "lag");
  out.joint_cmd_lag_t_arm = ReadTbdDouble(lag, "T_arm", out.joint_cmd_lag_t_arm);
  out.joint_cmd_lag_lead_enable = ReadOptional(lag, "lead_enable", false);

  const YAML::Node robot_arm = ReadSection(ReadSection(node, "robot"), "arm");
  out.robot_arm_limit_margin = ReadTbdDouble(robot_arm, "limit_margin", out.robot_arm_limit_margin);

  out.supervisor_track_err_abort =
      ReadTbdDouble(supervisor, "track_err_abort", out.supervisor_track_err_abort);
  out.supervisor_n_qp = ReadPositiveCount(supervisor, "n_qp", out.supervisor_n_qp);
  out.supervisor_stale_committed_max_s =
      ReadTbdDouble(supervisor, "stale_committed_max_s", out.supervisor_stale_committed_max_s);
  out.supervisor_sat_ticks = ReadPositiveCount(supervisor, "sat_ticks", out.supervisor_sat_ticks);
  const YAML::Node homing = ReadSection(supervisor, "homing");
  out.supervisor_homing_v_max = ReadOptional(homing, "v_max", out.supervisor_homing_v_max);
  out.supervisor_homing_eta_a = ReadOptional(homing, "eta_a", out.supervisor_homing_eta_a);
  out.supervisor_homing_qd_tol = ReadOptional(homing, "qd_tol", out.supervisor_homing_qd_tol);
  const YAML::Node ready = ReadSection(supervisor, "ready");
  if (ready["wait_pose"]) {
    // L7 §6 once named the wait pose here as well as under `planner`. Two
    // keys for one pose is how they end up different (#537 S7, C-10).
    Reject("supervisor.ready.wait_pose was removed — the wait pose is planner.wait_pose");
  }
  out.supervisor_ready_pose_tol = ReadOptional(ready, "pose_tol", out.supervisor_ready_pose_tol);
  const YAML::Node contact = ReadSection(supervisor, "contact");
  out.supervisor_contact_f_min = ReadOptional(contact, "f_min", out.supervisor_contact_f_min);
  out.supervisor_contact_k_sigma = ReadOptional(contact, "k_sigma", out.supervisor_contact_k_sigma);
  out.supervisor_contact_n_debounce =
      ReadPositiveCount(contact, "n_debounce", out.supervisor_contact_n_debounce);
  out.supervisor_contact_m_min = ReadPositiveCount(contact, "m_min", out.supervisor_contact_m_min);
  out.supervisor_contact_t_confirm =
      ReadOptional(contact, "T_confirm", out.supervisor_contact_t_confirm);
  out.supervisor_contact_t_stale = ReadOptional(contact, "t_stale", out.supervisor_contact_t_stale);
  out.supervisor_contact_baseline_alpha =
      ReadOptional(contact, "baseline_alpha", out.supervisor_contact_baseline_alpha);
  out.supervisor_contact_n_baseline_min =
      ReadPositiveCount(contact, "n_baseline_min", out.supervisor_contact_n_baseline_min);

  const YAML::Node core = ReadSection(node, "core");
  out.ball = ReadBallSpec(ReadSection(core, "ball"));

  const YAML::Node sim = ReadSection(node, "sim");
  out.sim_ball_drag_k = ReadTbdDouble(ReadSection(sim, "ball"), "drag_k", out.sim_ball_drag_k);
  // The sim OVERLAY of a shared key (A-S5-2). Nested under `sim:` for the same
  // reason `sim.ball.drag_k` is: this YAML is loaded by both the sim and the
  // hardware bring-up, so "sim only" has to be said in the schema rather than
  // in which file the value happens to sit.
  out.sim_io_future_tol =
      ReadTbdDouble(ReadSection(sim, "io"), "future_tol", out.sim_io_future_tol);

  const YAML::Node robot = ReadSection(node, "robot");
  out.hand = ReadHandProfile(ReadSection(robot, "hand"));

  return out;
}

namespace {

// ── Validation ────────────────────────────────────────────────────────────
// L4 §4.7 discrete stability boundary s = omega*h: unstable at/above
// 2*sqrt(2) - 2, accuracy recommendation is s <= 0.05. Written as literals
// (not std::sqrt) so the bound stays a compile-time constant without relying
// on a constexpr math function the standard does not guarantee pre-C++23.
inline constexpr double kSqrt2 = 1.4142135623730951;
inline constexpr double kDiscreteStabilityLimit = 2.0 * kSqrt2 - 2.0;  // ~0.8284271 (L4 §4.7)
inline constexpr double kDiscreteAccuracyWarnLimit = 0.05;             // L4 §4.7

void AddFailure(CatchingValidationReport& report, CatchingValidationReason reason, const char* key,
                int index = -1) noexcept {
  report.armable = false;
  if (report.failure_count < CatchingValidationReport::kMaxFailures) {
    report.failures[report.failure_count++] = CatchingValidationEntry{reason, key, index};
  }
}

void AddWarning(CatchingValidationReport& report, CatchingValidationReason reason, const char* key,
                int index = -1) noexcept {
  if (report.warning_count < CatchingValidationReport::kMaxWarnings) {
    report.warnings[report.warning_count++] = CatchingValidationEntry{reason, key, index};
  }
}

/// Active-config TBD gate (G0-C). Returns true if the caller should go on to
/// check `v.value` (i.e. `v` is active and resolved).
bool CheckActiveTbd(CatchingValidationReport& report, const TbdDouble& v, const char* key,
                    bool active) noexcept {
  if (!active) {
    return false;  // inactive-config key: TBD or not, it is not evaluated (G0-C)
  }
  if (v.tbd) {
    AddFailure(report, CatchingValidationReason::kActiveConfigTbd, key);
    return false;
  }
  return true;
}

void CheckRange(CatchingValidationReport& report, const char* key, double value, double lo,
                double hi) noexcept {
  if (!std::isfinite(value) || value < lo || value > hi) {
    AddFailure(report, CatchingValidationReason::kRangeViolation, key);
  }
}

void CheckPositive(CatchingValidationReport& report, const char* key, double value) noexcept {
  if (!std::isfinite(value) || !(value > 0.0)) {
    AddFailure(report, CatchingValidationReason::kRangeViolation, key);
  }
}

/// Applies a provisional flag's L0 §5.3 rule: sim warns, real-arm blocks.
void CheckProvisional(CatchingValidationReport& report, const char* key, bool provisional,
                      bool real_arm_config) noexcept {
  if (!provisional) {
    return;
  }
  if (real_arm_config) {
    AddFailure(report, CatchingValidationReason::kProvisionalOnRealArm, key);
  } else {
    AddWarning(report, CatchingValidationReason::kProvisionalWarning, key);
  }
}

}  // namespace

void CheckCatchFrameProvisional(CatchingValidationReport& report, bool catch_frame_provisional,
                                bool real_arm_config) noexcept {
  CheckProvisional(report, kCatchFrameProvisionalKey, catch_frame_provisional, real_arm_config);
}

void CheckFreezeCoversClose(CatchingValidationReport& report, const CatchingParams& params,
                            double t_freeze_s, double control_rate_hz) noexcept {
  if (!std::isfinite(t_freeze_s) || params.hand.T_close_e2e.tbd || params.joint_cmd_lag_t_arm.tbd ||
      !std::isfinite(control_rate_hz) || !(control_rate_hz > 0.0)) {
    return;  // an unset value is reported by the check that owns it
  }
  const double h = 1.0 / control_rate_hz;
  const double need = params.hand.T_close_e2e.value + params.joint_cmd_lag_t_arm.value + h;
  if (!(t_freeze_s >= need)) {
    AddFailure(report, CatchingValidationReason::kFreezeShorterThanClose, kFreezeWindowKey);
  }
}

CatchingValidationReport ValidateCatchingParams(const CatchingParams& params,
                                                double control_rate_hz,
                                                bool real_arm_config) noexcept {
  CatchingValidationReport report;

  // control_rate (rtc_base-owned key; range is invariants.md §RT Path SSoT).
  const bool rate_ok = std::isfinite(control_rate_hz) &&
                       control_rate_hz >= rtc::kMinControlRateHz &&
                       control_rate_hz <= rtc::kMaxControlRateHz;
  if (!rate_ok) {
    AddFailure(report, CatchingValidationReason::kControlRateOutOfRange, "control_rate");
  }

  // reference.omega / zeta / v_max / a_max — active in every configuration.
  if (CheckActiveTbd(report, params.reference_omega, "reference.omega", true)) {
    CheckRange(report, "reference.omega", params.reference_omega.value, 1.0, 25.0);
  }
  if (CheckActiveTbd(report, params.reference_zeta, "reference.zeta", true)) {
    if (params.reference_zeta.value != 1.0) {
      AddFailure(report, CatchingValidationReason::kZetaNotCriticallyDamped, "reference.zeta");
    }
  }
  if (CheckActiveTbd(report, params.reference_v_max, "reference.v_max", true)) {
    CheckPositive(report, "reference.v_max", params.reference_v_max.value);
  }
  // The block as a whole (L0 §5.3). Keyed on `reference` without a trailing
  // dot — the flag is a property of the profile, not of a field of it, exactly
  // as `robot.hand` and `core.ball` are. A consumer's gate has to match the
  // dotless key too, or a provisional reference reaches a real arm unnoticed
  // (the hand profile shipped with that hole once).
  CheckProvisional(report, "reference", params.reference_provisional, real_arm_config);
  const bool a_max_ok = CheckActiveTbd(report, params.reference_a_max, "reference.a_max", true);
  if (a_max_ok) {
    CheckPositive(report, "reference.a_max", params.reference_a_max.value);
  }

  // planner.gamma.eta_v — D-9: 0 < eta_v <= 1.
  if (CheckActiveTbd(report, params.planner_gamma_eta_v, "planner.gamma.eta_v", true)) {
    const double eta_v = params.planner_gamma_eta_v.value;
    if (!std::isfinite(eta_v) || !(eta_v > 0.0) || !(eta_v <= 1.0)) {
      AddFailure(report, CatchingValidationReason::kEtaVOutOfRange, "planner.gamma.eta_v");
    }
  }

  // planner.catchability.manipulability_min.arm_5row (D-18) + its provisional flag.
  const bool manip_resolved =
      CheckActiveTbd(report, params.planner_catchability_manip_min_arm5row,
                     "planner.catchability.manipulability_min.arm_5row", true);
  if (manip_resolved) {
    CheckRange(report, "planner.catchability.manipulability_min.arm_5row",
               params.planner_catchability_manip_min_arm5row.value, 0.0,
               std::numeric_limits<double>::infinity());
    CheckProvisional(report, "planner.catchability.manipulability_min",
                     params.planner_catchability_manip_min_provisional, real_arm_config);
  }

  // supervisor.decel.a_dec — > 0 and <= reference.a_max (L7 §4.3, single-source key).
  const bool a_dec_ok =
      CheckActiveTbd(report, params.supervisor_decel_a_dec, "supervisor.decel.a_dec", true);
  if (a_dec_ok) {
    CheckPositive(report, "supervisor.decel.a_dec", params.supervisor_decel_a_dec.value);
    if (a_max_ok && params.supervisor_decel_a_dec.value > params.reference_a_max.value) {
      AddFailure(report, CatchingValidationReason::kDecelExceedsAMax, "supervisor.decel.a_dec");
    }
  }

  // core.ball.* — active in every configuration (L0 §6: "제어 경로에도 필요").
  const bool diameter_ok = CheckActiveTbd(report, params.ball.diameter, "core.ball.diameter", true);
  if (diameter_ok) {
    CheckRange(report, "core.ball.diameter", params.ball.diameter.value, 0.02, 0.3);
  }
  const bool mass_ok = CheckActiveTbd(report, params.ball.mass, "core.ball.mass", true);
  if (mass_ok) {
    CheckRange(report, "core.ball.mass", params.ball.mass.value, 0.005, 1.0);
  }
  const bool restitution_ok =
      CheckActiveTbd(report, params.ball.restitution, "core.ball.restitution", true);
  if (restitution_ok) {
    CheckRange(report, "core.ball.restitution", params.ball.restitution.value, 0.0, 1.0);
  }
  if (diameter_ok && mass_ok && restitution_ok) {
    CheckProvisional(report, "core.ball", params.ball.provisional, real_arm_config);
  }

  // sim.ball.drag_k — active ONLY in the sim configuration (fixture-only, L0 §1/§6).
  if (CheckActiveTbd(report, params.sim_ball_drag_k, "sim.ball.drag_k", !real_arm_config)) {
    CheckRange(report, "sim.ball.drag_k", params.sim_ball_drag_k.value, 0.0, 0.2);
  }

  // io.* / prediction.* — the vision ingress, active in every configuration
  // from S5.2 (the controller cannot judge a message's order, age or usable
  // window without them, and every one of those judgements is fail-closed).
  if (params.io_n_min >= 2) {
    if (params.io_n_min > kCap) {
      // A requirement above the snapshot capacity can never be met, so the
      // lane would be permanently closed with the rejection counter blaming
      // the publisher.
      AddFailure(report, CatchingValidationReason::kRangeViolation, "io.n_min");
    }
  } else if (params.io_n_min <= 0) {
    AddFailure(report, CatchingValidationReason::kActiveConfigTbd, "io.n_min");
  } else {
    AddFailure(report, CatchingValidationReason::kRangeViolation, "io.n_min");
  }

  if (CheckActiveTbd(report, params.io_t_stale, "io.t_stale", true)) {
    CheckRange(report, "io.t_stale", params.io_t_stale.value, 0.02, 0.2);
  }
  if (CheckActiveTbd(report, params.io_horizon_min, "io.horizon_min", true)) {
    CheckPositive(report, "io.horizon_min", params.io_horizon_min.value);
  }
  if (CheckActiveTbd(report, params.io_track_eval_offset, "io.track.eval_offset", true)) {
    CheckRange(report, "io.track.eval_offset", params.io_track_eval_offset.value, 0.0, 0.3);
  }
  // j_warn is NOT active: it is the threshold of a printed diagnostic, and no
  // decision is taken on it. Gating arming on a value nothing consumes would
  // make the operator resolve a number to get a warning they may not want.
  if (!params.io_track_j_warn.tbd) {
    CheckPositive(report, "io.track.j_warn", params.io_track_j_warn.value);
  }
  if (CheckActiveTbd(report, params.prediction_dt_expected, "prediction.dt_expected", true)) {
    CheckRange(report, "prediction.dt_expected", params.prediction_dt_expected.value, 1e-3, 1.0);
  }

  // future_tol: the shared key is active on both axes; the sim override is
  // active only in sim (A-S5-2), exactly like sim.ball.drag_k.
  if (CheckActiveTbd(report, params.io_future_tol, "io.future_tol", true)) {
    CheckRange(report, "io.future_tol", params.io_future_tol.value, 1e-4, 1e-2);
  }
  if (!real_arm_config && !params.sim_io_future_tol.tbd) {
    // Its own, much wider range: the sim ball lane's stamps ride the sim time
    // axis and lead wall by the in-flight phase error (D-3), which is two
    // orders of magnitude above any clock-sync budget. An absent override is
    // not a failure — the configuration then inherits the strict shared value,
    // which is the fail-closed direction.
    CheckRange(report, "sim.io.future_tol", params.sim_io_future_tol.value, 1e-4, 0.5);
  }

  // Consistency: a point-count floor below what the window requires cannot
  // enforce it. Both keys are individually in range in that case, so nothing
  // else catches a pair that disagrees — and the symptom would be messages
  // accepted on count and then rejected on horizon, which reads as a vision
  // fault rather than as a configuration one.
  if (params.io_n_min >= 2 && !params.io_horizon_min.tbd && !params.prediction_dt_expected.tbd &&
      params.prediction_dt_expected.value > 0.0) {
    // n samples spaced dt apart span (n-1)*dt, NOT n*dt. The off-by-one is not
    // academic: with the shipped 0.51 s / 0.05 s it is the difference between
    // 11 points (0.50 s — short, so every minimal message trips the horizon
    // warning this check exists to prevent) and 12.
    const double needed = params.io_horizon_min.value / params.prediction_dt_expected.value + 1.0;
    if (std::isfinite(needed) && static_cast<double>(params.io_n_min) < std::ceil(needed)) {
      AddFailure(report, CatchingValidationReason::kRangeViolation, "io.n_min");
    }
  }

  // joint_cmd.* / robot.arm.* — the CLIK step, active from S5.3.
  if (CheckActiveTbd(report, params.joint_cmd_k_p, "joint_cmd.K_p", true)) {
    CheckRange(report, "joint_cmd.K_p", params.joint_cmd_k_p.value, 1.0, 100.0);
  }
  if (CheckActiveTbd(report, params.joint_cmd_k_axis, "joint_cmd.K_a", true)) {
    CheckRange(report, "joint_cmd.K_a", params.joint_cmd_k_axis.value, 1e-3, 30.0);
  }
  if (CheckActiveTbd(report, params.joint_cmd_k_posture, "joint_cmd.K_n", true)) {
    CheckRange(report, "joint_cmd.K_n", params.joint_cmd_k_posture.value, 0.0, 10.0);
  }
  if (CheckActiveTbd(report, params.joint_cmd_damping_sq, "joint_cmd.damping_sq", true)) {
    CheckPositive(report, "joint_cmd.damping_sq", params.joint_cmd_damping_sq.value);
  }
  if (CheckActiveTbd(report, params.joint_cmd_w_smooth, "joint_cmd.w_smooth", true)) {
    CheckRange(report, "joint_cmd.w_smooth", params.joint_cmd_w_smooth.value, 0.0, 1.0);
  }
  if (params.joint_cmd_max_iter < 1) {
    AddFailure(report, CatchingValidationReason::kRangeViolation, "joint_cmd.qp.max_iter");
  }
  // decision K: each acceleration form's keys are active only when it is the
  // selected form (an unselected form's TBD is not a reason to refuse).
  const bool kinematic = params.joint_cmd_accel_constraint == CatchingAccelConstraint::kKinematic;
  const bool dynamic = params.joint_cmd_accel_constraint == CatchingAccelConstraint::kDynamic;
  if (CheckActiveTbd(report, params.joint_cmd_task_accel_max_linear,
                     "joint_cmd.task_accel_max_linear", kinematic)) {
    CheckRange(report, "joint_cmd.task_accel_max_linear",
               params.joint_cmd_task_accel_max_linear.value, 1e-3, 500.0);
  }
  if (CheckActiveTbd(report, params.joint_cmd_task_accel_max_angular,
                     "joint_cmd.task_accel_max_angular", kinematic)) {
    CheckRange(report, "joint_cmd.task_accel_max_angular",
               params.joint_cmd_task_accel_max_angular.value, 1e-3, 1000.0);
  }
  if (CheckActiveTbd(report, params.joint_cmd_eta_tau, "joint_cmd.eta_tau", dynamic)) {
    CheckRange(report, "joint_cmd.eta_tau", params.joint_cmd_eta_tau.value, 1e-3, 1.0);
  }
  if (CheckActiveTbd(report, params.joint_cmd_lag_t_arm, "joint_cmd.lag.T_arm", true)) {
    CheckRange(report, "joint_cmd.lag.T_arm", params.joint_cmd_lag_t_arm.value, 0.0, 0.5);
  }
  if (CheckActiveTbd(report, params.robot_arm_limit_margin, "robot.arm.limit_margin", true)) {
    CheckRange(report, "robot.arm.limit_margin", params.robot_arm_limit_margin.value, 0.0, 0.3);
  }

  // The WEIGHT ORDERING, not just three ranges (L5 §4.3: w_task ≫ w_arm ≫ μ²).
  // Each weight can be individually sensible while the set is wrong, and the
  // result is a controller that tracks its posture and treats the catch point
  // as a suggestion — plausible motion, missed ball, nothing in any log that
  // says why.
  //
  // The three checks are evaluated into locals FIRST. Written as a chained
  // `&&` they short-circuit, so a profile with all three TBD reported only
  // `w_task` — the operator resolves it, re-configures, and is told about
  // `w_a`; resolves that, re-configures, and is told about `w_arm`. Three
  // round trips for one report, while every other block in this validator
  // lists its failures in full. `CheckActiveTbd` is here for its side effect
  // on `report`, so the ordering of the AND is not a detail.
  const bool task_known = CheckActiveTbd(report, params.joint_cmd_w_task, "joint_cmd.w_task", true);
  const bool axis_known = CheckActiveTbd(report, params.joint_cmd_w_axis, "joint_cmd.w_a", true);
  const bool arm_known = CheckActiveTbd(report, params.joint_cmd_w_arm, "joint_cmd.w_arm", true);
  const bool weights_resolved = task_known && axis_known && arm_known;
  if (weights_resolved) {
    CheckPositive(report, "joint_cmd.w_task", params.joint_cmd_w_task.value);
    CheckPositive(report, "joint_cmd.w_a", params.joint_cmd_w_axis.value);
    if (!(params.joint_cmd_w_arm.value >= 0.0)) {
      AddFailure(report, CatchingValidationReason::kRangeViolation, "joint_cmd.w_arm");
    }
    const bool task_dominates = params.joint_cmd_w_task.value > params.joint_cmd_w_arm.value;
    const bool axis_dominates = params.joint_cmd_w_axis.value > params.joint_cmd_w_arm.value;
    if (!task_dominates || !axis_dominates) {
      AddFailure(report, CatchingValidationReason::kWeightOrdering, "joint_cmd.w_arm");
    }
    if (!params.joint_cmd_damping_sq.tbd &&
        !(params.joint_cmd_w_arm.value > params.joint_cmd_damping_sq.value)) {
      AddFailure(report, CatchingValidationReason::kWeightOrdering, "joint_cmd.damping_sq");
    }
  }

  // supervisor.track_err_abort / n_qp — L7 owns both keys; the joint command
  // layer reports to them, so they are active as soon as it exists.
  if (CheckActiveTbd(report, params.supervisor_track_err_abort, "supervisor.track_err_abort",
                     true)) {
    CheckPositive(report, "supervisor.track_err_abort", params.supervisor_track_err_abort.value);
  }
  if (params.supervisor_n_qp <= 0) {
    AddFailure(report, CatchingValidationReason::kActiveConfigTbd, "supervisor.n_qp");
  }
  // The S7.2 driver's keys (L7 §6).
  if (CheckActiveTbd(report, params.supervisor_stale_committed_max_s,
                     "supervisor.stale_committed_max_s", true)) {
    CheckRange(report, "supervisor.stale_committed_max_s",
               params.supervisor_stale_committed_max_s.value, 0.0, 1.0);
  }
  if (params.supervisor_sat_ticks <= 0) {
    AddFailure(report, CatchingValidationReason::kActiveConfigTbd, "supervisor.sat_ticks");
  }
  CheckPositive(report, "supervisor.homing.v_max", params.supervisor_homing_v_max);
  if (!std::isfinite(params.supervisor_homing_eta_a) || !(params.supervisor_homing_eta_a > 0.0) ||
      params.supervisor_homing_eta_a > 1.0) {
    AddFailure(report, CatchingValidationReason::kRangeViolation, "supervisor.homing.eta_a");
  }
  CheckPositive(report, "supervisor.homing.qd_tol", params.supervisor_homing_qd_tol);
  CheckPositive(report, "supervisor.ready.pose_tol", params.supervisor_ready_pose_tol);
  CheckRange(report, "supervisor.contact.f_min", params.supervisor_contact_f_min, 0.0,
             std::numeric_limits<double>::infinity());
  CheckRange(report, "supervisor.contact.k_sigma", params.supervisor_contact_k_sigma, 0.0,
             std::numeric_limits<double>::infinity());
  CheckRange(report, "supervisor.contact.T_confirm", params.supervisor_contact_t_confirm, 0.0, 1.0);
  if (!std::isfinite(params.supervisor_contact_t_stale) ||
      !(params.supervisor_contact_t_stale > 0.0) || params.supervisor_contact_t_stale > 0.5) {
    AddFailure(report, CatchingValidationReason::kRangeViolation, "supervisor.contact.t_stale");
  }
  if (!std::isfinite(params.supervisor_contact_baseline_alpha) ||
      !(params.supervisor_contact_baseline_alpha > 0.0) ||
      params.supervisor_contact_baseline_alpha > 1.0) {
    AddFailure(report, CatchingValidationReason::kRangeViolation,
               "supervisor.contact.baseline_alpha");
  }
  for (const auto& [value, key] :
       {std::pair{params.supervisor_contact_n_debounce, "supervisor.contact.n_debounce"},
        std::pair{params.supervisor_contact_m_min, "supervisor.contact.m_min"},
        std::pair{params.supervisor_contact_n_baseline_min, "supervisor.contact.n_baseline_min"}}) {
    if (value <= 0) {
      AddFailure(report, CatchingValidationReason::kActiveConfigTbd, key);
    }
  }

  // robot.hand.* — active in every configuration.
  //
  // eta_close / T_close_e2e are checked outside the profile branch: each is an
  // independent key with its own report line, so a TBD pose pair does not hide
  // a second missing value behind one failure (L6 §6).
  if (CheckActiveTbd(report, params.hand.eta_close, "robot.hand.eta_close", true)) {
    CheckRange(report, "robot.hand.eta_close", params.hand.eta_close.value, 0.5, 1.0);
  }
  if (CheckActiveTbd(report, params.hand.T_close_e2e, "robot.hand.T_close_e2e", true)) {
    CheckRange(report, "robot.hand.T_close_e2e", params.hand.T_close_e2e.value, 0.0,
               std::numeric_limits<double>::infinity());
  }
  if (params.hand.tbd) {
    AddFailure(report, CatchingValidationReason::kActiveConfigTbd, "robot.hand.q_pre/q_close");
  } else {
    if (params.hand.q_open_tbd) {
      AddFailure(report, CatchingValidationReason::kActiveConfigTbd, "robot.hand.q_open");
    }
    CheckPositive(report, "robot.hand.rho_eps", params.hand.rho_eps);
    for (int i = 0; i < params.hand.dof; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      if (!params.hand.caging_mask[idx]) {
        continue;
      }
      const double gap = std::abs(params.hand.q_close[idx] - params.hand.q_pre[idx]);
      if (!(gap > params.hand.rho_eps)) {
        AddFailure(report, CatchingValidationReason::kHandCagingGapTooSmall, "robot.hand.q_close",
                   i);
      }
    }
    CheckProvisional(report, "robot.hand", params.hand.provisional, real_arm_config);
  }
  // The sequencer's keys (S7.1, L6 §6). Independent report lines, like η and
  // T_close_e2e above, so one missing value does not hide the next.
  if (CheckActiveTbd(report, params.hand.T_hold, "robot.hand.T_hold", true)) {
    CheckRange(report, "robot.hand.T_hold", params.hand.T_hold.value, 0.0, 5.0);
  }
  if (CheckActiveTbd(report, params.hand.T_close_timeout, "robot.hand.T_close_timeout", true)) {
    CheckPositive(report, "robot.hand.T_close_timeout", params.hand.T_close_timeout.value);
    // Strictly above: a timeout equal to the identified closure time fires on
    // every nominal close, which turns the flag into noise.
    if (!params.hand.T_close_e2e.tbd &&
        !(params.hand.T_close_timeout.value > params.hand.T_close_e2e.value)) {
      AddFailure(report, CatchingValidationReason::kCloseTimeoutNotAboveE2e,
                 "robot.hand.T_close_timeout");
    }
  }
  // D-S8-6. TBD exactly when T_close_e2e or the poses are (derived from them).
  if (CheckActiveTbd(report, params.hand.T_release_timeout, "robot.hand.T_release_timeout", true)) {
    CheckPositive(report, "robot.hand.T_release_timeout", params.hand.T_release_timeout.value);
    // Strictly above, as for the close: this also refuses the 0 s timeout a
    // T_close_e2e of 0 would derive, which would disarm on every release.
    if (!params.hand.T_close_e2e.tbd &&
        !(params.hand.T_release_timeout.value > params.hand.T_close_e2e.value)) {
      AddFailure(report, CatchingValidationReason::kReleaseTimeoutNotAboveE2e,
                 "robot.hand.T_release_timeout");
    }
  }
  CheckPositive(report, "robot.hand.q_tol", params.hand.q_tol);
  CheckPositive(report, "robot.hand.qd_tol", params.hand.qd_tol);
  // D-S8-8 (b): active only when the section is present.
  if (const HandCaptureParams& cap = params.hand.capture; cap.enabled) {
    const bool rho_min_ok = CheckActiveTbd(report, cap.rho_min, "robot.hand.capture.rho_min", true);
    const bool rho_max_ok = CheckActiveTbd(report, cap.rho_max, "robot.hand.capture.rho_max", true);
    // ρ = 1 is a hand that reached q_close — the empty hand — so the band must
    // stop short of it, and start at or above the pose it closed from. Each
    // bound is reported under its own key; the ordering under rho_max.
    if (rho_min_ok && !(cap.rho_min.value >= 0.0)) {
      AddFailure(report, CatchingValidationReason::kRangeViolation, "robot.hand.capture.rho_min");
    }
    if (rho_max_ok && !(cap.rho_max.value < 1.0)) {
      AddFailure(report, CatchingValidationReason::kRangeViolation, "robot.hand.capture.rho_max");
    } else if (rho_min_ok && rho_max_ok && cap.rho_min.value >= 0.0 &&
               !(cap.rho_max.value > cap.rho_min.value)) {
      AddFailure(report, CatchingValidationReason::kRangeViolation, "robot.hand.capture.rho_max");
    }
    if (CheckActiveTbd(report, cap.effort_frac_min, "robot.hand.capture.effort_frac_min", true) &&
        !(cap.effort_frac_min.value > 0.0 && cap.effort_frac_min.value <= 1.0)) {
      AddFailure(report, CatchingValidationReason::kRangeViolation,
                 "robot.hand.capture.effort_frac_min");
    }
    // The witness reads "stopped short of q_close" as a ball, which holds only
    // while Hold commands q_close: under measured_offset an EMPTY hand settles
    // at its η crossing plus delta_rad — inside the band.
    if (params.hand.hold_mode != HandHoldMode::kCloseTarget) {
      AddFailure(report, CatchingValidationReason::kRangeViolation, "robot.hand.capture");
    }
    if (CheckActiveTbd(report, cap.t_persist, "robot.hand.capture.t_persist", true)) {
      // It has to fit inside HOLD, which is when the evidence can accrue.
      const double t_hold = params.hand.T_hold.tbd ? std::numeric_limits<double>::infinity()
                                                   : params.hand.T_hold.value;
      if (!std::isfinite(cap.t_persist.value) || cap.t_persist.value < 0.0 ||
          !(cap.t_persist.value < t_hold)) {
        AddFailure(report, CatchingValidationReason::kRangeViolation,
                   "robot.hand.capture.t_persist");
      }
    }
    int caging = 0;
    for (int i = 0; i < params.hand.dof; ++i) {
      caging += params.hand.caging_mask[static_cast<std::size_t>(i)] ? 1 : 0;
    }
    if (cap.min_joints < 1 || (!params.hand.tbd && cap.min_joints > caging)) {
      AddFailure(report, CatchingValidationReason::kRangeViolation,
                 "robot.hand.capture.min_joints");
    }
    CheckProvisional(report, "robot.hand.capture", cap.provisional, real_arm_config);
  }
  if (params.hand.hold_mode == HandHoldMode::kMeasuredOffset &&
      (!std::isfinite(params.hand.hold_delta_rad) || params.hand.hold_delta_rad < 0.0)) {
    AddFailure(report, CatchingValidationReason::kRangeViolation, "robot.hand.hold.delta_rad");
  }

  // ζ·ω·h (`dt` 기준, L4 §4.7): only meaningful once control_rate and omega
  // both resolved to a real number.
  if (rate_ok && !params.reference_omega.tbd) {
    const double h = 1.0 / control_rate_hz;
    const double s = params.reference_omega.value * h;
    if (s >= kDiscreteStabilityLimit) {
      AddFailure(report, CatchingValidationReason::kUnstableDiscretization, "reference.omega");
    } else if (s > kDiscreteAccuracyWarnLimit) {
      AddWarning(report, CatchingValidationReason::kDiscretizationAccuracy, "reference.omega");
    }
  }

  return report;
}

}  // namespace rtc::catching
