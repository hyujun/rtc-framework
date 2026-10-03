// Parser for `catching.planner.*` (S6). See planner_params.hpp.
#include "rtc_controllers/catching/planner_params.hpp"

#include "catching_yaml_read.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace rtc::catching {

namespace {

using params_detail::ReadSectionNode;
using params_detail::SectionKind;
using params_detail::Spelling;

[[noreturn]] void Reject(const std::string& what) {
  throw std::invalid_argument("ParsePlannerParams: " + what);
}

std::string Key(const std::string& path) {
  return "'planner." + path + "'";
}

/// A number for a message: the YAML text when the key is written (the way the
/// other range errors echo it), else the default in scientific notation —
/// `std::to_string` would print 5e-7 as 0.000000.
std::string Shown(const YAML::Node& sec, const char* key, double value) {
  const YAML::Node v = sec[key];
  if (v) {
    return Spelling(v);
  }
  std::ostringstream os;
  os << std::scientific << std::setprecision(6) << value;
  return os.str();
}

/// The section under `parent`, empty when absent; refuses a non-map.
YAML::Node Section(const YAML::Node& parent, const char* key, const std::string& path) {
  const auto s = ReadSectionNode(parent, key);
  if (s.kind == SectionKind::kNotAMap) {
    Reject(Key(path) + " must be a map");
  }
  return s.node;
}

/// A finite number inside [lo, hi], or the fallback when absent. `TBD` is NOT
/// accepted: these keys have documented defaults (L3 §6), so a TBD here is a
/// profile that meant something and did not say what.
double ReadBounded(const YAML::Node& sec, const char* key, const std::string& path, double fallback,
                   double lo, double hi) {
  const YAML::Node v = sec[key];
  if (!v) {
    return fallback;
  }
  double d = 0.0;
  try {
    d = v.as<double>();
  } catch (const YAML::Exception&) {
    Reject(Key(path) + " must be a number, got " + Spelling(v));
  }
  if (!std::isfinite(d) || d < lo || d > hi) {
    Reject(Key(path) + " = " + Spelling(v) + " is outside [" + std::to_string(lo) + ", " +
           std::to_string(hi) + "] (L3 §6)");
  }
  return d;
}

/// A finite number ≥ 0 with no upper bound, or the fallback when absent — for
/// the keys the consumer itself bounds only from below (a weight or a penalty
/// whose 0 means "off").
double ReadNonNegative(const YAML::Node& sec, const char* key, const std::string& path,
                       double fallback) {
  const YAML::Node v = sec[key];
  if (!v) {
    return fallback;
  }
  double d = 0.0;
  try {
    d = v.as<double>();
  } catch (const YAML::Exception&) {
    Reject(Key(path) + " must be a number, got " + Spelling(v));
  }
  if (!std::isfinite(d) || d < 0.0) {
    Reject(Key(path) + " = " + Spelling(v) + " must be a finite number >= 0");
  }
  return d;
}

/// A finite number in an interval whose ends may be open, or the fallback when
/// absent — for the keys whose range is "> 0" or "(0, π)" and where a closed
/// bound at a made-up epsilon would claim a floor the consumer does not have.
/// `text` is the range as the message spells it.
double ReadInterval(const YAML::Node& sec, const char* key, const std::string& path,
                    double fallback, double lo, bool lo_open, double hi, bool hi_open,
                    const char* text) {
  const YAML::Node v = sec[key];
  if (!v) {
    return fallback;
  }
  double d = 0.0;
  try {
    d = v.as<double>();
  } catch (const YAML::Exception&) {
    Reject(Key(path) + " must be a number, got " + Spelling(v));
  }
  const bool lo_ok = lo_open ? d > lo : d >= lo;
  const bool hi_ok = hi_open ? d < hi : d <= hi;
  if (!std::isfinite(d) || !lo_ok || !hi_ok) {
    Reject(Key(path) + " = " + Spelling(v) + " must be a finite number in " + text);
  }
  return d;
}

/// A finite number > 0, or the fallback when absent.
double ReadPositive(const YAML::Node& sec, const char* key, const std::string& path,
                    double fallback) {
  return ReadInterval(sec, key, path, fallback, 0.0, true, std::numeric_limits<double>::infinity(),
                      false, "(0, inf)");
}

/// A DECISION value: absent or `TBD` → NaN (unset, the binding parks);
/// otherwise a finite number inside [lo, hi].
double ReadDecision(const YAML::Node& sec, const char* key, const std::string& path, double lo,
                    double hi) {
  const YAML::Node v = sec[key];
  if (!v || (v.IsScalar() && v.Scalar() == "TBD")) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return ReadBounded(sec, key, path, 0.0, lo, hi);
}

int ReadInt(const YAML::Node& sec, const char* key, const std::string& path, int fallback, int lo,
            int hi) {
  const YAML::Node v = sec[key];
  if (!v) {
    return fallback;
  }
  int i = 0;
  try {
    i = v.as<int>();
  } catch (const YAML::Exception&) {
    Reject(Key(path) + " must be an integer, got " + Spelling(v));
  }
  if (i < lo || i > hi) {
    Reject(Key(path) + " = " + Spelling(v) + " is outside [" + std::to_string(lo) + ", " +
           std::to_string(hi) + "]");
  }
  return i;
}

bool ReadBool(const YAML::Node& sec, const char* key, const std::string& path, bool fallback) {
  const YAML::Node v = sec[key];
  if (!v) {
    return fallback;
  }
  try {
    return v.as<bool>();
  } catch (const YAML::Exception&) {
    Reject(Key(path) + " must be a bool, got " + Spelling(v));
  }
}

std::array<double, 3> Read3(const YAML::Node& v, const std::string& path) {
  if (!v.IsSequence() || v.size() != 3) {
    Reject(Key(path) + " must be three numbers, got " + Spelling(v));
  }
  std::array<double, 3> out{};
  for (std::size_t i = 0; i < 3; ++i) {
    try {
      out[i] = v[i].as<double>();
    } catch (const YAML::Exception&) {
      Reject(Key(path) + "[" + std::to_string(i) + "] must be a number");
    }
    if (!std::isfinite(out[i])) {
      Reject(Key(path) + "[" + std::to_string(i) + "] is not finite");
    }
  }
  return out;
}

}  // namespace

PlannerParams ParsePlannerParams(const YAML::Node& catching) {
  if (!catching || !catching.IsMap()) {
    Reject("must be given the `catching:` map");
  }
  PlannerParams out;
  const auto section = ReadSectionNode(catching, "planner");
  if (section.kind == SectionKind::kNotAMap) {
    Reject("'planner' must be a map");
  }
  if (section.kind == SectionKind::kAbsent) {
    return out;
  }
  const YAML::Node& planner = section.node;

  // ── Thread ─────────────────────────────────────────────────────────────────
  out.enabled = ReadBool(planner, "enabled", "enabled", out.enabled);
  out.provisional = ReadBool(planner, "provisional", "provisional", out.provisional);
  out.wake_timeout_s = ReadBounded(planner, "wake_timeout_s", "wake_timeout_s", out.wake_timeout_s,
                                   kPlannerWakeTimeoutMinS, kPlannerWakeTimeoutMaxS);
  out.budget_s = ReadBounded(planner, "budget_s", "budget_s", out.budget_s, kPlannerBudgetMinS,
                             kPlannerBudgetMaxS);
  if (const YAML::Node pose = planner["wait_pose"]; pose) {
    if (!pose.IsSequence() || pose.size() == 0) {
      Reject(Key("wait_pose") + " must be a non-empty sequence of joint angles, got " +
             Spelling(pose));
    }
    if (pose.size() > out.wait_pose.size()) {
      Reject(Key("wait_pose") + " has " + std::to_string(pose.size()) + " entries; at most " +
             std::to_string(out.wait_pose.size()) + " (kMaxPlanNv)");
    }
    for (std::size_t i = 0; i < pose.size(); ++i) {
      double q = 0.0;
      try {
        q = pose[i].as<double>();
      } catch (const YAML::Exception&) {
        Reject(Key("wait_pose[" + std::to_string(i) + "]") + " must be a number, got " +
               Spelling(pose[i]));
      }
      if (!std::isfinite(q)) {
        Reject(Key("wait_pose[" + std::to_string(i) + "]") + " is not finite");
      }
      out.wait_pose[i] = q;
    }
    out.wait_pose_n = static_cast<std::int32_t>(pose.size());
  }
  if (const YAML::Node v = planner["wait_pose_source"]; v) {
    const std::string spelled = v.IsScalar() ? v.Scalar() : std::string{};
    if (spelled == "yaml") {
      out.wait_pose_source = PlannerParams::WaitPoseSource::kYaml;
    } else if (spelled == "current") {
      out.wait_pose_source = PlannerParams::WaitPoseSource::kCurrent;
    } else {
      Reject(Key("wait_pose_source") + " must be \"yaml\" or \"current\", got " + Spelling(v));
    }
  }

  // ── Search ─────────────────────────────────────────────────────────────────
  if (const YAML::Node v = planner["sub_model"]; v) {
    if (!v.IsScalar() || v.Scalar().empty() || v.Scalar() == "TBD") {
      Reject(Key("sub_model") + " must name a urdf.sub_models entry, got " + Spelling(v));
    }
    out.sub_model = v.Scalar();
  }
  out.max_ik = ReadInt(planner, "max_ik", "max_ik", out.max_ik, 1, kPlannerMaxIkCapacity);
  out.n_settle = ReadInt(planner, "n_settle", "n_settle", out.n_settle, 0, 20);

  const YAML::Node slice = Section(planner, "slice", "slice");
  out.slice_dt = ReadBounded(slice, "dt", "slice.dt", out.slice_dt, 0.005, 0.05);
  out.slice_t_max = ReadBounded(slice, "t_max", "slice.t_max", out.slice_t_max, 0.2, 1.5);
  if (slice["t_lead_min"]) {
    out.slice_t_lead_min = ReadBounded(slice, "t_lead_min", "slice.t_lead_min", 0.0, 1e-3, 1.5);
  }

  const YAML::Node time = Section(planner, "time", "time");
  out.time_margin = ReadBounded(time, "margin", "time.margin", out.time_margin, 0.0, 0.2);

  const YAML::Node unc = Section(planner, "unc", "unc");
  out.kappa_sigma = ReadBounded(unc, "kappa_sigma", "unc.kappa_sigma", out.kappa_sigma, 0.05, 1.0);

  const YAML::Node gamma = Section(planner, "gamma", "gamma");
  out.gamma_margin = ReadBounded(gamma, "margin", "gamma.margin", out.gamma_margin, 0.0, 1.0);
  out.unit_speed_damping = ReadInterval(gamma, "unit_speed_damping", "gamma.unit_speed_damping",
                                        out.unit_speed_damping, 0.0, true, 1.0, false, "(0, 1]");
  out.eta_a = ReadBounded(gamma, "eta_a", "gamma.eta_a", out.eta_a, 1e-3, 1.0);
  out.eps_term = ReadBounded(gamma, "eps_term", "gamma.eps_term", out.eps_term, 1e-6, 1.0);
  const auto read_grid = [](const YAML::Node& sec, const char* key, const std::string& path,
                            auto& dst, std::size_t& n, double lo, double hi) {
    const YAML::Node v = sec[key];
    if (!v) {
      return;
    }
    if (!v.IsSequence() || v.size() == 0 || v.size() > dst.size()) {
      Reject(Key(path) + " must be a sequence of 1.." + std::to_string(dst.size()) +
             " numbers, got " + Spelling(v));
    }
    for (std::size_t i = 0; i < v.size(); ++i) {
      double d = 0.0;
      try {
        d = v[i].as<double>();
      } catch (const YAML::Exception&) {
        Reject(Key(path) + "[" + std::to_string(i) + "] must be a number");
      }
      if (!std::isfinite(d) || d < lo || d > hi) {
        Reject(Key(path) + "[" + std::to_string(i) + "] is outside [" + std::to_string(lo) + ", " +
               std::to_string(hi) + "]");
      }
      dst[i] = d;
    }
    n = v.size();
  };
  read_grid(gamma, "grid", "gamma.grid", out.gamma_grid, out.gamma_grid_n, 0.0, 1.0);
  read_grid(gamma, "window_grid", "gamma.window_grid", out.window_grid, out.window_grid_n, 1e-3,
            2.0);
  const YAML::Node rollout = Section(planner, "rollout", "rollout");
  out.rollout_dt_coarse =
      ReadBounded(rollout, "dt_coarse", "rollout.dt_coarse", out.rollout_dt_coarse, 1e-4, 0.05);

  const YAML::Node budget = Section(planner, "budget", "budget");
  out.n_sigma = ReadBounded(budget, "n_sigma", "budget.n_sigma", out.n_sigma, 1.0, 3.0);
  out.sigma_trk = ReadBounded(budget, "sigma_trk", "budget.sigma_trk", out.sigma_trk, 0.0, 1.0);
  out.clock_err = ReadBounded(budget, "clock_err", "budget.clock_err", out.clock_err, 0.0, 1.0);

  const YAML::Node hand = Section(planner, "hand", "hand");
  out.d_eff = ReadDecision(hand, "d_eff", "hand.d_eff", 1e-4, 10.0);
  out.r_cap = ReadDecision(hand, "r_cap", "hand.r_cap", 1e-4, 1.0);

  const YAML::Node sw = Section(planner, "switch", "switch");
  out.switch_delta_j = ReadBounded(sw, "delta_J", "switch.delta_J", out.switch_delta_j, 0.0, 1e6);
  // The distance limits the acceleration budget replaced (decision ⑥): a
  // profile still carrying them was tuned for the old rule, so it is refused
  // rather than silently run on the eta_jump default.
  for (const char* retired : {"e_jump_max", "ed_jump_max"}) {
    if (sw && sw.IsMap() && sw[retired]) {
      Reject(Key(std::string("switch.") + retired) +
             " was replaced by switch.eta_jump (L3 §4.7, an acceleration budget)");
    }
  }
  out.switch_eta_jump =
      ReadBounded(sw, "eta_jump", "switch.eta_jump", out.switch_eta_jump, 1e-6, 1.0);
  // The step bound divides by samples − 1: fewer than two instants is no ramp.
  // And every sample is a ramp evaluation on the planner thread per switch
  // check, so the count is capped (kSwitchSamplesMax).
  out.switch_samples =
      ReadInt(sw, "samples", "switch.samples", out.switch_samples, 2, kSwitchSamplesMax);

  const YAML::Node freeze = Section(planner, "freeze", "freeze");
  out.t_freeze = ReadDecision(freeze, "T_freeze", "freeze.T_freeze", 1e-3, 2.0);

  const YAML::Node score = Section(planner, "score", "score");
  out.score.w_sigma = ReadBounded(score, "w_sigma", "score.w_sigma", out.score.w_sigma, 0.0, 1e6);
  out.score.w_t = ReadBounded(score, "w_t", "score.w_t", out.score.w_t, 0.0, 1e6);
  out.score.w_q = ReadBounded(score, "w_q", "score.w_q", out.score.w_q, 0.0, 1e6);
  out.score.w_late = ReadBounded(score, "w_late", "score.w_late", out.score.w_late, 0.0, 1e6);
  out.score.w_gamma = ReadBounded(score, "w_gamma", "score.w_gamma", out.score.w_gamma, 0.0, 1e6);
  out.score.penalty = ReadBounded(score, "penalty", "score.penalty", out.score.penalty, 0.0, 1e9);

  const YAML::Node workspace = Section(planner, "workspace", "workspace");
  if (const YAML::Node box = workspace["catch_box"];
      box && !(box.IsScalar() && box.Scalar() == "TBD")) {
    if (!box.IsMap() || !box["min"] || !box["max"]) {
      Reject(Key("workspace.catch_box") + " must be {min: [x, y, z], max: [x, y, z]}");
    }
    out.catch_box.min = Read3(box["min"], "workspace.catch_box.min");
    out.catch_box.max = Read3(box["max"], "workspace.catch_box.max");
    for (std::size_t a = 0; a < 3; ++a) {
      if (out.catch_box.min[a] > out.catch_box.max[a]) {
        Reject(Key("workspace.catch_box") + " has min > max on axis " + std::to_string(a));
      }
    }
    out.catch_box.set = true;
  }

  // ── Decel MPC (MPC E1-F03) ─────────────────────────────────────────────────
  const YAML::Node decel = Section(planner, "decel_mpc", "decel_mpc");
  DecelPlannerParams& d = out.decel;
  const YAML::Node horizon = Section(decel, "horizon", "decel_mpc.horizon");
  // From the section's kind, not the node: an absent section reads as an
  // empty but DEFINED node (catching_yaml_read.hpp).
  d.horizon_explicit = ReadSectionNode(decel, "horizon").kind == SectionKind::kMap;
  d.n_nodes =
      ReadInt(horizon, "n_nodes", "decel_mpc.horizon.n_nodes", d.n_nodes, 3, kMaxDecelNodes);
  d.dt_s = ReadBounded(horizon, "dt_s", "decel_mpc.horizon.dt_s", d.dt_s, 0.005, 0.1);
  if (std::fabs(d.dt_s * 1e9 - static_cast<double>(d.DtNs())) > 1e-3) {
    Reject(Key("decel_mpc.horizon.dt_s") +
           " must be a whole number of nanoseconds (the grid "
           "t_c + k·Δ_s is integer ns)");
  }
  if (const YAML::Node b = horizon["blocks"]; b) {
    if (!b.IsSequence() || b.size() < 3 || b.size() > static_cast<std::size_t>(kMaxDecelNodes)) {
      Reject(Key("decel_mpc.horizon.blocks") + " must be a sequence of 3.." +
             std::to_string(kMaxDecelNodes) + " positive integers, got " + Spelling(b));
    }
    d.blocks = {};
    for (std::size_t i = 0; i < b.size(); ++i) {
      int v = 0;
      try {
        v = b[i].as<int>();
      } catch (const YAML::Exception&) {
        Reject(Key("decel_mpc.horizon.blocks[" + std::to_string(i) + "]") +
               " must be an integer, got " + Spelling(b[i]));
      }
      // Bounded above too: Σ is compared with n_nodes, and unbounded entries
      // could overflow the sum back into range.
      if (v < 1 || v > kMaxDecelNodes) {
        Reject(Key("decel_mpc.horizon.blocks[" + std::to_string(i) + "]") + " must be in [1, " +
               std::to_string(kMaxDecelNodes) + "]");
      }
      d.blocks[i] = v;
    }
    d.n_blocks = static_cast<int>(b.size());
  }
  int block_sum = 0;
  for (int i = 0; i < d.n_blocks; ++i) {
    block_sum += d.blocks[static_cast<std::size_t>(i)];
  }
  if (block_sum != d.n_nodes) {
    Reject(Key("decel_mpc.horizon.blocks") + " sums to " + std::to_string(block_sum) +
           " but n_nodes is " + std::to_string(d.n_nodes) + " (Σ blocks = N)");
  }
  const YAML::Node replan = Section(decel, "replan", "decel_mpc.replan");
  d.k_max = ReadInt(replan, "k_max", "decel_mpc.replan.k_max", d.k_max, 0, kMaxDecelReplans);
  for (int k = 0; k <= d.k_max; ++k) {
    std::array<int, kMaxDecelNodes> blocks{};
    int n_blocks = 0;
    if (!DecelBlocksFor(d, k, blocks, n_blocks)) {
      Reject(Key("decel_mpc.replan.k_max") + " = " + std::to_string(d.k_max) +
             ": replan instance " + std::to_string(k) + " (N = " + std::to_string(d.n_nodes - k) +
             ") would have fewer than 3 blocks");
    }
  }
  d.eta_tau = ReadBounded(decel, "eta_tau", "decel_mpc.eta_tau", d.eta_tau, 1e-3, 1.0);
  d.m_q = ReadBounded(decel, "m_q", "decel_mpc.m_q", d.m_q, 0.0, 0.5);
  const YAML::Node publish = Section(decel, "publish", "decel_mpc.publish");
  d.slack_max =
      ReadBounded(publish, "slack_max", "decel_mpc.publish.slack_max", d.slack_max, 0.0, 1.0);
  d.slack_terminal_max =
      ReadBounded(publish, "slack_terminal_max", "decel_mpc.publish.slack_terminal_max",
                  d.slack_terminal_max, 0.0, 1.0);
  d.catch_pos_err_max =
      ReadBounded(publish, "catch_pos_err_max", "decel_mpc.publish.catch_pos_err_max",
                  d.catch_pos_err_max, 1e-6, 1.0);

  // The pre-catch part (E1-F08). Its nodes and the stop's share the
  // payload's node capacity and the core's block array.
  const YAML::Node approach = Section(decel, "approach", "decel_mpc.approach");
  const int n_pre_cap = std::min(kMaxDecelNodes - d.n_nodes, kMaxDecelNodes - d.n_blocks);
  d.n_pre_max = ReadInt(approach, "n_pre_max", "decel_mpc.approach.n_pre_max", d.n_pre_max, 0,
                        std::max(n_pre_cap, 0));
  d.dt_pre_s =
      ReadBounded(approach, "dt_pre_s", "decel_mpc.approach.dt_pre_s", d.dt_pre_s, 0.005, 0.2);
  if (std::fabs(d.dt_pre_s * 1e9 - static_cast<double>(d.DtPreNs())) > 1e-3) {
    Reject(Key("decel_mpc.approach.dt_pre_s") +
           " must be a whole number of nanoseconds (the grid t_c − k·Δ_pre is integer ns)");
  }
  d.rest_tol =
      ReadBounded(approach, "rest_tol", "decel_mpc.approach.rest_tol", d.rest_tol, 0.0, 1.0);
  const YAML::Node dbudget = Section(decel, "budget", "decel_mpc.budget");
  d.budget_first_s = ReadBounded(dbudget, "first_s", "decel_mpc.budget.first_s", d.budget_first_s,
                                 kPlannerBudgetMinS, kPlannerBudgetMaxS);
  d.budget_replan_s = ReadBounded(dbudget, "replan_s", "decel_mpc.budget.replan_s",
                                  d.budget_replan_s, kPlannerBudgetMinS, kPlannerBudgetMaxS);
  d.replan_same_point =
      ReadBool(replan, "same_point", "decel_mpc.replan.same_point", d.replan_same_point);
  const YAML::Node dcatch = Section(decel, "catch", "decel_mpc.catch");
  d.w_axis = ReadBounded(dcatch, "w_axis", "decel_mpc.catch.w_axis", d.w_axis, 0.0, 1e6);
  d.w_v_par = ReadBounded(dcatch, "w_v_par", "decel_mpc.catch.w_v_par", d.w_v_par, 0.0, 1e6);
  d.w_v_perp = ReadBounded(dcatch, "w_v_perp", "decel_mpc.catch.w_v_perp", d.w_v_perp, 0.0, 1e6);
  d.gamma_ref =
      ReadBounded(dcatch, "gamma_ref", "decel_mpc.catch.gamma_ref", d.gamma_ref, 1e-6, 1.0);
  d.kappa = ReadBounded(dcatch, "kappa", "decel_mpc.catch.kappa", d.kappa, 1e-6, 1e6);
  d.sigma_floor =
      ReadBounded(dcatch, "sigma_floor", "decel_mpc.catch.sigma_floor", d.sigma_floor, 1e-6, 1.0);
  d.w_max = ReadBounded(dcatch, "w_max", "decel_mpc.catch.w_max", d.w_max, 1e-6, 1e9);
  d.w_const = ReadBounded(dcatch, "w_const", "decel_mpc.catch.w_const", d.w_const, 1e-6, 1e9);
  d.sigma_ref =
      ReadBounded(dcatch, "sigma_ref", "decel_mpc.catch.sigma_ref", d.sigma_ref, 1e-6, 1.0);
  // The relative-velocity slack row (0 = off). The core refuses the slack
  // without a bound to be slack against (Init → kParamsInvalid); here the
  // profile is told which key to set.
  d.rho_v = ReadNonNegative(dcatch, "rho_v", "decel_mpc.catch.rho_v", d.rho_v);
  d.v_rel_allow =
      ReadNonNegative(dcatch, "v_rel_allow", "decel_mpc.catch.v_rel_allow", d.v_rel_allow);
  if (d.rho_v > 0.0 && !(d.v_rel_allow > 0.0)) {
    Reject(Key("decel_mpc.catch.rho_v") + " = " + std::to_string(d.rho_v) +
           " turns the slack on, which needs " + Key("decel_mpc.catch.v_rel_allow") + " > 0 (got " +
           std::to_string(d.v_rel_allow) + ")");
  }
  // The core's own design values (MD-92). Each range is the core's Init check
  // (decel_mpc.cpp) with the key's name on it, so a profile is told which key
  // to fix instead of reading "kParamsInvalid" at configure.
  const YAML::Node cost = Section(decel, "cost", "decel_mpc.cost");
  if (const YAML::Node jw = cost["jerk_weight"]; jw) {
    if (!jw.IsSequence() || jw.size() == 0 || jw.size() > static_cast<std::size_t>(kMaxPlanNv)) {
      Reject(Key("decel_mpc.cost.jerk_weight") +
             " must be a non-empty sequence of one positive number per arm joint, got " +
             Spelling(jw));
    }
    d.jerk_weight.clear();
    for (std::size_t i = 0; i < jw.size(); ++i) {
      const std::string path = "decel_mpc.cost.jerk_weight[" + std::to_string(i) + "]";
      double w = 0.0;
      try {
        w = jw[i].as<double>();
      } catch (const YAML::Exception&) {
        Reject(Key(path) + " must be a number, got " + Spelling(jw[i]));
      }
      if (!std::isfinite(w) || !(w > 0.0)) {
        Reject(Key(path) + " = " + Spelling(jw[i]) + " must be a finite number > 0");
      }
      d.jerk_weight.push_back(w);
    }
  }
  d.u_scale = ReadPositive(cost, "u_scale", "decel_mpc.cost.u_scale", d.u_scale);
  d.w_delta = ReadNonNegative(cost, "w_delta", "decel_mpc.cost.w_delta", d.w_delta);
  d.rho_tau = ReadNonNegative(cost, "rho_tau", "decel_mpc.cost.rho_tau", d.rho_tau);
  // The stop-path weight (0 = off). Bounded from below only, as the core's
  // own check; the line it penalises is built by the planner per solve.
  d.w_perp = ReadNonNegative(cost, "w_perp", "decel_mpc.cost.w_perp", d.w_perp);
  d.axis_theta_max =
      ReadInterval(dcatch, "axis_theta_max", "decel_mpc.catch.axis_theta_max", d.axis_theta_max,
                   0.0, true, 3.14159265358979323846, true, "(0, pi)");
  const YAML::Node lin = Section(decel, "linearization", "decel_mpc.linearization");
  d.delta_tr = ReadPositive(lin, "delta_tr", "decel_mpc.linearization.delta_tr", d.delta_tr);
  // Upper bound: DecelPlanner::Judge takes a solve as published only when node N
  // rests to reference_rest_tol, and the RT admits a payload only when node N
  // rests to kDecelRestTol (ValidateDecelNodes). A looser tolerance would let
  // Judge accept nodes the validator then refuses.
  d.reference_rest_tol =
      ReadInterval(lin, "reference_rest_tol", "decel_mpc.linearization.reference_rest_tol",
                   d.reference_rest_tol, 0.0, true, kDecelRestTol, false,
                   "(0, 1e-3] (kDecelRestTol, the bound the RT admits a published node N by)");
  d.ref_speed_fraction =
      ReadInterval(lin, "ref_speed_fraction", "decel_mpc.linearization.ref_speed_fraction",
                   d.ref_speed_fraction, 0.0, true, 1.0, false, "(0, 1]");
  const YAML::Node solver = Section(decel, "solver", "decel_mpc.solver");
  d.solver_max_iter = ReadInt(solver, "max_iter", "decel_mpc.solver.max_iter", d.solver_max_iter, 1,
                              std::numeric_limits<int>::max());
  d.solver_max_iter_in = ReadInt(solver, "max_iter_in", "decel_mpc.solver.max_iter_in",
                                 d.solver_max_iter_in, 1, std::numeric_limits<int>::max());
  d.solver_eps_abs = ReadPositive(solver, "eps_abs", "decel_mpc.solver.eps_abs", d.solver_eps_abs);
  d.solver_eps_rel =
      ReadNonNegative(solver, "eps_rel", "decel_mpc.solver.eps_rel", d.solver_eps_rel);
  // A shifted previous solution meets the terminal equality only to eps_abs
  // (the core's check, decel_mpc.cpp Init): a rest tolerance at or below it
  // would refuse every warm solve.
  if (!(d.reference_rest_tol > d.solver_eps_abs)) {
    Reject(Key("decel_mpc.linearization.reference_rest_tol") + " = " +
           Shown(lin, "reference_rest_tol", d.reference_rest_tol) + " must exceed " +
           Key("decel_mpc.solver.eps_abs") + " (= " + Shown(solver, "eps_abs", d.solver_eps_abs) +
           ")");
  }
  return out;
}

bool DecelBlocksFor(const DecelPlannerParams& p, int k, std::array<int, kMaxDecelNodes>& blocks,
                    int& n_blocks) noexcept {
  if (k < 0 || k >= p.n_nodes || p.n_blocks < 1 || p.n_blocks > kMaxDecelNodes) {
    return false;
  }
  std::array<int, kMaxDecelNodes> b = p.blocks;
  int n = p.n_blocks;
  for (int step = 0; step < k; ++step) {
    int largest = 0;
    for (int i = 1; i < n; ++i) {
      if (b[static_cast<std::size_t>(i)] >= b[static_cast<std::size_t>(largest)]) {
        largest = i;  // `>=`: the LAST of equal blocks
      }
    }
    if (--b[static_cast<std::size_t>(largest)] == 0) {
      for (int i = largest; i + 1 < n; ++i) {
        b[static_cast<std::size_t>(i)] = b[static_cast<std::size_t>(i + 1)];
      }
      b[static_cast<std::size_t>(--n)] = 0;
    }
  }
  if (n < 3) {
    return false;
  }
  blocks = b;
  n_blocks = n;
  return true;
}

}  // namespace rtc::catching
