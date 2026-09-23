// Parser for `catching.planner.*` (S6). See planner_params.hpp.
#include "rtc_controllers/catching/planner_params.hpp"

#include "catching_yaml_read.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
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
  return out;
}

}  // namespace rtc::catching
