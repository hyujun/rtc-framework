// Parser for `catching.planner.*`. See planner_params.hpp.
#include "rtc_controllers/catching/planner_params.hpp"

#include "catching_yaml_read.hpp"

#include <algorithm>
#include <array>
#include <cctype>
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

/// A group name: it becomes part of parameter names, so [A-Za-z][A-Za-z0-9_]*.
[[nodiscard]] bool IsGroupToken(const std::string& s) {
  if (s.empty() || std::isalpha(static_cast<unsigned char>(s[0])) == 0) {
    return false;
  }
  return std::all_of(s.begin(), s.end(), [](char c) {
    return std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_';
  });
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
/// accepted: these keys have documented defaults, so a TBD here is a
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
           std::to_string(hi) + "]");
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

}  // namespace

PlannerParams ParsePlannerParams(const YAML::Node& catching, const PlannerKeySelection& selection) {
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
  // Each function's design values sit under its own map (#711): the grid
  // search's under `planner.search.grid`, the mpc segment planner's under
  // `planner.segment.mpc`. What is read from `planner` itself is the thread's.
  //
  // ONLY A SELECTED FUNCTION'S MAP IS OPENED (PlannerKeySelection). One that is
  // not selected is taken as absent, which is how every reader below already
  // treats a missing section: defaults, and the unset decisions unset.
  const YAML::Node search = Section(planner, "search", "search");
  const YAML::Node grid =
      selection.search_grid ? Section(search, "grid", "search.grid") : YAML::Node();

  // ── Thread ─────────────────────────────────────────────────────────────────
  out.enabled = ReadBool(planner, "enabled", "enabled", out.enabled);
  out.provisional = ReadBool(planner, "provisional", "provisional", out.provisional);
  out.wake_timeout_s = ReadBounded(planner, "wake_timeout_s", "wake_timeout_s", out.wake_timeout_s,
                                   kPlannerWakeTimeoutMinS, kPlannerWakeTimeoutMaxS);
  out.budget_s = ReadBounded(grid, "budget_s", "search.grid.budget_s", out.budget_s,
                             kPlannerBudgetMinS, kPlannerBudgetMaxS);
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

  // Joint groups: names and device joint names only — the binding
  // resolves them against the device (ResolveJointGroups), and each segment
  // block reads its per-group values by these names.
  if (const YAML::Node groups = planner["groups"]; groups) {
    if (!groups.IsSequence() || groups.size() == 0) {
      Reject(Key("groups") + " must be a non-empty sequence of {name, joints}, got " +
             Spelling(groups));
    }
    if (groups.size() > static_cast<std::size_t>(kMaxJointGroups)) {
      Reject(Key("groups") + " lists " + std::to_string(groups.size()) + " groups, at most " +
             std::to_string(kMaxJointGroups) + " (kMaxJointGroups)");
    }
    std::vector<std::string> all_joints;
    for (std::size_t i = 0; i < groups.size(); ++i) {
      const std::string where = "groups[" + std::to_string(i) + "]";
      const YAML::Node g = groups[i];
      if (!g.IsMap()) {
        Reject(Key(where) + " must be a map {name, joints}, got " + Spelling(g));
      }
      for (const auto& kv : g) {
        const std::string k = kv.first.as<std::string>();
        if (k != "name" && k != "joints") {
          Reject(Key(where + "." + k) + " is not a key of a group (name, joints)");
        }
      }
      JointGroupSpec spec;
      const YAML::Node name = g["name"];
      if (!name || !name.IsScalar() || !IsGroupToken(name.Scalar())) {
        Reject(Key(where + ".name") +
               " must be a [A-Za-z][A-Za-z0-9_]* token (it names parameters), got " +
               (name ? Spelling(name) : std::string("nothing")));
      }
      spec.name = name.Scalar();
      // `planner.groups.names` is the mirror's list of the names themselves.
      if (spec.name == "names") {
        Reject(Key(where + ".name") + " 'names' is reserved (planner.groups.names lists them)");
      }
      if (FindJointGroup(out.groups, spec.name) >= 0) {
        Reject(Key(where + ".name") + " '" + spec.name + "' names a group already declared");
      }
      const YAML::Node joints = g["joints"];
      if (!joints || !joints.IsSequence() || joints.size() == 0) {
        Reject(Key(where + ".joints") + " must be a non-empty sequence of joint names, got " +
               (joints ? Spelling(joints) : std::string("nothing")));
      }
      for (std::size_t j = 0; j < joints.size(); ++j) {
        if (!joints[j].IsScalar() || joints[j].Scalar().empty()) {
          Reject(Key(where + ".joints[" + std::to_string(j) + "]") + " must be a joint name, got " +
                 Spelling(joints[j]));
        }
        const std::string joint = joints[j].Scalar();
        if (std::find(all_joints.begin(), all_joints.end(), joint) != all_joints.end()) {
          Reject(Key(where + ".joints") + " lists '" + joint +
                 "', which is already in a group (the groups are disjoint)");
        }
        all_joints.push_back(joint);
        spec.joints.push_back(joint);
      }
      out.groups.push_back(std::move(spec));
    }
  }

  // ── Search ─────────────────────────────────────────────────────────────────
  if (const YAML::Node v = planner["sub_model"]; v) {
    if (!v.IsScalar() || v.Scalar().empty() || v.Scalar() == "TBD") {
      Reject(Key("sub_model") + " must name a urdf.sub_models entry, got " + Spelling(v));
    }
    out.sub_model = v.Scalar();
  }
  // The model the search runs on, when it is not the one above.
  if (search.IsMap()) {
    if (const YAML::Node v = search["sub_model"]; v) {
      if (!v.IsScalar() || v.Scalar().empty() || v.Scalar() == "TBD") {
        Reject(Key("search.sub_model") + " must name a urdf.sub_models entry, got " + Spelling(v));
      }
      out.search_sub_model = v.Scalar();
    }
  }
  out.max_ik = ReadInt(grid, "max_ik", "search.grid.max_ik", out.max_ik, 1, kPlannerMaxIkCapacity);
  out.n_settle = ReadInt(grid, "n_settle", "search.grid.n_settle", out.n_settle, 0, 20);

  const YAML::Node slice = Section(grid, "slice", "search.grid.slice");
  out.slice_dt = ReadBounded(slice, "dt", "search.grid.slice.dt", out.slice_dt, 0.005, 0.05);
  out.slice_t_max =
      ReadBounded(slice, "t_max", "search.grid.slice.t_max", out.slice_t_max, 0.2, 1.5);
  if (slice["t_lead_min"]) {
    out.slice_t_lead_min =
        ReadBounded(slice, "t_lead_min", "search.grid.slice.t_lead_min", 0.0, 1e-3, 1.5);
  }

  const YAML::Node time = Section(grid, "time", "search.grid.time");
  out.time_margin =
      ReadBounded(time, "margin", "search.grid.time.margin", out.time_margin, 0.0, 0.2);

  const YAML::Node unc = Section(grid, "unc", "search.grid.unc");
  out.kappa_sigma =
      ReadBounded(unc, "kappa_sigma", "search.grid.unc.kappa_sigma", out.kappa_sigma, 0.05, 1.0);

  const YAML::Node gamma = Section(grid, "gamma", "search.grid.gamma");
  out.gamma_margin =
      ReadBounded(gamma, "margin", "search.grid.gamma.margin", out.gamma_margin, 0.0, 1.0);
  out.unit_speed_damping =
      ReadInterval(gamma, "unit_speed_damping", "search.grid.gamma.unit_speed_damping",
                   out.unit_speed_damping, 0.0, true, 1.0, false, "(0, 1]");
  out.eta_a = ReadBounded(gamma, "eta_a", "search.grid.gamma.eta_a", out.eta_a, 1e-3, 1.0);
  out.eps_term =
      ReadBounded(gamma, "eps_term", "search.grid.gamma.eps_term", out.eps_term, 1e-6, 1.0);
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
  read_grid(gamma, "grid", "search.grid.gamma.grid", out.gamma_grid, out.gamma_grid_n, 0.0, 1.0);
  read_grid(gamma, "window_grid", "search.grid.gamma.window_grid", out.window_grid,
            out.window_grid_n, 1e-3, 2.0);
  const YAML::Node rollout = Section(grid, "rollout", "search.grid.rollout");
  out.rollout_dt_coarse = ReadBounded(rollout, "dt_coarse", "search.grid.rollout.dt_coarse",
                                      out.rollout_dt_coarse, 1e-4, 0.05);

  const YAML::Node budget = Section(grid, "budget", "search.grid.budget");
  out.n_sigma = ReadBounded(budget, "n_sigma", "search.grid.budget.n_sigma", out.n_sigma, 1.0, 3.0);
  out.sigma_trk =
      ReadBounded(budget, "sigma_trk", "search.grid.budget.sigma_trk", out.sigma_trk, 0.0, 1.0);
  out.clock_err =
      ReadBounded(budget, "clock_err", "search.grid.budget.clock_err", out.clock_err, 0.0, 1.0);

  const YAML::Node hand = Section(grid, "hand", "search.grid.hand");
  out.d_eff = ReadDecision(hand, "d_eff", "search.grid.hand.d_eff", 1e-4, 10.0);
  out.r_cap = ReadDecision(hand, "r_cap", "search.grid.hand.r_cap", 1e-4, 1.0);

  const YAML::Node sw = Section(grid, "switch", "search.grid.switch");
  out.switch_delta_j =
      ReadBounded(sw, "delta_J", "search.grid.switch.delta_J", out.switch_delta_j, 0.0, 1e6);
  // The distance limits the acceleration budget replaced (decision ⑥): a
  // profile still carrying them was tuned for the old rule, so it is refused
  // rather than silently run on the eta_jump default.
  for (const char* retired : {"e_jump_max", "ed_jump_max"}) {
    if (sw && sw.IsMap() && sw[retired]) {
      Reject(Key(std::string("search.grid.switch.") + retired) +
             " was replaced by planner.search.grid.switch.eta_jump (an acceleration budget)");
    }
  }
  out.switch_eta_jump =
      ReadBounded(sw, "eta_jump", "search.grid.switch.eta_jump", out.switch_eta_jump, 1e-6, 1.0);
  // The step bound divides by samples − 1: fewer than two instants is no ramp.
  // And every sample is a ramp evaluation on the planner thread per switch
  // check, so the count is capped (kSwitchSamplesMax).
  out.switch_samples = ReadInt(sw, "samples", "search.grid.switch.samples", out.switch_samples, 2,
                               kSwitchSamplesMax);

  const YAML::Node freeze = Section(planner, "freeze", "freeze");
  out.t_freeze = ReadDecision(freeze, "T_freeze", "freeze.T_freeze", 1e-3, 2.0);
  // Absent → T_freeze: the search stops where the plan freezes. That it is not
  // BELOW T_freeze is the binding's check (it parks, naming both values) — a
  // malformed number is refused here like any other.
  out.t_stop_plan = freeze["t_stop_plan"]
                        ? ReadBounded(freeze, "t_stop_plan", "freeze.t_stop_plan", 0.0, 1e-3, 2.0)
                        : out.t_freeze;

  const YAML::Node score = Section(grid, "score", "search.grid.score");
  out.score.w_sigma =
      ReadBounded(score, "w_sigma", "search.grid.score.w_sigma", out.score.w_sigma, 0.0, 1e6);
  out.score.w_t = ReadBounded(score, "w_t", "search.grid.score.w_t", out.score.w_t, 0.0, 1e6);
  out.score.w_q = ReadBounded(score, "w_q", "search.grid.score.w_q", out.score.w_q, 0.0, 1e6);
  out.score.w_late =
      ReadBounded(score, "w_late", "search.grid.score.w_late", out.score.w_late, 0.0, 1e6);
  out.score.w_gamma =
      ReadBounded(score, "w_gamma", "search.grid.score.w_gamma", out.score.w_gamma, 0.0, 1e6);
  out.score.penalty =
      ReadBounded(score, "penalty", "search.grid.score.penalty", out.score.penalty, 0.0, 1e9);

  // `search.grid.workspace` (the removed catch box) is not read: the
  // search no longer judges where the catch point is. A tree that
  // still writes it is the binding's to park on (kRemovedCatchingKeys).

  // ── Segment MPC ────────────────────────────────────────────────────────────
  // `planner.segment.mode` and, under `mpc`, `switch_margin` and `eta_v` are
  // ParseCatchingParams' (the validator judges them); the rest is read here.
  const YAML::Node segment = Section(planner, "segment", "segment");
  const YAML::Node mpc =
      selection.segment_mpc ? Section(segment, "mpc", "segment.mpc") : YAML::Node();
  MpcSegmentPlannerParams& d = out.mpc_segment;
  const YAML::Node horizon = Section(mpc, "horizon", "segment.mpc.horizon");
  // From the section's kind, not the node: an absent section reads as an
  // empty but DEFINED node (catching_yaml_read.hpp).
  d.horizon_explicit = ReadSectionNode(mpc, "horizon").kind == SectionKind::kMap;
  d.n_nodes =
      ReadInt(horizon, "n_nodes", "segment.mpc.horizon.n_nodes", d.n_nodes, 3, kMaxSegmentNodes);
  d.dt_s = ReadBounded(horizon, "dt_s", "segment.mpc.horizon.dt_s", d.dt_s, 0.005, 0.1);
  if (std::fabs(d.dt_s * 1e9 - static_cast<double>(d.DtNs())) > 1e-3) {
    Reject(Key("segment.mpc.horizon.dt_s") +
           " must be a whole number of nanoseconds (the grid "
           "t_c + k·Δ_s is integer ns)");
  }
  if (const YAML::Node b = horizon["blocks"]; b) {
    if (!b.IsSequence() || b.size() < 3 || b.size() > static_cast<std::size_t>(kMaxSegmentNodes)) {
      Reject(Key("segment.mpc.horizon.blocks") + " must be a sequence of 3.." +
             std::to_string(kMaxSegmentNodes) + " positive integers, got " + Spelling(b));
    }
    d.blocks = {};
    for (std::size_t i = 0; i < b.size(); ++i) {
      int v = 0;
      try {
        v = b[i].as<int>();
      } catch (const YAML::Exception&) {
        Reject(Key("segment.mpc.horizon.blocks[" + std::to_string(i) + "]") +
               " must be an integer, got " + Spelling(b[i]));
      }
      // Bounded above too: Σ is compared with n_nodes, and unbounded entries
      // could overflow the sum back into range.
      if (v < 1 || v > kMaxSegmentNodes) {
        Reject(Key("segment.mpc.horizon.blocks[" + std::to_string(i) + "]") + " must be in [1, " +
               std::to_string(kMaxSegmentNodes) + "]");
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
    Reject(Key("segment.mpc.horizon.blocks") + " sums to " + std::to_string(block_sum) +
           " but n_nodes is " + std::to_string(d.n_nodes) + " (Σ blocks = N)");
  }
  const YAML::Node replan = Section(mpc, "replan", "segment.mpc.replan");
  d.k_max = ReadInt(replan, "k_max", "segment.mpc.replan.k_max", d.k_max, 0, kMaxMpcSegmentReplans);
  for (int k = 0; k <= d.k_max; ++k) {
    std::array<int, kMaxSegmentNodes> blocks{};
    int n_blocks = 0;
    if (!MpcSegmentBlocksFor(d, k, blocks, n_blocks)) {
      Reject(Key("segment.mpc.replan.k_max") + " = " + std::to_string(d.k_max) +
             ": replan instance " + std::to_string(k) + " (N = " + std::to_string(d.n_nodes - k) +
             ") would have fewer than 3 blocks");
    }
  }
  // Per joint group: `groups.<name>`, a `planner.groups` name.
  d.n_groups = static_cast<int>(out.groups.size());
  if (const YAML::Node gmap = mpc["groups"]; gmap) {
    if (!gmap.IsMap()) {
      Reject(Key("segment.mpc.groups") + " must be a map of planner.groups names, got " +
             Spelling(gmap));
    }
    for (const auto& kv : gmap) {
      const std::string name = kv.first.as<std::string>();
      const std::string path = "segment.mpc.groups." + name;
      const int g = FindJointGroup(out.groups, name);
      if (g < 0) {
        Reject(Key(path) + " names no group of 'planner.groups'");
      }
      const YAML::Node entry = kv.second;
      if (!entry.IsMap()) {
        Reject(Key(path) + " must be a map, got " + Spelling(entry));
      }
      for (const auto& e : entry) {
        const std::string k = e.first.as<std::string>();
        if (k != "blocks" && k != "w_vel" && k != "w_rest") {
          Reject(Key(path + "." + k) +
                 " is not a key of a group's segment.mpc entry (blocks, w_vel, w_rest)");
        }
      }
      MpcSegmentGroupParams& gp = d.groups[static_cast<std::size_t>(g)];
      gp.w_vel = ReadInterval(entry, "w_vel", path + ".w_vel", gp.w_vel, 0.0, false,
                              kWholeBodyWeightMax, false,
                              "[0, 1e4] (kWholeBodyWeightMax, the largest weight the "
                              "core is tested with)");
      gp.w_rest = ReadInterval(entry, "w_rest", path + ".w_rest", gp.w_rest, 0.0, false,
                               kWholeBodyWeightMax, false,
                               "[0, 1e4] (kWholeBodyWeightMax, the largest weight the "
                               "core is tested with)");
      if (gp.w_rest > 0.0 && out.wait_pose_n == 0) {
        Reject(Key(path + ".w_rest") + " > 0 holds the group near its rest posture, " +
               Key("wait_pose") + ", which is not given");
      }
      const YAML::Node blocks = Section(entry, "blocks", path + ".blocks");
      for (const auto& e : blocks) {
        const std::string k = e.first.as<std::string>();
        if (k != "pre" && k != "stop") {
          Reject(Key(path + ".blocks." + k) + " is not a key of a group's blocks (pre, stop)");
        }
      }
      // One group is the whole arm: the cores then plan without groups (one
      // block per pre-catch node, horizon.blocks), so its own would be ignored.
      if (out.groups.size() == 1 && blocks.IsMap() && blocks.size() != 0) {
        Reject(Key(path + ".blocks") +
               ": with one group the planner moves blocks as without groups (one per "
               "pre-catch node, " +
               Key("segment.mpc.horizon.blocks") +
               ") — declare a second group to give a group its own");
      }
      gp.pre_block =
          ReadInt(blocks, "pre", path + ".blocks.pre", gp.pre_block, 1, kMaxSegmentNodes);
      if (const YAML::Node st = blocks["stop"]; st) {
        const std::string sp = path + ".blocks.stop";
        if (!st.IsSequence() || st.size() < 3 ||
            st.size() > static_cast<std::size_t>(kMaxSegmentNodes)) {
          Reject(Key(sp) + " must be a sequence of 3.." + std::to_string(kMaxSegmentNodes) +
                 " positive integers, got " + Spelling(st));
        }
        int sum = 0;
        for (std::size_t i = 0; i < st.size(); ++i) {
          int v = 0;
          try {
            v = st[i].as<int>();
          } catch (const YAML::Exception&) {
            Reject(Key(sp + "[" + std::to_string(i) + "]") + " must be an integer, got " +
                   Spelling(st[i]));
          }
          if (v < 1 || v > kMaxSegmentNodes) {
            Reject(Key(sp + "[" + std::to_string(i) + "]") + " must be in [1, " +
                   std::to_string(kMaxSegmentNodes) + "]");
          }
          gp.stop_blocks[i] = v;
          sum += v;
        }
        gp.n_stop_blocks = static_cast<int>(st.size());
        if (sum != d.n_nodes) {
          Reject(Key(sp) + " sums to " + std::to_string(sum) + " but horizon.n_nodes is " +
                 std::to_string(d.n_nodes) + " (Σ blocks = N)");
        }
        // Every replan instance k ≤ k_max keeps three blocks: a partition loses
        // a block only once every block is 1, at N − k < 3, which k_max's own
        // check above already excludes.
      }
    }
  }
  // The ball speed below which its direction of travel is undefined — the mpc
  // segment planner's own floor, beside the search IK's `search.grid.ik.v_eps`.
  d.v_eps = ReadPositive(mpc, "v_eps", "segment.mpc.v_eps", d.v_eps);
  d.eta_tau = ReadBounded(mpc, "eta_tau", "segment.mpc.eta_tau", d.eta_tau, 1e-3, 1.0);
  d.m_q = ReadBounded(mpc, "m_q", "segment.mpc.m_q", d.m_q, 0.0, 0.5);
  const YAML::Node publish = Section(mpc, "publish", "segment.mpc.publish");
  d.slack_max =
      ReadBounded(publish, "slack_max", "segment.mpc.publish.slack_max", d.slack_max, 0.0, 1.0);
  d.slack_terminal_max =
      ReadBounded(publish, "slack_terminal_max", "segment.mpc.publish.slack_terminal_max",
                  d.slack_terminal_max, 0.0, 1.0);
  d.catch_pos_err_max =
      ReadBounded(publish, "catch_pos_err_max", "segment.mpc.publish.catch_pos_err_max",
                  d.catch_pos_err_max, 1e-6, 1.0);

  // The pre-catch part. Its nodes and the stop's share the
  // payload's node capacity and the core's block array.
  const YAML::Node approach = Section(mpc, "approach", "segment.mpc.approach");
  const int n_pre_cap = std::min(kMaxSegmentNodes - d.n_nodes, kMaxSegmentNodes - d.n_blocks);
  d.n_pre_max = ReadInt(approach, "n_pre_max", "segment.mpc.approach.n_pre_max", d.n_pre_max, 0,
                        std::max(n_pre_cap, 0));
  d.dt_pre_s =
      ReadBounded(approach, "dt_pre_s", "segment.mpc.approach.dt_pre_s", d.dt_pre_s, 0.005, 0.2);
  if (std::fabs(d.dt_pre_s * 1e9 - static_cast<double>(d.DtPreNs())) > 1e-3) {
    Reject(Key("segment.mpc.approach.dt_pre_s") +
           " must be a whole number of nanoseconds (the grid t_c − k·Δ_pre is integer ns)");
  }
  d.rest_tol =
      ReadBounded(approach, "rest_tol", "segment.mpc.approach.rest_tol", d.rest_tol, 0.0, 1.0);
  const YAML::Node dbudget = Section(mpc, "budget", "segment.mpc.budget");
  d.budget_first_s = ReadBounded(dbudget, "first_s", "segment.mpc.budget.first_s", d.budget_first_s,
                                 kPlannerBudgetMinS, kPlannerBudgetMaxS);
  d.budget_replan_s = ReadBounded(dbudget, "replan_s", "segment.mpc.budget.replan_s",
                                  d.budget_replan_s, kPlannerBudgetMinS, kPlannerBudgetMaxS);
  d.replan_same_point =
      ReadBool(replan, "same_point", "segment.mpc.replan.same_point", d.replan_same_point);
  const YAML::Node dcatch = Section(mpc, "catch", "segment.mpc.catch");
  d.w_axis = ReadBounded(dcatch, "w_axis", "segment.mpc.catch.w_axis", d.w_axis, 0.0, 1e6);
  d.w_v_par = ReadBounded(dcatch, "w_v_par", "segment.mpc.catch.w_v_par", d.w_v_par, 0.0, 1e6);
  d.w_v_perp = ReadBounded(dcatch, "w_v_perp", "segment.mpc.catch.w_v_perp", d.w_v_perp, 0.0, 1e6);
  d.gamma_ref =
      ReadBounded(dcatch, "gamma_ref", "segment.mpc.catch.gamma_ref", d.gamma_ref, 1e-6, 1.0);
  d.kappa = ReadBounded(dcatch, "kappa", "segment.mpc.catch.kappa", d.kappa, 1e-6, 1e6);
  d.sigma_floor =
      ReadBounded(dcatch, "sigma_floor", "segment.mpc.catch.sigma_floor", d.sigma_floor, 1e-6, 1.0);
  d.w_max = ReadBounded(dcatch, "w_max", "segment.mpc.catch.w_max", d.w_max, 1e-6, 1e9);
  d.w_const = ReadBounded(dcatch, "w_const", "segment.mpc.catch.w_const", d.w_const, 1e-6, 1e9);
  d.sigma_ref =
      ReadBounded(dcatch, "sigma_ref", "segment.mpc.catch.sigma_ref", d.sigma_ref, 1e-6, 1.0);
  // The relative-velocity slack row (0 = off). The core refuses the slack
  // without a bound to be slack against (Init → kParamsInvalid); here the
  // profile is told which key to set.
  d.rho_v = ReadNonNegative(dcatch, "rho_v", "segment.mpc.catch.rho_v", d.rho_v);
  d.v_rel_allow =
      ReadNonNegative(dcatch, "v_rel_allow", "segment.mpc.catch.v_rel_allow", d.v_rel_allow);
  if (d.rho_v > 0.0 && !(d.v_rel_allow > 0.0)) {
    Reject(Key("segment.mpc.catch.rho_v") + " = " + std::to_string(d.rho_v) +
           " turns the slack on, which needs " + Key("segment.mpc.catch.v_rel_allow") +
           " > 0 (got " + std::to_string(d.v_rel_allow) + ")");
  }
  // The core's own design values (YAML keys). Each range is the core's Init check
  // (mpc_segment_core.cpp) with the key's name on it, so a profile is told which key
  // to fix instead of reading "kParamsInvalid" at configure.
  const YAML::Node cost = Section(mpc, "cost", "segment.mpc.cost");
  if (const YAML::Node jw = cost["jerk_weight"]; jw) {
    if (!jw.IsSequence() || jw.size() == 0 || jw.size() > static_cast<std::size_t>(kMaxPlanNv)) {
      Reject(Key("segment.mpc.cost.jerk_weight") +
             " must be a non-empty sequence of one positive number per arm joint, got " +
             Spelling(jw));
    }
    d.jerk_weight.clear();
    for (std::size_t i = 0; i < jw.size(); ++i) {
      const std::string path = "segment.mpc.cost.jerk_weight[" + std::to_string(i) + "]";
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
  d.u_scale = ReadPositive(cost, "u_scale", "segment.mpc.cost.u_scale", d.u_scale);
  d.w_delta = ReadNonNegative(cost, "w_delta", "segment.mpc.cost.w_delta", d.w_delta);
  d.rho_tau = ReadNonNegative(cost, "rho_tau", "segment.mpc.cost.rho_tau", d.rho_tau);
  // The stop-path weight (0 = off). The core takes any value >= 0; the upper
  // bound is the largest one its solves are tested with — nothing downstream
  // (not the configure warm-up either) would refuse a larger one.
  d.w_perp =
      ReadInterval(cost, "w_perp", "segment.mpc.cost.w_perp", d.w_perp, 0.0, false,
                   kMpcSegmentStopPathWeightMax, false,
                   "[0, 3e3] (kMpcSegmentStopPathWeightMax, the largest weight the cores are "
                   "tested with)");
  // The closing-window path weight (#716; 0 = off): the same form as w_perp
  // on other nodes, so the same tested bound.
  d.w_path =
      ReadInterval(cost, "w_path", "segment.mpc.cost.w_path", d.w_path, 0.0, false,
                   kMpcSegmentStopPathWeightMax, false,
                   "[0, 3e3] (kMpcSegmentStopPathWeightMax, the largest weight the cores are "
                   "tested with)");
  const YAML::Node ang = Section(cost, "angular_momentum", "segment.mpc.cost.angular_momentum");
  for (const auto& e : ang) {
    const std::string k = e.first.as<std::string>();
    if (k != "w_pre" && k != "w_post" && k != "scale") {
      Reject(Key("segment.mpc.cost.angular_momentum." + k) +
             " is not a key of the angular-momentum term (w_pre, w_post, scale)");
    }
  }
  d.ang_w_pre = ReadInterval(ang, "w_pre", "segment.mpc.cost.angular_momentum.w_pre", d.ang_w_pre,
                             0.0, false, kWholeBodyWeightMax, false,
                             "[0, 1e4] (kWholeBodyWeightMax, the largest weight the core is "
                             "tested with)");
  d.ang_w_post =
      ReadInterval(ang, "w_post", "segment.mpc.cost.angular_momentum.w_post", d.ang_w_post, 0.0,
                   false, kWholeBodyWeightMax, false,
                   "[0, 1e4] (kWholeBodyWeightMax, the largest weight the core is tested with)");
  // NUM-2: the weight is divided by s², so s is a positive finite number.
  d.ang_scale = ReadPositive(ang, "scale", "segment.mpc.cost.angular_momentum.scale", d.ang_scale);
  d.axis_theta_max =
      ReadInterval(dcatch, "axis_theta_max", "segment.mpc.catch.axis_theta_max", d.axis_theta_max,
                   0.0, true, 3.14159265358979323846, true, "(0, pi)");
  const YAML::Node lin = Section(mpc, "linearization", "segment.mpc.linearization");
  d.delta_tr = ReadPositive(lin, "delta_tr", "segment.mpc.linearization.delta_tr", d.delta_tr);
  // Upper bound: MpcSegmentPlanner::Judge takes a solve as published only when node N
  // rests to reference_rest_tol, and the RT admits a payload only when node N
  // rests to kSegmentRestTol (ValidateSegmentNodes). A looser tolerance would let
  // Judge accept nodes the validator then refuses.
  d.reference_rest_tol =
      ReadInterval(lin, "reference_rest_tol", "segment.mpc.linearization.reference_rest_tol",
                   d.reference_rest_tol, 0.0, true, kSegmentRestTol, false,
                   "(0, 1e-3] (kSegmentRestTol, the bound the RT admits a published node N by)");
  // Upper bound: the minimum-jerk peak factor — at it the reference is cut only
  // where the velocity box cannot reach the catch pose in the span at all
  // (T_v > t_span); above it the key would stop meaning anything.
  d.ref_speed_fraction =
      ReadInterval(lin, "ref_speed_fraction", "segment.mpc.linearization.ref_speed_fraction",
                   d.ref_speed_fraction, 0.0, true, kMinJerkPeakFactor, false,
                   "(0, 1.875] (kMinJerkPeakFactor, the minimum-jerk peak over the box)");
  const YAML::Node solver = Section(mpc, "solver", "segment.mpc.solver");
  d.solver_max_iter = ReadInt(solver, "max_iter", "segment.mpc.solver.max_iter", d.solver_max_iter,
                              1, std::numeric_limits<int>::max());
  d.solver_max_iter_in = ReadInt(solver, "max_iter_in", "segment.mpc.solver.max_iter_in",
                                 d.solver_max_iter_in, 1, std::numeric_limits<int>::max());
  d.solver_max_iter_warm = ReadInt(solver, "max_iter_warm", "segment.mpc.solver.max_iter_warm",
                                   d.solver_max_iter_warm, 0, std::numeric_limits<int>::max());
  d.solver_eps_abs =
      ReadPositive(solver, "eps_abs", "segment.mpc.solver.eps_abs", d.solver_eps_abs);
  d.solver_eps_rel =
      ReadNonNegative(solver, "eps_rel", "segment.mpc.solver.eps_rel", d.solver_eps_rel);
  d.solver_eps_primal_inf = ReadNonNegative(
      solver, "eps_primal_inf", "segment.mpc.solver.eps_primal_inf", d.solver_eps_primal_inf);
  // A shifted previous solution meets the terminal equality only to eps_abs
  // (the core's check, mpc_segment_core.cpp Init): a rest tolerance at or below it
  // would refuse every warm solve.
  if (!(d.reference_rest_tol > d.solver_eps_abs)) {
    Reject(Key("segment.mpc.linearization.reference_rest_tol") + " = " +
           Shown(lin, "reference_rest_tol", d.reference_rest_tol) + " must exceed " +
           Key("segment.mpc.solver.eps_abs") + " (= " + Shown(solver, "eps_abs", d.solver_eps_abs) +
           ")");
  }
  return out;
}

bool MpcSegmentBlocksFor(const MpcSegmentPlannerParams& p, int k,
                         std::array<int, kMaxSegmentNodes>& blocks, int& n_blocks) noexcept {
  return MpcSegmentBlocksFor(p.blocks, p.n_blocks, p.n_nodes, k, blocks, n_blocks);
}

void MpcSegmentGroupStopBase(const MpcSegmentPlannerParams& p, int g,
                             std::array<int, kMaxSegmentNodes>& base, int& n_base) noexcept {
  const bool own =
      g >= 0 && g < kMaxJointGroups && p.groups[static_cast<std::size_t>(g)].n_stop_blocks > 0;
  base = own ? p.groups[static_cast<std::size_t>(g)].stop_blocks : p.blocks;
  n_base = own ? p.groups[static_cast<std::size_t>(g)].n_stop_blocks : p.n_blocks;
}

bool MpcSegmentBlocksFor(const std::array<int, kMaxSegmentNodes>& base, int n_base, int n_nodes,
                         int k, std::array<int, kMaxSegmentNodes>& blocks, int& n_blocks) noexcept {
  if (k < 0 || k >= n_nodes || n_base < 1 || n_base > kMaxSegmentNodes) {
    return false;
  }
  std::array<int, kMaxSegmentNodes> b = base;
  int n = n_base;
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

std::string MpcSegmentLeftFreeRefusal(const PlannerParams& planner, const JointGroups& groups,
                                      int n_dev, JointSet plan_joints) {
  const MpcSegmentPlannerParams& mpc = planner.mpc_segment;
  if (!(mpc.ang_w_pre > 0.0 || mpc.ang_w_post > 0.0)) {
    return {};
  }
  std::array<double, kMaxJointGroups> w_rest{};
  for (std::size_t g = 0; g < w_rest.size(); ++g) {
    w_rest[g] = mpc.groups[g].w_rest;
  }
  const int free = FirstGroupLeftFree(
      groups, n_dev, plan_joints,
      std::span<const double>(w_rest.data(), static_cast<std::size_t>(groups.Count())));
  if (free < 0) {
    return {};
  }
  return "planner.segment.mpc.cost.angular_momentum reads every joint of the segment model, but "
         "a plan of the search model leaves out joints of " +
         (groups.n_groups == 0
              ? std::string("the arm (no planner.groups)")
              : "group '" + planner.groups[static_cast<std::size_t>(free)].name + "'") +
         " with no rest weight — give planner.segment.mpc.groups.<that group>.w_rest > 0 (#641 "
         "D9')";
}

}  // namespace rtc::catching
