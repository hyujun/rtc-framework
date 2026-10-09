// E1-F16: the docking keys — `robot.hand.docking.*` (the hand's identified
// capture set), `planner.search.nlp.*` and `planner.segment.mpc_docking.*`, and
// the `core:` sub-map both functions share. Pure parsing: no model, no solver.
// That the parsed structs are ACCEPTED by the core's Init needs a model and is
// tested where the shipped models are loaded.
//
// The key tests are driven from tables (key path, a good value, a bad value,
// kind) so that no key is forgotten: for each key, absent → the default (or the
// decision unset and named), a bad value → std::invalid_argument naming the
// full key, the literal `TBD` → unset for a decision and refused for the rest,
// and an unknown key next to it → refused by name.
#include "rtc_controllers/catching/catching_params.hpp"
#include "rtc_controllers/catching/docking_params.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

using rtc::catching::ApplyHandDocking;
using rtc::catching::DockingGridMismatch;
using rtc::catching::HandDockingParams;
using rtc::catching::kMaxDockingFaces;
using rtc::catching::kMaxSegmentNodes;
using rtc::catching::MpcDockingSegmentCoreParams;
using rtc::catching::MpcDockingSegmentPlannerParams;
using rtc::catching::NlpCatchSearchParams;
using rtc::catching::ParseHandDockingParams;
using rtc::catching::ParseMpcDockingSegmentParams;
using rtc::catching::ParseNlpSearchParams;
using rtc::catching::ReadDockingCoreParams;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr int kNv = 6;

const char* const kHandRoot = "robot.hand.docking.";
const char* const kNlpRoot = "planner.search.nlp.";
const char* const kMpcRoot = "planner.segment.mpc_docking.";

// ── Building YAML from dotted paths ─────────────────────────────────────────────

using Pairs = std::vector<std::pair<std::string, std::string>>;

struct Tree {
  std::map<std::string, Tree> kids;
  std::string leaf;
  bool has_leaf{false};
};

void Insert(Tree& t, const std::string& path, const std::string& value) {
  Tree* cur = &t;
  std::size_t pos = 0;
  while (true) {
    const std::size_t dot = path.find('.', pos);
    const std::string seg =
        path.substr(pos, dot == std::string::npos ? std::string::npos : dot - pos);
    cur = &cur->kids[seg];
    if (dot == std::string::npos) {
      break;
    }
    pos = dot + 1;
  }
  cur->has_leaf = true;
  cur->leaf = value;
  cur->kids.clear();
}

std::string Emit(const Tree& t) {
  if (t.has_leaf) {
    return t.leaf;
  }
  std::string s = "{";
  bool first = true;
  for (const auto& kv : t.kids) {
    if (!first) {
      s += ", ";
    }
    first = false;
    s += kv.first + ": " + Emit(kv.second);
  }
  return s + "}";
}

/// The `catching:` map holding every (dotted path, YAML value text) pair.
YAML::Node Make(const Pairs& kv) {
  Tree t;
  for (const auto& p : kv) {
    Insert(t, p.first, p.second);
  }
  return YAML::Load(Emit(t));
}

/// `base` with `path` set to `value` (replaced when present).
Pairs With(Pairs base, const std::string& path, const std::string& value) {
  for (auto& p : base) {
    if (p.first == path) {
      p.second = value;
      return base;
    }
  }
  base.emplace_back(path, value);
  return base;
}

Pairs Without(Pairs base, const std::string& path) {
  base.erase(
      std::remove_if(base.begin(), base.end(), [&](const auto& p) { return p.first == path; }),
      base.end());
  return base;
}

std::string ParentOf(const std::string& path) {
  return path.substr(0, path.rfind('.'));
}

template <class F>
std::string ThrownMessage(F&& f) {
  try {
    f();
  } catch (const std::invalid_argument& e) {
    return e.what();
  }
  return "<no throw>";
}

bool Contains(const std::string& s, const std::string& what) {
  return s.find(what) != std::string::npos;
}

bool Same(double a, double b) {
  if (std::isnan(a) && std::isnan(b)) {
    return true;
  }
  if (a == b) {
    return true;
  }
  return std::fabs(a - b) <= 1e-12 * std::max(1.0, std::fabs(b));
}

double Last(const Eigen::VectorXd& v) {
  return v.size() == 0 ? -1.0 : v[v.size() - 1];
}

// ── The tables ──────────────────────────────────────────────────────────────────

enum class Kind { kTuning, kDecision };

template <class R>
struct Entry {
  std::string path;  ///< the full key
  std::string good;  ///< YAML text of a value inside the range, distinct from the default
  std::string bad;   ///< YAML text outside the range (never an ordering rule)
  Kind kind;
  std::function<double(const R&)> get;  ///< one number that shows the key was read
  double want;                          ///< get(parse(good))
  double unset{kNaN};                   ///< a decision's value when unset
};

const char* UnsetName(const HandDockingParams& r) {
  return r.FirstUnset();
}

template <class R>
const char* UnsetName(const R&) {
  return nullptr;
}

std::vector<Entry<HandDockingParams>> HandTable() {
  using R = HandDockingParams;
  const std::string p = kHandRoot;
  const Kind d = Kind::kDecision;
  const Kind t = Kind::kTuning;
  return {
      {p + "provisional", "false", "maybe", t, [](const R& r) { return r.provisional ? 1.0 : 0.0; },
       0.0},
      {p + "s_ent", "0.05", "0.6", d, [](const R& r) { return r.s_ent; }, 0.05},
      {p + "corridor.r_ent", "0.02", "0", d, [](const R& r) { return r.r_ent; }, 0.02},
      {p + "corridor.tan_theta", "0.4", "11", d, [](const R& r) { return r.tan_theta; }, 0.4},
      {p + "lateral.n_faces", "4", "2", d,
       [](const R& r) { return static_cast<double>(r.n_faces); }, 4.0, 0.0},
      {p + "lateral.faces_a", "[1, 0, -1, 0, 0, 1, 0, -1]", "[1, 0, -1, 0, 0, 1]", d,
       [](const R& r) { return r.face_a[2].y(); }, 1.0},
      {p + "lateral.faces_b", "[0.03, 0.03, 0.02, 0.02]", "[0.03, 0.9, 0.02, 0.02]", d,
       [](const R& r) { return r.face_b[3]; }, 0.02},
      {p + "lateral.rho_ref", "[0.001, -0.002]", "[0.001, 0.9]", d,
       [](const R& r) { return r.rho_ref.y(); }, -0.002},
      {p + "speed.c_min", "0.2", "0", d, [](const R& r) { return r.c_min; }, 0.2},
      {p + "speed.c_cap_max", "1.5", "21", d, [](const R& r) { return r.c_cap_max; }, 1.5},
      {p + "speed.c_ent_max", "1.2", "0", d, [](const R& r) { return r.c_ent_max; }, 1.2},
      {p + "speed.v_perp_max", "0.3", "21", d, [](const R& r) { return r.v_perp_max; }, 0.3},
      {p + "speed.a_brake", "6", "1001", d, [](const R& r) { return r.a_brake; }, 6.0},
      {p + "closure.delta_lo", "-0.01", "-3", d, [](const R& r) { return r.delta_lo; }, -0.01},
      {p + "closure.delta_hi", "0.04", "3", d, [](const R& r) { return r.delta_hi; }, 0.04},
      {p + "closure.sigma_tau", "0.005", "2", d, [](const R& r) { return r.sigma_tau; }, 0.005},
      {p + "impact.contact_point_hand", "[0, 0, 0.1]", "[0, 0, 0.9]", d,
       [](const R& r) { return r.contact_point_hand.z(); }, 0.1},
      {p + "impact.restitution", "0.4", "1.5", d, [](const R& r) { return r.restitution; }, 0.4},
      {p + "impact.e_max", "2.5", "0", t, [](const R& r) { return r.e_max; }, 2.5},
      {p + "impact.p_max", "0.1", "0", t, [](const R& r) { return r.p_max; }, 0.1},
  };
}

std::vector<Entry<NlpCatchSearchParams>> NlpOwnTable() {
  using R = NlpCatchSearchParams;
  const std::string p = kNlpRoot;
  const Kind t = Kind::kTuning;
  return {
      {p + "cand_dt", "0.01", "0", t, [](const R& r) { return r.cand_dt; }, 0.01},
      {p + "t_lead_min", "0.25", "0.001", t, [](const R& r) { return r.t_lead_min; }, 0.25},
      {p + "t_max", "0.8", "5", t, [](const R& r) { return r.t_max; }, 0.8},
      {p + "cand_capacity", "100", "0", t,
       [](const R& r) { return static_cast<double>(r.cand_capacity); }, 100.0},
      {p + "n_pre.min", "3", "0", t, [](const R& r) { return static_cast<double>(r.n_pre_min); },
       3.0},
      {p + "n_pre.max", "5", "25", t, [](const R& r) { return static_cast<double>(r.n_pre_max); },
       5.0},
      {p + "dt_pre_s", "0.08", "0.1000000004", t, [](const R& r) { return r.dt_pre; }, 0.08},
      {p + "stop.n_nodes", "7", "2", t, [](const R& r) { return static_cast<double>(r.n_stop); },
       7.0},
      {p + "stop.dt_s", "0.04", "0.3", t, [](const R& r) { return r.dt_stop; }, 0.04},
      {p + "stop.blocks", "[1, 2, 2, 2]", "[1, 1]", t,
       [](const R& r) { return static_cast<double>(r.stop_block_sizes[1]); }, 2.0},
      {p + "budget.budget_s", "0.03", "6", t, [](const R& r) { return r.budget_s; }, 0.03},
      {p + "budget.solve_s", "0.012", "0.0001", t, [](const R& r) { return r.solve_budget_s; },
       0.012},
      {p + "budget.start_lead_s", "0.006", "0.5", t, [](const R& r) { return r.start_lead_s; },
       0.006},
      {p + "budget.max_solves", "8", "33", t,
       [](const R& r) { return static_cast<double>(r.max_solves); }, 8.0},
      {p + "cost.w_time", "0.5", "-1", t, [](const R& r) { return r.w_time; }, 0.5},
      {p + "cost.w_switch", "0.6", "-1", t, [](const R& r) { return r.w_switch; }, 0.6},
      {p + "cost.rank_w_q", "2", "-1", t, [](const R& r) { return r.rank_w_q; }, 2.0},
      {p + "cost.rank_w_manip", "0.7", "-1", t, [](const R& r) { return r.rank_w_manip; }, 0.7},
      {p + "cost.t_ref_s", "0.4", "0", t, [](const R& r) { return r.t_ref_s; }, 0.4},
      {p + "continuous_tc", "true", "maybe", t,
       [](const R& r) { return r.continuous_tc ? 1.0 : 0.0; }, 1.0},
      {p + "follow_window", "3", "-2", t,
       [](const R& r) { return static_cast<double>(r.follow_window); }, 3.0},
      {p + "rest_tol", "0.002", "0", t, [](const R& r) { return r.rest_tol; }, 0.002},
      {p + "rt_state_age_max_s", "0.08", "2", t, [](const R& r) { return r.rt_state_age_max_s; },
       0.08},
  };
}

std::vector<Entry<MpcDockingSegmentPlannerParams>> MpcOwnTable() {
  using R = MpcDockingSegmentPlannerParams;
  const std::string p = kMpcRoot;
  const Kind t = Kind::kTuning;
  return {
      {p + "switch_margin", "1.5", "0", t, [](const R& r) { return r.switch_margin; }, 1.5},
      {p + "eta_v", "0.8", "1", t, [](const R& r) { return r.eta_v; }, 0.8},
      {p + "approach.n_pre_max", "8", "25", t,
       [](const R& r) { return static_cast<double>(r.n_pre_max); }, 8.0},
      {p + "approach.dt_pre_s", "0.08", "0.1000000004", t, [](const R& r) { return r.dt_pre_s; },
       0.08},
      {p + "approach.rest_tol", "0.002", "0", t, [](const R& r) { return r.rest_tol; }, 0.002},
      {p + "stop.n_nodes", "7", "2", t, [](const R& r) { return static_cast<double>(r.n_stop); },
       7.0},
      {p + "stop.dt_s", "0.04", "0.3", t, [](const R& r) { return r.dt_stop_s; }, 0.04},
      {p + "stop.blocks", "[1, 2, 2, 2]", "[1, 1]", t,
       [](const R& r) { return static_cast<double>(r.stop_block_sizes[1]); }, 2.0},
      {p + "budget.first_s", "0.04", "6", t, [](const R& r) { return r.budget_first_s; }, 0.04},
      {p + "budget.replan_s", "0.03", "0", t, [](const R& r) { return r.budget_replan_s; }, 0.03},
      {p + "replan.same_point", "false", "maybe", t,
       [](const R& r) { return r.replan_same_point ? 1.0 : 0.0; }, 0.0},
      {p + "publish.slack_c_max", "0.01", "-1", t, [](const R& r) { return r.slack_c_max; }, 0.01},
      {p + "publish.slack_v_max", "0.02", "-1", t, [](const R& r) { return r.slack_v_max; }, 0.02},
  };
}

/// The `core:` keys, relative to the `core` map.
std::vector<Entry<MpcDockingSegmentCoreParams>> CoreTable() {
  using R = MpcDockingSegmentCoreParams;
  const Kind t = Kind::kTuning;
  return {
      {"u_scale", "500", "0", t, [](const R& r) { return r.u_scale; }, 500.0},
      {"cost.r_tau", "2.5", "-1", t, [](const R& r) { return Last(r.r_tau); }, 2.5},
      {"cost.r_acc", "3.5", "-1", t, [](const R& r) { return Last(r.r_acc); }, 3.5},
      {"cost.r_jerk", "4.5", "0", t, [](const R& r) { return Last(r.r_jerk); }, 4.5},
      {"cost.w_q_nom", "0.25", "-1", t, [](const R& r) { return Last(r.w_q_nom); }, 0.25},
      {"cost.w_manip", "0.3", "-1", t, [](const R& r) { return r.w_manip; }, 0.3},
      {"cost.manip_d_lin", "2", "0", t, [](const R& r) { return r.manip_d_lin; }, 2.0},
      {"cost.manip_d_ang", "3", "0", t, [](const R& r) { return r.manip_d_ang; }, 3.0},
      {"cost.manip_delta", "0.01", "0", t, [](const R& r) { return r.manip_delta; }, 0.01},
      {"catch.q_p", "[1, 2, 3]", "[1, 2]", t, [](const R& r) { return r.q_p[1]; }, 2.0},
      {"catch.q_v", "[4, 5, 6]", "[1, -2, 3]", t, [](const R& r) { return r.q_v[2]; }, 6.0},
      {"catch.sigma_T", "0.2", "0", t, [](const R& r) { return r.sigma_T; }, 0.2},
      {"catch.nu_ref", "[0.1, 0.2, -0.7]", "[0, 0, 0.5]", t, [](const R& r) { return r.nu_ref[2]; },
       -0.7},
      {"catch.q_rho_f", "[7, 8]", "[7]", t, [](const R& r) { return r.q_rho_f[1]; }, 8.0},
      {"catch.q_nu_f", "[1, 1, 9]", "[1, 1, -9]", t, [](const R& r) { return r.q_nu_f[2]; }, 9.0},
      {"catch.w_impact", "0.5", "-1", t, [](const R& r) { return r.w_impact; }, 0.5},
      {"catch.e_ref", "2", "0", t, [](const R& r) { return r.e_ref; }, 2.0},
      {"approach.window", "0.2", "6", t, [](const R& r) { return r.approach_window; }, 0.2},
      {"approach.lambda1_c", "2", "-1", t, [](const R& r) { return r.lambda1_c; }, 2.0},
      {"approach.lambda2_c", "0.5", "-1", t, [](const R& r) { return r.lambda2_c; }, 0.5},
      {"approach.lambda1_v", "3", "-1", t, [](const R& r) { return r.lambda1_v; }, 3.0},
      {"approach.lambda2_v", "0.6", "-1", t, [](const R& r) { return r.lambda2_v; }, 0.6},
      {"capture.chance", "false", "maybe", t, [](const R& r) { return r.chance ? 1.0 : 0.0; }, 0.0},
      {"capture.face_eps", "0.02", "0.5", t,
       [](const R& r) { return r.face_eps[kMaxDockingFaces - 1]; }, 0.02},
      {"capture.speed_faces", "6", "2", t,
       [](const R& r) { return static_cast<double>(r.speed_faces); }, 6.0},
      {"capture.eps_nu", "0.02", "0.5", t, [](const R& r) { return r.eps_nu; }, 0.02},
      {"capture.eps_sigma", "0.001", "0", t, [](const R& r) { return r.eps_sigma; }, 0.001},
      {"timing.row", "false", "maybe", t, [](const R& r) { return r.timing_row ? 1.0 : 0.0; }, 0.0},
      {"timing.eps_t", "0.1", "0", t, [](const R& r) { return r.eps_t; }, 0.1},
      {"stop.r_jerk", "3", "0", t, [](const R& r) { return Last(r.r_jerk_stop); }, 3.0},
      {"stop.w_perp", "5", "-1", t, [](const R& r) { return r.w_perp; }, 5.0},
      {"rows.accel_box", "true", "maybe", t, [](const R& r) { return r.accel_box ? 1.0 : 0.0; },
       1.0},
      {"rows.jerk_box", "true", "maybe", t, [](const R& r) { return r.jerk_box ? 1.0 : 0.0; }, 1.0},
      {"catch_time.delta_t_step", "0.02", "0", t, [](const R& r) { return r.delta_t_step; }, 0.02},
      {"catch_time.mu_init_post_box", "200", "0", t, [](const R& r) { return r.mu_init_post_box; },
       200.0},
      {"catch_time.mu_init_terminal", "300", "0", t, [](const R& r) { return r.mu_init_terminal; },
       300.0},
      {"sqp.max_iterations", "30", "0", t,
       [](const R& r) { return static_cast<double>(r.max_iterations); }, 30.0},
      {"sqp.armijo_eta", "0.01", "1", t, [](const R& r) { return r.armijo_eta; }, 0.01},
      {"sqp.backtrack_beta", "0.4", "0", t, [](const R& r) { return r.backtrack_beta; }, 0.4},
      {"sqp.max_backtracks", "5", "101", t,
       [](const R& r) { return static_cast<double>(r.max_backtracks); }, 5.0},
      {"sqp.delta_tr", "0.2", "0", t, [](const R& r) { return r.delta_tr; }, 0.2},
      {"sqp.mu_init", "50", "0", t,
       [](const R& r) { return r.mu_init[rtc::catching::kNumDockingElasticGroups - 1]; }, 50.0},
      {"sqp.mu_growth", "5", "1", t, [](const R& r) { return r.mu_growth; }, 5.0},
      {"sqp.mu_max", "100000", "0", t, [](const R& r) { return r.mu_max; }, 1e5},
      {"sqp.mu_min_gain", "0.2", "1", t, [](const R& r) { return r.mu_min_gain; }, 0.2},
      {"sqp.stall_window", "4", "17", t,
       [](const R& r) { return static_cast<double>(r.stall_window); }, 4.0},
      {"sqp.stall_reduction", "0.02", "1", t, [](const R& r) { return r.stall_reduction; }, 0.02},
      {"sqp.tol_violation", "0.00001", "0", t, [](const R& r) { return r.tol_violation; }, 1e-5},
      {"sqp.tol_kkt", "0.001", "0", t, [](const R& r) { return r.tol_kkt; }, 1e-3},
      {"sqp.tol_complementarity", "0.0001", "0", t,
       [](const R& r) { return r.tol_complementarity; }, 1e-4},
      {"sqp.tol_linear", "0.000001", "0", t, [](const R& r) { return r.tol_linear; }, 1e-6},
      {"init.w_q", "2", "0", t, [](const R& r) { return r.init_w_q; }, 2.0},
      {"init.w_v", "0.1", "-1", t, [](const R& r) { return r.init_w_v; }, 0.1},
      {"init.pinv_damping", "0.1", "-1", t, [](const R& r) { return r.init_pinv_damping; }, 0.1},
      {"solver.eps_abs", "0.000001", "0", t, [](const R& r) { return r.solver.eps_abs; }, 1e-6},
      {"solver.eps_rel", "0.00000001", "-1", t, [](const R& r) { return r.solver.eps_rel; }, 1e-8},
      {"solver.max_iter", "200", "0", t,
       [](const R& r) { return static_cast<double>(r.solver.max_iter); }, 200.0},
  };
}

/// The core table lifted onto a parser's result: its keys under `prefix`, read
/// from `core_of(result)`.
template <class R, class CoreOf>
void AddCore(std::vector<Entry<R>>& out, const std::string& prefix, CoreOf core_of) {
  for (const auto& e : CoreTable()) {
    const auto g = e.get;
    out.push_back({prefix + e.path, e.good, e.bad, e.kind,
                   [g, core_of](const R& r) { return g(core_of(r)); }, e.want, e.unset});
  }
}

std::vector<Entry<NlpCatchSearchParams>> NlpTable() {
  auto t = NlpOwnTable();
  AddCore<NlpCatchSearchParams>(
      t, std::string(kNlpRoot) + "core.",
      [](const NlpCatchSearchParams& r) -> const auto& { return r.core; });
  return t;
}

std::vector<Entry<MpcDockingSegmentPlannerParams>> MpcTable() {
  auto t = MpcOwnTable();
  AddCore<MpcDockingSegmentPlannerParams>(
      t, std::string(kMpcRoot) + "core.",
      [](const MpcDockingSegmentPlannerParams& r) -> const auto& { return r.core; });
  return t;
}

template <class R>
Pairs AllGood(const std::vector<Entry<R>>& table) {
  Pairs out;
  for (const auto& e : table) {
    out.emplace_back(e.path, e.good);
  }
  return out;
}

/// Every key of `table`: absent, good, bad, `TBD`, and an unknown key beside it.
template <class R, class Parse>
void RunTable(const std::vector<Entry<R>>& table, Parse parse, bool names_unset) {
  const Pairs all = AllGood(table);
  for (const auto& e : table) {
    SCOPED_TRACE(e.path);
    const bool decision = e.kind == Kind::kDecision;

    // good: read, and the value shows it
    {
      const R r = parse(Make(all));
      EXPECT_TRUE(Same(e.get(r), e.want)) << "got " << e.get(r) << ", want " << e.want;
    }
    // absent: the default, or the decision unset and named
    {
      const R r = parse(Make(Without(all, e.path)));
      if (decision) {
        EXPECT_TRUE(Same(e.get(r), e.unset)) << "got " << e.get(r);
        if (names_unset) {
          const char* u = UnsetName(r);
          ASSERT_NE(u, nullptr);
          EXPECT_STREQ(u, e.path.c_str());
        }
      } else {
        EXPECT_TRUE(Same(e.get(r), e.get(R{}))) << "got " << e.get(r) << ", default " << e.get(R{});
      }
    }
    // bad: refused, naming the full key
    {
      const std::string m =
          ThrownMessage([&] { static_cast<void>(parse(Make(With(all, e.path, e.bad)))); });
      EXPECT_TRUE(Contains(m, e.path)) << "message: " << m;
    }
    // TBD: a decision is unset, everything else is refused
    if (decision) {
      const R r = parse(Make(With(all, e.path, "TBD")));
      EXPECT_TRUE(Same(e.get(r), e.unset));
      if (names_unset) {
        const char* u = UnsetName(r);
        ASSERT_NE(u, nullptr);
        EXPECT_STREQ(u, e.path.c_str());
      }
    } else {
      const std::string m =
          ThrownMessage([&] { static_cast<void>(parse(Make(With(all, e.path, "TBD")))); });
      EXPECT_TRUE(Contains(m, e.path)) << "message: " << m;
    }
    // an unknown key in the same map: refused by name
    {
      const std::string unknown = ParentOf(e.path) + ".zz_unknown";
      const std::string m =
          ThrownMessage([&] { static_cast<void>(parse(Make(With(all, unknown, "1")))); });
      EXPECT_TRUE(Contains(m, unknown)) << "message: " << m;
      EXPECT_TRUE(Contains(m, "is not a key")) << "message: " << m;
    }
  }
}

const auto kParseHand = [](const YAML::Node& n) { return ParseHandDockingParams(n); };
const auto kParseNlp = [](const YAML::Node& n) { return ParseNlpSearchParams(n, kNv); };
const auto kParseMpc = [](const YAML::Node& n) { return ParseMpcDockingSegmentParams(n, kNv); };

// ── Every key ───────────────────────────────────────────────────────────────────

TEST(DockingParamsKeys, HandDockingEveryKey) {
  RunTable<HandDockingParams>(HandTable(), kParseHand, true);
}

TEST(DockingParamsKeys, NlpSearchEveryKey) {
  RunTable<NlpCatchSearchParams>(NlpTable(), kParseNlp, false);
}

TEST(DockingParamsKeys, MpcDockingSegmentEveryKey) {
  RunTable<MpcDockingSegmentPlannerParams>(MpcTable(), kParseMpc, false);
}

TEST(DockingParamsKeys, TheTablesAreNotEmptyAndTheirKeysAreDistinct) {
  const auto hand = HandTable();
  const auto nlp = NlpTable();
  const auto mpc = MpcTable();
  EXPECT_EQ(hand.size(), 20u);
  EXPECT_EQ(CoreTable().size(), 57u);
  EXPECT_EQ(nlp.size(), 23u + CoreTable().size());
  EXPECT_EQ(mpc.size(), 13u + CoreTable().size());
  for (const auto& table : {AllGood(hand), AllGood(nlp), AllGood(mpc)}) {
    std::map<std::string, int> seen;
    for (const auto& p : table) {
      EXPECT_EQ(++seen[p.first], 1) << p.first;
    }
  }
}

TEST(DockingParamsKeys, AbsentMapsGiveTheDefaults) {
  const YAML::Node empty = YAML::Load("{}");
  const NlpCatchSearchParams n = kParseNlp(empty);
  const NlpCatchSearchParams dn;
  EXPECT_DOUBLE_EQ(n.cand_dt, dn.cand_dt);
  EXPECT_EQ(n.n_stop, dn.n_stop);
  EXPECT_EQ(n.n_stop_blocks, dn.n_stop_blocks);
  EXPECT_EQ(n.stop_block_sizes, dn.stop_block_sizes);
  EXPECT_DOUBLE_EQ(n.core.u_scale, dn.core.u_scale);
  EXPECT_EQ(n.core.r_jerk.size(), 0);
  EXPECT_EQ(n.wait_pose_n, 0);

  const MpcDockingSegmentPlannerParams m = kParseMpc(empty);
  const MpcDockingSegmentPlannerParams dm;
  EXPECT_DOUBLE_EQ(m.switch_margin, 1.0);
  EXPECT_DOUBLE_EQ(m.eta_v, 0.9);
  EXPECT_EQ(m.n_pre_max, dm.n_pre_max);
  EXPECT_EQ(m.DtPreNs(), 100'000'000);
  EXPECT_EQ(m.DtStopNs(), 50'000'000);
  EXPECT_EQ(m.stop_block_sizes, dm.stop_block_sizes);
  EXPECT_DOUBLE_EQ(m.core.sigma_T, dm.core.sigma_T);

  const HandDockingParams h = kParseHand(empty);
  EXPECT_TRUE(h.provisional);
  ASSERT_NE(h.FirstUnset(), nullptr);
  EXPECT_STREQ(h.FirstUnset(), "robot.hand.docking.s_ent");
  EXPECT_TRUE(std::isinf(h.e_max));
  EXPECT_TRUE(std::isinf(h.p_max));
  EXPECT_EQ(h.n_faces, 0);

  // The same for the maps present and empty, or present as `docking:` with nothing in it.
  EXPECT_STREQ(kParseHand(YAML::Load("{robot: {hand: {docking: }}}")).FirstUnset(),
               "robot.hand.docking.s_ent");
  EXPECT_NO_THROW(static_cast<void>(kParseNlp(YAML::Load("{planner: {search: {nlp: {}}}}"))));
  EXPECT_NO_THROW(
      static_cast<void>(kParseMpc(YAML::Load("{planner: {segment: {mpc_docking: }}}"))));
}

TEST(DockingParamsKeys, ARootThatIsNotAMapAndASectionThatIsNotAMapAreRefused) {
  for (const char* bad : {"3", "[1, 2]"}) {
    EXPECT_THROW(static_cast<void>(kParseHand(YAML::Load(bad))), std::invalid_argument) << bad;
    EXPECT_THROW(static_cast<void>(kParseNlp(YAML::Load(bad))), std::invalid_argument) << bad;
    EXPECT_THROW(static_cast<void>(kParseMpc(YAML::Load(bad))), std::invalid_argument) << bad;
  }
  EXPECT_TRUE(Contains(ThrownMessage([] {
                         static_cast<void>(kParseHand(YAML::Load("{robot: {hand: {docking: 3}}}")));
                       }),
                       "robot.hand.docking"));
  EXPECT_TRUE(Contains(ThrownMessage([] {
                         static_cast<void>(
                             kParseHand(YAML::Load("{robot: {hand: {docking: {speed: 1}}}}")));
                       }),
                       "robot.hand.docking.speed"));
  EXPECT_TRUE(Contains(ThrownMessage([] {
                         static_cast<void>(kParseNlp(YAML::Load("{planner: {search: {nlp: 3}}}")));
                       }),
                       "planner.search.nlp"));
  EXPECT_TRUE(Contains(ThrownMessage([] {
                         static_cast<void>(
                             kParseNlp(YAML::Load("{planner: {search: {nlp: {core: 3}}}}")));
                       }),
                       "planner.search.nlp.core"));
  EXPECT_TRUE(Contains(
      ThrownMessage([] {
        static_cast<void>(kParseMpc(YAML::Load("{planner: {segment: {mpc_docking: {stop: 3}}}}")));
      }),
      "planner.segment.mpc_docking.stop"));
}

TEST(DockingParamsKeys, TheNlpMapToleratesItsTwoForeignKeys) {
  const NlpCatchSearchParams p = kParseNlp(
      YAML::Load("{planner: {search: {nlp: {mode: grid, ik: {anything: 1}, cand_dt: 0.01}}}}"));
  EXPECT_DOUBLE_EQ(p.cand_dt, 0.01);
}

// L3 §4.9: the search does not judge where the catch point is. The removed
// `catch_box` is not read and does not make the parser throw, whatever is in
// it — the binding parks on the key (kRemovedCatchingKeys), which keeps the
// robot up where a throw here would fail the whole configure.
TEST(DockingParamsKeys, TheRemovedCatchBoxIsNotReadAndIsReportedAsRemoved) {
  const NlpCatchSearchParams defaults = kParseNlp(YAML::Load("{}"));
  for (const char* box : {"{min: [0, -0.2, 0.1], max: [0.6, 0.2, 0.5]}", "TBD", "3",
                          "{min: [1, 0, 0], max: [0, 1, 1]}", "{zz_unknown: 1}"}) {
    const std::string y = std::string("{planner: {search: {nlp: {catch_box: ") + box + "}}}}";
    const YAML::Node tree = YAML::Load(y);
    NlpCatchSearchParams p;
    ASSERT_NO_THROW(p = kParseNlp(tree)) << y;
    EXPECT_DOUBLE_EQ(p.cand_dt, defaults.cand_dt) << y;
    const auto removed = rtc::catching::FindRemovedCatchingKeys(tree);
    ASSERT_EQ(removed.size(), 1U) << y;
    EXPECT_STREQ(removed[0], "planner.search.nlp.catch_box") << y;
  }
}

// ── The hand's flat face lists ──────────────────────────────────────────────────

Pairs HandBase() {
  return AllGood(HandTable());
}

TEST(DockingParamsFaces, RightLengthsAreAcceptedAndNormalised) {
  const Pairs b3 = With(With(With(HandBase(), std::string(kHandRoot) + "lateral.n_faces", "3"),
                             std::string(kHandRoot) + "lateral.faces_a",
                             "[1, 0, -0.5, 0.8660254037844386, -0.5, -0.8660254037844386]"),
                        std::string(kHandRoot) + "lateral.faces_b", "[0.03, 0.02, 0.01]");
  const HandDockingParams h3 = kParseHand(Make(b3));
  EXPECT_EQ(h3.n_faces, 3);
  EXPECT_NEAR(h3.face_a[1].x(), -0.5, 1e-12);
  EXPECT_NEAR(h3.face_a[2].y(), -0.8660254037844386, 1e-12);
  EXPECT_DOUBLE_EQ(h3.face_b[2], 0.01);
  EXPECT_EQ(h3.face_a[3].x(), 0.0) << "entries past n_faces stay zero";
  EXPECT_EQ(h3.face_b[3], 0.0);
  EXPECT_EQ(h3.FirstUnset(), nullptr);

  // The full capacity.
  std::string a = "[";
  std::string bb = "[";
  for (int i = 0; i < kMaxDockingFaces; ++i) {
    a += std::string(i ? ", " : "") + (i % 2 == 0 ? "0.6, 0.8" : "-0.8, 0.6");
    bb += std::string(i ? ", " : "") + "0.02";
  }
  const Pairs b8 = With(With(With(HandBase(), std::string(kHandRoot) + "lateral.n_faces", "8"),
                             std::string(kHandRoot) + "lateral.faces_a", a + "]"),
                        std::string(kHandRoot) + "lateral.faces_b", bb + "]");
  const HandDockingParams h8 = kParseHand(Make(b8));
  EXPECT_EQ(h8.n_faces, kMaxDockingFaces);
  EXPECT_NEAR(h8.face_a[7].x(), -0.8, 1e-12);
}

TEST(DockingParamsFaces, WrongLengthsAreRefusedNamingTheKeyAndBothLengths) {
  const std::string a_key = std::string(kHandRoot) + "lateral.faces_a";
  const std::string b_key = std::string(kHandRoot) + "lateral.faces_b";
  // faces_a: 8 expected for n_faces = 4.
  for (const char* bad : {"[1, 0, -1, 0, 0, 1]", "[1, 0, -1, 0, 0, 1, 0, -1, 1, 0]", "[]", "5"}) {
    const std::string m =
        ThrownMessage([&] { static_cast<void>(kParseHand(Make(With(HandBase(), a_key, bad)))); });
    EXPECT_TRUE(Contains(m, a_key)) << bad << " -> " << m;
    EXPECT_TRUE(Contains(m, "2*n_faces = 8")) << bad << " -> " << m;
  }
  {
    const std::string m = ThrownMessage([&] {
      static_cast<void>(kParseHand(Make(With(HandBase(), a_key, "[1, 0, -1, 0, 0, 1]"))));
    });
    EXPECT_TRUE(Contains(m, "sequence of 6 entries")) << m;
  }
  // faces_b: 4 expected.
  for (const char* bad : {"[0.03, 0.03, 0.02]", "[0.03, 0.03, 0.02, 0.02, 0.01]", "0.03"}) {
    const std::string m =
        ThrownMessage([&] { static_cast<void>(kParseHand(Make(With(HandBase(), b_key, bad)))); });
    EXPECT_TRUE(Contains(m, b_key)) << bad << " -> " << m;
    EXPECT_TRUE(Contains(m, "n_faces = 4")) << bad << " -> " << m;
  }
  {
    const std::string m = ThrownMessage([&] {
      static_cast<void>(kParseHand(Make(With(HandBase(), b_key, "[0.03, 0.03, 0.02]"))));
    });
    EXPECT_TRUE(Contains(m, "sequence of 3 entries")) << m;
  }
}

TEST(DockingParamsFaces, ANormalIsUnitToWithinATolerance) {
  const std::string a_key = std::string(kHandRoot) + "lateral.faces_a";
  // Norm 1.01: refused.
  {
    const std::string m = ThrownMessage([&] {
      static_cast<void>(kParseHand(Make(With(HandBase(), a_key, "[1.01, 0, -1, 0, 0, 1, 0, -1]"))));
    });
    EXPECT_TRUE(Contains(m, a_key)) << m;
    EXPECT_TRUE(Contains(m, "unit")) << m;
  }
  // A zero normal: refused.
  EXPECT_THROW(
      static_cast<void>(kParseHand(Make(With(HandBase(), a_key, "[0, 0, -1, 0, 0, 1, 0, -1]")))),
      std::invalid_argument);
  // Norm 1.0004: accepted and returned with norm 1.
  const HandDockingParams h =
      kParseHand(Make(With(HandBase(), a_key, "[1.0004, 0, -0.6002, 0.8003, 0, 1, 0, -1]")));
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(h.face_a[static_cast<std::size_t>(i)].norm(), 1.0, 1e-12) << i;
  }
  EXPECT_NEAR(h.face_a[0].x(), 1.0, 1e-12);
  EXPECT_NEAR(h.face_a[1].x(), -0.6, 1e-3);
  EXPECT_NEAR(h.face_a[1].y() / h.face_a[1].x(), 0.8003 / -0.6002, 1e-12) << "direction is kept";
}

TEST(DockingParamsFaces, TheListsAreNotReadWhenNFacesIsUnset) {
  const std::string n_key = std::string(kHandRoot) + "lateral.n_faces";
  const std::string a_key = std::string(kHandRoot) + "lateral.faces_a";
  const std::string b_key = std::string(kHandRoot) + "lateral.faces_b";
  for (const Pairs& base : {Without(HandBase(), n_key), With(HandBase(), n_key, "TBD")}) {
    const Pairs garbage = With(With(base, a_key, "[not, numbers]"), b_key, "7");
    const HandDockingParams h = kParseHand(Make(garbage));
    EXPECT_EQ(h.n_faces, 0);
    ASSERT_NE(h.FirstUnset(), nullptr);
    EXPECT_STREQ(h.FirstUnset(), n_key.c_str());
  }
}

TEST(DockingParamsFaces, AListAsTbdOrAbsentIsUnsetAndNamed) {
  const std::string a_key = std::string(kHandRoot) + "lateral.faces_a";
  const std::string b_key = std::string(kHandRoot) + "lateral.faces_b";
  const HandDockingParams a = kParseHand(Make(With(HandBase(), a_key, "TBD")));
  ASSERT_NE(a.FirstUnset(), nullptr);
  EXPECT_STREQ(a.FirstUnset(), a_key.c_str());
  EXPECT_EQ(a.n_faces, 4);
  const HandDockingParams b = kParseHand(Make(With(HandBase(), b_key, "TBD")));
  ASSERT_NE(b.FirstUnset(), nullptr);
  EXPECT_STREQ(b.FirstUnset(), b_key.c_str());
  // Both unset: faces_a is named first.
  const HandDockingParams ab = kParseHand(Make(With(With(HandBase(), a_key, "TBD"), b_key, "TBD")));
  EXPECT_STREQ(ab.FirstUnset(), a_key.c_str());
}

// ── The ordering rules ──────────────────────────────────────────────────────────

TEST(DockingParamsOrdering, HandPairsMustBeStrictlyOrdered) {
  const std::string root = kHandRoot;
  for (const char* cap : {"0.2", "0.1"}) {  // equal to c_min, below c_min
    const std::string m = ThrownMessage([&] {
      static_cast<void>(kParseHand(Make(With(HandBase(), root + "speed.c_cap_max", cap))));
    });
    EXPECT_TRUE(Contains(m, root + "speed.c_cap_max")) << m;
    EXPECT_TRUE(Contains(m, root + "speed.c_min")) << m;
  }
  for (const char* hi : {"-0.01", "-0.5"}) {  // equal to delta_lo, below it
    const std::string m = ThrownMessage([&] {
      static_cast<void>(kParseHand(Make(With(HandBase(), root + "closure.delta_hi", hi))));
    });
    EXPECT_TRUE(Contains(m, root + "closure.delta_hi")) << m;
    EXPECT_TRUE(Contains(m, root + "closure.delta_lo")) << m;
  }
  // One of a pair unset: no ordering to judge.
  EXPECT_NO_THROW(
      static_cast<void>(kParseHand(Make(With(HandBase(), root + "speed.c_min", "TBD")))));
  EXPECT_NO_THROW(static_cast<void>(kParseHand(Make(With(
      With(HandBase(), root + "closure.delta_lo", "TBD"), root + "closure.delta_hi", "-1.5")))));
}

TEST(DockingParamsOrdering, NlpRules) {
  const std::string r = kNlpRoot;
  const auto parse = [&](const Pairs& kv) { return kParseNlp(Make(kv)); };
  const auto msg = [&](const Pairs& kv) {
    return ThrownMessage([&] { static_cast<void>(parse(kv)); });
  };
  // t_max > t_lead_min
  {
    const std::string m = msg({{r + "t_lead_min", "0.5"}, {r + "t_max", "0.5"}});
    EXPECT_TRUE(Contains(m, r + "t_max") && Contains(m, r + "t_lead_min")) << m;
  }
  EXPECT_NO_THROW(static_cast<void>(parse({{r + "t_lead_min", "0.5"}, {r + "t_max", "0.51"}})));
  // solve_s ≤ budget_s (equal is fine)
  {
    const std::string m = msg({{r + "budget.budget_s", "0.01"}, {r + "budget.solve_s", "0.02"}});
    EXPECT_TRUE(Contains(m, r + "budget.solve_s") && Contains(m, r + "budget.budget_s")) << m;
  }
  EXPECT_NO_THROW(
      static_cast<void>(parse({{r + "budget.budget_s", "0.02"}, {r + "budget.solve_s", "0.02"}})));
  // n_pre.max ≥ n_pre.min
  {
    const std::string m = msg({{r + "n_pre.min", "4"}, {r + "n_pre.max", "3"}});
    EXPECT_TRUE(Contains(m, r + "n_pre.max") && Contains(m, r + "n_pre.min")) << m;
  }
  EXPECT_NO_THROW(static_cast<void>(parse({{r + "n_pre.min", "4"}, {r + "n_pre.max", "4"}})));
  // n_pre.max alone below the default min (2) is the same rule
  EXPECT_THROW(static_cast<void>(parse({{r + "n_pre.max", "1"}})), std::invalid_argument);
}

TEST(DockingParamsOrdering, StopBlocksSumToTheNodeCount) {
  const std::string r = kNlpRoot;
  const std::string q = kMpcRoot;
  for (const std::string& root : {r, q}) {
    const auto msg = [&](const Pairs& kv) {
      return ThrownMessage([&] {
        if (root == r) {
          static_cast<void>(kParseNlp(Make(kv)));
        } else {
          static_cast<void>(kParseMpc(Make(kv)));
        }
      });
    };
    // Both given, and consistent: a different grid is fine.
    {
      const Pairs ok = {{root + "stop.n_nodes", "10"}, {root + "stop.blocks", "[2, 3, 5]"}};
      if (root == r) {
        const NlpCatchSearchParams p = kParseNlp(Make(ok));
        EXPECT_EQ(p.n_stop, 10);
        EXPECT_EQ(p.n_stop_blocks, 3);
        EXPECT_EQ(p.stop_block_sizes[2], 5);
        EXPECT_EQ(p.stop_block_sizes[3], 0) << "the default's tail must not survive";
      } else {
        const MpcDockingSegmentPlannerParams p = kParseMpc(Make(ok));
        EXPECT_EQ(p.n_stop, 10);
        EXPECT_EQ(p.n_stop_blocks, 3);
        EXPECT_EQ(p.stop_block_sizes[2], 5);
        EXPECT_EQ(p.stop_block_sizes[3], 0);
      }
    }
    // Inconsistent: refused naming both keys and both numbers.
    {
      const std::string m =
          msg({{root + "stop.n_nodes", "10"}, {root + "stop.blocks", "[2, 3, 4]"}});
      EXPECT_TRUE(Contains(m, root + "stop.blocks")) << m;
      EXPECT_TRUE(Contains(m, root + "stop.n_nodes")) << m;
      EXPECT_TRUE(Contains(m, "9") && Contains(m, "10")) << m;
    }
    // One given without the other, and the pair no longer sums.
    EXPECT_TRUE(Contains(msg({{root + "stop.n_nodes", "8"}}), root + "stop.blocks"));
    EXPECT_TRUE(Contains(msg({{root + "stop.blocks", "[1, 1, 1]"}}), root + "stop.n_nodes"));
    // A block of 0 is not a block.
    EXPECT_TRUE(Contains(msg({{root + "stop.n_nodes", "7"}, {root + "stop.blocks", "[1, 0, 6]"}}),
                         root + "stop.blocks[1]"));
    EXPECT_TRUE(
        Contains(msg({{root + "stop.blocks", "[1, 2, 2, 2, 0]"}}), root + "stop.blocks[4]"));
  }
}

TEST(DockingParamsOrdering, ThePreAndStopNodesFitTheCapacity) {
  const std::string r = kNlpRoot;
  const std::string q = kMpcRoot;
  // 17 + 7 = 24 fits, 18 + 7 = 25 does not.
  EXPECT_NO_THROW(static_cast<void>(kParseNlp(Make({{r + "n_pre.max", "17"}}))));
  {
    const std::string m =
        ThrownMessage([&] { static_cast<void>(kParseNlp(Make({{r + "n_pre.max", "18"}}))); });
    EXPECT_TRUE(Contains(m, r + "n_pre.max") && Contains(m, r + "stop.n_nodes")) << m;
    EXPECT_TRUE(Contains(m, std::to_string(kMaxSegmentNodes))) << m;
  }
  EXPECT_NO_THROW(static_cast<void>(kParseMpc(Make({{q + "approach.n_pre_max", "17"}}))));
  {
    const std::string m = ThrownMessage(
        [&] { static_cast<void>(kParseMpc(Make({{q + "approach.n_pre_max", "18"}}))); });
    EXPECT_TRUE(Contains(m, q + "approach.n_pre_max") && Contains(m, q + "stop.n_nodes")) << m;
    EXPECT_TRUE(Contains(m, std::to_string(kMaxSegmentNodes))) << m;
  }
  // A longer stop leaves less room for the pre-catch nodes.
  {
    const std::string m = ThrownMessage([&] {
      static_cast<void>(kParseNlp(Make({{r + "n_pre.max", "10"},
                                        {r + "stop.n_nodes", "15"},
                                        {r + "stop.blocks", "[3, 4, 8]"}})));
    });
    EXPECT_TRUE(Contains(m, r + "n_pre.max") && Contains(m, r + "stop.n_nodes")) << m;
  }
}

TEST(DockingParamsOrdering, TheTwoLambdaPairsMustEachBePositive) {
  const std::string c = std::string(kNlpRoot) + "core.approach.";
  for (const char* which : {"c", "v"}) {
    const std::string l1 = c + "lambda1_" + which;
    const std::string l2 = c + "lambda2_" + which;
    const std::string m =
        ThrownMessage([&] { static_cast<void>(kParseNlp(Make({{l1, "0"}, {l2, "0"}}))); });
    EXPECT_TRUE(Contains(m, l1) && Contains(m, l2)) << m;
    // Either one alone carries the penalty.
    const NlpCatchSearchParams a = kParseNlp(Make({{l1, "0"}, {l2, "2"}}));
    EXPECT_DOUBLE_EQ(which[0] == 'c' ? a.core.lambda2_c : a.core.lambda2_v, 2.0);
    EXPECT_NO_THROW(static_cast<void>(kParseNlp(Make({{l1, "2"}, {l2, "0"}}))));
  }
  // Zeroing one of a pair whose other is already zero by default is caught too.
  EXPECT_THROW(static_cast<void>(kParseNlp(Make({{c + "lambda1_c", "0"}}))), std::invalid_argument);
}

// ── ApplyHandDocking ────────────────────────────────────────────────────────────

TEST(DockingParamsApply, WritesTheHandAndTheDerivedDelta0AndNothingElse) {
  const HandDockingParams h = kParseHand(Make(HandBase()));
  ASSERT_EQ(h.FirstUnset(), nullptr);

  MpcDockingSegmentCoreParams core;
  // Markers the call must leave alone.
  core.n_pre = 5;
  core.dt_pre = 0.07;
  core.n_stop = 9;
  core.dt_stop = 0.03;
  core.n_blocks = 3;
  core.block_sizes = {2, 3, 4};
  core.face_eps.fill(0.0321);
  core.u_scale = 123.0;
  core.w_manip = 0.77;
  core.q_p = Eigen::Vector3d(1, 2, 3);
  core.r_jerk = Eigen::VectorXd::Constant(6, 4.0);
  core.w_perp = 8.0;
  core.eps_t = 0.0777;
  core.sigma_T = 0.321;
  core.timing_row = false;
  core.max_iterations = 17;
  const MpcDockingSegmentCoreParams before = core;

  ApplyHandDocking(h, 0.098, 0.034, 0.058, core);

  EXPECT_DOUBLE_EQ(core.s_ent, 0.05);
  EXPECT_DOUBLE_EQ(core.r_ent, 0.02);
  EXPECT_DOUBLE_EQ(core.tan_theta, 0.4);
  EXPECT_EQ(core.n_faces, 4);
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(core.face_a[i], h.face_a[i]) << i;
    EXPECT_DOUBLE_EQ(core.face_b[i], h.face_b[i]) << i;
  }
  EXPECT_EQ(core.rho_ref, h.rho_ref);
  EXPECT_DOUBLE_EQ(core.c_min, 0.2);
  EXPECT_DOUBLE_EQ(core.c_cap_max, 1.5);
  EXPECT_DOUBLE_EQ(core.c_ent_max, 1.2);
  EXPECT_DOUBLE_EQ(core.v_perp_max, 0.3);
  EXPECT_DOUBLE_EQ(core.a_brake, 6.0);
  EXPECT_DOUBLE_EQ(core.delta_lo, -0.01);
  EXPECT_DOUBLE_EQ(core.delta_hi, 0.04);
  EXPECT_DOUBLE_EQ(core.sigma_tau, 0.005);
  EXPECT_NEAR(core.delta_0, 0.098 - 0.034, 1e-15);
  EXPECT_EQ(core.contact_point_hand, h.contact_point_hand);
  EXPECT_DOUBLE_EQ(core.m_ball, 0.058);
  EXPECT_DOUBLE_EQ(core.restitution, 0.4);
  EXPECT_DOUBLE_EQ(core.e_max, 2.5);
  EXPECT_DOUBLE_EQ(core.p_max, 0.1);

  // Not touched.
  EXPECT_EQ(core.face_eps, before.face_eps);
  EXPECT_EQ(core.n_pre, before.n_pre);
  EXPECT_EQ(core.dt_pre, before.dt_pre);
  EXPECT_EQ(core.n_stop, before.n_stop);
  EXPECT_EQ(core.dt_stop, before.dt_stop);
  EXPECT_EQ(core.n_blocks, before.n_blocks);
  EXPECT_EQ(core.block_sizes, before.block_sizes);
  EXPECT_EQ(core.u_scale, before.u_scale);
  EXPECT_EQ(core.w_manip, before.w_manip);
  EXPECT_EQ(core.q_p, before.q_p);
  EXPECT_EQ(core.r_jerk, before.r_jerk);
  EXPECT_EQ(core.w_perp, before.w_perp);
  EXPECT_EQ(core.eps_t, before.eps_t);
  EXPECT_EQ(core.sigma_T, before.sigma_T);
  EXPECT_EQ(core.timing_row, before.timing_row);
  EXPECT_EQ(core.max_iterations, before.max_iterations);
  EXPECT_EQ(core.speed_faces, before.speed_faces);
}

TEST(DockingParamsApply, FaceEntriesPastNFacesStayAsTheyWere) {
  const std::string root = kHandRoot;
  const Pairs b3 =
      With(With(With(HandBase(), root + "lateral.n_faces", "3"), root + "lateral.faces_a",
                "[1, 0, -0.5, 0.8660254037844386, -0.5, -0.8660254037844386]"),
           root + "lateral.faces_b", "[0.03, 0.02, 0.01]");
  const HandDockingParams h = kParseHand(Make(b3));
  MpcDockingSegmentCoreParams core;  // four faces by default
  const auto a3 = core.face_a[3];
  const double b3_before = core.face_b[3];
  ApplyHandDocking(h, 0.1, 0.03, 0.05, core);
  EXPECT_EQ(core.n_faces, 3);
  EXPECT_EQ(core.face_a[3], a3);
  EXPECT_EQ(core.face_b[3], b3_before);
  EXPECT_DOUBLE_EQ(core.face_b[2], 0.01);
}

// ── ReadDockingCoreParams ───────────────────────────────────────────────────────

TEST(DockingParamsCore, APerJointWeightIsOneScalarForEveryJoint) {
  MpcDockingSegmentCoreParams core;
  ReadDockingCoreParams(YAML::Load("{cost: {r_tau: 2.5, r_acc: 3.5, r_jerk: 4.5, w_q_nom: 0.25}, "
                                   "stop: {r_jerk: 6.5}}"),
                        "planner.search.nlp.core", 7, core);
  for (const Eigen::VectorXd* v :
       {&core.r_tau, &core.r_acc, &core.r_jerk, &core.w_q_nom, &core.r_jerk_stop}) {
    ASSERT_EQ(v->size(), 7);
  }
  EXPECT_TRUE((core.r_tau.array() == 2.5).all());
  EXPECT_TRUE((core.r_acc.array() == 3.5).all());
  EXPECT_TRUE((core.r_jerk.array() == 4.5).all());
  EXPECT_TRUE((core.w_q_nom.array() == 0.25).all());
  EXPECT_TRUE((core.r_jerk_stop.array() == 6.5).all());
}

TEST(DockingParamsCore, ZeroLeavesTheThreeOffWeightsEmpty) {
  MpcDockingSegmentCoreParams core;
  core.r_tau = Eigen::VectorXd::Constant(6, 1.0);
  core.r_acc = Eigen::VectorXd::Constant(6, 1.0);
  core.w_q_nom = Eigen::VectorXd::Constant(6, 1.0);
  ReadDockingCoreParams(YAML::Load("{cost: {r_tau: 0, r_acc: 0, w_q_nom: 0.0}}"), "x.core", 6,
                        core);
  EXPECT_EQ(core.r_tau.size(), 0);
  EXPECT_EQ(core.r_acc.size(), 0);
  EXPECT_EQ(core.w_q_nom.size(), 0);
  // r_jerk must be positive: 0 is not "off".
  const std::string m = ThrownMessage(
      [&] { ReadDockingCoreParams(YAML::Load("{cost: {r_jerk: 0}}"), "x.core", 6, core); });
  EXPECT_TRUE(Contains(m, "x.core.cost.r_jerk")) << m;
}

TEST(DockingParamsCore, FaceEpsAndMuInitFillEveryEntry) {
  MpcDockingSegmentCoreParams core;
  ReadDockingCoreParams(YAML::Load("{capture: {face_eps: 0.02}, sqp: {mu_init: 50}}"), "x.core", 6,
                        core);
  for (int i = 0; i < kMaxDockingFaces; ++i) {
    EXPECT_DOUBLE_EQ(core.face_eps[static_cast<std::size_t>(i)], 0.02) << i;
  }
  for (std::size_t i = 0; i < core.mu_init.size(); ++i) {
    EXPECT_DOUBLE_EQ(core.mu_init[i], 50.0) << i;
  }
}

TEST(DockingParamsCore, AnAbsentMapChangesNothing) {
  const MpcDockingSegmentCoreParams d;
  for (const YAML::Node& n : {YAML::Node(), YAML::Node(YAML::NodeType::Null), YAML::Load("~")}) {
    MpcDockingSegmentCoreParams core;
    ReadDockingCoreParams(n, "x.core", 6, core);
    EXPECT_EQ(core.u_scale, d.u_scale);
    EXPECT_EQ(core.n_faces, d.n_faces);
    EXPECT_EQ(core.face_eps, d.face_eps);
    EXPECT_EQ(core.mu_init, d.mu_init);
    EXPECT_EQ(core.r_jerk.size(), 0);
    EXPECT_EQ(core.max_iterations, d.max_iterations);
    EXPECT_EQ(core.solver.max_iter, d.solver.max_iter);
    EXPECT_EQ(core.delta_tr, d.delta_tr);
  }
  // And an empty map: the same.
  MpcDockingSegmentCoreParams core;
  ReadDockingCoreParams(YAML::Load("{}"), "x.core", 6, core);
  EXPECT_EQ(core.u_scale, d.u_scale);
  EXPECT_EQ(core.sigma_T, d.sigma_T);
}

TEST(DockingParamsCore, AKeyAbsentKeepsWhatTheCallerHeld) {
  MpcDockingSegmentCoreParams core;
  core.u_scale = 321.0;
  core.r_jerk = Eigen::VectorXd::Constant(6, 9.0);
  core.r_jerk_stop = Eigen::VectorXd::Constant(6, 8.0);
  core.delta_tr = 0.25;
  ReadDockingCoreParams(YAML::Load("{sqp: {max_iterations: 12}}"), "x.core", 6, core);
  EXPECT_EQ(core.max_iterations, 12);
  EXPECT_EQ(core.u_scale, 321.0);
  EXPECT_TRUE((core.r_jerk.array() == 9.0).all());
  EXPECT_TRUE((core.r_jerk_stop.array() == 8.0).all());
  EXPECT_EQ(core.delta_tr, 0.25);
}

TEST(DockingParamsCore, AMapThatIsNotAMapAndABadJointCountAreRefused) {
  MpcDockingSegmentCoreParams core;
  EXPECT_TRUE(Contains(
      ThrownMessage([&] { ReadDockingCoreParams(YAML::Load("3"), "x.core", 6, core); }), "x.core"));
  EXPECT_TRUE(
      Contains(ThrownMessage([&] { ReadDockingCoreParams(YAML::Load("[1]"), "x.core", 6, core); }),
               "x.core"));
  for (const int nv : {0, -1, rtc::catching::kMaxPlanNv + 1}) {
    EXPECT_THROW(ReadDockingCoreParams(YAML::Load("{u_scale: 1}"), "x.core", nv, core),
                 std::invalid_argument)
        << nv;
  }
  // ...but an absent map does not look at nv at all.
  EXPECT_NO_THROW(ReadDockingCoreParams(YAML::Node(), "x.core", 0, core));
}

TEST(DockingParamsCore, AHandValueUnderCoreIsRefusedWithAPointerToWhereItBelongs) {
  const std::string path = "planner.search.nlp.core";
  for (const char* key :
       {"s_ent", "r_ent", "tan_theta", "c_min", "c_cap_max", "c_ent_max", "v_perp_max", "a_brake",
        "delta_lo", "delta_hi", "sigma_tau", "restitution", "e_max", "p_max", "n_faces", "face_a",
        "face_b", "rho_ref", "contact_point_hand"}) {
    MpcDockingSegmentCoreParams core;
    const std::string m = ThrownMessage([&] {
      ReadDockingCoreParams(YAML::Load(std::string("{") + key + ": 0.05}"), path, 6, core);
    });
    EXPECT_TRUE(Contains(m, path + "." + key)) << key << " -> " << m;
    EXPECT_TRUE(Contains(m, "is not a key of '" + path + "'")) << key << " -> " << m;
    EXPECT_TRUE(Contains(m, "robot.hand.docking")) << key << " -> " << m;
  }
  for (const char* key : {"delta_0", "m_ball"}) {
    MpcDockingSegmentCoreParams core;
    const std::string m = ThrownMessage([&] {
      ReadDockingCoreParams(YAML::Load(std::string("{") + key + ": 0.05}"), path, 6, core);
    });
    EXPECT_TRUE(Contains(m, path + "." + key)) << key << " -> " << m;
    EXPECT_TRUE(Contains(m, "derived")) << key << " -> " << m;
    EXPECT_FALSE(Contains(m, "robot.hand.docking")) << key << " -> " << m;
  }
  // Through the two parsers, with the full path of the parser's own map.
  EXPECT_TRUE(Contains(ThrownMessage([] {
                         static_cast<void>(kParseMpc(YAML::Load(
                             "{planner: {segment: {mpc_docking: {core: {s_ent: 0.05}}}}}")));
                       }),
                       "planner.segment.mpc_docking.core.s_ent"));
  // The same mistake one level down is named too.
  EXPECT_TRUE(Contains(ThrownMessage([] {
                         static_cast<void>(kParseNlp(YAML::Load(
                             "{planner: {search: {nlp: {core: {catch: {e_max: 1}}}}}}")));
                       }),
                       "planner.search.nlp.core.catch.e_max"));
}

TEST(DockingParamsCore, KeysThatAreDeliberatelyNotKeysAreRefused) {
  for (const char* key : {"catch_time_variable", "n_pre", "dt_pre", "n_stop", "dt_stop", "n_blocks",
                          "block_sizes", "q_nom", "manip_d_q"}) {
    MpcDockingSegmentCoreParams core;
    const std::string m = ThrownMessage([&] {
      ReadDockingCoreParams(YAML::Load(std::string("{") + key + ": 1}"), "x.core", 6, core);
    });
    EXPECT_TRUE(Contains(m, std::string("x.core.") + key)) << key << " -> " << m;
    EXPECT_TRUE(Contains(m, "is not a key")) << key << " -> " << m;
  }
}

TEST(DockingParamsCore, TheNlpAndMpcMapsCarryTheSameCoreSubSchema) {
  const Pairs core_pairs = [] {
    Pairs out;
    for (const auto& e : CoreTable()) {
      out.emplace_back(e.path, e.good);
    }
    return out;
  }();
  Pairs nlp;
  Pairs mpc;
  for (const auto& p : core_pairs) {
    nlp.emplace_back(std::string(kNlpRoot) + "core." + p.first, p.second);
    mpc.emplace_back(std::string(kMpcRoot) + "core." + p.first, p.second);
  }
  const NlpCatchSearchParams a = kParseNlp(Make(nlp));
  const MpcDockingSegmentPlannerParams b = kParseMpc(Make(mpc));
  EXPECT_EQ(a.core.r_tau, b.core.r_tau);
  EXPECT_EQ(a.core.r_jerk_stop, b.core.r_jerk_stop);
  EXPECT_EQ(a.core.q_p, b.core.q_p);
  EXPECT_EQ(a.core.nu_ref, b.core.nu_ref);
  EXPECT_EQ(a.core.face_eps, b.core.face_eps);
  EXPECT_EQ(a.core.mu_init, b.core.mu_init);
  EXPECT_EQ(a.core.solver.max_iter, b.core.solver.max_iter);
  EXPECT_EQ(a.core.r_tau.size(), kNv);
}

// ── DockingGridMismatch ─────────────────────────────────────────────────────────

// A planner whose grid extent is set (the default is unset: a decision) to the
// search's default range.
MpcDockingSegmentPlannerParams PlannerGrid() {
  MpcDockingSegmentPlannerParams p;
  p.n_pre_max = NlpCatchSearchParams{}.n_pre_max;
  return p;
}

TEST(DockingParamsGrid, EqualGridsAgree) {
  const NlpCatchSearchParams s;
  const MpcDockingSegmentPlannerParams p = PlannerGrid();
  EXPECT_EQ(DockingGridMismatch(s, p), nullptr);
  // The search's range may be narrower than the planner's.
  NlpCatchSearchParams narrow;
  narrow.n_pre_max = 4;
  EXPECT_EQ(DockingGridMismatch(narrow, p), nullptr);
  // Parsed from the same YAML values, the grids agree.
  const NlpCatchSearchParams ps = kParseNlp(Make({{std::string(kNlpRoot) + "dt_pre_s", "0.08"}}));
  const MpcDockingSegmentPlannerParams pp =
      kParseMpc(Make({{std::string(kMpcRoot) + "approach.dt_pre_s", "0.08"},
                      {std::string(kMpcRoot) + "approach.n_pre_max", "6"}}));
  EXPECT_EQ(DockingGridMismatch(ps, pp), nullptr);
  // A planner whose grid extent is unset holds no search's range.
  EXPECT_NE(DockingGridMismatch(s, MpcDockingSegmentPlannerParams{}), nullptr);
  EXPECT_EQ(kParseMpc(Make({})).n_pre_max, 0);
}

TEST(DockingParamsGrid, EachDifferenceNamesItsPairOfKeys) {
  const auto expect_pair = [](const char* got, const char* a, const char* b) {
    ASSERT_NE(got, nullptr);
    EXPECT_NE(std::strstr(got, a), nullptr) << got;
    EXPECT_NE(std::strstr(got, b), nullptr) << got;
  };
  {
    NlpCatchSearchParams s;
    s.dt_pre = 0.08;
    expect_pair(DockingGridMismatch(s, PlannerGrid()), "planner.search.nlp.dt_pre_s",
                "planner.segment.mpc_docking.approach.dt_pre_s");
  }
  {
    // A difference below one nanosecond is the same grid; one nanosecond is not.
    NlpCatchSearchParams s;
    s.dt_pre = 0.1 + 4e-10;
    EXPECT_EQ(DockingGridMismatch(s, PlannerGrid()), nullptr);
    s.dt_pre = 0.1 + 1.2e-9;
    ASSERT_NE(DockingGridMismatch(s, PlannerGrid()), nullptr);
  }
  {
    NlpCatchSearchParams s;
    s.n_stop = 8;
    expect_pair(DockingGridMismatch(s, PlannerGrid()), "planner.search.nlp.stop.n_nodes",
                "planner.segment.mpc_docking.stop.n_nodes");
  }
  {
    NlpCatchSearchParams s;
    s.dt_stop = 0.04;
    expect_pair(DockingGridMismatch(s, PlannerGrid()), "planner.search.nlp.stop.dt_s",
                "planner.segment.mpc_docking.stop.dt_s");
  }
  {
    // The count differs.
    NlpCatchSearchParams s;
    s.n_stop_blocks = 3;
    s.stop_block_sizes = {2, 2, 3};
    expect_pair(DockingGridMismatch(s, PlannerGrid()), "planner.search.nlp.stop.blocks",
                "planner.segment.mpc_docking.stop.blocks");
  }
  {
    // The count agrees, one size does not (same sum).
    NlpCatchSearchParams s;
    s.stop_block_sizes = {1, 1, 3, 2};
    expect_pair(DockingGridMismatch(s, PlannerGrid()), "planner.search.nlp.stop.blocks",
                "planner.segment.mpc_docking.stop.blocks");
  }
  {
    NlpCatchSearchParams s;
    s.n_pre_max = 7;
    expect_pair(DockingGridMismatch(s, PlannerGrid()), "planner.search.nlp.n_pre.max",
                "planner.segment.mpc_docking.approach.n_pre_max");
  }
}

TEST(DockingParamsGrid, TheFirstDifferenceInTheStatedOrderIsTheOneNamed) {
  NlpCatchSearchParams s;
  s.dt_pre = 0.08;
  s.n_stop = 8;
  s.dt_stop = 0.04;
  s.n_stop_blocks = 3;
  s.n_pre_max = 7;
  const char* m = DockingGridMismatch(s, PlannerGrid());
  ASSERT_NE(m, nullptr);
  EXPECT_NE(std::strstr(m, "dt_pre_s"), nullptr) << m;
  s.dt_pre = 0.1;
  m = DockingGridMismatch(s, PlannerGrid());
  ASSERT_NE(m, nullptr);
  EXPECT_NE(std::strstr(m, "stop.n_nodes"), nullptr) << m;
  s.n_stop = 7;
  m = DockingGridMismatch(s, PlannerGrid());
  ASSERT_NE(m, nullptr);
  EXPECT_NE(std::strstr(m, "stop.dt_s"), nullptr) << m;
  s.dt_stop = 0.05;
  m = DockingGridMismatch(s, PlannerGrid());
  ASSERT_NE(m, nullptr);
  EXPECT_NE(std::strstr(m, "stop.blocks"), nullptr) << m;
  s.n_stop_blocks = 4;
  m = DockingGridMismatch(s, PlannerGrid());
  ASSERT_NE(m, nullptr);
  EXPECT_NE(std::strstr(m, "n_pre"), nullptr) << m;
  s.n_pre_max = 6;
  EXPECT_EQ(DockingGridMismatch(s, PlannerGrid()), nullptr);
}

TEST(DockingParamsHand, ASetHandHasNoUnsetDecisionAndDefaultsAreAllUnset) {
  const HandDockingParams h = kParseHand(Make(HandBase()));
  EXPECT_EQ(h.FirstUnset(), nullptr);
  EXPECT_FALSE(h.provisional);
  // Declaration order is the order FirstUnset reports in.
  const std::vector<std::string> order = {"s_ent",
                                          "corridor.r_ent",
                                          "corridor.tan_theta",
                                          "lateral.n_faces",
                                          "lateral.faces_a",
                                          "lateral.faces_b",
                                          "lateral.rho_ref",
                                          "speed.c_min",
                                          "speed.c_cap_max",
                                          "speed.c_ent_max",
                                          "speed.v_perp_max",
                                          "speed.a_brake",
                                          "closure.delta_lo",
                                          "closure.delta_hi",
                                          "closure.sigma_tau",
                                          "impact.contact_point_hand",
                                          "impact.restitution"};
  // Removing the keys from the back: the one removed last is the first in the
  // order, and FirstUnset must name it. (With n_faces gone the lists are not
  // read, which does not change that n_faces is named first.)
  Pairs left = HandBase();
  for (auto it = order.rbegin(); it != order.rend(); ++it) {
    left = Without(left, std::string(kHandRoot) + *it);
    const HandDockingParams r = kParseHand(Make(left));
    ASSERT_NE(r.FirstUnset(), nullptr) << *it;
    EXPECT_STREQ(r.FirstUnset(), (std::string(kHandRoot) + *it).c_str());
  }
}

TEST(DockingParamsHand, ProvisionalDefaultsToTrueAndEMaxAndPMaxToOff) {
  const HandDockingParams h =
      kParseHand(Make(Without(Without(Without(HandBase(), std::string(kHandRoot) + "provisional"),
                                      std::string(kHandRoot) + "impact.e_max"),
                              std::string(kHandRoot) + "impact.p_max")));
  EXPECT_TRUE(h.provisional);
  EXPECT_TRUE(std::isinf(h.e_max) && h.e_max > 0.0);
  EXPECT_TRUE(std::isinf(h.p_max) && h.p_max > 0.0);
  EXPECT_EQ(h.FirstUnset(), nullptr);
}

}  // namespace
