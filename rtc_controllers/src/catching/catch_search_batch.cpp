// The offline batch of the planner's search. See catch_search_batch.hpp.
#include "rtc_controllers/catching/catch_search_batch.hpp"

#include "batch_csv.hpp"
#include "rtc_controllers/catching/catch_pose_ik_params.hpp"
#include "rtc_controllers/catching/catching_params.hpp"
#include "rtc_controllers/catching/docking_params.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/time_types.hpp"
#include "rtc_controllers/catching/transition_table.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <istream>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace rtc::catching {
namespace {

using batch_csv::Num;

// Every wake is of the one activation and the one track a batch has: a reset
// search (ResetTrial) is what separates two throws, so what a wake is handed
// depends on the wake alone.
constexpr std::uint64_t kActivation = 1;
constexpr std::uint64_t kTrack = 1;

// The judgement reasons a candidate can carry, in the order of the `rej_*`
// columns. A reason added to JudgeReject has to be given a column here.
static_assert(kJudgeRejectCount == 6);
constexpr std::array<std::pair<JudgeReject, const char*>, 5> kJudgeColumns{{
    {JudgeReject::kInput, "rej_input"},
    {JudgeReject::kIk, "rej_ik"},
    {JudgeReject::kManipulability, "rej_manipulability"},
    {JudgeReject::kNotEvaluated, "rej_not_evaluated"},
    {JudgeReject::kTooFar, "rej_too_far"},
}};

// The NLP search's candidate reasons: NlpReject between kNone and the reasons
// that are a wake's only.
constexpr auto kNlpFirst = static_cast<std::size_t>(NlpReject::kFollowWindow);
constexpr auto kNlpEnd = static_cast<std::size_t>(NlpReject::kNoCandidate);

[[nodiscard]] std::string Flag(bool v) {
  return v ? "1" : "0";
}

// ── The binding file ─────────────────────────────────────────────────────────

[[nodiscard]] YAML::Node RequireMap(const YAML::Node& parent, const std::string& key,
                                    const std::string& path) {
  const YAML::Node node = parent[key];
  if (!node || !node.IsMap()) {
    throw std::invalid_argument("binding: '" + path + "' must be a map");
  }
  return node;
}

void RefuseUnknownKeys(const YAML::Node& map, const std::string& path,
                       const std::vector<std::string>& known) {
  for (const auto& entry : map) {
    const auto key = entry.first.as<std::string>();
    if (std::find(known.begin(), known.end(), key) == known.end()) {
      throw std::invalid_argument("binding: unknown key '" +
                                  (path.empty() ? key : path + "." + key) + "'");
    }
  }
  for (const std::string& key : known) {
    if (!map[key]) {
      throw std::invalid_argument("binding: '" + (path.empty() ? key : path + "." + key) +
                                  "' is required");
    }
  }
}

template <typename T>
[[nodiscard]] T Value(const YAML::Node& map, const std::string& key, const std::string& path) {
  try {
    const YAML::Node node = map[key];
    if (!node.IsScalar()) {
      throw std::invalid_argument("not a scalar");
    }
    return node.as<T>();
  } catch (const std::exception&) {
    throw std::invalid_argument("binding: '" + path + "." + key + "' has the wrong type");
  }
}

/// A joint vector: `nv` entries, or none when `may_be_empty`. Non-finite
/// entries are refused — a limit is not a decision the profile leaves open.
[[nodiscard]] std::vector<double> JointVector(const YAML::Node& map, const std::string& key,
                                              const std::string& path, int nv, bool may_be_empty) {
  const std::string full = path + "." + key;
  std::vector<double> out;
  try {
    const YAML::Node node = map[key];
    if (!node.IsSequence()) {
      throw std::invalid_argument("not a sequence");
    }
    out = node.as<std::vector<double>>();
  } catch (const std::exception&) {
    throw std::invalid_argument("binding: '" + full + "' must be a list of numbers");
  }
  if (!(may_be_empty && out.empty()) && out.size() != static_cast<std::size_t>(nv)) {
    throw std::invalid_argument("binding: '" + full + "' must have " + std::to_string(nv) +
                                " entries" + (may_be_empty ? " (or none)" : "") + ", got " +
                                std::to_string(out.size()));
  }
  if (!std::all_of(out.begin(), out.end(), [](double v) { return std::isfinite(v); })) {
    throw std::invalid_argument("binding: '" + full + "' has a non-finite entry");
  }
  return out;
}

[[nodiscard]] Eigen::VectorXd AsEigen(const std::vector<double>& v) {
  return Eigen::Map<const Eigen::VectorXd>(v.data(), static_cast<Eigen::Index>(v.size()));
}

}  // namespace

std::int64_t StoppedClock() noexcept {
  return 0;
}

// ── The wakes ────────────────────────────────────────────────────────────────

std::vector<SearchBatchWake> ParseSearchWakeCsv(std::istream& in) {
  const std::vector<std::string> required = {"throw_id", "wake", "now_ns", "t_ns", "p_x",
                                             "p_y",      "p_z",  "v_x",    "v_y",  "v_z",
                                             "a_x",      "a_y",  "a_z"};
  std::vector<SearchBatchWake> out;
  std::set<std::int64_t> closed_throws;
  batch_csv::ReadHeadered(in, "wake CSV", required, [&](const batch_csv::Row& row) {
    const auto line = std::to_string(row.line());
    const auto throw_id = row.Integer<std::int64_t>("throw_id");
    const auto wake = row.Integer<std::int64_t>("wake");
    const auto now_ns = row.Integer<std::int64_t>("now_ns");
    TrajSample s;
    s.t_ns = row.Integer<std::int64_t>("t_ns");
    s.p = {row.Finite("p_x"), row.Finite("p_y"), row.Finite("p_z")};
    s.v = {row.Finite("v_x"), row.Finite("v_y"), row.Finite("v_z")};
    s.a = {row.Finite("a_x"), row.Finite("a_y"), row.Finite("a_z")};
    if (wake < 0) {
      throw std::invalid_argument("line " + line + ": 'wake' is negative");
    }
    const bool same_wake =
        !out.empty() && out.back().throw_id == throw_id && out.back().wake == wake;
    if (!same_wake) {
      if (!out.empty() && out.back().throw_id == throw_id) {
        if (wake <= out.back().wake) {
          throw std::invalid_argument("line " + line + ": wake " + std::to_string(wake) +
                                      " of throw " + std::to_string(throw_id) +
                                      " does not follow wake " + std::to_string(out.back().wake));
        }
      } else {
        if (!out.empty()) {
          closed_throws.insert(out.back().throw_id);
        }
        if (closed_throws.contains(throw_id)) {
          throw std::invalid_argument("line " + line + ": throw " + std::to_string(throw_id) +
                                      " comes back after another throw");
        }
      }
      SearchBatchWake w;
      w.throw_id = throw_id;
      w.wake = wake;
      w.now_ns = now_ns;
      out.push_back(std::move(w));
    }
    SearchBatchWake& w = out.back();
    if (w.now_ns != now_ns) {
      throw std::invalid_argument("line " + line + ": 'now_ns' changes inside wake " +
                                  std::to_string(wake) + " of throw " + std::to_string(throw_id));
    }
    if (!w.samples.empty() && s.t_ns <= w.samples.back().t_ns) {
      throw std::invalid_argument("line " + line + ": 't_ns' does not increase inside its wake");
    }
    if (w.samples.size() >= static_cast<std::size_t>(kCap)) {
      throw std::invalid_argument("line " + line + ": wake " + std::to_string(wake) + " of throw " +
                                  std::to_string(throw_id) + " has more than " +
                                  std::to_string(kCap) + " samples");
    }
    w.samples.push_back(s);
  });
  return out;
}

SearchBatchInputs MakeSearchBatchInputs(const SearchBatchWake& wake,
                                        std::span<const double> q_rest) {
  if (wake.samples.empty() || wake.samples.size() > static_cast<std::size_t>(kCap)) {
    throw std::invalid_argument("wake " + std::to_string(wake.wake) + " of throw " +
                                std::to_string(wake.throw_id) + " has " +
                                std::to_string(wake.samples.size()) + " samples (1 to " +
                                std::to_string(kCap) + ")");
  }
  if (q_rest.empty() || q_rest.size() > static_cast<std::size_t>(kMaxPlanNv)) {
    throw std::invalid_argument("the rest pose has " + std::to_string(q_rest.size()) +
                                " entries (1 to " + std::to_string(kMaxPlanNv) + ")");
  }
  SearchBatchInputs in;
  in.traj.token.activation_generation = kActivation;
  in.traj.token.generation = kTrack;
  in.traj.token.snapshot_sequence = static_cast<std::uint64_t>(wake.wake) + 1;
  in.traj.token.traj_recv_ns = wake.now_ns;
  in.traj.n = static_cast<std::int32_t>(wake.samples.size());
  in.traj.valid = true;
  std::copy(wake.samples.begin(), wake.samples.end(), in.traj.s.begin());

  // The zero matrix at every sample (value-initialised), of this prediction.
  in.cov.token = in.traj.token;
  in.cov.n = in.traj.n;
  in.cov.valid = true;

  in.rt.valid = true;
  in.rt.activation_generation = kActivation;
  in.rt.rt_iteration = static_cast<std::uint64_t>(wake.wake) + 1;
  in.rt.rt_state_ns = wake.now_ns;
  in.rt.mode = static_cast<std::uint8_t>(Mode::kTracking);
  in.rt.armed = true;
  in.rt.nv = static_cast<std::int32_t>(q_rest.size());
  in.rt.cmd_seeded = true;
  std::copy(q_rest.begin(), q_rest.end(), in.rt.q_cmd.begin());
  return in;
}

// ── The result ───────────────────────────────────────────────────────────────

std::string SearchBatchCsvHeader(int nv) {
  std::string out =
      "throw_id,wake,now_ns,search_valid,plan_reason,settling,n_in_window,n_ik,n_pass";
  for (const auto& column : kJudgeColumns) {
    out += std::string(",") + column.second;
  }
  out += ",budget_hit,nlp_ran,nlp_reason,nlp_n_lattice,nlp_n_screened,nlp_n_solved,nlp_n_valid";
  for (std::size_t r = kNlpFirst; r < kNlpEnd; ++r) {
    out += std::string(",nlp_rej_") + NlpRejectName(static_cast<NlpReject>(r));
  }
  out +=
      ",t_c_ns,lead_s,p_c_x,p_c_y,p_c_z,v_c_x,v_c_y,v_c_z,score,w5,w6,rank_mask,gamma_f,"
      "nlp_lead_s";
  for (int i = 0; i < nv; ++i) {
    out += ",q_star" + std::to_string(i);
  }
  out += ",wall_us";
  return out;
}

std::string SearchBatchCsvRow(const SearchBatchRow& row, int nv) {
  const SearchStats& s = row.stats;
  const NlpSearchStats& n = s.nlp;
  const PlanSnapshot& p = row.plan;
  std::string out = std::to_string(row.throw_id) + ',' + std::to_string(row.wake) + ',' +
                    std::to_string(row.now_ns) + ',' + Flag(p.valid) + ',' +
                    std::to_string(static_cast<int>(p.reason)) + ',' + Flag(s.settling) + ',' +
                    std::to_string(s.n_in_window) + ',' + std::to_string(s.n_ik) + ',' +
                    std::to_string(s.n_pass);
  for (const auto& column : kJudgeColumns) {
    out += ',' + std::to_string(s.judge_rejects[static_cast<std::size_t>(column.first)]);
  }
  out += ',' + Flag(s.budget_hit) + ',' + Flag(n.ran) + ',' +
         (n.ran ? NlpRejectName(n.reason) : "off") + ',' + std::to_string(n.n_lattice) + ',' +
         std::to_string(n.n_screened) + ',' + std::to_string(n.n_solved) + ',' +
         std::to_string(n.n_valid);
  for (std::size_t r = kNlpFirst; r < kNlpEnd; ++r) {
    out += ',' + std::to_string(n.rejects[r]);
  }
  // The plan: empty cells when the wake chose none.
  const auto cell = [&out, &p](const std::string& text) { out += ',' + (p.valid ? text : ""); };
  cell(std::to_string(p.t_c_ns));
  cell(Num(static_cast<double>(p.t_c_ns - row.now_ns) * kNsToS));
  for (const double v : p.p_c) {
    cell(Num(v));
  }
  for (const double v : p.v_c) {
    cell(Num(v));
  }
  cell(Num(p.score));
  cell(Num(p.w5));
  cell(Num(p.w6));
  cell(std::to_string(s.chosen_rank_mask));
  cell(Num(s.chosen_gamma_f));
  out += ',' + (p.valid && n.ran ? Num(n.chosen_lead_s) : std::string());
  for (int i = 0; i < nv; ++i) {
    cell(Num(p.q_star[static_cast<std::size_t>(i)]));
  }
  out += ',' + Num(row.wall_us);
  return out;
}

std::vector<SearchBatchRow> RunSearchBatch(CatchSearch& search, std::span<const double> q_rest,
                                           const std::vector<SearchBatchWake>& wakes) {
  // No segment: the RT of a batch wake follows no plan.
  const ReportedSegments no_segments{};
  std::vector<SearchBatchRow> out;
  out.reserve(wakes.size());
  bool first = true;
  bool planned = false;
  std::int64_t throw_id = 0;
  for (const SearchBatchWake& wake : wakes) {
    if (first || wake.throw_id != throw_id) {
      first = false;
      throw_id = wake.throw_id;
      planned = false;
      search.ResetTrial();
    }
    if (planned) {
      continue;
    }
    const SearchBatchInputs in = MakeSearchBatchInputs(wake, q_rest);
    SearchBatchRow row;
    row.throw_id = wake.throw_id;
    row.wake = wake.wake;
    row.now_ns = wake.now_ns;
    // Offline tool: the wall clock around the call, never handed to the search.
    const auto started = std::chrono::steady_clock::now();
    row.plan = search.Plan(in.traj, in.cov, /*cov_matched=*/true, in.rt, no_segments,
                           NowReal{wake.now_ns}, /*budget_cap_ns=*/0, row.stats);
    row.wall_us =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - started)
            .count();
    planned = row.plan.valid;
    out.push_back(row);
  }
  return out;
}

// ── The binding ──────────────────────────────────────────────────────────────

SearchBatchBinding ParseSearchBatchBinding(const YAML::Node& root, int nv) {
  if (nv <= 0 || nv > kMaxPlanNv) {
    throw std::invalid_argument("binding: the arm has " + std::to_string(nv) + " joints (1 to " +
                                std::to_string(kMaxPlanNv) + ")");
  }
  if (!root || !root.IsMap()) {
    throw std::invalid_argument("binding: the file must be a map");
  }
  SearchBatchBinding b;
  const auto kind = root["search"] && root["search"].IsScalar() ? root["search"].as<std::string>()
                                                                : std::string();
  if (kind == SearchModeName(CatchingSearchMode::kGrid)) {
    b.kind = SearchBatchKind::kGrid;
  } else if (kind == SearchModeName(CatchingSearchMode::kNlp)) {
    b.kind = SearchBatchKind::kNlp;
  } else {
    throw std::invalid_argument("binding: 'search' must be 'grid' or 'nlp', got '" + kind + "'");
  }
  RefuseUnknownKeys(root, "", {"search", "device_of_model", kind});

  try {
    b.device_of_model = root["device_of_model"].as<std::vector<int>>();
  } catch (const std::exception&) {
    throw std::invalid_argument("binding: 'device_of_model' must be a list of integers");
  }
  std::vector<int> sorted = b.device_of_model;
  std::sort(sorted.begin(), sorted.end());
  bool permutation = sorted.size() == static_cast<std::size_t>(nv);
  for (int i = 0; permutation && i < nv; ++i) {
    permutation = sorted[static_cast<std::size_t>(i)] == i;
  }
  if (!permutation) {
    throw std::invalid_argument("binding: 'device_of_model' must be a permutation of 0.." +
                                std::to_string(nv - 1));
  }

  const YAML::Node map = RequireMap(root, kind, kind);
  if (b.kind == SearchBatchKind::kGrid) {
    RefuseUnknownKeys(map, kind,
                      {"qdot_max", "qddot_max", "eta_v", "v_max", "a_dec", "t_arm_s",
                       "t_close_lead", "t_close_total", "ball_mass", "ref_omega", "ref_zeta",
                       "ref_a_max", "control_dt", "follows_segments"});
    b.qdot_max = JointVector(map, "qdot_max", kind, nv, false);
    b.qddot_max = JointVector(map, "qddot_max", kind, nv, true);
    b.grid.eta_v = Value<double>(map, "eta_v", kind);
    b.grid.v_max = Value<double>(map, "v_max", kind);
    b.grid.a_dec = Value<double>(map, "a_dec", kind);
    b.grid.t_arm_s = Value<double>(map, "t_arm_s", kind);
    b.grid.t_close_lead = Value<double>(map, "t_close_lead", kind);
    b.grid.t_close_total = Value<double>(map, "t_close_total", kind);
    b.grid.ball_mass = Value<double>(map, "ball_mass", kind);
    b.grid.ref_omega = Value<double>(map, "ref_omega", kind);
    b.grid.ref_zeta = Value<double>(map, "ref_zeta", kind);
    b.grid.ref_a_max = Value<double>(map, "ref_a_max", kind);
    b.grid.control_dt = Value<double>(map, "control_dt", kind);
    b.grid.follows_segments = Value<bool>(map, "follows_segments", kind);
    return b;
  }
  RefuseUnknownKeys(map, kind,
                    {"t_arm_s", "control_dt", "t_close_lead", "hand_t_close_e2e",
                     "hand_t_close_lead", "ball_mass", "limits"});
  b.nlp.t_arm_s = Value<double>(map, "t_arm_s", kind);
  b.nlp.control_dt = Value<double>(map, "control_dt", kind);
  b.nlp.t_close_lead = Value<double>(map, "t_close_lead", kind);
  b.hand_t_close_e2e = Value<double>(map, "hand_t_close_e2e", kind);
  b.hand_t_close_lead = Value<double>(map, "hand_t_close_lead", kind);
  b.ball_mass = Value<double>(map, "ball_mass", kind);
  const std::string limits_path = kind + ".limits";
  const YAML::Node limits = RequireMap(map, "limits", limits_path);
  RefuseUnknownKeys(limits, limits_path,
                    {"q_min", "q_max", "qd_max", "qdd_max", "tau_max", "tau_lo", "tau_hi"});
  b.limits.q_min = AsEigen(JointVector(limits, "q_min", limits_path, nv, false));
  b.limits.q_max = AsEigen(JointVector(limits, "q_max", limits_path, nv, false));
  b.limits.qd_max = AsEigen(JointVector(limits, "qd_max", limits_path, nv, false));
  b.limits.qdd_max = AsEigen(JointVector(limits, "qdd_max", limits_path, nv, true));
  b.limits.tau_max = AsEigen(JointVector(limits, "tau_max", limits_path, nv, false));
  b.limits.tau_lo = AsEigen(JointVector(limits, "tau_lo", limits_path, nv, false));
  b.limits.tau_hi = AsEigen(JointVector(limits, "tau_hi", limits_path, nv, false));
  b.limits.armature = Eigen::VectorXd::Zero(nv);
  return b;
}

SearchBatchSearch MakeSearchBatchSearch(const std::shared_ptr<const pinocchio::Model>& arm,
                                        rtc_urdf_bridge::RtModelHandle& handle,
                                        pinocchio::FrameIndex catch_frame,
                                        const YAML::Node& catching,
                                        const SearchBatchBinding& binding) {
  if (!arm || arm->nv <= 0 || arm->nv > kMaxPlanNv) {
    throw std::invalid_argument("the arm model has no joints or more than " +
                                std::to_string(kMaxPlanNv));
  }
  const int nv = arm->nv;
  const auto n = static_cast<std::size_t>(nv);
  if (binding.device_of_model.size() != n) {
    throw std::invalid_argument("binding: 'device_of_model' must have " + std::to_string(nv) +
                                " entries");
  }
  const bool grid = binding.kind == SearchBatchKind::kGrid;
  // Only the selected search's map is opened, as a controller's configure
  // does: a function that does not run has no say in whether this one builds.
  PlannerKeySelection keys;
  keys.search_grid = grid;
  keys.segment_mpc = false;
  // The parsers default an absent map; a map of the selected search that is
  // not in the tree would then be judged on compiled-in values and the map
  // would describe no profile at all.
  const char* const kind =
      SearchModeName(grid ? CatchingSearchMode::kGrid : CatchingSearchMode::kNlp);
  const YAML::Node search_map =
      catching.IsMap() && catching["planner"] && catching["planner"].IsMap() &&
              catching["planner"]["search"] && catching["planner"]["search"].IsMap()
          ? catching["planner"]["search"][kind]
          : YAML::Node();
  if (!search_map || !search_map.IsMap()) {
    throw std::invalid_argument(std::string("the tree has no planner.search.") + kind +
                                " map — the selected search would run on compiled-in values");
  }
  const PlannerParams planner = ParsePlannerParams(catching, keys);
  if (planner.wait_pose_n != nv) {
    throw std::invalid_argument("planner.wait_pose has " + std::to_string(planner.wait_pose_n) +
                                " entries, the arm " + std::to_string(nv) + " joints");
  }
  const CatchPoseIkConfig ik = ParseCatchPoseIkParams(catching, nullptr, kind);

  SearchBatchSearch out;
  out.q_rest.assign(planner.wait_pose.begin(),
                    planner.wait_pose.begin() + static_cast<std::ptrdiff_t>(n));
  std::array<int, kMaxPlanNv> device_of_model{};
  std::copy(binding.device_of_model.begin(), binding.device_of_model.end(),
            device_of_model.begin());

  if (grid) {
    if (binding.qdot_max.size() != n ||
        (!binding.qddot_max.empty() && binding.qddot_max.size() != n)) {
      throw std::invalid_argument("binding: grid.qdot_max / grid.qddot_max do not match the arm");
    }
    GridCatchSearchModel model;
    model.handle = &handle;
    model.catch_frame = catch_frame;
    model.nv = nv;
    model.device_of_model = device_of_model;
    std::copy(binding.qdot_max.begin(), binding.qdot_max.end(), model.qdot_max.begin());
    std::copy(binding.qddot_max.begin(), binding.qddot_max.end(), model.qddot_max.begin());
    model.accel_box = !binding.qddot_max.empty();
    out.search = MakeGridCatchSearch(model, binding.grid, planner, ik.options, &StoppedClock);
    if (out.search == nullptr) {
      out.error = "the grid search refused its model and parameters";
    }
    return out;
  }

  if (nv > kMaxSegmentNv) {
    out.error = "the nlp search plans arms of at most " + std::to_string(kMaxSegmentNv) + " joints";
    return out;
  }
  const HandDockingParams hand = ParseHandDockingParams(catching);
  if (const char* unset = hand.FirstUnset(); unset != nullptr) {
    out.error = std::string("robot.hand.docking: '") + unset + "' is not set";
    return out;
  }
  NlpCatchSearchModel model;
  model.arm = arm;
  model.handle = &handle;
  model.catch_frame = catch_frame;
  model.nv = nv;
  model.device_of_model = device_of_model;
  NlpCatchSearchParams params = ParseNlpSearchParams(catching, nv);
  params.wait_pose = planner.wait_pose;
  params.wait_pose_n = planner.wait_pose_n;
  params.limits = binding.limits;
  ApplyHandDocking(hand, binding.hand_t_close_e2e, binding.hand_t_close_lead, binding.ball_mass,
                   params.core);
  // The nominal posture of the posture cost: the wait pose, model order.
  params.core.q_nom.resize(nv);
  for (int m = 0; m < nv; ++m) {
    params.core.q_nom[m] =
        planner.wait_pose[static_cast<std::size_t>(device_of_model[static_cast<std::size_t>(m)])];
  }
  auto search = std::make_unique<NlpCatchSearch>();
  std::string error;
  if (!search->Configure(model, binding.nlp, params, ik.options, &StoppedClock, &error)) {
    out.error = "planner.search.nlp: " + error;
    return out;
  }
  out.search = std::move(search);
  return out;
}

// ── The reach bound ──────────────────────────────────────────────────────────

ReachTolerances SearchReachTolerances(const YAML::Node& catching) {
  const auto of = [&](CatchingSearchMode mode) {
    return ParseCatchPoseIkParams(catching, nullptr, SearchModeName(mode)).options.eps_pos;
  };
  ReachTolerances out;
  out.grid = of(CatchingSearchMode::kGrid);
  out.nlp = of(CatchingSearchMode::kNlp);
  out.selected = of(ParseCatchingParams(catching).planner_search_mode);
  return out;
}

std::string ReachBoundJson(const ReachBound& bound, std::string_view frame,
                           std::string_view sub_model, const ReachTolerances& tolerances) {
  const auto num = [](double v) { return std::isfinite(v) ? Num(v) : std::string("null"); };
  // Names are quoted verbatim: frame and joint names are URDF identifiers.
  const auto quote = [](std::string_view text) {
    std::string out = "\"";
    for (const char c : text) {
      if (c == '"' || c == '\\') {
        out += '\\';
      }
      out += c;
    }
    return out + '"';
  };
  return std::string("{\"schema\": \"catch_reach_bound/1\", \"frame\": ") + quote(frame) +
         ", \"sub_model\": " + quote(sub_model) + ", \"centre\": [" + num(bound.centre.x()) + ", " +
         num(bound.centre.y()) + ", " + num(bound.centre.z()) +
         "], \"radius\": " + num(bound.radius) + ", \"tolerance\": " + num(tolerances.selected) +
         ", \"tolerance_by_search\": {\"grid\": " + num(tolerances.grid) +
         ", \"nlp\": " + num(tolerances.nlp) + "}" +
         ", \"joints\": " + std::to_string(bound.joints) +
         ", \"unbounded_by\": " + quote(bound.unbounded_by) + "}";
}

}  // namespace rtc::catching
