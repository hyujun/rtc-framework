// The offline batch of the planner's search. See catch_search_batch.hpp.
#include "rtc_controllers/catching/catch_search_batch.hpp"

#include "batch_csv.hpp"
#include "rtc_controllers/catching/catch_pose_ik_params.hpp"
#include "rtc_controllers/catching/catching_params.hpp"
#include "rtc_controllers/catching/docking_config.hpp"
#include "rtc_controllers/catching/docking_params.hpp"
#include "rtc_controllers/catching/mpc_docking_segment_planner.hpp"
#include "rtc_controllers/catching/mpc_segment_planner.hpp"
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

/// Every key of `known` is required; a key of `optional` may be there.
void RefuseUnknownKeys(const YAML::Node& map, const std::string& path,
                       const std::vector<std::string>& known,
                       const std::vector<std::string>& optional = {}) {
  for (const auto& entry : map) {
    const auto key = entry.first.as<std::string>();
    if (std::find(known.begin(), known.end(), key) == known.end() &&
        std::find(optional.begin(), optional.end(), key) == optional.end()) {
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

/// The cores' box of a docking function: `<path>.limits`.
[[nodiscard]] MpcDockingSegmentCoreLimits DockingLimits(const YAML::Node& map,
                                                        const std::string& path, int nv) {
  const std::string limits_path = path + ".limits";
  const YAML::Node limits = RequireMap(map, "limits", limits_path);
  RefuseUnknownKeys(limits, limits_path,
                    {"q_min", "q_max", "qd_max", "qdd_max", "tau_max", "tau_lo", "tau_hi"});
  MpcDockingSegmentCoreLimits out;
  out.q_min = AsEigen(JointVector(limits, "q_min", limits_path, nv, false));
  out.q_max = AsEigen(JointVector(limits, "q_max", limits_path, nv, false));
  out.qd_max = AsEigen(JointVector(limits, "qd_max", limits_path, nv, false));
  out.qdd_max = AsEigen(JointVector(limits, "qdd_max", limits_path, nv, true));
  out.tau_max = AsEigen(JointVector(limits, "tau_max", limits_path, nv, false));
  out.tau_lo = AsEigen(JointVector(limits, "tau_lo", limits_path, nv, false));
  out.tau_hi = AsEigen(JointVector(limits, "tau_hi", limits_path, nv, false));
  out.armature = Eigen::VectorXd::Zero(nv);
  return out;
}

/// The `segment` map of a binding file.
[[nodiscard]] SegmentBatchBinding ParseSegmentBinding(const YAML::Node& root, int nv) {
  const std::string path = "segment";
  const YAML::Node segment = RequireMap(root, path, path);
  const auto mode = segment["mode"] && segment["mode"].IsScalar()
                        ? segment["mode"].as<std::string>()
                        : std::string();
  SegmentBatchBinding b;
  if (mode == SegmentModeName(CatchingSegmentMode::kMpc)) {
    b.kind = SegmentBatchKind::kMpc;
  } else if (mode == SegmentModeName(CatchingSegmentMode::kMpcDocking)) {
    b.kind = SegmentBatchKind::kMpcDocking;
  } else {
    throw std::invalid_argument("binding: 'segment.mode' must be 'mpc' or 'mpc_docking', got '" +
                                mode + "'");
  }
  RefuseUnknownKeys(segment, path, {"mode", mode});
  const std::string own = path + "." + mode;
  const YAML::Node map = RequireMap(segment, mode, own);
  if (b.kind == SegmentBatchKind::kMpc) {
    RefuseUnknownKeys(map, own,
                      {"q_min", "q_max", "qdot_max", "tau_max", "eta_v", "t_arm_s", "control_dt"});
    b.q_min = JointVector(map, "q_min", own, nv, false);
    b.q_max = JointVector(map, "q_max", own, nv, false);
    b.qdot_max = JointVector(map, "qdot_max", own, nv, false);
    b.tau_max = JointVector(map, "tau_max", own, nv, false);
    b.eta_v = Value<double>(map, "eta_v", own);
  } else {
    RefuseUnknownKeys(map, own,
                      {"t_arm_s", "control_dt", "qd_rating", "hand_t_close_e2e",
                       "hand_t_close_lead", "ball_mass", "limits"});
    b.qd_rating = JointVector(map, "qd_rating", own, nv, false);
    b.hand_t_close_e2e = Value<double>(map, "hand_t_close_e2e", own);
    b.hand_t_close_lead = Value<double>(map, "hand_t_close_lead", own);
    b.ball_mass = Value<double>(map, "ball_mass", own);
    b.limits = DockingLimits(map, own, nv);
  }
  b.t_arm_s = Value<double>(map, "t_arm_s", own);
  b.control_dt = Value<double>(map, "control_dt", own);
  return b;
}

/// Whether `catching` holds the map `planner.<group>.<name>`. The parsers
/// default an absent map, and a function built on compiled-in values would be
/// judged as no profile describes it.
[[nodiscard]] bool HasPlannerMap(const YAML::Node& catching, const char* group, const char* name) {
  if (!catching.IsMap() || !catching["planner"] || !catching["planner"].IsMap()) {
    return false;
  }
  const YAML::Node section = catching["planner"][group];
  if (!section || !section.IsMap()) {
    return false;
  }
  const YAML::Node map = section[name];
  return map && map.IsMap();
}

// The instant WakeClock reports: the wake PlanBatch::Run is on, on this thread.
thread_local std::int64_t wake_clock_ns = 0;

/// WakeClock is 0 again when a run ends — by a return or by a throw.
struct WakeClockScope {
  WakeClockScope() = default;
  WakeClockScope(const WakeClockScope&) = delete;
  WakeClockScope& operator=(const WakeClockScope&) = delete;

  ~WakeClockScope() { wake_clock_ns = 0; }
};

/// The nominal posture of the docking core's posture cost: the wait pose
/// (DEVICE order) in MODEL order.
void SetNominalPosture(const PlannerParams& planner,
                       const std::array<int, kMaxPlanNv>& device_of_model, int nv,
                       MpcDockingSegmentCoreParams& core) {
  core.q_nom.resize(nv);
  for (int m = 0; m < nv; ++m) {
    core.q_nom[m] =
        planner.wait_pose[static_cast<std::size_t>(device_of_model[static_cast<std::size_t>(m)])];
  }
}

[[nodiscard]] std::array<int, kMaxPlanNv> DeviceOfModel(const SearchBatchBinding& binding) {
  std::array<int, kMaxPlanNv> out{};
  std::copy(binding.device_of_model.begin(), binding.device_of_model.end(), out.begin());
  return out;
}

/// The loop both batches are: throws in the given order, `new_throw` before a
/// throw's first wake, `run_wake` on each of its wakes until one returns true
/// (the throw is done; its later wakes are not run).
template <typename NewThrow, typename RunWake>
[[nodiscard]] std::vector<SearchBatchRow> RunThrows(const std::vector<SearchBatchWake>& wakes,
                                                    NewThrow&& new_throw, RunWake&& run_wake) {
  std::vector<SearchBatchRow> out;
  out.reserve(wakes.size());
  bool first = true;
  bool done = false;
  std::int64_t throw_id = 0;
  for (const SearchBatchWake& wake : wakes) {
    if (first || wake.throw_id != throw_id) {
      first = false;
      throw_id = wake.throw_id;
      done = false;
      new_throw();
    }
    if (done) {
      continue;
    }
    SearchBatchRow row;
    row.throw_id = wake.throw_id;
    row.wake = wake.wake;
    row.now_ns = wake.now_ns;
    done = run_wake(wake, row);
    out.push_back(row);
  }
  return out;
}

/// The wall clock around `call` [us]. Offline tool: never handed to a planner.
template <typename Call>
[[nodiscard]] double WallUs(Call&& call) {
  const auto started = std::chrono::steady_clock::now();
  call();
  return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - started)
      .count();
}

}  // namespace

std::int64_t StoppedClock() noexcept {
  return 0;
}

std::int64_t WakeClock() noexcept {
  return wake_clock_ns;
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

namespace {

// What a whole-wake row adds, before `wall_us` (PlanBatchCsvHeader).
constexpr const char* kCycleColumns =
    ",outcome,segment_outcome,segment_kind,segment_core_reason,segment_k,segment_n_nodes,"
    "segment_iterations,segment_from_search,segment_ref_scaled,segment_ref_scale,"
    "segment_slack_max,segment_slack_terminal_max,segment_catch_pos_err,segment_catch_axis_err,"
    "segment_speed_ratio_max,segment_tau_ratio_max";

[[nodiscard]] std::string CsvHeader(int nv, bool cycle) {
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
  if (cycle) {
    out += kCycleColumns;
  }
  out += ",wall_us";
  return out;
}

[[nodiscard]] std::string CsvRow(const SearchBatchRow& row, int nv, bool cycle) {
  const SearchStats& s = row.stats;
  const NlpSearchStats& n = s.nlp;
  const PlanSnapshot& p = row.plan;
  // A whole wake's plan is the one the cycle stored: its search may have
  // produced one that was withheld.
  const bool search_valid = row.cycle ? row.search_valid : p.valid;
  std::string out = std::to_string(row.throw_id) + ',' + std::to_string(row.wake) + ',' +
                    std::to_string(row.now_ns) + ',' + Flag(search_valid) + ',' +
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
  if (cycle) {
    const SegmentRecord& g = row.segment;
    out += std::string(",") + CycleOutcomeName(row.outcome) + ',' + SegmentOutcomeName(g.outcome) +
           ',' + SegmentKindName(g.kind) + ',' + g.core_reason_name + ',' + std::to_string(g.k) +
           ',' + std::to_string(g.n_nodes) + ',' + std::to_string(g.iterations) + ',' +
           Flag(g.from_search) + ',' + Flag(g.ref_scaled) + ',' + Num(g.ref_scale) + ',' +
           Num(g.slack_max) + ',' + Num(g.slack_terminal_max) + ',' + Num(g.catch_pos_err) + ',' +
           Num(g.catch_axis_err) + ',' + Num(g.speed_ratio_max) + ',' + Num(g.tau_ratio_max);
  }
  out += ',' + Num(row.wall_us);
  return out;
}

}  // namespace

std::string SearchBatchCsvHeader(int nv) {
  return CsvHeader(nv, /*cycle=*/false);
}

std::string SearchBatchCsvRow(const SearchBatchRow& row, int nv) {
  return CsvRow(row, nv, /*cycle=*/false);
}

std::string PlanBatchCsvHeader(int nv) {
  return CsvHeader(nv, /*cycle=*/true);
}

std::string PlanBatchCsvRow(const SearchBatchRow& row, int nv) {
  return CsvRow(row, nv, /*cycle=*/true);
}

std::vector<SearchBatchRow> RunSearchBatch(CatchSearch& search, std::span<const double> q_rest,
                                           const std::vector<SearchBatchWake>& wakes) {
  // No segment: the RT of a batch wake follows no plan.
  const ReportedSegments no_segments{};
  return RunThrows(
      wakes, [&search] { search.ResetTrial(); },
      [&](const SearchBatchWake& wake, SearchBatchRow& row) {
        const SearchBatchInputs in = MakeSearchBatchInputs(wake, q_rest);
        row.wall_us = WallUs([&] {
          row.plan = search.Plan(in.traj, in.cov, /*cov_matched=*/true, in.rt, no_segments,
                                 NowReal{wake.now_ns}, /*budget_cap_ns=*/0, row.stats);
        });
        return row.plan.valid;
      });
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
  RefuseUnknownKeys(root, "", {"search", "device_of_model", kind}, {"segment"});
  if (root["segment"]) {
    b.segment = ParseSegmentBinding(root, nv);
  }

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
  b.limits = DockingLimits(map, kind, nv);
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
  if (!HasPlannerMap(catching, "search", kind)) {
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
  const std::array<int, kMaxPlanNv> device_of_model = DeviceOfModel(binding);

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
  SetNominalPosture(planner, device_of_model, nv, params.core);
  auto search = std::make_unique<NlpCatchSearch>();
  std::string error;
  if (!search->Configure(model, binding.nlp, params, ik.options, &StoppedClock, &error)) {
    out.error = "planner.search.nlp: " + error;
    return out;
  }
  out.search = std::move(search);
  return out;
}

// ── The whole wake ───────────────────────────────────────────────────────────

std::vector<SearchBatchRow> PlanBatch::Run(const std::vector<SearchBatchWake>& wakes) {
  const WakeClockScope clock;
  return RunThrows(
      wakes,
      // A new reset epoch is what tells the cycle the trial is another one: it
      // resets the search and the segment planner and withdraws the segment.
      [this] { ++reset_epoch_; },
      [this](const SearchBatchWake& wake, SearchBatchRow& row) {
        SearchBatchInputs in = MakeSearchBatchInputs(wake, q_rest_);
        in.rt.reset_epoch = reset_epoch_;
        cov_.Store(in.cov);
        traj_.Store(in.traj);
        rt_.Store(in.rt);
        wake_clock_ns = wake.now_ns;
        row.cycle = true;
        PlannerCycleRecord rec{};
        row.wall_us = WallUs([&] { rec = cycle_.Run(NowReal{wake.now_ns}); });
        row.stats = rec.search;
        row.search_valid = rec.search_valid;
        row.outcome = rec.outcome;
        row.segment = rec.segment;
        if (rec.outcome == CycleOutcome::kPublished) {
          row.plan = plan_.Load();
        }
        return rec.outcome == CycleOutcome::kPublished && rec.plan_valid;
      });
}

PlanBatchBuild MakePlanBatch(const std::shared_ptr<const pinocchio::Model>& arm,
                             rtc_urdf_bridge::RtModelHandle& handle,
                             pinocchio::FrameIndex catch_frame, const YAML::Node& catching,
                             const SearchBatchBinding& binding) {
  if (!binding.segment.has_value()) {
    throw std::invalid_argument("binding: a whole-wake batch needs 'segment'");
  }
  const SegmentBatchBinding& sb = *binding.segment;
  PlanBatchBuild out;
  SearchBatchSearch built = MakeSearchBatchSearch(arm, handle, catch_frame, catching, binding);
  if (built.search == nullptr) {
    out.error = built.error;
    return out;
  }
  const int nv = arm->nv;
  const auto n = static_cast<std::size_t>(nv);
  if (nv > kMaxSegmentNv) {
    out.error = "a segment carries at most " + std::to_string(kMaxSegmentNv) + " joints";
    return out;
  }
  const bool mpc = sb.kind == SegmentBatchKind::kMpc;
  const char* const mode =
      SegmentModeName(mpc ? CatchingSegmentMode::kMpc : CatchingSegmentMode::kMpcDocking);
  if (!HasPlannerMap(catching, "segment", mode)) {
    throw std::invalid_argument(std::string("the tree has no planner.segment.") + mode +
                                " map — the selected segment planner would run on compiled-in "
                                "values");
  }
  // The thread's parameters as a controller parses them: the selected
  // functions' maps only.
  PlannerKeySelection keys;
  keys.search_grid = binding.kind == SearchBatchKind::kGrid;
  keys.segment_mpc = mpc;
  const PlannerParams planner = ParsePlannerParams(catching, keys);
  const std::array<int, kMaxPlanNv> device_of_model = DeviceOfModel(binding);

  // What a controller's configure refuses of the docking functions' profile
  // (docking_config.hpp): the batch judges no profile that controller parks.
  const bool nlp = binding.kind == SearchBatchKind::kNlp;
  HandDockingParams hand;
  NlpCatchSearchParams nlp_params;
  MpcDockingSegmentPlannerParams docking_params;
  if (nlp || !mpc) {
    hand = ParseHandDockingParams(catching);
    if (nlp) {
      nlp_params = ParseNlpSearchParams(catching, nv);
    }
    if (!mpc) {
      docking_params = ParseMpcDockingSegmentParams(catching, nv);
    }
    DockingConfig config;
    config.nlp = nlp;
    config.docking = !mpc;
    config.catching = &catching;
    config.hand = &hand;
    // The binding's leads are each core's, from the entrance crossing.
    config.search_close_lead = binding.hand_t_close_lead;
    config.planner_close_lead = sb.hand_t_close_lead;
    config.search = &nlp_params;
    config.planner = &docking_params;
    if (std::string why = DockingConfigRefusal(config); !why.empty()) {
      out.error = std::move(why);
      return out;
    }
  }

  std::unique_ptr<SegmentPlanner> segment_planner;
  std::string error;
  if (mpc) {
    if (sb.q_min.size() != n || sb.q_max.size() != n || sb.qdot_max.size() != n ||
        sb.tau_max.size() != n) {
      throw std::invalid_argument("binding: the segment.mpc vectors do not match the arm");
    }
    MpcSegmentPlannerModel model;
    model.arm = arm;
    model.catch_frame = catch_frame;
    model.nv = nv;
    model.device_of_model = device_of_model;
    std::copy(sb.q_min.begin(), sb.q_min.end(), model.q_min.begin());
    std::copy(sb.q_max.begin(), sb.q_max.end(), model.q_max.begin());
    std::copy(sb.qdot_max.begin(), sb.qdot_max.end(), model.qdot_max.begin());
    std::copy(sb.tau_max.begin(), sb.tau_max.end(), model.tau_max.begin());
    MpcSegmentPlannerConstants consts;
    consts.eta_v = sb.eta_v;
    consts.t_arm_s = sb.t_arm_s;
    consts.control_dt = sb.control_dt;
    consts.v_eps = planner.mpc_segment.v_eps;
    segment_planner = MakeMpcSegmentPlanner(model, consts, planner.mpc_segment, &WakeClock, &error);
  } else {
    if (sb.qd_rating.size() != n) {
      throw std::invalid_argument("binding: segment.mpc_docking.qd_rating does not match the arm");
    }
    MpcDockingSegmentPlannerParams params = docking_params;
    MpcDockingSegmentPlannerModel model;
    model.arm = arm;
    model.catch_frame = catch_frame;
    model.nv = nv;
    model.device_of_model = device_of_model;
    model.limits = sb.limits;
    std::copy(sb.qd_rating.begin(), sb.qd_rating.end(), model.qd_rating.begin());
    model.warm_pose = planner.wait_pose;
    MpcDockingSegmentPlannerConstants consts;
    consts.t_arm_s = sb.t_arm_s;
    consts.control_dt = sb.control_dt;
    ApplyHandDocking(hand, sb.hand_t_close_e2e, sb.hand_t_close_lead, sb.ball_mass, params.core);
    SetNominalPosture(planner, device_of_model, nv, params.core);
    segment_planner = MakeMpcDockingSegmentPlanner(model, consts, params, &WakeClock, &error);
  }
  if (segment_planner == nullptr) {
    out.error = std::string("planner.segment.") + mode + ": " + error;
    return out;
  }

  auto batch = std::make_unique<PlanBatch>();
  batch->q_rest_ = std::move(built.q_rest);
  batch->cycle_.Configure(planner);
  batch->cycle_.SetClock(&WakeClock);
  if (!batch->cycle_.Bind(
          {&batch->traj_, &batch->cov_, &batch->rt_, &batch->plan_, &batch->segment_})) {
    throw std::invalid_argument("the planner cycle refused its boxes");
  }
  batch->cycle_.InstallSearch(std::move(built.search));
  batch->cycle_.InstallSegmentPlanner(std::move(segment_planner));
  out.batch = std::move(batch);
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
