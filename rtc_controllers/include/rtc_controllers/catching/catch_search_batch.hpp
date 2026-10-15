// ── Offline batch of the planner's search: which throws it would accept ─────
// dynamic_catching L3 §4.1. `catch_pose_ik_batch` judges a catch pose and
// `catch_gate_batch` the gates behind it, one candidate at a time. This runs
// the SEARCH ITSELF — `CatchSearch::Plan`, whichever implementation the
// configuration selects — over the wakes of a throw, exactly as a planner wake
// calls it, and records what each wake decided. The verdict is the runtime
// function's, called here and nowhere re-derived.
//
// WHAT A WAKE IS HANDED, AND WHAT THAT LEAVES OUT. Every wake is the search's
// first look at an arm that follows no plan:
//   • the arm is at rest on the wait pose, the RT reports no plan and no
//     segment, and its report is as old as the wake (age 0);
//   • the covariance is the zero matrix, valid and matched to the prediction —
//     a search that reads it judges on the mean alone;
//   • the clock does not advance (StoppedClock): nothing a search measures on
//     it elapses, so a verdict does not depend on the machine. What a search
//     derives from its CONFIGURED budget (how many solves a wake holds, where
//     its first node stands) is unchanged; what only a running clock produces
//     — a search cut by its budget, a solve past its deadline — never appears.
// The wakes of one throw are run in order until the first one that returns a
// plan, and no further: what a search does once the arm follows a plan (the
// switching rule, a replacement, a follow window) needs the segment planner
// and the RT, and is not reproduced here.
//
// THE WHOLE WAKE (PlanBatch). A binding that also carries a segment planner's
// values runs, per wake, what a planner thread runs: `PlannerCycle::Run` with
// that search and that segment planner installed — the search, the first
// segment's solve, and the cycle's decision to publish the two as a pair or to
// withhold both. The wakes of a throw then run up to the first whose PAIR is
// published, so a throw whose plans are all withheld runs every wake. What a
// wake is handed is the list above with one change: the clock stands at the
// wake's own instant (WakeClock) instead of at zero. It still does not advance
// inside a wake, and a search measures only differences on it, so the search's
// verdicts are those of the search-only run; the segment planner needs the
// instant itself (the age of the RT's report, where node 0 can stand).
// Not reproduced there either: a solve past its budget, a pair dropped because
// a newer prediction or a reset landed during the solve, and everything after
// the pair — the replans, a replacement, the RT's own admission of the pair.
//
// It knows no robot: the model, the catching tree, the values a controller
// binds at configure time (SearchBatchBinding) and the wakes all arrive from
// the caller.
#pragma once

#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controllers/catching/catch_pose_ik_params.hpp"
#include "rtc_controllers/catching/catch_search.hpp"
#include "rtc_controllers/catching/docking_params.hpp"
#include "rtc_controllers/catching/grid_catch_search.hpp"
#include "rtc_controllers/catching/mpc_docking_segment_core.hpp"
#include "rtc_controllers/catching/nlp_catch_search.hpp"
#include "rtc_controllers/catching/planner_cycle.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/reach_bound.hpp"
#include "rtc_controllers/catching/search_stats.hpp"
#include "rtc_controllers/catching/segment_planner.hpp"
#include "rtc_controllers/catching/traj_ingress.hpp"
#include "rtc_controllers/catching/trajectory.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <Eigen/Core>
#include <pinocchio/multibody/model.hpp>
#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <iosfwd>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace rtc::catching {

/// The clock a batch search is configured with: it never advances.
[[nodiscard]] std::int64_t StoppedClock() noexcept;

/// The clock of a whole-wake batch (PlanBatch): the instant of the wake
/// `PlanBatch::Run` is running on the calling thread, 0 outside one. It does
/// not advance inside a wake.
[[nodiscard]] std::int64_t WakeClock() noexcept;

// ── The wakes ────────────────────────────────────────────────────────────────

/// One wake of one throw: the prediction the search is handed at `now_ns`.
struct SearchBatchWake {
  std::int64_t throw_id{0};
  std::int64_t wake{0};             ///< index within the throw, ascending
  std::int64_t now_ns{0};           ///< the wake instant, on the samples' axis
  std::vector<TrajSample> samples;  ///< ascending instants, model world, ≤ kCap
};

/// Read wakes. Column ORDER IS TAKEN FROM THE HEADER. Required:
/// `throw_id,wake,now_ns,t_ns,p_{x,y,z},v_{x,y,z},a_{x,y,z}` — one row per
/// predicted sample, the rows of a wake contiguous and in instant order, the
/// wakes of a throw contiguous and in `wake` order. Throws
/// `std::invalid_argument` naming the line on anything else: a non-finite
/// cell, a wake with more than `kCap` samples or instants that do not
/// increase, a `now_ns` that changes inside a wake, a wake index that does not
/// increase inside a throw, a throw id that comes back after another throw.
[[nodiscard]] std::vector<SearchBatchWake> ParseSearchWakeCsv(std::istream& in);

/// What `Plan` is handed for `wake` (header note): the prediction, its zero
/// covariance, and the report of an arm at rest on `q_rest` (DEVICE order, one
/// entry per arm joint) that follows no plan.
struct SearchBatchInputs {
  TrajectorySnapshot traj{};
  CovarianceSnapshot cov{};
  PlannerRtState rt{};
};

[[nodiscard]] SearchBatchInputs MakeSearchBatchInputs(const SearchBatchWake& wake,
                                                      std::span<const double> q_rest);

// ── The result ───────────────────────────────────────────────────────────────

/// What one wake decided.
struct SearchBatchRow {
  std::int64_t throw_id{0};
  std::int64_t wake{0};
  std::int64_t now_ns{0};
  PlanSnapshot plan{};
  SearchStats stats{};
  /// Wall-clock duration of the `Plan` call alone [µs] (steady clock) — of the
  /// whole `PlannerCycle::Run` on a row of PlanBatch. Not the search's own
  /// clock, which does not advance: this is how long the machine took, so
  /// unlike every other field it is NOT reproducible.
  double wall_us{0.0};
  // ── A row of PlanBatch::Run only; RunSearchBatch leaves the defaults ──
  /// The row is a whole wake's. `plan` is then what the cycle STORED — the
  /// pair's plan, or a "no plan" — and is left at its default on a wake that
  /// stored nothing, whose search may still have produced one (`search_valid`).
  bool cycle{false};
  bool search_valid{false};  ///< the wake's search returned a plan
  CycleOutcome outcome{CycleOutcome::kIdle};
  SegmentRecord segment{};  ///< the first segment's solve
};

/// The columns that carry a planner wake's record use the names the planner
/// event log gives them (`search_valid`, `plan_reason`, `n_in_window`, `n_ik`,
/// `settling`, `n_pass`, `rej_*`, `budget_hit`, `nlp_ran`, `nlp_reason`, `nlp_n_*`,
/// `nlp_rej_*`), so one reader reduces both. After them: the plan
/// (`t_c_ns`, `lead_s` = t_c − now, `p_c_*`, `v_c_*`, `score`, `w5`, `w6`,
/// `rank_mask`, `gamma_f`, `nlp_lead_s`) and `q_star<i>` for i = 0…nv−1 in
/// DEVICE order. A wake without a plan leaves the plan columns empty. The LAST
/// column is `wall_us`, the wall-clock time of the `Plan` call [µs] — present
/// on every row.
[[nodiscard]] std::string SearchBatchCsvHeader(int nv);

/// One result row, doubles at round-trip precision.
[[nodiscard]] std::string SearchBatchCsvRow(const SearchBatchRow& row, int nv);

/// The header of a whole-wake batch: SearchBatchCsvHeader's columns in their
/// places, and before the last one (`wall_us`) what the cycle and the first
/// segment's solve did, under the planner event log's names: `outcome`
/// (CycleOutcomeName), `segment_outcome`, `segment_kind`,
/// `segment_core_reason` (names), `segment_k`, `segment_n_nodes`,
/// `segment_iterations`, `segment_from_search`, `segment_start_from_search`
/// (the solve started from the search's solution of a chain, widened —
/// SegmentRecord::start_from_search), `segment_ref_scaled`,
/// `segment_ref_scale`, `segment_slack_max`, `segment_slack_terminal_max`,
/// `segment_catch_pos_err`, `segment_catch_axis_err`,
/// `segment_speed_ratio_max`, `segment_tau_ratio_max`.
///
/// `search_valid` is the SEARCH's verdict on the wake; the plan columns are
/// filled only where a pair was published (`outcome` published and
/// `search_valid` 1) — a plan withheld with its segment leaves them empty.
[[nodiscard]] std::string PlanBatchCsvHeader(int nv);

/// One row of a whole-wake batch (`row.cycle` true).
[[nodiscard]] std::string PlanBatchCsvRow(const SearchBatchRow& row, int nv);

/// Run `search` over `wakes` (header note): throws in the given order, each
/// from a reset search (`ResetTrial`), its wakes in order up to and including
/// the first that returns a valid plan.
/// @param q_rest the wait pose, DEVICE order, one entry per arm joint
/// @throws std::invalid_argument a wake with no sample or more than `kCap`
[[nodiscard]] std::vector<SearchBatchRow> RunSearchBatch(CatchSearch& search,
                                                         std::span<const double> q_rest,
                                                         const std::vector<SearchBatchWake>& wakes);

// ── The binding ──────────────────────────────────────────────────────────────

/// Which search a binding builds.
enum class SearchBatchKind : std::uint8_t { kGrid, kNlp };

/// Which segment planner a binding builds.
enum class SegmentBatchKind : std::uint8_t { kMpc, kMpcDocking };

/// What a controller resolves at configure time and hands the segment planner
/// next to the catching tree. As SearchBatchBinding: nothing has a default,
/// joint vectors are in MODEL order, NaN is a value.
struct SegmentBatchBinding {
  SegmentBatchKind kind{SegmentBatchKind::kMpc};
  /// The SEGMENT model's device map, when the segment planner has a model of
  /// its own (the whole device, the search on a chain of it): a permutation of
  /// the device's joints. Empty = the search's `device_of_model` (one model).
  std::vector<int> device_of_model;
  double t_arm_s{0.0};     ///< the arm lag the lead axis runs on [s]
  double control_dt{0.0};  ///< the RT period [s]
  // ── mpc ──
  std::vector<double> q_min;  ///< the position box [rad]
  std::vector<double> q_max;
  std::vector<double> qdot_max;  ///< device ratings [rad/s]
  std::vector<double> tau_max;   ///< device ratings [N·m]
  double eta_v{0.0};             ///< the velocity row's margin
  /// The hand's close lead from t_c and T_close,tot = T_close_e2e + h/2 [s]
  /// (MpcSegmentPlannerConstants; NaN = TBD).
  double t_close_lead{std::numeric_limits<double>::quiet_NaN()};
  double t_close_total{std::numeric_limits<double>::quiet_NaN()};
  // ── mpc_docking ──
  MpcDockingSegmentCoreLimits limits{};  ///< `armature` is zero
  std::vector<double> qd_rating;         ///< device ratings [rad/s]
  double hand_t_close_e2e{0.0};          ///< the measured closure time [s]
  double hand_t_close_lead{0.0};         ///< the close lead from the cores' catch node [s]
  double ball_mass{0.0};                 ///< [kg]
};

/// What a controller resolves at configure time and hands the search next to
/// the catching tree — device ratings, margins already applied, the hand's
/// timing. Nothing here has a default: a search fed a guessed constant still
/// produces a complete, plausible map. Joint vectors are in MODEL order. NaN
/// is a value (the profile's "TBD": the gate that needs it fails).
struct SearchBatchBinding {
  SearchBatchKind kind{SearchBatchKind::kGrid};
  std::vector<int> device_of_model;  ///< the arm device's index of model joint j
  /// The arm device's joint count, when it is more than the search model's (a
  /// search on a chain of a wider device): `device_of_model` is then an
  /// injection into it, and `planner.wait_pose` has one entry per device joint.
  /// 0 = the search model's (the model is the whole device).
  int n_dev{0};
  // ── grid ──
  std::vector<double> qdot_max;   ///< device ratings [rad/s]
  std::vector<double> qddot_max;  ///< the acceleration box [rad/s²]; empty = none
  GridCatchSearchConstants grid{};
  // ── nlp ──
  NlpCatchSearchConstants nlp{};
  MpcDockingSegmentCoreLimits limits{};  ///< `armature` is zero
  double hand_t_close_e2e{0.0};          ///< the measured closure time [s]
  double hand_t_close_lead{0.0};         ///< the close lead from the cores' catch node [s]
  double ball_mass{0.0};                 ///< [kg]
  /// The segment planner of a whole-wake batch (MakePlanBatch); none in a
  /// search-only binding.
  std::optional<SegmentBatchBinding> segment;
};

/// Read a binding file. Top level: `search` (`grid` | `nlp`),
/// `device_of_model`, optionally `n_dev` (the device's joint count when the
/// search model is a chain of a wider device — `device_of_model` is then an
/// injection into 0…n_dev−1), and the map of that search:
///   grid: qdot_max, qddot_max, eta_v, v_max, a_dec, t_arm_s, t_close_lead,
///         t_close_total, ball_mass, ref_omega, ref_zeta, ref_a_max,
///         control_dt, follows_segments
///   nlp:  t_arm_s, control_dt, t_close_lead, hand_t_close_e2e,
///         hand_t_close_lead, ball_mass,
///         limits: {q_min, q_max, qd_max, qdd_max, tau_max, tau_lo, tau_hi}
/// and, optionally, `segment`: `mode` (`mpc` | `mpc_docking`) and the map of
/// that planner:
///   mpc:         q_min, q_max, qdot_max, tau_max, eta_v, t_arm_s, control_dt
///   mpc_docking: t_arm_s, control_dt, qd_rating, hand_t_close_e2e,
///                hand_t_close_lead, ball_mass,
///                limits: {q_min, q_max, qd_max, qdd_max, tau_max, tau_lo, tau_hi}
/// and, with a segment model of its own, `segment.device_of_model` (a
/// permutation of 0…segment_nv−1; absent = the search's map).
/// Every key of a selected map is required and no other key is accepted
/// (`n_dev` and `segment.device_of_model` are the two optional ones — and
/// `n_dev` is required, equal to `segment_nv`, once the segment model is wider
/// than the search's: that model is the whole device).
/// `qddot_max` / `limits.qdd_max` may be empty (no acceleration box); every
/// other joint vector has one entry per joint of ITS model — `nv` for the
/// search's map, `segment_nv` for the segment planner's. Scalars may be `.nan`.
/// @param nv the search model's joint count
/// @param segment_nv the segment model's, 0 = `nv` (one model)
/// @throws std::invalid_argument naming the key on anything else, including a
///         `device_of_model` that is not a permutation of 0…nv−1 (an injection
///         into 0…n_dev−1 with `n_dev`), an `n_dev` below `nv` or above
///         kMaxPlanNv, and — `segment_nv` above `nv` — an `n_dev` that is
///         absent or not `segment_nv`.
[[nodiscard]] SearchBatchBinding ParseSearchBatchBinding(const YAML::Node& root, int nv,
                                                         int segment_nv = 0);

/// A configured search and what it stands on.
struct SearchBatchSearch {
  std::unique_ptr<CatchSearch> search;  ///< null when `error` is set
  std::vector<double> q_rest;           ///< `planner.wait_pose`, DEVICE order
  std::string error;                    ///< why the search refused its configuration
};

/// Build the search `binding` selects, the way a controller's configure does:
/// the parsers read `catching` (the composed `catching:` tree) for everything
/// that is a key of it, `binding` gives the rest, and the search is configured
/// on StoppedClock.
/// @param arm the search's model (the catch sub-model, or a chain of a wider
///        device with `binding.n_dev`); `handle` is a reorder-free handle on
///        it and outlives the returned search
/// @throws std::invalid_argument what the parsers throw, a tree without the
///         selected search's `planner.search.<kind>` map (the parsers would
///         default it), and a `planner.wait_pose` that is not one entry per
///         joint of the arm DEVICE
[[nodiscard]] SearchBatchSearch MakeSearchBatchSearch(
    const std::shared_ptr<const pinocchio::Model>& arm, rtc_urdf_bridge::RtModelHandle& handle,
    pinocchio::FrameIndex catch_frame, const YAML::Node& catching,
    const SearchBatchBinding& binding);

// ── The whole wake ───────────────────────────────────────────────────────────

struct PlanBatchBuild;

/// The segment planner's model, when it is not the search's: the whole arm
/// device (`planner.sub_model`), the search on a chain of it
/// (`planner.search.sub_model`).
struct SegmentBatchModel {
  std::shared_ptr<const pinocchio::Model> arm;
  pinocchio::FrameIndex catch_frame{0};
};

/// A planner cycle with a search and a segment planner installed, and the
/// boxes it reads and writes (header: "THE WHOLE WAKE").
class PlanBatch {
 public:
  PlanBatch() = default;
  PlanBatch(const PlanBatch&) = delete;
  PlanBatch& operator=(const PlanBatch&) = delete;

  /// Run the cycle over `wakes`: throws in the given order, each behind a
  /// trial reset of the cycle, its wakes in order up to and including the
  /// first whose pair is published.
  /// @throws std::invalid_argument a wake with no sample or more than `kCap`
  [[nodiscard]] std::vector<SearchBatchRow> Run(const std::vector<SearchBatchWake>& wakes);

  /// `planner.wait_pose`, DEVICE order.
  [[nodiscard]] const std::vector<double>& RestPose() const noexcept { return q_rest_; }

 private:
  friend PlanBatchBuild MakePlanBatch(const std::shared_ptr<const pinocchio::Model>&,
                                      rtc_urdf_bridge::RtModelHandle&, pinocchio::FrameIndex,
                                      const YAML::Node&, const SearchBatchBinding&,
                                      const SegmentBatchModel*);

  rtc::SeqLock<TrajectorySnapshot> traj_{};
  rtc::SeqLock<CovarianceSnapshot> cov_{};
  rtc::SeqLock<PlannerRtState> rt_{};
  rtc::SeqLock<PlanSnapshot> plan_{};
  rtc::SeqLock<SegmentSnapshot> segment_{};
  PlannerCycle cycle_{};
  std::vector<double> q_rest_;
  std::uint32_t reset_epoch_{0};  // one per throw: the cycle resets on its change
};

/// A configured whole-wake batch, or why a function refused its configuration.
struct PlanBatchBuild {
  std::unique_ptr<PlanBatch> batch;  ///< null when `error` is set
  std::string error;
};

/// Build the cycle `binding` selects, the way a controller's configure does:
/// the search as MakeSearchBatchSearch builds it, the segment planner of
/// `binding.segment` from its own map of `catching`
/// (`planner.segment.<mode>`) and the binding's values, both installed in a
/// cycle configured with the tree's `planner.*` and timed on WakeClock. A
/// profile on which that controller would park its docking functions is
/// reported in `error`, in the controller's words (DockingConfigRefusal).
/// @param segment_model the segment planner's model when it has one of its
///        own (the search's `arm` is then a chain of it); null = `arm` for both
/// @throws std::invalid_argument what MakeSearchBatchSearch and the parsers
///         throw, a binding without `segment`, a segment vector or device map
///         that does not match the segment model, a segment model that is not
///         the whole device, and a tree without the selected planner's map
[[nodiscard]] PlanBatchBuild MakePlanBatch(const std::shared_ptr<const pinocchio::Model>& arm,
                                           rtc_urdf_bridge::RtModelHandle& handle,
                                           pinocchio::FrameIndex catch_frame,
                                           const YAML::Node& catching,
                                           const SearchBatchBinding& binding,
                                           const SegmentBatchModel* segment_model = nullptr);

// ── The reach bound ──────────────────────────────────────────────────────────

/// The `ik.eps_pos` [m] each search tests its reach pre-filter with, as
/// `ParseCatchPoseIkParams` — the parse `MakeSearchBatchSearch` hands the
/// search — reads it from that search's own map
/// (`planner.search.<kind>.ik.eps_pos`). The two may differ in one tree.
/// `selected` is that of the search the tree's `planner.search.mode` names.
struct ReachTolerances {
  double selected{0.0};
  double grid{0.0};
  double nlp{0.0};
};

/// @throws std::invalid_argument what the parsers throw
[[nodiscard]] ReachTolerances SearchReachTolerances(const YAML::Node& catching);

/// The reach bound as one JSON line (no newline), doubles at `%.17g`:
/// `{"schema": "catch_reach_bound/1", "frame": ..., "sub_model": ...,
/// "centre": [x, y, z], "radius": r, "tolerance": eps_pos,
/// "tolerance_by_search": {"grid": ..., "nlp": ...}, "joints": n,
/// "unbounded_by": ...}`. `centre` is in the model world [m]; `tolerance` is
/// the selected search's. An unbounded chain (infinite radius) writes
/// `"radius": null`, JSON having no infinity; a non-finite centre or
/// tolerance is written as `null` for the same reason.
[[nodiscard]] std::string ReachBoundJson(const ReachBound& bound, std::string_view frame,
                                         std::string_view sub_model,
                                         const ReachTolerances& tolerances);

}  // namespace rtc::catching
