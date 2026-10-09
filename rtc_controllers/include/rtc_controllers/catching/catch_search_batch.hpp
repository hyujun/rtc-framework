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
// It knows no robot: the model, the catching tree, the values a controller
// binds at configure time (SearchBatchBinding) and the wakes all arrive from
// the caller.
#pragma once

#include "rtc_controllers/catching/catch_pose_ik_params.hpp"
#include "rtc_controllers/catching/catch_search.hpp"
#include "rtc_controllers/catching/docking_params.hpp"
#include "rtc_controllers/catching/grid_catch_search.hpp"
#include "rtc_controllers/catching/nlp_catch_search.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/search_stats.hpp"
#include "rtc_controllers/catching/traj_ingress.hpp"
#include "rtc_controllers/catching/trajectory.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <Eigen/Core>
#include <pinocchio/multibody/model.hpp>
#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <iosfwd>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace rtc::catching {

/// The clock a batch search is configured with: it never advances.
[[nodiscard]] std::int64_t StoppedClock() noexcept;

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
};

/// The columns that carry a planner wake's record use the names the planner
/// event log gives them (`search_valid`, `plan_reason`, `n_in_window`, `n_ik`,
/// `settling`, `n_pass`, `rej_*`, `budget_hit`, `nlp_ran`, `nlp_reason`, `nlp_n_*`,
/// `nlp_rej_*`), so one reader reduces both. After them: the plan
/// (`t_c_ns`, `lead_s` = t_c − now, `p_c_*`, `v_c_*`, `score`, `w5`, `w6`,
/// `rank_mask`, `gamma_f`, `nlp_lead_s`) and `q_star<i>` for i = 0…nv−1 in
/// DEVICE order. A wake without a plan leaves the plan columns empty.
[[nodiscard]] std::string SearchBatchCsvHeader(int nv);

/// One result row, doubles at round-trip precision.
[[nodiscard]] std::string SearchBatchCsvRow(const SearchBatchRow& row, int nv);

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

/// What a controller resolves at configure time and hands the search next to
/// the catching tree — device ratings, margins already applied, the hand's
/// timing. Nothing here has a default: a search fed a guessed constant still
/// produces a complete, plausible map. Joint vectors are in MODEL order. NaN
/// is a value (the profile's "TBD": the gate that needs it fails).
struct SearchBatchBinding {
  SearchBatchKind kind{SearchBatchKind::kGrid};
  std::vector<int> device_of_model;  ///< the arm device's index of model joint j
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
};

/// Read a binding file. Top level: `search` (`grid` | `nlp`),
/// `device_of_model`, and the map of that search:
///   grid: qdot_max, qddot_max, eta_v, v_max, a_dec, t_arm_s, t_close_lead,
///         t_close_total, ball_mass, ref_omega, ref_zeta, ref_a_max,
///         control_dt, follows_segments
///   nlp:  t_arm_s, control_dt, t_close_lead, hand_t_close_e2e,
///         hand_t_close_lead, ball_mass,
///         limits: {q_min, q_max, qd_max, qdd_max, tau_max, tau_lo, tau_hi}
/// Every key of the selected search's map is required and no other key is
/// accepted. `qddot_max` / `limits.qdd_max` may be empty (no acceleration
/// box); every other joint vector has `nv` entries. Scalars may be `.nan`.
/// @throws std::invalid_argument naming the key on anything else, including a
///         `device_of_model` that is not a permutation of 0…nv−1.
[[nodiscard]] SearchBatchBinding ParseSearchBatchBinding(const YAML::Node& root, int nv);

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
/// @param arm the catch sub-model; `handle` is a reorder-free handle on it and
///        outlives the returned search
/// @throws std::invalid_argument what the parsers throw, a tree without the
///         selected search's `planner.search.<kind>` map (the parsers would
///         default it), and a `planner.wait_pose` that is not one entry per
///         arm joint
[[nodiscard]] SearchBatchSearch MakeSearchBatchSearch(
    const std::shared_ptr<const pinocchio::Model>& arm, rtc_urdf_bridge::RtModelHandle& handle,
    pinocchio::FrameIndex catch_frame, const YAML::Node& catching,
    const SearchBatchBinding& binding);

}  // namespace rtc::catching
