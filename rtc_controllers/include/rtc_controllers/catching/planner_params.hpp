// ── Planner parameters (dynamic_catching S6, L3 §6) ──────────────────────────
//
// The `catching.planner.*` keys the runtime planner reads. Sibling of
// `catching_params.hpp` (the G0-C validator subset) and
// `catch_pose_ik_params.hpp` (`planner.search.grid.ik.*` / `planner.search.grid.catchability.*`),
// not part of either: those two describe WHAT a catch pose is, this one describes how the planner
// searches for one and how it ranks what it finds. Non-RT
// (`on_configure`).
//
// Keys join in the commit that first reads them (a parsed key nothing reads is
// a key nobody notices is wrong): S6-A the thread keys, S6-B the search,
// ranking, switching and freeze keys below, MPC E1-F03 `planner.segment.mpc.*`
// (the segment MPC's stop horizon, replan window and publish thresholds), MPC
// E1-F08 its APPROACH–stop keys (`approach`, `budget`, `catch`).
//
// WHOSE KEYS ARE READ (E1-F16). `planner.search.grid.*` is the grid search's
// and `planner.segment.mpc.*` the mpc segment planner's. A configuration runs
// one search and at most one segment planner (`planner.search.mode`,
// `planner.segment.mode`), and a function that does not run has no say in
// whether the configuration is valid: the caller names the functions it
// selected (PlannerKeySelection) and only their maps are opened. A map that is
// not opened leaves its fields here at the defaults, its unset decisions
// unset, and a malformed value in it unreported. What is read from `planner`
// itself (the thread keys, `wait_pose`, `sub_model`, `freeze`) is read under
// every selection.
//
// TWO KINDS OF "MISSING". A key with a documented default (L3 §6) takes it when
// absent. A key whose value is a DECISION (`freeze.T_freeze`,
// `workspace.catch_box`, `sub_model`) has no default: absent or `TBD` is
// recorded as unset, and the binding parks the controller rather than guess —
// the same rule as a consumed TBD elsewhere (L0 §5.3, A-S5-12). A present but
// malformed value of either kind is refused (std::invalid_argument).
#pragma once

#include "rtc_controllers/catching/trajectory.hpp"  // kMaxPlanNv, kMaxSegmentNodes

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace rtc::catching {

/// Range bounds (L3 §6). Exposed so the tests and the doc table cite the same
/// numbers the parser enforces.
inline constexpr double kPlannerWakeTimeoutMinS = 0.005;
inline constexpr double kPlannerWakeTimeoutMaxS = 0.5;
inline constexpr double kPlannerBudgetMinS = 0.001;
inline constexpr double kPlannerBudgetMaxS = 0.05;
/// Upper bound on IK candidates per cycle — the R-2 pre-filter. The value is a
/// capacity (fixed-size scratch), not a tuning default; the default is 8.
inline constexpr int kPlannerMaxIkCapacity = 40;

inline constexpr std::size_t kPlannerMaxGammaGrid = 16;
inline constexpr std::size_t kPlannerMaxWindowGrid = 8;

/// Upper bound of `planner.search.grid.switch.samples`. `SwitchStep` evaluates the followed
/// ramp once per sample on the planner thread for every switch check, so the
/// count is a cost on the cycle budget, not only a resolution (default 9).
inline constexpr int kSwitchSamplesMax = 64;

/// Upper bound of `planner.segment.mpc.cost.w_perp` [1/m²]: the largest
/// stop-path weight the cores are tested to solve with (a stop core alone in
/// test_catching_mpc_segment_core.cpp, the planner's catch and stop cores on both
/// test arms in test_catching_approach_planner.cpp). The weight raises the
/// QP's condition number with nothing else to bound it — the configure
/// warm-up cannot notice a value too large, its line runs through the catch
/// frame — so the parser refuses anything above.
inline constexpr double kMpcSegmentStopPathWeightMax = 1e4;

/// Capacity of the MPC segment planner's replan window: instances k = 0..k_max, one
/// MpcSegmentCore each (MD-31). A capacity, not a default (the default k_max is 4).
inline constexpr int kMaxMpcSegmentReplans = 8;

/// `planner.segment.mpc.*` (MPC plan E1-F03 · E1-F08, MD-24 · MD-31 · MD-33 ·
/// MD-54 – MD-64, MD-91). The segment MPC's settings the planner owns. Every
/// design field of the core's MpcSegmentCoreParams comes from YAML — the grid
/// (`horizon.*`, `approach.n_pre_max` / `dt_pre_s`), `eta_tau`, `m_q`, the
/// catch weights and the relative-velocity slack (`catch.*`), the cost scalars
/// (`cost.*`), the trust region and the rest tolerance (`linearization.*`) and
/// the solver tolerances (`solver.*`); the shipped values are the core's own
/// defaults. Two of them are numbers the grid search has a key of its own for
/// — `v_eps` here, and `eta_v` (ParseCatchingParams reads it, with `mode` and
/// `switch_margin`): a function keeps every value it is designed with under its
/// own map, so the two can be tuned apart (#711).
/// What stays in code is not a design value: the solver's preconditioner and
/// KKT backend (the RT no-allocation and infeasibility verdict rely on them),
/// the test-only `reference_assembly`, and the capacities.
struct MpcSegmentPlannerParams {
  // There is no `enabled`: the planner solves these segments exactly when
  // `planner.segment.mode` is mpc. That needs `planner.enabled` (the binding
  // parks the pair otherwise) and a pre-catch grid (`approach.n_pre_max` ≥ 1).
  /// `horizon.n_nodes` N_s and `horizon.dt_s` Δ_s: N_s·Δ_s IS the stopping
  /// time (MD-21). The default 14 × 0.025 = 0.35 s is E1-F03's (MD-24); the
  /// shipped profiles set MD-54's 7 × 0.05 with the pre-catch grid below.
  /// Δ_s must be a whole number of nanoseconds (the grid is integer ns).
  int n_nodes{14};
  double dt_s{0.025};
  /// `horizon.blocks` — move blocking, Σ = n_nodes, B ≥ 3. Post-catch replan
  /// k uses MpcSegmentBlocksFor(k) (the largest trailing block shrinks first).
  std::array<int, kMaxSegmentNodes> blocks{1, 1, 2, 2, 4, 4};
  int n_blocks{6};
  /// `replan.k_max` — post-catch replans only at grid points k ≤ k_max; the
  /// stop still ends at t_c + N_s·Δ_s (MD-31).
  int k_max{4};
  /// `eta_tau` — torque row fraction of τ_max (core η'_τ).
  double eta_tau{0.7};
  /// `m_q` [rad] — position margin inside the joint limits.
  double m_q{0.05};
  /// `v_eps` [m/s] — the ball speed below which its direction of travel is
  /// undefined: a solve that needs the direction is withheld (> 0). The mpc
  /// segment planner's own floor; the search IK has `planner.search.grid.ik.v_eps`.
  double v_eps{1e-6};
  /// `publish.slack_max` / `publish.slack_terminal_max` — the largest torque
  /// slack (fraction of τ_max) a published segment may carry, over nodes 1..N
  /// and at node N (MD-33). Both 0.1 until E1-F06 tightens the terminal one.
  double slack_max{0.1};
  double slack_terminal_max{0.1};

  // ── The pre-catch part (E1-F08 #661, MD-55 – MD-64) ────────────────────────
  // All PROVISIONAL (#663 tunes them).

  /// `approach.n_pre_max` — the most pre-catch intervals a segment may start
  /// with (MD-54: 6). One catch core per count 1..n_pre_max is built at
  /// configure time (MD-64). 0 = no pre-catch grid: a plan is published only
  /// with a segment that starts before t_c, so the MPC segment planner refuses to
  /// configure and `planner.segment.mode: mpc` parks (MD-70).
  int n_pre_max{0};
  /// `approach.dt_pre_s` [s] — the pre-catch spacing Δ_pre (MD-54), a whole
  /// number of nanoseconds.
  double dt_pre_s{0.1};
  /// `approach.rest_tol` [rad/s] — the first solve assumes the arm rests at
  /// its wait pose; above this max |q̇_cmd| it withholds (kNotAtRest).
  double rest_tol{0.05};
  /// `budget.first_s` / `budget.replan_s` [s] — a solve's ceiling and the
  /// lead it is planned with: the first solve (with the search, one wake)
  /// and every later one (MD-56).
  double budget_first_s{0.035};
  double budget_replan_s{0.025};
  /// `replan.same_point` — re-solve a pre-catch grid point the followed
  /// segment already starts at, with a newer prediction (MD-58).
  bool replan_same_point{true};
  /// `publish.catch_pos_err_max` [m] — the largest catch-node position error
  /// (FK at the solution vs the predicted ball) a segment may carry (MD-62).
  double catch_pos_err_max{0.02};
  /// `catch.*` — the core's catch terms (E1-F07 measured values): approach
  /// axis, relative velocity along / across the ball's travel, its target
  /// fraction γ_ref; the position weight W_p = κ(Σ_p + σ_floor² I)⁻¹ capped
  /// at w_max, or w_const·I without a usable Σ_p; w_Δ's schedule
  /// clamp(tr Σ_p / σ_ref², 0, 1) (MD-63). σ_ref is compared with a TRACE,
  /// so it is not a per-axis σ (tr ≈ 3σ²).
  double w_axis{100.0};
  double w_v_par{1.0};
  double w_v_perp{20.0};
  double gamma_ref{1.0};
  double kappa{1.0};
  double sigma_floor{0.01};
  double w_max{1e4};
  double w_const{2500.0};
  double sigma_ref{0.03};
  /// `catch.rho_v` / `catch.v_rel_allow` [m/s] — the relative-velocity slack
  /// row at the catch node: |v̂_b − v_C| ≤ v_rel_allow·(1 + s_v) per axis,
  /// penalised by rho_v·s_v. rho_v 0 (the default) builds no slack variable
  /// and no rows; rho_v > 0 needs v_rel_allow > 0 (the parser refuses the pair
  /// otherwise). s_v is RECORDED (SegmentRecord::slack_v), never a publish
  /// gate: no threshold is defined for it, and with γ_ref < 1 the cost's own
  /// optimum sits off the rows, so s_v > 0 is then structural.
  double rho_v{0.0};
  double v_rel_allow{0.0};

  // ── The core's own design values (YAML keys) ───────────────────────────────
  // Each default equals MpcSegmentCoreParams' (a test pins it), so a profile without
  // the keys solves what it always did.
  /// `cost.jerk_weight` — R_j per joint in ARM (device) order, each > 0; empty
  /// = all 1 (the core's default). A non-empty list must have one entry per
  /// arm joint — the planner's configure checks that (the parser has no DOF).
  std::vector<double> jerk_weight{};
  /// `cost.u_scale` [rad/s³]: the jerk cost is (u/u_scale)², so it re-weights
  /// jerk against w_Δ and ρ_τ — not a pure preconditioner.
  double u_scale{1e3};
  /// `cost.w_delta` [1/rad²] — pull toward the reference.
  double w_delta{1.0};
  /// `cost.rho_tau` — torque slack penalty; 0 = the torque rows are off, which
  /// makes the publish judgement's slack condition vacuous.
  double rho_tau{10.0};
  /// `cost.w_perp` [1/m²], in [0, kMpcSegmentStopPathWeightMax] — the stop-path
  /// term: on the nodes from the catch on, the catch frame's distance from a
  /// line is penalised; 0 (the default) = off, and no line is then built or
  /// required. The line is the BALL's (user decision 2026-10-03, #698):
  /// through its predicted catch position along its direction of travel at
  /// t_c, as the catch-core solve takes them. A stop-core replan keeps the
  /// line of the segment the RT follows (its source). A solve whose line
  /// cannot be built is withheld (MpcSegmentPlanner, mpc_segment_planner.hpp), never run
  /// on a default line.
  double w_perp{0.0};
  /// `catch.axis_theta_max` [rad], in (0, π) — the largest axis error of the
  /// reference the approach-axis term is linearised at.
  double axis_theta_max{1.5707963267948966};
  /// `linearization.delta_tr` [rad] — trust-region half-width, finite > 0 (the
  /// core also takes +inf; a profile does not). `m_q` must stay below it.
  double delta_tr{0.1};
  /// `linearization.reference_rest_tol` — |q̇̄_N|, |q̈̄_N| bound of a supplied
  /// reference; above `solver.eps_abs`.
  double reference_rest_tol{1e-4};
  /// `linearization.ref_speed_fraction` ∈ (0, 1] — the first solve's reference
  /// reaches its target at this fraction of the velocity box η_v·q̇_max (MD-62).
  double ref_speed_fraction{0.9};
  /// `solver.max_iter` · `max_iter_in` (≥ 1) · `eps_abs` (> 0) · `eps_rel`
  /// (≥ 0): the QP's tolerances. Preconditioner and backend stay in code.
  int solver_max_iter{200};
  int solver_max_iter_in{100};
  double solver_eps_abs{1e-6};
  double solver_eps_rel{0.0};
  /// Whether the profile set `horizon` itself: the code default above is
  /// MD-24's horizon, not MD-54's, and the configure warns when the MPC segment
  /// planner runs on it.
  bool horizon_explicit{false};

  [[nodiscard]] std::int64_t DtNs() const noexcept {
    return static_cast<std::int64_t>(std::llround(dt_s * 1e9));
  }

  [[nodiscard]] std::int64_t DtPreNs() const noexcept {
    return static_cast<std::int64_t>(std::llround(dt_pre_s * 1e9));
  }
};

/// Move-blocking pattern of replan instance k (N' = n_nodes − k): starting from
/// `p.blocks`, take one node off the LARGEST block k times (the last one among
/// equals), dropping a block that reaches zero. The early, fine blocks — where
/// the stop's jerk lives — keep their resolution. {1,1,2,2,4,4} gives
/// {1,1,2,2,4,3} {1,1,2,2,3,3} {1,1,2,2,3,2} {1,1,2,2,2,2} for k = 1..4.
/// @return false when k is outside [0, n_nodes) or the pattern would drop
///         below B = 3 (the core's freedom rule).
[[nodiscard]] bool MpcSegmentBlocksFor(const MpcSegmentPlannerParams& p, int k,
                                       std::array<int, kMaxSegmentNodes>& blocks,
                                       int& n_blocks) noexcept;

/// Axis-aligned box in the model world frame (decision I).
struct CatchBox {
  std::array<double, 3> min{};
  std::array<double, 3> max{};
  bool set{false};

  [[nodiscard]] constexpr bool Contains(double x, double y, double z) const noexcept {
    return set && x >= min[0] && x <= max[0] && y >= min[1] && y <= max[1] && z >= min[2] &&
           z <= max[2];
  }
};

struct ScoreWeights {
  double w_sigma{1.0};  ///< σ_max / r_cap
  double w_t{1.0};      ///< max t_min / (t_k − now − T_arm)
  double w_q{0.1};      ///< ‖q* − q_n‖²
  double w_late{0.0};   ///< (t_k,max − t_k): > 0 prefers a late catch
  double w_gamma{5.0};  ///< −γ_f: > 0 prefers a soft catch
  /// Added once per failed RANK gate (decision D, 2026-09-23). An invented key
  /// (`planner.search.grid.score.penalty`): the decision fixes the rule, not the size. It
  /// must dominate the continuous terms or a failed gate is a rounding error.
  double penalty{10.0};
};

struct PlannerParams {
  // ── Thread (S6-A) ──────────────────────────────────────────────────────────
  /// `planner.enabled` — spawn the planner thread at activation.
  bool enabled{false};
  /// `planner.wake_timeout_s` [s] (decision H). Also the thread's period.
  double wake_timeout_s{0.05};
  /// `planner.search.grid.budget_s` [s] — one cycle's compute budget (R-2).
  double budget_s{0.020};
  /// `planner.wait_pose` [rad] — IK seed, ARM joint (device) order. 0 = absent.
  std::array<double, kMaxPlanNv> wait_pose{};
  std::int32_t wait_pose_n{0};
  /// `planner.wait_pose_source` (S8-I, #537): where the wait pose the trial
  /// homes to and seeds the IK from comes from. `kYaml` = `wait_pose` above.
  /// `kCurrent` = the arm's measured pose on the first readable tick of each
  /// activation ("the pose the arm was switched in at"); `wait_pose` stays the
  /// configure-time seed and the fallback when that pose is refused.
  enum class WaitPoseSource : std::uint8_t { kYaml = 0, kCurrent = 1 };
  WaitPoseSource wait_pose_source{WaitPoseSource::kYaml};
  /// `planner.provisional` (invented, same shape as `reference.provisional`):
  /// the block as a whole is provisional — sim warns, a real arm is parked.
  bool provisional{true};

  // ── Search (S6-B) ──────────────────────────────────────────────────────────
  /// `planner.sub_model` — the robot config's `urdf.sub_models` entry that
  /// reaches the catch frame's parent (R-3). Empty = unset.
  std::string sub_model;
  /// `planner.search.grid.max_ik` — IK candidates per cycle after the pre-filter (R-2).
  int max_ik{8};
  /// `planner.search.grid.slice.dt` [s] — candidate spacing; the vision grid is thinned to
  /// it (never interpolated at S6-B: L3 §5.3 "격자를 그대로").
  double slice_dt{0.05};
  /// `planner.search.grid.slice.t_lead_min` [s]; NaN = "equal to freeze.T_freeze".
  double slice_t_lead_min{std::numeric_limits<double>::quiet_NaN()};
  /// `planner.search.grid.slice.t_max` [s] — the latest candidate, from now.
  double slice_t_max{0.95};
  /// `planner.search.grid.time.margin` [s] (§4.3).
  double time_margin{0.03};
  /// `planner.search.grid.n_settle` — snapshots to skip after a track change (§4.4).
  int n_settle{3};
  /// `planner.search.grid.unc.kappa_sigma` (§4.4).
  double kappa_sigma{0.3};
  /// `planner.search.grid.gamma.margin` [m/s] (§4.5).
  double gamma_margin{0.1};
  /// `planner.search.grid.gamma.unit_speed_damping` — λ of the DLS unit-speed solve behind
  /// v_dir,max (§4.5), in (0, 1]. The offline map reads the same key
  /// (catch_gate_map) so the two cannot differ.
  double unit_speed_damping{1e-3};
  /// The rollout (§4.8, S6-C): `planner.search.grid.gamma.grid` (γ_f candidates),
  /// `window_grid` [s] (T_w candidates), `eta_a`, `eps_term` [m], and the
  /// screening step `planner.search.grid.rollout.dt_coarse` [s] (invented key: §4.8 left
  /// the coarse-to-fine method to S6.3).
  std::array<double, kPlannerMaxGammaGrid> gamma_grid{0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  std::size_t gamma_grid_n{7};
  std::array<double, kPlannerMaxWindowGrid> window_grid{0.3, 0.45, 0.6};
  std::size_t window_grid_n{3};
  double eta_a{0.8};
  double eps_term{0.002};
  double rollout_dt_coarse{0.01};
  /// `planner.search.grid.budget.*` (§4.6). σ_trk and δ are 0 until measured (S10).
  double n_sigma{2.0};
  double sigma_trk{0.0};
  double clock_err{0.0};
  /// `planner.search.grid.hand.d_eff` / `r_cap` [m] — the γ window's and the budget's hand
  /// constants (first runtime consumer). NaN = unset.
  double d_eff{std::numeric_limits<double>::quiet_NaN()};
  double r_cap{std::numeric_limits<double>::quiet_NaN()};
  /// `planner.search.grid.switch.*` (§4.7). `eta_jump`: the fraction of the reference's
  /// `a_max` a switch may step u_des by (decision ⑥, 2026-09-23 — replaces the
  /// distance limits `e_jump_max` / `ed_jump_max`, which the parser refuses).
  double switch_delta_j{0.1};
  double switch_eta_jump{0.25};
  /// `planner.search.grid.switch.samples` in [2, kSwitchSamplesMax] — instants of the followed ramp
  /// the §4.7 step bound is taken the worst over (the code divides by samples − 1).
  int switch_samples{9};
  /// `planner.freeze.T_freeze` [s] (decision G). NaN = unset.
  double t_freeze{std::numeric_limits<double>::quiet_NaN()};
  /// `planner.freeze.t_stop_plan` [s], ≥ T_freeze: while the RT follows a plan
  /// on a segment planner's segments, the search goes on running until the
  /// followed plan's catch instant is this close — then only the segment is
  /// replanned. Absent from the YAML it is T_freeze (the parser fills it in).
  /// NaN = the search does not run at all while a plan is followed: the value
  /// of a PlannerParams no parser filled, and of an unset T_freeze.
  double t_stop_plan{std::numeric_limits<double>::quiet_NaN()};
  /// `planner.search.grid.score.*` (§4.10 + decision D).
  ScoreWeights score{};
  /// `planner.search.grid.workspace.catch_box` (decision I). `set` false = unset.
  CatchBox catch_box{};

  // ── Segment MPC (MPC E1-F03) ─────────────────────────────────────────────────
  /// `planner.segment.mpc.*`. Absent = the defaults.
  MpcSegmentPlannerParams mpc_segment{};

  /// The candidate lead floor actually used: t_lead_min, or T_freeze.
  [[nodiscard]] double LeadMin() const noexcept {
    return slice_t_lead_min == slice_t_lead_min ? slice_t_lead_min : t_freeze;
  }
};

/// Which functions' maps ParsePlannerParams opens (the header's "WHOSE KEYS ARE
/// READ"). The default opens every one — what a caller with no configuration
/// to select from wants (a test of one map, a tool that lists the keys).
struct PlannerKeySelection {
  bool search_grid{true};  ///< `planner.search.grid.*`
  bool segment_mpc{true};  ///< `planner.segment.mpc.*`
};

/// Parse `planner.*` from the `catching:` tree root (the same node
/// ParseCatchingParams takes). An absent `planner:` section yields the
/// defaults, and so does a map `selection` leaves closed. Throws `std::invalid_argument` (and only
/// that) on a present but malformed key: a non-map section, a non-bool flag, a number outside its
/// L3 §6 range, a `wait_pose` that is empty / non-finite / longer than `kMaxPlanNv`, a `catch_box`
/// whose min exceeds its max, a `segment.mpc` horizon whose blocks do not sum to n_nodes, a Δ_s or
/// Δ_pre that is not whole ns, a k_max whose replan patterns would drop below three blocks, or an
/// n_pre_max whose pre-catch nodes or blocks would not fit next to the stop's.
[[nodiscard]] PlannerParams ParsePlannerParams(const YAML::Node& catching,
                                               const PlannerKeySelection& selection = {});

}  // namespace rtc::catching
