// ── Planner parameters (dynamic_catching S6, L3 §6) ──────────────────────────
//
// The `catching.planner.*` keys the runtime planner reads. Sibling of
// `catching_params.hpp` (the G0-C validator subset) and
// `catch_pose_ik_params.hpp` (`planner.ik.*` / `planner.catchability.*`), not
// part of either: those two describe WHAT a catch pose is, this one describes
// how the planner searches for one and how it ranks what it finds. Non-RT
// (`on_configure`).
//
// Keys join in the commit that first reads them (a parsed key nothing reads is
// a key nobody notices is wrong): S6-A the thread keys, S6-B the search,
// ranking, switching and freeze keys below, MPC E1-F03 `planner.decel_mpc.*`
// (the decel MPC's stop horizon, replan window and publish thresholds).
//
// TWO KINDS OF "MISSING". A key with a documented default (L3 §6) takes it when
// absent. A key whose value is a DECISION (`freeze.T_freeze`,
// `workspace.catch_box`, `sub_model`) has no default: absent or `TBD` is
// recorded as unset, and the binding parks the controller rather than guess —
// the same rule as a consumed TBD elsewhere (L0 §5.3, A-S5-12). A present but
// malformed value of either kind is refused (std::invalid_argument).
#pragma once

#include "rtc_controllers/catching/trajectory.hpp"  // kMaxPlanNv, kMaxDecelNodes

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

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

/// Capacity of the decel planner's replan window: instances k = 0..k_max, one
/// DecelMpc each (MD-31). A capacity, not a default (the default k_max is 4).
inline constexpr int kMaxDecelReplans = 8;

/// `planner.decel_mpc.*` (MPC plan E1-F03, MD-24 · MD-26 · MD-31 · MD-33).
/// The decel MPC's settings the planner owns; the core's own tuning (jerk
/// weight, trust region, slack penalty, solver) stays at DecelMpcParams'
/// defaults, and η_v is `planner.gamma.eta_v` (no second key for one margin).
struct DecelPlannerParams {
  /// `enabled` — pre-compute the stop segment in COMMITTED / CLOSING / DECEL.
  /// Needs `planner.enabled` (the binding parks the pair otherwise).
  bool enabled{false};
  /// `horizon.n_nodes` N_s and `horizon.dt_s` Δ_s: N_s·Δ_s IS the stopping
  /// time (MD-21). Shipped 14 × 0.025 = 0.35 s (MD-24). Δ_s must be a whole
  /// number of nanoseconds (the grid is integer ns).
  int n_nodes{14};
  double dt_s{0.025};
  /// `horizon.blocks` — move blocking, Σ = n_nodes, B ≥ 3. Post-catch replan
  /// k uses DecelBlocksFor(k) (the largest trailing block shrinks first).
  std::array<int, kMaxDecelNodes> blocks{1, 1, 2, 2, 4, 4};
  int n_blocks{6};
  /// `replan.t_pre_s` [s] — the first (cold) solve waits until t_c − now_lead
  /// ≤ t_pre (MD-26): before that the initial state would be extrapolated over
  /// a T_freeze-long span.
  double t_pre_s{0.1};
  /// `replan.k_max` — post-catch replans only at grid points k ≤ k_max; the
  /// stop still ends at t_c + N_s·Δ_s (MD-31).
  int k_max{4};
  /// `eta_tau` — torque row fraction of τ_max (core η'_τ).
  double eta_tau{0.7};
  /// `m_q` [rad] — position margin inside the joint limits.
  double m_q{0.05};
  /// `publish.slack_max` / `publish.slack_terminal_max` — the largest torque
  /// slack (fraction of τ_max) a published segment may carry, over nodes 1..N
  /// and at node N (MD-33). Both 0.1 until E1-F06 tightens the terminal one.
  double slack_max{0.1};
  double slack_terminal_max{0.1};

  [[nodiscard]] std::int64_t DtNs() const noexcept {
    return static_cast<std::int64_t>(std::llround(dt_s * 1e9));
  }
};

/// Move-blocking pattern of replan instance k (N' = n_nodes − k): starting from
/// `p.blocks`, take one node off the LARGEST block k times (the last one among
/// equals), dropping a block that reaches zero. The early, fine blocks — where
/// the stop's jerk lives — keep their resolution. {1,1,2,2,4,4} gives
/// {1,1,2,2,4,3} {1,1,2,2,3,3} {1,1,2,2,3,2} {1,1,2,2,2,2} for k = 1..4.
/// @return false when k is outside [0, n_nodes) or the pattern would drop
///         below B = 3 (the core's freedom rule).
[[nodiscard]] bool DecelBlocksFor(const DecelPlannerParams& p, int k,
                                  std::array<int, kMaxDecelNodes>& blocks, int& n_blocks) noexcept;

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
  /// (`planner.score.penalty`): the decision fixes the rule, not the size. It
  /// must dominate the continuous terms or a failed gate is a rounding error.
  double penalty{10.0};
};

struct PlannerParams {
  // ── Thread (S6-A) ──────────────────────────────────────────────────────────
  /// `planner.enabled` — spawn the planner thread at activation.
  bool enabled{false};
  /// `planner.wake_timeout_s` [s] (decision H). Also the thread's period.
  double wake_timeout_s{0.05};
  /// `planner.budget_s` [s] — one cycle's compute budget (R-2).
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
  /// `planner.max_ik` — IK candidates per cycle after the pre-filter (R-2).
  int max_ik{8};
  /// `planner.slice.dt` [s] — candidate spacing; the vision grid is thinned to
  /// it (never interpolated at S6-B: L3 §5.3 "격자를 그대로").
  double slice_dt{0.05};
  /// `planner.slice.t_lead_min` [s]; NaN = "equal to freeze.T_freeze".
  double slice_t_lead_min{std::numeric_limits<double>::quiet_NaN()};
  /// `planner.slice.t_max` [s] — the latest candidate, from now.
  double slice_t_max{0.95};
  /// `planner.time.margin` [s] (§4.3).
  double time_margin{0.03};
  /// `planner.n_settle` — snapshots to skip after a track change (§4.4).
  int n_settle{3};
  /// `planner.unc.kappa_sigma` (§4.4).
  double kappa_sigma{0.3};
  /// `planner.gamma.margin` [m/s] (§4.5).
  double gamma_margin{0.1};
  /// The rollout (§4.8, S6-C): `planner.gamma.grid` (γ_f candidates),
  /// `window_grid` [s] (T_w candidates), `eta_a`, `eps_term` [m], and the
  /// screening step `planner.rollout.dt_coarse` [s] (invented key: §4.8 left
  /// the coarse-to-fine method to S6.3).
  std::array<double, kPlannerMaxGammaGrid> gamma_grid{0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  std::size_t gamma_grid_n{7};
  std::array<double, kPlannerMaxWindowGrid> window_grid{0.3, 0.45, 0.6};
  std::size_t window_grid_n{3};
  double eta_a{0.8};
  double eps_term{0.002};
  double rollout_dt_coarse{0.01};
  /// `planner.budget.*` (§4.6). σ_trk and δ are 0 until measured (S10).
  double n_sigma{2.0};
  double sigma_trk{0.0};
  double clock_err{0.0};
  /// `planner.hand.d_eff` / `r_cap` [m] — the γ window's and the budget's hand
  /// constants (first runtime consumer). NaN = unset.
  double d_eff{std::numeric_limits<double>::quiet_NaN()};
  double r_cap{std::numeric_limits<double>::quiet_NaN()};
  /// `planner.switch.*` (§4.7). `eta_jump`: the fraction of the reference's
  /// `a_max` a switch may step u_des by (decision ⑥, 2026-09-23 — replaces the
  /// distance limits `e_jump_max` / `ed_jump_max`, which the parser refuses).
  double switch_delta_j{0.1};
  double switch_eta_jump{0.25};
  /// `planner.freeze.T_freeze` [s] (decision G). NaN = unset.
  double t_freeze{std::numeric_limits<double>::quiet_NaN()};
  /// `planner.score.*` (§4.10 + decision D).
  ScoreWeights score{};
  /// `planner.workspace.catch_box` (decision I). `set` false = unset.
  CatchBox catch_box{};

  // ── Decel MPC (MPC E1-F03) ─────────────────────────────────────────────────
  /// `planner.decel_mpc.*`. Absent = the defaults with `enabled` false.
  DecelPlannerParams decel{};

  /// The candidate lead floor actually used: t_lead_min, or T_freeze.
  [[nodiscard]] double LeadMin() const noexcept {
    return slice_t_lead_min == slice_t_lead_min ? slice_t_lead_min : t_freeze;
  }
};

/// Parse `planner.*` from the `catching:` tree root (the same node
/// ParseCatchingParams takes). An absent `planner:` section yields the
/// defaults. Throws `std::invalid_argument` (and only that) on a present but
/// malformed key: a non-map section, a non-bool flag, a number outside its L3
/// §6 range, a `wait_pose` that is empty / non-finite / longer than
/// `kMaxPlanNv`, a `catch_box` whose min exceeds its max, a `decel_mpc`
/// horizon whose blocks do not sum to n_nodes, a Δ_s that is not whole ns, or
/// a k_max whose replan patterns would drop below three blocks.
[[nodiscard]] PlannerParams ParsePlannerParams(const YAML::Node& catching);

}  // namespace rtc::catching
