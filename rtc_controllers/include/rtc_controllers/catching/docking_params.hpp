// ── Docking parameters (dynamic_catching E1-F16) ─────────────────────────────
//
// The YAML keys of the two docking-based functions, and the hand values both
// share. Non-RT (`on_configure`); sibling of `planner_params.hpp`, which reads
// `planner.search.grid.*` and `planner.segment.mpc.*`.
//
// THE KEY BELONGS TO THE FUNCTION, THE MEASURED VALUE BELONGS TO THE HAND. The
// capture set a hand has (entrance plane, corridor, lateral polygon, speed
// limits, closure window, impact) is identified by measuring that hand once,
// and then holds for whichever function plans the arm: it lives under
// `catching.robot.hand.docking.*` (HandDockingParams). What a function tunes —
// its grid, its budget, its cost — lives under its own map:
// `catching.planner.search.nlp.*` (NlpCatchSearchParams) and
// `catching.planner.segment.mpc_docking.*` (MpcDockingSegmentPlannerParams).
// Both carry a `core:` sub-map with the docking core's tuning, the same
// sub-schema under both (ReadDockingCoreParams). A hand value written under a
// function's `core:` is refused with a pointer to where it belongs, because it
// is an easy mistake and a silently ignored one would run on a default.
//
// TWO KINDS OF "MISSING" (as planner_params.hpp). A tuning key takes its
// default when absent, and `TBD` is refused for it. A hand value is a DECISION:
// absent or `TBD` is recorded as unset (HandDockingParams::FirstUnset names it)
// and the binding parks the controller rather than guess. A present but
// malformed value of either kind, and a key the schema does not know, is
// refused with std::invalid_argument naming the full key path.
//
// The tree root every parser takes is the `catching:` map.
#pragma once

#include "rtc_controllers/catching/mpc_docking_segment_core.hpp"
#include "rtc_controllers/catching/nlp_catch_search.hpp"
#include "rtc_controllers/catching/trajectory.hpp"  // kMaxSegmentNodes

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace rtc::catching {

/// `catching.robot.hand.docking.*`: the identified capture set of one hand. A
/// decision value has no default: NaN (a double) or 0 (`n_faces`) is unset.
struct HandDockingParams {
  /// `docking.provisional`: true = the values are not cleared for hardware.
  bool provisional{true};
  /// `docking.s_ent` [m], [0, 0.5]: entrance plane offset.
  double s_ent{std::numeric_limits<double>::quiet_NaN()};
  /// `docking.corridor.r_ent` [m], (0, 0.5]: corridor radius at the entrance.
  double r_ent{std::numeric_limits<double>::quiet_NaN()};
  /// `docking.corridor.tan_theta`, [0, 10]: corridor half-angle tangent.
  double tan_theta{std::numeric_limits<double>::quiet_NaN()};
  /// `docking.lateral.n_faces`, 3..kMaxDockingFaces; 0 = unset.
  int n_faces{0};
  /// `docking.lateral.faces_a`: a FLAT list of 2·n_faces numbers
  /// [ax0, ay0, ax1, ay1, …] — flat because the launch overlay bridge cannot
  /// carry a list of maps. Unit normals (a YAML carries a few decimals, so the
  /// parser accepts |‖a‖ − 1| ≤ 1e-3 and normalises exactly: the core refuses
  /// non-unit normals). Unset entries are NaN; in a parsed value the entries
  /// past n_faces are zero.
  std::array<Eigen::Vector2d, kMaxDockingFaces> face_a{};
  /// `docking.lateral.faces_b`: n_faces numbers [m], each in [-0.5, 0.5].
  std::array<double, kMaxDockingFaces> face_b{};
  /// `docking.lateral.rho_ref`: [x, y] [m], each in [-0.5, 0.5].
  Eigen::Vector2d rho_ref{std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::quiet_NaN()};
  /// `docking.speed.c_min` [m/s], (0, 20].
  double c_min{std::numeric_limits<double>::quiet_NaN()};
  /// `docking.speed.c_cap_max` [m/s], (0, 20], and > c_min when both are set.
  double c_cap_max{std::numeric_limits<double>::quiet_NaN()};
  /// `docking.speed.c_ent_max` [m/s], (0, 20].
  double c_ent_max{std::numeric_limits<double>::quiet_NaN()};
  /// `docking.speed.v_perp_max` [m/s], (0, 20].
  double v_perp_max{std::numeric_limits<double>::quiet_NaN()};
  /// `docking.speed.a_brake` [m/s²], [0, 1000].
  double a_brake{std::numeric_limits<double>::quiet_NaN()};
  /// `docking.closure.delta_lo` [s], [-2, 2]: the closure window after the
  /// crossing of the entrance plane (see ApplyHandDocking for the axis).
  double delta_lo{std::numeric_limits<double>::quiet_NaN()};
  /// `docking.closure.delta_hi` [s], [-2, 2], and > delta_lo when both are set.
  double delta_hi{std::numeric_limits<double>::quiet_NaN()};
  /// `docking.closure.sigma_tau` [s], [0, 1]: closure latency jitter.
  double sigma_tau{std::numeric_limits<double>::quiet_NaN()};
  /// `docking.impact.contact_point_hand`: [x, y, z] [m], each in [-0.5, 0.5].
  Eigen::Vector3d contact_point_hand{std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::quiet_NaN()};
  /// `docking.impact.restitution`, [0, 1].
  double restitution{std::numeric_limits<double>::quiet_NaN()};
  /// `docking.impact.e_max` [J], > 0. NOT a decision: absent = +inf (the row is
  /// off), and `TBD` is refused.
  double e_max{std::numeric_limits<double>::infinity()};
  /// `docking.impact.p_max` [N·s], > 0; absent = +inf (as e_max).
  double p_max{std::numeric_limits<double>::infinity()};

  /// The full key (`robot.hand.docking.s_ent`, `…lateral.faces_a`, …) of the
  /// first unset decision, in the order the fields are declared above, or
  /// nullptr when every decision is set. `faces_a`/`faces_b` count as unset
  /// when any of their first n_faces entries is not finite.
  [[nodiscard]] const char* FirstUnset() const noexcept;
};

/// @brief Read `catching.robot.hand.docking.*`. The whole section absent leaves
///        every decision unset (no throw).
/// @throws std::invalid_argument a present but malformed value, a section that
///         is not a map, a key the schema does not know, a face list whose
///         length is not 2·n_faces / n_faces, a normal further than 1e-3 from
///         unit length, c_cap_max ≤ c_min or delta_hi ≤ delta_lo.
///         When `n_faces` is unset the two lists are not read at all.
[[nodiscard]] HandDockingParams ParseHandDockingParams(const YAML::Node& catching);

/// @brief Write a hand's values into a core's params: s_ent, r_ent, tan_theta,
///        n_faces, face_a and face_b (the first n_faces entries; the rest stay
///        as they are), rho_ref, c_min, c_cap_max, c_ent_max, v_perp_max,
///        a_brake, delta_lo, delta_hi, sigma_tau, contact_point_hand,
///        restitution, e_max, p_max — and from the caller `m_ball`, and the
///        DERIVED delta_0.
///
/// delta_0 = t_close_e2e_s − t_close_lead_s. The hand sequencer commands the
/// close `t_close_lead` BEFORE the catch instant, and the closure then takes
/// `t_close_e2e`, so it completes `t_close_e2e − t_close_lead` after the catch
/// node's crossing of the entrance plane. The window `delta_lo/hi` is on the
/// same axis (seconds after the crossing), which is why delta_0 is derived and
/// is not a key of any map.
///
/// Touches nothing else (`face_eps`, the grid and the cost weights stay). No
/// validation: the core's Init has the final say.
void ApplyHandDocking(const HandDockingParams& hand, double t_close_e2e_s, double t_close_lead_s,
                      double ball_mass_kg, MpcDockingSegmentCoreParams& core) noexcept;

/// @brief Read a function's `core:` sub-map into `core`, which arrives holding
///        the defaults to fall back on. `path` is the full key of the map
///        (`planner.search.nlp.core`), for messages. An absent or Null map
///        changes nothing.
///
/// A per-joint weight is ONE scalar in YAML, applied to every joint
/// (`Eigen::VectorXd::Constant(nv, value)`); exactly 0 for `r_tau`, `r_acc` and
/// `w_q_nom` leaves that vector EMPTY (the core's "off"). `face_eps` and
/// `mu_init` are one scalar for every face / row group.
///
/// Not keys: `catch_time_variable` (the search sets it per core), the grid
/// fields, `q_nom`, `manip_d_q`, and what ApplyHandDocking writes. `TBD` is
/// refused for every key (tuning has defaults; it is not a decision).
/// @throws std::invalid_argument a present but malformed value, a map that is
///         not a map, a key this reader does not know (a hand value under
///         `core:` is named as belonging under robot.hand.docking), a pair
///         lambda1 + lambda2 that is not positive, or `nv` outside
///         [1, kMaxPlanNv] when the map is present.
void ReadDockingCoreParams(const YAML::Node& core_map, const std::string& path, int nv,
                           MpcDockingSegmentCoreParams& core);

/// @brief `catching.planner.search.nlp.*` into a default-constructed
///        NlpCatchSearchParams, its `core:` sub-map into `.core`. Not read here
///        (the caller fills them): `wait_pose`, `wait_pose_n`, `limits`, and
///        the hand's values in `.core`. `planner.search.nlp` absent = the
///        defaults, `catch_box` unset. The siblings `mode` and `ik` of the map
///        are other parsers' and tolerated; any other unknown key is refused.
/// @throws std::invalid_argument as ReadDockingCoreParams, and for t_max ≤
///         t_lead_min, solve_s > budget_s, n_pre.max < n_pre.min, stop.blocks
///         that do not sum to stop.n_nodes, or n_pre.max + stop.n_nodes above
///         kMaxSegmentNodes.
[[nodiscard]] NlpCatchSearchParams ParseNlpSearchParams(const YAML::Node& catching, int nv);

/// `catching.planner.segment.mpc_docking.*`: the new segment planner's own
/// values. Defaults are the values of the search's shared grid.
struct MpcDockingSegmentPlannerParams {
  /// `switch_margin`: ρ_max of the RT's segment-switch gate, > 0.
  double switch_margin{1.0};
  /// `eta_v`: speed margin of the switch gate's headroom, (0, 1).
  double eta_v{0.9};
  /// `approach.n_pre_max`, 1..kMaxSegmentNodes: one core per pre-catch interval
  /// count 1..n_pre_max. A DECISION — 0 = unset (the key absent): a planner
  /// with no pre-catch grid solves nothing, so nobody may guess its extent,
  /// and the binding parks on it.
  int n_pre_max{0};
  /// `approach.dt_pre_s` [s], [0.005, 0.5], a whole number of nanoseconds.
  double dt_pre_s{0.1};
  /// `approach.rest_tol` [rad/s], (0, 1]: max |q̇_cmd| that counts as "at rest"
  /// for a first solve.
  double rest_tol{1e-3};
  /// `stop.n_nodes`, 3..kMaxSegmentNodes.
  int n_stop{7};
  /// `stop.dt_s` [s], [0.005, 0.2], a whole number of nanoseconds.
  double dt_stop_s{0.05};
  /// `stop.blocks`: positive sizes, at least 3, summing to n_stop. Entries past
  /// n_stop_blocks are zero.
  int n_stop_blocks{4};
  std::array<int, kMaxSegmentNodes> stop_block_sizes{1, 1, 2, 3};
  /// `budget.first_s` [s], [0.001, 5]: the first solve's budget.
  double budget_first_s{0.035};
  /// `budget.replan_s` [s], [0.001, 5]: a replan's budget.
  double budget_replan_s{0.025};
  /// `replan.same_point`.
  bool replan_same_point{true};
  /// `publish.slack_c_max` [m], ≥ 0: largest corridor slack a published segment
  /// may carry.
  double slack_c_max{0.0};
  /// `publish.slack_v_max` [m²/s²], ≥ 0: largest speed-envelope slack (the
  /// envelope row bounds the closing speed squared).
  double slack_v_max{0.0};
  /// `core:`.
  MpcDockingSegmentCoreParams core;

  [[nodiscard]] std::int64_t DtPreNs() const noexcept {
    return static_cast<std::int64_t>(std::llround(dt_pre_s * 1e9));
  }

  [[nodiscard]] std::int64_t DtStopNs() const noexcept {
    return static_cast<std::int64_t>(std::llround(dt_stop_s * 1e9));
  }
};

/// @brief `catching.planner.segment.mpc_docking.*`; the map absent = the
///        defaults. Unknown keys under it and its sub-maps are refused.
/// @throws std::invalid_argument as ParseNlpSearchParams (for the keys it
///         shares), and n_pre_max + n_stop above kMaxSegmentNodes.
[[nodiscard]] MpcDockingSegmentPlannerParams ParseMpcDockingSegmentParams(
    const YAML::Node& catching, int nv);

/// @brief Whether the nlp search can run with this planner. The planner
///        republishes the search's solution after re-evaluating it on its own
///        core, which needs identical grids.
/// @return nullptr when they agree, else a static string naming the first pair
///         of keys that differs, checked in this order: the two `dt_pre_s`
///         (whole ns), `stop.n_nodes`, `stop.dt_s` (whole ns), `stop.blocks`
///         (count and every size), and the search's `n_pre.max`, which must not
///         exceed the planner's `approach.n_pre_max`.
[[nodiscard]] const char* DockingGridMismatch(
    const NlpCatchSearchParams& search, const MpcDockingSegmentPlannerParams& planner) noexcept;

}  // namespace rtc::catching
