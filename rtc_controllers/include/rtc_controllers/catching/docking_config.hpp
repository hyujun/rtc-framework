// ── Whether the selected docking functions can run on a profile ──────────────
//
// The nlp search and the mpc_docking segment planner each parse their own map,
// and each parser refuses what is wrong in that map alone. What is checked here
// is what only shows when the maps, the hand's identified capture set and the
// selection are laid side by side: a value nobody may guess that is unset, a
// lead shorter than the flight from the entrance plane, a search allowed to run
// longer than the RT state a first segment is planned from may be old, two
// grids that are not one, a reference closing speed outside the hand's band.
//
// One function, so that every builder of the pair refuses the same profiles: a
// controller's configure, and the offline whole-wake batch that judges a
// profile the way that controller would run it.
//
// Non-RT: configure-time only (allocates the refusal string).
#pragma once

#include "rtc_controllers/catching/docking_params.hpp"

#include <yaml-cpp/yaml.h>

#include <limits>
#include <string>

namespace rtc::catching {

/// @brief The hand's close lead as the docking functions count it [s]: from the
///        ball's crossing of the ENTRANCE plane, `hand.s_ent` before the catch
///        point along the approach axis, at the core's reference closing speed
///        (`-core.nu_ref.z()`).
/// @param hand_close_lead `robot.hand.T_close_lead` [s], from the catch point
/// @return NaN when `hand_close_lead` or `hand.s_ent` is not finite, or the
///         reference closing speed is not positive
[[nodiscard]] double EntranceCloseLead(double hand_close_lead, const HandDockingParams& hand,
                                       const MpcDockingSegmentCoreParams& core) noexcept;

/// What DockingConfigRefusal reads. The pointers are borrowed for the call;
/// `search` is read when `nlp`, `planner` when `docking`.
struct DockingConfig {
  bool nlp{false};      ///< `planner.search.mode` is nlp
  bool docking{false};  ///< `planner.segment.mode` is mpc_docking
  /// The `catching:` tree (whether the nlp search's map is in it).
  const YAML::Node* catching{nullptr};
  const HandDockingParams* hand{nullptr};
  /// `robot.hand.T_close_e2e` or `robot.hand.T_close_lead` is unset or TBD.
  bool hand_close_unset{false};
  /// EntranceCloseLead on the search's core [s] (read when `nlp`), and on the
  /// planner's (read when `docking`) — the value each is handed as its close
  /// lead (ApplyHandDocking).
  double search_close_lead{std::numeric_limits<double>::quiet_NaN()};
  double planner_close_lead{std::numeric_limits<double>::quiet_NaN()};
  /// `core.ball.mass` is unset or TBD.
  bool ball_mass_unset{false};
  const NlpCatchSearchParams* search{nullptr};
  const MpcDockingSegmentPlannerParams* planner{nullptr};
};

/// @brief Why the selected docking functions cannot run on this profile.
/// @return empty when they can (and when neither is selected), else the first
///         refusal in this order: an unset key of the hand's capture set, an
///         unset close time or ball mass, the nlp search's map (with its
///         `core:`) missing from the tree, the planner's `approach.n_pre_max`
///         unset, a close lead of either core that is not >= 0, the search's
///         `budget.budget_s` not below the oldest RT state the selected segment
///         planner plans a first segment from, DockingGridMismatch and
///         `continuous_tc` (both selected), a reference closing speed outside
///         `robot.hand.docking.speed`. Keys are named from `catching.`.
/// @throws std::invalid_argument a null `hand`, or a null `catching` /
///         `search` / `planner` that the selection reads
[[nodiscard]] std::string DockingConfigRefusal(const DockingConfig& config);

}  // namespace rtc::catching
