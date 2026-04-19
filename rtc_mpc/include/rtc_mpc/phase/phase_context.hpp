#ifndef RTC_MPC_PHASE_PHASE_CONTEXT_HPP_
#define RTC_MPC_PHASE_PHASE_CONTEXT_HPP_

/// @file phase_context.hpp
/// @brief Aggregate handed from @ref PhaseManagerBase to the OCP builder.
///
/// A `PhaseContext` is produced per tick by a concrete phase manager and
/// consumed non-RT when the MPC handler decides to (re)build the OCP. It
/// groups the things OCP construction needs: contact topology, cost
/// weights, task target, and the dispatch key (`ocp_type`).
///
/// Carrying an explicit `phase_changed` flag lets the MPC handler short-
/// circuit OCP reconfiguration when nothing changed — the dominant case.
///
/// Robot-agnostic: no robot-specific phase enums live here. `phase_id` /
/// `phase_name` are opaque to rtc_mpc; only the concrete manager (Phase 7)
/// assigns meaning.

#include "rtc_mpc/phase/phase_cost_config.hpp"
#include "rtc_mpc/types/contact_plan_types.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

#include <string>

namespace rtc::mpc {

/// @brief Snapshot of the phase state for one OCP build / reconfigure.
struct PhaseContext {
  int phase_id{0};           ///< opaque id, assigned by concrete FSM
  std::string phase_name{};  ///< diagnostics only
  bool phase_changed{false}; ///< true iff `phase_id` differs from prev tick

  ContactPlan contact_plan{};    ///< active contact frames over the horizon
  PhaseCostConfig cost_config{}; ///< per-phase cost weights + references

  /// Dispatch key for @ref MPCFactory (Phase 5). Current values:
  /// `"kinodynamics"` (Phase 3), `"fulldynamics"` (Phase 4). Unknown
  /// strings cause factory rejection — never silently fall back.
  std::string ocp_type{"kinodynamics"};

  /// End-effector SE3 target in the model's world frame. Identity means
  /// "no target set" (track current pose); concrete managers overwrite.
  pinocchio::SE3 ee_target{pinocchio::SE3::Identity()};
};

} // namespace rtc::mpc

#endif // RTC_MPC_PHASE_PHASE_CONTEXT_HPP_
