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
#include <pinocchio/spatial.hpp>
#pragma GCC diagnostic pop

#include <cstdint>
#include <string>

namespace rtc::mpc {

struct HorizonReference;  // rtc_mpc/reference/stage_reference.hpp

/// @brief Snapshot of the phase state for one OCP build / reconfigure.
struct PhaseContext {
  int phase_id{0};            ///< opaque id, assigned by concrete FSM
  std::string phase_name{};   ///< diagnostics only
  bool phase_changed{false};  ///< true iff `phase_id` differs from prev tick

  ContactPlan contact_plan{};     ///< active contact frames over the horizon
  PhaseCostConfig cost_config{};  ///< per-phase cost weights + references

  /// Dispatch key for @ref MPCFactory (Phase 5). Current values:
  /// `"contact_light"` (Phase 3, renamed in 4.-1), `"contact_rich"` (Phase
  /// 4), `"kinodynamic"`. Unknown strings cause factory rejection — never
  /// silently fall back.
  std::string ocp_type{"contact_light"};

  /// End-effector SE3 target expressed in `RobotModelHandler::base_frame_id()`.
  /// When `base_frame` is universe (default fallback), this collapses to the
  /// world frame and matches the legacy semantics. cost_factory lifts it to
  /// world via `oMb · ee_target` before constructing the Aligator residual.
  /// Identity means "no target set" (track current pose); concrete managers
  /// overwrite.
  pinocchio::SE3 ee_target{pinocchio::SE3::Identity()};

  /// Per-node references and contact modes over the horizon, for an OCP that
  /// tracks them (`"kinodynamic"`); non-owning, null when there are none.
  /// The pointee must stay unchanged and alive through the `Build` /
  /// `UpdateReferences` / `Solve` call that reads it. The joint-torque OCPs
  /// do not read it.
  const HorizonReference* horizon{nullptr};

  /// Whether the horizon's node 0 sits on a fixed node grid (a manager that
  /// quantises its tick time to the node spacing). Then the handler shifts
  /// its warm start by `node_index` minus that of its last solve — 0 when
  /// the tick stayed in the same node interval, 2 or more after an overrun,
  /// a cold start when it went back or past the horizon. Off: one node a
  /// tick.
  bool on_node_grid{false};
  std::int64_t node_index{0};  ///< grid index of node 0, read when `on_node_grid`
  /// [s] the tick time minus node 0's time (in `[0, dt)` on a grid; 0 off
  /// it) — carried into `MPCSolution::horizon_offset_ns`.
  double horizon_offset{0.0};
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_PHASE_PHASE_CONTEXT_HPP_
