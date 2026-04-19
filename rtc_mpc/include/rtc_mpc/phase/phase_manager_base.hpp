#ifndef RTC_MPC_PHASE_PHASE_MANAGER_BASE_HPP_
#define RTC_MPC_PHASE_PHASE_MANAGER_BASE_HPP_

/// @file phase_manager_base.hpp
/// @brief Pure-virtual FSM interface consumed by the MPC thread.
///
/// `PhaseManagerBase` is the boundary between robot-agnostic rtc_mpc and
/// robot-specific logic. Concrete managers (e.g. `GraspPhaseManager` in
/// `ur5e_bringup`) implement the FSM; rtc_mpc only knows that *some* phase
/// exists and that it produces a @ref PhaseContext per tick.
///
/// Call-site contract (see Phase 6 `mpc_thread.cpp`):
/// 1. @ref Init once, with a YAML config tree.
/// 2. Optionally @ref SetTaskTarget when the upstream planner updates the
///    goal (non-RT, event-driven — topic callback, BT leaf, etc.).
/// 3. Each MPC tick (non-RT — phase decisions are evaluated off the 500Hz
///    loop): call @ref Update with the current state + elapsed time.
/// 4. Observe @ref PhaseContext::phase_changed to decide OCP rebuild vs
///    warm-start.
///
/// Thread-safety: an instance is owned by the MPC thread; no calls are
/// expected to race. `SetTaskTarget` may come from another thread and is
/// responsible for its own synchronisation in concrete implementations.

#include "rtc_mpc/phase/phase_context.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <Eigen/Core>
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <string>

namespace rtc::mpc {

/// @brief FSM producing a @ref PhaseContext per MPC tick.
class PhaseManagerBase {
public:
  virtual ~PhaseManagerBase() = default;

  PhaseManagerBase(const PhaseManagerBase &) = delete;
  PhaseManagerBase &operator=(const PhaseManagerBase &) = delete;
  PhaseManagerBase(PhaseManagerBase &&) = delete;
  PhaseManagerBase &operator=(PhaseManagerBase &&) = delete;

  /// @brief Configure the FSM. Called once before the MPC thread starts.
  ///
  /// May read robot-specific sub-trees (phase transitions, tolerances) and
  /// pre-build one @ref PhaseCostConfig per phase. Concrete implementers
  /// should surface config errors — rtc_mpc does not prescribe an enum.
  virtual void Init(const YAML::Node &cfg) = 0;

  /// @brief Advance the FSM one step and return the current @ref PhaseContext.
  ///
  /// @param q       current configuration (nq).
  /// @param v       current velocity (nv).
  /// @param sensor  concatenated sensor vector (F/T, tactile, …) — layout is
  ///                FSM-specific; rtc_mpc only forwards.
  /// @param tcp     current end-effector pose in the world frame.
  /// @param t       elapsed time since FSM start [s].
  /// @return snapshot with `phase_changed == true` iff the FSM transitioned
  ///         on this tick.
  virtual PhaseContext Update(const Eigen::VectorXd &q,
                              const Eigen::VectorXd &v,
                              const Eigen::VectorXd &sensor,
                              const pinocchio::SE3 &tcp, double t) = 0;

  /// @brief Update the task target (e.g. goal SE3 + per-target weights).
  ///
  /// Payload is FSM-specific; for a grasp manager this typically carries
  /// the object pose and a pre-grasp offset. rtc_mpc never parses the node.
  virtual void SetTaskTarget(const YAML::Node &target) = 0;

  [[nodiscard]] virtual int CurrentPhaseId() const = 0;
  [[nodiscard]] virtual std::string CurrentPhaseName() const = 0;

  /// @brief Jump to @p phase_id bypassing transition guards (diagnostics /
  ///        scripted scenarios). Idempotent if already in that phase.
  virtual void ForcePhase(int phase_id) = 0;

protected:
  PhaseManagerBase() = default;
};

} // namespace rtc::mpc

#endif // RTC_MPC_PHASE_PHASE_MANAGER_BASE_HPP_
