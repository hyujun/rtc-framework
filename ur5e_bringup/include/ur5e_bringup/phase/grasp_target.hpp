#ifndef UR5E_BRINGUP_PHASE_GRASP_TARGET_HPP_
#define UR5E_BRINGUP_PHASE_GRASP_TARGET_HPP_

/// @file grasp_target.hpp
/// @brief Passive record of the grasp-scenario goal poses consumed by
///        @ref GraspPhaseManager.
///
/// `GraspTarget` is updated off-MPC-thread (typically from a BT leaf or a
/// ROS topic callback) and published to the phase manager via
/// @ref GraspPhaseManager::SetTaskTarget. The manager takes a snapshot under
/// its internal mutex on each `Update` call so the MPC-thread path stays
/// lock-free-adjacent (one `std::mutex::try_lock` at phase-boundary only).
///
/// All poses are expressed in the model's world frame (same frame as the
/// `tcp` argument of `PhaseManagerBase::Update`).

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <Eigen/Core>
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

namespace ur5e_bringup::phase {

/// @brief External command bus driving the grasp FSM edges that are not
///        derived from TCP proximity / contact force.
enum class GraspCommand : int {
  kNone = 0,       ///< No pending command; FSM advances via geometric guards.
  kApproach = 1,   ///< IDLE → APPROACH — start the grasp sequence.
  kManipulate = 2, ///< HOLD → MANIPULATE — begin object manipulation.
  kRetreat = 3, ///< MANIPULATE → RETREAT — move arm back to approach start.
  kRelease = 4, ///< RETREAT → RELEASE — open the hand.
  kAbort = 5,   ///< Any phase → IDLE — user cancellation / fault.
};

/// @brief Aggregate of the poses the grasp FSM needs over one episode.
///
/// @var grasp_pose     TCP pose at full CLOSURE — the "make contact" target.
/// @var pregrasp_pose  TCP pose at PRE_GRASP — typically a local-frame offset
///                     above @c grasp_pose (e.g. +Z by 5–10 cm). Computed by
///                     the caller; the FSM treats it as an opaque target.
/// @var approach_start TCP pose recorded on APPROACH entry; RETREAT drives
///                     back to this pose before opening the hand.
struct GraspTarget {
  pinocchio::SE3 grasp_pose{pinocchio::SE3::Identity()};
  pinocchio::SE3 pregrasp_pose{pinocchio::SE3::Identity()};
  pinocchio::SE3 approach_start{pinocchio::SE3::Identity()};
};

} // namespace ur5e_bringup::phase

#endif // UR5E_BRINGUP_PHASE_GRASP_TARGET_HPP_
