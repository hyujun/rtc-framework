#ifndef UR5E_BRINGUP_CONTROLLERS_WBC_GRASP_TARGET_HPP_
#define UR5E_BRINGUP_CONTROLLERS_WBC_GRASP_TARGET_HPP_

/// @file grasp_target.hpp
/// @brief Passive record of the grasp-scenario goal poses consumed by
///        @ref GraspPhaseManager.
///
/// `GraspTarget` is updated off-MPC-thread (typically from a BT leaf or a
/// ROS topic callback) and published to the phase manager via
/// @ref GraspPhaseManager::SetTaskTarget. The manager publishes snapshots
/// through a wait-free `SeqLock<GraspTarget>` so the MPC thread (RT-equivalent
/// SCHED_FIFO) never touches a mutex — see invariants.md RT-4.
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

#include <array>
#include <cstring>
#include <type_traits>

namespace integrated_bringup::phase {

/// @brief External command bus driving the grasp FSM edges that are not
///        derived from TCP proximity / contact force.
enum class GraspCommand : int {
  kNone = 0,        ///< No pending command; FSM advances via geometric guards.
  kApproach = 1,    ///< IDLE → APPROACH — start the grasp sequence.
  kManipulate = 2,  ///< HOLD → MANIPULATE — begin object manipulation.
  kRetreat = 3,     ///< MANIPULATE → RETREAT — move arm back to approach start.
  kRelease = 4,     ///< RETREAT → RELEASE — open the hand.
  kAbort = 5,       ///< Any phase → IDLE — user cancellation / fault.
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

/// @brief Trivially-copyable mirror of @ref GraspTarget for `rtc::SeqLock`.
///
/// `pinocchio::SE3` (and its `Eigen::Matrix3d` / `Vector3d` components) report
/// `std::is_trivially_copyable = false` because Eigen defines explicit
/// copy-assignment operators for alignment safety — a well-known Eigen
/// false-negative, since the underlying storage is plain `double[N]` and
/// `memcpy` is safe. `rtc::SeqLock<T>` enforces the trait at compile time, so
/// we publish through this POD wrapper and marshal at the boundary.
///
/// Layout matches `Eigen::Matrix<double,3,3,ColMajor>` internal storage
/// (default Eigen storage order), so `.data()` is a flat `double[9]`.
inline constexpr std::size_t kSE3RotDoubles = 9;  // 3x3 column-major
inline constexpr std::size_t kSE3TransDoubles = 3;
inline constexpr std::array<double, kSE3RotDoubles> kIdentityRot3x3 = {1, 0, 0, 0, 1, 0, 0, 0, 1};
inline constexpr std::array<double, kSE3TransDoubles> kZeroTrans3 = {0, 0, 0};

struct GraspTargetPOD {
  std::array<double, kSE3RotDoubles> grasp_rot = kIdentityRot3x3;
  std::array<double, kSE3TransDoubles> grasp_t = kZeroTrans3;
  std::array<double, kSE3RotDoubles> pregrasp_rot = kIdentityRot3x3;
  std::array<double, kSE3TransDoubles> pregrasp_t = kZeroTrans3;
  std::array<double, kSE3RotDoubles> approach_rot = kIdentityRot3x3;
  std::array<double, kSE3TransDoubles> approach_t = kZeroTrans3;
};

static_assert(std::is_trivially_copyable_v<GraspTargetPOD>,
              "GraspTargetPOD must be trivially copyable for SeqLock<GraspTargetPOD>");

/// @brief Marshal @ref GraspTarget → @ref GraspTargetPOD via `memcpy` of the
///        underlying Eigen storage. Safe per Eigen's documented contract that
///        matrix/vector storage is contiguous plain `double[N]`.
inline GraspTargetPOD ToPOD(const GraspTarget& target) noexcept {
  GraspTargetPOD pod;
  std::memcpy(pod.grasp_rot.data(), target.grasp_pose.rotation().data(), sizeof(pod.grasp_rot));
  std::memcpy(pod.grasp_t.data(), target.grasp_pose.translation().data(), sizeof(pod.grasp_t));
  std::memcpy(pod.pregrasp_rot.data(), target.pregrasp_pose.rotation().data(),
              sizeof(pod.pregrasp_rot));
  std::memcpy(pod.pregrasp_t.data(), target.pregrasp_pose.translation().data(),
              sizeof(pod.pregrasp_t));
  std::memcpy(pod.approach_rot.data(), target.approach_start.rotation().data(),
              sizeof(pod.approach_rot));
  std::memcpy(pod.approach_t.data(), target.approach_start.translation().data(),
              sizeof(pod.approach_t));
  return pod;
}

/// @brief Unmarshal @ref GraspTargetPOD → @ref GraspTarget. Inverse of
///        @ref ToPOD; allocation-free.
inline GraspTarget FromPOD(const GraspTargetPOD& pod) noexcept {
  GraspTarget target;
  std::memcpy(target.grasp_pose.rotation().data(), pod.grasp_rot.data(), sizeof(pod.grasp_rot));
  std::memcpy(target.grasp_pose.translation().data(), pod.grasp_t.data(), sizeof(pod.grasp_t));
  std::memcpy(target.pregrasp_pose.rotation().data(), pod.pregrasp_rot.data(),
              sizeof(pod.pregrasp_rot));
  std::memcpy(target.pregrasp_pose.translation().data(), pod.pregrasp_t.data(),
              sizeof(pod.pregrasp_t));
  std::memcpy(target.approach_start.rotation().data(), pod.approach_rot.data(),
              sizeof(pod.approach_rot));
  std::memcpy(target.approach_start.translation().data(), pod.approach_t.data(),
              sizeof(pod.approach_t));
  return target;
}

}  // namespace integrated_bringup::phase

#endif  // UR5E_BRINGUP_CONTROLLERS_WBC_GRASP_TARGET_HPP_
