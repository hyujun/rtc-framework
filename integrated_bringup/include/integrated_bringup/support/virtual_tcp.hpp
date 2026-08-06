// ── Virtual TCP computation shared between Demo{Joint,Task}Controller ────────
#pragma once

#include <Eigen/Core>
#include <pinocchio/math.hpp>
#include <pinocchio/spatial.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

namespace integrated_bringup {

/// Virtual TCP computation mode for fingertip-based control point.
enum class VirtualTcpMode : uint8_t {
  kDisabled,  ///< Use tool0 as control point (default)
  kCentroid,  ///< Fingertip position centroid
  kWeighted,  ///< Contact-force weighted fingertip centroid
  kConstant   ///< Fixed offset from TCP frame (YAML configured)
};

/// YAML-configurable virtual TCP parameters.
struct VirtualTcpConfig {
  VirtualTcpMode mode{VirtualTcpMode::kDisabled};
  std::array<double, 3> offset{{0.0, 0.0, 0.0}};  ///< Constant mode: [x,y,z] in TCP frame [m]
  std::array<double, 3> orientation{
      {0.0, 0.0, 0.0}};  ///< RPY [rad] orientation of vtcp in TCP frame
};

/// Identity of the frame a task goal is expressed in.
///
/// `is_vtcp` alone does not identify the control frame. In centroid/weighted
/// mode the virtual TCP is the mean over the *participating* fingertips, so a
/// fingertip entering or leaving that set moves the control point while
/// `vtcp_valid_` stays true — a test that only watches the valid edge misses
/// that case (mixed hands; p1b keeps all four tips downstream of the loop and
/// so never hits it).
struct ControlFrameId {
  bool is_vtcp{false};          ///< Control point is the virtual TCP, not tool0
  std::uint8_t member_mask{0};  ///< bit f = fingertip f participates (vtcp only)
  bool bound{false};            ///< member_mask has been bound to a real frame
};

/// What ComputeControl owes the current goal on this tick — see
/// ClassifyFrameTransition.
enum class FrameTransition : std::uint8_t {
  kNone,            ///< Frames agree — run the existing CLIK unchanged
  kReseed,          ///< Hold seed: re-seed the goal at the current control pose
  kHoldExternal,    ///< External goal: hold the arm, keep the goal pending
  kExpireExternal,  ///< External goal waited past the bound: expire it, hold
  kReplanExternal,  ///< Same frame kind, control point jumped: replan from it
  kBindExternal,    ///< First tick the external goal's frame can be bound
};

/// Decide what to do with the current task goal given this tick's control frame.
///
/// The single invariant this enforces: *run CLIK only when the frame the goal
/// was authored in matches the frame being controlled this tick.* Otherwise the
/// pose difference between the two frames is consumed as task error — which is
/// exactly the arm jump seen when a closed-chain hand's FK walk-in ends and the
/// control point moves from tool0 to the virtual TCP mid-flight.
///
/// Kept a pure free function so the six transitions can be tested exhaustively.
/// The controller fixture can only reach four of them: its hand FK is serial, so
/// every fingertip is valid from tick 1 and `member_mask` never moves. The
/// controller tests pin the *wiring*; this function pins the *law*.
///
/// @param target       Frame the current goal was authored in
/// @param current      Frame this tick controls in
/// @param is_hold_seed Goal is still the self-init hold seed (no external goal)
/// @param wait_ticks   Consecutive ticks an external goal has been held so far
/// @param wait_bound   Tick budget before a held external goal expires
[[nodiscard]] inline FrameTransition ClassifyFrameTransition(ControlFrameId target,
                                                             ControlFrameId current,
                                                             bool is_hold_seed,
                                                             std::uint32_t wait_ticks,
                                                             std::uint32_t wait_bound) noexcept {
  const bool kind_differs = target.is_vtcp != current.is_vtcp;
  // tool0 has no fingertip membership, so `member_mask` is part of the frame's
  // identity only while the control point IS the virtual TCP. Comparing it
  // unconditionally would let a stale mask fabricate a transition on a
  // `virtual_tcp_mode: disabled` controller, whose behaviour must not change.
  const bool members_differ = current.is_vtcp && target.member_mask != current.member_mask;

  if (is_hold_seed) {
    // The hold seed is ours to move. Re-seeding it at the current control pose
    // leaves zero task error, so the arm stays put through any transition —
    // including a frame that flickers valid/invalid every tick, which is why no
    // latch or hysteresis is needed anywhere else in this path.
    return (kind_differs || members_differ) ? FrameTransition::kReseed : FrameTransition::kNone;
  }

  // External goal: authored in the controller's *intended* frame. A frame-kind
  // mismatch means the control point is not the one the goal speaks about, so
  // hold rather than silently reinterpreting the goal in the frame that happens
  // to be active — that reinterpretation is the defect being fixed here.
  if (kind_differs) {
    return (wait_ticks < wait_bound) ? FrameTransition::kHoldExternal
                                     : FrameTransition::kExpireExternal;
  }
  // Same frame kind. The participating set could not be known off-RT when the
  // goal was authored, so bind it the first tick the frames agree.
  if (!target.bound) {
    return FrameTransition::kBindExternal;
  }
  if (members_differ) {
    // The goal is still valid — only the control point moved. Replan from the
    // new one instead of holding: the goal survives and there is no jerk.
    return FrameTransition::kReplanExternal;
  }
  return FrameTransition::kNone;
}

/// Per-fingertip input for virtual TCP computation.
struct FingertipVtcpInput {
  Eigen::Vector3d position_in_tcp{Eigen::Vector3d::Zero()};  ///< Position in TCP/hand frame
  double force_magnitude{0.0};  ///< Force magnitude [N] (for weighted mode)
  bool active{false};           ///< true if this fingertip has valid FK data
};

/// Result of virtual TCP computation.
struct VirtualTcpResult {
  pinocchio::SE3 T_tcp_vtcp{pinocchio::SE3::Identity()};  ///< TCP -> virtual TCP transform
  pinocchio::SE3 world_pose{pinocchio::SE3::Identity()};  ///< World-frame virtual TCP pose
  bool valid{false};
};

/// Compute the virtual TCP pose from fingertip data.
///
/// @param config    Virtual TCP configuration (mode, offset, orientation)
/// @param T_base_tcp  Current base -> TCP (tool0) transform
/// @param fingertips  Per-fingertip positions in TCP frame + force data
/// @return  VirtualTcpResult with valid=true on success
[[nodiscard]] inline VirtualTcpResult ComputeVirtualTcp(
    const VirtualTcpConfig& config, const pinocchio::SE3& T_base_tcp,
    std::span<const FingertipVtcpInput> fingertips) noexcept {
  VirtualTcpResult result;
  if (config.mode == VirtualTcpMode::kDisabled)
    return result;

  // Pre-compute vtcp orientation from RPY config (applied to all modes)
  const Eigen::Matrix3d R_tcp_vtcp = pinocchio::rpy::rpyToMatrix(
      config.orientation[0], config.orientation[1], config.orientation[2]);

  switch (config.mode) {
    case VirtualTcpMode::kCentroid: {
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      int count = 0;
      for (const auto& ft : fingertips) {
        if (ft.active) {
          sum += ft.position_in_tcp;
          ++count;
        }
      }
      if (count == 0)
        return result;
      result.T_tcp_vtcp.translation() = sum / static_cast<double>(count);
      result.T_tcp_vtcp.rotation() = R_tcp_vtcp;
      break;
    }
    case VirtualTcpMode::kWeighted: {
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      double total_weight = 0.0;
      for (const auto& ft : fingertips) {
        if (!ft.active)
          continue;
        const double w = 1.0 + ft.force_magnitude;  // base weight ensures non-zero
        sum += w * ft.position_in_tcp;
        total_weight += w;
      }
      if (total_weight <= 0.0)
        return result;
      result.T_tcp_vtcp.translation() = sum / total_weight;
      result.T_tcp_vtcp.rotation() = R_tcp_vtcp;
      break;
    }
    case VirtualTcpMode::kConstant: {
      result.T_tcp_vtcp.translation() =
          Eigen::Vector3d(config.offset[0], config.offset[1], config.offset[2]);
      result.T_tcp_vtcp.rotation() = R_tcp_vtcp;
      break;
    }
    default:
      return result;
  }

  result.world_pose = T_base_tcp.act(result.T_tcp_vtcp);

  // Finiteness gate. Every guard above is a MAGNITUDE test and a NaN fails all
  // of them: `total_weight <= 0.0` is false for a NaN weight (one non-finite
  // force_magnitude poisons the whole sum), and a non-finite position_in_tcp
  // never reaches the centroid `count` at all. Without this the result leaves
  // with valid=true and a NaN pose, which the caller latches as its control
  // point. Judging the published value once, here, covers all three modes plus
  // a non-finite T_base_tcp with one branch; invalid keeps the caller's
  // existing hold-last-good behaviour, and the reset keeps the "invalid ⇒
  // untouched identity" shape the disabled/inactive paths already return.
  if (!result.T_tcp_vtcp.translation().allFinite() || !result.T_tcp_vtcp.rotation().allFinite() ||
      !result.world_pose.translation().allFinite() || !result.world_pose.rotation().allFinite()) {
    result.T_tcp_vtcp = pinocchio::SE3::Identity();
    result.world_pose = pinocchio::SE3::Identity();
    return result;
  }

  result.valid = true;
  return result;
}

}  // namespace integrated_bringup
