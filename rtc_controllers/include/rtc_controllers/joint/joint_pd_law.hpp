// ── Joint-space PD control law (#236 S1) ────────────────────────────────────
// The per-channel command of a joint-space PD regulator with optional velocity
// feedforward and gravity/Coriolis compensation:
//
//     e[i]   = q_ref[i] − q[i]
//     ė[i]   = (e[i] − e_prev[i]) / dt          (backward finite difference)
//     u[i]   = Kp[i]·e[i] + Kd[i]·ė[i]
//              + g(q)[i] + (C(q,q̇)·q̇)[i]        (torque lane, if enabled)
//              + v_ref[i]                       (position/velocity lane)
//
// and nothing else. The caller owns the model (g and C·v arrive already
// scattered to DEVICE channel order), the reference (q_ref, v_ref arrive as
// plain numbers), the mailbox, the E-STOP policy, the telemetry lanes and the
// output clamp.
//
// ── Why there is no trajectory here ─────────────────────────────────────────
// A control algorithm does not need a trajectory: the law consumes a reference
// SAMPLE, not a generator. Which trajectory feeds which algorithm — and how
// many — is a STRUCTURAL decision and belongs to the integration layer, where
// it already lives: integrated_bringup's demo_joint_controller drives this same
// law from TWO JointSpaceTrajectory instances (arm + hand) and demo_wbc_
// controller from three, a composition a trajectory-owning core could not
// express. The duration heuristic (max_dist / trajectory_speed) is trajectory
// parameterisation for the same reason, and `trajectory_speed` is already
// declared per-trajectory up there (robot_trajectory_speed / hand_trajectory_
// speed), not once per control law.
//
// ── Why e_prev is an in/out span and not a member ───────────────────────────
// The error history is SHARED with the E-STOP hold policy, which regulates to
// safe_position with the same Kp/Kd and writes the same buffer
// (joint_pd_controller.cpp ComputeEstop). That policy is an integration-layer
// concern, so the buffer belongs to the caller; owning it here would split one
// piece of state across two layers and hand every future binding a two-owner
// reset contract to get right. With the reference and the error history both
// caller-owned this law is STATELESS — a free function, like
// compliance/impedance_law.hpp.
//
// ── Extraction note (#236 S1) ───────────────────────────────────────────────
// This expression lived inline in JointPDController::Compute() (0f8da61c,
// joint_pd_controller.cpp:258-284). It was lifted here verbatim — same
// operation order, same association, same branch structure — so the migration
// is bit-for-bit inert; JointPdLaw.MatchesThePreExtractionInlineFormBitwise
// pins that against a literal copy of the pre-extraction form, and
// JointPdLaw.CoreDrivenShimMatchesTheAdapterBitwise pins it against the live
// adapter while the adapter still exists. Preserve the association if you touch
// it: `kp·e + kd·ė` accumulated in place is NOT bitwise the same as a
// re-grouped equivalent.
#pragma once

#include <rtc_base/types/types.hpp>

#include <algorithm>
#include <cstddef>
#include <span>

namespace rtc::joint {

/// Per-channel PD gains and the two dynamics-compensation switches, as a
/// non-owning view: the adapter holds these inside a SeqLock'd POD and the
/// binding will hold them wherever it likes, so the law borrows rather than
/// copies 2·kMaxRobotDOF doubles onto the RT stack every tick.
struct JointPdGainsView {
  std::span<const double> kp;  ///< stiffness, device channel order [N·m/rad | 1/s]
  std::span<const double> kd;  ///< damping, device channel order
  /// g(q) feedforward. N·m — TORQUE LANE ONLY; ignored on a position/velocity
  /// command, where mixing it would add newton-metres to radians (issue #172).
  bool enable_gravity_compensation{false};
  /// C(q,q̇)·q̇ feedforward. Same units, same lane restriction.
  bool enable_coriolis_compensation{false};
};

/// The per-tick numeric inputs, all in DEVICE channel order. Grouped in a
/// struct rather than passed positionally because five same-typed spans in a
/// row is a swap waiting to happen.
struct JointPdInputs {
  std::span<const double> q;         ///< measured joint positions [rad]
  std::span<const double> ref_pos;   ///< reference positions — a trajectory SAMPLE [rad]
  std::span<const double> ref_vel;   ///< reference velocities, feedforward [rad/s]
  std::span<const double> gravity;   ///< g(q) [N·m]; unread unless the gain flag is set
  std::span<const double> coriolis;  ///< C(q,q̇)·q̇ [N·m]; unread unless the gain flag is set
};

/// Write the PD command for channels [0, nc0) into `commands`.
///
/// @param k           gains + compensation switches
/// @param in          measured state, reference sample, dynamics terms
/// @param dt          tick period [s]; must be > 0 (see the NUM-2 note below)
/// @param nq          channels backed by a model joint — the caller's
///                    min(num_channels, nv) bound (issue #172 OOB)
/// @param nc0         channels the device actually has
/// @param cmd         command lane; selects the dynamics-vs-feedforward branch
///                    AND the tail policy
/// @param prev_error  IN/OUT error history, updated to this tick's error on
///                    channels [0, nq)
/// @param commands    OUT, written on [0, nc0)
///
/// Channels in [nq, nc0) have no model joint and get the TAIL POLICY, which is
/// part of the contract and not a fallback: zero torque (passive) in the torque
/// lane, hold-current-position in a position lane — emitting a fresh zero there
/// would command "go to the origin" (joint_pd_controller.cpp:454-456 is the
/// same policy on the E-STOP path).
///
/// NUM-2: ė divides by dt. The guard travels with the law — dt ≤ 0 zeroes the
/// derivative term ONLY, leaving Kp·e intact. It does not skip the write: an
/// unwritten command array is a fresh zero, which is passive in torque but
/// "go to the origin" in a position lane. On the reachable domain (dt > 0,
/// which the caller also enforces with its default-dt fallback) this branch is
/// bit-inert.
///
/// RT-safe: noexcept, no heap, fixed loops. Every span length participates in
/// the bound below, so a short or empty input degrades to the tail policy on
/// the affected channels instead of reading out of range; on a correctly sized
/// caller the bound is a no-op.
inline void ComputeJointPdCommand(const JointPdGainsView& k, const JointPdInputs& in, double dt,
                                  std::size_t nq, std::size_t nc0, CommandType cmd,
                                  std::span<double> prev_error,
                                  std::span<double> commands) noexcept {
  nc0 = std::min(nc0, commands.size());
  // Bound by every span the branches below will actually READ — the two
  // dynamics terms only when their flag is set, ref_vel only off the torque
  // lane — so an intentionally empty span (e.g. gravity off) never shrinks nq.
  std::size_t n = std::min(
      {nq, nc0, in.q.size(), in.ref_pos.size(), prev_error.size(), k.kp.size(), k.kd.size()});
  if (cmd == CommandType::kTorque) {
    if (k.enable_gravity_compensation)
      n = std::min(n, in.gravity.size());
    if (k.enable_coriolis_compensation)
      n = std::min(n, in.coriolis.size());
  } else {
    n = std::min(n, in.ref_vel.size());
  }

  for (std::size_t i = 0; i < n; ++i) {
    const double e = in.ref_pos[i] - in.q[i];
    const double de = (dt > 0.0) ? ((e - prev_error[i]) / dt) : 0.0;

    commands[i] = k.kp[i] * e + k.kd[i] * de;
    if (cmd == CommandType::kTorque) {
      // Torque mode (N·m): PD torque + optional gravity/Coriolis feedforward.
      if (k.enable_gravity_compensation) {
        commands[i] += in.gravity[i];
      }
      if (k.enable_coriolis_compensation) {
        commands[i] += in.coriolis[i];
      }
    } else {
      // Position/velocity mode: reference velocity feedforward only. N·m
      // dynamics terms (g, C·v) must NOT be mixed into a non-torque command
      // (issue #172): they are gated out here and rejected at config load.
      commands[i] += in.ref_vel[i];
    }

    prev_error[i] = e;
  }
  // Channels past the model DOF (nc0 > nv) carry no PD/dynamics term: zero
  // torque (passive) or hold the current position in a non-torque command.
  for (std::size_t i = n; i < nc0; ++i) {
    commands[i] = (cmd == CommandType::kTorque) ? 0.0 : ((i < in.q.size()) ? in.q[i] : 0.0);
  }
}

}  // namespace rtc::joint
