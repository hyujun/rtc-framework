#ifndef RTC_MPC_TYPES_WBC_REFERENCE_HPP_
#define RTC_MPC_TYPES_WBC_REFERENCE_HPP_

/// @file wbc_reference.hpp
/// @brief Trivially copyable reference handed from the MPC side to a
///        whole-body controller for one RT tick.
///
/// This is the **output contract** of `rtc_mpc` towards a downstream
/// whole-body QP: everything the controller needs for one tick, with the
/// provenance of the plan it came from, in a POD it can copy or SeqLock.
/// `rtc_mpc` does not know the consumer's own reference type — mapping into
/// it is the consumer's job.
///
/// Conventions (shared with @ref MPCSolution and the model handler):
/// - `q_ref`: pinocchio configuration. On a floating-base model the first
///   seven entries are the free-flyer `[p_xyz, quaternion_xyzw]`, then the
///   joint positions; on a fixed-base model the joint positions only.
/// - `v_ref` / `a_ref` / `u_fb_acc`: pinocchio tangent. On a floating-base
///   model the first six entries are the base twist / acceleration in the
///   base **LOCAL** frame (linear, then angular), then the joints.
/// - `a_ref` is the feedforward acceleration **without** Riccati feedback;
///   `u_fb_acc` is the Riccati acceleration feedback (gain scale applied) so
///   the consumer can fade or clamp it independently.
/// - `wrench_ref`: one block per contact frame, in the model handler's
///   contact-frame order, each of that frame's dimension (force, then torque
///   for a six-dimensional contact). Expressed on **world axes**, acting at
///   the contact frame origin — the kinodynamic input convention.
/// - `contact_mask`: one bit per contact frame, same order. No left/right
///   semantics (ARCH-1).
/// - Both stamps are in the consumer's clock (the one it hands to the
///   manager): `observation_stamp_ns` is when the state the plan started
///   from was observed, `receive_stamp_ns` is when the consumer acquired the
///   plan. Policy age is their difference; it is not a field so it cannot
///   go stale.
/// - No feedforward torque: a kinodynamic plan has none; the consumer's QP
///   produces torques.

#include "rtc_mpc/types/mpc_solution_types.hpp"

#include <array>
#include <cstdint>
#include <type_traits>

namespace rtc::mpc {

struct WbcReference {
  // ── References for this tick ────────────────────────────────────────
  std::array<double, kMaxNq> q_ref{};
  std::array<double, kMaxNv> v_ref{};
  std::array<double, kMaxNv> a_ref{};
  std::array<double, kMaxNv> u_fb_acc{};
  std::array<double, kMaxContactVars> wrench_ref{};
  uint32_t contact_mask{0};

  // ── Actual dimensions ───────────────────────────────────────────────
  int nq{0};
  int nv{0};
  int n_contact_vars{0};

  // ── Provenance (copied from the plan this tick was sampled from) ────
  uint64_t observation_stamp_ns{0};
  uint64_t observation_sequence{0};
  uint64_t receive_stamp_ns{0};
  uint64_t plan_id{0};
  uint32_t model_epoch{0};

  // ── Sampling status ─────────────────────────────────────────────────
  double progress{0.0};        ///< normalized position in the horizon, `[0, 1]`
  bool beyond_horizon{false};  ///< sampled past the last node (held)
  bool valid{false};           ///< false ⇒ the consumer must fall back
};

static_assert(std::is_trivially_copyable_v<WbcReference>,
              "WbcReference must be trivially copyable (SeqLock / memcpy contract)");

}  // namespace rtc::mpc

#endif  // RTC_MPC_TYPES_WBC_REFERENCE_HPP_
