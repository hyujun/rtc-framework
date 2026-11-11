// ── Dynamics feedforward law (pd_feedforward lane) ──────────────────────────
// The per-channel feedforward torque a position-servoed device receives on top
// of its own PD:
//
//     τ_ff[i] = clamp(k_g · g(q)[i] + k_c · (C(q,q̇)·q̇)[i] + k_M · (M(q)·q̈_ref)[i],
//                     −τ_max[i], +τ_max[i])
//
// and nothing else. The caller owns the model (g, C·q̇ and M·q̈_ref arrive
// already in DEVICE channel order — for a closed-chain hand that is the reduced
// g_a = Gᵀ g and M_a = Gᵀ M G, projected by the caller), the reference (the
// position travels on the command lane; its acceleration q̈_ref is multiplied
// into the inertia term by the caller, so the law sees one vector per term),
// the per-channel torque bound, the enable switch and the hold policy on ticks
// the model could not be evaluated.
//
// ── Why a second law next to joint_pd_law.hpp ───────────────────────────────
// ComputeJointPdCommand adds g / C·q̇ into a TORQUE command and refuses them on
// a position command (issue #172): on that lane the servo is the controller's.
// Here the servo is the DEVICE's (a firmware PD, a simulator <position>
// actuator) and the controller contributes only the feedforward overlay the
// `pd_feedforward` wire carries (rtc_msgs/JointCommand.feedforward, N·m). The
// two lanes have different units contracts and different clamps, so folding a
// "feedforward-only" branch into the PD law would make its #172 guard
// conditional — the guard that keeps N·m out of a radian command.
//
// ── Policy choices, and where the alternatives went ──────────────────────────
//  - Clamp per channel, by magnitude (sign kept). The bound is the smaller of
//    the YAML `max_abs` and the device torque rating, which the caller resolves
//    once at configure — the law sees one span.
//  - Non-finite in ANY term read → every channel 0 and `finite == false`. Not
//    per-channel: a NaN in one joint's gravity means the model evaluation is
//    not trustworthy, and a hand whose other fingers keep their feedforward
//    while one goes passive is a torque step nobody commanded. The caller
//    decides whether to hold its last finite vector instead; the law does not
//    keep state (it is a free function, like joint_pd_law.hpp).
//  - Channels past the model terms (a device wider than its model, or a
//    `max_abs` span shorter than nc) get 0: passive is the only tail that is
//    safe on a torque overlay. Every span length participates in the bound, so
//    a short input never reads out of range.
//  - The Coriolis and inertia terms are SLOTS: a gain of 0 leaves that span
//    unread (an empty span is fine), so a binding that ships gravity alone
//    passes nothing and pays nothing. Every term shares the one clamp and the
//    one finiteness rule — the bound is on what reaches the wire, not per term.
//  - The inertia term takes the PRODUCT M·q̈_ref, not M and q̈_ref: the matrix
//    product is the binding's (it owns the matrix layout and the hold of M),
//    and the law stays a per-channel loop with no matrix in its signature.
#pragma once

#include <rtc_base/types/types.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

namespace rtc::joint {

/// Gains and the per-channel bound, as a non-owning view (the binding keeps the
/// bound in a fixed array it resolved at configure; the gains in a SeqLock'd
/// POD it loads once per tick).
struct DynamicsFeedforwardGainsView {
  double gravity_gain{1.0};   ///< k_g, dimensionless. 0 switches the term off
  double coriolis_gain{0.0};  ///< k_c, dimensionless. 0 leaves `coriolis` unread
  double inertia_gain{0.0};   ///< k_M, dimensionless. 0 leaves `inertia` unread
  /// τ_max per channel [N·m], device channel order; applied by magnitude. A
  /// channel past the end of this span gets 0 (tail policy).
  std::span<const double> max_abs{};
};

/// The per-tick model terms, DEVICE channel order, [N·m].
struct DynamicsFeedforwardInputs {
  std::span<const double> gravity{};   ///< g(q); for a closed chain the reduced g_a
  std::span<const double> coriolis{};  ///< C(q,q̇)·q̇; unread unless coriolis_gain ≠ 0
  /// M(q)·q̈_ref (the reference acceleration through the model inertia; for a
  /// closed chain the reduced M_a); unread unless inertia_gain ≠ 0.
  std::span<const double> inertia{};
};

/// Width of DynamicsFeedforwardResult::clamped — one bit per device channel.
inline constexpr std::size_t kClampMaskBits = 64;
static_assert(kClampMaskBits >= static_cast<std::size_t>(kMaxDeviceChannels),
              "the clamp mask must cover every device channel");

/// What one call did, for the caller's telemetry.
struct DynamicsFeedforwardResult {
  /// False when a term that was read was NaN / Inf: `tau_ff[0, nc)` is then all
  /// zero and `clamped` is 0.
  bool finite{true};
  /// Bit i set: channel i was clamped to ±max_abs[i] (one bit per device
  /// channel, kClampMaskBits wide).
  std::uint64_t clamped{0};
  /// Channels [0, written) carry a model term; [written, nc) are the tail
  /// zeros. On a correctly sized caller written == nc.
  std::size_t written{0};
};

/// Write the feedforward torque for channels [0, nc) into `tau_ff`.
///
/// @param k       gains + per-channel bound
/// @param in      model terms, device channel order
/// @param nc      channels the device has (bounded by tau_ff.size())
/// @param tau_ff  OUT, written on [0, nc) — every channel, every call
///
/// RT-safe: noexcept, no heap, two fixed loops (finiteness first, then the
/// law — so a non-finite term found at channel 7 cannot leave channels 0..6
/// already written with a value the all-or-nothing rule retracts).
inline DynamicsFeedforwardResult ComputeDynamicsFeedforward(const DynamicsFeedforwardGainsView& k,
                                                            const DynamicsFeedforwardInputs& in,
                                                            std::size_t nc,
                                                            std::span<double> tau_ff) noexcept {
  DynamicsFeedforwardResult r;
  nc = std::min(nc, tau_ff.size());
  const bool use_coriolis = (k.coriolis_gain != 0.0);
  const bool use_inertia = (k.inertia_gain != 0.0);
  // Bound by every span the law will READ — a slot's span only when its gain
  // is set, so an intentionally empty span never shrinks n.
  std::size_t n = std::min({nc, in.gravity.size(), k.max_abs.size()});
  if (use_coriolis) {
    n = std::min(n, in.coriolis.size());
  }
  if (use_inertia) {
    n = std::min(n, in.inertia.size());
  }

  bool finite = std::isfinite(k.gravity_gain) &&
                (!use_coriolis || std::isfinite(k.coriolis_gain)) &&
                (!use_inertia || std::isfinite(k.inertia_gain));
  for (std::size_t i = 0; i < n && finite; ++i) {
    finite = std::isfinite(in.gravity[i]) && std::isfinite(k.max_abs[i]) &&
             (!use_coriolis || std::isfinite(in.coriolis[i])) &&
             (!use_inertia || std::isfinite(in.inertia[i]));
  }
  if (!finite) {
    for (std::size_t i = 0; i < nc; ++i) {
      tau_ff[i] = 0.0;
    }
    r.finite = false;
    return r;
  }

  for (std::size_t i = 0; i < n; ++i) {
    double tau = k.gravity_gain * in.gravity[i];
    if (use_coriolis) {
      tau += k.coriolis_gain * in.coriolis[i];
    }
    if (use_inertia) {
      tau += k.inertia_gain * in.inertia[i];
    }
    // A negative bound is read as 0 (passive) rather than inverting the clamp.
    const double lim = std::max(0.0, k.max_abs[i]);
    if (std::abs(tau) > lim) {
      tau = std::copysign(lim, tau);
      if (i < kClampMaskBits) {
        r.clamped |= (std::uint64_t{1} << i);
      }
    }
    tau_ff[i] = tau;
  }
  for (std::size_t i = n; i < nc; ++i) {
    tau_ff[i] = 0.0;
  }
  r.written = n;
  return r;
}

}  // namespace rtc::joint
