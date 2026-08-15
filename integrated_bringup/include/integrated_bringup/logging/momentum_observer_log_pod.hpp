#ifndef INTEGRATED_BRINGUP_LOGGING_MOMENTUM_OBSERVER_LOG_POD_HPP_
#define INTEGRATED_BRINGUP_LOGGING_MOMENTUM_OBSERVER_LOG_POD_HPP_

// Per-controller generalized-momentum observer log POD (#135 Layer 1b). One
// per-tick row (`momentum_observer.csv`) shared by the three demo controllers
// (joint / task / wbc) — the observer is a single shared component
// (MomentumObserverWiring), so the channel shape is controller-independent.
//
// PATH A: no rtc_msgs/.msg for this channel, and that is a decision rather than
// a shortcut (#135 D12). What Layer 1 produces is the joint-space residual `r`;
// the mass / CoM / confidence that a `PayloadEstimate` topic would exist to
// carry are Layer 2A's output and do not exist yet. Publishing that message now
// would ship fields pinned at zero for the life of the layer. The residual gets
// a CSV channel instead — the same call the in-plane pull estimator made (#167)
// — and the wire surface arrives with Layer 2A, in one msgs ABI event instead
// of two.
//
// This file mirrors DeviceStateLogPod's shape for the per-joint block: a
// fixed-size array with an integrated_bringup-private cap, and a runtime column
// count taken from the caller's joint-name list at registration.

#include "rtc_controllers/estimation/momentum_observer.hpp"

#include <algorithm>
#include <cmath>
#include <array>
#include <cstddef>
#include <cstdint>
#include <ostream>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>

namespace integrated_bringup {

// Closed-set YAML `msg_type` id and the fixed instance key for this channel —
// referenced by the LoadConfig validators, the log registration dispatch, and
// the lifecycle wiring so the string lives in exactly one place.
inline constexpr std::string_view kMomentumObserverLogMsgType =
    "integrated_bringup/MomentumObserverLog";
inline constexpr std::string_view kMomentumObserverLogInstance = "momentum_observer";

struct MomentumObserverLogPod {
  // Arm-side cap. The observer's own bound is kMaxObserverDof (== the
  // DeviceState channel cap, 64); this channel is narrower on purpose because
  // the surface is one arm sub-model and a 64-wide CSV of mostly-empty columns
  // is worse than a documented ceiling. Same convention, and the same
  // truncation behaviour, as DeviceStateLogPod::kMaxJoints.
  static constexpr std::size_t kMaxArmJoints = 16;

  // ── Timestamp (CM-provided, session-relative) ─────────────────────────────
  double t_relative_s{0.0};

  // CM RT-loop tick index (ControllerState::iteration). Rows are pushed one per
  // tick — including held and E-STOP ticks, which log valid=0 rather than
  // nothing — so a jump here is a *dropped* row (SPSC ring overflow), never a
  // tick the controller chose not to log. A wiring that was never configured
  // produces no file at all.
  std::uint64_t tick{0};

  // Residual r [N·m] in DEVICE order, `n_joints` entries valid. Frozen at its
  // last accepted value on any row with valid=0, which is the point: a zero
  // there would assert "no external torque" on a tick whose inputs were
  // refused.
  std::array<double, kMaxArmJoints> residual{};

  // ‖r‖∞ over the valid entries. Derivable from the columns above, carried
  // anyway because it is the quantity the acceptance criterion is stated in
  // ("no load ⇒ ‖r‖∞ below tol") and a plot of one column beats a max() over
  // a variable-width block.
  double residual_inf_norm{0.0};

  // Valid ticks since the momentum reference was last captured. While this is
  // small the residual is still converging from zero after a gap, so a small
  // ‖r‖ does not yet mean "no external torque" — a reader that must not
  // confuse the two discards roughly the first 3/K_I worth of ticks.
  std::uint32_t ticks_since_seed{0};

  // rtc::estimation::MomentumInvalidReason — the first gate that failed, or
  // kNone iff valid. Distinguishes the causes that otherwise collapse into a
  // single valid=0: a closed lane gate (kHeld, incl. E-STOP) reads differently
  // from a non-finite input or a stalled clock.
  std::uint8_t invalid_reason{0};

  bool valid{false};
};

static_assert(std::is_trivially_copyable_v<MomentumObserverLogPod>,
              "MomentumObserverLogPod must be trivially copyable for SPSC ring");

static_assert(sizeof(rtc::estimation::MomentumInvalidReason) == sizeof(std::uint8_t),
              "MomentumObserverLogPod::invalid_reason mirrors MomentumInvalidReason — widen it "
              "with the enum's underlying type");

/// Number of per-joint columns this channel will emit for `joint_names`.
/// Shared by the header writer and the fill so the two cannot disagree.
[[nodiscard]] inline std::size_t MomentumObserverLogColumns(
    std::span<const std::string> joint_names) noexcept {
  return std::min(joint_names.size(), MomentumObserverLogPod::kMaxArmJoints);
}

/// Emit the CSV header. `joint_names` is the arm device's joint order — the
/// same order the residual columns carry — so a stored file decodes without
/// that run's ROS log. The logger appends '\n'.
inline void WriteMomentumObserverLogHeader(std::ostream& os,
                                           std::span<const std::string> joint_names) {
  const std::size_t n = MomentumObserverLogColumns(joint_names);
  os << "t_relative_s,tick";
  for (std::size_t i = 0; i < n; ++i) {
    os << ",r_" << joint_names[i];
  }
  os << ",residual_inf_norm,valid,invalid_reason,ticks_since_seed";
}

/// Emit one row. `num_columns` MUST be the value the header was built with —
/// MomentumObserverLogColumns(joint_names) — not something the POD carries: a
/// row sized from the POD could disagree with the header and silently shift
/// every column after the residual block. The registration passes the same
/// list to both writers for that reason (the fix GraspDiagLog already carries).
/// The logger appends '\n' + flush.
inline void WriteMomentumObserverLogRow(std::ostream& os, const MomentumObserverLogPod& p,
                                        std::size_t num_columns) {
  const std::size_t n = std::min(num_columns, MomentumObserverLogPod::kMaxArmJoints);
  os << p.t_relative_s;
  os << ',' << p.tick;
  for (std::size_t i = 0; i < n; ++i) {
    os << ',' << p.residual[i];
  }
  os << ',' << p.residual_inf_norm;
  os << ',' << (p.valid ? 1 : 0);
  os << ',' << static_cast<unsigned>(p.invalid_reason);
  os << ',' << p.ticks_since_seed;
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_LOGGING_MOMENTUM_OBSERVER_LOG_POD_HPP_
