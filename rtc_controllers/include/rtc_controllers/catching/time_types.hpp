// ── Time axes of the catching core (dynamic_catching D-2, plan §3) ───────────
// Three strong types over one clock (absolute steady nanoseconds):
//
//   BallTime — a physical instant of the BALL: t_c, t_cmd, every predicted
//              trajectory sample.
//   NowReal  — the per-tick steady "now", measured (never tick count × dt).
//   NowLead  — NowReal + T_arm: the instant at which an arm command issued now
//              is realised.
//
// The point of the types is that every decision is pinned to the axis plan §3
// assigns it, at compile time. There is deliberately NO mixed-type comparison
// operator: a generic `NowReal < BallTime` would let a sampling decision be
// written on the real axis and still compile, which is exactly the bug that
// T_arm = 0 fixtures hide. Instead each §3 row is a named function taking the
// one now-type it is defined on:
//
//   | decision                              | axis     | function            |
//   |---------------------------------------|----------|---------------------|
//   | sampling, γ profile, reference        | NowLead  | LeadSecondsUntil    |
//   | CLOSING→DECEL                         | NowLead  | DecelDue            |
//   | trajectory horizon exhausted          | NowLead  | HorizonExceeded     |
//   | APPROACH→COMMITTED                    | NowReal  | CommitDue           |
//   | COMMITTED→CLOSING, hand close         | NowReal  | HandCommandDue      |
//   | hand preshape                         | NowReal  | PreshapeDue         |
//   | contact window [t_cmd, t_c + T_conf]  | NowReal  | InContactWindow     |
//   | message age / stale                   | NowReal  | AgeNs               |
//
// Message age is measured between two steady RECEIVE instants and never from a
// BallTime or a header stamp (repo clock rule; AgeNs takes only NowReal).
//
// All types are trivially copyable (they ride SeqLock payloads, L0 §5.2).
// Relative double seconds are formed only here, at the numeric-core boundary,
// and only as a difference on one axis.
#pragma once

#include <compare>
#include <cstdint>
#include <type_traits>

namespace rtc::catching {

inline constexpr double kNsToS = 1e-9;

/// Physical instant of the ball [absolute steady ns].
struct BallTime {
  std::int64_t ns{0};
  friend constexpr auto operator<=>(const BallTime&, const BallTime&) noexcept = default;
};

/// Measured steady now of the current tick (also used for steady receive
/// instants) [absolute steady ns].
struct NowReal {
  std::int64_t ns{0};
  friend constexpr auto operator<=>(const NowReal&, const NowReal&) noexcept = default;
};

/// now + T_arm [absolute steady ns].
struct NowLead {
  std::int64_t ns{0};
  friend constexpr auto operator<=>(const NowLead&, const NowLead&) noexcept = default;
};

static_assert(std::is_trivially_copyable_v<BallTime>);
static_assert(std::is_trivially_copyable_v<NowReal>);
static_assert(std::is_trivially_copyable_v<NowLead>);

/// The lead axis for this tick. `t_arm_ns` is the arm command latency T_arm
/// (≥ 0; the caller validates it at configure time).
[[nodiscard]] constexpr NowLead MakeNowLead(NowReal now, std::int64_t t_arm_ns) noexcept {
  return NowLead{now.ns + t_arm_ns};
}

/// Seconds from `a` to `b` on the ball axis (b − a). Both instants come from the
/// same clock, so the difference is origin-free.
[[nodiscard]] constexpr double SecondsBetween(BallTime a, BallTime b) noexcept {
  return static_cast<double>(b.ns - a.ns) * kNsToS;
}

/// Seconds from the lead now until ball instant `t` (t − now_lead). Negative once
/// `t` has passed on the lead axis. Used for sampling, γ profile and reference.
[[nodiscard]] constexpr double LeadSecondsUntil(NowLead now_lead, BallTime t) noexcept {
  return static_cast<double>(t.ns - now_lead.ns) * kNsToS;
}

/// CLOSING→DECEL: the arm command issued now is realised at or after t_c.
[[nodiscard]] constexpr bool DecelDue(NowLead now_lead, BallTime t_c) noexcept {
  return now_lead.ns >= t_c.ns;
}

/// Horizon warning: the lead axis has run past the last predicted sample. The
/// last sample itself is not extrapolation (L2 §4.6).
[[nodiscard]] constexpr bool HorizonExceeded(NowLead now_lead, BallTime last_sample) noexcept {
  return now_lead.ns > last_sample.ns;
}

/// APPROACH→COMMITTED: t_c − now ≤ T_freeze (real axis; T_freeze already
/// includes T_arm in its lower bound, plan §3).
[[nodiscard]] constexpr bool CommitDue(NowReal now, BallTime t_c,
                                       std::int64_t t_freeze_ns) noexcept {
  return t_c.ns - now.ns <= t_freeze_ns;
}

/// COMMITTED→CLOSING and the hand close command: now ≥ t_cmd. The hand has no
/// lead compensation, so this is the real axis.
[[nodiscard]] constexpr bool HandCommandDue(NowReal now, BallTime t_cmd) noexcept {
  return now.ns >= t_cmd.ns;
}

/// Hand preshape: now ≥ t_c − T_pre (real axis).
[[nodiscard]] constexpr bool PreshapeDue(NowReal now, BallTime t_c,
                                         std::int64_t t_pre_ns) noexcept {
  return now.ns >= t_c.ns - t_pre_ns;
}

/// Contact decision window [t_cmd, t_c + T_conf], closed at both ends (real axis).
[[nodiscard]] constexpr bool InContactWindow(NowReal now, BallTime t_cmd, BallTime t_c,
                                             std::int64_t t_conf_ns) noexcept {
  return now.ns >= t_cmd.ns && now.ns <= t_c.ns + t_conf_ns;
}

/// Message age: now_steady − recv_steady [ns]. Both operands are steady receive /
/// tick instants; a BallTime or header stamp cannot be passed here.
[[nodiscard]] constexpr std::int64_t AgeNs(NowReal now, NowReal recv_steady) noexcept {
  return now.ns - recv_steady.ns;
}

// ── D-2 (3): remote stamp → ball axis (E-1 recorded exception, S0.6) ─────────
// The receive callback converts the publisher's `header.stamp` ONCE:
//
//   t_ref_steady = recv_steady − (recv_wall − stamp)
//
// This is the only use of a header stamp in the catching core, and it is the
// recorded exception in agent_docs/invariants.md §Clock (approved 2026-09-19).
// Its conditions: (1) freshness / stale / watchdog use AgeNs() only, never this
// value; (2) a stamp more than `future_tol` in the future is rejected and
// counted — the conversion cannot be trusted; (3) sender and receiver share the
// host CLOCK_REALTIME or a verified PTP sync; (4) the origin delay
// recv_wall − stamp is published as a diagnostic so clock jumps are visible.
// A wall-clock jump still enters every deadline derived from t_ref (t_c, t_cmd)
// — that is the accepted cost of the exception, not something this function
// can detect.

enum class StampStatus : std::uint8_t {
  kOk = 0,
  kFutureStamp = 1,       // recv_wall − stamp < −future_tol
  kOverflow = 2,          // an intermediate did not fit in int64 (garbage stamp)
  kInvalidTolerance = 3,  // future_tol < 0
};

struct StampConversion {
  BallTime t_ref{};                 // valid only when status == kOk
  std::int64_t origin_delay_ns{0};  // recv_wall − stamp: diagnostic, also on reject
  StampStatus status{StampStatus::kOverflow};

  [[nodiscard]] constexpr bool IsOk() const noexcept { return status == StampStatus::kOk; }
};

/// Convert a remote wall-clock stamp to the ball axis (see block above).
/// `recv_steady` and `recv_wall_ns` must be taken together on arrival.
[[nodiscard]] constexpr StampConversion ConvertRemoteStamp(NowReal recv_steady,
                                                           std::int64_t recv_wall_ns,
                                                           std::int64_t stamp_wall_ns,
                                                           std::int64_t future_tol_ns) noexcept {
  StampConversion out{};
  if (future_tol_ns < 0) {
    out.status = StampStatus::kInvalidTolerance;
    return out;
  }
  std::int64_t delay = 0;
  if (__builtin_sub_overflow(recv_wall_ns, stamp_wall_ns, &delay)) {
    out.status = StampStatus::kOverflow;
    return out;
  }
  out.origin_delay_ns = delay;
  if (delay < -future_tol_ns) {
    out.status = StampStatus::kFutureStamp;
    return out;
  }
  std::int64_t t_ref = 0;
  if (__builtin_sub_overflow(recv_steady.ns, delay, &t_ref)) {
    out.status = StampStatus::kOverflow;
    return out;
  }
  out.t_ref = BallTime{t_ref};
  out.status = StampStatus::kOk;
  return out;
}

/// Ball instant of a trajectory sample: t_ref + horizon offset. Returns false on
/// int64 overflow (the sample must then be rejected).
[[nodiscard]] constexpr bool SampleBallTime(BallTime t_ref, std::int64_t horizon_ns,
                                            BallTime& out) noexcept {
  std::int64_t t = 0;
  if (__builtin_add_overflow(t_ref.ns, horizon_ns, &t))
    return false;
  out = BallTime{t};
  return true;
}

}  // namespace rtc::catching
