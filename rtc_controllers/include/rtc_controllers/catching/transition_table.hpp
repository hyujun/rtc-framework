// ── L7 supervisor state machine as data (L7 §4.1/§4.2, S1.8 / L7.1) ─────────
// "전이는 (상태 × 사유) 표를 데이터로 둔다" — §4.1 and §4.2's two human-
// readable tables are merged into ONE machine table here, and the RT
// supervisor body (S7.2) is meant to be a thin driver over it: look up
// (current Mode, evaluated Reason) and, if a row exists, take the
// transition. This header ships that table plus a startup self-check
// (`CheckTransitionTableComplete`, run once at configure — not itself an RT
// hot-path concern, but written noexcept/heap-free like everything else here
// since nothing forces it onto the stack any harder).
//
// ── Ambiguity resolutions (documented per task instruction) ─────────────────
// The design doc's tables mix true (Mode × Reason) events (§4.2's abort
// codes) with prose-only exit conditions that have no Reason in the closed
// §5.1 enum ("유효 궤적 수신", "$t_c-now\le T_{freeze}$", …). Three choices
// had no single unambiguous reading in the doc text, so S1.8 makes them
// explicitly (§4.1 itself defers exactly this kind of call to "S1.8 전이표
// 구현" for the IDLE-homing question, which is the same class of decision):
//
//  1. `Reason::kNone` keys the single nominal forward advance out of every
//     state that has one (IDLE→ARMED, ARMED→TRACKING, TRACKING→APPROACH, …).
//     It is not "no reason happened" so much as "no FAILURE reason — the
//     state's own §4.1 entry condition for the next state was met". The
//     supervisor body decides when to query the table with kNone; this core
//     only stores where that query goes.
//
//  2. IDLE homing (§4.1 v0.4 note: "homing 을 IDLE 의 하위 단계로 둘지 별도
//     Mode 로 둘지는 S1.8 전이표 구현에서 정한다") stays inside `kIdle` —
//     the §5.1 Mode enum this table is pinned to has no separate homing
//     state, so introducing one here would diverge from the interface the
//     rest of L7 is written against. The pre-/post-validation split within
//     IDLE is invisible to this table; IDLE→ARMED fires once homing AND
//     §4.5 are both satisfied (again, the supervisor body's job).
//
//  3. ARMED's second exit ("§4.5 조건 위반 → IDLE") has no dedicated Reason
//     in §4.2 either. §4.5 condition 1 is "L0 파라미터 검증 통과(TBD 없음)"
//     — textually the same gate `PARAMS_TBD` already names for IDLE's entry
//     — so this table reuses `kParamsTbd` for ARMED→IDLE rather than
//     inventing a Reason absent from the closed enum. §4.2's own
//     "발생 가능 상태: IDLE" column is read as "where this is currently
//     OBSERVED", not a closed set, since it is not itself a completeness
//     constraint the doc places on Reason applicability. Revisit in S7.2 if
//     a dedicated readiness-lost Reason is added later.
//
// Every row below is grouped by its FROM state with an inline pointer to the
// doc line it encodes, so a future reconciliation with a design doc edit can
// find its counterpart without re-deriving this table from scratch.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace rtc::catching {

// ── §5.1 interface (verbatim: the enum lists are pinned 1:1 to §4.2's rows
// plus Mode/Outcome, per the design doc's own C++ sketch) ───────────────────

enum class Mode : std::uint8_t {
  kIdle,
  kArmed,
  kTracking,
  kApproach,
  kCommitted,
  kClosing,
  kDecel,
  kHold,
  kRetreat,
  kAbortSafe,
  kFault,
};
inline constexpr std::size_t kNumModes = 11;

enum class Reason : std::uint8_t {
  kNone,
  kBallStale,
  kBallStaleCommitted,
  kBallStaleLong,
  kTrackChanged,
  kHorizonExtrap,
  kPredInconsistent,
  kNoCatchablePlan,
  kPlanInvalid,
  kQpFailed,
  kRefSaturated,
  kJointConflict,
  kTrackErr,
  kAbortEscalated,
  kEstop,
  kFaultReset,
  kSpeedScaling,
  kClockUnhealthy,
  kParamsTbd,
  kHandTimeout,
  kTipStale,
};
inline constexpr std::size_t kNumReasons = 21;

enum class Outcome : std::uint8_t {
  kNone,
  kCaptured,
  kMissed,
  kUndetermined,
  kAborted,
};

/// All Mode values, for iteration in the completeness check and in tests.
inline constexpr std::array<Mode, kNumModes> kAllModes = {
    Mode::kIdle,  Mode::kArmed, Mode::kTracking, Mode::kApproach,  Mode::kCommitted, Mode::kClosing,
    Mode::kDecel, Mode::kHold,  Mode::kRetreat,  Mode::kAbortSafe, Mode::kFault,
};

/// All Reason values, for iteration in the completeness check and in tests.
inline constexpr std::array<Reason, kNumReasons> kAllReasons = {
    Reason::kNone,
    Reason::kBallStale,
    Reason::kBallStaleCommitted,
    Reason::kBallStaleLong,
    Reason::kTrackChanged,
    Reason::kHorizonExtrap,
    Reason::kPredInconsistent,
    Reason::kNoCatchablePlan,
    Reason::kPlanInvalid,
    Reason::kQpFailed,
    Reason::kRefSaturated,
    Reason::kJointConflict,
    Reason::kTrackErr,
    Reason::kAbortEscalated,
    Reason::kEstop,
    Reason::kFaultReset,
    Reason::kSpeedScaling,
    Reason::kClockUnhealthy,
    Reason::kParamsTbd,
    Reason::kHandTimeout,
    Reason::kTipStale,
};

/// One (from, reason) → to edge. A well-formed table has at most one row per
/// (from, reason) pair — `CheckTransitionTableComplete` flags a second row
/// with the same key and a different `to` as a "duplicate cell" (ambiguity
/// resolution: see the header comment's completeness-check section below).
struct TransitionRow {
  Mode from;
  Reason reason;
  Mode to;
};

// ── The merged (Mode × Reason) → Mode table ─────────────────────────────────
// clang-format off
inline constexpr std::array<TransitionRow, 86> kTransitionTable = {{
    // IDLE (§4.1 row 1; §4.2 PARAMS_TBD, CLOCK_UNHEALTHY "IDLE 진입 거부")
    {Mode::kIdle, Reason::kNone, Mode::kArmed},
    {Mode::kIdle, Reason::kParamsTbd, Mode::kIdle},
    {Mode::kIdle, Reason::kClockUnhealthy, Mode::kIdle},
    {Mode::kIdle, Reason::kEstop, Mode::kIdle},

    // ARMED (§4.1 row 2; §4.2 CLOCK_UNHEALTHY "ARMED 진입 거부";
    // ambiguity resolution 3 above for the §4.5 readiness-lost edge)
    {Mode::kArmed, Reason::kNone, Mode::kTracking},
    {Mode::kArmed, Reason::kParamsTbd, Mode::kIdle},
    {Mode::kArmed, Reason::kClockUnhealthy, Mode::kArmed},
    {Mode::kArmed, Reason::kEstop, Mode::kIdle},

    // TRACKING (§4.1 row 3; §4.2 TRACK_CHANGED/BALL_STALE "TRACKING이면
    // ARMED", PRED_INCONSISTENT id., NO_CATCHABLE_PLAN self-loop + record,
    // "전 구간" fatal reasons, ESTOP clear)
    {Mode::kTracking, Reason::kNone, Mode::kApproach},
    {Mode::kTracking, Reason::kTrackChanged, Mode::kArmed},
    {Mode::kTracking, Reason::kBallStale, Mode::kArmed},
    {Mode::kTracking, Reason::kPredInconsistent, Mode::kArmed},
    {Mode::kTracking, Reason::kNoCatchablePlan, Mode::kTracking},
    {Mode::kTracking, Reason::kQpFailed, Mode::kAbortSafe},
    {Mode::kTracking, Reason::kJointConflict, Mode::kAbortSafe},
    {Mode::kTracking, Reason::kTrackErr, Mode::kAbortSafe},
    {Mode::kTracking, Reason::kSpeedScaling, Mode::kAbortSafe},
    {Mode::kTracking, Reason::kClockUnhealthy, Mode::kAbortSafe},
    {Mode::kTracking, Reason::kEstop, Mode::kIdle},

    // APPROACH (§4.1 row 4; §4.2 BALL_STALE/TRACK_CHANGED/HORIZON_EXTRAP/
    // PRED_INCONSISTENT/PLAN_INVALID/REF_SATURATED "APPROACH면 RETREAT",
    // "전 구간" fatal reasons, ESTOP clear)
    {Mode::kApproach, Reason::kNone, Mode::kCommitted},
    {Mode::kApproach, Reason::kBallStale, Mode::kRetreat},
    {Mode::kApproach, Reason::kTrackChanged, Mode::kRetreat},
    {Mode::kApproach, Reason::kHorizonExtrap, Mode::kRetreat},
    {Mode::kApproach, Reason::kPredInconsistent, Mode::kRetreat},
    {Mode::kApproach, Reason::kPlanInvalid, Mode::kRetreat},
    {Mode::kApproach, Reason::kRefSaturated, Mode::kRetreat},
    {Mode::kApproach, Reason::kQpFailed, Mode::kAbortSafe},
    {Mode::kApproach, Reason::kJointConflict, Mode::kAbortSafe},
    {Mode::kApproach, Reason::kTrackErr, Mode::kAbortSafe},
    {Mode::kApproach, Reason::kSpeedScaling, Mode::kAbortSafe},
    {Mode::kApproach, Reason::kClockUnhealthy, Mode::kAbortSafe},
    {Mode::kApproach, Reason::kEstop, Mode::kIdle},

    // COMMITTED (§4.1 row 5; §4.2 BALL_STALE_LONG/REF_SATURATED "동결 후
    // ABORT_SAFE", BALL_STALE_COMMITTED/HORIZON_EXTRAP/TIP_STALE self-loop +
    // record, "전 구간" fatal reasons, ESTOP clear)
    {Mode::kCommitted, Reason::kNone, Mode::kClosing},
    {Mode::kCommitted, Reason::kBallStaleLong, Mode::kAbortSafe},
    {Mode::kCommitted, Reason::kRefSaturated, Mode::kAbortSafe},
    {Mode::kCommitted, Reason::kBallStaleCommitted, Mode::kCommitted},
    {Mode::kCommitted, Reason::kHorizonExtrap, Mode::kCommitted},
    {Mode::kCommitted, Reason::kTipStale, Mode::kCommitted},
    {Mode::kCommitted, Reason::kQpFailed, Mode::kAbortSafe},
    {Mode::kCommitted, Reason::kJointConflict, Mode::kAbortSafe},
    {Mode::kCommitted, Reason::kTrackErr, Mode::kAbortSafe},
    {Mode::kCommitted, Reason::kSpeedScaling, Mode::kAbortSafe},
    {Mode::kCommitted, Reason::kClockUnhealthy, Mode::kAbortSafe},
    {Mode::kCommitted, Reason::kEstop, Mode::kIdle},

    // CLOSING (§4.1 row 6; §4.2 same "동결 후" set as COMMITTED plus
    // HAND_TIMEOUT self-loop + record, "전 구간" fatal reasons, ESTOP clear)
    {Mode::kClosing, Reason::kNone, Mode::kDecel},
    {Mode::kClosing, Reason::kBallStaleLong, Mode::kAbortSafe},
    {Mode::kClosing, Reason::kRefSaturated, Mode::kAbortSafe},
    {Mode::kClosing, Reason::kBallStaleCommitted, Mode::kClosing},
    {Mode::kClosing, Reason::kHorizonExtrap, Mode::kClosing},
    {Mode::kClosing, Reason::kTipStale, Mode::kClosing},
    {Mode::kClosing, Reason::kHandTimeout, Mode::kClosing},
    {Mode::kClosing, Reason::kQpFailed, Mode::kAbortSafe},
    {Mode::kClosing, Reason::kJointConflict, Mode::kAbortSafe},
    {Mode::kClosing, Reason::kTrackErr, Mode::kAbortSafe},
    {Mode::kClosing, Reason::kSpeedScaling, Mode::kAbortSafe},
    {Mode::kClosing, Reason::kClockUnhealthy, Mode::kAbortSafe},
    {Mode::kClosing, Reason::kEstop, Mode::kIdle},

    // DECEL (§4.1 row 7; §4.2 HAND_TIMEOUT/TIP_STALE self-loop + record —
    // REF_SATURATED/BALL_STALE_LONG do NOT apply here per their own
    // "발생 가능 상태" columns (APPROACH/COMMITTED/CLOSING only), "전 구간"
    // fatal reasons, ESTOP clear)
    {Mode::kDecel, Reason::kNone, Mode::kHold},
    {Mode::kDecel, Reason::kHandTimeout, Mode::kDecel},
    {Mode::kDecel, Reason::kTipStale, Mode::kDecel},
    {Mode::kDecel, Reason::kQpFailed, Mode::kAbortSafe},
    {Mode::kDecel, Reason::kJointConflict, Mode::kAbortSafe},
    {Mode::kDecel, Reason::kTrackErr, Mode::kAbortSafe},
    {Mode::kDecel, Reason::kSpeedScaling, Mode::kAbortSafe},
    {Mode::kDecel, Reason::kClockUnhealthy, Mode::kAbortSafe},
    {Mode::kDecel, Reason::kEstop, Mode::kIdle},

    // HOLD (§4.1 row 8; TIP_STALE self-loop, "전 구간" fatal reasons, ESTOP
    // clear)
    {Mode::kHold, Reason::kNone, Mode::kRetreat},
    {Mode::kHold, Reason::kTipStale, Mode::kHold},
    {Mode::kHold, Reason::kQpFailed, Mode::kAbortSafe},
    {Mode::kHold, Reason::kJointConflict, Mode::kAbortSafe},
    {Mode::kHold, Reason::kTrackErr, Mode::kAbortSafe},
    {Mode::kHold, Reason::kSpeedScaling, Mode::kAbortSafe},
    {Mode::kHold, Reason::kClockUnhealthy, Mode::kAbortSafe},
    {Mode::kHold, Reason::kEstop, Mode::kIdle},

    // RETREAT (§4.1 row 9; "전 구간" fatal reasons, ESTOP clear)
    {Mode::kRetreat, Reason::kNone, Mode::kArmed},
    {Mode::kRetreat, Reason::kQpFailed, Mode::kAbortSafe},
    {Mode::kRetreat, Reason::kJointConflict, Mode::kAbortSafe},
    {Mode::kRetreat, Reason::kTrackErr, Mode::kAbortSafe},
    {Mode::kRetreat, Reason::kSpeedScaling, Mode::kAbortSafe},
    {Mode::kRetreat, Reason::kClockUnhealthy, Mode::kAbortSafe},
    {Mode::kRetreat, Reason::kEstop, Mode::kIdle},

    // ABORT_SAFE (§4.1 row 10; ABORT_ESCALATED → FAULT, ESTOP clear)
    {Mode::kAbortSafe, Reason::kNone, Mode::kRetreat},
    {Mode::kAbortSafe, Reason::kAbortEscalated, Mode::kFault},
    {Mode::kAbortSafe, Reason::kEstop, Mode::kIdle},

    // FAULT (§4.1 row 11; FAULT_RESET, ESTOP clear)
    {Mode::kFault, Reason::kFaultReset, Mode::kIdle},
    {Mode::kFault, Reason::kEstop, Mode::kIdle},
}};
// clang-format on

/// Look up the (from, reason) edge. Returns false with `to` untouched if no
/// row matches (the supervisor body then treats the reason as inapplicable in
/// this state and does not transition) — this does NOT distinguish "no row"
/// from "duplicate row" the way `CheckTransitionTableComplete` does; run that
/// once at configure to catch the latter.
[[nodiscard]] constexpr bool LookupTransition(std::span<const TransitionRow> table, Mode from,
                                              Reason reason, Mode& to) noexcept {
  for (const TransitionRow& row : table) {
    if (row.from == from && row.reason == reason) {
      to = row.to;
      return true;
    }
  }
  return false;
}

/// Findings of the startup completeness self-check (§4.1: "기동 시 완전성을
/// 검사한다"). `ok` is true iff every array below is all-false and
/// `duplicate_cell` is false.
struct CompletenessResult {
  /// [i] true iff Mode kAllModes[i] never appears as a `to` from a different
  /// `from` — i.e. unreachable.
  std::array<bool, kNumModes> unreachable{};
  /// [i] true iff Mode kAllModes[i] never appears as a `from` with a `to`
  /// different from itself — i.e. has no exit.
  std::array<bool, kNumModes> no_exit{};
  /// [i] true iff Reason kAllReasons[i] appears in no row at all.
  std::array<bool, kNumReasons> unused_reason{};
  /// True iff some (from, reason) key appears in more than one row with
  /// different `to` values (an ambiguous / undefined cell — see the header
  /// comment above `TransitionRow`).
  bool duplicate_cell{false};
  Mode duplicate_from{Mode::kIdle};
  Reason duplicate_reason{Reason::kNone};

  [[nodiscard]] constexpr bool Ok() const noexcept {
    for (bool v : unreachable)
      if (v)
        return false;
    for (bool v : no_exit)
      if (v)
        return false;
    for (bool v : unused_reason)
      if (v)
        return false;
    return !duplicate_cell;
  }
};

/// Run the three completeness checks the header comment documents:
/// unreachable/no-exit states, unused reasons, and duplicate (ambiguous)
/// cells. Takes a span (not the fixed-size production array) so tests can
/// feed deliberately broken variants of arbitrary length.
[[nodiscard]] constexpr CompletenessResult CheckTransitionTableComplete(
    std::span<const TransitionRow> table) noexcept {
  CompletenessResult result{};

  for (std::size_t m = 0; m < kNumModes; ++m) {
    const Mode mode = kAllModes[m];
    bool reachable = false;
    bool has_exit = false;
    for (const TransitionRow& row : table) {
      if (row.to == mode && row.from != mode)
        reachable = true;
      if (row.from == mode && row.to != mode)
        has_exit = true;
    }
    result.unreachable[m] = !reachable;
    result.no_exit[m] = !has_exit;
  }

  for (std::size_t r = 0; r < kNumReasons; ++r) {
    const Reason reason = kAllReasons[r];
    bool used = false;
    for (const TransitionRow& row : table) {
      if (row.reason == reason) {
        used = true;
        break;
      }
    }
    result.unused_reason[r] = !used;
  }

  for (std::size_t i = 0; i < table.size() && !result.duplicate_cell; ++i) {
    for (std::size_t j = i + 1; j < table.size(); ++j) {
      if (table[i].from == table[j].from && table[i].reason == table[j].reason &&
          table[i].to != table[j].to) {
        result.duplicate_cell = true;
        result.duplicate_from = table[i].from;
        result.duplicate_reason = table[i].reason;
        break;
      }
    }
  }

  return result;
}

}  // namespace rtc::catching
