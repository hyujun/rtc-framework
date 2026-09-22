// ── L1 ingress judgement: order, track epoch, staleness ─────────────────────
//
// The ROS-free half of S5.2. The binding in the integration package owns the
// `PointCloud2` field map, the byte decode and the SeqLock; everything that
// DECIDES — is this message in order, is this snapshot still the one we are
// allowed to use, has it gone stale, has its horizon run out — lives here, so
// it can be tested against a struct instead of against a publisher (D-1).
//
// Two axes run through this file and they are not interchangeable (plan §3):
//
//   NowReal  — the steady receive axis. Staleness, ages and watchdogs are
//              judged on it and ONLY on it. `header.stamp` never reaches a
//              freshness decision (agent_docs/invariants.md §Clock).
//   NowLead  — now + T_arm. Sampling and horizon exhaustion are judged on it,
//              because the arm command being built this tick is realised
//              T_arm later; a horizon that ends before then is exhausted for
//              the command even though it has not expired in real time.
//
// The T_arm ≠ 0 fixture requirement (plan §3) exists because the two axes
// coincide at T_arm = 0, which hides every confusion between them.

#pragma once

#include "rtc_controllers/catching/time_types.hpp"
#include "rtc_controllers/catching/traj_sampler.hpp"
#include "rtc_controllers/catching/trajectory.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace rtc::catching {

// ── Order and track epoch (L1 §4.4, A-S5-4) ─────────────────────────────────

enum class IngressReject : std::uint8_t {
  kNone = 0,
  /// A point carries `validity` != VALID. C-1: the whole message is refused
  /// rather than the point dropped. S3.4 measured zero partial-invalid
  /// messages over 856 samples, so this is a fail-closed rule with no
  /// observed cost; the decision to revisit it (plan C-1) belongs to whoever
  /// first sees the counter move.
  kNotEvaluated = 1,
  /// `snapshot_sequence` at or below the last accepted value WITHIN the same
  /// generation — a duplicate or a message overtaken in flight. Accepting it
  /// would let an older prediction overwrite a newer one (L1 §4.4, v0.4 M1).
  kStaleSequence = 2,
};

/// What the ingress remembers between messages. One instance per subscription.
///
/// Deliberately NOT part of the snapshot: this is the receiver's memory of
/// what it has already accepted, and a snapshot that carried it would let a
/// stale copy re-authorise a message the receiver had already rejected.
struct IngressState {
  std::uint64_t last_generation{0};
  std::uint64_t last_sequence{0};
  bool seen{false};
};

/// The two identity fields every point of a message must agree on, plus the
/// message-wide verdict of the per-point `validity` field.
struct IngressHeader {
  std::uint64_t generation{0};
  std::uint64_t snapshot_sequence{0};
  bool all_points_valid{false};
};

/// Accept or reject on order and validity, updating `state` only on acceptance.
///
/// GENERATION CHANGE RESETS THE SEQUENCE EXPECTATION (A-S5-4). The vision node
/// numbers snapshots within a track epoch; a new epoch may restart at any
/// value, including one below what the previous epoch reached. Carrying the
/// expectation across the change would refuse every message of the new track
/// until it happened to climb past the old high-water mark — a permanent
/// blindness after a restart, with no error anywhere. S3.4 observed no
/// rewinds in 856 messages, so this rule costs nothing observed and removes
/// the one failure mode that would be silent.
///
/// The rejection order is validity first: an invalid message tells us nothing
/// about the sequence, so recording it would be recording a number we have
/// decided not to believe.
[[nodiscard]] inline IngressReject CheckOrder(const IngressHeader& hdr,
                                              IngressState& state) noexcept {
  if (!hdr.all_points_valid) {
    return IngressReject::kNotEvaluated;
  }
  const bool same_track = state.seen && hdr.generation == state.last_generation;
  if (same_track && hdr.snapshot_sequence <= state.last_sequence) {
    return IngressReject::kStaleSequence;
  }
  state.last_generation = hdr.generation;
  state.last_sequence = hdr.snapshot_sequence;
  state.seen = true;
  return IngressReject::kNone;
}

// ── RT-side read (L1 §5.3) ──────────────────────────────────────────────────

/// What one tick concluded about the snapshot it is holding.
struct TrajView {
  /// Unusable this tick: wrong activation, never filled, or older than
  /// `t_stale_ns` on the receive axis. A stale snapshot is not a reason to
  /// reach for an older one — there is no older one — it is a reason to stop
  /// planning against vision.
  bool stale{true};
  /// The prediction ends before the instant this tick's command realises.
  /// Separate from `stale` because a FRESH snapshot can be exhausted (the ball
  /// has flown past the end of the predicted window) and the two call for
  /// different reasons in the supervisor: BALL_STALE against HORIZON_EXTRAP.
  bool expired{false};
  /// A snapshot the previous tick had not seen. Judged from the payload's own
  /// `snapshot_sequence`, never from SeqLock::sequence() — D-21: reading the
  /// lock's counter and the payload separately lets a writer land between the
  /// two, pairing a new counter with an old payload and hiding the newest
  /// snapshot for as long as the pattern repeats.
  bool is_new{false};
  /// Age on the receive axis, or **-1 when nothing has ever been received**.
  /// Published as a diagnostic even when the snapshot is usable, because a
  /// horizon that is always fresh and always short is a vision problem that a
  /// boolean cannot express.
  ///
  /// The sentinel is not cosmetic. `traj_recv_ns` is 0 before the first
  /// message, so a plain `now - recv` reports the STEADY-CLOCK UPTIME as the
  /// age — 373966 s on the box this was caught on (2026-09-23), on every row
  /// of every run until the first prediction arrives. Any consumer that
  /// thresholds on the number, or averages it, silently eats the uptime. -1
  /// is the same "never" the fingertip lane uses for `tip_age_s`, so the two
  /// receive-axis lanes read the same way.
  std::int64_t age_ns{-1};
};

/// What the caller has already consumed off the payload lane.
///
/// `sequence` alone cannot carry this. 0 is a legal `snapshot_sequence` — the
/// value comes straight off the wire from the vision node and nothing in this
/// repo forbids it — so a bare `last_consumed_sequence == 0` means both
/// "nothing consumed yet" and "consumed number zero", and the first snapshot
/// of a lane that happens to be numbered 0 reads as already seen. The track
/// axis solved this with an explicit `seen` flag; this is the same flag for
/// the sequence axis.
///
/// `generation` is carried for the OTHER half: A-S5-4 lets a new track epoch
/// restart its numbering at any value, so a sequence that merely repeats the
/// last one is not evidence of a repeat unless the epoch also matches.
struct ConsumedToken {
  std::uint64_t sequence{0};
  std::uint64_t generation{0};
  bool seen{false};
};

/// Judge a snapshot the caller has already loaded.
///
/// Takes the loaded copy rather than the SeqLock so the decision is testable
/// against a struct, and so the caller keeps the D-21 obligation where it is
/// visible: `Load()` unconditionally, once per tick, before calling this.
///
/// `current_activation` is the controller's `ActivationGeneration()`. D-23: a
/// controller-owned subscription stays alive while the controller is inactive
/// (lifecycle gates publishers, not subscriptions), so without this check the
/// first tick after re-activation would consume a trajectory received while
/// the robot was under someone else's control.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
[[nodiscard]] inline TrajView ReadTraj(const TrajectorySnapshot& snap, NowReal now,
                                       NowLead now_lead, std::int64_t t_stale_ns,
                                       std::uint64_t current_activation,
                                       ConsumedToken& consumed) noexcept {
  TrajView view{};
  // `traj_recv_ns == 0` is "never received" (the same 0-is-absent polarity the
  // hole mask and the sensor lane use), and an age measured against it would
  // be the uptime rather than an age — see `TrajView::age_ns`.
  view.age_ns = snap.token.traj_recv_ns > 0 ? AgeNs(now, NowReal{snap.token.traj_recv_ns}) : -1;
  // Newness is a property of the (epoch, number) PAIR, not of the number. A
  // repeat is only a repeat within one epoch, and nothing at all has been
  // consumed until `seen` says so.
  view.is_new =
      snap.valid && (!consumed.seen || snap.token.snapshot_sequence != consumed.sequence ||
                     snap.token.generation != consumed.generation);
  if (view.is_new) {
    consumed.sequence = snap.token.snapshot_sequence;
    consumed.generation = snap.token.generation;
    consumed.seen = true;
  }

  const bool current = snap.token.activation_generation == current_activation;
  // A non-positive threshold is a configuration error, and the safe reading of
  // "no sample can satisfy this" is to withhold the lane rather than to treat
  // it as "no limit" — the same fail-closed convention as IsSensorGroupFresh.
  // `traj_recv_ns > 0` stays even though `age_ns` is now -1 without it: the
  // sentinel is smaller than any threshold, so dropping this term would make
  // "never received" read as the freshest possible sample.
  const bool fresh = t_stale_ns > 0 && snap.token.traj_recv_ns > 0 && view.age_ns >= 0 &&
                     view.age_ns <= t_stale_ns;
  view.stale = !snap.valid || !current || !fresh;

  // Horizon exhaustion is judged on the LEAD axis and only when there is a
  // sample to judge against. It is reported even for a stale snapshot: the
  // supervisor decides which reason wins, and a caller that wants one answer
  // is better served by one that is always computed the same way.
  if (snap.n > 0) {
    const std::size_t last = static_cast<std::size_t>(snap.n) - 1;
    view.expired = HorizonExceeded(now_lead, BallTime{snap.s[last].t_ns});
  }
  return view;
}

/// Whether a snapshot's predicted window is long enough to be worth planning
/// against (D-15). Shorter than `horizon_min` is a DIAGNOSIS, not a rejection:
/// the message is well-formed and the ball is real, the prediction simply does
/// not reach far enough for a commit decision, and the operator needs to see
/// that as a vision-configuration problem rather than as silence.
[[nodiscard]] inline bool HorizonShort(const TrajectorySnapshot& snap,
                                       std::int64_t horizon_min_ns) noexcept {
  if (snap.n < 2) {
    return true;
  }
  const std::size_t last = static_cast<std::size_t>(snap.n) - 1;
  const std::int64_t span = detail::SatSub(snap.s[last].t_ns, snap.s[0].t_ns);
  return span < horizon_min_ns;
}

// ── Covariance (A-3, D-22) ──────────────────────────────────────────────────

/// Per-point 6x6 position/velocity covariance, row-major (p_x..v_z), as vision
/// publishes it. NaN means "not known" for that element and is carried through
/// unchanged — an unknown element must stay unknown rather than become a zero
/// that reads as certainty.
///
/// SEPARATE FROM THE TRAJECTORY SNAPSHOT (A-3), not merged into it: the RT tick
/// never needs a covariance, and folding 11.5 KB into the payload the tick
/// copies every cycle would put the planner's data in the RT budget. It
/// carries the SAME token (D-22) so the planner can refuse a covariance that
/// does not belong to the trajectory it is holding — the N/N-1 mix-up that
/// token exists to make impossible.
///
/// Nothing consumes this yet; the consumer is the S6 planner (A-S5-5). It is
/// filled here because the covariance only exists at ingress — by the time the
/// planner runs, the message is gone.
struct CovarianceSnapshot {
  static constexpr std::size_t kElems = 36;

  ProvenanceToken token{};
  std::int32_t n{0};
  bool valid{false};
  std::array<std::array<double, kElems>, kCap> c{};
};

// ── Prediction jump (L1 §4.4 diagnostic) ────────────────────────────────────

/// Distance between two consecutive predictions of the SAME track, evaluated
/// at one absolute instant (L1 §4.4's J).
///
/// Sampling both at a COMMON instant is the whole point: two predictions
/// disagree about where the ball will be, and comparing them at their own
/// first samples would measure how far the ball flew between messages
/// instead. `eval_offset_ns` past the later origin keeps the instant inside
/// both windows without asking either to extrapolate.
///
/// Returns a negative value when no comparison is possible (different tracks,
/// an empty snapshot, or an instant outside a window) so a caller cannot
/// mistake "not compared" for "no jump".
[[nodiscard]] inline double JumpBetween(const TrajectorySnapshot& older,
                                        const TrajectorySnapshot& newer,
                                        std::int64_t eval_offset_ns) noexcept {
  if (!older.valid || !newer.valid || older.n < 1 || newer.n < 1) {
    return -1.0;
  }
  if (older.token.generation != newer.token.generation) {
    return -1.0;  // different balls; the distance between them means nothing
  }
  const std::int64_t origin =
      std::max(older.s[0].t_ns, newer.s[0].t_ns) + (eval_offset_ns > 0 ? eval_offset_ns : 0);
  int hint_a = 0;
  int hint_b = 0;
  const SampleEval a = SampleAt(older, NowLead{origin}, hint_a);
  const SampleEval b = SampleAt(newer, NowLead{origin}, hint_b);
  if (!a.valid || !b.valid || a.after_horizon || b.after_horizon) {
    return -1.0;
  }
  return (b.p - a.p).norm();
}

}  // namespace rtc::catching
