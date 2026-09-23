// L6 hand sequencer (dynamic_catching S7.1, L6 §5.3). The close instant (G6-A),
// the phase rules (Q4: open only while homing, wait at q_pre, release to q_pre),
// ρ, the hold rule and the fail-closed paths. Deterministic: the sequencer
// takes `now` as an argument, so every case below builds its own tick grid.
//
// no_malloc_scope.hpp MUST precede every Eigen header, and alloc_gate.hpp
// defines the replacement global operators — exactly one TU per binary; this
// is that TU (same arrangement as test_catching_supervisor_core.cpp).
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_controllers/catching/hand_sequencer.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <span>
#include <type_traits>

namespace {

using rtc::catching::BallTime;
using rtc::catching::HandHoldMode;
using rtc::catching::HandPhase;
using rtc::catching::HandSequencer;
using rtc::catching::HandSequencerConfig;
using rtc::catching::HandSequencerOutput;
using rtc::catching::kMaxHandDof;
using rtc::catching::NowLead;
using rtc::catching::NowReal;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kH = 2 * kMs;  // 500 Hz
constexpr int kDof = 3;

// The hand close uses the REAL axis. A lead instant must not even compile as
// the sequencer's clock — T_arm is the arm servo's lag, not the hand's.
static_assert(std::is_invocable_v<decltype(&HandSequencer::Update), HandSequencer&, NowReal,
                                  std::int64_t, std::span<const double>, std::span<const double>>);
static_assert(!std::is_invocable_v<decltype(&HandSequencer::Update), HandSequencer&, NowLead,
                                   std::int64_t, std::span<const double>, std::span<const double>>);

// Joint 1 closes in the NEGATIVE direction and joint 2 is off the caging set,
// so a sign or mask slip in ρ changes the answer instead of cancelling out.
HandSequencerConfig MakeConfig() {
  HandSequencerConfig c{};
  c.dof = kDof;
  c.q_open = {-0.3, 0.3, 0.0};
  c.q_pre = {0.0, 0.0, 0.1};
  c.q_close = {0.5, -0.4, 0.9};
  c.caging_mask = {true, true, false};
  c.eta_close = 0.7;
  c.t_close_e2e_ns = 150 * kMs;
  c.t_close_timeout_ns = 300 * kMs;
  c.q_tol = 0.01;
  c.qd_tol = 0.05;
  return c;
}

struct HandState {
  std::array<double, kMaxHandDof> q{};
  std::array<double, kMaxHandDof> qd{};

  [[nodiscard]] std::span<const double> Q() const {
    return {q.data(), static_cast<std::size_t>(kDof)};
  }

  [[nodiscard]] std::span<const double> Qd() const {
    return {qd.data(), static_cast<std::size_t>(kDof)};
  }
};

HandState At(const std::array<double, kMaxHandDof>& pose) {
  HandState s;
  s.q = pose;
  return s;
}

/// A pose `frac` of the way from q_pre to q_close on every joint.
HandState Between(const HandSequencerConfig& c, double frac) {
  HandState s;
  for (int i = 0; i < kDof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    s.q[u] = c.q_pre[u] + frac * (c.q_close[u] - c.q_pre[u]);
  }
  return s;
}

HandSequencer Ready(const HandSequencerConfig& c = MakeConfig()) {
  HandSequencer seq;
  EXPECT_TRUE(seq.Configure(c));
  seq.Ready();
  return seq;
}

// ── Configuration ───────────────────────────────────────────────────────────

TEST(HandSequencer, RefusesAConfigItCannotRunAndStaysInactive) {
  const auto refused = [](HandSequencerConfig c) {
    HandSequencer seq;
    EXPECT_FALSE(seq.Configure(c));
    seq.Ready();  // must not activate an unconfigured sequencer
    const HandState s = At(c.q_pre);
    EXPECT_FALSE(seq.Update(NowReal{0}, kH, s.Q(), s.Qd()).active);
  };
  HandSequencerConfig c = MakeConfig();
  c.t_close_timeout_ns = c.t_close_e2e_ns;  // must be strictly above
  refused(c);
  c = MakeConfig();
  c.caging_mask = {false, false, false};  // ρ would be a min over nothing
  refused(c);
  c = MakeConfig();
  c.q_close[0] = c.q_pre[0];  // ρ divides by this gap
  refused(c);
  c = MakeConfig();
  c.eta_close = std::numeric_limits<double>::quiet_NaN();
  refused(c);
  c = MakeConfig();
  c.q_tol = 0.0;
  refused(c);
}

TEST(HandSequencer, AProfileWithAnUnresolvedValueDoesNotConfigure) {
  rtc::catching::HandProfile p{};
  p.dof = kDof;
  p.tbd = false;
  p.q_open_tbd = false;
  p.q_pre = MakeConfig().q_pre;
  p.q_close = MakeConfig().q_close;
  p.caging_mask = MakeConfig().caging_mask;
  p.eta_close = rtc::catching::TbdDouble::Resolved(0.7);
  p.T_close_e2e = rtc::catching::TbdDouble::Resolved(0.15);
  p.T_close_timeout = rtc::catching::TbdDouble::Resolved(0.3);
  HandSequencer seq;
  EXPECT_TRUE(seq.Configure(HandSequencerConfig::FromProfile(p)));
  EXPECT_EQ(HandSequencerConfig::FromProfile(p).t_close_e2e_ns, 150 * kMs);

  p.T_close_timeout = rtc::catching::TbdDouble{};  // TBD
  EXPECT_FALSE(seq.Configure(HandSequencerConfig::FromProfile(p)));
}

// ── Phases (#537 S7 Q4) ─────────────────────────────────────────────────────

TEST(HandSequencer, IsInactiveUntilToldAndHomesToOpenThenWaitsAtPre) {
  const HandSequencerConfig c = MakeConfig();
  HandSequencer seq;
  ASSERT_TRUE(seq.Configure(c));
  const HandState s = At(c.q_pre);
  EXPECT_FALSE(seq.Update(NowReal{0}, kH, s.Q(), s.Qd()).active);

  seq.Home();
  HandSequencerOutput out = seq.Update(NowReal{0}, kH, s.Q(), s.Qd());
  EXPECT_TRUE(out.active);
  EXPECT_EQ(out.phase, HandPhase::kOpen);
  for (int i = 0; i < kDof; ++i) {
    EXPECT_DOUBLE_EQ(out.target[static_cast<std::size_t>(i)],
                     c.q_open[static_cast<std::size_t>(i)]);
  }

  seq.Ready();
  out = seq.Update(NowReal{kH}, kH, s.Q(), s.Qd());
  EXPECT_EQ(out.phase, HandPhase::kPreshape);
  for (int i = 0; i < kDof; ++i) {
    EXPECT_DOUBLE_EQ(out.target[static_cast<std::size_t>(i)], c.q_pre[static_cast<std::size_t>(i)]);
  }
  EXPECT_TRUE(out.at_target);

  seq.Deactivate();
  EXPECT_FALSE(seq.Update(NowReal{2 * kH}, kH, s.Q(), s.Qd()).active);
}

TEST(HandSequencer, CommitsOnlyFromPreshapeAndOnlyOnce) {
  HandSequencer seq;
  ASSERT_TRUE(seq.Configure(MakeConfig()));
  seq.Home();
  EXPECT_FALSE(seq.Commit(BallTime{kMs * 1000})) << "still homing: no catch to commit to";

  seq.Ready();
  EXPECT_TRUE(seq.Commit(BallTime{1000 * kMs}));
  EXPECT_EQ(seq.TCmd().ns, 850 * kMs);  // t_c − T_close_e2e
  EXPECT_FALSE(seq.Commit(BallTime{2000 * kMs})) << "t_c is frozen at the first commit";
  EXPECT_EQ(seq.TCmd().ns, 850 * kMs);
}

TEST(HandSequencer, ReadyCancelsACommitThatHasNotClosed) {
  HandSequencer seq = Ready();
  ASSERT_TRUE(seq.Commit(BallTime{100 * kMs}));
  seq.Ready();
  EXPECT_FALSE(seq.CommitArmed());
  const HandState s = At(MakeConfig().q_pre);
  for (std::int64_t t = 0; t < 200 * kMs; t += kH) {
    EXPECT_EQ(seq.Update(NowReal{t}, kH, s.Q(), s.Qd()).phase, HandPhase::kPreshape);
  }
}

// ── G6-A: the close command tick ────────────────────────────────────────────

/// Run ticks from `t0` on the given grid until the close goes out; return the
/// instant it did (or INT64_MIN).
template <typename NextTick>
std::int64_t CloseInstant(HandSequencer& seq, std::int64_t t0, NextTick next) {
  const HandState s = At(MakeConfig().q_pre);
  std::int64_t t = t0;
  for (int k = 0; k < 100000; ++k) {
    const HandSequencerOutput out = seq.Update(NowReal{t}, kH, s.Q(), s.Qd());
    if (out.close_issued_now) {
      EXPECT_EQ(out.phase, HandPhase::kClose);
      return t;
    }
    t = next(t);
  }
  return std::numeric_limits<std::int64_t>::min();
}

TEST(HandSequencer, G6ACloseTickIsWithinHalfATickOnARegularGrid) {
  // 1000 synthetic trials: the catch instant falls at a random phase of the
  // tick grid. On a regular grid the nearest tick is at most h/2 away.
  std::mt19937_64 rng(537);
  std::uniform_int_distribution<std::int64_t> phase(0, kH - 1);
  std::uniform_int_distribution<std::int64_t> tc_ms(400, 1200);
  std::int64_t worst = 0;
  for (int trial = 0; trial < 1000; ++trial) {
    HandSequencer seq = Ready();
    const std::int64_t t_c = tc_ms(rng) * kMs + phase(rng);
    ASSERT_TRUE(seq.Commit(BallTime{t_c}));
    const std::int64_t t_cmd = seq.TCmd().ns;
    const std::int64_t issued = CloseInstant(seq, 0, [](std::int64_t t) { return t + kH; });
    ASSERT_NE(issued, std::numeric_limits<std::int64_t>::min());
    const std::int64_t err = issued - t_cmd;
    EXPECT_GE(err, -kH / 2) << "trial " << trial;
    EXPECT_LT(err, kH / 2 + 1) << "trial " << trial;
    worst = std::max(worst, std::abs(err));
  }
  RecordProperty("worst_close_error_us", static_cast<int>(worst / 1000));
}

TEST(HandSequencer, G6ACloseTickUnderJitterIsNeverEarlierThanHalfATick) {
  // With a jittered grid the nearest-tick bound becomes "never more than h/2
  // early, and late by at most the spacing that straddled t_cmd minus h/2".
  std::mt19937_64 rng(538);
  const std::int64_t jitter = 300'000;  // ±0.3 ms
  std::uniform_int_distribution<std::int64_t> j(-jitter, jitter);
  std::uniform_int_distribution<std::int64_t> tc_ms(400, 1200);
  for (int trial = 0; trial < 1000; ++trial) {
    HandSequencer seq = Ready();
    const std::int64_t t_c = tc_ms(rng) * kMs + j(rng);
    ASSERT_TRUE(seq.Commit(BallTime{t_c}));
    const std::int64_t t_cmd = seq.TCmd().ns;
    std::int64_t last_spacing = kH;
    const std::int64_t issued = CloseInstant(seq, 0, [&](std::int64_t t) {
      last_spacing = kH + j(rng);
      return t + last_spacing;
    });
    const std::int64_t err = issued - t_cmd;
    EXPECT_GE(err, -kH / 2) << "trial " << trial;
    EXPECT_LE(err, last_spacing - kH / 2) << "trial " << trial;
  }
}

TEST(HandSequencer, ALateCommitClosesOnTheNextTick) {
  // A t_cmd already in the past (a freeze window shorter than the closure,
  // which the validator refuses — this is the defence behind it): the close
  // goes out at once rather than never.
  HandSequencer seq = Ready();
  ASSERT_TRUE(seq.Commit(BallTime{100 * kMs}));  // t_cmd = −50 ms
  const HandState s = At(MakeConfig().q_pre);
  EXPECT_TRUE(seq.Update(NowReal{0}, kH, s.Q(), s.Qd()).close_issued_now);
}

// ── Close → Hold (L6 §4.2) ──────────────────────────────────────────────────

TEST(HandSequencer, RhoIsTheMinimumOverCagingJointsWithEachJointsOwnSign) {
  const HandSequencerConfig c = MakeConfig();
  HandSequencer seq;
  ASSERT_TRUE(seq.Configure(c));
  // Joint 0 half-way (0.25 of 0.5), joint 1 (negative span) three quarters,
  // joint 2 not caging: ρ = 0.5 whatever joint 2 does.
  HandState s;
  s.q = {0.25, -0.3, 5.0};
  EXPECT_NEAR(seq.Rho(s.Q()), 0.5, 1e-12);
  s.q = {0.5, -0.1, -5.0};
  EXPECT_NEAR(seq.Rho(s.Q()), 0.25, 1e-12);
  s.q[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(std::isnan(seq.Rho(s.Q())));
}

TEST(HandSequencer, ClosesToHoldOnTheTickRhoReachesEta) {
  const HandSequencerConfig c = MakeConfig();
  HandSequencer seq = Ready(c);
  ASSERT_TRUE(seq.Commit(BallTime{150 * kMs}));  // t_cmd = 0
  HandState s = At(c.q_pre);
  ASSERT_TRUE(seq.Update(NowReal{0}, kH, s.Q(), s.Qd()).close_issued_now);
  // The hand travels 0.01 of the way per tick, so ρ crosses η = 0.7 around
  // k = 70. The expected tick is the first one whose reading the sequencer's
  // OWN ρ puts at or above η — the case is about WHEN the phase changes, not
  // about re-deriving ρ's rounding at the boundary.
  int expected = -1;
  for (int k = 1; k <= 100 && expected < 0; ++k) {
    if (seq.Rho(Between(c, 0.01 * k).Q()) >= c.eta_close) {
      expected = k;
    }
  }
  ASSERT_GE(expected, 69);
  ASSERT_LE(expected, 71);
  int hold_at = -1;
  for (int k = 1; k <= 100; ++k) {
    s = Between(c, 0.01 * k);
    const HandSequencerOutput out = seq.Update(NowReal{k * kH}, kH, s.Q(), s.Qd());
    if (out.phase == HandPhase::kHold) {
      hold_at = k;
      EXPECT_FALSE(out.timeout);
      break;
    }
    EXPECT_EQ(out.phase, HandPhase::kClose);
  }
  EXPECT_EQ(hold_at, expected);
}

TEST(HandSequencer, AStalledCloseEndsInHoldAtTheTimeoutAndSaysSo) {
  const HandSequencerConfig c = MakeConfig();
  HandSequencer seq = Ready(c);
  ASSERT_TRUE(seq.Commit(BallTime{150 * kMs}));
  const HandState stalled = Between(c, 0.5);  // ρ = 0.5 < η forever
  ASSERT_TRUE(seq.Update(NowReal{0}, kH, stalled.Q(), stalled.Qd()).close_issued_now);
  HandSequencerOutput out{};
  std::int64_t t = 0;
  while (t < 1000 * kMs) {
    t += kH;
    out = seq.Update(NowReal{t}, kH, stalled.Q(), stalled.Qd());
    if (out.phase == HandPhase::kHold) {
      break;
    }
  }
  EXPECT_EQ(t, c.t_close_timeout_ns);
  EXPECT_TRUE(out.timeout);
  EXPECT_NEAR(out.rho, 0.5, 1e-12);
}

TEST(HandSequencer, ANonFiniteReadingNeverClosesByRhoButTheTimeoutStillEndsTheClose) {
  const HandSequencerConfig c = MakeConfig();
  HandSequencer seq = Ready(c);
  ASSERT_TRUE(seq.Commit(BallTime{150 * kMs}));
  HandState bad = Between(c, 0.9);
  bad.q[1] = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(seq.Update(NowReal{0}, kH, bad.Q(), bad.Qd()).close_issued_now);
  const HandSequencerOutput mid = seq.Update(NowReal{kH}, kH, bad.Q(), bad.Qd());
  EXPECT_EQ(mid.phase, HandPhase::kClose) << "ρ from a NaN must not read as closed";
  EXPECT_FALSE(mid.at_target);
  EXPECT_EQ(mid.rho, 0.0);
  const HandSequencerOutput end = seq.Update(NowReal{c.t_close_timeout_ns}, kH, bad.Q(), bad.Qd());
  EXPECT_EQ(end.phase, HandPhase::kHold);
  EXPECT_TRUE(end.timeout);
}

// ── Hold rule (L6 §4.4, #537 S7 Q5) ─────────────────────────────────────────

HandSequencerOutput CloseToHold(HandSequencer& seq, const HandSequencerConfig& c,
                                const HandState& caged) {
  EXPECT_TRUE(seq.Commit(BallTime{150 * kMs}));
  const HandState pre = At(c.q_pre);
  EXPECT_TRUE(seq.Update(NowReal{0}, kH, pre.Q(), pre.Qd()).close_issued_now);
  return seq.Update(NowReal{kH}, kH, caged.Q(), caged.Qd());
}

TEST(HandSequencer, HoldCloseTargetKeepsCommandingTheClosedPosture) {
  const HandSequencerConfig c = MakeConfig();
  HandSequencer seq = Ready(c);
  const HandSequencerOutput out = CloseToHold(seq, c, Between(c, 0.8));
  ASSERT_EQ(out.phase, HandPhase::kHold);
  for (int i = 0; i < kDof; ++i) {
    EXPECT_DOUBLE_EQ(out.target[static_cast<std::size_t>(i)],
                     c.q_close[static_cast<std::size_t>(i)]);
  }
}

TEST(HandSequencer, HoldMeasuredOffsetSqueezesFromWhereTheFingersStopped) {
  HandSequencerConfig c = MakeConfig();
  c.hold_mode = HandHoldMode::kMeasuredOffset;
  c.hold_delta_rad = 0.05;
  HandSequencer seq = Ready(c);
  const HandState caged = Between(c, 0.8);
  const HandSequencerOutput out = CloseToHold(seq, c, caged);
  ASSERT_EQ(out.phase, HandPhase::kHold);
  EXPECT_NEAR(out.target[0], caged.q[0] + 0.05, 1e-12);  // closes toward +
  EXPECT_NEAR(out.target[1], caged.q[1] - 0.05, 1e-12);  // closes toward −
  EXPECT_DOUBLE_EQ(out.target[2], c.q_close[2]);         // not caging: the profile's posture
  // The target is latched at Hold entry, not re-offset from every reading
  // (which would ratchet the fingers closed tick by tick).
  const HandState later = Between(c, 0.95);
  const HandSequencerOutput next = seq.Update(NowReal{2 * kH}, kH, later.Q(), later.Qd());
  EXPECT_NEAR(next.target[0], caged.q[0] + 0.05, 1e-12);
}

TEST(HandSequencer, HoldMeasuredOffsetFromANonFiniteReadingFallsBackToTheClosedPosture) {
  // The Close ends on its timeout (ρ from a NaN never reads as closed); the
  // offset form then has no measured pose for that joint, and a NaN target
  // would reach the hand — the device clamp passes NaN through.
  HandSequencerConfig c = MakeConfig();
  c.hold_mode = HandHoldMode::kMeasuredOffset;
  c.hold_delta_rad = 0.05;
  HandSequencer seq = Ready(c);
  ASSERT_TRUE(seq.Commit(BallTime{150 * kMs}));
  const HandState pre = At(c.q_pre);
  ASSERT_TRUE(seq.Update(NowReal{0}, kH, pre.Q(), pre.Qd()).close_issued_now);
  HandState bad = Between(c, 0.5);
  bad.q[1] = std::numeric_limits<double>::quiet_NaN();
  const HandSequencerOutput end = seq.Update(NowReal{c.t_close_timeout_ns}, kH, bad.Q(), bad.Qd());
  ASSERT_EQ(end.phase, HandPhase::kHold);
  ASSERT_TRUE(end.timeout);
  EXPECT_NEAR(end.target[0], bad.q[0] + 0.05, 1e-12);  // a finite joint still offsets
  EXPECT_DOUBLE_EQ(end.target[1], c.q_close[1]);       // the NaN joint: the profile's posture
  for (int i = 0; i < kDof; ++i) {
    EXPECT_TRUE(std::isfinite(end.target[static_cast<std::size_t>(i)])) << "joint " << i;
  }
}

TEST(HandSequencer, HoldDeltaIsCheckedOnlyInTheModeThatReadsIt) {
  // The validator checks hold.delta_rad under measured_offset only; the
  // sequencer must not refuse, under close_target, a profile it passed.
  HandSequencerConfig c = MakeConfig();
  c.hold_mode = HandHoldMode::kCloseTarget;
  c.hold_delta_rad = -0.05;
  HandSequencer seq;
  EXPECT_TRUE(seq.Configure(c));
  c.hold_mode = HandHoldMode::kMeasuredOffset;
  EXPECT_FALSE(seq.Configure(c));
  c.hold_delta_rad = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(seq.Configure(c));
}

// ── Release (#537 S7 Q4/Q12) ────────────────────────────────────────────────

TEST(HandSequencer, ReleaseOpensToPreAndWaitsThereOnlyOnceSettled) {
  const HandSequencerConfig c = MakeConfig();
  HandSequencer seq = Ready(c);
  ASSERT_EQ(CloseToHold(seq, c, Between(c, 0.8)).phase, HandPhase::kHold);
  seq.Release();
  HandState s = Between(c, 0.8);
  HandSequencerOutput out = seq.Update(NowReal{2 * kH}, kH, s.Q(), s.Qd());
  EXPECT_EQ(out.phase, HandPhase::kRelease);
  for (int i = 0; i < kDof; ++i) {
    EXPECT_DOUBLE_EQ(out.target[static_cast<std::size_t>(i)], c.q_pre[static_cast<std::size_t>(i)]);
  }
  // AT q_pre but still moving: passing through is not being there.
  s = At(c.q_pre);
  s.qd[0] = 0.2;
  out = seq.Update(NowReal{3 * kH}, kH, s.Q(), s.Qd());
  EXPECT_EQ(out.phase, HandPhase::kRelease);
  EXPECT_FALSE(out.at_target);
  s.qd[0] = 0.0;
  out = seq.Update(NowReal{4 * kH}, kH, s.Q(), s.Qd());
  EXPECT_EQ(out.phase, HandPhase::kPreshape);
  EXPECT_TRUE(out.at_target);
}

TEST(HandSequencer, AnUnreadableHandIsAtNothing) {
  const HandSequencerConfig c = MakeConfig();
  HandSequencer seq = Ready(c);
  const std::array<double, 1> narrow{0.0};
  const HandSequencerOutput out = seq.Update(NowReal{0}, kH, narrow, narrow);
  EXPECT_TRUE(out.active);
  EXPECT_FALSE(out.at_target);
}

// ── RT: no allocation in Update (G7-D's pure half) ──────────────────────────

TEST(HandSequencer, UpdateDoesNotAllocate) {
  const HandSequencerConfig c = MakeConfig();
  HandSequencer seq = Ready(c);
  ASSERT_TRUE(seq.Commit(BallTime{150 * kMs}));
  const HandState s = Between(c, 0.3);
  {
    rtc::testing::ScopedAllocGate heap_gate;
    rtc::testing::ScopedNoMalloc eigen_gate;
    for (int k = 0; k < 500; ++k) {
      const HandSequencerOutput out = seq.Update(NowReal{k * kH}, kH, s.Q(), s.Qd());
      ASSERT_TRUE(out.active);
    }
    seq.Release();
    static_cast<void>(seq.Update(NowReal{0}, kH, s.Q(), s.Qd()));
    EXPECT_EQ(heap_gate.count(), 0u);
    EXPECT_EQ(eigen_gate.violations(), 0u);
  }
}

}  // namespace
