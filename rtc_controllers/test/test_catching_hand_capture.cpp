// Hand-joint capture witness (#537 S8-C, D-S8-8 (b)). Every clause of the
// per-joint stall test on its own (each one flipped must flip the answer),
// the boundaries, the sign of the closing direction, NaN and narrow lanes,
// `min_joints`, the config's fail-closed check, no allocation (G7-D's pure
// half), and G7-C: the synthetic-noise false-alarm rate of an EMPTY hand.
//
// no_malloc_scope.hpp MUST precede every Eigen header, and alloc_gate.hpp
// defines the replacement global operators — exactly one TU per binary; this
// is that TU (same arrangement as test_catching_hand_sequencer.cpp).
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_controllers/catching/hand_capture.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <random>
#include <span>
#include <sstream>
#include <string>

namespace {

using rtc::catching::EvaluateHandCapture;
using rtc::catching::HandCaptureConfig;
using rtc::catching::HandCaptureReading;
using rtc::catching::HandProfile;
using rtc::catching::kMaxHandDof;
using rtc::catching::TbdDouble;

constexpr int kDof = 4;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

// Joint 1 closes in the NEGATIVE direction, joint 2 has the smallest span
// (the one whose ρ a position error moves most) and joint 3 is off the caging
// set, so a sign, scale or mask slip changes the answer instead of cancelling.
HandCaptureConfig MakeConfig() {
  HandCaptureConfig c{};
  c.dof = kDof;
  c.q_pre = {0.0, 0.0, 0.2, 0.1};
  c.q_close = {1.0, -0.8, 0.313, 0.9};
  c.tau_max = {3.0, 2.0, 1.0, 3.0};
  c.caging_mask = {true, true, true, false};
  c.rho_min = 0.2;
  c.rho_max = 0.9;
  c.effort_frac_min = 0.5;
  c.qd_tol = 0.05;
  c.min_joints = 1;
  return c;
}

/// A hand reading: every joint at `rho` of its own span, at rest, with
/// `frac` of its own max_torque toward q_close.
struct Hand {
  std::array<double, kMaxHandDof> q{};
  std::array<double, kMaxHandDof> qd{};
  std::array<double, kMaxHandDof> tau{};

  Hand(const HandCaptureConfig& c, double rho, double frac) {
    for (int i = 0; i < c.dof; ++i) {
      const auto u = static_cast<std::size_t>(i);
      const double span = c.q_close[u] - c.q_pre[u];
      q[u] = c.q_pre[u] + rho * span;
      tau[u] = (span > 0.0 ? 1.0 : -1.0) * frac * c.tau_max[u];
    }
  }

  /// Joint `i` alone at (rho, frac); every other joint an empty finger at
  /// q_close holding nothing.
  static Hand OneJoint(const HandCaptureConfig& c, int i, double rho, double frac) {
    Hand h(c, 1.0, 0.0);
    const Hand j(c, rho, frac);
    const auto u = static_cast<std::size_t>(i);
    h.q[u] = j.q[u];
    h.tau[u] = j.tau[u];
    return h;
  }

  [[nodiscard]] HandCaptureReading Eval(const HandCaptureConfig& c) const {
    const auto n = static_cast<std::size_t>(c.dof);
    return EvaluateHandCapture(c, std::span<const double>(q.data(), n),
                               std::span<const double>(qd.data(), n),
                               std::span<const double>(tau.data(), n));
  }
};

// ── The witness ──────────────────────────────────────────────────────────────

TEST(HandCapture, AFingerStalledPartWayAndPushingIsBlocked) {
  const HandCaptureConfig c = MakeConfig();
  ASSERT_TRUE(c.Valid());
  // The negative-direction joint: a sign slip in ρ or τ would miss it.
  const HandCaptureReading r = Hand::OneJoint(c, 1, 0.6, 0.9).Eval(c);
  EXPECT_TRUE(r.blocked);
  EXPECT_EQ(r.stalled, 1);
  EXPECT_NEAR(r.effort_frac_max, 0.9, 1e-12);
}

TEST(HandCapture, AnEmptyHandAtQCloseIsNotBlocked) {
  const HandCaptureConfig c = MakeConfig();
  const HandCaptureReading r = Hand(c, 1.0, 0.0).Eval(c);
  EXPECT_FALSE(r.blocked);
  EXPECT_EQ(r.stalled, 0);
  EXPECT_NEAR(r.effort_frac_max, 0.0, 1e-12);
}

TEST(HandCapture, EachClauseAloneRefusesTheStall) {
  const HandCaptureConfig c = MakeConfig();
  // Reached q_close, pushing anyway (the "little squeeze" past a stall is ρ
  // above rho_max, and so is an empty hand with a PD residual).
  EXPECT_FALSE(Hand::OneJoint(c, 0, 0.95, 1.0).Eval(c).blocked);
  // Never left q_pre, pushing on something that is not the ball.
  EXPECT_FALSE(Hand::OneJoint(c, 0, 0.1, 1.0).Eval(c).blocked);
  // Part-way but not pushing: a finger at rest in free air.
  EXPECT_FALSE(Hand::OneJoint(c, 0, 0.6, 0.3).Eval(c).blocked);
  // Part-way and pushing, but still moving: passing through, not stopped.
  Hand moving = Hand::OneJoint(c, 0, 0.6, 1.0);
  moving.qd[0] = 0.2;
  EXPECT_FALSE(moving.Eval(c).blocked);
  moving.qd[0] = -0.2;  // |q̇|, not q̇
  EXPECT_FALSE(moving.Eval(c).blocked);
}

TEST(HandCapture, TheTorqueMustPointTowardQClose) {
  const HandCaptureConfig c = MakeConfig();
  // Pressed OPEN by something (τ away from q_close) is not holding it.
  Hand h = Hand::OneJoint(c, 0, 0.6, 1.0);
  h.tau[0] = -h.tau[0];
  const HandCaptureReading r = h.Eval(c);
  EXPECT_FALSE(r.blocked);
  EXPECT_NEAR(r.effort_frac_max, 0.0, 1e-12) << "the other joints hold nothing";
  Hand neg = Hand::OneJoint(c, 1, 0.6, 1.0);
  neg.tau[1] = -neg.tau[1];
  EXPECT_FALSE(neg.Eval(c).blocked);
}

TEST(HandCapture, TheBoundsAreInclusive) {
  const HandCaptureConfig c = MakeConfig();
  // Joint 3 is off the caging set, so ρ is exact on joint 0 (span 1.0).
  EXPECT_TRUE(Hand::OneJoint(c, 0, 0.9, 0.5).Eval(c).blocked);
  EXPECT_TRUE(Hand::OneJoint(c, 0, 0.2, 0.5).Eval(c).blocked);
  Hand at_tol = Hand::OneJoint(c, 0, 0.6, 1.0);
  at_tol.qd[0] = 0.05;
  EXPECT_TRUE(at_tol.Eval(c).blocked);
  EXPECT_FALSE(Hand::OneJoint(c, 0, 0.9 + 1e-9, 0.5).Eval(c).blocked);
  EXPECT_FALSE(Hand::OneJoint(c, 0, 0.2 - 1e-9, 0.5).Eval(c).blocked);
  EXPECT_FALSE(Hand::OneJoint(c, 0, 0.6, 0.5 - 1e-9).Eval(c).blocked);
}

TEST(HandCapture, AJointOffTheCagingSetNeverVotes) {
  const HandCaptureConfig c = MakeConfig();
  const HandCaptureReading r = Hand::OneJoint(c, 3, 0.6, 1.0).Eval(c);
  EXPECT_FALSE(r.blocked);
  EXPECT_NEAR(r.effort_frac_max, 0.0, 1e-12) << "nor does its torque";
}

TEST(HandCapture, MinJointsCountsStalledJoints) {
  HandCaptureConfig c = MakeConfig();
  c.min_joints = 2;
  ASSERT_TRUE(c.Valid());
  EXPECT_FALSE(Hand::OneJoint(c, 0, 0.6, 1.0).Eval(c).blocked);
  Hand two = Hand::OneJoint(c, 0, 0.6, 1.0);
  const Hand j1 = Hand::OneJoint(c, 1, 0.5, 0.8);
  two.q[1] = j1.q[1];
  two.tau[1] = j1.tau[1];
  const HandCaptureReading r = two.Eval(c);
  EXPECT_TRUE(r.blocked);
  EXPECT_EQ(r.stalled, 2);
  EXPECT_NEAR(r.effort_frac_max, 1.0, 1e-12);
}

// ── Fail-closed ──────────────────────────────────────────────────────────────

TEST(HandCapture, ANonFiniteReadingNeverStalls) {
  const HandCaptureConfig c = MakeConfig();
  for (int field = 0; field < 3; ++field) {
    Hand h = Hand::OneJoint(c, 0, 0.6, 1.0);
    (field == 0 ? h.q : field == 1 ? h.qd : h.tau)[0] = kNan;
    EXPECT_FALSE(h.Eval(c).blocked) << "field " << field;
  }
  Hand inf = Hand::OneJoint(c, 0, 0.6, 1.0);
  inf.tau[0] = std::numeric_limits<double>::infinity();
  const HandCaptureReading r = inf.Eval(c);
  EXPECT_FALSE(r.blocked) << "an infinite torque is not a measurement";
  EXPECT_NEAR(r.effort_frac_max, 0.0, 1e-12) << "and is not recorded as one";
}

TEST(HandCapture, ANarrowLaneIsNotReadable) {
  const HandCaptureConfig c = MakeConfig();
  const Hand h = Hand::OneJoint(c, 0, 0.6, 1.0);
  const auto n = static_cast<std::size_t>(c.dof);
  for (int lane = 0; lane < 3; ++lane) {
    const HandCaptureReading r =
        EvaluateHandCapture(c, std::span<const double>(h.q.data(), lane == 0 ? n - 1 : n),
                            std::span<const double>(h.qd.data(), lane == 1 ? n - 1 : n),
                            std::span<const double>(h.tau.data(), lane == 2 ? n - 1 : n));
    EXPECT_FALSE(r.blocked) << "lane " << lane;
    EXPECT_TRUE(std::isnan(r.effort_frac_max)) << "not evaluated, not zero";
  }
  EXPECT_FALSE(EvaluateHandCapture(c, {}, {}, {}).blocked);
}

TEST(HandCapture, AnInvalidConfigAnswersNotBlocked) {
  const auto blocked_under = [](const HandCaptureConfig& c) {
    return Hand::OneJoint(MakeConfig(), 0, 0.6, 1.0).Eval(c).blocked;
  };
  ASSERT_TRUE(blocked_under(MakeConfig()));
  HandCaptureConfig c = MakeConfig();
  c.rho_max = 1.0;  // would count the empty hand
  EXPECT_FALSE(c.Valid());
  EXPECT_FALSE(blocked_under(c));
  c = MakeConfig();
  c.rho_min = c.rho_max;
  EXPECT_FALSE(c.Valid());
  c = MakeConfig();
  c.rho_min = kNan;
  EXPECT_FALSE(c.Valid());
  c = MakeConfig();
  c.effort_frac_min = 0.0;
  EXPECT_FALSE(c.Valid());
  c = MakeConfig();
  c.effort_frac_min = 1.5;
  EXPECT_FALSE(c.Valid());
  c = MakeConfig();
  c.tau_max[1] = 0.0;  // a caging joint without a torque scale
  EXPECT_FALSE(c.Valid());
  EXPECT_FALSE(blocked_under(c));
  c = MakeConfig();
  c.tau_max[3] = 0.0;  // off the caging set: not needed
  EXPECT_TRUE(c.Valid());
  c = MakeConfig();
  c.q_close[2] = c.q_pre[2];
  EXPECT_FALSE(c.Valid());
  c = MakeConfig();
  c.min_joints = 4;  // only three caging joints
  EXPECT_FALSE(c.Valid());
  c = MakeConfig();
  c.min_joints = 0;
  EXPECT_FALSE(c.Valid());
  c = MakeConfig();
  c.dof = 0;
  EXPECT_FALSE(c.Valid());
}

TEST(HandCapture, FromProfileNeedsTheBlockAndEveryTorqueLimit) {
  HandProfile p{};
  p.dof = 2;
  p.tbd = false;
  p.q_pre = {0.0, 0.0};
  p.q_close = {1.0, -1.0};
  p.caging_mask = {true, true};
  p.qd_tol = 0.05;
  p.capture.enabled = true;
  p.capture.rho_min = TbdDouble::Resolved(0.2);
  p.capture.rho_max = TbdDouble::Resolved(0.9);
  p.capture.effort_frac_min = TbdDouble::Resolved(0.5);
  p.capture.t_persist = TbdDouble::Resolved(0.1);
  const std::array<double, 2> tau{3.0, 2.0};
  const HandCaptureConfig c = HandCaptureConfig::FromProfile(p, tau);
  ASSERT_TRUE(c.Valid());
  EXPECT_EQ(c.tau_max[1], 2.0);
  EXPECT_EQ(c.qd_tol, 0.05);
  EXPECT_FALSE(HandCaptureConfig::FromProfile(p, std::span<const double>(tau.data(), 1)).Valid())
      << "a torque list shorter than the hand";
  HandProfile off = p;
  off.capture.enabled = false;
  EXPECT_FALSE(HandCaptureConfig::FromProfile(off, tau).Valid());
  HandProfile tbd = p;
  tbd.capture.effort_frac_min = TbdDouble{};
  EXPECT_FALSE(HandCaptureConfig::FromProfile(tbd, tau).Valid());
  HandProfile offset = p;
  offset.hold_mode = rtc::catching::HandHoldMode::kMeasuredOffset;
  EXPECT_FALSE(HandCaptureConfig::FromProfile(offset, tau).Valid())
      << "an empty hand does not reach ρ = 1 under measured_offset";
}

// ── RT: no allocation (G7-D's pure half) ────────────────────────────────────

TEST(HandCapture, EvaluateDoesNotAllocate) {
  const HandCaptureConfig c = MakeConfig();
  const Hand h = Hand::OneJoint(c, 0, 0.6, 1.0);
  const auto n = static_cast<std::size_t>(c.dof);
  bool blocked = false;
  {
    rtc::testing::ScopedAllocGate heap_gate;
    rtc::testing::ScopedNoMalloc eigen_gate;
    for (int i = 0; i < 1000; ++i) {
      blocked = EvaluateHandCapture(c, std::span<const double>(h.q.data(), n),
                                    std::span<const double>(h.qd.data(), n),
                                    std::span<const double>(h.tau.data(), n))
                    .blocked;
    }
  }
  EXPECT_TRUE(blocked);
}

// ── G7-C: synthetic noise (L7 §9 — a false-alarm RATE, recorded) ─────────────
//
// The sim hand lanes carry no noise, so "false-Captured 0" in sim says nothing
// about a real hand. This is the noise half: an EMPTY hand — resting at
// q_close (what an empty close ends in) and at q_pre (a hand that never
// closed) — under gaussian noise on q, q̇ and τ, evaluated per sample. The
// noise is an assumption, not a measurement: σ_q 5 mrad, σ_q̇ 0.02 rad/s,
// σ_τ 10 % of max_torque. A positive control (a joint stalled at ρ 0.6 and
// pushing at 0.9 under the same noise) proves the evaluation can fire.

struct NoiseCounts {
  int fired{0};
  int samples{0};

  [[nodiscard]] double Rate() const { return static_cast<double>(fired) / samples; }
};

NoiseCounts RunNoise(const HandCaptureConfig& c, const Hand& clean, unsigned seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> nq(0.0, 0.005);
  std::normal_distribution<double> nqd(0.0, 0.02);
  std::normal_distribution<double> ntau(0.0, 0.1);
  NoiseCounts out;
  constexpr int kSamples = 20000;
  for (int s = 0; s < kSamples; ++s) {
    Hand h = clean;
    for (int i = 0; i < c.dof; ++i) {
      const auto u = static_cast<std::size_t>(i);
      h.q[u] += nq(rng);
      h.qd[u] += nqd(rng);
      h.tau[u] += ntau(rng) * c.tau_max[u];
    }
    out.fired += h.Eval(c).blocked ? 1 : 0;
    ++out.samples;
  }
  return out;
}

std::string Fixed(double v) {
  // Not std::to_string: it prints a small rate as 0.000000.
  std::ostringstream os;
  os << std::scientific << std::setprecision(3) << v;
  return os.str();
}

TEST(HandCaptureG7C, AnEmptyHandUnderNoiseDoesNotReadAsHolding) {
  const HandCaptureConfig c = MakeConfig();
  const NoiseCounts at_close = RunNoise(c, Hand(c, 1.0, 0.0), 101u);
  const NoiseCounts at_pre = RunNoise(c, Hand(c, 0.0, 0.0), 102u);
  const NoiseCounts positive = RunNoise(c, Hand::OneJoint(c, 0, 0.6, 0.9), 103u);
  RecordProperty("empty_at_q_close_false_alarm_rate", Fixed(at_close.Rate()));
  RecordProperty("empty_at_q_pre_false_alarm_rate", Fixed(at_pre.Rate()));
  RecordProperty("stalled_detection_rate", Fixed(positive.Rate()));
  EXPECT_EQ(at_close.fired, 0) << "rate " << Fixed(at_close.Rate());
  EXPECT_EQ(at_pre.fired, 0) << "rate " << Fixed(at_pre.Rate());
  // Per sample, the positive control is bounded by the q̇ clause: |N(0, 0.02)|
  // ≤ 0.05 holds with p ≈ 0.988. (A t_persist run needs every sample: over
  // 0.1 s at 500 Hz that is ≈ 0.988^50 ≈ 0.54 — a real hand's q̇ noise is an
  // S10 question for qd_tol and t_persist, not one the sim can answer.)
  EXPECT_GT(positive.Rate(), 0.97) << "the positive control must fire under the same noise";
}

}  // namespace
