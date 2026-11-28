// ── Sequencer: the support-height pin ────────────────────────────────────────
//
//   off          Init without a ground and with a disabled one: FSM + counters +
//                core bit-equal to each other over a gait (the pin is the only
//                difference the option makes)
//   when         one pin per stance release, on the removal tick, none on any
//                other exit (slip; a candidate's and an unknown's release) or tick
//   gate         a plane far from every anchor: max_gated gated pins, then an
//                inflated one, and the count restarts
//   Init         every refused ground field; a disabled ground is not checked
//   bias         a gait whose stance point rises while it is held (the sink
//                released): the height drifts without the pin and stays bounded
//                with it
#include "contact_fsm_testing.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace rtc::estimation::testing {
namespace {

using Access = ContactFsmTestAccess;

ContactInekfSequencer::GroundConfig Ground(double sigma = 1e-3, double height = 0.0,
                                           int max_gated = 3) {
  ContactInekfSequencer::GroundConfig g;
  g.enabled = true;
  g.height = height;
  g.sigma = sigma;
  g.gate_chi2 = 10.83;
  g.max_gated = max_gated;
  return g;
}

/// A sample stream of `sc` and the stationary start every run shares.
struct Stream {
  std::vector<Sample> samples;  // index k − 1
  StationaryImuAccumulator acc;
  std::array<Vec3, 2> pts;
  double z0{0.0};  // the true mean stance height at the start (the estimate's origin)
};

Stream MakeStream(const Scenario& sc, std::uint64_t seed) {
  Stream s;
  SyntheticWorld w(sc, seed);
  const int n = static_cast<int>(sc.seconds / kPeriod);
  for (int k = 1; k < n; ++k) {
    s.samples.push_back(w.At(k));
  }
  for (int k = 0; k < kInitTicks; ++k) {
    (void)s.acc.Add(s.samples[static_cast<std::size_t>(k)].gyro,
                    s.samples[static_cast<std::size_t>(k)].accel);
  }
  s.pts = {s.samples[kInitTicks - 1].kin[0].position, s.samples[kInitTicks - 1].kin[1].position};
  const double t0 = kInitTicks * kPeriod;
  s.z0 = 0.5 * (Foot(0, t0, sc).z() + Foot(1, t0, sc).z());
  return s;
}

void Start(ContactInekfSequencer& seq, const Stream& s) {
  ASSERT_EQ(seq.Initialize(s.acc.MeanGyro(), s.acc.MeanAccel(), kInitTicks * kPeriod, 0.0, s.pts),
            InekfStatus::kOk);
}

ContactFsmStatus Step(ContactInekfSequencer& seq, const Sample& x) {
  ContactStepInput in;
  in.gyro = x.gyro;
  in.accel = x.accel;
  in.kinematics = std::span<const ContactKinematics>(x.kin.data(), 2);
  in.joint_velocity = std::span<const Vec3>(x.rdot.data(), 2);
  in.planned_mask = x.planned;
  return seq.Step(in);
}

std::vector<double> State(const ContactInekfSequencer& s) {
  std::vector<double> v = Access::SequencerState(s);
  const Snapshot c = Capture(s.core());
  v.insert(v.end(), c.values.begin(), c.values.end());
  return v;
}

TEST(ContactFsmGround, ADisabledGroundChangesNothing) {
  const Scenario sc{World::kGait, Shift::kNone, 12.0};
  const Stream s = MakeStream(sc, kSeed + 21);
  ContactInekfSequencer a;
  ContactInekfSequencer b;
  a.Init(CoreFixture(), FsmFixture(), 2);
  ContactInekfSequencer::GroundConfig off = Ground();
  off.enabled = false;
  b.Init(CoreFixture(), FsmFixture(), 2, off);
  Start(a, s);
  Start(b, s);
  for (std::size_t k = kInitTicks; k < s.samples.size(); ++k) {
    ASSERT_EQ(Step(a, s.samples[k]), ContactFsmStatus::kOk);
    ASSERT_EQ(Step(b, s.samples[k]), ContactFsmStatus::kOk);
    ASSERT_TRUE(SameBits(State(a), State(b))) << k;
    for (int i = 0; i < 2; ++i) {
      ASSERT_EQ(b.ground_report(i).result, GroundUpdateResult::kNone);
    }
  }
}

TEST(ContactFsmGround, OnePinPerStanceReleaseAndNoOther) {
  for (const Shift shift : {Shift::kNone, Shift::kPlannedOffEarly, Shift::kPlannedOffLate}) {
    const Scenario sc{World::kGait, shift, 12.0};
    const Stream s = MakeStream(sc, kSeed + 22);
    ContactInekfSequencer seq;
    seq.Init(CoreFixture(), FsmFixture(), 2, Ground());
    Start(seq, s);
    int releases = 0;
    int pins = 0;
    int other_exits = 0;
    for (std::size_t k = kInitTicks; k < s.samples.size(); ++k) {
      std::array<ContactMode, 2> before{seq.fsm().mode(0), seq.fsm().mode(1)};
      ASSERT_EQ(Step(seq, s.samples[k]), ContactFsmStatus::kOk);
      for (int i = 0; i < 2; ++i) {
        const auto si = static_cast<std::size_t>(i);
        const bool released = before[si] == ContactMode::kStance &&
                              seq.fsm().mode(i) == ContactMode::kSwing &&
                              seq.fsm().reason(i) == ContactReason::kReleased;
        const bool pinned = seq.ground_report(i).result != GroundUpdateResult::kNone;
        releases += released ? 1 : 0;
        pins += pinned ? 1 : 0;
        EXPECT_EQ(pinned, released) << k << ' ' << i;
        EXPECT_TRUE(!pinned || seq.action(i).remove);
        other_exits +=
            !released && ContactFsm::HoldsAnchor(before[si]) && seq.action(i).remove ? 1 : 0;
      }
    }
    if (shift == Shift::kPlannedOffLate) {
      // The foot lifts before its plan bit drops: every exit is a slip, never pinned.
      EXPECT_EQ(releases, 0);
      EXPECT_GT(other_exits, 20);
    } else {
      EXPECT_GT(releases, 20);  // ≈ 2 per 0.8 s cycle over 11 s
    }
    EXPECT_EQ(pins, releases);
    std::printf("shift %d: releases %d, other anchor exits %d\n", static_cast<int>(shift), releases,
                other_exits);
  }
}

TEST(ContactFsmGround, NoPinOnACandidateOrUnknownRelease) {
  // Candidate: drop slot 0's plan bit right after its insertion, before it can
  // be confirmed. Unknown: a 2 cm kinematic step (kFkStep) sends slot 0 to
  // unknown; drop its plan bit there.
  for (const World w : {World::kStatic, World::kFkStep}) {
    const Scenario sc{w, Shift::kNone, 4.0};
    const Stream s = MakeStream(sc, kSeed + 25);
    ContactInekfSequencer seq;
    seq.Init(CoreFixture(), FsmFixture(), 2, Ground());
    Start(seq, s);
    const ContactMode target =
        w == World::kStatic ? ContactMode::kCandidate : ContactMode::kUnknown;
    bool dropping = false;
    int exits = 0;
    for (std::size_t k = kInitTicks; k < s.samples.size() && exits == 0; ++k) {
      Sample x = s.samples[k];
      const ContactMode before = seq.fsm().mode(0);
      dropping = dropping || before == target;
      if (dropping) {
        x.planned &= ~1U;
      }
      ASSERT_EQ(Step(seq, x), ContactFsmStatus::kOk);
      if (before == target && seq.fsm().mode(0) == ContactMode::kSwing) {
        ++exits;
        EXPECT_EQ(seq.fsm().reason(0), ContactReason::kReleased);
        EXPECT_TRUE(seq.action(0).remove);
      }
      for (int i = 0; i < 2; ++i) {
        EXPECT_EQ(seq.ground_report(i).result, GroundUpdateResult::kNone) << k << ' ' << i;
      }
    }
    EXPECT_EQ(exits, 1) << static_cast<int>(w);  // the case was reached
  }
}

TEST(ContactFsmGround, GatedPinsInflateAfterMaxGatedAndRestart) {
  const Scenario sc{World::kGait, Shift::kNone, 12.0};
  const Stream s = MakeStream(sc, kSeed + 23);
  ContactInekfSequencer seq;
  // A plane 5 cm under every anchor, held to 0.1 mm: every plain pin is gated.
  seq.Init(CoreFixture(), FsmFixture(), 2, Ground(1e-4, -0.05, 3));
  Start(seq, s);
  std::vector<GroundUpdateResult> results;
  for (std::size_t k = kInitTicks; k < s.samples.size(); ++k) {
    ASSERT_EQ(Step(seq, s.samples[k]), ContactFsmStatus::kOk);
    for (int i = 0; i < 2; ++i) {
      if (seq.ground_report(i).result != GroundUpdateResult::kNone) {
        results.push_back(seq.ground_report(i).result);
        const int want =
            results.back() == GroundUpdateResult::kGated ? static_cast<int>(results.size() % 4) : 0;
        EXPECT_EQ(seq.consecutive_gated_pins(), want);
      }
    }
  }
  ASSERT_GE(results.size(), 12U);
  for (std::size_t q = 0; q < 12; ++q) {
    EXPECT_EQ(results[q], (q % 4 == 3) ? GroundUpdateResult::kInflated : GroundUpdateResult::kGated)
        << q;
  }
}

TEST(ContactFsmGround, InitRefusesABadGroundAndChecksNoDisabledOne) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  auto refused = [](const ContactInekfSequencer::GroundConfig& g) {
    ContactInekfSequencer seq;
    try {
      seq.Init(CoreFixture(), FsmFixture(), 2, g);
    } catch (const std::invalid_argument&) {
      return true;
    }
    return false;
  };
  EXPECT_FALSE(refused(Ground()));
  for (double h : {nan, inf}) {
    EXPECT_TRUE(refused(Ground(1e-3, h)));
  }
  for (double sigma : {0.0, -1e-3, nan, inf, 1e-170, 1e200}) {
    EXPECT_TRUE(refused(Ground(sigma)));
  }
  for (double gate : {0.0, -1.0, nan}) {
    ContactInekfSequencer::GroundConfig g = Ground();
    g.gate_chi2 = gate;
    EXPECT_TRUE(refused(g));
  }
  EXPECT_TRUE(refused(Ground(1e-3, 0.0, 0)));
  ContactInekfSequencer::GroundConfig off = Ground(nan, nan, 0);
  off.enabled = false;
  EXPECT_FALSE(refused(off));
}

/// z error of the IMU estimate against the truth on the estimate's origin.
std::vector<double> HeightError(const Scenario& sc, const Stream& s,
                                const ContactInekfSequencer::GroundConfig* ground) {
  ContactInekfSequencer seq;
  if (ground != nullptr) {
    seq.Init(CoreFixture(), FsmFixture(), 2, *ground);
  } else {
    seq.Init(CoreFixture(), FsmFixture(), 2);
  }
  Start(seq, s);
  std::vector<double> e;
  for (std::size_t k = kInitTicks; k < s.samples.size(); ++k) {
    if (Step(seq, s.samples[k]) != ContactFsmStatus::kOk) {
      return {};
    }
    const double t = static_cast<double>(k + 1) * kPeriod;
    e.push_back(seq.core().position().z() - (ImuPos(t, sc).z() - s.z0));
  }
  return e;
}

TEST(ContactFsmGround, ARisingStancePointDriftsTheHeightAndThePinHoldsIt) {
  constexpr double kRise = 1.5e-3;
  Scenario flat{World::kGait, Shift::kNone, 12.0};
  Scenario rising = flat;
  rising.stance_rise = kRise;
  const Stream sf = MakeStream(flat, kSeed + 24);
  const Stream sr = MakeStream(rising, kSeed + 24);  // same noise draws
  const std::vector<double> free_flat = HeightError(flat, sf, nullptr);
  const std::vector<double> free_rising = HeightError(rising, sr, nullptr);
  // The plane where the ground is in the estimate's frame (its origin is the
  // start's mean stance height): the test is the hold, not the offset of a
  // plane taken at a sunk stance.
  const ContactInekfSequencer::GroundConfig g = Ground(1e-3, -sr.z0);
  const std::vector<double> pinned = HeightError(rising, sr, &g);
  ASSERT_FALSE(free_flat.empty());
  ASSERT_FALSE(free_rising.empty());
  ASSERT_FALSE(pinned.empty());
  const std::size_t from = static_cast<std::size_t>(3.0 / kPeriod);  // after the gait ramps up
  double worst = 0.0;
  for (std::size_t k = from; k < pinned.size(); ++k) {
    worst = std::max(worst, std::abs(pinned[k]));
  }
  const double effect = free_rising.back() - free_flat.back();
  std::printf(
      "z error at the end: free flat %.2f · free rising %.2f (effect %.2f) · pinned %.2f "
      "(worst %.2f) mm\n",
      1e3 * free_flat.back(), 1e3 * free_rising.back(), 1e3 * effect, 1e3 * pinned.back(),
      1e3 * worst);
  // Without the pin the rise reads as the base sinking, a little every stance
  // (this world drifts by itself too — the effect is the difference).
  EXPECT_LT(effect, -4.0 * kRise);
  // With it the height stays within about one rise of the truth.
  EXPECT_LT(worst, 2.0 * kRise);
  EXPECT_LT(std::abs(pinned.back()), 1.5 * kRise);
}

}  // namespace
}  // namespace rtc::estimation::testing
