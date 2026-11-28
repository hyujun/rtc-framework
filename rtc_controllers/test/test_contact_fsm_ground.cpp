// ── Sequencer: the support-height pin ────────────────────────────────────────
//
//   off          Init without a ground and with a disabled one: FSM + counters +
//                core bit-equal to each other over a gait (the pin is the only
//                difference the option makes)
//   when         one pin per anchor life, on its first stance tick settle_s
//                after the insertion; a life that ends sooner (a candidate
//                released, an unknown one) has none
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
constexpr double kSettle = 0.08;  // [s]
constexpr int kSettleTicks = 40;  // ⌈kSettle / kPeriod⌉

ContactInekfSequencer::GroundConfig Ground(double sigma = 1e-3, double height = 0.0,
                                           int max_gated = 3) {
  ContactInekfSequencer::GroundConfig g;
  g.enabled = true;
  g.height = height;
  g.sigma = sigma;
  g.gate_chi2 = 10.83;
  g.max_gated = max_gated;
  g.settle_s = kSettle;
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

/// Per slot and anchor life: ticks since insertion, pins, and whether it was
/// ever in stance once settled. Checks each tick's pin against the rule.
struct LifeTracker {
  std::array<int, 2> age{};
  std::array<int, 2> pins{};
  std::array<bool, 2> live{};
  std::array<bool, 2> due{};  // reached stance with age ≥ settle this life
  int lives_pinned = 0;
  int lives_due = 0;
  int violations = 0;

  void Close(int i) {
    const auto si = static_cast<std::size_t>(i);
    if (live[si]) {
      lives_pinned += pins[si] > 0 ? 1 : 0;
      lives_due += due[si] ? 1 : 0;
      violations += (pins[si] > 1 || (pins[si] == 1) != due[si]) ? 1 : 0;
    }
    live[si] = false;
  }

  void Tick(const ContactInekfSequencer& seq) {
    for (int i = 0; i < 2; ++i) {
      const auto si = static_cast<std::size_t>(i);
      if (seq.action(i).insert) {
        Close(i);
        live[si] = true;
        age[si] = 0;
        pins[si] = 0;
        due[si] = false;
      } else if (live[si] && seq.core().contact_active(i)) {
        ++age[si];
      }
      const bool stance = seq.fsm().mode(i) == ContactMode::kStance;
      const bool pinned = seq.ground_report(i).result != GroundUpdateResult::kNone;
      if (pinned) {
        ++pins[si];
        // On the first tick the rule allows: in stance, settled, none before.
        violations += (!live[si] || !stance || age[si] < kSettleTicks || due[si]) ? 1 : 0;
      }
      if (live[si] && stance && age[si] >= kSettleTicks) {
        due[si] = true;
      }
      if (live[si] && !seq.core().contact_active(i)) {
        Close(i);
      }
    }
  }
};

TEST(ContactFsmGround, OnePinPerAnchorLifeOnceSettled) {
  for (const Shift shift : {Shift::kNone, Shift::kPlannedOffEarly, Shift::kPlannedOffLate}) {
    const Scenario sc{World::kGait, shift, 12.0};
    const Stream s = MakeStream(sc, kSeed + 22);
    ContactInekfSequencer seq;
    seq.Init(CoreFixture(), FsmFixture(), 2, Ground());
    Start(seq, s);
    LifeTracker lt;
    for (std::size_t k = kInitTicks; k < s.samples.size(); ++k) {
      ASSERT_EQ(Step(seq, s.samples[k]), ContactFsmStatus::kOk);
      lt.Tick(seq);
    }
    EXPECT_EQ(lt.violations, 0) << static_cast<int>(shift);
    EXPECT_GT(lt.lives_pinned, 20) << static_cast<int>(shift);  // ≈ 2 per 0.8 s cycle
    EXPECT_EQ(lt.lives_pinned, lt.lives_due) << static_cast<int>(shift);
  }
}

TEST(ContactFsmGround, ALifeThatEndsBeforeSettlingHasNoPin) {
  // Candidate: drop slot 0's plan bit right after its insertion, before it can
  // be confirmed. Unknown: a 2 cm kinematic step (kFkStep) sends slot 0 to
  // unknown long after its pin; drop its plan bit there — no second pin.
  for (const World w : {World::kStatic, World::kFkStep}) {
    const Scenario sc{w, Shift::kNone, 4.0};
    const Stream s = MakeStream(sc, kSeed + 25);
    ContactInekfSequencer seq;
    seq.Init(CoreFixture(), FsmFixture(), 2, Ground());
    Start(seq, s);
    const ContactMode target =
        w == World::kStatic ? ContactMode::kCandidate : ContactMode::kUnknown;
    LifeTracker lt;
    bool dropping = false;
    int exits = 0;
    int pins_slot0 = 0;
    for (std::size_t k = kInitTicks; k < s.samples.size() && exits == 0; ++k) {
      Sample x = s.samples[k];
      const ContactMode before = seq.fsm().mode(0);
      dropping = dropping || before == target;
      if (dropping) {
        x.planned &= ~1U;
      }
      ASSERT_EQ(Step(seq, x), ContactFsmStatus::kOk);
      lt.Tick(seq);
      pins_slot0 += seq.ground_report(0).result != GroundUpdateResult::kNone ? 1 : 0;
      if (before == target && seq.fsm().mode(0) == ContactMode::kSwing) {
        ++exits;
        EXPECT_EQ(seq.fsm().reason(0), ContactReason::kReleased);
        EXPECT_TRUE(seq.action(0).remove);
      }
    }
    EXPECT_EQ(exits, 1) << static_cast<int>(w);  // the case was reached
    EXPECT_EQ(lt.violations, 0) << static_cast<int>(w);
    // The candidate never settled; the unknown one was pinned once, before.
    EXPECT_EQ(pins_slot0, w == World::kStatic ? 0 : 1) << static_cast<int>(w);
  }
}

TEST(ContactFsmGround, AnAnchorStillACandidateWhenSettledWaitsForItsConfirmation) {
  // Confirmation slower than the settle time: at 80 ms every new anchor is still
  // a candidate; its pin comes on its first stance tick, not before.
  const Scenario sc{World::kGait, Shift::kNone, 8.0};
  const Stream s = MakeStream(sc, kSeed + 26);
  ContactFsm::Config fsm = FsmFixture();
  fsm.confirm_dwell = 0.2;
  ContactInekfSequencer seq;
  seq.Init(CoreFixture(), fsm, 2, Ground());
  Start(seq, s);
  LifeTracker lt;
  int candidates_past_settle = 0;
  for (std::size_t k = kInitTicks; k < s.samples.size(); ++k) {
    ASSERT_EQ(Step(seq, s.samples[k]), ContactFsmStatus::kOk);
    lt.Tick(seq);
    for (int i = 0; i < 2; ++i) {
      const auto si = static_cast<std::size_t>(i);
      candidates_past_settle +=
          lt.live[si] && lt.age[si] == kSettleTicks && seq.fsm().mode(i) == ContactMode::kCandidate
              ? 1
              : 0;
    }
  }
  EXPECT_GT(candidates_past_settle, 5);  // the case was reached
  EXPECT_EQ(lt.violations, 0);
  EXPECT_GT(lt.lives_pinned, 5);
}

TEST(ContactFsmGround, AMismatchedStepPinsNothing) {
  // The tick the first pin is due on (a dry run), then the same stream with the
  // core desynchronised on that tick: kStateMismatch and no pin.
  const Scenario sc{World::kStatic, Shift::kNone, 3.0};
  const Stream s = MakeStream(sc, kSeed + 27);
  std::size_t due = 0;
  {
    ContactInekfSequencer dry;
    dry.Init(CoreFixture(), FsmFixture(), 2, Ground());
    Start(dry, s);
    for (std::size_t k = kInitTicks; k < s.samples.size() && due == 0; ++k) {
      ASSERT_EQ(Step(dry, s.samples[k]), ContactFsmStatus::kOk);
      due = dry.ground_report(0).result != GroundUpdateResult::kNone ? k : 0;
    }
  }
  ASSERT_GT(due, 0U);
  ContactInekfSequencer seq;
  seq.Init(CoreFixture(), FsmFixture(), 2, Ground());
  Start(seq, s);
  for (std::size_t k = kInitTicks; k < due; ++k) {
    ASSERT_EQ(Step(seq, s.samples[k]), ContactFsmStatus::kOk);
  }
  ASSERT_EQ(Access::Core(seq).RemoveContact(1), InekfStatus::kOk);  // FSM still holds it
  EXPECT_EQ(Step(seq, s.samples[due]), ContactFsmStatus::kStateMismatch);
  for (int i = 0; i < 2; ++i) {
    EXPECT_EQ(seq.ground_report(i).result, GroundUpdateResult::kNone) << i;
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
    bool any = false;
    for (int i = 0; i < 2; ++i) {  // both feet can settle on one tick (the start's stance)
      if (seq.ground_report(i).result != GroundUpdateResult::kNone) {
        results.push_back(seq.ground_report(i).result);
        any = true;
      }
    }
    if (any) {  // the count after this tick's pins: gated ones since the last inflated
      const int want =
          results.back() == GroundUpdateResult::kGated ? static_cast<int>(results.size() % 4) : 0;
      EXPECT_EQ(seq.consecutive_gated_pins(), want) << k;
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
  for (double settle : {0.0, -0.1, nan, inf, 1e6 * kPeriod * 1.001}) {
    ContactInekfSequencer::GroundConfig g = Ground();
    g.settle_s = settle;
    EXPECT_TRUE(refused(g)) << settle;
  }
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
