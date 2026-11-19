// ── Sequencer on the real core: refusals, failure rows, outputs, determinism ──
//
//   Init         either configuration's refusal, (s_u σ_d)² overflow, passing control
//   refusals     before Init / Initialize / after Reset, k < 1, span sizes:
//                status, and the FSM + core bit-unchanged
//   Propagate    refused (k·T beyond the core's step range, non-finite IMU):
//                kPropagateRefused, FSM + core bit-unchanged
//   Insert       refused by the core: the slot stays in swing, kInsertRefused
//   Update       refused by the core: no report — the NIS streak does not move
//   mismatch     FSM anchor ⇔ core slot disagreement: kStateMismatch, no update;
//                Reset + Initialize recovers
//   outputs      ticks without a confirmed anchor and consecutive gated ticks,
//                recomputed here from modes and reports
//   determinism  two sequencers, 10⁴ ticks: FSM + counters + core bit-equal,
//                reports field-equal; Reset + Initialize replays the run
#include "contact_fsm_testing.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace rtc::estimation::testing {
namespace {

using Access = ContactFsmTestAccess;

/// A sequencer fed by the static synthetic world, initialised after the
/// stationary window, with hooks to edit each sample before it is stepped.
class Rig {
 public:
  explicit Rig(int slots = 2, World w = World::kStatic)
      : sc_{w, Shift::kNone, 60.0}, world_(sc_, kSeed), slots_(slots) {
    seq_.Init(CoreFixture(), FsmFixture(), slots_);
    StationaryImuAccumulator acc;
    for (k_ = 1; k_ <= kInitTicks; ++k_) {
      last_ = world_.At(k_);
      (void)acc.Add(last_.gyro, last_.accel);
    }
    std::array<Vec3, kInekfMaxContacts> pts;
    for (int i = 0; i < slots_; ++i) {
      pts[static_cast<std::size_t>(i)] = last_.kin[static_cast<std::size_t>(i)].position;
    }
    init_gyro_ = acc.MeanGyro();
    init_accel_ = acc.MeanAccel();
    init_pts_ = pts;
    Reinitialize();
  }

  InekfStatus Reinitialize() {
    return seq_.Initialize(
        init_gyro_, init_accel_, kInitTicks * kPeriod, 0.0,
        std::span<const Vec3>(init_pts_.data(), static_cast<std::size_t>(slots_)));
  }

  /// Next world sample (not yet stepped).
  Sample& Next() {
    last_ = world_.At(k_++);
    return last_;
  }

  ContactFsmStatus Step(Sample& s, int periods = 1) {
    ContactStepInput in;
    in.gyro = s.gyro;
    in.accel = s.accel;
    in.kinematics =
        std::span<const ContactKinematics>(s.kin.data(), static_cast<std::size_t>(slots_));
    in.joint_velocity = std::span<const Vec3>(s.rdot.data(), static_cast<std::size_t>(slots_));
    in.planned_mask = s.planned;
    in.periods = periods;
    return seq_.Step(in);
  }

  ContactFsmStatus StepNext() { return Step(Next()); }

  /// Step clean samples until every slot is in stance (≤ 100 ticks).
  void ToStance() {
    for (int t = 0; t < 100; ++t) {
      ASSERT_EQ(StepNext(), ContactFsmStatus::kOk);
      bool all = true;
      for (int i = 0; i < slots_; ++i) {
        all = all && seq_.fsm().mode(i) == ContactMode::kStance;
      }
      if (all) {
        return;
      }
    }
    FAIL() << "never reached stance";
  }

  ContactInekfSequencer& seq() { return seq_; }

  ContactInekf& core() { return Access::Core(seq_); }

 private:
  Scenario sc_;
  SyntheticWorld world_;
  int slots_;
  int k_{1};
  Sample last_;
  ContactInekfSequencer seq_;
  Vec3 init_gyro_;
  Vec3 init_accel_;
  std::array<Vec3, kInekfMaxContacts> init_pts_;
};

struct Frozen {
  std::vector<double> seq;
  Snapshot core;
};

Frozen Freeze(Rig& r) {
  return {Access::SequencerState(r.seq()), Capture(r.seq().core())};
}

void ExpectUnchanged(const Frozen& a, Rig& r) {
  const Frozen b = Freeze(r);
  EXPECT_TRUE(SameBits(a.seq, b.seq));
  EXPECT_TRUE(BitEqual(a.core, b.core));
}

TEST(ContactFsmSequencer, InitRefusesEitherConfiguration) {
  ContactInekfSequencer s;
  ContactFsm::Config f = FsmFixture();
  f.slip_speed = f.enter_speed;
  EXPECT_THROW(s.Init(CoreFixture(), f, 2), std::invalid_argument);
  ContactInekf::Config c = CoreFixture();
  c.kinematic_sigma = 0.0;
  EXPECT_THROW(s.Init(c, FsmFixture(), 2), std::invalid_argument);
  f = FsmFixture();
  f.unknown_noise_scale = 1e200;  // (s_u σ_d)² overflows
  EXPECT_THROW(s.Init(CoreFixture(), f, 2), std::invalid_argument);
  EXPECT_THROW(s.Init(CoreFixture(), FsmFixture(), 0), std::invalid_argument);
  EXPECT_THROW(s.Init(CoreFixture(), FsmFixture(), kInekfMaxContacts + 1), std::invalid_argument);
  EXPECT_FALSE(s.fsm().configured());
  EXPECT_FALSE(s.core().configured());
  // One slot count sizes both, so they cannot disagree.
  s.Init(CoreFixture(), FsmFixture(), 3);
  EXPECT_EQ(s.num_slots(), 3);
  EXPECT_EQ(s.core().num_slots(), 3);
}

TEST(ContactFsmSequencer, RefusedCallsChangeNothing) {
  std::array<ContactKinematics, 2> kin{};
  std::array<Vec3, 2> rdot{Vec3::Zero(), Vec3::Zero()};
  ContactStepInput in;
  in.accel = Vec3(0.0, 0.0, kG0);
  in.kinematics = kin;
  in.joint_velocity = rdot;
  in.planned_mask = 3U;
  ContactInekfSequencer s;
  EXPECT_EQ(s.Step(in), ContactFsmStatus::kNotConfigured);
  EXPECT_EQ(s.Initialize(Vec3::Zero(), in.accel, 1.0, 0.0, {}), InekfStatus::kNotConfigured);
  s.Init(CoreFixture(), FsmFixture(), 2);
  EXPECT_EQ(s.Step(in), ContactFsmStatus::kNotInitialized);

  Rig r;
  r.ToStance();
  const Frozen before = Freeze(r);
  Sample& x = r.Next();
  EXPECT_EQ(r.Step(x, 0), ContactFsmStatus::kBadPeriods);
  ExpectUnchanged(before, r);
  ContactStepInput bad;
  bad.gyro = x.gyro;
  bad.accel = x.accel;
  bad.kinematics = std::span<const ContactKinematics>(x.kin.data(), 1);
  bad.joint_velocity = std::span<const Vec3>(x.rdot.data(), 2);
  EXPECT_EQ(r.seq().Step(bad), ContactFsmStatus::kSizeMismatch);
  bad.kinematics = std::span<const ContactKinematics>(x.kin.data(), 2);
  bad.joint_velocity = std::span<const Vec3>(x.rdot.data(), 3);
  EXPECT_EQ(r.seq().Step(bad), ContactFsmStatus::kSizeMismatch);
  ExpectUnchanged(before, r);

  // Reset: back to the post-Init state, then refuses until Initialize.
  r.seq().Reset();
  for (int i = 0; i < 2; ++i) {
    EXPECT_EQ(r.seq().fsm().mode(i), ContactMode::kSwing);
    EXPECT_FALSE(r.seq().core().contact_active(i));
  }
  EXPECT_FALSE(r.seq().core().initialized());
  EXPECT_EQ(r.Step(x), ContactFsmStatus::kNotInitialized);
  EXPECT_EQ(r.Reinitialize(), InekfStatus::kOk);
  r.ToStance();
}

TEST(ContactFsmSequencer, RefusedPropagateChangesNothing) {
  Rig r;
  r.ToStance();
  for (int t = 0; t < 3; ++t) {
    ASSERT_EQ(r.StepNext(), ContactFsmStatus::kOk);
  }
  const Frozen before = Freeze(r);
  Sample& x = r.Next();
  EXPECT_EQ(r.Step(x, 11), ContactFsmStatus::kPropagateRefused);  // 22 ms > 20 ms
  EXPECT_EQ(r.seq().core_status(), InekfStatus::kStepOutOfRange);
  EXPECT_FALSE(r.seq().report_valid());  // this tick: no update …
  for (int i = 0; i < 2; ++i) {
    EXPECT_FALSE(r.seq().action(i).use);  // … and no action
    EXPECT_FALSE(r.seq().action(i).insert || r.seq().action(i).remove ||
                 r.seq().action(i).set_scale);
  }
  ExpectUnchanged(before, r);
  Sample y = x;
  y.gyro.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(r.Step(y), ContactFsmStatus::kPropagateRefused);
  EXPECT_EQ(r.seq().core_status(), InekfStatus::kNonFiniteInput);
  ExpectUnchanged(before, r);
  EXPECT_EQ(r.Step(x, 10), ContactFsmStatus::kOk);  // 20 ms: the range end is accepted
  EXPECT_EQ(r.seq().core_status(), InekfStatus::kOk);
}

TEST(ContactFsmSequencer, RefusedInsertKeepsTheSlotInSwing) {
  Rig r;
  // The core already holds slot 0 (a desynchronised caller): the insertion is
  // refused, and the disagreement is reported every tick.
  ContactInekfTestAccess::SetActive(r.core(), 0, true);
  int refusals = 0;
  for (int t = 0; t < 12; ++t) {
    EXPECT_EQ(r.StepNext(), ContactFsmStatus::kStateMismatch);
    EXPECT_FALSE(r.seq().report_valid());
    EXPECT_EQ(r.seq().fsm().mode(0), ContactMode::kSwing);
    if (r.seq().core_status() == InekfStatus::kSlotActive) {  // refused this tick
      ++refusals;
      EXPECT_EQ(r.seq().fsm().reason(0), ContactReason::kInsertRefused);
      EXPECT_FALSE(r.seq().action(0).insert);
      EXPECT_FALSE(r.seq().action(0).use);
    }
  }
  // Enter dwell 5, streak restarted by the revert: refused at ticks 5 and 10.
  EXPECT_EQ(refusals, 2);
  // Recovery is Reset + Initialize.
  r.seq().Reset();
  ASSERT_EQ(r.Reinitialize(), InekfStatus::kOk);
  r.ToStance();
}

TEST(ContactFsmSequencer, RefusedUpdateGivesNoReport) {
  Rig r;
  r.ToStance();
  // Drive slot 0's NIS streak to 2 (two high reports read) with a 2 cm FK
  // offset, then refuse an update: the streak must not move on that tick.
  int high_read = 0;
  for (int t = 0; t < 20 && high_read < 2; ++t) {
    Sample& x = r.Next();
    x.kin[0].position.y() += 0.02;
    ASSERT_EQ(r.Step(x), ContactFsmStatus::kOk);
    const ContactUpdateResult res = r.seq().report(0).result;
    if (res == ContactUpdateResult::kGated ||
        (res == ContactUpdateResult::kUsed && r.seq().report(0).mahalanobis2 > 11.34)) {
      ++high_read;
    } else {
      high_read = 0;
    }
  }
  ASSERT_EQ(high_read, 2);
  ASSERT_EQ(r.seq().fsm().mode(0), ContactMode::kStance);
  // An encoder Jacobian whose JJᵀ overflows: finite input, non-finite S.
  Sample& bad = r.Next();
  bad.kin[0].position.y() += 0.02;
  bad.kin[0].jacobian.setConstant(1e160);
  EXPECT_EQ(r.Step(bad), ContactFsmStatus::kUpdateRefused);
  EXPECT_EQ(r.seq().core_status(), InekfStatus::kNonFiniteResult);
  EXPECT_FALSE(r.seq().report_valid());
  EXPECT_EQ(r.seq().fsm().mode(0), ContactMode::kStance);  // read high #2 → streak 2
  // Next tick reads "no report": still 2, no exit to unknown.
  Sample& x = r.Next();
  x.kin[0].position.y() += 0.02;
  ASSERT_EQ(r.Step(x), ContactFsmStatus::kOk);
  EXPECT_EQ(r.seq().fsm().mode(0), ContactMode::kStance);
}

TEST(ContactFsmSequencer, InsertionAnchorsThisTicksKinematics) {
  Rig r;
  r.ToStance();
  // Slot 0 released, then planned again with its point 5 cm away from where
  // it was last used: the new anchor must come from the inserting tick.
  const Vec3 offset(0.05, 0.0, 0.0);
  bool inserted = false;
  for (int t = 0; t < 30 && !inserted; ++t) {
    Sample& x = r.Next();
    if (t < 5) {
      x.planned &= ~1U;  // released at t = 1
    }
    if (t >= 2) {
      x.kin[0].position += offset;  // only after its last use
    }
    ASSERT_EQ(r.Step(x), ContactFsmStatus::kOk);
    if (t == 1) {
      ASSERT_EQ(r.seq().fsm().mode(0), ContactMode::kSwing);
    }
    if (r.seq().action(0).insert) {
      inserted = true;
      const ContactInekf& c = r.seq().core();
      const Vec3 expect = c.position() + c.rotation() * x.kin[0].position;
      // The same tick's update moves p̂ and d̂_0 by mm, not by the 5 cm offset.
      EXPECT_LT((c.contact_position(0) - expect).norm(), 5e-3);
    }
  }
  EXPECT_TRUE(inserted);
}

TEST(ContactFsmSequencer, NonFiniteJacobianLeavesOnlyThatSlotOut) {
  Rig r;
  r.ToStance();
  Sample& x = r.Next();
  x.kin[0].jacobian(1, 2) = std::numeric_limits<double>::quiet_NaN();
  // The core would refuse the whole update for a requested non-finite slot;
  // the sequencer leaves that slot out and updates the other.
  EXPECT_EQ(r.Step(x), ContactFsmStatus::kOk);
  EXPECT_FALSE(r.seq().action(0).use);
  EXPECT_EQ(r.seq().report(0).result, ContactUpdateResult::kNotRequested);
  EXPECT_EQ(r.seq().report(1).result, ContactUpdateResult::kUsed);
  EXPECT_EQ(r.seq().fsm().mode(0), ContactMode::kStance);
}

TEST(ContactFsmSequencer, DesynchronisedCoreIsReported) {
  Rig r;
  r.ToStance();
  ContactInekfTestAccess::SetActive(r.core(), 1, false);
  EXPECT_EQ(r.StepNext(), ContactFsmStatus::kStateMismatch);
  EXPECT_FALSE(r.seq().report_valid());
  EXPECT_EQ(r.seq().fsm().mode(1), ContactMode::kStance);     // the FSM does not guess
  EXPECT_EQ(r.StepNext(), ContactFsmStatus::kStateMismatch);  // until Reset
  r.seq().Reset();
  ASSERT_EQ(r.Reinitialize(), InekfStatus::kOk);
  r.ToStance();
}

TEST(ContactFsmSequencer, RecoveryOutputsFollowModesAndReports) {
  Rig r;
  int expect_no_anchor = 0;
  int expect_gated = 0;
  auto check = [&](int periods) {
    bool confirmed = false;
    bool used = false;
    bool gated = false;
    for (int i = 0; i < 2; ++i) {
      confirmed = confirmed || ContactFsm::Confirmed(r.seq().fsm().mode(i));
      if (r.seq().report_valid()) {
        used = used || r.seq().report(i).result == ContactUpdateResult::kUsed;
        gated = gated || r.seq().report(i).result == ContactUpdateResult::kGated;
      }
    }
    expect_no_anchor = confirmed ? 0 : expect_no_anchor + periods;
    expect_gated = used ? 0 : (gated ? expect_gated + periods : expect_gated);
    EXPECT_EQ(r.seq().ticks_without_confirmed_anchor(), expect_no_anchor);
    EXPECT_EQ(r.seq().consecutive_gated_ticks(), expect_gated);
  };
  int max_gated = 0;
  int max_no_anchor = 0;
  for (int t = 0; t < 600; ++t) {
    Sample& x = r.Next();
    const int phase = t / 100;
    if (phase == 2) {  // every point 30 cm off: gated
      x.kin[0].position.x() += 0.3;
      x.kin[1].position.x() += 0.3;
    }
    if (phase == 4) {  // nothing planned
      x.planned = 0U;
    }
    const int periods = (t % 7 == 3) ? 2 : 1;
    ASSERT_EQ(r.Step(x, periods), ContactFsmStatus::kOk) << t;
    check(periods);
    max_gated = std::max(max_gated, r.seq().consecutive_gated_ticks());
    max_no_anchor = std::max(max_no_anchor, r.seq().ticks_without_confirmed_anchor());
  }
  EXPECT_GE(max_gated, 3);       // the gated phase was seen
  EXPECT_GE(max_no_anchor, 90);  // the unplanned phase was seen
}

TEST(ContactFsmSequencer, TwoInstancesAndAReplayAreBitIdentical) {
  constexpr int kTicks = 10000;
  const Scenario sc{World::kGait, Shift::kNone, kTicks * kPeriod + 1.0};
  std::vector<Sample> samples;
  samples.reserve(kTicks + kInitTicks + 1);
  SyntheticWorld w(sc, kSeed + 3);
  for (int k = 1; k <= kTicks + kInitTicks; ++k) {
    samples.push_back(w.At(k));
  }
  StationaryImuAccumulator acc;
  for (int k = 0; k < kInitTicks; ++k) {
    (void)acc.Add(samples[static_cast<std::size_t>(k)].gyro,
                  samples[static_cast<std::size_t>(k)].accel);
  }
  const std::array<Vec3, 2> pts{samples[kInitTicks - 1].kin[0].position,
                                samples[kInitTicks - 1].kin[1].position};
  auto start = [&](ContactInekfSequencer& s) {
    ASSERT_EQ(s.Initialize(acc.MeanGyro(), acc.MeanAccel(), kInitTicks * kPeriod, 0.0, pts),
              InekfStatus::kOk);
  };
  ContactInekfSequencer a;
  ContactInekfSequencer b;
  a.Init(CoreFixture(), FsmFixture(), 2);
  b.Init(CoreFixture(), FsmFixture(), 2);
  start(a);
  start(b);
  std::vector<std::vector<double>> trace;
  trace.reserve(kTicks);
  auto step = [&](ContactInekfSequencer& s, const Sample& x) {
    ContactStepInput in;
    in.gyro = x.gyro;
    in.accel = x.accel;
    in.kinematics = std::span<const ContactKinematics>(x.kin.data(), 2);
    in.joint_velocity = std::span<const Vec3>(x.rdot.data(), 2);
    in.planned_mask = x.planned;
    return s.Step(in);
  };
  auto same_reports = [](const ContactInekfSequencer& x, const ContactInekfSequencer& y) {
    for (int i = 0; i < 2; ++i) {
      const ContactUpdateReport& p = x.report(i);
      const ContactUpdateReport& q = y.report(i);
      if (p.result != q.result || std::memcmp(&p.mahalanobis2, &q.mahalanobis2, sizeof(double)) ||
          std::memcmp(p.innovation.data(), q.innovation.data(), 3 * sizeof(double))) {
        return false;
      }
    }
    return true;
  };
  int transitions = 0;
  for (int k = kInitTicks; k < kTicks + kInitTicks; ++k) {
    const Sample& x = samples[static_cast<std::size_t>(k)];
    const ContactMode before = a.fsm().mode(0);
    ASSERT_EQ(step(a, x), ContactFsmStatus::kOk);
    ASSERT_EQ(step(b, x), ContactFsmStatus::kOk);
    transitions += a.fsm().mode(0) != before;
    std::vector<double> sa = Access::SequencerState(a);
    ASSERT_TRUE(SameBits(sa, Access::SequencerState(b))) << k;
    ASSERT_TRUE(BitEqual(Capture(a.core()), Capture(b.core()))) << k;
    ASSERT_TRUE(same_reports(a, b)) << k;
    const Snapshot c = Capture(a.core());
    sa.insert(sa.end(), c.values.begin(), c.values.end());
    trace.push_back(std::move(sa));
  }
  EXPECT_GT(transitions, 50);  // the run walks the table, not one mode
  a.Reset();
  start(a);
  for (int k = kInitTicks; k < kTicks + kInitTicks; ++k) {
    ASSERT_EQ(step(a, samples[static_cast<std::size_t>(k)]), ContactFsmStatus::kOk);
    std::vector<double> sa = Access::SequencerState(a);
    const Snapshot c = Capture(a.core());
    sa.insert(sa.end(), c.values.begin(), c.values.end());
    ASSERT_TRUE(SameBits(sa, trace[static_cast<std::size_t>(k - kInitTicks)])) << k;
  }
}

}  // namespace
}  // namespace rtc::estimation::testing
