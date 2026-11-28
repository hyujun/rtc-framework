// ── Contact FSM + sequencer: zero heap allocation on the RT path ─────────────
//
// Both gates armed around 1000 sequencer ticks that walk every transition and
// every refusal row (Propagate / Insert / Update refused, a desynchronised
// core, Reset + Initialize, k = 2), plus 1000 standalone FSM PreUpdate /
// PostUpdate ticks:
//   ScopedAllocGate  — global operator new (any TU, the library's included);
//   ScopedMallocGate — the C allocator for the whole executable.
// The walk itself is asserted (every reason and status seen), so a zero is
// over the paths it claims. Each gate's positive control is the FSM's own
// non-RT path: Init()'s refusal builds its message inside the library.
#include "contact_fsm_testing.hpp"
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace rtc::estimation::testing {
namespace {

ContactFsm::Config BadConfig() {
  ContactFsm::Config c = FsmFixture();
  c.slip_speed = c.enter_speed;
  return c;
}

TEST(ContactFsmAlloc, PositiveControlsFireInsideTheLibrary) {
  {
    ContactFsm f;
    rtc::testing::ScopedAllocGate gate;
    EXPECT_THROW(f.Init(BadConfig(), 2), std::invalid_argument);
    EXPECT_GT(gate.count(), 0U);
  }
  {
    ContactFsm f;
    rtc::testing::ScopedMallocGate gate;
    EXPECT_THROW(f.Init(BadConfig(), 2), std::invalid_argument);
    EXPECT_GT(gate.count(), 0U);
  }
}

TEST(ContactFsmAlloc, EveryPathAllocatesNothing) {
  constexpr int kTicks = 1000;
  const Scenario sc{World::kStatic, Shift::kNone, 10.0};
  SyntheticWorld world(sc, kSeed);
  std::vector<Sample> samples;
  samples.reserve(kInitTicks + kTicks);
  for (int k = 1; k <= kInitTicks + kTicks; ++k) {
    samples.push_back(world.At(k));
  }
  StationaryImuAccumulator acc;
  for (int k = 0; k < kInitTicks; ++k) {
    (void)acc.Add(samples[static_cast<std::size_t>(k)].gyro,
                  samples[static_cast<std::size_t>(k)].accel);
  }
  const std::array<Vec3, 2> pts{samples[kInitTicks - 1].kin[0].position,
                                samples[kInitTicks - 1].kin[1].position};
  ContactInekfSequencer seq;
  // With the support-height pin on, every settled anchor also runs it (applied,
  // gated or — max_gated 1 — inflated).
  ContactInekfSequencer::GroundConfig ground;
  ground.enabled = true;
  ground.sigma = 1e-3;
  ground.gate_chi2 = 10.83;
  ground.max_gated = 1;
  ground.settle_s = 0.02;
  seq.Init(CoreFixture(), FsmFixture(), 2, ground);
  ContactFsm fsm;
  fsm.Init(FsmFixture(), 2);
  std::array<ContactFsm::SlotAction, 2> actions{};
  std::array<ContactUpdateReport, 2> fake{};
  const double nan = std::numeric_limits<double>::quiet_NaN();

  std::array<bool, 10> reasons{};
  std::array<bool, 8> statuses{};
  std::array<bool, 4> pins{};
  int inits = 0;
  std::size_t news = 0;
  std::size_t mallocs = 0;
  {
    rtc::testing::ScopedAllocGate new_gate;
    rtc::testing::ScopedMallocGate malloc_gate;
    auto start = [&] {
      inits += seq.Initialize(acc.MeanGyro(), acc.MeanAccel(), kInitTicks * kPeriod, 0.0, pts) ==
                       InekfStatus::kOk
                   ? 1
                   : 0;
    };
    start();
    for (int t = 0; t < kTicks; ++t) {
      Sample& x = samples[static_cast<std::size_t>(kInitTicks + t)];
      int periods = 1;
      if (t >= 50 && t < 80) {
        x.kin[0].position.y() += 0.02;  // → unknown, then recovered
      }
      if (t >= 200 && t < 210) {
        x.rdot[0].x() += 0.5;  // → slip, then touchdown + confirmed
      }
      if (t >= 300 && t < 303) {
        x.planned &= ~1U;  // → released
      }
      if (t >= 305 && t < 330 && seq.fsm().mode(0) == ContactMode::kCandidate) {
        x.rdot[0].x() += 0.5;  // → rejected by speed
      }
      if (t >= 330 && t < 400 && seq.fsm().mode(0) == ContactMode::kCandidate) {
        x.kin[0].position.x() += 0.3;  // gated → rejected by NIS
      }
      if (t == 400) {
        periods = 11;  // Propagate refused
      }
      if (t == 401) {
        x.gyro.x() = nan;  // Propagate refused
      }
      if (t == 402) {
        periods = 2;
      }
      if (t == 410) {
        x.kin[0].jacobian.setConstant(1e160);  // Update refused
      }
      if (t == 500) {
        ContactInekfTestAccess::SetActive(ContactFsmTestAccess::Core(seq), 0, false);
      }
      if (t == 503 || t == 620) {
        seq.Reset();
        start();
      }
      if (t == 600) {
        seq.Reset();
        start();
        ContactInekfTestAccess::SetActive(ContactFsmTestAccess::Core(seq), 0, true);
      }
      ContactStepInput in;
      in.gyro = x.gyro;
      in.accel = x.accel;
      in.kinematics = std::span<const ContactKinematics>(x.kin.data(), 2);
      in.joint_velocity = std::span<const Vec3>(x.rdot.data(), 2);
      in.planned_mask = x.planned;
      in.periods = periods;
      const ContactFsmStatus st = seq.Step(in);
      statuses[static_cast<std::size_t>(st)] = true;
      for (int i = 0; i < 2; ++i) {
        reasons[static_cast<std::size_t>(seq.fsm().reason(i))] = true;
        pins[static_cast<std::size_t>(seq.ground_report(i).result)] = true;
      }

      // The FSM alone, with alternating inputs and reports.
      const std::array<Vec3, 2> u{Vec3(t % 50 < 40 ? 0.0 : 0.3, 0.0, 0.0),
                                  Vec3(t % 97 == 0 ? nan : 0.0, 0.0, 0.0)};
      (void)fsm.PreUpdate(1 + t % 2, t % 60 < 55 ? 3U : 1U, u, actions);
      fake[0].result = t % 9 == 0 ? ContactUpdateResult::kGated : ContactUpdateResult::kUsed;
      fake[0].mahalanobis2 = t % 30 < 25 ? 1.0 : 20.0;
      fake[1].result = ContactUpdateResult::kUsed;
      fsm.PostUpdate(fake, t % 13 != 0);
      if (actions[0].insert && t % 3 == 0) {
        fsm.RevertInsert(0);
      }
    }
    fsm.Reset();
    news = new_gate.count();
    mallocs = malloc_gate.count();
  }
  EXPECT_EQ(inits, 4);
  for (const ContactReason r :
       {ContactReason::kTouchdown, ContactReason::kConfirmed, ContactReason::kReleased,
        ContactReason::kRejectedSpeed, ContactReason::kRejectedNis, ContactReason::kSlip,
        ContactReason::kNisHigh, ContactReason::kRecovered, ContactReason::kInsertRefused}) {
    EXPECT_TRUE(reasons[static_cast<std::size_t>(r)]) << "reason " << static_cast<int>(r);
  }
  for (const ContactFsmStatus s :
       {ContactFsmStatus::kOk, ContactFsmStatus::kPropagateRefused,
        ContactFsmStatus::kUpdateRefused, ContactFsmStatus::kStateMismatch}) {
    EXPECT_TRUE(statuses[static_cast<std::size_t>(s)]) << "status " << static_cast<int>(s);
  }
  EXPECT_TRUE(pins[static_cast<std::size_t>(GroundUpdateResult::kUsed)] ||
              pins[static_cast<std::size_t>(GroundUpdateResult::kInflated)]);
  EXPECT_EQ(news, 0U);
  EXPECT_EQ(mallocs, 0U);
}

}  // namespace
}  // namespace rtc::estimation::testing
