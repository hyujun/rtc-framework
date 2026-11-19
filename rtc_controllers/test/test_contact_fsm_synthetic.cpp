// ── Contact FSM + sequencer on the synthetic world (test-only) ───────────────
//
// Every run steps the real core through the sequencer and counts, per tick,
// the core-contract checks of ContractChecker (insert only on candidate
// entry, promotion after a used report, slip drops the anchor, scale by mode,
// anchor ⇔ active slot) and every Step() that did not return kOk: both must
// be 0 in every run. The thresholds are pre-registered and do not move with
// the measurement; seeds are kSeed + 0 … 7.
//
//   static        60 s double support: no slip entry, ≤ 2 unknown entries
//                 over all seeds, no other transition after acquisition
//   gait          60 s alternating: precision / recall ≥ 0.999, touchdown
//                 delay p50 ≤ 24 · p99 ≤ 60 · max ≤ 100 · min ≥ 18 ms,
//                 liftoff ≤ 4 ms, IMU velocity error ≤ 0.06 m/s
//   shifted plan  planned ±40 ms from truth, four ways (see each test)
//   slip          0.3 m/s for 100 ms, 40 events: ≥ 38 detected within 30 ms,
//                 the other point leaves stance in ≤ 2, back in ≤ 60 ms
//   FK step       +2 cm for 100 ms, 40 events: unknown within 12 ms, no slip,
//                 ≤ 3 unknown entries per event, stance ≤ 60 ms after
//   all off       every plan bit off 0.3 s, 40 events: all re-acquired ≤ 40 ms
//   four points   tilted 0.2 rad plane, one point swinging at a time: the
//                 gait thresholds
// Report only (printed): a 0.1 m/s slip, a 30 m/s² accel step, 1.0 s all-off,
// and one point sliding at 0.07 – 0.13 m/s for 2 s (inside the hysteresis
// band, but within measurement noise of v_slip at the top: a declared limit —
// the band itself is guarded by the transition oracle).
#include "contact_fsm_testing.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <string>

namespace rtc::estimation::testing {
namespace {

constexpr int kSeeds = 8;
constexpr double kEps = 1e-12;  // tick-count delays compared as doubles

std::vector<RunLog> RunAll(const Scenario& sc) {
  std::vector<RunLog> logs;
  for (int s = 0; s < kSeeds; ++s) {
    logs.push_back(Run(sc, kSeed + static_cast<std::uint64_t>(s)));
  }
  return logs;
}

void ExpectClean(const std::vector<RunLog>& logs) {
  for (std::size_t s = 0; s < logs.size(); ++s) {
    EXPECT_EQ(logs[s].bad_status, 0) << "seed " << s;
    EXPECT_EQ(logs[s].contract, 0) << "seed " << s;
    EXPECT_GT(logs[s].insertions, 0) << "seed " << s;
  }
}

int Tick(double t) {
  return static_cast<int>(std::lround(t / kPeriod));
}

void Print(const char* name, const Detection& d) {
  std::printf(
      "[metric] %-14s precision %.6f recall %.6f | touchdown n=%zu min %.0f p50 %.0f p99 %.0f "
      "max %.0f ms | liftoff n=%zu max %.0f ms | slip in %d unknown in %d\n",
      name, d.precision(), d.recall(), d.touchdown.size(), 1e3 * Quantile(d.touchdown, 0.0),
      1e3 * Quantile(d.touchdown, 0.5), 1e3 * Quantile(d.touchdown, 0.99),
      1e3 * Quantile(d.touchdown, 1.0), d.liftoff.size(), 1e3 * Quantile(d.liftoff, 1.0),
      d.slip_entries, d.unknown_entries);
}

void ExpectGaitThresholds(const Detection& d) {
  EXPECT_GE(d.precision(), 0.999);
  EXPECT_GE(d.recall(), 0.999);
  ASSERT_FALSE(d.touchdown.empty());
  ASSERT_FALSE(d.liftoff.empty());
  EXPECT_LE(Quantile(d.touchdown, 0.5), 0.024 + kEps);
  EXPECT_LE(Quantile(d.touchdown, 0.99), 0.060 + kEps);
  EXPECT_LE(Quantile(d.touchdown, 1.0), 0.100 + kEps);
  // (n_enter − 1 + max(n_confirm, 2))·T: a shorter dwell or an early promotion fails.
  EXPECT_GE(Quantile(d.touchdown, 0.0), (kNEnter - 1 + std::max(kNConfirm, 2)) * kPeriod - kEps);
  EXPECT_LE(Quantile(d.liftoff, 1.0), kNRelease * kPeriod + kEps);
}

TEST(ContactFsmSynthetic, StaticDoubleSupport) {
  const std::vector<RunLog> logs = RunAll({World::kStatic, Shift::kNone, 60.0});
  ExpectClean(logs);
  int slip = 0;
  int unknown = 0;
  int other = 0;
  for (const RunLog& log : logs) {
    for (int i = 0; i < log.slots; ++i) {
      int first = -1;
      for (int k = log.first; k < static_cast<int>(log.tick.size()); ++k) {
        if (InStance(log, k, i)) {
          first = k;
          break;
        }
      }
      ASSERT_GE(first, 0) << "slot " << i << " never reached stance";
      for (int k = first + 1; k < static_cast<int>(log.tick.size()); ++k) {
        const ContactMode a = ModeAt(log, k - 1, i);
        const ContactMode b = ModeAt(log, k, i);
        if (a == b) {
          continue;
        }
        slip += b == ContactMode::kSlip;
        unknown += b == ContactMode::kUnknown;
        const bool stance_unknown = (a == ContactMode::kStance && b == ContactMode::kUnknown) ||
                                    (a == ContactMode::kUnknown && b == ContactMode::kStance);
        other += stance_unknown ? 0 : 1;
      }
    }
  }
  std::printf("[metric] static: slip in %d unknown in %d other %d\n", slip, unknown, other);
  EXPECT_EQ(slip, 0);
  EXPECT_LE(unknown, 2);
  EXPECT_EQ(other, 0);
}

TEST(ContactFsmSynthetic, AlternatingGait) {
  const std::vector<RunLog> logs = RunAll({World::kGait, Shift::kNone, 60.0});
  ExpectClean(logs);
  Detection all;
  double verr = 0.0;
  for (const RunLog& log : logs) {
    all.Merge(Detect(log, 0.0));
    verr = std::max(verr, log.vhat_err_after_1s);
  }
  Print("gait", all);
  std::printf("[metric] gait: IMU velocity error max %.4f m/s\n", verr);
  ExpectGaitThresholds(all);
  EXPECT_LE(verr, 0.06);
}

TEST(ContactFsmSynthetic, HysteresisBandReportOnly) {
  const std::vector<RunLog> logs = RunAll({World::kBand, Shift::kNone, 6.0});
  ExpectClean(logs);
  int slip = 0;
  int unknown = 0;
  for (const RunLog& log : logs) {
    ASSERT_TRUE(InStance(log, Tick(kBandStart) - 1, 0)) << "band must start in stance";
    for (int k = Tick(kBandStart); k <= Tick(kBandEnd + 0.1); ++k) {
      const ContactMode m = ModeAt(log, k, 0);
      if (m != ModeAt(log, k - 1, 0)) {
        slip += m == ContactMode::kSlip;
        unknown += m == ContactMode::kUnknown;
      }
    }
  }
  std::printf("[metric] band (report): slip in %d unknown in %d\n", slip, unknown);
}

TEST(ContactFsmSynthetic, PlannedOnEarly) {
  const std::vector<RunLog> logs = RunAll({World::kGait, Shift::kPlannedOnEarly, 30.0});
  ExpectClean(logs);
  long bad = 0;
  for (const RunLog& log : logs) {
    for (int k = log.first; k < static_cast<int>(log.tick.size()); ++k) {
      const TickLog& t = log.tick[static_cast<std::size_t>(k)];
      for (int i = 0; i < log.slots; ++i) {
        bad += !Bit(t.gt, i) && Bit(t.planned, i) && InStance(log, k, i);
      }
    }
  }
  std::printf("[metric] planned on early: stance while GT off %ld\n", bad);
  EXPECT_EQ(bad, 0);
}

TEST(ContactFsmSynthetic, PlannedOnLate) {
  const std::vector<RunLog> logs = RunAll({World::kGait, Shift::kPlannedOnLate, 30.0});
  ExpectClean(logs);
  long early = 0;
  Detection all;
  for (const RunLog& log : logs) {
    for (int k = log.first; k < static_cast<int>(log.tick.size()); ++k) {
      const TickLog& t = log.tick[static_cast<std::size_t>(k)];
      for (int i = 0; i < log.slots; ++i) {
        early += Bit(t.gt, i) && !Bit(t.planned, i) && InStance(log, k, i);
      }
    }
    all.Merge(Detect(log, kShiftD));
  }
  Print("on late", all);
  std::printf("[metric] planned on late: stance before planned %ld\n", early);
  EXPECT_EQ(early, 0);
  ASSERT_FALSE(all.touchdown.empty());
  EXPECT_LE(Quantile(all.touchdown, 1.0), 0.080 + kEps);
}

TEST(ContactFsmSynthetic, PlannedOffEarly) {
  const std::vector<RunLog> logs = RunAll({World::kGait, Shift::kPlannedOffEarly, 30.0});
  ExpectClean(logs);
  Detection all;
  for (const RunLog& log : logs) {
    all.Merge(Detect(log, kShiftD));
  }
  Print("off early", all);
  ASSERT_FALSE(all.liftoff.empty());
  EXPECT_LE(Quantile(all.liftoff, 1.0), kNRelease * kPeriod + kEps);
}

TEST(ContactFsmSynthetic, PlannedOffLate) {
  Scenario sc{World::kGait, Shift::kPlannedOffLate, 30.0};
  sc.foot_speed = true;
  const std::vector<RunLog> logs = RunAll(sc);
  ExpectClean(logs);
  const double v_slip = FsmFixture().slip_speed;
  std::vector<double> delay;
  for (const RunLog& log : logs) {
    const int n = static_cast<int>(log.tick.size());
    for (int i = 0; i < log.slots; ++i) {
      for (int e = Tick(kEvalStart); e < n; ++e) {
        const bool off_edge = !Bit(log.tick[static_cast<std::size_t>(e)].gt, i) &&
                              Bit(log.tick[static_cast<std::size_t>(e - 1)].gt, i);
        if (!off_edge) {
          continue;
        }
        int ref = e;
        while (ref < n &&
               log.foot_speed[static_cast<std::size_t>(ref)][static_cast<std::size_t>(i)] <=
                   v_slip) {
          ++ref;
        }
        int exit = e;
        while (exit < n && InStance(log, exit, i)) {
          ++exit;
        }
        if (ref < n && exit < n) {
          delay.push_back((exit - ref) * kPeriod);
        }
      }
    }
  }
  std::printf("[metric] planned off late: exit after the true speed crossing n=%zu max %.0f ms\n",
              delay.size(), 1e3 * Quantile(delay, 1.0));
  ASSERT_FALSE(delay.empty());
  EXPECT_LE(Quantile(delay, 1.0), 0.012 + kEps);
}

struct EventStats {
  int events{0};
  int detected{0};
  int other_left{0};
  std::vector<double> delay;
  std::vector<double> recover;
  std::vector<int> entries;  // unknown entries per event
  int slip_entries{0};
};

/// Last non-stance tick of slot i in [from, to) + 1 − from [s]; 0 when none.
double SettledAfter(const RunLog& log, int i, int from, int to) {
  int last = from - 1;
  for (int k = from; k < to; ++k) {
    if (!InStance(log, k, i)) {
      last = k;
    }
  }
  return (last + 1 - from) * kPeriod;
}

EventStats Events(const std::vector<RunLog>& logs, int duration_ticks) {
  EventStats st;
  for (const RunLog& log : logs) {
    for (int e = 0; e < NumEvents(log.sc); ++e) {
      const int k0 = Tick(kEventStart + kEventEvery * e);
      const int k1 = k0 + Tick(kEventWindow);
      const int end = k0 + duration_ticks;
      ++st.events;
      EXPECT_TRUE(InStance(log, k0 - 1, 0) && InStance(log, k0 - 1, 1))
          << "event must start in double stance";
      int det = -1;
      for (int k = k0; k < end + 100; ++k) {
        if (!InStance(log, k, 0)) {
          det = k;
          break;
        }
      }
      if (det >= 0) {
        ++st.detected;
        st.delay.push_back((det - k0) * kPeriod);
        st.recover.push_back(SettledAfter(log, 0, end, k1));
      }
      bool other = false;
      int unknown = 0;
      for (int k = k0; k < k1; ++k) {
        other = other || !InStance(log, k, 1);
        for (int i = 0; i < log.slots; ++i) {
          const ContactMode m = ModeAt(log, k, i);
          if (m != ModeAt(log, k - 1, i)) {
            unknown += i == 0 && m == ContactMode::kUnknown;
            st.slip_entries += m == ContactMode::kSlip;
          }
        }
      }
      st.other_left += other ? 1 : 0;
      st.entries.push_back(unknown);
    }
  }
  return st;
}

void Print(const char* name, const EventStats& st) {
  int max_entries = 0;
  double mean_entries = 0.0;
  for (int x : st.entries) {
    max_entries = std::max(max_entries, x);
    mean_entries += x;
  }
  mean_entries /= static_cast<double>(std::max<std::size_t>(1, st.entries.size()));
  std::printf(
      "[metric] %-12s events %d detected %d delay max %.0f ms | settled after max %.0f ms | "
      "other left %d | slip in %d | unknown entries per event max %d mean %.2f\n",
      name, st.events, st.detected, 1e3 * Quantile(st.delay, 1.0), 1e3 * Quantile(st.recover, 1.0),
      st.other_left, st.slip_entries, max_entries, mean_entries);
}

TEST(ContactFsmSynthetic, SlipDetected) {
  const std::vector<RunLog> logs = RunAll({World::kSlip, Shift::kNone, 10.0, 0.3});
  ExpectClean(logs);
  const EventStats st = Events(logs, Tick(0.12));
  Print("slip 0.3", st);
  EXPECT_EQ(st.events, 40);
  EXPECT_GE(st.detected, 38);
  ASSERT_FALSE(st.delay.empty());
  EXPECT_LE(Quantile(st.delay, 1.0), 0.030 + kEps);
  EXPECT_LE(st.other_left, 2);
  EXPECT_LE(Quantile(st.recover, 1.0), 0.060 + kEps);
}

TEST(ContactFsmSynthetic, KinematicStepGoesUnknown) {
  const std::vector<RunLog> logs = RunAll({World::kFkStep, Shift::kNone, 10.0});
  ExpectClean(logs);
  std::vector<double> to_unknown;
  for (const RunLog& log : logs) {
    for (int e = 0; e < NumEvents(log.sc); ++e) {
      const int k0 = Tick(kEventStart + kEventEvery * e);
      int k = k0;
      while (k < k0 + Tick(0.1) && ModeAt(log, k, 0) != ContactMode::kUnknown) {
        ++k;
      }
      to_unknown.push_back((k - k0) * kPeriod);
    }
  }
  const EventStats st = Events(logs, Tick(0.1));
  Print("FK step", st);
  std::printf("[metric] FK step: to unknown max %.0f ms\n", 1e3 * Quantile(to_unknown, 1.0));
  EXPECT_EQ(st.events, 40);
  EXPECT_LE(Quantile(to_unknown, 1.0), 0.012 + kEps);
  EXPECT_EQ(st.slip_entries, 0);
  for (int x : st.entries) {
    EXPECT_LE(x, 3);
  }
  ASSERT_EQ(st.recover.size(), 40U);
  EXPECT_LE(Quantile(st.recover, 1.0), 0.060 + kEps);
}

std::vector<double> Reacquire(const std::vector<RunLog>& logs, double off, int& complete) {
  std::vector<double> delay;
  complete = 0;
  for (const RunLog& log : logs) {
    for (int e = 0; e < NumEvents(log.sc); ++e) {
      const int k0 = Tick(kEventStart + kEventEvery * e);
      const int on = k0 + Tick(off);
      const int k1 = k0 + Tick(kEventWindow);
      bool all = true;
      for (int i = 0; i < log.slots; ++i) {
        int k = on;
        while (k < k1 && !InStance(log, k, i)) {
          ++k;
        }
        if (k < k1) {
          delay.push_back((k - on) * kPeriod);
        } else {
          all = false;
        }
      }
      complete += all ? 1 : 0;
    }
  }
  return delay;
}

TEST(ContactFsmSynthetic, AllPlannedOffReacquires) {
  const std::vector<RunLog> logs = RunAll({World::kAllOff, Shift::kNone, 10.0, 0.3, 0.3});
  ExpectClean(logs);
  int complete = 0;
  const std::vector<double> delay = Reacquire(logs, 0.3, complete);
  std::printf("[metric] all off 0.3 s: re-acquired %d / 40, max %.0f ms\n", complete,
              1e3 * Quantile(delay, 1.0));
  EXPECT_EQ(complete, 40);
  EXPECT_LE(Quantile(delay, 1.0), 0.040 + kEps);
}

TEST(ContactFsmSynthetic, FourPointsOnATiltedPlane) {
  Scenario sc{World::kQuad, Shift::kNone, 20.0};
  sc.tilt = 0.2;
  const std::vector<RunLog> logs = RunAll(sc);
  ExpectClean(logs);
  Detection all;
  for (const RunLog& log : logs) {
    all.Merge(Detect(log, 0.0));
  }
  Print("four points", all);
  ExpectGaitThresholds(all);
}

TEST(ContactFsmSynthetic, ReportOnly) {
  {
    const std::vector<RunLog> logs = RunAll({World::kSlip, Shift::kNone, 10.0, 0.1});
    ExpectClean(logs);
    Print("slip 0.1", Events(logs, Tick(0.12)));
  }
  {
    const std::vector<RunLog> logs = RunAll({World::kAccelSpike, Shift::kNone, 10.0, 30.0});
    ExpectClean(logs);
    Print("accel 30", Events(logs, 2));
  }
  {
    const std::vector<RunLog> logs = RunAll({World::kAllOff, Shift::kNone, 10.0, 0.3, 1.0});
    ExpectClean(logs);
    int complete = 0;
    const std::vector<double> delay = Reacquire(logs, 1.0, complete);
    int no_anchor = 0;
    for (const RunLog& log : logs) {
      no_anchor = std::max(no_anchor, log.max_no_anchor_ticks);
    }
    std::printf(
        "[metric] all off 1.0 s: re-acquired %d / 40, max %.0f ms, longest no-anchor %d ticks\n",
        complete, 1e3 * Quantile(delay, 1.0), no_anchor);
  }
}

}  // namespace
}  // namespace rtc::estimation::testing
