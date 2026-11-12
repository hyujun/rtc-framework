// GaitScheduler::PhaseInfo · GaitPhase · SwingStart (reference/): the W2-F02
// [SPRINT] PR B gates on the scheduler's two new readers. The phase oracle is a
// tile loop from the anchor (the tile index k with a + kT ≤ t < a + (k+1)T), not
// a modulo; the swing-start oracle is the phase-start list of the running gait.
#include "rtc_mpc/reference/gait_scheduler.hpp"
#include "rtc_mpc/reference/gait_table.hpp"
#include "test_utils/loco_reference_fixture.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>

namespace {

using rtc::mpc::FilteredCommand;
using rtc::mpc::GaitPhase;
using rtc::mpc::GaitPhaseInfo;
using rtc::mpc::GaitScheduler;
using rtc::mpc::GaitSchedulerError;
using rtc::mpc::GaitTable;
using rtc::mpc::GaitTemplate;
using rtc::mpc::PlanarVelocity;
using rtc::mpc::test_utils::LocoGaitTable;
using rtc::mpc::test_utils::RegisteredCommand;

constexpr double kInf = std::numeric_limits<double>::infinity();

// φ by the tile index: the k with a + kT ≤ t < a + (k+1)T, found by walking.
double TilePhase(double anchor, double period, double t) {
  long k = 0;
  if (t >= anchor) {
    while (anchor + static_cast<double>(k + 1) * period <= t) {
      ++k;
    }
  } else {
    while (anchor + static_cast<double>(k) * period > t) {
      --k;
    }
  }
  return (t - (anchor + static_cast<double>(k) * period)) / period;
}

double Circular(double a, double b) {
  const double d = std::fabs(a - b);
  return std::min(d, 1.0 - d);
}

void Walking(double t_anchor, GaitTable& table, GaitScheduler& s) {
  table = LocoGaitTable();
  EXPECT_EQ(s.Init(table, 1.0), GaitSchedulerError::kNoError);
  rtc::mpc::test_utils::WalkFrom(s, t_anchor);  // the walk runs since t_anchor − 1.27
}

TEST(GaitPhase, FalseBeforeTheFirstAdvanceAndATileOracleAfter) {
  GaitTable table = LocoGaitTable();
  GaitScheduler s;
  ASSERT_EQ(s.Init(table, 1.0), GaitSchedulerError::kNoError);
  GaitPhaseInfo info{};
  EXPECT_FALSE(s.PhaseInfo(info));
  ASSERT_EQ(s.Advance(5.0, RegisteredCommand(), PlanarVelocity{}), GaitSchedulerError::kNoError);
  ASSERT_TRUE(s.PhaseInfo(info));
  EXPECT_EQ(info.anchor, 5.0);
  EXPECT_EQ(info.period, 0.5);
  EXPECT_TRUE(info.single_mode);  // stance
  EXPECT_FALSE(info.pending);
  for (int i = 0; i <= 3000; ++i) {
    const double t = 5.0 + 3.0 * i / 3000.0;
    ASSERT_LE(Circular(GaitPhase(info, t), TilePhase(5.0, 0.5, t)), 1e-12) << t;
  }
  // Strictly inside [0, 1) at every tile edge and its neighbours.
  for (int k = 0; k <= 1000; ++k) {
    const double edge = 5.0 + k * 0.5;
    for (const double t : {edge, std::nextafter(edge, -kInf), std::nextafter(edge, kInf)}) {
      const double phi = GaitPhase(info, t);
      ASSERT_GE(phi, 0.0) << t;
      ASSERT_LT(phi, 1.0) << t;
    }
  }
  // A pending stance → walk: the walk enters at its first full-support phase
  // (switching_time 0.13), so φ(t_s) = 0.13 / 0.9 — the target's anchor is
  // t_s − 0.13, not t_s.
  s.RequestGait(1);
  ASSERT_EQ(s.Advance(5.1, RegisteredCommand(), PlanarVelocity{}), GaitSchedulerError::kNoError);
  ASSERT_TRUE(s.switch_pending());
  ASSERT_TRUE(s.PhaseInfo(info));
  ASSERT_TRUE(info.pending);
  EXPECT_NEAR(info.next_anchor, info.t_switch - 0.13, 1e-12);
  EXPECT_LE(Circular(GaitPhase(info, info.t_switch), 0.13 / 0.9), 1e-12);
  for (int i = 0; i <= 2000; ++i) {
    const double t = 5.1 + 2.0 * i / 2000.0;
    const double expect =
        t >= info.t_switch ? TilePhase(info.next_anchor, 0.9, t) : TilePhase(5.0, 0.5, t);
    ASSERT_LE(Circular(GaitPhase(info, t), expect), 1e-12) << t;
  }
}

TEST(GaitPhase, APendingSwitchPhasesTheTargetGaitFromItsSwitchTime) {
  GaitTable table;
  GaitScheduler s;
  Walking(10.0, table, s);
  s.RequestGait(2);  // hop: no full support → its first phase starts at t_s
  ASSERT_EQ(s.Advance(10.01, RegisteredCommand(), PlanarVelocity{}), GaitSchedulerError::kNoError);
  ASSERT_TRUE(s.switch_pending());
  GaitPhaseInfo info{};
  ASSERT_TRUE(s.PhaseInfo(info));
  EXPECT_TRUE(info.pending);
  EXPECT_EQ(info.t_switch, s.switch_time());
  EXPECT_EQ(info.next_period, 0.5);
  EXPECT_FALSE(info.single_mode);
  EXPECT_FALSE(info.next_single_mode);
  // φ(t_s) = switching_time[j_entry] / T of the target.
  const GaitTemplate& hop = table.gaits[2];
  const double entry = (s.switch_time() - info.next_anchor) / hop.period;
  bool is_a_phase_start = false;
  for (std::size_t j = 0; j < hop.n_phases; ++j) {
    is_a_phase_start =
        is_a_phase_start || std::fabs(entry - hop.switching_time[j] / hop.period) < 1e-12;
  }
  EXPECT_TRUE(is_a_phase_start);
  EXPECT_LE(Circular(GaitPhase(info, s.switch_time()), entry), 1e-12);
  for (int i = 0; i <= 2000; ++i) {
    const double t = 10.0 + 2.0 * i / 2000.0;
    const double expect =
        t >= info.t_switch ? TilePhase(info.next_anchor, 0.5, t) : TilePhase(info.anchor, 0.9, t);
    ASSERT_LE(Circular(GaitPhase(info, t), expect), 1e-12) << t;
  }
}

TEST(SwingStart, ReportsTheLiftOffOfASwingUnderWay) {
  GaitTable table;
  GaitScheduler s;
  Walking(10.0, table, s);
  GaitPhaseInfo info{};
  ASSERT_TRUE(s.PhaseInfo(info));
  const double a = info.anchor;  // the walk's tile 0
  // walk: b (bit 1) swings [0, 0.13), a (bit 0) swings [0.37, 0.52) of each tile.
  double t_lo = 0.0;
  bool clamped = true;
  // The first swing after the switch: the walk entered at phase 1 (full
  // support), so the a-swing of tile 0 starts at a + 0.37 ≥ since.
  ASSERT_TRUE(s.SwingStart(0, a + 0.40, t_lo, clamped));
  EXPECT_NEAR(t_lo, a + 0.37, 1e-12);
  EXPECT_FALSE(clamped);
  ASSERT_TRUE(s.SwingStart(1, a + 0.95, t_lo, clamped));
  EXPECT_NEAR(t_lo, a + 0.90, 1e-12);
  EXPECT_FALSE(clamped);
  ASSERT_TRUE(s.SwingStart(0, a + 0.90 + 0.51, t_lo, clamped));
  EXPECT_NEAR(t_lo, a + 0.90 + 0.37, 1e-12);
  // In stance: false, outputs untouched.
  t_lo = -1.0;
  EXPECT_FALSE(s.SwingStart(0, a + 0.2, t_lo, clamped));
  EXPECT_FALSE(s.SwingStart(1, a + 0.2, t_lo, clamped));
  EXPECT_FALSE(s.SwingStart(2, a + 0.4, t_lo, clamped));  // out of range
  EXPECT_FALSE(s.SwingStart(0, std::numeric_limits<double>::quiet_NaN(), t_lo, clamped));
  EXPECT_EQ(t_lo, -1.0);
  // The b-swing the walk's tiling puts before it took over is fiction: at
  // the first instant after the switch b is in stance (phase 1).
  const double t_s = s.switch_time();
  EXPECT_FALSE(s.SwingStart(1, t_s, t_lo, clamped));
  EXPECT_FALSE(s.SwingStart(1, std::nextafter(t_s, kInf), t_lo, clamped));
}

TEST(SwingStart, RightAfterASwitchAndAfterAResetItClampsAtTheTakeOver) {
  // A switch into hop (no full support): the gait starts with a's single
  // support, so b is in swing from t_s on, clamped there.
  GaitTable table;
  GaitScheduler s;
  Walking(20.0, table, s);
  s.RequestGait(2);
  ASSERT_EQ(s.Advance(20.01, RegisteredCommand(), PlanarVelocity{}), GaitSchedulerError::kNoError);
  const double t_s = s.switch_time();
  ASSERT_EQ(s.Advance(t_s + 0.1, RegisteredCommand(), PlanarVelocity{}),
            GaitSchedulerError::kNoError);
  ASSERT_EQ(s.current_gait(), 2U);
  double t_lo = 0.0;
  bool clamped = false;
  ASSERT_TRUE(s.SwingStart(1, t_s + 0.1, t_lo, clamped));
  EXPECT_NEAR(t_lo, t_s, 1e-12);
  EXPECT_TRUE(clamped);
  // After the flight phase at +0.15 the b-swing is still the one from t_s
  // (no on → off edge); a lifts at the flight start.
  ASSERT_TRUE(s.SwingStart(1, t_s + 0.2, t_lo, clamped));
  EXPECT_NEAR(t_lo, t_s, 1e-12);
  EXPECT_TRUE(clamped);
  ASSERT_TRUE(s.SwingStart(0, t_s + 0.2, t_lo, clamped));
  EXPECT_NEAR(t_lo, t_s + 0.15, 1e-12);
  EXPECT_FALSE(clamped);
  // Next tile: b lands at +0.25, lifts again at +0.40 — a real lift-off.
  ASSERT_TRUE(s.SwingStart(1, t_s + 0.42, t_lo, clamped));
  EXPECT_NEAR(t_lo, t_s + 0.40, 1e-12);
  EXPECT_FALSE(clamped);

  // A reset re-anchors gait 0 — in a table whose gait 0 is hop that is a
  // swing phase for b at once: clamped at the reset time.
  GaitTable hop_first;
  {
    YAML::Node y = YAML::Load(rtc::mpc::test_utils::kLocoGaitYaml);
    y["sequence"] = YAML::Load("[hop, walk]");
    y["gaits"]["hop"]["slower_below"] = YAML::Load("[0.0, 0.0]");
    y["gaits"]["hop"]["faster_above"] = YAML::Load("[0.1, 0.1]");
    y["gaits"]["walk"]["slower_below"] = YAML::Load("[0.05, 0.05]");
    y["gaits"]["walk"]["faster_above"] = YAML::Load("[0.5, 0.5]");
    y["gaits"].remove("stance");
    ASSERT_EQ(rtc::mpc::LoadGaitTable(y, rtc::mpc::test_utils::LocoContactNames(), hop_first),
              rtc::mpc::GaitTableError::kNoError);
  }
  GaitScheduler r;
  ASSERT_EQ(r.Init(hop_first, 1.0), GaitSchedulerError::kNoError);
  ASSERT_EQ(r.Advance(30.0, FilteredCommand{}, PlanarVelocity{}), GaitSchedulerError::kNoError);
  ASSERT_EQ(r.Advance(30.3, FilteredCommand{}, PlanarVelocity{}), GaitSchedulerError::kNoError);
  r.RequestReset();
  ASSERT_EQ(r.Advance(30.4, FilteredCommand{}, PlanarVelocity{}), GaitSchedulerError::kNoError);
  ASSERT_TRUE(r.SwingStart(1, 30.41, t_lo, clamped));
  EXPECT_NEAR(t_lo, 30.4, 1e-12);
  EXPECT_TRUE(clamped);
  GaitPhaseInfo after{};
  ASSERT_TRUE(r.PhaseInfo(after));
  EXPECT_EQ(after.anchor, 30.4);
}

// Far from the anchor: the phase stays in [0, 1) at 1e9 s, and SwingStart
// costs the same after a day of one gait as after a second (it starts at the
// tile of t, not at the take-over).
TEST(GaitPhase, LargeTimesAndALongRunningGaitStayCheap) {
  GaitTable table;
  GaitScheduler s;
  Walking(1e9, table, s);
  GaitPhaseInfo info{};
  ASSERT_TRUE(s.PhaseInfo(info));
  for (int k = 0; k <= 1000; ++k) {
    const double edge = info.anchor + k * 0.9;
    for (const double t : {edge, std::nextafter(edge, -kInf), std::nextafter(edge, kInf)}) {
      const double phi = GaitPhase(info, t);
      ASSERT_GE(phi, 0.0) << t;
      ASSERT_LT(phi, 1.0) << t;
      ASSERT_LE(Circular(phi, TilePhase(info.anchor, 0.9, t)), 1e-6) << t;  // ulp(1e9) ≈ 1e-7
    }
  }
  // A day later in the same gait: 1e5 lookups well under a second (an O(time
  // since take-over) scan would be ~1e5 tiles per call).
  const double late = info.anchor + 86400.0 + 0.40;
  double t_lo = 0.0;
  bool clamped = true;
  const auto t0 = std::chrono::steady_clock::now();
  std::size_t hits = 0;
  for (int i = 0; i < 100000; ++i) {
    hits += s.SwingStart(0, late + 1e-9 * i, t_lo, clamped) ? 1U : 0U;
  }
  const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_EQ(hits, 100000U);
  EXPECT_NEAR(t_lo, info.anchor + 86400.0 + 0.37, 1e-5);
  EXPECT_FALSE(clamped);
  EXPECT_LT(secs, 2.0);
}

}  // namespace
