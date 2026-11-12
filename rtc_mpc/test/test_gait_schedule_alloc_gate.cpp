/// @file test_gait_schedule_alloc_gate.cpp
/// @brief GaitScheduler runs on the MPC thread (SCHED_FIFO, RT path): from the
///        first Advance after Init on, it and the ContactSchedule helpers must
///        make no heap allocation. Two gates in this one TU: a C-level malloc
///        gate (sees allocations inside other libraries and this package's own
///        static library) and an operator-new counter. The positive control
///        proves both fire on a library allocation (YAML::Load in yaml-cpp).
///
/// malloc_gate.hpp may be included in exactly one translation unit per binary.
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_mpc/reference/contact_schedule.hpp"
#include "rtc_mpc/reference/gait_scheduler.hpp"
#include "rtc_mpc/reference/gait_table.hpp"
#include "test_utils/alloc_counter.hpp"
#include "test_utils/alloc_counter_overrides.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cstdlib>
#include <new>
#include <string>
#include <vector>

namespace {

using rtc::mpc::FilteredCommand;
using rtc::mpc::GaitScheduler;
using rtc::mpc::GaitSchedulerError;
using rtc::mpc::GaitTable;
using rtc::mpc::GaitTableError;
using rtc::mpc::PlanarVelocity;
using rtc::mpc::test_utils::AllocCounter;

const std::vector<std::string> kAbc = {"contact_a", "contact_b", "contact_c"};

constexpr const char* kYaml = R"(
sequence: [stance, walk, hop]
min_dwell: 0.2
switch_lead: 0.63
gaits:
  stance:
    modes: [[contact_a, contact_b, contact_c]]
    switching_times: [0.0, 0.5]
    faster_above: [0.1, 0.1]
    slower_below: [0.0, 0.0]
    measured_tolerance: [10.0, 10.0]
  walk:
    modes: [[contact_a, contact_c], [contact_a, contact_b, contact_c], [contact_b], [contact_a, contact_b, contact_c]]
    switching_times: [0.0, 0.13, 0.37, 0.52, 0.90]
    faster_above: [0.5, 0.5]
    slower_below: [0.05, 0.05]
    measured_tolerance: [0.05, 0.05]
  hop:
    modes: [[contact_a], [contact_b]]
    switching_times: [0.0, 0.25, 0.5]
    faster_above: [0.9, 0.9]
    slower_below: [0.45, 0.45]
    measured_tolerance: [0.05, 0.05]
)";

FilteredCommand Cmd(double vx) {
  FilteredCommand c;
  c.v_x = vx;
  c.valid = true;
  return c;
}

TEST(GaitScheduleAllocGate, PositiveControlLibraryAllocationIsCounted) {
  std::size_t malloc_seen = 0;
  AllocCounter::Arm();
  {
    const rtc::testing::ScopedMallocGate gate;
    const YAML::Node n = YAML::Load(kYaml);  // allocates inside libyaml-cpp
    malloc_seen = gate.count();
  }
  AllocCounter::Disarm();
  EXPECT_GT(malloc_seen, 0U);
  EXPECT_GT(AllocCounter::AllocCount(), 0);
}

TEST(GaitScheduleAllocGate, AdvanceAndTheScheduleHelpersAllocateNothing) {
  GaitTable table;
  ASSERT_EQ(rtc::mpc::LoadGaitTable(YAML::Load(kYaml), kAbc, table), GaitTableError::kNoError);
  GaitScheduler s;
  ASSERT_EQ(s.Init(table, 1.0), GaitSchedulerError::kNoError);
  const PlanarVelocity rest{};

  std::array<std::uint32_t, 21> nodes{};
  std::array<rtc::mpc::SwingInterval, 16> swings{};
  std::size_t transitions = 0;
  bool fallback_seen = false;
  std::uint32_t sink = 0;
  std::size_t malloc_count = 0;
  AllocCounter::Arm();
  {
    const rtc::testing::ScopedMallocGate gate;
    std::uint32_t last = 0;
    double t = 0.0;
    for (int k = 0; k < 1000; ++k, t += 0.01) {
      // 0–3 s command up: the policy goes stance → walk → hop. At 3 s a forced
      // walk: hop has no full support, so the switch falls back to the window end.
      const double v = t < 3.0 ? 1.5 : 0.0;
      const PlanarVelocity m{v, 0.0, 0.0};
      if (k == 300) {
        s.RequestGait(1);
      }
      if (s.Advance(t, Cmd(v), t < 3.0 ? m : rest) != GaitSchedulerError::kNoError) {
        ADD_FAILURE() << "Advance failed at k = " << k;
        break;
      }
      if (s.target_gait() != last) {
        ++transitions;
        last = s.target_gait();
      }
      if (k == 300 && s.switch_pending() && s.current_gait() == 2U &&
          s.switch_time() == s.schedule().t_end) {
        fallback_seen = true;
      }
    }
    const rtc::mpc::ContactSchedule& w = s.schedule();
    for (int i = 0; i < 1000; ++i) {
      sink += rtc::mpc::ModeAt(w, w.t0 + 0.001 * i);
    }
    double te = 0.0;
    std::uint32_t after = 0;
    sink += rtc::mpc::NextEvent(w, w.t0, te, after) ? after : 0U;
    sink += static_cast<std::uint32_t>(rtc::mpc::SwingIntervals(w, 0, swings));
    sink += rtc::mpc::SampleNodes(w, w.t0, 0.05, 20, nodes) ? nodes[3] : 0U;
    sink += rtc::mpc::SampleNodes(w, w.t0, 1.0, 20, nodes) ? 1U : 0U;  // refused: past t_end
    sink += s.Advance(t - 1.0, Cmd(0.0), rest) == GaitSchedulerError::kTimeWentBack ? 1U : 0U;
    malloc_count = gate.count();
  }
  AllocCounter::Disarm();
  EXPECT_EQ(malloc_count, 0U);
  EXPECT_EQ(AllocCounter::AllocCount(), 0);
  EXPECT_GE(transitions, 3U);  // two policy changes up, the forced one down
  EXPECT_TRUE(fallback_seen);
  EXPECT_GT(sink, 0U);
}

}  // namespace
