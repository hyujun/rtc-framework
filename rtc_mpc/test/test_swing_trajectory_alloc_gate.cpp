/// @file test_swing_trajectory_alloc_gate.cpp
/// @brief BuildSwing and SwingTrajectory::Evaluate run on the MPC thread
///        (SCHED_FIFO, RT path): neither may touch the heap. Two gates in one
///        TU — the C-level malloc gate (sees other libraries and this package's
///        static library) and the operator-new counter. The positive controls
///        prove both fire: YAML::Load, and the swing loader itself inside the
///        gate (it reads yaml-cpp nodes — configure time, allowed to allocate).
///
/// malloc_gate.hpp may be included in exactly one translation unit per binary.
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_mpc/reference/swing_trajectory.hpp"
#include "test_utils/alloc_counter.hpp"
#include "test_utils/alloc_counter_overrides.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstddef>
#include <limits>

namespace {

using rtc::mpc::BuildSwing;
using rtc::mpc::SwingConfig;
using rtc::mpc::SwingError;
using rtc::mpc::SwingInterval;
using rtc::mpc::SwingSample;
using rtc::mpc::SwingTrajectory;
using rtc::mpc::test_utils::AllocCounter;

constexpr const char* kYaml = R"(
height: 0.08
lift_off_velocity: 0.05
touch_down_velocity: -0.03
touch_down_offset: -0.001
time_scale: 0.4
impact_proximity:
  mid: 0.005
  lift_off_velocity: -0.15
  touch_down_velocity: 0.3
)";

TEST(SwingAllocGate, PositiveControlsLibraryAllocationAndTheLoaderAreCounted) {
  std::size_t malloc_load = 0, malloc_parse = 0;
  AllocCounter::Arm();
  {
    const rtc::testing::ScopedMallocGate gate;
    const YAML::Node n = YAML::Load(kYaml);  // allocates inside libyaml-cpp
    malloc_load = gate.count();
  }
  const std::int64_t new_load = AllocCounter::AllocCount();
  AllocCounter::Disarm();
  const YAML::Node parsed = YAML::Load(kYaml);
  SwingConfig cfg;
  AllocCounter::Arm();
  {
    const rtc::testing::ScopedMallocGate gate;
    EXPECT_EQ(rtc::mpc::LoadSwingConfig(parsed, cfg), SwingError::kNoError);
    malloc_parse = gate.count();
  }
  const std::int64_t new_parse = AllocCounter::AllocCount();
  AllocCounter::Disarm();
  EXPECT_GT(malloc_load, 0U);
  EXPECT_GT(new_load, 0);
  EXPECT_GT(malloc_parse, 0U);
  EXPECT_GT(new_parse, 0);
}

TEST(SwingAllocGate, BuildAndEvaluateAllocateNothing) {
  SwingConfig cfg;
  ASSERT_EQ(rtc::mpc::LoadSwingConfig(YAML::Load(kYaml), cfg), SwingError::kNoError);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  SwingInterval closed{1.0, 1.4, false, false};
  SwingInterval open_end{1.0, 1.2, false, true};
  SwingInterval open_start{1.0, 1.2, true, false};
  SwingInterval both{1.0, 1.2, true, true};
  SwingInterval degenerate{1.0, 1.0, false, false};
  SwingTrajectory t[6];
  double sink = 0.0;
  std::size_t malloc_count = 0;
  int errors = 0;
  AllocCounter::Arm();
  {
    const rtc::testing::ScopedMallocGate gate;
    for (int rep = 0; rep < 100; ++rep) {
      errors += BuildSwing(closed, cfg, 0.137, t[0]) != SwingError::kNoError;
      errors += BuildSwing(open_end, cfg, 0.137, t[1]) != SwingError::kNoError;
      errors += BuildSwing(open_start, cfg, 0.137, t[2]) != SwingError::kNoError;
      errors += BuildSwing(both, cfg, 0.137, t[3]) != SwingError::kNoError;
      t[4] = SwingTrajectory::Stance(0.137);
      errors += BuildSwing(degenerate, cfg, 0.137, t[5]) != SwingError::kDegenerateInterval;
      errors += BuildSwing(closed, cfg, nan, t[5]) != SwingError::kNonFinite;
    }
    for (int i = 0; i < 10000; ++i) {
      const double u = 0.8 + 0.8 * i / 9999.0;  // inside, outside and the ends of every case
      for (const SwingTrajectory& s : t) {
        const SwingSample p = s.Evaluate(u);
        sink += p.z + p.z_dot + p.z_ddot + p.impact + p.impact_dot;
      }
      if (i % 1000 == 0) {
        sink += t[0].Evaluate(nan).z + t[0].Evaluate(inf).z + t[0].Evaluate(-inf).z +
                t[0].Evaluate(1.0).z + t[0].Evaluate(1.4).z + t[0].Evaluate(1.2).z;
      }
    }
    malloc_count = gate.count();
  }
  AllocCounter::Disarm();
  EXPECT_EQ(malloc_count, 0U);
  EXPECT_EQ(AllocCounter::AllocCount(), 0);
  EXPECT_EQ(errors, 0);
  EXPECT_TRUE(std::isfinite(sink));
}

}  // namespace
