// Unit tests for hand_timing_profiler.hpp — per-phase timing measurement.
//
// Tier 1: Pure computation, no ROS2 or network dependencies.

#include <gtest/gtest.h>

#include "ur5e_hand_driver/hand_timing_profiler.hpp"

#include <string>

namespace rtc::test {

// ── Initial state ───────────────────────────────────────────────────────────

TEST(HandTimingProfiler, InitialState_Empty)
{
  HandTimingProfiler profiler;
  const auto stats = profiler.GetStats();

  EXPECT_EQ(stats.count, 0u);
  EXPECT_EQ(stats.individual_count, 0u);
  EXPECT_EQ(stats.bulk_count, 0u);
  EXPECT_EQ(stats.sensor_cycle_count, 0u);
  EXPECT_EQ(stats.ft_infer_count, 0u);
  EXPECT_FALSE(stats.is_bulk_mode);
}

TEST(HandTimingProfiler, Summary_Empty)
{
  HandTimingProfiler profiler;
  const auto summary = profiler.Summary();
  EXPECT_NE(summary.find("no data"), std::string::npos);
}

// ── Individual mode ─────────────────────────────────────────────────────────

class IndividualModeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    HandTimingProfiler::PhaseTiming pt;
    pt.write_us = 100.0;
    pt.read_pos_us = 200.0;
    pt.read_joint_pos_us = 150.0;
    pt.read_vel_us = 180.0;
    pt.read_sensor_us = 300.0;
    pt.sensor_proc_us = 50.0;
    pt.ft_infer_us = 0.0;
    pt.total_us = 980.0;
    pt.is_sensor_cycle = true;
    pt.is_bulk_mode = false;
    profiler_.Update(pt);
  }

  HandTimingProfiler profiler_;
};

TEST_F(IndividualModeTest, SingleUpdate)
{
  const auto s = profiler_.GetStats();

  EXPECT_EQ(s.count, 1u);
  EXPECT_EQ(s.individual_count, 1u);
  EXPECT_EQ(s.bulk_count, 0u);
  EXPECT_FALSE(s.is_bulk_mode);
  EXPECT_EQ(s.sensor_cycle_count, 1u);
  EXPECT_DOUBLE_EQ(s.write.mean_us, 100.0);
  EXPECT_DOUBLE_EQ(s.read_pos.mean_us, 200.0);
  EXPECT_DOUBLE_EQ(s.read_joint_pos.mean_us, 150.0);
  EXPECT_DOUBLE_EQ(s.read_vel.mean_us, 180.0);
  EXPECT_DOUBLE_EQ(s.read_sensor.mean_us, 300.0);
}

TEST_F(IndividualModeTest, MultipleUpdates_MinMax)
{
  // Second update with different values
  HandTimingProfiler::PhaseTiming pt2;
  pt2.write_us = 50.0;
  pt2.read_pos_us = 300.0;
  pt2.read_joint_pos_us = 100.0;
  pt2.read_vel_us = 220.0;
  pt2.read_sensor_us = 0.0;
  pt2.sensor_proc_us = 0.0;
  pt2.total_us = 670.0;
  pt2.is_sensor_cycle = false;
  pt2.is_bulk_mode = false;
  profiler_.Update(pt2);

  const auto s = profiler_.GetStats();
  EXPECT_EQ(s.count, 2u);
  EXPECT_EQ(s.individual_count, 2u);

  // write: min=50, max=100
  EXPECT_DOUBLE_EQ(s.write.min_us, 50.0);
  EXPECT_DOUBLE_EQ(s.write.max_us, 100.0);
  EXPECT_DOUBLE_EQ(s.write.mean_us, 75.0);  // (100+50)/2

  // read_pos: min=200, max=300
  EXPECT_DOUBLE_EQ(s.read_pos.min_us, 200.0);
  EXPECT_DOUBLE_EQ(s.read_pos.max_us, 300.0);

  // sensor_cycle_count unchanged (second was not sensor cycle)
  EXPECT_EQ(s.sensor_cycle_count, 1u);
}

TEST(HandTimingProfiler, NonSensorCycle_SensorStatsUnchanged)
{
  HandTimingProfiler profiler;

  HandTimingProfiler::PhaseTiming pt;
  pt.write_us = 100.0;
  pt.read_pos_us = 200.0;
  pt.read_joint_pos_us = 150.0;
  pt.read_vel_us = 180.0;
  pt.read_sensor_us = 999.0;  // should NOT count
  pt.sensor_proc_us = 0.0;
  pt.total_us = 629.0;
  pt.is_sensor_cycle = false;
  pt.is_bulk_mode = false;
  profiler.Update(pt);

  const auto s = profiler.GetStats();
  EXPECT_EQ(s.sensor_cycle_count, 0u);
  // read_sensor stats should have default values since no sensor cycle
  EXPECT_DOUBLE_EQ(s.read_sensor.mean_us, 0.0);
}

// ── Bulk mode ───────────────────────────────────────────────────────────────

TEST(HandTimingProfiler, BulkMode_SingleUpdate)
{
  HandTimingProfiler profiler;

  HandTimingProfiler::PhaseTiming pt;
  pt.write_us = 80.0;
  pt.read_all_motor_us = 250.0;
  pt.read_all_joint_motor_us = 240.0;
  pt.read_all_sensor_us = 400.0;
  pt.sensor_proc_us = 30.0;
  pt.ft_infer_us = 0.0;
  pt.total_us = 1000.0;
  pt.is_sensor_cycle = true;
  pt.is_bulk_mode = true;
  profiler.Update(pt);

  const auto s = profiler.GetStats();
  EXPECT_EQ(s.count, 1u);
  EXPECT_EQ(s.bulk_count, 1u);
  EXPECT_EQ(s.individual_count, 0u);
  EXPECT_TRUE(s.is_bulk_mode);
  EXPECT_DOUBLE_EQ(s.write.mean_us, 80.0);
  EXPECT_DOUBLE_EQ(s.read_all_motor.mean_us, 250.0);
  EXPECT_DOUBLE_EQ(s.read_all_joint_motor.mean_us, 240.0);
  EXPECT_DOUBLE_EQ(s.read_all_sensor.mean_us, 400.0);
  EXPECT_EQ(s.sensor_cycle_count, 1u);
}

TEST(HandTimingProfiler, BulkMode_NonSensorCycle)
{
  HandTimingProfiler profiler;

  HandTimingProfiler::PhaseTiming pt;
  pt.write_us = 80.0;
  pt.read_all_motor_us = 250.0;
  pt.read_all_joint_motor_us = 240.0;
  pt.read_all_sensor_us = 0.0;
  pt.sensor_proc_us = 0.0;
  pt.total_us = 570.0;
  pt.is_sensor_cycle = false;
  pt.is_bulk_mode = true;
  profiler.Update(pt);

  const auto s = profiler.GetStats();
  EXPECT_EQ(s.sensor_cycle_count, 0u);
  EXPECT_DOUBLE_EQ(s.read_all_sensor.mean_us, 0.0);
}

// ── FT inference tracking ───────────────────────────────────────────────────

TEST(HandTimingProfiler, FTInfer_TrackedWhenPositive)
{
  HandTimingProfiler profiler;

  HandTimingProfiler::PhaseTiming pt{};
  pt.ft_infer_us = 150.0;
  pt.total_us = 500.0;
  pt.is_sensor_cycle = true;
  profiler.Update(pt);

  const auto s = profiler.GetStats();
  EXPECT_EQ(s.ft_infer_count, 1u);
  EXPECT_DOUBLE_EQ(s.ft_infer.mean_us, 150.0);
}

TEST(HandTimingProfiler, FTInfer_IgnoredWhenZero)
{
  HandTimingProfiler profiler;

  HandTimingProfiler::PhaseTiming pt{};
  pt.ft_infer_us = 0.0;
  pt.total_us = 500.0;
  profiler.Update(pt);

  const auto s = profiler.GetStats();
  EXPECT_EQ(s.ft_infer_count, 0u);
}

// ── Sensor processing phase ─────────────────────────────────────────────────

TEST(HandTimingProfiler, SensorProc_TrackedOnSensorCycle)
{
  HandTimingProfiler profiler;

  HandTimingProfiler::PhaseTiming pt{};
  pt.sensor_proc_us = 42.0;
  pt.total_us = 500.0;
  pt.is_sensor_cycle = true;
  profiler.Update(pt);

  const auto s = profiler.GetStats();
  EXPECT_DOUBLE_EQ(s.sensor_proc.mean_us, 42.0);
}

TEST(HandTimingProfiler, SensorProc_NotTrackedOnNonSensorCycle)
{
  HandTimingProfiler profiler;

  HandTimingProfiler::PhaseTiming pt{};
  pt.sensor_proc_us = 42.0;
  pt.total_us = 500.0;
  pt.is_sensor_cycle = false;
  profiler.Update(pt);

  const auto s = profiler.GetStats();
  EXPECT_DOUBLE_EQ(s.sensor_proc.mean_us, 0.0);
}

// ── Reset ───────────────────────────────────────────────────────────────────

TEST(HandTimingProfiler, Reset_ClearsAll)
{
  HandTimingProfiler profiler;

  HandTimingProfiler::PhaseTiming pt;
  pt.write_us = 100.0;
  pt.read_all_motor_us = 200.0;
  pt.read_all_joint_motor_us = 180.0;
  pt.read_all_sensor_us = 300.0;
  pt.sensor_proc_us = 50.0;
  pt.ft_infer_us = 75.0;
  pt.total_us = 905.0;
  pt.is_sensor_cycle = true;
  pt.is_bulk_mode = true;
  profiler.Update(pt);

  profiler.Reset();

  const auto s = profiler.GetStats();
  EXPECT_EQ(s.count, 0u);
  EXPECT_EQ(s.individual_count, 0u);
  EXPECT_EQ(s.bulk_count, 0u);
  EXPECT_EQ(s.sensor_cycle_count, 0u);
  EXPECT_EQ(s.ft_infer_count, 0u);
  EXPECT_FALSE(s.is_bulk_mode);
}

// ── Summary strings ─────────────────────────────────────────────────────────

TEST(HandTimingProfiler, Summary_IndividualMode)
{
  HandTimingProfiler profiler;

  HandTimingProfiler::PhaseTiming pt{};
  pt.write_us = 100.0;
  pt.read_pos_us = 200.0;
  pt.read_joint_pos_us = 150.0;
  pt.read_vel_us = 180.0;
  pt.total_us = 630.0;
  pt.is_bulk_mode = false;
  profiler.Update(pt);

  const auto summary = profiler.Summary();
  EXPECT_NE(summary.find("HandUDP timing:"), std::string::npos);
  // Should NOT contain "[bulk]"
  EXPECT_EQ(summary.find("[bulk]"), std::string::npos);
}

TEST(HandTimingProfiler, Summary_BulkMode)
{
  HandTimingProfiler profiler;

  HandTimingProfiler::PhaseTiming pt{};
  pt.write_us = 80.0;
  pt.read_all_motor_us = 250.0;
  pt.read_all_joint_motor_us = 240.0;
  pt.total_us = 570.0;
  pt.is_bulk_mode = true;
  profiler.Update(pt);

  const auto summary = profiler.Summary();
  EXPECT_NE(summary.find("[bulk]"), std::string::npos);
}

TEST(HandTimingProfiler, Summary_WithFTInfer)
{
  HandTimingProfiler profiler;

  HandTimingProfiler::PhaseTiming pt{};
  pt.ft_infer_us = 120.0;
  pt.total_us = 500.0;
  profiler.Update(pt);

  const auto summary = profiler.Summary();
  EXPECT_NE(summary.find("ft="), std::string::npos);
}

}  // namespace rtc::test
