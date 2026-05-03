// Unit tests for hand_sensor_processor.hpp — sensor post-processing pipeline.
//
// Tier 1: Pure computation. Depends only on rtc_base filters.
// Note: rclcpp logging is used internally but doesn't require node init for
// standalone calls (get_logger works without rclcpp::init).

#include "udp_hand_driver/udp_hand_sensor_processor.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>

namespace udp_hand_driver::test {

// ── Construction & Init ─────────────────────────────────────────────────────

TEST(UdpHandSensorProcessor, DefaultConfig_ZeroFingertips) {
  UdpHandSensorProcessorConfig cfg{};
  cfg.num_fingertips = 0;
  UdpHandSensorProcessor proc(cfg);

  // Init with zero fingertips should be a no-op (no crash).
  // rtc::SensorRateEstimator defaults to 500Hz nominal before Init().
  proc.Init();
  EXPECT_GE(proc.actual_sensor_rate_hz(), 0.0);
}

TEST(UdpHandSensorProcessor, NegativeDecimation_ClampedToOne) {
  UdpHandSensorProcessorConfig cfg{};
  cfg.num_fingertips = 1;
  cfg.sensor_decimation = -5;
  cfg.baro_lpf_enabled = true;
  cfg.baro_lpf_cutoff_hz = 30.0;
  UdpHandSensorProcessor proc(cfg);

  // Should not crash; decimation clamped to 1
  proc.Init();
}

// ── ApplyFilters: No filters ────────────────────────────────────────────────

TEST(UdpHandSensorProcessor, NoFilters_Passthrough) {
  UdpHandSensorProcessorConfig cfg{};
  cfg.num_fingertips = 2;
  cfg.baro_lpf_enabled = false;
  cfg.tof_lpf_enabled = false;
  UdpHandSensorProcessor proc(cfg);
  proc.Init();

  std::array<int32_t, udp_hand_driver::kMaxHandSensors> data{};
  // Fill finger 0 and finger 1 sensor data
  for (int f = 0; f < 2; ++f) {
    const int base = f * udp_hand_driver::kSensorValuesPerFingertip;
    for (int i = 0; i < udp_hand_driver::kSensorValuesPerFingertip; ++i) {
      data[static_cast<std::size_t>(base + i)] = static_cast<int32_t>(f * 100 + i + 1);
    }
  }

  auto original = data;
  proc.ApplyFilters(data);

  // With no filters enabled, data should remain unchanged
  EXPECT_EQ(data, original);
}

// ── ApplyFilters: Barometer LPF ─────────────────────────────────────────────

class BaroLPFTest : public ::testing::Test {
 protected:
  void SetUp() override {
    UdpHandSensorProcessorConfig cfg{};
    cfg.num_fingertips = 1;
    cfg.sensor_decimation = 1;
    cfg.baro_lpf_enabled = true;
    cfg.baro_lpf_cutoff_hz = 30.0;
    cfg.tof_lpf_enabled = false;
    proc_ = std::make_unique<UdpHandSensorProcessor>(cfg);
    proc_->Init();
  }

  std::unique_ptr<UdpHandSensorProcessor> proc_;
};

TEST_F(BaroLPFTest, StepInput_Smoothed) {
  // Feed constant zero for a few cycles to settle filter, then step
  std::array<int32_t, udp_hand_driver::kMaxHandSensors> data{};

  // Settle with zeros
  for (int i = 0; i < 10; ++i) {
    data.fill(0);
    proc_->ApplyFilters(data);
  }

  // Step to 10000 on all barometer channels
  data.fill(0);
  for (int b = 0; b < udp_hand_driver::kBarometerCount; ++b) {
    data[static_cast<std::size_t>(b)] = 10000;
  }
  proc_->ApplyFilters(data);

  // After one step, the filter output should be less than the step value
  // (LPF attenuates sudden changes)
  for (int b = 0; b < udp_hand_driver::kBarometerCount; ++b) {
    EXPECT_LT(data[static_cast<std::size_t>(b)], 10000);
    EXPECT_GE(data[static_cast<std::size_t>(b)], 0);
  }
}

TEST_F(BaroLPFTest, TofUnchanged) {
  // ToF filter is disabled — ToF values should pass through
  std::array<int32_t, udp_hand_driver::kMaxHandSensors> data{};
  for (int t = 0; t < udp_hand_driver::kTofCount; ++t) {
    data[static_cast<std::size_t>(udp_hand_driver::kBarometerCount + t)] = 5000;
  }

  proc_->ApplyFilters(data);

  // ToF values should remain unchanged (no ToF filter)
  for (int t = 0; t < udp_hand_driver::kTofCount; ++t) {
    EXPECT_EQ(data[static_cast<std::size_t>(udp_hand_driver::kBarometerCount + t)], 5000);
  }
}

// ── ApplyFilters: ToF LPF ───────────────────────────────────────────────────

TEST(UdpHandSensorProcessor, TofLPF_StepInput_Smoothed) {
  UdpHandSensorProcessorConfig cfg{};
  cfg.num_fingertips = 1;
  cfg.sensor_decimation = 1;
  cfg.baro_lpf_enabled = false;
  cfg.tof_lpf_enabled = true;
  cfg.tof_lpf_cutoff_hz = 15.0;
  UdpHandSensorProcessor proc(cfg);
  proc.Init();

  std::array<int32_t, udp_hand_driver::kMaxHandSensors> data{};

  // Settle
  for (int i = 0; i < 10; ++i) {
    data.fill(0);
    proc.ApplyFilters(data);
  }

  // Step on ToF channels
  data.fill(0);
  for (int t = 0; t < udp_hand_driver::kTofCount; ++t) {
    data[static_cast<std::size_t>(udp_hand_driver::kBarometerCount + t)] = 8000;
  }
  proc.ApplyFilters(data);

  for (int t = 0; t < udp_hand_driver::kTofCount; ++t) {
    EXPECT_LT(data[static_cast<std::size_t>(udp_hand_driver::kBarometerCount + t)], 8000);
    EXPECT_GE(data[static_cast<std::size_t>(udp_hand_driver::kBarometerCount + t)], 0);
  }
}

// ── ApplyFilters: Both filters ──────────────────────────────────────────────

TEST(UdpHandSensorProcessor, BothFilters_Applied) {
  UdpHandSensorProcessorConfig cfg{};
  cfg.num_fingertips = 1;
  cfg.sensor_decimation = 1;
  cfg.baro_lpf_enabled = true;
  cfg.baro_lpf_cutoff_hz = 30.0;
  cfg.tof_lpf_enabled = true;
  cfg.tof_lpf_cutoff_hz = 15.0;
  UdpHandSensorProcessor proc(cfg);
  proc.Init();

  std::array<int32_t, udp_hand_driver::kMaxHandSensors> data{};

  // Settle
  for (int i = 0; i < 10; ++i) {
    data.fill(0);
    proc.ApplyFilters(data);
  }

  // Step both
  for (int b = 0; b < udp_hand_driver::kBarometerCount; ++b) {
    data[static_cast<std::size_t>(b)] = 10000;
  }
  for (int t = 0; t < udp_hand_driver::kTofCount; ++t) {
    data[static_cast<std::size_t>(udp_hand_driver::kBarometerCount + t)] = 8000;
  }
  proc.ApplyFilters(data);

  // Both should be attenuated
  for (int b = 0; b < udp_hand_driver::kBarometerCount; ++b) {
    EXPECT_LT(data[static_cast<std::size_t>(b)], 10000);
  }
  for (int t = 0; t < udp_hand_driver::kTofCount; ++t) {
    EXPECT_LT(data[static_cast<std::size_t>(udp_hand_driver::kBarometerCount + t)], 8000);
  }
}

// ── ApplyFilters: Multi-finger layout ───────────────────────────────────────

TEST(UdpHandSensorProcessor, MultiFingerLayout_CorrectIndexing) {
  UdpHandSensorProcessorConfig cfg{};
  cfg.num_fingertips = 4;
  cfg.sensor_decimation = 1;
  cfg.baro_lpf_enabled = true;
  cfg.baro_lpf_cutoff_hz = 30.0;
  cfg.tof_lpf_enabled = false;
  UdpHandSensorProcessor proc(cfg);
  proc.Init();

  std::array<int32_t, udp_hand_driver::kMaxHandSensors> data{};

  // Settle
  for (int i = 0; i < 10; ++i) {
    data.fill(0);
    proc.ApplyFilters(data);
  }

  // Set different values per finger
  for (int f = 0; f < 4; ++f) {
    const int base = f * udp_hand_driver::kSensorValuesPerFingertip;
    for (int b = 0; b < udp_hand_driver::kBarometerCount; ++b) {
      data[static_cast<std::size_t>(base + b)] = static_cast<int32_t>((f + 1) * 1000);
    }
  }

  proc.ApplyFilters(data);

  // Each finger should have attenuated values, and finger ordering preserved
  for (int f = 0; f < 4; ++f) {
    const int base = f * udp_hand_driver::kSensorValuesPerFingertip;
    const int step_val = (f + 1) * 1000;
    for (int b = 0; b < udp_hand_driver::kBarometerCount; ++b) {
      const auto idx = static_cast<std::size_t>(base + b);
      EXPECT_LT(data[idx], step_val);
      EXPECT_GE(data[idx], 0);
    }
  }

  // Larger step → larger (or equal) filtered value per finger
  for (int f = 1; f < 4; ++f) {
    const auto idx_prev = static_cast<std::size_t>((f - 1) * udp_hand_driver::kSensorValuesPerFingertip);
    const auto idx_curr = static_cast<std::size_t>(f * udp_hand_driver::kSensorValuesPerFingertip);
    EXPECT_GE(data[idx_curr], data[idx_prev]);
  }
}

// ── ApplyFilters: Convergence ───────────────────────────────────────────────

TEST(UdpHandSensorProcessor, BaroLPF_ConvergesToSteadyState) {
  UdpHandSensorProcessorConfig cfg{};
  cfg.num_fingertips = 1;
  cfg.sensor_decimation = 1;
  cfg.baro_lpf_enabled = true;
  cfg.baro_lpf_cutoff_hz = 50.0;
  UdpHandSensorProcessor proc(cfg);
  proc.Init();

  std::array<int32_t, udp_hand_driver::kMaxHandSensors> data{};
  constexpr int32_t kTarget = 5000;

  // Feed constant value for many cycles
  for (int i = 0; i < 500; ++i) {
    data.fill(0);
    for (int b = 0; b < udp_hand_driver::kBarometerCount; ++b) {
      data[static_cast<std::size_t>(b)] = kTarget;
    }
    proc.ApplyFilters(data);
  }

  // After convergence, output should be very close to target
  for (int b = 0; b < udp_hand_driver::kBarometerCount; ++b) {
    EXPECT_NEAR(data[static_cast<std::size_t>(b)], kTarget, 10);
  }
}

// ── DetectDrift ─────────────────────────────────────────────────────────────

TEST(UdpHandSensorProcessor, DetectDrift_Disabled_NoOp) {
  UdpHandSensorProcessorConfig cfg{};
  cfg.num_fingertips = 1;
  cfg.drift_detection_enabled = false;
  UdpHandSensorProcessor proc(cfg);
  proc.Init();

  std::array<int32_t, udp_hand_driver::kMaxHandSensors> raw{};
  // Should not crash even with drift-like data
  for (int i = 0; i < 100; ++i) {
    for (int b = 0; b < udp_hand_driver::kBarometerCount; ++b) {
      raw[static_cast<std::size_t>(b)] = i * 100;  // linear ramp
    }
    proc.DetectDrift(raw);
  }
}

TEST(UdpHandSensorProcessor, DetectDrift_ConstantData_NoDrift) {
  UdpHandSensorProcessorConfig cfg{};
  cfg.num_fingertips = 1;
  cfg.drift_detection_enabled = true;
  cfg.drift_threshold = 5.0;
  cfg.drift_window_size = 50;
  UdpHandSensorProcessor proc(cfg);
  proc.Init();

  std::array<int32_t, udp_hand_driver::kMaxHandSensors> raw{};
  for (int b = 0; b < udp_hand_driver::kBarometerCount; ++b) {
    raw[static_cast<std::size_t>(b)] = 1000;  // constant
  }

  for (int i = 0; i < 100; ++i) {
    proc.DetectDrift(raw);
  }
  // No crash, no assertion — constant data should not trigger drift
}

// ── Rate accessor ───────────────────────────────────────────────────────────

TEST(UdpHandSensorProcessor, ActualSensorRateHz_DefaultNominal) {
  UdpHandSensorProcessorConfig cfg{};
  cfg.num_fingertips = 1;
  UdpHandSensorProcessor proc(cfg);

  // Before Init(), rtc::SensorRateEstimator returns its default nominal rate (500Hz)
  EXPECT_DOUBLE_EQ(proc.actual_sensor_rate_hz(), 500.0);
}

TEST(UdpHandSensorProcessor, PreFilter_TicksRateEstimator) {
  UdpHandSensorProcessorConfig cfg{};
  cfg.num_fingertips = 1;
  UdpHandSensorProcessor proc(cfg);
  proc.Init();

  // After Init, rate should be near nominal (or at least non-negative)
  // Call PreFilter multiple times
  for (int i = 0; i < 10; ++i) {
    proc.PreFilter();
  }

  // Rate should be non-negative
  EXPECT_GE(proc.actual_sensor_rate_hz(), 0.0);
}

}  // namespace udp_hand_driver::test
