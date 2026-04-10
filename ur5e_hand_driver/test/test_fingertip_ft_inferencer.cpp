// Unit tests for fingertip_ft_inferencer.hpp — ONNX stub implementation.
//
// Tier 3: Tests the #ifndef HAS_ONNXRUNTIME stub path.
// When ONNX Runtime is available, these test the stub fallback behavior.
// When ONNX Runtime IS available, these tests still pass (InitFT behavior differs).

#include <gtest/gtest.h>

#include "ur5e_hand_driver/fingertip_ft_inferencer.hpp"

#include <array>
#include <cstdint>

namespace rtc::test {

// ── Stub behavior tests ────────────────────────────────────────────────────
// These tests are designed to work regardless of HAS_ONNXRUNTIME.
// When ONNX is not available, the stub class is used.

class FingertipFTInferencerTest : public ::testing::Test {
 protected:
  FingertipFTInferencer inferencer_;
};

TEST_F(FingertipFTInferencerTest, InitFT_NoException)
{
  FingertipFTInferencer::Config cfg{};
  cfg.enabled = false;
  cfg.num_fingertips = 4;
  // Should not throw
  EXPECT_NO_THROW(inferencer_.InitFT(cfg));
}

TEST_F(FingertipFTInferencerTest, IsInitialized_FalseWithoutModels)
{
  FingertipFTInferencer::Config cfg{};
  cfg.enabled = false;
  inferencer_.InitFT(cfg);

  // Without models, should not be initialized
  EXPECT_FALSE(inferencer_.is_initialized());
}

TEST_F(FingertipFTInferencerTest, IsCalibratedDefault)
{
  // Without init, default state
  EXPECT_FALSE(inferencer_.is_calibrated());
}

TEST_F(FingertipFTInferencerTest, NumModels_Zero)
{
  EXPECT_EQ(inferencer_.num_models(), 0);
}

TEST_F(FingertipFTInferencerTest, CalibrationCount_Zero)
{
  EXPECT_EQ(inferencer_.calibration_count(), 0);
}

TEST_F(FingertipFTInferencerTest, CalibrationTarget_Zero)
{
  EXPECT_EQ(inferencer_.calibration_target(), 0);
}

TEST_F(FingertipFTInferencerTest, Infer_ReturnsInvalid)
{
  std::array<int32_t, kMaxHandSensors> sensor_data{};
  const auto result = inferencer_.Infer(sensor_data, 4);
  EXPECT_FALSE(result.valid);
}

TEST_F(FingertipFTInferencerTest, FeedCalibration_ReturnsTrueOrFalse)
{
  std::array<int32_t, kMaxHandSensors> sensor_data{};
  // Stub returns true immediately
  // Real impl depends on calibration state
  // Both should not crash
  [[maybe_unused]] const bool result =
      inferencer_.FeedCalibration(sensor_data, 4);
}

TEST_F(FingertipFTInferencerTest, BaselineOffset_DefaultEmpty)
{
  const auto offsets = inferencer_.baseline_offset();
  // All offsets should be zero by default
  for (const auto& finger : offsets) {
    for (float val : finger) {
      EXPECT_FLOAT_EQ(val, 0.0f);
    }
  }
}

TEST_F(FingertipFTInferencerTest, ResetCalibration_NoCrash)
{
  // Should be safe to call on a freshly constructed inferencer
  // (both stub and real implementation must be noexcept).
  EXPECT_NO_THROW(inferencer_.ResetCalibration());
  EXPECT_NO_THROW(inferencer_.ResetCalibration(10));
  // After reset, baseline offsets remain all zero.
  const auto offsets = inferencer_.baseline_offset();
  for (const auto& finger : offsets) {
    for (float val : finger) {
      EXPECT_FLOAT_EQ(val, 0.0f);
    }
  }
}

// ── Config struct defaults ──────────────────────────────────────────────────

TEST(FingertipFTInferencerConfig, Defaults)
{
  FingertipFTInferencer::Config cfg{};
  EXPECT_FALSE(cfg.enabled);
  EXPECT_EQ(cfg.num_fingertips, kDefaultNumFingertips);
  EXPECT_EQ(cfg.history_length, kFTHistoryLength);
  EXPECT_TRUE(cfg.model_paths.empty());
  EXPECT_TRUE(cfg.calibration_enabled);
  EXPECT_EQ(cfg.calibration_samples, 500);
}

// ── FingertipFTState struct ─────────────────────────────────────────────────

TEST(FingertipFTState, DefaultValues)
{
  FingertipFTState state{};
  EXPECT_FALSE(state.valid);
  EXPECT_EQ(state.num_fingertips, 0);
  for (float val : state.ft_data) {
    EXPECT_FLOAT_EQ(val, 0.0f);
  }
  for (bool val : state.per_fingertip_valid) {
    EXPECT_FALSE(val);
  }
}

TEST(FingertipFTState, MaxFTValues)
{
  EXPECT_EQ(FingertipFTState::kMaxFTValues, kMaxFingertips * kFTValuesPerFingertip);
}

}  // namespace rtc::test
