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

TEST_F(FingertipFTInferencerTest, CalibrationTarget_Default)
{
  // Before InitFT(), calibration_target returns the default config value.
  // Real impl (HAS_ONNXRUNTIME): config_.calibration_samples = 500
  // Stub: returns 0
#ifdef HAS_ONNXRUNTIME
  EXPECT_EQ(inferencer_.calibration_target(), 500);
#else
  EXPECT_EQ(inferencer_.calibration_target(), 0);
#endif
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

// ── InitFT with enabled=true but no model paths ────────────────────────────

TEST(FingertipFTInferencerInit, EnabledNoModels_InitializedFalse)
{
  FingertipFTInferencer inf;
  FingertipFTInferencer::Config cfg{};
  cfg.enabled = true;  // enabled but no model_paths
  cfg.num_fingertips = 4;
  EXPECT_NO_THROW(inf.InitFT(cfg));
  EXPECT_FALSE(inf.is_initialized());  // no models loaded
}

TEST(FingertipFTInferencerInit, EnabledNoModels_InferReturnsInvalid)
{
  FingertipFTInferencer inf;
  FingertipFTInferencer::Config cfg{};
  cfg.enabled = true;
  cfg.num_fingertips = 4;
  inf.InitFT(cfg);

  std::array<int32_t, kMaxHandSensors> sensor_data{};
  const auto result = inf.Infer(sensor_data, 4);
  EXPECT_FALSE(result.valid);
}

// ── FeedCalibration on uninitialized inferencer ─────────────────────────────

TEST(FingertipFTInferencerInit, FeedCalibration_Uninitialized)
{
  FingertipFTInferencer inf;
  std::array<int32_t, kMaxHandSensors> sensor_data{};
  // Should not crash on uninitialized inferencer
  [[maybe_unused]] bool result = inf.FeedCalibration(sensor_data, 4);
}

// ── Config custom values ───────────────────────────────────────────────────

TEST(FingertipFTInferencerConfig, CustomValues)
{
  FingertipFTInferencer::Config cfg{};
  cfg.enabled = true;
  cfg.num_fingertips = 2;
  cfg.history_length = 8;
  cfg.calibration_enabled = false;
  cfg.calibration_samples = 100;

  EXPECT_TRUE(cfg.enabled);
  EXPECT_EQ(cfg.num_fingertips, 2);
  EXPECT_EQ(cfg.history_length, 8);
  EXPECT_FALSE(cfg.calibration_enabled);
  EXPECT_EQ(cfg.calibration_samples, 100);
}

// ── InitFT with calibration disabled — immediately calibrated ───────────────

#ifdef HAS_ONNXRUNTIME
TEST(FingertipFTInferencerInit, CalibrationDisabled_ImmediatelyCalibrated)
{
  FingertipFTInferencer inf;
  FingertipFTInferencer::Config cfg{};
  cfg.enabled = true;
  cfg.num_fingertips = 4;
  cfg.calibration_enabled = false;
  inf.InitFT(cfg);

  // With calibration disabled, should be immediately calibrated
  // (even though no models loaded)
  EXPECT_TRUE(inf.is_calibrated());
}

TEST(FingertipFTInferencerInit, CalibrationEnabled_NotCalibratedInitially)
{
  FingertipFTInferencer inf;
  FingertipFTInferencer::Config cfg{};
  cfg.enabled = true;
  cfg.num_fingertips = 4;
  cfg.calibration_enabled = true;
  cfg.calibration_samples = 10;
  inf.InitFT(cfg);

  // With calibration enabled, should not be calibrated until samples fed
  EXPECT_FALSE(inf.is_calibrated());
  EXPECT_EQ(inf.calibration_count(), 0);
  EXPECT_EQ(inf.calibration_target(), 10);
}
#endif

}  // namespace rtc::test
