/// @file test_phase_cost_config.cpp
/// @brief Unit tests for @ref rtc::mpc::PhaseCostConfig::LoadFromYaml.
///
/// Panda (9-DoF, 2 contacts × 3-dim) is used as a generic fixture (same
/// rationale as test_robot_model_handler.cpp). Robot-specific phase naming
/// does **not** appear here — this is pure config-round-trip verification.

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "rtc_mpc/phase/phase_cost_config.hpp"

namespace {

constexpr const char *kPandaUrdf =
    RTC_PANDA_URDF_PATH;

// YAML template with placeholders; swap via string::replace in cases that
// need to mutate one entry. Provides a well-formed baseline matching
// Panda (nq=9, 2 contacts × 3-dim = 6-vec F_target).
constexpr const char *kValidYaml = R"(
horizon_length: 20
dt: 0.01
w_frame_placement: 100.0
w_state_reg: 1.0
w_control_reg: 0.01
w_contact_force: 0.001
w_centroidal_momentum: 0.0
W_placement: [100.0, 100.0, 100.0, 10.0, 10.0, 10.0]
q_posture_ref: [0, 0, 0, -1.57, 0, 1.57, 0.785, 0.02, 0.02]
F_target: [0, 0, 0, 0, 0, 0]
custom_weights:
  hand_posture: 5.0
  joint_range: 2.0
)";

class PhaseCostConfigTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!std::filesystem::exists(kPandaUrdf)) {
      GTEST_SKIP() << "Panda URDF not installed at " << kPandaUrdf
                   << " — run ./install.sh to install Aligator deps";
    }
    pinocchio::urdf::buildModel(kPandaUrdf, model_);

    auto model_cfg = YAML::Load(R"(
end_effector_frame: panda_hand_tcp
contact_frames:
  - name: panda_leftfinger
    dim: 3
  - name: panda_rightfinger
    dim: 3
)");
    ASSERT_EQ(handler_.Init(model_, model_cfg),
              rtc::mpc::RobotModelInitError::kNoError);
  }

  pinocchio::Model model_{};
  rtc::mpc::RobotModelHandler handler_{};
};

TEST_F(PhaseCostConfigTest, ValidYamlRoundTrip) {
  auto cfg = YAML::Load(kValidYaml);
  rtc::mpc::PhaseCostConfig out;

  const auto err = rtc::mpc::PhaseCostConfig::LoadFromYaml(cfg, handler_, out);
  EXPECT_EQ(err, rtc::mpc::PhaseCostConfigError::kNoError);

  EXPECT_EQ(out.horizon_length, 20);
  EXPECT_DOUBLE_EQ(out.dt, 0.01);
  EXPECT_DOUBLE_EQ(out.w_frame_placement, 100.0);
  EXPECT_DOUBLE_EQ(out.w_state_reg, 1.0);
  EXPECT_DOUBLE_EQ(out.w_control_reg, 0.01);
  EXPECT_DOUBLE_EQ(out.w_contact_force, 0.001);
  EXPECT_DOUBLE_EQ(out.w_centroidal_momentum, 0.0);

  ASSERT_EQ(out.W_placement.size(), 6);
  EXPECT_DOUBLE_EQ(out.W_placement[0], 100.0);
  EXPECT_DOUBLE_EQ(out.W_placement[5], 10.0);

  ASSERT_EQ(out.q_posture_ref.size(), 9);
  EXPECT_DOUBLE_EQ(out.q_posture_ref[3], -1.57);
  EXPECT_DOUBLE_EQ(out.q_posture_ref[8], 0.02);

  ASSERT_EQ(out.F_target.size(), 6);
  EXPECT_DOUBLE_EQ(out.F_target[0], 0.0);

  EXPECT_DOUBLE_EQ(out.CustomWeight("hand_posture"), 5.0);
  EXPECT_DOUBLE_EQ(out.CustomWeight("joint_range"), 2.0);
}

TEST_F(PhaseCostConfigTest, AbsentCustomWeightsDefaultsToEmptyMap) {
  auto cfg = YAML::Load(R"(
horizon_length: 10
dt: 0.02
w_frame_placement: 1.0
w_state_reg: 1.0
w_control_reg: 1.0
w_contact_force: 1.0
w_centroidal_momentum: 1.0
W_placement: [1, 1, 1, 1, 1, 1]
q_posture_ref: [0, 0, 0, 0, 0, 0, 0, 0, 0]
F_target: [0, 0, 0, 0, 0, 0]
)");
  rtc::mpc::PhaseCostConfig out;

  EXPECT_EQ(rtc::mpc::PhaseCostConfig::LoadFromYaml(cfg, handler_, out),
            rtc::mpc::PhaseCostConfigError::kNoError);
  EXPECT_TRUE(out.custom_weights.empty());
  EXPECT_DOUBLE_EQ(out.CustomWeight("hand_posture"), 0.0); // absent key
  EXPECT_DOUBLE_EQ(out.CustomWeight("anything"), 0.0);
}

TEST_F(PhaseCostConfigTest, CustomWeightLookupPresentAndAbsent) {
  auto cfg = YAML::Load(kValidYaml);
  rtc::mpc::PhaseCostConfig out;
  ASSERT_EQ(rtc::mpc::PhaseCostConfig::LoadFromYaml(cfg, handler_, out),
            rtc::mpc::PhaseCostConfigError::kNoError);

  EXPECT_DOUBLE_EQ(out.CustomWeight("hand_posture"), 5.0);
  EXPECT_DOUBLE_EQ(out.CustomWeight("unknown_key"), 0.0);
}

TEST_F(PhaseCostConfigTest, NegativeScalarWeightRejected) {
  auto cfg = YAML::Load(kValidYaml);
  cfg["w_state_reg"] = -1.0;

  rtc::mpc::PhaseCostConfig out;
  EXPECT_EQ(rtc::mpc::PhaseCostConfig::LoadFromYaml(cfg, handler_, out),
            rtc::mpc::PhaseCostConfigError::kInvalidWeightSign);
}

TEST_F(PhaseCostConfigTest, ZeroHorizonRejected) {
  auto cfg = YAML::Load(kValidYaml);
  cfg["horizon_length"] = 0;

  rtc::mpc::PhaseCostConfig out;
  EXPECT_EQ(rtc::mpc::PhaseCostConfig::LoadFromYaml(cfg, handler_, out),
            rtc::mpc::PhaseCostConfigError::kInvalidHorizon);
}

TEST_F(PhaseCostConfigTest, NonPositiveDtRejected) {
  auto cfg = YAML::Load(kValidYaml);
  cfg["dt"] = -0.01;

  rtc::mpc::PhaseCostConfig out;
  EXPECT_EQ(rtc::mpc::PhaseCostConfig::LoadFromYaml(cfg, handler_, out),
            rtc::mpc::PhaseCostConfigError::kInvalidDt);
}

TEST_F(PhaseCostConfigTest, PostureRefDimMismatch) {
  auto cfg = YAML::Load(kValidYaml);
  cfg["q_posture_ref"] = std::vector<double>(5, 0.0); // nq=9 expected

  rtc::mpc::PhaseCostConfig out;
  EXPECT_EQ(rtc::mpc::PhaseCostConfig::LoadFromYaml(cfg, handler_, out),
            rtc::mpc::PhaseCostConfigError::kPostureRefDimMismatch);
}

TEST_F(PhaseCostConfigTest, ForceTargetDimMismatch) {
  auto cfg = YAML::Load(kValidYaml);
  cfg["F_target"] = std::vector<double>(4, 0.0); // 6 expected (2×3)

  rtc::mpc::PhaseCostConfig out;
  EXPECT_EQ(rtc::mpc::PhaseCostConfig::LoadFromYaml(cfg, handler_, out),
            rtc::mpc::PhaseCostConfigError::kForceTargetDimMismatch);
}

TEST_F(PhaseCostConfigTest, PlacementWeightDimMismatch) {
  auto cfg = YAML::Load(kValidYaml);
  cfg["W_placement"] = std::vector<double>(5, 1.0); // 6 required

  rtc::mpc::PhaseCostConfig out;
  EXPECT_EQ(rtc::mpc::PhaseCostConfig::LoadFromYaml(cfg, handler_, out),
            rtc::mpc::PhaseCostConfigError::kPlacementWeightDimMismatch);
}

TEST(PhaseCostConfigStandaloneTest, UninitialisedModelRejected) {
  rtc::mpc::RobotModelHandler uninit;
  auto cfg = YAML::Load("horizon_length: 1\n");
  rtc::mpc::PhaseCostConfig out;

  EXPECT_EQ(rtc::mpc::PhaseCostConfig::LoadFromYaml(cfg, uninit, out),
            rtc::mpc::PhaseCostConfigError::kModelNotInitialised);
}

} // namespace
