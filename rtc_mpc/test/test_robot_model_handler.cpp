/// @file test_robot_model_handler.cpp
/// @brief Unit tests for @ref rtc::mpc::RobotModelHandler.
///
/// Panda is used as a generic N-DoF fixed-base manipulator fixture (the only
/// URDF guaranteed to be present alongside Aligator / example-robot-data).
/// This file must NOT test UR5e or any robot-specific assumption — that
/// belongs in `ur5e_bringup/test/` (Phase 7).

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

#include "rtc_mpc/model/robot_model_handler.hpp"

namespace {

constexpr const char *kPandaUrdf =
    RTC_PANDA_URDF_PATH;

class RobotModelHandlerTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!std::filesystem::exists(kPandaUrdf)) {
      GTEST_SKIP() << "Panda URDF not installed at " << kPandaUrdf
                   << " — run ./install.sh to install Aligator deps";
    }
    pinocchio::urdf::buildModel(kPandaUrdf, model_);
  }

  pinocchio::Model model_{};
};

TEST_F(RobotModelHandlerTest, InitSucceedsWithValidYaml) {
  auto cfg = YAML::Load(R"(
end_effector_frame: panda_hand_tcp
contact_frames:
  - name: panda_leftfinger
    dim: 3
  - name: panda_rightfinger
    dim: 3
)");

  rtc::mpc::RobotModelHandler handler;
  const auto err = handler.Init(model_, cfg);

  EXPECT_EQ(err, rtc::mpc::RobotModelInitError::kNoError);
  EXPECT_TRUE(handler.Initialised());
  EXPECT_EQ(handler.nq(), 9); // Panda: 7 arm + 2 finger prismatic
  EXPECT_EQ(handler.nv(), 9);
  EXPECT_EQ(handler.nu(), 9);
  EXPECT_EQ(handler.n_contacts(), 2);
  EXPECT_GE(handler.end_effector_frame_id(), 0);
  EXPECT_EQ(handler.contact_frames()[0].name, "panda_leftfinger");
  EXPECT_EQ(handler.contact_frames()[0].dim, 3);
}

TEST_F(RobotModelHandlerTest, InitSucceedsWithNoContacts) {
  auto cfg = YAML::Load(R"(
end_effector_frame: panda_hand_tcp
)");

  rtc::mpc::RobotModelHandler handler;
  EXPECT_EQ(handler.Init(model_, cfg), rtc::mpc::RobotModelInitError::kNoError);
  EXPECT_EQ(handler.n_contacts(), 0);
}

TEST_F(RobotModelHandlerTest, MissingEndEffectorFrameReturnsError) {
  auto cfg = YAML::Load(R"(
end_effector_frame: not_a_real_frame
)");

  rtc::mpc::RobotModelHandler handler;
  EXPECT_EQ(handler.Init(model_, cfg),
            rtc::mpc::RobotModelInitError::kMissingEndEffectorFrame);
  EXPECT_FALSE(handler.Initialised()); // atomic: no partial state
}

TEST_F(RobotModelHandlerTest, MissingContactFrameReturnsError) {
  auto cfg = YAML::Load(R"(
end_effector_frame: panda_hand_tcp
contact_frames:
  - name: panda_leftfinger
  - name: ghost_frame
)");

  rtc::mpc::RobotModelHandler handler;
  EXPECT_EQ(handler.Init(model_, cfg),
            rtc::mpc::RobotModelInitError::kMissingContactFrame);
  EXPECT_FALSE(handler.Initialised());
}

TEST_F(RobotModelHandlerTest, InvalidContactDimReturnsError) {
  auto cfg = YAML::Load(R"(
end_effector_frame: panda_hand_tcp
contact_frames:
  - name: panda_leftfinger
    dim: 5
)");

  rtc::mpc::RobotModelHandler handler;
  EXPECT_EQ(handler.Init(model_, cfg),
            rtc::mpc::RobotModelInitError::kInvalidContactDim);
}

TEST_F(RobotModelHandlerTest, MissingEndEffectorKeyReturnsSchemaError) {
  auto cfg = YAML::Load("contact_frames: []\n");

  rtc::mpc::RobotModelHandler handler;
  EXPECT_EQ(handler.Init(model_, cfg),
            rtc::mpc::RobotModelInitError::kInvalidYamlSchema);
}

TEST_F(RobotModelHandlerTest, DoubleInitReturnsError) {
  auto cfg = YAML::Load("end_effector_frame: panda_hand_tcp\n");
  rtc::mpc::RobotModelHandler handler;
  ASSERT_EQ(handler.Init(model_, cfg), rtc::mpc::RobotModelInitError::kNoError);
  EXPECT_EQ(handler.Init(model_, cfg),
            rtc::mpc::RobotModelInitError::kModelAlreadyInitialised);
}

TEST_F(RobotModelHandlerTest, FrameIdLookup) {
  auto cfg = YAML::Load("end_effector_frame: panda_hand_tcp\n");
  rtc::mpc::RobotModelHandler handler;
  ASSERT_EQ(handler.Init(model_, cfg), rtc::mpc::RobotModelInitError::kNoError);

  EXPECT_TRUE(handler.FrameId("panda_hand_tcp").has_value());
  EXPECT_FALSE(handler.FrameId("no_such_frame").has_value());
}

TEST(RobotModelHandlerStandaloneTest, UninitialisedAccessorsSafe) {
  rtc::mpc::RobotModelHandler handler;
  EXPECT_FALSE(handler.Initialised());
  EXPECT_EQ(handler.nq(), 0);
  EXPECT_EQ(handler.nv(), 0);
  EXPECT_EQ(handler.nu(), 0);
  EXPECT_EQ(handler.n_contacts(), 0);
  EXPECT_FALSE(handler.FrameId("anything").has_value());
}

} // namespace
