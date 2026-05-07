/// @file test_robot_model_handler.cpp
/// @brief Unit tests for @ref rtc::mpc::RobotModelHandler.
///
/// Panda is used as a generic N-DoF fixed-base manipulator fixture (the only
/// URDF guaranteed to be present alongside Aligator / example-robot-data).
/// This file must NOT test any specific deployment robot or robot-specific
/// assumption — that belongs in `integrated_bringup/test/` (Phase 7).

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_mpc/model/robot_model_handler.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>

namespace {

constexpr const char* kPandaUrdf = RTC_PANDA_URDF_PATH;

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
  EXPECT_EQ(handler.nq(), 9);  // Panda: 7 arm + 2 finger prismatic
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
  EXPECT_EQ(handler.Init(model_, cfg), rtc::mpc::RobotModelInitError::kMissingEndEffectorFrame);
  EXPECT_FALSE(handler.Initialised());  // atomic: no partial state
}

TEST_F(RobotModelHandlerTest, MissingContactFrameReturnsError) {
  auto cfg = YAML::Load(R"(
end_effector_frame: panda_hand_tcp
contact_frames:
  - name: panda_leftfinger
  - name: ghost_frame
)");

  rtc::mpc::RobotModelHandler handler;
  EXPECT_EQ(handler.Init(model_, cfg), rtc::mpc::RobotModelInitError::kMissingContactFrame);
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
  EXPECT_EQ(handler.Init(model_, cfg), rtc::mpc::RobotModelInitError::kInvalidContactDim);
}

TEST_F(RobotModelHandlerTest, MissingEndEffectorKeyReturnsSchemaError) {
  auto cfg = YAML::Load("contact_frames: []\n");

  rtc::mpc::RobotModelHandler handler;
  EXPECT_EQ(handler.Init(model_, cfg), rtc::mpc::RobotModelInitError::kInvalidYamlSchema);
}

TEST_F(RobotModelHandlerTest, DoubleInitReturnsError) {
  auto cfg = YAML::Load("end_effector_frame: panda_hand_tcp\n");
  rtc::mpc::RobotModelHandler handler;
  ASSERT_EQ(handler.Init(model_, cfg), rtc::mpc::RobotModelInitError::kNoError);
  EXPECT_EQ(handler.Init(model_, cfg), rtc::mpc::RobotModelInitError::kModelAlreadyInitialised);
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

TEST_F(RobotModelHandlerTest, BaseFrameMissingFallsBackToUniverse) {
  auto cfg = YAML::Load("end_effector_frame: panda_hand_tcp\n");
  rtc::mpc::RobotModelHandler handler;
  ASSERT_EQ(handler.Init(model_, cfg), rtc::mpc::RobotModelInitError::kNoError);
  EXPECT_TRUE(handler.base_frame_is_universe());
  EXPECT_EQ(handler.base_frame_id(), 0);
}

TEST_F(RobotModelHandlerTest, BaseFrameExplicitResolvesId) {
  auto cfg = YAML::Load(R"(
end_effector_frame: panda_hand_tcp
base_frame: panda_link0
)");
  rtc::mpc::RobotModelHandler handler;
  ASSERT_EQ(handler.Init(model_, cfg), rtc::mpc::RobotModelInitError::kNoError);
  EXPECT_FALSE(handler.base_frame_is_universe());
  EXPECT_GT(handler.base_frame_id(), 0);
}

TEST_F(RobotModelHandlerTest, MissingBaseFrameReturnsError) {
  auto cfg = YAML::Load(R"(
end_effector_frame: panda_hand_tcp
base_frame: not_a_real_frame
)");
  rtc::mpc::RobotModelHandler handler;
  EXPECT_EQ(handler.Init(model_, cfg), rtc::mpc::RobotModelInitError::kMissingBaseFrame);
}

TEST_F(RobotModelHandlerTest, BaseOMfIdentityWhenUniverse) {
  auto cfg = YAML::Load("end_effector_frame: panda_hand_tcp\n");
  rtc::mpc::RobotModelHandler handler;
  ASSERT_EQ(handler.Init(model_, cfg), rtc::mpc::RobotModelInitError::kNoError);
  // universe fast path: oMb == Identity (no FK done).
  EXPECT_TRUE(handler.base_oMf().translation().isZero());
  EXPECT_TRUE(handler.base_oMf().rotation().isIdentity());
}

TEST_F(RobotModelHandlerTest, BaseOMfMatchesNeutralFK) {
  // panda_link0 is the URDF root frame attached to universe by the implicit
  // pinocchio joint — its placement at neutral q must equal Identity (the
  // robot is mounted at world origin).
  auto cfg = YAML::Load(R"(
end_effector_frame: panda_hand_tcp
base_frame: panda_link0
)");
  rtc::mpc::RobotModelHandler handler;
  ASSERT_EQ(handler.Init(model_, cfg), rtc::mpc::RobotModelInitError::kNoError);
  EXPECT_FALSE(handler.base_frame_is_universe());
  // Even though Panda's link0 sits at world origin, base_oMf must come from
  // FK (not Identity by accident). Verify by recomputing FK independently.
  pinocchio::Data data(model_);
  const Eigen::VectorXd q0 = pinocchio::neutral(model_);
  pinocchio::forwardKinematics(model_, data, q0);
  pinocchio::updateFramePlacement(model_, data,
                                  static_cast<pinocchio::FrameIndex>(handler.base_frame_id()));
  const auto& expected = data.oMf[static_cast<std::size_t>(handler.base_frame_id())];
  EXPECT_TRUE(handler.base_oMf().translation().isApprox(expected.translation()));
  EXPECT_TRUE(handler.base_oMf().rotation().isApprox(expected.rotation()));
}

TEST_F(RobotModelHandlerTest, BaseOMfNonIdentityForMidChainFrame) {
  // Mid-chain link as a *fixed-from-q-perspective* base is unusual but it
  // exercises the non-Identity oMb path: panda_link4 has a non-zero
  // placement at neutral q from FK. (It is not q-independent — this test
  // documents that callers must use a true fixed base, but verifies the
  // numeric pipeline.)
  auto cfg = YAML::Load(R"(
end_effector_frame: panda_hand_tcp
base_frame: panda_link4
)");
  rtc::mpc::RobotModelHandler handler;
  ASSERT_EQ(handler.Init(model_, cfg), rtc::mpc::RobotModelInitError::kNoError);
  EXPECT_FALSE(handler.base_frame_is_universe());
  // panda_link4 at neutral q has non-zero translation (~0.3 m above base).
  EXPECT_GT(handler.base_oMf().translation().norm(), 0.1);
}

}  // namespace
