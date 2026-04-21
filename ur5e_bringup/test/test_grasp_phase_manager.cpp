/// @file test_grasp_phase_manager.cpp
/// @brief Unit tests for the Phase 7a GraspPhaseManager FSM.
///
/// Panda 9-DoF + 2 fingertips is used as a generic fixture (same convention
/// as rtc_mpc/test — keeps the tests robot-agnostic and avoids pulling in
/// the ur5e_description + hand URDF macros). The FSM logic itself is
/// independent of the robot: edges fire on TCP distances + sensor sums +
/// external commands, never on joint counts or frame names.

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <string>

#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/phase/phase_context.hpp"
#include "ur5e_bringup/phase/grasp_phase_manager.hpp"
#include "ur5e_bringup/phase/grasp_target.hpp"

namespace {

constexpr const char *kPandaUrdf =
    "/usr/local/share/example-robot-data/robots/panda_description/urdf/"
    "panda.urdf";

using ur5e_bringup::phase::GraspCommand;
using ur5e_bringup::phase::GraspPhaseId;
using ur5e_bringup::phase::GraspPhaseManager;
using ur5e_bringup::phase::GraspTarget;

// Full phase_config.yaml-equivalent schema sized for Panda (nq=9, 2 × 3-dim
// contacts = 6-entry F_target).
constexpr const char *kPandaPhaseConfig = R"(
transition:
  approach_tolerance: 0.05
  pregrasp_tolerance: 0.01
  force_threshold: 0.5
  max_failures: 3

phases:
  idle:
    ocp_type: "light_contact"
    active_contact_indices: []
    cost: &cost_light
      horizon_length: 20
      dt: 0.01
      w_frame_placement: 100.0
      w_state_reg: 1.0
      w_control_reg: 0.01
      w_contact_force: 0.0
      w_centroidal_momentum: 0.0
      W_placement: [100.0, 100.0, 100.0, 10.0, 10.0, 10.0]
      q_posture_ref: [0, 0, 0, -1.57, 0, 1.57, 0.785, 0.02, 0.02]
      F_target: [0, 0, 0, 0, 0, 0]

  approach:
    ocp_type: "light_contact"
    active_contact_indices: []
    cost: *cost_light

  pre_grasp:
    ocp_type: "light_contact"
    active_contact_indices: []
    cost: *cost_light

  closure:
    ocp_type: "contact_rich"
    active_contact_indices: [0, 1]
    cost: &cost_rich
      horizon_length: 20
      dt: 0.01
      w_frame_placement: 50.0
      w_state_reg: 0.1
      w_control_reg: 0.01
      w_contact_force: 10.0
      w_centroidal_momentum: 0.0
      W_placement: [500.0, 500.0, 500.0, 50.0, 50.0, 50.0]
      q_posture_ref: [0, 0, 0, -1.57, 0, 1.57, 0.785, 0.02, 0.02]
      F_target: [0, 0, 2, 0, 0, 2]

  hold:
    ocp_type: "contact_rich"
    active_contact_indices: [0, 1]
    cost: *cost_rich

  manipulate:
    ocp_type: "contact_rich"
    active_contact_indices: [0, 1]
    cost: *cost_rich

  retreat:
    ocp_type: "light_contact"
    active_contact_indices: []
    cost: *cost_light

  release:
    ocp_type: "light_contact"
    active_contact_indices: []
    cost: *cost_light
)";

class GraspPhaseManagerTest : public ::testing::Test {
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

    manager_ = std::make_unique<GraspPhaseManager>(handler_);
  }

  // Convenience: pose at translation p, identity rotation.
  static pinocchio::SE3 PoseAt(const Eigen::Vector3d &p) {
    return pinocchio::SE3(Eigen::Matrix3d::Identity(), p);
  }

  // Advance one FSM tick with all-zero sensor + given TCP.
  rtc::mpc::PhaseContext
  Step(const pinocchio::SE3 &tcp,
       const Eigen::VectorXd &sensor = Eigen::VectorXd::Zero(2)) {
    return manager_->Update(q_, v_, sensor, tcp, /*t=*/0.0);
  }

  GraspTarget MakeTarget(const Eigen::Vector3d &grasp,
                         const Eigen::Vector3d &pregrasp,
                         const Eigen::Vector3d &start) {
    GraspTarget t;
    t.grasp_pose = PoseAt(grasp);
    t.pregrasp_pose = PoseAt(pregrasp);
    t.approach_start = PoseAt(start);
    return t;
  }

  pinocchio::Model model_{};
  rtc::mpc::RobotModelHandler handler_{};
  std::unique_ptr<GraspPhaseManager> manager_{};
  Eigen::VectorXd q_ = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd v_ = Eigen::VectorXd::Zero(9);
};

// ── Init path ───────────────────────────────────────────────────────────────

TEST_F(GraspPhaseManagerTest, LoadValidYamlSucceeds) {
  const auto cfg = YAML::Load(kPandaPhaseConfig);
  EXPECT_EQ(manager_->Load(cfg),
            ur5e_bringup::phase::GraspPhaseInitError::kNoError);
  EXPECT_TRUE(manager_->Initialised());
  EXPECT_EQ(manager_->CurrentPhaseId(), static_cast<int>(GraspPhaseId::kIdle));
  EXPECT_EQ(manager_->CurrentPhaseName(), "idle");
}

TEST_F(GraspPhaseManagerTest, LoadRejectsMissingPhase) {
  auto cfg = YAML::Load(kPandaPhaseConfig);
  cfg["phases"].remove("closure");
  EXPECT_EQ(manager_->Load(cfg),
            ur5e_bringup::phase::GraspPhaseInitError::kMissingPhase);
  EXPECT_FALSE(manager_->Initialised());
}

TEST_F(GraspPhaseManagerTest, LoadRejectsUnknownOcpType) {
  auto cfg = YAML::Load(kPandaPhaseConfig);
  cfg["phases"]["closure"]["ocp_type"] = "holographic";
  EXPECT_EQ(manager_->Load(cfg),
            ur5e_bringup::phase::GraspPhaseInitError::kInvalidOcpType);
}

TEST_F(GraspPhaseManagerTest, LoadRejectsNonPositiveThreshold) {
  auto cfg = YAML::Load(kPandaPhaseConfig);
  cfg["transition"]["approach_tolerance"] = 0.0;
  EXPECT_EQ(manager_->Load(cfg),
            ur5e_bringup::phase::GraspPhaseInitError::kInvalidThreshold);
}

TEST_F(GraspPhaseManagerTest, LoadRejectsOutOfRangeContactIndex) {
  auto cfg = YAML::Load(kPandaPhaseConfig);
  cfg["phases"]["closure"]["active_contact_indices"].push_back(7);
  EXPECT_EQ(manager_->Load(cfg),
            ur5e_bringup::phase::GraspPhaseInitError::kInvalidContactIndex);
}

TEST_F(GraspPhaseManagerTest, InitThrowsOnMalformedYaml) {
  EXPECT_THROW(manager_->Init(YAML::Load("[1, 2, 3]")), std::runtime_error);
}

// ── Happy-path traversal ────────────────────────────────────────────────────

TEST_F(GraspPhaseManagerTest, FullHappyPathTraversal) {
  ASSERT_EQ(manager_->Load(YAML::Load(kPandaPhaseConfig)),
            ur5e_bringup::phase::GraspPhaseInitError::kNoError);

  // Set up poses: start at origin, pregrasp at (0.1, 0, 0), grasp at
  // (0.15, 0, 0), retreat back to start.
  const Eigen::Vector3d start{0.0, 0.0, 0.0};
  const Eigen::Vector3d pregrasp{0.1, 0.0, 0.0};
  const Eigen::Vector3d grasp{0.15, 0.0, 0.0};
  manager_->SetTaskTarget(MakeTarget(grasp, pregrasp, start));
  ASSERT_TRUE(manager_->HasTarget());

  // IDLE → APPROACH on kApproach command.
  manager_->SetCommand(GraspCommand::kApproach);
  {
    auto ctx = Step(PoseAt(start));
    EXPECT_EQ(ctx.phase_id, static_cast<int>(GraspPhaseId::kApproach));
    EXPECT_TRUE(ctx.phase_changed);
    EXPECT_EQ(ctx.ocp_type, "light_contact");
  }

  // APPROACH → PRE_GRASP when TCP close to pregrasp (< 0.05).
  {
    auto ctx = Step(PoseAt(Eigen::Vector3d{0.07, 0.0, 0.0}));
    EXPECT_EQ(ctx.phase_id, static_cast<int>(GraspPhaseId::kPreGrasp));
    EXPECT_TRUE(ctx.phase_changed);
    EXPECT_EQ(ctx.ocp_type, "light_contact");
  }

  // PRE_GRASP → CLOSURE when TCP close to grasp (< 0.01) — cross-mode swap!
  {
    auto ctx = Step(PoseAt(Eigen::Vector3d{0.145, 0.0, 0.0}));
    EXPECT_EQ(ctx.phase_id, static_cast<int>(GraspPhaseId::kClosure));
    EXPECT_TRUE(ctx.phase_changed);
    EXPECT_EQ(ctx.ocp_type, "contact_rich");
    EXPECT_FALSE(ctx.contact_plan.phases.empty());
    EXPECT_EQ(ctx.contact_plan.phases.front().active_frame_ids.size(), 2u);
  }

  // CLOSURE → HOLD when Σ sensor > 0.5.
  {
    Eigen::VectorXd sensor(2);
    sensor << 0.4, 0.3; // sum = 0.7 > 0.5
    auto ctx = Step(PoseAt(Eigen::Vector3d{0.15, 0.0, 0.0}), sensor);
    EXPECT_EQ(ctx.phase_id, static_cast<int>(GraspPhaseId::kHold));
    EXPECT_TRUE(ctx.phase_changed);
    EXPECT_EQ(ctx.ocp_type, "contact_rich");
  }

  // HOLD → MANIPULATE on kManipulate.
  manager_->SetCommand(GraspCommand::kManipulate);
  {
    auto ctx = Step(PoseAt(Eigen::Vector3d{0.15, 0.0, 0.0}));
    EXPECT_EQ(ctx.phase_id, static_cast<int>(GraspPhaseId::kManipulate));
    EXPECT_TRUE(ctx.phase_changed);
  }

  // MANIPULATE → RETREAT on kRetreat.
  manager_->SetCommand(GraspCommand::kRetreat);
  {
    auto ctx = Step(PoseAt(Eigen::Vector3d{0.15, 0.0, 0.0}));
    EXPECT_EQ(ctx.phase_id, static_cast<int>(GraspPhaseId::kRetreat));
    EXPECT_TRUE(ctx.phase_changed);
    EXPECT_EQ(ctx.ocp_type, "light_contact");
  }

  // RETREAT → RELEASE when back near approach_start.
  // (approach_start was snapshotted to TCP at IDLE→APPROACH, i.e. pose
  // `start`.)
  {
    auto ctx = Step(PoseAt(Eigen::Vector3d{0.02, 0.0, 0.0}));
    EXPECT_EQ(ctx.phase_id, static_cast<int>(GraspPhaseId::kRelease));
    EXPECT_TRUE(ctx.phase_changed);
  }

  // RELEASE → IDLE on kRelease.
  manager_->SetCommand(GraspCommand::kRelease);
  {
    auto ctx = Step(PoseAt(start));
    EXPECT_EQ(ctx.phase_id, static_cast<int>(GraspPhaseId::kIdle));
    EXPECT_TRUE(ctx.phase_changed);
  }
}

// ── Failure guard (max_failures) ────────────────────────────────────────────

TEST_F(GraspPhaseManagerTest, ClosureFailureCountAbortsToIdle) {
  ASSERT_EQ(manager_->Load(YAML::Load(kPandaPhaseConfig)),
            ur5e_bringup::phase::GraspPhaseInitError::kNoError);

  manager_->SetTaskTarget(MakeTarget({0.1, 0, 0}, {0.1, 0, 0}, {0.0, 0, 0}));
  manager_->ForcePhase(static_cast<int>(GraspPhaseId::kClosure));
  // Consume the force edge.
  {
    auto ctx = Step(PoseAt({0.1, 0, 0}));
    EXPECT_EQ(ctx.phase_id, static_cast<int>(GraspPhaseId::kClosure));
  }

  // max_failures = 3 → after 3 zero-force ticks, revert to IDLE.
  for (int i = 0; i < 2; ++i) {
    auto ctx = Step(PoseAt({0.1, 0, 0}));
    EXPECT_EQ(ctx.phase_id, static_cast<int>(GraspPhaseId::kClosure));
    EXPECT_FALSE(ctx.phase_changed);
  }
  auto ctx = Step(PoseAt({0.1, 0, 0}));
  EXPECT_EQ(ctx.phase_id, static_cast<int>(GraspPhaseId::kIdle));
  EXPECT_TRUE(ctx.phase_changed);
  EXPECT_EQ(manager_->FailureCount(), 0);
}

// ── Abort ───────────────────────────────────────────────────────────────────

TEST_F(GraspPhaseManagerTest, AbortFromAnyPhaseReturnsToIdle) {
  ASSERT_EQ(manager_->Load(YAML::Load(kPandaPhaseConfig)),
            ur5e_bringup::phase::GraspPhaseInitError::kNoError);

  manager_->ForcePhase(static_cast<int>(GraspPhaseId::kHold));
  (void)Step(PoseAt({0.0, 0, 0}));

  manager_->SetCommand(GraspCommand::kAbort);
  auto ctx = Step(PoseAt({0.0, 0, 0}));
  EXPECT_EQ(ctx.phase_id, static_cast<int>(GraspPhaseId::kIdle));
  EXPECT_TRUE(ctx.phase_changed);
}

// ── ForcePhase ──────────────────────────────────────────────────────────────

TEST_F(GraspPhaseManagerTest, ForcePhaseBypassesGuards) {
  ASSERT_EQ(manager_->Load(YAML::Load(kPandaPhaseConfig)),
            ur5e_bringup::phase::GraspPhaseInitError::kNoError);

  // Jump from IDLE straight to MANIPULATE without a target or TCP proximity.
  manager_->ForcePhase(static_cast<int>(GraspPhaseId::kManipulate));
  auto ctx = Step(PoseAt({0.0, 0, 0}));
  EXPECT_EQ(ctx.phase_id, static_cast<int>(GraspPhaseId::kManipulate));
  EXPECT_TRUE(ctx.phase_changed);

  // Invalid ids are silently ignored.
  manager_->ForcePhase(999);
  ctx = Step(PoseAt({0.0, 0, 0}));
  EXPECT_EQ(ctx.phase_id, static_cast<int>(GraspPhaseId::kManipulate));
  EXPECT_FALSE(ctx.phase_changed);
}

// ── PhaseContext contents ───────────────────────────────────────────────────

TEST_F(GraspPhaseManagerTest, PhaseContextCarriesCostAndTarget) {
  ASSERT_EQ(manager_->Load(YAML::Load(kPandaPhaseConfig)),
            ur5e_bringup::phase::GraspPhaseInitError::kNoError);

  manager_->SetTaskTarget(MakeTarget({0.5, 0, 0}, {0.45, 0, 0}, {0.0, 0, 0}));
  manager_->ForcePhase(static_cast<int>(GraspPhaseId::kHold));
  auto ctx = Step(PoseAt({0.5, 0, 0}));

  EXPECT_EQ(ctx.phase_id, static_cast<int>(GraspPhaseId::kHold));
  EXPECT_EQ(ctx.ocp_type, "contact_rich");
  EXPECT_EQ(ctx.cost_config.horizon_length, 20);
  EXPECT_DOUBLE_EQ(ctx.cost_config.w_contact_force, 10.0);
  EXPECT_EQ(ctx.cost_config.q_posture_ref.size(), handler_.nq());
  EXPECT_DOUBLE_EQ(ctx.ee_target.translation().x(), 0.5);
}

TEST_F(GraspPhaseManagerTest, YamlTargetSetterBuildsPregraspFromOffset) {
  ASSERT_EQ(manager_->Load(YAML::Load(kPandaPhaseConfig)),
            ur5e_bringup::phase::GraspPhaseInitError::kNoError);

  manager_->SetTaskTarget(YAML::Load(R"(
grasp_translation: [0.3, 0.0, 0.2]
pregrasp_offset_local: [0.0, 0.0, 0.1]
)"));
  EXPECT_TRUE(manager_->HasTarget());

  // Advance into APPROACH to read ctx.ee_target = pregrasp_pose.
  manager_->SetCommand(GraspCommand::kApproach);
  auto ctx = Step(PoseAt({0.0, 0, 0}));
  ASSERT_EQ(ctx.phase_id, static_cast<int>(GraspPhaseId::kApproach));
  // Identity rotation: pregrasp = grasp + local_offset → (0.3, 0, 0.3).
  EXPECT_NEAR(ctx.ee_target.translation().z(), 0.3, 1e-9);
}

TEST_F(GraspPhaseManagerTest, UninitialisedUpdateYieldsSafeIdleContext) {
  auto ctx = Step(PoseAt({0, 0, 0}));
  EXPECT_EQ(ctx.phase_id, static_cast<int>(GraspPhaseId::kIdle));
  EXPECT_FALSE(ctx.phase_changed);
  EXPECT_EQ(ctx.ocp_type, "light_contact");
}

} // namespace
