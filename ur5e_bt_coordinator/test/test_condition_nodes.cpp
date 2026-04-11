/// Unit tests for all condition nodes.

#include "test_helpers.hpp"
#include "ur5e_bt_coordinator/condition_nodes/check_shape_type.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_force_above.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_grasp_phase.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_grasped.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_object_detected.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_vision_target_ready.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <thread>

using namespace rtc_bt;
using namespace rtc_bt::test;

class ConditionNodeTest : public RosTestFixture {
protected:
  void SetUp() override
  {
    RosTestFixture::SetUp();
    factory_.registerNodeType<IsForceAbove>("IsForceAbove", bridge_);
    factory_.registerNodeType<IsGrasped>("IsGrasped", bridge_);
    factory_.registerNodeType<IsGraspPhase>("IsGraspPhase", bridge_);
    factory_.registerNodeType<IsObjectDetected>("IsObjectDetected", bridge_);
    factory_.registerNodeType<IsVisionTargetReady>("IsVisionTargetReady", bridge_);
    factory_.registerNodeType<CheckShapeType>("CheckShapeType", bridge_);
  }

  BT::Tree CreateTree(const std::string& xml)
  {
    const std::string full = R"(<root BTCPP_format="4"><BehaviorTree ID="T">)" +
                             xml + R"(</BehaviorTree></root>)";
    return factory_.createTreeFromText(full);
  }

  CachedGraspState MakeGraspState(int contacts, float max_force, bool detected,
                                   float threshold = 1.0f, int min_ft = 2)
  {
    CachedGraspState gs;
    gs.num_active_contacts = contacts;
    gs.max_force = max_force;
    gs.grasp_detected = detected;
    gs.force_threshold = threshold;
    gs.min_fingertips = min_ft;
    return gs;
  }

  CachedGraspState MakeDetailedGraspState(
      const std::vector<std::tuple<float, float, bool>>& fingertips)
  {
    CachedGraspState gs;
    gs.force_threshold = 1.0f;
    gs.min_fingertips = 2;
    for (const auto& [force, contact, valid] : fingertips) {
      CachedGraspState::Fingertip ft;
      ft.force_magnitude = force;
      ft.contact_flag = contact;
      ft.inference_valid = valid;
      gs.fingertips.push_back(ft);
    }
    return gs;
  }

  BT::BehaviorTreeFactory factory_;
};

// ══════════════════════════════════════════════════════════════════════════
// IsForceAbove
// ══════════════════════════════════════════════════════════════════════════

TEST_F(ConditionNodeTest, IsForceAbove_Success)
{
  auto gs = MakeGraspState(3, 5.0f, true, 1.5f, 2);
  PublishGraspState(gs);
  Spin();

  auto tree = CreateTree(
      R"(<IsForceAbove threshold_N="1.5" min_fingertips="2"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(ConditionNodeTest, IsForceAbove_Failure)
{
  auto gs = MakeGraspState(1, 0.5f, false, 1.5f, 2);
  PublishGraspState(gs);
  Spin();

  auto tree = CreateTree(
      R"(<IsForceAbove threshold_N="1.5" min_fingertips="2"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(ConditionNodeTest, IsForceAbove_CustomThresholdRecount)
{
  // Use different threshold than controller defaults → triggers manual recount
  auto gs = MakeDetailedGraspState({
      {3.0f, 0.9f, true},   // above 2.0N, valid contact
      {2.5f, 0.8f, true},   // above 2.0N, valid contact
      {0.1f, 0.1f, true},   // below threshold
  });
  PublishGraspState(gs);
  Spin();

  auto tree = CreateTree(
      R"(<IsForceAbove threshold_N="2.0" min_fingertips="2"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(ConditionNodeTest, IsForceAbove_SustainedNotMet)
{
  auto gs = MakeGraspState(3, 5.0f, true, 1.5f, 2);
  PublishGraspState(gs);
  Spin();

  auto tree = CreateTree(
      R"(<IsForceAbove threshold_N="1.5" min_fingertips="2"
                       sustained_ms="200"/>)");
  // First tick starts sustain timer → FAILURE (not enough time yet)
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(ConditionNodeTest, IsForceAbove_SustainedMet)
{
  auto gs = MakeGraspState(3, 5.0f, true, 1.5f, 2);
  PublishGraspState(gs);
  Spin();

  auto tree = CreateTree(
      R"(<IsForceAbove threshold_N="1.5" min_fingertips="2"
                       sustained_ms="20"/>)");

  // First tick: starts timer
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  // Second tick: sustained time exceeded
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

// ══════════════════════════════════════════════════════════════════════════
// IsGrasped
// ══════════════════════════════════════════════════════════════════════════

TEST_F(ConditionNodeTest, IsGrasped_Detected)
{
  auto gs = MakeGraspState(3, 5.0f, true);
  PublishGraspState(gs);
  Spin();

  auto tree = CreateTree(R"(<IsGrasped/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(ConditionNodeTest, IsGrasped_NotDetected)
{
  auto gs = MakeGraspState(0, 0.0f, false);
  PublishGraspState(gs);
  Spin();

  auto tree = CreateTree(R"(<IsGrasped/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(ConditionNodeTest, IsGrasped_CustomThresholdRecount)
{
  // Use threshold/min_fingertips that differ from controller defaults
  // to trigger manual recount path
  auto gs = MakeDetailedGraspState({
      {2.0f, 0.9f, true},
      {1.5f, 0.8f, true},
      {0.1f, 0.1f, true},
  });
  gs.grasp_detected = false;
  // Set controller defaults to something different so custom path is taken
  gs.force_threshold = 5.0f;   // different from our 1.0
  gs.min_fingertips = 3;       // different from our 2
  PublishGraspState(gs);
  Spin();

  // Custom threshold 1.0N with min 2: two fingertips qualify via manual recount
  auto tree = CreateTree(
      R"(<IsGrasped force_threshold_N="1.0" min_fingertips="2"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

// ══════════════════════════════════════════════════════════════════════════
// IsGraspPhase
// ══════════════════════════════════════════════════════════════════════════

TEST_F(ConditionNodeTest, IsGraspPhase_Matches)
{
  CachedGraspState gs;
  gs.grasp_phase = 4;  // "holding"
  PublishGraspState(gs);
  Spin();

  auto tree = CreateTree(R"(<IsGraspPhase phase="holding"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(ConditionNodeTest, IsGraspPhase_DoesNotMatch)
{
  CachedGraspState gs;
  gs.grasp_phase = 0;  // "idle"
  PublishGraspState(gs);
  Spin();

  auto tree = CreateTree(R"(<IsGraspPhase phase="holding"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(ConditionNodeTest, IsGraspPhase_AllPhases)
{
  const std::map<std::string, uint8_t> phases = {
      {"idle", 0}, {"approaching", 1}, {"contact", 2},
      {"force_control", 3}, {"holding", 4}, {"releasing", 5}};

  for (const auto& [name, val] : phases) {
    CachedGraspState gs;
    gs.grasp_phase = val;
    PublishGraspState(gs);
    Spin();

    auto tree = CreateTree(
        R"(<IsGraspPhase phase=")" + name + R"("/>)");
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS)
        << "Phase " << name << " (val=" << static_cast<int>(val) << ") should match";
  }
}

TEST_F(ConditionNodeTest, IsGraspPhase_InvalidPhase)
{
  CachedGraspState gs;
  gs.grasp_phase = 0;
  PublishGraspState(gs);
  Spin();

  auto tree = CreateTree(R"(<IsGraspPhase phase="invalid_phase"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

// ── min_duration_s (dwell timer) ────────────────────────────────────────────
//
// When ``min_duration_s > 0`` the node becomes stateful: the target phase
// must be continuously observed for at least that long before SUCCESS is
// returned. These tests exercise the new dwell path added to support the
// "force_control sustained for 3 s → success" condition used by the
// ``ForcePIGrasp`` subtree.

TEST_F(ConditionNodeTest, IsGraspPhase_DwellTimer_FailsBeforeMinDuration)
{
  CachedGraspState gs;
  gs.grasp_phase = 3;  // "force_control"
  PublishGraspState(gs);
  Spin();

  auto tree = CreateTree(
      R"(<IsGraspPhase phase="force_control" min_duration_s="0.5"/>)");

  // First tick starts the dwell timer → expect FAILURE (not yet elapsed).
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  // A second tick ~10 ms later is still far below the 500 ms threshold.
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(ConditionNodeTest, IsGraspPhase_DwellTimer_SucceedsAfterMinDuration)
{
  CachedGraspState gs;
  gs.grasp_phase = 3;  // "force_control"
  PublishGraspState(gs);
  Spin();

  auto tree = CreateTree(
      R"(<IsGraspPhase phase="force_control" min_duration_s="0.1"/>)");

  // Start the dwell timer.
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  // Wait longer than the 100 ms threshold, then re-tick.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(ConditionNodeTest, IsGraspPhase_DwellTimer_ResetsOnMismatch)
{
  // Use a 300 ms threshold to leave generous headroom above the ~50 ms
  // Spin() duration used below, so the test is not sensitive to scheduler
  // jitter on CI runners.
  constexpr auto kMinDurationMs = std::chrono::milliseconds(300);

  // Phase initially matches — dwell timer arms.
  CachedGraspState gs;
  gs.grasp_phase = 3;  // "force_control"
  PublishGraspState(gs);
  Spin();

  auto tree = CreateTree(
      R"(<IsGraspPhase phase="force_control" min_duration_s="0.3"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE)
      << "First tick should start the dwell timer and return FAILURE";

  // Phase transiently leaves the target → dwell timer must reset.
  gs.grasp_phase = 2;  // "contact"
  PublishGraspState(gs);
  Spin();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  // Phase returns to target — the dwell clock starts over from zero.
  gs.grasp_phase = 3;
  PublishGraspState(gs);
  Spin();
  const auto reentry_time = std::chrono::steady_clock::now();

  // Tick well before the 300 ms threshold has elapsed from the re-entry
  // moment → should still fail even though the tree has been alive longer.
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  // Wait until well past the threshold measured from the re-entry moment.
  std::this_thread::sleep_until(reentry_time + kMinDurationMs +
                                std::chrono::milliseconds(100));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(ConditionNodeTest, IsGraspPhase_DwellTimer_ZeroIsLegacyBehavior)
{
  // min_duration_s == 0.0 (the port default) must match the pre-existing
  // semantics: immediate SUCCESS on first match without any dwell.
  CachedGraspState gs;
  gs.grasp_phase = 4;  // "holding"
  PublishGraspState(gs);
  Spin();

  auto tree = CreateTree(
      R"(<IsGraspPhase phase="holding" min_duration_s="0.0"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

// ══════════════════════════════════════════════════════════════════════════
// IsObjectDetected
// ══════════════════════════════════════════════════════════════════════════

TEST_F(ConditionNodeTest, IsObjectDetected_WithObject)
{
  // Set TCP orientation so it can be merged
  Pose6D tcp{0.5, 0.5, 0.5, 1.0, 2.0, 3.0};
  PublishArmState(tcp, {0, 0, 0, 0, 0, 0});
  PublishWorldTarget(0.3, -0.2, 0.1);
  Spin();

  auto tree = CreateTree(R"(<IsObjectDetected pose="{p}"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto p = tree.rootBlackboard()->get<Pose6D>("p");
  EXPECT_NEAR(p.x, 0.3, 0.01);
  EXPECT_NEAR(p.y, -0.2, 0.01);
  // Orientation from TCP
  EXPECT_NEAR(p.roll, 1.0, 0.01);
}

TEST_F(ConditionNodeTest, IsObjectDetected_NoObject)
{
  auto tree = CreateTree(R"(<IsObjectDetected pose="{p}"/>)");
  Spin();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

// ══════════════════════════════════════════════════════════════════════════
// IsVisionTargetReady
// ══════════════════════════════════════════════════════════════════════════

TEST_F(ConditionNodeTest, IsVisionTargetReady_WithTarget)
{
  Pose6D tcp{0, 0, 0, 0.5, 1.0, 1.5};
  PublishArmState(tcp, {0, 0, 0, 0, 0, 0});
  PublishWorldTarget(0.4, -0.1, 0.2);
  Spin();

  auto tree = CreateTree(R"(<IsVisionTargetReady pose="{p}"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto p = tree.rootBlackboard()->get<Pose6D>("p");
  EXPECT_NEAR(p.x, 0.4, 0.01);
  EXPECT_NEAR(p.pitch, 1.0, 0.01);
}

TEST_F(ConditionNodeTest, IsVisionTargetReady_NoTarget)
{
  auto tree = CreateTree(R"(<IsVisionTargetReady pose="{p}"/>)");
  Spin();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(ConditionNodeTest, IsVisionTargetReady_ZeroCoordinatesInvalid)
{
  Pose6D tcp{0, 0, 0, 0, 0, 0};
  PublishArmState(tcp, {0, 0, 0, 0, 0, 0});
  PublishWorldTarget(0.0, 0.0, 0.0);  // All zeros → not valid
  Spin();

  auto tree = CreateTree(R"(<IsVisionTargetReady pose="{p}"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

// ══════════════════════════════════════════════════════════════════════════
// CheckShapeType
// ══════════════════════════════════════════════════════════════════════════

TEST_F(ConditionNodeTest, CheckShapeType_ExtractionOnly)
{
  PublishShapeEstimate(2, 0.95, 500);  // SPHERE
  Spin();

  auto tree = CreateTree(
      R"(<CheckShapeType estimate="{est}"
                         shape_type="{st}" shape_name="{sn}" confidence="{conf}"/>)");

  // Set the estimate on blackboard
  shape_estimation_msgs::msg::ShapeEstimate est;
  est.shape_type = 2;
  est.confidence = 0.95;
  est.num_points_used = 500;
  tree.rootBlackboard()->set("est", est);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto sn = tree.rootBlackboard()->get<std::string>("sn");
  EXPECT_EQ(sn, "sphere");

  auto st = tree.rootBlackboard()->get<unsigned>("st");
  EXPECT_EQ(st, 2u);

  auto conf = tree.rootBlackboard()->get<double>("conf");
  EXPECT_NEAR(conf, 0.95, 1e-6);
}

TEST_F(ConditionNodeTest, CheckShapeType_ExpectedMatch)
{
  auto tree = CreateTree(
      R"(<CheckShapeType estimate="{est}" expected_type="cylinder"
                         shape_type="{st}" shape_name="{sn}" confidence="{conf}"/>)");

  shape_estimation_msgs::msg::ShapeEstimate est;
  est.shape_type = 3;  // CYLINDER
  est.confidence = 0.8;
  tree.rootBlackboard()->set("est", est);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(ConditionNodeTest, CheckShapeType_ExpectedMismatch)
{
  auto tree = CreateTree(
      R"(<CheckShapeType estimate="{est}" expected_type="box"
                         shape_type="{st}" shape_name="{sn}" confidence="{conf}"/>)");

  shape_estimation_msgs::msg::ShapeEstimate est;
  est.shape_type = 2;  // SPHERE, not BOX
  est.confidence = 0.9;
  tree.rootBlackboard()->set("est", est);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(ConditionNodeTest, CheckShapeType_AllTypes)
{
  const std::map<uint8_t, std::string> types = {
      {0, "unknown"}, {1, "plane"}, {2, "sphere"}, {3, "cylinder"}, {4, "box"}};

  for (const auto& [val, name] : types) {
    auto tree = CreateTree(
        R"(<CheckShapeType estimate="{est}"
                           shape_type="{st}" shape_name="{sn}" confidence="{conf}"/>)");

    shape_estimation_msgs::msg::ShapeEstimate est;
    est.shape_type = val;
    est.confidence = 0.5;
    tree.rootBlackboard()->set("est", est);

    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(tree.rootBlackboard()->get<std::string>("sn"), name)
        << "type=" << static_cast<int>(val);
  }
}
