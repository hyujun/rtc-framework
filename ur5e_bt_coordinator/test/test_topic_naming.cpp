/// Unit tests for topic_namer.hpp: robot-agnostic topic-name factory (Seam A).
///
/// These tests are the regression anchor for the topic-naming contract:
///   - The default profile {ur5e, hand} must reproduce the legacy hardcoded
///     strings byte-for-byte (Sprint item 1).
///   - The p1b profile {ur5e, p1b} must swap only the hand group segment
///     (Sprint item 2).
/// Later seams migrate values/types; these strings must stay pinned so any
/// accidental drift in the coordinator's subscribe/publish paths is caught.

#include "ur5e_bt_coordinator/topic_namer.hpp"

#include <gtest/gtest.h>

using rtc_bt::TopicNamer;

namespace {
// A representative controller namespace (= "/" + active_controller_name).
constexpr const char* kCtrlNs = "/demo_task_controller";
}  // namespace

// ── Default profile == legacy hardcoded strings ───────────────────────────

TEST(TopicNamer, DefaultArmHandGroups) {
  TopicNamer t;  // {ur5e, hand}
  EXPECT_EQ(t.arm_group, "ur5e");
  EXPECT_EQ(t.hand_group, "hand");
}

TEST(TopicNamer, DefaultJointStatesMatchLegacy) {
  TopicNamer t;
  EXPECT_EQ(t.ArmJointStates(), "/rtc_cm/ur5e/joint_states");
  EXPECT_EQ(t.HandJointStates(), "/rtc_cm/hand/joint_states");
}

TEST(TopicNamer, DefaultControllerOwnedMatchLegacy) {
  TopicNamer t;
  EXPECT_EQ(t.GraspState(kCtrlNs), "/demo_task_controller/hand/grasp_state");
  EXPECT_EQ(t.WbcState(kCtrlNs), "/demo_task_controller/hand/wbc_state");
  EXPECT_EQ(t.ArmJointGoal(kCtrlNs), "/demo_task_controller/ur5e/joint_goal");
  EXPECT_EQ(t.HandJointGoal(kCtrlNs), "/demo_task_controller/hand/joint_goal");
  // Transforms is the whole-controller TFMessage stream — controller-owned but
  // NOT group-scoped (no arm/hand token), fed into tf_buffer_ for TCP lookup.
  EXPECT_EQ(t.Transforms(kCtrlNs), "/demo_task_controller/transforms");
}

TEST(TopicNamer, TransformsIsGroupAgnostic) {
  // Transforms carries no arm/hand token, so swapping the hand group (p1b) or
  // both groups (iiwa7/leap) must NOT change the path — only `ns` matters.
  EXPECT_EQ(TopicNamer{}.Transforms(kCtrlNs), "/demo_task_controller/transforms");
  EXPECT_EQ((TopicNamer{"ur5e", "p1b"}.Transforms(kCtrlNs)), "/demo_task_controller/transforms");
  EXPECT_EQ((TopicNamer{"iiwa7", "leap"}.Transforms(kCtrlNs)), "/demo_task_controller/transforms");
}

TEST(TopicNamer, EmptyNsGivesRelativeHealthLabel) {
  // Pre-rewire watchdog label form (before any controller activates) —
  // matches the legacy static "/hand/grasp_state" / "/hand/wbc_state".
  TopicNamer t;
  EXPECT_EQ(t.GraspState(""), "/hand/grasp_state");
  EXPECT_EQ(t.WbcState(""), "/hand/wbc_state");
}

// ── p1b profile swaps only the hand group ──────────────────────────────────

TEST(TopicNamer, P1bHandGroupNamespacing) {
  TopicNamer t{"ur5e", "p1b"};
  // Hand topics move to the p1b group...
  EXPECT_EQ(t.HandJointStates(), "/rtc_cm/p1b/joint_states");
  EXPECT_EQ(t.GraspState(kCtrlNs), "/demo_task_controller/p1b/grasp_state");
  EXPECT_EQ(t.WbcState(kCtrlNs), "/demo_task_controller/p1b/wbc_state");
  EXPECT_EQ(t.HandJointGoal(kCtrlNs), "/demo_task_controller/p1b/joint_goal");
  // ...while arm topics stay on ur5e (p1b variant keeps the UR5e arm).
  EXPECT_EQ(t.ArmJointStates(), "/rtc_cm/ur5e/joint_states");
  EXPECT_EQ(t.ArmJointGoal(kCtrlNs), "/demo_task_controller/ur5e/joint_goal");
}

TEST(TopicNamer, P1bRelativeHealthLabel) {
  TopicNamer t{"ur5e", "p1b"};
  EXPECT_EQ(t.GraspState(""), "/p1b/grasp_state");
  EXPECT_EQ(t.WbcState(""), "/p1b/wbc_state");
}

// ── Fully custom profile (iiwa7_leap dry-run — arm group swaps too) ────────

TEST(TopicNamer, CustomArmAndHandGroups) {
  TopicNamer t{"iiwa7", "leap"};
  EXPECT_EQ(t.ArmJointStates(), "/rtc_cm/iiwa7/joint_states");
  EXPECT_EQ(t.HandJointStates(), "/rtc_cm/leap/joint_states");
  EXPECT_EQ(t.ArmJointGoal(kCtrlNs), "/demo_task_controller/iiwa7/joint_goal");
  EXPECT_EQ(t.HandJointGoal(kCtrlNs), "/demo_task_controller/leap/joint_goal");
}
