/// Task-frame alignment for the BT bridge (#292).
///
/// The bridge read a hardcoded `tool0_actual` while a task controller with
/// `virtual_tcp_mode` enabled controls a virtual TCP instead. That matters more
/// here than in the GUI: MoveToPose and TrackTrajectory judge *convergence*
/// with GetTcpPose(), so a frame mismatch is not a display bug but a goal that
/// can never be reached — and, in the rewire case below, one that reports
/// SUCCESS without the arm having moved.
///
/// Three distinct defects are pinned here:
///   1. the read frame follows the ACTIVE controller (availability-driven, no
///      controller name anywhere)
///   2. a rewire drops the tf cache, so the previous controller's frames stop
///      resolving through tf2's 10 s transform cache
///   3. GetTcpPose() no longer passes off a stale cache as live — the bridge's
///      own version of the GUI defect, made worse by the convergence consumers
///
/// DDS-free: everything travels the production handler seam
/// (BridgeStateInjector → On*), same as the other Tier-2 inject tests.

#include "inject_fixture.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace {

using rtc_bt::Pose6D;
using rtc_bt::test::InjectTestFixture;

const std::vector<double> kJoints{0.0, -1.57, 1.57, -1.57, -1.57, 0.0};

class TaskFrameTest : public InjectTestFixture {};

// ── 1. Frame selection follows the active controller ────────────────────────

TEST_F(TaskFrameTest, VirtualTcpBroadcastSelectsTheVirtualTcpFrame) {
  // The controller broadcasts virtual_tcp_actual only on ticks it is actually
  // controlling that point (integrated_bringup owned_topics.cpp gates the slot
  // on virtual_tcp_pose_valid), so one sighting is conclusive evidence.
  injector_->ActiveController("demo_task_controller");
  PublishVirtualTcpState(Pose6D{0.3, 0.1, 0.5, 0.0, 0.0, 0.0}, kJoints);

  EXPECT_EQ(bridge_->GetTaskFrame(), "virtual_tcp_actual");
  EXPECT_TRUE(bridge_->IsTcpPoseValid());
  const auto pose = bridge_->GetTcpPose();
  EXPECT_NEAR(pose.x, 0.3, 1e-6);
  EXPECT_NEAR(pose.y, 0.1, 1e-6);
  EXPECT_NEAR(pose.z, 0.5, 1e-6);
}

TEST_F(TaskFrameTest, Tool0OnlyBroadcastSettlesOnTheFallbackFrame) {
  // A controller that never publishes a virtual TCP — WBC, or task with the
  // vtcp disabled — settles on tool0 without any controller name being tested.
  injector_->ActiveController("demo_joint_controller");
  PublishArmState(Pose6D{0.4, 0.0, 0.6, 0.0, 0.0, 0.0}, kJoints);

  EXPECT_EQ(bridge_->GetTaskFrame(), "tool0_actual");
  EXPECT_TRUE(bridge_->IsTcpPoseValid());
  EXPECT_NEAR(bridge_->GetTcpPose().x, 0.4, 1e-6);
}

TEST_F(TaskFrameTest, FallbackDoesNotLatchOnTheFirstMessage) {
  // The settle window is the whole point: during a closed-chain hand's FK
  // walk-in a vtcp-configured controller publishes tool0 ALONE. Latching on the
  // first message picks the wrong frame in exactly the case this exists for,
  // and the wrong frame then persists until the next rewire.
  injector_->ActiveController("demo_task_controller");
  injector_->Transforms(MakeTaskTf(Pose6D{0.4, 0.0, 0.6, 0.0, 0.0, 0.0}, "tool0_actual"));

  EXPECT_TRUE(bridge_->GetTaskFrame().empty()) << "tool0 latched on the first message";
  EXPECT_FALSE(bridge_->IsTcpPoseValid());
}

TEST_F(TaskFrameTest, VirtualTcpArrivingInsideTheWindowWins) {
  // The walk-in, played out: tool0 for a while, then the virtual TCP appears.
  injector_->ActiveController("demo_task_controller");
  const auto tool0_tf = MakeTaskTf(Pose6D{0.4, 0.0, 0.6, 0.0, 0.0, 0.0}, "tool0_actual");
  for (int i = 0; i < 30; ++i) {
    injector_->Transforms(tool0_tf);
  }
  ASSERT_TRUE(bridge_->GetTaskFrame().empty());

  injector_->Transforms(MakeTaskTf(Pose6D{0.3, 0.1, 0.5, 0.0, 0.0, 0.0}, "virtual_tcp_actual"));
  EXPECT_EQ(bridge_->GetTaskFrame(), "virtual_tcp_actual");
  EXPECT_NEAR(bridge_->GetTcpPose().x, 0.3, 1e-6);
}

TEST_F(TaskFrameTest, SelectionIsFrozenOnceLatched) {
  // A control point that drops out for a tick must not move what convergence is
  // measured against.
  injector_->ActiveController("demo_task_controller");
  PublishVirtualTcpState(Pose6D{0.3, 0.1, 0.5, 0.0, 0.0, 0.0}, kJoints);
  ASSERT_EQ(bridge_->GetTaskFrame(), "virtual_tcp_actual");

  const auto tool0_tf = MakeTaskTf(Pose6D{0.4, 0.0, 0.6, 0.0, 0.0, 0.0}, "tool0_actual");
  for (int i = 0; i < 300; ++i) {
    injector_->Transforms(tool0_tf);
  }
  EXPECT_EQ(bridge_->GetTaskFrame(), "virtual_tcp_actual");
}

// ── 2/3. A rewire drops the previous controller's frames AND its pose ───────

TEST_F(TaskFrameTest, RewireInvalidatesTheFrameAndTheCachedPose) {
  injector_->ActiveController("demo_task_controller");
  PublishVirtualTcpState(Pose6D{0.3, 0.1, 0.5, 0.0, 0.0, 0.0}, kJoints);
  ASSERT_EQ(bridge_->GetTaskFrame(), "virtual_tcp_actual");
  ASSERT_TRUE(bridge_->IsTcpPoseValid());

  injector_->ActiveController("demo_joint_controller");

  // The frame selection restarts — nothing the previous controller broadcast is
  // evidence about what the new one controls.
  EXPECT_TRUE(bridge_->GetTaskFrame().empty());
  EXPECT_FALSE(bridge_->IsTcpPoseValid());
}

TEST_F(TaskFrameTest, RewireDoesNotServeThePreviousControllersPoseAsLive) {
  // The sharp one, and the reason the tf buffer is replaced rather than just
  // re-subscribed: tf2 caches transforms for 10 s by default, so without a new
  // buffer the previous controller's virtual_tcp_actual keeps resolving through
  // lookupTransform after its publisher is gone. GetTcpPose would then hand a
  // pose nobody is holding to MoveToPose's convergence test, which can report
  // SUCCESS for a motion that never happened.
  injector_->ActiveController("demo_task_controller");
  PublishVirtualTcpState(Pose6D{0.3, 0.1, 0.5, 0.0, 0.0, 0.0}, kJoints);
  ASSERT_TRUE(bridge_->IsTcpPoseValid());

  injector_->ActiveController("demo_joint_controller");

  // GetTcpPose still returns a last-known value for callers that want one...
  (void)bridge_->GetTcpPose();
  // ...but it must not claim to be live, and no lookup may resolve: the new
  // controller has broadcast nothing, so even re-settling on tool0 finds an
  // empty buffer rather than the previous controller's cached frames.
  EXPECT_FALSE(bridge_->IsTcpPoseValid());

  const auto tool0_tf = MakeTaskTf(Pose6D{0.9, 0.9, 0.9, 0.0, 0.0, 0.0}, "tool0_actual");
  for (int i = 0; i < 300; ++i) {
    injector_->Transforms(tool0_tf);
  }
  ASSERT_EQ(bridge_->GetTaskFrame(), "tool0_actual");
  const auto pose = bridge_->GetTcpPose();
  EXPECT_TRUE(bridge_->IsTcpPoseValid());
  EXPECT_NEAR(pose.x, 0.9, 1e-6) << "served a pose from the previous controller";
}

TEST_F(TaskFrameTest, PoseIsNotLiveBeforeAnyControllerIsWired) {
  // Nothing broadcast yet: there is no frame and therefore no pose. Reporting
  // one here would be reporting the zero-initialised cache as a measurement.
  EXPECT_TRUE(bridge_->GetTaskFrame().empty());
  EXPECT_FALSE(bridge_->IsTcpPoseValid());
  (void)bridge_->GetTcpPose();
  EXPECT_FALSE(bridge_->IsTcpPoseValid());
}

}  // namespace
