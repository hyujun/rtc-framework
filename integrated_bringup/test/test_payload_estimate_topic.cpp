// ── #135 D12: the PayloadEstimate wire surface ───────────────────────────────
//
// Layer 1b deliberately shipped the observer with a CSV lane and no topic, on
// the grounds that the fields a `PayloadEstimate` message exists to carry did
// not exist yet. Layer 2A produced them; this file covers the topic that
// arrived with them.
//
// What is actually at risk here is not arithmetic — the estimator's own tests
// own that — but TRANSCRIPTION. The publish path copies a flat POD into a
// structured message, and three of those copies are silently reversible:
//   - `wrench` is force-first in the POD and force/torque-named in the message,
//     so a swapped half produces a well-formed message that is wrong,
//   - `residual` is bounded by a width chosen at configure time, so an
//     off-by-one bound shifts every column after it,
//   - `joint_names` is what makes the residual's DEVICE order readable at all;
//     dropping it leaves an anonymous array that a consumer will pair with a
//     pinocchio-order Jacobian and get a finite, smooth, wrong answer.
// Each of those is asserted below against asymmetric values, so a transposition
// cannot pass.
//
// The fixture drives the real API (SetupPayloadEstimatePublisher →
// PublishOwnedTopicsFromSnapshot) on a real LifecycleNode, then reads the
// handle's own pre-filled message rather than subscribing: the fill is what is
// under test, and a DDS round trip would add timing without adding evidence.
// Mock-controller pattern follows test_controller_target_cb_group_invariant.

#include "integrated_bringup/logging/momentum_observer_log_pod.hpp"
#include "integrated_bringup/support/owned_topics.hpp"
#include <rtc_base/types/types.hpp>
#include <rtc_controller_interface/rt_controller_interface.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace integrated_bringup {

// Minimal RTControllerInterface stub — the publisher helpers only need
// `get_lifecycle_node()`, and the base's `node_` is protected.
class PayloadTopicMockController : public rtc::RTControllerInterface {
 public:
  PayloadTopicMockController() = default;

  [[nodiscard]] rtc::ControllerOutput Compute(const rtc::ControllerState&) noexcept override {
    return {};
  }

  void SetDeviceTarget(int, std::span<const double>) noexcept override {}

  [[nodiscard]] std::string_view Name() const noexcept override { return "payload_topic_mock"; }

  void SetNodeForTest(rclcpp_lifecycle::LifecycleNode::SharedPtr node) { node_ = std::move(node); }
};

namespace {

const std::vector<std::string>& ArmJointNames() {
  static const std::vector<std::string> kNames{"j1", "j2", "j3", "j4", "j5", "j6", "j7"};
  return kNames;
}

/// A row whose every field is distinct and whose wrench halves are asymmetric,
/// so a force/torque transposition or a shifted residual bound cannot pass.
MomentumObserverLogPod DistinctRow() {
  MomentumObserverLogPod pod{};
  pod.t_relative_s = 12.5;
  pod.tick = 4321;
  for (std::size_t i = 0; i < ArmJointNames().size(); ++i) {
    pod.residual[i] = 1.0 + static_cast<double>(i);  // 1..7, no repeats
  }
  pod.residual_inf_norm = 7.0;
  pod.ticks_since_seed = 91;
  pod.invalid_reason = 0;
  pod.valid = true;
  pod.payload_wrench = {1.5, 2.5, 3.5, -4.5, -5.5, -6.5};  // force > 0, torque < 0
  pod.payload_mass = 0.75;
  pod.payload_sigma_min = 0.031;
  pod.payload_lambda_sq = 0.0;
  pod.payload_fit_error = 0.048;
  pod.payload_reason = 0;
  pod.payload_valid = true;
  return pod;
}

rtc::PublishSnapshot SnapshotAt(std::int64_t stamp_ns) {
  rtc::PublishSnapshot snap{};
  snap.stamp_ns = stamp_ns;
  return snap;
}

class PayloadEstimateTopicTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    rclcpp::NodeOptions opts;
    opts.use_global_arguments(false);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_payload_estimate_node", "",
                                                             opts);
    ctrl_ = std::make_unique<PayloadTopicMockController>();
    ctrl_->SetNodeForTest(node_);
  }

  void TearDown() override {
    ResetOwnedTopics(handles_);
    ctrl_.reset();
    node_.reset();
  }

  void Setup(const std::vector<std::string>& joint_names, const std::string& payload_frame) {
    SetupPayloadEstimatePublisher(*ctrl_, handles_, "payload_estimate", joint_names, payload_frame);
  }

  void Publish(const MomentumObserverLogPod& pod) {
    PublishOwnedTopicsFromSnapshot(SnapshotAt(1'000'000'000L), handles_, /*grasp=*/nullptr,
                                   /*wbc=*/nullptr, /*tof=*/nullptr, /*payload=*/&pod);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::unique_ptr<PayloadTopicMockController> ctrl_;
  ControllerTopicHandles handles_;
};

// ── Configure-time pre-fill ──────────────────────────────────────────────────

TEST_F(PayloadEstimateTopicTest, PrefillsJointNamesAndSizesTheResidualToMatch) {
  Setup(ArmJointNames(), "ee_link");

  ASSERT_TRUE(handles_.payload_pub);
  EXPECT_EQ(handles_.payload_msg.joint_names, ArmJointNames());
  // The two arrays index the same velocity space; a consumer reads them zipped,
  // so a length disagreement is not a cosmetic defect.
  EXPECT_EQ(handles_.payload_msg.residual.size(), ArmJointNames().size());
  EXPECT_EQ(handles_.payload_msg.payload_frame, "ee_link");
}

TEST_F(PayloadEstimateTopicTest, TruncatesToTheChannelCeilingRatherThanNamingAbsentColumns) {
  std::vector<std::string> too_many;
  for (std::size_t i = 0; i < MomentumObserverLogPod::kMaxArmJoints + 4; ++i) {
    too_many.push_back("j" + std::to_string(i));
  }
  Setup(too_many, "ee_link");

  // The POD carries kMaxArmJoints entries, so naming more columns than that
  // would promise values the row can never supply.
  EXPECT_EQ(handles_.payload_msg.joint_names.size(), MomentumObserverLogPod::kMaxArmJoints);
  EXPECT_EQ(handles_.payload_msg.residual.size(), MomentumObserverLogPod::kMaxArmJoints);
}

TEST_F(PayloadEstimateTopicTest, PayloadFrameIsEmptyWhenTheEstimatorIsNotConfigured) {
  // The residual half of the message is still live in this state — that is the
  // point of gating the publisher on the observer rather than the estimator.
  Setup(ArmJointNames(), "");

  ASSERT_TRUE(handles_.payload_pub);
  EXPECT_TRUE(handles_.payload_msg.payload_frame.empty());
  EXPECT_EQ(handles_.payload_msg.joint_names.size(), ArmJointNames().size());
}

TEST_F(PayloadEstimateTopicTest, SetupIsIdempotent) {
  Setup(ArmJointNames(), "ee_link");
  const auto* first = handles_.payload_pub.get();
  Setup(ArmJointNames(), "other_link");

  EXPECT_EQ(handles_.payload_pub.get(), first);
  EXPECT_EQ(handles_.payload_msg.payload_frame, "ee_link");  // not re-filled
}

TEST_F(PayloadEstimateTopicTest, StateFrameIdStampsTheWrenchAxisFrame) {
  Setup(ArmJointNames(), "ee_link");
  SetOwnedStateFrameId(handles_, "base_link");

  // The two frames are not interchangeable: header.frame_id is the axis
  // convention (LOCAL_WORLD_ALIGNED), payload_frame is the point the wrench
  // acts at. Losing either makes the moment uninterpretable.
  EXPECT_EQ(handles_.payload_msg.header.frame_id, "base_link");
  EXPECT_EQ(handles_.payload_msg.payload_frame, "ee_link");
}

// ── Publish-time transcription ───────────────────────────────────────────────

TEST_F(PayloadEstimateTopicTest, CopiesTheRowIntoTheMessage) {
  Setup(ArmJointNames(), "ee_link");
  const auto pod = DistinctRow();
  Publish(pod);

  const auto& msg = handles_.payload_msg;
  EXPECT_EQ(msg.tick, pod.tick);
  EXPECT_DOUBLE_EQ(msg.t_relative_s, pod.t_relative_s);

  ASSERT_EQ(msg.residual.size(), ArmJointNames().size());
  for (std::size_t i = 0; i < msg.residual.size(); ++i) {
    EXPECT_DOUBLE_EQ(msg.residual[i], pod.residual[i]) << "column " << i;
  }
  EXPECT_DOUBLE_EQ(msg.residual_inf_norm, pod.residual_inf_norm);
  EXPECT_EQ(msg.ticks_since_seed, pod.ticks_since_seed);
  EXPECT_EQ(msg.residual_reason, pod.invalid_reason);
  EXPECT_TRUE(msg.residual_valid);

  EXPECT_DOUBLE_EQ(msg.mass, pod.payload_mass);
  EXPECT_DOUBLE_EQ(msg.sigma_min, pod.payload_sigma_min);
  EXPECT_DOUBLE_EQ(msg.lambda_sq, pod.payload_lambda_sq);
  EXPECT_DOUBLE_EQ(msg.fit_error, pod.payload_fit_error);
  EXPECT_EQ(msg.payload_reason, pod.payload_reason);
  EXPECT_TRUE(msg.payload_valid);
}

TEST_F(PayloadEstimateTopicTest, WrenchKeepsForceBeforeTorque) {
  Setup(ArmJointNames(), "ee_link");
  const auto pod = DistinctRow();
  Publish(pod);

  // The POD is force-first. A swapped half yields a perfectly well-formed
  // message whose mass would then be derived from a torque — so this is
  // asserted element-wise, and the fixture signs the halves oppositely.
  const auto& w = handles_.payload_msg.wrench;
  EXPECT_DOUBLE_EQ(w.force.x, pod.payload_wrench[0]);
  EXPECT_DOUBLE_EQ(w.force.y, pod.payload_wrench[1]);
  EXPECT_DOUBLE_EQ(w.force.z, pod.payload_wrench[2]);
  EXPECT_DOUBLE_EQ(w.torque.x, pod.payload_wrench[3]);
  EXPECT_DOUBLE_EQ(w.torque.y, pod.payload_wrench[4]);
  EXPECT_DOUBLE_EQ(w.torque.z, pod.payload_wrench[5]);
}

TEST_F(PayloadEstimateTopicTest, HeldTickIsPublishedCarryingTheFrozenResidual) {
  Setup(ArmJointNames(), "ee_link");
  auto pod = DistinctRow();
  pod.valid = false;
  pod.invalid_reason = 2;  // MomentumInvalidReason::kHeld — E-STOP included
  pod.payload_valid = false;
  pod.payload_reason = 2;  // PayloadInvalidReason::kHeld
  Publish(pod);

  // A held tick must not be a gap on the wire: the reason plus the frozen
  // residual is what distinguishes "the observer stopped looking" from "the
  // publisher is late". A zeroed residual here would instead assert that
  // nothing is pushing the arm.
  const auto& msg = handles_.payload_msg;
  EXPECT_FALSE(msg.residual_valid);
  EXPECT_EQ(msg.residual_reason, 2);
  EXPECT_FALSE(msg.payload_valid);
  EXPECT_EQ(msg.payload_reason, 2);
  ASSERT_EQ(msg.residual.size(), ArmJointNames().size());
  EXPECT_DOUBLE_EQ(msg.residual[0], pod.residual[0]);
  EXPECT_DOUBLE_EQ(msg.residual_inf_norm, pod.residual_inf_norm);
}

TEST_F(PayloadEstimateTopicTest, NoPublisherLeavesTheMessageUntouched) {
  // Setup deliberately not called — a controller whose observer is disabled.
  const auto pod = DistinctRow();
  Publish(pod);

  EXPECT_FALSE(handles_.payload_pub);
  EXPECT_TRUE(handles_.payload_msg.joint_names.empty());
  EXPECT_TRUE(handles_.payload_msg.residual.empty());
  EXPECT_EQ(handles_.payload_msg.tick, 0U);
}

TEST_F(PayloadEstimateTopicTest, ResetReleasesThePublisher) {
  Setup(ArmJointNames(), "ee_link");
  ASSERT_TRUE(handles_.payload_pub);

  ResetOwnedTopics(handles_);

  // A subsequent on_configure must be able to rebuild the channel; a leaked
  // handle would make the second Setup a no-op against a stale node.
  EXPECT_FALSE(handles_.payload_pub);
}

}  // namespace
}  // namespace integrated_bringup
