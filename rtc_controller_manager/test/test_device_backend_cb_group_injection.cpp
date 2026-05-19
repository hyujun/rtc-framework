// ── test_device_backend_cb_group_injection.cpp ──────────────────────────────
// Invariant test for the DeviceBackend::Configure(node, cfg, state_cb_group)
// contract introduced in Phase 3 (`f6a78b6`) of the thread-layout-v3 sprint.
//
// Contract under test: every state-lane subscription a backend creates (joint
// / motor / sensor) MUST attach itself to the supplied `state_cb_group` via
// `rclcpp::SubscriptionOptions::callback_group`. In production this routes
// hardware/sim state callbacks onto the rt_inbound thread (FIFO 70). If a
// future fourth backend forgets the SubscriptionOptions plumbing, RCL silently
// falls the subscription back onto the node's default callback group — a
// regression invisible to build/test, only detectable via `ps -T` or runtime
// jitter. This test locks the invariant at the ABI level using a
// production-shape stub.
//
// The stub mirrors the real backend pattern (UR / udp_hand / mujoco) closely
// enough that the cb_group plumbing is exercised end-to-end:
//   - create_subscription with SubscriptionOptions{ .callback_group = arg }
//   - one joint-state lane plus two optional motor/sensor lanes
// We then verify get_actual_callback_group() on each subscription matches the
// injected group exactly (non-null, same shared_ptr identity).
//
// Adding a fourth real backend? Add it (or a faithful stub mirroring its sub
// creation) to this file's parametrised cases.
// ─────────────────────────────────────────────────────────────────────────────
#include <rtc_controller_manager/device_backend.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

namespace {

// Production-shape backend stub. Creates the same lane subscriptions a real
// backend would (joint + optional motor + optional sensor), each plumbed
// through SubscriptionOptions.callback_group. WriteCommand/ReadState are
// no-ops — the test only inspects subscription wiring.
class CbGroupStubBackend : public rtc::DeviceBackend {
 public:
  CbGroupStubBackend(bool has_motor, bool has_sensor)
      : has_motor_(has_motor), has_sensor_(has_sensor) {}

  void Configure(rclcpp_lifecycle::LifecycleNode* node, const rtc::DeviceBackendConfig& config,
                 rclcpp::CallbackGroup::SharedPtr state_cb_group) override {
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = state_cb_group;

    state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
        config.state_topic, rclcpp::SensorDataQoS(),
        [](sensor_msgs::msg::JointState::ConstSharedPtr) {}, sub_opts);

    if (has_motor_) {
      motor_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
          config.motor_topic, rclcpp::SensorDataQoS(),
          [](std_msgs::msg::Float64MultiArray::ConstSharedPtr) {}, sub_opts);
    }
    if (has_sensor_) {
      sensor_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
          config.sensor_topic, rclcpp::SensorDataQoS(),
          [](std_msgs::msg::Float64MultiArray::ConstSharedPtr) {}, sub_opts);
    }
  }

  void Activate() override {}

  void Deactivate() override {}

  [[nodiscard]] bool ReadState(rtc::DeviceStateCache&) noexcept override { return false; }

  void WriteCommand(const rtc::PublishSnapshot::GroupCommandSlot&,
                    rtc::CommandType) noexcept override {}

  [[nodiscard]] std::chrono::steady_clock::time_point LastStateStamp() const noexcept override {
    return {};
  }

  [[nodiscard]] bool HasMotorState() const noexcept override { return has_motor_; }

  [[nodiscard]] bool HasSensorState() const noexcept override { return has_sensor_; }

  // Test-only accessors.
  rclcpp::SubscriptionBase::SharedPtr state_sub() const { return state_sub_; }

  rclcpp::SubscriptionBase::SharedPtr motor_sub() const { return motor_sub_; }

  rclcpp::SubscriptionBase::SharedPtr sensor_sub() const { return sensor_sub_; }

 private:
  bool has_motor_;
  bool has_sensor_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr motor_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sensor_sub_;
};

class CbGroupInjectionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    rclcpp::NodeOptions opts;
    opts.use_global_arguments(false);
    node_ =
        std::make_shared<rclcpp_lifecycle::LifecycleNode>("cb_group_injection_test_node", "", opts);
    // Production CM creates the state cb_group as MutuallyExclusive
    // (single-writer SeqLock invariant). Mirror that here.
    state_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  }

  void TearDown() override {
    state_cb_group_.reset();
    node_.reset();
  }

  rtc::DeviceBackendConfig MakeConfig() const {
    rtc::DeviceBackendConfig cfg;
    cfg.group_name = "test_group";
    cfg.type = "cb_group_stub";
    cfg.state_topic = "/test/joint_states";
    cfg.motor_topic = "/test/motor_states";
    cfg.sensor_topic = "/test/sensor_states";
    return cfg;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr state_cb_group_;

  // True iff `sub` is registered in `group`. Reverse-lookup via
  // CallbackGroup::find_subscription_ptrs_if — the jazzy rclcpp ABI does not
  // expose the group from a subscription directly, but each group keeps weak
  // refs to its subscriptions which it lets us scan.
  static bool GroupContainsSubscription(const rclcpp::CallbackGroup::SharedPtr& group,
                                        const rclcpp::SubscriptionBase::SharedPtr& sub) {
    if (!group || !sub)
      return false;
    auto match =
        group->find_subscription_ptrs_if([&](const rclcpp::SubscriptionBase::SharedPtr& candidate) {
          return candidate.get() == sub.get();
        });
    return match != nullptr;
  }
};

}  // namespace

// Joint-only lane (UR-style backend shape).
TEST_F(CbGroupInjectionTest, JointLaneAttachesToInjectedGroup) {
  CbGroupStubBackend backend(/*has_motor=*/false, /*has_sensor=*/false);
  backend.Configure(node_.get(), MakeConfig(), state_cb_group_);

  ASSERT_NE(backend.state_sub(), nullptr);
  EXPECT_TRUE(GroupContainsSubscription(state_cb_group_, backend.state_sub()))
      << "state sub did not land in the injected cb_group — SubscriptionOptions.callback_group "
         "was likely not set, silently falling back to the node's default group";
}

// All three lanes (udp_hand_native shape: joint + motor + sensor).
TEST_F(CbGroupInjectionTest, AllLanesAttachToInjectedGroup) {
  CbGroupStubBackend backend(/*has_motor=*/true, /*has_sensor=*/true);
  backend.Configure(node_.get(), MakeConfig(), state_cb_group_);

  ASSERT_NE(backend.state_sub(), nullptr);
  ASSERT_NE(backend.motor_sub(), nullptr);
  ASSERT_NE(backend.sensor_sub(), nullptr);

  EXPECT_TRUE(GroupContainsSubscription(state_cb_group_, backend.state_sub()));
  EXPECT_TRUE(GroupContainsSubscription(state_cb_group_, backend.motor_sub()))
      << "motor-lane sub silently fell back to the default group";
  EXPECT_TRUE(GroupContainsSubscription(state_cb_group_, backend.sensor_sub()))
      << "sensor-lane sub silently fell back to the default group";
}

// Null cb_group fallback (test fixtures without an executor). Backends must
// tolerate null — RCL routes the sub onto the node's default group instead,
// which here means it is NOT registered in our explicit state_cb_group_.
TEST_F(CbGroupInjectionTest, NullCbGroupFallsBackToDefault) {
  CbGroupStubBackend backend(/*has_motor=*/false, /*has_sensor=*/false);
  backend.Configure(node_.get(), MakeConfig(), /*state_cb_group=*/nullptr);

  ASSERT_NE(backend.state_sub(), nullptr);
  EXPECT_FALSE(GroupContainsSubscription(state_cb_group_, backend.state_sub()))
      << "null cb_group should have fallen back to the node default group, not the explicit one";
}
