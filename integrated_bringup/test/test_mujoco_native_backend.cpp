// Tests for MujocoNativeBackend's fingertip wrench lane (Layer C).
//
// These tests pin the contract the WBC / Joint controllers rely on:
//   - HasSensorState() reports true (sensor lane is now owned by mujoco)
//   - inference_data slots 1..3 carry fx/fy/fz, slots 0 / 4..6 are zeroed
//   - stale detection trips inference_enable[f] after
//     max_consecutive_missed_ticks empty RT ticks, but the last force
//     values are preserved so the controller can keep using them
//   - NaN/Inf wrench messages are dropped without disturbing the mirror
//
// Plus the WriteCommand direct-copy contract (command-ordering convention):
//   - published JointCommand carries joint_names == joint_command_names and
//     values[i] == slot.commands[i] — no command-side reorder

#include "integrated_bringup/backends/mujoco_native_backend.hpp"
#include "integrated_bringup/controllers/hand_sensor_layout.hpp"
#include "rtc_controller_manager/device_backend.hpp"
#include "rtc_controller_manager/device_state_cache.hpp"
#include <rtc_msgs/msg/joint_command.hpp>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace {

using namespace std::chrono_literals;

// Per-fingertip inference stride (sensor union: contact + F + u = 7).
// Sourced from integrated_bringup's own SSoT so tests stay in lockstep when
// the layout evolves (e.g. capacitive emulation extending sensor B path).
constexpr std::size_t kInferStride = integrated_bringup::kHandInferenceValuesPerFingertipCapacity;

constexpr const char* kGroupName = "test_leap";
constexpr int kMaxMissedTicks = 5;
const std::vector<std::string> kWrenchTopics = {
    "/test_mujoco/ft0/contact_wrench",
    "/test_mujoco/ft1/contact_wrench",
    "/test_mujoco/ft2/contact_wrench",
    "/test_mujoco/ft3/contact_wrench",
};

class MujocoNativeBackendTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::NodeOptions options;
    // Pre-inject parameter overrides so backend->Configure picks them up
    // when it calls declare_parameter (Y2c ros 2 param channel).
    const std::string prefix = std::string("devices.") + kGroupName + ".backend";
    options.parameter_overrides({
        rclcpp::Parameter(prefix + ".fingertip_wrench_topics", kWrenchTopics),
        rclcpp::Parameter(prefix + ".max_consecutive_missed_ticks",
                          static_cast<int64_t>(kMaxMissedTicks)),
    });
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_mujoco_backend", options);
    state_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rtc::DeviceBackendConfig cfg;
    cfg.group_name = kGroupName;
    cfg.type = "mujoco_native";
    cfg.state_topic = "/test_mujoco/joint_states";
    cfg.command_topic = "/test_mujoco/joint_command";
    backend_ = std::make_unique<rtc::MujocoNativeBackend>();
    backend_->Configure(node_.get(), cfg, state_cb_group_);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());

    // Publishers live on the same node — cb_group=default is fine for outbound.
    for (const auto& topic : kWrenchTopics) {
      wrench_pubs_.push_back(node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
          topic, rclcpp::SensorDataQoS()));
    }
  }

  void TearDown() override {
    wrench_pubs_.clear();
    executor_->remove_node(node_->get_node_base_interface());
    executor_.reset();
    backend_.reset();
    node_.reset();
  }

  void PublishWrench(int finger, float fx, float fy, float fz) {
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = node_->now();
    msg.wrench.force.x = static_cast<double>(fx);
    msg.wrench.force.y = static_cast<double>(fy);
    msg.wrench.force.z = static_cast<double>(fz);
    wrench_pubs_[static_cast<std::size_t>(finger)]->publish(msg);
    SpinUntilDelivered();
  }

  void SpinUntilDelivered() {
    // SensorDataQoS publish + intra-process delivery may take a few spin
    // cycles; spin_some up to a small budget so tests stay deterministic.
    const auto deadline = std::chrono::steady_clock::now() + 200ms;
    for (int i = 0; i < 20; ++i) {
      executor_->spin_some(20ms);
      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }
    }
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr state_cb_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<rtc::MujocoNativeBackend> backend_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr> wrench_pubs_;
};

// 1. Capability flag — sensor lane now owned by mujoco backend.
TEST_F(MujocoNativeBackendTest, HasSensorState_ReturnsTrue) {
  EXPECT_TRUE(backend_->HasSensorState());
}

// 2. Pre-message state: nothing fresh, controller sees zero fingertips.
TEST_F(MujocoNativeBackendTest, ReadSensorState_BeforeAnyMessage_DisablesAll) {
  rtc::DeviceStateCache cache{};
  backend_->ReadSensorState(cache);
  EXPECT_EQ(cache.num_inference_groups, 0);
  for (std::size_t i = 0; i < cache.inference_enable.size(); ++i) {
    EXPECT_FALSE(cache.inference_enable[i]) << "fingertip " << i;
  }
}

// 3. Single wrench → inference_data slots 1..3 populated, slot 0/4..6 zero.
TEST_F(MujocoNativeBackendTest, ReadSensorState_AfterWrenchMessage_PopulatesForce) {
  PublishWrench(0, 1.0F, 2.0F, 3.0F);

  rtc::DeviceStateCache cache{};
  backend_->ReadSensorState(cache);

  ASSERT_GE(cache.num_inference_groups, 1);
  EXPECT_TRUE(cache.inference_enable[0]);
  EXPECT_FLOAT_EQ(cache.inference_data[1], 1.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[2], 2.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[3], 3.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[0], 0.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[4], 0.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[5], 0.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[6], 0.0F);
}

// 4. Independent subscriptions: each fingertip mapped to its own stride block.
TEST_F(MujocoNativeBackendTest, AllFingertips_IndependentSubscriptions) {
  PublishWrench(0, 1.0F, 0.0F, 0.0F);
  PublishWrench(1, 0.0F, 2.0F, 0.0F);
  PublishWrench(2, 0.0F, 0.0F, 3.0F);
  PublishWrench(3, 4.0F, 5.0F, 6.0F);

  rtc::DeviceStateCache cache{};
  backend_->ReadSensorState(cache);

  ASSERT_GE(cache.num_inference_groups, 4);
  for (int f = 0; f < 4; ++f) {
    EXPECT_TRUE(cache.inference_enable[static_cast<std::size_t>(f)]) << "f=" << f;
  }
  EXPECT_FLOAT_EQ(cache.inference_data[0 * kInferStride + 1], 1.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[1 * kInferStride + 2], 2.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[2 * kInferStride + 3], 3.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[3 * kInferStride + 1], 4.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[3 * kInferStride + 2], 5.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[3 * kInferStride + 3], 6.0F);
}

// 5. NaN/Inf force is rejected — mirror keeps last valid value.
TEST_F(MujocoNativeBackendTest, NaN_InForce_Rejected_LastValuePreserved) {
  PublishWrench(0, 7.0F, 8.0F, 9.0F);
  rtc::DeviceStateCache before{};
  backend_->ReadSensorState(before);
  ASSERT_TRUE(before.inference_enable[0]);

  // Publish NaN; mirror update must be skipped.
  PublishWrench(0, std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F);

  rtc::DeviceStateCache after{};
  backend_->ReadSensorState(after);
  EXPECT_FLOAT_EQ(after.inference_data[1], 7.0F);
  EXPECT_FLOAT_EQ(after.inference_data[2], 8.0F);
  EXPECT_FLOAT_EQ(after.inference_data[3], 9.0F);
}

// 6. Stale detection: after max_consecutive_missed_ticks empty reads the
//    enable flag flips to false, but the cached force is preserved.
TEST_F(MujocoNativeBackendTest, Stale_AfterMaxMissedTicks_DisablesEnable) {
  PublishWrench(0, 10.0F, 0.0F, 0.0F);

  rtc::DeviceStateCache cache{};
  backend_->ReadSensorState(cache);
  EXPECT_TRUE(cache.inference_enable[0]);

  // miss_count progresses 1..(max-1) — still fresh.
  for (int i = 0; i < kMaxMissedTicks - 1; ++i) {
    backend_->ReadSensorState(cache);
    EXPECT_TRUE(cache.inference_enable[0]) << "after " << (i + 1) << " misses";
  }
  // Crossing the threshold — enable flips, force preserved.
  backend_->ReadSensorState(cache);
  EXPECT_FALSE(cache.inference_enable[0]);
  EXPECT_FLOAT_EQ(cache.inference_data[1], 10.0F);
}

// 7. Recovery after stale: a new wrench resets the counter, enable=true again.
TEST_F(MujocoNativeBackendTest, Recovery_AfterStale_ReenablesOnNewMessage) {
  PublishWrench(0, 1.0F, 0.0F, 0.0F);
  rtc::DeviceStateCache cache{};
  for (int i = 0; i < kMaxMissedTicks + 1; ++i) {
    backend_->ReadSensorState(cache);
  }
  ASSERT_FALSE(cache.inference_enable[0]);

  PublishWrench(0, 11.0F, 12.0F, 13.0F);
  backend_->ReadSensorState(cache);
  EXPECT_TRUE(cache.inference_enable[0]);
  EXPECT_FLOAT_EQ(cache.inference_data[1], 11.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[2], 12.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[3], 13.0F);
}

// 8. Per-fingertip miss counters do not bleed into each other.
TEST_F(MujocoNativeBackendTest, PerFingertip_MissCount_Independent) {
  PublishWrench(0, 1.0F, 0.0F, 0.0F);
  PublishWrench(1, 2.0F, 0.0F, 0.0F);

  rtc::DeviceStateCache cache{};
  backend_->ReadSensorState(cache);
  ASSERT_TRUE(cache.inference_enable[0]);
  ASSERT_TRUE(cache.inference_enable[1]);

  // Refresh finger 0 each tick; let finger 1 starve past the threshold.
  for (int i = 0; i < kMaxMissedTicks + 2; ++i) {
    PublishWrench(0, 1.0F, 0.0F, 0.0F);
    backend_->ReadSensorState(cache);
  }
  EXPECT_TRUE(cache.inference_enable[0]);
  EXPECT_FALSE(cache.inference_enable[1]);
}

// 9. Miss counter saturates — many empty ticks must not overflow / wrap.
TEST_F(MujocoNativeBackendTest, SaturatingMissCount_NoOverflow) {
  PublishWrench(0, 1.0F, 0.0F, 0.0F);

  rtc::DeviceStateCache cache{};
  for (int i = 0; i < 1000; ++i) {
    backend_->ReadSensorState(cache);
  }
  EXPECT_FALSE(cache.inference_enable[0]);

  // Single recovery still works after long starvation.
  PublishWrench(0, 2.0F, 0.0F, 0.0F);
  backend_->ReadSensorState(cache);
  EXPECT_TRUE(cache.inference_enable[0]);
  EXPECT_FLOAT_EQ(cache.inference_data[1], 2.0F);
}

// 10. Stride 7 dead slots are explicitly zero (not garbage from prior frames).
TEST_F(MujocoNativeBackendTest, Stride7_DeadSlotsAreZero) {
  PublishWrench(1, 9.0F, 9.0F, 9.0F);

  rtc::DeviceStateCache cache{};
  backend_->ReadSensorState(cache);
  ASSERT_GE(cache.num_inference_groups, 2);
  EXPECT_FLOAT_EQ(cache.inference_data[1 * kInferStride + 0], 0.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[1 * kInferStride + 4], 0.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[1 * kInferStride + 5], 0.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[1 * kInferStride + 6], 0.0F);
}

// ── WriteCommand direct-copy contract ────────────────────────────────────────
//
// Pins the command-ordering convention: `slot.commands` is already in
// `joint_command_names` order, so the published JointCommand carries
// `joint_names == config.joint_command_names` and `values[i] ==
// slot.commands[i]` — no command-side reorder. Wire-order differences are the
// receiver's job (it reorders by `joint_names`).

const std::vector<std::string> kCmdNames = {"j2", "j0", "j3", "j1"};

class MujocoBackendWriteCommandTest : public ::testing::Test {
 protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_mujoco_backend_cmd");
    state_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rtc::DeviceBackendConfig cfg;
    cfg.group_name = "test_leap_cmd";
    cfg.type = "mujoco_native";
    cfg.state_topic = "/test_mujoco_cmd/joint_states";
    cfg.command_topic = "/test_mujoco_cmd/joint_command";
    cfg.joint_command_names = kCmdNames;
    backend_ = std::make_unique<rtc::MujocoNativeBackend>();
    backend_->Configure(node_.get(), cfg, state_cb_group_);
    backend_->Activate();  // LifecyclePublisher publishes only when active.

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());

    rclcpp::QoS qos{1};
    qos.best_effort();
    cmd_sub_ = node_->create_subscription<rtc_msgs::msg::JointCommand>(
        cfg.command_topic, qos,
        [this](rtc_msgs::msg::JointCommand::SharedPtr msg) { received_.push_back(*msg); });
  }

  void TearDown() override {
    cmd_sub_.reset();
    executor_->remove_node(node_->get_node_base_interface());
    executor_.reset();
    backend_.reset();
    node_.reset();
  }

  // Publishes via the backend and spins until the sub delivers (or timeout).
  rtc_msgs::msg::JointCommand WriteAndReceive(const rtc::PublishSnapshot::GroupCommandSlot& slot,
                                              rtc::CommandType command_type) {
    received_.clear();
    backend_->WriteCommand(slot, command_type);
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (received_.empty() && std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(20ms);
    }
    EXPECT_FALSE(received_.empty()) << "JointCommand not delivered within timeout";
    return received_.empty() ? rtc_msgs::msg::JointCommand{} : received_.back();
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr state_cb_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<rtc::MujocoNativeBackend> backend_;
  rclcpp::Subscription<rtc_msgs::msg::JointCommand>::SharedPtr cmd_sub_;
  std::vector<rtc_msgs::msg::JointCommand> received_;
};

// 11. Position command: names follow config order, values are a direct copy of
//     slot.commands (index-aligned), feedforward stays zeroed.
TEST_F(MujocoBackendWriteCommandTest, WriteCommand_DirectCopy_PositionMode) {
  rtc::PublishSnapshot::GroupCommandSlot slot{};
  slot.num_channels = static_cast<int>(kCmdNames.size());
  const std::vector<double> cmds = {0.1, -0.2, 0.3, -0.4};
  for (std::size_t i = 0; i < cmds.size(); ++i) {
    slot.commands[i] = cmds[i];
    slot.feedforward[i] = 100.0 + static_cast<double>(i);  // must NOT leak out
  }

  const auto msg = WriteAndReceive(slot, rtc::CommandType::kPosition);

  EXPECT_EQ(msg.joint_names, kCmdNames);
  ASSERT_EQ(msg.values.size(), kCmdNames.size());
  ASSERT_EQ(msg.feedforward.size(), kCmdNames.size());
  for (std::size_t i = 0; i < kCmdNames.size(); ++i) {
    EXPECT_DOUBLE_EQ(msg.values[i], cmds[i]) << "i=" << i;
    EXPECT_DOUBLE_EQ(msg.feedforward[i], 0.0) << "i=" << i;
  }
}

// 12. PD-feedforward command: feedforward is also a direct index-aligned copy.
TEST_F(MujocoBackendWriteCommandTest, WriteCommand_DirectCopy_PdFeedforwardMode) {
  rtc::PublishSnapshot::GroupCommandSlot slot{};
  slot.num_channels = static_cast<int>(kCmdNames.size());
  const std::vector<double> cmds = {1.0, 2.0, 3.0, 4.0};
  const std::vector<double> ffs = {-0.5, 0.25, -0.125, 0.0625};
  for (std::size_t i = 0; i < cmds.size(); ++i) {
    slot.commands[i] = cmds[i];
    slot.feedforward[i] = ffs[i];
  }

  const auto msg = WriteAndReceive(slot, rtc::CommandType::kPdFeedforward);

  EXPECT_EQ(msg.joint_names, kCmdNames);
  ASSERT_EQ(msg.values.size(), kCmdNames.size());
  ASSERT_EQ(msg.feedforward.size(), kCmdNames.size());
  for (std::size_t i = 0; i < kCmdNames.size(); ++i) {
    EXPECT_DOUBLE_EQ(msg.values[i], cmds[i]) << "i=" << i;
    EXPECT_DOUBLE_EQ(msg.feedforward[i], ffs[i]) << "i=" << i;
  }
}

// 13. Header stamp: WriteCommand propagates slot.stamp_ns (wall-clock ns) into
//     the JointCommand header, split into sec/nanosec. Pins the wall-clock time
//     axis contract for /<key>/joint_command (rosbag/tf2/message_filters).
TEST_F(MujocoBackendWriteCommandTest, WriteCommand_PropagatesStampToHeader) {
  rtc::PublishSnapshot::GroupCommandSlot slot{};
  slot.num_channels = static_cast<int>(kCmdNames.size());
  const std::vector<double> cmds = {0.1, 0.2, 0.3, 0.4};
  for (std::size_t i = 0; i < cmds.size(); ++i)
    slot.commands[i] = cmds[i];
  // Arbitrary wall-clock ns with a non-zero sub-second remainder.
  constexpr int64_t kStampNs = 1'752'000'000'123'456'789LL;
  slot.stamp_ns = kStampNs;

  const auto msg = WriteAndReceive(slot, rtc::CommandType::kPosition);

  EXPECT_EQ(msg.header.stamp.sec, static_cast<int32_t>(kStampNs / 1'000'000'000LL));
  EXPECT_EQ(msg.header.stamp.nanosec, static_cast<uint32_t>(kStampNs % 1'000'000'000LL));
}

// End-to-end named-JointState reorder (issue #156): a shuffled named message
// published onto the state topic must land in joint_command_names (device)
// order in ReadState(). All three backends delegate to the same
// JointStateReorder helper (unit-tested in test_joint_state_reorder.cpp);
// this locks the topic→callback→SeqLock wiring on one representative backend.
TEST(MujocoNativeBackendReorderTest, NamedShuffledJointState_ReadsBackInDeviceOrder) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_mujoco_reorder");
  auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rtc::DeviceBackendConfig cfg;
  cfg.group_name = "test_leap_reorder";
  cfg.type = "mujoco_native";
  cfg.state_topic = "/test_mujoco_reorder/joint_states";
  cfg.command_topic = "/test_mujoco_reorder/joint_command";
  cfg.joint_command_names = {"j0", "j1", "j2"};

  auto backend = std::make_unique<rtc::MujocoNativeBackend>();
  backend->Configure(node.get(), cfg, cb_group);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());

  auto pub = node->create_publisher<sensor_msgs::msg::JointState>(
      cfg.state_topic, rclcpp::SensorDataQoS().keep_last(1));
  // LifecycleNode::create_publisher returns a gated LifecyclePublisher —
  // publish() silently drops until on_activate().
  pub->on_activate();

  sensor_msgs::msg::JointState msg;
  msg.name = {"j2", "j0", "j1"};
  msg.position = {12.0, 10.0, 11.0};
  msg.velocity = {22.0, 20.0, 21.0};
  msg.effort = {32.0, 30.0, 31.0};

  rtc::DeviceStateCache cache{};
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    pub->publish(msg);
    exec.spin_some(20ms);
    if (backend->ReadState(cache))
      break;
  }

  ASSERT_TRUE(cache.valid) << "state message never delivered";
  EXPECT_EQ(cache.num_channels, 3);
  EXPECT_DOUBLE_EQ(cache.positions[0], 10.0);
  EXPECT_DOUBLE_EQ(cache.positions[1], 11.0);
  EXPECT_DOUBLE_EQ(cache.positions[2], 12.0);
  EXPECT_DOUBLE_EQ(cache.velocities[0], 20.0);
  EXPECT_DOUBLE_EQ(cache.velocities[2], 22.0);
  EXPECT_DOUBLE_EQ(cache.efforts[0], 30.0);
  EXPECT_DOUBLE_EQ(cache.efforts[2], 32.0);

  exec.remove_node(node->get_node_base_interface());
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
