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

#include "integrated_bringup/backends/floating_base_conventions.hpp"
#include "integrated_bringup/backends/mujoco_native_backend.hpp"
#include "integrated_bringup/controllers/hand_sensor_layout.hpp"
#include "rtc_base/types/types.hpp"
#include "rtc_controller_manager/device_backend.hpp"
#include "rtc_controller_manager/device_state_cache.hpp"
#include <rtc_msgs/msg/joint_command.hpp>
#include <rtc_msgs/msg/sim_sensor.hpp>
#include <rtc_msgs/msg/sim_sensor_state.hpp>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
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

// ── Sensor-lane receipt time and sample counter ──────────────────────────────
//
// These four pin the fields that let a consumer apply its OWN staleness
// deadline instead of inheriting this backend's `max_consecutive_missed_ticks`.
// The distinction they defend: `inference_enable` is this backend's verdict,
// the stamp is when a sample last arrived, and a catching controller needs the
// second to decide TIP_STALE against a catch deadline measured in
// milliseconds rather than in ticks of whatever rate the loop happens to run.

TEST_F(MujocoNativeBackendTest, ReadSensorState_BeforeAnyMessage_ClaimsNoReceipt) {
  rtc::DeviceStateCache cache{};
  backend_->ReadSensorState(cache);
  // Zero is "never received", not "received at time zero" — the same no-claim
  // polarity the hole masks use, so a consumer fails closed on a lane that was
  // never wired rather than computing an age against an epoch.
  for (std::size_t i = 0; i < cache.inference_recv_steady_ns.size(); ++i) {
    EXPECT_EQ(cache.inference_recv_steady_ns[i], 0) << "fingertip " << i;
    EXPECT_EQ(cache.inference_sequence[i], 0U) << "fingertip " << i;
  }
}

TEST_F(MujocoNativeBackendTest, ReadSensorState_StampsReceiptAndAdvancesSequence) {
  const int64_t before = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::steady_clock::now().time_since_epoch())
                             .count();
  PublishWrench(0, 1.0F, 0.0F, 0.0F);
  const int64_t after = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();

  rtc::DeviceStateCache first{};
  backend_->ReadSensorState(first);
  // Bracketed by two reads of the same clock the field claims to be on. A
  // stamp taken from any other clock (wall, ROS time) would land outside this
  // window, which is the failure that makes an age silently meaningless.
  EXPECT_GE(first.inference_recv_steady_ns[0], before);
  EXPECT_LE(first.inference_recv_steady_ns[0], after);
  EXPECT_EQ(first.inference_sequence[0], 1U);
  // Untouched fingertips stay unclaimed — the lane is per tip, which is the
  // whole point of the per-tip lane (one finger can go quiet while the others report).
  EXPECT_EQ(first.inference_recv_steady_ns[1], 0);
  EXPECT_EQ(first.inference_sequence[1], 0U);

  PublishWrench(0, 2.0F, 0.0F, 0.0F);
  rtc::DeviceStateCache second{};
  backend_->ReadSensorState(second);
  EXPECT_EQ(second.inference_sequence[0], 2U);
  EXPECT_GE(second.inference_recv_steady_ns[0], first.inference_recv_steady_ns[0]);
}

TEST_F(MujocoNativeBackendTest, DroppedNaNWrench_DoesNotRefreshReceipt) {
  PublishWrench(0, 7.0F, 8.0F, 9.0F);
  rtc::DeviceStateCache before{};
  backend_->ReadSensorState(before);
  ASSERT_GT(before.inference_recv_steady_ns[0], 0);

  PublishWrench(0, std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F);

  rtc::DeviceStateCache after{};
  backend_->ReadSensorState(after);
  // EXACT equality, deliberately: the force that is still in the mirror is the
  // one from the accepted message, so the age must be that message's age. A
  // stamp refreshed by the rejected sample would report a fresh force that is
  // in fact as old as the last good packet — the precise misreading the
  // stamp exists to prevent, and it would still pass an inequality-only assertion.
  EXPECT_EQ(after.inference_recv_steady_ns[0], before.inference_recv_steady_ns[0]);
  // The sequence counts ACCEPTED samples, so it does not move either. (It is
  // sourced from the same counter the miss detector uses, which is why a
  // dropped message must not bump it — that would also hide the miss.)
  EXPECT_EQ(after.inference_sequence[0], before.inference_sequence[0]);
}

TEST_F(MujocoNativeBackendTest, StaleTip_KeepsReceiptSoAgeStaysComputable) {
  PublishWrench(0, 10.0F, 0.0F, 0.0F);
  rtc::DeviceStateCache fresh{};
  backend_->ReadSensorState(fresh);
  ASSERT_TRUE(fresh.inference_enable[0]);
  // Guards this test against being vacuous: the comparison below is an
  // equality, so with a backend that never stamps at all it would compare two
  // zeros and pass while proving nothing.
  ASSERT_GT(fresh.inference_recv_steady_ns[0], 0);

  rtc::DeviceStateCache stale{};
  for (int i = 0; i <= kMaxMissedTicks; ++i) {
    stale = rtc::DeviceStateCache{};
    backend_->ReadSensorState(stale);
  }
  ASSERT_FALSE(stale.inference_enable[0]) << "miss counter did not trip";
  // Published even once the backend has given up on the tip: a consumer
  // deciding what to do about a stale fingertip needs to know HOW stale, and
  // zeroing the stamp alongside the flag would collapse "went quiet 20 ms ago"
  // into "never existed".
  EXPECT_EQ(stale.inference_recv_steady_ns[0], fresh.inference_recv_steady_ns[0]);
  EXPECT_EQ(stale.inference_sequence[0], fresh.inference_sequence[0]);
}

// ── Floating-base lane (WD-16, #976) ─────────────────────────────────────────
//
// The backend looks the configured MJCF sensor names up in a SimSensorState,
// guards them, and hands the arrays to floating_base_conventions.hpp. These
// cases pin what reaches DeviceStateCache: the converted values (against the
// header, which has its own test), the verdict / stamp / counter contract the
// wrench lane already has, and the two ways a sample is refused.

// The lane is not claimed by the fixture above (no base_state.* params).
TEST_F(MujocoNativeBackendTest, HasFloatingBase_FalseWithoutBaseStateConfig) {
  EXPECT_FALSE(backend_->HasFloatingBase());
  rtc::DeviceStateCache cache{};
  backend_->ReadBaseState(cache);
  EXPECT_FALSE(cache.base_valid);
  EXPECT_EQ(cache.base_sequence, 0U);
  EXPECT_EQ(cache.base_recv_steady_ns, 0);
}

constexpr const char* kBaseGroup = "test_floating";
constexpr const char* kBaseTopic = "/test_mujoco_base/sim_sensors";
// Robot-neutral stand-ins for the MJCF sensor names; the names reach the
// backend only through YAML, which is the ARCH-1 boundary.
constexpr const char* kPosName = "root_pos";
constexpr const char* kQuatName = "root_quat";
constexpr const char* kLinvelName = "root_linvel";
constexpr const char* kAngvelName = "root_angvel";
const std::vector<std::string> kGyroNames = {"imu_a_gyro", "imu_b_gyro"};
const std::vector<std::string> kAccelNames = {"imu_a_accel", "imu_b_accel"};

struct BaseInputs {
  std::array<double, 3> pos{0.1, -0.2, 0.75};
  // (w, x, y, z), unit, all components distinct and non-zero.
  std::array<double, 4> quat_wxyz{0.8, 0.2, -0.4, 0.4};  // |q|² = 0.64+0.04+0.16+0.16 = 1
  std::array<double, 3> linvel_w{1.0, -2.0, 0.5};
  std::array<double, 3> angvel_w{0.3, 0.6, -0.9};
  std::array<std::array<double, 3>, 2> gyro{{{1.0, 2.0, 3.0}, {-1.0, -2.0, -3.0}}};
  std::array<std::array<double, 3>, 2> accel{{{4.0, 5.0, 6.0}, {-4.0, -5.0, -6.0}}};
};

rtc_msgs::msg::SimSensor Sensor(const std::string& name, const std::vector<double>& values) {
  rtc_msgs::msg::SimSensor s;
  s.name = name;
  s.values = values;
  return s;
}

// A message in a deliberately shuffled order, with an unrelated sensor first,
// so a backend that trusted positions instead of names would read the wrong
// quantity.
rtc_msgs::msg::SimSensorState BaseMessage(const BaseInputs& in, int num_imu = 2) {
  rtc_msgs::msg::SimSensorState msg;
  msg.sensors.push_back(Sensor("unrelated_touch", {42.0}));
  msg.sensors.push_back(Sensor(kAngvelName, {in.angvel_w[0], in.angvel_w[1], in.angvel_w[2]}));
  for (int s = 0; s < num_imu; ++s) {
    const auto su = static_cast<std::size_t>(s);
    msg.sensors.push_back(
        Sensor(kAccelNames[su], {in.accel[su][0], in.accel[su][1], in.accel[su][2]}));
    msg.sensors.push_back(Sensor(kGyroNames[su], {in.gyro[su][0], in.gyro[su][1], in.gyro[su][2]}));
  }
  msg.sensors.push_back(
      Sensor(kQuatName, {in.quat_wxyz[0], in.quat_wxyz[1], in.quat_wxyz[2], in.quat_wxyz[3]}));
  msg.sensors.push_back(Sensor(kPosName, {in.pos[0], in.pos[1], in.pos[2]}));
  msg.sensors.push_back(Sensor(kLinvelName, {in.linvel_w[0], in.linvel_w[1], in.linvel_w[2]}));
  return msg;
}

class MujocoBaseLaneTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::NodeOptions options;
    const std::string prefix = std::string("devices.") + kBaseGroup + ".backend";
    options.parameter_overrides({
        rclcpp::Parameter(prefix + ".max_consecutive_missed_ticks",
                          static_cast<int64_t>(kMaxMissedTicks)),
        rclcpp::Parameter(prefix + ".base_state.sensor_topic", std::string(kBaseTopic)),
        rclcpp::Parameter(prefix + ".base_state.pos", std::string(kPosName)),
        rclcpp::Parameter(prefix + ".base_state.quat", std::string(kQuatName)),
        rclcpp::Parameter(prefix + ".base_state.linvel", std::string(kLinvelName)),
        rclcpp::Parameter(prefix + ".base_state.angvel", std::string(kAngvelName)),
        rclcpp::Parameter(prefix + ".base_state.imu_gyro", kGyroNames),
        rclcpp::Parameter(prefix + ".base_state.imu_accel", kAccelNames),
    });
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_mujoco_base_lane", options);
    state_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rtc::DeviceBackendConfig cfg;
    cfg.group_name = kBaseGroup;
    cfg.type = "mujoco_native";
    cfg.state_topic = "/test_mujoco_base/joint_states";
    cfg.command_topic = "/test_mujoco_base/joint_command";
    backend_ = std::make_unique<rtc::MujocoNativeBackend>();
    backend_->Configure(node_.get(), cfg, state_cb_group_);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());

    rclcpp::QoS qos{1};
    qos.best_effort();
    pub_ = node_->create_publisher<rtc_msgs::msg::SimSensorState>(kBaseTopic, qos);
    pub_->on_activate();
  }

  void TearDown() override {
    pub_.reset();
    executor_->remove_node(node_->get_node_base_interface());
    executor_.reset();
    backend_.reset();
    node_.reset();
  }

  // Publishes and pumps until the backend's sequence moves past `seq_before`
  // (or, for a sample the backend must refuse, for a fixed grace period).
  void Publish(const rtc_msgs::msg::SimSensorState& msg, bool expect_accept) {
    rtc::DeviceStateCache probe{};
    backend_->ReadBaseState(probe);
    const uint64_t seq_before = probe.base_sequence;
    pub_->publish(msg);
    const auto deadline = std::chrono::steady_clock::now() + (expect_accept ? 2000ms : 300ms);
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(20ms);
      if (expect_accept) {
        backend_->ReadBaseState(probe);
        if (probe.base_sequence != seq_before)
          return;
      }
    }
    if (expect_accept) {
      FAIL() << "base sample never accepted";
    }
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr state_cb_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<rtc::MujocoNativeBackend> backend_;
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::SimSensorState>::SharedPtr pub_;
};

TEST_F(MujocoBaseLaneTest, HasFloatingBase_TrueWithBaseStateConfig) {
  EXPECT_TRUE(backend_->HasFloatingBase());
}

TEST_F(MujocoBaseLaneTest, BeforeAnyMessage_IdentityPoseInvalidNoReceipt) {
  rtc::DeviceStateCache cache{};
  backend_->ReadBaseState(cache);
  EXPECT_FALSE(cache.base_valid);
  EXPECT_EQ(cache.base_sequence, 0U);
  EXPECT_EQ(cache.base_recv_steady_ns, 0);
  EXPECT_EQ(cache.num_imu_sites, 0);
  // The mirror's default is the identity pose, the same no-claim default
  // DeviceState documents: a consumer that copies before checking the flag
  // gets a rotation, not a zero quaternion.
  EXPECT_DOUBLE_EQ(cache.base_pose[6], 1.0);
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(cache.base_pose[i], 0.0) << i;
  }
}

// 1 message → pose · twist · imu equal the header's conversion to 1e-12,
// base_valid true, sequence 1, receipt stamped on the steady clock.
TEST_F(MujocoBaseLaneTest, OneMessage_ConvertedValuesValidSequenceAndStamp) {
  const BaseInputs in;
  const int64_t before = rtc::SteadyNowNs();
  Publish(BaseMessage(in), /*expect_accept=*/true);
  const int64_t after = rtc::SteadyNowNs();

  rtc::DeviceStateCache cache{};
  backend_->ReadBaseState(cache);
  ASSERT_TRUE(cache.base_valid);
  EXPECT_EQ(cache.base_sequence, 1U);
  EXPECT_GE(cache.base_recv_steady_ns, before);
  EXPECT_LE(cache.base_recv_steady_ns, after);

  const auto expect = rtc::MujocoFrameToPinocchio(in.pos, in.quat_wxyz, in.linvel_w, in.angvel_w);
  for (std::size_t i = 0; i < expect.pose.size(); ++i) {
    EXPECT_NEAR(cache.base_pose[i], expect.pose[i], 1e-12) << "pose[" << i << "]";
  }
  for (std::size_t i = 0; i < expect.twist.size(); ++i) {
    EXPECT_NEAR(cache.base_twist[i], expect.twist[i], 1e-12) << "twist[" << i << "]";
  }
  // Order pinned independently of the header: q_xyzw ends with w = 0.8, and
  // the twist is NOT the world vector (the quaternion is not the identity).
  EXPECT_DOUBLE_EQ(cache.base_pose[6], 0.8);
  EXPECT_DOUBLE_EQ(cache.base_pose[3], 0.2);
  EXPECT_GT(std::abs(cache.base_twist[0] - in.linvel_w[0]) +
                std::abs(cache.base_twist[1] - in.linvel_w[1]),
            1e-3);

  ASSERT_EQ(cache.num_imu_sites, 2);
  for (std::size_t s = 0; s < 2; ++s) {
    for (std::size_t i = 0; i < 3; ++i) {
      EXPECT_DOUBLE_EQ(cache.imu[s][i], in.gyro[s][i]) << "site " << s << " gyro " << i;
      EXPECT_DOUBLE_EQ(cache.imu[s][3 + i], in.accel[s][i]) << "site " << s << " accel " << i;
    }
  }
}

TEST_F(MujocoBaseLaneTest, SecondMessage_AdvancesSequenceByOne) {
  BaseInputs in;
  Publish(BaseMessage(in), true);
  in.pos[2] = 0.5;
  Publish(BaseMessage(in), true);
  rtc::DeviceStateCache cache{};
  backend_->ReadBaseState(cache);
  EXPECT_EQ(cache.base_sequence, 2U);
  EXPECT_DOUBLE_EQ(cache.base_pose[2], 0.5);
}

// max_consecutive_missed_ticks empty RT reads → base_valid false, values and
// stamp kept (the DeviceState contract: the flag is a verdict, not a gate).
TEST_F(MujocoBaseLaneTest, Stale_AfterMaxMissedTicks_ClearsValidKeepsValues) {
  const BaseInputs in;
  Publish(BaseMessage(in), true);
  rtc::DeviceStateCache fresh{};
  backend_->ReadBaseState(fresh);
  ASSERT_TRUE(fresh.base_valid);
  ASSERT_GT(fresh.base_recv_steady_ns, 0);

  // RT reads since the sample arrived: Publish()'s acceptance probe was the
  // first (miss 0), `fresh` above the second (miss 1). The verdict holds up
  // to miss max-1 and flips AT miss == max — the same count the wrench lane's
  // Stale_AfterMaxMissedTicks_DisablesEnable pins, offset by the probe.
  rtc::DeviceStateCache cache{};
  for (int i = 0; i < kMaxMissedTicks - 2; ++i) {
    backend_->ReadBaseState(cache);
    EXPECT_TRUE(cache.base_valid) << "after " << (i + 2) << " misses";
  }
  backend_->ReadBaseState(cache);  // miss == kMaxMissedTicks
  EXPECT_FALSE(cache.base_valid);
  EXPECT_EQ(cache.base_pose, fresh.base_pose);
  EXPECT_EQ(cache.base_twist, fresh.base_twist);
  EXPECT_EQ(cache.base_recv_steady_ns, fresh.base_recv_steady_ns);
  EXPECT_EQ(cache.base_sequence, fresh.base_sequence);

  // Recovery on the next accepted sample.
  Publish(BaseMessage(in), true);
  backend_->ReadBaseState(cache);
  EXPECT_TRUE(cache.base_valid);
  EXPECT_EQ(cache.base_sequence, 2U);
}

// A non-finite component anywhere in the sample drops the WHOLE sample:
// values, stamp and counter stay those of the last accepted one.
TEST_F(MujocoBaseLaneTest, NonFiniteSample_DroppedValuesAndStampUnchanged) {
  const BaseInputs good;
  Publish(BaseMessage(good), true);
  rtc::DeviceStateCache before{};
  backend_->ReadBaseState(before);
  ASSERT_TRUE(before.base_valid);

  BaseInputs bad = good;
  bad.pos[0] = 9.0;  // would be visible if the sample got through …
  bad.linvel_w[1] = std::numeric_limits<double>::quiet_NaN();  // … but this refuses it
  Publish(BaseMessage(bad), /*expect_accept=*/false);

  BaseInputs bad_imu = good;
  bad_imu.pos[0] = 8.0;
  bad_imu.gyro[1][2] = std::numeric_limits<double>::infinity();
  Publish(BaseMessage(bad_imu), false);

  rtc::DeviceStateCache after{};
  backend_->ReadBaseState(after);
  EXPECT_EQ(after.base_sequence, before.base_sequence);
  EXPECT_EQ(after.base_recv_steady_ns, before.base_recv_steady_ns);
  EXPECT_EQ(after.base_pose, before.base_pose);
  EXPECT_EQ(after.base_twist, before.base_twist);
  EXPECT_EQ(after.imu, before.imu);
}

TEST_F(MujocoBaseLaneTest, ZeroQuaternion_Dropped) {
  const BaseInputs good;
  Publish(BaseMessage(good), true);
  BaseInputs bad = good;
  bad.quat_wxyz = {0.0, 0.0, 0.0, 0.0};
  Publish(BaseMessage(bad), false);
  rtc::DeviceStateCache cache{};
  backend_->ReadBaseState(cache);
  EXPECT_EQ(cache.base_sequence, 1U);
  EXPECT_DOUBLE_EQ(cache.base_pose[6], 0.8);
}

// A configured name the message does not carry: nothing is accepted, so the
// lane never becomes valid and the counter never moves (plus a one-shot WARN
// that this test cannot observe — it is for the launch log).
TEST_F(MujocoBaseLaneTest, MissingConfiguredName_LaneStaysInvalid) {
  const BaseInputs in;
  auto msg = BaseMessage(in);
  // Remove the linvel sensor.
  for (auto it = msg.sensors.begin(); it != msg.sensors.end(); ++it) {
    if (it->name == kLinvelName) {
      msg.sensors.erase(it);
      break;
    }
  }
  Publish(msg, /*expect_accept=*/false);

  rtc::DeviceStateCache cache{};
  backend_->ReadBaseState(cache);
  EXPECT_FALSE(cache.base_valid);
  EXPECT_EQ(cache.base_sequence, 0U);
  EXPECT_EQ(cache.base_recv_steady_ns, 0);

  // A wrong dimension under the right name is the same failure.
  auto msg2 = BaseMessage(in);
  for (auto& s : msg2.sensors) {
    if (s.name == kQuatName)
      s.values = {1.0, 0.0, 0.0};  // 3 values, not 4
  }
  Publish(msg2, false);
  backend_->ReadBaseState(cache);
  EXPECT_FALSE(cache.base_valid);
  EXPECT_EQ(cache.base_sequence, 0U);

  // And a missing IMU sensor refuses the sample as a whole, not just the IMU.
  auto msg3 = BaseMessage(in, /*num_imu=*/1);
  Publish(msg3, false);
  backend_->ReadBaseState(cache);
  EXPECT_FALSE(cache.base_valid);
  EXPECT_EQ(cache.base_sequence, 0U);
}

// `base_state.max_consecutive_missed_ticks` is the base lane's OWN budget:
// here 2 while the shared key stays at kMaxMissedTicks (5), so the verdict
// flips after 2 silent ticks, not 5. (The fixture above leaves it unset and
// so pins the fallback to the shared value.)
TEST(MujocoBaseLaneBudgetTest, OwnMissBudgetOverridesSharedOne) {
  constexpr const char* kGroup = "test_floating_budget";
  constexpr const char* kTopic = "/test_mujoco_base_budget/sim_sensors";
  rclcpp::NodeOptions options;
  const std::string prefix = std::string("devices.") + kGroup + ".backend";
  options.parameter_overrides({
      rclcpp::Parameter(prefix + ".max_consecutive_missed_ticks",
                        static_cast<int64_t>(kMaxMissedTicks)),
      rclcpp::Parameter(prefix + ".base_state.max_consecutive_missed_ticks",
                        static_cast<int64_t>(2)),
      rclcpp::Parameter(prefix + ".base_state.sensor_topic", std::string(kTopic)),
      rclcpp::Parameter(prefix + ".base_state.pos", std::string(kPosName)),
      rclcpp::Parameter(prefix + ".base_state.quat", std::string(kQuatName)),
      rclcpp::Parameter(prefix + ".base_state.linvel", std::string(kLinvelName)),
      rclcpp::Parameter(prefix + ".base_state.angvel", std::string(kAngvelName)),
  });
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_mujoco_base_budget", options);
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rtc::DeviceBackendConfig cfg;
  cfg.group_name = kGroup;
  cfg.type = "mujoco_native";
  cfg.state_topic = "/test_mujoco_base_budget/joint_states";
  cfg.command_topic = "/test_mujoco_base_budget/joint_command";
  auto backend = std::make_unique<rtc::MujocoNativeBackend>();
  backend->Configure(node.get(), cfg, group);
  ASSERT_TRUE(backend->HasFloatingBase());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  rclcpp::QoS qos{1};
  qos.best_effort();
  auto pub = node->create_publisher<rtc_msgs::msg::SimSensorState>(kTopic, qos);
  pub->on_activate();

  // No IMU names configured → a message without IMU sensors is complete.
  const BaseInputs in;
  rtc::DeviceStateCache cache{};
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    pub->publish(BaseMessage(in, /*num_imu=*/0));
    exec.spin_some(20ms);
    backend->ReadBaseState(cache);  // this read, on arrival, is miss 0
    if (cache.base_sequence > 0)
      break;
  }
  ASSERT_TRUE(cache.base_valid) << "base sample never accepted";
  EXPECT_EQ(cache.num_imu_sites, 0);

  backend->ReadBaseState(cache);  // miss 1
  EXPECT_TRUE(cache.base_valid);
  backend->ReadBaseState(cache);  // miss 2 == own budget → invalid (shared would allow 5)
  EXPECT_FALSE(cache.base_valid);

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
