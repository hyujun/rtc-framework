// ── UdpHandNativeBackend sensor lane ─────────────────────────────────────────
//
// This suite exists because the fingertip lane grew a per-group receipt time
// and sample counter, and this backend is one of the two producers that has to
// fill them. Before that the only receipt time on this path was `last_state_ns_`, which the JOINT
// callback stamps — so a hand whose joints keep arriving while the sensor stream stops looked fresh
// by every timestamp a controller could reach, and an old force read as a new
// contact.
//
// The packing assertions below are not the point but are kept alongside: they
// are what makes the receipt assertions meaningful (a stamp on a lane that
// decoded nothing would prove nothing), and this backend had no test file at
// all before, so its sensor-lane contract was pinned only indirectly through
// the demo controllers that read it.
//
// The mirror-image suite for the other producer is
// test_mujoco_native_backend.cpp; the seam BETWEEN a backend and a controller
// (the DeviceStateCache → DeviceState copy) is spanned by
// rtc_controller_manager's test_rt_loop_pipeline.
//
// The command-lane suite at the bottom pins the feedforward field: a
// pd_feedforward command carries slot.feedforward (N·m) name-aligned with
// values, every other mode carries zeros of the same length (the driver treats
// a same-length zero vector as "no feedforward"), and the backend declares
// both modes to the controller manager's pairing check.

#include "integrated_bringup/backends/udp_hand_native_backend.hpp"
#include <rtc_msgs/msg/hand_sensor_state.hpp>
#include <rtc_msgs/msg/joint_command.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace {

using namespace std::chrono_literals;

constexpr const char* kGroupName = "test_hand";
constexpr const char* kSensorTopic = "/test_udp_hand/sensor_state";
constexpr int kNumFingertips = 4;
// The shipped p1b layout: 8 barometers + 3 ToF per fingertip, and the
// 7-value inference block (contact, f[3], u[3]) the decode path requires.
constexpr int kPrimaryPerGroup = 8;
constexpr int kSecondaryPerGroup = 3;
constexpr int kInferPerGroup = 7;

class UdpHandNativeBackendTest : public ::testing::Test {
 protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_udp_hand_backend");
    state_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rtc::DeviceSensorLayout layout;
    layout.primary_count_per_group = kPrimaryPerGroup;
    layout.secondary_count_per_group = kSecondaryPerGroup;
    layout.values_per_group = kPrimaryPerGroup + kSecondaryPerGroup;
    layout.inference_values_per_group = kInferPerGroup;

    rtc::DeviceBackendConfig cfg;
    cfg.group_name = kGroupName;
    cfg.type = "udp_hand_native";
    cfg.sensor_topic = kSensorTopic;

    backend_ = std::make_unique<rtc::UdpHandNativeBackend>();
    // Layout BEFORE Configure — the header states that order, and OnSensorState
    // returns early without one, which would make every assertion here vacuous.
    backend_->SetSensorLayout(layout);
    backend_->Configure(node_.get(), cfg, state_cb_group_);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());

    rclcpp::QoS qos{1};
    qos.best_effort();
    sensor_pub_ = node_->create_publisher<rtc_msgs::msg::HandSensorState>(kSensorTopic, qos);
  }

  void TearDown() override {
    sensor_pub_.reset();
    executor_->remove_node(node_->get_node_base_interface());
    executor_.reset();
    backend_.reset();
    node_.reset();
  }

  /// One message carrying `n_tips` fingertips. `enable_mask` bit f sets that
  /// fingertip's `inference_enable`; the force is f-dependent so a mis-indexed
  /// decode is visible rather than plausible.
  void PublishSensorState(int n_tips, unsigned enable_mask) {
    rtc_msgs::msg::HandSensorState msg;
    msg.header.stamp = node_->now();
    msg.fingertips.resize(static_cast<std::size_t>(n_tips));
    for (int f = 0; f < n_tips; ++f) {
      auto& ft = msg.fingertips[static_cast<std::size_t>(f)];
      ft.inference_enable = ((enable_mask >> static_cast<unsigned>(f)) & 1U) != 0U;
      ft.contact_flag = static_cast<float>(f) + 0.5F;
      for (int j = 0; j < 3; ++j) {
        ft.f[static_cast<std::size_t>(j)] = static_cast<float>((f * 10) + j);
        ft.u[static_cast<std::size_t>(j)] = static_cast<float>((f * 100) + j);
      }
      for (int b = 0; b < kPrimaryPerGroup; ++b) {
        ft.barometer[static_cast<std::size_t>(b)] = static_cast<float>((f * 8) + b);
      }
    }
    sensor_pub_->publish(msg);
    SpinUntilDelivered();
  }

  void SpinUntilDelivered() {
    const auto deadline = std::chrono::steady_clock::now() + 200ms;
    for (int i = 0; i < 20; ++i) {
      executor_->spin_some(20ms);
      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }
    }
  }

  static int64_t SteadyNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr state_cb_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<rtc::UdpHandNativeBackend> backend_;
  rclcpp::Publisher<rtc_msgs::msg::HandSensorState>::SharedPtr sensor_pub_;
};

TEST_F(UdpHandNativeBackendTest, BeforeAnyMessage_ClaimsNoReceipt) {
  rtc::DeviceStateCache cache{};
  backend_->ReadSensorState(cache);
  EXPECT_EQ(cache.num_inference_groups, 0);
  for (std::size_t i = 0; i < cache.inference_recv_steady_ns.size(); ++i) {
    // 0 = never received. A consumer reads that as "no claim" and withholds
    // the lane, rather than computing an age against the steady epoch.
    EXPECT_EQ(cache.inference_recv_steady_ns[i], 0) << "fingertip " << i;
    EXPECT_EQ(cache.inference_sequence[i], 0U) << "fingertip " << i;
  }
}

TEST_F(UdpHandNativeBackendTest, DecodesInferenceBlockPerFingertip) {
  PublishSensorState(kNumFingertips, /*enable_mask=*/0xFU);

  rtc::DeviceStateCache cache{};
  backend_->ReadSensorState(cache);

  ASSERT_EQ(cache.num_inference_groups, kNumFingertips);
  for (int f = 0; f < kNumFingertips; ++f) {
    const auto base = static_cast<std::size_t>(f * kInferPerGroup);
    EXPECT_TRUE(cache.inference_enable[static_cast<std::size_t>(f)]) << "f=" << f;
    EXPECT_FLOAT_EQ(cache.inference_data[base], static_cast<float>(f) + 0.5F);
    EXPECT_FLOAT_EQ(cache.inference_data[base + 1], static_cast<float>(f * 10));
    EXPECT_FLOAT_EQ(cache.inference_data[base + 4], static_cast<float>(f * 100));
  }
}

TEST_F(UdpHandNativeBackendTest, StampsReceiptAndAdvancesSequencePerGroup) {
  const int64_t before = SteadyNowNs();
  PublishSensorState(kNumFingertips, /*enable_mask=*/0xFU);
  const int64_t after = SteadyNowNs();

  rtc::DeviceStateCache first{};
  backend_->ReadSensorState(first);
  for (int f = 0; f < kNumFingertips; ++f) {
    const auto fu = static_cast<std::size_t>(f);
    // Bracketed by the clock the field claims to be on. A stamp from ROS time
    // or the message header would fall outside this window — and `header.stamp`
    // is specifically not what freshness is judged on in this repo.
    EXPECT_GE(first.inference_recv_steady_ns[fu], before) << "f=" << f;
    EXPECT_LE(first.inference_recv_steady_ns[fu], after) << "f=" << f;
    EXPECT_EQ(first.inference_sequence[fu], 1U) << "f=" << f;
  }

  PublishSensorState(kNumFingertips, /*enable_mask=*/0xFU);
  rtc::DeviceStateCache second{};
  backend_->ReadSensorState(second);
  for (int f = 0; f < kNumFingertips; ++f) {
    const auto fu = static_cast<std::size_t>(f);
    EXPECT_EQ(second.inference_sequence[fu], 2U) << "f=" << f;
    EXPECT_GE(second.inference_recv_steady_ns[fu], first.inference_recv_steady_ns[fu]) << "f=" << f;
  }
}

TEST_F(UdpHandNativeBackendTest, DisabledFingertipStillRecordsReceipt) {
  // Every fingertip arrives; only fingertip 1 carries a usable estimate.
  PublishSensorState(kNumFingertips, /*enable_mask=*/0x2U);

  rtc::DeviceStateCache cache{};
  backend_->ReadSensorState(cache);

  EXPECT_FALSE(cache.inference_enable[0]);
  EXPECT_TRUE(cache.inference_enable[1]);
  // The two fields answer different questions: the flag is the firmware's
  // verdict on the VALUE, the stamp is when a sample last ARRIVED. Folding
  // them would make a live-but-disabled tip indistinguishable from a dead
  // lane, which is the confusion the stamp exists to remove — so a disabled tip
  // still records its receipt.
  EXPECT_GT(cache.inference_recv_steady_ns[0], 0);
  EXPECT_EQ(cache.inference_sequence[0], 1U);
}

TEST_F(UdpHandNativeBackendTest, ShortMessageLeavesAbsentFingertipsUntouched) {
  PublishSensorState(kNumFingertips, /*enable_mask=*/0xFU);
  rtc::DeviceStateCache full{};
  backend_->ReadSensorState(full);
  ASSERT_EQ(full.inference_sequence[3], 1U);

  // A message that stops reporting the last fingertip — the per-finger quiet
  // lane.
  PublishSensorState(/*n_tips=*/2, /*enable_mask=*/0x3U);
  rtc::DeviceStateCache partial{};
  backend_->ReadSensorState(partial);

  EXPECT_EQ(partial.inference_sequence[0], 2U);
  EXPECT_EQ(partial.inference_sequence[1], 2U);
  // Fingertip 3's counter and stamp stay where the last message that carried
  // it left them. That is what makes its age GROW tick after tick, which is
  // the only signal a consumer has that this finger went quiet: the cached
  // force is still there and `inference_enable` still reads true, because
  // this backend copies the firmware's bit and no bit arrived to clear it.
  EXPECT_EQ(partial.inference_sequence[3], full.inference_sequence[3]);
  EXPECT_EQ(partial.inference_recv_steady_ns[3], full.inference_recv_steady_ns[3]);
}

// ═══════════════════════════════════════════════════════════════════════════
// Command lane — WriteCommand
// ═══════════════════════════════════════════════════════════════════════════

const std::vector<std::string> kCmdNames = {"j_a", "j_b", "j_c", "j_d"};

class UdpHandBackendWriteCommandTest : public ::testing::Test {
 protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_udp_hand_backend_cmd");
    state_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rtc::DeviceBackendConfig cfg;
    cfg.group_name = "test_hand_cmd";
    cfg.type = "udp_hand_native";
    cfg.state_topic = "/test_udp_hand_cmd/joint_states";
    cfg.command_topic = "/test_udp_hand_cmd/joint_command";
    cfg.joint_command_names = kCmdNames;
    backend_ = std::make_unique<rtc::UdpHandNativeBackend>();
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
  std::unique_ptr<rtc::UdpHandNativeBackend> backend_;
  rclcpp::Subscription<rtc_msgs::msg::JointCommand>::SharedPtr cmd_sub_;
  std::vector<rtc_msgs::msg::JointCommand> received_;
};

// A position command carries values only: feedforward goes out as a
// same-length ZERO vector (what the driver reads as "no feedforward"), never
// as the slot's content and never empty.
TEST_F(UdpHandBackendWriteCommandTest, PositionMode_ZeroesFeedforwardAtCommandWidth) {
  rtc::PublishSnapshot::GroupCommandSlot slot{};
  slot.num_channels = static_cast<int>(kCmdNames.size());
  const std::vector<double> cmds = {0.1, -0.2, 0.3, -0.4};
  for (std::size_t i = 0; i < cmds.size(); ++i) {
    slot.commands[i] = cmds[i];
    slot.feedforward[i] = 100.0 + static_cast<double>(i);  // must NOT leak out
  }

  const auto msg = WriteAndReceive(slot, rtc::CommandType::kPosition);

  EXPECT_EQ(msg.command_type, "position");
  EXPECT_EQ(msg.joint_names, kCmdNames);
  ASSERT_EQ(msg.values.size(), kCmdNames.size());
  ASSERT_EQ(msg.feedforward.size(), kCmdNames.size());
  for (std::size_t i = 0; i < kCmdNames.size(); ++i) {
    EXPECT_DOUBLE_EQ(msg.values[i], cmds[i]) << "i=" << i;
    EXPECT_DOUBLE_EQ(msg.feedforward[i], 0.0) << "i=" << i;
  }
}

// A pd_feedforward command carries slot.feedforward index-aligned with values
// (the driver re-orders both by joint_names, so the alignment here is what
// keeps a torque on the joint it was computed for).
TEST_F(UdpHandBackendWriteCommandTest, PdFeedforwardMode_CopiesFeedforward) {
  rtc::PublishSnapshot::GroupCommandSlot slot{};
  slot.num_channels = static_cast<int>(kCmdNames.size());
  const std::vector<double> cmds = {1.0, 2.0, 3.0, 4.0};
  const std::vector<double> ffs = {-0.05, 0.025, -0.0125, 0.00625};
  for (std::size_t i = 0; i < cmds.size(); ++i) {
    slot.commands[i] = cmds[i];
    slot.feedforward[i] = ffs[i];
  }

  const auto msg = WriteAndReceive(slot, rtc::CommandType::kPdFeedforward);

  EXPECT_EQ(msg.command_type, "pd_feedforward");
  ASSERT_EQ(msg.values.size(), kCmdNames.size());
  ASSERT_EQ(msg.feedforward.size(), kCmdNames.size());
  for (std::size_t i = 0; i < kCmdNames.size(); ++i) {
    EXPECT_DOUBLE_EQ(msg.values[i], cmds[i]) << "i=" << i;
    EXPECT_DOUBLE_EQ(msg.feedforward[i], ffs[i]) << "i=" << i;
  }
}

// Switching back to a position command after a pd_feedforward one clears the
// field: the message object is reused across ticks, so a stale torque would
// otherwise ride on every later position command.
TEST_F(UdpHandBackendWriteCommandTest, PositionAfterPdFeedforward_ClearsStaleFeedforward) {
  rtc::PublishSnapshot::GroupCommandSlot slot{};
  slot.num_channels = static_cast<int>(kCmdNames.size());
  for (std::size_t i = 0; i < kCmdNames.size(); ++i) {
    slot.commands[i] = 0.5;
    slot.feedforward[i] = 0.07;
  }
  (void)WriteAndReceive(slot, rtc::CommandType::kPdFeedforward);
  const auto msg = WriteAndReceive(slot, rtc::CommandType::kPosition);
  ASSERT_EQ(msg.feedforward.size(), kCmdNames.size());
  for (std::size_t i = 0; i < kCmdNames.size(); ++i) {
    EXPECT_DOUBLE_EQ(msg.feedforward[i], 0.0) << "i=" << i;
  }
}

// The safe output replays the last position target but never the last torque:
// a feedforward computed for an earlier pose must not be held open-loop on the
// tick the controller manager has no honest command.
TEST_F(UdpHandBackendWriteCommandTest, SafeCommand_ReplaysPositionWithZeroFeedforward) {
  rtc::PublishSnapshot::GroupCommandSlot slot{};
  slot.num_channels = static_cast<int>(kCmdNames.size());
  for (std::size_t i = 0; i < kCmdNames.size(); ++i) {
    slot.commands[i] = 0.3 + static_cast<double>(i);
    slot.feedforward[i] = 0.07;
  }
  (void)WriteAndReceive(slot, rtc::CommandType::kPdFeedforward);
  received_.clear();
  backend_->WriteSafeCommand();
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (received_.empty() && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some(20ms);
  }
  ASSERT_FALSE(received_.empty());
  const auto& msg = received_.back();
  ASSERT_EQ(msg.values.size(), kCmdNames.size());
  ASSERT_EQ(msg.feedforward.size(), kCmdNames.size());
  for (std::size_t i = 0; i < kCmdNames.size(); ++i) {
    EXPECT_DOUBLE_EQ(msg.values[i], 0.3 + static_cast<double>(i)) << "i=" << i;
    EXPECT_DOUBLE_EQ(msg.feedforward[i], 0.0) << "i=" << i;
  }
}

// The pairing check: both position-bearing modes are declared, torque is not.
TEST(UdpHandNativeBackendCommandTypes, AcceptsPositionAndPdFeedforwardOnly) {
  rtc::UdpHandNativeBackend backend;
  EXPECT_TRUE(backend.AcceptsCommandType(rtc::CommandType::kPosition));
  EXPECT_TRUE(backend.AcceptsCommandType(rtc::CommandType::kPdFeedforward));
  EXPECT_FALSE(backend.AcceptsCommandType(rtc::CommandType::kTorque));
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
