// Tests for MujocoNativeBackend's fingertip wrench lane (Layer C).
//
// These tests pin the contract the WBC / Joint controllers rely on:
//   - HasSensorState() reports true (sensor lane is now owned by mujoco)
//   - inference_data slots 1..3 carry fx/fy/fz, slots 0 / 4..6 are zeroed
//   - stale detection trips inference_enable[f] after
//     max_consecutive_missed_ticks empty RT ticks, but the last force
//     values are preserved so the controller can keep using them
//   - NaN/Inf wrench messages are dropped without disturbing the mirror

#include <gtest/gtest.h>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "integrated_bringup/backends/mujoco_native_backend.hpp"
#include "rtc_controller_manager/device_backend.hpp"
#include "rtc_controller_manager/device_state_cache.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace {

using namespace std::chrono_literals;

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
    state_cb_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

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
      wrench_pubs_.push_back(
          node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
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
  EXPECT_FLOAT_EQ(cache.inference_data[0 * 7 + 1], 1.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[1 * 7 + 2], 2.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[2 * 7 + 3], 3.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[3 * 7 + 1], 4.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[3 * 7 + 2], 5.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[3 * 7 + 3], 6.0F);
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
  EXPECT_FLOAT_EQ(cache.inference_data[1 * 7 + 0], 0.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[1 * 7 + 4], 0.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[1 * 7 + 5], 0.0F);
  EXPECT_FLOAT_EQ(cache.inference_data[1 * 7 + 6], 0.0F);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
