// Node-level — this node's /system/estop_status subscription connects to the
// CM's publisher (issue #588, decisions Q3 · Q14).
//
// The CM published E-STOP VOLATILE while this node has always subscribed
// TRANSIENT_LOCAL. DDS matches a transient_local reader only with a
// transient_local writer, so the two never connected and ExploreLoopCallback's
// E-STOP branch never saw a message. The CM publisher is transient_local now;
// the second case keeps the old volatile writer as the positive control.
//
// What this does NOT cover: the abort itself. On the node's own spin (one
// SingleThreadedExecutor, as main() does) the /shape/explore accept handler
// blocks on the /rtc_cm/switch_controller reply, which only that same thread
// could deliver, so the exploration never starts and the E-STOP branch is
// unreachable for a reason unrelated to QoS (found while writing this test,
// #588 R2). "Matched" is therefore the whole of what the QoS change can be
// shown to do from here.

#include "shape_estimation/shape_estimation_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

namespace {

using namespace std::chrono_literals;

constexpr const char* kEstopTopic = "/system/estop_status";

class EstopSubscriptionTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      // The subscription is created with the exploration path, which exists
      // only with enable_exploration (default false). The node's constructor
      // takes no NodeOptions, so the override goes in as a global parameter.
      const char* argv[] = {"test_estop_subscription", "--ros-args", "-p",
                            "enable_exploration:=true"};
      rclcpp::init(4, argv);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  // The stand-in CM's E-STOP writer, with E-STOP already latched true before
  // the node under test exists — the late-join case the durability is for.
  void BringUp(const rclcpp::QoS& estop_qos) {
    cm_ = std::make_shared<rclcpp::Node>("estop_subscription_cm");
    estop_pub_ = cm_->create_publisher<std_msgs::msg::Bool>(kEstopTopic, estop_qos);
    std_msgs::msg::Bool estop;
    estop.data = true;
    estop_pub_->publish(estop);

    shape_ = std::make_shared<shape_estimation::ShapeEstimationNode>();
    ASSERT_EQ(shape_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(shape_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(cm_);
    exec_->add_node(shape_->get_node_base_interface());
    spin_thread_ = std::thread([this]() { exec_->spin(); });

    // The graph sees the node's reader whatever its QoS — so a matched count
    // of 0 below means "incompatible", not "not there yet".
    ASSERT_TRUE(WaitFor([this] { return cm_->count_subscribers(kEstopTopic) == 1U; }, 5s))
        << "the node never created its E-STOP subscription";
  }

  void TearDown() override {
    if (exec_) {
      exec_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    shape_.reset();
  }

  static bool WaitFor(const std::function<bool()>& pred, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return pred();
  }

  std::shared_ptr<rclcpp::Node> cm_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  std::shared_ptr<shape_estimation::ShapeEstimationNode> shape_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::thread spin_thread_;
};

TEST_F(EstopSubscriptionTest, TheSubscriptionMatchesTheCmTransientLocalPublisher) {
  // The CM's publisher QoS (rt_controller_node_publishers.cpp).
  rclcpp::QoS cm_qos{1};
  cm_qos.transient_local();
  ASSERT_NO_FATAL_FAILURE(BringUp(cm_qos));
  EXPECT_TRUE(WaitFor([this] { return estop_pub_->get_subscription_count() == 1U; }, 5s))
      << "the node's transient_local reader did not match the CM writer";
}

TEST_F(EstopSubscriptionTest, AVolatilePublisherNeverMatches) {
  // Positive control: the pre-#588 writer. Present on the graph, never matched.
  ASSERT_NO_FATAL_FAILURE(BringUp(rclcpp::QoS(1)));
  EXPECT_FALSE(WaitFor([this] { return estop_pub_->get_subscription_count() > 0U; }, 1500ms))
      << "a volatile writer matched a transient_local reader — this control is vacuous";
}

}  // namespace
