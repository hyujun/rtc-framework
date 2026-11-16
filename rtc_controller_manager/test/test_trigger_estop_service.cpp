// Tier 1 — /rtc_cm/trigger_estop service contract.
//
// The operator's software E-STOP. What only a service round-trip can show is
// the REPLY: `success` is the state of the latch after the call, `message`
// says whether this call raised it or found it up, and the two must not read
// alike — an operator who is told "raised" when a detector had already
// stopped the arm would clear the wrong reason. The latch itself
// (TriggerGlobalEstop's CAS, propagation, the swallowed-trigger counter) is
// pinned in test_estop.cpp; its hold on the wire and the behaviour inside a
// clear-verification window in test_estop_hold_latch.cpp.
//
// No RT loop runs here: the trigger needs no tick to take effect, and the one
// clear in this file is only asked to ACCEPT the operator reason (it then waits
// out its window and reports the latch down, unverified — the shape
// test_clear_estop_service pins).

#include "rclcpp_suite_fixture.hpp"
#include "rt_cm_log_sink.hpp"
#include "rt_cm_test_access.hpp"
#include <rtc_msgs/srv/clear_estop.hpp>

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <rcutils/logging.h>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rtc {
namespace {

using Access = ControllerLifecycleTestAccess;

class TriggerEstopServiceTest : public RclcppSuiteTest {
 protected:
  void SetUp() override {
    LogSink::Install();
    node_ = std::make_shared<RtControllerNode>("test_trigger_estop_node");

    std::vector<std::unique_ptr<RTControllerInterface>> ctrls;
    auto a = std::make_unique<MockController>("ctrl_a");
    ctrl_a_ = a.get();
    ctrls.push_back(std::move(a));
    Access::InjectControllers(*node_, std::move(ctrls), {"type_a"});
    Access::EnsureEstopPublisher(*node_);
    Access::SetActiveIdx(*node_, 0);
    Access::BringServicesOnline(*node_);

    client_node_ = std::make_shared<rclcpp::Node>("test_trigger_estop_client");
    trigger_ = client_node_->create_client<std_srvs::srv::Trigger>("/rtc_cm/trigger_estop");
    clear_ = client_node_->create_client<rtc_msgs::srv::ClearEstop>("/rtc_cm/clear_estop");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    ASSERT_TRUE(trigger_->wait_for_service(std::chrono::seconds(2)))
        << "trigger_estop service did not appear";
    ASSERT_TRUE(clear_->wait_for_service(std::chrono::seconds(2)));
  }

  void TearDown() override {
    if (executor_) {
      StopSpinner(*executor_, spin_thread_);
      executor_->remove_node(client_node_);
      executor_->remove_node(node_->get_node_base_interface());
    }
    LogSink::Restore();
  }

  std_srvs::srv::Trigger::Response::SharedPtr CallTrigger() {
    return CallService(*trigger_, std::make_shared<std_srvs::srv::Trigger::Request>());
  }

  rtc_msgs::srv::ClearEstop::Response::SharedPtr CallClear(const std::string& ack) {
    auto req = std::make_shared<rtc_msgs::srv::ClearEstop::Request>();
    req->reason_ack = ack;
    return CallService(*clear_, std::move(req));
  }

  std::shared_ptr<RtControllerNode> node_;
  std::shared_ptr<rclcpp::Node> client_node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_;
  rclcpp::Client<rtc_msgs::srv::ClearEstop>::SharedPtr clear_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  MockController* ctrl_a_{nullptr};
};

const std::string kOperatorReason(Access::kOperatorEstopReason);

TEST_F(TriggerEstopServiceTest, RaisesTheLatchAndTheOperatorReasonClearsIt) {
  ASSERT_FALSE(Access::IsEstopped(*node_));
  const auto resp = CallTrigger();
  ASSERT_NE(nullptr, resp);
  EXPECT_TRUE(resp->success) << resp->message;
  EXPECT_NE(std::string::npos, resp->message.find("raised by this call")) << resp->message;
  EXPECT_NE(std::string::npos, resp->message.find(kOperatorReason)) << resp->message;

  EXPECT_TRUE(Access::IsEstopped(*node_));
  EXPECT_TRUE(Access::EstopStatusValue(*node_));
  EXPECT_EQ(Access::GetGlobalEstopReason(*node_), kOperatorReason);
  // Propagated once, like every other trigger.
  EXPECT_EQ(1, ctrl_a_->TriggerEstopCount());
  EXPECT_TRUE(ctrl_a_->HandEstop());

  // The reason the reply quoted is the acknowledgement the clear accepts.
  const auto cleared = CallClear(kOperatorReason);
  ASSERT_NE(nullptr, cleared);
  EXPECT_FALSE(Access::IsEstopped(*node_)) << "the operator reason was not accepted as the ack";
  EXPECT_EQ(1, ctrl_a_->ClearEstopCount());
  // No tick ran: the latch is down but unverified — the clear service's shape.
  EXPECT_FALSE(cleared->ok);
  EXPECT_NE(std::string::npos, cleared->message.find("DOWN")) << cleared->message;
}

TEST_F(TriggerEstopServiceTest, AnAlreadyLatchedEstopIsReportedWithItsReasonUnchanged) {
  Access::CallTriggerEstop(*node_, "arm_timeout");
  ASSERT_EQ(1, ctrl_a_->TriggerEstopCount());

  const auto resp = CallTrigger();
  ASSERT_NE(nullptr, resp);
  // The latch stands, so the call "succeeded" — the arm is stopped. What it
  // did not do is raise it, and the message must say which.
  EXPECT_TRUE(resp->success) << resp->message;
  EXPECT_NE(std::string::npos, resp->message.find("already latched: arm_timeout")) << resp->message;
  EXPECT_EQ(std::string::npos, resp->message.find("raised by this call")) << resp->message;

  // Idempotent: the detector's reason is the one the operator must echo, not
  // ours; nothing was propagated a second time.
  EXPECT_EQ(Access::GetGlobalEstopReason(*node_), "arm_timeout");
  EXPECT_EQ(1, ctrl_a_->TriggerEstopCount());
  EXPECT_TRUE(Access::IsEstopped(*node_));

  // One WARN line, so a flood of operator presses is visible but not noisy.
  const auto warns = LogSink::Matching(RCUTILS_LOG_SEVERITY_WARN, {"already latched"});
  EXPECT_EQ(1U, warns.size());
}

TEST_F(TriggerEstopServiceTest, ASecondPressWhileLatchedByTheOperatorNamesTheOperatorReason) {
  ASSERT_NE(nullptr, CallTrigger());
  const auto resp = CallTrigger();
  ASSERT_NE(nullptr, resp);
  EXPECT_TRUE(resp->success);
  EXPECT_NE(std::string::npos, resp->message.find("already latched: " + kOperatorReason))
      << resp->message;
  EXPECT_EQ(1, ctrl_a_->TriggerEstopCount());
}

// Many callers at once — the race the CAS exists for. Exactly one of them
// raised it (measured on the CAS result, not on a pre-test), and the
// controllers heard about it exactly once.
TEST_F(TriggerEstopServiceTest, ConcurrentTriggersRaiseOnceAndPropagateOnce) {
  constexpr int kThreads = 8;
  std::atomic<int> raised{0};
  std::atomic<bool> go{false};
  std::vector<std::thread> threads;
  threads.reserve(kThreads);
  for (int i = 0; i < kThreads; ++i) {
    threads.emplace_back([this, &raised, &go]() {
      while (!go.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      if (Access::CallTriggerEstopRaised(*node_, Access::kOperatorEstopReason)) {
        raised.fetch_add(1, std::memory_order_relaxed);
      }
    });
  }
  go.store(true, std::memory_order_release);
  for (auto& t : threads) {
    t.join();
  }
  EXPECT_EQ(1, raised.load());
  EXPECT_EQ(1, ctrl_a_->TriggerEstopCount());
  EXPECT_EQ(static_cast<std::uint64_t>(kThreads), Access::GetEstopTriggerRequestCount(*node_))
      << "every entry is counted, including the ones the CAS turned away";
  EXPECT_TRUE(Access::IsEstopped(*node_));
}

}  // namespace
}  // namespace rtc
