// Tier 1 — /rtc_cm/clear_estop service contract (issue #288).
//
// The refusal paths are the point of this file. ClearGlobalEstop's own
// behaviour (ordering, the swallowed-trigger refusal) is covered in
// test_estop.cpp by calling it directly; what only a real service round-trip
// can show is which answer the operator gets, and the answers are the whole
// design: an unconfirmed clear, a mismatched acknowledgement, and a latch that
// is down but unverified must not read alike.
//
// No RT loop runs here. That is deliberate for the last case — "the loop
// completed no tick" is a real operating condition (stalled, overrunning, or
// deactivated) and it is the one answer that differs structurally from
// /rtc_cm/reset_fault's, because here the latch really is down by the time the
// reply is written.

#include "rt_cm_test_access.hpp"
#include <rtc_msgs/srv/clear_estop.hpp>

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rtc {
namespace {

class ClearEstopServiceTest : public ::testing::Test {
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
    node_ = std::make_shared<RtControllerNode>("test_clear_estop_node");

    std::vector<std::unique_ptr<RTControllerInterface>> ctrls;
    auto a = std::make_unique<MockController>("ctrl_a");
    ctrl_a_ = a.get();
    ctrls.push_back(std::move(a));
    ControllerLifecycleTestAccess::InjectControllers(*node_, std::move(ctrls), {"type_a"});
    ControllerLifecycleTestAccess::EnsureEstopPublisher(*node_);
    ControllerLifecycleTestAccess::SetActiveIdx(*node_, 0);
    ControllerLifecycleTestAccess::BringServicesOnline(*node_);

    client_node_ = std::make_shared<rclcpp::Node>("test_clear_estop_client");
    client_ = client_node_->create_client<rtc_msgs::srv::ClearEstop>("/rtc_cm/clear_estop");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    ASSERT_TRUE(client_->wait_for_service(std::chrono::seconds(2)))
        << "clear_estop service did not appear";
  }

  void TearDown() override {
    if (executor_) {
      executor_->cancel();
      executor_->remove_node(client_node_);
      executor_->remove_node(node_->get_node_base_interface());
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  // The service waits on RtTickCount for tens of periods before giving up, so
  // the client deadline has to outlast that — a client timeout here would look
  // exactly like a hung service.
  rtc_msgs::srv::ClearEstop::Response::SharedPtr CallClear(const std::string& ack) {
    auto req = std::make_shared<rtc_msgs::srv::ClearEstop::Request>();
    req->reason_ack = ack;
    auto fut = client_->async_send_request(req);
    if (fut.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      return nullptr;
    }
    return fut.get();
  }

  std::shared_ptr<RtControllerNode> node_;
  std::shared_ptr<rclcpp::Node> client_node_;
  rclcpp::Client<rtc_msgs::srv::ClearEstop>::SharedPtr client_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  MockController* ctrl_a_{nullptr};
};

TEST_F(ClearEstopServiceTest, NoOpWhenNothingIsLatched) {
  auto resp = CallClear("");
  ASSERT_NE(nullptr, resp);
  EXPECT_TRUE(resp->ok) << resp->message;
  EXPECT_NE(std::string::npos, resp->message.find("not latched")) << resp->message;
  EXPECT_EQ(0, ctrl_a_->ClearEstopCount());
}

TEST_F(ClearEstopServiceTest, RefusesAnEmptyAckAndHandsBackTheReasonToEcho) {
  ControllerLifecycleTestAccess::CallTriggerEstop(*node_, "arm_timeout");

  auto resp = CallClear("");
  ASSERT_NE(nullptr, resp);
  EXPECT_FALSE(resp->ok);
  // The refusal is also the discovery path — it has to name the string the
  // caller is expected to echo, or the confirmation step is unusable.
  EXPECT_NE(std::string::npos, resp->message.find("arm_timeout")) << resp->message;
  EXPECT_TRUE(ControllerLifecycleTestAccess::IsEstopped(*node_));
  EXPECT_EQ(0, ctrl_a_->ClearEstopCount());
}

TEST_F(ClearEstopServiceTest, RefusesAMismatchedAckWithoutTouchingAnything) {
  ControllerLifecycleTestAccess::CallTriggerEstop(*node_, "arm_timeout");

  auto resp = CallClear("hand_timeout");
  ASSERT_NE(nullptr, resp);
  EXPECT_FALSE(resp->ok);
  EXPECT_TRUE(ControllerLifecycleTestAccess::IsEstopped(*node_));
  EXPECT_EQ(0, ctrl_a_->ClearEstopCount());
}

TEST_F(ClearEstopServiceTest, MatchingAckClearsButReportsUnverifiedWithNoRtLoop) {
  ControllerLifecycleTestAccess::CallTriggerEstop(*node_, "arm_timeout");

  const auto t0 = std::chrono::steady_clock::now();
  auto resp = CallClear("arm_timeout");
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  ASSERT_NE(nullptr, resp);

  // Pins the SIZE of the observation window, which no assertion on the reply
  // can reach: with no tick ever arriving the service waits out its whole
  // deadline, so the call's duration IS the window. At the default 500 Hz that
  // is (500/50 watchdog divisor + 2) × 4 × 2 ms ≈ 96 ms, where reset_fault's
  // two-tick proof would have been 2 × 4 × 2 ms = 16 ms. The floor sits between
  // the two rather than near either, so ordinary scheduling noise cannot move
  // it but shrinking the window back to a tick count would.
  EXPECT_GT(elapsed, std::chrono::milliseconds(40))
      << "observation window collapsed — a dead device would be reported recovered";

  // The latch IS down — the clear happened. What is missing is the
  // confirmation, because no tick ran to let a cause detector re-latch.
  EXPECT_FALSE(ControllerLifecycleTestAccess::IsEstopped(*node_));
  EXPECT_EQ(1, ctrl_a_->ClearEstopCount());
  EXPECT_FALSE(ctrl_a_->HandEstop());

  // ok=false, but the message must say the latch is down. An operator who read
  // this as reset_fault's "nothing was touched" would re-issue the call and
  // then be surprised by an unlatched arm.
  EXPECT_FALSE(resp->ok);
  EXPECT_NE(std::string::npos, resp->message.find("DOWN")) << resp->message;
  EXPECT_NE(std::string::npos, resp->message.find("arm_timeout")) << resp->message;
}

}  // namespace
}  // namespace rtc
