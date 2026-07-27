// ── /rtc_cm/reset_fault service tests (issue #260) ──────────────────────────
//
// Drives the service through the wire, like test_switch_service.cpp, and pins
// the half of the recovery path the CM owns:
//   - controller_name is REQUIRED and must name the ACTIVE controller. The
//     naming is the operator confirmation step (E-8), so an empty request is a
//     refusal, not a convenience default, and there is no wildcard.
//   - A non-active target is refused rather than queued — a pending request on
//     an inactive controller would be consumed on its next activation, which is
//     the laundering the controller side also refuses.
//   - The response reports what HAPPENED, not what was sent: cleared, no-op,
//     or re-latched because the fault cause is still present.
//   - E-8 separation in both directions: reset_fault does not clear the global
//     E-STOP, and a global E-STOP does not block or clear a controller fault.
//
// FaultyMockController deliberately consumes the reset request ONLY inside
// Compute(), the way the real compliance controllers do, and the fixture runs a
// tick thread to supply those Compute() calls. Clearing the latch inside
// ResetFault() itself would make the service's wait for an RT tick untestable —
// a service that never waited would pass just as well.
#include "rtc_controller_manager/rt_controller_node.hpp"
#include <rtc_msgs/srv/reset_fault.hpp>

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <span>
#include <string>
#include <thread>
#include <vector>

namespace rtc {

// A controller with a latch that behaves like ComplianceStateMachine's
// SAFE_STOP: entered by a critical cause, never left by any input other than a
// reset, and re-entered on the same tick if the cause is still present.
class FaultyMockController : public RTControllerInterface {
 public:
  explicit FaultyMockController(std::string name) : name_(std::move(name)) {}

  CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) noexcept override {
    return RTControllerInterface::on_activate(prev);
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*prev*/) noexcept override {
    return CallbackReturn::SUCCESS;
  }

  // The RT side. Consumes the reset request edge first (mirroring the real
  // controllers, which do it at the head of Compute() ahead of their E-STOP
  // early return), then re-latches if the cause persists.
  ControllerOutput Compute(const ControllerState& /*state*/) noexcept override {
    if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
      latched_.store(false, std::memory_order_release);
    }
    if (cause_present_.load(std::memory_order_acquire)) {
      latched_.store(true, std::memory_order_release);
    }
    ticks_.fetch_add(1, std::memory_order_relaxed);
    return ControllerOutput{};
  }

  void SetDeviceTarget(int /*device_idx*/, std::span<const double> /*target*/) noexcept override {}

  std::string_view Name() const noexcept override { return name_; }

  void TriggerEstop() noexcept override { estopped_.store(true, std::memory_order_release); }

  void ClearEstop() noexcept override { estopped_.store(false, std::memory_order_release); }

  bool IsEstopped() const noexcept override { return estopped_.load(std::memory_order_acquire); }

  void ResetFault() noexcept override { reset_requested_.store(true, std::memory_order_release); }

  bool HasLatchedFault() const noexcept override {
    return latched_.load(std::memory_order_acquire);
  }

  // ── Test drivers ────────────────────────────────────────────────────────
  void LatchFault() { latched_.store(true, std::memory_order_release); }

  void SetCausePresent(bool v) { cause_present_.store(v, std::memory_order_release); }

  std::uint64_t Ticks() const { return ticks_.load(std::memory_order_relaxed); }

 private:
  std::string name_;
  std::atomic<bool> latched_{false};
  std::atomic<bool> cause_present_{false};
  std::atomic<bool> reset_requested_{false};
  std::atomic<bool> estopped_{false};
  std::atomic<std::uint64_t> ticks_{0};
};

// Friend bridge — distinct class per test binary, same pattern as
// test_switch_service.cpp (each gtest binary is its own TU).
class ControllerLifecycleTestAccess {
 public:
  static void InjectControllers(RtControllerNode& node,
                                std::vector<std::unique_ptr<RTControllerInterface>> ctrls,
                                const std::vector<std::string>& types) {
    node.controllers_ = std::move(ctrls);
    node.controller_states_ = std::vector<std::atomic<int>>(node.controllers_.size());
    node.controller_topic_configs_.assign(node.controllers_.size(), {});
    node.controller_slot_mappings_.assign(node.controllers_.size(), {});
    node.controller_name_to_idx_.clear();
    node.controller_types_.clear();
    for (std::size_t i = 0; i < node.controllers_.size(); ++i) {
      node.controller_name_to_idx_[std::string(node.controllers_[i]->Name())] = static_cast<int>(i);
      node.controller_types_.push_back(i < types.size() ? types[i] : "");
    }
  }

  static void SetActiveIdx(RtControllerNode& node, int idx) {
    node.active_controller_idx_.store(idx, std::memory_order_release);
  }

  static void SetState(RtControllerNode& node, std::size_t idx, int v) {
    node.controller_states_[idx].store(v, std::memory_order_release);
  }

  static void SetEstopFlag(RtControllerNode& node, bool v) {
    node.global_estop_.store(v, std::memory_order_release);
  }

  static bool IsEstopped(const RtControllerNode& node) { return node.IsGlobalEstopped(); }

  static double ControlRate(const RtControllerNode& node) { return node.control_rate_; }

  static void BringServicesOnline(RtControllerNode& node) {
    if (!node.cb_group_nrt_callback_) {
      node.cb_group_nrt_callback_ =
          node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    }
    node.CreateServices();
  }
};

// ── Fixture ─────────────────────────────────────────────────────────────

class ResetFaultServiceTest : public ::testing::Test {
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
    node_ = std::make_shared<RtControllerNode>("test_reset_fault_node");

    std::vector<std::unique_ptr<RTControllerInterface>> ctrls;
    auto a = std::make_unique<FaultyMockController>("ctrl_a");
    auto b = std::make_unique<FaultyMockController>("ctrl_b");
    ctrl_a_ = a.get();
    ctrl_b_ = b.get();
    ctrls.push_back(std::move(a));
    ctrls.push_back(std::move(b));

    ControllerLifecycleTestAccess::InjectControllers(*node_, std::move(ctrls),
                                                     {"type_a", "type_b"});
    ControllerLifecycleTestAccess::SetActiveIdx(*node_, 0);
    ControllerLifecycleTestAccess::SetState(*node_, 0, 1);  // ctrl_a active
    ControllerLifecycleTestAccess::BringServicesOnline(*node_);

    client_node_ = std::make_shared<rclcpp::Node>("test_reset_fault_client");
    reset_client_ = client_node_->create_client<rtc_msgs::srv::ResetFault>("/rtc_cm/reset_fault");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    // Stand-in for the RT loop: the service's outcome depends on a Compute()
    // running while it waits, so something has to supply those ticks. One
    // period at the CM's configured rate, the same clock the service sizes its
    // wait against.
    const double rate_hz = ControllerLifecycleTestAccess::ControlRate(*node_);
    tick_period_ = std::chrono::microseconds(static_cast<long>(1'000'000.0 / rate_hz));
    ticking_.store(true, std::memory_order_release);
    tick_thread_ = std::thread([this]() {
      ControllerState st{};
      while (ticking_.load(std::memory_order_acquire)) {
        if (ctrl_a_) {
          (void)ctrl_a_->Compute(st);
        }
        std::this_thread::sleep_for(tick_period_);
      }
    });

    ASSERT_TRUE(reset_client_->wait_for_service(std::chrono::seconds(2)))
        << "reset_fault service did not appear";
  }

  void TearDown() override {
    ticking_.store(false, std::memory_order_release);
    if (tick_thread_.joinable()) {
      tick_thread_.join();
    }
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  rtc_msgs::srv::ResetFault::Response::SharedPtr CallReset(const std::string& name) {
    auto req = std::make_shared<rtc_msgs::srv::ResetFault::Request>();
    req->controller_name = name;
    auto fut = reset_client_->async_send_request(req);
    if (fut.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      return nullptr;
    }
    return fut.get();
  }

  std::shared_ptr<RtControllerNode> node_;
  std::shared_ptr<rclcpp::Node> client_node_;
  rclcpp::Client<rtc_msgs::srv::ResetFault>::SharedPtr reset_client_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::thread tick_thread_;
  std::atomic<bool> ticking_{false};
  std::chrono::microseconds tick_period_{2000};
  FaultyMockController* ctrl_a_{nullptr};
  FaultyMockController* ctrl_b_{nullptr};
};

// ── Authority: the name is the operator confirmation ────────────────────

TEST_F(ResetFaultServiceTest, EmptyControllerNameIsRefusedAndNamesTheActiveOne) {
  ctrl_a_->LatchFault();

  auto resp = CallReset("");
  ASSERT_NE(nullptr, resp);
  EXPECT_FALSE(resp->ok) << "an unnamed request cleared a fault";
  EXPECT_NE(std::string::npos, resp->message.find("required"));
  EXPECT_NE(std::string::npos, resp->message.find("ctrl_a"))
      << "the refusal did not tell the caller which controller is active";
  EXPECT_TRUE(ctrl_a_->HasLatchedFault()) << "the latch was cleared by a refused request";
}

TEST_F(ResetFaultServiceTest, NonActiveControllerIsRefusedRatherThanQueued) {
  ctrl_a_->LatchFault();
  ctrl_b_->LatchFault();

  auto resp = CallReset("ctrl_b");
  ASSERT_NE(nullptr, resp);
  EXPECT_FALSE(resp->ok);
  EXPECT_NE(std::string::npos, resp->message.find("not the active controller"));
  // Nothing may have been delivered to the inactive controller: a queued
  // request there would be consumed on its next activation (E-8 laundering).
  EXPECT_TRUE(ctrl_b_->HasLatchedFault());
  ControllerState st{};
  (void)ctrl_b_->Compute(st);
  EXPECT_TRUE(ctrl_b_->HasLatchedFault())
      << "a request was queued on the inactive controller and consumed on its next tick";
}

TEST_F(ResetFaultServiceTest, UnknownControllerNameIsRefused) {
  ctrl_a_->LatchFault();
  auto resp = CallReset("no_such_controller");
  ASSERT_NE(nullptr, resp);
  EXPECT_FALSE(resp->ok);
  EXPECT_TRUE(ctrl_a_->HasLatchedFault());
}

// ── Outcome reporting ───────────────────────────────────────────────────

TEST_F(ResetFaultServiceTest, LatchedFaultIsClearedAndReportedAsCleared) {
  ctrl_a_->LatchFault();
  ASSERT_TRUE(ctrl_a_->HasLatchedFault());

  auto resp = CallReset("ctrl_a");
  ASSERT_NE(nullptr, resp);
  EXPECT_TRUE(resp->ok) << "reset refused: " << resp->message;
  EXPECT_NE(std::string::npos, resp->message.find("cleared"));
  EXPECT_FALSE(ctrl_a_->HasLatchedFault());
}

TEST_F(ResetFaultServiceTest, NoLatchedFaultIsReportedAsNoOp) {
  ASSERT_FALSE(ctrl_a_->HasLatchedFault());
  auto resp = CallReset("ctrl_a");
  ASSERT_NE(nullptr, resp);
  EXPECT_TRUE(resp->ok);
  EXPECT_NE(std::string::npos, resp->message.find("no-op"));
}

// The response must not claim a recovery that did not happen: with the cause
// still present the state machine re-latches on the same tick it is reset.
TEST_F(ResetFaultServiceTest, PersistentCauseReLatchesAndIsReportedAsFailure) {
  ctrl_a_->LatchFault();
  ctrl_a_->SetCausePresent(true);

  auto resp = CallReset("ctrl_a");
  ASSERT_NE(nullptr, resp);
  EXPECT_FALSE(resp->ok) << "the service reported a recovery that did not happen";
  EXPECT_NE(std::string::npos, resp->message.find("still present"));
  EXPECT_TRUE(ctrl_a_->HasLatchedFault());

  // Remove the cause and the same request now succeeds — the refusal above was
  // about the cause, not about the request being malformed.
  ctrl_a_->SetCausePresent(false);
  auto resp2 = CallReset("ctrl_a");
  ASSERT_NE(nullptr, resp2);
  EXPECT_TRUE(resp2->ok) << resp2->message;
  EXPECT_FALSE(ctrl_a_->HasLatchedFault());
}

// ── E-8: the two latches are separate, in both directions ───────────────

TEST_F(ResetFaultServiceTest, ResetFaultDoesNotClearTheGlobalEstop) {
  ControllerLifecycleTestAccess::SetEstopFlag(*node_, true);
  ctrl_a_->TriggerEstop();
  ctrl_a_->LatchFault();

  auto resp = CallReset("ctrl_a");
  ASSERT_NE(nullptr, resp);
  EXPECT_TRUE(resp->ok) << "a global E-STOP blocked a controller-local fault reset: "
                        << resp->message;
  EXPECT_FALSE(ctrl_a_->HasLatchedFault()) << "the controller fault was not cleared";

  // The global latch is untouched, and the operator is told so rather than
  // being left to infer the arm is free to move.
  EXPECT_TRUE(ControllerLifecycleTestAccess::IsEstopped(*node_))
      << "reset_fault cleared the global E-STOP latch (E-8)";
  EXPECT_TRUE(ctrl_a_->IsEstopped());
  EXPECT_NE(std::string::npos, resp->message.find("E-STOP"))
      << "the reply did not mention the still-latched global E-STOP: " << resp->message;
}

TEST_F(ResetFaultServiceTest, GlobalEstopClearDoesNotClearTheControllerFault) {
  ControllerLifecycleTestAccess::SetEstopFlag(*node_, true);
  ctrl_a_->TriggerEstop();
  ctrl_a_->LatchFault();

  // The other direction of the same separation: whoever clears the global latch
  // must not release the controller-local one on the way past.
  ctrl_a_->ClearEstop();
  ControllerLifecycleTestAccess::SetEstopFlag(*node_, false);
  std::this_thread::sleep_for(tick_period_ * 3);

  EXPECT_TRUE(ctrl_a_->HasLatchedFault())
      << "clearing the global E-STOP released the controller-local fault latch (E-8)";
  auto resp = CallReset("ctrl_a");
  ASSERT_NE(nullptr, resp);
  EXPECT_TRUE(resp->ok) << resp->message;
  EXPECT_EQ(std::string::npos, resp->message.find("E-STOP"))
      << "the reply warned about an E-STOP that is no longer latched";
}

}  // namespace rtc
