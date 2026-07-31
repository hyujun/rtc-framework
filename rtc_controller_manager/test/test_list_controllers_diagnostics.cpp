// ── /rtc_cm/list_controllers diagnostics fields (issue #287) ────────────────
//
// Four states are tracked exactly inside RTControllerInterface and, until this
// slice, had no way out of the process: the controller-local fault latch and
// the three target-mailbox counters. They belong together because they share a
// failure MODE, not an implementation — every one of them presents to an
// operator as "I sent a goal and nothing moved", and without them there is no
// way to tell which of the four happened:
//
//   has_latched_fault      the controller refuses to act until reset_fault
//   target_drop_count      the goal was valid but arrived faster than a tick
//   target_reject_count    the goal was malformed / addressed to no device
//   target_unhandled_count the controller never overrode ApplyPendingTarget
//
// These tests drive the service THROUGH THE WIRE, like test_switch_service.cpp,
// because the acceptance criterion is about what a remote operator can see. A
// direct call to the accessors would pass even if the CM never filled the
// response, which is the whole defect being closed.
//
// DiagMockController deliberately does NOT override ApplyPendingTarget: that
// omission is what target_unhandled_count exists to report, so the mock has to
// contain the defect for the field to have anything to say. Its Compute() is
// called from the test body rather than by a stand-in RT loop — these fields
// are cumulative, so the tests need to know exactly how many drains happened.
#include "rtc_controller_manager/rt_controller_node.hpp"
#include <rtc_msgs/srv/list_controllers.hpp>

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <span>
#include <string>
#include <thread>
#include <vector>

namespace rtc {

// A controller that uses the base mailbox and can latch a fault, i.e. one that
// can move all four diagnostics. Everything else is the minimum
// RTControllerInterface needs to be injectable.
class DiagMockController : public RTControllerInterface {
 public:
  explicit DiagMockController(std::string name) : name_(std::move(name)) {}

  CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) noexcept override {
    return RTControllerInterface::on_activate(prev);
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*prev*/) noexcept override {
    return CallbackReturn::SUCCESS;
  }

  // The RT side: drain whatever the ingress queued. Nothing is applied — see
  // the file header on why the missing ApplyPendingTarget override is the
  // point rather than an oversight.
  ControllerOutput Compute(const ControllerState& /*state*/) noexcept override {
    (void)DrainPendingTargets();
    return ControllerOutput{};
  }

  // Off-RT ingress, the same marshal every real controller inherits.
  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override {
    PushPendingTarget(device_idx, target, /*is_task=*/false);
  }

  std::string_view Name() const noexcept override { return name_; }

  bool HasLatchedFault() const noexcept override {
    return latched_.load(std::memory_order_acquire);
  }

  // ── Test drivers ────────────────────────────────────────────────────────
  void LatchFault() { latched_.store(true, std::memory_order_release); }

  void ClearFault() { latched_.store(false, std::memory_order_release); }

  // Move each counter by a DIFFERENT amount, so a response that reports the
  // right numbers in the wrong fields fails: +1 drop, +2 reject, +3 unhandled.
  void DriveOneRoundOfEachFailure() {
    const std::vector<double> values(6, 1.0);
    // Depth is four with one slot reserved, so three fit between drains and
    // the fourth push is the one that saturates → +1 drop.
    for (int i = 0; i < 4; ++i) {
      SetDeviceTarget(0, values);
    }
    // Refused at ingress, before the queue → +2 reject, and no depth consumed.
    SetDeviceTarget(-1, values);
    SetDeviceTarget(ControllerState::kMaxDevices, values);
    // Drains the three that were accepted; each lands on the base's counting
    // default → +3 unhandled.
    ControllerState st{};
    (void)Compute(st);
  }

 private:
  std::string name_;
  std::atomic<bool> latched_{false};
};

// Friend bridge — one per gtest binary, same pattern as test_switch_service.cpp
// (each test binary is its own TU, so the repeated definition is ODR-safe).
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

  static void BringServicesOnline(RtControllerNode& node) {
    if (!node.cb_group_nrt_callback_) {
      node.cb_group_nrt_callback_ =
          node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    }
    node.CreateServices();
  }
};

// ── Fixture ─────────────────────────────────────────────────────────────

class ListControllersDiagnosticsTest : public ::testing::Test {
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
    node_ = std::make_shared<RtControllerNode>("test_list_diag_node");

    std::vector<std::unique_ptr<RTControllerInterface>> ctrls;
    auto a = std::make_unique<DiagMockController>("ctrl_a");
    auto b = std::make_unique<DiagMockController>("ctrl_b");
    ctrl_a_ = a.get();
    ctrl_b_ = b.get();
    ctrls.push_back(std::move(a));
    ctrls.push_back(std::move(b));

    ControllerLifecycleTestAccess::InjectControllers(*node_, std::move(ctrls),
                                                     {"type_a", "type_b"});
    ControllerLifecycleTestAccess::SetActiveIdx(*node_, 0);
    ControllerLifecycleTestAccess::SetState(*node_, 0, 1);  // ctrl_a active
    ControllerLifecycleTestAccess::BringServicesOnline(*node_);

    client_node_ = std::make_shared<rclcpp::Node>("test_list_diag_client");
    list_client_ =
        client_node_->create_client<rtc_msgs::srv::ListControllers>("/rtc_cm/list_controllers");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    ASSERT_TRUE(list_client_->wait_for_service(std::chrono::seconds(2)))
        << "list_controllers service did not appear";
  }

  void TearDown() override {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  rtc_msgs::srv::ListControllers::Response::SharedPtr CallList() {
    auto req = std::make_shared<rtc_msgs::srv::ListControllers::Request>();
    auto fut = list_client_->async_send_request(req);
    if (fut.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      return nullptr;
    }
    return fut.get();
  }

  std::shared_ptr<RtControllerNode> node_;
  std::shared_ptr<rclcpp::Node> client_node_;
  rclcpp::Client<rtc_msgs::srv::ListControllers>::SharedPtr list_client_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  DiagMockController* ctrl_a_{nullptr};
  DiagMockController* ctrl_b_{nullptr};
};

// A controller nothing has gone wrong with must report so ACTIVELY — an
// all-zero row is the answer an operator uses to rule the controller out, so
// it has to come from the accessors rather than from the fields never being
// touched.
TEST_F(ListControllersDiagnosticsTest, UntouchedControllersReportCleanDiagnostics) {
  auto resp = CallList();
  ASSERT_NE(nullptr, resp);
  ASSERT_EQ(2u, resp->controllers.size());

  for (const auto& cs : resp->controllers) {
    EXPECT_FALSE(cs.has_latched_fault) << cs.name;
    EXPECT_EQ(0u, cs.target_drop_count) << cs.name;
    EXPECT_EQ(0u, cs.target_reject_count) << cs.name;
    EXPECT_EQ(0u, cs.target_unhandled_count) << cs.name;
  }
}

// The three counters reach the response in their own fields, with the values
// the base actually holds, and only for the controller that earned them. The
// amounts are deliberately different (1 / 2 / 3) so a response that fills the
// fields in the wrong order still fails.
TEST_F(ListControllersDiagnosticsTest, EachCounterIsReportedInItsOwnFieldAndForItsOwnController) {
  ctrl_a_->DriveOneRoundOfEachFailure();

  auto resp = CallList();
  ASSERT_NE(nullptr, resp);
  ASSERT_EQ(2u, resp->controllers.size());
  ASSERT_EQ("ctrl_a", resp->controllers[0].name);
  ASSERT_EQ("ctrl_b", resp->controllers[1].name);

  const auto& a = resp->controllers[0];
  EXPECT_EQ(1u, a.target_drop_count) << "queue saturation did not reach the response";
  EXPECT_EQ(2u, a.target_reject_count) << "ingress refusals did not reach the response";
  EXPECT_EQ(3u, a.target_unhandled_count) << "the missing ApplyPendingTarget override is invisible";

  // Same values the base holds — the response is a report, not a second
  // accounting of its own.
  EXPECT_EQ(ctrl_a_->GetTargetDropCount(), a.target_drop_count);
  EXPECT_EQ(ctrl_a_->GetTargetRejectCount(), a.target_reject_count);
  EXPECT_EQ(ctrl_a_->GetTargetUnhandledCount(), a.target_unhandled_count);

  // The untouched controller stays at zero: a snapshot that read one
  // controller's counters for every row would make every controller look
  // equally broken.
  const auto& b = resp->controllers[1];
  EXPECT_EQ(0u, b.target_drop_count);
  EXPECT_EQ(0u, b.target_reject_count);
  EXPECT_EQ(0u, b.target_unhandled_count);
}

// Monotonicity is the property that makes two polls comparable: an operator
// diagnoses saturation by seeing a counter MOVE between requests, so a field
// that reset per call (or per drain) would read as "no problem" forever. Pinned
// at the service level because that is where the comparison happens.
TEST_F(ListControllersDiagnosticsTest, CountersAreCumulativeAcrossSuccessivePolls) {
  ctrl_a_->DriveOneRoundOfEachFailure();

  auto first = CallList();
  ASSERT_NE(nullptr, first);
  ASSERT_EQ(2u, first->controllers.size());
  const auto before = first->controllers[0];
  ASSERT_EQ(1u, before.target_drop_count);
  ASSERT_EQ(2u, before.target_reject_count);
  ASSERT_EQ(3u, before.target_unhandled_count);

  // A poll must not consume what it reports.
  auto repeat = CallList();
  ASSERT_NE(nullptr, repeat);
  EXPECT_EQ(before.target_drop_count, repeat->controllers[0].target_drop_count);
  EXPECT_EQ(before.target_reject_count, repeat->controllers[0].target_reject_count);
  EXPECT_EQ(before.target_unhandled_count, repeat->controllers[0].target_unhandled_count);

  ctrl_a_->DriveOneRoundOfEachFailure();

  auto second = CallList();
  ASSERT_NE(nullptr, second);
  const auto after = second->controllers[0];
  EXPECT_GT(after.target_drop_count, before.target_drop_count);
  EXPECT_GT(after.target_reject_count, before.target_reject_count);
  EXPECT_GT(after.target_unhandled_count, before.target_unhandled_count);
  EXPECT_EQ(2u, after.target_drop_count);
  EXPECT_EQ(4u, after.target_reject_count);
  EXPECT_EQ(6u, after.target_unhandled_count);
}

// The latch flag tracks the controller's own HasLatchedFault(), both edges.
// Before this field the only way to learn a controller was latched was to CALL
// /rtc_cm/reset_fault — which requires naming the latched controller, so the
// question could not be answered without already knowing the answer.
TEST_F(ListControllersDiagnosticsTest, LatchedFaultMirrorsTheControllerFlagOnBothEdges) {
  ctrl_a_->LatchFault();

  auto latched = CallList();
  ASSERT_NE(nullptr, latched);
  ASSERT_EQ(2u, latched->controllers.size());
  EXPECT_TRUE(latched->controllers[0].has_latched_fault);
  EXPECT_EQ(ctrl_a_->HasLatchedFault(), latched->controllers[0].has_latched_fault);
  EXPECT_FALSE(latched->controllers[1].has_latched_fault) << "the latch leaked onto a controller "
                                                             "that never latched";

  ctrl_a_->ClearFault();

  auto cleared = CallList();
  ASSERT_NE(nullptr, cleared);
  EXPECT_FALSE(cleared->controllers[0].has_latched_fault)
      << "the field latches on its own instead of mirroring the controller";
}

// The latch of an INACTIVE controller is reported too. That is the case the
// field exists for: /rtc_cm/reset_fault refuses a non-active target, so an
// operator who finds a latched inactive controller has to switch to it first —
// and cannot decide to do that if the list only ever describes the active one.
TEST_F(ListControllersDiagnosticsTest, AnInactiveControllersLatchIsStillReported) {
  ctrl_b_->LatchFault();  // ctrl_b is the inactive one (fixture activates a)

  auto resp = CallList();
  ASSERT_NE(nullptr, resp);
  ASSERT_EQ(2u, resp->controllers.size());
  ASSERT_FALSE(resp->controllers[1].is_active) << "fixture no longer has an inactive ctrl_b";
  EXPECT_TRUE(resp->controllers[1].has_latched_fault);
  EXPECT_FALSE(resp->controllers[0].has_latched_fault);
}

}  // namespace rtc
