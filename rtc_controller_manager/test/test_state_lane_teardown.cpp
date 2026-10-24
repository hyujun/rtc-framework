// Tier 1 — teardown drains the state lane before destroying a backend (#941).
//
// The hazard: a backend's state-lane subscription callback runs on the
// rt_callback executor (SCHED_FIFO) while a lifecycle callback on another
// thread tears the node down. rclcpp keeps only the Subscription object alive
// for the duration of a dispatch — not the backend whose `this` the lambda
// captured — so `backend.reset()` in ReleaseConfiguredResources() could free
// the object under a callback that was still reading its members. #224 fixed
// the fd half (close the eventfds after the backends); this is the residual it
// left: the destroy itself has to wait for the callback.
//
// What is locked here:
//   - on_cleanup / on_error observe an in-flight callback and wait for it. The
//     backend records, from its own destructor, whether a callback body was
//     still executing — with the pre-#941 order it was.
//   - The wait is bounded. A callback that outlives
//     DeviceBackend::kStateLaneCloseTimeout is reported (ERROR) and the backend
//     is leaked, not destroyed under it; the closed lane rejects later
//     messages even though the subscription survives.
//   - The destructor path closes the lane before the backends go. Production
//     reaches it only after RtControllerMain has joined every executor, so no
//     latch case — the structural fact is asserted instead (precedent: #218).
//
// Not a timing-race test: each case parks the callback on a latch the test
// releases, so "destroyed while in flight" is a decision the test makes, not a
// window it hopes to hit. Every negative assertion sits behind a witness that
// proves the message was delivered to this executor at all.

#include "rclcpp_suite_fixture.hpp"
#include "rt_cm_log_sink.hpp"
#include "rt_cm_test_access.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace rtc {
namespace {

using namespace std::chrono_literals;

// Everything the callback and the destructor report. Owned by the test, never
// by the backend: the whole point is that the backend may be gone while the
// callback is still writing.
struct LaneObservation {
  std::atomic<int> entered{0};           ///< callback bodies started (delivery witness)
  std::atomic<int> completed{0};         ///< callback bodies that ran to the end
  std::atomic<bool> in_body{false};      ///< a body is between entry and exit
  std::atomic<bool> release{false};      ///< test → callback: you may finish
  std::atomic<bool> closed_seen{false};  ///< the body saw its lane closed
  std::atomic<bool> destroyed{false};
  std::atomic<bool> destroyed_in_flight{false};  ///< destructor ran with in_body set
  std::atomic<bool> closed_at_destroy{false};    ///< StateLaneClosed() in the destructor
};

// A backend whose only state lane parks on a latch. The body touches `this`
// after the latch exactly where a shipped backend writes its SeqLock — that is
// the use-after-free the gate exists to prevent.
class LatchBackend : public DeviceBackend {
 public:
  explicit LatchBackend(LaneObservation& obs) : obs_(obs) {}

  ~LatchBackend() override {
    obs_.destroyed_in_flight.store(obs_.in_body.load(std::memory_order_acquire));
    obs_.closed_at_destroy.store(StateLaneClosed());
    obs_.destroyed.store(true, std::memory_order_release);
  }

  void Configure(rclcpp_lifecycle::LifecycleNode* node, const DeviceBackendConfig& config,
                 rclcpp::CallbackGroup::SharedPtr state_cb_group) override {
    rclcpp::QoS qos{1};
    qos.reliable();
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = std::move(state_cb_group);
    // The observation is captured by reference on purpose: the lambda lives
    // in the subscription, which the executor keeps alive during dispatch, so
    // the signalling stays valid even when `this` does not. Only the member
    // touch below depends on the backend's lifetime — as in production.
    sub_ = node->create_subscription<std_msgs::msg::Int32>(
        config.state_topic, qos,
        GuardStateLane([this, &obs = obs_](std_msgs::msg::Int32::SharedPtr) { OnState(obs); }),
        opts);
  }

  void Activate() override {}

  void Deactivate() override {}

  [[nodiscard]] bool ReadState(DeviceStateCache&) noexcept override { return false; }

  void WriteCommand(const PublishSnapshot::GroupCommandSlot&, CommandType) noexcept override {}

  void WriteSafeCommand() noexcept override {}

  [[nodiscard]] std::chrono::steady_clock::time_point LastStateStamp() const noexcept override {
    return {};
  }

 private:
  void OnState(LaneObservation& obs) {
    obs.in_body.store(true, std::memory_order_release);
    obs.entered.fetch_add(1, std::memory_order_acq_rel);
    while (!obs.release.load(std::memory_order_acquire)) {
      if (StateLaneClosed()) {
        obs.closed_seen.store(true, std::memory_order_release);
      }
      std::this_thread::sleep_for(100us);
    }
    ++member_touches_;  // the member write a shipped backend's SeqLock store is
    obs.completed.fetch_add(1, std::memory_order_acq_rel);
    obs.in_body.store(false, std::memory_order_release);
  }

  LaneObservation& obs_;
  int member_touches_{0};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

// Sleep-only wait (PROC-8): the executor under test spins on its own thread.
template <typename Pred>
bool WaitUntil(Pred pred, std::chrono::milliseconds budget) {
  const auto deadline = std::chrono::steady_clock::now() + budget;
  while (!pred()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(200us);
  }
  return true;
}

class StateLaneTeardownTest : public RclcppSuiteTest {
 protected:
  void SetUp() override {
    // Per-case names: a backend leaked by the timeout case keeps its
    // subscription alive, and it must not hear the next case's messages.
    const std::string case_name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    topic_ = "/rtc_cm_941/" + case_name;
    node_ = std::make_shared<RtControllerNode>("test_state_lane_" + case_name);
    ControllerLifecycleTestAccess::EnsureRtCallbackGroup(*node_);
    ASSERT_TRUE(ControllerLifecycleTestAccess::OpenTeardownEventfds(*node_))
        << "eventfd() unavailable in this environment";

    DeviceBackendConfig cfg;
    cfg.group_name = "latch";
    cfg.type = "latch";
    cfg.state_topic = topic_;
    auto backend = std::make_unique<LatchBackend>(obs_);
    backend->Configure(node_.get(), cfg, node_->GetRtCallbackGroup());
    backend_raw_ = backend.get();
    ControllerLifecycleTestAccess::InjectBackend(*node_, 0, std::move(backend));

    // Delivery witness in the SAME MutuallyExclusive group as the backend's
    // sub: when it has seen message N, this executor has received message N.
    rclcpp::QoS qos{1};
    qos.reliable();
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = node_->GetRtCallbackGroup();
    witness_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        topic_, qos,
        [this](std_msgs::msg::Int32::SharedPtr) {
          witnessed_.fetch_add(1, std::memory_order_acq_rel);
        },
        opts);

    // Only the rt callback group rides this executor — the production shape
    // (RtControllerMain Phase 3), where lifecycle callbacks run elsewhere.
    executor_.add_callback_group(node_->GetRtCallbackGroup(), node_->get_node_base_interface());
    spin_thread_ = std::thread([this] { executor_.spin(); });

    pub_node_ = std::make_shared<rclcpp::Node>("test_state_lane_pub_" + case_name);
    pub_ = pub_node_->create_publisher<std_msgs::msg::Int32>(topic_, qos);
    ASSERT_TRUE(WaitUntil([this] { return pub_->get_subscription_count() >= 2; }, 5s))
        << "backend + witness subscriptions never matched the publisher";
  }

  void TearDown() override {
    obs_.release.store(true, std::memory_order_release);  // never leave a lane parked
    StopExecutor();
    LogSink::Restore();
    pub_.reset();
    pub_node_.reset();
    witness_sub_.reset();
    node_.reset();
  }

  void StopExecutor() {
    if (spin_thread_.joinable()) {
      executor_.cancel();
      spin_thread_.join();
    }
  }

  // Publishes once and waits until the witness has seen it. The backend's own
  // entry is asserted by the caller where it is expected.
  [[nodiscard]] bool PublishAndWitness() {
    const int before = witnessed_.load(std::memory_order_acquire);
    std_msgs::msg::Int32 msg;
    msg.data = before + 1;
    pub_->publish(msg);
    return WaitUntil([this, before] { return witnessed_.load() > before; }, 2s);
  }

  static rclcpp_lifecycle::State InactiveState() {
    return rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  }

  // Drives `transition` on its own thread while this one plays the executor's
  // part: wait for the lane to be closed (or for the backend to be gone, which
  // is the failure this test exists for), then let the callback finish.
  RtControllerNode::CallbackReturn RunTeardownAgainstParkedCallback(
      const std::function<RtControllerNode::CallbackReturn()>& transition) {
    RtControllerNode::CallbackReturn ret = RtControllerNode::CallbackReturn::ERROR;
    std::thread lifecycle([&] { ret = transition(); });
    // A generous budget: the lifecycle thread does file I/O (the drain lane)
    // before it reaches the drain, and a slow disk must not turn into a red
    // run in which the drain was never exercised. A correct teardown is still
    // inside its 50 ms wait when the latch opens, because the body polls
    // StateLaneClosed() every 100 us and this loop every 200 us.
    static_cast<void>(
        WaitUntil([this] { return obs_.closed_seen.load() || obs_.destroyed.load(); }, 5s));
    obs_.release.store(true, std::memory_order_release);
    lifecycle.join();
    return ret;
  }

  LaneObservation obs_;
  std::string topic_;
  std::shared_ptr<RtControllerNode> node_;
  LatchBackend* backend_raw_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr witness_sub_;
  std::atomic<int> witnessed_{0};
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  std::shared_ptr<rclcpp::Node> pub_node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
};

TEST_F(StateLaneTeardownTest, CleanupWaitsForAnInFlightStateCallback) {
  ASSERT_TRUE(PublishAndWitness());
  ASSERT_TRUE(WaitUntil([this] { return obs_.entered.load() == 1; }, 2s))
      << "state callback never entered";

  const auto ret =
      RunTeardownAgainstParkedCallback([this] { return node_->on_cleanup(InactiveState()); });

  EXPECT_EQ(RtControllerNode::CallbackReturn::SUCCESS, ret);
  ASSERT_TRUE(obs_.destroyed.load()) << "on_cleanup leaked a backend whose callback did finish";
  EXPECT_FALSE(obs_.destroyed_in_flight.load())
      << "on_cleanup destroyed the backend while its state callback was still executing (#941)";
  EXPECT_TRUE(obs_.closed_seen.load()) << "the callback never saw its lane close before teardown";
  EXPECT_TRUE(obs_.closed_at_destroy.load());
  EXPECT_EQ(1, obs_.completed.load());
}

TEST_F(StateLaneTeardownTest, ErrorWaitsForAnInFlightStateCallback) {
  ASSERT_TRUE(PublishAndWitness());
  ASSERT_TRUE(WaitUntil([this] { return obs_.entered.load() == 1; }, 2s))
      << "state callback never entered";

  const auto ret =
      RunTeardownAgainstParkedCallback([this] { return node_->on_error(InactiveState()); });

  EXPECT_EQ(RtControllerNode::CallbackReturn::SUCCESS, ret);
  ASSERT_TRUE(obs_.destroyed.load()) << "on_error leaked a backend whose callback did finish";
  EXPECT_FALSE(obs_.destroyed_in_flight.load())
      << "on_error destroyed the backend while its state callback was still executing (#941)";
  EXPECT_TRUE(obs_.closed_seen.load());
  EXPECT_TRUE(obs_.closed_at_destroy.load());
  EXPECT_EQ(1, obs_.completed.load());
}

// Bounded wait: a callback that does not return within the timeout is not
// something teardown can fix, so it must neither block forever nor destroy
// the backend under it. The lane stays closed, so the leaked subscription
// drops what arrives afterwards.
TEST_F(StateLaneTeardownTest, CleanupLeaksABackendWhoseCallbackOutlivesTheTimeout) {
  LogSink::Install();
  ASSERT_TRUE(PublishAndWitness());
  ASSERT_TRUE(WaitUntil([this] { return obs_.entered.load() == 1; }, 2s))
      << "state callback never entered";

  // Synchronous: the latch is held across the whole call, so this is the
  // timeout path by construction.
  const auto t0 = std::chrono::steady_clock::now();
  const auto ret = node_->on_cleanup(InactiveState());
  const auto waited = std::chrono::steady_clock::now() - t0;

  EXPECT_EQ(RtControllerNode::CallbackReturn::SUCCESS, ret);
  EXPECT_GE(waited, DeviceBackend::kStateLaneCloseTimeout) << "teardown gave up before the bound";
  ASSERT_FALSE(obs_.destroyed.load())
      << "backend destroyed under a callback that had not returned (#941)";
  EXPECT_TRUE(backend_raw_->StateLaneClosed());
  EXPECT_FALSE(LogSink::Matching(RCUTILS_LOG_SEVERITY_ERROR, {"state-lane", "#941"}).empty())
      << "the leak was not reported";

  // The body is still parked — the leak decision above was made against a
  // callback that genuinely had not returned. It also holds the executor
  // thread, so nothing else on this group can be dispatched until it is let
  // go.
  ASSERT_EQ(0, obs_.completed.load());
  obs_.release.store(true, std::memory_order_release);
  ASSERT_TRUE(WaitUntil([this] { return obs_.completed.load() == 1; }, 2s));

  // The lane stays closed on the leaked backend: later messages reach this
  // executor (witness) but never enter the body, although the subscription is
  // still alive. Several messages, not one — the witness and the backend's
  // sub are two DDS readers of the same sample, so a single witnessed message
  // could still be undelivered to the backend; five in a row plus a grace are
  // not.
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(PublishAndWitness());
  }
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(1, obs_.entered.load()) << "a closed lane admitted a message";

  // Reclaim the leak once nothing can be dispatching into it any more.
  StopExecutor();
  delete backend_raw_;
  backend_raw_ = nullptr;
  EXPECT_TRUE(obs_.destroyed.load());
}

// Destructor path: production only reaches it with every executor joined, so
// what is asserted is the order — the lane is closed when the backend goes.
TEST_F(StateLaneTeardownTest, DestructorClosesTheLaneBeforeDestroyingBackends) {
  StopExecutor();
  witness_sub_.reset();
  node_.reset();

  ASSERT_TRUE(obs_.destroyed.load()) << "destructor did not release the backend at all";
  EXPECT_FALSE(obs_.destroyed_in_flight.load());
  EXPECT_TRUE(obs_.closed_at_destroy.load())
      << "destructor destroyed the backend with its state lane still open (#941)";
}

}  // namespace
}  // namespace rtc
