// Tier 1 — teardown ordering invariant: the device backends must be destroyed
// BEFORE the eventfds their state-ready callback writes (issue #224).
//
// Why this is not a race test. The hazard is a genuine two-thread window
// (lifecycle executor tearing down while the rt_callback executor dispatches a
// backend state message into `eventfd_write(nrt_publish_eventfd_, 1)`), but
// reproducing it would be timing-dependent and flaky. What actually fixes the
// bug is the *order* of two statements, so that is what is locked here: a stub
// backend records, from inside its own destructor, the fd values the node
// still holds. With the pre-#224 order those reads see -1 (fds already closed
// while the subscription that writes them is still alive); with the corrected
// order they see the live descriptors.
//
// Precedent: #218 settled for a structural assertion where no deterministic
// red test existed. Here one does exist, so we take it.
//
// Residual NOT covered (documented in on_cleanup): destroying a backend does
// not join a callback already executing on the rt_callback executor, so a
// write can still land in the two instructions between the reset and the
// close. Closing that would need executor quiescence, which CM cannot request.

#include "rt_cm_test_access.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

namespace rtc {
namespace {

// Records the node's eventfd values at the moment this backend is destroyed —
// i.e. at the exact point where production destroys the subscriptions that
// dispatch the state-ready callback.
class TeardownProbeBackend : public DeviceBackend {
 public:
  struct Observation {
    bool destroyed{false};
    int nrt_publish_fd{-2};  // -2 = destructor never ran
    int sim_wake_fd{-2};
  };

  TeardownProbeBackend(RtControllerNode& node, Observation& out) : node_(node), out_(out) {}

  ~TeardownProbeBackend() override {
    out_.destroyed = true;
    out_.nrt_publish_fd = ControllerLifecycleTestAccess::GetNrtPublishEventfd(node_);
    out_.sim_wake_fd = ControllerLifecycleTestAccess::GetSimWakeEventfd(node_);
  }

  void Configure(rclcpp_lifecycle::LifecycleNode*, const DeviceBackendConfig&,
                 rclcpp::CallbackGroup::SharedPtr) override {}

  void Activate() override {}

  void Deactivate() override {}

  [[nodiscard]] bool ReadState(DeviceStateCache&) noexcept override { return false; }

  void WriteCommand(const PublishSnapshot::GroupCommandSlot&, CommandType) noexcept override {}

  void WriteSafeCommand() noexcept override {}

  [[nodiscard]] std::chrono::steady_clock::time_point LastStateStamp() const noexcept override {
    return {};
  }

 private:
  RtControllerNode& node_;
  Observation& out_;
};

class EventfdTeardownOrderTest : public ::testing::Test {
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
    node_ = std::make_shared<RtControllerNode>("test_eventfd_teardown_order_node");
    // Real descriptors, opened the way on_configure opens them, so close()
    // on the teardown path is a real close and not a no-op on a sentinel.
    ASSERT_TRUE(ControllerLifecycleTestAccess::OpenTeardownEventfds(*node_))
        << "eventfd() unavailable in this environment";
    ControllerLifecycleTestAccess::InjectBackend(
        *node_, 0, std::make_unique<TeardownProbeBackend>(*node_, obs_));
  }

  static rclcpp_lifecycle::State InactiveState() {
    return rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  }

  // Declared BEFORE node_ on purpose: members are destroyed in reverse
  // declaration order, and the probe backend writes into obs_ from its
  // destructor — which runs while node_ is being torn down.
  TeardownProbeBackend::Observation obs_;
  std::shared_ptr<RtControllerNode> node_;
};

TEST_F(EventfdTeardownOrderTest, CleanupDestroysBackendBeforeClosingEventfds) {
  ASSERT_EQ(RtControllerNode::CallbackReturn::SUCCESS, node_->on_cleanup(InactiveState()));

  ASSERT_TRUE(obs_.destroyed) << "on_cleanup did not release the backend at all";
  EXPECT_GE(obs_.nrt_publish_fd, 0)
      << "nrt_publish_eventfd_ was closed while the backend owning the state-ready "
         "subscription was still alive (issue #224)";
  EXPECT_GE(obs_.sim_wake_fd, 0)
      << "sim_wake_eventfd_ was closed while the backend owning the state-ready "
         "subscription was still alive (issue #224)";
}

TEST_F(EventfdTeardownOrderTest, ErrorDestroysBackendBeforeClosingEventfds) {
  ASSERT_EQ(RtControllerNode::CallbackReturn::SUCCESS, node_->on_error(InactiveState()));

  ASSERT_TRUE(obs_.destroyed) << "on_error did not release the backend at all";
  EXPECT_GE(obs_.nrt_publish_fd, 0) << "on_error closed nrt_publish_eventfd_ too early (#224)";
  EXPECT_GE(obs_.sim_wake_fd, 0) << "on_error closed sim_wake_eventfd_ too early (#224)";
}

// The reordering must not turn into "never closed" — cleanup still owns the
// release, it just happens later in the sequence.
TEST_F(EventfdTeardownOrderTest, CleanupStillReleasesBothEventfds) {
  ASSERT_EQ(RtControllerNode::CallbackReturn::SUCCESS, node_->on_cleanup(InactiveState()));

  EXPECT_EQ(-1, ControllerLifecycleTestAccess::GetNrtPublishEventfd(*node_));
  EXPECT_EQ(-1, ControllerLifecycleTestAccess::GetSimWakeEventfd(*node_));
}

// Destructor path (SIGTERM without a graceful lifecycle transition). `backends_`
// is a member, so before #224 it outlived the destructor body and the fds were
// closed first; the fix resets the backends explicitly up front.
TEST_F(EventfdTeardownOrderTest, DestructorDestroysBackendBeforeClosingEventfds) {
  node_.reset();

  ASSERT_TRUE(obs_.destroyed) << "destructor did not release the backend at all";
  EXPECT_GE(obs_.nrt_publish_fd, 0) << "destructor closed nrt_publish_eventfd_ too early (#224)";
  EXPECT_GE(obs_.sim_wake_fd, 0) << "destructor closed sim_wake_eventfd_ too early (#224)";
}

}  // namespace
}  // namespace rtc
