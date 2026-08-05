// Unit tests for DeviceBackend::StateLaneTimingScope — the rt_callback-lane
// timing instrument added by issue #349.
//
// The lane it measures is slot 2 (rt_callback, SCHED_FIFO 70): the thread
// #349 proposes to merge the nrt lanes onto, and the one RT thread that had
// no measurement channel. These tests pin the properties the before/after
// comparison depends on — if the split is wrong or a sample goes missing, the
// CSV still looks plausible, so the failure mode is a silently biased
// measurement rather than a crash.
//
// Deliberately does NOT spin an executor: the scope is a plain RAII object and
// its contract is per-callback, so driving it directly keeps the assertions
// about the instrument instead of about ROS dispatch.

#include "rtc_controller_manager/device_backend.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <vector>

namespace rtc {
namespace {

/// Busy-wait rather than sleep: the lane measures wall time spent inside the
/// callback, and a spin is what a decode actually looks like to it. sleep_for
/// would also elapse wall time but hands the CPU away, which is the one thing
/// a SCHED_FIFO callback does not do.
void BurnFor(std::chrono::microseconds d) {
  const auto deadline = std::chrono::steady_clock::now() + d;
  while (std::chrono::steady_clock::now() < deadline) {
  }
}

/// Minimal backend that replays the two callback shapes present in the tree:
/// a lane that ends in NotifyStateReady() (arm joint, hand joint) and a lane
/// that returns without notifying (hand motor, hand sensor).
class TimingStubBackend : public DeviceBackend {
 public:
  void Configure(rclcpp_lifecycle::LifecycleNode* /*node*/, const DeviceBackendConfig& /*config*/,
                 rclcpp::CallbackGroup::SharedPtr /*state_cb_group*/) override {}

  void Activate() override {}

  void Deactivate() override {}

  bool ReadState(DeviceStateCache& /*cache*/) noexcept override { return false; }

  void WriteCommand(const PublishSnapshot::GroupCommandSlot& /*slot*/,
                    CommandType /*command_type*/) noexcept override {}

  void WriteSafeCommand() noexcept override {}

  [[nodiscard]] std::chrono::steady_clock::time_point LastStateStamp() const noexcept override {
    return {};
  }

  /// One state-lane callback. `decode` is burned before the notify split,
  /// `handoff` inside the state-ready callback (i.e. after it).
  void RunLane(std::chrono::microseconds decode, bool notify,
               std::chrono::microseconds handoff = std::chrono::microseconds{0}) {
    StateLaneTimingScope timing_scope(*this);
    BurnFor(decode);
    if (!notify) {
      return;  // hand motor / sensor shape — no mailbox hand-off
    }
    handoff_ = handoff;
    NotifyStateReady();
  }

  /// Installs a state-ready callback that burns `handoff_`, standing in for
  /// CM's mailbox work (dirty bit + eventfd write).
  void InstallBurningStateReadyCallback() {
    SetStateReadyCallback([this]() { BurnFor(handoff_); });
  }

 private:
  std::chrono::microseconds handoff_{0};
};

std::vector<RtTickTimingPayload> DrainAll(RtCallbackTimingBuffer& buffer) {
  std::vector<RtTickTimingPayload> out;
  buffer.Drain([&out](const RtTickTimingSample& s) { out.push_back(s.payload); });
  return out;
}

using namespace std::chrono_literals;

// Without a sink the scope must be completely inert. This is the shipped
// default for any backend CM has not configured, and for every test fixture in
// the tree, so a scope that pushed anyway would fire on uninstrumented paths.
TEST(StateLaneTimingScope, NoSinkPushesNothing) {
  TimingStubBackend backend;
  RtCallbackTimingBuffer buffer;  // deliberately NOT installed

  backend.RunLane(100us, /*notify=*/false);

  EXPECT_TRUE(DrainAll(buffer).empty());
  EXPECT_EQ(buffer.DropCount(), 0U);
}

// One row per callback, in arrival order. The tick counter is what lets the
// analysis tell "the lane went quiet" from "the drain lost rows".
TEST(StateLaneTimingScope, PushesOneSamplePerCallback) {
  TimingStubBackend backend;
  RtCallbackTimingBuffer buffer;
  backend.SetStateLaneTimingSink(&buffer);

  backend.RunLane(50us, /*notify=*/false);
  backend.RunLane(50us, /*notify=*/false);
  backend.RunLane(50us, /*notify=*/false);

  EXPECT_EQ(DrainAll(buffer).size(), 3U);
  EXPECT_EQ(buffer.LastTickCount(), 3U);
}

// The split is the point of the instrument: decode cost (cache-pollution
// sensitive) must be separable from the mailbox hand-off, and the two must
// account for the whole callback.
TEST(StateLaneTimingScope, SplitsDecodeFromHandoff) {
  TimingStubBackend backend;
  RtCallbackTimingBuffer buffer;
  backend.SetStateLaneTimingSink(&buffer);
  backend.InstallBurningStateReadyCallback();

  backend.RunLane(/*decode=*/2000us, /*notify=*/true, /*handoff=*/1000us);

  const auto samples = DrainAll(buffer);
  ASSERT_EQ(samples.size(), 1U);
  const auto& p = samples.front();

  EXPECT_GT(p.t_state_us, 0.0);
  EXPECT_GT(p.t_publish_us, 0.0);
  // Ordering, not absolute values: a loaded CI box stretches both burns, but
  // the 2:1 ratio survives anything that does not preempt one burn and not
  // the other.
  EXPECT_GT(p.t_state_us, p.t_publish_us);
  // Conservation — the two phases tile the callback exactly.
  EXPECT_DOUBLE_EQ(p.t_state_us + p.t_publish_us, p.t_total_us);
  // This lane runs no control law and is not deadline-scheduled.
  EXPECT_DOUBLE_EQ(p.t_compute_us, 0.0);
  EXPECT_DOUBLE_EQ(p.jitter_us, 0.0);
}

// Hand motor / sensor lanes never call NotifyStateReady(). They must still be
// measured — they occupy slot 2 exactly like the joint lanes do — and must
// report their whole span as decode rather than inventing a hand-off.
TEST(StateLaneTimingScope, LaneWithoutNotifyReportsAllDecode) {
  TimingStubBackend backend;
  RtCallbackTimingBuffer buffer;
  backend.SetStateLaneTimingSink(&buffer);

  backend.RunLane(/*decode=*/1000us, /*notify=*/false);

  const auto samples = DrainAll(buffer);
  ASSERT_EQ(samples.size(), 1U);
  EXPECT_GT(samples.front().t_total_us, 0.0);
  EXPECT_DOUBLE_EQ(samples.front().t_publish_us, 0.0);
  EXPECT_DOUBLE_EQ(samples.front().t_state_us, samples.front().t_total_us);
}

// The split marker persists across callbacks, so a non-notifying lane that
// follows a notifying one still sees the previous stamp. The destructor's
// range check is what discards it. This is the ordering that matters most in
// the tree: hand joint (notifies) is followed by hand motor and hand sensor
// (neither notifies) on every hand packet.
//
// Removing the range check does not produce a subtly wrong split — the
// subtraction is unsigned, so t_state_us wraps to ~10^10 µs and the CSV fills
// with garbage that dwarfs every real measurement.
TEST(StateLaneTimingScope, StaleNotifyStampDoesNotLeakIntoNextLane) {
  TimingStubBackend backend;
  RtCallbackTimingBuffer buffer;
  backend.SetStateLaneTimingSink(&buffer);
  backend.InstallBurningStateReadyCallback();

  backend.RunLane(/*decode=*/500us, /*notify=*/true, /*handoff=*/500us);
  backend.RunLane(/*decode=*/500us, /*notify=*/false);

  const auto samples = DrainAll(buffer);
  ASSERT_EQ(samples.size(), 2U);
  EXPECT_GT(samples[0].t_publish_us, 0.0);
  EXPECT_DOUBLE_EQ(samples[1].t_publish_us, 0.0);
  EXPECT_DOUBLE_EQ(samples[1].t_state_us, samples[1].t_total_us);
}

// A full ring must account for what it discarded. The lane can outrun the
// 100 Hz drain (four state lanes at 500 Hz), so "rows are missing" has to be
// distinguishable from "the lane was idle" — CM prints the drop count in its
// timing summary for exactly this reason.
TEST(StateLaneTimingScope, DropsAreCountedNotSilent) {
  TimingStubBackend backend;
  RtCallbackTimingBuffer buffer;
  backend.SetStateLaneTimingSink(&buffer);

  constexpr std::size_t kPushes = kRtCallbackTimingBufferCapacity + 100;
  for (std::size_t i = 0; i < kPushes; ++i) {
    backend.RunLane(0us, /*notify=*/false);
  }

  const auto drained = DrainAll(buffer).size();
  EXPECT_GT(buffer.DropCount(), 0U);
  // Conservation: every callback either produced a row or incremented the
  // drop counter. Nothing vanishes.
  EXPECT_EQ(drained + buffer.DropCount(), kPushes);
}

}  // namespace
}  // namespace rtc
