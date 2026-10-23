// The E-STOP reason crosses threads (#935). TriggerGlobalEstop writes it from
// the RT loop with no lock, while the drain lane (FlushEstopLog, which also
// retires it once the latch is cleared) and the services
// (GlobalEstopReason) read it. A read must come back as one whole reason or
// as none — never the head of one reason over the tail of another.
//
// The writer here runs trigger/clear back to back on one thread, as the RT
// loop and a clear service would between them; the reader runs on another,
// taking the drain lock exactly as a DrainLog() pass does.
#include "rt_cm_test_access.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <rcutils/logging.h>

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>

namespace rtc {
namespace {

using Access = ControllerLifecycleTestAccess;

constexpr int kRounds = 200000;

TEST(EstopReasonRaceTest, AReadNeverMixesTwoReasons) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<RtControllerNode>("test_estop_reason_race_node");
  // Every drained trigger and clear is a log line; the reads are the subject.
  ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_set_logger_level(node->get_logger().get_name(),
                                                             RCUTILS_LOG_SEVERITY_FATAL));

  // Same length, so a mixed read cannot hide as a shorter whole one.
  const std::string reason_a(100, 'a');
  const std::string reason_b(100, 'b');

  std::atomic<bool> done{false};
  std::thread writer([&] {
    for (int i = 0; i < kRounds; ++i) {
      Access::CallTriggerEstop(*node, (i % 2) == 0 ? reason_a : reason_b);
      (void)Access::CallClearEstop(*node);
    }
    done.store(true, std::memory_order_release);
  });

  std::size_t reads = 0;
  std::size_t whole = 0;
  std::size_t mixed = 0;
  std::string first_mixed;
  while (!done.load(std::memory_order_acquire)) {
    Access::CallFlushEstopLog(*node);
    const std::string read = Access::GetGlobalEstopReason(*node);
    ++reads;
    if (read == reason_a || read == reason_b) {
      ++whole;
    } else if (!read.empty()) {
      if (mixed == 0) {
        first_mixed = read;
      }
      ++mixed;
    }
  }
  writer.join();

  RecordProperty("reads", static_cast<int>(reads));
  RecordProperty("whole", static_cast<int>(whole));
  RecordProperty("mixed", static_cast<int>(mixed));
  // Reads that saw a reason at all, so a reader that only ever gave up (or
  // only ever ran between a drain and the next trigger) cannot pass.
  EXPECT_GT(whole, 0U);
  EXPECT_EQ(0U, mixed) << mixed << " of " << reads << " reads were neither a whole reason nor "
                       << "none; the first: '" << first_mixed << "'";

  node.reset();
  rclcpp::shutdown();
}

}  // namespace
}  // namespace rtc
