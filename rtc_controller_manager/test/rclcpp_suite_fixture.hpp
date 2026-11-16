// One rclcpp context per test suite: initialised before the suite's first
// case, shut down after its last. Suites whose cases shut the context down
// from inside the node (the abort paths) re-initialise per case instead and do
// not derive from this.

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <thread>

namespace rtc {

// Stops an executor spinning on `spinner`. A cancel() that lands before spin()
// has started is lost and the join never returns — a case too short to make a
// service call hits exactly that — so the stop waits for the spinner first.
inline void StopSpinner(rclcpp::Executor& executor, std::thread& spinner,
                        std::chrono::milliseconds wait = std::chrono::seconds(2)) {
  const auto deadline = std::chrono::steady_clock::now() + wait;
  while (!executor.is_spinning() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
}

// One service round-trip; nullptr when no reply arrived within `timeout`. The
// CM's latch services wait on the RT loop for tens of periods before giving
// up, so a client deadline has to outlast that or a slow reply looks hung.
template <class Srv>
typename Srv::Response::SharedPtr CallService(
    rclcpp::Client<Srv>& client, std::shared_ptr<typename Srv::Request> request,
    std::chrono::seconds timeout = std::chrono::seconds(10)) {
  auto future = client.async_send_request(std::move(request));
  if (future.wait_for(timeout) != std::future_status::ready) {
    return nullptr;
  }
  return future.get();
}

class RclcppSuiteTest : public ::testing::Test {
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
};

}  // namespace rtc
