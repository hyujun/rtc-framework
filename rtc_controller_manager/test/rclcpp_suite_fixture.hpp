// One rclcpp context per test suite: initialised before the suite's first
// case, shut down after its last. Suites whose cases shut the context down
// from inside the node (the abort paths) re-initialise per case instead and do
// not derive from this.

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

namespace rtc {

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
