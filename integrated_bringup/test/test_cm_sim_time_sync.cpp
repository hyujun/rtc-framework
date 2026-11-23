// ── CmSimTimeSync — the fail-closed read of the CM's sim-sync fact ──────────
//
// The CM declares `cm_sim_time_sync` on every controller node it builds
// (rtc_controller_manager test_cm_config_pipeline pins that side). This pins
// the controller side: only a bool true reads as sim-synced; a node without
// the parameter, one with false and one with a value of another type all read
// as "not sim-synced" — and the reader declares nothing itself.

#include "integrated_bringup/support/sim_time_sync.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace {

using integrated_bringup::CmSimTimeSync;
using integrated_bringup::kCmSimTimeSyncParam;

class CmSimTimeSyncTest : public ::testing::Test {
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

  static std::shared_ptr<rclcpp_lifecycle::LifecycleNode> MakeNode() {
    static int n = 0;
    return std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "cm_sim_time_sync_" + std::to_string(n++),
        rclcpp::NodeOptions().use_global_arguments(false));
  }
};

TEST_F(CmSimTimeSyncTest, UndeclaredReadsAsNotSynced) {
  auto node = MakeNode();
  EXPECT_FALSE(CmSimTimeSync(*node));
  EXPECT_FALSE(node->has_parameter(kCmSimTimeSyncParam)) << "the reader declares nothing";
}

TEST_F(CmSimTimeSyncTest, OnlyATrueBoolReadsAsSynced) {
  auto node = MakeNode();
  node->declare_parameter<bool>(kCmSimTimeSyncParam, false);
  EXPECT_FALSE(CmSimTimeSync(*node));
  node->set_parameter(rclcpp::Parameter(kCmSimTimeSyncParam, true));
  EXPECT_TRUE(CmSimTimeSync(*node));
}

TEST_F(CmSimTimeSyncTest, AnotherTypeReadsAsNotSynced) {
  auto node = MakeNode();
  node->declare_parameter<std::string>(kCmSimTimeSyncParam, "true");
  EXPECT_FALSE(CmSimTimeSync(*node));
}

}  // namespace
