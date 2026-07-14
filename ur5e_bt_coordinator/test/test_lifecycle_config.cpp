/// Configuration-time coverage (Phase 4b) for the two on_configure safety nets,
/// exercised at the mechanism level so the assertions stay deterministic:
///   - Seam-D capability gating: RegisterBtNodes drops a sensor's node group
///     when its capability flag is false, so a tree referencing a gated-out
///     node fails to build (the "unknown node" error on_configure turns into a
///     clean lifecycle FAILURE) — and builds when the capability is present.
///   - LoadFingerMap validation: an out-of-range finger_map index throws at
///     configure time (which on_configure catches → FAILURE) instead of
///     becoming a latent OOB the first time that finger is commanded.
///
/// These avoid driving the full BtCoordinatorNode lifecycle (which would load
/// the large composite tree files); the capability flags and finger-map range
/// check are the actual units under test.

#include "ur5e_bt_coordinator/bt_node_registration.hpp"
#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using namespace rtc_bt;

namespace {

class ConfigSafetyTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  // A single-leaf tree referencing `node_name`, used to probe whether that node
  // type is registered under a given capability set.
  static std::string LeafTree(const std::string& node_name) {
    return R"(<root BTCPP_format="4"><BehaviorTree ID="T"><)" + node_name +
           R"(/></BehaviorTree></root>)";
  }

  // A LifecycleNode carrying the given parameter overrides (auto-declared),
  // enough to back a BtRosBridge for the finger-map range check.
  static rclcpp_lifecycle::LifecycleNode::SharedPtr MakeNode(
      const std::vector<rclcpp::Parameter>& params) {
    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);
    opts.parameter_overrides(params);
    return std::make_shared<rclcpp_lifecycle::LifecycleNode>("cfg_test_node", opts);
  }
};

// ── Seam-D capability gating (RegisterBtNodes flags) ────────────────────────

TEST_F(ConfigSafetyTest, ToFNodeNotRegisteredWhenCapabilityAbsent) {
  BT::BehaviorTreeFactory factory;
  std::shared_ptr<BtRosBridge> null_bridge;
  RegisterBtNodes(factory, null_bridge, RobotCapabilities{true, /*has_tof=*/false, true});
  // StartToFCollection is gated behind has_tof; unregistered → build throws the
  // factory's "unknown node" error (what on_configure catches into a FAILURE).
  EXPECT_THROW((void)factory.createTreeFromText(LeafTree("StartToFCollection")), std::exception);
}

TEST_F(ConfigSafetyTest, ToFNodeRegisteredWhenCapabilityPresent) {
  BT::BehaviorTreeFactory factory;
  std::shared_ptr<BtRosBridge> null_bridge;
  RegisterBtNodes(factory, null_bridge, RobotCapabilities{true, /*has_tof=*/true, true});
  // Registered → the tree builds (a null bridge is fine; nodes only deref it on
  // tick, which we never do here).
  EXPECT_NO_THROW((void)factory.createTreeFromText(LeafTree("StartToFCollection")));
}

TEST_F(ConfigSafetyTest, ShapeNodeGatedByHasShape) {
  BT::BehaviorTreeFactory factory;
  std::shared_ptr<BtRosBridge> null_bridge;
  RegisterBtNodes(factory, null_bridge, RobotCapabilities{true, true, /*has_shape=*/false});
  EXPECT_THROW((void)factory.createTreeFromText(LeafTree("TriggerShapeEstimation")), std::exception);
}

TEST_F(ConfigSafetyTest, GraspNodeGatedByHasGraspSensing) {
  BT::BehaviorTreeFactory factory;
  std::shared_ptr<BtRosBridge> null_bridge;
  RegisterBtNodes(factory, null_bridge, RobotCapabilities{/*has_grasp_sensing=*/false, true, true});
  EXPECT_THROW((void)factory.createTreeFromText(LeafTree("IsGrasped")), std::exception);
}

// ── LoadFingerMap range validation ──────────────────────────────────────────

TEST_F(ConfigSafetyTest, FingerMapOutOfRangeIndexThrows) {
  // hand_dof=10, finger_map.thumb=[0,1,99] → index 99 is out of range and must
  // throw at load time (on_configure wraps LoadFingerMap in its try/catch).
  auto node = MakeNode({rclcpp::Parameter("finger_map.thumb", std::vector<int64_t>{0, 1, 99})});
  BtRosBridge bridge(node, RobotProfile{TopicNamer{"ur5e", "hand"}, 6, 10});
  EXPECT_THROW(bridge.LoadFingerMap(node), std::exception);
}

TEST_F(ConfigSafetyTest, FingerMapInRangeIndexLoads) {
  auto node = MakeNode({rclcpp::Parameter("finger_map.thumb", std::vector<int64_t>{0, 1, 2})});
  BtRosBridge bridge(node, RobotProfile{TopicNamer{"ur5e", "hand"}, 6, 10});
  EXPECT_NO_THROW(bridge.LoadFingerMap(node));
}

TEST_F(ConfigSafetyTest, FingerMapNegativeIndexThrows) {
  auto node = MakeNode({rclcpp::Parameter("finger_map.index", std::vector<int64_t>{3, -1})});
  BtRosBridge bridge(node, RobotProfile{TopicNamer{"ur5e", "hand"}, 6, 10});
  EXPECT_THROW(bridge.LoadFingerMap(node), std::exception);
}

}  // namespace
