/// Unit tests for bt_node_registration.hpp: capability-gated BT node
/// registration (Seam D).
///
///   - The default profile (all capabilities present) must register every node
///     type — the legacy register-everything behavior (regression anchor).
///   - Clearing a capability flag must drop exactly that sensor's node group
///     and leave every other node registered.
///   - A tree referencing a gated-out node must fail to build (the factory's
///     "unknown node" error), which is how a capability/tree mismatch surfaces
///     as a clean on_configure FAILURE rather than a silent no-data tick.

#include "ur5e_bt_coordinator/bt_node_registration.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using rtc_bt::RegisterBtNodes;
using rtc_bt::RobotCapabilities;

namespace {

bool Registered(const BT::BehaviorTreeFactory& factory, const std::string& name) {
  return factory.manifests().count(name) > 0;
}

// The capability-gated node groups (see RobotCapabilities).
const std::vector<std::string> kGraspNodes = {"GraspControl", "IsForceAbove", "IsGraspPhase",
                                              "IsGrasped"};
const std::vector<std::string> kToFNodes = {"StartToFCollection", "StopToFCollection",
                                            "ProcessSearchData"};
const std::vector<std::string> kShapeNodes = {"TriggerShapeEstimation", "WaitShapeResult",
                                              "CheckShapeType"};

// Representative always-on nodes (never gated).
const std::vector<std::string> kAlwaysOn = {"MoveToPose", "MoveToJoints", "SetHandPose",
                                            "MoveFinger", "IsObjectDetected"};

void ExpectAll(const BT::BehaviorTreeFactory& factory, const std::vector<std::string>& names,
               bool present) {
  for (const auto& n : names) {
    EXPECT_EQ(Registered(factory, n), present)
        << n << " expected " << (present ? "" : "not ") << "registered";
  }
}

}  // namespace

// ── Default (all capabilities) == register everything ─────────────────────

TEST(NodeRegistration, DefaultRegistersEveryGroup) {
  BT::BehaviorTreeFactory factory;
  std::shared_ptr<rtc_bt::BtRosBridge> null_bridge;
  RegisterBtNodes(factory, null_bridge);  // default RobotCapabilities{} = all true

  ExpectAll(factory, kAlwaysOn, true);
  ExpectAll(factory, kGraspNodes, true);
  ExpectAll(factory, kToFNodes, true);
  ExpectAll(factory, kShapeNodes, true);
}

// ── Each flag gates exactly its own group ─────────────────────────────────

TEST(NodeRegistration, NoGraspSensingDropsGraspGroupOnly) {
  BT::BehaviorTreeFactory factory;
  std::shared_ptr<rtc_bt::BtRosBridge> null_bridge;
  RegisterBtNodes(factory, null_bridge, RobotCapabilities{/*grasp=*/false, true, true});

  ExpectAll(factory, kGraspNodes, false);
  ExpectAll(factory, kToFNodes, true);
  ExpectAll(factory, kShapeNodes, true);
  ExpectAll(factory, kAlwaysOn, true);
}

TEST(NodeRegistration, NoToFDropsToFGroupOnly) {
  BT::BehaviorTreeFactory factory;
  std::shared_ptr<rtc_bt::BtRosBridge> null_bridge;
  RegisterBtNodes(factory, null_bridge, RobotCapabilities{true, /*tof=*/false, true});

  ExpectAll(factory, kToFNodes, false);
  ExpectAll(factory, kGraspNodes, true);
  ExpectAll(factory, kShapeNodes, true);
  ExpectAll(factory, kAlwaysOn, true);
}

TEST(NodeRegistration, NoShapeDropsShapeGroupOnly) {
  BT::BehaviorTreeFactory factory;
  std::shared_ptr<rtc_bt::BtRosBridge> null_bridge;
  RegisterBtNodes(factory, null_bridge, RobotCapabilities{true, true, /*shape=*/false});

  ExpectAll(factory, kShapeNodes, false);
  ExpectAll(factory, kGraspNodes, true);
  ExpectAll(factory, kToFNodes, true);
  ExpectAll(factory, kAlwaysOn, true);
}

// ── Position-only profile (iiwa7_leap scaffolding) drops all three ────────

TEST(NodeRegistration, PositionOnlyDropsAllSensorGroups) {
  BT::BehaviorTreeFactory factory;
  std::shared_ptr<rtc_bt::BtRosBridge> null_bridge;
  RegisterBtNodes(factory, null_bridge, RobotCapabilities{false, false, false});

  ExpectAll(factory, kGraspNodes, false);
  ExpectAll(factory, kToFNodes, false);
  ExpectAll(factory, kShapeNodes, false);
  // Motion / hand-pose nodes remain — a position-only hand can still be driven.
  ExpectAll(factory, kAlwaysOn, true);
}

// ── A tree needing a gated-out node fails to build ────────────────────────

TEST(NodeRegistration, TreeReferencingGatedNodeFailsToBuild) {
  const std::string grasp_tree =
      R"(<root BTCPP_format="4"><BehaviorTree ID="Main">
           <GraspControl/>
         </BehaviorTree></root>)";

  {
    BT::BehaviorTreeFactory factory;
    std::shared_ptr<rtc_bt::BtRosBridge> null_bridge;
    RegisterBtNodes(factory, null_bridge, RobotCapabilities{/*grasp=*/false, true, true});
    // GraspControl is not registered → factory rejects the tree.
    EXPECT_THROW(factory.createTreeFromText(grasp_tree), std::exception);
  }
  {
    BT::BehaviorTreeFactory factory;
    std::shared_ptr<rtc_bt::BtRosBridge> null_bridge;
    RegisterBtNodes(factory, null_bridge);  // grasp present → same tree builds
    EXPECT_NO_THROW(factory.createTreeFromText(grasp_tree));
  }
}
