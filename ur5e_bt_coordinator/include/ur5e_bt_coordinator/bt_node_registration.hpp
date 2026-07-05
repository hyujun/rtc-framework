// file: include/ur5e_bt_coordinator/bt_node_registration.hpp
#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <memory>

namespace rtc_bt {

/// Optional-sensor capabilities of a robot (Seam D). Each flag gates a group
/// of BT node types at *registration* time, so a robot that lacks a sensor
/// never exposes nodes that would read absent data — a tree referencing a
/// gated-out node then fails to build with the factory's "unknown node" error
/// instead of ticking a node with no backing data.
///
/// Default = all present, which reproduces the legacy register-everything
/// behavior byte-for-byte (the ur5e/hand profile has all three sensors).
struct RobotCapabilities {
  /// grasp-force / grasp-state feedback (GraspControl, IsForceAbove, IsGrasped,
  /// IsGraspPhase — all read the Force-PI grasp_state feedback path).
  bool has_grasp_sensing{true};
  /// ToF proximity sensors (StartToFCollection, StopToFCollection,
  /// ProcessSearchData — the latter consumes the collected ToF buffer).
  bool has_tof{true};
  /// external shape-estimation pipeline (TriggerShapeEstimation,
  /// WaitShapeResult, CheckShapeType).
  bool has_shape{true};
};

/// Register every BT node type into `factory`, skipping the capability-gated
/// groups whose flag is false. `bridge` is shared with all bridge-backed nodes
/// (may be null for offline validation, matching the legacy call sites).
void RegisterBtNodes(BT::BehaviorTreeFactory& factory, const std::shared_ptr<BtRosBridge>& bridge,
                     const RobotCapabilities& capabilities = {});

}  // namespace rtc_bt
