// file: include/ur5e_bt_coordinator/bt_node_registration.hpp
#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <array>
#include <memory>
#include <string_view>

namespace rtc_bt {

/// Registration names of the action nodes that resolve a finger key into hand
/// joint indices (see GetFingerJointIndices). A tree containing any of these
/// cannot tick before hand joint_states arrive without throwing BT::RuntimeError
/// out of the tick callback (#161) — the coordinator's readiness gate keys on
/// this list. SetHandPose is deliberately absent: it resolves no finger index
/// (no-feedback pose node), so it neither needs the gate nor triggers it.
///
/// Single source: RegisterBtNodes() below registers these same three names, and
/// both usages live in bt_node_registration.cpp so a rename cannot drift the two
/// apart unnoticed.
inline constexpr std::array<std::string_view, 3> kFingerIndexNodeNames = {
    "MoveFinger", "FlexExtendFinger", "MoveOpposition"};

/// True when `registration_name` is one of kFingerIndexNodeNames.
[[nodiscard]] bool IsFingerIndexNode(std::string_view registration_name);

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
