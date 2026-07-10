// file: include/ur5e_bt_coordinator/topic_namer.hpp
#pragma once

#include <string>

namespace rtc_bt {

/// Robot-agnostic topic-name factory (Seam A).
///
/// Centralizes every arm/hand device-group token that appears in the topic
/// paths the BT coordinator subscribes/publishes to. The default
/// {arm_group="ur5e", hand_group="hand"} reproduces the legacy hardcoded
/// strings byte-for-byte; a different profile (e.g. hand_group="p1b") swaps
/// only the group segment so the coordinator drives another robot without
/// touching call sites.
///
/// Two families of topics:
///   - Controller-manager per-group joint states live at a fixed path
///     (/rtc_cm/<group>/joint_states) independent of the active controller.
///   - Controller-owned topics live under the active controller namespace
///     `ns` (= "/" + <active_controller_name>), rebound on every controller
///     switch (see BtRosBridge::RewireControllerTopics).
struct TopicNamer {
  std::string arm_group{"ur5e"};
  std::string hand_group{"hand"};

  // ── Controller-manager per-group joint states (fixed path) ────────────────
  std::string ArmJointStates() const { return "/rtc_cm/" + arm_group + "/joint_states"; }

  std::string HandJointStates() const { return "/rtc_cm/" + hand_group + "/joint_states"; }

  // ── Controller-owned topics under a controller namespace `ns` ─────────────
  // Pass ns="" for the pre-rewire relative form (used as the initial health
  // watchdog label before any controller has been activated).
  std::string GraspState(const std::string& ns) const {
    return ns + "/" + hand_group + "/grasp_state";
  }

  std::string WbcState(const std::string& ns) const { return ns + "/" + hand_group + "/wbc_state"; }

  // Active controller's broadcast TF (arm-tip `_actual` frames). Controller-
  // owned but NOT group-scoped — it is the whole controller's TFMessage stream,
  // so no arm/hand token appears (mirrors demo_gui `{ns}/transforms`).
  std::string Transforms(const std::string& ns) const { return ns + "/transforms"; }

  std::string ArmJointGoal(const std::string& ns) const {
    return ns + "/" + arm_group + "/joint_goal";
  }

  std::string HandJointGoal(const std::string& ns) const {
    return ns + "/" + hand_group + "/joint_goal";
  }
};

}  // namespace rtc_bt
