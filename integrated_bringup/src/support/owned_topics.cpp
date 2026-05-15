#include "integrated_bringup/support/owned_topics.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>

namespace integrated_bringup {

namespace {

// Convention: child frame_id = "<link>_actual" so RViz tf trees can
// distinguish controller-broadcast frames from URDF-derived
// (robot_state_publisher) ones at a glance.
[[nodiscard]] std::string MakeActualChildFrame(const std::string& link) {
  return link + "_actual";
}

// Pre-populate GraspState per-finger arrays to kMaxFingertips so publish()
// never resizes on the hot path.
void PrefillGraspMessage(const rtc::DeviceNameConfig* cfg, rtc_msgs::msg::GraspState& msg) {
  if (cfg != nullptr) {
    msg.fingertip_names.assign(cfg->sensor_names.begin(), cfg->sensor_names.end());
  }
  const auto max_ft = static_cast<std::size_t>(rtc::kMaxFingertips);
  msg.force_magnitude.assign(max_ft, 0.0F);
  msg.contact_flag.assign(max_ft, 0.0F);
  msg.inference_valid.assign(max_ft, false);
  msg.finger_s.assign(max_ft, 0.0F);
  msg.finger_filtered_force.assign(max_ft, 0.0F);
  msg.finger_force_error.assign(max_ft, 0.0F);
}

// Pre-populate WbcState per-finger arrays so publish() never resizes.
void PrefillWbcMessage(const rtc::DeviceNameConfig* cfg, rtc_msgs::msg::WbcState& msg) {
  if (cfg != nullptr) {
    msg.fingertip_names.assign(cfg->sensor_names.begin(), cfg->sensor_names.end());
  }
  const auto max_ft = static_cast<std::size_t>(rtc::kMaxFingertips);
  msg.force_magnitude.assign(max_ft, 0.0F);
  msg.contact_flag.assign(max_ft, 0.0F);
  msg.displacement.assign(max_ft, 0.0F);
}

}  // namespace

void CreateOwnedTopics(rtc::RTControllerInterface& ctrl, ControllerTopicHandles& handles) {
  auto node = ctrl.get_lifecycle_node();
  if (!node) {
    throw std::runtime_error(
        "CreateOwnedTopics: controller has no LifecycleNode (on_configure not "
        "yet called?)");
  }

  int group_idx = 0;
  for (const auto& [group_name, group] : ctrl.GetTopicConfig().groups) {
    const auto gi = static_cast<std::size_t>(group_idx);

    for (const auto& sub : group.subscribe) {
      if (sub.ownership != rtc::TopicOwnership::kController) {
        continue;
      }
      // Phase 4 trailing cleanup: SubscribeRole enum dropped — the only
      // remaining subscribe lane is the controller target (RobotTarget).
      if (gi < kMaxOwnedGroups) {
        const std::string name_capture = group_name;
        const int idx_capture = group_idx;
        handles.target_subs[gi] = node->create_subscription<rtc_msgs::msg::RobotTarget>(
            sub.topic_name, 10,
            [&ctrl, name_capture, idx_capture](rtc_msgs::msg::RobotTarget::SharedPtr msg) {
              ctrl.DeliverTargetMessage(name_capture, idx_capture, *msg);
            });
      }
    }

    for (const auto& pub : group.publish) {
      if (pub.ownership != rtc::TopicOwnership::kController) {
        continue;
      }
      switch (pub.role) {
        case rtc::PublishRole::kGraspState: {
          rclcpp::QoS grasp_qos{10};
          handles.grasp_pub =
              node->create_publisher<rtc_msgs::msg::GraspState>(pub.topic_name, grasp_qos);
          PrefillGraspMessage(ctrl.GetDeviceNameConfig(group_name), handles.grasp_msg);
          handles.grasp_group_idx = group_idx;
          break;
        }
        case rtc::PublishRole::kToFSnapshot: {
          rclcpp::QoS tof_qos{5};
          tof_qos.best_effort();
          handles.tof_pub =
              node->create_publisher<rtc_msgs::msg::ToFSnapshot>(pub.topic_name, tof_qos);
          handles.tof_group_idx = group_idx;
          break;
        }
        case rtc::PublishRole::kWbcState: {
          rclcpp::QoS wbc_qos{10};
          handles.wbc_pub =
              node->create_publisher<rtc_msgs::msg::WbcState>(pub.topic_name, wbc_qos);
          PrefillWbcMessage(ctrl.GetDeviceNameConfig(group_name), handles.wbc_msg);
          handles.wbc_group_idx = group_idx;
          break;
        }
        case rtc::PublishRole::kRobotTransforms: {
          // Single TF publisher per controller (D-2). YAML places the entry
          // under the first group (D-10); duplicate entries in other groups
          // are ignored to keep the "1 publisher per controller" invariant.
          if (handles.tf_pub) {
            break;
          }
          rclcpp::QoS tf_qos{10};
          tf_qos.reliable();  // D-7: display purpose, not control
          handles.tf_pub = node->create_publisher<tf2_msgs::msg::TFMessage>(pub.topic_name, tf_qos);
          // Reserve capacity once; controller fills tf_slots[] later via
          // AppendArmTipSlot / AppendHandTipSlots / AppendVirtualTcpSlot.
          handles.tf_msg.transforms.reserve(kMaxControllerTransforms);
          break;
        }
        default:
          // Controller-owned role we don't handle yet — ignore rather than
          // throw so that future additions are opt-in.
          break;
      }
    }
    ++group_idx;
  }
}

// ── TfFrameSlot append helpers ───────────────────────────────────────────

bool AppendArmTipSlot(ControllerTopicHandles& handles, const std::string& parent_frame,
                      const std::string& child_link, int group_idx) {
  if (static_cast<std::size_t>(handles.num_tf_slots) >= kMaxControllerTransforms) {
    return false;
  }
  auto& slot = handles.tf_slots[static_cast<std::size_t>(handles.num_tf_slots)];
  slot.parent_frame_id = parent_frame;
  slot.child_frame_id = MakeActualChildFrame(child_link);
  slot.source = TfFrameSlot::Source::kArmTip;
  slot.group_idx = group_idx;
  slot.source_index = 0;
  slot.slot_valid = true;
  ++handles.num_tf_slots;
  return true;
}

void AppendHandTipSlots(ControllerTopicHandles& handles, const std::string& parent_frame,
                        const std::vector<std::string>& tip_links, int group_idx) {
  for (std::size_t i = 0; i < tip_links.size(); ++i) {
    if (static_cast<std::size_t>(handles.num_tf_slots) >= kMaxControllerTransforms) {
      return;
    }
    auto& slot = handles.tf_slots[static_cast<std::size_t>(handles.num_tf_slots)];
    slot.parent_frame_id = parent_frame;
    slot.child_frame_id = MakeActualChildFrame(tip_links[i]);
    slot.source = TfFrameSlot::Source::kHandTip;
    slot.group_idx = group_idx;
    slot.source_index = static_cast<int>(i);
    slot.slot_valid = true;
    ++handles.num_tf_slots;
  }
}

bool AppendVirtualTcpSlot(ControllerTopicHandles& handles, const std::string& parent_frame,
                          int group_idx) {
  if (static_cast<std::size_t>(handles.num_tf_slots) >= kMaxControllerTransforms) {
    return false;
  }
  auto& slot = handles.tf_slots[static_cast<std::size_t>(handles.num_tf_slots)];
  slot.parent_frame_id = parent_frame;
  slot.child_frame_id = "virtual_tcp_actual";
  slot.source = TfFrameSlot::Source::kVirtualTcp;
  slot.group_idx = group_idx;
  slot.source_index = 0;
  slot.slot_valid = true;
  ++handles.num_tf_slots;
  return true;
}

bool AppendCustomPlaceholderSlot(ControllerTopicHandles& handles, const std::string& parent_frame,
                                 const std::string& child_frame) {
  if (static_cast<std::size_t>(handles.num_tf_slots) >= kMaxControllerTransforms) {
    return false;
  }
  auto& slot = handles.tf_slots[static_cast<std::size_t>(handles.num_tf_slots)];
  slot.parent_frame_id = parent_frame;
  slot.child_frame_id = child_frame;
  slot.source = TfFrameSlot::Source::kCustom;
  slot.group_idx = 0;
  slot.source_index = 0;
  slot.slot_valid = false;  // placeholder — controller activates later
  ++handles.num_tf_slots;
  return true;
}

void ActivateOwnedTopics(const rclcpp_lifecycle::State& /*prev*/,
                         ControllerTopicHandles& handles) noexcept {
  if (handles.grasp_pub) {
    handles.grasp_pub->on_activate();
  }
  if (handles.tof_pub) {
    handles.tof_pub->on_activate();
  }
  if (handles.wbc_pub) {
    handles.wbc_pub->on_activate();
  }
  if (handles.tf_pub) {
    handles.tf_pub->on_activate();
  }
}

void DeactivateOwnedTopics(const rclcpp_lifecycle::State& /*prev*/,
                           ControllerTopicHandles& handles) noexcept {
  if (handles.grasp_pub) {
    handles.grasp_pub->on_deactivate();
  }
  if (handles.tof_pub) {
    handles.tof_pub->on_deactivate();
  }
  if (handles.wbc_pub) {
    handles.wbc_pub->on_deactivate();
  }
  if (handles.tf_pub) {
    handles.tf_pub->on_deactivate();
  }
}

void ResetOwnedTopics(ControllerTopicHandles& handles) noexcept {
  for (auto& sub : handles.target_subs) {
    sub.reset();
  }
  handles.grasp_pub.reset();
  handles.grasp_group_idx = -1;
  handles.tof_pub.reset();
  handles.tof_group_idx = -1;
  handles.wbc_pub.reset();
  handles.wbc_group_idx = -1;
  handles.tf_pub.reset();
  handles.tf_msg.transforms.clear();
  for (auto& slot : handles.tf_slots) {
    slot.parent_frame_id.clear();
    slot.child_frame_id.clear();
    slot.slot_valid = false;
  }
  handles.num_tf_slots = 0;
}

void PublishOwnedTopicsFromSnapshot(const rtc::PublishSnapshot& snap,
                                    ControllerTopicHandles& handles) noexcept {
  const auto sec = static_cast<int32_t>(snap.stamp_ns / 1'000'000'000L);
  const auto nsec = static_cast<uint32_t>(snap.stamp_ns % 1'000'000'000L);

  // ── Grasp state ─────────────────────────────────────────────────────
  if (handles.grasp_pub && handles.grasp_group_idx >= 0 &&
      handles.grasp_group_idx < rtc::PublishSnapshot::kMaxGroups) {
    const auto gi = static_cast<std::size_t>(handles.grasp_group_idx);
    const auto& gs = snap.group_commands[gi].grasp_state;
    auto& msg = handles.grasp_msg;
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nsec;
    const auto nf = static_cast<std::size_t>(
        std::min(gs.num_fingertips, static_cast<int>(rtc::kMaxFingertips)));
    for (std::size_t i = 0; i < nf; ++i) {
      msg.force_magnitude[i] = gs.force_magnitude[i];
      msg.contact_flag[i] = gs.contact_flag[i];
      msg.inference_valid[i] = gs.inference_valid[i];
      msg.finger_s[i] = gs.finger_s[i];
      msg.finger_filtered_force[i] = gs.finger_filtered_force[i];
      msg.finger_force_error[i] = gs.finger_force_error[i];
    }
    msg.num_active_contacts = gs.num_active_contacts;
    msg.max_force = gs.max_force;
    msg.grasp_detected = gs.grasp_detected;
    msg.force_threshold = gs.force_threshold;
    msg.min_fingertips = gs.min_fingertips_for_grasp;
    msg.grasp_phase = gs.grasp_phase;
    msg.grasp_target_force = gs.grasp_target_force;
    handles.grasp_pub->publish(msg);
  }

  // ── WBC state ───────────────────────────────────────────────────────
  if (handles.wbc_pub && handles.wbc_group_idx >= 0 &&
      handles.wbc_group_idx < rtc::PublishSnapshot::kMaxGroups) {
    const auto gi = static_cast<std::size_t>(handles.wbc_group_idx);
    const auto& ws = snap.group_commands[gi].wbc_state;
    auto& msg = handles.wbc_msg;
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nsec;
    const auto nf = static_cast<std::size_t>(
        std::min(ws.num_fingertips, static_cast<int>(rtc::kMaxFingertips)));
    for (std::size_t i = 0; i < nf; ++i) {
      msg.force_magnitude[i] = ws.force_magnitude[i];
      msg.contact_flag[i] = ws.contact_flag[i];
      msg.displacement[i] = ws.displacement[i];
    }
    msg.phase = ws.phase;
    msg.num_active_contacts = ws.num_active_contacts;
    msg.max_force = ws.max_force;
    msg.grasp_target_force = ws.grasp_target_force;
    msg.grasp_detected = ws.grasp_detected;
    msg.min_fingertips = ws.min_fingertips_for_grasp;
    msg.tsid_solve_us = ws.tsid_solve_us;
    msg.tsid_solver_ok = ws.tsid_solver_ok;
    msg.qp_fail_count = ws.qp_fail_count;
    handles.wbc_pub->publish(msg);
  }

  // ── ToF snapshot ────────────────────────────────────────────────────
  if (handles.tof_pub && handles.tof_group_idx >= 0 &&
      handles.tof_group_idx < rtc::PublishSnapshot::kMaxGroups) {
    const auto gi = static_cast<std::size_t>(handles.tof_group_idx);
    const auto& ts = snap.group_commands[gi].tof_snapshot;
    if (ts.populated) {
      auto& msg = handles.tof_msg;
      msg.stamp.sec = sec;
      msg.stamp.nanosec = nsec;
      const int total_sensors =
          std::min(ts.num_fingers * ts.sensors_per_finger, static_cast<int>(msg.distances.size()));
      const int num_poses = std::min(ts.num_fingers, static_cast<int>(msg.tip_poses.size()));
      for (int i = 0; i < total_sensors; ++i) {
        const auto ui = static_cast<std::size_t>(i);
        msg.distances[ui] = ts.distances[ui];
        msg.valid[ui] = ts.valid[ui];
      }
      for (int f = 0; f < num_poses; ++f) {
        const auto fi = static_cast<std::size_t>(f);
        const auto& src = ts.tip_poses[fi];
        auto& dst = msg.tip_poses[fi];
        dst.position.x = src.position[0];
        dst.position.y = src.position[1];
        dst.position.z = src.position[2];
        dst.orientation.w = src.quaternion[0];
        dst.orientation.x = src.quaternion[1];
        dst.orientation.y = src.quaternion[2];
        dst.orientation.z = src.quaternion[3];
      }
      handles.tof_pub->publish(msg);
    }
  }

  // ── Robot transforms (kRobotTransforms) ─────────────────────────────
  // Walk tf_slots[] and emit a TFMessage with one TransformStamped per
  // valid slot whose source pose is also valid in the snapshot. Allocation
  // is bounded by kMaxControllerTransforms (capacity reserved in
  // CreateOwnedTopics) so this stays predictable on the publish thread.
  if (handles.tf_pub && handles.num_tf_slots > 0) {
    auto& tf_msg = handles.tf_msg;
    tf_msg.transforms.clear();
    for (int i = 0; i < handles.num_tf_slots; ++i) {
      const auto& slot = handles.tf_slots[static_cast<std::size_t>(i)];
      if (!slot.slot_valid) {
        continue;
      }
      if (slot.group_idx < 0 || slot.group_idx >= rtc::PublishSnapshot::kMaxGroups) {
        continue;
      }
      const auto gi = static_cast<std::size_t>(slot.group_idx);
      const auto& gc = snap.group_commands[gi];

      const rtc::Pose* src_pose = nullptr;
      switch (slot.source) {
        case TfFrameSlot::Source::kArmTip:
          if (gc.arm_tip_pose_valid) {
            src_pose = &gc.arm_tip_pose;
          }
          break;
        case TfFrameSlot::Source::kHandTip: {
          const auto si = static_cast<std::size_t>(slot.source_index);
          if (si < gc.fingertip_poses.size() && gc.fingertip_pose_valid[si]) {
            src_pose = &gc.fingertip_poses[si];
          }
          break;
        }
        case TfFrameSlot::Source::kVirtualTcp:
          if (gc.virtual_tcp_pose_valid) {
            src_pose = &gc.virtual_tcp_pose;
          }
          break;
        case TfFrameSlot::Source::kWbcTipInBase:
        case TfFrameSlot::Source::kCustom:
          // Phase 3 / D-5 — sources without RT producers yet. Skip.
          break;
      }
      if (src_pose == nullptr) {
        continue;
      }

      geometry_msgs::msg::TransformStamped tfs;
      tfs.header.stamp.sec = sec;
      tfs.header.stamp.nanosec = nsec;
      tfs.header.frame_id = slot.parent_frame_id;
      tfs.child_frame_id = slot.child_frame_id;
      tfs.transform.translation.x = src_pose->position[0];
      tfs.transform.translation.y = src_pose->position[1];
      tfs.transform.translation.z = src_pose->position[2];
      tfs.transform.rotation.w = src_pose->quaternion[0];
      tfs.transform.rotation.x = src_pose->quaternion[1];
      tfs.transform.rotation.y = src_pose->quaternion[2];
      tfs.transform.rotation.z = src_pose->quaternion[3];
      tf_msg.transforms.push_back(tfs);
    }
    if (!tf_msg.transforms.empty()) {
      handles.tf_pub->publish(tf_msg);
    }
  }
}

}  // namespace integrated_bringup
