#include "ur5e_bringup/controllers/owned_topics.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace ur5e_bringup {

namespace {

// Pre-populate GuiPosition joint_names + joint_positions vectors from the
// controller's device_name_configs_ so publish() never resizes.
void PrefillGuiMessage(const rtc::DeviceNameConfig *cfg,
                       rtc_msgs::msg::GuiPosition &msg) {
  if (cfg == nullptr) {
    return;
  }
  msg.joint_names.assign(cfg->joint_state_names.begin(),
                         cfg->joint_state_names.end());
  msg.joint_positions.assign(cfg->joint_state_names.size(), 0.0);
}

// Pre-populate GraspState per-finger arrays to kMaxFingertips so publish()
// never resizes on the hot path.
void PrefillGraspMessage(const rtc::DeviceNameConfig *cfg,
                         rtc_msgs::msg::GraspState &msg) {
  if (cfg != nullptr) {
    msg.fingertip_names.assign(cfg->sensor_names.begin(),
                               cfg->sensor_names.end());
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
void PrefillWbcMessage(const rtc::DeviceNameConfig *cfg,
                       rtc_msgs::msg::WbcState &msg) {
  if (cfg != nullptr) {
    msg.fingertip_names.assign(cfg->sensor_names.begin(),
                               cfg->sensor_names.end());
  }
  const auto max_ft = static_cast<std::size_t>(rtc::kMaxFingertips);
  msg.force_magnitude.assign(max_ft, 0.0F);
  msg.contact_flag.assign(max_ft, 0.0F);
  msg.displacement.assign(max_ft, 0.0F);
}

} // namespace

void CreateOwnedTopics(rtc::RTControllerInterface &ctrl,
                       ControllerTopicHandles &handles) {
  auto node = ctrl.get_lifecycle_node();
  if (!node) {
    throw std::runtime_error(
        "CreateOwnedTopics: controller has no LifecycleNode (on_configure not "
        "yet called?)");
  }

  int group_idx = 0;
  for (const auto &[group_name, group] : ctrl.GetTopicConfig().groups) {
    const auto gi = static_cast<std::size_t>(group_idx);

    for (const auto &sub : group.subscribe) {
      if (sub.ownership != rtc::TopicOwnership::kController) {
        continue;
      }
      if (sub.role == rtc::SubscribeRole::kTarget && gi < kMaxOwnedGroups) {
        const std::string name_capture = group_name;
        const int idx_capture = group_idx;
        handles.target_subs[gi] =
            node->create_subscription<rtc_msgs::msg::RobotTarget>(
                sub.topic_name, 10,
                [&ctrl, name_capture,
                 idx_capture](rtc_msgs::msg::RobotTarget::SharedPtr msg) {
                  ctrl.DeliverTargetMessage(name_capture, idx_capture, *msg);
                });
      }
    }

    for (const auto &pub : group.publish) {
      if (pub.ownership != rtc::TopicOwnership::kController) {
        continue;
      }
      switch (pub.role) {
      case rtc::PublishRole::kGuiPosition: {
        if (gi >= kMaxOwnedGroups) {
          break;
        }
        handles.gui_pubs[gi] =
            node->create_publisher<rtc_msgs::msg::GuiPosition>(pub.topic_name,
                                                               10);
        PrefillGuiMessage(ctrl.GetDeviceNameConfig(group_name),
                          handles.gui_msgs[gi]);
        handles.gui_group_idx[gi] = group_idx;
        break;
      }
      case rtc::PublishRole::kGraspState: {
        rclcpp::QoS grasp_qos{10};
        handles.grasp_pub = node->create_publisher<rtc_msgs::msg::GraspState>(
            pub.topic_name, grasp_qos);
        PrefillGraspMessage(ctrl.GetDeviceNameConfig(group_name),
                            handles.grasp_msg);
        handles.grasp_group_idx = group_idx;
        break;
      }
      case rtc::PublishRole::kToFSnapshot: {
        rclcpp::QoS tof_qos{5};
        tof_qos.best_effort();
        handles.tof_pub = node->create_publisher<rtc_msgs::msg::ToFSnapshot>(
            pub.topic_name, tof_qos);
        handles.tof_group_idx = group_idx;
        break;
      }
      case rtc::PublishRole::kWbcState: {
        rclcpp::QoS wbc_qos{10};
        handles.wbc_pub = node->create_publisher<rtc_msgs::msg::WbcState>(
            pub.topic_name, wbc_qos);
        PrefillWbcMessage(ctrl.GetDeviceNameConfig(group_name),
                          handles.wbc_msg);
        handles.wbc_group_idx = group_idx;
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

void ActivateOwnedTopics(const rclcpp_lifecycle::State & /*prev*/,
                         ControllerTopicHandles &handles) noexcept {
  for (auto &pub : handles.gui_pubs) {
    if (pub) {
      pub->on_activate();
    }
  }
  if (handles.grasp_pub) {
    handles.grasp_pub->on_activate();
  }
  if (handles.tof_pub) {
    handles.tof_pub->on_activate();
  }
  if (handles.wbc_pub) {
    handles.wbc_pub->on_activate();
  }
}

void DeactivateOwnedTopics(const rclcpp_lifecycle::State & /*prev*/,
                           ControllerTopicHandles &handles) noexcept {
  for (auto &pub : handles.gui_pubs) {
    if (pub) {
      pub->on_deactivate();
    }
  }
  if (handles.grasp_pub) {
    handles.grasp_pub->on_deactivate();
  }
  if (handles.tof_pub) {
    handles.tof_pub->on_deactivate();
  }
  if (handles.wbc_pub) {
    handles.wbc_pub->on_deactivate();
  }
}

void ResetOwnedTopics(ControllerTopicHandles &handles) noexcept {
  for (auto &sub : handles.target_subs) {
    sub.reset();
  }
  for (auto &pub : handles.gui_pubs) {
    pub.reset();
  }
  handles.gui_group_idx.fill(-1);
  handles.grasp_pub.reset();
  handles.grasp_group_idx = -1;
  handles.tof_pub.reset();
  handles.tof_group_idx = -1;
  handles.wbc_pub.reset();
  handles.wbc_group_idx = -1;
}

void PublishOwnedTopicsFromSnapshot(const rtc::PublishSnapshot &snap,
                                    ControllerTopicHandles &handles) noexcept {
  const auto sec = static_cast<int32_t>(snap.stamp_ns / 1'000'000'000L);
  const auto nsec = static_cast<uint32_t>(snap.stamp_ns % 1'000'000'000L);

  // ── GUI position (per device group) ───────────────────────────────────
  for (std::size_t slot = 0; slot < kMaxOwnedGroups; ++slot) {
    auto &pub = handles.gui_pubs[slot];
    const int gi_signed = handles.gui_group_idx[slot];
    if (!pub || gi_signed < 0 ||
        gi_signed >= rtc::PublishSnapshot::kMaxGroups) {
      continue;
    }
    const auto gi = static_cast<std::size_t>(gi_signed);
    const auto &gc = snap.group_commands[gi];
    auto &msg = handles.gui_msgs[slot];
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nsec;
    const auto n_actual =
        std::min(static_cast<std::size_t>(gc.actual_num_channels),
                 msg.joint_positions.size());
    for (std::size_t i = 0; i < n_actual; ++i) {
      msg.joint_positions[i] = gc.actual_positions[i];
    }
    // task_positions is SE(3) — populated only for the first (robot) group.
    if (gi == 0) {
      std::copy(snap.actual_task_positions.begin(),
                snap.actual_task_positions.end(), msg.task_positions.begin());
    } else {
      msg.task_positions.fill(0.0);
    }
    pub->publish(msg);
  }

  // ── Grasp state ─────────────────────────────────────────────────────
  if (handles.grasp_pub && handles.grasp_group_idx >= 0 &&
      handles.grasp_group_idx < rtc::PublishSnapshot::kMaxGroups) {
    const auto gi = static_cast<std::size_t>(handles.grasp_group_idx);
    const auto &gs = snap.group_commands[gi].grasp_state;
    auto &msg = handles.grasp_msg;
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
    const auto &ws = snap.group_commands[gi].wbc_state;
    auto &msg = handles.wbc_msg;
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
    const auto &ts = snap.group_commands[gi].tof_snapshot;
    if (!ts.populated) {
      return;
    }
    auto &msg = handles.tof_msg;
    msg.stamp.sec = sec;
    msg.stamp.nanosec = nsec;
    const int total_sensors = std::min(ts.num_fingers * ts.sensors_per_finger,
                                       static_cast<int>(msg.distances.size()));
    const int num_poses =
        std::min(ts.num_fingers, static_cast<int>(msg.tip_poses.size()));
    for (int i = 0; i < total_sensors; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      msg.distances[ui] = ts.distances[ui];
      msg.valid[ui] = ts.valid[ui];
    }
    for (int f = 0; f < num_poses; ++f) {
      const auto fi = static_cast<std::size_t>(f);
      const auto &src = ts.tip_poses[fi];
      auto &dst = msg.tip_poses[fi];
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

} // namespace ur5e_bringup
