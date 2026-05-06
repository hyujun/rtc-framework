// ── Publish offload thread (SPSC drain + ROS2 publish) ───────────────────────
#include "rtc_controller_manager/rt_controller_node.hpp"
#include <rtc_base/threading/thread_utils.hpp>

#include <poll.h>         // poll
#include <sched.h>        // sched_yield (fallback if eventfd unavailable)
#include <sys/eventfd.h>  // eventfd_read

#include <algorithm>

namespace urtc = rtc;

// ── Publish offload thread
// ────────────────────────────────────────────────────
//
// Drains the SPSC publish buffer and performs all ROS2 publish() calls.
// Runs on a non-RT core (Core 5/6, SCHED_OTHER nice -3).
// All DDS serialization, string allocation, and sendto() syscalls happen here,
// keeping the RT path free of unbounded-latency operations.

void RtControllerNode::PublishLoopEntry(const urtc::ThreadConfig& cfg) {
  static_cast<void>(urtc::ApplyThreadConfig(cfg));
  publish_running_.store(true, std::memory_order_release);

  urtc::PublishSnapshot snap{};

  while (publish_running_.load(std::memory_order_acquire) && rclcpp::ok()) {
    if (!publish_buffer_.Pop(snap)) {
      WaitForPublishWakeup();
      continue;
    }

    const auto& active_tc =
        controller_topic_configs_[static_cast<std::size_t>(snap.active_controller_idx)];

    // Shared timestamp for JointCommand messages
    const auto sec = static_cast<int32_t>(snap.stamp_ns / 1'000'000'000L);
    const auto nsec = static_cast<uint32_t>(snap.stamp_ns % 1'000'000'000L);
    const char* cmd_type_str =
        (snap.command_type == urtc::CommandType::kTorque) ? "torque" : "position";

    // Publish all device groups uniformly (manager-owned entries only —
    // controller-owned entries are skipped; the controller forwards them
    // via PublishNonRtSnapshot below).
    std::size_t group_idx = 0;
    for ([[maybe_unused]] const auto& [group_name, group] : active_tc.groups) {
      for (const auto& entry : group.publish) {
        if (entry.ownership == urtc::TopicOwnership::kController)
          continue;
        switch (entry.role) {
          case urtc::PublishRole::kJointCommand:
            PublishJointCommandEntry(snap, entry, group_idx, sec, nsec, cmd_type_str);
            break;
          case urtc::PublishRole::kRos2Command:
            PublishRos2CommandEntry(snap, entry, group_idx);
            break;
          default:
            // Controller-output roles (kRobotTarget / kGraspState /
            // kToFSnapshot / kWbcState / kRobotTransforms) are owned by the
            // active controller's LifecycleNode and forwarded via
            // PublishNonRtSnapshot below — CM has no publisher for them.
            // CreatePublishers() throws on misconfigured manager-owned
            // controller-output roles, so reaching here means an entry that
            // CM correctly skipped at publisher creation time.
            break;
        }
      }
      ++group_idx;
    }

    // Forward the snapshot to the active controller so it can publish its
    // own (controller-owned) topics via LifecyclePublishers it created in
    // on_configure / activated in on_activate.  Index is read from the
    // snapshot (fixed by the RT loop this tick) to stay consistent with
    // active_tc above.
    const auto aidx = static_cast<std::size_t>(snap.active_controller_idx);
    if (aidx < controllers_.size() && controllers_[aidx]) {
      controllers_[aidx]->PublishNonRtSnapshot(snap);
    }
  }
}

void RtControllerNode::WaitForPublishWakeup() {
  // Wait for RT thread signal via eventfd (or 1ms polling fallback)
  if (publish_eventfd_ >= 0) {
    struct pollfd pfd {};

    pfd.fd = publish_eventfd_;
    pfd.events = POLLIN;
    poll(&pfd, 1, 1);  // 1ms timeout
    if (pfd.revents & POLLIN) {
      eventfd_t val{};
      static_cast<void>(eventfd_read(publish_eventfd_, &val));
    }
  } else {
    sched_yield();
  }
}

void RtControllerNode::PublishJointCommandEntry(const urtc::PublishSnapshot& snap,
                                                const urtc::PublishTopicEntry& entry,
                                                std::size_t group_idx, int32_t sec, uint32_t nsec,
                                                const char* cmd_type_str) {
  const auto& gc = snap.group_commands[group_idx];
  const int nc = gc.num_channels;
  // Skip publishing when the controller has no output for this device
  // (nc=0). This prevents sending zero-filled commands before the device
  // state is known (e.g. hand not yet valid).
  if (nc <= 0) {
    return;
  }
  auto jc_it = joint_command_publishers_.find(entry.topic_name);
  if (jc_it == joint_command_publishers_.end()) {
    return;
  }
  auto& jce = jc_it->second;
  jce.msg.header.stamp.sec = sec;
  jce.msg.header.stamp.nanosec = nsec;
  jce.msg.command_type = cmd_type_str;
  const auto n = std::min(static_cast<std::size_t>(nc), jce.msg.values.size());
  if (!jce.reorder_map.empty()) {
    // Reorder from joint_state_names order → joint_command_names order
    for (std::size_t i = 0; i < n; ++i) {
      const int src = (i < jce.reorder_map.size()) ? jce.reorder_map[i] : -1;
      jce.msg.values[i] = (src >= 0 && src < nc) ? gc.commands[static_cast<std::size_t>(src)] : 0.0;
    }
  } else {
    for (std::size_t i = 0; i < n; ++i) {
      jce.msg.values[i] = gc.commands[i];
    }
  }
  jce.publisher->publish(jce.msg);
}

void RtControllerNode::PublishRos2CommandEntry(const urtc::PublishSnapshot& snap,
                                               const urtc::PublishTopicEntry& entry,
                                               std::size_t group_idx) {
  const auto& gc = snap.group_commands[group_idx];
  const int nc = gc.num_channels;
  auto it = ros2_command_publishers_.find(entry.topic_name);
  if (it == ros2_command_publishers_.end()) {
    return;
  }
  auto& pe = it->second;
  const auto n = std::min(static_cast<std::size_t>(nc), pe.msg.data.size());
  if (!pe.reorder_map.empty()) {
    // Reorder from joint_state_names order → joint_command_names order
    for (std::size_t i = 0; i < n; ++i) {
      const int src = (i < pe.reorder_map.size()) ? pe.reorder_map[i] : -1;
      pe.msg.data[i] = (src >= 0 && src < nc) ? gc.commands[static_cast<std::size_t>(src)] : 0.0;
    }
  } else {
    for (std::size_t i = 0; i < n; ++i) {
      pe.msg.data[i] = gc.commands[i];
    }
  }
  pe.publisher->publish(pe.msg);
}

void RtControllerNode::StartPublishLoop(const urtc::ThreadConfig& pub_cfg) {
  publish_thread_ = std::jthread([this, pub_cfg]() { PublishLoopEntry(pub_cfg); });
}

void RtControllerNode::StopPublishLoop() {
  publish_running_.store(false, std::memory_order_release);
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
}

void RtControllerNode::PublishEstopStatus(bool estopped) {
  std_msgs::msg::Bool msg;
  msg.data = estopped;
  estop_pub_->publish(msg);
}
