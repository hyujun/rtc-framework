// ── Publish offload thread (SPSC drain + ROS2 publish) ───────────────────────
#include "rtc_controller_manager/rt_controller_node.hpp"
#include <rtc_base/threading/thread_utils.hpp>

#include <poll.h>         // poll
#include <sched.h>        // sched_yield (fallback if eventfd unavailable)
#include <sys/eventfd.h>  // eventfd_read

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

    // Per-group device command publish is delegated to the backend bound to
    // each group's slot. Controller-output topics (kRobotTarget /
    // kRobotTransforms / kDigitalTwinState — plus the SeqLock-routed
    // GraspState / WbcState / ToFSnapshot) are forwarded via
    // PublishNonRtSnapshot below — CM publishes nothing for those roles.
    const auto& slot_mapping =
        controller_slot_mappings_[static_cast<std::size_t>(snap.active_controller_idx)];
    std::size_t group_idx = 0;
    for ([[maybe_unused]] const auto& [group_name, group] : active_tc.groups) {
      if (group_idx >= static_cast<std::size_t>(urtc::PublishSnapshot::kMaxGroups))
        break;
      const int slot = slot_mapping.slots[group_idx];
      if (slot >= 0 && slot < kMaxDevices && backends_[static_cast<std::size_t>(slot)]) {
        backends_[static_cast<std::size_t>(slot)]->WriteCommand(snap.group_commands[group_idx],
                                                                snap.command_type);
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
