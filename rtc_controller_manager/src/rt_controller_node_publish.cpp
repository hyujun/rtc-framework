// ── Publish offload threads (SPSC drains + ROS2 publish) ─────────────────────
#include "rtc_controller_manager/rt_controller_node.hpp"
#include <rtc_base/threading/thread_utils.hpp>

#include <poll.h>         // poll
#include <sched.h>        // sched_yield (fallback if eventfd unavailable)
#include <sys/eventfd.h>  // eventfd_read

namespace urtc = rtc;

// ── rt_outbound thread (actuator command lane) ──────────────────────────────
//
// Drains publish_buffer_ and forwards DeviceBackend.WriteCommand calls only
// (controller↔hardware boundary, RT). After Phase 4 this thread is promoted
// to SCHED_FIFO 65 and shares its core with rt_inbound (FIFO 70 has priority).
// Controller-owned non-RT publishes (RobotTarget / Transforms / DigitalTwin)
// were peeled off into the nrt_callback thread below — see Phase 2 of the
// thread-layout-v3 sprint.

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
    // each group's slot. Controller-owned non-RT topics ride a separate lane
    // drained by NrtPublishLoopEntry below.
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

// ── nrt_callback thread (controller-owned non-RT publish lane) ──────────────
//
// Drains nrt_publish_buffer_ and forwards controller.PublishNonRtSnapshot —
// the controller owns LifecyclePublishers for kRobotTarget / kRobotTransforms
// / kDigitalTwinState (plus any owned topics behind owned_topics.cpp). These
// publishes are outside the controller↔hardware RT boundary, so they ride a
// non-RT consumer on the nrt_callback core (SCHED_OTHER nice 0).

void RtControllerNode::NrtPublishLoopEntry(const urtc::ThreadConfig& cfg) {
  static_cast<void>(urtc::ApplyThreadConfig(cfg));
  nrt_publish_running_.store(true, std::memory_order_release);

  urtc::PublishSnapshot snap{};

  while (nrt_publish_running_.load(std::memory_order_acquire) && rclcpp::ok()) {
    if (!nrt_publish_buffer_.Pop(snap)) {
      WaitForNrtPublishWakeup();
      continue;
    }

    // Forward the snapshot to the active controller so it can publish its
    // own (controller-owned) topics. Index is captured by the RT loop this
    // tick so the controller pulled out of controllers_ matches the topic
    // config that produced the snapshot.
    const auto aidx = static_cast<std::size_t>(snap.active_controller_idx);
    if (aidx < controllers_.size() && controllers_[aidx]) {
      controllers_[aidx]->PublishNonRtSnapshot(snap);
    }
  }
}

void RtControllerNode::WaitForNrtPublishWakeup() {
  if (nrt_publish_eventfd_ >= 0) {
    struct pollfd pfd {};

    pfd.fd = nrt_publish_eventfd_;
    pfd.events = POLLIN;
    poll(&pfd, 1, 1);  // 1ms timeout
    if (pfd.revents & POLLIN) {
      eventfd_t val{};
      static_cast<void>(eventfd_read(nrt_publish_eventfd_, &val));
    }
  } else {
    sched_yield();
  }
}

void RtControllerNode::StartNrtPublishLoop(const urtc::ThreadConfig& nrt_pub_cfg) {
  nrt_publish_thread_ = std::jthread([this, nrt_pub_cfg]() { NrtPublishLoopEntry(nrt_pub_cfg); });
}

void RtControllerNode::StopNrtPublishLoop() {
  nrt_publish_running_.store(false, std::memory_order_release);
  if (nrt_publish_thread_.joinable()) {
    nrt_publish_thread_.join();
  }
}

void RtControllerNode::PublishEstopStatus(bool estopped) {
  std_msgs::msg::Bool msg;
  msg.data = estopped;
  estop_pub_->publish(msg);
}
