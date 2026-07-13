// ── Publish offload threads (SPSC drain + ROS2 publish) ─────────────────────
#include "rtc_controller_manager/rt_controller_node.hpp"
#include <rtc_base/threading/thread_utils.hpp>

#include <poll.h>         // poll
#include <sched.h>        // sched_yield (fallback if eventfd unavailable)
#include <sys/eventfd.h>  // eventfd_read

namespace urtc = rtc;

// Layout v4: the actuator command lane (former rt_outbound +
// publish_buffer_) was removed. DeviceBackend.WriteCommand is now called
// inline on the rt_control thread at the end of each rt_loop tick (see
// rt_controller_node_rt_loop.cpp). Only the controller-owned non-RT publish
// lane below remains.

// ── nrt_callback thread (controller-owned non-RT publish lane) ──────────────
//
// Drains nrt_publish_buffer_ and forwards controller.PublishNonRtSnapshot —
// the controller owns LifecyclePublishers for kRobotTarget / kRobotTransforms
// / kDigitalTwinState (plus any owned topics behind owned_topics.cpp). These
// publishes are outside the controller↔hardware RT boundary, so they ride a
// non-RT consumer on the nrt_callback core (SCHED_OTHER nice 0).

void RtControllerNode::NrtPublishLoopEntry(const urtc::ThreadConfig& cfg) {
  static_cast<void>(urtc::ApplyThreadConfigVerbose(cfg));
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
