// ── Publish offload threads (SPSC drain + ROS2 publish) ─────────────────────
#include "rtc_controller_manager/rt_controller_node.hpp"
#include <rtc_base/threading/thread_utils.hpp>
#include <rtc_base/tracing/trace_scope.hpp>

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
// the controller owns the LifecyclePublisher for kRobotTransforms (the only
// PublishRole after issue #196 Phase 5) plus any topics it creates directly
// via the SeqLock<T> + Setup*Publisher pattern (owned_topics.cpp). These
// publishes are outside the controller↔hardware RT boundary, so they ride a
// non-RT consumer on the nrt_callback core (SCHED_OTHER nice 0).

void RtControllerNode::NrtPublishLoopEntry(const urtc::ThreadConfig& cfg) {
  static_cast<void>(urtc::ApplyThreadConfigVerbose(cfg));
  nrt_publish_running_.store(true, std::memory_order_release);

  urtc::PublishSnapshot snap{};

  while (nrt_publish_running_.load(std::memory_order_acquire) && rclcpp::ok()) {
    bool did_work = false;

    if (nrt_publish_buffer_.Pop(snap)) {
      // L1 span covers only the drained-work section — the 1 ms poll wait
      // below stays outside so idle polling does not pollute the slice
      // statistics.
      RTC_TRACE_SCOPE("nrt_publish_drain");

      // Forward the snapshot to the active controller so it can publish its
      // own (controller-owned) topics. Index is captured by the RT loop this
      // tick so the controller pulled out of controllers_ matches the topic
      // config that produced the snapshot.
      const auto aidx = static_cast<std::size_t>(snap.active_controller_idx);
      if (aidx < controllers_.size() && controllers_[aidx]) {
        controllers_[aidx]->PublishNonRtSnapshot(snap);
      }
      did_work = true;
    }

    // The twin lane is signalled by the device state callbacks, not by the RT
    // tick, so it is drained unconditionally rather than as an else-branch —
    // a tick-less arrival must not wait for the next snapshot to be noticed.
    did_work = DrainDigitalTwin() || did_work;

    if (!did_work) {
      WaitForNrtPublishWakeup();
    }
  }
}

bool RtControllerNode::DrainDigitalTwin() {
  bool published = false;

  for (std::size_t slot = 0; slot < dt_dirty_.size(); ++slot) {
    if (!dt_dirty_[slot].exchange(false, std::memory_order_acquire)) {
      continue;
    }
    auto& dte = digital_twin_by_slot_[slot];
    // A state message can arrive during on_configure — the mock/UR driver
    // already streams /joint_states before on_activate flips the
    // LifecyclePublisher — so a not-yet-activated publisher drops the sample
    // instead of warning on every startup. The bit is already cleared; the
    // next message re-raises it.
    if (!dte.publisher || !dte.publisher->is_activated() || !backends_[slot]) {
      continue;
    }

    // Read straight from the backend's SeqLock (lock-free single-writer /
    // multi-reader), so the twin always carries the newest state rather than
    // whatever a queue happened to hold. Coalesced bursts lose samples, never
    // ordering or freshness.
    urtc::DeviceStateCache cache{};
    static_cast<void>(backends_[slot]->ReadState(cache));
    const auto n = dte.msg.position.size();
    for (std::size_t i = 0; i < n; ++i) {
      dte.msg.position[i] = cache.positions[i];
      dte.msg.velocity[i] = cache.velocities[i];
      dte.msg.effort[i] = cache.efforts[i];
    }
    dte.publisher->publish(dte.msg);
    published = true;
  }
  return published;
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
