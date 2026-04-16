// ── Global E-Stop trigger and clear ──────────────────────────────────────────
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <cstdio> // std::snprintf
#include <string_view>

namespace urtc = rtc;

// ── Global E-Stop
// ──────────────────────────────────────────────────────────────
void RtControllerNode::TriggerGlobalEstop(std::string_view reason) noexcept {
  // Idempotent — only the first call logs and propagates.
  bool expected = false;
  if (!global_estop_.compare_exchange_strong(expected, true,
                                             std::memory_order_acq_rel,
                                             std::memory_order_relaxed)) {
    return; // already estopped
  }

  // Fixed-size copy — no heap allocation on the RT path.
  std::snprintf(estop_reason_.data(), estop_reason_.size(), "%.*s",
                static_cast<int>(reason.size()), reason.data());

  // Propagate to all controllers — TriggerEstop is safe from any thread.
  for (auto &ctrl : controllers_) {
    ctrl->TriggerEstop();
    ctrl->SetHandEstop(true);
  }
  PublishEstopStatus(true);

  // Defer the RCLCPP_ERROR to the non-RT log thread (DrainLog).
  estop_log_pending_.store(true, std::memory_order_release);
}

void RtControllerNode::ClearGlobalEstop() noexcept {
  if (!global_estop_.load(std::memory_order_acquire)) {
    return;
  }
  global_estop_.store(false, std::memory_order_release);

  for (auto &ctrl : controllers_) {
    ctrl->ClearEstop();
    ctrl->SetHandEstop(false);
  }
  PublishEstopStatus(false);

  // Defer the RCLCPP_INFO to the non-RT log thread (DrainLog).
  estop_log_pending_.store(true, std::memory_order_release);
}
