// locomotion_controller: the velocity command path (the Twist subscription,
// the command filter's timer) and the status topic's publish. All off the RT
// thread. See locomotion_controller.hpp.
#include "integrated_bringup/controllers/locomotion_controller.hpp"
#include "integrated_bringup/support/owned_topics.hpp"
#include "rtc_base/types/types.hpp"

#include <chrono>
#include <cmath>

namespace integrated_bringup {

namespace mpc = rtc::mpc;
using rtc::SteadyNowNs;

// The subscription callback (the node's default group): store the message
// with the steady time it arrived. Nothing else — the filter has one writer.
void LocomotionController::OnVelocityCommand(const geometry_msgs::msg::Twist& msg) noexcept {
  RawVelocityCommandSample sample{};
  sample.twist.linear_x = msg.linear.x;
  sample.twist.linear_y = msg.linear.y;
  sample.twist.linear_z = msg.linear.z;
  sample.twist.angular_x = msg.angular.x;
  sample.twist.angular_y = msg.angular.y;
  sample.twist.angular_z = msg.angular.z;
  sample.recv_steady_ns = SteadyNowNs();
  sample.sequence = raw_command_.Load().sequence + 1;  // this callback is the slot's writer
  raw_command_.Store(sample);
}

// The one writer of the phase manager's command slot (and of the publish
// thread's copy, and of the diagnostic log's): the timer, and
// StartCommandTimer on the same executor thread before the timer exists.
void LocomotionController::PushCommand(const mpc::FilteredCommand& cmd) noexcept {
  if (phase_ != nullptr) {
    phase_->SetCommand(cmd);
  }
  command_out_.Store(cmd);
  LocomotionCommandLogSample& log = command_log_.StartWrite();
  log.v_x = cmd.v_x;
  log.v_y = cmd.v_y;
  log.omega_z = cmd.omega_z;
  log.base_height = cmd.base_height;
  log.fresh = cmd.valid && cmd.fresh;
  log.gait = phase_ != nullptr ? static_cast<std::int32_t>(phase_->CurrentPhaseId()) : -1;
  command_log_.FinishWrite();
}

// The timer (a mutually exclusive group): one filter step per period, with
// whatever the subscription last stored — also when that is stale or nothing
// (the Bessel stage assumes a fixed period; the filter zeroes a stale input).
void LocomotionController::StepCommand() noexcept {
  const RawVelocityCommandSample raw = raw_command_.Load();
  const mpc::FilteredCommand& out =
      command_filter_.Step(raw.twist, raw.recv_steady_ns, SteadyNowNs());
  // The filter clamps every axis and its height integrator is bounded, so a
  // non-finite output cannot come from a finite input; this is the last check
  // before the planner, not a path the tests can reach.
  if (!(std::isfinite(out.v_x) && std::isfinite(out.v_y) && std::isfinite(out.omega_z) &&
        std::isfinite(out.base_height))) {
    RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 1000,
                         "[locomotion] the command filter put out a non-finite value; the "
                         "planner keeps the previous command");
    return;
  }
  PushCommand(out);
}

void LocomotionController::StartCommandTimer() noexcept {
  if (command_timer_ || !node_ || !velocity_sub_) {
    return;
  }
  // A fresh start with the robot: the height back to the stance height, no
  // accepted axis kept from the last session. Reset and Step share this thread
  // (the CM's single-threaded nrt_callback executor runs the lifecycle calls
  // and the timer). The reset cannot fail: Init accepted this height.
  const auto& cfg = command_filter_.config();
  static_cast<void>(command_filter_.Reset(cfg.base_height));
  // A message from the last session is not a command for this one (the
  // callback runs on this thread too, so this is the slot's one writer now).
  raw_command_.Store(RawVelocityCommandSample{});
  PushCommand(command_filter_.output());
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / cfg.sample_rate_hz));
  command_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  command_timer_ = node_->create_wall_timer(period, [this]() { StepCommand(); }, command_cb_group_);
}

void LocomotionController::StopCommandTimer() noexcept {
  command_timer_.reset();
  command_cb_group_.reset();
}

// On the thread that owns mpc_thread_ (the 1 Hz timer, the lifecycle calls):
// the publish thread reads the SeqLock, so the thread can go away under it.
void LocomotionController::StoreMpcCounters() noexcept {
  LocomotionMpcCounters c{};
  if (const auto* t = mpc_thread_.get()) {
    c.solves = t->TotalSolves();
    c.failed = t->FailedSolves();
    c.deadline_miss = t->DeadlineMissCount();
    c.withheld = t->WithheldCount();
    c.overrun = t->OnlineOverrunCount();
  }
  mpc_counters_.Store(c);
}

void LocomotionController::PublishNonRtSnapshot(const rtc::PublishSnapshot& snap) noexcept {
  PublishOwnedTopicsFromSnapshot(snap, owned_topics_);
  PublishLocomotionStatus(owned_topics_, status_lock_.Load(), mpc_enabled_, command_out_.Load(),
                          mpc_counters_.Load());
}

}  // namespace integrated_bringup
