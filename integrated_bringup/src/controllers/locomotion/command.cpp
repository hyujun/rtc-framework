// locomotion_controller: the velocity command path (the Twist subscription,
// the command filter's timer) and the status topic's publish. All off the RT
// thread. See locomotion_controller.hpp.
#include "integrated_bringup/controllers/locomotion_controller.hpp"
#include "integrated_bringup/support/owned_topics.hpp"
#include "rtc_base/types/types.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <span>

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

// The external wrench subscription callback: the lane's one writer (the node's
// default callback group is mutually exclusive, so never two at once).
void LocomotionController::OnExternalWrench(const geometry_msgs::msg::WrenchStamped& msg) noexcept {
  locomotion::ExternalWrenchSample& sample = external_wrench_.StartWrite();
  sample.wrench = {msg.wrench.force.x,  msg.wrench.force.y,  msg.wrench.force.z,
                   msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z};
  sample.recv_steady_ns = SteadyNowNs();
  sample.sequence = ++external_wrench_count_;
  external_wrench_.FinishWrite();
}

// The Joy subscription callback (`velocity_command.type: joy`, instead of the
// Twist one): the message as a command (joy_command.hpp — zero unless the
// deadman reads 1 and the mapped axes are readable), stored the same way.
// Every message refreshes the receive time, a zero one too: a released
// deadman is a fresh "stop", not silence.
void LocomotionController::OnJoyCommand(const sensor_msgs::msg::Joy& msg) noexcept {
  const RawVelocityCommandSample prev = raw_command_.Load();  // this callback is the writer
  const locomotion::JoyCommandResult r = locomotion::JoyToCommand(
      cfg_.velocity_command_joy_map, joy_scale_, std::span<const float>(msg.axes),
      std::span<const std::int32_t>(msg.buttons));
  RawVelocityCommandSample sample{};
  sample.twist = r.twist;
  sample.source.input = 1;
  sample.source.deadman = r.deadman;
  sample.source.joy_rejects = prev.source.joy_rejects + (r.rejected ? 1U : 0U);
  sample.recv_steady_ns = SteadyNowNs();
  sample.sequence = prev.sequence + 1;
  raw_command_.Store(sample);
}

RawVelocityCommandSample LocomotionController::RestCommandSample() const noexcept {
  RawVelocityCommandSample rest{};
  if (joy_sub_ != nullptr) {
    rest.source.input = 1;
    rest.source.deadman = 0;
  }
  return rest;
}

// The one writer of the publish thread's copy and of the diagnostic log's (and,
// with to_planner, of the phase manager's command slot): under free_run the
// timer, and StartCommandTimer on the same executor thread before the timer
// exists; under lock-step the RT tick, and StartCommandTimer before the first
// tick of the activation.
void LocomotionController::PushCommand(const mpc::FilteredCommand& cmd,
                                       const LocomotionCommandSource& source,
                                       bool to_planner) noexcept {
  if (to_planner && phase_ != nullptr) {
    phase_->SetCommand(cmd);
  }
  command_out_.Store(LocomotionCommandOut{cmd, source});
  LocomotionCommandLogSample& log = command_log_.StartWrite();
  log.v_x = cmd.v_x;
  log.v_y = cmd.v_y;
  log.omega_z = cmd.omega_z;
  log.base_height = cmd.base_height;
  log.fresh = cmd.valid && cmd.fresh;
  log.gait = phase_ != nullptr ? static_cast<std::int32_t>(phase_->CurrentPhaseId()) : -1;
  log.input = source.input;
  log.deadman = source.deadman;
  command_log_.FinishWrite();
}

// The timer (a mutually exclusive group): one filter step per period, with
// whatever the subscription last stored — also when that is stale or nothing
// (the Bessel stage assumes a fixed period; the filter zeroes a stale input).
void LocomotionController::StepCommand() noexcept {
  // The activation's measured height, handed over by the RT tick (header): the
  // filter starts over at it, on its writer's thread.
  const double height =
      pending_height_.exchange(std::numeric_limits<double>::quiet_NaN(), std::memory_order_acq_rel);
  if (std::isfinite(height)) {
    static_cast<void>(command_filter_.Reset(height));  // clamped into the range by the tick
  }
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
  PushCommand(out, raw.source, /*to_planner=*/true);
}

void LocomotionController::StepCommandOnTick() noexcept {
  const RawVelocityCommandSample raw = raw_command_.Load();
  // The input's age on the tick clock: the tick that first saw this sample.
  if (raw.sequence != cmd_seen_sequence_) {
    cmd_seen_sequence_ = raw.sequence;
    cmd_seen_tick_ns_ = raw.sequence == 0 ? std::numeric_limits<std::int64_t>::min()
                                          : static_cast<std::int64_t>(mpc_t_ns_);
  }
  const mpc::FilteredCommand& out =
      command_filter_.Step(raw.twist, cmd_seen_tick_ns_, static_cast<std::int64_t>(mpc_t_ns_));
  if (!(std::isfinite(out.v_x) && std::isfinite(out.v_y) && std::isfinite(out.omega_z) &&
        std::isfinite(out.base_height))) {
    cmd_nonfinite_outputs_.fetch_add(1, std::memory_order_relaxed);  // RT-3: the 1 Hz log says
    return;
  }
  cmd_latest_ = out;
  PushCommand(out, raw.source, /*to_planner=*/false);  // the planner on the trigger tick
}

void LocomotionController::LatchActivationHeight() noexcept {
  const auto& cfg = command_filter_.config();
  const double z = qp_state_.q(2);
  if (!std::isfinite(z)) {
    return;  // not reachable with a valid state; the configured height stands
  }
  const double height = std::clamp(z, cfg.h_min, cfg.h_max);
  if (cmd_step_ticks_ > 0) {
    // Lock-step: this tick is the filter's writer. The reset cannot fail (a
    // finite height inside the range); the trigger tick hands it to the planner.
    static_cast<void>(command_filter_.Reset(height));
    cmd_latest_ = command_filter_.output();
    PushCommand(cmd_latest_, raw_command_.Load().source, /*to_planner=*/false);
    return;
  }
  pending_height_.store(height, std::memory_order_release);  // the wall timer applies it
}

void LocomotionController::StartCommandTimer() noexcept {
  // Before the first tick of this activation: a height the last one handed
  // over and no timer step took is not this activation's.
  pending_height_.store(std::numeric_limits<double>::quiet_NaN(), std::memory_order_release);
  if (command_timer_ || !node_ ||
      !(VelocityCommandSubscribed() || cfg_.mpc.activation_height_measured)) {
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
  const RawVelocityCommandSample rest = RestCommandSample();
  raw_command_.Store(rest);
  PushCommand(command_filter_.output(), rest.source, /*to_planner=*/true);
  if (cmd_step_ticks_ > 0) {
    // Lock-step: the RT tick steps the filter (StepCommandOnTick); no timer.
    // Its tick-side state starts from the reset (ServiceRequests sees this
    // activation before the first step).
    return;
  }
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
    c.overrun = t->OverrunCount();
  }
  mpc_counters_.Store(c);
}

void LocomotionController::PublishNonRtSnapshot(const rtc::PublishSnapshot& snap) noexcept {
  PublishOwnedTopicsFromSnapshot(snap, owned_topics_);
  const LocomotionCommandOut out = command_out_.Load();
  PublishLocomotionStatus(owned_topics_, status_lock_.Load(), mpc_enabled_, out.cmd, out.source,
                          mpc_counters_.Load());
}

}  // namespace integrated_bringup
