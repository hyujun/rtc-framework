// ── RT control loop, timeout watchdog, log drain ──────────────────────
#include "rtc_controller_manager/rt_controller_node.hpp"
#include <rtc_base/threading/thread_utils.hpp>
#include <rtc_base/tracing/trace_scope.hpp>

#include <poll.h>         // poll (sim-sync wake wait)
#include <sys/eventfd.h>  // eventfd_write / eventfd_read
#include <time.h>         // clock_nanosleep, clock_gettime, CLOCK_MONOTONIC, TIMER_ABSTIME

#include <algorithm>  // std::min
#include <array>
#include <cmath>  // std::abs
#include <cstddef>
#include <cstdio>  // std::snprintf

namespace urtc = rtc;

namespace {

// Bound a runtime channel/group count against the fixed array capacity that
// backs it before the count is used as a copy length.
//
// Both producers of these counts are out-of-tree contracts: DeviceBackend
// implementations fill DeviceStateCache, and controller Compute() fills
// ControllerOutput. A faulty producer reporting a negative count turns into a
// huge length on the size_t cast, and one past the capacity walks off the end
// of the fixed std::array — either way copy_n runs out of bounds on the RT
// thread (same defect class as issue #172). Clamping is branch-only: no
// allocation, no logging, no throw, so it is safe on the tick path.
[[nodiscard]] constexpr std::size_t BoundedCount(int count, int capacity) noexcept {
  if (count <= 0) {
    return 0;
  }
  return static_cast<std::size_t>(count < capacity ? count : capacity);
}

}  // namespace

// ── Hold command for a rejected ControllerOutput ─────────────────────────────
bool RtControllerNode::BuildHoldOutput(const urtc::ControllerState& state,
                                       urtc::CommandType cmd_type) noexcept {
  bool complete = true;
  hold_output_.valid = true;
  // The resolved type is stamped once at the top level and every device is
  // left inheriting it (command_type == nullopt). Per-device overrides exist
  // for mixed-mode controllers, but reconstructing that mix would require
  // trusting the rejected output — a hold is uniform by design.
  hold_output_.command_type = cmd_type;

  const auto nd = BoundedCount(state.num_devices, urtc::ControllerOutput::kMaxDevices);
  hold_output_.num_devices = static_cast<int>(nd);
  for (std::size_t i = 0; i < nd; ++i) {
    const auto& dstate = state.devices[i];
    auto& dout = hold_output_.devices[i];
    const auto nc = BoundedCount(dstate.num_channels, urtc::kMaxDeviceChannels);
    dout.num_channels = static_cast<int>(nc);
    dout.command_type.reset();
    dout.goal_type = urtc::GoalType::kJoint;

    // The hold is only as trustworthy as the state it is built from, and the
    // device read path bounds counts but copies the position doubles verbatim
    // — a backend publishing a non-finite position would otherwise make this
    // function emit the very thing the validator just rejected
    // (kNonFiniteCommand). There is no honest substitute for an unknown joint
    // position: 0.0 would command the joint to its origin. So the device is
    // dropped to a zero-length command (every backend treats that as "no
    // update") and the caller is told the hold is incomplete, which escalates
    // immediately rather than waiting out the consecutive-reject window.
    bool device_finite = true;
    for (std::size_t c = 0; c < nc; ++c) {
      if (!std::isfinite(dstate.positions[c])) {
        device_finite = false;
        break;
      }
    }
    if (!device_finite) {
      dout.num_channels = 0;
      complete = false;
      continue;
    }

    // Only [0, nc) is written each tick — the snapshot fill copies exactly
    // that many entries, so entries past nc are never read and re-zeroing the
    // whole 64-wide array on every rejected tick would be wasted RT budget.
    for (std::size_t c = 0; c < nc; ++c) {
      const double measured = dstate.positions[c];
      // kPosition / kPdFeedforward servo to where the joint already is, which
      // is a true stop. kTorque has no such command: the CM carries no dynamic
      // model, so it cannot synthesise a gravity-compensating torque and 0 N·m
      // is the only value it can honestly emit. That means a torque-mode arm
      // sags under gravity for up to kOutputRejectEstopSeconds before the
      // E-STOP escalation lands — configurations with torque-mode devices
      // should tune that window down.
      dout.commands[c] = (cmd_type == urtc::CommandType::kTorque) ? 0.0 : measured;
      // Position-semantics telemetry stays position-valued in every mode so
      // the GUI/log lanes show where the hold is parked, not a 0 that would
      // read as "commanded to origin".
      dout.goal_positions[c] = measured;
      dout.target_positions[c] = measured;
      dout.trajectory_positions[c] = measured;
      dout.target_velocities[c] = 0.0;
      dout.trajectory_velocities[c] = 0.0;
      dout.feedforward[c] = 0.0;
    }
  }

  // Shared lanes are cleared rather than carried over: a pose left from an
  // earlier tick would be republished as if it were current.
  hold_output_.actual_task_positions.fill(0.0);
  hold_output_.task_goal_positions.fill(0.0);
  hold_output_.trajectory_task_positions.fill(0.0);
  hold_output_.trajectory_task_velocities.fill(0.0);
  hold_output_.arm_tip_pose_valid = false;
  hold_output_.virtual_tcp_pose_valid = false;
  hold_output_.task_link_pose_valid.fill(false);
  return complete;
}

// ── 50 Hz watchdog (E-STOP)
// ───────────────────────────────────────────────────
bool RtControllerNode::DeviceLive(const DeviceTimeoutEntry& entry,
                                  std::chrono::steady_clock::time_point now) const noexcept {
  if (entry.slot < 0 || entry.slot >= kMaxDevices) {
    return false;
  }
  const auto& backend = backends_[static_cast<std::size_t>(entry.slot)];
  if (!backend) {
    // Configured group whose backend failed to come up. Not live, and not
    // skippable: silently running the control law for a device that has no
    // driver at all is the failure mode this gate exists to prevent.
    return false;
  }
  const auto stamp = backend->LastStateStamp();
  if (stamp == std::chrono::steady_clock::time_point{}) {
    return false;  // never reported — see LastStateStamp's contract
  }
  return (now - stamp) <= entry.timeout;
}

int RtControllerNode::FirstUnreadyDevice(std::chrono::steady_clock::time_point now) const noexcept {
  for (std::size_t i = 0; i < device_timeouts_.size(); ++i) {
    if (!DeviceLive(device_timeouts_[i], now)) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void RtControllerNode::CheckTimeouts() {
  if (device_timeouts_.empty()) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  for (auto& dt : device_timeouts_) {
    // A device that has never reported is the startup gate's business, not the
    // watchdog's: before init it is expected, and after init it cannot happen,
    // because the gate refuses to complete until every device has reported.
    // (It used to be skipped here with nothing else covering it — the fail-open
    // in §1.)
    if (dt.slot >= 0 && dt.slot < kMaxDevices && backends_[static_cast<std::size_t>(dt.slot)] &&
        backends_[static_cast<std::size_t>(dt.slot)]->LastStateStamp() ==
            std::chrono::steady_clock::time_point{}) {
      continue;
    }
    if (!DeviceLive(dt, now) && !IsGlobalEstopped()) {
      // Fixed-size buffer — concatenating the group name would have been the
      // only heap allocation left on the RT path (same pattern as the
      // actuator-boundary escalation below). group_name is const for the node
      // lifetime, so reading it here is a plain load, not a copy.
      std::array<char, kEstopReasonBufferSize> reason{};
      static_cast<void>(
          std::snprintf(reason.data(), reason.size(), "%s_timeout", dt.group_name.c_str()));
      TriggerGlobalEstop(reason.data());
      return;
    }
  }
}

// ── RT control loop (period = 1 / control_rate)
// ───────────────────────────────────────────────────────
void RtControllerNode::ControlLoop() {
  // Layer 0 span: the whole RT tick (raw clock_nanosleep thread — invisible to
  // ros2_tracing). All nested Compute spans stack under this. Early returns
  // below (not-ready / init-wait) close the span via RAII, so idle ticks show
  // as empty rt_control_tick slices.
  RTC_TRACE_SCOPE("rt_control_tick");
  // ── Phase 0: tick start + readiness check ──────────────────────────────
  const auto t0 = std::chrono::steady_clock::now();

  // Startup gate. EVERY configured device must have reported a state within
  // its timeout before the control law runs — one device's first packet used
  // to release the gate for all of them, so a device that was silent from the
  // start never blocked Compute()/WriteCommand() and was skipped by the
  // watchdog forever (issue #198 §1). Once the gate has opened it stays open;
  // staleness from then on is the watchdog's job.
  if (!init_complete_) {
    const int unready = FirstUnreadyDevice(t0);
    if (unready >= 0) {
      if (init_timeout_ticks_ > 0 && ++init_wait_ticks_ > init_timeout_ticks_) {
        // Terminal one-shot: names the device that held the gate shut, which
        // is the whole point of tracking readiness per device rather than
        // globally. Followed immediately by shutdown, so the log call is not
        // on a recurring tick path.
        RCLCPP_FATAL(get_logger(), "Initialization timeout (%.1f s): device '%s' has not reported",
                     static_cast<double>(init_timeout_ticks_) / control_rate_,
                     device_timeouts_[static_cast<std::size_t>(unready)].group_name.c_str());
        std::array<char, kEstopReasonBufferSize> reason{};
        static_cast<void>(
            std::snprintf(reason.data(), reason.size(), "%s_init_timeout",
                          device_timeouts_[static_cast<std::size_t>(unready)].group_name.c_str()));
        TriggerGlobalEstop(reason.data());
        rclcpp::shutdown();
      }
      return;
    }
    init_complete_ = true;
  }

  // Global E-Stop: controller Compute() handles safe-position internally.
  // We still run the full loop to keep logging and timing active.

  // ── Phase 1: non-blocking state acquisition ────────────────────────────
  // Single atomic load — ensures the same controller index is used for
  // topic config lookup (Phase 1), Compute() dispatch (Phase 2), and
  // publish snapshot (Phase 3) within the same tick.
  const int active_idx = active_controller_idx_.load(std::memory_order_acquire);
  const auto& active_tc = controller_topic_configs_[static_cast<std::size_t>(active_idx)];

  const auto& slot_mapping = controller_slot_mappings_[static_cast<std::size_t>(active_idx)];

  urtc::ControllerState state{};
  {
    RTC_TRACE_SCOPE("CM::ReadDeviceState");
    std::size_t di = 0;
    for ([[maybe_unused]] const auto& [gname, ggroup] : active_tc.groups) {
      const auto slot = static_cast<std::size_t>(slot_mapping.slots[di]);
      const auto cap = slot_mapping.capabilities[di];
      auto& dev = state.devices[di];
      urtc::DeviceStateCache cache{};
      if (backends_[slot]) {
        (void)backends_[slot]->ReadState(cache);
        if (backends_[slot]->HasMotorState())
          backends_[slot]->ReadMotorState(cache);
        if (backends_[slot]->HasSensorState())
          backends_[slot]->ReadSensorState(cache);
      }
      // The bounded count is written back into DeviceState, not just used as
      // the copy length: controllers read state.devices[i].num_channels and
      // would otherwise index past the array we just filled.
      const auto nc = BoundedCount(cache.num_channels, urtc::kMaxDeviceChannels);
      dev.num_channels = static_cast<int>(nc);
      std::copy_n(cache.positions.data(), nc, dev.positions.data());
      std::copy_n(cache.velocities.data(), nc, dev.velocities.data());
      std::copy_n(cache.efforts.data(), nc, dev.efforts.data());
      if (urtc::HasCapability(cap, urtc::DeviceCapability::kMotorState) &&
          cache.num_motor_channels > 0) {
        const auto nmc = BoundedCount(cache.num_motor_channels, urtc::kMaxDeviceChannels);
        dev.num_motor_channels = static_cast<int>(nmc);
        std::copy_n(cache.motor_positions.data(), nmc, dev.motor_positions.data());
        std::copy_n(cache.motor_velocities.data(), nmc, dev.motor_velocities.data());
        std::copy_n(cache.motor_efforts.data(), nmc, dev.motor_efforts.data());
      }
      if (urtc::HasCapability(cap, urtc::DeviceCapability::kSensorData) &&
          cache.num_sensor_channels > 0) {
        const auto nsc = BoundedCount(cache.num_sensor_channels, urtc::kMaxSensorChannels);
        dev.num_sensor_channels = static_cast<int>(nsc);
        std::copy_n(cache.sensor_data.data(), nsc, dev.sensor_data.data());
        std::copy_n(cache.sensor_data_raw.data(), nsc, dev.sensor_data_raw.data());
      }
      if (urtc::HasCapability(cap, urtc::DeviceCapability::kInference) &&
          cache.num_inference_groups > 0 && slot < slot_to_sensor_layout_.size() &&
          slot_to_sensor_layout_[slot].has_value()) {
        const int infer_per_group = slot_to_sensor_layout_[slot]->inference_values_per_group;
        // Both factors are bounded before the multiply: the product is what
        // indexes inference_data, and a YAML-supplied per-group width times a
        // backend-supplied group count can otherwise overflow int outright.
        const auto ngrp = BoundedCount(cache.num_inference_groups, urtc::kMaxSensorGroups);
        const auto per_group = BoundedCount(infer_per_group, urtc::kMaxInferenceValues);
        const auto nif =
            std::min(ngrp * per_group, static_cast<std::size_t>(urtc::kMaxInferenceValues));
        dev.num_inference_groups = static_cast<int>(ngrp);
        std::copy_n(cache.inference_data.data(), nif, dev.inference_data.data());
        std::copy_n(cache.inference_enable.data(), ngrp, dev.inference_enable.data());
      }
      dev.valid = cache.valid;
      ++di;
    }
    state.num_devices = static_cast<int>(di);

    // Per-controller targets are owned by the active controller (SeqLock +
    // SPSC marshal). CM no longer mirrors them — Compute() reads its own
    // SeqLock snapshot internally.
    state.dt = 1.0 / control_rate_;
    state.iteration = loop_count_;

    // Session origin (captured once on the very first tick — independent
    // of enable_logging_, never reset on controller switch). Controllers
    // read state.t_relative_s for any timestamp embedded in their own
    // logs/telemetry instead of calling chrono::*::now().
    if (loop_count_ == 0) {
      log_start_time_ = t0;
    }
    state.t_relative_s = std::chrono::duration<double>(t0 - log_start_time_).count();
  }

  rt_loop_.StampStateAcquired();

  // ── Phase 2: compute control law ───────────────────────────────────────
  // Measure Compute() wall-clock time via ControllerTimingProfiler.
  auto& active_controller = *controllers_[static_cast<std::size_t>(active_idx)];
  const urtc::ControllerOutput raw_output =
      timing_profiler_.MeasuredCompute(active_controller, state);

  rt_loop_.StampComputeDone();

  // ── Phase 2b: actuator-boundary validation (issue #196 Phase 4) ────────
  //
  // Last gate before the output reaches DeviceBackend::WriteCommand. A
  // rejected tick is replaced wholesale by a hold command — see
  // BuildHoldOutput for why holding beats dropping the write, and why the
  // hold is rebuilt from `state` rather than salvaged from the output.
  //
  // Both consumers below (the publish snapshot and the WriteCommand loop)
  // read the same `output` reference, so this single substitution covers the
  // actuator lane and the telemetry lane together.
  const urtc::ControllerOutputValidation validation =
      urtc::ValidateControllerOutput(raw_output, state);
  if (!validation.Ok()) {
    const bool hold_complete = BuildHoldOutput(state, active_controller.GetCommandType());
    rejected_output_count_.fetch_add(1, std::memory_order_relaxed);
    const uint64_t consecutive =
        consecutive_rejected_outputs_.fetch_add(1, std::memory_order_relaxed) + 1;
    // A hold masks a bad tick; it cannot fix a controller that keeps
    // producing them. Escalate on the same principle as the overrun watchdog:
    // sustained failure of the control law is a safety event, not a glitch.
    // An incomplete hold escalates on the very first tick instead of riding
    // out the window. The window is only defensible because the hold makes
    // the interim safe, and that premise is exactly what fails when no hold
    // can be built from the state.
    if ((consecutive >= invalid_output_estop_ticks_ || !hold_complete) && !IsGlobalEstopped()) {
      // Fixed-size buffer, scalar-only format — no allocation on the RT path
      // (same pattern TriggerGlobalEstop uses for the reason string).
      std::array<char, kEstopReasonBufferSize> reason{};
      static_cast<void>(std::snprintf(
          reason.data(), reason.size(), "%s_%s",
          hold_complete ? "invalid_controller_output" : "unholdable_controller_output",
          urtc::OutputRejectReasonToString(validation.reason)));
      TriggerGlobalEstop(reason.data());
    }
  } else {
    consecutive_rejected_outputs_.store(0, std::memory_order_relaxed);
  }
  // Validation keeps running after an E-STOP: the controller's ComputeEstop
  // path can produce a bad output too, and fail-closed must hold in the
  // terminal state as well.
  //
  // ── Phase 2c: E-STOP command substitution (issue #198 Phase 3) ─────────
  //
  // Composed *after* Phase 2b, never in place of it — the block above owns
  // the "terminal state is still validated" contract and this must not
  // become a path around it. Once the latch is set, nothing the controller
  // computed reaches DeviceBackend::WriteCommand: actuator safety cannot
  // depend on each controller implementing its E-STOP hook correctly, and
  // in-tree it demonstrably does not (OperationalSpaceController's E-STOP
  // path emits a position-scale value on a kTorque command by its own
  // admission; PController never overrides SetHandEstop at all).
  //
  // The substitution is the same BuildHoldOutput the invalid-output path
  // uses, so when both fire the command is identical — but the two keep
  // separate counters, because "which guard stopped the write" is exactly
  // what a test of either guard has to be able to see.
  //
  // What this deliberately drops: every in-tree ComputeEstop slews toward a
  // configured `safe_position_` rather than holding. That retreat is a
  // controller-level policy expressed in a controller-level unit, and the
  // manager has no model with which to check it. Restoring it belongs in
  // the backend safe-output contract (Phase 4), where position / torque /
  // hand each get their own semantics — not in a manager that would have to
  // trust the output it just decided not to trust.
  const bool estopped = IsGlobalEstopped();
  if (estopped) {
    estop_substituted_outputs_.fetch_add(1, std::memory_order_relaxed);
    if (validation.Ok()) {
      // Phase 2b already rebuilt hold_output_ from this tick's state when the
      // validation failed; rebuilding it would be redundant, not different.
      static_cast<void>(BuildHoldOutput(state, active_controller.GetCommandType()));
    }
  }
  const urtc::ControllerOutput& output = (estopped || !validation.Ok()) ? hold_output_ : raw_output;

  // ── Phase 3: push publish snapshot to SPSC buffer (lock-free, O(1)) ────
  // All ROS2 publish() calls are offloaded to the non-RT publish thread.
  {
    urtc::PublishSnapshot snap{};
    {
      RTC_TRACE_SCOPE("CM::FillPublishSnapshot");
      snap.actual_task_positions = output.actual_task_positions;
      // Topic-boundary header stamp is ROS wall clock (CLOCK_REALTIME) so
      // published headers (joint_command / grasp_state / transforms) share the
      // ecosystem time axis. VDSO-backed, no syscall/lock — RT-safe like the
      // steady_clock read used for internal timing (t0). Watchdog/staleness stay
      // on monotonic steady_clock elsewhere.
      snap.stamp_ns = std::chrono::system_clock::now().time_since_epoch().count();
      snap.active_controller_idx = active_idx;

      // Per-group commands → group_commands slots
      std::size_t gi = 0;
      for (const auto& [gname, ggroup] : active_tc.groups) {
        static_cast<void>(gname);
        if (gi >= static_cast<std::size_t>(urtc::PublishSnapshot::kMaxGroups))
          break;
        auto& gc = snap.group_commands[gi];
        const auto& dout = output.devices[gi];
        const auto& dstate = state.devices[gi];
        // `output` has already passed ValidateControllerOutput above (or is
        // the hold that replaced it), so these counts are in contract. The
        // bound stays as defence in depth: it is what makes the copy length
        // safe by construction rather than by an argument about an upstream
        // check, and it is the only guard on the day someone adds a second
        // path into this block. dstate was already bounded on the read path;
        // re-bounding keeps the invariant local.
        const auto onc = BoundedCount(dout.num_channels, urtc::kMaxDeviceChannels);
        const auto snc = BoundedCount(dstate.num_channels, urtc::kMaxDeviceChannels);
        gc.num_channels = static_cast<int>(onc);
        gc.actual_num_channels = static_cast<int>(snc);
        gc.stamp_ns = snap.stamp_ns;  // wall-clock ns for JointCommand header
        std::copy_n(dout.commands.data(), onc, gc.commands.data());
        std::copy_n(dout.goal_positions.data(), onc, gc.goal_positions.data());
        std::copy_n(dout.target_positions.data(), onc, gc.target_positions.data());
        std::copy_n(dout.target_velocities.data(), onc, gc.target_velocities.data());
        std::copy_n(dout.trajectory_positions.data(), onc, gc.trajectory_positions.data());
        std::copy_n(dout.trajectory_velocities.data(), onc, gc.trajectory_velocities.data());
        std::copy_n(dout.feedforward.data(), onc, gc.feedforward.data());
        // Resolve per-device command type: nullopt inherits the global default,
        // so single-mode controllers keep their existing behaviour while mixed
        // controllers (WBC arm=kPosition, hand=kPdFeedforward) override per group.
        gc.command_type = dout.command_type.value_or(output.command_type);
        gc.goal_type = dout.goal_type;
        std::copy_n(dstate.positions.data(), snc, gc.actual_positions.data());
        std::copy_n(dstate.velocities.data(), snc, gc.actual_velocities.data());
        std::copy_n(dstate.efforts.data(), snc, gc.efforts.data());
        if (dstate.num_motor_channels > 0) {
          const auto nmc = BoundedCount(dstate.num_motor_channels, urtc::kMaxDeviceChannels);
          gc.num_motor_channels = static_cast<int>(nmc);
          std::copy_n(dstate.motor_positions.data(), nmc, gc.motor_positions.data());
          std::copy_n(dstate.motor_velocities.data(), nmc, gc.motor_velocities.data());
          std::copy_n(dstate.motor_efforts.data(), nmc, gc.motor_efforts.data());
        }
        if (dstate.num_sensor_channels > 0) {
          const auto nsc = BoundedCount(dstate.num_sensor_channels, urtc::kMaxSensorChannels);
          gc.num_sensor_channels = static_cast<int>(nsc);
          std::copy_n(dstate.sensor_data.data(), nsc, gc.sensor_data.data());
          std::copy_n(dstate.sensor_data_raw.data(), nsc, gc.sensor_data_raw.data());
        }
        // Inference output for DeviceSensorLog. Layout (values per inference
        // group) is configured via YAML per device — CM stays sensor-agnostic.
        if (dstate.num_inference_groups > 0) {
          const auto group_slot = static_cast<std::size_t>(slot_mapping.slots[gi]);
          if (group_slot < slot_to_sensor_layout_.size() &&
              slot_to_sensor_layout_[group_slot].has_value()) {
            const int infer_per_group =
                slot_to_sensor_layout_[group_slot]->inference_values_per_group;
            gc.inference_valid = true;
            // The copy loop below is already bounded by inference_output.size();
            // bounding the count itself keeps the value published downstream
            // (DeviceSensorLog) consistent with what was actually copied.
            const auto niv =
                std::min(BoundedCount(dstate.num_inference_groups, urtc::kMaxSensorGroups) *
                             BoundedCount(infer_per_group, urtc::kMaxInferenceValues),
                         static_cast<std::size_t>(urtc::kMaxInferenceValues));
            gc.num_inference_values = static_cast<int>(niv);
            for (std::size_t i = 0; i < niv && i < gc.inference_output.size(); ++i) {
              gc.inference_output[i] = dstate.inference_data[i];
            }
          }
        }
        // Controller-owned non-RT outputs (grasp / wbc / tof) are no longer
        // ferried through PublishSnapshot — each controller owns a per-output
        // SeqLock<T> and publishes via its own LifecyclePublisher. CM mirrors
        // only SE3 poses (kRobotTransforms) and per-group device data here.
        //
        // SE3 poses for kRobotTransforms are populated regardless of which
        // group hosts the TF publisher (D-10): owned_topics walks tf_slots[]
        // and reads from group_commands[slot.group_idx], so we mirror the
        // output's pose fields into every group slot. Controllers that don't
        // broadcast TF leave *_valid = false → publish path is a no-op.
        gc.arm_tip_pose = output.arm_tip_pose;
        gc.arm_tip_pose_valid = output.arm_tip_pose_valid;
        gc.virtual_tcp_pose = output.virtual_tcp_pose;
        gc.virtual_tcp_pose_valid = output.virtual_tcp_pose_valid;
        gc.task_link_poses = output.task_link_poses;
        gc.task_link_pose_valid = output.task_link_pose_valid;
        ++gi;
      }
      snap.num_groups = static_cast<int>(gi);
    }

    // Layout v4.1: actuator command publish is performed inline on the
    // rt_control thread (Core 1 FIFO 90) — DeviceBackend.WriteCommand is
    // RT-safe by contract (see device_backend.hpp). Removing the SPSC
    // hand-off to rt_outbound eliminates one rt_callback-core thread +
    // eventfd and closes the cross-core path between RT loop and actuator
    // publish.
    //
    // Per-group device command publish is delegated to the backend bound to
    // each group's slot. Controller-owned non-RT topics still ride the SPSC
    // lane drained by NrtPublishLoopEntry on the nrt_callback core.
    // Reuses active_tc / slot_mapping resolved at the top of ControlLoop().
    {
      RTC_TRACE_SCOPE("CM::WriteCommand");
      std::size_t out_idx = 0;
      for ([[maybe_unused]] const auto& [group_name, group] : active_tc.groups) {
        if (out_idx >= static_cast<std::size_t>(urtc::PublishSnapshot::kMaxGroups))
          break;
        const int slot = slot_mapping.slots[out_idx];
        if (slot >= 0 && slot < kMaxDevices && backends_[static_cast<std::size_t>(slot)]) {
          backends_[static_cast<std::size_t>(slot)]->WriteCommand(
              snap.group_commands[out_idx], snap.group_commands[out_idx].command_type);
        }
        ++out_idx;
      }
    }
    static_cast<void>(nrt_publish_buffer_.Push(snap));
    if (nrt_publish_eventfd_ >= 0) {
      static_cast<void>(eventfd_write(nrt_publish_eventfd_, 1));
    }
  }

  // Per-tick timing payload (t_state/t_compute/t_publish/t_total/jitter) is
  // captured by the base PeriodicRtThread automatically via the StampStateAcquired/
  // StampComputeDone breakpoints earlier in this function and the cm_timing_producer_
  // wired through SetTimingProducer<> in StartRtLoop. Controller-owned data CSVs
  // are drained by each controller's own ControllerLogSet timer (Phase C).

  ++loop_count_;
  // Signal the log thread to print timing summary every 1 000 iterations.
  static constexpr std::size_t kTimingSummaryInterval = 1000;
  if (loop_count_ % kTimingSummaryInterval == 0) {
    print_timing_summary_.store(true, std::memory_order_relaxed);
  }
}

// File I/O and diagnostic logging stay exclusively in the log thread (Core 4).
void RtControllerNode::DrainLog() {
  // L2 under the log-timer executor's ros2:callback_* (L1) — the CSV drain +
  // deferred E-STOP log + timing-summary print body on the nrt_logging lane.
  RTC_TRACE_SCOPE("CM::DrainLog");
  // Drain per-tick CM timing samples → CSV. Producer is the RT thread; this
  // drain runs at the log-thread cadence (10 ms timer ⇒ ≤5 samples/drain
  // at the default 500 Hz; scales as control_rate / 100). Controller-owned
  // data CSVs are drained by each controller's
  // own ControllerLogSet timer (Phase C) — not from here.
  cm_timing_producer_.Drain(
      [this](const urtc::RtTickTimingSample& s) { cm_timing_logger_.Log(s); });

  // Drain the deferred /system/estop_status publish (issue #198 Phase 3).
  // The latch is what stops the actuators; this topic reports it, so it rides
  // the aux lane rather than a plain publish on the RT thread. Publishing the
  // latch's value at drain time (not a value captured at trigger time) is
  // what makes a trigger/clear pair landing between two drains converge on
  // the final state instead of a stale one.
  if (estop_status_pending_.exchange(false, std::memory_order_acquire)) {
    PublishEstopStatus(global_estop_.load(std::memory_order_acquire));
  }

  // Drain deferred E-STOP log messages (set by TriggerGlobalEstop /
  // ClearGlobalEstop).
  if (estop_log_pending_.exchange(false, std::memory_order_acquire)) {
    if (global_estop_.load(std::memory_order_relaxed)) {
      RCLCPP_ERROR(get_logger(), "GLOBAL E-STOP triggered: %s", estop_reason_.data());
    } else {
      RCLCPP_INFO(get_logger(), "GLOBAL E-STOP cleared (was: %s)", estop_reason_.data());
      estop_reason_.fill('\0');
    }
  }

  // Print timing summary when signalled by the RT thread. Each print is
  // followed by a Reset() so the reported mean/max reflect the most recent
  // kTimingSummaryInterval ticks (≈2 s at the default 500 Hz; scales
  // inversely with control_rate), not the whole session — old start-up
  // spikes would otherwise permanently skew the window stats. Detailed
  // per-tick percentiles / over-budget counts live in cm_timing_log.csv.
  // The rt_controller_node owns cumulative counters (overruns, skips,
  // pub_drops, timing_drops) separately, so they are unaffected by the
  // reset.
  if (print_timing_summary_.exchange(false, std::memory_order_relaxed)) {
    int idx = active_controller_idx_.load(std::memory_order_acquire);
    const auto overruns = rt_loop_.OverrunCount();
    const auto skips = rt_loop_.SkipCount();
    const auto nrt_pub_drops = nrt_publish_buffer_.drop_count();
    const auto timing_drops = cm_timing_producer_.DropCount();

    // Window elapsed semantics differ by mode:
    //   sim   — controller is in lock-step with the simulator step, so the
    //           controller-perceived window length is the nominal
    //           `samples × dt`, regardless of the wall-clock RTF
    //   robot — RT loop runs against wall-clock deadlines (clock_nanosleep
    //           TIMER_ABSTIME), so wall delta between consecutive prints is
    //           the meaningful "real time" the CM observed
    // First print since (re)activation falls through to the nominal form so
    // the very first line still reads as "≈2.0s" instead of the time elapsed
    // since steady_clock epoch.
    constexpr double kUsToS = 1e-6;
    const auto window_count = timing_profiler_.GetStats().count;
    const auto wall_now = std::chrono::steady_clock::now();
    double elapsed_s = static_cast<double>(window_count) * budget_us_ * kUsToS;
    if (!use_sim_time_sync_ && last_summary_wall_.time_since_epoch().count() != 0) {
      elapsed_s = std::chrono::duration<double>(wall_now - last_summary_wall_).count();
    }
    last_summary_wall_ = wall_now;

    RCLCPP_INFO(
        get_logger(), "%s  overruns=%lu  skips=%lu  nrt_pub_drops=%lu  timing_drops=%lu",
        timing_profiler_
            .Summary(std::string(controllers_[static_cast<std::size_t>(idx)]->Name()), elapsed_s)
            .c_str(),
        static_cast<unsigned long>(overruns), static_cast<unsigned long>(skips),
        static_cast<unsigned long>(nrt_pub_drops), static_cast<unsigned long>(timing_drops));
    timing_profiler_.Reset();
  }
}

// ── RT loop (rtc::PeriodicRtThread) ─────────────────────────────────────────
//
// The base provides the deterministic clock_nanosleep TIMER_ABSTIME schedule
// + overrun detection + skip recovery + Pause/Resume scaffolding.
// ControlLoopThread overrides the wakeup primitive (CV in simulation mode),
// the overrun reaction (E-STOP at kMaxConsecutiveOverruns), the abort path
// (sim-sync timeout shutdown), and the stop wake (notify the CV). The tick
// body is forwarded into RtControllerNode::ControlLoop() + the 50 Hz
// watchdog; everything else is shared with the MPC solve thread.

void RtControllerNode::ControlLoopThread::OnTick() noexcept {
  owner_->ControlLoop();
  // kWatchdogCheckHz watchdog — the divisor is control_rate / kWatchdogCheckHz,
  // resolved at parameter load so the check cadence is the same wall-clock rate
  // at every supported control_rate (see watchdog_check_divisor_).
  static thread_local std::uint32_t timeout_tick = 0;
  if (owner_->enable_estop_ && ++timeout_tick % owner_->watchdog_check_divisor_ == 0) {
    RTC_TRACE_SCOPE("CM::CheckTimeouts");
    owner_->CheckTimeouts();
  }
}

rtc::PeriodicRtThread::WaitResult RtControllerNode::ControlLoopThread::WaitForNextTick() noexcept {
  if (!owner_->use_sim_time_sync_) {
    // Real-robot mode: defer to the base's clock_nanosleep schedule +
    // overrun bookkeeping.
    return rtc::PeriodicRtThread::WaitForNextTick();
  }

  // Simulation mode: block on /joint_states arrival. The base's deadline
  // overrun detection is intentionally bypassed here (sim cadence is
  // dictated by the simulator, not a fixed period).
  //
  // poll() on the wake eventfd rather than a condition_variable: the producer
  // is the device state callback on the rt_callback executor (FIFO 70) and
  // the consumer is this rt_control thread (FIFO 90). A shared mutex between
  // two SCHED_FIFO threads is exactly the inversion RT-4 forbids, and the
  // eventfd counter makes the wake edge impossible to miss without one
  // (issue #198 §3).
  if (!rclcpp::ok()) {
    return WaitResult::kAbort;
  }
  if (owner_->sim_wake_eventfd_ < 0) {
    return WaitResult::kAbort;
  }

  const int timeout_ms = static_cast<int>(owner_->sim_sync_timeout_sec_ * 1000.0);

  struct pollfd pfd {};

  pfd.fd = owner_->sim_wake_eventfd_;
  pfd.events = POLLIN;
  const int rc = poll(&pfd, 1, timeout_ms);
  if (rc <= 0) {
    // 0 = the simulator went quiet for the whole timeout; <0 = poll error
    // (EINTR included — shutdown takes the same abort path as before).
    return WaitResult::kAbort;
  }

  // Non-semaphore read drains the counter to zero, so a burst that arrived
  // while the previous tick was computing collapses into this single wake
  // instead of queueing up spare ticks.
  eventfd_t val{};
  static_cast<void>(eventfd_read(owner_->sim_wake_eventfd_, &val));

  if (!rclcpp::ok()) {
    return WaitResult::kAbort;
  }
  return WaitResult::kProceed;
}

void RtControllerNode::ControlLoopThread::OnOverrun(std::uint64_t consecutive) noexcept {
  if (consecutive >= RtControllerNode::kMaxConsecutiveOverruns) {
    owner_->TriggerGlobalEstop("consecutive_overrun");
  }
}

void RtControllerNode::ControlLoopThread::OnLoopAborted() noexcept {
  if (!owner_->use_sim_time_sync_) {
    return;
  }
  RCLCPP_FATAL(owner_->get_logger(),
               "Simulation sync timeout (%.1f s): no /joint_states received — "
               "simulator may have crashed. Shutting down.",
               owner_->sim_sync_timeout_sec_);
  owner_->TriggerGlobalEstop("sim_sync_timeout");
  rclcpp::shutdown();
}

void RtControllerNode::ControlLoopThread::OnRequestStop() noexcept {
  // Sim mode wait blocks on sim_wake_eventfd_; nudge it so RequestStop / Join
  // observe the stop_token without waiting out sim_sync_timeout_sec_.
  if (owner_->sim_wake_eventfd_ >= 0) {
    static_cast<void>(eventfd_write(owner_->sim_wake_eventfd_, 1));
  }
}

bool RtControllerNode::ControlLoopThread::JitterMeaningful() const noexcept {
  // Real-robot mode uses the deadline-driven default wait → |actual_period
  // − budget| is true RT jitter. Sim mode blocks on sim_wake_eventfd_ until the
  // simulator publishes /joint_states, so the cadence is dictated by
  // MuJoCo step completion (max_rtf, viewer load, OS schedule) rather than
  // a fixed period — comparing against a 2 ms budget would just report
  // sim-cadence noise as jitter. The base then leaves jitter_us at 0.0
  // for sim CSVs while the t_state/compute/publish/total_us measurements
  // (which run identically in both modes) stay live.
  return !owner_->use_sim_time_sync_;
}

void RtControllerNode::StartRtLoop(const urtc::ThreadConfig& rt_cfg) {
  rt_loop_.SetTimingProducer<urtc::kCmTimingBufferCapacity>(&cm_timing_producer_);
  if (use_sim_time_sync_) {
    RCLCPP_INFO(get_logger(), "RT loop: simulation sync mode (CV wakeup, timeout=%.1f s)",
                sim_sync_timeout_sec_);
  }
  urtc::PeriodicRtThreadConfig pcfg{};
  pcfg.thread_config = rt_cfg;
  pcfg.frequency_hz = control_rate_;
  rt_loop_.Start(pcfg);
}

void RtControllerNode::StopRtLoop() {
  rt_loop_.Join();
}
