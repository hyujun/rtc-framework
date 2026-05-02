// ── 500 Hz RT control loop, timeout watchdog, log drain ──────────────────────
#include "rtc_controller_manager/rt_controller_node.hpp"
#include <rtc_base/threading/thread_utils.hpp>

#include <sys/eventfd.h>  // eventfd_write
#include <time.h>         // clock_nanosleep, clock_gettime, CLOCK_MONOTONIC, TIMER_ABSTIME

#include <cmath>  // std::abs

namespace urtc = rtc;

// ── 50 Hz watchdog (E-STOP)
// ───────────────────────────────────────────────────
void RtControllerNode::CheckTimeouts() {
  if (device_timeouts_.empty()) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  for (auto& dt : device_timeouts_) {
    if (!dt.received.load(std::memory_order_relaxed))
      continue;
    if ((now - dt.last_update) > dt.timeout && !IsGlobalEstopped()) {
      TriggerGlobalEstop(dt.group_name + "_timeout");
      return;
    }
  }
}

bool RtControllerNode::AllTimeoutDevicesReceived() const noexcept {
  for (const auto& dt : device_timeouts_) {
    if (!dt.received.load(std::memory_order_relaxed))
      return false;
  }
  return true;
}

// ── 500 Hz control loop
// ───────────────────────────────────────────────────────
void RtControllerNode::ControlLoop() {
  // ── Phase 0: tick start + readiness check ──────────────────────────────
  const auto t0 = std::chrono::steady_clock::now();

  if (!state_received_.load(std::memory_order_acquire)) {
    if (!init_complete_ && init_timeout_ticks_ > 0 && ++init_wait_ticks_ > init_timeout_ticks_) {
      RCLCPP_FATAL(get_logger(), "Initialization timeout (%.1f s): robot=%d, target=%d",
                   static_cast<double>(init_timeout_ticks_) / control_rate_,
                   state_received_.load(std::memory_order_relaxed) ? 1 : 0,
                   target_received_.load(std::memory_order_relaxed) ? 1 : 0);
      TriggerGlobalEstop("init_timeout");
      rclcpp::shutdown();
    }
    return;
  }

  if (!target_received_.load(std::memory_order_acquire)) {
    if (auto_hold_position_) {
      const int idx = active_controller_idx_.load(std::memory_order_acquire);
      const auto uidx = static_cast<std::size_t>(idx);
      const auto& active_tc = controller_topic_configs_[uidx];

      const auto& hold_slots = controller_slot_mappings_[uidx];
      urtc::ControllerState hold_state{};
      {
        std::size_t di = 0;
        for ([[maybe_unused]] const auto& [gname, ggroup] : active_tc.groups) {
          const auto slot = static_cast<std::size_t>(hold_slots.slots[di]);
          auto& dev = hold_state.devices[di];
          const auto cache = device_states_[slot].Load();
          dev.num_channels = cache.num_channels;
          dev.positions = cache.positions;
          dev.velocities = cache.velocities;
          dev.efforts = cache.efforts;
          dev.valid = cache.valid;
          ++di;
        }
        hold_state.num_devices = static_cast<int>(di);
      }
      hold_state.dt = 1.0 / control_rate_;

      bool all_devices_valid = true;
      for (std::size_t d = 0; d < static_cast<std::size_t>(hold_state.num_devices); ++d) {
        if (!hold_state.devices[d].valid) {
          all_devices_valid = false;
          break;
        }
      }
      if (!all_devices_valid) {
        return;
      }

      controllers_[uidx]->InitializeHoldPosition(hold_state);

      target_received_.store(true, std::memory_order_release);
      RCLCPP_INFO(get_logger(), "Auto-hold: initialized target from current position (%s)",
                  controllers_[uidx]->Name().data());
    } else {
      if (!init_complete_ && init_timeout_ticks_ > 0 && ++init_wait_ticks_ > init_timeout_ticks_) {
        RCLCPP_FATAL(get_logger(), "Initialization timeout (%.1f s): robot=%d, target=%d",
                     static_cast<double>(init_timeout_ticks_) / control_rate_, 1,
                     target_received_.load(std::memory_order_relaxed) ? 1 : 0);
        TriggerGlobalEstop("init_timeout");
        rclcpp::shutdown();
      }
      return;
    }
  }
  init_complete_ = true;

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
  std::size_t di = 0;
  for ([[maybe_unused]] const auto& [gname, ggroup] : active_tc.groups) {
    const auto slot = static_cast<std::size_t>(slot_mapping.slots[di]);
    const auto cap = slot_mapping.capabilities[di];
    auto& dev = state.devices[di];
    const auto cache = device_states_[slot].Load();
    const auto nc = static_cast<std::size_t>(cache.num_channels);
    dev.num_channels = cache.num_channels;
    std::copy_n(cache.positions.data(), nc, dev.positions.data());
    std::copy_n(cache.velocities.data(), nc, dev.velocities.data());
    std::copy_n(cache.efforts.data(), nc, dev.efforts.data());
    if (urtc::HasCapability(cap, urtc::DeviceCapability::kMotorState) &&
        cache.num_motor_channels > 0) {
      const auto nmc = static_cast<std::size_t>(cache.num_motor_channels);
      dev.num_motor_channels = cache.num_motor_channels;
      std::copy_n(cache.motor_positions.data(), nmc, dev.motor_positions.data());
      std::copy_n(cache.motor_velocities.data(), nmc, dev.motor_velocities.data());
      std::copy_n(cache.motor_efforts.data(), nmc, dev.motor_efforts.data());
    }
    if (urtc::HasCapability(cap, urtc::DeviceCapability::kSensorData) &&
        cache.num_sensor_channels > 0) {
      const auto nsc = static_cast<std::size_t>(cache.num_sensor_channels);
      dev.num_sensor_channels = cache.num_sensor_channels;
      std::copy_n(cache.sensor_data.data(), nsc, dev.sensor_data.data());
      std::copy_n(cache.sensor_data_raw.data(), nsc, dev.sensor_data_raw.data());
    }
    if (urtc::HasCapability(cap, urtc::DeviceCapability::kInference) &&
        cache.num_inference_fingertips > 0) {
      const auto nif =
          static_cast<std::size_t>(cache.num_inference_fingertips * urtc::kFTValuesPerFingertip);
      dev.num_inference_fingertips = cache.num_inference_fingertips;
      std::copy_n(cache.inference_data.data(), nif, dev.inference_data.data());
      std::copy_n(cache.inference_enable.data(),
                  static_cast<std::size_t>(cache.num_inference_fingertips),
                  dev.inference_enable.data());
    }
    dev.valid = cache.valid;
    ++di;
  }
  state.num_devices = static_cast<int>(di);

  {
    std::unique_lock lock(target_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      device_target_snapshots_ = device_targets_;
    }
  }
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

  rt_loop_.StampStateAcquired();

  // ── Phase 2: compute control law ───────────────────────────────────────
  // Measure Compute() wall-clock time via ControllerTimingProfiler.
  const urtc::ControllerOutput output =
      timing_profiler_.MeasuredCompute(*controllers_[static_cast<std::size_t>(active_idx)], state);

  rt_loop_.StampComputeDone();

  // ── Phase 3: push publish snapshot to SPSC buffer (lock-free, O(1)) ────
  // All ROS2 publish() calls are offloaded to the non-RT publish thread.
  {
    urtc::PublishSnapshot snap{};
    snap.command_type = output.command_type;
    snap.actual_task_positions = output.actual_task_positions;
    snap.stamp_ns = std::chrono::steady_clock::now().time_since_epoch().count();
    snap.active_controller_idx = active_idx;

    // Per-group commands → group_commands slots
    std::size_t gi = 0;
    for ([[maybe_unused]] const auto& [gname, ggroup] : active_tc.groups) {
      if (gi >= static_cast<std::size_t>(urtc::PublishSnapshot::kMaxGroups))
        break;
      auto& gc = snap.group_commands[gi];
      const auto& dout = output.devices[gi];
      const auto& dstate = state.devices[gi];
      const auto onc = static_cast<std::size_t>(dout.num_channels);
      const auto snc = static_cast<std::size_t>(dstate.num_channels);
      gc.num_channels = dout.num_channels;
      gc.actual_num_channels = dstate.num_channels;
      std::copy_n(dout.commands.data(), onc, gc.commands.data());
      std::copy_n(dout.goal_positions.data(), onc, gc.goal_positions.data());
      std::copy_n(dout.target_positions.data(), onc, gc.target_positions.data());
      std::copy_n(dout.target_velocities.data(), onc, gc.target_velocities.data());
      std::copy_n(dout.trajectory_positions.data(), onc, gc.trajectory_positions.data());
      std::copy_n(dout.trajectory_velocities.data(), onc, gc.trajectory_velocities.data());
      gc.goal_type = dout.goal_type;
      std::copy_n(dstate.positions.data(), snc, gc.actual_positions.data());
      std::copy_n(dstate.velocities.data(), snc, gc.actual_velocities.data());
      std::copy_n(dstate.efforts.data(), snc, gc.efforts.data());
      if (dstate.num_motor_channels > 0) {
        const auto nmc = static_cast<std::size_t>(dstate.num_motor_channels);
        gc.num_motor_channels = dstate.num_motor_channels;
        std::copy_n(dstate.motor_positions.data(), nmc, gc.motor_positions.data());
        std::copy_n(dstate.motor_velocities.data(), nmc, gc.motor_velocities.data());
        std::copy_n(dstate.motor_efforts.data(), nmc, gc.motor_efforts.data());
      }
      if (dstate.num_sensor_channels > 0) {
        const auto nsc = static_cast<std::size_t>(dstate.num_sensor_channels);
        gc.num_sensor_channels = dstate.num_sensor_channels;
        std::copy_n(dstate.sensor_data.data(), nsc, gc.sensor_data.data());
        std::copy_n(dstate.sensor_data_raw.data(), nsc, gc.sensor_data_raw.data());
      }
      // Inference output for DeviceSensorLog
      if (dstate.num_inference_fingertips > 0) {
        gc.inference_valid = true;
        gc.num_inference_values = dstate.num_inference_fingertips * urtc::kFTValuesPerFingertip;
        const auto niv = static_cast<std::size_t>(gc.num_inference_values);
        for (std::size_t i = 0; i < niv && i < gc.inference_output.size(); ++i) {
          gc.inference_output[i] = dstate.inference_data[i];
        }
      }
      // task_goals: copy from controller's task goal target
      snap.task_goals[gi] = output.task_goal_positions;
      // Grasp state from controller output
      gc.grasp_state = output.grasp_state;
      // WBC state from controller output (TSID-based controllers)
      gc.wbc_state = output.wbc_state;
      // ToF snapshot from controller output
      gc.tof_snapshot = output.tof_snapshot;
      ++gi;
    }
    snap.num_groups = static_cast<int>(gi);

    static_cast<void>(publish_buffer_.Push(snap));
    if (publish_eventfd_ >= 0) {
      static_cast<void>(eventfd_write(publish_eventfd_, 1));
    }
  }

  const auto t3 = std::chrono::steady_clock::now();  // end of publish

  // ── Phase 4: compute-overrun counter + log push ────────────────────────
  // Per-tick timing payload (t_state/t_compute/t_publish/t_total/jitter)
  // is captured by the base PeriodicRtThread automatically — the t1/t2
  // breakpoints have already been stamped via rt_loop_.StampStateAcquired/
  // StampComputeDone earlier in this function, and cm_timing_producer_ is
  // wired through SetTimingProducer<> in StartRtLoop. We only retain the
  // *compute-overrun* counter here because it tracks "ControlLoop body >
  // budget" specifically — distinct from the deadline overrun the base
  // detects across the sleep step.
  const double t_total_us = std::chrono::duration<double, std::micro>(t3 - t0).count();
  if (t_total_us > budget_us_) {
    compute_overrun_count_.fetch_add(1, std::memory_order_relaxed);
  }

  // CM no longer fills LogEntry (controller-owned logging path lives on
  // each controller's ControllerLogSet — Phase C). cm_timing_log.csv is
  // still produced via cm_timing_producer_ → cm_timing_logger_ in
  // DrainLog().

  ++loop_count_;
  // Signal the log thread to print timing summary every 1 000 iterations.
  static constexpr std::size_t kTimingSummaryInterval = 1000;
  if (loop_count_ % kTimingSummaryInterval == 0) {
    print_timing_summary_.store(true, std::memory_order_relaxed);
  }
}

// File I/O and diagnostic logging stay exclusively in the log thread (Core 4).
void RtControllerNode::DrainLog() {
  // Drain per-tick CM timing samples → CSV. Producer is the RT thread; this
  // drain runs at the log-thread cadence (10 ms timer ⇒ ≤5 samples/drain
  // at 500 Hz). Controller-owned data CSVs are drained by each controller's
  // own ControllerLogSet timer (Phase C) — not from here.
  cm_timing_producer_.Drain(
      [this](const urtc::RtTickTimingSample& s) { cm_timing_logger_.Log(s); });

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
  // followed by a Reset() so the reported mean/p95/p99 reflect the most
  // recent kTimingSummaryInterval ticks (~2 s at 500 Hz), not the whole
  // session — old start-up spikes would otherwise permanently skew p99.
  // The rt_controller_node owns cumulative counters (overruns, skips,
  // pub_drops, timing_drops) separately, so they are unaffected
  // by the reset.
  if (print_timing_summary_.exchange(false, std::memory_order_relaxed)) {
    int idx = active_controller_idx_.load(std::memory_order_acquire);
    const auto overruns = rt_loop_.OverrunCount();
    const auto compute_overruns = compute_overrun_count_.load(std::memory_order_relaxed);
    const auto skips = rt_loop_.SkipCount();
    const auto pub_drops = publish_buffer_.drop_count();
    const auto timing_drops = cm_timing_producer_.DropCount();
    RCLCPP_INFO(
        get_logger(),
        "%s  overruns=%lu  compute_overruns=%lu  skips=%lu  "
        "pub_drops=%lu  timing_drops=%lu",
        timing_profiler_.Summary(std::string(controllers_[static_cast<std::size_t>(idx)]->Name()))
            .c_str(),
        static_cast<unsigned long>(overruns), static_cast<unsigned long>(compute_overruns),
        static_cast<unsigned long>(skips), static_cast<unsigned long>(pub_drops),
        static_cast<unsigned long>(timing_drops));
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
  // 50 Hz watchdog — every 10th tick at 500 Hz.
  static thread_local std::uint32_t timeout_tick = 0;
  static constexpr int kWatchdogCheckDivisor = 10;
  if (owner_->enable_estop_ && ++timeout_tick % kWatchdogCheckDivisor == 0) {
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
  const auto sim_timeout = std::chrono::duration<double>(owner_->sim_sync_timeout_sec_);
  bool woken = false;
  {
    std::unique_lock<std::mutex> lock(owner_->state_cv_mutex_);
    woken = owner_->state_cv_.wait_for(lock, sim_timeout, [this] {
      return owner_->state_fresh_.load(std::memory_order_acquire) || !rclcpp::ok();
    });
    owner_->state_fresh_.store(false, std::memory_order_release);
  }
  if (!woken) {
    return WaitResult::kAbort;
  }
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
  // Sim mode wait blocks on state_cv_; nudge it so RequestStop / Join
  // observe the stop_token without waiting out sim_sync_timeout_sec_.
  owner_->state_cv_.notify_all();
}

bool RtControllerNode::ControlLoopThread::JitterMeaningful() const noexcept {
  // Real-robot mode uses the deadline-driven default wait → |actual_period
  // − budget| is true RT jitter. Sim mode blocks on state_cv_ until the
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
