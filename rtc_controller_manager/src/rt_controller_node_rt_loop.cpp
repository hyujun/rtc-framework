// ── 500 Hz RT control loop, timeout watchdog, log drain ──────────────────────
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <rtc_base/threading/thread_utils.hpp>

#include <sys/eventfd.h> // eventfd_write
#include <time.h> // clock_nanosleep, clock_gettime, CLOCK_MONOTONIC, TIMER_ABSTIME

#include <cmath> // std::abs

namespace urtc = rtc;

// ── 50 Hz watchdog (E-STOP)
// ───────────────────────────────────────────────────
void RtControllerNode::CheckTimeouts() {
  if (device_timeouts_.empty()) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  for (auto &dt : device_timeouts_) {
    if (!dt.received.load(std::memory_order_relaxed))
      continue;
    if ((now - dt.last_update) > dt.timeout && !IsGlobalEstopped()) {
      TriggerGlobalEstop(dt.group_name + "_timeout");
      return;
    }
  }
}

bool RtControllerNode::AllTimeoutDevicesReceived() const noexcept {
  for (const auto &dt : device_timeouts_) {
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
    if (!init_complete_ && init_timeout_ticks_ > 0 &&
        ++init_wait_ticks_ > init_timeout_ticks_) {
      RCLCPP_FATAL(get_logger(),
                   "Initialization timeout (%.1f s): robot=%d, target=%d",
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
      const auto &active_tc = controller_topic_configs_[uidx];

      const auto &hold_slots = controller_slot_mappings_[uidx];
      urtc::ControllerState hold_state{};
      {
        std::size_t di = 0;
        for ([[maybe_unused]] const auto &[gname, ggroup] : active_tc.groups) {
          const auto slot = static_cast<std::size_t>(hold_slots.slots[di]);
          auto &dev = hold_state.devices[di];
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
      for (std::size_t d = 0;
           d < static_cast<std::size_t>(hold_state.num_devices); ++d) {
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
      RCLCPP_INFO(get_logger(),
                  "Auto-hold: initialized target from current position (%s)",
                  controllers_[uidx]->Name().data());
    } else {
      if (!init_complete_ && init_timeout_ticks_ > 0 &&
          ++init_wait_ticks_ > init_timeout_ticks_) {
        RCLCPP_FATAL(get_logger(),
                     "Initialization timeout (%.1f s): robot=%d, target=%d",
                     static_cast<double>(init_timeout_ticks_) / control_rate_,
                     1,
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
  const auto &active_tc =
      controller_topic_configs_[static_cast<std::size_t>(active_idx)];

  const auto &slot_mapping =
      controller_slot_mappings_[static_cast<std::size_t>(active_idx)];

  urtc::ControllerState state{};
  std::size_t di = 0;
  for ([[maybe_unused]] const auto &[gname, ggroup] : active_tc.groups) {
    const auto slot = static_cast<std::size_t>(slot_mapping.slots[di]);
    const auto cap = slot_mapping.capabilities[di];
    auto &dev = state.devices[di];
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
      std::copy_n(cache.motor_positions.data(), nmc,
                  dev.motor_positions.data());
      std::copy_n(cache.motor_velocities.data(), nmc,
                  dev.motor_velocities.data());
      std::copy_n(cache.motor_efforts.data(), nmc, dev.motor_efforts.data());
    }
    if (urtc::HasCapability(cap, urtc::DeviceCapability::kSensorData) &&
        cache.num_sensor_channels > 0) {
      const auto nsc = static_cast<std::size_t>(cache.num_sensor_channels);
      dev.num_sensor_channels = cache.num_sensor_channels;
      std::copy_n(cache.sensor_data.data(), nsc, dev.sensor_data.data());
      std::copy_n(cache.sensor_data_raw.data(), nsc,
                  dev.sensor_data_raw.data());
    }
    if (urtc::HasCapability(cap, urtc::DeviceCapability::kInference) &&
        cache.num_inference_fingertips > 0) {
      const auto nif = static_cast<std::size_t>(cache.num_inference_fingertips *
                                                urtc::kFTValuesPerFingertip);
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

  const auto t1 = std::chrono::steady_clock::now(); // end of state acquisition

  // ── Phase 2: compute control law ───────────────────────────────────────
  // Measure Compute() wall-clock time via ControllerTimingProfiler.
  const urtc::ControllerOutput output = timing_profiler_.MeasuredCompute(
      *controllers_[static_cast<std::size_t>(active_idx)], state);

  const auto t2 = std::chrono::steady_clock::now(); // end of compute

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
    for ([[maybe_unused]] const auto &[gname, ggroup] : active_tc.groups) {
      if (gi >= static_cast<std::size_t>(urtc::PublishSnapshot::kMaxGroups))
        break;
      auto &gc = snap.group_commands[gi];
      const auto &dout = output.devices[gi];
      const auto &dstate = state.devices[gi];
      const auto onc = static_cast<std::size_t>(dout.num_channels);
      const auto snc = static_cast<std::size_t>(dstate.num_channels);
      gc.num_channels = dout.num_channels;
      gc.actual_num_channels = dstate.num_channels;
      std::copy_n(dout.commands.data(), onc, gc.commands.data());
      std::copy_n(dout.goal_positions.data(), onc, gc.goal_positions.data());
      std::copy_n(dout.target_positions.data(), onc,
                  gc.target_positions.data());
      std::copy_n(dout.target_velocities.data(), onc,
                  gc.target_velocities.data());
      std::copy_n(dout.trajectory_positions.data(), onc,
                  gc.trajectory_positions.data());
      std::copy_n(dout.trajectory_velocities.data(), onc,
                  gc.trajectory_velocities.data());
      gc.goal_type = dout.goal_type;
      std::copy_n(dstate.positions.data(), snc, gc.actual_positions.data());
      std::copy_n(dstate.velocities.data(), snc, gc.actual_velocities.data());
      std::copy_n(dstate.efforts.data(), snc, gc.efforts.data());
      if (dstate.num_motor_channels > 0) {
        const auto nmc = static_cast<std::size_t>(dstate.num_motor_channels);
        gc.num_motor_channels = dstate.num_motor_channels;
        std::copy_n(dstate.motor_positions.data(), nmc,
                    gc.motor_positions.data());
        std::copy_n(dstate.motor_velocities.data(), nmc,
                    gc.motor_velocities.data());
        std::copy_n(dstate.motor_efforts.data(), nmc, gc.motor_efforts.data());
      }
      if (dstate.num_sensor_channels > 0) {
        const auto nsc = static_cast<std::size_t>(dstate.num_sensor_channels);
        gc.num_sensor_channels = dstate.num_sensor_channels;
        std::copy_n(dstate.sensor_data.data(), nsc, gc.sensor_data.data());
        std::copy_n(dstate.sensor_data_raw.data(), nsc,
                    gc.sensor_data_raw.data());
      }
      // Inference output for DeviceSensorLog
      if (dstate.num_inference_fingertips > 0) {
        gc.inference_valid = true;
        gc.num_inference_values =
            dstate.num_inference_fingertips * urtc::kFTValuesPerFingertip;
        const auto niv = static_cast<std::size_t>(gc.num_inference_values);
        for (std::size_t i = 0; i < niv && i < gc.inference_output.size();
             ++i) {
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

  const auto t3 = std::chrono::steady_clock::now(); // end of publish

  // ── Phase 4: per-phase timing + log push ───────────────────────────────
  const double t_state_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
  const double t_compute_us =
      std::chrono::duration<double, std::micro>(t2 - t1).count();
  const double t_publish_us =
      std::chrono::duration<double, std::micro>(t3 - t2).count();
  const double t_total_us =
      std::chrono::duration<double, std::micro>(t3 - t0).count();

  // Jitter = |actual_period - expected_period|
  double jitter_us = 0.0;
  if (loop_count_ > 0) {
    const double actual_period_us =
        std::chrono::duration<double, std::micro>(t0 - prev_loop_start_)
            .count();
    jitter_us = std::abs(actual_period_us - budget_us_);
  }
  prev_loop_start_ = t0;

  // Compute overrun — ControlLoop() itself exceeded tick budget.
  // (Distinguished from RT loop overrun which includes sleep jitter.)
  if (t_total_us > budget_us_) {
    compute_overrun_count_.fetch_add(1, std::memory_order_relaxed);
  }

  // Push log entry to the SPSC ring buffer — O(1), no syscall.
  // DrainLog() (log thread, Core 4) pops entries and writes the CSV file.
  if (enable_logging_) {
    if (loop_count_ == 0) {
      log_start_time_ = t0;
    }
    const double timestamp =
        std::chrono::duration<double>(t0 - log_start_time_).count();

    urtc::LogEntry entry{};
    entry.timestamp = timestamp;
    entry.t_state_acquire_us = t_state_us;
    entry.t_compute_us = t_compute_us;
    entry.t_publish_us = t_publish_us;
    entry.t_total_us = t_total_us;
    entry.jitter_us = jitter_us;
    entry.actual_task_positions = output.actual_task_positions;
    entry.task_goal_positions = output.task_goal_positions;
    entry.trajectory_task_positions = output.trajectory_task_positions;
    entry.trajectory_task_velocities = output.trajectory_task_velocities;
    entry.command_type = output.command_type;

    // Per-device logging
    for (std::size_t dvi = 0; dvi < static_cast<std::size_t>(state.num_devices);
         ++dvi) {
      auto &dl = entry.devices[dvi];
      const auto &dout = output.devices[dvi];
      const auto &dstate = state.devices[dvi];
      dl.num_channels = dout.num_channels;
      dl.valid = dstate.valid;
      for (std::size_t j = 0; j < static_cast<std::size_t>(dout.num_channels);
           ++j) {
        dl.goal_positions[j] = dout.goal_positions[j];
        dl.actual_positions[j] = dstate.positions[j];
        dl.actual_velocities[j] = dstate.velocities[j];
        dl.efforts[j] = dstate.efforts[j];
        dl.commands[j] = dout.commands[j];
        dl.trajectory_positions[j] = dout.trajectory_positions[j];
        dl.trajectory_velocities[j] = dout.trajectory_velocities[j];
      }
      dl.goal_type = dout.goal_type;
      dl.num_motor_channels = dstate.num_motor_channels;
      for (std::size_t j = 0;
           j < static_cast<std::size_t>(dstate.num_motor_channels); ++j) {
        dl.motor_positions[j] = dstate.motor_positions[j];
        dl.motor_velocities[j] = dstate.motor_velocities[j];
        dl.motor_efforts[j] = dstate.motor_efforts[j];
      }
      dl.num_sensor_channels = dstate.num_sensor_channels;
      for (std::size_t j = 0;
           j < static_cast<std::size_t>(dstate.num_sensor_channels); ++j) {
        dl.sensor_data[j] = static_cast<float>(dstate.sensor_data[j]);
        dl.sensor_data_raw[j] = static_cast<float>(dstate.sensor_data_raw[j]);
      }
    }
    entry.num_devices = state.num_devices;

    // Inference output for sensor log (from hand device, typically device index
    // 1)
    for (std::size_t dvi = 0; dvi < static_cast<std::size_t>(state.num_devices);
         ++dvi) {
      const auto &dstate = state.devices[dvi];
      if (dstate.num_inference_fingertips > 0) {
        entry.inference_valid = true;
        entry.num_inference_values =
            dstate.num_inference_fingertips * urtc::kFTValuesPerFingertip;
        const auto niv = static_cast<std::size_t>(entry.num_inference_values);
        for (std::size_t j = 0; j < niv && j < entry.inference_output.size();
             ++j) {
          entry.inference_output[j] = dstate.inference_data[j];
        }
        break; // only one device has inference
      }
    }

    static_cast<void>(
        log_buffer_.Push(entry)); // silently drops if buffer is full
  }

  ++loop_count_;
  // Signal the log thread to print timing summary every 1 000 iterations.
  static constexpr std::size_t kTimingSummaryInterval = 1000;
  if (loop_count_ % kTimingSummaryInterval == 0) {
    print_timing_summary_.store(true, std::memory_order_relaxed);
  }
}

// File I/O and diagnostic logging stay exclusively in the log thread (Core 4).
void RtControllerNode::DrainLog() {
  if (logger_) {
    logger_->DrainBuffer(log_buffer_);
  }

  // Drain deferred E-STOP log messages (set by TriggerGlobalEstop /
  // ClearGlobalEstop).
  if (estop_log_pending_.exchange(false, std::memory_order_acquire)) {
    if (global_estop_.load(std::memory_order_relaxed)) {
      RCLCPP_ERROR(get_logger(), "GLOBAL E-STOP triggered: %s",
                   estop_reason_.data());
    } else {
      RCLCPP_INFO(get_logger(), "GLOBAL E-STOP cleared (was: %s)",
                  estop_reason_.data());
      estop_reason_.fill('\0');
    }
  }

  // Print timing summary when signalled by the RT thread. Each print is
  // followed by a Reset() so the reported mean/p95/p99 reflect the most
  // recent kTimingSummaryInterval ticks (~2 s at 500 Hz), not the whole
  // session — old start-up spikes would otherwise permanently skew p99.
  // The rt_controller_node owns cumulative counters (overruns, skips,
  // log_drops, pub_drops) separately, so they are unaffected by the reset.
  if (print_timing_summary_.exchange(false, std::memory_order_relaxed)) {
    int idx = active_controller_idx_.load(std::memory_order_acquire);
    const auto overruns = overrun_count_.load(std::memory_order_relaxed);
    const auto compute_overruns =
        compute_overrun_count_.load(std::memory_order_relaxed);
    const auto skips = skip_count_.load(std::memory_order_relaxed);
    const auto log_drops = log_buffer_.drop_count();
    const auto pub_drops = publish_buffer_.drop_count();
    RCLCPP_INFO(get_logger(),
                "%s  overruns=%lu  compute_overruns=%lu  skips=%lu  "
                "log_drops=%lu  pub_drops=%lu",
                timing_profiler_
                    .Summary(std::string(
                        controllers_[static_cast<std::size_t>(idx)]->Name()))
                    .c_str(),
                static_cast<unsigned long>(overruns),
                static_cast<unsigned long>(compute_overruns),
                static_cast<unsigned long>(skips),
                static_cast<unsigned long>(log_drops),
                static_cast<unsigned long>(pub_drops));
    timing_profiler_.Reset();
  }
}

// ── RT loop (clock_nanosleep)
// ──────────────────────────────────────────────────
//
// Replaces create_wall_timer() with a tight POSIX absolute-time sleep loop.
// Eliminates ~50-200 µs of executor/epoll dispatch jitter.  Overrun recovery
// skips missed ticks and realigns to the next period boundary — no burst.
//
// Threading: runs as std::jthread on Core 2, SCHED_FIFO 90.
//            CheckTimeouts() inlined every 10th tick (50 Hz).

void RtControllerNode::RtLoopEntry(const urtc::ThreadConfig &cfg) {
  static_cast<void>(urtc::ApplyThreadConfig(cfg));
  rt_loop_running_.store(true, std::memory_order_release);

  uint32_t timeout_tick = 0;

  if (use_sim_time_sync_) {
    // ═══ Simulation mode: CV-based wakeup on state arrival ═══
    const auto sim_timeout =
        std::chrono::duration<double>(sim_sync_timeout_sec_);

    RCLCPP_INFO(get_logger(),
                "RT loop: simulation sync mode (CV wakeup, timeout=%.1f s)",
                sim_sync_timeout_sec_);

    while (rt_loop_running_.load(std::memory_order_acquire) && rclcpp::ok()) {
      bool woken = false;
      {
        std::unique_lock lock(state_cv_mutex_);
        woken = state_cv_.wait_for(lock, sim_timeout, [this] {
          return state_fresh_.load(std::memory_order_acquire) ||
                 !rt_loop_running_.load(std::memory_order_acquire) ||
                 !rclcpp::ok();
        });
        state_fresh_.store(false, std::memory_order_release);
      }

      if (!rt_loop_running_.load(std::memory_order_acquire)) {
        break;
      }

      if (!woken) {
        RCLCPP_FATAL(
            get_logger(),
            "Simulation sync timeout (%.1f s): no /joint_states received — "
            "simulator may have crashed. Shutting down.",
            sim_sync_timeout_sec_);
        TriggerGlobalEstop("sim_sync_timeout");
        rclcpp::shutdown();
        return;
      }

      ControlLoop();

      static constexpr int kWatchdogCheckDivisor = 10; // 50 Hz at 500 Hz loop
      if (enable_estop_ && ++timeout_tick % kWatchdogCheckDivisor == 0) {
        CheckTimeouts();
      }
    }
  } else {
    // ═══ Real robot mode: deterministic clock_nanosleep 500 Hz ═══
    const int64_t period_ns = static_cast<int64_t>(1.0e9 / control_rate_);
    struct timespec next_wake {};
    clock_gettime(CLOCK_MONOTONIC, &next_wake);

    while (rt_loop_running_.load(std::memory_order_acquire) && rclcpp::ok()) {
      // Advance to next absolute wake time
      next_wake.tv_nsec += period_ns;
      if (next_wake.tv_nsec >= 1'000'000'000L) {
        next_wake.tv_sec += next_wake.tv_nsec / 1'000'000'000L;
        next_wake.tv_nsec %= 1'000'000'000L;
      }

      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wake, nullptr);

      // ── Overrun detection & recovery ────────────────────────────────────
      struct timespec now_ts {};
      clock_gettime(CLOCK_MONOTONIC, &now_ts);
      const int64_t now_ns = now_ts.tv_sec * 1'000'000'000L + now_ts.tv_nsec;
      const int64_t wake_ns =
          next_wake.tv_sec * 1'000'000'000L + next_wake.tv_nsec;
      const int64_t lag_ns = now_ns - wake_ns;

      if (lag_ns > period_ns) {
        const int64_t missed_ticks = lag_ns / period_ns;
        const int64_t advance_ns = missed_ticks * period_ns;
        next_wake.tv_nsec += static_cast<long>(advance_ns);
        if (next_wake.tv_nsec >= 1'000'000'000L) {
          next_wake.tv_sec += next_wake.tv_nsec / 1'000'000'000L;
          next_wake.tv_nsec %= 1'000'000'000L;
        }
        overrun_count_.fetch_add(1, std::memory_order_relaxed);
        skip_count_.fetch_add(static_cast<uint64_t>(missed_ticks),
                              std::memory_order_relaxed);

        const auto consecutive =
            consecutive_overruns_.fetch_add(1, std::memory_order_relaxed) + 1;
        if (consecutive >= kMaxConsecutiveOverruns) {
          TriggerGlobalEstop("consecutive_overrun");
        }
      } else {
        consecutive_overruns_.store(0, std::memory_order_relaxed);
      }

      ControlLoop();

      static constexpr int kWatchdogCheckDivisor = 10; // 50 Hz at 500 Hz loop
      if (enable_estop_ && ++timeout_tick % kWatchdogCheckDivisor == 0) {
        CheckTimeouts();
      }
    }
  }
}

void RtControllerNode::StartRtLoop(const urtc::ThreadConfig &rt_cfg) {
  rt_loop_thread_ = std::jthread([this, rt_cfg]() { RtLoopEntry(rt_cfg); });
}

void RtControllerNode::StopRtLoop() {
  rt_loop_running_.store(false, std::memory_order_release);
  // Wake sim-mode CV wait so the RT thread observes the stop flag immediately
  // instead of sleeping until sim_sync_timeout_sec_. Caller is destructor /
  // on_deactivate / on_shutdown — never the RT tick.
  state_cv_.notify_all();
  if (rt_loop_thread_.joinable()) {
    rt_loop_thread_.join();
  }
}
