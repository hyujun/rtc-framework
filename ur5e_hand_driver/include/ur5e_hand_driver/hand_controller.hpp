#ifndef UR5E_HAND_DRIVER_HAND_CONTROLLER_HPP_
#define UR5E_HAND_DRIVER_HAND_CONTROLLER_HPP_

// High-level hand controller: event-driven UDP driver.
//
// Uses a single UDP socket for both send and receive.
// Driven by ControlLoop events (pipeline mode):
//   Phase 4 of tick N: SendCommandAndRequestStates(cmd)
//     -> Hand thread wakes and executes (individual mode):
//       1. Write position  (0x01) + recv echo
//       2. Read position   (0x11)
//       3. Read velocity   (0x12)
//       4. Read sensor 0-3 (0x14..0x17) x 4
//     -> Hand thread (bulk mode):
//       1. Write position  (0x01) + recv echo
//       2. Read all motors (0x10)
//       3. Read all sensors (0x19)
//     -> State ready for tick N+1
//   Phase 1 of tick N+1: GetLatestState() returns pre-fetched state
//
// RT safety:
//   - All hot-path operations are allocation-free.
//   - No printf/stdout on the EventLoop thread.
//   - Shared state uses SeqLock (lock-free) instead of mutex.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/logging.hpp>

#include "rtc_base/types/types.hpp"
#include "rtc_base/threading/thread_config.hpp"
#include "rtc_base/threading/thread_utils.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "ur5e_hand_driver/fingertip_ft_inferencer.hpp"
#include "ur5e_hand_driver/hand_packets.hpp"
#include "ur5e_hand_driver/hand_sensor_processor.hpp"
#include "ur5e_hand_driver/hand_timing_profiler.hpp"
#include "ur5e_hand_driver/hand_udp_transport.hpp"

namespace rtc {

// ── Communication mode ────────────────────────────────────────────────────────
enum class HandCommunicationMode {
  kIndividual,  // 0x11 pos + 0x12 vel + 0x14~0x17 sensors
  kBulk,        // 0x10 all motors + 0x19 all sensors
};

class HandController {
 public:
  using StateCallback = std::function<void(const HandState&, const FingertipFTState&)>;

  explicit HandController(
      std::string target_ip,
      int target_port,
      const ThreadConfig& thread_cfg = kUdpRecvConfig,
      double recv_timeout_ms = 10.0,
      bool /*enable_write_ack*/ = false,  // deprecated
      int sensor_decimation = 1,
      int num_fingertips = kDefaultNumFingertips,
      bool use_fake_hand = false,
      const std::vector<std::string>& fingertip_names = {},
      HandCommunicationMode communication_mode = HandCommunicationMode::kIndividual,
      bool tof_lpf_enabled = false,
      double tof_lpf_cutoff_hz = 15.0,
      bool baro_lpf_enabled = false,
      double baro_lpf_cutoff_hz = 30.0,
      FingertipFTInferencer::Config ft_config = {},
      bool drift_detection_enabled = false,
      double drift_threshold = 5.0,
      int drift_window_size = 2500) noexcept
      : thread_cfg_(thread_cfg),
        sensor_decimation_(sensor_decimation < 1 ? 1 : sensor_decimation),
        num_fingertips_(num_fingertips > kMaxFingertips ? kMaxFingertips
                       : (num_fingertips < 0 ? 0 : num_fingertips)),
        use_fake_hand_(use_fake_hand),
        fingertip_names_(fingertip_names.empty()
                         ? kDefaultFingertipNames
                         : fingertip_names),
        communication_mode_(communication_mode),
        ft_config_(std::move(ft_config)),
        transport_(std::move(target_ip), target_port, recv_timeout_ms),
        sensor_processor_(HandSensorProcessorConfig{
            num_fingertips_, sensor_decimation_,
            tof_lpf_enabled, tof_lpf_cutoff_hz,
            baro_lpf_enabled, baro_lpf_cutoff_hz,
            drift_detection_enabled, drift_threshold, drift_window_size})
  {
    const int name_count = static_cast<int>(fingertip_names_.size());
    if (name_count < num_fingertips_) {
      num_fingertips_ = name_count;
    }
  }

  ~HandController() { Stop(); }

  HandController(const HandController&)            = delete;
  HandController& operator=(const HandController&) = delete;
  HandController(HandController&&)                 = delete;
  HandController& operator=(HandController&&)      = delete;

  // ── Lifecycle ──────────────────────────────────────────────────────────

  [[nodiscard]] bool Start() {
    // F/T inferencer initialization
    RCLCPP_INFO(rclcpp::get_logger("HandController"),
                "FT config: enabled=%d, num_fingertips=%d, "
                "model_paths.size=%zu, calibration_enabled=%d, calibration_samples=%d",
                ft_config_.enabled ? 1 : 0, num_fingertips_,
                ft_config_.model_paths.size(),
                ft_config_.calibration_enabled ? 1 : 0,
                ft_config_.calibration_samples);
    if (ft_config_.enabled && num_fingertips_ > 0) {
      for (std::size_t i = 0; i < ft_config_.model_paths.size(); ++i) {
        RCLCPP_INFO(rclcpp::get_logger("HandController"),
                    "FT model_path[%zu]=\"%s\"",
                    i, ft_config_.model_paths[i].c_str());
      }
      ft_inferencer_ = std::make_unique<FingertipFTInferencer>();
      try {
        ft_inferencer_->InitFT(ft_config_);
        ft_enabled_ = ft_inferencer_->is_initialized();
        RCLCPP_INFO(rclcpp::get_logger("HandController"),
                    "FT init OK: initialized=%d, num_active_models=%d, calibrated=%d",
                    ft_inferencer_->is_initialized() ? 1 : 0,
                    ft_inferencer_->num_models(),
                    ft_inferencer_->is_calibrated() ? 1 : 0);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("HandController"),
                     "FT init FAILED: %s", e.what());
        ft_enabled_ = false;
        ft_inferencer_.reset();
      } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("HandController"),
                     "FT init FAILED: unknown exception");
        ft_enabled_ = false;
        ft_inferencer_.reset();
      }
    } else {
      RCLCPP_WARN(rclcpp::get_logger("HandController"),
                  "FT inference SKIPPED: enabled=%d, num_fingertips=%d",
                  ft_config_.enabled ? 1 : 0, num_fingertips_);
    }

    if (use_fake_hand_) {
      running_.store(true, std::memory_order_release);
      return true;
    }

    if (!transport_.Open()) return false;

    // Sensor initialization: NN -> RAW mode
    if (num_fingertips_ > 0) {
      sensor_init_ok_ = transport_.InitializeSensors();
    }

    // Sensor processor initialization (rate estimator, filters, drift detector)
    if (num_fingertips_ > 0) {
      sensor_processor_.Init();
    }

    running_.store(true, std::memory_order_release);
    event_thread_ = std::jthread([this](std::stop_token st) {
      EventLoop(std::move(st));
    });

    return true;
  }

  void Stop() noexcept {
    running_.store(false, std::memory_order_release);
    if (use_fake_hand_) { return; }
    event_thread_.request_stop();
    event_cv_.notify_all();
    transport_.Close();
  }

  // ── Callback ───────────────────────────────────────────────────────────

  void SetCallback(StateCallback cb) noexcept {
    callback_ = std::move(cb);
  }

  // ── E-Stop flag (shared with RtControllerNode) ─────────────────────────

  void SetEstopFlag(std::atomic<bool>* flag) noexcept {
    estop_flag_ = flag;
  }

  // ── Event-driven API (called from ControlLoop) ─────────────────────────

  void SendCommandAndRequestStates(
      const std::array<float, kNumHandMotors>& cmd) noexcept {
    // Fake mode: echo-back
    if (use_fake_hand_) {
      HandState fake_state{};
      std::copy(cmd.begin(), cmd.end(), fake_state.motor_positions.begin());
      fake_state.num_fingertips = num_fingertips_;
      fake_state.valid = true;

      if (ft_enabled_ && ft_inferencer_) {
        if (!ft_inferencer_->is_calibrated()) {
          static_cast<void>(ft_inferencer_->FeedCalibration(
              fake_state.sensor_data, num_fingertips_));
        } else {
          auto ft_result = ft_inferencer_->Infer(
              fake_state.sensor_data, num_fingertips_);
          ft_seqlock_.Store(ft_result);
        }
      }

      state_seqlock_.Store(fake_state);
      if (callback_) {
        callback_(fake_state, ft_seqlock_.Load());
      }
      cycle_count_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    if (busy_.load(std::memory_order_acquire)) {
      event_skip_count_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    {
      std::lock_guard lock(event_mutex_);
      staged_cmd_ = cmd;
      event_pending_ = true;
    }
    event_cv_.notify_one();
  }

  // ── Legacy API (standalone hand_udp_node) ──────────────────────────────

  void SetTargetPositions(const std::array<float, kNumHandMotors>& positions) noexcept {
    SendCommandAndRequestStates(positions);
  }

  // ── State access ───────────────────────────────────────────────────────

  [[nodiscard]] HandState GetLatestState() const noexcept {
    return state_seqlock_.Load();
  }

  [[nodiscard]] std::array<float, kNumHandMotors> GetLatestPositions() const noexcept {
    return state_seqlock_.Load().motor_positions;
  }

  [[nodiscard]] bool IsRunning() const noexcept {
    return running_.load(std::memory_order_acquire);
  }

  // ── F/T inference accessors ──────────────────────────────────────────────

  [[nodiscard]] FingertipFTState GetLatestFTState() const noexcept {
    return ft_seqlock_.Load();
  }

  [[nodiscard]] bool ft_inference_enabled() const noexcept {
    return ft_enabled_;
  }

  [[nodiscard]] bool IsSensorInitialized() const noexcept {
    return sensor_init_ok_;
  }

  [[nodiscard]] std::size_t cycle_count() const noexcept {
    return cycle_count_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] uint64_t recv_error_count() const noexcept {
    return transport_.recv_error_count();
  }

  [[nodiscard]] uint64_t event_skip_count() const noexcept {
    return event_skip_count_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] uint64_t consecutive_recv_failures() const noexcept {
    return consecutive_recv_failures_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] HandCommStats comm_stats() const noexcept {
    HandCommStats stats = transport_.comm_stats();
    stats.event_skip_count = event_skip_count_.load(std::memory_order_relaxed);
    return stats;
  }

  [[nodiscard]] HandTimingProfiler::Stats timing_stats() const noexcept {
    return timing_profiler_.GetStats();
  }

  [[nodiscard]] std::string TimingSummary() const noexcept {
    return timing_profiler_.Summary();
  }

  [[nodiscard]] HandCommunicationMode communication_mode() const noexcept {
    return communication_mode_;
  }

  [[nodiscard]] double recv_timeout_ms() const noexcept {
    return transport_.recv_timeout_ms();
  }

  [[nodiscard]] double actual_sensor_rate_hz() const noexcept {
    return sensor_processor_.actual_sensor_rate_hz();
  }

 private:
  // Event-driven loop: condvar wait -> write + read -> sensor processing -> state publish.
  void EventLoop(std::stop_token stop_token) {
    (void)ApplyThreadConfig(thread_cfg_);

    const bool is_bulk = (communication_mode_ == HandCommunicationMode::kBulk);

    std::array<uint8_t, hand_packets::kMotorPacketSize> send_buf{};
    std::array<uint8_t, hand_packets::kMotorPacketSize> echo_buf{};
    std::array<float, hand_packets::kMotorDataCount> motor_pos_buf{};
    std::array<float, hand_packets::kMotorDataCount> motor_vel_buf{};
    std::array<int32_t, kSensorValuesPerFingertip> sensor_raw_buf{};
    std::array<float, kNumHandMotors> pending_cmd{};

    std::array<int32_t, kMaxHandSensors> cached_sensor_data{};
    int sensor_cycle_counter = 0;

    while (!stop_token.stop_requested()) {
      {
        std::unique_lock lock(event_mutex_);
        event_cv_.wait(lock, [&] {
          return event_pending_ || stop_token.stop_requested();
        });
        if (stop_token.stop_requested()) break;
        pending_cmd = staged_cmd_;
        event_pending_ = false;
      }

      busy_.store(true, std::memory_order_release);

      // E-Stop check
      if (estop_flag_ && estop_flag_->load(std::memory_order_acquire)) {
        std::array<float, kNumHandMotors> zeros{};
        transport_.WritePositionFireAndForget(zeros);
        busy_.store(false, std::memory_order_release);
        break;
      }

      HandState state{};
      bool any_recv_ok = false;
      double ft_infer_elapsed_us = 0.0;
      std::array<int32_t, kMaxHandSensors> cached_sensor_data_raw{};

      const auto t0 = std::chrono::steady_clock::now();

      // 1. Write position + recv echo
      if (transport_.WritePositionWithEcho(pending_cmd, send_buf, echo_buf)) {
        any_recv_ok = true;
      }

      const auto t1 = std::chrono::steady_clock::now();

      // Sensor decimation
      ++sensor_cycle_counter;
      const bool is_sensor_cycle = (sensor_cycle_counter >= sensor_decimation_);
      if (is_sensor_cycle) sensor_cycle_counter = 0;

      if (is_bulk) {
        // ── Bulk mode ─────────────────────────────────────────────────────
        if (transport_.RequestAllMotorRead(motor_pos_buf, motor_vel_buf)) {
          std::copy_n(motor_pos_buf.begin(), kNumHandMotors,
                      state.motor_positions.begin());
          std::copy_n(motor_vel_buf.begin(), kNumHandMotors,
                      state.motor_velocities.begin());
          any_recv_ok = true;
        }

        const auto t2 = std::chrono::steady_clock::now();

        if (is_sensor_cycle) {
          if (transport_.RequestAllSensorRead(cached_sensor_data.data(), num_fingertips_,
                                              hand_packets::SensorMode::kRaw)) {
            any_recv_ok = true;
          }
        }

        const auto t3 = std::chrono::steady_clock::now();

        // Sensor processing + FT inference
        if (is_sensor_cycle) {
          cached_sensor_data_raw = cached_sensor_data;
          sensor_processor_.PreFilter();
          sensor_processor_.ApplyFilters(cached_sensor_data);
          sensor_processor_.DetectDrift(cached_sensor_data_raw);
          ft_infer_elapsed_us = RunFTInference(cached_sensor_data);
        }

        const auto t4 = std::chrono::steady_clock::now();

        state.sensor_data_raw = cached_sensor_data_raw;
        state.sensor_data = cached_sensor_data;
        state.num_fingertips = num_fingertips_;
        state.valid = any_recv_ok;

        state_seqlock_.Store(state);
        if (callback_) { callback_(state, ft_seqlock_.Load()); }

        const auto t5 = std::chrono::steady_clock::now();

        HandTimingProfiler::PhaseTiming pt;
        pt.is_bulk_mode       = true;
        pt.write_us           = std::chrono::duration<double, std::micro>(t1 - t0).count();
        pt.read_all_motor_us  = std::chrono::duration<double, std::micro>(t2 - t1).count();
        pt.read_all_sensor_us = std::chrono::duration<double, std::micro>(t3 - t2).count();
        pt.sensor_proc_us     = std::chrono::duration<double, std::micro>(t4 - t3).count()
                                - ft_infer_elapsed_us;
        pt.ft_infer_us        = ft_infer_elapsed_us;
        pt.total_us           = std::chrono::duration<double, std::micro>(t5 - t0).count();
        pt.is_sensor_cycle    = is_sensor_cycle;
        timing_profiler_.Update(pt);

      } else {
        // ── Individual mode ───────────────────────────────────────────────
        if (transport_.RequestMotorRead(hand_packets::Command::kReadPosition, motor_pos_buf)) {
          std::copy_n(motor_pos_buf.begin(), kNumHandMotors,
                      state.motor_positions.begin());
          any_recv_ok = true;
        }

        const auto t2 = std::chrono::steady_clock::now();

        if (transport_.RequestMotorRead(hand_packets::Command::kReadVelocity, motor_vel_buf)) {
          std::copy_n(motor_vel_buf.begin(), kNumHandMotors,
                      state.motor_velocities.begin());
          any_recv_ok = true;
        }

        const auto t3 = std::chrono::steady_clock::now();

        if (is_sensor_cycle) {
          for (int i = 0; i < num_fingertips_; ++i) {
            auto cmd = hand_packets::SensorCommand(i);
            if (transport_.RequestSensorRead(cmd, sensor_raw_buf,
                                             hand_packets::SensorMode::kRaw)) {
              std::copy_n(sensor_raw_buf.begin(), kSensorValuesPerFingertip,
                          cached_sensor_data.begin() + i * kSensorValuesPerFingertip);
              any_recv_ok = true;
            }
          }
        }

        const auto t4 = std::chrono::steady_clock::now();

        if (is_sensor_cycle) {
          cached_sensor_data_raw = cached_sensor_data;
          sensor_processor_.PreFilter();
          sensor_processor_.ApplyFilters(cached_sensor_data);
          sensor_processor_.DetectDrift(cached_sensor_data_raw);
          ft_infer_elapsed_us = RunFTInference(cached_sensor_data);
        }

        const auto t5 = std::chrono::steady_clock::now();

        state.sensor_data_raw = cached_sensor_data_raw;
        state.sensor_data = cached_sensor_data;
        state.num_fingertips = num_fingertips_;
        state.valid = any_recv_ok;

        state_seqlock_.Store(state);
        if (callback_) { callback_(state, ft_seqlock_.Load()); }

        const auto t6 = std::chrono::steady_clock::now();

        HandTimingProfiler::PhaseTiming pt;
        pt.write_us       = std::chrono::duration<double, std::micro>(t1 - t0).count();
        pt.read_pos_us    = std::chrono::duration<double, std::micro>(t2 - t1).count();
        pt.read_vel_us    = std::chrono::duration<double, std::micro>(t3 - t2).count();
        pt.read_sensor_us = std::chrono::duration<double, std::micro>(t4 - t3).count();
        pt.sensor_proc_us = std::chrono::duration<double, std::micro>(t5 - t4).count()
                            - ft_infer_elapsed_us;
        pt.ft_infer_us    = ft_infer_elapsed_us;
        pt.total_us       = std::chrono::duration<double, std::micro>(t6 - t0).count();
        pt.is_sensor_cycle = is_sensor_cycle;
        timing_profiler_.Update(pt);
      }

      transport_.comm_stats_mut().total_cycles++;
      cycle_count_.fetch_add(1, std::memory_order_relaxed);

      if (any_recv_ok) {
        consecutive_recv_failures_.store(0, std::memory_order_relaxed);
      } else {
        consecutive_recv_failures_.fetch_add(1, std::memory_order_relaxed);
      }

      busy_.store(false, std::memory_order_release);
    }
  }

  // Run FT calibration or inference. Returns inference elapsed time in us.
  double RunFTInference(
      const std::array<int32_t, kMaxHandSensors>& sensor_data) noexcept {
    if (!ft_enabled_ || !ft_inferencer_) return 0.0;

    if (!ft_inferencer_->is_calibrated()) {
      static_cast<void>(ft_inferencer_->FeedCalibration(sensor_data, num_fingertips_));
      return 0.0;
    }

    const auto ft_t0 = std::chrono::steady_clock::now();
    auto ft_result = ft_inferencer_->Infer(sensor_data, num_fingertips_);
    const auto ft_t1 = std::chrono::steady_clock::now();
    ft_seqlock_.Store(ft_result);
    return std::chrono::duration<double, std::micro>(ft_t1 - ft_t0).count();
  }

  ThreadConfig thread_cfg_;
  int  sensor_decimation_;
  int  num_fingertips_;
  bool use_fake_hand_;
  std::vector<std::string> fingertip_names_;
  HandCommunicationMode communication_mode_;
  bool sensor_init_ok_{false};

  // F/T inferencer config (before transport_ for initializer list order)
  FingertipFTInferencer::Config ft_config_;

  // Sub-systems
  HandUdpTransport     transport_;
  HandSensorProcessor  sensor_processor_;
  HandTimingProfiler   timing_profiler_;
  std::unique_ptr<FingertipFTInferencer> ft_inferencer_;
  SeqLock<FingertipFTState> ft_seqlock_{};
  bool ft_enabled_{false};

  // E-Stop flag (set by RtControllerNode, null if not used)
  std::atomic<bool>* estop_flag_{nullptr};

  // Event synchronisation
  std::mutex              event_mutex_;
  std::condition_variable event_cv_;
  bool                    event_pending_{false};
  std::array<float, kNumHandMotors> staged_cmd_{};

  // EventLoop busy flag
  std::atomic<bool>     busy_{false};
  std::atomic<uint64_t> event_skip_count_{0};

  // Shared state
  std::atomic<bool> running_{false};
  StateCallback     callback_;
  SeqLock<HandState> state_seqlock_{};
  std::atomic<std::size_t> cycle_count_{0};

  // Link health
  std::atomic<uint64_t> consecutive_recv_failures_{0};

  std::jthread event_thread_;
};

}  // namespace rtc

#endif  // UR5E_HAND_DRIVER_HAND_CONTROLLER_HPP_
