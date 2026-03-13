#ifndef UR5E_HAND_UDP_HAND_FAILURE_DETECTOR_HPP_
#define UR5E_HAND_UDP_HAND_FAILURE_DETECTOR_HPP_

// Non-RT failure detector for hand data.
//
// Runs its own 50 Hz std::jthread.  Reads the latest HandState from
// HandController and checks two failure conditions:
//   1. All-zero data for N consecutive checks
//   2. Duplicate (unchanged) data for N consecutive checks
//
// On failure, invokes a registered callback (typically triggers global E-Stop).

#include "ur5e_hand_udp/hand_controller.hpp"
#include "ur5e_rt_base/types/types.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

namespace ur5e_rt_controller {

struct HandFailureDetectorConfig {
  int  failure_threshold{5};   ///< 연속 감지 횟수 임계값
  bool check_motor{true};      ///< 모터 위치 데이터 검사
  bool check_sensor{true};     ///< 센서 데이터 검사
};

class HandFailureDetector {
public:
  using Config = HandFailureDetectorConfig;
  using FailureCallback = std::function<void(const std::string&)>;

  /// @param controller  Reference to the HandController to monitor.
  /// @param cfg         Detection configuration.
  explicit HandFailureDetector(HandController& controller,
                               Config cfg = Config{})
    : controller_(controller), cfg_(cfg) {}

  ~HandFailureDetector() { Stop(); }

  // Non-copyable, non-movable
  HandFailureDetector(const HandFailureDetector&) = delete;
  HandFailureDetector& operator=(const HandFailureDetector&) = delete;

  /// Register a callback invoked when a failure is detected.
  void SetFailureCallback(FailureCallback cb) { on_failure_ = std::move(cb); }

  /// Start the 50 Hz detector thread.
  void Start() {
    if (running_.load(std::memory_order_relaxed)) return;
    running_.store(true, std::memory_order_relaxed);
    thread_ = std::jthread([this](std::stop_token st) { DetectLoop(st); });
  }

  /// Stop the detector thread.
  void Stop() {
    running_.store(false, std::memory_order_relaxed);
    if (thread_.joinable()) {
      thread_.request_stop();
      thread_.join();
    }
  }

  [[nodiscard]] bool failed() const noexcept {
    return failed_.load(std::memory_order_relaxed);
  }

private:
  void DetectLoop(std::stop_token st) {
    using namespace std::chrono_literals;
    while (!st.stop_requested() && running_.load(std::memory_order_relaxed)) {
      const HandState state = controller_.GetLatestState();
      if (state.valid) {
        Check(state);
      }
      std::this_thread::sleep_for(20ms);  // ~50 Hz
    }
  }

  void Check(const HandState& state) {
    if (cfg_.check_motor) {
      CheckMotor(state);
    }
    if (cfg_.check_sensor) {
      CheckSensor(state);
    }
  }

  void CheckMotor(const HandState& state) {
    const auto& pos = state.motor_positions;

    // All-zero check
    bool all_zero = true;
    for (int i = 0; i < kNumHandMotors; ++i) {
      if (pos[i] != 0.0f) { all_zero = false; break; }
    }
    if (all_zero) {
      ++motor_zero_count_;
    } else {
      motor_zero_count_ = 0;
    }

    // Duplicate check
    if (prev_motor_valid_ && pos == prev_motor_) {
      ++motor_dup_count_;
    } else {
      motor_dup_count_ = 0;
    }
    prev_motor_ = pos;
    prev_motor_valid_ = true;

    if (motor_zero_count_ >= cfg_.failure_threshold) {
      RaiseFailure("hand_motor_all_zero (count=" +
                   std::to_string(motor_zero_count_) + ")");
    }
    if (motor_dup_count_ >= cfg_.failure_threshold) {
      RaiseFailure("hand_motor_duplicate (count=" +
                   std::to_string(motor_dup_count_) + ")");
    }
  }

  void CheckSensor(const HandState& state) {
    const auto& sens = state.sensor_data;

    bool all_zero = true;
    for (int i = 0; i < kNumHandSensors; ++i) {
      if (sens[i] != 0.0f) { all_zero = false; break; }
    }
    if (all_zero) {
      ++sensor_zero_count_;
    } else {
      sensor_zero_count_ = 0;
    }

    if (prev_sensor_valid_ && sens == prev_sensor_) {
      ++sensor_dup_count_;
    } else {
      sensor_dup_count_ = 0;
    }
    prev_sensor_ = sens;
    prev_sensor_valid_ = true;

    if (sensor_zero_count_ >= cfg_.failure_threshold) {
      RaiseFailure("hand_sensor_all_zero (count=" +
                   std::to_string(sensor_zero_count_) + ")");
    }
    if (sensor_dup_count_ >= cfg_.failure_threshold) {
      RaiseFailure("hand_sensor_duplicate (count=" +
                   std::to_string(sensor_dup_count_) + ")");
    }
  }

  void RaiseFailure(const std::string& reason) {
    bool expected = false;
    if (!failed_.compare_exchange_strong(expected, true)) return;
    if (on_failure_) {
      on_failure_(reason);
    }
  }

  HandController& controller_;
  Config          cfg_;
  FailureCallback on_failure_;

  std::jthread       thread_;
  std::atomic<bool>  running_{false};
  std::atomic<bool>  failed_{false};

  // Motor state
  std::array<float, kNumHandMotors> prev_motor_{};
  bool prev_motor_valid_{false};
  int  motor_zero_count_{0};
  int  motor_dup_count_{0};

  // Sensor state
  std::array<float, kNumHandSensors> prev_sensor_{};
  bool prev_sensor_valid_{false};
  int  sensor_zero_count_{0};
  int  sensor_dup_count_{0};
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_HAND_UDP_HAND_FAILURE_DETECTOR_HPP_
