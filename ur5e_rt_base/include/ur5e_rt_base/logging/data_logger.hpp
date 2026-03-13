#ifndef UR5E_RT_BASE_DATA_LOGGER_HPP_
#define UR5E_RT_BASE_DATA_LOGGER_HPP_

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <span>

#include <iomanip>

#include "ur5e_rt_base/logging/log_buffer.hpp"
#include "ur5e_rt_base/types/types.hpp"

namespace ur5e_rt_controller {

// Writes control data to a CSV file in a non-RT (logging) thread context.
// Copy is disabled; move is enabled for deferred construction.
//
// All methods are defined inline to keep ur5e_rt_base header-only.
class DataLogger {
public:
  explicit DataLogger(std::filesystem::path log_path) {
    file_.open(log_path);
    if (file_.is_open()) {
      WriteHeader();
    }
  }

  ~DataLogger() { Flush(); }

  DataLogger(const DataLogger &) = delete;
  DataLogger &operator=(const DataLogger &) = delete;
  DataLogger(DataLogger &&) = default;
  DataLogger &operator=(DataLogger &&) = default;

  // Log one control step with full per-phase timing breakdown.
  void LogEntry(const ur5e_rt_controller::LogEntry &entry) {
    if (!file_.is_open()) {
      return;
    }
    file_ << std::fixed << std::setprecision(6) << entry.timestamp;
    for (const auto v : entry.current_positions) {
      file_ << ',' << v;
    }
    for (const auto v : entry.target_positions) {
      file_ << ',' << v;
    }
    for (const auto v : entry.commands) {
      file_ << ',' << v;
    }
    file_ << ',' << entry.t_state_acquire_us
           << ',' << entry.t_compute_us
           << ',' << entry.t_publish_us
           << ',' << entry.t_total_us
           << ',' << entry.jitter_us;
    // Hand state
    file_ << ',' << (entry.hand_valid ? 1 : 0);
    for (const auto v : entry.hand_positions) {
      file_ << ',' << v;
    }
    for (const auto v : entry.hand_velocities) {
      file_ << ',' << v;
    }
    for (const auto v : entry.hand_sensors) {
      file_ << ',' << v;
    }
    file_ << '\n';
  }

  // Legacy overload — delegates to the new LogEntry-based method.
  void
  LogControlData(double timestamp,
                 std::span<const double, kNumRobotJoints> current_positions,
                 std::span<const double, kNumRobotJoints> target_positions,
                 std::span<const double, kNumRobotJoints> commands,
                 double compute_time_us = 0.0) {
    if (!file_.is_open()) {
      return;
    }
    file_ << std::fixed << std::setprecision(6) << timestamp;
    for (const auto v : current_positions) {
      file_ << ',' << v;
    }
    for (const auto v : target_positions) {
      file_ << ',' << v;
    }
    for (const auto v : commands) {
      file_ << ',' << v;
    }
    // Per-phase timing: only compute_time_us available in legacy API
    file_ << ',' << 0.0            // t_state_acquire_us
           << ',' << compute_time_us  // t_compute_us
           << ',' << 0.0            // t_publish_us
           << ',' << 0.0            // t_total_us
           << ',' << 0.0            // jitter_us
           << '\n';
  }

  // Log hand state for a given timestamp (not written to the control CSV).
  void LogHandData(double /*timestamp*/,
                   std::span<const float, kNumHandMotors> /*hand_positions*/) {
    // Reserved for future hand logging support.
  }

  void Flush() {
    if (file_.is_open()) {
      file_.flush();
    }
  }

  [[nodiscard]] bool IsOpen() const { return file_.is_open(); }

  // Drains all pending entries from the ring buffer and writes them to the CSV.
  // Must be called exclusively from the log thread — never from the RT thread.
  void DrainBuffer(ControlLogBuffer &buf) {
    ur5e_rt_controller::LogEntry entry;
    while (buf.Pop(entry)) {
      LogEntry(entry);
    }
  }

private:
  std::ofstream file_;

  void WriteHeader() {
    file_ << "timestamp";
    for (int i = 0; i < kNumRobotJoints; ++i) {
      file_ << ",current_pos_" << i;
    }
    for (int i = 0; i < kNumRobotJoints; ++i) {
      file_ << ",target_pos_" << i;
    }
    for (int i = 0; i < kNumRobotJoints; ++i) {
      file_ << ",command_" << i;
    }
    file_ << ",t_state_acquire_us"
           << ",t_compute_us"
           << ",t_publish_us"
           << ",t_total_us"
           << ",jitter_us"
           << ",hand_valid";
    for (int i = 0; i < kNumHandMotors; ++i) {
      file_ << ",hand_pos_" << i;
    }
    for (int i = 0; i < kNumHandMotors; ++i) {
      file_ << ",hand_vel_" << i;
    }
    for (int i = 0; i < kNumHandSensors; ++i) {
      file_ << ",hand_sensor_" << i;
    }
    file_ << '\n';
  }
};

} // namespace ur5e_rt_controller

#endif // UR5E_RT_BASE_DATA_LOGGER_HPP_
