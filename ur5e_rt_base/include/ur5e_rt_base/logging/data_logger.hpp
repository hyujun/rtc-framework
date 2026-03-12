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

  // Log one control step: timestamp (s), current positions, target positions,
  // computed commands (all kNumRobotJoints elements), and optional compute
  // timing in µs (0.0 if profiling is disabled).
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
    file_ << ',' << compute_time_us << '\n';
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
    LogEntry entry;
    while (buf.Pop(entry)) {
      LogControlData(entry.timestamp, entry.current_positions,
                     entry.target_positions, entry.commands,
                     entry.compute_time_us);
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
    file_ << ",compute_time_us\n";
  }
};

} // namespace ur5e_rt_controller

#endif // UR5E_RT_BASE_DATA_LOGGER_HPP_
