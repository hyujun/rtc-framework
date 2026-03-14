#ifndef UR5E_RT_BASE_DATA_LOGGER_HPP_
#define UR5E_RT_BASE_DATA_LOGGER_HPP_

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>

#include "ur5e_rt_base/logging/log_buffer.hpp"
#include "ur5e_rt_base/types/types.hpp"

namespace ur5e_rt_controller {

// Writes control data to three separate CSV files in a non-RT (logging) thread:
//   - timing_log:  per-phase timing breakdown (6 columns)
//   - robot_log:   robot joint states (49 columns — 4-category: goal/state/command/trajectory)
//   - hand_log:    hand motor states + sensor data (87 columns — 4-category: goal/state/command)
//
// All three files share the same timestamp column for post-hoc join.
// Copy is disabled; move is enabled for deferred construction.
//
// All methods are defined inline to keep ur5e_rt_base header-only.
class DataLogger {
public:
  /// Construct with paths for each log file.  Pass an empty path to disable
  /// that particular log category.
  DataLogger(const std::filesystem::path & timing_path,
             const std::filesystem::path & robot_path,
             const std::filesystem::path & hand_path) {
    if (!timing_path.empty()) {
      timing_file_.open(timing_path);
      if (timing_file_.is_open()) { WriteTimingHeader(); }
    }
    if (!robot_path.empty()) {
      robot_file_.open(robot_path);
      if (robot_file_.is_open()) { WriteRobotHeader(); }
    }
    if (!hand_path.empty()) {
      hand_file_.open(hand_path);
      if (hand_file_.is_open()) { WriteHandHeader(); }
    }
  }

  ~DataLogger() { Flush(); }

  DataLogger(const DataLogger &) = delete;
  DataLogger &operator=(const DataLogger &) = delete;
  DataLogger(DataLogger &&) = default;
  DataLogger &operator=(DataLogger &&) = default;

  // Write one log entry to all open files.
  void LogEntry(const ur5e_rt_controller::LogEntry &entry) {
    WriteTimingRow(entry);
    WriteRobotRow(entry);
    WriteHandRow(entry);
  }

  void Flush() {
    if (timing_file_.is_open()) { timing_file_.flush(); }
    if (robot_file_.is_open())  { robot_file_.flush(); }
    if (hand_file_.is_open())   { hand_file_.flush(); }
  }

  [[nodiscard]] bool IsOpen() const {
    return timing_file_.is_open() || robot_file_.is_open() || hand_file_.is_open();
  }

  // Drains all pending entries from the ring buffer and writes them.
  // Must be called exclusively from the log thread — never from the RT thread.
  void DrainBuffer(ControlLogBuffer &buf) {
    ur5e_rt_controller::LogEntry entry;
    while (buf.Pop(entry)) {
      LogEntry(entry);
    }
  }

private:
  std::ofstream timing_file_;
  std::ofstream robot_file_;
  std::ofstream hand_file_;

  // ── Timing CSV ──────────────────────────────────────────────────────────────
  void WriteTimingHeader() {
    timing_file_ << "timestamp"
                 << ",t_state_acquire_us"
                 << ",t_compute_us"
                 << ",t_publish_us"
                 << ",t_total_us"
                 << ",jitter_us"
                 << '\n';
  }

  void WriteTimingRow(const ur5e_rt_controller::LogEntry &e) {
    if (!timing_file_.is_open()) { return; }
    timing_file_ << std::fixed << std::setprecision(6) << e.timestamp
                 << ',' << e.t_state_acquire_us
                 << ',' << e.t_compute_us
                 << ',' << e.t_publish_us
                 << ',' << e.t_total_us
                 << ',' << e.jitter_us
                 << '\n';
  }

  // ── Robot CSV ───────────────────────────────────────────────────────────────
  // 4-카테고리 순서: Goal → Current State → Command → Trajectory
  void WriteRobotHeader() {
    robot_file_ << "timestamp";
    // 카테고리 1: Goal State
    for (int i = 0; i < kNumRobotJoints; ++i) { robot_file_ << ",goal_pos_" << i; }
    // 카테고리 2: Current State
    for (int i = 0; i < kNumRobotJoints; ++i) { robot_file_ << ",actual_pos_" << i; }
    for (int i = 0; i < kNumRobotJoints; ++i) { robot_file_ << ",actual_vel_" << i; }
    for (int i = 0; i < kNumRobotJoints; ++i) { robot_file_ << ",actual_torque_" << i; }
    for (int i = 0; i < 6; ++i)               { robot_file_ << ",task_pos_" << i; }
    // 카테고리 3: Control Command
    for (int i = 0; i < kNumRobotJoints; ++i) { robot_file_ << ",command_" << i; }
    robot_file_ << ",command_type";
    // 카테고리 4: Trajectory State
    for (int i = 0; i < kNumRobotJoints; ++i) { robot_file_ << ",traj_pos_" << i; }
    for (int i = 0; i < kNumRobotJoints; ++i) { robot_file_ << ",traj_vel_" << i; }
    robot_file_ << '\n';
  }

  void WriteRobotRow(const ur5e_rt_controller::LogEntry &e) {
    if (!robot_file_.is_open()) { return; }
    robot_file_ << std::fixed << std::setprecision(6) << e.timestamp;
    // 카테고리 1: Goal State
    for (const auto v : e.goal_positions)          { robot_file_ << ',' << v; }
    // 카테고리 2: Current State
    for (const auto v : e.actual_positions)        { robot_file_ << ',' << v; }
    for (const auto v : e.actual_velocities)       { robot_file_ << ',' << v; }
    for (const auto v : e.actual_torques)          { robot_file_ << ',' << v; }
    for (const auto v : e.actual_task_positions)   { robot_file_ << ',' << v; }
    // 카테고리 3: Control Command
    for (const auto v : e.robot_commands)          { robot_file_ << ',' << v; }
    robot_file_ << ',' << (e.command_type == CommandType::kPosition ? 0 : 1);
    // 카테고리 4: Trajectory State
    for (const auto v : e.trajectory_positions)    { robot_file_ << ',' << v; }
    for (const auto v : e.trajectory_velocities)   { robot_file_ << ',' << v; }
    robot_file_ << '\n';
  }

  // ── Hand CSV ────────────────────────────────────────────────────────────────
  // 4-카테고리 순서: Goal → Current State → Command
  void WriteHandHeader() {
    hand_file_ << "timestamp"
               << ",hand_valid";
    // 카테고리 1: Goal State
    for (int i = 0; i < kNumHandMotors; ++i) { hand_file_ << ",hand_goal_pos_" << i; }
    // 카테고리 2: Current State
    for (int i = 0; i < kNumHandMotors; ++i) { hand_file_ << ",hand_actual_pos_" << i; }
    for (int i = 0; i < kNumHandMotors; ++i) { hand_file_ << ",hand_actual_vel_" << i; }
    // Sensor columns: barometer (8 per fingertip) + tof (3 per fingertip)
    for (int f = 0; f < kNumFingertips; ++f) {
      for (int b = 0; b < kBarometerCount; ++b) {
        hand_file_ << ",baro_f" << f << "_" << b;
      }
      for (int t = 0; t < kTofCount; ++t) {
        hand_file_ << ",tof_f" << f << "_" << t;
      }
    }
    // 카테고리 3: Control Command
    for (int i = 0; i < kNumHandMotors; ++i) { hand_file_ << ",hand_cmd_" << i; }
    hand_file_ << '\n';
  }

  void WriteHandRow(const ur5e_rt_controller::LogEntry &e) {
    if (!hand_file_.is_open()) { return; }
    hand_file_ << std::fixed << std::setprecision(6) << e.timestamp
               << ',' << (e.hand_valid ? 1 : 0);
    // 카테고리 1: Goal State
    for (const auto v : e.hand_goal_positions)    { hand_file_ << ',' << v; }
    // 카테고리 2: Current State
    for (const auto v : e.hand_actual_positions)  { hand_file_ << ',' << v; }
    for (const auto v : e.hand_actual_velocities) { hand_file_ << ',' << v; }
    for (const auto v : e.hand_sensors)           { hand_file_ << ',' << v; }
    // 카테고리 3: Control Command
    for (const auto v : e.hand_commands)          { hand_file_ << ',' << v; }
    hand_file_ << '\n';
  }
};

} // namespace ur5e_rt_controller

#endif // UR5E_RT_BASE_DATA_LOGGER_HPP_
