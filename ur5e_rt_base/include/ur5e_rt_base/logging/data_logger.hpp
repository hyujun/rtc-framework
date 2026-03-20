#ifndef UR5E_RT_BASE_DATA_LOGGER_HPP_
#define UR5E_RT_BASE_DATA_LOGGER_HPP_

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

#include "ur5e_rt_base/logging/log_buffer.hpp"
#include "ur5e_rt_base/types/types.hpp"

namespace ur5e_rt_controller {

// Writes control data to three separate CSV files in a non-RT (logging) thread:
//   - timing_log:  per-phase timing breakdown (6 columns)
//   - robot_log:   robot joint states (49 columns — 4-category: goal/state/command/trajectory)
//   - hand_log:    hand motor states + sensor data (variable columns — 4-category)
//
// All three files share the same timestamp column for post-hoc join.
// Copy is disabled; move is enabled for deferred construction.
//
// All methods are defined inline to keep ur5e_rt_base header-only.
class DataLogger {
public:
  /// Construct with paths for each log file.  Pass an empty path to disable
  /// that particular log category.
  /// joint_names/motor_names/fingertip_names are used for CSV headers (v5.14.0).
  /// Empty vectors → numeric index fallback.
  DataLogger(const std::filesystem::path & timing_path,
             const std::filesystem::path & robot_path,
             const std::filesystem::path & hand_path,
             const std::vector<std::string>& joint_names = {},
             const std::vector<std::string>& motor_names = {},
             const std::vector<std::string>& fingertip_names = {})
      : joint_names_(joint_names),
        motor_names_(motor_names),
        fingertip_names_(fingertip_names),
        num_fingertips_(fingertip_names.empty()
                            ? kDefaultNumFingertips
                            : static_cast<int>(fingertip_names.size()))
  {
    if (num_fingertips_ > kMaxFingertips) { num_fingertips_ = kMaxFingertips; }

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

  // 이름 벡터 (v5.14.0)
  std::vector<std::string> joint_names_;
  std::vector<std::string> motor_names_;
  std::vector<std::string> fingertip_names_;
  int num_fingertips_;

  // 조인트 이름 또는 인덱스 문자열 반환
  std::string JointLabel(std::size_t i) const {
    return (i < joint_names_.size()) ? joint_names_[i] : std::to_string(i);
  }
  std::string MotorLabel(std::size_t i) const {
    return (i < motor_names_.size()) ? motor_names_[i] : std::to_string(i);
  }
  std::string FingertipLabel(int f) const {
    return (static_cast<std::size_t>(f) < fingertip_names_.size())
               ? fingertip_names_[static_cast<std::size_t>(f)]
               : "f" + std::to_string(f);
  }

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
    for (int i = 0; i < kNumRobotJoints; ++i) {
      robot_file_ << ",goal_pos_" << JointLabel(static_cast<std::size_t>(i));
    }
    // 카테고리 2: Current State
    for (int i = 0; i < kNumRobotJoints; ++i) {
      robot_file_ << ",actual_pos_" << JointLabel(static_cast<std::size_t>(i));
    }
    for (int i = 0; i < kNumRobotJoints; ++i) {
      robot_file_ << ",actual_vel_" << JointLabel(static_cast<std::size_t>(i));
    }
    for (int i = 0; i < kNumRobotJoints; ++i) {
      robot_file_ << ",actual_torque_" << JointLabel(static_cast<std::size_t>(i));
    }
    for (int i = 0; i < 6; ++i)               { robot_file_ << ",task_pos_" << i; }
    // 카테고리 3: Control Command
    for (int i = 0; i < kNumRobotJoints; ++i) {
      robot_file_ << ",command_" << JointLabel(static_cast<std::size_t>(i));
    }
    robot_file_ << ",command_type";
    // 카테고리 4: Trajectory State
    for (int i = 0; i < kNumRobotJoints; ++i) {
      robot_file_ << ",traj_pos_" << JointLabel(static_cast<std::size_t>(i));
    }
    for (int i = 0; i < kNumRobotJoints; ++i) {
      robot_file_ << ",traj_vel_" << JointLabel(static_cast<std::size_t>(i));
    }
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
    for (int i = 0; i < kNumHandMotors; ++i) {
      hand_file_ << ",hand_goal_pos_" << MotorLabel(static_cast<std::size_t>(i));
    }
    // 카테고리 2: Current State
    for (int i = 0; i < kNumHandMotors; ++i) {
      hand_file_ << ",hand_actual_pos_" << MotorLabel(static_cast<std::size_t>(i));
    }
    for (int i = 0; i < kNumHandMotors; ++i) {
      hand_file_ << ",hand_actual_vel_" << MotorLabel(static_cast<std::size_t>(i));
    }
    // Stage 1: Raw sensor (pre-LPF)
    for (int f = 0; f < num_fingertips_; ++f) {
      const auto fl = FingertipLabel(f);
      for (int b = 0; b < kBarometerCount; ++b) {
        hand_file_ << ",baro_raw_" << fl << "_" << b;
      }
      for (int t = 0; t < kTofCount; ++t) {
        hand_file_ << ",tof_raw_" << fl << "_" << t;
      }
    }
    // Stage 2: Filtered sensor (post-LPF, pre-inference)
    for (int f = 0; f < num_fingertips_; ++f) {
      const auto fl = FingertipLabel(f);
      for (int b = 0; b < kBarometerCount; ++b) {
        hand_file_ << ",baro_" << fl << "_" << b;
      }
      for (int t = 0; t < kTofCount; ++t) {
        hand_file_ << ",tof_" << fl << "_" << t;
      }
    }
    // Stage 3: F/T inference output
    // Output: [contact(1), F(3), u(3), Fn(3), Fx(1), Fy(1), Fz(1)] = 13 per fingertip
    hand_file_ << ",ft_valid";
    for (int f = 0; f < num_fingertips_; ++f) {
      const auto fl = FingertipLabel(f);
      hand_file_ << ",ft_" << fl << "_contact"
                 << ",ft_" << fl << "_fx"
                 << ",ft_" << fl << "_fy"
                 << ",ft_" << fl << "_fz"
                 << ",ft_" << fl << "_ux"
                 << ",ft_" << fl << "_uy"
                 << ",ft_" << fl << "_uz"
                 << ",ft_" << fl << "_fnx"
                 << ",ft_" << fl << "_fny"
                 << ",ft_" << fl << "_fnz"
                 << ",ft_" << fl << "_fx_scalar"
                 << ",ft_" << fl << "_fy_scalar"
                 << ",ft_" << fl << "_fz_scalar";
    }
    // 카테고리 3: Control Command
    for (int i = 0; i < kNumHandMotors; ++i) {
      hand_file_ << ",hand_cmd_" << MotorLabel(static_cast<std::size_t>(i));
    }
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
    // Stage 1: Raw sensor data (pre-LPF)
    const int num_sensors = e.num_fingertips * kSensorValuesPerFingertip;
    for (int i = 0; i < num_sensors; ++i) {
      hand_file_ << ',' << e.hand_sensors_raw[static_cast<std::size_t>(i)];
    }
    // Stage 2: Filtered sensor data (post-LPF)
    for (int i = 0; i < num_sensors; ++i) {
      hand_file_ << ',' << e.hand_sensors[static_cast<std::size_t>(i)];
    }
    // Stage 3: F/T inference output
    hand_file_ << ',' << (e.hand_ft_valid ? 1 : 0);
    const int num_ft = e.num_fingertips * kFTValuesPerFingertip;
    for (int i = 0; i < num_ft; ++i) {
      hand_file_ << ',' << e.hand_ft_data[static_cast<std::size_t>(i)];
    }
    // 카테고리 3: Control Command
    for (const auto v : e.hand_commands)          { hand_file_ << ',' << v; }
    hand_file_ << '\n';
  }
};

} // namespace ur5e_rt_controller

#endif // UR5E_RT_BASE_DATA_LOGGER_HPP_
