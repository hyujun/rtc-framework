#ifndef RTC_BASE_DATA_LOGGER_HPP_
#define RTC_BASE_DATA_LOGGER_HPP_

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

#include "rtc_base/logging/log_buffer.hpp"
#include "rtc_base/types/types.hpp"

namespace rtc {

// Writes control data to three separate CSV files in a non-RT (logging) thread:
//   - timing_log:  per-phase timing breakdown (6 columns)
//   - robot_log:   robot joint states (variable columns — 4-category: goal/state/command/trajectory)
//   - device_log:  device motor states + sensor data (variable columns — 4-category)
//
// All three files share the same timestamp column for post-hoc join.
// Copy is disabled; move is enabled for deferred construction.
//
// All methods are defined inline to keep rtc_base header-only.
class DataLogger {
public:
  /// Construct with paths for each log file.  Pass an empty path to disable
  /// that particular log category.
  /// joint_names/motor_names/fingertip_names are used for CSV headers (v5.14.0).
  /// Empty vectors → numeric index fallback.
  /// num_robot_joints / num_device_channels control how many columns are written.
  DataLogger(const std::filesystem::path & timing_path,
             const std::filesystem::path & robot_path,
             const std::filesystem::path & device_path,
             const std::vector<std::string>& joint_names = {},
             const std::vector<std::string>& motor_names = {},
             const std::vector<std::string>& fingertip_names = {},
             int num_robot_joints = kNumRobotJoints,
             int num_device_channels = kNumHandMotors,
             int num_sensor_channels = 0,
             int num_inference_values = 0)
      : joint_names_(joint_names),
        motor_names_(motor_names),
        fingertip_names_(fingertip_names),
        num_robot_joints_(num_robot_joints),
        num_device_channels_(num_device_channels),
        num_sensor_channels_(num_sensor_channels),
        num_inference_values_(num_inference_values),
        num_fingertips_(fingertip_names.empty()
                            ? kDefaultNumFingertips
                            : static_cast<int>(fingertip_names.size()))
  {
    if (num_fingertips_ > kMaxFingertips) { num_fingertips_ = kMaxFingertips; }
    if (num_robot_joints_ > kMaxRobotDOF) { num_robot_joints_ = kMaxRobotDOF; }
    if (num_device_channels_ > kMaxDeviceChannels) { num_device_channels_ = kMaxDeviceChannels; }
    if (num_sensor_channels_ > kMaxSensorChannels) { num_sensor_channels_ = kMaxSensorChannels; }
    if (num_inference_values_ > kMaxInferenceValues) { num_inference_values_ = kMaxInferenceValues; }

    if (!timing_path.empty()) {
      timing_file_.open(timing_path);
      if (timing_file_.is_open()) { WriteTimingHeader(); }
    }
    if (!robot_path.empty()) {
      robot_file_.open(robot_path);
      if (robot_file_.is_open()) { WriteRobotHeader(); }
    }
    if (!device_path.empty()) {
      device_file_.open(device_path);
      if (device_file_.is_open()) { WriteDeviceHeader(); }
    }
  }

  ~DataLogger() { Flush(); }

  DataLogger(const DataLogger &) = delete;
  DataLogger &operator=(const DataLogger &) = delete;
  DataLogger(DataLogger &&) = default;
  DataLogger &operator=(DataLogger &&) = default;

  // Write one log entry to all open files.
  void LogEntry(const rtc::LogEntry &entry) {
    WriteTimingRow(entry);
    WriteRobotRow(entry);
    WriteDeviceRow(entry);
  }

  void Flush() {
    if (timing_file_.is_open()) { timing_file_.flush(); }
    if (robot_file_.is_open())  { robot_file_.flush(); }
    if (device_file_.is_open()) { device_file_.flush(); }
  }

  [[nodiscard]] bool IsOpen() const {
    return timing_file_.is_open() || robot_file_.is_open() || device_file_.is_open();
  }

  // Drains all pending entries from the ring buffer and writes them.
  // Must be called exclusively from the log thread — never from the RT thread.
  void DrainBuffer(ControlLogBuffer &buf) {
    rtc::LogEntry entry;
    while (buf.Pop(entry)) {
      LogEntry(entry);
    }
  }

private:
  std::ofstream timing_file_;
  std::ofstream robot_file_;
  std::ofstream device_file_;

  // 이름 벡터 (v5.14.0)
  std::vector<std::string> joint_names_;
  std::vector<std::string> motor_names_;
  std::vector<std::string> fingertip_names_;
  int num_robot_joints_;
  int num_device_channels_;
  int num_sensor_channels_;
  int num_inference_values_;
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

  void WriteTimingRow(const rtc::LogEntry &e) {
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
    const int nj = num_robot_joints_;
    robot_file_ << "timestamp";
    // 카테고리 1: Goal State
    for (int i = 0; i < nj; ++i) {
      robot_file_ << ",goal_pos_" << JointLabel(static_cast<std::size_t>(i));
    }
    // 카테고리 2: Current State
    for (int i = 0; i < nj; ++i) {
      robot_file_ << ",actual_pos_" << JointLabel(static_cast<std::size_t>(i));
    }
    for (int i = 0; i < nj; ++i) {
      robot_file_ << ",actual_vel_" << JointLabel(static_cast<std::size_t>(i));
    }
    for (int i = 0; i < nj; ++i) {
      robot_file_ << ",actual_torque_" << JointLabel(static_cast<std::size_t>(i));
    }
    for (int i = 0; i < 6; ++i)               { robot_file_ << ",task_pos_" << i; }
    // 카테고리 3: Control Command
    for (int i = 0; i < nj; ++i) {
      robot_file_ << ",command_" << JointLabel(static_cast<std::size_t>(i));
    }
    robot_file_ << ",command_type";
    // 카테고리 4: Trajectory State
    for (int i = 0; i < nj; ++i) {
      robot_file_ << ",traj_pos_" << JointLabel(static_cast<std::size_t>(i));
    }
    for (int i = 0; i < nj; ++i) {
      robot_file_ << ",traj_vel_" << JointLabel(static_cast<std::size_t>(i));
    }
    robot_file_ << '\n';
  }

  void WriteRobotRow(const rtc::LogEntry &e) {
    if (!robot_file_.is_open()) { return; }
    const int nj = e.num_robot_joints;
    robot_file_ << std::fixed << std::setprecision(6) << e.timestamp;
    // 카테고리 1: Goal State
    for (int i = 0; i < nj; ++i) { robot_file_ << ',' << e.goal_positions[static_cast<std::size_t>(i)]; }
    // 카테고리 2: Current State
    for (int i = 0; i < nj; ++i) { robot_file_ << ',' << e.actual_positions[static_cast<std::size_t>(i)]; }
    for (int i = 0; i < nj; ++i) { robot_file_ << ',' << e.actual_velocities[static_cast<std::size_t>(i)]; }
    for (int i = 0; i < nj; ++i) { robot_file_ << ',' << e.actual_torques[static_cast<std::size_t>(i)]; }
    for (const auto v : e.actual_task_positions)   { robot_file_ << ',' << v; }
    // 카테고리 3: Control Command
    for (int i = 0; i < nj; ++i) { robot_file_ << ',' << e.robot_commands[static_cast<std::size_t>(i)]; }
    robot_file_ << ',' << (e.command_type == CommandType::kPosition ? 0 : 1);
    // 카테고리 4: Trajectory State
    for (int i = 0; i < nj; ++i) { robot_file_ << ',' << e.trajectory_positions[static_cast<std::size_t>(i)]; }
    for (int i = 0; i < nj; ++i) { robot_file_ << ',' << e.trajectory_velocities[static_cast<std::size_t>(i)]; }
    robot_file_ << '\n';
  }

  // ── Device CSV ────────────────────────────────────────────────────────────────
  // 4-카테고리 순서: Goal → Current State → Command
  void WriteDeviceHeader() {
    const int nc = num_device_channels_;
    device_file_ << "timestamp"
                 << ",device_valid";
    // 카테고리 1: Goal State
    for (int i = 0; i < nc; ++i) {
      device_file_ << ",device_goal_" << MotorLabel(static_cast<std::size_t>(i));
    }
    // 카테고리 2: Current State
    for (int i = 0; i < nc; ++i) {
      device_file_ << ",device_actual_" << MotorLabel(static_cast<std::size_t>(i));
    }
    for (int i = 0; i < nc; ++i) {
      device_file_ << ",device_vel_" << MotorLabel(static_cast<std::size_t>(i));
    }
    // Sensor data: raw (pre-LPF)
    for (int i = 0; i < num_sensor_channels_; ++i) {
      device_file_ << ",sensor_raw_" << i;
    }
    // Sensor data: filtered (post-LPF)
    for (int i = 0; i < num_sensor_channels_; ++i) {
      device_file_ << ",sensor_" << i;
    }
    // Inference output
    device_file_ << ",inference_valid";
    for (int i = 0; i < num_inference_values_; ++i) {
      device_file_ << ",inference_" << i;
    }
    // 카테고리 3: Control Command
    for (int i = 0; i < nc; ++i) {
      device_file_ << ",device_cmd_" << MotorLabel(static_cast<std::size_t>(i));
    }
    device_file_ << '\n';
  }

  void WriteDeviceRow(const rtc::LogEntry &e) {
    if (!device_file_.is_open()) { return; }
    const int nc = e.num_device_channels;
    device_file_ << std::fixed << std::setprecision(6) << e.timestamp
                 << ',' << (e.device_valid ? 1 : 0);
    // 카테고리 1: Goal State
    for (int i = 0; i < nc; ++i) { device_file_ << ',' << e.device_goal[static_cast<std::size_t>(i)]; }
    // 카테고리 2: Current State
    for (int i = 0; i < nc; ++i) { device_file_ << ',' << e.device_actual[static_cast<std::size_t>(i)]; }
    for (int i = 0; i < nc; ++i) { device_file_ << ',' << e.device_velocities[static_cast<std::size_t>(i)]; }
    // Sensor data: raw (pre-LPF)
    const int ns = e.num_sensor_channels;
    for (int i = 0; i < ns; ++i) {
      device_file_ << ',' << e.sensor_data_raw[static_cast<std::size_t>(i)];
    }
    // Sensor data: filtered (post-LPF)
    for (int i = 0; i < ns; ++i) {
      device_file_ << ',' << e.sensor_data[static_cast<std::size_t>(i)];
    }
    // Inference output
    device_file_ << ',' << (e.inference_valid ? 1 : 0);
    const int ni = e.num_inference_values;
    for (int i = 0; i < ni; ++i) {
      device_file_ << ',' << e.inference_output[static_cast<std::size_t>(i)];
    }
    // 카테고리 3: Control Command
    for (int i = 0; i < nc; ++i) { device_file_ << ',' << e.device_commands[static_cast<std::size_t>(i)]; }
    device_file_ << '\n';
  }
};

} // namespace rtc

#endif // RTC_BASE_DATA_LOGGER_HPP_
