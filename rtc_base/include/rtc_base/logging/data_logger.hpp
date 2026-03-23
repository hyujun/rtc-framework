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

// Per-log-file configuration — one per CSV file created by DataLogger.
// Each config corresponds to a publish role (kDeviceStateLog or kDeviceSensorLog)
// in the active controller's YAML topic configuration.
struct DeviceLogConfig {
  std::string device_name;
  PublishRole role{PublishRole::kDeviceStateLog};
  int device_index{0};                         // LogEntry.devices[] index
  std::filesystem::path path;                  // CSV file path (empty = disabled)
  std::vector<std::string> joint_names;        // for CSV header labels
  std::vector<std::string> sensor_names;       // for CSV header labels
  int num_channels{0};                         // expected joint channel count
  int num_sensor_channels{0};                  // expected sensor channel count
};

// Writes control data to role-based CSV files + a timing CSV in a non-RT thread.
//
// File creation is driven by the controller YAML's publish roles:
//   - kDeviceStateLog  → {group}_state_log.csv  (state+cmd+goal+traj+task_pos)
//   - kDeviceSensorLog → {group}_sensor_log.csv (sensor+inference)
//   - timing_log.csv is always created if timing_path is non-empty.
//
// All files share the same timestamp column for post-hoc join.
// All methods are defined inline to keep rtc_base header-only.
class DataLogger {
public:
  DataLogger(const std::filesystem::path & timing_path,
             std::vector<DeviceLogConfig> device_configs,
             int num_inference_values = 0)
      : device_configs_(std::move(device_configs)),
        num_inference_values_(num_inference_values)
  {
    if (num_inference_values_ > kMaxInferenceValues) {
      num_inference_values_ = kMaxInferenceValues;
    }

    if (!timing_path.empty()) {
      timing_file_.open(timing_path);
      if (timing_file_.is_open()) { WriteTimingHeader(); }
    }

    device_files_.resize(device_configs_.size());
    for (std::size_t i = 0; i < device_configs_.size(); ++i) {
      auto& cfg = device_configs_[i];
      if (cfg.num_channels > kMaxDeviceChannels) cfg.num_channels = kMaxDeviceChannels;
      if (cfg.num_sensor_channels > kMaxSensorChannels) cfg.num_sensor_channels = kMaxSensorChannels;

      if (!cfg.path.empty()) {
        device_files_[i].open(cfg.path);
        if (device_files_[i].is_open()) {
          switch (cfg.role) {
            case PublishRole::kDeviceStateLog:
              WriteStateLogHeader(i);
              break;
            case PublishRole::kDeviceSensorLog:
              WriteSensorLogHeader(i);
              break;
            default:
              break;
          }
        }
      }
    }
  }

  ~DataLogger() { Flush(); }

  DataLogger(const DataLogger &) = delete;
  DataLogger &operator=(const DataLogger &) = delete;
  DataLogger(DataLogger &&) = default;
  DataLogger &operator=(DataLogger &&) = default;

  void LogEntry(const rtc::LogEntry &entry) {
    WriteTimingRow(entry);
    for (std::size_t i = 0; i < device_files_.size(); ++i) {
      if (!device_files_[i].is_open()) continue;
      switch (device_configs_[i].role) {
        case PublishRole::kDeviceStateLog:
          WriteStateLogRow(i, entry);
          break;
        case PublishRole::kDeviceSensorLog:
          WriteSensorLogRow(i, entry);
          break;
        default:
          break;
      }
    }
  }

  void Flush() {
    if (timing_file_.is_open()) { timing_file_.flush(); }
    for (auto& f : device_files_) {
      if (f.is_open()) { f.flush(); }
    }
  }

  [[nodiscard]] bool IsOpen() const {
    if (timing_file_.is_open()) return true;
    for (const auto& f : device_files_) {
      if (f.is_open()) return true;
    }
    return false;
  }

  void DrainBuffer(ControlLogBuffer &buf) {
    rtc::LogEntry entry;
    while (buf.Pop(entry)) {
      LogEntry(entry);
    }
  }

private:
  std::ofstream timing_file_;
  std::vector<std::ofstream> device_files_;
  std::vector<DeviceLogConfig> device_configs_;
  int num_inference_values_{0};

  // Label helpers — fall back to numeric index
  std::string JointLabel(std::size_t dev_idx, std::size_t i) const {
    if (dev_idx < device_configs_.size() && i < device_configs_[dev_idx].joint_names.size()) {
      return device_configs_[dev_idx].joint_names[i];
    }
    return std::to_string(i);
  }
  std::string SensorLabel(std::size_t dev_idx, std::size_t i) const {
    if (dev_idx < device_configs_.size() && i < device_configs_[dev_idx].sensor_names.size()) {
      return device_configs_[dev_idx].sensor_names[i];
    }
    return std::to_string(i);
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

  // ── StateLog CSV (DeviceStateLog) ───────────────────────────────────────────
  void WriteStateLogHeader(std::size_t idx) {
    auto& f = device_files_[idx];
    const auto& cfg = device_configs_[idx];
    const int nc = cfg.num_channels;

    f << "timestamp";

    // State
    for (int i = 0; i < nc; ++i) {
      f << ",actual_pos_" << JointLabel(idx, static_cast<std::size_t>(i));
    }
    for (int i = 0; i < nc; ++i) {
      f << ",actual_vel_" << JointLabel(idx, static_cast<std::size_t>(i));
    }
    for (int i = 0; i < nc; ++i) {
      f << ",effort_" << JointLabel(idx, static_cast<std::size_t>(i));
    }

    // Command
    for (int i = 0; i < nc; ++i) {
      f << ",command_" << JointLabel(idx, static_cast<std::size_t>(i));
    }
    f << ",command_type";

    // Goal
    f << ",goal_type";
    for (int i = 0; i < nc; ++i) {
      f << ",joint_goal_" << JointLabel(idx, static_cast<std::size_t>(i));
    }
    for (int i = 0; i < 6; ++i) { f << ",task_goal_" << i; }

    // Trajectory
    for (int i = 0; i < nc; ++i) {
      f << ",traj_pos_" << JointLabel(idx, static_cast<std::size_t>(i));
    }
    for (int i = 0; i < nc; ++i) {
      f << ",traj_vel_" << JointLabel(idx, static_cast<std::size_t>(i));
    }

    // Task-space FK
    for (int i = 0; i < 6; ++i) { f << ",task_pos_" << i; }

    f << '\n';
  }

  void WriteStateLogRow(std::size_t idx, const rtc::LogEntry &e) {
    auto& f = device_files_[idx];
    const auto& cfg = device_configs_[idx];
    const int di = cfg.device_index;
    if (di >= e.num_devices) { return; }

    const auto & d = e.devices[di];
    const int nc = std::min(d.num_channels, cfg.num_channels);

    f << std::fixed << std::setprecision(6) << e.timestamp;

    // State
    for (int i = 0; i < nc; ++i) { f << ',' << d.actual_positions[static_cast<std::size_t>(i)]; }
    for (int i = 0; i < nc; ++i) { f << ',' << d.actual_velocities[static_cast<std::size_t>(i)]; }
    for (int i = 0; i < nc; ++i) { f << ',' << d.efforts[static_cast<std::size_t>(i)]; }

    // Command
    for (int i = 0; i < nc; ++i) { f << ',' << d.commands[static_cast<std::size_t>(i)]; }
    f << ',' << (e.command_type == CommandType::kPosition ? 0 : 1);

    // Goal
    f << ',' << GoalTypeToString(d.goal_type);
    for (int i = 0; i < nc; ++i) { f << ',' << d.goal_positions[static_cast<std::size_t>(i)]; }
    for (const auto v : e.actual_task_positions) { f << ',' << v; }  // task_goal placeholder

    // Trajectory
    for (int i = 0; i < nc; ++i) { f << ',' << d.trajectory_positions[static_cast<std::size_t>(i)]; }
    for (int i = 0; i < nc; ++i) { f << ',' << d.trajectory_velocities[static_cast<std::size_t>(i)]; }

    // Task-space FK
    for (const auto v : e.actual_task_positions) { f << ',' << v; }

    f << '\n';
  }

  // ── SensorLog CSV (DeviceSensorLog) ─────────────────────────────────────────
  void WriteSensorLogHeader(std::size_t idx) {
    auto& f = device_files_[idx];
    const auto& cfg = device_configs_[idx];

    const bool has_sensors = !cfg.sensor_names.empty();

    f << "timestamp";

    // Raw sensor data
    if (has_sensors) {
      for (const auto& sn : cfg.sensor_names) {
        for (int b = 0; b < kBarometerCount; ++b) { f << ",baro_raw_" << sn << "_" << b; }
        for (int t = 0; t < kTofCount; ++t)        { f << ",tof_raw_" << sn << "_" << t; }
      }
      // Filtered sensor data
      for (const auto& sn : cfg.sensor_names) {
        for (int b = 0; b < kBarometerCount; ++b) { f << ",baro_" << sn << "_" << b; }
        for (int t = 0; t < kTofCount; ++t)        { f << ",tof_" << sn << "_" << t; }
      }
    } else {
      const int ns = cfg.num_sensor_channels;
      for (int i = 0; i < ns; ++i) { f << ",sensor_raw_" << i; }
      for (int i = 0; i < ns; ++i) { f << ",sensor_" << i; }
    }

    // Inference output
    f << ",inference_valid";
    if (has_sensors && num_inference_values_ > 0) {
      const char* const kFtComp[] = {
        "contact", "fx", "fy", "fz", "ux", "uy", "uz"};
      constexpr int kFtCompCount = 7;
      int ft_idx = 0;
      for (const auto& sn : cfg.sensor_names) {
        for (int c = 0; c < kFtCompCount && ft_idx < num_inference_values_; ++c, ++ft_idx) {
          f << ",ft_" << sn << "_" << kFtComp[c];
        }
      }
      for (; ft_idx < num_inference_values_; ++ft_idx) { f << ",inference_" << ft_idx; }
    } else {
      for (int i = 0; i < num_inference_values_; ++i) { f << ",inference_" << i; }
    }

    f << '\n';
  }

  void WriteSensorLogRow(std::size_t idx, const rtc::LogEntry &e) {
    auto& f = device_files_[idx];
    const auto& cfg = device_configs_[idx];
    const int di = cfg.device_index;
    if (di >= e.num_devices) { return; }

    const auto & d = e.devices[di];
    const int ns = d.num_sensor_channels;

    f << std::fixed << std::setprecision(6) << e.timestamp;

    // Raw sensor data
    for (int i = 0; i < ns; ++i) {
      f << ',' << d.sensor_data_raw[static_cast<std::size_t>(i)];
    }
    // Filtered sensor data
    for (int i = 0; i < ns; ++i) {
      f << ',' << d.sensor_data[static_cast<std::size_t>(i)];
    }

    // Inference output
    f << ',' << (e.inference_valid ? 1 : 0);
    const int ni = e.num_inference_values;
    for (int i = 0; i < ni; ++i) {
      f << ',' << e.inference_output[static_cast<std::size_t>(i)];
    }

    f << '\n';
  }
};

} // namespace rtc

#endif // RTC_BASE_DATA_LOGGER_HPP_
