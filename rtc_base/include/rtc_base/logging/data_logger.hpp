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

// Per-device log configuration — passed to DataLogger at construction time.
struct DeviceLogConfig {
  std::string device_name;
  std::filesystem::path path;                // CSV file path (empty = disabled)
  std::vector<std::string> joint_names;      // for CSV header labels
  std::vector<std::string> sensor_names;     // for CSV header labels
  int num_channels{0};                       // expected joint channel count
  int num_sensor_channels{0};                // expected sensor channel count
};

// Writes control data to per-device CSV files + a timing CSV in a non-RT thread:
//   - timing_log:     per-phase timing breakdown
//   - <device>_log:   per-device joint states, sensor data, commands
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
        if (device_files_[i].is_open()) { WriteDeviceHeader(i); }
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
      WriteDeviceRow(i, entry);
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

  // Joint/sensor name label helper — falls back to numeric index
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

  // ── Per-device CSV ──────────────────────────────────────────────────────────
  void WriteDeviceHeader(std::size_t dev_idx) {
    auto& f = device_files_[dev_idx];
    const auto& cfg = device_configs_[dev_idx];
    const int nc = cfg.num_channels;
    const int ns = cfg.num_sensor_channels;

    // Devices with sensors (e.g. hand) use device-name prefix so that
    // plot_rtc_log.py can distinguish columns per device type.
    const bool has_sensors = !cfg.sensor_names.empty();
    const std::string dp = has_sensors ? cfg.device_name + "_" : "";

    f << "timestamp"
      << "," << dp << "valid";

    // Goal State
    for (int i = 0; i < nc; ++i) {
      f << "," << dp << "goal_pos_" << JointLabel(dev_idx, static_cast<std::size_t>(i));
    }
    // Current State
    for (int i = 0; i < nc; ++i) {
      f << "," << dp << "actual_pos_" << JointLabel(dev_idx, static_cast<std::size_t>(i));
    }
    for (int i = 0; i < nc; ++i) {
      f << "," << dp << "actual_vel_" << JointLabel(dev_idx, static_cast<std::size_t>(i));
    }
    for (int i = 0; i < nc; ++i) {
      f << "," << dp << "actual_torque_" << JointLabel(dev_idx, static_cast<std::size_t>(i));
    }

    // Task position (shared, only for first device)
    if (dev_idx == 0) {
      for (int i = 0; i < 6; ++i) { f << ",task_pos_" << i; }
    }

    // Sensor data
    if (has_sensors) {
      // Structured naming: baro_raw_{label}_{ch}, tof_raw_{label}_{ch}
      for (const auto& sn : cfg.sensor_names) {
        for (int b = 0; b < kBarometerCount; ++b) { f << ",baro_raw_" << sn << "_" << b; }
        for (int t = 0; t < kTofCount; ++t)        { f << ",tof_raw_" << sn << "_" << t; }
      }
      for (const auto& sn : cfg.sensor_names) {
        for (int b = 0; b < kBarometerCount; ++b) { f << ",baro_" << sn << "_" << b; }
        for (int t = 0; t < kTofCount; ++t)        { f << ",tof_" << sn << "_" << t; }
      }
    } else {
      // Numeric fallback for devices without named sensors
      for (int i = 0; i < ns; ++i) { f << ",sensor_raw_" << i; }
      for (int i = 0; i < ns; ++i) { f << ",sensor_" << i; }
    }

    // Inference output (last device only)
    if (dev_idx == device_configs_.size() - 1) {
      f << ",ft_valid";
      if (has_sensors && num_inference_values_ > 0) {
        // 3-head F/T model: 7 outputs per fingertip
        const char* const kFtComp[] = {
          "contact", "fx", "fy", "fz", "ux", "uy", "uz"};
        constexpr int kFtCompCount = 7;
        int idx = 0;
        for (const auto& sn : cfg.sensor_names) {
          for (int c = 0; c < kFtCompCount && idx < num_inference_values_; ++c, ++idx) {
            f << ",ft_" << sn << "_" << kFtComp[c];
          }
        }
        // Remaining values (if model has extra outputs)
        for (; idx < num_inference_values_; ++idx) { f << ",inference_" << idx; }
      } else {
        for (int i = 0; i < num_inference_values_; ++i) { f << ",inference_" << i; }
      }
    }

    // Control Command — use "cmd" for device log, "command" for robot log
    for (int i = 0; i < nc; ++i) {
      if (has_sensors) {
        f << "," << dp << "cmd_" << JointLabel(dev_idx, static_cast<std::size_t>(i));
      } else {
        f << ",command_" << JointLabel(dev_idx, static_cast<std::size_t>(i));
      }
    }

    if (dev_idx == 0) {
      f << ",command_type";
    }

    // Trajectory State
    for (int i = 0; i < nc; ++i) {
      f << "," << dp << "traj_pos_" << JointLabel(dev_idx, static_cast<std::size_t>(i));
    }
    for (int i = 0; i < nc; ++i) {
      f << "," << dp << "traj_vel_" << JointLabel(dev_idx, static_cast<std::size_t>(i));
    }

    f << '\n';
  }

  void WriteDeviceRow(std::size_t dev_idx, const rtc::LogEntry &e) {
    auto& f = device_files_[dev_idx];
    if (!f.is_open()) { return; }
    if (static_cast<int>(dev_idx) >= e.num_devices) { return; }

    const auto& cfg = device_configs_[dev_idx];
    const auto & d = e.devices[dev_idx];
    const int nc = std::min(d.num_channels, cfg.num_channels);

    f << std::fixed << std::setprecision(6) << e.timestamp
      << ',' << (d.valid ? 1 : 0);

    // Goal State
    for (int i = 0; i < nc; ++i) { f << ',' << d.goal_positions[static_cast<std::size_t>(i)]; }
    // Current State
    for (int i = 0; i < nc; ++i) { f << ',' << d.actual_positions[static_cast<std::size_t>(i)]; }
    for (int i = 0; i < nc; ++i) { f << ',' << d.actual_velocities[static_cast<std::size_t>(i)]; }
    for (int i = 0; i < nc; ++i) { f << ',' << d.efforts[static_cast<std::size_t>(i)]; }

    // Task position (shared, only for first device)
    if (dev_idx == 0) {
      for (const auto v : e.actual_task_positions) { f << ',' << v; }
    }

    // Sensor data
    const int ns = d.num_sensor_channels;
    for (int i = 0; i < ns; ++i) {
      f << ',' << d.sensor_data_raw[static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < ns; ++i) {
      f << ',' << d.sensor_data[static_cast<std::size_t>(i)];
    }

    // Inference output (last device only)
    if (dev_idx == device_configs_.size() - 1) {
      f << ',' << (e.inference_valid ? 1 : 0);
      const int ni = e.num_inference_values;
      for (int i = 0; i < ni; ++i) {
        f << ',' << e.inference_output[static_cast<std::size_t>(i)];
      }
    }

    // Control Command
    for (int i = 0; i < nc; ++i) { f << ',' << d.commands[static_cast<std::size_t>(i)]; }

    if (dev_idx == 0) {
      f << ',' << (e.command_type == CommandType::kPosition ? 0 : 1);
    }

    // Trajectory State
    for (int i = 0; i < nc; ++i) { f << ',' << d.trajectory_positions[static_cast<std::size_t>(i)]; }
    for (int i = 0; i < nc; ++i) { f << ',' << d.trajectory_velocities[static_cast<std::size_t>(i)]; }

    f << '\n';
  }
};

} // namespace rtc

#endif // RTC_BASE_DATA_LOGGER_HPP_
