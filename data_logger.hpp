// data_logger.hpp - v2 (CSV Logging)
#pragma once

#include <string>
#include <fstream>
#include <vector>
#include <chrono>
#include <memory>

namespace ur5e_controller {

class DataLogger {
public:
  explicit DataLogger(const std::string& filename);
  ~DataLogger();
  
  // Disable copy
  DataLogger(const DataLogger&) = delete;
  DataLogger& operator=(const DataLogger&) = delete;
  
  // Enable move
  DataLogger(DataLogger&&) = default;
  DataLogger& operator=(DataLogger&&) = default;
  
  // Logging interface
  void log_control_data(
      double timestamp,
      const std::vector<double>& current_positions,
      const std::vector<double>& target_positions,
      const std::vector<double>& commands);
  
  void log_hand_data(
      double timestamp,
      const std::vector<double>& hand_positions);
  
  void flush();
  bool is_open() const;

private:
  std::ofstream file_;
  size_t log_count_{0};
  
  void write_header();
};

}  // namespace ur5e_controller
