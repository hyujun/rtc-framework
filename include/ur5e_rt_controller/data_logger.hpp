#ifndef UR5E_RT_CONTROLLER_DATA_LOGGER_H_
#define UR5E_RT_CONTROLLER_DATA_LOGGER_H_

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <span>

#include "ur5e_rt_controller/rt_controller_interface.hpp"

namespace ur5e_rt_controller {

// Writes control data to a CSV file in a non-RT (logging) thread context.
// Copy is disabled; move is enabled for deferred construction.
class DataLogger {
 public:
  explicit DataLogger(std::filesystem::path log_path);
  ~DataLogger();

  DataLogger(const DataLogger&)            = delete;
  DataLogger& operator=(const DataLogger&) = delete;
  DataLogger(DataLogger&&)                 = default;
  DataLogger& operator=(DataLogger&&)      = default;

  // Log one control step: timestamp (s), current positions, target positions,
  // and computed commands (all kNumRobotJoints elements).
  void LogControlData(
      double timestamp,
      std::span<const double, kNumRobotJoints> current_positions,
      std::span<const double, kNumRobotJoints> target_positions,
      std::span<const double, kNumRobotJoints> commands);

  // Log hand state for a given timestamp.
  void LogHandData(double timestamp,
                   std::span<const double, kNumHandJoints> hand_positions);

  void Flush();
  [[nodiscard]] bool IsOpen() const;

 private:
  std::ofstream file_;
  std::size_t   log_count_{0};

  void WriteHeader();
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_CONTROLLER_DATA_LOGGER_H_
