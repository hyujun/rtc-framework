#ifndef UR5E_RT_BASE_TYPES_HPP_
#define UR5E_RT_BASE_TYPES_HPP_

// Shared compile-time constants and plain-data structures used by all packages
// in the ur5e RT controller stack.
//
// Namespace ur5e_rt_controller is kept consistent across all packages so that
// downstream code (controllers, nodes) needs no namespace changes after the
// multi-package split.

#include <array>
#include <concepts>
#include <cstdint>

namespace ur5e_rt_controller {

// ── Compile-time constants ─────────────────────────────────────────────────────
inline constexpr int kNumRobotJoints = 6;
inline constexpr int kNumHandJoints  = 11;
inline constexpr int kNumHandSensors = 44;  // 4 sensors × 11 joints

// ── C++20 Concepts ─────────────────────────────────────────────────────────────
// Constrains template parameters to floating-point types (double, float, etc.).
// Used for gain parameters and filter coefficients.
template <typename T>
concept FloatingPointType = std::floating_point<T>;

// ── Data structures (aggregate, zero-initialised by default) ──────────────────
struct RobotState {
  std::array<double, kNumRobotJoints> positions{};
  std::array<double, kNumRobotJoints> velocities{};
  std::array<double, 3>               tcp_position{};
  double   dt{0.002};
  uint64_t iteration{0};
};

struct HandState {
  std::array<double, kNumHandJoints>  motor_positions{};
  std::array<double, kNumHandJoints>  motor_velocities{};
  std::array<double, kNumHandJoints>  motor_currents{};
  std::array<double, kNumHandSensors> sensor_data{};
  bool valid{false};
};

struct ControllerState {
  RobotState robot{};
  HandState  hand{};
  // NOTE: dt and iteration duplicate robot.dt and robot.iteration.
  // Both MUST be kept in sync when assembling ControllerState.
  // Controllers may read either field; inconsistency causes subtle bugs.
  double   dt{0.002};
  uint64_t iteration{0};
};

enum class CommandType { kPosition, kTorque };

struct ControllerOutput {
  std::array<double, kNumRobotJoints> robot_commands{};
  std::array<double, kNumHandJoints>  hand_commands{};
  std::array<double, kNumRobotJoints> actual_target_positions{};
  std::array<double, 6>               actual_task_positions{};
  bool        valid{true};
  CommandType command_type{CommandType::kPosition};
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_BASE_TYPES_HPP_
