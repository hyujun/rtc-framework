#ifndef UR5E_RT_CONTROLLER_RT_CONTROLLER_INTERFACE_H_
#define UR5E_RT_CONTROLLER_RT_CONTROLLER_INTERFACE_H_

#include <array>
#include <concepts>
#include <cstdint>
#include <span>
#include <string_view>

namespace ur5e_rt_controller {

// ── Compile-time constants ─────────────────────────────────────────────────────
inline constexpr int kNumRobotJoints = 6;
inline constexpr int kNumHandJoints  = 11;
inline constexpr int kNumHandSensors = 44;  // 4 sensors × 11 joints

// ── C++20 Concepts ─────────────────────────────────────────────────────────────
// Constrains gain parameters to non-negative floating-point types.
template <typename T>
concept NonNegativeFloat = std::floating_point<T>;

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
  double   dt{0.002};
  uint64_t iteration{0};
};

struct ControllerOutput {
  std::array<double, kNumRobotJoints> robot_commands{};
  std::array<double, kNumHandJoints>  hand_commands{};
  bool valid{true};
};

// ── Abstract interface (Strategy Pattern) ─────────────────────────────────────
//
// All virtual methods are noexcept to guarantee real-time safety: any
// exception thrown inside a 500 Hz timer would terminate the process.
class RTControllerInterface {
 public:
  virtual ~RTControllerInterface() = default;

  RTControllerInterface(const RTControllerInterface&)            = delete;
  RTControllerInterface& operator=(const RTControllerInterface&) = delete;
  RTControllerInterface(RTControllerInterface&&)                 = delete;
  RTControllerInterface& operator=(RTControllerInterface&&)      = delete;

  // Compute one control step. Must be noexcept for RT safety.
  [[nodiscard]] virtual ControllerOutput Compute(
      const ControllerState& state) noexcept = 0;

  virtual void SetRobotTarget(
      std::span<const double, kNumRobotJoints> target) noexcept = 0;

  virtual void SetHandTarget(
      std::span<const double, kNumHandJoints> target) noexcept = 0;

  [[nodiscard]] virtual std::string_view Name() const noexcept = 0;

  // E-STOP interface — default no-ops for controllers that do not need it.
  virtual void TriggerEstop() noexcept                      {}
  virtual void ClearEstop() noexcept                        {}
  [[nodiscard]] virtual bool IsEstopped() const noexcept    { return false; }
  virtual void SetHandEstop(bool /*enabled*/) noexcept      {}

 protected:
  RTControllerInterface() = default;
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_CONTROLLER_RT_CONTROLLER_INTERFACE_H_
