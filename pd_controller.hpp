// pd_controller.hpp - v3 (E-STOP Support)
#pragma once

#include "ur5e_rt_controller/rt_controller_interface.hpp"
#include <vector>
#include <array>
#include <chrono>

namespace ur5e_controller {

class PDController : public rt_controller_interface {
public:
  PDController(double kp, double kd);
  
  std::vector<double> compute_command(
      const std::vector<double>& current_positions,
      const std::vector<double>& current_velocities,
      const std::vector<double>& target_positions,
      double dt) override;
  
  void reset() override;
  bool is_initialized() const override { return initialized_; }
  
  // Gain adjustment
  void set_gains(double kp, double kd) {
    kp_ = kp;
    kd_ = kd;
  }
  
  double get_kp() const { return kp_; }
  double get_kd() const { return kd_; }
  
  // v3: E-STOP override
  void trigger_estop() override;
  void clear_estop() override;

private:
  double kp_;
  double kd_;
  bool initialized_{false};
  
  std::array<double, NUM_JOINTS> previous_positions_{};
  std::array<double, NUM_JOINTS> previous_errors_{};
  
  // v3: Safe position for E-STOP recovery
  std::array<double, NUM_JOINTS> safe_position_{0, -1.57, 1.57, -1.57, -1.57, 0};
  
  std::array<double, NUM_JOINTS> clamp_command(
      const std::array<double, NUM_JOINTS>& command) const;
  
  double compute_derivative(double current_error, double previous_error, double dt) const;
};

}  // namespace ur5e_controller
