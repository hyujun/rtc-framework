// p_controller.hpp - v2
#pragma once

#include "ur5e_rt_controller/rt_controller_interface.hpp"
#include <vector>
#include <array>

namespace ur5e_controller {

class PController : public rt_controller_interface {
public:
  explicit PController(double kp);
  
  std::vector<double> compute_command(
      const std::vector<double>& current_positions,
      const std::vector<double>& current_velocities,
      const std::vector<double>& target_positions,
      double dt) override;
  
  void reset() override;
  bool is_initialized() const override { return initialized_; }
  
  // Gain adjustment
  void set_kp(double kp) { kp_ = kp; }
  double get_kp() const { return kp_; }

private:
  double kp_;
  bool initialized_{false};
  
  std::array<double, NUM_JOINTS> clamp_command(
      const std::array<double, NUM_JOINTS>& command) const;
};

}  // namespace ur5e_controller
