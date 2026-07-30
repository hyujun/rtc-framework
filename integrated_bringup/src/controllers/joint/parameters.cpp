#include "integrated_bringup/controllers/demo_joint_controller.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace integrated_bringup {

// ── Phase D: gain → ROS 2 parameter declaration & callback ────────────────
//
// Mirrors the DemoTaskController pattern (Phase B) for the joint-space demo.
// Tunable: robot_trajectory_speed, hand_trajectory_speed,
//          grasp_{contact,force}_threshold, grasp_min_fingertips.
// Read-only (D-2): robot_max_traj_velocity, hand_max_traj_velocity.
void DemoJointController::DeclareGainParameters() noexcept {
  if (!node_) {
    return;
  }

  auto g = gains_lock_.Load();

  auto declare_double = [&](const std::string& name, double default_val,
                            const std::string& description, bool read_only = false) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = description;
    d.read_only = read_only;
    if (!node_->has_parameter(name)) {
      return node_->declare_parameter<double>(name, default_val, d);
    }
    return node_->get_parameter(name).as_double();
  };
  auto declare_int = [&](const std::string& name, int64_t default_val,
                         const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = description;
    if (!node_->has_parameter(name)) {
      return node_->declare_parameter<int64_t>(name, default_val, d);
    }
    return node_->get_parameter(name).as_int();
  };
  auto declare_string = [&](const std::string& name, const std::string& default_val,
                            const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = description;
    if (!node_->has_parameter(name)) {
      return node_->declare_parameter<std::string>(name, default_val, d);
    }
    return node_->get_parameter(name).as_string();
  };

  g.robot_trajectory_speed =
      std::max(1e-6, declare_double("robot_trajectory_speed", g.robot_trajectory_speed,
                                    "Arm joint trajectory speed [rad/s]"));
  g.hand_trajectory_speed =
      std::max(1e-6, declare_double("hand_trajectory_speed", g.hand_trajectory_speed,
                                    "Hand motor trajectory speed [rad/s]"));

  g.robot_max_traj_velocity =
      declare_double("robot_max_traj_velocity", g.robot_max_traj_velocity,
                     "Max arm joint velocity during trajectory [rad/s] (read-only)",
                     /*read_only=*/true);
  g.hand_max_traj_velocity =
      declare_double("hand_max_traj_velocity", g.hand_max_traj_velocity,
                     "Max hand motor velocity during trajectory [rad/s] (read-only)",
                     /*read_only=*/true);

  g.grasp_contact_threshold = static_cast<float>(declare_double(
      "grasp_contact_threshold", g.grasp_contact_threshold, "Contact probability threshold [0,1]"));
  g.grasp_force_threshold = static_cast<float>(declare_double(
      "grasp_force_threshold", g.grasp_force_threshold, "Force magnitude threshold [N]"));
  g.grasp_min_fingertips =
      static_cast<int>(declare_int("grasp_min_fingertips", g.grasp_min_fingertips,
                                   "Min fingertips with contact for grasp detection"));

  // WRITABLE as of B-4b (was read-only, display-only). Seeded from the mode
  // LoadConfig resolved out of demo_shared.yaml.
  //
  // Where a value other than the YAML's can come from — NOT a launch override, as
  // this comment used to claim: per-controller child nodes are built with
  // use_global_arguments(false) and no parameter_overrides
  // (rt_controller_node_params.cpp), so launch-file and CLI parameter values never
  // reach this node. The reachable source is `ros2 param set` issued while the
  // controller is CLEANED UP: on_cleanup drops the on-set callback but leaves the
  // parameter declared, and declare_string below then returns that stored value
  // instead of the seed.
  //
  // The parse is wrapped because THIS FUNCTION IS noexcept: an off-whitelist value
  // would otherwise leave ParseGraspHandMode throwing out of a noexcept frame,
  // i.e. std::terminate. It also cannot fail the lifecycle transition (void
  // return) — the second thing this comment used to claim — so the honest fallback
  // is to keep the value demo_shared.yaml already validated and say so loudly. A
  // bad value in the YAML itself still fails on_configure, as before.
  const GraspHandMode configured_mode = g.grasp_hand_mode;
  const std::string requested_mode =
      declare_string("grasp_controller_type", GraspHandModeName(g.grasp_hand_mode),
                     "Active hand grasp mode: 'contact_stop' | 'force_pi' | 'none'. "
                     "Runtime-settable while the hand is quiet (no contact_stop latch, "
                     "no force_pi grasp in progress); otherwise the set is refused "
                     "with a reason.");
  try {
    const GraspHandMode requested = ParseGraspHandMode(requested_mode);
    // The whitelist is not the whole gate. declare_string returns the EXISTING
    // parameter value when the parameter is already declared, and on_cleanup drops
    // the on-set callback while leaving the parameter itself in place — so a
    // `ros2 param set … force_pi` issued between cleanup and configure arrives
    // here having passed no capability check at all, and iiwa7_leap (no
    // force_pi_grasp block) would come up claiming a mode it cannot run.
    // Same SSoT as the runtime path rather than a second opinion. The two quiet
    // inputs are facts here, not mirrors: LoadConfig ran earlier in this bring-up
    // and cleared the latch, and BuildGraspController either just made a fresh
    // (idle) controller or left nullptr.
    if (const char* why =
            GraspModeChangeRejectReason(configured_mode, requested,
                                        {.has_controller = grasp_controller_ != nullptr,
                                         .contact_latched = false,
                                         .grasp_phase_idle = true});
        why != nullptr) {
      RCLCPP_ERROR(logger_, "grasp_controller_type '%s' rejected at configure (%s); keeping '%s'",
                   requested_mode.c_str(), why, GraspHandModeName(configured_mode));
    } else {
      g.grasp_hand_mode = requested;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "grasp_controller_type override rejected (%s); keeping '%s'", e.what(),
                 GraspHandModeName(g.grasp_hand_mode));
  }

  gains_lock_.Store(g);
}

rcl_interfaces::msg::SetParametersResult DemoJointController::OnGainParametersSet(
    const std::vector<rclcpp::Parameter>& params) noexcept {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  auto g = gains_lock_.Load();
  bool gains_dirty = false;

  for (const auto& p : params) {
    const auto& name = p.get_name();
    try {
      if (name == "robot_trajectory_speed") {
        g.robot_trajectory_speed = std::max(1e-6, p.as_double());
        gains_dirty = true;
      } else if (name == "hand_trajectory_speed") {
        g.hand_trajectory_speed = std::max(1e-6, p.as_double());
        gains_dirty = true;
      } else if (name == "grasp_contact_threshold") {
        g.grasp_contact_threshold = static_cast<float>(p.as_double());
        gains_dirty = true;
      } else if (name == "grasp_force_threshold") {
        g.grasp_force_threshold = static_cast<float>(p.as_double());
        gains_dirty = true;
      } else if (name == "grasp_min_fingertips") {
        g.grasp_min_fingertips = static_cast<int>(p.as_int());
        gains_dirty = true;
      } else if (name == "grasp_controller_type") {
        // Whitelist first. p.as_string() throwing is a genuine type error and
        // belongs to the outer catch; an off-whitelist STRING is a different
        // mistake and deserves its own reason rather than a "type error" prefix.
        GraspHandMode requested{};
        try {
          requested = ParseGraspHandMode(p.as_string());
        } catch (const std::runtime_error& e) {
          result.successful = false;
          result.reason = e.what();
          return result;
        }
        // Quiet gate. Both inputs come from RT-written atomics, never from the
        // RT-owned originals — see the mirrors on the controller. The check is
        // inherently a tick behind and cannot stop the RT thread, which is why
        // ComputeControl carries the re-seed race closure for the window between
        // this decision and the Store below.
        if (const char* why = GraspModeChangeRejectReason(
                g.grasp_hand_mode, requested,
                // Designated initialisers, not positional-with-comments: all three
                // fields are bool, so swapping two of them would otherwise compile
                // and silently invert a safety decision. C++20 requires these in
                // declaration order, which turns that mistake into a build error.
                {.has_controller = grasp_controller_ != nullptr,
                 .contact_latched = contact_latched_pub_.load(std::memory_order_acquire),
                 .grasp_phase_idle = grasp_phase_pub_.load(std::memory_order_acquire) ==
                                     static_cast<uint8_t>(rtc::grasp::GraspPhase::kIdle)});
            why != nullptr) {
          result.successful = false;
          result.reason = why;
          return result;
        }
        g.grasp_hand_mode = requested;
        gains_dirty = true;
      }
      // Unknown names: silently allowed (other callbacks may own them).
    } catch (const std::exception& e) {
      result.successful = false;
      result.reason = std::string("type error on '") + name + "': " + e.what();
      return result;
    }
  }

  if (gains_dirty) {
    gains_lock_.Store(g);
  }
  return result;
}

}  // namespace integrated_bringup
