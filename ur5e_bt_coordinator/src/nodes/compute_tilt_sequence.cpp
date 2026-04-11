#include "ur5e_bt_coordinator/action_nodes/compute_tilt_sequence.hpp"
#include "ur5e_bt_coordinator/bt_logging.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>

namespace rtc_bt {

namespace {
auto logger() { return ::rtc_bt::logging::ActionLogger("compute_tilt_sequence"); }
constexpr double kDegToRad = M_PI / 180.0;
}  // namespace

BT::PortsList ComputeTiltSequence::providedPorts()
{
  return {
    BT::InputPort<Pose6D>("base_pose"),
    BT::InputPort<double>("amplitude_deg", 15.0, "Tilt amplitude [deg]"),
    BT::InputPort<int>("num_steps", 6, "Number of tilt waypoints"),
    BT::OutputPort<std::vector<Pose6D>>("waypoints"),
  };
}

BT::NodeStatus ComputeTiltSequence::tick()
{
  auto base = getInput<Pose6D>("base_pose");
  if (!base) {
    RCLCPP_ERROR(logger(), "missing base_pose port");
    throw BT::RuntimeError("ComputeTiltSequence: missing base_pose");
  }

  double amplitude_deg = getInput<double>("amplitude_deg").value_or(15.0);
  int n = getInput<int>("num_steps").value_or(6);
  if (n < 2) n = 2;

  double amplitude_rad = amplitude_deg * kDegToRad;
  const Pose6D& b = base.value();

  std::vector<Pose6D> waypoints;
  waypoints.reserve(static_cast<std::size_t>(n + 1));

  // Generate sinusoidal tilt pattern alternating pitch and roll
  for (int i = 0; i < n; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(n);
    double angle = amplitude_rad * std::sin(2.0 * M_PI * t);

    Pose6D wp = b;
    if (i % 2 == 0) {
      // Even steps: pitch variation
      wp.pitch = b.pitch + angle;
    } else {
      // Odd steps: roll variation
      wp.roll = b.roll + angle;
    }
    waypoints.push_back(wp);
  }

  // Final waypoint: return to base orientation
  waypoints.push_back(b);

  RCLCPP_INFO(logger(),
              "%zu waypoints, amplitude=%.1f deg",
              waypoints.size(), amplitude_deg);

  setOutput("waypoints", waypoints);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rtc_bt
