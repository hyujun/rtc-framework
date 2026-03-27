#include "ur5e_bt_coordinator/action_nodes/compute_sweep_trajectory.hpp"

#include <cmath>

namespace rtc_bt {

BT::PortsList ComputeSweepTrajectory::providedPorts()
{
  return {
    BT::InputPort<Pose6D>("start_pose"),
    BT::InputPort<double>("direction_x", 1.0, "Sweep direction X"),
    BT::InputPort<double>("direction_y", 0.0, "Sweep direction Y"),
    BT::InputPort<double>("distance", 0.3, "Sweep distance [m]"),
    BT::InputPort<double>("arc_height", 0.05, "Arc peak height [m]"),
    BT::InputPort<int>("num_waypoints", 8, "Number of waypoints"),
    BT::OutputPort<std::vector<Pose6D>>("waypoints"),
  };
}

BT::NodeStatus ComputeSweepTrajectory::tick()
{
  auto start = getInput<Pose6D>("start_pose");
  if (!start) {
    throw BT::RuntimeError("ComputeSweepTrajectory: missing start_pose");
  }

  double dir_x = getInput<double>("direction_x").value_or(1.0);
  double dir_y = getInput<double>("direction_y").value_or(0.0);
  double distance = getInput<double>("distance").value_or(0.3);
  double arc_height = getInput<double>("arc_height").value_or(0.05);
  int n = getInput<int>("num_waypoints").value_or(8);
  if (n < 2) n = 2;

  // Normalize direction
  double len = std::sqrt(dir_x * dir_x + dir_y * dir_y);
  if (len > 1e-6) { dir_x /= len; dir_y /= len; }

  const Pose6D& s = start.value();
  std::vector<Pose6D> waypoints;
  waypoints.reserve(static_cast<std::size_t>(n));

  for (int i = 0; i < n; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(n - 1);

    Pose6D wp = s;
    wp.x += dir_x * distance * t;
    wp.y += dir_y * distance * t;
    // Arc profile: sinusoidal rise and fall
    wp.z += arc_height * std::sin(M_PI * t);
    // Orientation preserved from start pose

    waypoints.push_back(wp);
  }

  setOutput("waypoints", waypoints);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rtc_bt
