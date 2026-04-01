#include "ur5e_bt_coordinator/bt_coordinator_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Enable auto-declare so that YAML parameters (hand_pose.*, arm_pose.*, bb.*)
  // are available without explicit declaration.
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rtc_bt::BtCoordinatorNode>(options);
  node->Initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
