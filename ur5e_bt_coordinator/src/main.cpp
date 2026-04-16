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
  // Constructor only declares parameters — launch event handler triggers
  // configure/activate via lifecycle services.
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
