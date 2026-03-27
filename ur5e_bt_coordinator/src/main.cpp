#include "ur5e_bt_coordinator/bt_coordinator_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rtc_bt::BtCoordinatorNode>();
  node->Initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
