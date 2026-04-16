#include "shape_estimation/shape_estimation_node.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <rclcpp/rclcpp.hpp>
#pragma GCC diagnostic pop

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<shape_estimation::ShapeEstimationNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
