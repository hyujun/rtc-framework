// ── closure_state_publisher 실행 진입점 ─────────────────────────────────────────
#include "rtc_urdf_bridge/closure_state_publisher.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rtc_urdf_bridge::ClosureStatePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
