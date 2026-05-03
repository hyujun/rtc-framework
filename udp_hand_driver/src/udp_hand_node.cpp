#include "udp_hand_driver/udp_hand_node.hpp"

#include "udp_hand_driver/udp_hand_logging.hpp"

#include <rclcpp/rclcpp.hpp>

#include <pthread.h>   // pthread_setaffinity_np
#include <sys/mman.h>  // mlockall

#include <cerrno>
#include <cstring>
#include <memory>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  const auto logger = ::udp_hand_driver::logging::NodeLogger();

  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    RCLCPP_WARN(logger, "mlockall failed (errno=%d: %s)", errno, strerror(errno));
  }

  // Pin main thread (ROS2 executor, DDS) to Core 0-1 (OS/DDS cores).
  {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    CPU_SET(1, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
      RCLCPP_WARN(logger, "Main thread CPU affinity failed (errno=%d)", errno);
    }
  }

  auto node = std::make_shared<UdpHandNode>();
  // Constructor is empty — launch event handler triggers configure/activate.
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
