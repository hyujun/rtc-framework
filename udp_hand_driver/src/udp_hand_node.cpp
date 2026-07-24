#include "udp_hand_driver/udp_hand_node.hpp"

#include "rtc_base/threading/thread_config.hpp"  // SelectThreadConfigs
#include "rtc_base/threading/thread_utils.hpp"   // SelectThreadConfigs, SlotToLogicalCpu
#include "udp_hand_driver/udp_hand_logging.hpp"

#include <rclcpp/rclcpp.hpp>

#include <pthread.h>   // pthread_setaffinity_np
#include <sys/mman.h>  // mlockall

#include <cerrno>
#include <cstddef>
#include <cstring>
#include <memory>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  const auto logger = ::udp_hand_driver::logging::NodeLogger();

  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    RCLCPP_WARN(logger, "mlockall failed (errno=%d: %s)", errno, strerror(errno));
  }

  // Pin the main thread (ROS2 executor + DDS) to the tier-aware hand_driver
  // core so the worker threads spawned later in on_activate — the CommLoop RT
  // thread (hand_udp_recv, SCHED_FIFO 65) and the failure detector — INHERIT
  // that affinity. Both self-set cpu_core=-1 and rely on inheriting the hand
  // process affinity (thread_config.hpp: "hand UDP recv inherits affinity from
  // the launch-level taskset, NOT represented in SystemThreadConfigs"). Pinning
  // here rather than the old hardcoded {0,1} makes that inheritance land on the
  // planned core (12-core NUC13: hand slot 7 -> logical 11) even before the
  // launch taskset fires — and correctly even when use_cpu_affinity:=false, so
  // the RT comm loop is never left on the shared OS/DDS cores. The launch-level
  // `taskset -a` remains as belt-and-suspenders (it also sweeps the DDS threads
  // rclcpp spawned during init(), before this pin). cpu_core is a *slot index*;
  // SlotToLogicalCpu translates it to a logical CPU (identity when sysfs is
  // unavailable, e.g. a container) — the same translation ApplyThreadConfig and
  // the launch pinning use (issue #163). Affinity only: the thread name stays
  // "udp_hand_node" so the launch pgrep -nx match is unaffected.
  {
    const int hand_slot = rtc::SelectThreadConfigs().hand_driver.cpu_core;
    const int logical_cpu = rtc::SlotToLogicalCpu(hand_slot);
    if (logical_cpu < 0) {
      RCLCPP_WARN(logger, "Main thread CPU affinity skipped (hand slot %d unresolved)", hand_slot);
    } else {
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(static_cast<std::size_t>(logical_cpu), &cpuset);
      if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
        RCLCPP_WARN(logger,
                    "Main thread CPU affinity to hand_driver core "
                    "(slot %d -> logical %d) failed (errno=%d)",
                    hand_slot, logical_cpu, errno);
      }
    }
  }

  auto node = std::make_shared<UdpHandNode>();
  // Constructor is empty — launch event handler triggers configure/activate.
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
