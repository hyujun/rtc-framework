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
#include <thread>

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

  // ── Lane split (issue #345) ────────────────────────────────────────────────
  // Two SingleThreadedExecutors over one node:
  //
  //   main thread   default callback group — joint/calib command subs, calib
  //                 status timer, lifecycle services, and (Phase 3) the NRT
  //                 publish consumer. Stays on the hand_driver core.
  //   hand_aux_io   cb_group_aux_ only — the blocking file writes (timing CSV
  //                 drain, stats JSON). Moves to the aux slot.
  //
  // Deliberately NOT the controller_manager bootstrap shape
  // (rt_controller_main_impl.cpp), which spins a *temporary* lifecycle executor
  // until Active and then hands off: there on_configure runs on a thread that is
  // subsequently joined and destroyed, so anything pinned or bound to it would
  // be pinning a corpse. Here the main thread spins the node from start to
  // finish, which is what lets the Phase 4 self-pin land on the executor thread.
  //
  // Order matters: the aux group is claimed before add_node so that adding the
  // node cannot sweep it into the main executor.
  rclcpp::executors::SingleThreadedExecutor aux_executor;
  aux_executor.add_callback_group(node->GetAuxCallbackGroup(), node->get_node_base_interface());

  rclcpp::executors::SingleThreadedExecutor main_executor;
  main_executor.add_node(node->get_node_base_interface());

  rtc::ThreadConfig aux_cfg = udp_hand_driver::kHandAuxIoConfig;
  aux_cfg.cpu_core = node->AuxCpuSlot();
  std::thread aux_thread([&aux_executor, aux_cfg]() {
    // Applied on the aux thread itself: affinity, policy and name all act on
    // pthread_self(). Verbose because a failure here is not fatal — the lane
    // still runs, just on the inherited core — and must not be silent.
    (void)rtc::ApplyThreadConfigVerbose(aux_cfg);
    aux_executor.spin();
  });

  // Launch event handler triggers configure/activate; both run on this thread.
  main_executor.spin();

  // Explicit cancel before join: spin() returns on context shutdown, but an
  // executor blocked in its wait set does not always observe that promptly on
  // Jazzy + FastDDS.
  aux_executor.cancel();
  aux_thread.join();
  aux_executor.remove_callback_group(node->GetAuxCallbackGroup());
  main_executor.remove_node(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
