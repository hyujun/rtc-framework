// ── Reusable entry-point logic
// ────────────────────────────────────────────────
//
// Library-only API. rtc_controller_manager is robot-agnostic and exports no
// executable. Robot-specific bringup packages (e.g. ur5e_bringup) supply
// their own main() that calls rtc::RtControllerMain(argc, argv, node_name).
// See agent_docs/design-principles.md for the agnostic-vs-specific rule.
//
// 3-phase lifecycle executor pattern:
//   Phase 1: lifecycle_executor spins to process configure/activate service
//            calls from launch event handlers.
//   Phase 2: Poll until the node reaches Active state (on_activate starts
//            RT loop + publish offload thread).
//   Phase 3: Add sensor/log/aux callback groups to dedicated executors
//            with RT thread configs. The default callback group stays on
//            aux_executor to continue processing lifecycle services.
//
// Threading model:
//   rt_loop          Core 2  SCHED_FIFO 90   clock_nanosleep 500Hz + 50Hz
//   E-STOP publish_thread   Core 5  SCHED_OTHER -3   SPSC drain → all publish()
//   calls sensor_executor  Core 3  SCHED_FIFO 70   /joint_states, /target,
//   /hand subs log_executor     Core 4  SCHED_OTHER -5   SpscLogBuffer → CSV
//   drain aux_executor     Core 5  SCHED_OTHER  0   E-STOP status + lifecycle
//   services

#include "rtc_controller_manager/rt_controller_main.hpp"
#include "rtc_controller_manager/rt_controller_node.hpp"

#include "rtc_base/threading/thread_config.hpp"
#include "rtc_base/threading/thread_utils.hpp"

#include <lifecycle_msgs/msg/state.hpp>

#include <sys/mman.h> // mlockall

#include <chrono>
#include <fstream>
#include <string>
#include <thread>

namespace rtc {

int RtControllerMain(int argc, char **argv, const std::string &node_name) {
  // mlockall BEFORE rclcpp::init.
  // MCL_CURRENT locks pages already mapped; MCL_FUTURE ensures every page
  // allocated afterwards (including DDS/RMW heaps) is also locked.
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    fprintf(stderr, "[WARN] mlockall failed — page faults possible\n");
    fprintf(stderr, "       Check: /etc/security/limits.conf @realtime memlock "
                    "unlimited\n");
  }

  rclcpp::init(argc, argv);

  // Check CPU isolation status — warn if RT cores are not isolated
  {
    std::ifstream isolated_file("/sys/devices/system/cpu/isolated");
    std::string isolated;
    if (isolated_file.is_open()) {
      std::getline(isolated_file, isolated);
    }
    if (isolated.empty()) {
      fprintf(stderr,
              "[WARN] No CPU isolation detected (RT cores may receive OS "
              "interrupts)\n");
      fprintf(stderr,
              "       Run: sudo cpu_shield.sh on --robot  (or launch via "
              "ur_control.launch.py)\n");
    } else {
      fprintf(stdout, "[INFO] CPU isolation active: Core %s\n",
              isolated.c_str());
    }
  }

  auto node = std::make_shared<RtControllerNode>(node_name);

  // ═══ Phase 1: lifecycle executor ═══════════════════════════════════════════
  // Spin the node so it can receive lifecycle service calls (configure /
  // activate) from launch event handlers.
  rclcpp::executors::SingleThreadedExecutor lifecycle_executor;
  lifecycle_executor.add_node(node->get_node_base_interface());

  std::thread lifecycle_thread(
      [&lifecycle_executor]() { lifecycle_executor.spin(); });

  // ═══ Phase 2: wait for Active state ═══════════════════════════════════════
  // on_configure creates callback groups, publishers, subscribers.
  // on_activate starts the RT loop + publish offload thread.
  using namespace std::chrono_literals;
  while (rclcpp::ok()) {
    const auto state_id = node->get_current_state().id();
    if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      break;
    }
    if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
      fprintf(stderr, "[ERROR] Node reached Finalized before Active\n");
      lifecycle_executor.cancel();
      lifecycle_thread.join();
      rclcpp::shutdown();
      return 1;
    }
    std::this_thread::sleep_for(50ms);
  }

  if (!rclcpp::ok()) {
    lifecycle_executor.cancel();
    lifecycle_thread.join();
    return 1;
  }

  fprintf(stdout, "[INFO] RtControllerNode reached Active state\n");

  // ═══ Phase 3: switch to dedicated executors ═══════════════════════════════
  // Remove node from lifecycle executor, then add callback groups to
  // dedicated executors for sensor/log/aux threads.
  lifecycle_executor.cancel();
  lifecycle_thread.join();
  lifecycle_executor.remove_node(node->get_node_base_interface());

  const auto cfgs = SelectThreadConfigs();

  rclcpp::executors::SingleThreadedExecutor sensor_executor;
  rclcpp::executors::SingleThreadedExecutor log_executor;
  rclcpp::executors::SingleThreadedExecutor aux_executor;

  sensor_executor.add_callback_group(node->GetSensorGroup(),
                                     node->get_node_base_interface());
  log_executor.add_callback_group(node->GetLogGroup(),
                                  node->get_node_base_interface());
  aux_executor.add_callback_group(node->GetAuxGroup(),
                                  node->get_node_base_interface());

  // Keep default callback group on aux_executor so lifecycle services
  // (deactivate, cleanup, shutdown) continue to be processed at runtime.
  aux_executor.add_node(node->get_node_base_interface());

  // Attach each controller's LifecycleNode to aux_executor so controller-
  // owned subscriptions/publishers are processed off the RT path.  Created
  // during CM on_configure; stable for the lifetime of the CM node.
  for (const auto &ctrl_node : node->GetControllerNodes()) {
    if (ctrl_node) {
      aux_executor.add_node(ctrl_node->get_node_base_interface());
    }
  }

  // Helper lambda to create executor thread with RT config
  auto make_thread = [](auto &executor, const ThreadConfig &cfg) {
    return std::thread([&executor, cfg]() {
      if (!ApplyThreadConfig(cfg)) {
        fprintf(stderr,
                "[WARN] Thread config failed for '%s' (need realtime "
                "permissions)\n",
                cfg.name);
      } else {
        fprintf(stdout, "[INFO] Thread '%s' configured:\n%s", cfg.name,
                VerifyThreadConfig().c_str());
      }
      executor.spin();
    });
  };

  auto t_sensor = make_thread(sensor_executor, cfgs.sensor);
  auto t_log = make_thread(log_executor, cfgs.logging);
  auto t_aux = make_thread(aux_executor, cfgs.aux);

  t_sensor.join();
  t_log.join();
  t_aux.join();

  // Graceful shutdown — lifecycle deactivate/cleanup handled via
  // ros2 lifecycle CLI or launch shutdown event handlers.
  // Destructor provides safety-net Stop calls.

  rclcpp::shutdown();
  return 0;
}

} // namespace rtc
