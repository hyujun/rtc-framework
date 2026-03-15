// ── Includes: project header first, then system, then C++ stdlib ─────────────
#include "ur5e_rt_controller/rt_controller_node.hpp"

#include "ur5e_rt_base/threading/thread_config.hpp"
#include "ur5e_rt_base/threading/thread_utils.hpp"

#include <sys/mman.h>  // mlockall

#include <fstream>
#include <string>
#include <thread>

namespace urtc = ur5e_rt_controller;

// ── Entry point ───────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  // Fix 7: mlockall BEFORE rclcpp::init.
  // MCL_CURRENT locks pages already mapped; MCL_FUTURE ensures every page
  // allocated afterwards (including DDS/RMW heaps) is also locked.
  // Calling mlockall after rclcpp::init leaves the DDS stack unprotected.
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

  auto node = std::make_shared<RtControllerNode>();

  // Create executors for each callback group
  rclcpp::executors::SingleThreadedExecutor rt_executor;
  rclcpp::executors::SingleThreadedExecutor sensor_executor;
  rclcpp::executors::SingleThreadedExecutor log_executor;
  rclcpp::executors::SingleThreadedExecutor aux_executor;

  // Add callback groups to respective executors
  rt_executor.add_callback_group(node->GetRtGroup(),
                                 node->get_node_base_interface());
  sensor_executor.add_callback_group(node->GetSensorGroup(),
                                     node->get_node_base_interface());
  log_executor.add_callback_group(node->GetLogGroup(),
                                  node->get_node_base_interface());
  aux_executor.add_callback_group(node->GetAuxGroup(),
                                  node->get_node_base_interface());

  // Helper lambda to create thread with RT config
  auto make_thread = [](auto & executor, const urtc::ThreadConfig & cfg) {
      return std::thread([&executor, cfg]() {
                 if (!urtc::ApplyThreadConfig(cfg)) {
                   fprintf(stderr,
                "[WARN] Thread config failed for '%s' (need realtime "
                "permissions)\n",
                cfg.name);
                 } else {
                   fprintf(stdout, "[INFO] Thread '%s' configured:\n%s", cfg.name,
                urtc::VerifyThreadConfig().c_str());
                 }
                 executor.spin();
      });
    };

  // Fix 9: select 6-core or 4-core thread configs at runtime based on the
  // number of online CPUs detected via sysconf(_SC_NPROCESSORS_ONLN).
  const auto cfgs = urtc::SelectThreadConfigs();

  auto t_rt = make_thread(rt_executor, cfgs.rt_control);
  auto t_sensor = make_thread(sensor_executor, cfgs.sensor);
  auto t_log = make_thread(log_executor, cfgs.logging);
  auto t_aux = make_thread(aux_executor, cfgs.aux);

  t_rt.join();
  t_sensor.join();
  t_log.join();
  t_aux.join();

  rclcpp::shutdown();
  return 0;
}
