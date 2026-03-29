// ── Reusable entry-point logic ────────────────────────────────────────────────
//
// Extracted from rt_controller_main.cpp so that every robot-specific bringup
// package can reuse it without duplicating 100+ lines of boilerplate.
//
// Threading model (v5.16.0):
//   rt_loop          Core 2  SCHED_FIFO 90   clock_nanosleep 500Hz + 50Hz E-STOP
//   publish_thread   Core 5  SCHED_OTHER -3   SPSC drain → all publish() calls
//   sensor_executor  Core 3  SCHED_FIFO 70   /joint_states, /target, /hand subs
//   log_executor     Core 4  SCHED_OTHER -5   SpscLogBuffer → CSV drain
//   aux_executor     Core 5  SCHED_OTHER  0   E-STOP status publisher

#include "rtc_controller_manager/rt_controller_main.hpp"
#include "rtc_controller_manager/rt_controller_node.hpp"

#include "rtc_base/threading/thread_config.hpp"
#include "rtc_base/threading/thread_utils.hpp"

#include <sys/mman.h>  // mlockall

#include <fstream>
#include <string>
#include <thread>

namespace rtc
{

int RtControllerMain(int argc, char ** argv)
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

  // Status Monitor requires shared_from_this(), so it must be initialized
  // after make_shared completes (not inside the constructor).
  node->InitStatusMonitor();

  // Select thread configs based on physical core count
  const auto cfgs = SelectThreadConfigs();

  // ── RT loop + Publish offload (jthreads, managed by node) ───────────────
  node->StartRtLoop(cfgs.rt_control);
  node->StartPublishLoop(cfgs.publish);

  // ── ROS2 executors (sensor, log, aux) ──────────────────────────────────
  rclcpp::executors::SingleThreadedExecutor sensor_executor;
  rclcpp::executors::SingleThreadedExecutor log_executor;
  rclcpp::executors::SingleThreadedExecutor aux_executor;

  sensor_executor.add_callback_group(node->GetSensorGroup(),
                                     node->get_node_base_interface());
  log_executor.add_callback_group(node->GetLogGroup(),
                                  node->get_node_base_interface());
  aux_executor.add_callback_group(node->GetAuxGroup(),
                                  node->get_node_base_interface());

  // Helper lambda to create executor thread with RT config
  auto make_thread = [](auto & executor, const ThreadConfig & cfg) {
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

  // Graceful shutdown of RT loop + publish thread
  node->StopRtLoop();
  node->StopPublishLoop();

  rclcpp::shutdown();
  return 0;
}

}  // namespace rtc
