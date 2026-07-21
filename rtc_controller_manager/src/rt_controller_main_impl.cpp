// ── Reusable entry-point logic
// ────────────────────────────────────────────────
//
// Library-only API. rtc_controller_manager is robot-agnostic and exports no
// executable. Robot-specific bringup packages (e.g. integrated_bringup) supply
// their own main() that calls rtc::RtControllerMain(argc, argv, node_name).
// See agent_docs/design-principles.md for the agnostic-vs-specific rule.
//
// 3-phase lifecycle executor pattern:
//   Phase 1: lifecycle_executor spins to process configure/activate service
//            calls from launch event handlers.
//   Phase 2: Poll until the node reaches Active state (on_activate starts
//            RT loop + nrt publish offload thread).
//   Phase 3: Add rt_callback/nrt_logging/nrt_callback callback groups to
//            dedicated executors with RT thread configs. The default callback
//            group stays on nrt_callback_executor to continue processing
//            lifecycle services.
//
// Threading model (layout v4.1, SSoT: rtc_base/threading/thread_config.hpp):
//   rt_control       Core 1  SCHED_FIFO 90   clock_nanosleep @ control_rate (default 500Hz) + 50Hz
//                                            timeout checker. Performs DeviceBackend.WriteCommand
//                                            inline (actuator command publish, RT-safe) and pushes
//                                            controller-owned snapshots into nrt_publish_buffer_.
//   rt_callback      Core 2  SCHED_FIFO 70   rt_callback_executor pinned here.
//                                            cb_group_rt_callback_ — DeviceBackend
//                                            state subs only (/joint_states,
//                                            hand state/motor/sensor) via
//                                            DeviceBackend::Configure(node, cfg,
//                                            state_cb_group) injection. RobotTarget
//                                            subs (CM-owned and controller-owned)
//                                            stay on default group (nrt_callback)
//                                            per RT-boundary decision. DDS recv
//                                            thread co-pinned to this core via
//                                            launch-time taskset for cache locality.
//   nrt_publish      nrt_callback core, CFS  nrt_publish_buffer_ drain (cap 16) →
//                                            controller.PublishNonRtSnapshot
//                                            (controller-owned non-RT topics:
//                                            Transforms / grasp_state /
//                                            wbc_state / tof_snapshot).
//   nrt_logging      tier-aware (4c: Core 0; nrt_logging_executor — cm_timing_log.csv
//                    ≥ 6c: dedicated core)   drain + deferred E-STOP log.
//                                            SCHED_OTHER -5.
//   nrt_callback     tier-aware (4c: Core 0; nrt_callback_executor —
//                    ≥ 6c: dedicated core)   cb_group_nrt_callback_ + CM node default
//                                            SCHED_OTHER 0                group
//                                            (CM-owned RobotTarget sub) + every
//                                            controller LifecycleNode default group
//                                            (owned RobotTarget subs, grasp_command
//                                            services). E-STOP status + lifecycle
//                                            services + nrt_publish_thread snapshot
//                                            drain co-located here.

#include "rtc_base/threading/thread_config.hpp"
#include "rtc_base/threading/thread_utils.hpp"
#include "rtc_controller_manager/rt_controller_main.hpp"
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/utilities.hpp>

#include <sys/mman.h>  // mlockall

#include <chrono>
#include <fstream>
#include <string>
#include <thread>

namespace rtc {

int RtControllerMain(int argc, char** argv, const std::string& node_name) {
  // mlockall BEFORE rclcpp::init.
  // MCL_CURRENT locks pages already mapped; MCL_FUTURE ensures every page
  // allocated afterwards (including DDS/RMW heaps) is also locked.
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    fprintf(stderr, "[WARN] mlockall failed — page faults possible\n");
    fprintf(stderr,
            "       Check: /etc/security/limits.conf @realtime memlock "
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
      fprintf(stdout, "[INFO] CPU isolation active: Core %s\n", isolated.c_str());
    }
  }

  auto node = std::make_shared<RtControllerNode>(node_name);

  // ═══ Phase 1: lifecycle executor ═══════════════════════════════════════════
  // Spin the node so it can receive lifecycle service calls (configure /
  // activate) from launch event handlers.
  rclcpp::executors::SingleThreadedExecutor lifecycle_executor;
  lifecycle_executor.add_node(node->get_node_base_interface());

  std::thread lifecycle_thread([&lifecycle_executor]() { lifecycle_executor.spin(); });

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

  rclcpp::executors::SingleThreadedExecutor rt_callback_executor;
  rclcpp::executors::SingleThreadedExecutor nrt_logging_executor;
  rclcpp::executors::SingleThreadedExecutor nrt_callback_executor;

  rt_callback_executor.add_callback_group(node->GetRtCallbackGroup(),
                                          node->get_node_base_interface());
  nrt_logging_executor.add_callback_group(node->GetNrtLoggingGroup(),
                                          node->get_node_base_interface());
  nrt_callback_executor.add_callback_group(node->GetNrtCallbackGroup(),
                                           node->get_node_base_interface());

  // Keep default callback group on nrt_callback_executor so lifecycle services
  // (deactivate, cleanup, shutdown) continue to be processed at runtime.
  nrt_callback_executor.add_node(node->get_node_base_interface());

  // Attach each controller's LifecycleNode to nrt_callback_executor so controller-
  // owned subscriptions/publishers are processed off the RT path.  Created
  // during CM on_configure; stable for the lifetime of the CM node.
  for (const auto& ctrl_node : node->GetControllerNodes()) {
    if (ctrl_node) {
      nrt_callback_executor.add_node(ctrl_node->get_node_base_interface());
    }
  }

  // Helper lambda to create executor thread with RT config
  auto make_thread = [](auto& executor, const ThreadConfig& cfg) {
    return std::thread([&executor, cfg]() {
      static_cast<void>(ApplyThreadConfigVerbose(cfg));
      executor.spin();
    });
  };

  auto t_rt_callback = make_thread(rt_callback_executor, cfgs.rt_callback);
  auto t_nrt_logging = make_thread(nrt_logging_executor, cfgs.nrt_logging);
  auto t_nrt_callback = make_thread(nrt_callback_executor, cfgs.nrt_callback);

  // Block here until rclcpp signals shutdown (SIGINT / SIGTERM via
  // rclcpp's installed signal handler, or explicit `rclcpp::shutdown()`).
  // We cannot rely on the rmw guard condition alone to wake every
  // executor — cores collected during sim shutdown showed t_nrt_callback still in
  // rmw_fastrtps_shared_cpp::__rmw_wait after rclcpp::ok() flipped false,
  // so cancel each executor explicitly here before joining.
  while (rclcpp::ok()) {
    std::this_thread::sleep_for(50ms);
  }
  rt_callback_executor.cancel();
  nrt_logging_executor.cancel();
  nrt_callback_executor.cancel();

  t_rt_callback.join();
  t_nrt_logging.join();
  t_nrt_callback.join();

  // Drop executor↔node references explicitly so the executors' internal
  // weak_ptr bookkeeping releases its node refs before local destruction
  // begins. Without this, controller LifecycleNodes attached via
  // add_node() (per-controller `/<config_key>` namespace) could outlive
  // their expected scope during local-variable teardown.
  for (const auto& ctrl_node : node->GetControllerNodes()) {
    if (ctrl_node) {
      nrt_callback_executor.remove_node(ctrl_node->get_node_base_interface());
    }
  }
  nrt_callback_executor.remove_node(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}

}  // namespace rtc
