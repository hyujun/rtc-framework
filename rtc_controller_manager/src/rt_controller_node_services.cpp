// ── /rtc_cm/* services (Phase 3) ────────────────────────────────────────
//
// list_controllers — read-only snapshot of controllers_ + controller_states_.
//   Aux thread; no blocking on RT path. Response order matches controllers_.
//
// switch_controller — single-active D-A1 wrapper around
//   SwitchActiveController(name, message). STRICT validates inputs and
//   refuses violations; BEST_EFFORT trims to the first activate target and
//   warns on extras. Timeout field is accepted but currently unused — the
//   underlying switch helper is sync and bounded by sleep_for(1.5×dt) +
//   controller hooks (~ms). M-1 will measure actual latency in Phase 4.
#include "rtc_controller_manager/rt_controller_node.hpp"

namespace urtc = rtc;

void RtControllerNode::CreateServices() {
  const rclcpp::QoS srv_qos = rclcpp::ServicesQoS();

  list_controllers_srv_ = create_service<rtc_msgs::srv::ListControllers>(
      "/rtc_cm/list_controllers",
      [this](const std::shared_ptr<rtc_msgs::srv::ListControllers::Request> /*req*/,
             std::shared_ptr<rtc_msgs::srv::ListControllers::Response> resp) {
        const std::size_t n = controllers_.size();
        resp->controllers.clear();
        resp->controllers.reserve(n);
        const int active_idx = active_controller_idx_.load(std::memory_order_acquire);
        for (std::size_t i = 0; i < n; ++i) {
          rtc_msgs::msg::ControllerState cs;
          cs.name = std::string(controllers_[i]->Name());
          const int s = controller_states_[i].load(std::memory_order_acquire);
          cs.state = (s == 1) ? "active" : "inactive";
          cs.is_active = (static_cast<int>(i) == active_idx) && (s == 1);
          cs.type = (i < controller_types_.size()) ? controller_types_[i] : "";
          if (i < controller_topic_configs_.size()) {
            cs.claimed_groups.reserve(controller_topic_configs_[i].groups.size());
            for (const auto& [group_name, _g] : controller_topic_configs_[i].groups) {
              cs.claimed_groups.push_back(group_name);
            }
          }
          resp->controllers.push_back(std::move(cs));
        }
      },
      srv_qos, cb_group_aux_);

  switch_controller_srv_ = create_service<rtc_msgs::srv::SwitchController>(
      "/rtc_cm/switch_controller",
      [this](const std::shared_ptr<rtc_msgs::srv::SwitchController::Request> req,
             std::shared_ptr<rtc_msgs::srv::SwitchController::Response> resp) {
        const auto strict = req->strictness;
        const bool best_effort = strict == rtc_msgs::srv::SwitchController::Request::BEST_EFFORT;

        // Single-active validation (D-A1). STRICT: any multi-target → reject.
        // BEST_EFFORT: trim to first entry, warn on extras.
        if (req->activate_controllers.size() > 1) {
          if (!best_effort) {
            resp->ok = false;
            resp->message =
                "Single-active: activate_controllers must have ≤ 1 "
                "entry under STRICT";
            return;
          }
          RCLCPP_WARN(get_logger(),
                      "switch_controller BEST_EFFORT: trimming "
                      "activate_controllers (%zu → 1)",
                      req->activate_controllers.size());
        }
        if (req->deactivate_controllers.size() > 1) {
          if (!best_effort) {
            resp->ok = false;
            resp->message =
                "Single-active: deactivate_controllers must have ≤ "
                "1 entry under STRICT";
            return;
          }
          RCLCPP_WARN(get_logger(),
                      "switch_controller BEST_EFFORT: trimming "
                      "deactivate_controllers (%zu → 1)",
                      req->deactivate_controllers.size());
        }

        // No activate target → either pure deactivate (unsupported in
        // single-active) or empty no-op. Empty is a no-op success; pure
        // deactivate is rejected because RT loop always needs an active idx.
        if (req->activate_controllers.empty()) {
          if (!req->deactivate_controllers.empty()) {
            resp->ok = false;
            resp->message =
                "Pure deactivate not supported under single-active (D-A1) — "
                "specify the replacement in activate_controllers";
            return;
          }
          resp->ok = true;
          resp->message = "no-op (empty request)";
          return;
        }

        const std::string& target = req->activate_controllers.front();
        std::string sub_msg;
        const bool ok = SwitchActiveController(target, sub_msg);
        resp->ok = ok;
        if (ok) {
          resp->message = "switched -> " + target + (sub_msg.empty() ? "" : " (" + sub_msg + ")");
        } else {
          resp->message = sub_msg;
        }
      },
      srv_qos, cb_group_aux_);

  RCLCPP_INFO(get_logger(),
              "Services ready: /rtc_cm/list_controllers, "
              "/rtc_cm/switch_controller");
}
