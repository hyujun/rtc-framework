#include "ur5e_bt_coordinator/action_nodes/set_gains.hpp"

#include "ur5e_bt_coordinator/bt_logging.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"
#include <rtc_msgs/srv/grasp_command.hpp>

#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <future>
#include <string>
#include <vector>

namespace rtc_bt {

namespace {
auto logger() {
  return ::rtc_bt::logging::ActionLogger("set_gains");
}

// Normalize controller name for comparison: strip underscores and lowercase.
// e.g. "demo_task_controller" and "DemoTaskController" both become
// "demotaskcontroller".
std::string NormalizeName(const std::string& s) {
  std::string r;
  r.reserve(s.size());
  for (char c : s) {
    if (c != '_') {
      r.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
  }
  return r;
}

// Two distinct waits, two budgets — sharing one made the readiness wait fail
// far too eagerly (issue #158).
//
// Before the send, the wait is for the active controller's service to be
// *discovered*: SetGains typically runs right after a SwitchController, and the
// switch srv returning ok does not mean this node's clients are bound yet — the
// bridge still has to receive the latched /rtc_cm/active_controller_name and
// rewire, and the fresh client then has to discover the service. That is
// DDS discovery, which stretches under participant churn.
//
// After the send, the wait is for the controller to *answer* a request it has
// already received. That is a live controller's compute time, not discovery,
// and a slow answer there is a real fault worth failing on quickly.
//
// Neither is unbounded: a service that never appears still fails, just with
// enough grace to cover discovery.
constexpr double kReadyTimeoutS = 5.0;  // stage entry → send (discovery grace)
constexpr double kSrvTimeoutS = 2.0;    // send → response
}  // namespace

SetGains::SetGains(const std::string& name, const BT::NodeConfig& config,
                   std::shared_ptr<BtRosBridge> bridge)
    : BT::StatefulActionNode(name, config), bridge_(std::move(bridge)) {}

BT::PortsList SetGains::providedPorts() {
  return {
      // ── Common ────────────────────────────────────────────────────────────
      BT::InputPort<double>("trajectory_speed",
                            "[m/s] DemoTask: trajectory_speed; "
                            "DemoJoint: robot_trajectory_speed; "
                            "DemoWbc: arm_trajectory_speed"),
      BT::InputPort<double>("trajectory_angular_speed", "[rad/s] DemoTask only"),
      BT::InputPort<double>("hand_trajectory_speed", "[rad/s] All demos"),

      // ── DemoTask CLIK gains ───────────────────────────────────────────────
      BT::InputPort<std::string>("kp_translation", "", "e.g. \"5.0,5.0,5.0\""),
      BT::InputPort<std::string>("kp_rotation", "", "e.g. \"3.0,3.0,3.0\""),
      BT::InputPort<double>("damping", "Damped pseudoinverse lambda"),
      BT::InputPort<double>("null_kp", "Null-space P gain"),
      BT::InputPort<bool>("enable_null_space", "Enable null-space task"),
      BT::InputPort<bool>("control_6dof", "false=3-DOF, true=6-DOF"),

      // ── DemoJoint / DemoTask grasp detection ──────────────────────────────
      BT::InputPort<double>("grasp_contact_threshold", "Contact probability threshold [0,1]"),
      BT::InputPort<double>("grasp_force_threshold", "Force magnitude threshold [N]"),
      BT::InputPort<int>("grasp_min_fingertips", "Min fingertips with contact for grasp"),

      // ── DemoWbc ───────────────────────────────────────────────────────────
      BT::InputPort<double>("se3_weight", "TSID SE3Task weight"),
      BT::InputPort<double>("force_weight", "TSID ForceTask weight"),
      BT::InputPort<double>("posture_weight", "TSID PostureTask weight"),
      BT::InputPort<bool>("mpc_enable", "MPC output gate"),
      BT::InputPort<double>("riccati_gain_scale", "Riccati feedback scale [0,1]"),

      // ── Force-PI grasp (one-shot, separate srv) ───────────────────────────
      BT::InputPort<int>("grasp_command", 0, "0=none, 1=grasp, 2=release (Force-PI)"),
      BT::InputPort<double>("grasp_target_force", 2.0, "Target grip force [N]"),
  };
}

bool SetGains::BuildParams() {
  const auto active = NormalizeName(bridge_->GetActiveController());
  const bool is_joint = (active == "demojointcontroller");
  const bool is_task = (active == "demotaskcontroller");
  const bool is_wbc = (active == "demowbccontroller");
  if (!is_joint && !is_task && !is_wbc) {
    RCLCPP_WARN(logger(), "unknown active controller: '%s' — skipping",
                bridge_->GetActiveController().c_str());
    return false;
  }

  std::vector<rclcpp::Parameter>& params = params_;
  params.clear();
  params.reserve(16);

  // ── Map trajectory_speed input → controller-specific param name ──────────
  if (auto v = getInput<double>("trajectory_speed"); v) {
    if (is_task) {
      params.emplace_back("trajectory_speed", v.value());
    } else if (is_joint) {
      params.emplace_back("robot_trajectory_speed", v.value());
    } else if (is_wbc) {
      params.emplace_back("arm_trajectory_speed", v.value());
    }
  }
  if (auto v = getInput<double>("hand_trajectory_speed"); v) {
    params.emplace_back("hand_trajectory_speed", v.value());
  }

  // ── DemoTask-only ────────────────────────────────────────────────────────
  if (is_task) {
    if (auto v = getInput<double>("trajectory_angular_speed"); v) {
      params.emplace_back("trajectory_angular_speed", v.value());
    }
    if (auto s = getInput<std::string>("kp_translation"); s && !s->empty()) {
      auto vals = ParseCsvList<double>(s.value());
      if (vals.size() == 3) {
        params.emplace_back("kp_translation", vals);
      } else {
        RCLCPP_WARN(logger(), "kp_translation must be 3 values (got %zu)", vals.size());
      }
    }
    if (auto s = getInput<std::string>("kp_rotation"); s && !s->empty()) {
      auto vals = ParseCsvList<double>(s.value());
      if (vals.size() == 3) {
        params.emplace_back("kp_rotation", vals);
      } else {
        RCLCPP_WARN(logger(), "kp_rotation must be 3 values (got %zu)", vals.size());
      }
    }
    if (auto v = getInput<double>("damping"); v) {
      params.emplace_back("damping", v.value());
    }
    if (auto v = getInput<double>("null_kp"); v) {
      params.emplace_back("null_kp", v.value());
    }
    if (auto v = getInput<bool>("enable_null_space"); v) {
      params.emplace_back("enable_null_space", v.value());
    }
    if (auto v = getInput<bool>("control_6dof"); v) {
      params.emplace_back("control_6dof", v.value());
    }
  }

  // ── DemoJoint / DemoTask grasp detection ─────────────────────────────────
  if (is_joint || is_task) {
    if (auto v = getInput<double>("grasp_contact_threshold"); v) {
      params.emplace_back("grasp_contact_threshold", v.value());
    }
    if (auto v = getInput<double>("grasp_force_threshold"); v) {
      params.emplace_back("grasp_force_threshold", v.value());
    }
    if (auto v = getInput<int>("grasp_min_fingertips"); v) {
      params.emplace_back("grasp_min_fingertips", static_cast<int64_t>(v.value()));
    }
  }

  // ── DemoWbc ──────────────────────────────────────────────────────────────
  if (is_wbc) {
    if (auto v = getInput<double>("se3_weight"); v) {
      params.emplace_back("se3_weight", v.value());
    }
    if (auto v = getInput<double>("force_weight"); v) {
      params.emplace_back("force_weight", v.value());
    }
    if (auto v = getInput<double>("posture_weight"); v) {
      params.emplace_back("posture_weight", v.value());
    }
    if (auto v = getInput<bool>("mpc_enable"); v) {
      params.emplace_back("mpc_enable", v.value());
    }
    if (auto v = getInput<double>("riccati_gain_scale"); v) {
      params.emplace_back("riccati_gain_scale", v.value());
    }
  }

  // ── Force-PI grasp command (one-shot, separate srv) — validated here so an
  // invalid value fails before any service call ────────────────────────────
  grasp_command_ = getInput<int>("grasp_command").value_or(0);
  grasp_force_ = getInput<double>("grasp_target_force").value_or(2.0);
  if (grasp_command_ != 0 &&
      grasp_command_ != static_cast<int>(rtc_msgs::srv::GraspCommand::Request::GRASP) &&
      grasp_command_ != static_cast<int>(rtc_msgs::srv::GraspCommand::Request::RELEASE)) {
    RCLCPP_ERROR(logger(), "grasp_command must be 1 (GRASP) or 2 (RELEASE), got %d",
                 grasp_command_);
    return false;
  }
  return true;
}

BT::NodeStatus SetGains::onStart() {
  if (!BuildParams()) {
    return BT::NodeStatus::FAILURE;
  }
  stage_sent_ = false;
  stage_start_ = std::chrono::steady_clock::now();
  last_err_.clear();

  // Choose the first stage: parameters (if any) then grasp; nothing → SUCCESS.
  if (!params_.empty()) {
    stage_ = Stage::kParams;
  } else if (grasp_command_ != 0) {
    stage_ = Stage::kGrasp;
  } else {
    return BT::NodeStatus::SUCCESS;  // no ports set → no-op
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetGains::onRunning() {
  // stage_start_ is re-stamped at send, so each budget times only its own wait.
  const double budget = stage_sent_ ? kSrvTimeoutS : kReadyTimeoutS;
  if (ElapsedSeconds(stage_start_) > budget) {
    RCLCPP_ERROR(logger(), "timeout (%.1fs) waiting for %s in %s stage: %s", budget,
                 stage_sent_ ? "response" : "service discovery",
                 stage_ == Stage::kParams ? "set_parameters" : "grasp_command",
                 last_err_.empty() ? "no response" : last_err_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  switch (stage_) {
    case Stage::kParams: {
      if (!stage_sent_) {
        param_future_ = bridge_->SetActiveControllerGainsAsync(params_, last_err_);
        if (param_future_.valid()) {
          stage_sent_ = true;
          stage_start_ = std::chrono::steady_clock::now();  // response clock starts at send
        }
        return BT::NodeStatus::RUNNING;  // retry next tick if not ready
      }
      if (param_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
      }
      // A future can become ready with a broken promise if the active
      // controller changed and RewireControllerTopics dropped the client that
      // owned this pending request. .get() then throws std::future_error;
      // surface it as FAILURE instead of letting it escape the tick timer
      // callback (which would propagate through rclcpp::spin and crash).
      rcl_interfaces::msg::SetParametersResult result;
      try {
        result = param_future_.get();
      } catch (const std::future_error& e) {
        RCLCPP_ERROR(logger(), "set_parameters future broken (controller rebound?): %s", e.what());
        return BT::NodeStatus::FAILURE;
      }
      if (!result.successful) {
        RCLCPP_ERROR(logger(), "set_parameters_atomically rejected: %s", result.reason.c_str());
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(logger(), "%s: %zu parameter(s) updated", bridge_->GetActiveController().c_str(),
                  params_.size());
      // Advance to the grasp stage, or finish if none requested.
      if (grasp_command_ == 0) {
        return BT::NodeStatus::SUCCESS;
      }
      stage_ = Stage::kGrasp;
      stage_sent_ = false;
      stage_start_ = std::chrono::steady_clock::now();
      last_err_.clear();
      return BT::NodeStatus::RUNNING;
    }
    case Stage::kGrasp: {
      if (!stage_sent_) {
        grasp_future_ = bridge_->SendGraspCommandAsync(static_cast<uint8_t>(grasp_command_),
                                                       grasp_force_, last_err_);
        if (grasp_future_.valid()) {
          stage_sent_ = true;
          stage_start_ = std::chrono::steady_clock::now();  // response clock starts at send
        }
        return BT::NodeStatus::RUNNING;  // retry next tick if not ready
      }
      if (grasp_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
      }
      // See the set_parameters stage: guard against a broken promise from a
      // client rebind so the exception cannot escape the tick timer callback.
      rtc_msgs::srv::GraspCommand::Response::SharedPtr resp;
      try {
        resp = grasp_future_.get();
      } catch (const std::future_error& e) {
        RCLCPP_ERROR(logger(), "grasp_command future broken (controller rebound?): %s", e.what());
        return BT::NodeStatus::FAILURE;
      }
      if (!resp->ok) {
        RCLCPP_ERROR(logger(), "grasp_command srv failed: %s", resp->message.c_str());
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(logger(), "grasp_command %s: %s", grasp_command_ == 1 ? "GRASP" : "RELEASE",
                  resp->message.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    case Stage::kDone:
      break;
  }
  return BT::NodeStatus::SUCCESS;
}

void SetGains::onHalted() {
  // Unlike SwitchController::onHalted this does not cancel the in-flight
  // param/grasp request: the parameter client is an AsyncParametersClient with
  // no request-id cancel API, and a rebind-broken future is already handled
  // defensively at the .get() sites above (→ FAILURE, no throw). A halted
  // request simply resolves and is discarded, harmless under the single-thread
  // executor. Adding a Cancel* pair would be needed only if that model changes.
  RCLCPP_INFO(logger(), "halted (stage=%s)",
              stage_ == Stage::kParams  ? "set_parameters"
              : stage_ == Stage::kGrasp ? "grasp_command"
                                        : "done");
}

}  // namespace rtc_bt
