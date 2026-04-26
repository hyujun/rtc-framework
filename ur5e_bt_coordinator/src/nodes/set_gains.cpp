#include "ur5e_bt_coordinator/action_nodes/set_gains.hpp"
#include "ur5e_bt_coordinator/bt_logging.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtc_msgs/srv/grasp_command.hpp>

#include <string>
#include <vector>

namespace rtc_bt {

namespace {
auto logger() { return ::rtc_bt::logging::ActionLogger("set_gains"); }

// Normalize controller name for comparison: strip underscores and lowercase.
// e.g. "demo_task_controller" and "DemoTaskController" both become
// "demotaskcontroller".
std::string NormalizeName(const std::string &s) {
  std::string r;
  r.reserve(s.size());
  for (char c : s) {
    if (c != '_') {
      r.push_back(
          static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
  }
  return r;
}

constexpr double kSrvTimeoutS = 2.0;
} // namespace

SetGains::SetGains(const std::string &name, const BT::NodeConfig &config,
                   std::shared_ptr<BtRosBridge> bridge)
    : BT::SyncActionNode(name, config), bridge_(std::move(bridge)) {}

BT::PortsList SetGains::providedPorts() {
  return {
      // ── Common ────────────────────────────────────────────────────────────
      BT::InputPort<double>("trajectory_speed",
                            "[m/s] DemoTask: trajectory_speed; "
                            "DemoJoint: robot_trajectory_speed; "
                            "DemoWbc: arm_trajectory_speed"),
      BT::InputPort<double>("trajectory_angular_speed",
                            "[rad/s] DemoTask only"),
      BT::InputPort<double>("hand_trajectory_speed", "[rad/s] All demos"),

      // ── DemoTask CLIK gains ───────────────────────────────────────────────
      BT::InputPort<std::string>("kp_translation", "", "e.g. \"5.0,5.0,5.0\""),
      BT::InputPort<std::string>("kp_rotation", "", "e.g. \"3.0,3.0,3.0\""),
      BT::InputPort<double>("damping", "Damped pseudoinverse lambda"),
      BT::InputPort<double>("null_kp", "Null-space P gain"),
      BT::InputPort<bool>("enable_null_space", "Enable null-space task"),
      BT::InputPort<bool>("control_6dof", "false=3-DOF, true=6-DOF"),

      // ── DemoJoint / DemoTask grasp detection ──────────────────────────────
      BT::InputPort<double>("grasp_contact_threshold",
                            "Contact probability threshold [0,1]"),
      BT::InputPort<double>("grasp_force_threshold",
                            "Force magnitude threshold [N]"),
      BT::InputPort<int>("grasp_min_fingertips",
                         "Min fingertips with contact for grasp"),

      // ── DemoWbc ───────────────────────────────────────────────────────────
      BT::InputPort<double>("se3_weight", "TSID SE3Task weight"),
      BT::InputPort<double>("force_weight", "TSID ForceTask weight"),
      BT::InputPort<double>("posture_weight", "TSID PostureTask weight"),
      BT::InputPort<bool>("mpc_enable", "MPC output gate"),
      BT::InputPort<double>("riccati_gain_scale",
                            "Riccati feedback scale [0,1]"),

      // ── Force-PI grasp (one-shot, separate srv) ───────────────────────────
      BT::InputPort<int>("grasp_command", 0,
                         "0=none, 1=grasp, 2=release (Force-PI)"),
      BT::InputPort<double>("grasp_target_force", 2.0, "Target grip force [N]"),
  };
}

BT::NodeStatus SetGains::tick() {
  const auto active = NormalizeName(bridge_->GetActiveController());
  const bool is_joint = (active == "demojointcontroller");
  const bool is_task = (active == "demotaskcontroller");
  const bool is_wbc = (active == "demowbccontroller");
  if (!is_joint && !is_task && !is_wbc) {
    RCLCPP_WARN(logger(), "unknown active controller: '%s' — skipping",
                bridge_->GetActiveController().c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::vector<rclcpp::Parameter> params;
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
        RCLCPP_WARN(logger(), "kp_translation must be 3 values (got %zu)",
                    vals.size());
      }
    }
    if (auto s = getInput<std::string>("kp_rotation"); s && !s->empty()) {
      auto vals = ParseCsvList<double>(s.value());
      if (vals.size() == 3) {
        params.emplace_back("kp_rotation", vals);
      } else {
        RCLCPP_WARN(logger(), "kp_rotation must be 3 values (got %zu)",
                    vals.size());
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
      params.emplace_back("grasp_min_fingertips",
                          static_cast<int64_t>(v.value()));
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

  // ── Set parameters (skipped when no port set) ────────────────────────────
  if (!params.empty()) {
    std::string msg;
    if (!bridge_->SetActiveControllerGains(params, kSrvTimeoutS, msg)) {
      RCLCPP_ERROR(logger(), "set_parameters_atomically failed: %s",
                   msg.c_str());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(logger(), "%s: %zu parameter(s) updated",
                bridge_->GetActiveController().c_str(), params.size());
  }

  // ── Force-PI grasp command (one-shot, separate srv) ─────────────────────
  if (auto cmd = getInput<int>("grasp_command"); cmd && cmd.value() != 0) {
    const auto cmd_val = cmd.value();
    if (cmd_val !=
            static_cast<int>(rtc_msgs::srv::GraspCommand::Request::GRASP) &&
        cmd_val !=
            static_cast<int>(rtc_msgs::srv::GraspCommand::Request::RELEASE)) {
      RCLCPP_ERROR(logger(),
                   "grasp_command must be 1 (GRASP) or 2 (RELEASE), got %d",
                   cmd_val);
      return BT::NodeStatus::FAILURE;
    }
    const double force = getInput<double>("grasp_target_force").value_or(2.0);
    std::string msg;
    if (!bridge_->SendGraspCommand(static_cast<uint8_t>(cmd_val), force,
                                   kSrvTimeoutS, msg)) {
      RCLCPP_ERROR(logger(), "grasp_command srv failed: %s", msg.c_str());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(logger(), "grasp_command %s: %s",
                cmd_val == 1 ? "GRASP" : "RELEASE", msg.c_str());
  }

  return BT::NodeStatus::SUCCESS;
}

} // namespace rtc_bt
