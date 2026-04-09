#include "ur5e_bt_coordinator/bt_coordinator_node.hpp"

// Action nodes
#include "ur5e_bt_coordinator/action_nodes/compute_offset_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/compute_sweep_trajectory.hpp"
#include "ur5e_bt_coordinator/action_nodes/compute_tilt_sequence.hpp"
#include "ur5e_bt_coordinator/action_nodes/get_current_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/grasp_control.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_to_joints.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_to_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/set_gains.hpp"
#include "ur5e_bt_coordinator/action_nodes/switch_controller.hpp"
#include "ur5e_bt_coordinator/action_nodes/track_trajectory.hpp"
#include "ur5e_bt_coordinator/action_nodes/wait_duration.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_finger.hpp"
#include "ur5e_bt_coordinator/action_nodes/flex_extend_finger.hpp"
#include "ur5e_bt_coordinator/action_nodes/set_hand_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/ur5e_hold_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_opposition.hpp"
#include "ur5e_bt_coordinator/action_nodes/trigger_shape_estimation.hpp"
#include "ur5e_bt_coordinator/action_nodes/wait_shape_result.hpp"

// Condition nodes
#include "ur5e_bt_coordinator/condition_nodes/is_force_above.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_grasp_phase.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_grasped.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_object_detected.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_vision_target_ready.hpp"
#include "ur5e_bt_coordinator/condition_nodes/check_shape_type.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <filesystem>
#include <functional>
#include <sstream>

namespace rtc_bt {

BtCoordinatorNode::BtCoordinatorNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("bt_coordinator", options)
{
  DeclareParameters();
}

void BtCoordinatorNode::Initialize()
{
  // Create ROS bridge — requires shared_from_this(), so must be called
  // after make_shared<BtCoordinatorNode>().
  bridge_ = std::make_shared<BtRosBridge>(shared_from_this());

  // Load pose overrides from ROS2 parameters (hand_pose.*, arm_pose.*)
  bridge_->LoadPoseOverrides(shared_from_this());

  RegisterBtNodes();
  LoadTree();

  // Inject blackboard variables from bb.* parameters
  InitializeBlackboard();

  // Groot2 monitoring
#ifdef BT_GROOT2_AVAILABLE
  if (groot2_port_ > 0) {
    try {
      groot2_publisher_ = std::make_unique<BT::Groot2Publisher>(*tree_, groot2_port_);
      RCLCPP_INFO(get_logger(), "[BtCoordinator] Groot2 publisher on port %d", groot2_port_);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "[BtCoordinator] Groot2 init failed: %s", e.what());
    }
  }
#else
  if (groot2_port_ > 0) {
    RCLCPP_WARN(get_logger(),
                "[BtCoordinator] groot2_port=%d requested but Groot2 not available "
                "(BehaviorTree.CPP compiled without ZMQ support)", groot2_port_);
  }
#endif

  // Tick timer (only if not in step mode)
  if (!step_mode_) {
    const auto period = std::chrono::duration<double>(1.0 / tick_rate_hz_);
    tick_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&BtCoordinatorNode::TickCallback, this));
  } else {
    RCLCPP_INFO(get_logger(), "[BtCoordinator] Step mode enabled — use ~/step service to tick");
  }

  // Step mode service (always available)
  step_service_ = create_service<std_srvs::srv::Trigger>(
      "~/step",
      std::bind(&BtCoordinatorNode::StepCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  // Parameter change callback for runtime tree switching and pause
  param_callback_ = add_on_set_parameters_callback(
      std::bind(&BtCoordinatorNode::OnParameterChange, this, std::placeholders::_1));

  // Watchdog timer
  if (watchdog_interval_s_ > 0.0) {
    watchdog_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(watchdog_interval_s_)),
        std::bind(&BtCoordinatorNode::WatchdogCheck, this));
  }

  RCLCPP_INFO(get_logger(),
              "[BtCoordinator] Tree loaded: %s (tick %.1f Hz, %s)",
              tree_file_.c_str(), tick_rate_hz_,
              step_mode_ ? "step mode" : "auto mode");
}

void BtCoordinatorNode::DeclareParameters()
{
  // When automatically_declare_parameters_from_overrides is true (for dynamic
  // params like bb.*, hand_pose.*, arm_pose.*), parameters from --params-file
  // are auto-declared before this function runs. Use get_parameter() for those
  // already declared, declare_parameter() otherwise.
  auto safe_declare = [this](const std::string& name, auto default_value) {
    using T = decltype(default_value);
    if (has_parameter(name)) {
      return get_parameter(name).get_value<T>();
    }
    return declare_parameter<T>(name, default_value);
  };

  tree_file_ = safe_declare("tree_file", std::string("pick_and_place.xml"));
  tick_rate_hz_ = safe_declare("tick_rate_hz", 20.0);
  repeat_ = safe_declare("repeat", false);
  repeat_delay_s_ = safe_declare("repeat_delay_s", 1.0);
  paused_ = safe_declare("paused", false);
  step_mode_ = safe_declare("step_mode", false);
  groot2_port_ = safe_declare("groot2_port", 0);
  watchdog_timeout_s_ = safe_declare("watchdog_timeout_s", 2.0);
  watchdog_interval_s_ = safe_declare("watchdog_interval_s", 5.0);
}

void BtCoordinatorNode::RegisterBtNodes()
{
  auto bridge = bridge_;

  // ── Action nodes ──────────────────────────────────────────────────────
  factory_.registerNodeType<MoveToPose>("MoveToPose", bridge);
  factory_.registerNodeType<MoveToJoints>("MoveToJoints", bridge);
  factory_.registerNodeType<GraspControl>("GraspControl", bridge);
  factory_.registerNodeType<TrackTrajectory>("TrackTrajectory", bridge);
  factory_.registerNodeType<SetGains>("SetGains", bridge);
  factory_.registerNodeType<SwitchController>("SwitchController", bridge);
  factory_.registerNodeType<ComputeOffsetPose>("ComputeOffsetPose");
  factory_.registerNodeType<ComputeSweepTrajectory>("ComputeSweepTrajectory");
  factory_.registerNodeType<ComputeTiltSequence>("ComputeTiltSequence");
  factory_.registerNodeType<GetCurrentPose>("GetCurrentPose", bridge);
  factory_.registerNodeType<WaitDuration>("WaitDuration");

  // ── Hand demo nodes ─────────────────────────────────────────────────
  factory_.registerNodeType<MoveFinger>("MoveFinger", bridge);
  factory_.registerNodeType<FlexExtendFinger>("FlexExtendFinger", bridge);
  factory_.registerNodeType<SetHandPose>("SetHandPose", bridge);
  factory_.registerNodeType<UR5eHoldPose>("UR5eHoldPose", bridge);
  factory_.registerNodeType<MoveOpposition>("MoveOpposition", bridge);

  // ── Shape estimation nodes ──────────────────────────────────────────
  factory_.registerNodeType<TriggerShapeEstimation>("TriggerShapeEstimation", bridge);
  factory_.registerNodeType<WaitShapeResult>("WaitShapeResult", bridge);

  // ── Condition nodes ───────────────────────────────────────────────────
  factory_.registerNodeType<IsForceAbove>("IsForceAbove", bridge);
  factory_.registerNodeType<IsGraspPhase>("IsGraspPhase", bridge);
  factory_.registerNodeType<IsGrasped>("IsGrasped", bridge);
  factory_.registerNodeType<IsObjectDetected>("IsObjectDetected", bridge);
  factory_.registerNodeType<IsVisionTargetReady>("IsVisionTargetReady", bridge);
  factory_.registerNodeType<CheckShapeType>("CheckShapeType", bridge);
}

void BtCoordinatorNode::LoadTree()
{
  std::filesystem::path tree_path;

  // Support absolute paths
  if (std::filesystem::path(tree_file_).is_absolute()) {
    tree_path = tree_file_;
  } else {
    std::string pkg_share =
        ament_index_cpp::get_package_share_directory("ur5e_bt_coordinator");
    tree_path = std::filesystem::path(pkg_share) / "trees" / tree_file_;
  }

  if (!std::filesystem::exists(tree_path)) {
    RCLCPP_FATAL(get_logger(), "[BtCoordinator] Tree file not found: %s", tree_path.c_str());
    throw std::runtime_error("BT tree file not found: " + tree_path.string());
  }

  RCLCPP_DEBUG(get_logger(), "[BtCoordinator] Loading tree from: %s", tree_path.c_str());

  tree_ = std::make_unique<BT::Tree>(
      factory_.createTreeFromFile(tree_path.string()));
}

void BtCoordinatorNode::InitializeBlackboard()
{
  if (!tree_) return;

  auto bb = tree_->rootBlackboard();
  auto result = list_parameters({"bb"}, 1);

  for (const auto& param_name : result.names) {
    const std::string prefix = "bb.";
    if (param_name.size() <= prefix.size()) continue;
    std::string key = param_name.substr(prefix.size());

    auto param = get_parameter(param_name);
    switch (param.get_type()) {
      case rclcpp::ParameterType::PARAMETER_STRING:
        bb->set(key, param.as_string());
        RCLCPP_INFO(get_logger(), "[Blackboard] %s = \"%s\" (string)",
                    key.c_str(), param.as_string().c_str());
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        bb->set(key, param.as_double());
        RCLCPP_INFO(get_logger(), "[Blackboard] %s = %f (double)",
                    key.c_str(), param.as_double());
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        bb->set(key, static_cast<int>(param.as_int()));
        RCLCPP_INFO(get_logger(), "[Blackboard] %s = %ld (int)",
                    key.c_str(), param.as_int());
        break;
      case rclcpp::ParameterType::PARAMETER_BOOL:
        bb->set(key, param.as_bool());
        RCLCPP_INFO(get_logger(), "[Blackboard] %s = %s (bool)",
                    key.c_str(), param.as_bool() ? "true" : "false");
        break;
      default:
        RCLCPP_WARN(get_logger(), "[Blackboard] %s: unsupported type, skipped",
                    key.c_str());
        break;
    }
  }
}

void BtCoordinatorNode::TickCallback()
{
  if (!tree_) return;

  // Paused check
  if (paused_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                          "[BtCoordinator] Paused (set 'paused' param to false to resume)");
    return;
  }

  if (bridge_->IsEstopped()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                          "[BtCoordinator] E-STOP active, tree paused");
    return;
  }

  auto status = tree_->tickOnce();
  RCLCPP_DEBUG(get_logger(), "[BtCoordinator] tick -> %s",
               BT::toStr(status).c_str());

  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(get_logger(), "[BtCoordinator] Tree completed: SUCCESS");
    if (repeat_) {
      RCLCPP_INFO(get_logger(), "[BtCoordinator] Restarting tree in %.1f s...",
                  repeat_delay_s_);
      tick_timer_->cancel();
      repeat_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(repeat_delay_s_)),
          [this]() {
            repeat_timer_->cancel();
            tree_->haltTree();
            tree_->rootBlackboard()->unset("object_pose");
            RCLCPP_INFO(get_logger(), "[BtCoordinator] Tree restarted");
            tick_timer_->reset();
          });
    } else {
      tick_timer_->cancel();
    }
  } else if (status == BT::NodeStatus::FAILURE) {
    RCLCPP_WARN(get_logger(), "[BtCoordinator] Tree completed: FAILURE");
    LogFailureDiagnosis();
    tick_timer_->cancel();
  }
}

void BtCoordinatorNode::LogFailureDiagnosis()
{
  if (!tree_) return;

  RCLCPP_WARN(get_logger(), "──── Failure Diagnosis ────");

  // Walk the tree to find failed nodes
  std::function<void(const BT::TreeNode*, int)> visit =
      [&](const BT::TreeNode* node, int depth) {
    if (!node) return;

    auto status = node->status();
    std::string indent(depth * 2, ' ');

    if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_WARN(get_logger(), "%s[FAILED] %s (type: %s)",
                  indent.c_str(),
                  node->name().c_str(),
                  node->registrationName().c_str());
    }

    // Visit children if it's a control/decorator node
    if (auto control = dynamic_cast<const BT::ControlNode*>(node)) {
      for (std::size_t i = 0; i < control->childrenCount(); ++i) {
        visit(control->child(i), depth + 1);
      }
    } else if (auto decorator = dynamic_cast<const BT::DecoratorNode*>(node)) {
      visit(decorator->child(), depth + 1);
    }
  };

  for (auto& subtree : tree_->subtrees) {
    for (auto& node : subtree->nodes) {
      if (node->status() == BT::NodeStatus::FAILURE && !node->name().empty()) {
        // Only log leaf/action/condition failures at root pass
      }
    }
  }

  // Comprehensive walk from root
  visit(tree_->rootNode(), 0);

  RCLCPP_WARN(get_logger(), "──── End Diagnosis ────");
}

void BtCoordinatorNode::WatchdogCheck()
{
  auto health = bridge_->GetTopicHealth(watchdog_timeout_s_);

  bool any_critical_down = false;
  for (const auto& h : health) {
    if (!h.received) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                            "[Watchdog] %s: no messages received yet", h.name.c_str());
      any_critical_down = true;
    } else if (!h.healthy) {
      RCLCPP_WARN(get_logger(),
                  "[Watchdog] %s: stale (%.1fs since last message, timeout=%.1fs)",
                  h.name.c_str(), h.seconds_since_last, watchdog_timeout_s_);
      any_critical_down = true;
    } else {
      RCLCPP_DEBUG(get_logger(), "[Watchdog] %s: healthy (%.2fs ago)",
                   h.name.c_str(), h.seconds_since_last);
    }
  }

  if (!any_critical_down) {
    RCLCPP_DEBUG(get_logger(), "[Watchdog] all topics healthy");
  }
}

rcl_interfaces::msg::SetParametersResult BtCoordinatorNode::OnParameterChange(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& param : params) {
    if (param.get_name() == "tree_file") {
      // Runtime tree switching
      std::string new_file = param.as_string();
      RCLCPP_INFO(get_logger(), "[BtCoordinator] Switching tree: %s → %s",
                  tree_file_.c_str(), new_file.c_str());

      try {
        // Stop ticking
        if (tick_timer_) tick_timer_->cancel();
        if (repeat_timer_) repeat_timer_->cancel();

        // Halt current tree
        if (tree_) {
          tree_->haltTree();
        }

        // Load new tree
        tree_file_ = new_file;
        LoadTree();
        InitializeBlackboard();

#ifdef BT_GROOT2_AVAILABLE
        // Re-create Groot2 publisher for new tree
        if (groot2_port_ > 0) {
          groot2_publisher_.reset();
          try {
            groot2_publisher_ = std::make_unique<BT::Groot2Publisher>(*tree_, groot2_port_);
          } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "[BtCoordinator] Groot2 re-init failed: %s", e.what());
          }
        }
#endif

        // Resume ticking
        if (!step_mode_ && tick_timer_) {
          tick_timer_->reset();
        }

        RCLCPP_INFO(get_logger(), "[BtCoordinator] Tree switched to: %s", new_file.c_str());
      } catch (const std::exception& e) {
        result.successful = false;
        result.reason = std::string("Failed to load tree: ") + e.what();
        RCLCPP_ERROR(get_logger(), "[BtCoordinator] Tree switch failed: %s", e.what());
      }
    } else if (param.get_name() == "paused") {
      paused_ = param.as_bool();
      RCLCPP_INFO(get_logger(), "[BtCoordinator] %s",
                  paused_ ? "Paused" : "Resumed");
    } else if (param.get_name() == "step_mode") {
      step_mode_ = param.as_bool();
      if (step_mode_) {
        if (tick_timer_) tick_timer_->cancel();
        RCLCPP_INFO(get_logger(), "[BtCoordinator] Step mode ON — use ~/step service");
      } else {
        if (tick_timer_) tick_timer_->reset();
        RCLCPP_INFO(get_logger(), "[BtCoordinator] Step mode OFF — auto ticking resumed");
      }
    } else if (param.get_name() == "repeat") {
      repeat_ = param.as_bool();
    } else if (param.get_name() == "repeat_delay_s") {
      repeat_delay_s_ = param.as_double();
    } else if (param.get_name() == "watchdog_timeout_s") {
      watchdog_timeout_s_ = param.as_double();
    }
  }

  return result;
}

void BtCoordinatorNode::StepCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!tree_) {
    RCLCPP_ERROR(get_logger(), "[Step] no tree loaded");
    response->success = false;
    response->message = "No tree loaded";
    return;
  }

  if (bridge_->IsEstopped()) {
    RCLCPP_WARN(get_logger(), "[Step] E-STOP active, cannot tick");
    response->success = false;
    response->message = "E-STOP active";
    return;
  }

  auto status = tree_->tickOnce();

  std::string status_str;
  switch (status) {
    case BT::NodeStatus::SUCCESS: status_str = "SUCCESS"; break;
    case BT::NodeStatus::FAILURE: status_str = "FAILURE"; break;
    case BT::NodeStatus::RUNNING: status_str = "RUNNING"; break;
    default: status_str = "IDLE"; break;
  }

  response->success = (status != BT::NodeStatus::FAILURE);
  response->message = "Tree status: " + status_str;

  RCLCPP_INFO(get_logger(), "[Step] Tick result: %s", status_str.c_str());

  if (status == BT::NodeStatus::FAILURE) {
    LogFailureDiagnosis();
  }
}

}  // namespace rtc_bt
